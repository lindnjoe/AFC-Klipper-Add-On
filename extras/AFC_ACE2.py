# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC unit driver for the Anycubic ACE PRO 2 filament changer.
#
# The Pro 2 speaks a completely different wire protocol from the original ACE
# Pro: a binary, sequence-numbered, Protocol-Buffers-style framing (NOT the V1
# JSON protocol). That wire protocol — the framing, Kermit-style CRC16, the
# command opcodes, the per-command protobuf field layouts, and the response
# decoders — is adapted from the multiACE project:
#
#     multiACE — https://github.com/decay71/multiace (GPL-3.0)
#
# Everything else (lane load/unload, feed assist, RFID->Spoolman, dryer, the
# diagnostics) is inherited unchanged from the V1 AFC_ACE unit: this class just
# swaps the serial transport. multiACE's protocol maps the same V1 method names
# and returns V1-shaped result dicts, so the V1 unit code runs as-is.

from __future__ import annotations

import traceback
from configparser import Error as error

import logging
import struct

from typing import Any, Dict, List, Optional, Tuple, Union, TYPE_CHECKING

try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_ACE import (
    afcACE, ACEConnection, ACESerialError, ACETimeoutError, REQUEST_TIMEOUT,
    )
except: raise error(ERROR_STR.format(import_lib="AFC_ACE", trace=traceback.format_exc()))

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from reactor import SelectReactor as Reactor

_logger = logging.getLogger("AFC_ACE2")


# ── V2 wire protocol (adapted from multiACE, decay71/multiace, GPL-3.0) ──────
# Frame: PREAMBLE(2) | flags(1) | seq(2 LE) | cmd(1) | payLen(1) | payload |
#        CRC16(2 LE) | END_MARKER(1)
PREAMBLE = b'\xFF\xAA'
END_MARKER = 0xFE
FLAG_REQUEST = 0x00
FLAG_RESPONSE = 0x80
# ACE2 firmware encoder-per-mm scale: the feed-check expects
# encoder_reading = commanded_mm * 1.2342. Used to validate SET_FEED_CHECK
# params and report slip.
ACE2_ENCODER_SCALE = 1.2342
HEADER_LEN = 7
TRAILER_LEN = 3
MIN_FRAME_LEN = HEADER_LEN + TRAILER_LEN
MAX_PAYLOAD_LEN = 100


class Cmd:
    """ACE2 serial-protocol command opcodes.

    Each attribute is the integer command byte sent to (or received from) the
    ACE2 controller in a protocol frame. Grouped roughly by function: device
    discovery/identity, status/info queries, feed/rollback motion, drying, RFID,
    and miscellaneous hardware controls (valve, fan, temperature).
    """
    DISCOVER_DEVICE = 0
    ASSIGN_DEVICE_ID = 1
    IAP_VERSION = 5
    GET_STATUS = 6
    GET_INFO = 7
    FEED_OR_ROLLBACK = 8
    STOP_FEED_OR_ROLLBACK = 9
    UPDATE_SPEED = 10
    DRYING = 11
    SET_DRY_TEMP = 12
    GET_FILAMENT_INFO = 13
    SET_RFID_ENABLE = 14
    GET_MATERIAL_INFO = 16
    SET_MATERIAL_NAME = 18
    SET_FEED_CHECK = 19
    GET_TEMP = 64
    SET_DRY_POWER = 65
    SET_VALVE = 66
    FILAMENT_IDENTIFY = 68
    RFID_TEST = 69
    SET_FAN = 71
    # Opcode 73 (GET_SENSOR_STATE) returns the 17-channel filament-sensor
    # bitmask; kept under both names for clarity.
    GET_KEY_STATE = 73
    GET_SENSOR_STATE = 73
    GET_FEED_INFO = 76
    # Custom firmware patch (ace2_rfid): expose the MFRC522 registers so the host
    # runs the full OpenRFID stack (Bambu MIFARE, Anycubic NTAG, raw UID). Both
    # take a single packed-u32 field 1; reg-read replies field 1 = value.
    #   reg-read  arg = (slot<<16) | reg              -> {1: value}
    #   reg-write arg = (slot<<16) | (reg<<8) | value -> {} (ack)
    MFRC522_REG_READ = 0x50
    MFRC522_REG_WRITE = 0x51
    # ace2_rfid v2: host-owned reader power so encrypted (Bambu MIFARE) reads
    # aren't interrupted by the firmware's own identify power-gating.
    #   reader-power arg = (reader_index<<16) | on   (on=1 power, 0 off) -> {} ack
    # reader_index is the physical slot >> 1 (2 readers cover 4 slots).
    MFRC522_READER_POWER = 0x52


SLOT_STATES = {
    0: 'ready', 1: 'feeding', 2: 'rollback', 3: 'assisting',
    4: 'rollback_assisting', 5: 'preloading', 6: 'upgrading',
    129: 'feed_error', 130: 'rollback_error', 131: 'assist_error',
    132: 'preload_error', 133: 'stuck_error', 134: 'tangled_error',
    135: 'motor_error',
}
FILAMENT_STATES = {0: 'empty', 1: 'unknown', 2: 'identified', 3: 'identifying'}

# GET_SENSOR_STATE (opcode 73) 17-channel bitmask layout: each of the 4 slots
# has 4 optical sensors, then a single shared buffer-feed sensor at bit 16.
# INSERT (4*slot+0) toggles when a spool is inserted. BUF_RST (4*slot+2) is ON
# at rest and BUF_BACK (4*slot+3) is ON at the back limit — the slot's internal
# buffer-position switches (the ACE's per-lane buffer state, its Turtleneck-style
# advance/trailing equivalent). The shared BUF_FEED (bit 16) pulses while the
# buffer moves between the two.
SENSORS_PER_SLOT = 4
SENSOR_INSERT, SENSOR_EMPTY, SENSOR_BUF_RST, SENSOR_BUF_BACK = 0, 1, 2, 3
SENSOR_BUF_FEED_BIT = 16     # shared buffer-feed sensor (not per-slot)
SENSOR_BITMASK_WIDTH = 17
DRY_STATES = {0: 'stop', 1: 'starting', 2: 'keeping',
              3: 'stopping', 4: 'ptc_error', 5: 'ntc_error'}

FEED_MODE_FEED = 0
FEED_MODE_ROLLBACK = 1
FEED_MODE_ASSIST = 2
FEED_MODE_ROLLBACK_ASSIST = 3

# Opcodes whose response reports a status/error in protobuf field 1 (0 = ok,
# non-zero = error). For every OTHER opcode without a dedicated decoder, field 1
# is DATA, not a code — treating it as an error code would make send_command
# raise on a good ack (e.g. GET_SENSOR_STATE, whose field 1 is a 17-bit sensor
# bitmask, hence its own branch). The feed/rollback family genuinely returns
# error_2 here (the unit refusing a feed or an over-limit assist start), and
# _start_feed_assist / the load sequence depend on that raise — so only these
# keep field-1-as-error.
_ERROR_CODE_OPCODES = frozenset({
    Cmd.FEED_OR_ROLLBACK, Cmd.STOP_FEED_OR_ROLLBACK, Cmd.UPDATE_SPEED,
})


def crc16_kermit(data: Union[bytes, bytearray]) -> int:
    """
    Compute the Kermit-style CRC16 used by the ACE 2 Pro framing.

    :param data: Bytes-like sequence to checksum.
    :return: 16-bit CRC value as an integer.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


# ---- protobuf encode helpers ----
def pb_varint(value: int) -> bytes:
    """
    Encode an unsigned integer as a protobuf base-128 varint.

    :param value: Non-negative integer to encode.
    :return: Bytes containing the varint encoding.
    """
    r = bytearray()
    value = int(value)
    if value < 0:
        # A negative slot/length/speed would silently truncate to a bogus small
        # positive (7-bit mask), sending the unit a wrong motion parameter with
        # no diagnostic. Reject it instead.
        raise ValueError("pb_varint cannot encode negative value %d" % value)
    while value > 0x7F:
        r.append((value & 0x7F) | 0x80)
        value >>= 7
    r.append(value & 0x7F)
    return bytes(r)


def pb_uint32(field: int, value: int) -> bytes:
    """
    Encode a protobuf varint field (wire type 0) with a uint32 value.

    :param field: Protobuf field number.
    :param value: Integer value to encode.
    :return: Bytes containing the tag and varint-encoded value.
    """
    return pb_varint((field << 3) | 0) + pb_varint(int(value))


def pb_bool(field: int, value: Any) -> bytes:
    """
    Encode a protobuf varint field (wire type 0) with a boolean value.

    :param field: Protobuf field number.
    :param value: Truthy value encoded as 1, falsy as 0.
    :return: Bytes containing the tag and varint-encoded boolean.
    """
    return pb_varint((field << 3) | 0) + pb_varint(1 if value else 0)


def pb_string(field: int, text: Union[str, bytes]) -> bytes:
    """
    Encode a protobuf length-delimited field (wire type 2) with a UTF-8 string.

    :param field: Protobuf field number.
    :param text: str (UTF-8 encoded) or bytes-like value.
    :return: Bytes containing the tag, length varint, and the raw bytes.
    """
    data = text.encode('utf-8') if isinstance(text, str) else bytes(text)
    return pb_varint((field << 3) | 2) + pb_varint(len(data)) + data


# ---- protobuf decode helpers ----
def pb_decode_varint(data: Union[bytes, bytearray], pos: int) -> Tuple[int, int]:
    """
    Decode a single base-128 varint from a buffer at a given offset.

    :param data: Bytes-like buffer to read from.
    :param pos: Index at which to start decoding.
    :return: Tuple of (decoded integer, index past the varint).
    """
    result, shift = 0, 0
    while pos < len(data):
        b = data[pos]
        pos += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, pos
        shift += 7
    return result, pos


def pb_decode(data: Union[bytes, bytearray]) -> Dict[int, List[Tuple[int, Any]]]:
    """
    Decode a protobuf message into a mapping of field number to entries.

    Supports varint (0), 64-bit (1), length-delimited (2) and 32-bit (5) wire
    types; decoding stops on an unsupported wire type or truncated data.

    :param data: Raw protobuf-encoded bytes.
    :return: Dict mapping each field number to a list of ``(wire_type, value)``
             tuples in the order they appeared.
    """
    fields: Dict[int, Any] = {}
    pos = 0
    while pos < len(data):
        tag, pos = pb_decode_varint(data, pos)
        fnum, wtype = tag >> 3, tag & 7
        if wtype == 0:
            val, pos = pb_decode_varint(data, pos)
        elif wtype == 1:
            if pos + 8 > len(data):
                break
            val = struct.unpack_from('<d', data, pos)[0]
            pos += 8
        elif wtype == 2:
            ln, pos = pb_decode_varint(data, pos)
            val = bytes(data[pos:pos + ln])
            pos += ln
        elif wtype == 5:
            if pos + 4 > len(data):
                break
            val = struct.unpack_from('<f', data, pos)[0]
            pos += 4
        else:
            break
        fields.setdefault(fnum, []).append((wtype, val))
    return fields


def _fval(fields: Dict[int, List[Tuple[int, Any]]], num: int,
          default: Any = 0) -> Any:
    """
    Return the value of the first entry for a protobuf field.

    :param fields: Decoded fields mapping from :func:`pb_decode`.
    :param num: Field number to look up.
    :param default: Value returned when the field is absent.
    :return: The field's first value, or ``default`` if not present.
    """
    return fields.get(num, [(0, default)])[0][1]


def _fstr(fields: Dict[int, List[Tuple[int, Any]]], num: int,
          default: str = '') -> str:
    """
    Return a decoded string for the first entry of a length-delimited field.

    :param fields: Decoded fields mapping from :func:`pb_decode`.
    :param num: Field number to look up.
    :param default: Value returned when the field is not bytes.
    :return: UTF-8 decoded string, a hex string if decoding fails, or
             ``default`` when the field value is not bytes.
    """
    val = fields.get(num, [(2, b'')])[0][1]
    if isinstance(val, bytes):
        try:
            return val.decode('utf-8')
        except UnicodeDecodeError:
            return val.hex()
    return default


def dump_fields(fields: Dict[int, List[Tuple[int, Any]]]) -> Dict[int, Any]:
    """
    Render a pb_decode field map into a readable diagnostic mapping, for
    raw-dump commands. field number -> decoded entry (int, float, printable
    'string', or 'hex:..' for binary / nested data).

    :param fields: Decoded fields mapping from :func:`pb_decode`.
    :return: Dict of field number -> value (or list when a field repeats).
    """
    out = {}
    for fnum in sorted(fields):
        entries = []
        for wtype, val in fields[fnum]:
            if wtype == 0:
                entries.append(val)
            elif wtype in (1, 5):
                entries.append(round(float(val), 4))
            elif isinstance(val, (bytes, bytearray)):
                try:
                    s = val.decode('utf-8')
                    entries.append(repr(s) if s.isprintable() else 'hex:' + val.hex())
                except Exception:
                    entries.append('hex:' + val.hex())
            else:
                entries.append(val)
        out[fnum] = entries if len(entries) > 1 else entries[0]
    return out


def method_to_v2(method: str,
                 params: Optional[Dict[str, Any]]) -> Tuple[int, bytes]:
    """
    Map a V1 method name + params to a (cmd_opcode, protobuf_payload) tuple.

    Unknown methods fall back to a harmless ``GET_STATUS`` query.

    :param method: V1 method name (e.g. ``feed_filament``, ``get_status``).
    :param params: Optional dict of method parameters; treated as empty if None.
    :return: Tuple of (V2 command opcode, protobuf-encoded payload bytes).
    """
    params = params or {}
    if method == 'get_info':
        return Cmd.GET_INFO, b''
    if method == 'get_status':
        return Cmd.GET_STATUS, b''
    if method == 'discover_device':
        return Cmd.DISCOVER_DEVICE, b''
    if method == 'start_feed_assist':
        slot = int(params.get('index', 0))
        speed = int(params.get('speed', 10))
        return Cmd.FEED_OR_ROLLBACK, (
            pb_uint32(1, slot) + pb_uint32(2, speed)
            + pb_uint32(3, 0) + pb_uint32(4, FEED_MODE_ASSIST))
    if method == 'stop_feed_assist':
        return Cmd.STOP_FEED_OR_ROLLBACK, pb_uint32(1, int(params.get('index', 0)))
    if method == 'feed_filament':
        slot = int(params.get('index', 0))
        length = int(params.get('length', 0))
        speed = int(params.get('speed', 50))
        return Cmd.FEED_OR_ROLLBACK, (
            pb_uint32(1, slot) + pb_uint32(2, speed)
            + pb_uint32(3, length) + pb_uint32(4, FEED_MODE_FEED))
    if method == 'unwind_filament':
        slot = int(params.get('index', 0))
        length = int(params.get('length', 0))
        speed = int(params.get('speed', 50))
        return Cmd.FEED_OR_ROLLBACK, (
            pb_uint32(1, slot) + pb_uint32(2, speed)
            + pb_uint32(3, length) + pb_uint32(4, FEED_MODE_ROLLBACK))
    if method == 'stop_feed_filament':
        return Cmd.STOP_FEED_OR_ROLLBACK, pb_uint32(1, int(params.get('index', 0)))
    if method == 'update_feeding_speed':
        slot = int(params.get('index', 0))
        speed = int(params.get('speed', 50))
        return Cmd.UPDATE_SPEED, pb_uint32(1, slot) + pb_uint32(2, speed)
    if method == 'get_filament_info':
        return Cmd.GET_FILAMENT_INFO, pb_uint32(1, int(params.get('index', 0)))
    if method == 'drying':
        temp = int(params.get('temp', 50))
        duration = int(params.get('duration', 0))
        # The DRYING fan field (request[8], a "fan mode byte" in the firmware) is
        # an ON/OFF mode, NOT a speed/RPM/percentage. The host passes a fan_speed
        # for V1-ACE call compatibility (V1 ignores it); on the ACE2 map any
        # positive value to fan-on (1) and 0 to fan-off (0). This is NOT the
        # 0-100% scale used by the separate SET_FAN command (cmd 71).
        fan_on = 1 if int(params.get('fan_speed', 1)) > 0 else 0
        return Cmd.DRYING, (pb_uint32(1, temp) + pb_uint32(2, duration)
                            + pb_uint32(3, fan_on))
    if method == 'drying_stop':
        return Cmd.DRYING, pb_uint32(1, 0) + pb_uint32(2, 0)
    if method == 'set_fan_speed':
        speed = int(params.get('speed', 0))
        return Cmd.SET_FAN, (pb_uint32(1, speed)
                             + pb_bool(2, speed > 0) + pb_bool(3, speed > 0))
    if method == 'set_rfid_enable':
        slot = int(params.get('index', 0))
        enable = bool(params.get('enable', True))
        return Cmd.SET_RFID_ENABLE, pb_uint32(1, slot) + pb_bool(2, enable)
    if method == 'set_feed_check':
        # V2 SET_FEED_CHECK: field 1 = check_length (the MINIMUM encoder reading
        # required to pass), field 2 = error_length (commanded feed distance /
        # checkpoint where the check is evaluated). Both 3..254. The constraint is
        # check_length < error_length * 1.2342 (enforced in __init__), not an
        # ordering between the two. Lower check_length to widen the slip tolerance
        # and cut false assist errors.
        check_len = int(params.get('check_length', 254))
        error_len = int(params.get('error_length', 254))
        return Cmd.SET_FEED_CHECK, (pb_uint32(1, check_len)
                                    + pb_uint32(2, error_len))
    if method == 'mfrc522_reg_read':
        # arg = (slot<<16) | reg  (see Cmd.MFRC522_REG_READ)
        return Cmd.MFRC522_REG_READ, pb_uint32(1, int(params.get('arg', 0)) & 0xFFFFFFFF)
    if method == 'mfrc522_reg_write':
        # arg = (slot<<16) | (reg<<8) | value
        return Cmd.MFRC522_REG_WRITE, pb_uint32(1, int(params.get('arg', 0)) & 0xFFFFFFFF)
    if method == 'mfrc522_reader_power':
        # arg = (reader_index<<16) | on  (see Cmd.MFRC522_READER_POWER)
        return Cmd.MFRC522_READER_POWER, pb_uint32(1, int(params.get('arg', 0)) & 0xFFFFFFFF)
    if method == 'filament_identify':
        return Cmd.FILAMENT_IDENTIFY, pb_uint32(1, int(params.get('index', 0)))
    if method == 'set_dry_temp':
        return Cmd.SET_DRY_TEMP, pb_uint32(1, int(params.get('temp', 50)))
    if method == 'get_temp':
        return Cmd.GET_TEMP, b''
    if method == 'get_feed_info':
        return Cmd.GET_FEED_INFO, b''
    if method == 'get_material_info':
        return Cmd.GET_MATERIAL_INFO, pb_uint32(1, int(params.get('index', 0)))
    if method == 'set_material_name':
        # field 1 = slot index, field 2 = the material-name string. Variable
        # length, persists to NVM, and clears the slot status byte.
        slot = int(params.get('index', 0))
        name = str(params.get('name', ''))
        return Cmd.SET_MATERIAL_NAME, pb_uint32(1, slot) + pb_string(2, name)
    if method in ('get_sensor_state', 'get_key_state'):
        return Cmd.GET_SENSOR_STATE, b''
    if method == 'raw':
        cmd_id = int(params.get('cmd', 0))
        hex_payload = params.get('hex', '') or ''
        try:
            payload = bytes.fromhex(hex_payload) if hex_payload else b''
        except ValueError:
            payload = b''
        return cmd_id, payload
    # Unknown method (e.g. V1-only enable_rfid/disable_rfid/drying_stop variants):
    # fall back to a harmless status query rather than sending garbage.
    _logger.debug("ACE2: unknown method %r -> GET_STATUS", method)
    return Cmd.GET_STATUS, b''


def encode_frame(request_id: int, cmd: int, payload: bytes = b'') -> bytes:
    """
    Build a complete V2 request frame from an explicit opcode and payload.

    Lower-level than :func:`encode_request`: takes the command byte directly
    instead of mapping a method name.

    :param request_id: Sequence number for the request (truncated to 16 bits).
    :param cmd: Command opcode byte to send.
    :param payload: Raw protobuf payload bytes (default empty).
    :return: Complete framed request as bytes (preamble, header, payload, CRC,
             end marker).
    :raises ValueError: When the payload exceeds ``MAX_PAYLOAD_LEN``.
    """
    seq = int(request_id) & 0xFFFF
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(
            "V2 payload exceeds %d bytes for opcode %d" % (MAX_PAYLOAD_LEN, cmd))
    inner = bytearray([
        FLAG_REQUEST,
        seq & 0xFF, (seq >> 8) & 0xFF,
        cmd & 0xFF,
        len(payload) & 0xFF,
    ])
    inner.extend(payload)
    crc = crc16_kermit(bytes(inner))
    return (bytes(PREAMBLE) + bytes(inner)
            + bytes([crc & 0xFF, (crc >> 8) & 0xFF, END_MARKER]))


def encode_request(request_id: int, method: str,
                   params: Optional[Dict[str, Any]]) -> bytes:
    """
    Build a complete V2 request frame for a method/params.

    :param request_id: Sequence number for the request (truncated to 16 bits).
    :param method: V1 method name to encode.
    :param params: Optional dict of method parameters.
    :return: Complete framed request as bytes (preamble, header, payload, CRC,
             end marker).
    :raises ValueError: When the encoded payload exceeds ``MAX_PAYLOAD_LEN``.
    """
    cmd, payload = method_to_v2(method, params)
    return encode_frame(request_id, cmd, payload)


def _decode_status(fields: Dict[int, List[Tuple[int, Any]]]) -> Dict[str, Any]:
    """
    Convert a decoded GET_STATUS message into a V1-shaped status dict.

    Builds a list of four slot entries (padding missing slots as empty), decodes
    the dryer sub-message, and derives an overall busy/ready status.

    :param fields: Decoded protobuf fields from :func:`pb_decode`.
    :return: V1-shaped status dict with ``slots``, ``dryer_status``, ``temp``,
             ``humidity`` and related keys.
    """
    slots: List[Dict[str, Any]] = []
    for wtype, slot_payload in fields.get(9, []):
        if wtype != 2:
            continue
        sub = pb_decode(slot_payload)
        slot_state = SLOT_STATES.get(_fval(sub, 1, 0), 'unknown')
        filament_state = FILAMENT_STATES.get(_fval(sub, 2, 0), 'empty')
        slots.append({
            # Slot index (0-based, by position) — matches the V1 status shape so
            # consumers (e.g. _derive_action's slot tag) can name the slot.
            'index': len(slots),
            # Any non-empty filament state means filament is present in the
            # slot. The unit only reports 'identified' once RFID is read; a plain
            # insert (or a spool with no/again-unread tag) reports 'unknown' /
            # 'identifying'. Treat all of those as 'ready' (present) so the lane
            # loads — RFID identification is carried separately in 'rfid'.
            'status': 'empty' if filament_state == 'empty' else 'ready',
            'slot_status': slot_state,
            'sku': '', 'type': '',
            'rfid': 2 if filament_state == 'identified' else 0,
            'brand': '',
            'color': [0, 0, 0],
        })
    while len(slots) < 4:
        slots.append({
            'index': len(slots),
            'status': 'empty', 'slot_status': 'unknown',
            'sku': '', 'type': '', 'rfid': 0, 'brand': '',
            'color': [0, 0, 0],
        })
    dry_status = {'status': 'stop', 'target_temp': 0,
                  'duration': 0, 'remain_time': 0}
    for wtype, dry_payload in fields.get(2, []):
        if wtype != 2:
            continue
        dsub = pb_decode(dry_payload)
        dry_status = {
            'status': DRY_STATES.get(_fval(dsub, 1, 0), 'stop'),
            'target_temp': _fval(dsub, 2, 0),
            'duration': _fval(dsub, 3, 0),
            'remain_time': _fval(dsub, 4, 0),
        }
        break
    any_busy = any(
        s.get('slot_status') in ('feeding', 'rollback', 'preloading')
        for s in slots)
    return {
        'status': 'busy' if any_busy else 'ready',
        'dryer_status': dry_status,
        'temp': _fval(fields, 3, 0),
        'humidity': _fval(fields, 4, 0),
        'enable_rfid': 1 if _fval(fields, 5, 0) else 0,
        'fan_speed': 0,
        'feed_assist_count': _fval(fields, 7, 0),
        'cont_assist_time': float(_fval(fields, 8, 0)),
        'slots': slots,
    }


def v2_response_to_v1(cmd: int, seq: int, payload: bytes,
                      logger: Optional[logging.Logger] = None) -> Dict[str, Any]:
    """
    Decode a V2 response payload into a V1-shaped {id, code, msg, result}.

    :param cmd: V2 command opcode the response corresponds to.
    :param seq: Sequence number echoed from the request.
    :param payload: Raw protobuf payload bytes from the response frame.
    :param logger: Optional logger for decode failures; module logger if None.
    :return: V1-shaped dict with ``id``, ``code``, ``msg`` and ``result`` keys.
    """
    # '_cmd' lets the transport verify a reply's opcode matches the pending
    # request (id-reuse guard in ACE2Connection._response_matches_pending). It's
    # a top-level key, so send_command's result.get('result') never exposes it.
    ret = {'id': seq, '_cmd': cmd, 'code': 0, 'msg': 'success', 'result': {}}
    if not payload:
        return ret
    try:
        fields = pb_decode(payload)
    except Exception as e:
        (logger or _logger).debug("ACE2 protobuf decode failure cmd=%d: %s", cmd, e)
        return ret
    if cmd == Cmd.DISCOVER_DEVICE:
        ret['result'] = {'uid1': _fval(fields, 1), 'uid2': _fval(fields, 2),
                         'uid3': _fval(fields, 3)}
    elif cmd == Cmd.GET_INFO:
        ret['result'] = {'model': 'ACE 2 Pro',
                         'firmware': _fstr(fields, 1, ''),
                         'boot_version': _fstr(fields, 2, '')}
    elif cmd == Cmd.GET_STATUS:
        ret['result'] = _decode_status(fields)
    elif cmd == Cmd.GET_TEMP:
        ret['result'] = {
            'box1_temp': _fval(fields, 1, 0.0), 'box2_temp': _fval(fields, 2, 0.0),
            'ptc1_temp': _fval(fields, 3, 0.0), 'ptc2_temp': _fval(fields, 4, 0.0),
            'env_temp': _fval(fields, 5, 0.0), 'env_humidity': _fval(fields, 6, 0.0),
        }
    elif cmd == Cmd.GET_MATERIAL_INFO:
        # Stored per-slot material name (+ status byte). The name lives in a
        # nested message at field 2 -> field 1 (request field 1 = slot index;
        # response field 2 = {1: name}). Written by SET_MATERIAL_NAME (cmd 18).
        material_name = ''
        for wtype, sub_payload in fields.get(2, []):
            if wtype != 2:
                continue
            material_name = _fstr(pb_decode(sub_payload), 1, '')
            break
        ret['result'] = {
            'index': _fval(fields, 1, 0),
            'material_name': material_name,
            'status': _fval(fields, 3, 0),
            'raw': dump_fields(fields),
        }
    elif cmd == Cmd.GET_SENSOR_STATE:
        # 17-channel filament-sensor bitmask in field 1. NOTE: without this
        # branch the generic decoder below mis-reads field 1 as an error code
        # (a real bitmask like 70928 shows up as 'error_70928'). Decode it into
        # the raw mask plus a per-channel boolean list.
        mask = _fval(fields, 1, 0)
        mask = int(mask) if isinstance(mask, int) else 0
        sensors = [bool(mask & (1 << i)) for i in range(SENSOR_BITMASK_WIDTH)]
        # Decode into per-slot named signals so callers get each lane's buffer
        # state (buf_rst / buf_back) without re-deriving bit offsets.
        slot_sensors = []
        for slot in range(4):
            base = slot * SENSORS_PER_SLOT
            slot_sensors.append({
                'insert': sensors[base + SENSOR_INSERT],
                'empty': sensors[base + SENSOR_EMPTY],
                'buf_rst': sensors[base + SENSOR_BUF_RST],
                'buf_back': sensors[base + SENSOR_BUF_BACK],
            })
        ret['result'] = {
            'sensor_bitmask': mask,
            'sensors': sensors,
            'slot_sensors': slot_sensors,
            'buf_feed': sensors[SENSOR_BUF_FEED_BIT],
        }
    elif cmd == Cmd.GET_FEED_INFO:
        # Per-slot feed diagnostics: repeated FeedInfo { steps, length, decoder }.
        # Field numbers follow the declaration order; best-effort decode for
        # tuning/diagnostics only.
        feeds = []
        for wtype, sub_payload in fields.get(1, []):
            if wtype != 2:
                continue
            sub = pb_decode(sub_payload)
            feeds.append({
                'steps': _fval(sub, 1, 0),
                'length': _fval(sub, 2, 0),
                'decoder': _fval(sub, 3, 0),
            })
        ret['result'] = {'feed_info': feeds,
                         'raw_fields': sorted(fields.keys())}
    elif cmd in (Cmd.GET_FILAMENT_INFO, Cmd.FILAMENT_IDENTIFY):
        ftype = _fstr(fields, 4, '')
        color = [0, 0, 0]
        for wtype, color_payload in fields.get(5, []):
            if wtype != 2:
                continue
            csub = pb_decode(color_payload)
            rgba = _fval(csub, 1, 0)
            color = [(rgba >> 24) & 0xFF, (rgba >> 16) & 0xFF, (rgba >> 8) & 0xFF]
            break
        # Temperature ranges live in nested {1: min, 2: max} tag fields: field 6
        # = nozzle, field 7 = bed. Shaped as the {min,max} dicts the ACE
        # inventory (_apply_slot_info) consumes for extruder_temp_min/max and
        # bed_temp_min/max.
        def _temp_range(fnum: int) -> Dict[str, Any]:
            for _w, _payload in fields.get(fnum, []):
                if _w != 2:
                    continue
                _sub = pb_decode(_payload)
                _lo, _hi = _fval(_sub, 1, 0), _fval(_sub, 2, 0)
                return {'min': _lo, 'max': _hi} if (_lo or _hi) else {}
            return {}
        extruder_temp = _temp_range(6)
        bed_temp = _temp_range(7)
        ret['result'] = {
            'index': _fval(fields, 1, 0),
            'sku': _fstr(fields, 3, ''),
            'type': ftype, 'brand': '', 'color': color,
            'rfid': 2 if ftype else 0,
            # Richer ACE2 tag fields. diameter is uint32 in 0.01mm units.
            # field 11 is a static value on the read-only tag, surfaced as
            # total_length (mm). Not a live remaining; AFC/Spoolman tracks
            # consumption itself.
            'diameter': _fval(fields, 8, 0) / 100.0,
            'total_length': _fval(fields, 11, 0),
            'extruder_temp': extruder_temp,
            'hotbed_temp': bed_temp,
            # Full raw field map for diagnostics (see ACE_RFID_DUMP).
            'raw': dump_fields(fields),
        }
    elif cmd == Cmd.MFRC522_REG_READ:
        # Custom passthrough reply: field 1 = the MFRC522 register byte.
        ret['result'] = {'val': _fval(fields, 1, 0) & 0xFF}
    else:
        # Only opcodes that actually report a status in field 1 get the
        # field-1-as-error treatment (see _ERROR_CODE_OPCODES). For any other
        # opcode field 1 is data — mapping it to a code would make send_command
        # raise on a good ack (the GET_SENSOR_STATE 'error_70928' class of bug).
        if cmd in _ERROR_CODE_OPCODES:
            code = _fval(fields, 1, 0)
            if isinstance(code, int) and code != 0:
                ret['code'] = code
                ret['msg'] = 'error_%d' % code
        # No dedicated decoder for this opcode. Surface the decoded protobuf
        # field map under raw_fields. Action commands (feed/drying/valve/…)
        # land here; their callers ignore the result, so this key is harmless.
        ret['result'] = {'raw_fields': dump_fields(fields)}
    return ret


def decode_frames(buffer: bytearray,
                  logger: Optional[logging.Logger] = None) -> List[Dict[str, Any]]:
    """
    Consume complete V2 frames from a bytearray, returning V1-shaped
    response dicts (request frames are skipped). Mutates buffer in place.

    Resyncs on the preamble, validates payload length, end marker and CRC,
    dropping malformed or partial frames as needed.

    :param buffer: Mutable ``bytearray`` of received bytes; consumed in place.
    :param logger: Optional logger for diagnostics; module logger if None.
    :return: List of V1-shaped response dicts decoded from complete frames.
    """
    log = logger or _logger
    results = []
    while len(buffer) >= MIN_FRAME_LEN:
        start = buffer.find(PREAMBLE)
        if start < 0:
            if buffer.endswith(b'\xFF'):
                del buffer[:-1]
            else:
                del buffer[:]
            break
        if start > 0:
            del buffer[:start]
        if len(buffer) < HEADER_LEN:
            break
        payload_len = buffer[6]
        if payload_len > MAX_PAYLOAD_LEN:
            del buffer[:2]
            continue
        total_len = HEADER_LEN + payload_len + TRAILER_LEN
        if len(buffer) < total_len:
            break
        if buffer[total_len - 1] != END_MARKER:
            del buffer[:2]
            continue
        inner = bytes(buffer[2:HEADER_LEN + payload_len])
        crc_in_frame = (buffer[HEADER_LEN + payload_len]
                        | (buffer[HEADER_LEN + payload_len + 1] << 8))
        if crc_in_frame != crc16_kermit(inner):
            log.debug("ACE2 CRC mismatch, dropping frame")
            del buffer[:total_len]
            continue
        flags = buffer[2]
        seq = buffer[3] | (buffer[4] << 8)
        cmd = buffer[5]
        payload = bytes(buffer[HEADER_LEN:HEADER_LEN + payload_len])
        del buffer[:total_len]
        if not (flags & FLAG_RESPONSE):
            continue
        results.append(v2_response_to_v1(cmd, seq, payload, log))
    return results


# ── Transport: V2 framing over the inherited connection machinery ─────────
class ACE2Connection(ACEConnection):
    """ACE 2 Pro transport. Reuses the V1 connection's reactor I/O, request
    completion, heartbeat and reconnect machinery, but encodes/decodes the V2
    binary protocol. The high-level command wrappers (feed_filament, get_status,
    get_filament_info, …) are inherited verbatim — they just call send_command,
    which here speaks V2 and returns the same V1-shaped result dicts."""

    def _pre_info_handshake(self) -> None:
        """ACE 2 Pro must be discovered before it answers other commands. The V2
        initial handshake sends discover_device first (then get_info); without it
        the unit ignores get_status/get_info and never replies.
        """
        try:
            self.send_command("discover_device", timeout=3.0)
        except Exception as e:
            self._logger.debug(
                f"ACE2 discover_device failed (non-fatal): {e}")

    def _poll_extras(self) -> None:
        """Poll GET_TEMP and GET_SENSOR_STATE each heartbeat so the unit caches
        the box/PTC/env thermal channels (for temperature_ace sensors) and the
        per-lane buffer/filament sensor state (for get_status). Fire-and-forget:
        the replies are routed to the unit's caches by _on_hw_status_callback.
        ACE2 only (the V1 firmware has neither command)."""
        self.send_command_async("get_temp")
        self.send_command_async("get_sensor_state")

    def send_command(self, method: str, params: Optional[Dict[str, Any]] = None,
                     timeout: float = REQUEST_TIMEOUT) -> Any:
        """
        Encode and send a V2 request, then block for its matching response.

        :param method: V1 method name to send.
        :param params: Optional dict of method parameters.
        :param timeout: Maximum time in seconds to wait for the response.
        :return: The ``result`` portion of the decoded response.
        :raises ACESerialError: When not connected, encoding/writing fails, or
                                the device reports a non-zero error code.
        :raises ACETimeoutError: When no response arrives before the deadline.
        """
        if not self._connected or self._serial is None:
            raise ACESerialError("ACE2 not connected")
        # The V2 wire protocol carries a 16-bit sequence id (encode_request
        # masks it), and the unit echoes back only (id & 0xFFFF). Key the
        # pending completion by that same masked value — otherwise, once
        # _next_request_id passes 65535, the echoed id (e.g. 86480 -> 20944)
        # never matches self._pending and every request "times out".
        request_id = self._next_request_id & 0xFFFF
        self._next_request_id += 1
        try:
            cmd, payload = method_to_v2(method, params or {})
            frame = encode_frame(request_id, cmd, payload)
        except Exception as e:
            raise ACESerialError(f"ACE2 encode failed for '{method}': {e}")

        completion = self._reactor.completion()
        self._pending[request_id] = completion
        # Remember the opcode so a stale reply landing on a reused 16-bit id
        # (which recycle every 65536 requests) can't complete THIS request with
        # another command's data — see _response_matches_pending.
        self._pending_cmd[request_id] = cmd
        try:
            self._serial.write(frame)
            self._serial.flush()
        except Exception as e:
            self._pending.pop(request_id, None)
            self._pending_cmd.pop(request_id, None)
            self._track_timeout()
            # Write failure is a strong disconnect signal — reconnect now rather
            # than limp until the heartbeat-silence check.
            self.reconnect()
            raise ACESerialError(f"ACE2 write failed: {e}")

        self._logger.debug(f"ACE2 TX: id={request_id} {method} {params or {}}")
        deadline = self._reactor.monotonic() + timeout
        # try/finally so both pending maps are always cleared, even if wait() is
        # interrupted.
        try:
            result = completion.wait(deadline)
        finally:
            self._pending.pop(request_id, None)
            self._pending_cmd.pop(request_id, None)

        if result is None:
            self._track_timeout()
            raise ACETimeoutError(
                f"ACE2 command '{method}' (id={request_id}) timed out after {timeout}s")
        if isinstance(result, dict):
            code = result.get("code", 0)
            if code != 0:
                raise ACESerialError(
                    f"ACE2 command '{method}' failed: code={code}, "
                    f"msg={result.get('msg') or 'error'}")
            return result.get("result", result)
        return result

    def send_command_async(self, method: str,
                           params: Optional[Dict[str, Any]] = None) -> None:
        """
        Send a V2 request without waiting for a response (fire-and-forget).

        Encode/write failures are logged and swallowed so the caller never
        blocks; the request id is tracked so its reply can be discarded.

        :param method: V1 method name to send.
        :param params: Optional dict of method parameters.
        """
        if not self._connected or self._serial is None:
            return
        # 16-bit wire id (see send_command): track the masked value so the
        # unit's echoed id matches and its reply is recognised, not logged as
        # an "unknown request".
        request_id = self._next_request_id & 0xFFFF
        self._next_request_id += 1
        self._async_ids.append(request_id)
        try:
            frame = encode_request(request_id, method, params or {})
        except Exception as e:
            self._logger.debug(f"ACE2 async encode failed: {e}")
            return
        try:
            self._serial.write(frame)
            self._serial.flush()
        except Exception as e:
            self._logger.debug(f"ACE2 async write failed: {e}")
            self.reconnect()      # disconnect signal — reconnect immediately
            return
        self._logger.debug(f"ACE2 TX (async): id={request_id} {method}")

    def _response_matches_pending(self, response_id: int,
                                  response: Any) -> bool:
        """Reject a reply whose opcode doesn't match the pending request's. The
        16-bit wire id recycles every 65536 requests, so a very late reply for a
        timed-out id could otherwise complete a DIFFERENT later request that
        reused the id — handing the caller another command's data. The decoded
        response carries its opcode in '_cmd'; require it to match. If we have no
        recorded opcode for the id (async, or already cleared), don't block."""
        expected = self._pending_cmd.get(response_id)
        if expected is None:
            return True
        got = response.get('_cmd') if isinstance(response, dict) else None
        return got is None or got == expected

    def _parse_frames(self) -> None:
        """Decode V2 frames from the read buffer and dispatch to _handle_response
        (inherited), which routes by response id to the pending completion."""
        buf = bytearray(self._read_buffer)
        responses = decode_frames(buf, self._logger)
        self._read_buffer = bytes(buf)
        for response in responses:
            self._logger.debug(f"ACE2 RX: {response}")
            self._handle_response(response)

    # The Pro 2 enables RFID per-slot (V1's global enable_rfid/disable_rfid don't
    # exist). Best-effort enable/disable all slots; fire-and-forget so connect
    # never blocks on it.
    def enable_rfid(self) -> None:
        """Best-effort enable per-slot RFID identification on every slot."""
        for slot in range(self.slot_count):
            self.send_command_async(
                "set_rfid_enable", {"index": slot, "enable": True})

    def disable_rfid(self) -> None:
        """Best-effort disable per-slot RFID identification on every slot."""
        for slot in range(self.slot_count):
            self.send_command_async(
                "set_rfid_enable", {"index": slot, "enable": False})


# ── Unit: the V1 ACE unit with the V2 transport swapped in ───────────
class afcACE2(afcACE):
    """Anycubic ACE 2 Pro AFC unit. Reuses all of afcACE's AFC integration
    (load/unload, feed assist, RFID->Spoolman, dryer, diagnostics) and only
    swaps the serial transport to the V2 protocol."""

    _LOGO_TITLE = "ACE 2 PRO"
    # Unlike V1, the ACE 2's insert preload only grips the filament at the slot;
    # it does not advance it to the hub. Defer to prep_post_load's dist_hub feed
    # so the lane actually stages to the hub after a fresh insert settles.
    _preloads_to_hub_on_insert = False
    # The ACE 2 reads tags host-side over the MFRC522 passthrough
    # (AFC_ACE2_rfid), not through the firmware's get_filament_info — so it must
    # not do the V1 firmware inventory sweep at startup prep.
    _uses_firmware_rfid = False

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Initialize the ACE 2 Pro unit on top of the V1 ACE unit.

        :param config: ConfigWrapper for this unit; ``type`` defaults to ``ACE2``
                       and the dryer set-point ceiling defaults to 70C.
        """
        super().__init__(config)
        self.type = config.get('type', 'ACE2')
        # ACE 2 Pro V2 serial runs at 230400 baud (the V1 ACE default is
        # 115200); at the wrong baud the unit never sees a valid frame and
        # never replies. Override the inherited default.
        self.baud_rate = config.getint("baud_rate", 230400)
        # Pro 2 allows a higher dryer set-point than the Pro V1 (55C default).
        self.max_dryer_temperature = config.getfloat(
            "max_dryer_temperature", 70.0, minval=0.0)
        # Encoder feed-check tuning (V2 SET_FEED_CHECK):
        #   - feed_error_length: commanded feed distance (mm) at which the check
        #     is evaluated.
        #   - feed_check_length: the MINIMUM encoder reading required to pass.
        #   - the firmware expects encoder = feed_error_length * 1.2342 for a
        #     clean feed and transitions to FEED_ERROR / ASSIST_ERROR when the
        #     measured encoder reading < feed_check_length at that checkpoint.
        # So the slip tolerance is:
        #     tolerance_mm ~= feed_error_length - feed_check_length / 1.2342
        # To cut false assist errors, LOWER feed_check_length (this widens the
        # tolerance without moving the checkpoint). The default 200/185 gives
        # ~23mm of tolerance vs the firmware default 100/90 (~9mm). To
        # effectively disable the check, set feed_check_length to its minimum (3)
        # so essentially any movement passes.
        self.feed_check_length = config.getint(
            "feed_check_length", 200, minval=3, maxval=254)
        self.feed_error_length = config.getint(
            "feed_error_length", 185, minval=3, maxval=254)
        # The encoder can only ever reach feed_error_length * 1.2342, so a
        # feed_check_length at or above that is unreachable and makes EVERY feed
        # raise FEED_ERROR. Reject that misconfiguration (this is the real
        # constraint — feed_error_length and feed_check_length are independent
        # axes, not an ordering).
        _expected = self.feed_error_length * ACE2_ENCODER_SCALE
        if self.feed_check_length >= _expected:
            raise config.error(
                "[%s] feed_check_length (%d) must be < feed_error_length * %.4f "
                "= %.0f, otherwise the encoder can never reach it and every feed "
                "raises FEED_ERROR. Lower feed_check_length to widen tolerance "
                "(tolerance_mm ~= feed_error_length - feed_check_length / %.4f)."
                % (config.get_name(), self.feed_check_length, ACE2_ENCODER_SCALE,
                   _expected, ACE2_ENCODER_SCALE))
        # Stuck/tangle detection. Unlike the V1 ACE (which only exposes
        # cont_assist_time and forces the time-based heuristic in the parent
        # _check_stuck), the ACE2 has a real filament encoder: its OdometerTimer
        # task compares commanded motor steps against measured encoder movement
        # and reports a per-slot error state (stuck/tangled/assist/motor error)
        # when they diverge. That is a true mechanical-jam signal, so we enable
        # detection by default here and override _check_stuck (below) to react to
        # those firmware states instead of the assist-duration proxy. Re-read the
        # same key the parent read with default False, flipping the default to
        # True for the ACE2. ACE_STUCK_SPOOL_DETECTION still toggles it at runtime.
        self._stuck_detection = config.getboolean("stuck_spool_detection", True)

    def _apply_feed_check(self) -> None:
        """Push the encoder feed-check window to the ACE (V2 only). The unit
        forgets it across a reset, so this is sent on connect and reconnect.
        """
        if self._ace is None:
            return
        try:
            self._ace.send_command_async("set_feed_check", {
                "check_length": self.feed_check_length,
                "error_length": self.feed_error_length,
            })
            self.logger.info(
                "ACE2 %s: feed check set check_length=%d error_length=%d"
                % (self.name, self.feed_check_length, self.feed_error_length))
        except Exception as e:
            self.logger.warning(
                "ACE2 %s: set_feed_check failed (non-fatal): %s"
                % (self.name, e))

    # ACE2 slot states that mean the encoder/odometer disagrees with the
    # commanded motor move — a real upstream jam, tangle, slip, or motor fault.
    # The unit's OdometerTimer task derives these, so they are hardware-accurate.
    _ENCODER_JAM_STATES = (
        'stuck_error', 'tangled_error', 'assist_error', 'motor_error')

    def _check_stuck(self, result: Dict[str, Any]) -> None:
        """ACE2 stuck/tangle detection from the firmware odometer verdict.

        The ACE2 carries a filament encoder; its OdometerTimer task compares the
        commanded motor steps against the measured encoder movement and reports a
        per-slot error state (stuck/tangled/assist/motor error) when they
        diverge. That is a true mechanical-jam signal, so we react to it directly
        instead of inferring a jam from how long feed-assist has run (the V1
        ``_check_stuck`` heuristic, which this overrides). Gated to the active
        assist lane's slot — the one being driven during a print — so an idle
        slot's stale error can never pause a healthy print. The caller skips this
        during load/unload (``_operation_active``), where feed/rollback errors
        are part of normal operation.

        :param result: Decoded GET_STATUS dict with a per-slot ``slots`` list.
        """
        if not self._stuck_detection:
            return
        # Only meaningful during a real, un-paused print.
        if (not self.afc.function.in_print()
                or self.afc.function.is_paused()):
            self._stuck_tripped = False
            return
        name = self._active_assist_lane()
        if name is None:
            self._stuck_tripped = False
            return
        slot = self._slot_map.get(name)
        slots = result.get("slots")
        if (slot is None or not isinstance(slots, list)
                or slot >= len(slots) or not isinstance(slots[slot], dict)):
            return
        state = slots[slot].get("slot_status")
        if state not in self._ENCODER_JAM_STATES:
            # Healthy (or recovered) — re-arm the one-shot latch.
            self._stuck_tripped = False
            return
        if self._stuck_tripped:
            return  # already handled this jam; don't re-pause every heartbeat
        self._stuck_tripped = True
        # Defer the pause off the serial/heartbeat path onto the reactor.
        self.afc.reactor.register_callback(
            lambda et, n=name, s=slot, st=state:
            self._handle_encoder_jam(n, s, st))

    def _handle_encoder_jam(self, name: str, slot: int, state: str) -> None:
        """Stop assist on the jammed slot and pause the print (deferred from
        ``_check_stuck``) for an ACE2 firmware-reported encoder jam.

        :param name: Lane name whose slot reported the jam.
        :param slot: ACE2 slot index for that lane.
        :param state: The firmware slot_status that tripped (e.g. ``stuck_error``).
        """
        # Stop driving the motor into the blockage before we pause.
        try:
            self._stop_feed_assist(slot)
        except Exception:
            pass
        pretty = {
            'stuck_error': 'stuck spool (encoder saw no movement while feeding)',
            'tangled_error': 'tangled spool',
            'assist_error': 'feed-assist slip (encoder fell behind the motor)',
            'motor_error': 'motor error',
        }.get(state, state)
        msg = (
            "ACE2 {unit} lane {lane}: {what}. The unit's filament encoder reports "
            "the spool is not moving with the motor — likely a jam or tangle at "
            "the unit. Clear the snag, then resume. Run ACE_STUCK_SPOOL_DETECTION "
            "ENABLE=0 to disable this check.".format(
                unit=self.name, lane=name, what=pretty))
        try:
            # Route through AFC so it snapshots position and uses the AFC
            # pause/resume path (z-hop, restore on AFC_RESUME).
            self.afc.error.AFC_error(msg, pause=True)
        except Exception:
            self.logger.error(msg)
            self.gcode.run_script_from_command("PAUSE")

    def _make_connection(self, reactor: Reactor, serial_port: str,
                         logger: logging.Logger,
                         baud_rate: int) -> ACE2Connection:
        """
        Create the V2 transport used in place of the V1 ACE connection.

        :param reactor: Klipper reactor for scheduling I/O.
        :param serial_port: Serial device path for the ACE 2 Pro.
        :param logger: Logger passed to the connection.
        :param baud_rate: Serial baud rate.
        :return: A configured :class:`ACE2Connection` instance.
        """
        return ACE2Connection(reactor=reactor, serial_port=serial_port,
                              logger=logger, baud_rate=baud_rate)

    def _reader_sibling_slot(self, slot: int) -> Optional[int]:
        """ACE 2 Pro has 2 MFRC522 readers, each covering a slot PAIR (0/1 -> r0,
        2/3 -> r1). The paired slot shares the reader, so a static read of one can
        return the other's tag — used to skip ambiguous startup RFID reads."""
        sib = int(slot) ^ 1
        return sib if 0 <= sib < self.SLOTS_PER_UNIT else None

    # NOTE: the ACE 2 does NOT use the firmware inventory path at all. Factory
    # identify is disabled at startup and tags are read host-side over the
    # MFRC522 passthrough (AFC_ACE2_rfid), which carries the real per-tag UID and
    # drives Spoolman matching/creation straight onto the lane. So there is no
    # ACE2 _refresh_slot_inventory override — the base one is firmware-only and
    # already gated off by _uses_firmware_rfid = False.


def load_config_prefix(config: ConfigWrapper) -> afcACE2:
    """
    Klipper entry point that instantiates the ACE 2 Pro unit.

    :param config: ConfigWrapper for the unit section.
    :return: A new :class:`afcACE2` instance.
    """
    return afcACE2(config)
