"""
Branch-coverage tests for extras/AFC_ACE2.py (the ACE 2 Pro V2 serial
transport/protocol), covering the paths the other ACE2 test files leave
untested:

  protobuf encode/decode helpers   — pb_varint (negative reject), pb_bool
                                      (falsy), pb_decode_varint (truncation),
                                      pb_decode (64/32-bit + unsupported wire
                                      types), _fval / _fstr, dump_fields
  method_to_v2                     — every remaining method mapping + raw
                                      (valid/invalid/empty hex) + unknown fallback
  _decode_status                   — GET_STATUS slot/dryer/busy decode
  v2_response_to_v1                — DISCOVER_DEVICE, GET_INFO, GET_STATUS,
                                      GET_FEED_INFO, GET_FILAMENT_INFO /
                                      FILAMENT_IDENTIFY, MFRC522_REG_READ,
                                      the decode-failure guard
  decode_frames                    — framing, resync, oversize, CRC, end-marker,
                                      request-skip, partial-frame edge cases
  ACE2Connection                   — _pre_info_handshake, _poll_extras,
                                      send_command (not-connected / encode /
                                      write / timeout / error-code / non-dict),
                                      send_command_async, _response_matches_pending,
                                      _parse_frames, enable_rfid / disable_rfid
  afcACE2                          — _apply_feed_check, _handle_encoder_jam,
                                      _make_connection, _reader_sibling_slot

Style: typed local fakes (matching the sibling ACE2 tests), full state
verification, every branch driven both ways.
"""

from __future__ import annotations

import logging
import struct
from typing import Any, List, Optional, Tuple

import pytest

from extras.AFC_ACE2 import (
    Cmd,
    ACE2Connection,
    afcACE2,
    crc16_kermit,
    decode_frames,
    dump_fields,
    encode_request,
    method_to_v2,
    pb_bool,
    pb_decode,
    pb_decode_varint,
    pb_string,
    pb_uint32,
    pb_varint,
    v2_response_to_v1,
    _fstr,
    _fval,
    PREAMBLE,
    END_MARKER,
    FLAG_RESPONSE,
    HEADER_LEN,
    MAX_PAYLOAD_LEN,
    MIN_FRAME_LEN,
    ACE2_ENCODER_SCALE,
    FEED_MODE_FEED,
    FEED_MODE_ROLLBACK,
    FEED_MODE_ASSIST,
)
from extras.AFC_ACE import ACESerialError, ACETimeoutError

from tests.ace_helpers import FakeAFC, FakeLogger, Recorder


# ── Local fakes ───────────────────────────────────────────────────────────────

class RecordingLogger:
    """Logger that stores (level, formatted_message) tuples, applying %-args the
    way the module's logging calls do, so tests can assert the exact message."""

    def __init__(self) -> None:
        self.messages: List[Tuple[str, str]] = []

    def _log(self, level: str, msg: Any, args: Tuple[Any, ...]) -> None:
        self.messages.append((level, msg % args if args else msg))

    def debug(self, msg: Any, *args: Any, **k: Any) -> None:
        self._log("debug", msg, args)

    def info(self, msg: Any, *args: Any, **k: Any) -> None:
        self._log("info", msg, args)

    def warning(self, msg: Any, *args: Any, **k: Any) -> None:
        self._log("warning", msg, args)

    def error(self, msg: Any, *args: Any, **k: Any) -> None:
        self._log("error", msg, args)


class FakeCompletion:
    def __init__(self) -> None:
        self._value: Any = None

    def complete(self, value: Any) -> None:
        self._value = value

    def wait(self, deadline: float) -> Any:
        return self._value


class FakeReactor:
    NEVER = 9_999_999_999.0

    def monotonic(self) -> float:
        return 0.0

    def completion(self) -> FakeCompletion:
        return FakeCompletion()


class FakeSerial:
    def __init__(self, write_error: Optional[Exception] = None) -> None:
        self.frames: List[bytes] = []
        self.write_error = write_error
        self.on_write: Optional[Any] = None

    def write(self, frame: bytes) -> None:
        if self.write_error is not None:
            raise self.write_error
        self.frames.append(bytes(frame))
        if self.on_write is not None:
            self.on_write(bytes(frame))

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


def _make_conn(next_id: int = 0,
               logger: Optional[Any] = None) -> ACE2Connection:
    """Build a connected ACE2Connection through its real __init__, then attach
    test doubles for the serial/reactor and mark it connected."""
    conn = ACE2Connection(
        reactor=FakeReactor(), serial_port="/dev/ttyFAKE",
        logger=logger or RecordingLogger(), baud_rate=230400)
    conn._serial = FakeSerial()
    conn._connected = True
    conn._next_request_id = next_id
    conn._reconnect_enabled = False   # keep reconnect() a no-op in tests
    return conn


def _wire_seq(frame: bytes) -> int:
    inner = frame[len(PREAMBLE):]
    return inner[1] | (inner[2] << 8)


def _build_frame(flags: int, seq: int, cmd: int, payload: bytes) -> bytes:
    """Build a raw V2 frame with an explicit flags byte (encode_frame always
    sets FLAG_REQUEST, so responses are built here)."""
    inner = bytearray([flags, seq & 0xFF, (seq >> 8) & 0xFF,
                       cmd & 0xFF, len(payload) & 0xFF])
    inner.extend(payload)
    crc = crc16_kermit(bytes(inner))
    return (bytes(PREAMBLE) + bytes(inner)
            + bytes([crc & 0xFF, (crc >> 8) & 0xFF, END_MARKER]))


def _pb_double(field: int, value: float) -> bytes:
    """Encode a protobuf fixed64 double field (wire type 1)."""
    return bytes([(field << 3) | 1]) + struct.pack('<d', value)


def _pb_float(field: int, value: float) -> bytes:
    """Encode a protobuf fixed32 float field (wire type 5)."""
    return bytes([(field << 3) | 5]) + struct.pack('<f', value)


# ── pb_varint ─────────────────────────────────────────────────────────────────

class TestPbVarint:
    def test_single_byte(self) -> None:
        assert pb_varint(0x7F) == b'\x7f'

    def test_multi_byte(self) -> None:
        # 300 -> 0xAC 0x02: low 7 bits set continuation, then remainder.
        assert pb_varint(300) == bytes([0xAC, 0x02])

    def test_zero(self) -> None:
        assert pb_varint(0) == b'\x00'

    def test_negative_rejected(self) -> None:
        with pytest.raises(ValueError):
            pb_varint(-1)


# ── pb_bool ───────────────────────────────────────────────────────────────────

class TestPbBool:
    def test_truthy_encodes_one(self) -> None:
        # field 2 -> tag 0x10, value 1.
        assert pb_bool(2, True) == bytes([0x10, 0x01])

    def test_falsy_encodes_zero(self) -> None:
        assert pb_bool(2, 0) == bytes([0x10, 0x00])


# ── pb_decode_varint ──────────────────────────────────────────────────────────

class TestPbDecodeVarint:
    def test_multi_byte(self) -> None:
        assert pb_decode_varint(bytes([0xAC, 0x02]), 0) == (300, 2)

    def test_truncated_continuation_returns_partial(self) -> None:
        # A continuation bit at the last byte: the loop exits with the partial
        # result and pos at end (would be 0x7F longer had another byte existed).
        val, pos = pb_decode_varint(bytes([0x80]), 0)
        assert (val, pos) == (0, 1)


# ── pb_decode ─────────────────────────────────────────────────────────────────

class TestPbDecode:
    def test_varint_field(self) -> None:
        assert pb_decode(pb_uint32(1, 5)) == {1: [(0, 5)]}

    def test_double_field(self) -> None:
        fields = pb_decode(_pb_double(1, 2.5))
        assert fields[1][0][0] == 1
        assert fields[1][0][1] == pytest.approx(2.5)

    def test_double_truncated_breaks(self) -> None:
        # wire type 1 tag but fewer than 8 payload bytes -> decode stops early.
        assert pb_decode(bytes([(1 << 3) | 1, 0x00, 0x00])) == {}

    def test_length_delimited_field(self) -> None:
        assert pb_decode(pb_string(2, "AB")) == {2: [(2, b"AB")]}

    def test_float_field(self) -> None:
        fields = pb_decode(_pb_float(3, 1.5))
        assert fields[3][0][0] == 5
        assert fields[3][0][1] == pytest.approx(1.5)

    def test_float_truncated_breaks(self) -> None:
        assert pb_decode(bytes([(3 << 3) | 5, 0x00])) == {}

    def test_unsupported_wire_type_breaks(self) -> None:
        # wire type 3 (start-group) is unsupported: decode stops, field dropped.
        assert pb_decode(bytes([(1 << 3) | 3, 0x00])) == {}

    def test_repeated_field_kept_in_order(self) -> None:
        assert pb_decode(pb_uint32(1, 7) + pb_uint32(1, 8)) == {
            1: [(0, 7), (0, 8)]}


# ── _fval / _fstr ─────────────────────────────────────────────────────────────

class TestFval:
    def test_present(self) -> None:
        assert _fval({1: [(0, 42)]}, 1) == 42

    def test_absent_returns_default(self) -> None:
        assert _fval({}, 1, default=99) == 99


class TestFstr:
    def test_utf8_decode(self) -> None:
        assert _fstr({1: [(2, b"hi")]}, 1) == "hi"

    def test_invalid_utf8_falls_back_to_hex(self) -> None:
        assert _fstr({1: [(2, b"\xff\xfe")]}, 1) == "fffe"

    def test_non_bytes_returns_default(self) -> None:
        # A varint value (int) under the field number -> not bytes -> default.
        assert _fstr({1: [(0, 5)]}, 1, default="d") == "d"


# ── dump_fields ───────────────────────────────────────────────────────────────

class TestDumpFields:
    def test_varint_scalar(self) -> None:
        assert dump_fields({1: [(0, 5)]}) == {1: 5}

    def test_float_is_rounded(self) -> None:
        assert dump_fields({1: [(5, 1.234567)]}) == {1: round(1.234567, 4)}

    def test_printable_bytes_repr(self) -> None:
        assert dump_fields({1: [(2, b"AB")]}) == {1: repr("AB")}

    def test_nonprintable_bytes_hex(self) -> None:
        # decodes cleanly but isprintable() is False -> hex rendering.
        assert dump_fields({1: [(2, b"\x01\x02")]}) == {1: "hex:0102"}

    def test_invalid_utf8_bytes_hex(self) -> None:
        assert dump_fields({1: [(2, b"\xff\xfe")]}) == {1: "hex:fffe"}

    def test_non_bytes_wire2_falls_through_to_else(self) -> None:
        # A wtype-2 entry whose value isn't bytes hits the final else branch.
        assert dump_fields({1: [(2, 123)]}) == {1: 123}

    def test_repeated_field_becomes_list(self) -> None:
        assert dump_fields({1: [(0, 1), (0, 2)]}) == {1: [1, 2]}


# ── method_to_v2 ──────────────────────────────────────────────────────────────

class TestMethodToV2:
    def test_get_info(self) -> None:
        assert method_to_v2("get_info", None) == (Cmd.GET_INFO, b'')

    def test_get_status(self) -> None:
        assert method_to_v2("get_status", {}) == (Cmd.GET_STATUS, b'')

    def test_discover_device(self) -> None:
        assert method_to_v2("discover_device", {}) == (Cmd.DISCOVER_DEVICE, b'')

    def test_start_feed_assist(self) -> None:
        cmd, payload = method_to_v2("start_feed_assist", {"index": 2, "speed": 15})
        assert cmd == Cmd.FEED_OR_ROLLBACK
        assert payload == (pb_uint32(1, 2) + pb_uint32(2, 15)
                           + pb_uint32(3, 0) + pb_uint32(4, FEED_MODE_ASSIST))

    def test_start_feed_assist_defaults(self) -> None:
        cmd, payload = method_to_v2("start_feed_assist", {})
        assert payload == (pb_uint32(1, 0) + pb_uint32(2, 10)
                           + pb_uint32(3, 0) + pb_uint32(4, FEED_MODE_ASSIST))

    def test_stop_feed_assist(self) -> None:
        assert method_to_v2("stop_feed_assist", {"index": 3}) == (
            Cmd.STOP_FEED_OR_ROLLBACK, pb_uint32(1, 3))

    def test_feed_filament(self) -> None:
        cmd, payload = method_to_v2(
            "feed_filament", {"index": 1, "length": 40, "speed": 60})
        assert cmd == Cmd.FEED_OR_ROLLBACK
        assert payload == (pb_uint32(1, 1) + pb_uint32(2, 60)
                           + pb_uint32(3, 40) + pb_uint32(4, FEED_MODE_FEED))

    def test_unwind_filament(self) -> None:
        cmd, payload = method_to_v2(
            "unwind_filament", {"index": 1, "length": 40, "speed": 60})
        assert cmd == Cmd.FEED_OR_ROLLBACK
        assert payload == (pb_uint32(1, 1) + pb_uint32(2, 60)
                           + pb_uint32(3, 40) + pb_uint32(4, FEED_MODE_ROLLBACK))

    def test_stop_feed_filament(self) -> None:
        assert method_to_v2("stop_feed_filament", {"index": 2}) == (
            Cmd.STOP_FEED_OR_ROLLBACK, pb_uint32(1, 2))

    def test_update_feeding_speed(self) -> None:
        assert method_to_v2("update_feeding_speed", {"index": 1, "speed": 70}) == (
            Cmd.UPDATE_SPEED, pb_uint32(1, 1) + pb_uint32(2, 70))

    def test_get_filament_info(self) -> None:
        assert method_to_v2("get_filament_info", {"index": 3}) == (
            Cmd.GET_FILAMENT_INFO, pb_uint32(1, 3))

    def test_drying_fan_on(self) -> None:
        cmd, payload = method_to_v2(
            "drying", {"temp": 55, "duration": 120, "fan_speed": 40})
        assert cmd == Cmd.DRYING
        # fan_speed > 0 -> fan_on = 1
        assert payload == (pb_uint32(1, 55) + pb_uint32(2, 120)
                           + pb_uint32(3, 1))

    def test_drying_fan_off(self) -> None:
        cmd, payload = method_to_v2(
            "drying", {"temp": 55, "duration": 120, "fan_speed": 0})
        # fan_speed == 0 -> fan_on = 0
        assert payload == (pb_uint32(1, 55) + pb_uint32(2, 120)
                           + pb_uint32(3, 0))

    def test_drying_stop(self) -> None:
        assert method_to_v2("drying_stop", {}) == (
            Cmd.DRYING, pb_uint32(1, 0) + pb_uint32(2, 0))

    def test_set_fan_speed_on(self) -> None:
        cmd, payload = method_to_v2("set_fan_speed", {"speed": 80})
        # speed > 0 -> both bool fields True (1)
        assert cmd == Cmd.SET_FAN
        assert payload == (pb_uint32(1, 80) + pb_bool(2, True) + pb_bool(3, True))

    def test_set_fan_speed_off(self) -> None:
        cmd, payload = method_to_v2("set_fan_speed", {"speed": 0})
        # speed == 0 -> both bool fields False (0)
        assert payload == (pb_uint32(1, 0) + pb_bool(2, False) + pb_bool(3, False))

    def test_set_rfid_enable_true(self) -> None:
        assert method_to_v2("set_rfid_enable", {"index": 1, "enable": True}) == (
            Cmd.SET_RFID_ENABLE, pb_uint32(1, 1) + pb_bool(2, True))

    def test_set_rfid_enable_false(self) -> None:
        assert method_to_v2("set_rfid_enable", {"index": 1, "enable": False}) == (
            Cmd.SET_RFID_ENABLE, pb_uint32(1, 1) + pb_bool(2, False))

    def test_set_feed_check(self) -> None:
        assert method_to_v2(
            "set_feed_check", {"check_length": 100, "error_length": 90}) == (
            Cmd.SET_FEED_CHECK, pb_uint32(1, 100) + pb_uint32(2, 90))

    def test_mfrc522_reg_read(self) -> None:
        assert method_to_v2("mfrc522_reg_read", {"arg": 0x010203}) == (
            Cmd.MFRC522_REG_READ, pb_uint32(1, 0x010203))

    def test_mfrc522_reg_write(self) -> None:
        assert method_to_v2("mfrc522_reg_write", {"arg": 0x0405}) == (
            Cmd.MFRC522_REG_WRITE, pb_uint32(1, 0x0405))

    def test_mfrc522_reader_power(self) -> None:
        assert method_to_v2("mfrc522_reader_power", {"arg": 0x10001}) == (
            Cmd.MFRC522_READER_POWER, pb_uint32(1, 0x10001))

    def test_filament_identify(self) -> None:
        assert method_to_v2("filament_identify", {"index": 2}) == (
            Cmd.FILAMENT_IDENTIFY, pb_uint32(1, 2))

    def test_set_dry_temp(self) -> None:
        assert method_to_v2("set_dry_temp", {"temp": 65}) == (
            Cmd.SET_DRY_TEMP, pb_uint32(1, 65))

    def test_get_temp(self) -> None:
        assert method_to_v2("get_temp", {}) == (Cmd.GET_TEMP, b'')

    def test_get_feed_info(self) -> None:
        assert method_to_v2("get_feed_info", {}) == (Cmd.GET_FEED_INFO, b'')

    def test_raw_valid_hex(self) -> None:
        assert method_to_v2("raw", {"cmd": 20, "hex": "0102"}) == (20, b"\x01\x02")

    def test_raw_invalid_hex_falls_back_to_empty(self) -> None:
        # A non-hex string raises ValueError inside fromhex -> empty payload.
        assert method_to_v2("raw", {"cmd": 20, "hex": "zz"}) == (20, b"")

    def test_raw_empty_hex(self) -> None:
        assert method_to_v2("raw", {"cmd": 7}) == (7, b"")

    def test_unknown_method_falls_back_to_get_status(
            self, caplog: pytest.LogCaptureFixture) -> None:
        with caplog.at_level(logging.DEBUG, logger="AFC_ACE2"):
            result = method_to_v2("no_such_method", {})
        assert result == (Cmd.GET_STATUS, b'')
        assert caplog.record_tuples == [
            ("AFC_ACE2", logging.DEBUG,
             "ACE2: unknown method 'no_such_method' -> GET_STATUS")]


# ── _decode_status (via v2_response_to_v1 GET_STATUS) ─────────────────────────

def _slot_sub(slot_state: int, filament_state: int) -> bytes:
    return pb_string(9, pb_uint32(1, slot_state) + pb_uint32(2, filament_state))


def _dryer_sub(status: int, target: int, duration: int, remain: int) -> bytes:
    return pb_string(2, (pb_uint32(1, status) + pb_uint32(2, target)
                         + pb_uint32(3, duration) + pb_uint32(4, remain)))


class TestDecodeStatus:
    def test_busy_slot_identified_and_dryer(self) -> None:
        # slot0: feeding(1) + identified(2); slot1: ready(0) + empty(0).
        payload = (
            _slot_sub(1, 2) + _slot_sub(0, 0)
            + _dryer_sub(2, 60, 3600, 1800)
            + pb_uint32(3, 25) + pb_uint32(4, 40) + pb_uint32(5, 1)
            + pb_uint32(7, 9) + pb_uint32(8, 12))
        ret = v2_response_to_v1(Cmd.GET_STATUS, 4, payload)
        res = ret['result']
        # feeding slot -> overall busy
        assert res['status'] == 'busy'
        assert len(res['slots']) == 4
        # slot0: identified filament -> present + rfid=2 + slot_status feeding
        assert res['slots'][0]['status'] == 'ready'
        assert res['slots'][0]['rfid'] == 2
        assert res['slots'][0]['slot_status'] == 'feeding'
        # slot1: empty filament -> status empty, rfid 0
        assert res['slots'][1]['status'] == 'empty'
        assert res['slots'][1]['rfid'] == 0
        # padded slots 2 and 3
        assert res['slots'][2] == {
            'index': 2, 'status': 'empty', 'slot_status': 'unknown',
            'sku': '', 'type': '', 'rfid': 0, 'brand': '', 'color': [0, 0, 0]}
        assert res['dryer_status'] == {
            'status': 'keeping', 'target_temp': 60,
            'duration': 3600, 'remain_time': 1800}
        assert res['temp'] == 25
        assert res['humidity'] == 40
        assert res['enable_rfid'] == 1
        assert res['feed_assist_count'] == 9
        assert res['cont_assist_time'] == 12.0

    def test_ready_when_no_busy_slot_and_rfid_disabled(self) -> None:
        # Only a non-busy slot present; enable_rfid field absent -> 0.
        payload = _slot_sub(0, 1)  # ready + unknown filament
        ret = v2_response_to_v1(Cmd.GET_STATUS, 4, payload)
        res = ret['result']
        assert res['status'] == 'ready'
        # 'unknown' filament state still counts as present.
        assert res['slots'][0]['status'] == 'ready'
        assert res['slots'][0]['rfid'] == 0
        assert res['enable_rfid'] == 0
        assert res['dryer_status'] == {
            'status': 'stop', 'target_temp': 0, 'duration': 0, 'remain_time': 0}

    def test_non_message_slot_and_dryer_entries_skipped(self) -> None:
        # field 9 / field 2 arriving as varints (wtype 0) are skipped, leaving
        # all-padded slots and the default dryer status.
        payload = pb_uint32(9, 5) + pb_uint32(2, 5)
        ret = v2_response_to_v1(Cmd.GET_STATUS, 4, payload)
        res = ret['result']
        assert [s['slot_status'] for s in res['slots']] == ['unknown'] * 4
        assert res['dryer_status']['status'] == 'stop'
        assert res['status'] == 'ready'


# ── v2_response_to_v1 (opcode decoders not covered by sibling files) ──────────

class TestV2ResponseToV1:
    def test_empty_payload_short_circuits(self) -> None:
        ret = v2_response_to_v1(Cmd.GET_INFO, 3, b'')
        assert ret == {'id': 3, '_cmd': Cmd.GET_INFO, 'code': 0,
                       'msg': 'success', 'result': {}}

    def test_decode_failure_is_logged_and_returns_default(self) -> None:
        logger = RecordingLogger()
        # A non-bytes payload is truthy but makes pb_decode raise; the guard
        # logs and returns the empty default rather than propagating.
        bad = object()
        ret = v2_response_to_v1(Cmd.GET_INFO, 5, bad, logger)
        assert ret['result'] == {}
        assert len(logger.messages) == 1
        level, msg = logger.messages[0]
        assert level == "debug"
        assert msg.startswith("ACE2 protobuf decode failure cmd=%d" % Cmd.GET_INFO)

    def test_discover_device(self) -> None:
        payload = pb_uint32(1, 11) + pb_uint32(2, 22) + pb_uint32(3, 33)
        ret = v2_response_to_v1(Cmd.DISCOVER_DEVICE, 1, payload)
        assert ret['result'] == {'uid1': 11, 'uid2': 22, 'uid3': 33}

    def test_get_info(self) -> None:
        payload = pb_string(1, "v1.1.31") + pb_string(2, "boot9")
        ret = v2_response_to_v1(Cmd.GET_INFO, 1, payload)
        assert ret['result'] == {
            'model': 'ACE 2 Pro', 'firmware': 'v1.1.31', 'boot_version': 'boot9'}

    def test_get_feed_info(self) -> None:
        sub = pb_uint32(1, 100) + pb_uint32(2, 200) + pb_uint32(3, 300)
        # A non-message field-1 entry (wtype 0) must be skipped.
        payload = pb_string(1, sub) + pb_uint32(1, 5) + pb_uint32(4, 0)
        ret = v2_response_to_v1(Cmd.GET_FEED_INFO, 1, payload)
        assert ret['result']['feed_info'] == [
            {'steps': 100, 'length': 200, 'decoder': 300}]
        assert ret['result']['raw_fields'] == [1, 4]

    def test_mfrc522_reg_read_masks_low_byte(self) -> None:
        ret = v2_response_to_v1(Cmd.MFRC522_REG_READ, 1, pb_uint32(1, 0x1FF))
        assert ret['result'] == {'val': 0xFF}

    def test_filament_info_full(self) -> None:
        color_sub = pb_string(5, pb_uint32(1, 0x11223344))
        nozzle = pb_string(6, pb_uint32(1, 190) + pb_uint32(2, 230))
        bed = pb_string(7, pb_uint32(1, 55) + pb_uint32(2, 65))
        payload = (pb_uint32(1, 2) + pb_string(3, "SKU9") + pb_string(4, "PLA")
                   + color_sub + pb_uint32(8, 175) + pb_uint32(11, 330000)
                   + nozzle + bed)
        ret = v2_response_to_v1(Cmd.GET_FILAMENT_INFO, 1, payload)
        res = ret['result']
        assert res['index'] == 2
        assert res['sku'] == "SKU9"
        assert res['type'] == "PLA"
        # ftype present -> rfid 2
        assert res['rfid'] == 2
        assert res['color'] == [0x11, 0x22, 0x33]
        assert res['diameter'] == 175 / 100.0
        assert res['total_length'] == 330000
        assert res['extruder_temp'] == {'min': 190, 'max': 230}
        assert res['hotbed_temp'] == {'min': 55, 'max': 65}

    def test_filament_identify_no_type_no_color(self) -> None:
        # No field 4 (type) -> rfid 0; a non-message color entry (wtype 0) is
        # skipped so color stays default; temp ranges of all-zero -> {}.
        payload = pb_uint32(1, 0) + pb_uint32(5, 9) + pb_string(6, pb_uint32(1, 0))
        ret = v2_response_to_v1(Cmd.FILAMENT_IDENTIFY, 1, payload)
        res = ret['result']
        assert res['type'] == ""
        assert res['rfid'] == 0
        assert res['color'] == [0, 0, 0]
        assert res['extruder_temp'] == {}
        assert res['hotbed_temp'] == {}

    def test_filament_info_temp_range_non_message_skipped(self) -> None:
        # field 6 as a varint (wtype 0) is skipped -> extruder_temp stays {}.
        payload = pb_string(4, "PETG") + pb_uint32(6, 5)
        ret = v2_response_to_v1(Cmd.GET_FILAMENT_INFO, 1, payload)
        assert ret['result']['extruder_temp'] == {}

    def test_material_info_non_message_name_entry_skipped(self) -> None:
        # field 2 present as a varint (wtype 0) -> name loop skips it, name ''.
        payload = pb_uint32(1, 1) + pb_uint32(2, 7)
        ret = v2_response_to_v1(Cmd.GET_MATERIAL_INFO, 1, payload)
        assert ret['result']['material_name'] == ""
        assert ret['result']['index'] == 1


# ── decode_frames ─────────────────────────────────────────────────────────────

class TestDecodeFrames:
    def test_short_buffer_returns_empty(self) -> None:
        buf = bytearray(b'\x00' * (MIN_FRAME_LEN - 1))
        assert decode_frames(buf) == []
        assert len(buf) == MIN_FRAME_LEN - 1  # untouched

    def test_no_preamble_ending_in_ff_keeps_last_byte(self) -> None:
        buf = bytearray(b'\x11' * 9 + b'\xff')
        assert decode_frames(buf) == []
        assert bytes(buf) == b'\xff'

    def test_no_preamble_clears_buffer(self) -> None:
        buf = bytearray(b'\x11' * 10)
        assert decode_frames(buf) == []
        assert bytes(buf) == b''

    def test_leading_garbage_before_preamble_is_dropped(self) -> None:
        frame = _build_frame(FLAG_RESPONSE, 3, Cmd.GET_TEMP, pb_uint32(1, 20))
        buf = bytearray(b'\x00\x00' + frame)
        results = decode_frames(buf)
        assert len(results) == 1
        assert results[0]['id'] == 3
        assert bytes(buf) == b''

    def test_preamble_then_too_short_header_breaks(self) -> None:
        # 8 garbage bytes then a bare preamble: after resync <HEADER_LEN remains.
        buf = bytearray(b'\x00' * 8 + PREAMBLE)
        assert decode_frames(buf) == []
        assert bytes(buf) == bytes(PREAMBLE)

    def test_oversize_payload_len_resyncs(self) -> None:
        # payload_len byte (index 6) > MAX_PAYLOAD_LEN -> drop 2 bytes, resync.
        buf = bytearray(bytes(PREAMBLE)
                        + bytes([FLAG_RESPONSE, 0, 0, Cmd.GET_TEMP,
                                 MAX_PAYLOAD_LEN + 1])
                        + b'\x00' * 3)
        assert decode_frames(buf) == []

    def test_incomplete_frame_is_retained(self) -> None:
        # Header claims a 5-byte payload (total 15) but only 10 bytes present.
        header = bytes(PREAMBLE) + bytes([FLAG_RESPONSE, 0, 0, Cmd.GET_TEMP, 5])
        buf = bytearray(header + b'\x00' * 3)
        assert decode_frames(buf) == []
        assert bytes(buf) == bytes(header + b'\x00' * 3)  # kept for more bytes

    def test_bad_end_marker_resyncs(self) -> None:
        frame = bytearray(_build_frame(
            FLAG_RESPONSE, 3, Cmd.GET_TEMP, pb_uint32(1, 20)))
        frame[-1] = 0x00  # corrupt END_MARKER
        assert decode_frames(bytearray(frame)) == []

    def test_crc_mismatch_is_dropped_and_logged(self) -> None:
        logger = RecordingLogger()
        payload = pb_uint32(1, 20)
        frame = bytearray(_build_frame(FLAG_RESPONSE, 3, Cmd.GET_TEMP, payload))
        frame[HEADER_LEN + len(payload)] ^= 0xFF  # corrupt CRC low byte
        results = decode_frames(bytearray(frame), logger)
        assert results == []
        assert logger.messages == [("debug", "ACE2 CRC mismatch, dropping frame")]

    def test_request_frame_is_skipped(self) -> None:
        # A well-formed REQUEST frame (no FLAG_RESPONSE) is consumed but not
        # surfaced as a response.
        frame = encode_request(3, "get_status", {})
        buf = bytearray(frame)
        assert decode_frames(buf) == []
        assert bytes(buf) == b''

    def test_valid_response_frame_decoded(self) -> None:
        payload = pb_uint32(1, 21)
        buf = bytearray(_build_frame(FLAG_RESPONSE, 7, Cmd.GET_TEMP, payload))
        results = decode_frames(buf)
        assert len(results) == 1
        assert results[0] == v2_response_to_v1(Cmd.GET_TEMP, 7, payload)
        assert bytes(buf) == b''

    def test_two_frames_decoded_in_order(self) -> None:
        f1 = _build_frame(FLAG_RESPONSE, 1, Cmd.GET_TEMP, pb_uint32(1, 10))
        f2 = _build_frame(FLAG_RESPONSE, 2, Cmd.GET_TEMP, pb_uint32(1, 20))
        results = decode_frames(bytearray(f1 + f2))
        assert [r['id'] for r in results] == [1, 2]


# ── ACE2Connection.send_command ───────────────────────────────────────────────

class TestSendCommand:
    def test_not_connected_flag_raises(self) -> None:
        conn = _make_conn()
        conn._connected = False  # A alone true (serial still present)
        with pytest.raises(ACESerialError, match="not connected"):
            conn.send_command("get_status")
        assert conn._serial.frames == []

    def test_serial_none_raises(self) -> None:
        conn = _make_conn()
        conn._serial = None  # B alone true (connected still True)
        with pytest.raises(ACESerialError, match="not connected"):
            conn.send_command("get_status")

    def test_encode_failure_raises_serial_error(self) -> None:
        conn = _make_conn(logger=RecordingLogger())
        # A >100-byte material name overflows MAX_PAYLOAD_LEN in encode_frame.
        with pytest.raises(ACESerialError, match="encode failed"):
            conn.send_command("set_material_name", {"name": "x" * 200})
        assert conn._serial.frames == []
        assert conn._logger.messages == []

    def test_write_failure_reconnects_and_raises(self) -> None:
        conn = _make_conn(logger=RecordingLogger())
        conn._serial = FakeSerial(write_error=OSError("cable"))
        with pytest.raises(ACESerialError, match="write failed"):
            conn.send_command("get_status")
        # pending maps cleared, timeout tracked, TX debug never logged.
        assert conn._pending == {} and conn._pending_cmd == {}
        assert len(conn._timeout_timestamps) == 1
        assert conn._logger.messages == []

    def test_timeout_raises_and_tracks(self) -> None:
        conn = _make_conn(next_id=5, logger=RecordingLogger())
        # No echo -> completion.wait returns None -> timeout.
        with pytest.raises(ACETimeoutError, match="timed out"):
            conn.send_command("get_status", timeout=0.0)
        assert len(conn._timeout_timestamps) == 1
        assert conn._pending == {} and conn._pending_cmd == {}
        assert conn._logger.messages == [
            ("debug", "ACE2 TX: id=5 get_status {}")]

    def test_success_returns_result_and_logs_tx(self) -> None:
        conn = _make_conn(next_id=5, logger=RecordingLogger())

        def _echo(frame: bytes) -> None:
            rid = _wire_seq(frame)
            conn._handle_response(
                {"id": rid, "_cmd": Cmd.GET_STATUS, "code": 0,
                 "result": {"ok": 1}})

        conn._serial.on_write = _echo
        assert conn.send_command("get_status") == {"ok": 1}
        assert conn._logger.messages == [
            ("debug", "ACE2 TX: id=5 get_status {}")]

    def test_error_code_raises(self) -> None:
        conn = _make_conn(next_id=5)

        def _echo(frame: bytes) -> None:
            rid = _wire_seq(frame)
            conn._handle_response(
                {"id": rid, "_cmd": Cmd.GET_STATUS, "code": 2,
                 "msg": "error_2", "result": {}})

        conn._serial.on_write = _echo
        with pytest.raises(ACESerialError, match="code=2, msg=error_2"):
            conn.send_command("get_status")

    def test_non_dict_result_returned_verbatim(self) -> None:
        conn = _make_conn(next_id=5)

        def _echo(frame: bytes) -> None:
            rid = _wire_seq(frame)
            conn._pending[rid].complete(4242)  # non-dict completion value

        conn._serial.on_write = _echo
        assert conn.send_command("get_status") == 4242


# ── ACE2Connection.send_command_async ─────────────────────────────────────────

class TestSendCommandAsync:
    def test_not_connected_flag_returns_early(self) -> None:
        conn = _make_conn()
        conn._connected = False
        conn.send_command_async("get_status")
        assert conn._serial.frames == []
        assert list(conn._async_ids) == []

    def test_serial_none_returns_early(self) -> None:
        conn = _make_conn()
        conn._serial = None
        conn.send_command_async("get_status")  # must not raise
        assert list(conn._async_ids) == []

    def test_success_writes_frame_and_tracks_id(self) -> None:
        conn = _make_conn(next_id=0, logger=RecordingLogger())
        conn.send_command_async("get_status")
        assert conn._serial.frames == [encode_request(0, "get_status", {})]
        assert list(conn._async_ids) == [0]
        assert conn._logger.messages == [
            ("debug", "ACE2 TX (async): id=0 get_status")]

    def test_encode_failure_logged_and_swallowed(self) -> None:
        conn = _make_conn(next_id=0, logger=RecordingLogger())
        conn.send_command_async("set_material_name", {"name": "x" * 200})
        assert conn._serial.frames == []
        assert len(conn._logger.messages) == 1
        level, msg = conn._logger.messages[0]
        assert level == "debug"
        assert msg.startswith("ACE2 async encode failed:")

    def test_write_failure_reconnects_and_logs(self) -> None:
        conn = _make_conn(next_id=0, logger=RecordingLogger())
        conn._serial = FakeSerial(write_error=OSError("cable"))
        conn.send_command_async("get_status")  # must not raise
        assert len(conn._logger.messages) == 1
        level, msg = conn._logger.messages[0]
        assert level == "debug"
        assert msg.startswith("ACE2 async write failed:")


# ── ACE2Connection._response_matches_pending ──────────────────────────────────

class TestResponseMatchesPending:
    def test_no_recorded_opcode_accepts(self) -> None:
        conn = _make_conn()
        # id not in _pending_cmd -> expected None -> accept.
        assert conn._response_matches_pending(9, {"_cmd": 6}) is True

    def test_non_dict_response_accepts(self) -> None:
        conn = _make_conn()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, "not-a-dict") is True

    def test_dict_without_cmd_accepts(self) -> None:
        conn = _make_conn()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, {"code": 0}) is True

    def test_matching_opcode_accepts(self) -> None:
        conn = _make_conn()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, {"_cmd": 6}) is True

    def test_mismatched_opcode_rejected(self) -> None:
        conn = _make_conn()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, {"_cmd": 8}) is False


# ── ACE2Connection._parse_frames ──────────────────────────────────────────────

class TestParseFrames:
    def test_complete_frame_routed_and_buffer_consumed(self) -> None:
        conn = _make_conn(logger=RecordingLogger())
        payload = pb_uint32(1, 21)
        conn._pending[7] = FakeCompletion()
        conn._pending_cmd[7] = Cmd.GET_TEMP
        conn._read_buffer = _build_frame(FLAG_RESPONSE, 7, Cmd.GET_TEMP, payload)

        conn._parse_frames()

        expected = v2_response_to_v1(Cmd.GET_TEMP, 7, payload)
        assert conn._pending[7]._value == expected
        assert conn._read_buffer == b''
        assert conn._logger.messages == [("debug", f"ACE2 RX: {expected}")]

    def test_partial_frame_retained_in_buffer(self) -> None:
        conn = _make_conn(logger=RecordingLogger())
        # Header claims a 5-byte payload but the bytes aren't all present yet.
        header = bytes(PREAMBLE) + bytes([FLAG_RESPONSE, 0, 0, Cmd.GET_TEMP, 5])
        conn._read_buffer = header + b'\x00' * 3
        conn._parse_frames()
        assert conn._read_buffer == header + b'\x00' * 3
        assert conn._logger.messages == []


# ── ACE2Connection._pre_info_handshake ────────────────────────────────────────

class TestPreInfoHandshake:
    def test_sends_discover_device(self) -> None:
        conn = _make_conn()
        conn.send_command = Recorder(result={})
        conn._pre_info_handshake()
        assert conn.send_command.last_args == ("discover_device",)
        assert conn.send_command.last_kwargs == {"timeout": 3.0}

    def test_exception_is_swallowed_and_logged(self) -> None:
        conn = _make_conn(logger=RecordingLogger())
        conn.send_command = Recorder(raises=RuntimeError("no reply"))
        conn._pre_info_handshake()  # must not raise
        assert len(conn._logger.messages) == 1
        level, msg = conn._logger.messages[0]
        assert level == "debug"
        assert msg == "ACE2 discover_device failed (non-fatal): no reply"


# ── ACE2Connection._poll_extras ───────────────────────────────────────────────

class TestPollExtras:
    def test_polls_temp_and_sensor_state(self) -> None:
        conn = _make_conn()
        conn.send_command_async = Recorder()
        conn._poll_extras()
        assert [c[0] for c in conn.send_command_async.calls] == [
            ("get_temp",), ("get_sensor_state",)]


# ── ACE2Connection.enable_rfid / disable_rfid ─────────────────────────────────

class TestEnableRfid:
    def test_enables_every_slot(self) -> None:
        conn = _make_conn(next_id=0)
        conn.enable_rfid()
        expected = [
            encode_request(i, "set_rfid_enable", {"index": i, "enable": True})
            for i in range(conn.slot_count)]
        assert conn._serial.frames == expected
        assert list(conn._async_ids) == list(range(conn.slot_count))


class TestDisableRfid:
    def test_disables_every_slot(self) -> None:
        conn = _make_conn(next_id=0)
        conn.disable_rfid()
        expected = [
            encode_request(i, "set_rfid_enable", {"index": i, "enable": False})
            for i in range(conn.slot_count)]
        assert conn._serial.frames == expected
        assert list(conn._async_ids) == list(range(conn.slot_count))


# ── afcACE2._apply_feed_check ─────────────────────────────────────────────────

def _feed_check_unit() -> afcACE2:
    unit = afcACE2.__new__(afcACE2)
    unit.name = "ace2"
    unit.logger = FakeLogger()
    unit.feed_check_length = 200
    unit.feed_error_length = 185
    return unit


class TestApplyFeedCheck:
    def test_no_ace_returns_without_send(self) -> None:
        unit = _feed_check_unit()
        unit._ace = None
        unit._apply_feed_check()
        assert unit.logger.lines["info"] == []
        assert unit.logger.lines["warning"] == []

    def test_success_sends_and_logs_info(self) -> None:
        unit = _feed_check_unit()
        unit._ace = _AceStub()
        unit._apply_feed_check()
        assert unit._ace.send_command_async.last_args == (
            "set_feed_check", {"check_length": 200, "error_length": 185})
        assert unit.logger.lines["info"] == [
            "ACE2 ace2: feed check set check_length=200 error_length=185"]

    def test_send_failure_logs_warning(self) -> None:
        unit = _feed_check_unit()
        unit._ace = _AceStub(send_raises=RuntimeError("boom"))
        unit._apply_feed_check()
        assert unit.logger.lines["info"] == []
        assert unit.logger.lines["warning"] == [
            "ACE2 ace2: set_feed_check failed (non-fatal): boom"]


class _AceStub:
    def __init__(self, send_raises: Optional[Exception] = None) -> None:
        self.send_command_async = Recorder(raises=send_raises)


# ── afcACE2._handle_encoder_jam ───────────────────────────────────────────────

def _jam_unit(afc_error_raises: Optional[Exception] = None,
              stop_raises: Optional[Exception] = None) -> afcACE2:
    unit = afcACE2.__new__(afcACE2)
    unit.name = "ace2"
    unit.logger = FakeLogger()
    unit.gcode = _GcodeStub()
    unit._stop_feed_assist = Recorder(raises=stop_raises)
    unit.afc = FakeAFC()
    unit.afc.error.AFC_error = Recorder(raises=afc_error_raises)
    return unit


class _GcodeStub:
    def __init__(self) -> None:
        self.run_script_from_command = Recorder()


def _expected_jam_msg(lane: str, what: str) -> str:
    return (
        f"ACE2 ace2 lane {lane}: {what}. The unit's filament encoder reports "
        "the spool is not moving with the motor — likely a jam or tangle at "
        "the unit. Clear the snag, then resume. Run ACE_STUCK_SPOOL_DETECTION "
        "ENABLE=0 to disable this check.")


class TestHandleEncoderJam:
    def test_stuck_error_stops_assist_and_pauses_via_afc(self) -> None:
        unit = _jam_unit()
        unit._handle_encoder_jam("lane0", 0, "stuck_error")
        assert unit._stop_feed_assist.last_args == (0,)
        what = "stuck spool (encoder saw no movement while feeding)"
        assert unit.afc.error.AFC_error.last_args == (
            _expected_jam_msg("lane0", what),)
        assert unit.afc.error.AFC_error.last_kwargs == {"pause": True}
        # AFC path succeeded -> no fallback logging / gcode PAUSE.
        assert unit.logger.lines["error"] == []
        assert not unit.gcode.run_script_from_command.called

    @pytest.mark.parametrize("state,what", [
        ("tangled_error", "tangled spool"),
        ("assist_error", "feed-assist slip (encoder fell behind the motor)"),
        ("motor_error", "motor error"),
        ("weird_error", "weird_error"),  # unknown -> raw state passthrough
    ])
    def test_pretty_message_per_state(self, state: str, what: str) -> None:
        unit = _jam_unit()
        unit._handle_encoder_jam("lane1", 1, state)
        assert unit.afc.error.AFC_error.last_args == (
            _expected_jam_msg("lane1", what),)

    def test_stop_assist_exception_is_swallowed(self) -> None:
        unit = _jam_unit(stop_raises=RuntimeError("motor busy"))
        unit._handle_encoder_jam("lane0", 0, "stuck_error")
        # Despite the stop failing, the pause path still ran.
        assert unit.afc.error.AFC_error.called

    def test_afc_error_failure_falls_back_to_gcode_pause(self) -> None:
        unit = _jam_unit(afc_error_raises=RuntimeError("afc down"))
        unit._handle_encoder_jam("lane0", 0, "stuck_error")
        what = "stuck spool (encoder saw no movement while feeding)"
        assert unit.logger.lines["error"] == [_expected_jam_msg("lane0", what)]
        assert unit.gcode.run_script_from_command.last_args == ("PAUSE",)


# ── afcACE2._make_connection ──────────────────────────────────────────────────

class TestMakeConnection:
    def test_builds_ace2_connection(self) -> None:
        unit = afcACE2.__new__(afcACE2)
        conn = unit._make_connection(
            FakeReactor(), "/dev/ttyACE2", RecordingLogger(), 230400)
        assert isinstance(conn, ACE2Connection)
        assert conn._serial_port == "/dev/ttyACE2"
        assert conn._baud_rate == 230400


# ── afcACE2._reader_sibling_slot ──────────────────────────────────────────────

class TestReaderSiblingSlot:
    def test_pairs_within_range(self) -> None:
        unit = afcACE2.__new__(afcACE2)
        assert unit._reader_sibling_slot(0) == 1
        assert unit._reader_sibling_slot(1) == 0
        assert unit._reader_sibling_slot(2) == 3
        assert unit._reader_sibling_slot(3) == 2

    def test_out_of_range_sibling_returns_none(self) -> None:
        unit = afcACE2.__new__(afcACE2)
        # slot 100 -> sibling 101, outside [0, SLOTS_PER_UNIT) -> None.
        assert unit._reader_sibling_slot(100) is None


# ── module constant sanity (used by the encoders under test) ──────────────────

def test_encoder_scale_constant() -> None:
    assert ACE2_ENCODER_SCALE == pytest.approx(1.2342)
