"""
Tests for the ACE 2 Pro unit, extras/AFC_ACE2.py.

Init and wire identification, the raw frame path, buffer handling, material
reporting, the stuck-lane case, and a branch-coverage sweep over the rest.
Consolidated from seven files. A module-level helper two files both defined
carries its old file's tag, because those were different implementations that
happened to share a name. Section banners name the file each block came from.
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
import types
import extras.AFC_ACE2 as ace2mod
from extras.AFC_ACE2 import afcACE2, ACE2_ENCODER_SCALE
import collections
from extras.AFC_ACE2 import ACE2Connection, encode_request
from extras.AFC_ACE2 import PREAMBLE, FLAG_REQUEST
from extras.AFC_ACE import ACETimeoutError
from extras.AFC_ACE2 import (
    Cmd,
    encode_frame,
    encode_request,
    v2_response_to_v1,
    dump_fields,
    pb_decode,
    pb_uint32,
    crc16_kermit,
    PREAMBLE,
    END_MARKER,
    FLAG_REQUEST,
    MAX_PAYLOAD_LEN,
)
from extras.AFC_ACE2 import v2_response_to_v1, pb_uint32, Cmd
from extras.AFC_ACE import _derive_buffer_state
from extras.AFC_ACE2 import (
    Cmd,
    method_to_v2,
    v2_response_to_v1,
    pb_string,
    pb_uint32,
    pb_decode,
)
from extras.AFC_ACE2 import afcACE2


# ── Branch-coverage tests for extras/AFC_ACE2.py (the ACE 2 Pro V2 serial ─────
#
# was tests/test_AFC_ACE2_coverage.py
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


def _make_conn_coverage(next_id: int = 0,
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


def _wire_seq_coverage(frame: bytes) -> int:
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


def _pb_float_coverage(field: int, value: float) -> bytes:
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
        fields = pb_decode(_pb_float_coverage(3, 1.5))
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
        conn = _make_conn_coverage()
        conn._connected = False  # A alone true (serial still present)
        with pytest.raises(ACESerialError, match="not connected"):
            conn.send_command("get_status")
        assert conn._serial.frames == []

    def test_serial_none_raises(self) -> None:
        conn = _make_conn_coverage()
        conn._serial = None  # B alone true (connected still True)
        with pytest.raises(ACESerialError, match="not connected"):
            conn.send_command("get_status")

    def test_encode_failure_raises_serial_error(self) -> None:
        conn = _make_conn_coverage(logger=RecordingLogger())
        # A >100-byte material name overflows MAX_PAYLOAD_LEN in encode_frame.
        with pytest.raises(ACESerialError, match="encode failed"):
            conn.send_command("set_material_name", {"name": "x" * 200})
        assert conn._serial.frames == []
        assert conn._logger.messages == []

    def test_write_failure_reconnects_and_raises(self) -> None:
        conn = _make_conn_coverage(logger=RecordingLogger())
        conn._serial = FakeSerial(write_error=OSError("cable"))
        with pytest.raises(ACESerialError, match="write failed"):
            conn.send_command("get_status")
        # pending maps cleared, timeout tracked, TX debug never logged.
        assert conn._pending == {} and conn._pending_cmd == {}
        assert len(conn._timeout_timestamps) == 1
        assert conn._logger.messages == []

    def test_timeout_raises_and_tracks(self) -> None:
        conn = _make_conn_coverage(next_id=5, logger=RecordingLogger())
        # No echo -> completion.wait returns None -> timeout.
        with pytest.raises(ACETimeoutError, match="timed out"):
            conn.send_command("get_status", timeout=0.0)
        assert len(conn._timeout_timestamps) == 1
        assert conn._pending == {} and conn._pending_cmd == {}
        assert conn._logger.messages == [
            ("debug", "ACE2 TX: id=5 get_status {}")]

    def test_success_returns_result_and_logs_tx(self) -> None:
        conn = _make_conn_coverage(next_id=5, logger=RecordingLogger())

        def _echo(frame: bytes) -> None:
            rid = _wire_seq_coverage(frame)
            conn._handle_response(
                {"id": rid, "_cmd": Cmd.GET_STATUS, "code": 0,
                 "result": {"ok": 1}})

        conn._serial.on_write = _echo
        assert conn.send_command("get_status") == {"ok": 1}
        assert conn._logger.messages == [
            ("debug", "ACE2 TX: id=5 get_status {}")]

    def test_error_code_raises(self) -> None:
        conn = _make_conn_coverage(next_id=5)

        def _echo(frame: bytes) -> None:
            rid = _wire_seq_coverage(frame)
            conn._handle_response(
                {"id": rid, "_cmd": Cmd.GET_STATUS, "code": 2,
                 "msg": "error_2", "result": {}})

        conn._serial.on_write = _echo
        with pytest.raises(ACESerialError, match="code=2, msg=error_2"):
            conn.send_command("get_status")

    def test_non_dict_result_returned_verbatim(self) -> None:
        conn = _make_conn_coverage(next_id=5)

        def _echo(frame: bytes) -> None:
            rid = _wire_seq_coverage(frame)
            conn._pending[rid].complete(4242)  # non-dict completion value

        conn._serial.on_write = _echo
        assert conn.send_command("get_status") == 4242


# ── ACE2Connection.send_command_async ─────────────────────────────────────────

class TestSendCommandAsync:
    def test_not_connected_flag_returns_early(self) -> None:
        conn = _make_conn_coverage()
        conn._connected = False
        conn.send_command_async("get_status")
        assert conn._serial.frames == []
        assert list(conn._async_ids) == []

    def test_serial_none_returns_early(self) -> None:
        conn = _make_conn_coverage()
        conn._serial = None
        conn.send_command_async("get_status")  # must not raise
        assert list(conn._async_ids) == []

    def test_success_writes_frame_and_tracks_id(self) -> None:
        conn = _make_conn_coverage(next_id=0, logger=RecordingLogger())
        conn.send_command_async("get_status")
        assert conn._serial.frames == [encode_request(0, "get_status", {})]
        assert list(conn._async_ids) == [0]
        assert conn._logger.messages == [
            ("debug", "ACE2 TX (async): id=0 get_status")]

    def test_encode_failure_logged_and_swallowed(self) -> None:
        conn = _make_conn_coverage(next_id=0, logger=RecordingLogger())
        conn.send_command_async("set_material_name", {"name": "x" * 200})
        assert conn._serial.frames == []
        assert len(conn._logger.messages) == 1
        level, msg = conn._logger.messages[0]
        assert level == "debug"
        assert msg.startswith("ACE2 async encode failed:")

    def test_write_failure_reconnects_and_logs(self) -> None:
        conn = _make_conn_coverage(next_id=0, logger=RecordingLogger())
        conn._serial = FakeSerial(write_error=OSError("cable"))
        conn.send_command_async("get_status")  # must not raise
        assert len(conn._logger.messages) == 1
        level, msg = conn._logger.messages[0]
        assert level == "debug"
        assert msg.startswith("ACE2 async write failed:")


# ── ACE2Connection._response_matches_pending ──────────────────────────────────

class TestResponseMatchesPending:
    def test_no_recorded_opcode_accepts(self) -> None:
        conn = _make_conn_coverage()
        # id not in _pending_cmd -> expected None -> accept.
        assert conn._response_matches_pending(9, {"_cmd": 6}) is True

    def test_non_dict_response_accepts(self) -> None:
        conn = _make_conn_coverage()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, "not-a-dict") is True

    def test_dict_without_cmd_accepts(self) -> None:
        conn = _make_conn_coverage()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, {"code": 0}) is True

    def test_matching_opcode_accepts(self) -> None:
        conn = _make_conn_coverage()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, {"_cmd": 6}) is True

    def test_mismatched_opcode_rejected(self) -> None:
        conn = _make_conn_coverage()
        conn._pending_cmd[9] = 6
        assert conn._response_matches_pending(9, {"_cmd": 8}) is False


# ── ACE2Connection._parse_frames ──────────────────────────────────────────────

class TestParseFrames:
    def test_complete_frame_routed_and_buffer_consumed(self) -> None:
        conn = _make_conn_coverage(logger=RecordingLogger())
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
        conn = _make_conn_coverage(logger=RecordingLogger())
        # Header claims a 5-byte payload but the bytes aren't all present yet.
        header = bytes(PREAMBLE) + bytes([FLAG_RESPONSE, 0, 0, Cmd.GET_TEMP, 5])
        conn._read_buffer = header + b'\x00' * 3
        conn._parse_frames()
        assert conn._read_buffer == header + b'\x00' * 3
        assert conn._logger.messages == []


# ── ACE2Connection._pre_info_handshake ────────────────────────────────────────

class TestPreInfoHandshake:
    def test_sends_discover_device(self) -> None:
        conn = _make_conn_coverage()
        conn.send_command = Recorder(result={})
        conn._pre_info_handshake()
        assert conn.send_command.last_args == ("discover_device",)
        assert conn.send_command.last_kwargs == {"timeout": 3.0}

    def test_exception_is_swallowed_and_logged(self) -> None:
        conn = _make_conn_coverage(logger=RecordingLogger())
        conn.send_command = Recorder(raises=RuntimeError("no reply"))
        conn._pre_info_handshake()  # must not raise
        assert len(conn._logger.messages) == 1
        level, msg = conn._logger.messages[0]
        assert level == "debug"
        assert msg == "ACE2 discover_device failed (non-fatal): no reply"


# ── ACE2Connection._poll_extras ───────────────────────────────────────────────

class TestPollExtras:
    def test_polls_temp_and_sensor_state(self) -> None:
        conn = _make_conn_coverage()
        conn.send_command_async = Recorder()
        conn._poll_extras()
        assert [c[0] for c in conn.send_command_async.calls] == [
            ("get_temp",), ("get_sensor_state",)]


# ── ACE2Connection.enable_rfid / disable_rfid ─────────────────────────────────

class TestEnableRfid:
    def test_enables_every_slot(self) -> None:
        conn = _make_conn_coverage(next_id=0)
        conn.enable_rfid()
        expected = [
            encode_request(i, "set_rfid_enable", {"index": i, "enable": True})
            for i in range(conn.slot_count)]
        assert conn._serial.frames == expected
        assert list(conn._async_ids) == list(range(conn.slot_count))


class TestDisableRfid:
    def test_disables_every_slot(self) -> None:
        conn = _make_conn_coverage(next_id=0)
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


# ── init ──────────────────────────────────────────────────────────────────────
#
# was tests/test_AFC_ACE2_init.py
# Construction tests for extras/AFC_ACE2.py.
#
# The ACE 2 Pro inherits nearly everything from the V1 ACE and overrides a
# handful of values that are wrong for the newer hardware. Those overrides are
# the whole point of the subclass, and each one is a field failure if it
# regresses:
#
#   * 230400 baud. At the V1's 115200 the unit never sees a valid frame and
#     never replies -- it looks dead rather than misconfigured.
#   * the encoder feed-check window, where an unreachable threshold makes EVERY
#     feed raise FEED_ERROR.
#   * stuck-spool detection defaulting ON, because unlike the V1 the ACE2 has a
#     real encoder and reports a true mechanical-jam state.
#
# The parent's __init__ needs the whole Klipper/AFC stack, so it is stubbed:
# what is under test here is the subclass's own configuration.





class _ConfigError(Exception):
    pass


class _Config:
    def __init__(self, **opts):
        self._o = opts
        self.error = _ConfigError

    def get_name(self):
        return "AFC_ACE2 Ace2_1"

    def get(self, key, default=None):
        return self._o.get(key, default)

    def getint(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return int(v) if v is not None else None

    def getfloat(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return float(v) if v is not None else None

    def getboolean(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return default if v is None else bool(v)


@pytest.fixture(autouse=True)
def _stub_parent(monkeypatch):
    """Neutralise afcACE.__init__ -- the parent needs a printer, a serial port
    and lanes; the subclass's own configuration does not."""
    monkeypatch.setattr(ace2mod.afcACE, "__init__",
                        lambda self, config: None)


def _make(**opts):
    obj = afcACE2.__new__(afcACE2)
    afcACE2.__init__(obj, _Config(**opts))
    return obj


class TestDefaults:
    def test_type_defaults_to_ace2(self):
        assert _make().type == "ACE2"

    def test_type_can_be_overridden(self):
        assert _make(type="ACE2_custom").type == "ACE2_custom"

    def test_baud_defaults_to_230400_not_the_v1_115200(self):
        # The single most consequential override: at 115200 the ACE2 never
        # sees a valid frame and never answers, which reads as dead hardware.
        assert _make().baud_rate == 230400

    def test_baud_can_be_overridden(self):
        assert _make(baud_rate=115200).baud_rate == 115200

    def test_dryer_ceiling_is_70_not_the_v1_55(self):
        assert _make().max_dryer_temperature == 70.0

    def test_dryer_ceiling_can_be_overridden(self):
        assert _make(max_dryer_temperature=60).max_dryer_temperature == 60.0

    def test_stuck_detection_defaults_on_for_the_encoder_equipped_ace2(self):
        # The parent reads the same key defaulting False; the ACE2 has a real
        # encoder reporting a true jam state, so it defaults True here.
        assert _make()._stuck_detection is True

    def test_stuck_detection_can_be_disabled(self):
        assert _make(stuck_spool_detection=False)._stuck_detection is False

    def test_feed_check_window_defaults(self):
        obj = _make()
        assert (obj.feed_check_length, obj.feed_error_length) == (200, 185)


class TestFeedCheckValidation:
    """The encoder can only ever reach feed_error_length * 1.2342. A
    feed_check_length at or above that is unreachable, so EVERY feed would
    raise FEED_ERROR -- a misconfiguration that presents as broken hardware."""

    def test_the_default_pair_is_valid(self):
        _make()                       # must not raise

    def test_an_unreachable_threshold_is_rejected(self):
        # 185 * 1.2342 = 228.3; asking for 229 can never be satisfied.
        with pytest.raises(_ConfigError) as e:
            _make(feed_check_length=229, feed_error_length=185)
        assert "can never reach it" in str(e.value)

    def test_exactly_at_the_ceiling_is_rejected(self):
        err = int(100)
        unreachable = int(err * ACE2_ENCODER_SCALE)      # 123
        with pytest.raises(_ConfigError):
            _make(feed_check_length=unreachable + 1, feed_error_length=err)

    def test_just_under_the_ceiling_is_accepted(self):
        obj = _make(feed_check_length=120, feed_error_length=100)
        assert obj.feed_check_length == 120

    def test_the_message_names_the_section_and_both_numbers(self):
        with pytest.raises(_ConfigError) as e:
            _make(feed_check_length=250, feed_error_length=185)
        msg = str(e.value)
        assert "AFC_ACE2 Ace2_1" in msg and "250" in msg
        assert "tolerance_mm" in msg          # tells the operator what to change

    def test_lowering_feed_check_widens_tolerance_without_moving_the_check(self):
        # The documented tuning knob: same checkpoint, larger slip allowance.
        wide = _make(feed_check_length=100, feed_error_length=185)
        assert wide.feed_error_length == 185 and wide.feed_check_length == 100


class TestLoader:
    def test_load_config_prefix_builds_the_unit(self):
        obj = ace2mod.load_config_prefix(_Config())
        assert isinstance(obj, afcACE2)
        assert obj.baud_rate == 230400


# ── Regression test for the ACE2 16-bit wire-id wrap bug (extras/AFC_ACE2.py) ───
#
# was tests/test_AFC_ACE2_wire_id.py
WRAPPED_wire_id = 86480          # a real post-wrap counter value seen on hardware
MASKED_wire_id = WRAPPED_wire_id & 0xFFFF  # 20944


# ── Fakes ─────────────────────────────────────────────────────────────────────

class _FakeSerial:
    def __init__(self):
        self.frames = []
        self.on_write = None

    def write(self, frame):
        self.frames.append(bytes(frame))
        if self.on_write is not None:
            self.on_write(bytes(frame))

    def flush(self):
        pass


class _FakeCompletion:
    def __init__(self):
        self._value = None

    def complete(self, value):
        self._value = value

    def wait(self, deadline):
        return self._value


class _FakeReactor:
    def monotonic(self):
        return 0.0

    def completion(self):
        return _FakeCompletion()


class _FakeLogger:
    def __init__(self):
        self.debug_lines = []

    def debug(self, msg, *a, **k):
        self.debug_lines.append(msg)

    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


def _make_conn_wire_id(next_id):
    conn = ACE2Connection.__new__(ACE2Connection)
    conn._connected = True
    conn._serial = _FakeSerial()
    conn._reactor = _FakeReactor()
    conn._logger = _FakeLogger()
    conn._pending = {}
    conn._pending_cmd = {}
    conn._async_ids = collections.deque(maxlen=256)
    conn._next_request_id = next_id
    conn.status_callback = None
    conn._track_timeout = lambda: None
    conn._track_unsolicited = lambda: None
    return conn


def _wire_seq_wire_id(frame):
    """Extract the 16-bit sequence id from an encoded request frame."""
    inner = frame[len(PREAMBLE):]
    assert inner[0] == FLAG_REQUEST
    return inner[1] | (inner[2] << 8)


# ── The wire id itself ────────────────────────────────────────────────────────

def test_encode_masks_id_to_16_bits():
    frame = encode_request(WRAPPED_wire_id, "get_status", {})
    assert _wire_seq_wire_id(frame) == MASKED_wire_id


# ── send_command round trip after the counter wraps ───────────────────────────

def test_send_command_completes_after_wrap():
    conn = _make_conn_wire_id(next_id=WRAPPED_wire_id)

    def _echo(frame):
        # The unit replies with exactly the 16-bit id it received.
        rid = _wire_seq_wire_id(frame)
        conn._handle_response({"id": rid, "code": 0, "result": {"ok": 1}})

    conn._serial.on_write = _echo

    result = conn.send_command("get_status", timeout=1.0)

    # The pending completion matched the echoed (masked) id and completed.
    assert result == {"ok": 1}
    assert conn._pending == {}  # popped after completion
    assert _wire_seq_wire_id(conn._serial.frames[0]) == MASKED_wire_id


def test_reply_with_mismatched_opcode_is_dropped():
    # A stale reply landing on a reused 16-bit id but carrying a DIFFERENT
    # opcode must not complete the pending request (it would hand the caller
    # another command's data). It's dropped; the request then times out.
    conn = _make_conn_wire_id(next_id=WRAPPED_wire_id)

    def _wrong_opcode(frame):
        rid = _wire_seq_wire_id(frame)
        # get_status is opcode 6; reply tagged as opcode 8 (feed) must be rejected.
        conn._handle_response({"id": rid, "_cmd": 8, "code": 0,
                               "result": {"stale": 1}})

    conn._serial.on_write = _wrong_opcode
    with pytest.raises(ACETimeoutError):
        conn.send_command("get_status", timeout=0.0)
    assert conn._pending == {} and conn._pending_cmd == {}


def test_reply_with_matching_opcode_completes():
    conn = _make_conn_wire_id(next_id=WRAPPED_wire_id)

    def _right_opcode(frame):
        rid = _wire_seq_wire_id(frame)
        conn._handle_response({"id": rid, "_cmd": 6, "code": 0,
                               "result": {"ok": 1}})

    conn._serial.on_write = _right_opcode
    assert conn.send_command("get_status", timeout=1.0) == {"ok": 1}


def test_send_command_pending_keyed_by_masked_id():
    conn = _make_conn_wire_id(next_id=WRAPPED_wire_id)
    captured = {}

    def _capture(frame):
        captured["keys"] = list(conn._pending.keys())

    conn._serial.on_write = _capture
    # No echo -> the command times out; we only care that the pending key was
    # the masked id at write time.
    with pytest.raises(ACETimeoutError):
        conn.send_command("get_status", timeout=0.0)

    assert captured["keys"] == [MASKED_wire_id]  # NOT [86480]


# ── send_command_async round trip after wrap ──────────────────────────────────

def test_async_id_tracked_masked_and_recognised():
    conn = _make_conn_wire_id(next_id=WRAPPED_wire_id)

    conn.send_command_async("get_status")

    assert list(conn._async_ids) == [MASKED_wire_id]
    assert _wire_seq_wire_id(conn._serial.frames[0]) == MASKED_wire_id

    # The echoed reply with the masked id is recognised (removed from
    # _async_ids), NOT logged as an unknown request.
    conn._handle_response({"id": MASKED_wire_id, "code": 0, "result": {"status": "ready"}})

    assert list(conn._async_ids) == []
    assert not any("unknown request" in m for m in conn._logger.debug_lines)


def test_pre_wrap_ids_unaffected():
    """Below 65536 the masking is a no-op — behaviour is unchanged."""
    conn = _make_conn_wire_id(next_id=5)
    conn.send_command_async("get_status")
    assert list(conn._async_ids) == [5]
    assert _wire_seq_wire_id(conn._serial.frames[0]) == 5


# ── Unit tests for the ACE2 explicit-opcode frame builder and response decoding ───
#
# was tests/test_AFC_ACE2_raw.py
WRAPPED_raw = 86480            # a real post-wrap counter value seen on hardware
MASKED_raw = WRAPPED_raw & 0xFFFF  # 20944


# ── frame helpers ─────────────────────────────────────────────────────────────

def _split_request(frame):
    """Return (flags, seq, cmd, payload) from an encoded request/raw frame."""
    assert frame[:len(PREAMBLE)] == PREAMBLE
    assert frame[-1] == END_MARKER
    inner = frame[len(PREAMBLE):]
    flags = inner[0]
    seq = inner[1] | (inner[2] << 8)
    cmd = inner[3]
    payload_len = inner[4]
    payload = inner[5:5 + payload_len]
    # CRC covers flags..payload; the two bytes after payload are the CRC.
    crc_bytes = inner[5 + payload_len:5 + payload_len + 2]
    crc_in = crc_bytes[0] | (crc_bytes[1] << 8)
    assert crc_in == crc16_kermit(bytes(inner[:5 + payload_len]))
    return flags, seq, cmd, bytes(payload)


def _pb_float_raw(field, value):
    """Encode a protobuf fixed32 float field (wire type 5)."""
    return bytes([(field << 3) | 5]) + struct.pack('<f', value)


# ── encode_frame ──────────────────────────────────────────────────────────────

def test_encode_frame_structure():
    frame = encode_frame(7, Cmd.GET_TEMP, b'')
    flags, seq, cmd, payload = _split_request(frame)
    assert flags == FLAG_REQUEST
    assert seq == 7
    assert cmd == Cmd.GET_TEMP
    assert payload == b''


def test_encode_frame_embeds_payload():
    body = bytes([0x01, 0x02, 0x03])
    frame = encode_frame(9, 20, body)
    flags, seq, cmd, payload = _split_request(frame)
    assert cmd == 20
    assert payload == body


def test_encode_frame_masks_seq_to_16_bits():
    frame = encode_frame(WRAPPED_raw, Cmd.GET_STATUS, b'')
    _flags, seq, _cmd, _payload = _split_request(frame)
    assert seq == MASKED_raw


def test_encode_frame_rejects_oversized_payload():
    with pytest.raises(ValueError):
        encode_frame(1, 20, b'\x00' * (MAX_PAYLOAD_LEN + 1))


def test_encode_request_delegates_to_encode_frame():
    # get_temp maps to (GET_TEMP, b'') so both builders must agree byte-for-byte.
    assert encode_request(42, "get_temp", {}) == encode_frame(42, Cmd.GET_TEMP, b'')
    # get_info likewise.
    assert encode_request(42, "get_info", {}) == encode_frame(42, Cmd.GET_INFO, b'')


# ── v2_response_to_v1: GET_TEMP channel mapping ───────────────────────────────

def test_get_temp_decode_maps_all_channels_varint():
    payload = b''.join(
        pb_uint32(i, v) for i, v in (
            (1, 21), (2, 22), (3, 23), (4, 24), (5, 25), (6, 40)))
    ret = v2_response_to_v1(Cmd.GET_TEMP, 3, payload)
    assert ret['code'] == 0
    assert ret['result'] == {
        'box1_temp': 21, 'box2_temp': 22,
        'ptc1_temp': 23, 'ptc2_temp': 24,
        'env_temp': 25, 'env_humidity': 40,
    }


def test_get_temp_decode_float_channels():
    payload = b''.join(
        _pb_float_raw(i, v) for i, v in (
            (1, 30.5), (2, 31.5), (3, 55.0), (4, 60.0), (5, 24.25), (6, 41.5)))
    ret = v2_response_to_v1(Cmd.GET_TEMP, 1, payload)
    r = ret['result']
    assert r['box1_temp'] == pytest.approx(30.5)
    assert r['ptc1_temp'] == pytest.approx(55.0)
    assert r['env_temp'] == pytest.approx(24.25)
    assert r['env_humidity'] == pytest.approx(41.5)


def test_get_temp_missing_channels_default_zero():
    # Only box1 present; the rest default to 0.0.
    ret = v2_response_to_v1(Cmd.GET_TEMP, 1, pb_uint32(1, 27))
    assert ret['result']['box1_temp'] == 27
    assert ret['result']['ptc2_temp'] == 0.0
    assert ret['result']['env_humidity'] == 0.0


# ── v2_response_to_v1: generic else branch (raw opcode probing) ───────────────

def test_unmapped_opcode_surfaces_raw_fields():
    # Opcode 20 has no dedicated decoder; field 1 == 0 means "not an error".
    payload = pb_uint32(1, 0) + pb_uint32(5, 99)
    ret = v2_response_to_v1(20, 11, payload)
    assert ret['code'] == 0
    assert ret['msg'] == 'success'
    assert ret['result'] == {'raw_fields': dump_fields(pb_decode(payload))}
    assert ret['result']['raw_fields'] == {1: 0, 5: 99}


def test_non_error_opcode_does_not_extract_field1_as_error():
    # A generic opcode NOT in _ERROR_CODE_OPCODES must NOT treat field 1 as an
    # error code — field 1 there is data (e.g. a sensor bitmask, step count, or
    # slot echo). Treating it as a code made send_command raise on a good ack
    # (the GET_SENSOR_STATE 'error_70928' class of bug). Opcode 20 is unmapped.
    payload = pb_uint32(1, 400)
    ret = v2_response_to_v1(20, 11, payload)
    assert ret['code'] == 0
    assert ret['msg'] == 'success'
    # raw_fields still present so a probe/diagnostic sees the reply.
    assert ret['result'] == {'raw_fields': {1: 400}}


def test_feed_family_opcode_still_extracts_error_code():
    # The feed/rollback family DOES report status in field 1 (0 ok, non-zero
    # error). _start_feed_assist and the load sequence rely on send_command
    # raising on error_2, so this path must keep mapping field 1 -> code.
    for op in (Cmd.FEED_OR_ROLLBACK, Cmd.STOP_FEED_OR_ROLLBACK, Cmd.UPDATE_SPEED):
        ret = v2_response_to_v1(op, 11, pb_uint32(1, 2))
        assert ret['code'] == 2
        assert ret['msg'] == 'error_2'
    # a clean (code 0) feed ack does not raise
    ok = v2_response_to_v1(Cmd.FEED_OR_ROLLBACK, 11, pb_uint32(1, 0))
    assert ok['code'] == 0 and ok['msg'] == 'success'


def test_unmapped_opcode_empty_payload_no_raw_fields():
    # Empty payload short-circuits before decode; result stays the empty default.
    ret = v2_response_to_v1(20, 11, b'')
    assert ret['code'] == 0
    assert ret['result'] == {}


# ── Tests for the ACE2 per-lane buffer sensors: ───────────────────────────────
#
# was tests/test_AFC_ACE2_buffer.py
# ── GET_SENSOR_STATE decode ───────────────────────────────────────────────────

def _decode(mask):
    return v2_response_to_v1(Cmd.GET_SENSOR_STATE, 1, pb_uint32(1, mask))['result']


def test_decode_slot_bit_offsets():
    # slot0 buf_back(3), slot1 empty(5), slot2 insert(8), slot3 buf_rst(14),
    # shared buf_feed(16)
    mask = (1 << 3) | (1 << 5) | (1 << 8) | (1 << 14) | (1 << 16)
    r = _decode(mask)
    ss = r['slot_sensors']
    assert ss[0] == {'insert': False, 'empty': False,
                    'buf_rst': False, 'buf_back': True}
    assert ss[1]['empty'] is True and ss[1]['buf_back'] is False
    assert ss[2]['insert'] is True
    assert ss[3]['buf_rst'] is True and ss[3]['buf_back'] is False
    assert r['buf_feed'] is True
    assert r['sensor_bitmask'] == mask


def test_decode_all_clear():
    r = _decode(0)
    assert len(r['slot_sensors']) == 4
    assert all(not any(s.values()) for s in r['slot_sensors'])
    assert r['buf_feed'] is False


def test_decode_keeps_raw_sensor_list():
    r = _decode(1 << 16)
    assert len(r['sensors']) == 17
    assert r['sensors'][16] is True


# ── _derive_buffer_state (hardware-confirmed mapping) ─────────────────────────

def test_state_advancing_on_buf_back():
    # BUF_BACK = retracted (tension / "feed me").
    assert _derive_buffer_state(
        {'insert': True, 'empty': False, 'buf_rst': False, 'buf_back': True}
    ) == 'advancing'


def test_state_rest_on_buf_rst():
    assert _derive_buffer_state(
        {'insert': True, 'empty': False, 'buf_rst': True, 'buf_back': False}
    ) == 'rest'


def test_state_neutral_when_extended():
    # Neither per-slot switch set = buffer extended = normal loaded/feeding.
    assert _derive_buffer_state(
        {'insert': True, 'empty': False, 'buf_rst': False, 'buf_back': False}
    ) == 'neutral'


def test_state_buf_back_wins_over_rst():
    # Should never both be set, but buf_back (actionable) takes precedence.
    assert _derive_buffer_state({'buf_rst': True, 'buf_back': True}) == 'advancing'


def test_state_empty_without_data():
    assert _derive_buffer_state(None) == ''
    assert _derive_buffer_state({}) == ''


def test_state_decode_roundtrip():
    # A decoded slot with buf_back set derives to 'advancing'.
    r = _decode(1 << 3)          # slot0 buf_back
    assert _derive_buffer_state(r['slot_sensors'][0]) == 'advancing'
    assert _derive_buffer_state(r['slot_sensors'][1]) == 'neutral'


# ── Unit tests for the ACE2 material-name and sensor-state protocol additions ───
#
# was tests/test_AFC_ACE2_material.py
# ── pb_string ─────────────────────────────────────────────────────────────────

def test_pb_string_encodes_tag_len_and_bytes():
    out = pb_string(2, "AB")
    # field 2, wire type 2 -> tag 0x12; length 2; then 'AB'
    assert out == bytes([0x12, 0x02]) + b"AB"


def test_pb_string_accepts_bytes():
    assert pb_string(1, b"\x01\x02") == bytes([0x0A, 0x02, 0x01, 0x02])


def test_pb_string_roundtrips_through_pb_decode():
    fields = pb_decode(pb_string(2, "hello"))
    assert fields[2][0][1] == b"hello"


# ── method_to_v2 mappings ─────────────────────────────────────────────────────

def test_method_get_material_info():
    cmd, payload = method_to_v2("get_material_info", {"index": 3})
    assert cmd == Cmd.GET_MATERIAL_INFO == 16
    assert payload == pb_uint32(1, 3)


def test_method_get_material_info_defaults_slot_zero():
    cmd, payload = method_to_v2("get_material_info", {})
    assert cmd == 16
    assert payload == pb_uint32(1, 0)


def test_method_set_material_name():
    cmd, payload = method_to_v2("set_material_name", {"index": 2, "name": "PLA"})
    assert cmd == Cmd.SET_MATERIAL_NAME == 18
    # field1 = slot, field2 = name string — the layout confirmed on hardware.
    assert payload == pb_uint32(1, 2) + pb_string(2, "PLA")


def test_method_set_material_name_defaults():
    cmd, payload = method_to_v2("set_material_name", {})
    assert cmd == 18
    assert payload == pb_uint32(1, 0) + pb_string(2, "")


def test_method_get_sensor_state_aliases():
    for name in ("get_sensor_state", "get_key_state"):
        cmd, payload = method_to_v2(name, {})
        assert cmd == Cmd.GET_SENSOR_STATE == 73
        assert payload == b""


# ── v2_response_to_v1: GET_MATERIAL_INFO ──────────────────────────────────────

def _material_payload(slot, name, status=0):
    # response shape observed live: field1=slot, field2={field1=name}, field3=status
    inner = pb_string(1, name)                 # nested {1: name}
    payload = pb_uint32(1, slot) + pb_string(2, inner)
    if status:
        payload += pb_uint32(3, status)
    return payload


def test_get_material_info_decode_extracts_name():
    payload = _material_payload(0, "S0395MB251230046650C3")
    ret = v2_response_to_v1(Cmd.GET_MATERIAL_INFO, 5, payload)
    assert ret['code'] == 0
    assert ret['result']['index'] == 0
    assert ret['result']['material_name'] == "S0395MB251230046650C3"
    assert ret['result']['status'] == 0
    assert 'raw' in ret['result']


def test_get_material_info_decode_with_status_and_slot():
    payload = _material_payload(3, "PETG", status=1)
    ret = v2_response_to_v1(Cmd.GET_MATERIAL_INFO, 5, payload)
    assert ret['result']['index'] == 3
    assert ret['result']['material_name'] == "PETG"
    assert ret['result']['status'] == 1


def test_get_material_info_empty_name():
    # Only slot index present, no nested name field -> empty name, no crash.
    ret = v2_response_to_v1(Cmd.GET_MATERIAL_INFO, 5, pb_uint32(1, 1))
    assert ret['result']['index'] == 1
    assert ret['result']['material_name'] == ""


# ── v2_response_to_v1: GET_SENSOR_STATE ───────────────────────────────────────

def test_get_sensor_state_decodes_bitmask_not_error():
    # 70928 is the real 17-channel bitmask seen live; must NOT become an error.
    ret = v2_response_to_v1(Cmd.GET_SENSOR_STATE, 9, pb_uint32(1, 70928))
    assert ret['code'] == 0                        # not mis-read as error_70928
    assert ret['msg'] == 'success'
    assert ret['result']['sensor_bitmask'] == 70928
    sensors = ret['result']['sensors']
    assert len(sensors) == 17
    # Verify the boolean list matches the mask bit-for-bit.
    assert sensors == [bool(70928 & (1 << i)) for i in range(17)]


def test_get_sensor_state_zero_mask_all_false():
    ret = v2_response_to_v1(Cmd.GET_SENSOR_STATE, 9, pb_uint32(1, 0))
    assert ret['result']['sensor_bitmask'] == 0
    assert ret['result']['sensors'] == [False] * 17


def test_get_sensor_state_individual_bits():
    # bit 0 and bit 4 set -> mask 0x11 = 17
    ret = v2_response_to_v1(Cmd.GET_SENSOR_STATE, 9, pb_uint32(1, 0x11))
    sensors = ret['result']['sensors']
    assert sensors[0] is True
    assert sensors[4] is True
    assert sensors[1] is False
    assert sum(sensors) == 2


# ── Unit tests for the ACE2 firmware-odometer stuck detection in ──────────────
#
# was tests/test_AFC_ACE2_stuck.py
def _make_unit(active_lane="lane0", printing=True, paused=False,
               slot_map=None):
    unit = afcACE2.__new__(afcACE2)
    unit.logger = FakeLogger()
    unit._stuck_detection = True
    unit._stuck_tripped = False
    unit._slot_map = slot_map if slot_map is not None else {"lane0": 0, "lane1": 1}
    unit._active_assist_lane = Recorder(result=active_lane)
    unit.afc = FakeAFC()
    unit.afc.function.in_print_flag = printing
    unit.afc.function.paused = paused
    return unit


def _status(slot_statuses):
    return {"slots": [{"slot_status": s} for s in slot_statuses]}


# ── Firing + latch ────────────────────────────────────────────────────────────

def test_jam_on_active_slot_schedules_handler_once():
    unit = _make_unit()

    unit._check_stuck(_status(["stuck_error", "ready"]))
    unit._check_stuck(_status(["stuck_error", "ready"]))  # next heartbeat

    assert unit._stuck_tripped is True
    # one-shot: handler deferred exactly once despite repeated heartbeats
    assert unit.afc.reactor.register_callback.call_count == 1


def test_all_jam_states_trip():
    for state in afcACE2._ENCODER_JAM_STATES:
        unit = _make_unit()
        unit._check_stuck(_status([state]))
        assert unit._stuck_tripped is True, state
        assert unit.afc.reactor.register_callback.call_count == 1, state


def test_recovery_rearms_latch():
    unit = _make_unit()

    unit._check_stuck(_status(["tangled_error"]))
    assert unit._stuck_tripped is True

    unit._check_stuck(_status(["ready"]))        # recovered
    assert unit._stuck_tripped is False

    unit._check_stuck(_status(["stuck_error"]))  # a new jam fires again
    assert unit._stuck_tripped is True
    assert unit.afc.reactor.register_callback.call_count == 2


def test_healthy_slot_never_trips():
    unit = _make_unit()
    unit._check_stuck(_status(["ready"]))
    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_idle_slot_error_never_trips():
    """Only the active assist lane's slot is consulted — a stale error on an
    idle slot can't pause a healthy print."""
    unit = _make_unit(active_lane="lane0")

    unit._check_stuck(_status(["ready", "stuck_error"]))  # error on slot 1

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


# ── Gate branches ─────────────────────────────────────────────────────────────

def test_detection_disabled_by_config():
    unit = _make_unit()
    unit._stuck_detection = False
    unit._stuck_tripped = True  # must stay untouched — gate is before resets

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is True
    assert not unit.afc.reactor.register_callback.called


def test_not_printing_resets_and_skips():
    unit = _make_unit(printing=False)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_paused_print_resets_and_skips():
    unit = _make_unit(paused=True)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_no_active_lane_resets_and_skips():
    unit = _make_unit(active_lane=None)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_active_lane_missing_from_slot_map_skips():
    unit = _make_unit(active_lane="ghost")

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_malformed_status_is_ignored():
    unit = _make_unit()

    unit._check_stuck({})                          # no slots key
    unit._check_stuck({"slots": "garbage"})        # not a list
    unit._check_stuck(_status([]))                 # index out of range
    unit._check_stuck({"slots": ["not-a-dict"]})   # slot entry not a dict

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called

