"""
Unit tests for the ACE2 explicit-opcode frame builder and response decoding
(extras/AFC_ACE2.py).

  encode_frame          — explicit-opcode frame builder: structure, 16-bit seq
                          masking, payload embedding, oversize rejection, and
                          that encode_request now delegates to it byte-for-byte
  v2_response_to_v1     — GET_TEMP channel mapping (varint and float payloads)
                          and the generic else-branch that surfaces raw_fields
                          (plus device error-code extraction) for unmapped opcodes

Style: typed local fakes (matching test_AFC_ACE2_wire_id.py), full state
verification, every branch exercised.
"""

from __future__ import annotations

import struct

import pytest

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
from extras.AFC_ACE import ACESerialError, ACETimeoutError


WRAPPED = 86480            # a real post-wrap counter value seen on hardware
MASKED = WRAPPED & 0xFFFF  # 20944


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


def _pb_float(field, value):
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
    frame = encode_frame(WRAPPED, Cmd.GET_STATUS, b'')
    _flags, seq, _cmd, _payload = _split_request(frame)
    assert seq == MASKED


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
        _pb_float(i, v) for i, v in (
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
