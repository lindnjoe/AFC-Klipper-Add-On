"""
Unit tests for the ACE2 material-name and sensor-state protocol additions
(extras/AFC_ACE2.py), grounded in live firmware V1.1.31 behaviour:

  method_to_v2        — get_material_info (16, field1=slot), set_material_name
                        (18, field1=slot + field2=name string), and
                        get_sensor_state / get_key_state (73, empty)
  pb_string           — length-delimited UTF-8 field encoding
  v2_response_to_v1   — GET_MATERIAL_INFO name extraction (nested field2->field1)
                        and GET_SENSOR_STATE bitmask decode (the fix for the
                        bitmask being mis-read as an error code)

Style: typed local fakes, full state verification, branch coverage.
"""

from __future__ import annotations

from extras.AFC_ACE2 import (
    Cmd,
    method_to_v2,
    v2_response_to_v1,
    pb_string,
    pb_uint32,
    pb_decode,
)


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
