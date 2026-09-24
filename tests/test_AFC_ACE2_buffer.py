"""
Tests for the ACE2 per-lane buffer sensors:
  - GET_SENSOR_STATE 17-bit decode into per-slot {insert,empty,buf_rst,buf_back}
    + shared buf_feed (extras/AFC_ACE2.py).
  - _derive_buffer_state: per-lane buffer state from the decoded switches
    (extras/AFC_ACE.py), hardware-confirmed mapping.
No hardware/ACE connection.
"""
from __future__ import annotations

from extras.AFC_ACE2 import v2_response_to_v1, pb_uint32, Cmd
from extras.AFC_ACE import _derive_buffer_state


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
