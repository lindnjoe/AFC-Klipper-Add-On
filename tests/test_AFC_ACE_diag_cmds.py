"""
Unit tests for the ACE diagnostic gcode handlers in extras/AFC_ACE.py:

  cmd_ACE_TEMP_INFO — temperature/humidity readout. Not connected, get_temp
                      raises, non-dict reply, and the success formatting.

Style: typed fakes (tests/ace_helpers.py).
"""

from __future__ import annotations

from extras.AFC_ACE import afcACE

from tests.ace_helpers import (
    FakeAce2,
    FakeGcmd,
    Recorder,
)


def _unit(ace):
    unit = afcACE.__new__(afcACE)
    unit._ace = ace
    return unit


# ── cmd_ACE_TEMP_INFO ─────────────────────────────────────────────────────────

def test_temp_info_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd()
    unit.cmd_ACE_TEMP_INFO(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_temp_info_get_temp_raises():
    ace = FakeAce2()
    ace.get_temp = Recorder(raises=RuntimeError("unsupported"))
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    assert len(gcmd.responses) == 1
    assert "ACE_TEMP_INFO" in gcmd.responses[0]
    assert "unsupported" in gcmd.responses[0]


def test_temp_info_non_dict_reply():
    ace = FakeAce2()
    ace.get_temp = Recorder(result=None)
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    assert len(gcmd.responses) == 1
    assert "unexpected reply" in gcmd.responses[0]


def test_temp_info_success_formats_all_channels():
    ace = FakeAce2()
    ace.get_temp = Recorder(result={
        'box1_temp': 30.5, 'box2_temp': 31.0,
        'ptc1_temp': 55.0, 'ptc2_temp': 60.0,
        'env_temp': 24.0, 'env_humidity': 41.0,
    })
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    assert ace.get_temp.call_count == 1
    assert len(gcmd.responses) == 1
    msg = gcmd.responses[0]
    for token in ("box1=30.5", "box2=31.0", "ptc1=55.0",
                  "ptc2=60.0", "env=24.0", "humidity=41.0"):
        assert token in msg


def test_temp_info_missing_channels_render_na():
    ace = FakeAce2()
    ace.get_temp = Recorder(result={'box1_temp': 30.5})  # rest absent
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_TEMP_INFO(gcmd)

    msg = gcmd.responses[0]
    assert "box1=30.5" in msg
    assert "humidity=n/a" in msg


# ── cmd_ACE_MATERIAL_INFO ─────────────────────────────────────────────────────

def test_material_info_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd()
    unit.cmd_ACE_MATERIAL_INFO(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_material_info_default_slot_zero():
    ace = FakeAce2()
    ace.get_material_info = Recorder(result={
        'index': 0, 'material_name': 'S0395MB251230046650C3', 'status': 0})
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_MATERIAL_INFO(gcmd)

    assert ace.get_material_info.last_args == (0,)
    msg = gcmd.responses[0]
    assert "slot 0" in msg
    assert "S0395MB251230046650C3" in msg


def test_material_info_explicit_slot():
    ace = FakeAce2()
    ace.get_material_info = Recorder(result={
        'index': 3, 'material_name': 'PETG', 'status': 1})
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=3)

    unit.cmd_ACE_MATERIAL_INFO(gcmd)

    assert ace.get_material_info.last_args == (3,)
    assert "slot 3" in gcmd.responses[0]
    assert "PETG" in gcmd.responses[0]


def test_material_info_error_surfaced():
    ace = FakeAce2()
    ace.get_material_info = Recorder(raises=RuntimeError("timeout"))
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0)

    unit.cmd_ACE_MATERIAL_INFO(gcmd)

    assert "ACE_MATERIAL_INFO" in gcmd.responses[0]
    assert "timeout" in gcmd.responses[0]


# ── cmd_ACE_SET_MATERIAL ──────────────────────────────────────────────────────

def test_set_material_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd(SLOT=0, NAME="X")
    unit.cmd_ACE_SET_MATERIAL(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_set_material_requires_slot():
    ace = FakeAce2()
    unit = _unit(ace)
    gcmd = FakeGcmd(NAME="X")  # no SLOT
    unit.cmd_ACE_SET_MATERIAL(gcmd)
    assert "SLOT=<n> required" in gcmd.responses[0]
    assert ace.set_material_name.call_count == 0


def test_set_material_requires_name():
    ace = FakeAce2()
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0)  # no NAME
    unit.cmd_ACE_SET_MATERIAL(gcmd)
    assert "NAME=<text> required" in gcmd.responses[0]
    assert ace.set_material_name.call_count == 0


def test_set_material_writes_and_reads_back():
    ace = FakeAce2()
    ace.set_material_name = Recorder(result={})
    ace.get_material_info = Recorder(result={'index': 2, 'material_name': 'PLA_X'})
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=2, NAME="PLA_X")

    unit.cmd_ACE_SET_MATERIAL(gcmd)

    assert ace.set_material_name.last_args == (2, "PLA_X")
    assert ace.get_material_info.last_args == (2,)     # read-back
    assert "slot 2" in gcmd.responses[0]
    assert "PLA_X" in gcmd.responses[0]


def test_set_material_write_error_surfaced():
    ace = FakeAce2()
    ace.set_material_name = Recorder(raises=RuntimeError("boom"))
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0, NAME="X")

    unit.cmd_ACE_SET_MATERIAL(gcmd)

    assert "ACE_SET_MATERIAL" in gcmd.responses[0]
    assert "boom" in gcmd.responses[0]
    assert ace.get_material_info.call_count == 0       # never reached read-back


def test_set_material_readback_failure_still_reports_write():
    ace = FakeAce2()
    ace.set_material_name = Recorder(result={})
    ace.get_material_info = Recorder(raises=RuntimeError("readfail"))
    unit = _unit(ace)
    gcmd = FakeGcmd(SLOT=0, NAME="X")

    unit.cmd_ACE_SET_MATERIAL(gcmd)

    msg = gcmd.responses[0]
    assert "wrote 'X'" in msg
    assert "read-back failed" in msg


# ── cmd_ACE_SENSOR_STATE ──────────────────────────────────────────────────────

def test_sensor_state_not_connected():
    unit = _unit(FakeAce2(connected=False))
    gcmd = FakeGcmd()
    unit.cmd_ACE_SENSOR_STATE(gcmd)
    assert gcmd.responses == ["ACE not connected"]


def test_sensor_state_reports_mask_and_triggered():
    ace = FakeAce2()
    sensors = [bool(0x11 & (1 << i)) for i in range(17)]  # bits 0 and 4
    ace.get_sensor_state = Recorder(result={
        'sensor_bitmask': 0x11, 'sensors': sensors})
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_SENSOR_STATE(gcmd)

    msg = gcmd.responses[0]
    assert "0x11" in msg
    assert "[0, 4]" in msg


def test_sensor_state_error_surfaced():
    ace = FakeAce2()
    ace.get_sensor_state = Recorder(raises=RuntimeError("nope"))
    unit = _unit(ace)
    gcmd = FakeGcmd()

    unit.cmd_ACE_SENSOR_STATE(gcmd)

    assert "ACE_SENSOR_STATE" in gcmd.responses[0]
    assert "nope" in gcmd.responses[0]
