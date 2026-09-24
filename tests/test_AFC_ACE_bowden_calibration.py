"""
AFC refuses bowden calibration when the extruder has no pin_tool_start and no
buffer. An ACE lane on a U1 toolhead measures to the U1 filament sensor itself,
so its unit lets the calibration through; every other lane keeps AFC's check.
"""

from __future__ import annotations

import types

from extras.AFC_ACE import afcACE

from tests.ace_helpers import FakeAFC

_REFUSED = (True, False, "\nBowden Calibration Error:")


def _setup():
    calls = []

    def check(cur_lane):
        calls.append(cur_lane)
        return _REFUSED

    unit = afcACE.__new__(afcACE)
    unit.afc = FakeAFC()
    unit.afc.function = types.SimpleNamespace(
        _calibration_check_tool_start=check)
    return unit, calls


def _lane(unit_obj, tool_start=None, u1_sensor=True):
    extruder = types.SimpleNamespace(
        tool_start=tool_start,
        fila_tool_start=object() if u1_sensor else None)
    return types.SimpleNamespace(unit_obj=unit_obj, extruder_obj=extruder)


def test_ace_lane_with_u1_sensor_passes_the_check():
    unit, calls = _setup()
    unit._allow_bowden_calibration_on_u1_sensor()
    check = unit.afc.function._calibration_check_tool_start
    assert check(_lane(unit)) == (False, False, "")
    assert calls == []


def test_other_lanes_keep_afcs_check():
    unit, calls = _setup()
    unit._allow_bowden_calibration_on_u1_sensor()
    check = unit.afc.function._calibration_check_tool_start
    other = _lane(object())
    no_sensor = _lane(unit, u1_sensor=False)
    assert check(other) == _REFUSED
    assert check(no_sensor) == _REFUSED
    assert calls == [other, no_sensor]


def test_lane_with_pin_tool_start_keeps_afcs_check():
    unit, calls = _setup()
    unit._allow_bowden_calibration_on_u1_sensor()
    lane = _lane(unit, tool_start="buffer")
    assert unit.afc.function._calibration_check_tool_start(lane) == _REFUSED
    assert calls == [lane]


def test_wrapped_once_for_several_units():
    unit, calls = _setup()
    unit._allow_bowden_calibration_on_u1_sensor()
    first = unit.afc.function._calibration_check_tool_start
    second_unit = afcACE.__new__(afcACE)
    second_unit.afc = unit.afc
    second_unit._allow_bowden_calibration_on_u1_sensor()
    assert unit.afc.function._calibration_check_tool_start is first
    unit.afc.function._calibration_check_tool_start(_lane(object()))
    assert len(calls) == 1


def test_no_function_object_is_a_no_op():
    unit = afcACE.__new__(afcACE)
    unit.afc = types.SimpleNamespace()
    unit._allow_bowden_calibration_on_u1_sensor()
