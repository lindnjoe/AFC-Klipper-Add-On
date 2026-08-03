"""
Unit tests for ACE sensor/state helpers in extras/AFC_ACE.py and the ACE2
feed-check push in extras/AFC_ACE2.py

Style: typed fakes (tests/ace_helpers.py) instead of MagicMock, full state
verification, branch-complete coverage:

  _toolhead_sensor_triggered — filament_sensor_obj priority, fila_tool_start
      U1 runout_buttun_state (true/false), non-U1 sensor fallback, no
      sensors at all
  _is_virtual_hub / _set_hub_state — no hub / real hub / virtual hub with
      tool_loaded true and false; the staged flag is NOT the live signal
  _parse_ace_params — real JSON, console quote-stripped form, lists, bools,
      floats, empty/None
  ACE2 _apply_feed_check — push, no-hardware no-op, non-fatal error
"""

from __future__ import annotations

from extras.AFC_ACE import afcACE
from extras.AFC_ACE2 import afcACE2

from tests.ace_helpers import (
    FakeAce,
    FakeHub,
    FakeLane,
    FakeLogger,
    Recorder,
)


# ── Fakes ─────────────────────────────────────────────────────────────────────

class _U1Sensor:
    """A U1 motion sensor: exposes the raw physical switch state."""

    def __init__(self, buttun_state):
        self.runout_buttun_state = buttun_state


class _PlainSensor:
    """A plain switch sensor: no runout_buttun_state attribute."""


class _Extruder:
    """AFC_extruder stand-in with the two possible sensor attributes; omit
    either by passing the _MISSING sentinel."""

    _MISSING = object()

    def __init__(self, filament_sensor_obj=None, fila_tool_start=None):
        if filament_sensor_obj is not self._MISSING:
            self.filament_sensor_obj = filament_sensor_obj
        if fila_tool_start is not self._MISSING:
            self.fila_tool_start = fila_tool_start


def _make_unit():
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    return unit


def _sensor_lane(ext, pre_sensor=False):
    lane = FakeLane("lane0", extruder_obj=ext)
    lane.get_toolhead_pre_sensor_state = Recorder(result=pre_sensor)
    return lane


# ── _toolhead_sensor_triggered ────────────────────────────────────────────────

def test_toolhead_sensor_uses_u1_button_state_true():
    unit = _make_unit()
    lane = _sensor_lane(_Extruder(fila_tool_start=_U1Sensor(1)))

    assert unit._toolhead_sensor_triggered(lane) is True
    assert not lane.get_toolhead_pre_sensor_state.called  # no fallback


def test_toolhead_sensor_uses_u1_button_state_false():
    unit = _make_unit()
    lane = _sensor_lane(_Extruder(fila_tool_start=_U1Sensor(0)))

    assert unit._toolhead_sensor_triggered(lane) is False
    assert not lane.get_toolhead_pre_sensor_state.called


def test_toolhead_sensor_filament_sensor_obj_takes_priority():
    unit = _make_unit()
    ext = _Extruder(filament_sensor_obj=_U1Sensor(1),
                    fila_tool_start=_U1Sensor(0))
    lane = _sensor_lane(ext)

    assert unit._toolhead_sensor_triggered(lane) is True


def test_toolhead_sensor_plain_sensor_falls_back():
    """A sensor without runout_buttun_state -> normal pre-sensor read."""
    unit = _make_unit()
    lane = _sensor_lane(_Extruder(fila_tool_start=_PlainSensor()),
                        pre_sensor=True)

    assert unit._toolhead_sensor_triggered(lane) is True
    assert lane.get_toolhead_pre_sensor_state.call_count == 1


def test_toolhead_sensor_no_sensor_objects_falls_back():
    unit = _make_unit()
    lane = _sensor_lane(_Extruder(filament_sensor_obj=_Extruder._MISSING,
                                  fila_tool_start=_Extruder._MISSING),
                        pre_sensor=False)

    assert unit._toolhead_sensor_triggered(lane) is False
    assert lane.get_toolhead_pre_sensor_state.call_count == 1


# ── Virtual hub semantics ─────────────────────────────────────────────────────

def test_is_virtual_hub_all_branches():
    unit = _make_unit()

    lane = FakeLane("lane0", hub_obj=None)
    assert unit._is_virtual_hub(lane) is False

    class _RealHub:
        pass  # no is_virtual_pin attribute

    lane = FakeLane("lane0", hub_obj=_RealHub())
    assert unit._is_virtual_hub(lane) is False

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=True))
    assert unit._is_virtual_hub(lane) is True

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=False))
    assert unit._is_virtual_hub(lane) is False


def test_virtual_hub_occupancy_derives_from_tool_loaded():
    """The virtual hub reads 'loaded' only while filament is threaded THROUGH
    it to a toolhead — a staged-but-not-loaded lane stays clear so its own
    load doesn't trip 'Hub not clear'."""
    unit = _make_unit()

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=True), tool_loaded=False)
    unit._set_hub_state(lane, True)   # the staged flag is NOT the live signal
    assert lane._load_state is False

    lane = FakeLane("lane0", hub_obj=FakeHub(virtual=True), tool_loaded=True)
    unit._set_hub_state(lane, False)
    assert lane._load_state is True


def test_real_hub_left_alone():
    unit = _make_unit()
    lane = FakeLane("lane0", hub_obj=None)
    lane._load_state = "sentinel"

    unit._set_hub_state(lane, True)

    assert lane._load_state == "sentinel"


# ── _parse_ace_params ─────────────────────────────────────────────────────────

def test_parse_params_real_json():
    assert afcACE._parse_ace_params('{"index": 0, "type": "PLA"}') == {
        "index": 0, "type": "PLA"}


def test_parse_params_console_stripped_quotes():
    """The gcode console strips JSON double-quotes — the quote-less form must
    parse with int/float/bool/string inference."""
    result = afcACE._parse_ace_params('{index:0,length:50.5,type:PLA,dry:true}')
    assert result == {"index": 0, "length": 50.5, "type": "PLA", "dry": True}


def test_parse_params_false_bool():
    assert afcACE._parse_ace_params('{dry:false}') == {"dry": False}


def test_parse_params_list_values():
    result = afcACE._parse_ace_params('{index:0,color:[255,0,0]}')
    assert result == {"index": 0, "color": [255, 0, 0]}


def test_parse_params_empty_returns_none():
    assert afcACE._parse_ace_params("") is None
    assert afcACE._parse_ace_params(None) is None
    assert afcACE._parse_ace_params("   ") is None


# ── ACE2 _apply_feed_check ────────────────────────────────────────────────────

def _make_ace2(ace=None):
    unit = afcACE2.__new__(afcACE2)
    unit.name = "ACE_1"
    unit.logger = FakeLogger()
    unit.feed_check_length = 200
    unit.feed_error_length = 190
    unit._ace = ace
    return unit


def test_feed_check_pushes_configured_window():
    unit = _make_ace2(ace=FakeAce())
    unit._apply_feed_check()
    assert unit._ace.send_command_async.calls == [(
        ("set_feed_check", {"check_length": 200, "error_length": 190}), {})]
    assert len(unit.logger.lines["info"]) == 1


def test_feed_check_noop_without_connection():
    unit = _make_ace2(ace=None)
    unit._apply_feed_check()  # must not raise
    assert unit.logger.lines["info"] == []


def test_feed_check_error_is_nonfatal():
    ace = FakeAce()
    ace.send_command_async = Recorder(raises=RuntimeError("serial down"))
    unit = _make_ace2(ace=ace)

    unit._apply_feed_check()  # swallowed with a warning

    assert len(unit.logger.lines["warning"]) == 1
