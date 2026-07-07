"""
Unit tests for ACE sensor/state helpers in extras/AFC_ACE.py and the ACE2
feed-check push in extras/AFC_ACE2.py

Covers:
  - _toolhead_sensor_triggered: U1 motion-sensor fallback chain — prefers the
    raw runout_buttun_state (static switch) from fila_tool_start since
    filament_sensor_obj is never assigned, falling back to the normal
    pre-sensor state otherwise
  - _is_virtual_hub / _set_hub_state: virtual-hub occupancy derives from
    tool_loaded (the "Hub not clear" fix); real hubs are left alone
  - _parse_ace_params: JSON and the console quote-stripped {k:v} form
  - ACE2 _apply_feed_check: pushes the configured window, survives errors
"""

from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_ACE import afcACE
from extras.AFC_ACE2 import afcACE2


# ── _toolhead_sensor_triggered ────────────────────────────────────────────────

def _make_unit():
    unit = afcACE.__new__(afcACE)
    unit.logger = MagicMock()
    return unit


def test_toolhead_sensor_uses_u1_button_state():
    """A U1 motion sensor exposes runout_buttun_state (the physical switch);
    it wins over the motion-based filament_present."""
    unit = _make_unit()
    lane = MagicMock()
    lane.extruder_obj.filament_sensor_obj = None
    lane.extruder_obj.fila_tool_start.runout_buttun_state = 1
    lane.get_toolhead_pre_sensor_state = MagicMock(
        side_effect=AssertionError("fallback should not run"))

    assert unit._toolhead_sensor_triggered(lane) is True


def test_toolhead_sensor_button_state_false():
    unit = _make_unit()
    lane = MagicMock()
    lane.extruder_obj.filament_sensor_obj = None
    lane.extruder_obj.fila_tool_start.runout_buttun_state = 0

    assert unit._toolhead_sensor_triggered(lane) is False


def test_toolhead_sensor_falls_back_to_pre_sensor():
    """No U1 button state (normal switch sensor) -> normal pre-sensor read."""
    unit = _make_unit()
    lane = MagicMock()
    lane.extruder_obj.filament_sensor_obj = None
    lane.extruder_obj.fila_tool_start = MagicMock(spec=[])  # no buttun state
    lane.get_toolhead_pre_sensor_state = MagicMock(return_value=True)

    assert unit._toolhead_sensor_triggered(lane) is True
    lane.get_toolhead_pre_sensor_state.assert_called_once()


def test_toolhead_sensor_no_sensor_objects_falls_back():
    unit = _make_unit()
    lane = MagicMock()
    lane.extruder_obj = MagicMock(spec=[])  # neither attribute exists
    lane.get_toolhead_pre_sensor_state = MagicMock(return_value=False)

    assert unit._toolhead_sensor_triggered(lane) is False


# ── Virtual hub semantics ─────────────────────────────────────────────────────

def _virtual_hub_lane(tool_loaded):
    lane = MagicMock()
    lane.tool_loaded = tool_loaded
    lane._load_state = None
    lane.hub_obj.is_virtual_pin.return_value = True
    return lane


def test_virtual_hub_occupancy_derives_from_tool_loaded():
    """The virtual hub reads 'loaded' only while filament is threaded THROUGH
    it to a toolhead — a staged-but-not-loaded lane stays clear so its own
    load doesn't trip 'Hub not clear'."""
    unit = _make_unit()

    lane = _virtual_hub_lane(tool_loaded=False)
    unit._set_hub_state(lane, True)   # staged flag is NOT the live signal
    assert lane._load_state is False

    lane = _virtual_hub_lane(tool_loaded=True)
    unit._set_hub_state(lane, False)
    assert lane._load_state is True


def test_real_hub_left_alone():
    unit = _make_unit()
    lane = MagicMock()
    lane.hub_obj = MagicMock(spec=[])  # no is_virtual_pin -> real switch
    lane._load_state = "untouched"

    unit._set_hub_state(lane, True)

    assert lane._load_state == "untouched"


def test_is_virtual_hub_variants():
    unit = _make_unit()

    lane = MagicMock()
    lane.hub_obj = None
    assert unit._is_virtual_hub(lane) is False

    lane.hub_obj = MagicMock()
    lane.hub_obj.is_virtual_pin.return_value = True
    assert unit._is_virtual_hub(lane) is True

    lane.hub_obj.is_virtual_pin.return_value = False
    assert unit._is_virtual_hub(lane) is False


# ── _parse_ace_params ─────────────────────────────────────────────────────────

def test_parse_params_real_json():
    assert afcACE._parse_ace_params('{"index": 0, "type": "PLA"}') == {
        "index": 0, "type": "PLA"}


def test_parse_params_console_stripped_quotes():
    """The gcode console strips JSON double-quotes — the quote-less form must
    parse with type inference."""
    result = afcACE._parse_ace_params('{index:0,length:50.5,type:PLA,dry:true}')
    assert result == {"index": 0, "length": 50.5, "type": "PLA", "dry": True}


def test_parse_params_list_values():
    result = afcACE._parse_ace_params('{index:0,color:[255,0,0]}')
    assert result == {"index": 0, "color": [255, 0, 0]}


def test_parse_params_empty_returns_none():
    assert afcACE._parse_ace_params("") is None
    assert afcACE._parse_ace_params(None) is None


# ── ACE2 _apply_feed_check ────────────────────────────────────────────────────

def _make_ace2():
    unit = afcACE2.__new__(afcACE2)
    unit.name = "ACE_1"
    unit.logger = MagicMock()
    unit.feed_check_length = 200
    unit.feed_error_length = 190
    unit._ace = MagicMock()
    return unit


def test_feed_check_pushes_configured_window():
    unit = _make_ace2()
    unit._apply_feed_check()
    unit._ace.send_command_async.assert_called_once_with(
        "set_feed_check", {"check_length": 200, "error_length": 190})


def test_feed_check_noop_without_connection():
    unit = _make_ace2()
    unit._ace = None
    unit._apply_feed_check()  # must not raise


def test_feed_check_error_is_nonfatal():
    unit = _make_ace2()
    unit._ace.send_command_async.side_effect = RuntimeError("serial down")
    unit._apply_feed_check()  # swallowed with a warning
    unit.logger.warning.assert_called_once()
