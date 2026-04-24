"""
Unit tests for extras/AFC_ACE.py

Covers:
  - Module constants (MODE_COMBINED, MODE_DIRECT, SLOTS_PER_UNIT)
  - Inheritance from afcUnit
  - Helper getters: _get_feed_length, _get_retract_length, _get_dist_hub,
    _get_feed_assist, _get_feed_assist_for_slot, _quiet_speed
  - Slot mapping: _get_local_slot_for_lane
  - Hub helpers: _hub_has_real_pin, _hub_is_virtual
  - FPS passthrough: get_fps_value
  - Unit hook overrides: prep_capture_td1, capture_td1_data
  - Unit hook overrides: get_lane_reset_command
  - lane_unload: marks state correctly and returns True
"""

from __future__ import annotations

from unittest.mock import MagicMock
import pytest

from extras.AFC_ACE import afcACE, MODE_COMBINED, MODE_DIRECT
from extras.AFC_unit import afcUnit


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_ace(name="ACE_1", mode=MODE_COMBINED):
    """Build an afcACE bypassing __init__ so we can test pure-logic methods."""
    unit = afcACE.__new__(afcACE)

    from tests.conftest import MockAFC, MockPrinter, MockLogger, MockReactor

    afc = MockAFC()
    afc.toolhead = MagicMock()
    reactor = MockReactor()
    afc.reactor = reactor
    afc.logger = MockLogger()
    printer = MockPrinter(afc=afc)

    unit.printer = printer
    unit.afc = afc
    unit.logger = afc.logger
    unit.reactor = reactor
    unit.name = name
    unit.full_name = ["AFC_ACE", name]
    unit.type = "ACE"
    unit.mode = mode
    unit.gcode = afc.gcode

    # Config-style defaults
    unit.feed_length = 500.0
    unit.retract_length = 500.0
    unit.dist_hub = 200.0
    unit.feed_speed = 30
    unit.retract_speed = 30
    unit.max_feed_overshoot = 100.0

    # Per-slot / per-lane override registries
    unit._default_feed_assist = True
    unit._slot_feed_assist = {}
    unit._lane_feed_length = {}
    unit._lane_retract_length = {}
    unit._lane_dist_hub = {}
    unit._lane_feed_assist = {}

    # Lane/hub/ext/buffer links
    unit.lanes = {}
    unit.hub_obj = None
    unit.extruder_obj = None
    unit.buffer_obj = None
    unit.hub = None
    unit.extruder = None
    unit.buffer_name = None

    # Hardware state
    unit._ace = None
    unit._slot_inventory = [{} for _ in range(afcACE.SLOTS_PER_UNIT)]
    unit._cached_hw_status = {}
    unit._current_loaded_slot = -1
    unit._prev_slot_states = {}
    unit._feed_assist_active = set()
    unit._operation_active = False

    # FPS integration state
    unit._fps_obj = None
    unit._fps_extruder = None
    unit.fps_threshold = 0.9
    unit.fps_load_threshold = 0.65
    unit.fps_delta_threshold = 0.15
    unit.fps_confirm_count = 3
    unit._prev_fps_value = 0.0
    unit._fps_latched = False
    unit._fps_consecutive_hits = 0

    return unit


def _make_lane(name="lane1", index=1, loaded_to_hub=False,
               td1_when_loaded=False, td1_device_id=None):
    lane = MagicMock()
    lane.name = name
    lane.index = index
    lane.loaded_to_hub = loaded_to_hub
    lane.td1_when_loaded = td1_when_loaded
    lane.td1_device_id = td1_device_id
    return lane


# ── Module constants ──────────────────────────────────────────────────────────

class TestAceConstants:
    def test_mode_combined(self):
        assert MODE_COMBINED == "combined"

    def test_mode_direct(self):
        assert MODE_DIRECT == "direct"

    def test_slots_per_unit(self):
        assert afcACE.SLOTS_PER_UNIT == 4


# ── Inheritance ───────────────────────────────────────────────────────────────

class TestAceInheritance:
    def test_is_subclass_of_unit(self):
        assert issubclass(afcACE, afcUnit)

    def test_instance_is_afc_unit(self):
        unit = _make_ace()
        assert isinstance(unit, afcUnit)


# ── Feed / retract / dist_hub getters ────────────────────────────────────────

class TestFeedLengthGetter:
    def test_defaults_to_unit_value(self):
        unit = _make_ace()
        unit.feed_length = 800
        assert unit._get_feed_length() == 800

    def test_lane_override_takes_precedence(self):
        unit = _make_ace()
        unit._lane_feed_length["lane2"] = 2800
        assert unit._get_feed_length(_make_lane(name="lane2")) == 2800

    def test_missing_lane_falls_back_to_default(self):
        unit = _make_ace()
        unit._lane_feed_length["other"] = 999
        assert unit._get_feed_length(_make_lane(name="lane1")) == unit.feed_length


class TestRetractLengthGetter:
    def test_defaults_to_unit_value(self):
        unit = _make_ace()
        unit.retract_length = 510
        assert unit._get_retract_length() == 510

    def test_lane_override_takes_precedence(self):
        unit = _make_ace()
        unit._lane_retract_length["lane1"] = 2500
        assert unit._get_retract_length(_make_lane(name="lane1")) == 2500


class TestDistHubGetter:
    def test_defaults_to_unit_value(self):
        unit = _make_ace()
        unit.dist_hub = 210
        assert unit._get_dist_hub() == 210

    def test_lane_override_takes_precedence(self):
        unit = _make_ace()
        unit._lane_dist_hub["lane1"] = 90
        assert unit._get_dist_hub(_make_lane(name="lane1")) == 90


# ── Feed assist resolution ────────────────────────────────────────────────────

class TestFeedAssistResolution:
    def test_default_when_nothing_overridden(self):
        unit = _make_ace()
        unit._default_feed_assist = True
        assert unit._get_feed_assist_for_slot(0) is True
        assert unit._get_feed_assist(0) is True

    def test_default_off(self):
        unit = _make_ace()
        unit._default_feed_assist = False
        assert unit._get_feed_assist_for_slot(0) is False

    def test_slot_override_takes_precedence_over_default(self):
        unit = _make_ace()
        unit._default_feed_assist = True
        unit._slot_feed_assist[2] = False
        assert unit._get_feed_assist_for_slot(2) is False
        assert unit._get_feed_assist_for_slot(1) is True

    def test_lane_override_beats_slot_override(self):
        unit = _make_ace()
        unit._default_feed_assist = False
        unit._slot_feed_assist[1] = False
        unit._lane_feed_assist["lane_a"] = True
        lane = _make_lane(name="lane_a", index=2)
        assert unit._get_feed_assist(1, lane) is True


# ── _quiet_speed ──────────────────────────────────────────────────────────────

class TestQuietSpeed:
    def test_normal_mode_passes_through(self):
        unit = _make_ace()
        unit.afc._get_quiet_mode = MagicMock(return_value=False)
        assert unit._quiet_speed(60) == 60

    def test_quiet_mode_halves(self):
        unit = _make_ace()
        unit.afc._get_quiet_mode = MagicMock(return_value=True)
        assert unit._quiet_speed(60) == 30


# ── _get_local_slot_for_lane ─────────────────────────────────────────────────

class TestLocalSlotForLane:
    def test_maps_one_based_to_zero_based(self):
        unit = _make_ace()
        assert unit._get_local_slot_for_lane(_make_lane(index=1)) == 0
        assert unit._get_local_slot_for_lane(_make_lane(index=4)) == 3

    def test_out_of_range_returns_minus_one(self):
        unit = _make_ace()
        assert unit._get_local_slot_for_lane(_make_lane(index=0)) == -1
        assert unit._get_local_slot_for_lane(_make_lane(index=5)) == -1

    def test_non_int_index_returns_minus_one(self):
        unit = _make_ace()
        assert unit._get_local_slot_for_lane(_make_lane(index="bad")) == -1


# ── _hub_has_real_pin / _hub_is_virtual ───────────────────────────────────────

class TestHubPinHelpers:
    def test_real_pin(self):
        unit = _make_ace()
        hub = MagicMock()
        hub.switch_pin = "PA1"
        assert unit._hub_has_real_pin(hub) is True

    def test_virtual_pin(self):
        unit = _make_ace()
        hub = MagicMock()
        hub.switch_pin = "Virtual"
        assert unit._hub_has_real_pin(hub) is False

    def test_no_hub_returns_false(self):
        unit = _make_ace()
        assert unit._hub_has_real_pin(None) is False

    def test_placeholder_hub_without_switch_pin(self):
        unit = _make_ace()

        class Placeholder:
            pass

        assert unit._hub_has_real_pin(Placeholder()) is False

    def test_hub_is_virtual_when_no_hub_obj(self):
        unit = _make_ace()
        unit.hub_obj = None
        assert unit._hub_is_virtual() is True

    def test_hub_is_virtual_when_switch_pin_virtual(self):
        unit = _make_ace()
        unit.hub_obj = MagicMock()
        unit.hub_obj.switch_pin = "virtual"
        assert unit._hub_is_virtual() is True

    def test_hub_not_virtual_when_real_pin(self):
        unit = _make_ace()
        unit.hub_obj = MagicMock()
        unit.hub_obj.switch_pin = "PB4"
        assert unit._hub_is_virtual() is False


# ── get_fps_value ─────────────────────────────────────────────────────────────

class TestGetFpsValue:
    def test_no_fps_obj_returns_none(self):
        unit = _make_ace()
        unit._fps_obj = None
        assert unit.get_fps_value() is None

    def test_delegates_to_get_fps_value(self):
        unit = _make_ace()
        fps = MagicMock()
        fps.get_fps_value.return_value = 0.42
        unit._fps_obj = fps
        assert unit.get_fps_value() == 0.42

    def test_falls_back_to_fps_value_attr(self):
        unit = _make_ace()
        fps = MagicMock(spec=["fps_value"])  # no get_fps_value method
        fps.fps_value = 0.77
        unit._fps_obj = fps
        assert unit.get_fps_value() == 0.77


# ── prep_capture_td1 hook ─────────────────────────────────────────────────────

class TestPrepCaptureTd1:
    def test_returns_none_when_not_loaded_to_hub(self):
        unit = _make_ace()
        lane = _make_lane(td1_when_loaded=True, loaded_to_hub=False)
        assert unit.prep_capture_td1(lane) is None

    def test_returns_none_when_td1_disabled(self):
        unit = _make_ace()
        lane = _make_lane(td1_when_loaded=False, loaded_to_hub=True)
        assert unit.prep_capture_td1(lane) is None

    def test_returns_tuple_when_handled(self):
        unit = _make_ace()
        lane = _make_lane(td1_when_loaded=True, loaded_to_hub=True)
        result = unit.prep_capture_td1(lane)
        assert isinstance(result, tuple)
        assert result[0] is True


# ── capture_td1_data hook ─────────────────────────────────────────────────────

class TestCaptureTd1Data:
    def test_returns_none_when_no_ace(self):
        unit = _make_ace()
        unit._ace = None
        lane = _make_lane(td1_device_id="td1_a")
        assert unit.capture_td1_data(lane) is None

    def test_returns_none_when_ace_not_connected(self):
        unit = _make_ace()
        unit._ace = MagicMock()
        unit._ace.connected = False
        lane = _make_lane(td1_device_id="td1_a")
        assert unit.capture_td1_data(lane) is None

    def test_returns_none_when_no_td1_device_id(self):
        unit = _make_ace()
        unit._ace = MagicMock()
        unit._ace.connected = True
        lane = _make_lane(td1_device_id=None)
        assert unit.capture_td1_data(lane) is None

    def test_returns_error_tuple_when_bowden_length_missing(self):
        unit = _make_ace()
        unit._ace = MagicMock()
        unit._ace.connected = True
        lane = _make_lane(td1_device_id="td1_a")
        lane.td1_bowden_length = None
        lane.index = 1
        result = unit.capture_td1_data(lane)
        assert isinstance(result, tuple)
        assert result[0] is False
        assert "td1_bowden_length" in result[1]

    def test_returns_false_when_no_slot(self):
        unit = _make_ace()
        unit._ace = MagicMock()
        unit._ace.connected = True
        lane = _make_lane(td1_device_id="td1_a", index=99)  # out of range
        lane.td1_bowden_length = 100
        result = unit.capture_td1_data(lane)
        assert isinstance(result, tuple)
        assert result[0] is False


# ── get_lane_reset_command hook ───────────────────────────────────────────────

class TestGetLaneResetCommand:
    def test_returns_ace_lane_reset(self):
        unit = _make_ace()
        lane = _make_lane(name="lane2")
        cmd = unit.get_lane_reset_command(lane, 50)
        assert cmd == f"ACE_LANE_RESET UNIT={unit.name} LANE=lane2"

    def test_includes_unit_name(self):
        unit = _make_ace(name="ACE_2")
        lane = _make_lane(name="lane4")
        cmd = unit.get_lane_reset_command(lane, None)
        assert "UNIT=ACE_2" in cmd
        assert "LANE=lane4" in cmd


# ── lane_unload hook ──────────────────────────────────────────────────────────

class TestLaneUnload:
    def test_returns_true_to_skip_default_path(self):
        unit = _make_ace()
        lane = _make_lane()
        lane.status = "LOADED"
        unit.eject_lane = MagicMock()
        unit.lane_not_ready = MagicMock()
        unit.afc.save_vars = MagicMock()
        unit.afc.spool.set_spoolID = MagicMock()
        result = unit.lane_unload(lane)
        assert result is True

    def test_calls_eject_lane(self):
        unit = _make_ace()
        lane = _make_lane()
        lane.status = "LOADED"
        unit.eject_lane = MagicMock()
        unit.lane_not_ready = MagicMock()
        unit.lane_unload(lane)
        unit.eject_lane.assert_called_once_with(lane)

    def test_clears_loaded_to_hub(self):
        unit = _make_ace()
        lane = _make_lane(loaded_to_hub=True)
        lane.status = "LOADED"
        unit.eject_lane = MagicMock()
        unit.lane_not_ready = MagicMock()
        unit.lane_unload(lane)
        assert lane.loaded_to_hub is False

    def test_clears_spool_id(self):
        unit = _make_ace()
        lane = _make_lane()
        lane.status = "LOADED"
        unit.eject_lane = MagicMock()
        unit.lane_not_ready = MagicMock()
        unit.lane_unload(lane)
        unit.afc.spool.set_spoolID.assert_called_once_with(lane, None)


# ── afcUnit integration hook signatures ──────────────────────────────────────

class TestUnitHookSignatures:
    """Verify the ACE-level hook overrides have the signatures AFC expects."""

    def test_load_sequence_exists(self):
        unit = _make_ace()
        assert callable(getattr(unit, "load_sequence"))

    def test_unload_sequence_exists(self):
        unit = _make_ace()
        assert callable(getattr(unit, "unload_sequence"))

    def test_lane_unload_exists(self):
        unit = _make_ace()
        assert callable(getattr(unit, "lane_unload"))

    def test_prep_capture_td1_exists(self):
        unit = _make_ace()
        assert callable(getattr(unit, "prep_capture_td1"))

    def test_capture_td1_data_exists(self):
        unit = _make_ace()
        assert callable(getattr(unit, "capture_td1_data"))

    def test_get_lane_reset_command_exists(self):
        unit = _make_ace()
        assert callable(getattr(unit, "get_lane_reset_command"))
