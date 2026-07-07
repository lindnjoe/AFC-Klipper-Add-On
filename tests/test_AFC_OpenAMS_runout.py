"""
Unit tests for afcAMS runout handling in extras/AFC_OpenAMS.py

Covers:
  - handle_runout: no runout lane -> pause + handled; unresolvable runout
    lane -> pause + handled; same-extruder -> seamless same-FPS reload;
    cross-extruder -> _oams_runout_empty flag + defer to generic logic
  - _is_same_extruder: name matching incl. case/whitespace and missing objs
  - _resolve_lane_reference: exact, case-insensitive, missing
  - _should_block_sensor_for_runout: suppression window semantics
  - FollowerController: constructor contract (the e1a2da0 wrong-args bug)
"""

from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_OpenAMS import afcAMS, FollowerController, FollowerState
from extras.AFC_lane import AFCLaneState

from tests.conftest import MockReactor


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_lane(name, extruder_name="extruder", runout_lane=None,
               tool_loaded=False, status=None):
    lane = MagicMock()
    lane.name = name
    lane.runout_lane = runout_lane
    lane.tool_loaded = tool_loaded
    lane.status = status
    lane._oams_runout_detected = False
    lane._oams_runout_empty = False
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.name = extruder_name
    return lane


def _make_unit(lanes=None):
    unit = afcAMS.__new__(afcAMS)
    unit.name = "AMS_1"
    unit.logger = MagicMock()
    unit.afc = MagicMock()
    unit.afc.lanes = lanes or {}
    unit.lane_not_ready = MagicMock()
    unit.handle_same_fps_reload = MagicMock()
    return unit


# ── handle_runout classification ──────────────────────────────────────────────

def test_runout_without_runout_lane_pauses_and_handles():
    lane = _make_lane("lane0")
    unit = _make_unit({"lane0": lane})

    handled = unit.handle_runout(lane)

    assert handled is True
    assert lane.status == AFCLaneState.NONE
    unit.lane_not_ready.assert_called_once_with(lane)
    unit.afc.error.AFC_error.assert_called_once()
    assert unit.afc.error.AFC_error.call_args.kwargs.get("pause") is True
    unit.handle_same_fps_reload.assert_not_called()


def test_runout_with_unresolvable_target_pauses_and_handles():
    lane = _make_lane("lane0", runout_lane="lane_missing")
    unit = _make_unit({"lane0": lane})

    handled = unit.handle_runout(lane)

    assert handled is True
    unit.afc.error.AFC_error.assert_called_once()
    assert "not found" in unit.afc.error.AFC_error.call_args[0][0]
    unit.handle_same_fps_reload.assert_not_called()


def test_same_extruder_runout_does_seamless_reload():
    lane = _make_lane("lane0", extruder_name="extruder", runout_lane="lane1")
    target = _make_lane("lane1", extruder_name="extruder")
    unit = _make_unit({"lane0": lane, "lane1": target})

    handled = unit.handle_runout(lane)

    assert handled is True
    assert lane._oams_runout_detected is True
    unit.handle_same_fps_reload.assert_called_once_with(lane, target)
    unit.afc.error.AFC_error.assert_not_called()


def test_cross_extruder_runout_defers_to_generic_infinite_spool():
    lane = _make_lane("lane0", extruder_name="extruder", runout_lane="lane4")
    target = _make_lane("lane4", extruder_name="extruder4")
    unit = _make_unit({"lane0": lane, "lane4": target})

    handled = unit.handle_runout(lane)

    assert handled is False  # generic _perform_infinite_runout takes over
    assert lane._oams_runout_empty is True  # hardware unload will be skipped
    unit.handle_same_fps_reload.assert_not_called()
    unit.afc.error.AFC_error.assert_not_called()


# ── _is_same_extruder ─────────────────────────────────────────────────────────

def test_is_same_extruder_matches_case_and_whitespace():
    unit = _make_unit()
    a = _make_lane("a", extruder_name=" Extruder ")
    b = _make_lane("b", extruder_name="extruder")
    assert unit._is_same_extruder(a, b) is True


def test_is_same_extruder_differs():
    unit = _make_unit()
    a = _make_lane("a", extruder_name="extruder")
    b = _make_lane("b", extruder_name="extruder4")
    assert unit._is_same_extruder(a, b) is False


def test_is_same_extruder_missing_extruder_obj():
    unit = _make_unit()
    a = _make_lane("a")
    a.extruder_obj = None
    b = _make_lane("b")
    assert unit._is_same_extruder(a, b) is False


# ── _resolve_lane_reference ───────────────────────────────────────────────────

def test_resolve_lane_exact_match():
    lane = _make_lane("lane0")
    unit = _make_unit({"lane0": lane})
    assert unit._resolve_lane_reference("lane0") is lane


def test_resolve_lane_case_insensitive_fallback():
    lane = _make_lane("Lane0")
    unit = _make_unit({"Lane0": lane})
    assert unit._resolve_lane_reference("lane0") is lane


def test_resolve_lane_missing_and_empty():
    unit = _make_unit({})
    assert unit._resolve_lane_reference("nope") is None
    assert unit._resolve_lane_reference(None) is None
    assert unit._resolve_lane_reference("") is None


# ── _should_block_sensor_for_runout ───────────────────────────────────────────

def _runout_lane(status=AFCLaneState.INFINITE_RUNOUT):
    lane = _make_lane("lane0", tool_loaded=True, status=status)
    lane._oams_runout_detected = True
    return lane


def test_sensor_block_active_runout_blocks_true_updates():
    unit = _make_unit()
    unit.afc.function.is_printing.return_value = True
    lane = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is True
    # Flag stays set while blocking True updates
    assert lane._oams_runout_detected is True


def test_sensor_block_false_update_clears_flag():
    unit = _make_unit()
    unit.afc.function.is_printing.return_value = True
    lane = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, False) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_not_printing_clears_flag():
    unit = _make_unit()
    unit.afc.function.is_printing.return_value = False
    lane = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_noop_without_flag():
    unit = _make_unit()
    lane = _make_lane("lane0")
    assert unit._should_block_sensor_for_runout(lane, True) is False


# ── FollowerController constructor contract ──────────────────────────────────
# The e1a2da0 bug was a call-site/constructor mismatch:
# FollowerController(oams_obj, printer, logger) vs (oams_dict, reactor, logger).
# Lock in the contract the way afcAMS._init_follower_and_monitor constructs it.

def test_follower_controller_constructed_like_call_site():
    oams = MagicMock()
    reactor = MockReactor()
    logger = MagicMock()

    follower = FollowerController({"oams1": oams}, reactor, logger)

    assert follower.oams == {"oams1": oams}
    assert follower.reactor is reactor
    assert follower.logger is logger


def test_follower_state_created_on_demand():
    follower = FollowerController({"oams1": MagicMock()}, MockReactor(), MagicMock())
    state = follower.get_follower_state("oams1")
    assert isinstance(state, FollowerState)
    # Same object returned on subsequent calls
    assert follower.get_follower_state("oams1") is state
