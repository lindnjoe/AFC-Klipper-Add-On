"""
Unit tests for the stepperless-unit delegation hooks in extras/AFC_lane.py

Covers:
  - move_advanced: delegates to unit_obj.lane_move for stepperless units
    (what makes LANE_MOVE work on ACE; OpenAMS logs instead) and leaves the
    native stepper path untouched otherwise
  - get_td1_data: delegates to unit_obj.capture_td1_data when the unit
    provides it (manual AFC_GET_TD_ONE_LANE_DATA on stepperless units);
    a None return falls through to the stepper path
  - _prep_capture_td1: unit prep_capture_td1 hook intercepts the stepper path
"""

from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_lane import SpeedMode, AssistActive

from tests.test_AFC_lane import _make_afc_lane


# ── move_advanced delegation ──────────────────────────────────────────────────

def _stepperless_lane():
    lane = _make_afc_lane()
    lane.unit_obj = MagicMock()
    lane.unit_obj.stepperless_drive = True
    # If the stepper path ran, these would blow up loudly:
    lane.get_speed_accel = MagicMock(side_effect=AssertionError("stepper path taken"))
    lane.move = MagicMock(side_effect=AssertionError("stepper path taken"))
    return lane


def test_move_advanced_delegates_to_unit_lane_move():
    lane = _stepperless_lane()
    lane.move_advanced(50.0, SpeedMode.SHORT)
    lane.unit_obj.lane_move.assert_called_once_with(lane, 50.0, SpeedMode.SHORT)


def test_move_advanced_delegates_negative_distance():
    lane = _stepperless_lane()
    lane.move_advanced(-50.0, SpeedMode.LONG, assist_active=AssistActive.YES)
    lane.unit_obj.lane_move.assert_called_once_with(lane, -50.0, SpeedMode.LONG)


def test_move_advanced_native_stepper_path_untouched():
    """A native (non-stepperless) unit takes the stepper path; the unit's
    lane_move must NOT be called."""
    lane = _make_afc_lane()
    lane.unit_obj = MagicMock()
    lane.unit_obj.stepperless_drive = False
    lane.get_speed_accel = MagicMock(return_value=(50.0, 400.0))
    lane.get_active_assist = MagicMock(return_value=False)
    lane.move = MagicMock()

    lane.move_advanced(50.0, SpeedMode.SHORT)

    lane.unit_obj.lane_move.assert_not_called()
    lane.move.assert_called_once_with(50.0, 50.0, 400.0, False)


def test_move_advanced_stepperless_without_lane_move_falls_through():
    """A stepperless unit that doesn't provide lane_move still gets the
    (no-op) stepper path rather than an AttributeError."""
    lane = _make_afc_lane()
    unit = MagicMock(spec=["stepperless_drive"])  # no lane_move attribute
    unit.stepperless_drive = True
    lane.unit_obj = unit
    lane.get_speed_accel = MagicMock(return_value=(50.0, 400.0))
    lane.get_active_assist = MagicMock(return_value=False)
    lane.move = MagicMock()

    lane.move_advanced(50.0, SpeedMode.SHORT)

    lane.move.assert_called_once()


# ── get_td1_data delegation ───────────────────────────────────────────────────

def test_get_td1_data_delegates_to_unit_capture():
    lane = _make_afc_lane()
    lane.unit_obj = MagicMock()
    lane.unit_obj.capture_td1_data = MagicMock(return_value=(True, "captured"))

    result = lane.get_td1_data()

    assert result == (True, "captured")
    lane.unit_obj.capture_td1_data.assert_called_once_with(lane)


def test_get_td1_data_unit_failure_propagates():
    lane = _make_afc_lane()
    lane.unit_obj = MagicMock()
    lane.unit_obj.capture_td1_data = MagicMock(
        return_value=(False, "ACE not connected"))

    assert lane.get_td1_data() == (False, "ACE not connected")


def test_get_td1_data_none_falls_through_to_stepper_path():
    """A None return means 'not handled' — the base stepper path runs (and
    errors out on the missing td1_device_id in this minimal lane)."""
    lane = _make_afc_lane()
    lane.unit_obj = MagicMock(spec=[])  # no capture_td1_data at all
    lane.td1_device_id = None

    status, msg = lane.get_td1_data()

    assert status is False
    assert "td1_device_id" in msg


# ── _prep_capture_td1 hook ────────────────────────────────────────────────────

def test_prep_capture_td1_unit_hook_intercepts():
    lane = _make_afc_lane()
    lane.td1_when_loaded = True
    lane.unit_obj = MagicMock()
    lane.unit_obj.prep_capture_td1 = MagicMock(return_value=(True, "handled"))
    lane.get_td1_data = MagicMock(side_effect=AssertionError("stepper path taken"))

    lane._prep_capture_td1()

    lane.unit_obj.prep_capture_td1.assert_called_once_with(lane)


def test_prep_capture_td1_hook_none_falls_through():
    lane = _make_afc_lane()
    lane.td1_when_loaded = True
    lane.unit_obj = MagicMock()
    lane.unit_obj.prep_capture_td1 = MagicMock(return_value=None)
    lane.hub_obj = MagicMock()
    lane.hub_obj.state = False
    lane.afc.function.get_current_lane_obj.return_value = None
    lane.get_td1_data = MagicMock()

    lane._prep_capture_td1()

    lane.get_td1_data.assert_called_once()


def test_prep_capture_td1_disabled_does_nothing():
    lane = _make_afc_lane()
    lane.td1_when_loaded = False
    lane.unit_obj = MagicMock()
    lane.get_td1_data = MagicMock()

    lane._prep_capture_td1()

    lane.unit_obj.prep_capture_td1.assert_not_called()
    lane.get_td1_data.assert_not_called()
