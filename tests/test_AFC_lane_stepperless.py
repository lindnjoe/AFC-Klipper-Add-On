"""
Unit tests for the stepperless-unit delegation hooks in extras/AFC_lane.py

Style: typed in-file unit fakes (no MagicMock unit objects — a missing hook
attribute must genuinely be missing), full state verification, all branches:

  move_advanced   — stepperless delegation (what makes LANE_MOVE work on
      ACE), native stepper path untouched, stepperless-without-hook
      fall-through
  get_td1_data    — unit capture delegation (success/failure tuples), None
      falls through to the stepper path
  _prep_capture_td1 — disabled, unit hook intercept, hook-None fall-through
      to the stepper capture, blocked when hub shows filament / tool loaded
"""

from __future__ import annotations


from extras.AFC_lane import SpeedMode, AssistActive

from tests.test_AFC_lane import _make_afc_lane


# ── Typed unit fakes ──────────────────────────────────────────────────────────

class _Recorder:
    def __init__(self, result=None):
        self.calls = []
        self.result = result

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.result

    @property
    def called(self):
        return len(self.calls) > 0


class _StepperlessUnit:
    """A stepperless unit that provides lane_move (like ACE)."""

    def __init__(self):
        self.stepperless_drive = True
        self.lane_move = _Recorder()


class _StepperlessUnitNoMove:
    """A stepperless unit WITHOUT lane_move — hasattr must be False."""

    def __init__(self):
        self.stepperless_drive = True


class _NativeUnit:
    """A native stepper unit (BoxTurtle-style)."""

    def __init__(self):
        self.stepperless_drive = False
        self.lane_move = _Recorder()  # exists but must never be called


class _CaptureUnit:
    """A unit providing the TD-1 capture hook."""

    def __init__(self, result):
        self.capture_td1_data = _Recorder(result=result)


class _NoHooksUnit:
    """A unit with no optional hooks at all."""


class _PrepHookUnit:
    def __init__(self, result):
        self.prep_capture_td1 = _Recorder(result=result)


class _Hub:
    def __init__(self, state=False):
        self.state = state


# ── move_advanced delegation ──────────────────────────────────────────────────

def _guarded_lane(unit):
    """Lane whose stepper path explodes if taken."""
    lane = _make_afc_lane()
    lane.unit_obj = unit

    def _boom(*a, **k):
        raise AssertionError("stepper path taken")

    lane.get_speed_accel = _boom
    lane.move = _boom
    return lane


def test_move_advanced_delegates_to_unit_lane_move():
    unit = _StepperlessUnit()
    lane = _guarded_lane(unit)

    lane.move_advanced(50.0, SpeedMode.SHORT)

    assert unit.lane_move.calls == [((lane, 50.0, SpeedMode.SHORT), {})]


def test_move_advanced_delegates_negative_distance():
    unit = _StepperlessUnit()
    lane = _guarded_lane(unit)

    lane.move_advanced(-50.0, SpeedMode.LONG, assist_active=AssistActive.YES)

    assert unit.lane_move.calls == [((lane, -50.0, SpeedMode.LONG), {})]


def test_move_advanced_native_stepper_path_untouched():
    unit = _NativeUnit()
    lane = _make_afc_lane()
    lane.unit_obj = unit
    lane.get_speed_accel = _Recorder(result=(50.0, 400.0))
    lane.get_active_assist = _Recorder(result=False)
    lane.move = _Recorder()

    lane.move_advanced(50.0, SpeedMode.SHORT)

    assert not unit.lane_move.called
    assert lane.move.calls == [((50.0, 50.0, 400.0, False), {})]


def test_move_advanced_stepperless_without_lane_move_falls_through():
    """A stepperless unit that doesn't provide lane_move still gets the
    (no-op) stepper path rather than an AttributeError."""
    lane = _make_afc_lane()
    lane.unit_obj = _StepperlessUnitNoMove()
    lane.get_speed_accel = _Recorder(result=(50.0, 400.0))
    lane.get_active_assist = _Recorder(result=False)
    lane.move = _Recorder()

    lane.move_advanced(50.0, SpeedMode.SHORT)

    assert lane.move.calls == [((50.0, 50.0, 400.0, False), {})]


# ── get_td1_data delegation ───────────────────────────────────────────────────

def test_get_td1_data_delegates_to_unit_capture():
    lane = _make_afc_lane()
    unit = _CaptureUnit(result=(True, "captured"))
    lane.unit_obj = unit

    result = lane.get_td1_data()

    assert result == (True, "captured")
    assert unit.capture_td1_data.calls == [((lane,), {})]


def test_get_td1_data_unit_failure_propagates():
    lane = _make_afc_lane()
    lane.unit_obj = _CaptureUnit(result=(False, "ACE not connected"))

    assert lane.get_td1_data() == (False, "ACE not connected")


def test_get_td1_data_hook_none_falls_through_to_stepper_path():
    """A None return means 'not handled' — the base stepper path runs (and
    errors out on the missing td1_device_id in this minimal lane)."""
    lane = _make_afc_lane()
    lane.unit_obj = _CaptureUnit(result=None)
    lane.td1_device_id = None

    status, msg = lane.get_td1_data()

    assert status is False
    assert "td1_device_id" in msg


def test_get_td1_data_no_hook_falls_through():
    lane = _make_afc_lane()
    lane.unit_obj = _NoHooksUnit()
    lane.td1_device_id = None

    status, msg = lane.get_td1_data()

    assert status is False
    assert "td1_device_id" in msg


# ── _prep_capture_td1 hook ────────────────────────────────────────────────────

def test_prep_capture_td1_disabled_does_nothing():
    lane = _make_afc_lane()
    lane.td1_when_loaded = False
    unit = _PrepHookUnit(result=(True, "handled"))
    lane.unit_obj = unit
    lane.get_td1_data = _Recorder()

    lane._prep_capture_td1()

    assert not unit.prep_capture_td1.called
    assert not lane.get_td1_data.called


def test_prep_capture_td1_unit_hook_intercepts():
    lane = _make_afc_lane()
    lane.td1_when_loaded = True
    unit = _PrepHookUnit(result=(True, "handled"))
    lane.unit_obj = unit

    def _boom(*a, **k):
        raise AssertionError("stepper capture taken")

    lane.get_td1_data = _boom

    lane._prep_capture_td1()

    assert unit.prep_capture_td1.calls == [((lane,), {})]


def test_prep_capture_td1_hook_none_falls_through():
    lane = _make_afc_lane()
    lane.td1_when_loaded = True
    lane.unit_obj = _PrepHookUnit(result=None)
    lane.hub_obj = _Hub(state=False)
    lane.afc.function.get_current_lane_obj = _Recorder(result=None)
    lane.get_td1_data = _Recorder()

    lane._prep_capture_td1()

    assert lane.get_td1_data.calls == [((), {})]


def test_prep_capture_td1_blocked_when_hub_shows_filament():
    lane = _make_afc_lane()
    lane.td1_when_loaded = True
    lane.unit_obj = _NoHooksUnit()
    lane.hub_obj = _Hub(state=True)  # filament in path
    lane.afc.function.get_current_lane_obj = _Recorder(result=None)
    lane.get_td1_data = _Recorder()

    lane._prep_capture_td1()

    assert not lane.get_td1_data.called


def test_prep_capture_td1_blocked_when_toolhead_loaded():
    lane = _make_afc_lane()
    lane.td1_when_loaded = True
    lane.unit_obj = _NoHooksUnit()
    lane.hub_obj = _Hub(state=False)
    lane.afc.function.get_current_lane_obj = _Recorder(result=object())
    lane.get_td1_data = _Recorder()

    lane._prep_capture_td1()

    assert not lane.get_td1_data.called
