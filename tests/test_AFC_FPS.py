"""
Unit tests for extras/AFC_FPS.py

Covers:
  - Module-level constants
  - FPSEndstopWrapper: add_stepper, get_steppers, get_mcu, query_endstop
  - AFCFPSBuffer: get_fps_value, buffer_triggered / buffer_trailing_triggered,
    get_fault_sensitivity, enable_buffer / disable_buffer, set_multiplier /
    reset_multiplier, _adc_callback smoothing, extruder_pos_update_event
    multi-extruder guards, get_status
  - VirtualRunoutHelper, VirtualFilamentSensor
"""

from __future__ import annotations

from unittest.mock import MagicMock
import pytest

from extras.AFC_FPS import (
    AFCFPSBuffer,
    FPSEndstopWrapper,
    VirtualFilamentSensor,
    VirtualRunoutHelper,
    ADVANCING_STATE_NAME,
    TRAILING_STATE_NAME,
    NEUTRAL_STATE_NAME,
    CHECK_RUNOUT_TIMEOUT,
    FPS_ENDSTOP_POLL_TIME,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_buffer(name="FPS_1"):
    """Build an AFCFPSBuffer bypassing the complex __init__."""
    buf = AFCFPSBuffer.__new__(AFCFPSBuffer)

    from tests.conftest import MockAFC, MockReactor, MockLogger

    afc = MockAFC()
    afc.toolhead = MagicMock()
    buf.printer = MagicMock()
    buf.afc = afc
    buf.reactor = MockReactor()
    buf.gcode = afc.gcode
    buf.logger = MockLogger()
    buf.name = name

    buf.lanes = {}
    buf.last_state = "Unknown"
    buf.enable = False
    buf.current_lane = None
    buf.advance_state = False
    buf.trailing_state = False
    buf._advance_latched = False
    buf._latch_enabled = False
    buf.toolhead = MagicMock()
    buf.debug = False

    buf.ppins = MagicMock()
    buf.adc = MagicMock()
    buf.sample_count = 5
    buf.sample_time = 0.005
    buf.report_time = 0.1
    buf.reversed = False

    buf.fps_value = 0.5
    buf.set_point = 0.5
    buf.low_point = 0.1
    buf.high_point = 0.9
    buf.multiplier_high = 1.1
    buf.multiplier_low = 0.9
    buf.deadband = 0.3
    buf.smoothing = 0.3
    buf.smoothed_fps = 0.5
    buf.update_interval = 0.25

    buf.error_sensitivity = 0
    buf.fault_sensitivity = 0
    buf.filament_error_pos = None
    buf.past_extruder_position = None
    buf.extruder_pos_timer = None
    buf.fault_timer = None
    buf.min_event_systime = 0.0

    buf.led = False
    buf.led_index = None
    buf.led_advancing = "0,0,1,0"
    buf.led_trailing = "0,1,0,0"
    buf.led_neutral = "0,0.5,0.5,0"
    buf.led_buffer_disabled = "0,0,0,0.25"

    buf.fila_adv = MagicMock()
    buf.fila_trail = MagicMock()

    buf.correction_timer = "correction_timer_obj"

    return buf


def _make_lane(name="lane1", has_stepper=True):
    lane = MagicMock()
    lane.name = name
    lane.extruder_name = "extruder"
    if has_stepper:
        lane.extruder_stepper = MagicMock()
    else:
        lane.extruder_stepper = None
    lane.update_rotation_distance = MagicMock()
    return lane


# ── Module-level constants ────────────────────────────────────────────────────

class TestConstants:
    def test_advancing_state_name(self):
        assert ADVANCING_STATE_NAME == "Advancing"

    def test_trailing_state_name(self):
        assert TRAILING_STATE_NAME == "Trailing"

    def test_neutral_state_name(self):
        assert NEUTRAL_STATE_NAME == "Neutral"

    def test_check_runout_timeout(self):
        assert CHECK_RUNOUT_TIMEOUT > 0

    def test_fps_endstop_poll_time(self):
        assert FPS_ENDSTOP_POLL_TIME > 0


# ── FPSEndstopWrapper ─────────────────────────────────────────────────────────

class TestFPSEndstopWrapper:
    def _make_wrapper(self, trigger=lambda: False):
        buf = _make_buffer()
        return FPSEndstopWrapper(buf, trigger)

    def test_starts_with_no_steppers(self):
        wrap = self._make_wrapper()
        assert wrap.get_steppers() == []

    def test_add_stepper_accumulates(self):
        wrap = self._make_wrapper()
        s1, s2 = MagicMock(), MagicMock()
        wrap.add_stepper(s1)
        wrap.add_stepper(s2)
        assert wrap.get_steppers() == [s1, s2]

    def test_get_steppers_returns_copy(self):
        wrap = self._make_wrapper()
        wrap.add_stepper(MagicMock())
        lst = wrap.get_steppers()
        lst.clear()
        assert len(wrap.get_steppers()) == 1

    def test_query_endstop_returns_1_when_triggered(self):
        wrap = self._make_wrapper(trigger=lambda: True)
        assert wrap.query_endstop(print_time=0.0) == 1

    def test_query_endstop_returns_0_when_not_triggered(self):
        wrap = self._make_wrapper(trigger=lambda: False)
        assert wrap.query_endstop(print_time=0.0) == 0

    def test_get_mcu_delegates_to_adc(self):
        wrap = self._make_wrapper()
        wrap._fps_buffer.adc.get_mcu.return_value = "MCU_X"
        assert wrap.get_mcu() == "MCU_X"


# ── get_fps_value / buffer_triggered / buffer_trailing_triggered ──────────────

class TestBufferReadings:
    def test_get_fps_value_returns_raw(self):
        buf = _make_buffer()
        buf.fps_value = 0.42
        assert buf.get_fps_value() == 0.42

    def test_buffer_triggered_at_high_point(self):
        buf = _make_buffer()
        buf.high_point = 0.9
        buf.smoothed_fps = 0.95
        assert buf.buffer_triggered is True

    def test_buffer_triggered_below_high_point(self):
        buf = _make_buffer()
        buf.high_point = 0.9
        buf.smoothed_fps = 0.85
        assert buf.buffer_triggered is False

    def test_buffer_trailing_triggered_at_low_point(self):
        buf = _make_buffer()
        buf.low_point = 0.1
        buf.smoothed_fps = 0.05
        assert buf.buffer_trailing_triggered is True

    def test_buffer_trailing_triggered_above_low_point(self):
        buf = _make_buffer()
        buf.low_point = 0.1
        buf.smoothed_fps = 0.2
        assert buf.buffer_trailing_triggered is False


# ── get_fault_sensitivity ─────────────────────────────────────────────────────

class TestFaultSensitivity:
    def test_zero_returns_zero(self):
        buf = _make_buffer()
        assert buf.get_fault_sensitivity(0) == 0

    def test_positive_value_scales(self):
        buf = _make_buffer()
        # (11 - 1) * 10 = 100, (11 - 10) * 10 = 10
        assert buf.get_fault_sensitivity(1) == 100
        assert buf.get_fault_sensitivity(10) == 10

    def test_disable_and_restore(self):
        buf = _make_buffer()
        buf.error_sensitivity = 5
        buf.fault_sensitivity = buf.get_fault_sensitivity(5)
        assert buf.fault_sensitivity == 60
        buf.disable_fault_sensitivity()
        assert buf.fault_sensitivity == 0
        buf.restore_fault_sensitivity()
        assert buf.fault_sensitivity == 60

    def test_fault_detection_enabled(self):
        buf = _make_buffer()
        buf.fault_sensitivity = 0
        assert buf.fault_detection_enabled() is False
        buf.fault_sensitivity = 50
        assert buf.fault_detection_enabled() is True


# ── enable_buffer / disable_buffer ────────────────────────────────────────────

class TestEnableDisableBuffer:
    def test_enable_sets_current_lane(self):
        buf = _make_buffer()
        lane = _make_lane()
        buf.enable_buffer(lane)
        assert buf.enable is True
        assert buf.current_lane is lane

    def test_enable_clears_latch(self):
        buf = _make_buffer()
        buf._latch_enabled = True
        buf._advance_latched = True
        buf.enable_buffer(_make_lane())
        assert buf._latch_enabled is False
        assert buf._advance_latched is False

    def test_enable_resets_smoothed_to_raw(self):
        buf = _make_buffer()
        buf.fps_value = 0.77
        buf.smoothed_fps = 0.10
        buf.enable_buffer(_make_lane())
        assert buf.smoothed_fps == 0.77

    def test_disable_clears_current_lane(self):
        buf = _make_buffer()
        lane = _make_lane()
        buf.enable_buffer(lane)
        buf.disable_buffer()
        assert buf.enable is False
        assert buf.current_lane is None

    def test_enable_non_stepper_lane_skips_correction_timer(self):
        buf = _make_buffer()
        lane = _make_lane(has_stepper=False)
        buf.reactor.update_timer = MagicMock()
        buf.enable_buffer(lane)
        # With no stepper, correction timer should not have been started with NOW
        buf.reactor.update_timer.assert_not_called()

    def test_enable_stepper_lane_starts_correction_timer(self):
        buf = _make_buffer()
        lane = _make_lane(has_stepper=True)
        buf.reactor.update_timer = MagicMock()
        buf.enable_buffer(lane)
        buf.reactor.update_timer.assert_called()


# ── Multiplier / latch controls ───────────────────────────────────────────────

class TestMultiplier:
    def test_set_multiplier_noop_when_disabled(self):
        buf = _make_buffer()
        buf.enable = False
        lane = _make_lane()
        buf.current_lane = lane
        buf.set_multiplier(1.2)
        lane.update_rotation_distance.assert_not_called()

    def test_set_multiplier_noop_no_lane(self):
        buf = _make_buffer()
        buf.enable = True
        buf.current_lane = None
        buf.set_multiplier(1.2)  # must not error

    def test_set_multiplier_noop_no_stepper(self):
        buf = _make_buffer()
        buf.enable = True
        buf.current_lane = _make_lane(has_stepper=False)
        buf.set_multiplier(1.2)  # must not error

    def test_set_multiplier_applies(self):
        buf = _make_buffer()
        buf.enable = True
        lane = _make_lane(has_stepper=True)
        buf.current_lane = lane
        buf.set_multiplier(1.25)
        lane.update_rotation_distance.assert_called_once_with(1.25)

    def test_reset_multiplier_applies_1(self):
        buf = _make_buffer()
        lane = _make_lane(has_stepper=True)
        buf.current_lane = lane
        buf.reset_multiplier()
        lane.update_rotation_distance.assert_called_once_with(1)


class TestAdvanceLatch:
    def test_enable_advance_latch(self):
        buf = _make_buffer()
        buf._advance_latched = True
        buf.enable_advance_latch()
        assert buf._latch_enabled is True
        assert buf._advance_latched is False

    def test_clear_advance_latch(self):
        buf = _make_buffer()
        buf._latch_enabled = True
        buf._advance_latched = True
        buf.clear_advance_latch()
        assert buf._latch_enabled is False
        assert buf._advance_latched is False


# ── _adc_callback smoothing / state classification ───────────────────────────

class TestAdcCallback:
    def test_reversed_inverts(self):
        buf = _make_buffer()
        buf.reversed = True
        buf.smoothing = 0.0  # no smoothing
        buf._adc_callback(1.0, 0.2)
        assert buf.fps_value == 0.8
        assert buf.smoothed_fps == pytest.approx(0.8)

    def test_smoothing_applies_ema(self):
        buf = _make_buffer()
        buf.smoothing = 0.5
        buf.smoothed_fps = 0.5
        # new = 0.5*0.5 + 0.5*1.0 = 0.75
        buf._adc_callback(1.0, 1.0)
        assert buf.smoothed_fps == pytest.approx(0.75)

    def test_list_payload_unpacked(self):
        buf = _make_buffer()
        buf.smoothing = 0.0
        buf._adc_callback([(1.0, 0.3)])
        assert buf.fps_value == 0.3

    def test_empty_list_payload_noop(self):
        buf = _make_buffer()
        buf.fps_value = 0.1
        buf._adc_callback([])
        assert buf.fps_value == 0.1

    def test_single_arg_form(self):
        buf = _make_buffer()
        buf.smoothing = 0.0
        buf._adc_callback(0.42)
        assert buf.fps_value == 0.42

    def test_when_disabled_sets_advance_state_above_deadband(self):
        buf = _make_buffer()
        buf.enable = False
        buf.smoothing = 0.0
        buf._adc_callback(1.0, 0.85)  # above set_point + deadband/2 = 0.65
        assert buf.advance_state is True
        assert buf.trailing_state is False
        assert buf.last_state == ADVANCING_STATE_NAME

    def test_when_disabled_sets_trailing_below_deadband(self):
        buf = _make_buffer()
        buf.enable = False
        buf.smoothing = 0.0
        buf._adc_callback(1.0, 0.1)
        assert buf.advance_state is False
        assert buf.trailing_state is True
        assert buf.last_state == TRAILING_STATE_NAME

    def test_when_disabled_neutral_inside_deadband(self):
        buf = _make_buffer()
        buf.enable = False
        buf.smoothing = 0.0
        buf._adc_callback(1.0, 0.5)  # inside deadband
        assert buf.advance_state is False
        assert buf.trailing_state is False
        assert buf.last_state == NEUTRAL_STATE_NAME

    def test_latch_keeps_advance_after_drop(self):
        buf = _make_buffer()
        buf.enable = False
        buf.smoothing = 0.0
        buf._latch_enabled = True
        buf._adc_callback(1.0, 0.95)
        assert buf._advance_latched is True
        # Pressure drops briefly — should still report advance_state True
        buf._adc_callback(2.0, 0.3)
        assert buf.advance_state is True


# ── extruder_pos_update_event guards ──────────────────────────────────────────

class TestExtruderPosUpdateEvent:
    def test_skips_when_lane_has_no_stepper(self):
        buf = _make_buffer()
        lane = _make_lane(has_stepper=False)
        buf.current_lane = lane
        result = buf.extruder_pos_update_event(100.0)
        assert result == 100.0 + CHECK_RUNOUT_TIMEOUT

    def test_skips_when_active_extruder_differs(self):
        buf = _make_buffer()
        lane = _make_lane()
        lane.extruder_name = "extruder_a"
        buf.current_lane = lane
        active = MagicMock()
        active.name = "extruder_b"
        buf.afc.toolhead.get_extruder.return_value = active
        result = buf.extruder_pos_update_event(100.0)
        assert result == 100.0 + CHECK_RUNOUT_TIMEOUT

    def test_triggers_pause_when_extruder_pos_exceeds_threshold(self):
        buf = _make_buffer()
        buf.enable = True
        buf.min_event_systime = 0.0
        buf.filament_error_pos = 50.0
        buf.afc.error = MagicMock()
        buf.afc.function.is_paused.return_value = False
        buf.afc.function.is_printing.return_value = True
        buf.get_extruder_pos = MagicMock(return_value=55.0)
        buf.update_filament_error_pos = MagicMock()
        buf.extruder_pos_update_event(100.0)
        buf.afc.error.AFC_error.assert_called_once()

    def test_returns_eventtime_plus_timeout(self):
        buf = _make_buffer()
        buf.get_extruder_pos = MagicMock(return_value=None)
        result = buf.extruder_pos_update_event(100.0)
        assert result == 100.0 + CHECK_RUNOUT_TIMEOUT


# ── get_status ────────────────────────────────────────────────────────────────

class TestGetStatus:
    def test_includes_fps_metrics(self):
        buf = _make_buffer()
        buf.fps_value = 0.42
        buf.smoothed_fps = 0.43
        buf.set_point = 0.55
        status = buf.get_status(eventtime=100.0)
        assert status["fps_value"] == 0.42
        assert status["smoothed_fps"] == 0.43
        assert status["set_point"] == 0.55

    def test_reports_enabled_and_state(self):
        buf = _make_buffer()
        buf.enable = True
        buf.last_state = NEUTRAL_STATE_NAME
        status = buf.get_status(eventtime=0.0)
        assert status["enabled"] is True
        assert status["state"] == NEUTRAL_STATE_NAME


# ── VirtualRunoutHelper / VirtualFilamentSensor ───────────────────────────────

class TestVirtualRunoutHelper:
    def test_initial_state_not_present(self):
        helper = VirtualRunoutHelper(MagicMock(), "fps_adv")
        status = helper.get_status()
        assert status["filament_detected"] is False

    def test_note_filament_present_true(self):
        helper = VirtualRunoutHelper(MagicMock(), "fps_adv")
        helper.note_filament_present(eventtime=1.0, is_filament_present=True)
        assert helper.get_status()["filament_detected"] is True

    def test_note_filament_present_false(self):
        helper = VirtualRunoutHelper(MagicMock(), "fps_adv")
        helper.note_filament_present(is_filament_present=True)
        helper.note_filament_present(is_filament_present=False)
        assert helper.get_status()["filament_detected"] is False


class TestVirtualFilamentSensor:
    def test_has_runout_helper(self):
        sensor = VirtualFilamentSensor(MagicMock(), "FPS_adv")
        assert sensor.runout_helper is not None

    def test_get_status_exposes_detection(self):
        sensor = VirtualFilamentSensor(MagicMock(), "FPS_adv")
        sensor.runout_helper.note_filament_present(is_filament_present=True)
        status = sensor.get_status(eventtime=0.0)
        assert status.get("filament_detected") is True
