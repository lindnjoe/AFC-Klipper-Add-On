"""
Unit tests for the FPS/PFS buffer in extras/AFC_buffer.py

Covers:
  - ADC API dispatch across firmwares by driving the REAL AFCFPSBuffer
    __init__ against fake ADC classes with genuine signatures:
      * Kalico / older Klipper: setup_minmax(sample_time, sample_count) +
        setup_adc_callback(report_time, callback)
      * newer mainline Klipper: setup_adc_sample(report_time, sample_time,
        sample_count) + setup_adc_callback(callback)
      * mid-vintage Klipper: setup_adc_sample(sample_time, sample_count)
    (the arg-order bug jimmy reported would fail these immediately)
  - buffer_triggered fires at homing_high_point (not max_compression) and
    buffer_trailing_triggered at low_point
  - _check_deadband validation helper
  - _adc_callback: reversed inversion, EMA smoothing, advance/trailing state
  - _update_virtual_sensors mirrors advance_state (jimmy 3667d9b)
  - FPSEndstopWrapper: immediate completion, poll-until-trigger, home_wait
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from extras.AFC_buffer import (
    AFCFPSBuffer,
    FPSEndstopWrapper,
    TRAILING_STATE_NAME,
    ADVANCING_STATE_NAME,
    NEUTRAL_STATE_NAME,
    FPS_ENDSTOP_POLL_TIME,
)

from tests.conftest import MockConfig, MockPrinter, MockAFC


# ── Fake ADC objects with real firmware signatures ────────────────────────────

class _AdcBase:
    def __init__(self):
        self.calls = []

    def get_mcu(self):
        mcu = MagicMock()
        mcu.estimated_print_time.return_value = 42.0
        return mcu


class KalicoAdc(_AdcBase):
    """Kalico / older Klipper: setup_minmax + (report_time, callback)."""

    def setup_minmax(self, sample_time, sample_count, minval=0.0, maxval=1.0,
                     range_check_count=0):
        self.calls.append(("setup_minmax", sample_time, sample_count))

    def setup_adc_callback(self, report_time, callback):
        self.calls.append(("setup_adc_callback", report_time))


class NewKlipperAdc(_AdcBase):
    """Newer mainline Klipper: setup_adc_sample(report_time, ...) and a
    callback-only setup_adc_callback."""

    def setup_adc_sample(self, report_time, sample_time=0.0, sample_count=1,
                         batch_num=1, minval=0.0, maxval=1.0,
                         range_check_count=0):
        self.calls.append(
            ("setup_adc_sample", report_time, sample_time, sample_count))

    def setup_adc_callback(self, callback):
        self.calls.append(("setup_adc_callback",))


class MidKlipperAdc(_AdcBase):
    """Mid-vintage: setup_adc_sample WITHOUT report_time; callback takes
    (report_time, callback)."""

    def setup_adc_sample(self, sample_time, sample_count):
        self.calls.append(("setup_adc_sample", sample_time, sample_count))

    def setup_adc_callback(self, report_time, callback):
        self.calls.append(("setup_adc_callback", report_time))


class _FakePins:
    def __init__(self, adc):
        self._adc = adc

    def setup_pin(self, pin_type, pin):
        assert pin_type == "adc"
        return self._adc


# ── Buffer construction through the REAL __init__ ─────────────────────────────

def _make_fps_buffer(adc, values=None):
    afc = MockAFC()
    # AFCBuffer.__init__ reads these LED defaults off the afc object
    afc.led_buffer_advancing = "0,0,1,0"
    afc.led_buffer_trailing = "0,1,0,0"
    afc.led_buffer_neutral = "0,0,0,0.25"
    afc.led_buffer_disabled = "0,0,0,0.25"

    printer = MockPrinter(afc=afc)
    printer._objects["pins"] = _FakePins(adc)

    cfg_values = {"type": "FPS_PFS", "adc_pin": "PB1"}
    cfg_values.update(values or {})
    config = MockConfig(name="AFC_buffer FPS_test", printer=printer,
                        values=cfg_values)
    return AFCFPSBuffer(config)


SAMPLE_TIME_DEFAULT = 0.005
SAMPLE_COUNT_DEFAULT = 5
REPORT_TIME_DEFAULT = 0.100


def test_adc_dispatch_kalico_uses_setup_minmax():
    adc = KalicoAdc()
    _make_fps_buffer(adc)

    assert ("setup_minmax", SAMPLE_TIME_DEFAULT, SAMPLE_COUNT_DEFAULT) in adc.calls
    assert ("setup_adc_callback", REPORT_TIME_DEFAULT) in adc.calls
    # Never tries the mainline API
    assert not any(c[0] == "setup_adc_sample" for c in adc.calls)


def test_adc_dispatch_new_klipper_report_time_first():
    """report_time MUST be the first setup_adc_sample argument on newer
    mainline Klipper — the mis-ordered call was silently mis-sampling."""
    adc = NewKlipperAdc()
    _make_fps_buffer(adc)

    assert ("setup_adc_sample", REPORT_TIME_DEFAULT, SAMPLE_TIME_DEFAULT,
            SAMPLE_COUNT_DEFAULT) in adc.calls
    assert ("setup_adc_callback",) in adc.calls


def test_adc_dispatch_mid_klipper_without_report_time():
    adc = MidKlipperAdc()
    _make_fps_buffer(adc)

    assert ("setup_adc_sample", SAMPLE_TIME_DEFAULT, SAMPLE_COUNT_DEFAULT) in adc.calls
    assert ("setup_adc_callback", REPORT_TIME_DEFAULT) in adc.calls


def test_adc_config_overrides_flow_through():
    adc = NewKlipperAdc()
    _make_fps_buffer(adc, values={"sample_time": 0.002, "sample_count": 4,
                                  "report_time": 0.010})
    assert ("setup_adc_sample", 0.010, 0.002, 4) in adc.calls


# ── Trigger thresholds ────────────────────────────────────────────────────────

def test_buffer_triggered_uses_homing_high_point():
    """jimmy 81ef9e5: homing trips at homing_high_point (default 0.7), NOT at
    high_point/max_compression (0.9) — stops before grinding filament."""
    buf = _make_fps_buffer(KalicoAdc())

    buf.smoothed_fps = 0.75  # above homing_high_point, below high_point
    assert buf.buffer_triggered is True

    buf.smoothed_fps = 0.65
    assert buf.buffer_triggered is False


def test_buffer_trailing_triggered_at_low_point():
    buf = _make_fps_buffer(KalicoAdc())

    buf.smoothed_fps = buf.low_point - 0.01
    assert buf.buffer_trailing_triggered is True

    buf.smoothed_fps = buf.low_point + 0.05
    assert buf.buffer_trailing_triggered is False


# ── _check_deadband ───────────────────────────────────────────────────────────

def test_deadband_ok_returns_empty():
    buf = _make_fps_buffer(KalicoAdc())
    assert buf._check_deadband(0.5, 0.3) == ""


def test_deadband_too_wide_low_side():
    buf = _make_fps_buffer(KalicoAdc())
    msg = buf._check_deadband(0.2, 0.3)  # neutral_low 0.05 <= low_point 0.1
    assert "too wide" in msg


def test_deadband_too_wide_both_sides():
    buf = _make_fps_buffer(KalicoAdc())
    # With defaults low=0.1/high=0.9 a 0.5 setpoint needs deadband < 0.8;
    # a 0.6-wide band at setpoint 0.15 violates the low side only.
    msg = buf._check_deadband(0.15, 0.6)
    assert "neutral_low" in msg


# ── _adc_callback behavior ────────────────────────────────────────────────────

def test_adc_callback_reversed_inverts_reading():
    buf = _make_fps_buffer(KalicoAdc(), values={"reversed": True,
                                                "smoothing": 0.0})
    buf._adc_callback(1.0, 0.2)
    assert buf.fps_value == pytest.approx(0.8)


def test_adc_callback_ema_smoothing():
    buf = _make_fps_buffer(KalicoAdc(), values={"smoothing": 0.5})
    buf.smoothed_fps = 0.5
    buf._adc_callback(1.0, 0.9)
    assert buf.smoothed_fps == pytest.approx(0.5 * 0.5 + 0.5 * 0.9)


def test_adc_callback_batch_list_uses_last_sample():
    buf = _make_fps_buffer(KalicoAdc(), values={"smoothing": 0.0})
    buf._adc_callback([(1.0, 0.2), (2.0, 0.6)])
    assert buf.fps_value == pytest.approx(0.6)


def test_adc_callback_states():
    buf = _make_fps_buffer(KalicoAdc(), values={"smoothing": 0.0})

    buf._adc_callback(1.0, buf.high_point + 0.05)
    assert buf.advance_state is True
    assert buf.last_state == ADVANCING_STATE_NAME

    buf._adc_callback(2.0, buf.set_point)
    assert buf.advance_state is False
    assert buf.last_state == NEUTRAL_STATE_NAME

    buf._adc_callback(3.0, buf.low_point - 0.05)
    assert buf.trailing_state is True
    assert buf.last_state == TRAILING_STATE_NAME


def test_virtual_advance_sensor_mirrors_advance_state():
    """jimmy 3667d9b: the GUI advance sensor mirrors advance_state, not
    'pressure above low_point'."""
    buf = _make_fps_buffer(KalicoAdc(), values={"smoothing": 0.0})

    # Neutral pressure: filament present but NOT advancing -> sensor False
    buf._adc_callback(1.0, buf.set_point)
    assert buf.fila_adv.runout_helper.filament_present is False

    buf._adc_callback(2.0, buf.high_point + 0.05)
    assert buf.fila_adv.runout_helper.filament_present is True


# ── FPSEndstopWrapper ─────────────────────────────────────────────────────────

def _make_wrapper(triggered_values):
    """Wrapper around a stub fps whose trigger function pops values."""
    fps = MagicMock()
    reactor = MagicMock()
    completion = MagicMock()
    reactor.completion.return_value = completion
    fps.reactor = reactor
    fps.adc.get_mcu.return_value.estimated_print_time.return_value = 42.0

    state = {"vals": list(triggered_values)}

    def trigger():
        return state["vals"].pop(0) if state["vals"] else state.setdefault("last", False)

    wrapper = FPSEndstopWrapper(fps, trigger)
    return wrapper, reactor, completion


def test_endstop_already_triggered_completes_immediately():
    wrapper, reactor, completion = _make_wrapper([True])

    wrapper.home_start(0.0, 0.0, 0, 0.0)

    completion.complete.assert_called_once_with(True)
    reactor.register_timer.assert_not_called()
    assert wrapper.home_wait(99.0) == 42.0


def test_endstop_polls_until_triggered():
    wrapper, reactor, completion = _make_wrapper([False, False, True])

    wrapper.home_start(0.0, 0.0, 0, 0.0)
    reactor.register_timer.assert_called_once()
    completion.complete.assert_not_called()

    # Drive the poll callback like the reactor would
    assert wrapper._poll_fps(1.0) == 1.0 + FPS_ENDSTOP_POLL_TIME
    assert wrapper._poll_fps(2.0) == reactor.NEVER
    completion.complete.assert_called_once_with(True)
    assert wrapper._trigger_time == 42.0


def test_endstop_home_wait_unregisters_poll_timer():
    wrapper, reactor, _ = _make_wrapper([False])
    wrapper.home_start(0.0, 0.0, 0, 0.0)

    result = wrapper.home_wait(99.0)

    reactor.unregister_timer.assert_called_once()
    assert result == 0.0  # never triggered


def test_endstop_query():
    wrapper, _, _ = _make_wrapper([True, False])
    assert wrapper.query_endstop(0.0) == 1
    assert wrapper.query_endstop(0.0) == 0
