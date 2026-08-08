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

class _AdcMcu:
    def estimated_print_time(self, eventtime):
        return 42.0


class _AdcBase:
    def __init__(self):
        self.calls = []

    def get_mcu(self):
        return _AdcMcu()


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

    cfg_values = {"type": "FPS_PSF", "adc_pin": "PB1"}
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

    # Convention (jimmy's): last_state is the *correction direction* — a LOW
    # reading (spring stretched, no tension) is ADVANCING (feed more), a HIGH
    # reading (spring compressed, ramming the gears) is TRAILING. The
    # advance_state/trailing_state *booleans* follow the turtleneck switch
    # meaning instead: advance_state=True is the "advanced/pressed" position
    # (HIGH), trailing_state=True is the stretched position (LOW). advance_state
    # is what get_toolhead_pre_sensor_state() reports for a PSF buffer.
    buf._adc_callback(1.0, buf.low_point - 0.05)          # LOW / stretched
    assert buf.advance_state is False
    assert buf.trailing_state is True
    assert buf.last_state == ADVANCING_STATE_NAME

    buf._adc_callback(2.0, buf.set_point)
    assert buf.advance_state is False
    assert buf.trailing_state is False
    assert buf.last_state == NEUTRAL_STATE_NAME

    buf._adc_callback(3.0, buf.high_point + 0.05)         # HIGH / compressed
    assert buf.advance_state is True
    assert buf.trailing_state is False
    assert buf.last_state == TRAILING_STATE_NAME


def test_virtual_advance_sensor_mirrors_advance_state():
    """jimmy 3667d9b: the GUI advance sensor mirrors advance_state (the
    'advanced/pressed' position = HIGH reading), not 'pressure above low_point'."""
    buf = _make_fps_buffer(KalicoAdc(), values={"smoothing": 0.0})

    # Neutral: advance_state False -> sensor False
    buf._adc_callback(1.0, buf.set_point)
    assert buf.fila_adv.runout_helper.filament_present is False

    # LOW reading (stretched): advance_state False -> sensor False.
    buf._adc_callback(2.0, buf.low_point - 0.05)
    assert buf.fila_adv.runout_helper.filament_present is False

    # HIGH reading (compressed/pressed): advance_state True -> sensor True.
    buf._adc_callback(3.0, buf.high_point + 0.05)
    assert buf.fila_adv.runout_helper.filament_present is True


# ── FPSEndstopWrapper ─────────────────────────────────────────────────────────

class _WrapperReactor:
    """Typed reactor stand-in: records timer/completion interactions."""
    NOW = 0.0
    NEVER = 9_999_999_999.0

    class _Completion:
        def __init__(self):
            self.completed = []

        def complete(self, value):
            self.completed.append(value)

    def __init__(self):
        self.completions = []
        self.registered_timers = []
        self.unregistered_timers = []

    def monotonic(self):
        return 10.0

    def completion(self):
        comp = self._Completion()
        self.completions.append(comp)
        return comp

    def register_timer(self, callback, waketime=None):
        self.registered_timers.append(callback)
        return callback

    def unregister_timer(self, timer):
        self.unregistered_timers.append(timer)


class _WrapperFps:
    """fps stand-in providing the reactor and the ADC's MCU clock."""

    class _Adc:
        class _Mcu:
            def estimated_print_time(self, eventtime):
                return 42.0

        def get_mcu(self):
            return self._Mcu()

    def __init__(self, reactor):
        self.reactor = reactor
        self.adc = self._Adc()


def _make_wrapper(triggered_values):
    """Wrapper around a stub fps whose trigger function pops values."""
    reactor = _WrapperReactor()
    fps = _WrapperFps(reactor)

    state = {"vals": list(triggered_values)}

    def trigger():
        return state["vals"].pop(0) if state["vals"] else state.setdefault("last", False)

    wrapper = FPSEndstopWrapper(fps, trigger)
    return wrapper, reactor


def test_endstop_already_triggered_completes_immediately():
    wrapper, reactor = _make_wrapper([True])

    wrapper.home_start(0.0, 0.0, 0, 0.0)

    assert reactor.completions[0].completed == [True]
    assert reactor.registered_timers == []      # no polling needed
    assert wrapper._trigger_time == 42.0
    assert wrapper.home_wait(99.0) == 42.0


def test_endstop_polls_until_triggered():
    wrapper, reactor = _make_wrapper([False, False, True])

    wrapper.home_start(0.0, 0.0, 0, 0.0)
    assert len(reactor.registered_timers) == 1
    assert reactor.completions[0].completed == []

    # Drive the poll callback like the reactor would
    assert wrapper._poll_fps(1.0) == 1.0 + FPS_ENDSTOP_POLL_TIME
    assert wrapper._poll_fps(2.0) == reactor.NEVER
    assert reactor.completions[0].completed == [True]
    assert wrapper._trigger_time == 42.0


def test_endstop_home_wait_unregisters_poll_timer():
    wrapper, reactor = _make_wrapper([False])
    wrapper.home_start(0.0, 0.0, 0, 0.0)

    result = wrapper.home_wait(99.0)

    assert len(reactor.unregistered_timers) == 1
    assert wrapper._poll_timer is None
    assert result == 0.0  # never triggered


def test_endstop_query():
    wrapper, _ = _make_wrapper([True, False])
    assert wrapper.query_endstop(0.0) == 1
    assert wrapper.query_endstop(0.0) == 0


# ── stepper lane + disabled buffer must still refresh state (calibration fix) ──

def test_stepper_lane_disabled_buffer_refreshes_advance_state():
    # Regression: on a stepper lane (BoxTurtle) with the buffer DISABLED (during
    # bowden calibration / load), _adc_callback must still refresh advance/trailing
    # from the live fps. Gating it behind `not has_stepper` freezes the state
    # stale, so get_toolhead_pre_sensor_state() never reports filament arrival
    # and the ramming home-to-buffer calibration loops forever.
    # (Boolean convention is jimmy's: HIGH/compressed -> advance_state True.)
    buf = _make_fps_buffer(KalicoAdc(),
                           values={"smoothing": 0.0, "set_point": 0.5,
                                   "deadband": 0.1})
    buf._lane_has_rotation_control = lambda lane: True   # BoxTurtle-style stepper
    buf.enable = False                                   # buffer disabled (calibration)
    buf._correction_running = False

    buf.advance_state = False
    buf.trailing_state = False
    buf._adc_callback(1.0, 0.1)                          # LOW: stretched -> ADVANCING
    assert buf.last_state == ADVANCING_STATE_NAME        # refreshed, not stale
    assert buf.trailing_state is True
    assert buf.advance_state is False

    buf._adc_callback(1.0, 0.9)                          # HIGH: compressed/pressed
    assert buf.last_state == TRAILING_STATE_NAME
    assert buf.advance_state is True
    assert buf.trailing_state is False


def test_stepper_lane_active_correction_owns_state():
    # When the correction loop IS actively driving (enabled + stepper + running),
    # the ADC callback must NOT overwrite advance/trailing — the timer owns it.
    buf = _make_fps_buffer(KalicoAdc(),
                           values={"smoothing": 0.0, "set_point": 0.5,
                                   "deadband": 0.1})
    buf._lane_has_rotation_control = lambda lane: True
    buf.enable = True
    buf._correction_running = True

    buf.advance_state = False
    buf.trailing_state = False
    buf._adc_callback(1.0, 0.1)                          # would advance, but...
    assert buf.advance_state is False                    # ...left to the correction loop
    assert buf.trailing_state is False
