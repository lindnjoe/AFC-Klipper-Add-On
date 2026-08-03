"""
Branch-coverage unit tests for extras/temperature_ace.py.

These complement tests/test_temperature_ace.py (which covers the channel-read
logic of ``_sample_ace_temperature``). Here the sensor is built through its real
``__init__`` (mocking config/printer/reactor via conftest) so construction, the
Klipper sensor-interface accessors, ``handle_ready``, ``_resolve_unit``, the
shutdown/exception/callback branches of sampling, ``get_status`` and the module
load hooks are all exercised.
"""

from __future__ import annotations

import logging
from unittest.mock import MagicMock

import extras.temperature_ace as tace
from extras.temperature_ace import (
    ACE_REPORT_TIME,
    TemperatureACE,
    _fallback_logger,
)
from tests.conftest import MockAFC, MockConfig, MockLogger, MockPrinter


# ── Helpers ───────────────────────────────────────────────────────────────────

class _AceConfig(MockConfig):
    """MockConfig plus the ``getchoice`` accessor temperature_ace needs."""

    def getchoice(self, option, choices, default=None, **kwargs):
        val = self._values.get(option, default)
        return choices[val]


class _FakeUnit:
    """Stand-in for an AFC_ACE unit with the attributes the sensor reads."""

    def __init__(self, unit_type="ACE", hw_status=None, temp_info=None):
        self.type = unit_type
        self._cached_hw_status = hw_status or {}
        self._cached_temp_info = temp_info or {}


def _make_ace(values=None, afc=None, debug=False):
    """Build a TemperatureACE through its real constructor."""
    if afc is None:
        afc = MockAFC()
    printer = MockPrinter(afc=afc)
    printer.add_object = MagicMock()
    if debug:
        printer.start_args = {"debugoutput": "/dev/null"}
    config = _AceConfig(
        name="temperature_sensor ace_temp", printer=printer, values=values or {})
    return TemperatureACE(config)


# ── __init__ ──────────────────────────────────────────────────────────────────

class TestInit:
    def test_name_and_defaults(self):
        sensor = _make_ace()
        assert sensor.name == "ace_temp"
        assert sensor.ace_unit_name == "Ace1"
        assert sensor.channel == "default"
        assert sensor.temp == 0.0
        assert sensor.min_temp == 0.0
        assert sensor.max_temp == 70.0
        assert sensor.measured_min == float("inf")
        assert sensor.measured_max == 0.0
        assert sensor.humidity == 0.0
        assert sensor._has_humidity is False

    def test_custom_ace_unit_and_channel(self):
        sensor = _make_ace(values={"ace_unit": "MyAce", "channel": "ptc1"})
        assert sensor.ace_unit_name == "MyAce"
        assert sensor.channel == "ptc1"

    def test_simulate_aht3x_true_registers_aht10_object(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        printer.add_object = MagicMock()
        config = _AceConfig(name="temperature_sensor ace_temp", printer=printer)
        sensor = TemperatureACE(config)
        printer.add_object.assert_called_once_with("aht10 ace_temp", sensor)

    def test_simulate_aht3x_false_registers_temperature_ace_object(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        printer.add_object = MagicMock()
        config = _AceConfig(
            name="temperature_sensor ace_temp", printer=printer,
            values={"simulate_supported_sensor_mainsail": False})
        sensor = TemperatureACE(config)
        printer.add_object.assert_called_once_with("temperature_ace ace_temp", sensor)

    def test_non_debug_registers_timer_and_ready_handler(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        printer.add_object = MagicMock()
        config = _AceConfig(name="temperature_sensor ace_temp", printer=printer)
        sensor = TemperatureACE(config)
        assert hasattr(sensor, "sample_timer")
        assert sensor.handle_ready in printer._event_handlers["klippy:ready"]

    def test_debug_mode_skips_timer_and_handler(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        printer.add_object = MagicMock()
        printer.start_args = {"debugoutput": "/dev/null"}
        config = _AceConfig(name="temperature_sensor ace_temp", printer=printer)
        sensor = TemperatureACE(config)
        assert not hasattr(sensor, "sample_timer")
        assert "klippy:ready" not in printer._event_handlers


# ── _log ──────────────────────────────────────────────────────────────────────

class TestLog:
    def test_returns_afc_logger_when_set(self):
        sensor = _make_ace()
        afc_logger = MockLogger()
        sensor._logger = afc_logger
        assert sensor._log() is afc_logger

    def test_returns_fallback_logger_when_unset(self):
        sensor = _make_ace()
        sensor._logger = None
        assert sensor._log() is _fallback_logger


# ── handle_ready ──────────────────────────────────────────────────────────────

class TestHandleReady:
    def test_ace2_unit_sets_humidity_and_logs_link(self):
        afc = MockAFC()
        afc.units["Ace1"] = _FakeUnit(unit_type="ACE2")
        sensor = _make_ace(afc=afc)
        sensor.reactor.update_timer = MagicMock()

        sensor.handle_ready()

        assert sensor._has_humidity is True
        assert afc.logger.messages == [
            ("info", "temperature_ace: linked to AFC_ACE unit 'Ace1'")]
        sensor.reactor.update_timer.assert_called_once_with(
            sensor.sample_timer, sensor.reactor.NOW)

    def test_non_ace2_unit_keeps_humidity_false_and_logs_link(self):
        afc = MockAFC()
        afc.units["Ace1"] = _FakeUnit(unit_type="ACE")
        sensor = _make_ace(afc=afc)

        sensor.handle_ready()

        assert sensor._has_humidity is False
        assert afc.logger.messages == [
            ("info", "temperature_ace: linked to AFC_ACE unit 'Ace1'")]

    def test_missing_unit_logs_warning(self):
        afc = MockAFC()  # no unit registered
        sensor = _make_ace(afc=afc)

        sensor.handle_ready()

        assert sensor._ace_unit is None
        assert afc.logger.messages == [
            ("warning",
             "temperature_ace: AFC_ACE unit 'Ace1' not found; reporting 0C")]

    def test_afc_logger_lookup_failure_uses_fallback(self, caplog):
        sensor = _make_ace()
        unit = _FakeUnit(unit_type="ACE")
        sensor._resolve_unit = MagicMock(return_value=unit)
        sensor.printer.lookup_object = MagicMock(side_effect=Exception("no afc"))
        sensor.reactor.update_timer = MagicMock()

        with caplog.at_level(logging.INFO, logger="temperature_ace"):
            sensor.handle_ready()

        assert sensor._logger is None
        msgs = [r.getMessage() for r in caplog.records if r.name == "temperature_ace"]
        assert msgs == ["temperature_ace: linked to AFC_ACE unit 'Ace1'"]

    def test_no_sample_timer_skips_update_timer(self):
        afc = MockAFC()
        afc.units["Ace1"] = _FakeUnit(unit_type="ACE")
        sensor = _make_ace(afc=afc, debug=True)  # no sample_timer registered
        sensor.reactor.update_timer = MagicMock()

        sensor.handle_ready()

        sensor.reactor.update_timer.assert_not_called()
        assert afc.logger.messages == [
            ("info", "temperature_ace: linked to AFC_ACE unit 'Ace1'")]


# ── setup_minmax / setup_callback / get_report_time_delta ─────────────────────

class TestSetupMinmax:
    def test_sets_bounds(self):
        sensor = _make_ace()
        sensor.setup_minmax(5.0, 65.0)
        assert sensor.min_temp == 5.0
        assert sensor.max_temp == 65.0


class TestSetupCallback:
    def test_sets_callback(self):
        sensor = _make_ace()
        cb = MagicMock()
        sensor.setup_callback(cb)
        assert sensor._callback is cb


class TestGetReportTimeDelta:
    def test_returns_report_time(self):
        sensor = _make_ace()
        assert sensor.get_report_time_delta() == ACE_REPORT_TIME


# ── _resolve_unit ─────────────────────────────────────────────────────────────

class TestResolveUnit:
    def test_returns_unit_from_afc_units(self):
        afc = MockAFC()
        unit = _FakeUnit()
        afc.units["Ace1"] = unit
        sensor = _make_ace(afc=afc)
        assert sensor._resolve_unit() is unit

    def test_falls_back_to_direct_printer_lookup(self):
        afc = MockAFC()  # units empty
        sensor = _make_ace(afc=afc)
        unit = _FakeUnit()
        sensor.printer._objects["AFC_ACE Ace1"] = unit
        assert sensor._resolve_unit() is unit

    def test_returns_none_when_unit_absent_everywhere(self):
        afc = MockAFC()  # units empty and no printer object
        sensor = _make_ace(afc=afc)
        assert sensor._resolve_unit() is None

    def test_returns_none_when_lookup_raises(self):
        sensor = _make_ace()
        sensor.printer.lookup_object = MagicMock(side_effect=Exception("boom"))
        assert sensor._resolve_unit() is None


# ── _sample_ace_temperature ───────────────────────────────────────────────────

class TestSampleAceTemperature:
    def test_resolves_unit_when_none_then_reads_temp(self):
        afc = MockAFC()
        unit = _FakeUnit(hw_status={"temp": 30.0})
        afc.units["Ace1"] = unit
        sensor = _make_ace(afc=afc)
        sensor._ace_unit = None
        sensor._callback = None

        sensor._sample_ace_temperature(0.0)

        assert sensor._ace_unit is unit
        assert sensor.temp == 30.0

    def test_no_unit_sets_temp_zero(self):
        afc = MockAFC()  # nothing to resolve
        sensor = _make_ace(afc=afc)
        sensor._ace_unit = None
        sensor.temp = 5.0
        sensor._callback = None

        sensor._sample_ace_temperature(0.0)

        assert sensor.temp == 0.0

    def test_shutdown_below_minimum(self):
        sensor = _make_ace()
        sensor._ace_unit = _FakeUnit(hw_status={"temp": 40.0})
        sensor.min_temp = 50.0
        sensor.max_temp = 1000.0
        sensor._callback = None
        sensor.printer.invoke_shutdown = MagicMock()

        sensor._sample_ace_temperature(0.0)

        sensor.printer.invoke_shutdown.assert_called_once_with(
            "ACE temperature 40.0 below minimum of 50.0")

    def test_no_min_shutdown_when_temp_above_minimum(self):
        sensor = _make_ace()
        sensor._ace_unit = _FakeUnit(hw_status={"temp": 60.0})
        sensor.min_temp = 50.0
        sensor.max_temp = 1000.0
        sensor._callback = None
        sensor.printer.invoke_shutdown = MagicMock()

        sensor._sample_ace_temperature(0.0)

        sensor.printer.invoke_shutdown.assert_not_called()

    def test_no_min_shutdown_when_temp_not_positive(self):
        # temp == 0 is below min_temp but ``temp > 0`` is False, so the min
        # guard must not fire (proves the first sub-condition independently).
        sensor = _make_ace()
        sensor._ace_unit = _FakeUnit(hw_status={"temp": 0.0})
        sensor.min_temp = 50.0
        sensor.max_temp = 1000.0
        sensor._callback = None
        sensor.printer.invoke_shutdown = MagicMock()

        sensor._sample_ace_temperature(0.0)

        assert sensor.temp == 0.0
        sensor.printer.invoke_shutdown.assert_not_called()

    def test_shutdown_above_maximum(self):
        sensor = _make_ace()
        sensor._ace_unit = _FakeUnit(hw_status={"temp": 80.0})
        sensor.min_temp = 0.0
        sensor.max_temp = 70.0
        sensor._callback = None
        sensor.printer.invoke_shutdown = MagicMock()

        sensor._sample_ace_temperature(0.0)

        sensor.printer.invoke_shutdown.assert_called_once_with(
            "ACE temperature 80.0 above maximum of 70.0")

    def test_no_max_shutdown_when_temp_at_maximum(self):
        sensor = _make_ace()
        sensor._ace_unit = _FakeUnit(hw_status={"temp": 70.0})
        sensor.min_temp = 0.0
        sensor.max_temp = 70.0
        sensor._callback = None
        sensor.printer.invoke_shutdown = MagicMock()

        sensor._sample_ace_temperature(0.0)

        sensor.printer.invoke_shutdown.assert_not_called()

    def test_exception_logs_once_then_suppresses(self):
        sensor = _make_ace()
        sensor._ace_unit = _FakeUnit(hw_status={"temp": "bad"})  # float() raises
        sensor._callback = None
        logger = MockLogger()
        sensor._logger = logger
        sensor.temp = 12.0

        sensor._sample_ace_temperature(0.0)

        assert sensor.temp == 0.0
        assert sensor._sample_error_logged is True
        assert logger.messages == [
            ("error",
             "temperature_ace: error sampling ACE temperature: "
             "could not convert string to float: 'bad'")]

        sensor._sample_ace_temperature(0.0)  # second failure must not log again
        assert logger.messages == [
            ("error",
             "temperature_ace: error sampling ACE temperature: "
             "could not convert string to float: 'bad'")]

    def test_callback_invoked_with_temperature(self):
        sensor = _make_ace()
        sensor._ace_unit = _FakeUnit(hw_status={"temp": 25.0})
        sensor.min_temp = 0.0
        sensor.max_temp = 1000.0
        cb = MagicMock()
        sensor._callback = cb

        result = sensor._sample_ace_temperature(0.0)

        assert sensor.temp == 25.0
        assert cb.call_count == 1
        assert cb.call_args.args[1] == 25.0
        assert result == 0.0 + ACE_REPORT_TIME


# ── get_temp ──────────────────────────────────────────────────────────────────

class TestGetTemp:
    def test_returns_temp_and_zero_error(self):
        sensor = _make_ace()
        sensor.temp = 33.3
        assert sensor.get_temp(0.0) == (33.3, 0.0)


# ── stats ─────────────────────────────────────────────────────────────────────

class TestStats:
    def test_returns_false_and_status_line(self):
        sensor = _make_ace()
        sensor.temp = 44.4
        assert sensor.stats(0.0) == (False, "temperature_ace ace_temp: temp=44.4")


# ── get_status ────────────────────────────────────────────────────────────────

class TestGetStatus:
    def test_without_humidity(self):
        sensor = _make_ace()
        sensor.temp = 21.239
        sensor._has_humidity = False
        assert sensor.get_status(0.0) == {"temperature": 21.24}

    def test_with_humidity(self):
        sensor = _make_ace()
        sensor.temp = 21.239
        sensor.humidity = 55.678
        sensor._has_humidity = True
        assert sensor.get_status(0.0) == {"temperature": 21.24, "humidity": 55.68}


# ── load_config / _register_sensor_factory ────────────────────────────────────

class TestLoadConfig:
    def test_registers_both_factories(self):
        tace._REGISTERED = False
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        heaters = MagicMock()
        printer._objects["heaters"] = heaters
        config = MockConfig(name="dummy", printer=printer)

        tace.load_config(config)

        assert heaters.add_sensor_factory.call_count == 2
        heaters.add_sensor_factory.assert_any_call("temperature_ace", TemperatureACE)
        heaters.add_sensor_factory.assert_any_call("aht2x", TemperatureACE)
        assert tace._REGISTERED is True


class TestRegisterSensorFactory:
    def test_already_registered_returns_early(self):
        tace._REGISTERED = True
        printer = MockPrinter(afc=MockAFC())
        heaters = MagicMock()
        printer._objects["heaters"] = heaters

        tace._register_sensor_factory(printer)

        heaters.add_sensor_factory.assert_not_called()

    def test_load_object_fallback_when_lookup_fails(self):
        tace._REGISTERED = False
        printer = MockPrinter(afc=MockAFC())
        heaters = MagicMock()
        printer.lookup_object = MagicMock(side_effect=Exception("no heaters"))
        printer.load_object = MagicMock(return_value=heaters)

        tace._register_sensor_factory(printer)

        assert heaters.add_sensor_factory.call_count == 2
        assert tace._REGISTERED is True

    def test_load_object_failure_logs_warning(self, caplog):
        tace._REGISTERED = False
        printer = MockPrinter(afc=MockAFC())
        printer.lookup_object = MagicMock(side_effect=Exception("no heaters"))
        printer.load_object = MagicMock(side_effect=Exception("boom"))

        with caplog.at_level(logging.WARNING, logger="temperature_ace"):
            tace._register_sensor_factory(printer)

        msgs = [r.getMessage() for r in caplog.records if r.name == "temperature_ace"]
        assert msgs == ["temperature_ace: failed to load heaters: boom"]
        assert tace._REGISTERED is False
