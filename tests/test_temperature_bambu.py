# Tests for extras/temperature_bambu.py — the [temperature_sensor] adapter that
# puts a Bambu AMS's humidity and drying-chamber temperature on the
# Mainsail/Fluidd temperature card.
#
# The behaviour worth pinning here is all about what this sensor must NOT do.
# It sits in the heaters system, which shuts the printer down on an
# out-of-range reading, and it reports a value that is legitimately absent most
# of the time: a plain AMS has no dryer at all, and an AMS 2 only produces a
# chamber temperature while its controller is talking. Reporting 0.0 into a
# min_temp of 10 would read as a sensor fault and halt a running print over a
# display value.
from __future__ import annotations

import types

import pytest

from extras.temperature_bambu import TemperatureBambu


class _Reactor:
    def __init__(self):
        self.t = 100.0
        self.timers = []

    def monotonic(self):
        return self.t

    def register_timer(self, cb, when=None):
        self.timers.append(cb)
        return cb

    NOW = 0.0


class _Printer:
    def __init__(self, unit=None):
        self.reactor = _Reactor()
        self._unit = unit
        self.shutdowns = []
        self._handlers = {}

    def get_reactor(self):
        return self.reactor

    def register_event_handler(self, name, cb):
        self._handlers[name] = cb

    def lookup_object(self, name, default="__raise__"):
        if name == "mcu":
            return types.SimpleNamespace(estimated_print_time=lambda t: t)
        if name == "heaters":
            return types.SimpleNamespace(add_sensor_factory=lambda *a: None)
        if self._unit is not None and "BambuAMS" in name:
            return self._unit
        if default == "__raise__":
            raise Exception("no such object %s" % name)
        return default

    def lookup_objects(self, prefix):
        if self._unit is not None and prefix.startswith("AFC_BambuAMS"):
            return [("AFC_BambuAMS u", self._unit)]
        return []

    def add_object(self, name, obj):
        self.objects = getattr(self, "objects", {})
        if name in self.objects:
            raise Exception("object %s already registered" % name)
        self.objects[name] = obj

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)


class _Config:
    def __init__(self, printer, **opts):
        self._p = printer
        self._o = opts

    def get_printer(self):
        return self._p

    def get_name(self):
        return "temperature_sensor BambuAMS_1"

    def get(self, key, default=None):
        return self._o.get(key, default)

    def getint(self, key, default=None, **kw):
        return int(self._o.get(key, default))

    def getfloat(self, key, default=None, **kw):
        return float(self._o.get(key, default))

    def getboolean(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return default if v is None else bool(v)


def _unit(**status):
    return types.SimpleNamespace(
        get_status=lambda et=None: dict(status),
        has_heater=status.pop("_has_heater", True))


def _make(printer, **opts):
    s = TemperatureBambu(_Config(printer, **opts))
    s.setup_minmax(10.0, 90.0)
    return s


class TestNeverShutsDownOnAMissingReading:
    """A unit with no dryer, or one that has not spoken yet, must not halt the
    printer. This sensor lives in the heaters system, where an out-of-range
    value is a shutdown."""

    def test_no_temperature_reported_does_not_shut_down(self):
        p = _Printer(_unit(humidity=45))
        s = _make(p, bambu_unit="BambuAMS_1")
        s._sample(1.0)
        assert p.shutdowns == []

    def test_callback_reports_min_temp_not_zero_before_any_reading(self):
        # 0.0 against a min_temp of 10 looks exactly like a failed sensor.
        p = _Printer(_unit(humidity=45))
        s = _make(p, bambu_unit="BambuAMS_1")
        seen = []
        s.setup_callback(lambda t, v: seen.append(v))
        s._sample(1.0)
        assert seen == [10.0]

    def test_a_real_reading_out_of_range_does_shut_down(self):
        # The guard is for ABSENT readings, not implausible ones.
        p = _Printer(_unit(humidity=40, temperature=140.0))
        s = _make(p, bambu_unit="BambuAMS_1")
        s._sample(1.0)
        assert len(p.shutdowns) == 1
        assert "outside range" in p.shutdowns[0]

    def test_a_real_reading_in_range_is_reported(self):
        p = _Printer(_unit(humidity=40, temperature=55.0))
        s = _make(p, bambu_unit="BambuAMS_1")
        seen = []
        s.setup_callback(lambda t, v: seen.append(v))
        s._sample(1.0)
        assert seen == [55.0]
        assert p.shutdowns == []

    def test_unit_that_raises_is_swallowed(self):
        # A status error must not propagate into the heaters timer.
        bad = types.SimpleNamespace(
            has_heater=True,
            get_status=lambda et=None: (_ for _ in ()).throw(RuntimeError("x")))
        p = _Printer(bad)
        s = _make(p, bambu_unit="BambuAMS_1")
        s._sample(1.0)          # must not raise
        assert p.shutdowns == []


class TestStatusOmitsTemperatureOnAHeaterlessUnit:
    """A plain AMS has no chamber. Publishing a flat 0.0 beside a live humidity
    reads as a broken sensor rather than an absent feature."""

    def test_heaterless_unit_omits_temperature(self):
        u = types.SimpleNamespace(has_heater=False,
                                  get_status=lambda et=None: {"humidity": 44})
        p = _Printer(u)
        s = _make(p, bambu_unit="BambuAMS_1")
        s._sample(1.0)
        st = s.get_status(1.0)
        assert st["humidity"] == 44.0
        assert "temperature" not in st

    def test_heater_capable_unit_publishes_temperature(self):
        p = _Printer(_unit(humidity=44, temperature=31.25))
        s = _make(p, bambu_unit="BambuAMS_1")
        s._sample(1.0)
        st = s.get_status(1.0)
        assert st["temperature"] == 31.2 or st["temperature"] == 31.3
        assert st["humidity"] == 44.0


class TestReportTime:
    def test_default_and_override(self):
        # A fresh printer per sensor: two sensors of the same name would
        # collide on the "aht10 <name>" registration, exactly as they would
        # in Klipper.
        assert _make(_Printer(_unit(humidity=1)),
                     bambu_unit="u").get_report_time_delta() == 5
        assert _make(_Printer(_unit(humidity=1)), bambu_unit="u",
                     report_time=2).get_report_time_delta() == 2

    def test_sample_schedules_the_next_run(self):
        p = _Printer(_unit(humidity=1))
        s = _make(p, bambu_unit="BambuAMS_1", report_time=3)
        nxt = s._sample(1.0)
        assert nxt == pytest.approx(p.reactor.monotonic() + 3)


class TestUnitResolution:
    def test_missing_unit_is_tolerated(self):
        # Config can name a unit that does not exist yet (or at all); the
        # sensor must keep ticking rather than throw every report_time.
        p = _Printer(None)
        s = _make(p, bambu_unit="NoSuchUnit")
        s._sample(1.0)
        assert p.shutdowns == []
        assert s.get_status(1.0)["humidity"] == 0.0


class TestObjectRegistrationForTheUIs:
    """Both UIs are fed by registering the sensor under extra object names.
    Mainsail reads an "aht10 <name>" object directly; Fluidd resolves the
    section's sensor_type to an object of that name. Registering both means a
    section can say aht2x/aht3x/sht3x and still show humidity."""

    def test_registers_aht10_alias_by_default(self):
        p = _Printer(_unit(humidity=1))
        _make(p, bambu_unit="BambuAMS_1")
        assert "aht10 BambuAMS_1" in p.objects

    def test_registers_the_configured_sensor_type_alias_too(self):
        p = _Printer(_unit(humidity=1))
        _make(p, bambu_unit="BambuAMS_1", sensor_type="sht3x")
        assert "aht10 BambuAMS_1" in p.objects
        assert "sht3x BambuAMS_1" in p.objects

    def test_alias_already_taken_by_a_real_sensor_is_not_fatal(self):
        p = _Printer(_unit(humidity=1))
        p.objects = {"sht3x BambuAMS_1": object()}     # a real driver got there
        _make(p, bambu_unit="BambuAMS_1", sensor_type="sht3x")
        assert "aht10 BambuAMS_1" in p.objects         # ours still registered

    def test_aht_simulation_can_be_turned_off(self):
        p = _Printer(_unit(humidity=1))
        _make(p, bambu_unit="BambuAMS_1",
              simulate_supported_sensor_mainsail=False)
        assert "temperature_bambu BambuAMS_1" in p.objects
        assert "aht10 BambuAMS_1" not in p.objects


class TestReadyStartsSampling:
    def test_handle_ready_arms_the_timer(self):
        p = _Printer(_unit(humidity=1))
        s = _make(p, bambu_unit="BambuAMS_1")
        armed = []
        p.reactor.update_timer = lambda t, when: armed.append((t, when))
        s._handle_ready()
        assert armed and armed[0][0] is s.sample_timer


class TestLoadConfigRegistersBothFactories:
    """aht4x is an INVENTED sensor_type name on purpose: Klipper loads all of
    its own sensor modules while resolving a temperature_sensor section, so a
    real type would have this factory overwritten by Klipper's, and the genuine
    driver would then reject bambu_unit and halt the printer."""

    def test_registers_temperature_bambu_and_aht4x(self):
        import extras.temperature_bambu as tb
        registered = []
        cfg = types.SimpleNamespace(get_printer=lambda: types.SimpleNamespace(
            lookup_object=lambda n: types.SimpleNamespace(
                add_sensor_factory=lambda name, cls: registered.append(name))))
        tb.load_config(cfg)
        assert registered == ["temperature_bambu", "aht4x"]
