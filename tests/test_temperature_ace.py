"""
Unit tests for the temperature_ace sensor channel selection
(extras/temperature_ace.py).

  channel='default' — historical behaviour: single box/env temp + humidity from
                      the unit's get_status cache (_cached_hw_status).
  channel=box1/ptc1/env/... — the richer GET_TEMP channel from _cached_temp_info,
                      surfacing env_humidity alongside.

Style: typed fakes, full state verification. The sensor is built with __new__
so no Klipper config is needed; the callback is left unset so sampling exercises
only the channel-read logic.
"""

from __future__ import annotations

from extras.temperature_ace import TemperatureACE


class _FakeUnit:
    def __init__(self, hw_status=None, temp_info=None):
        self._cached_hw_status = hw_status or {}
        self._cached_temp_info = temp_info or {}


def _make_sensor(channel, unit):
    s = TemperatureACE.__new__(TemperatureACE)
    s.channel = channel
    s._ace_unit = unit
    s.temp = 0.0
    s.humidity = 0.0
    s._has_humidity = False
    s.min_temp = 0.0
    s.max_temp = 1000.0
    s.measured_min = float("inf")
    s.measured_max = 0.0
    s._callback = None          # skip the heaters-callback block
    s._sample_error_logged = False
    return s


def test_default_channel_reads_status_temp_and_humidity():
    unit = _FakeUnit(hw_status={"temp": 42.0, "humidity": 31.0})
    s = _make_sensor("default", unit)

    s._sample_ace_temperature(0.0)

    assert s.temp == 42.0
    assert s._has_humidity is True
    assert s.humidity == 31.0


def test_default_channel_v1_no_humidity_key():
    unit = _FakeUnit(hw_status={"temp": 40.0})  # V1 ACE omits humidity
    s = _make_sensor("default", unit)

    s._sample_ace_temperature(0.0)

    assert s.temp == 40.0
    assert s._has_humidity is False


def test_ptc1_channel_reads_from_temp_cache():
    unit = _FakeUnit(
        hw_status={"temp": 27.0, "humidity": 30.0},   # must NOT be used
        temp_info={"ptc1_temp": 55.0, "ptc2_temp": 60.0,
                   "box1_temp": 24.0, "env_temp": 27.0, "env_humidity": 30.0})
    s = _make_sensor("ptc1", unit)

    s._sample_ace_temperature(0.0)

    assert s.temp == 55.0                 # ptc1, not the status temp 27
    assert s._has_humidity is True
    assert s.humidity == 30.0


def test_box_and_env_channels_select_the_right_field():
    unit = _FakeUnit(temp_info={
        "box1_temp": 24.5, "box2_temp": 25.5,
        "ptc1_temp": 55.0, "ptc2_temp": 60.0, "env_temp": 27.0})
    for channel, expected in (("box1", 24.5), ("box2", 25.5),
                              ("ptc2", 60.0), ("env", 27.0)):
        s = _make_sensor(channel, unit)
        s._sample_ace_temperature(0.0)
        assert s.temp == expected, f"channel {channel}"


def test_missing_channel_field_reads_zero():
    unit = _FakeUnit(temp_info={"box1_temp": 24.0})  # no ptc1 present
    s = _make_sensor("ptc1", unit)

    s._sample_ace_temperature(0.0)

    assert s.temp == 0.0


def test_non_default_channel_tracks_measured_min_max():
    unit = _FakeUnit(temp_info={"ptc1_temp": 55.0})
    s = _make_sensor("ptc1", unit)

    s._sample_ace_temperature(0.0)

    assert s.measured_max == 55.0
    assert s.measured_min == 55.0
