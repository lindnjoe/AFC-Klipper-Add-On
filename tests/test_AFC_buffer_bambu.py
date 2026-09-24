"""
Unit tests for the Bambu AMS buffer (`type: bambu`) in extras/AFC_BambuAMS_buffer.py

The Bambu AMS has no toolhead switch of ours in it. This driver stands in for
one by requiring TWO signals the unit does publish: its spring buffer reading
and its own filament odometer.

Every trace in this file is REAL, captured from printer 1 on 2026-09-22 by
subscribing to the live status stream over Moonraker at the rate the bridge
publishes it (250 ms, measured dead on), and lined up against the moment
`_feed_until_sensor` logged the real toolhead switch firing. Three loads:

    load  unit     friction peak  fps>=0.9 lead   AND lead   switch at
    1     HT       0.33           1.71 s          0.20 s     126421.61
    2     AMS 2    0.26           2.02 s          0.77 s     126500.86
    3     HT       0.39           2.03 s          0.52 s     128467.51

The point of testing against the traces rather than invented numbers is that
the two failure modes these tests pin are both things that ACTUALLY HAPPENED
on the wire, not hypotheticals: bowden friction walking the buffer up to 0.39
on the way in, and the odometer reading a dead stop mid-feed with a metre
still to run.
"""

from __future__ import annotations

import pytest

from extras.AFC_BambuAMS_buffer import AFCBambuBuffer
from extras.AFC_buffer import AFCFPSBuffer, load_config_prefix

from tests.conftest import MockConfig, MockPrinter, MockAFC


# ── Fakes ─────────────────────────────────────────────────────────────────────

class _AdcMcu:
    def estimated_print_time(self, eventtime):
        return 42.0


class _Adc:
    """Kalico-shaped ADC; the dispatch itself is covered in the FPS tests."""

    def __init__(self):
        self.calls = []

    def get_mcu(self):
        return _AdcMcu()

    def setup_minmax(self, sample_time, sample_count, minval=0.0, maxval=1.0,
                     range_check_count=0):
        self.calls.append(("setup_minmax", sample_time, sample_count))

    def setup_adc_callback(self, report_time, callback):
        self.calls.append(("setup_adc_callback", report_time))


class _FakePins:
    def setup_pin(self, pin_type, pin):
        assert pin_type == "adc"
        return _Adc()


class _FakeLane:
    def __init__(self, name, buffer_name):
        self.name = name
        self.buffer_name = buffer_name


class _FakeBambuUnit:
    """Stands in for an AFC_BambuAMS unit: a name, lanes, an odometer."""

    def __init__(self, name, buffer_name, lane_name="lane12"):
        self.name = name
        self.lanes = {lane_name: _FakeLane(lane_name, buffer_name)}
        self.odom_m = None

    def _odom_now_mm(self):
        return None if self.odom_m is None else self.odom_m * 1000.0


BUF_NAME = "Bambu_AMS_Buffer"


def _make_bambu_buffer(values=None, units=("Bambu_AMS_1",)):
    afc = MockAFC()
    afc.led_buffer_advancing = "0,0,1,0"
    afc.led_buffer_trailing = "0,1,0,0"
    afc.led_buffer_neutral = "0,0,0,0.25"
    afc.led_buffer_disabled = "0,0,0,0.25"

    printer = MockPrinter(afc=afc)
    printer._objects["pins"] = _FakePins()

    unit_objs = []
    for uname in units:
        unit = _FakeBambuUnit(uname, BUF_NAME)
        printer._objects[f"AFC_BambuAMS {uname}"] = unit
        unit_objs.append(unit)

    cfg_values = {"type": "bambu", "adc_pin": f"bambu_buffer:fps"}
    cfg_values.update(values or {})
    config = MockConfig(name=f"AFC_buffer {BUF_NAME}", printer=printer,
                        values=cfg_values)
    return AFCBambuBuffer(config), unit_objs


def _spend_bind_grace(buf):
    """
    Put a unit-less buffer past its bind grace.

    A buffer with no Bambu unit on it only degrades to pressure alone once
    it has waited out ``odom_bind_grace_seconds``, because early in a boot
    "no unit bound" means the pool has not claimed one yet. Tests about the
    degraded behaviour itself start from the far side of that wait.
    """
    # Far enough in the past that any sample time the test then feeds is
    # already on the far side of the wait.
    buf._first_sample_t = -(buf.odom_bind_grace_seconds + 1.0)
    return buf


def _feed(buf, unit, samples):
    """
    Replay (fps, odom_m) samples through the driver as the ADC would.

    :param buf: the buffer under test
    :param unit: the unit whose odometer the samples belong to
    :param samples: iterable of (fps, odom_m); odom_m None leaves it alone
    :return list: advance_state after each sample
    """
    out = []
    # Continue this buffer's own clock. The gate measures in seconds now, so
    # a second _feed() starting back at zero would run time backwards.
    t = buf._now_t
    for fps, odom in samples:
        if odom is not None:
            unit.odom_m = odom
        t += 0.25
        buf._adc_callback(t, fps)
        out.append(buf.advance_state)
    return out


# ── Registration ──────────────────────────────────────────────────────────────

def test_load_config_prefix_builds_a_bambu_buffer():
    afc = MockAFC()
    afc.led_buffer_advancing = afc.led_buffer_trailing = "0,0,1,0"
    afc.led_buffer_neutral = afc.led_buffer_disabled = "0,0,0,0.25"
    printer = MockPrinter(afc=afc)
    printer._objects["pins"] = _FakePins()
    config = MockConfig(name=f"AFC_buffer {BUF_NAME}", printer=printer,
                        values={"type": "bambu", "adc_pin": "bambu_buffer:fps"})

    buf = load_config_prefix(config)

    assert isinstance(buf, AFCBambuBuffer)
    # It is still an FPS buffer, so the gauge, QUERY_BUFFER and ramming all
    # keep working; only the load answer is narrowed.
    assert isinstance(buf, AFCFPSBuffer)


def test_unknown_type_names_bambu_in_the_error():
    afc = MockAFC()
    printer = MockPrinter(afc=afc)
    config = MockConfig(name=f"AFC_buffer {BUF_NAME}", printer=printer,
                        values={"type": "nonsense"})

    with pytest.raises(Exception) as exc:
        load_config_prefix(config)

    assert "bambu" in str(exc.value)


def test_it_finds_its_units_by_their_lanes_buffer_name():
    buf, units = _make_bambu_buffer(units=("Bambu_AMS_1", "Bambu_AMS_HT_1"))

    assert buf._bound_units() == units


def test_a_scan_that_finds_nothing_is_not_remembered_as_never():
    # LIVE FAULT, printer 1, 2026-09-22. The pool claims its units a moment
    # after klippy:ready, so the first ADC callback scanned an empty roster.
    # That empty answer was cached, and the gate then ran the whole session
    # on pressure alone -- with two claimed units reporting odometers the
    # whole time. AFC.log carried only the one degrade warning.
    buf, _units = _make_bambu_buffer(units=())
    assert buf._bound_units() == []          # nothing bound yet
    late = _FakeBambuUnit("Bambu_AMS_1", BUF_NAME)
    buf.printer._objects["AFC_BambuAMS Bambu_AMS_1"] = late
    assert buf._bound_units() == [late]      # the next scan finds it


def test_a_late_unit_clears_the_degrade_warning_latch():
    # So a genuine later loss of every odometer says so again, instead of
    # being swallowed by the warning the empty startup scan already spent.
    buf, _units = _make_bambu_buffer(units=())
    _spend_bind_grace(buf)
    assert buf._odom_confirms() is True      # degrades, and warns once
    assert buf._odom_warned is True
    buf.printer._objects["AFC_BambuAMS Bambu_AMS_1"] = _FakeBambuUnit(
        "Bambu_AMS_1", BUF_NAME)
    buf._bound_units()
    assert buf._odom_warned is False


def test_movement_stops_counting_once_the_unit_has_long_since_stopped():
    # LIVE, printer 1, 2026-09-22: idle at the gate, odom_moved had latched
    # true and the unit had been still for half a minute -- so "moved, then
    # stopped" was really
    # just "not moving", which anything that compresses an idle unit's buffer
    # would satisfy.
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, [(0.46, 0.100), (0.46, 0.140), (0.46, 0.180)])
    assert buf._odom_moved is True
    # Now sit still well past the time a braking tail could account for.
    lapse = int(buf.odom_move_ttl_seconds / 0.25) + 2
    _feed(buf, unit, [(0.46, 0.180)] * lapse)
    assert buf._odom_moved is False
    assert buf._odom_confirms() is False


def test_the_lapse_is_far_longer_than_a_real_braking_tail():
    # The real tail between the filament stopping and the switch closing is
    # one or two samples on all three logged loads; the lapse must never
    # reach into that.
    buf, (unit,) = _make_bambu_buffer()
    assert buf.odom_move_ttl_seconds >= 2.0
    _feed(buf, unit, LOAD3)
    assert buf._gate_latched is True        # still detected, unchanged


def test_a_unit_on_a_different_buffer_is_not_bound():
    buf, _ = _make_bambu_buffer(units=("Bambu_AMS_1",))
    other = _FakeBambuUnit("Bambu_AMS_9", "Some_Other_Buffer", "lane90")
    buf.printer._objects["AFC_BambuAMS Bambu_AMS_9"] = other
    buf._odom_units = None      # force re-resolution

    assert other not in buf._bound_units()


# ── The two failure modes, from the real traces ───────────────────────────────

# Load 3 (HT, 2026-09-22, eventtime 128457.5 -> 128467.2). The switch fired at
# 128467.51, between the last two samples here.
LOAD3 = [
    (0.02, 0.002), (0.02, 0.044), (0.02, 0.111), (0.02, 0.151),
    (0.02, 0.208), (0.02, 0.283), (0.02, 0.357), (0.02, 0.432),
    (0.02, 0.509), (0.02, 0.578), (0.02, 0.658), (0.02, 0.719),
    (0.02, 0.811), (0.02, 0.883), (0.02, 0.961), (0.02, 1.037),
    (0.02, 1.112), (0.02, 1.186), (0.02, 1.249), (0.02, 1.329),
    (0.02, 1.411), (0.02, 1.487), (0.02, 1.561), (0.03, 1.641),
    # friction plateau: the tube is filling and the buffer is loading up
    (0.35, 1.717), (0.35, 1.787), (0.35, 1.861), (0.35, 1.936),
    (0.35, 2.011), (0.36, 2.089), (0.37, 2.161), (0.38, 2.237),
    (0.39, 2.309),
    # at the gears
    (0.96, 2.364), (0.96, 2.428), (0.96, 2.483), (0.96, 2.533),
    (0.95, 2.538), (0.94, 2.542), (0.94, 2.543), (0.94, 2.545),
    (0.95, 2.545),
]
LOAD3_FIRST_HIGH = 33          # index of the first 0.96 sample
LOAD3_SWITCH_AFTER = 47        # the switch fired after index 47 (128467.23)


def test_friction_plateau_alone_never_reports_filament():
    """
    The 0.35-0.39 plateau is bowden drag with a metre of tube still to fill.
    A buffer-only sensor reads that as arrival; this one must not.
    """
    buf, (unit,) = _make_bambu_buffer()

    states = _feed(buf, unit, LOAD3[:LOAD3_FIRST_HIGH])

    assert not any(states)


def test_the_load_is_detected_before_the_real_switch_fired():
    """
    Both conditions first held 0.52 s before the switch on this load. Pin
    that it fires, and that it fires inside the window the switch closed.
    """
    buf, (unit,) = _make_bambu_buffer()

    states = _feed(buf, unit, LOAD3)

    assert states[-1] is True
    first = states.index(True)
    # After the buffer went high...
    assert first >= LOAD3_FIRST_HIGH
    # ...and no later than the sample the real switch beat us to.
    assert first <= LOAD3_SWITCH_AFTER


def test_pressure_alone_would_have_fired_two_seconds_early():
    """
    The guard earns its keep: a plain FPS buffer on this same trace calls it
    at the first 0.96, eight samples (about 2 s) before this one does.
    """
    buf, (unit,) = _make_bambu_buffer()
    # A buffer with no Bambu unit on it at all, which is what degrades to
    # pressure alone. (Blanking _odom_units no longer does it: an empty scan
    # is "not yet" and gets retried, which is the point of that fix.)
    plain, _ = _make_bambu_buffer(units=())
    plain.odom_required = False
    _spend_bind_grace(plain)

    gated = _feed(buf, unit, LOAD3)
    ungated = _feed(plain, _FakeBambuUnit("x", BUF_NAME), LOAD3)

    assert ungated.index(True) == LOAD3_FIRST_HIGH
    assert gated.index(True) > ungated.index(True)


# Load 2 (boxed AMS 2, 2026-09-22 eventtime 126491.3): the odometer repeated
# 0.558 m for a whole sample with a metre still to run.
def test_a_bare_value_sample_still_carries_a_moving_clock():
    """
    LIVE, printer 1, 2026-09-22: the virtual chip delivers bare values with
    no time attached, so every sample was being stamped 0.0. No interval ever
    elapsed, the stillness clock never reached its threshold, and the gate
    could not fire at all. It showed as `odom_still_s 0.0` alongside
    `odom_moved true` -- movement seen, yet no time since.

    The reactor is the fallback clock, which is what the base class does with
    the same argument shape.
    """
    buf, (unit,) = _make_bambu_buffer()
    unit.odom_m = 0.10

    buf.reactor._monotonic = 100.0
    buf._adc_callback(0.46)              # a bare value, no time
    assert buf._now_t == 100.0
    assert buf._first_sample_t == 100.0

    buf.reactor._monotonic = 100.6
    buf._adc_callback(0.46)
    assert buf._now_t == 100.6


def test_a_bare_value_feed_can_still_detect_a_load():
    # The end-to-end version of the above: on the bare-value path the gate
    # must still see movement, then stillness, then latch.
    buf, (unit,) = _make_bambu_buffer()
    t = 100.0
    for fps, odom in LOAD3:
        unit.odom_m = odom
        t += 0.25
        buf.reactor._monotonic = t
        buf._adc_callback(fps)           # bare value throughout

    assert buf._gate_latched is True


def test_oversampling_a_moving_feed_is_not_mistaken_for_stillness():
    """
    THE 10 Hz TRAP, measured on printer 1 on 2026-09-22.

    The ADC callback runs at 9.98 Hz; the bridge only refreshes the odometer
    every 250 ms. So mid-feed, at full speed, two or three consecutive
    callbacks read the SAME odometer value purely because no new frame has
    landed. A stillness test that counted callbacks scored that as "stopped"
    and, with the buffer already compressed by bowden friction, would have
    called the load -- collapsing the gate back to pressure alone, which is
    the one thing it exists to prevent.

    Here the filament is moving hard the whole time (74 mm per bridge frame)
    and the buffer is at the friction plateau. Nothing may fire.
    """
    buf, (unit,) = _make_bambu_buffer()

    t = 0.0
    odom_m = 0.10
    frame_due = 0.0
    fired = []
    for _ in range(120):                  # 12 s at 10 Hz
        t += 0.1
        if t >= frame_due:                # a new bridge frame every 250 ms
            odom_m += 0.074
            frame_due += 0.25
        unit.odom_m = odom_m
        buf._adc_callback(t, 0.39)        # the measured friction plateau
        fired.append(buf.buffer_triggered)

    assert not any(fired)
    assert buf._gate_latched is False
    # It saw plenty of movement -- it just never saw it stop.
    assert buf._odom_moved is True


def test_a_feed_at_speed_never_looks_still_for_the_full_window():
    # The same trap stated as the invariant: while frames keep arriving with
    # movement in them, the stillness clock can never reach its threshold.
    buf, (unit,) = _make_bambu_buffer()

    t = 0.0
    odom_m = 0.10
    frame_due = 0.0
    worst = 0.0
    for _ in range(200):
        t += 0.1
        if t >= frame_due:
            odom_m += 0.074
            frame_due += 0.25
        unit.odom_m = odom_m
        buf._adc_callback(t, 0.39)
        worst = max(worst, buf._odom_still_for())

    assert worst < buf.odom_still_seconds


def test_a_dead_stop_mid_feed_does_not_report_filament():
    """
    The odometer alone would call this arrival. The buffer was at 0.02, which
    is what rejects it -- this is the case that makes BOTH signals necessary
    rather than belt-and-braces.
    """
    buf, (unit,) = _make_bambu_buffer()

    states = _feed(buf, unit, [
        (0.02, 0.412), (0.02, 0.484), (0.02, 0.558),
        (0.02, 0.558),      # <- dead stop, still a metre to go
        (0.02, 0.558),
        (0.02, 0.701), (0.02, 0.774),
    ])

    assert not any(states)


def test_a_compressed_buffer_that_never_moved_does_not_report_filament():
    """
    MOVED FIRST, then still. Without that, a gate opening before the unit
    starts calls an unstarted feed "arrived" -- the same discipline
    _wait_move's stop_when_still follows.
    """
    buf, (unit,) = _make_bambu_buffer()

    states = _feed(buf, unit, [(0.96, 1.500)] * 6)

    assert not any(states)


# ── The epsilon ───────────────────────────────────────────────────────────────

# The braking tail of load 3, in mm: +5, +4, +1, +2, 0.
BRAKE_TAIL = [(0.96, 2.533), (0.95, 2.538), (0.94, 2.542),
              (0.94, 2.543), (0.94, 2.545), (0.95, 2.545)]


@pytest.mark.parametrize("eps", [1.0, 2.0])
def test_a_small_epsilon_flickers_on_the_braking_tail(eps):
    """
    Why the default is 3 mm and not 1 or 2: at those the +1/+2 mm samples in
    the tail read as movement and reset the run. Guards the constant against
    a well-meaning tightening.
    """
    buf, (unit,) = _make_bambu_buffer(values={"odom_eps_mm": eps,
                                              "odom_still_seconds": 0.5})
    _feed(buf, unit, LOAD3[:LOAD3_FIRST_HIGH])
    buf._odom_moved = True

    stills = []
    t = buf._now_t
    for fps, odom in BRAKE_TAIL:
        unit.odom_m = odom
        t += 0.25
        buf._now_t = t
        buf._update_odom()
        stills.append(buf._odom_still_for())

    assert 0.0 in stills[1:]        # the clock got reset somewhere in the tail


def test_the_default_epsilon_resolves_the_braking_tail():
    buf, (unit,) = _make_bambu_buffer()
    assert buf.odom_eps_mm == 3.0
    assert buf.odom_still_seconds == 0.5

    _feed(buf, unit, LOAD3[:LOAD3_FIRST_HIGH])
    buf._odom_moved = True
    t = buf._now_t
    for fps, odom in BRAKE_TAIL:
        unit.odom_m = odom
        t += 0.25
        buf._now_t = t
        buf._update_odom()

    assert buf._odom_still_for() >= buf.odom_still_seconds


# ── Staying loaded ────────────────────────────────────────────────────────────

def test_it_stays_loaded_through_a_print_with_a_frozen_odometer():
    """
    Measured: the odometer does NOT advance while a print consumes filament
    (unchanged across 60 s of printing), because it only moves during a
    commanded feed, while the buffer sawtooths 0.14 to 0.96 as the AMS
    refills it. Re-testing the gate every tick would drop the sensor
    mid-print, so arrival latches.
    """
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, LOAD3)
    assert buf.advance_state is True

    # A real printing sawtooth, odometer frozen at the load's end position.
    # Run it long enough that the movement evidence lapses partway through
    # (odom_move_ttl_seconds), because that is the whole point of the latch:
    # once the print starts, nothing about the odometer can vouch for the
    # filament any more, and the sensor must not drop when it stops doing so.
    cycle = [(0.95, 2.545), (0.59, 2.545), (0.40, 2.545), (0.13, 2.545),
             (0.39, 2.545), (0.47, 2.545), (0.63, 2.545), (0.52, 2.545)]
    printing = _feed(buf, unit, cycle * 6)

    assert all(printing)
    assert buf._odom_moved is False        # lapsed, as it should have
    assert buf._odom_confirms() is False   # and the odometer now vouches for nothing
    assert buf._gate_latched is True       # the latch is what is carrying it
    # buffer_triggered is deliberately NOT asserted here: it is the homing
    # endstop for the ramming move, and it reads live pressure, so mid-
    # sawtooth it is legitimately False. What must not drop mid-print is
    # advance_state, which is what an extruder on tool_start: buffer reads.


def test_the_end_of_a_load_does_not_destroy_the_latch():
    """
    LIVE FAULT, printer 1, 2026-09-22. enable_buffer() does NOT run at the
    start of a load, it runs at the END, right after arrival -- so resetting
    the gate there wiped the latch one second after it was set. lane15 sat
    loaded with the buffer compressed at 0.94 and the sensor reporting NOT
    loaded, which on the following unload is the dangerous way round: AFC
    watches this sensor to know the path has cleared.
    """
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, LOAD3)
    assert buf._gate_latched is True

    buf.enable_buffer(_FakeLane("lane15", BUF_NAME))

    assert buf._gate_latched is True
    assert buf.advance_state is True


def _lane(name, loaded):
    ln = _FakeLane(name, BUF_NAME)
    ln.tool_loaded = loaded
    return ln


def test_afcs_own_record_is_believed_at_startup():
    # How the other units come back from a restart: a lane keeps tool_loaded
    # across a reboot, and AFC_BambuAMS._startup_restore_loaded re-asserts
    # the follower off exactly this flag. The gate reports an arrival, and
    # after a restart there was no arrival to witness.
    buf, (unit,) = _make_bambu_buffer()
    buf.lanes = {"lane28": _lane("lane28", True)}

    buf._adc_callback(0.25, 0.94)        # buffer agrees: compressed

    assert buf._gate_latched is True
    assert buf.advance_state is True
    assert [m for lvl, m in buf.logger.messages
            if lvl == "info" and "from AFC's own record" in m]


def test_a_record_the_buffer_contradicts_is_refused_out_loud():
    # Max tension is what an EMPTY path reads like. A record claiming loaded
    # against that is a contradiction, and adopting it quietly would be the
    # dangerous direction on the unload that follows.
    buf, (unit,) = _make_bambu_buffer()
    buf.lanes = {"lane28": _lane("lane28", True)}

    buf._adc_callback(0.25, 0.02)        # bottomed out

    assert buf._gate_latched is False
    assert [m for lvl, m in buf.logger.messages
            if lvl == "warning" and "max tension" in m]


def test_an_empty_record_adopts_nothing():
    buf, (unit,) = _make_bambu_buffer()
    buf.lanes = {"lane28": _lane("lane28", False)}

    buf._adc_callback(0.25, 0.94)

    assert buf._gate_latched is False
    assert not [m for lvl, m in buf.logger.messages
                if "from AFC's own record" in m]


def test_the_runout_release_is_not_undone_by_a_stale_record():
    """
    The re-adopt loop this guards against: the release fires because the
    buffer has PROVED the path empty, so re-arming the record check inside
    _reset_gate() would latch straight back on from a record that has not
    caught up yet. Only an unload re-arms it.
    """
    buf, (unit,) = _make_bambu_buffer()
    buf.lanes = {"lane28": _lane("lane28", True)}
    _feed(buf, unit, LOAD3)
    assert buf._gate_latched is True

    # The path empties and stays empty, well past unload_confirm_seconds.
    drain = int(buf.unload_confirm_seconds / 0.25) + 4
    _feed(buf, unit, [(0.02, 2.545)] * drain)

    assert buf._gate_latched is False    # released, and it stays released
    _feed(buf, unit, [(0.02, 2.545)] * 4)
    assert buf._gate_latched is False


def test_the_record_is_consulted_again_until_afc_restores_it():
    """
    The record is NOT there at the first sample. The ADC callback starts at
    connect; AFC restores tool_loaded from saved vars once the units have
    claimed their lanes -- measured 8 s later on printer 1. Answering once
    and giving up therefore always asked before the answer existed, and a
    lane that was loaded across the restart came back reading empty.
    """
    buf, (unit,) = _make_bambu_buffer()
    mid = (buf.low_point + buf.set_point) / 2.0   # resting: not compressed
    lane = _lane("lane28", False)
    buf.lanes = {"lane28": lane}

    t = 0.0
    for _ in range(12):                  # 3 s of AFC still restoring
        t += 0.25
        buf._adc_callback(t, mid)
    assert buf._gate_latched is False
    assert buf._record_checked is False  # "not restored", never "nothing"

    lane.tool_loaded = True              # saved vars land
    t += 0.25
    buf._adc_callback(t, mid)

    assert buf._gate_latched is True
    assert [m for lvl, m in buf.logger.messages
            if lvl == "info" and "from AFC's own record" in m]


def test_the_record_window_shuts():
    # Adoption is a RESTART recovery and nothing else. Past the window a
    # lane that is loaded got there through a load this gate watched.
    buf, (unit,) = _make_bambu_buffer({"record_check_seconds": 1.0})
    mid = (buf.low_point + buf.set_point) / 2.0
    lane = _lane("lane28", False)
    buf.lanes = {"lane28": lane}

    t = 0.0
    for _ in range(8):                   # 2 s, twice the window
        t += 0.25
        buf._adc_callback(t, mid)
    assert buf._record_checked is True

    lane.tool_loaded = True
    t += 0.25
    buf._adc_callback(t, mid)

    assert buf._gate_latched is False


def test_an_unload_does_not_re_open_the_record():
    """
    AFC calls disable_buffer() near the TOP of TOOL_UNLOAD and only clears
    tool_loaded at the very end, so for the whole of an unload the record
    still reads loaded. Re-arming the check here would adopt the lane on
    its way out and latch the gate straight back on.
    """
    buf, (unit,) = _make_bambu_buffer()
    buf.lanes = {"lane28": _lane("lane28", True)}
    buf._adc_callback(0.25, 0.94)
    assert buf._gate_latched is True

    buf.disable_buffer()
    assert buf._gate_latched is False
    assert buf._record_checked is True

    buf._adc_callback(0.5, 0.94)         # record has not caught up yet

    assert buf._gate_latched is False


def test_filament_already_at_the_gears_is_adopted_after_a_restart():
    # The odometer route reports an ARRIVAL; a filament sensor has to report
    # a STATE. After a restart nobody witnessed the arrival, so without this
    # a loaded lane comes back reading empty.
    buf, (unit,) = _make_bambu_buffer()
    unit.odom_m = 2.624                  # parked where the load left it

    t = 0.0
    for _ in range(int(buf.resting_load_seconds / 0.25) + 2):
        t += 0.25
        buf._adc_callback(t, 0.94)       # compressed, nothing moving

    assert buf._gate_latched is True
    assert buf.advance_state is True
    assert [m for lvl, m in buf.logger.messages
            if lvl == "info" and "already at the gears" in m]


def test_an_unload_is_never_adopted_as_resting():
    # Printer 1, lane15, 2026-09-24: the cut pushes the tip back into the
    # hotend with the unit parked, so the buffer sat compressed and the
    # odometer still -- and the resting path logged "load detected ...
    # filament already at the gears" in the middle of the unload.
    buf, (unit,) = _make_bambu_buffer()
    lane = _lane("lane15", True)
    lane.status = "Tool Unloading"
    buf.lanes = {"lane15": lane}
    buf._record_checked = True           # the record route is not under test
    buf.disable_buffer()                 # as TOOL_UNLOAD does, near its top
    unit.odom_m = 2.623

    t = 0.0
    for _ in range(int(buf.resting_load_seconds / 0.25) * 3):
        t += 0.25
        buf._adc_callback(t, 0.94)       # compressed, nothing moving

    assert buf._gate_latched is False
    assert not [m for lvl, m in buf.logger.messages
                if "load detected" in m]


def test_a_moving_feed_can_never_be_adopted_as_resting():
    # The safety of the above rests entirely on this: a feed refreshes the
    # odometer every 250 ms, so the stillness window cannot accumulate,
    # however long the buffer stays compressed.
    buf, (unit,) = _make_bambu_buffer()

    t = 0.0
    odom_m = 0.10
    for _ in range(int(buf.resting_load_seconds / 0.25) * 4):
        t += 0.25
        odom_m += 0.074                  # still feeding hard
        unit.odom_m = odom_m
        buf._adc_callback(t, 0.94)       # and compressed the whole way
        if buf._gate_latched:
            break

    assert buf._gate_latched is False
    assert buf._resting_load() is False


def test_the_friction_plateau_never_reaches_the_resting_path_either():
    # Bowden friction peaked at 0.39 on the real loads, which is below the
    # advance threshold, so the compression clock never even starts.
    buf, (unit,) = _make_bambu_buffer()

    t = 0.0
    for _ in range(int(buf.resting_load_seconds / 0.25) * 3):
        t += 0.25
        buf._adc_callback(t, 0.39)

    assert buf._compressed_since is None
    assert buf._gate_latched is False


def test_unloading_resets_the_gate():
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, LOAD3)
    assert buf._gate_latched is True

    buf.disable_buffer()

    assert buf._gate_latched is False
    assert buf._odom_moved is False
    assert buf._odom_prev == {}


# ── Degrading, and refusing to ────────────────────────────────────────────────

def test_without_any_odometer_it_falls_back_to_pressure_and_warns():
    buf, _ = _make_bambu_buffer(units=())
    _spend_bind_grace(buf)

    states = _feed(buf, _FakeBambuUnit("unbound", BUF_NAME), LOAD3)

    assert states[LOAD3_FIRST_HIGH] is True
    assert buf._odom_warned is True


def test_a_bound_unit_that_has_not_reported_yet_is_not_a_missing_odometer():
    # LIVE, printer 1, 2026-09-22, seconds after a Klipper restart:
    # odom_confirms read True with odom_moved False, because the units were
    # bound but no bridge frame had landed. "Nothing read back yet" was
    # taking the same branch as "this buffer has no odometer", which hands
    # the startup window to pressure alone -- the exact false positive the
    # gate exists to stop.
    buf, (unit,) = _make_bambu_buffer()
    unit.odom_m = None                       # bound, silent

    assert buf._bound_units() == [unit]
    assert buf._odom_confirms() is False     # "not yet", not "sure"
    assert not [m for lvl, m in buf.logger.messages if lvl == "warning"]


def test_no_degrade_warning_while_the_pool_is_still_claiming_units():
    # Joe saw this warning on 2026-09-22 and reasonably read it as the gate
    # being broken. It was not: it fires in the seconds between Klipper
    # being ready and BridgeBox's pool binding a unit to a lane, which looks
    # identical to a buffer with no Bambu unit on it. Every restart produced
    # one -- five in AFC.log across an afternoon of deploys.
    buf, _units = _make_bambu_buffer(units=())

    t = 0.0
    while t < buf.odom_bind_grace_seconds - 0.5:
        t += 0.25
        buf._adc_callback(t, 0.46)

    assert not [m for lvl, m in buf.logger.messages if lvl == "warning"]
    # And it REFUSES rather than degrading while it waits, so a load landing
    # in that window is not judged on pressure alone either.
    assert buf._odom_confirms() is False


def test_a_unit_claimed_during_the_grace_is_never_warned_about():
    buf, _units = _make_bambu_buffer(units=())
    t = 0.0
    for _ in range(3):
        t += 0.25
        buf._adc_callback(t, 0.46)
    buf.printer._objects["AFC_BambuAMS Bambu_AMS_1"] = _FakeBambuUnit(
        "Bambu_AMS_1", BUF_NAME)

    while t < buf.odom_bind_grace_seconds * 2:
        t += 0.25
        buf._adc_callback(t, 0.46)

    assert not [m for lvl, m in buf.logger.messages if lvl == "warning"]


def test_a_buffer_that_really_has_no_unit_still_says_so():
    # The warning must not be lost -- only delayed until it is true.
    buf, _units = _make_bambu_buffer(units=())

    t = 0.0
    while t < buf.odom_bind_grace_seconds + 0.5:
        t += 0.25
        buf._adc_callback(t, 0.46)

    assert buf._odom_confirms() is True          # degrades, as documented
    assert [m for lvl, m in buf.logger.messages
            if lvl == "warning" and "no Bambu unit" in m]


def test_odom_required_refuses_rather_than_degrading():
    buf, _ = _make_bambu_buffer(values={"odom_required": True}, units=())

    states = _feed(buf, _FakeBambuUnit("unbound", BUF_NAME), LOAD3)

    assert not any(states)


# ── The endstop sees the same gate ────────────────────────────────────────────

def test_buffer_triggered_carries_the_odometer_gate():
    """
    The software endstop homes on buffer_triggered, so pressure alone must
    not satisfy it either.
    """
    buf, (unit,) = _make_bambu_buffer()

    _feed(buf, unit, LOAD3[:LOAD3_FIRST_HIGH + 1])
    assert buf.smoothed_fps >= buf._homing_high_point
    assert buf.buffer_triggered is False

    _feed(buf, unit, LOAD3[LOAD3_FIRST_HIGH + 1:])
    assert buf.buffer_triggered is True


# ── Status ────────────────────────────────────────────────────────────────────

def test_the_detection_instant_is_logged_once():
    # So a load can be checked afterwards against the toolhead switch's own
    # line at the same eventtime, without anyone watching the status fields
    # live. The 30 minutes spent recording an idle printer is why this exists.
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, LOAD3)

    lines = [m for lvl, m in buf.logger.messages
             if lvl == "info" and "load detected" in m]
    assert len(lines) == 1
    msg = lines[0]
    assert "fps" in msg and "odometer still" in msg
    assert "Bambu_AMS_1=" in msg          # which unit, and where its odometer was


def test_a_load_that_is_never_detected_logs_nothing():
    buf, (unit,) = _make_bambu_buffer()
    # The friction plateau: compressed, but the filament never stopped.
    _feed(buf, unit, [(0.39, 0.10), (0.39, 0.20), (0.39, 0.30)])

    assert not [m for _lvl, m in buf.logger.messages
                if "load detected" in m]


def test_status_publishes_the_gate():
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, LOAD3)

    status = buf.get_status(0.0)

    assert status["load_latched"] is True
    assert status["odom_moved"] is True
    assert status["odom_confirms"] is True
    # The verdict itself, so the gate can be watched on a printer whose
    # extruder is still on a real switch and never consults it.
    assert status["load_detected"] is True
    assert status["advance_state"] is True
    # and still an FPS buffer to anything reading it as one
    assert "fps_value" in status and "set_point" in status


def test_status_reports_no_load_before_one_happens():
    buf, (unit,) = _make_bambu_buffer()
    unit.odom_m = 0.101
    buf._adc_callback(0.25, 0.46)          # resting, centred buffer

    status = buf.get_status(0.0)

    assert status["load_detected"] is False
    assert status["advance_state"] is False
    assert status["odom_confirms"] is False


# ── Releasing the latch on a real runout ──────────────────────────────────────

def test_sustained_max_tension_releases_the_latch():
    """
    The latch cannot be unconditional or a runout would never be reported.
    An emptied buffer sits at 0.02 and stays there, unlike the printing
    sawtooth, which bottoms around 0.13 and recovers within a second.
    """
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, LOAD3)
    assert buf._gate_latched is True

    states = _feed(buf, unit, [(0.02, 2.545)] * 12)

    assert buf._gate_latched is False
    assert states[-1] is False


def test_the_printing_sawtooth_never_starts_the_runout_count():
    """
    The sawtooth's floor is above low_point, so the release counter never
    even begins -- the 2 s dwell is the second line of defence, not the
    first.
    """
    buf, (unit,) = _make_bambu_buffer()
    _feed(buf, unit, LOAD3)

    _feed(buf, unit, [(0.13, 2.545), (0.39, 2.545), (0.63, 2.545),
                      (0.14, 2.545), (0.47, 2.545), (0.95, 2.545)] * 3)

    assert buf._tension_since is None
    assert buf._gate_latched is True
