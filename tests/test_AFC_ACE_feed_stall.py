"""
The ACE 2 reports 'feed_error' when its encoder sees the filament stop while
the motor turns. AFC records it during a move and, when a load still fails
after its retries, says so instead of blaming the bowden length.
"""

from __future__ import annotations

import types

from extras.AFC_ACE import afcACE, MODE_DIRECT

from tests.ace_helpers import FakeAce, FakeAFC, FakeLane, FakeLogger, Recorder


class _Clock:
    """Reactor whose time moves on every pause, so deadlines expire."""

    def __init__(self, start=100.0, step=0.5):
        self.t = start
        self.step = step

    def monotonic(self):
        return self.t

    def pause(self, until):
        self.t = max(self.t + self.step, until)
        return self.t


def _status(slot_status="ready", busy=False):
    slots = [{"index": i, "status": "ready", "slot_status": "ready"}
             for i in range(4)]
    slots[3]["slot_status"] = slot_status
    return {"status": "busy" if busy else "ready", "slots": slots}


def _unit(frames=()):
    unit = afcACE.__new__(afcACE)
    unit.name = "Ace2_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.afc.reactor = _Clock()
    unit.feed_departure_timeout = 3.0
    unit._cached_hw_status = {}
    unit._hw_status_time = None
    unit._last_move_error = None
    ace = FakeAce(connected=True)
    seq = list(frames)
    ace.get_status = lambda timeout=None: seq.pop(0) if len(seq) > 1 else seq[0]
    unit._ace = ace
    return unit


def test_wait_records_the_feed_error_frame():
    unit = _unit([_status("feeding", busy=True), _status("feed_error"),
                  _status(), _status()])
    assert unit._wait_for_feed_complete(3, 3900.0, 140.0) is True
    assert unit._last_move_error == "feed_error"
    assert any("feed_error" in w for w in unit.logger.lines["warning"])


def test_wait_ignores_a_heartbeat_error_older_than_the_move():
    unit = _unit([_status("feeding", busy=True), _status(), _status()])
    unit._cached_hw_status = _status("feed_error")
    unit._hw_status_time = 50.0                    # before the move began
    unit._wait_for_feed_complete(3, 100.0, 140.0)
    assert unit._last_move_error is None


def test_wait_uses_a_heartbeat_error_newer_than_the_move():
    unit = _unit([_status("feeding", busy=True), _status(), _status()])
    unit._cached_hw_status = _status("feed_error")
    unit._hw_status_time = 1000.0
    unit._wait_for_feed_complete(3, 100.0, 140.0)
    assert unit._last_move_error == "feed_error"


def _load_unit(stall):
    lane = FakeLane("lane3", tool_loaded=False)
    lane.buffer_obj = None
    lane.hub_obj = types.SimpleNamespace(afc_bowden_length=3900.0)
    lane.loaded_to_hub = True
    unit = _unit([_status()])
    unit.mode = MODE_DIRECT
    unit.lanes = {"lane3": lane}
    unit._slot_map = {"lane3": 3}
    unit._feed_assist_active = set()
    unit._hub_load_suppressed = set()
    unit.feed_speed = 140.0
    unit.load_retry_pulse = 100.0
    unit.load_retry_timeout = 10.0
    unit._set_hub_state = lambda l, s: None
    unit._wait_for_ace_ready = lambda *a, **k: None
    unit._toolhead_sensor_triggered = lambda l: False
    unit._ace.feed_filament = Recorder()

    def wait(slot, length, speed, lane=None):
        unit._last_move_error = "feed_error" if stall else None
        return True
    unit._wait_for_feed_complete = wait
    unit.afc.function = types.SimpleNamespace(in_print=lambda: False)
    unit.afc.error = types.SimpleNamespace(handle_lane_failure=Recorder())
    return unit, lane


def test_stalled_load_still_kicks_then_reports_the_stall():
    unit, lane = _load_unit(stall=True)
    assert unit._ace_load_inner(lane, types.SimpleNamespace()) is False
    assert unit._ace.feed_filament.call_count > 1          # the kicks ran
    msg = unit.afc.error.handle_lane_failure.last_args[1]
    assert "feed error" in msg and "lane3" in msg
    assert "bowden length" not in msg


def test_short_load_without_a_stall_keeps_the_bowden_hint():
    unit, lane = _load_unit(stall=False)
    assert unit._ace_load_inner(lane, types.SimpleNamespace()) is False
    msg = unit.afc.error.handle_lane_failure.last_args[1]
    assert "bowden length" in msg and "feed error" not in msg


# ── Gentle arrival at the toolhead ────────────────────────────────────────────

def test_load_feeds_fast_then_approaches_slowly():
    unit, lane = _load_unit(stall=False)
    unit.load_approach_length = 150.0
    unit.load_approach_speed = 25.0
    hits = iter([False, False, True])           # pre-check, before approach, done
    unit._toolhead_sensor_triggered = lambda l: next(hits, True)
    unit._start_feed_assist = lambda *a, **k: None
    unit._use_feed_assist = lambda l: False
    assert unit._ace_load_inner(lane, types.SimpleNamespace(tool_stn=0)) is True
    assert [c[0] for c in unit._ace.feed_filament.calls] == [
        (3, 3750.0, 140.0), (3, 150.0, 25.0)]


def test_load_skips_the_approach_once_the_sensor_has_it():
    unit, lane = _load_unit(stall=False)
    unit.load_approach_length = 150.0
    unit.load_approach_speed = 25.0
    hits = iter([False, True])                  # pre-check, before approach
    unit._toolhead_sensor_triggered = lambda l: next(hits, True)
    unit._start_feed_assist = lambda *a, **k: None
    unit._use_feed_assist = lambda l: False
    assert unit._ace_load_inner(lane, types.SimpleNamespace(tool_stn=0)) is True
    assert [c[0] for c in unit._ace.feed_filament.calls] == [(3, 3750.0, 140.0)]


def test_wait_stops_the_feed_at_the_toolhead_sensor():
    unit = _unit([_status("feeding", busy=True), _status("feeding", busy=True),
                  _status()])
    lane = FakeLane("lane3")
    hits = iter([False, False, True])
    unit._toolhead_sensor_triggered = lambda l: next(hits, True)
    assert unit._wait_for_feed_complete(3, 150.0, 25.0, lane) is True
    assert unit._ace.stop_feed_filament.calls[0][0] == (3,)


def test_stop_at_the_sensor_is_resent_until_confirmed():
    # A stop inside the unit's ~250 ms setup window is dropped.
    unit = _unit([_status("feeding", busy=True), _status()])
    unit._stop_feed_at_sensor(3)
    assert unit._ace.stop_feed_filament.call_count == 2
    assert not unit.logger.lines["warning"]


# ── A long feed that trips the slip check early is resent ─────────────────────

def _resend_unit(errors, fed=None):
    unit, lane = _load_unit(stall=False)
    unit.load_approach_length = 150.0
    unit.load_approach_speed = 25.0
    unit._start_feed_assist = lambda *a, **k: None
    unit._use_feed_assist = lambda l: False
    errs = iter(errors)

    def wait(slot, length, speed, lane=None):
        unit._last_move_error = "feed_error" if next(errs, False) else None
        return True
    unit._wait_for_feed_complete = wait
    unit._ace.send_command = lambda cmd, *a, **k: (
        {"feed_info": [{}, {}, {}, {"length": fed}]} if fed is not None else {})
    # pre-check, after each fast attempt..., before the approach, the end
    return unit, lane


def test_early_feed_error_resends_the_rest_of_the_long_feed():
    unit, lane = _resend_unit([True, False], fed=300.0)
    hits = iter([False, False, False, True])    # pre, after 1st, approach, end
    unit._toolhead_sensor_triggered = lambda l: next(hits, True)
    assert unit._ace_load_inner(lane, types.SimpleNamespace(tool_stn=0)) is True
    assert [c[0] for c in unit._ace.feed_filament.calls] == [
        (3, 3750.0, 140.0), (3, 3450.0, 140.0), (3, 150.0, 25.0)]
    assert any("feeding the remaining 3450mm again" in m
               for m in unit.logger.lines["info"])


def test_resend_without_feed_info_sends_the_full_length():
    unit, lane = _resend_unit([True, False])
    hits = iter([False, False, True])           # pre, after 1st, approach
    unit._toolhead_sensor_triggered = lambda l: next(hits, True)
    assert unit._ace_load_inner(lane, types.SimpleNamespace(tool_stn=0)) is True
    assert [c[0] for c in unit._ace.feed_filament.calls] == [
        (3, 3750.0, 140.0), (3, 3750.0, 140.0)]


def test_resends_are_capped():
    unit, lane = _resend_unit([True, True, True, True], fed=100.0)
    unit._toolhead_sensor_triggered = lambda l: False
    assert unit._ace_load_inner(lane, types.SimpleNamespace(tool_stn=0)) is False
    longs = [c[0] for c in unit._ace.feed_filament.calls if c[0][2] == 140.0]
    assert longs == [(3, 3750.0, 140.0), (3, 3650.0, 140.0), (3, 3550.0, 140.0)]
