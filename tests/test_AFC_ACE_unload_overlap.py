"""
Tests for the concurrent (overlapping) ACE unload retract in extras/AFC_ACE.py
(_ace_unload_inner).

The tip is cleared in two stages. Stopping the ACE assist frees ~tool_stn of
slack, so the FIRST retract pulls the tip back HALF a tool_stn_unload into that
freed space and blocks. The SECOND retract (a full tool_stn_unload) is issued
async, right before the ACE rollback (unwind_filament), so the two OVERLAP: the
ACE keeps winding filament onto the spool (opening space) while the extruder
pushes the tip the rest of the way through the gears.

These pin the ordering/flags that make the two-stage retract real:
  * assist is (re-)stopped before the retracts,
  * the first retract is half tool_stn_unload and blocks (wait_tool=True),
  * the second retract is a full tool_stn_unload and is async (wait_tool=False),
  * both retracts are issued BEFORE unwind_filament (so the second overlaps).

Style: typed fakes, ordered-event capture.
"""

from __future__ import annotations

from extras.AFC_ACE import afcACE
from tests.ace_helpers import (
    FakeAce, FakeError, FakeFunction, FakeLane, FakeHub, FakeLogger, Recorder,
)


class _Extruder:
    def __init__(self, tool_stn_unload, tool_unload_speed=25.0):
        self.tool_stn_unload = tool_stn_unload
        self.tool_unload_speed = tool_unload_speed


class _DeltaTime:
    def log_with_time(self, msg, debug=True):
        pass


class _AFC:
    def __init__(self, events):
        self._events = events
        self.error = FakeError()
        self.error.handle_lane_failure = Recorder()
        self.function = FakeFunction()
        self.function.log_toolhead_pos = Recorder()
        self.afcDeltaTime = _DeltaTime()
        self.move_calls = []

    def move_e_pos(self, e_amount, speed, log_string="", wait_tool=False):
        self.move_calls.append(
            {"e_amount": e_amount, "speed": speed, "wait_tool": wait_tool})
        self._events.append(("move", wait_tool))


def _make_ace(events, tool_stn_unload=60.0):
    unit = afcACE.__new__(afcACE)
    unit.afc = _AFC(events)
    unit.logger = FakeLogger()
    unit.serial_port = "/dev/ttyACM0"
    unit.retract_speed = 50.0
    ace = FakeAce(connected=True)

    def _unwind(*a, **k):
        events.append(("unwind",))
    ace.unwind_filament = _unwind
    unit._ace = ace
    unit._hub_load_suppressed = set()
    unit._get_slot = Recorder(result=3)
    unit._wait_for_ace_ready = Recorder()
    unit._wait_for_feed_complete = Recorder()
    unit._set_hub_state = Recorder()
    unit.lane_tool_unloaded = Recorder()

    def _stop(slot):
        events.append(("stop_assist", slot))
    unit._stop_feed_assist = _stop
    return unit, _Extruder(tool_stn_unload)


def _idx(events, tag):
    return next(i for i, e in enumerate(events) if e[0] == tag)


def test_two_retracts_first_blocks_second_overlaps_unwind():
    events = []
    unit, ext = _make_ace(events, tool_stn_unload=60.0)
    lane = FakeLane("lane3", hub_obj=FakeHub())

    assert unit._ace_unload_inner(lane, ext) is True

    # assist stopped, THEN two extruder retracts, THEN the ACE rollback — both
    # retracts precede the rollback so the second (async) one overlaps it.
    moves = [i for i, e in enumerate(events) if e[0] == "move"]
    assert len(moves) == 2
    assert _idx(events, "stop_assist") < moves[0] < moves[1] < _idx(events, "unwind")
    assert len(unit.afc.move_calls) == 2
    first, second = unit.afc.move_calls
    assert first["wait_tool"] is True            # first retract blocks (freed slack)
    assert first["e_amount"] == -30.0            # -tool_stn_unload / 2
    assert first["speed"] == ext.tool_unload_speed
    assert second["wait_tool"] is False          # second retract overlaps the unwind
    assert second["e_amount"] == -60.0           # full -tool_stn_unload
    assert second["speed"] == ext.tool_unload_speed


def test_no_retract_move_when_tool_stn_unload_zero_but_still_unwinds():
    events = []
    unit, ext = _make_ace(events, tool_stn_unload=0.0)
    assert unit._ace_unload_inner(FakeLane("lane3", hub_obj=FakeHub()), ext) is True

    # No extruder move when disabled, but the ACE rollback still runs.
    assert unit.afc.move_calls == []
    assert any(e[0] == "unwind" for e in events)
