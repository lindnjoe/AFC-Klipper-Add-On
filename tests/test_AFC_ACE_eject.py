"""
Tests for AFC_ACE.eject_lane's reload-suppression (extras/AFC_ACE.py).

Ejecting retracts filament back into the unit, but the spool stays in the slot
(sensor not cleared), so the heartbeat's ready-slot sync would otherwise pull
the just-ejected filament right back in. eject_lane must add the lane to
_hub_load_suppressed (like the tool-unload path does) so auto reload-to-hub is
suppressed until the spool is physically removed or an explicit load runs.
"""
from __future__ import annotations

import contextlib

from extras.AFC_ACE import afcACE
from tests.ace_helpers import FakeAce, FakeHub, FakeLane, FakeLogger, Recorder


def _make_ace(connected=True):
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    unit.retract_speed = 50.0
    unit.eject_buffer = 475.0
    unit._ace = FakeAce(connected=connected)
    unit._ace.unwind_filament = Recorder()
    unit._hub_load_suppressed = set()
    unit._get_slot = Recorder(result=3)
    unit._operation = lambda: contextlib.nullcontext()
    unit._stop_feed_assist = Recorder()
    unit._wait_for_ace_ready = Recorder()
    unit._wait_for_feed_complete = Recorder()
    unit._set_hub_state = Recorder()
    return unit


def _hub_staged_lane():
    lane = FakeLane("lane3", hub_obj=FakeHub())
    lane.dist_hub = 100.0
    lane.tool_loaded = False
    lane.loaded_to_hub = True
    return lane


def test_eject_suppresses_auto_reload():
    unit = _make_ace()
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    # The fix: the ejected lane is suppressed so the ready-slot sync won't
    # immediately pull the filament back in while the spool is still present.
    assert "lane3" in unit._hub_load_suppressed


def test_eject_clears_hub_state():
    unit = _make_ace()
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    assert lane.loaded_to_hub is False
    # _set_hub_state(lane, False) was called — hub signal cleared.
    assert unit._set_hub_state.calls
    assert unit._set_hub_state.last_args[1] is False


def test_eject_retracts_hub_stage_distance():
    unit = _make_ace()
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    # Hub-staged (not tool-loaded): dist_hub (100) + eject_buffer (475) at
    # retract_speed (50).
    assert unit._ace.unwind_filament.calls
    args = unit._ace.unwind_filament.last_args
    assert args[1] == 575.0
    assert args[2] == 50.0


def test_eject_noop_when_disconnected():
    unit = _make_ace(connected=False)
    lane = _hub_staged_lane()
    unit.eject_lane(lane)
    # Nothing happens when the ACE isn't connected — no suppression, no retract,
    # and the caller's loaded_to_hub is left untouched.
    assert "lane3" not in unit._hub_load_suppressed
    assert not unit._ace.unwind_filament.calls
    assert lane.loaded_to_hub is True


# LANE_UNLOAD calls lane_unloading -> prepare_unload before eject_lane, which
# sets _operation_active for an unload sequence that an eject never runs. The
# flag used to survive the eject, stopping slot sync until a restart: a removed
# spool left the lane "detected but not loaded" and a new insert never showed.

def test_eject_ends_the_unload_prepare_unload_started():
    unit = _make_ace()
    unit._operation_active = True      # as prepare_unload leaves it
    unit._prev_states_stale = False
    unit.eject_lane(_hub_staged_lane())
    assert unit._operation_active is False
    assert unit._prev_states_stale is True


def test_eject_clears_the_flag_when_disconnected():
    unit = _make_ace(connected=False)
    unit._operation_active = True
    unit.eject_lane(_hub_staged_lane())
    assert unit._operation_active is False


def test_eject_clears_the_flag_when_the_unwind_fails():
    unit = _make_ace()
    unit._operation_active = True

    def boom(*a, **k):
        raise RuntimeError("unwind refused")

    unit._ace.unwind_filament = boom
    unit.eject_lane(_hub_staged_lane())
    assert unit._operation_active is False


def test_slot_sync_runs_again_after_prepare_unload_and_eject():
    """On the real _operation() guard, which restores the flag it found:
    prepare_unload then eject must leave slot sync running."""
    unit = _make_ace()
    unit._operation_active = False
    unit._operation = afcACE._operation.__get__(unit)
    unit.prepare_unload(_hub_staged_lane(), None, None)
    assert unit._operation_active is True
    unit.eject_lane(_hub_staged_lane())
    assert unit._operation_active is False
