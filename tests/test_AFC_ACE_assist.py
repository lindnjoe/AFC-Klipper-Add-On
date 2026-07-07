"""
Unit tests for afcACE feed-assist management in extras/AFC_ACE.py

Style: typed fakes (tests/ace_helpers.py) instead of MagicMock, full
state verification after every call, branch-complete coverage:

  _active_assist_lane   — toolhead-lookup failure, every skip branch, the
                          section-name vs th_extruder_name match (the
                          SET_LANE_LOADED watchdog fix), exact-match priority
                          over the fallback candidate
  _maybe_assist_watchdog — disabled / no target / suppressed / already
                          correct / missing lane / assist-off / fires
  cmd_ACE_FEED_ASSIST   — every parameter-validation branch, stop (tracked,
                          drifted-with-ace, drifted-without-ace), start
                          (stop-others-first protocol)
  _start_feed_assist    — suppression cleared on any explicit start; early
                          outs (already active, no hardware)
  _reconcile_feed_assist — stop-others/start ordering, at_toolhead gate incl.
                          the exception path, suppression veto, already-active
                          no-op, assist-disabled no-op, other-unit stop,
                          unresolvable-name no-op
  _get_eject_length / _get_unload_length — both hub cases
  check_runout          — printing / idle / exception
"""

from __future__ import annotations

import pytest

from extras.AFC_ACE import afcACE

from tests.ace_helpers import (
    FakeAce,
    FakeAFC,
    FakeExtruderObj,
    FakeGcmd,
    FakeLane,
    FakeLogger,
    FakeToolheadPrinter,
    Recorder,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _lane(name, ext_section="extruder", ext_physical=None, tool_loaded=True,
          lane_loaded=None):
    ext = FakeExtruderObj(name=ext_section, th_extruder_name=ext_physical,
                          lane_loaded=lane_loaded)
    return FakeLane(name, extruder_obj=ext, tool_loaded=tool_loaded)


def _make_unit(lanes=(), slot_map=None, active_extruder="extruder"):
    unit = afcACE.__new__(afcACE)
    unit.name = "ACE_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.lanes = {}
    for lane in lanes:
        unit.lanes[lane.name] = lane
        unit.afc.lanes[lane.name] = lane
    unit._slot_map = dict(slot_map or {})
    unit._feed_assist_active = set()
    unit._assist_suppressed = set()
    unit._assist_watchdog = True
    unit._ace = None
    unit.printer = FakeToolheadPrinter(active_extruder=active_extruder)
    return unit


# ── _active_assist_lane ───────────────────────────────────────────────────────

def test_active_assist_lane_matches_section_name():
    lane = _lane("lane0", ext_section="extruder", lane_loaded="lane0")
    unit = _make_unit([lane], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_matches_physical_name():
    """[AFC_extruder e0] with extruder_name: extruder — toolhead reports the
    PHYSICAL name; the lane must still resolve (SET_LANE_LOADED fix)."""
    lane = _lane("lane0", ext_section="e0", ext_physical="extruder",
                 lane_loaded="lane0")
    unit = _make_unit([lane], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_toolhead_lookup_failure_returns_none():
    lane = _lane("lane0", lane_loaded="lane0")
    unit = _make_unit([lane], active_extruder=None)  # lookup raises
    assert unit._active_assist_lane() is None


def test_active_assist_lane_skips_lane_without_extruder():
    lane = _lane("lane0", lane_loaded="lane0")
    lane.extruder_obj = None
    unit = _make_unit([lane], active_extruder="extruder")
    assert unit._active_assist_lane() is None


def test_active_assist_lane_skips_unloaded_lanes():
    lane = _lane("lane0", tool_loaded=False)
    unit = _make_unit([lane], active_extruder="extruder")
    assert unit._active_assist_lane() is None


def test_active_assist_lane_ignores_other_extruders():
    lane = _lane("lane0", ext_section="e0", ext_physical="extruder",
                 lane_loaded="lane0")
    unit = _make_unit([lane], active_extruder="extruder4")
    assert unit._active_assist_lane() is None


def test_active_assist_lane_fallback_when_lane_loaded_lags():
    """extruder.lane_loaded lags tool_loaded at print start — the unique
    loaded lane is still the assist target via the fallback candidate."""
    lane = _lane("lane0", lane_loaded=None)
    unit = _make_unit([lane], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane0"


def test_active_assist_lane_exact_match_beats_candidate():
    """A lane the extruder RECORDS as loaded wins over an earlier
    tool_loaded-only candidate."""
    ext = FakeExtruderObj(name="extruder", lane_loaded="lane1")
    lane0 = FakeLane("lane0", extruder_obj=ext, tool_loaded=True)
    lane1 = FakeLane("lane1", extruder_obj=ext, tool_loaded=True)
    unit = _make_unit([lane0, lane1], active_extruder="extruder")
    assert unit._active_assist_lane() == "lane1"


# ── _maybe_assist_watchdog ────────────────────────────────────────────────────

def _watchdog_unit(**kw):
    lane = _lane("lane0", lane_loaded="lane0")
    unit = _make_unit([lane], slot_map={"lane0": 0},
                      active_extruder="extruder", **kw)
    unit._use_feed_assist = Recorder(result=True)
    return unit


def test_watchdog_schedules_reconcile_when_assist_missing():
    unit = _watchdog_unit()
    unit._maybe_assist_watchdog()
    assert unit.afc.reactor.register_callback.call_count == 1
    assert "enabling feed assist" in unit.logger.lines["info"][0]


def test_watchdog_fires_when_wrong_slot_assisting():
    unit = _watchdog_unit()
    unit._feed_assist_active = {3}
    unit._maybe_assist_watchdog()
    assert unit.afc.reactor.register_callback.call_count == 1


def test_watchdog_noop_when_assist_already_correct():
    unit = _watchdog_unit()
    unit._feed_assist_active = {0}
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_respects_manual_suppression():
    unit = _watchdog_unit()
    unit._assist_suppressed = {0}
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_disabled_by_config():
    unit = _watchdog_unit()
    unit._assist_watchdog = False
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_noop_without_active_lane():
    unit = _make_unit(active_extruder="extruder")  # no lanes at all
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


def test_watchdog_noop_when_assist_disabled_for_lane():
    unit = _watchdog_unit()
    unit._use_feed_assist = Recorder(result=False)
    unit._maybe_assist_watchdog()
    assert not unit.afc.reactor.register_callback.called


# ── cmd_ACE_FEED_ASSIST ───────────────────────────────────────────────────────

def test_feed_assist_cmd_requires_enable():
    unit = _make_unit()
    with pytest.raises(RuntimeError, match="ENABLE is required"):
        unit.cmd_ACE_FEED_ASSIST(FakeGcmd(LANE="lane0"))


def test_feed_assist_cmd_requires_lane_or_slot():
    unit = _make_unit()
    with pytest.raises(RuntimeError, match="LANE or SLOT"):
        unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=0))


def test_feed_assist_cmd_unknown_lane():
    unit = _make_unit()
    with pytest.raises(RuntimeError, match="Unknown lane"):
        unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=0, LANE="nope"))


def test_feed_assist_stop_suppresses_and_stops_tracked_slot():
    unit = _make_unit(slot_map={"lane0": 2})
    unit._feed_assist_active = {2}
    unit._stop_feed_assist = Recorder()

    gcmd = FakeGcmd(ENABLE=0, LANE="lane0")
    unit.cmd_ACE_FEED_ASSIST(gcmd)

    assert unit._assist_suppressed == {2}
    assert unit._stop_feed_assist.calls == [((2,), {})]
    assert "suppressed" in gcmd.responses[0]


def test_feed_assist_stop_sends_firmware_stop_on_tracking_drift():
    """Slot not tracked as assisting, but firmware might be — the manual stop
    must still reach the hardware."""
    unit = _make_unit(slot_map={"lane0": 2})
    unit._ace = FakeAce(connected=True)
    unit._stop_feed_assist = Recorder()

    unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=0, LANE="lane0"))

    assert unit._assist_suppressed == {2}
    assert not unit._stop_feed_assist.called            # not tracked
    assert unit._ace.stop_feed_assist.calls == [((2,), {})]  # raw stop


def test_feed_assist_stop_untracked_without_hardware_only_suppresses():
    unit = _make_unit(slot_map={"lane0": 2})  # _ace is None
    unit._stop_feed_assist = Recorder()

    gcmd = FakeGcmd(ENABLE=0, LANE="lane0")
    unit.cmd_ACE_FEED_ASSIST(gcmd)

    assert unit._assist_suppressed == {2}
    assert not unit._stop_feed_assist.called
    assert len(gcmd.responses) == 1


def test_feed_assist_start_stops_other_slots_first():
    """The ACE can only feed-assist one slot at a time — a manual start must
    stop the other assisting slot(s) before starting (or the ACE refuses
    with error_2)."""
    unit = _make_unit(slot_map={"lane0": 0, "lane1": 1})
    unit._feed_assist_active = {1}
    calls = []
    unit._stop_feed_assist = lambda s: calls.append(("stop", s))
    unit._start_feed_assist = lambda s: calls.append(("start", s))

    gcmd = FakeGcmd(ENABLE=1, LANE="lane0")
    unit.cmd_ACE_FEED_ASSIST(gcmd)

    assert calls == [("stop", 1), ("start", 0)]
    assert "started" in gcmd.responses[0]


def test_feed_assist_start_accepts_slot_param():
    unit = _make_unit()
    unit._start_feed_assist = Recorder()
    unit.cmd_ACE_FEED_ASSIST(FakeGcmd(ENABLE=1, SLOT=3))
    assert unit._start_feed_assist.calls == [((3,), {})]


# ── _start_feed_assist (real method) early-outs ───────────────────────────────

def test_explicit_start_clears_suppression():
    """Any explicit start ends the manual suppression — with no hardware
    connected the method early-outs right after the discard."""
    unit = _make_unit()
    unit._assist_suppressed = {2}
    unit._start_feed_assist(2)
    assert unit._assist_suppressed == set()
    assert unit._feed_assist_active == set()  # no hardware -> not tracked


def test_start_already_active_is_noop():
    unit = _make_unit()
    unit._ace = FakeAce(connected=True)
    unit._feed_assist_active = {2}

    unit._start_feed_assist(2)

    assert not unit._ace.start_feed_assist.called
    assert unit._feed_assist_active == {2}


# ── _reconcile_feed_assist ────────────────────────────────────────────────────

def _reconcile_unit():
    lane = _lane("lane0")
    unit = _make_unit([lane], slot_map={"lane0": 0, "lane1": 1})
    unit._use_feed_assist = Recorder(result=True)
    unit._toolhead_sensor_triggered = Recorder(result=True)
    unit._stop_feed_assist = Recorder()
    unit._start_feed_assist = Recorder()
    return unit


def test_reconcile_stops_other_slots_then_starts_target():
    unit = _reconcile_unit()
    unit._feed_assist_active = {1}
    calls = []
    unit._stop_feed_assist = lambda s: calls.append(("stop", s))
    unit._start_feed_assist = lambda s: calls.append(("start", s))

    unit._reconcile_feed_assist("lane0")

    assert calls == [("stop", 1), ("start", 0)]


def test_reconcile_does_not_start_before_filament_at_toolhead():
    unit = _reconcile_unit()
    unit._toolhead_sensor_triggered = Recorder(result=False)
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_sensor_exception_treated_as_not_at_toolhead():
    unit = _reconcile_unit()
    unit._toolhead_sensor_triggered = Recorder(raises=RuntimeError("boom"))
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_respects_suppression():
    unit = _reconcile_unit()
    unit._assist_suppressed = {0}
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_already_active_does_not_restart():
    unit = _reconcile_unit()
    unit._feed_assist_active = {0}
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called
    assert not unit._stop_feed_assist.called  # target slot is never stopped


def test_reconcile_assist_disabled_for_lane():
    unit = _reconcile_unit()
    unit._use_feed_assist = Recorder(result=False)
    unit._reconcile_feed_assist("lane0")
    assert not unit._start_feed_assist.called


def test_reconcile_lane_on_other_unit_stops_ours():
    unit = _reconcile_unit()
    unit._feed_assist_active = {0}
    unit.afc.lanes["other_lane"] = _lane("other_lane")

    unit._reconcile_feed_assist("other_lane")  # not in our _slot_map

    assert unit._stop_feed_assist.calls == [((0,), {})]
    assert not unit._start_feed_assist.called


def test_reconcile_unresolvable_name_leaves_assist_untouched():
    unit = _reconcile_unit()
    unit._feed_assist_active = {0}

    unit._reconcile_feed_assist("ghost")  # not our lane, not any afc lane

    assert not unit._stop_feed_assist.called
    assert not unit._start_feed_assist.called
    assert unit._feed_assist_active == {0}


# ── Retract distance math ─────────────────────────────────────────────────────

class _Hub:
    def __init__(self, unload=None, bowden=None):
        if unload is not None:
            self.afc_unload_bowden_length = unload
        if bowden is not None:
            self.afc_bowden_length = bowden


def _distance_unit():
    unit = _make_unit()
    unit.eject_buffer = 475.0
    return unit


def test_eject_length_staged_at_hub():
    unit = _distance_unit()
    lane = FakeLane("lane0")
    lane.dist_hub = 300.0
    assert unit._get_eject_length(lane) == 300.0 + 475.0


def test_eject_length_tool_loaded_uses_full_unload_path():
    unit = _distance_unit()
    lane = FakeLane("lane0", tool_loaded=True, hub_obj=None)
    lane.dist_hub = 300.0
    lane.hub_obj = _Hub(unload=1000.0)
    assert unit._get_eject_length(lane) == 300.0 + 1000.0


def test_unload_length_falls_back_to_bowden_length():
    unit = _distance_unit()
    lane = FakeLane("lane0")
    lane.dist_hub = 300.0
    lane.hub_obj = _Hub(bowden=900.0)  # no afc_unload_bowden_length
    assert unit._get_unload_length(lane) == 300.0 + 900.0


def test_unload_length_without_hub():
    unit = _distance_unit()
    lane = FakeLane("lane0")
    lane.dist_hub = 300.0
    lane.hub_obj = None
    assert unit._get_unload_length(lane) == 300.0


# ── check_runout ──────────────────────────────────────────────────────────────

def test_check_runout_true_while_printing():
    unit = _make_unit()
    unit.afc.function.printing = True
    assert unit.check_runout(FakeLane("lane0")) is True


def test_check_runout_false_when_idle():
    unit = _make_unit()
    unit.afc.function.printing = False
    assert unit.check_runout(FakeLane("lane0")) is False


def test_check_runout_false_on_exception():
    unit = _make_unit()
    unit.afc.function.raise_on_is_printing = RuntimeError("boom")
    assert unit.check_runout(FakeLane("lane0")) is False
