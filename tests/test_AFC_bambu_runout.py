# Tests for the Bambu bay-presence runout: a bay emptying under the lane that
# is printing runs AFC's runout (infinite spool or pause), as OpenAMS does for
# its F1S sensor.
from __future__ import annotations
import types
from extras.AFC_BambuAMS import afcBambuAMS


class _Logger:
    def __init__(self): self.msgs = []
    def _rec(self, m, *a, **k): self.msgs.append(str(m))
    info = debug = warning = error = raw = _rec


class _Reactor:
    """Holds deferred callbacks so a test decides when the confirm fires."""
    def __init__(self): self.cbs = []
    def monotonic(self): return 100.0
    def register_callback(self, cb, when=0.0): self.cbs.append((cb, when))

    def fire_all(self):
        cbs, self.cbs = self.cbs, []
        for cb, _when in cbs:
            cb(None)


def _lane(name, loaded=True, runout=None, ext=None):
    calls = []
    if ext is None:
        ext = types.SimpleNamespace(lane_loaded=name if loaded else None)
    lane = types.SimpleNamespace(
        name=name, tool_loaded=loaded, extruder_obj=ext, runout_lane=runout,
        status=None,
        _perform_infinite_runout=lambda: calls.append("infinite"),
        _perform_pause_runout=lambda: calls.append("pause"))
    return lane, calls


def _unit(lane, printing=True, online=True, preslen=4, slot=2):
    reactor = _Reactor()
    saved = []
    errors = []
    u = types.SimpleNamespace(
        errors=errors,
        lane_not_ready=lambda ln: None,
        name="Bambu_AMS_1",
        logger=_Logger(),
        ams_index=0,
        pool=False,
        _slot_map={lane.name: slot},
        lanes={lane.name: lane},
        _slots=[{"present": True} for _ in range(4)],
        _scan_primed=True,
        _bridge=types.SimpleNamespace(latest_status=lambda: {
            "units": [{"n": 0, "online": online, "preslen": preslen}]}),
        afc=types.SimpleNamespace(
            reactor=reactor,
            error_state=False,
            lanes={lane.name: lane},
            error=types.SimpleNamespace(
                AFC_error=lambda msg, pause=True: errors.append((msg, pause))),
            function=types.SimpleNamespace(is_printing=lambda: printing),
            save_vars=lambda: saved.append(1)))
    return u, reactor, saved


def _empty(u, slot=2):
    u._slots[slot] = {"present": False}
    afcBambuAMS._arm_presence_runout(u, slot)


def test_bay_emptying_under_the_printing_lane_pauses_without_a_runout_lane():
    # Past the feeder the AMS cannot pull the filament back, so this pauses
    # and never unloads: not AFC's pause runout, which unloads when
    # unload_on_runout is set.
    lane, calls = _lane("lane6")
    u, reactor, saved = _unit(lane)
    _empty(u)
    assert u.errors == []                   # nothing until the confirm fires
    assert reactor.cbs and reactor.cbs[0][1] == 100.0 + afcBambuAMS.RUNOUT_CONFIRM_S
    reactor.fire_all()
    assert calls == []
    assert len(u.errors) == 1 and u.errors[0][1] is True
    assert "Runout on lane6 (Bambu_AMS_1 bay 3)" in u.errors[0][0]
    assert "no runout lane" in u.errors[0][0]
    assert "follower keeps pushing" in u.errors[0][0]
    # ...and its unload skips the AMS retract and keeps the follower on.
    assert lane._bambu_runout_empty is True
    assert saved
    assert any("RUNOUT on lane6" in m for m in u.logger.msgs)


def test_a_runout_lane_on_the_same_extruder_pauses_too():
    # The leftover filament sits in the shared path, so the next lane
    # cannot load through it.
    lane, calls = _lane("lane6", runout="lane4")
    other, _ = _lane("lane4", loaded=False, ext=lane.extruder_obj)
    u, reactor, _ = _unit(lane)
    u.afc.lanes["lane4"] = other
    _empty(u)
    reactor.fire_all()
    assert calls == []
    assert "feeds the same extruder" in u.errors[0][0]


def test_a_runout_lane_on_another_extruder_takes_afcs_infinite_spool():
    lane, calls = _lane("lane6", runout="lane4")
    other, _ = _lane("lane4", loaded=False)
    u, reactor, _ = _unit(lane)
    u.afc.lanes["lane4"] = other
    _empty(u)
    reactor.fire_all()
    assert calls == ["infinite"]
    assert u.errors == []
    # ...and the unload that follows skips the AMS retract.
    assert lane._bambu_runout_empty is True


def test_a_runout_lane_that_does_not_exist_pauses():
    lane, calls = _lane("lane6", runout="lane99")
    u, reactor, _ = _unit(lane)
    _empty(u)
    reactor.fire_all()
    assert calls == []
    assert "lane99 was not found" in u.errors[0][0]


def test_a_bay_that_fills_again_before_the_confirm_is_not_a_runout():
    lane, calls = _lane("lane6")
    u, reactor, _ = _unit(lane)
    _empty(u)
    u._slots[2] = {"present": True}
    reactor.fire_all()
    assert calls == [] and u.errors == []


def test_the_insert_edge_cancels_a_pending_runout():
    # A stray empty frame followed by a full one: the full frame's pass
    # through _maybe_auto_scan drops the pending runout, whatever _slots says
    # by the time the confirm fires.
    lane, calls = _lane("lane6")
    u, reactor, _ = _unit(lane)
    _empty(u)
    u._prev_present = [True, True, False, True]
    u._present_seen = {0, 1, 2, 3}
    u._scan_in_flight = lambda s: True      # take the early, silent path
    afcBambuAMS._maybe_auto_scan(u, 2, True, {})
    u._slots[2] = {"present": False}
    reactor.fire_all()
    assert calls == [] and u.errors == []


def test_not_printing_is_a_spool_taken_out_not_a_runout():
    lane, calls = _lane("lane6")
    u, reactor, _ = _unit(lane, printing=False)
    _empty(u)
    assert reactor.cbs == []
    assert calls == [] and u.errors == []


def test_a_bay_whose_lane_is_not_at_the_toolhead_is_ignored():
    lane, calls = _lane("lane6", loaded=False)
    u, reactor, _ = _unit(lane)
    _empty(u)
    assert reactor.cbs == []


def test_an_offline_unit_or_a_pico_boot_frame_cannot_run_a_lane_out():
    for kw in ({"online": False}, {"preslen": 0}):
        lane, calls = _lane("lane6")
        u, reactor, _ = _unit(lane, **kw)
        _empty(u)
        reactor.fire_all()
        assert calls == [] and u.errors == [], kw
        assert any("cannot vouch" in m for m in u.logger.msgs)


def test_a_print_that_paused_meanwhile_is_left_alone():
    lane, calls = _lane("lane6")
    state = {"printing": True}
    u, reactor, _ = _unit(lane)
    u.afc.function.is_printing = lambda: state["printing"]
    _empty(u)
    state["printing"] = False
    reactor.fire_all()
    assert calls == [] and u.errors == []


def test_a_stale_confirm_does_not_fire_a_newer_arm():
    lane, calls = _lane("lane6")
    u, reactor, _ = _unit(lane)
    _empty(u)
    u._runout_pending[2] = 999.0             # re-armed since
    reactor.fire_all()
    assert calls == [] and u.errors == []


def test_the_removal_edge_arms_the_runout():
    # Through the real edge: the REMOVED line and the arm come together.
    lane, calls = _lane("lane6")
    u, reactor, _ = _unit(lane)
    u._prev_present = [True, True, True, True]
    u._present_seen = {0, 1, 2, 3}
    u._auto_scanned = [False] * 4
    u._scan_notag = [False] * 4
    u._clear_lane_filament = lambda ln: None
    u._unbind_spool = lambda ln: None
    u._release_scan_hold = lambda s: None
    u._spoolman_latched = set()
    u._save_lane_vars = lambda: None
    u._lane_for_slot = afcBambuAMS._lane_for_slot.__get__(u)
    u._slots[2] = {"present": False}
    afcBambuAMS._maybe_auto_scan(u, 2, False, {})
    assert any("spool REMOVED from slot 2" in m for m in u.logger.msgs)
    reactor.fire_all()
    assert len(u.errors) == 1


# ── an AMS 2 bay that still reads present after the tail passed its inlet ──

def _inlet_unit(sw=2, t=90.0):
    lane, calls = _lane("lane14")
    u, reactor, saved = _unit(lane)
    u.dry_dev_addr = 0x0700
    state = {"sw": (sw, t)}
    u._bridge.tray_switches = (
        lambda addr, unit, slot: state["sw"] if (addr, unit, slot)
        == (0x0700, 0, 2) else None)
    u._prev_present = [True, True, True, True]
    u._present_seen = {0, 1, 2, 3}
    u._scan_in_flight = lambda s: False
    return lane, calls, u, reactor, state


def test_an_inlet_that_cleared_under_the_printing_lane_is_a_runout():
    # Printer 1, 2026-09-30: "tray[2] sw_sta update, 3 -> 2" and the bay kept
    # reading present, so lane14 printed its leftover until a manual pause.
    lane, calls, u, reactor, _ = _inlet_unit()
    afcBambuAMS._maybe_auto_scan(u, 2, True, {})
    assert any("inlet switch cleared under lane14" in m for m in u.logger.msgs)
    afcBambuAMS._maybe_auto_scan(u, 2, True, {})    # the next frame
    reactor.fire_all()
    assert len(u.errors) == 1
    assert "Runout on lane14 (Bambu_AMS_1 bay 3)" in u.errors[0][0]
    assert lane._bambu_runout_empty is True


def test_one_inlet_narration_arms_one_runout():
    lane, calls, u, reactor, _ = _inlet_unit()
    afcBambuAMS._maybe_auto_scan(u, 2, True, {})
    reactor.fire_all()
    afcBambuAMS._maybe_auto_scan(u, 2, True, {})
    assert reactor.cbs == []


def test_an_inlet_that_reads_filament_again_before_the_confirm_is_not_one():
    lane, calls, u, reactor, state = _inlet_unit()
    afcBambuAMS._maybe_auto_scan(u, 2, True, {})
    state["sw"] = (3, 95.0)
    reactor.fire_all()
    assert calls == [] and u.errors == []


def test_an_inlet_narration_from_before_the_spool_went_in_is_ignored():
    lane, calls, u, reactor, _ = _inlet_unit(t=90.0)
    u._inserted_at = {2: 95.0}
    afcBambuAMS._maybe_auto_scan(u, 2, True, {})
    assert reactor.cbs == []


def test_a_cleared_inlet_counts_as_a_ran_out_bay():
    # So BAMBU_RUNOUT_PURGE, the leftover push and the stall guard see it.
    lane, calls, u, reactor, _ = _inlet_unit()
    u._presence_ok = True
    assert afcBambuAMS._bay_ran_out(u, lane) is True
