# _wait_move's completion signals, and the two attempts at a third one.
#
# THE DEFECT. An unload's reel-back finished in 11s on an AMS 2 and 24s on an
# HT, and both then sat out the full 35s deadline. Neither existing signal sees
# it: the fault path only fires on failure, and _MOTION_FINISH_RE matches none
# of the words a completed retract uses.
#
# ATTEMPT 1, THE ODOMETER -- REMOVED. Ending the wait once the unit's own
# odometer went quiet is wrong: the odometer goes quiet during pauses WITHIN
# an operation, and on hardware it ended a real unload early.
#
# ATTEMPT 2, THE UNIT'S OWN WORD. A completed retract ends with
# "state_switch finish, sucessful, err_code:0x00" on BOTH the AMS 2 and the HT
# -- verified across every capture carrying sniff frames, 13 occurrences at
# 0x00, every one after a completed operation and none before one. That is a
# statement of completion by the unit, not an inference from its silence.
#
# AMS 1 IS NOT COVERED and is not meant to be: it speaks the AMS_DEV dialect
# and never emits this line (0 hits in the AMS 1 captures). It keeps the
# odom-reset / no-tray completion the reader already gives it.
from __future__ import annotations

import types

import logging
import pytest

from extras.AFC_BambuAMS import afcBambuAMS


class _Reactor:
    """A clock the test drives, so a 3s quiet window costs no wall time."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def pause(self, until):
        self.t = max(self.t, until)


class _Bridge:
    def __init__(self, finish=(0, True, "")):
        self._finish = finish

    def last_finish(self):
        return self._finish


def _unit(odom_series, *, finish=(0, True, "")):
    """A unit whose odometer reads through `odom_series`, then holds the last."""
    u = afcBambuAMS.__new__(afcBambuAMS)
    u.name = "AMS"
    u.ams_index = 0
    u._bridge = _Bridge(finish)
    r = _Reactor()
    u.afc = types.SimpleNamespace(reactor=r)
    u.reactor = r
    u.logger = types.SimpleNamespace(info=lambda *a, **k: None,
                                     debug=lambda *a, **k: None,
                                     warning=lambda *a, **k: None)
    seq = list(odom_series)
    def next_odom():
        return seq.pop(0) if seq else (odom_series[-1] if odom_series else None)
    u._odom_now_mm = next_odom
    u._ams_fault_since = lambda mark, consume=False: False
    return u, r


def _wait(u, mm=2000.0, **kw):
    return afcBambuAMS._wait_move(u, mm, **kw)


def test_a_still_odometer_does_not_end_the_wait():
    # The unit pauses mid-move during its own retry cycles, so a quiet
    # odometer is never read as a finished move.
    u, r = _unit([100.0, 200.0, 300.0] + [300.0] * 500)
    t0 = r.t
    assert _wait(u) is False
    assert r.t - t0 >= 30.0


def test_narration_still_wins_when_it_arrives():
    # The existing signal keeps priority and its success/failure verdict.
    u, r = _unit([100.0] * 500, finish=(7, False, "pull finish -1"))
    assert _wait(u) is False   # ok=False from narration


def test_every_exit_actually_says_which_signal_it_was():
    # REGRESSION, and a nasty one. The diagnostics were refactored behind a
    # local _say() helper whose body was rewritten to call _say(msg) -- itself.
    # Every diagnostic recursed, RecursionError was swallowed by the helper's
    # own except, and the logging went silent while every test still passed.
    # On hardware that showed up as two unloads producing no attribution at
    # all. So assert the log line, not just the return value.
    for kw, want in ((dict(), "DEADLINE"),):
        seen = []
        u, r = _unit([100.0, 200.0, 300.0] + [300.0] * 500)
        u.logger = types.SimpleNamespace(
            info=lambda *a, **k: None, warning=lambda *a, **k: None,
            debug=lambda m, *a, **k: seen.append(str(m)))
        _wait(u, **kw)
        assert any(want in m for m in seen), \
            f"no {want} diagnostic; got {seen}"

    seen = []
    u, r = _unit([100.0] * 500)
    # The seq must CHANGE after the wait samples it at the start -- a fixed
    # value can never trip `seq != start_seq`, which is the whole signal.
    calls = {"n": 0}
    def moving_finish():
        calls["n"] += 1
        return (0, True, "") if calls["n"] == 1 else (9, True, "pull finish")
    u._bridge.last_finish = moving_finish
    u.logger = types.SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
        debug=lambda m, *a, **k: seen.append(str(m)))
    _wait(u)
    assert any("NARRATION" in m for m in seen), seen

    seen = []
    u, r = _unit([100.0] * 500)
    u._ams_fault_since = lambda mark, consume=False: True
    u.logger = types.SimpleNamespace(
        info=lambda *a, **k: None, warning=lambda *a, **k: None,
        debug=lambda m, *a, **k: seen.append(str(m)))
    _wait(u, fault_mark=1)
    assert any("FAULT" in m for m in seen), seen


def test_a_missing_logger_still_cannot_break_a_move():
    # The other half: the guard must survive a shim with no logger at all.
    u, r = _unit([100.0] * 500, finish=(7, True, "pull finish"))
    del u.logger
    _wait(u)


def test_a_declared_fault_ends_the_wait():
    u, r = _unit([100.0] * 500)
    u._ams_fault_since = lambda mark, consume=False: True
    t0 = r.t
    assert _wait(u, fault_mark=1) is False
    assert r.t - t0 < 5.0, "a declared fault must end the wait immediately"


# ── attempt 2: the unit's own completion word ───────────────────────────────

from extras.AFC_BambuAMS_bridge import _STATE_SWITCH_DONE_RE


class _SwitchBridge(_Bridge):
    """A bridge whose state-switch counter the test advances by hand."""

    def __init__(self, finish=(0, True, ""), switch=(0, "")):
        super().__init__(finish)
        self._switch = switch

    def last_switch_finish(self):
        return self._switch


# Verbatim from the captures named beside them. If the regex is ever retuned,
# these are the lines it has to keep matching.
AMS2_UNLOAD_LINE = "[AMS_SWITCH]SRL_state_switch finish, sucessful, err_code:0x00"
HT_UNLOAD_LINE = "[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, err_code:0x00"
HT_LOAD_LINE_OLDFW = "[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, err_code:0x25"


@pytest.mark.parametrize("line", [AMS2_UNLOAD_LINE, HT_UNLOAD_LINE])
def test_the_regex_matches_a_real_completed_retract(line):
    # ams2_postupd_unload and ams_ht_postupd_full respectively. The prefix
    # differs between them, which is why it is not anchored.
    assert _STATE_SWITCH_DONE_RE.search(line)


def test_a_nonzero_err_code_is_not_a_completion():
    # Six of these exist, all on pre-update firmware and all AFTER a feed
    # finish that already ended the wait. Matching them would end a wait on a
    # line that says nothing about this move.
    assert not _STATE_SWITCH_DONE_RE.search(HT_LOAD_LINE_OLDFW)


def test_the_switch_ends_the_wait_only_when_asked():
    u, r = _unit([100.0] * 500)
    u._bridge = _SwitchBridge(switch=(7, AMS2_UNLOAD_LINE))
    # Not asked: the switch is already at 7 and must be ignored entirely.
    t0 = r.t
    assert _wait(u) is False                     # runs to the deadline
    assert r.t - t0 >= 30.0


def test_a_switch_that_predates_the_wait_does_not_end_it():
    # THE REASON THIS HAS ITS OWN COUNTER. On a load the token lands just after
    # the feed finish that ended the previous wait; if the next wait accepted a
    # stale sequence it would return instantly having measured nothing.
    u, r = _unit([100.0] * 500)
    u._bridge = _SwitchBridge(switch=(3, AMS2_UNLOAD_LINE))
    t0 = r.t
    assert _wait(u, accept_switch_finish=True) is False
    assert r.t - t0 >= 30.0, "a pre-existing switch must not end the wait"


def test_a_switch_arriving_during_the_wait_ends_it():
    u, r = _unit([100.0] * 500)
    br = _SwitchBridge(switch=(3, ""))
    u._bridge = br
    real_pause = r.pause
    def pause(until):
        real_pause(until)
        if r.t > 1002.0:                          # a couple of seconds in
            br._switch = (4, HT_UNLOAD_LINE)
    r.pause = pause
    t0 = r.t
    assert _wait(u, accept_switch_finish=True) is True
    assert r.t - t0 < 10.0, "should end on the switch, not the deadline"


# ── ATTEMPT 3, AND THE ONE THE AMS 2 ACTUALLY SPEAKS ────────────────────────
#
# `state_switch finish` (attempt 2, above) is right about the HT and only
# 6/7 right about the AMS 2. Measured on the U1's AMS 2 across a day of real
# unloads, each with a known retract start and a known AFC completion:
#
#     AMS_CTRL_state_switch finish   6/7    -24s .. +1s vs AFC "done"
#     [AMS_SWITCH]pull finish        2/7
#     tray_now -> 255                7/7    always at the tray switch release
#
# The tray_now EDGE is the unit's own report that the tray it was unloading is
# no longer the current one. Two things make it usable, and both are tested
# here because each was falsified by the same day's data:
#
#   * it is an EDGE. `state:0,tray_now:255` is the RESTING level (120 hits),
#     so a waiter reading the value completes before the reel turns.
#   * it must be OUR tray. 18 of the day's 25 edges were the operator
#     preloading or inserting a spool, which produces an identical transition
#     on whichever tray they touched.


class _TrayBridge:
    """A bridge that reports tray releases, per unit, as (seq, tray)."""

    def __init__(self, release=(0, None), finish=(0, True, "")):
        self._release = release
        self._finish = finish

    def last_finish(self):
        return self._finish

    def last_switch_finish(self):
        return (0, "")

    def last_tray_release(self, unit=None):
        if unit is None:
            return (0, None)
        return self._release


def test_the_tray_release_of_our_tray_ends_the_wait():
    u, r = _unit([100.0] * 500)
    br = _TrayBridge()
    u._bridge = br
    real_pause = r.pause

    def pause(until):
        real_pause(until)
        if r.t > 1002.0:
            br._release = (1, 2)              # tray 2 released, mid-wait
    r.pause = pause
    t0 = r.t
    assert _wait(u, accept_tray_release=2) is True
    assert r.t - t0 < 10.0, "should end on the release, not the deadline"


def test_a_release_of_someone_elses_tray_is_ignored():
    """The operator handling a spool during an unload must not end it.

    18 of 25 tray_now edges in a measured day were exactly this.
    """
    u, r = _unit([100.0] * 500)
    br = _TrayBridge()
    u._bridge = br
    real_pause = r.pause

    def pause(until):
        real_pause(until)
        if r.t > 1002.0:
            br._release = (1, 0)              # tray 0 -- not the one we pulled
    r.pause = pause
    t0 = r.t
    assert _wait(u, accept_tray_release=2) is False
    assert r.t - t0 >= 30.0, "another tray's release must not end this move"


def test_a_release_that_predates_the_wait_does_not_end_it():
    """The resting level is 255, so only a NEW edge may count."""
    u, r = _unit([100.0] * 500)
    u._bridge = _TrayBridge(release=(5, 2))   # already released, before we began
    t0 = r.t
    assert _wait(u, accept_tray_release=2) is False
    assert r.t - t0 >= 30.0, "a pre-existing release must not end the wait"


def test_the_tray_release_is_opt_in():
    u, r = _unit([100.0] * 500)
    u._bridge = _TrayBridge(release=(1, 2))
    t0 = r.t
    assert _wait(u) is False
    assert r.t - t0 >= 30.0, "must not apply unless the caller names its tray"


def test_an_older_bridge_without_the_accessor_falls_back_to_the_deadline():
    u, r = _unit([100.0] * 500)
    u._bridge = _SwitchBridge(switch=(0, ""))   # no last_tray_release at all
    t0 = r.t
    assert _wait(u, accept_tray_release=2) is False
    assert r.t - t0 >= 30.0


def test_the_retract_passes_its_own_slot():
    import inspect
    unload = inspect.getsource(afcBambuAMS.unit_unload_lane)
    assert "accept_tray_release=self._slot_of(cur_lane)" in unload, \
        "the tray index is a guard, not decoration -- it must be the lane's"
    load = inspect.getsource(afcBambuAMS._unit_load_lane)
    assert "accept_tray_release" not in load, \
        "a load ends on `feed finish`; releasing a tray is not a load event"


def test_only_the_retract_opts_into_the_switch_finish():
    import inspect
    unload = inspect.getsource(afcBambuAMS.unit_unload_lane)
    assert unload.count("accept_switch_finish=True") >= 1, \
        "the retract is the case that has no other completion signal"
    load = inspect.getsource(afcBambuAMS._unit_load_lane)
    assert "accept_switch_finish" not in load, \
        "a load already ends on `feed finish`; the switch lands just after it"


# ── a bridge that goes away mid-print must PAUSE, not just reconnect ─────────

class TestLinkLossPause:
    """Reconnecting quietly is not enough during a print.

    When frames stop, the extruder keeps pulling against an AMS nobody is
    driving: the buffer bottoms out, the filament grinds, and the print is lost
    long before the transport has finished backing off and reconnecting. So the
    detector fires in seconds, not tens of seconds.
    """

    class _Bridge:
        def __init__(self, quiet):
            self._q = quiet
        def silent_for(self):
            return self._q

    def _unit(self, *, quiet, printing=True, lane="lane28",
              threshold=5.0, latched=False):
        u = afcBambuAMS.__new__(afcBambuAMS)
        u.name = "HT_1"
        u.link_loss_pause_s = threshold
        u._bridge = self._Bridge(quiet)
        u._link_loss_paused = latched
        u.logger = logging.getLogger("test-linkloss")
        u.raised = []
        u._raise_ams_fault = lambda ln, msg: u.raised.append((ln, msg))
        u._tool_loaded_lane = lambda: lane
        fn = types.SimpleNamespace(in_print=lambda: printing)
        u.afc = types.SimpleNamespace(function=fn)
        return u

    def test_a_brief_hiccup_does_not_pause(self):
        u = self._unit(quiet=1.5)
        u._check_link_loss()
        assert u.raised == []

    def test_THE_ONE_THAT_MATTERS_sustained_silence_pauses_the_print(self):
        u = self._unit(quiet=6.0)
        u._check_link_loss()
        assert len(u.raised) == 1
        lane, msg = u.raised[0]
        assert lane == "lane28"
        assert "sent nothing" in msg and "Pausing" in msg

    def test_it_fires_once_per_outage_not_every_tick(self):
        u = self._unit(quiet=6.0)
        for _ in range(5):
            u._check_link_loss()
        assert len(u.raised) == 1

    def test_recovery_rearms_it_for_the_next_outage(self):
        # A link that drops, recovers and drops again must pause BOTH times.
        u = self._unit(quiet=6.0)
        u._check_link_loss()
        u._bridge._q = 0.2                      # heard from
        u._check_link_loss()
        assert u._link_loss_paused is False
        u._bridge._q = 6.0                      # gone again
        u._check_link_loss()
        assert len(u.raised) == 2

    def test_not_when_no_lane_of_this_unit_is_at_the_toolhead(self):
        # A dead bridge with nothing loaded here is worth reporting, not worth
        # stopping the machine for.
        u = self._unit(quiet=60.0, lane=None)
        u._check_link_loss()
        assert u.raised == []

    def test_not_outside_a_print(self):
        u = self._unit(quiet=60.0, printing=False)
        u._check_link_loss()
        assert u.raised == []

    def test_in_print_raising_is_treated_as_not_printing(self):
        u = self._unit(quiet=60.0)
        u.afc.function.in_print = lambda: (_ for _ in ()).throw(RuntimeError())
        u._check_link_loss()
        assert u.raised == []

    def test_zero_threshold_disables_it(self):
        u = self._unit(quiet=600.0, threshold=0.0)
        u._check_link_loss()
        assert u.raised == []

    def test_a_bridge_that_never_connected_does_not_pause(self):
        u = self._unit(quiet=None)
        u._check_link_loss()
        assert u.raised == []

    def test_the_default_threshold_is_far_below_the_transport_timers(self):
        # The transport drops a silent link at 30s; this must act well before.
        import extras.AFC_BambuAMS_bridge as br
        assert 2.0 <= 5.0 <= br.BambuBridge.QUIET_DROP_S / 2.0


def test_slot_of_is_total_so_it_cannot_abort_an_unload():
    """accept_tray_release reads _slot_of on the unload path.

    An optimisation that only ever SHORTENS a wait must not be able to raise
    and take the unload with it, so a unit with no slot map answers None --
    which just leaves the wait on its other signals.
    """
    u = afcBambuAMS.__new__(afcBambuAMS)
    lane = types.SimpleNamespace(name="lane5")
    assert afcBambuAMS._slot_of(u, lane) is None      # no _slot_map at all
    u._slot_map = {"lane5": 1}
    assert afcBambuAMS._slot_of(u, lane) == 1
    assert afcBambuAMS._slot_of(u, types.SimpleNamespace(name="nope")) is None
