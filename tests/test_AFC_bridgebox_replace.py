"""AFC_BridgeBox: a new unit takes over the bay of a unit that is offline.

With a pool, a unit plugged in while every bay of its family is held waits
for one. When a bay of its family is held for a unit that is offline, the
one no-bay console line names AFC_BRIDGEBOX_REPLACE beside FORGET, and once
the new unit has been on the chain for enroll_grace and the old one offline
for release_grace on a live chain, the new unit is offered that bay in a
popup, once per wait. AFC_BRIDGEBOX_REPLACE is FORGET of the old unit, then
ASSIGN of the new one onto the bay it frees: the new unit takes the bay's
name, lanes and T#, and none of what was saved for the old one.
"""
from __future__ import annotations

import types

import pytest

from extras.AFC_BridgeBox import _ChildCommand
from tests import test_AFC_bridgebox_lane_records as lr
from tests.test_AFC_BridgeBox import _CapGCode, _GCmd, _Logger, _mk_files
from tests.test_AFC_BridgeBox import _Printer
from tests.test_AFC_bridgebox_ams_bays import (A, B, C, D, E, FOUR, G, H,
                                               POOL, SEC, _boot, _claimable,
                                               _on_the_wire, _record, _uids)
from tests.test_AFC_bridgebox_two_chains import _master

REPLACE_D = (f"AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={E} OLD={D}")
OFFER = "action:prompt_begin No free bay for new AMS"


class _Clock:
    """A reactor whose monotonic() is the time the test last ticked at."""

    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def register_callback(self, cb, when=None):
        pass

    def register_timer(self, cb, when=None):
        return object()


class _PrintStats:
    def __init__(self, state="standby"):
        self.state = state

    def get_status(self, _now):
        return {"state": self.state}


def _stuck(tmp_path, monkeypatch, online=(A, B, C, E, H), roster=FOUR,
           uids=(A, B, C, D, E, G, H), maps=None, **over):
    """Four AMS recorded and on their bays but D, which is unplugged; E is
    plugged in live and finds no free AMS bay. ``uids`` is the chain, H at
    index 6; ``maps`` the name_map and lane_map an earlier start saved."""
    _record(tmp_path, roster, **(maps or {}))
    clock, gcode = _Clock(), _CapGCode()
    printer = _Printer({"gcode": gcode})
    printer.get_reactor = lambda: clock
    opts = dict(POOL)
    opts.update(over)
    roster_opt = opts.pop("roster_option", "")
    m = _mk_files(tmp_path, roster=roster_opt, printer=printer, **opts)[0]
    m.logger = _Logger()
    _claimable(m, printer, monkeypatch)
    printer.objects["AFC"].tools = {}
    bridge = _on_the_wire(m, monkeypatch, set(online), uids=uids)
    bridge._htmask = 1 << 6                         # H is the HT
    return types.SimpleNamespace(m=m, p=printer, clock=clock, gcode=gcode,
                                 bridge=bridge)


def _tick(s, t0, t1):
    """Run the chain watch once a second from t0 to t1, inclusive."""
    t = t0
    while t <= t1:
        s.clock.now = float(t)
        s.m._scout_tick(float(t))
        t += 1


def _offers(s):
    return [x for x in s.gcode.raw if x.startswith("// " + OFFER)]


def _bay(m, name):
    return next(pu for pu in m._pool_units if pu["name"] == name)


def _state(tmp_path):
    return (tmp_path / "AFC_BridgeBox.cfg").read_bytes()


def _no_bay_lines(s, uid=E):
    return [x for x in s.m.logger.lines if f"{uid} has no" in x]


# ── the no-bay console line ─────────────────────────────────────────────────

class TestTheNoBayLine:
    def test_it_names_replace_beside_forget_once(self, tmp_path,
                                                 monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 130)
        (msg,) = _no_bay_lines(s)
        assert (f"Bambu_AMS_4 ({D}) is offline: if this AMS replaces it, "
                f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={D} frees that bay "
                f"and this AMS "
                f"claims it live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={E} "
                f"OLD=Bambu_AMS_4 does both in one step.") in msg
        # A Bambu bus addresses four AMS: no restart or pool_ams adds a bay.
        assert "raise pool_ams" not in msg and "RESTART builds" not in msg
        assert s.m.get_status()["waiting_for_bay"] == [E]

    def test_with_every_holder_online_it_names_no_replace(self, tmp_path,
                                                          monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, D, E, H))
        _tick(s, 100, 130)
        (msg,) = _no_bay_lines(s)
        assert "REPLACE" not in msg
        assert _offers(s) == []
        assert s.m.get_status()["waiting_for_bay"] == [E]
        s.bridge._online[3] = False                 # D is unplugged later
        _tick(s, 131, 140)
        assert _offers(s) == []
        _tick(s, 141, 141)
        assert len(_offers(s)) == 1
        assert len(_no_bay_lines(s)) == 1

    def test_with_roster_set_it_names_no_replace(self, tmp_path,
                                                 monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster_option=FOUR)
        _tick(s, 100, 130)
        (msg,) = _no_bay_lines(s)
        assert "REPLACE" not in msg and "roster:" in msg
        assert _offers(s) == [] and s.m._replace_offered == set()

    def test_with_fewer_ams_bays_it_names_the_offline_one_first(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster=f"boxed:{A}, boxed:{B}, "
                                                 f"ht:{H}",
                   online=(A, E, H), pool_ams=2)
        _tick(s, 100, 101)
        (msg,) = _no_bay_lines(s)
        assert (f"every AMS bay built belongs to a known unit. Bambu_AMS_2 "
                f"({B}) is offline: if this AMS replaces it, "
                f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={B} frees that bay "
                f"and this AMS "
                f"claims it live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={E} "
                f"OLD=Bambu_AMS_2 does both in one step. Once it is "
                f"recorded, RESTART builds it one, past the AMS band") in msg

    def test_an_ht_is_told_too(self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster=f"boxed:{A}, ht:{H}",
                   online=(A, G), pool_ams=1, pool_ht=1)
        s.bridge._htmask |= 1 << 5                  # G answers as an HT
        _tick(s, 100, 101)
        (msg,) = _no_bay_lines(s, G)
        assert (f"Bambu_AMS_HT_1 ({H}) is offline: if this HT replaces it, "
                f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={H} frees that bay "
                f"and this HT "
                f"claims it live. AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={G} "
                f"OLD=Bambu_AMS_HT_1 does both in one step.") in msg

    def test_two_ams_plugged_in_for_one_spare_are_not_told_to_restart(
            self, tmp_path, monkeypatch):
        # E takes the spare; G finds all four AMS bays held. E is saved on
        # the spare once recorded, so a restart gives G no bay either.
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, E, G, H),
                   roster=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _tick(s, 100, 120)
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E
        (msg,) = _no_bay_lines(s, G)
        assert msg.startswith(
            f"AFC_BridgeBox chain1: AMS {G} has no bay: all 4 AMS bays belong "
            f"to other units (Bambu_AMS_1 ({A}), Bambu_AMS_2 ({B}), "
            f"Bambu_AMS_3 ({C}), Bambu_AMS_4 ({E})), and a Bambu bus "
            f"addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
            f"one.")
        assert "RESTART builds" not in msg and "raise pool_ams" not in msg
        assert "REPLACE" not in msg                 # every holder is online
        roster = s.m._state_get(SEC, "roster")
        assert f"boxed:{E}" in roster and f"boxed:{G}" in roster
        _m2, p2 = _boot(tmp_path)
        assert _uids(p2)["Bambu_AMS_4"] == E
        assert G not in _uids(p2).values()

    def test_a_roster_option_with_room_says_to_list_it(self, tmp_path,
                                                       monkeypatch):
        # roster: lists three AMS; E, unlisted, holds the spare. G is told
        # to add itself there, and the restart then builds it that bay.
        three = f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}"
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, E, G, H),
                   roster=three, roster_option=three)
        _tick(s, 100, 101)
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E
        (msg,) = _no_bay_lines(s, G)
        assert msg.endswith(f"roster: is set and does not list it: add "
                            f"boxed:{G} to roster: and RESTART to give it a "
                            f"bay.")
        m2 = _mk_files(tmp_path, roster=three + f", boxed:{G}", **POOL)[0]
        assert {pu["name"]: pu["uid"] for pu in m2._pool_units}[
            "Bambu_AMS_4"] == G

    def test_a_roster_option_listing_it_fifth_names_no_replace(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster_option=FOUR + f", boxed:{E}")
        _tick(s, 100, 130)
        msg = s.m._no_bay_message(E, "ams")
        assert (f"Bambu_AMS_4 ({D}) is offline: if this AMS replaces it, "
                f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={D} frees that bay "
                f"and this AMS claims it live; remove it from roster: too."
                ) in msg
        assert "REPLACE" not in msg
        assert _offers(s) == []


# ── the popup ───────────────────────────────────────────────────────────────

class TestTheOffer:
    def test_it_is_offered_once_after_both_graces(self, tmp_path,
                                                  monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 114)
        assert _offers(s) == []
        _tick(s, 115, 200)
        assert len(_offers(s)) == 1
        at = s.gcode.raw.index("// " + OFFER)
        shown = s.gcode.raw[at:s.gcode.raw.index("// action:prompt_show",
                                                 at)]
        buttons = [x for x in shown if "prompt_button" in x]
        assert buttons == [f"// action:prompt_button Replace Bambu_AMS_4|"
                           f"{REPLACE_D}|error"]
        assert any("Bambu_AMS_4: DDDD, lane36-lane39 (T36-T39), offline 15s"
                   in x for x in shown)

    def test_a_unit_waiting_since_boot_is_offered_too(self, tmp_path,
                                                      monkeypatch):
        # E was recorded beside the four AMS: the ready note told it, so the
        # watch says nothing more, and the offer still comes.
        s = _stuck(tmp_path, monkeypatch, roster=FOUR + f", boxed:{E}")
        _tick(s, 100, 114)
        recorded = [x for x in s.gcode.raw
                    if f"action:prompt_begin No bay for AMS {E}" in x]
        assert _offers(s) == [] and recorded == []
        _tick(s, 115, 115)
        # E is recorded, so the picker does not call it new.
        recorded = [x for x in s.gcode.raw
                    if f"action:prompt_begin No bay for AMS {E}" in x]
        assert len(recorded) == 1 and _offers(s) == []
        assert (f"UID {E} is recorded but has no bay: every AMS bay is "
                f"taken.") in "".join(s.gcode.raw)
        assert _no_bay_lines(s) == []

    def test_an_offer_queued_before_a_print_waits_for_it(self, tmp_path,
                                                         monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        s.m._popup_active_until = 1e9               # another popup's turn
        _tick(s, 100, 115)
        assert s.m._popup_queue == [("replace", E)]
        stats = s.p.objects["print_stats"] = _PrintStats("printing")
        s.m._popup_active_until = 0.0
        _tick(s, 116, 130)
        assert _offers(s) == [] and E not in s.m._replace_offered
        stats.state = "standby"
        _tick(s, 131, 131)
        assert len(_offers(s)) == 1

    def test_it_waits_for_release_grace(self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, release_grace=40.0)
        _tick(s, 100, 139)
        assert _offers(s) == []
        _tick(s, 140, 140)
        assert len(_offers(s)) == 1

    def test_a_flapping_new_unit_is_never_offered(self, tmp_path,
                                                  monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        for t in range(100, 160):
            s.bridge._online[4] = t % 2 == 0          # E blips
            _tick(s, t, t)
        assert _offers(s) == [] and s.m._replace_offered == set()

    def test_a_phantom_blip_does_not_restart_the_absence_clock(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 104)
        s.bridge._online[3] = True                  # D reads online once
        _tick(s, 105, 105)
        s.bridge._online[3] = False
        _tick(s, 106, 115)
        assert len(_offers(s)) == 1
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == D   # claimed on the blip

    @pytest.mark.parametrize("outage", ["link-down", "chain-dark"])
    def test_an_outage_is_not_absence(self, tmp_path, monkeypatch, outage):
        s = _stuck(tmp_path, monkeypatch)
        online = list(s.bridge._online)
        if outage == "link-down":
            s.bridge._serial = None
        else:
            s.bridge._online = [False] * len(online)
        _tick(s, 100, 130)
        assert s.m._missing_since == {} and _offers(s) == []
        s.bridge._serial, s.bridge._online = object(), online
        _tick(s, 131, 140)
        assert _offers(s) == []                      # D's clock began at 131
        _tick(s, 141, 150)
        assert len(_offers(s)) == 1

    def test_during_a_print_it_waits_for_the_print_to_end(self, tmp_path,
                                                          monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        stats = s.p.objects["print_stats"] = _PrintStats("printing")
        _tick(s, 100, 150)
        assert _offers(s) == [] and s.m._replace_offered == set()
        stats.state = "complete"
        _tick(s, 151, 151)
        assert len(_offers(s)) == 1

    def test_a_bay_with_a_loaded_lane_is_not_offered(self, tmp_path,
                                                     monkeypatch):
        # PREP restored the extruder's record of D's lane36 from the var
        # file, though D's bay was never claimed this session.
        s = _stuck(tmp_path, monkeypatch)
        extruder = types.SimpleNamespace(lane_loaded="lane36")
        s.p.objects["AFC"].tools = {"extruder": extruder}
        _tick(s, 100, 150)
        assert _offers(s) == []
        extruder.lane_loaded = None                 # the record cleared
        _tick(s, 151, 151)
        assert len(_offers(s)) == 1

    def test_an_offer_for_a_unit_no_longer_waiting_is_dropped_and_made_again(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        s.m._popup_active_until = 1e9               # another popup's turn
        _tick(s, 100, 115)
        assert s.m._popup_queue == [("replace", E)]
        s.bridge._online[4] = False                 # E pulled before its turn
        _tick(s, 116, 116)
        assert E not in s.m._no_bay
        s.m._popup_active_until = 0.0
        _tick(s, 117, 117)
        assert _offers(s) == [] and E not in s.m._replace_offered
        s.bridge._online[4] = True
        _tick(s, 118, 133)
        assert len(_offers(s)) == 1

    def test_getting_a_bay_ends_the_wait(self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 120)
        s.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        _tick(s, 121, 121)
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E
        assert E not in s.m._no_bay and E not in s.m._no_bay_told
        assert E not in s.m._replace_offered
        assert s.m.get_status()["waiting_for_bay"] == []

    def test_a_pump_of_a_replace_event_shows_the_picker(self, tmp_path,
                                                        monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        s.m._queue_popup(("replace", E))
        s.m._pump_popups(111.0)
        assert len(_offers(s)) == 1

    def test_a_unit_pulled_and_plugged_back_is_offered_again(self, tmp_path,
                                                             monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 115)
        assert len(_offers(s)) == 1
        s.bridge._online[4] = False                 # E pulled
        _tick(s, 116, 125)
        assert E not in s.m._replace_offered
        s.bridge._online[4] = True                  # and back
        _tick(s, 126, 140)
        assert len(_offers(s)) == 1                 # enroll_grace again
        _tick(s, 141, 141)
        assert len(_offers(s)) == 2
        assert len(_no_bay_lines(s)) == 2           # told again too

    def test_a_unit_that_leaves_while_the_chain_is_dark_waits_no_longer(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        assert s.m.get_status()["waiting_for_bay"] == [E]
        online = list(s.bridge._online)
        s.bridge._online = [False] * len(online)    # the chain powered off
        _tick(s, 112, 115)
        online[4] = False                           # E taken off meanwhile
        s.bridge._online = online
        _tick(s, 116, 117)
        assert s.m.get_status()["waiting_for_bay"] == []
        cmd = _GCmd()
        s.m.cmd_AFC_BRIDGEBOX_BAYS(cmd)
        assert "Waiting:" not in cmd.responses[0]
        assert not [x for x in s.gcode.raw if "Replace for" in x]
        with pytest.raises(Exception, match="no unit is waiting for a bay"):
            s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(OLD=D))
        with pytest.raises(Exception, match=f"{E} is not on chain chain1"):
            s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D))
        assert _bay(s.m, "Bambu_AMS_4")["uid"] == D
        s.bridge._online[4] = True                  # E back: told again
        _tick(s, 118, 118)
        assert len(_no_bay_lines(s)) == 2

    def test_the_watch_reads_no_state_file_to_make_an_offer(self, tmp_path,
                                                            monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 114)

        def _read():
            raise AssertionError("state file read")
        monkeypatch.setattr(s.m, "_read_state", _read)
        s.m._offer_replace(115.0, {A, B, C, E, H}, s.m._online_since)
        assert s.m._popup_queue == [("replace", E)]

    def test_an_ht_offer_says_ams_ht(self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster=f"boxed:{A}, ht:{H}",
                   online=(A, G), pool_ams=1, pool_ht=1)
        s.bridge._htmask |= 1 << 5                  # G answers as an HT
        _tick(s, 100, 115)
        at = s.gcode.raw.index("// action:prompt_begin No free bay for new "
                               "AMS HT")
        shown = s.gcode.raw[at:s.gcode.raw.index("// action:prompt_show",
                                                 at)]
        assert [x for x in shown if "prompt_button" in x] == [
            f"// action:prompt_button Replace Bambu_AMS_HT_1|"
            f"AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={G} OLD={H}|error"]
        assert (f"// action:prompt_text UID {G} has no free bay: every AMS HT "
                f"bay is taken.") in shown

    def test_a_bay_no_longer_held_stops_its_absence_clock(self, tmp_path,
                                                          monkeypatch):
        # G claimed the spare and was pulled before it was recorded; with
        # auto_drop the release frees the bay, and G's clock goes with it.
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                   roster=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}",
                   auto_drop=True)
        _tick(s, 100, 104)
        s.bridge._online[5] = False                 # G pulled
        _tick(s, 105, 114)
        assert G in s.m._missing_since
        _tick(s, 115, 115)
        assert _bay(s.m, "Bambu_AMS_4")["uid"] is None
        assert G not in s.m._missing_since


# ── AFC_BRIDGEBOX_REPLACE ───────────────────────────────────────────────────

class TestReplace:
    def _ready(self, tmp_path, monkeypatch, **over):
        """E waiting, D offline past release_grace."""
        s = _stuck(tmp_path, monkeypatch, **over)
        _tick(s, 100, 111)
        return s

    def test_it_forgets_the_old_unit_and_claims_the_new_one_live(
            self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        cmd = _GCmd(UID=E, OLD=D)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        bay = _bay(s.m, "Bambu_AMS_4")
        assert (bay["uid"], bay["bound"]) == (E, E)
        assert s.p.objects["AFC_BambuAMS Bambu_AMS_4"].claimed == [E]
        roster = s.m._state_get(SEC, "roster")
        assert f"boxed:{E}" in roster and D not in roster
        assert f"{E}:Bambu_AMS_4" in s.m._state_get(SEC, "name_map")
        assert f"{E}:36:4" in s.m._state_get(SEC, "lane_map")
        assert D not in (s.m._state_get(SEC, "name_map")
                         + s.m._state_get(SEC, "lane_map"))
        lead, forgot, assigned = cmd.responses
        assert lead == (f"AFC_BridgeBox chain1: replacing {D} on Bambu_AMS_4 "
                        f"with {E}.")
        assert forgot.startswith(f"AFC_BridgeBox chain1: forgot {D} -- "
                                 f"lanes 36-39 and the name Bambu_AMS_4 "
                                 f"freed for reuse")
        assert assigned.startswith(f"AFC_BridgeBox chain1: assigned {E} to "
                                   f"bay 'Bambu_AMS_4' (lane36-lane39, "
                                   f"T36-T39) -- claimed LIVE")
        assert s.m.get_status()["waiting_for_bay"] == []
        # The restart keeps E there, and nothing moves.
        m2, p2 = _boot(tmp_path)
        assert _uids(p2)["Bambu_AMS_4"] == E
        assert m2._layout_notes == []

    def test_old_may_name_the_bay(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD="Bambu_AMS_4"))
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_without_uid_it_takes_the_one_waiting_unit(self, tmp_path,
                                                       monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(OLD=D))
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_without_uid_and_nothing_waiting_it_refuses(self, tmp_path,
                                                        monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, H))
        _tick(s, 100, 111)
        with pytest.raises(Exception, match="no unit is waiting for a bay"):
            s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(OLD=D))

    def test_an_offline_new_unit_is_pinned_for_its_return(self, tmp_path,
                                                          monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster=FOUR + f", boxed:{E}",
                   online=(A, B, C, H))
        _tick(s, 100, 111)
        cmd = _GCmd(UID=E, OLD=D)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        bay = _bay(s.m, "Bambu_AMS_4")
        assert (bay["uid"], bay["bound"]) == (E, None)
        assert cmd.responses[-1].endswith(
            "-- pinned; it claims this bay when next online.")
        s.bridge._online[4] = True
        _tick(s, 112, 112)
        assert bay["bound"] == E

    def test_an_unrecorded_spare_occupant_is_replaced(self, tmp_path,
                                                      monkeypatch):
        # G claimed the spare live and was pulled before it was recorded:
        # nothing is saved for it, but it holds the bay.
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                   roster=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _tick(s, 100, 104)
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == G
        s.bridge._online[5] = False                 # G pulled
        s.bridge._online[4] = True                  # E plugged in
        _tick(s, 105, 116)
        assert G not in s.m._state_get(SEC, "roster")
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=G))
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_two_waiting_units_and_one_bay(self, tmp_path, monkeypatch):
        # E and G both wait; E's offer shows first and E takes D's bay. G's
        # offer, next in the queue, has no bay left: it is dropped unshown,
        # and a button still naming D is refused.
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, E, G, H))
        _tick(s, 100, 115)
        assert len(_offers(s)) == 1
        assert s.m._popup_queue == [("replace", G)]
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D))
        _tick(s, 116, 140)
        assert len(_offers(s)) == 1 and s.m._popup_queue == []
        assert G not in s.m._replace_offered and G in s.m._no_bay
        with pytest.raises(Exception, match=f"no pool bay is named or held "
                                            f"for {D}"):
            s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=G, OLD=D))

    def test_the_handlers_get_their_own_parameters(self, tmp_path,
                                                   monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        seen = []
        monkeypatch.setattr(s.m, "cmd_AFC_BRIDGEBOX_FORGET", lambda g: seen
                            .append(("forget", g.get("UID"), g.get("OLD"),
                                     g.get_int("FORCE", 0))))
        monkeypatch.setattr(s.m, "cmd_AFC_BRIDGEBOX_ASSIGN", lambda g: seen
                            .append(("assign", g.get("UID"), g.get("NAME"),
                                     g.get_int("FORCE", 0))))
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD="Bambu_AMS_4"))
        assert seen == [("forget", D, None, 0),
                        ("assign", E, "Bambu_AMS_4", 0)]

    def test_the_child_command_answers_through_its_parent(self):
        parent = _GCmd(UID=E, FORCE=1)
        child = _ChildCommand(parent, UID=D)
        assert child.get("UID") == D and child.get("NAME", "") == ""
        assert child.get_int("FORCE", 0) == 0
        assert child.get_command_parameters() == {"UID": D}
        child.respond_info("hello")
        assert parent.responses == ["hello"]
        assert child.error is parent.error

    def test_a_failed_assign_says_what_is_left(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)

        def _refuse(gcmd):
            raise gcmd.error("AFC_BRIDGEBOX_ASSIGN: no")
        monkeypatch.setattr(s.m, "cmd_AFC_BRIDGEBOX_ASSIGN", _refuse)
        with pytest.raises(Exception, match=f"{D} is forgotten and bay "
                                            f"'Bambu_AMS_4' is free, but {E} "
                                            f"is not on it"):
            s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D))
        _tick(s, 112, 112)                          # E claims the free bay
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_a_failed_claim_says_the_bay_is_pinned(self, tmp_path,
                                                   monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        unit = s.p.objects["AFC_BambuAMS Bambu_AMS_4"]
        claim = unit.claim

        def _fail(uid, model):
            raise RuntimeError("bus busy")
        unit.claim = _fail
        with pytest.raises(Exception, match=f"{D} is forgotten and bay "
                                            f"'Bambu_AMS_4' is pinned to {E}, "
                                            f"but {E} is not claimed onto it "
                                            f"\\(bus busy\\)"):
            s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D))
        assert _bay(s.m, "Bambu_AMS_4")["uid"] == E
        unit.claim = claim
        _tick(s, 112, 112)                          # the watch claims it
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E


class TestReplaceRefuses:
    """Each refusal changes nothing: not the state file, not a bay."""

    def _refused(self, s, tmp_path, match, **params):
        before = (_state(tmp_path),
                  [dict(pu) for pu in s.m._pool_units])
        with pytest.raises(Exception, match=match) as err:
            s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(**params))
        assert (_state(tmp_path),
                [dict(pu) for pu in s.m._pool_units]) == before
        return str(err.value)

    def _ready(self, tmp_path, monkeypatch, until=111, **over):
        s = _stuck(tmp_path, monkeypatch, **over)
        _tick(s, 100, until)
        return s

    def test_an_online_old_unit_even_with_force(self, tmp_path,
                                                monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        for force in (0, 1):
            self._refused(s, tmp_path, f"{A} on bay 'Bambu_AMS_1' is online",
                          UID=E, OLD=A, FORCE=force)
        msg = self._refused(s, tmp_path, "is online", UID=E, OLD=A)
        assert msg.endswith(f"-- unplug it first, or AFC_BRIDGEBOX_FORGET "
                            f"CHAIN=chain1 UID={A} forgets it and frees the "
                            f"bay now")

    def test_an_old_unit_whose_online_flag_cannot_be_read_even_with_force(
            self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        s.bridge._serial = None                     # the link is down
        for old in (D, A):                          # A is on the chain
            self._refused(s, tmp_path, f"cannot tell whether {old} is "
                                       f"offline: the bridge link is down -- "
                                       f"run this again once it reconnects",
                          UID=E, OLD=old, FORCE=1)
        s.bridge._serial = object()
        s.bridge.latest_status = lambda: None       # no status read yet
        self._refused(s, tmp_path, f"cannot tell whether {D} is offline: the "
                                   f"bridge has not reported the chain yet",
                      UID=E, OLD=D, FORCE=1)

    def test_a_new_unit_on_a_bay_even_with_force(self, tmp_path,
                                                 monkeypatch):
        # A mistyped uid would move a unit that has a bay onto the offline
        # unit's, and forget the offline unit for good.
        s = self._ready(tmp_path, monkeypatch)
        for force in (0, 1):
            self._refused(s, tmp_path, f"{A} is on bay 'Bambu_AMS_1', and "
                                       f"only a unit with no bay takes over "
                                       f"another's -- AFC_BRIDGEBOX_ASSIGN "
                                       f"CHAIN=chain1 UID={A} NAME=<bay name> "
                                       f"moves a unit between bays",
                          UID=A, OLD=D, FORCE=force)
        self._refused(s, tmp_path, f"{A} is on bay 'Bambu_AMS_1'", UID=A)
        assert _offers(s) == []

    def test_an_offline_unit_named_for_its_own_bay(self, tmp_path,
                                                   monkeypatch):
        # D is offline and pinned there: it is not waiting for a bay, and a
        # FORGET would erase its learned values.
        s = self._ready(tmp_path, monkeypatch)
        self._refused(s, tmp_path, f"{D} is on bay 'Bambu_AMS_4'",
                      UID=D, OLD="Bambu_AMS_4", FORCE=1)

    def test_a_bay_saved_for_another_recorded_unit_even_with_force(
            self, tmp_path, monkeypatch):
        # ASSIGN would refuse the bay once FORGET had run, so it is refused
        # before.
        s = self._ready(tmp_path, monkeypatch)
        s.m._state_set({SEC: {"roster": FOUR + f", boxed:{G}"}})
        s.m._name_map[G] = "Bambu_AMS_4"
        self._refused(s, tmp_path, f"bay 'Bambu_AMS_4' is saved for {G} too "
                                   f"-- AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
                                   f"UID={G} or AFC_BRIDGEBOX_UNASSIGN "
                                   f"CHAIN=chain1 UID={G} first",
                      UID=E, OLD=D, FORCE=1)

    def test_before_prep_without_force(self, tmp_path, monkeypatch):
        # PREP restores AFC's loaded-lane record; before it, a lane the
        # record names is not known.
        s = self._ready(tmp_path, monkeypatch)
        s.p.objects["AFC"].prep_done = False
        s.m._ready_at = s.clock.now
        self._refused(s, tmp_path, "PREP has not run yet", UID=E, OLD=D)
        cmd = _GCmd(UID=E, OLD=D, FORCE=1)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert cmd.responses[-1].endswith(
            "-- pinned; it claims this bay once PREP finishes.")
        s.p.objects["AFC"].prep_done = True
        _tick(s, 112, 112)
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_a_bay_of_the_other_family(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch, online=(A, B, C, E))
        self._refused(s, tmp_path, "is an AMS unit but bay 'Bambu_AMS_HT_1' "
                                   "is an HT bay", UID=E, OLD=H)

    def test_a_free_bay_points_at_assign(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        s.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=D))   # frees Bambu_AMS_4
        self._refused(s, tmp_path, f"bay 'Bambu_AMS_4' is free -- "
                                   f"AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 "
                                   f"UID={E} NAME=Bambu_AMS_4 puts {E} on it",
                      UID=E, OLD="Bambu_AMS_4")

    def test_a_free_bay_of_the_other_family_is_refused_for_its_family(
            self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        assert _bay(s.m, "Bambu_AMS_HT_2")["uid"] is None
        self._refused(s, tmp_path, "is an AMS unit but bay 'Bambu_AMS_HT_2' "
                                   "is an HT bay", UID=E, OLD="Bambu_AMS_HT_2")

    def test_an_unknown_bay_or_unit(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        self._refused(s, tmp_path, "no pool bay is named or held for Nope",
                      UID=E, OLD="Nope")
        self._refused(s, tmp_path, "FFFF is not on chain chain1",
                      UID="FFFF", OLD=D)

    def test_a_roster_option(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch, roster_option=FOUR)
        self._refused(s, tmp_path, "roster: is set, so it decides which "
                                   "units get a bay. To swap an offline unit "
                                   f"for {E}, change its entry to boxed:{E} "
                                   "in roster:, run AFC_BRIDGEBOX_FORGET "
                                   "CHAIN=chain1 UID=<old uid> \\(its bay "
                                   "frees now and the new unit claims it "
                                   "live\\), then RESTART.",
                      UID=E, OLD=D)

    def test_a_chain_without_a_pool(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch, pool_ams=0, pool_ht=0)
        self._refused(s, tmp_path, "has no pool bays.*AFC_BRIDGEBOX_FORGET "
                                   "CHAIN=chain1 UID=<old uid> and RESTART",
                      UID=E, OLD=D)

    def test_a_print_without_force(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        s.p.objects["print_stats"] = _PrintStats("printing")
        self._refused(s, tmp_path, "a print is active", UID=E, OLD=D)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D, FORCE=1))
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_inside_release_grace_without_force(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch, until=105)
        self._refused(s, tmp_path, "offline 5s, under release_grace \\(10s\\)",
                      UID=E, OLD=D)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D, FORCE=1))
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_no_counted_absence_without_force(self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        s.bridge._serial = None
        _tick(s, 112, 112)
        s.bridge._serial = object()
        self._refused(s, tmp_path, f"no absence is counted for {D}",
                      UID=E, OLD=D)

    def test_a_boot_restored_loaded_lane_and_force_clears_it(
            self, tmp_path, monkeypatch):
        s = self._ready(tmp_path, monkeypatch)
        saves = []
        ext = types.SimpleNamespace(lane_loaded="lane36")
        s.p.objects["AFC"].tools = {"extruder": ext}
        s.p.objects["AFC"].save_vars = lambda: saves.append(ext.lane_loaded)
        # The lane is not registered while the bay is unclaimed, so no
        # unload or UNSET_LANE_LOADED reaches it.
        msg = self._refused(s, tmp_path, "AFC records lane36 on Bambu_AMS_4 "
                                         "as loaded to the toolhead",
                            UID=E, OLD=D)
        assert msg.endswith(
            f", from {D}. While Bambu_AMS_4 is unclaimed its lanes are not "
            f"registered, so no unload or UNSET_LANE_LOADED reaches them -- "
            f"plug {D} back in and unload it, or take the filament out by "
            f"hand and FORCE=1 clears the record and replaces {D}")
        cmd = _GCmd(UID=E, OLD=D, FORCE=1)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert ext.lane_loaded is None and saves[0] is None
        assert cmd.responses[0].endswith(
            "; cleared lane36 from the toolhead.")
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_a_loaded_lane_of_a_bound_unit_and_force_clears_it(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, D, H))
        _tick(s, 100, 100)
        s.bridge._online[3], s.bridge._online[4] = False, True   # D for E
        _tick(s, 101, 112)
        bay = _bay(s.m, "Bambu_AMS_4")
        assert bay["bound"] == D                    # auto_drop off
        lane = s.p.objects["AFC_lane lane37"]
        lane.unassigned, lane.tool_loaded = False, True
        msg = self._refused(s, tmp_path, "AFC records lane37 on Bambu_AMS_4",
                            UID=E, OLD=D)
        assert msg.endswith(
            "as loaded to the toolhead -- unload it first (UNSET_LANE_LOADED "
            "if the filament is already out), or FORCE=1 to clear it from the "
            "toolhead and replace anyway")
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D, FORCE=1))
        assert lane.tool_loaded is False
        assert bay["bound"] == E


# ── the picker, the bay manager, ASSIGN ─────────────────────────────────────

class TestThePicker:
    def test_replace_without_old_opens_the_picker(self, tmp_path,
                                                  monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        cmd = _GCmd(UID=E)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert len(_offers(s)) == 1
        assert f"Replace Bambu_AMS_4|{REPLACE_D}|error" in "".join(s.gcode.raw)
        assert cmd.responses == [f"AFC_BridgeBox chain1: opened the replace "
                                 f"picker for {E}."]

    def test_a_loaded_bay_is_listed_without_a_button(self, tmp_path,
                                                     monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        s.p.objects["AFC"].tools = {
            "extruder": types.SimpleNamespace(lane_loaded="lane36")}
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E))
        raw = "".join(s.gcode.raw)
        assert "prompt_button Replace" not in raw
        assert (f"AFC records lane36 as loaded to the toolhead: plug {D} back "
                f"in and unload it, or take the filament out by hand and "
                f"{REPLACE_D} FORCE=1 clears the record") in raw
        assert "UNSET_LANE_LOADED" not in raw

    def test_a_claimed_loaded_bay_is_listed_with_its_unload(self, tmp_path,
                                                            monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, D, H))
        _tick(s, 100, 100)
        s.bridge._online[3], s.bridge._online[4] = False, True   # D for E
        _tick(s, 101, 112)
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == D   # auto_drop off
        lane = s.p.objects["AFC_lane lane37"]
        lane.unassigned, lane.tool_loaded = False, True
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E))
        raw = "".join(s.gcode.raw)
        assert "prompt_button Replace" not in raw
        assert (f"AFC records lane37 as loaded to the toolhead: unload it "
                f"(UNSET_LANE_LOADED if the filament is already out), or "
                f"{REPLACE_D} FORCE=1 clears it") in raw

    def test_before_release_grace_it_says_why(self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 103)
        cmd = _GCmd(UID=E)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert _offers(s) == []
        assert cmd.responses == [
            f"AFC_BridgeBox chain1: no AMS bay is held for a unit offline "
            f"10s or longer: Bambu_AMS_4 ({D}, offline 3s) -- "
            f"AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={E} OLD=Bambu_AMS_4 "
            f"FORCE=1 skips the wait."]

    def test_the_bay_manager_lists_waiting_units_with_replace_first(
            self, tmp_path, monkeypatch):
        new = "0123456789ABCDEF01234567"             # a full 24-hex uid
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, new, H),
                   uids=(A, B, C, D, new, G, H))
        _tick(s, 100, 111)
        cmd = _GCmd()
        s.m.cmd_AFC_BRIDGEBOX_BAYS(cmd)
        buttons = [x for x in s.gcode.raw if "prompt_button" in x]
        assert buttons[0] == (f"// action:prompt_button Replace for 01234567|"
                              f"AFC_BRIDGEBOX_REPLACE CHAIN=chain1 UID={new}|"
                              f"primary")
        assert f"Waiting: {new} [AMS] -- no free bay" in cmd.responses[0]

    def test_the_bay_manager_gives_a_roster_option_no_button(self, tmp_path,
                                                             monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster_option=FOUR)
        _tick(s, 100, 111)
        cmd = _GCmd()
        s.m.cmd_AFC_BRIDGEBOX_BAYS(cmd)
        assert f"Waiting: {E} [AMS] -- no free bay" in cmd.responses[0]
        assert not [x for x in s.gcode.raw if "Replace for" in x]

    def test_assign_without_a_name_points_a_waiting_unit_at_replace(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 101)
        with pytest.raises(Exception, match=f"{E} is on the chain, but every "
                                            f"AMS bay is taken -- "
                                            f"AFC_BRIDGEBOX_REPLACE "
                                            f"CHAIN=chain1 UID={E}"):
            s.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=E))

    def test_assign_names_no_replace_while_every_holder_is_online(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, D, E, H))
        _tick(s, 100, 101)
        with pytest.raises(Exception) as err:
            s.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=E))
        assert str(err.value) == (f"AFC_BRIDGEBOX_ASSIGN: {E} is on the "
                                  f"chain, but every AMS bay is taken")

    def test_assign_says_when_the_chain_has_no_bay_of_the_family(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, roster=f"boxed:{A}",
                   online=(A, G), pool_ams=1, pool_ht=0)
        s.bridge._htmask = 1 << 5                   # G answers as an HT
        _tick(s, 100, 101)
        with pytest.raises(Exception) as err:
            s.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=G))
        assert str(err.value) == (f"AFC_BRIDGEBOX_ASSIGN: {G} is on the "
                                  f"chain, but chain chain1 has no HT bay")

    def test_status_reports_pool_absences_until_release_grace(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 105)
        assert s.m.get_status()["missing"] == {D: 5}
        _tick(s, 106, 110)
        assert s.m.get_status()["missing"] == {}
        assert D in s.m._missing_since              # still counted


# ── FORGET, which REPLACE runs ──────────────────────────────────────────────

class TestForget:
    def test_it_accepts_an_unrecorded_unit_that_holds_a_bay(self, tmp_path,
                                                             monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                   roster=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _tick(s, 100, 101)
        s.bridge._online[5] = False
        _tick(s, 102, 103)
        # A uid neither recorded nor on a bay is still refused.
        with pytest.raises(Exception, match="nothing recorded"):
            s.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="FFFF"))
        cmd = _GCmd(UID=G)
        s.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        bay = _bay(s.m, "Bambu_AMS_4")
        assert (bay["uid"], bay["bound"]) == (None, None)
        assert cmd.responses[0].startswith(f"AFC_BridgeBox chain1: forgot {G}")
        assert "slot freed to the pool LIVE" in cmd.responses[0]

    def test_it_drops_what_the_watch_tracks_for_the_uid(self, tmp_path,
                                                        monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        m = s.m
        m._last_online[D], m._online_run[D] = 1.0, 1.0
        m._no_bay[D] = "boxed"
        m._no_bay_told.add(D)
        m._replace_offered.add(D)
        m._popup_queue = [("removed", D, "Bambu_AMS_4"), ("replace", D),
                          ("new", A)]
        assert D in m._missing_since
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        assert D not in m._missing_since and D not in m._last_online
        assert D not in m._online_run and D not in m._no_bay
        assert D not in m._no_bay_told and D not in m._replace_offered
        assert m._popup_queue == [("new", A)]


def test_the_command_registers_muxed_with_a_default(tmp_path):
    class _GCode:
        def __init__(self):
            self.mux = []

        def register_mux_command(self, cmd, key, value, fn, desc=None):
            if (cmd, key, value) in self.mux:
                raise Exception("already registered")
            self.mux.append((cmd, key, value))

    gcode = _GCode()
    _mk_files(tmp_path, printer=_Printer({"gcode": gcode}))
    assert ("AFC_BRIDGEBOX_REPLACE", "CHAIN", "chain1") in gcode.mux
    assert ("AFC_BRIDGEBOX_REPLACE", "CHAIN", None) in gcode.mux


def test_a_unit_on_another_chain_is_not_on_this_one(tmp_path):
    _master(tmp_path, _Printer(), "chain1", register=False)._state_set({
        "AFC_BridgeBox chain1": {"roster": "ht:AAAA"},
        "AFC_BridgeBox chain2": {"roster": "ht:BBBB"}})
    printer = _Printer()
    m1 = _master(tmp_path, printer, "chain1", roster="")
    m2 = _master(tmp_path, printer, "chain2", roster="",
                 unit_prefix="Bambu_AMS_B")
    assert m2._family_of_uid("BBBB") == "ht"
    with pytest.raises(Exception, match="BBBB is not on chain chain1"):
        m1.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID="BBBB", OLD="AAAA"))


# ── what the new unit gets on the bay ───────────────────────────────────────

@pytest.mark.parametrize("saved", ["released-this-session", "at-last-boot"])
def test_the_new_unit_gets_none_of_the_old_units_data(tmp_path, monkeypatch,
                                                      saved):
    """H's bay Hot (lane32) held its spool 136 on T45, and H had learned its
    bowden length. J, an HT with no bay, replaces H: it gets the home T32,
    AFC's defaults for its untagged spool, and no learned value."""
    H_, J = "HHHH", "JJJJ"
    rec = lr._rec("T45", spool_id=136, material="PLA", color="#0086D6",
                  weight=750.0)
    boot = saved == "at-last-boot"
    ch = lr._chain(tmp_path, roster=f"boxed:{A}, ht:{H_}", pool_ams=2,
                   names="Alpha, Bravo",
                   var={"Hot": {"lane32": rec}} if boot else None,
                   owners=f"{H_}:Hot" if boot else None)
    ch.afc.spoolman = object()
    lane = ch.lanes["lane32"]
    if not boot:
        ch.claim(H_, "ht")
        ch.afc.tool_cmds.pop("T32")
        ch.gcode.register_command("T32", None)
        lane.map, lane.current_map = ["T45"], "T45"
        ch.afc.tool_cmds["T45"] = "lane32"
        ch.gcode.register_command("T45", ch.afc.cmd_CHANGE_TOOL)
        lane.spool_id, lane.material = 136, "PLA"
        lane.color, lane.weight = "#0086D6", 750.0
        ch.m._release_pool_unit(H_)
    assert ch.m._held["Hot"]["uid"] == H_
    assert ch.m._bay_of_uid(H_)["name"] == "Hot"
    ch.m._state_set({ch.m._learned_section(H_):
                     {"afc_bowden_length": "3632.0"}})
    learned = []
    ch.units["Hot"].apply_learned = learned.append
    ch.online(monkeypatch, [A, H_, J], [True, False, True])
    assert ch.claim(J, "ht") is None                # every HT bay is taken
    ch.clock.now = 100.0
    ch.m._missing_since[H_] = 0.0
    ch.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=J, OLD="Hot"))
    assert ch.m._bay_of_uid(J)["bound"] == J
    assert ch.holds("Hot")[-1] == {}
    assert (lane.map, lane.current_map) == (["T32"], "T32")
    assert "T45" not in ch.afc.tool_cmds
    assert learned == [{}] and ch.m._learned_for(H_) == {}
    assert ch.m._owners().get("Hot") == J and "Hot" not in ch.m._held
    unit = ch.units["Hot"]
    unit.bays({"present": True})
    unit._prime_scan_baseline()
    assert lane.spool_id is None and ch.afc.bound == []
    assert unit.finalized == [(0, False, True)]     # AFC's defaults
    lr._consistent(ch.afc)


# ── freeing a bay AFC records a loaded lane on ──────────────────────────────

class TestFreeingALoadedBay:
    """FORGET, UNASSIGN, ASSIGN and the bay manager treat a bay that AFC
    records a lane of as loaded to a toolhead the way REPLACE does, claimed
    or not: the unit that claims the bay next would otherwise take that
    record as its own lane's and start its follower on it."""

    def _loaded(self, tmp_path, monkeypatch, lane="lane36"):
        """D is offline and unclaimed on Bambu_AMS_4, and PREP restored an
        extruder record naming one of its lanes."""
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        saves = []
        ext = types.SimpleNamespace(lane_loaded=lane)
        s.p.objects["AFC"].tools = {"extruder": ext}
        s.p.objects["AFC"].save_vars = lambda: saves.append(ext.lane_loaded)
        return s, ext, saves

    @staticmethod
    def _refused(s, tmp_path, run, cmd):
        before = (_state(tmp_path), [dict(pu) for pu in s.m._pool_units])
        with pytest.raises(Exception) as err:
            run(cmd)
        assert (_state(tmp_path),
                [dict(pu) for pu in s.m._pool_units]) == before
        return str(err.value)

    def test_forget_refuses_and_force_clears_the_record(self, tmp_path,
                                                        monkeypatch):
        s, ext, saves = self._loaded(tmp_path, monkeypatch)
        msg = self._refused(s, tmp_path, s.m.cmd_AFC_BRIDGEBOX_FORGET,
                            _GCmd(UID=D))
        assert msg == (
            f"AFC_BRIDGEBOX_FORGET: AFC records lane36 on Bambu_AMS_4 as "
            f"loaded to the toolhead, from {D}. While Bambu_AMS_4 is "
            f"unclaimed its lanes are not registered, so no unload or "
            f"UNSET_LANE_LOADED reaches them -- plug {D} back in and unload "
            f"it, or take the filament out by hand and FORCE=1 clears the "
            f"record and forgets {D}")
        cmd = _GCmd(UID=D, FORCE=1)
        s.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert ext.lane_loaded is None and saves == [None]
        assert "cleared lane36 from the toolhead" in cmd.responses[0]
        _tick(s, 112, 113)
        assert _bay(s.m, "Bambu_AMS_4")["bound"] == E

    def test_unassign_refuses_and_force_clears_the_record(self, tmp_path,
                                                          monkeypatch):
        s, ext, _saves = self._loaded(tmp_path, monkeypatch)
        msg = self._refused(s, tmp_path, s.m.cmd_AFC_BRIDGEBOX_UNASSIGN,
                            _GCmd(UID=D))
        assert msg.startswith("AFC_BRIDGEBOX_UNASSIGN: AFC records lane36 "
                              "on Bambu_AMS_4 as loaded to the toolhead")
        assert msg.endswith(f"clears the record and unassigns {D}")
        cmd = _GCmd(UID=D, FORCE=1)
        s.m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert ext.lane_loaded is None
        assert "cleared lane36 from the toolhead" in cmd.responses[0]

    def test_assign_off_the_bay_refuses_and_force_clears_the_record(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, H),
                   roster=f"boxed:{A}, boxed:{D}, ht:{H}")
        _tick(s, 100, 111)
        assert _bay(s.m, "Bambu_AMS_2")["uid"] == D
        ext = types.SimpleNamespace(lane_loaded="lane29")
        s.p.objects["AFC"].tools = {"extruder": ext}
        s.p.objects["AFC"].save_vars = lambda: None
        msg = self._refused(s, tmp_path, s.m.cmd_AFC_BRIDGEBOX_ASSIGN,
                            _GCmd(UID=D, NAME="Bambu_AMS_3"))
        assert msg.startswith("AFC_BRIDGEBOX_ASSIGN: AFC records lane29 on "
                              "Bambu_AMS_2 as loaded to the toolhead")
        assert msg.endswith(f"clears the record and moves {D}")
        cmd = _GCmd(UID=D, NAME="Bambu_AMS_3", FORCE=1)
        s.m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert ext.lane_loaded is None
        assert _bay(s.m, "Bambu_AMS_3")["uid"] == D
        assert "cleared lane29 from the toolhead" in cmd.responses[0]

    def test_before_prep_forget_of_an_unclaimed_bay_waits(self, tmp_path,
                                                          monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        s.p.objects["AFC"].prep_done = False
        s.m._ready_at = s.clock.now
        msg = self._refused(s, tmp_path, s.m.cmd_AFC_BRIDGEBOX_FORGET,
                            _GCmd(UID=D))
        assert msg == ("AFC_BRIDGEBOX_FORGET: PREP has not run yet, so which "
                       "lane AFC records as loaded to the toolhead is not "
                       "known -- run this again once it has")
        s.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D, FORCE=1))
        assert _bay(s.m, "Bambu_AMS_4")["uid"] is None

    def test_the_bay_manager_offers_no_unassign_for_it(self, tmp_path,
                                                       monkeypatch):
        s, _ext, _saves = self._loaded(tmp_path, monkeypatch)
        cmd = _GCmd()
        s.m.cmd_AFC_BRIDGEBOX_BAYS(cmd)
        assert (f"Bambu_AMS_4 [AMS]: {D} -- lane36 in the toolhead from {D}, "
                f"which is not claimed: plug it back in and unload it before "
                f"unassigning") in cmd.responses[0]
        assert not [x for x in s.gcode.raw if "Unassign Bambu_AMS_4" in x]
        assert [x for x in s.gcode.raw if "Unassign Bambu_AMS_1" in x]

    def test_the_bay_manager_offers_no_unassign_for_it_before_prep(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        s.p.objects["AFC"].prep_done = False
        s.m._ready_at = s.clock.now
        cmd = _GCmd()
        s.m.cmd_AFC_BRIDGEBOX_BAYS(cmd)
        assert (f"Bambu_AMS_4 [AMS]: {D} -- PREP has not run yet"
                in cmd.responses[0])
        assert not [x for x in s.gcode.raw if "Unassign Bambu_AMS_4" in x]

    def test_the_no_bay_line_says_what_the_forget_needs_first(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, H))
        _tick(s, 100, 111)
        s.p.objects["AFC"].tools = {
            "extruder": types.SimpleNamespace(lane_loaded="lane36")}
        s.bridge._online[4] = True                  # E is plugged in now
        _tick(s, 112, 113)
        (msg,) = _no_bay_lines(s)
        assert (f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={D} (AFC records "
                f"lane36 as loaded to the toolhead: plug it back in and "
                f"unload it, or take the filament out by hand and add "
                f"FORCE=1, which clears the record) frees that bay") in msg


class TestTheLoadedRefusalComesFirst:
    """The loaded-lane refusal names the lane FORCE=1 clears, so it comes
    before the print and online refusals FORCE=1 also overrides."""

    def _bound(self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, D, H))
        _tick(s, 100, 101)
        lane = s.p.objects["AFC_lane lane37"]
        lane.unassigned, lane.tool_loaded = False, True
        return s

    def test_forget_mid_print_names_the_lane_and_the_print(self, tmp_path,
                                                           monkeypatch):
        s = self._bound(tmp_path, monkeypatch)
        s.p.objects["print_stats"] = _PrintStats("printing")
        with pytest.raises(Exception) as err:
            s.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        assert str(err.value) == (
            f"AFC_BRIDGEBOX_FORGET: AFC records lane37 on {D} as loaded to "
            f"the toolhead -- unload it first (UNSET_LANE_LOADED if the "
            f"filament is already out), or FORCE=1 to clear it from the "
            f"toolhead and release anyway (a print is active, and FORCE=1 "
            f"also pulls the lane out from under it)")

    def test_unassign_of_an_online_unit_names_the_lane(self, tmp_path,
                                                       monkeypatch):
        s = self._bound(tmp_path, monkeypatch)
        with pytest.raises(Exception) as err:
            s.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=D))
        assert str(err.value).startswith(
            f"AFC_BRIDGEBOX_UNASSIGN: AFC records lane37 on {D} as loaded to "
            f"the toolhead")

    def test_the_unset_hint_names_a_toolhead_that_is_not_active(
            self, tmp_path, monkeypatch):
        s = self._bound(tmp_path, monkeypatch)
        afc = s.p.objects["AFC"]
        afc.tools = {"extruder": types.SimpleNamespace(lane_loaded="lane1"),
                     "extruder1": types.SimpleNamespace(
                         lane_loaded="lane37")}
        afc.function = types.SimpleNamespace(
            get_current_extruder=lambda: "extruder")
        with pytest.raises(Exception) as err:
            s.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        assert ("unload it first (UNSET_LANE_LOADED with extruder1 as the "
                "active tool if the filament is already out)"
                in str(err.value))
        afc.function = types.SimpleNamespace(
            get_current_extruder=lambda: "extruder1")
        with pytest.raises(Exception) as err:
            s.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        assert "unload it first (UNSET_LANE_LOADED if" in str(err.value)


class TestTheForceHint:
    """REPLACE's reply offers FORCE=1 as skipping the wait only when it
    would override nothing else."""

    def test_a_recorded_ams_the_cap_left_waiting_is_told_why(
            self, tmp_path, monkeypatch):
        # E ran on a fifth bay under an earlier start; a bus addresses four.
        maps = dict(name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                              f"{C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
                              f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
                    lane_map=(f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, "
                              f"{E}:40:4, {H}:44:1"))
        s = _stuck(tmp_path, monkeypatch, roster=FOUR + f", boxed:{E}",
                   maps=maps)
        assert s.m._unbayed == {E: "Bambu_AMS_5"}
        _tick(s, 100, 115)
        raw = "".join(s.gcode.raw)
        assert f"action:prompt_begin No bay for AMS {E}" in raw
        assert (f"UID {E} is recorded but has no bay: every AMS bay is "
                f"taken. A Bambu bus addresses at most 4 AMS, and its saved "
                f"bay Bambu_AMS_5 is not one of the 4 AMS bays.") in raw

    def test_during_a_print_it_offers_no_force(self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 103)
        s.p.objects["print_stats"] = _PrintStats("printing")
        cmd = _GCmd(UID=E)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert cmd.responses == [
            f"AFC_BridgeBox chain1: no AMS bay is held for a unit offline "
            f"10s or longer: Bambu_AMS_4 ({D}, offline 3s) -- a print is "
            f"active: run this again once it ends."]

    def test_with_a_loaded_lane_it_says_force_clears_it(self, tmp_path,
                                                        monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 103)
        s.p.objects["AFC"].tools = {
            "extruder": types.SimpleNamespace(lane_loaded="lane36")}
        cmd = _GCmd(UID=E)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(cmd)
        assert cmd.responses[0].endswith(
            f"Bambu_AMS_4 ({D}, offline 3s) -- AFC records lane36 as loaded "
            f"to the toolhead: plug {D} back in and unload it, or take the "
            f"filament out by hand and AFC_BRIDGEBOX_REPLACE CHAIN=chain1 "
            f"UID={E} OLD=Bambu_AMS_4 FORCE=1 clears the record and skips "
            f"the wait.")

    def test_the_picker_offers_no_force_during_a_print(self, tmp_path,
                                                       monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        s.p.objects["AFC"].tools = {
            "extruder": types.SimpleNamespace(lane_loaded="lane36")}
        s.p.objects["print_stats"] = _PrintStats("printing")
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E))
        raw = "".join(s.gcode.raw)
        assert ("AFC records lane36 as loaded to the toolhead, and a print "
                "is active: replace it once the print ends") in raw
        assert "FORCE=1" not in raw


class TestRepliesWithoutAPool:
    """Without a pool nothing claims a bay live, so no reply says one
    will."""

    def test_forget_says_nothing_claims_the_freed_bay(self, tmp_path,
                                                      monkeypatch):
        s = _stuck(tmp_path, monkeypatch, pool_ams=0, pool_ht=0)
        cmd = _GCmd(UID=D)
        s.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.responses[0].endswith(
            "-- its bay is free now, but with no pool nothing claims it: "
            "AFC_BRIDGEBOX_ASSIGN another unit onto it while that unit is "
            "online, or RESTART.")
        assert "LIVE" not in cmd.responses[0]
        assert "regularise" not in cmd.responses[0]

    def test_assign_of_an_offline_unit_says_to_run_it_again(self, tmp_path,
                                                            monkeypatch):
        s = _stuck(tmp_path, monkeypatch, pool_ams=0, pool_ht=0)
        s.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        cmd = _GCmd(UID=G, NAME="Bambu_AMS_4")
        s.m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert cmd.responses == [
            f"AFC_BridgeBox chain1: assigned {G} to bay 'Bambu_AMS_4' "
            f"(lane36-lane39, T36-T39) -- pinned; with no pool nothing claims "
            f"it, so run this again once {G} is online and PREP has "
            f"finished, and after every restart."]

    def test_with_a_pool_forget_names_no_restart_regularising(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        cmd = _GCmd(UID=D)
        s.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.responses[0].endswith(
            "-- slot freed to the pool LIVE; the next same-family unit "
            "claims it with no reboot.")


class TestSmallTexts:
    def test_forget_accepts_a_released_unit_only_its_bay_names(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch, online=(A, B, C, G, H),
                   roster=f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        _tick(s, 100, 101)
        bay = _bay(s.m, "Bambu_AMS_4")
        assert bay["bound"] == G
        s.bridge._online[5] = False                 # G is pulled
        s.m._release_pool_unit(G)
        bay["uid"] = None
        assert s.m._owners().get("Bambu_AMS_4") == G
        s.m._prompt_removed_unit(G, "Bambu_AMS_4")
        raw = "".join(s.gcode.raw)
        assert "Or forget it if it is not coming back:" in raw
        cmd = _GCmd(UID=G)
        s.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.responses[0] == f"AFC_BridgeBox chain1: forgot {G}."
        assert "Bambu_AMS_4" not in s.m._owners()

    def test_a_unit_on_a_built_bay_is_promised_no_temperature_card(
            self, tmp_path, monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        s.m.cmd_AFC_BRIDGEBOX_REPLACE(_GCmd(UID=E, OLD=D))
        del s.m.logger.lines[:]
        s.m._announce_enrolled([f"boxed:{E}"])
        (line,) = s.m.logger.lines
        assert "temperature card" not in line

    def test_the_picker_of_a_reserved_offline_unit_says_so(self, tmp_path,
                                                           monkeypatch):
        s = _stuck(tmp_path, monkeypatch)
        _tick(s, 100, 111)
        s.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=D))
        raw = "".join(s.gcode.raw)
        assert (f"UID {D} is on 'Bambu_AMS_4' (reserved for it; its T# and "
                f"lanes are not live yet).") in raw
        s.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=A))
        assert (f"UID {A} is on 'Bambu_AMS_1' (its T# and lanes are live)."
                in "".join(s.gcode.raw))

    def test_the_help_texts_match_the_commands(self, tmp_path):
        class _GCode:
            def __init__(self):
                self.desc = {}

            def register_mux_command(self, cmd, key, value, fn, desc=None):
                self.desc[cmd] = desc

        gcode = _GCode()
        _mk_files(tmp_path, printer=_Printer({"gcode": gcode}))
        assert gcode.desc["AFC_BRIDGEBOX_BAYS"].startswith(
            "Pop the bay manager: every pool bay, its occupant, an Unassign "
            "button for each occupied bay, and a Replace button for a unit "
            "waiting for a bay")
        assert gcode.desc["AFC_BRIDGEBOX_FORGET"].startswith(
            "Release a departed unit's lane numbers and unit name for reuse "
            "and erase its learned values and saved lane records")
        assert "NAME=<bay name> [FORCE=1]" in gcode.desc[
            "AFC_BRIDGEBOX_ASSIGN"]
        assert "forget" not in gcode.desc["AFC_BRIDGEBOX_BAYS"]
