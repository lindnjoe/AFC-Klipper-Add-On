"""Which unit a pool bay's saved records belong to, across restarts, pin
changes and FORGET; lanes a bay moved to; and what a claim says and saves.

A state from before bay_owner names no owner, so the first start guesses one
per bay (see afcBridgeBox._capture_boot_records). The guess is recorded as
the bay's owner and written, so a later start neither loses the records nor
hands them to whichever unit its own pins then put on the bay.
"""
from __future__ import annotations

import json
import types

import pytest

from extras import AFC_BridgeBox as bb
from tests import test_AFC_bridgebox_lane_records as records
from tests import test_AFC_bridgebox_two_chains as two_chains
from tests.test_AFC_BridgeBox import _GCmd
from tests.test_AFC_bridgebox_lane_records import (
    A, C, D, H, SEC, _Afc, _Log, _chain, _consistent, _rec)

VAR = {"Alpha": {"lane24": _rec("T24", spool_id=159, material="PLA",
                                color="#0086D6", weight=412.0),
                 "lane25": _rec("T25", material="PETG")}}


def _saves(ch):
    """AFC's own save_vars and write queue on the chain's AFC."""
    return records.TestTheVarFileKeepsHeldRecords._afc_saves(None, ch)


def _drain(ch, saved):
    """
    :return: every snapshot queued since the last read, in order
    """
    out = []
    while not ch.afc._var_write_queue.empty():
        out.append(saved())
    return out


def _upgrade(tmp_path, **kw):
    """
    The first start after an upgrade: AAAA is pinned to Alpha, the var file
    holds Alpha's records, and the state has no bay_owner key.
    """
    pins = {k: v for k, v in kw.items() if k == "option"}
    _chain(tmp_path, ready=False, **pins)                 # pins AAAA:Alpha
    ch = _chain(tmp_path, var=VAR, owners=None, ready=False, **kw)
    saved = _saves(ch)
    ch.m._scout_ready()
    return ch, saved


class TestTheUpgradeGuessIsRecorded:
    def test_the_guess_is_the_owner_and_is_written_once_prep_has_run(
            self, tmp_path):
        ch, saved = _upgrade(tmp_path)
        assert ch.m._held["Alpha"]["uid"] == A
        assert ch.m._owners() == {"Alpha": A}
        assert ch.m._state_get(SEC, "bay_owner") is None
        ch.afc.prep_done = False
        ch.m._persist_bay_owner()
        assert ch.m._state_get(SEC, "bay_owner") is None
        ch.afc.prep_done = True
        ch.m._persist_bay_owner()                   # the tick after PREP
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert ch.m._bay_owner_pending is False

    def test_another_claim_writes_the_guess_with_it(self, tmp_path):
        # AAAA stays offline all session while CCCC claims a spare; the
        # restart holds AAAA's records again, and AAAA gets them.
        ch, saved = _upgrade(tmp_path)
        ch.afc.save_vars()                          # PREP's first save
        assert saved()["Alpha"] == VAR["Alpha"]
        assert ch.claim(C) is ch.units["Bravo"]
        _drain(ch, saved)
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha, CCCC:Bravo"
        ch.afc.save_vars()
        assert _drain(ch, saved)[-1]["Alpha"] == VAR["Alpha"]
        again = _chain(tmp_path)
        assert again.m._held["Alpha"] == {"uid": A, "lanes": VAR["Alpha"]}
        again.claim(A)
        assert again.lanes["lane24"].spool_id == 159

    def test_the_floater_takes_the_spare_its_records_are_on(self, tmp_path):
        # roster: leaves CCCC out; the recorded roster lists it, and the
        # records on the second spare are guessed for it.
        _chain(tmp_path, ready=False)                     # pins AAAA:Alpha
        var = {"Charlie": {"lane32": _rec("T32", spool_id=7,
                                          material="PETG")}}
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:CCCC", var=var,
                    option="boxed:AAAA")
        assert ch.m._held["Charlie"]["uid"] == C
        assert ch.claim(C) is ch.units["Charlie"]
        assert ch.lanes["lane32"].map == ["T32"]
        assert ch.m._held["Charlie"]["uid"] == C


class TestAGuessIsNotMadeAgain:
    @pytest.mark.parametrize("ticked", [True, False])
    def test_unassign_and_assign_give_the_new_unit_nothing(self, tmp_path,
                                                           ticked):
        # AAAA (offline) is swapped for CCCC before CCCC is plugged in, then
        # the printer restarts: CCCC is pinned to Alpha now, and a guess from
        # that pin would hand it AAAA's Spoolman spool.
        ch, saved = _upgrade(tmp_path)
        ch.afc.save_vars()                          # PREP's save
        saved()
        if ticked:
            ch.m._persist_bay_owner()               # the tick after PREP
        ch.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=A))
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=C, NAME="Alpha"))
        ch.afc.save_vars()
        assert saved()["Alpha"] == VAR["Alpha"]
        again = _chain(tmp_path, roster="boxed:AAAA, boxed:CCCC")
        assert again.m._pins_at_boot.get(C) == "Alpha"
        assert {b: e["uid"] for b, e in again.m._held.items()} == {
            "Alpha": A}
        again.claim(C)
        lane = again.lanes["lane24"]
        assert (lane.spool_id, lane.material, lane.weight) == (None, "", 0)
        assert lane.map == ["T24"]

    def test_a_pin_changed_before_prep_writes_what_the_file_holds(
            self, tmp_path):
        # Before PREP, AFC.var.unit holds what this start held for each bay;
        # the owner a claim before PREP records is left for the tick after.
        ch, _saved = _upgrade(tmp_path)
        ch.afc.prep_done = False
        ch.m._set_bay_owner("Bravo", D)
        ch.m._drop_pin(A)
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert ch.m._bay_owner_pending is True
        ch.afc.prep_done = True
        ch.m._persist_bay_owner()
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha, DDDD:Bravo"

    @pytest.mark.parametrize("ticked", [True, False])
    def test_forget_erases_the_records_it_says_it_erased(self, tmp_path,
                                                         ticked):
        # REPLACE's refusal with roster: set says: FORGET the old unit, list
        # the new one in roster:, RESTART.
        ch, saved = _upgrade(tmp_path, option="boxed:AAAA")
        ch.afc.save_vars()                          # PREP's save
        assert saved()["Alpha"] == VAR["Alpha"]
        if ticked:
            ch.m._persist_bay_owner()               # the tick after PREP
        cmd = _GCmd(UID=A)
        ch.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert "saved lane records erased" in cmd.responses[0]
        assert "Alpha" not in ch.m._held and ch.m._owners() == {}
        assert _drain(ch, saved)[-1]["Alpha"] == {}   # FORGET saves
        assert ch.m._state_get(SEC, "bay_owner") == ""
        # Were the file still holding them, the next start must not guess
        # them for the unit drawing Alpha.
        ch.write_var(VAR)
        again = _chain(tmp_path, option="boxed:CCCC", roster="boxed:CCCC")
        assert again.bay("Alpha")["uid"] == C
        assert again.m._held == {}
        again.claim(C)
        assert again.lanes["lane24"].spool_id is None


class TestAMovedBayKeepsItsRecords:
    def _boot(self, tmp_path, rec):
        seed = bb.afcBridgeBox.__new__(bb.afcBridgeBox)
        seed.state_file = str(tmp_path / "AFC_BridgeBox.cfg")
        seed._state_set({"AFC_BridgeBox chain1": {
            "bay_owner": "HHHH:Bambu_AMS_HT_1"}})
        (tmp_path / "AFC.var.unit").write_text(json.dumps({
            "Bambu_AMS_HT_1": {"lane40": rec}}))
        ht = two_chains.TestAHeldHtBandMakesWayForALaterChain()
        m1, _m2, notes = ht._boot(tmp_path, lane_base=36)
        assert "cannot keep its lanes" in notes[0]
        pu = next(p for p in m1._pool_units
                  if p["name"] == "Bambu_AMS_HT_1")
        assert pu["lanes"] == ["lane28"] and pu["uid"] == H
        m1.printer.objects["AFC"] = _Afc(tmp_path)
        m1.logger = _Log()
        m1._capture_boot_records()
        return m1

    def test_an_ht_moved_for_a_later_chain_keeps_its_record(self, tmp_path):
        rec = dict(_rec("T40", spool_id=55, material="PLA", weight=300.0),
                   name="lane40", tool_loaded=True)
        m1 = self._boot(tmp_path, rec)
        held = m1._held["Bambu_AMS_HT_1"]
        assert held["uid"] == H
        # Its old home T# is not its T# now; the rest of the record is.
        assert held["lanes"] == {"lane28": {
            "name": "lane28", "spool_id": 55, "material": "PLA",
            "weight": 300.0, "tool_loaded": False}}

    def test_a_map_set_by_hand_moves_with_it(self, tmp_path):
        m1 = self._boot(tmp_path, _rec("T3", spool_id=55))
        assert m1._held["Bambu_AMS_HT_1"]["lanes"]["lane28"]["map"] == "T3"

    def test_records_that_do_not_fit_the_bay_are_not_moved(self):
        pu = {"lanes": ["lane24", "lane25", "lane26", "lane27"]}
        saved = {"lane28": _rec("T28"), "lane29": _rec("T29")}
        assert bb.afcBridgeBox._moved_bay_records(pu, saved) == {}
        saved = {"lane24": _rec("T24"), "lane29": _rec("T29"),
                 "lane30": {}, "lane31": {}}
        assert bb.afcBridgeBox._moved_bay_records(pu, saved) == {}


class TestTheClaimWarningForAToolheadLane:
    def _two_tools(self, ch):
        ch.afc.tools = {
            "extruder": types.SimpleNamespace(name="extruder",
                                              lane_loaded="lane5"),
            "extruder1": types.SimpleNamespace(name="extruder1",
                                               lane_loaded="lane28")}
        ch.afc.function.get_current_extruder = lambda: "extruder"
        ch.afc.function.get_current_lane = lambda: "lane5"

    def test_no_record_attributed_says_it_cannot_tell(self, tmp_path):
        ch = _chain(tmp_path)
        self._two_tools(ch)
        assert ch.claim(C) is ch.units["Bravo"]
        assert ch.log.warnings == [
            "AFC bambu Bravo: extruder1 records lane28 as loaded, but no "
            "saved record of lane28 is attributed to this unit (CCCC), so "
            "AFC cannot tell which unit the filament in extruder1 is from, "
            "and lane28 is left unloaded. Unload that filament by hand and "
            "run UNSET_LANE_LOADED with extruder1 as the active tool to "
            "clear the record."]
        assert ch.lanes["lane28"].tool_loaded is False

    def test_another_units_records_say_it_is_not_this_units(self, tmp_path):
        ch = _chain(tmp_path, var={"Bravo": {"lane28": _rec(
            "T28", material="PLA", tool_loaded=True)}}, owners="DDDD:Bravo")
        self._two_tools(ch)
        assert ch.m._held["Bravo"]["uid"] == D
        assert ch.claim(C) is ch.units["Bravo"]
        assert ch.log.warnings == [
            "AFC bambu Bravo: extruder1 records lane28 as loaded, but lane28 "
            "was not saved loaded under this unit (CCCC), so the filament "
            "in extruder1 is not from this unit and lane28 is left "
            "unloaded. Unload that filament by hand and run "
            "UNSET_LANE_LOADED with extruder1 as the active tool to clear "
            "the record."]


class TestTheBayManagerDuringAPrint:
    def test_a_live_unit_gets_no_unassign_button(self, tmp_path,
                                                 monkeypatch):
        ch = _chain(tmp_path)
        ch.claim(A)
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=C, NAME="Bravo"))
        ch.online(monkeypatch, [A], [True])
        state = ["printing"]
        ch.printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": state[0]})
        shown = []
        monkeypatch.setattr(ch.m, "_prompt",
                            lambda title, lines, buttons, **k:
                            shown.append((lines, buttons)))
        ch.m.cmd_AFC_BRIDGEBOX_BAYS(_GCmd())
        lines, buttons = shown[-1]
        assert lines[0] == ("Alpha [AMS]: AAAA (live) -- a print is "
                            "active, unassign it once it ends")
        # An unclaimed bay's pin drops no lane or T#.
        assert [b[0] for b in buttons] == ["Unassign Bravo"]
        state[0] = "complete"
        ch.m.cmd_AFC_BRIDGEBOX_BAYS(_GCmd())
        assert [b[0] for b in shown[-1][1]] == ["Unassign Alpha",
                                                "Unassign Bravo"]


class TestAHomeMoveDuringAPrint:
    """The home T# wins between Bambu lanes, in AFC.log only, also when the
    claim that takes it comes during a print and waits for its end."""

    def test_the_wait_and_the_take_go_to_afc_log_only(self, tmp_path):
        ch = _chain(tmp_path, var={"Bravo": {"lane28": _rec("T24")}},
                    owners="CCCC:Bravo")
        assert ch.claim(C) is ch.units["Bravo"]
        lane28 = ch.lanes["lane28"]
        assert lane28.map == ["T24"]
        state = ["printing"]
        ch.printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": state[0]})
        del ch.log.lines[:]
        assert ch.claim(A) is ch.units["Alpha"]
        lane24 = ch.lanes["lane24"]
        assert (lane24.map, lane28.map) == (["NONE"], ["T24"])
        state[0] = "complete"
        ch.m._take_deferred_tools()
        assert (lane24.map, lane28.map) == (["T24"], ["T28"])
        assert ch.log.warnings == []
        assert [x for x in ch.log.lines if "T24" in x] == []
        said = [x for x in ch.log.debugs if "T24" in x]
        assert any("lane24 takes T24 from lane28 once the print ends" in x
                   for x in said)
        assert ("AFC_BridgeBox chain1: the printer is idle, so the T#s the "
                "claim left in use are taken: lane24 is T24.") in said
        _consistent(ch.afc)

    @pytest.mark.parametrize("maps,after,quiet", [
        ("T24, T28", ["T28"], True),
        ("T24, T28, T40", ["T28", "T40"], True),
        ("T24, T40", ["T40"], False)])
    def test_a_holder_mapped_to_more_t_numbers(self, tmp_path, maps, after,
                                               quiet):
        # A holder that keeps its own home T# ends where a claim with no
        # print running leaves it; one that keeps only another T# does not.
        ch = _chain(tmp_path, var={"Bravo": {"lane28": _rec(maps)}},
                    owners="CCCC:Bravo")
        assert ch.claim(C) is ch.units["Bravo"]
        lane28 = ch.lanes["lane28"]
        state = ["printing"]
        ch.printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": state[0]})
        del ch.log.lines[:]
        assert ch.claim(A) is ch.units["Alpha"]
        lane24 = ch.lanes["lane24"]
        assert lane24.map == ["NONE"]
        state[0] = "complete"
        ch.m._take_deferred_tools()
        assert (lane24.map, lane28.map) == (["T24"], after)
        said = [x for x in ch.log.lines if "lane24 takes T24" in x
                or "the claim left in use" in x]
        assert (said == []) is quiet
        _consistent(ch.afc)


class TestTheClaimsOwnSaves:
    def test_no_save_has_a_lane_waiting_for_its_map_on_none(self,
                                                             tmp_path):
        # A restart between two of the claim's saves restores what the last
        # one written says.
        var = {"Alpha": {"lane24": _rec("T3"), "lane25": _rec("T25"),
                         "lane26": _rec("NONE", current="")}}
        ch = _chain(tmp_path, var=var, owners="AAAA:Alpha", ready=False)
        saved = _saves(ch)
        ch.m._scout_ready()
        ch.claim(A)
        snaps = [d["Alpha"] for d in _drain(ch, saved) if d.get("Alpha")]
        assert len(snaps) > 2
        for snap in snaps:
            assert {ln: r["map"] for ln, r in snap.items()} == {
                "lane24": "T3", "lane25": "T25", "lane26": "NONE",
                "lane27": "T27"}
        assert ch.m._claim_plans == {}
