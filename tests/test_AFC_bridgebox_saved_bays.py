"""A live unit's bay is saved once it is recorded; learned values follow its UID.

A unit claimed live onto a spare pool bay is pinned there (name and lanes)
once it is recorded and has been online for enroll_grace, and the bay is held
for it from then on. Without the pin the restart gives it whatever name it
draws, so a FORGET that frees a lower bay, or two units claimed in one tick,
would move it to other lanes and T#.

What a unit learns about itself -- the bowden lengths its AMS measures -- is
saved under its UID, not under the bay it sat on, so another unit claiming
that bay never inherits it and the unit takes it to whatever bay it claims.
"""
from __future__ import annotations

import types

import pytest

from extras import AFC_BambuAMS_bridge as bridge_mod
from extras.AFC_BridgeBox import afcBridgeBox
from tests.test_AFC_BridgeBox import (_Bridge, _CapGCode, _Config,
                                      _FakeReactor, _FileConfig, _GCmd,
                                      _Logger, _mk_files, _Printer)

SEC = "AFC_BridgeBox chain1"
POOL = dict(pool_ams=3, pool_ht=1)


def _seed_state(path, updates):
    """Write ``updates`` into the managed block of the state file at ``path``,
    as an earlier session would have left it."""
    m = afcBridgeBox.__new__(afcBridgeBox)
    m.state_file = str(path)
    m._state_set(updates)


class _PoolUnit:
    """A fabricated pool bay as the master drives it: claim() binds a uid,
    release() frees it, apply_learned() records what the claim handed over."""

    def __init__(self, name):
        self.name = name
        self.pool = True
        self.unit_uid = None
        self.ams_model = "boxed"
        self.has_heater = False
        self.dry_max_temp = 65
        self.measure_on_insert = False
        self.claim_ok = True
        self.learned = []

    def set_master(self, master):
        self.master = master

    def claim(self, uid, model):
        if not self.claim_ok:
            return False
        self.unit_uid, self.ams_model, self.pool = uid, model, False
        return True

    def release(self):
        self.pool, self.unit_uid = True, None

    def apply_learned(self, values):
        self.learned.append(dict(values))


def _live_pool(tmp_path, monkeypatch, bridge, file_roster=None, **over):
    """A pooled chain whose bays are _PoolUnits, watching ``bridge``.

    With ``file_roster`` the roster is the recorded one (no roster: option),
    seeded into the state file; otherwise ``roster`` passes through, so
    roster="boxed:AAAA" is the option. Bays: lane_base 24, three AMS
    (24-35) and one HT (36) unless overridden.
    """
    printer = _Printer({"gcode": _CapGCode()})
    printer.get_reactor = lambda: _FakeReactor()
    opts = dict(POOL)
    opts.update(over)
    if file_roster is not None:
        _seed_state(tmp_path / "AFC_BridgeBox.cfg",
                    {SEC: {"roster": file_roster}})
        opts["roster"] = ""
    m, printer, _a, _s = _mk_files(tmp_path, printer=printer, **opts)
    m.logger = _Logger()
    for name in [n for n in printer.objects if n.startswith("AFC_lane ")]:
        del printer.objects[name]
    printer.objects["AFC"] = types.SimpleNamespace(tool_cmds={})
    units = {}
    for pu in m._pool_units:
        units[pu["name"]] = _PoolUnit(pu["name"])
        printer.objects[f"AFC_BambuAMS {pu['name']}"] = units[pu["name"]]
    monkeypatch.setattr(bridge_mod, "_BRIDGES", {m.serial_port: bridge},
                        raising=False)
    m._test_gcode = printer.objects["gcode"]
    return m, units


def _tick(m, bridge, t, online=None):
    if online is not None:
        bridge._online = list(online)
    m._scout_tick(t)


def _bay(m, name):
    return next(p for p in m._pool_units if p["name"] == name)


def _saved_lines(m):
    return [x for x in m.logger.lines if ": saved " in x]


def _new_popups(m):
    return [x for x in m._test_gcode.raw if "prompt_begin New AMS" in x]


def _prompt_texts(m):
    return [x.split("prompt_text ", 1)[1] for x in m._test_gcode.raw
            if "action:prompt_text " in x]


def _force_claim_order(monkeypatch, first):
    """Make the watch tick meet CCCC before ZZZZ (``first`` "new") or ZZZZ
    before CCCC ("returning") in the set of units to claim, by handing any
    set the module sorts to sorted() in that order."""
    import builtins
    from extras import AFC_BridgeBox as bb_mod
    real = builtins.sorted
    order = ["CCCC", "ZZZZ"] if first == "new" else ["ZZZZ", "CCCC"]

    def _forced(it, *a, **k):
        if isinstance(it, (set, frozenset)):
            it = [u for u in order if u in it] + real(
                u for u in it if u not in order)
        return real(it, *a, **k)
    monkeypatch.setattr(bb_mod, "sorted", _forced, raising=False)


def _reboot(tmp_path, **over):
    """The next boot, in file mode, as the state file now stands."""
    opts = dict(POOL)
    opts.update(over)
    return _mk_files(tmp_path, roster="", **opts)


def _loaded(printer, section):
    """The keys a section was fabricated with."""
    wrapper = dict(printer.loaded)[section]
    return dict(wrapper.fileconfig.items(section))


def _state_names(m):
    raw = m._state_get(SEC, "name_map") or ""
    return dict(e.strip().split(":", 1) for e in raw.split(",") if e.strip())


# ── A: a recorded unit's bay is saved, so a restart never moves it ───────────

class TestABaySavedWhenRecorded:

    def _cccc_saved(self, tmp_path, monkeypatch, **over):
        """CCCC plugged into a chain that records boxed:AAAA (offline): it
        claims the Bambu_AMS_2 spare at once and is recorded 16s later."""
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA", **over)
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        return m, units, bridge

    def test_a_new_unit_is_saved_on_the_bay_it_claimed(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert "CCCC" not in m._name_map          # not recorded yet
        _tick(m, bridge, 116.0)
        assert m._name_map["CCCC"] == "Bambu_AMS_2"
        assert m._lane_map["CCCC"] == (28, 4)
        assert _state_names(m)["CCCC"] == "Bambu_AMS_2"
        assert "CCCC:28:4" in m._state_get(SEC, "lane_map")
        assert m._bay_held(_bay(m, "Bambu_AMS_2"))
        _tick(m, bridge, 130.0)
        assert _saved_lines(m) == [
            "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_2 (lane28-lane31, "
            "T28-T31); it comes back there after a restart."]
        # The enrollment line comes first and promises nothing the save
        # line after it does not show.
        new = [x for x in m.logger.lines if "NEW unit(s)" in x]
        assert new == ["AFC_BridgeBox chain1: NEW unit(s) on the chain: "
                       "boxed:CCCC -- recorded. A restart adds its "
                       "temperature card."]
        assert m.logger.lines.index(new[0]) < \
            m.logger.lines.index(_saved_lines(m)[0])
        assert "It is saved on this bay once it has been online 15s." in \
            _prompt_texts(m)

    def test_a_restart_keeps_it_after_a_lower_bay_is_freed(
            self, tmp_path, monkeypatch):
        # FORGET frees Bambu_AMS_1. A unit with no saved name would draw it
        # at the restart and come up on lane24/T24 instead of lane28/T28.
        m, _u, _b = self._cccc_saved(tmp_path, monkeypatch)
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        m2, p2, _a, _s = _reboot(tmp_path)
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_2")["unit_uid"] == "CCCC"
        assert _loaded(p2, "AFC_lane lane28")["unit"] == "Bambu_AMS_2:1"
        assert "unit_uid" not in _loaded(p2, "AFC_BambuAMS Bambu_AMS_1")

    def test_two_units_claimed_on_one_tick_keep_their_bays(
            self, tmp_path, monkeypatch):
        # Two new units claimed on one tick: whichever takes the lower bay,
        # the restart must match the live bays.
        bridge = _Bridge(uids=["AAAA", "CCCC", "EEEE"],
                         online=[False, True, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        live = {pu["bound"]: pu["name"] for pu in m._pool_units
                if pu["bound"]}
        assert set(live) == {"CCCC", "EEEE"}
        assert {u: m._name_map[u] for u in live} == live     # saved live
        assert any("boxed:CCCC, boxed:EEEE -- recorded. A restart adds their "
                   "temperature cards." in x for x in m.logger.lines)
        _m2, p2, _a, _s = _reboot(tmp_path)
        for uid, name in live.items():
            assert _loaded(p2, f"AFC_BambuAMS {name}")["unit_uid"] == uid

    def test_units_keep_their_bays_whatever_order_they_are_recorded_in(
            self, tmp_path, monkeypatch):
        # CCCC claims first but blips offline, so EEEE is recorded first. A
        # restart drawing names in roster order would swap their bays.
        bridge = _Bridge(uids=["AAAA", "CCCC", "EEEE"],
                         online=[False, True, False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)                      # CCCC -> Bambu_AMS_2
        _tick(m, bridge, 101.0, [False, False, True])  # EEEE -> Bambu_AMS_3
        for t in range(102, 120):
            _tick(m, bridge, float(t), [False, True, True])
        assert m._state_get(SEC, "roster") == \
            "boxed:AAAA, boxed:EEEE, boxed:CCCC"
        assert (_bay(m, "Bambu_AMS_2")["bound"],
                _bay(m, "Bambu_AMS_3")["bound"]) == ("CCCC", "EEEE")
        _m2, p2, _a, _s = _reboot(tmp_path)
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_2")["unit_uid"] == "CCCC"
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_3")["unit_uid"] == "EEEE"

    def test_a_unit_pulled_before_the_grace_is_saved_only_once_it_holds(
            self, tmp_path, monkeypatch):
        # Pulled inside enroll_grace: nothing is saved and the bay goes back
        # to the pool. Plugged back and held online, it is saved on the bay
        # it claims then.
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", auto_drop=True,
                               release_grace=2.0)
        _tick(m, bridge, 100.0)
        for t in (101.0, 102.0, 103.0, 104.0):
            _tick(m, bridge, t, [False, False])
        bay = _bay(m, "Bambu_AMS_2")
        assert bay["bound"] is None and bay["uid"] is None
        assert "CCCC" not in m._name_map and _saved_lines(m) == []
        assert not any("slot kept" in x for x in m.logger.lines)
        for t in range(105, 121):                   # back, past the graces
            _tick(m, bridge, float(t), [False, True])
        assert bay["bound"] == "CCCC"
        assert m._name_map["CCCC"] == "Bambu_AMS_2"
        assert len(_saved_lines(m)) == 1

    def test_a_saved_bay_is_held_across_an_auto_drop(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC", "EEEE"],
                         online=[False, True, False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", auto_drop=True)
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)                     # CCCC saved on _2
        for t in range(117, 130):
            _tick(m, bridge, float(t), [False, False, False])
        bay = _bay(m, "Bambu_AMS_2")
        assert bay["bound"] is None and bay["uid"] == "CCCC"   # held
        assert any("released Bambu_AMS_2" in x and "slot kept for re-plug"
                   in x for x in m.logger.lines)
        _tick(m, bridge, 131.0, [False, False, True])   # a new unit
        assert _bay(m, "Bambu_AMS_3")["bound"] == "EEEE"
        for t in range(132, 150):                   # CCCC re-plugged
            _tick(m, bridge, float(t), [False, True, True])
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        popups = _new_popups(m)
        assert popups == ["// action:prompt_begin New AMS on Bambu_AMS_2",
                          "// action:prompt_begin New AMS on Bambu_AMS_3"]

    def test_enroll_before_claim_pins_on_the_claim_tick(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", claim_grace=30.0)
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        assert "boxed:CCCC" in m._state_get(SEC, "roster")    # recorded
        assert "CCCC" not in m._name_map                      # no bay yet
        _tick(m, bridge, 130.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert m._name_map["CCCC"] == "Bambu_AMS_2"

    def test_a_unit_on_a_wrong_family_bay_is_not_saved(
            self, tmp_path, monkeypatch):
        # No htmask at the claim: chain index 4 reads as an HT, so CCCC
        # claims the HT bay. By enrollment the htmask says boxed. Saving
        # a boxed unit on a one-lane HT bay would stop the next boot.
        bridge = _Bridge(uids=["AAAA", "", "", "", "CCCC"],
                         online=[False, False, False, False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_HT_1")["bound"] == "CCCC"
        bridge._htmask = 1 << 5
        for t in (116.0, 117.0, 130.0):
            _tick(m, bridge, t)
        assert "boxed:CCCC" in m._state_get(SEC, "roster")
        assert "CCCC" not in m._name_map
        warned = [x for x in m.logger.lines if "not saving it" in x]
        assert len(warned) == 1
        assert ("AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=CCCC FORCE=1"
                in warned[0])

    def test_option_mode_does_not_save_an_unlisted_unit(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        _tick(m, bridge, 130.0)
        assert "boxed:CCCC" in m._state_get(SEC, "roster")    # the file
        assert "CCCC" not in m._name_map and "CCCC" not in m._lane_map
        assert not m._bay_held(_bay(m, "Bambu_AMS_2"))
        # A restart names a newly listed unit in roster: order, so the bay
        # it holds now is not promised.
        (line,) = [x for x in m.logger.lines if "NEW unit(s)" in x]
        assert ("Add boxed:CCCC to roster: to enroll it (the option is set "
                "and overrides the file); the next restart gives it the "
                "lowest free bay of its family, which need not be the one it "
                "is on now.") in line
        assert "keep its bay" not in line
        assert _saved_lines(m) == []

    def test_a_name_saved_for_another_recorded_uid_is_not_double_pinned(
            self, tmp_path, monkeypatch):
        # ZZZZ is recorded and its saved name is Bambu_AMS_2, which a spare
        # wears this session: ZZZZ gets that bay at restart. It is the only
        # free AMS bay, so CCCC takes it, but is not saved there.
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=2)
        m._state_set({SEC: {"roster": "boxed:AAAA, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_2"
        for t in (100.0, 116.0, 117.0, 130.0):
            _tick(m, bridge, t)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert "CCCC" not in m._name_map
        warned = [x for x in m.logger.lines if "is saved for ZZZZ" in x]
        assert len(warned) == 1
        # Its popup does not promise a save that cannot happen.
        assert not any("saved on this bay" in x for x in _prompt_texts(m))
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="ZZZZ"))
        _tick(m, bridge, 131.0)
        assert m._name_map["CCCC"] == "Bambu_AMS_2"

    def test_a_new_unit_leaves_a_bay_saved_for_another_unit_free(
            self, tmp_path, monkeypatch):
        # With another free bay, CCCC takes that one instead and is saved
        # there, so neither unit moves at the next restart.
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        m._state_set({SEC: {"roster": "boxed:AAAA, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_2"
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] is None
        assert _bay(m, "Bambu_AMS_3")["bound"] == "CCCC"
        assert m._name_map["CCCC"] == "Bambu_AMS_3"
        assert not any("is saved for ZZZZ" in x for x in m.logger.lines)

    def test_a_name_saved_outside_the_roster_is_taken_over(
            self, tmp_path, monkeypatch):
        # ZZZZ is not recorded: its name holds no bay, and saving CCCC on
        # the spare wearing it drops ZZZZ's entries.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
            "lane_map": "AAAA:24:4, ZZZZ:28:4"}})
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        assert m._name_map.get("ZZZZ") == "Bambu_AMS_2"
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        assert _state_names(m) == {"AAAA": "Bambu_AMS_1",
                                   "CCCC": "Bambu_AMS_2"}
        assert "ZZZZ" not in m._lane_map

    def test_unassign_then_replug_saves_the_new_bay(
            self, tmp_path, monkeypatch):
        m, _u, bridge = self._cccc_saved(tmp_path, monkeypatch)
        bridge._uids.append("EEEE")
        _tick(m, bridge, 117.0, [False, False, False])
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC"))
        assert "CCCC" not in m._name_map
        _tick(m, bridge, 118.0, [False, False, True])   # EEEE takes _2
        for t in range(119, 136):
            _tick(m, bridge, float(t), [False, True, True])
        assert _bay(m, "Bambu_AMS_3")["bound"] == "CCCC"
        assert m._name_map["CCCC"] == "Bambu_AMS_3"
        assert m._name_map["EEEE"] == "Bambu_AMS_2"
        assert _state_names(m)["CCCC"] == "Bambu_AMS_3"

    def test_a_replugged_recorded_unit_is_saved_only_after_enroll_grace(
            self, tmp_path, monkeypatch):
        # Recorded already, so only the online run gates the save: a unit
        # that just came back, or a phantom online blip, saves nothing.
        m, _u, bridge = self._cccc_saved(tmp_path, monkeypatch)
        _tick(m, bridge, 117.0, [False, False])
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC"))
        _tick(m, bridge, 118.0, [False, True])
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert "CCCC" not in m._name_map
        _tick(m, bridge, 132.0)
        assert "CCCC" not in m._name_map
        _tick(m, bridge, 133.0)
        assert m._name_map["CCCC"] == "Bambu_AMS_2"

    def test_a_live_unassign_rehomes_and_saves_on_the_next_tick(
            self, tmp_path, monkeypatch):
        m, _u, bridge = self._cccc_saved(tmp_path, monkeypatch)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC", FORCE=1))
        assert _bay(m, "Bambu_AMS_2")["uid"] is None
        assert "CCCC" not in m._name_map
        # The command released it and cleared the bay: no unplug, no re-plug.
        assert [x for x in m.logger.lines if "released" in x] == [
            "AFC_BridgeBox chain1: released Bambu_AMS_2 (UID CCCC, "
            "AFC_BRIDGEBOX_UNASSIGN); lanes dropped live"]
        _tick(m, bridge, 120.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert m._name_map["CCCC"] == "Bambu_AMS_2"
        assert len(_saved_lines(m)) == 2

    def test_a_failed_claim_hands_an_adopted_bay_back_but_keeps_a_held_one(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA")
        # A spare adopted for the claim goes back to the pool.
        units["Bambu_AMS_2"].claim_ok = False
        assert m._claim_pool_unit("CCCC", "boxed") is None
        assert _bay(m, "Bambu_AMS_2")["uid"] is None
        # So does a rostered bay FORGET freed.
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        units["Bambu_AMS_1"].claim_ok = False
        assert m._claim_pool_unit("CCCC", "boxed") is None
        assert _bay(m, "Bambu_AMS_1")["uid"] is None
        # A bay the uid is saved on keeps it.
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_3"))
        units["Bambu_AMS_3"].claim_ok = False
        assert m._claim_pool_unit("CCCC", "boxed") is None
        assert _bay(m, "Bambu_AMS_3")["uid"] == "CCCC"

    def test_release_keeps_a_saved_bay_and_frees_an_unsaved_one(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        m._persist_pin("CCCC", 28, 4, "Bambu_AMS_2", "boxed")
        for name, uid in (("Bambu_AMS_2", "CCCC"), ("Bambu_AMS_3", "EEEE")):
            _bay(m, name)["uid"] = _bay(m, name)["bound"] = uid
        m._release_pool_unit("CCCC")
        m._release_pool_unit("EEEE")
        assert _bay(m, "Bambu_AMS_2")["uid"] == "CCCC"
        assert _bay(m, "Bambu_AMS_3")["uid"] is None
        kept = [x for x in m.logger.lines if "slot kept for re-plug" in x]
        assert len(kept) == 1 and "Bambu_AMS_2" in kept[0]

    def test_assign_onto_a_spare_is_held_across_an_auto_drop(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", auto_drop=True)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_3"))
        assert _bay(m, "Bambu_AMS_3")["bound"] == "CCCC"
        for t in range(100, 112):
            _tick(m, bridge, float(t), [False, False])
        bay = _bay(m, "Bambu_AMS_3")
        assert bay["bound"] is None and bay["uid"] == "CCCC"

    def test_assign_refuses_a_name_saved_for_another_uid(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        m._state_set({SEC: {"roster": "boxed:AAAA, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_3"
        with pytest.raises(Exception, match="saved for ZZZZ"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_3"))
        assert _bay(m, "Bambu_AMS_3")["uid"] is None
        assert m._name_map["ZZZZ"] == "Bambu_AMS_3"

    def test_a_returning_uid_takes_the_spare_wearing_its_saved_name(
            self, tmp_path, monkeypatch):
        # ZZZZ left the roster without FORGET; a spare wears its name. When
        # it comes back it claims that spare, not the lowest, and no "new
        # unit" popup asks where to put it.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_3",
            "lane_map": "AAAA:24:4, ZZZZ:32:4"}})
        bridge = _Bridge(uids=["AAAA", "ZZZZ"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_3")["bound"] == "ZZZZ"
        assert _bay(m, "Bambu_AMS_2")["bound"] is None
        assert _new_popups(m) == []
        # Not recorded yet, so it has no bay at restart and holds none.
        assert not m._bay_held(_bay(m, "Bambu_AMS_3"))
        _tick(m, bridge, 116.0)
        assert m._bay_held(_bay(m, "Bambu_AMS_3"))
        _m2, p2, _a, _s = _reboot(tmp_path)
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_3")["unit_uid"] == "ZZZZ"

    def test_an_unlisted_uid_on_the_spare_wearing_its_name_is_not_held(
            self, tmp_path, monkeypatch):
        # ZZZZ was pinned in an earlier file-mode session; the roster: option
        # does not list it. It may come back to the spare wearing its name,
        # but it gets no bay of its own at restart, so that spare goes back
        # to the pool when it is pulled, and its popup offers roster:.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_3",
            "lane_map": "AAAA:24:4, ZZZZ:32:4"}})
        bridge = _Bridge(uids=["AAAA", "ZZZZ"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               roster="boxed:AAAA", auto_drop=True)
        _tick(m, bridge, 100.0)
        bay = _bay(m, "Bambu_AMS_3")
        assert bay["bound"] == "ZZZZ" and not m._bay_held(bay)
        assert _new_popups(m) == [
            "// action:prompt_begin New AMS on Bambu_AMS_3"]
        for t in range(101, 115):
            _tick(m, bridge, float(t), [False, False])
        assert bay["bound"] is None and bay["uid"] is None
        assert not any("slot kept" in x for x in m.logger.lines)

    def test_pools_off_never_auto_pins(self, tmp_path, monkeypatch):
        # Nothing is claimed live with no pool, so nothing is pinned live;
        # the boot still gives a rostered bay its uid's record.
        s = TestLearnedFollowsUid._state(tmp_path)
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            s._learned_section("AAAA"): {"afc_bowden_length": "3632.0"}})
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=0,
                               pool_ht=0)
        assert _loaded(m.printer, "AFC_BambuAMS Bambu_AMS_1")[
            "afc_bowden_length"] == "3632.0"
        for t in (100.0, 116.0, 130.0):
            _tick(m, bridge, t)
        assert "boxed:CCCC" in m._state_get(SEC, "roster")
        assert "CCCC" not in m._name_map and _saved_lines(m) == []
        assert any("RESTART to enroll" in x for x in m.logger.lines)
        cmd = _GCmd(UID="AAAA")
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert cmd.responses[0].endswith(
            "(learned values stay with the unit). It takes the lowest free "
            "bay of its family at the next restart.")

    def test_status_does_not_list_a_unit_saved_this_session_as_a_tombstone(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_3"))
        assert "CCCC" in m._lane_map
        assert m.get_status()["tombstones"] == []
        _tick(m, bridge, 100.0, [False, False])     # pulled; still recorded
        m._release_pool_unit("CCCC")
        assert m.get_status()["tombstones"] == []


    def test_a_tombstone_seen_for_a_moment_does_not_hold_the_spare(
            self, tmp_path, monkeypatch):
        # ZZZZ is outside the roster, so its name holds no bay. It blips
        # online (a unit the bridge still re-asserts can), claims the spare
        # wearing its name, and drops: the spare goes back to the pool for
        # the next unit instead of waiting all session for ZZZZ.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
            "lane_map": "AAAA:24:4, ZZZZ:28:4"}})
        bridge = _Bridge(uids=["AAAA", "ZZZZ", "NEWW"],
                         online=[False, True, False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=2,
                               auto_drop=True)
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "ZZZZ"
        for t in range(101, 115):
            _tick(m, bridge, float(t), [False, False, False])
        bay = _bay(m, "Bambu_AMS_2")
        assert bay["bound"] is None and bay["uid"] is None
        assert not any("slot kept" in x for x in m.logger.lines)
        for t in range(115, 131):
            _tick(m, bridge, float(t), [False, False, True])
        assert bay["bound"] == "NEWW"
        assert not any("no free pool slot" in x for x in m.logger.lines)
        assert _state_names(m) == {"AAAA": "Bambu_AMS_1",
                                   "NEWW": "Bambu_AMS_2"}

    @pytest.mark.parametrize("first", ["new", "returning"])
    def test_a_returning_and_a_new_unit_on_one_tick_keep_their_bays(
            self, tmp_path, monkeypatch, first):
        # ZZZZ's saved name is worn by a spare, and a new unit CCCC appears
        # on the same tick. Whichever the tick meets first, ZZZZ gets its
        # own bay back and CCCC another, and the restart matches the live
        # bays.
        _force_claim_order(monkeypatch, first)
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
            "lane_map": "AAAA:24:4, ZZZZ:28:4"}})
        bridge = _Bridge(uids=["AAAA", "CCCC", "ZZZZ"],
                         online=[False, True, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        for t in range(100, 118):
            _tick(m, bridge, float(t))
        live = {pu["bound"]: pu["name"] for pu in m._pool_units
                if pu["bound"]}
        assert live == {"ZZZZ": "Bambu_AMS_2", "CCCC": "Bambu_AMS_3"}
        assert not any("is saved for" in x for x in m.logger.lines)
        _m2, p2, _a, _s = _reboot(tmp_path)
        for uid, name in live.items():
            assert _loaded(p2, f"AFC_BambuAMS {name}")["unit_uid"] == uid

    @pytest.mark.parametrize("first", ["new", "returning"])
    def test_a_last_bay_another_unit_comes_back_to_is_left_to_it(
            self, tmp_path, monkeypatch, first):
        # CCCC was last claimed onto Bambu_AMS_2, and ZZZZ's saved name is
        # Bambu_AMS_2; both appear on one tick. CCCC leaves the bay to
        # ZZZZ, so both are saved where they are live and the restart
        # moves neither.
        _force_claim_order(monkeypatch, first)
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
            "lane_map": "AAAA:24:4, ZZZZ:28:4",
            "bay_owner": "AAAA:Bambu_AMS_1, CCCC:Bambu_AMS_2"}})
        bridge = _Bridge(uids=["AAAA", "CCCC", "ZZZZ"],
                         online=[False, True, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        for t in (100.0, 116.0, 130.0):
            _tick(m, bridge, t)
        live = {pu["bound"]: pu["name"] for pu in m._pool_units
                if pu["bound"]}
        assert live == {"ZZZZ": "Bambu_AMS_2", "CCCC": "Bambu_AMS_3"}
        assert {u: m._name_map[u] for u in live} == live
        assert not any("is saved for" in x for x in m.logger.lines)
        _m2, p2, _a, _s = _reboot(tmp_path)
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_2")["unit_uid"] == "ZZZZ"
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_3")["unit_uid"] == "CCCC"

    @pytest.mark.parametrize("first", ["new", "returning"])
    def test_a_returning_unit_claims_its_bay_before_a_new_one(
            self, tmp_path, monkeypatch, first):
        # The spare wearing ZZZZ's name is the only free AMS bay, and both
        # units appear on one tick: ZZZZ claims first, whatever the order
        # the tick meets them in, and CCCC waits for a bay.
        _force_claim_order(monkeypatch, first)
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
            "lane_map": "AAAA:24:4, ZZZZ:28:4"}})
        bridge = _Bridge(uids=["AAAA", "CCCC", "ZZZZ"],
                         online=[False, True, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=2)
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "ZZZZ"
        assert not any(pu["bound"] == "CCCC" for pu in m._pool_units)

    def test_a_new_unit_leaves_the_bay_a_waiting_unit_comes_back_to(
            self, tmp_path, monkeypatch):
        # ZZZZ was released moments ago, so it must hold online for
        # flap_claim_grace before it claims again; CCCC, plugged at the same
        # time, claims at once. It leaves the spare wearing ZZZZ's name for
        # ZZZZ, so both are saved where they are live.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
            "lane_map": "AAAA:24:4, ZZZZ:28:4"}})
        bridge = _Bridge(uids=["AAAA", "CCCC", "ZZZZ"],
                         online=[False, True, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        m._released_at = {"ZZZZ": 95.0}
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_3")["bound"] == "CCCC"
        assert _bay(m, "Bambu_AMS_2")["bound"] is None
        for t in range(101, 118):
            _tick(m, bridge, float(t))
        assert _bay(m, "Bambu_AMS_2")["bound"] == "ZZZZ"
        assert {u: m._name_map[u] for u in ("CCCC", "ZZZZ")} == {
            "CCCC": "Bambu_AMS_3", "ZZZZ": "Bambu_AMS_2"}
        _m2, p2, _a, _s = _reboot(tmp_path)
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_2")["unit_uid"] == "ZZZZ"
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_3")["unit_uid"] == "CCCC"

    def test_a_unit_saved_on_another_bay_of_this_session_keeps_that_pin(
            self, tmp_path, monkeypatch):
        # ZZZZ is saved on Bambu_AMS_2 but live on Bambu_AMS_3, because
        # XXXX sat on its bay when it came back: it goes back to its bay at
        # restart, and XXXX, on a bay saved for ZZZZ, is not saved there.
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        m._name_map["ZZZZ"] = "Bambu_AMS_2"
        for name, uid in (("Bambu_AMS_2", "XXXX"), ("Bambu_AMS_3", "ZZZZ")):
            _bay(m, name)["uid"] = _bay(m, name)["bound"] = uid
        recorded = {"AAAA", "XXXX", "ZZZZ"}
        for t in (100.0, 101.0):
            m._pin_recorded_units(recorded, t, {"XXXX": 0.0, "ZZZZ": 0.0})
        assert [x for x in m.logger.lines if "this session" in x] == [
            "AFC_BridgeBox chain1: ZZZZ is on Bambu_AMS_3 this session, not "
            "on Bambu_AMS_2, the bay it is saved on; it comes back to "
            "Bambu_AMS_2 after a restart."]
        assert len([x for x in m.logger.lines
                    if "Bambu_AMS_2 is saved for ZZZZ, so XXXX" in x]) == 1
        assert m._name_map == {"AAAA": "Bambu_AMS_1", "ZZZZ": "Bambu_AMS_2"}
        assert _saved_lines(m) == []

    def test_a_saved_name_that_is_no_bay_now_gives_way_to_the_live_bay(
            self, tmp_path, monkeypatch):
        # ZZZZ is saved as Bambu_AMS_5, which a pool of three AMS bays does
        # not have. Kept, it would come up past the HT lanes (or be redrawn)
        # at restart; its live bay is saved instead.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {SEC: {
            "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_5",
            "lane_map": "AAAA:24:4, ZZZZ:40:4"}})
        bridge = _Bridge(uids=["AAAA", "ZZZZ"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "ZZZZ"
        assert _saved_lines(m) == [
            "AFC_BridgeBox chain1: saved ZZZZ on Bambu_AMS_2 (lane28-lane31, "
            "T28-T31); it comes back there after a restart."]
        assert not any("this session" in x for x in m.logger.lines)
        _m2, p2, _a, _s = _reboot(tmp_path)
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_2")["unit_uid"] == "ZZZZ"
        assert _loaded(p2, "AFC_lane lane28")["unit"] == "Bambu_AMS_2:1"

    def test_an_ht_is_saved_on_its_one_lane(self, tmp_path, monkeypatch):
        # Chain index 4 with no htmask reads as an HT.
        bridge = _Bridge(uids=["AAAA", "", "", "", "HHHH"],
                         online=[False, False, False, False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        assert _saved_lines(m) == [
            "AFC_BridgeBox chain1: saved HHHH on Bambu_AMS_HT_1 (lane36, "
            "T36); it comes back there after a restart."]
        _m2, p2, _a, _s = _reboot(tmp_path)
        assert _loaded(p2, "AFC_BambuAMS Bambu_AMS_HT_1")["unit_uid"] == \
            "HHHH"
        assert _loaded(p2, "AFC_lane lane36")["unit"] == "Bambu_AMS_HT_1:1"

    def test_a_unit_recorded_with_no_bay_is_saved_when_one_frees(
            self, tmp_path, monkeypatch):
        # One AMS bay, AAAA's. CCCC is recorded with no bay; FORGET frees
        # AAAA's, CCCC claims it on the next tick and, long online by then,
        # is saved on that tick.
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=1)
        _tick(m, bridge, 100.0)
        _tick(m, bridge, 116.0)
        assert "boxed:CCCC" in m._state_get(SEC, "roster")
        # The claim said CCCC has no free bay; the enrollment line does not
        # say it again.
        assert any("new AMS CCCC has no free bay" in x for x in m.logger.lines)
        assert ("AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC "
                "-- recorded." in m.logger.lines)
        assert not any("temperature card" in x for x in m.logger.lines)
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        _tick(m, bridge, 117.0)
        assert _bay(m, "Bambu_AMS_1")["bound"] == "CCCC"
        assert _saved_lines(m) == [
            "AFC_BridgeBox chain1: saved CCCC on Bambu_AMS_1 (lane24-lane27, "
            "T24-T27); it comes back there after a restart."]


# ── B: learned values follow the UID ─────────────────────────────────────────

class TestLearnedFollowsUid:

    @staticmethod
    def _state(tmp_path):
        m = afcBridgeBox.__new__(afcBridgeBox)
        m.state_file = str(tmp_path / "AFC_BridgeBox.cfg")
        m.name = "chain1"
        return m

    def test_migration_moves_a_name_value_to_the_pinned_uid(self, tmp_path):
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA", "name_map": "AAAA:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4"},
            "AFC_BambuAMS Bambu_AMS_1": {"afc_bowden_length": "3632.0"}})
        m, p, _a, _s = _reboot(tmp_path)
        assert m._learned_for("AAAA") == {"afc_bowden_length": "3632.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_1",
                            "afc_bowden_length") is None
        assert _loaded(p, "AFC_BambuAMS Bambu_AMS_1")[
            "afc_bowden_length"] == "3632.0"
        m.logger = _Logger()
        p.get_reactor = lambda: _FakeReactor()
        m._scout_ready()
        assert any("now belong to its unit AAAA" in x
                   for x in m.logger.lines)

    def test_an_unowned_name_value_is_dropped_not_inherited(self, tmp_path):
        # Bambu_AMS_2 is a spare: whoever measured 3632 there, the next unit
        # to claim it did not.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA"},
            "AFC_BambuAMS Bambu_AMS_2": {"afc_bowden_length": "3632.0"}})
        m, p, _a, _s = _reboot(tmp_path)
        assert m._state_get("AFC_BambuAMS Bambu_AMS_2",
                            "afc_bowden_length") is None
        assert "afc_bowden_length" not in _loaded(
            p, "AFC_BambuAMS Bambu_AMS_2")
        m.logger = _Logger()
        p.get_reactor = lambda: _FakeReactor()
        m._scout_ready()
        assert any("stored under Bambu_AMS_2 dropped" in x
                   for x in m.logger.lines)

    def test_a_spare_value_goes_to_the_one_unit_roster_leaves_out(
            self, tmp_path):
        # roster: lists AAAA only. A build before bay_owner ran the recorded
        # CCCC on the Bambu_AMS_2 spare every session, and it measured 1800
        # there.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA, boxed:CCCC",
                  "name_map": "AAAA:Bambu_AMS_1", "lane_map": "AAAA:24:4"},
            "AFC_BambuAMS Bambu_AMS_2": {"afc_bowden_length": "1800.0"}})
        m, _p, _a, _s = _mk_files(tmp_path, **dict(POOL, roster="boxed:AAAA"))
        assert m._roster_source == "option"
        assert m._learned_for("CCCC") == {"afc_bowden_length": "1800.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_2",
                            "afc_bowden_length") is None
        assert [t for _w, t in m._learned_notes] == [
            "learned values stored under Bambu_AMS_2 now belong to its unit "
            "CCCC (afc_bowden_length)"]

    @pytest.mark.parametrize("recorded, spares", [
        ("boxed:AAAA, boxed:CCCC, boxed:DDDD", ["Bambu_AMS_2"]),
        ("boxed:AAAA, boxed:CCCC", ["Bambu_AMS_2", "Bambu_AMS_3"]),
    ], ids=["two-units", "two-spares"])
    def test_a_spare_value_is_not_guessed_among_several(self, tmp_path,
                                                         recorded, spares):
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", dict(
            {SEC: {"roster": recorded, "name_map": "AAAA:Bambu_AMS_1",
                   "lane_map": "AAAA:24:4"}},
            **{f"AFC_BambuAMS {n}": {"afc_bowden_length": "1800.0"}
               for n in spares}))
        m, _p, _a, _s = _mk_files(tmp_path, **dict(POOL, roster="boxed:AAAA"))
        assert m._learned_for("CCCC") == {}
        assert m._learned_for("DDDD") == {}
        assert [t for _w, t in m._learned_notes] == [
            f"learned values stored under {n} dropped -- no unit is "
            f"recorded with that name" for n in spares]

    def test_the_uid_record_wins_a_migration_conflict(self, tmp_path):
        s = self._state(tmp_path)
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA", "name_map": "AAAA:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4"},
            s._learned_section("AAAA"): {"afc_bowden_length": "3000.0"},
            "AFC_BambuAMS Bambu_AMS_1": {"afc_bowden_length": "3632.0",
                                         "afc_unload_bowden_length": "3500"}})
        m, p, _a, _s = _reboot(tmp_path)
        assert m._learned_for("AAAA") == {
            "afc_bowden_length": "3000.0", "afc_unload_bowden_length": "3500"}
        assert _loaded(p, "AFC_BambuAMS Bambu_AMS_1")[
            "afc_bowden_length"] == "3000.0"

    def test_two_owners_resolve_to_the_rostered_bay(self, tmp_path):
        # A legacy state recording two uids with one name. The rostered one
        # takes the name at boot, so the values are its own.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA",
                  "name_map": "AAAA:Bambu_AMS_1, XXXX:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4, XXXX:24:4"},
            "AFC_BambuAMS Bambu_AMS_1": {"afc_bowden_length": "3632.0"}})
        m, _p, _a, _s = _reboot(tmp_path)
        assert m._learned_for("AAAA") == {"afc_bowden_length": "3632.0"}
        assert m._learned_for("XXXX") == {}
        # The same narrowing inside the migration itself, and a name left
        # to two uids neither on its bay stays unread.
        m._name_map.update({"XXXX": "Bambu_AMS_1", "YYYY": "Bambu_AMS_9",
                            "WWWW": "Bambu_AMS_9"})
        m._state_set({"AFC_BambuAMS Bambu_AMS_1":
                      {"afc_unload_bowden_length": "3500"},
                      "AFC_BambuAMS Bambu_AMS_9":
                      {"afc_bowden_length": "1111"}})
        m._learned_notes = []
        m._migrate_name_learned()
        assert m._learned_for("AAAA")["afc_unload_bowden_length"] == "3500"
        assert m._learned_for("XXXX") == {}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_9",
                            "afc_bowden_length") == "1111"
        assert [w for w, _t in m._learned_notes] == [False, True]

    def test_a_uid_moved_off_its_name_keeps_what_it_learned_there(
            self, tmp_path):
        # The boot can move a recorded uid to another name, or leave it with
        # none. What was stored under its old name is its own; what is
        # stored under the name it moved to belongs to the uid that wore
        # that name before.
        m = self._state(tmp_path)
        m._pool_units = []
        m._learned_notes = []
        m._state_set({
            "AFC_BambuAMS Bambu_AMS_5": {"afc_bowden_length": "3632.0"},
            "AFC_BambuAMS Bambu_AMS_3": {"afc_bowden_length": "1111.0"},
            "AFC_BambuAMS Bambu_AMS_6": {"afc_bowden_length": "2222.0"}})
        m._name_map = {"XXXX": "Bambu_AMS_3"}
        m._migrate_name_learned({"XXXX": "Bambu_AMS_5",
                                 "TTTT": "Bambu_AMS_3",
                                 "YYYY": "Bambu_AMS_6"})
        assert m._learned_for("XXXX") == {"afc_bowden_length": "3632.0"}
        assert m._learned_for("TTTT") == {"afc_bowden_length": "1111.0"}
        assert m._learned_for("YYYY") == {"afc_bowden_length": "2222.0"}
        cp = m._read_state()
        assert not [s for s in cp.sections() if s.startswith("AFC_BambuAMS")]

    def test_a_non_learned_key_is_not_migrated_or_folded(self, tmp_path):
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA", "name_map": "AAAA:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4"},
            "AFC_BambuAMS Bambu_AMS_1": {"pool": "False", "heater": "True",
                                         "afc_bowden_length": "3632.0"}})
        (tmp_path / "AFC_auto_vars.cfg").write_text(
            "[AFC_BambuAMS Bambu_AMS_1]\nmeasure_on_insert : True\n")
        m, p, _a, _s = _reboot(tmp_path)
        u = _loaded(p, "AFC_BambuAMS Bambu_AMS_1")
        assert u["pool"] == "True"
        assert "heater" not in u
        assert u["measure_on_insert"] == "False"
        assert u["afc_bowden_length"] == "3632.0"
        assert m._learned_for("AAAA") == {"afc_bowden_length": "3632.0"}
        text = (tmp_path / "AFC_BridgeBox.cfg").read_text()
        assert "heater" not in text and "measure_on_insert" not in text

    def test_migration_leaves_another_chains_sections_alone(self, tmp_path):
        # Bambu_AMS_B_1 is no name of this chain: it belongs to a chain
        # with its own prefix, which migrates it itself.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA", "name_map": "AAAA:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4"},
            "AFC_BambuAMS Bambu_AMS_1": {"afc_bowden_length": "3632.0"},
            "AFC_BambuAMS Bambu_AMS_B_1": {"afc_bowden_length": "1234"}})
        m, _p, _a, _s = _reboot(tmp_path)
        assert m._learned_for("AAAA") == {"afc_bowden_length": "3632.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_B_1",
                            "afc_bowden_length") == "1234"

    def test_a_moved_unit_takes_its_values_to_the_new_bay_at_restart(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA",
                               ams_names="Alpha, Bravo, Charlie")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        assert m.persist_learned("Bravo", "afc_bowden_length", 3632.0)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Charlie",
                                         FORCE=1))
        _m2, p2, _a, _s = _reboot(tmp_path,
                                  ams_names="Alpha, Bravo, Charlie")
        assert _loaded(p2, "AFC_BambuAMS Charlie")[
            "afc_bowden_length"] == "3632.0"
        assert "afc_bowden_length" not in _loaded(p2, "AFC_BambuAMS Bravo")

    def test_persist_then_restart_round_trip(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert m.persist_learned("Bambu_AMS_1", "afc_bowden_length", 3627.0)
        assert m._learned_for("AAAA") == {"afc_bowden_length": "3627.0"}
        m2, p2, _a, _s = _reboot(tmp_path)
        folded = dict(m2._fold_and_sweep(_Config({}, p2),
                                         m2._roster_sections(m2.units)))
        assert folded["AFC_BambuAMS Bambu_AMS_1"][
            "afc_bowden_length"] == "3627.0"

    def test_persist_learned_files_under_the_bound_uid(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert m.persist_learned("Bambu_AMS_2", "afc_bowden_length", 3627.0)
        assert m._learned_for("CCCC") == {"afc_bowden_length": "3627.0"}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_2",
                            "afc_bowden_length") is None
        # Nothing is on Bambu_AMS_3, so nothing is saved for it.
        assert not m.persist_learned("Bambu_AMS_3", "afc_bowden_length", 1.0)

    def test_a_live_claim_applies_the_uids_record(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA")
        m._state_set({m._learned_section("CCCC"):
                      {"afc_bowden_length": "3632.0"}})
        _tick(m, bridge, 100.0)
        assert units["Bambu_AMS_2"].learned == [{"afc_bowden_length": 3632.0}]
        assert any("Bambu_AMS_2 takes the afc_bowden_length 3632mm that UID "
                   "CCCC learned" in x for x in m.logger.lines)

    def test_the_claim_line_shows_a_learned_fraction(self, tmp_path,
                                                     monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA")
        m._state_set({m._learned_section("CCCC"):
                      {"afc_bowden_length": "1234.5",
                       "afc_unload_bowden_length": "1234.44"}})
        _tick(m, bridge, 100.0)
        assert any("takes the afc_bowden_length 1234.5mm, "
                   "afc_unload_bowden_length 1234.4mm that UID CCCC learned"
                   in x for x in m.logger.lines)

    def test_a_different_uid_on_the_same_bay_gets_defaults(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC", "EEEE"],
                         online=[False, True, False])
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA", auto_drop=True)
        m._state_set({m._learned_section("CCCC"):
                      {"afc_bowden_length": "3632.0"}})
        _tick(m, bridge, 100.0)
        for t in range(101, 113):
            _tick(m, bridge, float(t), [False, False, False])
        assert _bay(m, "Bambu_AMS_2")["bound"] is None
        _tick(m, bridge, 113.0, [False, False, True])
        assert _bay(m, "Bambu_AMS_2")["bound"] == "EEEE"
        assert units["Bambu_AMS_2"].learned == [
            {"afc_bowden_length": 3632.0}, {}]

    def test_an_override_beats_the_record_at_claim(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        fc = _FileConfig({"AFC_BridgeBox Bambu_AMS_2":
                          {"afc_bowden_length": "1800"}})
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA", fileconfig=fc)
        m._state_set({m._learned_section("CCCC"):
                      {"afc_bowden_length": "3632.0",
                       "afc_unload_bowden_length": "3632.0"}})
        _tick(m, bridge, 100.0)
        assert units["Bambu_AMS_2"].learned == [
            {"afc_bowden_length": 1800.0, "afc_unload_bowden_length": 3632.0}]

    def test_forget_erases_the_uid_record_and_unassign_keeps_it(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        for uid in ("AAAA", "CCCC"):
            m._state_set({m._learned_section(uid):
                          {"afc_bowden_length": "3632.0"}})
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_2"))
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC"))
        assert m._learned_for("CCCC") == {"afc_bowden_length": "3632.0"}
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        assert m._learned_for("AAAA") == {}
        assert m._learned_for("CCCC") == {"afc_bowden_length": "3632.0"}

    def test_forget_says_learned_values_erased_only_when_there_were_some(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        cmd = _GCmd(UID="AAAA")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.responses[0].startswith(
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Bambu_AMS_1 freed for reuse -- slot freed")
        assert "learned values" not in cmd.responses[0]

    def test_forget_keeps_a_name_another_uid_still_holds(self, tmp_path):
        # The state records XXXX and YYYY with one name, so what is stored
        # under it is left unread. After FORGET of XXXX it is YYYY's, and the
        # next restart gives it the values; FORGET must not erase the bay's
        # sections under it.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA",
                  "name_map": "AAAA:Bambu_AMS_1, XXXX:Bambu_AMS_3, "
                              "YYYY:Bambu_AMS_3",
                  "lane_map": "AAAA:24:4, XXXX:32:4, YYYY:32:4"},
            "AFC_BambuAMS Bambu_AMS_3": {"afc_bowden_length": "3632.0"},
            "AFC_hub Bambu_AMS_3": {"afc_bowden_length": "1800"}})
        m, _p, _a, _s = _reboot(tmp_path)
        warned = [t for w, t in m._learned_notes if w]
        assert warned == [
            "learned values stored under Bambu_AMS_3 left unread -- XXXX, "
            "YYYY are all recorded with that name. Run AFC_BRIDGEBOX_FORGET "
            "CHAIN=chain1 UID=<uid> for each one that is gone, and the one "
            "left takes them at the next restart."]
        cmd = _GCmd(UID="XXXX")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.responses[0] == ("AFC_BridgeBox chain1: forgot XXXX.")
        assert m._state_get("AFC_BambuAMS Bambu_AMS_3",
                            "afc_bowden_length") == "3632.0"
        assert m._state_get("AFC_hub Bambu_AMS_3",
                            "afc_bowden_length") == "1800"
        m2, _p2, _a2, _s2 = _reboot(tmp_path)
        assert m2._learned_for("YYYY") == {"afc_bowden_length": "3632.0"}
        assert m2._learned_for("XXXX") == {}

    def test_a_name_taken_over_at_boot_keeps_its_values_with_its_holder(
            self, tmp_path):
        # ZZZZ is outside the roster and CCCC, new in it, draws ZZZZ's name
        # at this boot. What is stored under the name is ZZZZ's: CCCC
        # measures its own.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA, boxed:CCCC",
                  "name_map": "AAAA:Bambu_AMS_1, ZZZZ:Bambu_AMS_2",
                  "lane_map": "AAAA:24:4, ZZZZ:28:4"},
            "AFC_BambuAMS Bambu_AMS_2": {"afc_bowden_length": "3632.0"}})
        m, p, _a, _s = _reboot(tmp_path)
        assert m._name_map == {"AAAA": "Bambu_AMS_1", "CCCC": "Bambu_AMS_2"}
        assert m._learned_for("CCCC") == {}
        assert m._learned_for("ZZZZ") == {"afc_bowden_length": "3632.0"}
        assert "afc_bowden_length" not in _loaded(
            p, "AFC_BambuAMS Bambu_AMS_2")
        assert [t for _w, t in m._learned_notes] == [
            "learned values stored under Bambu_AMS_2 now belong to its unit "
            "ZZZZ (afc_bowden_length), which wore that name before CCCC took "
            "it; CCCC measures its own"]

    def test_a_name_several_gone_uids_shared_is_dropped_once_taken_over(
            self, tmp_path):
        # Nobody left can claim it: the uids that shared the name lost it
        # to RRRR, which must not get their values.
        m = self._state(tmp_path)
        m._pool_units = []
        m._learned_notes = []
        m._state_set({"AFC_BambuAMS Bambu_AMS_3":
                      {"afc_bowden_length": "3632.0"}})
        m._name_map = {"RRRR": "Bambu_AMS_3"}
        m._migrate_name_learned({"T1T1": "Bambu_AMS_3",
                                 "T2T2": "Bambu_AMS_3"})
        assert m._learned_for("RRRR") == {}
        assert not m._read_state().has_section("AFC_BambuAMS Bambu_AMS_3")
        assert m._learned_notes == [(
            False, "learned values stored under Bambu_AMS_3 dropped -- "
            "T1T1, T2T2 were all recorded with that name and another unit "
            "took it over")]

    def test_forget_accepts_a_learned_only_uid(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        m._state_set({m._learned_section("QQQQ"):
                      {"afc_bowden_length": "3632.0"}})
        cmd = _GCmd(UID="QQQQ")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._learned_for("QQQQ") == {}
        assert m._read_state().has_section(m._learned_section("QQQQ")) is False
        assert cmd.responses[0] == (
            "AFC_BridgeBox chain1: forgot QQQQ -- learned values erased.")

    def test_a_record_that_is_not_a_length_is_never_folded(self, tmp_path):
        # The fold writes a record into the unit section, and the unit reads
        # it with getfloat(above=0): anything else would stop Klipper.
        s = self._state(tmp_path)
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA", "name_map": "AAAA:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4"},
            s._learned_section("AAAA"): {"afc_bowden_length": "abc",
                                         "afc_unload_bowden_length": "-5",
                                         "pool": "False"}})
        m, p, _a, _s = _reboot(tmp_path)
        assert m._learned_for("AAAA") == {}
        u = _loaded(p, "AFC_BambuAMS Bambu_AMS_1")
        assert "afc_bowden_length" not in u
        assert "afc_unload_bowden_length" not in u
        assert u["pool"] == "True"

    def test_a_length_that_is_not_finite_is_never_folded_or_applied(
            self, tmp_path, monkeypatch):
        s = self._state(tmp_path)
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA", "name_map": "AAAA:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4"},
            s._learned_section("AAAA"): {"afc_bowden_length": "inf",
                                         "afc_unload_bowden_length": "nan"},
            s._learned_section("CCCC"): {"afc_bowden_length": "1e400"}})
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, units = _live_pool(tmp_path, monkeypatch, bridge, file_roster=None,
                              roster="")
        u = _loaded(m.printer, "AFC_BambuAMS Bambu_AMS_1")
        assert "afc_bowden_length" not in u
        assert "afc_unload_bowden_length" not in u
        assert m._learned_for("CCCC") == {}
        _tick(m, bridge, 100.0)
        assert units["Bambu_AMS_2"].learned == [{}]

    def test_an_auto_vars_value_that_is_not_a_length_is_not_folded(
            self, tmp_path):
        # A leftover auto_vars section for a rostered bay folds only a
        # usable length: getfloat(above=0) would stop this very boot.
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA", "name_map": "AAAA:Bambu_AMS_1",
                  "lane_map": "AAAA:24:4"}})
        (tmp_path / "AFC_auto_vars.cfg").write_text(
            "[AFC_BambuAMS Bambu_AMS_1]\nafc_bowden_length : 0\n"
            "afc_unload_bowden_length : inf\n")
        m, p, _a, _s = _reboot(tmp_path)
        u = _loaded(p, "AFC_BambuAMS Bambu_AMS_1")
        assert "afc_bowden_length" not in u
        assert "afc_unload_bowden_length" not in u
        assert m._learned_for("AAAA") == {}
        assert " learned AAAA" not in (
            tmp_path / "AFC_BridgeBox.cfg").read_text()
        assert [t for _w, t in m._learned_notes] == [
            "auto_vars [AFC_BambuAMS Bambu_AMS_1]: afc_bowden_length, "
            "afc_unload_bowden_length not folded -- a unit takes only its "
            "bowden lengths from there, each a positive number"]

    def test_two_chains_keep_separate_records(self, tmp_path):
        def _chain(name, **over):
            opts = {"serial_port": f"/dev/serial/by-id/usb-{name}-if00",
                    "extruder": "extruder", "roster": "boxed:AAAA",
                    "pool_ams": 1, "pool_ht": 0,
                    "auto_vars_file": str(tmp_path / "AFC_auto_vars.cfg"),
                    "state_file": str(tmp_path / "AFC_BridgeBox.cfg")}
            opts.update(over)
            return afcBridgeBox(_Config(opts, _Printer(),
                                        name=f"AFC_BridgeBox {name}"))
        m1 = _chain("chain1", lane_base=24)
        m2 = _chain("chain2", lane_base=40, unit_prefix="Bambu_AMS_B",
                    buffer_chip_name="bambu_buffer_chain2")
        assert m1._learned_section("AAAA") != m2._learned_section("AAAA")
        m1._state_set({m1._learned_section("AAAA"):
                       {"afc_bowden_length": "3632.0"}})
        assert m2._learned_for("AAAA") == {}
        p = _Printer()
        again = afcBridgeBox(_Config(
            {"serial_port": "/dev/serial/by-id/usb-chain2-if00",
             "extruder": "extruder", "roster": "boxed:AAAA", "pool_ams": 1,
             "pool_ht": 0, "lane_base": 40, "unit_prefix": "Bambu_AMS_B",
             "buffer_chip_name": "bambu_buffer_chain2",
             "auto_vars_file": str(tmp_path / "AFC_auto_vars.cfg"),
             "state_file": str(tmp_path / "AFC_BridgeBox.cfg")}, p,
            name="AFC_BridgeBox chain2"))
        assert again.name == "chain2"
        assert "afc_bowden_length" not in _loaded(
            p, "AFC_BambuAMS Bambu_AMS_B_1")

    def test_an_auto_vars_leftover_for_a_spare_is_swept_not_folded(
            self, tmp_path):
        (tmp_path / "AFC_auto_vars.cfg").write_text(
            "[AFC_BambuAMS Bambu_AMS_2]\nafc_bowden_length : 3632.0\n")
        _seed_state(tmp_path / "AFC_BridgeBox.cfg",
                    {SEC: {"roster": "boxed:AAAA"}})
        m, p, autov, _s = _reboot(tmp_path)
        assert "afc_bowden_length" not in _loaded(
            p, "AFC_BambuAMS Bambu_AMS_2")
        assert "Bambu_AMS_2" not in autov.read_text()
        assert "3632" not in (tmp_path / "AFC_BridgeBox.cfg").read_text()
        assert any("swept, not folded" in t for _w, t in m._learned_notes)

    def test_an_old_state_file_boots_cleanly(self, tmp_path):
        _seed_state(tmp_path / "AFC_BridgeBox.cfg", {
            SEC: {"roster": "boxed:AAAA, ht:HHHH",
                  "name_map": "AAAA:Bambu_AMS_1, HHHH:Bambu_AMS_HT_1",
                  "lane_map": "AAAA:24:4, HHHH:36:1"}})
        m, p, _a, _s = _reboot(tmp_path)
        assert _loaded(p, "AFC_BambuAMS Bambu_AMS_1")["unit_uid"] == "AAAA"
        assert _loaded(p, "AFC_BambuAMS Bambu_AMS_HT_1")["unit_uid"] == "HHHH"
        assert m._learned_notes == []
        cp = m._read_state()
        assert [s for s in cp.sections() if " learned " in s] == []


class TestASavedUnitLearnsItsOwnPath:
    """The live half of B: a unit claimed onto a bay the previous occupant
    measured measures its own path and files it under its own uid."""

    def test_the_new_uid_files_its_own_measurement(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC", "EEEE"],
                         online=[False, True, False])
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA", auto_drop=True)
        unit = units["Bambu_AMS_2"]
        _tick(m, bridge, 100.0)                     # CCCC on the bay
        assert m.persist_learned(unit.name, "afc_bowden_length", 3600.0)
        for t in range(101, 113):                   # pulled, bay back to pool
            _tick(m, bridge, float(t), [False, False, False])
        _tick(m, bridge, 113.0, [False, False, True])   # EEEE takes it
        assert _bay(m, "Bambu_AMS_2")["bound"] == "EEEE"
        assert unit.learned[-1] == {}               # not CCCC's 3600
        assert m.persist_learned(unit.name, "afc_bowden_length", 3100.0)
        assert m._learned_for("CCCC") == {"afc_bowden_length": "3600.0"}
        assert m._learned_for("EEEE") == {"afc_bowden_length": "3100.0"}
        assert not m._read_state().has_section("AFC_BambuAMS Bambu_AMS_2")


# ── UNASSIGN of a uid the roster: option does not list ──────────────────────

class TestUnassignAnUnlistedUid:
    """A set roster: option is the whole roster, so a uid it does not list
    is never saved on a bay: the reply says so instead of promising it."""

    def test_with_a_pool_it_floats_for_the_session(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        cmd = _GCmd(UID="CCCC", FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert cmd.responses == [
            "AFC_BridgeBox chain1: unassigned CCCC from bay 'Bambu_AMS_2' -- "
            "lanes dropped live (learned values stay with the unit). roster: "
            "is set and does not list it, so it takes a free bay of its "
            "family for this session only and is not saved there; add "
            "boxed:CCCC to roster: and RESTART to give it a bay of its own."]
        for t in range(101, 140):
            _tick(m, bridge, float(t))
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert "CCCC" not in m._name_map
        assert _saved_lines(m) == []

    def test_an_unlisted_ht_is_named_as_an_ht(self, tmp_path, monkeypatch):
        # Not recorded yet: the bay it held says what it is.
        bridge = _Bridge(uids=["AAAA", "", "", "", "HHHH"],
                         online=[False, False, False, False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_HT_1")["bound"] == "HHHH"
        cmd = _GCmd(UID="HHHH", FORCE=1)
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert "add ht:HHHH to roster: and RESTART" in cmd.responses[0]

    def test_without_a_pool_it_gets_no_bay_at_restart(self, tmp_path):
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        m, _p, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB")
        cmd = _GCmd(UID="AAAA")
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert cmd.responses[0].endswith(
            "(learned values stay with the unit). roster: is set and does "
            "not list it, so it gets no bay at the next restart; add "
            "boxed:AAAA to roster: to give it one.")
        _m2, p2, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB")
        assert [s for s, _w in p2.loaded
                if s.startswith("AFC_BambuAMS ")] == [
            "AFC_BambuAMS Bambu_AMS_2"]

    def test_without_a_pool_an_unlisted_ht_is_named_as_an_ht(self, tmp_path):
        # The option is not written to the file: the name kept for it says
        # what it is.
        _mk_files(tmp_path, roster="boxed:BBBB, ht:HHHH")
        m, _p, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB")
        cmd = _GCmd(UID="HHHH")
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert "add ht:HHHH to roster: to give it one." in cmd.responses[0]


# ── the chain watch around a claim ──────────────────────────────────────────

class TestTheWatchAroundAClaim:
    def test_a_failing_loaded_lane_check_stops_no_claim(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")

        def _boom():
            raise RuntimeError("no tools")
        monkeypatch.setattr(m, "_check_moved_loaded", _boom)
        for t in (100.0, 101.0, 116.0):
            _tick(m, bridge, t)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert m._name_map["CCCC"] == "Bambu_AMS_2"
        assert [x for x in m.logger.lines if "loaded-lane check" in x] == [
            "AFC_BridgeBox chain1: loaded-lane check failed (RuntimeError: "
            "no tools); the chain watch carries on."]
        assert not any("chain watch tick failed" in x
                       for x in m.logger.lines)

    def test_the_enrollment_line_survives_a_tick_that_fails(
            self, tmp_path, monkeypatch):
        # CCCC is recorded on a tick whose claim raises: the line saying so
        # comes on the next tick, which claims it, and only once.
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        real = m._claim_pool_unit
        mode = {"now": "wait"}

        def _claim(uid, model):
            if mode["now"] == "wait":
                return None
            if mode["now"] == "raise":
                raise RuntimeError("bus busy")
            return real(uid, model)
        monkeypatch.setattr(m, "_claim_pool_unit", _claim)
        _tick(m, bridge, 100.0)
        mode["now"] = "raise"
        _tick(m, bridge, 116.0)
        assert "boxed:CCCC" in m._state_get(SEC, "roster")
        assert not [x for x in m.logger.lines if "NEW unit(s)" in x]
        mode["now"] = "claim"
        _tick(m, bridge, 117.0)
        _tick(m, bridge, 118.0)
        assert [x for x in m.logger.lines if "NEW unit(s)" in x] == [
            "AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:CCCC -- "
            "recorded. A restart adds its temperature card."]
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"

    @pytest.mark.parametrize("gone", ["missing", "not-a-pool-unit"])
    def test_a_bay_with_no_pool_unit_behind_it_is_handed_back(
            self, tmp_path, monkeypatch, gone):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, units = _live_pool(tmp_path, monkeypatch, bridge,
                              file_roster="boxed:AAAA")
        if gone == "missing":
            del m.printer.objects["AFC_BambuAMS Bambu_AMS_2"]
        else:
            units["Bambu_AMS_2"].pool = False
        assert m._claim_pool_unit("CCCC", "boxed") is None
        assert _bay(m, "Bambu_AMS_2")["uid"] is None
        assert m._bay_of_uid("CCCC") is None

    def test_the_new_unit_popup_offers_only_bays_assign_takes(
            self, tmp_path, monkeypatch):
        # Bambu_AMS_2 is free but saved for the recorded ZZZZ, which ASSIGN
        # refuses; the popup offers Bambu_AMS_4 alone.
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=4)
        m._state_set({SEC: {"roster": "boxed:AAAA, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_2"
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_3")["bound"] == "CCCC"
        assert len(_new_popups(m)) == 1
        buttons = [x for x in m._test_gcode.raw if "prompt_button" in x]
        assert buttons == [
            "// action:prompt_button Bambu_AMS_4|AFC_BRIDGEBOX_ASSIGN "
            "CHAIN=chain1 UID=CCCC NAME=Bambu_AMS_4|primary"]
        with pytest.raises(Exception, match="saved for ZZZZ"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_2"))


# ── ASSIGN of a uid not recorded yet ────────────────────────────────────────

class TestAssignAnUnrecordedUid:
    """A uid not recorded yet goes by the bay it holds, or the model it
    waits with: a four-lane AMS is never pinned to a one-lane HT bay."""

    def test_a_new_ams_on_a_spare_is_refused_an_ht_bay(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        _tick(m, bridge, 100.0)
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert m._model_for_uid("CCCC") is None
        with pytest.raises(Exception, match=(
                "CCCC is an AMS unit but bay 'Bambu_AMS_HT_1' is an HT bay")):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC",
                                             NAME="Bambu_AMS_HT_1"))
        assert _bay(m, "Bambu_AMS_2")["bound"] == "CCCC"
        assert _bay(m, "Bambu_AMS_HT_1")["uid"] is None
        assert "CCCC" not in (m._state_get(SEC, "roster") or "")
        assert "CCCC" not in m._name_map

    def test_a_waiting_ams_is_refused_an_ht_bay(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=1)
        _tick(m, bridge, 100.0)
        assert m._bay_of_uid("CCCC") is None
        assert m._no_bay["CCCC"] == "boxed"
        with pytest.raises(Exception, match="is an AMS unit"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC",
                                             NAME="Bambu_AMS_HT_1"))
        assert _bay(m, "Bambu_AMS_HT_1")["uid"] is None
        assert "CCCC" not in (m._state_get(SEC, "roster") or "")


# ── command hints name the chain ────────────────────────────────────────────

class TestHintsNameTheChain:
    def test_assign_refusals_name_the_command_to_run(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA")
        with pytest.raises(Exception) as held:
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_1"))
        assert str(held.value).endswith(
            "is already assigned to AAAA -- AFC_BRIDGEBOX_UNASSIGN "
            "CHAIN=chain1 UID=AAAA first")
        m._state_set({SEC: {"roster": "boxed:AAAA, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_3"
        with pytest.raises(Exception) as saved:
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bambu_AMS_3"))
        assert str(saved.value).endswith(
            "is saved for ZZZZ -- AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
            "UID=ZZZZ or AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=ZZZZ first")

    def test_the_save_refusal_names_the_chain(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[False, True])
        m, _units = _live_pool(tmp_path, monkeypatch, bridge,
                               file_roster="boxed:AAAA", pool_ams=2)
        m._state_set({SEC: {"roster": "boxed:AAAA, boxed:ZZZZ"}})
        m._name_map["ZZZZ"] = "Bambu_AMS_2"
        for t in (100.0, 116.0):
            _tick(m, bridge, t)
        (warned,) = [x for x in m.logger.lines if "is saved for ZZZZ" in x]
        assert warned.endswith(
            "Bambu_AMS_2 is saved for ZZZZ, so CCCC is not saved on it. "
            "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=ZZZZ or "
            "AFC_BRIDGEBOX_UNASSIGN CHAIN=chain1 UID=ZZZZ, or "
            "AFC_BRIDGEBOX_ASSIGN CHAIN=chain1 UID=CCCC NAME=<other bay> "
            "moves CCCC to another bay.")
