"""Releasing a pool unit while AFC records one of its lanes in a toolhead.

A release pools the unit's lanes: they leave afc.lanes and go unassigned. A
lane AFC still records as loaded to a toolhead (its tool_loaded, or an
extruder's lane_loaded naming it) would keep that record pointing at a lane AFC
no longer has, and save_vars writes the extruder's half out for the next boot.
So auto-drop keeps such a unit claimed until the record is cleared, and the
commands that release a claimed unit refuse without FORCE=1; with it, the
record is cleared the way AFC clears it before the lane is pooled.
"""
from __future__ import annotations

import types

import pytest

from extras.AFC_BridgeBox import lane_in_toolhead, unset_tool_loaded
from extras.AFC_functions import afcFunction
from extras.AFC_lane import AFCLane, AFCLaneState
from tests.test_AFC_BridgeBox import (_Bridge, _CapGCode, _FakeReactor,
                                      _GCmd, _Logger, _mk_files, _Printer)

UID = "AAAA"


class _Afc:
    """What the release and AFC's own unload helpers touch on the AFC object,
    with the real afcFunction.unset_lane_loaded behind function."""

    def __init__(self, tools, active="extruder"):
        self.lanes = {}
        self.tools = dict(tools)
        self.tool_cmds = {}
        self.current_loading = "loading"
        self.saves = []                 # the extruders' lane_loaded, per save
        self.active_spools = []
        self.spool = types.SimpleNamespace(
            set_active_spool=self.active_spools.append)
        self.activated = 0
        fn = types.SimpleNamespace(afc=self, logger=_Logger())
        fn.get_current_lane = lambda: self.tools[active].lane_loaded
        fn.get_current_lane_obj = lambda: self.lanes.get(fn.get_current_lane())
        fn.handle_activate_extruder = self._activate
        fn.unset_lane_loaded = types.MethodType(
            afcFunction.unset_lane_loaded, fn)
        self.function = fn

    def _activate(self):
        self.activated += 1

    def save_vars(self):
        self.saves.append({n: e.lane_loaded for n, e in self.tools.items()})


def _ext(name):
    return types.SimpleNamespace(name=name, lane_loaded=None, lanes={})


def _lane(name, afc, unit, ext):
    """A claimed Bambu lane: assigned, registered, spool staged in its bay."""
    lane = AFCLane.__new__(AFCLane)
    lane.name = name
    lane.fullname = f"AFC_lane {name}"
    lane.unassigned = False
    lane.afc = afc
    lane.unit_obj = unit
    lane.extruder_obj = ext
    lane.hub_obj = None
    lane.buffer_obj = None
    lane.buffer_name = None
    lane.drive_stepper = None
    lane.tool_loaded = False
    lane.loaded_to_hub = True
    lane.status = AFCLaneState.LOADED
    lane.map = [f"T{name[4:]}"]
    lane._map = list(lane.map)
    lane.current_map = lane.map[0]
    lane._load_state = True
    lane.spool_id = None
    lane._material = None
    lane.color = ""
    lane.weight = 0.
    afc.lanes[name] = lane
    afc.tool_cmds[lane.map[0]] = name
    unit.lanes[name] = lane
    ext.lanes[name] = lane
    return lane


def _load(lane, ext):
    """What set_tool_loaded leaves: both halves of the record."""
    lane.tool_loaded = True
    lane.status = AFCLaneState.TOOLED
    ext.lane_loaded = lane.name


def _claimed(tmp_path, monkeypatch, online=False, **over):
    """AAAA claimed onto its bay, offline unless ``online``, with real lanes
    behind the bay's lane names and an AFC that has one toolhead."""
    gcode = _CapGCode()
    printer = _Printer({"gcode": gcode})
    printer.get_reactor = lambda: _FakeReactor()
    opts = dict(auto_drop=True, release_grace=10.0, release_settle=5.0,
                pool_ams=1, pool_ht=0)
    opts.update(over)
    m = _mk_files(tmp_path, roster=f"boxed:{UID}", printer=printer, **opts)[0]
    m.logger = _Logger()
    bridge = _Bridge(uids=[UID], online=[online])
    from extras import AFC_BambuAMS_bridge as bridge_mod
    monkeypatch.setattr(bridge_mod, "_BRIDGES",
                        {m.serial_port: bridge}, raising=False)
    pu = next(p for p in m._pool_units if (p.get("uid") or "") == UID)
    pu["bound"] = UID
    ext = _ext("extruder")
    afc = _Afc({"extruder": ext})
    printer.objects["AFC"] = afc
    unit = types.SimpleNamespace(
        lanes={}, type="AFC_BambuAMS", released=0,
        lane_tool_unloaded=lambda lane: None,
        return_to_home=lambda: None)
    unit.release = lambda: setattr(unit, "released", unit.released + 1)
    printer.objects[f"AFC_BambuAMS {pu['name']}"] = unit
    lanes = []
    for n in pu["lanes"]:
        lanes.append(_lane(n, afc, unit, ext))
        printer.objects[f"AFC_lane {n}"] = lanes[-1]
    return types.SimpleNamespace(m=m, pu=pu, afc=afc, ext=ext, unit=unit,
                                 lanes=lanes, bridge=bridge, gcode=gcode)


def _removed_popups(c):
    return [x for x in c.gcode.raw if "action:prompt_begin AMS removed" in x]


def _held_lines(m):
    return [x for x in m.logger.lines if "keeping it claimed" in x]


def _assert_pooled_and_cleared(c, lane):
    assert c.pu.get("bound") is None
    assert lane.unassigned is True
    assert lane.name not in c.afc.lanes
    assert lane.tool_loaded is False
    assert lane.loaded_to_hub is False
    assert c.ext.lane_loaded is None
    assert c.afc.saves[-1] == {"extruder": None}   # the var file gets it too


# ── auto-drop ────────────────────────────────────────────────────────────────

class TestAutoDrop:
    def test_a_unit_with_a_lane_in_the_toolhead_stays_claimed(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        _load(c.lanes[1], c.ext)
        for t in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0):
            c.m._scout_tick(t)
        assert c.pu.get("bound") == UID
        assert c.lanes[1].unassigned is False
        assert c.lanes[1].name in c.afc.lanes
        assert c.ext.lane_loaded == c.lanes[1].name
        assert c.unit.released == 0
        assert _removed_popups(c) == []

    def test_the_hold_is_logged_once_not_every_tick(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        _load(c.lanes[0], c.ext)
        for t in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0):
            c.m._scout_tick(t)
        held = _held_lines(c.m)
        assert len(held) == 1
        assert c.lanes[0].name in held[0]

    def test_it_releases_on_the_next_tick_after_the_unload(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        _load(c.lanes[2], c.ext)
        for t in (0.0, 5.0, 10.0, 15.0):
            c.m._scout_tick(t)
        assert c.pu.get("bound") == UID
        c.afc.function.unset_lane_loaded()        # UNSET_LANE_LOADED
        c.m._scout_tick(20.0)
        assert c.pu.get("bound") is None
        assert all(ln.unassigned for ln in c.lanes)
        assert c.unit.released == 1
        assert _removed_popups(c) == [
            f"// action:prompt_begin AMS removed: {c.pu['name']}"]

    def test_the_hold_names_a_toolhead_that_is_not_active(
            self, tmp_path, monkeypatch):
        # UNSET_LANE_LOADED clears the active tool's lane only.
        c = _claimed(tmp_path, monkeypatch)
        _load(c.lanes[0], c.ext)
        other = _ext("extruder0")
        other.lane_loaded = "lane1"
        c.afc.tools = {"extruder0": other, "extruder": c.ext}
        c.afc.function.get_current_extruder = lambda: "extruder0"
        for t in (0.0, 5.0, 10.0, 15.0):
            c.m._scout_tick(t)
        (held,) = _held_lines(c.m)
        assert ("Unload it (UNSET_LANE_LOADED with extruder as the active "
                "tool if the filament is already out) and it is released on "
                "the next check.") in held

    def test_an_extruder_record_alone_holds_it(self, tmp_path, monkeypatch):
        # PREP restores lane_loaded from the var file; a pool lane's own
        # tool_loaded is not, so the extruder's half is all there may be.
        c = _claimed(tmp_path, monkeypatch)
        c.ext.lane_loaded = c.lanes[3].name
        for t in (0.0, 5.0, 10.0, 15.0):
            c.m._scout_tick(t)
        assert c.pu.get("bound") == UID

    def test_a_replug_between_absences_logs_the_next_hold_again(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        _load(c.lanes[0], c.ext)
        for t in (0.0, 5.0, 10.0, 15.0):
            c.m._scout_tick(t)
        c.bridge._online = [True]
        for t in (20.0, 25.0, 30.0):
            c.m._scout_tick(t)
        c.bridge._online = [False]
        for t in (35.0, 40.0, 45.0, 50.0):
            c.m._scout_tick(t)
        assert c.pu.get("bound") == UID
        assert len(_held_lines(c.m)) == 2

    def test_nothing_in_the_toolhead_releases_as_before(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        for t in (0.0, 5.0, 10.0, 15.0):
            c.m._scout_tick(t)
        assert c.pu.get("bound") is None
        assert _held_lines(c.m) == []
        assert c.afc.saves == []                  # nothing was cleared


# ── the commands ─────────────────────────────────────────────────────────────

class TestUnassign:
    def test_refuses_without_force_and_changes_nothing(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        lane = c.lanes[1]
        _load(lane, c.ext)
        with pytest.raises(Exception, match="loaded to the toolhead"):
            c.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=UID))
        assert c.pu.get("bound") == UID
        assert c.pu.get("uid") == UID
        assert lane.tool_loaded is True
        assert c.ext.lane_loaded == lane.name
        assert lane.name in c.afc.lanes

    def test_force_clears_the_toolhead_record_before_pooling(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        lane = c.lanes[1]
        _load(lane, c.ext)
        c.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=UID, FORCE=1))
        _assert_pooled_and_cleared(c, lane)
        assert lane.status == AFCLaneState.NONE
        # The active toolhead's lane goes through AFC's UNSET_LANE_LOADED
        # path: toolchange bookkeeping dropped and the extruder re-activated.
        assert c.afc.activated == 1
        assert c.afc.current_loading is None
        assert c.afc.active_spools == [None]
        assert any("cleared lane" in x for x in c.m.logger.lines)

    def test_force_on_a_live_unit_clears_it_too(self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch, online=True)
        lane = c.lanes[0]
        _load(lane, c.ext)
        c.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=UID, FORCE=1))
        _assert_pooled_and_cleared(c, lane)

    def test_force_on_a_lane_in_another_toolhead_saves_it_cleared(
            self, tmp_path, monkeypatch):
        # Not the active tool, so AFC's own unset_lane_loaded (which saves)
        # is not the path; only the release's save puts the cleared
        # extruder1 in the var file for PREP to read at the next boot.
        c = _claimed(tmp_path, monkeypatch)
        e1 = _ext("extruder1")
        c.afc.tools["extruder1"] = e1
        lane = c.lanes[2]
        del c.ext.lanes[lane.name]
        lane.extruder_obj = e1
        e1.lanes[lane.name] = lane
        _load(lane, e1)
        c.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=UID, FORCE=1))
        assert c.pu.get("bound") is None
        assert lane.unassigned is True
        assert lane.tool_loaded is False and lane.loaded_to_hub is False
        assert e1.lane_loaded is None
        assert c.afc.saves[-1] == {"extruder": None, "extruder1": None}
        assert c.afc.activated == 0               # the active tool untouched
        assert c.afc.current_loading == "loading"

    def test_an_offline_unit_with_nothing_loaded_needs_no_force(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        c.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=UID))
        assert c.pu.get("bound") is None


class TestBayManager:
    # Its Unassign button sends FORCE=1, which would clear the toolhead
    # record without a word; a bay holding a lane there gets none.

    def _bays(self, c):
        c.gcode.raw.clear()
        c.m.cmd_AFC_BRIDGEBOX_BAYS(_GCmd())
        return "\n".join(c.gcode.raw)

    def test_a_bay_with_a_lane_in_the_toolhead_has_no_unassign_button(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch, online=True)
        _load(c.lanes[1], c.ext)
        text = self._bays(c)
        assert f"Unassign {c.pu['name']}" not in text
        assert f"{c.lanes[1].name} in the toolhead" in text
        assert c.pu.get("bound") == UID

    def test_the_button_is_back_once_it_is_unloaded(
            self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch, online=True)
        _load(c.lanes[1], c.ext)
        c.afc.function.unset_lane_loaded()        # UNSET_LANE_LOADED
        text = self._bays(c)
        assert (f"Unassign {c.pu['name']}|AFC_BRIDGEBOX_UNASSIGN "
                f"CHAIN={c.m.name} UID={UID} FORCE=1") in text
        assert "in the toolhead" not in text


class TestForget:
    def test_refuses_before_erasing_anything(self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        _load(c.lanes[0], c.ext)
        sec = "AFC_BridgeBox chain1"
        roster = c.m._state_get(sec, "roster")
        with pytest.raises(Exception, match="loaded to the toolhead"):
            c.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=UID))
        assert c.m._state_get(sec, "roster") == roster
        assert c.pu.get("bound") == UID
        assert c.ext.lane_loaded == c.lanes[0].name

    def test_force_clears_and_forgets(self, tmp_path, monkeypatch):
        c = _claimed(tmp_path, monkeypatch)
        _load(c.lanes[0], c.ext)
        c.m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=UID, FORCE=1))
        _assert_pooled_and_cleared(c, c.lanes[0])
        assert (c.pu.get("uid") or "") != UID


class TestAssignMove:
    def _two_bays(self, tmp_path, monkeypatch):
        return _claimed(tmp_path, monkeypatch, pool_ams=2,
                        ams_names="Alpha, Bravo")

    def test_moving_a_loaded_unit_refuses_without_force(
            self, tmp_path, monkeypatch):
        c = self._two_bays(tmp_path, monkeypatch)
        _load(c.lanes[3], c.ext)
        with pytest.raises(Exception, match="loaded to the toolhead"):
            c.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=UID, NAME="Bravo"))
        assert c.pu.get("bound") == UID
        assert c.pu.get("uid") == UID
        bravo = next(p for p in c.m._pool_units if p["name"] == "Bravo")
        assert bravo.get("uid") is None

    def test_force_clears_it_and_moves(self, tmp_path, monkeypatch):
        c = self._two_bays(tmp_path, monkeypatch)
        _load(c.lanes[3], c.ext)
        c.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=UID, NAME="Bravo", FORCE=1))
        _assert_pooled_and_cleared(c, c.lanes[3])
        bravo = next(p for p in c.m._pool_units if p["name"] == "Bravo")
        assert bravo.get("uid") == UID


# ── the clear itself ─────────────────────────────────────────────────────────

class TestUnsetToolLoaded:
    def _setup(self):
        e0, e1 = _ext("extruder"), _ext("extruder1")
        afc = _Afc({"extruder": e0, "extruder1": e1})
        unit = types.SimpleNamespace(lanes={}, type="AFC_BambuAMS",
                                     lane_tool_unloaded=lambda lane: None,
                                     return_to_home=lambda: None)
        return afc, unit, e0, e1

    def test_a_lane_in_another_toolhead_skips_the_toolchange_bookkeeping(self):
        afc, unit, e0, e1 = self._setup()
        mine = _lane("lane24", afc, unit, e1)
        active = _lane("lane25", afc, unit, e0)
        _load(mine, e1)
        _load(active, e0)
        assert unset_tool_loaded(mine, afc) is True
        assert mine.tool_loaded is False and mine.loaded_to_hub is False
        assert e1.lane_loaded is None
        assert e0.lane_loaded == "lane25"         # the active tool untouched
        assert afc.active_spools == [] and afc.activated == 0
        assert afc.current_loading == "loading"

    def test_an_extruder_naming_another_lane_is_left_alone(self):
        afc, unit, e0, _e1 = self._setup()
        mine = _lane("lane24", afc, unit, e0)
        other = _lane("lane25", afc, unit, e0)
        _load(other, e0)
        mine.tool_loaded = True                   # a stale flag on its own
        assert unset_tool_loaded(mine, afc) is True
        assert mine.tool_loaded is False
        assert e0.lane_loaded == "lane25"
        assert other.tool_loaded is True

    def test_a_lane_not_in_a_toolhead_is_untouched(self):
        afc, unit, e0, _e1 = self._setup()
        lane = _lane("lane24", afc, unit, e0)
        assert lane_in_toolhead(lane, afc) is False
        assert unset_tool_loaded(lane, afc) is False
        assert lane.loaded_to_hub is True         # staged spool left as it is
        assert lane.status == AFCLaneState.LOADED
