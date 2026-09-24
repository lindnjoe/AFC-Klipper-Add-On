"""AFC_RESET_MAPPING with claimed Bambu lanes.

AFC's reset (AFC_spool._reset_mapping) gives a lane with a config map: that
T# and numbers every other lane from T0 up in unit order. A pool lane has no
map: -- its claim gives it its home T<lane number> -- so the reset numbered it
like any other lane: an HT on lane28 behind sixteen lanes came out T16, and
its saved map kept T16 across restarts. The chain master wraps the spool
object's reset so every claimed Bambu lane ends on its home T#, while AFC
numbers the other lanes around it and saves as at any reset.

These drive AFC's own _reset_mapping (and AFC.save_vars) on the chain
harness of test_AFC_bridgebox_lane_records, laid out as a printer with twelve
lanes ahead of the pool: AMS bays lane12-15, lane16-19, lane20-23, lane24-27
and an HT bay on lane28.
"""
from __future__ import annotations

import queue
import types

import pytest

from extras import AFC_BridgeBox as bb
from extras.AFC import afc as AfcCore
from extras.AFC_functions import afcFunction
from extras.AFC_spool import AFCSpool
from tests.test_AFC_bridgebox_lane_records import _chain, _consistent, _rec
from tests.test_AFC_spool import _make_gcmd, _make_spool

A, B, H = "AAAA", "BBBB", "HHHH"
HOMES = {f"lane{n}": f"T{n}" for n in (12, 13, 14, 15)}


class _Plain:
    """A lane of a non-Bambu unit, as PREP leaves it: ``config`` is its
    config map: (AFC's _map)."""

    def __init__(self, name, maps, config=()):
        self.name, self.fullname = name, f"AFC_stepper {name}"
        self.map, self._map = list(maps), list(config)
        self.current_map = self.map[0] if self.map else ""
        self.runout_lane = None
        self.sent = []

    def send_lane_data(self):
        self.sent.append(list(self.map))

    def get_status(self, eventtime=None, save_to_file=False):
        return {"name": self.name,
                "map": ", ".join(self.map) or "NONE",
                "current_map": self.current_map}


class _Printer1:
    """Twelve lanes on a non-Bambu unit (T0-T11), then the chain's pool
    bays, with AFC's save, write queue and AFC_spool."""

    def __init__(self, tmp_path, var=None, owners=None, plain=None,
                 ready=True):
        self.ch = ch = _chain(
            tmp_path, roster=f"boxed:{A}, boxed:{B}, ht:{H}", pool_ams=4,
            names="Alpha, Bravo, Charlie, Delta", lane_base=12, var=var,
            owners=owners, ready=False)
        self.afc = afc = ch.afc
        self.writes = queue.Queue()
        afc._var_write_queue = self.writes
        afc.current = None
        afc.get_bypass_state = lambda: False
        afc.save_vars = types.MethodType(AfcCore.save_vars, afc)
        afc.function.ConfigRewrite = lambda *a: None
        self.box = types.SimpleNamespace(name="Box", lanes={})
        for n in range(12):
            maps, config = (plain or {}).get(n, ([f"T{n}"], ()))
            lane = _Plain(f"lane{n}", maps, config)
            self.box.lanes[lane.name] = afc.lanes[lane.name] = lane
            for t in maps:
                afc.tool_cmds[t] = lane.name
                ch.gcode.ready_gcode_handlers[t] = afc.cmd_CHANGE_TOOL
        afc.units = {"Box": self.box}
        for pu in ch.m._pool_units:
            afc.units[pu["name"]] = ch.units[pu["name"]]
        self.spool = spool = _make_spool()
        spool.afc, spool.gcode, spool.logger = afc, ch.gcode, ch.log
        spool.function = afc.function
        afc.spool = spool
        if ready:
            ch.m._scout_ready()

    def lane(self, name):
        return self.afc.lanes.get(name) or self.ch.lanes[name]

    def maps(self, *names):
        return {n: self.lane(n).map for n in names}

    def reset(self):
        self.spool.cmd_AFC_RESET_MAPPING(_make_gcmd(RUNOUT="no"))

    def saved(self):
        """The last snapshot AFC's writer was handed."""
        last = None
        while not self.writes.empty():
            last = self.writes.get_nowait()
        assert last is not None, "nothing saved"
        return last

    def printing(self, state):
        self.ch.printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": state[0]})


def _box_is_numbered_1_to_1(p):
    for n in range(12):
        assert p.lane(f"lane{n}").map == [f"T{n}"], n


class TestTheResetPutsBambuLanesHome:
    def test_the_ht_behind_sixteen_lanes_is_back_on_t28(self, tmp_path):
        # Printer 1: the unit on lane16-19 is offline, and an earlier reset
        # left the HT on T16, which its saved map brought back.
        p = _Printer1(tmp_path, var={"Hot": {"lane28": _rec("T16")}},
                      owners=f"{A}:Alpha, {B}:Bravo, {H}:Hot")
        assert p.ch.claim(A) is not None
        assert p.ch.claim(H, "ht") is not None
        ht = p.lane("lane28")
        assert ht.map == ["T16"]
        p.reset()
        assert (ht.map, ht.current_map) == (["T28"], "T28")
        assert ht._map == []
        assert p.maps(*HOMES) == {ln: [t] for ln, t in HOMES.items()}
        _box_is_numbered_1_to_1(p)
        assert p.afc.tool_cmds["T28"] == "lane28"
        # T16 is no lane's: gone from AFC's table and from Klipper's.
        handlers = p.ch.gcode.ready_gcode_handlers
        assert "T16" not in p.afc.tool_cmds and "T16" not in handlers
        # T28 is registered to CHANGE_TOOL, under its own name.
        assert handlers["T28"] == p.afc.cmd_CHANGE_TOOL
        assert "_T28" not in handlers
        assert ht.sent[-1] == ["T28"]
        _consistent(p.afc)
        # The bay held for the offline unit is left alone.
        assert "lane16" not in p.afc.lanes
        bay = p.ch.lanes["lane16"]
        assert (bay.map, bay._map) == ([], [])
        # Saved as any reset: the HT and the AMS lanes on their home T#s.
        snap = p.saved()
        assert snap["Hot"]["lane28"]["map"] == "T28"
        assert snap["Hot"]["lane28"]["current_map"] == "T28"
        assert {ln: r["map"] for ln, r in snap["Alpha"].items()} == HOMES
        assert snap["Box"]["lane0"]["map"] == "T0"
        assert ("AFC_BridgeBox chain1: the mapping reset put the Bambu lanes "
                "on their home T#s: lane28 T16->T28.") in p.ch.log.debugs
        assert p.ch.log.warnings == []

    def test_every_bay_claimed(self, tmp_path):
        p = _Printer1(tmp_path)
        for uid, model in ((A, "boxed"), (B, "boxed"), (H, "ht")):
            assert p.ch.claim(uid, model) is not None
        p.reset()
        homes = dict(HOMES, lane28="T28",
                     **{f"lane{n}": f"T{n}" for n in (16, 17, 18, 19)})
        assert p.maps(*homes) == {ln: [t] for ln, t in homes.items()}
        _box_is_numbered_1_to_1(p)
        _consistent(p.afc)
        snap = p.saved()
        assert snap["Hot"]["lane28"]["map"] == "T28"
        assert snap["Bravo"]["lane16"]["map"] == "T16"

    def test_a_released_bay_is_left_out_and_comes_back_home(self, tmp_path):
        p = _Printer1(tmp_path)
        for uid, model in ((A, "boxed"), (B, "boxed"), (H, "ht")):
            p.ch.claim(uid, model)
        p.ch.m._release_pool_unit(B)
        assert "lane16" not in p.afc.lanes
        p.reset()
        assert p.maps("lane28", *HOMES) == dict(
            {ln: [t] for ln, t in HOMES.items()}, lane28=["T28"])
        assert p.ch.lanes["lane16"].map == []
        assert "lane16" not in p.afc.lanes
        _consistent(p.afc)
        # Its records are held for its return, on its home T#s.
        held = p.ch.m._held["Bravo"]["lanes"]
        assert held["lane16"]["map"] == "T16"
        assert p.saved()["Bravo"]["lane16"]["map"] == "T16"
        p.ch.claim(B)
        assert p.maps("lane16", "lane28") == {"lane16": ["T16"],
                                              "lane28": ["T28"]}
        _consistent(p.afc)

    def test_a_remap_on_a_bambu_lane_is_undone(self, tmp_path):
        p = _Printer1(tmp_path)
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        p.spool.cmd_SET_MAP(_make_gcmd(LANE="lane28", MAP="T3"))
        assert (p.lane("lane28").map, p.lane("lane3").map) == (["T3"],
                                                               ["T28"])
        p.reset()
        assert (p.lane("lane28").map, p.lane("lane3").map) == (["T28"],
                                                               ["T3"])
        _consistent(p.afc)

    @pytest.mark.parametrize("how", ["AFC_RESET_MAPPING",
                                     "AFC_ENABLE_MULTIPLE_MAPPING ENABLE=0"])
    def test_a_multi_mapped_bambu_lane_is_back_to_one_tool(self, tmp_path,
                                                            how):
        p = _Printer1(tmp_path)
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        p.spool.enable_multiple_mapping = True
        p.spool.cmd_SET_MAP(_make_gcmd(LANE="lane28", MAP="T2"))
        assert sorted(p.lane("lane28").map) == ["T2", "T28"]
        assert p.lane("lane2").map == []
        if how == "AFC_RESET_MAPPING":
            p.reset()
        else:
            p.spool.cmd_AFC_ENABLE_MULTIPLE_MAPPING(_make_gcmd(ENABLE=0))
        assert (p.lane("lane28").map, p.lane("lane28").current_map) == (
            ["T28"], "T28")
        assert p.lane("lane2").map == ["T2"]
        _box_is_numbered_1_to_1(p)
        _consistent(p.afc)
        assert p.saved()["Hot"]["lane28"]["map"] == "T28"


class TestAConfigMapHoldsTheHomeTool:
    def _conflict(self, tmp_path):
        # lane5's config gives it T28, the HT's home T#.
        p = _Printer1(tmp_path, plain={5: (["T28"], ["T28"])})
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        return p

    def test_the_claim_says_what_a_reset_does(self, tmp_path):
        p = self._conflict(tmp_path)
        assert p.lane("lane28").map == ["T28"]
        (warning,) = p.ch.log.warnings
        assert ("AFC_RESET_MAPPING puts lane5 back on T28 and lane28 on "
                "another T#.") in warning

    def test_the_reset_leaves_the_tool_with_the_config_lane(self, tmp_path):
        p = self._conflict(tmp_path)
        del p.ch.log.warnings[:]
        p.reset()
        ht = p.lane("lane28")
        assert p.lane("lane5").map == ["T28"]
        assert p.afc.tool_cmds["T28"] == "lane5"
        assert ht.map and ht.map != ["T28"]
        assert ht.map[0] not in HOMES.values()
        assert ht._map == []
        assert p.maps(*HOMES) == {ln: [t] for ln, t in HOMES.items()}
        _consistent(p.afc)
        assert p.saved()["Hot"]["lane28"]["map"] == ht.map[0]
        (warning,) = p.ch.log.warnings
        assert warning == (
            f"AFC_BridgeBox chain1: the mapping reset left lane28 on "
            f"{ht.map[0]}, not its home T28: map: T28 in [AFC_stepper lane5] "
            f"keeps T28 for lane5. Set that map: outside T12-T28 to have "
            f"lane28 on T28.")
        # Said once a session; AFC.log after that.
        p.reset()
        assert len(p.ch.log.warnings) == 1
        assert any("the mapping reset left lane28" in m
                   for m in p.ch.log.debugs)


class TestADeferredTake:
    def _waiting(self, tmp_path):
        # lane11 is on T28 (PREP restored it there before the HT was
        # plugged), and the HT is claimed during the print.
        p = _Printer1(tmp_path, plain={11: (["T28"], ())})
        state = ["printing"]
        p.printing(state)
        p.ch.claim(A)
        assert p.ch.claim(H, "ht") is not None
        assert p.lane("lane28").map == ["NONE"]
        assert "lane28" in p.ch.m._deferred_takes
        return p, state

    def test_a_reset_before_the_print_ends_takes_its_place(self, tmp_path):
        p, state = self._waiting(tmp_path)
        p.reset()                                  # PRINT_END
        assert p.maps("lane28", "lane11") == {"lane28": ["T28"],
                                              "lane11": ["T11"]}
        assert p.ch.m._deferred_takes == {}
        # The reset's save has the map the reset gave, not the planned one.
        assert p.saved()["Hot"]["lane28"]["map"] == "T28"
        _consistent(p.afc)
        state[0] = "complete"
        del p.ch.log.lines[:]
        p.ch.m._take_deferred_tools()
        assert p.maps("lane28", "lane11") == {"lane28": ["T28"],
                                              "lane11": ["T11"]}
        assert p.ch.log.lines == []
        with pytest.raises(queue.Empty):
            p.writes.get_nowait()

    def test_a_failed_reset_puts_the_wait_and_the_maps_back(self, tmp_path):
        p, _state = self._waiting(tmp_path)
        entry = dict(p.ch.m._deferred_takes["lane28"])

        def save_vars():
            raise RuntimeError("disk full")
        p.afc.save_vars = save_vars
        with pytest.raises(RuntimeError):
            p.reset()
        assert p.ch.m._deferred_takes == {"lane28": entry}
        for n in (12, 13, 14, 15, 28):
            assert p.lane(f"lane{n}")._map == [], n
        # The next reset that goes through takes the wait's place.
        p.afc.save_vars = types.MethodType(AfcCore.save_vars, p.afc)
        p.reset()
        assert p.lane("lane28").map == ["T28"]
        assert p.ch.m._deferred_takes == {}
        assert p.saved()["Hot"]["lane28"]["map"] == "T28"


class TestTheWrap:
    def test_it_is_set_once_per_spool_object(self, tmp_path):
        p = _Printer1(tmp_path, ready=False)
        calls = []
        real = p.spool._reset_mapping

        def counted(*a, **k):
            calls.append(a)
            return real(*a, **k)
        p.spool._reset_mapping = counted
        p.ch.m._scout_ready()
        wrapped = p.spool._reset_mapping
        assert wrapped.__wrapped__ is counted
        p.ch.m._hook_reset(p.afc)
        # A new master of the same chain (as a RESTART builds) takes the
        # old one's place on the same wrap.
        again = _chain(tmp_path / "again", roster=f"boxed:{A}, ht:{H}",
                       pool_ams=4, names="Alpha, Bravo, Charlie, Delta",
                       lane_base=12, ready=False).m
        again._hook_reset(p.afc)
        assert p.spool._reset_mapping is wrapped
        assert getattr(p.spool, bb._RESET_MASTERS) == {"chain1": again}
        p.ch.claim(H, "ht")
        p.reset()
        assert len(calls) == 1
        assert p.lane("lane28").map == ["T28"]

    def test_a_restart_wraps_the_new_spool_object_not_the_class(
            self, tmp_path):
        p = _Printer1(tmp_path)
        assert p.spool._reset_mapping.__wrapped__.__func__ is (
            AFCSpool.__dict__["_reset_mapping"])
        # RESTART: a new spool object; this module and its class stay.
        fresh = _make_spool()
        fresh.afc, fresh.gcode, fresh.logger = (p.afc, p.ch.gcode,
                                                p.ch.log)
        fresh.function = p.afc.function
        p.afc.spool = fresh
        p.ch.m._hook_reset(p.afc)
        assert fresh._reset_mapping.__wrapped__.__func__ is (
            AFCSpool.__dict__["_reset_mapping"])
        assert not hasattr(AFCSpool.__dict__["_reset_mapping"],
                           "__wrapped__")
        assert "_reset_mapping" not in vars(_make_spool())
        p.ch.claim(H, "ht")
        fresh.cmd_AFC_RESET_MAPPING(_make_gcmd(RUNOUT="no"))
        assert p.lane("lane28").map == ["T28"]

    def test_a_spool_object_without_a_reset_is_left_alone(self, tmp_path):
        p = _Printer1(tmp_path, ready=False)
        bare = types.SimpleNamespace(afc=p.afc)
        p.afc.spool = bare
        p.ch.m._scout_ready()
        assert vars(bare) == {"afc": p.afc}
        assert any("AFC has no mapping reset to wrap" in m
                   for m in p.ch.log.debugs)
        assert bb._hook_reset_mapping(None, p.ch.m) is False
        assert p.ch.claim(H, "ht") is not None
        assert p.lane("lane28").map == ["T28"]

    def test_the_maps_come_back_when_the_reset_raises(self, tmp_path):
        p = _Printer1(tmp_path, ready=False)
        seen = {}

        def boom(*a, **k):
            seen.update({n: list(p.lane(n)._map)
                         for n in ("lane12", "lane28", "lane0")})
            raise RuntimeError("reset failed")
        p.spool._reset_mapping = boom
        p.ch.m._scout_ready()
        assert p.spool._reset_mapping.__wrapped__ is boom
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        with pytest.raises(RuntimeError):
            p.reset()
        # During the call each Bambu lane had its home T# as its map:...
        assert seen == {"lane12": ["T12"], "lane28": ["T28"], "lane0": []}
        # ...and after it, the map: it had before.
        for n in (12, 13, 14, 15, 28):
            assert p.lane(f"lane{n}")._map == [], n
        assert p.lane("lane0")._map == []
        assert p.lane("lane28").map == ["T28"]


class TestAConfigMapOfMoreThanOneTool:
    """AFC's reset gives a lane with a config map: only its first T#
    (AFC_spool._reset_mapping), so only that one keeps a Bambu lane off its
    home T#."""

    def _claimed(self, tmp_path, config):
        p = _Printer1(tmp_path, plain={5: (list(config), list(config))})
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        return p

    def test_a_home_tool_after_the_first_goes_home(self, tmp_path):
        p = self._claimed(tmp_path, ["T5", "T28"])
        # The claim takes T28 and does not say a reset gives it back.
        (warning,) = p.ch.log.warnings
        assert "lane5 was mapped to it and is now T5." in warning
        assert "AFC_RESET_MAPPING" not in warning
        del p.ch.log.warnings[:]
        p.reset()
        assert p.maps("lane5", "lane28") == {"lane5": ["T5"],
                                             "lane28": ["T28"]}
        assert p.afc.tool_cmds["T28"] == "lane28"
        assert (p.ch.gcode.ready_gcode_handlers["T28"]
                == p.afc.cmd_CHANGE_TOOL)
        _box_is_numbered_1_to_1(p)
        _consistent(p.afc)
        assert p.saved()["Hot"]["lane28"]["map"] == "T28"
        assert p.ch.log.warnings == []

    def test_a_home_tool_first_in_the_map_stays_with_it(self, tmp_path):
        p = self._claimed(tmp_path, ["T28", "T5"])
        (warning,) = p.ch.log.warnings
        assert ("AFC_RESET_MAPPING puts lane5 back on T28 and lane28 on "
                "another T#.") in warning
        del p.ch.log.warnings[:]
        p.reset()
        assert p.lane("lane5").map == ["T28"]
        assert p.afc.tool_cmds["T28"] == "lane5"
        assert p.lane("lane28").map == ["T16"]
        _consistent(p.afc)
        (warning,) = p.ch.log.warnings
        assert "keeps T28 for lane5" in warning


def _macro(gcmd=None):
    """A user's own [gcode_macro], not AFC's CHANGE_TOOL."""


class TestAMacroHoldsTheHomeTool:
    """A macro other than AFC's CHANGE_TOOL on a Bambu lane's home T#. A
    claim with a saved record leaves the T# to it (see _plan_lane_map), and
    AFC's reset registers a T# without renaming what holds it, so the reset
    leaves it to the macro too."""

    @pytest.mark.parametrize("force", [False, True])
    def test_the_reset_leaves_the_lane_to_afc(self, tmp_path, force):
        p = _Printer1(tmp_path, var={"Hot": {"lane28": _rec("T16")}},
                      owners=f"{A}:Alpha, {B}:Bravo, {H}:Hot")
        p.afc.force_assign_map = force
        fn = p.afc.function
        fn._rename = types.MethodType(afcFunction._rename, fn)
        handlers = p.ch.gcode.ready_gcode_handlers
        handlers["T28"] = _macro
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        ht = p.lane("lane28")
        assert ht.map == ["T16"]
        assert handlers["T28"] is _macro
        assert p.ch.log.warnings == []
        p.reset()
        # AFC numbers it: the seventeenth lane, past the AMS's T12-T15.
        assert (ht.map, ht.current_map, ht._map) == (["T16"], "T16", [])
        assert handlers["T28"] is _macro
        assert "T28" not in p.afc.tool_cmds
        assert p.maps(*HOMES) == {ln: [t] for ln, t in HOMES.items()}
        _box_is_numbered_1_to_1(p)
        _consistent(p.afc)
        assert p.saved()["Hot"]["lane28"]["map"] == "T16"
        assert p.ch.log.warnings == [
            "AFC_BridgeBox chain1: the mapping reset left lane28 on T16, "
            "not its home T28: T28 is a macro other than AFC's CHANGE_TOOL, "
            "which AFC does not replace. Remove or rename that macro to "
            "have lane28 on T28."]
        # Said once a session; AFC.log after that.
        p.reset()
        assert len(p.ch.log.warnings) == 1
        assert any("the mapping reset left lane28" in m
                   for m in p.ch.log.debugs)

    def test_a_lane_already_on_it_stays_and_the_macro_too(self, tmp_path):
        # A claim with no record takes its home T# whatever holds it.
        p = _Printer1(tmp_path)
        handlers = p.ch.gcode.ready_gcode_handlers
        handlers["T28"] = _macro
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        ht = p.lane("lane28")
        assert ht.map == ["T28"]
        del p.ch.log.warnings[:]
        p.reset()
        # Left where the claim put it: numbering it elsewhere would have
        # AFC's reset unregister T28, the user's macro, as a T# no lane has.
        assert ht.map == ["T28"]
        assert handlers["T28"] is _macro
        assert p.ch.log.warnings == []

    def test_the_warning_says_when_the_tool_it_got_is_a_macro(self,
                                                              tmp_path):
        # lane5's config holds T28, and T11, where AFC's reset numbers the
        # HT, is a macro (lane11 is on T40).
        p = _Printer1(tmp_path, plain={5: (["T28"], ["T28"]),
                                       11: (["T40"], ())})
        p.ch.gcode.ready_gcode_handlers["T11"] = _macro
        p.ch.claim(A)
        p.ch.claim(H, "ht")
        del p.ch.log.warnings[:]
        p.reset()
        assert p.lane("lane28").map == ["T11"]
        assert p.ch.gcode.ready_gcode_handlers["T11"] is _macro
        assert any("Error trying to map lane lane28 to T11" in w
                   for w in p.ch.log.warnings)
        assert (
            "AFC_BridgeBox chain1: the mapping reset left lane28 on T11, not "
            "its home T28: map: T28 in [AFC_stepper lane5] keeps T28 for "
            "lane5. Set that map: outside T12-T28 to have lane28 on T28. T11 "
            "is a macro other than AFC's CHANGE_TOOL, so it does not select "
            "lane28: SET_MAP LANE=lane28 MAP=<T#> gives it a T# that does."
        ) in p.ch.log.warnings
