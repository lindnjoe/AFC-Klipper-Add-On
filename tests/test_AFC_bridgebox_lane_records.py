"""Lane records and T# maps for units claimed onto pool bays.

A pool bay's lanes are pooled while PREP restores AFC's lanes, so PREP gives
them nothing. AFC_BridgeBox holds what AFC.var.unit saved for each bay (read
at klippy:ready, and taken again at every release), names the unit it was
saved under (bay_owner), and hands it only to that unit claiming that bay:
the T# maps at the claim, the spool details at the unit's scan priming, for a
bay with no tag. A tagged bay keeps what its tag says.
"""
from __future__ import annotations

import builtins
import json
import types

import pytest

from extras import AFC_BridgeBox as bb
from extras.AFC_BambuAMS import afcBambuAMS
from extras.AFC_functions import afcFunction
from tests.test_AFC_BambuAMS_rfid import _measuring_unit, _reclaim
from tests.test_AFC_BridgeBox import (_Bridge, _GCmd, _mk_files,
                                      _mk_recorded, _Printer)
from tests.test_AFC_bridgebox_two_chains import _fc, _master

SEC = "AFC_BridgeBox chain1"
A, B, C, D, E, H, J = ("AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "HHHH",
                       "JJJJ")


class _Log:
    def __init__(self):
        self.lines, self.warnings, self.debugs = [], [], []

    def info(self, msg):
        self.lines.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)

    error = warning

    def debug(self, msg, only_debug=False, traceback=None):
        self.debugs.append(msg)


class _Clock:
    NEVER = float("inf")

    def __init__(self):
        self.now = 0.0
        self.callbacks = []

    def monotonic(self):
        return self.now

    def register_callback(self, cb, when=None):
        self.callbacks.append((cb, when))

    def register_timer(self, cb, when=None):
        return object()


class _GCode:
    """Klipper's command table as T# registration uses it: a second
    registration of a live command is refused, None unregisters."""

    def __init__(self):
        self.ready_gcode_handlers = {}
        self.raw = []

    def register_command(self, cmd, func, desc=None):
        if func is None:
            return self.ready_gcode_handlers.pop(cmd, None)
        if cmd in self.ready_gcode_handlers:
            raise Exception(f"gcode command {cmd} already registered")
        self.ready_gcode_handlers[cmd] = func

    def register_mux_command(self, *a, **k):
        pass

    def respond_raw(self, msg):
        self.raw.append(msg)

    respond_info = respond_raw


class _Afc:
    """The AFC object, with AFC's own TcmdAssign and register_tool_macro."""

    cmd_CHANGE_TOOL_help = "change tool"

    def cmd_CHANGE_TOOL(self, gcmd):
        pass

    def __init__(self, tmp_path=None, gcode=None, logger=None):
        self.lanes, self.tool_cmds, self.tools = {}, {}, {}
        self.gcode = gcode or _GCode()
        self.logger = logger or _Log()
        self.VarFile = str(tmp_path / "AFC.var") if tmp_path else "/nonexist"
        self.prep_done = True
        self.force_assign_map = False
        self.spoolman = None
        self.bound = []                   # set_spoolID calls
        self.spool = types.SimpleNamespace(
            set_spoolID=lambda ln, sid, save_vars=True: self.bound.append(
                (ln.name, sid, save_vars)))
        self.master = None
        self.saves = []                   # bay_owner as each save found it
        fn = types.SimpleNamespace(afc=self, logger=self.logger)
        fn.register_tool_macro = types.MethodType(
            afcFunction.register_tool_macro, fn)
        fn.TcmdAssign = types.MethodType(afcFunction.TcmdAssign, fn)
        self.function = fn

    def save_vars(self):
        self.saves.append(self.master._state_get(SEC, "bay_owner")
                          if self.master is not None else None)


class _Lane:
    """A pooled Bambu lane: what activation, the claim's T# work, a release
    and save_vars read and write on it."""

    def __init__(self, name, afc, unit):
        self.name, self.fullname = name, f"AFC_lane {name}"
        self.afc, self.unit_obj = afc, unit
        self.unassigned = True
        self.map, self._map, self.current_map = [], [], ""
        self.hub_obj = self.extruder_obj = self.buffer_obj = None
        self.buffer_name = None
        self.spool_id, self._material, self.color, self.weight = (
            None, None, "", 0.)
        self.sub_type = ""
        self.extruder_temp = self.bed_temp = None
        self.tool_loaded, self.need_purge = False, False
        self.runout_lane, self.td1_data = None, {}
        self._load_state = False
        self._afc_prep_done = False
        self.sent = []
        self.synced = 0

    material = property(lambda s: s._material or "",
                        lambda s, v: setattr(s, "_material", v))

    def is_direct_hub(self):
        return False

    def set_afc_prep_done(self):
        self._afc_prep_done = True

    def send_lane_data(self):
        self.sent.append(list(self.map))

    def sync_to_extruder(self):
        self.synced += 1

    def get_status(self, eventtime=None, save_to_file=False):
        return {"name": self.name,
                "map": ", ".join(m for m in self.map if m != "NONE")
                or "NONE",
                "current_map": self.current_map,
                "spool_id": self.spool_id, "material": self.material,
                "color": self.color, "weight": self.weight,
                "sub_type": self.sub_type,
                "extruder_temp": self.extruder_temp,
                "bed_temp": self.bed_temp,
                "tool_loaded": self.tool_loaded,
                "runout_lane": self.runout_lane,
                "td1_data": self.td1_data,
                "need_purge": self.need_purge}


class _Unit:
    """A pool unit as the master drives it. The record handover, release,
    scan priming and the untagged-bay pass are the unit's own; claim() is
    reduced to what the master sees of it."""

    type = "AFC_BambuAMS"
    SLOTS_PER_UNIT = 4
    _CLAIM_RESTORE_FIELDS = afcBambuAMS._CLAIM_RESTORE_FIELDS
    _persisted_lane = afcBambuAMS._persisted_lane
    _restore_claimed_lane_vars = afcBambuAMS._restore_claimed_lane_vars
    _restore_untagged_defaults = afcBambuAMS._restore_untagged_defaults
    _prep_claimed_lanes = afcBambuAMS._prep_claimed_lanes
    _prime_scan_baseline = afcBambuAMS._prime_scan_baseline
    _lane_for_slot = afcBambuAMS._lane_for_slot
    _save_lane_vars = afcBambuAMS._save_lane_vars
    _boot_hold = afcBambuAMS._boot_hold
    _resolve_held_lanes = afcBambuAMS._resolve_held_lanes
    _settle_restored_bay = afcBambuAMS._settle_restored_bay
    _clear_lane_filament = afcBambuAMS._clear_lane_filament
    _unbind_spool = afcBambuAMS._unbind_spool

    def __init__(self, name, lane_names, events, afc, logger):
        self.name, self.events, self.afc, self.logger = (name, events, afc,
                                                         logger)
        self.lane_names = list(lane_names)
        self.lanes = {}
        self.pool, self.unit_uid, self.claim_ok = True, None, True
        self._held_lanes = None
        self.ams_model, self.has_heater = "boxed", False
        self.dry_max_temp, self.measure_on_insert = 65, False
        self.finalized = []

    def set_master(self, master):
        # Where the real unit keeps it (afcBambuAMS.set_master).
        self.master = self._master = master

    def hold_lanes(self, records):
        self.events.append(("hold", self.name,
                            {k: dict(v) for k, v in (records or {}).items()}))
        afcBambuAMS.hold_lanes(self, records)

    def claim(self, uid, model):
        self.events.append(("claim", self.name, uid))
        if not self.claim_ok:
            return False
        self.unit_uid, self.pool = uid, False
        self._slot_map = {n: i for i, n in enumerate(self.lane_names)}
        afcBambuAMS._reset_lookup_state(self)
        self._afc_owned = set()
        afcBambuAMS._apply_held_lanes(self)
        return True

    def release(self):
        self.events.append(("release", self.name))
        afcBambuAMS.release(self)

    # What scan priming finds once the bridge has polled the unit.
    def bays(self, *infos):
        self._slots = [dict(i, index=n) for n, i in enumerate(infos)]
        self.unit_slots = len(infos)
        self._prev_present = [bool(i.get("present")) for i in infos]
        self._auto_scanned = [True] * len(infos)
        self._untagged_rearmed = [False] * len(infos)
        self._afc_owned, self._presence_ok = set(), True
        self._prep_seen = False

    def _finalize_scan(self, slot, scanned=True, no_record=False):
        self.finalized.append((slot, scanned, no_record))

    def _surface_slot_info(self, lane, info):
        lane.material = info.get("material")

    def _print_claimed_prep(self):
        pass

    def _reconcile_empty_bays(self):
        pass


def _other(afc, name, maps):
    """A non-Bambu lane holding T#s, registered the way PREP leaves it."""
    lane = types.SimpleNamespace(name=name, fullname=f"AFC_stepper {name}",
                                 map=list(maps), _map=[],
                                 current_map=maps[0] if maps else "",
                                 sent=[])
    lane.send_lane_data = lambda: lane.sent.append(list(lane.map))
    afc.lanes[name] = lane
    for t in maps:
        afc.tool_cmds[t] = name
        afc.gcode.ready_gcode_handlers[t] = afc.cmd_CHANGE_TOOL
    return lane


class _Chain:
    """A chain master with its pool bays built on stand-ins."""

    def __init__(self, tmp_path, roster, pool_ams, names, var, owners,
                 ready, **over):
        tmp_path.mkdir(parents=True, exist_ok=True)
        self.tmp = tmp_path
        self.gcode, self.log, self.clock = _GCode(), _Log(), _Clock()
        self.printer = _Printer({"gcode": self.gcode})
        self.printer.get_reactor = lambda: self.clock
        option = over.pop("option", None)
        kw = dict(pool_ams=pool_ams, pool_ht=1, ams_names=names,
                  ht_names="Hot", printer=self.printer, **over)
        if option is None:
            self.m = _mk_recorded(tmp_path, roster, **kw)[0]
        else:
            # roster: is the option; ``roster`` is what the state recorded.
            (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(
                "\n".join(e.strip() for e in roster.split(",")) + "\n")
            self.m = _mk_files(tmp_path, roster=option, **kw)[0]
        self.afc = _Afc(tmp_path, self.gcode, self.log)
        self.afc.master = self.m
        self.printer.objects["AFC"] = self.afc
        self.events, self.units, self.lanes = [], {}, {}
        for pu in self.m._pool_units:
            unit = _Unit(pu["name"], pu["lanes"], self.events, self.afc,
                         self.log)
            self.units[pu["name"]] = unit
            self.printer.objects["AFC_BambuAMS " + pu["name"]] = unit
            for ln in pu["lanes"]:
                self.lanes[ln] = _Lane(ln, self.afc, unit)
                self.printer.objects["AFC_lane " + ln] = self.lanes[ln]
        self.m.logger = self.log
        if owners is not None:
            self.m._state_set({SEC: {"bay_owner": owners}})
        if var is not None:
            self.write_var(var)
        if ready:
            self.m._scout_ready()

    def write_var(self, data):
        (self.tmp / "AFC.var.unit").write_text(
            data if isinstance(data, str) else json.dumps(data))

    def bay(self, name):
        return next(p for p in self.m._pool_units if p["name"] == name)

    def claim(self, uid, model="boxed"):
        return self.m._claim_pool_unit(uid, model)

    def holds(self, bay):
        return [e[2] for e in self.events if e[:2] == ("hold", bay)]

    def online(self, monkeypatch, uids, online):
        bridge = _Bridge(uids=uids, online=online)
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {self.m.serial_port: bridge}, raising=False)
        return bridge


def _chain(tmp_path, roster="boxed:AAAA", pool_ams=3,
           names="Alpha, Bravo, Charlie", var=None, owners=None, ready=True,
           **over):
    return _Chain(tmp_path, roster, pool_ams, names, var, owners, ready,
                  **over)


def _rec(maps="T24", current=None, **fields):
    rec = {"map": maps, "current_map": current if current is not None
           else maps.split(",")[0].strip()}
    rec.update(fields)
    return rec


def _consistent(afc):
    """Each T# is on at most one live lane's map, AFC's tool table names that
    lane, and the command is registered; the table names nothing else."""
    owner = {}
    for lane in afc.lanes.values():
        for t in lane.map or []:
            if t == "NONE":
                continue
            assert t not in owner, f"{t} on {owner[t]} and {lane.name}"
            owner[t] = lane.name
            assert afc.tool_cmds.get(t) == lane.name, t
            assert afc.gcode.ready_gcode_handlers.get(t) is not None, t
    assert afc.tool_cmds == owner


class TestPlanLaneMap:
    def _plan(self, afc, rec, lname="lane12", home="T12"):
        warns = []
        return bb._plan_lane_map(afc, lname, home, rec, warns.append), warns

    def test_no_record_or_no_map_takes_the_home_tool(self):
        afc = _Afc()
        assert self._plan(afc, {}) == ((["T12"], "T12", True), [])
        assert self._plan(afc, {"spool_id": 5}) == ((["T12"], "T12", True), [])

    def test_a_free_saved_tool_comes_back_and_home_is_not_taken(self):
        assert self._plan(_Afc(), _rec("T3")) == ((["T3"], "T3", False), [])

    def test_a_tool_another_lane_holds_is_dropped(self):
        afc = _Afc()
        _other(afc, "lane5", ["T3"])
        assert self._plan(afc, _rec("T3")) == (
            (["T12"], "T12", False),
            ["lane12: saved T3 is held by lane5 -- not restored; lane12 is "
             "back on T12"])

    def test_the_home_tool_of_the_bambu_lane_holding_it_is_a_note(self):
        # The holder's home wins and the lane is back on its own: nothing
        # for the user to do, so AFC.log only.
        afc = _Afc()
        _other(afc, "lane28", ["T28"])
        warns, notes = [], []
        plan = bb._plan_lane_map(afc, "lane12", "T12", _rec("T28"),
                                 warns.append, notes.append, {"lane28"})
        assert plan == (["T12"], "T12", False) and warns == []
        assert notes == ["lane12: saved T28 is held by lane28 -- not "
                         "restored; lane12 is back on T12"]

    @pytest.mark.parametrize("holder,maps,bambu,home_held", [
        ("lane28", ["T3"], {"lane28"}, False),     # not its home tool
        ("lane28", ["T28"], set(), False),         # not a Bambu lane
        ("lane28", ["T28"], {"lane28"}, True)])    # lane left with no T#
    def test_anything_else_still_warns(self, holder, maps, bambu, home_held):
        afc = _Afc()
        _other(afc, holder, maps)
        if home_held:
            _other(afc, "lane6", ["T12"])
        warns, notes = [], []
        bb._plan_lane_map(afc, "lane12", "T12", _rec(maps[0]),
                          warns.append, notes.append, bambu)
        assert notes == [] and warns

    def test_a_multi_map_keeping_its_home_tool_is_a_note(self):
        afc = _Afc()
        _other(afc, "lane28", ["T28"])
        warns, notes = [], []
        plan = bb._plan_lane_map(afc, "lane12", "T12", _rec("T12, T28"),
                                 warns.append, notes.append, {"lane28"})
        assert plan == (["T12"], "T12", True) and warns == []
        assert notes == ["lane12: saved T28 is held by lane28 -- not "
                         "restored"]
        # A non-Bambu holder still warns.
        warns, notes = [], []
        bb._plan_lane_map(afc, "lane12", "T12", _rec("T12, T28"),
                          warns.append, notes.append, set())
        assert warns and notes == []

    def test_with_the_home_tool_held_too_the_lane_gets_none(self):
        afc = _Afc()
        _other(afc, "lane5", ["T3"])
        _other(afc, "lane6", ["T12"])
        assert self._plan(afc, _rec("T3")) == (
            (["NONE"], "", False),
            ["lane12: saved T3 is held by lane5 -- not restored",
             "lane12 has no T# -- its home T12 is held by lane6; use SET_MAP"])

    def test_a_left_over_table_entry_holds_nothing(self):
        afc = _Afc()
        afc.tool_cmds["T3"] = "lane5"                  # lane5 is not live
        assert self._plan(afc, _rec("T3"))[0] == (["T3"], "T3", False)
        _other(afc, "lane5", ["T7"])
        afc.tool_cmds["T3"] = "lane5"                  # its map lacks T3
        assert self._plan(afc, _rec("T3"))[0] == (["T3"], "T3", False)

    def test_a_macro_keeps_its_tool_unless_force_assign_map(self):
        afc = _Afc()
        afc.gcode.ready_gcode_handlers["T3"] = lambda gcmd: None
        assert self._plan(afc, _rec("T3")) == (
            (["T12"], "T12", False),
            ["lane12: saved T3 is held by a macro -- not restored; lane12 is "
             "back on T12"])
        afc.force_assign_map = True
        assert self._plan(afc, _rec("T3")) == ((["T3"], "T3", False), [])

    def test_a_saved_none_stays_none_whether_home_is_free_or_held(self):
        # As PREP restores an AFC lane's NONE: the owner took its last T#
        # away, and only SET_MAP gives it one.
        afc = _Afc()
        assert self._plan(afc, _rec("NONE", "")) == (
            (["NONE"], "", False), [])
        _other(afc, "lane6", ["T12"])
        assert self._plan(afc, _rec("NONE", "")) == (
            (["NONE"], "", False), [])

    def test_a_blank_saved_map_is_home_while_free_and_none_while_held(self):
        afc = _Afc()
        assert self._plan(afc, _rec("", ""))[0] == (["T12"], "T12", False)
        _other(afc, "lane6", ["T12"])
        assert self._plan(afc, _rec("", "")) == ((["NONE"], "", False), [])

    def test_a_multi_map_keeps_its_current_tool(self):
        plan, warns = self._plan(_Afc(), _rec("T12, T40", "T40"))
        assert plan == (["T12", "T40"], "T40", True) and warns == []

    def test_a_current_tool_that_was_dropped_falls_to_the_first_kept(self):
        afc = _Afc()
        _other(afc, "lane5", ["T40"])
        plan, _w = self._plan(afc, _rec("T3, T40", "T40"))
        assert plan == (["T3"], "T3", False)

    def test_a_saved_map_reads_as_prep_reads_it(self):
        assert bb._parse_map(" T3 ,T3, none,T40") == ["T3", "T40"]
        assert bb._parse_map(["T3", "NONE"]) == ["T3"]
        assert bb._parse_map(None) == []


class TestBootCapture:
    VAR = {"Alpha": {"lane24": _rec("T24", spool_id=159, material="PLA"),
                     "lane25": {}}}

    def test_a_bays_records_are_held_for_its_owner_and_nothing_is_written(
            self, tmp_path):
        ch = _chain(tmp_path, var=self.VAR, owners="AAAA:Alpha", ready=False)
        # The band this start built is recorded once, when it changes.
        ch.m._state_set({SEC: {"ams_band": ch.m._ams_band}})
        state = (tmp_path / "AFC_BridgeBox.cfg").read_bytes()
        ch.m._scout_ready()
        assert ch.m._held == {"Alpha": {"uid": A, "lanes": {
            "lane24": self.VAR["Alpha"]["lane24"]}}}
        assert (tmp_path / "AFC_BridgeBox.cfg").read_bytes() == state

    def test_the_owner_claiming_after_prep_gets_them_before_it_claims(
            self, tmp_path):
        ch = _chain(tmp_path, var=self.VAR, owners="AAAA:Alpha")
        ch.write_var({"Alpha": {}})               # PREP's first save
        assert ch.claim(A) is ch.units["Alpha"]
        hold = ch.events.index(
            ("hold", "Alpha", {"lane24": self.VAR["Alpha"]["lane24"]}))
        assert hold < ch.events.index(("claim", "Alpha", A))

    def test_another_unit_on_that_bay_gets_nothing(self, tmp_path):
        ch = _chain(tmp_path, var=self.VAR, owners="AAAA:Alpha")
        ch.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=A))
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=C, NAME="Alpha"))
        assert ch.claim(C) is ch.units["Alpha"]
        assert ch.holds("Alpha") == [{}]
        assert ch.lanes["lane24"].map == ["T24"]

    @pytest.mark.parametrize("owners", ["CCCC:Charlie", ""],
                             ids=["other-bay", "emptied"])
    def test_a_bay_the_owner_key_does_not_name_holds_nothing(self, tmp_path,
                                                             owners):
        # AAAA was pinned to Alpha before this boot, but a key that is there
        # at all (emptied by FORGET, too) says who owns what.
        _chain(tmp_path, ready=False)                     # pins AAAA:Alpha
        ch = _chain(tmp_path, var=self.VAR, owners=owners)
        assert ch.m._pins_at_boot == {A: "Alpha"}
        assert ch.m._held == {}

    def test_records_are_taken_before_the_chain_watch_starts(self, tmp_path,
                                                            monkeypatch):
        ch = _chain(tmp_path, ready=False)
        order = []
        monkeypatch.setattr(ch.m, "_capture_boot_records",
                            lambda: order.append("capture"))
        ch.clock.register_timer = lambda cb, when=None: order.append("watch")
        ch.m._scout_ready()
        assert order == ["capture", "watch"]

    def test_state_from_before_the_owner_key_goes_to_the_unit_on_the_bay(
            self, tmp_path):
        # BBBB was claimed onto Bravo live in the last session, which saved
        # no name for it: it draws Bravo again now, as that build did.
        _chain(tmp_path, ready=False)                     # pins AAAA:Alpha
        var = dict(self.VAR, Bravo={"lane28": _rec("T28", spool_id=7)})
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB", var=var)
        assert ch.bay("Bravo")["uid"] == B                # named this boot
        assert {b: e["uid"] for b, e in ch.m._held.items()} == {
            "Alpha": A, "Bravo": B}
        assert ch.claim(B) is ch.units["Bravo"]
        assert ch.holds("Bravo") == [{"lane28": var["Bravo"]["lane28"]}]

    def test_a_bay_another_unit_was_named_for_is_not_guessed(self, tmp_path):
        first = _chain(tmp_path, ready=False)             # pins AAAA:Alpha
        first.m._state_set({SEC: {
            "name_map": "AAAA:Alpha, ZZZZ:Bravo",
            "lane_map": "AAAA:24:4, ZZZZ:28:4"}})
        var = dict(self.VAR, Bravo={"lane28": _rec("T28", spool_id=7)})
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB", var=var)
        assert ch.bay("Bravo")["uid"] == B
        assert set(ch.m._held) == {"Alpha"}

    def test_a_unit_that_wore_another_name_is_not_guessed(self, tmp_path):
        # BBBB was saved as Charlie, which the config no longer builds.
        first = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB",
                       ready=False)
        assert first.m._name_map == {A: "Alpha", B: "Bravo"}
        first.m._state_set({SEC: {"name_map": "AAAA:Alpha, BBBB:Charlie"}})
        var = dict(self.VAR, Bravo={"lane28": _rec("T28", spool_id=7)})
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB", pool_ams=2,
                    names="Alpha, Bravo", var=var)
        assert ch.m._pins_at_boot == {A: "Alpha", B: "Charlie"}
        assert ch.bay("Bravo")["uid"] == B
        assert set(ch.m._held) == {"Alpha"}

    def test_an_unlisted_unit_gets_the_one_spare_it_ran_on(self, tmp_path):
        # roster: lists AAAA only; the last session ran CCCC on the Bravo
        # spare and recorded it, with no name.
        _chain(tmp_path, ready=False)                     # pins AAAA:Alpha
        var = dict(self.VAR, Bravo={"lane28": _rec("T28", spool_id=7)})
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:CCCC", var=var,
                    option="boxed:AAAA")
        assert ch.bay("Bravo")["uid"] is None
        assert {b: e["uid"] for b, e in ch.m._held.items()} == {
            "Alpha": A, "Bravo": C}
        assert ch.claim(C) is ch.units["Bravo"]
        assert ch.holds("Bravo") == [{"lane28": var["Bravo"]["lane28"]}]

    @pytest.mark.parametrize("recorded, var", [
        ("boxed:AAAA, boxed:CCCC, boxed:DDDD",
         {"Bravo": {"lane28": _rec("T28", spool_id=7)}}),
        ("boxed:AAAA, boxed:CCCC",
         {"Bravo": {"lane28": _rec("T28", spool_id=7)},
          "Charlie": {"lane32": _rec("T32", spool_id=8)}}),
    ], ids=["two-units", "two-spares"])
    def test_an_unlisted_unit_is_not_guessed_among_several(self, tmp_path,
                                                          recorded, var):
        _chain(tmp_path, ready=False)                     # pins AAAA:Alpha
        ch = _chain(tmp_path, roster=recorded, var=var, option="boxed:AAAA")
        assert ch.m._held == {}

    def test_with_the_owner_key_an_unlisted_unit_gets_nothing(self,
                                                             tmp_path):
        _chain(tmp_path, ready=False)                     # pins AAAA:Alpha
        var = dict(self.VAR, Bravo={"lane28": _rec("T28", spool_id=7)})
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:CCCC", var=var,
                    option="boxed:AAAA", owners="AAAA:Alpha")
        assert set(ch.m._held) == {"Alpha"}

    @pytest.mark.parametrize("var", [None, "{not json", "[1, 2]",
                                     '{"Alpha": "x"}'])
    def test_an_unreadable_var_file_holds_nothing(self, tmp_path, var):
        ch = _chain(tmp_path, var=var, owners="AAAA:Alpha")
        assert ch.m._held == {}

    def test_without_afc_nothing_is_held(self, tmp_path):
        ch = _chain(tmp_path, var=self.VAR, owners="AAAA:Alpha", ready=False)
        del ch.printer.objects["AFC"]
        ch.m._capture_boot_records()
        assert ch.m._held == {}

    def test_a_scout_only_chain_holds_nothing(self, tmp_path):
        m = _mk_files(tmp_path, roster="")[0]
        m.printer.objects["AFC"] = _Afc(tmp_path)
        m._capture_boot_records()
        assert m._held == {}


class TestAnHtPastALoweredAmsBand:
    """pool_ams lowered below the AMS band a recorded HT sits past: the HT
    keeps its lanes, so what its lane saved -- spool and T# map -- comes
    back with it at each boot."""

    NAMES = "Alpha, Bravo, Charlie, Delta"
    ROSTER = "boxed:AAAA, boxed:BBBB, ht:HHHH"

    def test_the_ht_keeps_its_lane_record_and_map(self, tmp_path):
        first = _chain(tmp_path, roster=self.ROSTER, pool_ams=4,
                       names=self.NAMES, ready=False)
        assert first.bay("Hot")["lanes"] == ["lane40"]
        var = {"Hot": {"lane40": _rec("T7", spool_id=5, material="PETG")}}
        for _again in range(2):
            ch = _chain(tmp_path, roster=self.ROSTER, pool_ams=2,
                        names=self.NAMES, var=var, owners="HHHH:Hot")
            assert ch.bay("Hot")["lanes"] == ["lane40"]
            assert ch.m._held["Hot"] == {"uid": H,
                                         "lanes": {"lane40": var["Hot"][
                                             "lane40"]}}
            assert ch.claim(H, "ht") is ch.units["Hot"]
            assert ch.lanes["lane40"].map == ["T7"]
            assert ch.holds("Hot")[-1] == {"lane40": var["Hot"]["lane40"]}


class TestBayOwner:
    def test_the_claim_records_the_owner_before_any_tool_is_assigned(
            self, tmp_path):
        ch = _chain(tmp_path)
        ch.claim(A)
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert ch.afc.saves and set(ch.afc.saves) == {"AAAA:Alpha"}

    def test_an_unchanged_owner_is_not_written_again(self, tmp_path,
                                                     monkeypatch):
        ch = _chain(tmp_path)
        ch.claim(A)
        ch.m._release_pool_unit(A)
        writes = []
        real = ch.m._state_set
        monkeypatch.setattr(ch.m, "_state_set",
                            lambda u: (writes.append(u), real(u)))
        ch.claim(A)
        assert not [w for w in writes if "bay_owner" in (w.get(SEC) or {})]

    def test_a_failed_claim_records_nothing_and_takes_its_records_back(
            self, tmp_path):
        ch = _chain(tmp_path)
        ch.m._held = {"Alpha": {"uid": A, "lanes": {"lane24": _rec()}}}
        ch.units["Alpha"].claim_ok = False
        assert ch.claim(A) is None
        assert ch.holds("Alpha")[0] == {"lane24": _rec()}
        assert ch.m._state_get(SEC, "bay_owner") is None
        assert ch.holds("Alpha")[-1] == {}
        assert ch.units["Alpha"]._held_lanes == {}

    def test_a_unit_is_the_owner_of_one_bay(self, tmp_path):
        ch = _chain(tmp_path)
        ch.claim(C)                                   # a spare: Bravo
        ch.m._release_pool_unit(C)
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=C, NAME="Charlie"))
        ch.claim(C)
        assert ch.m._state_get(SEC, "bay_owner") == "CCCC:Charlie"

    def test_a_unit_moved_back_gets_nothing_from_before_the_move(
            self, tmp_path):
        ch = _chain(tmp_path)
        ch.claim(A)
        ch.lanes["lane24"].spool_id = 159
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=A, NAME="Charlie"))
        ch.claim(A)
        assert "Alpha" not in ch.m._held
        ch.lanes["lane32"].spool_id = 777
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=A, NAME="Alpha"))
        ch.claim(A)
        assert ch.holds("Alpha")[-1] == {}
        assert ch.m._held == {}
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha"

    def test_the_claim_saves_the_lanes_when_no_tool_needed_assigning(
            self, tmp_path):
        ch = _chain(tmp_path)
        for n in (24, 25, 26, 27):                    # left registered
            ch.gcode.ready_gcode_handlers[f"T{n}"] = ch.afc.cmd_CHANGE_TOOL
        ch.claim(A)
        assert ch.afc.tool_cmds["T24"] == "lane24"
        assert ch.afc.saves == ["AAAA:Alpha"]

    def test_an_owner_the_state_file_missed_is_written_at_the_next_claim(
            self, tmp_path, monkeypatch):
        ch = _chain(tmp_path)
        real = ch.m._write_state
        monkeypatch.setattr(ch.m, "_write_state", lambda cp: None)
        ch.claim(A)
        assert ch.m._state_get(SEC, "bay_owner") is None
        ch.m._release_pool_unit(A)
        monkeypatch.setattr(ch.m, "_write_state", real)
        ch.claim(A)
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha"

    def test_two_chains_on_one_state_file_keep_their_own_owners(
            self, tmp_path):
        printer = _Printer()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=_fc())
        m2 = _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                     unit_prefix="Bambu_AMS_B")
        b1, b2 = m1._pool_units[0]["name"], m2._pool_units[0]["name"]
        m1._set_bay_owner(b1, A)
        m2._set_bay_owner(b2, B)
        assert m1._state_get("AFC_BridgeBox chain1", "bay_owner") == \
            f"AAAA:{b1}"
        assert m2._state_get("AFC_BridgeBox chain2", "bay_owner") == \
            f"BBBB:{b2}"
        again = _master(tmp_path, _Printer(), "chain2", fileconfig=_fc(),
                        unit_prefix="Bambu_AMS_B")
        assert again._load_bay_owner() == ({b2: B}, True)

    @pytest.mark.parametrize("live", [False, True])
    def test_forget_drops_the_owner_and_the_held_records(
            self, tmp_path, monkeypatch, live):
        ch = _chain(tmp_path)
        ch.claim(A)
        ch.claim(C)
        ch.lanes["lane24"].spool_id = 159
        if live:
            ch.online(monkeypatch, [A, C], [True, True])
        else:
            ch.m._release_pool_unit(A)
            assert "Alpha" in ch.m._held
        cmd = _GCmd(UID=A)
        ch.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert ch.m._state_get(SEC, "bay_owner") == "CCCC:Bravo"
        assert ch.m._owners() == {"Bravo": C}
        assert "Alpha" not in ch.m._held
        # No learned record was saved for AAAA, so none is said erased.
        assert "freed for reuse, saved lane records erased" in \
            cmd.responses[0]

    def test_forget_with_an_owner_entry_and_nothing_held_erases_no_records(
            self, tmp_path):
        # AAAA stayed away through a restart, so PREP wrote its bay empty:
        # bay_owner still names it, and nothing is held for it.
        _chain(tmp_path, ready=False)                     # pins AAAA:Alpha
        ch = _chain(tmp_path, var={"Alpha": {}}, owners="AAAA:Alpha")
        assert (ch.m._held, ch.m._owners()) == ({}, {"Alpha": A})
        cmd = _GCmd(UID=A)
        ch.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert cmd.responses[0].startswith(
            "AFC_BridgeBox chain1: forgot AAAA -- lanes 24-27 and the name "
            "Alpha freed for reuse -- slot freed to the pool LIVE")
        assert "records" not in cmd.responses[0]
        assert ch.m._owners() == {}
        assert A not in (ch.m._state_get(SEC, "bay_owner") or "")

    def test_forgetting_a_floating_unit_says_its_records_went(self,
                                                              tmp_path):
        ch = _chain(tmp_path)
        ch.claim(A)
        ch.lanes["lane24"].spool_id = 159
        ch.m._release_pool_unit(A)
        ch.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=A))     # no pin: floats
        cmd = _GCmd(UID=A)
        ch.m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert (ch.m._held, ch.m._owners()) == ({}, {})
        assert cmd.responses[0] == (
            "AFC_BridgeBox chain1: forgot AAAA -- saved lane records erased. "
            "Applies at the next RESTART.")

    def test_unassign_keeps_the_owner_and_the_held_records(self, tmp_path):
        ch = _chain(tmp_path)
        ch.claim(A)
        ch.lanes["lane24"].spool_id = 159
        ch.m._release_pool_unit(A)
        cmd = _GCmd(UID=A)
        ch.m.cmd_AFC_BRIDGEBOX_UNASSIGN(cmd)
        assert ch.m._state_get(SEC, "bay_owner") == "AAAA:Alpha"
        assert ch.m._held["Alpha"]["uid"] == A
        assert cmd.responses[0].endswith(
            "(learned values stay with the unit). It takes a free bay of its "
            "family, its last one first, and is saved there once it has been "
            "online 15s.")


class TestPrepGate:
    def _waiting(self, tmp_path, monkeypatch):
        ch = _chain(tmp_path)
        ch.afc.prep_done = False
        ch.online(monkeypatch, [A, C], [True, True])
        return ch

    def _tick(self, ch, t):
        ch.clock.now = t
        ch.m._scout_tick(t)

    def test_nothing_is_claimed_or_adopted_before_prep(self, tmp_path,
                                                       monkeypatch):
        ch = self._waiting(tmp_path, monkeypatch)
        self._tick(ch, 10.0)
        self._tick(ch, 89.0)
        assert [p["bound"] for p in ch.m._pool_units] == [None] * 4
        assert ch.bay("Bravo")["uid"] is None
        assert ch.events == [] and ch.log.warnings == []

    def test_units_claim_once_prep_has_run(self, tmp_path, monkeypatch):
        ch = self._waiting(tmp_path, monkeypatch)
        self._tick(ch, 10.0)
        ch.afc.prep_done = True
        self._tick(ch, 13.0)
        assert ch.bay("Alpha")["bound"] == A
        assert ch.bay("Bravo")["bound"] == C

    def test_a_prep_that_never_finishes_holds_claims_for_90s_only(
            self, tmp_path, monkeypatch):
        ch = self._waiting(tmp_path, monkeypatch)
        self._tick(ch, 90.0)
        self._tick(ch, 93.0)
        assert ch.bay("Alpha")["bound"] == A
        assert ch.log.warnings == [
            "AFC_BridgeBox chain1: PREP has not finished 90s after startup; "
            "claiming units anyway."]

    def test_a_long_moonraker_timeout_holds_claims_longer(self, tmp_path,
                                                           monkeypatch):
        ch = _chain(tmp_path, ready=False)
        ch.afc.moonraker_connect_to = "75"
        ch.m._scout_ready()
        ch.afc.prep_done = False
        ch.online(monkeypatch, [A, C], [True, True])
        self._tick(ch, 100.0)
        self._tick(ch, 134.0)
        assert ch.bay("Alpha")["bound"] is None
        self._tick(ch, 135.0)
        self._tick(ch, 138.0)
        assert ch.bay("Alpha")["bound"] == A
        assert ch.log.warnings == [
            "AFC_BridgeBox chain1: PREP has not finished 135s after startup; "
            "claiming units anyway."]

    # Past the wait, AFC.var.unit still holds what was saved while the bay's
    # last owner was claimed: save_vars writes nothing until PREP has run.
    BRAVO = {"Bravo": {"lane28": _rec("T28", spool_id=159, material="PLA")}}

    def _claimed_before_prep(self, tmp_path):
        ch = _chain(tmp_path, var=self.BRAVO, owners="EEEE:Bravo")
        ch.afc.prep_done = False
        ch.clock.now = 91.0
        assert ch.claim(D) is ch.units["Bravo"]
        assert ch.m._owners() == {"Bravo": D}
        return ch

    def test_a_claim_before_prep_leaves_the_saved_owner_alone(self,
                                                               tmp_path):
        ch = self._claimed_before_prep(tmp_path)
        assert ch.m._state_get(SEC, "bay_owner") == "EEEE:Bravo"
        again = _chain(tmp_path)                     # a restart, PREP unrun
        assert again.m._held == {"Bravo": {"uid": E,
                                           "lanes": self.BRAVO["Bravo"]}}
        again.claim(D)
        assert again.holds("Bravo") == [{}]
        assert again.lanes["lane28"].map == ["T28"]

    def test_the_owner_is_written_once_prep_has_run(self, tmp_path):
        ch = self._claimed_before_prep(tmp_path)
        ch.m._scout_tick(92.0)
        assert ch.m._state_get(SEC, "bay_owner") == "EEEE:Bravo"
        ch.afc.prep_done = True
        ch.m._scout_tick(95.0)
        assert ch.m._state_get(SEC, "bay_owner") == "DDDD:Bravo"
        assert ch.m._bay_owner_pending is False

    def test_an_afc_without_a_prep_flag_or_a_chain_not_ready_is_settled(
            self, tmp_path):
        ch = _chain(tmp_path)
        del ch.afc.prep_done
        assert ch.m._prep_settled() is True
        ch.afc.prep_done = False
        ch.m._ready_at = None
        assert ch.m._prep_settled() is True

    def test_assign_before_prep_pins_and_says_it_waits(self, tmp_path,
                                                       monkeypatch):
        ch = self._waiting(tmp_path, monkeypatch)
        ch.online(monkeypatch, [A, C], [False, True])
        cmd = _GCmd(UID=C, NAME="Charlie")
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert ch.bay("Charlie")["uid"] == C
        assert ch.bay("Charlie")["bound"] is None
        assert cmd.responses[0].endswith(
            " -- pinned; it claims this bay once PREP finishes.")

    def test_assign_before_prep_without_a_pool_says_to_run_it_again(
            self, tmp_path, monkeypatch):
        ch = self._waiting(tmp_path, monkeypatch)
        ch.m.pool_ams = ch.m.pool_ht = 0              # nothing claims later
        ch.online(monkeypatch, [A, C], [False, True])
        cmd = _GCmd(UID=C, NAME="Charlie")
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert ch.bay("Charlie")["bound"] is None
        assert cmd.responses[0].endswith(
            " -- pinned; PREP has not finished, run this again once it has.")


class TestReleaseSnapshot:
    def _loaded(self, tmp_path, **kw):
        ch = _chain(tmp_path, **kw)
        ch.claim(A)
        lane = ch.lanes["lane24"]
        lane.spool_id, lane.material, lane.color = 159, "PLA", "#0086D6"
        return ch

    def test_a_release_holds_the_lanes_records_with_their_maps(
            self, tmp_path):
        ch = self._loaded(tmp_path)
        ch.m._release_pool_unit(A)
        held = ch.m._held["Alpha"]
        assert held["uid"] == A
        assert held["lanes"]["lane24"]["spool_id"] == 159
        assert (held["lanes"]["lane24"]["map"],
                held["lanes"]["lane24"]["current_map"]) == ("T24", "T24")
        assert set(held["lanes"]) == {"lane24", "lane25", "lane26", "lane27"}
        assert ch.m.get_status()["held_bays"] == {"Alpha": A}

    def test_the_same_unit_gets_them_back_and_another_does_not(
            self, tmp_path):
        ch = self._loaded(tmp_path)
        ch.m._release_pool_unit(A)
        ch.claim(A)
        assert ch.holds("Alpha")[-1]["lane24"]["spool_id"] == 159
        ch.m._release_pool_unit(A)
        ch.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=A))
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=C, NAME="Alpha"))
        ch.claim(C)
        assert ch.holds("Alpha")[-1] == {}

    def test_spares_come_back_to_their_own_bays_with_their_records(
            self, tmp_path):
        ch = _chain(tmp_path)
        ch.claim(C)
        ch.claim(D)
        assert (ch.bay("Bravo")["bound"], ch.bay("Charlie")["bound"]) == (C, D)
        ch.lanes["lane28"].spool_id, ch.lanes["lane32"].spool_id = 11, 22
        ch.m._release_pool_unit(C)
        ch.m._release_pool_unit(D)
        ch.claim(D)                                   # Bravo is lower, free
        ch.claim(C)
        assert (ch.bay("Bravo")["bound"], ch.bay("Charlie")["bound"]) == (C, D)
        assert ch.holds("Charlie")[-1]["lane32"]["spool_id"] == 22
        assert ch.holds("Bravo")[-1]["lane28"]["spool_id"] == 11

    def test_records_the_unit_had_not_settled_are_held_again(self, tmp_path):
        var = {"Alpha": {"lane24": _rec("T24", spool_id=159,
                                        material="PETG")}}
        ch = _chain(tmp_path, var=var, owners="AAAA:Alpha")
        ch.claim(A)                              # the unit never primes
        lane = ch.lanes["lane24"]
        assert (lane.spool_id, lane.material) == (159, "PETG")
        # SET_MAP moves it before the unit drops: the claim applied the map.
        ch.afc.tool_cmds.pop("T24")
        ch.gcode.register_command("T24", None)
        lane.map, lane.current_map = ["T7"], "T7"
        ch.afc.tool_cmds["T7"] = "lane24"
        ch.gcode.register_command("T7", ch.afc.cmd_CHANGE_TOOL)
        # SET_COLOR and SET_WEIGHT before priming are the user's too.
        lane.color, lane.weight = "#123456", 640.0
        ch.m._release_pool_unit(A)
        rec = ch.m._held["Alpha"]["lanes"]["lane24"]
        assert (rec["spool_id"], rec["material"]) == (159, "PETG")
        assert (rec["map"], rec["current_map"]) == ("T7", "T7")
        assert (rec["color"], rec["weight"]) == ("#123456", 640.0)
        ch.claim(A)
        assert (lane.spool_id, lane.color, lane.weight, lane.map) == (
            159, "#123456", 640.0, ["T7"])


class TestTheVarFileKeepsHeldRecords:
    """AFC.save_vars saves each unit's registered lanes, so a pool bay no
    unit is claimed onto is saved empty. Every save passes through the chain
    master, which writes the records it holds for the bay's owner there: a
    restart before that unit is claimed again (offline all boot, released,
    or not claimed yet after PREP's first save) holds them again."""

    VAR = {"Alpha": {"lane24": _rec("T24", spool_id=159, material="PLA",
                                    color="#0086D6", weight=412.0),
                     "lane25": _rec("T25", material="PETG")}}

    def _afc_saves(self, ch):
        """AFC's own save_vars and write queue on the chain's AFC."""
        import queue

        from extras.AFC import afc as AFC
        a = ch.afc
        a._var_write_queue = queue.Queue()
        a.units = dict(ch.units)
        a.current = None
        a.get_bypass_state = lambda: False
        a.save_vars = types.MethodType(AFC.save_vars, a)

        def saved():
            item = a._var_write_queue.get_nowait()
            ch.write_var(item)                    # AFC's writer
            return item
        return saved

    def _ready(self, tmp_path, var=None, owners="AAAA:Alpha"):
        ch = _chain(tmp_path, var=self.VAR if var is None else var,
                    owners=owners, ready=False)
        saved = self._afc_saves(ch)
        ch.m._scout_ready()
        return ch, saved

    def test_preps_save_before_any_claim_keeps_them(self, tmp_path):
        ch, saved = self._ready(tmp_path)
        ch.afc.save_vars()                        # PREP's first save
        data = saved()
        assert data["Alpha"] == self.VAR["Alpha"]
        assert (data["Bravo"], data["Charlie"]) == ({}, {})
        # A restart on that file holds them again, for the same unit.
        again = _chain(tmp_path, owners="AAAA:Alpha")
        assert again.m._held["Alpha"] == {"uid": A,
                                          "lanes": self.VAR["Alpha"]}
        again.claim(A)
        assert again.lanes["lane24"].spool_id == 159

    def test_the_file_keeps_its_own_copy(self, tmp_path):
        ch, saved = self._ready(tmp_path)
        ch.afc.save_vars()
        saved()["Alpha"]["lane24"]["spool_id"] = 7
        assert ch.m._held["Alpha"]["lanes"]["lane24"]["spool_id"] == 159

    def test_a_claimed_bay_is_saved_from_its_lanes(self, tmp_path):
        ch, saved = self._ready(tmp_path)
        ch.claim(A)
        while not ch.afc._var_write_queue.empty():
            saved()
        ch.lanes["lane24"].material = "PETG"      # SET_MATERIAL
        ch.afc.save_vars()
        assert saved()["Alpha"]["lane24"]["material"] == "PETG"

    def test_a_released_bay_is_saved_with_its_records(self, tmp_path):
        ch, saved = self._ready(tmp_path)
        ch.claim(A)
        ch.lanes["lane24"].weight = 300.0
        ch.m._release_pool_unit(A)
        ch.afc.save_vars()
        data = saved()
        while not ch.afc._var_write_queue.empty():
            data = saved()
        assert data["Alpha"]["lane24"]["weight"] == 300.0
        again = _chain(tmp_path, owners="AAAA:Alpha")
        assert again.m._held["Alpha"]["lanes"]["lane24"]["weight"] == 300.0

    def test_the_release_save_of_a_toolhead_lane_keeps_them(self, tmp_path):
        # The release saves the cleared toolhead record, once the bay is
        # unbound and its records held.
        ch, saved = self._ready(tmp_path)
        ch.claim(A)
        while not ch.afc._var_write_queue.empty():
            saved()
        ch.afc.tools = {"extruder": types.SimpleNamespace(
            name="extruder", lane_loaded="lane24")}
        ch.lanes["lane24"].tool_loaded = True
        ch.m._release_pool_unit(A)
        data = saved()
        assert data["Alpha"]["lane24"]["spool_id"] == 159
        assert ch.afc._var_write_queue.empty()

    def test_a_lane_waiting_for_its_tool_is_saved_with_its_plan(
            self, tmp_path):
        # Claimed during a print, lane24 waits for T24, which lane5 holds.
        # A restart before the print ends brings it back to that plan, not
        # to a map without T24.
        ch, saved = self._ready(tmp_path)
        _other(ch.afc, "lane5", ["T24"])
        ch.printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": "printing"})
        ch.claim(A)
        assert ch.lanes["lane24"].map == ["NONE"]
        # Every save from the claim's own on, with no other due until the
        # print ends.
        snaps = []
        while not ch.afc._var_write_queue.empty():
            snaps.append(saved())
        ch.afc.save_vars()
        snaps.append(saved())
        recs = [d["Alpha"]["lane24"] for d in snaps if "lane24" in d["Alpha"]]
        assert len(recs) > 2
        assert {(r["map"], r["current_map"]) for r in recs} == {("T24", "T24")}
        assert snaps[-1]["Alpha"]["lane25"]["map"] == "T25"
        again = _chain(tmp_path, owners="AAAA:Alpha")
        lane5 = _other(again.afc, "lane5", ["T24"])
        again.claim(A)
        assert again.lanes["lane24"].map == ["T24"]
        assert lane5.map == ["T37"]
        _consistent(again.afc)

    def test_state_from_before_the_owner_key_is_kept_by_preps_save(
            self, tmp_path):
        # No bay_owner key (the first start after upgrading): the unit the
        # bay's records are guessed for (see _capture_boot_records) keeps
        # them through PREP's save as well.
        ch, saved = self._ready(tmp_path, owners=None)
        assert ch.m._held["Alpha"]["uid"] == A
        ch.afc.save_vars()
        assert saved()["Alpha"] == self.VAR["Alpha"]

    def test_a_bay_held_for_nobody_is_saved_empty(self, tmp_path):
        # bay_owner is recorded and names another bay: no unit owns Alpha.
        ch, saved = self._ready(tmp_path, owners="DDDD:Charlie")
        ch.afc.save_vars()
        assert saved()["Alpha"] == {}

    def test_the_writers_stop_passes_and_the_hook_is_set_once(
            self, tmp_path):
        ch, saved = self._ready(tmp_path)
        hook = ch.afc._var_write_queue
        ch.m._scout_ready()
        assert ch.afc._var_write_queue is hook
        stop = object()
        hook.put_nowait(stop)
        assert hook.get() is stop

    def test_an_afc_without_the_queue_is_left_alone(self, tmp_path):
        ch = _chain(tmp_path, var=self.VAR, owners="AAAA:Alpha")
        assert not hasattr(ch.afc, "_var_write_queue")
        assert any("no var-file write queue" in x for x in ch.log.debugs)


class TestSparePreference:
    def test_a_floating_unit_goes_back_to_its_last_bay(self, tmp_path):
        ch = _chain(tmp_path, owners="DDDD:Charlie")
        assert ch.claim(D) is ch.units["Charlie"]
        assert ch.bay("Bravo")["uid"] is None

    def test_a_reserved_or_other_family_bay_is_not_taken_back(self,
                                                              tmp_path):
        ch = _chain(tmp_path, owners="DDDD:Alpha, EEEE:Hot")
        assert ch.claim(D) is ch.units["Bravo"]       # Alpha is AAAA's
        assert ch.claim(E) is ch.units["Charlie"]     # Hot is an HT bay

    def test_it_claims_before_a_new_unit_seen_on_the_same_tick(
            self, tmp_path, monkeypatch):
        ch = _chain(tmp_path, owners="DDDD:Bravo")
        ch.online(monkeypatch, [A, D, E], [False, True, True])
        real = builtins.sorted

        def new_first(items, key=None, reverse=False):
            # Break every tie against the returning unit.
            return real(real(items, key=lambda x: x != E), key=key,
                        reverse=reverse)
        monkeypatch.setattr(bb, "sorted", new_first, raising=False)
        ch.m._scout_tick(0.0)
        assert (ch.bay("Bravo")["bound"], ch.bay("Charlie")["bound"]) == (D, E)

    def test_an_owner_entry_changes_no_fabricated_section(self, tmp_path):
        before = _chain(tmp_path / "a", ready=False)
        after = _chain(tmp_path / "b", ready=False)
        after.m._state_set({SEC: {"bay_owner": "DDDD:Charlie, AAAA:Alpha"}})
        rebuilt = _chain(tmp_path / "b", ready=False)
        assert rebuilt.m._roster_sections(rebuilt.m.units) == \
            before.m._roster_sections(before.m.units)


class TestMapRestoreOnClaim:
    def _swapped(self, tmp_path, **kw):
        # SET_MAP left lane24 on T3 and gave its T24 to lane5.
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T3")}},
                    owners="AAAA:Alpha", **kw)
        lane5 = _other(ch.afc, "lane5", ["T24"])
        return ch, lane5

    def test_a_saved_map_comes_back_and_the_home_tool_stays_put(
            self, tmp_path):
        ch, lane5 = self._swapped(tmp_path)
        assert ch.claim(A) is not None
        lane24 = ch.lanes["lane24"]
        assert (lane24.map, lane24.current_map) == (["T3"], "T3")
        assert ch.afc.tool_cmds["T3"] == "lane24"
        assert (lane5.map, ch.afc.tool_cmds["T24"]) == (["T24"], "lane5")
        assert [ch.lanes[f"lane{n}"].map for n in (25, 26, 27)] == [
            ["T25"], ["T26"], ["T27"]]
        assert ch.log.warnings == []
        assert ch.log.lines[-1].endswith(
            "(4 lanes) -- live, no restart. Saved maps: lane24->T3.")
        _consistent(ch.afc)

    def test_only_a_home_tool_is_taken_and_only_after_the_claim(
            self, tmp_path, monkeypatch):
        ch, lane5 = self._swapped(tmp_path)
        seen = []
        real = ch.m._take_home_tool

        def spy(afc, lane):
            seen.append((lane.name, list(lane.map), ch.units["Alpha"].pool))
            return real(afc, lane)
        monkeypatch.setattr(ch.m, "_take_home_tool", spy)
        ch.units["Alpha"].claim_ok = False
        assert ch.claim(A) is None
        assert seen == []
        assert (lane5.map, ch.afc.tool_cmds) == (["T24"], {"T24": "lane5"})
        ch.units["Alpha"].claim_ok = True
        ch.claim(A)
        assert seen == [(f"lane{n}", [f"T{n}"], False) for n in (25, 26, 27)]
        _consistent(ch.afc)

    def test_a_swap_inside_a_unit_comes_back(self, tmp_path):
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T25"),
                                             "lane25": _rec("T24")}},
                    owners="AAAA:Alpha")
        ch.claim(A)
        assert (ch.lanes["lane24"].map, ch.lanes["lane25"].map) == (
            ["T25"], ["T24"])
        assert ch.log.warnings == []
        _consistent(ch.afc)

    @pytest.mark.parametrize("order", [(A, B), (B, A)])
    def test_a_swap_across_bays_comes_back_in_either_order(self, tmp_path,
                                                           order):
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB", pool_ams=2,
                    names="Alpha, Bravo",
                    var={"Alpha": {"lane24": _rec("T28")},
                         "Bravo": {"lane28": _rec("T24")}},
                    owners="AAAA:Alpha, BBBB:Bravo")
        for uid in order:
            ch.claim(uid)
        assert (ch.lanes["lane24"].map, ch.lanes["lane28"].map) == (
            ["T28"], ["T24"])
        assert ch.log.warnings == []
        _consistent(ch.afc)

    def test_a_saved_tool_a_claimed_lane_took_stays_with_it(self, tmp_path):
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB", pool_ams=2,
                    names="Alpha, Bravo",
                    var={"Alpha": {"lane24": _rec("T28")}},
                    owners="AAAA:Alpha")
        ch.claim(B)                                  # lane28 takes T28
        ch.claim(A)
        assert ch.lanes["lane24"].map == ["T24"]
        # Both lanes are on their own home tools: AFC.log only.
        assert ch.log.warnings == []
        assert ("AFC_BridgeBox chain1: lane24: saved T28 is held by lane28 "
                "-- not restored; lane24 is back on T24") in ch.log.debugs
        _consistent(ch.afc)

    def test_a_saved_tool_a_bambu_lane_holds_off_its_home_still_warns(
            self, tmp_path):
        # lane28 holds T3 (not its home) by SET_MAP: the user has something
        # to look at.
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB", pool_ams=2,
                    names="Alpha, Bravo",
                    var={"Alpha": {"lane24": _rec("T3")},
                         "Bravo": {"lane28": _rec("T3")}},
                    owners="AAAA:Alpha, BBBB:Bravo")
        ch.claim(B)
        ch.claim(A)
        assert ch.lanes["lane24"].map == ["T24"]
        assert ch.log.warnings == [
            "AFC_BridgeBox chain1: lane24: saved T3 is held by lane28 -- not "
            "restored; lane24 is back on T24"]
        _consistent(ch.afc)

    @pytest.mark.parametrize("order", [(A, C), (C, A)])
    def test_a_lane_on_a_sibling_home_tool_goes_back_to_its_own(
            self, tmp_path, order):
        # lane24 was saved on T28 while Bravo was unclaimed; a unit with no
        # record for Bravo claims it and takes its home T28.
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T28")}},
                    owners="AAAA:Alpha")
        for uid in order:
            ch.claim(uid)
        assert ch.bay("Bravo")["bound"] == C
        lane24 = ch.lanes["lane24"]
        assert (lane24.map, lane24.current_map) == (["T24"], "T24")
        assert ch.lanes["lane28"].map == ["T28"]
        _consistent(ch.afc)
        # The home wins and both lanes end on their own home tools, so the
        # move is in AFC.log only.
        assert ch.log.warnings == []
        if order[0] == A:
            assert ("AFC_BridgeBox chain1: T28 is the tool of Bambu lane "
                    "lane28. lane24 was mapped to it and is back on T24."
                    ) in ch.log.debugs
            assert lane24.sent == [["T28"], ["T24"]]
        ch.m._release_pool_unit(A)
        assert ch.m._held["Alpha"]["lanes"]["lane24"]["map"] == "T24"

    def test_a_bay_mate_on_a_lanes_home_tool_goes_back_to_its_own(
            self, tmp_path):
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T25"),
                                             "lane25": {"spool_id": 5}}},
                    owners="AAAA:Alpha")
        ch.claim(A)
        assert (ch.lanes["lane24"].map, ch.lanes["lane25"].map) == (
            ["T24"], ["T25"])
        assert ch.log.lines[-1].endswith("(4 lanes) -- live, no restart.")
        _consistent(ch.afc)

    def test_a_lane_whose_own_home_tool_is_held_is_still_moved_off(
            self, tmp_path):
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T28")}},
                    owners="AAAA:Alpha")
        lane5 = _other(ch.afc, "lane5", ["T24"])
        ch.claim(A)
        ch.claim(C)
        assert (ch.lanes["lane24"].map, lane5.map) == (["T37"], ["T24"])
        assert ch.lanes["lane28"].map == ["T28"]
        _consistent(ch.afc)
        # Left on a spare, not its home tool: the user has something to do.
        assert any("lane24 was mapped to it and is now T37" in w
                   for w in ch.log.warnings)

    @pytest.mark.parametrize("order", [(A, C), (C, A)])
    def test_a_multi_map_on_a_sibling_home_tool_keeps_its_own_quietly(
            self, tmp_path, order):
        # SET_MAP gave lane24 T28 as well while Bravo was unclaimed. The
        # home wins, and lane24 keeps its own T24: AFC.log only.
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T24, T28")}},
                    owners="AAAA:Alpha")
        for uid in order:
            ch.claim(uid)
        assert ch.lanes["lane24"].map == ["T24"]
        assert ch.lanes["lane28"].map == ["T28"]
        assert ch.log.warnings == []
        assert [d for d in ch.log.debugs if "T28" in d and "lane24" in d]
        _consistent(ch.afc)

    def test_a_multi_map_on_a_config_mapped_tool_still_warns(self, tmp_path):
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T24, T28")}},
                    owners="AAAA:Alpha")
        ch.claim(A)
        ch.lanes["lane24"]._map = ["T28"]            # map: in its config
        ch.claim(C)
        assert ch.lanes["lane24"].map == ["T24"]
        assert any("Also set map: in [AFC_lane lane24]" in w
                   for w in ch.log.warnings)

    def test_the_claim_line_lists_saved_maps_one_lane_each(self, tmp_path):
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T3, T40"),
                                             "lane25": _rec("T5")}},
                    owners="AAAA:Alpha")
        _other(ch.afc, "lane5", ["T5"])
        _other(ch.afc, "lane6", ["T25"])
        ch.claim(A)
        assert ch.lanes["lane25"].map == ["NONE"]
        assert ch.log.lines[-1].endswith(
            "(4 lanes) -- live, no restart. Saved maps: lane24->T3+T40.")

    def test_a_release_drops_the_restored_tool_and_not_the_home_one(
            self, tmp_path):
        ch, lane5 = self._swapped(tmp_path)
        ch.claim(A)
        ch.m._release_pool_unit(A)
        assert "T3" not in ch.afc.tool_cmds
        assert "T3" not in ch.gcode.ready_gcode_handlers
        assert ch.afc.tool_cmds == {"T24": "lane5"}
        assert ch.gcode.ready_gcode_handlers.get("T24") is not None
        _consistent(ch.afc)

    def test_a_map_set_while_claimed_survives_a_replug(self, tmp_path):
        ch = _chain(tmp_path)
        ch.claim(A)
        lane = ch.lanes["lane25"]
        ch.afc.tool_cmds.pop("T25")
        ch.gcode.register_command("T25", None)
        lane.map, lane.current_map = ["T7"], "T7"
        ch.afc.tool_cmds["T7"] = "lane25"
        ch.gcode.register_command("T7", ch.afc.cmd_CHANGE_TOOL)
        ch.m._release_pool_unit(A)
        ch.claim(A)
        assert (lane.map, lane.current_map) == (["T7"], "T7")
        assert ch.afc.tool_cmds.get("T25") is None
        _consistent(ch.afc)

    def test_a_removed_last_tool_stays_removed_across_a_replug(self,
                                                              tmp_path):
        ch = _chain(tmp_path)
        ch.claim(A)
        lane = ch.lanes["lane25"]
        # AFC_REMOVE_MAPPING MAPPING=T25, as AFC_spool does it.
        ch.afc.tool_cmds.pop("T25")
        lane.map = [c for c in lane.map if c != "T25"]
        lane.current_map = ""
        ch.gcode.register_command("T25", None)
        ch.m._release_pool_unit(A)
        assert ch.m._held["Alpha"]["lanes"]["lane25"]["map"] == "NONE"
        ch.claim(A)
        assert (lane.map, lane.current_map) == (["NONE"], "")
        assert "T25" not in ch.afc.tool_cmds
        assert "T25" not in ch.gcode.ready_gcode_handlers
        assert ch.log.warnings == []
        _consistent(ch.afc)

    def test_a_removed_last_tool_stays_removed_across_a_restart(self,
                                                               tmp_path):
        ch = _chain(tmp_path, var={"Alpha": {"lane25": _rec("NONE", "")}},
                    owners="AAAA:Alpha")
        ch.claim(A)
        assert ch.lanes["lane25"].map == ["NONE"]
        assert "T25" not in ch.gcode.ready_gcode_handlers
        assert ch.log.lines[-1].endswith("Saved maps: lane25->NONE.")
        _consistent(ch.afc)

    def test_a_tool_moved_to_a_bay_mate_stays_there(self, tmp_path):
        # Multiple mapping: SET_MAP LANE=lane27 MAP=T25 left lane25 on NONE.
        ch = _chain(tmp_path, var={"Alpha": {
            "lane25": _rec("NONE", ""), "lane27": _rec("T25, T27", "T27")}},
            owners="AAAA:Alpha")
        ch.claim(A)
        assert (ch.lanes["lane25"].map, ch.lanes["lane27"].map) == (
            ["NONE"], ["T25", "T27"])
        assert ch.afc.tool_cmds["T25"] == "lane27"
        assert ch.log.warnings == []
        _consistent(ch.afc)

    @pytest.mark.parametrize("order", [(A, B), (B, A)])
    def test_a_tool_moved_to_another_bay_stays_there_in_either_order(
            self, tmp_path, order):
        ch = _chain(tmp_path, roster="boxed:AAAA, boxed:BBBB", pool_ams=2,
                    names="Alpha, Bravo",
                    var={"Alpha": {"lane25": _rec("NONE", "")},
                         "Bravo": {"lane28": _rec("T25, T28", "T28")}},
                    owners="AAAA:Alpha, BBBB:Bravo")
        for uid in order:
            ch.claim(uid)
        assert (ch.lanes["lane25"].map, ch.lanes["lane28"].map) == (
            ["NONE"], ["T25", "T28"])
        assert ch.log.warnings == []
        _consistent(ch.afc)

    def test_a_claimed_lane_is_marked_prep_done(self, tmp_path):
        ch = _chain(tmp_path)
        ch.claim(A)
        assert ch.lanes["lane24"]._afc_prep_done is True


class TestAClaimDuringAPrint:
    """A unit plugged in during a print is claimed at once: its lanes take
    their records, the save writes them, and a lane AFC records in a
    toolhead is restored. Only a T# another live lane or a macro holds,
    which the print may be using, is left where it is until the print ends,
    said once."""

    REC = {"Alpha": {"lane24": _rec("T24", material="PLA", color="#FF0000",
                                    weight=412.0, tool_loaded=True)}}

    def _printing(self, ch, state):
        ch.printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": state[0]})

    def test_the_claim_leaves_a_live_tool_until_the_print_ends(
            self, tmp_path):
        ch = _chain(tmp_path, var=self.REC, owners="AAAA:Alpha")
        ch.afc.tools = {"extruder": types.SimpleNamespace(
            name="extruder", lane_loaded="lane24")}
        lane5 = _other(ch.afc, "lane5", ["T24"])
        state = ["printing"]
        self._printing(ch, state)
        assert ch.claim(A) is ch.units["Alpha"]
        assert ch.bay("Alpha")["bound"] == A
        lane24 = ch.lanes["lane24"]
        # The records are on the lanes and saved, the toolhead lane is
        # restored, and the live T# stays with lane5.
        assert (lane24.material, lane24.weight) == ("PLA", 412.0)
        assert lane24.tool_loaded is True
        assert ch.afc.saves and ch.m._owners() == {"Alpha": A}
        assert (lane24.map, lane5.map) == (["NONE"], ["T24"])
        assert ch.afc.tool_cmds["T24"] == "lane5"
        assert ch.lanes["lane25"].map == ["T25"]
        assert ch.log.lines[0] == (
            "AFC_BridgeBox chain1: lane24 takes T24 from lane5 once the print "
            "ends, as the print may be using it; until then lane24 has no "
            "T#.")
        _consistent(ch.afc)
        ch.m._take_deferred_tools()                   # still printing
        assert (lane24.map, lane5.map) == (["NONE"], ["T24"])
        state[0] = "complete"
        del ch.log.lines[:]
        saves = len(ch.afc.saves)
        ch.m._take_deferred_tools()
        assert (lane24.map, lane24.current_map) == (["T24"], "T24")
        assert lane5.map == ["T37"]
        assert ch.log.lines == [
            "AFC_BridgeBox chain1: the printer is idle, so the T#s the claim "
            "left in use are taken: lane24 is T24."]
        assert len(ch.afc.saves) > saves
        assert ch.m._deferred_takes == {}
        _consistent(ch.afc)
        ch.m._take_deferred_tools()                   # once
        assert len(ch.log.lines) == 1

    def test_a_spare_bay_it_adopts_is_claimed_too(self, tmp_path):
        ch = _chain(tmp_path)
        _other(ch.afc, "lane5", ["T28"])
        self._printing(ch, ["printing"])
        assert ch.claim(C) is ch.units["Bravo"]
        assert ch.bay("Bravo")["bound"] == C
        assert ch.lanes["lane28"].map == ["NONE"]
        assert "lane28" in ch.m._deferred_takes

    def test_a_macro_is_not_renamed_during_the_print(self, tmp_path):
        # force_assign_map lets TcmdAssign rename a macro out of the way.
        ch = _chain(tmp_path)
        ch.afc.force_assign_map = True
        fn = ch.afc.function
        fn._rename = types.MethodType(afcFunction._rename, fn)
        handlers = ch.gcode.ready_gcode_handlers
        macro = handlers["T24"] = lambda gcmd: None
        state = ["printing"]
        self._printing(ch, state)
        ch.claim(A)
        assert handlers["T24"] is macro and "_T24" not in handlers
        assert ch.lanes["lane24"].map == ["NONE"]
        assert ch.log.lines[0].startswith(
            "AFC_BridgeBox chain1: lane24 takes T24 from a macro once the "
            "print ends")
        state[0] = "standby"
        ch.m._take_deferred_tools()
        assert handlers["_T24"] is macro
        assert handlers["T24"] == ch.afc.cmd_CHANGE_TOOL
        assert ch.lanes["lane24"].map == ["T24"]
        assert ch.afc.tool_cmds["T24"] == "lane24"

    def test_a_release_before_the_print_ends_drops_the_wait(self, tmp_path):
        ch = _chain(tmp_path)
        lane5 = _other(ch.afc, "lane5", ["T24"])
        state = ["printing"]
        self._printing(ch, state)
        ch.claim(A)
        state[0] = "complete"
        ch.m._release_pool_unit(A)
        assert ch.m._deferred_takes == {}
        ch.m._take_deferred_tools()
        assert lane5.map == ["T24"] and ch.afc.tool_cmds["T24"] == "lane5"
        # The next claim is handed the map this claim planned.
        rec = ch.m._held["Alpha"]["lanes"]["lane24"]
        assert (rec["map"], rec["current_map"]) == ("T24", "T24")

    def test_a_tool_the_lane_is_given_meanwhile_stays(self, tmp_path):
        ch = _chain(tmp_path)
        _other(ch.afc, "lane5", ["T24"])
        state = ["printing"]
        self._printing(ch, state)
        ch.claim(A)
        lane24 = ch.lanes["lane24"]
        lane24.map, lane24.current_map = ["T40"], "T40"   # SET_MAP
        ch.afc.tool_cmds["T40"] = "lane24"
        ch.gcode.ready_gcode_handlers["T40"] = ch.afc.cmd_CHANGE_TOOL
        state[0] = "complete"
        ch.m._take_deferred_tools()
        assert (lane24.map, lane24.current_map) == (["T24", "T40"], "T24")
        _consistent(ch.afc)

    def test_a_running_command_on_an_idle_printer_says_so(self, tmp_path):
        ch = _chain(tmp_path)
        _other(ch.afc, "lane5", ["T24"])
        busy = ["Printing"]
        ch.printer.objects["idle_timeout"] = types.SimpleNamespace(
            get_status=lambda et: {"state": busy[0]})
        ch.claim(A)
        assert ch.log.lines[0] == (
            "AFC_BridgeBox chain1: lane24 takes T24 from lane5 once the "
            "printer is idle, as a running command may be using it; until "
            "then lane24 has no T#.")
        ch.m._take_deferred_tools()
        assert ch.lanes["lane24"].map == ["NONE"]
        busy[0] = "Idle"
        ch.m._take_deferred_tools()
        assert ch.lanes["lane24"].map == ["T24"]

    def test_assign_says_the_unit_is_claimed(self, tmp_path, monkeypatch):
        ch = _chain(tmp_path)
        _other(ch.afc, "lane5", ["T28"])
        self._printing(ch, ["printing"])
        ch.online(monkeypatch, [C], [True])
        cmd = _GCmd(UID=C, NAME="Bravo")
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert ch.bay("Bravo")["bound"] == C
        assert cmd.responses[-1].endswith("-- claimed LIVE, no restart.")

    def test_a_claim_that_takes_no_live_tool_waits_for_nothing(
            self, tmp_path):
        ch = _chain(tmp_path)
        _other(ch.afc, "lane5", ["T5"])
        self._printing(ch, ["printing"])
        assert ch.claim(A) is ch.units["Alpha"]
        assert ch.lanes["lane24"].map == ["T24"]
        assert not [x for x in ch.log.lines if "once the print ends" in x]
        assert ch.m._deferred_takes == {}

    def test_a_saved_map_that_leaves_the_home_tool_alone_goes_ahead(
            self, tmp_path):
        ch = _chain(tmp_path, var={"Alpha": {"lane24": _rec("T3")}},
                    owners="AAAA:Alpha")
        _other(ch.afc, "lane5", ["T24"])
        self._printing(ch, ["printing"])
        assert ch.claim(A) is ch.units["Alpha"]
        assert ch.afc.tool_cmds["T24"] == "lane5"
        assert ch.lanes["lane24"].map == ["T3"]
        assert ch.m._deferred_takes == {}


class TestAReplugRestoresTheSpool:
    """A bridge OTA flash drops every unit off USB for longer than the
    release grace. The HT on Hot (lane32, Spoolman spool 136) is released
    and claimed back seconds later, its bay present with no record yet."""

    def _ht(self, tmp_path, spool_id=136, spoolman=True):
        ch = _chain(tmp_path, roster="boxed:AAAA, ht:HHHH", pool_ams=2,
                    names="Alpha, Bravo")
        if spoolman:
            ch.afc.spoolman = object()
        ch.claim(H, "ht")
        lane = ch.lanes["lane32"]
        lane.spool_id, lane.material = spool_id, "PLA" if spool_id else ""
        lane.color, lane.weight = "#0086D6", 750.0
        ch.m._release_pool_unit(H)
        assert (lane.spool_id, lane.material) == (None, "")
        return ch, lane

    def _prime(self, ch):
        unit = ch.units["Hot"]
        unit.bays({"present": True})
        unit._prime_scan_baseline()
        return unit

    def test_the_same_unit_gets_its_spool_back_through_spoolman(
            self, tmp_path):
        ch, lane = self._ht(tmp_path)
        ch.claim(H, "ht")
        assert ch.holds("Hot")[-1]["lane32"]["spool_id"] == 136
        # On the lane from the claim, before scan priming.
        assert (lane.spool_id, lane.material) == (136, "PLA")
        assert ch.afc.bound == [("lane32", 136, False)]
        unit = self._prime(ch)
        assert lane.spool_id == 136
        assert ch.afc.bound == [("lane32", 136, False)]
        assert unit.finalized == []                   # no defaults
        assert unit._restored_bays == {0}
        assert ("AFC bambu Hot: restored the saved record of the untagged "
                "spool on lane32 (spool 136)") in ch.log.lines
        assert unit._held_lanes == {}

    def test_without_spoolman_the_profile_comes_back(self, tmp_path):
        ch, lane = self._ht(tmp_path, spoolman=False)
        ch.claim(H, "ht")
        self._prime(ch)
        assert (lane.material, lane.color, lane.weight) == (
            "PLA", "#0086D6", 750.0)
        assert ch.afc.bound == []

    def test_a_different_unit_gets_defaults(self, tmp_path):
        ch, lane = self._ht(tmp_path)
        ch.m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=H))
        ch.m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID=J, NAME="Hot"))
        ch.claim(J, "ht")
        unit = self._prime(ch)
        assert lane.spool_id is None and ch.afc.bound == []
        assert unit.finalized == [(0, False, True)]

    def test_a_lane_that_held_no_spool_comes_back_empty(self, tmp_path):
        ch, lane = self._ht(tmp_path, spool_id=None)
        ch.claim(H, "ht")
        unit = self._prime(ch)
        assert lane.spool_id is None and ch.afc.bound == []
        assert unit._restored_bays == set()
        assert unit.finalized == [(0, False, True)]    # a spool: defaults
        assert lane.map == ["T32"]


def _unit(held=None, tmp_path=None, spoolman=False):
    afc = _Afc(tmp_path)
    if spoolman:
        afc.spoolman = object()
    u = _Unit("AMS", ["lane15"], [], afc, _Log())
    lane = _Lane("lane15", afc, u)
    u.lanes = {"lane15": lane}
    u._slot_map = {"lane15": 0}
    u._held_lanes = held
    afcBambuAMS._reset_lookup_state(u)
    u.bays({"present": True})
    u.saved = 0

    def save():
        u.saved += 1
    u._save_lane_vars = save
    return u, lane


def _claimed(held=None, spoolman=False, bay=None):
    """A unit whose claim put ``held`` on its lane, before scan priming."""
    u, lane = _unit(held, spoolman=spoolman)
    if bay is not None:
        u.bays(bay)
    afcBambuAMS._apply_held_lanes(u)
    return u, lane


class TestTheUnitsHeldRecords:
    REC = {"spool_id": 159, "material": "PLA", "color": "#050505",
           "weight": 1000.0, "sub_type": "Matte"}

    def test_held_records_are_read_without_the_file(self, tmp_path,
                                                    monkeypatch):
        (tmp_path / "AFC.var.unit").write_text(
            json.dumps({"AMS": {"lane15": {"spool_id": 7}}}))
        u, _lane = _unit(None, tmp_path)
        assert u._persisted_lane("lane15") == {"spool_id": 7}

        def no_disk(*a, **k):
            raise AssertionError("read the file")
        monkeypatch.setattr(builtins, "open", no_disk)
        u._held_lanes = {"lane15": dict(self.REC)}
        assert u._persisted_lane("lane15") == self.REC
        u._held_lanes = {}
        assert u._persisted_lane("lane15") == {}

    def test_hold_lanes_keeps_a_copy_until_priming(self):
        u, _lane = _unit()
        records = {"lane15": dict(self.REC)}
        afcBambuAMS.hold_lanes(u, records)
        records["lane15"]["spool_id"] = 1
        assert u._held_lanes == {"lane15": self.REC}
        u._prime_scan_baseline()
        assert u._held_lanes == {}

    def test_release_empties_them(self):
        u, _lane = _unit({"lane15": dict(self.REC)})
        afcBambuAMS.release(u)
        assert u._held_lanes == {}

    def test_the_claim_keeps_what_the_master_handed_it(self, monkeypatch):
        # The master hands the records over between release() and claim().
        u, _lane, _obj = _measuring_unit(None, pool=True)
        records = {"lane8": dict(self.REC)}
        release = u.release

        def release_then_hold():
            release()
            u.hold_lanes(records)
        u.release = release_then_hold
        _reclaim(u, monkeypatch)
        assert u._held_lanes == records

    def test_the_claim_puts_the_record_on_the_lane_before_priming(self):
        rec = dict(self.REC, extruder_temp="215", bed_temp="NONE",
                   runout_lane="lane16", need_purge=True,
                   td1_data={"td": 1.2})
        u, lane = _claimed({"lane15": rec})
        assert (lane.spool_id, lane.material, lane.color, lane.weight,
                lane.sub_type) == (159, "PLA", "#050505", 1000.0, "Matte")
        # Read as PREP reads them: numbers, and NONE as none.
        assert (lane.extruder_temp, lane.bed_temp) == (215.0, None)
        # What PREP restores for every lane besides the spool.
        assert (lane.runout_lane, lane.need_purge, lane.td1_data) == (
            "lane16", True, {"td": 1.2})
        assert u.finalized == [] and u.saved == 0

    def test_a_saved_none_runout_lane_comes_back_as_none(self):
        u, lane = _claimed({"lane15": dict(self.REC, runout_lane="NONE")})
        assert lane.runout_lane is None

    def test_with_spoolman_the_link_is_fetched_over_the_stored_profile(self):
        u, lane = _claimed({"lane15": dict(self.REC)}, spoolman=True)
        assert u.afc.bound == [("lane15", 159, False)]
        # Spoolman down: the fetch fails, and the lane still has its profile.
        assert (lane.spool_id, lane.material, lane.weight) == (
            159, "PLA", 1000.0)

    def test_an_untagged_bay_keeps_its_record_at_priming(self):
        u, lane = _claimed({"lane15": dict(self.REC)})
        u._prime_scan_baseline()
        assert (lane.spool_id, lane.material, lane.sub_type) == (
            159, "PLA", "Matte")
        assert u.finalized == []
        assert (u._prev_present, u._untagged_rearmed) == ([True], [False])
        assert u._restored_bays == {0} and u._afc_owned == set()
        assert u.logger.lines == [
            "AFC bambu AMS: restored the saved record of the untagged spool "
            "on lane15 (spool 159)"]

    def test_a_reclaim_keeps_it_too(self):
        u, lane = _claimed({"lane15": dict(self.REC)})
        u._prep_seen = True                       # claimed before
        u._prime_scan_baseline()
        assert lane.spool_id == 159 and u._restored_bays == {0}

    def test_one_line_names_every_lane_that_kept_a_record(self):
        afc = _Afc()
        afc.default_material_type = "PLA"
        u = _Unit("AMS", ["lane15", "lane16", "lane17", "lane18"], [], afc,
                  _Log())
        u.lanes = {n: _Lane(n, afc, u) for n in u.lane_names}
        u._slot_map = {n: i for i, n in enumerate(u.lane_names)}
        u._held_lanes = {"lane15": dict(self.REC),
                         "lane16": {"material": "PETG", "color": "#FF0000"},
                         "lane17": {"material": "PLA", "color": ""},
                         "lane18": {"material": "PLA"}}
        afcBambuAMS._reset_lookup_state(u)
        u.bays(*[{"present": True}] * 4)
        afcBambuAMS._apply_held_lanes(u)
        u._prime_scan_baseline()
        assert u.logger.lines == [
            "AFC bambu AMS: restored the saved records of the untagged spools "
            "on lane15 (spool 159), lane16; lane17, lane18 have only the AFC "
            "defaults saved last session, as nothing has read their spools; "
            "reseat them, or run AFC_BAMBU_SCAN LANE=<lane>"]
        # Defaults give way to a tag; a record waits for its tag.
        assert (u._restored_bays, u._defaulted_bays) == ({0, 1}, {2, 3})

    def test_a_record_of_only_the_defaults_says_how_to_read_the_spool(self):
        u, lane = _unit({"lane15": {"material": "PLA"}})
        u.afc.default_material_type = "PLA"
        afcBambuAMS._apply_held_lanes(u)
        u._prime_scan_baseline()
        assert u.logger.lines == [
            "AFC bambu AMS: lane15 has only the AFC defaults saved last "
            "session, as nothing has read its spool; reseat it, or run "
            "AFC_BAMBU_SCAN LANE=lane15"]
        assert u._boot_hold(lane, {"index": 0}) is False     # a tag is a read

    def test_a_tag_of_the_same_spool_keeps_the_record(self):
        # Spoolman link, weight, variant and temperatures stay; the tag's
        # material and colour win. The lane is held as AFC restored it.
        rec = dict(self.REC, material="pla", color="", weight=412.0,
                   extruder_temp=225.0)
        u, lane = _claimed({"lane15": rec}, bay={
            "present": True, "material": "PLA Matte", "color": "050505",
            "rfid_uid": "0A1B2C3D", "temp_min": 190})
        u._prime_scan_baseline()
        assert (lane.spool_id, lane.weight, lane.sub_type,
                lane.extruder_temp) == (159, 412.0, "Matte", 225.0)
        assert (lane.material, lane.color) == ("PLA", "#050505")
        assert u._afc_owned == {0} and u._restored_bays == set()
        assert u._boot_hold(lane, u._slots[0]) is True
        assert u.saved == 1 and u.logger.lines == []

    @pytest.mark.parametrize("tag", [
        {"material": "PETG", "color": "00AE42"},
        {"material": "PLA Silk", "color": "050505"},
        {"material": "PLA Matte", "color": "FF0000"},
        {"material": "PLA Matte", "color": None, "color_black": True}])
    def test_a_tag_of_another_spool_replaces_a_record_with_no_link(self, tag):
        rec = dict(self.REC, spool_id=None, extruder_temp=225.0,
                   td1_data={"td": 1.2}, need_purge=True)
        u, lane = _claimed({"lane15": rec},
                           bay=dict(tag, present=True, rfid_uid="0A1B2C3D"))
        u._prime_scan_baseline()
        # The tag is applied as a read (the stand-in surfaces the material),
        # and the record's extras go with it.
        assert lane.material == tag["material"]
        assert (lane.extruder_temp, lane.bed_temp, lane.td1_data,
                lane.need_purge, lane.weight) == (None, None, {}, False, 0)
        assert u._restored_bays == set()
        assert any("is not the spool saved for this lane" in ln
                   for ln in u.logger.lines)

    def test_a_tag_of_another_material_keeps_a_linked_record(self):
        # A linked record's material is Spoolman's or the user's words: only
        # Spoolman can show the link is another spool's (_check_kept_link).
        # The tag's material and colour win, and the rest stays.
        rec = dict(self.REC, weight=412.0, extruder_temp=225.0)
        u, lane = _claimed({"lane15": rec}, bay={
            "present": True, "material": "PETG", "color": "FF0000",
            "rfid_uid": "0A1B2C3D"})
        u._prime_scan_baseline()
        assert (lane.spool_id, lane.weight, lane.extruder_temp) == (
            159, 412.0, 225.0)
        assert (lane.material, lane.color) == ("PETG", "#FF0000")
        assert u._afc_owned == {0}
        assert not [ln for ln in u.logger.lines if "is not the spool" in ln]

    @pytest.mark.parametrize("lane_material,tag", [
        ("PLA", {"material": "PLA Silk", "color": "050505"}),
        ("PLA", {"material": "PLA Matte", "color": "FF0000"}),
        ("PLA", {"material": "PLA Matte", "color": None,
                 "color_black": True}),
        ("PLA+", {"material": "PLA Basic", "color": "050505"}),
        ("Silk PLA", {"material": "PLA Silk", "color": "050505"}),
        ("PETG HF", {"material": "PETG HF", "color": "050505"})])
    def test_a_linked_record_keeps_its_spool_against_its_own_words(
            self, lane_material, tag):
        # A linked record's material and colour are Spoolman's words or the
        # user's (SET_COLOR, SET_MATERIAL), not the tag's, and differ from
        # them for the same spool; whether the spool carries the tag is
        # Spoolman's to say (_check_kept_link).
        rec = dict(self.REC, material=lane_material, weight=412.0,
                   extruder_temp=225.0)
        u, lane = _claimed({"lane15": rec},
                           bay=dict(tag, present=True, rfid_uid="0A1B2C3D"))
        u._prime_scan_baseline()
        assert (lane.spool_id, lane.weight, lane.extruder_temp) == (
            159, 412.0, 225.0)
        assert u._afc_owned == {0}
        assert not [ln for ln in u.logger.lines if "is not the spool" in ln]

    def test_a_black_record_is_not_the_spool_of_a_coloured_tag(self):
        # A lane dressed from a black tag carries its variant and no colour.
        rec = {"material": "PLA", "color": "", "sub_type": "Matte",
               "weight": 640.0}
        u, lane = _claimed({"lane15": rec}, bay={
            "present": True, "material": "PLA Matte", "color": "FF0000",
            "rfid_uid": "0A1B2C3D"})
        u._prime_scan_baseline()
        assert (lane.material, lane.sub_type, lane.weight) == ("PLA Matte",
                                                               "", 0)
        assert any("is not the spool saved for this lane" in ln
                   for ln in u.logger.lines)
        # The black tag of the same spool keeps it.
        u, lane = _claimed({"lane15": dict(rec)}, bay={
            "present": True, "material": "PLA Matte", "color": None,
            "color_black": True, "rfid_uid": "0A1B2C3D"})
        u._prime_scan_baseline()
        assert lane.weight == 640.0 and u._afc_owned == {0}

    @pytest.mark.parametrize("extra", [
        {"weight": 640.0},                     # AFC counted it down
        {"extruder_temp": 190.0},              # set by hand
        {"bed_temp": 70.0},                    # set by hand
        {"sub_type": "Matte"}])                # a black tag's variant
    def test_a_record_with_anything_of_the_users_is_not_the_defaults(
            self, extra):
        rec = dict({"material": "PLA", "color": "#FFFFFF", "weight": 1000.0},
                   **extra)
        u, lane = _unit({"lane15": rec})
        u.afc.default_material_type, u.afc.default_color = "PLA", "#FFFFFF"
        afcBambuAMS._apply_held_lanes(u)
        u._prime_scan_baseline()
        assert u._restored_bays == {0} and u._defaulted_bays == set()
        assert (lane.weight, lane.extruder_temp) == (
            rec["weight"], rec.get("extruder_temp"))
        assert "restored the saved record" in u.logger.lines[0]

    def test_a_black_bare_tags_record_at_its_weight_is_kept(self):
        # A black tag gives no colour and, bare, no variant: only the weight
        # AFC counted down tells it from the defaults.
        u, lane = _unit({"lane15": {"material": "ABS", "color": "",
                                    "weight": 640.0, "bed_temp": 95.0}})
        u.afc.default_material_type = "ABS"
        afcBambuAMS._apply_held_lanes(u)
        u._prime_scan_baseline()
        assert u._restored_bays == {0} and lane.weight == 640.0

    @pytest.mark.parametrize("bed", [None, "derived", 60.0])
    def test_what_the_defaults_derive_is_still_the_defaults(self, bed):
        from extras.AFC_BambuAMS import bed_temp_for_material
        rec = {"material": "PLA", "color": "#FFFFFF", "weight": 1000.0,
               "bed_temp": (bed_temp_for_material("PLA") if bed == "derived"
                            else bed)}
        u, lane = _unit({"lane15": rec})
        u.afc.default_material_type, u.afc.default_color = "PLA", "#FFFFFF"
        afcBambuAMS._apply_held_lanes(u)
        u._prime_scan_baseline()
        assert u._defaulted_bays == {0} and u._restored_bays == set()

    def test_a_linked_record_leaves_need_purge_to_spoolman(self):
        # AFC_prep restores need_purge only for a lane Spoolman does not
        # fetch.
        u, lane = _claimed({"lane15": dict(self.REC, need_purge=True)},
                           spoolman=True)
        assert lane.need_purge is False
        u, lane = _claimed({"lane15": dict(self.REC, need_purge=True)})
        assert lane.need_purge is True

    def _linked(self, contradicted, post=None):
        u, lane = _claimed({"lane15": dict(self.REC)}, spoolman=True, bay={
            "present": True, "material": "PLA Matte", "color": "050505",
            "rfid_uid": "0A1B2C3D"})
        asked = []

        def check(sid, uid):
            asked.append((sid, uid))
            return contradicted
        u._spool = types.SimpleNamespace(_binding_contradicted=check,
                                         _spoolman_bg=lambda job: job())
        if post is not None:
            u.afc.reactor = types.SimpleNamespace(
                register_async_callback=post)
        u._prime_scan_baseline()
        return u, lane, asked

    def test_spoolman_is_asked_whether_a_kept_link_carries_the_tag(self):
        u, lane, asked = self._linked(False)
        assert asked == [(159, "0A1B2C3D")]
        assert lane.spool_id == 159 and u._afc_owned == {0}

    def test_a_link_spoolman_gives_other_tags_is_replaced_by_the_tag(self):
        posted = []
        u, lane, _asked = self._linked(True, post=posted.append)
        assert lane.spool_id == 159             # the reactor runs it later
        saved = u.saved
        posted[0](0.0)
        assert lane.spool_id in (None, "") and lane.material == "PLA Matte"
        assert u.saved == saved + 1
        assert any("(Spoolman records other tags for spool 159)" in ln
                   and "link to spool 159" in ln for ln in u.logger.lines)

    def test_a_contradiction_for_a_lane_changed_since_does_nothing(self):
        posted = []
        u, lane, _asked = self._linked(True, post=posted.append)
        lane.spool_id = 42                      # linked again meanwhile
        posted[0](0.0)
        assert lane.spool_id == 42 and lane.material == "PLA"

    def _late(self, bay, contradicted=False):
        u, lane = _unit({"lane15": dict(self.REC, extruder_temp=215.0)},
                        spoolman=True)
        u.bays(bay)
        u._spool = types.SimpleNamespace(
            _binding_contradicted=lambda sid, uid: contradicted,
            _spoolman_bg=lambda job: job())
        fetches = []

        def set_spool_id(ln, sid, save_vars=True, on_done=None):
            def land():
                ln.spool_id, ln.material = sid, "PLA"
                ln.extruder_temp = 215.0
                on_done()
            fetches.append(land)
        u.afc.spool.set_spoolID = set_spool_id
        afcBambuAMS._apply_held_lanes(u)
        u._prime_scan_baseline()
        return u, lane, fetches

    def test_a_fetch_landing_after_the_tag_dropped_the_record_is_undone(
            self):
        u, lane, fetches = self._late({"present": True, "material": "PETG",
                                       "color": "FF0000",
                                       "rfid_uid": "0A1B2C3D"},
                                      contradicted=True)
        assert lane.spool_id in (None, "") and lane.material == "PETG"
        saved = u.saved
        fetches[0]()                            # Spoolman answers now
        assert lane.spool_id in (None, "") and lane.material == "PETG"
        assert lane.extruder_temp is None and u.saved == saved + 1

    def test_a_fetch_landing_after_the_bay_emptied_is_undone(
            self, monkeypatch):
        monkeypatch.setattr(_Unit, "_reconcile_empty_bays",
                            afcBambuAMS._reconcile_empty_bays)
        monkeypatch.setattr(_Unit, "_forget_spoolman_miss",
                            lambda self, slot: None, raising=False)
        u, lane, fetches = self._late({"present": False})
        assert (lane.spool_id, lane.material) == ("", "")
        fetches[0]()
        assert (lane.spool_id, lane.material, lane.extruder_temp) == (
            "", "", None)

    @pytest.mark.parametrize("lane,tag,differ", [
        ("PLA", "PLA", False), ("PLA+", "PLA", False),
        ("PLA Matte", "PLA", False), ("Silk PLA", "PLA", False),
        ("PLA-CF", "PLA", False), ("PET", "PETG", False),
        ("PETG", "PET", False), ("PETG", "PLA", True), ("ABS", "ASA", True),
        ("", "PLA", False), ("PLA", "", False)])
    def test_materials_differ_only_when_no_word_matches(self, lane, tag,
                                                        differ):
        from extras.AFC_BambuAMS import _materials_differ
        assert _materials_differ(lane, tag) is differ

    def test_a_fetch_for_a_kept_record_is_left_alone(self):
        u, lane, fetches = self._late({"present": True})
        fetches[0]()
        assert (lane.spool_id, lane.material) == (159, "PLA")

    def test_a_tag_that_cannot_tell_them_apart_keeps_the_record(self):
        # A link saved while Spoolman was down carries no material or colour.
        u, lane = _claimed({"lane15": {"spool_id": 159}}, bay={
            "present": True, "material": "PETG", "color": "FF0000"})
        u._prime_scan_baseline()
        assert (lane.spool_id, lane.material, lane.color) == (
            159, "PETG", "#FF0000")
        assert u._afc_owned == {0}

    def test_a_late_tag_is_settled_against_the_kept_record(self):
        u, lane = _claimed({"lane15": dict(self.REC)})
        u._prime_scan_baseline()
        assert u._restored_bays == {0}
        # A UID with no profile yet settles nothing.
        u._settle_restored_bay(lane, {"index": 0, "rfid_uid": "0A"})
        assert u._restored_bays == {0}
        assert u._boot_hold(lane, {"index": 0}) is True
        u._settle_restored_bay(lane, {"index": 0, "material": "PLA Matte",
                                      "color": "050505"})
        assert lane.spool_id == 159 and u._restored_bays == set()
        assert u._afc_owned == {0}
        assert u._boot_hold(lane, {"index": 0}) is True

    def test_a_late_tag_of_another_spool_is_a_read(self):
        u, lane = _claimed({"lane15": dict(self.REC, spool_id=None)})
        u._prime_scan_baseline()
        u._settle_restored_bay(lane, {"index": 0, "material": "ABS",
                                      "color": "00AE42"})
        assert (lane.spool_id in (None, ""), lane.material) == (True, "")
        assert u._boot_hold(lane, {"index": 0}) is False

    def test_a_late_tag_of_another_material_keeps_a_linked_record(self):
        u, lane = _claimed({"lane15": dict(self.REC)})
        u._prime_scan_baseline()
        u._settle_restored_bay(lane, {"index": 0, "material": "ABS",
                                      "color": "050505"})
        assert (lane.spool_id, lane.material, lane.weight) == (
            159, "ABS", 1000.0)
        assert u._afc_owned == {0}

    def test_an_empty_bay_is_left_for_the_reconcile(self):
        u, lane = _claimed({"lane15": dict(self.REC)},
                           bay={"present": False})
        cleared = []
        u._reconcile_empty_bays = lambda: cleared.append(lane.spool_id)
        u._prime_scan_baseline()
        assert cleared == [159] and u._restored_bays == set()

    def test_a_bay_with_nothing_held_gets_defaults(self):
        u, lane = _claimed({})
        u._prime_scan_baseline()
        assert u.finalized == [(0, False, True)] and lane.spool_id is None

    def test_a_unit_no_master_claims_still_leaves_a_saved_bay_alone(self):
        u, lane = _unit(None)
        u._persisted_lane = lambda n: dict(self.REC)
        u._restore_untagged_defaults(claimed_live=False)
        assert lane.spool_id is None and u.finalized == [] and u.saved == 0

    def test_a_lane_cleared_since_the_claim_is_left_alone(self):
        u, lane = _claimed({"lane15": dict(self.REC)})
        lane.spool_id, lane.material = None, ""        # the user cleared it
        u._prime_scan_baseline()
        assert lane.spool_id is None and u.finalized == []
        assert u._restored_bays == set()

    def test_a_second_priming_does_not_bring_a_cleared_lane_back(self):
        u, lane = _claimed({"lane15": dict(self.REC)}, spoolman=True)
        u._prime_scan_baseline()
        assert lane.spool_id == 159 and u._held_lanes == {}
        lane.spool_id, lane.material = None, ""        # the user cleared it
        u._prime_scan_baseline()                        # a reconnect
        assert lane.spool_id is None
        assert u.afc.bound == [("lane15", 159, False)]
        assert u.finalized == [(0, False, True)]

    def test_a_new_connection_forgets_the_restored_bays(self):
        u = types.SimpleNamespace(_restored_bays={0, 2})
        afcBambuAMS._reset_lookup_state(u)
        assert u._restored_bays == set()

    def test_a_record_that_names_no_spool_gives_no_profile(self):
        # Temperatures left in a record with no link and no material are the
        # last spool's: the defaults the next untagged spool gets fill only
        # blanks, so they would stay under them.
        rec = {"spool_id": None, "material": "", "color": "", "weight": 0,
               "extruder_temp": 250.0, "bed_temp": 100.0, "sub_type": "HF",
               "runout_lane": "lane16", "need_purge": True}
        u, lane = _claimed({"lane15": rec})
        assert (lane.extruder_temp, lane.bed_temp, lane.sub_type) == (
            None, None, "")
        assert (lane.runout_lane, lane.need_purge) == ("lane16", True)
        assert u._restore_claimed_lane_vars(lane, rec) is False

    def test_a_blank_variant_is_filled_and_a_set_one_kept(self):
        u, lane = _unit({"lane15": dict(self.REC)})
        assert u._restore_claimed_lane_vars(lane) is True
        assert lane.sub_type == "Matte"
        lane.sub_type = "Silk"
        u._restore_claimed_lane_vars(lane)
        assert lane.sub_type == "Silk"


class TestTheToolheadLaneAtTheClaim:
    """A lane AFC records in a toolhead gets from its unit's claim what PREP
    gives it at boot, which it missed while pooled: tool_loaded, the sync to
    the active extruder, and the Snapmaker print task config."""

    def _setup(self, current="lane24", moved=False):
        ch_afc = _Afc()
        u = _Unit("Alpha", ["lane24", "lane25"], [], ch_afc, _Log())
        u.lanes = {n: _Lane(n, ch_afc, u) for n in u.lane_names}
        ch_afc.tools = {"extruder": types.SimpleNamespace(
            name="extruder", lane_loaded="lane24")}
        ch_afc.function.get_current_lane = lambda: current
        params = []
        ch_afc.spool.set_snapmaker_filament_params = lambda ln: params.append(
            (ln.name, ln.material, ln.tool_loaded))
        u._master = types.SimpleNamespace(
            loaded_lane_moved=lambda name: moved)
        u._held_lanes = {"lane24": {"material": "PLA", "spool_id": None,
                                    "tool_loaded": True}}
        return u, params

    def test_the_loaded_lane_is_restored_as_prep_restores_it(self):
        u, params = self._setup()
        afcBambuAMS._apply_held_lanes(u)
        lane = u.lanes["lane24"]
        assert lane.tool_loaded is True and lane.synced == 1
        assert params == [("lane24", "PLA", True)]
        assert u.lanes["lane25"].tool_loaded is False

    def test_only_the_active_toolheads_lane_is_synced(self):
        u, params = self._setup(current="lane9")
        afcBambuAMS._apply_held_lanes(u)
        assert u.lanes["lane24"].tool_loaded is True
        assert u.lanes["lane24"].synced == 0

    def test_a_record_the_master_says_is_another_units_is_left_alone(self):
        u, params = self._setup(moved=True)
        afcBambuAMS._apply_held_lanes(u)
        assert u.lanes["lane24"].tool_loaded is False and params == []
        assert u.logger.warnings == []            # the master says it

    def test_a_lane_not_saved_loaded_under_this_unit_is_left_alone(self):
        # AFC saves an extruder's loaded lane by name alone: another unit
        # claimed onto the bay then, or none, loaded it.
        u, params = self._setup()
        u._held_lanes = {"lane24": {"material": "PLA", "spool_id": None,
                                    "tool_loaded": False}}
        afcBambuAMS._apply_held_lanes(u)
        lane = u.lanes["lane24"]
        assert (lane.tool_loaded, lane.synced, params) == (False, 0, [])
        assert u.logger.warnings == [
            "AFC bambu Alpha: extruder records lane24 as loaded, but lane24 "
            "was not saved loaded under this unit (None), so the filament in "
            "extruder is not from this unit and lane24 is left unloaded. "
            "Unload that filament by hand and run UNSET_LANE_LOADED to clear "
            "the record."]
        # Nor does the follower restore take it as this unit's.
        assert afcBambuAMS._toolhead_record_is_mine(u, "lane24") is False
        u._held_lanes = {}
        afcBambuAMS._apply_held_lanes(u)
        assert afcBambuAMS._toolhead_record_is_mine(u, "lane24") is False

    def test_the_follower_restore_follows_the_claims_record(self):
        engaged = []
        u = types.SimpleNamespace(
            name="Alpha", logger=_Log(), _bridge=object(), _id_resolved=True,
            _master=types.SimpleNamespace(loaded_lane_moved=lambda n: False),
            lanes={"lane24": types.SimpleNamespace(name="lane24",
                                                   tool_loaded=False)},
            afc=types.SimpleNamespace(tools={"extruder": types.SimpleNamespace(
                lane_loaded="lane24")}, reactor=_Clock()),
            _startup_restore_loaded=lambda: engaged.append(True),
            _toolhead_records=set())
        assert afcBambuAMS._restore_loaded_follower(u) is False
        assert engaged == [] and u.lanes["lane24"].tool_loaded is False
        u._loaded_restore_done, u._toolhead_records = False, {"lane24"}
        assert afcBambuAMS._restore_loaded_follower(u) is True
        assert engaged == [True] and u.lanes["lane24"].tool_loaded is True

    def test_a_unit_no_master_hands_records_trusts_afcs_record(self):
        u, params = self._setup()
        u._held_lanes = None
        afcBambuAMS._apply_held_lanes(u)
        assert u.lanes["lane24"].tool_loaded is True
        assert afcBambuAMS._toolhead_record_is_mine(u, "lane24") is True


def _save_var(ch):
    """AFC.save_vars as the chain's printer runs it: every unit's registered
    lanes, as get_status(save_to_file=True) gives them (a pooled bay's
    lanes are not registered, so the bay is saved empty), and each
    extruder's loaded lane."""
    data = {}
    for pu in ch.m._pool_units:
        data[pu["name"]] = {ln: ch.lanes[ln].get_status(save_to_file=True)
                            for ln in pu["lanes"]
                            if not ch.lanes[ln].unassigned}
    data["system"] = {"extruders": {
        name: {"lane_loaded": ext.lane_loaded}
        for name, ext in ch.afc.tools.items()}}
    ch.write_var(data)
    return data


class TestARestartBeforeScanPrimingLosesNothing:
    """A claim's first save writes AFC.var.unit before the unit's scan
    priming (8 s after the claim, up to ~38 s while the unit owes re-reads).
    A restart in that window boots on that file: the lanes it saved are the
    records the claim put on them, not blank lanes."""

    SPOOLS = {"lane24": {"spool_id": 159, "material": "PLA",
                         "color": "#0086D6", "weight": 412.0,
                         "extruder_temp": 225.0, "sub_type": "Matte",
                         "runout_lane": "lane25"},
              "lane25": {"material": "PETG", "color": "#FF0000",
                         "weight": 800.0, "extruder_temp": 245.0},
              "lane26": {"spool_id": 12, "material": "ABS",
                         "color": "#000000", "weight": 950.0}}

    def _boot(self, tmp_path, var, loaded=None):
        ch = _chain(tmp_path, var=var, owners="AAAA:Alpha")
        if loaded:
            ch.afc.tools = {"extruder": types.SimpleNamespace(
                name="extruder", lane_loaded=loaded)}
        return ch

    def _first_var(self):
        var = {"Alpha": {ln: _rec(f"T{ln[4:]}", **rec)
                         for ln, rec in self.SPOOLS.items()}}
        var["Alpha"]["lane24"]["tool_loaded"] = True
        return var

    def _check(self, lanes):
        for ln, rec in self.SPOOLS.items():
            lane = lanes[ln]
            for key, val in rec.items():
                assert getattr(lane, key) == val, (ln, key)

    def test_the_records_survive_two_restarts_in_the_window(self, tmp_path):
        ch = self._boot(tmp_path, self._first_var(), loaded="lane24")
        ch.claim(A)                       # the claim's own save
        self._check(ch.lanes)
        saved = _save_var(ch)
        for ln, rec in self.SPOOLS.items():
            for key, val in rec.items():
                assert saved["Alpha"][ln][key] == val, (ln, key)
        assert saved["Alpha"]["lane24"]["tool_loaded"] is True
        # Klipper restarts before the unit primed: boot on that file, twice.
        for n in range(2):
            ch = self._boot(tmp_path / ".." / tmp_path.name, None,
                            loaded="lane24")
            assert set(ch.m._held["Alpha"]["lanes"]) >= set(self.SPOOLS)
            ch.claim(A)
            self._check(ch.lanes)
            assert ch.lanes["lane24"].tool_loaded is True
            _save_var(ch)

    def test_untagged_and_tagged_bays_keep_them_after_priming(self,
                                                              tmp_path):
        ch = self._boot(tmp_path, self._first_var())
        ch.claim(A)
        _save_var(ch)
        ch = self._boot(tmp_path, None)
        ch.claim(A)
        unit = ch.units["Alpha"]
        # lane24 tagged with the same spool, lane25 untagged, lane26 tagged
        # with the same black ABS, lane27 empty.
        unit.bays({"present": True, "material": "PLA Matte",
                   "color": "0086D6", "rfid_uid": "0A1B2C3D"},
                  {"present": True},
                  {"present": True, "material": "ABS", "color": None,
                   "color_black": True, "rfid_uid": "0A1B2C3E"},
                  {"present": False})
        unit._prime_scan_baseline()
        self._check(ch.lanes)
        assert unit._afc_owned == {0, 2} and unit._restored_bays == {1}
        saved = _save_var(ch)
        assert saved["Alpha"]["lane24"]["spool_id"] == 159
        assert saved["Alpha"]["lane26"]["spool_id"] == 12
        assert saved["Alpha"]["lane25"]["material"] == "PETG"
