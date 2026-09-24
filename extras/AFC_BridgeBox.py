# AFC BambuAMS chain master -- one section that fabricates the rest.
#
# Copyright (C) 2026 J0eB0l
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Chain auto-configuration. A hand-written BambuAMS setup is three
# boilerplate sections per unit -- [AFC_BambuAMS <name>], one [AFC_lane] per
# slot, an [AFC_hub] with a virtual switch -- and every value in them follows
# from which units are on the chain and which extruder/buffer the chain's
# master feeds. This module writes those sections itself, the same way
# AFC_utils.add_filament_switch fabricates filament_switch_sensor sections:
# build a RawConfigParser, wrap it, printer.load_object() each section. To
# every other module the result is indistinguishable from printer.cfg text.
#
#   [AFC_BridgeBox chain1]
#   serial_port: /dev/serial/by-id/usb-Raspberry_Pi_Pico_XXXX-if00
#   extruder: extruder
#   buffer: Bamb_1
#   lane_base: 24
#   roster: ht:872C3B871C00B0084A343331
#
# Sections must exist at config parse, before the serial port is usable, so
# the roster comes from config plus a persisted roster file the bridge
# updates at runtime; units it records are fabricated at the next restart,
# and pooled spares (pool_ams / pool_ht) cover units that appear live.
#
# Roster format: an HT is distinguishable at enrollment (it answers on
# device 0x1800), but AMS 1 and AMS 2 Pro are both
# 0x0700 and identical until their first cycle end names the dialect. Lane
# count does not depend on that -- every boxed unit takes four lanes, an HT
# one -- so a roster entry is `<model>:<unit_uid>`, model from
# _SLOTS_BY_MODEL, and a boxed unit of unknown generation enrolls as `boxed`
# and is refined later without its lanes moving.
#
# Unit names are pinned per uid in a persisted map, tombstoned forever, and
# a unit's lanes follow from its name's rank within its family: a unit keeps
# its lanes across restarts, roster edits, and even its own removal -- so
# removing a mid-roster unit never renumbers the survivors (lane names carry
# Spoolman bindings and T# macros), and a unit that returns gets its old
# lanes, and with them its bindings, back.
from __future__ import annotations

import configparser
import os
from typing import Any, Dict, List, Optional, Tuple

from extras.AFC_BambuAMS import _AMS_MODELS, _HT_MODELS

#: Every option the chain master consumes itself. Anything else in a
#: [AFC_BridgeBox <master>] section is a chain-wide default for the units it
#: fabricates (see _chain_defaults), and learned-value folding must never let a
#: leftover file rewrite identity or wiring. A test re-reads __init__'s
#: config.get* calls and fails if one is missing here.
_MASTER_OPTIONS = frozenset({
    "serial_port", "tcp_key", "extruder", "unit_prefix", "removal_grace",
    "buffer_chip_name", "buffer", "buffer_type", "lane_base", "pool_ams",
    "pool_ht",
    "ams_names", "ht_names", "auto_drop", "release_grace", "release_settle",
    "enroll_grace", "claim_grace", "flap_claim_grace", "flap_window",
    "afc_bowden_length", "td1_bowden_length", "dry_max_temp", "auto_vars_file",
    "state_file", "roster", "hotplug_poll",
})

_STRUCTURAL_KEYS = frozenset(
    {"serial_port", "tcp_key", "unit_uid", "unit", "hub", "extruder", "buffer",
     "ams_model", "switch_pin", "sensor_type", "bambu_unit"})

# Model facts come from AFC_BambuAMS's _AMS_MODELS, the one model table: the
# tags a roster may carry are exactly the ams_model values a unit accepts.
# `boxed` is a 0x0700 unit whose generation is not yet confirmed (same lane
# count either way, so enrollment never has to guess); `lite` is reserved for
# the AMS Lite, which is neither implemented nor tested.

#: Lane count per roster model tag: an HT has one bay, every boxed unit four.
_SLOTS_BY_MODEL = {m: (1 if m in _HT_MODELS else 4) for m in _AMS_MODELS}


def _norm_model(name: Optional[str]) -> str:
    """
    A model tag as written, trimmed and lower-cased.

    :param name: a model tag, possibly None
    :return str: the normalised tag (unknown names are left as they are)
    """
    return (name or "").strip().lower()


def _norm_uid(uid: Optional[str]) -> str:
    """
    A unit uid trimmed and upper-cased.

    :param uid: a unit uid, possibly None
    :return str: the normalised uid, "" for None
    """
    return (uid or "").strip().upper()


def _pool_laneno(pu: Dict[str, Any]) -> int:
    """
    Sort key: the first lane number of a pool unit.

    :param pu: pool unit record
    :return int: lane number, large when unnumbered
    """
    ds = "".join(c for c in (pu.get("lanes") or ["0"])[0] if c.isdigit())
    return int(ds) if ds else 1 << 30

#: Models whose unit section carries heater: True (the HT and the AMS 2 Pro).
#: `boxed` gets no heater until its generation is confirmed: a heater key on
#: an AMS 1 invites commands the unit ignores.
_HEATED_MODELS = {m for m, spec in _AMS_MODELS.items() if spec[0]}

#: Drying ceiling per heated model, so the panel never commands a temperature
#: the hardware does not honour.
_DRY_CEILING = {m: _AMS_MODELS[m][3] for m in _HEATED_MODELS}

#: Models whose fabricated section arms measure_on_insert. See the emission
#: site for why this is an allow-list rather than every model.
_HT_MODELS_BB = set(_HT_MODELS)


# ── pooled lanes ────────────────────────────────────────────────────────────
# A pooled lane is an [AFC_lane] with `unassigned: True`. AFC_lane reads that
# flag and keeps the lane out of every registry at connect; these three move it
# in and out of them as a unit claims and releases it. They live here rather
# than on AFCLane so AFC_lane.py carries only the flag.

def activate_from_pool(lane: Any) -> None:
    """
    Register a pooled lane into the registries AFC_lane.handle_connect skipped.

    Mirrors the add_to_other_obj block in handle_connect. Always re-asserts
    the writes (they are idempotent) rather than skipping on the flag, so a
    re-claimed unit always gets its lanes back.
    """
    from extras.AFC_lane import EXCLUDE_TYPES
    lane.unassigned = False
    if not (lane.unit_obj.type not in EXCLUDE_TYPES
            or (lane.unit_obj.type in EXCLUDE_TYPES
                and "AFC_lane" in lane.fullname)):
        return
    lane.unit_obj.lanes[lane.name] = lane
    lane.afc.lanes[lane.name] = lane
    if lane.hub_obj is not None and not lane.is_direct_hub():
        try: lane.hub_obj.lanes[lane.name] = lane
        except Exception: pass
    if lane.extruder_obj is not None:
        lane.extruder_obj.lanes[lane.name] = lane
        try: lane.extruder_obj.check_lanes()
        except Exception: pass
    if lane.buffer_obj is not None:
        try: lane.buffer_obj.lanes[lane.name] = lane
        except Exception: pass
        # get_status publishes the name; without it the panel shows "-:-".
        if lane.buffer_name is None:
            lane.buffer_name = lane.buffer_obj.name


def assign_pool_tcmd(lane: Any, afc: Any = None) -> None:
    """
    Give a claimed pool lane its T# command without registering it twice.

    A lane claimed before PREP, or back on a release -> re-claim, already has
    its T# registered to AFC's CHANGE_TOOL. TcmdAssign would register it again,
    Klipper refuses ("already setup"), and register_tool_macro logs that as a
    mapping conflict. When every T# on the lane is already ours, only the
    lookup table TcmdAssign would have filled is updated; anything else goes
    through TcmdAssign as normal.
    """
    afc = afc if afc is not None else lane.afc
    handlers = getattr(getattr(afc, "gcode", None), "ready_gcode_handlers",
                       None) or {}
    change_tool = getattr(afc, "cmd_CHANGE_TOOL", None)
    # Compare the underlying function: cmd_CHANGE_TOOL is a bound method, and
    # each attribute access builds a new one, so `is` on it never matches.
    ours = getattr(change_tool, "__func__", change_tool)
    maps = [m for m in (getattr(lane, "map", None) or []) if m != "NONE"]
    if (ours is not None and maps
            and all(getattr(handlers.get(m), "__func__", handlers.get(m)) is ours
                    for m in maps)):
        for m in maps:
            afc.tool_cmds[m] = lane.name
        if not lane.current_map:
            lane.current_map = maps[0]
        return
    afc.function.TcmdAssign(lane)


def restore_pool_spool(lane: Any) -> bool:
    """
    Put back the spool identity deactivate_to_pool() wiped.

    The wipe is right for a lane going back to the generic pool but wrong
    for a bay held for a re-plug, which is the common case: the bridge
    dropping off USB for longer than the release grace (e.g. an OTA flash)
    releases every unit, and without this the re-claim seconds later would
    bring the lanes back blank (no spool, no colour, no measurements).

    The var file cannot do this job: a released lane is popped from
    unit_obj.lanes, and save_vars() walks exactly that, so the lane's
    record is dropped from AFC.var.unit within a second of the release.
    The snapshot taken at release is the only surviving copy.

    The snapshot is deliberately session-lived. A restart while a unit is released must come up with the lane
    empty: the unit was not on the chain when the restart began, so it is
    not on the chain when the restart finishes, and inventing a spool for a
    bay nobody can see is worse than an empty bay. Only a re-plug inside
    the same session -- where the release and the claim are two halves of
    one event -- has anything to carry across.

    Re-binding through set_spoolID (not a bare attribute write) is what
    AFC_prep does at startup, so the re-claimed lane refreshes from
    Spoolman the same way a booted one does.

    :return bool: True if a snapshot was restored, False if there was none
    """
    snap = getattr(lane, "_pool_spool", None)
    if not snap:
        return False
    lane._pool_spool = None
    lane.spool_id  = snap.get("spool_id")
    lane._material = snap.get("material")
    lane.color     = snap.get("color") or ""
    lane.weight    = snap.get("weight") or 0.
    if lane.spool_id and getattr(lane.afc, "spoolman", None) is not None:
        try:
            lane.afc.spool.set_spoolID(lane, lane.spool_id,
                                       save_vars=False)
        except Exception:
            pass
    return True


def deactivate_to_pool(lane: Any) -> None:
    """
    Reverse activate_from_pool: drop this lane from every registry and
    return it to the inert pool state (empty map, no spool). The mirror of
    release on the AFC_BridgeBox side.

    The spool identity is snapshotted on the way out so a re-plug of the
    same unit can put it back -- see restore_pool_spool().
    """
    lane._pool_spool = {
        "spool_id": lane.spool_id,
        "material": lane._material,
        "color":    lane.color,
        "weight":   lane.weight,
    }
    for reg in (getattr(lane.unit_obj, "lanes", None),
                lane.afc.lanes,
                getattr(lane.hub_obj, "lanes", None),
                getattr(lane.extruder_obj, "lanes", None),
                getattr(lane.buffer_obj, "lanes", None)):
        try:
            if isinstance(reg, dict):
                reg.pop(lane.name, None)
        except Exception:
            pass
    lane.map = []
    lane._map = []
    lane.current_map = ""
    lane.spool_id = None
    lane._material = None
    lane.color = ""
    lane.weight = 0.
    # _load_state, not load_state: the public name is a read-only property,
    # and an AttributeError here would be swallowed by the caller and skip
    # the `unassigned = True` below.
    lane._load_state = False
    lane.unassigned = True


class afcBridgeBox:
    """The chain master: parses the roster and fabricates unit sections."""

    def __init__(self, config: Any) -> None:
        """
        Set up the chain master: transport, roster, pool units and commands.

        :param config: the [AFC_BridgeBox <name>] section
        """
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.serial_port = config.get("serial_port")
        # Passed to fabricated units alongside serial_port: the key
        # belongs to the link, so every unit on one bridge shares it.
        self.tcp_key = config.get("tcp_key", None)
        self.extruder = config.get("extruder")
        # The chain's buffer. One chain feeds one extruder through one
        # buffer, so the buffer belongs to the master the same way the
        # extruder does: name an existing [AFC_buffer] to use it, or leave
        # unset and the master fabricates its own -- an FPS_PSF reading the
        # bridge's virtual ADC chip, wired to every unit on the chain.
        self.unit_prefix = config.get("unit_prefix", "Bambu_AMS")
        # How long a rostered unit must be continuously absent from a live
        # chain before its removal is recorded (applied at the next
        # restart). The online flag flips within ~1.5s and transient
        # absences (power-cycling, re-enrollment) resolve in seconds, so two
        # minutes offline on an otherwise-answering chain means unhooked.
        # 0 disables auto-removal.
        self.removal_grace = config.getfloat("removal_grace", 120.0,
                                             minval=0.0)
        self._missing_since: Dict[str, float] = {}
        # UIDs forgotten while still physically online. The scout treats any
        # online uid it does not know as a fresh arrival and would re-enroll and
        # re-claim it within a tick or two -- so a live FORGET parks the uid here
        # and both halves skip it, until it is genuinely pulled (drops out of the
        # online set) and cleared, at which point a real re-plug enrolls it fresh.
        self._forget_suppressed: set = set()
        self.buffer_chip_name = config.get("buffer_chip_name", "bambu_buffer")
        # The fabricated buffer's type. "FPS_PSF" is the plain tension-follower;
        # "bambu" is the same follower plus the
        # odometer gate, which lets the buffer stand in for a toolhead sensor
        # (see AFC_BambuAMS_buffer.AFCBambuBuffer). Opt-in because "bambu" makes
        # buffer_triggered demand that the odometer moved and then stopped --
        # right for a unit that meters its own moves, wrong for anything whose
        # odometer this host cannot read. Ignored when a hand-written
        # [AFC_buffer] is adopted below; that section names its own type.
        self.buffer_type = config.get("buffer_type", "FPS_PSF")
        self.buffer = config.get("buffer", None)
        self._fabricate_buffer = self.buffer is None
        if self._fabricate_buffer:
            # Adopt before fabricating. A hand-written [AFC_buffer] already
            # reading this chain's chip is the same physical buffer, and a
            # second section on the same adc pin halts klippy ("pin fps used
            # multiple times in config"). Only a config with no such buffer
            # gets one fabricated.
            adopted = self._find_chip_buffer(config)
            if adopted:
                self.buffer = adopted
                self._fabricate_buffer = False
            else:
                self.buffer = f"{self.unit_prefix}_Buffer"
        # 0 (the default) = the next lane number after every lane the config
        # already declares -- computed once and then remembered in the store,
        # because "next available" must not mean "renumbers when the config
        # grows". A lane name is what Spoolman bindings and T# macros hang
        # off; a base that moved with later config edits would orphan all of
        # them. Resolution: explicit option > stored base > computed.
        self.lane_base = config.getint("lane_base", 0)
        # Live hot-enroll pool. Klipper freezes its object graph at config parse, so
        # pool_ams four-slot and pool_ht single-slot units are fabricated at boot as
        # invisible spares; a UID appearing on the chain claims one live (lanes, T#,
        # panel) and releases it on removal. Roster units fill their slots by name.
        self.pool_ams = config.getint("pool_ams", 4, minval=0)
        self.pool_ht = config.getint("pool_ht", 8, minval=0)
        # Optional custom unit names, assigned in enrollment order.
        # ams_names[0] is the 1st AMS to be named, ams_names[1] the 2nd, ...;
        # ht_names does the same for HTs. Past the end of a list a unit falls
        # back to Bambu_AMS_# / Bambu_AMS_HT_#. A unit keeps its name once given
        # (identity is stable -- coming or going never renumbers a survivor);
        # editing these lists names the next new unit, and FORGET frees a name
        # for the next unit to reuse.
        self.ams_names = [n.strip() for n in
                          (config.get("ams_names", "") or "").split(",")
                          if n.strip()]
        self.ht_names = [n.strip() for n in
                         (config.get("ht_names", "") or "").split(",")
                         if n.strip()]
        # Auto-drop on removal. chain_uids stays sticky after an unplug, but the
        # per-unit online flag flips false within ~1s, so a claimed unit whose flag
        # stays false for release_grace is released live. auto_drop: False leaves the
        # lanes until the next restart or AFC_BAMBU_RELINK.
        self.auto_drop = config.getboolean("auto_drop", False)
        # Continuous seconds a claimed unit's online flag must stay false before
        # its lanes drop. Rides out any brief flap; a re-plug resets the clock.
        self.release_grace = config.getfloat("release_grace", 10.0, minval=2.0)
        # Seconds a bound unit must hold continuously online before an online
        # read cancels a pending release. A physically-absent unit can leave a
        # phantom online flag that flaps true every second or two; only a
        # sustained run clears the release clock -- symmetric to
        # flap_claim_grace on the claim side. Kept well under release_grace so
        # a genuine re-plug still cancels the drop in time.
        self.release_settle = config.getfloat("release_settle", 5.0, minval=0.0)
        # Seconds a new uid must hold continuously online before the scout
        # writes it into the recorded roster, so a phantom online flag (a
        # pulled unit the bridge still flaps online) cannot re-record a unit
        # the operator just forgot. A real plug-in holds online steadily and
        # enrolls after this; a ghost that spends most ticks offline never does.
        self.enroll_grace = config.getfloat("enroll_grace", 15.0, minval=0.0)
        # Anti-flap claim hysteresis. Normally a unit claims as soon as it is
        # seen online (claim_grace 0 -- keeps boot/first-plug instant, so the
        # known units still claim before PREP walks them). But right AFTER a
        # release, for flap_window seconds, it must hold continuously online for
        # flap_claim_grace before it may reclaim -- so a marginal/flapping link
        # (online stretches of up to ~12s) cannot keep re-claiming on a blip.
        # A cleanly re-seated unit holds online steadily and reclaims after
        # flap_claim_grace.
        self.claim_grace = config.getfloat("claim_grace", 0.0, minval=0.0)
        self.flap_claim_grace = config.getfloat("flap_claim_grace", 15.0,
                                                minval=0.0)
        self.flap_window = config.getfloat("flap_window", 120.0, minval=0.0)
        # How often the chain watch asks the bridge who is on the chain.
        # The firmware notices a plug in about a second (it enrolls off the
        # announce), so this interval is most of the hot-plug delay. Polling
        # is cheap: see the note in _scout_tick.
        self.hotplug_poll = config.getfloat("hotplug_poll", 1.0, minval=0.5)
        self.afc_bowden_length = config.getfloat("afc_bowden_length", 2100.0)
        self.td1_bowden_length = config.getfloat("td1_bowden_length", 850.0)
        # An int, and emitted as one: the consumer is AFC_BambuAMS's
        # config.getint("dry_max_temp"), and a fabricated "85.0" halts the
        # printer with "Unable to parse option". Every emitted value must take
        # the lexical form its consumer's getter parses.
        # 0 (the default) = each heated model's own ceiling from
        # _DRY_CEILING; a non-zero value overrides all heated units.
        self.dry_max_temp = config.getint("dry_max_temp", 0)
        # Prefix for fabricated unit names. Only needs setting when a second
        # [AFC_BridgeBox] chain exists -- two chains fabricating Bambu_AMS_HT_1
        # would collide, and the collision check refuses rather than resolves.
        # auto_vars_file: where AFC's ConfigRewrite persists learned values
        # (dw_len-adopted bowden lengths and the like); see _fold_and_sweep
        # for how they are folded into fabricated sections.
        self.auto_vars_file = os.path.expanduser(config.get(
            "auto_vars_file",
            "~/printer_data/config/AFC/AFC_auto_vars.cfg"))
        # One file holds everything this module persists -- the discovered
        # roster, the locked lane base, the learned values -- as a managed
        # comment block (like SAVE_CONFIG): every state line is prefixed #~#,
        # so klippy never parses it. That file is the one the operator's own
        # [AFC_BridgeBox] section lives in, found by grepping the config root
        # (from start_args) for the section header, since klippy does not say
        # which file a section came from. state_file: overrides this.
        sf = config.get("state_file", None)
        self.state_file = (os.path.expanduser(sf) if sf
                           else (self._locate_own_file()
                                 or os.path.expanduser(
                                     "~/printer_data/config/AFC/"
                                     "AFC_BridgeBox.cfg")))
        # The roster, from wherever it exists: the roster: option wins, then the roster
        # file written by a previous scout. Neither means scout mode: fabricate nothing,
        # bring the bridge up at ready, ask the chain who is on it, write the file and
        # say "RESTART to enroll".
        self._register_commands()
        self._migrate_legacy_state()
        # Chain-wide unit defaults, written on the master: any option the master does
        # not consume is a default for every unit this chain fabricates, at the lowest
        # precedence (master < model < unit, see _fold). Consumed, not just read, since
        # Klipper rejects a section with untouched options.
        self._chain_defaults: Dict[str, str] = {}
        try:
            for opt in config.get_prefix_options(""):
                if opt in _MASTER_OPTIONS or opt in _STRUCTURAL_KEYS:
                    continue
                self._chain_defaults[opt] = config.get(opt)
        except Exception:
            pass
        # Not logged here: self.logger is only assigned at connect, so it does
        # not exist yet in __init__. The fold reports what it applied.
        roster_raw = (config.get("roster", None) or "").strip()
        self._roster_source = "option"
        if not roster_raw:
            roster_raw = self._state_get(self._BASE_SECTION + " " + self.name,
                                         "roster") or ""
            self._roster_source = "file"
        if not roster_raw:
            self._roster_source = "scout"
            if not (self.pool_ams or self.pool_ht):
                # Bridge-only scout: no roster and no pool, so nothing to
                # fabricate -- just watch the chain and write what it reports.
                self.units: List[Dict[str, Any]] = []
                # Other config sections depend on a loaded unit: [AFC_buffer]
                # with `adc_pin: bambu_buffer:fps` needs the virtual pin chip a
                # unit registers, or klippy halts with "Unknown pin chip name
                # 'bambu_buffer'". So scout mode registers the same chip class,
                # backed by a shim whose buffer reads "no data" -- identical to
                # a unit whose Pico is absent, which everything downstream
                # already tolerates.
                try:
                    self._register_scout_chip(config)
                except Exception:
                    pass
                self.printer.register_event_handler(
                    "klippy:ready", self._scout_ready)
                return
            # Pool scout: no roster, but a pool is configured, so fabricate the
            # empty pool (pool_ams AMS bays + pool_ht HT bays, all free) and let
            # a plugged unit claim a named bay live -- the roster-free mode. A detected uid is still appended to the recorded
            # roster (so it enrolls on the next restart), but nothing has to be
            # rostered up front. The fabricated spare units register the real
            # bridge chip, so no scout shim is needed here.

        if not self.lane_base:
            self.lane_base = self._resolve_lane_base(config)
        # uid -> (first lane, span) and uid -> unit name. These maps make
        # removal safe: lanes and names are assigned per-uid once and
        # tombstoned forever, so removing a mid-roster unit cannot renumber
        # or rename the survivors -- lane names carry Spoolman bindings and
        # T# macros, unit names key the learned values -- and a unit that
        # returns gets its lanes, name, and calibration back. Tombstones are
        # never reused either: a new unit wearing a removed unit's name
        # would inherit that unit's learned bowden length.
        self._lane_map: Dict[str, Tuple[int, int]] = self._load_lane_map()
        self._name_map: Dict[str, str] = self._load_name_map()
        self._maps_dirty = False

        if self._roster_source == "scout":
            roster: List[Dict[str, Any]] = []      # pool scout: no known units
        else:
            try:
                roster = self._parse_roster(roster_raw)
            except ValueError as e:
                where = (f"state file {self.state_file}"
                         if self._roster_source == "file" else "roster:")
                error_str = f"[AFC_BridgeBox {self.name}] {where}: {e}"
                raise config.error(error_str)

        self.units: List[Dict[str, Any]] = roster
        sections = self._roster_sections(roster)
        self._flush_maps()           # persist any lanes/names just assigned
        sections = self._fold_and_sweep(config, sections)

        # Collisions are refused, not resolved: a fabricated section landing on a
        # hand-written one would silently shadow half a config. A bay name clashing
        # with another AFC unit is refused up front, before the duplicate-lane crash
        # ("LANE laneN already registered") it would otherwise cause.
        foreign = self._foreign_unit_names(config)
        for section, _keys in sections:
            if section.startswith("AFC_BambuAMS ") and \
                    section.split(" ", 1)[1] in foreign:
                nm = section.split(" ", 1)[1]
                raise config.error(
                    f"[AFC_BridgeBox {self.name}] pool bay name {nm!r} collides "
                    f"with an existing AFC unit of the same name -- rename that "
                    f"ams_names / ht_names entry to something unique. AFC "
                    f"identifies units by name, so a duplicate double-registers "
                    f"that unit's lanes and stops Klipper from starting.")
            existing = self.printer.lookup_object(section, None)
            if existing is not None:
                raise config.error(
                    f"[AFC_BridgeBox {self.name}] would fabricate [{section}] "
                    f"but it already exists in the config -- remove one")

        fileconfig = configparser.RawConfigParser()
        for section, keys in sections:
            fileconfig.add_section(section)
            for k, v in keys.items():
                fileconfig.set(section, k, str(v))

        import configfile
        # The third argument must be the live access-tracking dict.
        # ConfigWrapper records every option it is asked for into it, and
        # Klipper builds configfile's `settings` status from that
        # (configfile.py: _build_status_settings). With a throwaway {} a
        # fabricated section works but is missing from configfile.settings,
        # and Mainsail/Fluidd -- which read each section's sensor_type from
        # those settings before looking for the humidity object -- would show
        # a fabricated [temperature_sensor <unit>] without its humidity.
        tracking = self._live_access_tracking()
        for section, _keys in sections:
            wrapper = configfile.ConfigWrapper(
                self.printer, fileconfig, tracking, section)
            self.printer.load_object(wrapper, section)
        # Watch the chain even with units fabricated, so a unit plugged in
        # later announces itself. Registered after the fabrication loop on
        # purpose: the units registered their ready handlers during
        # load_object above, so they run first, and the first of them creates
        # and owns the bridge (start, variant, status prime). If the scout
        # created the bridge instead, every fabricated unit would take the
        # "sharing" path with no owner: no status prime, no pin, empty lanes.
        self.printer.register_event_handler("klippy:ready", self._scout_ready)

    # ── pure parts, split out so tests can exercise them directly ───────────

    @staticmethod
    def _parse_roster(raw: str) -> List[Dict[str, Any]]:
        """
        `model:uid, model:uid, ...` -> [{"model", "uid"}], validated.

        :param raw: the roster option's text
        :return list: one dict per unit, in roster order
        """
        units: List[Dict[str, Any]] = []
        seen: set = set()
        for entry in [e.strip() for e in raw.split(",") if e.strip()]:
            model, sep, uid = entry.partition(":")
            model = model.strip().lower()
            uid = _norm_uid(uid)
            if not sep or not uid:
                raise ValueError(
                    f"roster entry {entry!r} is not <model>:<unit_uid>")
            if model not in _SLOTS_BY_MODEL:
                raise ValueError(
                    f"roster entry {entry!r}: unknown model {model!r} "
                    f"(one of {sorted(_SLOTS_BY_MODEL)})")
            if uid in seen:
                error_str = f"roster lists uid {uid} twice"
                raise ValueError(error_str)
            seen.add(uid)
            units.append({"model": model, "uid": uid})
        if not units:
            error_str = "roster is empty -- nothing to fabricate"
            raise ValueError(error_str)
        return units

    def _roster_sections(
            self, roster: List[Dict[str, Any]]
    ) -> List[Tuple[str, Dict[str, Any]]]:
        """
        Every section the roster implies, in load order.

        Unit first, then its lanes, then its hub -- the same order a
        hand-written config uses. Cross-references (unit->hub, lane->unit)
        are names resolved at klippy:connect, so a unit may name a hub whose
        section loads after it.

        :param roster: parsed roster entries
        :return list: (section name, {key: value}) pairs
        """
        sections: List[Tuple[str, Dict[str, Any]]] = []
        # Every fabricated Bambu unit -- known (roster) and spare -- is a pool
        # slot: inert and invisible at boot, claimed onto its lanes live, and
        # released back on removal. This tracks them for the claim path.
        self._pool_units: List[Dict[str, Any]] = []
        # Names read like the hand-written ones (Bambu_AMS_HT_1, Bambu_AMS_1), numbered
        # per family. A uid keeps its first-assigned name via the persisted name map;
        # only a new uid draws the smallest number its family does not hold, tombstones
        # included, since learned values are keyed by name. AFC_BRIDGEBOX_FORGET frees one.
        # Fixed-band pool layout: the AMS band always reserves pool_ams * 4 lanes from
        # lane_base and the HT band starts past it, so a bay's lane depends on its
        # family rank and the declared pool, never on how many units are online.
        # Pass 1 enumerates bays, roster units first, then the spares; lanes are set in
        # pass 1c once names fix each bay's rank.
        bays: List[Dict[str, Any]] = []
        for u in roster:
            slots = _SLOTS_BY_MODEL[u["model"]]
            fam = "ht" if u["model"] in _HT_MODELS_BB else "ams"
            bays.append({"family": fam, "slots": slots,
                         "uid": u["uid"], "model": u["model"], "spare": False})
        have_ams = sum(1 for b in bays if b["family"] == "ams")
        have_ht = sum(1 for b in bays if b["family"] == "ht")
        spare_specs = ([("ams", 4)] * max(0, self.pool_ams - have_ams)
                       + [("ht", 1)] * max(0, self.pool_ht - have_ht))
        for fam, slots in spare_specs:
            bays.append({"family": fam, "slots": slots, "uid": None,
                         "model": "ht" if fam == "ht" else "boxed",
                         "spare": True})
        # Pass 1b -- name each bay, which also fixes its rank within its family.
        # A known uid keeps its persisted name (macros, Spoolman bindings and
        # learned values hang off it), so a unit coming or going never
        # renumbers a survivor. A uid with no name yet, and every spare,
        # draws the lowest unheld name of its family (ams_names / ht_names by
        # index, else the Bambu_AMS_# default). The rank is the index that minted
        # the name: Bambu_AMS_HT_1 is HT rank 0, always. A tombstone's name stays
        # held for its return; only FORGET releases it. Known units are named
        # first (roster order), so a real unit always takes the lower rank.
        used = set(self._name_map.values())

        def _draw_name(family: str) -> Tuple[str, int]:
            """
            Mint the first unused name in a family.

            :param family: unit family (HT or AMS)
            :return tuple: (name, rank)
            """
            i = 0
            while True:
                nm = self._name_for_index(family, i)
                if nm not in used:
                    used.add(nm)
                    return nm, i
                i += 1

        def _rank_of(family: str, name: str) -> int:
            """
            Recover the rank that minted a persisted name.

            :param family: unit family
            :param name: persisted unit name
            :return int: rank, 0 when off the list
            """
            for i in range(16):
                if self._name_for_index(family, i) == name:
                    return i
            return 0
        for b in bays:
            if b["spare"]:
                b["name"], b["rank"] = _draw_name(b["family"])
                continue
            nm = self._name_map.get(b["uid"])
            if not nm:
                nm, rank = _draw_name(b["family"])
                self._name_map[b["uid"]] = nm
                self._maps_dirty = True
            else:
                rank = _rank_of(b["family"], nm)
            b["name"], b["rank"] = nm, rank
        # Pass 1c -- lane from the fixed band. AMS rank r -> lane_base + r*4; HT
        # rank r -> ht_base + r (one lane each). The AMS band spans the declared
        # pool (pool_ams * 4 lanes), so the HT band base is fixed at
        # lane_base + pool_ams*4 and does not slide down when fewer AMS are
        # online. max(pool_ams, have_ams) only matters if MORE real AMS are
        # present than declared (a config past the ceiling): the band widens to
        # keep HT clear of them rather than colliding. In normal use
        # (have_ams <= pool_ams) it is exactly lane_base + pool_ams*4.
        ht_base = self.lane_base + max(self.pool_ams, have_ams) * 4
        for b in bays:
            if b["family"] == "ht":
                b["lane"] = ht_base + b["rank"]
            else:
                b["lane"] = self.lane_base + b["rank"] * 4
            if not b["spare"]:
                self._lane_map[b["uid"]] = (b["lane"], b["slots"])
                self._maps_dirty = True
        # Pass 2 -- fabricate the sections, in ascending lane order (AFC_lane
        # sections must load ascending; the buffer's virtual chip needs a unit
        # loaded first, handled after this loop).
        for b in sorted(bays, key=lambda x: x["lane"]):
            name, lane_no, slots = b["name"], b["lane"], b["slots"]
            unit_keys: Dict[str, Any] = {
                "serial_port": self.serial_port,
                "tcp_key": self.tcp_key,
                "ams_model": b["model"],
                "extruder": self.extruder,
                "hub": name,
                "auto_error_recovery": True,
                # A known HT measures on insert; a boxed unit does not (forcing
                # it can wedge an AMS 2 Pro mid-RFID auth). Spares stay off
                # until a claim decides. Override per
                # model with a serial_port-less [AFC_BridgeBox <model>] section.
                "measure_on_insert": (not b["spare"]
                                      and b["model"] in _HT_MODELS_BB),
                "buffer": self.buffer,
                # Fabricated inert, known and spare alike: real name/model/lanes
                # for stable identity/T#/learned values, but boots as an idle
                # pool slot claimed live by uid -- the same mechanism that makes
                # removal drop lanes live and re-plug restore them.
                "pool": True,
            }
            if not b["spare"]:
                unit_keys["unit_uid"] = b["uid"]
            if b["model"] in _HEATED_MODELS:
                unit_keys["heater"] = True
                unit_keys["dry_max_temp"] = (self.dry_max_temp
                                             or _DRY_CEILING[b["model"]])
            sections.append((f"{self._UNIT_SECTION} {name}", unit_keys))
            for slot in range(slots):
                sections.append((f"AFC_lane lane{lane_no + slot}",
                                 {"unit": f"{name}:{slot + 1}",
                                  "unassigned": True}))
            sections.append((f"AFC_hub {name}", {
                "switch_pin": "virtual",
                "afc_bowden_length": self.afc_bowden_length,
                "afc_unload_bowden_length": self.afc_bowden_length,
                "td1_bowden_length": self.td1_bowden_length,
            }))
            # Known units get a Mainsail/Fluidd temperature card (humidity comes
            # off every AMS's motion-long reply). A spare gets none, so it shows
            # nothing until claimed.
            if not b["spare"]:
                sections.append((f"temperature_sensor {name}", {
                    "sensor_type": "aht2x",
                    "bambu_unit": name,
                    "min_temp": 0,
                    "max_temp": 90,
                }))
            self._pool_units.append({
                "name": name,
                "lanes": [f"lane{lane_no + slot}" for slot in range(slots)],
                "family": b["family"],
                "uid": b["uid"],
                "bound": None,
                "spare": b["spare"],
            })
        if self._fabricate_buffer:
            # Last on purpose: the buffer's adc_pin references the virtual chip a unit
            # registers at load, so a unit section must load first ("Unknown pin chip
            # name" otherwise). Deadband 0.48; error sensitivity 0 because the AMS
            # meters its own moves, so jam detection here would false-alarm.
            sections.append((f"AFC_buffer {self.buffer}", {
                "type": self.buffer_type,
                "adc_pin": f"{self.buffer_chip_name}:fps",
                "deadband": 0.48,
                "filament_error_sensitivity": 0,
            }))
        return sections

    _BASE_SECTION = "AFC_BridgeBox"
    #: Prefix of the sections this module fabricates for its units. Distinct
    #: from _BASE_SECTION, which names this module's own section; learned
    #: values written under the wrong one are never read back by
    #: _fold_and_sweep.
    _UNIT_SECTION = "AFC_BambuAMS"

    def _find_chip_buffer(self, config: Any) -> Optional[str]:
        """
        The name of an existing [AFC_buffer] on this chain's chip, if any.

        :param config: this section's wrapper (for the merged fileconfig)
        :return: the buffer's name as written, or None
        """
        try:
            import re as _re
            fc = getattr(config, "fileconfig", None)
            for sec in (fc.sections() if fc else []):
                m = _re.match(r"(?i)afc_buffer\s+(\S+)$", sec)
                if not m:
                    continue
                pin = dict(fc.items(sec)).get("adc_pin", "")
                if pin.strip().lower().startswith(
                        self.buffer_chip_name.lower() + ":"):
                    return m.group(1)
        except Exception:
            pass
        return None

    def _override_sections(self, config: Any) -> Dict[str, Dict[str, str]]:
        """
        Operator overrides from serial_port-less [AFC_BridgeBox ...] sections.

        A section with serial_port is a chain master; one without is an
        override carrier. Three target grammars, most-specific wins:
        - a model tag ("ht", "ams1", "ams2", "boxed"; "lite" is reserved for
          the untested AMS Lite) applies to the unit section of every unit of
          that model on the chain;
        - a bare unit name ("Bambu_AMS_1") targets that unit's
          [AFC_BambuAMS] section;
        - a qualified name ("AFC_hub Bambu_AMS_1", "AFC_lane lane13")
          targets any fabricated section exactly.
        Targets nothing fabricates are ignored, so a removed unit's leftover
        override does not halt boot.

        :param config: this section's wrapper (for the merged fileconfig)
        :return dict: fabricated section name (or "model:<tag>") -> {k: v}
        """
        out: Dict[str, Dict[str, str]] = {}
        try:
            import re as _re
            fc = getattr(config, "fileconfig", None)
            for sec in (fc.sections() if fc else []):
                m = _re.match(r"(?i)afc_bridgebox\s+(.+)$", sec)
                if not m:
                    continue
                kv = dict(fc.items(sec))
                if kv.get("serial_port"):
                    continue                  # a chain master, not an override
                target = m.group(1).strip()
                canon = _norm_model(target)
                if canon in _SLOTS_BY_MODEL:
                    target = "model:" + canon
                elif " " not in target:
                    target = f"AFC_BambuAMS {target}"
                out.setdefault(target, {}).update(kv)
        except Exception:
            pass
        return out

    def _resolve_lane_base(self, config: Any) -> int:
        """
        The auto lane base: stored if ever resolved, else computed.

        Computed = one past the highest laneN the parsed config declares
        under [AFC_lane] or [AFC_stepper], case-insensitively -- the two
        section families lanes live in.

        When no numbered laneN sections exist (a machine whose toolhead lanes
        are named, e.g. [AFC_extruder e1] map: T1, not laneN), fall in behind
        the highest tool number (map: T<n>, or the number an [AFC_extruder eN]
        section carries) already assigned to any AFC lane / stepper /
        extruder instead of jumping to 24 -- so the fabricated pool
        continues the tool sequence with lane# == T# (T1/T2/T3 in use -> base 4
        -> lane4/T4). The 24 is only a last resort, when the config cannot be
        seen at all (no fileconfig) or nothing at all is numbered or mapped.

        :param config: this section's wrapper
        :return int: the base to allocate fabricated lanes from
        """
        stored = self._state_get(self._BASE_SECTION + " " + self.name,
                                 "lane_base")
        if stored:
            try:
                return int(stored)
            except Exception:
                pass
        base = 24
        try:
            import re as _re
            fc = getattr(config, "fileconfig", None)
            secs = list(fc.sections()) if fc else []
            # One past the highest numbered laneN section.
            nums = [int(m.group(1)) for sec in secs
                    for m in [_re.match(r"(?i)afc_(?:lane|stepper)\s+lane(\d+)$",
                                        sec)] if m]
            if nums:
                base = max(nums) + 1
            else:
                # No numbered lanes -- key off the highest tool number in use
                # (map: T<n>) so a named-lane machine's pool starts at the next
                # free T#, keeping lane# == T#, rather than the 24 fallback.
                tnums = []
                for sec in secs:
                    if not _re.match(r"(?i)afc_(?:lane|stepper|extruder)\s+", sec):
                        continue
                    try:
                        mv = fc.get(sec, "map")
                    except Exception:
                        mv = ""
                    tnums += [int(t) for t in _re.findall(r"[Tt](\d+)", mv or "")]
                    # A toolchanger's tools are numbered by their extruder
                    # sections ([AFC_extruder e0] .. e3 are T0..T3) even when
                    # no map: is written, since the T# is assigned at runtime.
                    me = _re.match(r"(?i)afc_extruder\s+\D*(\d+)$", sec)
                    if me:
                        tnums.append(int(me.group(1)))
                if tnums:
                    base = max(tnums) + 1
        except Exception:
            pass
        self._resolved_base = base            # _fold_and_sweep persists it
        return base

    def _foreign_unit_names(self, config: Any) -> set:
        """
        Unit names that existing AFC_lane / AFC_stepper sections in the config
        already belong to (their `unit:` field, before the ':slot').

        A fabricated pool bay named the same as one of these collides: AFC keys
        its `AFC_unit_<name>:connect` event by the bare unit name, so a
        duplicate name makes that unit's lanes register their mux commands
        twice and Klipper fails to boot (`... LANE laneN already registered`).
        BridgeBox's own fabricated lanes live in a private fileconfig and are not
        in this one, so only foreign units appear here.

        :param config: this section's wrapper (its fileconfig sees the merged
            user config)
        :return set: foreign unit names
        """
        import re as _re
        fc = getattr(config, "fileconfig", None)
        names: set = set()
        if fc is None:
            return names
        for sec in fc.sections():
            if not _re.match(r"(?i)afc_(?:lane|stepper)\s+\S+$", sec):
                continue
            try:
                unit = dict(fc.items(sec)).get("unit", "") or ""
            except Exception:
                unit = ""
            unit = unit.split(":")[0].strip()
            if unit:
                names.add(unit)
        return names

    def _load_lane_map(self) -> Dict[str, Tuple[int, int]]:
        """
        The persisted uid -> (first lane, span) map, tombstones included.

        Serialized in the state block as `lane_map: UID:start:span, ...`.
        Entries are never deleted, only added: an entry whose unit left the
        roster is a tombstone that keeps its lanes reserved, which is what
        makes removal safe (see _prune_missing) and return cheap.

        :return dict: uid -> (first lane number, lane count)
        """
        raw = self._state_get(self._BASE_SECTION + " " + self.name,
                              "lane_map") or ""
        out: Dict[str, Tuple[int, int]] = {}
        for entry in [e.strip() for e in raw.split(",") if e.strip()]:
            try:
                uid, start, span = entry.split(":")
                out[_norm_uid(uid)] = (int(start), int(span))
            except Exception:
                continue                      # one bad entry loses one entry
        return out

    def _load_name_map(self) -> Dict[str, str]:
        """
        The persisted uid -> unit name map -- the lane map's twin, same
        tombstone rules, serialized as `name_map: UID:Name, ...`.

        :return dict: uid -> fabricated unit name
        """
        raw = self._state_get(self._BASE_SECTION + " " + self.name,
                              "name_map") or ""
        out: Dict[str, str] = {}
        for entry in [e.strip() for e in raw.split(",") if e.strip()]:
            uid, sep, name = entry.partition(":")
            if sep and uid.strip() and name.strip():
                out[_norm_uid(uid)] = name.strip()
        return out

    def _name_for_index(self, family: str, i: int) -> str:
        """
        The name of the i-th bay of a family (0-based) -- the operator's
        ams_names / ht_names entry if one is set for that index, else the
        Bambu_AMS_# / Bambu_AMS_HT_# default. Names are assigned by rank within
        a family (lowest lane = index 0), so position defines the name.

        :param family: "ams" or "ht"
        :param i: the bay's 0-based rank within its family
        :return str: the bay name
        """
        if family == "ht":
            return (self.ht_names[i] if i < len(self.ht_names)
                    else f"{self.unit_prefix}_HT_{i + 1}")
        return (self.ams_names[i] if i < len(self.ams_names)
                else f"{self.unit_prefix}_{i + 1}")

    @staticmethod
    def _ser_maps(lane_map: Dict[str, Tuple[int, int]],
                  name_map: Dict[str, str]) -> Dict[str, str]:
        """
        :param lane_map: uid -> (first lane, span)
        :param name_map: uid -> unit name
        :return dict: both maps as their state-block key/value strings
        """
        lanes = ", ".join(
            f"{uid}:{start}:{span}"
            for uid, (start, span) in sorted(lane_map.items(),
                                             key=lambda kv: kv[1][0]))
        names = ", ".join(
            f"{uid}:{name_map[uid]}"
            for uid in sorted(name_map,
                              key=lambda u: lane_map.get(u, (1 << 30,))))
        return {"lane_map": lanes, "name_map": names}

    def _flush_maps(self) -> None:
        """Persist the lane and name maps when fabrication just grew them."""
        if not getattr(self, "_maps_dirty", False):
            return
        self._state_set({self._BASE_SECTION + " " + self.name:
                         self._ser_maps(self._lane_map, self._name_map)})
        self._maps_dirty = False

    def _register_mux(self, name: str, handler: Any, desc: str) -> None:
        """Register one CHAIN-muxed command. The first master to load also
        claims the no-CHAIN default so a single-chain operator never has to
        type CHAIN=; a second chain is reached via CHAIN=<its name>."""
        gcode = self.printer.lookup_object("gcode", None)
        if gcode is None:
            return
        gcode.register_mux_command(name, "CHAIN", self.name, handler, desc=desc)
        try:
            gcode.register_mux_command(name, "CHAIN", None, handler, desc=desc)
        except Exception:
            pass                  # a second chain: it is reached via CHAIN=

    def _register_commands(self) -> None:
        """Register the master's operator commands (CHAIN-muxed)."""
        self._register_mux(
            "AFC_BRIDGEBOX_FORGET", self.cmd_AFC_BRIDGEBOX_FORGET,
            "Release a departed unit's lane numbers, unit name, and learned "
            "values for reuse: AFC_BRIDGEBOX_FORGET UID=<uid> "
            "(or NAME=<unit name>) [FORCE=1]. With no argument, pops a picker "
            "of every recorded unit with a Forget button each.")
        self._register_mux(
            "AFC_BRIDGEBOX_ASSIGN", self.cmd_AFC_BRIDGEBOX_ASSIGN,
            "Pin a detected unit's UID onto a named pool bay, live: "
            "AFC_BRIDGEBOX_ASSIGN UID=<uid> NAME=<bay name> (or UID=<uid> alone "
            "to pop the bay picker). Refuses an occupied bay -- UNASSIGN it "
            "first.")
        self._register_mux(
            "AFC_BRIDGEBOX_UNASSIGN", self.cmd_AFC_BRIDGEBOX_UNASSIGN,
            "Unlink a UID from its pool bay (keeps learned values), live: "
            "AFC_BRIDGEBOX_UNASSIGN UID=<uid> (or NAME=<bay name>) [FORCE=1]")
        self._register_mux(
            "AFC_BRIDGEBOX_BAYS", self.cmd_AFC_BRIDGEBOX_BAYS,
            "Pop the bay manager: every pool bay, its occupant, and buttons "
            "to unassign/forget each -- AFC_BRIDGEBOX_BAYS")

    def cmd_AFC_BRIDGEBOX_FORGET(self, gcmd: Any) -> None:
        """
        The deliberate half of removal: erase a unit's tombstones.

        Auto-removal (see _prune_missing) only edits the roster and keeps
        the unit's lane numbers, unit name, and learned values reserved in
        case it returns. For a unit that is not coming back, this command
        releases them: the uid leaves the recorded roster and the lane and
        name maps, and every learned-value section under its fabricated names
        is erased (from the state block and auto_vars), so the freed name and
        lanes are safe for the next new unit to reuse.

        Works on a live unit too: it drops the unit's lanes/T# immediately and
        parks the uid on a suppress list (see _forget_suppressed) so the scout
        does not re-enroll the hardware still on the wire; the hold clears
        when the unit is physically pulled, so a genuine re-plug enrolls it
        fresh. Forgetting an online unit mid-print is refused (FORCE=1
        overrides), since it would pull a live lane out from under the job.

        :param gcmd: UID= or NAME= names the unit; FORCE=1 overrides the
                     mid-print refusal. With neither, pops a picker listing
                     every recorded unit, each with its own Forget button.
        """
        uid = _norm_uid(gcmd.get("UID", ""))
        name = (gcmd.get("NAME", "") or "").strip()
        sec = self._BASE_SECTION + " " + self.name
        lane_map = self._load_lane_map()
        name_map = self._load_name_map()
        if not uid and not name:
            # No target: pop a picker of every recorded unit (roster + the
            # name/lane tombstones a departed unit leaves), one Forget button
            # each. Live units are flagged so the operator can see which uids
            # are on the wire right now.
            roster_raw = self._state_get(sec, "roster") or ""
            order = [_norm_uid(e.partition(":")[2])
                     for e in roster_raw.split(",")
                     if e.partition(":")[2].strip()]
            known, seen = [], set()
            for u in order + list(name_map) + list(lane_map):
                u = _norm_uid(u)
                if u and u not in seen:
                    seen.add(u)
                    known.append(u)
            lines, buttons = [], []
            for u in known:
                nm = name_map.get(u) or "(unnamed)"
                live = "  -- LIVE (drops now)" if self._uid_online_now(u) \
                    else ""
                lines.append(f"{nm}  [{u[:8]}…]{live}")
                if len(buttons) < self._MAX_BAY_BUTTONS:
                    buttons.append((
                        f"Forget {nm}",
                        f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID={u}",
                        "error"))
            if not lines:
                lines = ["No recorded units to forget."]
            self._prompt("Forget a Bambu AMS unit", lines, buttons,
                         timeout=180.0)
            gcmd.respond_info(
                f"AFC_BridgeBox {self.name}: forget picker -- "
                + (", ".join(name_map.get(u) or u[:8] for u in known)
                   or "nothing recorded"))
            return
        if not uid and name:
            hits = [u for u, n in name_map.items() if n == name]
            if not hits:
                raise gcmd.error(
                    f"AFC_BRIDGEBOX_FORGET: no recorded unit named {name!r} "
                    f"(known: {', '.join(sorted(name_map.values())) or 'none'})")
            uid = hits[0]
        if not uid:
            raise gcmd.error(
                "AFC_BRIDGEBOX_FORGET: give UID=<unit_uid> or NAME=<unit name>")
        roster_raw = self._state_get(sec, "roster") or ""
        entries = [e.strip() for e in roster_raw.split(",") if e.strip()]
        in_roster = any(_norm_uid(e.partition(":")[2]) == uid
                        for e in entries)
        if uid not in lane_map and uid not in name_map and not in_roster:
            raise gcmd.error(
                f"AFC_BRIDGEBOX_FORGET: nothing recorded for uid {uid}")
        # A live FORGET drops the unit's lanes/T# now and holds the uid on the
        # suppress list (below). Mid-print it refuses without FORCE, matching
        # the auto-drop print gate.
        online_now = self._uid_online_now(uid)
        if online_now and self._is_printing() and not gcmd.get_int("FORCE", 0):
            raise gcmd.error(
                f"AFC_BRIDGEBOX_FORGET: {uid} is ONLINE and a print is active "
                f"-- dropping its lanes now would disrupt the print. Finish the "
                f"print, or FORCE=1 to forget it anyway")
        # Tell the bridge to forget it too, so it stops re-asserting the unit on
        # the bus. A re-assert frame is byte-identical to a reply, so a
        # gone-but-still-enrolled unit reads its own re-assert back and flaps
        # online indefinitely. The bridge re-learns the unit on a real replug.
        self._bridge_forget(uid)
        unit_name = name_map.pop(uid, None)
        start_span = lane_map.pop(uid, None)
        cp = self._read_state()
        if not cp.has_section(sec):
            cp.add_section(sec)
        if roster_raw:                # never invent an empty roster key
            cp.set(sec, "roster", ", ".join(
                e for e in entries
                if _norm_uid(e.partition(":")[2]) != uid))
        for k, v in self._ser_maps(lane_map, name_map).items():
            cp.set(sec, k, v)
        doomed = []
        if unit_name:
            doomed += [f"AFC_BambuAMS {unit_name}", f"AFC_hub {unit_name}",
                       f"temperature_sensor {unit_name}"]
        if start_span:
            doomed += [f"AFC_lane lane{n}"
                       for n in range(start_span[0], sum(start_span))]
        for section in doomed:
            if cp.has_section(section):
                cp.remove_section(section)
        self._write_state(cp)
        autov = self._read_ini(self.auto_vars_file)
        if autov is not None and any(autov.has_section(s) for s in doomed):
            for section in doomed:
                if autov.has_section(section):
                    autov.remove_section(section)
            try:
                with open(self.auto_vars_file, "w") as fp:
                    fp.write("# This file is autogenerated and updated when "
                             "variables are not in your normal AFC config "
                             "files\n\n")
                    autov.write(fp)
            except Exception:
                pass
        # This session keeps running on what was fabricated at boot; the
        # in-memory maps sync so a unit appearing later this session cannot
        # collide with the freed range.
        if hasattr(self, "_lane_map"):
            self._lane_map.pop(uid, None)
            self._name_map.pop(uid, None)
        self._missing_since.pop(uid, None)
        # Live reclaim: free the forgotten unit's held slot to the pool now,
        # not just at the next restart -- lanes off, T# unregistered, uid
        # cleared. The next hot-swapped unit of the same family claims those
        # lanes/T# immediately, taking over the slot's own object and name (no
        # rename, no reboot). A cross-generation unit (an ams2 onto a freed
        # ams1 slot) wears the slot's name until the next restart regularises it.
        pu = next((p for p in getattr(self, "_pool_units", [])
                   if _norm_uid(p.get("uid")) == uid), None)
        freed_live = False
        if pu is not None:
            if pu.get("bound"):
                self._release_pool_unit(pu["bound"])   # drop its lanes/T# live
            pu["uid"] = None                            # -> free pool slot
            # "learned values erased" includes the spool bindings the release
            # above snapshotted: forget means not coming back, so the next
            # claim on this slot starts from clean lanes.
            pu["lane_spool_uid"] = None
            freed_live = True
        # If it is still on the wire, suppress the scout's re-enroll and re-claim
        # for this uid until it is physically pulled -- otherwise the next tick
        # would treat it as a new arrival and put it right back.
        if online_now:
            self._forget_suppressed.add(uid)
        freed = []
        if start_span:
            lo, hi = start_span[0], sum(start_span) - 1
            freed.append(f"lane{lo}" if lo == hi else f"lanes {lo}-{hi}")
        if unit_name:
            freed.append(f"the name {unit_name}")
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: forgot {uid}"
            + (f" -- {' and '.join(freed)} freed for reuse, learned values "
               f"erased" if freed else "")
            + (" -- slot freed to the pool LIVE; the next same-family unit "
               "claims it with no reboot. Names regularise at the next restart."
               if freed_live else ". Applies at the next RESTART.")
            + (" It is still on the wire, so re-enroll is suppressed until you "
               "physically pull it -- a re-plug then enrolls it fresh."
               if online_now else "")
            + (" NOTE: your roster: option still lists this uid -- remove "
               "it there too, the option overrides the recorded roster."
               if self._roster_source == "option"
               and uid in {x["uid"] for x in self.units} else ""))
        self._close_prompt()          # a removal-popup button may have run this

    def _model_for_uid(self, uid: str) -> Optional[str]:
        """
        :param uid: 24-hex unit uid
        :return: the roster model tag recorded for this uid (ht/ams1/ams2/
            boxed), or None if it is not enrolled yet. Prefers the explicit
            roster: option, then the recorded roster file/ledger.
        """
        uid = _norm_uid(uid)
        for u in self.units:
            if _norm_uid(u.get("uid")) == uid:
                return u["model"]
        sec = self._BASE_SECTION + " " + self.name
        for e in (self._state_get(sec, "roster") or "").split(","):
            m, _sep, u = e.partition(":")
            if _norm_uid(u) == uid:
                return m.strip().lower()
        return None

    def _persist_pin(self, uid: str, first_lane: int, span: int,
                     name: str, model: str) -> None:
        """
        Persist a uid -> bay pin so it survives a restart: write the lane and
        name maps, and enroll the uid in the recorded roster if it is not
        there yet. In-memory maps are synced too so this session stays
        consistent. Learned values are untouched (they follow the name).

        :param uid: the unit's uid
        :param first_lane: the bay's first lane number
        :param span: 1 (HT) or 4 (AMS)
        :param name: the bay name the uid now owns
        :param model: the uid's roster model tag, for a fresh roster entry
        """
        sec = self._BASE_SECTION + " " + self.name
        lane_map = self._load_lane_map()
        name_map = self._load_name_map()
        lane_map[uid] = (first_lane, span)
        name_map[uid] = name
        cp = self._read_state()
        if not cp.has_section(sec):
            cp.add_section(sec)
        for k, v in self._ser_maps(lane_map, name_map).items():
            cp.set(sec, k, v)
        entries = [e.strip() for e in (self._state_get(sec, "roster") or ""
                                       ).split(",") if e.strip()]
        if not any(_norm_uid(e.partition(":")[2]) == uid for e in entries):
            entries.append(f"{model}:{uid}")
            cp.set(sec, "roster", ", ".join(entries))
        self._write_state(cp)
        self._lane_map[uid] = (first_lane, span)
        self._name_map[uid] = name
        # A deliberate re-add (ASSIGN) lifts any live-forget suppression: the
        # operator is putting this uid back on purpose, so the scout should stop
        # treating it as forgotten and let the release/claim logic run normally.
        if hasattr(self, "_forget_suppressed"):
            self._forget_suppressed.discard(uid)

    def _drop_pin(self, uid: str) -> None:
        """
        Remove a uid's lane/name pin from the persisted maps (its roster
        enrolment and learned values are kept), and sync the in-memory maps.
        The unit reverts to a floating spare: it re-claims onto the lowest
        free bay of its family, live and on the next restart alike.

        :param uid: the unit's uid
        """
        sec = self._BASE_SECTION + " " + self.name
        lane_map = self._load_lane_map()
        name_map = self._load_name_map()
        lane_map.pop(uid, None)
        name_map.pop(uid, None)
        cp = self._read_state()
        if cp.has_section(sec):
            for k, v in self._ser_maps(lane_map, name_map).items():
                cp.set(sec, k, v)
            self._write_state(cp)
        self._lane_map.pop(uid, None)
        self._name_map.pop(uid, None)

    def cmd_AFC_BRIDGEBOX_ASSIGN(self, gcmd: Any) -> None:
        """
        Pin a detected unit's UID onto a chosen named pool bay, live.

        The bays are named by lane position (ams_names / ht_names, or the
        Bambu_AMS_# / Bambu_AMS_HT_# defaults). Auto-claim lands a fresh unit
        on the lowest free bay of its family; this command instead binds a
        specific uid to a specific named bay so it always comes up there.
        If the unit is online now it moves onto that bay immediately (lanes +
        T# re-registered, no restart); the pin is persisted so it holds across
        reboots. Same-family only -- an HT bay is one lane, an AMS bay four.

        :param gcmd: UID=<uid> NAME=<bay name> [FORCE=1]
        """
        uid = _norm_uid(gcmd.get("UID", ""))
        name = (gcmd.get("NAME", "") or "").strip()
        if uid and not name:
            # No target named: pop the bay-picker for this uid (the same dialog
            # a fresh plug-in raises) so the operator can click a destination.
            if self._bay_of_uid(uid) is None:
                raise gcmd.error(
                    f"AFC_BRIDGEBOX_ASSIGN: {uid} is not on any bay -- plug it "
                    f"in first, or give NAME=<bay name> to pin it")
            self._prompt_new_unit(uid, timeout=180.0)   # manual: give time
            gcmd.respond_info(
                f"AFC_BridgeBox {self.name}: opened the bay picker for {uid}.")
            return
        if not uid or not name:
            raise gcmd.error(
                "AFC_BRIDGEBOX_ASSIGN: give UID=<unit_uid> and NAME=<bay name>")
        pools = getattr(self, "_pool_units", [])
        target = next((p for p in pools if p.get("name") == name), None)
        if target is None:
            bays = ", ".join(sorted(p["name"] for p in pools)) or "none"
            raise gcmd.error(
                f"AFC_BRIDGEBOX_ASSIGN: no pool bay named {name!r} "
                f"(bays: {bays})")
        model = self._model_for_uid(uid)
        fam = ("ht" if model == "ht" else "ams") if model else None
        if fam is not None and fam != target["family"]:
            raise gcmd.error(
                f"AFC_BRIDGEBOX_ASSIGN: {uid} is a {fam} unit but bay {name!r} "
                f"is a {target['family']} bay (their lane counts differ)")
        owner = _norm_uid(target.get("uid"))
        if owner and owner != uid:
            raise gcmd.error(
                f"AFC_BRIDGEBOX_ASSIGN: bay {name!r} is already assigned to "
                f"{owner} -- AFC_BRIDGEBOX_UNASSIGN it first")
        # Vacate the uid's current bay if it is sitting on a different one.
        cur = next((p for p in pools
                    if _norm_uid(p.get("uid")) == uid
                    and p is not target), None)
        if cur is not None:
            if cur.get("bound"):
                self._release_pool_unit(uid)
            cur["uid"] = None
        target["uid"] = uid
        first_lane = int("".join(c for c in target["lanes"][0] if c.isdigit()))
        span = len(target["lanes"])
        eff_model = model or ("ht" if target["family"] == "ht" else "boxed")
        self._persist_pin(uid, first_lane, span, name, eff_model)
        claimed = False
        if not target.get("bound") and self._uid_online_now(uid):
            if self._claim_pool_unit(uid, eff_model) is not None:
                claimed = True
        tools = (f"T{first_lane}" if span == 1
                 else f"T{first_lane}-T{first_lane + span - 1}")
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: assigned {uid} to bay {name!r} "
            f"(lane{first_lane}"
            + (f"-lane{first_lane + span - 1}" if span > 1 else "")
            + f", {tools})"
            + (" -- claimed LIVE, no restart." if claimed
               else " -- pinned; it claims this bay when next online.")
            + ("" if self._roster_source != "option"
               or uid in {x["uid"].upper() for x in self.units}
               else " NOTE: your roster: option is authoritative and does not "
               "list this uid, so the pin only fully holds at restart once the "
               "uid is in the option too."))
        self._close_prompt()          # a picker button ran this -- close it

    def cmd_AFC_BRIDGEBOX_UNASSIGN(self, gcmd: Any) -> None:
        """
        Unlink a UID from its pool bay -- the reverse of ASSIGN.

        Drops the name/lane pin so the unit reverts to a floating spare (it
        re-claims onto the lowest free bay of its family). Learned values and
        roster enrolment are kept -- this is not FORGET; re-assign or re-plug
        and its calibration is still there. Refuses a unit that is online now
        unless FORCE=1 (which drops its lanes live first).

        :param gcmd: UID=<uid> or NAME=<bay name> [FORCE=1]
        """
        uid = _norm_uid(gcmd.get("UID", ""))
        name = (gcmd.get("NAME", "") or "").strip()
        pools = getattr(self, "_pool_units", [])
        if not uid and name:
            pu = next((p for p in pools if p.get("name") == name), None)
            if pu is None:
                bays = ", ".join(sorted(p["name"] for p in pools)) or "none"
                raise gcmd.error(
                    f"AFC_BRIDGEBOX_UNASSIGN: no pool bay named {name!r} "
                    f"(bays: {bays})")
            uid = _norm_uid(pu.get("uid"))
            if not uid:
                raise gcmd.error(
                    f"AFC_BRIDGEBOX_UNASSIGN: bay {name!r} has no unit assigned")
        if not uid:
            raise gcmd.error(
                "AFC_BRIDGEBOX_UNASSIGN: give UID=<unit_uid> or NAME=<bay name>")
        pu = next((p for p in pools
                   if _norm_uid(p.get("uid")) == uid), None)
        if pu is None and uid not in self._lane_map and uid not in self._name_map:
            raise gcmd.error(
                f"AFC_BRIDGEBOX_UNASSIGN: nothing assigned for uid {uid}")
        if not gcmd.get_int("FORCE", 0) and self._uid_online_now(uid):
            raise gcmd.error(
                f"AFC_BRIDGEBOX_UNASSIGN: {uid} is ONLINE on the chain right "
                f"now -- unhook it first, or FORCE=1 to unassign it live "
                f"(drops its lanes)")
        was = pu.get("name") if pu is not None else self._name_map.get(uid)
        dropped = False
        if pu is not None:
            if pu.get("bound"):
                self._release_pool_unit(uid)
                dropped = True
            pu["uid"] = None
        self._drop_pin(uid)
        self._missing_since.pop(uid, None)
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: unassigned {uid}"
            + (f" from bay {was!r}" if was else "")
            + (" -- lanes dropped live" if dropped else "")
            + " (learned values kept). It now floats to the lowest free bay "
            "of its family.")
        self._close_prompt()          # a manager button ran this -- close it

    def cmd_AFC_BRIDGEBOX_BAYS(self, gcmd: Any) -> None:
        """
        Pop the bay manager: a dialog listing every pool bay with its occupant
        and buttons to act on the occupied ones (Unassign to unlink, live;
        Forget to free the bay for good). The manual entry point to the same
        assign/unassign the plug/unplug popups offer -- callable any time.

        :param gcmd: no parameters
        """
        pools = sorted(getattr(self, "_pool_units", []), key=_pool_laneno)
        lines, buttons = [], []
        for pu in pools:
            uid = _norm_uid(pu.get("uid"))
            live = " (live)" if pu.get("bound") else ""
            fam = "HT" if pu.get("family") == "ht" else "AMS"
            lines.append(f"{pu['name']} [{fam}]: "
                         + (f"{uid}{live}" if uid else "free"))
            if uid and len(buttons) < self._MAX_BAY_BUTTONS:
                buttons.append((
                    f"Unassign {pu['name']}",
                    f"AFC_BRIDGEBOX_UNASSIGN CHAIN={self.name} UID={uid} FORCE=1",
                    "warning"))
        if not lines:
            lines = ["No pool units configured (set pool_ams / pool_ht)."]
        # A manually-opened manager stays up longer than an event popup.
        self._prompt("Bambu AMS Units", lines, buttons, timeout=180.0)
        gcmd.respond_info(
            f"AFC_BridgeBox {self.name}: bay manager --\n  "
            + "\n  ".join(lines))

    def _chain_bridge(self) -> Any:
        """
        The bridge registered for our serial port, if any.

        :return Any: the BambuBridge, or None when none is registered
        """
        from extras import AFC_BambuAMS_bridge as _bridge_mod
        return _bridge_mod._BRIDGES.get(self.serial_port)

    def _uid_online_now(self, uid: str) -> bool:
        """
        :param uid: 24-hex unit uid
        :return bool: whether that unit is answering on the chain right now
        """
        try:
            bridge = (self._chain_bridge()
                      or getattr(self, "_bridge", None))
            if bridge is None or getattr(bridge, "_serial", None) is None:
                return False
            latest = bridge.latest_status() or {}
            online_idx = {u.get("n") for u in (latest.get("units") or [])
                          if u.get("online")}
            return any(i in online_idx
                       and _norm_uid(u) == uid
                       for i, u in enumerate(bridge.chain_uids()))
        except Exception:
            return False

    def _bridge_forget(self, uid: str) -> None:
        """
        Tell the bridge firmware to forget a uid: stop re-asserting it on the bus
        so a gone-but-still-enrolled unit stops reading its own re-assert back and
        flapping online. Best-effort; the bridge re-learns the unit on a real
        re-announce, and firmware without the command ignores it.

        :param uid: 24-hex unit uid
        """
        try:
            bridge = (self._chain_bridge()
                      or getattr(self, "_bridge", None))
            if bridge is not None:
                bridge.send({"cmd": "forget",
                             "uid": (uid or "").strip().lower()})
        except Exception:
            pass

    @staticmethod
    def _read_ini(path: str) -> Optional[configparser.RawConfigParser]:
        """
        :param path: INI file to read
        :return: parser, or None when absent/unreadable
        """
        if not os.path.exists(path):
            return None
        cp = configparser.RawConfigParser(delimiters=(":", "="))
        try:
            with open(path) as fp:
                cp.read_file(fp)
        except Exception:
            return None
        return cp

    def _fold_and_sweep(
            self, config: Any,
            sections: List[Tuple[str, Dict[str, Any]]]
    ) -> List[Tuple[str, Dict[str, Any]]]:
        """
        Close the loop between fabricated sections and AFC's learned values.

        ConfigRewrite files a learned key whose section is in no .cfg file
        into AFC_auto_vars.cfg under the section's name, and that file is
        klippy-parsed config. For a fabricated unit such a value would be
        invisible (its sections come from a private parser) and, once the
        fabricated name changed, klippy would instantiate a unit from the
        leftover at boot. This handles both:

        FOLD -- an auto_vars or store section matching a name fabricated now
        has its keys overlaid onto the fabricated section (structural keys
        excepted), so a learned bowden length survives a restart and reaches
        the unit that learned it.

        SWEEP -- folded auto_vars sections move into this module's own store
        file, which klippy never parses, and are deleted from auto_vars. An
        orphan -- an AFC_BambuAMS section in auto_vars whose name nothing
        fabricates and which exists in no other config file (its key set in
        the merged fileconfig equals its key set in auto_vars alone) -- is
        deleted too: it is a leftover for a renamed unit, and the renamed
        unit's values are already travelling under its new name.

        File edits take effect next boot (this boot already parsed
        auto_vars); the fail-soft unit makes the remaining session harmless.

        :param config: this section's wrapper (for the merged fileconfig)
        :param sections: fabricated (name, keys) pairs
        :return: the same pairs with learned values folded in
        """
        fabricated = {name for name, _keys in sections}
        store = self._read_state()
        autov = self._read_ini(self.auto_vars_file)
        overrides = self._override_sections(config)
        # Kept so a model override can be applied later. A spare pool bay is
        # fabricated before anything is plugged into it, so its ams_model is
        # the placeholder `boxed` and an [AFC_BridgeBox ams1] / ams2 section
        # cannot match it here. The real model is known only once a unit is
        # claimed onto the bay and the dialect verdict lands, at runtime. See
        # _reapply_model_override.
        self._overrides = overrides

        folded: List[Tuple[str, Dict[str, Any]]] = []
        updates: Dict[str, Dict[str, Any]] = {}
        for name, keys in sections:
            merged = dict(keys)
            for src in (store, autov):        # auto_vars newest, wins last
                if src is not None and src.has_section(name):
                    for k, v in src.items(name):
                        if k not in _STRUCTURAL_KEYS:
                            merged[k] = v
            # Learned = anything that differs from the pure fabricated
            # defaults, whichever file it arrived from. Persisting exactly
            # that set means the state carries the override even after the
            # auto_vars copy is swept, and never carries noise. Computed
            # before the operator overrides land: those live in the
            # operator's own config and must not be copied into "learned"
            # state, so deleting the override section removes the override.
            learned = {k: v for k, v in merged.items() if keys.get(k) != v}
            # The operator outranks history: override keys overlay last,
            # identity keys still protected. Model-wide first ("every ht"),
            # then per-unit, so the specific always beats the general. See
            # _override_sections for where these come from -- serial_port-
            # less [AFC_BridgeBox <target>] sections, writable right in the
            # master's own .cfg above the managed block.
            ov: Dict[str, Any] = {}
            # Chain-wide first, so model and unit sections both override it.
            ov.update(getattr(self, "_chain_defaults", None) or {})
            if keys.get("ams_model"):
                mkey = "model:" + _norm_model(str(keys["ams_model"]))
                ov.update(overrides.get(mkey) or {})
            ov.update(overrides.get(name) or {})
            for k, v in ov.items():
                if k not in _STRUCTURAL_KEYS:
                    merged[k] = v
            folded.append((name, merged))
            if learned:
                updates[name] = learned

        # An override whose target names nothing known is likely a typo.
        # Remember the misses; _scout_ready reports them once the AFC logger
        # exists (there is none this early in __init__).
        #
        # The test is "is this name known", not "did it apply to anything": a
        # model section for a model with no unit plugged in right now (e.g.
        # [AFC_BridgeBox ams1] on a chain holding an ams2 and an ht) is a valid
        # default that takes effect when such a unit is claimed.
        known = {"model:" + m for m in _SLOTS_BY_MODEL}
        known |= {name for name, _keys in sections}
        self._unmatched_overrides = sorted(set(overrides) - known)

        changed = False
        if autov is not None:
            for name in list(autov.sections()):
                if name in fabricated:
                    autov.remove_section(name)
                    changed = True
                elif name.startswith("AFC_BambuAMS "):
                    try:
                        fc = getattr(config, "fileconfig", None)
                        in_real_cfg = bool(
                            fc is not None and fc.has_section(name)
                            and set(dict(fc.items(name)))
                            - set(dict(autov.items(name))))
                    except Exception:
                        in_real_cfg = True    # cannot prove orphan: keep it
                    if not in_real_cfg:
                        autov.remove_section(name)
                        changed = True
        # A freshly computed auto base is remembered alongside the learned
        # values, so it survives config growth.
        if getattr(self, "_resolved_base", None):
            updates.setdefault(self._BASE_SECTION + " " + self.name, {})[
                "lane_base"] = self._resolved_base
        try:
            if changed:
                with open(self.auto_vars_file, "w") as fp:
                    fp.write("# This file is autogenerated and updated when "
                             "variables are not in your normal AFC config "
                             "files\n\n")
                    autov.write(fp)
        except Exception:
            pass                              # persistence is best-effort
        if updates:
            self._state_set(updates)
        return folded

    def _register_scout_chip(self, config: Any) -> None:
        """
        Register the bambu_buffer pin chip with no unit behind it.

        Same registry and dedupe the units use, so an enrolled boot's real
        chip and a scout boot's stub can never coexist or double-register.

        :param config: this section's wrapper (unused beyond symmetry)
        """
        import types as _types
        from extras.AFC_BambuAMS import _register_bambu_buffer_chip
        shim = _types.SimpleNamespace(
            printer=self.printer,
            buffer_chip_name=self.buffer_chip_name,
            fps_buffer_value=lambda: None)
        _register_bambu_buffer_chip(shim)

    # ── scouting: the chain names its own roster ─────────────────────────────

    def _locate_own_file(self) -> Optional[str]:
        """
        The .cfg file declaring this [AFC_BridgeBox <name>] section.

        Searched under the main config file's directory, because that is
        where include chains live. First match wins; None when the section
        cannot be found (odd layouts), and the caller falls back to a
        default path rather than guessing.

        :return: absolute path, or None
        """
        try:
            import glob as _glob
            import re as _re
            cfg = (getattr(self.printer, "start_args", {}) or {}).get(
                "config_file")
            if not cfg:
                return None
            root = os.path.dirname(os.path.abspath(cfg))
            pat = _re.compile(
                r"^[ \t]*\[[ \t]*AFC_BridgeBox[ \t]+"
                + _re.escape(self.name) + r"[ \t]*\]", _re.I | _re.M)
            for path in sorted(_glob.glob(os.path.join(root, "**", "*.cfg"),
                                          recursive=True)):
                try:
                    with open(path) as fp:
                        if pat.search(fp.read()):
                            return path
                except Exception:
                    continue
        except Exception:
            pass
        return None

    _STATE_MARK = ("#~# --- AFC_BridgeBox managed state -- "
                   "everything below is auto-written ---")
    _STATE_PREFIX = "#~# "

    def _read_state(self) -> configparser.RawConfigParser:
        """
        The managed block of the state file, decommented and parsed.

        :return: parser over the state (empty when absent)
        """
        cp = configparser.RawConfigParser(delimiters=(":", "="))
        try:
            with open(self.state_file) as fp:
                text = fp.read()
        except Exception:
            return cp
        if self._STATE_MARK not in text:
            return cp
        block = text.split(self._STATE_MARK, 1)[1]
        lines = [ln[len(self._STATE_PREFIX):] if
                 ln.startswith(self._STATE_PREFIX) else ln.lstrip("#~ ")
                 for ln in block.splitlines() if ln.startswith("#~#")]
        try:
            cp.read_string("\n".join(lines))
        except Exception:
            return configparser.RawConfigParser(delimiters=(":", "="))
        return cp

    def _write_state(self, cp: configparser.RawConfigParser) -> None:
        """
        Rewrite the managed block, preserving everything the operator wrote
        above the marker -- their [AFC_BridgeBox] section included.

        :param cp: the state to serialize
        """
        try:
            head = ""
            try:
                with open(self.state_file) as fp:
                    head = fp.read().split(self._STATE_MARK, 1)[0]
            except Exception:
                pass
            if not head.strip():
                head = ("# AFC_BridgeBox: put your [AFC_BridgeBox <name>] "
                        "section here (serial_port + extruder is enough).\n"
                        "# The block below is maintained by the module.\n\n")
            import io
            buf = io.StringIO()
            cp.write(buf)
            body = "".join(self._STATE_PREFIX + ln + "\n"
                           for ln in buf.getvalue().splitlines())
            with open(self.state_file, "w") as fp:
                fp.write(head.rstrip("\n") + "\n\n"
                         + self._STATE_MARK + "\n" + body)
        except Exception:
            pass                              # persistence is best-effort

    def _state_get(self, section: str, key: str) -> Optional[str]:
        """
        :param section: state section name
        :param key: key within it
        :return: the value, or None
        """
        cp = self._read_state()
        if cp.has_section(section):
            return dict(cp.items(section)).get(key)
        return None

    def _state_set(self, updates: Dict[str, Dict[str, Any]]) -> None:
        """
        Merge updates into the state block and write it back.

        :param updates: {section: {key: value}}
        """
        cp = self._read_state()
        for section, keys in updates.items():
            if not cp.has_section(section):
                cp.add_section(section)
            for k, v in keys.items():
                cp.set(section, k, str(v))
        self._write_state(cp)

    def persist_learned(self, unit_name: str, key: str, value: Any) -> bool:
        """
        Record one value a unit learned about itself, so it survives a restart.

        ConfigRewrite would file the key under [AFC_BambuAMS <name>] in
        AFC_auto_vars.cfg, which klippy parses; a pool-fabricated unit has no
        such section in any .cfg, so at next boot check_unused_options would
        reject the orphan and halt. This writes to the state file instead,
        which klippy never parses, and _fold_and_sweep overlays store
        sections onto their unit at boot.

        Structural keys are refused outright: those define what a unit is, are
        owned by the fabricator, and a learned value must never redefine one.

        :param unit_name: the fabricated unit's section name, e.g. the value of
          the unit's own ``name``
        :param key: option to persist
        :param value: value to persist (stringified)
        :return bool: True if it was written
        """
        if not unit_name or not key:
            return False
        if key in _STRUCTURAL_KEYS:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: refusing to persist structural key "
                f"'{key}' for {unit_name}")
            return False
        # _UNIT_SECTION, not _BASE_SECTION: this has to be the exact section
        # name the fabricator emits (see the sections.append of
        # f"AFC_BambuAMS {name}"), because that is what _fold_and_sweep
        # matches on.
        section = self._UNIT_SECTION + " " + unit_name
        try:
            cur = self._read_state()
            if cur.has_section(section) and cur.has_option(section, key):
                if str(cur.get(section, key)) == str(value):
                    return False          # unchanged; no rewrite
            self._state_set({section: {key: value}})
            return True
        except Exception as ex:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: could not persist {key} for "
                f"{unit_name}: {ex}")
            return False

    def _migrate_legacy_state(self) -> None:
        """One-time absorb of the split .roster/.vars files this replaced."""
        base = os.path.dirname(self.auto_vars_file)
        legacy_roster = os.path.join(base, f"AFC_BridgeBox_{self.name}.roster")
        legacy_vars = os.path.join(base, f"AFC_BridgeBox_{self.name}.vars")
        updates: Dict[str, Dict[str, Any]] = {}
        try:
            with open(legacy_roster) as fp:
                roster = ", ".join(
                    ln.strip() for ln in fp
                    if ln.strip() and not ln.lstrip().startswith("#"))
            if roster:
                updates.setdefault(
                    self._BASE_SECTION + " " + self.name, {})["roster"] = roster
        except Exception:
            pass
        old = self._read_ini(legacy_vars)
        if old is not None:
            for sec in old.sections():
                tgt = (self._BASE_SECTION + " " + self.name
                       if sec == self._BASE_SECTION else sec)
                updates.setdefault(tgt, {}).update(dict(old.items(sec)))
        if updates:
            self._state_set(updates)
            for f in (legacy_roster, legacy_vars):
                try:
                    os.remove(f)
                except Exception:
                    pass

    @staticmethod
    def _chain_to_roster(uids: List[str], htmask: int) -> str:
        """
        The chain reply, rendered in roster syntax.

        Model comes from the firmware's htmask where it reports one -- bit
        per chain index -- and falls back to the enrollment convention (boxed
        units at 0..3, HTs at 4..) when htmask is 0. Empty positions are
        unenrolled slots and are skipped, but indices are preserved, because
        position is the polling address.

        :param uids: chain_uids() -- index -> 24-hex UID, empties kept
        :param htmask: chain_diag()[0] -- per-index HT flag bits, 0 if unknown
        :return str: "ht:UID, boxed:UID, ..." in chain order, or ""
        """
        entries = []
        for i, uid in enumerate(uids):
            uid = _norm_uid(uid)
            if not uid or set(uid) == {"F"}:
                continue
            is_ht = bool(htmask >> i & 1) if htmask else i >= 4
            entries.append(f"{'ht' if is_ht else 'boxed'}:{uid}")
        return ", ".join(entries)

    def _scout_ready(self, eventtime: Optional[float] = None) -> None:
        """klippy:ready -- take AFC's logger, make sure a bridge exists on
        our port, and start the slow chain watch."""
        afc = self.printer.lookup_object("AFC", None)
        if afc is not None:
            self.logger = afc.logger
        # Report overrides that matched nothing (see where
        # _unmatched_overrides is built) now that a logger exists.
        for miss in getattr(self, "_unmatched_overrides", None) or []:
            shown = miss[len("model:"):] if miss.startswith("model:") else miss
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: [AFC_BridgeBox {shown}] overrides "
                f"nothing -- no unit and no model by that name, so its keys are "
                f"ignored. Models: {', '.join(sorted(_SLOTS_BY_MODEL))}.")
        # Only pure scout mode may create a bridge. With units fabricated,
        # the first unit is the bridge's owner and does its bridge-wide init;
        # a scout-created bridge would put every unit on the ownerless
        # "sharing" path. Enrolled mode waits for the unit's bridge to appear
        # in the registry.
        if self._roster_source == "scout":
            try:
                self._ensure_bridge()
            except Exception as e:
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: could not open the bridge "
                    f"for scouting ({e}); will keep retrying.")
        reactor = self.printer.get_reactor()
        self._scout_timer = reactor.register_timer(
            self._scout_tick, reactor.monotonic() + 2.0)
        if self._roster_source == "scout":
            self.logger.info(
                f"AFC_BridgeBox {self.name}: no roster configured and no "
                f"recorded roster -- scouting the chain. Detected units are "
                f"recorded in {os.path.basename(self.state_file)}; RESTART "
                f"then enrolls them.")

    def _ensure_bridge(self) -> Any:
        """
        The bridge for our serial port -- shared with any fabricated units
        via the same registry they use, created if scouting found none.

        :return: the bridge
        """
        bridge = self._chain_bridge()
        if bridge is None:
            from extras import AFC_BambuAMS_bridge as _bridge_mod
            from extras.AFC_BambuAMS_bridge import BambuBridge, TcpPort
            is_tcp = str(self.serial_port or "").strip().lower() \
                .startswith("tcp://")

            def _open() -> Any:
                """
                Open the master's port, the same way the units do: a socket
                for a ``tcp://host:port`` bridge, else the USB-CDC serial port.

                :return Any: an open port, pyserial-shaped either way
                """
                if is_tcp:
                    host, port = TcpPort.parse(self.serial_port)
                    return TcpPort(host, port, timeout=0.1, write_timeout=0.5,
                                   key=self.tcp_key)
                import serial
                return serial.Serial(self.serial_port, 115200,
                                     timeout=0.1, write_timeout=0.5)

            bridge = BambuBridge(_open, self.printer.get_reactor(),
                                 self.logger)
            _bridge_mod._BRIDGES[self.serial_port] = bridge
            # A network bridge that is not up yet keeps retrying rather than
            # failing the scout, as it does for the units.
            bridge.start(defer_open=is_tcp)
        self._bridge = bridge
        return bridge

    def _scout_tick(self, eventtime: float) -> float:
        """
        The chain watch: ask, read, and record what changed.

        In scout mode this is how the first roster comes to exist; with
        units already fabricated it is how a unit plugged in later gets
        noticed. Either way the answer lands in the recorded roster and one
        console line says what to do -- the record is only consulted when
        the roster: option is absent, so an explicit option stays
        authoritative. With a pool configured, units are also claimed onto
        and released from their bays live.

        A new uid is appended to the recorded roster once it has stayed
        online for enroll_grace. Absence alone proves little (a Pico reboot,
        a unit power-cycling mid-dry, the whole chain briefly deaf), so a
        rostered unit is only dropped after removal_grace of continuous
        absence from an otherwise-live chain -- see _prune_missing. Roster
        edits take effect at the next restart.

        :param eventtime: reactor time
        :return float: next wake
        """
        self._tick_count = getattr(self, "_tick_count", 0) + 1
        try:
            if self._roster_source == "scout":
                bridge = getattr(self, "_bridge", None) or self._ensure_bridge()
            else:
                bridge = self._chain_bridge()
                if bridge is None:
                    self._watch_state = "no-bridge"
                    return eventtime + 3.0      # pool-owner bridge still coming up
            # Do not poll while the bridge is silent. An AMS 1 capscan holds the
            # bridge firmware in a blocking burst for 10-15 s with its USB side
            # unread, so a chain poll sent then cannot be answered and only
            # queues in the kernel, adding to the traffic cdc-acm may drop.
            # Skip the ask, not the tick: everything below reads the cached
            # chain and the last status's online flags, which stay frozen
            # through the silence. The floor keeps an ordinary poll gap from
            # counting as silence. A bridge that cannot say (no silent_for, or
            # None because it has never spoken) is asked.
            _sf = getattr(bridge, "silent_for", None)
            try:
                _quiet = _sf() if callable(_sf) else None
            except Exception:
                _quiet = None
            if (not isinstance(_quiet, (int, float))
                    or _quiet <= max(2.0, 1.5 * self.hotplug_poll)):
                bridge.send({"cmd": "chain"})
            uids = bridge.chain_uids()
            htmask = bridge.chain_diag()[0]
            seen = self._chain_to_roster(uids, htmask)
            sec = self._BASE_SECTION + " " + self.name
            entries = [e.strip()
                       for e in (self._state_get(sec, "roster") or "").split(",")
                       if e.strip()]
            known_uids = {_norm_uid(e.partition(":")[2])
                          for e in entries}
            # Enrollment and claim require the unit be online, not merely
            # present in the firmware's sticky chain_uids: a pulled unit lingers
            # in chain_uids (its online flag flips false in ~1s) until the Pico
            # reboots, so enrolling off chain_uids would re-add a unit the
            # operator just cleared. Read the per-unit online flags once here
            # and let both halves key off them.
            latest = bridge.latest_status() or {}
            online_idx = {u.get("n") for u in (latest.get("units") or [])
                          if u.get("online")}
            online_uids = {_norm_uid(uids[i])
                           for i in online_idx
                           if i < len(uids) and (uids[i] or "").strip()}
            # A uid forgotten while still online is held on the suppress list so
            # neither enroll nor claim treats it as a fresh arrival. Once it is
            # physically pulled (drops out of online_uids) the hold is released,
            # so a genuine re-plug enrolls it fresh. Feed a suppressed uid's
            # absence into `online_uids` here so both halves below skip it.
            suppressed = getattr(self, "_forget_suppressed", None)
            if suppressed:
                for u in [x for x in suppressed if x not in online_uids]:
                    suppressed.discard(u)          # gone -- let a re-plug enroll
                online_uids = online_uids - suppressed
            # Enroll only a unit that holds online continuously for enroll_grace
            # (see its option in __init__); a ghost that drops out between
            # blips resets and never enrolls.
            esince = getattr(self, "_enroll_since", None)
            if esince is None:
                esince = self._enroll_since = {}
            cand: Dict[str, str] = {}
            for e in seen.split(", "):
                u = _norm_uid(e.partition(":")[2])
                if e and u and u not in known_uids and u in online_uids:
                    cand[u] = e
            for u in list(esince):
                if u not in cand:
                    del esince[u]                  # dropped offline -> reset hold
            for u in cand:
                esince.setdefault(u, eventtime)
            new = [cand[u] for u in cand
                   if eventtime - esince.get(u, eventtime) >= self.enroll_grace]
            if new:
                entries = entries + new
                self._state_set({sec: {"roster": ", ".join(entries)}})
                if self._roster_source == "scout":
                    self.logger.info(
                        f"AFC_BridgeBox {self.name}: chain reports "
                        f"[{', '.join(entries)}] -- written to "
                        f"{os.path.basename(self.state_file)}. RESTART to "
                        f"enroll, or copy it into roster: to pin it.")
                else:
                    self.logger.info(
                        f"AFC_BridgeBox {self.name}: NEW unit(s) on the "
                        f"chain: {', '.join(new)} -- recorded. RESTART to "
                        f"enroll"
                        + (", or add to roster: (it is set and overrides "
                           "the file)." if self._roster_source == "option"
                           else "."))
            # Live claim: every present unit not yet bound gets its slot now --
            # a known unit onto its own named slot (same lanes/name/T#), a new
            # one onto a spare -- with no restart. Roster is authoritative for a
            # known unit's model (including a refined ams1/ams2); htmask (via
            # `seen`) classifies a brand-new unit as ht or boxed.
            if self.pool_ams or self.pool_ht:
                model_by_uid: Dict[str, str] = {}
                for e in seen.split(", "):
                    m, _sep, u = e.partition(":")
                    if u.strip():
                        model_by_uid[_norm_uid(u)] = m.strip().lower()
                for e in entries:
                    m, _sep, u = e.partition(":")
                    if u.strip():
                        model_by_uid[_norm_uid(u)] = m.strip().lower()
                bound = {pu["bound"] for pu in getattr(self, "_pool_units", [])
                         if pu.get("bound")}

                def _slot_laneno(u: str) -> int:
                    """
                    Sort key: the first lane number of the slot a uid owns.

                    :param u: unit uid
                    :return int: lane number, 9999 for a uid with no slot yet
                    """
                    for pu in self._pool_units:
                        if pu.get("uid") == u and pu.get("lanes"):
                            ds = "".join(c for c in pu["lanes"][0] if c.isdigit())
                            return int(ds) if ds else 9999
                    return 9999

                # Presence is the per-unit online flag (online_uids, above), not
                # sticky chain_uids. Claim and release both key off it, so they
                # never fight over a sticky-but-absent unit.
                # Track how long each online unit has been continuously online;
                # a single sample offline resets its clock.
                online_since = getattr(self, "_online_since", None)
                if online_since is None:
                    online_since = self._online_since = {}
                for u in online_uids:
                    online_since.setdefault(u, eventtime)
                for u in list(online_since):
                    if u not in online_uids:
                        del online_since[u]
                rel_at = getattr(self, "_released_at", None)
                if rel_at is None:
                    rel_at = self._released_at = {}
                # Claim an online unit onto its slot once it has held online long
                # enough -- a short claim_grace normally, or the longer
                # flap_claim_grace if it was released within flap_window (so a
                # flapping link does not keep re-claiming). Lowest-lane# first so
                # TcmdAssign's lowest-free T# lines up with the lane numbers (HT
                # on lane12 -> T12, not T16 for claiming after the AMS2).
                for u in sorted(online_uids - bound, key=_slot_laneno):
                    req = (self.flap_claim_grace
                           if eventtime - rel_at.get(u, -1e9) < self.flap_window
                           else self.claim_grace)
                    if eventtime - online_since.get(u, eventtime) >= req:
                        # A unit that already owns a slot is just coming back to
                        # it (no popup -- nothing to decide); one that owns none
                        # is new and takes a spare. Checked before the claim,
                        # which is what adopts the spare onto it.
                        owned_before = any(
                            _norm_uid(pu.get("uid")) == u
                            for pu in self._pool_units)
                        if self._claim_pool_unit(
                                u, model_by_uid.get(u, "boxed")) is not None \
                                and not owned_before \
                                and not self._is_printing():
                            # It already has a home (the spare it just claimed);
                            # the popup only offers to move it to a named bay.
                            # Queued so a burst of adds serialize, none lost.
                            self._queue_popup(("new", u))
                # Release a claimed unit whose online flag has stayed false for
                # release_grace -- lanes + T# dropped, unit back to pool, no
                # restart, survivors untouched. A re-plug inside the window is
                # seen online again (its clock resets) and nothing happens.
                online_seen = getattr(self, "_last_online", None)
                if online_seen is None:
                    online_seen = self._last_online = {}
                # A unit's continuous-online run start (absent = not online).
                # Only a run held for release_settle clears the release clock, so
                # a phantom online flag that merely blips can never cancel a drop.
                run = getattr(self, "_online_run", None)
                if run is None:
                    run = self._online_run = {}
                bound_now = {pu["bound"] for pu in self._pool_units
                             if pu.get("bound")}
                for b in bound_now:
                    if b in online_uids:
                        run.setdefault(b, eventtime)           # run continues
                        if eventtime - run[b] >= self.release_settle:
                            online_seen[b] = eventtime          # solidly back
                    else:
                        run.pop(b, None)                       # a gap breaks it
                        online_seen.setdefault(b, eventtime)   # start the clock
                if self.auto_drop and not self._is_printing():
                    for b in list(bound_now):
                        # Drop only on a tick the unit is actually offline: this
                        # spares a genuine re-plug that reads online right now
                        # (its run has not cleared the clock yet), while a
                        # phantom that spends most ticks offline still trips it.
                        if (b not in online_uids
                                and eventtime - online_seen.get(b, eventtime)
                                >= self.release_grace):
                            # The bay's name, captured before release clears the
                            # binding, for the removal popup.
                            gone = next(
                                (pu.get("name") for pu in self._pool_units
                                 if pu.get("bound") == b), None)
                            self._release_pool_unit(b)
                            online_seen.pop(b, None)
                            run.pop(b, None)
                            rel_at[b] = eventtime   # anti-flap: bar rises before reclaim
                            self._queue_popup(("removed", b, gone))
                # Walk the popup queue: one dialog per _POPUP_HOLD, so a burst
                # of plug/unplugs is shown in turn rather than clobbered.
                self._pump_popups(eventtime)
            # One fixed, fast interval, because this tick is cheap: the
            # firmware's "chain" handler (hostapi.c) only prints its cached
            # state over USB-CDC and sends no RS-485 frame, so polling never
            # touches the bus or collides with motion or a scan. Frequent
            # samples also let the release clock see a flapping unit's offline
            # stretches. All graces are measured against eventtime, so the
            # interval does not change them.
            self._watch_next = self.hotplug_poll
            entries = self._refine_models(bridge, uids, entries, sec)
            self._prune_missing(bridge, uids, entries, eventtime, sec)
            # Re-read what the record now says (refine and prune both may
            # have rewritten it) so get_status can show the recorded chain
            # beside the running one (the difference applies at restart).
            self._recorded_raw = self._state_get(sec, "roster") or ""
        except Exception as e:
            # Best-effort, but not silent: log each distinct failure once.
            msg = f"{type(e).__name__}: {e}"
            self._watch_state = f"error: {msg}"
            if msg != getattr(self, "_tick_err_logged", None):
                self._tick_err_logged = msg
                try:
                    self.logger.warning(
                        f"AFC_BridgeBox {self.name}: chain watch tick failed "
                        f"({msg}); will keep retrying.")
                except Exception:
                    pass
        return eventtime + getattr(self, "_watch_next", self.hotplug_poll)

    def _is_printing(self) -> bool:
        """
        True while a print is active or paused -- auto-drop must never fire
        then: dropping a lane mid-print would disrupt a follower/feed. A pull
        during a print is left alone; the unit stays claimed (lanes intact)
        until the print ends, then the drop runs.
        """
        try:
            now = self.printer.get_reactor().monotonic()
        except Exception:
            now = 0.0
        for name, states in (("print_stats", ("printing", "paused")),
                             ("idle_timeout", ("Printing",))):
            try:
                obj = self.printer.lookup_object(name, None)
                if obj is not None:
                    st = obj.get_status(now).get("state")
                    if st in states:
                        return True
            except Exception:
                pass
        return False

    def _prompt(self, title: str, lines: List[str],
                buttons: List[Tuple[str, str, str]],
                timeout: float = 45.0) -> None:
        """
        Raise a Mainsail/Fluidd interactive dialog (the action:prompt protocol
        Klipper speaks over the g-code channel -- no panel change needed). Each
        button runs a g-code command when clicked; a Dismiss footer closes it
        with no action, and it auto-closes after `timeout` so an unattended
        popup never lingers. Best-effort: a UI notification must never fault the
        watch tick.

        :param title: dialog title
        :param lines: body text lines
        :param buttons: (label, gcode command, style) each -- style is one of
            primary/secondary/info/warning/error
        :param timeout: seconds before the dialog self-dismisses
        """
        gcode = self.printer.lookup_object("gcode", None)
        if gcode is None:
            return
        try:
            # Each popup gets a generation number, and its auto-dismiss timer
            # only fires prompt_end if it is still the current popup, so an
            # earlier popup's timer cannot close a later dialog early.
            gen = getattr(self, "_prompt_gen", 0) + 1
            self._prompt_gen = gen
            respond = gcode.respond_raw
            respond(f"// action:prompt_begin {title}")
            for ln in lines:
                respond(f"// action:prompt_text {ln}")
            for label, cmd, style in buttons:
                respond(f"// action:prompt_button {label}|{cmd}|{style}")
            respond("// action:prompt_footer_button "
                    "Dismiss|RESPOND TYPE=command MSG=action:prompt_end|info")
            respond("// action:prompt_show")
            if timeout:
                reactor = self.printer.get_reactor()

                def _close(e, _gen=gen):
                    """
                    Close the prompt if it is still the one this timer armed.

                    :param e: reactor event time
                    :param _gen: prompt generation this callback belongs to
                    """
                    if getattr(self, "_prompt_gen", 0) == _gen:
                        gcode.respond_raw("// action:prompt_end")
                reactor.register_callback(
                    _close, reactor.monotonic() + timeout)
        except Exception as ex:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: prompt error: {ex}")

    def _close_prompt(self) -> None:
        """
        Close whatever action:prompt dialog is open -- called at the end of an
        action a popup button triggered (assign/unassign/forget), because
        Mainsail runs a prompt_button's command but does not close the dialog
        on its own. Bumps the generation so no pending auto-dismiss fires
        against a later popup. Harmless when no dialog is open (called direct).
        """
        gcode = self.printer.lookup_object("gcode", None)
        if gcode is None:
            return
        self._prompt_gen = getattr(self, "_prompt_gen", 0) + 1
        try:
            gcode.respond_raw("// action:prompt_end")
        except Exception:
            pass

    #: How many "move to a named bay" buttons a new-unit popup offers before it
    #: stops -- a dialog with thirty buttons helps no one. The free bays are
    #: taken lowest-lane first, so the nearest alternatives show.
    _MAX_BAY_BUTTONS = 8

    def _bay_of_uid(self, uid: str) -> Optional[Dict[str, Any]]:
        """The pool bay a uid is bound to, or (failing that) reserves."""
        uid = _norm_uid(uid)
        pools = getattr(self, "_pool_units", [])
        return (next((pu for pu in pools
                      if _norm_uid(pu.get("bound")) == uid), None)
                or next((pu for pu in pools
                         if _norm_uid(pu.get("uid")) == uid), None))

    def _prompt_new_unit(self, uid: str, timeout: float = 45.0) -> None:
        """
        Bay-picker popup for a unit that already has a home (the bay shown in
        the title): the buttons only offer to move it to one of the operator's
        other free bays of the same family via AFC_BRIDGEBOX_ASSIGN. Dismiss =
        keep where it is -- there is no stuck state, because it already sits on
        a real named bay. Fired automatically when a brand-new unit is claimed,
        and on demand by AFC_BRIDGEBOX_ASSIGN UID=<uid> with no NAME.
        """
        uid = _norm_uid(uid)
        here = self._bay_of_uid(uid)
        if here is None:
            return
        family = here.get("family")
        pools = getattr(self, "_pool_units", [])

        free = sorted((pu for pu in pools
                       if pu is not here and pu.get("family") == family
                       and not pu.get("bound") and not pu.get("uid")),
                      key=_pool_laneno)
        buttons = [(pu["name"],
                    f"AFC_BRIDGEBOX_ASSIGN CHAIN={self.name} UID={uid} "
                    f"NAME={pu['name']}", "primary")
                   for pu in free[:self._MAX_BAY_BUTTONS]]
        head = (f"UID {uid} is on {here['name']!r} (its T# and lanes are live).")
        lines = ([head, "Keep it here, or move it to another named bay:"]
                 if buttons
                 else [head, "No other free bay of this type to move it to."])
        self._prompt(f"New AMS on {here['name']}", lines, buttons,
                     timeout=timeout)

    def _prompt_removed_unit(self, uid: str, name: Optional[str],
                             timeout: float = 45.0) -> None:
        """
        Popup when a unit is auto-dropped on unplug: its bay stays reserved for
        a re-plug (nothing to do -- Dismiss). The one button offers to forget
        it, freeing the bay to the pool for good, for when it is not coming
        back. Never shown during a print (release is gated then anyway).
        """
        label = name or uid
        self._prompt(
            f"AMS removed: {label}",
            [f"{label} (UID {uid}) was unplugged; its bay is held for a "
             "re-plug.",
             "Re-plug it and it reclaims the same lanes/T#. Or forget it to "
             "free the bay to the pool:"],
            [(f"Forget {label}",
              f"AFC_BRIDGEBOX_FORGET CHAIN={self.name} UID={uid}", "error")],
            timeout=timeout)

    #: A queued popup holds the screen for this long before the next one in the
    #: queue replaces it -- so several units plugged in at once each get a turn
    #: rather than all but the last being clobbered by the next prompt_begin.
    _POPUP_HOLD = 20.0

    def _queue_popup(self, ev: Tuple[Any, ...]) -> None:
        """
        Enqueue a popup event so simultaneous plug/unplugs serialize instead of
        overwriting each other (Mainsail shows only the newest prompt). Deduped
        by (kind, uid): a unit that flaps does not stack duplicate popups.

        :param ev: ("new", uid) or ("removed", uid, name)
        """
        q = getattr(self, "_popup_queue", None)
        if q is None:
            q = self._popup_queue = []
        key = (ev[0], ev[1])
        q[:] = [e for e in q if (e[0], e[1]) != key]
        q.append(ev)

    def _pump_popups(self, eventtime: float) -> None:
        """
        Show the next queued popup if the last one has had its turn. Called
        every watch tick; one popup shows per _POPUP_HOLD so a burst of adds is
        walked through, none lost. A "new" event whose unit is no longer
        present is dropped silently.
        """
        q = getattr(self, "_popup_queue", None)
        if not q or eventtime < getattr(self, "_popup_active_until", 0.0):
            return
        while q:
            ev = q.pop(0)
            if ev[0] == "new":
                if self._bay_of_uid(ev[1]) is None:
                    continue                  # gone before its turn -- skip
                self._prompt_new_unit(ev[1], timeout=self._POPUP_HOLD)
            else:
                self._prompt_removed_unit(ev[1], ev[2], timeout=self._POPUP_HOLD)
            self._popup_active_until = eventtime + self._POPUP_HOLD
            return

    def _release_pool_unit(self, uid: str) -> None:
        """
        Release the pool unit bound to ``uid`` -- the hot-unplug half. Drops its
        lanes' T# (by reverse lookup on tool_cmds, since lane.map surfaces as
        None on the AFC_stepper view) and returns each lane + the unit object to
        idle, live, no restart. A spare goes back to the generic pool (uid
        cleared); a known unit keeps its slot so a re-plug re-claims the same
        lanes/name/T#. No relink, no survivor is touched.

        :param uid: the UID of the unit to release
        """
        uid = _norm_uid(uid)
        afc = self.printer.lookup_object("AFC", None)
        for pu in getattr(self, "_pool_units", []):
            if pu.get("bound") != uid:
                continue
            gcode = self.printer.lookup_object("gcode", None)
            for lname in pu["lanes"]:
                lane = self.printer.lookup_object(f"AFC_lane {lname}", None)
                if lane is None or getattr(lane, "unassigned", True):
                    continue
                # Collect this lane's T# from both the lane's own map and a
                # reverse lookup on tool_cmds, then drop each from tool_cmds and
                # unregister its g-code macro: TcmdAssign refuses to reuse a T#
                # whose command still exists (ready_gcode_handlers), so a
                # leftover macro would push the re-claim onto the next free set.
                cmds = set(getattr(lane, "map", None) or [])
                if afc is not None:
                    for m, owner in list(afc.tool_cmds.items()):
                        if owner == lane.name:
                            cmds.add(m)
                            afc.tool_cmds.pop(m, None)
                for cmd in cmds:
                    if cmd and cmd != "NONE" and gcode is not None:
                        try: gcode.register_command(cmd, None)
                        except Exception: pass
                try: deactivate_to_pool(lane)
                except Exception: pass
            unit = self.printer.lookup_object(
                f"AFC_BambuAMS {pu['name']}", None)
            if unit is not None:
                try: unit.release()
                except Exception: pass
            pu["bound"] = None
            # Whose spool bindings the lanes are now holding in their release
            # snapshots. Only this uid coming back may have them put back; any
            # other unit claiming this slot gets clean lanes.
            pu["lane_spool_uid"] = uid
            if pu.get("spare"):
                pu["uid"] = None                           # spare -> generic pool
            self.logger.info(
                f"AFC_BridgeBox {self.name}: released {pu['name']} (UID {uid} "
                f"offline >{self.release_grace:.0f}s); lanes dropped live"
                + ("" if pu.get("spare") else ", slot kept for re-plug"))
            return

    def _live_access_tracking(self) -> dict:
        """
        The printer's own config access-tracking dict, or a throwaway.

        Klipper has moved this between objects (it lives on configfile.validate
        in current versions and on configfile itself in older ones), so it is
        looked up by shape rather than by path. A miss only leaves fabricated
        sections absent from configfile.settings and never stops a chain
        coming up.

        :return dict: the live tracking dict, or a fresh one if not found
        """
        try:
            cf = self.printer.lookup_object("configfile", None)
            for owner in (getattr(cf, "validate", None), cf):
                at = getattr(owner, "access_tracking", None)
                if isinstance(at, dict):
                    return at
        except Exception as e:
            self.logger.debug(
                f"AFC_BridgeBox {self.name}: no access tracking to share "
                f"({e}); fabricated sections stay out of configfile.settings")
        return {}

    def _claim_pool_unit(self, uid: str, model: str) -> Any:
        """
        Bind a free pool unit to ``uid`` and bring it fully online, live.

        Activates the pool unit's lanes (registers them everywhere), mints
        their T# commands, and calls the unit's claim() to join the bus. The
        binding itself is not persisted: a restart brings every pool unit back
        up idle, and the roster (which recorded the uid) gives the uid its own
        named slot, which it re-claims when seen online.

        :param uid: the new physical AMS's UID
        :param model: its generation tag (ht/ams1/ams2/boxed)
        :return: the claimed unit object, or None if none was free/possible
        """
        uid = _norm_uid(uid)
        if not uid:
            return None
        afc = self.printer.lookup_object("AFC", None)
        if afc is None:
            return None
        pools = getattr(self, "_pool_units", [])
        if any(pu.get("bound") == uid for pu in pools):
            return None                       # already live on a pool unit
        family = "ht" if model == "ht" else "ams"
        # Prefer the slot this uid owns (a known unit's named slot, so it comes
        # back on the same lanes/name/T#); else the next free spare of the
        # right family, which this uid then adopts.
        chosen = next((pu for pu in pools
                       if pu.get("bound") is None and pu.get("uid") == uid),
                      None)
        if chosen is None:
            chosen = next((pu for pu in pools
                           if pu.get("bound") is None and pu.get("uid") is None
                           and pu.get("family") == family), None)
            if chosen is not None:
                chosen["uid"] = uid           # the spare now belongs to this uid
        if chosen is None:
            self.logger.warning(
                f"AFC_BridgeBox {self.name}: new {family} unit {uid} but no free "
                f"pool slot (raise pool_ams/pool_ht); RESTART to enroll.")
            return None
        unit = self.printer.lookup_object(
            f"AFC_BambuAMS {chosen['name']}", None)
        if unit is None or not getattr(unit, "pool", False):
            return None
        # Start each lane from a clean T#/map slate. A previous claim on this
        # slot may have left stale tool_cmds entries pointing at these lanes;
        # TcmdAssign would see those tools taken and hand out later ones. Pop
        # anything pointing at these lanes (by the lane's own map and by
        # reverse lookup) before assigning.
        for lname in chosen["lanes"]:
            lane = self.printer.lookup_object(f"AFC_lane {lname}", None)
            if lane is None:
                continue
            for m in list(getattr(lane, "map", None) or []):
                afc.tool_cmds.pop(m, None)
            for m, owner in list(afc.tool_cmds.items()):
                if owner == lname:
                    afc.tool_cmds.pop(m, None)
            # Pin the lane to its home tool T<lane number> (lane12 -> T12)
            # instead of letting TcmdAssign hand out the lowest-free one, which
            # depends on claim order. A fixed home tool keeps T# equal to
            # lane# whatever order units claim in. Pre-seeding lane.map makes
            # TcmdAssign register this tool (it only scans when map is empty);
            # pop any stale owner of the home tool first.
            ds = "".join(c for c in lname if c.isdigit())
            if ds:
                home = "T" + ds
                afc.tool_cmds.pop(home, None)
                lane.map = [home]
                lane._map = []
                lane.current_map = home
            else:
                lane.map = []
                lane._map = []
                lane.current_map = ""
        activated = []
        for lname in chosen["lanes"]:
            lane = self.printer.lookup_object(f"AFC_lane {lname}", None)
            if lane is None:
                continue
            try:
                activate_from_pool(lane)
                activated.append(lane)
            except Exception as ex:
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: pool lane {lname} "
                    f"activate failed: {ex}")
        # The unit has no other way back to us: it is fabricated, so it never
        # parsed a config section naming this master. Set before claim() so
        # anything the claim itself learns can already be persisted.
        try: unit.set_master(self)
        except Exception: pass
        if not unit.claim(uid, model):
            for lane in activated:
                try: deactivate_to_pool(lane)
                except Exception: pass
            if chosen.get("spare"):
                chosen["uid"] = None          # hand the spare back on failure
            return None
        # Put the spool bindings back before anything else reads the lanes,
        # but only for the same unit returning (see restore_pool_spool).
        same_unit = chosen.get("lane_spool_uid") == uid
        restored = []
        for lane in activated:
            try:
                if not same_unit:
                    lane._pool_spool = None      # different unit: clean lanes
                elif restore_pool_spool(lane) and lane.spool_id:
                    restored.append(f"{lane.name}->spool {lane.spool_id}")
            except Exception as ex:
                self.logger.debug(
                    f"AFC_BridgeBox {self.name}: pool lane {lane.name} spool "
                    f"restore failed: {ex}")
        chosen["lane_spool_uid"] = None
        if restored:
            self.logger.info(
                f"AFC_BridgeBox {self.name}: restored the held bay's spool "
                f"bindings on re-plug: {', '.join(restored)}")
        for lane in activated:
            try:
                assign_pool_tcmd(lane, afc)
            except Exception as ex:
                self.logger.warning(
                    f"AFC_BridgeBox {self.name}: TcmdAssign {lane.name} "
                    f"failed: {ex}")
        self._clear_standalone(activated)
        chosen["bound"] = uid
        # The bay's model is known now (it was fabricated as `boxed`), so a
        # model-keyed override can reach it. Done here as well as in
        # _apply_model_live because a rostered unit is claimed with its real
        # model and never produces a dialect verdict to trigger that path.
        self._reapply_model_override(unit, model)
        self.logger.info(
            f"AFC_BridgeBox {self.name}: CLAIMED {uid} as {model} onto "
            f"{chosen['name']} ({len(activated)} lanes) -- live, no restart.")
        return unit

    def _clear_standalone(self, lanes: list) -> None:
        """
        Take the extruder out of standalone mode once a live claim gives it lanes.

        An extruder with no lanes at ready registers itself as its own lane and
        sets no_lanes (AFC_extruder.handle_ready). Pool spares are unassigned at
        ready, so on a scout-only setup the extruder comes up standalone. A later
        claim runs check_lanes() through activate_from_pool, which pops that
        self-lane -- but no_lanes stays set, so is_standalone() keeps answering
        True and the toolhead sensor callback still fires the standalone
        auto-load (load_unload_sequence) the moment a claimed lane reaches the
        sensor. That loader moves the extruder stepper onto its private trapq
        while the unit's own tool_stn advance is still driving it through the
        toolhead queue -- two step sources on one stepper, which shuts Klipper
        down with "stepcompress ... Invalid sequence".

        Once the self-lane is gone the extruder has real lanes, so mirror what
        ready would have concluded had they been there: standalone off.
        """
        seen = set()
        for lane in lanes:
            ext = getattr(lane, "extruder_obj", None)
            if ext is None or id(ext) in seen:
                continue
            seen.add(id(ext))
            if not getattr(ext, "no_lanes", False):
                continue
            tc = getattr(ext, "tc_lane", None)
            if tc is not None and tc.name in getattr(ext, "lanes", {}):
                continue                      # self-lane still registered
            ext.no_lanes = False
            self.logger.info(
                f"AFC_BridgeBox {self.name}: {ext.name} left standalone mode "
                f"(claimed lanes attached)")

    #: Unanswered addressed 0x3702 queries from an online unit before its
    #: generation is called ams1. An AMS 1 is identified by the absence of an
    #: answer, so this is the confirmation window and cannot be zero. The
    #: firmware probes each unit with its own addressed 0x3702 (byte[15]=0x01);
    #: an AMS 2 answers and latches ams2 at AMS2_ANS_MIN (~3 asks, ~1.5s), while
    #: an AMS 1 stays silent. ~6s at the probe's 500ms cadence.
    _AMS1_ASK_FLOOR = 12

    def _refine_models(self, bridge: Any, uids: List[str],
                       entries: List[str], sec: str) -> List[str]:
        """
        Confirm `boxed` roster entries' generation from the bus dialect.

        `boxed` means "not confirmed yet": the 0x3702 version query separates
        the generations on the wire (an AMS 2 answers it, an AMS 1 never
        does), and the firmware counts asks and answers per unit. One
        answered ask rewrites the entry ams2; _AMS1_ASK_FLOOR unanswered asks
        from an online unit rewrite it ams1. The verdict is recorded and
        applied to the running unit (see _apply_model_live), and moves
        nothing: lane count is identical across the boxed family, so the lane
        map and unit name hold.

        :param bridge: the chain's bridge
        :param uids: chain_uids() -- index -> UID, empties kept
        :param entries: the recorded roster, parsed to entry strings
        :param sec: the state section holding the roster
        :return list: the entries, refined where the bus was decisive
        """
        getter = getattr(bridge, "chain_dialect", None)
        if not callable(getter):
            return entries                    # bridge has no dialect counters
        a2mask, a2asks = getter()
        try:
            htmask = int(bridge.chain_diag()[0])
        except Exception:
            htmask = 0
        latest = bridge.latest_status() or {}
        online_idx = {u.get("n") for u in (latest.get("units") or [])
                      if u.get("online")}
        idx_by_uid = {_norm_uid(u): i
                      for i, u in enumerate(uids) if (u or "").strip()}
        refined: List[str] = []
        changed = []
        for e in entries:
            model, _sep, uid = e.partition(":")
            uid = _norm_uid(uid)
            i = idx_by_uid.get(uid)
            if (model.strip().lower() != "boxed" or i is None
                    or htmask >> i & 1):
                # Not boxed, not on the chain, or HT-flagged: the dialect
                # verdict is a boxed-family question, and a stale boxed
                # record pointing at an HT's index must never rewrite (or
                # live-flip) an HT.
                refined.append(e)
                continue
            if a2mask >> i & 1:
                new = "ams2"
            elif (i in online_idx and i < len(a2asks)
                    and a2asks[i] >= self._AMS1_ASK_FLOOR):
                new = "ams1"
            else:
                refined.append(e)
                continue
            refined.append(f"{new}:{uid}")
            changed.append(f"{uid} -> {new}")
            # Applied live, unlike additions and removals -- those change
            # klippy's object graph (new lanes are new steppers, macros, UI
            # objects) and only a restart can do that. A refinement is a
            # label and a heater flag on objects that already exist: boxed
            # runs on the ams2 spec everywhere, so ams2 is a rename, and
            # ams1 just drops the heater it never had.
            self._apply_model_live(uid, new)
        if changed:
            self._state_set({sec: {"roster": ", ".join(refined)}})
            self.logger.info(
                f"AFC_BridgeBox {self.name}: generation confirmed by bus "
                f"dialect (0x3702): {', '.join(changed)} -- applied to the "
                f"running unit and recorded. Lanes and names do not move.")
        return refined

    #: Options a model override may re-apply to an already-fabricated unit.
    #: An allow-list: _fold_and_sweep runs once at config load
    #: and most keys it folds are wiring (lanes, hub, buffer, serial_port) that
    #: a running unit cannot re-read, so re-applying them late would desync the
    #: object from the sections Klipper actually built. These are the ones that
    #: are pure policy, settable on a live object, and pushed to the firmware.
    _LATE_MODEL_KEYS = frozenset({"measure_on_insert"})

    def _reapply_model_override(self, obj: Any, model: str) -> None:
        """
        Re-apply a ``[AFC_BridgeBox <model>]`` override once the model is known.

        A model override cannot reach a spare at config load: a pool bay is
        fabricated before anything is plugged into it, so its ams_model is the
        placeholder ``boxed`` and _fold_and_sweep looks up ``model:boxed``.
        Without this, e.g. ``[AFC_BridgeBox ams1] measure_on_insert: True``
        would match nothing and the bay would keep the spare default (False,
        see the emission site).

        :param obj: the live AFC_BambuAMS unit object
        :param model: the now-known canonical model tag
        """
        ov = (getattr(self, "_overrides", None) or {}).get(
            "model:" + _norm_model(model))
        if not ov:
            return
        # Chain defaults sit below model, and a per-unit section above it, so a
        # unit named explicitly still wins -- the same precedence _fold applies.
        name = f"{self._UNIT_SECTION} {getattr(obj, 'name', '')}"
        per_unit = (getattr(self, "_overrides", None) or {}).get(name) or {}
        for key in self._LATE_MODEL_KEYS:
            if key in per_unit or key not in ov:
                continue
            val = str(ov[key]).strip().lower() in ("1", "true", "yes", "on")
            if bool(getattr(obj, key, False)) == val:
                continue
            setattr(obj, key, val)
            self.logger.info(
                f"AFC_BridgeBox {self.name}: {getattr(obj, 'name', '?')} is a "
                f"{model}; applied {key}={val} from [AFC_BridgeBox {model}] "
                f"(the bay was fabricated as a spare, before its model was "
                f"known)")
            # Push it to the firmware too: measure_on_insert lives in the
            # firmware's capen mask, which cap_open reads, so setting the
            # attribute alone leaves the measurement refused. _send_ht_flag is
            # the one place that emits {"cmd":"capen"}; it also re-asserts
            # htunit, which is idempotent.
            try:
                obj._send_ht_flag(getattr(obj, "_bridge", None))
            except Exception:
                pass

    def _apply_model_live(self, uid: str, model: str) -> None:
        """
        Flip a running unit's model in place after a dialect verdict.

        :param uid: the unit's 24-hex uid
        :param model: the confirmed model tag (ams1/ams2)
        """
        try:
            for u in self.units:
                if u.get("uid") == uid:
                    u["model"] = model
            for _name, obj in self.printer.lookup_objects("AFC_BambuAMS"):
                if str(getattr(obj, "unit_uid", "")).upper() == uid:
                    obj.ams_model = model
                    if model == "ams1":
                        obj.has_heater = False
                    # The model is only now known, so this is the first moment
                    # a model-keyed override can be applied to this bay.
                    self._reapply_model_override(obj, model)
        except Exception:
            pass                              # the record still applies later

    def _prune_missing(self, bridge: Any, uids: List[str],
                       entries: List[str], eventtime: float,
                       sec: str) -> None:
        """
        Drop rostered units that have been provably gone for removal_grace.

        Presence, not enrollment, is the evidence: the firmware's chain map
        keeps a unit enrolled after it is unplugged, so membership in the
        chain reply cannot prove absence. What can is the per-unit online
        flag in the status stream -- enrolled at index i AND units[i].online.
        Absence only counts while it is distinguishable from an outage:
        the serial link must be up and at least one unit must be online,
        otherwise "everything is missing" reads as "the chain is off", every
        clock resets, and nothing is removed. (Corollary: the sole unit of a
        single-unit chain is never auto-removed -- its absence and a chain
        power-off look identical. Edit the recorded roster by hand for that.)

        The removal itself only rewrites the record. This session's
        fabricated sections, lanes, and bridge polling stand until the next
        restart -- the same apply-at-restart contract as additions -- and
        the unit's lane_map entry and learned values are deliberately kept,
        so plugging it back in re-adds it with its lanes, its Spoolman
        bindings, and its calibration intact.

        :param bridge: the chain's bridge
        :param uids: chain_uids() -- index -> UID, empties kept
        :param entries: the recorded roster, parsed to entry strings
        :param eventtime: reactor time
        :param sec: the state section holding the roster
        """
        # With the pool active, a removed unit's slot is deliberately kept for
        # re-plug (its lanes drop live; the record stays), so never prune the
        # roster here -- pruning it would drop the named slot and a re-plug
        # would land on a generic spare (pool_N) instead of its own name.
        # AFC_BRIDGEBOX_FORGET is the explicit way to release a slot for good.
        if self.pool_ams or self.pool_ht:
            self._watch_state = "watching"
            return
        if not self.removal_grace:
            self._watch_state = "removal-disabled"
            return
        if getattr(bridge, "_serial", None) is None:
            self._watch_state = "link-down"
            self._missing_since.clear()
            return
        latest = bridge.latest_status() or {}
        online_idx = {u.get("n") for u in (latest.get("units") or [])
                      if u.get("online")}
        if not online_idx:
            self._watch_state = "chain-dark"
            self._missing_since.clear()
            return
        self._watch_state = "watching"

        present = {_norm_uid(u)
                   for i, u in enumerate(uids) if i in online_idx}
        survivors: List[str] = []
        dropped: List[str] = []
        for e in entries:
            uid = _norm_uid(e.partition(":")[2])
            if uid in present:
                self._missing_since.pop(uid, None)
                survivors.append(e)
                continue
            if uid not in self._missing_since:
                # Announce that the countdown started, so a running countdown
                # is distinguishable from a stalled watch.
                self._missing_since[uid] = eventtime
                name = getattr(self, "_name_map", {}).get(uid, uid)
                self.logger.info(
                    f"AFC_BridgeBox {self.name}: {name} ({uid}) is offline "
                    f"on a live chain -- if it stays gone "
                    f"{self.removal_grace:.0f}s it will be recorded as "
                    f"removed (takes effect at the next RESTART).")
            since = self._missing_since[uid]
            if eventtime - since < self.removal_grace:
                survivors.append(e)
                continue
            self._missing_since.pop(uid, None)
            dropped.append(e)
        if dropped:
            self._state_set({sec: {"roster": ", ".join(survivors)}})
            self.logger.info(
                f"AFC_BridgeBox {self.name}: unit(s) gone from the chain for "
                f"over {self.removal_grace:.0f}s: {', '.join(dropped)} -- "
                f"removed from the recorded roster; applies at the next "
                f"RESTART. Their lane numbers and learned values are kept, "
                f"so plugging one back in restores it unchanged. If one is "
                f"never coming back, AFC_BRIDGEBOX_FORGET UID=<uid> frees "
                f"its lanes and name for reuse.")

    def _pending_restart(self) -> List[str]:
        """
        :return list: human-readable diffs between the recorded roster and
            the running enrollment -- empty when a restart would change
            nothing. Reads the tick's cached copy of the record, so a
            status poll costs no file I/O.
        """
        raw = getattr(self, "_recorded_raw", None)
        if raw is None:
            return []
        try:
            recorded = self._parse_roster(raw) if raw.strip() else []
        except Exception:
            return []
        rec = {u["uid"]: u["model"] for u in recorded}
        run = {u["uid"]: u["model"] for u in self.units}
        out: List[str] = []
        for uid, model in rec.items():
            if uid not in run:
                out.append(f"add {model}:{uid}")
            elif run[uid] != model:
                out.append(f"{run[uid]} -> {model}: {uid}")
        for uid, model in run.items():
            if uid not in rec:
                out.append(f"remove {model}:{uid}")
        return sorted(out)

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        :param eventtime: reactor time (unused)
        :return dict: the roster as fabricated, for the status API
        """
        enrolled = {u["uid"] for u in self.units}
        try:
            now = self.printer.get_reactor().monotonic()
        except Exception:
            now = 0.0
        # Chain dialect, per chain index: is the unit enumerated
        # and online, is it HT-flagged or has it answered 0x3702 (ams2), and how
        # many 0x3702 asks has it logged toward the ams1 floor.
        dialect: Dict[str, Any] = {}
        try:
            bridge = (self._chain_bridge()
                      or getattr(self, "_bridge", None))
            if bridge is not None and getattr(bridge, "_serial", None) is not None:
                a2mask, a2asks = bridge.chain_dialect()
                htmask = int(bridge.chain_diag()[0])
                latest = bridge.latest_status() or {}
                online = {u.get("n") for u in (latest.get("units") or [])
                          if u.get("online")}
                uids = bridge.chain_uids()
                dialect = {
                    "ams1_ask_floor": self._AMS1_ASK_FLOOR,
                    "chain": [
                        {"i": i, "uid": (u or "").upper(),
                         "online": i in online,
                         "ht": bool(htmask >> i & 1),
                         "answered_3702": bool(a2mask >> i & 1),
                         "asks_3702": a2asks[i] if i < len(a2asks) else 0}
                        for i, u in enumerate(uids) if (u or "").strip()],
                }
        except Exception as e:
            dialect = {"error": str(e)}
        # Fresh dict copies, not list(self.units): _apply_model_live flips a
        # unit's model in place, and Moonraker detects status changes by
        # diffing against its cached copy of the last result. Shared dicts
        # would mutate that cached copy too, so the change would never be
        # published.
        return {"units": [dict(u) for u in self.units],
                "lane_base": self.lane_base,
                "dialect": dialect,
                "roster_source": self._roster_source,
                # Departed units still holding lanes/names -- the ones
                # AFC_BRIDGEBOX_FORGET exists for.
                "tombstones": sorted(
                    set(getattr(self, "_lane_map", {})) - enrolled),
                # The removal debounce: how long each rostered-but-absent uid
                # has been gone (seconds), and whether the chain watch is
                # ticking at all.
                "missing": {uid: int(now - t)
                            for uid, t in self._missing_since.items()},
                # The recorded chain vs the running one: roster additions
                # and removals land in the record and apply at the next
                # restart, so this lists what the next restart will change.
                "pending_restart": self._pending_restart(),
                "watch_ticks": getattr(self, "_tick_count", 0),
                # Which branch the last tick took.
                "watch_state": getattr(self, "_watch_state", "not-started")}


class BridgeBoxOverrideHolder:
    """A [AFC_BridgeBox <name>] section with no serial_port is an override
    carrier, not a second chain master:

        [AFC_BridgeBox ht]              # every HT on the chain
        measure_on_insert: False

        [AFC_BridgeBox ams2]            # every AMS 2 Pro
        dry_max_temp: 60

        [AFC_BridgeBox Bambu_AMS_1]     # this one unit
        measure_on_insert: True

        [AFC_BridgeBox AFC_hub Bambu_AMS_1]   # any fabricated section,
        afc_bowden_length: 1800              # named exactly

    The keys overlay the fabricated values at config time (identity keys
    protected), win over learned values, and never persist into state --
    deleting the section removes the override. This class only makes the
    section legal to klippy and consumes its options; the master reads the
    values from the merged fileconfig it already holds.
    """

    def __init__(self, config: Any) -> None:
        """
        Hold an override section, reading every option so Klipper accepts it.

        :param config: the override section
        """
        self.name = config.get_name().split(None, 1)[-1]
        try:
            for opt in config.get_prefix_options(""):
                config.get(opt)
        except Exception:
            pass


def load_config_prefix(config: Any) -> Any:
    # serial_port is the discriminator: every chain master requires one,
    # and no override should ever carry one (it is an identity key the
    # fold refuses anyway).
    """
    Build a chain master, or an override holder when the section has no serial_port.

    :param config: the [AFC_BridgeBox <name>] section
    :return Any: afcBridgeBox or BridgeBoxOverrideHolder
    """
    if config.get("serial_port", None) is None:
        return BridgeBoxOverrideHolder(config)
    return afcBridgeBox(config)
