"""AFC_BridgeBox: the chain master that fabricates BambuAMS unit sections.

This is the proof-of-wiring for chain auto-configuration, so what these tests
hold still is the WIRING: exactly which sections a roster implies, with which
keys, in which load order, and every refusal. Value plumbing through real
klippy config machinery is the printer's half of the proof -- the module goes
through the same RawConfigParser -> ConfigWrapper -> load_object path
add_filament_switch has always used, and the fail-soft serial design means a
fabricated unit on a dead port boots to "offline", not to a dead printer.
"""
from __future__ import annotations

import configparser
import inspect
import types

import pytest

from extras.AFC_BambuAMS import _AMS_MODELS, afcBambuAMS
from extras.AFC_BridgeBox import afcBridgeBox, load_config_prefix


class _Printer:
    def __init__(self, objects=None):
        self.objects = dict(objects or {})
        self.loaded = []                    # (section, wrapper) in call order
        self.handlers = []                  # (event, callback)
        self.start_args = {}

    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def load_object(self, wrapper, section):
        self.loaded.append((section, wrapper))
        self.objects[section] = object()
        return self.objects[section]

    def lookup_objects(self, module=None):
        return [(n, o) for n, o in self.objects.items()
                if module is None or n == module
                or n.startswith(module + " ")]


class _Config:
    error = configparser.Error

    def __init__(self, opts, printer=None, name="AFC_BridgeBox chain1"):
        self._opts = dict(opts)
        self._printer = printer or _Printer()
        self._name = name

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

    def get(self, key, default=None):
        return self._opts.get(key, default)

    def getint(self, key, default=0, **kw):
        return int(self._opts.get(key, default))

    def getfloat(self, key, default=0.0, **kw):
        return float(self._opts.get(key, default))

    def getboolean(self, key, default=False, **kw):
        v = self._opts.get(key, default)
        if isinstance(v, bool):
            return v
        return str(v).strip().lower() in ("1", "true", "yes", "on")


def _mk(roster="ht:0123456789ABCDEF00003331", **over):
    # state_file points into a directory that does not exist: reads come
    # back empty and the best-effort writes (lane/name map flush) go
    # nowhere, so no _mk test can touch the real ~/printer_data.
    opts = {"serial_port": "/dev/serial/by-id/usb-Pico-if00",
            "extruder": "extruder", "buffer": "Bamb_1",
            "lane_base": 24, "roster": roster,
            # Pool spares are an additive layer these roster-level tests do not
            # exercise; default them off so a test sees only the roster unit's
            # fabricated sections. A pool-specific test can pass pool_ams/pool_ht.
            "pool_ams": 0, "pool_ht": 0,
            "state_file": "/nonexistent-bridgebox-test/state.cfg"}
    opts.update(over)
    printer = over.pop("printer", None) or _Printer()
    cfg = _Config(opts, printer)
    return load_config_prefix(cfg), printer


# ── the fabricated shape, held exactly ────────────────────────────────────────

class TestFabricatedSections:
    def test_an_ht_is_unit_lane_hub_sensor_in_that_order(self):
        m, printer = _mk()
        names = [s for s, _w in printer.loaded]
        assert names == [
            "AFC_BambuAMS Bambu_AMS_HT_1",
            "AFC_lane lane24",
            "AFC_hub Bambu_AMS_HT_1",
            "temperature_sensor Bambu_AMS_HT_1",
        ]

    def test_the_unit_keys_mirror_the_handwritten_block(self):
        m, printer = _mk()
        sections = dict(m._roster_sections(m.units))
        u = sections["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u["serial_port"] == "/dev/serial/by-id/usb-Pico-if00"
        assert u["ams_model"] == "ht"
        assert u["extruder"] == "extruder"
        assert u["buffer"] == "Bamb_1"
        assert u["hub"] == "Bambu_AMS_HT_1"
        assert u["unit_uid"] == "0123456789ABCDEF00003331"
        assert u["heater"] is True            # an HT dries
        # getint on the consuming side: "85.0" halts the printer at config
        # parse ("Unable to parse option") -- the value must stringify with
        # no decimal point.
        assert u["dry_max_temp"] == 85 and isinstance(u["dry_max_temp"], int)
        assert "." not in str(u["dry_max_temp"])

    def test_the_lane_points_at_its_unit_and_slot(self):
        m, printer = _mk()
        sections = dict(m._roster_sections(m.units))
        assert sections["AFC_lane lane24"] == {"unit": "Bambu_AMS_HT_1:1",
                                               "unassigned": True}

    def test_the_hub_is_virtual_with_the_masters_bowden(self):
        m, printer = _mk(afc_bowden_length=2100.0, td1_bowden_length=850.0)
        sections = dict(m._roster_sections(m.units))
        h = sections["AFC_hub Bambu_AMS_HT_1"]
        assert h["switch_pin"] == "virtual"
        assert h["afc_bowden_length"] == 2100.0
        assert h["afc_unload_bowden_length"] == 2100.0
        assert h["td1_bowden_length"] == 850.0

    def test_every_unit_gets_a_temperature_card_sensor(self):
        # Humidity comes off every AMS's reporting; the sensor itself
        # suppresses temperature on units with no drying chamber. aht2x is
        # the sensor_type on Fluidd's shows-humidity list, and bambu_unit
        # routes the section to the dispatch factory the unit registers.
        m, printer = _mk(roster="ht:AAAA, boxed:BBBB")
        sections = dict(m._roster_sections(m.units))
        for name in ("Bambu_AMS_HT_1", "Bambu_AMS_1"):
            s = sections[f"temperature_sensor {name}"]
            assert s == {"sensor_type": "aht2x", "bambu_unit": name,
                         "min_temp": 0, "max_temp": 90}

    def test_a_boxed_unit_takes_four_lanes(self):
        m, printer = _mk(roster="boxed:AAAABBBBCCCCDDDD")
        names = [s for s, _w in printer.loaded]
        assert names == [
            "AFC_BambuAMS Bambu_AMS_1",
            "AFC_lane lane24", "AFC_lane lane25",
            "AFC_lane lane26", "AFC_lane lane27",
            "AFC_hub Bambu_AMS_1",
            "temperature_sensor Bambu_AMS_1",
        ]
        lanes = dict(m._roster_sections(m.units))
        assert lanes["AFC_lane lane27"] == {"unit": "Bambu_AMS_1:4",
                                            "unassigned": True}

    def test_an_ams2_pin_gets_its_own_ceiling_not_the_hts(self):
        # boxed -> ams2 is the confirmation path (the operator knows the
        # hardware); the pin must bring the AMS 2 Pro's 65C ceiling, not the
        # shared 85 the HT uses -- 20C past what the hardware honours. The
        # generation steers heater/ceiling but NOT the name (Bambu_AMS_1).
        m, printer = _mk(roster="ht:AAAA, ams2:BBBB")
        sections = dict(m._roster_sections(m.units))
        assert sections["AFC_BambuAMS Bambu_AMS_HT_1"]["dry_max_temp"] == 85
        u = sections["AFC_BambuAMS Bambu_AMS_1"]
        assert u["ams_model"] == "ams2"
        assert u["heater"] is True
        assert u["dry_max_temp"] == 65

    def test_an_explicit_dry_max_temp_still_caps_every_heater(self):
        m, printer = _mk(roster="ht:AAAA, ams2:BBBB", dry_max_temp=60)
        sections = dict(m._roster_sections(m.units))
        assert sections["AFC_BambuAMS Bambu_AMS_HT_1"]["dry_max_temp"] == 60
        assert sections["AFC_BambuAMS Bambu_AMS_1"]["dry_max_temp"] == 60

    def test_a_chain_dry_max_temp_caps_but_never_raises_a_ceiling(self):
        # 80 holds the HT (85) down to 80 but leaves the AMS 2 Pro at its
        # own 65: the chain value is a cap, not a replacement.
        m, printer = _mk(roster="ht:AAAA, ams2:BBBB", dry_max_temp=80)
        sections = dict(m._roster_sections(m.units))
        assert sections["AFC_BambuAMS Bambu_AMS_HT_1"]["dry_max_temp"] == 80
        assert sections["AFC_BambuAMS Bambu_AMS_1"]["dry_max_temp"] == 65

    @pytest.mark.parametrize("pool", [dict(pool_ams=2), dict(roster="")],
                             ids=["pool", "bridge-only-scout"])
    def test_a_negative_dry_max_temp_is_unset_and_never_stops_a_start(
            self, tmp_path, pool):
        # klippy's getint enforces minval; 5900c01 read dry_max_temp with
        # none, so a negative value must not become a config error now.
        class _Strict(_Config):
            def getint(self, key, default=0, minval=None, **kw):
                v = int(self._opts.get(key, default))
                if minval is not None and v < minval:
                    raise self.error(f"Option '{key}' must have minimum "
                                     f"of {minval}")
                return v
        opts = {"serial_port": "/dev/serial/by-id/usb-Pico-if00",
                "extruder": "extruder", "buffer": "Bamb_1", "lane_base": 24,
                "roster": "", "pool_ams": 0, "pool_ht": 0,
                "dry_max_temp": -1,
                "auto_vars_file": str(tmp_path / "AFC_auto_vars.cfg"),
                "state_file": str(tmp_path / "AFC_BridgeBox.cfg")}
        opts.update(pool)
        printer = _Printer()
        m = load_config_prefix(_Strict(opts, printer))
        assert m.dry_max_temp == 0
        log = _Logger()
        printer.objects["AFC"] = types.SimpleNamespace(logger=log)
        printer.get_reactor = lambda: _FakeReactor()
        m._scout_ready()
        (note,) = [n for n in log.lines if "dry_max_temp" in n]
        assert "dry_max_temp is -1, below 0, so it is ignored" in note

    def test_an_unconfirmed_boxed_unit_gets_no_heater_key(self):
        # `boxed` means "generation not yet confirmed" -- an AMS 1 given
        # heater: True invites commands the unit ignores, and the lane count
        # is the same either way, which is the whole point of the tag.
        m, printer = _mk(roster="boxed:AAAABBBBCCCCDDDD")
        u = dict(m._roster_sections(m.units))["AFC_BambuAMS Bambu_AMS_1"]
        assert "heater" not in u

    def test_ams_takes_the_low_band_ht_the_high_band_regardless_of_order(self):
        # FIXED bands: the 4-slot AMS pool occupies the low lanes and the HT
        # band sits above it, no matter the roster order. The HT is listed
        # FIRST here, yet the boxed AMS still takes the low block (24-27) and
        # the HT lands in its own band above it (28) -- it never interleaves.
        m, printer = _mk(roster="ht:1111222233334444, boxed:5555666677778888")
        lanes = [s for s, _w in printer.loaded if s.startswith("AFC_lane")]
        assert lanes == [f"AFC_lane lane{n}" for n in (24, 25, 26, 27, 28)]
        sections = dict(m._roster_sections(m.units))
        assert sections["AFC_lane lane24"]["unit"] == "Bambu_AMS_1:1"     # ams low
        assert sections["AFC_lane lane27"]["unit"] == "Bambu_AMS_1:4"
        assert sections["AFC_lane lane28"]["unit"] == "Bambu_AMS_HT_1:1"  # ht above

    def test_each_family_numbers_independently(self):
        # Two HTs and a boxed: HT_1, HT_2, and the boxed is Bambu_AMS_1 --
        # not _3. The number says "which HT", the way the hand-written names
        # (BambuAMS_1, BambuAMS_2, BambuAMS_HT) always read. Units fabricate
        # ams-first (low band) then ht, so compare the numbering order-free.
        m, printer = _mk(roster="ht:AAAA, ht:BBBB, boxed:CCCC")
        units = [s.split()[1] for s, _w in printer.loaded
                 if s.startswith("AFC_BambuAMS")]
        assert sorted(units) == ["Bambu_AMS_1", "Bambu_AMS_HT_1",
                                 "Bambu_AMS_HT_2"]

    def test_a_second_chain_can_set_its_own_prefix(self):
        m, printer = _mk(roster="ht:AAAA", unit_prefix="Bambu_AMS_B")
        units = [s.split()[1] for s, _w in printer.loaded
                 if s.startswith("AFC_BambuAMS")]
        assert units == ["Bambu_AMS_B_HT_1"]

    def test_the_same_roster_always_fabricates_the_same_names(self):
        # Spoolman bindings and T# macros hang off these names; a roster that
        # renamed anything between restarts would orphan them all.
        a, _p1 = _mk(roster="ht:1111222233334444, boxed:5555666677778888")
        b, _p2 = _mk(roster="ht:1111222233334444, boxed:5555666677778888")
        assert a._roster_sections(a.units) == b._roster_sections(b.units)


# ── fixed bands: AMS low, HT high, no collapse; FORGET frees for reuse ────────

class TestFlatPool:
    def _units(self, m):
        secs = dict(m._roster_sections(m.units))
        return {f"lane{n}": secs[f"AFC_lane lane{n}"]["unit"]
                for n in range(12, 44) if f"AFC_lane lane{n}" in secs}

    def test_two_ams_then_ht_pack_12_16_20(self):
        # The operator's case: two boxed AMS occupy 12-15 and 16-19, and the HT
        # band starts right above the AMS block at 20.
        m, _p = _mk(roster="boxed:AAAA, boxed:BBBB, ht:CCCC", lane_base=12)
        u = self._units(m)
        assert u["lane12"] == "Bambu_AMS_1:1"
        assert u["lane16"] == "Bambu_AMS_2:1"
        assert u["lane20"] == "Bambu_AMS_HT_1:1"

    def test_every_ams_sits_below_the_ht_band(self):
        # A 3rd AMS extends the AMS block (12,16,20) and the HT stays ABOVE all
        # of them (24). AMS never lands inside the HT band -- the two never
        # interleave, whichever order they enrol in.
        m, _p = _mk(roster="boxed:AAAA, boxed:BBBB, ht:CCCC, boxed:DDDD",
                    lane_base=12)
        u = self._units(m)
        assert u["lane12"] == "Bambu_AMS_1:1"
        assert u["lane16"] == "Bambu_AMS_2:1"
        assert u["lane20"] == "Bambu_AMS_3:1"         # 3rd AMS extends the block
        assert u["lane24"] == "Bambu_AMS_HT_1:1"      # HT above every AMS

    def test_ht_never_takes_an_ams_band_lane_even_first_in_roster(self):
        # Roster order does not move a unit between bands: an HT enrolled first
        # still lands in the HT band (16) above the AMS block (12-15), never on
        # a low AMS lane.
        m, _p = _mk(roster="ht:CCCC, boxed:AAAA", lane_base=12)
        u = self._units(m)
        assert u["lane12"] == "Bambu_AMS_1:1"
        assert u["lane16"] == "Bambu_AMS_HT_1:1"

    def test_declared_pool_ams_pins_the_ht_band_so_it_cannot_collapse(self):
        # The real guarantee: with a DECLARED pool (pool_ams=4), the AMS band is
        # reserved at its full width (4*4=16 lanes) whether or not that many AMS
        # are online, so the HT band base is fixed at lane_base+16 regardless of
        # the live AMS count -- it never collapses toward lower lanes on a
        # restart with fewer AMS present.
        one = self._units(_mk(roster="boxed:AAAA, ht:CCCC",
                              lane_base=12, pool_ams=4, pool_ht=8)[0])
        four = self._units(_mk(
            roster="boxed:AAAA, boxed:BBBB, boxed:EEEE, boxed:FFFF, ht:CCCC",
            lane_base=12, pool_ams=4, pool_ht=8)[0])
        assert one["lane28"] == "Bambu_AMS_HT_1:1"    # 1 AMS online -> HT at 28
        assert four["lane28"] == "Bambu_AMS_HT_1:1"   # 4 AMS online -> still 28

    def test_forget_frees_the_lane_and_name_for_the_next_unit(self, tmp_path):
        # FORGET clears a unit's lanes AND name; the next new unit reuses the
        # lowest freed lane and the freed name.
        m1, _p, _a, _s = _mk_files(
            tmp_path, roster="boxed:AAAA, boxed:BBBB, ht:CCCC", lane_base=12)
        m1.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))   # frees AMS_1 @ 12-15
        m2, _p2, _a2, _s2 = _mk_files(
            tmp_path, roster="boxed:BBBB, ht:CCCC, boxed:DDDD", lane_base=12)
        secs = dict(m2._roster_sections(m2.units))
        assert secs["AFC_lane lane12"]["unit"] == "Bambu_AMS_1:1"   # reused
        assert secs["AFC_BambuAMS Bambu_AMS_1"]["unit_uid"] == "DDDD"
        assert secs["AFC_BambuAMS Bambu_AMS_2"]["unit_uid"] == "BBBB"  # kept

    def test_a_removed_tombstone_never_renumbers_a_survivor(self, tmp_path):
        # Remove (not forget) AAAA: it keeps its reserved lanes/name. A new
        # unit skips the reservation and takes a fresh name -- BBBB is never
        # renumbered, and AAAA's name is never handed out.
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB", lane_base=12)
        m2, _p2, _a2, _s2 = _mk_files(
            tmp_path, roster="boxed:BBBB, boxed:DDDD", lane_base=12)
        secs = dict(m2._roster_sections(m2.units))
        # AAAA tombstone holds 12-15 / AMS_1; BBBB keeps AMS_2 @ 16-19;
        # DDDD is new -> next free block (20-23) and next free name (AMS_3).
        assert secs["AFC_BambuAMS Bambu_AMS_2"]["unit_uid"] == "BBBB"
        assert secs["AFC_lane lane16"]["unit"] == "Bambu_AMS_2:1"
        assert secs["AFC_BambuAMS Bambu_AMS_3"]["unit_uid"] == "DDDD"
        assert secs["AFC_lane lane20"]["unit"] == "Bambu_AMS_3:1"


# ── refusals ─────────────────────────────────────────────────────────────────

class TestRefusals:
    def test_a_name_collision_refuses_before_loading_anything(self):
        printer = _Printer({"AFC_lane lane24": object()})
        with pytest.raises(configparser.Error) as e:
            _mk(printer=printer)
        assert "lane24" in str(e.value)
        assert printer.loaded == []           # nothing half-fabricated

    def test_an_empty_roster_scouts_rather_than_halting(self):
        # Changed contract, deliberately: empty/absent used to be a config
        # error, but that made from-scratch setup a chicken-and-egg (see
        # TestScouting). Malformed non-empty rosters still halt loudly.
        m, printer = _mk(roster="")
        assert printer.loaded == []
        assert m.get_status()["roster_source"] == "scout"

    def test_a_malformed_entry_names_itself(self):
        with pytest.raises(configparser.Error) as e:
            _mk(roster="0123456789ABCDEF00003331")   # uid with no model
        assert "0123456789ABCDEF00003331" in str(e.value)

    def test_an_unknown_model_lists_the_known_ones(self):
        with pytest.raises(configparser.Error) as e:
            _mk(roster="ams9:1111222233334444")
        assert "ams9" in str(e.value) and "ht" in str(e.value)

    def test_a_duplicated_uid_is_refused(self):
        with pytest.raises(configparser.Error) as e:
            _mk(roster="ht:AAAA, boxed:aaaa")        # same uid, case aside
        assert "twice" in str(e.value)


# ── status ───────────────────────────────────────────────────────────────────

def test_status_reports_the_roster_as_fabricated():
    m, _printer = _mk(roster="ht:1111222233334444")
    st = m.get_status()
    assert st["units"] == [{"model": "ht", "uid": "1111222233334444"}]
    assert st["lane_base"] == 24


# ── fold and sweep: the AFC_auto_vars loop, closed ───────────────────────────

class _FileConfig:
    """The merged klippy fileconfig, as the orphan check consults it."""

    def __init__(self, sections=None):
        self._s = {k: dict(v) for k, v in (sections or {}).items()}

    def has_section(self, name):
        return name in self._s

    def sections(self):
        return list(self._s)

    def items(self, name):
        return list(self._s[name].items())


def _mk_files(tmp_path, autov_text=None, store_text=None, fileconfig=None,
              roster="ht:0123456789ABCDEF00003331", getsection=None, **over):
    autov = tmp_path / "AFC_auto_vars.cfg"
    if autov_text is not None:
        autov.write_text(autov_text)
    store = tmp_path / "AFC_BridgeBox_chain1.vars"
    if store_text is not None:
        store.write_text(store_text)
    opts = {"serial_port": "/dev/serial/by-id/usb-Pico-if00",
            "extruder": "extruder", "buffer": "Bamb_1", "lane_base": 24,
            "roster": roster, "auto_vars_file": str(autov),
            # Pool spares are an additive layer these roster-level tests do not
            # exercise; default them off so a test sees only the roster unit's
            # fabricated sections. A pool-specific test can pass pool_ams/pool_ht.
            "pool_ams": 0, "pool_ht": 0,
            "state_file": str(tmp_path / "AFC_BridgeBox.cfg")}
    printer = over.pop("printer", None) or _Printer()
    opts.update(over)
    cfg = _Config(opts, printer)
    if fileconfig is not None:
        cfg.fileconfig = fileconfig
    if getsection is not None:
        cfg.getsection = getsection
    return load_config_prefix(cfg), printer, autov, store


def _mk_recorded(tmp_path, roster, **over):
    """_mk_files with no roster: option: the roster is the recorded one, as a
    scout leaves it, so ASSIGN may pin (and enroll) a uid it does not list."""
    (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(
        "\n".join(e.strip() for e in roster.split(",")) + "\n")
    return _mk_files(tmp_path, roster="", **over)


class TestFoldAndSweep:
    AUTOV = ("[AFC_BambuAMS Bambu_AMS_HT_1]\n"
             "afc_bowden_length : 3632.0\n"
             "afc_unload_bowden_length : 3632.0\n")

    def test_a_learned_value_reaches_the_fabricated_section(self, tmp_path):
        # THE LIVE SEQUENCE. The unit adopted its dw_len-measured path and
        # ConfigRewrite filed it in AFC_auto_vars.cfg; the fabricated unit's
        # private parser never saw it, so it re-learned every session.
        m, printer, autov, store = _mk_files(tmp_path, autov_text=self.AUTOV)
        assert "AFC_BambuAMS Bambu_AMS_HT_1" in dict(printer.loaded)
        folded = dict(m._fold_and_sweep(
            _Config({}, printer), m._roster_sections(m.units)))
        assert folded["AFC_BambuAMS Bambu_AMS_HT_1"][
            "afc_bowden_length"] == "3632.0"

    def test_the_folded_section_is_swept_out_of_auto_vars(self, tmp_path):
        m, printer, autov, store = _mk_files(tmp_path, autov_text=self.AUTOV)
        assert "Bambu_AMS_HT_1" not in autov.read_text()

    def test_and_survives_in_the_state_file_for_the_next_boot(self, tmp_path):
        m, printer, autov, store = _mk_files(tmp_path, autov_text=self.AUTOV)
        text = (tmp_path / "AFC_BridgeBox.cfg").read_text()
        # Filed under the unit's uid, not its bay name: the bay is worn by
        # whichever unit claims it.
        assert m._learned_section("0123456789ABCDEF00003331") in text
        assert "3632.0" in text
        # Every state line rides in the managed comment block -- klippy must
        # never parse a word of it. That property is the whole reason the
        # auto_vars ghost/halt class cannot return through this file.
        for ln in text.splitlines():
            if "3632.0" in ln or "Bambu_AMS_HT_1" in ln:
                assert ln.startswith("#~#")
        # Round trip: a fresh boot with the swept auto_vars still folds the
        # value in, from the store.
        m2, p2, _a, _s = _mk_files(tmp_path)
        folded = dict(m2._fold_and_sweep(
            _Config({}, p2), m2._roster_sections(m2.units)))
        assert folded["AFC_BambuAMS Bambu_AMS_HT_1"][
            "afc_bowden_length"] == "3632.0"

    def test_an_orphan_for_a_renamed_unit_is_deleted(self, tmp_path):
        # The rename time bomb: [AFC_BambuAMS chain1_ht0_3331] left in
        # auto_vars after nothing fabricates that name. Its keys in the
        # merged fileconfig equal its auto_vars keys -- declared nowhere
        # else -- so it is provably a leftover.
        stale = ("[AFC_BambuAMS chain1_ht0_3331]\n"
                 "afc_bowden_length : 3632.0\n")
        fc = _FileConfig({"AFC_BambuAMS chain1_ht0_3331":
                          {"afc_bowden_length": "3632.0"}})
        m, printer, autov, store = _mk_files(
            tmp_path, autov_text=stale, fileconfig=fc)
        assert "chain1_ht0_3331" not in autov.read_text()

    def test_a_handwritten_units_entry_is_left_alone(self, tmp_path):
        # BambuAMS_HT exists in real config (extra keys beyond auto_vars),
        # so its auto_vars entry is a legitimate merge target -- not ours.
        entry = ("[AFC_BambuAMS BambuAMS_HT]\n"
                 "afc_bowden_length : 2100.0\n")
        fc = _FileConfig({"AFC_BambuAMS BambuAMS_HT":
                          {"afc_bowden_length": "2100.0",
                           "serial_port": "/dev/real"}})
        m, printer, autov, store = _mk_files(
            tmp_path, autov_text=entry, fileconfig=fc)
        assert "BambuAMS_HT" in autov.read_text()

    def test_a_leftover_cannot_rewrite_identity(self, tmp_path):
        # Folding is for calibrations, never wiring: a serial_port or
        # unit_uid in a leftover file must not reach the fabricated section.
        evil = ("[AFC_BambuAMS Bambu_AMS_HT_1]\n"
                "serial_port : /dev/evil\n"
                "unit_uid : FFFFFFFFFFFFFFFFFFFFFFFF\n"
                "afc_bowden_length : 3632.0\n")
        m, printer, autov, store = _mk_files(tmp_path, autov_text=evil)
        folded = dict(m._fold_and_sweep(
            _Config({}, printer), m._roster_sections(m.units)))
        u = folded["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u["serial_port"] == "/dev/serial/by-id/usb-Pico-if00"
        assert u["unit_uid"] == "0123456789ABCDEF00003331"
        assert u["afc_bowden_length"] == "3632.0"

    def test_no_files_at_all_is_a_clean_noop(self, tmp_path):
        m, printer, autov, store = _mk_files(tmp_path)
        assert not autov.exists()
        assert not store.exists()

    def test_a_lane_and_hub_no_longer_built_are_moved_and_stood_in_for(
            self, tmp_path):
        # An earlier start built eight AMS bays and put H1 on lane56, and AFC
        # saved values for lane56 and Bambu_AMS_5's hub to auto_vars. Now four
        # AMS bays are built and H1 sits on lane40: klippy parsed those two
        # sections at this start, and an [AFC_lane] with no unit would stop
        # Klipper from starting.
        from extras.AFC_BridgeBox import _SweptSection

        class _Adding(_Printer):
            def add_object(self, name, obj):
                assert name not in self.objects
                self.objects[name] = obj

        class _Section:
            def __init__(self, used, name):
                self._used, self._name = used, name

            def get(self, opt, default=None):
                self._used.add((self._name.lower(), opt.lower()))
                return default

        m0 = _mk_files(tmp_path, roster="")[0]
        m0._state_set({"AFC_BridgeBox chain1": {
            "roster": "boxed:A1, ht:H1", "lane_base": "24",
            "lane_map": "A1:24:4, H1:56:1",
            "name_map": "A1:Bambu_AMS_1, H1:Bambu_AMS_HT_1"}})
        autov_text = ("[AFC_lane lane56]\nremember_spool : False\n\n"
                      "[AFC_hub Bambu_AMS_5]\nafc_bowden_length : 1900\n\n"
                      "[AFC_lane lane99]\nremember_spool : False\n")
        fc = _FileConfig({
            "AFC_lane lane56": {"remember_spool": "False"},
            "AFC_hub Bambu_AMS_5": {"afc_bowden_length": "1900"},
            "AFC_lane lane99": {"remember_spool": "False"}})
        printer, used = _Adding(), set()
        m, _p, autov, _s = _mk_files(
            tmp_path, autov_text=autov_text, fileconfig=fc, roster="",
            pool_ams=8, pool_ht=2, printer=printer,
            getsection=lambda name: _Section(used, name))
        assert m._lane_map["H1"] == (40, 1)
        text = autov.read_text()
        assert "lane56" not in text and "Bambu_AMS_5" not in text
        assert "[AFC_lane lane99]" in text           # not this chain's
        assert m._state_get("AFC_lane lane56", "remember_spool") == "False"
        assert m._state_get("AFC_hub Bambu_AMS_5",
                            "afc_bowden_length") == "1900"
        for name in ("AFC_lane lane56", "AFC_hub Bambu_AMS_5"):
            assert isinstance(printer.objects[name], _SweptSection)
        assert "AFC_lane lane99" not in printer.objects
        assert used == {("afc_lane lane56", "remember_spool"),
                        ("afc_hub bambu_ams_5", "afc_bowden_length")}
        assert (False, "auto_vars [AFC_lane lane56] moved to "
                       "AFC_BridgeBox.cfg: no chain builds that section "
                       "now") in m._learned_notes
        # The stand-in is no lane: a later chain's automatic base ignores it.
        assert 56 not in m._loaded_lane_numbers()


# ── scouting: the from-scratch path ──────────────────────────────────────────

class TestScouting:
    def test_scout_mode_still_provides_the_buffer_pin_chip(self, tmp_path):
        # The dependency that halted a real printer: [AFC_buffer] with
        # adc_pin: bambu_buffer:fps loads whether or not a unit exists, and
        # the chip it references is registered BY the unit. Scout mode must
        # supply it -- stub-backed, reading "no data", which is the same
        # fail-soft state as a unit whose Pico is unplugged.
        class _Pins:
            def __init__(self):
                self.chips = {}

            def register_chip(self, name, chip):
                self.chips[name] = chip

        pins = _Pins()
        printer = _Printer({"pins": pins})
        m, printer, _a, _s = _mk_files(tmp_path, roster="", printer=printer)
        assert "bambu_buffer" in pins.chips
        chips = getattr(printer, "_bambu_buffer_chips", {})
        assert "bambu_buffer" in chips              # same dedupe the units use
        assert chips["bambu_buffer"]._unit.fps_buffer_value() is None

    def test_enrolled_mode_does_not_register_the_stub(self, tmp_path):
        class _Pins:
            def __init__(self):
                self.chips = {}

            def register_chip(self, name, chip):
                self.chips[name] = chip

        pins = _Pins()
        printer = _Printer({"pins": pins})
        m, printer, _a, _s = _mk_files(tmp_path, printer=printer)
        assert pins.chips == {}     # the real unit registers its own at load

    def test_no_roster_anywhere_scouts_instead_of_halting(self, tmp_path):
        # THE FROM-SCRATCH ANSWER. A new user cannot know a unit_uid before
        # something has talked to the chain; the old requirement was a
        # chicken-and-egg bootstrapped through a throwaway hand-written unit
        # and AFC_BAMBU_UIDS.
        m, printer, _a, _s = _mk_files(tmp_path, roster="")
        assert printer.loaded == []                 # nothing fabricated
        assert m.get_status()["roster_source"] == "scout"
        assert ("klippy:ready", m._scout_ready) in printer.handlers

    def _scout_bridge(self, tmp_path, monkeypatch, port):
        from extras import AFC_BambuAMS_bridge as _bridge_mod
        made = {}

        class _Bridge:
            def __init__(self, opener, reactor, logger):
                made["open"] = opener

            def start(self, defer_open=False):
                made["defer"] = defer_open

        monkeypatch.setattr(_bridge_mod, "BambuBridge", _Bridge)
        monkeypatch.setattr(_bridge_mod, "_BRIDGES", {})
        m, printer, _a, _s = _mk_files(tmp_path, roster="", serial_port=port,
                                       tcp_key="k")
        printer.get_reactor = lambda: None
        m.logger = None                         # set at ready; unused here
        m._ensure_bridge()
        return made

    def test_scouting_opens_a_tcp_bridge_like_the_units(self, tmp_path,
                                                        monkeypatch):
        # A WiFi bridge on a from-scratch chain: the scout opens a socket, with
        # the chain's tcp_key, and a bridge not up yet keeps retrying.
        from extras import AFC_BambuAMS_bridge as _bridge_mod
        seen = {}

        class _Tcp:
            parse = staticmethod(_bridge_mod.TcpPort.parse)

            def __init__(self, host, port, **kw):
                seen.update(host=host, port=port, **kw)

        monkeypatch.setattr(_bridge_mod, "TcpPort", _Tcp)
        made = self._scout_bridge(tmp_path, monkeypatch,
                                  "tcp://bridge.local:3333")
        assert made["defer"] is True
        made["open"]()
        assert (seen["host"], seen["port"], seen["key"]) == \
            ("bridge.local", 3333, "k")

    def test_scouting_a_usb_bridge_still_fails_loud(self, tmp_path,
                                                    monkeypatch):
        made = self._scout_bridge(tmp_path, monkeypatch,
                                  "/dev/serial/by-id/usb-Pico-if00")
        assert made["defer"] is False

    def test_a_bay_name_colliding_with_a_foreign_unit_is_refused(self, tmp_path):
        # The ams_names footgun: a bay named the same as an existing AFC unit
        # (here an OpenAMS whose lane4 says `unit: AMS_1`) double-registers that
        # unit's lanes and stops Klipper booting. Refuse it up front, clearly,
        # instead of letting the duplicate-lane crash happen.
        fc = _FileConfig({"AFC_lane lane4": {"unit": "AMS_1:1"}})
        with pytest.raises(Exception, match="collides with an existing AFC"):
            _mk_files(tmp_path, roster="", pool_ams=1, pool_ht=0,
                      ams_names="AMS_1", fileconfig=fc)

    def test_a_unique_bay_name_beside_a_foreign_unit_is_fine(self, tmp_path):
        # The neighbouring OpenAMS is fine as long as the bay name differs.
        fc = _FileConfig({"AFC_lane lane4": {"unit": "AMS_1:1"}})
        m, p, _a, _s = _mk_files(tmp_path, roster="", pool_ams=1, pool_ht=0,
                                 ams_names="AMS1_1", fileconfig=fc)
        assert any(sec.endswith("AMS1_1") for sec, _w in p.loaded
                   if sec.startswith("AFC_BambuAMS"))

    def test_a_pool_scout_fabricates_the_empty_named_pool(self, tmp_path):
        # No roster, but a pool IS configured: fabricate the empty pool so a
        # plugged unit claims a NAMED bay live (and pops the new-unit dialog),
        # no roster needed. This is the fully dynamic, roster-free mode.
        m, printer, _a, _s = _mk_files(tmp_path, roster="", pool_ams=2,
                                       pool_ht=1, ams_names="One, Two",
                                       ht_names="Hot")
        assert m.get_status()["roster_source"] == "scout"
        units = [sec.split()[1] for sec, _w in printer.loaded
                 if sec.startswith("AFC_BambuAMS")]
        assert units == ["One", "Two", "Hot"]        # all free, all named
        assert all(pu.get("uid") is None and pu.get("spare")
                   for pu in m._pool_units)          # nothing pre-claimed
        assert ("klippy:ready", m._scout_ready) in printer.handlers

    def test_a_roster_file_from_a_previous_scout_enrolls(self, tmp_path):
        (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(
            "# chain detected by [AFC_BridgeBox chain1]\n"
            "ht:0123456789ABCDEF00003331\n")
        m, printer, _a, _s = _mk_files(tmp_path, roster="")
        assert "AFC_BambuAMS Bambu_AMS_HT_1" in dict(printer.loaded)
        assert m.get_status()["roster_source"] == "file"

    def test_the_option_overrides_the_file(self, tmp_path):
        (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(
            "ht:AAAABBBBCCCCDDDD\n")
        m, printer, _a, _s = _mk_files(
            tmp_path, roster="boxed:1111222233334444")
        units = [sec.split()[1] for sec, _w in printer.loaded
                 if sec.startswith("AFC_BambuAMS")]
        assert units == ["Bambu_AMS_1"]
        assert m.get_status()["roster_source"] == "option"

    def test_enrolled_units_still_watch_for_newcomers(self, tmp_path):
        m, printer, _a, _s = _mk_files(tmp_path)
        assert ("klippy:ready", m._scout_ready) in printer.handlers

    def test_the_watch_registers_after_the_fabricated_units(self, tmp_path):
        # Ready handlers run in registration order, and the first UNIT to run
        # creates and owns the bridge (start, variant, status prime). A boot
        # where the scout registered first created the bridge itself and
        # demoted every unit to the ownerless "sharing" path: no pin, no
        # status, lanes empty -- a deaf unit on a live chain, seen live.
        m, printer, _a, _s = _mk_files(tmp_path)
        assert printer.loaded, "units were fabricated"
        assert printer.handlers[-1] == ("klippy:ready", m._scout_ready)

    def test_the_enrolled_watch_never_creates_a_bridge(self, tmp_path, monkeypatch):
        # It waits for the unit's bridge instead. _ensure_bridge from the
        # enrolled tick was the deaf-unit boot's mechanism.
        m, printer, _a, _s = _mk_files(tmp_path)
        called = []
        monkeypatch.setattr(m, "_ensure_bridge",
                            lambda: called.append(True))
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES", {}, raising=False)
        # Waits for the pool owner to bring the shared bridge up (polling fast),
        # and does NOT create one itself.
        assert m._scout_tick(100.0) == 103.0
        assert called == []


class TestChainToRoster:
    from extras.AFC_BridgeBox import afcBridgeBox as _M

    def test_htmask_names_the_models(self):
        # Firmware's own flag: bit per chain index.
        got = self._M._chain_to_roster(
            ["1111222233334444", "", "", "", "0123456789ABCDEF00003331"],
            htmask=0b10000)
        assert got == "boxed:1111222233334444, ht:0123456789ABCDEF00003331"

    def test_no_htmask_falls_back_to_enrollment_convention(self):
        # Boxed at 0..3, HTs at 4.. -- the addressing ranges enrollment uses.
        got = self._M._chain_to_roster(
            ["1111222233334444", "", "", "", "0123456789ABCDEF00003331"],
            htmask=0)
        assert got == "boxed:1111222233334444, ht:0123456789ABCDEF00003331"

    def test_empty_and_placeholder_positions_are_skipped(self):
        assert self._M._chain_to_roster(
            ["", "FFFFFFFFFFFFFFFFFFFFFFFF", ""], htmask=0) == ""

    def test_an_empty_chain_is_an_empty_roster(self):
        assert self._M._chain_to_roster([], htmask=0) == ""

    def test_the_output_is_valid_roster_syntax(self):
        # What the scout writes, _parse_roster must read back -- the file is
        # the same grammar as the option, by design, so it can be copied in.
        got = self._M._chain_to_roster(
            ["AAAA", "", "", "", "BBBB"], htmask=0b10000)
        parsed = self._M._parse_roster(got)
        assert parsed == [{"model": "boxed", "uid": "AAAA"},
                          {"model": "ht", "uid": "BBBB"}]


# ── the auto lane base: next available, remembered ───────────────────────────

class TestAutoLaneBase:
    def _fc(self):
        return _FileConfig({
            "AFC_stepper lane8": {}, "afc_stepper lane11": {},
            "AFC_lane lane4": {}, "AFC_lane lane7": {},
            "AFC_hub Turtle_1": {},               # not a lane
        })

    def test_default_is_one_past_the_highest_declared_lane(self, tmp_path):
        m, printer, _a, _s = _mk_files(tmp_path, lane_base=0,
                                       fileconfig=self._fc())
        assert m.lane_base == 12
        assert "AFC_lane lane12" in dict(printer.loaded)

    def test_the_computed_base_is_remembered(self, tmp_path):
        # "Next available" must not mean "renumbers when the config grows":
        # lane names carry Spoolman bindings and T# macros. Once computed,
        # the base persists in the store and later config growth cannot move
        # it.
        _mk_files(tmp_path, lane_base=0, fileconfig=self._fc())
        grown = _FileConfig({"AFC_stepper lane30": {}})
        m2, p2, _a, _s = _mk_files(tmp_path, lane_base=0, fileconfig=grown)
        assert m2.lane_base == 12             # stored, not recomputed to 31

    def test_an_explicit_option_still_wins(self, tmp_path):
        m, printer, _a, _s = _mk_files(tmp_path, lane_base=24,
                                       fileconfig=self._fc())
        assert m.lane_base == 24

    def test_toolchanger_extruders_set_the_base(self, tmp_path):
        # A U1-style toolchanger: tools e0..e3, standalone lanes named after
        # them, no laneN and no map: in the config. The pool continues the
        # tool numbers (lane4 = T4), not 24.
        fc = _FileConfig({"AFC_extruder e0": {}, "AFC_extruder e1": {},
                          "AFC_extruder e2": {}, "AFC_extruder e3": {}})
        m, printer, _a, _s = _mk_files(tmp_path, lane_base=0, fileconfig=fc)
        assert m.lane_base == 4

    def test_a_single_unnumbered_extruder_keeps_the_fallback(self, tmp_path):
        fc = _FileConfig({"AFC_extruder extruder": {}})
        m, printer, _a, _s = _mk_files(tmp_path, lane_base=0, fileconfig=fc)
        assert m.lane_base == 24

    def test_no_visible_config_falls_back_to_24(self, tmp_path):
        m, printer, _a, _s = _mk_files(tmp_path, lane_base=0)
        assert m.lane_base == 24


# ── the chain's buffer: fabricated with the master, shared by its units ──────

class TestFabricatedBuffer:
    def test_no_buffer_option_fabricates_one_for_the_chain(self, tmp_path):
        m, printer, _a, _s = _mk_files(tmp_path, buffer=None)
        sections = dict(printer.loaded)
        assert "AFC_buffer Bambu_AMS_Buffer" in sections
        b = dict(m._roster_sections(m.units))["AFC_buffer Bambu_AMS_Buffer"]
        assert b["type"] == "FPS_PSF"
        assert b["adc_pin"] == "bambu_buffer:fps"
        # The two values tuned on hardware: the AMS buffer's swing, and jam
        # detection off because the AMS meters its own moves.
        assert b["deadband"] == 0.48
        assert b["filament_error_sensitivity"] == 0

    def test_every_unit_on_the_chain_references_it(self, tmp_path):
        m, printer, _a, _s = _mk_files(
            tmp_path, buffer=None, roster="ht:AAAA, boxed:BBBB")
        sections = dict(m._roster_sections(m.units))
        assert sections["AFC_BambuAMS Bambu_AMS_HT_1"][
            "buffer"] == "Bambu_AMS_Buffer"
        assert sections["AFC_BambuAMS Bambu_AMS_1"][
            "buffer"] == "Bambu_AMS_Buffer"

    def test_the_buffer_loads_after_the_units(self, tmp_path):
        # Its adc_pin references the virtual chip a UNIT registers at load;
        # a buffer loading first is the "Unknown pin chip name" halt.
        m, printer, _a, _s = _mk_files(tmp_path, buffer=None)
        names = [sec for sec, _w in printer.loaded]
        assert names.index("AFC_buffer Bambu_AMS_Buffer") \
            > names.index("AFC_BambuAMS Bambu_AMS_HT_1")

    def test_a_named_external_buffer_suppresses_fabrication(self, tmp_path):
        m, printer, _a, _s = _mk_files(tmp_path, buffer="Bamb_1")
        assert not any(sec.startswith("AFC_buffer")
                       for sec, _w in printer.loaded)
        u = dict(m._roster_sections(m.units))["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u["buffer"] == "Bamb_1"

    def test_buffer_type_defaults_to_the_plain_tension_follower(self,
                                                                tmp_path):
        # The odometer gate is opt-in. A config that says nothing keeps the
        # buffer it has always had, so an existing printer does not silently
        # acquire a new condition on buffer_triggered.
        m, _printer, _a, _s = _mk_files(tmp_path, buffer=None)
        b = dict(m._roster_sections(m.units))["AFC_buffer Bambu_AMS_Buffer"]
        assert b["type"] == "FPS_PSF"

    def test_buffer_type_bambu_fabricates_the_gated_buffer(self, tmp_path):
        # This is the whole opt-in: one line on the chain master turns the
        # fabricated buffer into the one that also watches the odometer, so
        # it can stand in for a toolhead sensor.
        m, _printer, _a, _s = _mk_files(tmp_path, buffer=None,
                                        buffer_type="bambu")
        b = dict(m._roster_sections(m.units))["AFC_buffer Bambu_AMS_Buffer"]
        assert b["type"] == "bambu"
        # Everything else about the section is unchanged -- same pin, same
        # hardware-tuned deadband, jam detection still off.
        assert b["adc_pin"] == "bambu_buffer:fps"
        assert b["deadband"] == 0.48
        assert b["filament_error_sensitivity"] == 0

    def test_buffer_type_never_reaches_a_unit_section(self, tmp_path):
        # It is the master's own option. Broadcast as a chain default it
        # would land on [AFC_BambuAMS ...], which knows no such option.
        m, _printer, _a, _s = _mk_files(tmp_path, buffer=None,
                                        buffer_type="bambu")
        u = dict(m._roster_sections(m.units))["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert "buffer_type" not in u

    def test_an_adopted_buffer_keeps_its_own_type(self, tmp_path):
        # A hand-written section names its own type; the master's option has
        # nothing to fabricate and must not rewrite what the operator wrote.
        fc = _FileConfig({"AFC_buffer Bamb_1": {"adc_pin": "bambu_buffer:fps",
                                                "type": "FPS_PSF"}})
        m, printer, _a, _s = _mk_files(tmp_path, buffer=None,
                                       buffer_type="bambu", fileconfig=fc)
        assert not any(sec.startswith("AFC_buffer")
                       for sec, _w in printer.loaded)

    def test_a_second_chain_gets_its_own_buffer_name(self, tmp_path):
        m, printer, _a, _s = _mk_files(
            tmp_path, buffer=None, unit_prefix="Bambu_AMS_B",
            buffer_chip_name="bambu_buffer_b")
        b = dict(m._roster_sections(m.units))[
            "AFC_buffer Bambu_AMS_B_Buffer"]
        assert b["adc_pin"] == "bambu_buffer_b:fps"


class TestBufferAdoption:
    def test_an_existing_chip_buffer_is_adopted_not_rivalled(self, tmp_path):
        # The halt this earns its keep against, seen live: a hand-written
        # [AFC_buffer Bamb_1] on bambu_buffer:fps plus a fabricated buffer on
        # the same pin is "pin fps used multiple times in config" -- klippy
        # down. Same chip = same physical buffer; wire the units to it.
        fc = _FileConfig({"AFC_buffer Bamb_1": {"adc_pin": "bambu_buffer:fps",
                                                "type": "FPS_PSF"}})
        m, printer, _a, _s = _mk_files(tmp_path, buffer=None, fileconfig=fc)
        assert not any(sec.startswith("AFC_buffer")
                       for sec, _w in printer.loaded)
        u = dict(m._roster_sections(m.units))["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u["buffer"] == "Bamb_1"

    def test_a_buffer_on_another_chip_is_not_adopted(self, tmp_path):
        # fps_buffer1 reads a real MCU pin; it belongs to a different
        # extruder entirely and must not be claimed for this chain.
        fc = _FileConfig({"AFC_buffer fps_buffer1": {"adc_pin": "fps:PA2"}})
        m, printer, _a, _s = _mk_files(tmp_path, buffer=None, fileconfig=fc)
        assert "AFC_buffer Bambu_AMS_Buffer" in dict(printer.loaded)


# ── the state follows the section's own file ─────────────────────────────────

class TestStateFileLocation:
    def test_state_lands_in_the_file_declaring_the_section(self, tmp_path):
        # Name the file anything: the module greps the config tree for its
        # own [AFC_BridgeBox <name>] header and appends its managed block
        # there. No hardcoded filename to know about.
        cfgdir = tmp_path / "config"
        (cfgdir / "AFC").mkdir(parents=True)
        (cfgdir / "printer.cfg").write_text("[include AFC/*.cfg]\n")
        my = cfgdir / "AFC" / "whatever_i_called_it.cfg"
        my.write_text("[AFC_BridgeBox chain1]\n"
                      "serial_port: /dev/x\nextruder: extruder\n")
        autov = tmp_path / "AFC_auto_vars.cfg"
        printer = _Printer()
        printer.start_args = {"config_file": str(cfgdir / "printer.cfg")}
        opts = {"serial_port": "/dev/x", "extruder": "extruder",
                "lane_base": 24, "roster": "ht:AAAA", "buffer": "Bamb_1",
                "auto_vars_file": str(autov)}
        m = load_config_prefix(_Config(opts, printer))
        assert m.state_file == str(my)
        # ...and a state write preserves the operator's section above it.
        m._state_set({"AFC_BridgeBox chain1": {"roster": "ht:AAAA"}})
        text = my.read_text()
        assert text.index("[AFC_BridgeBox chain1]") \
            < text.index(m._STATE_MARK)
        assert "serial_port: /dev/x" in text
        assert m._state_get("AFC_BridgeBox chain1", "roster") == "ht:AAAA"

    def test_unfindable_section_falls_back_to_the_default_path(self, tmp_path):
        # Scout mode (empty roster) so nothing fabricates -- fabrication
        # would flush the lane map to the real fallback path this asserts.
        printer = _Printer()          # no start_args config_file
        opts = {"serial_port": "/dev/x", "extruder": "extruder",
                "lane_base": 24, "roster": "", "buffer": "Bamb_1",
                "auto_vars_file": str(tmp_path / "av.cfg")}
        m = load_config_prefix(_Config(opts, printer))
        assert m.state_file.endswith("AFC_BridgeBox.cfg")

    def test_an_explicit_state_file_still_wins(self, tmp_path):
        printer = _Printer()
        printer.start_args = {"config_file": str(tmp_path / "printer.cfg")}
        opts = {"serial_port": "/dev/x", "extruder": "extruder",
                "lane_base": 24, "roster": "ht:AAAA", "buffer": "Bamb_1",
                "auto_vars_file": str(tmp_path / "av.cfg"),
                "state_file": str(tmp_path / "pinned.cfg")}
        m = load_config_prefix(_Config(opts, printer))
        assert m.state_file == str(tmp_path / "pinned.cfg")


# ── operator overrides: the per-unit knob on fabricated sections ─────────────

class TestOperatorOverrides:
    def test_a_section_without_serial_port_is_an_override_not_a_master(self):
        # The discriminator: every chain master requires serial_port; a
        # section without one is an override carrier and must not demand
        # serial_port/extruder or fabricate anything.
        from extras.AFC_BridgeBox import BridgeBoxOverrideHolder
        printer = _Printer()
        h = load_config_prefix(_Config(
            {"measure_on_insert": "True"}, printer,
            name="AFC_BridgeBox Bambu_AMS_1"))
        assert isinstance(h, BridgeBoxOverrideHolder)
        assert printer.loaded == []

    def test_a_second_master_is_never_read_as_an_override(self, tmp_path):
        # Another chain's [AFC_BridgeBox] carries serial_port -- its options
        # must not overlay this chain's units.
        fc = _FileConfig({"AFC_BridgeBox chain2":
                          {"serial_port": "/dev/other",
                           "unit_prefix": "Bambu_AMS_B"}})
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc)
        u = self._fold(m, printer, fc)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert "unit_prefix" not in u

    def _fold(self, m, printer, fc=None):
        cfg = _Config({}, printer)
        if fc is not None:
            cfg.fileconfig = fc
        return dict(m._fold_and_sweep(cfg, m._roster_sections(m.units)))

    def test_a_bare_name_overrides_that_units_section(self, tmp_path):
        # THE question this answers: with every section fabricated, where
        # does the operator set measure_on_insert for one unit?
        fc = _FileConfig({"AFC_BridgeBox Bambu_AMS_HT_1":
                          {"measure_on_insert": "False"}})
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc)
        u = self._fold(m, printer, fc)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u["measure_on_insert"] == "False"

    def test_a_qualified_name_reaches_hubs_and_lanes(self, tmp_path):
        fc = _FileConfig({
            "AFC_BridgeBox AFC_hub Bambu_AMS_HT_1":
                {"afc_bowden_length": "1800"},
            "AFC_BridgeBox AFC_lane lane24":
                {"dist_hub": "120"}})
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc)
        folded = self._fold(m, printer, fc)
        assert folded["AFC_hub Bambu_AMS_HT_1"]["afc_bowden_length"] == "1800"
        assert folded["AFC_lane lane24"]["dist_hub"] == "120"

    def test_a_model_tag_covers_every_unit_of_that_model(self, tmp_path):
        # Per-TYPE policy: one block for all HTs, one for all AMS 2 Pros.
        fc = _FileConfig({
            "AFC_BridgeBox ht": {"measure_on_insert": "False"},
            "AFC_BridgeBox ams2": {"dry_max_temp": "60"}})
        m, printer, _a, _s = _mk_files(
            tmp_path, roster="ht:AAAA, ht:BBBB, ams2:CCCC", fileconfig=fc)
        folded = self._fold(m, printer, fc)
        for name in ("Bambu_AMS_HT_1", "Bambu_AMS_HT_2"):
            assert folded[f"AFC_BambuAMS {name}"][
                "measure_on_insert"] == "False"
        # The per-MODEL block matches by model, not name: the ams2 unit (now
        # named Bambu_AMS_1, no generation in the name) still gets it.
        assert folded["AFC_BambuAMS Bambu_AMS_1"]["dry_max_temp"] == "60"
        # ...and the model tag never leaks across models: the ams2 keeps
        # its fabricated default (False -- measure is HT-proven only).
        assert folded["AFC_BambuAMS Bambu_AMS_1"][
            "measure_on_insert"] is False

    def test_the_unit_exception_beats_the_model_policy(self, tmp_path):
        fc = _FileConfig({
            "AFC_BridgeBox ht": {"measure_on_insert": "False"},
            "AFC_BridgeBox Bambu_AMS_HT_2":
                {"measure_on_insert": "True"}})
        m, printer, _a, _s = _mk_files(
            tmp_path, roster="ht:AAAA, ht:BBBB", fileconfig=fc)
        folded = self._fold(m, printer, fc)
        assert folded["AFC_BambuAMS Bambu_AMS_HT_1"][
            "measure_on_insert"] == "False"
        assert folded["AFC_BambuAMS Bambu_AMS_HT_2"][
            "measure_on_insert"] == "True"

    def test_identity_keys_are_protected(self, tmp_path):
        # unit_uid is filtered by the structural-key rule; serial_port needs
        # no filter at all -- carrying one makes the section a MASTER, so it
        # is never read as an override in the first place (see the
        # second-master test above).
        fc = _FileConfig({"AFC_BridgeBox Bambu_AMS_HT_1":
                          {"unit_uid": "FFFF",
                           "measure_on_insert": "False"}})
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc)
        u = self._fold(m, printer, fc)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u["unit_uid"] == "0123456789ABCDEF00003331"
        assert u["measure_on_insert"] == "False"

    def test_the_operator_outranks_learned_values(self, tmp_path):
        # A learned bowden length in the state must lose to an explicit
        # override -- history yields to intent.
        autov = ("[AFC_hub Bambu_AMS_HT_1]\n"
                 "afc_bowden_length : 3632.0\n")
        fc = _FileConfig({"AFC_BridgeBox AFC_hub Bambu_AMS_HT_1":
                          {"afc_bowden_length": "2000"}})
        m, printer, _a, _s = _mk_files(tmp_path, autov_text=autov,
                                       fileconfig=fc)
        folded = self._fold(m, printer, fc)
        assert folded["AFC_hub Bambu_AMS_HT_1"]["afc_bowden_length"] == "2000"

    def test_overrides_are_never_laundered_into_learned_state(self, tmp_path):
        # Deleting the override section must mean the override is GONE. If
        # the fold persisted it as a learned value, it would come back from
        # the state block forever.
        fc = _FileConfig({"AFC_BridgeBox Bambu_AMS_HT_1":
                          {"measure_on_insert": "False"}})
        _mk_files(tmp_path, fileconfig=fc)
        m2, p2, _a, _s = _mk_files(tmp_path)          # override removed
        u = self._fold(m2, p2)["AFC_BambuAMS Bambu_AMS_HT_1"]
        assert u["measure_on_insert"] is True

    def test_an_orphan_override_is_ignored_not_fatal(self, tmp_path):
        # A removed unit's leftover override must not halt the boot.
        fc = _FileConfig({"AFC_BridgeBox Bambu_AMS_9":
                          {"measure_on_insert": "False"}})
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc)
        assert "AFC_BambuAMS Bambu_AMS_HT_1" in dict(printer.loaded)


# ── lanes and names pin to the uid, tombstoned forever ───────────────────────

class TestLaneAndNameTombstones:
    def test_removing_a_unit_never_renumbers_or_renames_survivors(
            self, tmp_path):
        # Lane names carry Spoolman bindings and T# macros, and a unit's name
        # fixes its lanes. Roster-order allocation alone would hand BBBB
        # lane24 and the name Bambu_AMS_1 the moment AAAA left -- along with
        # AAAA's Spoolman bindings.
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        m2, p2, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB")
        sections = dict(m2._roster_sections(m2.units))
        assert "AFC_BambuAMS Bambu_AMS_2" in sections
        assert sections["AFC_lane lane28"] == {"unit": "Bambu_AMS_2:1",
                                               "unassigned": True}
        assert "AFC_lane lane24" not in sections    # tombstoned, not reused

    def test_a_returning_unit_gets_its_lanes_and_name_back(self, tmp_path):
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        _mk_files(tmp_path, roster="boxed:BBBB")
        # ...and it returns at the END of the roster, order be damned.
        m3, p3, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB, boxed:AAAA")
        sections = dict(m3._roster_sections(m3.units))
        assert sections["AFC_lane lane24"] == {"unit": "Bambu_AMS_1:1",
                                               "unassigned": True}
        assert sections["AFC_BambuAMS Bambu_AMS_1"]["unit_uid"] == "AAAA"

    def test_a_new_unit_allocates_past_every_tombstone(self, tmp_path):
        # A NEW unit wearing a removed unit's name would take over its lanes,
        # and with them its Spoolman bindings and T#.
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        m2, p2, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB, boxed:CCCC")
        sections = dict(m2._roster_sections(m2.units))
        u = sections["AFC_BambuAMS Bambu_AMS_3"]
        assert u["unit_uid"] == "CCCC"
        assert sections["AFC_lane lane32"] == {"unit": "Bambu_AMS_3:1",
                                               "unassigned": True}

    def test_families_number_independently_through_the_map(self, tmp_path):
        # Names are POSITIONAL: an AMS bay is numbered by its slot in the low
        # band, an HT bay by its slot in the high band. The HT family's names
        # nest inside the boxed family's prefix (Bambu_AMS_HT_1 startswith
        # Bambu_AMS_), and each band still counts only its own. Sized so the
        # ht_base is stable as a second boxed unit is added (pool_ams covers
        # both), the way a real config with headroom behaves.
        m, p, _a, _s = _mk_files(tmp_path, roster="ht:AAAA, boxed:BBBB",
                                 pool_ams=2, pool_ht=1)
        m2, p2, _a2, _s2 = _mk_files(tmp_path,
                                     roster="ht:AAAA, boxed:BBBB, boxed:CCCC",
                                     pool_ams=2, pool_ht=1)
        names = [s.split()[1] for s, _w in p2.loaded
                 if s.startswith("AFC_BambuAMS")]
        assert sorted(names) == ["Bambu_AMS_1", "Bambu_AMS_2",
                                 "Bambu_AMS_HT_1"]

    @pytest.mark.parametrize("bad", ["AAAA:24:400", "AAAA:24:2",
                                     "AAAA:-4:4"])
    def test_a_lane_map_entry_no_unit_could_have_is_dropped(self, tmp_path,
                                                            bad):
        # A hand-edited span would otherwise have every lane it covers
        # walked at boot (and a typo like 40000000 exhausts a Pi's memory).
        m0 = _mk_files(tmp_path, roster="")[0]
        m0._state_set({"AFC_BridgeBox chain1": {
            "lane_map": f"{bad}, HHHH:40:1",
            "name_map": "AAAA:Bambu_AMS_1, HHHH:Bambu_AMS_HT_1"}})
        m, _p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA, ht:HHHH",
                                  pool_ams=4, pool_ht=2)
        assert m._lane_map["AAAA"] == (24, 4)
        assert m._lane_map["HHHH"] == (40, 1)
        assert m._layout_notes == []

    def test_the_maps_live_in_the_managed_comment_block(self, tmp_path):
        _mk_files(tmp_path, roster="ht:AAAA")
        text = (tmp_path / "AFC_BridgeBox.cfg").read_text()
        for key in ("lane_map", "name_map"):
            line = next(ln for ln in text.splitlines() if key in ln)
            assert line.startswith("#~#")
        m2, _p, _a, _s = _mk_files(tmp_path, roster="ht:AAAA")
        assert m2._lane_map == {"AAAA": (24, 1)}
        assert m2._name_map == {"AAAA": "Bambu_AMS_HT_1"}


# ── removal: debounced, evidence-driven, applied at restart ──────────────────

class _Bridge:
    """The chain as _prune_missing consults it: an enrollment map that can
    go stale, per-unit online flags that cannot."""

    def __init__(self, uids, online, htmask=0, link_up=True,
                 a2mask=0, a2asks=None):
        self._uids = list(uids)
        self._online = list(online)
        self._htmask = htmask
        self._serial = object() if link_up else None
        self._a2mask = a2mask
        self._a2asks = list(a2asks or [])
        self.sent = []

    def chain_dialect(self):
        return (self._a2mask, list(self._a2asks))

    def send(self, obj):
        self.sent.append(obj)

    def chain_uids(self):
        return list(self._uids)

    def chain_diag(self):
        return (self._htmask, "", (-1, 0, 0))

    def latest_status(self):
        return {"units": [{"n": i, "online": b}
                          for i, b in enumerate(self._online)]}


class _Logger:
    def __init__(self):
        self.lines = []             # what reaches the console
        self.debugs = []            # AFC.log only (console with debug on)

    def info(self, msg):
        self.lines.append(msg)

    warning = error = info

    def debug(self, msg, only_debug=False, traceback=None):
        self.debugs.append(msg)


class TestHotPlugPollInterval:
    """How fast the HOST asks who is on the chain.

    The firmware sees a plug in about a second -- it enrolls off the announce
    -- so the interval below is most of the hot-plug delay an operator feels.
    It used to climb a ladder that was slowest (5s bound, 30s quiet) in
    exactly the state a hot-plug arrives in, and the ladder lived inside the
    pool branch, so a roster-only chain never reached it and polled at a flat
    30s forever.
    """

    SEC = "AFC_BridgeBox chain1"

    def _watching(self, tmp_path, monkeypatch, bridge, **over):
        m, printer, _a, _s = _mk_files(tmp_path, **over)
        m.logger = _Logger()
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        return m

    def test_a_roster_only_chain_does_not_poll_at_thirty_seconds(
            self, tmp_path, monkeypatch):
        # THE REGRESSION THIS PINS. _watch_next was assigned inside
        # `if self.pool_ams or self.pool_ht:`, so a chain with no pool fell
        # through to the getattr default and asked once every 30s -- the
        # difference between noticing a plug in a second and in half a minute.
        bridge = _Bridge(uids=["0123456789ABCDEF00003331"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        assert not (m.pool_ams or m.pool_ht), "no pool: the uncovered path"
        assert m._scout_tick(100.0) - 100.0 == m.hotplug_poll

    def test_a_settled_pooled_chain_does_not_fall_back_to_a_heartbeat(
            self, tmp_path, monkeypatch):
        # The old ladder relaxed to a 5s heartbeat once every bound unit was
        # online -- which is precisely the quiet state a new unit is plugged
        # into, so the feature paid its worst latency at the only moment it
        # mattered.
        bridge = _Bridge(uids=["0123456789ABCDEF00003331"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge,
                           pool_ams=1, pool_ht=1)
        assert m._scout_tick(100.0) - 100.0 == m.hotplug_poll

    def test_the_default_is_about_a_second(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=[], online=[])
        m = self._watching(tmp_path, monkeypatch, bridge)
        assert m.hotplug_poll == 1.0

    def test_an_operator_can_slow_it_down(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=[], online=[])
        m = self._watching(tmp_path, monkeypatch, bridge, hotplug_poll=4.0)
        assert m._scout_tick(100.0) == 104.0

    def test_the_chain_query_never_reaches_the_bus(
            self, tmp_path, monkeypatch):
        # WHY POLLING FAST IS AFFORDABLE, held as a test because it is the
        # whole justification. The bridge answers {"cmd":"chain"} from its
        # cached state and puts no frame on the RS-485 bus. If the tick ever
        # starts issuing something that does move the bus, this interval has
        # to be reconsidered, and this is where that shows up.
        bridge = _Bridge(uids=["0123456789ABCDEF00003331"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        m._scout_tick(100.0)
        assert bridge.sent, "the tick did ask"
        assert {c.get("cmd") for c in bridge.sent} <= {"chain"}


class TestRemovalDebounce:
    SEC = "AFC_BridgeBox chain1"

    def _watching(self, tmp_path, monkeypatch, bridge,
                  roster="boxed:AAAA, boxed:BBBB", **over):
        # The recorded roster, which removal edits: a roster: option is the
        # roster a restart builds from, so nothing is removed with one.
        m, printer, _a, _s = _mk_recorded(tmp_path, roster, **over)
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": roster}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        return m

    def _roster(self, m):
        return m._state_get(self.SEC, "roster")

    def test_a_unit_absent_past_the_grace_is_recorded_removed(
            self, tmp_path, monkeypatch):
        # AAAA gone, BBBB online at index 0 -- the chain is provably alive,
        # so absence means absence. Removal edits only the RECORD; this
        # session's sections stand until restart, like every roster change.
        bridge = _Bridge(uids=["BBBB"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        m._scout_tick(100.0)
        assert self._roster(m) == "boxed:AAAA, boxed:BBBB"   # clock started
        m._scout_tick(180.0)
        assert self._roster(m) == "boxed:AAAA, boxed:BBBB"   # still in grace
        m._scout_tick(300.0)
        assert self._roster(m) == "boxed:BBBB"
        assert any("removed from the recorded roster" in ln
                   for ln in m.logger.lines)

    def test_the_removal_line_names_the_chain_in_its_forget_hint(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["BBBB"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        m._scout_tick(100.0)
        m._scout_tick(300.0)
        (line,) = [ln for ln in m.logger.lines
                   if "removed from the recorded roster" in ln]
        assert line.endswith(
            "If one is never coming back, AFC_BRIDGEBOX_FORGET CHAIN=chain1 "
            "UID=<uid> frees its lanes and name for reuse.")

    def test_the_removal_line_promises_no_bay_a_waiting_ams_takes(
            self, tmp_path, monkeypatch):
        # Four AMS and E recorded, no pool: E waits with no bay. D stays
        # offline for removal_grace, and the next restart gives E its bay.
        four = "boxed:AAAA, boxed:BBBB, boxed:CCCC, boxed:DDDD"
        bridge = _Bridge(uids=["AAAA", "BBBB", "CCCC", "DDDD", "EEEE"],
                         online=[True, True, True, False, True])
        m = self._watching(tmp_path, monkeypatch, bridge,
                           roster=four + ", boxed:EEEE")
        assert m._unbayed == {"EEEE": None}
        m._scout_tick(100.0)
        m._scout_tick(300.0)
        (line,) = [ln for ln in m.logger.lines
                   if "removed from the recorded roster" in ln]
        assert "restores it unchanged" not in line
        assert ("Their learned values are kept. Plugged back in before "
                "then, one keeps its lanes and name; after the restart they "
                "stay reserved for its return unless a new AMS would "
                "otherwise have no bay or push the HT lanes up (EEEE is "
                "waiting for a bay and takes one).") in line
        m2 = _mk_files(tmp_path, roster="")[0]
        assert m2._name_map.get("EEEE") == "Bambu_AMS_4"

    def test_a_roster_option_is_not_pruned_or_reported_pending(
            self, tmp_path, monkeypatch):
        # The option is the roster a restart builds from.
        bridge = _Bridge(uids=["BBBB", "X9X9"], online=[True, True])
        m, _p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": "boxed:AAAA, boxed:BBBB"}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        for t in (100.0, 116.0, 300.0):
            m._scout_tick(t)
        assert "boxed:AAAA" in self._roster(m)
        assert not any("removed from the recorded roster" in ln
                       for ln in m.logger.lines)
        assert m.get_status()["pending_restart"] == []

    def test_a_sticky_offline_unit_is_not_enrolled(
            self, tmp_path, monkeypatch):
        # A pulled unit lingers in the firmware's chain_uids with its online
        # flag false. Enrolling straight off chain_uids would re-add that ghost
        # the tick after the operator cleared it -- refilling an emptied roster
        # on the next boot. Only a unit that is actually ONLINE enrolls.
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m = self._watching(tmp_path, monkeypatch, bridge, roster="")
        m._scout_tick(100.0)
        assert (self._roster(m) or "") == ""            # ghost NOT enrolled
        bridge._online = [True]                          # it truly comes online
        m._scout_tick(105.0)                             # hold starts
        assert "AAAA" not in (self._roster(m) or "")     # not on a single blip
        m._scout_tick(121.0)                             # held past enroll_grace
        assert "AAAA" in (self._roster(m) or "")         # now it enrolls

    def test_a_blip_online_does_not_enroll_a_forgotten_ghost(
            self, tmp_path, monkeypatch):
        # THE PHANTOM CASE. A pulled unit whose bridge flag flaps online for a
        # tick here and there must NEVER be re-recorded -- that is what undid a
        # FORGET and brought the unit back on the next restart. It never holds
        # online continuously, so it never enrolls.
        bridge = _Bridge(uids=["AAAA"], online=[False])
        m = self._watching(tmp_path, monkeypatch, bridge, roster="")
        for i, on in enumerate([True, False, True, False, True, False, True]):
            bridge._online = [on]
            m._scout_tick(100.0 + i * 5.0)               # 35s of flapping
        assert (self._roster(m) or "") == ""             # never enrolled

    def test_a_live_forget_is_not_rescouted(self, tmp_path, monkeypatch):
        # A uid FORGOTTEN while still on the wire is parked on the suppress list.
        # The scout must not write it straight back into the roster it just left.
        bridge = _Bridge(uids=["AAAA", "BBBB"], online=[True, True])
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:BBBB")
        m._forget_suppressed = {"AAAA"}
        m._scout_tick(100.0)
        assert self._roster(m) == "boxed:BBBB"           # AAAA NOT re-enrolled
        assert "AAAA" in m._forget_suppressed             # hold still stands

    def test_pulling_a_suppressed_unit_lifts_the_hold(
            self, tmp_path, monkeypatch):
        # The hold lasts only while the hardware stays online. Pull it (it drops
        # out of the online set) and the hold clears, so a genuine re-plug
        # enrolls it fresh -- exactly the "add it back" path.
        bridge = _Bridge(uids=["AAAA", "BBBB"], online=[False, True])
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:BBBB")
        m._forget_suppressed = {"AAAA"}
        m._scout_tick(100.0)                              # AAAA offline -> clear
        assert "AAAA" not in m._forget_suppressed
        bridge._online = [True, True]                     # re-plugged
        m._scout_tick(105.0)                              # hold starts
        m._scout_tick(121.0)                              # held past enroll_grace
        assert "AAAA" in (self._roster(m) or "")          # enrolls fresh

    def test_the_countdown_announces_itself_when_it_starts(
            self, tmp_path, monkeypatch):
        # An operator testing removal watched a silent console for minutes
        # with no way to tell a running countdown from a broken one.
        bridge = _Bridge(uids=["BBBB"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        m._scout_tick(100.0)
        assert any("is offline on a live chain" in ln
                   for ln in m.logger.lines)
        m._scout_tick(110.0)                  # ...but it announces ONCE
        assert sum("is offline on a live chain" in ln
                   for ln in m.logger.lines) == 1

    def test_reappearing_resets_the_clock(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["BBBB"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        m._scout_tick(100.0)
        bridge._uids = ["BBBB", "AAAA"]       # a blip: AAAA answers again
        bridge._online = [True, True]
        m._scout_tick(400.0)
        bridge._uids = ["BBBB"]
        bridge._online = [True]
        m._scout_tick(800.0)                  # 400s absent, not 700
        assert self._roster(m) == "boxed:AAAA, boxed:BBBB"

    def test_a_dead_link_proves_nothing(self, tmp_path, monkeypatch):
        # Serial gone = the Pico rebooted or the USB flapped. Every absence
        # clock resets; only absence from a LIVE chain counts.
        bridge = _Bridge(uids=["BBBB"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        m._scout_tick(100.0)
        bridge._serial = None
        m._scout_tick(200.0)
        bridge._serial = object()
        m._scout_tick(900.0)                  # clock restarted here
        assert self._roster(m) == "boxed:AAAA, boxed:BBBB"

    def test_a_dark_chain_removes_nothing(self, tmp_path, monkeypatch):
        # ALL units offline reads as "the chain is off", not "the operator
        # removed everything" -- the two are indistinguishable, so nothing
        # is ever removed on that evidence. Same reason the sole unit of a
        # single-unit chain is never auto-removed.
        bridge = _Bridge(uids=["BBBB"], online=[False])
        m = self._watching(tmp_path, monkeypatch, bridge)
        for t in (100.0, 800.0, 1500.0):
            m._scout_tick(t)
        assert self._roster(m) == "boxed:AAAA, boxed:BBBB"

    def test_grace_zero_disables_auto_removal(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["BBBB"], online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge, removal_grace=0)
        for t in (100.0, 100000.0):
            m._scout_tick(t)
        assert self._roster(m) == "boxed:AAAA, boxed:BBBB"

    def test_additions_merge_instead_of_replacing(
            self, tmp_path, monkeypatch):
        # The wholesale `roster = seen` this replaced would have dropped
        # AAAA the very tick CCCC appeared without it.
        bridge = _Bridge(uids=["BBBB", "CCCC"], online=[True, True])
        m = self._watching(tmp_path, monkeypatch, bridge)
        m._scout_tick(100.0)                              # hold starts for CCCC
        m._scout_tick(116.0)                              # held past enroll_grace
        assert self._roster(m) == "boxed:AAAA, boxed:BBBB, boxed:CCCC"


# ── a confirmed generation renames into its family, calibration intact ───────

class TestGenerationRename:
    def test_a_refined_unit_keeps_its_name_no_rename(self, tmp_path):
        # The name carries NO generation, so a boxed unit confirming as ams2
        # does NOT rename -- it stays Bambu_AMS_1 and its learned values stay
        # put. That is what avoids a live rename (which Klipper forbids) and
        # lets a first-plugged AMS keep its name with no reboot.
        _mk_files(tmp_path, roster="boxed:AAAA")
        m1, _p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA")
        m1._state_set({"AFC_hub Bambu_AMS_1":
                       {"afc_bowden_length": "3632.0"}})
        m2, p2, _a, _s = _mk_files(tmp_path, roster="ams2:AAAA")
        sections = dict(m2._roster_sections(m2.units))
        assert "AFC_BambuAMS Bambu_AMS_1" in sections           # no rename
        assert "AFC_BambuAMS Bambu_AMS2_1" not in sections
        assert sections["AFC_lane lane24"] == {"unit": "Bambu_AMS_1:1",
                                               "unassigned": True}
        assert m2._state_get("AFC_hub Bambu_AMS_1",
                             "afc_bowden_length") == "3632.0"

    def test_all_ams_share_one_family_ht_separate(self, tmp_path):
        # Every 4-lane unit -- ams2, ams1, boxed -- counts in ONE Bambu_AMS_#
        # family; only the HT is separate. (Fabricated ams-first, then ht.)
        m, printer, _a, _s = _mk_files(
            tmp_path, roster="ams2:AAAA, ams1:BBBB, boxed:CCCC, ht:DDDD")
        names = [sec.split()[1] for sec, _w in printer.loaded
                 if sec.startswith("AFC_BambuAMS")]
        assert names == ["Bambu_AMS_1", "Bambu_AMS_2", "Bambu_AMS_3",
                         "Bambu_AMS_HT_1"]


# ── the chain confirms a boxed unit's generation itself ──────────────────────

class TestDialectRefinement:
    SEC = "AFC_BridgeBox chain1"

    def _watching(self, tmp_path, monkeypatch, bridge, roster):
        m, printer, _a, _s = _mk_files(tmp_path)
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": roster}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        return m

    def test_an_answered_3702_confirms_ams2(self, tmp_path, monkeypatch):
        # `boxed` was only ever "not confirmed YET" -- one answered version
        # query is the wire's own word that this is an AMS 2.
        bridge = _Bridge(uids=["AAAA"], online=[True], a2mask=0b1,
                         a2asks=[5])
        m = self._watching(tmp_path, monkeypatch, bridge, "boxed:AAAA")
        m._scout_tick(100.0)
        assert m._state_get(self.SEC, "roster") == "ams2:AAAA"
        assert any("confirmed by bus dialect" in ln for ln in m.logger.lines)

    def test_hundreds_of_silent_asks_confirm_ams1(self, tmp_path,
                                                  monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[True], a2mask=0,
                         a2asks=[301])
        m = self._watching(tmp_path, monkeypatch, bridge, "boxed:AAAA")
        m._scout_tick(100.0)
        assert m._state_get(self.SEC, "roster") == "ams1:AAAA"

    def test_silence_from_an_offline_unit_proves_nothing(self, tmp_path,
                                                         monkeypatch):
        # An unplugged unit answers nothing -- that is absence, not dialect.
        bridge = _Bridge(uids=["AAAA", "BBBB"], online=[False, True],
                         a2mask=0, a2asks=[999, 10])
        m = self._watching(tmp_path, monkeypatch, bridge,
                           "boxed:AAAA, boxed:BBBB")
        m._scout_tick(100.0)
        assert m._state_get(self.SEC, "roster") == "boxed:AAAA, boxed:BBBB"

    def test_a_refinement_applies_to_the_running_unit_live(self, tmp_path,
                                                           monkeypatch):
        # Additions and removals change klippy's object graph and wait for
        # the restart; a refinement is a label and a heater flag on objects
        # that already exist, so it lands immediately.
        import types
        bridge = _Bridge(uids=["AAAA"], online=[True], a2mask=0b1,
                         a2asks=[5])
        m, printer, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA")
        fake_unit = types.SimpleNamespace(unit_uid="AAAA",
                                          ams_model="boxed",
                                          has_heater=False)
        printer.objects["AFC_BambuAMS Bambu_AMS_1"] = fake_unit
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": "boxed:AAAA"}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m._scout_tick(100.0)
        assert fake_unit.ams_model == "ams2"
        assert fake_unit.has_heater is True   # boxed withheld the dryer
        assert m.units[0]["model"] == "ams2"  # status agrees immediately
        assert m.get_status()["pending_restart"] == []

    def test_status_names_the_gap_a_restart_will_close(self, tmp_path,
                                                        monkeypatch):
        # A newly recorded ADDITION cannot apply live (new lanes are new
        # klippy objects), so the status names it until the restart lands.
        bridge = _Bridge(uids=["0123456789ABCDEF00003331", "BBBB"],
                         online=[True, True], htmask=0b1)
        m, printer, _a, _s = _mk_recorded(tmp_path,
                                          "ht:0123456789ABCDEF00003331")
        m.logger = _Logger()
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m._scout_tick(100.0)                  # hold starts for BBBB
        m._scout_tick(116.0)                  # held online past enroll_grace
        pend = m.get_status()["pending_restart"]
        assert any(p.startswith("add ") and "BBBB" in p for p in pend)

    def test_a_pinned_entry_is_never_second_guessed(self, tmp_path,
                                                    monkeypatch):
        # The operator wrote ams1: -- the bus does not overrule the operator.
        bridge = _Bridge(uids=["AAAA"], online=[True], a2mask=0b1,
                         a2asks=[50])
        m = self._watching(tmp_path, monkeypatch, bridge, "ams1:AAAA")
        m._scout_tick(100.0)
        assert m._state_get(self.SEC, "roster") == "ams1:AAAA"

    def test_a_bridge_without_the_counters_is_a_noop(self, tmp_path,
                                                     monkeypatch):
        class _Old:
            _serial = object()
            def send(self, obj): pass
            def chain_uids(self): return ["AAAA"]
            def chain_diag(self): return (0, "", (-1, 0, 0))
            def latest_status(self): return {"units": [
                {"n": 0, "online": True}]}
        m = self._watching(tmp_path, monkeypatch, _Old(), "boxed:AAAA")
        m._scout_tick(100.0)
        assert m._state_get(self.SEC, "roster") == "boxed:AAAA"

    def test_a_counter_reset_never_unrefines_and_boxed_needs_fresh_evidence(
            self, tmp_path, monkeypatch):
        # The firmware restarts an index's counters only when a different
        # UID takes that index (never on an online edge, so a re-plug keeps
        # them): the answered bit clears and asks count from 0.
        A, B, C = "A" * 24, "B" * 24, "C" * 24
        bridge = _Bridge(uids=[A, B, C], online=[True, True, True],
                         a2mask=0b001, a2asks=[3, 300, 11])
        m = self._watching(tmp_path, monkeypatch, bridge,
                           f"boxed:{A}, boxed:{B}, boxed:{C}")
        m._scout_tick(100.0)
        assert m._state_get(self.SEC, "roster") == \
            f"ams2:{A}, ams1:{B}, boxed:{C}"
        logged = list(m.logger.lines)
        # Reset. C's 11 from before is gone, not carried: 5 fresh asks are 5.
        bridge._a2mask, bridge._a2asks = 0, [0, 0, 5]
        m._scout_tick(101.0)
        assert m._state_get(self.SEC, "roster") == \
            f"ams2:{A}, ams1:{B}, boxed:{C}"
        assert m.logger.lines == logged           # nothing said
        bridge._a2asks = [0, 0, 12]               # the floor, on its own count
        m._scout_tick(102.0)
        assert m._state_get(self.SEC, "roster") == \
            f"ams2:{A}, ams1:{B}, ams1:{C}"


# ── one read of the chain reply per tick ─────────────────────────────────────

class _TornBridge(_Bridge):
    """The reader thread swaps the chain cache right after the first read:
    whatever is read after it sees the next reply. `first` and `then` are
    (uids, a2mask, a2asks)."""

    def __init__(self, first, then, online):
        super().__init__(uids=first[0], online=online, a2mask=first[1],
                         a2asks=first[2])
        self._then = then
        self.snapshots = 0

    def _landed(self, value):
        if self._then is not None:
            self._uids, self._a2mask, self._a2asks = (
                list(self._then[0]), self._then[1], list(self._then[2]))
            self._then = None
        return value

    def chain_snapshot(self):
        self.snapshots += 1
        return self._landed({"seq": self.snapshots, "uids": list(self._uids),
                             "htmask": self._htmask, "a2mask": self._a2mask,
                             "a2asks": list(self._a2asks)})

    def chain_uids(self):
        return self._landed(super().chain_uids())

    def chain_dialect(self):
        return self._landed(super().chain_dialect())


class TestOneChainSnapshotPerTick:
    """uids and the per-index counters judged against them come from one
    reply. Read separately, a reply landing mid-tick pairs a UID with the
    counters of whatever took its index next."""

    SEC = "AFC_BridgeBox chain1"
    A, B = "A" * 24, "B" * 24

    def _watching(self, tmp_path, monkeypatch, bridge, roster):
        m, printer, _a, _s = _mk_files(tmp_path)
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": roster}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        return m

    def test_a_torn_read_cannot_produce_a_verdict(self, tmp_path,
                                                  monkeypatch):
        # Reply k: A at index 0, not answered. Reply k+1: B has taken index 0
        # and answered 0x3702. Read separately, A was labelled ams2.
        bridge = _TornBridge(([self.A], 0, [0]), ([self.B], 0b1, [3]),
                             online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge, "boxed:" + self.A)
        m._scout_tick(100.0)
        assert bridge.snapshots == 1
        m._scout_tick(101.0)                  # reply k+1: A is not on it
        assert bridge.snapshots == 2
        assert m._state_get(self.SEC, "roster") == "boxed:" + self.A
        assert not any("confirmed" in ln for ln in m.logger.lines)

    def test_status_rows_come_from_one_snapshot(self, tmp_path, monkeypatch):
        bridge = _TornBridge(([self.A], 0, [2]), ([self.B], 0b1, [7]),
                             online=[True])
        m = self._watching(tmp_path, monkeypatch, bridge, "boxed:" + self.A)
        assert m.get_status()["dialect"]["chain"] == [
            {"i": 0, "uid": self.A, "online": True, "ht": False,
             "answered_3702": False, "asks_3702": 2}]
        assert bridge.snapshots == 1

    def test_a_bridge_without_a_snapshot_is_read_through_its_getters(self):
        snap = afcBridgeBox._chain_snapshot(
            _Bridge(uids=["AAAA", ""], online=[True], htmask=0b10,
                    a2mask=0b1, a2asks=[4, 0]))
        assert snap == {"seq": None, "uids": ["AAAA", ""], "htmask": 0b10,
                        "a2mask": 0b1, "a2asks": [4, 0]}

        class _Old:
            def chain_uids(self): return ["AAAA"]
            def chain_diag(self): return (0, "", (-1, 0, 0))
        assert afcBridgeBox._chain_snapshot(_Old())["a2mask"] == 0

    def test_the_real_bridge_takes_one(self):
        from extras.AFC_BambuAMS_bridge import BambuBridge
        assert callable(getattr(BambuBridge, "chain_snapshot", None))


# ── a live refinement reaches the unit and the firmware ──────────────────────

class _LiveUnit:
    """A claimed, index-resolved unit as _apply_model_live finds it, with the
    unit's own firmware-push helpers."""

    _PROFILES = afcBambuAMS._PROFILES
    profile = afcBambuAMS.profile
    _is_ht = afcBambuAMS._is_ht
    _send_unit_model = afcBambuAMS._send_unit_model
    _send_bindings = afcBambuAMS._send_bindings
    _send_ht_flag = afcBambuAMS._send_ht_flag
    _send_rc_span = afcBambuAMS._send_rc_span
    bus_serial = ""

    def __init__(self, printer, bridge, uid, index=0, resolved=True,
                 name="Bambu_AMS_1"):
        self.printer = printer
        self._bridge = bridge
        self.name = name
        self.unit_uid = uid
        self.ams_index = index
        self.ams_model = "boxed"
        self.has_heater = False
        self.dry_max_temp = 65
        self.measure_on_insert = False
        self._id_resolved = resolved


class TestLiveRefinementSideEffects:
    SEC = "AFC_BridgeBox chain1"
    A, B, C = "A" * 24, "B" * 24, "C" * 24

    def _run(self, tmp_path, monkeypatch, bridge, resolved=True, index=0,
             printing=False, **over):
        m, printer, _a, _s = _mk_files(tmp_path, roster="boxed:" + self.A,
                                       **over)
        unit = _LiveUnit(printer, bridge, self.A, index=index,
                         resolved=resolved)
        printer.objects["AFC_BambuAMS Bambu_AMS_1"] = unit
        if printing:
            printer.objects["print_stats"] = types.SimpleNamespace(
                get_status=lambda et: {"state": "printing"})
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": "boxed:" + self.A}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m._scout_tick(100.0)
        pushed = [c for c in bridge.sent if c.get("cmd") != "chain"]
        return m, unit, pushed

    def test_ams2_gets_its_heater_and_the_firmware_is_told(self, tmp_path,
                                                           monkeypatch):
        # boxed has no heater, so an ams2 kept none until a restart.
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0b1,
                         a2asks=[5])
        m, unit, pushed = self._run(tmp_path, monkeypatch, bridge)
        assert (unit.ams_model, unit.has_heater) == ("ams2", True)
        bind = {"cmd": "bind", "uid": self.A, "idx": 0, "m": 1}
        assert pushed == [{"cmd": "model", "unit": 0, "m": 1}, bind,
                          bind, {"cmd": "idsave"}]    # the table, saved once
        assert not any("RESTART" in ln for ln in m.logger.lines)

    def test_ams1_moves_the_firmware_off_the_ams2_vocabulary(self, tmp_path,
                                                             monkeypatch):
        # boxed runs on the AMS 2 profile; untold, the firmware kept judging an
        # AMS 1's narration by AMS 2's phrases until a restart.
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0,
                         a2asks=[12])
        m, unit, pushed = self._run(tmp_path, monkeypatch, bridge)
        assert (unit.ams_model, unit.has_heater) == ("ams1", False)
        bind = {"cmd": "bind", "uid": self.A, "idx": 0, "m": 0}
        assert pushed == [{"cmd": "model", "unit": 0, "m": 0}, bind,
                          bind, {"cmd": "idsave"}]
        assert not any("RESTART" in ln for ln in m.logger.lines)

    def test_an_unresolved_index_is_not_told(self, tmp_path, monkeypatch):
        # Its ams_index may still be the default -- another unit's. The
        # adoption sends the model once it resolves.
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0,
                         a2asks=[12])
        m, unit, pushed = self._run(tmp_path, monkeypatch, bridge,
                                    resolved=False)
        assert unit.ams_model == "ams1"
        assert pushed == []

    @pytest.mark.parametrize("held, told", [(0, False), (1, True)])
    def test_only_the_index_the_verdict_was_read_at_is_told(
            self, tmp_path, monkeypatch, held, told):
        # The verdict is about index 1, where the snapshot has A. An object
        # still holding index 0 (C's, before a re-pin) would relabel C, so it
        # is sent nothing; its re-pin sends the model from ams_model.
        bridge = _Bridge(uids=[self.C, self.A], online=[True, True],
                         a2mask=0, a2asks=[0, 12])
        m, unit, pushed = self._run(tmp_path, monkeypatch, bridge,
                                    index=held)
        assert unit.ams_model == "ams1"
        assert m._state_get(self.SEC, "roster") == "ams1:" + self.A
        if told:
            assert pushed[0] == {"cmd": "model", "unit": 1, "m": 0}
        else:
            assert pushed == []

    def test_an_unresolved_unit_with_a_model_override_is_not_told(
            self, tmp_path, monkeypatch):
        # The override's capen/htunit are keyed by ams_index as well. The
        # attribute is set, and the adoption's _send_ht_flag carries it.
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0,
                         a2asks=[12])
        fc = _FileConfig({"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        m, unit, pushed = self._run(tmp_path, monkeypatch, bridge,
                                    resolved=False, fileconfig=fc)
        assert (unit.ams_model, unit.measure_on_insert) == ("ams1", True)
        assert pushed == []

    def test_a_model_override_rides_the_one_save(self, tmp_path,
                                                 monkeypatch):
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0,
                         a2asks=[12])
        fc = _FileConfig({"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        m, unit, pushed = self._run(tmp_path, monkeypatch, bridge,
                                    fileconfig=fc)
        assert {"cmd": "capen", "unit": 0, "on": 1} in pushed
        assert [c["cmd"] for c in pushed].count("idsave") == 1
        assert pushed[-1] == {"cmd": "idsave"}

    def test_nothing_is_saved_to_flash_mid_print(self, tmp_path,
                                                 monkeypatch):
        # idsave erases flash with the bus stalled. Mid-print the firmware is
        # told in RAM only; the next connect's _send_ht_flag saves it.
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0,
                         a2asks=[12])
        m, unit, pushed = self._run(tmp_path, monkeypatch, bridge,
                                    printing=True)
        assert pushed == [{"cmd": "model", "unit": 0, "m": 0},
                          {"cmd": "bind", "uid": self.A, "idx": 0, "m": 0}]
        assert any("next connect" in ln for ln in m.logger.lines
                   if "confirmed" in ln)

    def test_two_verdicts_in_one_tick_save_once(self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=[self.A, self.B], online=[True, True],
                         a2mask=0, a2asks=[12, 12])
        roster = f"boxed:{self.A}, boxed:{self.B}"
        m, printer, _a, _s = _mk_files(tmp_path, roster=roster)
        units = [_LiveUnit(printer, bridge, self.A, index=0),
                 _LiveUnit(printer, bridge, self.B, index=1,
                           name="Bambu_AMS_2")]
        for u in units:
            printer.objects["AFC_BambuAMS " + u.name] = u
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": roster}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m._scout_tick(100.0)
        pushed = [c for c in bridge.sent if c.get("cmd") != "chain"]
        assert [u.ams_model for u in units] == ["ams1", "ams1"]
        assert {"cmd": "model", "unit": 0, "m": 0} in pushed
        assert {"cmd": "model", "unit": 1, "m": 0} in pushed
        assert [c["cmd"] for c in pushed].count("idsave") == 1
        assert pushed[-1] == {"cmd": "idsave"}   # after every bind

    def test_a_claim_time_override_waits_for_the_index(self, tmp_path):
        # With no index of the caller's own, the push still waits for the
        # unit's pinned one: claim() leaves it unresolved until its chain
        # request is answered.
        fc = _FileConfig({"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        m, printer, _a, _s = _mk_files(tmp_path, roster="boxed:" + self.A,
                                       fileconfig=fc)
        m.logger = _Logger()
        bridge = _Bridge(uids=[self.A], online=[True])
        unit = _LiveUnit(printer, bridge, self.A, resolved=False)
        printer.objects["AFC_BambuAMS Bambu_AMS_1"] = unit
        m._reapply_model_override(unit, "ams1")
        assert unit.measure_on_insert is True
        assert bridge.sent == []
        unit._id_resolved, unit.measure_on_insert = True, False
        m._reapply_model_override(unit, "ams1")
        assert {"cmd": "capen", "unit": 0, "on": 1} in bridge.sent
        assert bridge.sent[-1] == {"cmd": "idsave"}   # saved, as before

    def test_the_chains_dry_ceiling_reaches_a_live_ams2(self, tmp_path,
                                                        monkeypatch):
        # A fabricated ams2 section carries the chain's dry_max_temp; boxed
        # never got it.
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0b1,
                         a2asks=[5])
        m, unit, _p = self._run(tmp_path, monkeypatch, bridge,
                                dry_max_temp=55)
        assert unit.dry_max_temp == 55

    def test_a_chain_ceiling_above_the_ams2s_leaves_it_at_65(self, tmp_path,
                                                            monkeypatch):
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0b1,
                         a2asks=[5])
        m, unit, _p = self._run(tmp_path, monkeypatch, bridge,
                                dry_max_temp=80)
        assert unit.dry_max_temp == 65


class TestALiveAms2GetsTheDryerARestartWould:
    """A unit confirmed ams2 live must run with the heater flag and ceiling
    the next restart fabricates for it -- never a hotter one."""

    SEC = "AFC_BridgeBox chain1"
    A = "A" * 24

    def _live(self, tmp_path, monkeypatch, fc, a2mask=0b1, a2asks=(5,),
              **over):
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=a2mask,
                         a2asks=list(a2asks))
        (tmp_path / "live").mkdir()
        m, printer, _a, _s = _mk_files(tmp_path / "live",
                                       roster="boxed:" + self.A,
                                       fileconfig=fc, **over)
        unit = _LiveUnit(printer, bridge, self.A)      # boxed: 65, no heater
        printer.objects["AFC_BambuAMS Bambu_AMS_1"] = unit
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": "boxed:" + self.A}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m._scout_tick(100.0)
        return unit

    def _restart(self, tmp_path, fc, **over):
        # What the next boot builds: the same config, the roster now ams2,
        # read the way the unit reads it (heater default from the model).
        (tmp_path / "restart").mkdir()
        m, printer, _a, _s = _mk_files(tmp_path / "restart",
                                       roster="ams2:" + self.A,
                                       fileconfig=fc, **over)
        cfg = _Config({}, printer)
        if fc is not None:
            cfg.fileconfig = fc
        sec = dict(m._fold_and_sweep(cfg, m._roster_sections(m.units)))[
            "AFC_BambuAMS Bambu_AMS_1"]
        heater = str(sec.get("heater", True)).strip().lower() in (
            "1", "true", "yes", "on")
        return heater, int(sec.get("dry_max_temp", 65))

    @pytest.mark.parametrize("sections, over", [
        ({}, {}),
        ({}, {"dry_max_temp": 55}),
        ({}, {"dry_max_temp": 75}),     # raised, but only to the restart's
        ({"AFC_BridgeBox ams2": {"dry_max_temp": "50"}}, {}),
        ({"AFC_BridgeBox ams2": {"dry_max_temp": "50"}},
         {"dry_max_temp": 60}),         # the model section beats the chain
        ({"AFC_BridgeBox ams2": {"dry_max_temp": "50"},
          "AFC_BridgeBox Bambu_AMS_1": {"dry_max_temp": "45"}}, {}),
    ])
    def test_the_live_ceiling_is_the_restarts(self, tmp_path, monkeypatch,
                                              sections, over):
        fc = _FileConfig(sections)
        unit = self._live(tmp_path, monkeypatch, fc, **over)
        assert (unit.has_heater, unit.dry_max_temp) == \
            self._restart(tmp_path, fc, **over)

    def test_a_heater_override_is_kept(self, tmp_path, monkeypatch):
        fc = _FileConfig({"AFC_BridgeBox Bambu_AMS_1": {"heater": "False"}})
        unit = self._live(tmp_path, monkeypatch, fc)
        assert unit.ams_model == "ams2"
        assert unit.has_heater is False
        assert self._restart(tmp_path, fc)[0] is False

    def test_the_ceiling_is_clamped_to_the_hard_max(self, tmp_path,
                                                    monkeypatch):
        # The unit's getint would refuse 99 at the next boot; live, it is
        # held to the bound instead.
        fc = _FileConfig({"AFC_BridgeBox ams2": {"dry_max_temp": "99"}})
        unit = self._live(tmp_path, monkeypatch, fc)
        assert unit.dry_max_temp == 85

    def test_an_ams1_keeps_its_ceiling_and_gets_no_heater(self, tmp_path,
                                                          monkeypatch):
        unit = self._live(tmp_path, monkeypatch, _FileConfig({}), a2mask=0,
                          a2asks=(12,), dry_max_temp=55)
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == \
            ("ams1", False, 65)

    def _spare(self, tmp_path, fc):
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc, pool_ams=1)
        m.logger = _Logger()
        for name in [n for n in printer.objects if n.startswith("AFC_lane ")]:
            del printer.objects[name]
        printer.objects["AFC"] = types.SimpleNamespace(tool_cmds={})
        bay = next(p for p in m._pool_units
                   if p.get("spare") and p.get("family") == "ams")
        unit = _LiveUnit(printer, None, None, resolved=False, name=bay["name"])
        unit.pool = True
        unit.set_master = lambda master: None

        def claim(uid, model):
            # What claim() does to these fields: the model, and the heater
            # from the model table.
            unit.unit_uid, unit.ams_model, unit.pool = uid, model, False
            unit.has_heater = _AMS_MODELS[model][0]
            return True
        unit.claim = claim
        printer.objects["AFC_BambuAMS " + bay["name"]] = unit
        return m, unit

    def test_a_boxed_spare_claimed_as_ams2_gets_the_same(self, tmp_path):
        fc = _FileConfig({"AFC_BridgeBox ams2": {"dry_max_temp": "50"}})
        m, unit = self._spare(tmp_path, fc)
        assert m._claim_pool_unit(self.A, "ams2") is unit
        assert (unit.has_heater, unit.dry_max_temp) == (True, 50)

    def test_a_claim_as_the_built_model_leaves_the_ceiling(self, tmp_path):
        fc = _FileConfig({"AFC_BridgeBox ams2": {"dry_max_temp": "50"}})
        m, unit = self._spare(tmp_path, fc)
        assert m._claim_pool_unit(self.A, "boxed") is unit
        assert (unit.has_heater, unit.dry_max_temp) == (False, 65)


class TestTheTemperatureCardFollowsTheDryer:
    """The unit's [temperature_sensor] leaves the chamber temperature out
    while the unit has no dryer. A `boxed` unit confirmed ams2 gains one
    live, so the card must follow without a restart; report_temperature
    still pins it."""

    def _sensor(self, unit, opts=None):
        from extras.temperature_bambu import TemperatureBambu
        reactor = types.SimpleNamespace(
            register_timer=lambda cb: None, monotonic=lambda: 0.0, NOW=0.0)
        printer = types.SimpleNamespace(
            get_reactor=lambda: reactor,
            add_object=lambda name, obj: None,
            register_event_handler=lambda *a: None,
            lookup_object=lambda name, default=None: (
                unit if name == "AFC_BambuAMS Bambu_AMS_1" else default))
        o = dict(opts or {})
        cfg = types.SimpleNamespace(
            get_printer=lambda: printer,
            get_name=lambda: "temperature_sensor Bambu_AMS_1",
            get=lambda k, d=None: o.get(k, d),
            getint=lambda k, d=None, **kw: int(o.get(k, d)),
            getboolean=lambda k, d=None: o.get(k, d))
        return TemperatureBambu(cfg)

    def _unit(self):
        return types.SimpleNamespace(
            has_heater=False,
            get_status=lambda et: {"humidity": 30.0, "temperature": 41.0})

    def test_a_live_ams2_shows_its_chamber_temperature(self):
        unit = self._unit()
        s = self._sensor(unit)
        s._sample(0.0)
        assert "temperature" not in s.get_status(0.0)
        unit.has_heater = True               # the live ams2 refinement
        s._sample(1.0)
        assert s.get_status(1.0)["temperature"] == 41.0

    def test_report_temperature_still_pins_it(self):
        unit = self._unit()
        s = self._sensor(unit, {"report_temperature": False})
        unit.has_heater = True
        s._sample(0.0)
        assert "temperature" not in s.get_status(0.0)


# ── the chain poll waits out a silent bridge ─────────────────────────────────

class _SilentBridge(_Bridge):
    """A _Bridge that can say how long it has been quiet, as BambuBridge does."""

    def __init__(self, *a, quiet=None, **k):
        super().__init__(*a, **k)
        self.quiet = quiet

    def silent_for(self):
        return self.quiet


class TestTheChainPollWaitsOutASilentBridge:
    """An AMS 1 capscan parks the bridge firmware for 10-15 s with its USB
    side unread. The 1 Hz chain poll sent into that timed out once a second
    (9 lines on lane8, 13 on lane9, printer 2 2026-09-22) and could not be
    answered anyway. The ask is skipped; the tick is not."""

    SEC = "AFC_BridgeBox chain1"

    def _watching(self, tmp_path, monkeypatch, bridge, **over):
        m, printer, _a, _s = _mk_files(tmp_path, **over)
        m.logger = _Logger()
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        return m

    def _asks(self, bridge):
        return [c for c in bridge.sent if c.get("cmd") == "chain"]

    def test_not_asked_mid_stall(self, tmp_path, monkeypatch):
        bridge = _SilentBridge(uids=["AAAA"], online=[True], quiet=5.0)
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:AAAA")
        assert m._scout_tick(100.0) - 100.0 == m.hotplug_poll   # still ticks
        assert self._asks(bridge) == []

    @pytest.mark.parametrize("quiet", [0.3, 2.0, None])
    def test_asked_when_fresh_or_when_it_cannot_say(self, tmp_path,
                                                    monkeypatch, quiet):
        bridge = _SilentBridge(uids=["AAAA"], online=[True], quiet=quiet)
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:AAAA")
        m._scout_tick(100.0)
        assert self._asks(bridge) == [{"cmd": "chain"}]

    def test_a_bridge_without_silent_for_is_asked_as_before(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=["AAAA"], online=[True])
        assert not hasattr(bridge, "silent_for")
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:AAAA")
        m._scout_tick(100.0)
        assert self._asks(bridge) == [{"cmd": "chain"}]

    def test_the_threshold_follows_a_slow_poll(self, tmp_path, monkeypatch):
        # hotplug_poll 4 s: an ordinary gap between asks is up to 4 s, so the
        # bar is 6 s, not 2.
        bridge = _SilentBridge(uids=["AAAA"], online=[True], quiet=5.0)
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:AAAA",
                           hotplug_poll=4.0)
        m._scout_tick(100.0)
        assert self._asks(bridge) == [{"cmd": "chain"}]
        bridge.quiet = 6.5
        m._scout_tick(104.0)
        assert len(self._asks(bridge)) == 1

    def test_the_real_bridge_answers_the_question(self):
        from extras.AFC_BambuAMS_bridge import BambuBridge
        assert callable(getattr(BambuBridge, "silent_for", None))

    def test_prune_still_runs_on_the_cached_chain(self, tmp_path,
                                                  monkeypatch):
        # AAAA absent from a live chain for the grace: recorded removed, even
        # though every tick fell inside a silence and asked nothing.
        bridge = _SilentBridge(uids=["BBBB"], online=[True], quiet=12.0)
        (tmp_path / "AFC_BridgeBox_chain1.roster").write_text(
            "boxed:AAAA\nboxed:BBBB\n")
        m = self._watching(tmp_path, monkeypatch, bridge, roster="")
        m._state_set({self.SEC: {"roster": "boxed:AAAA, boxed:BBBB"}})
        for t in (100.0, 180.0, 300.0):
            m._scout_tick(t)
        assert self._asks(bridge) == []
        assert m._state_get(self.SEC, "roster") == "boxed:BBBB"

    def test_claim_still_runs_on_the_cached_chain(self, tmp_path,
                                                  monkeypatch):
        bridge = _SilentBridge(uids=["AAAA"], online=[True], quiet=12.0)
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:AAAA",
                           pool_ams=1, pool_ht=0)
        claimed = []
        monkeypatch.setattr(m, "_claim_pool_unit",
                            lambda u, model: claimed.append((u, model)))
        m._scout_tick(100.0)
        assert self._asks(bridge) == []
        assert claimed == [("AAAA", "boxed")]

    def test_a_frozen_online_flag_does_not_release_a_unit(self, tmp_path,
                                                          monkeypatch):
        # The flags stop moving during a silence. A unit that was online when
        # it began is still online to the release clock, however long it runs.
        bridge = _SilentBridge(uids=["AAAA"], online=[True], quiet=0.2)
        m = self._watching(tmp_path, monkeypatch, bridge, roster="boxed:AAAA",
                           pool_ams=1, pool_ht=0, auto_drop=True,
                           release_grace=10.0, release_settle=5.0)
        pu = next(p for p in m._pool_units if (p.get("uid") or "") == "AAAA")
        pu["bound"] = "AAAA"
        m._scout_tick(0.0)
        bridge.quiet = 15.0
        for t in (3.0, 6.0, 9.0, 12.0, 15.0):
            m._scout_tick(t)
        assert pu.get("bound") == "AAAA"
        assert len(self._asks(bridge)) == 1


# ── FORGET: the deliberate half of removal ───────────────────────────────────

class _GCmd:
    error = Exception

    def __init__(self, **params):
        self._p = params
        self.responses = []

    def get(self, key, default=None):
        return self._p.get(key, default)

    def get_int(self, key, default=0):
        return int(self._p.get(key, default))

    def respond_info(self, msg):
        self.responses.append(msg)


class TestForget:
    SEC = "AFC_BridgeBox chain1"

    def _seeded(self, tmp_path, **over):
        """Two boxed units known, then a boot with only BBBB -- AAAA is now
        a tombstone holding lanes 24-27 and the name Bambu_AMS_1."""
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        m, p, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB", **over)
        return m

    def test_forget_frees_the_lanes_and_name_for_the_next_unit(
            self, tmp_path):
        # The question this feature answers: a unit fails and never comes
        # back -- its lane numbers must not be pinned forever.
        m = self._seeded(tmp_path)
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        m2, p2, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB, boxed:CCCC")
        sections = dict(m2._roster_sections(m2.units))
        # CCCC slots into the freed range and wears the freed name -- safe
        # because forget erased what was stored under that name and lanes.
        assert sections["AFC_lane lane24"] == {"unit": "Bambu_AMS_1:1",
                                               "unassigned": True}
        assert sections["AFC_BambuAMS Bambu_AMS_1"]["unit_uid"] == "CCCC"

    def test_forget_frees_the_held_slot_live(self, tmp_path):
        # A roster unit that is unhooked keeps its reserved pool slot (uid held)
        # for a re-plug. FORGET says it is not coming back, so it drops that slot
        # to the pool in the RUNNING session -- its pool entry's uid is cleared,
        # so a new same-family unit can claim it with no reboot.
        m, p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        assert any((pu.get("uid") or "") == "AAAA" for pu in m._pool_units), \
            "AAAA should hold a reserved pool slot before forget"
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))   # offline (no bridge)
        assert not any((pu.get("uid") or "") == "AAAA" for pu in m._pool_units)

    def test_forget_erases_the_learned_values(self, tmp_path):
        # Forget means not coming back: the uid's own record goes, and so
        # does everything stored under the bay it frees (its hub, and a
        # name-keyed unit section).
        m = self._seeded(tmp_path)
        m._state_set({self.SEC: {"roster": "boxed:AAAA, boxed:BBBB"},
                      m._learned_section("AAAA"):
                      {"afc_bowden_length": "3632.0"},
                      "AFC_BambuAMS Bambu_AMS_1":
                      {"afc_bowden_length": "3632.0"},
                      "AFC_hub Bambu_AMS_1": {"afc_bowden_length": "3632.0"}})
        cmd = _GCmd(UID="AAAA")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert m._learned_for("AAAA") == {}
        assert m._state_get("AFC_BambuAMS Bambu_AMS_1",
                            "afc_bowden_length") is None
        assert m._state_get("AFC_hub Bambu_AMS_1",
                            "afc_bowden_length") is None
        assert m._state_get(self.SEC, "roster") == "boxed:BBBB"
        assert "AAAA" not in (m._state_get(self.SEC, "lane_map") or "")
        assert "learned values erased" in cmd.responses[0]

    def test_forget_by_name(self, tmp_path):
        m = self._seeded(tmp_path)
        cmd = _GCmd(NAME="Bambu_AMS_1")
        m.cmd_AFC_BRIDGEBOX_FORGET(cmd)
        assert "AAAA" in cmd.responses[0]
        assert "24-27" in cmd.responses[0]

    def test_forget_refuses_an_unknown_uid(self, tmp_path):
        m = self._seeded(tmp_path)
        with pytest.raises(Exception, match="nothing recorded"):
            m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="FEED"))

    def test_forget_drops_a_live_unit_and_suppresses_reenroll(
            self, tmp_path, monkeypatch):
        # A live FORGET no longer refuses: it clears the unit NOW and parks its
        # uid on the suppress list so the scout does not re-enroll the hardware
        # the operator just cleared (it is still on the wire). No FORCE needed.
        m = self._seeded(tmp_path)
        bridge = _Bridge(uids=["AAAA", "BBBB"], online=[True, True])
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))      # online, not printing
        assert "AAAA" not in (m._state_get(self.SEC, "lane_map") or "")
        assert "AAAA" in m._forget_suppressed

    def test_forget_drops_a_bound_live_unit(self, tmp_path, monkeypatch):
        # With the unit actually claimed onto its slot, a live FORGET releases
        # it (bound cleared) and returns the slot to the pool -- the drop half,
        # not just the record edit.
        m, p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB")
        m.logger = _Logger()
        bridge = _Bridge(uids=["AAAA", "BBBB"], online=[True, True])
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        pu = next(x for x in m._pool_units if (x.get("uid") or "") == "AAAA")
        pu["bound"] = "AAAA"                              # pretend it is claimed
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        assert pu.get("bound") is None                   # lanes released live
        assert (pu.get("uid") or "") != "AAAA"           # slot back in the pool
        assert "AAAA" in m._forget_suppressed

    def test_forget_refuses_a_unit_on_a_bay_mid_print(self, tmp_path,
                                                      monkeypatch):
        # The one guard kept: yanking a lane out mid-print would disrupt the
        # job, so a unit claimed onto a bay is refused while printing unless
        # FORCE=1 -- matching the auto-drop print gate.
        class _PrintStats:
            def get_status(self, _now):
                return {"state": "printing"}
        printer = _Printer({"print_stats": _PrintStats()})
        m, p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA, boxed:BBBB",
                                 printer=printer)
        m.logger = _Logger()
        bridge = _Bridge(uids=["AAAA", "BBBB"], online=[True, True])
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        pu = next(x for x in m._pool_units if (x.get("uid") or "") == "AAAA")
        pu["bound"] = "AAAA"
        with pytest.raises(Exception, match="AAAA is on Bambu_AMS_1 and a "
                                            "print is active"):
            m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA", FORCE=1))   # override
        assert "AAAA" not in (m._state_get(self.SEC, "lane_map") or "")

    def test_forget_of_an_online_unit_on_no_bay_mid_print_goes_ahead(
            self, tmp_path, monkeypatch):
        # AAAA is on the wire but holds no bay, lanes or T#: forgetting it
        # touches nothing the print uses.
        class _PrintStats:
            def get_status(self, _now):
                return {"state": "printing"}
        printer = _Printer({"print_stats": _PrintStats()})
        m = self._seeded(tmp_path, printer=printer)
        bridge = _Bridge(uids=["AAAA", "BBBB"], online=[True, True])
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID="AAAA"))
        assert "AAAA" not in (m._state_get(self.SEC, "lane_map") or "")
        assert "AAAA" in m._forget_suppressed

    def test_without_forget_the_tombstone_still_holds(self, tmp_path):
        # The counterpart that makes forget THE reclaim path: mere absence,
        # however long, never frees anything.
        self._seeded(tmp_path)
        m2, p2, _a, _s = _mk_files(tmp_path, roster="boxed:BBBB, boxed:CCCC")
        sections = dict(m2._roster_sections(m2.units))
        assert sections["AFC_lane lane32"] == {"unit": "Bambu_AMS_3:1",
                                               "unassigned": True}
        assert "AFC_lane lane24" not in sections

    def test_the_command_registers_muxed_with_a_default(self, tmp_path):
        class _GCode:
            def __init__(self):
                self.mux = []

            def register_mux_command(self, cmd, key, value, fn, desc=None):
                if (cmd, key, value) in [(c, k, v)
                                         for c, k, v, _f in self.mux]:
                    raise Exception("already registered")
                self.mux.append((cmd, key, value, fn))

        gcode = _GCode()
        printer = _Printer({"gcode": gcode})
        m, printer, _a, _s = _mk_files(tmp_path, printer=printer)
        got = [(c, k, v) for c, k, v, _f in gcode.mux]
        assert ("AFC_BRIDGEBOX_FORGET", "CHAIN", "chain1") in got
        assert ("AFC_BRIDGEBOX_FORGET", "CHAIN", None) in got
        # ASSIGN/UNASSIGN register the same way, muxed with the no-CHAIN default.
        assert ("AFC_BRIDGEBOX_ASSIGN", "CHAIN", "chain1") in got
        assert ("AFC_BRIDGEBOX_ASSIGN", "CHAIN", None) in got
        assert ("AFC_BRIDGEBOX_UNASSIGN", "CHAIN", "chain1") in got
        assert ("AFC_BRIDGEBOX_UNASSIGN", "CHAIN", None) in got

    def test_every_command_help_shows_chain(self, tmp_path):
        # A second chain's commands are reached only with CHAIN=.
        class _GCode:
            def __init__(self):
                self.desc = {}

            def register_mux_command(self, cmd, key, value, fn, desc=None):
                self.desc[cmd] = desc

        gcode = _GCode()
        _mk_files(tmp_path, printer=_Printer({"gcode": gcode}))
        assert sorted(gcode.desc) == [
            "AFC_BRIDGEBOX_ASSIGN", "AFC_BRIDGEBOX_BAYS",
            "AFC_BRIDGEBOX_FORGET", "AFC_BRIDGEBOX_REPLACE",
            "AFC_BRIDGEBOX_UNASSIGN"]
        for cmd, desc in gcode.desc.items():
            assert f"{cmd} [CHAIN=<chain>]" in desc, cmd


class TestAssignUnassign:
    """AFC_BRIDGEBOX_ASSIGN pins a uid to a named bay; UNASSIGN unlinks it.
    Named bays come from ams_names / ht_names, positional. These run offline
    (no bridge) so ASSIGN only pins + persists -- the live-claim half is the
    same _claim_pool_unit exercised elsewhere. The roster is the recorded one
    (no roster: option), where ASSIGN enrolls a new uid."""

    SEC = "AFC_BridgeBox chain1"

    def _pool(self, tmp_path, **over):
        # One known boxed unit (AAAA -> lowest ams bay) plus one free ams
        # spare and one free ht spare, with the bays named.
        return _mk_recorded(tmp_path, "boxed:AAAA", pool_ams=2, pool_ht=1,
                            ams_names="Alpha, Bravo", ht_names="Hot",
                            **over)[0]

    def _bay(self, m, name):
        return next((p for p in m._pool_units if p.get("name") == name), None)

    def test_a_spare_bay_wears_its_configured_name(self, tmp_path):
        m = self._pool(tmp_path)
        # AAAA on the lowest ams bay -> "Alpha"; the free ams spare -> "Bravo";
        # the ht spare -> "Hot".
        assert self._bay(m, "Alpha") is not None
        assert self._bay(m, "Bravo") is not None
        assert (self._bay(m, "Bravo") or {}).get("uid") is None
        assert self._bay(m, "Hot") is not None

    def test_assign_pins_a_uid_to_a_named_bay(self, tmp_path):
        m = self._pool(tmp_path)
        cmd = _GCmd(UID="CCCC", NAME="Bravo")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert self._bay(m, "Bravo")["uid"] == "CCCC"
        # Persisted so it holds across a restart: maps + a fresh roster entry.
        assert "CCCC:28:4" in (m._state_get(self.SEC, "lane_map") or "")
        assert "CCCC:Bravo" in (m._state_get(self.SEC, "name_map") or "")
        assert "boxed:CCCC" in (m._state_get(self.SEC, "roster") or "")
        assert "Bravo" in cmd.responses[0]

    def test_a_pinned_bay_survives_a_restart(self, tmp_path):
        m = self._pool(tmp_path)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        # ASSIGN enrolled CCCC in the roster and pinned its lane; a reboot that
        # reads that roster fabricates it back on Bravo (lane28) from the
        # persisted lane map, no re-detection needed.
        m2 = _mk_files(tmp_path, roster="boxed:AAAA, boxed:CCCC",
                       pool_ams=2, pool_ht=1,
                       ams_names="Alpha, Bravo", ht_names="Hot")[0]
        u = dict(m2._roster_sections(m2.units))["AFC_BambuAMS Bravo"]
        assert u["unit_uid"] == "CCCC"
        assert m2._lane_map["CCCC"] == (28, 4)

    def test_an_option_boxed_unit_the_bus_confirmed_ams2_is_claimed_ams2(
            self, tmp_path, monkeypatch):
        # A roster: option `boxed` is only unconfirmed: an ams1/ams2 the
        # recorded roster holds for the uid stands, as for the watch's claim,
        # so an AMS 2 Pro moved by ASSIGN keeps its heater.
        m = _mk_files(tmp_path, roster="boxed:AAAA", pool_ams=2, pool_ht=0,
                      ams_names="Alpha, Bravo")[0]
        m._state_set({self.SEC: {"roster": "ams2:AAAA"}})
        assert m._model_for_uid("AAAA") == "ams2"
        claimed = []
        monkeypatch.setattr(m, "_uid_online_now", lambda uid: True)
        monkeypatch.setattr(m, "_claim_pool_unit",
                            lambda uid, model: claimed.append((uid, model)))
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="AAAA", NAME="Bravo"))
        assert claimed == [("AAAA", "ams2")]
        m._state_set({self.SEC: {"roster": "boxed:AAAA"}})
        assert m._model_for_uid("AAAA") == "boxed"

    def test_assign_refuses_an_occupied_bay(self, tmp_path):
        m = self._pool(tmp_path)
        with pytest.raises(Exception, match="already assigned"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="DDDD", NAME="Alpha"))

    def test_assign_refuses_a_family_mismatch(self, tmp_path):
        # HHHH is a known HT; an HT (one lane) cannot take a four-lane AMS bay.
        m = self._mk_ht(tmp_path)
        with pytest.raises(Exception, match="lane counts differ"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="HHHH", NAME="Bravo"))

    def _mk_ht(self, tmp_path):
        return _mk_files(tmp_path, roster="boxed:AAAA, ht:HHHH",
                         pool_ams=2, pool_ht=1,
                         ams_names="Alpha, Bravo", ht_names="Hot")[0]

    def test_assign_refuses_an_unknown_bay(self, tmp_path):
        m = self._pool(tmp_path)
        with pytest.raises(Exception, match="no pool bay named"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Nope"))

    def test_assign_needs_both_uid_and_name(self, tmp_path):
        m = self._pool(tmp_path)
        with pytest.raises(Exception, match="UID.*NAME|give"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC"))

    def test_assign_moves_a_unit_between_bays(self, tmp_path):
        # Two free ams bays so a same-family move has somewhere to go.
        m = _mk_recorded(tmp_path, "boxed:AAAA", pool_ams=3, pool_ht=1,
                         ams_names="Alpha, Bravo, Charlie", ht_names="Hot")[0]
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Charlie"))
        assert self._bay(m, "Bravo")["uid"] is None      # vacated
        assert self._bay(m, "Charlie")["uid"] == "CCCC"
        assert "CCCC:Charlie" in (m._state_get(self.SEC, "name_map") or "")

    def test_unassign_frees_the_bay(self, tmp_path):
        m = self._pool(tmp_path)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC"))
        assert self._bay(m, "Bravo")["uid"] is None
        assert "CCCC" not in (m._state_get(self.SEC, "name_map") or "")
        assert "CCCC" not in (m._state_get(self.SEC, "lane_map") or "")

    def test_unassign_keeps_the_learned_values(self, tmp_path):
        # Unlike FORGET, UNASSIGN leaves calibration in place: it stays with
        # the unit, for whatever bay it lands on next.
        m = self._pool(tmp_path)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        m._state_set({m._learned_section("CCCC"):
                      {"afc_bowden_length": "3632.0"}})
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC"))
        assert m._learned_for("CCCC") == {"afc_bowden_length": "3632.0"}

    def test_unassign_by_bay_name(self, tmp_path):
        m = self._pool(tmp_path)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(NAME="Bravo"))
        assert self._bay(m, "Bravo")["uid"] is None

    def test_unassign_refuses_an_online_unit(self, tmp_path, monkeypatch):
        m = self._pool(tmp_path)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        bridge = _Bridge(uids=["AAAA", "CCCC"], online=[True, True])
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        with pytest.raises(Exception, match="ONLINE"):
            m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC"))
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC", FORCE=1))
        assert self._bay(m, "Bravo")["uid"] is None


class _CapGCode:
    """A gcode that records respond_raw lines and no-ops registration, so a
    popup's action:prompt output can be inspected."""

    def __init__(self):
        self.raw = []

    def respond_raw(self, msg):
        self.raw.append(msg)

    def respond_info(self, msg):
        self.raw.append(msg)

    def register_mux_command(self, *a, **k):
        pass

    def register_command(self, *a, **k):
        pass


class _FakeReactor:
    NEVER = float("inf")

    def __init__(self):
        self.callbacks = []            # (cb, when), for a test to fire by hand

    def monotonic(self):
        return 0.0

    def register_callback(self, cb, when=None):
        self.callbacks.append((cb, when))

    def register_timer(self, cb, when=None):
        return object()


class TestFlapTolerantRelease:
    """A physically-absent unit can leave a phantom online flag that flaps true
    every second or two (seen live: a pulled AMS on an otherwise HT-only chain).
    A single blip must NOT reset the release clock, or its lanes never drop --
    but a genuine re-plug that reads SOLIDLY online must still cancel the drop."""

    def _bound(self, tmp_path, monkeypatch, first_online, **over):
        g = _CapGCode()
        printer = _Printer({"gcode": g})
        reactor = _FakeReactor()
        printer.get_reactor = lambda: reactor
        opts = dict(auto_drop=True, release_grace=10.0, release_settle=5.0,
                    pool_ams=1, pool_ht=0)
        opts.update(over)
        m = _mk_files(tmp_path, roster="boxed:AAAA", printer=printer, **opts)[0]
        m.logger = _Logger()
        bridge = _Bridge(uids=["AAAA"], online=[first_online])
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        pu = next(p for p in m._pool_units
                  if (p.get("uid") or "") == "AAAA")
        pu["bound"] = "AAAA"                      # as a live claim would leave it
        return m, bridge, pu

    def test_a_phantom_flap_still_drops_after_the_grace(
            self, tmp_path, monkeypatch):
        # online reads 0101...; no single blip may cancel the drop.
        m, bridge, pu = self._bound(tmp_path, monkeypatch, False)
        t = 0.0
        for on in [False, True, False, True, False, True, False]:  # ...through t=18
            bridge._online = [on]
            m._scout_tick(t)
            t += 3.0
        assert pu.get("bound") is None            # dropped despite the blips

    def test_a_solid_replug_is_not_dropped(self, tmp_path, monkeypatch):
        # Reads online every tick from the start: never drops.
        m, bridge, pu = self._bound(tmp_path, monkeypatch, True)
        for t in (0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0):
            bridge._online = [True]
            m._scout_tick(t)
        assert pu.get("bound") == "AAAA"          # solidly present -> held

    def test_a_late_solid_replug_cancels_the_pending_drop(
            self, tmp_path, monkeypatch):
        # Offline almost to the grace, then a genuine re-plug reads solidly
        # online: the offline-this-tick guard spares it even before the run
        # clears the clock, so it is never dropped.
        m, bridge, pu = self._bound(tmp_path, monkeypatch, False)
        for t, on in [(0.0, False), (3.0, False), (6.0, False), (9.0, True),
                      (12.0, True), (15.0, True), (18.0, True), (21.0, True)]:
            bridge._online = [on]
            m._scout_tick(t)
        assert pu.get("bound") == "AAAA"          # re-plug held it

    def test_a_clean_sustained_offline_drops(self, tmp_path, monkeypatch):
        # The ordinary case still works: continuously offline past the grace.
        m, bridge, pu = self._bound(tmp_path, monkeypatch, False)
        for t in (0.0, 3.0, 6.0, 9.0, 12.0):
            bridge._online = [False]
            m._scout_tick(t)
        assert pu.get("bound") is None


class _ReturningUnit:
    """A bound unit as _restore_returned finds it: records each return it is
    told about and answers whether it engaged (it owns the loaded lane)."""

    def __init__(self, engaged=True):
        self.returns = []
        self.engaged = engaged

    def restore_follower_on_return(self, since):
        self.returns.append(since)
        return self.engaged


class TestAReturnRestoresTheFollower:
    """A unit that stays bound through an unplug (auto_drop off, or back
    inside release_grace) is never claimed again, and the AMS comes back idle
    with the loaded tray no longer followed. Its settled return re-runs the
    follower restore; a lone offline blip is not a return, and a unit that was
    released leaves the restore to its reclaim."""

    def _bound(self, tmp_path, monkeypatch, engaged=True, **over):
        printer = _Printer({"gcode": _CapGCode()})
        reactor = _FakeReactor()
        printer.get_reactor = lambda: reactor
        opts = dict(release_grace=10.0, release_settle=5.0,
                    pool_ams=1, pool_ht=0)
        opts.update(over)
        m = _mk_files(tmp_path, roster="boxed:AAAA", printer=printer, **opts)[0]
        m.logger = _Logger()
        bridge = _Bridge(uids=["AAAA"], online=[True])
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        pu = next(p for p in m._pool_units
                  if (p.get("uid") or "") == "AAAA")
        pu["bound"] = "AAAA"                      # as a live claim would leave it
        unit = _ReturningUnit(engaged)
        printer.objects[f"AFC_BambuAMS {pu['name']}"] = unit
        return m, bridge, pu, unit

    @staticmethod
    def _reads(m, bridge, reads):
        for t, on in reads:
            bridge._online = [on]
            m._scout_tick(t)

    def test_a_settled_return_after_an_absence_restores_once(
            self, tmp_path, monkeypatch):
        # auto_drop is off (the default): the unit stays bound throughout.
        m, bridge, pu, unit = self._bound(tmp_path, monkeypatch)
        self._reads(m, bridge, [(0.0, True)]
                    + [(t, False) for t in (1.0, 2.0, 3.0, 4.0)]
                    + [(t, True) for t in (5.0, 7.0, 9.0)])
        assert unit.returns == []                 # not settled yet
        self._reads(m, bridge, [(10.0, True)])
        assert unit.returns == [5.0]              # since the run began
        self._reads(m, bridge, [(t, True) for t in (11.0, 20.0, 60.0)])
        assert unit.returns == [5.0]              # once per return
        assert pu.get("bound") == "AAAA"
        assert any("is back after 3s offline" in ln
                   and "restore engaged." in ln for ln in m.logger.debugs)
        # The unit that engages prints its own line; the return adds none.
        assert not any("is back after" in ln for ln in m.logger.lines)

    def test_a_return_with_nothing_to_engage_stays_off_the_console(
            self, tmp_path, monkeypatch):
        # The unit does not own the loaded lane (or nothing is loaded, or the
        # follower has another owner): it is asked, engages nothing, and the
        # console hears nothing of it.
        m, bridge, pu, unit = self._bound(tmp_path, monkeypatch,
                                          engaged=False)
        self._reads(m, bridge, [(0.0, True)]
                    + [(t, False) for t in (1.0, 2.0, 3.0, 4.0)]
                    + [(t, True) for t in (5.0, 10.0)])
        assert unit.returns == [5.0]
        assert m.logger.lines == []
        assert any("restore engaged nothing." in ln
                   for ln in m.logger.debugs)

    def test_a_lone_blip_is_not_a_return(self, tmp_path, monkeypatch):
        m, bridge, pu, unit = self._bound(tmp_path, monkeypatch)
        self._reads(m, bridge, [(0.0, True), (1.0, False)]
                    + [(float(t), True) for t in range(2, 20)])
        assert unit.returns == []

    def test_a_short_dropout_is_not_a_return(self, tmp_path, monkeypatch):
        # Two offline reads a second apart span less than _REPLUG_ABSENCE.
        m, bridge, pu, unit = self._bound(tmp_path, monkeypatch)
        self._reads(m, bridge, [(0.0, True), (1.0, False), (2.0, False)]
                    + [(float(t), True) for t in range(3, 20)])
        assert unit.returns == []

    def test_a_flapping_absence_restores_only_on_the_settled_return(
            self, tmp_path, monkeypatch):
        # A pulled unit whose flag blips online every other poll is still
        # away; blips neither end the absence nor count as the return.
        m, bridge, pu, unit = self._bound(tmp_path, monkeypatch)
        flap = [(float(t), t % 2 == 0) for t in range(1, 30)]
        self._reads(m, bridge, [(0.0, True)] + flap)
        assert unit.returns == []
        self._reads(m, bridge, [(float(t), True) for t in range(30, 40)])
        assert unit.returns == [30.0]

    def test_a_return_mid_print_still_restores(self, tmp_path, monkeypatch):
        # A power-cycled AMS feeding the print is the one that needs it most.
        m, bridge, pu, unit = self._bound(tmp_path, monkeypatch)
        m._is_printing = lambda: True
        self._reads(m, bridge, [(0.0, True)]
                    + [(t, False) for t in (1.0, 2.0, 3.0, 4.0)]
                    + [(t, True) for t in (5.0, 10.0)])
        assert unit.returns == [5.0]

    def test_a_released_unit_leaves_the_restore_to_its_reclaim(
            self, tmp_path, monkeypatch):
        # Offline past release_grace with auto_drop on: released, then
        # reclaimed when it comes back. claim() schedules the restore itself,
        # so the return must not schedule a second one.
        m, bridge, pu, unit = self._bound(tmp_path, monkeypatch,
                                          auto_drop=True)
        claimed = []

        def _claim(u, model):
            claimed.append(u)
            pu["bound"] = u
            return unit

        monkeypatch.setattr(m, "_claim_pool_unit", _claim)
        self._reads(m, bridge, [(0.0, True)]
                    + [(float(t), False) for t in range(1, 13)])
        assert pu.get("bound") is None            # released
        self._reads(m, bridge, [(float(t), True) for t in range(13, 60)])
        assert claimed == ["AAAA"]                # reclaimed once
        assert unit.returns == []                 # the claim owns the restore


class TestPopups:
    """The Mainsail/Fluidd action:prompt popups on plug/unplug, their queue,
    and the direct commands that raise them on demand."""

    def _m(self, tmp_path, **over):
        g = _CapGCode()
        printer = _Printer({"gcode": g})
        reactor = _FakeReactor()
        printer.get_reactor = lambda: reactor
        m = _mk_recorded(tmp_path, "boxed:AAAA", pool_ams=3, pool_ht=1,
                         ams_names="Alpha, Bravo, Charlie", ht_names="Hot",
                         printer=printer, **over)[0]
        m._test_reactor = reactor
        g.raw.clear()               # drop any load-time chatter
        return m, g

    def _bind(self, m, name, uid):
        pu = next(p for p in m._pool_units if p["name"] == name)
        pu["uid"] = uid
        pu["bound"] = uid
        return pu

    def test_new_unit_popup_offers_only_free_same_family_bays(self, tmp_path):
        m, g = self._m(tmp_path)
        self._bind(m, "Bravo", "CCCC")       # CCCC landed on the Bravo spare
        m._prompt_new_unit("CCCC")
        text = "\n".join(g.raw)
        assert "action:prompt_begin New AMS on Bravo" in text
        assert "NAME=Charlie" in text        # other free ams bay -> offered
        assert "NAME=Hot" not in text        # HT bay -> wrong family
        assert "NAME=Alpha" not in text      # occupied by AAAA
        assert "NAME=Bravo" not in text      # its own current bay

    def test_forget_with_no_args_pops_a_picker_of_known_units(self, tmp_path):
        # AFC_BRIDGEBOX_FORGET with no UID/NAME lists every recorded unit, one
        # Forget button each. _m enrolls AAAA (named Alpha via ams_names).
        m, g = self._m(tmp_path)
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd())
        text = "\n".join(g.raw)
        assert "action:prompt_begin Forget a Bambu AMS unit" in text
        assert "Forget Alpha" in text
        assert "AFC_BRIDGEBOX_FORGET" in text and "UID=AAAA" in text

    def test_removed_unit_popup_offers_forget(self, tmp_path):
        m, g = self._m(tmp_path)
        m._prompt_removed_unit("AAAA", "Alpha")
        text = "\n".join(g.raw)
        assert "action:prompt_begin AMS removed: Alpha" in text
        assert "AFC_BRIDGEBOX_FORGET" in text and "UID=AAAA" in text

    def test_removed_popup_says_held_only_for_a_reserved_bay(self, tmp_path):
        # AAAA is rostered, so its bay keeps its uid after a release. CCCC sat
        # on the Bravo spare, which a release hands back to the pool.
        m, g = self._m(tmp_path)
        m._prompt_removed_unit("AAAA", "Alpha")
        assert "its bay is held for a re-plug" in "\n".join(g.raw)
        g.raw.clear()
        m._prompt_removed_unit("CCCC", "Bravo")
        text = "\n".join(g.raw)
        assert "held for" not in text
        assert "went back to the pool" in text
        assert "AFC_BRIDGEBOX_FORGET" in text and "UID=CCCC" in text

    def test_the_queue_shows_one_popup_per_hold(self, tmp_path):
        m, g = self._m(tmp_path)
        self._bind(m, "Bravo", "CCCC")
        self._bind(m, "Charlie", "DDDD")
        m._queue_popup(("new", "CCCC"))
        m._queue_popup(("new", "DDDD"))
        m._pump_popups(0.0)                  # first turn
        assert sum("prompt_begin" in x for x in g.raw) == 1
        m._pump_popups(1.0)                  # still within the hold -> nothing
        assert sum("prompt_begin" in x for x in g.raw) == 1
        m._pump_popups(0.0 + m._POPUP_HOLD)  # hold elapsed -> second turn
        assert sum("prompt_begin" in x for x in g.raw) == 2

    def test_the_queue_dedupes_a_flapping_uid(self, tmp_path):
        m, _g = self._m(tmp_path)
        m._queue_popup(("new", "CCCC"))
        m._queue_popup(("new", "CCCC"))
        assert m._popup_queue == [("new", "CCCC")]

    def test_assign_with_no_name_pops_the_picker(self, tmp_path):
        m, g = self._m(tmp_path)
        self._bind(m, "Bravo", "CCCC")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC"))
        assert any("action:prompt_begin New AMS on Bravo" in x for x in g.raw)

    def test_assign_closes_the_dialog_on_success(self, tmp_path):
        # Clicking a picker button runs ASSIGN; Mainsail won't close the dialog
        # on its own, so ASSIGN emits prompt_end itself.
        m, g = self._m(tmp_path)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        assert any("action:prompt_end" in x for x in g.raw)

    def test_unassign_closes_the_dialog(self, tmp_path):
        m, g = self._m(tmp_path)
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="CCCC", NAME="Bravo"))
        g.raw.clear()
        m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID="CCCC"))
        assert any("action:prompt_end" in x for x in g.raw)

    def test_assign_with_no_name_needs_a_placed_unit(self, tmp_path):
        m, _g = self._m(tmp_path)
        with pytest.raises(Exception, match="not on any bay"):
            m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="ZZZZ"))

    def test_a_stale_dismiss_timer_does_not_close_a_later_popup(self, tmp_path):
        # The BAYS-closes-too-fast bug: an earlier popup's auto-dismiss timer
        # fired while a later dialog was open and closed it. Each popup's timer
        # now only closes IT (generation guard), so a stale timer is a no-op.
        m, g = self._m(tmp_path)
        self._bind(m, "Bravo", "CCCC")
        m._prompt_new_unit("CCCC")                      # popup gen 1
        close1 = m._test_reactor.callbacks[-1][0]
        m.cmd_AFC_BRIDGEBOX_BAYS(_GCmd())                # popup gen 2 (manager)
        close2 = m._test_reactor.callbacks[-1][0]
        g.raw.clear()
        close1(0.0)                                     # stale timer fires
        assert not any("prompt_end" in x for x in g.raw)  # ...and is ignored
        close2(0.0)                                      # the manager's own timer
        assert any("prompt_end" in x for x in g.raw)      # ...does close it

    def test_bays_manager_lists_and_offers_unassign(self, tmp_path):
        m, g = self._m(tmp_path)
        self._bind(m, "Bravo", "CCCC")
        m.cmd_AFC_BRIDGEBOX_BAYS(_GCmd())
        text = "\n".join(g.raw)
        assert "action:prompt_begin Bambu AMS Units" in text
        assert "Alpha" in text and "Bravo" in text and "Hot" in text
        # Occupied bays get an Unassign button; free ones do not.
        assert "Unassign Bravo" in text
        assert "Unassign Charlie" not in text


class TestNamesOutsideTheRoster:
    """A name recorded for a uid the roster does not list holds no bay. The AMS
    band is pool_ams bays wide and the HT band starts right past it, so a
    spare pushed past such a name lands on HT lanes and Klipper stops at
    DuplicateSectionError. With roster: set, ASSIGN of a uid it does not list
    would record such a name, so it is refused."""

    SEC = "AFC_BridgeBox chain1"
    POOL = dict(pool_ams=2, pool_ht=1)

    @staticmethod
    def _lanes(printer):
        return [s for s, _w in printer.loaded if s.startswith("AFC_lane")]

    def _names(self, m):
        raw = m._state_get(self.SEC, "name_map") or ""
        return dict(e.strip().split(":", 1) for e in raw.split(",") if e.strip())

    def test_assign_refuses_a_uid_the_roster_option_does_not_list(
            self, tmp_path):
        m = _mk_files(tmp_path, roster="boxed:AAAA", **self.POOL)[0]
        cmd = _GCmd(UID="CCCC", NAME="Bambu_AMS_2")
        with pytest.raises(Exception, match="roster:") as e:
            m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert "boxed:CCCC" in str(e.value)       # the entry to add
        assert cmd.responses == []
        assert "CCCC" not in self._names(m)       # nothing recorded
        bay = next(p for p in m._pool_units if p["name"] == "Bambu_AMS_2")
        assert bay.get("uid") is None

    def test_assign_still_moves_a_listed_uid(self, tmp_path):
        m = _mk_files(tmp_path, roster="boxed:AAAA", **self.POOL)[0]
        cmd = _GCmd(UID="AAAA", NAME="Bambu_AMS_2")
        m.cmd_AFC_BRIDGEBOX_ASSIGN(cmd)
        assert self._names(m) == {"AAAA": "Bambu_AMS_2"}
        assert "only fully holds" not in cmd.responses[0]

    def test_the_new_unit_popup_offers_no_bay_to_an_unlisted_uid(
            self, tmp_path):
        g = _CapGCode()
        printer = _Printer({"gcode": g})
        printer.get_reactor = lambda: _FakeReactor()
        m = _mk_files(tmp_path, roster="boxed:AAAA", pool_ams=3, pool_ht=1,
                      printer=printer)[0]
        pu = next(p for p in m._pool_units if p["name"] == "Bambu_AMS_2")
        pu["uid"] = pu["bound"] = "CCCC"          # claimed live, unlisted
        g.raw.clear()
        m._prompt_new_unit("CCCC")
        assert any("prompt_begin New AMS on Bambu_AMS_2" in x for x in g.raw)
        assert not any("prompt_button" in x for x in g.raw)
        text = "\n".join(x for x in g.raw if "prompt_text" in x)
        assert "boxed:CCCC" in text and "roster:" in text

    def test_a_uid_dropped_from_the_roster_option_frees_its_bay(
            self, tmp_path):
        # CCCC leaves roster: without FORGET; its recorded name must not push
        # the AMS spare onto the HT band's first lane.
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:CCCC", **self.POOL)
        m, p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA", **self.POOL)
        lanes = self._lanes(p)
        assert lanes == [f"AFC_lane lane{n}" for n in range(24, 33)]
        secs = dict(m._roster_sections(m.units))
        assert secs["AFC_lane lane28"]["unit"] == "Bambu_AMS_2:1"   # spare
        assert secs["AFC_lane lane32"]["unit"] == "Bambu_AMS_HT_1:1"

    def test_a_pin_recorded_for_an_unlisted_uid_no_longer_overlaps(
            self, tmp_path):
        # A pin for a uid roster: does not list, as a state file can hold.
        m = _mk_files(tmp_path, roster="boxed:AAAA", **self.POOL)[0]
        m._persist_pin("CCCC", 28, 4, "Bambu_AMS_2", "boxed")
        _m2, p2, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA",
                                    **self.POOL)
        assert self._lanes(p2) == [f"AFC_lane lane{n}" for n in range(24, 33)]

    def test_a_new_rostered_uid_takes_the_name_over(self, tmp_path):
        # DDDD joins while CCCC's name is still recorded: DDDD wears it (the
        # bay it would have claimed live), and CCCC's entries go, so no two
        # uids are recorded with one name.
        _mk_files(tmp_path, roster="boxed:AAAA, boxed:CCCC", **self.POOL)
        _mk_files(tmp_path, roster="boxed:AAAA", **self.POOL)
        m, p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA, boxed:DDDD",
                                 **self.POOL)
        assert self._lanes(p) == [f"AFC_lane lane{n}" for n in range(24, 33)]
        names = self._names(m)
        assert names == {"AAAA": "Bambu_AMS_1", "DDDD": "Bambu_AMS_2"}
        assert "CCCC" not in (m._state_get(self.SEC, "lane_map") or "")

    def test_without_a_pool_a_tombstone_never_puts_an_ams_on_the_ht(
            self, tmp_path):
        # No pool: a new AMS passes over a tombstone unless that lands it on
        # the HT band (AMS count 1 -> the HT sits at lane28).
        _mk_files(tmp_path, roster="boxed:AAAA, ht:HHHH")
        m, p, _a, _s = _mk_files(tmp_path, roster="ht:HHHH, boxed:BBBB")
        secs = dict(m._roster_sections(m.units))
        assert secs["AFC_BambuAMS Bambu_AMS_1"]["unit_uid"] == "BBBB"
        assert secs["AFC_lane lane28"]["unit"] == "Bambu_AMS_HT_1:1"
        assert "AAAA" not in self._names(m)

    def test_assign_onto_a_spare_wearing_a_recorded_name_takes_it_over(
            self, tmp_path):
        m = _mk_recorded(tmp_path, "boxed:AAAA, boxed:CCCC", **self.POOL)[0]
        m._state_set({self.SEC: {"roster": "boxed:AAAA"}})   # CCCC dropped
        m2 = _mk_files(tmp_path, roster="", **self.POOL)[0]
        m2.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="DDDD", NAME="Bambu_AMS_2"))
        assert self._names(m2) == {"AAAA": "Bambu_AMS_1",
                                   "DDDD": "Bambu_AMS_2"}
        assert m2._name_map == {"AAAA": "Bambu_AMS_1", "DDDD": "Bambu_AMS_2"}

    def test_lowering_pool_ams_below_a_recorded_ams_keeps_its_bay(
            self, tmp_path):
        # AAAA pinned to the 4th AMS bay; pool_ams then lowered to 2. The AMS
        # band still covers that bay, so the HT band stays at lane40.
        m = _mk_files(tmp_path, roster="boxed:AAAA", pool_ams=4, pool_ht=8)[0]
        m.cmd_AFC_BRIDGEBOX_ASSIGN(_GCmd(UID="AAAA", NAME="Bambu_AMS_4"))
        m2, p2, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA", pool_ams=2,
                                   pool_ht=8)
        assert self._lanes(p2) == [f"AFC_lane lane{n}"
                                   for n in [*range(24, 28), *range(36, 48)]]
        secs = dict(m2._roster_sections(m2.units))
        assert secs["AFC_BambuAMS Bambu_AMS_4"]["unit_uid"] == "AAAA"
        assert secs["AFC_lane lane24"]["unit"] == "Bambu_AMS_1:1"   # spare
        assert secs["AFC_lane lane40"]["unit"] == "Bambu_AMS_HT_1:1"

    def test_a_renamed_ams_names_entry_redraws_the_unit(self, tmp_path):
        _mk_files(tmp_path, roster="boxed:AAAA", pool_ams=2, pool_ht=0,
                  ams_names="Alpha, Bravo")
        m, p, _a, _s = _mk_files(tmp_path, roster="boxed:AAAA", pool_ams=2,
                                 pool_ht=0, ams_names="Red, Blue")
        assert self._lanes(p) == [f"AFC_lane lane{n}" for n in range(24, 32)]
        assert self._names(m) == {"AAAA": "Red"}
        (note,) = m._layout_notes
        assert "AAAA" in note and "Alpha" in note and "Red" in note


class TestFabricatedSectionsAreVisibleToTheUI:
    """
    A Bambu AMS card showed temperature and no humidity while the ACE 2 card
    beside it showed both -- same sensor_type (aht2x, on the supported list in
    both UIs), same object shape, the humidity sitting right there:

        aht10 Bambu_AMS_1_1  {humidity: 49.0, temperature: 23.0}
        aht10 ace2_temp      {temperature: 28.0, humidity: 39.0}

    Mainsail and Fluidd start from configfile.settings: they read a section's
    sensor_type, match it against their list, and only THEN look up the
    humidity object. Klipper builds those settings out of the access_tracking
    dict handed to each ConfigWrapper -- and the fabrication passed a
    throwaway {}, so every fabricated section was loaded, working and absent
    from settings. There was nothing for the UI to read the type off.
    """

    def _obj(self, cf):
        obj = afcBridgeBox.__new__(afcBridgeBox)
        obj.name = "chain1"
        obj.printer = types.SimpleNamespace(lookup_object=lambda n, d=None: cf)
        obj.logger = types.SimpleNamespace(debug=lambda *a, **k: None)
        return obj

    def test_it_finds_tracking_on_the_validate_object(self):
        # where current Klipper keeps it
        live = {}
        cf = types.SimpleNamespace(validate=types.SimpleNamespace(access_tracking=live))
        assert self._obj(cf)._live_access_tracking() is live

    def test_it_finds_tracking_on_configfile_itself(self):
        # where older Klipper keeps it
        live = {}
        cf = types.SimpleNamespace(access_tracking=live)
        assert self._obj(cf)._live_access_tracking() is live

    def test_a_shape_it_does_not_recognise_is_not_fatal(self):
        # the cost of a miss is what the old code always had, not a dead chain
        for cf in (None, types.SimpleNamespace(), types.SimpleNamespace(validate=None)):
            assert self._obj(cf)._live_access_tracking() == {}

    def test_the_fabrication_shares_it_rather_than_a_throwaway(self):
        # the fabrication lives in __init__, which is where the sections are
        # built and loaded
        src = inspect.getsource(afcBridgeBox.__init__)
        assert "tracking = self._live_access_tracking()" in src
        assert "fileconfig, tracking, section" in src, (
            "the wrapper must be handed the live dict, not {}")
        assert "fileconfig, {}, section" not in src, (
            "a throwaway dict is what made fabricated sections invisible")


# ── assign_pool_tcmd ─────────────────────────────────────────────────────────

class _AfcForTcmd:
    """Just what assign_pool_tcmd reads: registered handlers, CHANGE_TOOL,
    the tool_cmds table and a TcmdAssign that records its calls."""

    def cmd_CHANGE_TOOL(self, gcmd):
        pass

    def __init__(self, registered=()):
        self.tool_cmds = {}
        self.assigned = []
        self.gcode = types.SimpleNamespace(ready_gcode_handlers={})
        for cmd in registered:
            # A fresh bound method each time, as Klipper stores it.
            self.gcode.ready_gcode_handlers[cmd] = self.cmd_CHANGE_TOOL
        self.function = types.SimpleNamespace(
            TcmdAssign=lambda lane: self.assigned.append(lane.name))


def _tcmd_lane(name, maps, current=""):
    return types.SimpleNamespace(name=name, map=list(maps), current_map=current)


def test_a_reclaimed_lane_whose_tcmd_is_ours_is_not_registered_again():
    from extras.AFC_BridgeBox import assign_pool_tcmd
    afc = _AfcForTcmd(registered=("T7",))
    lane = _tcmd_lane("lane7", ["T7"])
    assign_pool_tcmd(lane, afc)
    assert afc.assigned == []
    assert afc.tool_cmds == {"T7": "lane7"}
    assert lane.current_map == "T7"


def test_a_new_lane_goes_through_tcmdassign():
    from extras.AFC_BridgeBox import assign_pool_tcmd
    afc = _AfcForTcmd()
    assign_pool_tcmd(_tcmd_lane("lane7", []), afc)
    assign_pool_tcmd(_tcmd_lane("lane8", ["T8"]), afc)
    assert afc.assigned == ["lane7", "lane8"]


def test_a_tcmd_held_by_another_macro_still_goes_through_tcmdassign():
    from extras.AFC_BridgeBox import assign_pool_tcmd
    afc = _AfcForTcmd()
    afc.gcode.ready_gcode_handlers["T7"] = lambda gcmd: None   # a user macro
    assign_pool_tcmd(_tcmd_lane("lane7", ["T7"]), afc)
    assert afc.assigned == ["lane7"]                # its conflict is reported


class _LevelLogger:
    """AFC's logger, with warnings kept apart from everything else."""

    def __init__(self):
        self.warnings = []
        self.lines = []

    def warning(self, msg):
        self.warnings.append(msg)

    def info(self, msg):
        self.lines.append(msg)

    debug = error = info


class TestAClaimTakesItsHomeTool:
    """A Bambu lane is T<lane number> whichever unit claims first. A lane
    PREP numbered into the Bambu range gives that T# up at the claim, moves
    to a free T# past the Bambu lanes, and the console names both."""

    A = "A" * 24

    def _pool(self, tmp_path, monkeypatch, other_map=None, other=None):
        # One AMS spare on lane24-lane27 (the roster HT sits on lane28), with
        # lane objects that register into afc.lanes the way activation does.
        from extras import AFC_BridgeBox as bb
        tmp_path.mkdir(parents=True, exist_ok=True)
        m, printer, _a, _s = _mk_files(tmp_path, pool_ams=1)
        m.logger = _LevelLogger()
        # PREP registered T24 to CHANGE_TOOL for whichever lane it numbered.
        afc = _AfcForTcmd(registered=("T24",))
        afc.lanes = {}
        afc.saves = 0
        afc.registered = []

        def save_vars():
            afc.saves += 1
        afc.save_vars = save_vars

        def register_tool_macro(lane_name, cmd, rename_map=""):
            afc.registered.append((lane_name, cmd))
            afc.gcode.ready_gcode_handlers[cmd] = afc.cmd_CHANGE_TOOL
        afc.function.register_tool_macro = register_tool_macro
        printer.objects["AFC"] = afc
        bay = next(p for p in m._pool_units if p.get("spare"))
        for lname in bay["lanes"]:
            printer.objects["AFC_lane " + lname] = types.SimpleNamespace(
                name=lname, map=[], _map=[], current_map="", unassigned=True)

        def activate(lane):
            lane.unassigned = False
            afc.lanes[lane.name] = lane

        def deactivate(lane):
            afc.lanes.pop(lane.name, None)
            lane.map, lane.current_map, lane.unassigned = [], "", True
        monkeypatch.setattr(bb, "activate_from_pool", activate)
        monkeypatch.setattr(bb, "deactivate_to_pool", deactivate)
        unit = _LiveUnit(printer, None, None, resolved=False, name=bay["name"])
        unit.pool = True
        unit.set_master = lambda master: None
        unit.claim_ok = True

        def claim(uid, model):
            if not unit.claim_ok:
                return False
            unit.unit_uid, unit.pool = uid, False
            return True
        unit.claim = claim
        unit.release = lambda: setattr(unit, "pool", True)
        printer.objects["AFC_BambuAMS " + bay["name"]] = unit
        if other is None and other_map is not None:
            other = types.SimpleNamespace(
                name="lane5", map=list(other_map), _map=[],
                current_map=other_map[-1] if other_map else "",
                fullname="AFC_stepper lane5")
        if other is not None:
            other.sent = []
            other.send_lane_data = lambda: other.sent.append(list(other.map))
            afc.lanes[other.name] = other
            for t in other.map:
                afc.tool_cmds[t] = other.name
        return m, afc, other, unit, printer

    def test_it_takes_the_tool_and_moves_the_other_lane_past_the_bambu_lanes(
            self, tmp_path, monkeypatch):
        m, afc, other, _u, _p = self._pool(tmp_path, monkeypatch,
                                           other_map=["T24"])
        assert m._claim_pool_unit(self.A, "boxed") is not None
        assert afc.tool_cmds["T24"] == "lane24"
        assert afc.lanes["lane24"].map == ["T24"]
        # lane24-lane28 are the Bambu lanes: lane5 takes the first T# after.
        assert (other.map, other.current_map) == (["T29"], "T29")
        assert afc.tool_cmds["T29"] == "lane5"
        assert afc.registered == [("lane5", "T29")]
        assert other.sent == [["T29"]]       # the panel moves lane5 to T29
        assert afc.saves >= 1
        (warning,) = m.logger.warnings
        for part in ("T24", "lane24", "lane5", "is now T29", "T24-T28",
                     "SET_MAP LANE=lane5"):
            assert part in warning, part
        assert "AFC_RESET_MAPPING" not in warning

    def test_the_saved_map_brings_the_new_tool_back_after_a_restart(
            self, tmp_path, monkeypatch):
        # A real lane, so the var file gets what AFCLane writes for it.
        from extras.AFC_lane import AFCLane
        lane5 = AFCLane.__new__(AFCLane)
        lane5.name, lane5.fullname = "lane5", "AFC_stepper lane5"
        lane5.map, lane5._map, lane5.current_map = ["T24"], [], "T24"
        m, afc, _o, _u, _p = self._pool(tmp_path / "boot1", monkeypatch,
                                        other=lane5)
        saved = []
        afc.save_vars = lambda: saved.append(lane5.map_to_string())
        m._claim_pool_unit(self.A, "boxed")
        assert saved[-1] == "T29"
        # Next boot: PREP puts the saved map back ahead of the config one,
        # and the claim finds T24 free.
        restored = saved[-1].replace(" ", "").split(",")
        m2, afc2, other2, _u2, _p2 = self._pool(tmp_path / "boot2",
                                                monkeypatch,
                                                other_map=restored)
        m2._claim_pool_unit(self.A, "boxed")
        assert afc2.tool_cmds["T24"] == "lane24"
        assert (other2.map, afc2.tool_cmds["T29"]) == (["T29"], "lane5")
        assert m2.logger.warnings == []

    def test_a_config_map_in_the_bambu_range_is_named(self, tmp_path,
                                                      monkeypatch):
        m, afc, other, _u, _p = self._pool(tmp_path, monkeypatch,
                                           other_map=["T24"])
        other._map = ["T24"]
        m._claim_pool_unit(self.A, "boxed")
        assert other.map == ["T29"]
        (warning,) = m.logger.warnings
        for part in ("[AFC_stepper lane5]", "outside T24-T28",
                     "AFC_RESET_MAPPING"):
            assert part in warning, part

    def test_the_new_tool_skips_taken_ones(self, tmp_path, monkeypatch):
        m, afc, other, _u, _p = self._pool(tmp_path, monkeypatch,
                                           other_map=["T24"])
        afc.gcode.ready_gcode_handlers["T29"] = lambda gcmd: None  # a macro
        afc.tool_cmds["T30"] = "lane6"
        afc.lanes["lane6"] = types.SimpleNamespace(name="lane6", map=["T30"],
                                                   _map=["T31"])
        # A CHANGE_TOOL no lane uses is taken as it is, not registered again.
        afc.gcode.ready_gcode_handlers["T32"] = afc.cmd_CHANGE_TOOL
        m._claim_pool_unit(self.A, "boxed")
        assert other.map == ["T32"]
        assert afc.tool_cmds["T32"] == "lane5"
        assert afc.registered == []

    def test_the_other_lane_keeps_its_other_tools(self, tmp_path,
                                                  monkeypatch):
        m, afc, other, _u, _p = self._pool(tmp_path, monkeypatch,
                                           other_map=["T3", "T24"])
        m._claim_pool_unit(self.A, "boxed")
        assert (other.map, other.current_map) == (["T3"], "T3")
        assert afc.tool_cmds["T3"] == "lane5"
        assert afc.tool_cmds["T24"] == "lane24"
        assert afc.registered == []
        (warning,) = m.logger.warnings
        assert "is now T3" in warning
        assert "SET_MAP" not in warning

    def test_a_claim_over_free_tools_is_quiet(self, tmp_path, monkeypatch):
        m, afc, _o, _u, _p = self._pool(tmp_path, monkeypatch)
        assert m._claim_pool_unit(self.A, "boxed") is not None
        assert afc.tool_cmds["T24"] == "lane24"
        assert m.logger.warnings == []

    def test_a_replug_does_not_warn_again(self, tmp_path, monkeypatch):
        m, afc, other, _u, _p = self._pool(tmp_path, monkeypatch,
                                           other_map=["T24"])
        m._claim_pool_unit(self.A, "boxed")
        m._release_pool_unit(self.A)
        assert m._claim_pool_unit(self.A, "boxed") is not None
        assert afc.tool_cmds["T24"] == "lane24"
        assert other.map == ["T29"]
        assert len(m.logger.warnings) == 1

    def test_a_lane_already_on_its_home_tool_is_left_alone(
            self, tmp_path, monkeypatch):
        # Claimed, then claimed again with no release in between: lane24 is
        # still live and still owns T24.
        m, afc, _o, unit, _p = self._pool(tmp_path, monkeypatch)
        m._claim_pool_unit(self.A, "boxed")
        next(p for p in m._pool_units if p.get("bound"))["bound"] = None
        unit.pool = True
        assert m._claim_pool_unit(self.A, "boxed") is unit
        assert afc.tool_cmds["T24"] == "lane24"
        assert m.logger.warnings == []

    def test_a_failed_claim_leaves_the_other_lane_its_tool(
            self, tmp_path, monkeypatch):
        m, afc, other, unit, _p = self._pool(tmp_path, monkeypatch,
                                             other_map=["T24"])
        unit.claim_ok = False
        assert m._claim_pool_unit(self.A, "boxed") is None
        assert afc.tool_cmds["T24"] == "lane5"
        assert other.map == ["T24"]
        assert m.logger.warnings == []

    def test_a_stale_map_does_not_pop_another_lanes_tool(
            self, tmp_path, monkeypatch):
        # lane24 comes back carrying T3 in its map; T3 is lane5's.
        m, afc, other, _u, printer = self._pool(tmp_path, monkeypatch,
                                                other_map=["T3"])
        printer.objects["AFC_lane lane24"].map = ["T3"]
        m._claim_pool_unit(self.A, "boxed")
        assert afc.tool_cmds["T3"] == "lane5"
        assert other.map == ["T3"]
        assert afc.lanes["lane24"].map == ["T24"]


class TestLiveRefineLeavesWhatARestartWould:
    """A live refine, then a re-plug, never runs a unit hotter, or tells an
    index more, than a restart would."""

    SEC = "AFC_BridgeBox chain1"
    A = "A" * 24
    B = "B" * 24
    C = "C" * 24

    def test_replug_after_a_live_refine_keeps_the_ceiling(self, tmp_path):
        # heater: False plus a lower ceiling for ams2; the unit is claimed as
        # boxed, the bus confirms ams2, then it is pulled and re-plugged.
        # claim() turns the heater back on from the model table; the re-plug
        # must still end with the heater off and the 50 a restart would fold
        # in, not boxed's 65.
        fc = _FileConfig({"AFC_BridgeBox ams2": {"heater": "False",
                                                 "dry_max_temp": "50"}})
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc, pool_ams=1)
        m.logger = _Logger()
        for name in [n for n in printer.objects if n.startswith("AFC_lane ")]:
            del printer.objects[name]
        printer.objects["AFC"] = types.SimpleNamespace(tool_cmds={})
        bay = next(p for p in m._pool_units
                   if p.get("spare") and p.get("family") == "ams")
        unit = _LiveUnit(printer, None, None, resolved=False, name=bay["name"])
        unit.pool = True
        unit.set_master = lambda master: None

        def claim(uid, model):
            unit.unit_uid, unit.ams_model, unit.pool = uid, model, False
            unit.has_heater = _AMS_MODELS[model][0]
            return True
        unit.claim = claim
        unit.release = lambda: setattr(unit, "pool", True)
        printer.objects["AFC_BambuAMS " + bay["name"]] = unit
        assert m._claim_pool_unit(self.A, "boxed") is unit
        m._apply_model_live(self.A, "ams2", 0)
        assert unit.has_heater is False and unit.dry_max_temp == 50
        bay["bound"] = None
        bay["uid"] = None
        unit.pool = True
        assert m._claim_pool_unit(self.A, "ams2") is unit
        assert unit.has_heater is False and unit.dry_max_temp == 50

    def test_an_override_is_not_pushed_to_an_index_the_unit_left(
            self, tmp_path, monkeypatch):
        # The snapshot puts A at index 1, but A's object still holds index 0
        # (C's). The ams1 verdict applies measure_on_insert on the object and
        # tells the firmware nothing: adoption re-sends it once re-pinned.
        bridge = _Bridge(uids=[self.C, self.A], online=[True, True],
                         a2mask=0, a2asks=[0, 12])
        fc = _FileConfig({"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        m, printer, _a, _s = _mk_files(tmp_path, roster="boxed:" + self.A,
                                       fileconfig=fc)
        unit = _LiveUnit(printer, bridge, self.A, index=0, resolved=True)
        printer.objects["AFC_BambuAMS Bambu_AMS_1"] = unit
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": "boxed:" + self.A}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m._scout_tick(100.0)
        assert unit.measure_on_insert is True
        assert [c for c in bridge.sent if c.get("cmd") != "chain"] == []

    def test_while_printing_only_the_refined_uid_is_bound_and_nothing_saved(
            self, tmp_path, monkeypatch):
        bridge = _Bridge(uids=[self.A, self.B], online=[True, True],
                         a2mask=0, a2asks=[12, 0])
        roster = f"boxed:{self.A}, ams2:{self.B}"
        m, printer, _a, _s = _mk_files(tmp_path, roster=roster)
        ua = _LiveUnit(printer, bridge, self.A, index=0)
        ub = _LiveUnit(printer, bridge, self.B, index=1, name="Bambu_AMS_2")
        ub.ams_model = "ams2"
        printer.objects["AFC_BambuAMS Bambu_AMS_1"] = ua
        printer.objects["AFC_BambuAMS Bambu_AMS_2"] = ub
        printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": "printing"})
        m.logger = _Logger()
        m._state_set({self.SEC: {"roster": roster}})
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        m._scout_tick(100.0)
        pushed = [c for c in bridge.sent if c.get("cmd") != "chain"]
        assert "idsave" not in [c["cmd"] for c in pushed]
        assert [c["uid"] for c in pushed if c["cmd"] == "bind"] == [self.A]


# ── every claim resolves the claimed model's sections ───────────────────────

class TestEveryClaimResolvesTheClaimedModel:
    """After a claim the unit runs with the heater flag, drying ceiling and
    measure_on_insert of the model it was claimed as, with that model's and
    the unit's sections applied, whatever model the bay was built as."""

    SEC = "AFC_BridgeBox chain1"
    A, B = "A" * 24, "B" * 24

    @staticmethod
    def _on(value):
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _unit(self, printer, name):
        # Built the way the unit reads its fabricated section at boot; claim()
        # and release() do to these fields what the real ones do.
        section = "AFC_BambuAMS " + name
        keys = dict(dict(printer.loaded)[section].fileconfig.items(section))
        model = keys["ams_model"]
        unit = _LiveUnit(printer, None, None, resolved=False, name=name)
        unit.ams_model = model
        unit.has_heater = self._on(keys.get("heater", _AMS_MODELS[model][0]))
        unit.dry_max_temp = int(keys.get("dry_max_temp",
                                         _AMS_MODELS[model][3]))
        unit.measure_on_insert = self._on(keys.get("measure_on_insert", False))
        unit.pool = True
        unit.set_master = lambda master: None
        unit.measure_at_claim = []

        def claim(uid, model):
            unit.measure_at_claim.append(unit.measure_on_insert)
            unit.unit_uid, unit.ams_model, unit.pool = uid, model, False
            unit.has_heater = _AMS_MODELS[model][0]
            return True
        unit.claim = claim
        unit.release = lambda: setattr(unit, "pool", True)
        printer.objects[section] = unit
        return unit

    def _chain(self, tmp_path, monkeypatch=None, bridge=None, fc=None,
               **over):
        m, printer, _a, _s = _mk_files(tmp_path, fileconfig=fc, **over)
        m.logger = _Logger()
        for name in [n for n in printer.objects if n.startswith("AFC_lane ")]:
            del printer.objects[name]
        printer.objects["AFC"] = types.SimpleNamespace(tool_cmds={})
        units = {pu["name"]: self._unit(printer, pu["name"])
                 for pu in m._pool_units}
        if bridge is not None:
            from extras import AFC_BambuAMS_bridge as bridge_mod
            monkeypatch.setattr(bridge_mod, "_BRIDGES",
                                {m.serial_port: bridge}, raising=False)
        return m, units

    @pytest.mark.parametrize("model, section, name, pool", [
        ("ams2", "AFC_BridgeBox ams2", "Bambu_AMS_1", {"pool_ams": 1}),
        ("ams2", "AFC_BridgeBox Bambu_AMS_1", "Bambu_AMS_1", {"pool_ams": 1}),
        ("ht", "AFC_BridgeBox ht", "Bambu_AMS_HT_1", {"pool_ht": 1}),
    ])
    def test_a_same_model_claim_keeps_a_heater_override(
            self, tmp_path, model, section, name, pool):
        # Every bay is claimed live after ready, a rostered one too, and
        # claim() takes the heater from the model table.
        fc = _FileConfig({section: {"heater": "False"}})
        m, units = self._chain(tmp_path, fc=fc, roster=f"{model}:{self.A}",
                               **pool)
        unit = units[name]
        built = (unit.ams_model, unit.has_heater, unit.dry_max_temp)
        assert built[:2] == (model, False)
        assert m._claim_pool_unit(self.A, model) is unit
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == built

    def test_a_claim_does_not_keep_another_models_measure_setting(
            self, tmp_path):
        # An AMS 1 claims the spare and takes its model section's
        # measure_on_insert; an AMS 2 claiming the bay next has no section and
        # gets the spare's default, not the AMS 1's setting.
        fc = _FileConfig({"AFC_BridgeBox ams1": {"measure_on_insert": "True"}})
        m, units = self._chain(tmp_path, fc=fc, pool_ams=1)
        bay = next(p for p in m._pool_units if p["spare"])
        unit = units[bay["name"]]
        assert m._claim_pool_unit(self.A, "ams1") is unit
        assert unit.measure_on_insert is True
        m._release_pool_unit(self.A)
        assert m._claim_pool_unit(self.B, "ams2") is unit
        assert unit.measure_on_insert is False
        # Already in place when each claim() ran: the index adoption the
        # claim starts sends it to the firmware from the attribute.
        assert unit.measure_at_claim == [True, False]

    def test_the_fabricated_measure_default_still_holds(self, tmp_path):
        # No section sets it: a known HT measures on insert, a spare stays
        # off (see the emission site).
        m, units = self._chain(tmp_path, roster="ht:" + self.A, pool_ht=2)
        known = next(p for p in m._pool_units if not p["spare"])
        spare = next(p for p in m._pool_units if p["spare"])
        assert m._claim_pool_unit(self.A, "ht") is units[known["name"]]
        assert m._claim_pool_unit(self.B, "ht") is units[spare["name"]]
        assert units[known["name"]].measure_on_insert is True
        assert units[spare["name"]].measure_on_insert is False

    @pytest.mark.parametrize("recorded", [None, "boxed"])
    def test_the_roster_option_names_the_claimed_model(
            self, tmp_path, monkeypatch, recorded):
        # The bay is built as the option's ams2; the chain alone can only say
        # boxed until the bus confirms the generation.
        bridge = _Bridge(uids=[self.A], online=[True])
        m, units = self._chain(tmp_path, monkeypatch, bridge,
                               roster="ams2:" + self.A, pool_ams=1)
        if recorded:
            m._state_set({self.SEC: {"roster": f"{recorded}:{self.A}"}})
        m._scout_tick(100.0)
        unit = units["Bambu_AMS_1"]
        assert unit.unit_uid == self.A
        assert (unit.ams_model, unit.has_heater) == ("ams2", True)

    def test_a_boxed_option_entry_takes_the_recorded_generation(
            self, tmp_path, monkeypatch):
        # boxed is what the scout writes and an operator copies into roster:.
        # Once the bus has confirmed the generation no further verdict comes
        # for the uid, so the claim takes the recorded one.
        bridge = _Bridge(uids=[self.A], online=[True])
        m, units = self._chain(tmp_path, monkeypatch, bridge,
                               roster="boxed:" + self.A, pool_ams=1)
        m._state_set({self.SEC: {"roster": "ams2:" + self.A}})
        m._scout_tick(100.0)
        unit = units["Bambu_AMS_1"]
        assert (unit.ams_model, unit.has_heater) == ("ams2", True)

    def test_a_refinement_after_the_claim_lands_and_the_reclaim_keeps_it(
            self, tmp_path, monkeypatch):
        # Claimed as the option's boxed, confirmed ams2 in the same tick, then
        # pulled and re-plugged: the re-plug claims ams2, with the ams2
        # section's heater and ceiling.
        bridge = _Bridge(uids=[self.A], online=[True], a2mask=0b1,
                         a2asks=[5])
        fc = _FileConfig({"AFC_BridgeBox ams2": {"heater": "False",
                                                 "dry_max_temp": "50"}})
        m, units = self._chain(tmp_path, monkeypatch, bridge, fc=fc,
                               roster="boxed:" + self.A, pool_ams=1)
        m._state_set({self.SEC: {"roster": "boxed:" + self.A}})
        unit = units["Bambu_AMS_1"]
        m._scout_tick(100.0)
        assert m._state_get(self.SEC, "roster") == "ams2:" + self.A
        assert m.units[0]["model"] == "ams2"
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == \
            ("ams2", False, 50)
        m._release_pool_unit(self.A)
        assert unit.pool is True
        m._scout_tick(101.0)
        assert unit.pool is False
        assert (unit.ams_model, unit.has_heater, unit.dry_max_temp) == \
            ("ams2", False, 50)


# ── a claim during a print leaves the binding table in RAM ───────────────────

class _UnitLogger(_Logger):
    debug = _Logger.info


class _PoolBay(_LiveUnit):
    """An idle pool bay running the unit's real claim path: claim(), the UID
    resolve, the index adoption and the announce it replays. The reactor
    queues callbacks, so the resolve lands after the claim returns, as it
    does on a printer."""

    claim = afcBambuAMS.claim
    set_master = afcBambuAMS.set_master
    _resolve_uid_index = afcBambuAMS._resolve_uid_index
    _match_uid_index = afcBambuAMS._match_uid_index
    _adopt_index = afcBambuAMS._adopt_index
    _announce_unit = afcBambuAMS._announce_unit
    _send_mc_addr = afcBambuAMS._send_mc_addr
    SLOTS_PER_UNIT = 4
    mc_ams_id, mc_id_base, mc_dev_addr = -1, 0, 0x0700

    def __init__(self, printer, bridge, name):
        super().__init__(printer, bridge, None, resolved=False, name=name)
        self.pool = True
        self.lanes = {}
        self.logger = _UnitLogger()
        self._announce_deferred = False
        self._dry_id_follows_index = False
        self.queued = []
        self.afc = types.SimpleNamespace(reactor=types.SimpleNamespace(
            monotonic=lambda: 0.0,
            register_callback=lambda cb, t=0.0: self.queued.append((t, cb)),
            register_timer=lambda cb, t=0.0: None))

    def resolve(self):
        """Run what is due within a second of the claim: the UID resolve."""
        due = [cb for t, cb in self.queued if t < 1.0]
        self.queued = [q for q in self.queued if q[0] >= 1.0]
        for cb in due:
            cb(0.6)


class TestAClaimDuringAPrintHoldsTheSave:
    """A claim pins the new UID in the bridge's binding table and asks for
    the table to be saved. A UID the bridge has not stored before makes that
    save a flash erase that stalls the bus, so during a print the table stays
    in RAM (where it already applies) until the print ends."""

    A = "A" * 24

    def _chain(self, tmp_path, monkeypatch, state):
        m, printer, _a, _s = _mk_files(tmp_path, pool_ams=1)
        m.logger = _Logger()
        for name in [n for n in printer.objects if n.startswith("AFC_lane ")]:
            del printer.objects[name]
        printer.objects["AFC"] = types.SimpleNamespace(tool_cmds={})
        printer.objects["print_stats"] = types.SimpleNamespace(
            get_status=lambda et: {"state": state[0]})
        bay = next(p for p in m._pool_units
                   if p.get("spare") and p.get("family") == "ams")
        bridge = _Bridge(uids=[self.A], online=[True])
        unit = _PoolBay(printer, bridge, bay["name"])
        printer.objects["AFC_BambuAMS " + bay["name"]] = unit
        from extras import AFC_BambuAMS_bridge as bridge_mod
        monkeypatch.setattr(bridge_mod, "_BRIDGES",
                            {m.serial_port: bridge}, raising=False)
        return m, unit, bridge

    @staticmethod
    def _cmds(bridge):
        return [c.get("cmd") for c in bridge.sent]

    def test_a_claim_mid_print_saves_nothing_until_the_print_ends(
            self, tmp_path, monkeypatch):
        state = ["printing"]
        m, unit, bridge = self._chain(tmp_path, monkeypatch, state)
        m._scout_tick(100.0)                  # online: claimed at once
        assert unit.pool is False
        unit.resolve()
        assert unit._id_resolved is True
        bind = {"cmd": "bind", "uid": self.A, "idx": 0, "m": 1}
        assert bind in bridge.sent            # the UID applies from RAM
        assert "idsave" not in self._cmds(bridge)
        m._scout_tick(101.0)                  # still printing
        assert "idsave" not in self._cmds(bridge)
        state[0] = "complete"
        del bridge.sent[:]
        m._scout_tick(102.0)                  # the print is over: saved
        assert self._cmds(bridge).count("idsave") == 1
        assert bridge.sent[-2:] == [bind, {"cmd": "idsave"}]
        del bridge.sent[:]
        m._scout_tick(103.0)                  # once
        assert "idsave" not in self._cmds(bridge)

    def test_a_claim_when_idle_saves_straight_away(self, tmp_path,
                                                   monkeypatch):
        state = ["standby"]
        m, unit, bridge = self._chain(tmp_path, monkeypatch, state)
        m._scout_tick(100.0)
        unit.resolve()
        bind = {"cmd": "bind", "uid": self.A, "idx": 0, "m": 1}
        cmds = self._cmds(bridge)
        assert "idsave" in cmds
        assert bridge.sent.index(bind) < cmds.index("idsave")
        del bridge.sent[:]
        m._scout_tick(101.0)                  # nothing was held
        assert "idsave" not in self._cmds(bridge)
