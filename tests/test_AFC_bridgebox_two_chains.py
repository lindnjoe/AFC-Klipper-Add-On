# Two [AFC_BridgeBox] chains on one printer.
#
# klippy loads sections in config order and registers each object when its
# load function returns. A master's fabricated lanes come from its private
# parser, so a master further down the config cannot see them in the
# fileconfig; it finds them in the printer's object registry instead, and
# its automatic lane_base starts past them. The buffer pin chip is
# printer-wide too, so only the first chain defaults to bambu_buffer and
# every unit a chain fabricates is told its own chain's chip.
from __future__ import annotations

import types

from extras.AFC_BambuAMS import afcBambuAMS
from extras.AFC_BridgeBox import (BridgeBoxOverrideHolder, afcBridgeBox,
                                  load_config_prefix)
from tests.test_AFC_BridgeBox import (_Config, _FakeReactor, _FileConfig,
                                      _Logger, _Printer)


def _master(tmp_path, printer, name, fileconfig=None, register=True, **over):
    """
    Build one chain master the way klippy does, sharing a state file.

    :return: the master (registered on the printer unless register=False)
    """
    opts = {"serial_port": f"/dev/serial/by-id/usb-{name}-if00",
            "extruder": "extruder", "lane_base": 0,
            "roster": "ht:AAAA" if name == "chain1" else "ht:BBBB",
            "pool_ams": 1, "pool_ht": 2,
            "auto_vars_file": str(tmp_path / "AFC_auto_vars.cfg"),
            "state_file": str(tmp_path / "AFC_BridgeBox.cfg")}
    opts.update(over)
    cfg = _Config(opts, printer, name=f"AFC_BridgeBox {name}")
    if fileconfig is not None:
        cfg.fileconfig = fileconfig
    m = load_config_prefix(cfg)
    if register:
        printer.objects[f"AFC_BridgeBox {name}"] = m
    return m


def _lanes(printer, since=0):
    """Lane numbers fabricated onto the printer, from load call `since` on."""
    return [int(s.split("lane")[-1]) for s, _w in printer.loaded[since:]
            if s.startswith("AFC_lane ")]


def _fc():
    return _FileConfig({"AFC_stepper lane1": {}, "AFC_stepper lane4": {}})


def _loaded_keys(printer, section):
    """The keys a fabricated section was loaded with."""
    wrapper = dict(printer.loaded)[section]
    return dict(wrapper.fileconfig.items(section))


# ── _resolve_lane_base ───────────────────────────────────────────────────────

class TestResolveLaneBaseAcrossChains:
    def test_a_second_chain_starts_past_every_lane_the_first_fabricated(
            self, tmp_path):
        printer = _Printer()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=_fc())
        first = _lanes(printer)
        n = len(printer.loaded)
        m2 = _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                     unit_prefix="Bambu_AMS_B")
        second = _lanes(printer, n)
        # lane4 is the highest declared lane. Chain 1: a one-AMS band at
        # 5-8, then an HT band of two at 9-10. Chain 2 repeats the shape
        # from 11: AMS 11-14, HTs 15-16.
        assert m1.lane_base == 5
        assert first == [5, 6, 7, 8, 9, 10]
        assert m2.lane_base == 11
        assert second == [11, 12, 13, 14, 15, 16]
        assert not set(first) & set(second)

    def test_each_chain_keeps_its_own_locked_base(self, tmp_path):
        p1 = _Printer()
        _master(tmp_path, p1, "chain1", fileconfig=_fc())
        _master(tmp_path, p1, "chain2", fileconfig=_fc(),
                unit_prefix="Bambu_AMS_B")
        # Next boot: the config has grown past both chains. Neither base
        # moves, and each is read from the chain's own state section.
        grown = _FileConfig({"AFC_stepper lane1": {}, "AFC_lane lane40": {}})
        p2 = _Printer()
        a = _master(tmp_path, p2, "chain1", fileconfig=grown)
        b = _master(tmp_path, p2, "chain2", fileconfig=grown,
                    unit_prefix="Bambu_AMS_B")
        assert (a.lane_base, b.lane_base) == (5, 11)
        assert a._state_get("AFC_BridgeBox chain1", "lane_base") == "5"
        assert b._state_get("AFC_BridgeBox chain2", "lane_base") == "11"

    def test_named_lane_machine_second_chain_continues_the_sequence(
            self, tmp_path):
        # A toolchanger with tools e0..e3 and no laneN anywhere: chain 1
        # continues at T4 (lanes 4-9), chain 2 past it at 10.
        fc = _FileConfig({f"AFC_extruder e{i}": {} for i in range(4)})
        printer = _Printer()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=fc)
        m2 = _master(tmp_path, printer, "chain2", fileconfig=fc,
                     unit_prefix="Bambu_AMS_B")
        assert m1.lane_base == 4
        assert m2.lane_base == 10

    def test_a_first_chain_with_an_explicit_base_is_stepped_over(
            self, tmp_path):
        # Chain 1 is pinned at 30 (AMS 30-33, HTs 34-35), past every
        # declared lane; the automatic chain 2 starts after it.
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=_fc(), lane_base=30)
        m2 = _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                     unit_prefix="Bambu_AMS_B")
        assert m2.lane_base == 36

    def test_a_single_chain_base_is_unchanged(self, tmp_path):
        printer = _Printer()
        m = _master(tmp_path, printer, "chain1", fileconfig=_fc())
        assert m.lane_base == 5
        assert _lanes(printer) == [5, 6, 7, 8, 9, 10]

    def test_a_printer_that_cannot_list_objects_keeps_the_config_base(
            self, tmp_path):
        class _Blind(_Printer):
            def lookup_objects(self, module=None):
                raise RuntimeError("no registry")

        printer = _Blind()
        m = _master(tmp_path, printer, "chain1", fileconfig=_fc())
        assert m.lane_base == 5
        assert m.buffer_chip_name == "bambu_buffer"


# ── _loaded_lane_numbers ─────────────────────────────────────────────────────

class TestLoadedLaneNumbers:
    def test_numbered_lanes_and_steppers_only(self, tmp_path):
        printer = _Printer()
        m = _master(tmp_path, printer, "chain1", register=False, pool_ams=0,
                    pool_ht=0, lane_base=24)
        printer.objects = {"AFC_lane lane3": object(),
                           "AFC_lane e0": object(),
                           "AFC_stepper lane7": object(),
                           "AFC_stepper Lane12": object(),
                           "AFC_hub lane9": object()}
        assert sorted(m._loaded_lane_numbers()) == [3, 7, 12]

    def test_an_unlistable_registry_yields_nothing(self, tmp_path):
        printer = _Printer()
        m = _master(tmp_path, printer, "chain1", register=False, pool_ams=0,
                    pool_ht=0, lane_base=24)
        m.printer = object()
        assert m._loaded_lane_numbers() == []


# ── _earlier_chains ──────────────────────────────────────────────────────────

class TestEarlierChains:
    def test_only_masters_count_not_override_holders(self, tmp_path):
        printer = _Printer()
        printer.objects["AFC_BridgeBox ht"] = load_config_prefix(_Config(
            {"measure_on_insert": "False"}, printer,
            name="AFC_BridgeBox ht"))
        m1 = _master(tmp_path, printer, "chain1", fileconfig=_fc())
        m2 = _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                     register=False, unit_prefix="Bambu_AMS_B")
        assert isinstance(printer.objects["AFC_BridgeBox ht"],
                          BridgeBoxOverrideHolder)
        assert m2._earlier_chains() == [m1]

    def test_an_unlistable_registry_yields_nothing(self, tmp_path):
        printer = _Printer()
        m = _master(tmp_path, printer, "chain1", register=False, pool_ams=0,
                    pool_ht=0, lane_base=24)
        m.printer = object()
        assert m._earlier_chains() == []


# ── __init__: each chain's buffer chip ───────────────────────────────────────

class TestBufferChipPerChain:
    def test_the_first_chain_keeps_the_plain_default(self, tmp_path):
        printer = _Printer()
        printer.objects["AFC_BridgeBox ht"] = load_config_prefix(_Config(
            {"measure_on_insert": "False"}, printer,
            name="AFC_BridgeBox ht"))
        m = _master(tmp_path, printer, "chain1", fileconfig=_fc())
        assert m.buffer_chip_name == "bambu_buffer"
        assert _loaded_keys(printer, "AFC_buffer Bambu_AMS_Buffer")[
            "adc_pin"] == "bambu_buffer:fps"

    def test_a_second_chain_defaults_to_a_chip_of_its_own(self, tmp_path):
        printer = _Printer()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=_fc())
        m2 = _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                     unit_prefix="Bambu_AMS_B")
        assert m1.buffer_chip_name == "bambu_buffer"
        assert m2.buffer_chip_name == "bambu_buffer_chain2"
        assert _loaded_keys(printer, "AFC_buffer Bambu_AMS_B_Buffer")[
            "adc_pin"] == "bambu_buffer_chain2:fps"

    def test_every_unit_is_told_its_chains_chip(self, tmp_path):
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=_fc())
        _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                unit_prefix="Bambu_AMS_B")
        chips = {s.split()[1]: _loaded_keys(printer, s)["buffer_chip_name"]
                 for s, _w in printer.loaded
                 if s.startswith("AFC_BambuAMS ")}
        assert chips == {
            "Bambu_AMS_1": "bambu_buffer",
            "Bambu_AMS_HT_1": "bambu_buffer",
            "Bambu_AMS_HT_2": "bambu_buffer",
            "Bambu_AMS_B_1": "bambu_buffer_chain2",
            "Bambu_AMS_B_HT_1": "bambu_buffer_chain2",
            "Bambu_AMS_B_HT_2": "bambu_buffer_chain2",
        }

    def test_an_explicit_chip_name_still_wins(self, tmp_path):
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=_fc())
        m2 = _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                     unit_prefix="Bambu_AMS_B", buffer_chip_name="pico_b")
        assert m2.buffer_chip_name == "pico_b"

    def test_two_scouting_chains_register_two_chips(self, tmp_path):
        # With no roster and no pool a master registers its chip itself,
        # under the same per-chain name its units would use.
        class _Pins:
            def __init__(self):
                self.chips = {}

            def register_chip(self, name, chip):
                self.chips[name] = chip

        pins = _Pins()
        printer = _Printer({"pins": pins})
        for name in ("chain1", "chain2"):
            _master(tmp_path, printer, name, fileconfig=_fc(), roster="",
                    pool_ams=0, pool_ht=0)
        assert sorted(pins.chips) == ["bambu_buffer", "bambu_buffer_chain2"]

    def test_a_buffer_on_the_plain_chip_below_a_scout_is_adopted(
            self, tmp_path):
        # chain1 only watches its bridge, so bambu_buffer is its stub chip:
        # the hand-written buffer on it belongs to chain2, as it did when
        # every chain defaulted to bambu_buffer, and chain2 fabricates none.
        fc = _FileConfig({"AFC_stepper lane4": {},
                          "AFC_buffer Bambu_AMS_Buffer":
                              {"adc_pin": "bambu_buffer:fps"}})
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=fc, roster="",
                pool_ams=0, pool_ht=0)
        m2 = _master(tmp_path, printer, "chain2", fileconfig=fc, roster="",
                     pool_ams=2, pool_ht=1)
        assert m2.buffer_chip_name == "bambu_buffer"
        assert m2.buffer == "Bambu_AMS_Buffer"
        assert not any(s.startswith("AFC_buffer ") for s, _w in
                       printer.loaded)
        assert {_loaded_keys(printer, s)["buffer_chip_name"]
                for s, _w in printer.loaded
                if s.startswith("AFC_BambuAMS ")} == {"bambu_buffer"}

    def test_a_chain_below_a_real_chain_keeps_its_own_chip(self, tmp_path):
        fc = _FileConfig({"AFC_stepper lane4": {},
                          "AFC_buffer Hand": {"adc_pin": "bambu_buffer:fps"}})
        printer = _Printer()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=fc)
        m2 = _master(tmp_path, printer, "chain2", fileconfig=fc,
                     unit_prefix="Bambu_AMS_B")
        assert m1.buffer == "Hand"
        assert m2.buffer_chip_name == "bambu_buffer_chain2"
        assert m2.buffer == "Bambu_AMS_B_Buffer"

    def test_the_scout_stub_chip_reads_the_first_unit_on_it(self):
        from extras.AFC_BambuAMS import _register_bambu_buffer_chip

        class _Pins:
            def register_chip(self, name, chip):
                pass

        printer = _Printer({"pins": _Pins()})

        def _unit(**kw):
            return types.SimpleNamespace(printer=printer,
                                         buffer_chip_name="bambu_buffer",
                                         **kw)
        stub, unit, later = _unit(scout_stub=True), _unit(), _unit()
        _register_bambu_buffer_chip(stub)
        chip = printer._bambu_buffer_chips["bambu_buffer"]
        assert chip._unit is stub
        _register_bambu_buffer_chip(_unit(scout_stub=True))
        assert chip._unit is stub
        _register_bambu_buffer_chip(unit)
        _register_bambu_buffer_chip(later)
        assert printer._bambu_buffer_chips == {"bambu_buffer": chip}
        assert chip._unit is unit

    def test_an_override_section_cannot_rewire_one_units_chip(self, tmp_path):
        # The chip is chain wiring, like buffer: a per-unit override leaves
        # the unit on the chip its chain's buffer reads.
        fc = _FileConfig({"AFC_stepper lane4": {},
                          "AFC_BridgeBox Bambu_AMS_HT_1":
                              {"buffer_chip_name": "elsewhere"}})
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=fc)
        assert _loaded_keys(printer, "AFC_BambuAMS Bambu_AMS_HT_1")[
            "buffer_chip_name"] == "bambu_buffer"


# ── _fold_and_sweep: auto_vars shared by two chains ──────────────────────────

class TestSharedAutoVars:
    """Both chains fold from one AFC_auto_vars.cfg, and the second has not
    fabricated its units while the first loads."""

    AUTOV = ("[AFC_BambuAMS Bambu_AMS_B_HT_1]\n"
             "afc_bowden_length : 3632.0\n\n"
             "[AFC_BambuAMS Gone]\n"
             "afc_bowden_length : 1500.0\n")

    def _fc(self):
        """The merged config: both masters, an override holder, and the
        auto_vars sections klippy parsed."""
        return _FileConfig({
            "AFC_stepper lane1": {}, "AFC_stepper lane4": {},
            "AFC_BridgeBox chain1": {
                "serial_port": "/dev/serial/by-id/usb-chain1-if00"},
            "AFC_BridgeBox ht": {"measure_on_insert": "False"},
            "AFC_BridgeBox chain2": {
                "serial_port": "/dev/serial/by-id/usb-chain2-if00"},
            "AFC_BambuAMS Bambu_AMS_B_HT_1": {"afc_bowden_length": "3632.0"},
            "AFC_BambuAMS Gone": {"afc_bowden_length": "1500.0"}})

    def test_the_second_chain_folds_its_own_sections(self, tmp_path):
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text(self.AUTOV)
        # What chain2 learned for BBBB is kept under chain2's name.
        seed = afcBridgeBox.__new__(afcBridgeBox)
        seed.state_file = str(tmp_path / "AFC_BridgeBox.cfg")
        seed._state_set({"AFC_BridgeBox chain2 learned BBBB":
                         {"afc_unload_bowden_length": "999.0"}})
        fc = self._fc()
        printer = _Printer()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=fc)
        text = autov.read_text()
        assert "Bambu_AMS_B_HT_1" in text and "[AFC_BambuAMS Gone]" in text
        assert m1._learned_for("BBBB") == {}
        m2 = _master(tmp_path, printer, "chain2", fileconfig=fc,
                     unit_prefix="Bambu_AMS_B")
        keys = _loaded_keys(printer, "AFC_BambuAMS Bambu_AMS_B_HT_1")
        assert keys["afc_bowden_length"] == "3632.0"
        assert keys["afc_unload_bowden_length"] == "999.0"
        assert m2._learned_for("BBBB") == {
            "afc_bowden_length": "3632.0",
            "afc_unload_bowden_length": "999.0"}
        # The last chain sweeps: its own section folded, the orphan gone.
        text = autov.read_text()
        assert "Bambu_AMS_B_HT_1" not in text and "Gone" not in text

    def test_a_last_chain_that_fabricates_nothing_sweeps(self, tmp_path):
        # chain2 only watches its bridge (no roster, no pool), and it is the
        # last master: it sweeps, so the orphan does not come back at every
        # boot as an offline unit.
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text(self.AUTOV)
        fc = self._fc()
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=fc)
        assert "[AFC_BambuAMS Gone]" in autov.read_text()
        m2 = _master(tmp_path, printer, "chain2", fileconfig=fc,
                     unit_prefix="Bambu_AMS_B", roster="", pool_ams=0,
                     pool_ht=0)
        assert m2._roster_source == "scout" and m2.units == []
        assert "Gone" not in autov.read_text()

    def test_a_first_chain_that_fabricates_nothing_leaves_it_to_the_last(
            self, tmp_path):
        autov = tmp_path / "AFC_auto_vars.cfg"
        autov.write_text(self.AUTOV)
        fc = self._fc()
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=fc, roster="",
                pool_ams=0, pool_ht=0)
        assert autov.read_text() == self.AUTOV
        _master(tmp_path, printer, "chain2", fileconfig=fc,
                unit_prefix="Bambu_AMS_B")
        keys = _loaded_keys(printer, "AFC_BambuAMS Bambu_AMS_B_HT_1")
        assert keys["afc_bowden_length"] == "3632.0"
        text = autov.read_text()
        assert "Bambu_AMS_B_HT_1" not in text and "Gone" not in text

    def test_later_chains_are_the_masters_not_loaded_yet(self, tmp_path):
        fc = self._fc()
        printer = _Printer()
        cfg = _Config({}, printer)
        cfg.fileconfig = fc
        m1 = _master(tmp_path, printer, "chain1", fileconfig=fc)
        assert m1._later_chains(cfg) == ["chain2"]
        m2 = _master(tmp_path, printer, "chain2", fileconfig=fc,
                     unit_prefix="Bambu_AMS_B")
        assert m2._later_chains(cfg) == []
        assert m1._later_chains(cfg) == []


# ── the AMS band a recorded HT holds ─────────────────────────────────────────

class TestAStartAnotherChainStopped:
    """chain1 saves its maps while it loads, and chain2 can still stop that
    start: the HT band chain1 holds is the one a start that reached ready
    built."""

    @staticmethod
    def _boot(tmp_path, **over):
        """
        Load both chains and, when both load, run klippy:ready.

        :return tuple: (chain1, chain2 or the error that stopped it)
        """
        printer = _Printer()
        printer.objects["AFC"] = types.SimpleNamespace(logger=_Logger())
        printer.get_reactor = lambda: _FakeReactor()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=_fc(), **over)
        try:
            m2 = _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                         unit_prefix="Bambu_AMS_B")
        except Exception as e:
            return m1, e
        m1._scout_ready()
        m2._scout_ready()
        return m1, m2

    def test_reverting_the_change_that_stopped_it_starts_again(
            self, tmp_path):
        m1, m2 = self._boot(tmp_path)
        assert m1._lane_map["AAAA"] == (9, 1) and m2.lane_base == 11
        # pool_ams 2 moves chain1's HT onto lane13, inside chain2's lanes.
        m1, err = self._boot(tmp_path, pool_ams=2)
        assert m1._lane_map["AAAA"] == (13, 1)
        assert str(err) == (
            f"[AFC_BridgeBox chain2] would fabricate [AFC_lane lane11], but "
            f"[AFC_BridgeBox chain1] above it in the config already builds "
            f"it: this chain's lane_base 11 (saved in the #~# block of "
            f"{tmp_path / 'AFC_BridgeBox.cfg'}, used while lane_base: is "
            f"unset or 0) falls inside that chain's lanes. Set lane_base: in "
            f"one chain's section past the other chain's last lane.")
        for _again in range(2):
            m1, m2 = self._boot(tmp_path)
            assert isinstance(m2, afcBridgeBox)
            assert m1._lane_map["AAAA"] == (9, 1)


    def test_a_lane_base_given_in_the_section_is_named(self, tmp_path):
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=_fc(), lane_base=24)
        try:
            _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                    unit_prefix="Bambu_AMS_B", lane_base=26)
        except Exception as e:
            err = str(e)
        assert ("would fabricate [AFC_lane lane26], but [AFC_BridgeBox "
                "chain1] above it in the config already builds it: this "
                "chain's lane_base 26 (set by lane_base: in this section)"
                in err)

    def test_a_unit_name_both_chains_build_names_unit_prefix(self, tmp_path):
        printer = _Printer()
        _master(tmp_path, printer, "chain1", fileconfig=_fc())
        try:
            _master(tmp_path, printer, "chain2", fileconfig=_fc(),
                    roster="ht:AAAA")
        except Exception as e:
            err = str(e)
        assert err.startswith(
            "[AFC_BridgeBox chain2] would fabricate [AFC_BambuAMS "
            "Bambu_AMS_1], but [AFC_BridgeBox chain1] above it in the config "
            "already builds it. Give one chain its own unit_prefix")

    def test_a_section_of_the_config_is_still_named_as_one(self, tmp_path):
        printer = _Printer()
        printer.objects["AFC_hub Bambu_AMS_1"] = object()
        try:
            _master(tmp_path, printer, "chain1", fileconfig=_fc())
        except Exception as e:
            err = str(e)
        assert err == ("[AFC_BridgeBox chain1] would fabricate [AFC_hub "
                       "Bambu_AMS_1] but it already exists in the config -- "
                       "remove one")


class TestAHeldHtBandMakesWayForALaterChain:
    """An HT recorded past fewer AMS bays than chain1 now builds keeps its
    lanes -- unless a chain further down the config builds one of them,
    which would stop Klipper from starting. Then the HT follows the band
    chain1 builds, as it did before the hold existed."""

    def _fc(self, tmp_path, **chain2):
        opts = {"serial_port": "/dev/serial/by-id/usb-chain2-if00",
                "state_file": str(tmp_path / "AFC_BridgeBox.cfg"),
                "pool_ams": "2", "pool_ht": "0"}
        opts.update({k: str(v) for k, v in chain2.items()})
        return _FileConfig({
            "AFC_BridgeBox chain1": {
                "serial_port": "/dev/serial/by-id/usb-chain1-if00"},
            "AFC_BridgeBox chain2": opts})

    def _boot(self, tmp_path, stored=None, **chain2):
        """chain1 recorded HT HHHH on lane40 behind four AMS bays and now
        builds one; chain2 below it builds two AMS bays."""
        seed = afcBridgeBox.__new__(afcBridgeBox)
        seed.state_file = str(tmp_path / "AFC_BridgeBox.cfg")
        seed._state_set({"AFC_BridgeBox chain1": {
            "roster": "ht:HHHH", "lane_base": "24", "ams_band": "4",
            "lane_map": "HHHH:40:1", "name_map": "HHHH:Bambu_AMS_HT_1"}})
        if stored:
            seed._state_set({"AFC_BridgeBox chain2": {"lane_base": stored}})
        fc = self._fc(tmp_path, **chain2)
        printer = _Printer()
        m1 = _master(tmp_path, printer, "chain1", fileconfig=fc, roster="",
                     lane_base=24, pool_ams=1, pool_ht=2)
        notes = list(m1._layout_notes)
        m2 = _master(tmp_path, printer, "chain2", fileconfig=fc, roster="",
                     unit_prefix="Bambu_AMS_B", pool_ams=2, pool_ht=0,
                     lane_base=int(chain2.get("lane_base", 0)))
        return m1, m2, notes

    def test_an_explicit_lane_base_over_the_ht_lane_moves_it(self, tmp_path):
        m1, m2, notes = self._boot(tmp_path, lane_base=36)
        assert m1._lane_map["HHHH"] == (28, 1)
        assert m2.lane_base == 36
        assert notes[0] == (
            "AFC_BridgeBox chain1: HT HHHH (Bambu_AMS_HT_1) on lane40 (T40) "
            "cannot keep its lanes: [AFC_BridgeBox chain2] further down the "
            "config builds lane40. The AMS band is 1 bays, what pool_ams and "
            "the recorded AMS need, and the HT lanes follow it.")

    def test_a_saved_lane_base_over_the_ht_lane_moves_it(self, tmp_path):
        m1, m2, _notes = self._boot(tmp_path, stored="36")
        assert m1._lane_map["HHHH"] == (28, 1)
        assert m2.lane_base == 36

    def test_a_chain_clear_of_the_ht_lane_leaves_it(self, tmp_path):
        m1, m2, _notes = self._boot(tmp_path, lane_base=28)
        assert m1._lane_map["HHHH"] == (40, 1)
        assert m2.lane_base == 28

    def test_a_chain_with_its_base_still_to_compute_leaves_it(self,
                                                             tmp_path):
        m1, m2, _notes = self._boot(tmp_path)
        assert m1._lane_map["HHHH"] == (40, 1)
        assert m2.lane_base == 42


# ── AFC_BambuAMS.__init__: the unit registers the chip it is told ────────────

def _unit_config(**values):
    from tests.conftest import MockConfig

    class _C(MockConfig):
        def getchoice(self, option, choices, default=None, **kw):
            return self._require(option, default)

    return _C(name="AFC_BambuAMS Bambu_AMS_B_1",
              values=dict({"serial_port": "/dev/fake"}, **values))


class TestAfcBambuAMSInitBufferChip:
    def test_a_named_chip_is_registered_under_that_name(self):
        u = afcBambuAMS(_unit_config(buffer_chip_name="bambu_buffer_chain2"))
        assert u.buffer_chip_name == "bambu_buffer_chain2"
        assert list(u.printer._bambu_buffer_chips) == ["bambu_buffer_chain2"]

    def test_no_name_registers_the_plain_default(self):
        u = afcBambuAMS(_unit_config())
        assert u.buffer_chip_name == "bambu_buffer"
        assert list(u.printer._bambu_buffer_chips) == ["bambu_buffer"]
