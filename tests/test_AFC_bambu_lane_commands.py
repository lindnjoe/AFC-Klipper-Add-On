"""
Bambu commands that act on one lane take LANE= alone.

Every AFC_BAMBU_* command is mux'd on UNIT=, and Klipper refuses a mux command
with no key ("missing UNIT") unless a no-key default is registered. The log
lines that tell an operator what to run say ``AFC_BAMBU_SCAN LANE=lane14``, so
on 2026-09-24 that advice failed on printer 1. The commands that act on a lane
now register that default too (_register_lane_command), and it finds the unit
holding the lane (_run_by_lane).

The fake g-code object below reproduces klippy/gcode.py's mux table, including
its refusals, because the behaviour under test IS that table's.
"""

from __future__ import annotations

import inspect
import re
import types

import pytest

from extras.AFC_BambuAMS import (afcBambuAMS, _register_lane_command,
                                 _run_by_lane)


LANE_COMMANDS = (
    "AFC_BAMBU_FOLLOWER", "AFC_BAMBU_PRIME", "AFC_BAMBU_RECOVER",
    "AFC_BAMBU_SCAN", "AFC_BAMBU_REID", "AFC_BAMBU_CAPSCAN", "AFC_BAMBU_FEED",
)

_MISSING = object()


class _Error(Exception):
    pass


class _Gcmd:
    def __init__(self, **params):
        self.params = params

    def get(self, name, default=_MISSING):
        if name in self.params:
            return self.params[name]
        if default is _MISSING:
            raise _Error(f"missing {name}")
        return default

    def error(self, msg):
        return _Error(msg)


class _Gcode:
    """klippy/gcode.py's command and mux tables, refusals included."""

    def __init__(self):
        self.commands = {}
        self.mux_commands = {}

    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if cmd in self.commands:
            raise RuntimeError(f"gcode command {cmd} already registered")
        self.commands[cmd] = func

    def register_mux_command(self, cmd, key, value, func, desc=None):
        prev = self.mux_commands.get(cmd)
        if prev is None:
            self.register_command(
                cmd, lambda gcmd, _c=cmd: self._cmd_mux(_c, gcmd), desc=desc)
            self.mux_commands[cmd] = prev = (key, {})
        prev_key, values = prev
        if prev_key != key:
            raise RuntimeError(f"mux command {cmd} may have only one key")
        if value in values:
            raise RuntimeError(
                f"mux command {cmd} {key} {value} already registered")
        values[value] = func

    def _cmd_mux(self, cmd, gcmd):
        key, values = self.mux_commands[cmd]
        if None in values:
            key_param = gcmd.get(key, None)
        else:
            key_param = gcmd.get(key)
        if key_param not in values:
            raise _Error(f"The value '{key_param}' is not valid for {key}")
        values[key_param](gcmd)

    def run(self, cmd, **params):
        self.commands[cmd](_Gcmd(**params))


class _Printer:
    def __init__(self):
        self.objects = {}

    def lookup_objects(self, module=None):
        prefix = module + " "
        return [(n, o) for n, o in self.objects.items()
                if n.startswith(prefix)]


def _unit(printer, gcode, name, lanes):
    u = types.SimpleNamespace(name=name, gcode=gcode, printer=printer,
                              lanes={ln: object() for ln in lanes}, ran=[])
    printer.objects[f"AFC_BambuAMS {name}"] = u
    return u


def _register(u, cmd="AFC_BAMBU_SCAN"):
    _register_lane_command(
        u, cmd, lambda gcmd, _u=u: _u.ran.append((cmd, gcmd.params)),
        desc="test")


@pytest.fixture
def two_units():
    printer, gcode = _Printer(), _Gcode()
    ams = _unit(printer, gcode, "Bambu_AMS_1",
                ["lane12", "lane13", "lane14", "lane15"])
    ht = _unit(printer, gcode, "Bambu_AMS_HT_1", ["lane28"])
    # Objects that share the prefix without the space, and another unit type
    # holding a lane, must never be picked.
    printer.objects["AFC_BambuAMS_rfid"] = types.SimpleNamespace(
        name="rfid", lanes={"lane28": object()})
    printer.objects["AFC_BoxTurtle Turtle_1"] = types.SimpleNamespace(
        name="Turtle_1", lanes={"lane1": object()})
    _register(ams)
    _register(ht)
    return gcode, ams, ht


def test_lane_alone_runs_the_unit_that_holds_the_lane(two_units):
    gcode, ams, ht = two_units
    gcode.run("AFC_BAMBU_SCAN", LANE="lane14")
    gcode.run("AFC_BAMBU_SCAN", LANE="lane28")
    assert ams.ran == [("AFC_BAMBU_SCAN", {"LANE": "lane14"})]
    assert ht.ran == [("AFC_BAMBU_SCAN", {"LANE": "lane28"})]


def test_unit_still_reaches_its_unit_with_or_without_a_lane(two_units):
    gcode, ams, ht = two_units
    gcode.run("AFC_BAMBU_SCAN", UNIT="Bambu_AMS_1")
    gcode.run("AFC_BAMBU_SCAN", UNIT="Bambu_AMS_HT_1", LANE="lane28")
    assert ams.ran == [("AFC_BAMBU_SCAN", {"UNIT": "Bambu_AMS_1"})]
    assert ht.ran == [("AFC_BAMBU_SCAN",
                       {"UNIT": "Bambu_AMS_HT_1", "LANE": "lane28"})]


def test_a_unit_that_does_not_exist_is_still_refused(two_units):
    gcode, ams, ht = two_units
    with pytest.raises(_Error, match="not valid for UNIT"):
        gcode.run("AFC_BAMBU_SCAN", UNIT="Bambu_AMS_9", LANE="lane14")
    assert ams.ran == [] and ht.ran == []


def test_a_lane_no_bambu_unit_holds_is_refused_naming_the_units(two_units):
    gcode, ams, ht = two_units
    for lane in ("lane99", "lane1"):          # unknown; on a BoxTurtle
        with pytest.raises(_Error) as e:
            gcode.run("AFC_BAMBU_SCAN", LANE=lane)
        assert f"'{lane}'" in str(e.value)
        assert "Bambu_AMS_1, Bambu_AMS_HT_1" in str(e.value)
    assert ams.ran == [] and ht.ran == []


def test_neither_lane_nor_unit_asks_for_the_lane(two_units):
    gcode, ams, ht = two_units
    with pytest.raises(_Error, match=r"needs LANE= naming the lane$"):
        gcode.run("AFC_BAMBU_SCAN")
    assert ams.ran == [] and ht.ran == []


def test_a_bare_scan_offers_unit_and_the_rest_ask_for_the_lane():
    # Only SCAN runs with UNIT= and no LANE= (it then scans the whole unit);
    # the others read LANE with no default, so offering UNIT= alone would send
    # the operator straight into "missing LANE".
    gcode = _Gcode()
    unit = _registering_unit(gcode, "Bambu_AMS_1", _Printer())
    unit.printer.objects["AFC_BambuAMS Bambu_AMS_1"] = unit
    afcBambuAMS._register_gcode_commands(unit)
    for cmd in LANE_COMMANDS:
        with pytest.raises(_Error) as e:
            gcode.run(cmd)
        assert "needs LANE=" in str(e.value)
        assert ("UNIT=" in str(e.value)) == (cmd == "AFC_BAMBU_SCAN"), cmd
        # Moonraker's HTTP API turns an error holding "<" into "Unknown".
        assert "<" not in str(e.value) and ">" not in str(e.value)
    reads = {c: getattr(afcBambuAMS, "cmd_" + c) for c in LANE_COMMANDS}
    optional = {c for c, fn in reads.items()
                if re.search(r"""gcmd\.get\(\s*["']LANE["']\s*,""",
                             inspect.getsource(fn))}
    assert optional == {"AFC_BAMBU_SCAN"}
    assert unit.ran == []


def test_a_lane_released_to_the_pool_is_no_longer_found(two_units):
    # deactivate_to_pool takes the lane out of its unit's lanes.
    gcode, ams, ht = two_units
    del ht.lanes["lane28"]
    with pytest.raises(_Error, match="'lane28'"):
        gcode.run("AFC_BAMBU_SCAN", LANE="lane28")
    assert ht.ran == []


def test_only_units_that_registered_the_command_are_candidates():
    # A unit holding the lane but without this command is not handed a
    # command it never registered.
    printer, gcode = _Printer(), _Gcode()
    a = _unit(printer, gcode, "Bambu_AMS_1", ["lane12"])
    b = _unit(printer, gcode, "Bambu_AMS_2", ["lane16"])
    _register(a, "AFC_BAMBU_PRIME")
    _register(b, "AFC_BAMBU_SCAN")
    with pytest.raises(_Error, match="'lane16'"):
        gcode.run("AFC_BAMBU_PRIME", LANE="lane16")
    gcode.run("AFC_BAMBU_PRIME", LANE="lane12")
    assert a.ran == [("AFC_BAMBU_PRIME", {"LANE": "lane12"})] and b.ran == []


def test_the_default_is_claimed_once_and_later_units_register_cleanly():
    printer, gcode = _Printer(), _Gcode()
    units = [_unit(printer, gcode, f"Bambu_AMS_{i}", [f"lane{i}"])
             for i in range(1, 5)]
    for u in units:
        _register(u)                       # no "already registered" escapes
    key, values = gcode.mux_commands["AFC_BAMBU_SCAN"]
    assert key == "UNIT"
    assert sorted(v for v in values if v is not None) == [
        u.name for u in units]
    assert None in values


def test_a_new_printer_in_the_same_process_gets_the_default_again():
    # Klipper's RESTART builds a new printer and g-code object in the same
    # Python process; nothing module-level may decide it is already done.
    for _ in range(2):
        printer, gcode = _Printer(), _Gcode()
        u = _unit(printer, gcode, "Bambu_AMS_1", ["lane14"])
        _register(u)
        gcode.run("AFC_BAMBU_SCAN", LANE="lane14")
        assert u.ran == [("AFC_BAMBU_SCAN", {"LANE": "lane14"})]


def test_run_by_lane_without_a_printer_refuses_instead_of_crashing():
    with pytest.raises(_Error, match="Bambu units: none"):
        _run_by_lane(None, "AFC_BAMBU_SCAN", _Gcmd(LANE="lane14"))


# ── the unit module's own command table ─────────────────────────────────────

def _registering_unit(gcode, name, printer=None):
    """A stand-in carrying every cmd_ handler afcBambuAMS registers."""
    ns = types.SimpleNamespace(name=name, gcode=gcode, printer=printer,
                               logger=None, lanes={}, ran=[])
    for attr in dir(afcBambuAMS):
        if attr.startswith("cmd_AFC_BAMBU_"):
            setattr(ns, attr, lambda gcmd, _a=attr: ns.ran.append(_a))
    return ns


def _reads_lane(fn):
    return re.search(r"""gcmd\.get\w*\(\s*["']LANE["']""",
                     inspect.getsource(fn)) is not None


def test_every_unit_command_that_takes_a_lane_takes_it_alone():
    gcode = _Gcode()
    afcBambuAMS._register_gcode_commands(_registering_unit(gcode, "A"))
    defaults = {c for c, (_k, v) in gcode.mux_commands.items() if None in v}
    assert defaults == set(LANE_COMMANDS)
    # A command added later that reads LANE= but skips the helper fails here.
    reads = {c for c, (_k, v) in gcode.mux_commands.items()
             if _reads_lane(getattr(afcBambuAMS, "cmd_" + c))}
    assert reads == set(LANE_COMMANDS)


def test_units_on_one_printer_share_the_lane_defaults():
    printer, gcode = _Printer(), _Gcode()
    a = _registering_unit(gcode, "Bambu_AMS_1", printer)
    b = _registering_unit(gcode, "Bambu_AMS_HT_1", printer)
    a.lanes, b.lanes = {"lane14": object()}, {"lane28": object()}
    printer.objects.update({"AFC_BambuAMS Bambu_AMS_1": a,
                            "AFC_BambuAMS Bambu_AMS_HT_1": b})
    afcBambuAMS._register_gcode_commands(a)
    afcBambuAMS._register_gcode_commands(b)
    gcode.run("AFC_BAMBU_CAPSCAN", LANE="lane28")
    gcode.run("AFC_BAMBU_SCAN", LANE="lane14")
    assert a.ran == ["cmd_AFC_BAMBU_SCAN"]
    assert b.ran == ["cmd_AFC_BAMBU_CAPSCAN"]


def test_fault_reload_is_registered_once_per_printer_not_per_process():
    # A module-level "already registered" flag survived a Klipper RESTART and
    # left the new printer without AFC_BAMBU_FAULT_RELOAD, the command every
    # fault pause names (seen on printer 1, 2026-09-24).
    for _ in range(2):
        gcode = _Gcode()
        a = _registering_unit(gcode, "Bambu_AMS_1")
        b = _registering_unit(gcode, "Bambu_AMS_HT_1")
        afcBambuAMS._register_gcode_commands(a)
        afcBambuAMS._register_gcode_commands(b)
        gcode.run("AFC_BAMBU_FAULT_RELOAD")
        assert a.ran == ["cmd_AFC_BAMBU_FAULT_RELOAD"] and b.ran == []
        assert "AFC_BAMBU_UIDS" in gcode.commands
