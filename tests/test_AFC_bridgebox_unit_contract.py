# Armored Turtle Automated Filament Control
#
# Copyright (C) 2024-2026 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""The fabricated-unit contract between AFC_BridgeBox and AFC_BambuAMS.

BridgeBox does not import the unit class it configures. It names the section
as a STRING (``_UNIT_SECTION = "AFC_BambuAMS"``) and hands klippy a key set,
and klippy resolves that prefix to ``extras/AFC_BambuAMS.py`` by filename. So
nothing -- not an import, not a type check, not a linter -- connects the keys
BridgeBox writes to the options the unit reads. Rename an option on either
side and the first thing that notices is an operator's printer failing to
start, or worse, a unit that boots with the setting silently unapplied.

These tests are that missing link.
"""
from __future__ import annotations

import ast
import inspect

import extras.AFC_BambuAMS as unit_mod
import extras.AFC_BambuAMS_bridge as bridge_mod
import extras.AFC_unit as unit_base_mod
from extras.AFC_BridgeBox import afcBridgeBox
from tests.test_AFC_BridgeBox import _mk

#: ConfigWrapper accessors that take an option name as their first argument.
_GETTERS = {"get", "getint", "getfloat", "getboolean",
            "getlist", "getlists", "getchoice"}


def _options_read(module) -> set:
    """
    Every literal option name the module passes to a ``config.get*()``.

    Static, not runtime: an option read on a branch no test exercises still
    counts, which is the point -- this is asking "does the reader know this
    name at all", not "did this run".

    :param module: an imported module object
    :return set: option names appearing as the first argument of a getter
    """
    names = set()
    for node in ast.walk(ast.parse(inspect.getsource(module))):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if not (isinstance(fn, ast.Attribute) and fn.attr in _GETTERS):
            continue
        if not node.args:
            continue
        first = node.args[0]
        if isinstance(first, ast.Constant) and isinstance(first.value, str):
            names.add(first.value)
    return names


def _emitted_unit_keys() -> set:
    """
    Every key BridgeBox writes into a fabricated ``[AFC_BambuAMS ...]``.

    Three models on purpose: ``ht`` and ``ams2`` are heated, so they carry
    heater/dry_max_temp, and ``boxed`` is not -- covering the conditional
    keys as well as the always-emitted ones.

    :return set: the union of fabricated unit-section keys
    """
    keys = set()
    for roster in ("ht:AAAA", "ams2:BBBB", "boxed:CCCC"):
        master, _printer = _mk(roster=roster)
        for section, values in master._roster_sections(master.units):
            if section.startswith(afcBridgeBox._UNIT_SECTION + " "):
                keys |= set(values)
    return keys


class TestFabricatedKeysAreReadByTheUnit:
    def test_every_fabricated_key_is_an_option_the_unit_reads(self):
        # THE CONTRACT. A key BridgeBox emits that the unit never reads is
        # dead config at best; more often it means the option was renamed on
        # the reading side and the fabricator was not updated with it, so
        # every fabricated unit quietly loses that setting.
        emitted = _emitted_unit_keys()
        read = _options_read(unit_mod) | _options_read(unit_base_mod)
        orphaned = sorted(emitted - read)
        assert not orphaned, (
            "AFC_BridgeBox fabricates [%s <name>] keys that AFC_BambuAMS "
            "(and its afcUnit base) never read: %s -- either the option was "
            "renamed on the reading side and the fabricator was not updated, "
            "or the key is dead and should stop being emitted."
            % (afcBridgeBox._UNIT_SECTION, ", ".join(orphaned)))

    def test_the_emitted_set_is_not_accidentally_empty(self):
        # Guards the guard: if _mk or _roster_sections ever stops producing
        # unit sections, the test above would pass vacuously forever.
        emitted = _emitted_unit_keys()
        assert len(emitted) >= 8, emitted
        for expected in ("serial_port", "ams_model", "extruder", "hub"):
            assert expected in emitted, (expected, emitted)


class TestCrossModuleSeam:
    def test_the_fabricated_section_prefix_resolves_to_the_module_checked(self):
        # klippy maps [<prefix> name] to extras/<prefix>.py by FILENAME, so
        # the prefix BridgeBox emits is what decides which module receives
        # these keys. Pin that it is the module whose options we validated.
        assert afcBridgeBox._UNIT_SECTION == "AFC_BambuAMS"
        assert unit_mod.__name__.rsplit(".", 1)[-1] == afcBridgeBox._UNIT_SECTION

    def test_private_symbols_bridgebox_reaches_for_still_exist(self):
        # BridgeBox reaches for these by name at runtime, and four of its six
        # call sites sit inside best-effort `except Exception` blocks --
        # correct for a transient bus failure, but it means a RENAME degrades
        # to "no bridge" / "no buffer chip" with nothing raised. Catch that
        # here instead of on a printer.
        assert hasattr(bridge_mod, "_BRIDGES"), (
            "the bridge registry lives in AFC_BambuAMS_bridge, beside the "
            "BambuBridge class it holds; BridgeBox reads it at 5 sites")
        assert hasattr(unit_mod, "_register_bambu_buffer_chip"), (
            "AFC_BridgeBox imports _register_bambu_buffer_chip from "
            "AFC_BambuAMS to register the scout's buffer chip")

    def test_the_bridge_registry_has_exactly_one_home(self):
        # THE BUG THIS EXISTS FOR. While _BRIDGES lived in AFC_BambuAMS and
        # readers did `from ... import _BRIDGES`, that bound the dict OBJECT:
        # a test rebinding the module attribute was invisible to code holding
        # the old binding, so BridgeBox's scout silently read an empty
        # registry and skipped every enrol/prune tick. One home, reached
        # through the module, is what keeps a rebind honest.
        assert not hasattr(unit_mod, "_BRIDGES"), (
            "_BRIDGES is aliased back onto AFC_BambuAMS -- two names for one "
            "dict means a rebind of one is invisible through the other, which "
            "is the exact failure this move removed")
