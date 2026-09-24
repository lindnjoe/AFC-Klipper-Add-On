# The learned PTFE length has to survive a restart, and the obvious way to do
# that HALTS THE PRINTER.
#
# A unit that ConfigRewrites its own key gets it filed under
# [AFC_BambuAMS <name>] in AFC_auto_vars.cfg, which klippy PARSES. A
# pool-fabricated unit has no such section in any .cfg, so the next boot finds
# an orphan and check_unused_options rejects it. That is not hypothetical:
# 4e2497058, "Bowden self-measure halted the printer at the next restart",
# removed the write for exactly this reason.
#
# The route these tests pin goes to AFC_BridgeBox's state file instead -- which
# klippy never parses -- and relies on _fold_and_sweep, which already overlays
# store sections onto the unit they belong to at boot.
from __future__ import annotations

import configparser
import types

import pytest

from extras.AFC_BambuAMS import afcBambuAMS, PATH_ADOPT_TOLERANCE_MM


class _Master:
    """Records what a unit asks to persist, the way AFC_BridgeBox would."""

    def __init__(self):
        self.saved = {}
        self.refused = []

    def persist_learned(self, unit, key, value):
        # Mirrors the real guard: structural keys define what a unit IS.
        if key in ("serial_port", "ams_model", "unit_uid"):
            self.refused.append(key)
            return False
        if self.saved.get((unit, key)) == str(value):
            return False
        self.saved[(unit, key)] = str(value)
        return True


def _unit(*, bowden=3000.0, master=None):
    u = afcBambuAMS.__new__(afcBambuAMS)
    u.name = "Bambu_AMS_HT_1"
    u.afc_bowden_length = bowden
    u.afc_unload_bowden_length = bowden
    u._path_adopted = False
    u.logger = types.SimpleNamespace(info=lambda *a, **k: None,
                                     debug=lambda *a, **k: None,
                                     warning=lambda *a, **k: None)
    if master is not None:
        u.set_master(master)
    return u


def test_a_measured_path_is_persisted_through_the_master():
    m = _Master()
    u = _unit(master=m)
    u._adopt_measured_path(3627.0, "odometer")
    assert u.afc_bowden_length == 3627.0, "adopted in memory"
    assert m.saved[("Bambu_AMS_HT_1", "afc_bowden_length")] == "3627.0"


def test_the_unload_length_follows_only_when_it_was_tracking():
    m = _Master()
    u = _unit(master=m)
    u._adopt_measured_path(3627.0, "odometer")
    assert ("Bambu_AMS_HT_1", "afc_unload_bowden_length") in m.saved

    m2 = _Master()
    u2 = _unit(master=m2)
    u2.afc_unload_bowden_length = 1234.0        # set deliberately by an operator
    u2._adopt_measured_path(3627.0, "odometer")
    assert u2.afc_unload_bowden_length == 1234.0, "an explicit value is kept"
    assert ("Bambu_AMS_HT_1", "afc_unload_bowden_length") not in m2.saved


def test_a_unit_with_no_master_still_adopts():
    # A statically configured unit, or a test. Persistence is an optimisation
    # for the next session's first load, never a correctness requirement.
    u = _unit(master=None)
    u._adopt_measured_path(3627.0, "odometer")
    assert u.afc_bowden_length == 3627.0


def test_a_master_that_raises_does_not_break_the_load():
    class _Broken:
        def persist_learned(self, *a, **k):
            raise RuntimeError("state file is read-only")
    u = _unit(master=_Broken())
    u._adopt_measured_path(3627.0, "odometer")
    assert u.afc_bowden_length == 3627.0, "the load must not care"


def test_a_measurement_within_tolerance_writes_nothing():
    # The figure wobbles a few mm between calibrations; rewriting the state
    # file on every boot for that would be churn.
    m = _Master()
    u = _unit(bowden=3627.0, master=m)
    u._adopt_measured_path(3627.0 + PATH_ADOPT_TOLERANCE_MM / 2.0, "odometer")
    assert m.saved == {}


def test_it_adopts_once_per_session():
    m = _Master()
    u = _unit(master=m)
    u._adopt_measured_path(3627.0, "odometer")
    u._adopt_measured_path(9999.0, "odometer")
    assert u.afc_bowden_length == 3627.0


def test_structural_keys_are_refused():
    # A learned value must never be able to redefine what a unit IS.
    m = _Master()
    assert m.persist_learned("Bambu_AMS_HT_1", "serial_port", "/dev/evil") is False
    assert "serial_port" in m.refused


def test_the_master_persists_where_klippy_cannot_parse_it():
    """The real thing: persist_learned writes to the state file, not auto_vars.

    This is the whole point. auto_vars is klippy-parsed config, and a section
    there for a fabricated unit is the orphan that halted the printer.
    """
    from extras.AFC_BridgeBox import afcBridgeBox, _STRUCTURAL_KEYS

    m = afcBridgeBox.__new__(afcBridgeBox)
    m.name = "chain1"
    m.logger = types.SimpleNamespace(info=lambda *a, **k: None,
                                     debug=lambda *a, **k: None,
                                     warning=lambda *a, **k: None)
    written = {}
    m._read_state = lambda: configparser.RawConfigParser(delimiters=(":", "="))
    m._state_set = lambda upd: written.update(upd)
    # DELIBERATELY NOT STUBBING THE SECTION PREFIXES. The first version of this
    # test set _BASE_SECTION = "AFC_BambuAMS" by hand, which made the wrong
    # constant look like the right one and hid the bug completely. The real
    # class attributes are what ship.

    assert m.persist_learned("Bambu_AMS_HT_1", "afc_bowden_length", 3627.0)
    # TIED TO THE FABRICATOR, not hardcoded. The first version of this used
    # _BASE_SECTION -- which names the MASTER's own section, not its units --
    # so it wrote [AFC_BridgeBox Bambu_AMS_HT_1]. That persisted perfectly and
    # _fold_and_sweep, which matches on the fabricated unit name, never looked
    # at it. Silently write-only. Caught on hardware, not by the test that
    # hardcoded the string it expected.
    expected = "%s Bambu_AMS_HT_1" % afcBridgeBox._UNIT_SECTION
    assert list(written) == [expected]
    assert written[expected] == {"afc_bowden_length": 3627.0}
    assert afcBridgeBox._UNIT_SECTION != afcBridgeBox._BASE_SECTION, \
        "the unit prefix and the master's own prefix are different things"
    # And it will not write a structural key whatever a unit asks for.
    for k in list(_STRUCTURAL_KEYS)[:3]:
        assert m.persist_learned("Bambu_AMS_HT_1", k, "x") is False
