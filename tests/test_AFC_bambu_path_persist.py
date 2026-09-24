# The learned PTFE length has to survive a restart, and the obvious way to do
# that HALTS THE PRINTER.
#
# A unit that ConfigRewrites its own key gets it filed under
# [AFC_BambuAMS <name>] in AFC_auto_vars.cfg, which klippy PARSES. A
# pool-fabricated unit has no such section in any .cfg, so the next boot finds
# an orphan and check_unused_options rejects it. That is not hypothetical: an
# earlier Bowden self-measure write halted the printer at the next restart and
# was removed for exactly this reason.
#
# The route these tests pin goes to AFC_BridgeBox's state file instead -- which
# klippy never parses -- filed under the UID of the unit claimed onto the bay,
# so the value follows the physical unit: _fold_and_sweep lays it over that
# unit's section at boot and every claim gives it back (apply_learned).
from __future__ import annotations

import configparser
import types

import pytest

from extras.AFC_BambuAMS import (DEFAULT_BOWDEN_MM, PATH_ADOPT_TOLERANCE_MM,
                                 afcBambuAMS)


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


UID = "0123456789ABCDEF00003331"


def _master(bound=UID, unit_uid=None):
    """A master whose bay Bambu_AMS_HT_1 is bound to ``bound``, and whose unit
    object carries ``unit_uid``; its state writes land in ``m.written``."""
    from extras.AFC_BridgeBox import afcBridgeBox

    m = afcBridgeBox.__new__(afcBridgeBox)
    m.name = "chain1"
    m.logger = types.SimpleNamespace(info=lambda *a, **k: None,
                                     debug=lambda *a, **k: None,
                                     warning=lambda *a, **k: None)
    m.written = {}
    m._read_state = lambda: configparser.RawConfigParser(delimiters=(":", "="))
    m._state_set = lambda upd: m.written.update(upd)
    m._pool_units = [{"name": "Bambu_AMS_HT_1", "bound": bound,
                      "lanes": ["lane24"], "family": "ht"}]
    unit = types.SimpleNamespace(unit_uid=unit_uid)
    m.printer = types.SimpleNamespace(
        lookup_object=lambda n, d=None:
        unit if n == "AFC_BambuAMS Bambu_AMS_HT_1" else d)
    return m


def test_the_master_persists_where_klippy_cannot_parse_it():
    """The real thing: persist_learned writes to the state file, not auto_vars.

    This is the whole point. auto_vars is klippy-parsed config, and a section
    there for a fabricated unit is the orphan that halted the printer.
    """
    from extras.AFC_BridgeBox import afcBridgeBox, _STRUCTURAL_KEYS

    m = _master()
    # DELIBERATELY NOT STUBBING THE SECTION PREFIXES. The first version of this
    # test set _BASE_SECTION = "AFC_BambuAMS" by hand, which made the wrong
    # constant look like the right one and hid the bug completely. The real
    # class attributes are what ship.

    assert m.persist_learned("Bambu_AMS_HT_1", "afc_bowden_length", 3627.0)
    # TIED TO THE READER, not hardcoded: the expected section comes from
    # _learned_section, the call the fold and _apply_learned read through, so
    # a writer and reader that disagree fail here instead of persisting a
    # value nothing reads back. The record is keyed by the uid bound to the
    # bay, so the value goes wherever that unit goes.
    expected = m._learned_section(UID)
    assert list(m.written) == [expected]
    assert m.written[expected] == {"afc_bowden_length": 3627.0}
    assert UID in expected and "Bambu_AMS_HT_1" not in expected
    assert afcBridgeBox._UNIT_SECTION != afcBridgeBox._BASE_SECTION, \
        "the unit prefix and the master's own prefix are different things"
    # And it will not write a structural key whatever a unit asks for.
    for k in list(_STRUCTURAL_KEYS)[:3]:
        assert m.persist_learned("Bambu_AMS_HT_1", k, "x") is False


def test_a_key_outside_the_learned_set_is_refused():
    # Only the bowden lengths are learned. A leftover key saved as learned
    # would ride along with the unit to every bay it claims.
    m = _master()
    assert m.persist_learned("Bambu_AMS_HT_1", "measure_on_insert",
                             "False") is False
    assert m.written == {}


def test_an_unbound_bay_saves_nothing():
    # No unit is on the bay, so there is no uid to file the value under.
    m = _master(bound=None)
    assert m.persist_learned("Bambu_AMS_HT_1", "afc_bowden_length",
                             3627.0) is False
    assert m.written == {}


def test_a_unit_carrying_another_uid_saves_nothing():
    # The bay and its unit object disagree about who is on it: filing the
    # value under either could hand one unit's path to the other.
    m = _master(unit_uid="FFFFFFFFFFFFFFFFFFFFFFFF")
    assert m.persist_learned("Bambu_AMS_HT_1", "afc_bowden_length",
                             3627.0) is False
    assert m.written == {}
    m2 = _master(unit_uid=UID.lower())
    assert m2.persist_learned("Bambu_AMS_HT_1", "afc_bowden_length", 3627.0)


# ── a claim hands the unit its own values and its own measurement ───────────

def _claimed(values, *, bowden=3000.0, master=None):
    u = _unit(bowden=bowden, master=master)
    u.apply_learned(values)
    return u


def test_apply_learned_sets_both_and_rearms_adoption():
    u = _unit(bowden=3627.0)
    u._path_adopted = True
    u.apply_learned({"afc_bowden_length": 1800.0,
                     "afc_unload_bowden_length": 1750.0})
    assert (u.afc_bowden_length, u.afc_unload_bowden_length) == \
        (1800.0, 1750.0)
    assert u._path_adopted is False


def test_apply_learned_with_nothing_is_the_default():
    # A unit with no record starts from the default, not from whatever the
    # bay's previous occupant measured.
    u = _claimed({}, bowden=3627.0)
    assert u.afc_bowden_length == DEFAULT_BOWDEN_MM
    assert u.afc_unload_bowden_length == DEFAULT_BOWDEN_MM


def test_apply_learned_keeps_a_distinct_unload():
    u = _claimed({"afc_bowden_length": 1800.0})
    assert u.afc_unload_bowden_length == 1800.0
    u = _claimed({"afc_bowden_length": 1800.0,
                  "afc_unload_bowden_length": 2000.0})
    assert u.afc_unload_bowden_length == 2000.0


def test_a_fresh_claim_skips_the_pre_load_adoption():
    # The previous occupant's odometer delta, or a figure the bridge still
    # holds for the chain index, reads as a measurement at the top of the new
    # unit's first load. It must not be adopted or saved under the new uid.
    m = _Master()
    u = _claimed({}, master=m)
    u._path_measurement = lambda: (3600.0, "odometer")
    u._adopt_measured_path()
    assert u.afc_bowden_length == DEFAULT_BOWDEN_MM
    assert m.saved == {}
    assert u._path_adopted is False


def test_the_post_load_call_adopts_after_the_flag_clears():
    # _unit_load_lane clears the wait right before the post-load adoption:
    # that measurement is this unit's own completed load.
    m = _Master()
    u = _claimed({}, master=m)
    u._path_wait_load = False
    u._adopt_measured_path(3600.0, "odometer")
    assert u.afc_bowden_length == 3600.0
    assert m.saved[("Bambu_AMS_HT_1", "afc_bowden_length")] == "3600.0"
    u2 = _claimed({}, master=_Master())
    u2._path_wait_load = False
    u2._path_measurement = lambda: (3600.0, "odometer")
    u2._adopt_measured_path()                 # and later loads' pre-load call
    assert u2.afc_bowden_length == 3600.0


def test_apply_learned_clears_the_previous_loads_odometer_delta():
    u = _unit()
    u._load_odom_start, u._load_odom_at_sensor = 10.0, 3610.0
    u.apply_learned({})
    assert u._measure_path_from_odom() is None


def test_the_load_clears_the_wait_right_before_its_post_load_adoption():
    # Pinned in the source because _unit_load_lane cannot run here: the flag
    # must drop after the load's own measurement is taken and before it is
    # adopted, so only this unit's completed load can end the wait.
    import inspect
    src = inspect.getsource(afcBambuAMS._unit_load_lane)
    i_meas = src.index("_mm, _src = self._path_measurement()")
    i_clear = src.index("self._path_wait_load = False")
    i_adopt = src.index("self._adopt_measured_path(_mm, _src)")
    assert i_meas < i_clear < i_adopt
