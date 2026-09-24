"""
Unit tests for the AFC_BambuAMS / AFC_BambuAMS_rfid split.

The Bambu AMS reads its own tags, so unlike the ViViD/OpenAMS/ACE2 readers
there is no host-side reader to detach. What splits out is everything
DOWNSTREAM of the tag: the Spoolman lookup/bind/sync, the UID binding memos,
and the physical remaining weight the AMS measures by radius. Scanning and tag
application stay on the unit, because on this hardware the tag IS the slot
status.

What these tests pin:
  - the seam itself: which names live where, so a later edit cannot quietly
    drag Spoolman back into the unit module
  - the unit runs with NO [AFC_BambuAMS_rfid] section: every Spoolman shim
    no-ops, and an object that never ran __init__ still reads as "no Spoolman"
    instead of raising
  - the MEASUREMENT is not Spoolman work and lands without the section, with
    it disabled, and with AFC_RFID missing, through a Spoolman-off delegate --
    and the split state (the percent, the held summary, the bound tag) is read
    and cleared where it actually lives
  - with the section, the shims actually reach the delegate
  - a measurement is applied once, at the time it is taken: the stamp a
    restart or a reconnect finds is recorded and never applied, the only
    follow-up is the spool a later Spoolman bind attaches, and remain_pct is
    read back out of the lane's grams
  - lane hygiene (unbind, tare restore, cross-unit UID dedup) stayed behind --
    it is not Spoolman work and must keep running without it
  - a restored lane with no spool has its bay's tag looked up once per
    connection, match-only, only on a record that describes the lane; an
    empty bay's record is never put back on its lane; and a spool put in
    during a print waits on lane defaults for its read (printer 1's lane15)
"""

from __future__ import annotations

import contextlib
import inspect
import types

import pytest

import extras.AFC_BambuAMS as core
import extras.AFC_BambuAMS_rfid as rfid
from extras.AFC_BambuAMS_rfid import BambuSpoolman


SPOOLMAN_METHODS = ('_spoolman_sync', '_spoolman_slot_info', '_spoolman_bg',
                    '_bind_by_uid_bg', '_remember_bound_uid',
                    '_forget_spoolman_miss',
                    '_apply_remain_weight', '_push_measured_to_spoolman',
                    '_adopt_measured_remain', '_queue_spool_summary',
                    '_drain_spool_summary', '_say_spool_summary')

#: Lane/tag hygiene that is NOT Spoolman work: clearing a departed spool's
#: link, putting the configured tare back, and refusing a UID another unit
#: already claims. These have to keep working with Spoolman switched off.
CORE_METHODS = ('_unbind_spool', '_restore_config_tare',
                '_uid_claimed_elsewhere', '_finalize_scan',
                '_surface_slot_info', '_maybe_auto_scan')

#: AFC_RFID's Spoolman half. The unit module must not name any of it.
SPOOLMAN_SYMBOLS = ('sync_rfid_to_spoolman', 'find_spool_by_uid',
                    'SpoolmanClient', '_spool_uids', '_norm_uid',
                    '_bambu_spoolman_client')


# ── the seam ─────────────────────────────────────────────────────────────────

def test_the_unit_module_names_no_spoolman_symbol():
    leaked = [n for n in SPOOLMAN_SYMBOLS if hasattr(core, n)]
    assert not leaked, f"Spoolman leaked back into AFC_BambuAMS: {leaked}"


def test_the_delegate_owns_every_spoolman_method():
    missing = [m for m in SPOOLMAN_METHODS if not hasattr(BambuSpoolman, m)]
    assert not missing, f"not on the delegate: {missing}"


def test_lane_hygiene_stayed_on_the_unit():
    missing = [m for m in CORE_METHODS if not hasattr(core.afcBambuAMS, m)]
    assert not missing, f"lost from the unit: {missing}"
    # ...and did NOT also get copied into the delegate.
    dupes = [m for m in ('_unbind_spool', '_restore_config_tare',
                         '_uid_claimed_elsewhere')
             if hasattr(BambuSpoolman, m)]
    assert not dupes, f"lane hygiene duplicated onto the delegate: {dupes}"


def test_the_worker_queue_is_shared_by_every_unit():
    # One Spoolman thread for the whole printer, not one per AMS: the jobs are
    # HTTP calls kept off the reactor and they serialize fine.
    assert 'BambuSpoolman' in rfid.__dict__
    assert hasattr(BambuSpoolman, '_spool_q')
    assert hasattr(BambuSpoolman, '_spool_t')


# ── with no [AFC_BambuAMS_rfid] section ──────────────────────────────────────

def _bare_unit():
    """A unit that never ran __init__ -- a subclass, or anything built with
    __new__. It must still read as "no Spoolman" rather than raising."""
    return core.afcBambuAMS.__new__(core.afcBambuAMS)


def test_a_unit_that_skipped_init_reads_as_no_spoolman():
    u = _bare_unit()
    u.printer = types.SimpleNamespace(lookup_object=lambda n, d=None: d)
    assert u._spool is None


def test_every_spoolman_shim_no_ops_without_the_section():
    """
    The SPOOLMAN shims only. The measurement shims (_adopt_measured_remain,
    _drain_spool_summary) are left out on purpose: they no longer no-op
    without the section. A measured percent becoming the lane's grams is not
    Spoolman work, so they go to a Spoolman-off delegate and act -- which is
    what the "measurement lands" tests below pin. Asserting here that they
    leave the lane alone would be asserting the bug back in.
    """
    u = _bare_unit()
    u.printer = types.SimpleNamespace(lookup_object=lambda n, d=None: d)
    lane = types.SimpleNamespace(name="lane4", spool_id=7, weight=100)
    # None of these may raise, and none may touch the lane.
    u._spoolman_sync(lane, {"index": 0})
    u._apply_remain_weight(lane, {"index": 0})
    u._forget_spoolman_miss(0)
    u._bind_by_uid_bg(lane, 0, "deadbeef", "")
    assert lane.spool_id == 7 and lane.weight == 100
    # ...and none of them builds the measurement delegate on the way: the
    # Spoolman side has no business creating measurement state.
    assert u._meas_obj is None


def test_the_lookup_happens_once_even_when_it_finds_nothing():
    calls = []
    u = _bare_unit()
    u.printer = types.SimpleNamespace(
        lookup_object=lambda n, d=None: calls.append(n) or d)
    assert u._spool is None
    assert u._spool is None
    assert calls == ['AFC_BambuAMS_rfid'], \
        "the miss must be latched -- this runs on every status pass"


def test_a_broken_lookup_is_survived():
    u = _bare_unit()
    u.logger = types.SimpleNamespace(debug=lambda *a, **k: None)
    u.name = "AMS"

    def _boom(name, default=None):
        raise RuntimeError("printer objects not ready")

    u.printer = types.SimpleNamespace(lookup_object=_boom)
    assert u._spool is None


# ── with the section ─────────────────────────────────────────────────────────

def _wired_unit():
    """A unit with an [AFC_BambuAMS_rfid] object configured."""
    u = _bare_unit()
    obj = rfid.load_config(_Config())
    u.printer = types.SimpleNamespace(
        lookup_object=lambda n, d=None: obj if n == 'AFC_BambuAMS_rfid' else d)
    return u, obj


def test_the_delegate_is_built_and_bound_to_its_unit():
    u, _obj = _wired_unit()
    sp = u._spool
    assert isinstance(sp, BambuSpoolman)
    assert sp._u is u, "the delegate must reach back to its own unit"


def test_the_delegate_is_built_once_per_unit():
    u, _obj = _wired_unit()
    assert u._spool is u._spool


def test_the_delegate_starts_with_empty_memos():
    u, _obj = _wired_unit()
    sp = u._spool
    assert sp._spoolman_no_match == set()
    assert sp._spoolman_inflight == set()
    assert sp._bound_uid == {} and sp._binding_check == {}
    assert sp._bind_owed == {} and sp._measured_remain == {}
    assert sp._convert_owed == {} and sp._pending_summary == {}


def test_a_shim_reaches_the_delegate():
    u, _obj = _wired_unit()
    seen = []
    u._spool._forget_spoolman_miss = lambda slot: seen.append(slot)
    u._forget_spoolman_miss(3)
    assert seen == [3]


def test_two_units_get_their_own_memos():
    # Memos are per-unit state; sharing them would let one AMS's miss suppress
    # another's lookup.
    u1, obj = _wired_unit()
    u2 = _bare_unit()
    u2.printer = u1.printer
    assert u1._spool is not u2._spool
    assert u1._spool._spoolman_no_match is not u2._spool._spoolman_no_match


# ── the Klipper object ───────────────────────────────────────────────────────

class _Config:
    """Config stub that records which options were READ."""

    def __init__(self, **opts):
        self.opts = opts
        self.reads = []

    def get_printer(self):
        return object()

    def get_name(self):
        return "AFC_BambuAMS_rfid"

    def getboolean(self, name, default=None):
        self.reads.append(name)
        return self.opts.get(name, default)


def test_the_section_reads_at_least_one_option():
    # NOT a style rule -- Klipper's configfile.check_unused rejects the section
    # without it. It builds valid_sections from printer.lookup_objects() (keyed
    # by the section name in its ORIGINAL case) plus access_tracking (keyed
    # LOWERCASED), then looks up the lowercased name. A mixed-case section
    # therefore only matches via access_tracking, which is populated by
    # config.get*() calls -- get_printer()/get_name() do not count.
    #
    # With no option read, [AFC_BambuAMS_rfid] was rejected as "not a valid
    # config section" even though the module imported fine, and it halted two
    # printers. Every sibling reader survives because it has real options:
    # AFC_BoxTurtle_rfid reads `bus`, AFC_U1_rfid reads `lane_channels`,
    # AFC_rfid_keys reads its key names.
    cfg = _Config()
    rfid.load_config(cfg)
    assert cfg.reads, \
        "the section must read an option or Klipper rejects it outright"


def test_load_config_builds_the_object():
    obj = rfid.load_config(_Config())
    assert isinstance(obj, rfid.AFC_BambuAMS_RFID)
    assert obj.enabled is True


def test_for_unit_hands_back_a_delegate_for_that_unit():
    obj = rfid.load_config(_Config())
    unit = object()
    sp = obj.for_unit(unit)
    assert isinstance(sp, BambuSpoolman) and sp._u is unit


def test_enabled_false_keeps_the_section_but_switches_spoolman_off():
    obj = rfid.load_config(_Config(enabled=False))
    assert obj.enabled is False
    assert obj.for_unit(object()) is None


def test_a_disabled_section_leaves_the_unit_with_no_delegate():
    u = _bare_unit()
    obj = rfid.load_config(_Config(enabled=False))
    u.printer = types.SimpleNamespace(
        lookup_object=lambda n, d=None: obj if n == 'AFC_BambuAMS_rfid' else d)
    assert u._spool is None
    lane = types.SimpleNamespace(name="lane4", spool_id=7, weight=100)
    u._spoolman_sync(lane, {"index": 0})
    assert lane.spool_id == 7


def test_get_status_makes_the_object_visible():
    # Klipper's objects/list webhook filters to objects with get_status, so
    # without this the section loads fine and is still invisible to the API --
    # which is how a working deploy got mistaken for a broken one.
    obj = rfid.load_config(_Config())
    st = obj.get_status(0.0)
    assert st == {"enabled": True, "units": []}


def test_get_status_names_the_units_that_wired_up():
    obj = rfid.load_config(_Config())
    obj.for_unit(types.SimpleNamespace(name="Bambu_AMS_1"))
    obj.for_unit(types.SimpleNamespace(name="Bambu_AMS_HT_1"))
    assert obj.get_status()["units"] == ["Bambu_AMS_1", "Bambu_AMS_HT_1"]


def test_get_status_reports_a_disabled_section():
    obj = rfid.load_config(_Config(enabled=False))
    assert obj.get_status()["enabled"] is False
    obj.for_unit(types.SimpleNamespace(name="Bambu_AMS_1"))
    assert obj.get_status()["units"] == [], \
        "a disabled section must not claim units it never served"


# ── core must run with NO RFID module deployed at all ────────────────────────

@contextlib.contextmanager
def _without_afc_rfid():
    """Import the unit module AND this module with extras.AFC_RFID unimportable.

    AFC_RFID is not in the first official release, so "works without RFID"
    means the file is ABSENT -- not merely that Spoolman is switched off. This
    reproduces that by blocking the import and re-importing both modules, and
    keeps that world in place for the body of the ``with``: the unit reaches
    the rfid module lazily (its measurement delegate is imported on first
    use), so a test that measures has to run while the fresh modules are the
    ones sys.modules hands out. Everything is restored on exit so the rest of
    the suite is unaffected.

    :yields tuple: (the fresh extras.AFC_BambuAMS, the fresh
        extras.AFC_BambuAMS_rfid)
    """
    import importlib
    import sys

    class _Blocker:
        def find_spec(self, fullname, path=None, target=None):
            if fullname == "extras.AFC_RFID":
                raise ImportError("AFC_RFID is not deployed")
            return None

    import extras
    names = ("extras.AFC_BambuAMS", "extras.AFC_RFID",
             "extras.AFC_BambuAMS_rfid")
    saved = {n: sys.modules[n] for n in names if n in sys.modules}
    # A submodule import rebinds the parent package's attribute too
    # (extras.AFC_BambuAMS = <module>), so snapshot those as well.
    saved_attrs = {n: getattr(extras, n.rsplit(".", 1)[1])
                   for n in names if hasattr(extras, n.rsplit(".", 1)[1])}
    for n in names:
        sys.modules.pop(n, None)
    blocker = _Blocker()
    sys.meta_path.insert(0, blocker)
    try:
        core_mod = importlib.import_module("extras.AFC_BambuAMS")
        rfid_mod = importlib.import_module("extras.AFC_BambuAMS_rfid")
        yield core_mod, rfid_mod
    finally:
        sys.meta_path.remove(blocker)
        for n in names:
            sys.modules.pop(n, None)
        sys.modules.update(saved)
        # Restoring sys.modules alone is NOT enough: the fresh import above
        # rebound extras.AFC_BambuAMS on the package to the throwaway module.
        # Leaving it there splits `import extras.AFC_BambuAMS as x` (reads the
        # package attribute) from `from extras.AFC_BambuAMS import y` (reads
        # sys.modules) onto two different module objects -- which left
        # AFC_BridgeBox's scout reading an empty _BRIDGES and silently skipping
        # every enrol/prune tick. Put the original attributes back too.
        for n, mod in saved_attrs.items():
            setattr(extras, n.rsplit(".", 1)[1], mod)


def _import_core_without_afc_rfid():
    """Import extras.AFC_BambuAMS with extras.AFC_RFID made unimportable.

    See _without_afc_rfid; this is the one-shot form, for tests that only
    need the unit module's own module-level fallbacks.
    """
    with _without_afc_rfid() as (core_mod, _rfid_mod):
        return core_mod


def test_the_unit_module_imports_with_no_afc_rfid_present():
    mod = _import_core_without_afc_rfid()
    assert mod.apply_filament_defaults is not None
    assert mod.build_filament_name is not None


def test_lane_data_is_still_applied_with_no_afc_rfid_present():
    # The whole point: a bay must still show its filament in Mainsail on a
    # printer that never deployed AFC_RFID.
    mod = _import_core_without_afc_rfid()
    lane = types.SimpleNamespace(name="lane4", material=None, color=None,
                                 extruder_temp=None, bed_temp=None,
                                 sub_type="", spool_vendor="", weight=0)
    mod.apply_filament_defaults(lane, {
        "material": "PLA", "sub_type": "Matte", "brand": "Bambu",
        "color_hex": "0086D6", "extruder_temp": 220, "bed_temp": 60,
    })
    assert lane.material == "PLA"
    assert lane.color == "#0086D6"
    assert lane.extruder_temp == 220.0
    assert lane.bed_temp == 60.0
    assert lane.sub_type == "Matte"
    assert lane.spool_vendor == "Bambu"
    assert lane.weight == 1000


def test_the_fallback_name_matches_the_real_one():
    mod = _import_core_without_afc_rfid()
    from extras.AFC_RFID import build_filament_name as real
    for args in (("Bambu", "PLA", "Basic"), ("Bambu", "PLA", "PLA Matte"),
                 ("", "PETG", ""), ("Bambu", "", ""), ("", "", "")):
        assert mod.build_filament_name(*args) == real(*args), args


def test_the_fallback_only_fills_blanks():
    # Never overwrite what the operator or an earlier read already set.
    mod = _import_core_without_afc_rfid()
    lane = types.SimpleNamespace(name="lane4", material="ABS", color="#112233",
                                 extruder_temp=250.0, bed_temp=100.0,
                                 sub_type="Custom", spool_vendor="Mine",
                                 weight=750)
    mod.apply_filament_defaults(lane, {
        "material": "PLA", "sub_type": "Matte", "brand": "Bambu",
        "color_hex": "0086D6", "extruder_temp": 220, "bed_temp": 60,
    })
    assert (lane.material, lane.color, lane.extruder_temp, lane.bed_temp,
            lane.sub_type, lane.spool_vendor, lane.weight) == (
        "ABS", "#112233", 250.0, 100.0, "Custom", "Mine", 750)


# ── the capacity sample row ──────────────────────────────────────────────────
#
# capsample is the raw record the mass model gets fitted against (see
# _log_capacity_sample's own docstring). Two things have to be true of every
# row or the dataset is worse than empty:
#
#   - a radius belongs to the percent printed beside it. The unit narrates
#     both from one cycle, so agreeing on the percent is how a row proves it
#     is one measurement and not two stitched together.
#   - a row says where its radius came from, because they are not equally
#     direct: the narration is the line itself, the slot record is the
#     firmware's stamp of it, and "none" is an honest gap.
#
# The gap these cover was live on 2026-09-20: a boxed unit narrated R:0.094
# with 144% at 13:15, Klipper restarted, and the re-adoption of that same 144%
# at 13:26 logged no radius at all -- a fresh BambuBridge has an empty
# narration cache. Taking whatever the cache held instead would have been
# worse: after a restart it pairs a percent with some earlier spool's radius,
# and nothing downstream can tell that row from a real one.

def _sample_unit(slots, rec=None):
    """A delegate whose capsample line is captured, over given slots/cache."""
    u, _obj = _wired_unit()
    lines = []
    u.logger = types.SimpleNamespace(debug=lambda msg: lines.append(msg))
    u.name = "AMS"
    u.dry_dev_addr = 0x0700
    u._slots = slots
    u._bridge = types.SimpleNamespace(last_cap_measure=lambda addr: rec)
    return u, lines


def _field(line, key):
    for tok in line.split():
        if tok.startswith(key + "="):
            return tok[len(key) + 1:]
    return None


def test_the_narrated_radius_is_used_when_its_percent_agrees():
    u, lines = _sample_unit(
        [{"index": 0, "meas_pct": 144, "meas_radius_mm": 94}],
        rec={"pct_raw": 144, "radius_m": 0.094, "circumference_m": 0.592,
             "save_radius_m": 0.093643})
    u._spool._log_capacity_sample(0, None, 144, 1000, 1000, "capscan")
    assert _field(lines[0], "radius_src") == "narration"
    # The unrounded radius rides along: a calibration from the CHANGE in
    # radius across a print cannot work from a millimetre-rounded figure.
    assert _field(lines[0], "save_r_m") == "0.093643"
    assert _field(lines[0], "radius_m") == "0.094"
    # Only the narration states a circumference; it is the live line.
    assert _field(lines[0], "circ_m") == "0.592"


def test_a_stale_narration_is_refused_and_the_row_says_so():
    # The cache holds the PREVIOUS spool's measurement. Pairing its radius
    # with this percent would look exactly like a real row.
    u, lines = _sample_unit(
        [{"index": 0}],
        rec={"pct_raw": 60, "radius_m": 0.071, "circumference_m": 0.446})
    u._spool._log_capacity_sample(0, None, 144, 1000, 1000, "capscan")
    assert _field(lines[0], "radius_m") == "None"
    assert _field(lines[0], "radius_src") == "stale(rec=60)"


def test_the_slot_record_supplies_the_radius_after_a_restart():
    # No narration cache at all -- a restart built a fresh bridge -- but the
    # firmware stamped the radius onto the bay beside the percent (AFC-2.72).
    u, lines = _sample_unit(
        [{"index": 0, "meas_pct": 144, "meas_radius_mm": 94}], rec=None)
    u._spool._log_capacity_sample(0, None, 144, 1000, 1000,
                                  "physical AMS measurement")
    assert _field(lines[0], "radius_src") == "slotrec"
    assert _field(lines[0], "radius_m") == "0.094"
    # The row names what produced it: a re-adoption is not a fresh pull, and
    # a fit should be able to drop the duplicates.
    assert "source='physical AMS measurement'" in lines[0]


def test_a_stale_slot_record_is_refused_too():
    # A boxed unit is known to advance meas_seq while leaving meas_pct behind,
    # so the record's own percent has to agree before its radius is believed.
    u, lines = _sample_unit(
        [{"index": 0, "meas_pct": 127, "meas_radius_mm": 88}], rec=None)
    u._spool._log_capacity_sample(0, None, 144, 1000, 1000, "capscan")
    assert _field(lines[0], "radius_m") == "None"
    assert _field(lines[0], "radius_src") == "none"


def test_older_firmware_leaves_an_honest_gap():
    # No mrad in the status frame: the row records the measurement and says
    # the radius was never stated, rather than inventing one.
    u, lines = _sample_unit([{"index": 0, "meas_pct": 144}], rec=None)
    u._spool._log_capacity_sample(0, None, 144, 1000, 1000, "capscan")
    assert _field(lines[0], "radius_src") == "none"


def test_the_bridge_slot_map_carries_the_radius():
    info = core.bridge_slot_to_info({"i": 0, "mpct": 144, "mrad": 94})
    assert info["meas_radius_mm"] == 94
    # 0 is the firmware's "the line stated none", not a radius of zero.
    assert core.bridge_slot_to_info({"i": 0, "mrad": 0})["meas_radius_mm"] \
        is None


# ── two tags, one reel ───────────────────────────────────────────────────────
#
# A Bambu spool carries an RFID tag on EACH flange, and the AMS reads whichever
# faces its reader. The 4-byte chip UIDs differ; the 16-byte tray_uid does not.
# Measured on 2026-09-20 by moving three reels between bays:
#
#   orange ABS   7392020a / c32a080a  ->  Spoolman 163 + 150
#   grey Matte   d34e4e39 / 13f56d32  ->  Spoolman 132 + 124
#   blue Basic   4b8e44f6 / 95f2c30c  ->  Spoolman 136 + 141
#
# Every reel already had TWO records, and its consumption had been splitting
# between them by which way round it went into the bay. So the roll identity is
# looked up first and the chip UID second, and whichever answers teaches the
# record the other half.

class _FakeClient:
    """Records what the bind wrote, and answers the two lookups."""

    def __init__(self, by_tray=None, by_uid=None):
        self._by_tray = by_tray
        self._by_uid = by_uid
        self.uid_writes = []
        self.tray_writes = []

    def write_spool_metadata(self, spool_id, uid=None, lot_nr=None):
        self.uid_writes.append((spool_id, uid))

    def write_tray_uid(self, spool_id, tray_uid):
        self.tray_writes.append((spool_id, tray_uid))


def _bind_unit(monkeypatch, client, lane_spool=None):
    """A wired unit whose bind runs inline instead of on the worker thread."""
    u, _obj = _wired_unit()
    u.name = "AMS"
    u.logger = types.SimpleNamespace(debug=lambda *a, **k: None,
                                     info=lambda *a, **k: None)
    reactor = types.SimpleNamespace(
        register_async_callback=lambda cb: cb(),
        monotonic=lambda: 0.0)
    bound = []
    u.afc = types.SimpleNamespace(
        reactor=reactor, spoolman=object(), moonraker=object(),
        spool=types.SimpleNamespace(
            set_spoolID=lambda lane, sid: bound.append(sid)))
    monkeypatch.setattr(rfid, "_bambu_spoolman_client", lambda afc: client)
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: job())
    # The shared matcher both the host readers and this path go through:
    # the roll first, the chip UID second.
    monkeypatch.setattr(
        rfid, "match_spool_for_tag",
        lambda c, u_, t="": ((c._by_tray, True, None) if (t and c._by_tray)
                             else (c._by_uid, False, None)))
    lane = types.SimpleNamespace(name="lane14", spool_id=lane_spool, weight=0)
    return u, lane, bound


TRAY = "cf34cf1d212f46b5bc8561e05eb644c8"


def test_the_roll_is_looked_up_before_the_tag(monkeypatch):
    # The other face of a known reel: the chip UID is a stranger, the tray UID
    # is not. Without this the reel binds to nothing and Spoolman grows a
    # second record for it.
    client = _FakeClient(by_tray={"id": 132, "extra":
                                  {"card_uids": '"D34E4E39"'}}, by_uid=None)
    u, lane, bound = _bind_unit(monkeypatch, client)
    u._spool._bind_by_uid_bg(lane, 2, "13f56d32", "", tray_uid=TRAY)
    assert bound == [132]
    # ...and the record is taught this face, so the next flip matches at once.
    assert client.uid_writes == [(132, "13f56d32")]
    assert client.tray_writes == []       # it already had the roll identity


def test_a_chip_match_stamps_the_roll_identity_on(monkeypatch):
    # The face we already know. Nothing to learn about the UID, but the record
    # carries no tray UID yet -- stamp it, or the OTHER face is still a
    # stranger the next time this reel goes in.
    client = _FakeClient(by_tray=None,
                         by_uid={"id": 132, "extra": {}})
    u, lane, bound = _bind_unit(monkeypatch, client)
    u._spool._bind_by_uid_bg(lane, 2, "d34e4e39", "", tray_uid=TRAY)
    assert bound == [132]
    assert client.tray_writes == [(132, TRAY)]


def test_a_conflicting_roll_identity_is_left_alone(monkeypatch):
    # The record already names a DIFFERENT roll. Overwriting would bury the
    # contradiction; the record describes another reel and someone should see
    # that rather than have it quietly repointed.
    client = _FakeClient(by_tray=None,
                         by_uid={"id": 132,
                                 "extra": {"tray_uid": '"4e3177c3"'}})
    u, lane, bound = _bind_unit(monkeypatch, client)
    u._spool._bind_by_uid_bg(lane, 2, "d34e4e39", "", tray_uid=TRAY)
    assert bound == [132]
    assert client.tray_writes == []


def test_a_tagless_brand_still_binds_by_chip_uid(monkeypatch):
    # Nothing but Bambu is known to write a tray UID, and every other reader
    # keys off the chip UID. No tray UID must cost nothing.
    client = _FakeClient(by_tray=None, by_uid={"id": 77, "extra": {}})
    u, lane, bound = _bind_unit(monkeypatch, client)
    u._spool._bind_by_uid_bg(lane, 2, "aabbccdd", "", tray_uid="")
    assert bound == [77]
    assert client.tray_writes == []
    assert client.uid_writes == [(77, "aabbccdd")]


# ── grams are a MASS now, not the percent read as one ────────────────────────
#
# The AMS percent is a volume ratio -- cross-sectional area against a fixed
# reference geometry -- so reading it as a mass fraction is right only for a
# material whose density matches the reference, and wrong in the direction that
# over-reports. Three reels weighed on 2026-09-20 (gross minus a 260 g spool):
#
#     orange ABS  141%, rho 1.04  ->  983 g   model 972   tag-linear 1000
#     blue PLA     82%, rho 1.24  ->  663 g   model 674   tag-linear  820
#     grey Matte   87%, rho 1.32  ->  806 g   model 761   tag-linear  870
#
# The model is within 2% on the two reels whose density is not in doubt; the
# tag-linear figure was 24% out on the blue.

def _grams_unit(material="PLA", density=None, rec=None, slots=None):
    u, _obj = _wired_unit()
    u.name = "AMS"
    u.dry_dev_addr = 0x0700
    u.logger = types.SimpleNamespace(debug=lambda *a, **k: None,
                                     info=lambda *a, **k: None)
    u._slots = slots if slots is not None else [{"index": 0,
                                                 "material": material}]
    u._bridge = types.SimpleNamespace(last_cap_measure=lambda addr: rec)
    lane = types.SimpleNamespace(name="lane1", material=material,
                                 density=density, weight=0)
    return u, lane


def test_grams_follow_the_density_not_the_tag():
    # The blue reel: 82% of a 1 kg spool is 820 g only if PLA weighs what the
    # reference geometry assumes. It held 663.
    u, lane = _grams_unit("PLA")
    assert u._spool._grams_for(0, lane, 82, 1000) == 674


def test_a_denser_material_reads_heavier_at_the_same_percent():
    # Same geometry, more mass. ABS is the mirror image -- less dense, so a
    # full reel reads PROUD of 100% and the percent alone would invent
    # filament that is not there.
    u, lane = _grams_unit("ABS")
    assert u._spool._grams_for(0, lane, 141, 1000) == 972
    u2, lane2 = _grams_unit("PLA")
    assert u2._spool._grams_for(0, lane2, 141, 1000) == 1000   # capped


def test_the_tag_weight_is_still_the_ceiling():
    # A reading proud of the reference geometry is a full reel sitting
    # slightly large, not more filament than the spool was sold with.
    u, lane = _grams_unit("ABS")
    assert u._spool._grams_for(0, lane, 160, 1000) == 1000
    assert u._spool._grams_for(0, lane, 160, 750) == 750


def test_the_unrounded_radius_beats_the_integer_percent():
    # The percent quantises the answer to ~5 g; the "odom save" line states
    # the radius to six decimals. Same geometry, better resolution.
    rec = {"pct_raw": 82, "radius_m": 0.077, "save_radius_m": 0.077357}
    u, lane = _grams_unit("PLA", rec=rec)
    with_r = u._spool._grams_for(0, lane, 82, 1000)
    u2, lane2 = _grams_unit("PLA")
    without_r = u2._spool._grams_for(0, lane2, 82, 1000)
    assert with_r != without_r
    # R 77.357 mm against hub 47.5 / full 82.6 is 81.6%, not 82.
    assert with_r == 671


def test_a_stale_record_is_not_used_for_the_radius():
    # The record must describe THIS measurement -- same rule the capsample row
    # follows. A radius from another cycle is worse than no radius.
    rec = {"pct_raw": 60, "radius_m": 0.071, "save_radius_m": 0.071200}
    u, lane = _grams_unit("PLA", rec=rec)
    assert u._spool._grams_for(0, lane, 82, 1000) == 674   # the percent's own


def test_an_unknown_material_keeps_the_old_answer():
    # A density we do not have is not a number to guess at: fall back to the
    # tag-linear figure rather than invent a material.
    u, lane = _grams_unit("", slots=[{"index": 0}])
    lane.material = None
    assert u._spool._grams_for(0, lane, 82, 1000) == 820


def test_the_lane_density_wins_over_the_table():
    # Spoolman's figure is the filament's own; the table is the fallback.
    u, lane = _grams_unit("PLA", density=1.32)
    assert u._spool._grams_for(0, lane, 87, 1000) == 761


def test_the_variant_is_part_of_the_material_for_density():
    # A lane splits the tag: material "PLA", sub_type "Matte". The variant is
    # exactly the part that changes the density, so asking about "PLA" alone
    # wrote 691 g onto a reel that weighed 806.
    key = rfid.BambuSpoolman._material_key
    lane = types.SimpleNamespace(material="PLA", sub_type="Matte")
    assert key(lane, {}) == "PLA Matte"
    # The bridge's own string is already complete and is preferred.
    assert key(lane, {"material": "PLA Matte"}) == "PLA Matte"
    # No variant, or one already spelled out, must not double up.
    assert key(types.SimpleNamespace(material="PLA", sub_type=""), {}) == "PLA"
    assert key(types.SimpleNamespace(material="PLA Matte",
                                     sub_type="Matte"), {}) == "PLA Matte"
    # A lane with nothing on it yet falls back to the bay's record.
    assert key(types.SimpleNamespace(material=None, sub_type=""),
               {"material": "ABS"}) == "ABS"


def test_the_variant_reaches_the_density_lookup_end_to_end():
    # The lookup is what was broken: a lane says "PLA" with the variant in
    # sub_type, so a variant with its own density never got one. Plain PLA and
    # PLA Matte currently share 1.24, so this pins the PATH rather than a
    # number -- the day a matte density is measured properly it takes effect
    # instead of silently not.
    from extras.AFC_RFID import MATERIAL_DENSITY
    u, _obj = _wired_unit()
    u.name = "AMS"
    u.dry_dev_addr = 0x0700
    u.logger = types.SimpleNamespace(debug=lambda *a, **k: None,
                                     info=lambda *a, **k: None)
    u._slots = [{"index": 0, "material": "PLA Matte"}]
    u._bridge = types.SimpleNamespace(last_cap_measure=lambda addr: None)
    lane = types.SimpleNamespace(name="lane14", material="PLA",
                                 sub_type="Matte", density=None, weight=0)
    assert rfid.BambuSpoolman._material_key(lane, u._slots[0]) == "PLA Matte"
    expect = int(round(MATERIAL_DENSITY["plamatte"] * 663.0 * 0.87))
    assert u._spool._grams_for(0, lane, 87, 1000) == expect


# ── the measurement lands without Spoolman ───────────────────────────────────
#
# Since the split, a unit with no [AFC_BambuAMS_rfid] section no-opped its
# measurements: the percent-to-grams-to-vars closure lives on the Spoolman
# delegate, and with no delegate a capscan measured, narrated, and changed
# nothing on the lane. Printer 2 ran that way. And with the section, the unit's
# own reads of the measurement memos (remain_pct, the held summary, the bound
# tag, the removal edge) went dead, because the memos moved to the delegate and
# the reads did not.
#
# Every test here drives a REAL unit (object.__new__, given only what the code
# reads) through its real shims into a real BambuSpoolman.

class _Log:
    """Every line, with its level."""

    def __init__(self):
        self.lines = []

    def info(self, msg, *a, **k):
        self.lines.append(("INFO", msg))

    def debug(self, msg, *a, **k):
        self.lines.append(("DEBUG", msg))

    def warning(self, msg, *a, **k):
        self.lines.append(("WARNING", msg))

    def having(self, text, level=None):
        return [m for lv, m in self.lines
                if text in m and (level is None or lv == level)]


def _measuring_unit(section=None, spool_id=None, material="PLA Basic",
                    lane_material="PLA", core_mod=None, rfid_mod=None,
                    afc_spoolman=None, pool=False):
    """
    A real unit with bay 1 mapped to lane8, holding a tagged 1 kg reel.

    :param section: None = no [AFC_BambuAMS_rfid] section, True = enabled,
        False = present with ``enabled: False``
    :param spool_id: the lane's Spoolman binding
    :param material: the bay record's material, which picks the density
    :param lane_material: the lane's material
    :param core_mod: the unit module to build from (a re-import, for the
        AFC_RFID-absent case); the suite's own when None
    :param rfid_mod: the rfid module the section comes from, likewise
    :param afc_spoolman: AFC core's own Spoolman setting (a URL, or None)
    :param pool: build it as an unclaimed pool spare
    :return tuple: (unit, lane8, the section object or None)
    """
    core_mod = core_mod or core
    rfid_mod = rfid_mod or rfid
    u = core_mod.afcBambuAMS.__new__(core_mod.afcBambuAMS)
    obj = (None if section is None
           else rfid_mod.load_config(_Config(enabled=section)))
    u.lookups = []

    def _lookup(name, default=None):
        u.lookups.append(name)
        return obj if (obj is not None and name == 'AFC_BambuAMS_rfid') \
            else default

    u.printer = types.SimpleNamespace(lookup_object=_lookup)
    u.name = "Bambu_AMS_1"
    u.logger = _Log()
    u.pool = pool
    lane = types.SimpleNamespace(name="lane8", weight=999, spool_id=spool_id,
                                 material=lane_material, sub_type="Basic",
                                 density=None, tool_loaded=False)
    u.lanes = {"lane8": lane}
    u._slot_map = {"lane8": 1}
    u._slots = [{"index": 0, "present": False},
                {"index": 1, "present": True, "remain_pct": 80,
                 "weight": 1000, "rfid_uid": "d13fdb0e",
                 "material": material}, {}, {}]
    u._bridge = None
    u.saves = []
    u.afc = types.SimpleNamespace(
        save_vars=lambda: u.saves.append(1),
        reactor=types.SimpleNamespace(monotonic=lambda: 100.0),
        spoolman=afc_spoolman, moonraker=object())
    return u, lane, obj


def _published(u):
    """What get_status would publish for each slot, through the real path."""
    st = {"slots": [dict(s) for s in u._slots]}
    u.__class__._status_apply_measurements(u, st)
    return st["slots"]


@pytest.fixture
def no_spoolman_http(monkeypatch):
    """Record any attempt to reach Spoolman; run the worker's jobs inline."""
    calls = []

    def _client(afc):
        calls.append("client")
        return types.SimpleNamespace(
            set_remaining_weight=lambda sid, g: calls.append(("set", sid, g)))

    monkeypatch.setattr(rfid, "_bambu_spoolman_client", _client)
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: job())
    return calls


@pytest.mark.parametrize("section", [None, False],
                         ids=["no-section", "enabled-false"])
def test_the_measurement_lands_without_spoolman(section, no_spoolman_http):
    u, lane, _obj = _measuring_unit(section)
    assert u._spool is None
    assert u._adopt_measured_remain(1, 63, "capscan", seq=9456.4) is True
    assert lane.weight == 518            # the density model: PLA at 1.24
    assert u.saves == [1], "a measurement that is not saved is lost at restart"
    assert _published(u)[1]["remain_pct"] == 63
    assert no_spoolman_http == [], "nothing may reach for Spoolman"
    said = u.logger.having("Measured about 63% left", "INFO")
    assert len(said) == 1
    assert ("kept on the lane -- the Spoolman module ([AFC_BambuAMS_rfid]) "
            "is off") in said[0]


@pytest.mark.parametrize("section", [None, False],
                         ids=["no-section", "enabled-false"])
def test_a_bound_lane_is_named_and_not_written(section, no_spoolman_http):
    # The lane's binding is exactly when the operator needs to hear it: the
    # spool now disagrees with the lane, and nothing here updated it.
    u, lane, _obj = _measuring_unit(section, spool_id=164)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert lane.weight == 518
    assert no_spoolman_http == [], "no Spoolman client may even be built"
    said = u.logger.having("Measured about 63% left", "INFO")
    assert said and "so Spoolman spool 164 was not updated" in said[0]
    assert not u.logger.having("updated Spoolman spool"), (
        "never claim a Spoolman update when nothing talked to Spoolman")


def test_with_the_section_spoolman_is_still_written(no_spoolman_http):
    u, lane, _obj = _measuring_unit(True, spool_id=164)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert lane.weight == 518
    assert ("set", 164, 518.0) in no_spoolman_http


def test_the_measurement_delegate_is_not_a_unit_spoolman_serves():
    # The section's status lists the units IT serves; a unit measuring on its
    # own must not appear there and claim a Spoolman link it does not have.
    u, lane, obj = _measuring_unit(False)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert u._measure is not None and u._measure.spoolman_on is False
    assert obj.get_status()["units"] == []


def test_measure_is_the_spoolman_delegate_when_there_is_one():
    u, _lane, _obj = _measuring_unit(True)
    assert u._measure is u._spool
    assert u._meas_obj is None, "no second delegate beside the real one"


def test_the_measure_object_is_built_once():
    u, _lane, _obj = _measuring_unit(None)
    first = u._measure
    assert first is u._measure is u._measure
    assert first._u is u
    assert u.lookups == ['AFC_BambuAMS_rfid'], (
        "the section lookup is latched -- this runs on every status pass")


def test_apply_remain_weight_never_rewrites_a_held_measurement():
    # It finishes what an adoption still owes -- a bind that lands after it,
    # grams made before the material was known -- and reaches the
    # measurement-only delegate for the second. A measurement merely held is
    # owed nothing: without Spoolman there is no bind, and a write here would
    # undo AFC's own consumption count after every unload.
    u, lane, _obj = _measuring_unit(None)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    lane.weight = 450                    # consumed since
    u._apply_remain_weight(lane, {"index": 1, "weight": 1000})
    assert lane.weight == 450
    # ...and not by luck of the shim: the delegate itself declines.
    u._measure._apply_remain_weight(lane, {"index": 1, "weight": 1000})
    assert lane.weight == 450


def test_with_afc_rfid_absent_the_measurement_still_lands():
    # No density table without AFC_RFID, so the grams are tag-linear -- and
    # still land, and still save.
    with _without_afc_rfid() as (core_mod, rfid_mod):
        assert rfid_mod._AFC_RFID_ERR is not None
        u, lane, _obj = _measuring_unit(None, material="ABS",
                                        lane_material="ABS",
                                        core_mod=core_mod, rfid_mod=rfid_mod)
        assert u._adopt_measured_remain(1, 63, "capscan", seq=1.0) is True
        assert lane.weight == 630
        assert u.saves == [1]
        assert u._measure.spoolman_on is False


def test_with_afc_rfid_absent_the_section_says_why_once():
    # The section asks for Spoolman and cannot have it. Say so -- once for the
    # printer, not once per AMS -- and measure anyway.
    with _without_afc_rfid() as (core_mod, rfid_mod):
        u1, lane1, obj = _measuring_unit(True, material="ABS",
                                         lane_material="ABS",
                                         core_mod=core_mod, rfid_mod=rfid_mod)
        u2 = core_mod.afcBambuAMS.__new__(core_mod.afcBambuAMS)
        u2.printer, u2.name, u2.logger = u1.printer, "Bambu_AMS_2", u1.logger
        assert u1._spool is None and u2._spool is None
        warned = u1.logger.having("AFC_RFID could not be loaded", "WARNING")
        assert len(warned) == 1, u1.logger.lines
        assert "Measurements are still kept on the lanes" in warned[0]
        assert obj.get_status()["units"] == []
        u1._adopt_measured_remain(1, 63, "capscan", seq=1.0)
        assert lane1.weight == 630


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_remain_pct_shows_the_measurement(section):
    # With the section this was the dead read: get_status looked at the UNIT's
    # _measured_remain, which only the delegate ever wrote. It is now read back
    # out of the lane's grams (518 g of PLA against a 1 kg tag), which is the
    # same 63% while nothing has been used.
    u, _lane, _obj = _measuring_unit(section)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert _published(u)[1]["remain_pct"] == 63
    assert u._slots[1]["remain_pct"] == 63, "the next poll's copy too"


def test_remain_pct_is_the_floor_the_grams_came_from():
    # Filament does not grow: the second, higher reading of the same reel is
    # odometer noise, the lane keeps the first figure's grams, and remain_pct
    # -- read from those grams -- says the same thing rather than the raw 66.
    u, lane, _obj = _measuring_unit(None)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    u._adopt_measured_remain(1, 66, "capscan", seq=2.0)
    assert lane.weight == 518
    assert u._measure._measured_remain[1] == 66   # the raw identity is kept
    assert _published(u)[1]["remain_pct"] == 63


def test_remain_pct_is_capped_at_100():
    # A full PLA reel reads proud of the reference radius. The grams are
    # capped at the spool's nominal, and 1000 g of PLA reads back as 121% of
    # the reference; the share of the spool is capped too.
    u, lane, _obj = _measuring_unit(None)
    u._adopt_measured_remain(1, 138, "capscan", seq=1.0)
    assert lane.weight == 1000
    assert _published(u)[1]["remain_pct"] == 100


def test_a_bay_with_no_nominal_publishes_its_held_measurement_capped():
    # The lane cannot answer for a bay with no tag nominal (a tagless reel:
    # the bridge publishes its weight as None), so the held measurement is
    # published -- capped as a share of the spool. A tagless ABS reel reads
    # 138-151%, under the adopt ceiling of 200.
    u, _lane, _obj = _measuring_unit(None)
    u._slots[1]["weight"] = None
    u._slots[1]["rfid_uid"] = ""
    assert u._adopt_measured_remain(1, 138, "capscan", seq=1.0) is True
    assert _published(u)[1]["remain_pct"] == 100


def test_a_bay_with_no_nominal_publishes_the_reels_floor():
    # A tag whose profile would not decode: a UID but no nominal. Its held
    # measurement is published at the reel's floor, like the grams: 63 then
    # a noisy 66 of the same reel publishes 63.
    u, _lane, _obj = _measuring_unit(None)
    u._slots[1]["weight"] = None
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    u._adopt_measured_remain(1, 66, "capscan", seq=2.0)
    assert _published(u)[1]["remain_pct"] == 63


def test_a_pool_spare_builds_no_measurement_delegate():
    # A status poll on every unclaimed spare must not give each one a
    # delegate -- nor, with the section, list it among Spoolman's units.
    u, _lane, obj = _measuring_unit(True, pool=True)
    # A lane weight and a tag nominal are both there to read a percent from;
    # a spare still publishes nothing new.
    assert _published(u)[1]["remain_pct"] == 80
    assert core.afcBambuAMS._held_measurements(u) == {}
    assert u._meas_obj is None and u._spool_obj is None
    assert u.lookups == [], "not even the section lookup"
    assert obj.get_status()["units"] == []


def test_a_raising_delegate_still_closes_the_window_and_frees_the_bus():
    # The capscan adopt is followed by the window close, the adopted-reading
    # mark and the bus release. A raise inside the delegate used to skip all
    # three and leave the bus held until its own timeout.
    class _StuckLane:
        name = "lane8"
        spool_id = None
        material = "PLA"
        sub_type = "Basic"
        density = None
        tool_loaded = False

        @property
        def weight(self):
            return 999

        @weight.setter
        def weight(self, _v):
            raise RuntimeError("lane refused the write")

    u, _lane, _obj = _measuring_unit(None)
    u.lanes = {"lane8": _StuckLane()}
    released = []
    reading = {"pct": 56, "pct_raw": 56, "save_tray": 1, "restored": False,
               "t": 110.0}
    u._bridge = types.SimpleNamespace(
        last_cap_measure=lambda addr: dict(reading),
        last_ht_cali=lambda unit: None,
        release_bus=lambda name: released.append(name),
        _rfid_end_by_addr={})
    u.dry_dev_addr = 0x0700
    u.ams_index = 0
    u._cali_disarm_lane = None
    core.afcBambuAMS._cap_open_pending(u, 1, asked=True)
    _published(u)
    assert 1 not in u._cap_pending, "the window must close"
    assert u._cap_pending_slot is None
    assert u._cap_adopted_t == 110.0
    assert released == ["Bambu_AMS_1"], "the bus must be handed back"
    assert u.logger.having("could not apply a measurement", "WARNING")
    assert not u.logger.having("capscan/calibrate/follower status"), (
        "the raise must not have reached the status pass's own catch-all")


def _removal_ready(u):
    """Give a unit what the real removal edge in _maybe_auto_scan reads."""
    u._prev_present = [False, True, False, False]
    u._scan_primed = True
    u._auto_scanned = [False] * 4
    u._scan_notag = [False] * 4
    u._scan_t0 = [None] * 4


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_the_removal_edge_clears_the_delegates_memos(section):
    # The departed spool's percent must not be republished for the next one,
    # and its held summary and bound tag go with it.
    u, _lane, _obj = _measuring_unit(section)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    m = u._measure
    m._pending_summary[1] = (63, 518, 1000, None)
    m._bound_uid[1] = "d13fdb0e"
    m._bind_owed[1] = (63, 1000, None, True)
    m._convert_owed[1] = (63, 1000, 630)
    u._meas_baselined = {1}
    _removal_ready(u)
    u._slots[1] = {"index": 1, "present": False}
    core.afcBambuAMS._maybe_auto_scan(u, 1, False, u._slots[1])
    assert u.logger.having("spool REMOVED from slot 1")
    for memo in ("_measured_remain", "_meas_seq_seen", "_pending_summary",
                 "_bound_uid", "_bind_owed", "_convert_owed"):
        assert 1 not in getattr(m, memo), memo
    assert u._meas_baselined == set()
    assert core.afcBambuAMS._held_measurements(u) == {}


def test_release_clears_the_delegates_memos():
    # A released unit may be reclaimed by a different AMS whose bays share
    # the slot numbers; the old unit's measurements must not follow.
    u, _lane, _obj = _measuring_unit(True)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    m = u._measure
    m._pending_summary[1] = (63, 518, 1000, None)
    m._bound_uid[1] = "d13fdb0e"
    m._bind_owed[1] = (63, 1000, None, True)
    m._convert_owed[1] = (63, 1000, 630)
    u.SLOTS_PER_UNIT = 4
    u.release()
    for memo in ("_measured_remain", "_pending_summary", "_bound_uid",
                 "_bind_owed", "_convert_owed"):
        assert getattr(m, memo) == {}, memo
    # The stamp memo is the measurement's identity, not a fact about the
    # spool: it stays, so the same unit coming back does not re-adopt it.
    assert m._meas_seq_seen == {1: 1.0}
    assert u.pool is True


def _syncing(u):
    """Give a unit what the real _sync_lanes reads for a settled bay."""
    _removal_ready(u)
    u._prep_seen = True
    u._afc_owned = set()
    u.auto_scan = False
    u.surfaced = []
    # AFC core's lane plumbing, not under test here.
    u._surface_slot_info = lambda lane, info: u.surfaced.append(lane.name)
    u.lane_loaded = lambda lane: None
    u.lane_illuminate_spool = lambda lane: None


def _reclaim(u, monkeypatch, model="ams1", bridge=None,
             uid="A9CD393238310D0030383131"):
    """Release the unit and claim it again through the real claim(), with
    the bus plumbing stubbed: the bridge the pool unit finds on its port, the
    UID resolve, the announce and the lane restore.

    :param model: the model the unit is claimed as
    :param bridge: the bridge on the unit's port (a bare one by default)
    :param uid: the UID of the AMS it is claimed onto
    """
    u.SLOTS_PER_UNIT = 4
    u.serial_port = "/dev/fake"
    u.ams_index = 0
    for lane in u.lanes.values():
        lane.index = u._slot_map[lane.name] + 1
    u.release()
    if bridge is None:
        bridge = types.SimpleNamespace(add_listener=lambda cb: None,
                                       add_reconnect_listener=lambda cb: None)
    monkeypatch.setitem(core._bridge_mod._BRIDGES, "/dev/fake", bridge)
    u._resolve_uid_index = lambda tries: None
    u._announce_unit = lambda: None
    u._restore_claimed_lane_vars = lambda ln: None
    u.afc.reactor.register_callback = lambda cb, t=None: None
    u.afc.reactor.register_timer = lambda cb, t=None: None
    assert u.claim(uid, model) is True


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_same_unit_reclaim_does_not_readopt_its_stamp(section,
                                                        monkeypatch,
                                                        no_spoolman_http):
    # BridgeBox releases a unit that has been offline for 10 s and reclaims
    # the SAME UID when it comes back, and the bridge keeps every bay's stamp
    # (meas_seq/meas_pct) throughout. That stamp was adopted before the
    # release; taking it again on the reclaim wrote it over the grams used
    # since -- on the lane, and with the section in Spoolman too.
    u, lane, _obj = _measuring_unit(section, spool_id=164,
                                    afc_spoolman="http://spoolman:7912")
    rec = dict(u._slots[1], meas_seq=4, meas_pct=63)
    _syncing(u)
    u._stamp_looked = {1}                    # looked at before the stamp came
    u._slots[1] = dict(rec)
    core.afcBambuAMS._sync_lanes(u)          # watched appearing: adopted
    core.afcBambuAMS._sync_lanes(u)          # steady
    assert lane.weight == 518
    assert len(u.logger.having("Measured about 63% left", "INFO")) == 1
    writes, saves = list(no_spoolman_http), list(u.saves)
    lane.weight = 400                        # consumed since
    _reclaim(u, monkeypatch)                 # reclaimed: the same UID
    u._slots = [{"index": 0, "present": False}, dict(rec), {}, {}]
    core.afcBambuAMS._sync_lanes(u)
    assert lane.weight == 400, "the consumption since must survive"
    assert no_spoolman_http == writes, "no second write of the old figure"
    assert u.saves == saves
    assert len(u.logger.having("Measured about 63% left", "INFO")) == 1


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_claim_baselines_a_stamp_that_moved_while_released(
        section, monkeypatch, no_spoolman_http):
    # The claim is a new connection for the unit, so what its bays carry at
    # the claim was not watched being taken: recorded, not applied -- even a
    # stamp that moved while the unit was released.
    u, lane = _connected_unit(monkeypatch, section, spool_id=164, weight=999,
                              stamp=(4, 60))
    _frames(u)
    _in_session(u)
    _stamp(u, 5, 89)
    _frames(u)
    assert lane.weight == 732
    lane.weight = 678                     # printed from
    writes, saves = list(no_spoolman_http), list(u.saves)
    rec = dict(u._slots[1])
    _reclaim(u, monkeypatch)
    u._slots = [{"index": 0, "present": False},
                dict(rec, meas_seq=6, meas_pct=70), {}, {}]
    _frames(u, 3)
    assert lane.weight == 678
    assert no_spoolman_http == writes and u.saves == saves
    assert u._meas_seen[1] == (6, 70), "recorded as seen"
    assert len(u.logger.having("Measured", "INFO")) == 1


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_held_summary_is_said_when_the_record_lands(section):
    # The measurement finishes before the bay's record catches up, so its
    # summary waits -- on the delegate that queued it. _sync_lanes only
    # looked for one on the unit, and never drained it.
    u, _lane, _obj = _measuring_unit(section)
    rec = dict(u._slots[1])
    u._slots[1] = {"index": 1, "present": True, "weight": 1000}
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert 1 in u._measure._pending_summary, "the record had not landed"
    assert not u.logger.having("Measured about 63% left")
    _syncing(u)
    u._slots[1] = rec
    core.afcBambuAMS._sync_lanes(u)
    assert u.surfaced == ["lane8"]
    said = u.logger.having("Measured about 63% left", "INFO")
    assert len(said) == 1 and "PLA Basic" in said[0]
    assert 1 not in u._measure._pending_summary


def _settle_no_new_read(u):
    """A scan of bay 1 that ended with no new read, the record unchanged: the
    real _scan_verdict answers "notag" (no bridge left to ask)."""
    _syncing(u)
    u._scan_t0 = [None, 50.0, None, None]
    core.afcBambuAMS._sync_lanes(u)


def test_the_settle_path_sees_the_delegates_held_measurement():
    # "already measured this session" keys off the held percent, which lives
    # on the delegate -- read from the unit it was never there, and the line
    # said the unit "reported no NEW read" about a spool it had just measured.
    u, _lane, _obj = _measuring_unit(True)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert core.afcBambuAMS._held_measurements(u) == {1: 63}
    _settle_no_new_read(u)
    assert u.logger.having("already measured this session (63% held)",
                           "INFO")
    assert not u.logger.having("reported no NEW read")


@pytest.mark.parametrize("bound, same", [("d13fdb0e", True),
                                         ("7392020a", False),
                                         (None, False)],
                         ids=["same-tag", "other-tag", "never-bound"])
def test_the_settle_path_sees_the_delegates_bound_tag(bound, same):
    # The bound-tag memo is the delegate's. Read from the unit, the "same
    # spool" note never printed: the unit's own copy is never written.
    u, _lane, _obj = _measuring_unit(True)
    if bound:
        u._spool._bound_uid[1] = bound
    _settle_no_new_read(u)
    said = u.logger.having("reported no NEW read", "INFO")
    assert len(said) == 1
    assert ("(same spool as last time)" in said[0]) is same


def test_the_readout_quotes_the_grams_the_lane_got():
    u, lane, _obj = _measuring_unit(None)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    info = dict(u._slots[1], meas_pct=63, meas_seq=1)
    core.afcBambuAMS._log_tag_readout(u, lane, info, force=True)
    assert u.logger.having("63% MEASURED (~518 g)"), u.logger.lines


def test_the_readout_quotes_the_reels_floor_without_the_section():
    # A re-scan that reads above the reel's floor: the lane holds the grams
    # for 63%, so the readout has to say so rather than quote 66%'s grams.
    # The floor lives on the measurement delegate, not on _spool.
    u, lane, _obj = _measuring_unit(None)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    info = dict(u._slots[1], meas_pct=66, meas_seq=2)
    core.afcBambuAMS._log_tag_readout(u, lane, info, force=True)
    assert u.logger.having("66% MEASURED, held at 63% (~518 g)"), (
        u.logger.lines)


@pytest.mark.parametrize("section", [None, False],
                         ids=["no-section", "enabled-false"])
def test_a_summary_without_the_module_is_not_held_for_a_bind(section):
    # AFC core has Spoolman and the bay was scanned this session, which is
    # when a Spoolman-on summary waits for the bind to go out first. With the
    # module off no bind is ever sent, so the summary is said at once.
    u, _lane, _obj = _measuring_unit(section,
                                     afc_spoolman="http://spoolman:7912")
    u._scanned_bays = {1}
    u._spoolman_latched = set()
    assert u._adopt_measured_remain(1, 63, "capscan", seq=1.0) is True
    assert len(u.logger.having("Measured about 63% left", "INFO")) == 1


@pytest.mark.parametrize("section, advised", [
    (None, False), (False, False), (True, True),
], ids=["no-section", "enabled-false", "section"])
def test_the_bind_advice_needs_the_module(section, advised):
    # A tag whose profile will not decode is still a UID Spoolman can match
    # -- with the module on. With it off nothing matches a UID, and the same
    # line goes on to say the module is off.
    u, lane, _obj = _measuring_unit(section)
    u._slots[1] = {"index": 1, "present": True, "weight": 1000,
                   "rfid_uid": "aabbccdd"}
    u._measure._say_spool_summary(1, lane, 63, 518, 1000)
    said = u.logger.having("Measured about 63% left", "INFO")
    assert len(said) == 1
    assert ("tag AABBCCDD read but its profile could not be decoded "
            "(not a Bambu tag?)") in said[0]
    assert ("bind that UID to a spool in Spoolman" in said[0]) is advised


def test_a_later_frame_writes_nothing_after_a_held_reading(no_spoolman_http):
    # _apply_remain_weight used to re-assert the measured grams on every frame
    # while the lane was not at the toolhead, so a bound lane printed from and
    # unloaded went straight back to its measured grams -- and Spoolman was
    # written again. A consumed bound lane is not written back now.
    u, lane, _obj = _measuring_unit(True, spool_id=164)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    u._adopt_measured_remain(1, 66, "capscan", seq=2.0)
    assert lane.weight == 518
    writes = list(no_spoolman_http)
    lane.weight = 450                     # consumed since
    u._apply_remain_weight(lane, dict(u._slots[1]))
    assert lane.weight == 450
    assert no_spoolman_http == writes, "no write of the measured grams"


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_new_scan_voids_the_held_measurement(section):
    # The second clearing site beside the removal edge: a re-scan with no
    # removal (a CAPSCAN on the same bay, a swap the edge missed) must not
    # keep publishing the previous measured percent. A held measurement is
    # published only where the lane cannot answer -- a bay with no tag
    # nominal -- so that is the bay this looks at.
    u, _lane, _obj = _measuring_unit(section)
    u._slots[1]["weight"] = None
    rec = dict(u._slots[1])
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert core.afcBambuAMS._held_measurements(u) == {1: 63}
    assert _published(u)[1]["remain_pct"] == 63
    u._slots[1] = dict(rec)                 # the next frame
    _removal_ready(u)
    core.afcBambuAMS._open_scan(u, 1)
    assert core.afcBambuAMS._held_measurements(u) == {}
    assert _published(u)[1]["remain_pct"] == 80, "back to the bay's record"


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_no_tag_scan_keeps_the_measured_grams(section):
    # #51: a failed read is no evidence about how much filament is on the
    # reel. The guard reads the held measurement, which lives on the
    # delegate with or without the section.
    u, lane, _obj = _measuring_unit(section)
    u._adopt_measured_remain(1, 63, "capscan", seq=1.0)
    assert lane.weight == 518
    _removal_ready(u)
    # AFC core's lane plumbing, not under test here; the blank zeroes the
    # weight exactly as the real one does.
    u._release_scan_hold = lambda slot: None
    u._clear_lane_filament = lambda ln: setattr(ln, "weight", 0)
    u._unbind_spool = lambda ln, *a: None
    core.afcBambuAMS._finalize_scan(u, 1, scanned=True)
    assert lane.weight == 518
    assert u.logger.having("kept the measured 518 g through a no-tag scan")


# ── the CAPSCAN-time warning for a bound lane ────────────────────────────────
#
# With the Spoolman module off, a measurement lands on the lane and nowhere
# else. AFC core reloads a bound lane's weight from Spoolman at restart
# (AFC_prep: set_spoolID -> remaining_weight), so on a lane that still carries
# a spool_id the lane and the spool can disagree afterwards; the bay's stamp is
# recorded at the restart and never re-applied, so the spool's figure is the
# one the lane shows. Said once, when the measure is asked for -- never per
# reading.

class _GCmd:
    def __init__(self, **params):
        self.params = params
        self.said = []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def respond_info(self, msg):
        self.said.append(msg)

    def error(self, msg):
        return RuntimeError(msg)


def _capscan(monkeypatch, section, spool_id, afc_spoolman):
    u, lane, _obj = _measuring_unit(section, spool_id=spool_id,
                                    afc_spoolman=afc_spoolman)
    u._bridge = types.SimpleNamespace()
    u.afc.function = types.SimpleNamespace(in_print=lambda: False)
    asked = []
    # The calibrate itself is hardware; the command's own path is not.
    monkeypatch.setattr(core.afcBambuAMS, "_run_calibrate",
                        lambda self, ln, slot: asked.append(slot))
    g = _GCmd(LANE="lane8")
    u.cmd_AFC_BAMBU_CAPSCAN(g)
    assert asked == [1] and g.said, "the command must have run"
    return u.logger.having("can disagree", "WARNING")


def test_a_bound_lane_with_the_module_off_is_warned_at_capscan(monkeypatch):
    warned = _capscan(monkeypatch, None, 164, "http://spoolman:7912")
    assert len(warned) == 1
    assert "lane8 is bound to Spoolman spool 164" in warned[0]
    assert "lane8 and spool 164 can disagree" in warned[0]
    # It says they can disagree, and leaves it there: which figure a lane
    # shows after a restart is the restart's business, not this warning's.
    assert "will show" not in warned[0]


@pytest.mark.parametrize("section, spool_id, afc_spoolman", [
    (True, 164, "http://spoolman:7912"),     # the module writes the spool
    (None, None, "http://spoolman:7912"),    # nothing bound, nothing reloads
    (None, 164, None),                       # AFC has no Spoolman to reload from
], ids=["module-on", "unbound", "afc-without-spoolman"])
def test_no_capscan_warning_otherwise(monkeypatch, section, spool_id,
                                      afc_spoolman):
    assert _capscan(monkeypatch, section, spool_id, afc_spoolman) == []


def test_enabled_false_counts_as_the_module_off(monkeypatch):
    assert len(_capscan(monkeypatch, False, 164, "http://spoolman:7912")) == 1


# ── a measurement is applied once, at the time it is taken ───────────────────
#
# The firmware keeps each bay's last measurement stamp (meas_pct/meas_seq) in
# RAM and publishes it in every status frame, so a Klipper restart hands the
# old reading straight back -- and it was applied as new. Printer 1's lane12
# re-announced a 138% (seq 7) reading on every restart and wrote it over the
# lower weight AFC had just restored. And once applied, a measurement was
# applied again on every frame: lane10 measured 89% (732 g), printed down to
# 678 g, and went back to 732 g the moment it was unloaded.
#
# So: a stamp present when a connection starts is recorded and never applied;
# a stamp that changes during the connection is adopted, once; the only
# follow-up is handing it to the spool a Spoolman bind attaches after it; and
# remain_pct is read back out of the lane's grams.
#
# These drive a real unit through the real _sync_lanes (with its real
# _surface_slot_info), _status_apply_measurements and get_status, into a real
# BambuSpoolman, with the section present and absent.

class _Spool:
    """AFC core's spool object. set_spoolID fetches the spool from moonraker
    asynchronously, so the lane is bound when the test says the fetch landed
    -- measured 0.43 s after "assigned to lane8" on printer 2."""

    def __init__(self):
        self.asked = []

    def set_spoolID(self, lane, sid):
        self.asked.append((lane, sid))

    def land(self, stored):
        """The fetch answers: spool_id and Spoolman's stored weight together.

        :param stored: the remaining_weight Spoolman had for the spool
        """
        for lane, sid in self.asked:
            lane.spool_id, lane.weight = sid, stored
        self.asked = []


def _connected_unit(monkeypatch, section, spool_id=None, material="PLA Basic",
                    weight=999, stamp=None, name="lane8"):
    """
    A real unit whose connection has just started: bay 1 -> the lane, no bay
    looked at, not primed, driven through the real _sync_lanes and its real
    _surface_slot_info. AFC core has Spoolman; a bind by UID matches spool
    163 through the delegate's own worker path and lands on ``u.afc.spool``.

    :param section: as _measuring_unit
    :param spool_id: the lane's restored binding
    :param material: the bay record's material
    :param weight: the lane's restored grams
    :param stamp: (meas_seq, meas_pct) the bay's record carries, or None
    :param name: the lane's name
    :return tuple: (unit, lane)
    """
    base = material.split()[0] if material else ""
    u, lane, _obj = _measuring_unit(section, spool_id=spool_id,
                                    material=material, lane_material=base,
                                    afc_spoolman="http://spoolman:7912")
    _syncing(u)
    del u._surface_slot_info          # the real one: it carries the follow-up
    lane.name = name
    lane.weight = weight
    if base != "PLA":
        lane.sub_type = ""
    u._scan_primed = False
    u._stamp_looked = set()
    u._scanned_bays = set()
    u._spoolman_latched = set()
    u.auto_spoolman_create = False
    u.afc.spool = _Spool()
    monkeypatch.setattr(rfid, "match_spool_for_tag",
                        lambda client, uid, tray: ({"id": 163}, False, None))
    if stamp is not None:
        u._slots[1] = dict(u._slots[1], meas_seq=stamp[0], meas_pct=stamp[1])
    return u, lane


def _frames(u, n=2):
    """``n`` status frames through the real _sync_lanes."""
    for _ in range(n):
        core.afcBambuAMS._sync_lanes(u)


def _stamp(u, seq, pct):
    """The firmware stamps bay 1 with a measurement.

    :param seq: meas_seq
    :param pct: meas_pct
    """
    u._slots[1] = dict(u._slots[1], meas_seq=seq, meas_pct=pct)


def _in_session(u):
    """PREP has run, the prime has fired, and bay 1 has been scanned and its
    bind dispatched on this connection -- a spool the unit is living with."""
    u._prep_seen = True
    u._scan_primed = True
    u._scanned_bays.add(1)
    u._spoolman_latched.add(1)


def _writes(calls):
    """The Spoolman weight writes in a no_spoolman_http record."""
    return [c for c in calls if c != "client"]


def _status(u, monkeypatch):
    """The slots the real get_status publishes."""
    monkeypatch.setattr(core.afcUnit, "get_status",
                        lambda self, eventtime=None: {})
    for k, v in (("unit_slots", 4), ("ams_index", 0),
                 ("_following_lane", None), ("_follow_fault_hold", None),
                 ("_drying", False), ("has_heater", False),
                 ("ams_model", "ams2"), ("dry_max_temp", None)):
        if not hasattr(u, k):
            setattr(u, k, v)
    return u.get_status()["slots"]


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_restart_does_not_apply_the_stamp_it_finds(section, monkeypatch,
                                                     no_spoolman_http):
    # Printer 1's loop: lane12 carries seq 7 at 138% from the day before, and
    # every restart applied it -- 952 g over the 600 g AFC had just restored,
    # an operator summary, and a Spoolman write. Three restarts here, each a
    # fresh unit, as Klipper builds them.
    for _restart in range(3):
        u, lane = _connected_unit(monkeypatch, section, spool_id=163,
                                  material="ABS", weight=600, stamp=(7, 138),
                                  name="lane12")
        u._prep_seen = False
        u._afc_owned = {1}                # PREP found the lane restored
        _frames(u)                        # before PREP
        u._prep_seen = True
        _frames(u)                        # after PREP
        u._scan_primed = True
        _frames(u, 3)                     # after the prime
        assert lane.weight == 600, "the restored grams stand"
        assert _writes(no_spoolman_http) == [], "no Spoolman write"
        assert not u.logger.having("Measured"), "no operator summary"
        assert u.saves == [], "nothing new to save"
        assert u._meas_seen[1] == (7, 138), "recorded as seen"
        # remain_pct is what the restored grams stand for: 600 g of ABS
        # (1.04 g/cm3 x 663 cm3 at 100%) is 87%.
        assert _status(u, monkeypatch)[1]["remain_pct"] == 87


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_lane8_after_a_restart_reads_its_own_grams(section, monkeypatch,
                                                   no_spoolman_http):
    # Printer 2's lane8 measured 56% and was given 459 g; after the restart it
    # holds 460 g, the stamp is still 56%, and the tag's own record says 80.
    # The lane's grams answer -- the tag's 80 does not win, and nothing is
    # applied to get there.
    u, lane = _connected_unit(monkeypatch, section, spool_id=164, weight=460,
                              stamp=(3, 56))
    _frames(u)
    u._scan_primed = True
    _frames(u)
    assert lane.weight == 460
    assert _writes(no_spoolman_http) == [] and u.saves == []
    assert not u.logger.having("Measured")
    assert u._slots[1]["remain_pct"] == 80, "the record's own figure"
    assert _status(u, monkeypatch)[1]["remain_pct"] == 56
    assert _published(u)[1]["remain_pct"] == 56


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_lane10_keeps_what_it_printed(section, monkeypatch, no_spoolman_http):
    # Measured 89% -> 732 g, loaded, printed down to 678 g, unloaded. The
    # measurement was applied when it was taken and never again: after the
    # unload the lane reads 678 g and 82%, and nothing writes.
    u, lane = _connected_unit(monkeypatch, section, spool_id=164, weight=999,
                              stamp=(4, 60), name="lane10")
    _frames(u)                            # connection start: 60% recorded
    _in_session(u)
    _stamp(u, 5, 89)                      # a capscan measures
    _frames(u)
    assert lane.weight == 732
    assert len(u.logger.having("Measured about 89% left", "INFO")) == 1
    assert _writes(no_spoolman_http) == ([("set", 164, 732.0)] if section
                                         else [])
    writes, saves = list(no_spoolman_http), list(u.saves)
    lane.tool_loaded = True
    _frames(u)
    lane.weight = 678                     # AFC counts the print off
    _frames(u)
    lane.tool_loaded = False              # unloaded
    _frames(u, 3)
    assert lane.weight == 678, "the unload must not put 732 g back"
    assert no_spoolman_http == writes and u.saves == saves
    assert len(u.logger.having("Measured about 89% left", "INFO")) == 1
    assert _status(u, monkeypatch)[1]["remain_pct"] == 82


def test_a_bind_that_lands_later_gets_the_measurement_once(monkeypatch,
                                                           no_spoolman_http):
    # A fresh insert is measured before its Spoolman lookup answers. The bind
    # then hydrates the lane with Spoolman's stored figure, so the spool it
    # attaches is owed the measurement -- once. A weight set by hand later is
    # the operator's, and stays.
    u, lane = _connected_unit(monkeypatch, True, weight=999, stamp=(4, 60))
    _frames(u)
    u._prep_seen = u._scan_primed = True
    u._scanned_bays.add(1)                # the insert's scan: bind not sent yet
    _stamp(u, 5, 89)
    _frames(u)                            # adopted, then the bind goes out
    assert lane.weight == 732 and lane.spool_id is None
    assert u.afc.spool.asked, "the bind was dispatched"
    assert u._spool._bind_owed == {1: (89, 1000, None, True)}
    assert _writes(no_spoolman_http) == [], "nothing bound to write to yet"
    _frames(u)                            # the fetch has not answered
    assert u._spool._bind_owed, "still on its way"
    u.afc.spool.land(1000)                # Spoolman's stored full reel
    _frames(u, 3)
    assert lane.spool_id == 163 and lane.weight == 732
    assert _writes(no_spoolman_http) == [("set", 163, 732.0)], "exactly once"
    assert u._spool._bind_owed == {}
    lane.weight = 700                     # set by hand
    _frames(u, 3)
    assert lane.weight == 700, "not re-asserted"
    assert _writes(no_spoolman_http) == [("set", 163, 732.0)]


def test_without_the_section_nothing_is_owed_to_a_bind(monkeypatch,
                                                       no_spoolman_http):
    # No module, no bind: the adoption is the only write there is.
    u, lane = _connected_unit(monkeypatch, None, weight=999, stamp=(4, 60))
    _frames(u)
    u._prep_seen = u._scan_primed = True
    u._scanned_bays.add(1)
    _stamp(u, 5, 89)
    _frames(u)
    assert lane.weight == 732
    assert u._measure._bind_owed == {}
    lane.spool_id, lane.weight = 163, 1000   # bound some other way
    _frames(u, 3)
    assert lane.weight == 1000
    assert _writes(no_spoolman_http) == []


def test_a_bind_owed_is_dropped_once_the_spool_is_fed(monkeypatch,
                                                      no_spoolman_http):
    # Loaded before the bind answered: extrusion owns the weight now, and the
    # spool no longer holds what was measured.
    u, lane = _connected_unit(monkeypatch, True, weight=999, stamp=(4, 60))
    _frames(u)
    u._prep_seen = u._scan_primed = True
    u._scanned_bays.add(1)
    _stamp(u, 5, 89)
    _frames(u)
    lane.tool_loaded = True
    _frames(u)
    assert u._spool._bind_owed == {}
    u.afc.spool.land(1000)
    lane.weight = 950
    lane.tool_loaded = False
    _frames(u, 3)
    assert lane.weight == 950
    assert _writes(no_spoolman_http) == []


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_bound_lane_is_written_at_adoption_and_never_again(
        section, monkeypatch, no_spoolman_http):
    u, lane = _connected_unit(monkeypatch, section, spool_id=164, weight=999,
                              stamp=(4, 60))
    _frames(u)
    _in_session(u)
    _stamp(u, 5, 89)
    _frames(u, 4)
    assert lane.weight == 732
    expect = [("set", 164, 732.0)] if section else []
    assert _writes(no_spoolman_http) == expect
    assert core.afcBambuAMS._built_measure_objs(u)[-1]._bind_owed == {}
    lane.weight = 998                     # Spoolman re-hydrates the lane
    _frames(u, 3)
    assert lane.weight == 998, "not re-asserted"
    assert _writes(no_spoolman_http) == expect


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_new_measurement_after_use_adopts_normally(section, monkeypatch,
                                                     no_spoolman_http):
    u, lane = _connected_unit(monkeypatch, section, spool_id=164, weight=999,
                              stamp=(4, 60))
    _frames(u)
    _in_session(u)
    _stamp(u, 5, 89)
    _frames(u)
    lane.weight = 678                     # printed from
    _frames(u)
    _stamp(u, 6, 82)                      # measured again
    _frames(u, 3)
    assert lane.weight == 674             # 1.24 x 663 x 82%
    assert len(u.logger.having("Measured about 82% left", "INFO")) == 1
    if section:
        assert _writes(no_spoolman_http) == [("set", 164, 732.0),
                                             ("set", 164, 674.0)]
    assert _status(u, monkeypatch)[1]["remain_pct"] == 82


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_after_a_removal_the_new_spools_first_stamp_adopts(
        section, monkeypatch, no_spoolman_http):
    u, lane = _connected_unit(monkeypatch, section, weight=999,
                              stamp=(4, 60))
    _frames(u)                            # connection start: 60% recorded
    u._prep_seen = u._scan_primed = True
    u._slots[1] = {"index": 1, "present": False, "meas_seq": 4,
                   "meas_pct": None}      # the firmware clears the percent
    _frames(u)
    assert u.logger.having("spool REMOVED from slot 1")
    assert 1 not in u._meas_seen
    u._slots[1] = {"index": 1, "present": True, "weight": 1000,
                   "rfid_uid": "7392020a", "material": "PLA Basic",
                   "meas_seq": 4, "meas_pct": None}
    _frames(u)
    assert u.logger.having("spool INSERTED in slot 1")
    u._scanned_bays.add(1)
    _stamp(u, 5, 75)                      # the new spool's first reading
    _frames(u)
    assert lane.weight == 617             # 1.24 x 663 x 75%
    assert len(u.logger.having("Measured about 75% left", "INFO")) == 1


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_bridge_reconnect_does_not_readopt(section, monkeypatch,
                                             no_spoolman_http):
    # A primed unit, a measurement adopted and printed from, then the link
    # drops. The Pico kept its stamps; one bay's even moved while the link was
    # down, with nothing of ours asking for it. Neither is applied.
    u, lane = _connected_unit(monkeypatch, section, spool_id=164, weight=999,
                              stamp=(4, 60))
    _frames(u)
    _in_session(u)
    _stamp(u, 5, 89)
    _frames(u)
    lane.weight = 678
    writes, saves = list(no_spoolman_http), list(u.saves)
    u.afc.reactor.register_callback = lambda cb, t=None: None
    core.afcBambuAMS._on_bridge_reconnect(u)
    _frames(u)                            # the same stamp, handed back
    assert lane.weight == 678
    core.afcBambuAMS._on_bridge_reconnect(u)
    _stamp(u, 6, 70)                      # moved while the link was down
    _frames(u, 3)
    assert lane.weight == 678
    assert u._meas_seen[1] == (6, 70), "recorded as seen"
    assert no_spoolman_http == writes and u.saves == saves
    assert len(u.logger.having("Measured", "INFO")) == 1
    # And a stamp that moves AFTER the reconnect is a measurement again.
    _stamp(u, 7, 80)
    _frames(u)
    assert lane.weight == 658             # 1.24 x 663 x 80%


def test_a_capscan_answered_across_a_reconnect_still_adopts(monkeypatch,
                                                            no_spoolman_http):
    # The one stamp a new connection may apply on first sight: the answer to a
    # window of ours that is still open. The link blipped mid-capscan.
    u, lane = _connected_unit(monkeypatch, None, spool_id=164, weight=999,
                              stamp=(4, 60))
    _frames(u)
    _in_session(u)
    core.afcBambuAMS._cap_open_pending(u, 1, asked=True)
    u.afc.reactor.register_callback = lambda cb, t=None: None
    core.afcBambuAMS._on_bridge_reconnect(u)
    _stamp(u, 5, 89)
    _frames(u)
    assert lane.weight == 732


def test_a_claim_and_a_reconnect_start_a_new_look():
    # A reclaim keeps _scan_primed set and a reconnect never clears it, which
    # is why the baseline is keyed on the first look instead.
    for fn in (core.afcBambuAMS.claim, core.afcBambuAMS._on_bridge_reconnect,
               core.afcBambuAMS._handle_disconnect):
        assert "self._stamp_looked = set()" in inspect.getsource(fn), fn


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_tagless_bay_keeps_its_old_remain_pct(section, monkeypatch):
    # lane9 is tagless: no nominal to read its grams against, and its lane
    # weight is AFC's 1000 g default -- read as a share of anything it would
    # say 100. So it keeps what it had: its record's figure, or a measurement
    # held for it.
    u, lane = _connected_unit(monkeypatch, section, weight=1000,
                              material=None, name="lane9")
    lane.material = ""
    u._slots[1] = {"index": 1, "present": True, "remain_pct": None,
                   "weight": None, "rfid_uid": None, "material": None,
                   "meas_seq": 0, "meas_pct": None}   # never measured
    _frames(u)
    assert _status(u, monkeypatch)[1]["remain_pct"] is None
    u._prep_seen = u._scan_primed = True
    _stamp(u, 5, 25)
    _frames(u)
    assert _status(u, monkeypatch)[1]["remain_pct"] == 25


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_an_ht_spool_over_100_percent_publishes_100(section, monkeypatch):
    # 141% of ABS is 972 g -- under the tag's 1 kg -- and reads back as 141%
    # of the reference. The panel's field is a share of the spool.
    u, lane = _connected_unit(monkeypatch, section, material="ABS",
                              weight=999, stamp=(4, 60))
    _frames(u)
    _in_session(u)
    _stamp(u, 5, 141)
    _frames(u)
    assert lane.weight == 972
    assert _status(u, monkeypatch)[1]["remain_pct"] == 100


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_an_unknown_density_reads_grams_against_the_nominal(section,
                                                            monkeypatch):
    # Nothing names the material, so there is no density to invert through:
    # 460 g of a 1000 g reel is 46%, not the 56% PLA would make it.
    u, lane = _connected_unit(monkeypatch, section, material=None,
                              weight=460)
    lane.material = lane.sub_type = ""
    assert _status(u, monkeypatch)[1]["remain_pct"] == 46


def test_the_inverse_undoes_grams_for():
    # One model, both directions: a measurement reads back as its own percent.
    u, lane, _obj = _measuring_unit(None)
    m = u._measure
    for pct in (23, 56, 63, 89):
        grams = m._grams_for(1, lane, pct, 1000)
        assert round(m._pct_for(1, lane, grams, 1000)) == pct
    assert m._pct_for(1, lane, 0, 1000) is None
    assert m._pct_for(1, lane, 500, 0) is None


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_pool_spare_publishes_nothing_new_through_get_status(section,
                                                               monkeypatch):
    u, _lane, obj = _measuring_unit(section, pool=True)
    assert _status(u, monkeypatch)[1]["remain_pct"] == 80
    assert u._meas_obj is None and u._spool_obj is None
    assert u.lookups == []


def test_a_record_that_gains_its_stamp_late_is_still_a_first_look(
        monkeypatch, no_spoolman_http):
    # A frame with no stamp field says nothing about what the bay was
    # measured at, so it does not use up the connection's first look: an old
    # stamp that only shows up a frame later is still recorded, not applied.
    u, lane = _connected_unit(monkeypatch, True, spool_id=163, weight=600)
    u._prep_seen = u._scan_primed = True
    _frames(u)                            # no meas_seq in the record yet
    _stamp(u, 7, 138)
    _frames(u)
    assert lane.weight == 600
    assert _writes(no_spoolman_http) == []
    assert u._meas_seen[1] == (7, 138)


def test_a_bind_that_finds_no_spool_is_owed_nothing(monkeypatch,
                                                     no_spoolman_http):
    # Spoolman does not know the tag, so no bind is coming. A spool bound by
    # hand afterwards is the operator's choice and keeps its own figure.
    u, lane = _connected_unit(monkeypatch, True, weight=999, stamp=(4, 60))
    monkeypatch.setattr(rfid, "match_spool_for_tag",
                        lambda client, uid, tray: (None, False, None))
    _frames(u)
    u._prep_seen = u._scan_primed = True
    u._scanned_bays.add(1)
    _stamp(u, 5, 89)
    _frames(u)
    assert lane.weight == 732
    assert u._spool._bind_owed == {}
    lane.spool_id, lane.weight = 170, 1000    # SET_SPOOL_ID by hand
    _frames(u, 3)
    assert lane.weight == 1000
    assert _writes(no_spoolman_http) == []


def test_a_lane_bound_when_its_bind_goes_out_is_written_once(
        monkeypatch, no_spoolman_http):
    # The insert's bind has not been sent yet, but the lane already carries a
    # binding (restored with it, and Spoolman does not contradict it). The
    # adoption writes that spool; nothing is held back to write it again.
    u, lane = _connected_unit(monkeypatch, True, spool_id=164, weight=999,
                              stamp=(4, 60))
    _frames(u)
    u._prep_seen = u._scan_primed = True
    u._scanned_bays.add(1)                # scanned, bind not dispatched
    _stamp(u, 5, 89)
    _frames(u, 3)
    assert lane.weight == 732
    assert _writes(no_spoolman_http) == [("set", 164, 732.0)]
    assert u._spool._bind_owed == {}


def test_a_tagless_spool_is_owed_nothing(monkeypatch, no_spoolman_http):
    # No UID, so no bind will ever come for it. A spool bound by hand later
    # is the operator's choice and keeps its own figure.
    u, lane = _connected_unit(monkeypatch, True, weight=999, name="lane9")
    u._slots[1] = {"index": 1, "present": True, "remain_pct": None,
                   "weight": None, "rfid_uid": None, "material": None,
                   "meas_seq": 4, "meas_pct": 60}
    _frames(u)
    u._prep_seen = u._scan_primed = True
    u._scanned_bays.add(1)
    _stamp(u, 5, 25)
    _frames(u)
    assert u._spool._bind_owed == {}
    lane.spool_id, lane.weight = 170, 1000    # SET_SPOOL_ID by hand
    _frames(u, 3)
    assert lane.weight == 1000
    assert _writes(no_spoolman_http) == []


def test_a_measurement_landing_while_the_bind_is_in_flight_is_owed(
        monkeypatch, no_spoolman_http):
    # The other order: the bind went out first and its lookup is still on the
    # worker when the measurement lands. _bind_pending is what says a bind is
    # on its way then, and the spool it attaches is owed the figure the same.
    u, lane = _connected_unit(monkeypatch, True, weight=999, stamp=(4, 60))
    queued = []
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: queued.append(job))
    _frames(u)
    u._prep_seen = u._scan_primed = True
    u._scanned_bays.add(1)
    _frames(u)                            # the bind goes out, lookup queued
    assert u._spool._bind_pending == {1}
    _stamp(u, 5, 89)
    _frames(u)                            # measured while it is in flight
    assert u._spool._bind_owed == {1: (89, 1000, None, True)}
    while queued:
        queued.pop(0)()                   # the lookup answers
    u.afc.spool.land(1000)
    _frames(u, 3)
    while queued:
        queued.pop(0)()
    assert lane.spool_id == 163 and lane.weight == 732
    assert _writes(no_spoolman_http) == [("set", 163, 732.0)]


# ── an insert edge on a connection's first frame ─────────────────────────────
#
# A reclaim keeps _prev_present and _scan_primed, so its first frame is an
# INSERT edge for every bay the previous unit had empty: printer 2, 18:39:52,
# UID A9CD... claimed onto Bambu_AMS_1 right after 6827... was released, and
# "spool INSERTED in slot 2" 0.7 s later; printer 1's HT the same after its
# offline releases. The edge opens a capacity window, and that window used to
# exempt the bay's first look -- so when the scan ended with no new
# measurement, the stamp already sitting in the bay was applied.

class _BusBridge:
    """The bridge as an insert edge and its scan use it: a free bus, commands
    that go nowhere, and a unit cycle that has or has not ended."""

    def __init__(self):
        self.sent = []
        self.ended = False

    def send(self, obj):
        self.sent.append(obj)

    def try_claim_bus(self, name, now):
        return True

    def bus_owner(self):
        return None

    def rfid_cycle_ended_since(self, t, addr=None):
        return self.ended

    def __getattr__(self, name):
        return lambda *a, **k: None


def _insert_edge_ready(u, model, addr):
    """Give a unit what the real insert edge in _maybe_auto_scan reads.

    :param model: ams_model
    :param addr: the unit's device address
    """
    u.SLOTS_PER_UNIT = 4
    u.auto_scan = True
    u.ams_model = model
    u.ams_index = 0
    u.unit_slots = 4
    u.calibrate_on_insert = False
    u.measure_on_insert = True
    u._scan_motion_t0 = [None] * 4
    u._scan_defer = [False] * 4
    u._untagged_rearmed = [False] * 4
    u.dry_dev_addr = addr
    u._load_in_progress = u._unload_in_progress = False
    u.afc.function = types.SimpleNamespace(in_print=lambda: False)


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
@pytest.mark.parametrize("model, addr", [("ams1", 0x0700), ("ht", 0x1800)],
                         ids=["ams1", "ht"])
def test_a_reclaims_insert_edge_does_not_apply_the_stamp_it_finds(
        section, model, addr, monkeypatch, no_spoolman_http):
    u, lane = _connected_unit(monkeypatch, section, spool_id=164, weight=400,
                              stamp=(3, 56))
    u._prev_present = [True, False, False, False]  # the previous unit's bays
    u._scan_primed = True                          # survives a reclaim
    rec = dict(u._slots[1])
    bridge = _BusBridge()
    _reclaim(u, monkeypatch, model=model, bridge=bridge)
    u._slots[1] = rec                  # the bridge still has the bay's stamp
    _insert_edge_ready(u, model, addr)
    _frames(u, 1)
    assert u.logger.having("spool INSERTED in slot 1"), "the phantom edge"
    assert u._meas_seen[1] == (3, 56), "recorded on the first look"
    # The unit ends its cycle without measuring again: the same stamp.
    bridge.ended = True
    u._slots[1] = dict(u._slots[1], scan_seq=1, scan_res=1)
    _frames(u, 4)
    assert lane.weight == 400
    assert _writes(no_spoolman_http) == []
    assert not u.logger.having("Measured")


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_prime_before_the_first_frame_does_not_apply_the_restart_stamp(
        section, monkeypatch, no_spoolman_http):
    # The same edge after a restart whose first frame lands after the 8 s
    # prime: every bay that came up present looks freshly inserted.
    u, lane = _connected_unit(monkeypatch, section, spool_id=163,
                              material="ABS", weight=600, stamp=(7, 138),
                              name="lane12")
    u._bridge = _BusBridge()
    _insert_edge_ready(u, "ams2", 0x0700)
    u._prev_present = [False] * 4
    u._scan_primed = True                  # the prime timer already fired
    _frames(u, 1)
    u._bridge.ended = True
    u._slots[1] = dict(u._slots[1], scan_seq=8, scan_res=1)
    _frames(u, 4)
    assert lane.weight == 600
    assert _writes(no_spoolman_http) == []
    assert not u.logger.having("Measured")


# ── the same percent, measured now ───────────────────────────────────────────

@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_a_capscan_after_a_restart_reading_the_same_percent_is_applied(
        section, monkeypatch, no_spoolman_http):
    # lane12 comes back from a restart carrying (7, 138), and AFC restored
    # 900 g. The operator's CAPSCAN reads 138% again -- lane12 read
    # 138/139/145/138 across calibrates -- under seq 8, and the narrated line
    # was lost. The baselined 138 was never applied, so the same number is
    # not "already on the lane": this one was taken now.
    u, lane = _connected_unit(monkeypatch, section, spool_id=163,
                              material="ABS", weight=900, stamp=(7, 138),
                              name="lane12")
    clock = [100.0]
    u.afc.reactor.monotonic = lambda: clock[0]
    _frames(u)
    _in_session(u)
    _frames(u)
    assert lane.weight == 900
    core.afcBambuAMS._cap_open_pending(u, 1, asked=True)   # the CAPSCAN
    _stamp(u, 8, 138)
    _frames(u, 3)
    assert lane.weight == 952             # 1.04 x 663 x 138%
    assert len(u.logger.having("Measured", "INFO")) == 1
    assert _writes(no_spoolman_http) == ([("set", 163, 952.0)] if section
                                         else [])
    # ...and the capscan is not reported as unanswered when its window ends.
    clock[0] += 200.0
    core.afcBambuAMS._cap_live_pending(u)
    assert not u.logger.having("capscan ended without a measurement")


def test_the_same_percent_with_no_window_open_is_not_applied(
        monkeypatch, no_spoolman_http):
    # Outside a window of ours, a new seq on the baselined number is still
    # what the percent rule says it is: the record re-serialised.
    u, lane = _connected_unit(monkeypatch, True, spool_id=163,
                              material="ABS", weight=900, stamp=(7, 138),
                              name="lane12")
    _frames(u)
    _in_session(u)
    _stamp(u, 8, 138)
    _frames(u, 3)
    assert lane.weight == 900
    assert _writes(no_spoolman_http) == []
    assert not u.logger.having("Measured")


# ── a measurement that lands before its record ──────────────────────────────
#
# Printer 1's lane14 (15:05:30-15:06:07): removed, reinserted, and its record
# still blank ("mat=- uid=-") when the unit measured 84% at 15:06:04; the tag
# landed at 15:06:07 and the bind matched spool 124, loading Spoolman's 890 g.
# The measurement had no UID to be owed to a bind by, and no material to be
# weighed at.

def _blank_insert(monkeypatch, section, spool_id=None, name="lane14"):
    """A bay reinserted with its record still blank and the insert's scan
    waiting on the unit; the lane blanked (and unbound, when spool_id is
    None) by the removal.

    :return tuple: (unit, lane, the bay's full record)
    """
    u, lane = _connected_unit(monkeypatch, section, spool_id=spool_id,
                              weight=999, material="PLA Matte", name=name)
    full = dict(u._slots[1], meas_seq=4, meas_pct=60, scan_seq=4,
                scan_res=1)
    u._slots[1] = dict(full)
    _frames(u)                            # connection start: 60% recorded
    u._prep_seen = u._scan_primed = True
    lane.material = lane.sub_type = ""
    lane.weight = 0
    core.afcBambuAMS._open_scan(u, 1)     # the insert's scan
    u._slots[1] = dict(full, material=None, rfid_uid=None, weight=None)
    _frames(u)
    assert u._scan_verdict(1) == "waiting"
    return u, lane, full


def _record_lands(u, full, pct=84):
    """The unit reads the tag: the full record, with its stamp of the
    measurement it narrated."""
    u._slots[1] = dict(full, meas_seq=5, meas_pct=pct, scan_seq=5,
                       scan_res=1)


def test_a_measurement_on_a_blank_record_is_owed_to_the_bind_after_it(
        monkeypatch, no_spoolman_http):
    u, lane, full = _blank_insert(monkeypatch, True)
    # Narrated while the record is blank: no material, no UID yet.
    assert u._adopt_measured_remain(1, 84, "capscan", seq=5) is True
    assert lane.spool_id is None
    _record_lands(u, full)
    _frames(u)                            # the tag applies, the bind goes out
    assert u.afc.spool.asked, "the bind was dispatched"
    u.afc.spool.land(890)                 # Spoolman's stored figure
    _frames(u, 3)
    # 1.24 x 663 x 84% -- the measurement, once, on the lane and the spool.
    assert lane.spool_id == 163 and lane.weight == 691
    assert _writes(no_spoolman_http) == [("set", 163, 691.0)]
    assert u._spool._bind_owed == {} and u._spool._convert_owed == {}
    assert _status(u, monkeypatch)[1]["remain_pct"] == 84
    said = u.logger.having("Measured about 84% left", "INFO")
    assert len(said) == 1 and "roughly 691 g" in said[0]


@pytest.mark.parametrize("section", [None, True],
                         ids=["no-section", "section"])
def test_grams_made_before_the_material_was_known_are_weighed_once(
        section, monkeypatch, no_spoolman_http):
    # No density at the measurement, so it went on tag-linear: 840 g for 84%.
    # Once the tag names PLA Matte the conversion is finished -- 691 g, the
    # percent reads back as 84 -- and never again after that.
    u, lane, full = _blank_insert(monkeypatch, section)
    u._adopt_measured_remain(1, 84, "capscan", seq=5)
    assert lane.weight == 840
    _record_lands(u, full)
    _frames(u)
    assert lane.weight == 691
    assert _status(u, monkeypatch)[1]["remain_pct"] == 84
    said = u.logger.having("Measured about 84% left", "INFO")
    assert len(said) == 1 and "roughly 691 g" in said[0]
    saves = list(u.saves)
    lane.weight = 650                     # printed from
    _frames(u, 3)
    assert lane.weight == 650
    assert u.saves == saves


def test_a_conversion_is_not_finished_over_a_lane_that_moved(
        monkeypatch, no_spoolman_http):
    # Only the grams it wrote are the measurement's to correct: a lane changed
    # before the material was named (by hand, by use) is newer than it.
    u, lane, full = _blank_insert(monkeypatch, None)
    u._adopt_measured_remain(1, 84, "capscan", seq=5)
    lane.weight = 800
    _record_lands(u, full)
    _frames(u, 3)
    assert lane.weight == 800


def test_a_lane_printed_to_nothing_does_not_republish_its_measurement(
        no_spoolman_http):
    # AFC clamps the count at 0 g with the reel still in the bay. The held
    # 89% is from before the print, and must not come back.
    u, lane, _obj = _measuring_unit(True, spool_id=164)
    u._adopt_measured_remain(1, 89, "capscan", seq=1.0)
    lane.weight = 5.0
    assert _published(u)[1]["remain_pct"] == 1
    lane.weight = 0
    assert _published(u)[1]["remain_pct"] == 0


# ── a stale binding rebound after the measurement ────────────────────────────
#
# A spool swapped while Klipper was down comes back with the previous reel's
# binding (spool 150). A scan measures the reel actually in the bay -- the
# adoption can only write the binding there is -- and the scan then rebinds
# the lane to the spool the tag names (163). 163 is the spool that was
# measured, and is owed it.

def test_a_stale_binding_rebound_by_a_read_less_scan_gets_the_measurement(
        monkeypatch, no_spoolman_http):
    u, lane = _connected_unit(monkeypatch, True, spool_id=150, weight=900,
                              name="lane10")
    u.afc.default_material_type = "PLA"
    u.afc.default_color = "#FFFFFF"
    u._slots[1] = dict(u._slots[1], meas_seq=4, meas_pct=60, scan_seq=4,
                       scan_res=1)
    _frames(u)
    u._prep_seen = u._scan_primed = True
    core.afcBambuAMS._open_scan(u, 1)     # AFC_BAMBU_SCAN
    # An AMS 1 record: the UID, no profile.
    u._slots[1] = dict(u._slots[1], material=None, meas_seq=5, meas_pct=89)
    assert u._adopt_measured_remain(1, 89, "capscan", seq=5) is True
    assert _writes(no_spoolman_http) == [("set", 150, 732.0)]
    # The scan ends with no readable profile: _finalize_scan unbinds 150 and
    # the read-less bind finds 163. The bay stays held on that verdict.
    u._slots[1] = dict(u._slots[1], scan_seq=5, scan_res=3)
    _frames(u, 2)
    assert u.logger.having("no readable tag profile in slot 1")
    assert u.afc.spool.asked, "the read-less bind was dispatched"
    u.afc.spool.land(1000)                # 163's stored figure
    _frames(u, 3)
    assert lane.spool_id == 163 and lane.weight == 732
    assert _writes(no_spoolman_http) == [("set", 150, 732.0),
                                         ("set", 163, 732.0)]
    assert u._spool._bind_owed == {}


def test_a_contradicted_binding_rebinds_and_gets_the_measurement(
        monkeypatch, no_spoolman_http):
    # The other rebind: the tag reads, and Spoolman says spool 150 does not
    # carry it. That branch unbinds through the unit -- it called a method
    # the delegate never had, and raised out of _surface_slot_info.
    monkeypatch.setattr(rfid.BambuSpoolman, "_binding_contradicted",
                        lambda self, bound, uid: True)
    u, lane = _connected_unit(monkeypatch, True, spool_id=150, weight=900,
                              name="lane10")
    u._slots[1] = dict(u._slots[1], meas_seq=4, meas_pct=60, scan_seq=4,
                       scan_res=1)
    _frames(u)
    u._prep_seen = u._scan_primed = True
    core.afcBambuAMS._open_scan(u, 1)
    assert u._adopt_measured_remain(1, 89, "capscan", seq=5) is True
    u._slots[1] = dict(u._slots[1], meas_seq=5, meas_pct=89, scan_seq=5,
                       scan_res=1)
    _frames(u)                            # read: unbind 150, bind by UID
    assert u.afc.spool.asked
    u.afc.spool.land(1000)
    _frames(u, 3)
    assert lane.spool_id == 163 and lane.weight == 732
    assert _writes(no_spoolman_http) == [("set", 150, 732.0),
                                         ("set", 163, 732.0)]


def test_the_readout_does_not_call_a_restart_stamp_measured(
        monkeypatch, no_spoolman_http):
    # After a restart the record still carries lane8's 63% (seq 4). It is
    # recorded and not applied, so the tag readout must not call it
    # "MEASURED (~518 g)" beside a lane that did not get those grams.
    u, lane = _connected_unit(monkeypatch, True, weight=0, stamp=(4, 63))
    lane.material = lane.sub_type = ""
    _frames(u, 3)
    readout = u.logger.having("lane8 tag --")
    assert readout, u.logger.lines
    assert "63% stamped before this connection, not applied" in readout[0]
    assert not u.logger.having("MEASURED")


# ── a tagged bay whose lane has no spool: printer 1's lane15 ─────────────────
#
# Bambu_AMS_1 slot 3 -> lane15, tag 95F2C30C, Spoolman spool 136 (card_uids
# "4B8E44F6,95F2C30C", tray_uid 4e3177c3...). A mid-print presence flap
# unbound it and saved it as material PLA with no spool. From then on every
# restart restored that lane, PREP claimed the bay into _afc_owned, and
# nothing asked Spoolman about the tag again: the owned branch of _sync_lanes
# fills the variant and continues, the boot hold returns early, and both sit
# in front of the only automatic lookup in _surface_slot_info. A capscan opens
# no scan window, so it released neither.

UID15 = "95f2c30c"
TRAY15 = "4e3177c31fea42d08bc240c818a37f80"
#: The radius the capscan narrated: 74% at the unrounded radius is 611 g of
#: PLA, where the integer percent alone gives 608.
R15_M = 0.075167


def _spoolman136(monkeypatch):
    """Printer 1's Spoolman as the tag lookup sees it: spool 136 by its roll
    or its chip UID, nothing else.

    :return list: every (uid, tray_uid) it is asked about -- a lookup
    """
    asked = []

    def _match(client, uid, tray_uid=""):
        asked.append((uid, tray_uid))
        if tray_uid == TRAY15:
            return {"id": 136, "remaining_weight": 670.8}, True, None
        if str(uid).lower() == UID15:
            return {"id": 136, "remaining_weight": 670.8}, False, None
        return None, False, None

    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    return asked


def _as_lane15(u, lane, owned=True, scanned=(), material="PLA",
               colour="#0086D6", weight=1000, **record):
    """Put printer 1's lane15 and its bay on a unit: restored from
    AFC.var.unit, PREP run, the prime fired, the bay's record carrying
    95F2C30C.

    :param owned: PREP claimed the bay (the lane came back with data)
    :param scanned: bays scanned on this connection
    :param material: the lane's restored material
    :param colour: the lane's restored colour
    :param weight: the lane's restored grams
    :param record: overrides for the bay's record
    """
    lane.name = "lane15"
    lane.material = material
    lane.sub_type = "Basic" if material == "PLA" else ""
    lane.color = colour
    lane.weight = weight
    lane.spool_vendor = lane.filament_name = ""
    lane.extruder_temp = lane.bed_temp = None
    lane.bambu_sku = None
    u.lanes = {"lane15": lane}
    u._slot_map = {"lane15": 3}
    rec = {"index": 3, "present": True, "material": "PLA Basic",
           "color": "0086D6", "weight": 1000, "rfid_uid": UID15,
           "tray_uid": TRAY15, "sku": "GFA00", "scan_seq": 0,
           "scan_res": 0, "meas_seq": 0, "meas_pct": 0,
           "reread_pending": False}
    rec.update(record)
    u._slots = [{"index": 0, "present": False},
                {"index": 1, "present": False},
                {"index": 2, "present": False}, rec]
    u._prev_present = [False, False, False, True]
    u._afc_owned = {3} if owned else set()
    u._scanned_bays = set(scanned)
    u._stamp_looked = set()
    u._prep_seen = True
    u._scan_primed = True
    u.auto_scan = False
    u.auto_spoolman_create = False
    u.afc.spool = _Spool()


def _lane15(monkeypatch, spool_id=None, **kw):
    """A real unit (built with __new__) holding lane15, with the real
    delegate and printer 1's Spoolman behind it.

    :return tuple: (unit, lane, the lookups Spoolman was asked)
    """
    u, lane, _obj = _measuring_unit(True, spool_id=spool_id,
                                    material="PLA Basic",
                                    afc_spoolman="http://spoolman:7912")
    _syncing(u)
    del u._surface_slot_info              # the real one
    u._spoolman_latched = set()           # what __init__ gives a real unit
    _as_lane15(u, lane, **kw)
    return u, lane, _spoolman136(monkeypatch)


#: The lane fields a restart restores, which nothing may touch before a bind.
_RESTORED = ("material", "sub_type", "color", "weight", "extruder_temp",
             "bed_temp", "spool_id")


def _restored(lane):
    return {k: getattr(lane, k, None) for k in _RESTORED}


def _capscan_74(u):
    """AFC_BAMBU_CAPSCAN on lane15's bay: a capacity window of ours, no scan
    window, and the radius the unit narrates for its 74%."""
    u._bridge = types.SimpleNamespace(
        last_cap_measure=lambda addr: {"pct_raw": 74, "save_radius_m": R15_M})
    core.afcBambuAMS._cap_open_pending(u, 3, asked=True)


def _measured_74(u, **record):
    """The unit re-reads the tag and stamps its 74% on the bay."""
    u._slots[3] = dict(u._slots[3], scan_seq=1, scan_res=1, meas_seq=1,
                       meas_pct=74, **record)


def test_a_restart_looks_up_a_claimed_lane_with_no_spool_once(
        monkeypatch, no_spoolman_http):
    u, lane, asked = _lane15(monkeypatch)
    before = _restored(lane)
    _frames(u, 3)
    assert asked == [(UID15, TRAY15)], "one lookup, roll identity first"
    assert u.afc.spool.asked == [(lane, 136)]
    assert u._spoolman_latched == {3}
    # Nothing restored moved before the bind; the owned branch's variant fill
    # (vendor and name) is the only write, as it always was.
    assert _restored(lane) == before
    assert (lane.spool_vendor, lane.filament_name) == ("Bambu",
                                                       "Bambu PLA Basic")
    assert not u.logger.having("applied tag to lane15")
    assert 3 in u._afc_owned
    _frames(u, 5)
    assert asked == [(UID15, TRAY15)], "no second lookup"
    # The bind lands: AFC loads Spoolman's remaining weight, as it does for
    # every bound lane at a restart, and nothing of ours asks again.
    u.afc.spool.land(670.8)
    _frames(u, 5)
    assert lane.spool_id == 136 and lane.weight == 670.8
    assert asked == [(UID15, TRAY15)]
    assert _writes(no_spoolman_http) == []


def test_a_claimed_lane_that_has_its_spool_is_not_looked_up(
        monkeypatch, no_spoolman_http):
    u, lane, asked = _lane15(monkeypatch, spool_id=109)
    _frames(u, 5)
    assert asked == [] and u.afc.spool.asked == []
    assert u._spoolman_latched == set()
    assert lane.spool_id == 109


@pytest.mark.parametrize("stale", ["material", "colour", "no-colour",
                                   "claimed-elsewhere"])
def test_a_record_that_does_not_describe_the_lane_binds_nothing(
        stale, monkeypatch, no_spoolman_http):
    # A spool swapped while the bridge was down leaves the bay's record
    # describing the one that left, a lane on defaults has no colour to test
    # a record against, and a UID another unit also reports is a copied
    # record, not a reading. None of them may bind a spool to this lane.
    kw = {"material": {"material": "PETG"}, "colour": {"colour": "#FF0000"},
          "no-colour": {"colour": ""}}.get(stale, {})
    u, lane, asked = _lane15(monkeypatch, **kw)
    if stale == "claimed-elsewhere":
        other = core.afcBambuAMS.__new__(core.afcBambuAMS)
        other._slots = [{"index": 0, "present": True, "rfid_uid": "95F2C30C"}]
        u.afc.units = {"Bambu_AMS_1": u, "Bambu_AMS_2": other}
    _frames(u, 5)
    assert asked == [] and u.afc.spool.asked == []
    assert lane.spool_id is None
    assert u._spoolman_latched == set()


def test_the_boot_hold_asks_once_and_writes_no_profile_field(
        monkeypatch, no_spoolman_http):
    # The second gate on its own: the bay is not claimed, but it has not been
    # scanned on this connection and the lane has material, so the hold
    # returns before the dispatch -- and still returns before every write.
    u, lane, asked = _lane15(monkeypatch, owned=False)
    lane.spool_vendor, lane.filament_name = "Bambu", "Bambu PLA Basic"
    assert core.afcBambuAMS._boot_hold(u, lane, u._slots[3]) is True
    before = dict(_restored(lane), spool_vendor=lane.spool_vendor,
                  filament_name=lane.filament_name)
    _frames(u, 3)
    assert asked == [(UID15, TRAY15)]
    assert dict(_restored(lane), spool_vendor=lane.spool_vendor,
                filament_name=lane.filament_name) == before
    assert not u.logger.having("applied tag to lane15")
    assert not u.logger.having("filling in")
    _frames(u, 5)
    assert asked == [(UID15, TRAY15)], "the latch holds it to one"
    u.afc.spool.land(670.8)
    _frames(u, 3)
    assert lane.spool_id == 136 and asked == [(UID15, TRAY15)]


@pytest.mark.parametrize("owned", [True, False],
                         ids=["claimed-at-prep", "boot-hold"])
def test_a_capscan_on_an_unbound_lane_hands_its_grams_to_the_spool(
        owned, monkeypatch, no_spoolman_http):
    # AFC_BAMBU_CAPSCAN LANE=lane15 on 2026-09-23: a capacity window and no
    # scan window, so the verdict stays "none" and the bay stays held. The
    # lookup waits for the measurement, the measurement is owed to the bind
    # the lookup makes, and spool 136 gets the measured 611 g -- not the
    # 670.8 g Spoolman had stored.
    u, lane, asked = _lane15(monkeypatch, owned=owned)
    _capscan_74(u)
    _frames(u, 3)
    assert asked == [], "no lookup while the window is open"
    assert core.afcBambuAMS._scan_verdict(u, 3) == "none"
    _measured_74(u)
    _frames(u, 1)
    assert asked == [(UID15, TRAY15)], "asked once the measurement was in"
    assert lane.weight == 611 and lane.spool_id is None
    assert u._spool._bind_owed == {3: (74, 1000, None, True)}
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "roughly 611 g" in said[0]
    assert ("a Spoolman lookup for 95F2C30C is in progress; the measurement "
            "goes to the spool it finds") in said[0]
    assert _writes(no_spoolman_http) == [], "nothing bound to write to yet"
    u.afc.spool.land(670.8)                # AFC loads Spoolman's stored figure
    _frames(u, 3)
    assert lane.spool_id == 136 and lane.weight == 611
    assert _writes(no_spoolman_http) == [("set", 136, 611.0)]
    assert u._spool._bind_owed == {}
    _frames(u, 3)
    assert _writes(no_spoolman_http) == [("set", 136, 611.0)], "once"
    assert asked == [(UID15, TRAY15)]
    assert len(u.logger.having("Measured about 74% left", "INFO")) == 1


@pytest.mark.parametrize("rrq", [True, False],
                         ids=["reread-flagged", "reread-not-flagged"])
def test_a_measurement_on_a_blanked_profile_is_owed_to_the_lookup(
        rrq, monkeypatch, no_spoolman_http):
    # The firmware blanks a bay's profile when a capacity window opens and
    # re-reads it after, while the chip and tray UIDs stay published -- so the
    # measurement can land on a record with a UID and no material. It is owed
    # to the lookup that the profile's return lets go out.
    u, lane, asked = _lane15(monkeypatch)
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u, reread_pending=rrq, material=None, sku=None, color=None)
    _frames(u, 1)
    assert asked == [], "nothing to test the lane against yet"
    assert lane.weight == 611
    assert u._spool._bind_owed == {3: (74, 1000, None, False)}
    assert not u.logger.having("Measured about 74% left"), \
        "said before the lookup it waits for"
    u._slots[3] = dict(u._slots[3], material="PLA Basic", sku="GFA00",
                       color="0086D6", reread_pending=False)
    _frames(u, 1)
    assert asked == [(UID15, TRAY15)]
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1 and "in progress" in said[0]
    u.afc.spool.land(670.8)
    _frames(u, 3)
    assert lane.spool_id == 136 and lane.weight == 611
    assert _writes(no_spoolman_http) == [("set", 136, 611.0)]


# ── what the restart lookup will and will not bind ──────────────────────────
#
# AFC's set_spoolID rejects a spool with no remaining weight on record by
# CLEARING the lane (AFC_spool._apply_spool_data, unless
# disable_weight_check). The restart lookup binds a lane AFC restored, so a
# match whose Spoolman weight is missing or not above zero would wipe
# lane15's material, colour and grams. Refused, the lane stays as restored
# and the reader is told which spool to correct.

#: Spoolman's shape for a spool it cannot weigh: the key is left out.
ABSENT = object()


def _spoolman136_weighing(monkeypatch, remaining):
    """Spoolman answering 95F2C30C with spool 136 carrying ``remaining`` g.

    :param remaining: the grams, or ABSENT for a record without the key
    :return list: the UIDs it is asked about
    """
    asked = []

    def _match(client, uid, tray_uid=""):
        asked.append(uid)
        spool = {"id": 136, "used_weight": 0.0}
        if remaining is not ABSENT:
            spool["remaining_weight"] = remaining
        return spool, True, None
    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    return asked


@pytest.mark.parametrize("remaining", [0.0, -2.5, None, ABSENT],
                         ids=["zero", "negative", "null", "absent"])
def test_a_restored_lane_is_not_bound_to_a_spool_afc_would_clear_it_for(
        remaining, monkeypatch, no_spoolman_http):
    u, lane, _asked = _lane15(monkeypatch)
    asked = _spoolman136_weighing(monkeypatch, remaining)
    before = _restored(lane)
    _frames(u, 3)
    assert asked == [UID15]
    assert u.afc.spool.asked == [], "set_spoolID would clear lane15"
    assert _restored(lane) == before
    said = u.logger.having("has no remaining weight on record", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "matches Spoolman spool 136" in said[0]
    assert "run AFC_BAMBU_SCAN LANE=lane15 to link it" in said[0]
    _frames(u, 5)
    assert asked == [UID15] and u.afc.spool.asked == []
    assert len(u.logger.having("has no remaining weight on record")) == 1
    # A capscan's summary names the spool and what to do about it.
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u)
    _frames(u, 2)
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert ("95F2C30C matches Spoolman spool 136, which has no remaining "
            "weight on record; correct spool 136's weight in Spoolman, then "
            "run AFC_BAMBU_SCAN LANE=lane15 to link it") in said[0]
    assert asked == [UID15]


@pytest.mark.parametrize("case", ["weighed", "weight-check-off"])
def test_a_restored_lane_binds_a_spool_afc_will_keep(
        case, monkeypatch, no_spoolman_http):
    u, lane, _asked = _lane15(monkeypatch)
    asked = _spoolman136_weighing(
        monkeypatch, 670.8 if case == "weighed" else 0.0)
    if case == "weight-check-off":
        u.afc.spool.disable_weight_check = True
    _frames(u, 3)
    assert asked == [UID15]
    assert u.afc.spool.asked == [(lane, 136)]
    assert not u.logger.having("has no remaining weight on record")


def test_a_read_of_the_bay_binds_whatever_spoolman_says(
        monkeypatch, no_spoolman_http):
    # Only the lookup this module makes on its own for a lane AFC restored
    # is refused. A read of the bay is a spool being linked, and AFC's own
    # check stays the last word there.
    u, lane, _asked = _lane15(monkeypatch, owned=False, scanned=(3,),
                              material="")
    asked = _spoolman136_weighing(monkeypatch, 0.0)
    _frames(u, 3)
    assert asked == [UID15]
    assert u.afc.spool.asked == [(lane, 136)]
    assert not u.logger.having("has no remaining weight on record")


@pytest.mark.parametrize("spoolman", ["zero-weight", "unknown-uid"])
def test_a_restart_lookup_is_match_only_whatever_auto_create_says(
        spoolman, monkeypatch, no_spoolman_http):
    # AFC_BridgeBox hands auto_spoolman_create: True to every unit, and the
    # create path, for a UID Spoolman does not know, makes a spool at the
    # tag's nominal 1000 g that set_spoolID then loads over the restored
    # grams. The restart lookup only matches.
    u, lane, _asked = _lane15(monkeypatch, weight=611)
    u.auto_spoolman_create = True
    monkeypatch.setattr(rfid, "get_auto_spoolman_create",
                        lambda ln, default: True)
    created = []

    def _sync(afc, ln, si, logger, source, allow_create=False, on_done=None,
              **k):
        created.append((ln.name, allow_create))
    monkeypatch.setattr(rfid, "sync_rfid_to_spoolman", _sync)
    if spoolman == "zero-weight":
        asked = _spoolman136_weighing(monkeypatch, 0.0)
    else:
        asked = []

        def _match(client, uid, tray_uid=""):
            asked.append(uid)
            return None, False, None
        monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    before = _restored(lane)
    _frames(u, 3)
    assert created == [], "the restart lookup took the create path"
    assert asked == [UID15]
    assert u.afc.spool.asked == []
    assert _restored(lane) == before and lane.weight == 611
    _frames(u, 5)
    assert created == [] and asked == [UID15]


def test_a_restart_lookup_spoolman_did_not_answer_is_asked_again(
        monkeypatch):
    # A cold boot: the lookup goes out on the first frame after PREP, before
    # Spoolman (or Moonraker's proxy) answers. search_spools gives [] on an
    # error, which reads as "no spool carries 95F2C30C" -- a miss the server
    # never gave. It is asked again instead.
    u, lane, _asked = _lane15(monkeypatch, weight=611)
    clock = [100.0]
    u.afc.reactor = types.SimpleNamespace(monotonic=lambda: clock[0])
    up = [False]
    asked, writes = [], []
    client = types.SimpleNamespace(
        reachable=lambda: up[0],
        set_remaining_weight=lambda sid, g: writes.append((sid, g)))
    monkeypatch.setattr(rfid, "_bambu_spoolman_client", lambda afc: client)
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: job())

    def _match(c, uid, tray_uid=""):
        asked.append(uid)
        if not up[0]:
            return None, False, None       # search_spools gave [] on error
        return {"id": 136, "remaining_weight": 670.8}, True, None
    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    _frames(u, 3)
    assert asked == [UID15] and lane.spool_id is None
    assert not u._spool._spoolman_no_match, "no answer recorded as a miss"
    assert 3 not in u._spoolman_latched
    _frames(u, 5)
    assert asked == [UID15], "waiting LOOKUP_RETRY_S, not asking at 1 Hz"
    # A capscan while it waits: the measurement is owed to the lookup still
    # to come, and nothing says Spoolman has no such spool.
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u)
    _frames(u, 2)
    assert lane.weight == 611 and asked == [UID15]
    assert u._spool._bind_owed.get(3) is not None
    up[0] = True                           # Spoolman is up
    clock[0] += u.LOOKUP_RETRY_S
    _frames(u, 2)
    assert asked == [UID15, UID15]
    assert u.afc.spool.asked == [(lane, 136)]
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1 and "in progress" in said[0], u.logger.lines
    u.afc.spool.land(670.8)
    _frames(u, 3)
    assert lane.spool_id == 136 and lane.weight == 611
    assert writes == [(136, 611.0)]
    assert not u.logger.having("Spoolman has no spool carrying")
    _frames(u, 5)
    assert asked == [UID15, UID15]


def _spoolman136_later(monkeypatch):
    """Printer 1's Spoolman before spool 136 carries 95F2C30C, and after.

    :return tuple: (the UIDs it is asked about, a one-item list: set its
        item True once the spool is in Spoolman)
    """
    known = [False]
    asked = []

    def _match(client, uid, tray_uid=""):
        asked.append(uid)
        return (({"id": 136, "remaining_weight": 670.8}, True, None)
                if known[0] else (None, False, None))
    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    return asked, known


def _scan_command(u, monkeypatch):
    """AFC_BAMBU_SCAN LANE=lane15 through the real command, the capscan it
    issues recorded (and its capacity window opened, as the real one does).

    :return list: the capscans issued
    """
    issued = []

    def _capscan(self, slot, cali=False, insert=False, **k):
        issued.append(slot)
        core.afcBambuAMS._cap_open_pending(self, slot, asked=not insert)
        return True
    monkeypatch.setattr(core.afcBambuAMS, "_start_capscan", _capscan)
    monkeypatch.setattr(core.afcBambuAMS, "_measure_in_flight_slot",
                        lambda self: None)
    g = _GCmd(LANE="lane15")
    core.afcBambuAMS.cmd_AFC_BAMBU_SCAN(u, g)
    assert g.said and issued == [3], "the command must have run"
    return issued


def test_a_missed_restart_lookup_is_said_as_a_miss_and_a_scan_asks_again(
        monkeypatch, no_spoolman_http):
    u, lane, _asked = _lane15(monkeypatch)
    asked, known = _spoolman136_later(monkeypatch)
    u._auto_scanned = [False] * 4
    u._scan_notag = [False] * 4
    u._scan_t0 = [None] * 4
    _frames(u, 3)
    assert asked == [UID15] and lane.spool_id is None
    assert u._spoolman_latched == {3}
    # A capscan measures 74%: the bay's one lookup is spent, so nothing is
    # asked, and the summary says what that lookup answered.
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u)
    _frames(u, 3)
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "Spoolman has no spool carrying 95F2C30C" in said[0]
    assert asked == [UID15], "a capscan re-arms nothing"
    assert _writes(no_spoolman_http) == []
    # The spool is in Spoolman now, and AFC_BAMBU_SCAN asks again.
    known[0] = True
    _scan_command(u, monkeypatch)
    assert 3 not in u._spoolman_latched
    _frames(u, 2)
    u._slots[3] = dict(u._slots[3], scan_seq=2, scan_res=1, meas_seq=2,
                       meas_pct=73)
    _frames(u, 3)
    assert asked == [UID15, UID15]
    assert u.afc.spool.asked == [(lane, 136)]
    u.afc.spool.land(670.8)
    _frames(u, 3)
    assert lane.spool_id == 136 and lane.weight != 670.8
    assert [w[1] for w in _writes(no_spoolman_http)] == [136]
    _frames(u, 3)
    assert asked == [UID15, UID15]


# ── a fresh process ─────────────────────────────────────────────────────────

class _UnitConfig:
    """Just enough ConfigWrapper for afcBambuAMS.__init__ to run."""

    def __new__(cls, **values):
        from tests.conftest import MockConfig

        class _C(MockConfig):
            def getchoice(self, option, choices, default=None, **kw):
                return self._require(option, default)

        return _C(name="AFC_BambuAMS Bambu_AMS_1",
                  values=dict({"serial_port": "/dev/fake"}, **values))


def _fresh_unit(section=True):
    """A unit Klipper built: through __init__, so the latch is its own.

    :return tuple: (unit, lane, the section object)
    """
    u = core.afcBambuAMS(_UnitConfig())
    obj = rfid.load_config(_Config(enabled=section))
    u.printer._objects["AFC_BambuAMS_rfid"] = obj
    u.logger = _Log()
    u.saves = []
    u.afc = types.SimpleNamespace(
        save_vars=lambda: u.saves.append(1),
        reactor=types.SimpleNamespace(monotonic=lambda: 100.0),
        spoolman="http://spoolman:7912", moonraker=object())
    lane = types.SimpleNamespace(name="lane15", spool_id=None, density=None,
                                 tool_loaded=False)
    u.lane_loaded = lambda ln: None
    u.lane_illuminate_spool = lambda ln: None
    u._save_lane_vars = lambda: u.saves.append(1)
    return u, lane, obj


def test_a_unit_klipper_builds_starts_with_an_empty_latch():
    u, _lane, _obj = _fresh_unit()
    assert u._spoolman_latched == set()
    assert u._scanned_bays == set()
    assert u._removed_bays == set() and u._meas_departed == {}
    assert u._cleared_bays == set() and u._defaults_due == {}


@pytest.mark.parametrize("restored", ["PLA", "PETG"],
                         ids=["lane15-as-restored", "spool-swapped-while-down"])
def test_a_fresh_processs_first_scan_hands_its_measurement_to_the_bind(
        restored, monkeypatch, no_spoolman_http):
    # AFC_BAMBU_SCAN LANE=lane15 in a process that had dispatched nothing yet.
    # With no latch, _sync_owed and _bind_coming read "no bind coming", the
    # measurement taken in the scan was owed to nothing, and the lane took
    # Spoolman's 670.8 g over the measured figure. The latch here is
    # __init__'s own. A lane restored as PETG while the bay holds the PLA
    # reel is not the restart lookup's (its record does not describe the
    # lane), so that case rests on the latch alone.
    u, lane, _obj = _fresh_unit()
    _as_lane15(u, lane, material=restored)
    asked = _spoolman136(monkeypatch)
    u._bridge = types.SimpleNamespace(
        last_cap_measure=lambda addr: {"pct_raw": 74, "save_radius_m": R15_M})
    u._prep_seen = False
    _frames(u, 2)                          # connection start, before PREP
    core.afcBambuAMS._open_scan(u, 3)      # AFC_BAMBU_SCAN
    u._prep_seen = True
    _frames(u, 2)
    assert core.afcBambuAMS._scan_verdict(u, 3) == "waiting"
    assert asked == []
    _measured_74(u)                        # the unit reads and measures
    _frames(u, 1)
    assert asked == [(UID15, TRAY15)]
    assert u._spool._bind_owed == {3: (74, 1000, None, True)}
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1 and "no Spoolman lookup" not in said[0]
    u.afc.spool.land(670.8)
    _frames(u, 3)
    assert lane.spool_id == 136 and lane.weight == 611
    assert _writes(no_spoolman_http) == [("set", 136, 611.0)]


# ── a spool that goes in during a print ─────────────────────────────────────
#
# 2026-09-21 22:40:59, printer 1 at "Change 18 out of 116": one frame said
# Bambu_AMS_1 slot 3 was empty and the next, 0.24 s later, said it was back.
# The removal edge cleared lane15 and unbound spool 136; the same pass put the
# empty bay's leftover record straight back on the cleared lane and spent the
# one-shot latch on a sync that returns for an empty bay; the insert edge was
# dropped for the print. Driven here from raw bridge frames through
# _on_status, so the whole path is the real one.

class _FlapBridge:
    """Records anything the unit asks of the bridge -- a scan, a pull."""

    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)

        def _rec(*a, **k):
            self.calls.append(name)
        return _rec


def _raw_slot(slot, present, tagged=True, **fields):
    """One raw 'slots' entry as the bridge firmware sends it.

    :param fields: raw keys to override (sseq, sres, mseq, mpct, uid, ...)
    """
    e = {"unit": 0, "i": slot, "present": 1 if present else 0,
         "state": "loaded" if present else "empty", "rrq": 0, "sseq": 0,
         "sres": 0, "mseq": 0, "mpct": 0}
    if tagged:
        e.update({"material": "PLA Basic", "sku": "GFA00",
                  "color": "0086D6FF", "tmin": 190, "tmax": 230,
                  "weight": 1000, "uid": UID15, "tray_uid": TRAY15,
                  "remain": 0})
    e.update(fields)
    return e


def _raw_frame(present3, ht=False, tagged=True, **fields):
    """Slot 0 empty and slot 3 lane15's bay -- or, on an HT, lane15's bay as
    its one slot.

    :param ht: the one-bay layout of an AMS HT
    :param tagged: the bay's record carries lane15's tag
    :param fields: raw keys to override on lane15's bay's entry
    """
    if ht:
        return {"slots": [_raw_slot(0, present3, tagged, **fields)]}
    return {"slots": [_raw_slot(0, False, tagged=False),
                      _raw_slot(3, present3, tagged, **fields)]}


def _gone_frame(ht=False, **fields):
    """lane15's bay emptied as the firmware sees a removal: present 0 and the
    record blanked (its presence decoder clears it on the edge it sees)."""
    return _raw_frame(False, ht, tagged=False, **fields)


#: The reel put in in the swap cases: a PETG reel Spoolman knows as 200.
UID_NEW = "aabbccdd"
TRAY_NEW = "f" * 32


def _new_reel(ht=False, **fields):
    """lane15's bay once the unit has read the PETG reel, and measured it."""
    return _raw_frame(True, ht, **dict(
        {"material": "PETG Basic", "sku": "GFG00", "color": "FF0000FF",
         "tmin": 230, "tmax": 260, "uid": UID_NEW, "tray_uid": TRAY_NEW,
         "sseq": 1, "sres": 1, "mseq": 7, "mpct": 95}, **fields))


class _SetSpool:
    """AFC core's spool object, binding at once."""

    def __init__(self):
        self.asked = []

    def set_spoolID(self, lane, sid, *a, **k):
        self.asked.append((lane.name, sid))
        lane.spool_id = sid


def _feed(u, frame, dt=0.24):
    """One raw status frame, ``dt`` seconds after the last."""
    u.clock[0] += dt
    core.afcBambuAMS._on_status(u, frame)


def _flap_unit(scanned=(), stamp=(0, 0), ht=False):
    """lane15 mid-print: bound to 136, bay present, PREP run and primed, the
    bay claimed by AFC.

    :param scanned: bays scanned on this connection
    :param stamp: (meas_seq, meas_pct) the bay's record carries
    :param ht: an AMS HT, lane15 in its one bay (slot 0), instead
    :return tuple: (unit, lane)
    """
    slot = 0 if ht else 3
    lane = types.SimpleNamespace(
        name="lane15", index=4, spool_id=136, material="PLA",
        sub_type="Basic", spool_vendor="Bambu",
        filament_name="Bambu PLA Basic", color="#0086D6", weight=670,
        density=None, tool_loaded=False, status=core.AFCLaneState.LOADED,
        extruder_temp=190.0, bed_temp=55.0, bambu_sku="GFA00",
        bambu_slot_info=None, prep_state=True, loaded_to_hub=True,
        _load_state=False, send_lane_data=lambda: None)
    u = core.afcBambuAMS.__new__(core.afcBambuAMS)
    u.name = "Bambu_AMS_1"
    u.logger = _Log()
    u.logger.error = u.logger.warning
    u.pool = False
    u.unit_uid = ""
    u.ams_index = 0
    u.ams_model = "ht" if ht else "ams1"
    u.has_heater = False
    u.dry_dev_addr = 0x1800 if ht else 0x0700
    u._status_err_last = None
    u.lanes = {"lane15": lane}
    u._slot_map = {"lane15": slot}
    u.unit_slots = 1 if ht else 4
    u._slots = [{} for _ in range(4)]
    u._prev_present = [False] * 4
    u._auto_scanned = [False] * 4
    u._scan_notag = [False] * 4
    u._scan_t0 = [None] * 4
    u._scan_motion_t0 = [None] * 4
    u._scan_defer = [False] * 4
    u._untagged_rearmed = [False] * 4
    u._scan_primed = False
    u._prep_seen = False
    u._afc_owned = set()
    u._scanned_bays = set(scanned)
    u._spoolman_latched = set()
    u._stamp_looked = set()
    u._meas_seen = {}
    u._meas_baselined = set()
    u._unload_in_progress = u._load_in_progress = False
    u.auto_scan = True
    u.auto_spoolman_create = False
    u._bridge = _FlapBridge()
    u.clock = [126000.0]
    u.printing = [True]
    u.saves = []
    u.afc = types.SimpleNamespace(
        spoolman="http://spoolman:7912", spool=_SetSpool(), moonraker=None,
        units={}, lanes={"lane15": lane}, prep_done=True,
        default_material_type="PLA",       # both printers' AFC.cfg
        save_vars=lambda: u.saves.append(1),
        function=types.SimpleNamespace(in_print=lambda: u.printing[0]),
        reactor=types.SimpleNamespace(
            monotonic=lambda: u.clock[0],
            register_async_callback=lambda cb: cb(),
            register_callback=lambda cb, t=None: None,
            register_timer=lambda cb, t=None: None, NOW=0.0, NEVER=9e99))
    u._spool_obj = rfid.BambuSpoolman(u)
    u._spool_looked_up = True
    u.lane_loaded = u.lane_illuminate_spool = u.lane_not_ready = \
        lambda ln: None
    seq, pct = stamp
    _feed(u, _raw_frame(True, ht, mseq=seq, mpct=pct))  # the baseline frame
    u._prep_seen = True
    u._afc_owned.add(slot)                 # PREP: the lane has data
    u._scan_primed = True
    for _ in range(3):
        _feed(u, _raw_frame(True, ht, mseq=seq, mpct=pct), dt=1.0)
    return u, lane


def _flap_spoolman(monkeypatch):
    """Printer 1's Spoolman behind a flap unit, answered on the spot: spool
    136 by lane15's tag, spool 200 by the PETG reel's. A capscan a scan
    starts is recorded instead of sent, and opens the window the real one
    opens.

    :return tuple: (the lookups Spoolman was asked, the capscans started,
        the weight writes)
    """
    asked = _spoolman136(monkeypatch)
    only136 = rfid.match_spool_for_tag

    def _match(client, uid, tray_uid=""):
        if tray_uid == TRAY_NEW or str(uid).lower() == UID_NEW:
            asked.append((uid, tray_uid))
            return ({"id": 200, "remaining_weight": 800.0},
                    tray_uid == TRAY_NEW, None)
        return only136(client, uid, tray_uid)
    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    writes = []
    monkeypatch.setattr(rfid, "_bambu_spoolman_client",
                        lambda afc: types.SimpleNamespace(
                            set_remaining_weight=lambda sid, g:
                                writes.append((sid, g))))
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: job())
    capscans = []

    def _capscan(self, slot, cali=False, insert=False, **k):
        capscans.append(slot)
        core.afcBambuAMS._cap_open_pending(self, slot, asked=not insert)
        return True
    monkeypatch.setattr(core.afcBambuAMS, "_start_capscan", _capscan)
    return asked, capscans, writes


#: What lane defaults leave on lane15 (default_material_type PLA; AFC has no
#: default colour).
_DEFAULTS = {"material": "PLA", "color": "", "sub_type": "",
             "filament_name": "", "spool_vendor": ""}


def _profile(lane):
    return {k: getattr(lane, k, None) for k in _DEFAULTS}


@pytest.mark.parametrize("scanned", [(), (3,)],
                         ids=["bay-never-scanned", "bay-already-scanned"])
def test_a_mid_print_flap_finds_its_spool_again_without_moving_anything(
        scanned, monkeypatch):
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned)
    assert lane.spool_id == 136 and asked == []
    # The frame that says slot 3 is empty, its record kept: a removal the
    # firmware's own presence decoder never saw (one it sees blanks the
    # record).
    _feed(u, _raw_frame(False))
    assert u.logger.having("spool REMOVED from slot 3", "INFO")
    assert u.logger.having("unbinding lane15 from spool 136 -- the bay is "
                           "empty", "DEBUG")
    assert not u.logger.having("applied tag to lane15"), \
        "an empty bay's record describes the spool that left"
    assert lane.spool_id in (None, "", 0) and lane.material == ""
    assert 3 not in u._spoolman_latched, "the one-shot is not spent on it"
    assert asked == []
    # Back 0.24 s later, its record whole throughout: the same spool, linked
    # again from its record on this frame, and not put on lane defaults.
    _feed(u, _raw_frame(True))
    assert u.logger.having("spool INSERTED in slot 3", "INFO")
    assert asked == [(UID15, TRAY15)]
    assert lane.spool_id == 136 and lane.material == "PLA"
    assert lane.color == "#0086D6" and lane.sub_type == "Basic"
    assert u.afc.spool.asked == [("lane15", 136)]
    assert not u.logger.having("nothing has read it")
    saves = len(u.saves)
    for _ in range(20):
        _feed(u, _raw_frame(True), dt=1.0)
    # Past the settle a real insert waits out: still no lane defaults.
    assert not u.logger.having("nothing has read it")
    assert len(u.saves) == saves
    assert _profile(lane)["color"] == "#0086D6" and lane.spool_id == 136
    u.printing[0] = False
    for _ in range(60):
        _feed(u, _raw_frame(True), dt=1.0)
    assert asked == [(UID15, TRAY15)], "one lookup"
    assert u.afc.spool.asked == [("lane15", 136)]
    assert u._bridge.calls == [] and capscans == [], "nothing moved"
    assert not [m for lv, m in u.logger.lines if lv == "WARNING"]


@pytest.mark.parametrize("scanned", [(), (3,)],
                         ids=["bay-never-scanned", "bay-already-scanned"])
def test_a_flap_does_not_hand_back_the_stamp_its_spool_left_with(
        scanned, monkeypatch):
    # The firmware keeps the stamp through a flap it never saw and publishes
    # it again, on the empty frame and on the re-insert. The removal edge
    # forgets the stamp's identity (a new spool must be able to establish a
    # first reading), so the old stamp read as a measurement never seen:
    # lane15 at an 80% stamp printed down to 400 g would have had 800 g
    # written to spool 136 on the relink.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned, stamp=(3, 80))
    lane.weight = 400                      # hours of printing since
    writes.clear()
    _feed(u, _raw_frame(False, mseq=3, mpct=80))
    assert u._meas_departed == {3: (3, 80)}
    for _ in range(20):                    # back, and the print goes on
        _feed(u, _raw_frame(True, mseq=3, mpct=80), dt=1.0)
    u.printing[0] = False
    for _ in range(10):
        _feed(u, _raw_frame(True, mseq=3, mpct=80), dt=1.0)
    assert lane.spool_id == 136 and asked == [(UID15, TRAY15)]
    assert writes == [], "the departed measurement was written to spool 136"
    assert not u.logger.having("Measured about 80% left")
    assert not u.logger.having("nothing has read it")
    assert lane.weight != 800
    # A measurement the unit takes now is new, and is the spool's.
    for _ in range(3):
        _feed(u, _raw_frame(True, mseq=4, mpct=78, sseq=1, sres=1), dt=1.0)
    assert u.logger.having("Measured about 78% left")
    assert writes == [(136, float(lane.weight))], writes
    for _ in range(5):
        _feed(u, _raw_frame(True, mseq=4, mpct=78, sseq=1, sres=1), dt=1.0)
    assert len(writes) == 1, "once"


def test_a_departed_stamp_goes_when_the_bays_sequence_starts_again(
        monkeypatch):
    # The same (seq, pct) after the counter started again is a measurement
    # taken now, not the one the spool left with. (A removal the firmware
    # sees clears the percent and keeps the sequence.)
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(stamp=(1, 80))
    writes.clear()
    _feed(u, _raw_frame(False, mseq=1, mpct=-1))
    assert u._meas_departed == {3: (1, 80)}
    for _ in range(3):
        _feed(u, _raw_frame(True, mseq=0, mpct=0), dt=1.0)
    assert u._meas_departed == {}
    for _ in range(3):
        _feed(u, _raw_frame(True, mseq=1, mpct=80), dt=1.0)
    assert u.logger.having("Measured about 80% left")
    assert writes == [(136, float(lane.weight))], writes


@pytest.mark.parametrize("insert", ["blank", "reread-owed", "ht-blank"])
def test_a_mid_print_insert_waits_on_lane_defaults_for_its_read(
        insert, monkeypatch):
    # A real removal (the firmware blanked the record) and a spool put in
    # during the print. Nothing scans during a print, and the record says
    # nothing about the new spool -- it is blank, or the departed reel's with
    # the re-read the firmware owes it -- so the lane takes AFC's defaults,
    # linked to nothing, and the reader is told once how to have it read.
    # The unit's own read then applies the tag and links its spool.
    ht = insert == "ht-blank"
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(ht=ht)
    saved = []
    u.afc.save_vars = lambda: saved.append(
        dict(_profile(lane), spool_id=lane.spool_id or None))
    writes.clear()
    for _ in range(3):
        _feed(u, _gone_frame(ht), dt=1.0)
    assert lane.spool_id in (None, "", 0)
    frame = (_raw_frame(True, ht, rrq=1) if insert == "reread-owed"
             else _raw_frame(True, ht, tagged=False))
    for _ in range(8):
        _feed(u, frame, dt=1.0)
    assert asked == [] and u.afc.spool.asked == [], "linked before a read"
    assert lane.spool_id in (None, "", 0)
    assert _profile(lane) == _DEFAULTS
    assert not u.logger.having("applied tag to lane15")
    assert saved[-1] == dict(_DEFAULTS, spool_id=None), "defaults are saved"
    said = u.logger.having("nothing has read it", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "AFC_BAMBU_SCAN LANE=lane15" in said[0]
    assert u._bridge.calls == [] and capscans == [], "nothing moved"
    # The unit reads the PETG reel on its own, and measures it.
    for _ in range(5):
        _feed(u, _new_reel(ht), dt=1.0)
    assert asked == [(UID_NEW, TRAY_NEW)], asked
    assert u.afc.spool.asked == [("lane15", 200)]
    assert lane.spool_id == 200 and lane.material == "PETG"
    assert lane.color == "#FF0000" and lane.sub_type == "Basic"
    assert writes and {sid for sid, _g in writes} == {200}, writes
    said = u.logger.having("Measured about 95% left", "INFO")
    assert len(said) == 1 and "no Spoolman lookup" not in said[0]
    for _ in range(5):
        _feed(u, _new_reel(ht), dt=1.0)
    assert asked == [(UID_NEW, TRAY_NEW)], "one lookup"
    assert set(u._bridge.calls) <= {"last_cap_measure"}, u._bridge.calls
    assert capscans == []
    assert len(u.logger.having("nothing has read it")) == 1


@pytest.mark.parametrize("link", ["hiccup", "reboot"])
def test_a_bridge_reconnect_puts_no_lane_on_defaults(link, monkeypatch):
    # _on_bridge_reconnect forgets the connection's presence history. A
    # rebooted Pico reports every bay empty before its first presence poll
    # (printer 1: "bridge reconnected after 4s down", REMOVED 24 ms later,
    # INSERTED 1.5 s after that with blank records), and that removal is not
    # one this connection saw: the insert that follows mid-print is no new
    # spool, and gets no defaults. Its record coming back links it again.
    # These frames carry no units list, as from a firmware that does not
    # publish preslen; current firmware's reboot never reaches the lanes
    # (test_a_rebooted_bridge_only_answers_whether_the_bay_is_occupied).
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    core.afcBambuAMS._on_bridge_reconnect(u)
    assert u._removed_bays == set() and u._present_seen == set()
    if link == "reboot":
        _feed(u, _gone_frame())
        for _ in range(3):
            _feed(u, _raw_frame(True, tagged=False), dt=1.0)
        assert u.logger.having("spool INSERTED in slot 3", "INFO")
    for _ in range(5):
        _feed(u, _raw_frame(True), dt=1.0)
    assert not u.logger.having("nothing has read it")
    assert lane.spool_id == 136 and lane.material == "PLA"
    assert lane.color == "#0086D6"
    assert asked == ([] if link == "hiccup" else [(UID15, TRAY15)])
    assert capscans == []



# ── a bridge that has not asked yet ────────────────────────────────────────
# What a Pico publishes between booting and the unit's first presence reply:
# every bay empty, every record blank, and preslen 0 for the unit (the length
# of the last presence reply the firmware accepted; zero only until the
# first). After that, the bays as the unit reports them -- and an AMS 1 does
# not hand a tag back unasked, so an occupied bay's record stays blank
# (printer 1, 2026-09-22 19:36-19:47, lane12: mat=- uid=- until a reseat).

def _unit_entry(preslen):
    return {"n": 0, "online": bool(preslen), "preslen": preslen,
            "presbyte": 8 if preslen else 0}


def _booting_frame(ht=False):
    """The frames before the unit's first presence reply."""
    f = _raw_frame(False, ht, tagged=False, mpct=-1)
    f["units"] = [_unit_entry(0)]
    return f


def _polled_frame(present3=True, ht=False, tagged=False, **fields):
    """A frame after it: lane15's bay as the unit reports it."""
    f = _raw_frame(present3, ht, tagged=tagged, **dict({"mpct": -1}, **fields))
    f["units"] = [_unit_entry(60)]
    return f


_TOUCHED = ("REMOVED", "INSERTED", "unbinding", "clearing it",
            "nothing has read", "applied lane defaults", "scanning tag")


def _untouched(u, lane):
    assert lane.spool_id == 136
    assert lane.material == "PLA" and lane.color == "#0086D6"
    assert lane.sub_type == "Basic"
    assert lane.status == core.AFCLaneState.LOADED and lane.prep_state
    for text in _TOUCHED:
        assert not u.logger.having(text), text
    assert [c for c in u._bridge.calls if c != "request_info"] == []


@pytest.mark.parametrize("printing", [True, False], ids=["printing", "idle"])
@pytest.mark.parametrize("tag_back", [False, True],
                         ids=["record-blank", "record-back"])
@pytest.mark.parametrize("ht", [False, True], ids=["ams1", "ht"])
def test_a_rebooted_bridge_only_answers_whether_the_bay_is_occupied(
        printing, tag_back, ht, monkeypatch):
    # Printer 1 lost lane12's link to spool 150 exactly this way: the Pico
    # rebooted after a flash, its first frames called bay 1 empty, the host
    # unbound the lane, and the insert 1.8 s later started a tag scan. The
    # lane is AFC's saved state; the bridge only says whether the bay is
    # still occupied, and it cannot say that until it has asked.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(0 if ht else 3,), ht=ht)
    u.printing[0] = printing
    core.afcBambuAMS._on_bridge_reconnect(u)
    for _ in range(7):
        _feed(u, _booting_frame(ht))
    assert u._presence_ok is False
    for _ in range(20):
        _feed(u, _polled_frame(True, ht, tagged=tag_back), dt=1.0)
    assert u._presence_ok is True
    _untouched(u, lane)
    assert asked == [] and capscans == [] and writes == []
    assert u.afc.spool.asked == []


@pytest.mark.parametrize("ht", [False, True], ids=["ams1", "ht"])
def test_a_bay_emptied_while_the_bridge_was_down_is_cleared(ht, monkeypatch):
    # The other half of the same answer: occupied before, empty once the
    # unit has been asked, is a removal like any other.
    _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(0 if ht else 3,), ht=ht)
    u.printing[0] = False
    core.afcBambuAMS._on_bridge_reconnect(u)
    for _ in range(7):
        _feed(u, _booting_frame(ht))
    assert not u.logger.having("REMOVED")
    _feed(u, _polled_frame(False, ht), dt=1.0)
    assert u.logger.having("REMOVED")
    assert lane.spool_id in (None, "", 0)


def _restarting(u):
    """_flap_unit's unit as a Klipper restart leaves it before its first
    frame: _handle_disconnect's reset, the bridge rebuilt, PREP run from
    AFC.var.unit (lane15 as saved), priming not yet due."""
    u.serial_port = "/dev/null-bridge"
    core.afcBambuAMS._handle_disconnect(u)
    u._bridge = _FlapBridge()
    u._afc_owned = {3}


@pytest.mark.parametrize("wait", ["booting", "no-frames"])
def test_priming_waits_for_the_bridge_to_have_asked(wait, monkeypatch):
    # Priming (eight seconds after ready) reconciles the lanes against the
    # bays: an empty bay clears its lane. With the Pico still booting --
    # or not connected yet -- every bay read empty and every lane with data
    # was cleared. It now waits for the first frame from a bridge that has
    # asked, and runs on it.
    asked, capscans, _w = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(3,))
    u.printing[0] = False
    _restarting(u)
    if wait == "booting":
        for _ in range(7):
            _feed(u, _booting_frame())
    core.afcBambuAMS._prime_scan_baseline(u)          # the 8 s timer
    assert u._scan_primed is False and u._prime_waiting is True
    _untouched(u, lane)
    _feed(u, _polled_frame(True), dt=1.0)
    assert u._scan_primed is True and u._prime_waiting is False
    for _ in range(10):
        _feed(u, _polled_frame(True), dt=1.0)
    _untouched(u, lane)
    assert capscans == []


def test_priming_on_the_first_answer_still_clears_an_empty_bay(monkeypatch):
    # A spool pulled while Klipper was down: the saved lane still holds it,
    # and the bridge, once it has asked, says the bay is empty.
    _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(3,))
    u.printing[0] = False
    _restarting(u)
    for _ in range(7):
        _feed(u, _booting_frame())
    core.afcBambuAMS._prime_scan_baseline(u)
    assert lane.spool_id == 136
    _feed(u, _polled_frame(False), dt=1.0)
    assert u._scan_primed is True
    assert u.logger.having("clearing it")
    assert lane.spool_id in (None, "", 0)


def test_priming_waits_for_the_re_read_a_booted_bridge_owes(monkeypatch):
    # After a Pico boot every occupied boxed bay is an insert edge to the
    # firmware: its first answer carries rrq=1 and a blank record, and the
    # tag lands a frame or two later. Primed on that first answer, a lane
    # with nothing restored got its defaults, and the tag that followed was
    # held off it as restored state.
    asked, capscans, _w = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    u.printing[0] = False
    _restarting(u)
    u._afc_owned = set()
    for k, v in (("spool_id", None), ("material", ""), ("color", ""),
                 ("sub_type", ""), ("filament_name", ""),
                 ("spool_vendor", "")):
        setattr(lane, k, v)
    for _ in range(7):
        _feed(u, _booting_frame())
    core.afcBambuAMS._prime_scan_baseline(u)
    _feed(u, _polled_frame(True, rrq=1), dt=1.0)
    _feed(u, _polled_frame(True, rrq=1), dt=1.0)
    assert u._scan_primed is False
    for _ in range(6):
        _feed(u, _polled_frame(True, tagged=True), dt=1.0)
    assert u._scan_primed is True
    assert not u.logger.having("applied lane defaults")
    assert not u.logger.having("nothing has read")
    assert lane.material == "PLA" and lane.color == "#0086D6"
    assert capscans == []


@pytest.mark.parametrize("ht", [False, True], ids=["ams1", "ht"])
def test_a_tag_after_restart_defaults_is_written_over_them(ht, monkeypatch):
    # An HT's first answer after a Pico boot has no rrq and a blank record;
    # its cache hands the tag back a moment later. Priming on that answer
    # gives a lane with nothing restored its defaults -- and the tag that
    # follows is the read those defaults stood in for, not restored state.
    asked, capscans, _w = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(ht=ht)
    u.printing[0] = True
    _restarting(u)
    u._afc_owned = set()
    for k, v in (("spool_id", None), ("material", ""), ("color", ""),
                 ("sub_type", ""), ("filament_name", ""),
                 ("spool_vendor", "")):
        setattr(lane, k, v)
    for _ in range(7):
        _feed(u, _booting_frame(ht))
    core.afcBambuAMS._prime_scan_baseline(u)
    _feed(u, _polled_frame(True, ht), dt=1.0)
    assert u._scan_primed is True
    assert u.logger.having("applied lane defaults")
    for _ in range(6):
        _feed(u, _polled_frame(True, ht, tagged=True), dt=1.0)
    assert lane.material == "PLA" and lane.color == "#0086D6"
    assert lane.spool_id == 136
    assert capscans == []


def test_a_re_read_that_never_settles_does_not_keep_priming_off(monkeypatch):
    _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(3,))
    u.printing[0] = False
    _restarting(u)
    _feed(u, _polled_frame(True, rrq=1), dt=1.0)
    core.afcBambuAMS._prime_scan_baseline(u)
    assert u._scan_primed is False
    for _ in range(int(core.afcBambuAMS.PRIME_REREAD_WAIT_S) + 2):
        _feed(u, _polled_frame(True, rrq=1), dt=1.0)
    assert u._scan_primed is True
    _untouched(u, lane)


def test_a_bay_emptied_while_the_bridge_was_down_waits_on_defaults(
        monkeypatch):
    # Emptied while the Pico was rebooting, mid-print, then refilled: the
    # removal is a real one (the booting frames never reached the lanes), so
    # the new spool gets what a removal seen live gets -- lane defaults, no
    # link, and the line naming the scan.
    asked, capscans, _w = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(3,))
    core.afcBambuAMS._on_bridge_reconnect(u)
    for _ in range(7):
        _feed(u, _booting_frame())
    _feed(u, _polled_frame(False), dt=1.0)
    assert u.logger.having("REMOVED")
    for _ in range(5):
        _feed(u, _polled_frame(True), dt=1.0)
    assert u.logger.having("AFC_BAMBU_SCAN LANE=lane15")
    assert lane.material == "PLA" and lane.spool_id in (None, "", 0)
    assert asked == [] and capscans == []
    assert [c for c in u._bridge.calls if c != "request_info"] == []


def _scanning(u, seq0):
    """lane15's bay with a tag scan open on it, baselined at ``seq0``."""
    for _ in range(2):
        _feed(u, _raw_frame(True, sseq=seq0), dt=1.0)
    core.afcBambuAMS._open_scan(u, 3)
    assert u._scan_t0[3] is not None


@pytest.mark.parametrize("seq0", [0, 2])
@pytest.mark.parametrize("tag_back", [False, True],
                         ids=["record-blank", "record-back"])
def test_a_scan_a_bridge_reboot_killed_leaves_the_lane_alone(
        seq0, tag_back, monkeypatch):
    # A reboot zeroes every bay's scan_seq and takes the scan window with
    # it; the counter's move (or, from 0, the verdict cap) read as the
    # unit's "no tag", which unbinds the lane and puts it on defaults.
    asked, capscans, _w = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(3,))
    u.printing[0] = False
    _scanning(u, seq0)
    core.afcBambuAMS._on_bridge_reconnect(u)
    for _ in range(7):
        _feed(u, _booting_frame())
    _feed(u, _polled_frame(True, tagged=tag_back, sseq=0), dt=1.0)
    assert u._scan_t0[3] is None
    assert u.logger.having("AFC_BAMBU_SCAN LANE=lane15 to scan it again")
    for _ in range(int(core.afcBambuAMS.SCAN_VERDICT_CAP) + 5):
        _feed(u, _polled_frame(True, tagged=tag_back, sseq=0), dt=1.0)
    assert lane.spool_id == 136 and lane.material == "PLA"
    assert not u.logger.having("unbinding")
    assert not u.logger.having("applied lane defaults")


def test_a_scan_open_across_a_hiccup_keeps_going(monkeypatch):
    # A link drop without a reboot: the firmware's window and counter
    # survive, so the scan is still the unit's to answer.
    _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(3,))
    u.printing[0] = False
    _scanning(u, 2)
    core.afcBambuAMS._on_bridge_reconnect(u)
    _feed(u, _polled_frame(True, tagged=True, sseq=2), dt=1.0)
    assert u._scan_t0[3] is not None
    assert not u.logger.having("to scan it again")


def test_a_reboot_seen_only_as_a_counter_going_back_ends_the_scan(
        monkeypatch):
    # A reconnect late enough that the Pico had already polled the unit:
    # no booting frames, but the bay's scan counter went backwards.
    _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(scanned=(3,))
    u.printing[0] = False
    _scanning(u, 2)
    core.afcBambuAMS._on_bridge_reconnect(u)
    _feed(u, _polled_frame(True, sseq=0), dt=1.0)
    assert u._scan_t0[3] is None
    assert u.logger.having("AFC_BAMBU_SCAN LANE=lane15 to scan it again")
    assert lane.spool_id == 136


@pytest.mark.parametrize("units", [
    None, [], [{"n": 0, "online": True}], [{"n": 0, "preslen": None}],
    [{"n": 0, "preslen": "x"}], [{"n": 0, "preslen": 60}]],
    ids=["no-list", "empty-list", "older-firmware", "null", "garbage",
         "asked"])
def test_a_frame_that_cannot_say_is_trusted_as_before(units):
    u = types.SimpleNamespace(ams_index=0)
    f = {"slots": []}
    if units is not None:
        f["units"] = units
    assert core.afcBambuAMS._presence_known(u, f) is True


def test_a_unit_the_bridge_is_not_polling_yet_is_not_known():
    # A rebooted Pico polls only the units it has been told of; the HT at
    # index 4 is not listed until the announce.
    u = types.SimpleNamespace(ams_index=4)
    f = {"units": [{"n": 0, "preslen": 60}], "slots": []}
    assert core.afcBambuAMS._presence_known(u, f) is False
    assert core.afcBambuAMS._presence_known(
        u, {"units": [{"n": 0, "preslen": 60}, {"n": 4, "preslen": 0}]}) \
        is False
    assert core.afcBambuAMS._presence_known(
        u, {"units": [{"n": 4, "preslen": 60}]}) is True

@pytest.mark.parametrize("owned", [True, False],
                         ids=["claimed-at-prep", "boot-hold"])
@pytest.mark.parametrize("saved_as", ["defaults", "its-tag"])
def test_a_restart_onto_saved_defaults_links_and_fills_nothing(
        saved_as, owned, monkeypatch, no_spoolman_http):
    # Restored from the defaults a mid-print insert saved -- material PLA,
    # no colour, no variant, no spool -- with the bay still reporting the
    # departed reel's record (95F2C30C, PLA Basic, 0086D6), a restart neither
    # dresses the lane from that record nor looks the tag up: nothing on the
    # lane says the record is its spool's. The control: the same lane saved
    # with that tag's profile is looked up.
    colour = "" if saved_as == "defaults" else "#0086D6"
    u, lane, asked = _lane15(monkeypatch, owned=owned, colour=colour)
    if saved_as == "defaults":
        lane.sub_type = ""
    _frames(u, 5)
    if saved_as == "its-tag":
        assert asked == [(UID15, TRAY15)]
        assert u.afc.spool.asked == [(lane, 136)]
        return
    assert asked == [] and u.afc.spool.asked == []
    assert _profile(lane) == _DEFAULTS
    assert not u.logger.having("filling in")
    assert not u.logger.having("applied tag to lane15")
    assert 3 not in u._spoolman_latched
    # A scan of the bay is what links it.
    u._auto_scanned = [False] * 4
    u._scan_notag = [False] * 4
    u._bridge = types.SimpleNamespace(
        last_cap_measure=lambda addr: {"pct_raw": 74, "save_radius_m": R15_M})
    _scan_command(u, monkeypatch)
    _measured_74(u)
    _frames(u, 3)
    assert asked == [(UID15, TRAY15)]
    assert u.afc.spool.asked == [(lane, 136)]


# ── pool spares (AFC_BridgeBox) ─────────────────────────────────────────────

def test_an_unclaimed_pool_spare_is_never_looked_up(monkeypatch,
                                                     no_spoolman_http):
    # An idle spare holds no unit's bays; even handed a present, tagged
    # record it asks nothing, owes nothing, and builds no delegate.
    u, lane, _obj = _measuring_unit(True, pool=True,
                                    afc_spoolman="http://spoolman:7912")
    _syncing(u)
    del u._surface_slot_info
    u._spoolman_latched = set()
    _as_lane15(u, lane)
    asked = _spoolman136(monkeypatch)
    info = u._slots[3]
    assert core.afcBambuAMS._unbound_lookup_state(u, lane, info) == "no"
    assert not core.afcBambuAMS._lookup_coming(u, 3, info)
    core.afcBambuAMS._lookup_unbound(u, lane, info)
    assert asked == [] and u._spoolman_latched == set()
    assert u._spool_obj is None and u.lookups == []


@pytest.mark.parametrize("tag", ["another-reel", "same-tag-added-since"])
def test_each_claim_of_a_pool_spare_gets_its_own_lookup(
        tag, monkeypatch, no_spoolman_http):
    # The spare object outlives its claims. Claimed onto one AMS, lane15's bay
    # was asked about (a miss: Spoolman had no spool with 95F2C30C yet) and
    # scanned. Claimed onto another AMS, that bay's latch, scan and miss are
    # the old claim's, and must not stop this one's lookup.
    u, lane, _asked = _lane15(monkeypatch)
    known = [False]
    asked = []

    def _match(client, uid, tray_uid=""):
        asked.append(str(uid).lower())
        if str(uid).lower() == UID_NEW:
            return {"id": 200, "remaining_weight": 800.0}, False, None
        return (({"id": 136, "remaining_weight": 670.8}, False, None)
                if known[0] else (None, False, None))
    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    u.pool = True
    _reclaim(u, monkeypatch)
    _as_lane15(u, lane, owned=False)
    core.afcBambuAMS._prep_claimed_lanes(u)
    _frames(u, 3)
    assert asked == [UID15] and lane.spool_id is None
    assert u._spoolman_latched == {3}
    assert {m.lower() for m in u._spool._spoolman_no_match} == {UID15}
    u._scanned_bays.add(3)                  # scanned under this claim
    u._removed_bays.add(3)
    u._cleared_bays.add(3)
    u._meas_departed[3] = (5, 60)
    slots, present = [dict(r) for r in u._slots], list(u._prev_present)
    # Released, and claimed onto another AMS.
    _reclaim(u, monkeypatch, uid="0123456789ABCDEF01234567")
    assert u._spoolman_latched == set()
    assert u._scanned_bays == set()
    assert u._removed_bays == set() and u._meas_departed == {}
    assert u._cleared_bays == set()
    assert u._spool._spoolman_no_match == set()
    if tag == "another-reel":
        new_uid, sid = UID_NEW, 200
        slots[3].update(rfid_uid=UID_NEW, tray_uid=None)
    else:
        new_uid, sid = UID15, 136
        known[0] = True
    u._slots, u._prev_present = slots, present
    core.afcBambuAMS._prep_claimed_lanes(u)
    _frames(u, 3)
    assert asked == [UID15, new_uid], "one lookup for this claim"
    assert u.afc.spool.asked == [(lane, sid)]
    _frames(u, 3)
    assert asked == [UID15, new_uid]


# ── what the first review round found ───────────────────────────────────────

def test_a_flap_relink_with_auto_create_on_runs_spoolman_off_the_reactor(
        monkeypatch):
    # Printer 1 runs auto_spoolman_create on, so the relink after a mid-print
    # flap takes the create path -- and that path ran its Spoolman HTTP inline
    # in the status pass, with the print running: ten seconds a call when
    # Spoolman is slow or gone. It is handed the reactor, so the HTTP runs on
    # moonraker's writer thread and the lane binds when the answer comes back.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    u.auto_spoolman_create = True
    calls, worker = [], []

    def _sync(afc, ln, si, logger, prefix, allow_create=False,
              set_next=False, reactor=None, on_done=None):
        calls.append((ln.name, allow_create, reactor is afc.reactor))

        def _answer():
            afc.spool.set_spoolID(ln, 136)
            on_done()
        if reactor is None:
            _answer()                      # inline, in this status pass
        else:
            worker.append(_answer)         # moonraker's writer thread
    monkeypatch.setattr(rfid, "sync_rfid_to_spoolman", _sync)
    _feed(u, _raw_frame(False))
    _feed(u, _raw_frame(True))
    assert calls == [("lane15", True, True)], "Spoolman ran on the reactor"
    assert lane.spool_id in (None, "", 0), "bound before the answer came"
    worker.pop()()                         # the answer comes back
    assert lane.spool_id == 136
    for _ in range(10):
        _feed(u, _raw_frame(True), dt=1.0)
    assert len(calls) == 1 and u.afc.spool.asked == [("lane15", 136)]
    assert not u._spool._spoolman_no_match, "a bind recorded as a miss"
    assert u._spool._bound_uid.get(3) == UID15
    assert u._bridge.calls == [] and capscans == []


def test_a_flap_after_a_repeat_reading_does_not_hand_back_its_stamp(
        monkeypatch):
    # Two capscans read 80%. The repeat moves the record's sequence (1 -> 2)
    # and not the pair last adopted, which stays (1, 80) -- so the flap's
    # (2, 80) matched no departed stamp taken from that pair, and 80% was
    # adopted again over what the print had used since.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    u.printing[0] = False
    for seq in (1, 2):
        core.afcBambuAMS._cap_open_pending(u, 3, asked=True)
        for _ in range(3):
            _feed(u, _raw_frame(True, mseq=seq, mpct=80), dt=1.0)
        core.afcBambuAMS._cap_close_pending(u, 3)
    assert u._meas_seen[3] == (1, 80)
    assert len(u.logger.having("Measured about 80% left", "INFO")) == 1
    u.printing[0] = True
    lane.weight = 400                      # printed since
    writes.clear()
    _feed(u, _raw_frame(False, mseq=2, mpct=80))
    assert u._meas_departed == {3: (2, 80)}
    for _ in range(20):
        _feed(u, _raw_frame(True, mseq=2, mpct=80), dt=1.0)
    u.printing[0] = False
    for _ in range(10):
        _feed(u, _raw_frame(True, mseq=2, mpct=80), dt=1.0)
    assert lane.spool_id == 136 and lane.weight != 658   # 80% of PLA
    assert writes == [], "the departed measurement was written to spool 136"
    assert len(u.logger.having("Measured about 80% left", "INFO")) == 1


@pytest.mark.parametrize("owned", [True, False],
                         ids=["claimed-at-prep", "boot-hold"])
@pytest.mark.parametrize("colour", ["", "#000000"],
                         ids=["dressed-from-the-tag", "bound-since"])
def test_a_black_reels_restored_lane_is_looked_up(colour, owned, monkeypatch,
                                                  no_spoolman_http):
    # A black tag's record carries no colour (bridge_color_to_rgb maps 000000
    # to None), so neither does the lane it dressed -- or the lane carries
    # Spoolman's "#000000" from a bind since. Its variant says the record is
    # its spool's, where a lane on defaults has none.
    u, lane, asked = _lane15(monkeypatch, owned=owned, colour=colour,
                             color=None, color_black=True)
    before = _restored(lane)
    _frames(u, 3)
    assert asked == [(UID15, TRAY15)]
    assert u.afc.spool.asked == [(lane, 136)]
    assert _restored(lane) == before
    if owned:
        assert (lane.spool_vendor, lane.filament_name) == ("Bambu",
                                                           "Bambu PLA Basic")
    _frames(u, 3)
    assert asked == [(UID15, TRAY15)]


@pytest.mark.parametrize("owned", [True, False],
                         ids=["claimed-at-prep", "boot-hold"])
def test_a_lane_on_defaults_is_not_linked_to_a_black_record(
        owned, monkeypatch, no_spoolman_http):
    u, lane, asked = _lane15(monkeypatch, owned=owned, colour="", color=None,
                             color_black=True)
    lane.sub_type = ""
    _frames(u, 5)
    assert asked == [] and u.afc.spool.asked == []
    assert _profile(lane) == _DEFAULTS


def test_one_stray_occupied_frame_puts_no_lane_on_defaults(monkeypatch):
    # Printer 1, 22:40:59: a bay empty since a real removal read INSERTED on
    # one frame and REMOVED 0.24 s later, 7 ms after a select ack. No spool
    # went in -- no defaults, no save for them, and no line telling the
    # operator to scan an empty bay.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    for _ in range(3):
        _feed(u, _gone_frame(), dt=1.0)
    assert 3 in u._removed_bays
    saves = len(u.saves)
    _feed(u, _raw_frame(True, tagged=False))
    _feed(u, _gone_frame())
    for _ in range(5):
        _feed(u, _gone_frame(), dt=1.0)
    assert u.logger.having("spool INSERTED in slot 3", "INFO")
    assert not u.logger.having("nothing has read it")
    assert lane.material == "" and lane.spool_id in (None, "", 0)
    assert len(u.saves) == saves + 1, "only the removal's own save"
    assert u._defaults_due == {}


def test_a_bay_that_reads_occupied_for_under_the_settle_gets_no_defaults(
        monkeypatch):
    # Occupied for 1.8 s, then empty again: shorter than DEFAULTS_SETTLE_S,
    # so still no spool to put defaults on. Occupied for longer, it is one.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    for _ in range(3):
        _feed(u, _gone_frame(), dt=1.0)
    saves = len(u.saves)
    for _ in range(4):
        _feed(u, _raw_frame(True, tagged=False), dt=0.6)
    _feed(u, _gone_frame())
    for _ in range(5):
        _feed(u, _gone_frame(), dt=1.0)
    assert not u.logger.having("nothing has read it")
    assert lane.material == "" and len(u.saves) == saves + 1
    for _ in range(5):
        _feed(u, _raw_frame(True, tagged=False), dt=0.6)
    assert len(u.logger.having("nothing has read it", "INFO")) == 1
    assert _profile(lane) == _DEFAULTS


@pytest.mark.parametrize("insert", ["blank", "reread-owed", "ht-blank"])
def test_a_link_drop_between_a_mid_print_insert_and_its_read_keeps_the_read(
        insert, monkeypatch):
    # The bridge link drops and comes back (a WiFi blip, AFC_BAMBU_RELINK)
    # after lane15 went on defaults and before the unit read the new reel.
    # The reconnect forgets the link's presence history, but this process's
    # removal edge still cleared the lane: the boot hold must not keep the
    # read off it.
    ht = insert == "ht-blank"
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit(ht=ht)
    for _ in range(3):
        _feed(u, _gone_frame(ht), dt=1.0)
    frame = (_raw_frame(True, ht, rrq=1) if insert == "reread-owed"
             else _raw_frame(True, ht, tagged=False))
    for _ in range(8):
        _feed(u, frame, dt=1.0)
    assert _profile(lane) == _DEFAULTS
    core.afcBambuAMS._on_bridge_reconnect(u)
    for _ in range(3):
        _feed(u, frame, dt=1.0)
    for _ in range(10):
        _feed(u, _new_reel(ht), dt=1.0)
    assert asked == [(UID_NEW, TRAY_NEW)], asked
    assert lane.spool_id == 200 and lane.material == "PETG"
    assert lane.color == "#FF0000"
    assert capscans == []
    assert len(u.logger.having("nothing has read it")) == 1


def test_a_pico_reboot_after_a_mid_print_swap_puts_no_lane_on_defaults(
        monkeypatch):
    # What the removal edge leaves behind outlives the link only for the boot
    # hold. The swapped reel was read and linked; then the Pico reboots
    # mid-print, reports the bay empty and brings it back blank until the
    # record is read again. No new spool went in.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    for _ in range(3):
        _feed(u, _gone_frame(), dt=1.0)
    for _ in range(3):
        _feed(u, _raw_frame(True, tagged=False), dt=1.0)
    for _ in range(5):
        _feed(u, _new_reel(), dt=1.0)
    assert lane.spool_id == 200
    core.afcBambuAMS._on_bridge_reconnect(u)
    _feed(u, _gone_frame())
    for _ in range(5):
        _feed(u, _raw_frame(True, tagged=False), dt=1.0)
    assert len(u.logger.having("nothing has read it")) == 1, "the swap's own"
    for _ in range(5):                     # counters restarted with the Pico
        _feed(u, _new_reel(sseq=0, sres=0, mseq=0, mpct=0), dt=1.0)
    assert lane.spool_id == 200 and lane.material == "PETG"


@pytest.mark.parametrize("link", ["steady", "reconnected"])
def test_a_refusal_is_still_the_answer_after_a_bridge_reconnect(
        link, monkeypatch, no_spoolman_http):
    # The bay stays latched after a refusal, and a link drop re-arms nothing,
    # so the refusal has to last as long as the latch. Forgotten, the capscan
    # summary sent the operator to AFC_BAMBU_SCAN, whose read binds the 0 g
    # spool -- and AFC clears lane15 for it.
    u, lane, _asked = _lane15(monkeypatch)
    asked = _spoolman136_weighing(monkeypatch, 0.0)
    _frames(u, 3)
    assert u._lookup_refused.get(3) == (UID15, 136)
    if link == "reconnected":
        u._bridge = types.SimpleNamespace(request_info=lambda: None)
        u.afc.reactor.register_callback = lambda cb, t=None: None
        core.afcBambuAMS._on_bridge_reconnect(u)
        _frames(u, 3)
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u)
    _frames(u, 2)
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert ("95F2C30C matches Spoolman spool 136, which has no remaining "
            "weight on record; correct spool 136's weight in Spoolman") \
        in said[0]
    assert asked == [UID15]


def _spoolman_down(monkeypatch):
    """Spoolman not answering: a search comes back empty (search_spools
    gives [] on an error) and a reachability probe fails. The worker's jobs
    are queued, to run when the test says.

    :return tuple: (the UIDs asked about, the queued jobs)
    """
    asked, queued = [], []

    def _match(client, uid, tray_uid=""):
        asked.append(str(uid).lower())
        return None, False, None
    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    monkeypatch.setattr(rfid, "_bambu_spoolman_client",
                        lambda afc: types.SimpleNamespace(
                            reachable=lambda: False,
                            set_remaining_weight=lambda sid, g: None))
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: queued.append(job))
    return asked, queued


@pytest.mark.parametrize("meanwhile", ["flap", "out-and-back", "emptied"])
def test_a_late_unanswered_lookup_leaves_no_retry_behind(meanwhile,
                                                          monkeypatch):
    # With Spoolman timing out, a restored lane's lookup comes back up to
    # 20 s later, and the bay can change in between. A "no answer" about a
    # spool that has left re-armed a retry for an empty bay; one that lands
    # after the bay's own lookup has taken over left a retry nothing would
    # send, and every summary after it said a lookup was in progress.
    u, lane = _flap_unit()
    u.printing[0] = False
    u.auto_scan = False
    asked, queued = _spoolman_down(monkeypatch)
    u._lookup_retry = {}                   # what __init__ gives a real unit
    lane.spool_id = None                   # restored with no spool
    _feed(u, _raw_frame(True), dt=1.0)
    assert len(queued) == 1 and u._spoolman_latched == {3}
    rec = {}
    if meanwhile == "flap":
        _feed(u, _raw_frame(False))
        _feed(u, _raw_frame(True))
    else:
        for _ in range(2):
            _feed(u, _gone_frame(), dt=1.0)
    if meanwhile == "out-and-back":
        for _ in range(2):
            _feed(u, _raw_frame(True, tagged=False, rrq=1), dt=1.0)
        rec = {"sseq": 1, "sres": 1}
        for _ in range(3):
            _feed(u, _raw_frame(True, **rec), dt=1.0)
    queued.pop(0)()                        # the restart lookup: no answer
    if meanwhile == "emptied":
        assert u._lookup_retry == {} and u._spoolman_latched == set()
        return
    for _ in range(5):
        _feed(u, _raw_frame(True, **rec), dt=1.0)
        while queued:
            queued.pop(0)()
    assert u._lookup_retry == {}
    u.clock[0] += 120.0
    n = len(asked)
    for _ in range(5):
        _feed(u, _raw_frame(True, **rec), dt=1.0)
        while queued:
            queued.pop(0)()
    assert len(asked) == n, "asked again with nothing to ask for"
    core.afcBambuAMS._cap_open_pending(u, 3, asked=True)
    for _ in range(3):
        _feed(u, _raw_frame(True, mseq=5, mpct=74, **rec), dt=1.0)
        while queued:
            queued.pop(0)()
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "in progress" not in said[0]


# ── what the second review round found ──────────────────────────────────────

@pytest.mark.parametrize("printing", [True, False], ids=["mid-print", "idle"])
def test_a_create_path_answer_that_binds_nothing_is_said_as_a_miss(
        printing, monkeypatch):
    # Printer 1 runs auto_spoolman_create on, so a reel read after a swap is
    # bound through the create path. When that answers without a spool --
    # Spoolman unreachable, a create refused -- the measurement held for the
    # bind has nowhere to go, and the summary says what the answer was.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    u.printing[0] = printing
    u.auto_spoolman_create = True
    worker = []

    def _sync(afc, ln, si, logger, prefix, allow_create=False,
              set_next=False, reactor=None, on_done=None):
        worker.append(on_done)             # answers later, binding nothing
    monkeypatch.setattr(rfid, "sync_rfid_to_spoolman", _sync)
    for _ in range(3):
        _feed(u, _gone_frame(), dt=1.0)
    for _ in range(3):
        _feed(u, _raw_frame(True, tagged=False), dt=1.0)
    for _ in range(5):
        _feed(u, _new_reel(), dt=1.0)
    assert len(worker) == 1
    assert u._spool._bind_owed.get(3) == (95, 1000, None, True)
    worker.pop()()
    for _ in range(3):
        _feed(u, _new_reel(), dt=1.0)
    said = u.logger.having("Measured about 95% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "in progress" not in said[0]
    assert "Spoolman has no spool carrying AABBCCDD" in said[0]
    assert u._spool._bind_owed == {}
    # Linked by hand afterwards: the operator's choice, handed nothing.
    u.afc.spool.set_spoolID(lane, 999)
    for _ in range(3):
        _feed(u, _new_reel(), dt=1.0)
    assert not [w for w in writes if w[0] == 999], writes


class _FetchingSpool:
    """AFC core's spool object as AFC_spool has it: set_spoolID fetches the
    spool from moonraker, puts Spoolman's record on the lane when the fetch
    lands, and only then fires on_done."""

    def __init__(self):
        self.asked, self.fetching = [], []

    def set_spoolID(self, lane, sid, save_vars=True, on_done=None):
        self.asked.append((lane.name, sid))
        self.fetching.append((lane, sid, on_done))

    def land(self, stored=670.8):
        """The fetches answer with spool 136's record."""
        fetching, self.fetching = self.fetching, []
        for lane, sid, on_done in fetching:
            lane.spool_id, lane.weight = sid, stored
            lane.material, lane.color = "PLA", "#0086D6"
            if on_done is not None:
                on_done()


@pytest.mark.parametrize("lands", ["bay-empty", "after-defaults"])
@pytest.mark.parametrize("path", ["create", "match"])
def test_a_link_that_lands_after_its_spool_left_is_undone(path, lands,
                                                          monkeypatch):
    # A mid-print flap relinks lane15; the reel is then really pulled while
    # Spoolman answers, and the answer binds the lane of an emptied bay. The
    # removal edge has already been and gone, so nothing else would unbind
    # it -- and a reel put in after it would be charged to spool 136 under
    # lane defaults that say it is linked to nothing.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    if path == "create":
        u.auto_spoolman_create = True
        worker = []

        def _sync(afc, ln, si, logger, prefix, allow_create=False,
                  set_next=False, reactor=None, on_done=None):
            def _answer():
                ln.spool_id, ln.material, ln.color = 136, "PLA", "#0086D6"
                on_done()
            worker.append(_answer)
        monkeypatch.setattr(rfid, "sync_rfid_to_spoolman", _sync)
        land = lambda: worker.pop()()      # noqa: E731
    else:
        u.afc.spool = _FetchingSpool()
        land = u.afc.spool.land
    _feed(u, _raw_frame(False))
    _feed(u, _raw_frame(True))
    assert lane.spool_id in (None, "", 0), "the relink has not landed yet"
    _feed(u, _gone_frame())
    unread = _raw_frame(True, tagged=False)
    if lands == "after-defaults":
        for _ in range(4):
            _feed(u, unread, dt=1.0)
        assert _profile(lane) == _DEFAULTS
    land()
    assert lane.spool_id in (None, "", 0)
    assert u._spool._bound_uid.get(3) is None
    assert u._spool._bind_pending == set()
    if lands == "bay-empty":
        assert lane.material == ""
        for _ in range(2):
            _feed(u, _gone_frame(), dt=1.0)
    for _ in range(8):
        _feed(u, unread, dt=1.0)
    assert lane.spool_id in (None, "", 0)
    assert _profile(lane) == _DEFAULTS
    assert len(u.logger.having("nothing has read it", "INFO")) == 1
    assert u._bridge.calls == [] and capscans == []


@pytest.mark.parametrize("lands", ["while-booting", "after-first-poll"])
def test_a_link_in_flight_across_a_bridge_reboot_is_kept(lands, monkeypatch):
    # The relink after a flap is still being fetched when the Pico reboots.
    # Once the bridge has polled the bay it is occupied, with a blank record
    # (an AMS 1 does not hand its tag back unasked): the reel never left, so
    # the link that lands is kept.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    u.afc.spool = _FetchingSpool()
    _feed(u, _raw_frame(False))
    _feed(u, _raw_frame(True))
    assert u.afc.spool.fetching
    core.afcBambuAMS._on_bridge_reconnect(u)
    for _ in range(7):
        _feed(u, _booting_frame())
    if lands == "after-first-poll":
        for _ in range(3):
            _feed(u, _polled_frame(True), dt=1.0)
    u.afc.spool.land()
    for _ in range(5):
        _feed(u, _polled_frame(True), dt=1.0)
    assert lane.spool_id == 136 and lane.color == "#0086D6"
    assert not u.logger.having("landed after that spool left")
    assert not u.logger.having("nothing has read it")
    assert u._bridge.calls.count("request_info") == len(u._bridge.calls)


def test_a_lookup_answered_after_its_spool_left_binds_nothing(monkeypatch):
    # The relink after a flap is answered on the reactor after the worker's
    # lookup, and the reel was really pulled in between: the bay no longer
    # holds the tag, so the answer binds nothing.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    queued = []
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: queued.append(job))
    _feed(u, _raw_frame(False))
    _feed(u, _raw_frame(True))
    assert len(queued) == 1
    _feed(u, _gone_frame())
    queued.pop()()
    assert u.afc.spool.asked == [] and lane.spool_id in (None, "", 0)
    assert u._spool._bind_pending == set()
    assert not u._spool._spoolman_inflight


def test_a_lane_put_on_defaults_drops_a_link_that_landed_after_its_removal(
        monkeypatch):
    # An AFC whose set_spoolID has no on_done binds when its fetch lands and
    # says nothing, so a relink sent before a real removal can bind the lane
    # of the emptied bay unseen. A reel put in during the print goes on lane
    # defaults linked to nothing, not to the spool that left.
    asked, capscans, writes = _flap_spoolman(monkeypatch)
    u, lane = _flap_unit()
    u.afc.spool = _Spool()
    _feed(u, _raw_frame(False))
    _feed(u, _raw_frame(True))
    assert u.afc.spool.asked == [(lane, 136)]
    _feed(u, _gone_frame())
    u.afc.spool.land(670.8)
    assert lane.spool_id == 136
    for _ in range(8):
        _feed(u, _raw_frame(True, tagged=False), dt=1.0)
    assert lane.spool_id in (None, "", 0)
    assert _profile(lane) == _DEFAULTS
    assert len(u.logger.having("nothing has read it", "INFO")) == 1


class _SpoolSaying(_Spool):
    """_Spool with AFC_spool's on_done: fired once the fetch has landed and
    the lane carries the spool."""

    def __init__(self):
        super().__init__()
        self.done = []

    def set_spoolID(self, lane, sid, on_done=None):
        super().set_spoolID(lane, sid)
        self.done.append(on_done)

    def land(self, stored):
        done, self.done = self.done, []
        super().land(stored)
        for cb in done:
            if cb is not None:
                cb()


def test_a_measurement_taken_while_afc_fetches_the_spool_goes_to_it(
        monkeypatch, no_spoolman_http):
    # The restart lookup has matched 136 and AFC is fetching the spool to
    # bind it. A measurement adopted in that fetch is the bound spool's: held
    # for the bind, and not said as "no spool is linked" -- which was followed
    # by AFC loading Spoolman's 670.8 g over the measured 611 g.
    u, lane, asked = _lane15(monkeypatch)
    u.afc.spool = _SpoolSaying()
    _frames(u, 3)
    assert asked == [(UID15, TRAY15)]
    assert u.afc.spool.asked == [(lane, 136)]
    assert u._spool._bind_pending == {3}, "out until AFC has fetched it"
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u)
    _frames(u, 1)
    assert u._spool._bind_owed == {3: (74, 1000, None, True)}
    assert not u.logger.having("Measured about 74% left")
    u.afc.spool.land(670.8)
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "updated Spoolman spool 136" in said[0]
    _frames(u, 3)
    assert lane.spool_id == 136 and lane.weight == 611
    assert _writes(no_spoolman_http) == [("set", 136, 611.0)]
    assert u._spool._bind_pending == set()
    assert u._spool._bound_uid.get(3) == UID15
    _frames(u, 3)
    assert asked == [(UID15, TRAY15)]
    assert _writes(no_spoolman_http) == [("set", 136, 611.0)]


def test_a_scan_that_reads_no_new_measurement_still_asks_again(
        monkeypatch, no_spoolman_http):
    # AFC_BAMBU_SCAN after a missed restart lookup, where the unit re-reads
    # the tag and its cycle ends without a measurement: the scan forgot the
    # miss, so the tag is asked about again and linked.
    u, lane, _asked = _lane15(monkeypatch)
    asked, known = _spoolman136_later(monkeypatch)
    u._auto_scanned = [False] * 4
    u._scan_notag = [False] * 4
    u._scan_t0 = [None] * 4
    _frames(u, 3)
    assert asked == [UID15] and lane.spool_id is None
    known[0] = True
    u._bridge = types.SimpleNamespace(last_cap_measure=lambda addr: None)
    _scan_command(u, monkeypatch)
    _frames(u, 2)
    u._slots[3] = dict(u._slots[3], scan_seq=2, scan_res=1)
    u._cap_pending = {}
    u._cap_pending_slot = None
    _frames(u, 3)
    assert asked == [UID15, UID15]
    assert u.afc.spool.asked == [(lane, 136)]


def test_a_claimed_lane_waits_for_the_reread_its_record_is_owed(
        monkeypatch, no_spoolman_http):
    # The record names lane15's spool, but the firmware still owes it a
    # re-read: until then it can be the departed spool's, and it is not
    # looked up.
    u, lane, asked = _lane15(monkeypatch, reread_pending=True)
    _frames(u, 3)
    assert asked == [] and u._spoolman_latched == set()
    u._slots[3] = dict(u._slots[3], reread_pending=False)
    _frames(u, 2)
    assert asked == [(UID15, TRAY15)]
    assert u.afc.spool.asked == [(lane, 136)]


def test_a_retry_nothing_will_send_is_not_said_to_be_in_progress(
        monkeypatch):
    # Spoolman did not answer the restart lookup, and a retry was set. The
    # lane is then changed to PETG, so the record no longer describes it and
    # the retry will never go out: the summary claims no lookup in progress.
    u, lane, _asked = _lane15(monkeypatch, weight=611)
    u.afc.reactor = types.SimpleNamespace(monotonic=lambda: 100.0)
    asked = []
    monkeypatch.setattr(rfid, "_bambu_spoolman_client",
                        lambda afc: types.SimpleNamespace(
                            reachable=lambda: False,
                            set_remaining_weight=lambda sid, g: None))
    monkeypatch.setattr(rfid.BambuSpoolman, "_spoolman_bg",
                        lambda self, job: job())

    def _match(c, uid, tray_uid=""):
        asked.append(uid)
        return None, False, None
    monkeypatch.setattr(rfid, "match_spool_for_tag", _match)
    _frames(u, 3)
    assert asked == [UID15] and 3 in u._lookup_retry
    lane.material, lane.sub_type = "PETG", ""
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u)
    _frames(u, 3)
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "in progress" not in said[0]
    assert asked == [UID15]


def test_a_refusal_after_a_measurement_was_owed_is_said_as_the_refusal(
        monkeypatch, no_spoolman_http):
    # A capscan's measurement is owed to the lookup it waits for, and that
    # lookup matches spool 136 with no weight on record: nothing is bound,
    # and the summary names the spool to correct.
    u, lane, _asked = _lane15(monkeypatch)
    asked = _spoolman136_weighing(monkeypatch, 0.0)
    _capscan_74(u)
    _frames(u, 3)
    assert asked == []
    _measured_74(u)
    _frames(u, 3)
    assert asked == [UID15]
    assert u._spool._bind_owed == {}
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "in progress" not in said[0]
    assert ("95F2C30C matches Spoolman spool 136, which has no remaining "
            "weight on record") in said[0]


@pytest.mark.parametrize("owned", [True, False],
                         ids=["claimed-at-prep", "boot-hold"])
@pytest.mark.parametrize("record", ["read", "profile-blanked"])
def test_a_capscan_on_a_lane_on_defaults_says_no_lookup_was_made(
        record, owned, monkeypatch, no_spoolman_http):
    # lane15 restored on defaults (material, no colour, no variant): no
    # record can be tested against it, so nothing is looked up, and a
    # capscan's summary is said at once without claiming a lookup.
    u, lane, asked = _lane15(monkeypatch, owned=owned, colour="")
    lane.sub_type = ""
    _frames(u, 3)
    _capscan_74(u)
    _frames(u, 2)
    if record == "read":
        _measured_74(u)
    else:
        _measured_74(u, material=None, sku=None, color=None)
    _frames(u, 1)
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "in progress" not in said[0]
    if record == "read":
        assert ("no Spoolman lookup has been made for 95F2C30C, so the "
                "measurement stays on the lane only -- run AFC_BAMBU_SCAN "
                "LANE=lane15 to link it") in said[0]
    assert asked == []


def test_a_claimed_lane_says_its_summary_when_the_record_turns_out_stale(
        monkeypatch, no_spoolman_http):
    # A capscan blanks lane15's record and measures; the lookup waits for the
    # profile to test the lane against. It comes back naming another reel, so
    # no lookup follows and nothing will answer for the summary: the claimed
    # lane's own pass says it, on that frame.
    u, lane, asked = _lane15(monkeypatch)
    _capscan_74(u)
    _frames(u, 2)
    _measured_74(u, material=None, sku=None, color=None)
    _frames(u, 1)
    assert not u.logger.having("Measured about 74% left")
    u._slots[3] = dict(u._slots[3], material="PETG Basic", sku="GFG00",
                       color="FF0000")
    _frames(u, 1)
    said = u.logger.having("Measured about 74% left", "INFO")
    assert len(said) == 1, u.logger.lines
    assert "no Spoolman lookup has been made for 95F2C30C" in said[0]
    assert asked == []


def test_a_bind_that_lands_during_a_reread_still_gets_its_measurement(
        monkeypatch, no_spoolman_http):
    # The capscan's measurement is owed to the lookup, the lookup has gone
    # out, and the firmware flags the record for a re-read (a second capacity
    # window) as the bind lands. The surface path waits for the re-read, but
    # the spool the bind attached is handed its measurement now.
    u, lane, asked = _lane15(monkeypatch, owned=False)
    _capscan_74(u)
    _frames(u, 3)
    _measured_74(u)
    _frames(u, 1)
    assert asked == [(UID15, TRAY15)]
    assert u._spool._bind_owed == {3: (74, 1000, None, True)}
    u._slots[3] = dict(u._slots[3], reread_pending=True)
    u.afc.spool.land(670.8)
    _frames(u, 2)
    assert lane.spool_id == 136 and lane.weight == 611
    assert _writes(no_spoolman_http) == [("set", 136, 611.0)]


def test_spoolman_jobs_run_off_the_reactor_for_a_real_delegate():
    """A real BambuSpoolman queues HTTP work on the worker thread.

    The flag lives on the delegate class, so it must be read from the
    delegate: the unit it wraps does not carry it.
    """
    import threading
    delegate = BambuSpoolman.__new__(BambuSpoolman)
    delegate._u = types.SimpleNamespace()        # a unit without the flag
    ran = threading.Event()
    seen = {}

    def job():
        seen["thread"] = threading.current_thread().name
        ran.set()

    delegate._spoolman_bg(job)
    assert ran.wait(5.0)
    assert seen["thread"] == "afc_bambu_spool"


def test_a_stand_in_without_the_flag_runs_the_job_inline():
    stand_in = types.SimpleNamespace(_u=types.SimpleNamespace())
    calls = []
    BambuSpoolman._spoolman_bg(stand_in, lambda: calls.append(1))
    assert calls == [1]
