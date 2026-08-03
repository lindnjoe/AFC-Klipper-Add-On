"""
Unit tests for the pure helper functions in extras/AFC_RFID.py

These feed both the ACE RFID inventory and the U1 RFID scanner paths:
material density lookup, RGB/hex conversion, UID normalization, and
bed-temp defaults.
"""

from __future__ import annotations

import extras.AFC_RFID as rfid
from extras.AFC_RFID import (
    density_for_material,
    build_filament_name,
    format_tag_summary,
    enrich_from_spool,
    prompt_hold_spool,
    dismiss_prompt,
    rgb_array_to_hex,
    default_bed_temp_for_material,
    find_spool_by_uid,
    _norm_uid,
)


# ── density_for_material ──────────────────────────────────────────────────────

def test_density_known_materials():
    assert density_for_material("PLA") == 1.24
    assert density_for_material("pla") == 1.24


def test_density_separator_and_case_insensitive():
    """'PLA-CF', 'pla cf', 'pla_cf' all normalize to the same key."""
    d = density_for_material("PLA-CF")
    assert d == density_for_material("pla cf") == density_for_material("pla_cf")


def test_density_prefix_fallback():
    """Unknown variants fall back to the longest matching base material."""
    assert density_for_material("PLA Silk Rainbow") == density_for_material("PLA Silk Rainbow".replace(" ", ""))


def test_density_unknown_defaults_to_pla():
    assert density_for_material("unobtainium") == 1.24
    assert density_for_material("") == 1.24
    assert density_for_material(None) == 1.24


# ── build_filament_name ───────────────────────────────────────────────────────

def test_build_filament_name_full():
    assert build_filament_name("Bambu", "PLA", "Basic") == "Bambu PLA Basic"
    assert build_filament_name("Bambu", "PLA", "Matte") == "Bambu PLA Matte"


def test_build_filament_name_drops_duplicate_material():
    # sub_type already spells out the material -> don't repeat it
    assert build_filament_name("Bambu", "PLA", "PLA Basic") == "Bambu PLA Basic"


def test_build_filament_name_skips_empty_parts():
    assert build_filament_name("", "PLA", "") == "PLA"
    assert build_filament_name("Bambu", "", "") == "Bambu"
    assert build_filament_name("", "", "") == ""


# ── format_tag_summary ────────────────────────────────────────────────────────

def test_format_tag_summary_full():
    s = format_tag_summary({
        "brand": "Bambu", "material": "PLA", "sub_type": "Basic",
        "color_hex": "00ff00", "extruder_temp": 220, "bed_temp": 60,
    }, "ACE2 RFID: read lane1")
    assert s.splitlines() == [
        "ACE2 RFID: read lane1",
        "  Name: Bambu PLA Basic",
        "  Brand: Bambu",
        "  Material: PLA",
        "  Color: #00ff00",
        "  Nozzle temp: 220°C",
        "  Bed temp: 60°C",
    ]


def test_format_tag_summary_dual_color_joins_hex():
    s = format_tag_summary(
        {"brand": "Bambu", "material": "PLA",
         "multi_color": ["e94b3c", "#ffffff"]}, "hdr")
    assert "  Color: #e94b3c + #ffffff" in s


def test_format_tag_summary_bare_uid_is_header_only():
    # a UID-only decode (no fields) -> just the header, so callers can skip it
    s = format_tag_summary({"uid": "AABBCCDD"}, "hdr")
    assert s == "hdr"
    assert "\n" not in s


def test_format_tag_summary_enriched_fields():
    # display_name (matched Spoolman name) wins; extras render when present
    s = format_tag_summary({
        "brand": "Bambu", "material": "PLA", "sub_type": "Basic",
        "display_name": "My Custom Name", "color_hex": "00ff00",
        "diameter": 1.75, "remaining_weight": 812.4, "spool_id": 42,
    }, "hdr")
    lines = s.splitlines()
    assert "  Name: My Custom Name" in lines
    assert "  Diameter: 1.75mm" in lines
    assert "  Remaining: 812g" in lines           # rounded
    assert "  Spoolman ID: 42" in lines


# ── enrich_from_spool ─────────────────────────────────────────────────────────

class _SpoolClient:
    def __init__(self, spool):
        self._spool = spool
    def get_spool(self, spool_id):
        return self._spool


def test_enrich_from_spool_overlays_record():
    slot = {"brand": "Bambu", "material": "PLA", "color_hex": "00ff00"}
    client = _SpoolClient({
        "remaining_weight": 640.0,
        "filament": {
            "name": "Bambu PLA Basic", "material": "PLA",
            "settings_extruder_temp": 220, "settings_bed_temp": 60,
            "diameter": 1.75, "vendor": {"name": "Bambu Lab"},
        },
    })
    d = enrich_from_spool(client, 7, slot)
    assert d["display_name"] == "Bambu PLA Basic"
    assert d["brand"] == "Bambu Lab"
    assert d["extruder_temp"] == 220
    assert d["remaining_weight"] == 640.0
    assert d["spool_id"] == 7
    assert slot.get("display_name") is None       # input not mutated


def test_enrich_from_spool_no_id_returns_copy():
    slot = {"brand": "Bambu", "material": "PLA"}
    d = enrich_from_spool(_SpoolClient({}), None, slot)
    assert d == slot and d is not slot


# ── prompt_hold_spool / dismiss_prompt ────────────────────────────────────────

def test_prompt_hold_spool_emits_action_prompt():
    out = []
    prompt_hold_spool(out.append, "lane1")
    assert out[0] == "// action:prompt_begin RFID Scan"
    assert any("Tag detected on lane1" in m and "hold the spool" in m
               for m in out)
    assert out[-1] == "// action:prompt_show"


def test_dismiss_prompt_emits_prompt_end():
    out = []
    dismiss_prompt(out.append)
    assert out == ["// action:prompt_end"]


# ── rgb_array_to_hex ──────────────────────────────────────────────────────────

def test_rgb_array_to_hex():
    assert rgb_array_to_hex([255, 0, 0]) == "#ff0000"
    assert rgb_array_to_hex((0, 128, 255)) == "#0080ff"


def test_rgb_array_to_hex_invalid_input():
    assert rgb_array_to_hex(None) == "#000000"
    assert rgb_array_to_hex([255]) == "#000000"
    assert rgb_array_to_hex("FF0000") == "#000000"


# ── _norm_uid ─────────────────────────────────────────────────────────────────

def test_norm_uid_separator_and_case_insensitive():
    """'E5:CA:F0:A1', 'e5-ca-f0-a1' and 'E5CAF0A1' all compare equal."""
    assert _norm_uid("E5:CA:F0:A1") == "E5CAF0A1"
    assert _norm_uid("e5-ca-f0-a1") == "E5CAF0A1"
    assert _norm_uid("e5 ca f0 a1") == "E5CAF0A1"
    assert _norm_uid("") == ""
    assert _norm_uid(None) == ""


def test_norm_uid_distinct_uids_differ():
    assert _norm_uid("56A36AEA") != _norm_uid("26A36AEA")


# ── default_bed_temp_for_material ─────────────────────────────────────────────

def test_bed_temp_defaults():
    pla = default_bed_temp_for_material("PLA")
    abs_temp = default_bed_temp_for_material("ABS")
    assert pla and abs_temp
    assert abs_temp > pla  # ABS beds run hotter than PLA


# ── find_spool_by_uid: one tag == one spool (case/separator-insensitive) ──────

class _FakeSpoolClient:
    def __init__(self, spools, raise_on_search=False):
        self._spools = spools
        self._raise = raise_on_search

    def search_spools(self, filament_id=None):
        if self._raise:
            raise RuntimeError("spoolman unreachable")
        return self._spools


def _spool_with_uids(sid, uids):
    # card_uids is a comma-separated list in the spool 'extra' (Snapmaker conv).
    return {"id": sid, "extra": {"card_uids": ",".join(uids)}}


def test_find_spool_by_uid_matches_regardless_of_case_or_separators():
    client = _FakeSpoolClient([
        _spool_with_uids(1, ["AAAA1111"]),
        _spool_with_uids(2, ["10C7E32F", "7BF0AFFF"]),
    ])
    # the same physical tag always resolves to its one spool, any format
    assert find_spool_by_uid(client, "7bf0afff")["id"] == 2
    assert find_spool_by_uid(client, "10:C7:E3:2F")["id"] == 2
    assert find_spool_by_uid(client, "AAAA1111")["id"] == 1


def test_find_spool_by_uid_unknown_uid_returns_none():
    client = _FakeSpoolClient([_spool_with_uids(1, ["AAAA1111"])])
    assert find_spool_by_uid(client, "DEADBEEF") is None


def test_find_spool_by_uid_search_failure_returns_none():
    # A transient list failure must NOT masquerade as "no match" downstream —
    # returning None makes the sync layer skip (leave the tag's values in place)
    # rather than create a duplicate.
    client = _FakeSpoolClient([], raise_on_search=True)
    assert find_spool_by_uid(client, "AAAA1111") is None


# ── resolve_rfid_keys: shared [AFC_rfid_keys] fallback ───────────────────────

class _SharedKeys:
    def __init__(self, bambu=None, creality=None, creality_enc=None):
        self.bambu_master_key = bambu
        self.creality_key = creality
        self.creality_encryption_key = creality_enc


class _PrinterWithShared:
    def __init__(self, shared):
        self._shared = shared

    def lookup_object(self, name, default=None):
        return self._shared if name == "AFC_rfid_keys" else default


def test_resolve_rfid_keys_no_shared_section_is_passthrough():
    from extras.AFC_RFID import resolve_rfid_keys
    printer = _PrinterWithShared(None)
    assert resolve_rfid_keys(printer, b"\x01", None, None) == (b"\x01", None, None)


def test_resolve_rfid_keys_fills_unset_from_shared():
    from extras.AFC_RFID import resolve_rfid_keys
    shared = _SharedKeys(bambu=b"\xaa", creality=b"\xbb", creality_enc=b"\xcc")
    printer = _PrinterWithShared(shared)
    # Nothing set locally -> all come from the shared section.
    assert resolve_rfid_keys(printer, None, None, None) == (b"\xaa", b"\xbb", b"\xcc")


def test_resolve_rfid_keys_own_key_wins():
    from extras.AFC_RFID import resolve_rfid_keys
    shared = _SharedKeys(bambu=b"\xaa", creality=b"\xbb", creality_enc=b"\xcc")
    printer = _PrinterWithShared(shared)
    # A locally-set key overrides the shared one; only unset keys fall back.
    out = resolve_rfid_keys(printer, b"\x11", None, b"\x33")
    assert out == (b"\x11", b"\xbb", b"\x33")


# ── AFCUnitRFID mixin (shared per-unit apply path) ────────────────────────────

import extras.AFC_RFID as _rfidmod
from extras.AFC_RFID import AFCUnitRFID


class _Unit(AFCUnitRFID):
    """Minimal adapter satisfying the AFCUnitRFID contract."""
    def __init__(self, afc, auto_create=False):
        self.afc = afc
        self.auto_create = auto_create
        self.log_prefix = "TEST RFID"
        import logging
        self.logger = logging.getLogger("test_unit_rfid")

    def _map(self, tag):
        # echo the tag as slot_info, adding a weight the base forwards
        return {"uid": tag.get("uid"), "material": tag.get("material"),
                "weight_g": 250}


def test_mixin_apply_to_lane_maps_applies_and_syncs(monkeypatch):
    applied, synced = [], []
    monkeypatch.setattr(_rfidmod, "apply_filament_defaults",
                        lambda lane, si: applied.append((lane, si)))
    monkeypatch.setattr(_rfidmod, "sync_rfid_to_spoolman",
                        lambda afc, lane, si, logger, prefix, **kw:
                        synced.append((prefix, kw)))
    lane = object()
    unit = _Unit(afc=types_ns(spoolman=object()))
    out = unit.apply_to_lane(lane, {"uid": "aa", "material": "PLA"})
    assert out["uid"] == "aa" and out["material"] == "PLA"
    assert applied and applied[0][0] is lane
    assert len(synced) == 1
    prefix, kw = synced[0]
    assert prefix == "TEST RFID"                 # log_prefix threaded through
    # weight now travels inside slot_info (weight_g), not as a kwarg — the sync
    # uses it only when creating the spool (initial/remaining, never tare).
    assert "spool_weight" not in kw
    assert out["weight_g"] == 250


def test_mixin_apply_to_lane_skips_sync_without_spoolman(monkeypatch):
    synced = []
    monkeypatch.setattr(_rfidmod, "apply_filament_defaults", lambda *a, **k: None)
    monkeypatch.setattr(_rfidmod, "sync_rfid_to_spoolman",
                        lambda *a, **k: synced.append(1))
    # afc present but no spoolman -> apply defaults, but no Spoolman sync
    unit = _Unit(afc=types_ns(spoolman=None))
    unit.apply_to_lane(object(), {"uid": "aa"})
    assert synced == []
    # afc None -> also no sync, no crash
    unit2 = _Unit(afc=None)
    unit2.apply_to_lane(object(), {"uid": "aa"})
    assert synced == []


def test_mixin_resolve_auto_create_prefers_lane(monkeypatch):
    monkeypatch.setattr(_rfidmod, "get_auto_spoolman_create",
                        lambda lane, default: True)
    unit = _Unit(afc=None, auto_create=False)
    assert unit._resolve_auto_create(object()) is True   # lane setting wins
    # if the helper raises, fall back to the unit default
    monkeypatch.setattr(_rfidmod, "get_auto_spoolman_create",
                        lambda lane, default: (_ for _ in ()).throw(RuntimeError()))
    unit2 = _Unit(afc=None, auto_create=True)
    assert unit2._resolve_auto_create(object()) is True


def types_ns(**kw):
    import types
    return types.SimpleNamespace(**kw)


# ── sync_rfid_to_spoolman: incomplete-decode guard, UID-only create ───────────

from extras.AFC_RFID import sync_rfid_to_spoolman


class _SyncClient:
    """Stub SpoolmanClient recording create calls; reachable, no UID match."""
    def __init__(self, existing_filaments=None):
        self.created_filaments = []
        self.created_spools = []
    def reachable(self): return True
    def search_spools(self, filament_id=None): return []
    def get_or_create_vendor(self, name): return {"id": 7}
    def create_filament(self, **kw):
        self.created_filaments.append(kw)
        return {"id": 99, "name": kw.get("name"), "color_hex": kw.get("color_hex")}
    def create_spool(self, **kw):
        self.created_spools.append(kw); return {"id": 500, "remaining_weight": 1000}
    def update_filament(self, *a, **k): return None
    def write_filament_variant(self, *a, **k): return None
    def write_spool_metadata(self, *a, **k): return None


def _afc_ns(client):
    import types
    spool = types.SimpleNamespace(next_spool_info=None, next_spool_id=None,
                                  set_spoolID=lambda lane, sid: setattr(lane, "spool_id", sid))
    return types.SimpleNamespace(spoolman=object(), moonraker=object(), spool=spool), client


def _run_sync(monkeypatch, slot_info, existing=None):
    client = _SyncClient(existing)
    monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: client)
    monkeypatch.setattr(_rfidmod, "find_spool_by_uid", lambda c, u: None)
    import types, logging
    afc, _ = _afc_ns(client)
    lane = types.SimpleNamespace(name="lane1", spool_id=None,
                                 send_lane_data=lambda: None)
    sync_rfid_to_spoolman(afc, lane, slot_info, logging.getLogger("t"),
                          "TEST", allow_create=True)
    return client, lane


def test_sync_refuses_incomplete_decode(monkeypatch):
    # material present but NO colour -> incomplete decode -> no create at all
    client, lane = _run_sync(monkeypatch, {
        "uid": "AABBCCDD", "brand": "Bambu", "material": "PLA",
        "color_hex": "", "sub_type": "Basic"})
    assert client.created_filaments == []
    assert client.created_spools == []


def test_sync_creates_new_filament_for_new_uid(monkeypatch):
    # UID is the only match key; an unseen UID always creates a new filament +
    # spool (no colour/identity reuse).
    client, lane = _run_sync(monkeypatch, {
        "uid": "AABBCCDD", "brand": "Bambu", "material": "PLA",
        "color_hex": "ffffff", "sub_type": "Basic"})
    assert len(client.created_filaments) == 1
    assert len(client.created_spools) == 1
    assert lane.spool_id == 500


def test_sync_no_create_without_uid(monkeypatch):
    # no tag UID -> nothing to re-match on -> never create (SKU path is gone)
    client, lane = _run_sync(monkeypatch, {
        "uid": "", "brand": "Bambu", "material": "PLA",
        "color_hex": "ffffff", "sub_type": "Basic"})
    assert client.created_filaments == [] and client.created_spools == []
