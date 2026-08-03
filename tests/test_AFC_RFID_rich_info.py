"""
Unit tests for the rich tag-info surfacing added to the AFC RFID stack:

  - map_tag_to_slot_info: the single shared read_tag() -> slot_info mapping
  - make_tag_record: the uniform "last read" get_status record
  - AFCUnitRFID.record_tag_read / last_reads_status / undecoded_hint
  - SpoolmanClient.write_filament_drying (+ lazy field creation)
  - apply_filament_defaults: vendor / multi-colour / density lane fields
  - sync_rfid_to_spoolman: net-weight-at-creation routing, tag density, SKU
  - format_tag_summary: the extended field lines
  - Bambu decoder: nozzle diameter / spool width / filament length
  - Anycubic + Creality decoders: raw length_m
  - AFC_U1_rfid._map_to_slot_info: weight / diameter / rich optional keys
"""

from __future__ import annotations

import logging
import struct
import types

import extras.AFC_RFID as _rfidmod
from extras.AFC_RFID import (
    AFCUnitRFID,
    SpoolmanClient,
    apply_filament_defaults,
    format_tag_summary,
    make_tag_record,
    map_tag_to_slot_info,
    sync_rfid_to_spoolman,
)
import extras.AFC_ACE2_rfid as ace2


def _ns(**kw):
    return types.SimpleNamespace(**kw)


# ── map_tag_to_slot_info ──────────────────────────────────────────────────────

def test_map_empty_tag_gives_defaults():
    si = map_tag_to_slot_info({})
    assert si["material"] == ""
    assert si["color_hex"] == ""
    assert si["multi_color"] == []
    assert si["is_dual_color"] is False
    assert si["diameter"] == 1.75
    assert si["extruder_temp"] is None
    assert si["uid"] == ""
    # no optional rich keys on an empty tag
    for key in ("extruder_temp_min", "extruder_temp_max", "color_alpha",
                "serial", "tray_uid", "density", "drying_temp",
                "drying_time_h", "color_count", "nozzle_diameter",
                "spool_width_mm", "length_m", "tag_type"):
        assert key not in si


def test_map_none_tag_gives_defaults():
    si = map_tag_to_slot_info(None)
    assert si["material"] == "" and si["uid"] == ""


def test_map_primary_color_and_opaque_alpha():
    tag = {"filament": {"color_argb": 0xFF12CD56}}
    si = map_tag_to_slot_info(tag)
    assert si["color_hex"] == "12cd56"
    assert si["multi_color"] == ["12cd56"]
    assert "color_alpha" not in si                    # 0xFF alpha not surfaced


def test_map_translucent_alpha_surfaced():
    tag = {"filament": {"color_argb": 0x8012CD56}}
    si = map_tag_to_slot_info(tag)
    assert si["color_alpha"] == 0x80


def test_map_colors_argb_dedup_and_none_skip():
    tag = {"filament": {"color_argb": 0xFFAA0000,
                        "colors_argb": [0xFFAA0000, None, 0xFF00BB00,
                                        0xFFAA0000]}}
    si = map_tag_to_slot_info(tag)
    assert si["multi_color"] == ["aa0000", "00bb00"]
    assert si["is_dual_color"] is True


def test_map_temp_midpoint_and_range():
    tag = {"filament": {"hotend_min_c": 210, "hotend_max_c": 231}}
    si = map_tag_to_slot_info(tag)
    assert si["extruder_temp"] == (210 + 231) // 2
    assert si["extruder_temp_min"] == 210
    assert si["extruder_temp_max"] == 231


def test_map_temp_max_only():
    si = map_tag_to_slot_info({"filament": {"hotend_max_c": 240}})
    assert si["extruder_temp"] == 240
    assert "extruder_temp_min" not in si
    assert si["extruder_temp_max"] == 240


def test_map_temp_min_only_gives_none_midpoint():
    # min alone can't produce a midpoint (mirrors the pre-existing behaviour)
    si = map_tag_to_slot_info({"filament": {"hotend_min_c": 200}})
    assert si["extruder_temp"] is None
    assert si["extruder_temp_min"] == 200
    assert "extruder_temp_max" not in si


def test_map_optional_rich_fields_copied():
    fil = {"serial": "S123", "tray_uid": "aa" * 16, "density": 1.24,
           "drying_temp_c": 70, "drying_time_h": 8, "color_count": 2,
           "nozzle_diameter": 0.4, "spool_width_mm": 66.2, "length_m": 330}
    si = map_tag_to_slot_info({"filament": fil})
    assert si["serial"] == "S123"
    assert si["tray_uid"] == "aa" * 16
    assert si["density"] == 1.24
    assert si["drying_temp"] == 70
    assert si["drying_time_h"] == 8
    assert si["color_count"] == 2
    assert si["nozzle_diameter"] == 0.4
    assert si["spool_width_mm"] == 66.2
    assert si["length_m"] == 330


def test_map_optional_rich_fields_skip_unset_values():
    fil = {"serial": "", "density": None, "drying_temp_c": 0,
           "length_m": 0}
    si = map_tag_to_slot_info({"filament": fil})
    for key in ("serial", "density", "drying_temp", "length_m"):
        assert key not in si


def test_map_tag_type_copied_only_when_set():
    si = map_tag_to_slot_info({"uid": "aabb", "tag_type": "MifareClassic1k"})
    assert si["tag_type"] == "MifareClassic1k"
    si2 = map_tag_to_slot_info({"uid": "aabb"})
    assert "tag_type" not in si2


def test_map_brand_from_decode_only():
    si = map_tag_to_slot_info({"filament": {"manufacturer": "Elegoo"}})
    assert si["brand"] == "Elegoo"
    si2 = map_tag_to_slot_info({"uid": "aa", "tag_type": "MifareClassic1k"})
    assert si2["brand"] == ""                         # never guessed from type


# ── make_tag_record ───────────────────────────────────────────────────────────

def test_record_drops_empty_values():
    rec = make_tag_record({"material": "PLA", "sku": "", "bed_temp": None,
                           "multi_color": []}, 100.0)
    assert rec["material"] == "PLA"
    for key in ("sku", "bed_temp", "multi_color"):
        assert key not in rec


def test_record_decoded_flag_and_time():
    rec = make_tag_record({"material": "PLA"}, 1234.56789)
    assert rec["decoded"] is True
    assert rec["scan_time"] == round(1234.56789, 3)


def test_record_failed_decode_uses_uid_and_type_args():
    rec = make_tag_record(None, 5.0, decoded=False, uid="AABB",
                          tag_type="MifareUltralight")
    assert rec["decoded"] is False
    assert rec["uid"] == "AABB"
    assert rec["tag_type"] == "MifareUltralight"


def test_record_slot_info_uid_wins_over_arg():
    rec = make_tag_record({"uid": "CCDD"}, 5.0, uid="AABB")
    assert rec["uid"] == "CCDD"


# ── AFCUnitRFID.record_tag_read / last_reads_status / undecoded_hint ──────────

class _Unit(AFCUnitRFID):
    """Minimal adapter satisfying the AFCUnitRFID contract."""
    def __init__(self):
        self.afc = None
        self.auto_create = False
        self.log_prefix = "TEST RFID"
        self.logger = logging.getLogger("test_rich_unit")

    def _map(self, tag):
        return map_tag_to_slot_info(tag)


class TestRecordTagRead:
    def test_stores_under_string_key_and_returns_record(self):
        unit = _Unit()
        rec = unit.record_tag_read("lane1", {"material": "PLA"})
        assert unit.last_reads_status()["lane1"] == rec
        assert rec["material"] == "PLA" and rec["decoded"] is True

    def test_failed_decode_records_uid(self):
        unit = _Unit()
        rec = unit.record_tag_read(3, None, decoded=False, uid="AABB",
                                   tag_type="MifareClassic1k")
        assert unit.last_reads_status()["3"] == rec
        assert rec["decoded"] is False and rec["uid"] == "AABB"

    def test_new_read_overwrites_old(self):
        unit = _Unit()
        unit.record_tag_read("lane1", {"material": "PLA"})
        unit.record_tag_read("lane1", {"material": "PETG"})
        assert unit.last_reads_status()["lane1"]["material"] == "PETG"


class TestLastReadsStatus:
    def test_empty_before_first_read(self):
        assert _Unit().last_reads_status() == {}

    def test_returns_copy(self):
        unit = _Unit()
        unit.record_tag_read("lane1", {"material": "PLA"})
        status = unit.last_reads_status()
        status.clear()
        assert unit.last_reads_status() != {}


class TestUndecodedHint:
    def test_hint_for_fresh_undecoded_read(self, monkeypatch):
        unit = _Unit()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False, uid="AABB",
                             tag_type="MifareClassic1k")
        hint = unit.undecoded_hint("lane1")
        assert hint == (" (saw tag UID AABB, MifareClassic1k"
                        " — no decoder/key matched)")

    def test_hint_without_tag_type(self, monkeypatch):
        unit = _Unit()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False, uid="AABB")
        assert unit.undecoded_hint("lane1") == \
            " (saw tag UID AABB — no decoder/key matched)"

    def test_no_hint_for_decoded_read(self, monkeypatch):
        unit = _Unit()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", {"material": "PLA", "uid": "AABB"})
        assert unit.undecoded_hint("lane1") == ""

    def test_no_hint_without_uid(self, monkeypatch):
        unit = _Unit()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False)
        assert unit.undecoded_hint("lane1") == ""

    def test_no_hint_when_stale(self, monkeypatch):
        unit = _Unit()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False, uid="AABB")
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1011.0)
        assert unit.undecoded_hint("lane1") == ""

    def test_no_hint_for_unknown_key(self):
        assert _Unit().undecoded_hint("nope") == ""


# ── SpoolmanClient.write_filament_drying / _ensure_drying_fields ──────────────

def _client():
    mr = _ns(host="http://mr", logger=logging.getLogger("t"))
    client = SpoolmanClient(mr)
    calls = []

    def proxy(method, path, body=None, print_error=True):
        calls.append((method, path, body))
        if method == "GET" and path == "/v1/field/filament":
            return []                                 # no fields exist yet
        return {"ok": True}

    client._spoolman_proxy = proxy
    return client, calls


class TestWriteFilamentDrying:
    def test_noop_when_both_unset(self):
        client, calls = _client()
        assert client.write_filament_drying(1, None, None) is None
        assert calls == []                            # not even the ensure GET

    def test_writes_both_fields(self):
        client, calls = _client()
        result = client.write_filament_drying(7, 70, 8)
        assert result == {"ok": True}
        patches = [c for c in calls if c[0] == "PATCH"]
        assert len(patches) == 1
        method, path, body = patches[0]
        assert path == "/v1/filament/7"
        assert body["extra"]["drying_temp_c"] == "70"
        assert body["extra"]["drying_time_h"] == "8"

    def test_temp_only(self):
        client, calls = _client()
        client.write_filament_drying(7, 65, None)
        body = [c for c in calls if c[0] == "PATCH"][0][2]
        assert body["extra"]["drying_temp_c"] == "65"
        assert "drying_time_h" not in body["extra"]

    def test_noop_when_values_current(self):
        client, calls = _client()
        current = {"drying_temp_c": "70", "drying_time_h": "8"}
        assert client.write_filament_drying(7, 70, 8,
                                            current_extra=current) is None
        assert [c for c in calls if c[0] == "PATCH"] == []

    def test_ensure_creates_fields_once(self):
        client, calls = _client()
        client.write_filament_drying(1, 70, 8)
        client.write_filament_drying(2, 60, 6)
        gets = [c for c in calls if c[0] == "GET"]
        posts = [c for c in calls if c[0] == "POST"]
        assert len(gets) == 1                         # ensure is cached
        assert {p[1] for p in posts} == {
            "/v1/field/filament/drying_temp_c",
            "/v1/field/filament/drying_time_h",
        }

    def test_ensure_skips_existing_fields(self):
        client, calls = _client()

        def proxy(method, path, body=None, print_error=True):
            calls.append((method, path, body))
            if method == "GET":
                return [{"key": "drying_temp_c"}, {"key": "drying_time_h"}]
            return {"ok": True}

        client._spoolman_proxy = proxy
        client.write_filament_drying(1, 70, 8)
        assert [c for c in calls if c[0] == "POST"] == []


# ── apply_filament_defaults: rich lane fields ─────────────────────────────────

def _lane(**overrides):
    lane = _ns(material=None, color=None, extruder_temp=None, bed_temp=None,
               weight=0, spool_vendor="", multi_color=[],
               filament_density=1.24, sub_type="")
    for key, val in overrides.items():
        setattr(lane, key, val)
    return lane


class TestApplyFilamentDefaultsRich:
    def test_vendor_applied_when_unset(self):
        lane = _lane()
        apply_filament_defaults(lane, {"brand": "Elegoo"})
        assert lane.spool_vendor == "Elegoo"

    def test_vendor_kept_when_already_set(self):
        lane = _lane(spool_vendor="Existing")
        apply_filament_defaults(lane, {"brand": "Elegoo"})
        assert lane.spool_vendor == "Existing"

    def test_vendor_not_applied_when_tag_has_none(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.spool_vendor == ""

    def test_multi_color_applied_for_dual(self):
        lane = _lane()
        apply_filament_defaults(
            lane, {"multi_color": ["#aa0000", "00bb00"]})
        assert lane.multi_color == ["aa0000", "00bb00"]

    def test_multi_color_not_applied_for_single(self):
        lane = _lane()
        apply_filament_defaults(lane, {"multi_color": ["aa0000"]})
        assert lane.multi_color == []

    def test_multi_color_kept_when_already_set(self):
        lane = _lane(multi_color=["111111", "222222"])
        apply_filament_defaults(
            lane, {"multi_color": ["aa0000", "00bb00"]})
        assert lane.multi_color == ["111111", "222222"]

    def test_density_applied_when_tag_carries_one(self):
        lane = _lane()
        apply_filament_defaults(lane, {"density": 1.31})
        assert lane.filament_density == 1.31

    def test_density_untouched_without_tag_value(self):
        lane = _lane(filament_density=1.04)
        apply_filament_defaults(lane, {"material": "ABS"})
        assert lane.filament_density == 1.04

    def test_density_invalid_value_ignored(self):
        lane = _lane(filament_density=1.04)
        apply_filament_defaults(lane, {"density": "junk"})
        assert lane.filament_density == 1.04


# ── sync_rfid_to_spoolman: weight / density / sku routing ─────────────────────

class _SyncClient:
    """Stub SpoolmanClient recording create calls; reachable, no UID match."""
    def __init__(self):
        self.created_filaments = []
        self.created_spools = []
        self.drying_writes = []

    def reachable(self):
        return True

    def get_or_create_vendor(self, name):
        return {"id": 7}

    def create_filament(self, **kw):
        self.created_filaments.append(kw)
        return {"id": 99, "name": kw.get("name"),
                "color_hex": kw.get("color_hex")}

    def create_spool(self, **kw):
        self.created_spools.append(kw)
        return {"id": 500, "remaining_weight": kw.get("remaining_weight")}

    def update_filament(self, *a, **k):
        return None

    def write_filament_variant(self, *a, **k):
        return None

    def write_filament_drying(self, filament_id, temp, hours, current_extra=None):
        self.drying_writes.append((filament_id, temp, hours))
        return None

    def write_spool_metadata(self, *a, **k):
        return None


def _run_sync(monkeypatch, slot_info):
    client = _SyncClient()
    monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: client)
    monkeypatch.setattr(_rfidmod, "find_spool_by_uid", lambda c, u: None)
    spool = _ns(next_spool_info=None, next_spool_id=None,
                set_spoolID=lambda lane, sid: setattr(lane, "spool_id", sid))
    afc = _ns(spoolman=object(), moonraker=object(), spool=spool)
    lane = _ns(name="lane1", spool_id=None, send_lane_data=lambda: None)
    sync_rfid_to_spoolman(afc, lane, slot_info, logging.getLogger("t"),
                          "TEST", allow_create=True)
    return client


_BASE_TAG = {"uid": "AABBCCDD", "brand": "BQ Tech", "material": "PLA",
             "color_hex": "aa0000", "diameter": 1.75}


class TestSyncWeightRouting:
    def test_tag_weight_seeds_creation_only(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG, weight_g=750))
        fil = client.created_filaments[0]
        spool = client.created_spools[0]
        assert fil["weight"] == 750
        assert spool["initial_weight"] == 750
        assert spool["remaining_weight"] == 750
        # NEVER tare: the tag's weight is net filament, not the empty spool
        assert "spool_weight" not in fil
        assert "spool_weight" not in spool

    def test_no_tag_weight_falls_back_to_1000(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG))
        assert client.created_filaments[0]["weight"] == 1000
        assert client.created_spools[0]["initial_weight"] == 1000
        assert client.created_spools[0]["remaining_weight"] == 1000

    def test_invalid_tag_weight_falls_back_to_1000(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG, weight_g="junk"))
        assert client.created_filaments[0]["weight"] == 1000


class TestSyncDensity:
    def test_tag_density_wins(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG, density=1.31))
        assert client.created_filaments[0]["density"] == 1.31

    def test_material_table_density_without_tag(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG))
        # independently derived: PLA -> 1.24 per the density table
        assert client.created_filaments[0]["density"] == 1.24


class TestSyncSkuAtCreate:
    def test_sku_passed_as_article_number(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG, sku="AC123"))
        assert client.created_filaments[0]["article_number"] == "AC123"

    def test_no_sku_passes_none(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG))
        assert client.created_filaments[0]["article_number"] is None


class TestSyncDrying:
    def test_drying_written_on_create(self, monkeypatch):
        client = _run_sync(monkeypatch,
                           dict(_BASE_TAG, drying_temp=70, drying_time_h=8))
        assert client.drying_writes == [(99, 70, 8)]

    def test_no_drying_write_without_values(self, monkeypatch):
        client = _run_sync(monkeypatch, dict(_BASE_TAG))
        assert client.drying_writes == []


# ── format_tag_summary: extended lines ────────────────────────────────────────

class TestFormatTagSummaryRich:
    def test_all_new_lines(self):
        s = format_tag_summary({
            "brand": "BQ Tech", "material": "PET", "color_hex": "12cd56",
            "diameter": 1.75, "density": 1.24,
            "extruder_temp": 220, "extruder_temp_min": 210,
            "extruder_temp_max": 230, "bed_temp": 60,
            "drying_temp": 70, "drying_time_h": 8,
            "weight_g": 750, "length_m": 330,
            "sku": "AC123", "serial": "S99", "mfg_date": "2024-08-12",
            "uid": "AABBCCDD",
        }, "hdr")
        lines = s.splitlines()
        assert "  Density: 1.24g/cm³" in lines
        assert "  Nozzle temp: 220°C (210–230)" in lines
        assert "  Drying: 70°C for 8h" in lines
        assert "  Tag weight: 750g" in lines
        assert "  Length: 330m" in lines
        assert "  SKU: AC123" in lines
        assert "  Serial: S99" in lines
        assert "  Mfg date: 2024-08-12" in lines
        assert "  Tag UID: AABBCCDD" in lines

    def test_temp_line_without_range(self):
        s = format_tag_summary({"material": "PLA", "extruder_temp": 220}, "hdr")
        assert "  Nozzle temp: 220°C" in s.splitlines()
        assert "(" not in s

    def test_drying_time_only(self):
        s = format_tag_summary({"material": "PLA", "drying_time_h": 6}, "hdr")
        assert "  Drying: ? for 6h" in s.splitlines()

    def test_drying_temp_only(self):
        s = format_tag_summary({"material": "PLA", "drying_temp": 55}, "hdr")
        assert "  Drying: 55°C" in s.splitlines()

    def test_uid_shown_only_with_decoded_fields(self):
        with_fields = format_tag_summary(
            {"material": "PLA", "uid": "AABB"}, "hdr")
        assert "  Tag UID: AABB" in with_fields.splitlines()
        bare = format_tag_summary({"uid": "AABB"}, "hdr")
        assert bare == "hdr"

    def test_new_lines_absent_when_unset(self):
        s = format_tag_summary({"material": "PLA"}, "hdr")
        for token in ("Density", "Drying", "Tag weight", "Length", "SKU",
                      "Serial", "Mfg date", "Tag UID"):
            assert token not in s


# ── Bambu decoder: nozzle diameter / spool width / length ─────────────────────

def _bambu_image(nozzle=0.4, spool_width=6620, length=330):
    d = bytearray(1024)
    d[32:35] = b"PLA"                                 # type (blk2)
    d[80:84] = bytes([0x12, 0x34, 0x56, 0xFF])        # RGBA (blk5)
    struct.pack_into("<f", d, 88, 1.75)               # diameter
    struct.pack_into("<f", d, 140, nozzle)            # nozzle diameter (blk8)
    d[164:166] = struct.pack("<H", spool_width)       # spool width (blk10)
    d[228:230] = struct.pack("<H", length)            # length in m (blk14)
    return bytes(d)


class TestDecodeBambuRich:
    def test_nozzle_spool_width_length_parsed(self):
        f = ace2.decode_bambu(_bambu_image())
        assert f["nozzle_diameter"] == 0.4
        assert f["spool_width_mm"] == 6620 / 100.0
        assert f["length_m"] == 330

    def test_nozzle_zero_is_none(self):
        f = ace2.decode_bambu(_bambu_image(nozzle=0.0))
        assert f["nozzle_diameter"] is None

    def test_nozzle_out_of_range_is_none(self):
        # garbage float in an unwritten block must not surface as a nozzle
        f = ace2.decode_bambu(_bambu_image(nozzle=87.0))
        assert f["nozzle_diameter"] is None

    def test_spool_width_zero_is_none(self):
        f = ace2.decode_bambu(_bambu_image(spool_width=0))
        assert f["spool_width_mm"] is None

    def test_length_zero_is_none(self):
        f = ace2.decode_bambu(_bambu_image(length=0))
        assert f["length_m"] is None


# ── Anycubic / Creality decoders: raw length_m ────────────────────────────────

def _anycubic_image(length=330):
    d = bytearray(0x80)
    d[0x10:0x14] = ace2.ANYCUBIC_MAGIC
    d[0x3C:0x3F] = b"PLA"
    d[0x7A:0x7C] = struct.pack("<H", length)
    return bytes(d)


class TestDecodeAnycubicLength:
    def test_length_kept(self):
        f = ace2.decode_anycubic(_anycubic_image(length=330))
        assert f["length_m"] == 330
        assert f["weight_g"] == 1000                  # 330 -> 1000g per table

    def test_length_zero_is_none(self):
        f = ace2.decode_anycubic(_anycubic_image(length=0))
        assert f["length_m"] is None


def _creality_payload(length_hex=b"0165"):
    payload = (b"ABC21" + b"0276" + b"01" + b"101001" + b"0FF5F0B"
               + length_hex + b"736314" + b"\x00" * 14)
    assert len(payload) == 48
    return payload


class TestDecodeCrealityLength:
    def test_length_kept(self):
        f = ace2.decode_creality(_creality_payload(b"0165"))
        assert f["length_m"] == 0x165                 # 357 m
        assert f["weight_g"] == 500

    def test_length_zero_is_none(self):
        f = ace2.decode_creality(_creality_payload(b"0000"))
        assert f["length_m"] is None
        assert f["weight_g"] is None


# ── AFC_U1_rfid._map_to_slot_info: rich fields ────────────────────────────────

from extras.AFC_U1_rfid import AFC_U1_RFID


class _U1Printer:
    def lookup_object(self, name, default=None):
        raise KeyError(name)                          # webhook reg is best-effort

    def get_reactor(self):
        return _ns(monotonic=lambda: 0.0)

    def register_event_handler(self, event, cb):
        return None


class _U1Config:
    def __init__(self):
        self._printer = _U1Printer()

    def get_printer(self):
        return self._printer

    def get(self, key, default=None):
        return {"lane_channels": "", "channels": "", "scanner_channels": "",
                "scanner_lanes": ""}.get(key, default)

    def getboolean(self, key, default=False):
        return default

    def getint(self, key, default=0, minval=None):
        return default

    def getfloat(self, key, default=0.0, minval=None):
        return default

    def error(self, msg):
        return Exception(msg)


def _u1():
    return AFC_U1_RFID(_U1Config())


class TestU1MapToSlotInfoRich:
    def test_weight_consumed(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA", "WEIGHT": 600})
        assert si["weight_g"] == 600

    def test_weight_zero_skipped(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA", "WEIGHT": 0})
        assert "weight_g" not in si

    def test_weight_invalid_skipped(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA", "WEIGHT": "junk"})
        assert "weight_g" not in si

    def test_diameter_from_tag(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA", "DIAMETER": 2.85})
        assert si["diameter"] == 2.85

    def test_diameter_defaults_without_key(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA"})
        assert si["diameter"] == 1.75

    def test_diameter_invalid_defaults(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA", "DIAMETER": "junk"})
        assert si["diameter"] == 1.75

    def test_temp_range_kept(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA",
                                      "HOTEND_MIN_TEMP": 200,
                                      "HOTEND_MAX_TEMP": 230})
        assert si["extruder_temp"] == (200 + 230) // 2
        assert si["extruder_temp_min"] == 200
        assert si["extruder_temp_max"] == 230

    def test_temp_range_absent_when_unset(self):
        si = _u1()._map_to_slot_info({"MAIN_TYPE": "PLA"})
        assert "extruder_temp_min" not in si
        assert "extruder_temp_max" not in si

    def test_optional_rich_keys_copied(self):
        si = _u1()._map_to_slot_info({
            "MAIN_TYPE": "PLA", "SERIAL": "S1", "DENSITY": 1.31,
            "DRYING_TEMP": 70, "DRYING_TIME": 8, "COLOR_NUMS": 2})
        assert si["serial"] == "S1"
        assert si["density"] == 1.31
        assert si["drying_temp"] == 70
        assert si["drying_time_h"] == 8
        assert si["color_count"] == 2

    def test_optional_rich_keys_skip_unset(self):
        si = _u1()._map_to_slot_info({
            "MAIN_TYPE": "PLA", "SERIAL": "", "DENSITY": 0,
            "DRYING_TEMP": None, "COLOR_NUMS": "0"})
        for key in ("serial", "density", "drying_temp", "color_count"):
            assert key not in si


class TestU1GetStatus:
    def test_shape_when_empty(self):
        status = _u1().get_status()
        assert status == {"lane_channel_map": {}, "scanner_channels": [],
                          "last_reads": {}}

    def test_last_reads_included_and_copied(self):
        reader = _u1()
        reader._tag_reads["lane1"] = {"material": "PLA"}
        status = reader.get_status()
        assert status["last_reads"] == {"lane1": {"material": "PLA"}}
        status["last_reads"].clear()
        assert reader._tag_reads == {"lane1": {"material": "PLA"}}
