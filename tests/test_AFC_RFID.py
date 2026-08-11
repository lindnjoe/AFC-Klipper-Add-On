"""
Tests for the shared RFID layer: extras/AFC_RFID.py (Spoolman sync and the
slot_info mapping every reader publishes through) and extras/AFC_rfid_keys.py.

Helpers, the rich-info decode, the per-unit key material, and a branch-coverage
sweep. Consolidated from four files; banners name the file each came from.
"""

from __future__ import annotations
import json
import types
import extras.AFC_RFID as _rfidmod
from extras.AFC_RFID import (
    AFCUnitRFID,
    SpoolmanClient,
    apply_filament_defaults,
    default_bed_temp_for_material,
    density_for_material,
    enrich_from_spool,
    find_spool_by_uid,
    get_auto_spoolman_create,
    log_new_filament,
    log_new_spool,
    sync_rfid_to_spoolman,
    _decode_extra,
    _missing_filament_fields,
    _spool_uids,
)
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
from extras.AFC_RFID import AFCUnitRFID
from extras.AFC_RFID import sync_rfid_to_spoolman
import logging
import struct
from extras.AFC_RFID import (
    AFCUnitRFID,
    SpoolmanClient,
    apply_filament_defaults,
    format_tag_summary,
    make_tag_record,
    map_tag_to_slot_info,
    sync_rfid_to_spoolman,
)
import extras.AFC_rfid_readers as readers
from extras.AFC_U1_rfid import AFC_U1_RFID
import pytest
from extras.AFC_rfid_keys import AFC_rfid_keys, load_config, _hex_key


# ── Branch-coverage tests for the shared RFID core in extras/AFC_RFID.py ──────
#
# was tests/test_AFC_RFID_coverage.py
def _ns_coverage(**kw):
    return types.SimpleNamespace(**kw)


class _Logger:
    """Recording logger: keeps an ordered (level, message) list."""

    def __init__(self):
        self.messages = []

    def _log(self, level, msg, args):
        self.messages.append((level, msg % args if args else msg))

    def info(self, msg, *args):
        self._log("info", msg, args)

    def error(self, msg, *args):
        self._log("error", msg, args)

    def warning(self, msg, *args):
        self._log("warning", msg, args)

    def debug(self, msg, *args):
        self._log("debug", msg, args)


class _FakeMoonraker:
    """Minimal moonraker stand-in for wrapping in a real SpoolmanClient."""

    def __init__(self, host="http://mr/", result=None):
        self.host = host
        self.logger = _Logger()
        self.result = result
        self.get_results_calls = []
        self.spools: dict = {}

    def _get_results(self, url_string, print_error=True):
        self.get_results_calls.append((url_string, print_error))
        return self.result

    def get_spool(self, spool_id):
        return self.spools.get(spool_id)


def _client_coverage(responder=None):
    """Build a real SpoolmanClient with _spoolman_proxy stubbed by ``responder``.

    Returns (client, calls) where calls records (method, path, body) tuples.
    """
    mr = _FakeMoonraker()
    client = SpoolmanClient(mr)
    calls = []

    def proxy(method, path, body=None, print_error=True):
        calls.append((method, path, body))
        return responder(method, path, body) if responder else None

    client._spoolman_proxy = proxy
    return client, calls


# ── SpoolmanClient._get_results ───────────────────────────────────────────────

class TestGetResults:
    def test_delegates_to_moonraker(self):
        mr = _FakeMoonraker(result="R")
        client = SpoolmanClient(mr)
        assert client._get_results("http://x", print_error=False) == "R"
        assert mr.get_results_calls == [("http://x", False)]


# ── SpoolmanClient._spoolman_proxy ────────────────────────────────────────────

class TestSpoolmanProxy:
    def test_get_none_result_does_not_log(self):
        mr = _FakeMoonraker(result=None)
        assert SpoolmanClient(mr)._spoolman_proxy("GET", "/v1/info") is None
        assert mr.logger.messages == []

    def test_non_get_none_result_logs_decoded_body(self):
        mr = _FakeMoonraker(result=None)
        client = SpoolmanClient(mr)
        assert client._spoolman_proxy("POST", "/v1/x", body='{"a": 1}') is None
        expected = json.dumps({"a": 1})
        assert mr.logger.messages == [
            ("error", f"Spoolman POST /v1/x failed; request body: {expected}")]

    def test_invalid_json_string_body_kept_as_string(self):
        mr = _FakeMoonraker(result=None)
        SpoolmanClient(mr)._spoolman_proxy("POST", "/v1/x", body="nothex")
        expected = json.dumps("nothex")
        assert mr.logger.messages == [
            ("error", f"Spoolman POST /v1/x failed; request body: {expected}")]

    def test_none_body_reports_none(self):
        mr = _FakeMoonraker(result=None)
        SpoolmanClient(mr)._spoolman_proxy("PATCH", "/v1/x")
        assert mr.logger.messages == [
            ("error", "Spoolman PATCH /v1/x failed; request body: (none)")]

    def test_success_dict_body_returns_result_no_log(self):
        mr = _FakeMoonraker(result={"ok": 1})
        client = SpoolmanClient(mr)
        assert client._spoolman_proxy("POST", "/v1/x", body={"a": 1}) == {"ok": 1}
        assert mr.logger.messages == []


# ── SpoolmanClient.reachable ──────────────────────────────────────────────────

class TestReachable:
    def test_true_when_info_returns(self):
        client, calls = _client_coverage(lambda m, p, b: {"version": "1"})
        assert client.reachable() is True
        assert calls == [("GET", "/v1/info", None)]

    def test_false_when_info_none(self):
        client, _ = _client_coverage(lambda m, p, b: None)
        assert client.reachable() is False

    def test_false_on_exception(self):
        client, _ = _client_coverage()

        def boom(*a, **k):
            raise RuntimeError("down")

        client._spoolman_proxy = boom
        assert client.reachable() is False


# ── SpoolmanClient.search_spools ──────────────────────────────────────────────

class TestSearchSpools:
    def test_no_filter_returns_list(self):
        client, calls = _client_coverage(lambda m, p, b: [{"id": 1}])
        assert client.search_spools() == [{"id": 1}]
        assert calls == [("GET", "/v1/spool", None)]

    def test_filter_by_filament_id(self):
        client, calls = _client_coverage(lambda m, p, b: [])
        client.search_spools(filament_id=5)
        assert calls == [("GET", "/v1/spool?filament.id=5", None)]

    def test_non_list_returns_empty(self):
        client, _ = _client_coverage(lambda m, p, b: {"not": "list"})
        assert client.search_spools() == []


# ── SpoolmanClient.get_or_create_vendor ───────────────────────────────────────

class TestGetOrCreateVendor:
    def test_exact_case_insensitive_match(self):
        resp = [{"id": 1, "name": "Other"}, {"id": 2, "name": "bambu lab"}]
        client, _ = _client_coverage(lambda m, p, b: resp if m == "GET" else None)
        assert client.get_or_create_vendor("Bambu Lab") == \
            {"id": 2, "name": "bambu lab"}

    def test_no_exact_match_returns_first(self):
        resp = [{"id": 3, "name": "Bambu X"}, {"id": 4, "name": "Bambu Y"}]
        client, _ = _client_coverage(lambda m, p, b: resp if m == "GET" else None)
        assert client.get_or_create_vendor("Elegoo") == {"id": 3, "name": "Bambu X"}

    def test_empty_list_creates_vendor(self):
        def responder(m, p, b):
            return [] if m == "GET" else {"id": 9, "name": "New"}

        client, calls = _client_coverage(responder)
        assert client.get_or_create_vendor("New") == {"id": 9, "name": "New"}
        assert ("POST", "/v1/vendor", json.dumps({"name": "New"})) in calls

    def test_none_response_creates_vendor(self):
        def responder(m, p, b):
            return None if m == "GET" else {"id": 10}

        client, _ = _client_coverage(responder)
        assert client.get_or_create_vendor("X") == {"id": 10}


# ── SpoolmanClient.create_filament ────────────────────────────────────────────

class TestCreateFilament:
    def test_minimal_only_name(self):
        client, calls = _client_coverage(lambda m, p, b: {"id": 1})
        client.create_filament(name="X")
        assert json.loads(calls[-1][2]) == {"name": "X"}

    def test_all_scalar_fields(self):
        client, calls = _client_coverage(lambda m, p, b: {"id": 1})
        client.create_filament(
            name="X", vendor_id=2, material="PLA", density=1.24, diameter=1.75,
            color_hex="#00ff00", settings_extruder_temp=220, settings_bed_temp=60,
            weight=1000, spool_weight=250, article_number="SKU")
        assert json.loads(calls[-1][2]) == {
            "name": "X", "vendor_id": 2, "material": "PLA", "density": 1.24,
            "diameter": 1.75, "color_hex": "00ff00", "settings_extruder_temp": 220,
            "settings_bed_temp": 60, "weight": 1000, "spool_weight": 250,
            "article_number": "SKU"}

    def test_multi_color_list_drops_color_hex_default_direction(self):
        client, calls = _client_coverage(lambda m, p, b: {"id": 1})
        client.create_filament(name="X", color_hex="#aaaaaa",
                               multi_color_hexes=["#aa0000", "00bb00"])
        body = json.loads(calls[-1][2])
        assert "color_hex" not in body
        assert body["multi_color_hexes"] == "aa0000,00bb00"
        assert body["multi_color_direction"] == "coaxial"

    def test_multi_color_string_and_explicit_direction(self):
        client, calls = _client_coverage(lambda m, p, b: {"id": 1})
        client.create_filament(name="X", multi_color_hexes="aa0000,00bb00",
                               multi_color_direction="longitudinal")
        body = json.loads(calls[-1][2])
        assert body["multi_color_hexes"] == "aa0000,00bb00"
        assert body["multi_color_direction"] == "longitudinal"


# ── SpoolmanClient.update_filament ────────────────────────────────────────────

class TestUpdateFilament:
    def test_empty_fields_noop(self):
        client, calls = _client_coverage()
        assert client.update_filament(5, {}) is None
        assert calls == []

    def test_patches_fields(self):
        client, calls = _client_coverage(lambda m, p, b: {"id": 5})
        assert client.update_filament(5, {"material": "PLA"}) == {"id": 5}
        assert calls == [("PATCH", "/v1/filament/5", {"material": "PLA"})]


# ── SpoolmanClient.create_spool ───────────────────────────────────────────────

class TestCreateSpool:
    def test_minimal(self):
        client, calls = _client_coverage(lambda m, p, b: {"id": 1})
        client.create_spool(filament_id=7)
        assert json.loads(calls[-1][2]) == {"filament_id": 7}

    def test_all_weights(self):
        client, calls = _client_coverage(lambda m, p, b: {"id": 1})
        client.create_spool(filament_id=7, initial_weight=1000,
                            remaining_weight=900, spool_weight=250)
        assert json.loads(calls[-1][2]) == {
            "filament_id": 7, "initial_weight": 1000, "remaining_weight": 900,
            "spool_weight": 250}


# ── SpoolmanClient._ensure_spool_fields ───────────────────────────────────────

class TestEnsureSpoolFields:
    def test_creates_missing_and_caches(self):
        client, calls = _client_coverage(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        client._ensure_spool_fields()
        client._ensure_spool_fields()
        assert len([c for c in calls if c[0] == "GET"]) == 1
        assert [c for c in calls if c[0] == "POST"] == [
            ("POST", "/v1/field/spool/card_uids",
             {"name": "Card UIDs", "field_type": "text"})]

    def test_skips_existing_field(self):
        client, calls = _client_coverage(
            lambda m, p, b: [{"key": "card_uids"}] if m == "GET" else None)
        client._ensure_spool_fields()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_still_creates(self):
        client, calls = _client_coverage(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_spool_fields()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/spool/card_uids"]

    def test_ignores_non_dict_entries(self):
        client, calls = _client_coverage(
            lambda m, p, b: ["junk", {"key": "card_uids"}] if m == "GET"
            else None)
        client._ensure_spool_fields()
        assert [c for c in calls if c[0] == "POST"] == []


# ── SpoolmanClient._ensure_flow_k_field ───────────────────────────────────────

class TestEnsureFlowKField:
    def test_creates_missing_and_caches(self):
        client, calls = _client_coverage(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        client._ensure_flow_k_field()
        client._ensure_flow_k_field()
        assert len([c for c in calls if c[0] == "GET"]) == 1
        assert [c for c in calls if c[0] == "POST"] == [
            ("POST", "/v1/field/spool/flow_k",
             {"name": "Flow K", "field_type": "float"})]

    def test_skips_existing_ignoring_non_dict(self):
        client, calls = _client_coverage(
            lambda m, p, b: ["junk", {"key": "flow_k"}] if m == "GET" else None)
        client._ensure_flow_k_field()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_still_creates(self):
        client, calls = _client_coverage(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_flow_k_field()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/spool/flow_k"]


# ── SpoolmanClient._ensure_filament_fields ────────────────────────────────────

class TestEnsureFilamentFields:
    def test_creates_variant_field_and_caches(self):
        client, calls = _client_coverage(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        client._ensure_filament_fields()
        client._ensure_filament_fields()
        assert len([c for c in calls if c[0] == "GET"]) == 1
        assert [c for c in calls if c[0] == "POST"] == [
            ("POST", "/v1/field/filament/variant",
             {"name": "Variant", "field_type": "text"})]

    def test_skips_existing_ignoring_non_dict(self):
        client, calls = _client_coverage(
            lambda m, p, b: ["junk", {"key": "variant"}] if m == "GET"
            else None)
        client._ensure_filament_fields()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_still_creates(self):
        client, calls = _client_coverage(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_filament_fields()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/filament/variant"]


# ── SpoolmanClient._ensure_drying_fields ──────────────────────────────────────

class TestEnsureDryingFields:
    def test_creates_only_missing_field(self):
        # time field already exists -> only the temp field is POSTed
        client, calls = _client_coverage(
            lambda m, p, b: [{"key": "drying_time_h"}] if m == "GET"
            else {"ok": 1})
        client._ensure_drying_fields()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/filament/drying_temp_c"]

    def test_skips_when_both_present_ignoring_non_dict(self):
        client, calls = _client_coverage(
            lambda m, p, b: ["junk", {"key": "drying_temp_c"},
                             {"key": "drying_time_h"}] if m == "GET"
            else {"ok": 1})
        client._ensure_drying_fields()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_creates_both(self):
        client, calls = _client_coverage(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_drying_fields()
        assert sorted(c[1] for c in calls if c[0] == "POST") == [
            "/v1/field/filament/drying_temp_c",
            "/v1/field/filament/drying_time_h"]


# ── SpoolmanClient.write_filament_variant ─────────────────────────────────────

class TestWriteFilamentVariant:
    def test_noop_when_empty(self):
        client, calls = _client_coverage()
        assert client.write_filament_variant(5, "") is None
        assert calls == []

    def test_noop_when_already_current(self):
        client, calls = _client_coverage(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        current = {"variant": json.dumps("Matte")}
        assert client.write_filament_variant(
            5, "Matte", current_extra=current) is None
        assert [c for c in calls if c[0] == "PATCH"] == []

    def test_writes_variant_merged(self):
        client, calls = _client_coverage(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        result = client.write_filament_variant(
            5, "Silk", current_extra={"other": "x"})
        assert result == {"ok": 1}
        patch = [c for c in calls if c[0] == "PATCH"][0]
        assert patch[1] == "/v1/filament/5"
        assert patch[2]["extra"]["variant"] == json.dumps("Silk")
        assert patch[2]["extra"]["other"] == "x"


# ── SpoolmanClient._patch_spool ───────────────────────────────────────────────

class TestPatchSpool:
    def test_noop_when_nothing(self):
        client, calls = _client_coverage()
        assert client._patch_spool(5) is None
        assert calls == []

    def test_lot_nr_only(self):
        client, calls = _client_coverage(lambda m, p, b: {"ok": 1})
        assert client._patch_spool(5, lot_nr="2024-01") == {"ok": 1}
        assert calls == [("PATCH", "/v1/spool/5", {"lot_nr": "2024-01"})]

    def test_extra_updates_merges_existing(self):
        client, calls = _client_coverage(lambda m, p, b: {"ok": 1})
        client._mr.spools[5] = {"extra": {"keep": "1"}}
        assert client._patch_spool(5, extra_updates={"new": "2"}) == {"ok": 1}
        assert calls == [
            ("PATCH", "/v1/spool/5", {"extra": {"keep": "1", "new": "2"}})]

    def test_extra_updates_absent_spool(self):
        client, calls = _client_coverage(lambda m, p, b: {"ok": 1})
        client._patch_spool(9, extra_updates={"new": "2"})
        assert calls == [("PATCH", "/v1/spool/9", {"extra": {"new": "2"}})]


# ── SpoolmanClient.write_spool_metadata ───────────────────────────────────────

class TestWriteSpoolMetadata:
    def test_noop_when_nothing(self):
        client, calls = _client_coverage()
        assert client.write_spool_metadata(5, lot_nr=None, uid=None) is None
        assert calls == []

    def test_lot_nr_only_no_uid(self):
        client, calls = _client_coverage(lambda m, p, b: {"ok": 1})
        assert client.write_spool_metadata(5, lot_nr="2024-05", uid="") == \
            {"ok": 1}
        assert calls == [("PATCH", "/v1/spool/5", {"lot_nr": "2024-05"})]

    def test_uid_merged_into_card_uids(self):
        def responder(m, p, b):
            return [] if (m, p) == ("GET", "/v1/field/spool") else {"ok": 1}

        client, calls = _client_coverage(responder)
        client._mr.spools[5] = {"extra": {"card_uids": json.dumps("AABB")}}
        client.write_spool_metadata(5, lot_nr="2024-05", uid="cc:dd")
        patch = [c for c in calls if c[0] == "PATCH"][0]
        assert patch[1] == "/v1/spool/5"
        assert patch[2]["lot_nr"] == "2024-05"
        assert patch[2]["extra"]["card_uids"] == json.dumps("AABB,CCDD")


# ── SpoolmanClient.get_spool ──────────────────────────────────────────────────

class TestGetSpool:
    def test_delegates_to_moonraker(self):
        client, _ = _client_coverage()
        client._mr.spools[3] = {"id": 3}
        assert client.get_spool(3) == {"id": 3}


# ── SpoolmanClient.read_flow_k ────────────────────────────────────────────────

class TestReadFlowK:
    def test_none_when_no_spool(self):
        client, _ = _client_coverage()
        assert client.read_flow_k(1) is None

    def test_reads_value(self):
        client, _ = _client_coverage()
        client._mr.spools[1] = {"extra": {"flow_k": json.dumps(1.234567)}}
        assert client.read_flow_k(1) == 1.234567

    def test_none_when_field_absent(self):
        client, _ = _client_coverage()
        client._mr.spools[1] = {"extra": {}}
        assert client.read_flow_k(1) is None

    def test_none_when_empty_string(self):
        client, _ = _client_coverage()
        client._mr.spools[1] = {"extra": {"flow_k": ""}}
        assert client.read_flow_k(1) is None

    def test_none_on_invalid_value(self):
        client, _ = _client_coverage()
        client._mr.spools[1] = {"extra": {"flow_k": "not-json{"}}
        assert client.read_flow_k(1) is None


# ── SpoolmanClient.write_flow_k ───────────────────────────────────────────────

class TestWriteFlowK:
    def test_none_when_no_spool(self):
        client, calls = _client_coverage(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        assert client.write_flow_k(1, 1.5) is None
        assert [c for c in calls if c[0] == "PATCH"] == []

    def test_writes_rounded_k_merged(self):
        client, calls = _client_coverage(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        client._mr.spools[1] = {"extra": {"keep": "x"}}
        assert client.write_flow_k(1, 1.23456789) == {"ok": 1}
        patch = [c for c in calls if c[0] == "PATCH"][0]
        assert patch[1] == "/v1/spool/1"
        assert patch[2]["extra"]["flow_k"] == json.dumps(round(1.23456789, 6))
        assert patch[2]["extra"]["keep"] == "x"


# ── _decode_extra ─────────────────────────────────────────────────────────────

class TestDecodeExtra:
    def test_absent_returns_none(self):
        assert _decode_extra({}, "k") is None
        assert _decode_extra(None, "k") is None

    def test_empty_string_returns_none(self):
        assert _decode_extra({"k": ""}, "k") is None

    def test_valid_json_decoded(self):
        assert _decode_extra({"k": json.dumps([1, 2])}, "k") == [1, 2]

    def test_non_json_returns_raw(self):
        assert _decode_extra({"k": "not json{"}, "k") == "not json{"


# ── _spool_uids ───────────────────────────────────────────────────────────────

class TestSpoolUids:
    def test_no_card_uids_empty(self):
        assert _spool_uids({"extra": {}}) == set()

    def test_blank_parts_skipped(self):
        s = {"extra": {"card_uids": json.dumps("AABB,,  ,CCDD")}}
        assert _spool_uids(s) == {"AABB", "CCDD"}


# ── find_spool_by_uid (empty-target guard) ────────────────────────────────────

class _Search:
    def __init__(self, spools):
        self._spools = spools

    def search_spools(self, filament_id=None):
        return self._spools


class TestFindSpoolByUidEmpty:
    def test_empty_uid_returns_none_without_search(self):
        assert find_spool_by_uid(_Search([{"id": 1}]), "") is None


# ── enrich_from_spool (false/exception branches) ──────────────────────────────

class TestEnrichFromSpool:
    def test_client_none_returns_copy(self):
        slot = {"material": "PLA"}
        out = enrich_from_spool(None, 5, slot)
        assert out == {"material": "PLA"} and out is not slot

    def test_get_spool_exception_returns_copy(self):
        class _C:
            def get_spool(self, sid):
                raise RuntimeError("down")

        out = enrich_from_spool(_C(), 5, {"material": "PLA"})
        assert out == {"material": "PLA"}

    def test_non_dict_spool_returns_copy(self):
        class _C:
            def get_spool(self, sid):
                return None

        out = enrich_from_spool(_C(), 5, {"material": "PLA"})
        assert out == {"material": "PLA"}

    def test_only_present_fields_overlaid(self):
        class _C:
            def get_spool(self, sid):
                return {"filament": {"name": "N"}}      # no vendor/material/...

        out = enrich_from_spool(_C(), 5, {"material": "PLA", "brand": "B"})
        assert out["display_name"] == "N"
        assert out["brand"] == "B"                      # untouched, no vendor
        assert out["material"] == "PLA"                 # untouched
        assert out["spool_id"] == 5
        assert "remaining_weight" not in out            # spool had none

    def test_vendor_without_filament_name(self):
        class _C:
            def get_spool(self, sid):
                return {"filament": {"vendor": {"name": "V"}},
                        "remaining_weight": 5}

        out = enrich_from_spool(_C(), 5, {"brand": "old"})
        assert "display_name" not in out                # filament had no name
        assert out["brand"] == "V"
        assert out["remaining_weight"] == 5


# ── log_new_filament ──────────────────────────────────────────────────────────

class TestLogNewFilament:
    def test_full_breakdown(self):
        logger = _Logger()
        log_new_filament(logger, "U1 RFID", {"id": 7}, "Bambu", "PLA",
                         "00ff00", 1.75, 220, 60, "SKU9")
        expected = "\n".join([
            "U1 RFID: created filament #7 in Spoolman:",
            "  vendor: Bambu",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
            "  SKU: SKU9",
        ])
        assert logger.messages == [("info", expected)]

    def test_minimal_skips_optionals(self):
        logger = _Logger()
        log_new_filament(logger, "U1 RFID", {}, "", "", "", 1.75, 0, 0)
        expected = "\n".join([
            "U1 RFID: created filament #? in Spoolman:",
            "  diameter: 1.75mm",
        ])
        assert logger.messages == [("info", expected)]


# ── log_new_spool ─────────────────────────────────────────────────────────────

class TestLogNewSpool:
    def test_with_spool_weight(self):
        logger = _Logger()
        log_new_spool(logger, "U1 RFID", {"id": 5}, 1000, spool_weight=250)
        expected = "\n".join([
            "U1 RFID: created spool #5 in Spoolman:",
            "  filament weight: 1000g",
            "  spool weight (tare): 250g",
            "  remaining: 1000g",
        ])
        assert logger.messages == [("info", expected)]

    def test_without_spool_weight(self):
        logger = _Logger()
        log_new_spool(logger, "U1 RFID", {}, 900)
        expected = "\n".join([
            "U1 RFID: created spool #? in Spoolman:",
            "  filament weight: 900g",
            "  remaining: 900g",
        ])
        assert logger.messages == [("info", expected)]


# ── get_auto_spoolman_create ──────────────────────────────────────────────────

class TestGetAutoSpoolmanCreate:
    def test_unit_opts_in(self):
        lane = _ns_coverage(unit_obj=_ns_coverage(auto_spoolman_create=True), extruder_obj=None)
        assert get_auto_spoolman_create(lane) is True

    def test_extruder_opts_in_when_unit_off(self):
        lane = _ns_coverage(unit_obj=_ns_coverage(auto_spoolman_create=False),
                   extruder_obj=_ns_coverage(auto_spoolman_create=True))
        assert get_auto_spoolman_create(lane) is True

    def test_both_present_but_off_returns_default(self):
        lane = _ns_coverage(unit_obj=_ns_coverage(auto_spoolman_create=False),
                   extruder_obj=_ns_coverage(auto_spoolman_create=False))
        assert get_auto_spoolman_create(lane, unit_default=True) is True

    def test_neither_present_returns_default(self):
        lane = _ns_coverage(unit_obj=None, extruder_obj=None)
        assert get_auto_spoolman_create(lane, unit_default=True) is True
        assert get_auto_spoolman_create(lane) is False


# ── default_bed_temp_for_material (edge branches) ─────────────────────────────

class TestDefaultBedTempEdges:
    def test_empty_and_none_return_none(self):
        assert default_bed_temp_for_material("") is None
        assert default_bed_temp_for_material(None) is None

    def test_all_symbols_return_none(self):
        assert default_bed_temp_for_material("!!!") is None

    def test_unknown_material_returns_none(self):
        assert default_bed_temp_for_material("xyz") is None


# ── apply_filament_defaults ───────────────────────────────────────────────────

def _lane_coverage(**kw):
    base = dict(material=None, color=None, extruder_temp=None, bed_temp=None,
                weight=0, spool_vendor="", multi_color=None, sub_type="",
                filament_density=1.24, spool_id=None)
    base.update(kw)
    return _ns_coverage(**base)


class TestApplyFilamentDefaults:
    def test_material_unknown_cleared(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "unknown"})
        assert lane.material is None

    def test_material_applied_when_unset(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.material == "PLA"

    def test_material_kept_when_set(self):
        lane = _lane_coverage(material="PETG")
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.material == "PETG"

    def test_color_hex_gets_hash_prefix(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA", "color_hex": "00ff00"})
        assert lane.color == "#00ff00"

    def test_color_hex_already_prefixed(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA", "color_hex": "#abcdef"})
        assert lane.color == "#abcdef"

    def test_color_from_converter_when_no_hex(self):
        lane = _lane_coverage()
        apply_filament_defaults(
            lane, {"material": "PLA", "color": [255, 0, 0]},
            color_converter=lambda rgb: "#ff0000")
        assert lane.color == "#ff0000"

    def test_color_converter_skipped_for_black(self):
        lane = _lane_coverage()
        called = []
        apply_filament_defaults(
            lane, {"material": "PLA", "color": [0, 0, 0]},
            color_converter=lambda rgb: called.append(rgb) or "#x")
        assert called == []
        assert lane.color is None

    def test_extruder_temp_applied_as_float(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA", "extruder_temp": 220})
        assert lane.extruder_temp == 220.0

    def test_extruder_temp_invalid_ignored(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA", "extruder_temp": "x"})
        assert lane.extruder_temp is None

    def test_bed_temp_applied_as_float(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA", "bed_temp": 60})
        assert lane.bed_temp == 60.0

    def test_bed_temp_invalid_then_material_default(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA", "bed_temp": "x"})
        assert lane.bed_temp == float(default_bed_temp_for_material("PLA"))

    def test_bed_default_when_tag_has_none(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "ABS"})
        assert lane.bed_temp == float(default_bed_temp_for_material("ABS"))

    def test_no_bed_default_for_unknown_material(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "xyz"})
        assert lane.bed_temp is None

    def test_sub_type_stashed(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA", "sub_type": "Matte"})
        assert lane.sub_type == "Matte"

    def test_weight_defaulted_when_zero(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.weight == 1000

    def test_weight_kept_when_set(self):
        lane = _lane_coverage(weight=500)
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.weight == 500

    def test_afc_defaults_fill_material_and_color(self):
        lane = _lane_coverage()
        apply_filament_defaults(
            lane, {}, afc_defaults={"default_material_type": "PLA",
                                    "default_color": "#123456"})
        assert lane.material == "PLA"
        assert lane.color == "#123456"

    def test_afc_defaults_not_used_when_tag_supplied(self):
        lane = _lane_coverage()
        apply_filament_defaults(
            lane, {"material": "PETG", "color_hex": "abcdef"},
            afc_defaults={"default_material_type": "PLA",
                          "default_color": "#123456"})
        assert lane.material == "PETG"
        assert lane.color == "#abcdef"

    def test_afc_defaults_present_but_empty_values(self):
        lane = _lane_coverage()
        apply_filament_defaults(
            lane, {}, afc_defaults={"default_material_type": "",
                                    "default_color": ""})
        assert lane.material is None
        assert lane.color is None

    def test_slot_info_none_only_defaults_weight(self):
        lane = _lane_coverage()
        apply_filament_defaults(lane, None)
        assert lane.weight == 1000
        assert lane.material is None
        assert lane.bed_temp is None


# ── _missing_filament_fields ──────────────────────────────────────────────────

class TestMissingFilamentFields:
    def test_all_empty_filament_filled(self):
        slot = {"material": "PLA", "diameter": 1.75, "extruder_temp": 220,
                "bed_temp": 60, "sku": "S1", "multi_color": ["00ff00"]}
        out = _missing_filament_fields({}, slot)
        assert out["material"] == "PLA"
        assert out["density"] == density_for_material("PLA")
        assert out["diameter"] == 1.75
        assert out["settings_extruder_temp"] == 220
        assert out["settings_bed_temp"] == 60
        assert out["article_number"] == "S1"
        assert out["color_hex"] == "00ff00"

    def test_nothing_when_all_present(self):
        fil = {"material": "PLA", "density": 1.24, "diameter": 1.75,
               "settings_extruder_temp": 220, "settings_bed_temp": 60,
               "article_number": "S1", "color_hex": "abcdef"}
        slot = {"material": "PETG", "diameter": 2.85, "extruder_temp": 250,
                "bed_temp": 90, "sku": "S2", "multi_color": ["112233"]}
        assert _missing_filament_fields(fil, slot) == {}

    def test_tag_density_wins_over_table(self):
        assert _missing_filament_fields({}, {"density": 1.31})["density"] == 1.31

    def test_material_density_kept_when_filament_has_one(self):
        out = _missing_filament_fields({"density": 1.0}, {"material": "PLA"})
        assert out == {"material": "PLA"}

    def test_multi_color_two_hexes(self):
        out = _missing_filament_fields({}, {"multi_color": ["#aa0000", "00bb00"]})
        assert out["multi_color_hexes"] == "aa0000,00bb00"
        assert out["multi_color_direction"] == "coaxial"

    def test_color_skipped_when_filament_has_multi(self):
        out = _missing_filament_fields(
            {"multi_color_hexes": "112233,445566"}, {"multi_color": ["aa0000"]})
        assert "color_hex" not in out and "multi_color_hexes" not in out

    def test_color_skipped_when_filament_has_single(self):
        out = _missing_filament_fields(
            {"color_hex": "abcdef"}, {"multi_color": ["aa0000"]})
        assert "color_hex" not in out

    def test_no_color_when_tag_has_none(self):
        out = _missing_filament_fields({}, {"material": "PLA"})
        assert "color_hex" not in out and "multi_color_hexes" not in out


# ── sync_rfid_to_spoolman ─────────────────────────────────────────────────────

class _SyncStub:
    """SpoolmanClient stub for sync_rfid_to_spoolman; records write calls."""

    def __init__(self, reachable=True, spool=None):
        self._reachable = reachable
        self._spool = spool
        self.created_filaments = []
        self.created_spools = []
        self.updates = []
        self.variant_writes = []
        self.drying_writes = []
        self.metadata = []
        self.create_filament_result = {"id": 99, "name": "N", "color_hex": ""}
        self.create_spool_result = {"id": 500, "remaining_weight": 1000}

    def reachable(self):
        return self._reachable

    def get_or_create_vendor(self, name):
        return {"id": 7}

    def create_filament(self, **kw):
        self.created_filaments.append(kw)
        return self.create_filament_result

    def create_spool(self, **kw):
        self.created_spools.append(kw)
        return self.create_spool_result

    def update_filament(self, fid, updates):
        self.updates.append((fid, updates))
        return None                                     # keep original filament

    def write_filament_variant(self, *a, **k):
        self.variant_writes.append((a, k))
        return None

    def write_filament_drying(self, *a, **k):
        self.drying_writes.append((a, k))
        return None

    def write_spool_metadata(self, *a, **k):
        self.metadata.append((a, k))
        return None


def _sync(monkeypatch, slot_info, stub, *, allow_create=True, set_next=False,
          spool_id_on_lane=None, spoolman=object(), moonraker=object()):
    monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: stub)
    monkeypatch.setattr(_rfidmod, "find_spool_by_uid", lambda c, u: stub._spool)
    logger = _Logger()
    spool_ns = _ns_coverage(next_spool_info=None, next_spool_id=None,
                   set_spoolID=lambda lane, sid: setattr(lane, "spool_id", sid))
    afc = _ns_coverage(spoolman=spoolman, moonraker=moonraker, spool=spool_ns)
    lane = _ns_coverage(name="lane1", spool_id=spool_id_on_lane,
               send_lane_data=lambda: None)
    sync_rfid_to_spoolman(afc, lane, slot_info, logger, "TEST",
                          allow_create=allow_create, set_next=set_next)
    return logger, lane, afc


class TestSyncRfidToSpoolman:
    def test_set_next_stashes_info_then_returns_no_spoolman(self, monkeypatch):
        stub = _SyncStub()
        logger, lane, afc = _sync(monkeypatch, {"uid": "AA", "material": "PLA"},
                                  stub, set_next=True, spoolman=None)
        assert afc.spool.next_spool_info == {"uid": "AA", "material": "PLA"}
        assert stub.created_filaments == []
        assert logger.messages == []

    def test_set_next_dict_failure_swallowed(self, monkeypatch):
        stub = _SyncStub()
        monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: stub)
        logger = _Logger()
        spool_ns = _ns_coverage(next_spool_info="orig", next_spool_id=None,
                       set_spoolID=lambda lane, sid: None)
        afc = _ns_coverage(spoolman=None, moonraker=object(), spool=spool_ns)
        lane = _ns_coverage(name="l", spool_id=None, send_lane_data=lambda: None)
        sync_rfid_to_spoolman(afc, lane, 123, logger, "TEST", set_next=True)
        assert afc.spool.next_spool_info == "orig"      # dict(123) failed
        assert logger.messages == []

    def test_existing_spool_id_skips(self, monkeypatch):
        stub = _SyncStub()
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AA", "material": "PLA", "color_hex": "00ff00"}, stub,
            spool_id_on_lane=42)
        assert stub.created_filaments == []
        assert lane.spool_id == 42
        assert logger.messages == []

    def test_unreachable_logs_and_returns(self, monkeypatch):
        stub = _SyncStub(reachable=False)
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AA", "material": "PLA", "color_hex": "00ff00"}, stub)
        assert stub.created_filaments == []
        assert logger.messages == [
            ("info", "TEST: Spoolman unreachable — using the tag's own values "
                     "on the lane (no Spoolman match this scan)")]

    def test_bed_default_none_not_written(self, monkeypatch):
        stub = _SyncStub(spool=None)
        slot = {"uid": "AABB", "material": "xyz", "color_hex": "00ff00"}
        logger, lane, afc = _sync(monkeypatch, slot, stub, allow_create=False)
        assert "bed_temp" not in slot                   # unknown -> no default
        assert logger.messages == [
            ("info", "TEST: no Spoolman spool matches UID AABB and auto-create "
                     "is OFF (set 'auto_spoolman_create: True' to create one)")]

    def test_no_match_no_uid_on(self, monkeypatch):
        stub = _SyncStub(spool=None)
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "", "material": "PLA", "color_hex": "00ff00"}, stub)
        assert logger.messages == [
            ("info", "TEST: no Spoolman spool matches this tag (no UID) and "
                     "auto-create is ON")]

    def test_incomplete_missing_material_and_colour(self, monkeypatch):
        stub = _SyncStub(spool=None)
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "material": "", "color_hex": ""}, stub)
        assert stub.created_filaments == []
        assert logger.messages == [
            ("info", "TEST: incomplete tag decode (missing material, colour) — "
                     "applied to the lane, not creating a Spoolman entry")]

    def test_matched_spool_no_backfill_assigns(self, monkeypatch):
        matched = {"id": 300, "remaining_weight": 800,
                   "filament": {"id": 88, "name": "Bambu PLA", "material": "PLA",
                                "density": 1.24, "diameter": 1.75,
                                "settings_extruder_temp": 220,
                                "settings_bed_temp": 60, "color_hex": "00ff00",
                                "article_number": "S1"}}
        stub = _SyncStub(spool=matched)
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABBCCDD", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60, "sku": "S1",
            "multi_color": ["00ff00"], "mfg_date": "2024-01"}, stub)
        assert stub.updates == []
        assert lane.spool_id == 300
        assert stub.metadata                            # UID/lot stamped
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABBCCDD"),
            ("info", "TEST: spool #300 ('Bambu PLA', #00ff00, 800g left) "
                     "assigned to lane1")]

    def test_matched_spool_backfill_logs(self, monkeypatch):
        matched = {"id": 301, "remaining_weight": 250.4,
                   "filament": {"id": 88, "name": "Existing",
                                "color_hex": "abcdef"}}
        stub = _SyncStub(spool=matched)
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "multi_color": ["00ff00"]}, stub)
        assert stub.updates[0][0] == 88
        keys = sorted(stub.updates[0][1])
        assert "color_hex" not in keys                  # filament already has one
        assert logger.messages == [
            ("info", "TEST: matched spool #301 by tag UID AABB"),
            ("info", f"TEST: backfilled {', '.join(keys)} on filament #88"),
            ("info", "TEST: spool #301 ('Existing', #abcdef, 250g left) "
                     "assigned to lane1")]

    def test_matched_backfill_exception_debug(self, monkeypatch):
        stub = _SyncStub(spool={"id": 300, "remaining_weight": 100,
                                "filament": {"id": 88, "name": "N",
                                             "color_hex": "00ff00"}})

        def boom(*a, **k):
            raise RuntimeError("bf fail")

        stub.update_filament = boom
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "multi_color": ["00ff00"]}, stub)
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("debug", "TEST: filament backfill skipped: bf fail"),
            ("info", "TEST: spool #300 ('N', #00ff00, 100g left) "
                     "assigned to lane1")]

    def test_metadata_stamp_failure_warns(self, monkeypatch):
        matched = {"id": 300, "remaining_weight": 100,
                   "filament": {"id": 88, "name": "N", "material": "PLA",
                                "density": 1.24, "diameter": 1.75,
                                "settings_extruder_temp": 220,
                                "settings_bed_temp": 60, "color_hex": "00ff00",
                                "article_number": "S1"}}
        stub = _SyncStub(spool=matched)

        def boom(*a, **k):
            raise RuntimeError("stamp fail")

        stub.write_spool_metadata = boom
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60, "sku": "S1",
            "multi_color": ["00ff00"], "mfg_date": "2024-01"}, stub)
        assert lane.spool_id == 300
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("warning", "TEST: stamping UID/lot on new spool #300 failed "
                        "(stamp fail) — next scan of this tag may not "
                        "re-match it"),
            ("info", "TEST: spool #300 ('N', #00ff00, 100g left) "
                     "assigned to lane1")]

    def test_no_match_no_create_off(self, monkeypatch):
        stub = _SyncStub(spool=None)
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "material": "PLA", "color_hex": "00ff00"}, stub,
            allow_create=False)
        assert stub.created_filaments == []
        assert logger.messages == [
            ("info", "TEST: no Spoolman spool matches UID AABB and auto-create "
                     "is OFF (set 'auto_spoolman_create: True' to create one)")]

    def test_create_filament_failure_warns(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = None
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "Bambu", "material": "PLA",
            "color_hex": "00ff00", "sub_type": "Basic"}, stub)
        assert stub.created_spools == []
        assert logger.messages == [
            ("warning", "TEST: Spoolman create_filament FAILED for "
                        "'Bambu PLA Basic' — check Spoolman/moonraker")]

    def test_created_filament_no_id_warns(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"name": "X"}     # no id
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60}, stub)
        fil_log = "\n".join([
            "TEST: created filament #? in Spoolman:",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
        ])
        assert logger.messages == [
            ("info", fil_log),
            ("warning", "TEST: resolved filament has no id — aborting")]

    def test_create_spool_failure_warns(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"id": 99, "name": "PLA",
                                       "color_hex": "00ff00"}
        stub.create_spool_result = None
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60}, stub)
        keys = sorted(stub.updates[0][1])
        fil_log = "\n".join([
            "TEST: created filament #99 in Spoolman:",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
        ])
        assert logger.messages == [
            ("info", fil_log),
            ("info", f"TEST: backfilled {', '.join(keys)} on filament #99"),
            ("warning", "TEST: Spoolman create_spool FAILED for filament #99 "
                        "— check Spoolman/moonraker")]

    def test_create_full_path_logs(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"id": 99, "name": "Bambu PLA Basic",
                                       "color_hex": "00ff00"}
        stub.create_spool_result = {"id": 500, "remaining_weight": 1000}
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABBCCDD", "brand": "Bambu", "material": "PLA",
            "sub_type": "Basic", "color_hex": "00ff00", "diameter": 1.75,
            "extruder_temp": 220, "bed_temp": 60, "sku": "SKU9",
            "multi_color": ["00ff00"], "mfg_date": "2024-01",
            "drying_temp": 70, "drying_time_h": 8, "weight_g": 750}, stub)
        assert lane.spool_id == 500
        assert stub.variant_writes and stub.drying_writes
        assert stub.created_filaments[0]["weight"] == 750
        keys = sorted(stub.updates[0][1])
        fil_log = "\n".join([
            "TEST: created filament #99 in Spoolman:",
            "  vendor: Bambu",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
            "  SKU: SKU9",
        ])
        spool_log = "\n".join([
            "TEST: created spool #500 in Spoolman:",
            "  filament weight: 750.0g",
            "  remaining: 750.0g",
        ])
        assert logger.messages == [
            ("info", fil_log),
            ("info", f"TEST: backfilled {', '.join(keys)} on filament #99"),
            ("info", spool_log),
            ("info", "TEST: spool #500 ('Bambu PLA Basic', #00ff00, 1000g left) "
                     "assigned to lane1")]

    def test_create_variant_and_drying_exceptions_debug(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"id": 99, "name": "PLA",
                                       "color_hex": "00ff00"}
        stub.create_spool_result = {"id": 500, "remaining_weight": 1000}

        def boom(*a, **k):
            raise RuntimeError("x")

        stub.write_filament_variant = boom
        stub.write_filament_drying = boom
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60,
            "sub_type": "Basic", "drying_temp": 70, "drying_time_h": 8}, stub)
        assert lane.spool_id == 500
        keys = sorted(stub.updates[0][1])
        fil_log = "\n".join([
            "TEST: created filament #99 in Spoolman:",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
        ])
        spool_log = "\n".join([
            "TEST: created spool #500 in Spoolman:",
            "  filament weight: 1000g",
            "  remaining: 1000g",
        ])
        assert logger.messages == [
            ("info", fil_log),
            ("info", f"TEST: backfilled {', '.join(keys)} on filament #99"),
            ("debug", "TEST: filament variant write skipped: x"),
            ("debug", "TEST: filament drying write skipped: x"),
            ("info", spool_log),
            ("info", "TEST: spool #500 ('PLA', #00ff00, 1000g left) "
                     "assigned to lane1")]

    def test_set_next_stages_spool_id(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"id": 99, "name": "PLA",
                                       "color_hex": "00ff00"}
        stub.create_spool_result = {"id": 500, "remaining_weight": 1000}
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60}, stub,
            set_next=True)
        assert afc.spool.next_spool_id == 500
        assert afc.spool.next_spool_info["uid"] == "AABB"
        assert logger.messages[-1] == (
            "info", "TEST: spool #500 ('PLA', #00ff00, 1000g left) staged as "
                    "next_spool_id")

    def test_matched_backfill_updated_dict_replaces_filament(self, monkeypatch):
        stub = _SyncStub(spool={"id": 300, "remaining_weight": 100,
                                "filament": {"id": 88}})

        def upd(fid, updates):
            stub.updates.append((fid, updates))
            return {"id": 88, "name": "Updated", "color_hex": "112233"}

        stub.update_filament = upd
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "multi_color": ["00ff00"]}, stub)
        keys = sorted(stub.updates[0][1])
        # PATCH result replaces the filament -> desc uses the updated name/color
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("info", f"TEST: backfilled {', '.join(keys)} on filament #88"),
            ("info", "TEST: spool #300 ('Updated', #112233, 100g left) "
                     "assigned to lane1")]

    def test_create_backfill_updated_dict_replaces_filament(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"id": 99, "name": "Init",
                                       "color_hex": "00ff00"}
        stub.create_spool_result = {"id": 500, "remaining_weight": 1000}

        def upd(fid, updates):
            stub.updates.append((fid, updates))
            return {"id": 99, "name": "Patched", "color_hex": "00ff00"}

        stub.update_filament = upd
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60}, stub)
        keys = sorted(stub.updates[0][1])
        fil_log = "\n".join([
            "TEST: created filament #99 in Spoolman:",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
        ])
        spool_log = "\n".join([
            "TEST: created spool #500 in Spoolman:",
            "  filament weight: 1000g",
            "  remaining: 1000g",
        ])
        assert logger.messages == [
            ("info", fil_log),
            ("info", f"TEST: backfilled {', '.join(keys)} on filament #99"),
            ("info", spool_log),
            ("info", "TEST: spool #500 ('Patched', #00ff00, 1000g left) "
                     "assigned to lane1")]

    def test_create_backfill_exception_debug(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"id": 99, "name": "PLA",
                                       "color_hex": "00ff00"}
        stub.create_spool_result = {"id": 500, "remaining_weight": 1000}

        def boom(*a, **k):
            raise RuntimeError("bf")

        stub.update_filament = boom
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60}, stub)
        assert lane.spool_id == 500
        fil_log = "\n".join([
            "TEST: created filament #99 in Spoolman:",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
        ])
        spool_log = "\n".join([
            "TEST: created spool #500 in Spoolman:",
            "  filament weight: 1000g",
            "  remaining: 1000g",
        ])
        assert logger.messages == [
            ("info", fil_log),
            ("debug", "TEST: filament backfill skipped: bf"),
            ("info", spool_log),
            ("info", "TEST: spool #500 ('PLA', #00ff00, 1000g left) "
                     "assigned to lane1")]

    def test_create_no_backfill_when_complete(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {
            "id": 99, "name": "Full", "material": "PLA", "density": 1.24,
            "diameter": 1.75, "settings_extruder_temp": 220,
            "settings_bed_temp": 60, "color_hex": "00ff00",
            "article_number": "S1"}
        stub.create_spool_result = {"id": 500, "remaining_weight": 1000}
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60, "sku": "S1",
            "multi_color": ["00ff00"]}, stub)
        assert stub.updates == []                       # nothing to backfill
        fil_log = "\n".join([
            "TEST: created filament #99 in Spoolman:",
            "  material: PLA",
            "  color: #00ff00",
            "  diameter: 1.75mm",
            "  nozzle temp: 220°C",
            "  bed temp: 60°C",
            "  SKU: S1",
        ])
        spool_log = "\n".join([
            "TEST: created spool #500 in Spoolman:",
            "  filament weight: 1000g",
            "  remaining: 1000g",
        ])
        assert logger.messages == [
            ("info", fil_log),
            ("info", spool_log),
            ("info", "TEST: spool #500 ('Full', #00ff00, 1000g left) "
                     "assigned to lane1")]

    def test_create_vendor_lookup_none_leaves_vendor_id_unset(self, monkeypatch):
        stub = _SyncStub(spool=None)
        stub.get_or_create_vendor = lambda name: None   # vendor create failed
        stub.create_filament_result = {"id": 99, "name": "Bambu PLA",
                                       "color_hex": "00ff00"}
        stub.create_spool_result = {"id": 500, "remaining_weight": 1000}
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "AABB", "brand": "Bambu", "material": "PLA",
            "color_hex": "00ff00", "diameter": 1.75, "extruder_temp": 220,
            "bed_temp": 60}, stub)
        assert stub.created_filaments[0]["vendor_id"] is None
        assert lane.spool_id == 500

    def test_brandless_tag_names_the_filament_after_its_material(
            self, monkeypatch):
        # No brand and no sub_type leaves build_filament_name nothing to build
        # from, so the material carries the name. (The "Unknown" fallback beside
        # it is unreachable: creation is already gated on material being
        # present, so `material or "Unknown"` can only ever yield the material.)
        stub = _SyncStub(spool=None)
        stub.create_filament_result = {"id": 98, "name": "PETG",
                                       "color_hex": "0000ff"}
        stub.create_spool_result = {"id": 502, "remaining_weight": 1000}
        logger, lane, afc = _sync(monkeypatch, {
            "uid": "EEFF", "material": "PETG", "color_hex": "0000ff",
            "diameter": 1.75, "extruder_temp": 240, "bed_temp": 80}, stub)
        assert stub.created_filaments[0]["name"] == "PETG"

    def test_outer_exception_logs_error(self, monkeypatch):
        matched = {"id": 300, "remaining_weight": 100,
                   "filament": {"id": 88, "name": "N", "material": "PLA",
                                "density": 1.24, "diameter": 1.75,
                                "settings_extruder_temp": 220,
                                "settings_bed_temp": 60, "color_hex": "00ff00",
                                "article_number": "S1"}}
        stub = _SyncStub(spool=matched)
        monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: stub)
        monkeypatch.setattr(_rfidmod, "find_spool_by_uid",
                            lambda c, u: stub._spool)
        logger = _Logger()

        def boom(lane, sid):
            raise RuntimeError("kaboom")

        spool_ns = _ns_coverage(next_spool_info=None, next_spool_id=None, set_spoolID=boom)
        afc = _ns_coverage(spoolman=object(), moonraker=object(), spool=spool_ns)
        lane = _ns_coverage(name="lane9", spool_id=None, send_lane_data=lambda: None)
        sync_rfid_to_spoolman(afc, lane, {
            "uid": "AABB", "material": "PLA", "color_hex": "00ff00",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60, "sku": "S1",
            "multi_color": ["00ff00"]}, logger, "TEST", allow_create=True)
        assert logger.messages == [
            ("info", "TEST: matched spool #300 by tag UID AABB"),
            ("error", "TEST Spoolman sync failed for lane9: kaboom")]


# ── AFCUnitRFID: apply_to_lane sync-error + _console_read_out / auto-create ────

class _ConsoleUnit(AFCUnitRFID):
    """Adapter exposing gcode + a recording logger for the console path."""

    def __init__(self, afc=None, gcode=None, auto_create=False):
        self.afc = afc
        self.auto_create = auto_create
        self.log_prefix = "TEST RFID"
        self.logger = _Logger()
        self.gcode = gcode

    def _map(self, tag):
        return {"uid": tag.get("uid"), "material": tag.get("material")}


class TestApplyToLaneSyncError:
    def test_sync_exception_warns(self, monkeypatch):
        monkeypatch.setattr(_rfidmod, "apply_filament_defaults",
                            lambda lane, si: None)

        def boom(*a, **k):
            raise RuntimeError("sync boom")

        monkeypatch.setattr(_rfidmod, "sync_rfid_to_spoolman", boom)
        unit = _ConsoleUnit(afc=_ns_coverage(spoolman=object(), moonraker=None),
                            gcode=None)
        out = unit.apply_to_lane(_ns_coverage(name="l"), {"uid": "aa", "material": "PLA"})
        assert out["material"] == "PLA"
        assert unit.logger.messages == [
            ("warning", "TEST RFID Spoolman sync failed: sync boom")]


class TestResolveAutoCreateNoneHelper:
    def test_none_helper_uses_unit_default(self, monkeypatch):
        monkeypatch.setattr(_rfidmod, "get_auto_spoolman_create", None)
        unit = _ConsoleUnit(afc=None, auto_create=True)
        assert unit._resolve_auto_create(_ns_coverage()) is True


class TestConsoleReadOut:
    def test_no_gcode_noop(self):
        unit = _ConsoleUnit(afc=_ns_coverage(moonraker=object()), gcode=None)
        unit._console_read_out(_ns_coverage(name="l", spool_id=5), {"material": "PLA"})
        assert unit.logger.messages == []

    def test_prints_summary_when_multiline(self):
        printed = []
        gcode = _ns_coverage(respond_info=lambda s: printed.append(s))
        unit = _ConsoleUnit(afc=_ns_coverage(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns_coverage(name="lane1", spool_id=None),
                               {"material": "PLA", "color_hex": "00ff00"})
        assert len(printed) == 1
        assert printed[0].startswith("TEST RFID: read spool on lane1")
        assert "  Material: PLA" in printed[0].splitlines()
        assert unit.logger.messages == []

    def test_enriches_when_spool_id_present(self, monkeypatch):
        printed = []
        gcode = _ns_coverage(respond_info=lambda s: printed.append(s))
        monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: object())
        monkeypatch.setattr(
            _rfidmod, "enrich_from_spool",
            lambda client, sid, si: dict(si, display_name="Enriched",
                                         spool_id=sid))
        unit = _ConsoleUnit(afc=_ns_coverage(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns_coverage(name="lane1", spool_id=7), {"material": "PLA"})
        lines = printed[0].splitlines()
        assert "  Name: Enriched" in lines
        assert "  Spoolman ID: 7" in lines

    def test_afc_none_skips_enrich(self, monkeypatch):
        printed = []
        gcode = _ns_coverage(respond_info=lambda s: printed.append(s))
        called = []
        monkeypatch.setattr(_rfidmod, "enrich_from_spool",
                            lambda *a, **k: called.append(1) or {})
        unit = _ConsoleUnit(afc=None, gcode=gcode)
        unit._console_read_out(_ns_coverage(name="lane1", spool_id=7), {"material": "PLA"})
        assert called == []                             # afc None -> mr None
        assert "  Material: PLA" in printed[0].splitlines()

    def test_bare_uid_summary_not_printed(self):
        printed = []
        gcode = _ns_coverage(respond_info=lambda s: printed.append(s))
        unit = _ConsoleUnit(afc=_ns_coverage(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns_coverage(name="lane1", spool_id=None), {"uid": "AABB"})
        assert printed == []                            # header only, no newline

    def test_no_lane_name(self):
        printed = []
        gcode = _ns_coverage(respond_info=lambda s: printed.append(s))
        unit = _ConsoleUnit(afc=_ns_coverage(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns_coverage(name="", spool_id=None), {"material": "PLA"})
        assert printed[0].startswith("TEST RFID: read spool\n")

    def test_exception_logged_debug(self):
        def boom(s):
            raise RuntimeError("ui down")

        gcode = _ns_coverage(respond_info=boom)
        unit = _ConsoleUnit(afc=_ns_coverage(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns_coverage(name="lane1", spool_id=None),
                               {"material": "PLA"})
        assert unit.logger.messages == [
            ("debug", "TEST RFID console read-out skipped: ui down")]


# ── Unit tests for the pure helper functions in extras/AFC_RFID.py ────────────
#
# was tests/test_AFC_RFID_helpers.py
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



class _Unit_helpers(AFCUnitRFID):
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
    unit = _Unit_helpers(afc=types_ns(spoolman=object()))
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
    unit = _Unit_helpers(afc=types_ns(spoolman=None))
    unit.apply_to_lane(object(), {"uid": "aa"})
    assert synced == []
    # afc None -> also no sync, no crash
    unit2 = _Unit_helpers(afc=None)
    unit2.apply_to_lane(object(), {"uid": "aa"})
    assert synced == []


def test_mixin_resolve_auto_create_prefers_lane(monkeypatch):
    monkeypatch.setattr(_rfidmod, "get_auto_spoolman_create",
                        lambda lane, default: True)
    unit = _Unit_helpers(afc=None, auto_create=False)
    assert unit._resolve_auto_create(object()) is True   # lane setting wins
    # if the helper raises, fall back to the unit default
    monkeypatch.setattr(_rfidmod, "get_auto_spoolman_create",
                        lambda lane, default: (_ for _ in ()).throw(RuntimeError()))
    unit2 = _Unit_helpers(afc=None, auto_create=True)
    assert unit2._resolve_auto_create(object()) is True


def types_ns(**kw):
    import types
    return types.SimpleNamespace(**kw)


# ── sync_rfid_to_spoolman: incomplete-decode guard, UID-only create ───────────



class _SyncClient_helpers:
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


def _run_sync_helpers(monkeypatch, slot_info, existing=None):
    client = _SyncClient_helpers(existing)
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
    client, lane = _run_sync_helpers(monkeypatch, {
        "uid": "AABBCCDD", "brand": "Bambu", "material": "PLA",
        "color_hex": "", "sub_type": "Basic"})
    assert client.created_filaments == []
    assert client.created_spools == []


def test_sync_creates_new_filament_for_new_uid(monkeypatch):
    # UID is the only match key; an unseen UID always creates a new filament +
    # spool (no colour/identity reuse).
    client, lane = _run_sync_helpers(monkeypatch, {
        "uid": "AABBCCDD", "brand": "Bambu", "material": "PLA",
        "color_hex": "ffffff", "sub_type": "Basic"})
    assert len(client.created_filaments) == 1
    assert len(client.created_spools) == 1
    assert lane.spool_id == 500


def test_sync_no_create_without_uid(monkeypatch):
    # no tag UID -> nothing to re-match on -> never create (SKU path is gone)
    client, lane = _run_sync_helpers(monkeypatch, {
        "uid": "", "brand": "Bambu", "material": "PLA",
        "color_hex": "ffffff", "sub_type": "Basic"})
    assert client.created_filaments == [] and client.created_spools == []


# ── Unit tests for the rich tag-info surfacing added to the AFC RFID stack: ───
#
# was tests/test_AFC_RFID_rich_info.py
def _ns_rich_info(**kw):
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

class _Unit_rich_info(AFCUnitRFID):
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
        unit = _Unit_rich_info()
        rec = unit.record_tag_read("lane1", {"material": "PLA"})
        assert unit.last_reads_status()["lane1"] == rec
        assert rec["material"] == "PLA" and rec["decoded"] is True

    def test_failed_decode_records_uid(self):
        unit = _Unit_rich_info()
        rec = unit.record_tag_read(3, None, decoded=False, uid="AABB",
                                   tag_type="MifareClassic1k")
        assert unit.last_reads_status()["3"] == rec
        assert rec["decoded"] is False and rec["uid"] == "AABB"

    def test_new_read_overwrites_old(self):
        unit = _Unit_rich_info()
        unit.record_tag_read("lane1", {"material": "PLA"})
        unit.record_tag_read("lane1", {"material": "PETG"})
        assert unit.last_reads_status()["lane1"]["material"] == "PETG"


class TestLastReadsStatus:
    def test_empty_before_first_read(self):
        assert _Unit_rich_info().last_reads_status() == {}

    def test_returns_copy(self):
        unit = _Unit_rich_info()
        unit.record_tag_read("lane1", {"material": "PLA"})
        status = unit.last_reads_status()
        status.clear()
        assert unit.last_reads_status() != {}


class TestUndecodedHint:
    def test_hint_for_fresh_undecoded_read(self, monkeypatch):
        unit = _Unit_rich_info()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False, uid="AABB",
                             tag_type="MifareClassic1k")
        hint = unit.undecoded_hint("lane1")
        assert hint == (" (saw tag UID AABB, MifareClassic1k"
                        " — no decoder/key matched)")

    def test_hint_without_tag_type(self, monkeypatch):
        unit = _Unit_rich_info()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False, uid="AABB")
        assert unit.undecoded_hint("lane1") == \
            " (saw tag UID AABB — no decoder/key matched)"

    def test_no_hint_for_decoded_read(self, monkeypatch):
        unit = _Unit_rich_info()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", {"material": "PLA", "uid": "AABB"})
        assert unit.undecoded_hint("lane1") == ""

    def test_no_hint_without_uid(self, monkeypatch):
        unit = _Unit_rich_info()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False)
        assert unit.undecoded_hint("lane1") == ""

    def test_no_hint_when_stale(self, monkeypatch):
        unit = _Unit_rich_info()
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1000.0)
        unit.record_tag_read("lane1", None, decoded=False, uid="AABB")
        monkeypatch.setattr(_rfidmod.time, "time", lambda: 1011.0)
        assert unit.undecoded_hint("lane1") == ""

    def test_no_hint_for_unknown_key(self):
        assert _Unit_rich_info().undecoded_hint("nope") == ""


# ── SpoolmanClient.write_filament_drying / _ensure_drying_fields ──────────────

def _client_rich_info():
    mr = _ns_rich_info(host="http://mr", logger=logging.getLogger("t"))
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
        client, calls = _client_rich_info()
        assert client.write_filament_drying(1, None, None) is None
        assert calls == []                            # not even the ensure GET

    def test_writes_both_fields(self):
        client, calls = _client_rich_info()
        result = client.write_filament_drying(7, 70, 8)
        assert result == {"ok": True}
        patches = [c for c in calls if c[0] == "PATCH"]
        assert len(patches) == 1
        method, path, body = patches[0]
        assert path == "/v1/filament/7"
        assert body["extra"]["drying_temp_c"] == "70"
        assert body["extra"]["drying_time_h"] == "8"

    def test_temp_only(self):
        client, calls = _client_rich_info()
        client.write_filament_drying(7, 65, None)
        body = [c for c in calls if c[0] == "PATCH"][0][2]
        assert body["extra"]["drying_temp_c"] == "65"
        assert "drying_time_h" not in body["extra"]

    def test_noop_when_values_current(self):
        client, calls = _client_rich_info()
        current = {"drying_temp_c": "70", "drying_time_h": "8"}
        assert client.write_filament_drying(7, 70, 8,
                                            current_extra=current) is None
        assert [c for c in calls if c[0] == "PATCH"] == []

    def test_ensure_creates_fields_once(self):
        client, calls = _client_rich_info()
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
        client, calls = _client_rich_info()

        def proxy(method, path, body=None, print_error=True):
            calls.append((method, path, body))
            if method == "GET":
                return [{"key": "drying_temp_c"}, {"key": "drying_time_h"}]
            return {"ok": True}

        client._spoolman_proxy = proxy
        client.write_filament_drying(1, 70, 8)
        assert [c for c in calls if c[0] == "POST"] == []


# ── apply_filament_defaults: rich lane fields ─────────────────────────────────

def _lane_rich_info(**overrides):
    lane = _ns_rich_info(material=None, color=None, extruder_temp=None, bed_temp=None,
               weight=0, spool_vendor="", multi_color=[],
               filament_density=1.24, sub_type="")
    for key, val in overrides.items():
        setattr(lane, key, val)
    return lane


class TestApplyFilamentDefaultsRich:
    def test_vendor_applied_when_unset(self):
        lane = _lane_rich_info()
        apply_filament_defaults(lane, {"brand": "Elegoo"})
        assert lane.spool_vendor == "Elegoo"

    def test_vendor_kept_when_already_set(self):
        lane = _lane_rich_info(spool_vendor="Existing")
        apply_filament_defaults(lane, {"brand": "Elegoo"})
        assert lane.spool_vendor == "Existing"

    def test_vendor_not_applied_when_tag_has_none(self):
        lane = _lane_rich_info()
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.spool_vendor == ""

    def test_multi_color_applied_for_dual(self):
        lane = _lane_rich_info()
        apply_filament_defaults(
            lane, {"multi_color": ["#aa0000", "00bb00"]})
        assert lane.multi_color == ["aa0000", "00bb00"]

    def test_multi_color_not_applied_for_single(self):
        lane = _lane_rich_info()
        apply_filament_defaults(lane, {"multi_color": ["aa0000"]})
        assert lane.multi_color == []

    def test_multi_color_kept_when_already_set(self):
        lane = _lane_rich_info(multi_color=["111111", "222222"])
        apply_filament_defaults(
            lane, {"multi_color": ["aa0000", "00bb00"]})
        assert lane.multi_color == ["111111", "222222"]

    def test_density_applied_when_tag_carries_one(self):
        lane = _lane_rich_info()
        apply_filament_defaults(lane, {"density": 1.31})
        assert lane.filament_density == 1.31

    def test_density_untouched_without_tag_value(self):
        lane = _lane_rich_info(filament_density=1.04)
        apply_filament_defaults(lane, {"material": "ABS"})
        assert lane.filament_density == 1.04

    def test_density_invalid_value_ignored(self):
        lane = _lane_rich_info(filament_density=1.04)
        apply_filament_defaults(lane, {"density": "junk"})
        assert lane.filament_density == 1.04


# ── sync_rfid_to_spoolman: weight / density / sku routing ─────────────────────

class _SyncClient_rich_info:
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


def _run_sync_rich_info(monkeypatch, slot_info):
    client = _SyncClient_rich_info()
    monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: client)
    monkeypatch.setattr(_rfidmod, "find_spool_by_uid", lambda c, u: None)
    spool = _ns_rich_info(next_spool_info=None, next_spool_id=None,
                set_spoolID=lambda lane, sid: setattr(lane, "spool_id", sid))
    afc = _ns_rich_info(spoolman=object(), moonraker=object(), spool=spool)
    lane = _ns_rich_info(name="lane1", spool_id=None, send_lane_data=lambda: None)
    sync_rfid_to_spoolman(afc, lane, slot_info, logging.getLogger("t"),
                          "TEST", allow_create=True)
    return client


_BASE_TAG = {"uid": "AABBCCDD", "brand": "BQ Tech", "material": "PLA",
             "color_hex": "aa0000", "diameter": 1.75}


class TestSyncWeightRouting:
    def test_tag_weight_seeds_creation_only(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG, weight_g=750))
        fil = client.created_filaments[0]
        spool = client.created_spools[0]
        assert fil["weight"] == 750
        assert spool["initial_weight"] == 750
        assert spool["remaining_weight"] == 750
        # NEVER tare: the tag's weight is net filament, not the empty spool
        assert "spool_weight" not in fil
        assert "spool_weight" not in spool

    def test_no_tag_weight_falls_back_to_1000(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG))
        assert client.created_filaments[0]["weight"] == 1000
        assert client.created_spools[0]["initial_weight"] == 1000
        assert client.created_spools[0]["remaining_weight"] == 1000

    def test_invalid_tag_weight_falls_back_to_1000(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG, weight_g="junk"))
        assert client.created_filaments[0]["weight"] == 1000


class TestSyncDensity:
    def test_tag_density_wins(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG, density=1.31))
        assert client.created_filaments[0]["density"] == 1.31

    def test_material_table_density_without_tag(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG))
        # independently derived: PLA -> 1.24 per the density table
        assert client.created_filaments[0]["density"] == 1.24


class TestSyncSkuAtCreate:
    def test_sku_passed_as_article_number(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG, sku="AC123"))
        assert client.created_filaments[0]["article_number"] == "AC123"

    def test_no_sku_passes_none(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG))
        assert client.created_filaments[0]["article_number"] is None


class TestSyncDrying:
    def test_drying_written_on_create(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch,
                           dict(_BASE_TAG, drying_temp=70, drying_time_h=8))
        assert client.drying_writes == [(99, 70, 8)]

    def test_no_drying_write_without_values(self, monkeypatch):
        client = _run_sync_rich_info(monkeypatch, dict(_BASE_TAG))
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
        f = readers.decode_bambu(_bambu_image())
        assert f["nozzle_diameter"] == 0.4
        assert f["spool_width_mm"] == 6620 / 100.0
        assert f["length_m"] == 330

    def test_nozzle_zero_is_none(self):
        f = readers.decode_bambu(_bambu_image(nozzle=0.0))
        assert f["nozzle_diameter"] is None

    def test_nozzle_out_of_range_is_none(self):
        # garbage float in an unwritten block must not surface as a nozzle
        f = readers.decode_bambu(_bambu_image(nozzle=87.0))
        assert f["nozzle_diameter"] is None

    def test_spool_width_zero_is_none(self):
        f = readers.decode_bambu(_bambu_image(spool_width=0))
        assert f["spool_width_mm"] is None

    def test_length_zero_is_none(self):
        f = readers.decode_bambu(_bambu_image(length=0))
        assert f["length_m"] is None


# ── Anycubic / Creality decoders: raw length_m ────────────────────────────────

def _anycubic_image(length=330):
    d = bytearray(0x80)
    d[0x10:0x14] = readers.ANYCUBIC_MAGIC
    d[0x3C:0x3F] = b"PLA"
    d[0x7A:0x7C] = struct.pack("<H", length)
    return bytes(d)


class TestDecodeAnycubicLength:
    def test_length_kept(self):
        f = readers.decode_anycubic(_anycubic_image(length=330))
        assert f["length_m"] == 330
        assert f["weight_g"] == 1000                  # 330 -> 1000g per table

    def test_length_zero_is_none(self):
        f = readers.decode_anycubic(_anycubic_image(length=0))
        assert f["length_m"] is None


def _creality_payload(length_hex=b"0165"):
    payload = (b"ABC21" + b"0276" + b"01" + b"101001" + b"0FF5F0B"
               + length_hex + b"736314" + b"\x00" * 14)
    assert len(payload) == 48
    return payload


class TestDecodeCrealityLength:
    def test_length_kept(self):
        f = readers.decode_creality(_creality_payload(b"0165"))
        assert f["length_m"] == 0x165                 # 357 m
        assert f["weight_g"] == 500

    def test_length_zero_is_none(self):
        f = readers.decode_creality(_creality_payload(b"0000"))
        assert f["length_m"] is None
        assert f["weight_g"] is None


# ── AFC_U1_rfid._map_to_slot_info: rich fields ────────────────────────────────



class _U1Printer:
    def lookup_object(self, name, default=None):
        raise KeyError(name)                          # webhook reg is best-effort

    def get_reactor(self):
        return _ns_rich_info(monotonic=lambda: 0.0)

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


# ── Unit tests for extras/AFC_rfid_keys.py ────────────────────────────────────
#
# was tests/test_AFC_rfid_keys.py
class _ConfigError(Exception):
    pass


class _Config:
    """Minimal Klipper config stand-in: get() + error()."""

    def __init__(self, opts=None):
        self._opts = opts or {}

    def get(self, key, default=None):
        return self._opts.get(key, default)

    def error(self, msg):
        return _ConfigError(msg)


# ── _hex_key ──────────────────────────────────────────────────────────────────

def test_hex_key_parses_to_bytes():
    cfg = _Config({"k": "aabbcc"})
    assert _hex_key(cfg, "k") == b"\xaa\xbb\xcc"


def test_hex_key_unset_is_none():
    assert _hex_key(_Config(), "missing") is None
    assert _hex_key(_Config({"k": "   "}), "k") is None    # blank -> None


def test_hex_key_bad_hex_raises_config_error():
    with pytest.raises(_ConfigError):
        _hex_key(_Config({"k": "nothex!"}), "k")


# ── AFC_rfid_keys ─────────────────────────────────────────────────────────────

def test_keys_parsed_from_section():
    obj = AFC_rfid_keys(_Config({
        "bambu_master_key": "00112233445566778899aabbccddeeff",
        "creality_key": "0f0e0d",
        # creality_encryption_key left unset
    }))
    assert obj.bambu_master_key == bytes.fromhex("00112233445566778899aabbccddeeff")
    assert obj.creality_key == b"\x0f\x0e\x0d"
    assert obj.creality_encryption_key is None


def test_load_config_returns_keys_object():
    obj = load_config(_Config({"bambu_master_key": "abcd"}))
    assert isinstance(obj, AFC_rfid_keys)
    assert obj.bambu_master_key == b"\xab\xcd"

