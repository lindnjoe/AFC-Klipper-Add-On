"""
Branch-coverage tests for the shared RFID core in extras/AFC_RFID.py.

Complements test_AFC_RFID_helpers / test_AFC_RFID_rich_info / test_AFC_rfid_keys
by driving the branches those files leave uncovered: the SpoolmanClient write
API, enrich/logging helpers, apply_filament_defaults, _missing_filament_fields,
the sync_rfid_to_spoolman state machine, and the AFCUnitRFID console path.
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


def _ns(**kw):
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


def _client(responder=None):
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
        client, calls = _client(lambda m, p, b: {"version": "1"})
        assert client.reachable() is True
        assert calls == [("GET", "/v1/info", None)]

    def test_false_when_info_none(self):
        client, _ = _client(lambda m, p, b: None)
        assert client.reachable() is False

    def test_false_on_exception(self):
        client, _ = _client()

        def boom(*a, **k):
            raise RuntimeError("down")

        client._spoolman_proxy = boom
        assert client.reachable() is False


# ── SpoolmanClient.search_spools ──────────────────────────────────────────────

class TestSearchSpools:
    def test_no_filter_returns_list(self):
        client, calls = _client(lambda m, p, b: [{"id": 1}])
        assert client.search_spools() == [{"id": 1}]
        assert calls == [("GET", "/v1/spool", None)]

    def test_filter_by_filament_id(self):
        client, calls = _client(lambda m, p, b: [])
        client.search_spools(filament_id=5)
        assert calls == [("GET", "/v1/spool?filament.id=5", None)]

    def test_non_list_returns_empty(self):
        client, _ = _client(lambda m, p, b: {"not": "list"})
        assert client.search_spools() == []


# ── SpoolmanClient.get_or_create_vendor ───────────────────────────────────────

class TestGetOrCreateVendor:
    def test_exact_case_insensitive_match(self):
        resp = [{"id": 1, "name": "Other"}, {"id": 2, "name": "bambu lab"}]
        client, _ = _client(lambda m, p, b: resp if m == "GET" else None)
        assert client.get_or_create_vendor("Bambu Lab") == \
            {"id": 2, "name": "bambu lab"}

    def test_no_exact_match_returns_first(self):
        resp = [{"id": 3, "name": "Bambu X"}, {"id": 4, "name": "Bambu Y"}]
        client, _ = _client(lambda m, p, b: resp if m == "GET" else None)
        assert client.get_or_create_vendor("Elegoo") == {"id": 3, "name": "Bambu X"}

    def test_empty_list_creates_vendor(self):
        def responder(m, p, b):
            return [] if m == "GET" else {"id": 9, "name": "New"}

        client, calls = _client(responder)
        assert client.get_or_create_vendor("New") == {"id": 9, "name": "New"}
        assert ("POST", "/v1/vendor", json.dumps({"name": "New"})) in calls

    def test_none_response_creates_vendor(self):
        def responder(m, p, b):
            return None if m == "GET" else {"id": 10}

        client, _ = _client(responder)
        assert client.get_or_create_vendor("X") == {"id": 10}


# ── SpoolmanClient.create_filament ────────────────────────────────────────────

class TestCreateFilament:
    def test_minimal_only_name(self):
        client, calls = _client(lambda m, p, b: {"id": 1})
        client.create_filament(name="X")
        assert json.loads(calls[-1][2]) == {"name": "X"}

    def test_all_scalar_fields(self):
        client, calls = _client(lambda m, p, b: {"id": 1})
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
        client, calls = _client(lambda m, p, b: {"id": 1})
        client.create_filament(name="X", color_hex="#aaaaaa",
                               multi_color_hexes=["#aa0000", "00bb00"])
        body = json.loads(calls[-1][2])
        assert "color_hex" not in body
        assert body["multi_color_hexes"] == "aa0000,00bb00"
        assert body["multi_color_direction"] == "coaxial"

    def test_multi_color_string_and_explicit_direction(self):
        client, calls = _client(lambda m, p, b: {"id": 1})
        client.create_filament(name="X", multi_color_hexes="aa0000,00bb00",
                               multi_color_direction="longitudinal")
        body = json.loads(calls[-1][2])
        assert body["multi_color_hexes"] == "aa0000,00bb00"
        assert body["multi_color_direction"] == "longitudinal"


# ── SpoolmanClient.update_filament ────────────────────────────────────────────

class TestUpdateFilament:
    def test_empty_fields_noop(self):
        client, calls = _client()
        assert client.update_filament(5, {}) is None
        assert calls == []

    def test_patches_fields(self):
        client, calls = _client(lambda m, p, b: {"id": 5})
        assert client.update_filament(5, {"material": "PLA"}) == {"id": 5}
        assert calls == [("PATCH", "/v1/filament/5", {"material": "PLA"})]


# ── SpoolmanClient.create_spool ───────────────────────────────────────────────

class TestCreateSpool:
    def test_minimal(self):
        client, calls = _client(lambda m, p, b: {"id": 1})
        client.create_spool(filament_id=7)
        assert json.loads(calls[-1][2]) == {"filament_id": 7}

    def test_all_weights(self):
        client, calls = _client(lambda m, p, b: {"id": 1})
        client.create_spool(filament_id=7, initial_weight=1000,
                            remaining_weight=900, spool_weight=250)
        assert json.loads(calls[-1][2]) == {
            "filament_id": 7, "initial_weight": 1000, "remaining_weight": 900,
            "spool_weight": 250}


# ── SpoolmanClient._ensure_spool_fields ───────────────────────────────────────

class TestEnsureSpoolFields:
    def test_creates_missing_and_caches(self):
        client, calls = _client(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        client._ensure_spool_fields()
        client._ensure_spool_fields()
        assert len([c for c in calls if c[0] == "GET"]) == 1
        assert [c for c in calls if c[0] == "POST"] == [
            ("POST", "/v1/field/spool/card_uids",
             {"name": "Card UIDs", "field_type": "text"})]

    def test_skips_existing_field(self):
        client, calls = _client(
            lambda m, p, b: [{"key": "card_uids"}] if m == "GET" else None)
        client._ensure_spool_fields()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_still_creates(self):
        client, calls = _client(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_spool_fields()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/spool/card_uids"]

    def test_ignores_non_dict_entries(self):
        client, calls = _client(
            lambda m, p, b: ["junk", {"key": "card_uids"}] if m == "GET"
            else None)
        client._ensure_spool_fields()
        assert [c for c in calls if c[0] == "POST"] == []


# ── SpoolmanClient._ensure_flow_k_field ───────────────────────────────────────

class TestEnsureFlowKField:
    def test_creates_missing_and_caches(self):
        client, calls = _client(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        client._ensure_flow_k_field()
        client._ensure_flow_k_field()
        assert len([c for c in calls if c[0] == "GET"]) == 1
        assert [c for c in calls if c[0] == "POST"] == [
            ("POST", "/v1/field/spool/flow_k",
             {"name": "Flow K", "field_type": "float"})]

    def test_skips_existing_ignoring_non_dict(self):
        client, calls = _client(
            lambda m, p, b: ["junk", {"key": "flow_k"}] if m == "GET" else None)
        client._ensure_flow_k_field()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_still_creates(self):
        client, calls = _client(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_flow_k_field()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/spool/flow_k"]


# ── SpoolmanClient._ensure_filament_fields ────────────────────────────────────

class TestEnsureFilamentFields:
    def test_creates_variant_field_and_caches(self):
        client, calls = _client(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        client._ensure_filament_fields()
        client._ensure_filament_fields()
        assert len([c for c in calls if c[0] == "GET"]) == 1
        assert [c for c in calls if c[0] == "POST"] == [
            ("POST", "/v1/field/filament/variant",
             {"name": "Variant", "field_type": "text"})]

    def test_skips_existing_ignoring_non_dict(self):
        client, calls = _client(
            lambda m, p, b: ["junk", {"key": "variant"}] if m == "GET"
            else None)
        client._ensure_filament_fields()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_still_creates(self):
        client, calls = _client(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_filament_fields()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/filament/variant"]


# ── SpoolmanClient._ensure_drying_fields ──────────────────────────────────────

class TestEnsureDryingFields:
    def test_creates_only_missing_field(self):
        # time field already exists -> only the temp field is POSTed
        client, calls = _client(
            lambda m, p, b: [{"key": "drying_time_h"}] if m == "GET"
            else {"ok": 1})
        client._ensure_drying_fields()
        assert [c[1] for c in calls if c[0] == "POST"] == \
            ["/v1/field/filament/drying_temp_c"]

    def test_skips_when_both_present_ignoring_non_dict(self):
        client, calls = _client(
            lambda m, p, b: ["junk", {"key": "drying_temp_c"},
                             {"key": "drying_time_h"}] if m == "GET"
            else {"ok": 1})
        client._ensure_drying_fields()
        assert [c for c in calls if c[0] == "POST"] == []

    def test_non_list_existing_creates_both(self):
        client, calls = _client(
            lambda m, p, b: None if m == "GET" else {"ok": 1})
        client._ensure_drying_fields()
        assert sorted(c[1] for c in calls if c[0] == "POST") == [
            "/v1/field/filament/drying_temp_c",
            "/v1/field/filament/drying_time_h"]


# ── SpoolmanClient.write_filament_variant ─────────────────────────────────────

class TestWriteFilamentVariant:
    def test_noop_when_empty(self):
        client, calls = _client()
        assert client.write_filament_variant(5, "") is None
        assert calls == []

    def test_noop_when_already_current(self):
        client, calls = _client(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        current = {"variant": json.dumps("Matte")}
        assert client.write_filament_variant(
            5, "Matte", current_extra=current) is None
        assert [c for c in calls if c[0] == "PATCH"] == []

    def test_writes_variant_merged(self):
        client, calls = _client(lambda m, p, b: [] if m == "GET" else {"ok": 1})
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
        client, calls = _client()
        assert client._patch_spool(5) is None
        assert calls == []

    def test_lot_nr_only(self):
        client, calls = _client(lambda m, p, b: {"ok": 1})
        assert client._patch_spool(5, lot_nr="2024-01") == {"ok": 1}
        assert calls == [("PATCH", "/v1/spool/5", {"lot_nr": "2024-01"})]

    def test_extra_updates_merges_existing(self):
        client, calls = _client(lambda m, p, b: {"ok": 1})
        client._mr.spools[5] = {"extra": {"keep": "1"}}
        assert client._patch_spool(5, extra_updates={"new": "2"}) == {"ok": 1}
        assert calls == [
            ("PATCH", "/v1/spool/5", {"extra": {"keep": "1", "new": "2"}})]

    def test_extra_updates_absent_spool(self):
        client, calls = _client(lambda m, p, b: {"ok": 1})
        client._patch_spool(9, extra_updates={"new": "2"})
        assert calls == [("PATCH", "/v1/spool/9", {"extra": {"new": "2"}})]


# ── SpoolmanClient.write_spool_metadata ───────────────────────────────────────

class TestWriteSpoolMetadata:
    def test_noop_when_nothing(self):
        client, calls = _client()
        assert client.write_spool_metadata(5, lot_nr=None, uid=None) is None
        assert calls == []

    def test_lot_nr_only_no_uid(self):
        client, calls = _client(lambda m, p, b: {"ok": 1})
        assert client.write_spool_metadata(5, lot_nr="2024-05", uid="") == \
            {"ok": 1}
        assert calls == [("PATCH", "/v1/spool/5", {"lot_nr": "2024-05"})]

    def test_uid_merged_into_card_uids(self):
        def responder(m, p, b):
            return [] if (m, p) == ("GET", "/v1/field/spool") else {"ok": 1}

        client, calls = _client(responder)
        client._mr.spools[5] = {"extra": {"card_uids": json.dumps("AABB")}}
        client.write_spool_metadata(5, lot_nr="2024-05", uid="cc:dd")
        patch = [c for c in calls if c[0] == "PATCH"][0]
        assert patch[1] == "/v1/spool/5"
        assert patch[2]["lot_nr"] == "2024-05"
        assert patch[2]["extra"]["card_uids"] == json.dumps("AABB,CCDD")


# ── SpoolmanClient.get_spool ──────────────────────────────────────────────────

class TestGetSpool:
    def test_delegates_to_moonraker(self):
        client, _ = _client()
        client._mr.spools[3] = {"id": 3}
        assert client.get_spool(3) == {"id": 3}


# ── SpoolmanClient.read_flow_k ────────────────────────────────────────────────

class TestReadFlowK:
    def test_none_when_no_spool(self):
        client, _ = _client()
        assert client.read_flow_k(1) is None

    def test_reads_value(self):
        client, _ = _client()
        client._mr.spools[1] = {"extra": {"flow_k": json.dumps(1.234567)}}
        assert client.read_flow_k(1) == 1.234567

    def test_none_when_field_absent(self):
        client, _ = _client()
        client._mr.spools[1] = {"extra": {}}
        assert client.read_flow_k(1) is None

    def test_none_when_empty_string(self):
        client, _ = _client()
        client._mr.spools[1] = {"extra": {"flow_k": ""}}
        assert client.read_flow_k(1) is None

    def test_none_on_invalid_value(self):
        client, _ = _client()
        client._mr.spools[1] = {"extra": {"flow_k": "not-json{"}}
        assert client.read_flow_k(1) is None


# ── SpoolmanClient.write_flow_k ───────────────────────────────────────────────

class TestWriteFlowK:
    def test_none_when_no_spool(self):
        client, calls = _client(lambda m, p, b: [] if m == "GET" else {"ok": 1})
        assert client.write_flow_k(1, 1.5) is None
        assert [c for c in calls if c[0] == "PATCH"] == []

    def test_writes_rounded_k_merged(self):
        client, calls = _client(lambda m, p, b: [] if m == "GET" else {"ok": 1})
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
        lane = _ns(unit_obj=_ns(auto_spoolman_create=True), extruder_obj=None)
        assert get_auto_spoolman_create(lane) is True

    def test_extruder_opts_in_when_unit_off(self):
        lane = _ns(unit_obj=_ns(auto_spoolman_create=False),
                   extruder_obj=_ns(auto_spoolman_create=True))
        assert get_auto_spoolman_create(lane) is True

    def test_both_present_but_off_returns_default(self):
        lane = _ns(unit_obj=_ns(auto_spoolman_create=False),
                   extruder_obj=_ns(auto_spoolman_create=False))
        assert get_auto_spoolman_create(lane, unit_default=True) is True

    def test_neither_present_returns_default(self):
        lane = _ns(unit_obj=None, extruder_obj=None)
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

def _lane(**kw):
    base = dict(material=None, color=None, extruder_temp=None, bed_temp=None,
                weight=0, spool_vendor="", multi_color=None, sub_type="",
                filament_density=1.24, spool_id=None)
    base.update(kw)
    return _ns(**base)


class TestApplyFilamentDefaults:
    def test_material_unknown_cleared(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "unknown"})
        assert lane.material is None

    def test_material_applied_when_unset(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.material == "PLA"

    def test_material_kept_when_set(self):
        lane = _lane(material="PETG")
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.material == "PETG"

    def test_color_hex_gets_hash_prefix(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA", "color_hex": "00ff00"})
        assert lane.color == "#00ff00"

    def test_color_hex_already_prefixed(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA", "color_hex": "#abcdef"})
        assert lane.color == "#abcdef"

    def test_color_from_converter_when_no_hex(self):
        lane = _lane()
        apply_filament_defaults(
            lane, {"material": "PLA", "color": [255, 0, 0]},
            color_converter=lambda rgb: "#ff0000")
        assert lane.color == "#ff0000"

    def test_color_converter_skipped_for_black(self):
        lane = _lane()
        called = []
        apply_filament_defaults(
            lane, {"material": "PLA", "color": [0, 0, 0]},
            color_converter=lambda rgb: called.append(rgb) or "#x")
        assert called == []
        assert lane.color is None

    def test_extruder_temp_applied_as_float(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA", "extruder_temp": 220})
        assert lane.extruder_temp == 220.0

    def test_extruder_temp_invalid_ignored(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA", "extruder_temp": "x"})
        assert lane.extruder_temp is None

    def test_bed_temp_applied_as_float(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA", "bed_temp": 60})
        assert lane.bed_temp == 60.0

    def test_bed_temp_invalid_then_material_default(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA", "bed_temp": "x"})
        assert lane.bed_temp == float(default_bed_temp_for_material("PLA"))

    def test_bed_default_when_tag_has_none(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "ABS"})
        assert lane.bed_temp == float(default_bed_temp_for_material("ABS"))

    def test_no_bed_default_for_unknown_material(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "xyz"})
        assert lane.bed_temp is None

    def test_sub_type_stashed(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA", "sub_type": "Matte"})
        assert lane.sub_type == "Matte"

    def test_weight_defaulted_when_zero(self):
        lane = _lane()
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.weight == 1000

    def test_weight_kept_when_set(self):
        lane = _lane(weight=500)
        apply_filament_defaults(lane, {"material": "PLA"})
        assert lane.weight == 500

    def test_afc_defaults_fill_material_and_color(self):
        lane = _lane()
        apply_filament_defaults(
            lane, {}, afc_defaults={"default_material_type": "PLA",
                                    "default_color": "#123456"})
        assert lane.material == "PLA"
        assert lane.color == "#123456"

    def test_afc_defaults_not_used_when_tag_supplied(self):
        lane = _lane()
        apply_filament_defaults(
            lane, {"material": "PETG", "color_hex": "abcdef"},
            afc_defaults={"default_material_type": "PLA",
                          "default_color": "#123456"})
        assert lane.material == "PETG"
        assert lane.color == "#abcdef"

    def test_afc_defaults_present_but_empty_values(self):
        lane = _lane()
        apply_filament_defaults(
            lane, {}, afc_defaults={"default_material_type": "",
                                    "default_color": ""})
        assert lane.material is None
        assert lane.color is None

    def test_slot_info_none_only_defaults_weight(self):
        lane = _lane()
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
    spool_ns = _ns(next_spool_info=None, next_spool_id=None,
                   set_spoolID=lambda lane, sid: setattr(lane, "spool_id", sid))
    afc = _ns(spoolman=spoolman, moonraker=moonraker, spool=spool_ns)
    lane = _ns(name="lane1", spool_id=spool_id_on_lane,
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
        spool_ns = _ns(next_spool_info="orig", next_spool_id=None,
                       set_spoolID=lambda lane, sid: None)
        afc = _ns(spoolman=None, moonraker=object(), spool=spool_ns)
        lane = _ns(name="l", spool_id=None, send_lane_data=lambda: None)
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

        spool_ns = _ns(next_spool_info=None, next_spool_id=None, set_spoolID=boom)
        afc = _ns(spoolman=object(), moonraker=object(), spool=spool_ns)
        lane = _ns(name="lane9", spool_id=None, send_lane_data=lambda: None)
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
        unit = _ConsoleUnit(afc=_ns(spoolman=object(), moonraker=None),
                            gcode=None)
        out = unit.apply_to_lane(_ns(name="l"), {"uid": "aa", "material": "PLA"})
        assert out["material"] == "PLA"
        assert unit.logger.messages == [
            ("warning", "TEST RFID Spoolman sync failed: sync boom")]


class TestResolveAutoCreateNoneHelper:
    def test_none_helper_uses_unit_default(self, monkeypatch):
        monkeypatch.setattr(_rfidmod, "get_auto_spoolman_create", None)
        unit = _ConsoleUnit(afc=None, auto_create=True)
        assert unit._resolve_auto_create(_ns()) is True


class TestConsoleReadOut:
    def test_no_gcode_noop(self):
        unit = _ConsoleUnit(afc=_ns(moonraker=object()), gcode=None)
        unit._console_read_out(_ns(name="l", spool_id=5), {"material": "PLA"})
        assert unit.logger.messages == []

    def test_prints_summary_when_multiline(self):
        printed = []
        gcode = _ns(respond_info=lambda s: printed.append(s))
        unit = _ConsoleUnit(afc=_ns(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns(name="lane1", spool_id=None),
                               {"material": "PLA", "color_hex": "00ff00"})
        assert len(printed) == 1
        assert printed[0].startswith("TEST RFID: read spool on lane1")
        assert "  Material: PLA" in printed[0].splitlines()
        assert unit.logger.messages == []

    def test_enriches_when_spool_id_present(self, monkeypatch):
        printed = []
        gcode = _ns(respond_info=lambda s: printed.append(s))
        monkeypatch.setattr(_rfidmod, "SpoolmanClient", lambda mr: object())
        monkeypatch.setattr(
            _rfidmod, "enrich_from_spool",
            lambda client, sid, si: dict(si, display_name="Enriched",
                                         spool_id=sid))
        unit = _ConsoleUnit(afc=_ns(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns(name="lane1", spool_id=7), {"material": "PLA"})
        lines = printed[0].splitlines()
        assert "  Name: Enriched" in lines
        assert "  Spoolman ID: 7" in lines

    def test_afc_none_skips_enrich(self, monkeypatch):
        printed = []
        gcode = _ns(respond_info=lambda s: printed.append(s))
        called = []
        monkeypatch.setattr(_rfidmod, "enrich_from_spool",
                            lambda *a, **k: called.append(1) or {})
        unit = _ConsoleUnit(afc=None, gcode=gcode)
        unit._console_read_out(_ns(name="lane1", spool_id=7), {"material": "PLA"})
        assert called == []                             # afc None -> mr None
        assert "  Material: PLA" in printed[0].splitlines()

    def test_bare_uid_summary_not_printed(self):
        printed = []
        gcode = _ns(respond_info=lambda s: printed.append(s))
        unit = _ConsoleUnit(afc=_ns(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns(name="lane1", spool_id=None), {"uid": "AABB"})
        assert printed == []                            # header only, no newline

    def test_no_lane_name(self):
        printed = []
        gcode = _ns(respond_info=lambda s: printed.append(s))
        unit = _ConsoleUnit(afc=_ns(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns(name="", spool_id=None), {"material": "PLA"})
        assert printed[0].startswith("TEST RFID: read spool\n")

    def test_exception_logged_debug(self):
        def boom(s):
            raise RuntimeError("ui down")

        gcode = _ns(respond_info=boom)
        unit = _ConsoleUnit(afc=_ns(moonraker=object()), gcode=gcode)
        unit._console_read_out(_ns(name="lane1", spool_id=None),
                               {"material": "PLA"})
        assert unit.logger.messages == [
            ("debug", "TEST RFID console read-out skipped: ui down")]
