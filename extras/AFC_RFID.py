# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Common RFID infrastructure for AFC. Provides shared utilities for
# material density, Spoolman sync (match by tag UID only), and filament
# defaults. Implementation-specific readers (U1, ACE, ACE2, Vivid) import
# from here.

from __future__ import annotations
import json
import re
import time
from typing import Any, Callable, Dict, Optional, Tuple, Union, TYPE_CHECKING
from urllib.request import Request
from urllib.parse import urljoin, quote

if TYPE_CHECKING:
    from klippy import Printer
    from extras.AFC_lane import AFCLane


class SpoolmanClient:
    """Spoolman write-client for the RFID sync path.

    The upstream AFC_moonraker only exposes read helpers (get_spool, GET
    _get_results) — none of the create/search methods our RFID needs. Rather
    than edit the upstream AFC_utils, this wraps the live moonraker
    object (reusing its host + GET plumbing) and adds the Spoolman write API.
    """

    def __init__(self, moonraker: Any) -> None:
        """Wrap a live moonraker object to add the Spoolman write API.

        :param moonraker: AFC moonraker object whose host and GET plumbing
            (``_get_results``, ``get_spool``) are reused.
        """
        self._mr = moonraker
        self.host = moonraker.host
        self.logger = moonraker.logger
        self._fields_ensured = False
        self._filament_fields_ensured = False
        self._flow_k_field_ensured = False
        self._drying_fields_ensured = False

    def _get_results(self, url_string: Union[str, Request],
                     print_error: bool = True) -> Any:
        """Delegate a GET/request to the wrapped moonraker's HTTP plumbing.

        :param url_string: URL string or ``urllib`` Request to fetch.
        :param print_error: Whether moonraker should log on failure.
        :return: Parsed result, or None on failure.
        """
        return self._mr._get_results(url_string, print_error)

    def _spoolman_proxy(self, method: str, path: str, body: Optional[Any] = None,
                        print_error: bool = True) -> Any:
        """Spoolman API call via moonraker's proxy endpoint.

        :param method: HTTP method (e.g. 'GET', 'POST', 'PATCH').
        :param path: Spoolman API path (e.g. '/v1/filament').
        :param body: Optional request body; a JSON string is decoded to an
            object so moonraker's proxy sets the JSON Content-Type.
        :param print_error: Whether to log on a failed request.
        :return: Parsed JSON result, or None on failure.
        """
        payload = {"request_method": method, "path": path}
        if body is not None:
            # Moonraker's proxy needs body as a JSON object so it sets
            # Content-Type: application/json upstream (a raw string body is
            # forwarded without that header and Spoolman 422s).
            if isinstance(body, str):
                try:
                    body = json.loads(body)
                except (ValueError, TypeError):
                    pass
            payload["body"] = body
        url = urljoin(self.host, 'server/spoolman/proxy')
        req = Request(url, json.dumps(payload).encode('utf-8'),
                      headers={"Content-Type": "application/json"})
        result = self._get_results(req, print_error)
        if result is None and method != "GET":
            self.logger.error(
                f"Spoolman {method} {path} failed; request body: "
                f"{json.dumps(body) if body is not None else '(none)'}")
        return result

    def reachable(self) -> bool:
        """Return True if the Spoolman server answers a lightweight request.

        Used to tell a genuine 'no match' apart from the server being down, so a
        real outage falls back to the tag's own values instead of creating a
        duplicate spool (not triggered on an empty but reachable Spoolman).
        """
        try:
            return self._spoolman_proxy(
                "GET", "/v1/info", print_error=False) is not None
        except Exception:
            return False

    def search_spools(self, filament_id: Optional[int] = None) -> list:
        """Search Spoolman spools, optionally filtered by filament id.

        :param filament_id: Filament id to filter spools by; all spools if None.
        :return list: Matching spool dicts (empty list if none/error).
        """
        parts = []
        if filament_id is not None:
            parts.append(f"filament.id={filament_id}")
        query = "&".join(parts)
        path = f"/v1/spool?{query}" if query else "/v1/spool"
        resp = self._spoolman_proxy("GET", path, print_error=False)
        return resp if isinstance(resp, list) else []

    def get_or_create_vendor(self, name: str) -> Optional[dict]:
        """Return an existing Spoolman vendor by name, creating it if absent.

        Prefers a case-insensitive exact name match, else the first result;
        creates a new vendor when none exist.

        :param name: Vendor name to look up or create.
        :return dict: The matched or newly created vendor dict.
        """
        resp = self._spoolman_proxy(
            "GET", f"/v1/vendor?name={quote(str(name))}", print_error=False)
        if isinstance(resp, list) and resp:
            for v in resp:
                if v.get("name", "").strip().lower() == name.strip().lower():
                    return v
            return resp[0]
        return self._spoolman_proxy("POST", "/v1/vendor",
                                    body=json.dumps({"name": name}))

    def create_filament(self, name: str, vendor_id: Optional[int] = None,
                        material: Optional[str] = None,
                        density: Optional[float] = None,
                        diameter: Optional[float] = None,
                        color_hex: Optional[str] = None,
                        settings_extruder_temp: Optional[float] = None,
                        settings_bed_temp: Optional[float] = None,
                        weight: Optional[float] = None,
                        spool_weight: Optional[float] = None,
                        article_number: Optional[str] = None,
                        multi_color_hexes: Optional[Any] = None,
                        multi_color_direction: Optional[str] = None) -> Optional[dict]:
        """Create a Spoolman filament from the given fields.

        Only non-None fields are sent. A multi-colour spool uses
        ``multi_color_hexes`` (and direction); since Spoolman accepts only one
        of ``color_hex`` / ``multi_color_hexes``, ``color_hex`` is dropped then.

        :param name: Filament name.
        :param vendor_id: Spoolman vendor id.
        :param material: Material string.
        :param density: Material density (g/cm^3).
        :param diameter: Filament diameter (mm).
        :param color_hex: Single colour hex (leading '#' stripped).
        :param settings_extruder_temp: Recommended extruder temperature.
        :param settings_bed_temp: Recommended bed temperature.
        :param weight: Net filament weight (g).
        :param spool_weight: Empty spool tare weight (g).
        :param article_number: Filament article number / SKU.
        :param multi_color_hexes: List/tuple or comma string of colour hexes
            for a multi-colour spool.
        :param multi_color_direction: Multi-colour direction (default 'coaxial').
        :return dict: The created filament dict, or None on failure.
        """
        data = {"name": name}
        if vendor_id is not None: data["vendor_id"] = vendor_id
        if material is not None: data["material"] = material
        if density is not None: data["density"] = density
        if diameter is not None: data["diameter"] = diameter
        if color_hex is not None: data["color_hex"] = color_hex.lstrip("#")
        # Multi-colour spools use multi_color_hexes (+ direction); Spoolman
        # accepts only one of color_hex / multi_color_hexes, so drop color_hex.
        if multi_color_hexes:
            data.pop("color_hex", None)
            data["multi_color_hexes"] = ",".join(
                c.lstrip("#") for c in multi_color_hexes) \
                if isinstance(multi_color_hexes, (list, tuple)) \
                else multi_color_hexes
            data["multi_color_direction"] = multi_color_direction or "coaxial"
        if settings_extruder_temp is not None:
            data["settings_extruder_temp"] = settings_extruder_temp
        if settings_bed_temp is not None: data["settings_bed_temp"] = settings_bed_temp
        if weight is not None: data["weight"] = weight
        if spool_weight is not None: data["spool_weight"] = spool_weight
        if article_number is not None: data["article_number"] = article_number
        return self._spoolman_proxy("POST", "/v1/filament", body=json.dumps(data))

    def update_filament(self, filament_id: int, fields: dict) -> Optional[dict]:
        """PATCH a filament with the given fields (used to backfill empties on a
        matched filament). No-op on an empty dict.

        :param filament_id: Spoolman filament id to update.
        :param fields: Dict of {field: value} to PATCH.
        :return: PATCH result dict, or None when there's nothing to do.
        """
        if not fields:
            return None
        return self._spoolman_proxy("PATCH", f"/v1/filament/{filament_id}",
                                    body=fields)

    def create_spool(self, filament_id: int, initial_weight: Optional[float] = None,
                     remaining_weight: Optional[float] = None,
                     spool_weight: Optional[float] = None) -> Optional[dict]:
        """Create a Spoolman spool for a filament.

        :param filament_id: Spoolman filament id this spool is made of.
        :param initial_weight: Initial net filament weight (g).
        :param remaining_weight: Remaining net filament weight (g).
        :param spool_weight: Empty spool tare weight (g).
        :return dict: The created spool dict, or None on failure.
        """
        data = {"filament_id": filament_id}
        if initial_weight is not None: data["initial_weight"] = initial_weight
        if remaining_weight is not None: data["remaining_weight"] = remaining_weight
        if spool_weight is not None: data["spool_weight"] = spool_weight
        return self._spoolman_proxy("POST", "/v1/spool", body=json.dumps(data))

    # ── Spool metadata: extra fields (RFID UID) + lot_nr ──
    # The tag UID lives in a dedicated Spoolman *extra field* (created on
    # demand), not the comment — so the comment stays free for the user. The
    # tag's manufacturing date goes to the built-in lot_nr. Extra-field values
    # are JSON-encoded per Spoolman (float -> "1.23", text -> "\"abc\"").
    # NFC tag UIDs live in a 'card_uids' spool extra field as a comma-separated
    # list of uppercase-hex UIDs — matching the Snapmaker-Extended convention so
    # spools created by either tool interoperate (a spool can carry >1 tag).
    SPOOL_EXTRA_CARD_UIDS = "card_uids"      # field_type: text
    # Flow K (auto-calibration) lives in its own 'flow_k' spool extra field, but
    # that field is created lazily by write_flow_k() ONLY when AFC_autocal
    # persists a value — so it never appears in Spoolman for users who don't run
    # auto-cal. See read_flow_k / write_flow_k below.
    SPOOL_EXTRA_FLOW_K = "flow_k"            # field_type: float (name "Flow K")
    # The tag's filament sub-type / variant (e.g. "Matte", "Silk", "Basic") lives
    # in a 'variant' filament extra field (also matching Snapmaker-Extended).
    FILAMENT_EXTRA_VARIANT = "variant"
    # Tag drying recommendations (Bambu / BTT tags) live in their own filament
    # extra fields, created lazily ONLY when a tag actually carries drying data
    # — so they never appear in Spoolman for users whose tags don't have them.
    FILAMENT_EXTRA_DRYING_TEMP = "drying_temp_c"   # field_type: integer
    FILAMENT_EXTRA_DRYING_TIME = "drying_time_h"   # field_type: integer

    def _ensure_spool_fields(self) -> None:
        """Create the RFID 'card_uids' spool extra field if it doesn't exist.

        Idempotent and cached: one GET /v1/field/spool per client, then a POST
        only if missing. POST is create-or-update, so a re-create is harmless if
        a race adds it first. (Flow K is created separately, gated on auto-cal —
        see _ensure_flow_k_field.)

        :return: None
        """
        if self._fields_ensured:
            return
        existing = self._spoolman_proxy("GET", "/v1/field/spool",
                                        print_error=False)
        keys = set()
        if isinstance(existing, list):
            keys = {f.get("key") for f in existing if isinstance(f, dict)}
        if self.SPOOL_EXTRA_CARD_UIDS not in keys:
            self._spoolman_proxy(
                "POST", f"/v1/field/spool/{self.SPOOL_EXTRA_CARD_UIDS}",
                body={"name": "Card UIDs", "field_type": "text"})
        self._fields_ensured = True

    def _ensure_flow_k_field(self) -> None:
        """Create the 'flow_k' spool extra field if it doesn't exist yet.

        Kept separate from _ensure_spool_fields and called ONLY from
        write_flow_k, so the field is created lazily the first time AFC_autocal
        persists a K — never for users who don't run auto-calibration.

        :return: None
        """
        if self._flow_k_field_ensured:
            return
        existing = self._spoolman_proxy("GET", "/v1/field/spool",
                                        print_error=False)
        keys = set()
        if isinstance(existing, list):
            keys = {f.get("key") for f in existing if isinstance(f, dict)}
        if self.SPOOL_EXTRA_FLOW_K not in keys:
            self._spoolman_proxy(
                "POST", f"/v1/field/spool/{self.SPOOL_EXTRA_FLOW_K}",
                body={"name": "Flow K", "field_type": "float"})
        self._flow_k_field_ensured = True

    def _ensure_filament_fields(self) -> None:
        """Create the AFC filament extra fields if they don't exist yet.
        Idempotent and cached (one GET /v1/field/filament per client).

        :return: None
        """
        if self._filament_fields_ensured:
            return
        existing = self._spoolman_proxy("GET", "/v1/field/filament",
                                        print_error=False)
        keys = set()
        if isinstance(existing, list):
            keys = {f.get("key") for f in existing if isinstance(f, dict)}
        if self.FILAMENT_EXTRA_VARIANT not in keys:
            self._spoolman_proxy(
                "POST", f"/v1/field/filament/{self.FILAMENT_EXTRA_VARIANT}",
                body={"name": "Variant", "field_type": "text"})
        self._filament_fields_ensured = True

    def _ensure_drying_fields(self) -> None:
        """Create the drying filament extra fields if they don't exist yet.

        Kept separate from _ensure_filament_fields and called ONLY from
        write_filament_drying, so the fields are created lazily the first time
        a tag actually carries drying data.

        :return: None
        """
        if self._drying_fields_ensured:
            return
        existing = self._spoolman_proxy("GET", "/v1/field/filament",
                                        print_error=False)
        keys = set()
        if isinstance(existing, list):
            keys = {f.get("key") for f in existing if isinstance(f, dict)}
        if self.FILAMENT_EXTRA_DRYING_TEMP not in keys:
            self._spoolman_proxy(
                "POST", f"/v1/field/filament/{self.FILAMENT_EXTRA_DRYING_TEMP}",
                body={"name": "Drying temp (C)", "field_type": "integer"})
        if self.FILAMENT_EXTRA_DRYING_TIME not in keys:
            self._spoolman_proxy(
                "POST", f"/v1/field/filament/{self.FILAMENT_EXTRA_DRYING_TIME}",
                body={"name": "Drying time (h)", "field_type": "integer"})
        self._drying_fields_ensured = True

    def write_filament_drying(self, filament_id: int,
                              drying_temp: Optional[int],
                              drying_time_h: Optional[int],
                              current_extra: Optional[dict] = None) -> Optional[dict]:
        """Store a tag's drying recommendation in the drying filament extra
        fields. Merges with any existing extra; a no-op when neither value is
        set or both are already current.

        :param filament_id: Spoolman filament id to update.
        :param drying_temp: Drying temperature (C), or None.
        :param drying_time_h: Drying time (hours), or None.
        :param current_extra: The filament's existing extra dict, to merge into.
        :return: PATCH result dict, or None when there's nothing to write.
        """
        if not drying_temp and not drying_time_h:
            return None
        self._ensure_drying_fields()
        extra = dict(current_extra or {})
        changed = False
        for key, value in ((self.FILAMENT_EXTRA_DRYING_TEMP, drying_temp),
                           (self.FILAMENT_EXTRA_DRYING_TIME, drying_time_h)):
            if not value:
                continue
            new_val = json.dumps(int(value))
            if extra.get(key) != new_val:
                extra[key] = new_val
                changed = True
        if not changed:
            return None
        return self._spoolman_proxy(
            "PATCH", f"/v1/filament/{filament_id}", body={"extra": extra})

    def write_filament_variant(self, filament_id: int, variant: str,
                               current_extra: Optional[dict] = None) -> Optional[dict]:
        """Store the filament sub-type/variant (e.g. 'Matte', 'Silk') in the
        'variant' filament extra field. Merges with any existing extra; a no-op
        when the value is empty or already current.

        :param filament_id: Spoolman filament id to update.
        :param variant: Sub-type/variant string to store.
        :param current_extra: The filament's existing extra dict, to merge into.
        :return: PATCH result dict, or None when there's nothing to write.
        """
        if not variant:
            return None
        self._ensure_filament_fields()
        extra = dict(current_extra or {})
        new_val = json.dumps(str(variant))
        if extra.get(self.FILAMENT_EXTRA_VARIANT) == new_val:
            return None
        extra[self.FILAMENT_EXTRA_VARIANT] = new_val
        return self._spoolman_proxy(
            "PATCH", f"/v1/filament/{filament_id}", body={"extra": extra})

    def _patch_spool(self, spool_id: int, lot_nr: Optional[str] = None,
                     extra_updates: Optional[dict] = None) -> Optional[dict]:
        """PATCH a spool's lot_nr and/or extra fields. Reads the current extra
        and merges, so other extra fields (e.g. slicer presets) are preserved.

        :param spool_id: Spoolman spool id to update.
        :param lot_nr: Lot number (manufacturing date) to set.
        :param extra_updates: Dict of extra-field updates to merge in.
        :return: PATCH result dict, or None when there's nothing to write.
        """
        body = {}
        if lot_nr is not None:
            body["lot_nr"] = lot_nr
        if extra_updates:
            spool = self.get_spool(spool_id)
            extra = dict((spool or {}).get("extra") or {})
            extra.update(extra_updates)
            body["extra"] = extra
        if not body:
            return None
        return self._spoolman_proxy("PATCH", f"/v1/spool/{spool_id}", body=body)

    def write_spool_metadata(self, spool_id: int, lot_nr: Optional[str] = None,
                             uid: Optional[str] = None) -> Optional[dict]:
        """Write the tag's manufacturing date (-> lot_nr) and NFC UID (-> the
        'card_uids' extra field, merged into the spool's existing UID list as
        comma-separated uppercase hex).

        :param spool_id: Spoolman spool id to update.
        :param lot_nr: Manufacturing date string to store as lot_nr.
        :param uid: NFC tag UID to add to the spool's 'card_uids' list.
        :return: PATCH result dict, or None when there's nothing to write.
        """
        extra = None
        norm = _norm_uid(uid)
        if norm:
            self._ensure_spool_fields()
            uids = _spool_uids(self.get_spool(spool_id) or {})
            uids.add(norm)
            extra = {self.SPOOL_EXTRA_CARD_UIDS:
                     json.dumps(",".join(sorted(uids)))}
        if lot_nr is None and extra is None:
            return None
        return self._patch_spool(spool_id, lot_nr=lot_nr, extra_updates=extra)

    def get_spool(self, spool_id: int) -> Optional[dict]:
        """Read a spool dict from Spoolman (delegated to the live moonraker,
        which already has get_spool).

        :param spool_id: Spoolman spool id to fetch.
        :return dict: The spool dict, or None if not found.
        """
        return self._mr.get_spool(spool_id)

    def read_flow_k(self, spool_id: int) -> Optional[float]:
        """Read flow K from the 'flow_k' extra field.

        :param spool_id: Spoolman spool id to read.
        :return float: The stored flow K, or None if unset/unavailable.
        """
        spool = self.get_spool(spool_id)
        if not spool:
            return None
        raw = (spool.get("extra") or {}).get(self.SPOOL_EXTRA_FLOW_K)
        if raw is not None and raw != "":
            try:
                return float(json.loads(raw))
            except (ValueError, TypeError):
                pass
        return None

    def write_flow_k(self, spool_id: int, k: float) -> Optional[dict]:
        """Persist a calibrated flow K to the 'flow_k' extra field.

        :param spool_id: Spoolman spool id to update.
        :param k: Flow K value to store (rounded to 6 decimals).
        :return: PATCH result dict, or None when the spool can't be read.
        """
        self._ensure_flow_k_field()
        spool = self.get_spool(spool_id)
        if not spool:
            return None
        extra = dict(spool.get("extra") or {})
        extra[self.SPOOL_EXTRA_FLOW_K] = json.dumps(round(float(k), 6))
        return self._spoolman_proxy(
            "PATCH", f"/v1/spool/{spool_id}", body={"extra": extra})

MATERIAL_DENSITY = {
    'pla':     1.24,
    'plahf':   1.24,
    'pla+':    1.24,
    'plapro':  1.24,
    'plasilk': 1.24,
    'plamatte':1.24,
    'placf':   1.30,
    'plagf':   1.30,
    'petg':    1.27,
    'pet':     1.34,
    'petgcf':  1.32,
    'petgf':   1.32,
    'tpu':     1.21,
    'tpu95':   1.21,
    'tpu85':   1.18,
    'abs':     1.04,
    'abscf':   1.13,
    'absgf':   1.13,
    'asa':     1.07,
    'asacf':   1.14,
    'pc':      1.20,
    'pccf':    1.28,
    'pa':      1.13,
    'nylon':   1.13,
    'pacf':    1.16,
    'pagf':    1.20,
    'pa6':     1.14,
    'pa6cf':   1.20,
    'pa12':    1.01,
    'peek':    1.30,
    'pps':     1.34,
    'ppscf':   1.42,
    'hips':    1.04,
    'pva':     1.23,
    'bvoh':    1.10,
}


def _norm_uid(uid: Any) -> str:
    """Normalize an RFID UID for comparison: drop separators, upper-case.
    So 'E5:CA:F0:A1', 'e5caf0a1' and 'E5CAF0A1' all compare equal.

    :param uid: Raw UID (any separators / casing).
    :return str: Separator-free uppercase UID ('' if uid is empty).
    """
    return re.sub(r'[\s:_\-]', '', str(uid or '')).strip().upper()


# Spool UID extra-field key, captured from the client class at import so this
# doesn't depend on the global SpoolmanClient name at call time.
_CARD_UIDS_FIELD = SpoolmanClient.SPOOL_EXTRA_CARD_UIDS


def _decode_extra(extra: Optional[dict], key: str) -> Any:
    """JSON-decode a Spoolman extra-field value; return None when absent.

    :param extra: A spool/filament 'extra' dict (or None).
    :param key: Extra-field key to read.
    :return: Decoded value, the raw string if not JSON, or None when absent.
    """
    raw = (extra or {}).get(key)
    if raw in (None, ""):
        return None
    try:
        return json.loads(raw)
    except (ValueError, TypeError):
        return raw


def _spool_uids(spool: dict) -> set:
    """Set of normalized NFC UIDs stored on a spool: the 'card_uids'
    comma-separated list (Snapmaker-Extended convention).

    :param spool: Spoolman spool dict.
    :return set: Normalized UID strings carried by the spool.
    """
    extra = spool.get("extra") or {}
    uids = set()
    val = _decode_extra(extra, _CARD_UIDS_FIELD)
    if val is not None:
        for part in str(val).split(","):
            n = _norm_uid(part)
            if n:
                uids.add(n)
    return uids


def find_spool_by_uid(client: SpoolmanClient, uid: Any) -> Optional[dict]:
    """Find the Spoolman spool that carries this NFC tag UID (in its 'card_uids'
    list). The UID is the primary identity of a physical spool: if it matches, it
    IS that spool. Spoolman can't query extra fields, so we scan all spools once.
    Returns the spool dict or None.

    :param client: SpoolmanClient used to list spools.
    :param uid: NFC tag UID to look for.
    :return dict: The matching spool dict, or None.
    """
    target = _norm_uid(uid)
    if not target:
        return None
    try:
        spools = client.search_spools()
    except Exception:
        return None
    for s in spools:
        if target in _spool_uids(s):
            return s
    return None


def density_for_material(material: str) -> float:
    """Return Spoolman density (g/cm^3) for a material string.
    Strips spaces, dashes, underscores so 'PLA-CF', 'pla cf', 'pla_cf'
    all match 'placf'. Falls back to PLA (1.24) for unknown materials.

    :param material: Material string (any casing/separators).
    :return float: Density in g/cm^3 (1.24 fallback for unknown materials).
    """
    if not material:
        return 1.24
    key = material.strip().lower()
    for ch in (' ', '-', '_', '/'):
        key = key.replace(ch, '')
    if key in MATERIAL_DENSITY:
        return MATERIAL_DENSITY[key]
    for k in sorted(MATERIAL_DENSITY, key=len, reverse=True):
        if key.startswith(k):
            return MATERIAL_DENSITY[k]
    return 1.24


def build_filament_name(brand: str, material: str, sub_type: str) -> str:
    """Build the product/display name from a decoded tag.

    Joins "<brand> <material> <sub_type>" (e.g. "Bambu PLA Basic",
    "Bambu PLA Matte"). Material is dropped when the sub_type already spells it
    out (avoids "Bambu PLA PLA Basic"); empty parts are skipped. Used both when
    creating a Spoolman filament and in the RFID scan notifications so every
    surface shows the same name.

    :param brand: Filament brand (may be "").
    :param material: Base material such as "PLA" (may be "").
    :param sub_type: Variant/sub-type such as "Basic" or "Matte" (may be "").
    :return str: The joined name, or "" when nothing is known.
    """
    parts = []
    if brand:
        parts.append(brand)
    if material and not (sub_type and material.lower() in sub_type.lower()):
        parts.append(material)
    if sub_type:
        parts.append(sub_type)
    return " ".join(parts).strip()


def map_tag_to_slot_info(tag: dict) -> dict:
    """Convert a host-side ``read_tag()`` result to the shared AFC slot_info.

    This is THE single mapping used by every host-side reader (ACE2, ViViD,
    OpenAMS) so all of them surface the same rich field set; the U1/OpenRFID
    webhook path builds the same shape from its JSON payload. Optional keys are
    included only when the tag carried them, so consumers can use plain
    ``.get()``.

    Keys always present: material, color_hex, multi_color, is_dual_color, sku,
    brand, sub_type, diameter, extruder_temp, bed_temp, mfg_date, uid, weight_g.
    Optional rich keys: extruder_temp_min/extruder_temp_max (the tag's real
    range — extruder_temp is their midpoint), serial, tray_uid, density,
    drying_temp, drying_time_h, color_count, color_alpha, nozzle_diameter,
    spool_width_mm, length_m, tag_type.

    :param tag: Raw dict from ``read_tag()`` (uid, sak, tag_type, filament).
    :return dict: The AFC slot_info dict.
    """
    f = (tag or {}).get("filament") or {}
    argb = f.get("color_argb")
    color_hex = f"{argb & 0xFFFFFF:06x}" if argb is not None else ""
    colors = f.get("colors_argb")
    if colors:
        multi_color = []
        for c in colors:
            if c is None:
                continue
            hx = f"{c & 0xFFFFFF:06x}"
            if hx not in multi_color:
                multi_color.append(hx)
    else:
        multi_color = [color_hex] if color_hex else []
    ext_min, ext_max = f.get("hotend_min_c"), f.get("hotend_max_c")
    if ext_min and ext_max:
        ext_temp = (int(ext_min) + int(ext_max)) // 2
    elif ext_max:
        ext_temp = int(ext_max)
    else:
        ext_temp = None
    # Brand comes from the decode ONLY — every decoder (Bambu/Snapmaker/
    # Creality/Anycubic/Elegoo/BTT) sets manufacturer. Do NOT assume "Bambu" for
    # a bare MifareClassic1k: Snapmaker and Creality are Classic too, so a
    # UID-only (undecoded) read must not be mislabelled Bambu.
    info = {
        "material": f.get("type", "") or "",
        "color_hex": color_hex,
        "multi_color": multi_color,
        "is_dual_color": len(multi_color) >= 2,
        "sku": f.get("sku", "") or "",
        "brand": f.get("manufacturer") or "",
        "sub_type": f.get("detailed", "") or "",
        "diameter": f.get("diameter_mm", 1.75) or 1.75,
        "extruder_temp": ext_temp,
        "bed_temp": f.get("bed_temp_c"),
        "mfg_date": f.get("production", "") or "",
        "uid": (tag or {}).get("uid", "") or "",
        "weight_g": f.get("weight_g"),
    }
    # Optional rich fields — only when the tag carried them.
    if ext_min:
        info["extruder_temp_min"] = int(ext_min)
    if ext_max:
        info["extruder_temp_max"] = int(ext_max)
    if argb is not None:
        alpha = (argb >> 24) & 0xFF
        if alpha != 0xFF:
            info["color_alpha"] = alpha
    for src, dst in (("serial", "serial"), ("tray_uid", "tray_uid"),
                     ("density", "density"), ("drying_temp_c", "drying_temp"),
                     ("drying_time_h", "drying_time_h"),
                     ("color_count", "color_count"),
                     ("nozzle_diameter", "nozzle_diameter"),
                     ("spool_width_mm", "spool_width_mm"),
                     ("length_m", "length_m")):
        v = f.get(src)
        if v not in (None, "", 0):
            info[dst] = v
    tag_type = (tag or {}).get("tag_type")
    if tag_type:
        info["tag_type"] = tag_type
    return info


def make_tag_record(slot_info: Optional[dict], scan_time: float,
                    decoded: bool = True, uid: str = "",
                    tag_type: str = "") -> dict:
    """Build the uniform per-slot "last read" record every reader exposes in
    ``get_status`` (``last_reads``). Failed decodes still record the UID and
    tag type so the front-end can show WHAT was seen even when the payload
    couldn't be decoded (e.g. missing decode key).

    :param slot_info: The decoded slot_info (None/{} for a failed decode).
    :param scan_time: Wall-clock time of the read (``time.time()``).
    :param decoded: Whether the tag payload was decoded.
    :param uid: UID override for a failed decode (slot_info wins when present).
    :param tag_type: Tag type override for a failed decode.
    :return dict: The status record (empty/None fields dropped).
    """
    rec = {}
    for k, v in (slot_info or {}).items():
        if v in (None, "", []):
            continue
        rec[k] = v
    if uid and not rec.get("uid"):
        rec["uid"] = uid
    if tag_type and not rec.get("tag_type"):
        rec["tag_type"] = tag_type
    rec["decoded"] = bool(decoded)
    rec["scan_time"] = round(float(scan_time), 3)
    return rec


def format_tag_summary(slot_info: dict, header: str) -> str:
    """Build the console read-out for a decoded tag (shared U1 scan format).

    A header line, then indented Name / Brand / Material / Color / Diameter /
    Density / temps (with the tag's real min–max range when carried) / Drying /
    Tag weight / Length / Remaining / SKU / Serial / Mfg date / Tag UID /
    Spoolman ID fields (each shown only when present). Colour is hex only;
    dual-colour tags join with ' + '. A pre-resolved ``display_name``
    (e.g. the matched Spoolman filament name) wins over the built name. Returns
    just the header when nothing was decoded (caller can skip an empty summary).

    :param slot_info: The decoded (optionally Spoolman-enriched) slot_info dict.
    :param header: The first, un-indented line (e.g. "ACE2 RFID: read lane1").
    :return str: The formatted multi-line summary.
    """
    brand = slot_info.get("brand", "") or ""
    material = slot_info.get("material", "") or ""
    name = slot_info.get("display_name") or build_filament_name(
        brand, material, slot_info.get("sub_type", "") or "")
    multi = [c for c in (slot_info.get("multi_color") or []) if c]
    if multi:
        color = " + ".join("#%s" % c.lstrip("#") for c in multi)
    else:
        hexv = (slot_info.get("color_hex", "") or "").lstrip("#")
        color = "#%s" % hexv if hexv else ""
    ext = slot_info.get("extruder_temp")
    ext_min = slot_info.get("extruder_temp_min")
    ext_max = slot_info.get("extruder_temp_max")
    bed = slot_info.get("bed_temp")
    diameter = slot_info.get("diameter")
    remaining = slot_info.get("remaining_weight")
    spool_id = slot_info.get("spool_id")
    lines = [header]
    if name:
        lines.append(f"  Name: {name}")
    if brand:
        lines.append(f"  Brand: {brand}")
    if material:
        lines.append(f"  Material: {material}")
    if color:
        lines.append(f"  Color: {color}")
    if diameter:
        lines.append(f"  Diameter: {diameter}mm")
    density = slot_info.get("density")
    if density:
        lines.append(f"  Density: {density}g/cm³")
    if ext:
        if ext_min and ext_max:
            lines.append(f"  Nozzle temp: {ext}°C ({ext_min}–{ext_max})")
        else:
            lines.append(f"  Nozzle temp: {ext}°C")
    if bed:
        lines.append(f"  Bed temp: {bed}°C")
    drying_temp = slot_info.get("drying_temp")
    drying_time = slot_info.get("drying_time_h")
    if drying_temp or drying_time:
        drying = f"{drying_temp}°C" if drying_temp else "?"
        if drying_time:
            drying += f" for {drying_time}h"
        lines.append(f"  Drying: {drying}")
    weight_g = slot_info.get("weight_g")
    if weight_g:
        lines.append(f"  Tag weight: {weight_g}g")
    length_m = slot_info.get("length_m")
    if length_m:
        lines.append(f"  Length: {length_m}m")
    if remaining is not None:
        lines.append(f"  Remaining: {round(float(remaining))}g")
    sku = slot_info.get("sku")
    if sku:
        lines.append(f"  SKU: {sku}")
    serial = slot_info.get("serial")
    if serial:
        lines.append(f"  Serial: {serial}")
    mfg_date = slot_info.get("mfg_date")
    if mfg_date:
        lines.append(f"  Mfg date: {mfg_date}")
    # UID only alongside decoded fields — a bare-UID read keeps the summary at
    # just the header so callers can skip it (the undecoded-hint path reports
    # the UID for those).
    uid = slot_info.get("uid")
    if uid and len(lines) > 1:
        lines.append(f"  Tag UID: {uid}")
    if spool_id:
        lines.append(f"  Spoolman ID: {spool_id}")
    return "\n".join(lines)


def prompt_hold_spool(respond: Callable[[str], None], where: str) -> None:
    """Pop a Mainsail/Fluidd dialog telling the user a tag was DETECTED and to
    hold the spool at the reader until the read completes.

    For the spool_scanner rest-scan: the tag must stay in the antenna field for
    several confirmation reads, so the moment we first see it we ask the user to
    hold steady. The result notification (or dismiss_prompt on a timeout)
    replaces this dialog. Shared so every scanner shows the same prompt.

    :param respond: Emits a raw gcode line (e.g. ``gcode.respond_raw``).
    :param where: Where the tag was detected (lane/slot label).
    """
    respond("// action:prompt_begin RFID Scan")
    respond("// action:prompt_text Tag detected on %s — hold the spool at the "
            "reader until the read completes…" % where)
    respond("// action:prompt_show")


def dismiss_prompt(respond: Callable[[str], None]) -> None:
    """Close any open Mainsail/Fluidd action:prompt dialog.

    :param respond: Emits a raw gcode line (e.g. ``gcode.respond_raw``).
    """
    respond("// action:prompt_end")


def enrich_from_spool(client: "SpoolmanClient", spool_id: Any,
                      slot_info: dict) -> dict:
    """Overlay a matched Spoolman spool's stored record onto a tag's slot_info.

    The spool record is authoritative (complete even when an at-rest read only
    got the UID), so its name/material/temps/diameter and remaining weight win;
    the tag values fill any gaps. Colour is left as the tag decoded it. Returns
    a NEW dict (input is not mutated). Best-effort — any Spoolman error just
    returns the tag values unchanged.

    :param client: A SpoolmanClient (or None to skip enrichment).
    :param spool_id: The matched Spoolman spool id (None/0 skips enrichment).
    :param slot_info: The tag's decoded slot_info.
    :return dict: slot_info overlaid with display_name / spool_id / remaining.
    """
    d = dict(slot_info)
    if not spool_id or client is None:
        return d
    try:
        sp = client.get_spool(spool_id)
    except Exception:
        sp = None
    if not isinstance(sp, dict):
        return d
    fil = sp.get("filament") or {}
    vendor = (fil.get("vendor") or {}).get("name")
    if fil.get("name"):
        d["display_name"] = fil["name"]
    if vendor:
        d["brand"] = vendor
    if fil.get("material"):
        d["material"] = fil["material"]
    if fil.get("settings_extruder_temp"):
        d["extruder_temp"] = fil["settings_extruder_temp"]
    if fil.get("settings_bed_temp"):
        d["bed_temp"] = fil["settings_bed_temp"]
    if fil.get("diameter"):
        d["diameter"] = fil["diameter"]
    rem = sp.get("remaining_weight")
    if rem is not None:
        d["remaining_weight"] = rem
    d["spool_id"] = spool_id
    return d


def rgb_array_to_hex(color_array: Any) -> str:
    """Convert an [r, g, b] array to '#rrggbb' hex string.

    :param color_array: An [r, g, b] list/tuple of ints.
    :return str: '#rrggbb' hex string ('#000000' for invalid input).
    """
    if isinstance(color_array, (list, tuple)) and len(color_array) >= 3:
        r, g, b = int(color_array[0]), int(color_array[1]), int(color_array[2])
        return f"#{r:02x}{g:02x}{b:02x}"
    return "#000000"


def log_new_filament(logger: Any, prefix: str, filament: dict, brand: str,
                     material: str, color_hex: str, diameter: Any, ext_temp: Any,
                     bed_temp: Any, sku: str = "") -> None:
    """Log a detailed breakdown when a new filament is created in Spoolman.

    :param logger: Logger for the info message.
    :param prefix: Log prefix (e.g. 'U1 RFID').
    :param filament: The created Spoolman filament dict.
    :param brand: Vendor/brand name.
    :param material: Material string.
    :param color_hex: Single colour hex (no '#').
    :param diameter: Filament diameter (mm).
    :param ext_temp: Nozzle temperature.
    :param bed_temp: Bed temperature.
    :param sku: Product SKU / article number.
    :return: None
    """
    fid = filament.get("id", "?")
    parts = [f"{prefix}: created filament #{fid} in Spoolman:"]
    if brand:
        parts.append(f"  vendor: {brand}")
    if material:
        parts.append(f"  material: {material}")
    if color_hex:
        parts.append(f"  color: #{color_hex}")
    parts.append(f"  diameter: {diameter}mm")
    if ext_temp:
        parts.append(f"  nozzle temp: {ext_temp}°C")
    if bed_temp:
        parts.append(f"  bed temp: {bed_temp}°C")
    if sku:
        parts.append(f"  SKU: {sku}")
    logger.info("\n".join(parts))


def log_new_spool(logger: Any, prefix: str, spool: dict, weight: Any,
                  spool_weight: Optional[float] = None) -> None:
    """Log a detailed breakdown when a new spool is created in Spoolman.

    :param logger: Logger for the info message.
    :param prefix: Log prefix (e.g. 'U1 RFID').
    :param spool: The created Spoolman spool dict.
    :param weight: Net filament weight (g).
    :param spool_weight: Empty spool tare weight (g), if known.
    :return: None
    """
    sid = spool.get("id", "?")
    parts = [f"{prefix}: created spool #{sid} in Spoolman:"]
    parts.append(f"  filament weight: {weight}g")
    if spool_weight:
        parts.append(f"  spool weight (tare): {spool_weight}g")
    parts.append(f"  remaining: {weight}g")
    logger.info("\n".join(parts))


def get_auto_spoolman_create(lane: Any, unit_default: bool = False) -> bool:
    """Check if auto Spoolman creation is enabled — unit then extruder fallback.

    :param lane: AFC lane whose unit/extruder are consulted.
    :param unit_default: Value returned when neither unit nor extruder opts in.
    :return bool: True if auto Spoolman create is enabled for the lane.
    """
    unit = getattr(lane, 'unit_obj', None)
    if unit is not None and getattr(unit, 'auto_spoolman_create', False):
        return True
    extruder = getattr(lane, 'extruder_obj', None)
    if extruder is not None and getattr(extruder, 'auto_spoolman_create', False):
        return True
    return unit_default


def resolve_rfid_keys(printer: Printer, bambu_master_key: Optional[bytes] = None,
                      creality_key: Optional[bytes] = None,
                      creality_encryption_key: Optional[bytes] = None) -> Tuple:
    """Fall back to a shared ``[AFC_rfid_keys]`` section for any decode key a
    host-side RFID reader (ACE2/ViViD) didn't set on its own section — so the keys
    can be configured once and used by every reader. A reader's own key always
    wins; only unset (None) keys are filled from the shared section.

    :param printer: the Klipper printer object (for looking up AFC_rfid_keys).
    :param bambu_master_key: reader's own Bambu key (bytes) or None.
    :param creality_key: reader's own Creality key (bytes) or None.
    :param creality_encryption_key: reader's own Creality enc key (bytes) or None.
    :return tuple: (bambu, creality, creality_enc) as bytes/None after fallback.
    """
    shared = printer.lookup_object("AFC_rfid_keys", None)
    if shared is None:
        return bambu_master_key, creality_key, creality_encryption_key
    return (bambu_master_key or getattr(shared, "bambu_master_key", None),
            creality_key or getattr(shared, "creality_key", None),
            creality_encryption_key
            or getattr(shared, "creality_encryption_key", None))


# Bed temperatures are frequently absent from RFID tags (Bambu tags carry none -
# Bambu Studio sets the bed temp from the plate/material preset, not the tag), so
# they read back as 0. These per-material fallbacks fill a usable bed temp when the
# tag (and Spoolman) have none. Edit to taste; unknown materials get no default.
_DEFAULT_BED_TEMPS = {
    "pla": 60, "petg": 75, "pet": 70, "abs": 95, "asa": 95,
    "tpu": 40, "pc": 100, "pa": 80, "nylon": 80, "pva": 50,
    "hips": 95, "pp": 35,
}


def default_bed_temp_for_material(material: Any) -> Optional[int]:
    """Best-effort default bed temp (C) for a material when the tag/Spoolman has
    none. Matches the longest known key the normalized material name starts with,
    so 'PLA Basic'/'PLA-CF' -> pla, 'PETG HF' -> petg. None if unknown.

    :param material: Material string (any casing/separators).
    :return int: Bed temperature in C, or None for an unknown material.
    """
    if not material:
        return None
    m = re.sub(r"[^a-z0-9]", "", str(material).lower())
    if not m:
        return None
    for key in sorted(_DEFAULT_BED_TEMPS, key=len, reverse=True):
        if m.startswith(key):
            return _DEFAULT_BED_TEMPS[key]
    return None


def apply_filament_defaults(lane: AFCLane, slot_info: dict,
                            color_converter: Optional[Callable] = None,
                            afc_defaults: Optional[dict] = None) -> None:
    """Apply RFID material/color/temps to a lane if not already set.

    :param lane: Lane to update.
    :param slot_info: Dict with material, color_hex or color, extruder_temp, bed_temp.
    :param color_converter: Optional callable to convert slot_info 'color' to hex string.
    :param afc_defaults: Optional dict with 'default_material_type', 'default_color' fallbacks.
    """
    has_material = getattr(lane, "material", None) not in (None, "")
    has_color = getattr(lane, "color", None) not in (None, "", "#000000")
    has_extruder_temp = getattr(lane, "extruder_temp", None) is not None
    has_bed_temp = getattr(lane, "bed_temp", None) is not None

    rfid_material = slot_info.get("material", "") if slot_info else ""
    rfid_extruder_temp = slot_info.get("extruder_temp") if slot_info else None
    rfid_bed_temp = slot_info.get("bed_temp") if slot_info else None
    rfid_sub_type = slot_info.get("sub_type", "") if slot_info else ""

    if rfid_material and rfid_material.lower() == "unknown":
        rfid_material = ""

    color_hex = slot_info.get("color_hex", "") if slot_info else ""
    if not color_hex and color_converter is not None:
        raw_color = slot_info.get("color", [0, 0, 0]) if slot_info else [0, 0, 0]
        if raw_color != [0, 0, 0]:
            color_hex = color_converter(raw_color)

    if not has_material and rfid_material:
        lane.material = rfid_material
    if not has_color and color_hex:
        lane.color = color_hex if color_hex.startswith("#") else f"#{color_hex}"
    if not has_extruder_temp and rfid_extruder_temp is not None:
        try:
            lane.extruder_temp = float(rfid_extruder_temp)
        except (TypeError, ValueError):
            pass
    if not has_bed_temp and rfid_bed_temp is not None:
        try:
            lane.bed_temp = float(rfid_bed_temp)
        except (TypeError, ValueError):
            pass
    # Tag carried no bed temp (e.g. Bambu tags read 0): fall back to a per-material
    # default so the lane still gets a usable bed temp.
    if getattr(lane, "bed_temp", None) is None:
        _bed_default = default_bed_temp_for_material(
            getattr(lane, "material", None) or rfid_material)
        if _bed_default is not None:
            lane.bed_temp = float(_bed_default)
    # Stash the tag's sub-type/variant on the lane (read side of the Spoolman
    # 'variant' field) so consumers like the U1 print config can use the real
    # sub-type instead of a hardcoded default.
    if rfid_sub_type:
        lane.sub_type = rfid_sub_type

    # Rich lane fields, so a lane is fully described even without a Spoolman
    # round-trip: vendor and secondary colours only when unset; density only
    # when the tag explicitly carries one (BTT tags do — authoritative for the
    # physical spool, same as the Spoolman path).
    rfid_brand = slot_info.get("brand", "") if slot_info else ""
    if rfid_brand and not getattr(lane, "spool_vendor", ""):
        lane.spool_vendor = rfid_brand
    multi = [c.lstrip("#") for c in (slot_info.get("multi_color") or []) if c] \
        if slot_info else []
    if len(multi) > 1 and not getattr(lane, "multi_color", None):
        lane.multi_color = multi
    rfid_density = slot_info.get("density") if slot_info else None
    if rfid_density:
        try:
            lane.filament_density = float(rfid_density)
        except (TypeError, ValueError):
            pass

    if afc_defaults is not None:
        if not has_material and not getattr(lane, "material", None):
            default_mat = afc_defaults.get("default_material_type")
            if default_mat:
                lane.material = default_mat
        if not has_color and not getattr(lane, "color", None):
            default_color = afc_defaults.get("default_color")
            if default_color:
                lane.color = default_color

    if not getattr(lane, "weight", 0):
        lane.weight = 1000


def _missing_filament_fields(filament: dict, slot_info: dict) -> dict:
    """Return only the Spoolman filament fields the tag can supply that are
    currently EMPTY on the matched filament — so a scan backfills missing data
    without ever overwriting what's already set.

    :param filament: the existing Spoolman filament dict.
    :param slot_info: the scanned tag's normalized info.
    :return: dict of {field: value} to PATCH (empty if nothing to fill).
    """
    updates = {}

    material = (slot_info.get("material") or "").strip()
    if material and not (filament.get("material") or "").strip():
        updates["material"] = material
        if not filament.get("density"):
            updates["density"] = density_for_material(material)

    # The tag's own density (BTT tags carry one) wins over the table value.
    tag_density = slot_info.get("density")
    if tag_density and not filament.get("density"):
        updates["density"] = tag_density

    diameter = slot_info.get("diameter")
    if diameter and not filament.get("diameter"):
        updates["diameter"] = diameter

    ext = slot_info.get("extruder_temp")
    if ext and not filament.get("settings_extruder_temp"):
        updates["settings_extruder_temp"] = ext

    bed = slot_info.get("bed_temp")
    if bed and not filament.get("settings_bed_temp"):
        updates["settings_bed_temp"] = bed

    sku = (slot_info.get("sku") or "").strip()
    if sku and not (filament.get("article_number") or "").strip():
        updates["article_number"] = sku

    # Colour: only when the filament has neither a single nor a multi colour
    # (Spoolman accepts one or the other, never both).
    has_color = (filament.get("color_hex") or "").strip() or \
        (filament.get("multi_color_hexes") or "").strip()
    colors = [c.lstrip("#").lower()
              for c in (slot_info.get("multi_color") or []) if c]
    if colors and not has_color:
        if len(colors) > 1:
            updates["multi_color_hexes"] = ",".join(colors)
            updates["multi_color_direction"] = "coaxial"
        else:
            updates["color_hex"] = colors[0]

    return updates


def sync_rfid_to_spoolman(afc: Any, lane: Any, slot_info: dict, logger: Any,
                          prefix: str, allow_create: bool = False,
                          set_next: bool = False) -> None:
    """Sync an RFID tag to Spoolman, keyed on the tag UID.

    The tag UID is unique per physical spool and is the ONLY match key: if a
    Spoolman spool already carries this UID (in its card_uids extra field), bind
    to it; otherwise it is a new spool. There is no colour/identity/SKU matching.
    When creating (allow_create), an unseen UID always creates a new filament +
    spool from the tag's own decoded values.

    :param afc: AFC main object (needs .spoolman, .moonraker, .spool).
    :param lane: Lane to assign the spool to.
    :param slot_info: Dict with uid, brand, material, color_hex, diameter, etc.
    :param logger: Logger for info/error messages.
    :param prefix: Log prefix (e.g. 'ACE RFID', 'U1 RFID').
    :param allow_create: If True, create a new filament/spool for an unseen UID.
    :param set_next: If True, stage as next_spool_id instead of assigning to lane.
    """
    if set_next:
        try:
            afc.spool.next_spool_info = dict(slot_info)
        except Exception:
            pass

    if afc.spoolman is None or afc.moonraker is None:
        return
    if not set_next and getattr(lane, "spool_id", None) not in (None, "", 0):
        return

    brand = slot_info.get("brand", "")
    material = slot_info.get("material", "")
    sub_type = slot_info.get("sub_type", "")
    color_hex = slot_info.get("color_hex", "") or None
    multi_color = slot_info.get("multi_color") or ([color_hex] if color_hex else [])
    scanned_colors = [c.lstrip("#").lower() for c in multi_color if c]
    is_multi = len(scanned_colors) > 1
    diameter = slot_info.get("diameter", 1.75)
    ext_temp = slot_info.get("extruder_temp")
    bed_temp = slot_info.get("bed_temp")
    if not bed_temp:
        # No bed temp on the tag (Bambu et al. read 0): fall back to a per-material
        # default and write it into slot_info so the created/backfilled Spoolman
        # filament persists it (and the lane picks it up via set_spoolID).
        bed_temp = default_bed_temp_for_material(material)
        if bed_temp is not None:
            slot_info["bed_temp"] = bed_temp
    default_filament_weight = 1000
    # Tag weight is a birth-certificate NET filament weight — the tag never
    # tracks usage — so it seeds initial/remaining ONLY when creating the spool
    # below (never tare, never an update to an existing spool).
    try:
        tag_weight = float(slot_info.get("weight_g") or 0)
    except (TypeError, ValueError):
        tag_weight = 0.0
    net_weight = tag_weight if tag_weight > 0 else default_filament_weight
    sku = (slot_info.get("sku") or "").strip()
    # Prefer the tag's own density (BTT tags carry one) over the material table.
    density = slot_info.get("density") or density_for_material(material)

    moonraker = SpoolmanClient(afc.moonraker)
    scanned_uid = _norm_uid(slot_info.get("uid"))

    # If Spoolman is unreachable, don't fail — the tag's decoded values were
    # already applied to the lane (apply_filament_defaults), exactly like a spool
    # with no RFID. Skip the Spoolman match/create this scan; it resolves once
    # Spoolman is back.
    if not moonraker.reachable():
        logger.info(
            f"{prefix}: Spoolman unreachable — using the tag's own values on the "
            f"lane (no Spoolman match this scan)")
        return

    try:
        # Spool identity is the tag UID — the ONLY match criterion. A UID that's
        # already stamped on a Spoolman spool IS that spool.
        spool = find_spool_by_uid(moonraker, scanned_uid) if scanned_uid else None
        filament = (spool.get("filament") or None) if spool else None
        if spool is not None:
            logger.info(f"{prefix}: matched spool #{spool.get('id')} "
                        f"by tag UID {scanned_uid}")

        # Backfill a matched (existing) filament too — the create path below only
        # backfills brand-new ones. Pushes any tag fields the filament is missing,
        # e.g. a material-default bed temp the tag itself didn't carry.
        if (spool is not None and filament is not None
                and filament.get("id") is not None):
            try:
                updates = _missing_filament_fields(filament, slot_info)
                if updates:
                    updated = moonraker.update_filament(filament["id"], updates)
                    logger.info(f"{prefix}: backfilled "
                                f"{', '.join(sorted(updates))} on filament "
                                f"#{filament['id']}")
                    if isinstance(updated, dict):
                        filament = updated
            except Exception as e:
                logger.debug(f"{prefix}: filament backfill skipped: {e}")

        if spool is None:
            # Still unmatched. The tag UID is the ONLY unique identity, so create
            # only when permitted AND the tag carried a UID (so this spool
            # re-matches on the next insert). Otherwise leave the lane on the
            # values apply_filament_defaults already set.
            if not (allow_create and scanned_uid):
                ident = f"UID {scanned_uid}" if scanned_uid else "this tag (no UID)"
                logger.info(
                    f"{prefix}: no Spoolman spool matches {ident} and "
                    f"auto-create is {'ON' if allow_create else 'OFF'}"
                    + ("" if allow_create else
                       " (set 'auto_spoolman_create: True' to create one)"))
                return
            # Refuse to create from an INCOMPLETE decode: a full tag read yields a
            # material and a colour. A partial read (e.g. the colour block failed
            # to decrypt) was still applied to the lane, but must not spawn a
            # colourless junk filament/spool in Spoolman.
            has_color = bool(color_hex) or (is_multi and bool(scanned_colors))
            if not material or not has_color:
                missing = ", ".join(
                    m for m, ok in (("material", bool(material)),
                                    ("colour", has_color)) if not ok)
                logger.info(
                    f"{prefix}: incomplete tag decode (missing {missing}) — "
                    f"applied to the lane, not creating a Spoolman entry")
                return

            # The tag UID (in the spool's card_uids extra field) is the ONLY
            # match key. There is no colour/identity matching: an unseen UID
            # always creates a new filament + spool.
            vendor_id = None
            if brand:
                vendor = moonraker.get_or_create_vendor(brand)
                if vendor:
                    vendor_id = vendor.get("id")
            # "<brand> <material> <sub_type>" e.g. "Bambu PLA Basic" — the same
            # builder feeds the scan notifications so every surface matches.
            # Always non-empty here: creation is gated above on `material`
            # being present, and build_filament_name emits the material unless
            # the sub_type already spells it out (in which case it emits that).
            filament_name = build_filament_name(brand, material, sub_type)
            filament = moonraker.create_filament(
                name=filament_name,
                vendor_id=vendor_id,
                material=material or None,
                density=density,
                diameter=diameter,
                color_hex=color_hex if not is_multi else None,
                multi_color_hexes=scanned_colors if is_multi else None,
                settings_extruder_temp=ext_temp,
                settings_bed_temp=bed_temp,
                weight=net_weight,
                article_number=sku or None,
            )
            if filament is None:
                logger.warning(
                    f"{prefix}: Spoolman create_filament FAILED for "
                    f"'{filament_name}' — check Spoolman/moonraker")
                return
            log_new_filament(logger, prefix, filament,
                             brand, material, color_hex, diameter,
                             ext_temp, bed_temp, sku)

            filament_id = (filament or {}).get("id")
            if filament_id is None:
                logger.warning(
                    f"{prefix}: resolved filament has no id — aborting")
                return

            # Backfill any fields the tag has but the filament is missing.
            try:
                updates = _missing_filament_fields(filament, slot_info)
                if updates:
                    updated = moonraker.update_filament(filament_id, updates)
                    logger.info(f"{prefix}: backfilled {', '.join(sorted(updates))} "
                                f"on filament #{filament_id}")
                    if isinstance(updated, dict):
                        filament = updated
            except Exception as e:
                logger.debug(f"{prefix}: filament backfill skipped: {e}")

            # Store the tag's sub-type as a structured 'variant' filament field.
            if sub_type:
                try:
                    moonraker.write_filament_variant(
                        filament_id, sub_type,
                        current_extra=(filament or {}).get("extra"))
                except Exception as e:
                    logger.debug(f"{prefix}: filament variant write skipped: {e}")

            # Store the tag's drying recommendation (Bambu/BTT tags carry one).
            drying_temp = slot_info.get("drying_temp")
            drying_time = slot_info.get("drying_time_h")
            if drying_temp or drying_time:
                try:
                    moonraker.write_filament_drying(
                        filament_id, drying_temp, drying_time,
                        current_extra=(filament or {}).get("extra"))
                except Exception as e:
                    logger.debug(f"{prefix}: filament drying write skipped: {e}")

            # New physical spool for this UID. The tag's net weight (when
            # carried) seeds the initial/remaining amounts — creation only.
            spool = moonraker.create_spool(
                filament_id=filament_id,
                initial_weight=net_weight,
                remaining_weight=net_weight,
            )
            if spool is None:
                logger.warning(
                    f"{prefix}: Spoolman create_spool FAILED for filament "
                    f"#{filament_id} — check Spoolman/moonraker")
                return
            log_new_spool(logger, prefix, spool, net_weight)

        spool_id = spool.get("id")

        # Stamp tag metadata: manufacturing date -> lot_nr, UID -> card_uids.
        # The UID stamp is what makes THIS spool re-match on the next read (the
        # sole match criterion), so warn loudly if it fails — otherwise the next
        # scan of this tag won't find the spool and could create a duplicate.
        try:
            mfg_date = slot_info.get("mfg_date")
            uid = slot_info.get("uid")
            if mfg_date or uid:
                moonraker.write_spool_metadata(spool_id, lot_nr=mfg_date, uid=uid)
        except Exception as e:
            logger.warning(
                f"{prefix}: stamping UID/lot on new spool #{spool_id} failed "
                f"({e}) — next scan of this tag may not re-match it")

        fil_name = (filament or {}).get("name", "")
        fil_color = ((filament or {}).get("color_hex") or "").strip().lstrip("#")
        remaining = spool.get("remaining_weight")
        remaining_str = f", {remaining:.0f}g left" if remaining else ""
        color_str = f", #{fil_color}" if fil_color else ""
        desc = f"'{fil_name}'{color_str}{remaining_str}"

        if set_next:
            afc.spool.next_spool_id = spool_id
            logger.info(
                f"{prefix}: spool #{spool_id} ({desc}) staged as next_spool_id")
        else:
            afc.spool.set_spoolID(lane, spool_id)
            lane.send_lane_data()
            logger.info(
                f"{prefix}: spool #{spool_id} ({desc}) assigned to {lane.name}")

    except Exception as e:
        lane_name = getattr(lane, 'name', '?')
        logger.error(f"{prefix} Spoolman sync failed for {lane_name}: {e}")


# ──────────────────────────────
# Shared per-unit RFID adapter base
# ──────────────────────────────
class AFCUnitRFID:
    """Mixin base for per-unit RFID adapters (ACE2, ViViD, and future units).

    The free functions above are the unit-AGNOSTIC framework: the Spoolman
    client, ``apply_filament_defaults`` (tag data -> lane), ``sync_rfid_to_spoolman``,
    ``resolve_rfid_keys`` (Bambu/Creality keys), and the color/material/density
    utilities. This mixin adds the one bit of ORCHESTRATION glue every unit
    otherwise copies verbatim -- ``apply_to_lane`` -- so a new unit only has to
    implement its own transport and tag decode.

    Adapter contract -- a subclass MUST provide:
      - ``self.afc``         : the AFC main object (or None before ready)
      - ``self.logger``      : a logger
      - ``self.log_prefix``  : str, e.g. "ACE2 RFID" -- used in log lines / Spoolman
      - ``self.auto_create`` : bool, the unit's default Spoolman auto-create
      - ``_map(self, tag) -> dict`` : decode a raw hardware read into AFC slot_info
                                      (uid, sku, brand, material, color_hex, ...)

    Typical per-unit responsibilities that stay in the subclass (they're
    hardware/protocol specific and should NOT be shared): the reader transport
    (register access, reader power), the scan/stage orchestration and motion,
    slot<->lane mapping, and any shared-antenna sister-tag handling.

    Units whose apply flow needs extra unit-specific steps interleaved (e.g. the
    U1, which also notifies its screen and re-sends lane data) can skip this
    mixin and call ``apply_filament_defaults`` / ``sync_rfid_to_spoolman``
    directly -- they're the same framework either way.
    """

    def _resolve_auto_create(self, lane: Any) -> bool:
        """Resolve whether Spoolman auto-create is allowed for this lane: the
        lane/unit/extruder setting wins, falling back to the unit default.

        :param lane: AFC lane to resolve the auto-create setting for.
        :return bool: True if Spoolman auto-create is allowed for the lane.
        """
        allow = self.auto_create
        if get_auto_spoolman_create is not None:
            try:
                allow = get_auto_spoolman_create(lane, self.auto_create)
            except Exception:
                pass
        return allow

    def record_tag_read(self, key: Any, slot_info: Optional[dict] = None,
                        decoded: bool = True, uid: str = "",
                        tag_type: str = "") -> dict:
        """Store the uniform per-slot/lane "last read" record this reader
        exposes through ``get_status`` (``last_reads``). Lazy-inits the store so
        subclasses don't need a mixin ``__init__``.

        :param key: Lane name or slot label the read belongs to.
        :param slot_info: The decoded slot_info (None for a failed decode).
        :param decoded: Whether the tag payload was decoded.
        :param uid: UID of the tag for a failed decode (slot_info wins).
        :param tag_type: Tag type for a failed decode (slot_info wins).
        :return dict: The stored record.
        """
        if not hasattr(self, "_tag_reads"):
            self._tag_reads: Dict[str, dict] = {}
        rec = make_tag_record(slot_info, time.time(), decoded=decoded,
                              uid=uid, tag_type=tag_type)
        self._tag_reads[str(key)] = rec
        return rec

    def last_reads_status(self) -> Dict[str, dict]:
        """Return the per-slot/lane last-read records for ``get_status``.

        :return dict: {lane_or_slot: record} (empty before the first read).
        """
        return dict(getattr(self, "_tag_reads", {}) or {})

    def undecoded_hint(self, key: Any, max_age: float = 10.0) -> str:
        """Describe a tag that was SEEN but not decoded on the latest read of
        ``key`` — for "no tag decoded" messages, so the user learns the UID and
        tag family instead of nothing. Empty when the latest record is decoded,
        has no UID, or is older than ``max_age`` seconds (stale).

        :param key: Lane name or slot label the read belongs to.
        :param max_age: Maximum record age (s) to still report.
        :return str: A hint like " (saw tag UID x, MifareClassic1k — no
            decoder/key matched)", or "".
        """
        rec = self.last_reads_status().get(str(key)) or {}
        if rec.get("decoded") or not rec.get("uid"):
            return ""
        if time.time() - rec.get("scan_time", 0) > max_age:
            return ""
        hint = f" (saw tag UID {rec['uid']}"
        if rec.get("tag_type"):
            hint += f", {rec['tag_type']}"
        return hint + " — no decoder/key matched)"

    def apply_to_lane(self, lane: Any, tag: dict) -> dict:
        """Map a raw read to slot_info, apply filament defaults to the lane, and
        sync to Spoolman (shared AFC_RFID path). Returns the slot_info.

        :param lane: AFC lane the tag was read for.
        :param tag: Raw hardware tag read to decode into slot_info.
        :return dict: The decoded slot_info dict.
        """
        slot_info = self._map(tag)
        apply_filament_defaults(lane, slot_info)
        allow_create = self._resolve_auto_create(lane)
        if self.afc is not None and getattr(self.afc, "spoolman", None):
            try:
                sync_rfid_to_spoolman(
                    self.afc, lane, slot_info, self.logger, self.log_prefix,
                    allow_create=allow_create)
            except Exception as e:
                self.logger.warning(
                    "%s Spoolman sync failed: %s", self.log_prefix, e)
        # Console read-out AFTER the sync so it shows the enriched, matched
        # Spoolman record (name/temps/remaining) when available. NO Mainsail
        # popup here — that's reserved for the spool_scanner path.
        self._console_read_out(lane, slot_info)
        # Record AFTER the console path so the status record carries any
        # Spoolman enrichment (display_name/spool_id) added along the way.
        self.record_tag_read(getattr(lane, "name", "?"), slot_info)
        return slot_info

    def _console_read_out(self, lane: Any, slot_info: dict) -> None:
        """Print the decoded (Spoolman-enriched, if matched) tag to the console.

        Best-effort: a UI/Spoolman hiccup never faults the read, and a bare
        UID-only decode (no fields to show) is skipped. Requires the subclass to
        expose ``self.gcode``; a subclass without it simply prints nothing.

        :param lane: AFC lane the tag was read for (source of spool_id/name).
        :param slot_info: The tag's decoded slot_info.
        """
        gcode = getattr(self, "gcode", None)
        if gcode is None:
            return
        try:
            info = slot_info
            spool_id = getattr(lane, "spool_id", None)
            mr = getattr(self.afc, "moonraker", None) if self.afc else None
            if spool_id and mr is not None:
                info = enrich_from_spool(SpoolmanClient(mr), spool_id, slot_info)
            lane_name = getattr(lane, "name", "") or ""
            where = " on %s" % lane_name if lane_name else ""
            summary = format_tag_summary(
                info, "%s: read spool%s" % (self.log_prefix, where))
            if "\n" in summary:
                gcode.respond_info(summary)
        except Exception as e:
            self.logger.debug("%s console read-out skipped: %s",
                              self.log_prefix, e)
