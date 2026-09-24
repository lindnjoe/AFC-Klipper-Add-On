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
    _get_results), none of the create/search methods our RFID needs. Rather
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
        self._tray_uid_field_ensured = False

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
    # demand), not the comment, so the comment stays free for the user. The
    # tag's manufacturing date goes to the built-in lot_nr. Extra-field values
    # are JSON-encoded per Spoolman (float -> "1.23", text -> "\"abc\"").
    # NFC tag UIDs live in a 'card_uids' spool extra field as a comma-separated
    # list of uppercase-hex UIDs, matching the Snapmaker-Extended convention so
    # spools created by either tool interoperate (a spool can carry >1 tag).
    SPOOL_EXTRA_CARD_UIDS = "card_uids"      # field_type: text
    # Flow K (auto-calibration) lives in its own 'flow_k' spool extra field, but
    # that field is created lazily by write_flow_k() ONLY when AFC_autocal
    # persists a value, so it never appears in Spoolman for users who don't run
    # auto-cal. See read_flow_k / write_flow_k below.
    SPOOL_EXTRA_FLOW_K = "flow_k"            # field_type: float (name "Flow K")
    # ══ THE ROLL'S IDENTITY, WHERE THE CARD UID IS ONLY A STICKER'S. ══
    #
    # A Bambu spool carries TWO RFID tags, one per flange, and the AMS reads
    # whichever faces its reader. Their 4-byte Mifare chip UIDs DIFFER, so the
    # same physical reel presents two identities depending on which way round
    # it went into the bay -- and card_uids, keyed on the chip UID, made two
    # Spoolman records out of one reel. Measured on 2026-09-20 by moving three
    # reels between bays: every one read a different chip UID from its other
    # face, and every one already had two records:
    #
    #   orange ABS   7392020a / c32a080a   spools 163 + 150
    #   grey Matte   d34e4e39 / 13f56d32   spools 132 + 124
    #   blue Basic   4b8e44f6 / 95f2c30c   spools 136 + 141
    #
    # The 16-byte tray_uid was IDENTICAL across both tags of each reel, and
    # different between reels. So it is the roll's identity and the chip UID is
    # the tag's, and binding on it is what stops one reel's consumption being
    # split across two records.
    #
    # BAMBU ONLY, deliberately. No other brand on this printer is known to
    # write a tray_uid, and every other reader keys off the chip UID; a field
    # that is empty for most spools must not become the match key for them.
    # Created lazily (like flow_k) so it never appears for users with no Bambu
    # tags at all.
    SPOOL_EXTRA_TRAY_UID = "tray_uid"        # field_type: text
    # The tag's filament sub-type / variant (e.g. "Matte", "Silk", "Basic") lives
    # in a 'variant' filament extra field (also matching Snapmaker-Extended).
    FILAMENT_EXTRA_VARIANT = "variant"
    # Tag drying recommendations (Bambu / BTT tags) live in their own filament
    # extra fields, created lazily ONLY when a tag actually carries drying data
    #, so they never appear in Spoolman for users whose tags don't have them.
    FILAMENT_EXTRA_DRYING_TEMP = "drying_temp_c"   # field_type: integer
    FILAMENT_EXTRA_DRYING_TIME = "drying_time_h"   # field_type: integer

    def _ensure_spool_fields(self) -> None:
        """Create the RFID 'card_uids' spool extra field if it doesn't exist.

        Idempotent and cached: one GET /v1/field/spool per client, then a POST
        only if missing. POST is create-or-update, so a re-create is harmless if
        a race adds it first. (Flow K is created separately, gated on auto-cal,
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
        persists a K, never for users who don't run auto-calibration.

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

    def _ensure_tray_uid_field(self) -> None:
        """Create the 'tray_uid' spool extra field if it doesn't exist yet.

        Lazy and cached, the same shape as _ensure_flow_k_field: called only
        from write_tray_uid, so the field appears the first time a Bambu tag is
        actually bound and never for a printer with no Bambu spools on it.

        :return: None
        """
        if self._tray_uid_field_ensured:
            return
        existing = self._spoolman_proxy("GET", "/v1/field/spool",
                                        print_error=False)
        keys = set()
        if isinstance(existing, list):
            keys = {f.get("key") for f in existing if isinstance(f, dict)}
        if self.SPOOL_EXTRA_TRAY_UID not in keys:
            self._spoolman_proxy(
                "POST", f"/v1/field/spool/{self.SPOOL_EXTRA_TRAY_UID}",
                body={"name": "Tray UID", "field_type": "text"})
        self._tray_uid_field_ensured = True

    def write_tray_uid(self, spool_id: int,
                       tray_uid: Optional[str]) -> Optional[dict]:
        """Record the roll identity (Bambu's 16-byte tray UID) on a spool.

        Written, never merged: unlike card_uids this is ONE value per reel, so
        a second value arriving for the same spool means the record describes
        two different rolls and overwriting it would hide that. The caller
        checks for a conflict before it gets here.

        :param spool_id: Spoolman spool id to update.
        :param tray_uid: the tag's tray UID, hex, case-insensitive.
        :return: PATCH result dict, or None when there is nothing to write.
        """
        norm = _norm_tray_uid(tray_uid)
        if not norm:
            return None
        self._ensure_tray_uid_field()
        return self._patch_spool(
            spool_id,
            extra_updates={self.SPOOL_EXTRA_TRAY_UID: json.dumps(norm)})

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

    def set_remaining_weight(self, spool_id: int,
                             grams: float) -> Optional[dict]:
        """PATCH a spool's remaining net filament weight, in grams.

        Spoolman derives remaining_weight from used_weight, so it is set by
        writing used_weight = initial_weight - remaining. initial_weight comes
        from the spool (or its filament's weight) so the arithmetic matches
        Spoolman's own. Used for a PHYSICAL remaining measurement (e.g. an AMS
        that measured the spool by radius), which is authoritative in a way
        extrusion-accounting is not.

        :param spool_id: Spoolman spool id.
        :param grams: measured remaining net weight, grams.
        :return: PATCH result dict, or None.
        """
        if spool_id in (None, "", 0) or grams is None or grams < 0:
            return None
        spool = self.get_spool(spool_id) or {}
        initial = spool.get("initial_weight")
        if not initial:
            fil = spool.get("filament") or {}
            initial = fil.get("weight")
        try:
            initial = float(initial) if initial else None
        except (TypeError, ValueError):
            initial = None
        if not initial:
            return None
        used = max(0.0, initial - float(grams))
        return self._spoolman_proxy(
            "PATCH", f"/v1/spool/{spool_id}",
            body=json.dumps({"used_weight": round(used, 2)}))

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
        """Read a spool dict from Spoolman, through moonraker's proxy.

        This used to delegate to AFC_moonraker.get_spool. Upstream turned that
        method into a fire-and-forget queue -- ``get_spool(id, callback)``,
        returning None -- so the old call raised TypeError at the point of use
        and every caller here swallowed it as "no spool". It surfaced as UID
        stamping failing after a tag read.

        Going straight to the proxy keeps this synchronous, which is what its
        callers are: they run on the RFID worker threads and use the result on
        the next line. The reactor is not involved either way.

        :param spool_id: Spoolman spool id to fetch.
        :return dict: The spool dict, or None if not found.
        """
        result = self._spoolman_proxy("GET", f"/v1/spool/{int(spool_id)}")
        return result if isinstance(result, dict) else None

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
    # A matte is PLA plus a mattifying filler heavier than the polymer, so it
    # almost certainly does NOT weigh what plain PLA weighs -- but 1.24 stands
    # until someone measures it properly. A 1.32 taken from one reel was tried
    # here on 2026-09-20 and withdrawn: it came from a geometry fit whose
    # constants are not the ones this table is used with, and the reading it
    # rested on could not be attributed to that reel with confidence (the same
    # bay produces the same radius for two different reels, and the reel had
    # only ever been measured in two bays). A wrong density is worse than a
    # generic one -- it is wrong on every printer, not just the one it was
    # measured on. Spoolman's per-filament density overrides this for anyone
    # who has the vendor's figure.
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

# ══ THE BED TEMPERATURE THE TAG USUALLY DOES NOT CARRY. ══
#
# A Bambu tag HAS a bed-temperature field and nearly always writes zero into
# it. Mined out of the whole capture corpus plus two live reads -- 14 distinct
# tag records -- exactly three state one, all of them the pair (type 1, 35 C),
# which is the Cool Plate figure. The other eleven, including a PLA Basic and
# an ABS read off the wire on 2026-09-21, write 0 for both.
#
# That is worth being precise about, because it looked for a while like a
# decode bug on this side. It is not. In the same frame the neighbouring
# fields read correctly on every record: drying temp tracks the material (55 C
# on PLA, 80 C on ABS -- drying figures, not bed figures), drying time reads
# 8 h, and the hotend pair reads 230/190 on PLA and 270/240 on ABS. Sweeping
# both live frames for any unclaimed uint16 in the 40-120 range turns up only
# bytes already accounted for -- inside the SKU and name strings, or the
# drying temp itself. The number is absent from the tag, not missed by us.
#
# So it is derived from the material instead, the same shape as
# MATERIAL_DENSITY above and with the same rule: the tag wins when it states
# one, Spoolman wins over both, and a derived value is LABELLED as derived
# (slot_info's "bed_temp_source") so nothing downstream mistakes a table
# lookup for a reading.
#
# Figures are Bambu Studio's own plate defaults for the textured PEI plate,
# which is what these spools are sold against. They are a sane starting bed,
# not a vendor measurement -- unlike a wrong density, which silently corrupts
# every mass estimate, a bed temp that is 5 C out is visible and harmless, so
# a generic figure earns its place here where it did not there.
MATERIAL_BED_TEMP = {
    'pla':     55,
    'plahf':   55,
    'pla+':    55,
    'plapro':  55,
    'plasilk': 55,
    'plamatte':55,
    'placf':   55,
    'plagf':   55,
    'petg':    70,
    'pet':     70,
    'petgcf':  70,
    'petgf':   70,
    'tpu':     35,
    'tpu95':   35,
    'tpu85':   35,
    'abs':     90,
    'abscf':   90,
    'absgf':   90,
    'asa':     90,
    'asacf':   90,
    'pc':     100,
    'pccf':   100,
    'pa':     100,
    'nylon':  100,
    'pacf':   100,
    'pagf':   100,
    'pa6':    100,
    'pa6cf':  100,
    'pa12':   100,
    'pps':    100,
    'ppscf':  100,
    'hips':   100,
    'pva':     45,
    'bvoh':    45,
    # No PEEK row on purpose. It is not a bed this hardware reaches and a
    # guess at 120 C+ would be a number nobody measured, offered to someone
    # who cannot check it. An unknown material returns None, and None is an
    # honest answer.
}


def bed_temp_for_material(material: str) -> Optional[int]:
    """Return a default bed temperature (C) for a material string, or None.

    Matches MATERIAL_DENSITY's normalisation exactly -- separators stripped,
    then longest-prefix -- so 'PETG-CF', 'petg cf' and 'petg_cf' all land on
    the same row and the two tables can never disagree about what a material
    string means.

    Unlike density_for_material this does NOT fall back to a generic value.
    Density has a defensible default (most filament is close to PLA); a bed
    temperature does not -- putting 55 C under an unknown engineering polymer
    would be worse than saying nothing.

    :param material: Material string (any casing/separators).
    :return Optional[int]: Bed temperature in C, or None when unknown.
    """
    if not material:
        return None
    key = material.strip().lower()
    for ch in (' ', '-', '_', '/'):
        key = key.replace(ch, '')
    if key in MATERIAL_BED_TEMP:
        return MATERIAL_BED_TEMP[key]
    for k in sorted(MATERIAL_BED_TEMP, key=len, reverse=True):
        if key.startswith(k):
            return MATERIAL_BED_TEMP[k]
    return None


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
_TRAY_UID_FIELD = SpoolmanClient.SPOOL_EXTRA_TRAY_UID


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


def _norm_tray_uid(tray_uid: Any) -> str:
    """Normalize a Bambu tray UID for comparison: lowercase hex, no spaces.

    Lowercase, where _norm_uid upper-cases card UIDs, because that is how the
    bridge reports it and how it reads in a log next to the chip UID. What
    matters is that ONE convention is used on both sides of every comparison.

    :param tray_uid: raw tray UID from a tag read or a spool record.
    :return str: normalized tray UID, or "" if unusable.
    """
    if not tray_uid:
        return ""
    s = "".join(str(tray_uid).split()).replace(":", "").replace("-", "")
    s = s.strip().lower()
    # All-zero is what an unread field looks like, not a roll.
    if not s or not all(c in "0123456789abcdef" for c in s) or not s.strip("0"):
        return ""
    return s


def _spool_tray_uid(spool: dict) -> str:
    """The roll identity stored on a spool, normalized, or "".

    :param spool: Spoolman spool dict.
    :return str: normalized tray UID, or "" when the spool carries none.
    """
    return _norm_tray_uid(_decode_extra(spool.get("extra") or {},
                                        _TRAY_UID_FIELD))


def find_spool_by_tray_uid(client: SpoolmanClient,
                           tray_uid: Any) -> Optional[dict]:
    """Find the Spoolman spool carrying this ROLL identity (Bambu tray UID).

    The counterpart to find_spool_by_uid, and the stronger of the two for a
    Bambu reel: the chip UID identifies the tag that happened to face the
    reader, the tray UID identifies the roll both tags are stuck to. Same
    full-table scan, because Spoolman cannot query extra fields.

    Returns None on an ambiguous answer -- two spools carrying one tray UID is
    the duplicate-record state this exists to end, and picking one of them
    silently would bind consumption to a coin flip. The caller reports it.

    :param client: SpoolmanClient used to list spools.
    :param tray_uid: Bambu tray UID to look for.
    :return dict: the matching spool dict, or None.
    """
    target = _norm_tray_uid(tray_uid)
    if not target:
        return None
    try:
        spools = client.search_spools()
    except Exception:
        return None
    hits = [s for s in spools if _spool_tray_uid(s) == target]
    return hits[0] if len(hits) == 1 else None


def match_spool_for_tag(client: SpoolmanClient, uid: Any,
                        tray_uid: Any = "") -> tuple:
    """Find the Spoolman spool a tag read belongs to: the ROLL, then the tag.

    The one place the order is decided, so every reader answers the same way:
    the host-side readers through _spoolman_resolve, the Bambu AMS through its
    own match-only bind. A tag that carries no roll id behaves exactly as it
    always has -- this is gated on the TAG, never on a brand.

    ══ AND IT NAMES THE DUPLICATE IT WALKED PAST. ══

    A reel recorded twice does not announce itself: the roll lookup finds the
    record that carries the roll id, binds to it, and the other record just
    sits there collecting nothing. That is benign and invisible, which is how
    three reels here came to be duplicated for months. So when the chip UID
    ALSO belongs to a different spool than the one matched, that spool's id
    comes back with the match and the caller says so -- at the moment the reel
    is in a bay and the operator can act on it, rather than in an audit
    command nobody runs.

    ONE listing serves both lookups. They used to be a full-table scan each,
    on every tag read.

    :param client: SpoolmanClient used for the lookups.
    :param uid: the 4-byte chip UID read from the tag.
    :param tray_uid: the roll identity, when the tag carries one.
    :return tuple: (spool or None, True if matched by the roll, the id of a
        DIFFERENT spool that is the same physical reel, or None)
    """
    tray = _norm_tray_uid(tray_uid)
    uid_n = _norm_uid(uid)
    try:
        spools = client.search_spools()
    except Exception:
        return None, False, None
    by_tray = [s for s in spools if tray and _spool_tray_uid(s) == tray]
    by_uid = [s for s in spools if uid_n and uid_n in _spool_uids(s)]
    if len(by_tray) == 1:
        spool = by_tray[0]
        # The same chip UID on another record is that record describing this
        # very reel -- the duplicate, named.
        dupe = next((s.get("id") for s in by_uid
                     if s.get("id") != spool.get("id")), None)
        return spool, True, dupe
    if len(by_tray) > 1:
        # Two records already carrying one roll id. Nothing here can choose
        # between them without binding the consumption to a coin flip, so the
        # chip UID decides and both ids are worth reporting; the first that is
        # not the one we land on is enough to point at the pair.
        spool = by_uid[0] if by_uid else None
        dupe = next((s.get("id") for s in by_tray
                     if spool is None or s.get("id") != spool.get("id")), None)
        return spool, False, dupe
    return (by_uid[0] if by_uid else None), False, None


def _cached_spoolman_client(afc: Any) -> "SpoolmanClient":
    """One SpoolmanClient per AFC instance, not per call.

    The client's _fields_ensured/_filament_fields_ensured memos are
    per-instance; constructing a fresh client for every sync threw them away,
    so every bind re-fetched the Spoolman field schema -- more blocking HTTP
    on the reactor in exactly the burst that has taken the printer down
    ("Timer too close"). Cached on the afc object so all units share it.
    """
    client = getattr(afc, "_afc_spoolman_client_cache", None)
    if client is None:
        client = SpoolmanClient(afc.moonraker)
        try:
            afc._afc_spoolman_client_cache = client
        except Exception:
            pass
    return client


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
    range, extruder_temp is their midpoint), serial, tray_uid, density,
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
    # Brand comes from the decode ONLY, every decoder (Bambu/Snapmaker/
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
    # Optional rich fields, only when the tag carried them.
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
        color = " + ".join(f'#{c.lstrip("#")}' for c in multi)
    else:
        hexv = (slot_info.get("color_hex", "") or "").lstrip("#")
        color = f"#{hexv}" if hexv else ""
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
    # UID only alongside decoded fields, a bare-UID read keeps the summary at
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
    respond(f"// action:prompt_text Tag detected on {where}, hold the spool at the reader until "
        f"the read completes…")
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
    a NEW dict (input is not mutated). Best-effort, any Spoolman error just
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
    """Check if auto Spoolman creation is enabled, unit then extruder fallback.

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
    host-side RFID reader (ACE2/ViViD) didn't set on its own section, so the keys
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
    # when the tag explicitly carries one (BTT tags do, authoritative for the
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
    currently EMPTY on the matched filament, so a scan backfills missing data
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


class _DeferredLog:
    """A logger-shaped sink that RECORDS instead of emitting.

    AFC's logger does more than write a file: info() and warning() call
    send_callback, which answers the g-code console, and warning() appends to
    afc.message_queue. None of that may happen from a worker thread. So the
    blocking half below is handed one of these -- it looks exactly like the
    logger to every line inside, including the log_new_filament/log_new_spool
    helpers that take one as an argument -- and the reactor replays it after.

    Same reason AFC_moonraker has _log_async.
    """

    #: AFC's signatures, spelled out rather than **kw. A sink that swallowed
    #: keywords would drop console_only and traceback silently -- the same
    #: quiet-difference trap that makes printf-style args vanish into
    #: console_only.
    def __init__(self) -> None:
        """
        Buffer log calls until a logger is available.
        """
        self.entries: list = []

    def info(self, message: Any, console_only: bool = False) -> None:
        """
        Record an info entry for replay.

        :param message: log message
        :param console_only: AFC logger flag
        """
        self.entries.append(("info", message, {"console_only": console_only}))

    def warning(self, message: Any) -> None:
        """
        Record a warning entry for replay.

        :param message: log message
        """
        self.entries.append(("warning", message, {}))

    def debug(self, message: Any, only_debug: bool = False,
              traceback: Any = None) -> None:
        """
        Record a debug entry for replay.

        :param message: log message
        :param only_debug: AFC logger flag
        :param traceback: AFC logger traceback
        """
        self.entries.append(("debug", message,
                             {"only_debug": only_debug, "traceback": traceback}))

    def error(self, message: Any, traceback: Any = None,
              stack_name: str = "") -> None:
        """
        Record an error entry for replay.

        :param message: log message
        :param traceback: AFC logger traceback
        :param stack_name: AFC logger flag
        """
        self.entries.append(("error", message,
                             {"traceback": traceback, "stack_name": stack_name}))

    def replay(self, logger: Any) -> None:
        """Emit everything recorded, on the reactor.

        Retries without the keywords if the target rejects them: this also
        replays onto a plain stdlib logger, which has none of AFC's, and a
        dropped diagnostic is worse than a plainer one.

        :param logger: the real AFC logger (or a stdlib one)
        """
        for level, msg, kw in self.entries:
            fn = getattr(logger, level, None) or logger.info
            try:
                fn(msg, **kw)
            except TypeError:
                try:
                    fn(msg)
                except Exception:
                    pass
            except Exception:
                pass
        self.entries = []


def _spoolman_prepare(afc: Any, lane: Any, slot_info: dict,
                      set_next: bool) -> Optional[dict]:
    """Everything that reads or writes printer state -- so, the reactor.

    Also derives the scalars the HTTP half needs, because one of them
    (bed_temp) is written back into slot_info, and mutating the caller's dict
    from a worker thread would be a race for the sake of nothing.

    :param afc: AFC main object
    :param lane: the lane being assigned
    :param slot_info: decoded tag values
    :param set_next: stage as next spool rather than assigning the lane
    :return dict: values for _spoolman_resolve, or None to stop here
    """
    if set_next:
        try:
            afc.spool.next_spool_info = dict(slot_info)
        except Exception:
            pass

    if afc.spoolman is None or afc.moonraker is None:
        return None
    if not set_next and getattr(lane, "spool_id", None) not in (None, "", 0):
        return None

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
    # Tag weight is a birth-certificate NET filament weight, the tag never
    # tracks usage, so it seeds initial/remaining ONLY when creating the spool
    # below (never tare, never an update to an existing spool).
    try:
        tag_weight = float(slot_info.get("weight_g") or 0)
    except (TypeError, ValueError):
        tag_weight = 0.0
    net_weight = tag_weight if tag_weight > 0 else default_filament_weight
    sku = (slot_info.get("sku") or "").strip()
    # Prefer the tag's own density (BTT tags carry one) over the material table.
    density = slot_info.get("density") or density_for_material(material)

    return {
        "brand": brand, "material": material, "sub_type": sub_type,
        "color_hex": color_hex, "multi_color": multi_color,
        "scanned_colors": scanned_colors, "is_multi": is_multi,
        "diameter": diameter, "ext_temp": ext_temp, "bed_temp": bed_temp,
        "net_weight": net_weight, "sku": sku, "density": density,
    }


def _spoolman_resolve(afc: Any, prep: dict, slot_info: dict, logger: Any,
                      prefix: str, allow_create: bool) -> Optional[dict]:
    """THE BLOCKING HALF: every Spoolman round trip lives here.

    Runs on AFC_moonraker's background writer thread when the caller passes a
    reactor, because SpoolmanClient goes through moonraker's SYNCHRONOUS
    _get_results -- urlopen with REQUEST_TIMEOUT = 10 -- and doing that on the
    reactor is up to ten seconds of a printer that cannot answer its MCUs.
    _cached_spoolman_client's own docstring records where that ends: "Timer
    too close".

    Touches no lane and no afc.spool: it returns what it found and the caller
    applies it. ``logger`` here is a _DeferredLog when off-reactor.

    :param afc: AFC main object (for the cached client only)
    :param prep: the dict from _spoolman_prepare
    :param slot_info: decoded tag values (read only here)
    :param logger: real logger inline, _DeferredLog off-reactor
    :param prefix: log prefix, e.g. "U1 RFID"
    :param allow_create: create an unseen UID's filament + spool
    :return dict: {"spool_id": id, "desc": text}, or None if nothing to apply
    """
    brand = prep["brand"]
    material = prep["material"]
    sub_type = prep["sub_type"]
    color_hex = prep["color_hex"]
    scanned_colors = prep["scanned_colors"]
    is_multi = prep["is_multi"]
    diameter = prep["diameter"]
    ext_temp = prep["ext_temp"]
    bed_temp = prep["bed_temp"]
    net_weight = prep["net_weight"]
    sku = prep["sku"]
    density = prep["density"]

    moonraker = _cached_spoolman_client(afc)
    scanned_uid = _norm_uid(slot_info.get("uid"))

    # If Spoolman is unreachable, don't fail, the tag's decoded values were
    # already applied to the lane (apply_filament_defaults), exactly like a spool
    # with no RFID. Skip the Spoolman match/create this scan; it resolves once
    # Spoolman is back.
    if not moonraker.reachable():
        logger.info(
            f"{prefix}: Spoolman unreachable, using the tag's own values on the "
            f"lane (no Spoolman match this scan)")
        return

    try:
        # ══ THE ROLL FIRST, WHEN THE TAG CARRIES ONE. ══
        #
        # The chip UID identifies a TAG. Some spools carry more than one: a
        # Bambu reel has a tag on each flange, with DIFFERENT chip UIDs and the
        # SAME 16-byte tray_uid, so which identity a reader gets depends on
        # which way round the reel went in. Keyed on the chip UID alone, one
        # reel becomes two Spoolman records with its consumption split between
        # them -- measured here on 2026-09-20, three reels, every one already
        # duplicated (163+150, 132+124, 136+141).
        #
        # Gated on the TAG, not on a brand: a decode that produced no tray_uid
        # behaves exactly as before, which is every non-Bambu tag today. The
        # readers already carry it -- map_tag_to_slot_info has passed tray_uid
        # through all along -- so this reaches every one of them at once.
        scanned_tray = _norm_tray_uid(slot_info.get("tray_uid"))
        spool, by_tray, dupe_id = match_spool_for_tag(
            moonraker, scanned_uid, scanned_tray)
        if dupe_id is not None and spool is not None:
            # SAID WHERE IT IS ACTIONABLE. The reel is in a bay, in front of
            # someone, and Spoolman has it twice -- so its consumption has been
            # splitting between the two records by which way round it went in.
            # Not merged from here: merging discards one record's history, and
            # which to keep depends on which has actually been printed from.
            logger.warning(
                f"{prefix}: spool #{dupe_id} is the SAME PHYSICAL REEL as "
                f"#{spool.get('id')} -- a spool with a tag on each side, "
                f"recorded twice. Merge them in Spoolman (keep one, add the "
                f"other's card_uids to it, add the two used weights together) "
                f"or its filament count stays split between them.")
        # Set once the roll id has been dealt with on an EXISTING record --
        # written, or deliberately not written because the record names a
        # different roll. The create path below checks it so a conflict that
        # was refused here cannot be overwritten there.
        tray_settled = False
        filament = (spool.get("filament") or None) if spool else None
        if spool is not None:
            logger.info(
                f"{prefix}: matched spool #{spool.get('id')} by "
                + (f"tray UID {scanned_tray}" if by_tray
                   else f"tag UID {scanned_uid}"))
            # TEACH THE RECORD THE HALF IT DOES NOT HAVE. Matched by the roll,
            # this chip UID is the reel's other face and belongs in card_uids
            # (stamped below, which unions). Matched by the chip, stamp the
            # roll on so the OTHER face is recognised next time -- but never
            # over a DIFFERENT roll already recorded: that record describes
            # another reel, and overwriting buries it instead of surfacing it.
            if scanned_tray and not by_tray:
                tray_settled = True
                try:
                    have = _spool_tray_uid(spool)
                    if not have:
                        moonraker.write_tray_uid(spool.get("id"), scanned_tray)
                    elif have != scanned_tray:
                        logger.warning(
                            f"{prefix}: spool #{spool.get('id')} is recorded "
                            f"against roll {have} but this tag says "
                            f"{scanned_tray} -- left alone; one of them is on "
                            f"the wrong spool")
                except Exception as e:
                    logger.warning(
                        f"{prefix}: recording the roll id on spool "
                        f"#{spool.get('id')} failed ({e}); the tag on the "
                        f"other side of this spool will not match it yet")

        # Backfill a matched (existing) filament too, the create path below only
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
                    f"{prefix}: incomplete tag decode (missing {missing}), "
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
            # "<brand> <material> <sub_type>" e.g. "Bambu PLA Basic", the same
            # builder feeds the scan notifications so every surface matches.
            filament_name = build_filament_name(brand, material, sub_type)
            if not filament_name:
                filament_name = material or "Unknown"
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
                    f"'{filament_name}', check Spoolman/moonraker")
                return
            log_new_filament(logger, prefix, filament,
                             brand, material, color_hex, diameter,
                             ext_temp, bed_temp, sku)

            filament_id = (filament or {}).get("id")
            if filament_id is None:
                logger.warning(
                    f"{prefix}: resolved filament has no id, aborting")
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
            # carried) seeds the initial/remaining amounts, creation only.
            spool = moonraker.create_spool(
                filament_id=filament_id,
                initial_weight=net_weight,
                remaining_weight=net_weight,
            )
            if spool is None:
                logger.warning(
                    f"{prefix}: Spoolman create_spool FAILED for filament "
                    f"#{filament_id}, check Spoolman/moonraker")
                return
            log_new_spool(logger, prefix, spool, net_weight)

        spool_id = spool.get("id")

        # Stamp tag metadata: manufacturing date -> lot_nr, UID -> card_uids.
        # The UID stamp is what makes THIS spool re-match on the next read (the
        # sole match criterion), so warn loudly if it fails, otherwise the next
        # scan of this tag won't find the spool and could create a duplicate.
        try:
            mfg_date = slot_info.get("mfg_date")
            uid = slot_info.get("uid")
            if mfg_date or uid:
                moonraker.write_spool_metadata(spool_id, lot_nr=mfg_date, uid=uid)
            # The roll id on a spool this scan CREATED. Without it a brand new
            # reel is duplicated the first time it goes in the other way round
            # -- the exact hole that put three reels in twice here. Skipped
            # when the match above already settled it, so a refused conflict
            # is not overwritten from here.
            if scanned_tray and not by_tray and not tray_settled:
                moonraker.write_tray_uid(spool_id, scanned_tray)
        except Exception as e:
            logger.warning(
                f"{prefix}: stamping UID/lot on new spool #{spool_id} failed "
                f"({e}), next scan of this tag may not re-match it")

        fil_name = (filament or {}).get("name", "")
        fil_color = ((filament or {}).get("color_hex") or "").strip().lstrip("#")
        remaining = spool.get("remaining_weight")
        remaining_str = f", {remaining:.0f}g left" if remaining else ""
        color_str = f", #{fil_color}" if fil_color else ""
        desc = f"'{fil_name}'{color_str}{remaining_str}"

        return {"spool_id": spool_id, "desc": desc}

    except Exception as e:
        logger.error(f"{prefix} Spoolman sync failed: {e}")
        return None


def _spoolman_apply(afc: Any, lane: Any, logger: Any, prefix: str,
                    resolved: Optional[dict], set_next: bool,
                    done: Any = None) -> None:
    """Put the answer on the lane. Reactor only -- this mutates printer state.

    ``done`` FIRES WHEN THE LANE ACTUALLY CARRIES THE SPOOL, not when
    set_spoolID returns. set_spoolID is itself asynchronous: it asks moonraker
    for the spool and applies material/colour/temps/weight to the lane when
    that answer arrives. Chaining onto ITS on_done is what lets a caller read
    those fields instead of fetching the same record a third time.

    Falls back to firing immediately on an AFC whose set_spoolID predates
    on_done -- the lane data lands a moment later there, which is what that
    version always did.

    :param afc: AFC main object
    :param lane: the lane being assigned
    :param logger: the AFC logger
    :param prefix: log prefix
    :param resolved: the dict from _spoolman_resolve, or None
    :param set_next: stage as next spool rather than assigning the lane
    :param done: called once, after the lane is populated where possible
    """
    def _done() -> None:
        """
        Run the caller's done callback, swallowing errors.
        """
        if done is not None:
            try:
                done()
            except Exception:
                pass

    if not resolved:
        _done()
        return
    spool_id = resolved["spool_id"]
    desc = resolved["desc"]
    try:
        if set_next:
            afc.spool.next_spool_id = spool_id
            logger.info(
                f"{prefix}: spool #{spool_id} ({desc}) staged as next_spool_id")
            _done()
            return

        def _applied() -> None:
            """
            Push lane data and log the assignment once applied.
            """
            lane.send_lane_data()
            logger.info(
                f"{prefix}: spool #{spool_id} ({desc}) assigned to {lane.name}")
            _done()

        if _set_spoolid_takes_on_done(afc):
            afc.spool.set_spoolID(lane, spool_id, on_done=_applied)
        else:
            afc.spool.set_spoolID(lane, spool_id)
            _applied()
    except Exception as e:
        lane_name = getattr(lane, 'name', '?')
        logger.error(f"{prefix} Spoolman apply failed for {lane_name}: {e}")
        _done()


def _set_spoolid_takes_on_done(afc: Any) -> bool:
    """Whether this AFC's set_spoolID has the on_done completion callback.

    Asked of the real signature rather than assumed, because this module also
    ships against older AFC. Memoised on the afc object: inspect.signature is
    not free and this is on the scan path.

    :param afc: AFC main object
    :return bool: True when on_done can be passed
    """
    cached = getattr(afc, "_afc_setspoolid_on_done", None)
    if cached is not None:
        return cached
    ok = False
    try:
        import inspect
        ok = "on_done" in inspect.signature(afc.spool.set_spoolID).parameters
    except Exception:
        ok = False
    try:
        afc._afc_setspoolid_on_done = ok
    except Exception:
        pass
    return ok

def sync_rfid_to_spoolman(afc: Any, lane: Any, slot_info: dict, logger: Any,
                          prefix: str, allow_create: bool = False,
                          set_next: bool = False, reactor: Any = None,
                          on_done: Any = None) -> None:
    """Sync an RFID tag to Spoolman, keyed on the tag UID.

    The tag UID is unique per physical spool and is the ONLY match key: if a
    Spoolman spool already carries this UID (in its card_uids extra field), bind
    to it; otherwise it is a new spool. There is no colour/identity/SKU matching.
    When creating (allow_create), an unseen UID always creates a new filament +
    spool from the tag's own decoded values.

    PASS ``reactor`` FROM ANY CALLER THAT IS ON ONE. The Spoolman round trips
    go through moonraker's synchronous path -- urlopen with
    REQUEST_TIMEOUT = 10 -- so calling this from a reactor timer or a
    push-callback can stall the printer for ten seconds when Spoolman is slow
    or gone. With a reactor, the HTTP runs on AFC_moonraker's EXISTING
    background writer thread and the lane assignment comes back through
    register_async_callback: the shape upstream uses for get_spool, reusing the
    one thread rather than starting another.

    Without it everything runs inline, exactly as before -- right for a caller
    that is already off the reactor, and it keeps this callable from a test.

    :param afc: AFC main object (needs .spoolman, .moonraker, .spool).
    :param lane: Lane to assign the spool to.
    :param slot_info: Dict with uid, brand, material, color_hex, diameter, etc.
    :param logger: Logger for info/error messages.
    :param prefix: Log prefix (e.g. 'ACE RFID', 'U1 RFID').
    :param allow_create: If True, create a new filament/spool for an unseen UID.
    :param set_next: If True, stage as next_spool_id instead of assigning to lane.
    :param reactor: Klipper reactor. Given, the HTTP goes off the reactor.
    :param on_done: Called on the reactor once the lane has actually been
      assigned -- the hook a caller needs when its next step reads
      lane.spool_id, which is no longer set by the time this returns. Runs on
      every path, including the ones that give up early, so a caller cannot be
      left waiting for it. Same shape as AFC_spool.set_spoolID's on_done.
    """
    def _finish() -> None:
        """
        Run the caller's on_done callback, swallowing errors.
        """
        if on_done is not None:
            try:
                on_done()
            except Exception:
                pass

    prep = _spoolman_prepare(afc, lane, slot_info, set_next)
    if prep is None:
        _finish()
        return

    queue = getattr(getattr(afc, "moonraker", None), "_write_queue", None)
    if reactor is None or queue is None:
        resolved = _spoolman_resolve(afc, prep, slot_info, logger, prefix,
                                     allow_create)
        _spoolman_apply(afc, lane, logger, prefix, resolved, set_next,
                        done=_finish)
        return

    deferred = _DeferredLog()

    def _off_reactor() -> None:
        """Runs on moonraker's writer thread."""
        try:
            resolved = _spoolman_resolve(afc, prep, slot_info, deferred,
                                         prefix, allow_create)
        except Exception as e:                       # never kill that thread
            resolved = None
            deferred.error(f"{prefix} Spoolman sync failed: {e}")

        def _back(_eventtime: float) -> None:
            """
            Reactor callback: replay deferred logs and apply the Spoolman result.

            :param _eventtime: reactor event time
            """
            deferred.replay(logger)
            _spoolman_apply(afc, lane, logger, prefix, resolved, set_next,
                            done=_finish)

        reactor.register_async_callback(_back)

    queue.put_nowait((_off_reactor, ()))



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
        ``key``, for "no tag decoded" messages, so the user learns the UID and
        tag family instead of nothing. Empty when the latest record is decoded,
        has no UID, or is older than ``max_age`` seconds (stale).

        :param key: Lane name or slot label the read belongs to.
        :param max_age: Maximum record age (s) to still report.
        :return str: A hint like " (saw tag UID x, MifareClassic1k, no
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
        return hint + ", no decoder/key matched)"

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
                    f"{self.log_prefix} Spoolman sync failed: {e}")
        # Console read-out AFTER the sync so it shows the enriched, matched
        # Spoolman record (name/temps/remaining) when available.
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
                info = enrich_from_spool(_cached_spoolman_client(self.afc),
                                         spool_id, slot_info)
            lane_name = getattr(lane, "name", "") or ""
            where = f" on {lane_name}" if lane_name else ""
            summary = format_tag_summary(
                info, f"{self.log_prefix}: read spool{where}")
            if "\n" in summary:
                gcode.respond_info(summary)
        except Exception as e:
            self.logger.debug(
                f"{self.log_prefix} console read-out skipped: {e}")
