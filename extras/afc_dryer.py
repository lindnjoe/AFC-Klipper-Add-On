# AFC Unit Heaters — one web panel for every filament dryer on the printer
#
# Bambu AMS 2 Pro / AMS HT and Anycubic ACE units all have drying heaters, and
# all of them are otherwise driven by typed G-code with different names, units
# and argument spellings (BAMBU_HEATER_START ... TIME=minutes vs
# ACE_DRY ... DURATION=minutes). This serves a single control panel for all of
# them, discovered from the printer config at startup: whatever dryers exist in
# AFC.cfg appear here, drawn to match their hardware.
#
# Configuration:
#
#   [afc_dryer]
#   port: 8093              # HTTP port to serve on
#   bind: 0.0.0.0           # 127.0.0.1 to restrict to the host
#   poll: 2.0               # seconds between status snapshots
#   show_heaterless: False  # also list EVERY other AFC unit -- BoxTurtle,
#                           # HTLF, OpenAMS and so on -- read-only, so the
#                           # panel can show the whole machine at once
#
# Then add it to Mainsail/Fluidd as a webcam with service type "iframe" and
# URL http://<printer-host>:8093/ — an iframe camera is the only way to get an
# interactive panel into either UI, neither having a plugin API. It is also
# just a web page, so that URL works on its own in any browser.
#
# Adding a vendor
# ---------------
# Subclass _Backend, fill in the six hooks, and add it to _BACKENDS. Nothing
# else in the module knows what an AMS or an ACE is -- the page renders from
# the descriptor each backend returns.
#
# Threading
# ---------
# The HTTP server runs on its own thread and Klipper's reactor is emphatically
# not thread-safe, so this module never touches printer objects from a request:
#
#   * status is snapshotted by a reactor timer into a plain dict under a lock,
#     and requests only ever read that dict;
#   * commands are handed back to the reactor with register_async_callback and
#     run there, never inline in the handler.
#
# Reading a unit's get_status() from the HTTP thread would mostly appear to
# work, which is exactly what makes it a bad idea.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from __future__ import annotations

import hashlib
import json
import logging
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from typing import Any, Dict, List, Optional

# Offered in the UI. The per-unit ceiling disables the rest, so an ACE (55 C by
# default) and an AMS HT (85 C) share one list without either being wrong.
TEMP_CHOICES = (40, 45, 50, 55, 60, 65, 70, 75, 80, 85)
# Minutes, 1..12 h in whole hours. Sent as minutes because that is what both
# vendors' commands take (BAMBU_HEATER_START TIME= / ACE_DRY DURATION=); the
# labels are hours because that is how dry cycles are quoted.
TIME_CHOICES = tuple((h * 60, "%d h" % h) for h in range(1, 13))


def _css_color(value: Any) -> str:
    """
    Normalise a vendor's spool colour into something CSS can use.

    Bambu reports "RRGGBB", ACE reports [r, g, b]. Returning "" for anything
    unusable keeps "no colour known" distinguishable all the way to the page,
    which decides what to draw: an empty bay stays empty, and a LOADED spool
    with no colour draws black -- the same fallback AFC itself uses, so the
    panel agrees with the rest of the UI rather than inventing a shade.

    All-zero is treated as unknown rather than as black for the same reason:
    it is what both vendors send when they have no colour at all, and it
    reaches the page as "" so the loaded/empty distinction still decides.

    :param value: vendor colour field.
    :return str: a CSS colour, or "" when there isn't a usable one.
    """
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        try:
            r, g, b = (max(0, min(255, int(c))) for c in value[:3])
        except (TypeError, ValueError):
            return ""
        return "" if (r, g, b) == (0, 0, 0) else "rgb(%d,%d,%d)" % (r, g, b)
    if isinstance(value, str) and value:
        v = value.strip().lstrip("#")
        if len(v) >= 6:
            try:
                int(v[:6], 16)
            except ValueError:
                return ""
            return "" if v[:6].upper() == "000000" else "#" + v[:6]
    return ""


def _lane_bays(unit: Any, nslots: int) -> List[Dict[str, str]]:
    """
    Read spool colour and material per bay from the unit's AFC lanes.

    This is the colour the rest of AFC already agrees on -- set from an RFID
    read, from Spoolman via Moonraker, or from the lane's own config -- so it
    matches what Mainsail shows for the same spool. The vendor's own slot data
    is a poorer source: an ACE reports [0,0,0] for a spool it has no colour
    for, which would paint every bay the same fallback shade.

    Both vendors map lane -> bay through a ``_slot_map`` built from the lane's
    1-based config index, so one helper covers them; a unit without one falls
    back to that index directly.

    :param unit: the AFC unit object.
    :param nslots: how many bays the unit exposes.
    :return list: per-bay {"color", "material"}, blank where unknown.
    """
    bays: List[Dict[str, str]] = [{"color": "", "material": ""}
                                  for _ in range(nslots)]
    # Everything below colour/material is for the hover tooltip. It is the
    # same data Mainsail shows for the spool -- AFC populates these from an
    # RFID read, from Spoolman via Moonraker, or from lane config -- so the
    # panel does not need its own Spoolman client to say something useful.
    lanes = getattr(unit, "lanes", None) or {}
    slot_map = getattr(unit, "_slot_map", None) or {}
    for name, lane in lanes.items():
        slot = slot_map.get(name)
        if slot is None:
            try:
                slot = int(getattr(lane, "index", 0)) - 1
            except (TypeError, ValueError):
                continue
        if not 0 <= slot < nslots:
            continue
        weight = getattr(lane, "weight", 0) or 0
        try:
            weight = float(weight)
        except (TypeError, ValueError):
            weight = 0.0
        bays[slot] = {
            "color": _css_color(getattr(lane, "color", "") or ""),
            "material": str(getattr(lane, "material", "") or ""),
            "lane": str(name),
            # filament_name is Spoolman's own name for the spool when one is
            # linked; sub_type is the tag/Spoolman variant ("Matte").
            "filament": str(getattr(lane, "filament_name", "") or ""),
            "sub_type": str(getattr(lane, "sub_type", "") or ""),
            "vendor": str(getattr(lane, "spool_vendor", "") or ""),
            "spool_id": getattr(lane, "spool_id", None),
            # Grams remaining. 0 means "not tracked" rather than "empty", so
            # the tooltip omits it instead of claiming an empty spool.
            "weight": round(weight, 1) if weight > 0 else None,
            "temp": getattr(lane, "extruder_temp", None),
        }
    return bays


def _merge_bays(vendor: List[Dict[str, Any]],
                lane: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    """
    Overlay AFC lane colour/material onto the vendor's presence data.

    Presence stays with the vendor -- it is the one that can see whether a
    spool is physically in the bay -- while colour and material prefer the
    lane, falling back to the vendor's own value when AFC has none.

    :param vendor: per-bay dicts from the backend (authoritative for presence).
    :param lane: per-bay dicts from _lane_bays.
    :return list: merged per-bay dicts.
    """
    out = []
    for i, v in enumerate(vendor):
        ln = lane[i] if i < len(lane) else {}
        merged = {
            "present": bool(v.get("present")),
            "color": ln.get("color") or v.get("color") or "",
            "material": ln.get("material") or v.get("material") or "",
        }
        # Tooltip detail comes from the LANE only. The vendor payload has no
        # equivalent, and a bay with no lane mapped simply carries none --
        # which the page reads as "nothing to say" and shows no tooltip.
        for k in ("lane", "filament", "sub_type", "vendor", "spool_id",
                  "weight", "temp"):
            if ln.get(k) not in (None, ""):
                merged[k] = ln[k]
        out.append(merged)
    return out


def _reading(value: Any) -> Optional[float]:
    """
    A sensor value, or None when there is nothing to show.

    Every one of these sensors reports 0.0 for a channel it has not read yet:
    a Bambu HT's aht10 sits at temperature 0.0 while its humidity reads 41.0,
    and an unread OpenAMS driver reports 0.0 for both. The page renders a
    dash for None and the number for anything else, so passing that through
    puts "0 °C" on a card -- which looks like a real reading of a freezing
    chamber rather than an absent one.

    0 is treated as absent rather than plausible on purpose. These are ambient
    sensors inside a room; a genuine 0 °C or 0 % would mean something is
    badly wrong, and showing a dash for it is the safer error of the two.

    Applied HERE, where every backend's view converges, rather than in each
    backend -- one of them already did this and the others did not, which is
    exactly the kind of inconsistency that reappears with the next backend.

    :param value: a raw sensor reading, or None
    :return Optional[float]: the reading, or None if absent/unread
    """
    try:
        return float(value) or None
    except (TypeError, ValueError):
        return None


class _Backend:
    """One vendor's dryers. Subclasses map a unit object onto the panel's view."""

    #: Klipper object prefixes to enumerate, e.g. ("AFC_BambuAMS",). A vendor
    #: can ship more than one unit class -- the ACE 2 Pro is a separate class
    #: from the V1 ACE because it speaks a different wire protocol, even
    #: though it subclasses it and shares every dryer command.
    object_prefixes = ()
    #: Short tag used by the page to pick artwork.
    kind = ""

    def find(self, printer: Any) -> List[Any]:
        """Every unit this backend claims.

        Defaults to enumerating ``object_prefixes`` through Klipper. Override
        when a backend's units are not identifiable by object name -- the
        generic backend sources them from AFC's own registry instead.

        :param printer: The Klipper printer object
        :return list: unit objects, possibly with duplicates
        """
        found: List[Any] = []
        for prefix in self.object_prefixes:
            try:
                found.extend(u for _, u in printer.lookup_objects(prefix))
            except Exception:
                continue
        return found

    def has_heater(self, unit: Any) -> bool:
        """Whether this unit can dry at all."""
        raise NotImplementedError

    def describe(self, unit: Any) -> Dict[str, Any]:
        """Static descriptor: model, max_temp, slots. Read once at startup."""
        raise NotImplementedError

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        """Live view: online, drying, temperature, humidity, target, note."""
        raise NotImplementedError

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Per-bay spool presence and colour, for the artwork."""
        raise NotImplementedError

    def start_script(self, name: str, temp: int, minutes: int,
                     rotate: int) -> str:
        """G-code that starts a cycle on this vendor."""
        raise NotImplementedError

    def stop_script(self, name: str) -> str:
        """G-code that stops a cycle on this vendor."""
        raise NotImplementedError

    #: Whether the vendor supports spinning spools while drying.
    supports_rotate = False


class _BambuBackend(_Backend):
    """Bambu AMS 2 Pro (65 C, 4 bays) and AMS HT (85 C, 1 bay)."""

    object_prefixes = ("AFC_BambuAMS",)
    kind = "bambu"
    supports_rotate = True

    def has_heater(self, unit: Any) -> bool:
        return bool(getattr(unit, "has_heater", False))

    def describe(self, unit: Any) -> Dict[str, Any]:
        model = str(getattr(unit, "ams_model", "ams2") or "ams2").lower()
        return {
            "model": model,
            "label": {"ht": "AMS HT", "ams2": "AMS 2 Pro"}.get(model, "AMS"),
            "max_temp": int(getattr(unit, "dry_max_temp", 65)),
            "slots": int(getattr(unit, "unit_slots", 4)),
        }

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "online": bool(st.get("bridge_online")),
            "drying": bool(st.get("drying")),
            "temperature": st.get("temperature"),
            "humidity": st.get("humidity"),
            "target": st.get("dry_target"),
            # The AMS's own reason for declining, when it declined. It echoes
            # our temp/time back before refusing, so the command IS delivered
            # and we report success -- without this the card sits at "not
            # drying" with nothing to explain it.
            "note": "",
            # Kept SEPARATE from note. A note describes a running cycle
            # ("Drying -- ..."); this is the unit declining to run one, and
            # overloading the same field made the card read "Drying" next to
            # the reason it was not.
            "error": str(st.get("dry_error") or ""),
        }

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        out = []
        for s in (st.get("slots") or []):
            s = s or {}
            out.append({"present": bool(s.get("present")),
                        "color": _css_color(s.get("color")),
                        "material": s.get("material") or ""})
        return out

    def start_script(self, name, temp, minutes, rotate):
        return ("BAMBU_HEATER_START UNIT=%s TEMP=%d TIME=%d ROTATE=%d"
                % (name, temp, minutes, rotate))

    def stop_script(self, name):
        return "BAMBU_HEATER_STOP UNIT=%s" % (name,)


class _AceBackend(_Backend):
    """Anycubic ACE / ACE 2 Pro. Duration is minutes, as on the Bambu side."""

    object_prefixes = ("AFC_ACE", "AFC_ACE2")
    kind = "ace"
    # The ACE firmware has no spool-rotation option while drying.
    supports_rotate = False

    #: ace_dryer values that mean "not running". Anything else is treated as a
    #: live cycle and shown verbatim, so an unfamiliar state reads as itself
    #: rather than being silently rounded to "idle".
    _IDLE = ("", "stop", "stopped", "idle", "off", "none")

    def has_heater(self, unit: Any) -> bool:
        # Every ACE ships with a dryer; a unit can still opt out by setting
        # max_dryer_temperature: 0.
        return float(getattr(unit, "max_dryer_temperature", 55.0)) > 0

    def describe(self, unit: Any) -> Dict[str, Any]:
        # afcACE2 subclasses afcACE, so an isinstance check would call every
        # Pro 2 an ACE. Walk the MRO names instead: the Pro 2 is a different
        # machine to its owner and should say so.
        is_v2 = any("ACE2" in c.__name__.upper() for c in type(unit).__mro__)
        return {
            "model": "ace",
            "label": "ACE 2" if is_v2 else "ACE",
            "max_temp": int(float(getattr(unit, "max_dryer_temperature", 55.0))),
            "slots": int(getattr(unit, "SLOTS_PER_UNIT", 4)),
        }

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        dryer = str(st.get("ace_dryer") or "").strip()
        running = dryer.lower() not in self._IDLE
        return {
            "online": bool(st.get("ace_connected")),
            "drying": running,
            "temperature": st.get("ace_temp"),
            "humidity": st.get("ace_humidity"),
            # The ACE status payload carries no separate set-point, so there is
            # no target to show -- the vendor note carries its own wording.
            "target": None,
            "note": dryer if running else "",
        }

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        out = []
        for s in (st.get("ace_slots") or []):
            s = s or {}
            status = str(s.get("status") or "").lower()
            out.append({
                "present": status not in ("", "empty"),
                "color": _css_color(s.get("color")),
                "material": s.get("material") or "",
            })
        return out

    def start_script(self, name, temp, minutes, rotate):
        return "ACE_DRY UNIT=%s TEMP=%d DURATION=%d" % (name, temp, minutes)

    def stop_script(self, name):
        return "ACE_DRY_STOP UNIT=%s" % (name,)


def _env_reading(unit: Any) -> Dict[str, Optional[float]]:
    """
    Temperature and humidity for a unit that keeps them on a SEPARATE sensor.

    A Bambu unit reports its own chamber telemetry, so its backend just reads
    it. An OpenAMS does not: its HDC1080/AHT is an ordinary Klipper sensor
    section, and the humidity only reaches the printer objects through the
    sensor driver's own object -- `temperature_sensor oams1` carries
    temperature alone, while `aht10 oams1` carries both. That is why the panel
    showed nothing for a unit whose readings were sitting right there.

    Nor can the object be found by the unit's name: the AFC unit here is
    `ams_1` while its sensor is `oams1`, named after the [AFC_OAMS] controller.
    So candidates are tried in order of how specific they are -- the unit's
    declared controller first, then its own name.

    :param unit: an AFC unit object
    :return Dict[str, Optional[float]]: {"temperature": .., "humidity": ..}
    """
    empty = {"temperature": None, "humidity": None}
    printer = getattr(unit, "printer", None)
    if printer is None:
        return empty
    names = [n for n in (getattr(unit, "oams_name", None),
                         getattr(unit, "name", None)) if n]
    # Driver objects first: they are the ones that carry humidity. The
    # temperature_sensor wrapper is the fallback, and gives temperature only.
    for prefix in ("aht10", "aht2x", "temperature_oams", "temperature_sensor"):
        for name in names:
            try:
                obj = printer.lookup_object("%s %s" % (prefix, name), None)
                st = obj.get_status(None) if obj is not None else None
            except Exception:
                st = None
            if not st:
                continue
            temp = st.get("temperature")
            hum = st.get("humidity")
            # A driver that is present but has not read yet reports 0.0 for
            # both; treat that as "nothing to show" rather than a real 0C.
            temp, hum = _reading(temp), _reading(hum)
            if temp is not None or hum is not None:
                return {"temperature": temp, "humidity": hum}
    return empty


class _GenericBackend(_Backend):
    """Any other AFC unit -- BoxTurtle, HTLF, NightOwl, QuattroBox, OpenAMS,
    Vivid and anything added later.

    None of them dry, so they only ever appear when show_heaterless is on, and
    they are read-only: the panel is the one place an operator sees every unit
    on the machine at once, and a BoxTurtle missing from that list reads as a
    fault rather than as "this one has no heater".

    Sourced from AFC's own registry rather than object prefixes. Every unit
    class registers itself into afc.units, so this needs no list of class
    names to keep in step with new hardware -- which is exactly the
    maintenance burden that left the panel showing two vendors.

    Listed LAST in _BACKENDS: the vendor backends claim their own units first
    and _discover's `seen` set keeps this one from re-adding them, so a Bambu
    unit is still described as a Bambu unit.
    """

    kind = "generic"
    supports_rotate = False

    #: Registered in afc.units but not a filament source. A toolchanger is a
    #: unit only in AFC's bookkeeping sense -- it holds tools, not spools --
    #: so it has no bays to draw and nothing to report, and a card for it is
    #: noise on a page about what is in each bay. Matched on the class MRO
    #: rather than the `type` string, which is operator-settable.
    _NOT_FILAMENT_UNITS = ("TOOLCHANGER",)

    def find(self, printer: Any) -> List[Any]:
        afc = printer.lookup_object("AFC", None)
        units = getattr(afc, "units", None) if afc is not None else None
        if not units:
            return []
        return [u for u in units.values() if not self._is_excluded(u)]

    def _is_excluded(self, unit: Any) -> bool:
        """Whether this object holds tools rather than filament."""
        names = [c.__name__.upper() for c in type(unit).__mro__]
        return any(any(bad in n for n in names)
                   for bad in self._NOT_FILAMENT_UNITS)

    def has_heater(self, unit: Any) -> bool:
        return False

    def describe(self, unit: Any) -> Dict[str, Any]:
        # `type` is the unit's own config-declared model ("BoxTurtle",
        # "NightOwl", ...). Falling back to the class name keeps a unit that
        # does not set one from showing up as "?".
        label = str(getattr(unit, "type", "") or type(unit).__name__)
        return {
            "model": "generic",
            "label": label,
            "max_temp": 0,
            "slots": len(getattr(unit, "lanes", {}) or {}),
        }

    def snapshot(self, unit: Any, st: Dict[str, Any]) -> Dict[str, Any]:
        env = _env_reading(unit)
        has_env = env["temperature"] is not None or env["humidity"] is not None
        return {
            "online": True,
            "drying": False,
            "temperature": env["temperature"],
            "humidity": env["humidity"],
            "target": None,
            # "no dryer" is still true -- there is no heater to drive -- but
            # it reads as "nothing to report" next to a live reading, so say
            # what the card is actually showing.
            "note": "monitor only" if has_env else "no dryer",
        }

    def slots(self, unit: Any, st: Dict[str, Any]) -> List[Dict[str, Any]]:
        out = []
        for lane in (getattr(unit, "lanes", {}) or {}).values():
            out.append({
                "present": bool(getattr(lane, "load_state", False)),
                "color": _css_color(getattr(lane, "color", None)),
                "material": getattr(lane, "material", "") or "",
            })
        return out

    def start_script(self, name, temp, minutes, rotate):
        # Unreachable through the panel -- has_heater() is False, so no start
        # control is rendered -- but a clear error beats a broken g-code line
        # if some future caller reaches it anyway.
        raise ValueError("%s has no dryer" % (name,))

    def stop_script(self, name):
        raise ValueError("%s has no dryer" % (name,))


_BACKENDS = (_BambuBackend(), _AceBackend(), _GenericBackend())


class AFCDryer:
    """Discovers every dryer on the printer and serves the control panel."""

    def __init__(self, config: Any) -> None:
        """
        Read config and register lifecycle hooks. Unit discovery waits for
        klippy:ready -- the AFC units are built by the AFC framework, which has
        not run while this section is being parsed.

        :param config: Klipper config for the ``[afc_dryer]`` section.
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.logger = logging.getLogger("afc_dryer")
        self.port = config.getint("port", 8093, minval=1, maxval=65535)
        self.bind = config.get("bind", "0.0.0.0")
        self.poll = config.getfloat("poll", 2.0, minval=0.5)
        # A dryer-less unit has nothing to control, but listing it keeps the
        # panel a picture of the whole machine rather than a partial one.
        self.show_heaterless = config.getboolean("show_heaterless", False)

        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._timer = None
        self._lock = threading.Lock()
        self._units: List[Dict[str, Any]] = []
        self._state: Dict[str, Any] = {"units": [], "ready": False}

        self.printer.lookup_object("gcode").register_command(
            "AFC_DRYER_STATUS", self.cmd_STATUS,
            desc="Show the AFC Unit Heaters panel status")
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)

    # ---- lifecycle ----

    def _handle_ready(self) -> None:
        self._discover()
        if not self._units:
            self.logger.warning(
                "afc_dryer: no units with a drying heater found -- the panel "
                "will be empty (set show_heaterless: True to list every other "
                "AFC unit read-only)")
        self._timer = self.reactor.register_timer(
            self._snapshot, self.reactor.NOW)
        self._start_server()

    def _handle_disconnect(self) -> None:
        """Shut the server down so a Klipper restart can rebind the port."""
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
            self._server = None

    def _discover(self) -> None:
        """Resolve every unit's static descriptor once, from config."""
        units: List[Dict[str, Any]] = []
        seen = set()
        for backend in _BACKENDS:
            try:
                found = backend.find(self.printer)
            except Exception as e:
                self.logger.warning(
                    "afc_dryer: could not enumerate %s units: %s",
                    backend.kind, e)
                continue
            for unit in found:
                # A subclass matches its parent's prefix too, so the same
                # object can come back from more than one lookup.
                if id(unit) in seen:
                    continue
                seen.add(id(unit))
                try:
                    heated = backend.has_heater(unit)
                    if not heated and not self.show_heaterless:
                        continue
                    desc = backend.describe(unit)
                except Exception as e:
                    self.logger.warning(
                        "afc_dryer: skipping a %s unit: %s",
                        backend.kind, e)
                    continue
                desc.update({
                    "name": getattr(unit, "name", "?"),
                    "kind": backend.kind,
                    "has_heater": heated,
                    "rotate": backend.supports_rotate,
                    "_obj": unit,
                    "_backend": backend,
                })
                units.append(desc)
        units.sort(key=lambda u: (u["kind"], u["name"]))
        self._units = units
        self.logger.info(
            "afc_dryer: %d dryer(s): %s", len(units),
            ", ".join("%s (%s)" % (u["name"], u["label"]) for u in units)
            or "none")

    def _start_server(self) -> None:
        try:
            self._server = ThreadingHTTPServer(
                (self.bind, self.port), _make_handler(self))
            self._server.daemon_threads = True
            self._thread = threading.Thread(
                target=self._server.serve_forever, daemon=True)
            self._thread.start()
            self.logger.info("afc_dryer: serving on http://%s:%d/",
                             self.bind, self.port)
            self.logger.info(
                "  Mainsail/Fluidd: add a webcam, service 'iframe', URL "
                "http://<printer-host>:%d/", self.port)
        except Exception as e:
            self.logger.error(
                "afc_dryer: failed to start HTTP server on %s:%d: %s",
                self.bind, self.port, e)
            self._server = None

    # ---- status snapshot (reactor thread) ----

    def _snapshot(self, eventtime: float) -> float:
        """
        Reactor timer: copy each unit's status into a plain dict the HTTP
        thread can read safely. Only primitives are published -- no printer
        object escapes onto the other thread.

        :param eventtime: reactor time of this firing.
        :return float: next firing time.
        """
        units = []
        for desc in self._units:
            unit, backend = desc["_obj"], desc["_backend"]
            try:
                st = unit.get_status(eventtime) or {}
                view = backend.snapshot(unit, st)
                # Vendor data decides which bays are occupied; the AFC lanes
                # supply the colour, so the artwork matches the spool colours
                # the rest of the UI already shows.
                slots = _merge_bays(backend.slots(unit, st),
                                    _lane_bays(unit, desc["slots"]))
            except Exception as e:
                self.logger.debug("afc_dryer: status failed for %s: %s",
                                  desc["name"], e)
                view, slots = {"online": False, "drying": False}, []
            slots = slots[:desc["slots"]]
            # Pad so the artwork always draws the unit's full complement of
            # bays, empty ones included.
            while len(slots) < desc["slots"]:
                slots.append({"present": False, "color": "", "material": ""})
            row = {k: desc[k] for k in
                   ("name", "kind", "model", "label", "max_temp", "slots",
                    "has_heater", "rotate")}
            row.update({
                "online": bool(view.get("online")),
                "drying": bool(view.get("drying")),
                "temperature": _reading(view.get("temperature")),
                "humidity": _reading(view.get("humidity")),
                "target": view.get("target"),
                "note": view.get("note") or "",
                "error": view.get("error") or "",
                "bays": slots,
            })
            units.append(row)
        with self._lock:
            self._state = {"units": units, "ready": True}
        return eventtime + self.poll

    def get_state(self) -> Dict[str, Any]:
        """Thread-safe read of the latest snapshot, stamped with the page
        version so a stale browser can notice it is out of date."""
        with self._lock:
            state = dict(self._state)
        state["page_version"] = PAGE_VERSION
        return state

    # ---- command dispatch (HTTP thread -> reactor) ----

    def request_dry(self, name: str, action: str, temp: int, minutes: int,
                    rotate: int) -> str:
        """
        Validate a panel request and queue the matching G-code on the reactor.

        Every field is validated against the discovered units rather than
        trusted: the unit name is matched to a known unit and the rest are
        coerced to ints, so nothing from an HTTP body is ever interpolated
        into a G-code string as free text. The panel binds 0.0.0.0 by default,
        which makes that the difference between a control panel and a remote
        command shell.

        :param name: unit name from the request.
        :param action: "start" or "stop".
        :param temp: requested temperature in C.
        :param minutes: requested run time in minutes.
        :param rotate: 1 to spin the spools while drying (Bambu only).
        :return str: the G-code queued.
        :raises ValueError: if the request does not name a real dryer.
        """
        desc = next((u for u in self._units if u["name"] == name), None)
        if desc is None:
            raise ValueError("unknown unit %r" % (name,))
        if not desc["has_heater"]:
            raise ValueError("%s has no drying heater" % (name,))
        backend = desc["_backend"]
        if action == "stop":
            script = backend.stop_script(desc["name"])
        elif action == "start":
            # Clamp here as well as in the unit: the panel should not be able
            # to ask for something the unit will only refuse.
            temp = max(0, min(int(temp), desc["max_temp"]))
            minutes = max(0, min(int(minutes), 65535))
            rot = 1 if (int(rotate) and desc["rotate"]) else 0
            script = backend.start_script(desc["name"], temp, minutes, rot)
        else:
            raise ValueError("unknown action %r" % (action,))
        self.reactor.register_async_callback(
            lambda e, s=script: self._run_script(s))
        return script

    def _run_script(self, script: str) -> None:
        """Run a queued G-code on the reactor thread."""
        try:
            self.printer.lookup_object("gcode").run_script(script)
        except Exception as e:
            self.logger.error("afc_dryer: %r failed: %s", script, e)

    # ---- gcode / status ----

    def cmd_STATUS(self, gcmd: Any) -> None:
        """Report where the panel is served and what it found."""
        units = self.get_state().get("units", [])
        gcmd.respond_info(
            "AFC Unit Heaters: %s on http://%s:%d/ — %d dryer(s): %s"
            % ("running" if self._server else "NOT running", self.bind,
               self.port, len(units),
               ", ".join("%s [%s] %s" % (
                   u["name"], u["label"],
                   "drying" if u["drying"] else
                   ("idle" if u["online"] else "offline"))
                   for u in units) or "none"))

    def get_status(self, eventtime: Any = None) -> Dict[str, Any]:
        """Expose the panel's own state to the printer status API."""
        return {
            "running": self._server is not None,
            "port": self.port,
            "units": self.get_state().get("units", []),
        }


def _make_handler(panel: AFCDryer):
    """
    Build the request handler bound to a panel instance.

    :param panel: the AFCDryer to serve.
    :return: a BaseHTTPRequestHandler subclass.
    """

    class Handler(BaseHTTPRequestHandler):
        server_version = "AFCDryer/1.0"

        def log_message(self, fmt, *args):
            # The default logs every request to stderr, which lands in
            # klippy.log -- at 2s polling that is a lot of noise.
            panel.logger.debug("afc_dryer: " + fmt, *args)

        def _send(self, code, body, ctype):
            if isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            # The page is embedded in Mainsail/Fluidd via an iframe on another
            # origin and its fetches come back here -- both need CORS.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except Exception:
                pass          # client navigated away mid-response

        def _json(self, code, obj):
            self._send(code, json.dumps(obj), "application/json")

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self):
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            if path == "/":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(200, panel.get_state())
            elif path == "/api/options":
                self._json(200, {
                    "temps": list(TEMP_CHOICES),
                    "times": [{"minutes": m, "label": lbl}
                              for m, lbl in TIME_CHOICES]})
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            path = self.path.split("?", 1)[0].rstrip("/") or "/"
            if path != "/api/dry":
                self._json(404, {"error": "not found"})
                return
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
            except Exception as e:
                self._json(400, {"error": "bad request body: %s" % (e,)})
                return
            try:
                script = panel.request_dry(
                    name=str(body.get("unit", "")),
                    action=str(body.get("action", "")),
                    temp=int(body.get("temp", 55)),
                    minutes=int(body.get("minutes", 480)),
                    rotate=int(body.get("rotate", 0)))
            except (ValueError, TypeError) as e:
                self._json(400, {"error": str(e)})
                return
            self._json(200, {"ok": True, "queued": script})

    return Handler


# One self-contained document -- no CDN, no build step, no assets to install
# beside the module. The unit artwork is inline SVG for the same reason: it
# scales, themes with the page, and survives a plain file copy.
_PAGE_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AFC Unit Heaters</title>
<style>
  :root{
    --bg:#f5f6f8; --card:#fff; --fg:#1b1d21; --dim:#6b7280; --line:#dfe3e8;
    --accent:#2f8f4e; --hot:#e2673a; --off:#9aa0a6; --shadow:rgba(0,0,0,.08);
  }
  @media (prefers-color-scheme: dark){
    :root{
      --bg:#1b1d21; --card:#25282e; --fg:#e8eaed; --dim:#9aa0a6; --line:#343941;
      --accent:#4caf6d; --hot:#ff8a5c; --off:#5f6570; --shadow:rgba(0,0,0,.35);
    }
  }
  *{box-sizing:border-box}
  body{margin:0;padding:14px;background:var(--bg);color:var(--fg);
    font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
  h1{font-size:13px;font-weight:600;margin:0 0 12px;color:var(--dim);
    letter-spacing:.06em;text-transform:uppercase}
  .grid{display:grid;gap:14px;grid-template-columns:repeat(auto-fill,minmax(330px,1fr))}
  .card{background:var(--card);border:1px solid var(--line);border-radius:10px;
    padding:14px;box-shadow:0 1px 3px var(--shadow)}
  .head{display:flex;align-items:center;gap:9px;margin-bottom:10px}
  .name{font-weight:600;font-size:15px}
  .model{font-size:10px;color:var(--dim);text-transform:uppercase;
    letter-spacing:.05em;border:1px solid var(--line);border-radius:4px;padding:1px 6px}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--off);
    margin-left:auto;flex:none}
  .dot.on{background:var(--accent)}
  .dot.hot{background:var(--hot);animation:pulse 1.6s ease-in-out infinite}
  @keyframes pulse{50%{opacity:.3}}
  .body{display:flex;gap:13px;align-items:flex-start}
  .art{flex:none}
  .readouts{flex:1;min-width:0;display:grid;grid-template-columns:1fr 1fr;gap:8px}
  .ro{background:var(--bg);border-radius:7px;padding:7px 9px;min-width:0}
  .ro .k{font-size:10px;color:var(--dim);text-transform:uppercase;letter-spacing:.04em}
  .ro .v{font-size:17px;font-weight:600;white-space:nowrap;overflow:hidden;
    text-overflow:ellipsis}
  .state{grid-column:1/-1;font-size:13px;font-weight:500}
  .state.hot{color:var(--hot)} .state.on{color:var(--accent)}
  .state.off{color:var(--dim)}
  .ctl{margin-top:12px;padding-top:12px;border-top:1px solid var(--line);
    display:flex;flex-wrap:wrap;gap:8px;align-items:flex-end}
  label{font-size:10px;color:var(--dim);text-transform:uppercase;
    letter-spacing:.04em;display:block;margin-bottom:3px}
  select{background:var(--bg);color:var(--fg);border:1px solid var(--line);
    border-radius:6px;padding:6px 8px;font:inherit;font-size:13px}
  select:disabled{opacity:.5}
  .sw{display:flex;align-items:center;gap:6px;font-size:13px;cursor:pointer;
    user-select:none;padding-bottom:6px;text-transform:none;letter-spacing:0;
    color:var(--fg);margin:0}
  .sw input{width:16px;height:16px;accent-color:var(--accent);cursor:pointer}
  button{border:0;border-radius:6px;padding:8px 15px;font:inherit;
    font-weight:600;font-size:13px;cursor:pointer;color:#fff;
    background:var(--accent)}
  .ctl button{margin-left:auto}
  button.stop{background:var(--hot)}
  button:disabled{opacity:.4;cursor:not-allowed}
  .msg{margin-top:8px;font-size:12px;color:var(--dim);min-height:1.2em;
    word-break:break-word}
  .msg.err{color:var(--hot)}
  .empty{color:var(--dim);padding:24px;text-align:center;background:var(--card);
    border:1px dashed var(--line);border-radius:10px}
  .spool{transition:fill .4s}
</style>
</head>
<body>
<h1>AFC Unit Heaters</h1>
<div id="grid" class="grid"></div>
<script>
"use strict";
var OPTS = {temps:[], times:[]};

// A loaded spool AFC has no colour for draws BLACK, which is the colour AFC
// itself falls back to -- so the panel and the rest of the UI agree instead of
// inventing a different placeholder shade. The bay ring is drawn separately in
// --line, so a black spool still reads on a dark theme. Full opacity: at .85 a
// black spool picked up the card behind it and came out charcoal.
function bay(cx, cy, r, s, hub){
  var fill = s.present ? (s.color || '#000000') : 'transparent';
  // Native SVG tooltip. No JS, no positioning maths, and screen readers get
  // it for free -- worth more than a styled div that only works with a mouse.
  //
  // The <g> is load-bearing, not tidiness. A <title> that is a direct child
  // of <svg> titles the WHOLE image, so the first bay's text was winning
  // everywhere and every spool in a unit showed the same tooltip. Scoping it
  // to a group is what makes it per-bay.
  var tip = spoolTip(s);
  return '<g>' + (tip ? '<title>' + tip + '</title>' : '') +
         '<circle cx="' + cx + '" cy="' + cy + '" r="' + r + '" fill="none" stroke="var(--line)" stroke-width="1.4"/>' +
         '<circle class="spool" cx="' + cx + '" cy="' + cy + '" r="' + (r-1.5) + '" fill="' + fill + '" opacity="' + (s.present ? '1' : '0') + '"/>' +
         '<circle cx="' + cx + '" cy="' + cy + '" r="' + hub + '" fill="var(--card)" stroke="var(--line)"/></g>';
}

// What a spool says when you point at it. Built from the AFC lane, which is
// already fed by RFID reads and by Spoolman through Moonraker, so this is the
// same spool identity Mainsail shows -- no second Spoolman client needed.
// Lines are omitted rather than shown blank: a tooltip reading "Weight: --"
// is worse than one that simply does not mention weight.
function esc(v){
  return String(v).replace(/&/g,'&amp;').replace(/</g,'&lt;')
                  .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}
function spoolTip(s){
  var out = [];
  if (s.lane) out.push(s.lane);
  if (!s.present){
    // An empty bay still names its lane, so hovering tells you WHICH bay it
    // is rather than nothing at all.
    return out.length ? esc(out[0] + ' — empty') : '';
  }
  var name = [s.vendor, s.filament || s.material, s.sub_type]
               .filter(Boolean).join(' ');
  if (name) out.push(name);
  else if (s.material) out.push(s.material);
  if (s.weight != null) out.push(s.weight + ' g remaining');
  if (s.temp != null) out.push(s.temp + ' °C');
  if (s.spool_id != null) out.push('Spoolman #' + s.spool_id);
  return esc(out.join('\n'));
}

// Drawn rather than photographed: the artwork themes with the page, scales to
// any card width, and costs nothing to ship next to the module.
function art(u){
  var hot = u.drying;
  var ring = function(w,h){ return hot ? '<rect x="6" y="6" width="' + w + '" height="' + h + '" rx="7" fill="none" stroke="var(--hot)" stroke-width="2"/>' : ''; };
  var bays = u.bays || [];
  if (u.model === 'ht'){                       // tall single-spool tower
    return '<svg width="86" height="112" viewBox="0 0 86 112" role="img" aria-label="AMS HT">' +
      '<rect x="6" y="6" width="74" height="100" rx="7" fill="var(--bg)" stroke="var(--line)"/>' + ring(74,100) +
      bay(43, 52, 27, bays[0] || {}, 9) +
      '<rect x="20" y="88" width="46" height="8" rx="4" fill="var(--line)" opacity=".6"/></svg>';
  }
  // One wide row of bays for everything except the HT. The ACE used to be
  // drawn as a 2-over-2 grid, which no real unit is: an ACE stands its four
  // spools side by side exactly as an AMS does. Sharing the renderer also
  // means a unit with any bay count -- a 5-lane BoxTurtle listed read-only,
  // say -- draws correctly without another special case.
  var b = '';
  for (var i=0;i<bays.length;i++) b += bay(21 + i*24, 44, 10.5, bays[i]||{}, 3.4);
  var w = 12 + bays.length*24;
  var label = u.label || u.model || 'unit';
  return '<svg width="' + (w+12) + '" height="86" viewBox="0 0 ' + (w+12) + ' 86" role="img" aria-label="' + label + '">' +
    '<rect x="6" y="6" width="' + w + '" height="74" rx="7" fill="var(--bg)" stroke="var(--line)"/>' + ring(w,74) + b +
    '<rect x="16" y="66" width="' + (w-20) + '" height="6" rx="3" fill="var(--line)" opacity=".6"/></svg>';
}

// What the heater is doing, stated only as far as the data supports. A unit
// streams its chamber readings ONLY while a cycle runs, so their absence
// during a cycle means "starting", not "off" -- and a vendor that reports no
// set-point at all gets its own status text rather than an invented target.
function phase(u){
  if (!u.online) return {cls:'off', text:'Offline'};
  if (!u.has_heater) return {cls:'off', text:'No dryer on this unit'};
  // A refusal outranks everything below it: the unit was ASKED and said no,
  // so neither "Idle" nor "Drying" describes it. This is checked before the
  // drying flag because that flag is our own intent -- we set it when the
  // command goes out -- and the whole point here is that the unit disagreed.
  if (u.error) return {cls:'off', text:'Refused — ' + u.error};
  if (!u.drying) return {cls:'off', text:'Idle'};
  if (u.note) return {cls:'hot', text:'Drying — ' + u.note};
  if (u.target == null)
    return {cls:'hot', text:(u.temperature == null)
      ? 'Starting — waiting for the unit to report' : 'Drying'};
  if (u.temperature == null) return {cls:'hot', text:'Heating to ' + u.target + '°C'};
  return (u.temperature < u.target - 2)
    ? {cls:'hot', text:'Heating to ' + u.target + '°C'}
    : {cls:'on',  text:'Holding at ' + u.target + '°C'};
}

function num(v, suffix){ return (v == null) ? '—' : (Math.round(v*10)/10) + suffix; }
function idOf(name){ return 'u' + name.replace(/[^A-Za-z0-9]/g,'_'); }
function esc(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }
// Config names are underscore-joined because Klipper section names cannot hold
// spaces; a heading is not a section name. DISPLAY only -- the real name still
// goes out in data-unit and is what the server matches a request against, so
// prettifying it here can never change which unit gets the command.
function pretty(name){ return String(name).replace(/_/g, ' '); }

// Built ONCE per unit. Everything that changes with polling is patched in
// place by update() -- rebuilding this every 2s tore an open <select> out of
// the DOM, which read as the dropdown collapsing on its own after a moment.
function card(u){
  var id = idOf(u.name);
  var temps = OPTS.temps.map(function(t){
    return '<option value="' + t + '"' + (t > u.max_temp ? ' disabled' : '') +
           (t === 55 ? ' selected' : '') + '>' + t + '°C</option>'; }).join('');
  var times = OPTS.times.map(function(t){
    return '<option value="' + t.minutes + '"' + (t.minutes === 480 ? ' selected' : '') +
           '>' + t.label + '</option>'; }).join('');
  var rot = u.rotate
    ? '<label class="sw"><input type="checkbox" id="' + id + '-r"> Rotate</label>'
    : '';
  var ctl = u.has_heater ? (
    '<div class="ctl">' +
      '<div><label>Temp</label><select id="' + id + '-t">' + temps + '</select></div>' +
      '<div><label>Time</label><select id="' + id + '-m">' + times + '</select></div>' + rot +
      '<button id="' + id + '-btn" data-unit="' + esc(u.name) +
        '" data-id="' + id + '" data-act="start">Start</button>' +
    '</div><div class="msg" id="' + id + '-msg"></div>') : '';
  return '<div class="card" id="' + id + '">' +
    '<div class="head"><span class="name">' + esc(pretty(u.name)) + '</span>' +
      '<span class="model">' + esc(u.label) + '</span>' +
      '<span class="dot" id="' + id + '-dot"></span></div>' +
    '<div class="body"><div class="art" id="' + id + '-art"></div><div class="readouts">' +
      '<div class="ro"><div class="k">Chamber</div><div class="v" id="' + id + '-temp">—</div></div>' +
      '<div class="ro"><div class="k">Humidity</div><div class="v" id="' + id + '-hum">—</div></div>' +
      '<div class="ro state" id="' + id + '-state"></div>' +
    '</div></div>' + ctl + '</div>';
}

function setText(el, text){ if (el && el.textContent !== text) el.textContent = text; }
function setCls(el, cls){ if (el && el.className !== cls) el.className = cls; }
function setDis(el, off){ if (el && el.disabled !== off) el.disabled = off; }

// Patch a card in place. Nothing here replaces a control, so an open dropdown
// stays open and a half-made selection survives the poll.
function update(u){
  var id = idOf(u.name), p = phase(u), busy = u.drying;
  setCls(document.getElementById(id+'-dot'),
         'dot ' + (busy ? 'hot' : (u.online ? 'on' : '')));
  // The artwork is a string; only touch the DOM when it actually differs, or
  // the spool-colour transition restarts on every poll.
  var a = document.getElementById(id+'-art'), svg = art(u);
  if (a && a.dataset.svg !== svg){ a.dataset.svg = svg; a.innerHTML = svg; }
  setText(document.getElementById(id+'-temp'), num(u.temperature,'°C'));
  setText(document.getElementById(id+'-hum'), num(u.humidity,'%'));
  var st = document.getElementById(id+'-state');
  setText(st, p.text); setCls(st, 'ro state ' + p.cls);
  // Settings are meaningless mid-cycle, but Stop must stay reachable whenever
  // the unit is talking to us at all.
  setDis(document.getElementById(id+'-t'), busy);
  setDis(document.getElementById(id+'-m'), busy);
  setDis(document.getElementById(id+'-r'), busy);
  // One button that becomes Stop mid-cycle. Patched, not rebuilt, so the
  // dropdowns beside it survive the poll.
  var b = document.getElementById(id+'-btn');
  if (b){
    setText(b, busy ? 'Stop' : 'Start');
    setCls(b, busy ? 'stop' : '');
    b.dataset.act = busy ? 'stop' : 'start';
    setDis(b, !u.online);
  }
}

var built = "";        // which units the grid was built for

function render(state){
  var g = document.getElementById('grid');
  if (!state.units || !state.units.length){
    if (built !== "__none__"){
      built = "__none__";
      g.innerHTML = '<div class="empty">No filament dryers found.<br>' +
        'Expected a <code>[AFC_BambuAMS]</code> unit with <code>ams_model: ams2</code> or ' +
        '<code>ht</code>, or an <code>[AFC_ACE]</code> / <code>[AFC_ACE2]</code> unit.</div>';
    }
    return;
  }
  // Rebuild only when the SET of units changes (a unit added, removed or
  // renamed). Steady state never touches the grid's structure.
  var key = state.units.map(function(u){ return u.name + ':' + u.max_temp; }).join('|');
  if (key !== built){
    built = key;
    g.innerHTML = state.units.map(card).join('');
  }
  state.units.forEach(update);
}

document.addEventListener('click', function(ev){
  var b = ev.target.closest ? ev.target.closest('button[data-unit]') : null;
  if (!b) return;
  var id = b.dataset.id, act = b.dataset.act;
  var msg = document.getElementById(id + '-msg');
  var payload = {unit:b.dataset.unit, action:act};
  if (act === 'start'){
    payload.temp = parseInt(document.getElementById(id+'-t').value, 10);
    payload.minutes = parseInt(document.getElementById(id+'-m').value, 10);
    var r = document.getElementById(id+'-r');
    payload.rotate = (r && r.checked) ? 1 : 0;
  }
  b.disabled = true;
  msg.className = 'msg';
  msg.textContent = (act === 'start') ? 'Starting…' : 'Stopping…';
  fetch('api/dry', {method:'POST', headers:{'Content-Type':'application/json'},
                    body:JSON.stringify(payload)})
    .then(function(r){ return r.json().then(function(j){ return {ok:r.ok, j:j}; }); })
    .then(function(res){
      // Only errors are worth saying out loud. On success the state line
      // above already reports what the unit is doing, and echoing the raw
      // G-code back is noise on a panel whose whole point is not typing it.
      if (!res.ok){ msg.className = 'msg err'; msg.textContent = res.j.error || 'failed'; }
      else { msg.textContent = ''; }
    })
    .catch(function(e){ msg.className = 'msg err'; msg.textContent = String(e); })
    .then(function(){ b.disabled = false; tick(); });
});

// Stamped into the page when it is served; the server reports the version it
// is currently serving. A mismatch means this tab is running markup from
// before a deploy -- reload once rather than leaving it subtly out of date.
var MY_VERSION = "__PAGE_VERSION__";
var reloading = false;

function tick(){
  fetch('api/state').then(function(r){ return r.json(); }).then(function(state){
    if (!reloading && state.page_version && state.page_version !== MY_VERSION){
      reloading = true;
      location.reload();
      return;
    }
    render(state);
  }).catch(function(){ /* Klipper restarting; the next poll picks it up */ });
}
fetch('api/options').then(function(r){ return r.json(); })
  .then(function(o){ OPTS = o; })
  .catch(function(){})
  .then(function(){ tick(); setInterval(tick, 2000); });
</script>
</body>
</html>
"""

# The page is served once and then polls; a deploy therefore leaves an open
# browser running the PREVIOUS markup while its data keeps updating, which
# reads as "the fix didn't land". The page carries a hash of itself, compares
# it against the one the server reports, and reloads when they differ.
PAGE_VERSION = hashlib.md5(_PAGE_TEMPLATE.encode("utf-8")).hexdigest()[:8]
PAGE = _PAGE_TEMPLATE.replace("__PAGE_VERSION__", PAGE_VERSION)



def load_config(config: Any) -> AFCDryer:
    """
    Register the ``[afc_dryer]`` object.

    :param config: ConfigWrapper for the section.
    :return: the panel instance.
    """
    return AFCDryer(config)
