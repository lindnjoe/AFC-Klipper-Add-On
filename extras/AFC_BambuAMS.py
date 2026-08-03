# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC transport for a STOCK Bambu Lab AMS driven over its native RS-485 bus by a
# Raspberry Pi Pico running the `bambu_ams_bridge` firmware, which is a separate
# project -- the Pico is the timing-critical bus master and this module is only
# the Klipper end of the link. Without that bridge flashed and wired to the AMS
# cable, nothing here has anything to talk to.
#
# The Pico is the timing-critical Bambu-Bus master; Klipper talks to it over
# USB-CDC with a newline-JSON API. This module is the Klipper end of that link:
# it maps each AMS's slots to AFC lanes, mirrors slot status onto those lanes,
# and issues select / feed / retract / assist / scan / dry as bridge commands.
#
# STATUS: working on real hardware. Presence, RFID tag read (material/color/temp),
# select + feed/retract + self-centering follower, the AMS's own multi-stage
# load/unload motion, drying, jam detection, and humidity/temperature reporting
# are all confirmed against stock boxed AMS (ams1), AMS 2 Pro, and AMS HT units.
# The wire protocol was reverse-engineered from live printer<->AMS captures, kept
# with the bridge firmware alongside the findings they support.
#
# ── Jam detection ───────────────────────────────────────────────────
# Two independent signals, because no single one covers every model:
#   * the AMS's own report -- an HT says "feed finish -1, stall", a boxed AMS
#     says "[AMS_SWITCH]switch_feed rocker stall" or "[AMS_LED]TIMEOUT error N",
#     and each leads the buffer by seconds.
#   * buffer starvation -- the buffer bottomed out while the extruder keeps
#     pulling. Model-independent, and the surest signal on a quiet unit.
# Either pauses the print, drops the assist, and holds the follower down until
# the print resumes -- re-arming into a jam just grinds the filament.
#
# A stall the unit RECOVERED from is not a jam. A boxed AMS retries a reluctant
# bay by itself ("...rocker stall, tray_cnt:0,16,0,0" is its own retry counter
# climbing) and then loads fine, so a verified load acknowledges every fault
# outstanding at that point; only trouble after the load is the follower's to
# report.
#
# ── Per-model differences (do not assume these generalise) ──────────
#   * addressing: every frame goes to the unit's OWN device with its OWN id --
#     device = unit CLASS (0x0700 boxed, 0x1800 HT), id = position in the chain
#     (first 0, next 1, ...). Two units of the same class share a wire by their
#     chain index alone. The narration payload byte is that same class-base OR
#     chain-index value, so a payload read against the wrong base names the
#     wrong unit.
#   * narration namespace: a plain AMS narrates "[AMS_DEV] STEP:.." and "[RF] .."
#     (note the space after the bracket); an AMS 2 / HT use [AMS_RFID] /
#     [AMS_TRAY] / [AMS_COMMON] with no space. A rule written against one
#     model matches nothing on the other.
#   * self-centring: an AMS 2 / HT refill their own buffer, so the feeder poke
#     may wait for a real sag. A plain AMS does not and must be fed on demand;
#     making it wait lets the buffer empty before the follower re-engages. See
#     `self_centres`.
#   * chamber temperature exists only where there is a dryer, and only in the
#     AMS's own text telemetry -- it is not in the binary protocol. Its field
#     SEPARATOR follows the addressing, not the model: addressed to its own
#     device a unit emits "s:2, rf:55, cd:55, vt:23.1" (commas, extra `cd:`),
#     addressed bus-wide it emits "s:2|rf:55,0|vt:44.0". Both are parsed.
#
# ── Configuration ───────────────────────────────────────────────────
# The AMS has no external hub switch (it multiplexes internally), so use a
# VIRTUAL hub — this unit drives its state OpenAMS-style: slot presence sets
# each lane's prep_state, and the hub reads occupied only while a lane's
# filament is threaded through to the toolhead (tool_loaded).
#
#   [AFC_hub BambuAMS_hub]
#   switch_pin: virtual
#
#   [AFC_BambuAMS BambuAMS_1]
#   serial_port: /dev/serial/by-id/... # the Pico's USB-CDC port (NOT an [mcu])
#   ams_model: ams1                    # ams1 | ams2 | ht  (see AMS type below)
#   hub: BambuAMS_hub
#   extruder: extruder                 # AFC_extruder for this unit
#   # baud: 115200                     # USB-CDC; value is nominal
#   # variant: auto                    # auto|ams|lite — one firmware does both;
#   #                                  # 'auto' probes the bus, pin if needed
#   # auto_scan: True                  # read a tag automatically on insert
#   # follow_always: False             # hold the follower window permanently
#   #                                  # open. On a unit that does NOT
#   #                                  # self-centre this pokes the feeder
#   #                                  # continuously and ticks at idle.
#   # self_centres: <by model>         # does this unit refill its own buffer?
#   # fault_detect: True               # act on jams (see Jam detection above)
#   # fault_pause: True                # pause the print on a jam
#
# Temperature/humidity on the Mainsail/Fluidd card come from a companion
# [temperature_sensor] section -- see extras/temperature_bambu.py.
#
#   [AFC_lane lane15]
#   unit: BambuAMS_1:1                 # <unit>:<slot> — slot index goes HERE
#   # ... lane16..18 with :2 :3 :4 (no separate 'index:' option exists)
#
# A Bambu AMS-lite (A1) uses this SAME unit type — set ams_model: ams1 and
# variant: lite (or leave it auto). The single Pico firmware handles both.
#
# ── AMS type (ams_model) ─────────────────────────────────────────────
# Set `ams_model` per unit to its TYPE so it uses the right addressing + limits:
#   ams_model: ams1   -- regular AMS / AMS-lite (no heater, 4 slots)
#   ams_model: ams2   -- AMS2 Pro (heater, drying at 0x0700, up to 65 C, 4 slots)
#   ams_model: ht     -- AMS HT   (heater, drying at 0x1800, up to 85 C, 1 slot)
# `heater:` and `dry_max_temp:` default from the model but can be overridden.
#
# ── RFID tag reads ───────────────────────────────────────────────────
# A boxed AMS / AMS2 Pro reads a freshly inserted spool's tag itself at its bay
# reader; this module also nudges a scan on the insert edge. The AMS HT scans on
# COMMAND: on the insert edge the FIRMWARE sends the captured type-07 select
# (id 0x80) and the HT pulls the filament in, reads the tag (feeding it past the
# reader if needed), and unwinds -- all on its own. BAMBU_SCAN sends the
# printer's "re-identify" variant, which physically re-scans even a SEATED
# spool (covers boot-time swaps and same-spool re-inserts). A removed spool's
# tag is never re-applied without real scan activity (stale-tag guard).
#
# ── Commands ─────────────────────────────────────────────────────────
#   BAMBU_HEATER_START UNIT=<u> [TEMP=] [TIME=] [ROTATE=]   start drying (ams2/ht)
#   BAMBU_HEATER_STOP  UNIT=<u>                             stop drying
#   BAMBU_SCAN         UNIT=<u> [LANE=<lane>]               trigger a tag read
#   BAMBU_FOLLOWER     UNIT=<u> LANE=<lane> [ENABLE=]       engage/stop follower
#   BAMBU_RECOVER      UNIT=<u> LANE=<lane>                 reel a failed load back
#   BAMBU_RELINK       UNIT=<u>                             clear a TIMEOUT/error
#   BAMBU_UIDS                                              list bus UIDs (setup)
# Diagnostics (normally unnecessary; they change bus behaviour):
#   BAMBU_MUTE UNIT=<u> MASK=<bits>   silence one class of frame (0 restores).
#                                     256 = the presence poll, the usual suspect
#                                     when a unit ticks at idle.
#   BAMBU_ARMMS UNIT=<u> MS=<ms>      11/04 keep-alive cadence -- the one
#                                     transmitter BAMBU_MUTE cannot silence.
#   BAMBU_HB / BAMBU_HTPOLL / BAMBU_DRAIN / BAMBU_HTID / BAMBU_FEED /
#   BAMBU_BUFFER_PROBE                cadence + addressing sweeps for bring-up
#
# ── Daisy-chained AMS (up to 12) ─────────────────────────────────────
# Bambu's wire tops out at 4 four-slot AMS (ams1/ams2, device 0x0700) PLUS up to
# 8 AMS HT (device 0x1800) = 12 units total, never more than 4 four-slot units.
#
# STRONGLY RECOMMENDED with >1 AMS: set `unit_uid` per unit. The firmware assigns
# chain indices by ANNOUNCE ORDER, which reshuffles across power-cycles -- so a
# fixed ams_index can silently start addressing the wrong physical AMS after a
# cold boot. With unit_uid set, each unit pins to its physical AMS by UID no
# matter what order they boot. Run BAMBU_UIDS to read the UIDs off the wire
# (it also shows what each unit holds so you can tell them apart).
#
#   [AFC_BambuAMS BambuAMS_1]
#   serial_port: /dev/serial/by-id/...   # same Pico
#   ams_model: ams2                      # AMS2 Pro
#   ams_index: 0                         # fallback if unit_uid can't resolve
#   unit_uid: 68273B498053B0024C303936   # <- pins to this physical AMS
#   hub: BambuAMS_hub
#   ...lanes with unit: BambuAMS_1:1..4
#
#   [AFC_BambuAMS BambuAMS_HT]
#   serial_port: /dev/serial/by-id/...   # SAME port as unit 1
#   ams_model: ht                        # AMS HT
#   ams_index: 1
#   unit_uid: 872C3B871C00B0084A343331
#   hub: BambuAMS_hub2
#   ...lanes with unit: BambuAMS_HT:1

from __future__ import annotations

import traceback
from configparser import Error as error
import logging
from typing import Any, Callable, Dict, List, Optional, Tuple

# Transport lives in its own module so it can be tested without a printer: a
# threaded serial reader, the newline-JSON protocol and the parsing of the
# AMS's own narration. Re-exported here because configs, tests and any other
# caller have always imported these from AFC_BambuAMS.
try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_BambuAMS_bridge import (      # noqa: F401  (re-export)
    BambuBridge,
    parse_bridge_line,
    _CHMB_STATE_RE,
    _AMS_HUMAN,
    _RFID_INFLIGHT_RE,
    _AMS_NOISE_RE,
    _ams_is_noise,
    _BRIDGE_EVENTS_KNOWN,
    _BLDC_I_RE,
    )
except: raise error(ERROR_STR.format(import_lib="AFC_BambuAMS_bridge", trace=traceback.format_exc()))

try:
    from extras.AFC_unit import afcUnit
except Exception:                        # allow import under unit tests
    afcUnit = object                     # type: ignore

try:
    from configfile import error as config_error
except Exception:                        # pragma: no cover - klipper runtime only
    config_error = Exception             # type: ignore

try:
    from extras.AFC_lane import AFCLaneState
except Exception:                        # allow import under unit tests
    class AFCLaneState:                   # type: ignore
        """Fallback lane-state constants when AFC_lane can't be imported
        (unit tests import this module without a Klipper runtime)."""
        NONE = 0
        ERROR = "Error"
        LOADED = "Loaded"
        TOOLED = "Tooled"
        TOOL_LOADED = "Tool Loaded"
        TOOL_LOADING = "Tool Loading"
        TOOL_UNLOADING = "Tool Unloading"
        EJECTING = "Ejecting"

try:
    from extras.AFC_RFID import apply_filament_defaults, build_filament_name
except Exception:                        # AFC_RFID is OPTIONAL (Spoolman path)
    apply_filament_defaults = None       # type: ignore
    build_filament_name = None           # type: ignore


#: Bambu's own filament brand. Every tag an AMS reads is a Bambu spool -- the
#: reader is keyed to their tags and returns nothing for anyone else's -- so
#: the brand is known without being on the wire.
BAMBU_BRAND = "Bambu"

#: Bambu names a filament as "<material> <variant>": "PLA Matte", "PLA Basic",
#: "PETG HF", "ABS". AFC keeps those in separate fields -- `material` drives
#: density and temperature lookups, `sub_type` is the variant Spoolman wants --
#: so the tag string is split rather than dumped whole into `material`.
#: Hyphenated composites ("PLA-CF", "PA6-CF") are ONE material and must not be
#: split, which is why this splits on whitespace only.
def _split_bambu_material(text: str) -> tuple:
    """
    Split a Bambu tag material string into (material, sub_type).

    :param text: the tag's material string, e.g. "PLA Matte"
    :return tuple: (material, sub_type); sub_type is "" when there is no
        variant, and the whole string is returned as the material.
    """
    parts = (text or "").split()
    if not parts:
        return "", ""
    return parts[0], " ".join(parts[1:])


SLOTS_PER_UNIT = 4

# Bambu's bus limit: up to 4 four-slot AMS (AMS1/AMS2) + up to 8 single-slot AMS
# HT = 12 units total. Each physical AMS is one AFC unit with a distinct ams_index
# in 0..MAX_AMS-1. Must match the firmware's MAX_AMS.
MAX_AMS = 12
MAX_AMS_4SLOT = 4          # never more than 4 four-slot AMS (AMS1/AMS2)
MAX_AMS_HT = 8             # up to 8 AMS HT

# Default drying heater ceiling (C) -- the AMS2 Pro element is rated to 65 C. A
# unit with a hotter heater (the AMS HT dries to 85 C) raises it per-unit via the
# `dry_max_temp` config. A higher requested TEMP is clamped, not rejected.
# Absolute upper bound accepted for `dry_max_temp` -- no Bambu drying heater
# exceeds this; it guards a fat-fingered config, not a real device limit.
DRY_TEMP_HARD_MAX = 85

# AMS type -> (has_heater, dry_dev_addr, dry_ams_id, dry_max_temp). Set the type
# per unit with `ams_model` so ONE setting picks the right behaviour + addressing:
#   ams1 -- regular AMS / AMS-lite: NO heater (drying commands rejected)
#   ams2 -- AMS2 Pro: heater, drying at device 0x0700, 65 C ceiling
#   ht   -- AMS HT:   heater, drying at device 0x1800, 85 C ceiling (single slot)
# The 0x2C drying command is routed by device address (bytes[7:8]); the id byte
# is the unit's chain index on OUR bridge (ams_index) -- ams_id None = use it.
# Confirmed on the wire: the HT answers drying only at 0x1800 with id=chain index
# (0x0700 and id=0x80 both got no ack); the AMS2 Pro uses 0x0700. See
# docs/captures/ams_ht_drying.txt. `heater:`/`dry_max_temp:` still override.
# Drying ceiling for a boxed AMS / AMS 2 Pro. Referenced from the test suite as
# the model default, so it is NOT unused despite having no in-module callers.
MAX_DRY_TEMP_C = 65

# MC POLL addressing, decoded from live printer<->AMS bus captures with ONE
# unit on the wire. device = the unit CLASS, id = which unit on the wire, so an
# AMS 1 and an AMS 2 are indistinguishable by address alone -- which is why the
# model comes from config and is never probed.
#
#   model -> (mc_dev_addr, mc_id_base)   id = mc_id_base | chain index
#
# The id is how two units of the SAME class share a wire: each is addressed by
# its chain position. A boxed AMS has base 0x00, so it is simply the index. The
# single HT we captured answered 0x80 at index 0, so the HT's base is 0x80 and
# a chained HT is expected at 0x81, 0x82... -- the base is the class bit and
# the low bits are the position. NOT verified with two HTs on one wire.
#
# Deliberately SEPARATE from dry_ams_id. The printer addresses an HT's polls
# with id 0x80, but an older on-wire note in this module records the HT
# answering DRYING only at its chain index, with 0x80 drawing no ack. Drying
# works today; the polls do not. So the poll path takes the printer-observed
# value and the dry path is left alone until someone re-tests it deliberately.
_MC_ADDRESSING = {
    "ams1":    (0x0700, 0x00),
    "ams":     (0x0700, 0x00),
    "ams2":    (0x0700, 0x00),
    "ams2pro": (0x0700, 0x00),
    # MEASURED on our own HT, and it overrides the capture reading above.
    # Swept the drain payload at 0x1800 with everything else held still:
    #
    #   P=0x80   352 polls, 0 replies      (what the capture was read as)
    #   P=0x01   134 polls, 0 replies
    #   P=0x00    70 polls, 35 replies, 19 carrying text
    #
    # 0x00 is the only value the HT answers, and the first thing it said was
    # "[AMS_SWITCH]feed to normal, len_det:0.252 m" -- the move-completion
    # narration whose absence has been making every move wait out its
    # deadline. The rule this replaces ("payload 01 is the log drain; 00 has
    # never drawn a reply") was true, but measured on an AMS 2 Pro and then
    # generalised to every model. On an HT it is exactly inverted, so a bus
    # with an HT on it went silent the moment per-unit addressing started
    # sending 0x80 and stopped falling back to the 0x0700 pair.
    "ht":      (0x1800, 0x00),
    "amsht":   (0x1800, 0x00),
}

_AMS_MODELS = {
    #            heater  dev_addr  ams_id  max_temp
    "ams1":    (False,  0x0700,   None,   65),   # regular AMS / lite (no heater)
    "ams":     (False,  0x0700,   None,   65),   # alias
    "ams2":    (True,   0x0700,   None,   65),   # AMS2 Pro
    "ams2pro": (True,   0x0700,   None,   65),   # alias
    "ht":      (True,   0x1800,   None,   85),   # AMS HT
    "amsht":   (True,   0x1800,   None,   85),   # alias
}

_HT_MODELS = ("ht", "amsht")


def check_ams_limits(models: List[str]) -> Optional[str]:
    """
    Validate configured AMS types against Bambu's bus limits: at most
    MAX_AMS_4SLOT four-slot AMS (ams1/ams2), MAX_AMS_HT AMS HT, MAX_AMS total.

    :param models: the ams_model of every [AFC_BambuAMS] unit on the bus
    :return Optional[str]: a description of any breach, or None if within limits
    """
    ht = sum(1 for m in models if m in _HT_MODELS)
    four = len(models) - ht
    problems = []
    if four > MAX_AMS_4SLOT:
        problems.append(f"{four} four-slot AMS (ams1/ams2) exceeds "
                        f"max {MAX_AMS_4SLOT}")
    if ht > MAX_AMS_HT:
        problems.append(f"{ht} AMS HT exceeds max {MAX_AMS_HT}")
    if len(models) > MAX_AMS:
        problems.append(f"{len(models)} total AMS exceeds max {MAX_AMS}")
    return "; ".join(problems) if problems else None


# ── Prep logo (house-style aligned box, matches ACE2/OpenAMS) ───────────────────

def _ams_box_logo(title: str, n_slots: int, name: str) -> str:
    """
    AMS-style unit logo: a titled box with one spool bay per slot, fronted by
    the R/E/A/D/Y banner. ASCII borders so every row lines up.

    :param title: text centered in the box header
    :param n_slots: number of spool bays to draw
    :param name: unit name appended below the box
    :return str: success-styled logo markup
    """
    n = max(1, int(n_slots) if n_slots else 1)
    bay_w = 3
    while n * bay_w + (n - 1) < len(title):
        bay_w += 1
    inner = n * bay_w + (n - 1)
    bar = "-" * bay_w
    spool = "O".center(bay_w)
    rows = [
        "+" + "-" * inner + "+",
        "|" + title.center(inner) + "|",
        "+" + "+".join([bar] * n) + "+",
        "|" + "|".join([spool] * n) + "|",
        "+" + "+".join([bar] * n) + "+",
    ]
    body = "\n".join(f"{banner}  {row}" for banner, row in zip("READY", rows))
    return f"<span class=success--text>{body}</span>\n   {name}\n"


def _ams_box_logo_error(title: str, n_slots: int, name: str) -> str:
    """
    Error variant of the AMS-style logo (red box, ERROR banner).

    :param title: text centered in the box header
    :param n_slots: number of spool bays to size the box for
    :param name: unit name appended below the box
    :return str: error-styled logo markup
    """
    n = max(1, int(n_slots) if n_slots else 1)
    bay_w = 3
    while n * bay_w + (n - 1) < len(title):
        bay_w += 1
    inner = max(n * bay_w + (n - 1), len("X ERROR"))
    rows = [
        "+" + "-" * inner + "+",
        "|" + title.center(inner) + "|",
        "+" + "-" * inner + "+",
        "|" + "X ERROR".center(inner) + "|",
        "+" + "-" * inner + "+",
    ]
    body = "\n".join(f"{banner}  {row}" for banner, row in zip("ERROR", rows))
    return f"<span class=error--text>{body}</span>\n   {name}\n"




def bridge_color_to_rgb(color: Any) -> Optional[str]:
    """
    Normalize the bridge's 0xRRGGBBAA hex color to a 6-digit RRGGBB string.

    :param color: The bridge 'color' field (an 8-hex-digit string)
    :return Optional[str]: uppercase RRGGBB, or None if unusable/zero
    """
    if not isinstance(color, str) or len(color) < 6:
        return None
    try:
        int(color, 16)
    except ValueError:
        return None
    rgb = color[:6].upper()
    return None if rgb == "000000" else rgb


def bridge_slot_to_info(slot: dict) -> dict:
    """
    Map a bridge slot dict to a normalized AFC slot_info dict.

    Bambu's tag is decoded to a PROFILE (material name, Bambu type code / sku,
    color, print temps) — there is no per-spool UID, so rfid_uid is always None.
    This mirrors a base-ACE unit rather than the UID-unique ACE2/U1/Vivid.

    :param slot: One entry from a bridge status frame's 'slots' list
    :return dict: normalized info (present, state, material, sku, color, temps)
    """
    material = slot.get("material") or None
    sku = slot.get("sku") or None

    def _temp(v):
        return v if isinstance(v, int) and v > 0 else None
    return {
        "index": slot.get("i"),
        "present": bool(slot.get("present")),
        "state": slot.get("state") or "empty",
        "material": material,
        "sku": sku,                         # Bambu profile code, e.g. "GFA00"
        "color": bridge_color_to_rgb(slot.get("color")),
        "temp_min": _temp(slot.get("tmin")),
        "temp_max": _temp(slot.get("tmax")),
        "weight": (slot.get("weight") or None) if slot.get("weight") else None,
        "rfid_uid": None,                   # Bambu never exposes a unique UID
    }


def build_slot_map(lanes: Dict[str, Any], slots_per_unit: int) -> Dict[str, int]:
    """
    Map each lane name to its 0-based slot from its 1-based config index.

    :param lanes: The unit's lane objects keyed by name (each has .index)
    :param slots_per_unit: Number of slots this unit exposes
    :return Dict[str, int]: lane name -> 0-based slot index
    :raises ValueError: on an out-of-range or duplicate lane index
    """
    slot_map: Dict[str, int] = {}
    owner: Dict[int, str] = {}
    for name, lane in lanes.items():
        idx = getattr(lane, "index", 0)
        if not 1 <= idx <= slots_per_unit:
            raise ValueError(
                f"lane '{name}' has index {idx}, outside this unit's slots "
                f"1..{slots_per_unit}")
        slot = idx - 1
        if slot in owner:
            raise ValueError(
                f"lanes '{owner[slot]}' and '{name}' both map to slot {slot} "
                f"(index {idx}); each lane needs a unique index")
        owner[slot] = name
        slot_map[name] = slot
    return slot_map


def prep_lane_state(info: dict, tool_loaded: bool, online: bool,
                    fallback_material: Optional[str] = None) -> tuple:
    """
    Compute a lane's PREP-time state from cached bridge slot info.

    Present spool -> prep_state and staged-at-hub; the virtual hub's LIVE
    occupancy comes only from tool_loaded (a merely-staged lane reads clear).

    :param info: The cached slot info dict (from bridge_slot_to_info)
    :param tool_loaded: Whether this lane's filament is threaded to the toolhead
    :param online: Whether the AMS is answering the bridge's master poll
    :param fallback_material: Material to show when the slot's tag hasn't been
        read yet (the AMS HT's read lands ~20s after boot, after PREP prints --
        AFC's saved lane material fills the gap)
    :return tuple: (prep_state, loaded_to_hub, load_state, message)
    """
    present = bool(info.get('present'))
    if present:
        msg = "<span class=success--text>LOCKED AND LOADED</span>"
        mat = info.get('material') or fallback_material
        if mat:
            msg += f" ({mat})"
    else:
        msg = 'EMPTY READY FOR SPOOL'
    if not online:
        msg += (" <span class=warning--text>(AMS offline — bridge "
                "protocol bring-up)</span>")
    return present, present, bool(tool_loaded), msg


def unit_env(latest: Optional[dict], ams_index: int) -> tuple:
    """
    Extract (humidity_pct, temperature_c) for a unit from a bridge status frame.

    The firmware reports humidity 0..100 (%RH) and temp x10 per unit, with -1
    meaning unknown. Ambient temperature is not in the base AMS protocol, so it
    is normally None (reserved for an AMS 2 Pro capture).

    :param latest: A bridge status dict (or None)
    :param ams_index: This unit's AMS number
    :return tuple: (humidity or None, temperature_c or None)
    """
    if not latest:
        return None, None
    for u in latest.get("units") or []:
        if u.get("n") == ams_index:
            h, t = u.get("humidity"), u.get("temp")
            hum = h if isinstance(h, int) and h >= 0 else None
            tmp = round(t / 10.0, 1) if isinstance(t, int) and t >= 0 else None
            return hum, tmp
    return None, None


# Buffer "fullness" thresholds (0..100) -> FPS-style state. 100 = full/compressed
# (fed/satisfied), 0 = stretched/expanded (extruder pulling, demand). Mirrors the
# AFC_buffer compressed/expanded wording so it reads the same in the UI.






# How long after a STOP live chamber telemetry is ignored. The AMS narrates
# roughly every 10s and does not go quiet the instant it is told to stop, so
# this must cover the wind-down or the stop re-adopts itself -- which on
# hardware meant Stop had to be pressed twice. Long enough to be reliable,
# short enough that a stop which genuinely failed still shows up as "still
# drying" quickly.
DRY_STOP_GRACE = 25.0


def _mono(obj: Any) -> float:
    """
    Reactor monotonic time for a unit, or 0.0 where there is no reactor.

    Module-level rather than a method because the heater commands are also
    exercised bound to a stand-in object in tests.

    :param obj: anything that may carry a ``reactor``
    :return float: current monotonic time, or 0.0
    """
    mono = getattr(getattr(obj, "reactor", None), "monotonic", None)
    return mono() if callable(mono) else 0.0




BUFFER_COMPRESSED_AT = 66
BUFFER_EXPANDED_AT = 33


def _buffer_state(buff: Optional[int]) -> Optional[str]:
    """
    Map the AMS FPS fullness (0..100) to an FPS-style state string.

    :param buff: fullness 0..100 (100=compressed/fed, 0=expanded/demand), or None
    :return Optional[str]: "compressed" | "neutral" | "expanded", or None if unknown
    """
    if buff is None:
        return None
    if buff >= BUFFER_COMPRESSED_AT:
        return "compressed"
    if buff <= BUFFER_EXPANDED_AT:
        return "expanded"
    return "neutral"


# ── Virtual FPS-buffer ADC pin ──────────────────────────────────────────────────
# Expose the AMS buffer as a host-registered ADC pin so a *stock* AFC FPS/PSF
# buffer can read it -- no real MCU pin, no wiring, no edits to AFC's buffer code.
# In config you write a normal buffer:  [AFC_buffer <name>] type: FPS_PSF
#                                        adc_pin: bambu_buffer:fps
# and the real AFCFPSBuffer runs (neutral centering, gauge, QUERY_BUFFER, Mainsail
# display) -- fed by the bridge stream instead of silicon. The value is the AMS FPS
# "fullness" 0..1 (1.0 = compressed/full, 0.0 = stretched/demand); matches FPS
# semantics (high = compressed, low = tension), so no `reversed` needed.
_BUFFER_CHIP_NAME = "bambu_buffer"


class _BambuBufferADC:
    """MCU_adc-compatible virtual pin. AFCFPSBuffer drives this exactly as a real
    ADC; the sample is streamed from the AMS bridge. Accepts both the Klipper and
    Kalico ADC setup signatures (they differ, so everything is *args-tolerant)."""

    def __init__(self, chip: "_BambuBufferChip") -> None:
        self._chip = chip
        self._callback: Optional[Callable] = None

    def setup_adc_sample(self, *args: Any, **kwargs: Any) -> None:
        return None

    def setup_minmax(self, *args: Any, **kwargs: Any) -> None:
        return None

    def setup_adc_callback(self, report_time: Any, callback: Any = None) -> None:
        # Klipper: (report_time, cb). Kalico/older: (cb).
        self._callback = report_time if callback is None else callback

    def get_mcu(self) -> Any:
        return self._chip.printer.lookup_object("mcu")

    def push(self, value: float) -> None:
        cb = self._callback
        if cb is not None:
            # Single-arg form: AFCFPSBuffer._adc_callback timestamps it itself.
            cb(value)


class _BambuBufferChip:
    """Host pin chip 'bambu_buffer'. `adc_pin: bambu_buffer:fps` -> a virtual ADC
    fed at report_time from the AMS bridge buffer. One chip per printer; the buffer
    value is global to the bridge so any AFC_BambuAMS unit can source it."""

    def __init__(self, unit: Any, report_time: float = 0.100) -> None:
        self.printer = unit.printer
        self._unit = unit
        self._report_time = report_time
        self._pins: List[_BambuBufferADC] = []
        self._timer = None
        self.printer.lookup_object("pins").register_chip(_BUFFER_CHIP_NAME, self)
        self.printer.register_event_handler("klippy:ready", self._start)

    def setup_pin(self, pin_type: str, pin_params: dict) -> _BambuBufferADC:
        if pin_type != "adc":
            raise self.printer.config_error(
                "bambu_buffer only provides 'adc' pins "
                "(use adc_pin: %s:fps)" % _BUFFER_CHIP_NAME)
        adc = _BambuBufferADC(self)
        self._pins.append(adc)
        return adc

    def _start(self, *args: Any) -> None:
        if self._timer is None:
            reactor = self.printer.get_reactor()
            self._timer = reactor.register_timer(self._update, reactor.NOW)

    def _update(self, eventtime: float) -> float:
        v = self._unit.fps_buffer_value()
        if v is not None:
            for adc in self._pins:
                adc.push(v)
        return eventtime + self._report_time


def _register_bambu_buffer_chip(unit: Any) -> None:
    """Register the shared bambu_buffer ADC chip once (first AFC_BambuAMS unit)."""
    printer = unit.printer
    if getattr(printer, "_bambu_buffer_chip", None) is not None:
        return
    printer._bambu_buffer_chip = _BambuBufferChip(unit)


# The AMS meters its own moves. Neither the mm nor the mm/s we send ever
# controls one -- bb_feed() in the firmware converts both into a runaway
# deadline, and _wait_move does the same on this side. These are therefore NOT
# commanded speeds; they are the nominal figures used to turn a distance into a
# watchdog, and there is nothing for an operator to tune in them. They are
# module constants rather than config options for that reason.
NOMINAL_MMPS = 20.0
MAX_MMPS = 30.0

# AMS bay -> hub staging distance. Fixed, not configurable: the hub here is
# virtual (the AMS multiplexes internally, there is no switch to reach) so
# there is nothing for an operator to measure, and like the bowden length it
# only sizes a deadline. 250mm is comfortably past any real bay-to-hub run.
DIST_HUB_MM = 250.0

# Default hub -> toolhead distance until the unit reports its own. Long on
# purpose: this sizes the load give-up deadline, and a first load has to be
# allowed to finish so the AMS can measure its path and write the real value
# back. A short default would abort that first load and the unit would never
# calibrate.
DEFAULT_BOWDEN_MM = 3000.0

# How far the AMS's measurement must sit from the configured value before
# it is worth rewriting the file. The unit's own figure moves a few mm
# between calibrations and none of this needs that precision.
PATH_ADOPT_TOLERANCE_MM = 25.0

#: Minimum seconds between automatic bridge reboots. The reset drops the USB
#: link, which fires the reconnect handler, which would reset again -- so this
#: is what stops a reboot loop, not politeness.
BRIDGE_RESET_COOLDOWN_S = 60.0

#: How long to let a freshly-connected Pico settle before announcing to it.
#:
#: Measured: a MANUAL BAMBU_RESTART (issued from a long-settled Klipper) fixes
#: the log-drain addressing every time -- dbg_texts moves off 0 and the
#: frames/polls ratio flips from 1:2 (the 0x0700 fallback) to ~1:1 (per-unit).
#: The IDENTICAL reset issued automatically on connect does not: the counters
#: reset but the ratio stays at the fallback and dbg_texts stays 0. Same
#: commands, same order -- the only difference is that the automatic path
#: announces the instant the USB CDC endpoint reopens, which is before the
#: firmware has finished coming up and can accept `mcaddr`. The announce is
#: fire-and-forget, so a dropped one is silent and permanent until the next
#: reconnect.
ANNOUNCE_SETTLE_S = 1.0

#: AMS mode values, decoded from live bus captures: a feed runs 0 idle ->
#: 2 feeding -> 3 feed done -> 4 following -> 1 assist. Reported as `fstate` in
#: every status frame. The full set is named so the field is readable in a
#: status dump; only IDLE and FOLLOWING are acted on.
AMS_MODE_IDLE = 0
AMS_MODE_ASSIST = 1
AMS_MODE_MOVING = 2
AMS_MODE_DONE = 3
AMS_MODE_FOLLOWING = 4

#: Hard ceiling on any single move's watchdog, in seconds.
#:
#: 35 s is measured: the AMS moves at ~136 mm/s, so a full 3.5 m path takes
#: about 25 s. This is the fallback for when the completion report does not
#: arrive, and returning early is cheap -- the caller simply proceeds -- so it
#: is sized just past a real move rather than generously.
#:
#: A hard ceiling is needed because the per-move deadline is derived from
#: distance / NOMINAL_MMPS, so a generous bowden default scales every fallback
#: wait with it. Without the cap a 3000 mm default puts a toolhead unload at a
#: 330 s worst case, leaving AFC in "Tool Unloading" long after the filament
#: has physically stopped.
#:
#: The deadline is a RUNAWAY GUARD, not a schedule -- the AMS meters its own
#: move and normally ends it by reporting a completion, so this only ever
#: matters when that report does not arrive.
MOVE_DEADLINE_MAX_S = 35.0

#: Ceiling on the load-to-sensor window -- a DIFFERENT quantity from the move
#: watchdog above, and not to be conflated with it.
#:
#: The AMS runs its own load routine: feed, stall at the extruder, retract,
#: retry, several times over before it gives up. Our job during that is to
#: nudge it and catch the sensor, NOT to interrupt it -- so this window has to
#: cover the bulk feed plus a handful of the unit's own retry cycles. Clamped
#: to MOVE_DEADLINE_MAX_S it would sit below load_retry_timeout alone and
#: truncate every attempt mid-cycle.
#:
#: Only a runaway guard, hence generous: the loop exits the moment the sensor
#: triggers, so this costs nothing on a load that works.
LOAD_SENSOR_MAX_S = 180.0

#: mm/s used ONLY to turn a distance into a watchdog deadline. Deliberately
#: separate from NOMINAL_MMPS, which is what goes on the wire: the commanded
#: speed is advisory (the AMS moves at its own rate) but changing it would
#: change what we transmit, and there is no reason to.
#:
#: Measured on hardware rather than guessed. From a real HT unload: the cut
#: completed at t=73.6 s and "[AMS_SWITCH]pull finish 0 ... len_det:3.531 m"
#: landed 25.9 s later -- 3531 mm in 25.9 s, about 136 mm/s. NOMINAL_MMPS is
#: 20, so every deadline was sized as if the AMS were seven times slower than
#: it is, which is what let a 3250 mm retract compute a 330 s wait.
#:
#: Set well under the measured rate on purpose. This is a runaway guard, so
#: erring slow errs safe -- it waits longer before crying stuck. At 60 mm/s a
#: full 3250 mm move allows 113 s against a real ~24 s.
DEADLINE_MMPS = 60.0


def _gcmd_int(gcmd: Any, name: str, default: int,
              minval: int, maxval: int) -> int:
    """
    Read an integer parameter that may be written in hex.

    Klipper's own get_int is decimal-only, so ADDR=0x0700 fails to parse --
    while every address in this file's docs, comments and captures is written
    in hex, because that is how the bus documents itself. Typing the
    documented value and having it rejected is a papercut on exactly the
    diagnostic commands reached for when something is already wrong.

    Accepts 0x/0b/0o prefixes and plain decimal (int(x, 0)).

    :param gcmd: The Klipper GCodeCommand
    :param name: Parameter name
    :param default: Value when the parameter is absent
    :param minval: Lowest accepted value, inclusive
    :param maxval: Highest accepted value, inclusive
    :return int: the parsed value
    """
    # Klipper's own parse FIRST, so decimal input keeps its exact behaviour
    # (including Klipper's range errors) and only a value it rejects -- which
    # is what a hex literal is -- reaches the fallback below.
    try:
        return gcmd.get_int(name, default, minval=minval, maxval=maxval)
    except Exception:
        pass
    try:
        raw = gcmd.get(name, None)
    except Exception:
        raw = None
    if raw is None:
        return default
    try:
        val = int(str(raw).strip(), 0)
    except (TypeError, ValueError):
        raise gcmd.error(
            f"Error on '{gcmd.get_commandline()}': unable to parse {raw}")
    if val < minval or val > maxval:
        raise gcmd.error(
            f"Error on '{gcmd.get_commandline()}': {name}={raw} is outside "
            f"{minval}..{maxval}")
    return val


def clamp_speed(mmps: float, ceiling: float) -> float:
    """
    Clamp a requested speed into (0, ceiling].

    :param mmps: Requested speed in mm/s
    :param ceiling: Maximum allowed speed in mm/s
    :return float: the clamped speed
    """
    if mmps <= 0:
        return ceiling
    return min(mmps, ceiling)




# One BambuBridge per serial port, shared by all units on that Pico (a
# daisy-chained AMS shows up as several AFC units on one bus / one bridge).
_BRIDGES: Dict[str, "BambuBridge"] = {}


# ── AFC unit ────────────────────────────────────────────────────────────────────

class afcBambuAMS(afcUnit):
    """
    AFC unit for a stock Bambu AMS behind the Pico Bambu-Bus bridge.

    Mirrors the bridge's per-slot status onto AFC lanes and issues transport as
    bridge commands. Deeper AFC load/unload orchestration is marked TODO where it
    must bind to the base unit's sequencing.
    """

    SLOTS_PER_UNIT = SLOTS_PER_UNIT

    # Hard ceiling on how long _finalize_scan will keep deferring to a read that
    # is still narrating. Only reached by a unit that narrates read steps
    # continuously without ever finishing; a normal read is well inside it and a
    # unit that gives up goes quiet and falls back on the base window.
    SCAN_FALLBACK_CAP = 45.0

    #: Alias of the module constant, for discoverability on the class.
    DRY_STOP_GRACE = DRY_STOP_GRACE

    def __init__(self, config: Any) -> None:
        """
        :param config: Klipper ConfigWrapper for this [AFC_BambuAMS] section
        """
        super().__init__(config)
        self.type = config.get("type", "BambuAMS")
        self.serial_port = config.get("serial_port")
        self.baud = config.getint("baud", 115200)
        # These lanes have no drive stepper — the AMS motors do the feeding, so
        # AFC_lane.move_to routes moves to our lane_move() (bridge feed/retract).
        self.stepperless_drive = True
        # Load/unload transport distances (mm). The virtual hub has no switch, so
        # these are commanded open-loop; the toolhead sensor confirms the load.
        #   afc_bowden_length        : hub -> toolhead feed distance
        #   afc_unload_bowden_length : toolhead -> hub retract distance
        #
        # The default is DEFAULT_BOWDEN_MM, deliberately long. These distances
        # size the load give-up deadline, and undershooting is the harmful
        # direction: a value under the real path kills good loads part-way,
        # while a long one only delays reporting a jam. A new user therefore
        # gets enough rope for a first load to complete, during which the unit
        # measures its own path and the real figure is written back here.
        self.afc_bowden_length = config.getfloat(
            "afc_bowden_length", DEFAULT_BOWDEN_MM, above=0.0)
        self.afc_unload_bowden_length = config.getfloat(
            "afc_unload_bowden_length", self.afc_bowden_length, above=0.0)
        # Post-feed load recovery: pulse this many mm at a time, up to this many
        # seconds, to nudge filament onto the toolhead sensor if the main feed
        # under-shot it (mirrors AFC_ACE load_retry_pulse/timeout).
        self.load_retry_pulse = config.getfloat(
            "load_retry_pulse", 100.0, minval=1.0)
        # Total patience for the load to reach the toolhead sensor. The AMS has
        # its OWN load state machine that retries several times (feed -> stall at
        # the extruder -> retract -> retry), so give it room to run through those
        # attempts before we call it a failure. Default generous.
        self.load_retry_timeout = config.getfloat(
            "load_retry_timeout", 40.0, minval=1.0)
        # How often WE re-kick the feed while waiting. Kick too often and each
        # push resets the AMS mid-retry (it never completes its own attempt);
        # kick too rarely and a genuinely-idle AMS sits still. Spaced to let one
        # AMS stall-retry cycle finish between our nudges.
        self.load_retry_interval = config.getfloat(
            "load_retry_interval", 4.0, minval=0.5)
        # On a load that never reaches the sensor, whether to reel the filament
        # back into the bay (AMS multi-stage unwind) before reporting the error.
        # Default False: the AMS keeps retrying loads on its own, so yanking the
        # filament back mid-retry fights it -- leave it staged and let it try
        # again (or intervene manually). Enable if you want an auto-clean path.
        self.reel_back_on_load_fail = config.getboolean(
            "reel_back_on_load_fail", False)
        # When the AMS exhausts its own load retries and stalls (state:7), run the
        # printer's "Retry": a re-home reset (mode 0F/0E) then re-feed. This many
        # reset+retry cycles before reporting the failure. 0 disables (old
        # behaviour: fail straight to handle_lane_failure). Sniffed from a real
        # printer -- fail -> re-home -> miss -> big re-home -> success.
        self.load_recover_attempts = config.getint(
            "load_recover_attempts", 2, minval=0)
        # Let the AMS's OWN arrival report finish a load when the toolhead
        # sensor does not. The sensor stays FIRST -- it is checked on every
        # pass of the feed loop and this is only consulted after -- so on a
        # sensored, calibrated lane it never fires (measured: the sensor
        # triggers 1-2 s ahead on both a boxed AMS and an HT). What it buys is
        # a lane with NO toolhead sensor being loadable at all, and a failed
        # sensor degrading to a correct completion instead of a silent
        # timeout followed by re-homing.
        self.ams_arrival_completes_load = config.getboolean(
            "ams_arrival_completes_load", True)
        # Extra mm (beyond bowden + DIST_HUB_MM) to retract when ejecting a lane so
        # the filament clears the hub and pulls fully back into the AMS bay.
        self.eject_buffer = config.getfloat("eject_buffer", 200.0, minval=0.0)
        # One firmware drives both the boxed AMS and the AMS-lite; it
        # auto-detects by default, or pin it here (auto|ams|lite).
        self.variant = config.getchoice(
            "variant", {"auto": "auto", "ams": "ams", "lite": "lite"}, "auto")
        # Which AMS on the daisy-chain this unit represents (0..3). Multiple
        # [AFC_BambuAMS] units on the same serial_port share one bridge.
        self.ams_index = config.getint("ams_index", 0, minval=0,
                                       maxval=MAX_AMS - 1)
        # Optional: pin this unit to its physical AMS by UID (12-byte hex from the
        # bridge `chain` command). The firmware assigns chain indices by ANNOUNCE
        # order, which reshuffles across power-cycles -- so a fixed ams_index can
        # end up talking to the wrong unit. With unit_uid set, on connect we look
        # up which chain index currently carries that UID and use THAT as
        # ams_index, so the mapping is stable no matter what order units boot.
        self.unit_uid = (config.get("unit_uid", "") or "").strip().upper() or None
        # Auto-trigger an RFID/tag scan when a spool is newly inserted into a bay
        # (AMS presence bitmap 0->1), so material/color populate without a manual
        # AMS_SCAN. A scan moves filament, so it's gated to the idle/not-printing
        # state and latched to fire once per insertion.
        self.auto_scan = config.getboolean("auto_scan", True)
        # Seconds to wait after an insert edge before triggering the scan. Only
        # used by the AMS HT (device 0x1800), which reads its RFID on its preload
        # switch -- the delay lets the spool settle in the bay so the 0x1800 read
        # lands. Boxed AMS scan immediately regardless of this value.
        # Demand-gated follower re-engage. The AMS holds its self-centering
        # follower (mode:4) only while there's demand; on our bus it drops to
        # idle (state:0) when the buffer is centred and does NOT re-engage from
        # the AP2 stream alone. Rather than blindly re-arm every poll (which
        # pokes the feeder at idle -- the "twitch"), we watch the toolhead
        # extruder and re-send the feeder select ONLY when it actually advances
        # (real extrusion demand). Silent at idle, engages the instant you feed.
        #   follow_poll_interval : how often to sample the extruder (s)
        #   follow_min_extrude   : mm of extrusion since last re-engage to fire
        self.follow_poll_interval = config.getfloat(
            "follow_poll_interval", 0.1, above=0.0)
        self.follow_min_extrude = config.getfloat(
            "follow_min_extrude", 0.1, above=0.0)
        # When True, keep the follower armed continuously while a lane is loaded
        # (the firmware re-arm never lapses) -- the "always on" behaviour that
        # reliably let the extruder pull, at the cost of a small idle twitch.
        # When False (default), only stay armed while the toolhead is actually
        # extruding, so it's silent at idle.
        self.follow_always = config.getboolean("follow_always", False)
        # Keep the follower auto-armed whenever a lane on this unit is threaded to
        # the toolhead ("always ready when loaded"). This bypasses two signals
        # that are unreliable here: the AMS buffer readback (the firmware often
        # reports a stuck default, so the demand trigger never fires) and the
        # per-lane extruder match (extruder_obj can be unset, so the
        # extrude-detect ping never fires). On tool-load it engages mode:4 and
        # holds the firmware re-arm window open every tick, so the AMS feeds at
        # its own steady cadence and the extruder cannot bottom the buffer out
        # before it feeds. The AMS still self-limits at its buffer centre, so it
        # cannot over-feed. Default on; set False to gate on extrusion instead.
        self.follow_when_loaded = config.getboolean("follow_when_loaded", True)
        #   follow_idle_ping : keep the firmware's feed window open whenever a
        # lane is loaded, instead of only while the extruder advances.
        #
        # Defaults False: the window now opens on real extrusion, so the lane
        # still arms into mode:4 and stays ready, but nothing is poked while the
        # printer sits idle. A real printer behaves this way -- its AMS ticks
        # constantly WHILE printing (90 motor transitions in one extrude
        # capture) and is silent otherwise.
        #
        # It defaulted True while three faults made demand-gating impossible:
        # the buffer decode read permanently "feed me", the follower arm sat in
        # the 300ms status poll, and the follower's select reset the odometer.
        # With those fixed, holding the window open is just an idle tick.
        # Set True to force continuous feeding if a unit needs it.
        self.follow_idle_ping = config.getboolean("follow_idle_ping", False)
        #   fault_detect : act on the AMS's own stall reports ("feed finish -1,
        # stall", "rocker stall", "bdc stall"). The unit names these itself, so
        # this is a report, not an inference.
        self.fault_detect = config.getboolean("fault_detect", True)
        #   fault_pause : pause the print on a stall. Off leaves it a warning.
        self.fault_pause = config.getboolean("fault_pause", True)
        #   fault_starved_below : buffer fullness under which the AMS is not
        # keeping up. Measured on an AMS 2: 51-59 throughout a print, and the
        # AMS's own narration puts the floor of a healthy follower cycle at
        # pos:0.08-0.10 (mapped 8-10). A held stuck spool sat at 3. So 25 is
        # clear of both the working range above and the stall floor below.
        self.fault_starved_below = config.getint(
            "fault_starved_below", 25, minval=0, maxval=100)
        #   fault_starved_seconds : how long it must stay there, WHILE the
        # extruder is advancing, before this is called a fault. Bottomed out
        # for this long is not a dip -- a healthy cycle refills in well under a
        # second -- and the AMS pulls HARD while stalled, so a long window just
        # means longer grinding on jammed filament before anything stops it.
        self.fault_starved_seconds = config.getfloat(
            "fault_starved_seconds", 2.0, above=0.5)
        self._starved_since: float = 0.0
        self._starved_e: float = 0.0
        self._starved_reads: Optional[int] = None
        # Last fault sequence handled, so one stall raises one error.
        self._fault_seen: int = 0
        # Only the AMS2 Pro has a drying heater. Default true; set `heater: false`
        # in the config for AMS1 / AMS-lite units so BAMBU_HEATER_START just says
        # so instead of sending a drying command a heaterless unit ignores.
        # AMS TYPE -- one setting picks heater on/off, drying device address, and
        # temp ceiling (see _AMS_MODELS): `ams1` (regular AMS, no heater), `ams2`
        # (AMS2 Pro), `ht` (AMS HT). This is how you tell each unit apart so it
        # uses the right addressing. `heater:` and `dry_max_temp:` override the
        # type's defaults if you ever need to.
        self.ams_model = config.get("ams_model", "ams2").strip().lower()
        #   self_centres : does this unit refill its own buffer? An AMS 2 Pro
        # and an AMS HT do; a regular AMS does not and must be fed whenever the
        # extruder pulls. Drives the firmware's feeder deadband -- see
        # _send_selfcentre_flag. Overridable if a model turns out to differ.
        self.self_centres = config.getboolean(
            "self_centres", self.ams_model not in ("ams1", "ams"))
        # follow_always holds the firmware's extrusion window permanently open.
        # On a self-centring unit that is harmless -- the feeder poke is
        # deadbanded, so it still only fires when the buffer has actually
        # sagged. On a unit that is NOT self-centring the poke is demand-only
        # (OR), so an always-open window makes it fire every REARM_MS forever:
        # ~7 motor commands a second with the printer idle, which is audible as
        # a continuous tick and stops the instant klippy does.
        if self.follow_always and not self.self_centres:
            self.logger.warning(
                f"AFC bambu {self.name}: follow_always is on for a unit that "
                f"does not self-centre ({self.ams_model}) -- this pokes the "
                f"feeder continuously and ticks at idle. Set "
                f"follow_always: False unless you are deliberately testing.")
        _spec = _AMS_MODELS.get(self.ams_model, _AMS_MODELS["ams2"])
        _model_heater, self.dry_dev_addr, self.dry_ams_id, _dry_default_max = _spec
        self.has_heater = config.getboolean("heater", _model_heater)
        # The drying id byte follows the unit's chain index (ams_index) unless the
        # type pins it. Track it so UID-pinning can update the id if the index is
        # re-resolved from the UID on connect.
        self._dry_id_follows_index = self.dry_ams_id is None
        # Real bay count for THIS unit: the AMS HT has a single slot, the 4-slot
        # models have four. Internal arrays stay SLOTS_PER_UNIT-sized for
        # uniformity, but everything user-visible (PREP logo, scans, insert
        # logging) is clamped to unit_slots so a 1-slot HT never shows or scans
        # phantom bays.
        self.unit_slots = 1 if self.ams_model in _HT_MODELS else SLOTS_PER_UNIT
        # MC poll addressing (see _MC_ADDRESSING). mc_ams_id None -> chain index.
        _mc = _MC_ADDRESSING.get(self.ams_model, (0x0700, 0x00))
        self.mc_dev_addr = config.getint("mc_dev_addr", _mc[0],
                                         minval=0, maxval=0xFFFF)
        self.mc_id_base = config.getint("mc_id_base", _mc[1],
                                        minval=0, maxval=0xFF)
        # Explicit override wins outright; -1 means "derive from base|index".
        self.mc_ams_id = config.getint("mc_ams_id", -1, minval=-1, maxval=0xFF)
        if self._dry_id_follows_index:
            self.dry_ams_id = self.ams_index
        self.dry_max_temp = config.getint(
            "dry_max_temp", _dry_default_max, minval=1, maxval=DRY_TEMP_HARD_MAX)
        self._following_lane: Optional[Any] = None
        # True while unit_unload_lane / eject_lane reels filament back. The
        # follower keep-alive tick's auto-arm MUST stand down for the duration:
        # its select+assist re-engage (fired because the lane is still
        # tool_loaded mid-unload) makes the bridge cancel the retract stream
        # (assist-on sets s_motion=0 in the firmware) — the "unload did
        # nothing" race.
        self._unload_in_progress = False
        # True while an AMS drying cycle is running (BAMBU_HEATER_START..STOP).
        # The firmware already holds the follower off during drying so the AMS
        # can run its self-check/vent doors; mirror that here so the module's
        # keep-alive tick stops pumping follow/select frames the firmware would
        # only drop, and so get_status reflects the drying state.
        self._drying: bool = False
        # Chamber telemetry must be NEWER than this to say anything about the
        # current cycle. A start sets it to now (the very next reading counts, so
        # the panel leaves "Starting" as soon as the unit speaks); a stop sets it
        # to now + DRY_STOP_GRACE, because a stopping AMS emits another line or
        # two while winding down and those must not resurrect the cycle just
        # ended. 0.0 at boot, so a dry already running when Klipper starts is
        # still adopted -- which is the whole point of adoption.
        self._dry_adopt_after: float = 0.0
        # Whether THIS cycle has ever reported chamber telemetry. Required
        # before a silence can be read as "the cycle ended": a freshly started
        # dry has not reported yet and must not be released as finished.
        self._dry_seen_live: bool = False
        self._follow_last_e: Optional[float] = None
        self._follow_timer = None
        self._uid_watch_timer = None
        self._follow_last_log: float = 0.0
        # Log the AMS buffer position + follower state while following. Off by
        # default (0) -- the follower is watchable live via get_status
        # (follow_buff/follow_state/following), so this is only for deep tuning.
        # When >0 it rate-limits AND only logs when the values actually change, so
        # even enabled it never streams a line every tick.
        self.follow_debug_interval = config.getfloat(
            "follow_debug_interval", 0.0, minval=0.0)
        self._follow_last_dbg: Optional[tuple] = None
        # Last status-apply failure text. Status frames arrive continuously, so
        # a stuck fault would warn on every frame; only a CHANGED message is
        # worth the console.
        self._status_err_last: Optional[str] = None
        # Latched by BAMBU_FOLLOWER ENABLE=0. The latch is what holds the AMS
        # out of mode:4 to work on it: without it the auto-arm below re-engages
        # on the next ~100ms tick and undoes the manual stop. Cleared by
        # ENABLE=1 or by the next load, so it cannot strand a print.
        self._follow_manual_off: bool = False
        #   follow_rearm_window : how long after real extrusion a dropped
        # follower is still worth re-arming. state:0 is the AMS's RESTING state,
        # not a fault -- it arms, finishes its assist within a second or two,
        # and reports 0 until something asks it for filament again. Re-arming on
        # state alone therefore never settles: arm -> "assist finish 0" ->
        # state:0 -> re-arm, every couple of seconds forever, each one an LED
        # flash and a motor nudge. Set it once, then only re-set it if it has
        # dropped AND the extruder actually wants filament.
        self.follow_rearm_window = config.getfloat(
            "follow_rearm_window", 3.0, above=0.0)
        self._follow_last_demand: float = 0.0
        # Latched when a stall pauses the print. Re-arming the follower against
        # a jam just makes the AMS grind on filament it cannot move, so the
        # auto-arm holds off until the print resumes (see _fault_hold_active).
        self._follow_fault_hold: bool = False
        self._follow_fault_saw_pause: bool = False
        self._slot_map: Dict[str, int] = {}
        self._bridge: Optional[BambuBridge] = None
        self._slots: List[dict] = [{} for _ in range(self.SLOTS_PER_UNIT)]
        self._prev_present: List[bool] = [False] * self.SLOTS_PER_UNIT
        self._auto_scanned: List[bool] = [False] * self.SLOTS_PER_UNIT
        # False until the startup presence baseline is recorded, so spools
        # already inserted at boot don't fire a scan (see _maybe_auto_scan).
        self._scan_primed: bool = False
        # PREP renders these after testing each lane; set early so they always
        # exist no matter which handler runs first.
        self.logo = self._make_logo(error=False)
        self.logo_error = self._make_logo(error=True)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:disconnect",
                                            self._handle_disconnect)
        # Expose the AMS buffer as an 'bambu_buffer' ADC pin so a stock AFC FPS/PSF
        # buffer can read it (adc_pin: bambu_buffer:fps). Best-effort: a buffer is
        # optional, so never let its registration break unit init.
        try:
            _register_bambu_buffer_chip(self)
        except Exception as e:
            self.logger.warning(f"AFC bambu {self.name}: buffer pin unavailable: {e}")
        # User-facing commands, mux'd by UNIT= (matches AFC_ACE). BAMBU_FOLLOWER
        # lets you engage/stop the self-centering follower (mode:4) for a loaded
        # lane by hand -- both a manual workaround and a test hook to watch the
        # LED react to the exact select+assist sequence the load path uses.
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "BAMBU_FOLLOWER", "UNIT", self.name, self.cmd_BAMBU_FOLLOWER,
            desc="Engage/stop the AMS self-centering follower (mode:4) for a "
                 "loaded lane. BAMBU_FOLLOWER UNIT=<unit> LANE=<lane> [ENABLE=1]")
        self.gcode.register_mux_command(
            "BAMBU_RECOVER", "UNIT", self.name, self.cmd_BAMBU_RECOVER,
            desc="Recover a stuck/failed load: relink the AMS, stop motion, reel "
                 "the lane's filament back to the bay, and reset its state. "
                 "BAMBU_RECOVER UNIT=<unit> LANE=<lane>")
        self.gcode.register_mux_command(
            "BAMBU_RELINK", "UNIT", self.name, self.cmd_BAMBU_RELINK,
            desc="Force an AMS relink / error-recovery reset (deregister + "
                 "re-register) to clear a TIMEOUT/error state without a power "
                 "cycle. BAMBU_RELINK UNIT=<unit>")
        self.gcode.register_mux_command(
            "BAMBU_SCAN", "UNIT", self.name, self.cmd_BAMBU_SCAN,
            desc="Trigger an RFID/tag scan on demand -- the same read the "
                 "auto-scan runs on a fresh insert. BAMBU_SCAN UNIT=<unit> "
                 "[LANE=<lane>] (no LANE = every slot on the unit). Use it on "
                 "the AMS HT, whose tag only reads when polled at 0x1800.")
        # AMS2 Pro / AMS HT heater drying (protocol from
        # docs/captures/ams2_drying.txt). Registered directly under the nice
        # names -- no cfg macro needed, same as BAMBU_FOLLOWER/RECOVER/RELINK
        # above. Do NOT also define a [gcode_macro Bambu_Heater_Start]: Klipper
        # upper-cases macro names, so it would register BAMBU_HEATER_START and
        # collide with this command ("already registered").
        self.gcode.register_mux_command(
            "BAMBU_MUTE", "UNIT", self.name, self.cmd_BAMBU_MUTE,
            desc="Suppress bridge transmitters to find what a unit reacts to. "
                 "BAMBU_MUTE UNIT=<unit> MASK=<bits> (0 = restore all)")
        self.gcode.register_mux_command(
            "BAMBU_RESTART", "UNIT", self.name, self.cmd_BAMBU_RESTART,
            desc="Reboot the Pico bridge into this same firmware, clearing all "
                 "its runtime state. BAMBU_RESTART UNIT=<unit>")
        self.gcode.register_mux_command(
            "BAMBU_ARMMS", "UNIT", self.name, self.cmd_BAMBU_ARMMS,
            desc="Set the 11/04 follower keep-alive cadence in ms (0 = "
                 "default). The one transmitter BAMBU_MUTE cannot silence. "
                 "BAMBU_ARMMS UNIT=<unit> MS=<ms>")
        self.gcode.register_mux_command(
            "BAMBU_HTID", "UNIT", self.name, self.cmd_BAMBU_HTID,
            desc="Set the id used on an AMS HT's 0x1800 commands. "
                 "BAMBU_HTID UNIT=<unit> ID=<0-255>")
        self.gcode.register_mux_command(
            "BAMBU_DRAIN", "UNIT", self.name, self.cmd_BAMBU_DRAIN,
            desc="Set the 1A/02 log-drain payload byte (255 = default). "
                 "BAMBU_DRAIN UNIT=<unit> P=<byte>")
        self.gcode.register_mux_command(
            "BAMBU_HB", "UNIT", self.name, self.cmd_BAMBU_HB,
            desc="Set the bus heartbeat cadence in ms (0 = default). "
                 "BAMBU_HB UNIT=<unit> MS=<ms>")
        self.gcode.register_mux_command(
            "BAMBU_HTPOLL", "UNIT", self.name, self.cmd_BAMBU_HTPOLL,
            desc="Set the 0x1800 keep-alive cadence in ms (0 = default). "
                 "BAMBU_HTPOLL UNIT=<unit> MS=<ms>")
        self.gcode.register_mux_command(
            "BAMBU_FEED", "UNIT", self.name, self.cmd_BAMBU_FEED,
            desc="Feed a bounded length from a lane's slot. "
                 "BAMBU_FEED UNIT=<unit> LANE=<lane> [MM=20] [SPEED=]")
        self.gcode.register_mux_command(
            "BAMBU_BUFFER_PROBE", "UNIT", self.name, self.cmd_BAMBU_BUFFER_PROBE,
            desc="Dump the raw AMS motion reply + buffer decode state. "
                 "BAMBU_BUFFER_PROBE UNIT=<unit>")
        self.gcode.register_mux_command(
            "BAMBU_HEATER_START", "UNIT", self.name, self.cmd_BAMBU_HEATER_START,
            desc="Start AMS drying (AMS2 Pro / AMS HT). "
                 "BAMBU_HEATER_START UNIT=<unit> [TEMP=55] [TIME=480] [ROTATE=0]")
        self.gcode.register_mux_command(
            "BAMBU_HEATER_STOP", "UNIT", self.name, self.cmd_BAMBU_HEATER_STOP,
            desc="Stop AMS drying. BAMBU_HEATER_STOP UNIT=<unit>")
        # Bus-wide UID list (not per-unit). Reads the UIDs straight off the wire
        # so you can copy them into each unit's `unit_uid`. Registered once;
        # guarded because every daisy-chained unit runs this init.
        try:
            self.gcode.register_command(
                "BAMBU_UIDS", self.cmd_BAMBU_UIDS,
                desc="List the AMS UIDs on the bus (chain index -> UID, with what "
                     "each holds) so you can pin units via unit_uid.")
        except Exception:
            pass        # another [AFC_BambuAMS] on this bus already registered it

        # Register the temperature_bambu sensor factory for [temperature_sensor]
        # sections using sensor_type: temperature_bambu, so an AMS's humidity
        # and drying-chamber temperature show up on the Mainsail/Fluidd
        # temperature card like the OpenAMS and ACE units do.
        try:
            from extras.temperature_bambu import TemperatureBambu
            pheaters = self.printer.load_object(config, "heaters")
            pheaters.add_sensor_factory("temperature_bambu", TemperatureBambu)
            # Fluidd-recognised alias (Fluidd maps it onto the "aht10 <name>"
            # object the sensor registers) so Fluidd's card shows humidity.
            # Distinct from OpenAMS's aht3x and ACE's aht2x so the three
            # factories cannot clobber each other.
            pheaters.add_sensor_factory("aht4x", TemperatureBambu)
            # Fluidd renders humidity only for sensor_types on its own fixed
            # list, and aht4x is not on it -- an AMS registering the very same
            # "aht10 <name>" object as OpenAMS still got a temperature-only
            # card. aht2x and aht3x ARE on it, which is exactly why the ACE and
            # OpenAMS sensors work.
            #
            # Those two already belong to ACE and OpenAMS, so claiming them
            # outright would break those units. Wrap instead: keep the existing
            # factory and dispatch on the section -- a section carrying
            # bambu_unit is ours, anything else goes to whoever had it. Both
            # names, so either works whichever unit is installed.
            for _alias in ("aht3x", "aht2x"):
                _prev = getattr(pheaters, "sensor_factories", {}).get(_alias)

                def _dispatch(cfg, _prev=_prev):
                    if cfg.get("bambu_unit", None) is not None:
                        return TemperatureBambu(cfg)
                    if _prev is not None:
                        return _prev(cfg)
                    return TemperatureBambu(cfg)

                pheaters.add_sensor_factory(_alias, _dispatch)
        except Exception as e:
            logging.info(
                "AFC_BambuAMS: temperature_bambu factory not registered (%s); "
                "use a [temperature_bambu <name>] section instead", e)
    # -- lifecycle --

    def cmd_BAMBU_FOLLOWER(self, gcmd: Any) -> None:
        """
        Manually engage or stop the follower for a lane's AMS tray.

        BAMBU_FOLLOWER UNIT=<unit> LANE=<lane> [ENABLE=1]

        ENABLE=1 (default) runs the finish->select->assist sequence that flips
        the tray to mode:4 and holds it (LED should start flashing); ENABLE=0
        stops the follower (LED goes solid). Use it to verify the follower on a
        tool-loaded lane independent of the load/startup paths.

        :param gcmd: The Klipper GCodeCommand
        """
        lane_name = gcmd.get("LANE")
        enable = gcmd.get_int("ENABLE", 1)
        lane = self.lanes.get(lane_name)
        if lane is None:
            raise gcmd.error(
                f"BAMBU_FOLLOWER: lane '{lane_name}' not on unit {self.name} "
                f"(lanes: {', '.join(self.lanes) or 'none'})")
        if self._bridge is None:
            msg = f"BAMBU_FOLLOWER: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if self._slot_of(lane) is None:
            raise gcmd.error(
                f"BAMBU_FOLLOWER: {lane_name} is not mapped to an AMS slot")
        if enable:
            self._follow_manual_off = False
            self._follow_fault_hold = False
            self._follow_fault_saw_pause = False
            self._engage_follower(lane)
            gcmd.respond_info(
                f"BAMBU_FOLLOWER: engaged follower (mode:4) for {lane_name} on "
                f"{self.name}; LED should flash. If it stays solid, the tray did "
                f"not reach mode:4.")
        else:
            # Latch it off, or the auto-arm re-engages on the next tick and the
            # stop appears to do nothing.
            self._follow_manual_off = True
            self.set_feed_assist(lane, False)
            gcmd.respond_info(
                f"BAMBU_FOLLOWER: stopped follower for {lane_name} on "
                f"{self.name}; LED should go solid. Stays off until "
                f"BAMBU_FOLLOWER ENABLE=1 or the next load.")

    def cmd_BAMBU_RECOVER(self, gcmd: Any) -> None:
        """
        Recover a stuck / failed load: stop motion, reel the lane's filament
        back to the bay, and reset its state so AFC is no longer mid-operation.

        BAMBU_RECOVER UNIT=<unit> LANE=<lane>

        Use after a load errors out (e.g. a feeder "rocker stall"): the AMS is
        left idle but with filament staged partway in the path and the lane
        stuck in a load/error state. This halts the AMS, winds the filament back
        into the bay (the shared eject reel-back), and clears the lane so you can
        re-insert / retry. If the feeder still stalls after this, the bay's
        filament tip is jammed -- open the AMS, trim the tip, and reinsert.

        :param gcmd: The Klipper GCodeCommand
        """
        lane_name = gcmd.get("LANE")
        lane = self.lanes.get(lane_name)
        if lane is None:
            raise gcmd.error(
                f"BAMBU_RECOVER: lane '{lane_name}' not on unit {self.name} "
                f"(lanes: {', '.join(self.lanes) or 'none'})")
        if self._bridge is None:
            raise gcmd.error(
                f"BAMBU_RECOVER: bridge not connected for {self.name}")
        if self._slot_of(lane) is None:
            raise gcmd.error(
                f"BAMBU_RECOVER: {lane_name} is not mapped to an AMS slot")
        gcmd.respond_info(
            f"BAMBU_RECOVER: stopping and reeling {lane_name} back to the bay...")
        self._recover_to_bay(lane)
        gcmd.respond_info(
            f"BAMBU_RECOVER: {lane_name} reset. If a load still stalls the "
            f"feeder, the bay filament tip is jammed -- open the AMS, trim the "
            f"tip, and reinsert.")

    def cmd_BAMBU_RELINK(self, gcmd: Any) -> None:
        """
        Force an AMS relink / error-recovery reset for this unit.

        BAMBU_RELINK UNIT=<unit>

        Sends the firmware relink (deregister sweep + re-registration) to clear
        a unit stuck in a TIMEOUT/error state (state:7) without a power cycle.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_RELINK: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        self.relink()
        gcmd.respond_info(
            f"BAMBU_RELINK: sent AMS relink/reset for {self.name}.")

    def cmd_BAMBU_FEED(self, gcmd: Any) -> None:
        """
        Feed a bounded length from a lane's slot toward the toolhead.

        BAMBU_FEED UNIT=<unit> LANE=<lane> [MM=20] [SPEED=<mm/s>]

        The same primitive the load path uses, which is known to move filament.
        Exposed on its own so the feed command can be tested independently of
        the follower -- the follower can sit armed in mode:4 and never drive the
        motor, and without this there is no way to tell an AMS that will not
        feed from one that was never asked to.

        Also the only way to relieve a bottomed-out buffer from software:
        feeding separates the two PTFE ends and compresses the spring.

        :param gcmd: The Klipper GCodeCommand
        """
        lane_name = gcmd.get("LANE")
        mm = gcmd.get_float("MM", 20.0, above=0.0, maxval=200.0)
        speed = gcmd.get_float("SPEED", 0.0, minval=0.0)
        lane = self.lanes.get(lane_name)
        if lane is None:
            msg = (f"BAMBU_FEED: lane '{lane_name}' not on unit {self.name} "
                   f"(lanes: {', '.join(self.lanes) or 'none'})")
            raise gcmd.error(msg)
        if self._bridge is None:
            msg = f"BAMBU_FEED: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if self._slot_of(lane) is None:
            msg = f"BAMBU_FEED: {lane_name} is not mapped to an AMS slot"
            raise gcmd.error(msg)
        ok = self.feed(lane, mm, speed if speed > 0 else None)
        gcmd.respond_info(
            f"BAMBU_FEED: {'issued' if ok else 'FAILED to issue'} {mm:.0f}mm on "
            f"{lane_name} ({self.name}).")

    def cmd_BAMBU_RESTART(self, gcmd: Any) -> None:
        """
        Reboot the Pico back into this same firmware.

        Klipper's FIRMWARE_RESTART for the bus master. The firmware holds a
        pile of runtime state the host announced to it -- unit count, HT flags,
        per-unit MC addressing, drain overrides, mute mask, follower state --
        and none of it survives a Pico reboot, so this clears the lot and the
        reconnect handler re-announces from scratch.

        Distinct from the bootsel reboot the flasher uses: that lands in the
        ROM UF2 loader with no bridge until an image is copied. This comes
        straight back up talking.

        Useful because the AMS chain and printer stay powered, which separates
        "the bridge's runtime state is wrong" from "the bus is wrong" -- a
        distinction otherwise only reachable by unplugging things.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            raise gcmd.error(
                f"BAMBU_RESTART: bridge not connected for {self.name}")
        self._bridge.send({"cmd": "reset"})
        gcmd.respond_info(
            f"BAMBU_RESTART: rebooting the bridge for {self.name}. It comes "
            f"back in a couple of seconds and every unit re-announces itself; "
            f"check with BAMBU_UIDS.")

    def cmd_BAMBU_MUTE(self, gcmd: Any) -> None:
        """
        Suppress individual bridge transmitters, to find which one a unit
        reacts to audibly.

        BAMBU_MUTE UNIT=<unit> MASK=<bits>

        1=MC_ONLINE 4=MC_023C 8=MC_3702 16=heartbeat 32=HT poll
        64=AP2 sync 128=L2C poke 256=presence 512=online_detect.
        MASK=0 restores everything.

        Bit 2 is INTENTIONALLY absent: it named MC_0411, which is no longer sent
        on a timer, and the firmware never actually tested that bit -- so the
        command reported "muted=MC_0411" while changing nothing, and a live test
        was credited to it. An advertised control that does nothing is worse
        than no control.

        Diagnostic only. Muting the wrong thing will drop the follower -- that
        is the point: it identifies what a unit actually depends on, in seconds,
        instead of one firmware flash per hypothesis.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_MUTE: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        mask = gcmd.get_int("MASK", 0, minval=0, maxval=1023)
        self._bridge.send({"cmd": "mute", "mask": mask})
        names = ("MC_ONLINE", None, "MC_023C", "MC_3702", "heartbeat",
                 "HT_poll", "AP2_sync", "L2C_poke", "presence", "online_detect")
        muted = [n for i, n in enumerate(names) if n and mask & (1 << i)]
        gcmd.respond_info(
            f"BAMBU_MUTE: mask={mask} "
            f"muted={', '.join(muted) if muted else 'nothing (all restored)'}")

    def cmd_BAMBU_HTPOLL(self, gcmd: Any) -> None:
        """
        Set the 0x1800 keep-alive cadence at runtime, in ms. 0 restores the
        firmware default.

        BAMBU_HTPOLL UNIT=<unit> MS=<ms>

        For finding this unit's actual limit by sweeping, instead of one reflash
        per value. BAMBU_MUTE cannot answer this: muting a poll stops the unit's
        liveness being refreshed, it reads offline, and offline gates OTHER
        transmitters -- so a mute proves only that something downstream of
        "online" stopped, not which frame was responsible.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_HTPOLL: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        ms = gcmd.get_int("MS", 0, minval=0, maxval=5000)
        self._bridge.send({"cmd": "htpoll", "ms": ms})
        gcmd.respond_info(
            f"BAMBU_HTPOLL: 0x1800 keep-alive "
            f"{'default (150ms)' if ms == 0 else str(ms) + 'ms'}")

    def cmd_BAMBU_DRAIN(self, gcmd: Any) -> None:
        """
        Set the payload byte of the 1A/02 log drain. 255 restores the default.

        BAMBU_DRAIN UNIT=<unit> P=<byte>

        The byte is per-model and there is no way to derive it: an AMS 2 Pro
        answers P=1 every time and ignores P=0 completely, which is why 2000+
        silent exchanges were once misread as "this unit cannot narrate". A
        regular AMS answers neither, so its value has to be found by sweeping.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_DRAIN: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        p = _gcmd_int(gcmd, "P", 255, 0, 255)
        # ADDR retargets the drain. The captured frame asks 0x0700 -- the AMS 2
        # Pro -- so on an HT-only bus it has never drawn a reply (snap counters
        # flat at 0 against 500+ empty exchanges) even while narration arrived
        # via other polls. An HT answers at 0x1800, and its own frames carry
        # payload 0x00 -- BAMBU_DRAIN ADDR=0x1800 P=0 is the pair that
        # answers. NOT 0x80, which the capture was read as and which
        # draws no reply at all; swept on hardware, see _MC_ADDRESSING.
        # 0 keeps the captured address.
        addr = _gcmd_int(gcmd, "ADDR", 0, 0, 0xFFFF)
        self._bridge.send({"cmd": "drain", "p": p, "addr": addr})
        gcmd.respond_info(
            f"BAMBU_DRAIN: log-drain payload "
            f"{'default (0x01/0x00)' if p == 255 else hex(p)}"
            f", device {'captured (0x0700)' if not addr else hex(addr)}")

    def cmd_BAMBU_HB(self, gcmd: Any) -> None:
        """
        Set the bus heartbeat cadence at runtime, in ms. 0 restores the default.

        BAMBU_HB UNIT=<unit> MS=<ms>

        Keep it under ~1000: the AMS declares itself offline without a heartbeat
        for about a second, and an offline unit gates other transmitters, which
        makes any measurement taken there uninterpretable.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_HB: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        ms = gcmd.get_int("MS", 0, minval=0, maxval=5000)
        self._bridge.send({"cmd": "hb", "ms": ms})
        gcmd.respond_info(
            f"BAMBU_HB: heartbeat "
            f"{'default (300ms)' if ms == 0 else str(ms) + 'ms'}")

    def cmd_BAMBU_ARMMS(self, gcmd: Any) -> None:
        """
        Set the 11/04 follower keep-alive cadence at runtime, in ms.

        BAMBU_ARMMS UNIT=<unit> MS=<ms>       (0 restores the printer's ~507ms)

        This is the ONE per-cycle transmitter with no MUTE_* bit, so when a
        unit ticks at idle it is the only suspect that cannot be bisected by
        BAMBU_MUTE. Winding the cadence out to tens of seconds is the
        equivalent, and needs no reflash. The frame is never answered, so
        slowing it costs a liveness signal and nothing else -- but the AMS does
        declare itself offline without one, so put it back afterwards.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_ARMMS: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        ms = _gcmd_int(gcmd, "MS", 0, 0, 600000)
        self._bridge.send({"cmd": "armms", "ms": ms})
        gcmd.respond_info(
            f"BAMBU_ARMMS: follower keep-alive "
            f"{'default (~507ms)' if ms == 0 else str(ms) + 'ms'}")

    def cmd_BAMBU_HTID(self, gcmd: Any) -> None:
        """
        Set the id byte the bridge uses on an AMS HT's 0x1800 commands.

        BAMBU_HTID UNIT=<unit> ID=<0-255>

        An HT only acts on commands addressed to the right id, and which id that
        is has not been settled: the real-printer capture uses 0x80 (128), the
        unit reports ref:165, and the chain index is 0. ID=0 falls back to the
        chain index, ID=255 tracks whatever the AMS reports.

        For finding the answer on hardware instead of by argument.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_HTID: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        htid = _gcmd_int(gcmd, "ID", 128, 0, 255)
        self._bridge.send({"cmd": "htid", "id": htid})
        gcmd.respond_info(
            f"BAMBU_HTID: 0x1800 commands now addressed to id {htid} "
            f"(0=chain index, 255=track the AMS ref).")

    def cmd_BAMBU_BUFFER_PROBE(self, gcmd: Any) -> None:
        """
        Dump the AMS's raw motion reply and the buffer decode's own state.

        BAMBU_BUFFER_PROBE UNIT=<unit>

        For working out where (or whether) an AMS model reports its FPS buffer.
        The mapped 0..100 value cannot show a decode that never ran or a
        calibration that saturates, so this prints the raw frame alongside
        reads/replylen/raw. Hold the buffer at a known position and compare
        frames to find the byte that tracks it.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_BUFFER_PROBE: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        self._bridge.send({"cmd": "reply"})
        # The reply lands on the reader thread; give it a moment to arrive.
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.3)
        latest = self._bridge.latest_status() or {}
        with self._bridge._lock:
            frame = self._bridge._last_raw_reply
        gcmd.respond_info(
            f"{self.name}: buff={latest.get('buff')} raw={latest.get('buffraw')} "
            f"replylen={latest.get('bufflen')} reads={latest.get('buffn')} "
            f"fstate={latest.get('fstate')} fstate_reads={latest.get('fstaten')} "
            # The narration counters. They were already in every status frame
            # and nothing surfaced them, so "the AMS has nothing to say" and
            # "we stopped asking" stayed indistinguishable -- the exact
            # ambiguity that cost an afternoon of guessing. polls counts drain
            # requests SENT, frames the narration-shaped replies, texts the
            # ones that actually carried words.
            f"dbg_polls={latest.get('dbgpolls')} "
            f"dbg_frames={latest.get('dbgframes')} "
            f"dbg_texts={latest.get('dbgtexts')} "
            # The receipt for the announce, read back out of the firmware. A
            # missing or zero mcack is the fallback drain's cause, not its
            # symptom: without it the poll set goes to the captured 0x0700 and
            # an HT at 0x1800 is never asked, so dbg_texts sits at 0 with the
            # bus otherwise perfectly healthy.
            f"mcack={self._mcaddr_ack_str()} "
            # The buffer as the UNIT measures it, which is finer than the
            # 0..100 status field above: buff_pos is its instantaneous
            # position and refill is (sagged_to, recovered_to, mm_fed) from
            # its last on-demand top-up -- the ramming event itself.
            f"buff_pos={self._bridge_call('last_buff_pos')} "
            f"refill={self._bridge_call('last_buff_refill')} "
            f"sync={latest.get('syncn')} pokes={latest.get('poken')}")
        gcmd.respond_info(f"{self.name}: frame={frame or '(none)'}")

    def _bridge_call_arg(self, name: str, arg: Any) -> Any:
        """
        Call an optional one-argument bridge accessor, tolerating an older
        bridge that does not have it.

        :param name: accessor name
        :param arg: its single argument
        :return Any: its value, or None if unavailable or it raised
        """
        fn = getattr(self._bridge, name, None)
        if not callable(fn):
            return None
        try:
            return fn(arg)
        except Exception:
            return None

    def _bridge_call(self, name: str) -> Any:
        """
        Call an optional bridge accessor, tolerating an older bridge.

        :param name: accessor name
        :return Any: its value, or None if unavailable or it raised
        """
        fn = getattr(self._bridge, name, None)
        if not callable(fn):
            return None
        try:
            return fn()
        except Exception:
            return None

    def _mcaddr_ack_str(self) -> str:
        """
        This unit's acknowledged MC address, formatted for the probe.

        "none" (never acknowledged) and "0x0000" (acknowledged as unset) are
        deliberately different strings -- see BambuBridge.mcaddr_ack.

        :return str: "0xNNNN", "none", or "?" if the bridge is too old to say
        """
        getter = getattr(self._bridge, "mcaddr_ack", None)
        if not callable(getter):
            return "?"
        try:
            ack = getter(self.ams_index)
        except Exception:
            return "?"
        return "none" if ack is None else f"0x{int(ack):04X}"

    def cmd_BAMBU_SCAN(self, gcmd: Any) -> None:
        """
        Trigger an RFID/tag scan on demand -- the exact read the auto-scan runs
        on a fresh insert, but callable by hand.

        BAMBU_SCAN UNIT=<unit> [LANE=<lane>]

        With no LANE, scans every slot on the unit; with LANE, just that lane's
        slot. Mainly for the AMS HT, whose tag only reads when the bridge polls
        it at 0x1800 -- run this if a spool's material never populated. The read
        also clears the per-slot auto-scan latch so the result is fresh.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_SCAN: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        lane_name = gcmd.get("LANE", None)
        if lane_name is not None:
            lane = self.lanes.get(lane_name)
            if lane is None:
                raise gcmd.error(
                    f"BAMBU_SCAN: lane '{lane_name}' not on unit {self.name} "
                    f"(lanes: {', '.join(self.lanes) or 'none'})")
            slot = self._slot_of(lane)
            if slot is None:
                raise gcmd.error(
                    f"BAMBU_SCAN: {lane_name} is not mapped to an AMS slot")
            # Drop the latch so the manual scan always re-reads (even if an
            # earlier auto-scan already fired for this slot).
            if 0 <= slot < len(self._auto_scanned):
                self._auto_scanned[slot] = False
            if not self.scan(slot):
                msg = (f"BAMBU_SCAN: scan command not issued for {lane_name} "
                       f"on {self.name}")
                raise gcmd.error(msg)
            gcmd.respond_info(
                f"BAMBU_SCAN: scanning {lane_name} (slot {slot}) on {self.name} "
                f"at 0x{getattr(self, 'dry_dev_addr', 0):04X}; material lands "
                f"when the tag reads.")
            return
        nslots = min(len(self._auto_scanned),
                     getattr(self, "unit_slots", len(self._auto_scanned)))
        for i in range(nslots):
            self._auto_scanned[i] = False
        if not self.scan(None):
            msg = f"BAMBU_SCAN: scan command not issued for {self.name}"
            raise gcmd.error(msg)
        gcmd.respond_info(
            f"BAMBU_SCAN: scanning {nslots} slot(s) on {self.name} at "
            f"0x{getattr(self, 'dry_dev_addr', 0):04X}.")

    def cmd_BAMBU_HEATER_START(self, gcmd: Any) -> None:
        """
        Start AMS drying (AMS2 Pro heater).

        BAMBU_HEATER_START UNIT=<unit> [TEMP=55] [TIME=480] [ROTATE=0]

        TEMP in C (clamped 0..65), TIME in minutes, ROTATE=1 spins the spools
        while drying. Replays the printer's drying command on our bus.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_HEATER_START: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if not self.has_heater:
            gcmd.respond_info(
                f"BAMBU_HEATER_START: {self.name} has no drying heater "
                f"(heater: false). Only the AMS2 Pro can dry -- ignoring.")
            return
        # Clamp (not hard-error) to this unit's drying ceiling (dry_max_temp:
        # 65 for AMS2 Pro, 85 for AMS HT). Asking for more would be rejected by
        # the AMS or risk the spools, so cap it and tell the user rather than
        # halting.
        temp = gcmd.get_int("TEMP", 55, minval=0)
        if temp > self.dry_max_temp:
            gcmd.respond_info(
                f"BAMBU_HEATER_START: TEMP {temp}C exceeds {self.name}'s drying "
                f"max {self.dry_max_temp}C -- clamping to {self.dry_max_temp}C.")
            temp = self.dry_max_temp
        tmin = gcmd.get_int("TIME", 480, minval=0, maxval=65535)
        rot = gcmd.get_int("ROTATE", 0, minval=0, maxval=1)
        # Safety: never spin the spools if any lane on this unit is committed
        # into the filament path (staged at the hub or threaded to the toolhead)
        # -- rotating a committed spool fights the path (jams the hub / yanks the
        # toolhead). Drying still runs, just without rotation.
        if rot:
            committed = self._committed_lanes()
            if committed:
                rot = 0
                names = ", ".join(getattr(ln, "name", "?") for ln in committed)
                gcmd.respond_info(
                    f"BAMBU_HEATER_START: ROTATE disabled for {self.name} -- "
                    f"{names} staged/loaded to the toolhead (spinning would "
                    f"fight the filament path). Drying without rotation.")
        # Quiet the follower before the AMS starts its self-check -- but NOT
        # when a lane on this unit is threaded to the toolhead. Drying while
        # that lane prints is legitimate (dry-while-printing); dropping the
        # follower there makes the extruder fight the pull, which is exactly
        # what a purge during an HT dry showed. Keep following in that case.
        # Pre-check: an AMS refuses to dry with filament out in the hub --
        # "[AMS_CHMB]err, filament hub load!" -- and it refuses AFTER echoing
        # our temp/time back, so the command looks delivered and the panel just
        # sits at "not drying". Say so before sending.
        #
        # HT only, and gated on tool_loaded ONLY.
        #
        # tool_loaded is the one state in which filament is genuinely OUT of
        # the unit -- threaded through the hub and into the toolhead. That is
        # what an HT's interlock objects to; it answers
        # "[AMS_CHMB]err, filament hub load!" and heats nothing.
        #
        # NOT loaded_to_hub. That is a STAGING state -- "parked near the hub
        # for a fast reload" -- an intent this module sets and clears itself,
        # with the filament still inside the unit. A first version of this
        # check included it and warned about lane23 while the HT went on to
        # heat perfectly well.
        #
        # HT only because an ACE and an ACE 2 heat while printing, so this is
        # a property of the unit rather than of drying, and whether an AMS 2
        # Pro shares it is untested. If one does refuse, its own words reach
        # the console and the dryer card regardless.
        #
        # WARN, not block: a unit whose interlock differs must stay dryable.
        loaded = [ln for ln in self.lanes.values()
                  if getattr(ln, "tool_loaded", False)] if self._is_ht() else []
        if loaded:
            names = ", ".join(getattr(ln, "name", "?") for ln in loaded)
            gcmd.respond_info(
                f"BAMBU_HEATER_START: {names} is loaded to the toolhead, so "
                f"this unit's filament is out of it. An AMS HT will not heat "
                f"in that state (it answers \"filament hub load!\") -- unload "
                f"the lane and start again.")
        self._drying = True
        # A new cycle: telemetry from before this point belongs to the old one,
        # and no reading for THIS cycle has arrived yet (which is what the
        # panel's "Starting -- waiting for the unit to report" means).
        self._dry_adopt_after = _mono(self)
        self._dry_seen_live = False
        if self._following_lane is not None and self._tool_loaded_lane() is None:
            try:
                self.set_feed_assist(self._following_lane, False)
            except Exception:
                pass
            self._following_lane = None
        # AMSID/ADDR overrides: diagnostic knobs for multi-AMS chains where
        # the drying id byte's mapping (normally = chain index) needs to be
        # confirmed per position on the wire. GUARDED: a stale override sends
        # this unit's dry frame at ANOTHER unit's id, which the target simply
        # ignores, so a non-matching id is refused unless FORCE=1.
        amsid = _gcmd_int(gcmd, "AMSID", self.dry_ams_id, 0, 0xFFFF)
        addr = _gcmd_int(gcmd, "ADDR", self.dry_dev_addr, 0, 0xFFFF)
        if ((amsid != self.dry_ams_id or addr != self.dry_dev_addr)
                and gcmd.get_int("FORCE", 0) != 1):
            raise gcmd.error(
                f"BAMBU_HEATER_START: AMSID/ADDR override ({amsid}/0x{addr:04X}) "
                f"does not match {self.name}'s own addressing "
                f"({self.dry_ams_id}/0x{self.dry_dev_addr:04X}) — another "
                f"unit's heater would be targeted. Drop the override, or add "
                f"FORCE=1 for deliberate diagnostics.")
        self._bridge.send({"cmd": "dry", "unit": self.ams_index, "on": 1,
                           "temp": temp, "time": tmin, "rotate": rot,
                           "addr": addr, "amsid": amsid})
        gcmd.respond_info(
            f"BAMBU_HEATER_START: {self.name} drying at {temp}C for {tmin}min"
            f"{' with spool rotation' if rot else ''}"
            f" (addr 0x{addr:04X}, id {amsid}).")

    def cmd_BAMBU_HEATER_STOP(self, gcmd: Any) -> None:
        """
        Stop AMS drying. BAMBU_HEATER_STOP UNIT=<unit>

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            msg = f"BAMBU_HEATER_STOP: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if not self.has_heater:
            gcmd.respond_info(
                f"BAMBU_HEATER_STOP: {self.name} has no drying heater "
                f"(heater: false) -- nothing to stop.")
            return
        # Must carry the same addr/amsid as start, or an HT (0x1800) never hears
        # the stop and keeps drying.
        self._bridge.send({"cmd": "dry", "unit": self.ams_index, "on": 0,
                           "addr": self.dry_dev_addr, "amsid": self.dry_ams_id})
        self._drying = False
        # Stamp the stop, PLUS a grace period. get_status adopts a cycle from
        # live chamber telemetry so the panel can catch up to a dry the host did
        # not start; the stamp keeps that adoption from re-setting _drying from
        # telemetry up to 120s old, i.e. from before this stop. The grace covers
        # the rest: an AMS does not fall silent the instant it is told to stop,
        # and the line or two it emits while winding down is NEWER than the
        # stop. If it is still reporting past the grace it really is still
        # drying, and adoption correctly re-arms rather than reporting a stop
        # that did not take.
        self._dry_adopt_after = _mono(self) + DRY_STOP_GRACE
        self._dry_seen_live = False
        gcmd.respond_info(f"BAMBU_HEATER_STOP: {self.name} drying stopped.")

    def cmd_BAMBU_UIDS(self, gcmd: Any) -> None:
        """
        Print the AMS UIDs currently on the bus, read straight off the wire.

        Requests the chain map from the bridge, then reports each chain index's
        UID plus what that unit holds (to tell them apart), so the UIDs can be
        copied into each section's ``unit_uid`` for stable mapping.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            raise gcmd.error("BAMBU_UIDS: bridge not connected")
        self._bridge.send({"cmd": "chain"})        # refresh the enrollment map
        try:                                        # report after the reply lands
            self.afc.reactor.register_callback(
                self._report_uids, self.afc.reactor.monotonic() + 0.5)
        except Exception:
            self._report_uids(0)                    # no reactor (tests)

    def _report_uids(self, eventtime: float) -> None:
        """Emit the cached chain UIDs + per-index occupancy to the console."""
        uids = self._bridge.chain_uids() if self._bridge else []
        if not uids:
            self.gcode.respond_info(
                "BAMBU_UIDS: no AMS UIDs read yet -- run it again in a moment.")
            return
        latest = (self._bridge.latest_status() if self._bridge else None) or {}
        slots = latest.get("slots", []) or []
        diag = getattr(self._bridge, "chain_diag", None) if self._bridge else None
        htmask, fw, sel = diag() if callable(diag) else (0, "", (-1, 0, 0))
        lines = [f"Bambu AMS bus (firmware {fw or 'pre-0.3.0'}) -- copy each "
                 f"UID into that unit's `unit_uid`:"]
        for i, u in enumerate(uids):
            occ = [f"slot{s.get('i')}={s.get('material') or 'present'}"
                   for s in slots
                   if s.get("unit") == i and s.get("present")]
            hint = ("  <- " + ", ".join(occ)) if occ else "  <- (empty)"
            ht = "  [HT-flagged]" if htmask & (1 << i) else ""
            lines.append(f"  chain index {i}: {u or '(none)'}{ht}{hint}")
        # Per-unit alignment: which chain index each configured unit resolved
        # to, and whether an HT unit's flag actually landed on its index --
        # THE thing to check when an insert-scan doesn't fire.
        # Select-probe verdict: which id the HT actually ACKED its type-07
        # select at (-1 = still probing / never acked). THE datum for the
        # insert-scan bring-up.
        selid, selsent, selack = sel
        if selsent:
            lines.append(
                f"HT select probe: sent={selsent} acked={selack} "
                f"locked_id={'none yet' if selid < 0 else hex(selid)}")
        # What the FIRMWARE actually holds per unit, not what the host thinks
        # it announced. The two diverging is otherwise invisible and has real
        # consequences: the narration log drain only goes per-unit when this is
        # set, and otherwise falls back to the captured 0x0700 pair, which
        # never asks an AMS HT at 0x1800.
        getter = getattr(self._bridge, "chain_mcaddr", None) \
            if self._bridge else None
        mcaddr = getter() if callable(getter) else None
        if isinstance(mcaddr, list) and any(mcaddr):
            shown = ", ".join(
                f"{i}:0x{int(a):04X}" for i, a in enumerate(mcaddr) if a)
            lines.append(f"Firmware MC addressing: {shown}")
        elif mcaddr is not None:
            lines.append(
                "Firmware MC addressing: NONE SET -- the log drain falls back "
                "to 0x0700 only, so an HT at 0x1800 is never asked for "
                "narration")
        lines.append("Configured units:")
        try:
            for _, unit in self.printer.lookup_objects("AFC_BambuAMS"):
                idx = getattr(unit, "ams_index", "?")
                is_ht = bool(getattr(unit, "has_heater", False) and
                             getattr(unit, "dry_dev_addr", 0) == 0x1800)
                mark = ""
                if is_ht:
                    flagged = isinstance(idx, int) and bool(htmask & (1 << idx))
                    mark = ("  [HT, flag OK]" if flagged
                            else "  [HT, FLAG MISSING -- insert-scan will NOT "
                                 "fire]")
                lines.append(f"  {getattr(unit, 'name', '?')} -> chain index "
                             f"{idx}{mark}")
        except Exception:
            pass
        self.gcode.respond_info("\n".join(lines))

    def _recover_to_bay(self, lane: Any) -> None:
        """
        Relink the AMS, stop it, reel this lane's filament back to the bay, and
        clear its loaded/error state. Shared by eject and BAMBU_RECOVER so a
        stuck load is always recoverable the same way. Best-effort: never raises.

        :param lane: The lane to recover
        """
        # Clear any AMS TIMEOUT/error (state:7) first so the reel-back can run.
        try:
            self.relink()
        except Exception:
            pass
        # Drop the follower so the re-arm timer doesn't fight the reel-back.
        try:
            self.set_feed_assist(lane, False)
        except Exception:
            pass
        # Shared reel-back: halt any in-flight feed/retry, then wind the filament
        # all the way back into the AMS bay (stop -> select -> long retract).
        try:
            self.eject_lane(lane)
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: recover reel-back for "
                f"{getattr(lane, 'name', '?')} did not complete: {e}")
        # Clear the toolhead-loaded flag, then re-derive the bay's state from
        # presence. On a Bambu AMS the spool STAYS in the bay after a reel-back,
        # so a present bay is "staged and ready" (LOADED) -- not "filament
        # detected but not loaded" (NONE). Only a genuinely empty bay goes NONE.
        try:
            lane.tool_loaded = False
            slot = self._slot_of(lane)
            info = (self._slots[slot]
                    if slot is not None and 0 <= slot < len(self._slots)
                    else {})
            if info.get("present"):
                lane.loaded_to_hub = True
                lane.status = AFCLaneState.LOADED
                try:
                    self.lane_loaded(lane)
                    self.lane_illuminate_spool(lane)
                except Exception:
                    pass
            else:
                lane.loaded_to_hub = False
                lane.status = AFCLaneState.NONE
                try:
                    self.lane_not_ready(lane)
                except Exception:
                    pass
            if self._is_virtual_hub(lane):
                lane._load_state = False          # not threaded to the toolhead
        except Exception:
            pass
        try:
            self.afc.save_vars()
        except Exception:
            pass

    def _handle_disconnect(self) -> None:
        """
        Tear down the shared bridge on host disconnect / FIRMWARE_RESTART.

        The bridge is cached in a module-global keyed by serial port. That
        global survives a FIRMWARE_RESTART (Python isn't re-imported), so without
        this the new unit instances reuse the OLD bridge whose reader thread and
        serial port are now dead -- no status frame ever arrives and every lane
        shows empty until a full reboot. Stopping it here (idempotent across the
        daisy-chained units) forces _handle_ready to rebuild a fresh connection
        and re-prime status.
        """
        bridge = _BRIDGES.pop(self.serial_port, None)
        if bridge is not None:
            try:
                bridge.stop()
            except Exception:
                pass
        self._bridge = None
        # Reset per-unit caches so the fresh connect re-seeds cleanly.
        self._slots = [{} for _ in range(self.SLOTS_PER_UNIT)]
        self._prev_present = [False] * self.SLOTS_PER_UNIT
        self._auto_scanned = [False] * self.SLOTS_PER_UNIT
        self._scan_primed = False           # re-prime the baseline after reconnect

    def _handle_ready(self) -> None:
        """Build the slot map and connect to the bridge once the reactor is up."""
        try:
            self._slot_map = build_slot_map(self.lanes, self.SLOTS_PER_UNIT)
        except ValueError as e:
            raise config_error(f"AFC_BambuAMS {self.name}: {e}")
        # Seed each lane's virtual-hub occupancy. Pinless lanes default
        # _load_state=True upstream, so a virtual hub (any(raw_load_state))
        # would read "loaded" for every lane until the first status frame.
        # Live occupancy is only true while the lane's tool is loaded.
        for lane in self.lanes.values():
            if self._is_virtual_hub(lane):
                lane._load_state = bool(getattr(lane, 'tool_loaded', False))
        # Reuse an existing bridge for this serial port (shared across the
        # daisy-chained units), or create + start it for the first unit.
        bridge = _BRIDGES.get(self.serial_port)
        fresh = bridge is None
        if fresh:
            bridge = BambuBridge(self._open_serial, self.afc.reactor,
                                 self.logger)
            # The AMS's narration goes to its own rotating file, independent of
            # AFC's `debug` flag. That keeps every STEP, finish, stall and
            # measured length on record with debug off, which is the normal
            # state for a working printer since the unit narrates continuously.
            # Once per bridge, not per unit: several units share one file.
            try:
                import os
                log_file = self.printer.start_args.get("log_file", None)
                if log_file:
                    bridge.set_narration_log(os.path.dirname(log_file))
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: no narration log ({e})")
            _BRIDGES[self.serial_port] = bridge
        self._bridge = bridge
        bridge.add_listener(self._on_status)
        bridge.add_reconnect_listener(self._on_bridge_reconnect)
        # After AFC restores saved lane state, re-assert the AMS "loaded" state
        # for any lane already tool-loaded (survives a reboot). We use the mode-07
        # "finish"/loaded signal, NOT continuous feed assist: blind feed while the
        # extruder is idle stall-retracts and fights (LED flashing). This keeps
        # the AMS aware the bay is loaded so it's ready to follow the extruder.
        try:
            self.afc.reactor.register_callback(
                lambda et: self._startup_restore_loaded(),
                self.afc.reactor.monotonic() + 5.0)
        except Exception:
            pass
        # After the firmware has reported its initial presence, mark the scan
        # baseline primed so only genuine post-startup inserts trigger a scan.
        try:
            self.afc.reactor.register_callback(
                lambda et: setattr(self, "_scan_primed", True),
                self.afc.reactor.monotonic() + 8.0)
        except Exception:
            self._scan_primed = True        # no reactor (tests): scan immediately
        # Demand-gated follower re-engage timer (watches the extruder; re-selects
        # only on real extrusion). One timer per unit; harmless if idle.
        try:
            if self._follow_timer is None:
                self._follow_timer = self.afc.reactor.register_timer(
                    self._follow_tick,
                    self.afc.reactor.monotonic() + self.follow_poll_interval)
        except Exception:
            pass
        # Chain self-heal watchdog: re-verify this unit's UID -> chain-index pin
        # every 30s. A unit that power-cycles MID-SESSION can re-enroll at a
        # different address; the firmware's chain map updates but nothing else
        # would tell us. Reading the map costs zero AMS-bus traffic (it comes
        # from the Pico's RAM), and a detected move re-pins + re-seeds lanes.
        try:
            if self.unit_uid and self._uid_watch_timer is None:
                self._uid_watch_timer = self.afc.reactor.register_timer(
                    self._uid_watch_tick,
                    self.afc.reactor.monotonic() + 30.0)
        except Exception:
            pass
        # UID-pin BEFORE announcing or seeding anything: with several units on
        # one bridge, the firmware's chain indices shuffle across power-cycles,
        # and everything per-unit (status filtering, PREP lane seeding, the HT
        # flag, drying) is keyed by ams_index. Resolve unit_uid -> index NOW,
        # synchronously, so PREP always reports the RIGHT physical unit's
        # occupancy and the announces below flag the right chain index. (The old
        # async-only resolution let PREP run first, so with no ams_index in the
        # config every unit seeded its lanes from index 0 -- AMS1's spools showed
        # up on every unit.)
        if not fresh:
            if self.unit_uid:
                self._resolve_uid_blocking()
            self._announce_unit()
            # One line per non-owning unit at startup restating a wiring detail
            # the user already configured -- AFC.log only.
            self.logger.debug(
                f"AFC bambu {self.name}: sharing bridge on {self.serial_port} "
                f"(ams_index={self.ams_index})")
            return
        try:
            bridge.start()
            # Pin the device variant if configured; 'auto' lets the firmware
            # detect AMS vs lite by probing both bus addresses.
            if self.variant != "auto":
                bridge.send({"cmd": "variant", "set": self.variant})
            if self.unit_uid:
                self._resolve_uid_blocking()
            self._announce_unit()
            # Prime state: ask the bridge for an immediate status frame so the
            # lanes reflect real slot presence before the first periodic poll.
            bridge.send({"cmd": "status"})
            self.logger.info(
                f"AFC bambu {self.name}: bridge connected on {self.serial_port} "
                f"(variant={self.variant}, ams_index={self.ams_index})")
            # Warn (once, on the bridge-owning unit) if the configured AMS types
            # exceed Bambu's bus limits (<=4 four-slot AMS, <=8 HT, <=12 total).
            try:
                models = []
                for _, u in self.printer.lookup_objects("AFC_BambuAMS"):
                    m = getattr(u, "ams_model", None)
                    if m:
                        models.append(m)
                warn = check_ams_limits(models)
                if warn:
                    self.logger.warning(
                        f"AFC bambu: AMS bus over Bambu limits -- {warn}. Extra "
                        f"units past the limit may not enroll.")
            except Exception:
                pass
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: bridge connect failed: {e}")

    def _resolve_uid_index(self, tries: int) -> None:
        """
        Pin ams_index to the chain index currently carrying ``unit_uid``.

        The firmware assigns chain indices by announce order (reshuffles across
        power-cycles). We ask the bridge for the chain UID map and, when this
        unit's UID appears, adopt its index -- so every command addresses the
        right physical unit no matter what order they booted. Retries because the
        firmware needs a moment after boot to enroll all units.

        :param tries: retry counter
        """
        if self._bridge is None or not self.unit_uid:
            return
        self._bridge.send({"cmd": "chain"})            # request the enrollment map
        # The reply comes back asynchronously on the bridge's read thread, so we
        # must NOT read chain_uids() on this same tick -- it would race ahead of
        # the reply and always see the previous (or empty) map, leaving UID
        # pinning to fall back to the config ams_index. Match AFTER a short
        # delay, exactly like BAMBU_UIDS does.
        try:
            self.afc.reactor.register_callback(
                lambda et: self._match_uid_index(tries),
                self.afc.reactor.monotonic() + 0.6)
        except Exception:
            self._match_uid_index(tries)               # no reactor (tests)

    def _match_uid_index(self, tries: int) -> None:
        """
        Match ``unit_uid`` against the cached chain map (populated by the earlier
        ``chain`` request) and adopt its index; retry the whole request if the map
        isn't ready yet. Split from _resolve_uid_index so the read happens AFTER
        the async reply has landed.

        :param tries: retry counter
        """
        if self._bridge is None or not self.unit_uid:
            return
        uids = self._bridge.chain_uids()
        idx = uids.index(self.unit_uid) if self.unit_uid in uids else -1
        if idx >= 0:
            self._adopt_index(idx)
            return
        # Keep retrying for ~1 min: enrollment after a Pico reboot (reflash /
        # replug / discovery resync) can take tens of seconds with a full chain,
        # and adopting late still self-heals the lanes (_adopt_index clears the
        # stale slot cache and re-seeds from a fresh status).
        if tries < 40:                                 # chain not ready yet -> retry
            try:
                self.afc.reactor.register_callback(
                    lambda et: self._resolve_uid_index(tries + 1),
                    self.afc.reactor.monotonic() + 1.5)
            except Exception:
                pass
        else:
            self.logger.warning(
                f"AFC bambu {self.name}: unit_uid {self.unit_uid} never appeared "
                f"on the chain ({uids}); check the unit_uid value against "
                f"BAMBU_UIDS. Holding chain index {self.ams_index}.")

    def _adopt_index(self, idx: int) -> None:
        """
        Adopt chain index ``idx`` as this unit's ams_index (UID-resolved) and
        re-key everything derived from it: the drying id, the firmware's polled
        unit count and HT flag, and this unit's cached slot data (anything cached
        under the old index belongs to ANOTHER physical unit and must never seed
        this unit's lanes).

        :param idx: The chain index carrying this unit's unit_uid
        """
        old = self.ams_index
        if idx == old:
            # Quiet: re-confirmations happen at every resolve/reconnect and say
            # nothing new. Only a CHANGED pin (below) is console-worthy.
            try:
                self.logger.debug(
                    f"AFC bambu {self.name}: UID {self.unit_uid} confirmed at "
                    f"ams_index {idx}")
            except Exception:
                pass
            self._send_ht_flag(self._bridge)     # re-assert (Pico may have rebooted)
            self._send_selfcentre_flag(self._bridge)
            self._send_mc_addr(self._bridge)
            return
        self.ams_index = idx
        if self._dry_id_follows_index:
            self.dry_ams_id = idx
        self._slots = [{} for _ in range(self.SLOTS_PER_UNIT)]
        try:
            self._bridge.send({"cmd": "units", "n": idx + 1})
            # Move the HT flag from the old index to the resolved one so the
            # firmware arms the insert-edge scan on the RIGHT unit.
            if self._is_ht():
                self._bridge.send({"cmd": "htunit", "unit": old, "on": 0})
            self._send_ht_flag(self._bridge)
            self._send_selfcentre_flag(self._bridge)
            self._send_mc_addr(self._bridge)
            self._bridge.send({"cmd": "status"})    # re-seed from the right unit
        except Exception:
            pass
        self.logger.info(
            f"AFC bambu {self.name}: pinned to UID {self.unit_uid} at "
            f"chain index {idx} (was ams_index {old})")

    def _resolve_uid_blocking(self, timeout: float = 30.0) -> bool:
        """
        Resolve ``unit_uid`` -> ams_index synchronously, pausing the reactor
        until the chain map arrives (or ``timeout``). Called from klippy:ready
        BEFORE anything seeds lanes or announces per-unit state, so PREP always
        reads the right physical unit. Falls back to the async retry path (and
        the config ams_index meanwhile) if the chain doesn't come up in time.

        :param timeout: Max seconds to wait for the chain map
        :return bool: True if the UID was resolved
        """
        if self._bridge is None or not self.unit_uid:
            return False
        try:
            reactor = self.afc.reactor
            end = reactor.monotonic() + timeout
            last_req = -10.0
            while reactor.monotonic() < end:
                now = reactor.monotonic()
                if now - last_req >= 1.0:      # (re-)request the enrollment map
                    last_req = now
                    try:
                        self._bridge.send({"cmd": "chain"})
                    except Exception:
                        pass
                uids = self._bridge.chain_uids()
                if self.unit_uid in uids:
                    self._adopt_index(uids.index(self.unit_uid))
                    return True
                reactor.pause(now + 0.25)
        except Exception:
            pass
        try:
            seen = self._bridge.chain_uids()
        except Exception:
            seen = []
        # Debug, not warning: a slow chain at startup is an expected transient
        # (Pico re-enumeration + a 3-unit rediscovery can outrun any window) and
        # the system self-heals -- the ~1min background retry and the 30s
        # watchdog re-pin, re-flag, and re-seed lanes when the UID appears. The
        # 40-retry give-up (wrong unit_uid VALUE) still warns loudly.
        try:
            self.logger.debug(
                f"AFC bambu {self.name}: unit_uid {self.unit_uid} not on the "
                f"chain after {timeout:.0f}s (chain so far: {seen or 'empty'}); "
                f"holding chain index {self.ams_index} until it appears "
                f"(background retry)")
        except Exception:
            pass
        try:
            self.afc.reactor.register_callback(
                lambda et: self._resolve_uid_index(0),
                self.afc.reactor.monotonic() + 2.0)
        except Exception:
            pass
        return False

    def _announce_unit(self) -> None:
        """
        Push this unit's per-unit firmware config: the polled-unit count and the
        HT flag. Called at connect (after UID resolution) and again on every
        bridge reconnect -- a Pico reboot (reflash / power-cycle / replug) resets
        both to factory defaults.
        """
        if self._bridge is None:
            return
        # Each send is independent, and deliberately not wrapped in one shared
        # try/except: a failure in any of the first three must not skip
        # _send_mc_addr. Without a per-unit MC address the firmware's log drain
        # falls back to the captured 0x0700 pair, which never asks an HT at
        # 0x1800 -- so every HT load, unload, stall and measured length is
        # discarded while the bus still looks healthy.
        for what, fn in (
                ("units", lambda: self._bridge.send(
                    {"cmd": "units", "n": self.ams_index + 1})),
                ("ht flag", lambda: self._send_ht_flag(self._bridge)),
                ("self-centre flag",
                 lambda: self._send_selfcentre_flag(self._bridge)),
                ("mc address", lambda: self._send_mc_addr(self._bridge))):
            try:
                fn()
            except Exception as e:
                # Say which one. A swallowed announce presents as a unit that
                # is online and polling but never narrates, which is a much
                # harder fault to find than a logged failure.
                self.logger.warning(
                    f"AFC bambu {self.name}: could not announce {what} to the "
                    f"bridge ({e}); narration or addressing may be wrong")

    def _on_bridge_reconnect(self) -> None:
        """
        The serial link came back, which usually means the Pico REBOOTED
        (reflash, power-cycle, replug): the firmware's unit count and HT flags
        are gone and the chain may have re-enrolled in a different order.
        Re-announce this unit's config and re-resolve the UID pin.
        """
        # Reboot the Pico FIRST, once per connection, then announce into a
        # clean firmware.
        #
        # Measured, not assumed: the bridge had wedged into the log drain's
        # 0x0700 fallback branch and stayed there across Klipper restarts,
        # module syncs and even Pico REFLASHES -- dbg_texts pinned at 0 with
        # the bus otherwise healthy. A BAMBU_RESTART fixed it instantly:
        # dbg_texts went 0 -> 9 and the frames/polls ratio flipped from 1:2
        # (fallback) to ~1:1 (per-unit), so the addressing finally applied.
        # A reflash does not achieve this because it reboots into the same
        # announce sequence that had already gone wrong.
        if self._reset_bridge_once():
            # The reset drops the link; the reconnect it causes runs this again
            # and the cooldown makes THAT pass do the announce, on a firmware
            # that has just come up fresh.
            return
        # ...but "the link is back" is not "the firmware is ready": announce
        # AFTER a settle delay, on the reactor, never by blocking here. This
        # runs on the bridge's reader thread and a sleep in it stops every unit
        # sharing the Pico from reading its status frames.
        try:
            reactor = self.afc.reactor
            reactor.register_callback(
                lambda et: self._announce_after_settle(),
                reactor.monotonic() + ANNOUNCE_SETTLE_S)
            return
        except Exception:
            # No usable reactor (early boot, or a shim in a test): announcing
            # immediately is still far better than not announcing at all.
            pass
        self._announce_after_settle()

    def _announce_after_settle(self) -> None:
        """
        The announce half of the reconnect handshake, run once the firmware has
        had ANNOUNCE_SETTLE_S to finish booting. Split out so it can be
        deferred onto the reactor instead of sleeping on the reader thread.
        """
        self._announce_unit()
        if self.unit_uid:
            self._resolve_uid_index(0)
        try:
            self._bridge.send({"cmd": "status"})
        except Exception:
            pass

    def _reset_bridge_once(self) -> bool:
        """
        Reboot the Pico, at most once per BRIDGE_RESET_COOLDOWN_S.

        The cooldown is load-bearing: the reset drops the USB link, which fires
        the reconnect handler, which calls this again. Without it that is a
        reboot loop, not a retry.

        Kept on the BRIDGE, not the unit -- several units share one Pico and
        must not each reboot it in turn.

        :return bool: True if a reset was sent (caller should stand down and
            let the reconnect that follows do the announcing)
        """
        br = self._bridge
        if br is None:
            return False
        now = _mono(self)
        last = getattr(br, "_last_reset_t", 0.0)
        if last and (now - last) < BRIDGE_RESET_COOLDOWN_S:
            return False
        br._last_reset_t = now
        try:
            br.send({"cmd": "reset"})
        except Exception:
            return False
        self.logger.debug(
            f"AFC bambu {self.name}: rebooting the bridge so the announce "
            f"lands on clean firmware state")
        return True

    def _uid_watch_tick(self, eventtime: float) -> float:
        """
        Periodic chain self-heal: request the chain map, then verify (after the
        async reply lands) that unit_uid still sits at our ams_index.

        :param eventtime: Reactor event time
        :return float: next wake time (30s cadence)
        """
        try:
            if self._bridge is not None and self.unit_uid:
                self._bridge.send({"cmd": "chain"})
                self.afc.reactor.register_callback(
                    lambda et: self._uid_watch_check(),
                    self.afc.reactor.monotonic() + 0.8)
        except Exception:
            pass
        return eventtime + 30.0

    def _uid_watch_check(self) -> None:
        """Adopt a moved chain index (unit re-enrolled mid-session); quiet
        no-op when the pin still matches."""
        try:
            uids = self._bridge.chain_uids() if self._bridge else []
            if self.unit_uid in uids:
                idx = uids.index(self.unit_uid)
                if idx != self.ams_index:
                    self.logger.warning(
                        f"AFC bambu {self.name}: chain moved mid-session -- UID "
                        f"{self.unit_uid} now at index {idx} (was "
                        f"{self.ams_index}); re-pinning")
                    self._adopt_index(idx)
        except Exception:
            pass

    def _startup_restore_loaded(self) -> None:
        """
        Re-assert the AMS loaded/follower state for lanes tool-loaded at boot.

        A lane can be tool-loaded across a reboot (restored from saved vars). The
        AMS itself comes up idle, so the extruder would pull against a dead motor.
        Re-assert the loaded state (mode-07 "finish") and engage the self-centering
        follower for the lane so the AMS is ready to feed as the extruder pulls.
        The follower is the AP2-sync heartbeat, not blind feed: the AMS keeps its
        own buffer centered and self-stops at center, so it holds pressure without
        fighting even while the printer is idle.
        """
        if self._bridge is None:
            return
        for lane in self.lanes.values():
            if getattr(lane, "tool_loaded", False):
                # Housekeeping on every restart with a lane loaded -- expected,
                # not an event the user acts on.
                if not self._ready_to_follow(lane):
                    # Steppers come up de-energised. Engaging here is what
                    # produces the post-restart pulsing: the AMS feeds to
                    # refill its buffer and nothing downstream is gripping the
                    # filament. Skipped, not lost -- the follow poll loop
                    # arms it as soon as the motors come on.
                    self.logger.debug(
                        f"AFC bambu {self.name}: {lane.name} loaded at "
                        f"startup on an unhomed machine with a de-energised "
                        f"extruder; leaving the follower for the poll loop to "
                        f"arm once either changes")
                    continue
                # Housekeeping on every restart with a lane loaded -- expected,
                # not an event the user acts on.
                self.logger.debug(
                    f"AFC bambu {self.name}: {lane.name} loaded at startup, "
                    f"re-asserting AMS loaded state + follower")
                self._engage_follower(lane)

    def _engage_follower(self, lane: Any) -> None:
        """
        Put a tool-loaded lane's AMS tray into mode:4 and hold it (follower).

        Order matters and must match the working load path: commit the loaded
        state FIRST (mode-07 "finish"), THEN select the tray (mode-09 on an
        already-loaded tray flips it to mode:4), THEN assist to hold mode:4 via
        the AP2 sync heartbeat. Doing finish AFTER select knocks the tray back
        out of mode:4, so assist ends up with nothing to hold and the follower
        never runs (LED solid, extruder can't pull) -- exactly the startup
        symptom. select must be the last mode-changing op before assist.

        :param lane: A lane whose filament is threaded to the toolhead
        """
        self.bridge_finish(lane)     # commit loaded (mode-07)
        self.select_lane(lane)       # mode-09 -> mode:4 (last mode change)
        self.set_feed_assist(lane, True)  # hold mode:4 via AP2 sync

    def _open_serial(self) -> Any:
        """
        Open the USB-CDC port to the Pico (pyserial imported lazily).

        :return Any: an open serial port
        """
        import serial
        return serial.Serial(self.serial_port, self.baud, timeout=0.1)

    def _make_logo(self, error: bool) -> str:
        """
        Build the PREP summary logo for this unit (house-style aligned box).

        :param error: Whether to build the error variant
        :return str: the logo text PREP prints after testing the unit's lanes
        """
        builder = _ams_box_logo_error if error else _ams_box_logo
        # unit_slots, not SLOTS_PER_UNIT: a 1-slot AMS HT draws one bay, not four.
        return builder("BambuAMS",
                       getattr(self, "unit_slots", self.SLOTS_PER_UNIT),
                       self.name)

    # -- PREP interface --

    def system_Test(self, cur_lane: Any, delay: float, assignTcmd: bool,
                    enable_movement: bool) -> bool:
        """
        PREP-time lane test: mirror bridge slot state onto the lane.

        Seeds prep/hub state from the bridge's cached slot info (present spool ->
        prep_state + staged-at-hub), keeps the virtual hub's live occupancy
        derived from tool_loaded, and assigns the lane's T-command. The bridge
        being offline (protocol bring-up) is reported but does not fail the lane.

        :param cur_lane: The lane to test
        :param delay: Prep delay between lanes (unused; no motion here)
        :param assignTcmd: Whether to (re)assign the lane's T-command
        :param enable_movement: Movement-enable flag (unused; no stepper)
        :return bool: True unless the bridge could not be created at all
        """
        msg = ''
        succeeded = True
        latest = self._bridge.latest_status() if self._bridge else None
        if self._bridge is None:
            msg = '<span class=error--text>BRIDGE NOT CONNECTED</span>'
            succeeded = False
            self.lane_not_ready(cur_lane)
        else:
            slot = self._slot_of(cur_lane)
            info = self._slots[slot] if slot is not None else {}
            prep, staged, live, msg = prep_lane_state(
                info, getattr(cur_lane, 'tool_loaded', False),
                self._unit_online(latest),
                fallback_material=getattr(cur_lane, 'material', None))
            cur_lane.prep_state = prep
            cur_lane.loaded_to_hub = staged
            # Virtual hub live occupancy: only while threaded to the toolhead.
            if self._is_virtual_hub(cur_lane):
                cur_lane._load_state = live
            if not prep:
                # Empty bay: LED off, lane idle.
                self.lane_not_ready(cur_lane)
                cur_lane.status = AFCLaneState.NONE
            else:
                # A present spool in an AMS bay IS staged and ready (there is no
                # separate load sensor), so mark it LOADED — mirrors AFC_ACE.
                self.lane_loaded(cur_lane)
                cur_lane.status = AFCLaneState.LOADED
                self.lane_illuminate_spool(cur_lane)
                # Surface the AMS tag PROFILE (material/color/temps) onto the lane.
                self._surface_slot_info(cur_lane, info)
        if assignTcmd:
            try:
                self.afc.function.TcmdAssign(cur_lane)
            except Exception as e:
                self.logger.warning(f"AFC bambu: TcmdAssign failed: {e}")
        try:
            cur_lane.send_lane_data()
        except Exception:
            pass
        try:
            cur_lane.do_enable(False)
        except Exception:
            pass                             # no drive stepper on these lanes
        self.logger.info(f"{cur_lane.name} tool cmd: {cur_lane.map} {msg}")
        try:
            cur_lane.set_afc_prep_done()
        except Exception as e:
            self.logger.warning(f"AFC bambu: set_afc_prep_done failed: {e}")
        return succeeded

    # Older AFC revisions call the lowercase name from PREP.
    system_test = system_Test

    # -- status mirroring --

    def _on_status(self, obj: dict) -> None:
        """
        Reactor callback: fold this unit's slice of a bridge status frame onto
        the lanes. On a daisy-chain, the shared bridge hands every unit the whole
        frame; we keep only the slots tagged with our ams_index.

        :param obj: A decoded bridge status event
        """
        try:
            for entry in obj.get("slots") or []:
                if entry.get("unit", 0) != self.ams_index:
                    continue
                info = bridge_slot_to_info(entry)
                idx = info.get("index")
                if isinstance(idx, int) and 0 <= idx < self.SLOTS_PER_UNIT:
                    self._slots[idx] = info
            self._sync_lanes()
        except Exception as e:
            msg = f"AFC bambu {self.name}: status apply error: {e}"
            if msg != self._status_err_last:
                self._status_err_last = msg
                self.logger.warning(msg)
            else:
                self.logger.debug(msg)

    def _unit_online(self, latest: Optional[dict]) -> bool:
        """
        Whether THIS unit's AMS is online in a bridge status frame.

        :param latest: A bridge status dict (or None)
        :return bool: True if this ams_index's unit reports online
        """
        if not latest:
            return False
        for u in latest.get("units") or []:
            if u.get("n") == self.ams_index:
                return bool(u.get("online"))
        return bool(latest.get("online"))     # single-unit fallback

    @staticmethod
    def _is_virtual_hub(lane: Any) -> bool:
        """
        Return whether the lane's hub is a virtual (pinless) hub.

        :param lane: Lane whose hub to inspect
        :return bool: True when the lane has a hub that reports is_virtual_pin()
        """
        hub = getattr(lane, 'hub_obj', None)
        return (hub is not None
                and hasattr(hub, 'is_virtual_pin')
                and hub.is_virtual_pin())

    # Lane states we must never overwrite from a passive status poll: a load,
    # unload, eject, or error is mid-flight and owns the lane's status.
    _ACTIVE_STATES = (AFCLaneState.TOOL_LOADED, AFCLaneState.TOOL_LOADING,
                      AFCLaneState.TOOL_UNLOADING, AFCLaneState.EJECTING,
                      AFCLaneState.ERROR)

    def _sync_lanes(self) -> None:
        """
        Push cached slot state onto each mapped lane (OpenAMS-style).

        Slot presence drives ``prep_state`` (filament inserted in the AMS bay).
        A present spool in an AMS bay IS staged and ready -- there is no separate
        hub/load sensor -- so we latch ``loaded_to_hub`` True and mark the lane
        LOADED, exactly like OpenAMS/AFC_ACE. That is what keeps Mainsail from
        ever showing "filament detected but not loaded": a detected spool always
        reads as staged/Loaded until it is tool-loaded or removed. We never touch
        a lane whose status is mid-operation (load/unload/eject/error).

        For a virtual hub, ``_load_state`` is the LIVE hub-occupancy signal -- the
        native AFC_hub aggregates ``any(lane.raw_load_state)`` -- so it stays True
        only while this lane's filament is threaded THROUGH the hub to the
        toolhead (``tool_loaded``), never for a merely-staged lane, or the lane's
        own load would trip the "hub not clear" gate.
        """
        for name, slot in self._slot_map.items():
            lane = self.lanes.get(name)
            if lane is None:
                continue
            info = self._slots[slot]
            if not info:
                continue
            present = info.get("present", False)
            lane.prep_state = present
            # Kick a tag scan on a freshly inserted spool (presence 0->1).
            self._maybe_auto_scan(slot, present, info)
            status = getattr(lane, "status", None)
            active = status in self._ACTIVE_STATES
            if present:
                # Staged and ready: latch staged-at-hub and show LOADED so it is
                # never rendered as "detected but not loaded".
                lane.loaded_to_hub = True
                if not active and status != AFCLaneState.LOADED:
                    try:
                        self.lane_loaded(lane)
                        self.lane_illuminate_spool(lane)
                    except Exception:
                        pass
                    lane.status = AFCLaneState.LOADED
            else:
                # Empty bay can't be staged; clear the latch so a re-inserted
                # spool re-runs the full load path.
                lane.loaded_to_hub = False
                if not active and status != AFCLaneState.NONE:
                    try:
                        self.lane_not_ready(lane)
                    except Exception:
                        pass
                    lane.status = AFCLaneState.NONE
            if self._is_virtual_hub(lane):
                lane._load_state = bool(getattr(lane, 'tool_loaded', False))
            self._surface_slot_info(lane, info)

    def _lane_for_slot(self, slot: int) -> Optional[Any]:
        """Return the lane mapped to ``slot`` on this unit, or None."""
        smap = getattr(self, "_slot_map", None) or {}
        lanes = getattr(self, "lanes", None) or {}
        return next((lanes.get(n) for n, sl in smap.items()
                     if sl == slot and n in lanes), None)

    def _is_ht(self) -> bool:
        """True if this unit is an AMS HT (device 0x1800). The HT scans its RFID
        itself on its preload switch, so the firmware -- not the module -- drives
        the scan (armed on the insert edge)."""
        return bool(self.has_heater) and getattr(self, "dry_dev_addr", 0) == 0x1800

    def _send_ht_flag(self, bridge: Any) -> None:
        """Tell the firmware whether this unit is an AMS HT, so it arms the RFID
        scan on the slot's insert edge (device 0x1800). Harmless for boxed AMS."""
        if bridge is None:
            return
        try:
            bridge.send({"cmd": "htunit", "unit": self.ams_index,
                         "on": 1 if self._is_ht() else 0})
        except Exception:
            pass

    def _send_mc_addr(self, bridge: Any) -> None:
        """
        Point this unit's MC poll set at ITS OWN device and id.

        The captured frames our firmware replays are all addressed to 0x0700
        with the payload byte 0x01. A real printer addresses every poll to the
        unit's own device with an id of <class base> | <chain index> --
        0x1800/0x00 for a lone HT, 0x0700/0 for a lone boxed AMS, as decoded
        from live bus captures. The index is what lets two units of the
        same class share a wire. On an HT bus our entire poll set was going to
        a device that is not present, which is why an HT-only bus never
        followed and never narrated on demand.

        :param bridge: The bridge to notify
        """
        if bridge is None:
            return
        # base | chain index: the base is the unit CLASS, the low bits are its
        # position on the wire. That is how two units of the same class are
        # told apart -- a boxed AMS has base 0x00 so it is just the index.
        pay = (self.mc_ams_id if self.mc_ams_id >= 0
               else (self.mc_id_base | self.ams_index))
        try:
            bridge.send({"cmd": "mcaddr", "unit": self.ams_index,
                         "addr": int(self.mc_dev_addr), "pay": int(pay)})
        except Exception:
            pass

    def _send_selfcentre_flag(self, bridge: Any) -> None:
        """
        Tell the firmware whether this unit refills its own buffer.

        An AMS 2 Pro and an AMS HT do: they run their own refill cycles off
        their own sensor ("BUFF,pos:0.10->0.74, det:28mm, i:0.521A"), so the
        feeder poke may hold off until the buffer has genuinely sagged, which
        cuts pointless motor commands. A regular AMS does not -- it reports
        mode:4 continuously while the buffer drains to the arm threshold, so
        holding off there lets the buffer empty before the follower re-engages.

        The firmware cannot tell the models apart; only the config knows.

        :param bridge: The bridge to notify
        """
        if bridge is None:
            return
        try:
            mask = getattr(bridge, "_selfcentre_mask", 0)
            bit = 1 << self.ams_index
            mask = (mask | bit) if self.self_centres else (mask & ~bit)
            bridge._selfcentre_mask = mask
            bridge.send({"cmd": "selfc", "mask": mask})
        except Exception:
            pass

    def _maybe_auto_scan(self, slot: int, present: bool, info: dict) -> None:
        """
        Trigger an RFID/tag scan when a spool is newly inserted into a bay.

        Fires once on the presence 0->1 edge for a slot, latched until the spool
        is removed. A scan physically moves the filament (feed past the reader +
        slow retract), so it is skipped during a print and when tag data is
        already present. Disabled by ``auto_scan: False``.

        :param slot: 0-based AMS slot index on this unit
        :param present: Whether the AMS currently reports a spool in the slot
        :param info: Normalized slot info (used to skip if already tagged)
        """
        if not (0 <= slot < len(self._prev_present)):
            return
        # Phantom-bay guard: a 1-slot AMS HT can report garbage bits for bays it
        # doesn't have -- never log inserts for them or scan them.
        if slot >= getattr(self, "unit_slots", len(self._prev_present)):
            return
        was_present = self._prev_present[slot]
        self._prev_present[slot] = present
        # Startup baseline: spools already inserted at boot must NOT look like
        # fresh 0->1 inserts (all _prev_present start False). Until primed, just
        # record presence -- don't scan or log -- so a reboot never re-reads what
        # AFC restored from saved vars at prep. Real edges after priming scan.
        if not getattr(self, "_scan_primed", True):
            if not present:
                self._auto_scanned[slot] = False
            return
        # Log every bay transition (all slots) so inserts/removals are visible.
        if present and not was_present:
            self.logger.info(
                f"AFC bambu {self.name}: spool INSERTED in slot {slot} "
                f"(AMS bay {slot + 1})")
        elif was_present and not present:
            self.logger.info(
                f"AFC bambu {self.name}: spool REMOVED from slot {slot} "
                f"(AMS bay {slot + 1})")
        if not present:
            # Removal edge: clear the slot's cached profile so the previous
            # spool's material/color doesn't linger, and so the next insert reads
            # fresh (the AMS HT never re-reads on its own -- without this its old
            # tag would be reapplied on a swap). Spoolman-linked lanes stay
            # authoritative.
            if was_present:
                lane = self._lane_for_slot(slot)
                if lane is not None and getattr(lane, "spool_id", None) in (
                        None, "", 0):
                    self._clear_lane_filament(lane)
            self._auto_scanned[slot] = False        # reinsertion re-scans
            return
        if (not self.auto_scan or was_present
                or self._auto_scanned[slot] or self._bridge is None):
            return
        afc = getattr(self, "afc", None)
        try:
            if afc is not None and afc.function.in_print():
                return                               # never move filament mid-print
        except Exception:
            pass
        # A genuine insert edge -> read the tag. Same trigger for every AMS type:
        # the module makes the scan call, the AMS carries it out (self.scan aims
        # at this unit's dry_dev_addr, so 0x1800 for the HT). We do NOT skip when
        # the slot already shows material -- a swapped-in spool must re-read, and
        # the HT never re-reads on its own, so its cached tag is the prior spool's.
        # First blank any stale profile so the UI doesn't show the old spool's
        # material/color/weight until the new tag reads. Spoolman is authoritative.
        lane = self._lane_for_slot(slot)
        if lane is not None and getattr(lane, "spool_id", None) in (None, "", 0):
            self._clear_lane_filament(lane)
        self._auto_scanned[slot] = True
        # The AMS HT scans its own tag on its preload switch -- the FIRMWARE arms
        # the 0x1800 poll the instant it sees the insert edge (no module round-
        # trip, no settle), which is the only way to catch the HT's scan while it
        # runs. So for the HT we do NOT send a scan here (that would just read its
        # stale flash and could clobber the firmware's min-window). The boxed AMS
        # scans by feeding past its bay reader, which the module drives via scan().
        # Both branches restate the INSERTED line above with the mechanism that
        # scans -- useful when a tag doesn't turn up, not on every insert.
        if self._is_ht():
            self.logger.debug(
                f"AFC bambu {self.name}: new spool in slot {slot}; HT scans it on "
                f"insert (firmware-driven at 0x1800)")
        else:
            self.logger.debug(
                f"AFC bambu {self.name}: new spool detected in slot {slot}, "
                f"scanning tag")
            self.scan(slot)
        # After the scan has had time to read, fall back to the lane defaults if
        # no readable tag turned up (missing/unreadable tag). Deferred so it can't
        # fire mid-scan and clobber a tag still landing.
        try:
            afc.reactor.register_callback(
                lambda et, s=slot: self._finalize_scan(
                    s, afc.reactor.monotonic() + self.SCAN_FALLBACK_CAP),
                # The HT's burst scan can take ~15-20s end to end (clear ->
                # select -> feed-with-rfid -> auth -> save -> read); a 14s
                # fallback raced it and flashed misleading "no readable tag"
                # defaults moments before the real tag landed. Give the HT the
                # full window; boxed AMS read much faster and keep the snappier
                # fallback.
                afc.reactor.monotonic() + (25.0 if self._is_ht() else 14.0))
        except Exception:
            pass

    def _clear_lane_filament(self, lane: Any) -> None:
        """
        Blank a lane's filament profile (material/color/weight/Bambu SKU) so a
        previous spool's data doesn't linger in the UI until a fresh tag reads or
        defaults are applied. Best-effort per attribute.

        :param lane: The AFC lane object
        """
        for attr, val in (("material", ""), ("color", ""), ("weight", 0)):
            try:
                setattr(lane, attr, val)
            except Exception:
                pass
        try:
            lane.bambu_sku = ""
        except Exception:
            pass

    def _finalize_scan(self, slot: int, cap: Optional[float] = None) -> None:
        """
        Deferred: after an auto-scan window, apply the lane's AFC defaults for a
        bay whose tag never read (no tag, or unreadable). A tag that DID read has
        already been applied by ``_surface_slot_info`` and leaves ``material``
        set, so this is a no-op then.

        Re-arms itself instead of applying defaults while the AMS is audibly
        still reading (see ``_RFID_INFLIGHT_RE``), up to ``cap``. A boxed AMS 1
        was measured landing a real tag 13.0s after the insert edge against this
        14.0s fallback -- a one-second margin, and the same race that already had
        to be fixed once for the HT. A fixed window cannot be picked safely for a
        step whose duration the unit chooses, so wait on the unit instead.

        :param slot: 0-based AMS slot index on this unit
        :param cap: Reactor monotonic time past which we stop waiting and apply
                    defaults regardless. None = no deferral (direct calls/tests).
        """
        if apply_filament_defaults is None:
            return
        if not (0 <= slot < len(self._slots)):
            return
        info = self._slots[slot]
        if not info or not info.get("present") or info.get("material"):
            return                                   # gone, or a tag read in time
        if cap is not None and self._bridge is not None:
            try:
                now = self.afc.reactor.monotonic()
                if now < cap and self._bridge.rfid_read_in_flight(now):
                    self.afc.reactor.register_callback(
                        lambda et, s=slot, c=cap: self._finalize_scan(s, c),
                        now + 2.0)
                    return
            except Exception:
                pass
        lane = next((self.lanes.get(n) for n, s in self._slot_map.items()
                     if s == slot and n in self.lanes), None)
        if lane is None:
            return
        if getattr(lane, "spool_id", None) not in (None, "", 0):
            return                                   # Spoolman-linked -> leave it
        if getattr(lane, "material", None) not in (None, ""):
            return                                   # already has a material
        afc = getattr(self, "afc", None)
        try:
            apply_filament_defaults(
                lane, info,
                afc_defaults={
                    "default_material_type": getattr(
                        afc, "default_material_type", None),
                    "default_color": getattr(afc, "default_color", None),
                })
            self.logger.info(
                f"AFC bambu {self.name}: no readable tag in slot {slot}; "
                f"applied lane defaults to {lane.name}")
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: default apply for slot {slot} "
                f"failed: {e}")

    def _surface_slot_info(self, lane: Any, info: dict) -> None:
        """
        Apply the AMS tag's PROFILE to a lane, base-ACE style: material, color,
        Bambu type code, and print temps — only when the lane doesn't already
        have them (a manual/Spoolman value wins). No unique UID exists, so this
        surfaces material/color rather than identifying a unique spool.

        :param lane: The AFC lane object
        :param info: Normalized slot info from bridge_slot_to_info
        """
        tag_material = info.get("material")
        if tag_material and tag_material.lower() == "unknown":
            tag_material = None
        # A Spoolman-linked lane (spool_id set) is authoritative -- leave it be.
        has_spool = getattr(lane, "spool_id", None) not in (None, "", 0)

        if tag_material and not has_spool:
            # The AMS tag is the source of truth for this bay. Apply it DIRECTLY,
            # overwriting any value auto-set from an AFC default -- the shared
            # helper's "only if empty" rule would otherwise let a default
            # applied before the tag was read lock out the real tag.
            color = info.get("color")
            color_hex = (color if color.startswith("#") else "#" + color) \
                if color else None
            tmin = info.get("temp_min")
            material, sub_type = _split_bambu_material(tag_material)
            changed = (getattr(lane, "material", None) != material
                       or getattr(lane, "sub_type", None) != sub_type
                       or (color_hex and getattr(lane, "color", None) != color_hex))
            lane.material = material
            # The three fields the tag implies but never carried into the lane,
            # so every surface that shows a spool -- the dryer panel, Spoolman,
            # the RFID notifications -- had a blank vendor and no variant.
            lane.sub_type = sub_type
            lane.spool_vendor = BAMBU_BRAND
            if build_filament_name is not None:
                # Same builder the ACE 2 RFID path uses, so both vendors render
                # a spool identically ("Bambu PLA Matte").
                lane.filament_name = build_filament_name(
                    BAMBU_BRAND, material, sub_type)
            if color_hex:
                lane.color = color_hex
            if tmin is not None:
                try:
                    lane.extruder_temp = float(tmin)
                except (TypeError, ValueError):
                    pass
            # A spool with 0 weight renders as empty/hidden in the UI. The AMS has
            # no scale, but the tag carries the nominal filament weight (1 kg / 250 g
            # mini); use it, defaulting to a full 1 kg spool, only when unset.
            if not getattr(lane, "weight", 0):
                tag_w = info.get("weight")
                try:
                    lane.weight = int(tag_w) if tag_w else 1000
                except (TypeError, ValueError):
                    lane.weight = 1000
            if changed:
                self.logger.info(
                    f"AFC bambu {self.name}: applied tag to {lane.name}: "
                    f"{getattr(lane, 'filament_name', '') or tag_material} "
                    f"{color_hex or ''}".rstrip())
        # No readable tag yet (a bay is staged but not yet fed past the reader):
        # do NOT apply an AFC default here. The tag arrives after the scan feeds
        # the spool past the reader, and a default applied on stage would show
        # the wrong material until (and lock out) the real tag. Leave the lane's
        # material untouched; the tag lands when the scan reads it.

        # Bambu profile code (e.g. GFA00) — a nice sub_type hint for Spoolman.
        if info.get("sku") and getattr(lane, "bambu_sku", None) in (None, ""):
            lane.bambu_sku = info["sku"]
        lane.bambu_slot_info = info

    def _record_fresh(self, rec: dict) -> bool:
        """
        Whether an address-keyed chamber record is recent enough to trust.

        :param rec: a record from _chamber_record().
        :return bool: True while it is under 120s old.
        """
        mono = getattr(self.reactor, "monotonic", None)
        nowm = mono() if callable(mono) else 0.0
        return (not nowm) or (nowm - rec.get("seen", 0.0) < 120.0)

    def _chamber_telemetry_fresh(self) -> bool:
        """
        Whether the bridge's last [AMS_CHMB] record is recent enough to trust.

        The AMS streams it only while a cycle runs, so a finished dry would
        otherwise leave a frozen reading looking live.

        :return bool: True while the last record is under 120s old.
        """
        seen = getattr(self._bridge, "_chmb_t_seen", 0.0)
        mono = getattr(self.reactor, "monotonic", None)
        nowm = mono() if callable(mono) else 0.0
        return (not nowm) or (nowm - seen < 120.0)

    def _chamber_record(self) -> Optional[dict]:
        """
        This unit's own chamber telemetry, by device address.

        The AMS's text frames carry the address that sent them at bytes [7:8],
        and it is per-model -- 0x0700 on an AMS 2 Pro, 0x1800 on an HT, which
        is exactly this unit's ``dry_dev_addr``. When the bridge firmware
        reports it (1.0.7.0+), several units on one bridge keep their chambers
        apart and two can dry at once without ambiguity.

        :return dict: {"temp", "target", "state", "seen"} for this unit, or
            None when nothing addressed to it has arrived.
        """
        by_addr = getattr(self._bridge, "_chmb_by_addr", None)
        if not by_addr:
            return None
        return by_addr.get(int(self.dry_dev_addr))

    def _owns_chamber_telemetry(self) -> bool:
        """
        Whether this unit may claim the bridge's chamber telemetry as its own.

        The telemetry arrives on the LOG DRAIN, which is addressed to 0x0700 --
        a bus-wide address, not a unit. (It is also the long frame dialect, so
        it cannot be retargeted with frame_for_unit without corrupting its
        length.) The firmware therefore cannot say which AMS narrated, and the
        text carries no unit id: a shared reading was being served to every
        unit on the bridge, so a drying HT would have published its chamber
        temperature as an idle AMS 2's as well.

        Only a unit in a dry cycle can be producing it, which resolves the
        common case exactly. When two units on the same bridge are drying at
        once nothing distinguishes their lines, so neither claims it: a blank
        readout is honest, a confidently wrong temperature is not.

        :return bool: True when this unit is the only one drying on its bridge.
        """
        # A bridge with only ONE heater-capable unit has no ambiguity to
        # resolve: nothing else on it can produce chamber telemetry. Claim it
        # regardless of self._drying, which is host state and is wrong after a
        # Klipper restart -- the AMS keeps drying, so the panel has to be able
        # to catch up to a cycle it did not start.
        heaters = [u for _, u in self.printer.lookup_objects("AFC_BambuAMS")
                   if getattr(u, "has_heater", False)
                   and getattr(u, "_bridge", None) is self._bridge]
        if len(heaters) <= 1:
            return bool(self.has_heater)
        if not self._drying:
            return False
        for _, other in self.printer.lookup_objects("AFC_BambuAMS"):
            if other is self or not getattr(other, "_drying", False):
                continue
            if getattr(other, "_bridge", None) is self._bridge:
                if not getattr(self, "_warned_chmb_share", False):
                    self._warned_chmb_share = True
                    self.logger.info(
                        f"AFC bambu {self.name}: another unit "
                        f"({getattr(other, 'name', '?')}) is drying on the "
                        f"same bridge -- the log drain is bus-wide and carries "
                        f"no unit id, so chamber temperature is reported for "
                        f"neither rather than guessed for both.")
                return False
        return True

    def _chamber_live(self) -> tuple:
        """
        Resolve this unit's chamber telemetry: is it live, can we attribute it,
        and when was it last seen.

        ``attributable`` says whether the ABSENCE of telemetry means anything
        for this unit -- either we hold an address-keyed record for it, or it is
        the only heater-capable unit drying on its bridge. Two same-address
        units drying at once cannot be told apart on a shared record, so we
        neither adopt nor release a cycle from it.

        :return tuple: (live, attributable, seen_time)
        """
        rec = self._chamber_record() if self._bridge is not None else None
        if rec is not None:
            return self._record_fresh(rec), True, rec.get("seen", 0.0)
        if self._bridge is not None and self._owns_chamber_telemetry():
            # Firmware too old to report the address: fall back to the shared
            # value, which is only safe when one unit is drying.
            live = (getattr(self._bridge, "_chmb_temp", None) is not None
                    and self._chamber_telemetry_fresh())
            return live, True, getattr(self._bridge, "_chmb_t_seen", 0.0)
        return False, False, 0.0

    def get_status(self, eventtime: Any = None) -> dict:
        """
        Extend the base unit status with the bridge's slot view.

        :param eventtime: Klipper eventtime (unused)
        :return dict: unit status including bridge online flag and slots
        """
        status = super().get_status(eventtime)
        latest = self._bridge.latest_status() if self._bridge else None
        status["bridge_online"] = self._unit_online(latest)
        status["ams_index"] = self.ams_index
        humidity, temperature = unit_env(latest, self.ams_index)
        status["humidity"] = humidity          # %RH, or None if unknown
        # Chamber temperature. The binary protocol carries none (temp_c10 is
        # -1), but a drying AMS streams it in its own telemetry. Treated as
        # stale after 120s so a finished dry cycle does not leave a frozen
        # reading looking live, and only read by the unit it belongs to --
        # see _owns_chamber_telemetry.
        rec = self._chamber_record() if self._bridge is not None else None
        # Resolve the chamber telemetry ONCE, here, and let every derived field
        # (temperature, drying, target, state) share the answer. They are all
        # readings of the same record and must agree; deriving them separately
        # is how the panel came to show a chamber temperature beside "Idle".
        live, attributable, seen_t = self._chamber_live()
        # Only telemetry produced AFTER the last start/stop says anything about
        # the current cycle. Without this, a stop was undone by the next status
        # poll: _drying went False, then readings from up to 120s earlier -- i.e.
        # from the cycle just ended -- re-adopted it. Pressing Stop appeared to
        # do nothing, and only a click landing after the window expired stuck.
        # The grace period matters as much as the stamp. A stopping AMS does not
        # go quiet instantly -- it emits at least one more [AMS_CHMB] line while
        # winding down, which is NEWER than the stop and so re-adopted the cycle
        # we just ended. On hardware that showed up as having to press Stop
        # twice. Give a stop a few reporting intervals (the AMS narrates roughly
        # every 10s) before live telemetry is allowed to win again; if the unit
        # really is still drying past that, adoption correctly re-arms and the
        # panel tells the truth that the stop did not take.
        fresh_for_this_cycle = live and seen_t > getattr(
            self, "_dry_adopt_after", 0.0)
        if temperature is None and fresh_for_this_cycle:
            temperature = (rec.get("temp") if rec is not None
                           else getattr(self._bridge, "_chmb_temp", None))
        status["temperature"] = temperature    # °C, or None when not drying
        status["slots"] = self._slots
        # Follower + buffer telemetry, surfaced like an FPS buffer so it can be
        # watched in the UI. buff is the AMS's FPS "fullness" 0..100, stated by
        # SPRING state because "compressed"/"stretched" invert depending on
        # whether you mean the spring or the buffer travel:
        #   100 = spring compressed, the two PTFE ends pushed APART (fed)
        #     0 = spring extended, PTFE ends together, bottomed out (feed me)
        # buffer_state mirrors AFC buffer wording: compressed (fed) / expanded
        # (demand) / neutral.
        buff = latest.get("buff") if latest else None
        status["follow_buff"] = buff              # 0..100 fullness
        status["buffer"] = buff                   # alias (FPS-style value)
        status["buffer_state"] = _buffer_state(buff)
        # How many times the firmware has actually decoded the buffer off the
        # wire. 0 means follow_buff is still the firmware's seed value, not a
        # reading. A seed value reads as a satisfied buffer and disables
        # anything gated on it, so surface the count rather than trusting the
        # number alone.
        status["follow_buff_reads"] = latest.get("buffn") if latest else None
        # Length of the last motion reply we tried to decode, and the raw 16-bit
        # field before calibration. A length <= 26 means the reply carries no
        # buffer at all on this AMS model; the raw value is what BUFF_POS_FULL /
        # BUFF_POS_EMPTY are calibrated against.
        status["follow_buff_replylen"] = latest.get("bufflen") if latest else None
        # Observability. follow_buff above is the firmware's MAPPED value, whose
        # calibration is known wrong on an AMS HT -- follow_buff_raw is the field
        # itself (signed LE) and is the one to trust.
        status["follow_buff_raw"] = latest.get("buffraw") if latest else None
        status["follow_state"] = latest.get("fstate") if latest else None
        # 0 means the AMS has never reported a follower state, so follow_state is
        # the firmware's seed (4) rather than a confirmation that it is following.
        status["follow_state_reads"] = latest.get("fstaten") if latest else None
        # The AMS's own reference id, from its "[AMS_COMMON]...ref:N" narration.
        # An AMS HT only acts on a SELECT addressed to this id, so a mismatch
        # here means tray selects are ignored and the unit reports tray:255.
        status["ams_ref"] = latest.get("amsref") if latest else None
        # The AMS's last self-reported stall, and the motor current it came
        # with. Surfaced so a fault is inspectable after the fact, not only at
        # the moment it paused.
        if self._bridge is not None:
            _seq, ftext, famps = self._bridge.last_fault()
            status["ams_fault"] = ftext or None
            status["ams_motor_amps"] = famps or None
        # True while a stall has the follower held off, waiting for a resume.
        status["follow_fault_hold"] = self._follow_fault_hold
        # Narration accounting. The AMS returns its PENDING log text in reply to
        # a 1A/02 poll, and an empty reply is ordinary traffic -- so a quiet
        # AFC.log cannot be read as "the unit said nothing". polls climbing with
        # frames flat means it is not answering the log drain at all; frames
        # climbing with texts flat means it is answering and has nothing queued.
        status["ams_narration_polls"] = latest.get("dbgpolls") if latest else None
        status["ams_narration_frames"] = latest.get("dbgframes") if latest else None
        status["ams_narration_texts"] = latest.get("dbgtexts") if latest else None
        # Raw result of the MC_ONLINE exchange alone, in a buffer no other poll
        # can overwrite. snap_empty climbing with snap_replies flat means the
        # AMS does not answer the log drain within REPLY_TIMEOUT_US; the
        # reverse means it answers and we are discarding it.
        status["ams_snap_replies"] = latest.get("snapn") if latest else None
        status["ams_snap_replies_p1"] = latest.get("snapn1") if latest else None
        status["ams_snap_empty"] = latest.get("snapempty") if latest else None
        # The two mechanisms we use to make the AMS feed. 0 means that mechanism
        # is not running at all, regardless of what the surrounding state says.
        status["follow_sync_frames"] = latest.get("syncn") if latest else None
        status["follow_pokes"] = latest.get("poken") if latest else None
        status["following"] = (self._following_lane.name
                               if self._following_lane is not None else None)
        # Drying is host state, set by BAMBU_HEATER_START -- so a Klipper
        # restart mid-cycle left the panel showing Idle beside a hot dryer.
        # Live chamber telemetry only streams WHILE a cycle runs, so its
        # presence is direct evidence from the unit itself; trust it over the
        # flag and the panel catches up to a cycle it did not start.
        if fresh_for_this_cycle:
            self._dry_seen_live = True
            self._drying = True   # adopt it, so a STOP from the panel is armed
        elif (self._drying and attributable
                and getattr(self, "_dry_seen_live", False) and not live):
            # The unit reported for this cycle and has now gone silent past the
            # staleness window: the cycle is over. Without this, a dryer that
            # finished its timer or was stopped at the unit left the panel
            # asserting "drying" forever, because _drying was host state that
            # only an explicit STOP ever cleared. Adoption is now symmetric.
            self._drying = False
            self._dry_seen_live = False
            self.logger.info(
                f"AFC bambu {self.name}: drying finished (the unit stopped "
                f"reporting chamber telemetry)")
        status["drying"] = bool(self._drying)
        # What the heater is DOING, for a UI. All three come from the AMS's own
        # [AMS_CHMB] telemetry, which only streams while a cycle is running, so
        # they share the same 120s staleness rule as the chamber temperature --
        # a finished cycle must not leave a frozen target looking live.
        status["has_heater"] = self.has_heater
        status["ams_model"] = self.ams_model
        status["dry_max_temp"] = self.dry_max_temp
        dry_target = None
        dry_state = None
        if fresh_for_this_cycle:
            if rec is not None:
                dry_target = rec.get("target")
                dry_state = rec.get("state")
            else:
                dry_target = getattr(self._bridge, "_chmb_target", None)
                dry_state = getattr(self._bridge, "_chmb_state", None)
        status["dry_target"] = dry_target      # C the AMS is driving to
        status["dry_state"] = dry_state        # AMS's own chamber state code
        # Why the unit last declined to dry, in its own words. The command is
        # DELIVERED in that case -- the AMS echoes our temp/time back before
        # refusing -- so we report success and the panel would otherwise just
        # sit at "not drying" with no reason.
        # NOT gated on self._drying. That is host INTENT -- set the moment
        # BAMBU_HEATER_START sends, whether or not the AMS accepted -- so
        # gating on it hid the reason in exactly the case it exists for: a
        # refused start, where we think we are drying and the unit is not.
        # The reason is cleared instead when the UNIT reports heating or a
        # self-check, which is machine state rather than our bookkeeping.
        status["dry_error"] = self._bridge_call_arg("last_dry_error",
                                                    self.dry_dev_addr)
        return status

    # -- transport primitives --

    def _slot_of(self, lane: Any) -> Optional[int]:
        """
        Return the 0-based AMS slot for a lane, or None if unmapped.

        :param lane: The AFC lane object
        :return Optional[int]: the slot index, or None
        """
        return self._slot_map.get(getattr(lane, "name", None))

    def fps_buffer_value(self) -> Optional[float]:
        """
        AMS buffer as a 0.0..1.0 FPS/PSF reading for the virtual ADC pin.

        We are an ANALOG buffer, so the convention that governs is the FPS/PSF
        one that AFC_buffer documents at the top of its FPS driver:

            0.1 (low)  -> stretched / tension    -> increase feed
            0.5 (mid)  -> centred / ideal
            0.9 (high) -> compressed / pushing   -> decrease feed
            aliases: max_tension -> low_point, max_compression -> high_point

        HIGH IS COMPRESSED. The sign matters because several callers decide
        things off this value and invert with it:

          - advance_state (smoothed > set_point + deadband/2), which is what
            get_toolhead_pre_sensor_state() returns when tool_start is
            "buffer".
          - buffer_triggered, the endstop-free load check.
          - the pre-feed guard, which refuses to load into an empty toolhead.
          - buffer ramming.

        With this polarity the readings land where the physics says they
        should: unloaded buff=1 -> 0.01 (tension), loaded and self-centred
        buff=56..60 -> ~0.58, a hair above the 0.5 set_point, which is what a
        unit that holds its own buffer centred reads at rest.

        :return Optional[float]: 0.0..1.0, or None if no reading yet
        """
        latest = self._bridge.latest_status() if self._bridge else None
        if not latest:
            return None
        b = latest.get("buff")
        if b is None:
            return None
        v = b / 100.0                      # compressed(buff 100)->1.0, empty(0)->0.0
        return 0.0 if v < 0.0 else 1.0 if v > 1.0 else v

    def _tool_loaded_lane(self) -> Optional[Any]:
        """
        Return a lane on this unit threaded to the toolhead AND belonging to
        the ACTIVE extruder, or None.

        Used to auto-arm the follower. On a toolchanger several lanes can be
        tool_loaded at once -- one per toolhead -- but only one extruder is
        active (receiving E moves); the rest are not pulling filament and need
        no follower.

        "Active" is Klipper's current extruder (AFC's get_current_extruder),
        NOT on_shuttle(): a docked toolhead can legitimately be the active
        extruder during async/pre-load, and gating on the shuttle would strip
        the follower exactly when that load needs it. Unknown/unwired cases
        fall back to on_shuttle(), then to "active", so single-toolhead and
        partially-configured setups never lose their follower.

        :return Optional[Any]: a tool-loaded, slot-mapped, ACTIVE lane, or None
        """
        current = None
        try:
            current = self.afc.function.get_current_extruder()
        except Exception:
            current = None
        for lane in self.lanes.values():
            if not getattr(lane, "tool_loaded", False):
                continue
            if self._slot_of(lane) is None:
                continue
            ext = getattr(lane, "extruder_obj", None)
            if ext is not None and current:
                name = getattr(ext, "th_extruder_name", None) or getattr(
                    ext, "name", None)
                if name and name != current:
                    continue            # another extruder is the active one
            elif ext is not None:
                # No active-extruder answer available -- fall back to the
                # shuttle test rather than following every loaded lane.
                try:
                    if not ext.on_shuttle():
                        continue
                except Exception:
                    pass                # can't tell -> treat as active
            return lane
        return None

    def _committed_lanes(self) -> list:
        """
        Lanes on this unit whose filament is advanced past the bay -- staged at
        the hub (``loaded_to_hub``) or threaded to the toolhead (``tool_loaded``).

        These are the lanes for which it is UNSAFE to spin the spool: rotating a
        committed spool fights the filament already in the path (jams the hub or
        yanks the toolhead). A spool merely sitting in its bay (``prep_state``
        only) is not committed and is safe to rotate -- that's normal drying.

        :return list: committed lane objects (empty if none)
        """
        out = []
        for lane in self.lanes.values():
            if (getattr(lane, "tool_loaded", False)
                    or getattr(lane, "loaded_to_hub", False)):
                out.append(lane)
        return out

    def select_lane(self, lane: Any, sel_prep: bool = False) -> tuple:
        """
        Route this lane's AMS slot to the output.

        :param lane: The lane to select
        :param sel_prep: Whether this is a prep-time selection (unused here)
        :return tuple: (ok, slot) — ok False when the lane isn't mapped
        """
        slot = self._slot_of(lane)
        if slot is None or self._bridge is None:
            return (False, -1)
        self._bridge.send(
            {"cmd": "select", "unit": self.ams_index, "slot": slot})
        return (True, slot)

    def prime(self, lane: Any) -> bool:
        """
        Run the AMS feeder (bay -> hub) for this lane's tray before the hub feed.

        The hub motor (our normal feed) only moves filament already at the hub;
        the feeder is what pulls it out of the BAY. A freshly-inserted bay (any
        but bay 0, whose filament tends to be pre-primed near the hub) never
        advances on hub feed alone, so we prime it first. This is a bounded,
        feeder-ONLY firmware step -- running feeder and hub together makes the
        AMS oscillate feed/retract, so the hub stage stays separate.

        :param lane: The lane whose tray to prime
        :return bool: True if the command was issued
        """
        slot = self._slot_of(lane)
        if slot is None or self._bridge is None:
            return False
        self._bridge.send(
            {"cmd": "prime", "unit": self.ams_index, "slot": slot})
        return True

    def feed(self, lane: Any, mm: float, mmps: Optional[float] = None) -> bool:
        """
        Feed filament from a lane's slot toward the toolhead.

        :param lane: The lane to feed
        :param mm: Length to feed in mm
        :param mmps: Speed in mm/s, or None for the configured default
        :return bool: True if the command was issued
        """
        return self._move("feed", lane, mm,
                          mmps if mmps is not None else NOMINAL_MMPS)

    def retract(self, lane: Any, mm: float, mmps: Optional[float] = None) -> bool:
        """
        Retract filament back into a lane's slot.

        :param lane: The lane to retract
        :param mm: Length to retract in mm
        :param mmps: Speed in mm/s, or None for the configured default
        :return bool: True if the command was issued
        """
        return self._move("retract", lane, mm,
                          mmps if mmps is not None else NOMINAL_MMPS)

    def _move(self, cmd: str, lane: Any, mm: float, mmps: float) -> bool:
        """
        Issue a feed/retract bridge command with a clamped speed.

        :param cmd: "feed" or "retract"
        :param lane: The lane to move
        :param mm: Length in mm
        :param mmps: Requested speed in mm/s (clamped to max_speed)
        :return bool: True if the command was issued
        """
        slot = self._slot_of(lane)
        if slot is None or self._bridge is None:
            return False
        self._bridge.send({"cmd": cmd, "unit": self.ams_index, "slot": slot,
                           "mm": round(mm, 2),
                           "mmps": round(clamp_speed(mmps, MAX_MMPS), 2)})
        return True

    def set_feed_assist(self, lane: Any, on: bool) -> bool:
        """
        Start/stop the AMS self-centering follower for a lane.

        This is the AMS's own buffer-regulated feed (its "loaded/assist" mode:4):
        once loaded it keeps its buffer (FPS) centered as the extruder pulls --
        feeding a short pulse when the buffer drops toward its trigger and
        self-stopping once centered. The firmware sustains it by streaming the
        AP2 sync heartbeat (the ``assist`` command), NOT by blind mode-03 feed.
        Blind feed has no stop condition, over-feeds, and stall-retracts, which
        pulled the filament back off the toolhead sensor and caused the load
        "fight"; the follower self-stops at center so it holds pressure without
        fighting. Captured live from a real printer <-> AMS 2 Pro.

        :param lane: The lane to assist
        :param on: True to engage the follower, False to stop it
        :return bool: True if the command was issued
        """
        slot = self._slot_of(lane)
        if slot is None or self._bridge is None:
            return False
        self._bridge.send({"cmd": "assist", "unit": self.ams_index,
                           "slot": slot, "on": bool(on)})
        # Track which lane the demand-gated re-engage timer should watch. Reset
        # the extruder baseline so the first sample after a (re)engage or a
        # tool change doesn't read as a huge jump and fire a spurious feed.
        if on:
            self._following_lane = lane
            self._follow_last_e = None
        elif self._following_lane is lane:
            self._following_lane = None
            self._follow_last_e = None
        return True

    def _raise_ams_fault(self, lane: Any, msg: str) -> None:
        """
        Report a stall and, when configured to pause, hold the follower off.

        Re-arming into a jam is worse than doing nothing: the AMS keeps driving
        against filament it cannot move. So the pause path latches the hold and
        drops the assist, and the auto-arm stays out until the print resumes.

        :param lane: The lane that stalled
        :param msg: Operator-facing description of the fault
        """
        if not self.fault_pause:
            self.logger.warning(msg)
            return
        self._follow_fault_hold = True
        self._follow_fault_saw_pause = False
        self._starved_since = 0.0
        try:
            self.set_feed_assist(lane, False)
        except Exception as e:
            # A fault report must never be able to break the follower tick.
            self.logger.debug(
                f"AFC bambu {self.name}: could not drop assist on stall: {e}")
        self.afc.error.AFC_error(msg, pause=True)

    def _fault_hold_active(self) -> bool:
        """
        Whether the stall hold is still suppressing the follower auto-arm.

        Releases on resume, which is the operator saying the jam is cleared. The
        pause is not instant -- AFC_error queues it -- so the hold only releases
        after a paused state has actually been observed; otherwise the very next
        tick would see "not paused" and re-arm straight back into the jam.

        :return bool: True while the follower must stay disengaged
        """
        if not self._follow_fault_hold:
            return False
        try:
            paused = bool(self.afc.function.is_paused())
        except Exception:
            paused = False
        if paused:
            self._follow_fault_saw_pause = True
            return True
        if not self._follow_fault_saw_pause:
            return True
        self._follow_fault_hold = False
        self._follow_fault_saw_pause = False
        self.logger.info(
            f"AFC bambu {self.name}: print resumed, re-arming the follower.")
        return False

    def _check_ams_fault(self, lane: Any) -> None:
        """
        Raise an AFC error when the AMS reports it stalled.

        The unit says so itself -- "feed finish -1, stall", "switch_feed rocker
        stall", "pull err, bdc stall" -- and it knows things we cannot see
        (which motor, which tray, rocker state), so its report beats anything
        inferred from buffer position.

        Only fires while a lane is genuinely feeding the toolhead. A scan
        legitimately reports "bldc stall exit" as it ends its pull-in, and an
        unload retracts against resistance by design; treating either as a
        fault would stop a healthy machine.

        :param lane: The lane currently followed
        """
        # Defensive throughout: this runs inside the follower's reactor timer,
        # and a fault reporter must never be able to stop the follower itself.
        if not getattr(self, "fault_detect", False) or self._bridge is None:
            return
        if getattr(self, "_unload_in_progress", False):
            return
        if getattr(self, "_drying", False):
            return
        getf = getattr(self._bridge, "last_fault", None)
        if not callable(getf):
            return
        seq, text, amps = getf()
        if seq == getattr(self, "_fault_seen", 0):
            return
        self._fault_seen = seq
        if not text:
            return
        # "stall exit" is the scan path finishing its pull-in, not a failure.
        if "stall exit" in text.lower():
            return
        current = f", motor {amps:.2f}A" if amps else ""
        msg = (f"AFC bambu {self.name}: AMS reported a stall on {lane.name}"
               f"{current} -- check for a stuck spool or a jammed path.\n"
               f"AMS said: {text}")
        self._raise_ams_fault(lane, msg)

    def _check_buffer_starved(self, lane: Any, eventtime: float) -> None:
        """
        Raise an AFC error when the extruder keeps pulling but the buffer stays
        empty -- the AMS is not keeping up.

        Needed because not every unit narrates. An AMS HT reports its stalls in
        words ("feed finish -1, stall"), which _check_ams_fault acts on, but a
        boxed AMS 2 emits no narration at all: during a real stuck-spool test it
        produced zero AMS lines, so there was nothing to catch. Its buffer is
        live and accurate, so that is the signal there.

        Requires the buffer to stay below the starved threshold for the whole
        window AND the extruder to have advanced across it. Both are measured
        END TO END, never tick to tick: this runs every ~100ms, in which the
        extruder moves well under follow_min_extrude and the buffer counter has
        usually not ticked over, so a per-tick test resets the window on most
        ticks and it can never fill. That is exactly why a real 41s stuck-spool
        stall on an AMS 2 passed without a fault.

        A stale reading means "cannot judge": it neither opens nor clears the
        window, because a frozen telemetry path reads exactly like an empty
        buffer, and an already-accumulating starvation is not disproved by the
        absence of news.

        :param lane: The lane currently followed
        :param eventtime: Reactor event time
        """
        if not getattr(self, "fault_detect", False) or self._bridge is None:
            return
        if (getattr(self, "_unload_in_progress", False)
                or getattr(self, "_drying", False)
                or getattr(self.afc, "in_toolchange", False)):
            # A toolchange tips, unloads and purges: the buffer legitimately
            # bottoms out and the extruder is moving, which is the fault
            # signature exactly. AFC verifies its own loads, so nothing is lost
            # by standing down here -- and with a 2s window this guard is what
            # keeps a purge from reading as a jam.
            self._starved_since = 0.0
            return
        latest = self._bridge.latest_status() or {}
        buff, reads = latest.get("buff"), latest.get("buffn")
        try:
            e = self.afc.toolhead.get_position()[3]
        except Exception:
            return
        fresh = reads is not None and reads != self._starved_reads
        if fresh:
            self._starved_reads = reads
        if buff is None or buff > self.fault_starved_below:
            # Only a fresh reading is allowed to say "recovered".
            if fresh:
                self._starved_since = 0.0
            return
        if not fresh:
            return
        if not self._starved_since:
            self._starved_since = eventtime
            self._starved_e = e
            return
        if eventtime - self._starved_since < self.fault_starved_seconds:
            return
        self._starved_since = 0.0
        # Starved for the full window -- but only a fault if the extruder was
        # actually asking for filament over it. A paused or idle printer may
        # sit starved harmlessly.
        if e - self._starved_e < self.follow_min_extrude:
            return
        msg = (f"AFC bambu {self.name}: {lane.name} buffer has been empty "
               f"(fullness {buff}) for {self.fault_starved_seconds:.0f}s while "
               f"the extruder kept pulling -- the AMS is not keeping up. Check "
               f"for a stuck spool or a jammed path.")
        self._raise_ams_fault(lane, msg)

    def _follow_tick(self, eventtime: float) -> float:
        """
        Demand-gated follower keep-alive. When the followed lane's toolhead
        extruder has actually advanced (real extrusion), send a lightweight
        ``follow`` PING that opens/refreshes the firmware's re-arm window. The
        firmware then re-arms mode:4 at its own STEADY rate while the window is
        open, so the extruder can pull; when extrusion stops the window lapses
        and the follower goes silent (no idle twitch). Crucially the re-arm rate
        is fixed in firmware, so rapid extrude/retract cycles only keep the
        window open -- they never rapid-fire the re-arm. Reschedules itself.

        :param eventtime: Reactor event time
        :return float: next fire time
        """
        if self._bridge is None:
            return eventtime + self.follow_poll_interval
        if getattr(self, "_drying", False) and self._tool_loaded_lane() is None:
            # Drying with nothing threaded to the toolhead: the AMS is running
            # its self-check / heating cycle and the firmware holds the
            # follower off. Idle the tick until BAMBU_HEATER_STOP clears the
            # flag. When a lane IS loaded, fall through -- dry-while-printing
            # still needs the follower or the extruder fights the pull.
            return eventtime + self.follow_poll_interval
        lane = self._following_lane
        # Evaluated every tick and never short-circuited into the test below:
        # this call is also what RELEASES the hold once the print resumes.
        fault_hold = self._fault_hold_active()
        # Auto-arm: keep the follower engaged whenever a lane on this unit is
        # threaded to the toolhead, so it's always ready to feed. This does not
        # depend on the AMS buffer readback (often a stuck default here) or on the
        # per-lane extruder wiring (extruder_obj can be None) -- both of which
        # otherwise leave the follower engaged-but-inert (fstate:4 yet never
        # re-arming), letting the extruder bottom the buffer out before it feeds.
        # Held off after a stall: re-arming into a jam just grinds the filament.
        if (self.follow_when_loaded
                and not fault_hold
                and not getattr(self, '_follow_manual_off', False)
                and not getattr(self, '_unload_in_progress', False)):
            loaded = self._tool_loaded_lane()
            if loaded is not None and not self._ready_to_follow(loaded):
                # The lane is threaded to a toolhead whose extruder is NOT the
                # active one -- a docked tool on a toolchanger, or the state a
                # restart leaves behind before anything has homed or selected.
                # Engaging here is what produces the post-restart pulsing: the
                # AMS feeds to refill its buffer, nothing downstream is holding
                # the filament, so the buffer does not respond the way the
                # follower expects and it keeps poking. Observed directly --
                # homing, which made this unit's extruder active, stopped it.
                #
                # Only the ENGAGE is gated. An already-running follower is left
                # alone deliberately: standing one down mid-print on a
                # toolchange is a bigger behavioural change than this fixes,
                # and is not what the evidence covers.
                loaded = None
            if loaded is not None:
                if lane is not loaded:
                    # Never engaged, or the loaded lane changed -> (re)engage mode:4.
                    self._engage_follower(loaded)   # sets _following_lane
                    lane = self._following_lane
                else:
                    # Already following: re-assert mode:4 if the AMS fell out of it
                    # (it drops to state:0 when it thinks it's centred) so it can't
                    # go inert mid-print. RATE-LIMITED: this tick runs every
                    # ~100ms, and an unlimited re-assert becomes a 10/s assist
                    # storm that hammers the bus (and each assist-on cancels any
                    # retract stream in the firmware).
                    #
                    # Re-assert from state 0 ONLY, matching the firmware. "not
                    # 4" is wrong: an AMS HT follows happily at state:3 -- lane
                    # loaded, buffer held, feeding -- so that test was true on
                    # every tick and turned this into a 2s assist storm at a
                    # perfectly healthy unit. 0 is the one value the AMS pairs
                    # with "assist finish 0, ref:0", i.e. genuinely dropped.
                    st = self._bridge.latest_status()
                    fstate = st.get("fstate") if st is not None else None
                    demanded = (eventtime - self._follow_last_demand
                                <= self.follow_rearm_window)
                    if (fstate == AMS_MODE_IDLE and demanded
                            and eventtime - getattr(
                                self, "_follow_reassert_last", 0.0) >= 2.0):
                        self._follow_reassert_last = eventtime
                        self.set_feed_assist(loaded, True)
            elif lane is not None:
                # Nothing loaded from this unit anymore -> stop so it can't twitch.
                self.set_feed_assist(lane, False)
                lane = self._following_lane        # now None
        # Fault detection follows the LOADED lane, not the FOLLOWED one.
        # BAMBU_FOLLOWER ENABLE=0 clears _following_lane, and running without
        # assist is exactly when the buffer is most likely to starve, so the
        # detectors must keep watching the loaded lane regardless. The two
        # coincide almost always; they diverge on a manual stop and briefly
        # mid-engage.
        watch = lane
        if watch is None:
            try:
                watch = self._tool_loaded_lane()
            except Exception:
                watch = None
        if watch is not None and self._bridge is not None:
            # The AMS reports its own stalls; act on them while it is feeding.
            self._check_ams_fault(watch)
            # ...and watch the buffer, for units that do not narrate at all.
            chk = getattr(self, "_check_buffer_starved", None)
            if callable(chk):
                chk(watch, eventtime)
        if lane is not None and self._bridge is not None:
            # Telemetry: buffer position + follower state, for deep tuning only
            # (off by default). Rate-limited AND change-gated -- only emits when
            # buff/fstate/online actually change, so it never streams a line every
            # tick. Guarded so it can never disturb the keep-alive tick.
            dbg = getattr(self, "follow_debug_interval", 0.0)
            if dbg > 0.0 and (eventtime - getattr(self, "_follow_last_log", 0.0)
                              >= dbg):
                getst = getattr(self._bridge, "latest_status", None)
                st = getst() if callable(getst) else None
                if st is not None:
                    vals = (st.get("buff"), st.get("fstate"), st.get("online"))
                    if vals != getattr(self, "_follow_last_dbg", None):
                        self._follow_last_log = eventtime
                        self._follow_last_dbg = vals
                        self.logger.debug(
                            f"AFC bambu {self.name}: follow {lane.name} "
                            f"buff={vals[0]} fstate={vals[1]} online={vals[2]}")
            try:
                if self.follow_always or self.follow_idle_ping:
                    # Hold the firmware's re-arm window open every tick, so the
                    # AMS is fed continuously rather than only while the extruder
                    # advances. This is what actually moves filament today: the
                    # arm alone leaves the AMS in mode:4 without feeding, even
                    # with the buffer bottomed out (measured, 40s, no motion).
                    # It ticks audibly at idle, which is the known trade.
                    # NOT demand: this ping is unconditional, so counting it
                    # would make the re-arm gate below always-true and restore
                    # the every-2s assist storm it exists to prevent. Only real
                    # extruder movement counts.
                    self._bridge.send({"cmd": "follow"})
                    return eventtime + self.follow_poll_interval
                # Demand-gated: refresh the firmware's re-arm window ONLY when the
                # extruder actually advanced. Pinging every tick regardless
                # holds the window permanently open,
                # so the firmware's L2C re-arm fires at REARM_MS forever -- ~20
                # pokes a second with the printer sitting idle. On an AMS HT that
                # also means a blocking 0x1800 arm round-trip on repeat, which
                # disrupts its registration (audible ticking, then red lights).
                # The lane stays ARMED in mode:4 either way (the auto-arm above),
                # so it is still ready to feed the instant extrusion resumes.
                #
                # Uses the active extruder's E directly rather than this lane's
                # extruder_obj: _tool_loaded_lane() already returns a lane only
                # when its extruder is the active one, and extruder_obj can be
                # None, which would leave this path dead.
                e = self.afc.toolhead.get_position()[3]
                last = self._follow_last_e
                if last is not None and e - last >= self.follow_min_extrude:
                    # Real extrusion since last check -> refresh the window.
                    self._bridge.send({"cmd": "follow"})
                    self._follow_last_e = e
                    self._follow_last_demand = eventtime
                elif last is None or e < last:
                    # Establish/reset baseline (first sample, or a retract).
                    self._follow_last_e = e
            except Exception:
                pass
        return eventtime + self.follow_poll_interval

    def stop(self) -> bool:
        """
        Abort any in-flight AMS motion (all slots on this bridge).

        :return bool: True if the command was issued
        """
        if self._bridge is None:
            return False
        self._bridge.send({"cmd": "stop"})
        return True

    def relink(self) -> bool:
        """
        Force a firmware relink / error-recovery reset of the AMS chain: a
        deregister sweep followed by a fresh online-detect + re-registration.
        Recovers a unit stuck in a TIMEOUT/error (state:7) without a power cycle.

        :return bool: True if the command was issued
        """
        if self._bridge is None:
            return False
        self._bridge.send({"cmd": "relink"})
        return True

    def rehome(self) -> bool:
        """
        Run the AMS re-home motion only (mode 0F/0E, ~3s) -- the printer's "Retry"
        reset that clears a stuck/errored load (state:7) WITHOUT deregistering the
        chain, so a load can be re-attempted immediately. Lighter than relink()
        (which drops and re-registers the whole chain).

        :return bool: True if the command was issued
        """
        if self._bridge is None:
            return False
        self._bridge.send({"cmd": "rehome"})
        return True

    def scan(self, lane_or_slot: Any = None) -> bool:
        """
        Trigger an RFID/tag re-scan, optionally for one lane's slot.

        :param lane_or_slot: A lane object, a 0-based slot index, or None to
          scan every slot on this unit
        :return bool: True if the command was issued
        """
        if self._bridge is None:
            return False
        # For the AMS HT the filament-info read must target its 0x1800 device
        # (dry_dev_addr); a 0x0700 read returns nothing. Harmless for AMS2 Pro /
        # boxed AMS where dry_dev_addr is 0x0700 (the firmware's default).
        cmd: Dict[str, Any] = {"cmd": "scan", "unit": self.ams_index,
                               "addr": getattr(self, "dry_dev_addr", 0)}
        slot: Optional[int]
        if isinstance(lane_or_slot, int):
            slot = lane_or_slot
        elif lane_or_slot is not None:
            slot = self._slot_of(lane_or_slot)
        else:
            slot = None
        if slot is not None:
            cmd["slot"] = slot
        self._bridge.send(cmd)
        return True

    # Captured mode-07 "STEP7:finish" handoff (CRC-correct, from the real P1 load
    # capture -- docs/captures/p1_ams_live.txt lines 122-124). A genuine load
    # walks motion modes 01->03->09->07; mode 07 is the AMS's load-complete
    # ("STEP7:finish") signal. We only stream mode 03 (feed), so without this the
    # AMS never leaves the feed stage and retract-retries forever. Sending these
    # the instant the toolhead sensor triggers tells the AMS the filament reached
    # the extruder -- it commits the load and holds tension instead of retrying.
    _FINISH_FRAMES = (
        "3DC50CC803000900A502800C",   # mode 09 feeder->hub handoff
        "3DC50CC8030007000002514C",   # mode 07 gate
        "3DC50CC8030007007F023654",   # mode 07 finish (STEP7:finish)
    )

    def bridge_finish(self, lane: Any = None) -> bool:
        """
        Signal the AMS that the load is complete (mode-07 "STEP7:finish").

        Sends the captured mode 09->07 handoff frames via the bridge 'raw'
        passthrough. This is the piece that tells the AMS the filament reached
        the extruder so it stops the retract-and-retry and holds tension; our
        normal feed only streams mode 03, which the AMS treats as "still
        feeding". Send after the toolhead sensor confirms filament arrival.

        :param lane: Unused (finish is a bus-wide state transition); accepted for
          call-site symmetry with feed/retract.
        :return bool: True if the frames were issued
        """
        if self._bridge is None:
            return False
        for hexf in self._FINISH_FRAMES:
            self._bridge.send({"cmd": "raw", "hex": hexf})
        return True

    def bridge_unload(self, lane: Any) -> bool:
        """
        Run the AMS's multi-stage unload motion (hub retract -> feeder retract)
        via the bridge 'unload' command.

        :param lane: The lane to unload
        :return bool: True if the command was issued
        """
        slot = self._slot_of(lane)
        if slot is None or self._bridge is None:
            return False
        self._bridge.send({"cmd": "select", "unit": self.ams_index,
                           "slot": slot})
        self._bridge.send({"cmd": "unload", "unit": self.ams_index,
                           "slot": slot})
        return True

    # -- stepperless drive (AFC_lane.move_to hook) --

    def lane_move(self, lane: Any, distance: float,
                  speed_mode: Any = None) -> bool:
        """
        Firmware-driven lane move for a stepperless lane.

        AFC_lane.move_to routes moves here (there is no drive stepper): a
        non-negative distance feeds toward the toolhead, a negative distance
        retracts back toward the AMS bay.

        :param lane: The lane to move
        :param distance: Signed distance in mm (>=0 feed, <0 retract)
        :param speed_mode: AFC SpeedMode (unused; the AMS uses its own rate)
        :return bool: True if the command was issued
        """
        if distance >= 0:
            return self.feed(lane, distance)
        return self.retract(lane, abs(distance))

    def _wait_move(self, mm: float,
                   mmps: Optional[float] = None) -> bool:
        """
        Wait for a bridge move to finish, preferring the AMS's own report.

        The AMS announces completion itself ("[AMS_SWITCH]feed finish...",
        "[AMS_PRELOAD]preload finish..."), so wait for that rather than sleeping
        for a computed duration: the unit moves at its own speed, not the mm/s
        we ask for, and a distance/speed estimate is therefore only ever a
        guess. It also tells us whether the move actually succeeded -- a stall
        reports "finish -1".

        Falls back to the estimated duration as a timeout, so hardware that
        does not narrate still behaves as before.

        :param mm: Distance commanded in mm
        :param mmps: Commanded speed in mm/s
        :return bool: True if the AMS reported a successful completion; False
          if it reported a stall or nothing arrived before the timeout
        """
        # DEADLINE_MMPS, not the commanded speed: this sizes a watchdog, and
        # the AMS moves at its own rate regardless of what we asked for.
        speed = DEADLINE_MMPS if mmps is None else clamp_speed(mmps, MAX_MMPS)
        duration = (abs(mm) / speed) if speed > 0 else 0.0
        # Generous ceiling: the estimate is a lower bound on how long the AMS
        # may legitimately take, not an upper one.
        deadline_s = min(duration * 2.0 + 5.0, MOVE_DEADLINE_MAX_S)
        bridge = self._bridge
        start_seq = bridge.last_finish()[0] if bridge is not None else 0
        reactor = self.afc.reactor
        # TWO completion signals, whichever lands first.
        #
        # Narration was the only one, and it is text: _wait_move waited for the
        # word "finish" in the AMS's own log drain. When that drain went quiet
        # -- which it did, for hours, with the bus otherwise healthy -- every
        # move fell through to the deadline instead, and an unload that
        # physically finished in seconds took the full timeout to register.
        # Diagnosis belongs in narration; control flow should not depend on it.
        #
        # `fstate` is the AMS's own mode, carried as a typed field in EVERY
        # status frame: 0 idle -> 2 feeding -> 3 feed done -> 4 following ->
        # 1 assist. Watching it leave the moving state is the same completion
        # the narration announces, from a channel that cannot go quiet without
        # the whole link going down.
        #
        # Narration is kept as the first signal because it also reports
        # SUCCESS -- "finish -1"/"stall" -- which the mode alone does not. The
        # mode path returns True: it says the AMS stopped moving, not that it
        # got where it was asked, and the caller's own sensor check is what
        # decides that.
        # Track a CHANGE away from the mode we started in, not the specific
        # value 2. The documented sequence (0 idle -> 2 feeding -> 3 feed done
        # -> 4 following -> 1 assist) is the FEED path; a retract does not
        # necessarily pass through 2, so keying on that value meant every
        # unload missed this signal and fell through to the deadline -- which
        # is exactly what it was added to prevent.
        try:
            end = reactor.monotonic() + deadline_s
            while reactor.monotonic() < end:
                if bridge is not None:
                    seq, ok, _text = bridge.last_finish()
                    if seq != start_seq:
                        return ok
                reactor.pause(reactor.monotonic() + 0.1)
        except Exception:
            pass
        return False

    def _toolhead_sensor_triggered(self, cur_lane: Any) -> bool:
        """
        Whether the lane's toolhead pre-sensor (or buffer) reports filament.

        :param cur_lane: The lane whose toolhead sensor to read
        :return bool: True when filament is detected at the toolhead
        """
        try:
            return bool(cur_lane.get_toolhead_pre_sensor_state())
        except Exception:
            return False

    def _feed_until_sensor(self, cur_lane: Any,
                           timeout: Optional[float] = None) -> bool:
        """
        Drive the AMS forward until the toolhead sensor triggers, then STOP it.

        The AMS feeds continuously (mode 03) once kicked -- the commanded mm is
        advisory, so distance is bounded only by when we stop it. The AMS has its
        OWN load routine that stall-retries (feed, stall at the extruder, retract,
        retry) SEVERAL times before giving up, so the job here is to let that run
        and catch the filament the instant it reaches the sensor. We poll the
        sensor tightly (50 ms) but only re-kick on ``load_retry_interval`` -- slow
        enough that the AMS completes each of its own retry cycles between our
        nudges instead of being reset mid-attempt (which made it look like it
        "only tried once").

        :param cur_lane: The lane being loaded
        :param timeout: Seconds to keep trying (default ``load_retry_timeout``)
        :return bool: True once the sensor triggers (AMS stopped), else False
        """
        if timeout is None:
            timeout = self.load_retry_timeout
        if self._toolhead_sensor_triggered(cur_lane):
            self.stop()
            return True
        deadline = self.afc.reactor.monotonic() + timeout
        # Where the AMS's own arrival is measured FROM. Read before the loop so
        # a completion left over from the previous move cannot be mistaken for
        # this one's.
        start_seq = self._finish_seq_now()
        last_kick = -1.0
        kicks = 0
        while self.afc.reactor.monotonic() < deadline:
            now = self.afc.reactor.monotonic()
            if now - last_kick >= self.load_retry_interval:
                kicks += 1
                last_kick = now
                self.logger.debug(
                    f"AFC bambu {self.name}: feeding {cur_lane.name} to sensor "
                    f"(kick {kicks}); letting the AMS run its own retry")
                self.feed(cur_lane, self.load_retry_pulse)
            if self._toolhead_sensor_triggered(cur_lane):
                self.stop()          # halt instantly so the AMS can't retract it
                # A clean load is already narrated by AFC's load path, so only
                # say something when this loop had to nudge the AMS along --
                # that is the case worth noticing before it becomes a failure.
                if kicks:
                    self.logger.info(
                        f"AFC bambu {self.name}: {cur_lane.name} reached the "
                        f"toolhead sensor after {kicks} feed kick(s)")
                else:
                    self.logger.debug(
                        f"AFC bambu {self.name}: sensor triggered for "
                        f"{cur_lane.name}, AMS stopped")
                return True
            # The AMS's own arrival, SECOND -- the toolhead sensor is first by
            # AFC design and is checked above on every pass.
            #
            # The unit knows it got there without our sensor: an HT feeds to
            # the end of its measured PTFE and says so ("feed finish, ...
            # len_det:3.601 m, tube_len:3.619 m"), a boxed AMS zeroes the
            # tray odometer ("STEP:odom reset tray 0"). Both reach us as a
            # completion the bridge has already judged for distance, so a feed
            # that stalled SHORT does not count.
            #
            # Measured on both units once the path length was calibrated: the
            # sensor triggers 1-2 s BEFORE the AMS reports arrival, so on a
            # sensored lane this changes nothing -- it is what makes a lane
            # with no toolhead sensor loadable at all, and what replaces a
            # silent 90 s timeout with a completion when a sensor fails.
            if self.ams_arrival_completes_load:
                arrived, ok = self._finish_since(start_seq)
                if arrived and ok:
                    self.stop()
                    self.logger.info(
                        f"AFC bambu {self.name}: {cur_lane.name} accepted as "
                        f"loaded on the AMS's own arrival report after "
                        f"{kicks} feed kick(s) -- the toolhead sensor did not "
                        f"trigger")
                    return True
            try:
                self.afc.reactor.pause(now + 0.05)
            except Exception:
                break
        self.stop()
        return False

    def _finish_seq_now(self) -> int:
        """
        The bridge's current motion-completion sequence number.

        :return int: the sequence, or 0 when there is no bridge
        """
        br = self._bridge
        if br is None:
            return 0
        try:
            return int(br.last_finish()[0])
        except Exception:
            return 0

    def _finish_since(self, start_seq: int) -> Tuple[bool, bool]:
        """
        Whether the AMS has reported a move completion since ``start_seq``.

        :param start_seq: sequence captured before the move was commanded
        :return Tuple[bool, bool]: (a completion arrived, it reported success)
        """
        br = self._bridge
        if br is None:
            return (False, False)
        try:
            seq, ok, _text = br.last_finish()
        except Exception:
            return (False, False)
        return (int(seq) != int(start_seq), bool(ok))

    # -- load / unload lifecycle (AFC firmware-unit hooks) --

    def unit_load_lane(self, cur_lane: Any, cur_extruder: Any = None) -> bool:
        """
        Full toolhead load for a stepperless Bambu AMS lane.

        AFC.load_sequence dispatches firmware units here. Select the lane's slot,
        feed the configured bowden distance via the bridge, then poll/pulse the
        toolhead sensor until filament arrives. Mirrors AFC_ACE.unit_load_lane.

        :param cur_lane: Lane to load
        :param cur_extruder: Extruder the lane loads into (defaults to the lane's)
        :return bool: True on a verified load, False on failure
        """
        afc = self.afc
        # A real load supersedes any manual follower override, so a forgotten
        # BAMBU_FOLLOWER ENABLE=0 cannot leave the next print without assist.
        self._follow_manual_off = False
        self._follow_fault_hold = False
        self._follow_fault_saw_pause = False
        if cur_extruder is None:
            cur_extruder = getattr(cur_lane, "extruder_obj", None)
        if self._bridge is None:
            self.logger.warning(
                f"AFC bambu {self.name}: bridge not connected, cannot load "
                f"{cur_lane.name}")
            return False
        ok, _slot = self.select_lane(cur_lane)
        if not ok:
            self.logger.warning(
                f"AFC bambu {self.name}: lane {cur_lane.name} is not mapped to "
                f"an AMS slot")
            return False

        # Take the unit's own path measurement if it has one, and save it.
        # First load on a fresh install runs on DEFAULT_BOWDEN_MM, which is
        # long enough to let that load finish so the AMS can measure.
        self._adopt_measured_path()
        # Full path from the bay when not yet staged; hub->toolhead when staged.
        if getattr(cur_lane, "loaded_to_hub", False):
            feed_dist = self.afc_bowden_length
        else:
            feed_dist = self.afc_bowden_length + DIST_HUB_MM

        # Pre-feed guard: never push into an already-occupied toolhead.
        if self._toolhead_sensor_triggered(cur_lane):
            afc.error.handle_lane_failure(
                cur_lane,
                f"Toolhead sensor already detects filament before loading "
                f"{cur_lane.name}.\nClear the toolhead before loading (manually "
                f"retract or run AFC_RESET for {cur_lane.name}).",
                pause=afc.function.in_print())
            return False

        # Kick the AMS's continuous feed (mode 03 forward -- the direction the old
        # eject proved actually feeds) and tight-poll the toolhead sensor the
        # whole way, stopping the instant filament arrives. The commanded mm is
        # advisory: the AMS feeds until we stop it, and with no armed odometer it
        # stall-retries, so catching the sensor and halting fast is the whole
        # game. Deadline covers the bulk bowden plus the retry window.
        # DEADLINE_MMPS, for the same reason _wait_move uses it: this sizes a
        # give-up timeout, and the AMS moves at its own measured ~136 mm/s, not
        # the 20 mm/s we nominally command. Dividing by 20 made a 3250mm feed
        # allow 162 s before AFC would even consider the load finished --
        # observed as a load that "took forever to notify AFC it was done".
        #
        # This window is NOT the move watchdog and must not borrow its cap.
        # MOVE_DEADLINE_MAX_S bounds how long we wait to hear that ONE move
        # finished; this bounds how long the AMS is given to complete a load
        # INCLUDING its own feed/stall/retract/retry cycles, which is a
        # different quantity by an order of magnitude. Capping this at 35s put
        # the give-up below both the bulk feed time and load_retry_timeout (40s)
        # on its own, so every attempt was cut off mid-cycle and handed to our
        # re-home recovery -- three truncated attempts and 166s for a load that
        # the AMS finishes unaided, instead of one continuous window it can
        # actually retry inside.
        speed = DEADLINE_MMPS
        bulk_time = (feed_dist / speed) if speed > 0 else 0.0
        timeout = min(bulk_time + self.load_retry_timeout, LOAD_SENSOR_MAX_S)
        self.feed(cur_lane, feed_dist)
        loaded = self._feed_until_sensor(cur_lane, timeout)
        # Printer "Retry": the AMS ran its own retries and still stalled (likely a
        # latched state:7). Re-home the AMS (mode 0F/0E reset) and feed again --
        # exactly what pressing Retry on the printer does (re-home -> re-feed,
        # missing once then succeeding in the captured recovery). Bounded.
        recover = 0
        while not loaded and recover < self.load_recover_attempts:
            recover += 1
            self.logger.info(
                f"AFC bambu {self.name}: load of {cur_lane.name} stalled; "
                f"re-homing AMS and retrying (recover {recover}/"
                f"{self.load_recover_attempts})")
            self.stop()                          # halt before the reset motion
            self.rehome()                        # ~3s mode-0F/0E re-home reset
            self.feed(cur_lane, feed_dist)       # re-attempt the load
            loaded = self._feed_until_sensor(cur_lane, timeout)
        if not loaded:
            # The AMS retries loads on its own. By default DON'T reel the filament
            # back on a miss -- yanking the tray back mid-retry fights the AMS's
            # own attempts. Leave it staged so it (or the user) can try again;
            # only unwind when explicitly asked (reel_back_on_load_fail).
            reeled = False
            if self.reel_back_on_load_fail:
                self.stop()      # stop before reversing the direction
                try:
                    self.retract(cur_lane, feed_dist)
                    self._wait_move(feed_dist)
                    self.bridge_unload(cur_lane)  # AMS multi-stage unwind to bay
                    cur_lane.loaded_to_hub = False
                    reeled = True
                except Exception as e:
                    self.logger.warning(
                        f"AFC bambu {self.name}: unload-back after failed load "
                        f"of {cur_lane.name} did not complete: {e}")
                finally:
                    self.stop()
                tail = ("reeled it back to the bay" if reeled
                        else "could not reel it back -- clear the path manually")
            else:
                # Leave the AMS as-is (still staged/retrying); do not stop it.
                tail = ("left it staged -- the AMS keeps retrying; re-run the "
                        "load or clear the path manually if it can't finish")
            afc.error.handle_lane_failure(
                cur_lane,
                f"Filament did not reach the toolhead sensor for "
                f"{cur_lane.name} within {timeout:.0f}s; {tail}.\nCheck the "
                f"filament path and afc_bowden_length calibration.",
                pause=afc.function.in_print())
            return False

        # Sensor hit and the AMS is already stopped (inside _feed_until_sensor).
        # Tell it the load is complete (mode-07 "STEP7:finish") so it commits.
        self.bridge_finish(cur_lane)

        # Advance the last stretch from the toolhead sensor to the nozzle and
        # engage the AMS's self-centering follower -- then LEAVE IT RUNNING for
        # the print. Sustained by the AP2 sync stream (mode:4), the AMS keeps its
        # own buffer (FPS) centered as the extruder consumes filament: it feeds a
        # pulse when the buffer drops (~0.08) and stops on its own once centered
        # (~0.73), captured live from a real printer. Because it self-stops at
        # center it does NOT stall-retract, pull the filament back off the
        # sensor, or fight the toolhead the way blind continuous feed did. The
        # follower is cleared on unload / stop().
        #
        # The follower only holds if the AMS actually thinks the tray is
        # LOADED (mode:4). After bridge_finish the tray's buffer is compressed
        # (filament reached the extruder), so re-selecting it (mode-09 feeder on
        # an already-loaded tray) flips it straight to mode:4 -- confirmed live:
        # `select <loaded-tray> <unit>` -> mode:4,ref:165, held by `assist`.
        # Do the select+assist BEFORE the tool_stn advance so the follower is
        # already engaged as the extruder pulls the last stretch, otherwise the
        # LED stays solid (idle) and the extruder can't pull the filament.
        tool_stn = getattr(cur_extruder, "tool_stn", 0) or 0
        try:
            cur_lane.activate_toolhead_extruder()
            self.select_lane(cur_lane)           # mode-09 -> mode:4 (loaded)
            self.set_feed_assist(cur_lane, True)  # hold mode:4 via AP2 sync
            if tool_stn > 0:
                afc.move_e_pos(tool_stn, cur_extruder.tool_load_speed, "tool stn")
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: tool_stn advance failed for "
                f"{cur_lane.name}: {e}")
            self.stop()

        cur_lane.loaded_to_hub = True
        cur_lane.status = AFCLaneState.TOOL_LOADED
        # Acknowledge anything the AMS complained about DURING this load. A
        # boxed AMS retries a reluctant bay by itself -- "switch_feed rocker
        # stall, tray_cnt:0,16,0,0" is its own retry counter climbing -- and
        # those reports stayed queued with an unhandled sequence number. The
        # load then succeeded, the follower armed, and its first tick replayed
        # a stall the unit had already recovered from: a spurious fault, a
        # latched follower hold, and the assist dropped on a lane that was
        # correctly loaded. Reaching the toolhead sensor IS the verdict on
        # those stalls; only trouble after this point is the follower's to
        # report.
        self._ack_faults()
        afc.save_vars()
        return True

    def _ack_faults(self) -> None:
        """
        Mark every fault the AMS has reported so far as already handled, so a
        later check cannot re-raise it.

        :return None:
        """
        getf = getattr(self._bridge, "last_fault", None) if self._bridge else None
        if callable(getf):
            try:
                self._fault_seen = getf()[0]
            except Exception:
                pass

    def unit_unload_lane(self, cur_lane: Any, cur_extruder: Any = None) -> bool:
        """
        Full toolhead unload for a stepperless Bambu AMS lane.

        Runs the shared toolhead phase (quick pull, buffer disable, sync, select,
        cut/tip-form), retracts the configured bowden distance via the bridge to
        stage the tip near the hub, then finalizes lane state. Mirrors
        AFC_ACE.unit_unload_lane.

        :param cur_lane: Lane to unload
        :param cur_extruder: Extruder the lane is synced to (defaults to lane's)
        :return bool: True on success, False on failure
        """
        afc = self.afc
        if cur_extruder is None:
            cur_extruder = getattr(cur_lane, "extruder_obj", None)
        if self._bridge is None:
            self.logger.warning(
                f"AFC bambu {self.name}: bridge not connected, cannot unload "
                f"{cur_lane.name}")
            return False

        cur_lane.status = AFCLaneState.TOOL_UNLOADING
        self._unload_in_progress = True
        try:
            # Shared toolhead phase. do_tool_cut_tip_form self-gates on
            # tool_cut/form_tip, so it's a no-op when both are disabled.
            afc.move_e_pos(-2, cur_extruder.tool_unload_speed, "Quick Pull",
                           wait_tool=False)
            cur_lane.disable_buffer()
            cur_lane.sync_to_extruder()
            cur_lane.select_lane()
            afc.do_tool_cut_tip_form(cur_lane, cur_extruder)

            # Stop feed assist before winding back, then unsync so the extruder
            # gears release the filament for the AMS to reel in. Also send a
            # hard STOP first: a failed load can leave the AMS mid feed/retry
            # (still streaming mode 03), in which state it swallows the retract.
            # This is the same discipline the eject path uses.
            self.set_feed_assist(cur_lane, False)
            self.stop()
            cur_lane.unsync_to_extruder()

            # 1) FULL STN unload: drive the tip fully out of the extruder gears
            #    and WAIT for it to finish before the AMS starts reeling, so the
            #    filament has cleared the hotend/gears first.
            if cur_extruder.tool_stn_unload > 0:
                afc.move_e_pos(cur_extruder.tool_stn_unload * -1,
                               cur_extruder.tool_unload_speed, "STN unload",
                               wait_tool=True)
            # 2) Reel the bowden back into the AMS, and at the SAME TIME run a
            #    second STN-unload retract (non-blocking) so the extruder gears
            #    keep spinning -- actively driving the filament toward the AMS --
            #    while the AMS pulls it back. This keeps the tip moving through
            #    the gears as the AMS reels, instead of the AMS dragging against
            #    stationary/holding gears (which can strip or jam the filament).
            #    Queue the extruder retract first (async), then kick the AMS so
            #    the two overlap.
            retract_dist = self.afc_unload_bowden_length
            if cur_extruder.tool_stn_unload > 0:
                afc.move_e_pos(cur_extruder.tool_stn_unload * -1,
                               cur_extruder.tool_unload_speed,
                               "STN unload (concurrent with AMS reel)",
                               wait_tool=False)
            self.retract(cur_lane, retract_dist)
            self._wait_move(retract_dist)

            # VERIFY the filament actually left the toolhead — a fire-and-
            # forget retract that the AMS ignores (busy/error state) would
            # otherwise report "unload done" with the filament never having
            # moved. If the toolhead sensor still sees filament,
            # re-kick with the eject discipline (stop -> select -> retract),
            # and fail loudly if it still won't clear.
            for attempt in range(2):
                try:
                    still_loaded = bool(
                        self._toolhead_sensor_triggered(cur_lane))
                except Exception:
                    still_loaded = False
                if not still_loaded:
                    break
                # Include the AMS's own slot state ("retracting" = it is
                # reeling; "idle"/"empty" = it ignored the command) so the log
                # distinguishes a mechanical jam from a dead command.
                try:
                    slot = self._slot_of(cur_lane)
                    info = (self._slots[slot]
                            if slot is not None and 0 <= slot < len(self._slots)
                            else {})
                    ams_state = (info or {}).get("state", "?")
                except Exception:
                    ams_state = "?"
                self.logger.warning(
                    f"AFC bambu {self.name}: toolhead sensor still sees "
                    f"filament after retract for {cur_lane.name} (AMS slot "
                    f"state: {ams_state}) — re-kicking (stop/select/retract, "
                    f"attempt {attempt + 1})")
                self.stop()
                self.select_lane(cur_lane)
                self.retract(cur_lane, retract_dist)
                self._wait_move(retract_dist)
            else:
                afc.error.handle_lane_failure(
                    cur_lane,
                    f"AFC bambu unload failed for {cur_lane.name}: filament "
                    f"still at the toolhead sensor after retract retries — "
                    f"AMS did not reel; run LANE_UNLOAD (eject) or check the "
                    f"unit",
                    pause=afc.function.in_print())
                return False
            # The filament is home, so STOP asking where it is. Without this
            # the bridge stays in retract motion after the reel finishes and
            # keeps polling the target tray, which the unit answers ~2 Hz with
            #
            #   [AMS_DEV] STEP:odom tray_id error 255          (boxed AMS)
            #   [AMS_SWITCH]SWITCH_pull ignore. idx_pull:255   (HT)
            #
            # -- "there is no tray" -- until an internal deadline expires.
            # Measured: 12 exchanges over 34 s after every unload, audible at
            # the unit, and pure noise on a bus that other units share.
            #
            # It is also the same signal _wait_move now completes on, which is
            # why this belongs here and not in a mute: the first answer is
            # wanted, every one after it is a question we should have stopped
            # asking.
            self.stop()

            # Do NOT send the bridge 'unload' command here: bb_do_unload() in
            # the current firmware replays its captured frames RAW (no unit
            # re-addressing), so it streams a 5s retract at whatever unit the
            # capture came from — the WRONG unit on a multi-AMS chain. The
            # addressed retract above already reels fully (eject uses exactly
            # that and works); re-enable this once the firmware re-addresses.
            # self.bridge_unload(cur_lane)

            if afc.post_unload_macro is not None:
                self.gcode.run_script_from_command(afc.post_unload_macro)

            cur_lane.set_tool_unloaded(normal_toolchange=True)
            cur_lane.status = AFCLaneState.NONE
            # Tip is staged near the hub, ready for a fast reload.
            cur_lane.loaded_to_hub = True
            # tool_loaded is cleared above; refresh the virtual-hub occupancy now
            # so a lane->lane toolchange doesn't bail "hub not clear" before the
            # next hardware poll (mirrors AFC_ACE's _set_hub_state(.., False)).
            if self._is_virtual_hub(cur_lane):
                cur_lane._load_state = False
            afc.save_vars()
            return True
        except Exception as e:
            afc.error.handle_lane_failure(
                cur_lane,
                f"AFC bambu unload failed for {cur_lane.name}: {e}",
                pause=afc.function.in_print())
            return False
        finally:
            self._unload_in_progress = False

    # -- PREP / eject helpers (base afcUnit overrides) --

    def prep_load(self, lane: Any) -> None:
        """
        No-op: the AMS drives filament to its bay itself; presence is read from
        the bridge status rather than a prep move.

        :param lane: The lane being prepped (unused)
        """
        return

    def prep_post_load(self, lane: Any) -> None:
        """
        Latch a present spool as staged-at-hub after a successful prep, and ARM
        the follower for a lane already threaded to the toolhead.

        AFC calls this per lane during startup prep, after it has restored saved
        lane state -- so ``tool_loaded`` is valid here (unlike the fixed-delay
        startup timer). A lane tool-loaded across a reboot must have its AMS put
        back into mode:4 now, or the extruder pulls against a dead motor until the
        first manual load. So: if the lane is tool-loaded, engage the follower.

        :param lane: The lane just prepped
        """
        slot = self._slot_of(lane)
        info = self._slots[slot] if slot is not None else {}
        if info.get("present"):
            lane.loaded_to_hub = True
        if (getattr(lane, "tool_loaded", False)
                and self._bridge is not None and slot is not None):
            if not self._ready_to_follow(lane):
                # Same gate as _startup_restore_loaded, and it belongs here
                # too: prep runs on every boot, so without it this path
                # re-armed the follower against a de-energised extruder and
                # reintroduced the post-restart pulsing by the back door.
                # Deferred, not lost -- the follow poll loop arms it as soon
                # as the motors come on or the machine is homed.
                self.logger.debug(
                    f"AFC bambu {self.name}: {lane.name} tool-loaded at prep "
                    f"but the machine is unhomed with a de-energised "
                    f"extruder; deferring the follower to the poll loop")
                return
            # Startup housekeeping on any reboot with a lane loaded.
            self.logger.debug(
                f"AFC bambu {self.name}: {lane.name} tool-loaded at prep, "
                f"engaging follower")
            self._engage_follower(lane)

    def get_lane_reset_command(self, lane: Any, dis: float) -> str:
        """
        Return the gcode AFC_LANE_RESET / AFC_RESET should run to reset this
        lane. Stepperless AMS lanes can't be reset by moving a lane stepper --
        only the bridge can reel the filament back -- so route AFC_RESET to
        BAMBU_RECOVER (stop + reel to bay + reset state), the same recovery the
        eject path uses. Mirrors AFC_ACE.get_lane_reset_command.

        :param lane: Lane to reset
        :param dis: Reset distance (unused; the recover reels the full path)
        :return str: the BAMBU_RECOVER command for this unit and lane
        """
        return f"BAMBU_RECOVER UNIT={self.name} LANE={lane.name}"

    def measured_path_mm(self) -> Optional[float]:
        """
        This unit's PTFE path length as the AMS itself measured it, in mm.

        The unit self-calibrates from consecutive feeds and narrates the
        result, so this is the real distance on this machine rather than a
        configured estimate. Returns None until it has calibrated (it reports
        0 before that) or on firmware too old to attribute narration.

        :return Optional[float]: measured path in mm, or None
        """
        br = self._bridge
        if br is None:
            return None
        try:
            return br.tube_len(getattr(self, "dry_dev_addr", None))
        except Exception:
            return None

    def _adopt_measured_path(self) -> None:
        """
        Take the AMS's own path measurement as afc_bowden_length and save it.

        The unit self-calibrates its PTFE length from consecutive feeds and
        narrates the result, which is the real distance on this machine --
        strictly better than anything an operator can measure by hand. Once it
        arrives it is adopted for this session AND written back to the config
        through AFC's normal ConfigRewrite path, so it survives a restart and
        shows up where the operator expects to find it. If the key is not in
        any .cfg yet (the new-user case, since the default is not written out)
        ConfigRewrite files it in AFC_auto_vars.cfg, which is the same
        behaviour every other AFC calibration has.

        Runs once per session, and only when the figure actually differs:
        rewriting a file on every load would be noise, and the measurement
        wobbles by a few mm between calibrations.

        afc_unload_bowden_length follows only if it was tracking the bowden
        length (its default). Somebody who set it deliberately keeps it.
        """
        if getattr(self, "_path_adopted", False):
            return
        measured = self.measured_path_mm()
        if measured is None:
            # No measurement, and on a boxed AMS there never will be: that
            # dialect has no tube_len and no len_det in its vocabulary at all
            # (8 distinct line shapes, verified across a full load/unload).
            #
            # Which is fine, and deliberately not warned about. The path
            # length is ADVISORY -- it sizes the commanded distance and the
            # give-up window, nothing else. What ends a move is the toolhead
            # sensor or the unit's own report (odom reset / tray gone / feed
            # finish), so a unit that cannot measure itself still loads and
            # unloads correctly on whatever value it is given.
            return                      # nothing to adopt; the default stands
        old = self.afc_bowden_length
        if abs(measured - old) < PATH_ADOPT_TOLERANCE_MM:
            self._path_adopted = True   # already right; nothing to write
            return
        self._path_adopted = True
        follow_unload = (self.afc_unload_bowden_length == old)
        self.afc_bowden_length = measured
        if follow_unload:
            self.afc_unload_bowden_length = measured
        section = " ".join(self.full_name)
        msg = (f"AFC bambu {self.name}: the AMS measured its own filament "
               f"path at {measured:.0f}mm (was {old:.0f}mm). Adopting it -- "
               f"this sizes the load give-up deadline.")
        # Never let a config write break a load. Adopting the value in memory
        # has already happened above, so a failed save costs persistence, not
        # this print.
        try:
            self.afc.function.ConfigRewrite(
                section, "afc_bowden_length", round(measured, 1), msg)
            if follow_unload:
                self.afc.function.ConfigRewrite(
                    section, "afc_unload_bowden_length", round(measured, 1))
        except Exception as e:
            self.logger.warning(
                f"{msg} Could not save it to the config ({e}); it will be "
                f"measured again next session.")

    def _extruder_motor_enabled(self, lane: Any = None) -> bool:
        """
        Whether the extruder this lane feeds is actually energised.

        This is the condition a restart gets wrong. Klipper comes up with the
        steppers de-energised, and the loaded state is restored from saved
        vars, so the follower is engaged against an extruder that is not
        gripping anything. The AMS then feeds to refill its buffer, the
        filament simply moves because nothing downstream is holding it, the
        buffer does not respond the way the follower expects, and it keeps
        poking -- the post-restart pulsing. Homing ends it because homing
        energises the motors.

        Note this is NOT the same question as which extruder is SELECTED: the
        toolhead's selected extruder does not change across a restart or a
        home (measured -- it read "extruder" on both sides of the home that
        stopped the pulsing). Selected and energised are different things and
        only the second one moved.

        Fails OPEN on every uncertainty. A follower that will not engage lets
        the extruder bottom the buffer out mid-print, which is worse than a
        tick.

        :param lane: The loaded lane, whose extruder wins over the unit's
        :return bool: True when the extruder motor is energised, or unknown
        """
        name = getattr(lane, "extruder", None) or getattr(
            self, "extruder", None)
        if not name:
            return True
        try:
            se = self.printer.lookup_object("stepper_enable", None)
            if se is None:
                return True
            line = se.lookup_enable(name)
            if line is None:
                return True
            return bool(line.is_motor_enabled())
        except Exception:
            # lookup_enable raises for a name it does not know, and this runs
            # during startup where objects may not all exist yet.
            return True

    def _toolhead_homed(self) -> bool:
        """
        Whether the machine has been homed on all three axes.

        Used as the second half of _ready_to_follow. Homing is the operator
        saying the machine is live and about to be used, which is the point at
        which holding buffer pressure is wanted again.

        :return bool: True when x, y and z are all homed (or unknowable)
        """
        try:
            th = self.printer.lookup_object("toolhead", None)
            if th is None:
                return True
            axes = th.get_status(self.reactor.monotonic()).get(
                "homed_axes", "")
        except Exception:
            return True
        return all(a in axes for a in "xyz")

    def _ready_to_follow(self, lane: Any = None) -> bool:
        """
        Whether it is safe to engage the follower for this lane.

        Two ways to qualify, and it needs only one:

          * the extruder motor is energised -- something downstream is
            gripping the filament, so a feed compresses the buffer and the
            follower settles;
          * or the machine is homed -- G28 does not necessarily energise the
            EXTRUDER (measured: homed_axes "xyz" with the extruder stepper
            still disabled), but it does mean the operator has brought the
            machine up deliberately rather than it sitting cold from a
            restart.

        Motor-state alone was too tight: it left the follower disarmed after a
        home, so the AMS would not hold pressure on a machine that was plainly
        in use. Homing alone would be too loose, since it stays true forever
        after. Either-of-two keeps the cold-restart case -- unhomed AND
        de-energised, which is exactly what a reboot leaves -- as the only one
        that blocks.

        Deliberately says NOTHING about which tool is active. An earlier
        version also required the lane to be on the toolhead's current tool,
        which would block async loading into a DOCKED tool -- a lane being
        loaded while its tool is parked needs its follower exactly as much as
        one on the shuttle. Whether a tool is docked is not evidence about
        whether filament is being moved.

        This gates only the AUTOMATIC arming (startup restore and the follow
        poll loop). An explicit load still engages the follower through
        unit_load_lane regardless, so nothing here can stop a deliberate
        load.

        :param lane: The loaded lane, whose extruder wins over the unit's
        :return bool: True when the follower may engage
        """
        return self._extruder_motor_enabled(lane) or self._toolhead_homed()

    def _eject_distance(self) -> float:
        """
        How far to command the eject retract.

        Prefers the AMS's own measured path plus ``eject_buffer``, falling back
        to the configured estimate. Deliberately takes the LARGER of the two:
        on this path a short distance leaves filament in the tube with no
        sensor to notice, while a long one just means the AMS finishes early
        and says so.

        :return float: distance in mm
        """
        configured = (self.afc_unload_bowden_length + DIST_HUB_MM
                      + self.eject_buffer)
        measured = self.measured_path_mm()
        if measured is None:
            return configured
        return max(configured, measured + self.eject_buffer)

    def eject_lane(self, lane: Any) -> None:
        """
        Reel a lane's filament fully back into the AMS bay via the bridge. This
        is the shared reel-back core used by the AFC eject flow (LANE_UNLOAD),
        BAMBU_RECOVER, and AFC_RESET (via get_lane_reset_command).

        :param lane: The lane to eject
        """
        if self._bridge is None:
            self.logger.warning(
                f"AFC bambu {self.name}: bridge not connected, cannot eject "
                f"{lane.name}")
            return
        # A failed load can leave the AMS mid feed/retry (still streaming mode
        # 03), which fights a fresh retract. Halt it first, then select and reel
        # the filament fully back into the bay. This path must work regardless of
        # the lane's error state so a stuck load can always be recovered.
        self._unload_in_progress = True
        try:
            self.stop()
            self.select_lane(lane)
            dist = self._eject_distance()
            self.retract(lane, dist)
            finished = self._wait_move(dist)
            self.stop()
            if not finished:
                # This path has NO sensor: when the AMS reports a completion we
                # know the filament is home, and when it does not, _wait_move
                # returns on its deadline and the stop() above is what ends the
                # retract. Time x the AMS's own speed then decides how far it
                # actually came back -- which may be short. Silently treating
                # that as success is how a lane ends up half-ejected with
                # nothing in the log, so say it plainly.
                # getattr: eject is a RECOVERY path -- it runs from
                # BAMBU_RECOVER and AFC_RESET when things are already wrong,
                # so a warning about a partial eject must not itself raise on
                # a lane object that is missing a field.
                self.logger.warning(
                    f"AFC bambu {self.name}: "
                    f"{getattr(lane, 'name', 'lane')} eject stopped on a "
                    f"timeout, not on the AMS's own completion report -- the "
                    f"filament may not be fully back in the bay. Check the "
                    f"bay before loading it again.")
        finally:
            self._unload_in_progress = False
        lane.loaded_to_hub = False
        if self._is_virtual_hub(lane):
            lane._load_state = False


def load_config_prefix(config: Any) -> afcBambuAMS:
    """
    Klipper entry point for a prefixed [AFC_BambuAMS <name>] section.

    :param config: The Klipper config object for the unit section
    :return afcBambuAMS: the configured unit
    """
    return afcBambuAMS(config)
