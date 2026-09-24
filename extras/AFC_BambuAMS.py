# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# AFC unit for a stock Bambu Lab AMS (AMS, AMS 2 Pro, AMS HT; the AMS Lite is a
# future addition, neither implemented nor tested) driven
# over its own RS-485 bus by a Pico running the bambu_ams_bridge firmware. The
# Pico is the bus master; this module talks to it over USB-CDC or TCP with a
# newline-JSON API, maps each AMS slot to an AFC lane, mirrors slot status onto
# those lanes and issues select / feed / retract / assist / scan / dry.
#
# The bridge firmware and its protocol notes are released separately as a
# flashed device. Spoolman is opt-in through [AFC_BambuAMS_rfid].
#
# Minimal config (one boxed AMS on a virtual hub):
#
#   [AFC_hub BambuAMS_hub]
#   switch_pin: virtual
#
#   [AFC_BambuAMS BambuAMS_1]
#   serial_port: /dev/serial/by-id/...   # the Pico's USB-CDC port, not an [mcu]
#   # tcp_key: <host:port>               # or the Pico 2 W over WiFi
#   ams_model: ams1                      # ams1 | ams2 | ht
#   hub: BambuAMS_hub
#   extruder: extruder
#
#   [AFC_lane lane15]
#   unit: BambuAMS_1:1                   # <unit>:<slot>, slots 1..4
#
# More than one AMS on the wire: set unit_uid per unit (AFC_BAMBU_UIDS lists
# them). Chain order reshuffles across power cycles, so ams_index alone can
# address the wrong unit.
#
#   [AFC_BambuAMS BambuAMS_HT]
#   serial_port: /dev/serial/by-id/...   # same Pico
#   ams_model: ht
#   ams_index: 1
#   unit_uid: 872C3B871C00B0084A343331
#   hub: BambuAMS_hub2
#
# Optional settings, all commented where they are read: unit_uid, tcp_key,
# bus_serial, mc_dev_addr, mc_id_base, mc_ams_id, rollcall_span_boxed,
# rollcall_span_ht, afc_bowden_length, afc_unload_bowden_length, eject_buffer,
# tool_bite_mm, arrival_assist_delay_s, load_retry_timeout, load_retry_interval,
# load_retry_pulse, load_recover_attempts, reel_back_on_load_fail,
# auto_error_recovery, auto_error_recovery_limit, follow_when_loaded,
# follow_poll_interval,
# follow_rearm_window, follow_min_extrude, follow_debug_interval, ht_0f_hold,
# auto_scan, measure_on_insert, calibrate_on_insert, insert_pullin,
# link_loss_pause_s, fault_detect, fault_pause, sync_measured_to_spoolman,
# auto_spoolman_create, heater, dry_max_temp, variant.
#
# Commands (per-unit ones take UNIT=<name>; the ones that act on one lane,
# SCAN, CAPSCAN, REID, RECOVER, FOLLOWER, FEED and PRIME, take LANE=<lane>
# alone and find its unit):
#   AFC_BAMBU_UIDS            list bus UIDs and firmware version
#   AFC_BAMBU_SCAN            re-read a tag, works on a seated spool
#   AFC_BAMBU_CAPSCAN         scan and measure the spool
#   AFC_BAMBU_AUTOSCAN        toggle scan-on-insert
#   AFC_BAMBU_HEATER_START    start drying (ams2/ht); AFC_BAMBU_HEATER_STOP
#   AFC_BAMBU_RECOVER         reel a failed load back to the bay
#   AFC_BAMBU_RELINK          clear a TIMEOUT/error state
#   AFC_BAMBU_CLEARFAULT      drop a latched fault; refuses while still jammed
#   AFC_BAMBU_FAULT_RELOAD    reload the lane after a fault, then resume
#   AFC_BAMBU_FOLLOWER        engage or stop the follower
#   AFC_BAMBU_FEED / PRIME / BITE / ARRIVAL / REMEASURE / CALIBRATE
#   AFC_BAMBU_SAVEIDS / REID  bind or re-identify units by UID
#   AFC_BAMBU_BUFFER_PROBE    read the buffer sensor chain
#   AFC_BAMBU_FLASH           flash the bridge firmware over the link
# Each prints its usage when run without arguments.

from __future__ import annotations

import traceback
from configparser import Error as error
import json
import os
import re
import struct
import time
import zlib
from typing import Any, Callable, Dict, List, Optional, Tuple

# Transport lives in AFC_BambuAMS_bridge so it can be tested without a
# printer.
#
# UF2 unwrapping: the bridge flashes one contiguous image over serial, so the
# 512-byte blocks are unwrapped and any gap between them is refused.
UF2_BLOCK        = 512
UF2_MAGIC0       = 0x0A324655        # "UF2\n"
UF2_MAGIC1       = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30
UF2_FLAG_NOT_MAIN = 0x00000001       # "not a main flash block" -- skip it
UF2_FLAG_FAMILY  = 0x00002000
#: Family ids, verbatim from pico-sdk boot/uf2.h. They are not a contiguous
#: "RP2350" range: ABSOLUTE and DATA sit between RP2040 and the RP2350 variants
#: and are chip-agnostic, so an image's first block says nothing about its
#: target.
UF2_FAMILY_RP2040    = 0xE48BFF56
UF2_FAMILY_ABSOLUTE  = 0xE48BFF57
UF2_FAMILY_DATA      = 0xE48BFF58
UF2_FAMILY_RP2350_S  = 0xE48BFF59
UF2_FAMILY_RP2350_RV = 0xE48BFF5A
UF2_FAMILY_RP2350_NS = 0xE48BFF5B
#: family -> the chip it can only run on. ABSOLUTE and DATA are absent
#: deliberately: they name no chip.
UF2_FAMILY_CHIP = {
    UF2_FAMILY_RP2040:    "RP2040",
    UF2_FAMILY_RP2350_S:  "RP2350",
    UF2_FAMILY_RP2350_RV: "RP2350",
    UF2_FAMILY_RP2350_NS: "RP2350",
}
RP2040_FLASH_BASE = 0x10000000
# Where a no_flash image's blocks point (same on RP2040 and RP2350). Used only
# to refuse such an image by name rather than as a generic "not a whole image".
SRAM_BASE = 0x20000000
# The largest hole uf2_to_image will fill rather than refuse. A sparse image
# legitimately skips erased regions; a jump of megabytes is a file holding two
# separate images, which must not be flattened into one.
FW_MAX_GAP = 64 * 1024


def uf2_to_image(blob: bytes):
    """
    Flatten a UF2 into the contiguous image the bridge's flash writer wants.

    :param blob: The whole .uf2 file
    :return bytes: The image, starting at the first byte of flash
    """
    if len(blob) < UF2_BLOCK or len(blob) % UF2_BLOCK:
        error_str = f"not a UF2: {len(blob)} bytes is not whole 512-byte blocks"
        raise ValueError(error_str)
    out = bytearray()
    want = None
    chips = set()
    for off in range(0, len(blob), UF2_BLOCK):
        b = blob[off:off + UF2_BLOCK]
        m0, m1, flags, addr, size, blkno, _n, fam = struct.unpack("<8I", b[:32])
        if m0 != UF2_MAGIC0 or m1 != UF2_MAGIC1 or \
                struct.unpack("<I", b[-4:])[0] != UF2_MAGIC_END:
            error_str = (f"block {off // UF2_BLOCK} has no UF2 magic -- "
                             "is this really a .uf2?")
            raise ValueError(error_str)
        if flags & UF2_FLAG_NOT_MAIN:
            continue                     # metadata, not flash content
        if flags & UF2_FLAG_FAMILY:
            if fam not in UF2_FAMILY_CHIP and fam not in (UF2_FAMILY_ABSOLUTE,
                                                          UF2_FAMILY_DATA):
                error_str = (f"block {blkno} has unknown chip family "
                             f"0x{fam:08X}")
                raise ValueError(error_str)
            if fam in (UF2_FAMILY_ABSOLUTE, UF2_FAMILY_DATA):
                # Not part of the image, though not flagged as such. picotool's
                # --abs-block marker (block 0 of an RP2350 image) has family
                # ABSOLUTE, address 0x10FFFF00 and flags 0x0000A000, with bit 0
                # (NOT_MAIN_FLASH) clear, so the flag check above misses it.
                # Taken as content it would set the start address to
                # 0x10FFFF00 and make every real block look like a gap.
                continue
            chip = UF2_FAMILY_CHIP.get(fam)
            if chip is not None:
                if chips and chip not in chips:
                    error_str = (f"image mixes chips: {sorted(chips)[0]} and "
                                 f"{chip}")
                    raise ValueError(error_str)
                chips.add(chip)
        if size > UF2_BLOCK - 36:
            error_str = (f"block {blkno} claims {size} payload bytes, "
                             "more than a block holds")
            raise ValueError(error_str)
        if want is None:
            if addr != RP2040_FLASH_BASE:
                # A no_flash image (the shape RP2350 encrypted boot requires)
                # lands here: its blocks target SRAM (0x20000000), and the
                # bridge's updater would write it to flash offset 0 and brick
                # the board. Name that case separately from a truncated file.
                where = ("SRAM" if addr >= SRAM_BASE else
                         f"0x{addr:08X}")
                extra = ("" if addr < SRAM_BASE else
                         " -- this is a no_flash (RAM) image, such as an "
                         "encrypted-boot build. Those are loaded by the "
                         "bootrom over USB, never written to flash and never "
                         "sent over the air")
                extra = extra or " -- this is not a whole firmware image"
                error_str = (f"starts at {where}, not the start of flash "
                    f"(0x{RP2040_FLASH_BASE:08X}){extra}")
                raise ValueError(error_str)
            want = addr
        if addr < want:
            # Going backwards is corruption: two blocks writing the same
            # address means one silently wins, depending on file order.
            error_str = (f"block {blkno} targets 0x{addr:08X}, behind "
                f"0x{want:08X} which comes next; blocks overlap")
            raise ValueError(error_str)
        if addr > want:
            # A forward gap is a hole, and holes are normal: picotool emits no
            # blocks for a region that stays erased. Fill with 0xFF rather than
            # closing up, so every block keeps the address it asked for; the commit
            # path erases the app region first, so 0xFF there is erased flash.
            hole = addr - want
            if hole > FW_MAX_GAP:
                error_str = (f"block {blkno} jumps 0x{hole:X} bytes to "
                    f"0x{addr:08X}; that is not a hole in one image, it is two "
                    "images in one file")
                raise ValueError(error_str)
            out += b"\xFF" * hole
        out += b[32:32 + size]
        want = addr + size
    if not out:
        error_str = "no flashable blocks in the file"
        raise ValueError(error_str)
    # The chip is returned rather than judged here: only the bridge knows what
    # silicon it runs on (it reports it in `info`), so the caller compares.
    return bytes(out), (sorted(chips)[0] if chips else None)


try: from extras.AFC_utils import ERROR_STR
except:
    trace = traceback.format_exc()
    err_str = f"Error when trying to import from AFC_utils\n{trace}"
    raise error(err_str)

try: from extras.AFC_BambuAMS_bridge import (
    BambuBridge,
    TcpPort,
    )
except:
    err_str = ERROR_STR.format(import_lib="AFC_BambuAMS_bridge",
                               trace=traceback.format_exc())
    raise error(err_str)

from extras import AFC_BambuAMS_bridge as _bridge_mod  # noqa: E402

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
    from extras.AFC_RFID import (apply_filament_defaults, bed_temp_for_material,
                                 build_filament_name)
except Exception:
    # AFC_RFID is optional. The AMS reads its own tags, so the lane-info path
    # must work with no RFID module deployed. Self-contained fallbacks for that
    # path only, as AFC_ACE does; everything Spoolman-side lives in
    # AFC_BambuAMS_rfid and is reached through shims that no-op without it.

    def build_filament_name(brand: str, material: str, sub_type: str) -> str:
        """"<brand> <material> <sub_type>", skipping empties and not repeating
        the material when the variant already spells it out ("Bambu PLA Basic",
        never "Bambu PLA PLA Basic") -- a dependency-free copy of
        AFC_RFID.build_filament_name.

        :param brand: filament brand, may be ""
        :param material: base material such as "PLA", may be ""
        :param sub_type: variant such as "Matte", may be ""
        :return str: the joined name, or "" when nothing is known
        """
        parts = []
        if brand:
            parts.append(brand)
        if material and not (sub_type and material.lower() in sub_type.lower()):
            parts.append(material)
        if sub_type:
            parts.append(sub_type)
        return " ".join(parts).strip()

    def bed_temp_for_material(material: str) -> Optional[int]:
        """A default bed temperature for a material, or None when unknown --
        a dependency-free copy of AFC_RFID.bed_temp_for_material, kept because
        a Bambu tag almost never states one and this path has to work with no
        RFID module deployed.

        :param material: material string, any casing/separators
        :return Optional[int]: bed temperature in C, or None
        """
        if not material:
            return None
        key = material.strip().lower()
        for ch in (' ', '-', '_', '/'):
            key = key.replace(ch, '')
        table = {'pla': 55, 'placf': 55, 'plagf': 55, 'petg': 70, 'pet': 70,
                 'petgcf': 70, 'petgf': 70, 'tpu': 35, 'abs': 90, 'abscf': 90,
                 'absgf': 90, 'asa': 90, 'asacf': 90, 'pc': 100, 'pccf': 100,
                 'pa': 100, 'nylon': 100, 'pacf': 100, 'pagf': 100, 'pa6': 100,
                 'pa12': 100, 'pps': 100, 'ppscf': 100, 'hips': 100, 'pva': 45,
                 'bvoh': 45}
        if key in table:
            return table[key]
        for k in sorted(table, key=len, reverse=True):
            if key.startswith(k):
                return table[k]
        return None

    def apply_filament_defaults(lane: Any, slot_info: Any,
                                color_converter: Optional[Any] = None,
                                afc_defaults: Optional[dict] = None) -> None:
        """Apply the AMS's tag material/colour/temps to a lane when not already
        set -- a dependency-free copy of AFC_RFID.apply_filament_defaults, so a
        bay still shows its filament in Mainsail with no AFC_RFID present.

        Only fills blanks: a field the operator or a previous read already set
        is never overwritten, so it is safe to call on every surface pass.

        :param lane: lane to populate
        :param slot_info: normalized slot info from the AMS record
        :param color_converter: optional [r,g,b] -> "#rrggbb" callable
        :param afc_defaults: optional default_material_type / default_color
        """
        info = slot_info or {}
        has_material = getattr(lane, "material", None) not in (None, "")
        has_color = getattr(lane, "color", None) not in (None, "", "#000000")
        has_ext_temp = getattr(lane, "extruder_temp", None) is not None
        has_bed_temp = getattr(lane, "bed_temp", None) is not None

        material = (info.get("material", "") or "")
        if material.lower() == "unknown":
            material = ""
        color_hex = info.get("color_hex", "") or ""
        if not color_hex and color_converter is not None:
            raw = info.get("color", [0, 0, 0])
            if raw != [0, 0, 0]:
                color_hex = color_converter(raw)

        if not has_material and material:
            lane.material = material
        if not has_color and color_hex:
            lane.color = color_hex if color_hex.startswith("#") else "#" + color_hex
        if not has_ext_temp and info.get("extruder_temp") is not None:
            try:
                lane.extruder_temp = float(info.get("extruder_temp"))
            except (TypeError, ValueError):
                pass
        if not has_bed_temp and info.get("bed_temp") is not None:
            try:
                lane.bed_temp = float(info.get("bed_temp"))
            except (TypeError, ValueError):
                pass
        sub_type = info.get("sub_type", "") or ""
        if sub_type and not getattr(lane, "sub_type", ""):
            lane.sub_type = sub_type
        brand = info.get("brand", "") or ""
        if brand and not getattr(lane, "spool_vendor", ""):
            lane.spool_vendor = brand

        if afc_defaults is not None:
            if not getattr(lane, "material", None):
                dm = afc_defaults.get("default_material_type")
                if dm:
                    lane.material = dm
            if not getattr(lane, "color", None):
                dc = afc_defaults.get("default_color")
                if dc:
                    lane.color = dc

        if not getattr(lane, "weight", 0):
            lane.weight = 1000
# Spoolman lives in AFC_BambuAMS_rfid, enabled by an [AFC_BambuAMS_rfid]
# section and reached only through the shims further down. Nothing here imports
# AFC_RFID's Spoolman half, so this module loads with or without it.




#: Bambu's own filament brand. Every tag an AMS reads is a Bambu spool -- the
#: reader is keyed to their tags and returns nothing for anyone else's -- so
#: the brand is known without being on the wire.
BAMBU_BRAND = "Bambu"

#: Bambu names a filament as "<material> <variant>": "PLA Matte", "PLA Basic",
#: "PETG HF", "ABS". AFC keeps those in separate fields -- `material` drives
#: density and temperature lookups, `sub_type` is the variant Spoolman wants --
#: so the tag string is split rather than dumped whole into `material`.
#: Hyphenated composites ("PLA-CF", "PA6-CF") are one material, so this splits
#: on whitespace only.
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

# AMS type -> (has_heater, dry_dev_addr, dry_ams_id, dry_max_temp), picked
# per unit by `ams_model`. The 0x2C drying command is routed by device
# address; the id byte is the unit's bus address (ams_id None = derive it with
# mc_id_for_index). `heater:`/`dry_max_temp:` override.
#   ams1 -- regular AMS: no heater (drying commands rejected). The AMS Lite
#           is a future addition, not implemented or tested yet.
#   ams2 -- AMS2 Pro: heater, drying at device 0x0700, 65 C ceiling
#   ht   -- AMS HT:   heater, drying at device 0x1800, 85 C ceiling (single slot)
# Boxed AMS / AMS 2 Pro ceiling; the test suite reads it as the model default.
MAX_DRY_TEMP_C = 65

# The id byte is a bus address, and the firmware assigns it: the bridge
# enrolls every unit at an address computed from its chain index, and a unit
# answers only the id it was enrolled at. A host-side derivation that disagrees
# gets silence, not a near miss.
#
# Indices 0-3 map to themselves; above that the firmware's addr_of() moves to
# the HT class, 0x80 + (index - 4). A boxed unit can sit up there too (the
# auto-HT relocation vacates low slots and a later enrollment lands high).
#
# A wrong id costs measurement only: tag reads ride the 0x0211 filament-info
# poll, addressed by device and bus address rather than by this byte, but the
# 1A/02 log drain uses this byte, and a measured radius exists only in that
# narration. The firmware also throttles an unanswered drain to 1-in-256
# cycles.
#
# Keep this in lockstep with addr_of() in the firmware's bambubus.c;
# tests/test_mc_id_matches_addr_of.py fails if the two drift.
def mc_id_for_index(idx: int) -> int:
    """
    The bus address the firmware enrolled chain index ``idx`` at.

    :param idx: The unit's chain index
    :return: Its id byte -- 0x00-0x03 for the boxed slots, 0x80 + (idx - 4)
             above them
    """
    i = int(idx)
    if i < 0:
        return 0
    return i if i < 4 else (0x80 + (i - 4)) & 0xFF


#: The one model table. AFC_BridgeBox derives its lane counts, heater and
#: drying ceilings from it, so a roster tag and an ams_model are the same
#: vocabulary.
_AMS_MODELS = {
    #            heater  dev_addr  ams_id  max_temp
    "ams1":    (False,  0x0700,   None,   65),   # AMS (no heater)
    "ams2":    (True,   0x0700,   None,   65),   # AMS 2 Pro
    "ht":      (True,   0x1800,   None,   85),   # AMS HT
    # AFC_BridgeBox's own tags, which it writes into the sections it builds.
    "boxed":   (False,  0x0700,   None,   65),   # 0x0700, generation unconfirmed
    "lite":    (False,  0x0700,   None,   65),   # AMS Lite
}

# MC poll addressing: device = the unit class (its dev_addr above), id = the
# unit's bus address (derive it only with mc_id_for_index). An AMS 1 and an
# AMS 2 share a class, so the model comes from config. An HT answers the
# drain only with payload 0x00. mc_id_base is an additive override, 0 for
# every model, and deliberately separate from dry_ams_id.
_MC_ADDRESSING = {m: (spec[1], 0x00) for m, spec in _AMS_MODELS.items()}

_HT_MODELS = ("ht",)
#: Models whose insert measurement gets one automatic retry when it one-edges.
#: The AMS 2 is included because the firmware's own auto-rescan deliberately
#: declines for it (an unarmed calibrate from the bridge always one-edges) and
#: defers to the host.
_RETRY_MEASURE_MODELS = _HT_MODELS + ("ams2",)


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


def _colour_key(colour: Any) -> str:
    """
    A colour as RRGGBB for comparison, whichever way it is written.

    A bay's record carries "0086D6" (bridge_color_to_rgb) and a lane
    "#0086D6", sometimes with an alpha pair after it.

    :param colour: a record's or a lane's colour, or None
    :return str: uppercase RRGGBB, or "" when there is none
    """
    c = str(colour or "").strip().lstrip("#").upper()
    return c[:6]


def _lane_colour_is_records(lane: Any, info: dict) -> bool:
    """
    Whether an unbound lane's colour says the bay's record is its spool's.

    BLACK HAS NO COLOUR ON A RECORD. bridge_color_to_rgb turns a tag's
    000000 into None (the record says it was black in "color_black"), so a
    lane dressed from a black tag carries no colour (and one bound since,
    Spoolman's "#000000"), exactly as a lane on AFC's defaults carries none.
    The variant tells them apart: the tag wrote its own onto the black lane,
    and defaults leave it blank. A black tag with no variant (a bare "ABS")
    cannot be told from defaults, and is not matched.

    :param lane: the lane mapped to the bay
    :param info: the bay's normalized record
    :return bool: True when the colours agree
    """
    lc = _colour_key(getattr(lane, "color", ""))
    rc = _colour_key(info.get("color"))
    if lc and lc == rc:
        return True
    if (rc or not info.get("color_black") or lc not in ("", "000000")
            or not info.get("material")):
        return False
    if lc == "000000":
        return True
    _base, sub = _split_bambu_material(info.get("material"))
    lane_sub = str(getattr(lane, "sub_type", "") or "").strip().lower()
    return bool(sub) and lane_sub == sub.strip().lower()


def bridge_slot_to_info(slot: dict) -> dict:
    """
    Map a bridge slot dict to a normalized AFC slot_info dict.

    Bambu's tag is decoded to a PROFILE (material name, Bambu type code / sku,
    color, print temps), there is no per-spool UID, so rfid_uid is always None.
    This mirrors a base-ACE unit rather than the UID-unique ACE2/U1/Vivid.

    :param slot: One entry from a bridge status frame's 'slots' list
    :return dict: normalized info (present, state, material, sku, color, temps)
    """
    material = slot.get("material") or None
    sku = slot.get("sku") or None

    def _temp(v: Any) -> Optional[int]:
        """
        A positive integer temperature, or None for anything else.

        :param v: the raw tag field
        :return Optional[int]: the temperature, or None
        """
        return v if isinstance(v, int) and v > 0 else None

    # The tag's bed temperature first, the material table only when the tag
    # is silent. A Bambu tag has a bed-temperature field but usually leaves it
    # 0 (the firmware reads it correctly; see MATERIAL_BED_TEMP). The source is
    # recorded, because "the spool says 35 C" and "PLA usually wants 55 C" are
    # different claims.
    tag_bed = (slot.get("bedt") if isinstance(slot.get("bedt"), int)
               and 0 < slot.get("bedt") <= 200 else None)
    if tag_bed is not None:
        bed_temp, bed_temp_source = tag_bed, "tag"
    else:
        bed_temp = bed_temp_for_material(material or "")
        bed_temp_source = "material" if bed_temp is not None else None
    return {
        "index": slot.get("i"),
        "present": bool(slot.get("present")),
        "state": slot.get("state") or "empty",
        "material": material,
        "sku": sku,                         # Bambu profile code, e.g. "GFA00"
        "color": bridge_color_to_rgb(slot.get("color")),
        # A black tag, which "color" cannot say: it maps 000000 to None, as it
        # does the 00000000 of a record with no colour decoded. The firmware
        # forces a decoded colour's alpha to FF, so 000000FF is black and
        # nothing else (see _lane_colour_is_records).
        "color_black": (isinstance(slot.get("color"), str)
                        and slot["color"].upper() == "000000FF"),
        "temp_min": _temp(slot.get("tmin")),
        "temp_max": _temp(slot.get("tmax")),
        "weight": slot.get("weight") or None,
        # Bed temperature, C: the tag's own figure when it states one, otherwise
        # derived from the material (see above). None only when the tag is
        # silent and the material is unknown. "bed_temp_source" says which,
        # and is None whenever bed_temp is.
        "bed_temp": bed_temp,
        "bed_temp_source": bed_temp_source,
        # Spool remaining capacity, percent, from the tag's persisted remain
        # fraction (0x0211 reply, RGBA-anchor +29). The AMS maintains it via
        # the odometer during its insert calibration. 0 is a real value (never
        # measured, or an exhausted spool); -1/absent means the field was not
        # read.
        "remain_pct": (slot.get("remain")
                       if isinstance(slot.get("remain"), int)
                       and slot.get("remain") >= 0 else None),
        # The tag UID. "uid" is the 4-byte Mifare chip UID (the card_uids match
        # key, shared with every other reader on this printer); "tray_uid" is
        # Bambu's 16-byte tray id. Empty until a read.
        "rfid_uid": (slot.get("uid") or None),
        "tray_uid": (slot.get("tray_uid") or None),
        # The firmware's scan-window verdict: scan_seq advances each time a
        # scan window resolves for this bay, scan_res says how (1 read /
        # 2 foreign / 3 no tag). The firmware runs one window at a time for a
        # known (unit, slot), so unlike the narration stamps this cannot credit
        # a sibling's read or cycle-end to this bay. None when absent, and the
        # verdict logic then falls back to the narration stamps.
        "scan_seq": (slot.get("sseq")
                     if isinstance(slot.get("sseq"), int) else None),
        "scan_res": (slot.get("sres")
                     if isinstance(slot.get("sres"), int) else None),
        # The measured percent, attributed by the firmware to the bay whose
        # capacity window produced it. The narration it comes from arrives on
        # device 0x0700, which all boxed units share, so the host cannot
        # attribute it itself. meas_seq advances per measurement so a repeat
        # of the same value still counts as new.
        "meas_pct": (slot.get("mpct")
                     if isinstance(slot.get("mpct"), int)
                     and slot.get("mpct") >= 0 else None),
        "meas_seq": (slot.get("mseq")
                     if isinstance(slot.get("mseq"), int) else None),
        # The radius that percent came from, in millimetres. It rides with
        # meas_pct because the pair is one measurement: the narration carries
        # both, but a stamp can outrun its narration. A Klipper restart only
        # records this stamp, it does not adopt it (see _baseline_meas_stamp).
        # 0/absent = the measurement line stated no radius.
        "meas_radius_mm": (slot.get("mrad")
                           if isinstance(slot.get("mrad"), int)
                           and slot.get("mrad") > 0 else None),
        # The firmware still owes this bay a re-read. While it does, an empty
        # record means "not fetched yet", never "no tag".
        "reread_pending": bool(slot.get("rrq")),
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
            error_str = (f"lane '{name}' has index {idx}, outside this unit's slots "
                f"1..{slots_per_unit}")
            raise ValueError(error_str)
        slot = idx - 1
        if slot in owner:
            error_str = (f"lanes '{owner[slot]}' and '{name}' both map to slot {slot} "
                f"(index {idx}); each lane needs a unique index")
            raise ValueError(error_str)
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
        msg += (" <span class=warning--text>(AMS offline, bridge "
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
# this must cover the wind-down or the stop re-adopts the dry. Short enough
# that a stop which genuinely failed still shows up as "still drying" quickly.
DRY_STOP_GRACE = 25.0

#: How long after a start command a refusal is ignored, in seconds.
#:
#: The bridge parses narration on its own thread, so a line the AMS emitted
#: before clear_dry_error() ran can be recorded after it and a stale refusal
#: read as this attempt's. A real refusal is repeated by the unit and outlives
#: this window.
DRY_REFUSE_GRACE = 20.0

#: How long an HT dry may stay completely silent before the cycle is
#: declared never-started, in seconds. A busy HT ignores the dry frame
#: rather than refusing it; an accepted dry answers within ~12s. HT only:
#: a boxed unit's heater deafens the receiver, so silence there is normal.
DRY_SILENT_GRACE = 45.0

#: Volts above which an AMS 2 Pro is running on its own 24V adapter rather
#: than off the bus wire.
#:
#: The unit reports its jack in the `ad:` field of the [AMS_CHMB] telemetry it
#: streams while drying: about 24 V with the adapter in, about 0.5 V without.
#: 18 sits under a sagging 24V rail; anything between 2 and 20 behaves the
#: same.
EXT_SUPPLY_MIN_V = 18.0

#: How stale the supply reading of a DRYING unit may be before it stops
#: counting as knowledge, in seconds.
#:
#: A heating unit narrates every ~10s, so this is four missed lines. Tighter
#: than the 120s display staleness because it gates a heater: no reading
#: refuses, while a stale one would allow.
EXT_SUPPLY_MAX_AGE = 45.0


def _unit_tool_loaded(unit: Any) -> bool:
    """
    Is any lane on this unit threaded to the toolhead?

    Broader than afcBambuAMS._tool_loaded_lane(), which finds the lane of the
    active extruder so the follower can be armed for it. Here the extruder does
    not matter: a tag scan feeds filament past the bay reader and pulls it back,
    and the AMS runs that cycle for the whole unit, so any loaded lane on the
    unit is disturbed no matter which toolhead owns it.

    A module function rather than a method so the scan path stays callable with
    a plain stand-in object, which is how the tests drive it.

    Fails open: if the lane map cannot be read this returns False and the scan
    proceeds, so a check that cannot answer never stops tags being read.

    :param unit: the AMS unit
    :return bool: True if a slot-mapped lane on this unit is tool_loaded
    """
    try:
        for lane in getattr(unit, "lanes", {}).values():
            if not getattr(lane, "tool_loaded", False):
                continue
            if unit._slot_of(lane) is not None:
                return True
    except Exception:
        return False
    return False


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


def _refuse_while_printing(printer: Any, reactor: Any, gcmd: Any,
                           cmd: str) -> None:
    """
    Raise a gcode error if a print is running or paused.

    A printer without print_stats has nothing to protect, so it passes.

    :param printer: Klipper printer object
    :param reactor: reactor supplying monotonic time
    :param gcmd: the command being refused, for its error type
    :param cmd: command name for the message
    """
    state = None
    try:
        ps = printer.lookup_object("print_stats", None)
        if ps is not None:
            state = ps.get_status(reactor.monotonic()).get("state")
    except Exception:
        state = None
    if state in ("printing", "paused"):
        raise gcmd.error(f"{cmd}: not while a print is active "
                         f"(print_stats state={state})")




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
        """
        Bind this virtual pin to its owning chip.

        :param chip: the _BambuBufferChip that streams samples into it
        """
        self._chip = chip
        self._callback: Optional[Callable] = None

    def setup_adc_sample(self, *args: Any, **kwargs: Any) -> None:
        """
        Accept and ignore hardware sample timing -- samples are pushed.

        :param args: whatever the caller's Klipper/Kalico signature passes
        :param kwargs: likewise
        """
        return None

    def setup_minmax(self, *args: Any, **kwargs: Any) -> None:
        """
        Accept and ignore ADC range checking -- there is no real ADC.

        :param args: whatever the caller's Klipper/Kalico signature passes
        :param kwargs: likewise
        """
        return None

    def setup_adc_callback(self, report_time: Any, callback: Any = None) -> None:
        """
        Register the consumer's sample callback.

        :param report_time: Klipper passes (report_time, cb); Kalico passes
                            only (cb), in which case this IS the callback
        :param callback: the callback, in the two-argument signature
        """
        # Klipper: (report_time, cb). Kalico/older: (cb).
        self._callback = report_time if callback is None else callback

    def get_mcu(self) -> Any:
        """
        Name the MCU this pin claims to live on.

        :return Any: the primary MCU object
        """
        return self._chip.printer.lookup_object("mcu")

    def push(self, value: float) -> None:
        """
        Deliver one buffer sample to the registered consumer.

        :param value: the buffer reading, 0..1
        """
        cb = self._callback
        if cb is not None:
            # Single-arg form: AFCFPSBuffer._adc_callback timestamps it itself.
            cb(value)


class _BambuBufferChip:
    """Host pin chip exposing one bus master's buffer as a virtual ADC.

    `adc_pin: bambu_buffer:fps` -> the buffer of the units on that Pico.

    One chip per master, not per printer. Every unit on a bus feeds one
    buffer and therefore one extruder, so the reading is shared across the
    units of a single Pico; a second Pico is a second buffer and a second
    extruder, and must not read the first one's value (with `tool_start:
    buffer` that would make the toolhead sensor read the wrong bus).

    The chip NAME is what an [AFC_buffer] section references, so a second
    master needs its own: set `buffer_chip_name` on its units and point that
    buffer's adc_pin at it.
    """

    def __init__(self, unit: Any, name: str = _BUFFER_CHIP_NAME,
                 report_time: float = 0.100) -> None:
        """
        Register the chip with Klipper's pin registry and arm its timer.

        :param unit: the AFC_BambuAMS unit whose master's buffer this reads
        :param name: chip name an [AFC_buffer] adc_pin references
        :param report_time: seconds between pushed samples
        """
        self.printer = unit.printer
        self._unit = unit
        self._name = name
        self._report_time = report_time
        self._pins: List[_BambuBufferADC] = []
        self._timer = None
        self.printer.lookup_object("pins").register_chip(name, self)
        self.printer.register_event_handler("klippy:ready", self._start)

    def setup_pin(self, pin_type: str, pin_params: dict) -> _BambuBufferADC:
        """
        Hand out a virtual ADC pin for this chip.

        :param pin_type: Klipper pin type; only "adc" exists here
        :param pin_params: parsed pin parameters (unused)
        :return _BambuBufferADC: the new pin
        """
        if pin_type != "adc":
            error_str = (f"{self._name} only provides 'adc' pins "
                         f"(use adc_pin: {self._name}:fps)")
            raise self.printer.config_error(error_str)
        adc = _BambuBufferADC(self)
        self._pins.append(adc)
        return adc

    def _start(self, *args: Any) -> None:
        """
        klippy:ready callback -- start the sample timer once.

        :param args: event arguments (unused)
        """
        if self._timer is None:
            reactor = self.printer.get_reactor()
            self._timer = reactor.register_timer(self._update, reactor.NOW)

    def _update(self, eventtime: float) -> float:
        """
        Timer callback: push the master's live buffer value to every pin.

        :param eventtime: reactor time of this firing
        :return float: the next firing time
        """
        v = self._unit.fps_buffer_value()
        if v is not None:
            for adc in self._pins:
                adc.push(v)
        return eventtime + self._report_time


def _register_bambu_buffer_chip(unit: Any) -> None:
    """
    Register a buffer ADC chip for this unit's bus master, once per master.

    Keyed by chip name, which defaults to "bambu_buffer" and is settable per
    unit. Units sharing a Pico share a buffer and so share the chip -- the
    first to initialise creates it and the rest find it here. A second Pico
    gets its own chip under its own name, and reads its own buffer.

    :param unit: the AFC_BambuAMS unit registering
    """
    printer = unit.printer
    chips = getattr(printer, "_bambu_buffer_chips", None)
    if chips is None:
        chips = {}
        printer._bambu_buffer_chips = chips
    name = getattr(unit, "buffer_chip_name", _BUFFER_CHIP_NAME)
    if name in chips:
        return
    chips[name] = _BambuBufferChip(unit, name)


# The AMS meters its own moves. Neither the mm nor the mm/s we send ever
# controls one -- bb_feed() in the firmware converts both into a runaway
# deadline, and _wait_move does the same on this side. These are not commanded
# speeds but nominal figures that turn a distance into a watchdog, so they are
# module constants rather than config options.
NOMINAL_MMPS = 20.0
MAX_MMPS = 30.0

# AMS bay -> hub staging distance. Fixed, not configurable: the hub here is
# virtual (the AMS multiplexes internally, there is no switch to reach) so
# there is nothing for an operator to measure, and like the bowden length it
# only sizes a deadline. 250mm is comfortably past any real bay-to-hub run.
DIST_HUB_MM = 250.0

# Default hub -> toolhead distance until the unit reports its own. Long on
# purpose: this sizes the load give-up deadline, and the first load must be
# allowed to finish so the AMS can measure its path and write the real value
# back; a short default would abort it and the unit would never calibrate.
DEFAULT_BOWDEN_MM = 3000.0

# How far the AMS's measurement must sit from the configured value before
# it is worth rewriting the file. The unit's own figure moves a few mm
# between calibrations and none of this needs that precision.
PATH_ADOPT_TOLERANCE_MM = 25.0


#: Cadence, in ms, sent to the bridge as `armms` for the idle 11/04
#: keep-alive sweep. 600000 (10 min) turns that sweep effectively off on
#: purpose: our master blocks the bus while it transmits, and an arm frame
#: landing mid-calibration kills the HT's second odometer edge (0 would mean
#: the firmware's 520 ms, the printer's own cadence, which measured worse).
#: Following is unaffected: the firmware still re-sends an unacknowledged
#: arm at 520 ms until the unit narrates state:4, and arms the HT during a
#: measure on its own cadence.
FOLLOW_ARM_MS = 600000.0

#: How long to let a freshly-connected Pico settle before announcing to
#: it. Announcing the instant the CDC endpoint reopens lands before the
#: firmware accepts `mcaddr`, and a dropped announce is silent until the
#: next reconnect.
ANNOUNCE_SETTLE_S = 1.0
#: Seconds to let a chain relink finish before the units re-announce.
#: The relink is a deregister sweep and the re-enrollment that follows
#: is the firmware's; announcing in the middle of it writes config to units
#: that are about to be renumbered.
RELINK_SETTLE_S = 3.0

#: AMS mode values: a feed runs 0 idle ->
#: 2 feeding -> 3 feed done -> 4 following -> 1 assist. Reported as `fstate` in
#: every status frame. The full set is named so the field is readable in a
#: status dump; only IDLE and FOLLOWING are acted on.
AMS_MODE_IDLE = 0
AMS_MODE_ASSIST = 1
AMS_MODE_MOVING = 2
AMS_MODE_DONE = 3
AMS_MODE_FOLLOWING = 4

#: Hard ceiling on any single move's watchdog, in seconds. A full 3.5 m path
#: takes about 25 s at the AMS's ~136 mm/s; this is the runaway guard for a
#: completion report that never arrives, not a schedule.
MOVE_DEADLINE_MAX_S = 35.0

#: Ceiling on the load-to-sensor window, a different quantity from the
#: move watchdog: it must cover the bulk feed plus several of the unit's
#: own feed/stall/retry cycles. Only a runaway guard, so generous; the
#: loop exits the moment the sensor triggers.
LOAD_SENSOR_MAX_S = 180.0

#: How far the dry countdown may fall between readings and still count as
#: a tick, since the register catches up in one step after a deaf spell.
DRYREM_TICK_SLACK_S = 120.0

#: mm/s used only to size a watchdog deadline; NOMINAL_MMPS is what goes on
#: the wire. The AMS retracts at about 136 mm/s, so this errs slow.
DEADLINE_MMPS = 60.0


def _gcmd_int(gcmd: Any, name: str, default: int,
              minval: int, maxval: int) -> int:
    """
    Read an integer parameter that may be written in hex.

    Klipper's own get_int is decimal-only, so ADDR=0x0700 would fail to
    parse, while bus addresses are conventionally written in hex.

    Accepts 0x/0b/0o prefixes and plain decimal (int(x, 0)).

    :param gcmd: The Klipper GCodeCommand
    :param name: Parameter name
    :param default: Value when the parameter is absent
    :param minval: Lowest accepted value, inclusive
    :param maxval: Highest accepted value, inclusive
    :return int: the parsed value
    """
    # Klipper's own parse first, so decimal input keeps its exact behaviour
    # (including Klipper's range errors) and only a value it rejects, such as
    # a hex literal, reaches the fallback below.
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
        error_str = f"Error on '{gcmd.get_commandline()}': unable to parse {raw}"
        raise gcmd.error(error_str)
    if val < minval or val > maxval:
        error_str = (f"Error on '{gcmd.get_commandline()}': {name}={raw} is outside "
            f"{minval}..{maxval}")
        raise gcmd.error(error_str)
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




#: Narration fragments that are link/keep-alive chatter, not a reason. The unit
#: streams these continuously; they are in the buffer at the moment of a fault
#: purely because they are in it at every moment.
_FAULT_NOISE = ("[AMS_COMMON]", "[AMS_LINK]", "[AMS_IDLE]", "[AMS_LED]TRAY")

# The AMS 1 states its give-up verdict on keep-alive tags ([AMS_COMMON]
# state:6/7, [AMS_LINK] en:0,mode:7) that the chatter filter would drop, so
# those fragments are kept for the message. Detection itself is the bridge's
# classifier.
_FAULT_SIGNAL = re.compile(
    r"state:[67]|en:0,\s*mode:7|stall|finish -1|timeout error", re.I)


def _register_lane_command(unit: Any, cmd: str, handler: Callable,
                           desc: str, unit_alone: bool = False) -> None:
    """
    Register a command that acts on one lane: UNIT= as usual, and LANE= alone.

    The UNIT-keyed form goes to ``unit`` like every other mux command. The
    first unit to register ``cmd`` on this printer also claims the no-UNIT
    default, which finds the Bambu unit holding the named lane and runs that
    unit's handler (_run_by_lane), so ``AFC_BAMBU_SCAN LANE=lane14`` works as
    the log lines that name it say. Later units find the default taken, which
    is expected.

    Per g-code object, not a module flag: a Klipper RESTART builds a new
    printer in the same process, and a module-level "already registered"
    survives it into a printer that never got the command.

    :param unit: the unit (afcBambuAMS) registering; ``unit.gcode`` is used
    :param cmd: the command name
    :param handler: the unit's handler for it
    :param desc: the command's help text
    :param unit_alone: the command also runs with UNIT= and no LANE=, so the
      refusal of a bare command may offer UNIT=
    """
    unit.gcode.register_mux_command(cmd, "UNIT", unit.name, handler, desc=desc)
    handlers = getattr(unit, "_lane_handlers", None)
    if handlers is None:
        handlers = {}
        unit._lane_handlers = handlers
    handlers[cmd] = handler
    try:
        unit.gcode.register_mux_command(
            cmd, "UNIT", None,
            lambda gcmd, _c=cmd, _u=unit, _a=unit_alone: _run_by_lane(
                getattr(_u, "printer", None), _c, gcmd, unit_alone=_a),
            desc=desc)
    except Exception:
        pass                    # another unit on this printer claimed it


def _run_by_lane(printer: Any, cmd: str, gcmd: Any,
                 unit_alone: bool = False) -> None:
    """
    Run ``cmd`` on the Bambu unit holding the lane named by LANE=.

    The no-UNIT default registered by _register_lane_command. The unit's own
    handler does the rest, including every check it makes when UNIT= is given.

    :param printer: the Klipper printer
    :param cmd: the command name
    :param gcmd: the Klipper GCodeCommand
    :param unit_alone: the command also runs with UNIT= and no LANE=
    """
    lane_name = gcmd.get("LANE", None)
    units = []
    try:
        units = [u for _n, u in printer.lookup_objects("AFC_BambuAMS")
                 if cmd in (getattr(u, "_lane_handlers", None) or {})]
    except Exception:
        pass
    names = ", ".join(sorted(str(u.name) for u in units)) or "none"
    # No angle brackets in these: Moonraker's HTTP API reports an error whose
    # text holds one as "Unknown".
    if not lane_name:
        if unit_alone:
            raise gcmd.error(
                f"{cmd}: needs LANE= (one lane) or UNIT= (every slot on that "
                f"unit). Bambu units: {names}")
        raise gcmd.error(f"{cmd}: needs LANE= naming the lane")
    for u in units:
        if lane_name in (getattr(u, "lanes", None) or {}):
            u._lane_handlers[cmd](gcmd)
            return
    raise gcmd.error(
        f"{cmd}: no Bambu unit holds a lane named '{lane_name}' "
        f"(Bambu units: {names})")


def _fault_reason(text: str) -> str:
    """
    The part of the AMS's narration that says why, without the chatter.

    The raw buffer at a stall is mostly link keep-alive:

        [AMS_COMMON]en:1,mode:3,idx:0,ref:0 [AMS_COMMON]en:1,mode:3,idx:2,
        ref:127 [AMS_COMMON]en:1,mode:3,idx:0,ref:0 [AMS_SWITCH]timeout,
        assist finish stall! pos:0.1

    Three of those four fragments say nothing; exactly one is the reason.

    Keeps every non-chatter fragment rather than matching known stall wording:
    the three unit types phrase it three ways and one says nothing at all, so a
    whitelist drops whichever dialect it has not seen. Falls back to the raw
    text when filtering leaves nothing -- better a dump than an empty reason.

    _FAULT_SIGNAL is the one exception: a chatter-tagged fragment is kept when
    it carries a known fault signature, which an AMS 1 needs because its
    give-up rides on [AMS_COMMON]/[AMS_LINK] while its odometry rides on
    [AMS_DEV]. The tag filter still decides everything it does not match.

    :param text: the raw narration buffer at the moment of the fault
    :return str: the meaningful fragments, or the original if none stand out
    """
    if not text:
        return text
    parts = [p.strip() for p in re.split(r"(?=\[AMS_)", text) if p.strip()]
    keep = [p for p in parts
            if not p.startswith(_FAULT_NOISE) or _FAULT_SIGNAL.search(p)]
    return "  ".join(keep) if keep else text.strip()


# The bridge registry lives in AFC_BambuAMS_bridge, next to BambuBridge itself.
# Reached as _bridge_mod._BRIDGES (never imported by name) so a rebind of the
# module attribute is seen by every reader -- see the note at its definition.


#: How long the link to the bridge may be down during a load before the load
#: stops feeding. The reader reconnects on a backoff capped at 5s, so this has
#: to outlast an ordinary reconnect (a load worth finishing survives a blip)
#: while still being a fraction of the load timeout plus two recovery rounds
#: that a dead link would otherwise burn.
LINK_DOWN_GRACE_S = 12.0

#: What an operator does after clearing a failed load. Named in every failure
#: message, because that message is the only place the recovery is explained.
_RETRY_HINT =("Once the path is clear, run AFC_BAMBU_FAULT_RELOAD to retry "
               "the load")


def _bridge_log_tag(serial_port: str) -> str:
    """
    Short, stable tag naming a bus master for its narration log.

    Empty for the first master registered, so a single-Pico printer writes
    AFC_BambuAMS.log. Later masters get a tag derived from their serial port
    (the key of the bridge registry), so a master keeps the same file across
    restarts regardless of initialisation order; only which one gets the
    unsuffixed name depends on order.

    :param serial_port: the port this master is reached on
    :return str: a filename-safe tag, or "" for the first master
    """
    if not _bridge_mod._BRIDGES:
        return ""
    base = str(serial_port).rsplit("/", 1)[-1]
    safe = "".join(c if (c.isalnum() or c in "-_") else "_" for c in base)
    return safe[-24:].strip("_") or "alt"


# ── AFC unit ────────────────────────────────────────────────────────────────────

class afcBambuAMS(afcUnit):
    """
    AFC unit for a stock Bambu AMS behind the Pico Bambu-Bus bridge.

    Mirrors the bridge's per-slot status onto AFC lanes and issues transport as
    bridge commands, binding the load/unload sequencing to the base unit.
    """

    SLOTS_PER_UNIT = SLOTS_PER_UNIT

    # Backstop for the motion guard: how long _scan_in_flight keeps ignoring
    # this bay's presence flap when the unit never announces the end of its
    # scan. Normally unused: every model narrates an end ("Calibration rst:0"
    # HT, "odom calib success exit 0" AMS 1, "STEP7:cali end" AMS 2) and
    # last_scan_end closes the guard there. Too short and the unit's own
    # retract re-triggers the scan in a loop; too long and a real removal goes
    # unnoticed.
    SCAN_MOTION_QUIET_S = 90.0

    # Backstop only: how long a scan may stay open with the unit saying
    # nothing at all. A scan normally ends when the unit narrates a successful
    # read or the end of its cycle; this only stops a unit that goes silent
    # mid-cycle (or a bridge that loses the narration channel) from leaving a
    # bay waiting forever.
    SCAN_FALLBACK_CAP = 45.0

    # The backstop for the firmware-verdict path (scan_seq/scan_res). Longer
    # than SCAN_FALLBACK_CAP on purpose: the firmware always publishes an
    # answer (window close, hijack or removal all resolve the seq), and a
    # presence flap on a slow pull can open the hold well before the spool
    # seats. 90 s clears the 61 s firmware window with margin, so it fires only
    # when the bridge is dead.
    SCAN_VERDICT_CAP = 90.0

    # How long an unbound lane waits before asking Spoolman again when its
    # lookup got no answer (server down or unreachable). A server that is not
    # answering has not said the spool is unknown, so the bay is not marked
    # as a miss; it is asked again on this clock until it answers, the lane is
    # bound, the bay empties or the connection changes.
    LOOKUP_RETRY_S = 30.0

    # How long a bay must stay occupied after an insert during a print before
    # its lane goes on lane defaults (_defaults_until_read). Longer than a
    # stray presence frame (a fraction of a second), short against the seconds
    # a reel takes to be pushed in and seated.
    DEFAULTS_SETTLE_S = 2.0

    #: Alias of the module constant, for discoverability on the class.
    DRY_STOP_GRACE = DRY_STOP_GRACE

    def __init__(self, config: Any) -> None:
        """
        :param config: Klipper ConfigWrapper for this [AFC_BambuAMS] section
        """
        super().__init__(config)
        self.type = config.get("type", "BambuAMS")
        # Optional, and None fails soft (one log line, unit offline). A section
        # can exist with no serial_port without anyone writing it: ConfigRewrite
        # persists learned values (e.g. an adopted bowden length) into
        # AFC_auto_vars.cfg keyed by section name, and if the section it names
        # is gone (a unit renamed or removed) klippy instantiates this class
        # from that leftover. Such a remnant must not stop the printer booting.
        self.serial_port = config.get("serial_port", None)
        #   tcp_key : the link key a tcp:// bridge demands before it will take
        #     a command. Must match what was entered in the setup portal.
        #
        # Only meaningful for a tcp:// port -- a USB cable is its own
        # authentication. Blank leaves the link open, which is what a board
        # with no key stored expects, so this can be rolled out to the config
        # before the boards have keys or the other way round.
        self.tcp_key = config.get("tcp_key", None)
        self.baud = config.getint("baud", 115200)
        # These lanes have no drive stepper, the AMS motors do the feeding, so
        # AFC_lane.move_to routes moves to our lane_move() (bridge feed/retract).
        self.stepperless_drive = True
        # Load/unload transport distances (mm), commanded open-loop; the
        # toolhead sensor confirms the load.
        #   afc_bowden_length        : hub -> toolhead feed distance
        #   afc_unload_bowden_length : toolhead -> hub retract distance
        # DEFAULT_BOWDEN_MM is deliberately long: a short value kills good loads
        # part-way, a long one only delays reporting a jam, and the unit writes
        # its measured path back after the first load.
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
        # its own load state machine that retries several times (feed -> stall
        # at the extruder -> retract -> retry), so this leaves room for those
        # attempts before the load counts as failed.
        self.load_retry_timeout = config.getfloat(
            "load_retry_timeout", 40.0, minval=1.0)
        # How often the host re-kicks the feed while waiting. Too often and each
        # push resets the AMS mid-retry (it never completes its own attempt);
        # too rarely and a genuinely idle AMS sits still. Spaced to let one AMS
        # stall-retry cycle finish between nudges.
        self.load_retry_interval = config.getfloat(
            "load_retry_interval", 4.0, minval=0.5)
        # On a load that never reaches the sensor, whether to reel the filament
        # back into the bay (AMS multi-stage unwind) before reporting the error.
        # Default False: the AMS keeps retrying loads on its own, and reeling
        # the filament back mid-retry fights it, so it is left staged.
        self.reel_back_on_load_fail = config.getboolean(
            "reel_back_on_load_fail", False)
        # When the AMS exhausts its own load retries and stalls (state:7), run the
        # printer's "Retry": a re-home reset (mode 0F/0E) then re-feed. This many
        # reset+retry cycles before reporting the failure. 0 disables it and
        # fails straight to handle_lane_failure.
        self.load_recover_attempts = config.getint(
            "load_recover_attempts", 2, minval=0)
        # How far the extruder bites before the follower is armed, so the gears
        # have hold of the filament without being mid-advance when the AMS runs
        # its own pull-back. Enough to grip, not enough to matter if the unit
        # tugs against it. 0 disables the split and advances tool_stn in one go.
        self.tool_bite_mm = config.getfloat("tool_bite_mm", 0.0, minval=0.0)
        # Cap, in seconds, on waiting for the unit to report it has stopped
        # after the toolhead sensor trips, before the assist is sent (see
        # _wait_arrival_settle). 0 skips the wait; AFC_BAMBU_ARRIVAL changes
        # it at runtime.
        self.arrival_assist_delay_s = config.getfloat(
            "arrival_assist_delay_s", 4.0, minval=0.0)
        # Extra mm (beyond bowden + DIST_HUB_MM) to retract when ejecting a lane so
        # the filament clears the hub and pulls fully back into the AMS bay.
        self.eject_buffer = config.getfloat("eject_buffer", 200.0, minval=0.0)
        # Boxed AMS by default (auto|ams). "lite" is reserved for the AMS Lite,
        # a future addition that is neither implemented nor tested yet.
        self.variant = config.getchoice(
            "variant", {"auto": "auto", "ams": "ams", "lite": "lite"}, "auto")
        # Which AMS on the daisy-chain this unit represents (0..MAX_AMS-1).
        # Multiple [AFC_BambuAMS] units on the same serial_port share one
        # bridge.
        self.ams_index = config.getint("ams_index", 0, minval=0,
                                       maxval=MAX_AMS - 1)
        # The config value, frozen before UID resolution mutates ams_index.
        # This is the operator's stated enrollment order for the uid-binding
        # table (class_rank_of ranks by bound idx): it decides which boxed
        # unit takes wire id 0 on the next fresh enrollment. Ranking by the
        # configured index rather than the unit name lets any model sit at 0.
        self._bind_rank = self.ams_index
        # Optional: pin this unit to its physical AMS by UID (12-byte hex from the
        # bridge `chain` command). The firmware assigns chain indices by announce
        # order, which reshuffles across power-cycles, so a fixed ams_index can
        # address the wrong unit. With unit_uid set, on connect the chain index
        # currently carrying that UID becomes ams_index, so the mapping is
        # stable whatever order units boot in.
        self.unit_uid = (config.get("unit_uid", "") or "").strip().upper() or None
        # A pool unit (AFC_BridgeBox live hot-enroll) exists as an object but is
        # inert until claimed: it takes no bridge listener, resolves no chain
        # index, routes no status, and its lanes are `unassigned` so they
        # register nowhere. It has no uid until a claim binds one. Claiming it
        # (bring_online) runs the ready-time bus registration then, live.
        self.pool = config.getboolean("pool", False)
        # Auto-trigger an RFID/tag scan when a spool is newly inserted into a bay
        # (AMS presence bitmap 0->1), so material/color populate without a manual
        # AMS_SCAN. A scan moves filament, so it's gated to the idle/not-printing
        # state and latched to fire once per insertion.
        self.auto_scan = config.getboolean("auto_scan", True)
        # Motion-only pull-in on insert while another lane on the unit is loaded.
        # A boxed AMS holding the follower (state:4) sets preload_disable and will
        # not run its autonomous preload on the insert edge, so a spool inserted
        # while a lane is loaded just sits at the bay switch. When idle (never in
        # a print), briefly drop the follower so the unit relaxes to state:0,
        # prime the new bay in with bb_prime (feeder only, RFID reader dormant:
        # no scan, no measure), then re-engage the loaded lane. The tag scan
        # stays deferred until the lane comes out.
        self.insert_pullin = config.getboolean("insert_pullin", True)
        # Create a Spoolman spool from a Bambu tag whose UID Spoolman does not
        # yet know. Off by default (binding to an existing spool by UID always
        # happens; only creation is gated). Per-lane auto_spoolman_create still
        # overrides via get_auto_spoolman_create, matching the ACE2/U1 readers.
        self.auto_spoolman_create = config.getboolean(
            "auto_spoolman_create", False)
        # Write the AMS's physical remaining measurement (P:NN% by radius) back
        # to a bound Spoolman spool's remaining_weight. On by default; turn it
        # off to stop the AMS correcting Spoolman.
        self.sync_measured_to_spoolman = config.getboolean(
            "sync_measured_to_spoolman", True)
        # Hold a following AMS HT with the dense statu-0F poll (as the printer
        # does) rather than the ht_poll_seq re-poke. On by default; set it
        # False to fall back to the re-poke.
        self.ht_0f_hold = config.getboolean("ht_0f_hold", True)
        # measure_on_insert: also measure spool capacity on insert (the
        # printer's "calculate remaining capacity" flag, op-04 byte 4). Off by
        # default: the unit pulls filament for 8-25 s to do it. The tag read is
        # unaffected and AFC_BAMBU_CAPSCAN still measures on demand.
        self.measure_on_insert = config.getboolean("measure_on_insert", False)
        # Whether an insert-triggered measurement runs the CALIBRATE
        # choreography instead of the plain capscan. An explicit opt-in rather
        # than keyed on the model: on an AMS 2 Pro the choreography can wedge
        # the unit's controller mid-RFID-auth (mode:2 at 1Hz, every insert
        # ignored until AFC_BAMBU_RELINK). AFC_BAMBU_CAPSCAN still runs it on
        # demand.
        self.calibrate_on_insert = config.getboolean(
            "calibrate_on_insert", False)
        # Demand-gated follower re-engage. The AMS holds its self-centering
        # follower (mode:4) only while there's demand; on this bus it drops to
        # idle (state:0) when the buffer is centred and does not re-engage from
        # the AP2 stream alone. Rather than re-arm every poll (which pokes the
        # feeder at idle, a visible twitch), this watches the toolhead extruder
        # and re-sends the feeder select only when it actually advances.
        #   follow_poll_interval : how often to sample the extruder (s)
        #   follow_min_extrude   : mm of extrusion since last re-engage to fire
        self.follow_poll_interval = config.getfloat(
            "follow_poll_interval", 0.1, above=0.0)
        self.follow_min_extrude = config.getfloat(
            "follow_min_extrude", 0.1, above=0.0)
        # True keeps the follower armed whenever a lane is loaded (a small idle
        # twitch, but the extruder can always pull); False arms it only while
        # extruding. The AMS self-limits at its buffer centre either way.
        self.follow_when_loaded = config.getboolean("follow_when_loaded", True)
        #   auto_error_recovery : on a stall mid-print, run the printer's own
        #     recovery: AFC's lane unload (which cuts, retracts and unloads),
        #     a reload of the same spool, then RESUME the print. Off by
        #     default, since it moves the toolhead and filament unasked. One
        #     attempt per fault: the AMS is already retrying inside its own
        #     window, and retrying on top of it fights it. The resume happens
        #     only when the reload took and the unit has not declared it gave
        #     up; an unfilled toolhead would print air.
        self.auto_error_recovery = config.getboolean(
            "auto_error_recovery", False)
        #   auto_error_recovery_limit : how many times in a row the recovery
        #     may resume the print by itself before it stops and leaves it
        #     paused. One by default: a jam the recovery cannot clear faults
        #     again as soon as the print resumes, and an unattended loop would
        #     grind filament. Pressing resume by hand resets the budget, so
        #     this caps unattended saves in a row, not the print. 0 never
        #     resumes: recover the filament, then leave the print paused.
        self.auto_error_recovery_limit = config.getint(
            "auto_error_recovery_limit", 1, minval=0)
        #   bus_serial : the 15-character printer serial the bridge carries in
        #     the op-05 announce, which the units answer. Any 15 chars; shorter
        #     is padded. Whether a unit validates it is unknown: if the units
        #     stop answering the announce after changing it, restore the
        #     previous value.
        self.bus_serial = config.get("bus_serial", "").strip()
        self._auto_recover_armed = False
        # Automatic resumes since the last hand-driven one, counted against
        # auto_error_recovery_limit. _reload_before_resume zeroes it, because a
        # human pressing resume is the intervention the cap exists to wait for.
        self._auto_resume_count = 0
        # Set when a Bambu fault is what paused the print, cleared once the lane
        # is fed again. The RESUME wrap reloads only while this is set, so an
        # ordinary pause (filament change, a look at the first layer) resumes
        # unchanged.
        self._resume_needs_reload = False
        # Set by any status frame carrying byte[19] == 0x07, cleared when a
        # fault is armed. A latch because the signal is intermittent (see
        # _on_status).
        self._declared_since_fault = False
        # True while auto recovery's own unload+reload is running. The recovery
        # drives a load, and unit_load_lane clears _auto_recover_armed on every
        # load, so without this the attempt would reset its own one-shot guard
        # and the next fault would arm another recovery.
        self._in_auto_recover = False
        # Odometer range seen since the fault was armed, in mm. Answers where a
        # jam is (see _jam_location); never used during a normal load.
        self._odom_lo: Optional[float] = None
        self._odom_hi: Optional[float] = None
        # A separate range for the load. _raise_ams_fault deliberately resets
        # the pair above (so it means "during this fault's recovery"), which
        # would otherwise wipe the load's range if a fault were raised partway
        # through.
        self._load_odom_lo: Optional[float] = None
        self._load_odom_hi: Optional[float] = None
        #   rollcall_span_boxed / rollcall_span_ht : how many ids of each class
        #     the roll-call walks. Unset = derived from the configured units
        #     plus one spare per class, so hot-plugging the next unit still
        #     works. 0 = all of that class, which is what a real printer does
        #     (4 boxed + 8 HT) and costs ~8ms per empty id.
        self.rollcall_span_boxed = config.getint(
            "rollcall_span_boxed", None, minval=0, maxval=4)
        self.rollcall_span_ht = config.getint(
            "rollcall_span_ht", None, minval=0, maxval=8)
        #   fault_detect : act on the AMS's own stall reports ("feed finish -1,
        # stall", "rocker stall", "bdc stall"). The unit names these itself, so
        # this is a report, not an inference.
        self.fault_detect = config.getboolean("fault_detect", True)
        #   fault_pause : pause the print on a stall. Off leaves it a warning.
        self.fault_pause = config.getboolean("fault_pause", True)
        #   link_loss_pause_s : pause the print when the bridge has said
        #     nothing for this long. 0 disables.
        # Much shorter than the transport's reconnect: mid-print the extruder
        # keeps pulling against an undriven AMS and grinds. The firmware streams
        # status many times a second, so seconds of silence means it is gone.
        self.link_loss_pause_s = config.getfloat("link_loss_pause_s", 5.0,
                                                 minval=0.)
        # Last fault sequence handled, so one stall raises one error.
        self._fault_seen: int = 0
        # The lane the last stall was raised on, so the reload on resume is
        # aimed at the slot that stalled.
        self._fault_lane: Any = None
        # AMS type: one setting picks heater on/off, drying device address and
        # temp ceiling (see _AMS_MODELS): `ams1` (regular AMS, no heater),
        # `ams2` (AMS2 Pro), `ht` (AMS HT), and sets the unit's bus addressing.
        # `heater:` and `dry_max_temp:` override the type's defaults.
        self.ams_model = config.get("ams_model", "ams2").strip().lower()
        # The follower is the same for every model: op-04 07/7F at 148 ms, the
        # printer's cadence, with no buffer deadband to tune and nothing sent
        # to the firmware to select a style.
        #
        # An unknown model is a typo: refuse it and name the valid spellings,
        # rather than fall back to some model's addressing.
        if self.ams_model not in _AMS_MODELS:
            raise config.error(
                f"[{config.get_name()}] ams_model: {self.ams_model!r} is not a "
                f"known model. Valid: {', '.join(sorted(_AMS_MODELS))}")
        _is_ht = self.ams_model in _HT_MODELS
        _spec = _AMS_MODELS[self.ams_model]
        _model_heater, self.dry_dev_addr, self.dry_ams_id, _dry_default_max = _spec
        self.has_heater = config.getboolean("heater", _model_heater)
        # The drying id byte follows the unit's bus address (mc_id_for_index of
        # its chain index) unless the type pins it. Track it so UID-pinning can
        # update the id if the index is re-resolved from the UID on connect.
        # (Drying on a unit above chain index 3, where address and index
        # differ, has not been exercised.)
        self._dry_id_follows_index = self.dry_ams_id is None
        # Real bay count for this unit: the AMS HT has a single slot, the 4-slot
        # models have four. Internal arrays stay SLOTS_PER_UNIT-sized for
        # uniformity, but everything user-visible (PREP logo, scans, insert
        # logging) is clamped to unit_slots so a 1-slot HT never shows or scans
        # phantom bays.
        self.unit_slots = 1 if self.ams_model in _HT_MODELS else SLOTS_PER_UNIT
        # MC poll addressing (see _MC_ADDRESSING).
        _mc = _MC_ADDRESSING.get(self.ams_model, (0x0700, 0x00))
        self.mc_dev_addr = config.getint("mc_dev_addr", _mc[0],
                                         minval=0, maxval=0xFFFF)
        self.mc_id_base = config.getint("mc_id_base", _mc[1],
                                        minval=0, maxval=0xFF)
        # Explicit override wins outright; -1 means derive it as
        # mc_id_base | mc_id_for_index(ams_index).
        self.mc_ams_id = config.getint("mc_ams_id", -1, minval=-1, maxval=0xFF)
        if self._dry_id_follows_index:
            self.dry_ams_id = mc_id_for_index(self.ams_index)
        self.dry_max_temp = config.getint(
            "dry_max_temp", _dry_default_max, minval=1, maxval=DRY_TEMP_HARD_MAX)
        # One bus-powered dryer at a time. An AMS 2 Pro will heat off the bus
        # wire's 24V with no adapter plugged in, and one is within budget; a
        # second collapses the supply a few seconds after its heater engages,
        # taking the whole bus down (every unit re-runs its power-up
        # self-check). Nothing on the wire refuses it, so it must be refused
        # here before the frame is sent. See _bus_supply_conflict.
        self.dry_bus_interlock = config.getboolean("dry_bus_interlock", True)
        self._following_lane: Optional[Any] = None
        # True while unit_unload_lane / eject_lane reels filament back. The
        # follower keep-alive tick's auto-arm must stand down for the duration:
        # its select+assist re-engage (fired because the lane is still
        # tool_loaded mid-unload) makes the bridge cancel the retract stream
        # (assist-on sets s_motion=0 in the firmware).
        self._unload_in_progress = False
        self._load_in_progress = False
        # The AMS's own words for the fault that ended the current load, kept
        # because _ams_declared_fault consumes the sequence and the recovery and
        # the final error both still need to know what was said. Cleared at the
        # start of every load so a stale verdict cannot describe a fresh
        # failure.
        self._declared_fault_text = None
        # True while an AMS drying cycle is running (AFC_BAMBU_HEATER_START..STOP).
        # The firmware already holds the follower off during drying so the AMS
        # can run its self-check/vent doors; mirror that here so the module's
        # keep-alive tick stops pumping follow/select frames the firmware would
        # only drop, and so get_status reflects the drying state.
        self._drying: bool = False
        #: Last (dryrem, when) seen from the unit. A frozen countdown
        #: repeats; a live one ticks -- see _dryrem_says_drying.
        self._dryrem_seen: Optional[Tuple[int, float]] = None
        # Chamber telemetry must be newer than this to say anything about the
        # current cycle. A start sets it to now (the very next reading counts, so
        # the panel leaves "Starting" as soon as the unit speaks); a stop sets it
        # to now + DRY_STOP_GRACE, because a stopping AMS emits another line or
        # two while winding down and those must not resurrect the cycle just
        # ended. 0.0 at boot, so a dry already running when Klipper starts is
        # still adopted.
        self._dry_adopt_after: float = 0.0
        # Whether this cycle has ever reported chamber telemetry. Required
        # before a silence can be read as "the cycle ended": a freshly started
        # dry has not reported yet and must not be released as finished.
        self._dry_seen_live: bool = False
        self._follow_last_e: Optional[float] = None
        self._follow_timer = None
        self._detector_timer = None
        self._uid_watch_timer = None
        #: Has unit_uid been resolved to a real chain index yet? Until it
        #: has, ams_index is only the config default, and announcing per-unit
        #: state at a guessed index registers this unit's flags/MC address
        #: against whichever unit really holds that index. A hand-written
        #: unit with no unit_uid counts as resolved from the start: its
        #: config index is the answer. A pool spare also has no unit_uid but
        #: its index is unknown until a uid claims it, so it defers on
        #: self.pool too. Otherwise unclaimed HT spares at index 0 would flag
        #: index 0 as an HT (htunit), the firmware would address it as 0x80,
        #: a boxed unit there would never answer, and the chain would never
        #: resolve.
        self._id_resolved = False
        self._announce_deferred = False
        # When the hold started, and whether we have already said it is
        # stuck -- the deferral is normal, never resolving is not.
        self._announce_defer_t0: float = 0.0
        self._announce_defer_warned: bool = False
        self._follow_last_log: float = 0.0
        # Log the AMS buffer position + follower state while following. Off by
        # default (0) -- the follower is watchable live via get_status
        # (follow_buff/follow_state/following), so this is only for deep tuning.
        # When >0 it rate-limits and only logs when the values actually change, so
        # even enabled it never streams a line every tick.
        self.follow_debug_interval = config.getfloat(
            "follow_debug_interval", 0.0, minval=0.0)
        self._follow_last_dbg: Optional[tuple] = None
        # Last status-apply failure text. Status frames arrive continuously, so
        # a stuck fault would warn on every frame; only a changed message is
        # logged.
        self._status_err_last: Optional[str] = None
        # Latched by AFC_BAMBU_FOLLOWER ENABLE=0. The latch is what holds the AMS
        # out of mode:4 to work on it: without it the auto-arm below re-engages
        # on the next ~100ms tick and undoes the manual stop. Cleared by
        # ENABLE=1 or by the next load, so it cannot strand a print.
        self._follow_manual_off: bool = False
        #   follow_rearm_window : how long after real extrusion a dropped
        # follower is still worth re-arming. state:0 is the AMS's resting state,
        # not a fault: it arms, finishes its assist within a second or two, and
        # reports 0 until something asks it for filament again. Re-arming on
        # state alone would loop (arm -> "assist finish 0" -> state:0 -> re-arm),
        # each time an LED flash and a motor nudge, so it is re-armed only if it
        # has dropped and the extruder actually wants filament.
        self.follow_rearm_window = config.getfloat(
            "follow_rearm_window", 3.0, above=0.0)
        self._follow_last_demand: float = 0.0
        # Latched when a stall pauses the print. Re-arming the follower against
        # a jam just makes the AMS grind on filament it cannot move, so the
        # auto-arm holds off until the print resumes (see _fault_hold_active).
        self._follow_fault_hold: bool = False
        self._follow_fault_saw_pause: bool = False
        # Latched when a link loss paused the print, so the detector fires once
        # per outage rather than every follower tick.
        self._link_loss_paused: bool = False
        self._slot_map: Dict[str, int] = {}
        self._bridge: Optional[BambuBridge] = None
        self._slots: List[dict] = [{} for _ in range(self.SLOTS_PER_UNIT)]
        self._prev_present: List[bool] = [False] * self.SLOTS_PER_UNIT
        self._auto_scanned: List[bool] = [False] * self.SLOTS_PER_UNIT
        # When this slot's scan was commanded; this alone is the hold.
        # None = no scan open, use the bay's record as it comes. Set = a scan
        # is waiting on the unit's answer, so nothing from this bay may reach
        # the lane yet (it still reports the previous spool's record until the
        # reader sees the new tag). _scan_verdict turns this plus the unit's
        # narration into the answer.
        self._scan_t0: List[Optional[float]] = [None] * self.SLOTS_PER_UNIT
        # Latch: the unit finished this slot's scan and read no tag. Stops the
        # defaults being re-applied on every status frame, and is cleared by a
        # removal or a new scan, the only two things that can change the
        # answer.
        self._scan_notag: List[bool] = [False] * self.SLOTS_PER_UNIT
        # Bays whose lane AFC already owns, recorded at PREP: occupied, and the
        # lane came back from the var file with a profile or Spoolman link. The
        # status path leaves those lanes alone until the spool comes out or the
        # unit answers a scan for that bay.
        self._afc_owned: set = set()
        # Has PREP walked this unit's lanes yet? The bridge polls well before
        # prep restores the var file, so the status path holds its filament
        # half until prep has run. Presence and lane status still mirror through.
        self._prep_seen: bool = False
        # Separate from _scan_t0: when the scan's physical motion began.
        # _scan_t0 is cleared on read success; this one is not.
        self._scan_motion_t0: List[Optional[float]] = (
            [None] * self.SLOTS_PER_UNIT)
        # False until the startup presence baseline is recorded, so spools
        # already inserted at boot don't fire a scan (see _maybe_auto_scan).
        self._scan_primed: bool = False
        # Whether the bridge on this connection has answered a presence poll
        # from this unit yet (see _presence_known), and whether scan priming
        # came due before it had and is waiting for it.
        self._presence_ok: bool = False
        self._presence_wait_said: bool = False
        self._prime_waiting: bool = False
        self._reread_wait_t0: Optional[float] = None
        # Bays _restore_untagged_defaults gave defaults on this connection:
        # nothing was restored there, so a tag that arrives afterwards is a
        # read, not restored state to hold off (see _boot_hold).
        self._defaulted_bays: set = set()
        # Frames from a bridge that had not polled the unit were seen, or a
        # reconnect happened: the next answered frame checks the open scans
        # (_close_scans_the_bridge_lost) and, after a reconnect, which bays
        # were occupied before it (_present_seen).
        self._booted_under_us: bool = False
        self._scans_to_check: bool = False
        self._seed_present_seen: bool = False
        # Bays a scan has actually run on during this connection. Empty at
        # boot: until a bay is in here nothing has looked at what is
        # physically in it, so the unit's cached tag record is the same one
        # the lane was already saved from. See _surface_slot_info's boot hold.
        self._scanned_bays: set = set()
        # Bays whose one Spoolman lookup has gone out on this connection (see
        # _handle_disconnect for the rule). Created here, not by the first
        # dispatch: _sync_owed and _bind_coming read a missing latch as "no
        # bind coming", which in a fresh process would let the bind that
        # follows a scan load Spoolman's stored weight over the measurement.
        self._spoolman_latched: set = set()
        # Bays seen occupied on this bridge connection, and of those, bays
        # whose removal edge this connection then saw: an insert into one of
        # those during a print waits on lane defaults for its read
        # (_defaults_until_read), from the time in _defaults_due. The first
        # set exists because a rebooted bridge reports every bay empty before
        # its first presence poll: that "removal" compares the old
        # connection's presence with the new one's, and is not one this
        # connection saw.
        self._present_seen: set = set()
        self._removed_bays: set = set()
        self._defaults_due: dict = {}
        # Bays whose lane such a removal edge cleared, kept as long as
        # _scanned_bays and not reset when the bridge link drops: nothing on
        # the lane since is state AFC restored, so the boot hold does not keep
        # the unit's later read of the bay off it (_boot_hold). A link that
        # drops between a mid-print insert and that read changes nothing on
        # the lane.
        self._cleared_bays: set = set()
        # slot -> the (meas_seq, meas_pct) the bay carried when its spool came
        # out. A presence flap keeps the record, stamp included, and hands it
        # back on the re-insert; it is the departed spool's measurement, not
        # one taken now. See the adopt block in _sync_lanes.
        self._meas_departed: dict = {}
        # _lookup_unbound's state: slot -> when a lookup Spoolman did not
        # answer may be asked again, and slot -> (uid, spool id) for a match
        # it would not bind because the spool has no remaining weight. Both
        # live as long as the latch they explain (_handle_disconnect), not
        # the bridge link: a latched bay is not asked again after a link
        # drop, so its answer must not be forgotten there either.
        self._lookup_retry: dict = {}
        self._lookup_refused: dict = {}
        # Bays whose measurement stamp has been looked at on this connection.
        # The first look at a bay records the stamp it carries as seen and
        # applies nothing: a measurement is applied at the time it is taken,
        # and one already sitting in the firmware when the connection started
        # was taken before it. See _baseline_meas_stamp.
        self._stamp_looked: set = set()
        # Bays whose _meas_seen entry was recorded by that first look rather
        # than applied. Such an entry is not "already on the lane", so the
        # adopt block in _sync_lanes may not treat a repeat of its percent as
        # nothing new. Kept with _meas_seen, not with the connection.
        self._meas_baselined: set = set()
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
        # User-facing commands, mux'd by UNIT= (matches AFC_ACE).
        self.gcode = self.printer.lookup_object('gcode')
        self._register_gcode_commands()

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
            # Fluidd renders humidity only for sensor_types on its fixed list, and
            # aht4x is not on it; aht2x/aht3x are, but belong to ACE and OpenAMS.
            # Wrap their factories and dispatch on the section: one carrying
            # bambu_unit is ours, anything else goes to whoever had it.
            for _alias in ("aht3x", "aht2x"):
                _prev = getattr(pheaters, "sensor_factories", {}).get(_alias)

                def _dispatch(cfg: Any, _prev: Any = _prev) -> Any:
                    """
                    Route an aht sensor section to its rightful owner.

                    :param cfg: the sensor section's config wrapper
                    :param _prev: the factory this wrap displaced, if any
                    :return Any: our sensor for bambu_unit sections, else
                        whatever the previous factory builds
                    """
                    if cfg.get("bambu_unit", None) is not None:
                        return TemperatureBambu(cfg)
                    if _prev is not None:
                        return _prev(cfg)
                    return TemperatureBambu(cfg)

                pheaters.add_sensor_factory(_alias, _dispatch)
        except Exception as e:
            self.logger.warning(
                f"AFC_BambuAMS: temperature_bambu factory not registered ({e}); "
                "use a [temperature_bambu <name>] section instead")
    # -- lifecycle --


    def _register_gcode_commands(self) -> None:
        """Register this unit's user-facing g-code commands (mux'd by UNIT=,
        matching AFC_ACE; the ones that act on a lane also take LANE= alone,
        see _register_lane_command) plus the printer-wide AFC_BAMBU_UIDS and
        AFC_BAMBU_FAULT_RELOAD."""
        _register_lane_command(
            self, "AFC_BAMBU_FOLLOWER", self.cmd_AFC_BAMBU_FOLLOWER,
            desc="Engage/stop the AMS self-centering follower (mode:4) for a "
                 "loaded lane. AFC_BAMBU_FOLLOWER LANE=<lane> [ENABLE=1]")
        _register_lane_command(
            self, "AFC_BAMBU_PRIME", self.cmd_AFC_BAMBU_PRIME,
            desc="Issue a feeder-only preload (bb_prime, ~2s, reader dormant) to "
                 "pull one bay's filament up to the hub -- the motion-only "
                 "pull-in. AFC_BAMBU_PRIME LANE=<lane>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_BITE", "UNIT", self.name, self.cmd_AFC_BAMBU_BITE,
            desc="Set the extruder BITE taken at the toolhead sensor before "
                 "the follower is armed, in mm (0 = none, advance tool_stn in "
                 "one go). AFC_BAMBU_BITE UNIT=<unit> [MM=<mm>]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_ARRIVAL", "UNIT", self.name, self.cmd_AFC_BAMBU_ARRIVAL,
            desc="Set the delay between the arrival and arming the hold; the "
                 "printer waits ~4s. Runtime only. "
                 "AFC_BAMBU_ARRIVAL UNIT=<unit> [ASSIST=<seconds>]")
        _register_lane_command(
            self, "AFC_BAMBU_RECOVER", self.cmd_AFC_BAMBU_RECOVER,
            desc="Recover a stuck/failed load: relink the AMS, stop motion, reel "
                 "the lane's filament back to the bay, and reset its state. "
                 "AFC_BAMBU_RECOVER LANE=<lane>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_RELINK", "UNIT", self.name, self.cmd_AFC_BAMBU_RELINK,
            desc="Force an AMS relink / error-recovery reset (deregister + "
                 "re-register) to clear a TIMEOUT/error state without a power "
                 "cycle. AFC_BAMBU_RELINK UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_SAVEIDS", "UNIT", self.name,
            self.cmd_AFC_BAMBU_SAVEIDS,
            desc="Commit the configured unit_uids to the bridge and restart "
                 "onto them, so a config change takes effect NOW instead of "
                 "at the next power-up. AFC_BAMBU_SAVEIDS UNIT=<unit> "
                 "[RESET=0] [RESTART=0]. Never during a print.")
        self.gcode.register_mux_command(
            "AFC_BAMBU_FLASH", "UNIT", self.name, self.cmd_AFC_BAMBU_FLASH,
            desc="Send a .uf2 to the bridge and have it write its own flash, "
                 "over the link Klipper already holds -- no bootloader, no "
                 "mount, no root. AFC_BAMBU_FLASH UNIT=<unit> "
                 "FILE=<name.uf2> [APPLY=0]. Never during a print.")
        _register_lane_command(
            self, "AFC_BAMBU_SCAN", self.cmd_AFC_BAMBU_SCAN,
            desc="Trigger an RFID/tag scan on demand -- the same read the "
                 "auto-scan runs on a fresh insert. AFC_BAMBU_SCAN LANE=<lane>, "
                 "or UNIT=<unit> alone for every slot on the unit. Use it on "
                 "the AMS HT, whose tag only reads when polled at 0x1800.",
            unit_alone=True)
        self.gcode.register_mux_command(
            "AFC_BAMBU_AUTOSCAN", "UNIT", self.name, self.cmd_AFC_BAMBU_AUTOSCAN,
            desc="Turn this unit's insert-edge tag scan on/off at runtime. "
                 "AFC_BAMBU_AUTOSCAN UNIT=<unit> ON=<0|1>")
        # A fault pauses the print with the toolhead empty; the pause message
        # names this command. Registered once per printer with no UNIT, since
        # a fault is printer-wide and the command finds the unit holding one:
        # the first unit registers it and the rest find the name taken. Not
        # guarded by a module-level flag, which would survive a Klipper RESTART
        # (same process, new printer) and leave the new printer without it.
        try:
            self.gcode.register_command(
                "AFC_BAMBU_FAULT_RELOAD", self.cmd_AFC_BAMBU_FAULT_RELOAD,
                desc="Reload whichever lane a Bambu AMS fault emptied, on any "
                     "unit -- it finds the one holding the fault. Safe to call "
                     "when nothing is wrong: it returns silently. "
                     "AFC_BAMBU_FAULT_RELOAD")
        except Exception:
            pass        # another [AFC_BambuAMS] on this printer registered it
        self.gcode.register_mux_command(
            "AFC_BAMBU_CLEARFAULT", "UNIT", self.name, self.cmd_AFC_BAMBU_CLEARFAULT,
            desc="Stream the printer's 0E clear at a parked unit for ~2s, then "
                 "report whether it actually left its fault. An ATTEMPT -- it "
                 "fails if the jam is still there. AFC_BAMBU_CLEARFAULT UNIT=<unit>")
        _register_lane_command(
            self, "AFC_BAMBU_REID", self.cmd_AFC_BAMBU_REID,
            desc="Send the printer menu's 're-identify' to one bay and nothing "
                 "else. On a boxed AMS this is the SECOND tag detection, "
                 "without which the unit cannot measure the spool. "
                 "AFC_BAMBU_REID LANE=<lane>")
        _register_lane_command(
            self, "AFC_BAMBU_CAPSCAN", self.cmd_AFC_BAMBU_CAPSCAN,
            desc="Run the printer's capacity-measuring re-scan on a bay: the "
                 "AMS re-reads the tag AND measures spool remain%. "
                 "AFC_BAMBU_CAPSCAN LANE=<lane>")
        _register_lane_command(
            self, "AFC_BAMBU_FEED", self.cmd_AFC_BAMBU_FEED,
            desc="Feed a bounded length from a lane's slot. "
                 "AFC_BAMBU_FEED LANE=<lane> [MM=20] [SPEED=]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_BUFFER_PROBE", "UNIT", self.name, self.cmd_AFC_BAMBU_BUFFER_PROBE,
            desc="Dump the raw AMS motion reply + buffer decode state. "
                 "AFC_BAMBU_BUFFER_PROBE UNIT=<unit>")
        # The drying commands are registered directly, with no cfg macro. Do
        # not also define a [gcode_macro Bambu_Heater_Start]: Klipper
        # upper-cases macro names, so it would collide with
        # AFC_BAMBU_HEATER_START.
        self.gcode.register_mux_command(
            "AFC_BAMBU_HEATER_START", "UNIT", self.name, self.cmd_AFC_BAMBU_HEATER_START,
            desc="Start AMS drying (AMS2 Pro / AMS HT). "
                 "AFC_BAMBU_HEATER_START UNIT=<unit> [TEMP=55] [TIME=480] [ROTATE=0]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_HEATER_STOP", "UNIT", self.name, self.cmd_AFC_BAMBU_HEATER_STOP,
            desc="Stop AMS drying. AFC_BAMBU_HEATER_STOP UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_SNIFF", "UNIT", self.name, self.cmd_AFC_BAMBU_SNIFF,
            desc="Put this unit's bridge into passive listen-only mode, or back. "
                 "AFC_BAMBU_SNIFF UNIT=<unit> ON=<0 or 1>")
        # Bus-wide UID list (not per-unit). Reads the UIDs straight off the wire
        # so you can copy them into each unit's `unit_uid`. Registered once;
        # guarded because every daisy-chained unit runs this init.
        try:
            self.gcode.register_command(
                "AFC_BAMBU_UIDS", self.cmd_AFC_BAMBU_UIDS,
                desc="List the AMS UIDs on the bus (chain index -> UID, with what "
                     "each holds) so you can pin units via unit_uid.")
        except Exception:
            pass        # another [AFC_BambuAMS] on this bus already registered it
    def _gcmd_lane_slot(self, cmd: str, gcmd: Any,
                        lane_name: Optional[str]) -> Tuple[Any, int]:
        """
        Resolve a lane command's LANE to this unit's lane and AMS slot.

        Raises the command's error when the lane is not on this unit, the
        bridge is not connected, or the lane has no AMS slot.

        :param cmd: The command name, for the error text
        :param gcmd: The Klipper GCodeCommand
        :param lane_name: The LANE argument
        :return tuple: (lane, slot)
        """
        lane = (getattr(self, "lanes", None) or {}).get(lane_name)
        if lane is None:
            raise gcmd.error(
                f"{cmd}: lane '{lane_name}' not on unit {self.name} "
                f"(lanes: {', '.join(getattr(self, 'lanes', None) or {}) or 'none'})")
        if self._bridge is None:
            raise gcmd.error(f"{cmd}: bridge not connected for {self.name}")
        slot = self._slot_of(lane)
        if slot is None:
            raise gcmd.error(f"{cmd}: {lane_name} is not mapped to an AMS slot")
        return lane, slot

    def cmd_AFC_BAMBU_PRIME(self, gcmd: Any) -> None:
        """
        Issue a feeder-only preload (bb_prime) for one bay.

        AFC_BAMBU_PRIME LANE=<lane> [UNIT=<unit>]

        Runs only the bay's feeder (mode:09) for ~2s to pull its filament up to
        the hub, with the RFID reader dormant: the motion-only "pull-in" a real
        printer does on insert, with no scan or measure. To do the
        insert-while-loaded pull-in by hand: drop the follower first
        (AFC_BAMBU_FOLLOWER ... ENABLE=0), prime the new bay, then re-engage.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_PRIME LANE=<lane> [UNIT=<unit>]`
        """
        lane_name = gcmd.get("LANE")
        lane, slot = self._gcmd_lane_slot("AFC_BAMBU_PRIME", gcmd, lane_name)
        self._bridge.send({"cmd": "prime", "unit": self.ams_index, "slot": slot})
        gcmd.respond_info(
            f"AFC_BAMBU_PRIME: sent feeder-only prime (~2s, reader dormant) to "
            f"{lane_name} (slot {slot}) on {self.name}")

    def cmd_AFC_BAMBU_FOLLOWER(self, gcmd: Any) -> None:
        """
        Manually engage or stop the follower for a lane's AMS tray.

        AFC_BAMBU_FOLLOWER LANE=<lane> [UNIT=<unit>] [ENABLE=1]

        ENABLE=1 (default) runs the finish->select->assist sequence that flips
        the tray to mode:4 and holds it (LED should start flashing); ENABLE=0
        stops the follower (LED goes solid). Use it to verify the follower on a
        tool-loaded lane independent of the load/startup paths.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_FOLLOWER LANE=<lane> [UNIT=<unit>] ENABLE=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_FOLLOWER LANE=lane1 ENABLE=1
        ```
        """
        lane_name = gcmd.get("LANE")
        enable = gcmd.get_int("ENABLE", 1)
        lane, _slot = self._gcmd_lane_slot("AFC_BAMBU_FOLLOWER", gcmd, lane_name)
        if enable:
            self._follow_manual_off = False
            self._follow_fault_hold = False
            self._auto_recover_armed = False   # re-arm auto recovery
            self._follow_fault_saw_pause = False
            self._engage_follower(lane)
            gcmd.respond_info(
                f"AFC_BAMBU_FOLLOWER: engaged follower (mode:4) for {lane_name} on "
                f"{self.name}; LED should flash. If it stays solid, the tray did "
                f"not reach mode:4.")
        else:
            # Latch it off, or the auto-arm re-engages on the next tick and the
            # stop appears to do nothing.
            self._follow_manual_off = True
            self.set_feed_assist(lane, False)
            gcmd.respond_info(
                f"AFC_BAMBU_FOLLOWER: stopped follower for {lane_name} on "
                f"{self.name}; LED should go solid. Stays off until "
                f"AFC_BAMBU_FOLLOWER LANE={lane_name} ENABLE=1 or the next "
                f"load.")

    def cmd_AFC_BAMBU_BITE(self, gcmd: Any) -> None:
        """
        Set the extruder bite taken at the toolhead sensor, at runtime.

        AFC_BAMBU_BITE UNIT=<unit> [MM=<mm>]

        With no arguments, reports the current value.

        The bite is the small advance the extruder takes the moment the sensor
        reads filament, before the follower is armed, so the gears have hold
        while the AMS runs its own pull-and-push. The remainder of tool_stn is
        fed afterwards. See docs/THE_LOAD.md step 7.

        MM=0 disables the split and advances tool_stn in one go.

        The value is not written to config: a restart returns it to the
        configured tool_bite_mm.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_BITE UNIT=<unit> MM=<mm>`

        Example
        -------
        ```
        AFC_BAMBU_BITE UNIT=BambuAMS_1 MM=1.0
        ```
        """
        mm = gcmd.get_float("MM", None, minval=0.0, maxval=50.0)
        if mm is not None:
            self.tool_bite_mm = mm
        gcmd.respond_info(
            f"AFC_BAMBU_BITE {self.name}: bite="
            f"{'OFF (single advance)' if self.tool_bite_mm <= 0 else f'{self.tool_bite_mm:.2f}mm'}"
            f"{'' if mm is None else '  (runtime only -- a restart restores the config value)'}")

    def cmd_AFC_BAMBU_ARRIVAL(self, gcmd: Any) -> None:
        """
        Set the arrival assist delay at runtime: the longest the load waits,
        after the toolhead sensor trips, for the unit to report it has stopped
        before arming the hold (see _wait_arrival_settle).

        AFC_BAMBU_ARRIVAL UNIT=<unit> [ASSIST=<seconds>]

        With no arguments, reports the current value. The printer itself
        waits a few seconds here; the gap is where the feeder's transmission
        comes out of reverse.

        Runtime only; a restart restores the configured arrival_assist_delay_s.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_ARRIVAL UNIT=<unit> ASSIST=<value>`

        Example
        -------
        ```
        AFC_BAMBU_ARRIVAL UNIT=BambuAMS_1 ASSIST=4.0
        ```
        """
        delay = gcmd.get_float("ASSIST", None, minval=0.0, maxval=30.0)
        if delay is not None:
            self.arrival_assist_delay_s = delay
        note = "" if delay is None else "  (runtime only)"
        gcmd.respond_info(
            f"AFC_BAMBU_ARRIVAL {self.name}: "
            f"assist-delay={self.arrival_assist_delay_s:.2f}s "
            f"(the printer waits ~4s){note}")

    def cmd_AFC_BAMBU_RECOVER(self, gcmd: Any) -> None:
        """
        Recover a stuck / failed load: stop motion, reel the lane's filament
        back to the bay, and reset its state so AFC is no longer mid-operation.

        AFC_BAMBU_RECOVER LANE=<lane> [UNIT=<unit>]

        Use after a load errors out (e.g. a feeder "rocker stall"): the AMS is
        left idle but with filament staged partway in the path and the lane
        stuck in a load/error state. This halts the AMS, winds the filament back
        into the bay (the shared eject reel-back), and clears the lane so you can
        re-insert / retry. If the feeder still stalls after this, the bay's
        filament tip is jammed -- open the AMS, trim the tip, and reinsert.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_RECOVER LANE=<lane> [UNIT=<unit>]`

        Example
        -------
        ```
        AFC_BAMBU_RECOVER LANE=lane1
        ```
        """
        lane_name = gcmd.get("LANE")
        lane, _slot = self._gcmd_lane_slot("AFC_BAMBU_RECOVER", gcmd, lane_name)
        gcmd.respond_info(
            f"AFC_BAMBU_RECOVER: stopping and reeling {lane_name} back to the bay...")
        self._recover_to_bay(lane)
        gcmd.respond_info(
            f"AFC_BAMBU_RECOVER: {lane_name} reset. If a load still stalls the "
            f"feeder, the bay filament tip is jammed -- open the AMS, trim the "
            f"tip, and reinsert.")

    def cmd_AFC_BAMBU_SAVEIDS(self, gcmd: Any) -> None:
        """
        Commit the configured unit_uids to the bridge and restart onto them.

        Usage
        -----
        `AFC_BAMBU_SAVEIDS UNIT=<unit> [RESET=0] [RESTART=0]`

        The identity table (uid -> chain index + model) lets the Pico enrol
        correctly the moment it powers up, instead of letting announce order
        (which changes between reboots) decide. Prep writes it automatically,
        but only the next power-up reads it. This does the whole thing at
        once: push, store, reboot the Pico onto the stored table, restart
        Klipper onto the result.

        Run it after putting the UIDs from AFC_BAMBU_UIDS into printer.cfg, or
        after changing one. Never during a print -- it drops the bus.

        The firmware replies to every idsave, so the Pico is not rebooted until
        the record is confirmed written; there is no timed sleep. The reboot
        itself is not waited on: Klipper's own restart takes longer than the
        Pico's, and re-resolves the chain on reconnect.

        RESET=0 stores and stops (no reboot, effective next power-up).
        RESTART=0 reboots the Pico but leaves Klipper alone -- expect the
        bridge to be offline until you restart it yourself.

        :param gcmd: Klipper gcode command object
        """
        do_reset = gcmd.get_int("RESET", 1) != 0
        do_restart = gcmd.get_int("RESTART", 1) != 0
        do_wipe = gcmd.get_int("WIPE", 0) != 0
        # Not while printing: this drops the bus and restarts the host.
        _refuse_while_printing(self.printer, self.afc.reactor, gcmd,
                               "AFC_BAMBU_SAVEIDS")
        bridge = self._bridge
        if bridge is None:
            error_str = "AFC_BAMBU_SAVEIDS: bridge not connected"
            raise gcmd.error(error_str)
        try:
            with bridge._lock:
                bridge._last_idsave = None
        except Exception:
            pass
        if do_wipe:
            # Erase instead of store. The next power-up then enrols from
            # announce order, and the prep after that writes a record from
            # scratch; use it for a chain whose units have been swapped, or to
            # exercise the write path on real flash.
            bridge.send({"cmd": "idwipe"})
        else:
            self._send_bindings(bridge)      # binds + the idsave that follows
        # Wait for the firmware's answer. reactor.pause yields, so this does
        # not hold the reactor (as the UID resolve at ready does).
        reactor = self.afc.reactor
        end = reactor.monotonic() + 5.0
        got = None
        while reactor.monotonic() < end:
            try:
                with bridge._lock:
                    got = bridge._last_idsave
            except Exception:
                got = None
            if got is not None:
                break
            reactor.pause(reactor.monotonic() + 0.05)
        if got is None:
            error_str = ("AFC_BAMBU_SAVEIDS: the bridge never answered; nothing was "
                "changed and the Pico was NOT rebooted")
            raise gcmd.error(error_str)
        state, n = got[0], got[1]
        if state == "failed":
            error_str = ("AFC_BAMBU_SAVEIDS: the bridge could not store the "
                "identities; the Pico was NOT rebooted. The chain will keep "
                "enrolling from announce order.")
            raise gcmd.error(error_str)
        self.gcode.respond_info(
            f"AFC_BAMBU_SAVEIDS: {n} unit identities "
            + {"written": "stored", "wiped": "ERASED"}.get(state,
                                                           "already stored"))
        if not do_reset:
            self.gcode.respond_info(
                "AFC_BAMBU_SAVEIDS: RESET=0 -- they take effect at the next "
                "power-up.")
            return
        try:
            bridge.send({"cmd": "reset"})    # reboot into the same firmware
        except Exception as e:
            error_str = f"AFC_BAMBU_SAVEIDS: reset failed: {e}"
            raise gcmd.error(error_str)
        if not do_restart:
            self.gcode.respond_info(
                "AFC_BAMBU_SAVEIDS: Pico rebooting; RESTART=0, so the bridge "
                "stays offline until you restart Klipper.")
            return
        self.gcode.respond_info(
            "AFC_BAMBU_SAVEIDS: Pico rebooting onto the stored identities; "
            "restarting Klipper onto the result.")
        self.gcode.run_script_from_command("FIRMWARE_RESTART")

    def _resolve_uf2(self, name: str) -> str:
        """
        Turn a FILE= value into a path, keeping a relative one inside the
        printer's config directory.

        A bare name is resolved against the config directory, because that is
        where a file uploaded through Moonraker lands and it is what someone at
        the console would type. An absolute path is taken as given -- a person
        with a shell needs no protecting from themselves -- but a relative one
        is not allowed to climb out with "..", since the same command can be
        driven from a browser.

        The config directory comes from `printer.get_start_args()
        ["config_file"]`, the path klippy was started with (configfile's
        get_status() does not name the file). `~/printer_data/config` is only
        the fallback, and is wrong on a printer whose Klipper runs without
        that HOME.

        :param name: The FILE= value
        :return str: Absolute path to the .uf2
        """
        if os.path.isabs(name):
            return name
        cfg_dir = ""
        try:
            cfg_dir = os.path.dirname(
                self.printer.get_start_args().get("config_file", "") or "")
        except Exception:
            cfg_dir = ""
        if not cfg_dir:
            cfg_dir = os.path.expanduser("~/printer_data/config")
        full = os.path.normpath(os.path.join(cfg_dir, name))
        if not full.startswith(os.path.normpath(cfg_dir) + os.sep):
            error_str = f"{name} is outside the config directory"
            raise ValueError(error_str)
        return full

    def _wait_fw(self, bridge: Any, want: tuple, timeout: float,
                 skip: tuple = ()) -> tuple:
        """
        Wait for the bridge's next {"evt":"fw"} line.

        :param bridge: The bridge transport
        :param want: States that count as success
        :param timeout: Seconds to wait
        :param skip: Intermediate states to keep waiting through rather than
            return, e.g. the sha_ok that precedes sig_ok when the manifest is
            sent as two commands, so an earlier ack is not read as the answer
            to a later question.
        :return tuple: (state, detail); state is "" when nothing arrived
        """
        reactor = self.afc.reactor
        end = reactor.monotonic() + timeout
        while reactor.monotonic() < end:
            with bridge._lock:
                got = bridge._last_fw
            if got is not None and got[0]:
                if got[0] in skip:
                    with bridge._lock:
                        if bridge._last_fw == got:
                            bridge._last_fw = None      # consume, keep waiting
                    reactor.pause(reactor.monotonic() + 0.02)
                    continue
                if got[0] in want:
                    return got
                return got                       # a real answer, just not ours
            reactor.pause(reactor.monotonic() + 0.02)
        return ("", "no answer")



    def cmd_AFC_BAMBU_FLASH(self, gcmd: Any) -> None:
        """
        Send a .uf2 to the bridge and have it write its own flash.

        Usage
        -----
        `AFC_BAMBU_FLASH UNIT=<unit> FILE=<name.uf2> [APPLY=0]`

        Flashes over the link Klipper already holds, with no bootloader
        mode, mount or root access needed on the host.

        Nothing is erased until the whole image has arrived and its CRC has
        matched. The image is staged in the Pico's RAM; a dropped link, a
        truncated file or a lost byte ends the transfer with the running
        firmware untouched. Only then does the bridge erase and program itself,
        and that write -- a second or two -- is the one window where an
        interruption leaves it needing the ROM bootloader. Hold BOOTSEL and
        replug if that ever happens; the case has a hole over the button.

        APPLY=0 transfers and verifies without writing anything: it exercises
        the whole path (file, link, CRC) and leaves the firmware as it was.

        ABORT=1 sends {"cmd":"fwabort"} to drop a staged transfer and nothing
        else. It clears the one wedge nothing else does: an image that arrives
        intact and is then refused at the signature check. fw_tick's 5s stall
        release is gated on fw_active() (`staged() && !s_ready`), and a
        transfer that reached the digest has s_ready set, so every later
        fwbegin answers "busy: a transfer is already running" until the board
        is power cycled or aborted.

        :param gcmd: Klipper gcode command object
        """
        name = gcmd.get("FILE", ".bridge_firmware.uf2")
        do_apply = gcmd.get_int("APPLY", 1) != 0

        if gcmd.get_int("ABORT", 0):
            bridge = self._bridge
            if bridge is None:
                raise gcmd.error("AFC_BAMBU_FLASH: bridge not connected")
            bridge.send({"cmd": "fwabort"})
            gcmd.respond_info(
                "AFC_BAMBU_FLASH: sent fwabort; any staged transfer is dropped")
            return

        # Not while printing: the write stops the bus for a second or two and
        # then reboots the master.
        _refuse_while_printing(self.printer, self.afc.reactor, gcmd,
                               "AFC_BAMBU_FLASH")

        bridge = self._bridge
        if bridge is None:
            error_str = "AFC_BAMBU_FLASH: bridge not connected"
            raise gcmd.error(error_str)

        try:
            path = self._resolve_uf2(name)
            with open(path, "rb") as fh:
                blob = fh.read()
            # Images are sent as plain UF2; the signed manifest below is what
            # gates an update.
            img, img_chip = uf2_to_image(blob)
        except (OSError, ValueError) as e:
            error_str = f"AFC_BAMBU_FLASH: {e}"
            raise gcmd.error(error_str)

        # A wrong-chip image bricks the board (the bootloader ignores every
        # block and leaves nothing to boot), so the image's chip is checked
        # against what the bridge reports it is, not against a constant: both
        # RP2040 and RP2350 bridges exist.
        bridge_chip = None
        try:
            bridge_chip = bridge.chip()
        except AttributeError:
            bridge_chip = None
        if img_chip and bridge_chip and img_chip != bridge_chip:
            error_str = (f"AFC_BAMBU_FLASH: image is for {img_chip}, the "
                         f"bridge is {bridge_chip}")
            raise gcmd.error(error_str)
        if img_chip and not bridge_chip:
            self.gcode.respond_info(
                f"AFC_BAMBU_FLASH: bridge does not report its chip (older "
                f"firmware); sending a {img_chip} image unchecked")

        crc = zlib.crc32(img) & 0xFFFFFFFF
        self.gcode.respond_info(
            f"AFC_BAMBU_FLASH: {os.path.basename(path)} -> {len(img)} bytes, "
            f"crc 0x{crc:08X}")

        # Signed update. fw_sign.py leaves a .manifest.json beside every
        # release image; it is sent ahead of the transfer so the bridge can
        # refuse fwapply for anything the release key did not sign. Sent
        # before _fw_raw claims the port, as ordinary line commands. A missing
        # manifest is reported up front, since a signed-update bridge will
        # refuse the APPLY.
        man = None
        man_path = path + ".manifest.json"
        try:
            with open(man_path) as fh:
                man = json.load(fh)
        except (OSError, ValueError):
            man = None
        if man and man.get("format") in ("BBFW1", "BBFW2"):
            with bridge._lock:
                bridge._last_fw = None
            bridge.send({"cmd": "fwsha", "hex": man["sha512"]})
            # Flags only when the manifest has them. BBFW2 carries a signed
            # flags word (currently just "this image may be installed over a
            # newer build"), and the firmware picks its signed-message layout
            # from it, so relaying a flags key the manifest does not have would
            # make the board check a message the key never signed and refuse a
            # good image. The host passes it through uninterpreted; the
            # signature is what gives it meaning.
            sig_cmd = {"cmd": "fwsig", "ver": int(man["ver"]),
                       "hex": man["sig"]}
            if man.get("flags"):
                sig_cmd["flags"] = int(man["flags"])
            bridge.send(sig_cmd)
            # Wait for the manifest's ack before the transfer. Both commands
            # answer (sha_ok, sig_ok) after the send returns; consuming them
            # here keeps the fwbegin wait below from reading a stale sig_ok as
            # "the bridge would not start".
            got = self._wait_fw(bridge, ("sig_ok",), 5.0, skip=("sha_ok",))
            if got[0] != "sig_ok":
                error_str = (f"AFC_BAMBU_FLASH: the bridge rejected the "
                    f"manifest ({got[0] or 'silence'}: {got[1]})")
                raise gcmd.error(error_str)
            self.gcode.respond_info(
                f"AFC_BAMBU_FLASH: signed manifest v{man['ver']} accepted")
        else:
            self.gcode.respond_info(
                "AFC_BAMBU_FLASH: no .manifest.json beside the image -- a "
                "signed-update bridge (>= AFC-1.20) will refuse APPLY. Sign "
                "the image with the release signing tool (fw_sign.py)")

        # The port is exclusive from here. The reactor keeps running during the
        # transfer (or every other timer in Klipper stalls), and any timer that
        # sends while the firmware is counting bytes would write into the
        # middle of the image. Cleared in the finally, so a refusal at any
        # stage hands the port back.
        reactor = self.afc.reactor
        bridge._fw_raw = True
        try:
            with bridge._lock:
                bridge._last_fw = None
            bridge.send({"cmd": "fwbegin", "len": len(img), "crc": crc})
            got = self._wait_fw(bridge, ("ready",), 5.0)
            if got[0] != "ready":
                error_str = (f"AFC_BAMBU_FLASH: the bridge would not start "
                    f"({got[0] or 'silence'}: {got[1]}); nothing was changed")
                raise gcmd.error(error_str)

            # Paced so the reactor keeps running: a g-code that blocks for the
            # whole transfer would stall every other timer in Klipper with it.
            with bridge._lock:
                bridge._last_fw = None
            chunk = 1024
            for i in range(0, len(img), chunk):
                if not bridge.write_raw(img[i:i + chunk]):
                    error_str = ("AFC_BAMBU_FLASH: the link died mid-transfer; the "
                        "bridge discards a partial image, so nothing was "
                        "changed")
                    raise gcmd.error(error_str)
                reactor.pause(reactor.monotonic() + 0.002)

            got = self._wait_fw(bridge, ("crc_ok",), 30.0)
            if got[0] != "crc_ok":
                error_str = (f"AFC_BAMBU_FLASH: the image did not verify "
                    f"({got[0] or 'silence'}: {got[1]}); nothing was changed")
                raise gcmd.error(error_str)
        finally:
            bridge._fw_raw = False

        if not do_apply:
            bridge.send({"cmd": "fwabort"})
            self.gcode.respond_info(
                "AFC_BAMBU_FLASH: APPLY=0 -- the image transferred and "
                "verified, and was then discarded. The bridge is still "
                "running what it was running.")
            return

        with bridge._lock:
            bridge._last_fw = None
        bridge.send({"cmd": "fwapply"})
        got = self._wait_fw(bridge, ("applying",), 5.0)
        # Silence here is the normal outcome of a successful apply: the board
        # answers "applying" and then rewrites the flash it is executing from,
        # so the answer usually loses the race against its own reboot.
        #
        # A real refusal is not silent: the firmware rejects an image with an
        # fw event carrying a state, and that errors below. No answer at all
        # is reported as unconfirmed, pointing at the version string to
        # settle it.
        if got[0] and got[0] != "applying":
            error_str = (f"AFC_BAMBU_FLASH: the bridge refused the image "
                f"({got[0]}: {got[1]}); nothing was changed")
            raise gcmd.error(error_str)
        if not got[0]:
            self.gcode.respond_info(
                "AFC_BAMBU_FLASH: apply sent; the bridge stopped answering "
                "without confirming. That is the NORMAL outcome -- it reboots "
                "into the new image before its reply can land. NOT confirmed "
                "either way here: check the firmware line once the link is "
                "back (AFC_BAMBU_UIDS, or the info REPLY in the log).")
            return
        self.gcode.respond_info(
            "AFC_BAMBU_FLASH: the bridge is writing its own flash and will "
            "reboot. The link drops for a few seconds; AFC_BAMBU_UIDS shows "
            "the new build once it is back.")

    def cmd_AFC_BAMBU_RELINK(self, gcmd: Any) -> None:
        """
        Force an AMS relink / error-recovery reset for this unit.

        AFC_BAMBU_RELINK UNIT=<unit>

        Sends the firmware relink (deregister sweep + re-registration) to clear
        a unit stuck in a TIMEOUT/error state (state:7) without a power cycle.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_RELINK UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_RELINK UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_RELINK: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        self.relink()
        gcmd.respond_info(
            f"AFC_BAMBU_RELINK: sent AMS relink/reset for {self.name}.")

    def cmd_AFC_BAMBU_FEED(self, gcmd: Any) -> None:
        """
        Feed a bounded length from a lane's slot toward the toolhead.

        AFC_BAMBU_FEED LANE=<lane> [UNIT=<unit>] [MM=20] [SPEED=<mm/s>]

        The same feed primitive the load path uses, exposed on its own so the
        feed can be tested independently of the follower (which can sit armed
        in mode:4 without driving the motor).

        Also the only way to relieve a bottomed-out buffer from software:
        feeding separates the two PTFE ends and compresses the spring.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_FEED LANE=<lane> [UNIT=<unit>] MM=<mm> SPEED=<value>`

        Example
        -------
        ```
        AFC_BAMBU_FEED LANE=lane1 MM=20.0 SPEED=1.0
        ```
        """
        lane_name = gcmd.get("LANE")
        mm = gcmd.get_float("MM", 20.0, above=0.0, maxval=200.0)
        speed = gcmd.get_float("SPEED", 0.0, minval=0.0)
        lane, _slot = self._gcmd_lane_slot("AFC_BAMBU_FEED", gcmd, lane_name)
        ok = self.feed(lane, mm, speed if speed > 0 else None)
        gcmd.respond_info(
            f"AFC_BAMBU_FEED: {'issued' if ok else 'FAILED to issue'} {mm:.0f}mm on "
            f"{lane_name} ({self.name}).")

    def cmd_AFC_BAMBU_AUTOSCAN(self, gcmd: Any) -> None:
        """
        Turn this unit's insert-edge tag scan on or off at runtime.

        AFC_BAMBU_AUTOSCAN UNIT=<unit> ON=<0|1>

        With the scan on, the host sends a scan as soon as it sees the insert
        edge (select + set tray_readid). With it off, the insert is left
        entirely to the AMS, which runs its own sequence (odometer calibration,
        then the RFID read) while the host sends only its four polls -- 1A/02,
        11/04, 3C/02, 37/02. A boxed AMS that receives the scan reports
        "STEP:odom invalid tray 0" and skips its odometer calibration.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_AUTOSCAN UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_AUTOSCAN UNIT=BambuAMS_1 ON=1
        ```
        """
        on = gcmd.get_int("ON", 1, minval=0, maxval=1)
        self.auto_scan = bool(on)
        gcmd.respond_info(
            f"AFC_BAMBU_AUTOSCAN: {self.name} insert-edge scan "
            f"{'ON' if on else 'OFF -- the AMS is left to run its own sequence'}")






    def cmd_AFC_BAMBU_FAULT_RELOAD(self, gcmd: Any) -> None:
        """
        The reload-after-fault recovery, callable by name.

        Silent and harmless when there is nothing to recover: not paused, no
        Bambu fault pending, or the lane already loaded all return without
        output, so running it when nothing is wrong costs nothing.

        Takes no UNIT parameter: _resume_reload_target searches every
        [AFC_BambuAMS] on the machine and recovers whichever one holds the
        fault.

        Two kinds of failure, and only one stops the resume:

        A refusal (the reload ran and the toolhead is still empty) raises;
        AFC_RESUME does not catch it, so the print stays paused rather than
        continuing into an empty toolhead.

        Any other exception is a bookkeeping failure. It is logged and the
        resume proceeds, so the operator is never locked out; a missed reload
        can be done by hand.

        Usage
        -----
        `AFC_BAMBU_FAULT_RELOAD`

        Example
        -------
        ```
        AFC_BAMBU_FAULT_RELOAD
        ```

        :param gcmd: The Klipper GCodeCommand
        """
        try:
            self._reload_before_resume(gcmd)
        except self.printer.command_error:
            raise                   # refused on purpose; the message says why
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: reload-after-fault check failed "
                f"({e}); resuming anyway. If the toolhead is empty, pause and "
                f"load the lane by hand.")

    def _reload_before_resume(self, gcmd: Any) -> None:
        """
        Reload the faulted lane, or explain why the print must stay paused.

        Returns quietly whenever there is nothing to do: this runs on every
        resume on the printer, including ones unrelated to an AMS.

        The lane comes from the fault itself (`_fault_lane`, recorded when the
        stall was raised) before `afc.current`, because by resume time the
        unload has usually already cleared `current`.

        :param gcmd: The Klipper GCodeCommand (for operator-facing output)
        :raises gcmd.error: if the reload did not take
        """
        try:
            if not self.afc.function.is_paused():
                return
        except Exception:
            return
        # A hand-driven resume is the intervention auto_error_recovery_limit
        # waits for, so it hands back a full budget. Skipped when the recovery
        # is resuming us itself (_in_auto_recover is still set at that point),
        # which would otherwise clear the very count that caps it.
        try:
            units = list(self._bambu_units())
            if not any(getattr(u, "_in_auto_recover", False) for u in units):
                for u in units:
                    u._auto_resume_count = 0
        except Exception:
            pass
        unit, lane = self._resume_reload_target()
        if unit is None or lane is None:
            return
        name = getattr(lane, "name", None)
        if not name:
            return
        if getattr(lane, "tool_loaded", False):
            unit._resume_needs_reload = False
            return                  # already fed; nothing for us to do
        gcmd.respond_info(
            f"{unit.name}: reloading {name} before resuming (a Bambu fault "
            f"left the toolhead empty)...")
        self.gcode.run_script_from_command(f"CHANGE_TOOL LANE={name}")
        if not getattr(lane, "tool_loaded", False):
            error_str = (f"{unit.name}: {name} did NOT reload -- the toolhead is empty, "
                f"so the print stays PAUSED. Clear the jam and resume again.")
            raise gcmd.error(error_str)
        unit._resume_needs_reload = False
        unit._auto_recover_armed = False     # a fresh fault may retry

    def _resume_reload_target(self) -> tuple:
        """
        Find the (unit, lane) a resume should reload, or (None, None).

        RESUME is printer-wide, so this searches every Bambu unit with a fault
        pending, not only this one. Whichever unit raised the fault owns the
        reload, regardless of which unit's wrapper is running.

        :return tuple: (unit, lane) or (None, None) if no reload is pending
        """
        for unit in self._bambu_units():
            if not getattr(unit, "_resume_needs_reload", False):
                continue
            lane = getattr(unit, "_fault_lane", None)
            if lane is not None:
                return unit, lane
            # The fault latch cleared its lane (buffer auto-reset does that)
            # but the reload is still owed. Fall back to what AFC thinks is
            # current, and only if it belongs to this unit.
            try:
                cur = getattr(self.afc, "current", None)
            except Exception:
                cur = None
            if cur and cur in unit.lanes:
                return unit, unit.lanes[cur]
        return None, None

    def _bambu_units(self) -> list:
        """
        Every [AFC_BambuAMS] unit on this printer, self first.

        Derived from AFC's lane table rather than a separate registry: lanes
        already point at their units, and a unit with no lanes cannot own a
        reload anyway.

        :return list: the Bambu units, self first, no duplicates
        """
        units = [self]
        try:
            for lane in self.afc.lanes.values():
                unit = getattr(lane, "unit_obj", None)
                if isinstance(unit, type(self)) and unit not in units:
                    units.append(unit)
        except Exception:
            pass
        return units



    def cmd_AFC_BAMBU_CLEARFAULT(self, gcmd: Any) -> None:
        """
        Stream the printer's fault clear at a parked unit, then check it took.

        AFC_BAMBU_CLEARFAULT UNIT=<unit>

        While a unit is parked the state channel carries op-04 0F/00 and the
        drive channel mirrors 0F/FF at 21ms. The clear is op-03 0E/FF streamed
        on the drive channel for ~2s (96-98 frames), matching what the printer
        sends, rather than a single frame.

        This is an attempt, not a command: the unit clears its own err_code
        only when it accepts a fresh operation, and it will not accept one
        while still jammed. So the unit state is checked afterwards and the
        actual outcome reported.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_CLEARFAULT UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_CLEARFAULT UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            error_str = "AFC_BAMBU_CLEARFAULT: bridge not connected"
            raise gcmd.error(error_str)
        before = self._unit_state(self._bridge.latest_status())
        # Addressed: the firmware builds the drive-channel frame as
        # { 0x03, unit_byte(s_tunit), ... }, so without a unit the clear goes
        # to whichever unit s_tunit last pointed at.
        self._bridge.send({"cmd": "clearfault",
                           "unit": int(getattr(self, "ams_index", 0) or 0)})
        # The burst is ~2s; wait it out plus a little for the state to settle.
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 2.6)
        after = self._unit_state(self._bridge.latest_status())
        if after in self.AMS_STATES_FAULTED:
            gcmd.respond_info(
                f"AFC_BAMBU_CLEARFAULT: {self.name}: STILL FAULTED (state {after}). "
                f"The unit will not accept a fresh operation while the jam is "
                f"there -- relieve the pressure (the buffer coming off the "
                f"floor is the sign) and run this again.")
            return
        # A unit that was not faulted before the burst is not reported as
        # "cleared": whatever the operator is seeing is caused by something
        # this command does not reach.
        if before not in self.AMS_STATES_FAULTED:
            gcmd.respond_info(
                f"AFC_BAMBU_CLEARFAULT: {self.name}: nothing to clear -- the "
                f"unit was not in a fault state before the burst (state "
                f"{before}) and is not now ({after}). The clear was still sent "
                f"to unit {getattr(self, 'ams_index', 0)}. If its LEDs are red, "
                f"the cause is not a parked feeder: try AFC_BAMBU_RELINK "
                f"UNIT={self.name}, which re-registers the unit without a power "
                f"cycle.")
            return
        gcmd.respond_info(
            f"AFC_BAMBU_CLEARFAULT: {self.name}: cleared (state {before} -> "
            f"{after}); the unit accepted the operation and left its park.")















    def cmd_AFC_BAMBU_BUFFER_PROBE(self, gcmd: Any) -> None:
        """
        Dump the AMS's raw motion reply and the buffer decode's own state.

        AFC_BAMBU_BUFFER_PROBE UNIT=<unit>

        For working out where (or whether) an AMS model reports its FPS buffer.
        The mapped 0..100 value cannot show a decode that never ran or a
        calibration that saturates, so this prints the raw frame alongside
        reads/replylen/raw. Hold the buffer at a known position and compare
        frames to find the byte that tracks it.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_BUFFER_PROBE UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_BUFFER_PROBE UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_BUFFER_PROBE: bridge not connected for {self.name}"
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
            # Per-unit receipt for this unit's follower arm; fstate above is
            # bus-wide, so the two can disagree.
            f"arm_acked={self._follow_arm_acked(latest)} "
            # Narration counters: polls = drain requests sent, frames =
            # narration-shaped replies, texts = replies that carried text.
            # Distinguishes "the AMS has nothing to say" from "not asking".
            f"dbg_polls={latest.get('dbgpolls')} "
            f"dbg_frames={latest.get('dbgframes')} "
            f"dbg_texts={latest.get('dbgtexts')} "
            f"dbg_cut={latest.get('dbgtrunc')} "
            # The MC-address announce as acknowledged by the firmware. Without
            # it the drain polls default to 0x0700, so an HT at 0x1800 is never
            # asked and dbg_texts stays 0 on an otherwise healthy bus.
            f"mcack={self._mcaddr_ack_str()} "
            # The buffer as the unit measures it, finer than the 0..100 status
            # field: buff_pos is its instantaneous position and refill is
            # (sagged_to, recovered_to, mm_fed) from its last on-demand top-up.
            f"tube_len={self.measured_path_mm()} "
            # The unit's end-of-feed length, the second source _path_measurement
            # tries for the bowden length. n= is the sample count.
            f"dw_len={self._dw_len_str()} "
            f"bowden={self.afc_bowden_length:.0f} "
            f"buff_pos={self._bridge_call('last_buff_pos')} "
            f"refill={self._bridge_call('last_buff_refill')}")
        gcmd.respond_info(f"{self.name}: frame={frame or '(none)'}")

    def _bridge_call_arg(self, name: str, arg: Any) -> Any:
        """
        Call an optional one-argument bridge accessor, returning None if it is
        absent or raises -- the accessor may legitimately not exist while the
        bridge is still connecting.

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

    def cmd_AFC_BAMBU_SCAN(self, gcmd: Any) -> None:
        """
        Trigger an RFID/tag scan on demand -- the exact read the auto-scan runs
        on a fresh insert, but callable by hand.

        AFC_BAMBU_SCAN LANE=<lane> [UNIT=<unit>]
        AFC_BAMBU_SCAN UNIT=<unit>

        With LANE, scans that lane's slot, and LANE alone finds the unit; with
        UNIT and no LANE, scans every slot on the unit. Mainly for the AMS HT,
        whose tag only reads when the bridge polls it at 0x1800 -- run this if
        a spool's material never populated. The read also clears the per-slot
        auto-scan latch so the result is fresh.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_SCAN LANE=<lane>`

        Example
        -------
        ```
        AFC_BAMBU_SCAN LANE=lane1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_SCAN: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        lane_name = gcmd.get("LANE", None)
        if lane_name is not None:
            lane, slot = self._gcmd_lane_slot("AFC_BAMBU_SCAN", gcmd,
                                              lane_name)
            busy = self._measure_in_flight_slot()
            if busy is not None:
                raise gcmd.error(
                    f"AFC_BAMBU_SCAN: {self.name} is already measuring slot "
                    f"{busy} (AMS bay {busy + 1}) -- one bay at a time, so this "
                    f"would be dropped. Wait ~40s, then retry.")
            # Drop the latch so the manual scan always re-reads, and open the
            # scan so it resolves the same way an auto-scan does: the unit
            # reads a tag, or the lane gets its defaults. Without the open, the
            # bay's leftover record would be taken as the answer.
            if 0 <= slot < len(self._auto_scanned):
                self._auto_scanned[slot] = False
            # A manual scan also measures: one tag read, then the spool
            # measurement (measured percent -> slot remain + lane grams).
            #
            # The scan window opens only after the capscan is accepted; opened
            # first, a refused bus claim would leave it armed with nothing
            # scanning, and the backstop would later finalize "no tag" and
            # wipe the lane.
            if not afcBambuAMS._start_capscan(self, slot):
                msg = (f"AFC_BAMBU_SCAN: scan command not issued for {lane_name} "
                       f"on {self.name}")
                raise gcmd.error(msg)
            self._open_scan(slot)
            gcmd.respond_info(
                f"AFC_BAMBU_SCAN: scanning {lane_name} (slot {slot}) on {self.name} "
                f"at 0x{getattr(self, 'dry_dev_addr', 0):04X}; material lands "
                f"when the tag reads, remaining %% when the measurement "
                f"finishes (~30s).")
            return
        nslots = min(len(self._auto_scanned),
                     getattr(self, "unit_slots", len(self._auto_scanned)))
        for i in range(nslots):
            self._auto_scanned[i] = False
            # Only a bay with a spool in it: an empty one has no question to
            # answer, and opening a scan on it would just walk it to "no tag".
            if (self._slots[i] or {}).get("present"):
                self._open_scan(i)
        if not self.scan(None):
            msg = f"AFC_BAMBU_SCAN: scan command not issued for {self.name}"
            raise gcmd.error(msg)
        gcmd.respond_info(
            f"AFC_BAMBU_SCAN: scanning {nslots} slot(s) on {self.name} at "
            f"0x{getattr(self, 'dry_dev_addr', 0):04X}.")

    def _run_calibrate(self, lane: Any, slot: int) -> Optional[str]:
        """
        Run a spool measurement on one bay.

        The single implementation shared by AFC_BAMBU_CAPSCAN (operator
        request) and the post-insert measurement (every insert, once the lane
        settles).

        The unit's own insert cycle runs unarmed and so does not save its
        measurement (see the save-gate note in cmd_AFC_BAMBU_CAPSCAN), which
        is why an insert is measured by running this.

        :param lane: the lane whose bay to measure
        :param slot: that lane's 0-based AMS slot
        :return Optional[str]: None on success, else why it did not start
        """
        if self._bridge is None:
            return "the bridge is not connected"
        if not (0 <= slot < len(self._slots)):
            return f"slot {slot} is not on this unit"
        if not (self._slots[slot] or {}).get("present"):
            return f"no spool present in slot {slot} (AMS bay {slot + 1})"
        # A measurement pulls the spool, so never run it while anything else
        # is moving filament.
        if afcBambuAMS._afc_motion_busy(self):
            return "a lane is loading/unloading"
        # ...and never into a measurement already under way: interrupting the
        # unit's own pull aborts it and leaves the assist grinding.
        try:
            if self._bridge.cap_calibrating(int(getattr(self, "dry_dev_addr", 0))):
                return "the unit is already measuring"
        except Exception:
            pass
        if 0 <= slot < len(self._auto_scanned):
            self._auto_scanned[slot] = False
        # A calibrate never arms the follower: arming sets s_follow, and the
        # firmware's phase machine then runs the load sequence instead -- the
        # AMS drives its assist against an unthreaded tray, slips, and never
        # reads the tag. An existing follower from a real load is left alone.
        if not afcBambuAMS._start_capscan(self, slot, cali=True):
            return "another spool operation owns the bus"
        # Mark this pending epoch as a calibrate's own. The auto-measure
        # watches for a cycle that ended without a measurement, which a
        # one-edged calibrate cycle resembles; without this mark it would
        # chain calibrates indefinitely.
        self._cali_epoch = getattr(self, "_cap_pending_t0", 0.0)
        # No _open_scan here, in either caller: a scan window resolves as tag
        # or defaults, and the defaults branch would overwrite the
        # measurement just taken. Identity is already on the lane.
        return None

    #: A capacity measure holds the unit ~40s and the firmware runs one bay at
    #: a time; a second measure asked inside this window is silently dropped
    #: by the unit, so a manual command refuses instead. Past the window a
    #: lingering marker is treated as done (a one-edged measure can leave it
    #: set), never a permanent block.
    _MEASURE_WINDOW_S = 50.0

    #: How long a bay stays on the pending list waiting for its measurement.
    #: Well past _MEASURE_WINDOW_S, which answers "may another measure start":
    #: a measurement can legitimately land after that window closes. This one
    #: answers "is this bay still expecting an answer".
    _PENDING_MAX_S = 180.0

    def _cap_open_pending(self, slot: int, asked: bool = False) -> None:
        """
        Mark a bay as expecting a measurement.

        Pending bays are kept in a map, each with its own t0, because several
        can be in flight at once: the unit measures on its own insert edges in
        its own order, and a bridge reboot makes every bay re-announce. A
        measurement is routed by the tray the unit names for it (see
        _cap_owner_of). `_cap_pending_slot` / `_cap_pending_t0` hold the most
        recent entry, for callers that ask "is a measure running".

        :param slot: 0-based AMS slot index now expecting a measurement
        :param asked: True when a command asked for this measurement (a
          capscan, a calibrate, a scan by hand) rather than an insert edge;
          it decides how loudly an unanswered wait is reported on expiry
        """
        try:
            now = self.afc.reactor.monotonic()
        except Exception:
            now = 0.0
        pend = getattr(self, "_cap_pending", None)
        if pend is None:
            pend = self._cap_pending = {}
        # Prune on the way in so no separate sweep is needed: a bay that never
        # got its answer must not claim a measurement minutes later.
        afcBambuAMS._cap_expire_pending(self, pend, now)
        pend[slot] = now
        self._cap_pending_slot: Optional[int] = slot
        self._cap_pending_t0 = now
        asked_set = getattr(self, "_cap_pending_asked", None)
        if asked_set is None:
            self._cap_pending_asked: set = set()
            asked_set = self._cap_pending_asked
        if asked:
            asked_set.add(slot)
        else:
            asked_set.discard(slot)
        # A new window needs a new answer.
        (getattr(self, "_cap_answered", None) or set()).discard(slot)

    def _cap_close_pending(self, slot: Optional[int]) -> None:
        """
        A bay's measurement arrived (or was given up on): stop expecting it.

        :param slot: the bay to drop, or None to drop the legacy marker only
        """
        pend = getattr(self, "_cap_pending", None) or {}
        if slot is not None:
            pend.pop(slot, None)
            (getattr(self, "_cap_pending_asked", None) or set()).discard(slot)
            (getattr(self, "_cap_answered", None) or set()).discard(slot)
        if slot is None or getattr(self, "_cap_pending_slot", None) == slot:
            # Fall back to whatever is still waiting, newest first, so the
            # "is a measure running" callers keep a truthful answer.
            if pend:
                nxt = max(pend, key=lambda s: pend[s])
                self._cap_pending_slot = nxt
                self._cap_pending_t0 = pend[nxt]
            else:
                self._cap_pending_slot = None

    def _cap_live_pending(self) -> dict:
        """
        The bays still expecting a measurement, pruned of the expired ones.

        An empty map means nothing is waiting. The single-slot marker is
        consulted only when the map has never been created (e.g. a test that
        sets only the marker), and it is cleared when pruning empties the map
        so an expired bay cannot come back.

        :return dict: {slot: t0}, newest last
        """
        pend = getattr(self, "_cap_pending", None)
        if pend is None:
            s = getattr(self, "_cap_pending_slot", None)
            return {} if s is None else {s: getattr(self, "_cap_pending_t0", 0.0)}
        try:
            now = self.afc.reactor.monotonic()
        except Exception:
            now = 0.0
        afcBambuAMS._cap_expire_pending(self, pend, now)
        if not pend and getattr(self, "_cap_pending_slot", None) is not None:
            self._cap_pending_slot = None
        return pend

    def _cap_expire_pending(self, pend: dict, now: float) -> None:
        """
        Drop the bays whose measurement is past _PENDING_MAX_S, saying so once.

        The only place a wait ends without an answer, so the only place that
        reports it (an AMS 1 narrates nothing for "no card, nothing measured").
        A bay answered by narration has already been closed. One answered only
        by the firmware's per-bay stamp (the slot record _sync_lanes adopts) is
        marked in _cap_answered and ends silently. Everything else went
        unanswered.

        Logged at INFO when a command asked for the measurement; at DEBUG when
        an insert opened the wait, since a reinserted spool the unit already
        knows ends its cycle without measuring by design.

        :param pend: the {slot: t0} map, pruned in place
        :param now: reactor time
        """
        asked = getattr(self, "_cap_pending_asked", None) or set()
        answered = getattr(self, "_cap_answered", None) or set()
        for s, t in list(pend.items()):
            if not (now and t and now - t > self._PENDING_MAX_S):
                continue
            del pend[s]
            loud = s in asked
            asked.discard(s)
            if s in answered:
                answered.discard(s)
                continue
            try:
                lane = self._lane_for_slot(s)
                where = (f"{lane.name} (slot {s})" if lane is not None
                         else f"slot {s}")
                msg = (f"AFC bambu {self.name}: {where} capscan ended without "
                       f"a measurement (no reading within "
                       f"{self._PENDING_MAX_S:.0f} s); it is no longer waiting "
                       f"for one")
                if loud:
                    self.logger.info(msg)
                else:
                    self.logger.debug(msg)
            except Exception:
                pass

    def _cap_owner_of(self, meas: dict) -> Optional[int]:
        """
        Which waiting bay a narrated measurement belongs to, or None.

        The narration batch that carries the reading also carries "odom save
        tray:N", the unit naming the bay it measured (see _CAP_SAVE_RE). That
        label decides: a named tray that is waiting gets it, and a named tray
        that is not waiting means the reading is not applied.

        Unlabelled, a single waiting bay takes it. Several waiting bays and no
        label is ambiguous, so it returns None.

        :param meas: the bridge's last_cap_measure record
        :return: the slot to adopt onto, or None to leave it alone
        """
        pend = self._cap_live_pending()
        if not pend:
            return None
        tray = meas.get("save_tray") if isinstance(meas, dict) else None
        if tray is not None:
            return tray if tray in pend else None
        if len(pend) == 1:
            return next(iter(pend))
        return None

    def _measure_ended_by_narration(self, t0: float) -> bool:
        """
        Has the unit narrated the end of the measure that started at ``t0``?

        Only the strict terminal counts (last_terminal, not 'odom calib
        success', which the unit emits while still moving). The terminals per
        model:

            0x1800  HT      [AMS_RFID] STEP4,Calibration rst:0   -- never STEP7
            0x0700  AMS 2   [AMS_RFID]STEP7:cali end
            0x0700  AMS 1   [AMS_DEV] STEP7:finish,cali tray

        _RFID_TERMINAL_RE searches the whole line, so the dialects' differing
        bracket tags and separators do not matter; the HT is carried by the
        Calibration rst: arm rather than a STEP7 one it never sends.

        The boxed units share 0x0700 and last_terminal is keyed by address, so
        one unit's ending could answer for another. A terminal therefore
        counts only if this unit holds the bus claim or nobody does.

        :param t0: monotonic time the measure was started
        :return bool: True if this unit narrated a terminal after ``t0``
        """
        if self._bridge is None or not t0:
            return False
        try:
            owner = self._bridge.bus_owner()
            if owner is not None and owner != self.name:
                return False
            term = self._bridge.last_terminal(getattr(self, "dry_dev_addr", 0))
        except Exception:
            return False                    # never block on a bridge hiccup
        return term is not None and term > t0

    def _measure_in_flight_slot(self) -> Optional[int]:
        """
        The slot this unit began measuring within the last _MEASURE_WINDOW_S,
        or None. Uses the host-side _cap_pending_slot marker rather than the
        firmware's cap_calibrating flag, which reads False in the gaps between
        edges.

        A scan that adopts no measurement (empty bay, non-Bambu chip) leaves
        the marker set, so the unit's narrated ending is checked first; the
        window is the backstop for a unit that narrates nothing.

        Does not clear _cap_pending_slot on the narration: the marker also
        gates adoption of a measurement that lands after the terminal. This
        only answers "may another measure start".

        :return: the busy slot index, or None if idle / the marker is stale
        """
        pend = getattr(self, "_cap_pending_slot", None)
        if pend is None:
            return None
        try:
            now = self.afc.reactor.monotonic()
        except Exception:
            return None
        t0 = getattr(self, "_cap_pending_t0", 0.0)
        if now - t0 > self._MEASURE_WINDOW_S:
            return None
        if self._measure_ended_by_narration(t0):
            return None
        return pend

    def _start_capscan(self, slot: int, cali: bool = False,
                       insert: bool = False) -> bool:
        """
        Kick the capacity choreography for one slot: one tag read, then the
        spool measurement, with the narrated percent attributed back to this
        slot (remain_pct + lane grams). The single entry point for the insert
        edge, AFC_BAMBU_SCAN LANE=, and AFC_BAMBU_CAPSCAN, so a scan always answers
        "how much filament is on this spool".

        :param slot: 0-based AMS slot index
        :param cali: True to run the full screen-calibrate choreography
        :param insert: True when triggered by a fresh physical insert
        :return bool: True if the command was issued
        """
        if self._bridge is None:
            return False
        # One spool operation on the bus at a time: two units scanning at once
        # can take Klipper down. The printer serialises these the same way.
        claim = getattr(self._bridge, "try_claim_bus", None)
        if callable(claim):
            try:
                now = self.afc.reactor.monotonic()
            except Exception:
                now = 0.0
            if not claim(self.name, now):
                who = self._bridge.bus_owner()
                # Log the defer at INFO once per scan sequence; retries
                # re-enter here every 5s for up to 2 minutes, so a retry (this
                # slot already has a claim-try counter) logs at DEBUG.
                retrying = getattr(self, "_scan_claim_tries", {}).get(slot, 0)
                msg = (f"AFC bambu {self.name}: deferring the scan of slot "
                       f"{slot} -- {who} has a spool operation running on this "
                       f"bus. It will be retried; nothing is lost.")
                if retrying:
                    self.logger.debug(msg)
                else:
                    self.logger.info(msg)
                return False
        afcBambuAMS._cap_open_pending(self, slot, asked=not insert)
        if self._is_ht():
            # The HT's scan is firmware-armed at 0x1800 and opens its own
            # measurement window; the plain scan command drives it and the
            # pending marker above attributes the narrated percent. cali is
            # ignored: the HT measures on every scan.
            return self.scan(slot)
        if cali:
            # The printer's screen-calibrate sequence: forces a full two-edge
            # measure and an odom save even on a tray whose R is already
            # stored, which a plain capscan does not. See bb_do_calibrate.
            #
            # "auto":1 for an insert: same script, but the capacity byte is
            # not forced, so measure_on_insert still decides whether a
            # measurement happens. An operator command keeps the override.
            self._bridge.send({"cmd": "cali", "auto": 1 if insert else 0,
                               "unit": self.ams_index, "slot": slot})
            return True
        # Insert vs re-read: the insert sequence opens with a clear and sends
        # one select, the re-read opens with a select and sends a pair, and
        # only the host knows which event this is. Commands use the re-read
        # (insert=False); the insert edge passes insert=True. See
        # bb_do_capscan_ins.
        self._bridge.send({"cmd": "capscan", "trig": 1,
                           "ins": 1 if insert else 0,
                           "unit": self.ams_index, "slot": slot})
        return True


    def cmd_AFC_BAMBU_REID(self, gcmd: Any) -> None:
        """
        Send the printer's "re-identify" to one bay, on its own.

        AFC_BAMBU_REID LANE=<lane> [UNIT=<unit>]

        Sends one frame only: the type-07 select with payload byte[7]=0x00
        (the printer menu's re-identify), so whatever the unit narrates next
        is its answer to that frame.

        On a boxed AMS an insert produces "first detected" and stops there;
        this frame produces the second detection, and the unit needs two tag
        passes to derive a circumference and hence a spool measurement.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_REID LANE=<lane> [UNIT=<unit>]`

        Example
        -------
        ```
        AFC_BAMBU_REID LANE=lane1
        ```
        """
        lane_name = gcmd.get("LANE")
        lane, slot = self._gcmd_lane_slot("AFC_BAMBU_REID", gcmd, lane_name)
        self._bridge.send({"cmd": "reid", "unit": self.ams_index, "slot": slot})
        gcmd.respond_info(
            f"AFC_BAMBU_REID: sent re-identify to {self.name} bay {slot} "
            f"({lane_name}). Watch for 'second detected' and a measurement.")

    def cmd_AFC_BAMBU_CAPSCAN(self, gcmd: Any) -> None:
        """
        Run the printer's capacity-measuring re-scan on one bay.

        The only measure command; it takes no options.

        AFC_BAMBU_CAPSCAN LANE=<lane> [UNIT=<unit>]

        The insert choreography a real printer performs, which plain
        AFC_BAMBU_SCAN does not: re-identify trigger, statu-01 probe, then the
        05/80 capacity ENABLE that arms ams_state 3 -- the state in which the
        AMS measures the spool's radius during its preload pull and persists
        remain% to the tag record ("odom calib success, dis:0.776"). Read the
        result a minute later: the slot's remain_pct in status.

        Works on both unit classes; the HT gets the same enable at unit byte
        0x80. Refused while printing and while any lane on the unit is
        tool-loaded: the probe bounces the AMS's link mode (mode 2 -> 0 -> 2),
        which must not happen to a unit feeding an extruder.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_CAPSCAN LANE=<lane> [UNIT=<unit>]`

        Example
        -------
        ```
        AFC_BAMBU_CAPSCAN LANE=lane1
        ```
        """
        if self._bridge is None:
            error_str = "AFC_BAMBU_CAPSCAN: bridge not connected"
            raise gcmd.error(error_str)
        # HT units are allowed: they take the same enable at unit byte 0x80
        # (068005008000, streamed through the scan window for both classes;
        # an HT without it reports "tray capacity no en").
        try:
            if self.afc.function.in_print():
                error_str = "AFC_BAMBU_CAPSCAN: refused while printing"
                raise gcmd.error(error_str)
        except AttributeError:
            pass
        if _unit_tool_loaded(self):
            error_str = ("AFC_BAMBU_CAPSCAN: refused -- a lane on this unit is tool-loaded "
                "and the probe bounces the unit's link mode")
            raise gcmd.error(error_str)
        lane_name = gcmd.get("LANE")
        lane, slot = self._gcmd_lane_slot("AFC_BAMBU_CAPSCAN", gcmd, lane_name)
        busy = self._measure_in_flight_slot()
        if busy is not None:
            raise gcmd.error(
                f"AFC_BAMBU_CAPSCAN: {self.name} is already measuring slot "
                f"{busy} (AMS bay {busy + 1}) -- one bay at a time, so this "
                f"would be dropped. Wait ~40s, then retry.")
        # No card-cache clear is sent first: the unit measures without one,
        # and a clear does not reach a boxed unit's card record anyway.
        # Every explicit measure goes through _run_calibrate, which carries
        # the guards. The plain cap_send_enable branch an AMS 2 would
        # otherwise take never terminates its enable ladder.
        why = afcBambuAMS._run_calibrate(self, lane, slot)
        if why is not None:
            error_str = f"AFC_BAMBU_CAPSCAN: not started -- {why}"
            raise gcmd.error(error_str)
        gcmd.respond_info(
            f"AFC_BAMBU_CAPSCAN: capacity re-scan armed for {lane_name} "
            f"(slot {slot}) on {self.name} -- the AMS pulls the spool, "
            f"re-reads the tag and measures remain%; check the slot's "
            f"remain_pct in about a minute")
        afcBambuAMS._warn_bound_without_spoolman(self, lane)

    def _warn_bound_without_spoolman(self, lane: Any) -> None:
        """
        Say, when a measure is asked for, that it will not reach the Spoolman
        spool a lane is still bound to while the Spoolman module is off.

        The measurement lands on the lane either way. But AFC core reloads a
        bound lane's weight from Spoolman at restart (AFC_prep: set_spoolID ->
        remaining_weight), and with [AFC_BambuAMS_rfid] off nothing writes the
        measurement to that spool -- so after a restart the lane shows the
        spool's figure, not the measurement: the bay's stamp is recorded at
        the restart and never re-applied (see _baseline_meas_stamp). Warned
        once per command, never per reading.

        Only when AFC itself has Spoolman: without it a spool_id reloads
        nothing, and the second half of the sentence would be false.

        :param lane: the lane being measured
        """
        try:
            sid = getattr(lane, "spool_id", None)
            if sid in (None, "", 0):
                return
            if getattr(getattr(self, "afc", None), "spoolman", None) is None:
                return
            if self._spool is not None:
                return
            name = getattr(lane, "name", "?")
            self.logger.warning(
                f"AFC bambu {self.name}: {name} is bound to Spoolman spool "
                f"{sid}, and with the Spoolman module ([AFC_BambuAMS_rfid]) "
                f"off this measurement stays on the lane and is not written "
                f"to that spool. AFC reloads a bound lane's weight from "
                f"Spoolman at restart, so {name} and spool {sid} can "
                f"disagree.")
        except Exception:
            pass

    def _supply_reading(self) -> Tuple[Optional[float], bool]:
        """
        This unit's own 24V jack as IT last reported it.

        Only available while the unit is narrating chamber telemetry; there
        is no idle source for it (docs/ams2_pro_protocol.md). "No reading" is
        the normal state of an idle unit and means unknown.

        :return tuple: (volts or None, whether it is recent enough to act on)
        """
        rec = self._chamber_record() if self._bridge is not None else None
        if not isinstance(rec, dict) or rec.get("ad_v") is None:
            return None, False
        mono = getattr(self.reactor, "monotonic", None)
        nowm = mono() if callable(mono) else 0.0
        fresh = (not nowm) or (nowm - rec.get("seen", 0.0) < EXT_SUPPLY_MAX_AGE)
        return float(rec["ad_v"]), fresh

    def _heating_now(self) -> bool:
        """
        Whether this unit is putting power into its heater right now.

        `_drying` is adopted from live telemetry only inside get_status, so a
        cycle started elsewhere, or already running at Klipper startup, reads
        False until something polls that unit. The unit's own telemetry is
        therefore consulted directly as well.

        :return bool: True while a cycle is running on this unit.
        """
        if getattr(self, "_drying", False):
            return True
        rec = self._chamber_record() if self._bridge is not None else None
        if not isinstance(rec, dict):
            return False
        # Past the last start/stop plus the wind-down grace, as get_status's
        # fresh_for_this_cycle reads it: a stopping unit emits another line
        # or two, which must not read as a running cycle.
        return (self._record_fresh(rec)
                and rec.get("seen", 0.0) > getattr(self, "_dry_adopt_after",
                                                   0.0))

    def _bus_supply_conflict(self) -> str:
        """
        Why this unit's heater must not start, in the operator's terms.

        Only one AMS 2 Pro may dry off the bus wire. A second one collapses the
        supply about 3s after its heater engages and resets every unit on the
        wire; nothing on the bus refuses it because it is a power fault, not a
        protocol one.

        The rule: a start is refused only while another AMS 2 on this bus is
        drying and not reporting ~24V at its own jack (i.e. running off the
        bus, or not yet reported).

        Stopping the bus-powered unit, or plugging an adapter into it
        mid-cycle, lifts the refusal. A unit on its own adapter never blocks
        anything. The unit being started is not consulted, since an idle
        unit's jack cannot be read; only units already heating constrain it.
        This holds for any number of units on the bus without counting them:
        whoever has spent the single bus allowance is heating and therefore
        narrating.

        HT units are skipped: an AMS HT will not heat without its own mains
        cord, so it cannot be the unit drawing off the bus.

        :return str: the refusal, or "" to allow the start.
        """
        if not self.dry_bus_interlock or not self.has_heater or self._is_ht():
            return ""
        for _n, u in self.printer.lookup_objects("AFC_BambuAMS"):
            if u is self or not getattr(u, "has_heater", False):
                continue
            # An unclaimed pool unit is not on the wire and owns no chamber
            # (see _chamber_record); judging one would read another unit's
            # telemetry as its own.
            if getattr(u, "pool", False):
                continue
            try:
                if u._is_ht() or u._bridge is not self._bridge:
                    continue        # another bridge is another bus
                if not u._heating_now():
                    continue
            except Exception:
                continue
            volts, fresh = u._supply_reading()
            if volts is not None and fresh and volts >= EXT_SUPPLY_MIN_V:
                continue            # on its own adapter: not our problem
            if volts is None or not fresh:
                # Drying, but has not reported how it is powered: refuse
                # rather than guess. The unit narrates every ~10s.
                return (f"{u.name} is drying and has not reported its power "
                        f"source yet, so there is no way to tell whether it is "
                        f"running off the bus. Wait ~10s for it to report and "
                        f"start again, or add FORCE=1 if you know it is on its "
                        f"own 24V adapter.")
            return (f"{u.name} is already drying off the BUS ({volts:.1f}V at "
                    f"its 24V jack, so nothing is plugged into it). Two units "
                    f"heating off the bus wire collapse the supply and reset "
                    f"every AMS on it. Either stop {u.name}, or plug a 24V "
                    f"adapter into it -- once it reports ~24V this start goes "
                    f"through with it still running. FORCE=1 overrides.")
        return ""

    def cmd_AFC_BAMBU_HEATER_START(self, gcmd: Any) -> None:
        """
        Start AMS drying (AMS2 Pro or AMS HT heater).

        AFC_BAMBU_HEATER_START UNIT=<unit> [TEMP=55] [TIME=480] [ROTATE=0]

        TEMP in C (clamped to the unit's dry_max_temp), TIME in minutes,
        ROTATE=1 spins the spools while drying. Sends the printer's drying
        command on the bus.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_HEATER_START UNIT=<unit> TEMP=<n> TIME=<n> ROTATE=<0 or 1> FORCE=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_HEATER_START UNIT=BambuAMS_1 TEMP=55 TIME=480 ROTATE=1 FORCE=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_HEATER_START: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if not self.has_heater:
            gcmd.respond_info(
                f"AFC_BAMBU_HEATER_START: {self.name} has no drying heater "
                f"(heater: false). Only the AMS2 Pro can dry -- ignoring.")
            return
        # Checked first, before the frame goes out: a supply collapse happens
        # ~3s after the second heater engages, before any reading could show
        # it, so it can only be prevented here.
        conflict = (self._bus_supply_conflict()
                    if gcmd.get_int("FORCE", 0) != 1 else "")
        if conflict:
            # Stored where the HT's refusals go, so the panel shows it as the
            # reason the dry did not start.
            self._dry_host_refusal = conflict
            gcmd.respond_info(f"AFC_BAMBU_HEATER_START: {conflict}")
            return

        # Clear the previous attempt's refusal; nothing else clears it on an
        # HT, so it would otherwise be reported over a dry that started fine.
        try:
            self._bridge.clear_dry_error(self.ams_index)
        except Exception:
            pass
        # Clamp (not error) to this unit's drying ceiling (dry_max_temp: 65 for
        # AMS2 Pro, 85 for AMS HT); more would be rejected by the AMS or risk
        # the spools.
        temp = gcmd.get_int("TEMP", 55, minval=0)
        if temp > self.dry_max_temp:
            gcmd.respond_info(
                f"AFC_BAMBU_HEATER_START: TEMP {temp}C exceeds {self.name}'s drying "
                f"max {self.dry_max_temp}C -- clamping to {self.dry_max_temp}C.")
            temp = self.dry_max_temp
        tmin = gcmd.get_int("TIME", 480, minval=0, maxval=65535)
        rot = gcmd.get_int("ROTATE", 0, minval=0, maxval=1)
        # The unit will not turn a spool that has filament in its feeder, so
        # drop the flag and tell the operator rather than send a flag it will
        # ignore.
        if rot:
            committed = self._committed_lanes()
            if committed:
                rot = 0
                names = ", ".join(getattr(ln, "name", "?") for ln in committed)
                gcmd.respond_info(
                    f"AFC_BAMBU_HEATER_START: ROTATE disabled for {self.name} -- "
                    f"{names} still has filament in the feeder, and the unit "
                    f"will not turn a spool that does. Unload the bays and "
                    f"leave the spools free to spin to dry with rotation. "
                    f"Drying without it.")
        # Quiet the follower before the self-check unless a lane on this unit
        # is printing (dry-while-printing needs it). loaded_to_hub is a staging
        # state and does not count.
        #
        # An HT with a tool-loaded lane is not refused here: HT firmware
        # 05.00.22.19 and later heats while printing. Older HT firmware answers
        # "[AMS_CHMB]err, filament hub load!" instead, which the last_dry_error
        # watch reports as a refusal and clears _drying (see _drop_silent_dry).
        self._dry_host_refusal = ""
        self._drying = True
        # A new cycle: telemetry from before this point belongs to the old one,
        # and no reading for this cycle has arrived yet (the panel's
        # "Starting -- waiting for the unit to report").
        self._dry_adopt_after = _mono(self)
        self._dry_seen_live = False
        self._dryrem_at_stop = None   # a new cycle retires the stop stamp
        if self._following_lane is not None and self._tool_loaded_lane() is None:
            try:
                self.set_feed_assist(self._following_lane, False)
            except Exception:
                pass
            self._following_lane = None
        # AMSID/ADDR overrides are diagnostic, for confirming the drying id
        # byte's mapping (normally = chain index) per chain position. A
        # non-matching override would target another unit's heater, so it is
        # refused unless FORCE=1.
        amsid = _gcmd_int(gcmd, "AMSID", self.dry_ams_id, 0, 0xFFFF)
        addr = _gcmd_int(gcmd, "ADDR", self.dry_dev_addr, 0, 0xFFFF)
        if ((amsid != self.dry_ams_id or addr != self.dry_dev_addr)
                and gcmd.get_int("FORCE", 0) != 1):
            error_str = (f"AFC_BAMBU_HEATER_START: AMSID/ADDR override ({amsid}/0x{addr:04X}) "
                f"does not match {self.name}'s own addressing "
                f"({self.dry_ams_id}/0x{self.dry_dev_addr:04X}), another "
                f"unit's heater would be targeted. Drop the override, or add "
                f"FORCE=1 for deliberate diagnostics.")
            raise gcmd.error(error_str)
        # Only command a unit that has actually answered (a real printer sends
        # WORK frames only to ids that replied), or _drying latches for a cycle
        # that is not running. Fail-open on "cannot tell": refuse only when
        # telemetry exists and says the unit is absent.
        online = getattr(self, "_unit_online", None)
        getst = getattr(self._bridge, "latest_status", None)
        latest = getst() if callable(getst) else None
        if latest and callable(online) and not online(latest):
            error_str = (f"AFC_BAMBU_HEATER_START: {self.name} is not online -- nothing on "
                f"the bus is answering for it, so the heater command would go "
                f"nowhere. Check the unit is powered and chained, then retry.")
            raise gcmd.error(error_str)
        self._bridge.send({"cmd": "dry", "unit": self.ams_index, "on": 1,
                           "temp": temp, "time": tmin, "rotate": rot,
                           "addr": addr, "amsid": amsid})
        # Record when this cycle was commanded and for how long, so a display
        # can show time remaining; the AMS does not report the duration, so it
        # exists only in the command sent here.
        self._dry_started_at = _mono(self)
        self._dry_minutes = int(tmin)
        self._dry_rotate = 1 if rot else 0
        # The commanded setpoint, kept because a boxed dry deafens the bridge's
        # receiver while the heater draws, so the telemetry that carries the
        # target may never arrive.
        self._dry_temp = int(temp)
        gcmd.respond_info(
            f"AFC_BAMBU_HEATER_START: {self.name} drying at {temp}C for {tmin}min"
            f"{' with spool rotation' if rot else ''}"
            f" (addr 0x{addr:04X}, id {amsid}).")

    def cmd_AFC_BAMBU_HEATER_STOP(self, gcmd: Any) -> None:
        """
        Stop AMS drying. AFC_BAMBU_HEATER_STOP UNIT=<unit>

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_HEATER_STOP UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_HEATER_STOP UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_HEATER_STOP: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if not self.has_heater:
            gcmd.respond_info(
                f"AFC_BAMBU_HEATER_STOP: {self.name} has no drying heater "
                f"(heater: false) -- nothing to stop.")
            return
        # Must carry the same addr/amsid as start, or an HT (0x1800) never hears
        # the stop and keeps drying.
        # A stop also retires any host-side refusal note: the operator has
        # acted, and the next start speaks for itself.
        self._dry_host_refusal = ""
        self._bridge.send({"cmd": "dry", "unit": self.ams_index, "on": 0,
                           "addr": self.dry_dev_addr, "amsid": self.dry_ams_id})
        self._drying = False
        self._dry_started_at = None
        self._dry_minutes = 0
        self._dry_rotate = 0
        # Stamp the stop plus a grace period. get_status adopts a cycle from
        # live chamber telemetry (so the panel can follow a dry the host did
        # not start); the stamp keeps telemetry from before this stop from
        # re-setting _drying, and the grace covers the line or two the AMS
        # emits while winding down. A unit still reporting past the grace is
        # still drying, and adoption re-arms.
        self._dry_adopt_after = _mono(self) + DRY_STOP_GRACE
        self._dry_seen_live = False
        # Stamp the firmware countdown being stopped. dryrem freezes whenever
        # 0x3C replies stop decoding (a boxed dry deafens the receiver, and
        # another unit still drying keeps it deaf past this stop); get_status
        # lets dryrem re-arm only once it differs from this stamp, i.e. is
        # actually ticking.
        self._dryrem_at_stop = None
        try:
            u = afcBambuAMS._unit_entry(self, self._bridge.latest_status() or {})
            if u is not None and u.get("dryrem") is not None:
                self._dryrem_at_stop = int(u["dryrem"])
        except Exception:
            pass
        gcmd.respond_info(f"AFC_BAMBU_HEATER_STOP: {self.name} drying stopped.")

    def cmd_AFC_BAMBU_SNIFF(self, gcmd: Any) -> None:
        """
        Put this unit's bridge into passive listen-only mode, or take it back
        out. AFC_BAMBU_SNIFF UNIT=<unit> ON=<0 or 1>

        In sniff mode the board transmits nothing: it stops being the bus
        master and only logs what it hears, so a bridge wired alongside a
        Bambu printer records that printer's conversation with its AMS.

        While sniffing, every load, unload, tag scan and heater command aimed
        at units on this bridge silently goes nowhere, so ON=1 is refused
        mid-print.

        Frames land in AFC.log as ``SNIFF <hex>`` lines (file only -- a busy
        bus is hundreds of frames a second). The mode lives in RAM: a reboot,
        or any reconnect, comes back driving.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_SNIFF UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_SNIFF UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_SNIFF: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        on = gcmd.get_int("ON", 1, minval=0, maxval=1)
        if on:
            # Refuse rather than warn: a bridge that silently stops driving
            # mid-print costs the print.
            try:
                printing = bool(self.afc.function.in_print())
            except Exception:
                printing = False
            if printing:
                raise gcmd.error(
                    f"AFC_BAMBU_SNIFF: refusing to sniff on {self.name} while "
                    f"a print is running -- sniff mode transmits nothing, so "
                    f"every load and unload on this bridge would go nowhere.")
        self._bridge.send({"cmd": "sniff", "on": on})
        if on:
            gcmd.respond_info(
                f"AFC_BAMBU_SNIFF: {self.name} bridge going listen-only. It "
                f"will NOT drive its AMS units until ON=0 (or a reboot). "
                f"Captured frames go to AFC.log as SNIFF lines.")
        else:
            gcmd.respond_info(
                f"AFC_BAMBU_SNIFF: {self.name} bridge back to driving the bus.")

    def cmd_AFC_BAMBU_UIDS(self, gcmd: Any) -> None:
        """
        Print the AMS UIDs currently on the bus, read straight off the wire.

        Requests the chain map from the bridge, then reports each chain index's
        UID plus what that unit holds (to tell them apart), so the UIDs can be
        copied into each section's ``unit_uid`` for stable mapping.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_UIDS UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_UIDS UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            error_str = "AFC_BAMBU_UIDS: bridge not connected"
            raise gcmd.error(error_str)
        self._bridge.send({"cmd": "chain"})        # refresh the enrollment map
        try:                                        # report after the reply lands
            self.afc.reactor.register_callback(
                self._report_uids, self.afc.reactor.monotonic() + 0.5)
        except Exception:
            self._report_uids(0)                    # no reactor (tests)

    def _report_uids(self, eventtime: float) -> None:
        """
        Emit the cached chain UIDs and per-index occupancy to the console.

        :param eventtime: reactor time supplied by the callback; unused
        """
        uids = self._bridge.chain_uids() if self._bridge else []
        if not uids:
            self.gcode.respond_info(
                "AFC_BAMBU_UIDS: no AMS UIDs read yet -- run it again in a moment.")
            return
        latest = (self._bridge.latest_status() if self._bridge else None) or {}
        slots = latest.get("slots", []) or []
        diag = getattr(self._bridge, "chain_diag", None) if self._bridge else None
        htmask, fw, sel = diag() if callable(diag) else (0, "", (-1, 0, 0))
        # Read straight off the bridge like the other chain diagnostics below.
        # None = this firmware does not report it (not "nothing measures").
        capmask = (getattr(self._bridge, "_chain_capmask", None)
                   if self._bridge else None)
        lines = [f"Bambu AMS bus (firmware {fw or 'pre-0.3.0'}) -- copy each "
                 f"UID into that unit's `unit_uid`:"]
        for i, u in enumerate(uids):
            occ = [f"slot{s.get('i')}={s.get('material') or 'present'}"
                   for s in slots
                   if s.get("unit") == i and s.get("present")]
            hint = ("  <- " + ", ".join(occ)) if occ else "  <- (empty)"
            ht = "  [HT-flagged]" if htmask & (1 << i) else ""
            # Announce-reply tag byte, shown for observation only; nothing acts
            # on it. It may indicate unit class, but is not a stable per-unit
            # id (the same unit can report different values).
            tags = (getattr(self._bridge, "_chain_tags", "") or "").split(",")
            tg = f"  tag={tags[i]}" if i < len(tags) and tags[i] else ""
            lines.append(f"  chain index {i}: {u or '(none)'}{ht}{tg}{hint}")
        # Capacity diagnostics: capn = lifetime capacity-stream frames, capdiag
        # = (last op-04 poll unit byte)<<8 | boxed-burst-ran. If the poll byte
        # is 00 while an HT is being scanned, the measure poll is leaking to the
        # boxed AMS (ams0) instead of the HT (0x80).
        cd = getattr(self._bridge, "_chain_capdiag", 0) if self._bridge else 0
        cn = getattr(self._bridge, "_chain_capn", 0) if self._bridge else 0
        # Why the last capacity window did or did not open (which of
        # cap_open's guards refused it). None = firmware older than AFC-2.68.
        cw = getattr(self._bridge, "_chain_capwhy", None) if self._bridge else None
        _CAPWHY = {0: "cap_open not called since boot",
                   1: "a window opened",
                   2: "REFUSED -- another window was already live",
                   3: "REFUSED -- this bay ran a window inside the last 45s",
                   4: "REFUSED -- measure is off for this unit and the "
                      "scan did not force it",
                   5: "DEFERRED -- arrived while another window was running, "
                      "held and run when that one closed"}
        lines.append(f"  capacity: capn={cn} poll_ub=0x{(cd>>8)&0xFF:02X} "
                     f"boxed_burst={cd & 1}")
        if cw is not None:
            _wu = getattr(self._bridge, "_chain_capwhy_unit", None)
            _ws = getattr(self._bridge, "_chain_capwhy_slot", None)
            # Name the bay, so "a window opened" can be told apart from an
            # earlier window on another bay.
            _who = (f" [unit {_wu} slot {_ws}]"
                    if isinstance(_wu, int) and _wu < 8 else "")
            lines.append(f"  last cap_open{_who}: "
                         f"{_CAPWHY.get(cw, f'unknown ({cw})')}")
        # Select-probe verdict: which id the HT acked its type-07 select at
        # (-1 = still probing / never acked).
        selid, selsent, selack = sel
        if selsent:
            lines.append(
                f"HT select probe: sent={selsent} acked={selack} "
                f"locked_id={'none yet' if selid < 0 else hex(selid)}")
        # The MC addressing the firmware actually holds per unit. The
        # narration log drain goes per-unit only when this is set; otherwise
        # it falls back to 0x0700, which never asks an AMS HT at 0x1800.
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
        # Per-unit alignment: which chain index each configured unit resolved
        # to, and whether an HT unit's flag landed on its index (an HT whose
        # flag is missing gets no insert-scan).
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
                # measure_on_insert as the firmware holds it, beside what this
                # unit is configured for. They differ only if a push was lost,
                # which is not visible elsewhere (the capen ack names the unit,
                # not the value).
                want = bool(getattr(unit, "measure_on_insert", False))
                if capmask is None:
                    meas = (f'  measure={("on" if want else "off")} (firmware pre-dates the '
                        f'readback)')
                elif not isinstance(idx, int):
                    meas = f'  measure={("on" if want else "off")}'
                else:
                    got = bool(capmask & (1 << idx))
                    meas = f'  measure={("on" if got else "off")}'
                    if got != want:
                        meas += (f'  [MISMATCH -- config says {("on" if want else "off")}; re-push '
                            f'with a Klipper restart]')
                lines.append(f"  {getattr(unit, 'name', '?')} -> chain index "
                             f"{idx}{mark}{meas}")
        except Exception:
            pass
        self.gcode.respond_info("\n".join(lines))

    def _recover_to_bay(self, lane: Any) -> None:
        """
        Relink the AMS, stop it, reel this lane's filament back to the bay, and
        clear its loaded/error state. Shared by eject and AFC_BAMBU_RECOVER so a
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

        The bridge is cached in a module-global keyed by serial port, which
        survives a FIRMWARE_RESTART (Python isn't re-imported). Without this,
        the new unit instances would reuse the old bridge whose reader thread
        and serial port are dead. Stopping it here (idempotent across the
        daisy-chained units) forces _handle_ready to rebuild a fresh connection
        and re-prime status.
        """
        bridge = _bridge_mod._BRIDGES.pop(self.serial_port, None)
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
        self._scan_notag = [False] * self.SLOTS_PER_UNIT
        # Bays handed back to the insert path by _restore_untagged_defaults,
        # so that happens once per bay per connection. See there.
        self._untagged_rearmed = [False] * self.SLOTS_PER_UNIT
        # Inserts whose scan was held back because a lane on this unit was
        # threaded to the toolhead. Retried once the unit is free; see
        # _maybe_auto_scan.
        self._scan_defer = [False] * self.SLOTS_PER_UNIT
        # When the current dry cycle was commanded, for how long, and whether
        # it spins the spool. Initialised here, not only in
        # cmd_AFC_BAMBU_HEATER_START, because get_status reads them from the
        # moment the object exists.
        self._dry_started_at = None
        self._dry_minutes = 0
        self._dry_rotate = 0
        self._dry_refusal_logged = False
        self._scan_t0 = [None] * self.SLOTS_PER_UNIT
        self._scan_motion_t0 = [None] * self.SLOTS_PER_UNIT
        # One Spoolman lookup per insert, like OpenAMS's _rfid_scanned latch.
        # _surface_slot_info runs on every status frame, and blocking HTTP
        # calls from it risk a "Timer too close" shutdown. Only the removal
        # edge (and a reconnect) discard a slot from the latch.
        self._spoolman_latched = set()
        self._scan_primed = False           # re-prime the baseline after reconnect
        self._prime_waiting = False         # and its wait on presence with it
        self._stamp_looked = set()          # and the measurement baseline with it
        # A reconnect is a fresh look at the hardware, so no bay counts as
        # scanned; the boot hold re-engages until a bay is read again.
        sb = getattr(self, "_scanned_bays", None)
        if isinstance(sb, set):
            sb.clear()
        # ...and the one-shot in _restore_untagged_defaults, so a still
        # unidentified bay gets another attempt on the next connection.
        ra = getattr(self, "_untagged_rearmed", None)
        if ra is not None:
            for i in range(len(ra)):
                ra[i] = False
        # The identity and lookup memos reset with the connection. A spool
        # swapped while the bridge was down produces no removal edge (the
        # baseline re-primes around it), so anything keyed to a slot -- the
        # bound-tag memo, a measured remain% -- would otherwise attribute the
        # old spool's facts to the new one.
        latch = getattr(self, "_spoolman_latched", None)
        if isinstance(latch, set):
            latch.clear()
        for name in ("_bound_uid", "_binding_check", "_measured_remain",
                     "_meas_seen",
                     "_pending_summary", "_scan_seq0", "_scan_claim_tries"):
            memo = getattr(self, name, None)
            if isinstance(memo, dict):
                memo.clear()
        # Which of those _meas_seen entries were baselined goes with them.
        bl = getattr(self, "_meas_baselined", None)
        if isinstance(bl, set):
            bl.clear()
        miss = getattr(self, "_spoolman_no_match", None)
        if isinstance(miss, set):
            miss.clear()
        # And the presence history, and the lookup state the latch explains.
        afcBambuAMS._reset_connection_state(self)
        afcBambuAMS._reset_lookup_state(self)

    def _reset_connection_state(self) -> None:
        """
        Forget the presence history of the last bridge connection.

        Each is rebound to a new object rather than cleared in place: this
        also runs from _on_bridge_reconnect, on the bridge reader thread,
        while the reactor may be iterating the old one.
        """
        self._present_seen = set()
        self._removed_bays = set()
        self._defaults_due = {}
        self._meas_departed = {}
        # Nothing this bridge says about the bays counts until it has asked
        # the unit (_presence_known).
        self._presence_ok = False
        self._presence_wait_said = False

    def _reset_lookup_state(self) -> None:
        """
        Forget what goes with _spoolman_latched and _scanned_bays: the lanes
        a removal edge cleared, and the answers of the lookups the latch
        records. Reset where those are (_handle_disconnect, claim), never on
        a bridge reconnect alone.
        """
        self._cleared_bays = set()
        self._lookup_retry = {}
        self._lookup_refused = {}
        # Like _cleared_bays: defaults put on a lane nothing knew about still
        # stand in for a read after a reconnect (_boot_hold).
        self._defaulted_bays = set()

    def _prime_scan_baseline(self, _et: Optional[float] = None) -> None:
        """
        Arm insert-edge scanning once the boot/claim baseline has settled.

        The frames before this fire ran with _scan_primed False, so every bay
        that came up already present was recorded as the baseline (_prev_present)
        rather than treated as a fresh 0->1 insert -- which is what stops a
        reboot from re-reading and re-measuring what AFC restored from saved
        vars, and stops every unit from piling a measure onto the shared bus at
        once. From here, only genuine post-startup inserts scan. Also reconciles
        empty bays and fills untagged defaults now that presence has reported.

        :param _et: reactor time of this firing (unused)
        """
        # Wait until the bridge has actually polled the unit: before that
        # every bay reads empty, and the reconcile below would clear every
        # lane with data. _on_status runs this again on the first frame that
        # says it has.
        #
        # Also wait for owed re-reads. After a Pico boot every occupied boxed
        # bay is an insert edge to the firmware, which owes it a re-read (rrq)
        # and publishes a blank record until the re-read lands; priming on
        # that frame would give the lane defaults and then hold the real tag
        # off it. Bounded, so a re-read that never settles cannot keep the
        # unit unprimed for the session.
        if (not getattr(self, "_presence_ok", True)
                or afcBambuAMS._rereads_owed(self)):
            if not getattr(self, "_prime_waiting", False):
                self._prime_waiting = True
                self.logger.debug(
                    f"AFC bambu {self.name}: scan priming waits until the "
                    f"bridge has polled this unit and re-read its bays")
            return
        self._prime_waiting = False
        self._reread_wait_t0 = None
        self._scan_primed = True
        # A pool unit claimed live never ran AFC's PREP lane-test for its
        # lanes (PREP ran at boot, while it was an inert pool slot), so
        # _prep_seen is still False and _sync_lanes surfaces nothing. Run that
        # test's data half now that presence has reported, then open the
        # status path; otherwise bays already full at claim never reach their
        # lanes, since a scan only fires on an insert edge.
        claimed_live = not getattr(self, "_prep_seen", False)
        if claimed_live:
            self._prep_claimed_lanes()
            self._prep_seen = True
            self._print_claimed_prep()
        self._reconcile_empty_bays()
        self._restore_untagged_defaults(claimed_live=claimed_live)

    # The fields AFC_prep copies out of VarFile.unit onto a lane, as
    # (saved key, lane attribute), so the claim path restores exactly what
    # boot does and nothing else.
    _CLAIM_RESTORE_FIELDS = (
        ("material", "material"),
        ("color", "color"),
        ("weight", "weight"),
        ("density", "filament_density"),
        ("diameter", "filament_diameter"),
        ("empty_spool_weight", "empty_spool_weight"),
        ("bed_temp", "bed_temp"),
        ("extruder_temp", "extruder_temp"),
    )

    def _restore_claimed_lane_vars(self, lane: Any) -> bool:
        """
        Give a lane on a live-claimed unit the saved state PREP never gave it.

        Mirrors AFC_prep's restore for one lane: a Spoolman link is re-fetched
        (which brings material, colour, weight and temps with it), and without
        Spoolman the stored profile is copied field by field.

        The link is assigned before the fetch is asked for, as PREP does:
        set_spoolID's fetch is async, so a caller checking lane.spool_id right
        after must already see it rather than treat the lane as blank.

        :param lane: The AFC lane object
        :return bool: True if anything was restored
        """
        saved = {}
        persisted = getattr(self, "_persisted_lane", None)
        if persisted is not None:
            saved = persisted(getattr(lane, "name", "")) or {}
        if not saved:
            return False
        afc = getattr(self, "afc", None)
        spool_id = saved.get("spool_id")
        if spool_id not in (None, "", 0):
            try:
                lane.spool_id = spool_id
            except Exception:
                return False
            if getattr(afc, "spoolman", None) is not None:
                try:
                    afc.spool.set_spoolID(lane, spool_id, save_vars=False)
                except Exception as e:
                    self.logger.debug(
                        f"AFC bambu {self.name}: could not re-fetch spool "
                        f"{spool_id} for {getattr(lane, 'name', lane)}: {e}")
                return True
        restored = spool_id not in (None, "", 0)
        for key, attr in self._CLAIM_RESTORE_FIELDS:
            val = saved.get(key)
            if val in (None, ""):
                continue
            try:
                setattr(lane, attr, val)
                restored = True
            except Exception:
                pass
        return restored

    def _prep_claimed_lanes(self) -> None:
        """
        The claim-time equivalent of the PREP lane-test's data half.

        Mirrors _prep_lane's rule for a unit brought online by claim() rather
        than at boot: for each present bay, AFC's own restored profile/link wins
        (defer via _afc_owned so a later scan can update it), and a genuinely
        blank lane gets the bay's tag record surfaced onto it right away. Moves
        no filament -- it reads the AMS's cached record, exactly as boot does.
        Untagged present bays are left for _restore_untagged_defaults.
        """
        slots = getattr(self, "_slots", None) or []
        for name, slot in (getattr(self, "_slot_map", None) or {}).items():
            lane = self.lanes.get(name)
            if lane is None or not (0 <= slot < len(slots)):
                continue
            info = slots[slot]
            if not info or not info.get("present"):
                continue
            # Restore saved vars first, as PREP would have: PREP walks
            # afc.lanes, and a claimed unit's lanes were not in it when PREP
            # ran, so nothing restored them from AFC.var.unit. (claim() also
            # does this; this covers a lane still blank here.)
            if not (getattr(lane, "material", "")
                    or getattr(lane, "spool_id", None)):
                restore = getattr(self, "_restore_claimed_lane_vars", None)
                if restore is not None:
                    restore(lane)
            if (getattr(lane, "material", "")
                    or getattr(lane, "spool_id", None)):
                self._afc_owned.add(slot)         # AFC restored it -> defer
            elif info.get("material") or info.get("rfid_uid"):
                self._surface_slot_info(lane, info)   # blank lane -> the tag
            # A present bay with no tag and nothing saved is left to
            # _restore_untagged_defaults, which runs on every claim, not only
            # the first.

    def _print_claimed_prep(self) -> None:
        """
        Print the PREP block a live-claimed unit missed at boot.

        PREP skips pool units, and a spare claims its AMS seconds after PREP
        has finished, so its "Prepping lanes" banner, per-lane lines and logo
        were never printed. Print the same block now, read-only: the lane
        state was already set by _prep_claimed_lanes.
        """
        try:
            self.logger.info(f"{self.type} {self.name} Prepping lanes")
            slots = getattr(self, "_slots", None) or []
            latest = self._bridge.latest_status() if self._bridge else None
            online = self._unit_online(latest)
            ok = self._bridge is not None
            order = sorted((getattr(self, "_slot_map", None) or {}).items(),
                           key=lambda kv: kv[1])
            for name, slot in order:
                lane = self.lanes.get(name)
                if lane is None:
                    continue
                info = slots[slot] if 0 <= slot < len(slots) else {}
                _p, _s, _l, msg = prep_lane_state(
                    info or {}, getattr(lane, "tool_loaded", False), online,
                    fallback_material=getattr(lane, "material", None))
                _map_txt = (lane.map_to_string()
                            if hasattr(lane, "map_to_string") else lane.map)
                self.logger.info(f"{lane.name} tool cmd: {_map_txt} {msg}")
            self.logger.raw(self.logo if ok else self.logo_error)
        except Exception as e:
            self.logger.debug(
                f"AFC bambu {self.name}: claimed-unit PREP printout failed: {e}")

    def _handle_ready(self) -> None:
        """Build the slot map and connect to the bridge once the reactor is up.

        A pool unit still runs this far: it helps bring the shared bridge up
        (the first unit to load owns it, and that init is bus-wide and
        uid-independent), then stops before the per-unit, uid-dependent online
        (index resolution, announce, timers) -- claim() runs that later. So the
        bridge comes up owned at boot even when every unit is an idle pool unit.
        """
        if self.lanes:
            try:
                self._slot_map = build_slot_map(self.lanes, self.SLOTS_PER_UNIT)
            except ValueError as e:
                error_str = f"AFC_BambuAMS {self.name}: {e}"
                raise config_error(error_str)
        # Seed each lane's virtual-hub occupancy. Pinless lanes default
        # _load_state=True upstream, so a virtual hub (any(raw_load_state))
        # would read "loaded" for every lane until the first status frame.
        # Live occupancy is only true while the lane's tool is loaded.
        for lane in self.lanes.values():
            if self._is_virtual_hub(lane):
                lane._load_state = bool(getattr(lane, 'tool_loaded', False))
        if not self.serial_port:
            self.logger.info(
                f"AFC bambu {self.name}: no serial_port configured -- unit "
                f"stays offline. (A section with no serial_port is usually a "
                f"leftover in AFC_auto_vars.cfg for a renamed or removed "
                f"unit; it is safe to delete that section.)")
            return
        # Reuse an existing bridge for this serial port (shared across the
        # daisy-chained units), or create + start it for the first unit.
        bridge = _bridge_mod._BRIDGES.get(self.serial_port)
        fresh = bridge is None
        if fresh:
            bridge = BambuBridge(self._open_serial, self.afc.reactor,
                                 self.logger)
            # The AMS's narration goes to its own rotating file, independent of
            # AFC's `debug` flag. Once per bridge and one file per master: two
            # boxed units on different buses both narrate as 0x0700, so the first
            # master keeps the plain AFC_BambuAMS.log and later ones get the port.
            try:
                import os
                log_file = self.printer.start_args.get("log_file", None)
                if log_file:
                    bridge.set_narration_log(os.path.dirname(log_file),
                                             _bridge_log_tag(self.serial_port))
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: no narration log ({e})")
            _bridge_mod._BRIDGES[self.serial_port] = bridge
        self._bridge = bridge
        bridge.add_listener(self._on_status)
        bridge.add_reconnect_listener(self._on_bridge_reconnect)
        if self.pool:
            # Idle pool unit: own the bridge if we are the first unit to load
            # (bus-wide init, no uid needed), but go no further -- no announce,
            # no index resolution, no timers. _on_status ignores frames while
            # pooled, so the listener above is inert until claim() flips
            # self.pool False and runs the per-unit online.
            if fresh:
                try:
                    bridge.start(defer_open=self._bridge_is_tcp())
                    if self.variant != "auto":
                        bridge.send({"cmd": "variant", "set": self.variant})
                    bridge.send({"cmd": "status"})
                    self.logger.info(
                        f"AFC bambu {self.name}: brought the bridge up as the "
                        f"pool owner on {self.serial_port} (variant="
                        f"{self.variant}); units claim onto it live.")
                except Exception as e:
                    self.logger.warning(
                        f"AFC bambu {self.name}: pool-owner bridge init "
                        f"failed: {e}")
            return
        # After AFC restores saved lane state, re-assert the AMS "loaded" state
        # for any lane already tool-loaded (survives a reboot), using the
        # mode-07 "finish"/loaded signal rather than continuous feed assist:
        # blind feed while the extruder is idle stall-retracts and fights.
        try:
            self.afc.reactor.register_callback(
                lambda et: self._startup_restore_loaded(),
                self.afc.reactor.monotonic() + 5.0)
        except Exception:
            pass
        # After the firmware has reported its initial presence, mark the scan
        # baseline primed so only genuine post-startup inserts trigger a scan.
        # Same callback fills in defaults for any bay that came up present with
        # no tag: by now the firmware has reported presence and AFC has restored
        # the lanes, so "still blank" means blank for real.
        try:
            self.afc.reactor.register_callback(
                self._prime_scan_baseline, self.afc.reactor.monotonic() + 8.0)
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
        # every 30s. A unit that power-cycles mid-session can re-enroll at a
        # different address. Reading the map costs no AMS-bus traffic (it comes
        # from the Pico's RAM), and a detected move re-pins + re-seeds lanes.
        try:
            if self.unit_uid and self._uid_watch_timer is None:
                self._uid_watch_timer = self.afc.reactor.register_timer(
                    self._uid_watch_tick,
                    self.afc.reactor.monotonic() + 30.0)
        except Exception:
            pass
        # UID-pin before announcing or seeding anything: with several units on
        # one bridge, chain indices shuffle across power-cycles, and everything
        # per-unit (status filtering, PREP lane seeding, the HT flag, drying)
        # is keyed by ams_index. Resolving synchronously here means PREP
        # reports the right physical unit's occupancy and the announces below
        # flag the right chain index.
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
            bridge.start(defer_open=self._bridge_is_tcp())
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

    def set_master(self, master: Any) -> None:
        """
        Remember the AFC_BridgeBox that fabricated this unit.

        A pool unit never parsed a config section naming its master, so this
        back-reference is the only route to one. Used to persist values the
        unit learns about itself -- see _adopt_measured_path.

        :param master: the AFC_BridgeBox instance
        """
        self._master = master

    def claim(self, uid: str, model: str) -> bool:
        """
        Bind this idle pool unit to a live UID and bring it onto the bus.

        Runs the ready-time bus registration that the pool early-return in
        _handle_ready skipped: attach the bridge listener, (re)build the slot
        map from the lanes the caller has just activated, and resolve
        unit_uid -> chain index (which re-seeds the lanes from a fresh status
        and replays the announce). No restart. Called by AFC_BridgeBox after it
        activates the lanes; the caller mints the T# commands.

        :param uid: the physical AMS's 24-hex UID
        :param model: confirmed/assumed generation (ht/ams1/ams2/boxed)
        :return bool: True if the unit is now live on the bridge
        """
        if not self.pool:
            return False
        self.unit_uid = (uid or "").strip().upper() or None
        self.ams_model = model
        # Reassert heater capability from the claimed model on every claim: a
        # spare can change model between claims (ams1 has no heater, ams2/ht
        # do). An unknown model falls back to the unheated ams1 spec, with a
        # warning: a missing heater only refuses a dry command, while a
        # phantom one sends heat commands to hardware that may lack it.
        _spec = _AMS_MODELS.get(model)
        if _spec is None:
            self.logger.warning(
                f"AFC bambu {self.name}: claimed unknown model {model!r}; "
                f"assuming NO heater. Known: {', '.join(sorted(_AMS_MODELS))}")
        self.has_heater = (_spec or _AMS_MODELS["ams1"])[0]
        # The bridge and our listener were wired at boot (pool units run
        # _handle_ready to the owner point). Wire them here only if that
        # somehow did not happen -- never double-add the listener.
        if getattr(self, "_bridge", None) is None:
            bridge = _bridge_mod._BRIDGES.get(self.serial_port)
            if bridge is None:
                self.logger.warning(
                    f"AFC bambu {self.name}: claim found no live bridge on "
                    f"{self.serial_port}; unit stays offline until restart.")
                return False
            self._bridge = bridge
            bridge.add_listener(self._on_status)
            try:
                bridge.add_reconnect_listener(self._on_bridge_reconnect)
            except Exception:
                pass
        # unit_uid set + _id_resolved False means _on_status DEFERS every frame
        # until the index resolves -- so flipping pool False now cannot route a
        # frame to the wrong (default index 0) slot in the gap before resolution.
        self._id_resolved = False
        # A claim is this unit's connection starting: whatever its bays were
        # stamped with before now is baselined, not applied (see
        # _baseline_meas_stamp). _scan_primed survives a reclaim; this must not.
        self._stamp_looked = set()
        # So is the rest of the per-connection state: a spare's bay numbers
        # are addresses on a shared bus, and a latch, read, removal or
        # Spoolman miss recorded under an earlier claim belongs to whatever
        # unit held the address then.
        self._spoolman_latched = set()
        self._scanned_bays = set()
        afcBambuAMS._reset_connection_state(self)
        afcBambuAMS._reset_lookup_state(self)
        # A priming wait left from an earlier claim is not this one's (this
        # claim schedules its own, below), and the bays _prev_present
        # remembers may be another unit's.
        self._prime_waiting = False
        self._seed_present_seen = False
        try:
            for o in afcBambuAMS._built_measure_objs(self):
                if isinstance(getattr(o, "_spoolman_no_match", None), set):
                    o._spoolman_no_match = set()
        except Exception:
            pass
        self.pool = False
        try:
            self._slot_map = build_slot_map(self.lanes, self.SLOTS_PER_UNIT)
        except Exception as e:
            self.logger.warning(f"AFC bambu {self.name}: claim slot map: {e}")
        # Restore saved lane vars here, before the first frame. AFC_prep walks
        # afc.lanes, and these lanes were not in it (the unit was an inert
        # pool slot when PREP ran), so nothing has restored them from
        # AFC.var.unit; every "does this lane already have data?" check would
        # otherwise fall through to the bay's tag, and the next save would
        # overwrite the saved state. It must happen here rather than in
        # _prep_claimed_lanes: _sync_lanes surfaces tags as soon as
        # afc.prep_done is set, which is already true at claim time.
        restore = getattr(self, "_restore_claimed_lane_vars", None)
        if restore is not None:
            for lane in (self.lanes or {}).values():
                if (getattr(lane, "material", "")
                        or getattr(lane, "spool_id", None)):
                    continue
                try:
                    restore(lane)
                except Exception as e:
                    self.logger.debug(
                        f"AFC bambu {self.name}: claim restore "
                        f"{getattr(lane, 'name', lane)}: {e}")
        # Pin index by UID: this re-seeds the lanes from a fresh status frame
        # and, once resolved, replays _announce_unit at the real index.
        try:
            self._resolve_uid_index(0)
        except Exception as e:
            self.logger.warning(f"AFC bambu {self.name}: claim resolve: {e}")
        try:
            self._announce_unit()
        except Exception:
            pass
        # Defer scan priming like the boot path (_handle_ready): the first
        # frames after the claim record which bays came up present (the
        # baseline) so they are not re-read/re-measured as fresh inserts and
        # do not all pile measures onto the shared bus.
        try:
            self.afc.reactor.register_callback(
                self._prime_scan_baseline, self.afc.reactor.monotonic() + 8.0)
        except Exception:
            self._scan_primed = True      # no reactor (tests): scan immediately
        try:
            if self.unit_uid and getattr(self, "_uid_watch_timer", None) is None:
                self._uid_watch_timer = self.afc.reactor.register_timer(
                    self._uid_watch_tick, self.afc.reactor.monotonic() + 30.0)
        except Exception:
            pass
        # Restore the follower for whatever lane AFC records as loaded to the
        # toolhead, once, deferred so the index has resolved and PREP has run.
        # One toolhead per chain means at most one engage across all units,
        # done once and latched; a per-unit keep-alive would flood the shared
        # bridge. (Re-engagement after a stall is the load path or
        # AFC_BAMBU_FOLLOWER.)
        # Link-loss and stall detection. A claimed unit gets these without the
        # follower auto-arm half of _follow_tick: the load path and the one
        # deferred restore below own the follower here.
        try:
            if getattr(self, "_detector_timer", None) is None:
                self._detector_timer = self.afc.reactor.register_timer(
                    self._detector_tick,
                    self.afc.reactor.monotonic() + self.follow_poll_interval)
        except Exception:
            pass
        try:
            self._loaded_restore_done = False
            self._loaded_restore_tries = 0
            self.afc.reactor.register_callback(
                lambda et: self._restore_loaded_follower(),
                self.afc.reactor.monotonic() + 8.0)
        except Exception:
            pass
        self.logger.info(
            f"AFC bambu {self.name}: claimed UID {self.unit_uid} as {model} "
            f"and brought online live (ams_index={self.ams_index}).")
        return True

    def release(self) -> None:
        """
        Reverse claim(): drop off the bus and return to the idle pool state so
        the object can be reclaimed by a different UID without a restart.
        """
        bridge = getattr(self, "_bridge", None)
        if bridge is not None:
            for drop in ("remove_listener", "remove_reconnect_listener"):
                try:
                    fn = getattr(bridge, drop, None)
                    if fn:
                        fn(self._on_status if "reconnect" not in drop
                           else self._on_bridge_reconnect)
                except Exception:
                    pass
        self._bridge = None
        self.unit_uid = None
        self._id_resolved = False
        # Clear the prior claim's slot data so a get_status/PREP read cannot
        # briefly surface the old unit's spools on the freshly re-mapped lanes
        # before _adopt_index re-seeds them ~0.6s into the next claim.
        self._slots = [{} for _ in range(self.SLOTS_PER_UNIT)]
        # Likewise the measured percent, held summary and bound tag, on every
        # holder (as the removal edge does -- the percent lives on the
        # delegate): the next claim may be a different unit whose bays share
        # the slot numbers.
        #
        # The stamp memos (_meas_seen, _meas_seq_seen) are kept. The bridge
        # keeps each bay's stamp across a release, and a unit briefly offline
        # is released and reclaimed under the same UID; clearing them would
        # re-adopt old measurements over the grams used since. claim() starts
        # a new look (_stamp_looked), so any stamp present at the claim is
        # recorded by _baseline_meas_stamp and not applied.
        try:
            for _o in afcBambuAMS._built_measure_objs(self):
                for _d in ("_measured_remain", "_pending_summary",
                           "_bound_uid", "_bind_owed", "_convert_owed"):
                    _memo = getattr(_o, _d, None)
                    if isinstance(_memo, dict):
                        _memo.clear()
        except Exception:
            pass
        self.pool = True

    def _resolve_uid_index(self, tries: int) -> None:
        """
        Pin ams_index to the chain index currently carrying ``unit_uid``.

        The firmware assigns chain indices by announce order (reshuffles across
        power-cycles). This asks the bridge for the chain UID map and, when this
        unit's UID appears, adopts its index, so every command addresses the
        right physical unit regardless of boot order. Retries because the
        firmware needs a moment after boot to enroll all units.

        :param tries: retry counter
        """
        if self._bridge is None or not self.unit_uid:
            return
        self._bridge.send({"cmd": "chain"})            # request the enrollment map
        # The reply arrives asynchronously on the bridge's read thread, so
        # chain_uids() read on this tick would see the previous (or empty) map.
        # Match after a short delay, as AFC_BAMBU_UIDS does.
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
        isn't ready yet. Split from _resolve_uid_index so the read happens after
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
                f"AFC_BAMBU_UIDS. Holding chain index {self.ams_index}.")

    def _adopt_index(self, idx: int) -> None:
        """
        Adopt chain index ``idx`` as this unit's ams_index (UID-resolved) and
        re-key everything derived from it: the drying id, the firmware's polled
        unit count and HT flag, and this unit's cached slot data (anything cached
        under the old index belongs to another physical unit and must never seed
        this unit's lanes).

        :param idx: The chain index carrying this unit's unit_uid
        """
        old = self.ams_index
        was_resolved = self._id_resolved
        self._id_resolved = True          # a real chain index, not the default
        # Anything cached under an unverified index is another unit's: chain
        # order flips across reboots and every unit starts on ams_index 0. So
        # the first confirmation clears the cache too, even when the index
        # did not change.
        if idx == old and not was_resolved:
            self._slots = [{} for _ in range(self.SLOTS_PER_UNIT)]
            self._presence_ok = False     # and nothing known about them yet
            self._presence_wait_said = False
            try:
                self._bridge.send({"cmd": "status"})   # re-seed, verified now
            except Exception:
                pass
        if idx == old:
            # Re-confirmations happen at every resolve/reconnect, so log at
            # debug; only a changed pin (below) is logged at info.
            try:
                self.logger.debug(
                    f"AFC bambu {self.name}: UID {self.unit_uid} confirmed at "
                    f"ams_index {idx}")
            except Exception:
                pass
            self._send_ht_flag(self._bridge)     # re-assert (Pico may have rebooted)
            afcBambuAMS._send_unit_model(self, self._bridge)  # which machine
            self._send_mc_addr(self._bridge)
            if self._announce_deferred:      # held at connect -- send it now
                self._announce_deferred = False
                self._announce_defer_t0 = 0.0
                self._announce_defer_warned = False
                self._announce_unit()
            return
        self.ams_index = idx
        if self._dry_id_follows_index:
            self.dry_ams_id = mc_id_for_index(idx)
        self._slots = [{} for _ in range(self.SLOTS_PER_UNIT)]
        self._presence_ok = False         # the new index has not answered
        self._presence_wait_said = False
        try:
            self._bridge.send({"cmd": "units", "n": idx + 1})
            # Move the HT flag from the old index to the resolved one so the
            # firmware arms the insert-edge scan on the right unit.
            if self._is_ht():
                self._bridge.send({"cmd": "htunit", "unit": old, "on": 0})
            self._send_ht_flag(self._bridge)
            afcBambuAMS._send_unit_model(self, self._bridge)
            self._send_mc_addr(self._bridge)
            self._bridge.send({"cmd": "status"})    # re-seed from the right unit
        except Exception:
            pass
        self.logger.info(
            f"AFC bambu {self.name}: pinned to UID {self.unit_uid} at "
            f"chain index {idx} (was ams_index {old})")
        if self._announce_deferred:          # held at connect -- send it now
            self._announce_deferred = False
            self._announce_defer_t0 = 0.0
            self._announce_defer_warned = False
            self._announce_unit()

    def _resolve_uid_blocking(self, timeout: float = 30.0) -> bool:
        """
        Resolve ``unit_uid`` -> ams_index synchronously, pausing the reactor
        until the chain map arrives (or ``timeout``). Called from klippy:ready
        before anything seeds lanes or announces per-unit state, so PREP always
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
        # Debug, not warning: a slow chain at startup is an expected transient,
        # and the ~1min background retry and the 30s watchdog re-pin, re-flag
        # and re-seed lanes when the UID appears. The 40-retry give-up (wrong
        # unit_uid value) warns.
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
        # Never register at a guessed index. Until unit_uid resolves (or, for
        # an unclaimed pool spare, until it is claimed), ams_index is the
        # config default 0, and every registration below would land on
        # whichever unit really holds index 0 -- e.g. the other units' HT
        # flag, MC address and self-centre flag sent to it. Defer instead:
        # the resolve retries in the background and _adopt_index replays this
        # once the real index is known.
        # getattr so a partially built unit cannot raise here.
        if ((self.unit_uid or getattr(self, "pool", False))
                and not self._id_resolved):
            self._announce_deferred = True
            # Debug level: waiting for the chain map at boot is normal. It is
            # only a problem if it never resolves, so record when the wait
            # started and let _check_chain_resolve escalate.
            if not getattr(self, "_announce_defer_t0", 0.0):
                try:
                    self._announce_defer_t0 = self.afc.reactor.monotonic()
                except Exception:
                    self._announce_defer_t0 = 0.0
            why = (f"UID {self.unit_uid}" if self.unit_uid
                   else "unclaimed pool slot, no UID yet")
            self.logger.debug(
                f"AFC bambu {self.name}: chain index not resolved yet ({why}); "
                f"holding this unit's registrations until the chain map "
                f"arrives")
            return
        # Each send has its own try/except so one failure cannot skip the
        # rest, in particular _send_mc_addr. Without a per-unit MC address the
        # firmware's log drain falls back to the 0x0700 pair, which never asks
        # an HT at 0x1800, so every HT load, unload, stall and measured length
        # is lost while the bus still looks healthy.
        for what, fn in (
                ("units", lambda: self._bridge.send(
                    {"cmd": "units", "n": self.ams_index + 1})),
                ("ht flag", lambda: self._send_ht_flag(self._bridge)),
                ("model", lambda: afcBambuAMS._send_unit_model(self, self._bridge)),
                ("mc address", lambda: self._send_mc_addr(self._bridge)),
                ("arm cadence", lambda: self._bridge.send(
                    {"cmd": "armms", "ms": int(FOLLOW_ARM_MS)}))):
            try:
                fn()
            except Exception as e:
                # Name the failed send: a silent failure looks like a unit
                # that is online and polling but never narrates.
                self.logger.warning(
                    f"AFC bambu {self.name}: could not announce {what} to the "
                    f"bridge ({e}); narration or addressing may be wrong")

    def _on_bridge_reconnect(self) -> None:
        """
        The serial link came back, which usually means the Pico rebooted
        (reflash, power-cycle, replug): the firmware's unit count and HT flags
        are gone and the chain may have re-enrolled in a different order.
        Re-announce this unit's config and re-resolve the UID pin.
        """
        # A new connection, so the next frame is a first look at every bay's
        # measurement stamp: one that changed while the link was down was not
        # watched being taken, and is baselined rather than applied (see
        # _baseline_meas_stamp). The bridge queues this callback ahead of the
        # new connection's first frame, so no frame is judged against the old
        # connection's look.
        self._stamp_looked = set()
        # Reset presence history too: a rebooted Pico restarts its sequence
        # counters, and a bay it reports empty before its first presence poll
        # was not seen removed (_present_seen). The lookup state and its
        # latch are kept.
        afcBambuAMS._reset_connection_state(self)
        # Frames that say when the bridge has polled (_presence_known) keep
        # the booting frames off the lanes, so _prev_present is still the
        # last real look; a bay it had occupied that the first answer calls
        # empty was emptied, and _on_status seeds _present_seen from it.
        self._seed_present_seen = True
        # A scan open across the gap is checked on that same answer
        # (_close_scans_the_bridge_lost).
        self._scans_to_check = True
        # The info reply (chip, fw, variant) is only sent on request. This
        # early ask may be lost to link-key auth; _announce_after_settle asks
        # again.
        try:
            self._bridge.request_info()
        except Exception:
            pass
        # The Pico is not rebooted on reconnect (a watchdog reboot can take a
        # USB-CAN adapter down with it). Announce after a settle delay on the
        # reactor; this runs on the bridge's reader thread.
        try:
            reactor = self.afc.reactor
            reactor.register_callback(
                lambda et: self._announce_after_settle(),
                reactor.monotonic() + ANNOUNCE_SETTLE_S)
            return
        except Exception:
            # No usable reactor (early boot, or a test shim): announce now.
            pass
        self._announce_after_settle()

    def _announce_after_settle(self) -> None:
        """
        The announce half of the reconnect handshake, run once the firmware has
        had ANNOUNCE_SETTLE_S to finish booting. Split out so it can be
        deferred onto the reactor instead of sleeping on the reader thread.
        """
        # Request info again after the settle. The ask in _on_bridge_reconnect
        # runs as the socket opens, before link-key auth completes, so the
        # board may never answer it. chip() (used by AFC_BAMBU_FLASH) and the
        # reboot-vs-hiccup detection both depend on this reply.
        try:
            self._bridge.request_info()
        except Exception:
            pass
        self._announce_unit()
        if self.unit_uid:
            self._resolve_uid_index(0)
        try:
            self._bridge.send({"cmd": "status"})
        except Exception:
            pass
        # Re-assert the loaded lane: a reconnect means the Pico rebooted while
        # Klipper kept running, so the AMS has no memory of which tray is
        # threaded and the follower goes dead. Same call and gate as
        # klippy:ready (tool_loaded AND ready_to_follow), after a short settle.
        try:
            self.afc.reactor.register_callback(
                lambda et: self._startup_restore_loaded(),
                self.afc.reactor.monotonic() + 1.0)
        except Exception:
            pass
        self._relink_after_reboot()

    def _relink_after_reboot(self) -> None:
        """Relink the chain when the bridge rebooted under it, not when the
        link merely dropped.

        When the Pico reboots with AMS units on the wire, the units lose
        their bus master mid-conversation and park with LEDs blinking.
        CLEARFAULT does not clear this (the units read state 0, not parked);
        a relink does, because it re-registers them.

        Not run on every reconnect: a relink is a deregister sweep, and a
        brief network drop should not interrupt the chain. It fires only when
        the bridge's uptime went backwards, which a dropped link cannot fake
        (see consume_reboot).

        Never during a print: a mid-print chain reset is worse than the
        blinking LEDs, which stay until the print ends.
        """
        bridge = self._bridge
        if bridge is None:
            return
        try:
            if not bridge.consume_reboot():     # one-shot, chain-wide
                return
        except AttributeError:
            return                              # bridge predates the counter
        try:
            if self.afc.function.in_print():
                self.logger.info(
                    f"AFC bambu {self.name}: bridge rebooted, but a print is "
                    f"running -- not relinking. Units may show a fault until "
                    f"the print ends; AFC_BAMBU_RELINK UNIT={self.name} clears "
                    f"it by hand.")
                return
        except Exception:
            return                              # cannot tell: do nothing
        self.logger.info(
            f"AFC bambu {self.name}: bridge rebooted -- relinking the chain so "
            f"the units drop their park (they lost their master mid-frame)")
        try:
            bridge.send({"cmd": "relink"})
        except Exception:
            self.logger.debug(f"AFC bambu {self.name}: post-reboot relink failed",
                              traceback=traceback.format_exc())
            return
        # The relink deregisters every unit, not just this one, so replay all
        # reconnect listeners to reconfigure the whole chain; otherwise only
        # this unit recovers. consume_reboot() has already fired, so the
        # replay cannot loop back into another relink.
        try:
            reactor = self.afc.reactor
            reactor.register_callback(
                lambda et: bridge.replay_reconnect_listeners(),
                reactor.monotonic() + RELINK_SETTLE_S)
        except Exception:
            try:
                bridge.replay_reconnect_listeners()
            except Exception:
                pass

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
                elif not self._id_resolved:
                    # Same number, but only as the unconfirmed config default.
                    # Adopt so the index counts as resolved and held-back
                    # registrations are released; otherwise a unit whose real
                    # index equals the default would stay deferred forever.
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
        for lane in getattr(self, "lanes", {}).values():
            if getattr(lane, "tool_loaded", False):
                # Routine on every restart with a lane loaded, hence debug.
                if not self._ready_to_follow(lane):
                    # Steppers come up de-energised; engaging now would make
                    # the AMS pulse feeding its buffer with nothing gripping
                    # the filament downstream. The follow poll loop arms it
                    # once the motors come on.
                    self.logger.debug(
                        f"AFC bambu {self.name}: {lane.name} loaded at "
                        f"startup on an unhomed machine with a de-energised "
                        f"extruder; leaving the follower for the poll loop to "
                        f"arm once either changes")
                    continue
                self.logger.debug(
                    f"AFC bambu {self.name}: {lane.name} loaded at startup, "
                    f"re-asserting AMS loaded state + follower")
                self._engage_follower(lane)

    def _restore_loaded_follower(self, _et: Optional[float] = None) -> None:
        """
        One-shot: re-engage the follower for the lane AFC records as loaded to
        the toolhead, after a claim. Scheduled deferred from claim() so the UID
        index has resolved (follow commands must address the right unit) and
        AFC's PREP has restored extruder.lane_loaded.

        A pool lane's own tool_loaded flag is NOT restored from vars, but AFC's
        extruder.lane_loaded IS -- that is the authoritative record of what is
        loaded. Repair tool_loaded from it, then let _startup_restore_loaded
        engage the tray (mode:4 latch).

        There is one toolhead per chain, so at most one lane across every unit
        on the bridge is loaded; only the unit that owns it acts, and it
        engages once (latched by _loaded_restore_done): one _engage_follower
        per chain, with no repeated sends.
        """
        if getattr(self, "_loaded_restore_done", False):
            return
        # Not addressable yet (index unresolved / bridge down): retry a bounded
        # number of times rather than latch and miss the restore.
        if self._bridge is None or not getattr(self, "_id_resolved", False):
            tries = getattr(self, "_loaded_restore_tries", 0)
            if tries < 6:
                self._loaded_restore_tries = tries + 1
                try:
                    self.afc.reactor.register_callback(
                        lambda et: self._restore_loaded_follower(),
                        self.afc.reactor.monotonic() + 3.0)
                except Exception:
                    pass
            return
        afc = self.afc
        loaded = {ln for ln in
                  (getattr(e, "lane_loaded", None)
                   for e in getattr(afc, "tools", {}).values()) if ln}
        mine = [lane for lane in getattr(self, "lanes", {}).values()
                if lane.name in loaded]
        self._loaded_restore_done = True
        if not mine:
            return                      # the loaded lane belongs to another unit
        for lane in mine:
            if not getattr(lane, "tool_loaded", False):
                lane.tool_loaded = True         # repair from AFC's authoritative record
        try:
            self._startup_restore_loaded()      # engages the tray -- mode:4, once
        except Exception as ex:
            self.logger.warning(
                f"AFC bambu {self.name}: loaded-follower restore failed: {ex}")
            return
        self.logger.info(
            f"AFC bambu {self.name}: restored the follower for "
            f"{', '.join(lane.name for lane in mine)} (AFC records it loaded to "
            f"the toolhead) -- mode:4, one-shot.")

    def _engage_follower(self, lane: Any) -> None:
        """
        Put a tool-loaded lane's AMS tray into mode:4 and hold it (follower).

        Order matters and matches the load path: commit the loaded state first
        (mode-07 "finish"), then select the tray (mode-09 on an already-loaded
        tray flips it to mode:4), then assist to hold mode:4 via the AP2 sync
        heartbeat. Finish after select knocks the tray back out of mode:4, so
        assist has nothing to hold and the follower never runs (LED solid,
        extruder can't pull). select must be the last mode change before
        assist.

        :param lane: A lane whose filament is threaded to the toolhead
        """
        self.bridge_finish(lane)     # commit loaded (mode-07)
        self.select_lane(lane)       # mode-09 -> mode:4 (last mode change)
        self.set_feed_assist(lane, True)  # hold mode:4 via AP2 sync

    def _bridge_is_tcp(self) -> bool:
        """
        Whether this unit's bridge is reached over the network.

        Decides whether a first connect that fails is a fault or just a bridge
        that is not up yet -- see BambuBridge.start(defer_open=...).

        :return bool: True when serial_port names a tcp:// endpoint
        """
        return str(self.serial_port or "").strip().lower().startswith("tcp://")

    def _open_serial(self) -> Any:
        """
        Open the link to the Pico (pyserial imported lazily).

        Normally the USB-CDC port. A serial_port written as ``tcp://host:port``
        opens a socket instead -- see TcpPort, and read its warning first: USB
        is the tested transport and this is opt-in.

        :return Any: an open port, pyserial-shaped either way
        """
        spec = str(self.serial_port or "")
        if spec.strip().lower().startswith("tcp://"):
            host, port = TcpPort.parse(spec)
            # Logged before the connect, since a refused connection raises out
            # of here and the address would otherwise never be named. Info
            # once, then debug: the reader calls this factory on every backoff
            # pass, and the watchdog reports the outage itself.
            _say = self.logger.info if not getattr(self, "_tcp_said", False) \
                else self.logger.debug
            self._tcp_said = True
            _say(f"AFC bambu {self.name}: bridge over TCP at tcp://{host}:{port}"
                 f" (opt-in transport -- USB-CDC is the supported one)")
            # Same timeouts as the serial path below, and for the same reason:
            # every send() is made from a reactor timer, so a far end that
            # stops reading must time out rather than block the reactor.
            return TcpPort(host, port, timeout=0.1, write_timeout=0.5,
                           key=self.tcp_key)
        import serial
        # write_timeout is required: every send() runs on the reactor, and a
        # Pico busy on the AMS bus stops draining its CDC. A blocked write
        # would stall the reactor long enough to shut the MCUs down.
        return serial.Serial(self.serial_port, self.baud,
                             timeout=0.1, write_timeout=0.5)

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
        PREP-time lane test: read the bay's sensors, keep what AFC restored.

        Seeds prep/hub state from the bridge's cached slot info (present spool ->
        prep_state + staged-at-hub), keeps the virtual hub's live occupancy
        derived from tool_loaded, and assigns the lane's T-command. The bridge
        being offline (protocol bring-up) is reported but does not fail the lane.

        Like OpenAMS and ACE at prep, it does not put the bay's tag record onto
        a lane AFC already populated. An occupied bay whose lane came back with
        a profile or a Spoolman link is marked in ``_afc_owned`` instead, which
        also holds the status path off until the unit answers a scan for that
        bay.

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
                # separate load sensor), so mark it LOADED, mirrors AFC_ACE.
                self.lane_loaded(cur_lane)
                cur_lane.status = AFCLaneState.LOADED
                self.lane_illuminate_spool(cur_lane)
                # Prep reads sensors; AFC's var file owns the filament data at
                # boot, like OpenAMS and ACE. sub_type is the exception: saved
                # by get_status but never restored by AFC_prep, so read it
                # back here.
                self._restore_sub_type(cur_lane)
                if (getattr(cur_lane, "material", "")
                        or getattr(cur_lane, "spool_id", None)):
                    self._afc_owned.add(slot)
                else:
                    self._surface_slot_info(cur_lane, info)
        self._prep_seen = True               # the status path may write now
        if assignTcmd:
            try:
                # A unit claimed from the pool before PREP already has its T#
                # registered; the pool helper skips the second registration.
                from extras.AFC_BridgeBox import assign_pool_tcmd
                assign_pool_tcmd(cur_lane, self.afc)
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
        _map_txt = (cur_lane.map_to_string()
                    if hasattr(cur_lane, "map_to_string") else cur_lane.map)
        self.logger.info(f"{cur_lane.name} tool cmd: {_map_txt} {msg}")
        try:
            cur_lane.set_afc_prep_done()
        except Exception as e:
            self.logger.warning(f"AFC bambu: set_afc_prep_done failed: {e}")
        return succeeded

    # -- status mirroring --

    def _on_status(self, obj: dict) -> None:
        """
        Reactor callback: fold this unit's slice of a bridge status frame onto
        the lanes. On a daisy-chain, the shared bridge hands every unit the whole
        frame; we keep only the slots tagged with our ams_index.

        :param obj: A decoded bridge status event
        """
        try:
            # A pooled/released unit ignores the bus entirely -- it may still
            # hold a listener a bridge without remove_listener could not drop.
            if getattr(self, "pool", False):
                return
            # Identity before data. With unit_uid configured, ams_index is
            # the config default until the chain map resolves the UID, so
            # every unit would match unit 0's slots. Status frames arrive
            # continuously, so nothing is lost by waiting.
            if (getattr(self, "unit_uid", None)
                    and not getattr(self, "_id_resolved", True)):
                return
            # Latch the unit's give-up (byte[19] == 0x07) on every frame: it
            # appears on only some park frames, so a single sample can miss
            # it. Cleared when a fault is armed. Own try, so a throw here
            # cannot take status mirroring down.
            try:
                if self._unit_state(obj) == self.AMS_STATE_STALLED:
                    self._declared_since_fault = True
                self._track_odom(obj)
            except Exception:
                pass
            # Identity is verified on every frame, not once at boot: the firmware
            # re-enrolls the chain on a bus hiccup and chain ORDER is discovery
            # order, so a stale ams_index would write the wrong unit's lanes.
            # chain_uids() is the bridge's cached list, no bus traffic.
            if getattr(self, "unit_uid", ""):
                try:
                    uids = self._bridge.chain_uids()
                    if uids and self.unit_uid in uids:
                        live = uids.index(self.unit_uid)
                        if live != self.ams_index:
                            self.logger.warning(
                                f"AFC bambu {self.name}: chain re-enrolled -- "
                                f"UID {self.unit_uid} moved from index "
                                f"{self.ams_index} to {live}; re-adopting")
                            self._adopt_index(live)
                            return      # this frame was keyed to the old index
                except Exception:
                    pass
            # A just-booted Pico publishes every bay empty, with a blank
            # record, until the unit answers its first presence poll. Those
            # frames are not folded: the lanes keep what AFC.var.unit restored
            # (after a restart) or what the last real frame said (after a
            # bridge reboot) until the bridge knows each bay's occupancy.
            # Folding them would unbind loaded spools and start tag scans.
            if not afcBambuAMS._presence_known(self, obj):
                self._booted_under_us = True
                if not getattr(self, "_presence_wait_said", False):
                    self._presence_wait_said = True
                    self.logger.debug(
                        f"AFC bambu {self.name}: the bridge has not polled "
                        f"this unit's bays yet; keeping the lanes as they are")
                return
            self._presence_ok = True
            if getattr(self, "_seed_present_seen", False):
                self._seed_present_seen = False
                ent = afcBambuAMS._unit_entry(self, obj)
                if ent is not None and ent.get("preslen") is not None:
                    seen = getattr(self, "_present_seen", None)
                    if seen is None:
                        seen = self._present_seen = set()
                    seen.update(i for i, p in enumerate(
                        getattr(self, "_prev_present", None) or []) if p)
            for entry in obj.get("slots") or []:
                if entry.get("unit", 0) != self.ams_index:
                    continue
                info = bridge_slot_to_info(entry)
                idx = info.get("index")
                if isinstance(idx, int) and 0 <= idx < self.SLOTS_PER_UNIT:
                    self._slots[idx] = info
            # A scan the bridge was running when it restarted died with it.
            if (getattr(self, "_booted_under_us", False)
                    or getattr(self, "_scans_to_check", False)):
                rebooted = getattr(self, "_booted_under_us", False)
                self._booted_under_us = False
                self._scans_to_check = False
                afcBambuAMS._close_scans_the_bridge_lost(self, rebooted)
            self._sync_lanes()
            # Priming that came due while the bridge had not asked runs now,
            # on the first frame that answers -- folded above, so the baseline
            # and the reconcile both see it.
            if getattr(self, "_prime_waiting", False):
                afcBambuAMS._prime_scan_baseline(self)
        except Exception as e:
            msg = f"AFC bambu {self.name}: status apply error: {e}"
            if msg != self._status_err_last:
                self._status_err_last = msg
                self.logger.warning(msg)
            else:
                self.logger.debug(msg)

    def _close_scans_the_bridge_lost(self, rebooted: bool) -> None:
        """
        End the tag scans a bridge restart killed, leaving their lanes alone.

        A reboot takes the firmware's scan window with it and zeroes every
        bay's scan_seq, so a scan open across it would read the counter's
        move (or, from a zero baseline, SCAN_VERDICT_CAP's expiry) as the
        unit's "no tag" -- and a "no tag" unbinds the lane and puts it on
        defaults. The bay is the same bay and the unit never said that. So
        the hold is released: the lane keeps what it has, a tag the unit
        re-reads by itself surfaces as usual, and the operator is told to
        scan again.

        Called on the first frame after a reconnect, or after frames from a
        bridge that had not polled the unit yet, with the new frame folded.

        :param rebooted: the bridge was seen booting (frames from before its
            first presence poll); otherwise a scan counter that went BACKWARDS
            is what shows the bridge restarted
        """
        t0s = getattr(self, "_scan_t0", None) or []
        base = getattr(self, "_scan_seq0", None) or {}
        for slot in range(len(t0s)):
            if t0s[slot] is None:
                continue
            killed = rebooted
            if not killed:
                try:
                    seq = (self._slots[slot] or {}).get("scan_seq")
                    b = base.get(slot)
                    killed = (seq is not None and b is not None
                              and int(seq) < int(b))
                except Exception:
                    killed = False
            if not killed:
                continue
            self._release_scan_hold(slot)
            base.pop(slot, None)
            (getattr(self, "_cycle_end_seen", None) or {}).pop(slot, None)
            mt = getattr(self, "_scan_motion_t0", None)
            if mt is not None and 0 <= slot < len(mt):
                mt[slot] = None
            lane = self._lane_for_slot(slot)
            name = getattr(lane, "name", None) or f"bay {slot + 1}"
            self.logger.info(
                f"AFC bambu {self.name}: the bridge restarted during the tag "
                f"scan of {name}, which ended the scan; {name} is left as it "
                f"is. Run AFC_BAMBU_SCAN LANE={name} to scan it again")

    #: How long scan priming waits on the firmware's own re-read of the bays
    #: it found occupied at boot. BLANK_READ_TRIES settles one in a few
    #: seconds; this only catches a re-read that never settles.
    PRIME_REREAD_WAIT_S = 30.0

    def _rereads_owed(self) -> bool:
        """
        Whether scan priming should still wait on a re-read the firmware owes.

        :return bool: True while an occupied bay's record is flagged
            ``reread_pending``, for at most PRIME_REREAD_WAIT_S
        """
        slots = getattr(self, "_slots", None) or []
        n = min(len(slots), getattr(self, "unit_slots", len(slots)))
        owed = any(slots[i] and slots[i].get("present")
                   and slots[i].get("reread_pending") for i in range(n))
        if not owed:
            self._reread_wait_t0 = None
            return False
        try:
            now = self.afc.reactor.monotonic()
        except Exception:
            return False
        t0 = getattr(self, "_reread_wait_t0", None)
        if t0 is None:
            self._reread_wait_t0 = now
            return True
        return (now - t0) < self.PRIME_REREAD_WAIT_S

    def _unit_entry(self, obj: dict) -> Optional[dict]:
        """
        This unit's entry in a status frame's per-unit list.

        :param obj: A bridge status frame
        :return dict: the entry, or None if the frame lists no such unit
        """
        for u in obj.get("units") or []:
            if isinstance(u, dict) and u.get("n") == self.ams_index:
                return u
        return None

    def _presence_known(self, obj: dict) -> bool:
        """
        Whether a status frame's bays say anything about THIS unit's bays.

        The firmware keeps, per unit, the length of the last presence reply
        it accepted (``preslen``). It is zero from boot until the unit
        answers its first presence poll and never zero again, so while it is
        zero every ``present`` in the frame is the boot default, not an
        answer. A frame without the per-unit list, or from a firmware that
        does not publish the field, cannot say, and is trusted.

        :param obj: A bridge status frame
        :return bool: False only when the frame shows the bridge has not yet
            had a presence reply from this unit
        """
        units = obj.get("units")
        if not isinstance(units, list) or not units:
            return True
        mine = afcBambuAMS._unit_entry(self, obj)
        if mine is None:
            # Units listed, this one not: the bridge is not polling it yet (a
            # rebooted Pico polls only the units it has been told of).
            return not any(isinstance(u, dict) and "preslen" in u
                           for u in units)
        if mine.get("preslen") is None:
            return True
        try:
            return int(mine["preslen"]) > 0
        except (TypeError, ValueError):
            return True

    def _unit_online(self, latest: Optional[dict]) -> bool:
        """
        Whether THIS unit's AMS is online in a bridge status frame.

        :param latest: A bridge status dict (or None)
        :return bool: True if this ams_index's unit reports online
        """
        # An idle pool bay has no unit on the wire and keeps the default
        # ams_index 0, so matching by ams_index would read another unit's
        # online flag. It is offline until claimed (claim() clears self.pool).
        if getattr(self, "pool", False):
            return False
        if not latest:
            return False
        u = afcBambuAMS._unit_entry(self, latest)
        if u is not None:
            return bool(u.get("online"))
        return bool(latest.get("online"))     # single-unit fallback

    #: The unit's own state, from op-04 reply byte[19]. 0x07 is STALLED -- the
    #: same "state:7" the HT and AMS 2 print in words, and the only fault
    #: signal an AMS 1 emits (it narrates neither the fault nor the
    #: recovery). Firmware debounces it; 255 = not heard from.
    AMS_STATE_STALLED = 0x07
    #: Every unit state that means "parked on a fault", matching the
    #: narration's `state:[67]` (see _AMS_FAULT_RE). The CLEARFAULT verdict
    #: checks against all of these, so a unit parked in 0x06 is not reported
    #: as cleared.
    AMS_STATES_FAULTED = (0x06, 0x07)

    #: How long ustate must read STALLED, without clearing, before a load
    #: attempt stops feeding and hands over to the recovery re-home. A unit
    #: genuinely retrying reads 2 the whole time; a latched one holds 7 for
    #: minutes, so ten seconds separates them cleanly.
    STALL_LATCH_S = 10.0

    def _unit_state(self, latest: Optional[dict]) -> Optional[int]:
        """
        THIS unit's reported state from a bridge status frame.

        :param latest: A bridge status dict (or None)
        :return int: the state byte, or None if this unit has not reported one
        """
        # As in _unit_online: an idle pool bay has no state of its own.
        if getattr(self, "pool", False):
            return None
        if not latest:
            return None
        u = afcBambuAMS._unit_entry(self, latest)
        if u is None:
            return None
        st = u.get("ustate")
        # 255 is the firmware saying "not heard from yet", which is not a state
        # and must never be read as one.
        return None if st is None or st == 0xFF else int(st)

    #: How far the AMS must have moved filament, in mm, during a recovery
    #: attempt to count as moving. A working AMS swings nearly the full tube
    #: during its retry and a unit that cannot move filament sits near zero,
    #: so this only separates "swung the tube" from "barely twitched".
    ODOM_MOVED_MM = 200.0

    def _track_odom(self, obj: dict) -> None:
        """
        Record the range the AMS's odometer covers while a fault is pending.

        The odometer is a position, not a consumption counter: 0 is home in
        the AMS, about tube length is at the toolhead. During a print it sits
        at tube length however much filament is consumed, so it carries no
        clog signal.

        The range does show where a jam is once a recovery (or a load) is
        running: a jammed retry sweeps most of the tube while a park barely
        moves. No extra bus traffic; it only records while a fault is pending
        or a load is in progress.

        :param obj: A decoded bridge status event
        """
        fault = self._follow_fault_hold
        load = getattr(self, "_load_in_progress", False)
        if not fault and not load:
            return                          # nothing is asking the question
        u = afcBambuAMS._unit_entry(self, obj)
        v = u.get("odom") if u is not None else None
        if v is None or int(v) == -1:       # firmware's unknown sentinel
            return
        mm = float(v)
        if fault:
            lo, hi = self._odom_lo, self._odom_hi
            self._odom_lo = mm if lo is None else min(lo, mm)
            self._odom_hi = mm if hi is None else max(hi, mm)
        if load:
            lo, hi = self._load_odom_lo, self._load_odom_hi
            self._load_odom_lo = mm if lo is None else min(lo, mm)
            self._load_odom_hi = mm if hi is None else max(hi, mm)

    def _odom_span_mm(self) -> Optional[float]:
        """
        How far the odometer ranged since the fault, in mm, or None.

        :return float: the span, or None if we never got two readings
        """
        if self._odom_lo is None or self._odom_hi is None:
            return None
        return self._odom_hi - self._odom_lo

    def _load_odom_span_mm(self) -> Optional[float]:
        """
        How far the odometer ranged during the current load, in mm, or None.

        A range, not a delta: the odometer is a position (see _track_odom),
        so a unit that swings out and back would read a delta of zero having
        moved the whole tube twice.

        :return float: the span, or None if we never got two readings
        """
        if self._load_odom_lo is None or self._load_odom_hi is None:
            return None
        return self._load_odom_hi - self._load_odom_lo

    def _jam_location(self, span: Optional[float] = None) -> str:
        """
        Say where the jam is, from how far the AMS moved filament.

        A toolhead jam and a spool tangle need different responses (cut and
        retract vs. a person at the AMS). If the AMS swung the filament
        along the tube and the toolhead sensor still never triggered, the
        blockage is downstream; if it barely moved, it is at the AMS end.

        Returns "" when the span is unknown (fewer than two readings, e.g.
        the unit went quiet).

        :param span: the range to judge, in mm. Defaults to the fault
          recovery's. A failed load passes its own window, which also
          catches a disconnected tube: the AMS feeds into thin air and only
          its odometer shows how far the filament went.
        :return str: an operator-facing sentence, or "" if we cannot tell
        """
        if span is None:
            span = self._odom_span_mm()
        if span is None:
            return ""
        if span >= self.ODOM_MOVED_MM:
            return (f"The AMS moved filament {span:.0f}mm during the attempt "
                    f"and it still did not reach the toolhead, so the blockage "
                    f"is DOWNSTREAM OF THE AMS -- the bowden or the toolhead, "
                    f"not the spool.")
        return (f"The AMS moved filament only {span:.0f}mm during the attempt, "
                f"so the blockage is AT THE AMS -- check the spool for a "
                f"tangle and the bay for a snag.")

    def _check_unit_stalled(self, lane: Any) -> bool:
        """
        Raise a fault when the unit itself reports it has stalled.

        The primary detector, and the only one covering every unit: all
        three set op-04 reply byte[19] to 0x07 when they stall, at the moment
        the two that narrate print "state:7". An AMS 1 sets the byte while
        emitting no fault text at all. Being a byte, it also avoids the
        narration's dialect differences between models.

        0x07 does not occur in healthy operation, and the firmware requires
        three agreeing replies before committing a state, so a misframed
        reply cannot pause a print.

        :param lane: The lane currently followed
        :return bool: True if a fault was raised
        """
        if not getattr(self, "fault_detect", False) or self._bridge is None:
            return False
        if (getattr(self, "_unload_in_progress", False)
                or getattr(self, "_drying", False)):
            return False
        state = self._unit_state(self._bridge.latest_status())
        if state != self.AMS_STATE_STALLED:
            self._stalled_seen = False
            return False
        if getattr(self, "_stalled_seen", False):
            return False            # already reported this stall; one per event
        self._stalled_seen = True
        msg = (f"AFC bambu {self.name}: the AMS reports it has STALLED "
               f"(state {state}) on {lane.name} -- the spool is likely tangled "
               f"or the path jammed. Clear the snag, then resume.")
        try:
            if self.afc.function.in_print():
                msg += "\nOnce cleared, click resume to continue printing"
        except Exception:
            pass
        self._raise_ams_fault(lane, msg)
        return True

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
        A present spool in an AMS bay is staged and ready -- there is no
        separate hub/load sensor -- so ``loaded_to_hub`` is latched True and
        the lane marked LOADED, like OpenAMS/AFC_ACE. That keeps Mainsail from
        showing "filament detected but not loaded". A lane whose status is
        mid-operation (load/unload/eject/error) is never touched.

        For a virtual hub, ``_load_state`` is the live hub-occupancy signal --
        the native AFC_hub aggregates ``any(lane.raw_load_state)`` -- so it is
        True only while this lane's filament is threaded through the hub to
        the toolhead (``tool_loaded``), never for a merely-staged lane, or the
        lane's own load would trip the "hub not clear" gate.
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
            # Before any gate: the first look at a bay's measurement stamp on
            # this connection must happen on the first frame that carries it,
            # whether or not PREP has run, or a later frame would apply it as
            # new. And before the insert edge, so a window that edge opens on
            # this same frame (e.g. a reclaim's stale presence making bays
            # look freshly inserted) cannot exempt a stamp it did not ask for.
            afcBambuAMS._baseline_meas_stamp(self, slot, info)
            # A stamp kept for the spool that left (see the adopt block) is
            # dropped once the bay's sequence runs below it: the counter
            # started again, so an equal pair from here is a new measurement.
            _dep = getattr(self, "_meas_departed", None)
            if _dep and slot in _dep and info.get("meas_seq") is not None:
                try:
                    if int(info["meas_seq"]) < int(_dep[slot][0]):
                        _dep.pop(slot, None)
                except Exception:
                    _dep.pop(slot, None)
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
                # The spool AFC's data described is gone, so there is nothing
                # left to defer to.
                self._afc_owned.discard(slot)
                if not active and status != AFCLaneState.NONE:
                    try:
                        self.lane_not_ready(lane)
                    except Exception:
                        pass
                    lane.status = AFCLaneState.NONE
            if self._is_virtual_hub(lane):
                lane._load_state = bool(getattr(lane, 'tool_loaded', False))
            # Before PREP restores the var file a blank lane says nothing about
            # the bay, so hold off. afc.prep_done is the gate rather than
            # _prep_seen: a pool spare claimed after PREP passed it is never
            # prepped, and _prep_seen would then skip it for the whole session.
            if not (getattr(self, "_prep_seen", False)
                    or getattr(getattr(self, "afc", None),
                               "prep_done", False)):
                continue
            # Until the unit answers a scan, the bay record is still the
            # previous spool's: the AMS reports its old record from the moment
            # a new spool goes in, long before the reader sees the new tag.
            # The unit decides the verdict: still working, read a tag, or
            # finished with none.
            verdict = self._scan_verdict(slot)
            # Debug trace: one line per verdict change per slot, with the
            # firmware's seq/res and the record's headline fields.
            try:
                tr = getattr(self, "_trace_verdict", None)
                if tr is None:
                    tr = self._trace_verdict = {}
                if tr.get(slot) != verdict:
                    tr[slot] = verdict
                    self.logger.debug(
                        f"AFC bambu {self.name}: slot {slot} scan verdict -> "
                        f"{verdict} (fw seq={info.get('scan_seq')} "
                        f"res={info.get('scan_res')} | record: "
                        f"mat={info.get('material') or '-'} "
                        f"uid={info.get('rfid_uid') or '-'} "
                        f"present={info.get('present')})")
            except Exception:
                pass
            if verdict in ("read", "notag"):
                # The unit has answered for this bay. That outranks whatever
                # AFC restored at boot, so stop deferring to it.
                self._afc_owned.discard(slot)
            if verdict == "waiting":
                continue                         # it has not answered yet
            if verdict == "notag":
                # It finished and read nothing: defaults, once (the latch),
                # and the hold stays so the previous spool's profile is not
                # put back. The unit will not measure an untagged spool, so any
                # weight the bay still reports is the previous tag's too. A
                # late tag (material AND uid) still beats the no-tag verdict.
                if info.get("material") and info.get("rfid_uid"):
                    # Say why nothing changed, or a scan that visibly ran reads
                    # as a fault. A same-tag read never settles a scan still in
                    # its cycle: a Spoolman bind written mid-cycle aborts the
                    # measurement. _fresh_insert only tags the one-time INFO
                    # line.
                    fi = getattr(self, "_fresh_insert", None) or {}
                    # "info same as last read" is narration, not a veto: the
                    # unit measures anyway, so the hold ends on the unit's
                    # answer (a measurement adopted, or its cycle-end) or
                    # the scan's time cap, never a pass counter.
                    ended = True
                    try:
                        started = (getattr(self, "_scan_t0", None)
                                   or [None] * 8)[slot]
                        bridge = getattr(self, "_bridge", None)
                        if bridge is not None and started is not None:
                            ended = bridge.rfid_cycle_ended_since(
                                started,
                                addr=getattr(self, "dry_dev_addr", None))
                    except Exception:
                        ended = True
                    # One silence, one wait: this hold and the scan verdict
                    # backstop the same quiet unit, so bound it by the
                    # scan's own clock (max of the two, not their sum). The
                    # unit's word still ends it at once.
                    _t0 = (getattr(self, "_scan_t0", None)
                           or [None] * 8)[slot]
                    _now = None
                    try:
                        _now = self.afc.reactor.monotonic()
                    except Exception:
                        _now = None
                    # getattr with a default: tests drive this path with
                    # a plain stand-in object.
                    _cap_s = getattr(self, "SCAN_FALLBACK_CAP", 45.0)
                    _capped = (_t0 is not None and _now is not None
                               and (_now - _t0) > _cap_s)
                    if not ended and not _capped:
                        if fi.get(slot) == 1:
                            fi[slot] = 2
                            self.logger.info(
                                f"AFC bambu {self.name}: {lane.name} -- "
                                f"fresh insert re-read its own tag; the "
                                f"measurement is still coming, holding "
                                f"the lane for the unit's answer")
                        continue
                    # Two exits with opposite meanings: the unit ended its
                    # cycle, or our clock ran out while it was still
                    # working. Log the timeout case explicitly so it is
                    # not mistaken for the unit's verdict, and so a late
                    # result can be told apart from an absent one.
                    if _capped and not ended:
                        self.logger.warning(
                            f"AFC bambu {self.name}: {lane.name} -- "
                            f"gave up waiting after {_cap_s:.0f}s "
                            f"(SCAN_FALLBACK_CAP); the unit had NOT "
                            f"ended its cycle, so anything below is our "
                            f"timer's verdict and not the unit's. A "
                            f"measurement arriving after this is late, "
                            f"not absent.")
                    fi.pop(slot, None)   # settled: by the unit, or by the cap above
                    # The measurement memo lives on the Spoolman
                    # delegate; _held_measurements finds it there.
                    held = afcBambuAMS._held_measurements(self).get(slot)
                    if held:
                        # A measurement from this session is already on
                        # the record: say so rather than "not measured".
                        self.logger.info(
                            f"AFC bambu {self.name}: {lane.name} -- "
                            f"cycle ended with no new read; already "
                            f"measured this session ({held}% held), "
                            f"spool unchanged")
                        self._release_scan_hold(slot)
                        self._scan_notag[slot] = False
                        self._surface_slot_info(lane, info)
                        continue
                    # The bound-tag memo lives on the Spoolman delegate. The
                    # record has a UID here (this branch needs one), so a bay
                    # never bound cannot match.
                    _bound = None
                    for _o in afcBambuAMS._built_measure_objs(self):
                        _bound = (getattr(_o, "_bound_uid", None)
                                  or {}).get(slot)
                        if _bound is not None:
                            break
                    same = (info.get("rfid_uid") == _bound)
                    self.logger.info(
                        f"AFC bambu {self.name}: {lane.name} -- the unit ran "
                        f"its scan and measure and reported no NEW read"
                        f"{' (same spool as last time)' if same else ''}; "
                        f"keeping the tag already on this bay "
                        f"({info.get('material')}"
                        f"{' ' + str(info.get('rfid_uid')).upper()}), "
                        f"so nothing on the lane changes")
                    self._release_scan_hold(slot)
                    self._scan_notag[slot] = False
                    self._surface_slot_info(lane, info)
                    continue
                # Do not call a bay empty while its record is still coming: a
                # boxed unit reads through its calibration ("STEP7:cali read
                # tray N") with none of the auth phrases. The firmware says
                # when it owes a bay a read; wait for it.
                if info.get("reread_pending"):
                    continue
                if not self._scan_notag[slot]:
                    self._scan_notag[slot] = True
                    self._finalize_scan(slot)
                # The hold keeps _surface_slot_info off this bay, but the
                # read-less bind _finalize_scan sent can still attach a spool
                # a measurement is owed to -- a UID-only tag never surfaces.
                afcBambuAMS._apply_remain_weight(self, lane, info)
                continue
            if verdict == "read":
                # The measurement comes after the tag read, and on some
                # models bus traffic during it aborts it. Hold the readout,
                # the apply and the Spoolman round-trip until the unit has
                # finished (_measure_settled logs the wait once).
                if not self._measure_settled(slot, info):
                    continue
                self._release_scan_hold(slot)    # the record is this spool's
                self._scan_notag[slot] = False
                afcBambuAMS._log_tag_readout(self, lane, info, force=True)
            # Measurement adopt. Runs on every verdict, before the "none"
            # branch below: a repeat measure of an unchanged spool is a
            # verdict of "none" (no new tag), but its measurement is still
            # new. It is deduped on the measurement stamp, not the tag.
            #
            # This is also the reliable attribution site: the firmware stamps
            # the measured percent onto the bay whose capacity window produced
            # it. The narration path in get_status keys off a device address
            # both boxed units answer on, so it cannot tell them apart.
            try:
                mseq = info.get("meas_seq")
                mpct = info.get("meas_pct")
                if mseq and mpct is not None:
                    seen = getattr(self, "_meas_seen", None)
                    if seen is None:
                        seen = self._meas_seen = {}
                    # The percent is the test, not the sequence. A boxed
                    # unit (AMS 2) advances meas_seq while leaving meas_pct
                    # stale, so a new seq on the same percent is the record
                    # being re-serialised, not a new measurement; the HT's
                    # percent moves with its seq. Only the HT adopts from
                    # here. A repeat of the same number needs no second
                    # announcement either.
                    # _prev is None only for a stamp this connection watched
                    # appear: whatever a bay carried when the connection
                    # started was recorded by _baseline_meas_stamp, and a
                    # removal forgets it so the next spool's first reading
                    # counts.
                    _prev = seen.get(slot)
                    # A presence flap hands back the stamp its spool left
                    # with. The removal edge forgets the bay's stamp, but the
                    # firmware keeps the record through a flap it never saw
                    # and publishes it again on the re-insert. That is the
                    # departed spool's old measurement, so it is recorded as
                    # seen (like a restart's stamp) and not applied; a stamp
                    # the unit takes from here is adopted as usual.
                    _gone = (getattr(self, "_meas_departed", None)
                             or {}).get(slot)
                    if _prev is None and _gone == (mseq, mpct):
                        seen[slot] = (mseq, mpct)
                        _blg = getattr(self, "_meas_baselined", None)
                        if _blg is None:
                            _blg = self._meas_baselined = set()
                        _blg.add(slot)
                        self.logger.debug(
                            f"AFC bambu {self.name}: slot {slot} carries "
                            f"measurement {mpct}% (seq {mseq}) from before its "
                            f"spool came out; recorded, not applied")
                        _prev = seen[slot]
                    # Exception: a baselined figure was never applied, so a
                    # new seq on the same percent can be a real measurement
                    # that happens to repeat (e.g. a CAPSCAN after a restart).
                    # Adopt it, but only while a capacity window of ours is
                    # open for this bay; anywhere else a new seq on the same
                    # number is still the record being re-serialised.
                    _bl = getattr(self, "_meas_baselined", None) or set()
                    _ours = (_prev is not None and slot in _bl
                             and mseq != _prev[0]
                             and slot in afcBambuAMS._cap_live_pending(self))
                    if _prev is None or _prev[1] != mpct or _ours:
                        _bl.discard(slot)
                        seen[slot] = (mseq, mpct)
                        _took = self._adopt_measured_remain(
                            slot, int(mpct), "physical AMS measurement",
                            seq=mseq)
                        # An answer to a waiting bay that did not come
                        # through the narration. Marked, not closed: the
                        # narration path (get_status) records it as adopted
                        # and hands the bus back, and needs the map intact.
                        # The wait then ends without an "ended without a
                        # measurement" line.
                        _waiting = getattr(self, "_cap_pending", None) or {}
                        if _took and slot in _waiting:
                            _ans = getattr(self, "_cap_answered", None)
                            if _ans is None:
                                self._cap_answered: set = set()
                                _ans = self._cap_answered
                            _ans.add(slot)
            except Exception:
                pass
            if verdict == "none" and slot in self._afc_owned:
                # AFC ownership protects data, not an empty lane. A bay is
                # claimed for AFC only when its lane has material or a
                # spool_id, but the lane can go empty afterwards (e.g.
                # _finalize_scan applies defaults to a bay the bridge has not
                # read yet, as after a bridge restart). Verdict is "none" for
                # anything the host did not command itself, including
                # AFC_BAMBU_CAPSCAN, so a blank owned lane could never be
                # filled from the bay. Re-check here, as _finalize_scan does.
                if (getattr(lane, "material", "")
                        or getattr(lane, "spool_id", None)):
                    self._fill_missing_variant(lane, info)
                    # This branch continues past the only automatic Spoolman
                    # lookup (_surface_slot_info), past the settle of a
                    # measurement owed to a bind and past the summary drain
                    # below -- so a restored lane with no spool gets its tag
                    # looked up here, and the other two run here too.
                    _lookup = getattr(self, "_lookup_unbound", None)
                    if _lookup is not None:
                        _lookup(lane, info)
                        afcBambuAMS._apply_remain_weight(self, lane, info)
                        if any(getattr(_o, "_pending_summary", None)
                               for _o in afcBambuAMS._built_measure_objs(self)):
                            self._drain_spool_summary(slot)
                    continue
                self._afc_owned.discard(slot)   # nothing left to defer to
            self._surface_slot_info(lane, info)
            # A measurement finishes before the record it describes catches up
            # (see _queue_spool_summary). Now that the record has surfaced, the
            # summary can say what is actually in the bay. Held summaries
            # live on the delegate that queued them; _built_measure_objs only
            # returns delegates that already exist, so this builds nothing.
            if any(getattr(_o, "_pending_summary", None)
                   for _o in afcBambuAMS._built_measure_objs(self)):
                self._drain_spool_summary(slot)

    def _baseline_meas_stamp(self, slot: int, info: dict) -> None:
        """
        Record the measurement stamp a bay carries when the connection starts.

        A measurement is applied when it is taken. The firmware keeps each
        bay's last meas_pct/meas_seq in RAM and publishes it in every status
        frame, so after a Klipper restart the first frame hands back a reading
        taken before it. The first frame on a connection that shows a bay's
        stamp therefore records it in _meas_seen and applies nothing: the lane
        keeps what AFC restored from AFC.var.unit, or what its Spoolman spool
        says. Only a stamp that changes after that is adopted.

        Keyed on the first look, not on _scan_primed: the priming timer can
        fire before a slow link has delivered anything, a reclaim keeps it
        set, and a bridge reconnect never clears it.

        Exception: a bay the host was already waiting on when this frame
        arrived (_cap_live_pending: a capscan, a calibrate, or an insert
        window carried across a reconnect). Its stamp may answer that window,
        so it is left for the adopt block to judge. Only a window from before
        this frame counts, which is why _sync_lanes runs this ahead of the
        insert edge: a reclaim's first frame can open insert windows on bays
        the previous unit had empty, and those did not ask for the stamp
        already in the bay.

        What this records is marked in _meas_baselined: it was never applied,
        so the adopt block must not read a repeat of its percent as "already
        on the lane".

        Known gap: a measurement that finishes between two connections, with
        no window of ours open, is recorded and not applied. A CAPSCAN or a
        reseat measures it again.

        :param slot: 0-based AMS slot index
        :param info: the bay's normalized slot info
        """
        looked = getattr(self, "_stamp_looked", None)
        # Fail safe: an object without the set (a stand-in that skipped
        # __init__) starts a look rather than adopting on first sight.
        if looked is None:
            looked = self._stamp_looked = set()
        if slot in looked:
            return
        mseq = info.get("meas_seq")
        # Firmware that publishes no stamp has nothing to baseline yet; keep
        # looking, so a record that gains one is still caught on first sight.
        if mseq is None:
            return
        looked.add(slot)
        mpct = info.get("meas_pct")
        if not (mseq and mpct is not None):
            return
        try:
            if slot in afcBambuAMS._cap_live_pending(self):
                return
        except Exception:
            pass
        seen = getattr(self, "_meas_seen", None)
        if seen is None:
            seen = self._meas_seen = {}
        prev = seen.get(slot)
        seen[slot] = (mseq, mpct)
        bl = getattr(self, "_meas_baselined", None)
        if bl is None:
            bl = self._meas_baselined = set()
        bl.add(slot)
        if prev is None or prev[1] != mpct:
            self.logger.debug(
                f"AFC bambu {self.name}: slot {slot} carries measurement "
                f"{mpct}% (seq {mseq}) from before this connection; recorded, "
                f"not applied -- the lane keeps what it had")

    def _measure_settled(self, slot: int, info: dict) -> bool:
        """
        Whether this model's post-read measurement is done being disturbed.

        True immediately for every model that measures fine while we talk.
        For a model with ``quiet_while_measuring``, True only once the unit
        has given one of its two possible answers:

        * ``meas_seq`` advanced past the value baselined at ``_open_scan`` --
          the measurement landed, and its percent is already on the record.
        * the unit narrated the end of its cycle -- it is finished and no
          measurement is coming.

        Neither is a timer; the scan's own backstop covers a bridge that
        stops talking, so nothing waits here forever.

        :param slot: 0-based AMS slot index on this unit
        :param info: Normalized slot info from bridge_slot_to_info
        :return bool: True when the lane may be written
        """
        try:
            if not self.profile.get("quiet_while_measuring"):
                return True
        except Exception:
            return True
        # measure_on_insert=False: no measurement was asked for, so the
        # "measurement landed" exit would never fire.
        if not getattr(self, "measure_on_insert", False):
            return True
        mseq = info.get("meas_seq")
        base = (getattr(self, "_meas_seq0", None) or {}).get(slot)
        if mseq and mseq != base:
            return True                      # it measured
        started = (getattr(self, "_scan_t0", None) or [None] * 8)[slot] \
            if 0 <= slot < len(getattr(self, "_scan_t0", None) or []) else None
        bridge = getattr(self, "_bridge", None)
        if bridge is not None and started is not None:
            try:
                if bridge.rfid_cycle_ended_since(
                        started, addr=getattr(self, "dry_dev_addr", None)):
                    return True              # finished, with nothing to show
            except Exception:
                return True                  # never block on a broken helper
        # Say it once per slot, so the wait after a visible read is not
        # mistaken for a fault.
        held = getattr(self, "_measure_wait_said", None)
        if held is None:
            held = self._measure_wait_said = set()
        if slot not in held:
            held.add(slot)
            name = next((n for n, s in (getattr(self, "_slot_map", None)
                                        or {}).items() if s == slot), None)
            self.logger.info(
                f"AFC bambu {self.name}: {name or f'slot {slot}'} read its "
                f"tag; holding the lane until the unit finishes measuring "
                f"the spool -- this model stops calibrating if we write "
                f"mid-cycle")
        return False


    def _fill_missing_variant(self, lane: Any, info: dict) -> None:
        """
        Give a boot-restored lane back the variant nothing else can supply.

        The panel renders material + sub_type itself, so a lane reading a
        bare "PLA" is missing its sub_type; the material must not be
        decorated with it.

        AFC's prep restores material, colour, weight and the Spoolman link
        from the var file but not the variant (see _restore_sub_type), and
        Spoolman has no sub_type field -- "Basic"/"Matte"/"Sparkle" lives
        inside the filament's name. The bay's record still has the whole
        string ("PLA Sparkle").

        Only blank fields are filled (colour, variant, vendor, filament
        name), and only when the record's base material is already the
        lane's own -- a bay reporting PETG cannot decorate a PLA lane.
        Nothing AFC restored is overwritten.

        :param lane: The AFC lane object
        :param info: Normalized slot info from bridge_slot_to_info
        """
        # Check all of sub_type, vendor, filament_name and colour: prep may
        # restore sub_type from vars, and a bay's colour lands a beat after
        # its material, so a lane can be complete except for one field.
        if (getattr(lane, "sub_type", "")
                and getattr(lane, "spool_vendor", "")
                and getattr(lane, "filament_name", "")
                and getattr(lane, "color", "")):
            return
        tag_material = info.get("material")
        if not tag_material or tag_material.lower() == "unknown":
            return
        # A UID two units both claim is not evidence of anything: one unit's
        # bay cannot hold another's spool, so a duplicated UID means the record
        # is a copy, not a reading. Refuse it.
        if self._uid_claimed_elsewhere(info.get("rfid_uid")):
            return
        material, sub_type = _split_bambu_material(tag_material)
        if (getattr(lane, "material", "") or "").strip().lower() != \
                material.strip().lower():
            return
        # A lane with no spool is filled only from a record matching its
        # colour: a bay keeps a departed reel's record until something reads
        # the bay, and matching on base material alone would dress the lane
        # as the reel that left. A bound lane's spool identifies the reel, so
        # it keeps the colour fill. (A black tag's lane has no colour; its
        # variant stands in for it.)
        if (getattr(lane, "spool_id", None) in (None, "", 0)
                and not _lane_colour_is_records(lane, info)):
            return
        filled = []
        # Colour first, outside the sub_type gate below: a tag whose material
        # carries no variant ("PLA" alone) still has a colour to fill.
        tag_color = info.get("color")
        if tag_color and not getattr(lane, "color", ""):
            lane.color = tag_color if tag_color.startswith("#") \
                else "#" + tag_color
            filled.append("color " + lane.color)
        if sub_type:
            if not getattr(lane, "sub_type", ""):
                lane.sub_type = sub_type
                filled.append("variant " + sub_type)
            if not getattr(lane, "spool_vendor", ""):
                lane.spool_vendor = BAMBU_BRAND
                filled.append("vendor")
            if not getattr(lane, "filament_name", ""):
                lane.filament_name = build_filament_name(
                    BAMBU_BRAND, material, sub_type)
                filled.append("name")
        # Nothing below runs for a lane that was already complete.
        if not filled:
            return
        # Say what was filled. Prep never reads sub_type back, and restores
        # colour only if it was ever written (a bay whose colour landed after
        # the material may never have saved one); both come from the bay's
        # record here.
        self.logger.info(
            f"AFC bambu {self.name}: {lane.name} -- filling in "
            f"{' and '.join(filled)} from the bay's record ({tag_material})")
        try:
            lane.send_lane_data()
        except Exception:
            pass
        self._save_lane_vars()

    def _restore_sub_type(self, lane: Any) -> None:
        """
        Give a lane back the variant the var file already holds.

        AFC's prep restores material, colour, weight and the Spoolman link and
        never reads sub_type, though get_status writes it on every save, so
        the unit reads its own lanes' variants back out of the var file.

        Only fills a blank: anything already on the lane came from a tag or a
        scan this session and is closer to the spool than a stored string.

        :param lane: The AFC lane object
        """
        if getattr(lane, "sub_type", ""):
            return
        try:
            path = f"{self.afc.VarFile}.unit"
            with open(path) as fh:
                units = json.load(fh)
            saved = (units.get(self.name) or {}).get(lane.name) or {}
            sub = (saved.get("sub_type") or "").strip()
        except Exception:
            return                      # no file, bad JSON: nothing to restore
        if sub:
            lane.sub_type = sub

    def _persisted_lane(self, lane_name: str) -> dict:
        """
        This lane's record as saved in ``VarFile.unit``, straight off disk.

        Unlike the lane object, this does not race. A lane's Spoolman data
        arrives from an async fetch that has usually not landed at scan
        priming, so at boot an empty lane may mean "empty" or "not restored
        yet". The saved file is read synchronously, as AFC_prep reads it.

        :param lane_name: the lane's name, as it appears in the file
        :return dict: the saved record, or {} if there is nothing to read
        """
        try:
            with open(f"{self.afc.VarFile}.unit") as fh:
                units = json.load(fh)
            return (units.get(self.name) or {}).get(lane_name) or {}
        except Exception:
            return {}                   # no file, bad JSON: nothing to go on

    def _save_lane_vars(self) -> None:
        """
        Persist lane state, so a profile survives a Klipper restart.

        Best-effort: a failed save is never worth losing the lane update that
        prompted it.
        """
        try:
            self.afc.save_vars()
        except Exception:
            pass

    def _restore_untagged_defaults(self, claimed_live: bool = False) -> None:
        """
        At startup, give a bay that is present but carries no tag its defaults.

        Spools already in the unit at boot are never re-scanned (see
        ``_maybe_auto_scan``'s priming) -- a scan physically moves filament.
        Applying defaults moves nothing, and without it an untagged bay would
        come back blank after a restart while a tagged bay re-derives its
        profile from the AMS record.

        Only touches a lane that is genuinely empty: a Spoolman link, a restored
        profile, or an AMS record all leave it alone. If a tagged spool went in
        while Klipper was down, ``_surface_slot_info`` overwrites these
        defaults when the record arrives.

        The "genuinely empty" test lives here rather than in
        ``_finalize_scan``. Both apply lane defaults but treat existing data
        oppositely:

            boot restore   the lane has data -> it is the user's, keep
            failed scan    the lane has data -> it is the last spool's, clear
        """
        # Defer to what is on disk: at boot the lane may still be waiting on
        # its Spoolman fetch, but VarFile.unit (see _persisted_lane) already
        # knows whether the user has a record for this bay. If so, the bay is
        # left alone for the normal restore. Defaults are only for a bay
        # nothing knows anything about.
        for slot in range(min(len(self._slots),
                              getattr(self, "unit_slots", len(self._slots)))):
            info = self._slots[slot]
            if not info or not info.get("present") or info.get("material"):
                continue
            lane = self._lane_for_slot(slot)
            if lane is not None:
                if getattr(lane, "spool_id", None) not in (None, "", 0):
                    continue                 # Spoolman-linked -> the user's
                if getattr(lane, "material", None) not in (None, ""):
                    continue                 # restored from vars -> the user's
                # The same question asked of the file, which answers it
                # correctly this early.
                saved = self._persisted_lane(getattr(lane, "name", ""))
                if (saved.get("spool_id") not in (None, "", 0)
                        or saved.get("material") not in (None, "")):
                    continue             # the user has a record -> hands off
            # Spool present and nothing anywhere knows anything about it: no
            # tag record, nothing on the lane, nothing in AFC.var.unit. Give
            # it AFC's defaults rather than leaving a blank lane.
            #
            # Pool units hit this: a lane returned to the pool is
            # unregistered from afc.lanes and unit.lanes
            # (AFC_BridgeBox.deactivate_to_pool), so a save while the unit is
            # unclaimed drops its saved lanes. Tagged bays rebuild from their
            # tags; an untagged bay gets defaults here.
            self._finalize_scan(slot, scanned=False, no_record=True)
            # The defaults are ours, not the user's: a tag the unit hands
            # back after this (an HT's cache, a boxed unit's re-read) is
            # written over them, not held off as restored.
            dfl = getattr(self, "_defaulted_bays", None)
            if dfl is None:
                dfl = self._defaulted_bays = set()
            dfl.add(slot)
            # On a unit claimed live, also hand the bay to the insert path: a
            # spool with no record anywhere arrived while the unit was not
            # watched (unplugged, or an inert pool slot), so the
            # no-rescan-at-startup rule has nothing to protect and would leave
            # it on defaults for ever.
            #
            # Re-arm the edge rather than calling the scan: clearing the
            # baseline makes the next _sync_lanes pass see a 0->1 and run the
            # ordinary insert path with all its guards (in-print skip,
            # tool-loaded hold, insert_pullin, measure_on_insert).
            #
            # Once per bay per connection, latched here: the material test
            # above only prevents a repeat when afc.default_material_type is
            # set, and a scan moves filament.
            #
            # Not at startup: the lane's Spoolman fetch may still be in flight
            # and AFC_prep may not have run, so "no record anywhere" is not
            # trustworthy. A manufactured insert on such a bay would scan it,
            # and an unreadable tag ends in _finalize_scan(scanned=True), which
            # clears the lane and drops its Spoolman link.
            if not claimed_live:
                continue
            ra = getattr(self, "_untagged_rearmed", None)
            if (ra is not None and 0 <= slot < len(ra) and not ra[slot]
                    and 0 <= slot < len(self._prev_present)):
                ra[slot] = True
                self._prev_present[slot] = False
                self._auto_scanned[slot] = False
                self.logger.info(
                    f"AFC bambu {self.name}: slot {slot} came up with a spool "
                    f"and no record of it -- scanning it as a fresh insert")

    def _release_scan_hold(self, slot: int) -> None:
        """
        Close the scan open on ``slot``, so its record surfaces normally again.

        :param slot: 0-based AMS slot index on this unit
        """
        arr = getattr(self, "_scan_t0", None)
        if arr is not None and 0 <= slot < len(arr):
            arr[slot] = None

    def _open_scan(self, slot: int) -> None:
        """
        Mark a scan as commanded for ``slot`` and arm its backstop.

        One timestamp is the entire hold: while it is set, ``_scan_verdict``
        asks the unit what happened and ``_sync_lanes`` surfaces nothing for
        this bay, so ``_surface_slot_info`` cannot put the old spool's
        profile straight back from the pre-scan record.

        Every scan goes through here, auto or manual, so both end the same two
        ways: the unit read a tag, or the lane gets defaults.

        :param slot: 0-based AMS slot index on this unit
        """
        self._scan_notag[slot] = False   # asking again -- the old answer is void
        # From here this bay has been looked at on this connection, so what
        # the unit reports next is a real observation rather than the cached
        # record the lane was saved from. Releases the boot hold in
        # _surface_slot_info for this bay only.
        try:
            self._scanned_bays.add(slot)
        except AttributeError:
            self._scanned_bays = {slot}
        # A scan gets its own Spoolman lookup: the one-shot is re-armed, and
        # a recorded miss, pending retry or refusal for the bay's current tag
        # is forgotten, since the answer may have changed (spool added to
        # Spoolman, or the server back up).
        try:
            (getattr(self, "_spoolman_latched", None) or set()).discard(slot)
            for _d in ("_lookup_retry", "_lookup_refused"):
                (getattr(self, _d, None) or {}).pop(slot, None)
            _cur = (self._slots[slot]
                    if 0 <= slot < len(getattr(self, "_slots", []) or [])
                    else None) or {}
            _uid = str(_cur.get("rfid_uid") or "").lower()
            if _uid:
                for _o in afcBambuAMS._built_measure_objs(self):
                    _miss = getattr(_o, "_spoolman_no_match", None)
                    if _miss:
                        for _m in [m for m in _miss
                                   if str(m).lower() == _uid]:
                            _miss.discard(_m)
        except Exception:
            pass
        # A new scan gets a new "holding for the measurement" line if it waits.
        try:
            (getattr(self, "_measure_wait_said", None) or set()).discard(slot)
        except Exception:
            pass
        t0 = getattr(self, "_scan_t0", None)
        if t0 is None or not (0 <= slot < len(t0)):
            return
        try:
            t0[slot] = self.afc.reactor.monotonic()
        except Exception:
            t0[slot] = None
            return
        # Baseline the firmware's per-bay verdict counter. This scan's verdict
        # is "the seq advanced past this value", which still fires when the
        # same spool is re-inserted and the record comes back identical.
        try:
            seq0 = getattr(self, "_scan_seq0", None)
            if seq0 is None:
                seq0 = self._scan_seq0 = {}
            info = (self._slots[slot]
                    if 0 <= slot < len(getattr(self, "_slots", []) or [])
                    else None) or {}
            seq0[slot] = info.get("scan_seq")
        except Exception:
            pass
        # The cycle-end arm (see _scan_verdict) is per scan: a leftover armed
        # flag would let this scan's first cycle-end conclude immediately.
        try:
            (getattr(self, "_cycle_end_seen", None) or {}).pop(slot, None)
        except Exception:
            pass
        # A new scan invalidates the bay's old measurement, including a
        # figure still owed to a bind (_bind_owed) or to its material
        # (_convert_owed): what was measured before may describe a different
        # spool. If this scan measures, the fresh value lands; if not, the
        # lane keeps what it has.
        try:
            for _o in afcBambuAMS._built_measure_objs(self):
                (getattr(_o, "_measured_remain", None) or {}).pop(slot, None)
                (getattr(_o, "_meas_seq_seen", None) or {}).pop(slot, None)
                (getattr(_o, "_bind_owed", None) or {}).pop(slot, None)
                (getattr(_o, "_convert_owed", None) or {}).pop(slot, None)
        except Exception:
            pass
        # Baseline the measurement counter the same way, for the models that
        # wait for it before touching the lane (see quiet_while_measuring).
        try:
            mseq0 = getattr(self, "_meas_seq0", None)
            if mseq0 is None:
                mseq0 = self._meas_seq0 = {}
            info = (self._slots[slot]
                    if 0 <= slot < len(getattr(self, "_slots", []) or [])
                    else None) or {}
            mseq0[slot] = info.get("meas_seq")
        except Exception:
            pass
        # One backstop, for a bridge that stops talking. On a working bus the
        # scan resolves in _sync_lanes as soon as the firmware publishes its
        # verdict (the firmware closes its window on the unit's cycle-end).
        try:
            self.afc.reactor.register_callback(
                lambda et, s=slot: self._scan_timeout(s),
                t0[slot] + self.SCAN_VERDICT_CAP + 1.0)
        except Exception:
            pass

    def _scan_timeout(self, slot: int) -> None:
        """
        Resolve a scan that outlived its backstop, for a unit gone silent.

        A no-op unless the scan is still open, which it will not be whenever
        status frames kept flowing -- ``_sync_lanes`` resolves it long first.

        :param slot: 0-based AMS slot index on this unit
        """
        verdict = self._scan_verdict(slot)
        if verdict == "read":
            self._release_scan_hold(slot)
        elif verdict == "notag" and not self._scan_notag[slot]:
            self._scan_notag[slot] = True
            self._finalize_scan(slot)

    def _scan_verdict(self, slot: Optional[int]) -> str:
        """
        What the unit has said about the scan open on ``slot``.

        The scan state machine. The module commands a scan, the unit carries
        it out and reports what happened; this reads that report back. It
        never inspects the record's content or guesses from a clock (except
        the backstop caps): only the unit knows whether it read a tag.

        ``"none"``     no scan is open; the record is just the bay's, use it.
        ``"waiting"``  commanded, no answer yet. The bay still reports the
                       PREVIOUS spool's record until the reader sees the new
                       tag, so nothing may be surfaced during this.
        ``"read"``     the unit narrated a successful read. Its record is now
                       this spool's -- even when it is byte-for-byte the old
                       one, which is exactly what re-inserting the same spool
                       produces.
        ``"notag"``    the unit reported the end of its cycle without a read
                       (confirmed on a later frame; see below).

        Scoped to this unit's device address, so a chain-mate's scan can
        neither answer for this one nor keep it waiting.

        :param slot: 0-based AMS slot index on this unit, or None when unknown
        :return str: one of "none", "waiting", "read", "notag"
        """
        if slot is None:
            return "none"
        t0arr = getattr(self, "_scan_t0", None)
        if not t0arr or not (0 <= slot < len(t0arr)):
            return "none"
        started = t0arr[slot]
        if started is None:
            return "none"
        # The firmware's per-bay verdict wins when present: narration stamps
        # are per address class, so a sibling boxed unit's "read success"
        # could answer for this bay. scan_seq moving past the _open_scan
        # baseline is this scan's answer; a firmware without scan_seq falls
        # through to the stamps.
        try:
            info = (self._slots[slot]
                    if 0 <= slot < len(getattr(self, "_slots", []) or [])
                    else None) or {}
            seq = info.get("scan_seq")
            if seq is not None:
                base = (getattr(self, "_scan_seq0", None) or {}).get(slot)
                if seq != base:
                    # 1 = read; 2 (foreign) and 3 (no tag) both finalize to
                    # defaults -- _finalize_scan's operator message already
                    # tells a third-party tag from an empty reader.
                    if info.get("scan_res") == 1:
                        return "read"
                    # Arm, then confirm on a later frame, as in the
                    # narration path below: an AMS 2 can publish res=3
                    # "no tag" and carry the tag in the record a couple of
                    # seconds later.
                    pend = getattr(self, "_cycle_end_seen", None)
                    if pend is None:
                        pend = self._cycle_end_seen = {}
                    if not pend.get(slot):
                        pend[slot] = True
                        return "waiting"
                    return "notag"
                # Unresolved: wait for the firmware, not for the narration
                # stamps below (they cross-credit siblings) nor for the
                # shorter SCAN_FALLBACK_CAP (the firmware always resolves).
                # Only a bridge whose frames stopped entirely can strand the
                # seq; SCAN_VERDICT_CAP covers that.
                try:
                    if (self.afc.reactor.monotonic() - started
                            >= self.SCAN_VERDICT_CAP):
                        return "notag"
                except Exception:
                    pass
                return "waiting"
        except Exception:
            pass
        bridge = getattr(self, "_bridge", None)
        if bridge is None:
            return "notag"           # nothing can answer; do not wait for it
        dev = getattr(self, "dry_dev_addr", 0) or None
        try:
            if bridge.rfid_read_succeeded_since(started, addr=dev):
                return "read"
            if bridge.rfid_cycle_ended_since(started, addr=dev):
                # A cycle end is not a verdict on its own. On a boxed AMS the
                # tag record surfaces after the capacity calibration finishes,
                # so "STEP7:cali end" can arrive while the read is still on
                # its way. The first sighting only arms the verdict; it is
                # confirmed on a later pass (one status frame), so a read
                # landing in that window wins -- the checks above run first.
                # SCAN_FALLBACK_CAP below still bounds a unit that goes
                # silent.
                pend = getattr(self, "_cycle_end_seen", None)
                if pend is None:
                    pend = self._cycle_end_seen = {}
                if not pend.get(slot):
                    pend[slot] = True
                    return "waiting"
                # Confirmed: the record has had its frame and no read
                # narration arrived in it.
                #
                # A material in the record does not count as a read: it may
                # be the previous spool's leftover (see test_a_leftover_
                # material_no_longer_counts_as_a_tag_that_read). The verdict
                # only reports what the unit said; _sync_lanes decides what
                # to do when the record holds a tag.
                return "notag"
        except Exception:
            return "notag"
        # Backstop only -- see SCAN_FALLBACK_CAP. Reached only when the unit
        # says nothing at all.
        try:
            if self.afc.reactor.monotonic() - started >= self.SCAN_FALLBACK_CAP:
                return "notag"
        except Exception:
            pass
        return "waiting"

    def _lane_for_slot(self, slot: int) -> Optional[Any]:
        """
        Return the lane mapped to ``slot`` on this unit, or None.

        :param slot: 0-based AMS slot index on this unit
        :return Optional[Any]: the mapped lane, or None if the slot is unmapped
        """
        smap = getattr(self, "_slot_map", None) or {}
        lanes = getattr(self, "lanes", None) or {}
        return next((lanes.get(n) for n, sl in smap.items()
                     if sl == slot and n in lanes), None)

    # One profile per model; every model decision lives here. AMS 1 measures
    # 12 s after its "finish" phrase, AMS 2 reads silently through its
    # calibration, the HT emits "read success" on attempts that retry and
    # answers a plain read from its flash cache. Add a model by adding a row.
    _PROFILES = {
        "ams1": {
            "fw_model": 0,
            "commands_scan": True,   # the module asks; the unit answers
            "pre_read_safe": True,   # a plain 0x211 is this bay's own record
            "measure_route": "narration",   # narrates "odom C:..,P:NN%"
            "quiet_while_measuring": False,  # measures with us talking
            "slots": 4,
        },
        "ams2": {
            "fw_model": 1,
            "commands_scan": True,
            "pre_read_safe": True,
            "measure_route": "narration",
            # Do not touch an AMS 2 between its read and its measurement:
            # applying the tag at "read success,valid" aborts its
            # calibration. The apply waits for the measurement or the unit's
            # own cycle-end.
            "quiet_while_measuring": True,
            "slots": 4,
        },
        "ht": {
            "fw_model": 2,
            "commands_scan": False,  # FIRMWARE-armed on the insert edge
            "pre_read_safe": False,  # a plain read serves the flash cache
            "measure_route": "narration",
            # The HT (like AMS 1) measures fine with the apply going out
            # mid-cycle, so it does not wait.
            "quiet_while_measuring": False,
            "slots": 1,
        },
    }

    @property
    def profile(self) -> dict:
        """This unit's model profile -- the single source of model behaviour."""
        return self._PROFILES.get(getattr(self, "ams_model", "ams2"),
                                  self._PROFILES["ams2"])

    def _send_unit_model(self, bridge: Any) -> None:
        """
        Tell the firmware which machine this unit is.

        Until it knows, it treats a unit as an AMS 1 and judges its narration
        by AMS 1's vocabulary. Sent at every connect, beside the HT flag, so
        it survives a Pico reboot.

        :param bridge: the BambuBridge to send on; ignored when None
        """
        if bridge is None:
            return
        # Everything getattr'd: tests use duck-typed stand-ins, and this send
        # must never break a connect.
        try:
            prof = afcBambuAMS._PROFILES.get(
                getattr(self, "ams_model", "") or "ams2",
                afcBambuAMS._PROFILES["ams2"])
            bridge.send({"cmd": "model",
                         "unit": int(getattr(self, "ams_index", 0) or 0),
                         "m": int(prof["fw_model"])})
        except Exception:
            pass

    def _is_ht(self) -> bool:
        """True if this unit is an AMS HT (device 0x1800). The HT scans its RFID
        itself on its preload switch, so the firmware -- not the module -- drives
        the scan (armed on the insert edge)."""
        # Answered from the model profile, the single source of truth for the
        # model. getattr, because test stand-ins carry only ams_model.
        prof = afcBambuAMS._PROFILES.get(
            getattr(self, "ams_model", "") or "",
            None)
        if prof is not None:
            return prof["fw_model"] == 2
        return bool(getattr(self, "has_heater", False)) and \
            getattr(self, "dry_dev_addr", 0) == 0x1800

    def _send_ht_flag(self, bridge: Any) -> None:
        """
        Tell the firmware whether this unit is an AMS HT, so it arms the RFID
        scan on the slot's insert edge (device 0x1800).

        Harmless for a boxed AMS. Also enables the dense-0F follower hold for
        an HT, which holds mode:4 smoothly under a poop without a re-arm tick
        or feed starvation. Sent at every connect so it survives a Pico
        reboot.

        :param bridge: the BambuBridge to send on; ignored when None
        """
        if bridge is None:
            return
        try:
            is_ht = self._is_ht()
            bridge.send({"cmd": "htunit", "unit": self.ams_index,
                         "on": 1 if is_ht else 0})
            if is_ht and getattr(self, "ht_0f_hold", True):
                bridge.send({"cmd": "ht0fhold", "on": 1})
            # measure_on_insert lives in the firmware because cap_open is the
            # single entry to the measurement window for every unit type --
            # an HT reaches it from ht_scan_arm() on the insert edge, without
            # the module. Re-sent at connect, like the HT flag above.
            bridge.send({"cmd": "capen", "unit": self.ams_index,
                         "on": 1 if getattr(self, "measure_on_insert", False)
                         else 0})
            # Class registry hint. Class addressing is live by default in the
            # firmware: HTs enroll into 0x80-0x87 and boxed units into
            # 0x00-0x03 (s_classaddr, addr_of()). The firmware derives the
            # class from the bus: s_autoht probes an unclassified unit on
            # op37/0x1800 (only an AMS HT answers), relocates it into index
            # 4..11, binds it and persists that (bambubus.c).
            #
            # Pushing a known HT's UID from printer.cfg lets the firmware skip
            # the probe and place it correctly on the first enrollment after
            # a cold start. Without it the unit still lands in the right
            # range, one probe cycle later.
            uid = getattr(self, "unit_uid", None)
            if is_ht and uid and len(str(uid).strip()) == 24:
                bridge.send({"cmd": "htuid", "uid": str(uid).strip().upper()})
            if self.bus_serial:
                bridge.send({"cmd": "serial", "s": self.bus_serial[:15]})
            self._send_bindings(bridge)
            self._send_rc_span(bridge)
        except Exception:
            pass

    def _send_bindings(self, bridge: Any) -> None:
        """
        Pin every configured unit's UID to a stable array index.

        A real printer keeps one UID at one bus id permanently. Without
        pinning, indices follow announce order, which can shuffle across
        reboots or put two units on the same index after a relink.

        The order is deterministic and comes from config, not the bus: boxed
        units first, then HTs, each sorted by bind rank then name, numbered
        0..N-1. Class placement (boxed 0-3, HT 4-11) is left to the firmware.

        Sent by every unit, identically -- each computes the same table, so it
        does not matter which one gets there first, and a unit whose bridge
        reconnects re-seeds it.

        :param bridge: the bridge to send on
        """
        if bridge is None:
            return
        try:
            units = []
            for name, unit in self.printer.lookup_objects("AFC_BambuAMS"):
                uid = (getattr(unit, "unit_uid", "") or "").strip().upper()
                if len(uid) != 24:
                    continue                  # nothing to pin it by
                try:
                    is_ht = bool(unit._is_ht())
                except Exception:
                    is_ht = False
                try:
                    model = int(unit.profile["fw_model"])
                except Exception:
                    model = 1             # AMS 2: the module's own default
                rank = int(getattr(unit, "_bind_rank", 0))
                units.append((is_ht, rank, name, uid, model))
            if not units:
                return
            # Plain sequential order: boxed first, then HTs, numbered 0..N-1.
            # Not the class layout (boxed 0-3 / HT 4-11): with class
            # addressing off, index 4 would map to bus id 0x04, which no real
            # printer uses. The firmware does the class placement when class
            # addressing is on; the host owns only the order.
            units.sort()          # boxed first, then configured ams_index
                                  # (frozen at init), name as the tiebreak
            for i, (_, _, _, uid, model) in enumerate(units):
                if i < 12:
                    # The model rides with the binding. The "model" command
                    # takes an index and is only sent at connect, so between
                    # Pico power-up and Klipper attaching every unit would be
                    # judged by AMS 1's vocabulary. Keyed by UID, the firmware
                    # can restore it from flash and have it right at the
                    # first enrollment.
                    bridge.send({"cmd": "bind", "uid": uid, "idx": i,
                                 "m": model})
            # Persist, so the next power-up does not need telling. The
            # firmware compares before writing, so a matching record touches
            # no flash; it writes only on a fresh Pico or a config change.
            bridge.send({"cmd": "idsave"})
        except Exception:
            pass

    def _send_rc_span(self, bridge: Any) -> None:
        """
        Tell the firmware how much of the address space to roll-call.

        A real printer walks all twelve ids continuously -- 4 boxed
        (0x00-0x03) and 8 HT (0x80-0x87) -- which is how it finds a
        hot-plugged unit within a second. Each empty id costs the full 8 ms
        reply timeout. A unit outside a reduced span is never discovered.

        Only sent when rollcall_span_boxed / rollcall_span_ht is set; 0 means
        "all of that class", the printer-faithful behaviour.

        :param bridge: the bridge to send on
        """
        if bridge is None:
            return
        try:
            boxed = getattr(self, "rollcall_span_boxed", None)
            ht = getattr(self, "rollcall_span_ht", None)
            # Opt-in only: with neither option set nothing is sent, and the
            # firmware keeps its default of all twelve ids.
            if boxed is None and ht is None:
                return
            bridge.send({"cmd": "rcspan",
                         "boxed": int(boxed or 0), "ht": int(ht or 0)})
        except Exception:
            pass

    def _send_mc_addr(self, bridge: Any) -> None:
        """
        Point this unit's MC poll set at its own device and id.

        The firmware's default poll frames are addressed to 0x0700 with
        payload byte 0x01. A real printer addresses every poll to the unit's
        own device with an id of the unit's bus address -- 0x1800/0x00 for a
        lone HT, 0x0700/0 for a lone boxed AMS. The address is what lets two
        units of the same class share a wire; without it an HT never follows
        or narrates on demand.

        :param bridge: The bridge to notify
        """
        if bridge is None:
            return
        # The unit's bus address, derived the way the firmware derives it
        # (see mc_id_for_index). mc_id_base is an additive override and 0 for
        # every shipped model.
        pay = (self.mc_ams_id if self.mc_ams_id >= 0
               else (self.mc_id_base | mc_id_for_index(self.ams_index)))
        try:
            bridge.send({"cmd": "mcaddr", "unit": self.ams_index,
                         "addr": int(self.mc_dev_addr), "pay": int(pay)})
        except Exception:
            pass




    def _reconcile_empty_bays(self) -> None:
        """
        Clear any lane whose bay the unit reports empty, once at startup.

        AFC restores lanes from saved vars, so a spool pulled while Klipper
        was down would leave its material, colour and Spoolman link on the
        lane; there is no removal edge because the bay was already empty when
        watching started.

        The unit's presence bits are the truth here, so this runs at priming --
        after the firmware has reported presence and AFC has restored the
        lanes.
        """
        try:
            for slot in range(min(getattr(self, "unit_slots", 0),
                                  len(self._prev_present))):
                info = (self._slots or [None] * self.SLOTS_PER_UNIT)[slot]
                if info and info.get("present"):
                    continue
                lane = self._lane_for_slot(slot)
                if lane is None:
                    continue
                if (getattr(lane, "material", None)
                        or getattr(lane, "spool_id", None) not in (None, "", 0)):
                    self.logger.info(
                        f"AFC bambu {self.name}: bay {slot + 1} is empty but "
                        f"{lane.name} still held filament data -- clearing it")
                    self._clear_lane_filament(lane)
                    self._unbind_spool(lane)
                # Forget this slot's Spoolman miss: a different spool (or the
                # same one after being added to Spoolman) must get a fresh
                # lookup rather than inheriting "we already asked".
                self._forget_spoolman_miss(slot)
        except Exception as e:
            self.logger.debug(
                f"AFC bambu {self.name}: empty-bay reconcile skipped: {e}")

    def _scan_in_flight(self, slot: int) -> bool:
        """
        Whether a scan is currently running for this slot.

        Used to ignore the presence flap a scan causes in its own bay -- the
        filament is fed past the reader and retracted, which takes it off the
        bay switch. Bounded by SCAN_MOTION_QUIET_S so a stuck scan cannot mask
        a real removal forever.

        :param slot: 0-based AMS slot index
        :return bool: True while this slot's scan is still within its window
        """
        # Not _scan_t0: that means "waiting for a tag" and is cleared when the
        # read succeeds, before the unit finishes retracting the filament off
        # the bay switch. The motion guard needs its own clock, cleared by the
        # unit's cycle-end or by expiry.
        t0s = getattr(self, "_scan_motion_t0", None)
        if not t0s or not (0 <= slot < len(t0s)):
            return False
        started = t0s[slot]
        if not started:
            return False
        # The unit says when it is done. Every model announces the end of its
        # scan/measure cycle -- "Calibration rst:0" (HT), "odom calib success
        # exit 0" (AMS 1), "STEP7:cali end" (AMS 2) -- so wait for that rather
        # than a timer: too short and the unit's own retract re-triggers the
        # scan; too long and a real removal goes unnoticed.
        try:
            getend = getattr(self._bridge, "last_scan_end", None)
            ended = getend() if callable(getend) else None
            if ended is not None and ended >= started:
                t0s[slot] = None        # the unit finished; stop guarding
                return False
        except Exception:
            pass
        # Backstop only, for a unit that never announces an end -- a scan that
        # dies mid-cycle must not gate this bay's presence forever.
        try:
            now = self.afc.reactor.monotonic()
        except Exception:
            return False
        if (now - started) >= self.SCAN_MOTION_QUIET_S:
            t0s[slot] = None
            return False
        return True

    # Motion states any AFC lane can be in while filament is actually moving.
    # A scan must not start against any of these (see _afc_motion_busy).
    _AFC_BUSY_STATES = ("Tool Loading", "Tool Unloading", "HUB Loading",
                        "Ejecting", "Calibrating", "Infinite Runout")

    def _afc_motion_busy(self) -> bool:
        """
        Is a load/unload in flight anywhere in AFC -- any lane, any unit?

        A tag scan does blocking bus work for tens of seconds. Run beside a
        load it starves Klipper's reactor until the toolhead MCU misses its
        timers ("Timer too close" shutdown).

        ``in_print()`` misses a bare TOOL_LOAD outside a print, and
        ``_unit_tool_loaded(self)`` only asks about this unit, while the
        hazard is filament moving on any unit. So every lane's status is
        checked.

        An AFC that cannot answer is treated as busy: a false busy only
        defers a scan (the caller replays it); a false idle risks the
        shutdown.

        :return bool: True while any lane is loading, unloading or ejecting
        """
        afc = getattr(self, "afc", None)
        if afc is None:
            return False                      # no AFC at all: nothing to hit
        try:
            if getattr(afc, "in_toolchange", False):
                return True
            for ln in (getattr(afc, "lanes", None) or {}).values():
                if str(getattr(ln, "status", "")) in self._AFC_BUSY_STATES:
                    return True
        except Exception:
            return True                       # cannot tell -> do not move
        return False

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
        # Which bays this connection has seen occupied: only a removal of one
        # of those is a removal this connection watched (see _present_seen).
        seen_in = getattr(self, "_present_seen", None)
        if seen_in is None:
            seen_in = self._present_seen = set()
        if present:
            seen_in.add(slot)
        # A scan moves the filament past the bay switch, so treating that as a
        # removal loops scan -> REMOVED -> INSERTED -> scan forever. While a
        # scan is in flight for this slot, track presence silently (bounded by
        # _scan_in_flight's own backstop).
        inflight = getattr(self, "_scan_in_flight", None)
        if callable(inflight) and inflight(slot):
            self._prev_present[slot] = present
            return
        # Nor while the unit is moving filament for us: the presence line
        # flickers during an unload, and a scan started mid-retract stalls
        # the reactor. Presence is tracked silently, as above.
        if (getattr(self, "_unload_in_progress", False)
                or getattr(self, "_load_in_progress", False)):
            self._prev_present[slot] = present
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
            # A physical insert means a measurement is coming, whatever the
            # tag says: on a reinsert the unit reports "info same as last
            # read" and still measures. This flag only selects the one-time
            # "fresh insert" INFO line in _sync_lanes; the same-tag hold
            # itself applies to every scan.
            fi = getattr(self, "_fresh_insert", None)
            if fi is None:
                fi = self._fresh_insert = {}
            fi[slot] = 1
            # A fresh insert deserves a fresh attempt: clear the once-per-
            # insert measure latch on the presence 0->1 edge, or a measurement
            # lost mid-way (a false fault, say) silently skips the next spool.
            if getattr(self, "_auto_cali_slot", None) == slot:
                self._auto_cali_slot = None
        elif was_present and not present:
            self.logger.info(
                f"AFC bambu {self.name}: spool REMOVED from slot {slot} "
                f"(AMS bay {slot + 1})")
        if not present:
            # Removal edge: clear the slot's cached profile so the previous
            # spool's material/color doesn't linger, and so the next insert reads
            # fresh (the AMS HT never re-reads on its own, so its old tag would
            # be reapplied on a swap). Spoolman-linked lanes stay
            # authoritative.
            if was_present:
                # The held measurement and its identity leave with the spool,
                # so a new spool in this bay establishes its own first
                # reading. Cleared on every object that holds these memos
                # (the measured percent lives on the Spoolman delegate).
                #
                # A removal of a bay this connection saw occupied is recorded
                # first, with the stamp the bay carried: it cleared the lane
                # (_boot_hold, _defaults_until_read), and a flap hands that
                # stamp back (the adopt block in _sync_lanes).
                if slot in seen_in:
                    try:
                        rb = getattr(self, "_removed_bays", None)
                        if rb is None:
                            rb = self._removed_bays = set()
                        rb.add(slot)
                        cb = getattr(self, "_cleared_bays", None)
                        if cb is None:
                            cb = self._cleared_bays = set()
                        cb.add(slot)
                        # Prefer the stamp on this frame's record: it is the
                        # one a flap hands back. _meas_seen holds the pair
                        # last adopted, whose sequence can lag the record's
                        # after a repeat reading of the same percent. If the
                        # firmware saw the removal and cleared the percent,
                        # fall back to the last pair seen.
                        stamp = None
                        if (info.get("meas_seq")
                                and info.get("meas_pct") is not None):
                            stamp = (info.get("meas_seq"),
                                     info.get("meas_pct"))
                        if stamp is None:
                            stamp = (getattr(self, "_meas_seen", None)
                                     or {}).get(slot)
                        dep = getattr(self, "_meas_departed", None)
                        if dep is None:
                            dep = self._meas_departed = {}
                        if stamp is not None:
                            dep[slot] = stamp
                        else:
                            dep.pop(slot, None)
                    except Exception:
                        pass
                # A lookup Spoolman did not answer, or a match refused, was
                # about the spool that left, and lane defaults waiting on the
                # insert were for a spool that did not stay.
                for _d in ("_lookup_retry", "_lookup_refused",
                           "_defaults_due"):
                    (getattr(self, _d, None) or {}).pop(slot, None)
                try:
                    for _o in afcBambuAMS._built_measure_objs(self):
                        for _d in ("_measured_remain", "_meas_seq_seen",
                                   "_meas_seen", "_bind_owed",
                                   "_convert_owed"):
                            (getattr(_o, _d, None) or {}).pop(slot, None)
                        (getattr(_o, "_meas_baselined", None)
                         or set()).discard(slot)
                except Exception:
                    pass
                lane = self._lane_for_slot(slot)
                if lane is not None:
                    # Clear regardless of a Spoolman binding: a binding to an empty bay is
                    # stale by definition and would block the next real tag from
                    # applying. A re-insert re-binds from the tag.
                    self._clear_lane_filament(lane)
                    self._unbind_spool(lane)
                # Drop any summary still waiting on the departed spool's
                # record, and which tag this bay's binding was made from (so
                # the next spool does not look "already bound by this tag").
                # On every holder, as above.
                for _o in afcBambuAMS._built_measure_objs(self):
                    for _d in ("_pending_summary", "_bound_uid"):
                        _memo = getattr(_o, _d, None)
                        if isinstance(_memo, dict):
                            _memo.pop(slot, None)
                # And the Spoolman-miss memo, so a spool added to Spoolman
                # after a missed lookup can bind without a Klipper restart.
                # (Unbound call: the removal edge runs on duck-typed stand-ins
                # in tests, and the method body getattrs everything.)
                afcBambuAMS._forget_spoolman_miss(self, slot)
                # Re-arm the one-shot: the next spool in this bay gets its
                # own single lookup (OpenAMS re-arms the same way).
                try:
                    self._spoolman_latched.discard(slot)
                except Exception:
                    pass
                # Persist the removal, or a restart before the next save would
                # restore the departed spool's record from the var file.
                # Best-effort: this edge must never raise.
                try:
                    self._save_lane_vars()
                except Exception:
                    pass
            self._auto_scanned[slot] = False        # reinsertion re-scans
            ra = getattr(self, "_untagged_rearmed", None)
            if ra is not None and 0 <= slot < len(ra):
                ra[slot] = False                    # and a new spool is a new question
            self._scan_notag[slot] = False          # a new spool is a new answer
            defer = getattr(self, "_scan_defer", None)
            if defer is not None and 0 <= slot < len(defer):
                defer[slot] = False                 # nothing left to replay
            self._release_scan_hold(slot)
            return
        # A scan held back (a lane on this unit at the toolhead, or AFC
        # busy). Replay it once that clears -- the edge that would have
        # triggered it has passed.
        defer = getattr(self, "_scan_defer", None)
        if defer is not None and 0 <= slot < len(defer) and defer[slot]:
            if _unit_tool_loaded(self):
                return
            # Re-check the same conditions as the hold, or the deferred scan
            # would fire into the next load.
            if afcBambuAMS._afc_motion_busy(self):
                return
            afc_d = getattr(self, "afc", None)
            try:
                if afc_d is not None and afc_d.function.in_print():
                    return
            except Exception:
                pass
            defer[slot] = False
            self.logger.info(
                f"AFC bambu {self.name}: unit is free -- scanning the spool in "
                f"slot {slot} that was held back")
            afcBambuAMS._start_tag_scan(self, slot, info)
            return
        # Lane defaults for an insert during a print (below), once the bay has
        # stayed occupied past a flap. The record is looked at as it is now,
        # so a read that landed in the meantime is what the lane gets instead.
        due = (getattr(self, "_defaults_due", None) or {}).get(slot)
        if due is not None and _mono(getattr(self, "afc", None)) >= due:
            self._defaults_due.pop(slot, None)
            afcBambuAMS._defaults_until_read(self, slot, info)
        if was_present:
            return
        scanned_once = getattr(self, "_auto_scanned", None) or []
        if 0 <= slot < len(scanned_once) and scanned_once[slot]:
            return
        afc = getattr(self, "afc", None)
        try:
            printing = afc is not None and afc.function.in_print()
        except Exception:
            printing = False
        if printing:
            # Never move filament mid-print: no scan, and none deferred. The
            # lane waits for a read on AFC's defaults when the removal cleared
            # it and the record cannot say what went in -- but only after
            # DEFAULTS_SETTLE_S, since a single stray frame can report an
            # emptied bay occupied.
            due = getattr(self, "_defaults_due", None)
            if due is None:
                due = self._defaults_due = {}
            due[slot] = _mono(afc) + getattr(
                self, "DEFAULTS_SETTLE_S", afcBambuAMS.DEFAULTS_SETTLE_S)
            return
        if not self.auto_scan or self._bridge is None:
            return
        # A lane on this unit is at the toolhead: hold the scan, since a tag
        # read pulses the follower for the whole unit. Held, not dropped:
        # _scan_defer replays it once the unit is free. An HT is exempt, its
        # scan is firmware-driven and it has one lane.
        if not self._is_ht() and _unit_tool_loaded(self):
            self._auto_scanned[slot] = True
            defer = getattr(self, "_scan_defer", None)
            if defer is not None and 0 <= slot < len(defer):
                defer[slot] = True
            # The insert still takes effect: hold the scan, not the lane. Apply
            # defaults now; _surface_slot_info overwrites them directly when
            # the deferred or manual scan finally reads the tag.
            self._finalize_scan(slot, scanned=False)
            self.logger.info(
                f"AFC bambu {self.name}: spool in slot {slot} -- applied lane "
                f"defaults and held the SCAN: a lane on this unit is loaded to "
                f"the toolhead and a scan would move filament on it. It scans "
                f"itself when that lane comes out, or run AFC_BAMBU_SCAN "
                f"LANE=<lane> whenever you like")
            # Still pull the new spool in (motion only, no scan) the way a
            # real printer does: drop the follower, prime the bay up to the
            # hub, re-engage. Never during a print.
            self._maybe_insert_pullin(slot)
            return
        # Nor while anything anywhere is moving filament (see
        # _afc_motion_busy). Applies to the HT too: arming its scan still
        # takes the bus. Deferred, not dropped -- replayed by the branch
        # above once AFC is idle.
        if afcBambuAMS._afc_motion_busy(self):
            self._auto_scanned[slot] = True
            defer = getattr(self, "_scan_defer", None)
            if defer is not None and 0 <= slot < len(defer):
                defer[slot] = True
            self.logger.info(
                f"AFC bambu {self.name}: spool in slot {slot} will be scanned "
                f"when AFC is idle -- a lane is loading or unloading and a "
                f"scan now would take the bus out from under it")
            return
        afcBambuAMS._start_tag_scan(self, slot, info)

    def _maybe_insert_pullin(self, slot: int) -> None:
        """
        Motion-only pull-in of a bay inserted while a lane is loaded (idle only).

        A boxed AMS holding the follower (mode:4/state:4) sets ``preload_disable``
        and does NOT run its autonomous preload on the insert edge, so the new
        spool sits at the bay switch un-staged. When the machine is idle (NEVER
        during a print -- the loaded lane must keep its follower to feed), briefly
        drop the follower so the unit relaxes to state:0, prime the new bay up to
        the hub with ``bb_prime`` (feeder-only, RFID reader dormant -- no scan, no
        measure), then re-engage the loaded lane's follower. The tag scan stays
        deferred (``_scan_defer``) until the loaded lane comes out.

        :param slot: 0-based AMS slot index that was just inserted
        """
        if not getattr(self, "insert_pullin", True) or self._bridge is None:
            return
        if self._is_ht():          # single lane -- situation cannot arise
            return
        # Never during a print: the loaded lane must keep its follower.
        afc = getattr(self, "afc", None)
        try:
            if afc is not None and afc.function.in_print():
                return
        except Exception:
            pass
        # Don't cut in on a load/unload/scan already moving filament on the bus.
        try:
            if afcBambuAMS._afc_motion_busy(self):
                return
        except Exception:
            pass
        # The lane(s) currently holding this unit's follower -- to drop now and
        # restore after. A boxed unit holds one at a time, but be general.
        loaded = [ln for ln in self.lanes.values()
                  if bool(getattr(ln, "tool_loaded", False))]
        if not loaded:
            return
        r = self.afc.reactor
        now = r.monotonic()
        # One sequence at a time: overlapping sequences would let one's re-engage
        # land in the middle of the next one's prime. Insert edges arriving while a
        # sequence is in flight are absorbed; the next edge after the follower is
        # back gets its own sequence.
        if now < getattr(self, "_pullin_until", 0.0):
            return
        self._pullin_until = now + 6.8      # reserve the drop->prime->re-engage window
        # 1) drop the follower so the unit leaves state:4 (preload_disable clears)
        for ln in loaded:
            try:
                self.set_feed_assist(ln, False)
            except Exception:
                pass

        # 2) prime the new bay up to the hub. First shot at +1.5s, once the unit
        #    has left state:4 (a prime sent in state:4 does nothing); a second at
        #    +3.5s covers a bay that needs more than one feeder pass. Reader
        #    dormant: no scan/measure.
        def _prime(_et):
            """
            Reactor callback: send the prime command for the slot.

            :param _et: reactor event time
            """
            try:
                self._bridge.send({"cmd": "prime", "unit": self.ams_index,
                                   "slot": slot})
            except Exception:
                pass
        r.register_callback(_prime, now + 1.5)
        r.register_callback(_prime, now + 3.5)

        # 3) re-engage the loaded lane's follower once the prime has settled, and
        #    release the in-flight guard.
        def _reengage(_et):
            """
            Reactor callback: re-engage the follower on every loaded lane.

            :param _et: reactor event time
            """
            for ln in loaded:
                try:
                    self._engage_follower(ln)
                except Exception:
                    pass
            self._pullin_until = 0.0
        r.register_callback(_reengage, now + 6.0)

        self.logger.info(
            f"AFC bambu {self.name}: slot {slot} inserted while a lane is loaded "
            f"and idle -- dropping the follower, priming the bay in (motion only, "
            f"no scan/measure), then re-engaging the loaded lane. Scan deferred.")

    def _start_tag_scan(self, slot: int, info: dict) -> None:
        """
        Begin a tag read for a slot, and arm the defaults fallback behind it.

        Shared by _maybe_auto_scan and the deferred retry so both run the
        identical sequence -- a held scan must take the same path, or the old
        spool's data survives.

        :param slot: 0-based AMS slot index on this unit
        :param info: Normalized slot info as of the insert
        """
        # A genuine insert edge -> read the tag, even when the slot already shows
        # material: a swapped-in spool must re-read, and the HT never re-reads on
        # its own. First blank the old profile so the UI doesn't show the previous
        # spool's data until the new tag reads (a Spoolman-bound lane is kept).
        lane = self._lane_for_slot(slot)
        if lane is not None and getattr(lane, "spool_id", None) in (None, "", 0):
            self._clear_lane_filament(lane)
        self._auto_scanned[slot] = True
        self._open_scan(slot)
        # The motion guard's own clock -- see _scan_in_flight. Set unconditionally:
        # the filament moves whether or not a pre-scan profile was recorded.
        mt = getattr(self, "_scan_motion_t0", None)
        if mt is not None and 0 <= slot < len(mt):
            try:
                mt[slot] = self.afc.reactor.monotonic()
            except Exception:
                mt[slot] = None
        # The AMS HT scans its own tag on its preload switch; the bridge firmware
        # arms the 0x1800 poll on the insert edge, so no scan is sent for the HT
        # (one would read stale flash and could clobber the firmware's window).
        # The boxed AMS scans by feeding past its bay reader, driven via scan().
        if self._is_ht():
            # The firmware arms both the HT's scan and its capacity window on
            # this edge (ht_scan_arm -> cap_open), so nothing is sent -- but the
            # slot must still be marked pending: the apply path is gated on the
            # pending list, which otherwise only _start_capscan opens, and an
            # unmarked measurement is discarded.
            afcBambuAMS._cap_open_pending(self, slot)
            self.logger.debug(
                f"AFC bambu {self.name}: new spool in slot {slot}; HT scans and "
                f"measures it on insert (firmware-driven at 0x1800)")
        else:
            self.logger.debug(
                f"AFC bambu {self.name}: new spool detected in slot {slot}, "
                f"scanning tag")
            # The capacity measurement rides on the insert's one autonomous
            # preload; measure_on_insert is enforced in the firmware (cap_open)
            # for every unit type. No seat gate: "sw_sta update" exists only on
            # the AMS 2. calibrate_on_insert is opt-in, see its declaration.
            _cal = bool(getattr(self, "calibrate_on_insert", False))
            if afcBambuAMS._start_capscan(self, slot, cali=_cal, insert=True):
                getattr(self, "_scan_claim_tries", {}).pop(slot, None)
            else:
                # The bus claim was refused: another unit's spool operation is
                # running, and two units scanning at once can take Klipper down.
                # Defer: re-run this same insert sequence once the bus frees.
                # _start_tag_scan re-opens the hold on each retry, so the no-tag
                # backstop restarts rather than expiring mid-queue and
                # defaulting a bay that was never scanned.
                if not afcBambuAMS._defer_scan_retry(self, slot, info,
                                                     "bus busy"):
                    # No reactor to defer on, or the queue has outlived any
                    # plausible sibling scan: fall back to the burst rather
                    # than never scanning at all.
                    getattr(self, "_scan_claim_tries", {}).pop(slot, None)
                    self.scan(slot)

    def _defer_scan_retry(self, slot: int, info: dict, why: str) -> bool:
        """
        Re-run this slot's insert scan in 5 s; True if the retry was armed.

        Shared by the not-yet-seated wait and the bus-claim wait -- both are
        "the world is not ready, ask again shortly", both bounded by the
        same counter so neither can queue forever.

        :param slot: 0-based AMS slot index on this unit
        :param info: Normalized slot info captured at the insert edge
        :param why: For the debug log
        :return bool: True when the retry was scheduled
        """
        tries = getattr(self, "_scan_claim_tries", None)
        if tries is None:
            tries = self._scan_claim_tries = {}
        n = tries.get(slot, 0) + 1
        tries[slot] = n
        if n > 24:                           # ~2 minutes of polite retry
            return False
        try:
            self.afc.reactor.register_callback(
                lambda et, s=slot, i=dict(info):
                    afcBambuAMS._retry_claimed_scan(self, s, i),
                self.afc.reactor.monotonic() + 5.0)
            self.logger.debug(
                f"AFC bambu {self.name}: {why}, deferring slot {slot} "
                f"tag scan (try {n})")
            return True
        except Exception:
            return False

    def _retry_claimed_scan(self, slot: int, info: dict) -> None:
        """
        Re-run a deferred insert scan once the bus should be free.

        A no-op when the question answered itself while queued: the spool was
        pulled, or a verdict already resolved for this bay (the firmware's
        window may have run via another path).

        :param slot: 0-based AMS slot index on this unit
        :param info: Normalized slot info captured at the insert edge
        """
        try:
            cur = (self._slots[slot]
                   if 0 <= slot < len(getattr(self, "_slots", []) or [])
                   else None) or {}
            if not cur.get("present"):
                getattr(self, "_scan_claim_tries", {}).pop(slot, None)
                return                           # spool gone; nothing to scan
            afcBambuAMS._start_tag_scan(self, slot, info)
        except Exception as e:
            self.logger.debug(
                f"AFC bambu {self.name}: deferred scan retry failed: {e}")

    def _clear_lane_filament(self, lane: Any) -> None:
        """
        Blank a lane's filament profile so a previous spool's data doesn't linger
        in the UI until a fresh tag reads or defaults are applied. Best-effort
        per attribute.

        Every field a tag sets must be cleared here. filament_name matters
        most: it is what the Mainsail card displays, and
        apply_filament_defaults does not write it, so a defaulted lane would
        otherwise keep the last tag's name.

        :param lane: The AFC lane object
        """
        for attr, val in (("material", ""), ("color", ""), ("weight", 0),
                          ("filament_name", ""), ("sub_type", ""),
                          ("spool_vendor", "")):
            try:
                setattr(lane, attr, val)
            except Exception:
                pass
        try:
            lane.bambu_sku = ""
        except Exception:
            pass
        # The tare came from the departed spool too -- see _restore_config_tare.
        # getattr so a duck-typed stand-in without the helper skips it rather
        # than raising.
        restore = getattr(self, "_restore_config_tare", None)
        if restore is not None:
            restore(lane)

    def _finalize_scan(self, slot: int, scanned: bool = True,
                       no_record: bool = False) -> None:
        """
        The no-tag outcome: give the lane its AFC defaults.

        Called when the unit has FINISHED a scan without reading a tag (see
        ``_scan_verdict``), and at boot for a present bay that carries no
        record at all (``_restore_untagged_defaults``). It does not decide
        whether a tag read -- that is the unit's to say, and asking it is
        ``_scan_verdict``'s single job.

        :param slot: 0-based AMS slot index on this unit
        :param scanned: Whether a read was actually attempted. False from the
            two callers that apply defaults WITHOUT scanning -- the boot/claim
            restore, and the hold taken when a lane on this unit is threaded to
            the toolhead. Without a scan (and without no_record) the lane is
            left untouched; otherwise it selects the log wording, so a bay
            that was never read is not reported as a reader fault.
        :param no_record: True only from _restore_untagged_defaults: the bay
            is occupied and the lane, saved record and unit tag are all empty,
            so defaults are applied even without a scan.
        """

        if not (0 <= slot < len(self._slots)):
            return
        info = self._slots[slot]
        if not info or not info.get("present"):
            self._release_scan_hold(slot)
            return                                   # spool gone
        # Two outcomes, never three: a tag read applies the record, no tag
        # applies defaults. No early return for a linked lane or one that
        # already has a material, both describe the PREVIOUS spool, and a
        # stale binding blocks the next real tag from applying.
        lane = self._lane_for_slot(slot)
        if lane is None:
            self._release_scan_hold(slot)
            return
        # No scan -> leave the lane alone. Callers that have not looked at the
        # bay (the boot/claim restore, the toolhead hold) cannot tell an empty
        # lane from one whose async Spoolman fetch has not landed yet, so
        # anything they changed could overwrite the user's binding. As with
        # OpenAMS and ACE, the lane at startup is whatever AFC restored; a bay
        # with no tag gets its profile from a real scan (AFC_BAMBU_SCAN or the
        # next insert).
        #
        # The one exception is no_record=True, which only
        # _restore_untagged_defaults passes: the bay is occupied and the lane,
        # the saved record and the unit's tag are all empty, so there is
        # nothing to lose and defaults beat a blank lane against a visible
        # spool.
        if not scanned and not no_record:
            self._release_scan_hold(slot)
            return
        # Blanked here rather than relied on from the insert edge, because the
        # status loop may have re-armed it in between. The Spoolman link goes
        # with the filament, as on a removal. Only when a scan actually ran:
        # without one there is no evidence the bay changed hands.
        if scanned:
            # A failed tag read says nothing about how much filament is on the
            # reel, so a weight the AMS measured is kept. A held
            # _measured_remain means "measured this session, no removal since"
            # (it is dropped on the removal edge and when a re-scan starts).
            # Only the weight is spared: material, colour and name come from
            # the tag, and a failed read is a reason to stop showing the
            # previous spool's.
            _held = None
            try:
                for _o in afcBambuAMS._built_measure_objs(self):
                    _held = (getattr(_o, "_measured_remain", None) or {}).get(slot)
                    if _held is not None:
                        break
            except Exception:
                _held = None
            _keep_w = getattr(lane, "weight", None) if _held is not None else None
            self._clear_lane_filament(lane)
            if _keep_w:
                try:
                    lane.weight = _keep_w
                    self.logger.debug(
                        f"AFC bambu {self.name}: {lane.name} -- kept the "
                        f"measured {_keep_w} g through a no-tag scan "
                        f"({_held}% held, no removal since)")
                except Exception:
                    pass
            self._unbind_spool(
                lane, "the scan read no tag, so this link is the previous spool's")
        try:
            afcBambuAMS._apply_lane_defaults(self, lane, info)
            # Surface the UID even here. "No readable tag" means the PROFILE
            # did not decode; the chip UID is usually right there, and it is
            # what a third-party spool gets bound by.
            _uid = None
            try:
                for _sl in (self._slots or []):
                    if _sl.get("index") == slot:
                        _uid = _sl.get("rfid_uid")
                        break
            except Exception:
                _uid = None
            # Distinguish a third-party tag from no tag: for a chip it cannot
            # authenticate the unit narrates
            #     STEP:stop goto auth / STEP:auth fail:-4
            #     STEP7:info_valid 0 or bbl:-1        (bbl = Bambu Lab)
            # and the log should not call a working reader blind.
            _foreign = False
            try:
                t0 = getattr(self, "_scan_t0", None)
                if self._bridge is not None and t0 is not None:
                    _foreign = self._bridge.rfid_foreign_tag_since(
                        t0[slot],
                        addr=getattr(self, "dry_dev_addr", 0) or None)
            except Exception:
                _foreign = False
            if _uid:
                _why = (f" -- the tag's UID is {str(_uid).upper()}, bind it to "
                        f"a spool in Spoolman to track this reel")
                # A read-less scan can still leave the chip UID, and Spoolman
                # may know that reel: look it up off the reactor through the
                # delegate, which carries every guard and no-ops without
                # Spoolman.
                try:
                    self._bind_by_uid_bg(lane, slot, str(_uid),
                                         " on a read-less scan")
                except Exception:
                    pass
            elif _foreign:
                _why = (" -- the bay HAS a tag, but its keys are not Bambu's "
                        "so the unit could not read the profile "
                        "(auth failed). A third-party spool: set the material "
                        "on the lane, or bind it in Spoolman by hand")
            elif scanned:
                # An AMS reports a UID only for a tag it can authenticate, so a
                # missing UID is not evidence of a missing chip (a Snapmaker or
                # Elegoo spool answers anticollision, then fails auth). The auth
                # refusal is not always caught by the _foreign branch, so name
                # both possibilities.
                _why = (" -- the unit read no tag here. That is an empty bay, "
                        "or a chip whose keys are not Bambu's: an AMS reports a "
                        "UID only for tags it can authenticate, so no UID does "
                        "not mean no chip")
            else:
                _why = (" -- nothing has read this bay yet. No scan was "
                        "attempted, so this says nothing about the reader or "
                        "the spool; reseat it, or run "
                        f"AFC_BAMBU_SCAN LANE={lane.name}")
            self.logger.info(
                f"AFC bambu {self.name}: "
                + (f"no readable tag profile in slot {slot}" if scanned
                   else f"no tag on record for slot {slot}")
                + f"; applied lane defaults to {lane.name}" + _why)
            # Persist only after a scan that actually ran, so the defaults
            # survive a restart. On the boot path the lane's Spoolman data may
            # still be in flight; defaults in memory are overwritten by the
            # later restore, but saved to disk they would replace the user's
            # record for good.
            if scanned:
                self._save_lane_vars()
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: default apply for slot {slot} "
                f"failed: {e}")

    def _apply_lane_defaults(self, lane: Any, info: dict) -> None:
        """
        Give a lane AFC's defaults, and nothing from the bay's record.

        The helper is handed the record with its profile stripped. Whoever
        calls this has no read of the spool in the bay, so any profile the
        record still carries is the PREVIOUS spool's -- and the helper prefers
        slot_info's own material/color over the AFC defaults, which would put
        it straight back on a lane that was just blanked, the same value by a
        different route. Links nothing and saves nothing; the caller decides.
        Raises what the helper raises.

        :param lane: the lane to fill
        :param info: the bay's normalized record
        """
        afc = getattr(self, "afc", None)
        info = {k: v for k, v in (info or {}).items()
                if k not in ("material", "sku", "color", "color_hex",
                             "temp_min", "temp_max", "weight",
                             "extruder_temp", "bed_temp")}
        apply_filament_defaults(
            lane, info,
            afc_defaults={
                "default_material_type": getattr(
                    afc, "default_material_type", None),
                "default_color": getattr(afc, "default_color", None),
            })







    def calibrate_bowden(self, cur_lane: Any, dis: float,
                         tol: float) -> "Tuple[bool, str, int]":
        """
        Bowden calibration is not necessary on a Bambu AMS.

        The AMS measures and drives its own lane distance -- it feeds to the
        toolhead sensor with its own feedback and calibrates the odometer per
        tray on insert. There is nothing for the host to measure by feeding
        against a stopwatch, so report success with a note rather than run a
        routine (or fall through to "function not defined").

        :param cur_lane: lane the calibration was requested for; unused
        :param dis: distance the caller would have fed; unused
        :param tol: tolerance the caller would have applied; unused
        :return tuple: (True, message, 0)
        """
        # Two surfacing paths, because CALIBRATE_AFC has two. The lane-loop
        # path honours the "calibration_lane" sentinel and shows
        # calibration_lane_message() cleanly (like OpenAMS). The BOWDEN command
        # path ignores the returned message entirely, so respond_info is the
        # only way to get the note in front of the user there.
        try:
            self.gcode.respond_info(
                "Bambu AMS measures its own lane automatically -- bowden "
                "calibration is not necessary.")
        except Exception:
            pass
        return (True, "calibration_lane", 0)

    def calibrate_lane(self, cur_lane: Any,
                       tol: float) -> "Tuple[bool, str, int]":
        """
        Lane calibration is automatic on a Bambu AMS, as with bowden.

        :param cur_lane: lane the calibration was requested for; unused
        :param tol: tolerance the caller would have applied; unused
        :return tuple: (True, message, 0)
        """
        return (True, "calibration_lane", 0)

    def calibrate_hub(self, cur_lane: Any,
                      tol: float) -> "Tuple[bool, str, int]":
        """
        Hub calibration is not needed on a Bambu AMS.

        A Bambu AMS multiplexes internally and has no physical hub switch (its
        AFC_hub is virtual), so there is no hub position to measure.

        :param cur_lane: lane the calibration was requested for; unused
        :param tol: tolerance the caller would have applied; unused
        :return tuple: (True, message, 0)
        """
        msg = ("Bambu AMS has no physical hub (internal multiplexing) -- "
               "hub calibration is not needed.")
        try:
            self.gcode.respond_info(msg)
        except Exception:
            pass
        return (True, msg, 0)

    def calibration_lane_message(self) -> str:
        """The completion-prompt text for a Bambu AMS bowden/lane calibration.

        Surfaced by the framework when calibrate_bowden/calibrate_lane return
        the "calibration_lane" sentinel -- the same path OpenAMS uses -- so the
        prompt shows THIS instead of a generic "Done!". {lanes} is filled by the
        framework with the lanes that were "calibrated".
        """
        return ("\nBambu AMS measures its own lane automatically -- bowden/lane "
                "calibration is not necessary for: {lanes}\n")



    def _log_tag_readout(self, lane: Any, info: dict,
                         force: bool = False) -> None:
        """
        Print what the tag actually said, on the read.

        Fires when a scan RESOLVES as a read -- not when the lane's values
        change. A re-scan of a bay that already showed the right spool is
        still an answer to a question the operator asked, and printing
        nothing reads as nothing having happened.

        remain% is labelled MEASURED or stored because the tag's stored
        figure and the unit's measurement routinely disagree, and only the
        measurement describes the spool in the bay.

        :param lane: the AFC lane the tag was applied to
        :param info: normalized bridge slot info
        :param force: log even when the readout matches the last one printed
        """
        try:
            mat = info.get("material")
            if not mat:
                return
            # Once per distinct answer. Two paths deliver a tag -- a scan the
            # operator asked for, and the background fill picking a bay up on
            # its own (verdict "none", which is how a re-insert into a fresh
            # bay lands) -- and both deserve the line. A steady status frame
            # repeating the same record does not. force=True is the asked-for
            # scan: it always answers, even when nothing changed.
            slot_i = info.get("index")
            key = (info.get("rfid_uid"), mat, info.get("meas_seq"))
            seen = getattr(self, "_readout_last", None)
            if seen is None:
                seen = self._readout_last = {}
            if not force and seen.get(slot_i) == key:
                return
            seen[slot_i] = key
            colour = info.get("color") or ""
            colour = ("#" + colour.lstrip("#")) if colour else "?"
            meas = info.get("meas_pct")
            try:
                nominal = int(info.get("weight") or 1000)
            except (TypeError, ValueError):
                nominal = 1000
            # A stamp is not a measurement of this session. The record carries
            # the bay's last meas_pct whenever it was taken, and the one found
            # when the connection started is recorded, not applied (see
            # _baseline_meas_stamp). Only a figure that is on the lane, or
            # about to be adopted, is called measured.
            stale = False
            if meas:
                try:
                    _seen = (getattr(self, "_meas_seen", None)
                             or {}).get(slot_i)
                    _held = afcBambuAMS._held_measurements(self).get(slot_i)
                    stale = (slot_i in (getattr(self, "_meas_baselined", None)
                                        or ())
                             and _seen is not None and _seen[1] == meas
                             and _held != meas)
                except Exception:
                    stale = False
            if stale:
                rtxt = f"{meas}% stamped before this connection, not applied"
            elif meas:
                # Quote the same grams the lane gets, so this line cannot
                # contradict it: the same floor (_remain_floor_pct, the lowest
                # a reel has measured, since the odometer wanders a few
                # percent) and the same _grams_for (measured radius and
                # density -> mass). The AMS reads a full reel over 100%, so the
                # fallback arithmetic caps the percent at 100. The delegate
                # lookup can raise, so each call has its own try and falls back
                # rather than costing the whole line.
                def _linear():
                    return max(1, (int(nominal) * min(int(eff), 100)) // 100)
                eff = int(meas)
                # _measure_of, not _spool: the lane's grams come from the
                # measurement delegate with or without Spoolman, and so does
                # this line's figure.
                try:
                    _fl = getattr(afcBambuAMS._measure_of(self),
                                  "_remain_floor_pct", None)
                    if _fl is not None:
                        eff = int(_fl(info.get("rfid_uid") or "", int(meas)))
                except Exception:
                    eff = int(meas)
                try:
                    _gf = getattr(afcBambuAMS._measure_of(self),
                                  "_grams_for", None)
                    grams_est = (_gf(slot_i, lane, eff, nominal)
                                 if _gf is not None else _linear())
                except Exception:
                    grams_est = _linear()
                rtxt = f"{meas}% MEASURED"
                if eff != int(meas):
                    rtxt += f", held at {eff}%"
                rtxt += f" (~{grams_est} g"
                rtxt += (" -- a full reel reads proud of the reference "
                         "radius)" if eff > 100 else ")")
            else:
                # The tag's own remain field is never shown: it is not a
                # quantity (a sealed 1 kg spool ships with 0.8 in it). Only a
                # real measurement produces a remaining figure.
                rtxt = "no measurement yet"
            # DEBUG, not INFO: _say_spool_summary is the operator's line for
            # a scan. This one keeps the diagnostic extras (SKU, nominal,
            # nozzle range) in AFC.log.
            self.logger.debug(
                f"AFC bambu {self.name}: {lane.name} tag -- {mat}"
                f"{(' ' + str(info.get('sku'))) if info.get('sku') else ''}"
                f" | color {colour}"
                f" | uid {str(info.get('rfid_uid') or '?').upper()}"
                f" | remaining {rtxt}"
                f" | nominal {nominal} g"
                f" | nozzle {info.get('temp_min') or '?'}"
                f"-{info.get('temp_max') or '?'}C"
                # Tagged with where it came from, because almost every Bambu
                # tag leaves this blank and the number below it is then the
                # material table's, not the spool's.
                f" | bed {info.get('bed_temp') or '?'}C"
                f" ({info.get('bed_temp_source') or 'unknown'})")
        except Exception:
            pass

    def _boot_hold(self, lane: Any, info: dict) -> bool:
        """
        Whether this bay's cached record must not be written onto its lane yet.

        True while BOTH are true: no scan has run on this bay during this
        connection, and the lane already carries state AFC restored from saved
        vars. That pairing is the whole test -- an unscanned bay means the unit
        is only repeating what it already knew, and a lane with data means
        there is something of the user's to lose.

        A blank lane is deliberately NOT held: filling it from the record is how
        a lane whose saved vars were lost comes back populated, and there is
        nothing there to overwrite.

        :param lane: The AFC lane object
        :param info: Normalized slot info for this bay
        :return bool: True to leave the lane alone
        """
        try:
            slot = info.get("index")
            if slot in (getattr(self, "_scanned_bays", None) or set()):
                return False
            # A bay whose removal edge cleared the lane in this process: what
            # is on it now (lane defaults for an insert nothing has read) is
            # not something AFC restored, whatever the bridge link did since.
            if slot in (getattr(self, "_cleared_bays", None) or set()):
                return False
            # Nor are defaults this connection put on a lane nothing knew
            # anything about (_restore_untagged_defaults).
            if slot in (getattr(self, "_defaulted_bays", None) or set()):
                return False
            return (getattr(lane, "spool_id", None) not in (None, "", 0)
                    or getattr(lane, "material", None) not in (None, ""))
        except Exception:
            return False

    def _unbound_lookup_state(self, lane: Any, info: dict) -> str:
        """
        Where a restored lane with no spool stands with its bay's one lookup.

        _surface_slot_info's dispatch is the only automatic Spoolman lookup,
        and a lane AFC restored with material and no spool never reaches it
        after a restart: the _afc_owned branch of _sync_lanes fills the
        variant and continues, and the boot hold returns first. Neither lets
        go until a scan window opens, and a capscan opens none, so the tag in
        the bay would never be asked about.

        _lookup_unbound asks instead, and only about a record that is plainly
        the lane's own spool. Every condition is required:

        * the bay is occupied and its record carries a chip UID;
        * the lane has no spool, and the bay's lookup has not gone out on this
          connection (_spoolman_latched);
        * no other unit reports the UID (_uid_claimed_elsewhere): a copied
          record is not a reading;
        * the record's base material is the lane's, and the lane has a colour
          and it is the record's. A spool swapped while the bridge was down
          leaves the departed one's record behind, and a lane put on defaults
          has no colour to match. A black tag's lane has none either, and its
          variant stands in for it (_lane_colour_is_records);
        * nothing is being asked of the bay: no scan or unanswered capacity
          window, no scan motion, no re-read the firmware owes it, and no
          retry of a lookup Spoolman did not answer still to come round.

        A capacity window blanks the record's profile and keeps the UID
        (cap_open), so while one is open -- answered or not -- the material
        cannot be tested and the lookup is only waited for; with nothing
        open, a record with no material has nothing to test the lane against.

        An unclaimed pool spare holds no unit's bays and is never asked.

        :param lane: the lane mapped to this bay
        :param info: the bay's normalized slot record
        :return str: "due" -- ask now; "wait" -- it will be asked once the
          bay is quiet; "stale" -- the record does not describe the lane;
          "no" -- nothing to ask for this bay
        """
        try:
            if getattr(self, "pool", False) or lane is None or not info:
                return "no"
            slot = info.get("index")
            uid = info.get("rfid_uid")
            if not info.get("present") or not uid or slot is None:
                return "no"
            if getattr(lane, "spool_id", None) not in (None, "", 0):
                return "no"
            if slot in (getattr(self, "_spoolman_latched", None) or ()):
                return "no"
            lane_mat = str(getattr(lane, "material", "") or "").strip().lower()
            # Neither a colour nor a variant: a lane on defaults, which no
            # record can be tested against.
            if not lane_mat or not (_colour_key(getattr(lane, "color", ""))
                                    or getattr(lane, "sub_type", "")):
                return "no"
            claimed = getattr(self, "_uid_claimed_elsewhere", None)
            if callable(claimed) and claimed(uid):
                return "stale"
            busy = bool(info.get("reread_pending"))
            t0 = getattr(self, "_scan_t0", None) or []
            if 0 <= slot < len(t0) and t0[slot] is not None:
                busy = True
            moving = getattr(self, "_scan_in_flight", None)
            if callable(moving) and moving(slot):
                busy = True
            # A capacity window is waited out until its measurement is in:
            # closed by the narration path, or marked _cap_answered by the
            # adopt block (which leaves it open for the narration).
            try:
                live = slot in afcBambuAMS._cap_live_pending(self)
            except Exception:
                live = slot in (getattr(self, "_cap_pending", None) or {})
            if live and slot not in (getattr(self, "_cap_answered", None)
                                     or ()):
                busy = True
            tag = info.get("material")
            if not tag or str(tag).lower() == "unknown":
                # The profile comes back once the window is done with it,
                # answered or not.
                return "wait" if (busy or live) else "no"
            material, _sub = _split_bambu_material(tag)
            if (material.strip().lower() != lane_mat
                    or not _lane_colour_is_records(lane, info)):
                return "stale"
            if busy:
                return "wait"
            due = (getattr(self, "_lookup_retry", None) or {}).get(slot)
            if due is not None and _mono(getattr(self, "afc", None)) < due:
                return "wait"
            return "due"
        except Exception:
            return "no"

    def _lookup_unbound(self, lane: Any, info: dict) -> None:
        """
        Ask Spoolman, once, for the spool carrying a restored lane's tag.

        Called where _surface_slot_info's own dispatch is not reached: the
        _afc_owned verdict-"none" branch of _sync_lanes, and the boot hold's
        early return. The conditions are _unbound_lookup_state's; the
        dispatch is the surface path's own -- the same one-shot latch and the
        same _spoolman_sync -- so a later surface of the bay sends no second
        lookup. AFC_BAMBU_SCAN re-arms the latch (_open_scan).

        It writes nothing onto the lane, and the lookup is match-only whatever
        auto_spoolman_create says (``restored``): nothing is created from a
        lane AFC restored. When the bind lands, AFC's set_spoolID loads the
        spool's remaining weight from Spoolman, as it does for every bound
        lane at restart; a measurement taken on this connection before the
        lookup went out is owed to the bind (_lookup_coming) and handed to
        the spool once it attaches. BambuSpoolman._bind_by_uid_bg refuses a
        spool with no remaining weight (AFC would clear the lane for it) and
        asks again later when Spoolman did not answer.

        Never raises: it runs inside the status pass.

        :param lane: the lane mapped to this bay
        :param info: the bay's normalized slot record
        """
        try:
            if afcBambuAMS._unbound_lookup_state(self, lane, info) != "due":
                return
            # Asked only now, after the conditions: this builds the delegate.
            if getattr(self, "_spool", None) is None:
                return
            slot = info.get("index")
            latch = getattr(self, "_spoolman_latched", None)
            if latch is None:
                latch = self._spoolman_latched = set()
            latch.add(slot)
            (getattr(self, "_lookup_retry", None) or {}).pop(slot, None)
            self.logger.debug(
                f"AFC bambu {self.name}: {lane.name} has no Spoolman spool and "
                f"its bay's tag {str(info.get('rfid_uid')).upper()} "
                f"({info.get('material')}) matches the lane -- looking the "
                f"tag up in Spoolman; the lane is left as AFC restored it")
            self._spoolman_sync(lane, info, restored=True)
        except Exception:
            try:
                self.logger.debug(
                    f"AFC bambu {self.name}: lookup for an unbound lane "
                    f"failed", traceback=traceback.format_exc())
            except Exception:
                pass

    def _lookup_coming(self, slot: Optional[int], info: dict) -> bool:
        """
        Whether _lookup_unbound will still send this bay its lookup.

        Asked by the Spoolman delegate's _sync_owed and _bind_coming, which
        otherwise count only a bay scanned on this connection as owed a bind.
        The adopt block runs ahead of the lookup in every pass, and an open
        capacity window holds the lookup until its measurement is in -- so a
        measurement adopted first is owed to this bind, or the bind loads
        Spoolman's stored weight over it and the summary is said before the
        lookup it is about.

        :param slot: 0-based AMS slot index
        :param info: that bay's normalized slot record
        :return bool: True while a lookup for this bay is still to be sent
        """
        try:
            if slot is None or not info:
                return False
            afc = getattr(self, "afc", None)
            if afc is None or getattr(afc, "spoolman", None) is None:
                return False
            lane = afcBambuAMS._lane_for_slot(self, slot)
            return afcBambuAMS._unbound_lookup_state(
                self, lane, info) in ("due", "wait")
        except Exception:
            return False

    def _defaults_until_read(self, slot: int, info: dict,
                             say: bool = True) -> None:
        """
        A spool put into a bay during a print: lane defaults until it is read.

        Nothing is scanned during a print (a scan moves filament), so the
        insert stays unread until the unit reads it on its own or
        AFC_BAMBU_SCAN runs. Called once the bay has stayed occupied for
        DEFAULTS_SETTLE_S after the insert edge (_maybe_auto_scan). The
        bay's removal edge on this connection cleared the lane
        (_removed_bays). A record with no identity -- the
        firmware blanks the record on a removal it sees -- or one it still
        owes a re-read (a boxed unit's insert edge sets needs_reread) says
        nothing about the spool now in the bay, so the lane takes AFC's
        defaults, linked to nothing, and the operator is told once how to
        have it read. The surface path links nothing while the re-read is
        owed, and applies the tag when the read lands. A link found on the
        lane is the departed spool's (a bind that landed after the removal
        edge unbound the lane), so it goes with that spool's profile, as in
        _finalize_scan.

        Nothing here for a record that still names a spool with no re-read
        owed: that is a presence flap the firmware never saw, the spool never
        left, and the surface path linked it again from the record on the
        insert frame, before this runs. Nor for a bay whose removal this
        connection did not see: a bridge that has just reconnected reports
        every bay empty before its first presence poll, and that lane was not
        cleared for a new spool.

        Never raises: it runs inside the status pass.

        :param slot: 0-based AMS slot index on this unit
        :param info: the bay's normalized slot record
        :param say: log the line telling the operator how to have it read
          (False when putting a lane back on defaults it was already given)
        """
        try:
            if slot not in (getattr(self, "_removed_bays", None) or ()):
                return
            named = info.get("rfid_uid") or info.get("material")
            if named and not info.get("reread_pending"):
                return
            lane = self._lane_for_slot(slot)
            if lane is None:
                return
            if getattr(lane, "spool_id", None) not in (None, "", 0):
                self._clear_lane_filament(lane)
                self._unbind_spool(
                    lane, "nothing has read the spool in the bay, so this "
                          "link is the previous spool's")
            afcBambuAMS._apply_lane_defaults(self, lane, info)
            if not say:
                return
            self.logger.info(
                f"AFC bambu {self.name}: spool INSERTED in slot {slot} during "
                f"a print, and nothing has read it -- {lane.name} is on lane "
                f"defaults and linked to no spool until it is read. The unit "
                f"may read it on its own; to read it yourself, run "
                f"AFC_BAMBU_SCAN LANE={lane.name} once the print is done")
            try:
                self._save_lane_vars()
            except Exception:
                pass
        except Exception:
            try:
                self.logger.debug(
                    f"AFC bambu {self.name}: lane defaults for the slot {slot} "
                    f"insert failed", traceback=traceback.format_exc())
            except Exception:
                pass

    def _surface_slot_info(self, lane: Any, info: dict) -> None:
        """
        Apply the AMS tag's profile to a lane: material, variant, color,
        Bambu type code and print temps. The tag overwrites these (a derived
        bed temp only fills a blank); the lane's weight is seeded only when
        unset or nominal. The tag's UID (the Mifare chip UID) drives Spoolman
        binding/creation via _spoolman_sync.

        :param lane: The AFC lane object
        :param info: Normalized slot info from bridge_slot_to_info
        """
        tag_material = info.get("material")
        if tag_material and tag_material.lower() == "unknown":
            tag_material = None

        # Not a record of the spool in the bay. An empty bay's record is the
        # one its departed spool left (the removal edge already cleared and
        # unbound the lane); writing it back would restore the departed
        # spool's profile and spend the bay's one lookup. A record the
        # firmware still owes a re-read is not the bay's yet either: a boxed
        # unit's insert edge flags it while the record can still be the
        # departed spool's, and a capacity window flags it while the profile
        # is being collected again. Wait for the read; a measurement owed to
        # a bind already sent is still handed over when that bind lands. A
        # record with no "present" key (a test stand-in) is not held.
        if (("present" in info and not info.get("present"))
                or info.get("reread_pending")):
            _arw = getattr(self, "_apply_remain_weight", None)
            if info.get("present") and callable(_arw):
                _arw(lane, info)
            return

        # At startup the lane is whatever AFC restored, as with OpenAMS and ACE.
        # This runs on every status frame and the AMS re-reports its cached tag
        # record in each, so until this bay has been scanned on this
        # connection (_open_scan records that) the record may only fill a
        # blank lane -- otherwise it would overwrite the state AFC restored
        # from AFC.var.unit. A real insert or AFC_BAMBU_SCAN releases the hold.
        # getattr so a duck-typed stand-in without the helper skips the hold
        # rather than raising.
        _hold = getattr(self, "_boot_hold", None)
        if _hold is not None and _hold(lane, info):
            # The hold keeps the record off the lane, not the lane off
            # Spoolman: a restored lane with no spool still has its tag looked
            # up (_lookup_unbound writes nothing onto the lane), and a
            # measurement owed to that bind is handed over when it lands.
            _lookup = getattr(self, "_lookup_unbound", None)
            if _lookup is not None:
                _lookup(lane, info)
                self._apply_remain_weight(lane, info)
            if info.get("sku") and getattr(lane, "bambu_sku", None) in (None, ""):
                lane.bambu_sku = info["sku"]
            return

        # No "already bound?" gate: the tag is the bay, so it is applied like
        # any other reader does (read -> lane -> Spoolman). Spoolman stays
        # authoritative for the spool_id and the remaining weight only.
        if tag_material:
            # The AMS tag is the source of truth for this bay. Apply it directly,
            # overwriting any AFC default -- the shared helper's "only if empty"
            # rule would let a default applied before the read lock out the tag.
            color = info.get("color")
            color_hex = (color if color.startswith("#") else "#" + color) \
                if color else None
            tmin = info.get("temp_min")
            material, sub_type = _split_bambu_material(tag_material)
            changed = (getattr(lane, "material", None) != material
                       or getattr(lane, "sub_type", None) != sub_type
                       or (color_hex and getattr(lane, "color", None) != color_hex))
            lane.material = material
            # Variant, vendor and display name, implied by the tag, so every
            # surface that shows a spool (dryer panel, Spoolman, RFID
            # notifications) has them.
            lane.sub_type = sub_type
            lane.spool_vendor = BAMBU_BRAND
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
            # Bed temp: most tags leave it blank and it is then derived from
            # the material. The tag's own figure overwrites like tmin; a
            # derived one only fills a blank, so it never replaces a value set
            # from Spoolman, saved vars or by hand.
            bed = info.get("bed_temp")
            if bed is not None and (info.get("bed_temp_source") == "tag"
                                    or getattr(lane, "bed_temp", None) is None):
                try:
                    lane.bed_temp = float(bed)
                except (TypeError, ValueError):
                    pass
            # A 0 g spool renders as empty in the UI, so seed the tag's nominal
            # weight when the lane's weight is unset or still exactly nominal
            # (AFC only decrements from here). remain 0 means "not measured".
            tag_w = info.get("weight")
            try:
                nominal = int(tag_w) if tag_w else 1000
            except (TypeError, ValueError):
                nominal = 1000
            # The tag's stored remain never sets the weight (it is not a
            # measurement). Seed the nominal only; a measurement arrives
            # through _adopt_measured_remain.
            w = nominal
            # Seed only a default or unset weight. A measurement is applied
            # once, when it is taken (_adopt_measured_remain), and a lane's
            # weight is otherwise AFC's to keep: extrusion counts it down, and
            # AFC.var.unit or Spoolman bring it back across a restart.
            cur_w = getattr(lane, "weight", 0)
            if not cur_w or int(cur_w) == nominal:
                lane.weight = w
            if changed:
                self.logger.info(
                    f"AFC bambu {self.name}: applied tag to {lane.name}: "
                    f"{getattr(lane, 'filament_name', '') or tag_material} "
                    f"{color_hex or ''}".rstrip())
                # Change-only, so this is not a write per status frame.
                # Persisted so the lane does not depend on the AMS still
                # holding the record at the next boot.
                self._save_lane_vars()
                # The fill delivers tags with no scan window open (verdict
                # "none"), e.g. a re-insert into a fresh bay; log the tag that
                # changed the lane.
                afcBambuAMS._log_tag_readout(self, lane, info)
            # Full decode + a UID -> bind/create in Spoolman, keyed on the UID.
            slot_i = info.get("index")
            latch = getattr(self, "_spoolman_latched", None)
            if latch is None:
                latch = self._spoolman_latched = set()
            if slot_i not in latch:
                latch.add(slot_i)
                self._spoolman_sync(lane, info)
        # No readable tag yet (a bay is staged but not yet fed past the reader):
        # do NOT apply an AFC default here. The tag arrives after the scan feeds
        # the spool past the reader, and a default applied on stage would show
        # the wrong material until (and lock out) the real tag. Leave the lane's
        # material untouched; the tag lands when the scan reads it.
        elif info.get("rfid_uid"):
            # UID-only: no profile but a good UID, so match Spoolman on the UID
            # alone. Only reached once the unit has answered (_sync_lanes),
            # because a bay's UID outlives its cleared profile and would bind
            # the previous spool mid-scan.
            slot_i = info.get("index")
            latch = getattr(self, "_spoolman_latched", None)
            if latch is None:
                latch = self._spoolman_latched = set()
            if slot_i not in latch:
                latch.add(slot_i)
                self._spoolman_sync(lane, info)

        # A measurement taken before this bay's Spoolman bind landed is owed to
        # the spool the bind attaches: handed over once, on the first frame
        # that finds the lane bound. One turned into grams before the bay's
        # material was known is finished once, now that the tag above has
        # named it. Nothing else here touches the weight -- a measurement is
        # applied at the time it is taken, not on every frame.
        self._apply_remain_weight(lane, info)

        # Bambu profile code (e.g. GFA00), a nice sub_type hint for Spoolman.
        if info.get("sku") and getattr(lane, "bambu_sku", None) in (None, ""):
            lane.bambu_sku = info["sku"]

    def _record_fresh(self, rec: dict) -> bool:
        """
        Whether a chamber record is recent enough to trust.

        :param rec: a record from _chamber_record().
        :return bool: True while it is under 120s old.
        """
        mono = getattr(self.reactor, "monotonic", None)
        nowm = mono() if callable(mono) else 0.0
        return (not nowm) or (nowm - rec.get("seen", 0.0) < 120.0)

    def _chamber_record(self) -> Optional[dict]:
        """
        This unit's own chamber telemetry, by CHAIN INDEX.

        The firmware stamps every narration line with the unit whose drain
        pulled it (completing the attribution itself when a class has exactly
        one member), so the record under this unit's index is exactly this
        unit's chamber -- N same-class dryers stay separable, and a unit that
        has never dried simply has no record.

        :return dict: {"temp", "target", "state", "seen"} for this unit, or
            None when it has not narrated chamber telemetry.
        """
        # A unit that is not on the bus owns no chamber. The chain index is a
        # unique key only for claimed units; an unclaimed pool placeholder sits
        # at the default 0 and would otherwise adopt the first real unit's
        # chamber (and its drying state) as its own.
        # The gate is the claim, not the online flag: claim() clears `pool`.
        # Gating on online would hide a real unit's chamber in the gap after a
        # reconnect, and with the record gone `attributable` goes false and
        # the "drying finished" release never fires. _unit_online uses `pool`
        # for the same reason.
        if getattr(self, "pool", False):
            return None
        by_unit = getattr(self._bridge, "_chmb_by_unit", None)
        if not by_unit:
            return None
        return by_unit.get(int(self.ams_index))

    def _chamber_live(self) -> tuple:
        """
        Resolve this unit's chamber telemetry: is it live, can we attribute it,
        and when was it last seen.

        ``attributable`` says whether the absence of telemetry means anything
        for this unit: it is True only when a chain-index-keyed record exists
        for it (see _chamber_record). Without one, a cycle is neither adopted
        nor released from telemetry.

        :return tuple: (live, attributable, seen_time)
        """
        rec = self._chamber_record() if self._bridge is not None else None
        if rec is not None:
            return self._record_fresh(rec), True, rec.get("seen", 0.0)
        return False, False, 0.0

    def _drop_silent_dry(self, refused_now: Any, started: Any) -> None:
        """
        Clear a dry cycle the unit silently ignored.

        The refusal guard in get_status needs an error on record, and an HT
        that was mid-task (scanning, feeding) when the dry frame landed leaves
        none -- it just never answers. Total silence past DRY_SILENT_GRACE
        means the command sank; surface that through the host refusal slot so
        the panel says why instead of counting down at a cold heater. HT only
        -- a boxed dry deafens the bridge's receiver, so silence proves nothing
        there (see DRY_SILENT_GRACE).

        :param refused_now: The unit's refusal on record for this attempt, if any
        :param started: When this cycle was commanded (_mono clock), or None
        """
        if (not self._drying or refused_now
                or self.ams_model not in _HT_MODELS
                or started is None
                or getattr(self, "_dry_seen_live", False)
                or (_mono(self) - started) <= DRY_SILENT_GRACE):
            return
        self._drying = False
        self._dry_host_refusal = (
            f"{self.name} never started the cycle -- the dry command drew no "
            f"reply and no chamber telemetry followed. An AMS HT that is "
            f"mid-task (scanning or feeding) ignores the command silently; "
            f"let it finish and start again.")
        self.logger.info(
            f"AFC bambu {self.name}: not drying -- the unit never answered "
            f"the dry command (silent for {int(DRY_SILENT_GRACE)}s)")

    def _follow_arm_acked(self, latest: Any) -> Any:
        """
        Whether the bridge has seen this unit acknowledge its follower arm.

        The arm frame (0x11/0x04) is never answered at the frame level, so the
        only receipt is the unit narrating ``state:4``. The bridge tracks that
        per unit and reports it as an ``armack`` bitmask; this pulls out our
        bit.

        Returns ``None``, not ``False``, when the status frame carries no
        mask: "not acknowledged" and "cannot tell" are different answers.

        :param latest: The most recent bridge status frame, or None
        :return: True/False when known, None when the firmware does not report
        """
        if not latest:
            return None
        mask = latest.get("armack")
        if not isinstance(mask, int):
            return None
        return bool(mask & (1 << self.ams_index))

    def _dryrem_says_drying(self, dr: int) -> bool:
        """
        Whether a positive firmware countdown may call this unit drying.

        dryrem only updates while 0x3C replies are decoding, and a boxed dry
        deafens the bridge's receiver -- so through a dry (and past a stop,
        if another unit keeps the receiver deaf) the firmware can hold a
        frozen positive countdown. A stop therefore stamps the countdown it
        stopped, and a stopped cycle only re-arms on a different number: a
        countdown that moved is evidence the stop did not take; one frozen at
        the stamp is the cycle just ended.

        Re-arming (a restart's adoption, or a stop that did not take) also
        arms host state, so the panel's STOP is live -- the same contract as
        telemetry adoption.

        The stop's grace window (_dry_adopt_after) applies here as it does to
        telemetry: a stop pressed seconds into a cycle can land before the
        countdown has decoded, leaving nothing to stamp, so inside the grace
        the countdown gets no vote.

        :param dr: the firmware's dry-remaining seconds, already > 0
        :return bool: True when the countdown counts as a running dry
        """
        prev = getattr(self, "_dryrem_seen", None)
        self._dryrem_seen = (int(dr), _mono(self))
        if self._drying:
            return True
        if _mono(self) < getattr(self, "_dry_adopt_after", 0.0):
            return False
        stamped = getattr(self, "_dryrem_at_stop", None)
        if stamped is not None and int(dr) == int(stamped):
            return False
        # From cold, a number is not a cycle: adopt a dry only once the
        # countdown has TICKED, falling by no more than the elapsed time plus
        # DRYREM_TICK_SLACK_S. A frozen register repeats, and an idle HT can
        # report a discontinuous jump; neither may latch _drying (which refuses
        # loads). A number different from the stop stamp is movement and is
        # shown at once.
        now = _mono(self)
        ticked = False
        if prev is not None:
            fell = int(prev[0]) - int(dr)
            elapsed = max(0.0, now - float(prev[1]))
            ticked = 0 < fell <= elapsed + DRYREM_TICK_SLACK_S
        # The stop stamp keeps its own rule: any number different from the one
        # a stop stamped is evidence the stop did not take, and reaches the
        # panel on the first such frame.
        unstopped = stamped is not None and int(stamped) != int(dr)
        if not (ticked or unstopped):
            return False
        was = int(prev[0]) if prev is not None else int(stamped)
        self._drying = True
        self._dryrem_at_stop = None
        # Log the adoption, so a load refused for drying can be traced to the
        # countdown. Guarded: bare test objects may lack a logger, and that
        # must not decide whether a unit is drying.
        try:
            self.logger.info(
                f"AFC bambu {self.name}: adopting the unit's own dry cycle -- "
                f"its countdown moved {was}s -> {int(dr)}s, so a cycle this "
                f"host did not start is running ({int(dr)}s left)")
        except Exception:
            pass
        return True

    def get_status(self, eventtime: Any = None) -> dict:
        """
        Extend the base unit status with the bridge's slot view.

        :param eventtime: Klipper eventtime, forwarded to super().get_status
        :return dict: unit status including bridge online flag and slots
        """
        status = super().get_status(eventtime)
        latest = self._bridge.latest_status() if self._bridge else None
        # An unclaimed pool bay may not read the roster. Everything below
        # attributes telemetry by matching u["n"] against self.ams_index, and
        # an unclaimed bay keeps the config default 0, so it would match the
        # first real unit on the bus and publish that unit's dry state and
        # chamber readings as its own. Gated once here, where the roster
        # enters: with it stripped every lookup below finds nothing.
        if latest and getattr(self, "pool", False):
            latest = {k: v for k, v in latest.items() if k != "units"}
        status["bridge_online"] = self._unit_online(latest)
        # The link itself. bridge_online answers "is the unit reporting" from
        # the last status frame, so a dead link leaves it stale rather than
        # false. These two describe the transport: whether a port is open, and
        # for how long it has not been.
        try:
            status["bridge_connected"] = (self._bridge.is_connected()
                                          if self._bridge else False)
            _dt = self._bridge.down_since() if self._bridge else None
            status["bridge_down_for"] = (round(time.monotonic() - _dt, 1)
                                         if _dt is not None else None)
        except Exception:
            status["bridge_connected"] = False
            status["bridge_down_for"] = None
        status["ams_index"] = self.ams_index
        # byte[19] of the op-04 reply, published: 04 healthy, 07 stalled, None
        # when the unit has not reported one. Every generation sends it, unlike
        # fault narration (the AMS 1 has none).
        status["unit_state"] = self._unit_state(latest)
        humidity, temperature = unit_env(latest, self.ams_index)
        status["humidity"] = humidity          # %RH, or None if unknown
        # Chamber temperature. The binary protocol carries none (temp_c10 is
        # -1), but a drying AMS streams it in its own telemetry. Treated as
        # stale after 120s so a finished dry cycle does not leave a frozen
        # reading looking live, and keyed by chain index so it can only ever
        # be this unit's own chamber -- see _chamber_record.
        rec = self._chamber_record() if self._bridge is not None else None
        # Resolve the chamber telemetry once, here, so every derived field
        # (temperature, drying, target, state) reads the same record and they
        # agree.
        live, attributable, seen_t = self._chamber_live()
        # Only telemetry produced after the last start/stop, plus a grace of a
        # few reporting intervals (~10s each), says anything about the current
        # cycle. Otherwise a stop would be undone by the next poll, or by the
        # one [AMS_CHMB] line a stopping unit still emits.
        fresh_for_this_cycle = live and seen_t > getattr(
            self, "_dry_adopt_after", 0.0)
        if temperature is None and fresh_for_this_cycle and rec is not None:
            temperature = rec.get("temp")
        status["temperature"] = temperature    # °C, or None when not drying
        # An HT has one bay. The arrays stay SLOTS_PER_UNIT wide so a stray
        # frame naming slot 3 cannot fault; only the published shape is trimmed
        # to unit_slots.
        status["slots"] = self._published_slots()
        # Apply a narrated capacity measurement to the slot that asked.
        self._status_apply_measurements(status)
        # Follower and buffer telemetry, surfaced like an FPS buffer: buff is
        # 0..100 by spring state (100 = compressed, fed). The buff/fstate
        # fields are bridge-wide globals, so they are attributed to the unit
        # that is following, else the one with a lane at the toolhead, else
        # None.
        mine = (self._following_lane is not None) or _unit_tool_loaded(self)
        latest_own = latest if mine else None
        buff = latest_own.get("buff") if latest_own else None
        # Odometer from the 0x03 motion reply, per unit and live at the poll
        # rate. Negative is a reading (the resting state sits slightly below
        # zero); only the firmware's -1 mm unknown sentinel is excluded.
        odom = None
        try:
            u = afcBambuAMS._unit_entry(self, latest)
            v = u.get("odom") if u is not None else None
            if v is not None and int(v) != -1:
                odom = int(v) / 1000.0         # mm -> metres
        except Exception:
            odom = None
        status["odom_m"] = odom
        status["follow_buff"] = buff              # 0..100 fullness
        status["buffer"] = buff                   # alias (FPS-style value)
        status["buffer_state"] = _buffer_state(buff)
        # How many times the firmware has actually decoded the buffer off the
        # wire. 0 means follow_buff is still the firmware's seed value, not a
        # reading. A seed value reads as a satisfied buffer and disables
        # anything gated on it, so surface the count rather than trusting the
        # number alone.
        status["follow_buff_reads"] = latest_own.get("buffn") if latest_own else None
        # Length of the last motion reply the firmware decoded, and the raw 16-bit
        # field before calibration. A length <= 26 means the reply carries no
        # buffer at all on this AMS model; the raw value is what BUFF_POS_FULL /
        # BUFF_POS_EMPTY are calibrated against.
        status["follow_buff_replylen"] = latest_own.get("bufflen") if latest_own else None
        # follow_buff above is the firmware's mapped value, whose calibration
        # is wrong on an AMS HT; follow_buff_raw is the field itself (signed
        # LE) and is the one to trust.
        status["follow_buff_raw"] = latest_own.get("buffraw") if latest_own else None
        status["follow_state"] = latest_own.get("fstate") if latest_own else None
        # 0 means the AMS has never reported a follower state, so follow_state is
        # the firmware's seed (4) rather than a confirmation that it is following.
        status["follow_state_reads"] = latest_own.get("fstaten") if latest_own else None
        # Whether this unit's follower arm has been acknowledged (see
        # _follow_arm_acked); until it is, the bridge re-sends the arm a few
        # times. False while armed-and-following means the arm is not landing;
        # False while idle simply means "not armed".
        status["follow_arm_acked"] = self._follow_arm_acked(latest)
        # The AMS's own reference id, from its "[AMS_COMMON]...ref:N" narration.
        # An AMS HT only acts on a SELECT addressed to this id, so a mismatch
        # here means tray selects are ignored and the unit reports tray:255.
        status["ams_ref"] = latest.get("amsref") if latest else None
        # decode_presence instrumentation: presdrop climbing while presok is
        # flat means a reply arrives and the address/index guard discards it.
        # nexp is how many unit indices the status poll walks; htmask which
        # indices are HT-flagged (the 0x1800 paths gate on it).
        status["poll_nexp"] = latest.get("nexp") if latest else None
        status["ht_mask"] = latest.get("htmask") if latest else None
        # Per-unit: the reply LENGTH decode_presence accepted for THIS unit and
        # the raw byte[9] it read as the slot bitmap. Splits "the unit says it
        # is empty" from "the wrong offset is being read in its reply".
        _u = afcBambuAMS._unit_entry(self, latest) if latest else None
        status["pres_len"] = _u.get("preslen") if _u else None
        status["pres_byte"] = _u.get("presbyte") if _u else None
        status["pres_ok"] = latest.get("presok") if latest else None
        status["pres_drop"] = latest.get("presdrop") if latest else None
        status["pres_addr"] = latest.get("presaddr") if latest else None
        status["pres_want"] = latest.get("preswant") if latest else None
        # Which phase the firmware's op-04 state channel is in. The channel
        # runs at ~148ms and only its mode/ref change; this is that value, and
        # it is the fastest way to see whether a transition actually happened
        # instead of inferring it from narration after the fact.
        _ph = latest.get("phase") if latest else None
        status["ams_phase"] = _ph
        # Keep this in step with ams_phase_t in bambubus.c.
        status["ams_phase_name"] = {
            0: "idle 01/00", 1: "select 01/FF", 2: "drive 03/00",
            3: "arrived 03/00 tail 02", 4: "enter 09/7F", 5: "pre 07/00",
            6: "hold 07/7F", 7: "release 07/00", 8: "done 09/3F",
            9: "error 0F/00",
        }.get(_ph)
        # Why the follower may be standing down (e.g. a manual-off latch).
        status["follow_manual_off"] = bool(
            getattr(self, "_follow_manual_off", False))
        status["follow_when_loaded"] = bool(
            getattr(self, "follow_when_loaded", False))
        # The build actually running on the Pico, as the firmware reports it on
        # the chain reply, so a panel can show which bridge build is running.
        try:
            status["bridge_fw"] = getattr(self._bridge, "_chain_fw", "") or None
        except Exception:
            status["bridge_fw"] = None
        # The AMS's last self-reported stall, and the motor current it came
        # with. Surfaced so a fault is inspectable after the fact, not only at
        # the moment it paused.
        if self._bridge is not None:
            # Scoped to THIS unit: a stall on a chain-mate is not a fault on
            # this lane's load. Both boxed units share 0x0700, so the unit
            # index is what separates them.
            _seq, ftext, famps = self._bridge.last_fault(
                unit=getattr(self, "ams_index", None))
            status["ams_fault"] = ftext or None
            status["ams_motor_amps"] = famps or None
        # True while a stall has the follower held off, waiting for a resume.
        status["follow_fault_hold"] = self._follow_fault_hold
        # Narration accounting. The AMS returns its pending log text in reply to
        # a 1A/02 poll, and an empty reply is ordinary traffic -- so a quiet
        # AFC.log cannot be read as "the unit said nothing". polls climbing with
        # frames flat means it is not answering the log drain at all; frames
        # climbing with texts flat means it is answering and has nothing queued.
        status["ams_narration_polls"] = latest.get("dbgpolls") if latest else None
        status["ams_narration_frames"] = latest.get("dbgframes") if latest else None
        status["ams_narration_texts"] = latest.get("dbgtexts") if latest else None
        # Narration lines the firmware had to cut. Normally 0: a full 174-byte
        # narration frame yields at most 159 characters, which fit. While it
        # reads 0, "the unit never said X" is a claim about the unit; if it
        # climbs, such claims are suspect.
        status["ams_narration_cut"] = latest.get("dbgtrunc") if latest else None
        # Raw result of the MC_ONLINE exchange alone, in a buffer no other poll
        # can overwrite. snap_empty climbing with snap_replies flat means the
        # AMS does not answer the log drain within REPLY_TIMEOUT_US; the
        # reverse means it answers and the reply is being discarded.
        status["ams_snap_replies"] = latest.get("snapn") if latest else None
        status["ams_snap_replies_p1"] = latest.get("snapn1") if latest else None
        status["ams_snap_empty"] = latest.get("snapempty") if latest else None
        status["following"] = (self._following_lane.name
                               if self._following_lane is not None else None)
        # Drying is host state, set by AFC_BAMBU_HEATER_START. Live chamber
        # telemetry only streams while a cycle runs, so its presence is
        # evidence from the unit itself; trust it over the flag, so a cycle
        # this host did not start (e.g. across a Klipper restart) is adopted.
        if fresh_for_this_cycle:
            self._dry_seen_live = True
            self._drying = True   # adopt it, so a STOP from the panel is armed
        elif (self._drying and attributable
                and getattr(self, "_dry_seen_live", False) and not live):
            # The unit reported for this cycle and has now gone silent past the
            # staleness window: the cycle is over (finished its timer or was
            # stopped at the unit).
            self._drying = False
            self._dry_seen_live = False
            self.logger.info(
                f"AFC bambu {self.name}: drying finished (the unit stopped "
                f"reporting chamber telemetry)")
        # A refused cycle is not a cycle: _drying is set optimistically at
        # HEATER_START, so a "[AMS_CHMB]err" on record for this attempt clears
        # it, unless the unit has already reported live for the cycle. Read
        # the cycle stamps before anything tests them.
        started = getattr(self, "_dry_started_at", None)
        minutes = getattr(self, "_dry_minutes", 0) or 0

        refused_now = self._bridge_call_arg("last_dry_error", self.ams_index)
        settled = (started is not None
                   and (_mono(self) - started) > DRY_REFUSE_GRACE)
        if (self._drying and refused_now and settled
                and not getattr(self, "_dry_seen_live", False)):
            self._drying = False
            # The cycle stamps are kept: if the unit later starts reporting
            # after all, the adoption above sets _drying again, and dry_rotate
            # needs _dry_started_at to use the commanded value rather than the
            # unit's echo (which always reports rotate:0). Nothing reads the
            # stamps while _drying is False, and the next start overwrites them.
            if not getattr(self, "_dry_refusal_logged", False):
                self._dry_refusal_logged = True
                self.logger.info(
                    f"AFC bambu {self.name}: not drying -- the unit refused: "
                    f"{refused_now}")
        elif not refused_now:
            self._dry_refusal_logged = False
        self._drop_silent_dry(refused_now, started)
        status["drying"] = bool(self._drying)
        # Whether the unit has confirmed this cycle with live telemetry.
        # _drying is optimistic from AFC_BAMBU_HEATER_START, and the unit
        # self-checks before telemetry starts; this lets a panel show
        # "starting" in that gap.
        status["dry_confirmed"] = bool(getattr(self, "_dry_seen_live", False))
        # Seconds left in the commanded cycle, or None.
        #
        # None is not zero: a cycle adopted from live chamber telemetry has no
        # known duration, and 0 would render as "finishing now".
        remaining = None
        if self._drying and started is not None and minutes:
            remaining = max(0, int(minutes * 60 - (_mono(self) - started)))
        status["dry_remaining"] = remaining
        status["dry_minutes"] = minutes or None
        # Whether the running cycle is spinning the spool. The unit's own
        # [AMS_CHMB] echo wins (it survives a restart and covers cycles this
        # host did not start); fall back to the recorded command otherwise.
        cfg = None
        try:
            cfg = self._bridge_call_arg("last_dry_cfg", self.ams_index)
        except Exception:
            cfg = None
        if cfg and cfg.get("dur"):
            status["dry_minutes"] = int(cfg["dur"])
            if not remaining and started is not None:
                status["dry_remaining"] = max(
                    0, int(cfg["dur"] * 60 - (_mono(self) - started)))
        # The AMS's own countdown outranks everything above. The 0x3C
        # telemetry reply carries dry-remaining in seconds (payload[33:35]),
        # reported per unit as "dryrem". It survives Klipper restarts and knows
        # the duration of a cycle this host never started -- but it freezes
        # whenever 0x3C replies stop decoding, so it may only re-arm a stopped
        # cycle on evidence it is still ticking: see _dryrem_says_drying.
        try:
            u = afcBambuAMS._unit_entry(self, latest)
            dr = u.get("dryrem") if u is not None else None
            if (dr is not None and int(dr) > 0
                    and self._dryrem_says_drying(int(dr))):
                status["dry_remaining"] = int(dr)
                status["drying"] = True
        except Exception:
            pass
        # Chamber temp/humidity from the same 0x3C decode, preferred over the
        # narration-scraped values when present.
        #
        # A first-gen AMS always leaves temperature None here: it never replies
        # to 0x3C, and no other reply it gives carries degrees (see
        # decode_3c_reply() in bambubus.c). Its humidity comes from the 0x04
        # motion-long reply, which it does answer.
        try:
            u = afcBambuAMS._unit_entry(self, latest) or {}
            et, eh = u.get("envt"), u.get("envh")
            if et is not None and int(et) > 0:
                status["temperature"] = float(int(et))
            # Fill-in only: the 0x04-reply humidity is preferred, and these
            # are likely different sensors, so one does not replace the other.
            if (status.get("humidity") is None and eh is not None
                    and 0 <= int(eh) <= 100):
                status["humidity"] = int(eh)
        except Exception:
            pass

        # ROTATE is the one field where the command outranks the echo: the unit
        # echoes rotate:0,0 even for ROTATE=1. The echo is used only for a
        # cycle this host did not start.
        if self._drying and started is not None:
            status["dry_rotate"] = 1 if getattr(self, "_dry_rotate", 0) else 0
        elif cfg:
            status["dry_rotate"] = 1 if cfg.get("rotate") else 0
        else:
            status["dry_rotate"] = None
        # What the heater is doing, for a UI. All three come from the AMS's own
        # [AMS_CHMB] telemetry, which only streams while a cycle is running, so
        # they share the same 120s staleness rule as the chamber temperature --
        # a finished cycle must not leave a frozen target looking live.
        status["has_heater"] = self.has_heater
        status["ams_model"] = self.ams_model
        status["dry_max_temp"] = self.dry_max_temp
        dry_target = None
        dry_state = None
        if fresh_for_this_cycle and rec is not None:
            dry_target = rec.get("target")
            dry_state = rec.get("state")
        if dry_target is None and status.get("drying"):
            # No live telemetry but a cycle is running. On a boxed dry the
            # bridge's receiver goes deaf to the whole bus while the heater
            # draws, so telemetry cannot arrive -- use the settings the unit
            # echoed before heating began, else the commanded temperature.
            if cfg and cfg.get("tmpr"):
                dry_target = cfg.get("tmpr")
            elif getattr(self, "_dry_temp", None):
                dry_target = self._dry_temp
        status["dry_target"] = dry_target      # C the AMS is driving to
        status["dry_state"] = dry_state        # AMS's own chamber state code
        # The unit's external 24 V jack, as the unit reports it: ~24 with the
        # adapter in, ~0.5 without. Only present while the chamber controller
        # is streaming telemetry; there is no idle source for it (see
        # docs/ams2_pro_protocol.md). Published so a heater interlock can read
        # it without reaching into the bridge's internals.
        _ad = (rec or {}).get("ad_v") if isinstance(rec, dict) else None
        status["ext_supply_v"] = _ad
        status["ext_supply_seen"] = (rec or {}).get("seen") \
            if isinstance(rec, dict) else None
        # Why a Dry button should be greyed right now, or None. The same string
        # AFC_BAMBU_HEATER_START would answer with, published so the refusal
        # can be shown before the press.
        try:
            status["dry_blocked"] = (self._bus_supply_conflict()
                                     if not self._drying else "") or None
        except Exception:
            status["dry_blocked"] = None
        # Why the unit last declined to dry, in its own words. Not gated on
        # self._drying, which is host intent set whether or not the AMS
        # accepted; the unit clears it when it reports heating or a self-check.
        status["dry_error"] = (self._bridge_call_arg("last_dry_error",
                                                     self.ams_index)
                               or getattr(self, "_dry_host_refusal", "")
                               or None)
        # ...unless the unit is demonstrably drying. The HT narrates warnings
        # ("ams-ht shell open!") through the same err, channel it uses for
        # refusals, so with a cycle running the stored text is a condition
        # report, not a decline: it moves to dry_note, which the panel renders
        # as "Drying -- <note>".
        status["dry_note"] = ""
        if status.get("dry_error") and status.get("drying") and (
                dry_state == 2 or status.get("dry_remaining")):
            status["dry_note"] = status["dry_error"]
            status["dry_error"] = ""
        return status

    # -- transport primitives --


    def _status_apply_measurements(self, status: dict) -> None:
        """Apply a narrated capacity measurement to the slot that asked for it.
        Split out of get_status; mutates self state and the status dict in
        place. Self-contained: it reads no get_status local but `status`, and
        produces none the rest of get_status uses."""
        # Apply a narrated capacity measurement to the slot that asked for it.
        #
        # The AMS measures the spool at the end of a capscan and narrates the
        # percent ("P:84%"), but a boxed unit does not persist it to its slot
        # record. The narrated number is the measurement, so the bridge
        # captures it and the module applies it here: the slot's remain_pct
        # and the lane's weight both become the measured value.
        try:
            # Expire the waits that ran out before reading the marker, on every
            # pass -- the calls below are reached only with a reading in hand,
            # so a bay whose scan measured nothing would otherwise wait forever.
            afcBambuAMS._cap_live_pending(self)
            pend = getattr(self, "_cap_pending_slot", None)
            # A measurement nobody claimed: log any reading never adopted,
            # once, with the state that would have decided it. Only when no
            # bay is waiting (the block below decides and explains a reading
            # when one is), and never on a pool spare (readings are stored by
            # bus address, which spares share).
            try:
                _m = self._bridge.last_cap_measure(
                    getattr(self, "dry_dev_addr", 0)) if self._bridge else None
                if (_m and _m.get("t") is not None
                        and not getattr(self, "pool", False)
                        and not afcBambuAMS._cap_live_pending(self)
                        and _m.get("t") != getattr(self, "_cap_adopted_t", None)
                        and _m.get("t") != getattr(self, "_cap_orphan_t", None)):
                    self._cap_orphan_t = _m.get("t")
                    self.logger.debug(
                        f"AFC bambu {self.name}: unclaimed {_m.get('pct_raw')}% "
                        f"reading (t={_m.get('t')}, restored={_m.get('restored')}"
                        f", pend={pend}, t0={getattr(self, '_cap_pending_t0', 0.0)}"
                        f", adopted_t={getattr(self, '_cap_adopted_t', None)})")
            except Exception:
                pass
            # A calibration's end is reported as a verdict code
            # ("Calibration rst:0" = completed). On an AMS HT the measured
            # percent lands on the tag and arrives through the next
            # filament-info read of the slot, so the verdict is what shows a
            # calibration finished.
            if pend is not None and self._bridge is not None:
                getc = getattr(self._bridge, "last_ht_cali", None)
                cali = (getc(self.ams_index) if callable(getc) else None)
                if cali and cali.get("t", 0) > getattr(self, "_cap_pending_t0", 0.0):
                    if cali.get("t", 0) != getattr(self, "_cap_cali_seen_t", None):
                        self._cap_cali_seen_t = cali.get("t", 0)
                        rst = cali.get("rst")
                        # Re-read the slot: the measurement lands after the tag
                        # read, and the firmware stops filling a bay once
                        # info_valid. rst:0 only means the cycle ended, a
                        # cached-tag reinsert ends it without measuring, so the
                        # narrated percent (which arrives first) decides.
                        measured = None
                        try:
                            measured = self._bridge.last_cap_measure(
                                getattr(self, "dry_dev_addr", 0))
                        except Exception:
                            measured = None
                        did_measure = bool(
                            measured
                            and measured.get("t", 0)
                               > getattr(self, "_cap_pending_t0", 0.0))
                        if rst == 0 and did_measure:
                            try:
                                self._bridge.send({"cmd": "reread",
                                                   "unit": self.ams_index,
                                                   "slot": pend})
                            except Exception:
                                pass
                        if rst == 0 and not did_measure:
                            lane = self._lane_for_slot(pend)
                            # One-edge retry. A native insert measure can
                            # detect the tag's first pass and not its second
                            # (rst:0, no percent); this is the one place that
                            # knows it failed, so retry once through
                            # _run_calibrate (the armed script), latched by
                            # _auto_cali_slot until the next physical insert.
                            # Covers every model in _RETRY_MEASURE_MODELS; the
                            # firmware's own auto-rescan skips the AMS 2, whose
                            # measure needs the follower armed, which the host
                            # orchestrates.
                            retried = False
                            said = False
                            if (getattr(self, "measure_on_insert", False)
                                    and str(getattr(self, "ams_model", "")
                                            ).lower() in _RETRY_MEASURE_MODELS
                                    and lane is not None
                                    and getattr(self, "_auto_cali_slot",
                                                None) != pend
                                    and not getattr(self, "_drying", False)
                                    and getattr(self, "_cali_epoch", None)
                                        != getattr(self, "_cap_pending_t0",
                                                   0.0)):
                                self._auto_cali_slot = pend
                                why = afcBambuAMS._run_calibrate(
                                    self, lane, pend)
                                if why is None:
                                    retried = True
                                    self.logger.info(
                                        f"AFC bambu {self.name}: slot {pend} "
                                        f"one-edged its insert measurement "
                                        f"(rst:0, no percent narrated) -- "
                                        f"measure_on_insert is retrying it "
                                        f"once")
                                else:
                                    said = True
                                    self.logger.info(
                                        f"AFC bambu {self.name}: slot {pend} "
                                        f"one-edged its insert measurement "
                                        f"and the retry could not start "
                                        f"({why}) -- AFC_BAMBU_CAPSCAN "
                                        f"UNIT={self.name} LANE={lane.name} "
                                        f"runs it by hand")
                            if retried:
                                said = True
                            elif not said and (
                                    getattr(self, "measure_on_insert", False)
                                    and str(getattr(self, "ams_model", "")
                                            ).lower() in _HT_MODELS
                                    and getattr(self, "_auto_cali_slot",
                                                None) == pend):
                                # The retry one-edged too (the latch is from
                                # this insert): say so and stop.
                                said = True
                                self.logger.info(
                                    f"AFC bambu {self.name}: slot {pend} "
                                    f"one-edged again on the retry -- giving "
                                    f"up for this insert. AFC_BAMBU_CAPSCAN "
                                    f"UNIT={self.name}"
                                    + (f" LANE={lane.name}"
                                       if lane is not None else "")
                                    + " runs it by hand")
                            if not said:
                                self.logger.info(
                                    f"AFC bambu {self.name}: slot {pend} "
                                    f"finished its cycle but "
                                    f"never narrated a percent (rst:0) -- it "
                                    f"took only ONE pull, so there was no "
                                    f"second edge to measure from. That is "
                                    f"geometric and it follows the SPOOL, not "
                                    f"the bay: the second edge needs a full "
                                    f"revolution of filament past the reader, "
                                    f"so a large or full reel can come up "
                                    f"short. Running it again often lands "
                                    f"it. Note the mechanism above was "
                                    f"characterised BEFORE the receive buffer "
                                    f"was fixed (AFC-2.64), when a measurement "
                                    f"that did happen could go unheard -- so "
                                    f"it is worth re-testing rather than "
                                    f"trusted; see docs/CAPTURE_FINDINGS.md.")
                        else:
                            self.logger.info(
                                f"AFC bambu {self.name}: slot {pend} calibration "
                                + ({0: "completed (rst:0) -- re-reading the bay "
                                       "to collect the measured percent",
                                    1: "refused (rst:1) -- capacity not enabled "
                                       "for this tray",
                                    4: "aborted (rst:4) -- stalled during "
                                       "calibration"
                                    }.get(rst, f"returned rst:{rst}")))
            if pend is not None and self._bridge is not None:
                meas = None
                try:
                    meas = self._bridge.last_cap_measure(
                        getattr(self, "dry_dev_addr", 0))
                except Exception:
                    meas = None
                # Log why a measurement was not taken up. Three gates stand
                # between a narrated percent and the lane: pend set, the
                # reading newer than its bay's window, and not a flash
                # restore; each logs once per distinct reading when it
                # declines.
                # Whose reading this is decides which window it must be newer
                # than: bays open their windows at different times, so the
                # owner is found before the freshness gate.
                owner = (afcBambuAMS._cap_owner_of(self, meas)
                         if meas else None)
                own_t0 = getattr(self, "_cap_pending_t0", 0.0)
                if owner is not None:
                    own_t0 = afcBambuAMS._cap_live_pending(self).get(
                        owner, own_t0)
                # Not for the reading this unit already adopted: the bridge
                # keeps the last reading per address indefinitely, so the next
                # scan's window always sees it. The log names the reading by
                # its own tray -- the save line's, or the one a flash restore
                # names ("odom load from flash 1,...") -- and by the waiting
                # bay only when it names none or names that bay, so one bay's
                # figure is not reported as another's.
                if (meas and not (meas.get("t", 0) > own_t0)
                        and meas.get("t") != getattr(self, "_cap_adopted_t",
                                                     None)):
                    if getattr(self, "_cap_stale_t", None) != meas.get("t"):
                        self._cap_stale_t = meas.get("t")
                        _pct = meas.get("pct_raw")
                        _tray = meas.get("save_tray")
                        if _tray is None:
                            _tray = meas.get("tray")
                        if owner is not None and _tray in (None, owner):
                            _what = (f"slot {owner} has a {_pct}% reading "
                                     f"but it is")
                        elif _tray is not None:
                            _what = f"tray {_tray}'s {_pct}% reading is"
                        else:
                            _what = (f"the last {_pct}% reading (no tray "
                                     f"named) is")
                        self.logger.debug(
                            f"AFC bambu {self.name}: {_what} not newer than "
                            f"this capscan (meas t={meas.get('t')}, "
                            f"window t0={own_t0}) -- not adopted")
                if meas and meas.get("t", 0) > own_t0:
                    # Prefer the unclamped reading (fresh spools measure over
                    # 100%). A flash restore ("odom load from flash") is the
                    # previous spool's measurement, not this one's, so it is
                    # not adopted and `pend` stays set for the live reading.
                    if meas.get("restored"):
                        # Say it once per restore, not once per status frame.
                        if getattr(self, "_cap_restore_t", None) != meas.get("t"):
                            self._cap_restore_t = meas.get("t")
                            self.logger.debug(
                                f"AFC bambu {self.name}: slot {pend} reported a "
                                f"flash-restored {meas.get('pct_raw')}% -- that "
                                f"is the previous spool's figure for this bay, "
                                f"not this spool's; waiting for the live "
                                f"measurement")
                    elif owner is None:
                        # Not attributable by narration: the unit named a bay
                        # that is not waiting, or several are waiting and it
                        # named none. Guessing could put one reel's reading on
                        # another's spool; the firmware's per-bay stamp still
                        # attributes it, so nothing is lost. Logged once per
                        # reading.
                        if getattr(self, "_cap_unattr_t", None) != meas.get("t"):
                            self._cap_unattr_t = meas.get("t")
                            _p = sorted(afcBambuAMS._cap_live_pending(self))
                            self.logger.debug(
                                f"AFC bambu {self.name}: a "
                                f"{meas.get('pct_raw')}% reading names tray "
                                f"{meas.get('save_tray')} and the bays waiting "
                                f"are {_p} -- not attributing it by narration; "
                                f"the firmware's per-bay stamp decides")
                    else:
                        pct = int(meas.get("pct_raw") or meas.get("pct") or 0)
                        # Pass a sequence identity: unsequenced,
                        # _adopt_measured_remain refuses to overwrite a slot
                        # that already holds a measurement, so this path could
                        # never adopt twice. The firmware stamps
                        # meas_pct/meas_seq onto the HT's slot record but not a
                        # boxed unit's, so narration is the boxed units' only
                        # route. meas["t"] (one timestamp per measurement, from
                        # the bridge) is the identity; it is only ever compared
                        # for equality with the last seq seen, so mixing it
                        # with the slot record's integer seq is safe, and the
                        # value check still stops a measurement being
                        # announced twice.
                        self._adopt_measured_remain(owner, pct, "capscan",
                                                    seq=meas.get("t"))
                        afcBambuAMS._cap_close_pending(self, owner)
                        self._cap_adopted_t = meas.get("t")
                        # Measurement in, operation over: hand the bus back so
                        # the next unit can start. The claim also lapses on the
                        # unit's own cycle-end marker, so a failed measurement
                        # cannot wedge the bus -- this is the clean path, not
                        # the only one.
                        rel = getattr(self._bridge, "release_bus", None)
                        if callable(rel):
                            rel(self.name)
            # Nothing pending: reset the one-shot insert-retry latch.
            if pend is None:
                self._auto_cali_slot = None
            # There is no host-driven insert auto-measure; AFC_BAMBU_CAPSCAN
            # measures on demand.
            #
            # The published percent is read from the lane, not re-applied onto
            # it. A measurement stops being true once the spool feeds, and the
            # tag's remain is never written back, while the lane's grams are
            # kept current (measurement, then extrusion, then AFC.var.unit or
            # Spoolman across a restart). So the percent is worked back out of
            # the grams through the same model (see _lane_remain_pct). A bay
            # the lane cannot answer for (no tag nominal, or no lane) falls
            # back to a held measurement, else its record. A lane at 0 g does
            # answer -- 0 -- and a held measurement must not outrank it. This
            # also survives a restart: the percent lives only in memory, the
            # grams persist, and remain_pct 0 means "never measured", not empty.
            #
            # Read through _held_measurements: the memo lives on the delegate.
            held_pct = afcBambuAMS._held_measurements(self)
            # Repair both the stored slots and the copies being published:
            # _published_slots() hands out dict copies that get_status took
            # before calling this, so fixing only self._slots would not reach
            # this poll.
            targets = list(self._slots or []) + list(status.get("slots") or [])
            for sl in targets:
                idx = sl.get("index")
                if idx is None or not sl.get("present"):
                    continue
                pct = afcBambuAMS._lane_remain_pct(self, idx, sl)
                if pct is not None:
                    sl["remain_pct"] = pct
                elif idx in held_pct:
                    # The floored figure, capped at 100 (see _shown_remain_pct).
                    sl["remain_pct"] = afcBambuAMS._shown_remain_pct(
                        self, idx, held_pct[idx])
        except Exception as e:
            self.logger.debug(
                f"AFC bambu {self.name}: capscan/calibrate/follower status: {e}")
    def _slot_of(self, lane: Any) -> Optional[int]:
        """
        Return the 0-based AMS slot for a lane, or None if unmapped.

        Never raises: a unit with no map at all answers None. It is read on
        the unload path (accept_tray_release), where a lookup that only
        shortens a wait must not be able to abort the unload; None leaves the
        wait on its other signals.

        :param lane: The AFC lane object
        :return Optional[int]: the slot index, or None
        """
        return (getattr(self, "_slot_map", None)
                or {}).get(getattr(lane, "name", None))

    def fps_buffer_value(self) -> Optional[float]:
        """
        AMS buffer as a 0.0..1.0 FPS/PSF reading for the virtual ADC pin.

        This is an analog buffer, so it follows the FPS/PSF convention that
        AFC_buffer documents at the top of its FPS driver:

            0.1 (low)  -> stretched / tension    -> increase feed
            0.5 (mid)  -> centred / ideal
            0.9 (high) -> compressed / pushing   -> decrease feed
            aliases: max_tension -> low_point, max_compression -> high_point

        High is compressed. The sign matters -- these callers invert with it:

          - advance_state (smoothed > set_point + deadband/2), which is what
            get_toolhead_pre_sensor_state() returns when tool_start is
            "buffer"
          - buffer_triggered, the endstop-free load check
          - the pre-feed guard, which refuses to load into an empty toolhead
          - buffer ramming

        On this polarity an unloaded buff=1 reads 0.01 (tension) and a loaded,
        self-centred buff=56..60 reads ~0.58, just above the 0.5 set_point.

        :return Optional[float]: 0.0..1.0, or None if no reading yet
        """
        latest = self._bridge.latest_status() if self._bridge else None
        if not latest:
            return None
        b = latest.get("buff")
        if b is None:
            return None
        # The seed is not a reading. The firmware seeds its buffer field at
        # 100 (1.0 below, fully compressed), so a bus that has never answered
        # would publish a satisfied buffer. `buffn` is the firmware's count of
        # how many times the field has been written: zero means the seed. An
        # absent key is "cannot tell", not zero, and keeps the reading.
        if latest.get("buffn") == 0:
            return None
        v = b / 100.0                      # compressed(buff 100)->1.0, empty(0)->0.0
        return 0.0 if v < 0.0 else 1.0 if v > 1.0 else v

    def _tool_loaded_lane(self) -> Optional[Any]:
        """
        Return a lane on this unit threaded to the toolhead and belonging to
        the active extruder, or None.

        Used to auto-arm the follower. On a toolchanger several lanes can be
        tool_loaded at once -- one per toolhead -- but only one extruder is
        active (receiving E moves); the rest need no follower. A loaded lane
        is skipped only when another extruder is positively known to be
        active: uncertainty must never strip a follower, because a real
        printer holds a loaded tray unconditionally.

        "Active" is Klipper's current extruder (AFC's get_current_extruder),
        not on_shuttle(): a docked toolhead can be the active extruder during
        async/pre-load. When the current extruder is unknown, a loaded lane is
        returned.

        :return Optional[Any]: a tool-loaded, slot-mapped, active lane, or None
        """
        # AFC's own answer first: the lane AFC believes is threaded to the
        # toolhead right now, which survives a G28 and a Klipper restart
        # (current_lane does not). If it is on this unit, follow it.
        try:
            # afc.current, not afc.current_load: the status dict publishes
            # afc.current under the key "current_load" (AFC.py get_status), and
            # "current_lane" is afc.current_loading.
            named = getattr(self.afc, "current", None)
            if named and named in self.lanes and self._slot_of(
                    self.lanes[named]) is not None:
                return self.lanes[named]
        except Exception:
            pass
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
            # No on_shuttle() check: a docked toolhead answers False (e.g.
            # after a G28). Only positive knowledge of a different active
            # extruder (above) skips a loaded lane.
            return lane
        return None

    def _committed_lanes(self) -> list:
        """
        Lanes on this unit with filament in the feeder, or threaded to a
        toolhead. These are the lanes that stop the spools turning.

        The unit will not rotate with filament inserted: that is the AMS's own
        precondition -- the bays must be empty and the spools free to spin.
        ``loaded_to_hub`` is that test here (on a Bambu lane it is assigned
        from the slot's ``present``, i.e. filament at the feeder), so this
        gate fires exactly when rotation is impossible and the operator is
        told why. Reading tool_loaded alone would send rotate=1 to a unit
        that cannot act on it, a silent no-op.

        The dry frame matches the printer's, and the unit echoes rotation as
        accepted:

            [AMS_CHMB]rotate:1,0, pw_lim:100, cool_down:0,55, dur:480
            [AMS_LINK]ams_ctc,ams128->MC,rotate_l_p:255,cool_down_t:55

        When the spools do not turn, look at the bays and the rollers, not at
        the bus.

        :return list: lanes that block rotation (empty if none)
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
        :return tuple: (ok, slot), ok False when the lane isn't mapped
        """
        slot = self._slot_of(lane)
        if slot is None or self._bridge is None:
            return (False, -1)
        self._bridge.send(
            {"cmd": "select", "unit": self.ams_index, "slot": slot})
        return (True, slot)

    # `mm` is not a distance. The AMS bus carries no distance and no speed --
    # a load drive is one frame repeated until the sender stops (see bb_feed).
    # The firmware turns `mm` and `mmps` into a duration:
    #
    #     ms = (mm / mmps) * 2000 + 5000      2x nominal + 5s slack
    #
    # That deadline is a runaway guard for a crashed host, not what ends a
    # move: the caller watches its sensors and sends stop. Without that (a bare
    # AFC_BAMBU_FEED) the motor runs to the deadline, far past `mm` (MM=20 at
    # 20 mm/s is a 7 s run, well over a metre). So AFC_BAMBU_FEED is a
    # developer tool and is left out of the operator cheat sheet.
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
        AP2 sync heartbeat (the ``assist`` command), not by blind mode-03 feed,
        which has no stop condition, over-feeds and stall-retracts the
        filament off the toolhead sensor; the follower self-stops at center,
        so it holds pressure without fighting the extruder.

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

    def _check_link_loss(self) -> None:
        """
        Pause the print when the bridge has gone silent.

        Routed through _raise_ams_fault rather than pausing directly, so a lost
        link behaves exactly like a stall: the assist drops, the follower is
        held off, and the pause only happens when a print is actually running
        (see _raise_ams_fault).

        Only while a lane of this unit is threaded to the toolhead. A dead
        bridge with nothing loaded here is a problem to report, not a reason to
        stop the machine.
        """
        if not self.link_loss_pause_s or self._bridge is None:
            return
        quiet = self._bridge.silent_for()
        if quiet is None or quiet < self.link_loss_pause_s:
            # Heard from: re-arm for the next outage. The latch is cleared here
            # rather than on resume, so a link that drops, recovers and drops
            # again pauses both times.
            self._link_loss_paused = False
            return
        if self._link_loss_paused:
            return                      # already reported this outage
        lane = self._tool_loaded_lane()
        if lane is None:
            return
        try:
            if not self.afc.function.in_print():
                return
        except Exception:
            return
        self._link_loss_paused = True
        self._raise_ams_fault(
            lane,
            f"AFC bambu {self.name}: the bridge has sent nothing for "
            f"{quiet:.0f}s while printing. Pausing: the extruder would go on "
            f"pulling against an AMS that is no longer being driven. The link "
            f"reconnects by itself -- resume once it is back.")

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
        # Kept so the reload on resume is aimed at the slot that stalled, not
        # whatever is loaded by the time the print resumes.
        self._fault_lane = lane
        # Arm the declaration latch against THIS fault. Cleared here, set by
        # any status frame carrying byte[19] == 0x07, so "declared" always
        # means "since this fault", never a leftover from the last one.
        self._declared_since_fault = False
        # Same for the odometer range: how far the AMS moves filament during
        # THIS fault's recovery is what says where the jam is.
        self._odom_lo = None
        self._odom_hi = None
        try:
            self.set_feed_assist(lane, False)
        except Exception as e:
            # A fault report must never be able to break the follower tick.
            self.logger.debug(
                f"AFC bambu {self.name}: could not drop assist on stall: {e}")
        # Pausing runs the PAUSE macro, which moves Z. Outside a print -- or
        # before the axes are homed -- that move raises "Must home axis first",
        # and inside the follower's reactor timer an escaped exception shuts
        # down all of Klipper. So only pause when a print is running, and never
        # let the pause path throw past here.
        try:
            printing = bool(self.afc.function.in_print())
        except Exception:
            printing = False
        try:
            self.afc.error.AFC_error(msg, pause=printing)
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: stall fault reported but the pause "
                f"could not run ({e}); left the follower held. Original: {msg}")
        # A Bambu fault paused this print, so the toolhead is about to be (or
        # already is) empty and RESUME alone would print air. Flagged here,
        # not inside _maybe_auto_recover, so the reload-on-resume also works
        # with auto_error_recovery off (the default).
        if printing:
            self._resume_needs_reload = True
            self._maybe_auto_recover(lane)

    def _maybe_auto_recover(self, lane: Any) -> None:
        """
        Run the printer's own recovery for a stalled lane, if enabled.

        A stock printer cuts at the toolhead, retracts the severed filament
        (~12 s of op-03 03/FF), drives a reload attempt (~70 s of 03/00), and
        only parks for a human if that fails. The cut never appears on the AMS
        bus because it happens at the toolhead, which is why the retract can
        run 12 s without fighting the nozzle.

        Uses AFC's own lane routines rather than driving the bus: the unload
        macro already does cut -> retract -> unload and the load macro does the
        reload, so reimplementing either would duplicate the cutter logic.

        Off by default (auto_error_recovery), since it moves the toolhead and
        filament unasked. When on, a recovery that refills the toolhead also
        resumes the print, so an unattended print can continue.

        Guards:
          * One attempt per fault. The AMS is already retrying inside its own
            70 s window; a second retry on top fights it and grinds filament.
          * At most auto_error_recovery_limit automatic resumes per print. A
            jam the recovery cannot clear would otherwise loop. A manual
            resume resets the count.
          * Never inline. This runs inside the follower's reactor timer, where
            a blocking macro loses clock sync and "Timer too close" shuts down
            every MCU. The work goes to a reactor callback that only queues
            g-code.

        :param lane: The lane that stalled
        """
        if not getattr(self, "auto_error_recovery", False):
            return
        if getattr(self, "_auto_recover_armed", False):
            return                          # one attempt per fault
        self._auto_recover_armed = True
        self._in_auto_recover = True
        name = getattr(lane, "name", None)
        if not name:
            # _run is never scheduled, so _done never clears these; clear
            # them here so a later fault can arm.
            self._in_auto_recover = False
            self._auto_recover_armed = False
            return
        self.logger.info(
            f"AFC bambu {self.name}: auto error recovery for {name} -- "
            f"unloading (cut, retract, unload) and reloading the same spool, "
            f"then resuming the print if the reload takes. "
            f"Disable with auto_error_recovery: False.")

        def _declared() -> bool:
            """
            Whether the unit has declared it gave up at any point since this fault.

            Reads a latch rather than the current frame: byte[19] == 0x07 is
            present in only a minority of frames while the unit is parked, so
            a single sample at the end of the attempt usually misses it.

            _on_status sets this on any frame carrying it;
            _raise_ams_fault clears it when a new fault is armed.
            """
            return bool(getattr(self, "_declared_since_fault", False))

        def _paused() -> bool:
            """Whether the print is paused, by either authority that knows."""
            try:
                return bool(self.afc.function.is_paused())
            except Exception:
                try:
                    return bool(self.printer.lookup_object(
                        "pause_resume").is_paused)
                except Exception:
                    return False

        def _done(rv: bool) -> bool:
            """
            Clear the in-progress flag on every exit from the attempt.

            _in_auto_recover must be cleared on every path, including failures;
            left set, it suppresses the re-arm on the next fault.

            :param rv: the value to hand back to the caller
            :return: rv, unchanged
            """
            self._in_auto_recover = False
            return rv

        def _run(eventtime: float, _n: str = name) -> None:
            """
            Reactor callback: queue the recovery unload+reload g-code.

            :param eventtime: reactor time of this firing
            :param _n: the lane name, captured at arm time
            """
            # Queue only. AFC's own macros own the cutter and the toolhead.
            try:
                # TOOL_UNLOAD, not UNSET_LANE_LOADED: UNSET only changes AFC's
                # state and moves no filament, so the severed strand would stay
                # in the toolhead and the reload would drive a second strand
                # into an occupied path. TOOL_UNLOAD cuts, retracts and unloads.
                self.gcode.run_script(
                    f"TOOL_UNLOAD LANE={_n}\nCHANGE_TOOL LANE={_n}")
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: auto error recovery for {_n} "
                    f"could not run ({e}); the lane is still paused and held. "
                    f"Recover it by hand.")
                return _done(self.afc.reactor.NEVER)
            # One attempt only: the AMS retries by itself, and the reload inside
            # unit_load_lane already retries over two recovery rounds.
            if _declared():
                self.logger.warning(
                    f"AFC bambu {self.name}: {_n} -- the unit has given up "
                    f"(state 7). Parked; the print stays PAUSED. Clear the jam "
                    f"and resume. {self._jam_location()}".rstrip())
                return _done(self.afc.reactor.NEVER)
            # Resume only if the reload actually filled the toolhead; the
            # RESUME wrap reloads an empty lane first and refuses to continue
            # into an empty toolhead.
            ln = self.lanes.get(_n)
            loaded = bool(getattr(ln, "tool_loaded", False)) if ln else False
            if not loaded:
                # The unit already retried on its own and is held in error,
                # waiting to be told to load. Report it; do not retry.
                self.logger.warning(
                    f"AFC bambu {self.name}: {_n} did NOT reload -- the AMS is "
                    f"HELD IN ERROR and will not try again on its own. Clear "
                    f"the jam, then press resume: that is what tells it to load."
                    + (f" {self._jam_location()}" if self._jam_location() else "")
                    + ("" if _paused() else
                       " NOTE: the print is no longer paused and the toolhead "
                       "is empty."))
                return _done(self.afc.reactor.NEVER)
            # The reload took, so drop the reload the resume wrap would
            # otherwise owe and re-arm: this fault is dealt with, and a later
            # one during the same print deserves its own attempt.
            self._resume_needs_reload = False
            self._auto_recover_armed = False
            if not _paused():
                self.logger.info(
                    f"AFC bambu {self.name}: {_n} is reloaded and ready; the "
                    f"print was not paused, so there is nothing to resume.")
                return _done(self.afc.reactor.NEVER)
            limit = getattr(self, "auto_error_recovery_limit", 0)
            resumed = getattr(self, "_auto_resume_count", 0)
            if resumed >= limit:
                self.logger.warning(
                    f"AFC bambu {self.name}: {_n} is reloaded and ready, but "
                    f"this print has already been resumed automatically "
                    f"{resumed} time(s) "
                    f"(auto_error_recovery_limit: {limit}). THE PRINT IS STILL "
                    f"PAUSED -- a jam that keeps coming back needs a person. "
                    f"Press resume when you are ready; that also clears the "
                    f"count.")
                return _done(self.afc.reactor.NEVER)
            self._auto_resume_count = resumed + 1
            try:
                # RESUME re-enters _reload_before_resume, which returns at once
                # because the lane is fed again. Queued like the unload above:
                # never run inline from the follower's timer.
                self.gcode.run_script("RESUME")
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: {_n} is reloaded but the resume "
                    f"failed ({e}); the print stays PAUSED. Press resume when "
                    f"you are ready.")
                return _done(self.afc.reactor.NEVER)
            self.logger.info(
                f"AFC bambu {self.name}: {_n} is reloaded and the print has "
                f"been RESUMED automatically "
                f"({self._auto_resume_count} of {limit} this print). Disable "
                f"with auto_error_recovery_limit: 0.")
            return _done(self.afc.reactor.NEVER)

        try:
            self.afc.reactor.register_callback(
                _run, self.afc.reactor.monotonic() + 1.0)
        except Exception:
            self._auto_recover_armed = False
            self._in_auto_recover = False

    #: Seconds a UID may stay unresolved before warning that the unit is not
    #: on the bus. A healthy chain resolves in well under a second.
    CHAIN_RESOLVE_WARN_S = 30.0

    def _check_chain_resolve(self, eventtime: float) -> None:
        """
        Warn once if this unit's UID never resolves to a chain index.

        Registrations are held at every boot until the chain map says which
        index this UID holds, so they do not land on whichever unit sits at
        the config default. Still waiting after CHAIN_RESOLVE_WARN_S means the
        unit is not answering the bus and its registrations (HT flag, MC
        address, self-centre, capacity enable) have not been sent.

        :param eventtime: Reactor event time
        """
        if not self._announce_deferred or self._id_resolved:
            return
        t0 = getattr(self, "_announce_defer_t0", 0.0)
        if not t0 or eventtime - t0 < self.CHAIN_RESOLVE_WARN_S:
            return
        if getattr(self, "_announce_defer_warned", False):
            return
        self._announce_defer_warned = True
        self.logger.warning(
            f"AFC bambu {self.name}: UID {self.unit_uid} still has no chain "
            f"index after {self.CHAIN_RESOLVE_WARN_S:.0f}s -- this unit is not "
            f"answering the bus, so its registrations have not been sent. "
            f"Check it is powered and chained; AFC_BAMBU_UIDS shows the map.")

    # A hold clears the way it is raised: the unit leaving its fault, or the
    # human resuming. Nothing releases it by watching the buffer -- during a
    # toolchange the quick pull and the cut's own retract lift the buffer while
    # the nozzle is still being cut, so a buffer-triggered reset feed would run
    # into it.

    def _fault_hold_active(self) -> bool:
        """
        Whether the stall hold is still suppressing the follower auto-arm.

        Releases on resume, i.e. the operator saying the jam is cleared. The
        pause is not instant (AFC_error queues it), so during a print the hold
        only releases after a paused state has been observed; otherwise the
        next tick would see "not paused" and re-arm into the jam. Outside a
        print it releases immediately.

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
            # No pause seen yet. During a print, keep waiting for it; outside a
            # print there will never be one, so release rather than latching
            # the follower off indefinitely.
            try:
                printing = bool(self.afc.function.in_print())
            except Exception:
                printing = True     # unknown: keep the safer, held behaviour
            if printing:
                return True
        was_printing = self._follow_fault_saw_pause
        self._follow_fault_hold = False
        # Re-arm auto recovery (one attempt per fault, not per print), but not
        # from inside the recovery's own attempt (see _in_auto_recover).
        if not getattr(self, "_in_auto_recover", False):
            self._auto_recover_armed = False
        self._follow_fault_saw_pause = False
        self.logger.info(
            f"AFC bambu {self.name}: "
            + ("print resumed, re-arming the follower."
               if was_printing else
               "no print to resume, releasing the follower hold."))
        return False

    def _check_ams_fault(self, lane: Any) -> None:
        """
        Raise an AFC error when the AMS reports it stalled.

        The unit reports stalls itself ("feed finish -1, stall", "switch_feed
        rocker stall", "pull err, bdc stall") and knows things the host cannot
        see (which motor, which tray, rocker state), so its report is used
        rather than inferring a stall from buffer position.

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
        # Scoped to this unit: boxed units share 0x0700, and every claimed
        # unit on a chain runs this check, so a bridge-wide read would report
        # a chain-mate's stall against this unit's lane.
        seq, text, amps = getf(unit=getattr(self, "ams_index", None))
        if seq == getattr(self, "_fault_seen", 0):
            return
        self._fault_seen = seq
        if not text:
            return
        low = text.lower()
        # "stall exit" is the scan path finishing its pull-in, not a failure.
        if "stall exit" in low:
            return
        # A stall "during calib" is the AMS's capacity calibration pulling the
        # spool to a hard stop to measure its radius ("check stall during
        # calib" / "Calibration rst"), a normal step, not a jam.
        if "calib" in low:
            return
        current = f", motor {amps:.2f}A" if amps else ""
        # The unit's own error code, when it has stated one ("err_code: 0 -> 23"
        # on HT, "err_code:0x00->0x80" on AMS 2); it changes when the jam is
        # actually cleared.
        code = None
        try:
            getec = getattr(self._bridge, "last_err_code", None)
            if callable(getec):
                code, _ = getec()
        except Exception:
            code = None
        err = f", err_code {code}" if code else ""
        msg = (f"AFC bambu {self.name}: AMS reported a stall on {lane.name}"
               f"{current}{err} -- the spool is likely tangled or the path "
               f"jammed. Clear the snag, then resume.\n"
               f"AMS said: {_fault_reason(text)}")
        # The resume is not blocked: the unit accepts a clear immediately but
        # cannot leave its error state until the filament moves, so resuming
        # into an uncleared jam simply stalls and fires this again.
        try:
            if self.afc.function.in_print():
                msg += "\nOnce cleared, click resume to continue printing"
        except Exception:
            pass
        self._raise_ams_fault(lane, msg)

    # There is no buffer-starvation detector. Inferring a stall from the
    # buffer bottoming out fires on healthy prints, and the buffer is shared
    # between units, so a unit could act on pressure that is not its own.
    # byte[19] == 0x07 reports the stalled state directly on all three unit
    # types, including the one that reports faults as state rather than words
    # ("state:6" / "en:0,mode:7").

    def _run_stall_checks(self, lane: Any) -> None:
        """
        Run both stall detectors against the tool-loaded lane, raising at most
        once per stall.

        The unit state byte (_check_unit_stalled) goes first: it is the only
        signal an AMS 1 gives. The narration check (_check_ams_fault) covers
        the stall text the AMS 2 and HT print. While a raised stall holds the
        follower, neither raises: the narration sequence is marked seen and a
        state still at 0x07 is latched, so the same stall is not raised again
        once the print resumes. A load reports its own stalls, so nothing runs
        while one is in progress.

        :param lane: The lane threaded to the toolhead from this unit
        """
        if getattr(self, "_load_in_progress", False):
            return
        if getattr(self, "_follow_fault_hold", False):
            try:
                seq, _t, _a = self._bridge.last_fault(
                    unit=getattr(self, "ams_index", None))
                self._fault_seen = seq
                if (self._unit_state(self._bridge.latest_status())
                        == self.AMS_STATE_STALLED):
                    self._stalled_seen = True
            except Exception:
                pass
            return
        if self._check_unit_stalled(lane):
            return
        self._check_ams_fault(lane)

    def _detector_tick(self, eventtime: float) -> float:
        """
        Periodic link-loss and stall check for a unit claimed live from the
        pool. Runs the same detectors as _follow_tick against this unit's
        tool-loaded lane, without engaging or re-arming the follower. Idle
        while the unit is released (no bridge).

        :param eventtime: Reactor event time
        :return float: next fire time
        """
        if self._bridge is None or self.pool:
            return eventtime + self.follow_poll_interval
        try:
            self._check_link_loss()
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: link-loss check raised {e!r}; "
                f"detector tick continuing.")
        # Called every tick: it is also what releases the hold on resume.
        try:
            self._fault_hold_active()
        except Exception:
            pass
        try:
            watch = self._tool_loaded_lane()
            if watch is not None and self._bridge is not None:
                self._run_stall_checks(watch)
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: fault check raised {e!r}; "
                f"detector tick continuing.")
        return eventtime + self.follow_poll_interval

    def _follow_tick(self, eventtime: float) -> float:
        """
        Periodic follower tick. Engages the follower on the lane loaded from
        this unit (or stands it down when none is), re-asserts mode:4 when the
        AMS has dropped to idle and the extruder has recently demanded
        filament, and runs the chain-resolve, link-loss and AMS-fault checks.
        Reschedules itself.

        :param eventtime: Reactor event time
        :return float: next fire time
        """
        if self._bridge is None:
            return eventtime + self.follow_poll_interval
        # Drying does not idle this tick: a load starting during a dry has
        # nothing tool_loaded yet and needs the follower, and the firmware does
        # not hold the follower off while drying (see DRY_ACTIVE in bambubus.c).
        lane = self._following_lane
        # Contained like every detector here: this is a reactor timer, and an
        # escaped exception takes all of Klipper down with it.
        try:
            chk = getattr(self, "_check_chain_resolve", None)
            if callable(chk):
                chk(eventtime)
            # No buffer-based auto-reset of the fault hold (see the note above
            # _fault_hold_active).
        except Exception as e:
            log = getattr(self, "logger", None)
            if log:
                log.warning(
                    f"AFC bambu {self.name}: fault auto-reset raised {e!r}; "
                    f"follower tick continuing.")
        # Bridge link lost mid-print: when frames stop, the extruder keeps
        # pulling against an AMS nobody is driving and the filament grinds
        # long before the transport reconnects. Contained (reactor timer).
        try:
            self._check_link_loss()
        except Exception as e:
            log = getattr(self, "logger", None)
            if log:
                log.warning(
                    f"AFC bambu {self.name}: link-loss check raised {e!r}; "
                    f"follower tick continuing.")
        # Evaluated every tick and never short-circuited into the test below:
        # this call is also what releases the hold once the print resumes.
        fault_hold = self._fault_hold_active()
        # Auto-arm: keep the follower engaged whenever a lane on this unit is
        # threaded to the toolhead. Deliberately independent of the AMS buffer
        # readback (often a stuck default) and of per-lane extruder wiring
        # (extruder_obj can be None), either of which could leave the follower
        # engaged but never re-arming. Held off after a stall: re-arming into
        # a jam grinds the filament.
        if (self.follow_when_loaded
                and not fault_hold
                and not getattr(self, '_follow_manual_off', False)
                and not getattr(self, '_unload_in_progress', False)):
            loaded = self._tool_loaded_lane()
            if loaded is not None and str(getattr(loaded, "status", "")) \
                    == AFCLaneState.ERROR:
                # A lane in ERROR is left to the operator: a re-armed assist
                # would push half-retracted filament forward again. None here
                # takes the stand-down branch below.
                loaded = None
            if loaded is not None:
                if lane is not loaded:
                    # Never engaged, or the loaded lane changed -> (re)engage mode:4.
                    self._engage_follower(loaded)   # sets _following_lane
                    lane = self._following_lane
                else:
                    # Already following: re-assert mode:4 if the AMS dropped to state 0,
                    # rate-limited (this tick runs every ~100ms) and only when the extruder
                    # asked for filament inside follow_rearm_window. State 0 at an idle
                    # unit means centred, not dropped, so state alone never settles.
                    try:
                        e_now = self.afc.toolhead.get_position()[3]
                        if self._follow_last_e is None:
                            self._follow_last_e = e_now
                        elif e_now - self._follow_last_e >= self.follow_min_extrude:
                            self._follow_last_demand = eventtime
                            self._follow_last_e = e_now
                        elif e_now < self._follow_last_e:
                            self._follow_last_e = e_now     # retract/reset
                    except Exception:
                        # No toolhead to read: treat as demand rather than
                        # never re-arming.
                        self._follow_last_demand = eventtime
                    st = self._bridge.latest_status()
                    fstate = st.get("fstate") if st is not None else None
                    wants = (eventtime - self._follow_last_demand
                             <= self.follow_rearm_window)
                    if (fstate == AMS_MODE_IDLE and wants
                            and eventtime - getattr(
                                self, "_follow_reassert_last", 0.0) >= 2.0):
                        self._follow_reassert_last = eventtime
                        self.set_feed_assist(loaded, True)
            elif lane is not None:
                # Nothing loaded from this unit anymore: stand the follower
                # down. Logged at most every 5s, since a
                # flickering _tool_loaded_lane() shows up here as an
                # engage/stand-down cycle.
                if (eventtime - getattr(self, "_follow_standdown_log", 0.0)
                        >= 5.0):
                    self._follow_standdown_log = eventtime
                    try:
                        cur_ext = self.afc.function.get_current_extruder()
                    except Exception:
                        cur_ext = "?"
                    self.logger.debug(
                        f"AFC bambu {self.name}: standing the follower down "
                        f"for {getattr(lane, 'name', '?')} -- no tool-loaded "
                        f"lane on this unit (afc.current="
                        f"{getattr(self.afc, 'current', None)}, active "
                        f"extruder={cur_ext}). If this repeats at a lane that "
                        f"IS loaded, the active-extruder gate is the flap.")
                self.set_feed_assist(lane, False)
                lane = self._following_lane        # now None
        # Fault detection follows the loaded lane, not the followed one:
        # AFC_BAMBU_FOLLOWER ENABLE=0 clears _following_lane, and running
        # without assist is when the buffer is most likely to starve.
        watch = lane
        if watch is None:
            try:
                watch = self._tool_loaded_lane()
            except Exception:
                watch = None
        if watch is not None and self._bridge is not None:
            # Contained: an exception escaping into reactor.run() shuts down
            # all of Klipper.
            try:
                self._run_stall_checks(watch)
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: fault check raised {e!r}; "
                    f"follower tick continuing.")
        if lane is not None and self._bridge is not None:
            # Telemetry: buffer position + follower state, for tuning (off by
            # default). Rate-limited and emitted only when buff/fstate/online
            # change.
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
            # No per-tick keep-alive is sent: the op-04 hold that keeps the
            # follower engaged is sustained by the assist arm (set_feed_assist /
            # s_follow_mask), which stays set until an explicit assist-off.
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

        Chain-wide, although it is invoked on one unit: the sweep deregisters
        every unit on the bus, so every unit's config (unit count, HT flags, MC
        addressing, model) must be sent again. After RELINK_SETTLE_S this
        replays every reconnect listener on the bridge, not just this unit's.

        :return bool: True if the command was issued
        """
        if self._bridge is None:
            return False
        self._bridge.send({"cmd": "relink"})
        try:
            reactor = self.afc.reactor
            reactor.register_callback(
                lambda et: self._bridge.replay_reconnect_listeners(),
                reactor.monotonic() + RELINK_SETTLE_S)
        except Exception:
            try:
                self._bridge.replay_reconnect_listeners()
            except Exception:
                pass
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
        self._bridge.send({"cmd": "rehome", "unit": int(self.ams_index)})
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

    # Mode 09->07 "STEP7:finish" handoff frames (CRC included). A load walks
    # motion modes 01->03->09->07; mode 07 is the AMS's load-complete signal.
    # The feed only streams mode 03, so without these the AMS never leaves the
    # feed stage and retract-retries forever. Sent when the toolhead sensor
    # triggers, they make the AMS commit the load and hold tension.
    _FINISH_FRAMES = (
        "3DC50CC803000900A502800C",   # mode 09 feeder->hub handoff
        "3DC50CC8030007000002514C",   # mode 07 gate
        "3DC50CC8030007007F023654",   # mode 07 finish (STEP7:finish)
    )

    def bridge_finish(self, lane: Any = None) -> bool:
        """
        Signal the AMS that the load is complete (mode-07 "STEP7:finish").

        Sends the _FINISH_FRAMES mode 09->07 handoff via the bridge 'raw'
        passthrough, telling the AMS the filament reached the extruder so it
        stops retract-and-retry and holds tension. Send after the toolhead
        sensor confirms filament arrival.

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
                   mmps: Optional[float] = None,
                   fault_mark: Optional[int] = None,
                   accept_switch_finish: bool = False,
                   accept_tray_release: Optional[int] = None) -> bool:
        """
        Wait for a bridge move to finish, preferring the AMS's own report.

        The AMS announces completion itself ("[AMS_SWITCH]feed finish...",
        "[AMS_PRELOAD]preload finish..."), so wait for that rather than for a
        computed duration: the unit moves at its own speed, not the commanded
        mm/s. The narration also says whether the move succeeded; a stall
        reports "finish -1".

        Falls back to the estimated duration as a timeout, so hardware that
        does not narrate still works.

        :param mm: Distance commanded in mm
        :param mmps: Commanded speed in mm/s
        :param fault_mark: A sequence from _ams_fault_seq taken before the
          move. When given, a fault raised past it ends the wait immediately
          instead of sitting out the deadline. Peeked, never consumed; the
          caller that acts on it reports it.
        :param accept_tray_release: The AMS slot this move is retracting. When
          given, the wait also ends when the unit reports that tray released
          (``tray_now`` -> 255), which is the AMS 2's unload completion. The
          slot must match: an operator preloading or inserting a spool
          produces the same edge on another tray. See
          AFC_BambuAMS_bridge._TRAY_NOW_RE.
        :return bool: True if the AMS reported a successful completion; False
          if it reported a stall or nothing arrived before the timeout
        """
        # The deadline uses DEADLINE_MMPS by default: it sizes a watchdog, and
        # the AMS moves at its own rate regardless of the commanded speed.
        # nm is resolved once with a default because the f-strings below are
        # built before _say can guard anything (tests use shims with no .name).
        nm = getattr(self, "name", "?")

        def _say(msg: str) -> None:
            """
            Debug-log a diagnostic line without ever raising from a missing logger.

            :param msg: line to log
            """
            try:
                self.logger.debug(msg)
            except Exception:
                pass

        speed = DEADLINE_MMPS if mmps is None else clamp_speed(mmps, MAX_MMPS)
        duration = (abs(mm) / speed) if speed > 0 else 0.0
        # Generous ceiling: the estimate is a lower bound on how long the AMS
        # may legitimately take, not an upper one.
        deadline_s = min(duration * 2.0 + 5.0, MOVE_DEADLINE_MAX_S)
        bridge = self._bridge
        start_seq = bridge.last_finish()[0] if bridge is not None else 0
        # The retract's own completion, sampled here (like start_seq) so only
        # a switch reported after this wait began can end it; a load's trailing
        # switch cannot leak into the next wait. See _STATE_SWITCH_DONE_RE in
        # the bridge.
        start_switch = 0
        if accept_switch_finish and bridge is not None:
            try:
                start_switch = bridge.last_switch_finish()[0]
            except AttributeError:
                accept_switch_finish = False    # older bridge; deadline stands
        # The tray-release edge, marked the same way: only a release reported
        # after this point can end this move. Together with the tray-index
        # check below, this keeps an operator handling spools from ending an
        # unload.
        start_tray = 0
        if accept_tray_release is not None and bridge is not None:
            try:
                start_tray = bridge.last_tray_release(
                    unit=getattr(self, "ams_index", None))[0]
            except (AttributeError, TypeError):
                accept_tray_release = None      # older bridge; deadline stands
        reactor = self.afc.reactor
        # What ends the wait: narration first (the only signal that reports
        # success as well as completion), then a fault. fstate is not used:
        # the firmware derives it from the narration drain and it is
        # bus-wide, not per unit. A still odometer is not used either: the
        # unit pauses mid-move during its own retry cycles.
        try:
            end = reactor.monotonic() + deadline_s
            while reactor.monotonic() < end:
                if bridge is not None:
                    seq, ok, _text = bridge.last_finish()
                    if seq != start_seq:
                        # Each exit logs which signal ended the wait; they
                        # fire under different conditions per model.
                        _say(
                            f"AFC bambu {nm}: move ended on NARRATION "
                            f"after {reactor.monotonic() - (end - deadline_s):.1f}s "
                            f"(ok={ok}): {str(_text)[:70]}")
                        return ok
                    # The unit has given up. A latched unit reports no
                    # completion, so this ends the wait instead of sitting out
                    # the deadline.
                    if (fault_mark is not None
                            and self._ams_fault_since(fault_mark,
                                                      consume=False)):
                        _say(
                            f"AFC bambu {nm}: move ended on a FAULT "
                            f"after {reactor.monotonic() - (end - deadline_s):.1f}s")
                        return False
                if accept_switch_finish and bridge is not None:
                    try:
                        sw_seq, sw_text = bridge.last_switch_finish()
                    except AttributeError:
                        sw_seq, sw_text = start_switch, ""
                    if sw_seq > start_switch:
                        _say(
                            f"AFC bambu {nm}: move ended on the STATE SWITCH "
                            f"after {reactor.monotonic() - (end - deadline_s):.1f}s "
                            f"-- {sw_text.strip()[:80]}")
                        return True
                if accept_tray_release is not None and bridge is not None:
                    try:
                        tr_seq, tr_from = bridge.last_tray_release(
                            unit=getattr(self, "ams_index", None))
                    except (AttributeError, TypeError):
                        tr_seq, tr_from = start_tray, None
                    # Both conditions: a release of another tray is the
                    # operator, not this reel, and is ignored.
                    if tr_seq > start_tray and tr_from == accept_tray_release:
                        _say(
                            f"AFC bambu {nm}: move ended on the TRAY RELEASE "
                            f"after {reactor.monotonic() - (end - deadline_s):.1f}s "
                            f"-- tray {tr_from} -> none")
                        return True
                reactor.pause(reactor.monotonic() + 0.1)
        except Exception:
            pass
        _say(
            f"AFC bambu {nm}: move ended on the DEADLINE "
            f"({deadline_s:.0f}s) -- no signal arrived")
        return False

    def _toolhead_sensor_triggered(self, cur_lane: Any) -> bool:
        """
        Whether the lane's toolhead pre-sensor (or buffer) reports filament.

        Reads the live switch state, not the cache. ``pin_tool_start`` has two
        consumers in AFC_extruder: a filament switch whose
        ``runout_helper.filament_present`` is the live state (what AFC's runout
        path reads), and a button callback maintaining ``tool_start_state``,
        which ``get_toolhead_pre_sensor_state()`` returns and which is only as
        current as the last callback.

        This matters for recovery: unit_load_lane's retry is gated on
        `if not loaded`, so a stale cache reading "filament" would make
        _feed_until_sensor return True and skip the retry.

        Falls back to the lane accessor, which is the right answer for a
        ``tool_start = buffer`` setup (no pin to read) and for anything not
        exposing the switch.

        :param cur_lane: The lane whose toolhead sensor to read
        :return bool: True when filament is detected at the toolhead
        """
        try:
            ext = getattr(cur_lane, "extruder_obj", None)
            sw = getattr(ext, "fila_tool_start", None)
            helper = getattr(sw, "runout_helper", None)
            if helper is not None:
                return bool(helper.filament_present)
        except Exception:
            pass
        try:
            return bool(cur_lane.get_toolhead_pre_sensor_state())
        except Exception:
            return False

    # The feed ends on a bare stop() when the sensor trips; the caller then
    # sends bridge_finish(). Sending the printer's op-03 motion byte here faults
    # the unit (TIMEOUT error 2/3), and byte[19] cannot gate the transitions
    # (see _follow_tick).

    def _ams_fault_seq(self) -> int:
        """
        This unit's current fault sequence, without consuming it.

        Take one before asking the unit to move; anything past it is a fault
        this move provoked rather than one left over from before.

        :return int: the current sequence, or 0 when unavailable
        """
        getf = getattr(self._bridge, "last_fault", None) if self._bridge else None
        if not callable(getf):
            return 0
        try:
            return int(getf(unit=getattr(self, "ams_index", None))[0] or 0)
        except Exception:
            return 0

    def _ams_fault_since(self, mark: int,
                         consume: bool = True) -> Optional[str]:
        """
        The AMS's own words for a real fault raised since ``mark``.

        The bridge bumps its fault sequence on any fault-shaped line, including
        a scan ending its pull-in ("bldc stall exit") and capacity calibration
        pulling the spool to a hard stop ("check stall during calib"); both are
        normal and are filtered out here (as in _check_ams_fault).

        Consuming marks the fault seen, which suppresses the follower tick's
        _check_ams_fault for that event: whoever consumes it owns reporting it.
        Pass ``consume=False`` to peek (e.g. to break a wait early) and leave
        the reporting to the caller that acts on it.

        :param mark: A sequence from _ams_fault_seq taken before the move
        :param consume: Whether to mark the fault seen (default True)
        :return str: the unit's own text, or None if nothing real is new
        """
        getf = getattr(self._bridge, "last_fault", None) if self._bridge else None
        if not callable(getf):
            return None
        try:
            seq, text, _amps = getf(unit=getattr(self, "ams_index", None))
        except Exception:
            return None
        if not seq or seq == mark or not text:
            return None
        low = text.lower()
        if "stall exit" in low or "calib" in low:
            return None
        if consume:
            self._fault_seen = seq
        return text.strip()

    def _ams_declared_fault(self) -> bool:
        """
        Whether the AMS has reported a new fault since the last check.

        Consumes the sequence, so one fault is reported once, but keeps the
        unit's words in _declared_fault_text so the load's final error can
        quote what the unit said.

        :return bool: True on a fault not yet seen
        """
        text = self._ams_fault_since(getattr(self, "_fault_seen", 0))
        if text is None:
            return False
        self._declared_fault_text = text
        return True

    def _feed_until_sensor(self, cur_lane: Any,
                           timeout: Optional[float] = None) -> bool:
        """
        Drive the AMS forward until the toolhead sensor triggers, then STOP it.

        The AMS feeds continuously (mode 03) once kicked; the commanded mm is
        advisory, so distance is bounded only by when it is stopped. The AMS
        has its own load routine that stall-retries (feed, stall at the
        extruder, retract, retry) several times before giving up, so this lets
        that run and catches the filament when it reaches the sensor. The
        sensor is polled every 50 ms, but the feed is re-kicked only every
        ``load_retry_interval`` so the AMS completes each of its own retry
        cycles instead of being reset mid-attempt.

        :param cur_lane: The lane being loaded
        :param timeout: Seconds to keep trying (default ``load_retry_timeout``)
        :return bool: True once the sensor triggers (AMS stopped), else False
        """
        if timeout is None:
            timeout = self.load_retry_timeout
        if self._toolhead_sensor_triggered(cur_lane):
            self.stop()
            return True
        attempt_t0 = self.afc.reactor.monotonic()
        deadline = attempt_t0 + timeout
        last_kick = -1.0
        kicks = 0
        latched_since = None                  # when ustate first read STALLED
        self._fault_said_this_load = False    # say it once per load, not per kick
        while self.afc.reactor.monotonic() < deadline:
            now = self.afc.reactor.monotonic()
            # A shut-down printer has no toolhead to feed and the sensor can no
            # longer change; the bridge is a separate MCU so nothing here fails
            # on its own. Exit as a timeout instead of riding it out.
            try:
                _down = self.printer.is_shutdown()
            except Exception:
                _down = False      # never let a state read stop a live load
            if _down:
                self.logger.warning(
                    f"AFC bambu {self.name}: {cur_lane.name} -- the printer "
                    f"has shut down, so this load stops feeding. Nothing can "
                    f"reach the toolhead until Klipper is restarted.")
                break
            # Same for a link that is down: every kick is dropped by send() and
            # no narration can end the wait. Only a link down past
            # LINK_DOWN_GRACE_S gives up; a brief reconnect does not.
            try:
                _link_down_t = (self._bridge.down_since()
                                if self._bridge is not None else None)
            except Exception:
                _link_down_t = None    # never let a state read stop a live load
            if _link_down_t is not None:
                _for = time.monotonic() - _link_down_t
                if _for > LINK_DOWN_GRACE_S:
                    self.logger.warning(
                        f"AFC bambu {self.name}: {cur_lane.name} -- the link "
                        f"to the bridge has been down {_for:.0f}s, so every "
                        f"command this load sends is being dropped. Stopping "
                        f"rather than feeding a bus we cannot reach. "
                        f"{_RETRY_HINT}.")
                    break
            if now - last_kick >= self.load_retry_interval:
                kicks += 1
                last_kick = now
                self.logger.debug(
                    f"AFC bambu {self.name}: feeding {cur_lane.name} to sensor "
                    f"(kick {kicks}); letting the AMS run its own retry")
                self.feed(cur_lane, self.load_retry_pulse)
            # A fault is not a reason to stop asking: the AMS recovers underneath
            # a continued request, and withdrawing it makes the fault terminal.
            # The exception is "state_switch finish, fail, retry:5": the unit's
            # own retry budget is spent (AMS 2 and HT only).
            try:
                # Scoped to this attempt, not the whole load: the caller's
                # re-home between attempts can unlatch a stuck unit, so a
                # give-up in an earlier attempt must not abort the next one.
                _gave_up = self._bridge is not None and self._bridge.gave_up_since(
                    attempt_t0, addr=getattr(self, "dry_dev_addr", None))
            except Exception:
                _gave_up = False       # never let a helper stop a live load
            if _gave_up:
                self.logger.warning(
                    f"AFC bambu {self.name}: {cur_lane.name} -- the AMS says "
                    f"it has STOPPED retrying, so this load stops asking. "
                    f"Clear the path at the unit. {_RETRY_HINT}.")
                break
            # The same give-up as a byte, for the AMS 1, which has no fault
            # words: ustate (op-04 byte[19]) latched at 7 never clears on its
            # own. A 7 must persist past STALL_LATCH_S; a unit still retrying
            # reads 2. Ending the attempt is not ending the load: the caller
            # re-homes and retries. A status read that raises must not end a
            # load.
            try:
                _ustate = self._unit_state(
                    self._bridge.latest_status() if self._bridge else None)
            except Exception:
                _ustate = None
            _hold = afcBambuAMS.STALL_LATCH_S
            if _ustate == afcBambuAMS.AMS_STATE_STALLED:
                if latched_since is None:
                    latched_since = now
                elif now - latched_since >= _hold:
                    self.logger.warning(
                        f"AFC bambu {self.name}: {cur_lane.name} -- the unit "
                        f"has reported STALLED for {_hold:.0f}s without "
                        f"clearing, so this attempt stops feeding and hands "
                        f"over to the recovery re-home.")
                    break
            else:
                latched_since = None          # working again; re-arm the hold
            if self._ams_declared_fault() and not getattr(
                    self, "_fault_said_this_load", False):
                self._fault_said_this_load = True
                # Info, not warning: the unit recovers itself and the load
                # carries on. The failure case has its own messages.
                self.logger.info(
                    f"AFC bambu {self.name}: {cur_lane.name} -- the AMS "
                    f"reported a fault during the load; still feeding, the "
                    f"unit runs its own recovery while the request continues")
            if self._toolhead_sensor_triggered(cur_lane):
                self.stop()          # halt instantly so the AMS can't retract it
                # Read the odometer now: it keeps climbing once the extruder
                # takes over (tool_stn plus the purge), so any later reading
                # would include what the toolhead consumed.
                self._load_odom_at_sensor = self._odom_now_mm()
                # A clean load is already narrated by AFC's load path; log at
                # info only when this loop had to re-kick the AMS.
                if kicks:
                    self.logger.info(
                        f"AFC bambu {self.name}: {cur_lane.name} reached the "
                        f"toolhead sensor after {kicks} feed kick(s)")
                else:
                    self.logger.debug(
                        f"AFC bambu {self.name}: sensor triggered for "
                        f"{cur_lane.name}, AMS stopped")
                return True
            # No fallback to the AMS's own arrival report: tool_start is always
            # a real pin or "buffer", and the unit reaching the end of its tube
            # differs from the filament reaching the toolhead when the path binds.
            try:
                self.afc.reactor.pause(now + 0.05)
            except Exception:
                break
        self.stop()
        return False

    def _log_delta(self, msg: str, debug: bool = True) -> None:
        """
        Call AFC's delta-time logger defensively.

        The Bambu load takes AFC's unit_load_lane branch, which skips every
        log_with_time call in AFC.load_sequence, so these markers record the
        load's stages (as AFC_ACE does).

        Defensive because upstream's log_with_time raises (datetime - None) when
        it runs before set_start_time(), which happens for a load reached
        outside a normal TOOL_LOAD (error recovery, a bare AFC_BAMBU command).
        Starts the clock first if needed, and never lets a timing log break a
        load.

        :param msg: the stage name to record
        :param debug: log at debug level
        """
        dt = getattr(self.afc, "afcDeltaTime", None)
        if dt is None:
            return
        try:
            if getattr(dt, "start_time", None) is None:
                dt.set_start_time()
            dt.log_with_time(msg, debug=debug)
        except Exception:
            pass

    def _settle_seqs(self) -> Optional[tuple]:
        """
        The three completion counters that mean "the unit has stopped moving".

        Read as one tuple so a caller can baseline them and watch for any to
        advance; which one advances differs by model (see
        _wait_arrival_settle).

        :return tuple: (finish, switch, assist) sequences, or None with no bridge
        """
        b = getattr(self, "_bridge", None)
        if b is None:
            return None
        try:
            return (b.last_finish()[0], b.last_switch_finish()[0],
                    b.last_assist_done())
        except Exception:
            return None

    def _wait_arrival_settle(self) -> float:
        """
        Wait for the unit to report it has stopped, capped at the configured gap.

        Ends as soon as any of the _settle_seqs counters advances, typically
        well under a second after the sensor trip.

        The models report it differently, so three counters are watched. After
        stop() at the sensor, the AMS 2 still emits its own "feed finish"
        (_finish_seq), while the HT reports the assist ending (_assist_seq) and
        its state switch (_switch_seq) instead.

        The AMS 1 narrates in the [AMS_DEV] dialect and says none of those; its
        only completion is "STEP:odom reset tray N", which _ODOM_RESET_RE turns
        into a _finish_seq bump. Whether that ends the wait early on an AMS 1
        is unverified; expect it to fall back to the cap.

        A unit that says nothing waits arrival_assist_delay_s and proceeds.
        arrival_assist_delay_s = 0 skips the wait entirely.

        :return float: seconds actually waited
        """
        cap = self.arrival_assist_delay_s
        if cap <= 0:
            return 0.0
        reactor = self.afc.reactor
        try:
            t0 = reactor.monotonic()
        except Exception:
            return 0.0
        base = self._settle_seqs()
        deadline = t0 + cap
        while True:
            try:
                now = reactor.monotonic()
            except Exception:
                return 0.0
            if now >= deadline:
                return now - t0
            # No baseline means no bridge to ask: wait out the cap.
            if base is not None:
                cur = self._settle_seqs()
                if cur is not None and cur != base:
                    return now - t0
            try:
                reactor.pause(min(now + 0.05, deadline))
            except Exception:
                return now - t0

    def _advance_into_extruder(self, cur_lane: Any, cur_extruder: Any) -> None:
        """
        Hand the filament to the extruder without fighting the AMS for it.

        Bite, wait for the unit to settle, then advance:

            bite (tool_bite_mm)          gears grip, nothing else moving
            settle (<= arrival_assist_delay_s)  the unit says it has stopped
            assist                       hold mode:4 via the AP2 sync
            advance (tool_stn - bite)    the rest, into a settled path

        No select is sent at the arrival: the AMS reaches mode:4 as the
        natural end of its own feed ("feed finish 0, mode:4") once driving
        stops, matching a stock printer. The gap before the hold ends when the
        unit reports it has stopped (see _wait_arrival_settle).

        tool_bite_mm = 0 advances in one go.

        :param cur_lane: the lane whose filament just reached the sensor
        :param cur_extruder: the extruder that will pull it in
        """
        afc = self.afc
        tool_stn = getattr(cur_extruder, "tool_stn", 0) or 0
        speed = getattr(cur_extruder, "tool_load_speed", 0) or 0
        cur_lane.activate_toolhead_extruder()
        bite = min(self.tool_bite_mm, tool_stn) if tool_stn > 0 else 0.0
        if bite > 0:
            afc.move_e_pos(bite, speed, "tool bite")
            self._log_delta("Bambu: tool bite")
        waited = self._wait_arrival_settle()
        self._log_delta(f"Bambu: unit settled ({waited:.2f}s of "
                        f"{self.arrival_assist_delay_s:.2f}s)")
        self.set_feed_assist(cur_lane, True)  # hold mode:4 via AP2 sync
        if tool_stn > bite:
            afc.move_e_pos(tool_stn - bite, speed, "tool stn")
            self._log_delta(f"Bambu: tool_stn queued "
                            f"({tool_stn - bite:.0f}mm @ {speed:.0f}mm/s)")

    def unit_load_lane(self, cur_lane: Any, cur_extruder: Any = None) -> bool:
        """
        Load a lane to the toolhead, with the follower tick held off throughout.

        cur_lane.status only becomes TOOL_LOADED at the end of the load, so
        during the arrival _tool_loaded_lane() answers None; without the guard
        the follower tick would drop the assist the load path just armed and
        re-arm it when the status lands.

        try/finally because the load has several returns and can raise; a
        guard left set would silence the follower for the rest of the session.

        :param cur_lane: Lane to load
        :param cur_extruder: Extruder the lane loads into (defaults to the lane's)
        :return bool: True on a verified load, False on failure
        """
        self._load_in_progress = True
        try:
            return self._unit_load_lane(cur_lane, cur_extruder)
        finally:
            self._load_in_progress = False

    def _unit_load_lane(self, cur_lane: Any, cur_extruder: Any = None) -> bool:
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
        # AFC_BAMBU_FOLLOWER ENABLE=0 cannot leave the next print without assist.
        self._follow_manual_off = False
        self._follow_fault_hold = False
        # Per-load state: a previous failure's words must not describe this one.
        self._declared_fault_text = None
        # Same for the odometer range, which locates a failure upstream or
        # downstream of the AMS.
        self._load_odom_lo = None
        self._load_odom_hi = None
        # Re-arm auto recovery (one attempt per fault), but not from inside the
        # recovery's own attempt: that attempt runs CHANGE_TOOL, which lands
        # here, and clearing the guard would let it retrigger itself.
        if not getattr(self, "_in_auto_recover", False):
            self._auto_recover_armed = False
        self._follow_fault_saw_pause = False
        if cur_extruder is None:
            cur_extruder = getattr(cur_lane, "extruder_obj", None)
        if self._bridge is None:
            self.logger.warning(
                f"AFC bambu {self.name}: bridge not connected, cannot load "
                f"{cur_lane.name}")
            return False
        # Loading while drying is allowed on all unit types (AMS HT firmware
        # 05.00.22.19 and later feeds while the chamber heats). While a dry
        # cycle runs, a stalled load retries without the 0F/0E re-home; see
        # the recovery loop below.

        # Claim narration for this unit while it loads: tube_len is narrated
        # by whichever unit is feeding, and the device address cannot tell two
        # boxed units apart.
        try:
            self._bridge.set_active_unit(self.ams_index)
        except Exception:
            pass
        ok, _slot = self.select_lane(cur_lane)
        if not ok:
            self.logger.warning(
                f"AFC bambu {self.name}: lane {cur_lane.name} is not mapped to "
                f"an AMS slot")
            return False

        # Take the unit's own path measurement if it has one, and save it.
        # Before one exists, DEFAULT_BOWDEN_MM is used; it is long enough for
        # that first load to finish so the AMS can measure.
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

        # Kick the AMS's continuous feed (mode 03 forward) and tight-poll the
        # toolhead sensor, stopping the instant filament arrives. The deadline
        # uses DEADLINE_MMPS (the AMS's actual ~136 mm/s) and covers the whole
        # feed/stall/retry cycle, so it must not borrow MOVE_DEADLINE_MAX_S.
        speed = DEADLINE_MMPS
        bulk_time = (feed_dist / speed) if speed > 0 else 0.0
        timeout = min(bulk_time + self.load_retry_timeout, LOAD_SENSOR_MAX_S)
        # Where the odometer stands before any filament moves. The distance to
        # the toolhead sensor is the delta from here, not the raw reading --
        # see _measure_path_from_odom.
        self._load_odom_start = self._odom_now_mm()
        self._load_odom_at_sensor = None
        # Freshness mark for the spool measurement this load may produce, so a
        # reading left in the bridge from an earlier load is not adopted as
        # this one's.
        self._load_t0 = afc.reactor.monotonic()
        # Stage the bay before feeding (non-HT units, not while drying).
        #
        # The AMS stages a bay itself on the insert edge, leaving the filament
        # near the hub at `sw_sta 1`; from there a load engages it to
        # `sw_sta 3` and feeds. That autonomous preload does not run while the
        # unit is busy (holding the follower sets preload_disable, and so does
        # a running dryer), so a bay that has dropped back (e.g. after a failed
        # load) stays dropped. bb_feed sends one mode-09 select and then
        # streams mode 03, the hub stage, which cannot move filament the
        # feeder has not brought up. bb_prime supplies the feeder stage: ~2 s
        # of streamed mode-09 with the RFID reader dormant. A bay already
        # staged stays where it is.
        #
        # Skipped while drying: bb_prime streams op-03 mode 0x09 ref 0x7F,
        # which the unit narrates as `en:1,mode:4,idx:0,ref:127` (narrated
        # mode = (raw-1)/2, so 09 -> 4, the follower arm). That tells the unit
        # to hold a tray it has not selected, just before the select, and a
        # stock printer sends no op-03 mode 09 between idle and the feed when
        # loading during a dry (01/FF then 03/tray). Skipped on the HT
        # entirely. The insert edge (_maybe_insert_pullin) still primes.
        _dry_now = False
        try:
            _dry_now = self._heating_now()
        except Exception:
            _dry_now = False           # never let a state read stop a load
        if _dry_now:
            self.logger.debug(
                f"AFC bambu {self.name}: skipping the pre-load prime -- the "
                f"chamber is drying and mode 09/7F arms the follower before "
                f"the select, which the printer never does")
        if not self._is_ht() and not _dry_now:
            try:
                slot = self._slot_of(cur_lane)
                if slot is not None:
                    self._bridge.send({"cmd": "prime",
                                       "unit": self.ams_index, "slot": slot})
            except Exception:
                # Never fail the load over staging, but log it so a failed
                # prime is distinguishable from one that was skipped.
                self.logger.debug(
                    f"AFC bambu {self.name}: pre-load prime failed",
                    traceback=traceback.format_exc())
        self.feed(cur_lane, feed_dist)
        loaded = self._feed_until_sensor(cur_lane, timeout)
        # Printer "Retry": the unit stalled after its own retries, so re-home
        # it (mode 0F/0E, what the printer's Retry sends) and feed again. Only
        # one retry at a unit that has declared a fault; a unit that ignores
        # the first will ignore the second.
        attempts = self.load_recover_attempts
        if self._declared_fault_text and attempts > 1:
            attempts = 1
            self.logger.info(
                f"AFC bambu {self.name}: {cur_lane.name} -- the AMS declared a "
                f"fault, so this gets ONE re-home retry rather than "
                f"{self.load_recover_attempts}; a unit that ignores the first "
                f"will ignore the second.")
        # A unit that is not listening gets no retry: a re-home unsticks a
        # latch, not a unit that dropped its tray selection ("odom tray_id
        # error", dropped writes). Checked before every round, scoped to
        # _load_t0, because the refusal arrives during the first re-home.
        def _not_listening() -> str:
            """
            Describe why the unit is not answering, for the error text.

            :return str: the description
            """
            try:
                _addr = getattr(self, "dry_dev_addr", None)
                _br = self._bridge
                if _br is None:
                    return ""
                # A down link is itself the answer: the give-up test below
                # relies on what the unit said, which is nothing on a dead
                # link, so the recovery rounds would run in full against a bus
                # no command can reach.
                if not _br.is_connected():
                    return "the link to it is down"
                if not _br.gave_up_since(self._load_t0, addr=_addr):
                    return ""
                _no_tray = _br.no_tray_since(self._load_t0, addr=_addr)
                _dropped = _br.writes_dropped_since(self._load_t0)
            except Exception:
                return ""                        # never block a live load
            return " and ".join(
                w for w, on in (
                    ("it is rejecting moves with no tray selected", _no_tray),
                    ("the bridge is dropping commands to it", _dropped))
                if on)
        recover = 0
        while not loaded and recover < attempts:
            why = _not_listening()
            if why:
                self.logger.info(
                    f"AFC bambu {self.name}: {cur_lane.name} -- the AMS gave "
                    f"up AND {why}, so it is not listening; failing now "
                    f"instead of re-homing into a unit that cannot answer. "
                    f"{_RETRY_HINT}.")
                break
            recover += 1
            # A re-home unstages the bay, and a drying unit cannot restage it.
            # rehome() is mode 0F/0E (park and clear): a parked unit has no
            # selected tray and pulls the filament back off the bay switch.
            # Normally the unit then runs its own [AMS_PRELOAD] to re-seat it,
            # but while the chamber is drying that preload is held until the
            # heater stops (AMS_DRY_STATE_UNLOCK), so tray_exit (the bitmask of
            # bays holding filament) clears and every later retry feeds a bay
            # the unit believes is empty. So while a cycle is running, retry
            # the feed without the park.
            _hot = False
            try:
                _hot = self._heating_now()
            except Exception:
                _hot = False               # never let a state read stop a load
            _how = ("retrying without the re-home -- the chamber is drying "
                    "and a park would unstage the bay" if _hot
                    else "re-homing AMS and retrying")
            self.logger.info(
                f"AFC bambu {self.name}: load of {cur_lane.name} stalled; "
                f"{_how} (recover {recover}/{attempts})")
            self.stop()                          # halt before the reset motion
            if not _hot:
                self.rehome()                    # ~3s mode-0F/0E re-home reset
            # Re-baseline after the re-home: mode 0F/0E is a reset and the
            # odometer moves with it, so a delta measured from before it would
            # be the re-home's motion plus the load's, not the path.
            self._load_odom_start = self._odom_now_mm()
            self.feed(cur_lane, feed_dist)       # re-attempt the load
            loaded = self._feed_until_sensor(cur_lane, timeout)
        if not loaded:
            # The AMS retries loads on its own, so by default the filament is
            # not reeled back on a miss (that would fight the AMS's attempts).
            # It is left staged; unwind only when reel_back_on_load_fail is set.
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
                tail = ("left it staged -- the AMS keeps retrying; clear "
                        "the path manually if it can't finish")
            # A load that ended because the AMS gave up is a different failure
            # from one that ran out of time, so quote the unit when it spoke.
            if self._declared_fault_text:
                cause = (f"The AMS declared a fault during the load and stopped "
                         f"trying; {tail}.\nAMS said: "
                         f"{_fault_reason(self._declared_fault_text)}\n"
                         f"Clear the path at the unit. {_RETRY_HINT}.")
            else:
                cause = (f"Filament did not reach the toolhead sensor for "
                         f"{cur_lane.name} within {timeout:.0f}s; {tail}.\n"
                         f"Check the path from the bay to the toolhead, and "
                         f"that the toolhead sensor is reading. The AMS meters "
                         f"this feed itself, so the distance is not a setting "
                         f"to tune.\n{_RETRY_HINT}.")
            # Where, from the unit's own odometer: how far the filament went
            # separates a blockage upstream of the AMS from one downstream.
            # A disconnected tube is not detected specifically (the units do
            # not reliably narrate tube_len); the raw span is quoted so an
            # operator can compare it with the tube length.
            try:
                where = self._jam_location(self._load_odom_span_mm())
            except Exception:
                where = ""
            if where:
                cause = f"{cause}\n{where}"
            # A load that gave up owes a reload on RESUME. The mid-print stall
            # detector never runs for it, so mark the reload here along with
            # _fault_lane, which _resume_reload_target reads for the lane.
            if afc.function.in_print():
                self._fault_lane = cur_lane
                self._resume_needs_reload = True
            afc.error.handle_lane_failure(
                cur_lane, cause, pause=afc.function.in_print())
            return False

        # The bowden feed is the biggest and most variable part of a Bambu
        # load (it includes any AMS retries), so it gets its own marker.
        self._log_delta("Bambu: fed to toolhead sensor")

        # Sensor hit and the AMS is already stopped (inside _feed_until_sensor).
        # Tell it the load is complete (mode-07 "STEP7:finish") so it commits.
        self.bridge_finish(cur_lane)

        # Advance the last stretch to the nozzle and engage the AMS's
        # self-centering follower (mode:4), left running for the print; it is
        # cleared on unload / stop(). No re-select at the arrival: a mode-09
        # select starts a switch cycle that pulls the tray back, which jams
        # with the filament pinned in the toolhead. A stock printer's arrival
        # is 09/A5 only.
        try:
            self._advance_into_extruder(cur_lane, cur_extruder)
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: tool_stn advance failed for "
                f"{cur_lane.name}: {e}")
            self.stop()

        # The sensor has the last word: the filament can fall back after the
        # trigger (e.g. a dead link dropping the assist during the advance),
        # so re-read the sensor after the advance.
        if not self._toolhead_sensor_triggered(cur_lane):
            dropped = ""
            try:
                if self._bridge is not None and not self._bridge.is_connected():
                    dropped = (" The link to the bridge is down, so the assist "
                               "that holds the filament through the advance was "
                               "dropped -- restore it before retrying.")
            except Exception:
                pass
            cause = (f"{cur_lane.name} reached the toolhead sensor and then "
                     f"LOST it during the {getattr(cur_extruder, 'tool_stn', 0)}"
                     f"mm advance, so the filament is not at the toolhead. It "
                     f"was not loaded, whatever the feed reported.{dropped}\n"
                     f"Check the path at the extruder and that the AMS still "
                     f"has the filament. {_RETRY_HINT}.")
            self.stop()
            if afc.function.in_print():
                self._fault_lane = cur_lane
                self._resume_needs_reload = True
            afc.error.handle_lane_failure(
                cur_lane, cause, pause=afc.function.in_print())
            return False
        cur_lane.loaded_to_hub = True
        cur_lane.status = AFCLaneState.TOOL_LOADED
        # Acknowledge anything the AMS complained about during this load, so
        # the follower's first tick does not report its own retries as a
        # fault. Reaching the toolhead sensor settles those stalls.
        self._ack_faults()
        # Adopt the path measurement again now the load has finished: the unit
        # reports it at the end of a load, so the call at the top only sees the
        # previous one. Latched once per session, so calling twice is cheap.
        # Odometer first (typed, all units), tube_len narration as fallback.
        _mm, _src = self._path_measurement()
        self._adopt_measured_path(_mm, _src)
        afc.save_vars()
        return True

    def _published_slots(self) -> list:
        """
        The slots this unit reports, trimmed to the bays it actually has.

        An HT has one bay. The internal arrays are SLOTS_PER_UNIT (4) wide on
        every unit type, because the bridge indexes them by slot number and a
        short array would fault on a stray frame naming slot 3, so the trim
        happens on output rather than in storage.

        :return list: the slot records for bays this unit has
        """
        # `scanning` is true from arm until the window closes, so a consumer
        # can tell "not yet" from "no tag" while a read-plus-measure runs.
        out = []
        for i, info in enumerate(self._slots[:self.unit_slots]):
            rec = dict(info) if isinstance(info, dict) else info
            try:
                if isinstance(rec, dict):
                    rec["scanning"] = bool(self._scan_in_flight(i))
            except Exception:
                pass          # a status field must never break the status frame
            out.append(rec)
        return out

    def _uid_claimed_elsewhere(self, uid: Optional[str]) -> bool:
        """
        Whether another Bambu unit reports a present bay with this same UID.

        A tag is one physical chip in one bay, so two units cannot both be
        reading it. When they appear to, the bridge has copied one unit's
        records onto the other's (a cross-unit leak), and nothing derived from
        that record may reach a lane.

        :param uid: The bay record's tag UID (may be None/"")
        :return bool: True when some other unit claims the same UID
        """
        uid = (uid or "").strip().lower()
        if not uid:
            return False
        try:
            units = (getattr(self.afc, "units", None) or {}).values()
        except Exception:
            return False
        for unit in units:
            if unit is self or not isinstance(unit, afcBambuAMS):
                continue
            for other in (getattr(unit, "_slots", None) or []):
                if not other or not other.get("present"):
                    continue
                if (other.get("rfid_uid") or "").strip().lower() == uid:
                    return True
        return False

    def _unbind_spool(self, lane: Any, reason: str = "the bay is empty") -> None:
        """
        Drop a lane's Spoolman link.

        Used when the physical spool leaves the bay: a binding to an empty bay
        is stale, and Spoolman-linked lanes are treated as authoritative
        elsewhere, so a stale one blocks the next real tag from applying.

        Also used when a scan finishes without reading a tag, where the binding
        is the previous spool's claim on a bay that now holds something else.
        ``reason`` says which, so the log does not claim an occupied bay is
        empty.

        :param lane: The lane to unbind
        :param reason: why the link is being dropped, for the log
        """
        try:
            if getattr(lane, "spool_id", None) in (None, "", 0):
                return
            self.logger.debug(
                f"AFC bambu {self.name}: unbinding {lane.name} from spool "
                f"{lane.spool_id} -- {reason}")
            lane.spool_id = ''
            restore = getattr(self, "_restore_config_tare", None)
            if restore is not None:
                restore(lane)
        except Exception as e:
            self.logger.debug(
                f"AFC bambu {self.name}: could not unbind {lane}: {e}")

    def _restore_config_tare(self, lane: Any) -> None:
        """
        Give an unbound lane its configured empty-spool weight back.

        The tare is the spool's, not the lane's: binding to Spoolman overwrites
        ``empty_spool_weight`` from the spool's ``spool_weight`` field
        (AFC_spool), and the value is persisted, so dropping the link must put
        it back or the lane keeps a departed spool's tare across restarts.

        The configured value comes from AFC_lane's ``_config``. A lane object
        without one (the duck-typed stand-ins in the tests) is left as it was.

        :param lane: the lane whose Spoolman link has just been dropped
        """
        try:
            cfg = getattr(lane, "_config", None)
            if cfg is None:
                return
            lane.empty_spool_weight = cfg.getfloat(
                "empty_spool_weight", 190, minval=1)
        except Exception:
            pass
    # -- Spoolman shims --
    #
    # The Spoolman layer lives in AFC_BambuAMS_rfid and is enabled by an
    # [AFC_BambuAMS_rfid] section. These forward to it and no-op without it,
    # so callers here need not know whether Spoolman is configured.

    #: The Spoolman delegate, or None when no [AFC_BambuAMS_rfid] section is
    #: configured. Class attributes, not set in __init__: the lookup is lazy
    #: (the section can be parsed after this unit's own), and an object that
    #: skipped __init__ -- a subclass, or anything built with __new__ -- must
    #: still read as "no Spoolman" rather than raising AttributeError.
    _spool_obj: Any = None
    _spool_looked_up: bool = False

    @property
    def _spool(self) -> Any:
        """The Spoolman delegate for this unit, or None when not enabled.

        Looked up on first use, not in __init__: the [AFC_BambuAMS_rfid]
        section can be parsed after this unit's own section.

        :return Any: a BambuSpoolman, or None
        """
        if not self._spool_looked_up:
            self._spool_looked_up = True
            try:
                rfid = self.printer.lookup_object('AFC_BambuAMS_rfid', None)
                if rfid is not None:
                    self._spool_obj = rfid.for_unit(self)
            except Exception:
                self.logger.debug(
                    f"AFC bambu {self.name}: could not attach the Spoolman "
                    f"module", traceback=traceback.format_exc())
                self._spool_obj = None
        return self._spool_obj

    # -- The measurement's owner --
    #
    # Turning a measured percent into the slot's remain_pct, the lane's grams
    # and the saved vars is not Spoolman work, but the code lives in the
    # Spoolman delegate. So measurement goes through _measure: the Spoolman
    # delegate when there is one, otherwise a Spoolman-off BambuSpoolman, so
    # a unit without an [AFC_BambuAMS_rfid] section still applies its
    # measurements. Spoolman itself (_spoolman_sync, the binders) stays on
    # _spool and no-ops without the section. _apply_remain_weight settles
    # whatever the adopting delegate still owes, via either one.

    #: The measurement-only delegate, built on first use when no Spoolman
    #: delegate exists. Class attributes for the same reason as _spool_obj.
    _meas_obj: Any = None
    _meas_looked_up: bool = False

    @property
    def _measure(self) -> Any:
        """The object that owns this unit's measurement state.

        The Spoolman delegate when [AFC_BambuAMS_rfid] gives one; otherwise a
        BambuSpoolman built with Spoolman off, once, on first use. Never
        registered with the section, so it does not count as a unit Spoolman
        serves.

        :return Any: a BambuSpoolman, or None if not even the measurement-only
            one could be built
        """
        sp = self._spool
        if sp is not None:
            return sp
        if not self._meas_looked_up:
            self._meas_looked_up = True
            try:
                from extras.AFC_BambuAMS_rfid import measurement_only
                self._meas_obj = measurement_only(self)
            except Exception:
                self.logger.debug(
                    f"AFC bambu {self.name}: could not build the measurement "
                    f"delegate -- measurements will not reach the lanes",
                    traceback=traceback.format_exc())
                self._meas_obj = None
        return self._meas_obj

    def _measure_of(self) -> Any:
        """
        _measure, callable unbound on the duck-typed stand-ins as well.

        Much of this file is called as ``afcBambuAMS.method(self, ...)`` with
        a namespace that has only the attributes a case needs; such a stand-in
        answers with its own ``_spool``.

        :return Any: the measurement delegate, or None
        """
        try:
            if isinstance(self, afcBambuAMS):
                return self._measure
            return getattr(self, "_spool", None)
        except Exception:
            return None

    def _built_measure_objs(self) -> list:
        """
        Every object that may hold this unit's measurement memos, building
        none.

        The unit itself (duck-typed stand-ins carry the memos directly), the
        Spoolman delegate and the measurement-only delegate, each only if it
        already exists. Readers and clearers use this rather than _measure: a
        delegate that was never built holds nothing, and building one would
        give every pool spare a delegate the first time anything looked.

        :return list: the objects, the unit first
        """
        if isinstance(self, afcBambuAMS):
            objs = [self, self._spool_obj, self._meas_obj]
        else:
            objs = [self, getattr(self, "_spool", None)]
        out: List[Any] = []
        for o in objs:
            if o is not None and all(o is not x for x in out):
                out.append(o)
        return out

    def _held_measurements(self) -> dict:
        """
        slot -> the measured percent (RAW) this unit is holding, wherever the
        memo lives.

        Empty on a pool spare: a spare has no lanes, and readings on a shared
        address are not its own.

        :return dict: a new dict; changing it changes nothing held
        """
        if getattr(self, "pool", False):
            return {}
        held: dict = {}
        try:
            # Unit first, so a delegate's figure (the live one) wins a clash.
            for o in afcBambuAMS._built_measure_objs(self):
                held.update(getattr(o, "_measured_remain", None) or {})
        except Exception:
            pass
        return held

    def _shown_remain_pct(self, slot: int, pct: Any) -> Any:
        """
        The remain_pct to publish for a held measurement, on a bay whose lane
        cannot answer for itself (see _lane_remain_pct: no tag nominal -- a
        tagless bay, or one whose profile would not decode -- or no lane).

        The same figure the lane's grams came from: the reel's floor (see
        BambuSpoolman._remain_floor_pct -- filament does not grow, and the
        odometer is worth about +/-3%), not the raw reading the memo keeps as
        its identity. Capped at 100, because the panel's field is a share of
        the spool and a full reel reads proud of the reference radius (the
        grams are capped at the spool's nominal for the same reason).

        :param slot: 0-based AMS slot index
        :param pct: the held RAW percent
        :return Any: the percent to publish, 0..100; pct itself if it is not
            a number
        """
        try:
            shown = int(pct)
        except (TypeError, ValueError):
            return pct
        try:
            uid = ""
            for sl in (getattr(self, "_slots", None) or []):
                if sl.get("index") == slot:
                    uid = str(sl.get("rfid_uid") or "")
                    break
            for o in afcBambuAMS._built_measure_objs(self):
                fl = getattr(o, "_remain_floor_pct", None)
                if callable(fl):
                    shown = int(fl(uid, shown))
                    break
        except Exception:
            pass
        return 0 if shown < 0 else (100 if shown > 100 else shown)

    def _lane_remain_pct(self, slot: Optional[int], sl: dict) -> Optional[int]:
        """
        The remain_pct a bay's lane stands for: its grams, read back through
        the capacity model (BambuSpoolman._pct_for, the inverse of _grams_for).

        A measurement reads back as the percent it was taken at, a spool
        printed from reads what is left, and a restart reads whatever AFC
        restored, with nothing re-applied. Rounded and capped at 100, as the
        panel's field is a share of the spool and a full reel of anything
        lighter than PLA reads above the reference.

        A lane counted down to 0 g reads 0 (not None), so the bay does not
        fall back to a measurement held from before the print. AFC clamps
        weight at 0 g and the reel floor understates, so a present bay can
        reach 0 with filament still on it.

        None (the caller keeps its old answer) without a tag nominal to read
        against (a tagless bay, whose lane weight is only AFC's default and
        would read full), without a lane, or on a pool spare, which has no
        lanes and must not be given a delegate to find that out.

        :param slot: 0-based AMS slot index
        :param sl: the bay's published slot dict (its weight is the nominal)
        :return Optional[int]: 0..100, or None when the lane cannot say
        """
        if getattr(self, "pool", False) or slot is None:
            return None
        try:
            nominal = float(sl.get("weight") or 0)
            if nominal <= 0:
                return None
            lane = self._lane_for_slot(slot)
            if lane is None:
                return None
            grams = float(getattr(lane, "weight", 0) or 0)
            if grams <= 0:
                return 0
            pct = None
            inv = getattr(afcBambuAMS._measure_of(self), "_pct_for", None)
            if callable(inv):
                pct = inv(slot, lane, grams, nominal)
            if pct is None:
                pct = grams * 100.0 / nominal
            shown = int(round(float(pct)))
        except Exception:
            return None
        return 0 if shown < 0 else (100 if shown > 100 else shown)

    def _forget_spoolman_miss(self, slot: int) -> None:
        """
        Delegate to the Spoolman helper when [AFC_BambuAMS_rfid] is enabled.

        :param slot: slot index
        """
        sp = getattr(self, "_spool", None)
        if sp: sp._forget_spoolman_miss(slot)

    def _bind_by_uid_bg(self, *args: Any, **kwargs: Any) -> None:
        """
        Delegate to the Spoolman helper when [AFC_BambuAMS_rfid] is enabled.

        :param args: forwarded to the helper
        :param kwargs: forwarded to the helper
        """
        sp = getattr(self, "_spool", None)
        if sp: sp._bind_by_uid_bg(*args, **kwargs)

    def _spoolman_sync(self, *args: Any, **kwargs: Any) -> None:
        """
        Delegate to the Spoolman helper when [AFC_BambuAMS_rfid] is enabled.

        :param args: forwarded to the helper
        :param kwargs: forwarded to the helper
        """
        sp = getattr(self, "_spool", None)
        if sp: sp._spoolman_sync(*args, **kwargs)

    def _apply_remain_weight(self, *args: Any, **kwargs: Any) -> None:
        """
        Settle what a measurement still owes this bay, on whichever delegate
        adopted it -- with or without Spoolman.

        Two follow-ups can be owed: the figure a Spoolman bind that has not
        landed yet is to be handed (Spoolman delegate only), and grams made
        before the bay's material was known (either delegate). Builds
        nothing: only a delegate that adopted a measurement can owe one.
        Never raises -- it runs inside the status pass.

        :param args: forwarded to the helper
        :param kwargs: forwarded to the helper
        """
        try:
            for o in afcBambuAMS._built_measure_objs(self):
                fn = getattr(o, "_apply_remain_weight", None)
                if o is not self and callable(fn):
                    fn(*args, **kwargs)
        except Exception:
            try:
                self.logger.debug(
                    f"AFC bambu {self.name}: owed measurement follow-up "
                    f"failed", traceback=traceback.format_exc())
            except Exception:
                pass

    def _adopt_measured_remain(self, *args: Any, **kwargs: Any) -> bool:
        """
        Delegate a measurement to _measure -- with or without Spoolman.

        Never raises. The capscan path calls this and then closes the bay's
        window, records the reading as adopted and hands the bus back; a
        raise would skip all three, leaving the window open and the bus held
        until their timeouts. A failure is logged and reported as not
        accepted instead.

        :param args: forwarded to the helper
        :param kwargs: forwarded to the helper
        :return bool: the helper's verdict; False without a helper, or when
            it failed
        """
        try:
            sp = self._measure
            return bool(sp._adopt_measured_remain(*args, **kwargs)) if sp else False
        except Exception as e:
            try:
                # The raise can come after the lane was written (the summary,
                # the save), so the message does not claim the lane is
                # unchanged; the traceback says where it failed.
                self.logger.warning(
                    f"AFC bambu {self.name}: could not apply a measurement "
                    f"({e}); the log has where it failed")
                self.logger.debug(
                    f"AFC bambu {self.name}: measurement apply failed",
                    traceback=traceback.format_exc())
            except Exception:
                pass
            return False

    def _drain_spool_summary(self, *args: Any, **kwargs: Any) -> None:
        """
        Delegate a held operator summary to _measure, which queued it.

        Never raises, for the same reason as _adopt_measured_remain: it runs
        at the end of a status pass.

        :param args: forwarded to the helper
        :param kwargs: forwarded to the helper
        """
        try:
            sp = self._measure
            if sp: sp._drain_spool_summary(*args, **kwargs)
        except Exception:
            try:
                self.logger.debug(
                    f"AFC bambu {self.name}: spool summary failed",
                    traceback=traceback.format_exc())
            except Exception:
                pass

    def _ack_faults(self) -> None:
        """
        Mark every fault the AMS has reported so far as already handled, so a
        later check cannot re-raise it.

        :return None:
        """
        getf = getattr(self._bridge, "last_fault", None) if self._bridge else None
        if callable(getf):
            try:
                self._fault_seen = getf(
                    unit=getattr(self, "ams_index", None))[0]
            except Exception:
                pass

    #: How long to let a re-home settle before re-reading the unit's state.
    #: The motion itself is ~3s (mode 0F/0E); a little over that lets the
    #: state byte catch up without stalling the unload.
    REHOME_SETTLE_S = 3.5

    def _clear_latched_error(self, lane: Any, what: str) -> bool:
        """
        If the unit is sitting in its stalled/error state, run the printer's
        "Retry" reset (re-home) so the next command is not sent into a unit
        that has stopped listening.

        A unit can latch (e.g. while AFC is still forming the tip), and
        commands sent to it then each wait out their full deadline. The unit
        reports this state directly (AMS_STATE_STALLED, the state:7 the HT and
        AMS 2 also print in words), so it is checked rather than discovered
        by timing out.

        A clear that does not take is itself the answer: the unit accepts a
        reset immediately when it can, and refuses while the filament is
        physically stuck. So False means "kicking this again is pointless".

        Best-effort: any failure to read or reset lets the caller proceed as
        normal; a diagnostic must never be the reason an unload does not
        happen.

        :param lane: Lane being unloaded, for the message
        :param what: Short description of what is about to be attempted
        :return bool: False only when the unit is latched AND the reset did not
            clear it; True otherwise (including when it was never latched)
        """
        if self._bridge is None:
            return True
        try:
            before = self._unit_state(self._bridge.latest_status())
        except Exception:
            return True
        if before != self.AMS_STATE_STALLED:
            return True

        self.logger.warning(
            f"AFC bambu {self.name}: unit is latched in error (state "
            f"{before}) before {what} for {lane.name}, re-homing to clear it "
            f"first")
        try:
            self.rehome()
            self.afc.reactor.pause(
                self.afc.reactor.monotonic() + self.REHOME_SETTLE_S)
            after = self._unit_state(self._bridge.latest_status())
        except Exception:
            self.logger.debug(
                f"AFC bambu {self.name}: re-home for {lane.name} raised",
                traceback=traceback.format_exc())
            return True

        if after == self.AMS_STATE_STALLED:
            self.logger.warning(
                f"AFC bambu {self.name}: {lane.name}, the unit would not "
                f"leave its error state after a re-home, so it is jammed "
                f"rather than merely latched. Not sending {what}.")
            return False
        self.logger.info(
            f"AFC bambu {self.name}: {lane.name}, error cleared by the "
            f"re-home (state {before} -> {after}), continuing with {what}")
        return True

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
        # The follower stand-down for this unload is the explicit assist-off
        # (set_feed_assist(False)) below, before the retract.
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

            # Clear a latched unit before any filament is driven at it, including
            # the STN retracts below: a unit in its error state takes up nothing,
            # so the filament buckles at the bay entry.
            self._clear_latched_error(cur_lane, "the toolhead retract")

            # 1) Full STN unload: drive the tip fully out of the extruder gears
            #    and wait for it to finish before the AMS starts reeling, so the
            #    filament has cleared the hotend/gears first.
            if cur_extruder.tool_stn_unload > 0:
                afc.move_e_pos(cur_extruder.tool_stn_unload * -1,
                               cur_extruder.tool_unload_speed, "STN unload",
                               wait_tool=True)
            # 2) Reel the bowden back into the AMS, and at the same time run a
            #    second STN-unload retract (non-blocking) so the extruder gears
            #    keep spinning -- actively driving the filament toward the AMS --
            #    while the AMS pulls it back. This keeps the tip moving through
            #    the gears as the AMS reels, instead of the AMS dragging against
            #    stationary/holding gears (which can strip or jam the filament).
            #    Queue the extruder retract first (async), then kick the AMS so
            #    the two overlap.
            retract_dist = self.afc_unload_bowden_length
            if cur_extruder.tool_stn_unload > 0:
                # Logged because move_e_pos never uses its log_string; this
                # separates a stalled retract from a jammed AMS. Issued, not
                # finished (wait_tool=False); the sensor check decides whether
                # the filament left.
                try:
                    _e0 = afc.gcode_move.last_position[3]
                except Exception:
                    _e0 = None
                self.logger.debug(
                    f"AFC bambu {self.name}: toolhead retract issued for "
                    f"{cur_lane.name}: {cur_extruder.tool_stn_unload:.0f}mm at "
                    f"{cur_extruder.tool_unload_speed:.0f}mm/s from E="
                    f"{'?' if _e0 is None else format(_e0, '.1f')} "
                    f"(queued, overlaps the AMS reel)")
                afc.move_e_pos(cur_extruder.tool_stn_unload * -1,
                               cur_extruder.tool_unload_speed,
                               "STN unload (concurrent with AMS reel)",
                               wait_tool=False)
            # Mark before the first retract: anything past this is a fault this
            # unload provoked, not one left over from the preceding load.
            fault0 = self._ams_fault_seq()
            # Re-check: the unit can latch again during the STN retracts.
            # Only a status read when it is healthy.
            self._clear_latched_error(cur_lane, "the unload retract")
            self.retract(cur_lane, retract_dist)
            # accept_switch_finish and accept_tray_release end the wait on the
            # unit's own word; whichever it speaks fires, and neither is
            # trusted to mean the filament is home. The sensor check below
            # still decides that.
            self._wait_move(retract_dist, fault_mark=fault0,
                            accept_switch_finish=True,
                            accept_tray_release=self._slot_of(cur_lane))

            # Verify the filament actually left the toolhead; a fire-and-forget
            # retract the AMS ignored would otherwise report "unload done". Three
            # checks, two re-kicks: the last pass is a sensor check only, so the
            # second re-kick's result is read.
            RETRIES = 2
            cleared = False
            latched = None
            for attempt in range(RETRIES + 1):
                try:
                    still_loaded = bool(
                        self._toolhead_sensor_triggered(cur_lane))
                except Exception:
                    still_loaded = False
                if not still_loaded:
                    cleared = True
                    break
                if attempt == RETRIES:
                    break                 # out of re-kicks; this pass judged
                # Stop asking a unit that has given up. Only consulted once the
                # sensor says something is wrong, since a lone stall mid-reel is
                # ordinary. Consumed, so the follower tick does not raise it again.
                latched = self._ams_fault_since(fault0)
                if latched:
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
                    f"state: {ams_state}), re-kicking (stop/select/retract, "
                    f"attempt {attempt + 1})")
                self.stop()
                # Don't kick a unit that is still latched: each stop/select/
                # retract would wait out its full deadline. Clear it first; if
                # the clear will not take, the unit is physically stuck, so stop
                # here and let the failure path report it.
                if not self._clear_latched_error(cur_lane,
                                                 f"re-kick {attempt + 1}"):
                    break
                self.select_lane(cur_lane)
                self.retract(cur_lane, retract_dist)
                self._wait_move(retract_dist, fault_mark=fault0,
                                accept_switch_finish=True)
            if not cleared:
                if latched:
                    # Halt only when the unit said it gave up. Without a
                    # verdict it may still be reeling, and a stop would abort a
                    # retract that could yet succeed. A latched unit is not
                    # reeling; it is polling a retract it will never finish
                    # (the ~2 Hz "there is no tray" noise, see below).
                    self.stop()
                    # Re-read the sensor: the loop breaks on the latch without
                    # another sensor pass, and "gripped at the toolhead" and
                    # "stalled reeling home" need different manual fixes.
                    try:
                        still_gripped = bool(
                            self._toolhead_sensor_triggered(cur_lane))
                    except Exception:
                        still_gripped = True
                    if still_gripped:
                        msg = (f"AFC bambu unload failed for "
                               f"{cur_lane.name}: the AMS gave up reeling it "
                               f"back and latched, the filament is still "
                               f"gripped at the toolhead. Free it by hand "
                               f"(heat, then retract), then run "
                               f"{self.get_lane_reset_command(cur_lane, 0.0)}"
                               f".\nAMS said: {_fault_reason(latched)}")
                    else:
                        msg = (f"AFC bambu unload failed for "
                               f"{cur_lane.name}: the filament cleared the "
                               f"toolhead but the AMS latched before reeling "
                               f"it home, the tip is stranded in the "
                               f"bowden. Run "
                               f"{self.get_lane_reset_command(cur_lane, 0.0)}"
                               f" then LANE_UNLOAD to finish the reel.\n"
                               f"AMS said: {_fault_reason(latched)}")
                else:
                    msg = (f"AFC bambu unload failed for {cur_lane.name}: "
                           f"filament still at the toolhead sensor after "
                           f"retract retries, AMS did not reel; run "
                           f"LANE_UNLOAD (eject) or check the unit")
                afc.error.handle_lane_failure(
                    cur_lane, msg, pause=afc.function.in_print())
                return False
            # The filament is home, so stop polling the target tray; otherwise
            # the unit answers "there is no tray" at ~2 Hz for ~34s, audible and
            # noise on a shared bus.
            self.stop()

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
        lane state, so ``tool_loaded`` is valid here (unlike the fixed-delay
        startup timer). A lane tool-loaded across a reboot has its AMS put back
        into mode:4 now, or the extruder would pull against a dead motor until
        the first manual load.

        :param lane: The lane just prepped
        """
        slot = self._slot_of(lane)
        info = self._slots[slot] if slot is not None else {}
        if info.get("present"):
            lane.loaded_to_hub = True
        if (getattr(lane, "tool_loaded", False)
                and self._bridge is not None and slot is not None):
            # A tray threaded across a reboot goes straight back into mode:4.
            self.logger.debug(
                f"AFC bambu {self.name}: {lane.name} tool-loaded at prep, "
                f"engaging follower")
            self._engage_follower(lane)

    def get_lane_reset_command(self, lane: Any, dis: float) -> str:
        """
        Return the gcode AFC_LANE_RESET / AFC_RESET should run to reset this
        lane. Stepperless AMS lanes can't be reset by moving a lane stepper --
        only the bridge can reel the filament back -- so route AFC_RESET to
        AFC_BAMBU_RECOVER (stop + reel to bay + reset state), the same recovery the
        eject path uses. Mirrors AFC_ACE.get_lane_reset_command.

        :param lane: Lane to reset
        :param dis: Reset distance (unused; the recover reels the full path)
        :return str: the AFC_BAMBU_RECOVER command for this unit and lane
        """
        return f"AFC_BAMBU_RECOVER UNIT={self.name} LANE={lane.name}"

    #: Bounds on a path length derived from the odometer, in mm: a
    #: garbled-read guard, wide enough for any real tube and narrow enough to
    #: reject a sign flip or a stuck sentinel.
    ODOM_PATH_MIN_MM = 300.0
    ODOM_PATH_MAX_MM = 8000.0

    def _odom_now_mm(self) -> Optional[float]:
        """
        This unit's odometer position in mm, from the binary status frame.

        Not from narration: tube_len and dw_len are text whose wording differs
        between units (the AMS 1 narrates neither). The odometer is a typed
        field, so a path length taken from it needs no dialect.

        Negative is a valid reading: the resting position is slightly below
        zero and an unload runs negative. Only the firmware's -1 mm
        unknown-sentinel is excluded, matching get_status.

        :return float: position in mm, or None if the unit has not reported one
        """
        try:
            latest = self._bridge.latest_status() if self._bridge else None
            u = afcBambuAMS._unit_entry(self, latest or {})
            v = u.get("odom") if u is not None else None
            if v is None or int(v) == -1:
                return None
            return float(v)
        except Exception:
            pass
        return None


    def _dw_len_str(self) -> str:
        """
        This unit's last end-of-feed length and sample count, for diagnostics.

        :return str: "3661mm n=2", or "None" if it has never said one
        """
        try:
            mm, n, _addr = self._bridge.dw_len(self.ams_index)
        except Exception:
            return "None"
        return f"{mm:.0f}mm n={n}" if mm else "None"

    def _dw_len_mm(self) -> Optional[float]:
        """
        This unit's PTFE path from its own end-of-feed length, in mm.

        The HT's only source: it reports odom as None. At the end of every
        feed it narrates "dw_len:3.672 m", which is repeatable to about 1 cm.

        Address-checked as well as unit-keyed: dw_len is filed under
        _active_unit, which a load sets and nothing clears, so it names
        whichever unit loaded last. A value that arrived on a device address
        this unit does not use is refused; that separates HT from boxed units
        (0x1800 vs 0x0700). Two boxed units share an address and are separated
        only by the unit key.

        :return float: path length in mm, or None
        """
        try:
            mm, _n, addr = self._bridge.dw_len(self.ams_index)
        except Exception:
            return None
        if not mm:
            return None
        mine = int(getattr(self, "dry_dev_addr", 0) or 0)
        if addr and mine and int(addr) != mine:
            return None
        if not (self.ODOM_PATH_MIN_MM <= mm <= self.ODOM_PATH_MAX_MM):
            return None
        return float(mm)

    def _path_measurement(self) -> tuple:
        """
        This unit's own PTFE length, from whichever source this unit provides.

        No single source covers every unit type, so this falls through in
        order of directness:

            odometer   distance travelled to the toolhead sensor this load
            dw_len     the unit's own end-of-feed figure for that journey
            tube_len   a stored self-calibration, narrated by few units

        A missing source means apply nothing: (None, "") leaves the
        configured value. No derived, averaged or defaulted figure is
        substituted.

        :return tuple: (mm, source) or (None, "")
        """
        mm = self._measure_path_from_odom()
        if mm is not None:
            return mm, "odometer"
        mm = self._dw_len_mm()
        if mm is not None:
            return mm, "dw_len"
        mm = self.measured_path_mm()
        if mm is not None:
            return mm, "tube_len"
        return None, ""

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
            # By unit first: an AMS 1 and an AMS 2 Pro share device address
            # 0x0700, so the address alone could adopt one unit's measurement
            # as the other's. The address-only call is the fallback for a
            # bridge that does not take ``unit``.
            return br.tube_len(getattr(self, "dry_dev_addr", None),
                               unit=self.ams_index)
        except TypeError:
            return br.tube_len(getattr(self, "dry_dev_addr", None))
        except Exception:
            return None





    def _measure_path_from_odom(self) -> Optional[float]:
        """
        The bay-to-toolhead-sensor distance this load just travelled, in mm.

        Both ends of the delta are taken at defined moments (before any
        motion, and when the toolhead sensor trips) because the odometer keeps
        climbing afterwards as the toolhead consumes filament through tool_stn
        and the purge.

        A delta, not the raw reading: the odometer is a position, zero only if
        the previous unload finished cleanly, and a lane staged at the hub
        starts partway along.

        Not taken from the unload: that trace runs 0 -> -0.999 and resets to 0
        without spanning the tube.

        :return float: the measured distance in mm, or None if either end of
          the delta is missing or the result is outside ODOM_PATH_MIN/MAX_MM
        """
        a = getattr(self, "_load_odom_start", None)
        b = getattr(self, "_load_odom_at_sensor", None)
        if a is None or b is None:
            return None
        span = b - a
        if not (self.ODOM_PATH_MIN_MM <= span <= self.ODOM_PATH_MAX_MM):
            return None
        return span

    def _adopt_measured_path(self, measured: Optional[float] = None,
                             source: str = "") -> None:
        """
        Adopt the AMS's own path measurement as afc_bowden_length, in memory.

        The measured figure (see _path_measurement) is the real distance on
        this machine. It is adopted in memory and persisted through the
        master's persist_learned(), not via ConfigRewrite: ConfigRewrite would
        write an [AFC_BambuAMS <name>] section into AFC_auto_vars.cfg, which
        klippy parses, and for a pool-fabricated unit that orphan section
        fails check_unused_options at the next restart. AFC_BridgeBox's
        state_file is never parsed by klippy, and its _fold_and_sweep overlays
        the stored value onto the unit at boot.

        A unit with no master still adopts in memory; persistence only lets
        the first load of the next session size its give-up window from the
        measured value instead of the default. The path length is otherwise
        advisory: the toolhead sensor and the unit's own report end every move.

        Runs once per session, and only writes when the figure differs by at
        least PATH_ADOPT_TOLERANCE_MM (the measurement wobbles by a few mm).

        afc_unload_bowden_length follows only if it was tracking the bowden
        length (its default); a deliberately set value is kept.

        :param measured: length in mm to adopt; None looks one up
        :param source: short label for where the figure came from, for the log
        """
        if getattr(self, "_path_adopted", False):
            return
        if measured is None:
            measured, source = self._path_measurement()
        if measured is None:
            # No measurement. Not warned about: the path length is advisory
            # (commanded distance and give-up window).
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
        # Persisted through the master, never by ConfigRewrite (see the
        # docstring). Best effort.
        saved = False
        master = getattr(self, "_master", None)
        if master is not None:
            try:
                saved = master.persist_learned(
                    self.name, "afc_bowden_length", round(measured, 1))
                if follow_unload:
                    master.persist_learned(
                        self.name, "afc_unload_bowden_length",
                        round(measured, 1))
            except Exception:
                saved = False
        self.logger.info(
            f"AFC bambu {self.name}: the AMS measured its own filament path at "
            f"{measured:.0f}mm (was {old:.0f}mm)"
            f"{' via ' + source if source else ''}. Adopting it"
            f"{' and saving it' if saved else ' for this session'} -- this "
            f"sizes the load give-up deadline.")

    def _ready_to_follow(self, lane: Any = None) -> bool:
        """
        Always True: a loaded tray is always followed.

        Does not gate on the extruder motor being energised. A stock printer
        holds a loaded tray with op-04 mode 07 / ref 7F continuously and does
        not check the steppers; gating on them would stand the follower down
        after an idle timeout, so filament pulled by hand would not be
        recovered. Any buffer packing from feeding into un-gripped gears shows
        up in the buff value the unit reports on every poll.

        The signature is kept so callers and tests do not change.

        :param lane: unused; kept for call-site compatibility
        :return bool: True
        """
        return True

    def _eject_distance(self) -> float:
        """
        How far to command the eject retract.

        Prefers the AMS's own measured path plus ``eject_buffer``, falling back
        to the configured estimate. Takes the larger of the two: on this path
        a short distance leaves filament in the tube with no sensor to notice,
        while a long one just means the AMS finishes early and says so.

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
        AFC_BAMBU_RECOVER, and AFC_RESET (via get_lane_reset_command).

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
        # The stop() below halts AMS motion and is the follower stand-down for
        # the eject.
        try:
            self.stop()
            self.select_lane(lane)
            dist = self._eject_distance()
            fault0 = self._ams_fault_seq()
            self.retract(lane, dist)
            finished = self._wait_move(dist, fault_mark=fault0)
            self.stop()
            # The unit's own verdict, if it reached one. Consumed and reported
            # here because the follower tick's fault check is muted while
            # _unload_in_progress, so it would otherwise surface late or not
            # at all.
            latched = self._ams_fault_since(fault0)
            if latched:
                self.logger.warning(
                    f"AFC bambu {self.name}: "
                    f"{getattr(lane, 'name', 'lane')} could not be reeled back "
                    f"-- the AMS gave up and latched. The filament is still "
                    f"out of the bay; free it by hand before loading this lane "
                    f"again.\nAMS said: {_fault_reason(latched)}")
            elif not finished:
                # This path has no sensor: without a completion, _wait_move
                # returns on its deadline and the filament may be short of the
                # bay. getattr because eject is a recovery path and must not
                # raise on a bare lane object.
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
