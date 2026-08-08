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
# spool capacity measurement, select + feed/retract + self-centering follower,
# the AMS's own multi-stage load/unload motion, drying, jam detection, and
# humidity/temperature reporting are all confirmed on a live three-unit bus:
# a stock boxed AMS, an AMS 2 Pro and an AMS HT together.
# The wire protocol was reverse-engineered from live printer<->AMS captures, kept
# with the bridge firmware alongside the findings they support.
#
# ── TWO NUMBERING SCHEMES. DO NOT MIX THEM. ──────────────────────────
# "ams1" means two different things and the collision has cost real time:
#
#   ams_model: ams1   the MODEL -- a regular boxed AMS (vs ams2, ht)
#   "get_slot ams1"   the CHAIN ID in the unit's own narration -- position on
#                     the wire, so `ams1` there is chain index 1, which is the
#                     SECOND unit and may well be an AMS 2 Pro.
#
# A Klipper unit name (BambuAMS_1, BambuAMS_2, BambuAMS_HT) is a third,
# unrelated label chosen by the operator. When reading narration, resolve the
# chain id through ams_index -- never through the unit's name or its model.
#
# ── Jam detection ───────────────────────────────────────────────────
# THREE signals, because the units disagree about how to say "error" and one of
# them never uses the WORD. Each unit was stuck-spooled deliberately, alone on
# the wire (2026-08-05):
#   * the unit's own words -- the HT declares it 2.0s before the printer even
#     reacts ("timeout, assist finish stall!", state:7, err_code: 0 -> 23), and
#     the AMS 2 likewise ("[AMS_LED]TIMEOUT error N"). Both lead the buffer.
#   * byte[19] of the op-04 reply (`ustate`) -- 04 healthy, 07 stalled. This is
#     the one signal EVERY generation sends, with no dialect and no spelling to
#     get wrong, and on the two units that also narrate it reads 07 at exactly
#     the moment they print "state:7". Debounced; a single reading cannot pause
#     a print.
#   * buffer starvation -- bottomed out while the extruder keeps pulling.
# The AMS 1 is why there are three. It narrates continuously, but it never uses
# fault WORDS: it reports the condition as state, "state:6" on [AMS_COMMON] and
# "en:0,mode:7,idx:255" on [AMS_LINK]. Reading for vocabulary leaves that unit
# with no fault detection at all -- an earlier version of this file claimed it
# "reports NOTHING", which was our matcher's blind spot, not the unit's silence.
# Any one of the three pauses the print, drops the assist, and holds the
# follower down until the print resumes -- re-arming into a jam just grinds the
# filament.
#
# A HOLD CLEARS THE WAY IT WAS RAISED: the unit leaving its fault, or the human
# resuming. Nothing watches the buffer to release it automatically -- "the
# buffer came off the floor" does not mean "somebody freed the jam". During a
# toolchange it means the opposite: the quick pull and the cut's own retract
# lift the buffer while the nozzle is still being cut.
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
#   * narration dialect: THE PUNCTUATION AFTER "STEP" IS WHAT DIFFERS, and the
#     bracket tag does NOT predict it. Counted across the captures:
#
#         [AMS_DEV] STEP:      x7      [AMS_RFID] STEP:   x3
#         [AMS_DEV] STEP,      x1      [AMS_RFID]STEP:    x1   (no space)
#                                      [AMS_RFID] STEP3,  x11  (digit + COMMA)
#
#     So both tags appear with both separators, with and without the space.
#     Match "STEP", an OPTIONAL DIGIT, then ':' or ',' -- that is what
#     _STEP_SEP in AFC_BambuAMS_bridge.py encodes, and _STEP() builds on.
#
#     This is not a style note. A matcher anchored on the literal "STEP:" is
#     silently HT-blind and still looks correct on a bench with a boxed unit:
#     _RFID_READ_OK_RE was exactly that, so an AMS HT that read its tag
#     perfectly was indistinguishable from one that never read a thing, and
#     four rounds of firmware were spent hunting it on the bus. Write the rule
#     against the punctuation, not against one model's habits.
#   * the follower is NOT per-model. Every unit is held the same way -- op-04
#     07/7F at 148 ms, the cadence a real printer uses -- with no buffer
#     threshold to tune. There is no "an AMS 2 refills its own buffer, a plain
#     AMS must be fed on demand" distinction -- measured, a regular AMS sits at
#     0.56-0.59 on the virtual FPS, indistinguishable from an HT.
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
#   # measure_on_insert: True          # measure spool capacity on insert, or
#                                      # just read the tag (boxed units; an HT
#                                      # measures in firmware regardless).
#                                      # Bambu-tagged spools only -- the unit
#                                      # will not measure any other kind.
#   # fault_detect: True               # act on jams (see Jam detection above)
#   # fault_pause: True                # pause the print on a jam
#
# ── Spoolman ────────────────────────────────────────────────────────
#   # sync_measured_to_spoolman: True  # write the AMS's PHYSICAL measurement
#                                      # (P:NN% by radius -> grams) back to the
#                                      # bound spool's remaining_weight.
#                                      # FALSE changes ONE thing: that write.
#                                      # The scan still runs, the spool is still
#                                      # measured, the lane still gets the grams
#                                      # and shows them -- Spoolman just is not
#                                      # corrected. The spool summary says so
#                                      # ("Spoolman sync is off, so this is kept
#                                      # on the lane only") rather than going
#                                      # quiet.
#   # auto_spoolman_create: False      # create a filament+spool in Spoolman
#                                      # from a tag it has never seen. Off by
#                                      # default: matching binds an existing
#                                      # spool by tag UID, which is almost
#                                      # always what you want.
#
# ── Everything else, by area ────────────────────────────────────────
# Defaults are shown; all are optional and none need setting for a working
# unit. Each is commented where it is read.
#   identity/bus    unit_uid("")  bus_serial("")  mc_dev_addr  mc_id_base
#                   mc_ams_id(-1)  rollcall_span_boxed  rollcall_span_ht
#   load path       afc_bowden_length  afc_unload_bowden_length  eject_buffer(200)
#                   tool_bite_mm(0)  pull_settle_s(6)
#                   pull_push_dwell_s(0)  arrival_select(False)
#                   arrival_assist_delay_s(4)
#   load retry      load_retry_timeout(40)  load_retry_interval(4)
#                   load_retry_pulse(100)  load_recover_attempts(2)
#                   reel_back_on_load_fail(False)  auto_error_recovery(False)
#   follower        follow_when_loaded(True)  follow_poll_interval(0.1)
#                   follow_rearm_window(3)  follow_min_extrude(0.1)
#                   follow_debug_interval(0)  ht_0f_hold(True)
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
# reader; this module also nudges a scan on the insert edge. The AMS HT is
# different in a way that shapes the whole path: it scans on its own preload
# switch, too fast for a module round trip, so the FIRMWARE arms the window on
# the insert edge and sends the type-07 select. The id is the unit's own
# chain-derived address (HT_ID_DEFAULT 0x00); the 0x80 in the printer captures
# was that printer's registration for its HT, not a constant.
#
# THE HT'S TAG LIVES IN ITS OWN FLASH, and a plain 0x0211 read is answered from
# that cache instantly, with no motion. So "read the record" and "scan the tag"
# are DIFFERENT QUESTIONS on an HT and the same question on a boxed unit. Read
# before the unit has committed a card and you get the previous spool.
#
# The window therefore reads NOTHING until the unit says it has the card
# ("save to flash ,card info valid", ~12 s after the insert edge), then reads
# once and takes the answer. There is no timer and no staleness compare: an
# earlier version read at a fixed 5 s and latched the old tag eight seconds
# early, and every guard layered on top of that was filtering an answer
# collected before the question had finished. If the window closes with no card
# announced, the bay is resolved-as-blank rather than filled from the cache.
#
# AFC_BAMBU_SCAN / AFC_BAMBU_CAPSCAN send the printer's "re-identify" variant, which
# physically re-scans even a SEATED spool -- so re-reading the same spool works,
# which is the case that has to work.
#
# COMMAND, THEN LISTEN. That is the whole module side, and it is four pieces:
#
#   _open_scan       we asked. Stamps _scan_t0[slot]; every scan goes through
#                    here, auto or manual, so all of them end the same way.
#   _scan_verdict    what the unit said: none / waiting / read / notag. The
#                    ONLY thing that decides, and it decides from the unit's
#                    narration, never from the record's contents or a clock.
#   _sync_lanes      acts on the verdict, once per status frame. waiting ->
#                    touch nothing; read -> _surface_slot_info; notag ->
#                    _finalize_scan.
#   _finalize_scan   the no-tag outcome: clear the lane, drop the Spoolman
#                    link, apply AFC defaults.
#
# TWO OUTCOMES, NEVER THREE. Either the unit read a tag and its record is this
# spool's, or it did not and the lane gets defaults. There is no third ending
# in which the previous spool's profile is left on the lane.
#
# Nothing here is on a clock, and nothing infers the verdict from the record's
# contents. Both are unreliable for the same reason: re-scanning the same spool
# returns identical bytes, so a changed record cannot mark the end of a read,
# and a fixed per-model window either expires on a tag still in flight or holds
# the lane stale long after the answer arrived. The unit narrates its read about
# a second in and narrates the end of its cycle either way, so neither number
# has to be guessed.
#
# ── Commands ─────────────────────────────────────────────────────────
# EVERYDAY
#   AFC_BAMBU_UIDS                                     list bus UIDs + firmware
#                                                  version; start here on setup
#   AFC_BAMBU_SCAN     UNIT=<u> [LANE=<lane>]          re-read a tag (works on a
#                                                  SEATED spool)
#   AFC_BAMBU_CAPSCAN  UNIT=<u> LANE=<lane>            scan AND measure the spool
#   AFC_BAMBU_HEATER_START UNIT=<u> [TEMP=] [TIME=] [ROTATE=]   start drying (ams2/ht)
#   AFC_BAMBU_HEATER_STOP  UNIT=<u>                             stop drying
#   AFC_BAMBU_RECOVER  UNIT=<u> LANE=<lane>            reel a failed load back
#   AFC_BAMBU_RELINK   UNIT=<u>                        clear a TIMEOUT/error
#   AFC_BAMBU_CLEARFAULT UNIT=<u>                      drop a latched fault; fails
#                                                  if the jam is still there
#   AFC_BAMBU_FOLLOWER UNIT=<u> LANE=<lane> [ENABLE=]  engage/stop the follower
#
# WHEN SOMETHING IS WRONG -- read-only, safe to run any time
#   AFC_BAMBU_LATEST   UNIT=<u>      the whole status frame: slots, counters, and
#                                the HT scan chain (htarm/htcard/htread/htgive).
#                                htgive climbing = the HT stopped announcing its
#                                card. dbgtexts frozen at 0 while dbgframes
#                                climbs = the narration channel is severed.
#   AFC_BAMBU_SLOTTRACE UNIT=<u> [S=]   watch one unit's slot records change
#   AFC_BAMBU_RC / AFC_BAMBU_ROLLCALL       roll-call state and diagnostics
#   AFC_BAMBU_CLSPROBE / AFC_BAMBU_MDIAG    class-addressing and motion diagnostics
#
# BRING-UP ONLY -- these CHANGE BUS BEHAVIOUR; do not leave them set
#   AFC_BAMBU_MUTE UNIT=<u> MASK=<bits>   silence one class of frame (0 restores).
#                                     256 = the presence poll, the usual suspect
#                                     when a unit ticks at idle.
#   AFC_BAMBU_ARMMS UNIT=<u> MS=<ms>      11/04 keep-alive cadence -- the one
#                                     transmitter AFC_BAMBU_MUTE cannot silence.
#   AFC_BAMBU_AUTOSCAN / AFC_BAMBU_REID / AFC_BAMBU_BITE / AFC_BAMBU_FEED / AFC_BAMBU_DRIVE /
#   AFC_BAMBU_SETTLE / AFC_BAMBU_ARRIVAL / AFC_BAMBU_TAIL / AFC_BAMBU_EXTMIMIC /
#   AFC_BAMBU_HB / AFC_BAMBU_HTPOLL / AFC_BAMBU_POLLMS / AFC_BAMBU_DRAIN / AFC_BAMBU_HTID /
#   AFC_BAMBU_CLASSADDR / AFC_BAMBU_TXECHO / AFC_BAMBU_BUFFER_PROBE / AFC_BAMBU_SENDRAW
#                                     cadence, addressing and frame sweeps.
#                                     Each carries its own usage string; run it
#                                     with no arguments to see it.
#
# ── Daisy-chained AMS (up to 12) ─────────────────────────────────────
# Bambu's wire tops out at 4 four-slot AMS (ams1/ams2, device 0x0700) PLUS up to
# 8 AMS HT (device 0x1800) = 12 units total, never more than 4 four-slot units.
#
# STRONGLY RECOMMENDED with >1 AMS: set `unit_uid` per unit. The firmware assigns
# chain indices by ANNOUNCE ORDER, which reshuffles across power-cycles -- so a
# fixed ams_index can silently start addressing the wrong physical AMS after a
# cold boot. With unit_uid set, each unit pins to its physical AMS by UID no
# matter what order they boot. Run AFC_BAMBU_UIDS to read the UIDs off the wire
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
import json
import logging
import re
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
try:
    from extras.AFC_RFID import (sync_rfid_to_spoolman,
                                 get_auto_spoolman_create,
                                 find_spool_by_uid, SpoolmanClient)
except Exception:                        # older AFC_RFID without these helpers
    sync_rfid_to_spoolman = None         # type: ignore
    get_auto_spoolman_create = None      # type: ignore
    find_spool_by_uid = None             # type: ignore
    SpoolmanClient = None                # type: ignore


def _bambu_spoolman_client(afc: Any):
    """
    Build the SpoolmanClient the way every AFC reader does.

    afc.spoolman is only a configured flag/URL, NOT the client, so calling
    client methods on it silently does nothing.

    :param afc: the AFC printer object
    :return Optional[SpoolmanClient]: the client, or None if Spoolman or
        moonraker is unavailable
    """
    if (SpoolmanClient is None or afc is None
            or getattr(afc, "spoolman", None) is None
            or getattr(afc, "moonraker", None) is None):
        return None
    try:
        return SpoolmanClient(afc.moonraker)
    except Exception:
        return None


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
        # Spool remaining capacity, percent, from the tag's persisted remain
        # fraction (0x0211 reply, RGBA-anchor +29). The AMS maintains it via
        # the odometer during its insert calibration; verified per-spool-stable
        # across captures months apart and a live read of the same spool. 0 is
        # a real value ("never measured" on every fresh tag seen so far, or an
        # exhausted spool); -1/absent means the field was not read.
        "remain_pct": (slot.get("remain")
                       if isinstance(slot.get("remain"), int)
                       and slot.get("remain") >= 0 else None),
        # The tag UID -- Bambu DOES expose it, confirmed byte-for-byte against
        # an OpenAMS reading the same spool. "uid" is the 4-byte Mifare chip
        # UID (the card_uids match key, shared with every other reader on this
        # printer); "tray_uid" is Bambu's 16-byte tray id. Empty until a read.
        "rfid_uid": (slot.get("uid") or None),
        "tray_uid": (slot.get("tray_uid") or None),
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

#: How long after a start command a refusal is ignored, in seconds.
#:
#: The bridge parses narration on its own thread, so a line the AMS emitted
#: BEFORE clear_dry_error() ran can be recorded after it -- and a stale refusal
#: then read as this attempt's. That made a perfectly good dry show DRY REFUSED
#: for a few seconds before flipping to the target temperature once telemetry
#: arrived. A real refusal is repeated by the unit and easily outlives this.
DRY_REFUSE_GRACE = 20.0


def _bb_crc8(d: bytes) -> int:
    c = 0x66
    for x in d:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x39) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def _bb_crc16(d: bytes) -> int:
    c = 0x913D
    for x in d:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def _poll37_frame(dev_addr: int, ams_id: int) -> str:
    """
    Build the 0x37 tray poll for a device, as hex.

    Byte-for-byte the frame the printer sends: the firmware's own FR_MC_3702 is
    reproduced exactly by this function for 0x0700 id 0, which is what says the
    construction is right rather than merely plausible.

    :param dev_addr: 0x0700 for a boxed AMS, 0x1800 for an HT
    :param ams_id: the unit's id byte
    :return str: hex, ready for {"cmd":"raw"}
    """
    f = bytearray(18)
    f[0] = 0x3D
    f[1] = 0x05
    f[4] = 0x12
    f[6] = _bb_crc8(bytes(f[:6]))
    f[8] = (int(dev_addr) >> 8) & 0xFF          # target high byte
    f[10] = 0x03                                 # source 0x0300 (printer)
    f[11] = 0x37
    f[12] = 0x02
    f[13] = int(ams_id) & 0xFF
    c = _bb_crc16(bytes(f[:16]))
    f[16] = c & 0xFF
    f[17] = (c >> 8) & 0xFF
    return bytes(f).hex()


def _unit_tool_loaded(unit: Any) -> bool:
    """
    Is ANY lane on this unit threaded to the toolhead?

    Deliberately broader than afcBambuAMS._tool_loaded_lane(), which answers a
    different question -- that one finds the lane belonging to the ACTIVE
    extruder so the follower can be armed for it. Here the extruder does not
    matter: a tag scan feeds filament past the bay reader and pulls it back, and
    the AMS runs that cycle for the WHOLE unit, so any loaded lane on the same
    unit is disturbed by it no matter which toolhead owns it.

    A module function rather than a method so the scan path stays callable with
    a plain stand-in object, which is how the tests drive it.

    Fails OPEN: if the lane map cannot be read this returns False and the scan
    proceeds, which is the behaviour that existed before this guard. A check
    that cannot answer should not silently stop tags from ever being read.

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
    """Host pin chip exposing one BUS MASTER's buffer as a virtual ADC.

    `adc_pin: bambu_buffer:fps` -> the buffer of the units on that Pico.

    ONE CHIP PER MASTER, not per printer. Every unit on a bus feeds one
    buffer and therefore one extruder, so the reading is shared across the
    units of a single Pico -- but a second Pico is a second buffer and a
    second extruder, and must not read the first one's value. Registering a
    single printer-wide chip made every `bambu_buffer:` pin report whichever
    unit happened to initialise first, which with `tool_start: buffer` is the
    toolhead authority reading the wrong bus entirely.

    The chip NAME is what an [AFC_buffer] section references, so a second
    master needs its own: set `buffer_chip_name` on its units and point that
    buffer's adc_pin at it.
    """

    def __init__(self, unit: Any, name: str = _BUFFER_CHIP_NAME,
                 report_time: float = 0.100) -> None:
        self.printer = unit.printer
        self._unit = unit
        self._name = name
        self._report_time = report_time
        self._pins: List[_BambuBufferADC] = []
        self._timer = None
        self.printer.lookup_object("pins").register_chip(name, self)
        self.printer.register_event_handler("klippy:ready", self._start)

    def setup_pin(self, pin_type: str, pin_params: dict) -> _BambuBufferADC:
        if pin_type != "adc":
            raise self.printer.config_error(
                "%s only provides 'adc' pins "
                "(use adc_pin: %s:fps)" % (self._name, self._name))
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
    """
    Register a buffer ADC chip for this unit's BUS MASTER, once per master.

    Keyed by chip name, which defaults to "bambu_buffer" and is settable per
    unit. Units sharing a Pico share a buffer and so share the chip -- the
    first to initialise creates it and the rest find it here. A second Pico
    gets its own chip under its own name, and reads its own buffer.

    One chip per PRINTER would silently give every `bambu_buffer:` pin the
    first-initialised unit's reading.

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


#: How often the firmware re-sends the 11/04 follower arm, in ms. Applied at
#: announce, so it survives a Pico reboot without a reflash.
#:
#: The firmware's built-in default is 520 ms, matched to the printer's own
#: observed cadence. It does not need to be anywhere near that. Measured with
#: the arm effectively disabled and everything else at stock: the follower held
#: for 5 min 33 s, and every filament pull was recovered in about a second with
#: no degradation and no "assist finish". The arm is not what sustains
#: following -- the AP2 sync is, which muting it demonstrated twice by
#: producing the drop/re-engage pulsing.
#:
#: Not set to "once and never again", tempting as that is. The same loop is
#: what arms a unit that comes online LATER, and it skips any unit that has not
#: answered within ONLINE_TIMEOUT_MS -- so it doubles as the re-arm for a unit
#: that dropped and came back, so it is not turned off entirely -- an hourly
#: backstop keeps that path alive while making the frame effectively one-shot.
#:
#: INTERIM. The right design is an acknowledged arm rather than a timer: the
#: firmware already latches the unit's own follower state out of narration
#: ("[AMS_COMMON]state:4," = following, state:0 = dropped), so it can arm once
#: and re-arm only when the unit has NOT confirmed. Two things block that
#: today -- s_ams_state is global rather than per-unit, so on a chain it holds
#: whichever unit narrated last; and the latch has a missing-braces bug where
#: s_ams_state_n++ runs unguarded. Both are firmware, so this constant is the
#: stand-in until that is built and flashed.
FOLLOW_ARM_MS = 3600000.0

#: How long to let a freshly-connected Pico settle before announcing to it.
#:
#: Measured: a MANUAL AFC_BAMBU_RESTART (issued from a long-settled Klipper) fixes
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

#: Dwell after the unit reports its mode:4 pull is done, covering the PUSH BACK
#: FORWARD half of the cycle -- which the unit does not announce. ("assist
#: finish" was tried and is not it: measured, it fires BEFORE the arrival, so
#: requiring it made the wait always time out.) The pull itself runs 0.5-2.2s,
#: so the push is given comparable room. Also covers the fact that the
#: narration is the unit's REPORT, not a guarantee the shaft has stopped.
PULL_PUSH_DWELL_S = 1.5


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




#: Narration fragments that are link/keep-alive chatter, not a reason. The unit
#: streams these continuously; they are in the buffer at the moment of a fault
#: purely because they are in it at every moment.
_FAULT_NOISE = ("[AMS_COMMON]", "[AMS_LINK]", "[AMS_IDLE]", "[AMS_LED]TRAY")

# THE AMS 1 CARRIES ITS VERDICT ON CHATTER TAGS.
#
# NOT because it is silent -- it is not, and calling it silent is the error
# this project has made twice. Its give-up buffer is full of narration:
#
#     [AMS_DEV] STEP:odom search, odo 0.516 ... 1.185     (hunting, not feeding)
#     [AMS_DEV] STEP:odom reset tray 0
#     [AMS_IDLE]set ams state switch
#     [AMS_COMMON]state:6,tray_now:255,tray_exit:6
#     [AMS_LINK]en:0,mode:7,idx:255,ref:0
#
# [AMS_DEV] is not chatter, so plenty survives the tag filter. What does NOT
# survive is the part that says it GAVE UP: state:6 rides on [AMS_COMMON] and
# en:0,mode:7 on [AMS_LINK], both keep-alive tags -- [AMS_COMMON] carries
# "en:1,mode:3,idx:0,ref:0" thousands of times a print. So the operator got a
# real message with real fragments in it, all of them odometry, and the verdict
# filtered out. The other two units are unaffected: they say "stall" and
# "TIMEOUT error" under their own tags, which were never chatter.
#
# state:6 AND state:7. Both are give-up states carried on a chatter tag, and
# only the first was listed -- so an HT that said "state:7 ... TIMEOUT error 0"
# had its state quietly dropped from its own error message. 7 is the one
# _check_unit_stalled reads as a byte (AMS_STATE_STALLED, 0x07, absent from
# 13,000+ healthy replies); this is the same declaration in words.
#
# Affects the MESSAGE only. Detection is the bridge's classifier, which is a
# separate list -- widening this cannot make anything fault that did not
# already, it can only stop the reason being censored after the fact.
_FAULT_SIGNAL = re.compile(
    r"state:[67]|en:0,\s*mode:7|stall|finish -1|timeout error", re.I)


def _fault_reason(text: str) -> str:
    """
    The part of the AMS's narration that says WHY, without the chatter.

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


# One BambuBridge per serial port, shared by all units on that Pico (a
# daisy-chained AMS shows up as several AFC units on one bus / one bridge).
_BRIDGES: Dict[str, "BambuBridge"] = {}

# RESUME is a PRINTER-WIDE g-code name. Exactly one [AFC_BambuAMS] section may
# wrap it, however many units are on however many buses -- wrapping twice would
# build a chain that reloads a lane twice, and re-registering a name Klipper has
# already handed out is how you lose RESUME for the whole machine. Module-level,
# not per-instance, because the units are separate objects.
_RESUME_WRAPPED = False

# The name our wrapper hands the previous RESUME handler, mirroring AFC's own
# _AFC_RENAMED_RESUME_. Two links in one chain: the button reaches us first, we
# reload the lane, then we call this -- which is AFC's handler, which in turn
# calls the printer's original.
AFC_BAMBU_RENAMED_RESUME = "_AFC_BAMBU_RENAMED_RESUME_"


def _bridge_log_tag(serial_port: str) -> str:
    """
    Short, stable tag naming a bus master for its narration log.

    Empty for the FIRST master registered, so a single-Pico printer keeps
    writing AFC_BambuAMS.log and nothing about it changes. Subsequent masters
    get a tag derived from their serial port, which is what distinguishes
    them -- _BRIDGES is keyed by it.

    Derived from the port rather than counted, so a master keeps the same file
    across restarts regardless of which one initialises first. Only the
    question of who gets the UNSUFFIXED name depends on order, and with one
    Pico there is no such question.

    :param serial_port: the port this master is reached on
    :return str: a filename-safe tag, or "" for the first master
    """
    if not _BRIDGES:
        return ""
    base = str(serial_port).rsplit("/", 1)[-1]
    safe = "".join(c if (c.isalnum() or c in "-_") else "_" for c in base)
    return safe[-24:].strip("_") or "alt"


# ── AFC unit ────────────────────────────────────────────────────────────────────

class afcBambuAMS(afcUnit):
    """
    AFC unit for a stock Bambu AMS behind the Pico Bambu-Bus bridge.

    Mirrors the bridge's per-slot status onto AFC lanes and issues transport as
    bridge commands. Deeper AFC load/unload orchestration is marked TODO where it
    must bind to the base unit's sequencing.
    """

    SLOTS_PER_UNIT = SLOTS_PER_UNIT

    # BACKSTOP for the MOTION guard -- how long _scan_in_flight will keep
    # ignoring this bay's presence flap when the unit never announces the end
    # of its scan. Normally unused: every model narrates an end ("Calibration
    # rst:0" HT, "odom calib success exit 0" AMS 1, "STEP7:cali end" AMS 2) and
    # last_scan_end closes the guard there.
    #
    # Wrong in both directions if it becomes the primary signal: too short and
    # the unit's own retract re-triggers the scan, which loops every 38-42 s;
    # too long and a real removal goes unnoticed for a minute and a half.
    SCAN_MOTION_QUIET_S = 90.0

    # BACKSTOP ONLY -- how long a scan may stay open with the unit saying
    # nothing at all.
    #
    # A scan normally ends because the UNIT ends it: it narrates a successful
    # read, or it narrates the end of its cycle. Neither of those is on a clock,
    # and no number here is ever reached on a working bus. This exists solely so
    # a unit that goes silent mid-cycle -- or a bridge that loses the narration
    # channel -- cannot leave a bay waiting forever.
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
        # How far the extruder bites BEFORE the follower is armed, so the gears
        # have hold of the filament without being mid-advance when the AMS runs
        # its own pull-back. Enough to grip, not enough to matter if the unit
        # tugs against it. 0 disables the split and advances tool_stn in one go.
        self.tool_bite_mm = config.getfloat("tool_bite_mm", 0.0, minval=0.0)
        # THE PULL SETTLE IS ITS OWN THING, not part of the bite.
        #
        # It answers a different question. The bite asks "do the gears have hold
        # of the filament?"; this asks "is anything else moving while the AMS
        # runs its pull?" They sit next to each other in the sequence and are
        # otherwise unrelated -- either can be useful without the other, and
        # they need to be separable to be tested separately.
        #
        # The pull is NATIVE: it fires on the mode change into mode:4 and is in
        # every working load ("pull sucess,mode change,mode:4 ... 0.052m",
        # taking 0.5-2.2s). This is time to LET IT FINISH, not a workaround.
        #
        # Named for the PULL, not the bite: it bounds how long the unit is
        # given to finish pulling, and is not a second parameter of the bite.
        # THE CEILING, NOT THE WAIT. _wait_for_pull returns the moment the unit
        # reports "pull sucess,mode change,mode:4", so this only bounds a unit
        # that never says it. Generous on purpose: at 2.0 it expired
        # microseconds BEFORE the event arrived --
        #
        #   07:26:55  ack assist
        #   07:26:57  pull sucess,mode change,mode:4
        #   07:26:57  no pull reported within 2.00s; advancing on the ceiling
        #
        # -- because the pull ends 2.02s after the assist and the ceiling was
        # 2.00s. Tuning a ceiling to the thing it is bounding is how it loses
        # that race; making it generous costs nothing when the event lands.
        # THE FEEDER HAS A TRANSMISSION, AND REVERSING IT TAKES TIME.
        #
        # "pull sucess" is the unit's COMPLETION REPORT FOR THE PULL. What
        # follows is not instant: the feeder's transmission has to change back
        # from reverse to forward -- about a second on the operator's ear --
        # and only then does the push-forward happen. Advance into that window
        # and the extruder is pulling while the drive is still swapping, which
        # is the "it didn't stay engaged" symptom.
        #
        # Runtime-tunable via AFC_BAMBU_SETTLE DWELL=, because the right value is a
        # property of the mechanism and has to be found on the machine, not
        # reasoned out. ("assist finish" was tried as an end-of-push signal and
        # is not one -- it fires BEFORE the arrival.)
        # Send the mode-09 select at the arrival? The printer does not, and it
        # is what turns the unit's arrival into a commanded switch cycle -- and
        # a switch cycle starts by pulling the tray back. ON by default because
        # it is present in every load of ours that works; AFC_BAMBU_ARRIVAL
        # SELECT=0 is the experiment.
        self.arrival_select = config.getboolean("arrival_select", False)
        # THE PRINTER WAITS BEFORE IT ARMS THE HOLD, and the gap is not small:
        #
        #   t+0.00  feed finish 0, mode:4 / [AMS_PMSM]mode:2->0 / state:4
        #   t+4.10  en:1,mode:3,idx:2,ref:0
        #   t+5.16  en:1,mode:3,idx:2,ref:127      the hold
        #
        # Four seconds of silence after the unit self-completes, then another
        # before the hold. We arm the assist IMMEDIATELY. If the feeder's
        # transmission needs time to come out of reverse -- which is what the
        # operator hears -- that gap is where the printer gives it.
        #
        # Default 0.0 so nothing changes until it is tested; AFC_BAMBU_ARRIVAL
        # ASSIST=4 is the experiment.
        self.arrival_assist_delay_s = config.getfloat(
            "arrival_assist_delay_s", 4.0, minval=0.0)
        self.pull_push_dwell_s = config.getfloat(
            "pull_push_dwell_s", 0.0, minval=0.0)
        self.pull_settle_s = config.getfloat("pull_settle_s", 6.0, minval=0.0)
        # Let the AMS's OWN arrival report finish a load when the toolhead
        # sensor does not. The sensor stays FIRST -- it is checked on every
        # pass of the feed loop and this is only consulted after -- so on a
        # sensored, calibrated lane it never fires (measured: the sensor
        # triggers 1-2 s ahead on both a boxed AMS and an HT). What it buys is
        # a lane with NO toolhead sensor being loadable at all, and a failed
        # sensor degrading to a correct completion instead of a silent
        # timeout followed by re-homing.
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
        # Create a Spoolman spool from a Bambu tag whose UID Spoolman does not
        # yet know. Off by default (binding to an existing spool by UID always
        # happens; only CREATION is gated). Per-lane auto_spoolman_create still
        # overrides via get_auto_spoolman_create, matching the ACE2/U1 readers.
        self.auto_spoolman_create = config.getboolean(
            "auto_spoolman_create", False)
        # Write the AMS's PHYSICAL remaining measurement (P:NN% by radius) back
        # to a bound Spoolman spool's remaining_weight. On by default: it is a
        # real reading and the whole point of measuring. Off for anyone who does
        # not want the AMS correcting Spoolman.
        self.sync_measured_to_spoolman = config.getboolean(
            "sync_measured_to_spoolman", True)
        # Hold a following AMS HT with the dense statu-0F poll (capture-faithful,
        # verified on hardware) rather than the ht_poll_seq re-poke. On by
        # default; the re-arm fallback is AFC_BAMBU_HT0FHOLD ON=0 at runtime.
        self.ht_0f_hold = config.getboolean("ht_0f_hold", True)
        #   measure_on_insert : run the spool CAPACITY measurement when a spool
        # is inserted, not just the tag read. On by default, which is what a
        # real printer does when its "calculate remaining capacity" setting is
        # ticked -- captured on the wire as a single flag in the routine poll
        # (op-04 body byte 4, 02 disabled -> 03 enabled), so this is our
        # equivalent of that checkbox rather than an invention.
        #
        # Why anyone would turn it off. The measurement is not free: the unit
        # physically pulls filament to derive the spool radius, and it takes
        # 8-25 s depending on generation (an AMS 1 was captured pulling 879 mm
        # hunting for the tag). On a bay you reload often, or where the tag's
        # own remain% is trusted, that is motion and time for a number you
        # already have. Turning it off leaves the TAG READ intact -- material,
        # colour, SKU and the tag's remain% still land; only the physical
        # measurement is skipped.
        #
        # Per unit, because the tradeoff is per unit: an HT holding a 1 kg
        # spool you rarely change is worth measuring, a 4-bay box you swap
        # constantly may not be. AFC_BAMBU_CAPSCAN still measures on demand
        # regardless of this setting -- it is the automatic behaviour that is
        # gated here, never the explicit command.
        #
        # It only ever applies to a BAMBU-TAGGED spool. The unit's own firmware
        # will not measure a spool it did not read a Bambu tag on, whatever we
        # send it, so on an untagged or third-party reel there is nothing for
        # this setting to turn off.
        self.measure_on_insert = config.getboolean("measure_on_insert", True)
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
        #   auto_error_recovery : on a stall mid-print, run the printer's own
        #     recovery -- AFC's lane unload (which cuts, retracts and unloads)
        #     then a reload of the same spool, then RESUME. OFF by default: a
        #     recovery that moves the toolhead and the filament unasked is not
        #     something to switch on for everyone silently. One attempt per
        #     fault; the AMS is already retrying inside its own window and a
        #     second machine retrying on top of it fights it.
        self.auto_error_recovery = config.getboolean(
            "auto_error_recovery", False)
        #   bus_serial : the 15-character serial the bridge announces as. The
        #     op-05 announce carries a printer serial and the units answer it;
        #     ours shipped as a real serial lifted from a capture, which was
        #     right for bring-up (proven to draw replies) and wrong to ship --
        #     it is one machine's identifier baked into everyone's firmware.
        #     Any 15 chars; shorter is padded. UNKNOWN whether a unit validates
        #     it, so if the units stop answering the announce after changing
        #     this, that IS the answer -- put the working value back and say so.
        self.bus_serial = config.get("bus_serial", "").strip()
        self._auto_recover_armed = False
        # Set when a Bambu fault is what paused the print, cleared once the lane
        # is fed again. Our RESUME wrap reloads ONLY while this is set, so an
        # ordinary pause -- filament change, a look at the first layer, someone
        # pressing pause -- resumes exactly as it always did.
        self._resume_needs_reload = False
        # Set by any status frame carrying byte[19] == 0x07, cleared when a
        # fault is armed. A LATCH because the signal is intermittent -- see
        # _on_status for the per-phase counts that forced this shape.
        self._declared_since_fault = False
        # True while auto recovery's own unload+reload is running. The recovery
        # drives a LOAD, and unit_load_lane clears _auto_recover_armed on every
        # load -- so without this the attempt resets its own one-shot guard and
        # the next fault arms another. Measured: fault at 12:40:49, reload OK at
        # 12:45:03, unit declared 12:45:14, SECOND recovery armed the same
        # second. Four and a half minutes of it, which is what "one attempt"
        # was supposed to prevent.
        self._in_auto_recover = False
        # Odometer range seen since the fault was armed, in mm. Answers WHERE a
        # jam is (see _jam_location); never used during a normal load.
        self._odom_lo: Optional[float] = None
        self._odom_hi: Optional[float] = None
        # A SECOND, INDEPENDENT range for the load. Sharing the pair above
        # would let _raise_ams_fault -- which resets it, deliberately, so the
        # range always means "during THIS fault's recovery" -- wipe the load's
        # evidence if a fault happened to be raised partway through. Two
        # questions, two ranges.
        self._load_odom_lo: Optional[float] = None
        self._load_odom_hi: Optional[float] = None
        #   rollcall_span_boxed / rollcall_span_ht : how many ids of each class
        #     the roll-call walks. Unset = derived from the configured units
        #     plus one spare per class, so hot-plugging the NEXT unit still
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
        # No starvation watchdog: inferring a stall from the buffer bottoming
        # out while the extruder pulls adds nothing, because byte[19] of the
        # op-04 reply reports the state directly on every unit -- see the note
        # at _check_ams_fault.
        self._starved_since: float = 0.0
        self._starved_e: float = 0.0
        self._starved_reads: Optional[int] = None
        # Last fault sequence handled, so one stall raises one error.
        self._fault_seen: int = 0
        # Auto-reset working state. _fault_floor_seen is the arming half: the
        # buffer must have actually been on the floor at some point under this
        # fault, or there is no "pressure came off" to detect and a fault raised
        # with a healthy buffer would self-clear on the spot.
        self._fault_lane: Any = None
        self._fault_floor_seen: bool = False
        self._fault_recover_since: float = 0.0
        self._fault_recover_reads: Optional[int] = None
        # Only the AMS2 Pro has a drying heater. Default true; set `heater: false`
        # in the config for AMS1 / AMS-lite units so AFC_BAMBU_HEATER_START just says
        # so instead of sending a drying command a heaterless unit ignores.
        # AMS TYPE -- one setting picks heater on/off, drying device address, and
        # temp ceiling (see _AMS_MODELS): `ams1` (regular AMS, no heater), `ams2`
        # (AMS2 Pro), `ht` (AMS HT). This is how you tell each unit apart so it
        # uses the right addressing. `heater:` and `dry_max_temp:` override the
        # type's defaults if you ever need to.
        self.ams_model = config.get("ams_model", "ams2").strip().lower()
        # NO PER-MODEL FOLLOWER BEHAVIOUR. Every unit is held the same way --
        # op-04 07/7F at 148 ms, the cadence a real printer uses -- with no
        # buffer deadband to tune. Measured: a regular AMS sits at 0.56-0.59 on
        # the virtual FPS, indistinguishable from an HT, so the "an AMS 2
        # refills its own buffer, a plain AMS must be fed on demand" rule was
        # never true.
        #
        # Nothing per-model is configurable here, and nothing is sent to the
        # firmware to select a follower style.
        _is_ht = self.ams_model in _HT_MODELS
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
        self._load_in_progress = False
        # The AMS's own words for the fault that ended the current load, kept
        # because _ams_declared_fault CONSUMES the sequence and the recovery and
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
        #: Has unit_uid been resolved to a real chain index yet? Until it
        #: has, this unit's ams_index is only the CONFIG DEFAULT (0), and
        #: announcing per-unit state at a guessed index registers this
        #: unit's flags/MC address against whichever unit really holds
        #: index 0 -- on a 3-unit bus that is the HT receiving two other
        #: units' registrations on every restart. A unit with no unit_uid
        #: configured has nothing to resolve: its config index IS the
        #: answer, so it counts as resolved from the start.
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
        # When >0 it rate-limits AND only logs when the values actually change, so
        # even enabled it never streams a line every tick.
        self.follow_debug_interval = config.getfloat(
            "follow_debug_interval", 0.0, minval=0.0)
        self._follow_last_dbg: Optional[tuple] = None
        # Last status-apply failure text. Status frames arrive continuously, so
        # a stuck fault would warn on every frame; only a CHANGED message is
        # worth the console.
        self._status_err_last: Optional[str] = None
        # Latched by AFC_BAMBU_FOLLOWER ENABLE=0. The latch is what holds the AMS
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
        # WHEN THIS SLOT'S SCAN WAS COMMANDED -- and the whole of the hold.
        # None = no scan open, use the bay's record as it comes. Set = we have
        # asked the unit a question and are waiting on its answer, so nothing
        # from this bay may reach the lane yet (it still reports the PREVIOUS
        # spool's record until the reader sees the new tag). _scan_verdict
        # turns this plus the unit's narration into the answer.
        self._scan_t0: List[Optional[float]] = [None] * self.SLOTS_PER_UNIT
        # Latch: the unit finished this slot's scan and read no tag. Stops the
        # defaults being re-applied on every status frame, and is cleared by a
        # removal or a new scan -- the only two things that can change the
        # answer.
        self._scan_notag: List[bool] = [False] * self.SLOTS_PER_UNIT
        # THE TAG UID EACH LANE'S SPOOLMAN BINDING WAS MADE FROM.
        #
        # Without it, "this lane is bound" and "this lane is bound to the spool
        # that is physically in it" are the same test, and they are not the same
        # fact. Measured: lane23 bound to spool 87 (PLA Glow, UID 0A1882AC), a
        # PLA Basic spool put in, tag 01D0EC0F read correctly -- and 810 g was
        # written to spool 87, because _spoolman_sync returns early on any bound
        # lane and never reconsiders. The tag is what identifies a spool, so the
        # binding has to follow it.
        #
        # A slot with NO entry here was bound by something other than a tag read
        # (a manual assignment, a restore from vars) and is left alone.
        self._bound_uid: Dict[int, str] = {}
        # Separate from _scan_t0: when the scan's physical motion began.
        # _scan_t0 is cleared on read success; this one is not.
        self._scan_motion_t0: List[Optional[float]] = (
            [None] * self.SLOTS_PER_UNIT)
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
        # User-facing commands, mux'd by UNIT= (matches AFC_ACE). AFC_BAMBU_FOLLOWER
        # lets you engage/stop the self-centering follower (mode:4) for a loaded
        # lane by hand -- both a manual workaround and a test hook to watch the
        # LED react to the exact select+assist sequence the load path uses.
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "AFC_BAMBU_FOLLOWER", "UNIT", self.name, self.cmd_AFC_BAMBU_FOLLOWER,
            desc="Engage/stop the AMS self-centering follower (mode:4) for a "
                 "loaded lane. AFC_BAMBU_FOLLOWER UNIT=<unit> LANE=<lane> [ENABLE=1]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_BITE", "UNIT", self.name, self.cmd_AFC_BAMBU_BITE,
            desc="Set the extruder BITE taken at the toolhead sensor before "
                 "the follower is armed, in mm (0 = none, advance tool_stn in "
                 "one go). AFC_BAMBU_BITE UNIT=<unit> [MM=<mm>]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_ARRIVAL", "UNIT", self.name, self.cmd_AFC_BAMBU_ARRIVAL,
            desc="Toggle the mode-09 select sent at the arrival. The printer "
                 "does not send it, and it is what turns our arrival into a "
                 "commanded switch cycle (which pulls the tray back). ASSIST= "
                 "delays arming the hold; the printer waits ~4s. "
                 "AFC_BAMBU_ARRIVAL UNIT=<unit> [SELECT=0|1] [ASSIST=<seconds>]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_SETTLE", "UNIT", self.name, self.cmd_AFC_BAMBU_SETTLE,
            desc="Stay off the filament while the AMS pulls. S= is the ceiling "
                 "on waiting for the pull; DWELL= is how long to then wait for "
                 "the feeder transmission to reverse and push forward. "
                 "AFC_BAMBU_SETTLE UNIT=<unit> [S=<seconds>] [DWELL=<seconds>]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_RECOVER", "UNIT", self.name, self.cmd_AFC_BAMBU_RECOVER,
            desc="Recover a stuck/failed load: relink the AMS, stop motion, reel "
                 "the lane's filament back to the bay, and reset its state. "
                 "AFC_BAMBU_RECOVER UNIT=<unit> LANE=<lane>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_RELINK", "UNIT", self.name, self.cmd_AFC_BAMBU_RELINK,
            desc="Force an AMS relink / error-recovery reset (deregister + "
                 "re-register) to clear a TIMEOUT/error state without a power "
                 "cycle. AFC_BAMBU_RELINK UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_SCAN", "UNIT", self.name, self.cmd_AFC_BAMBU_SCAN,
            desc="Trigger an RFID/tag scan on demand -- the same read the "
                 "auto-scan runs on a fresh insert. AFC_BAMBU_SCAN UNIT=<unit> "
                 "[LANE=<lane>] (no LANE = every slot on the unit). Use it on "
                 "the AMS HT, whose tag only reads when polled at 0x1800.")
        # AMS2 Pro / AMS HT heater drying (protocol from
        # docs/captures/ams2_drying.txt). Registered directly under the nice
        # names -- no cfg macro needed, same as AFC_BAMBU_FOLLOWER/RECOVER/RELINK
        # above. Do NOT also define a [gcode_macro Bambu_Heater_Start]: Klipper
        # upper-cases macro names, so it would register AFC_BAMBU_HEATER_START and
        # collide with this command ("already registered").
        self.gcode.register_mux_command(
            "AFC_BAMBU_MUTE", "UNIT", self.name, self.cmd_AFC_BAMBU_MUTE,
            desc="Suppress bridge transmitters to find what a unit reacts to. "
                 "AFC_BAMBU_MUTE UNIT=<unit> MASK=<bits> (0 = restore all)")
        self.gcode.register_mux_command(
            "AFC_BAMBU_ARMMS", "UNIT", self.name, self.cmd_AFC_BAMBU_ARMMS,
            desc="Set the 11/04 follower keep-alive cadence in ms (0 = "
                 "default). The one transmitter AFC_BAMBU_MUTE cannot silence. "
                 "AFC_BAMBU_ARMMS UNIT=<unit> MS=<ms>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_HTID", "UNIT", self.name, self.cmd_AFC_BAMBU_HTID,
            desc="Set the id used on an AMS HT's 0x1800 commands. "
                 "AFC_BAMBU_HTID UNIT=<unit> ID=<0-255>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_DRAIN", "UNIT", self.name, self.cmd_AFC_BAMBU_DRAIN,
            desc="Set the 1A/02 log-drain payload byte (255 = default). "
                 "AFC_BAMBU_DRAIN UNIT=<unit> P=<byte>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_HB", "UNIT", self.name, self.cmd_AFC_BAMBU_HB,
            desc="Set the bus heartbeat cadence in ms (0 = default). "
                 "AFC_BAMBU_HB UNIT=<unit> MS=<ms>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_POLLMS", "UNIT", self.name, self.cmd_AFC_BAMBU_POLLMS,
            desc="Set the status-poll cadence in ms (0 = default 300; a real "
                 "printer uses 11). AFC_BAMBU_POLLMS UNIT=<unit> MS=<ms>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_AUTOSCAN", "UNIT", self.name, self.cmd_AFC_BAMBU_AUTOSCAN,
            desc="Turn this unit's insert-edge tag scan on/off at runtime. "
                 "AFC_BAMBU_AUTOSCAN UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_POLL37", "UNIT", self.name, self.cmd_AFC_BAMBU_POLL37,
            desc="Send this unit's 0x37 tray poll and print the raw reply "
                 "(diagnostic). AFC_BAMBU_POLL37 UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_EXTMIMIC", "UNIT", self.name, self.cmd_AFC_BAMBU_EXTMIMIC,
            desc="Emulate the extruder's side of the AP2 sync (off by default). "
                 "AFC_BAMBU_EXTMIMIC UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_HT0FHOLD", "UNIT", self.name, self.cmd_AFC_BAMBU_HT0FHOLD,
            desc="Hold a following HT with the dense 0F poll (capture-faithful) "
                 "instead of dense ht_poll_seq. AFC_BAMBU_HT0FHOLD UNIT=<u> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_POLL0F", "UNIT", self.name, self.cmd_AFC_BAMBU_POLL0F,
            desc="Run the statu-0x0F loaded-state poll while following (the "
                 "real printer's assist keep-alive; needs MOTION6). "
                 "AFC_BAMBU_POLL0F UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_MOTION6", "UNIT", self.name, self.cmd_AFC_BAMBU_MOTION6,
            desc="Send the 6-byte op-0x03 motion body a real bus carries "
                 "instead of our 5-byte one. AFC_BAMBU_MOTION6 UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_UNIT80", "UNIT", self.name, self.cmd_AFC_BAMBU_UNIT80,
            desc="Address HT units as 0x80 in short frames, the way a real "
                 "printer does. AFC_BAMBU_UNIT80 UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_SENDRAW", "UNIT", self.name, self.cmd_AFC_BAMBU_SENDRAW,
            desc="Send raw frame hex on the bus and print the reply hex "
                 "(diagnostic). AFC_BAMBU_SENDRAW UNIT=<unit> HEX=<hex> [US=30000]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_LATEST", "UNIT", self.name, self.cmd_AFC_BAMBU_LATEST,
            desc="Print the bridge's latest status frame as JSON (diagnostic). "
                 "AFC_BAMBU_LATEST UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_MDIAG", "UNIT", self.name, self.cmd_AFC_BAMBU_MDIAG,
            desc="Print the firmware's short-motion poll counters (diagnostic). "
                 "AFC_BAMBU_MDIAG UNIT=<unit>")
        # AFC_BAMBU_DRIVE, not AFC_BAMBU_POLL0F. Klipper's parser splits a command name
        # at a digit followed by a letter, so "AFC_BAMBU_POLL0F UNIT=x" dispatches
        # as "AFC_BAMBU_POLL0" and dies with Unknown command -- the toggle has been
        # registered-but-unreachable since it was added, which is why the dense
        # drive poll could never actually be switched on from g-code. Any name
        # with a digit before a letter has this problem; keep them apart.
        self.gcode.register_mux_command(
            "AFC_BAMBU_DRIVE", "UNIT", self.name, self.cmd_AFC_BAMBU_POLL0F,
            desc="The 21ms op-03 drive channel while a tray is loaded (what a "
                 "real printer streams). AFC_BAMBU_DRIVE UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_TAIL", "UNIT", self.name, self.cmd_AFC_BAMBU_TAIL,
            desc="Derive the frame tail byte from the mode instead of the "
                 "hardcoded 0x02 every working load has used. OFF by default "
                 "-- it made loads worse. AFC_BAMBU_TAIL UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_TXECHO", "UNIT", self.name, self.cmd_AFC_BAMBU_TXECHO,
            desc="Record the frames WE transmit to the narration log, so a "
                 "load can be diffed against a printer capture. "
                 "AFC_BAMBU_TXECHO UNIT=<unit> ON=<0|1>")
        # No AFC_BAMBU_RESUME. The reload-on-resume lives on the ORDINARY resume
        # button now (see _arm_resume_wrap): a recovery command the operator
        # has to remember, at the one moment the machine is already in trouble,
        # is a recovery path nobody takes.
        self.gcode.register_mux_command(
            "AFC_BAMBU_CLEARFAULT", "UNIT", self.name, self.cmd_AFC_BAMBU_CLEARFAULT,
            desc="Stream the printer's 0E clear at a parked unit for ~2s, then "
                 "report whether it actually left its fault. An ATTEMPT -- it "
                 "fails if the jam is still there. AFC_BAMBU_CLEARFAULT UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_CLSPROBE", "UNIT", self.name, self.cmd_AFC_BAMBU_CLSPROBE,
            desc="Ask the unit at a bus id whether it answers on 0x1800 (HT) "
                 "or not (boxed). Standalone verification for bus-based class "
                 "detection. AFC_BAMBU_CLSPROBE UNIT=<unit> ID=<busid> [TRIES=1]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_CLASSADDR", "UNIT", self.name, self.cmd_AFC_BAMBU_CLASSADDR,
            desc="Enroll and address by CLASS the way a real printer does: "
                 "boxed 0x00-0x03, HT 0x80-0x87. Verify with AFC_BAMBU_RC. "
                 "AFC_BAMBU_CLASSADDR UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_ROLLCALL", "UNIT", self.name, self.cmd_AFC_BAMBU_ROLLCALL,
            desc="Run the printer's address register: probe all 12 bus ids "
                 "round-robin, one every 92ms. Adds bus traffic -- see "
                 "AFC_BAMBU_RC. AFC_BAMBU_ROLLCALL UNIT=<unit> ON=<0|1>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_RC", "UNIT", self.name, self.cmd_AFC_BAMBU_RC,
            desc="Print the roll-call counters and present-mask (diagnostic). "
                 "AFC_BAMBU_RC UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_SLOTTRACE", "UNIT", self.name, self.cmd_AFC_BAMBU_SLOTTRACE,
            desc="Record both sides of the slot-info traffic for a few "
                 "seconds -- what we believe about each bay, and the "
                 "firmware's read counters, before and after. "
                 "AFC_BAMBU_SLOTTRACE UNIT=<unit> [S=<seconds>]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_RDINFO", "UNIT", self.name, self.cmd_AFC_BAMBU_RDINFO,
            desc="Print the RAW 0x0211 filament-info reply for one bay, before "
                 "any decode. The only way to tell 'the unit did not send this "
                 "field' from 'our decode missed it'. "
                 "AFC_BAMBU_RDINFO UNIT=<unit> LANE=<lane>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_REID", "UNIT", self.name, self.cmd_AFC_BAMBU_REID,
            desc="Send the printer menu's 're-identify' to one bay and nothing "
                 "else. On a boxed AMS this is the SECOND tag detection, "
                 "without which the unit cannot measure the spool. "
                 "AFC_BAMBU_REID UNIT=<unit> LANE=<lane>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_CAPSCAN", "UNIT", self.name, self.cmd_AFC_BAMBU_CAPSCAN,
            desc="Run the printer's capacity-measuring re-scan on a bay: the "
                 "AMS re-reads the tag AND measures spool remain%. "
                 "AFC_BAMBU_CAPSCAN UNIT=<unit> LANE=<lane>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_HTPOLL", "UNIT", self.name, self.cmd_AFC_BAMBU_HTPOLL,
            desc="Set the 0x1800 keep-alive cadence in ms (0 = default). "
                 "AFC_BAMBU_HTPOLL UNIT=<unit> MS=<ms>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_FEED", "UNIT", self.name, self.cmd_AFC_BAMBU_FEED,
            desc="Feed a bounded length from a lane's slot. "
                 "AFC_BAMBU_FEED UNIT=<unit> LANE=<lane> [MM=20] [SPEED=]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_BUFFER_PROBE", "UNIT", self.name, self.cmd_AFC_BAMBU_BUFFER_PROBE,
            desc="Dump the raw AMS motion reply + buffer decode state. "
                 "AFC_BAMBU_BUFFER_PROBE UNIT=<unit>")
        self.gcode.register_mux_command(
            "AFC_BAMBU_HEATER_START", "UNIT", self.name, self.cmd_AFC_BAMBU_HEATER_START,
            desc="Start AMS drying (AMS2 Pro / AMS HT). "
                 "AFC_BAMBU_HEATER_START UNIT=<unit> [TEMP=55] [TIME=480] [ROTATE=0]")
        self.gcode.register_mux_command(
            "AFC_BAMBU_HEATER_STOP", "UNIT", self.name, self.cmd_AFC_BAMBU_HEATER_STOP,
            desc="Stop AMS drying. AFC_BAMBU_HEATER_STOP UNIT=<unit>")
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

    def cmd_AFC_BAMBU_FOLLOWER(self, gcmd: Any) -> None:
        """
        Manually engage or stop the follower for a lane's AMS tray.

        AFC_BAMBU_FOLLOWER UNIT=<unit> LANE=<lane> [ENABLE=1]

        ENABLE=1 (default) runs the finish->select->assist sequence that flips
        the tray to mode:4 and holds it (LED should start flashing); ENABLE=0
        stops the follower (LED goes solid). Use it to verify the follower on a
        tool-loaded lane independent of the load/startup paths.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_FOLLOWER UNIT=<unit> LANE=<lane> ENABLE=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_FOLLOWER UNIT=BambuAMS_1 LANE=lane1 ENABLE=1
        ```
        """
        lane_name = gcmd.get("LANE")
        enable = gcmd.get_int("ENABLE", 1)
        lane = self.lanes.get(lane_name)
        if lane is None:
            raise gcmd.error(
                f"AFC_BAMBU_FOLLOWER: lane '{lane_name}' not on unit {self.name} "
                f"(lanes: {', '.join(self.lanes) or 'none'})")
        if self._bridge is None:
            msg = f"AFC_BAMBU_FOLLOWER: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if self._slot_of(lane) is None:
            raise gcmd.error(
                f"AFC_BAMBU_FOLLOWER: {lane_name} is not mapped to an AMS slot")
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
                f"AFC_BAMBU_FOLLOWER ENABLE=1 or the next load.")

    def cmd_AFC_BAMBU_BITE(self, gcmd: Any) -> None:
        """
        Set the extruder bite taken at the toolhead sensor, at runtime.

        AFC_BAMBU_BITE UNIT=<unit> [MM=<mm>]

        With no arguments, reports the current value.

        THE BITE is the small advance the extruder takes the moment the sensor
        reads filament, BEFORE the follower is armed, so the gears have hold
        while the AMS runs its own pull-and-push. The remainder of tool_stn is
        fed afterwards. See docs/THE_LOAD.md step 7.

        The wait that follows is a SEPARATE mechanism with its own command --
        AFC_BAMBU_SETTLE. They sit next to each other in the sequence and answer
        different questions ("do the gears have hold?" vs "is anything else
        moving while the unit pulls?"), so they are controlled separately and
        can be tested separately.

        MM=0 disables the split entirely and advances tool_stn in one go, which
        is what the load did before the bite existed -- and is the A side of the
        A/B test this command is for. It is a runtime knob so that test costs a
        g-code line rather than a config edit and a restart.

        The value is NOT written to config: a restart returns it to the
        configured tool_bite_mm. That is deliberate for an experiment knob --
        nothing you set here can quietly become the machine's permanent
        behaviour.

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
        Toggle the mode-09 select we send at the arrival.

        AFC_BAMBU_ARRIVAL UNIT=<unit> [SELECT=0|1] [ASSIST=<seconds>]

        THE PRINTER DOES NOT SEND IT. Its AMS reaches mode:4 as the natural end
        of its own feed and its good load contains no "pull sucess" at all.
        Ours is commanded into mode:4 by this select, and the unit's own words
        for what follows are "pull sucess,MODE CHANGE,mode:4" -- a commanded
        switch cycle begins by pulling the tray back.

        SELECT=0 is the printer's arrival: stop, then let the unit finish into
        mode:4 by itself. If the pull disappears, the bite/settle/dwell
        apparatus built around it is unnecessary.

        Runtime only; a restart restores the configured arrival_select.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_ARRIVAL UNIT=<unit> SELECT=<0 or 1> ASSIST=<value>`

        Example
        -------
        ```
        AFC_BAMBU_ARRIVAL UNIT=BambuAMS_1 SELECT=1 ASSIST=1.0
        ```
        """
        sel = gcmd.get_int("SELECT", None, minval=0, maxval=1)
        delay = gcmd.get_float("ASSIST", None, minval=0.0, maxval=30.0)
        if sel is not None:
            self.arrival_select = bool(sel)
        if delay is not None:
            self.arrival_assist_delay_s = delay
        state = "ON (ours)" if self.arrival_select else "OFF (the printer's)"
        note = "" if sel is None and delay is None else "  (runtime only)"
        gcmd.respond_info(
            f"AFC_BAMBU_ARRIVAL {self.name}: select={state}  "
            f"assist-delay={self.arrival_assist_delay_s:.2f}s "
            f"(the printer waits ~4s){note}")

    def cmd_AFC_BAMBU_SETTLE(self, gcmd: Any) -> None:
        """
        Set the PULL SETTLE -- how long we stay off the filament while the AMS
        runs its own pull, at runtime.

        AFC_BAMBU_SETTLE UNIT=<unit> [S=<seconds>] [DWELL=<seconds>]

        With no arguments, reports the current value.

        The AMS pulls the tray back on the mode change into mode:4. That is
        NATIVE -- it is in every working load, taking 0.5-2.2s -- and this is
        the window in which nothing of ours moves while it happens. S=0 removes
        the wait, so the tool_stn advance runs straight through the pull, which
        is what it did before this existed.

        Separate from AFC_BAMBU_BITE on purpose: the bite is about the gears having
        hold, this is about staying out of the way. Either is useful without the
        other, and a single knob with two numbers is how they kept getting
        reasoned about as one thing.

        Runtime only; a restart restores the configured pull_settle_s.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_SETTLE UNIT=<unit> S=<value> DWELL=<value>`

        Example
        -------
        ```
        AFC_BAMBU_SETTLE UNIT=BambuAMS_1 S=1.0 DWELL=1.0
        ```
        """
        secs = gcmd.get_float("S", None, minval=0.0, maxval=30.0)
        dwell = gcmd.get_float("DWELL", None, minval=0.0, maxval=30.0)
        if secs is not None:
            self.pull_settle_s = secs
        if dwell is not None:
            self.pull_push_dwell_s = dwell
        gcmd.respond_info(
            f"AFC_BAMBU_SETTLE {self.name}: ceiling="
            f"{'OFF' if self.pull_settle_s <= 0 else f'{self.pull_settle_s:.2f}s'}"
            f"  dwell-after-pull={self.pull_push_dwell_s:.2f}s"
            f"{'' if secs is None and dwell is None else '  (runtime only -- a restart restores the config values)'}")

    def cmd_AFC_BAMBU_RECOVER(self, gcmd: Any) -> None:
        """
        Recover a stuck / failed load: stop motion, reel the lane's filament
        back to the bay, and reset its state so AFC is no longer mid-operation.

        AFC_BAMBU_RECOVER UNIT=<unit> LANE=<lane>

        Use after a load errors out (e.g. a feeder "rocker stall"): the AMS is
        left idle but with filament staged partway in the path and the lane
        stuck in a load/error state. This halts the AMS, winds the filament back
        into the bay (the shared eject reel-back), and clears the lane so you can
        re-insert / retry. If the feeder still stalls after this, the bay's
        filament tip is jammed -- open the AMS, trim the tip, and reinsert.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_RECOVER UNIT=<unit> LANE=<lane>`

        Example
        -------
        ```
        AFC_BAMBU_RECOVER UNIT=BambuAMS_1 LANE=lane1
        ```
        """
        lane_name = gcmd.get("LANE")
        lane = self.lanes.get(lane_name)
        if lane is None:
            raise gcmd.error(
                f"AFC_BAMBU_RECOVER: lane '{lane_name}' not on unit {self.name} "
                f"(lanes: {', '.join(self.lanes) or 'none'})")
        if self._bridge is None:
            raise gcmd.error(
                f"AFC_BAMBU_RECOVER: bridge not connected for {self.name}")
        if self._slot_of(lane) is None:
            raise gcmd.error(
                f"AFC_BAMBU_RECOVER: {lane_name} is not mapped to an AMS slot")
        gcmd.respond_info(
            f"AFC_BAMBU_RECOVER: stopping and reeling {lane_name} back to the bay...")
        self._recover_to_bay(lane)
        gcmd.respond_info(
            f"AFC_BAMBU_RECOVER: {lane_name} reset. If a load still stalls the "
            f"feeder, the bay filament tip is jammed -- open the AMS, trim the "
            f"tip, and reinsert.")

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

        AFC_BAMBU_FEED UNIT=<unit> LANE=<lane> [MM=20] [SPEED=<mm/s>]

        The same primitive the load path uses, which is known to move filament.
        Exposed on its own so the feed command can be tested independently of
        the follower -- the follower can sit armed in mode:4 and never drive the
        motor, and without this there is no way to tell an AMS that will not
        feed from one that was never asked to.

        Also the only way to relieve a bottomed-out buffer from software:
        feeding separates the two PTFE ends and compresses the spring.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_FEED UNIT=<unit> LANE=<lane> MM=<mm> SPEED=<value>`

        Example
        -------
        ```
        AFC_BAMBU_FEED UNIT=BambuAMS_1 LANE=lane1 MM=20.0 SPEED=1.0
        ```
        """
        lane_name = gcmd.get("LANE")
        mm = gcmd.get_float("MM", 20.0, above=0.0, maxval=200.0)
        speed = gcmd.get_float("SPEED", 0.0, minval=0.0)
        lane = self.lanes.get(lane_name)
        if lane is None:
            msg = (f"AFC_BAMBU_FEED: lane '{lane_name}' not on unit {self.name} "
                   f"(lanes: {', '.join(self.lanes) or 'none'})")
            raise gcmd.error(msg)
        if self._bridge is None:
            msg = f"AFC_BAMBU_FEED: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        if self._slot_of(lane) is None:
            msg = f"AFC_BAMBU_FEED: {lane_name} is not mapped to an AMS slot"
            raise gcmd.error(msg)
        ok = self.feed(lane, mm, speed if speed > 0 else None)
        gcmd.respond_info(
            f"AFC_BAMBU_FEED: {'issued' if ok else 'FAILED to issue'} {mm:.0f}mm on "
            f"{lane_name} ({self.name}).")

    def cmd_AFC_BAMBU_MUTE(self, gcmd: Any) -> None:
        """
        Suppress individual bridge transmitters, to find which one a unit
        reacts to audibly.

        AFC_BAMBU_MUTE UNIT=<unit> MASK=<bits>

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

        Usage
        -------
        `AFC_BAMBU_MUTE UNIT=<unit> MASK=<n>`

        Example
        -------
        ```
        AFC_BAMBU_MUTE UNIT=BambuAMS_1 MASK=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_MUTE: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        mask = gcmd.get_int("MASK", 0, minval=0, maxval=1023)
        self._bridge.send({"cmd": "mute", "mask": mask})
        names = ("MC_ONLINE", None, "MC_023C", "MC_3702", "heartbeat",
                 "HT_poll", "AP2_sync", "L2C_poke", "presence", "online_detect")
        muted = [n for i, n in enumerate(names) if n and mask & (1 << i)]
        gcmd.respond_info(
            f"AFC_BAMBU_MUTE: mask={mask} "
            f"muted={', '.join(muted) if muted else 'nothing (all restored)'}")

    def cmd_AFC_BAMBU_HTPOLL(self, gcmd: Any) -> None:
        """
        Set the 0x1800 keep-alive cadence at runtime, in ms. 0 restores the
        firmware default.

        AFC_BAMBU_HTPOLL UNIT=<unit> MS=<ms>

        For finding this unit's actual limit by sweeping, instead of one reflash
        per value. AFC_BAMBU_MUTE cannot answer this: muting a poll stops the unit's
        liveness being refreshed, it reads offline, and offline gates OTHER
        transmitters -- so a mute proves only that something downstream of
        "online" stopped, not which frame was responsible.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_HTPOLL UNIT=<unit> MS=<n>`

        Example
        -------
        ```
        AFC_BAMBU_HTPOLL UNIT=BambuAMS_1 MS=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_HTPOLL: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        ms = gcmd.get_int("MS", 0, minval=0, maxval=5000)
        self._bridge.send({"cmd": "htpoll", "ms": ms})
        gcmd.respond_info(
            f"AFC_BAMBU_HTPOLL: 0x1800 keep-alive "
            f"{'default (150ms)' if ms == 0 else str(ms) + 'ms'}")

    def cmd_AFC_BAMBU_DRAIN(self, gcmd: Any) -> None:
        """
        Set the payload byte of the 1A/02 log drain. 255 restores the default.

        AFC_BAMBU_DRAIN UNIT=<unit> P=<byte>

        The byte is per-model and there is no way to derive it: an AMS 2 Pro
        answers P=1 every time and ignores P=0 completely, which is why 2000+
        silent exchanges were once misread as "this unit cannot narrate". A
        regular AMS answers neither, so its value has to be found by sweeping.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_DRAIN UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_DRAIN UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_DRAIN: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        p = _gcmd_int(gcmd, "P", 255, 0, 255)
        # ADDR retargets the drain. The captured frame asks 0x0700 -- the AMS 2
        # Pro -- so on an HT-only bus it has never drawn a reply (snap counters
        # flat at 0 against 500+ empty exchanges) even while narration arrived
        # via other polls. An HT answers at 0x1800, and its own frames carry
        # payload 0x00 -- AFC_BAMBU_DRAIN ADDR=0x1800 P=0 is the pair that
        # answers. NOT 0x80, which the capture was read as and which
        # draws no reply at all; swept on hardware, see _MC_ADDRESSING.
        # 0 keeps the captured address.
        addr = _gcmd_int(gcmd, "ADDR", 0, 0, 0xFFFF)
        self._bridge.send({"cmd": "drain", "p": p, "addr": addr})
        gcmd.respond_info(
            f"AFC_BAMBU_DRAIN: log-drain payload "
            f"{'default (0x01/0x00)' if p == 255 else hex(p)}"
            f", device {'captured (0x0700)' if not addr else hex(addr)}")

    def cmd_AFC_BAMBU_HB(self, gcmd: Any) -> None:
        """
        Set the bus heartbeat cadence at runtime, in ms. 0 restores the default.

        AFC_BAMBU_HB UNIT=<unit> MS=<ms>

        Keep it under ~1000: the AMS declares itself offline without a heartbeat
        for about a second, and an offline unit gates other transmitters, which
        makes any measurement taken there uninterpretable.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_HB UNIT=<unit> MS=<n>`

        Example
        -------
        ```
        AFC_BAMBU_HB UNIT=BambuAMS_1 MS=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_HB: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        ms = gcmd.get_int("MS", 0, minval=0, maxval=5000)
        self._bridge.send({"cmd": "hb", "ms": ms})
        gcmd.respond_info(
            f"AFC_BAMBU_HB: heartbeat "
            f"{'default (300ms)' if ms == 0 else str(ms) + 'ms'}")

    def cmd_AFC_BAMBU_POLLMS(self, gcmd: Any) -> None:
        """
        Set the status-poll cadence in ms at runtime. 0 restores the default.

        AFC_BAMBU_POLLMS UNIT=<unit> MS=<ms>

        A real printer sends the 1A/02 drain every 11 ms -- measured at 11 ms in
        two independent captures, 98% answered -- and ours goes out at 300. That
        is 27x slower, and this codebase already carries one finding of exactly
        that shape: at a 5000 ms heartbeat the unit stayed ONLINE at fstate 4 and
        a spool insert would still not physically scan, until 300 ms was put
        back. Cadence gates physical action here in a way no status field shows.

        Sweep it rather than guess: 100, 50, 25, 11. Judge it against an insert
        actually calibrating, not against bridge_online.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_POLLMS UNIT=<unit> MS=<n>`

        Example
        -------
        ```
        AFC_BAMBU_POLLMS UNIT=BambuAMS_1 MS=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_POLLMS: bridge not connected")
        ms = gcmd.get_int("MS", 0, minval=0, maxval=5000)
        self._bridge.send({"cmd": "poll", "ms": ms})
        gcmd.respond_info(
            f"AFC_BAMBU_POLLMS: status poll "
            f"{'default (300ms)' if ms == 0 else str(ms) + 'ms'}")

    def cmd_AFC_BAMBU_AUTOSCAN(self, gcmd: Any) -> None:
        """
        Turn this unit's insert-edge tag scan on or off at runtime.

        AFC_BAMBU_AUTOSCAN UNIT=<unit> ON=<0|1>

        Exists to test whether OUR scan is what stops a boxed AMS calibrating
        its odometer on insert. During a real printer's insert the printer sends
        NOTHING but its four polls -- 1A/02, 11/04, 3C/02, 37/02 -- and the AMS
        runs the whole sequence itself:

            STEP,first detected
            STEP:odom calib success exit 0,dis:0.773
            STEP:rfid pull 0 ... card auth ... read success ... finish,cali tray

        Ours sends a scan the moment it sees the insert edge, which drives
        select + set tray_readid, and the same unit then reports
        "STEP:odom invalid tray 0" and never calibrates. No "first detected"
        appears in our logs at all.

        With this off, an insert should be left entirely to the AMS. If
        "odom calib success ... dis:N" then appears, the scan command was the
        problem and dis is the measurement worth chasing across spool fills.

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

    def cmd_AFC_BAMBU_POLL37(self, gcmd: Any) -> None:
        """
        Send this unit's 0x37 tray poll and print the raw reply.

        AFC_BAMBU_POLL37 UNIT=<unit>

        The 40-byte answer is largely unmapped. Two offsets are worth watching:
        [34] sits in 24..34 and moves slowly, which is consistent with a chamber
        temperature in degrees -- and if it IS one, that contradicts the note on
        the 0x04 decode, which states that no temperature field exists in a
        frame and that chamber temperature is only ever available from the
        unit's [AMS_CHMB] text. That note was reached from the 0x04 reply alone;
        0x37 was never looked at.

        [36:38] is a 16-bit that lands in 39.3..51.8 when divided by ten, which
        reads exactly like %RH and is NOT the humidity we already decode -- it
        disagrees with byte 8 of the 0x04 reply by up to twelve points on frames
        captured seconds apart.

        Diagnostic only: this sends one poll frame, which is a frame the printer
        itself sends continuously, and reads the answer. It commands nothing.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_POLL37 UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_POLL37 UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_POLL37: bridge not connected")
        frame = _poll37_frame(self.dry_dev_addr, self.dry_ams_id)
        self._bridge.send({"cmd": "raw", "hex": frame, "us": 30000})
        try:
            self.afc.reactor.register_callback(
                self._report_poll37, self.afc.reactor.monotonic() + 0.5)
        except Exception:
            self._report_poll37(0)

    def _report_poll37(self, eventtime: float) -> None:
        """
        Decode and print whatever the 0x37 poll brought back.

        :param eventtime: reactor time supplied by the callback; unused
        """
        hexs = ""
        try:
            hexs = getattr(self._bridge, "_last_raw_reply", "") or ""
        except Exception:
            pass
        if not hexs:
            self.gcode.respond_info(f"AFC_BAMBU_POLL37: {self.name}: no reply")
            return
        try:
            b = bytes.fromhex(hexs)
        except ValueError:
            self.gcode.respond_info(f"AFC_BAMBU_POLL37: {self.name}: unparseable {hexs}")
            return
        # Report the CHAMBER temperature the unit narrates alongside it, so the
        # two can be compared as the dryer drives the chamber. That comparison
        # is the whole point: a byte that merely sits in a plausible range is
        # not a temperature, and over a monotonic heating window every
        # incrementing counter in a frame correlates above r=0.95.
        narrated = getattr(self._bridge, "_chmb_temp", None)
        b34 = b[34] if len(b) > 34 else None
        b36 = (b[36] | (b[37] << 8)) if len(b) > 37 else None
        self.gcode.respond_info(
            f"AFC_BAMBU_POLL37: {self.name} len={len(b)} [34]={b34} "
            f"[36:38]={b36} narrated_chamber={narrated}\n{hexs}")

    def cmd_AFC_BAMBU_LATEST(self, gcmd: Any) -> None:
        """
        Print the bridge's most recent status frame, whole and unedited.

        AFC_BAMBU_LATEST UNIT=<unit>

        Every status field this module publishes is a projection of this one
        dict, and each projection embeds a mapping choice -- a gate, a sentinel,
        an index. When a field reads None the question is always "did the
        bridge not say, or did the mapping drop it?", and this answers it in
        one command instead of a debug deploy. (The odometer sat published at
        -74 while odom_m read None for exactly such a mapping gate.)

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_LATEST UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_LATEST UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_LATEST: bridge not connected")
        latest = self._bridge.latest_status()
        if latest is None:
            gcmd.respond_info(f"AFC_BAMBU_LATEST: {self.name}: no status frame yet")
            return
        try:
            text = json.dumps(latest)
        except Exception:
            text = repr(latest)
        # Klipper truncates very long respond_info lines; split into chunks.
        for i in range(0, len(text), 900):
            gcmd.respond_info(f"AFC_BAMBU_LATEST: {text[i:i+900]}")

    def cmd_AFC_BAMBU_MDIAG(self, gcmd: Any) -> None:
        """
        Print the firmware's short-motion (op 0x03) poll counters.

        AFC_BAMBU_MDIAG UNIT=<unit>

        sent/got split "no reply seen" from "reply seen"; len/op say what the
        last reply looked like; dec/pass/fail say what the float gate did with
        it; bits is the last candidate float, raw. Built to answer one specific
        question -- why AFC_BAMBU_SENDRAW sees a 44-byte odometer reply that the
        status frame never reflects -- but general enough to keep.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_MDIAG UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_MDIAG UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_MDIAG: bridge not connected")
        self._bridge.send({"cmd": "m3"})
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.4)
        m3 = getattr(self._bridge, "_last_m3", None)
        if not m3:
            gcmd.respond_info(f"AFC_BAMBU_MDIAG: {self.name}: no m3 reply "
                              "(firmware older than 1.0.17.0?)")
            return
        gcmd.respond_info(f"AFC_BAMBU_MDIAG: {self.name}: {m3}")

    # The bus ids the firmware's register walks, in mask-bit order. Kept here
    # only to name the bits when printing -- the firmware owns the real list.
    ROLLCALL_IDS = (0x00, 0x01, 0x02, 0x03,
                    0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87)

    def cmd_AFC_BAMBU_ROLLCALL(self, gcmd: Any) -> None:
        """
        Run (or stop) the firmware's address register.

        AFC_BAMBU_ROLLCALL UNIT=<unit> ON=<0|1>

        A real printer probes every id in the address space -- 0x00-0x03 boxed,
        0x80-0x87 HT -- one every ~92ms, forever, whether or not anyone has ever
        answered, and sends work frames only to the ids that answered. This runs
        that register. It is bus-wide, not per-unit; UNIT only picks which
        bridge to speak through.

        Off at boot, and worth leaving off until it has been watched: nine of
        the twelve ids have nobody on them and each of those probes burns the
        full 8ms reply timeout, so the register can take up to ~8.7% of the bus.
        At idle that is nothing. Inside the 21ms drive window it is the same
        shape as the dense 0F poll that once starved the HT keep-alive and
        dropped the unit into its red fault.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_ROLLCALL UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_ROLLCALL UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_ROLLCALL: bridge not connected")
        on = gcmd.get_int("ON", minval=0, maxval=1)
        self._bridge.send({"cmd": "rollcall", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_ROLLCALL: {self.name}: register {'ON' if on else 'OFF'} "
            "(read it back with AFC_BAMBU_RC)")

    #: How long after klippy:ready we keep looking for AFC to finish renaming
    #: RESUME. PREP is g-code driven, so it runs whenever the user's ready
    #: macro gets to it -- seconds on a warm boot, longer behind a slow
    #: MOONRAKER connect. Generous, because giving up early means wrapping the
    #: WRONG handler (the printer's original) and silently cutting AFC's own
    #: error-state clearing out of the resume chain.
    RESUME_WRAP_WAIT_S = 120.0
    #: How often we re-check while waiting.
    RESUME_WRAP_POLL_S = 2.0

    def _arm_resume_wrap(self) -> None:
        """
        Take over RESUME once AFC has finished renaming it -- once per printer.

        A Bambu fault parks the unit and pauses the print with the toolhead
        empty, so a plain resume prints air. Wrapping the resume button itself
        puts the recovery where the operator already reaches.

        Registration is timer-driven because the ordering is fixed and not ours
        to choose: pause_resume registers RESUME, then AFC_prep._rename_macros
        renames it to _AFC_RENAMED_RESUME_ and puts afcError.cmd_AFC_RESUME on
        RESUME, at an unknown point after ready. Wrapping before that is
        silently overwritten, so this polls the g-code dispatch table until
        RESUME actually IS AFC's handler rather than trusting a delay or a
        progress flag.

        The resulting chain:

            RESUME (the button)
              -> ours: reload the lane if a Bambu fault emptied the toolhead
              -> _AFC_BAMBU_RENAMED_RESUME_ = AFC's cmd_AFC_RESUME
                 (clears error state, z-hop, restores position)
              -> _AFC_RENAMED_RESUME_ = the printer's original RESUME

        The reload runs FIRST, while the print is still paused and the toolhead
        parked, then hands off -- AFC restores position and continues the print
        at the end of the chain, by which point there is filament to print
        with. Rename-and-delegate is AFC's own pattern, applied a level up, and
        touches no shared file.
        """
        if _RESUME_WRAPPED:
            return                  # another unit on this printer owns it
        try:
            reactor = self.afc.reactor
        except Exception:
            return                  # no reactor (tests): nothing to arm
        deadline = reactor.monotonic() + self.RESUME_WRAP_WAIT_S

        def _tick(eventtime):
            global _RESUME_WRAPPED
            if _RESUME_WRAPPED:
                return reactor.NEVER
            # CHECK THE HANDLER, NOT THE FLAG.
            #
            # Do not poll AFC_prep.rename_occurred -- it is a RACE:
            #
            #     if not self.rename_occurred:
            #         self.rename_occurred = True            <- set FIRST
            #         self.afc.function._rename(RESUME...)   <- rename AFTER
            #
            # The flag is set BEFORE the rename it announces, so we could wrap
            # in the gap and PREP would then overwrite us. Observed live: the
            # log said "RESUME wrapped" at 12:52:43 and RESUME was AFC's
            # handler afterwards. It worked three times before that on timing
            # alone, which is the worst way for a race to behave.
            #
            # The real precondition is not "has prep run" but "is the handler
            # we are about to displace AFC's". Ask that directly.
            # Wait for the EFFECT of AFC's rename, via the public command
            # table: _AFC_RENAMED_RESUME_ exists if and only if the rename has
            # completed, because AFC creates it as part of doing it.
            #
            # Two earlier attempts got this wrong, both by asking something
            # adjacent instead of the thing itself:
            #
            #   1. AFC_prep.rename_occurred -- a flag AFC sets BEFORE the
            #      rename it announces, so we wrapped in the gap and PREP
            #      overwrote us. Survived three restarts on timing alone.
            #   2. gcode.ready_gcode_handlers['RESUME'] -- private attributes
            #      that this Klipper does not present the way assumed, so the
            #      gate never matched at all and timed out after 120 s.
            #
            # get_command_help() is what /printer/gcode/help serves, so it is
            # both public and exactly what an operator can check by hand.
            renamed = False
            try:
                names = self.gcode.get_command_help()
                renamed = self.afc.error.AFC_RENAME_RESUME_NAME in names
            except Exception:
                renamed = False
            if not renamed:
                if eventtime < deadline:
                    return eventtime + self.RESUME_WRAP_POLL_S
                # Say it and stop. Wrapping now would capture the printer's
                # original RESUME and drop AFC's error handling out of the
                # chain, which is worse than not wrapping at all.
                self.logger.warning(
                    f"AFC bambu {self.name}: AFC never renamed RESUME "
                    f"({self.afc.error.AFC_RENAME_RESUME_NAME} absent), "
                    f"so the reload-on-resume is NOT active. After a Bambu "
                    f"fault, load the lane by hand (CHANGE_TOOL LANE=<lane>) "
                    f"before resuming or the print continues with an empty "
                    f"toolhead.")
                return reactor.NEVER
            try:
                self._wrap_resume()
                _RESUME_WRAPPED = True
                self.logger.debug(
                    f"AFC bambu {self.name}: RESUME wrapped -- a resume after "
                    f"a Bambu fault reloads the lane first")
            except Exception as e:
                # Never leave RESUME broken. _wrap_resume puts the previous
                # handler back itself if the second registration fails; all we
                # can add here is the warning.
                self.logger.warning(
                    f"AFC bambu {self.name}: could not wrap RESUME ({e}); "
                    f"resume works as normal but will NOT reload the lane "
                    f"after a Bambu fault.")
            return reactor.NEVER

        try:
            reactor.register_timer(_tick, reactor.monotonic() + 1.0)
        except Exception:
            pass

    def _wrap_resume(self) -> None:
        """
        Swap RESUME for our handler, keeping the old one reachable.

        Same three lines AFC's own function._rename uses. Deliberately not a
        call INTO that helper: it is a method on AFC's function object and this
        is our command chain, so borrowing the mechanism rather than the method
        keeps the coupling to a pattern instead of an API.

        If the second registration throws, the first has already removed
        RESUME, and a printer with no RESUME cannot be recovered without a
        restart. So that case puts the previous handler straight back.
        """
        prev = self.gcode.register_command(self.BASE_RESUME_NAME, None)
        if prev is None:
            raise RuntimeError("no RESUME command registered to wrap")
        try:
            self.gcode.register_command(
                AFC_BAMBU_RENAMED_RESUME, prev,
                desc=f"Renamed builtin of '{self.BASE_RESUME_NAME}'")
            self.gcode.register_command(
                self.BASE_RESUME_NAME, self.cmd_AFC_BAMBU_WRAPPED_RESUME,
                desc=self.cmd_AFC_BAMBU_WRAPPED_RESUME_help)
        except Exception:
            try:
                self.gcode.register_command(self.BASE_RESUME_NAME, prev)
            except Exception:
                pass
            raise

    #: The name we take over. Read from AFC's own constant at wrap time when we
    #: can; this is the fallback and the documentation of what it is.
    BASE_RESUME_NAME = "RESUME"

    cmd_AFC_BAMBU_WRAPPED_RESUME_help = (
        "RESUME, with the lane reloaded first if a Bambu AMS fault emptied "
        "the toolhead")

    def cmd_AFC_BAMBU_WRAPPED_RESUME(self, gcmd: Any) -> None:
        """
        The resume button, with the lane reloaded first when a fault emptied it.

        What the printer itself does after a human presses continue, straight
        off the measured arc: op-04 03/00 drive 12.2 s, 09/A5 enter 5.1 s, then
        back to 07/7F hold. That is an ORDINARY LOAD -- the same phases as any
        other load, quick because the jam is gone by then. There is no special
        resume sequence on the bus, which is why this reloads through
        CHANGE_TOOL rather than driving frames itself.

        THREE WAYS OUT, and only one of them continues the print:

          * Nothing to fix (not paused, no Bambu fault pending, or the lane is
            already loaded) -> delegate untouched. This is the common path and
            an ordinary pause/resume must not notice we exist.
          * Reload succeeded -> delegate; the print continues with filament.
          * Reload did NOT take -> raise, and DO NOT delegate. The print stays
            paused. A recovery that cannot verify itself must not proceed;
            resuming here is exactly the bug that restarted a print into an
            empty toolhead once already.

        Anything unexpected inside our own bookkeeping is caught and logged,
        then we delegate anyway. A broken resume button is worse than a missed
        reload -- the operator can always load a lane by hand, but they cannot
        un-break RESUME without restarting Klipper.

        :param gcmd: The Klipper GCodeCommand
        """
        params = gcmd.get_raw_command_parameters()
        try:
            self._reload_before_resume(gcmd)
        except self.printer.command_error:
            raise                   # refused on purpose; message says why
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: reload-on-resume check failed ({e}); "
                f"resuming anyway. If the toolhead is empty, pause and load "
                f"the lane.")
        self.gcode.run_script_from_command(
            f"{AFC_BAMBU_RENAMED_RESUME} {params}")

    def _reload_before_resume(self, gcmd: Any) -> None:
        """
        Reload the faulted lane, or explain why the print must stay paused.

        Returns quietly whenever there is nothing to do -- that silence is the
        contract. This runs on EVERY resume on the printer, including ones that
        have nothing to do with an AMS, so it earns its place by being
        invisible unless a Bambu fault actually parked us.

        The lane comes from the fault itself (`_fault_lane`, recorded when the
        stall was raised) before `afc.current`, because by the time a human
        presses resume the unload has usually already cleared `current` -- that
        is the whole reason the toolhead is empty.

        :param gcmd: The Klipper GCodeCommand (for operator-facing output)
        :raises gcmd.error: if the reload did not take
        """
        try:
            if not self.afc.function.is_paused():
                return
        except Exception:
            return
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
            raise gcmd.error(
                f"{unit.name}: {name} did NOT reload -- the toolhead is empty, "
                f"so the print stays PAUSED. Clear the jam and resume again.")
        unit._resume_needs_reload = False
        unit._auto_recover_armed = False     # a fresh fault may retry

    def _resume_reload_target(self) -> tuple:
        """
        Find the (unit, lane) a resume should reload, or (None, None).

        RESUME is printer-wide but we are one unit among possibly several, so
        this searches every Bambu unit that has a fault pending rather than
        only ourselves. Whichever unit raised the fault owns the reload, no
        matter which one happened to win the wrap.

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

        Derived from AFC's lane table rather than a registry we maintain: the
        lanes already point at their units, so there is nothing to keep in sync
        and a unit with no lanes cannot own a reload anyway.

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

    def cmd_AFC_BAMBU_TAIL(self, gcmd: Any) -> None:
        """
        Derive the frame tail byte from the mode, or send the hardcoded 0x02.

        AFC_BAMBU_TAIL UNIT=<unit> ON=<0|1>       (OFF by default)

        ON matches what a real printer sends -- 0x02 for hold and transition,
        0x00 otherwise, which agrees with 32,905 captured frames where the
        hardcoded 0x02 agrees with 60%. It also made loading WORSE across three
        sessions, ending with the spool trying to unwind at the moment of
        toolhead detection.

        The rule is right about the printer and wrong about us: we do not
        arrive in those states the way the printer does, and do_pending's
        OP_FEED / OP_RETRACT / OP_SELECT are the commands that physically move
        filament.

        THIS COMMAND EXISTS BECAUSE THE TOGGLE HAD NO G-CODE PATH. It was
        reachable only as bridge JSON, so proving it was the culprit needed a
        reflash instead of one command. A switch you cannot reach in the moment
        you need it is not a switch.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_TAIL UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_TAIL UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_TAIL: bridge not connected")
        on = gcmd.get_int("ON", minval=0, maxval=1)
        self._bridge.send({"cmd": "tail", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_TAIL: {self.name}: tail byte "
            + ("DERIVED from the mode (experimental -- made loads worse)"
               if on else "hardcoded 0x02 (what every working load has used)"))

    def cmd_AFC_BAMBU_TXECHO(self, gcmd: Any) -> None:
        """
        Record the frames WE TRANSMIT, so a load can be measured not inferred.

        AFC_BAMBU_TXECHO UNIT=<unit> ON=<0|1>

        THE ONE THING NOBODY CAN SEE on this bus is our own output. The sniff
        build is listen-only and WE are the master, so it can capture a real
        printer and never us. Every wrong call in one evening came from
        inferring what we put on the wire: a phase that could not be reached,
        an enrollment branch that could not be reached, and an op-03 byte that
        correlated with the ref across 32,000 captured frames and faulted a
        unit at 1.39A when we sent it.

        Lines land in the AMS narration log in the SAME shape as a capture --
        {"evt":"tx","us":..,"n":..,"hex":".."} -- so the existing tools diff a
        TX log against ht_clean_load with no changes.

        ON for a load, OFF straight after. It is a diagnostic, not a
        background cost: at 21ms the drive channel alone is ~48 lines a second.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_TXECHO UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_TXECHO UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_TXECHO: bridge not connected")
        on = gcmd.get_int("ON", minval=0, maxval=1)
        self._bridge.send({"cmd": "txecho", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_TXECHO: {'RECORDING' if on else 'off'} -- frames we "
            f"transmit go to the AMS narration log as evt:tx. "
            + ("Run the load, then AFC_BAMBU_TXECHO ON=0."
               if on else "Stopped."))

    def cmd_AFC_BAMBU_CLEARFAULT(self, gcmd: Any) -> None:
        """
        Stream the printer's fault clear at a parked unit, then check it took.

        AFC_BAMBU_CLEARFAULT UNIT=<unit>

        Measured identically on all three unit types: while a unit is parked the
        state channel carries op-04 0F/00 and the drive channel mirrors 0F/FF at
        21ms. The clear is op-03 0E/FF streamed on the DRIVE channel for ~2s
        (96-98 frames), so this streams it for the same span rather than firing
        once: the printer gives the unit two seconds of chances, not one.

        THIS IS AN ATTEMPT, NOT A COMMAND. The unit leaves its park only if the
        fault is actually gone -- it clears its own err_code when it accepts a
        fresh operation and it will not accept one while still jammed. On the HT
        capture the burst ran TWICE because the first did not take; the operator
        relieved the pressure and pressed continue again.

        So this reports what actually happened. Saying "recovered" without
        checking would be the machine lying about its own state, and the
        consequence is a print resuming into a jam.

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
            raise gcmd.error("AFC_BAMBU_CLEARFAULT: bridge not connected")
        before = self._unit_state(self._bridge.latest_status())
        self._bridge.send({"cmd": "clearfault"})
        # The burst is ~2s; wait it out plus a little for the state to settle.
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 2.6)
        after = self._unit_state(self._bridge.latest_status())
        if after == self.AMS_STATE_STALLED:
            gcmd.respond_info(
                f"AFC_BAMBU_CLEARFAULT: {self.name}: STILL FAULTED (state {after}). "
                f"The unit will not accept a fresh operation while the jam is "
                f"there -- relieve the pressure (the buffer coming off the "
                f"floor is the sign) and run this again.")
            return
        gcmd.respond_info(
            f"AFC_BAMBU_CLEARFAULT: {self.name}: cleared (state {before} -> "
            f"{after}); the unit accepted the operation and left its park.")

    def cmd_AFC_BAMBU_CLSPROBE(self, gcmd: Any) -> None:
        """
        Ask the unit at a bus id which subsystem it lives on.

        AFC_BAMBU_CLSPROBE UNIT=<unit> ID=<busid> [TRIES=1]

        An AMS HT answers on device 0x1800; a boxed AMS does not. That is the
        one class signal that is innate -- the announce tag byte is
        unit-generated, varies per session and is merely echoed by the master,
        so it cannot be used (measured: a boxed AMS and the HT both read
        tag=15).

        STANDALONE. Enrollment does not use this yet, on purpose. Prove it
        against units whose class you already know first, because the failure
        mode is the worst one on this bus: an HT that answers LATE is filed as
        boxed, enrolled at a boxed address, and goes silent.

        TRIES exists to answer exactly that. One silent round is not proof --
        ht_poll_seq is three round-trips and a busy unit can miss them. Find out
        how many a real HT needs before wiring this to anything.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_CLSPROBE UNIT=<unit> ID=<n> TRIES=<n>`

        Example
        -------
        ```
        AFC_BAMBU_CLSPROBE UNIT=BambuAMS_1 ID=1 TRIES=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_CLSPROBE: bridge not connected")
        bus_id = gcmd.get_int("ID", minval=0, maxval=255)
        tries = gcmd.get_int("TRIES", 1, minval=1, maxval=10)
        self._bridge.send({"cmd": "clsprobe", "id": bus_id, "tries": tries})
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.9)
        r = getattr(self._bridge, "_last_clsprobe", None)
        if not r:
            gcmd.respond_info(f"AFC_BAMBU_CLSPROBE: {self.name}: no reply "
                              "(firmware older than 1.0.85.0?)")
            return
        gcmd.respond_info(
            f"AFC_BAMBU_CLSPROBE: id=0x{int(r.get('id', 0)):02X} "
            f"tries={r.get('tries')} -> {'HT (0x1800)' if r.get('ht') else 'boxed / silent'}")

    def cmd_AFC_BAMBU_CLASSADDR(self, gcmd: Any) -> None:
        """
        Enroll and address units by CLASS, the way a real printer does.

        AFC_BAMBU_CLASSADDR UNIT=<unit> ON=<0|1>

        Boxed AMS take 0x00-0x03 and AMS HT take 0x80-0x87. Confirmed across
        every capture: work frames are only ever addressed to 0x00, 0x01 or
        0x80, and no UID ever appears in the other class's range.

        This changes ENROLLMENT and ADDRESSING together, which is the whole
        point -- fw 1.0.54.0 changed only the address and asked units for ids
        they had never been given. The HT UIDs must be registered first; that
        happens automatically from each HT unit's `unit_uid`.

        Verify with AFC_BAMBU_RC: the mask should move from three bits in 0-3 to two
        bits in 0-3 plus one bit in 4-11.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_CLASSADDR UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_CLASSADDR UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_CLASSADDR: bridge not connected")
        on = gcmd.get_int("ON", minval=0, maxval=1)
        self._bridge.send({"cmd": "classaddr", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_CLASSADDR: {self.name}: class addressing "
            f"{'ON' if on else 'OFF'} -- re-enrollment follows on the next "
            f"discovery round; check it with AFC_BAMBU_RC and AFC_BAMBU_UIDS")

    def cmd_AFC_BAMBU_RC(self, gcmd: Any) -> None:
        """
        Print the roll-call counters and which bus ids answered.

        AFC_BAMBU_RC UNIT=<unit>

        probes/answers are cumulative since the register was switched on;
        dividing probes by the elapsed time is how the 92ms cadence gets
        checked, because in master mode there is no sniff stream to time
        individual frames against -- the Pico cannot sniff and master at once.

        The mask is over BUS IDS, not our unit indices. Those are not the same
        thing and will not be until class-split addressing lands.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_RC UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_RC UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_RC: bridge not connected")
        self._bridge.send({"cmd": "rc"})
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.4)
        rc = getattr(self._bridge, "_last_rc", None)
        if not rc:
            gcmd.respond_info(f"AFC_BAMBU_RC: {self.name}: no rc reply "
                              "(firmware older than 1.0.59.0?)")
            return
        try:
            mask = int(str(rc.get("mask") or "0"), 16)
        except (TypeError, ValueError):
            mask = 0
        here = [f"0x{i:02X}" for b, i in enumerate(self.ROLLCALL_IDS)
                if mask & (1 << b)]
        gcmd.respond_info(
            f"AFC_BAMBU_RC: {self.name}: on={rc.get('on')} "
            f"probes={rc.get('probes')} answers={rc.get('answers')} "
            f"mask={rc.get('mask')} present=[{', '.join(here) or 'none'}]")

    def cmd_AFC_BAMBU_SENDRAW(self, gcmd: Any) -> None:
        """
        Send one raw frame on the bus and print whatever comes back, as hex.

        AFC_BAMBU_SENDRAW UNIT=<unit> HEX=<hex> [US=30000]

        The general form of AFC_BAMBU_POLL37: the bridge's raw passthrough with the
        reply surfaced. For working out what OUR units answer to a frame the
        captures show a printer sending -- reply lengths, field offsets -- without
        a firmware change per experiment. The frame goes out exactly as given
        (CRCs included), so build it correctly; the bridge does not fix it up.

        Diagnostic. It can command anything a frame can command, so treat it
        with the respect a raw bus write deserves.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_SENDRAW UNIT=<unit> HEX=<hex> US=<n>`

        Example
        -------
        ```
        AFC_BAMBU_SENDRAW UNIT=BambuAMS_1 HEX=3D0500 US=30000
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_SENDRAW: bridge not connected")
        hexf = gcmd.get("HEX")
        us = gcmd.get_int("US", 30000, minval=1000, maxval=200000)
        try:
            bytes.fromhex(hexf)
        except ValueError:
            raise gcmd.error(f"AFC_BAMBU_SENDRAW: bad hex {hexf!r}")
        self._bridge.send({"cmd": "raw", "hex": hexf, "us": us})
        try:
            self.afc.reactor.register_callback(
                self._report_sendraw, self.afc.reactor.monotonic() + 0.5)
        except Exception:
            self._report_sendraw(0)

    def _report_sendraw(self, eventtime: float) -> None:
        """
        Print the raw reply hex, unedited -- offsets are the caller's job.

        :param eventtime: reactor time supplied by the callback; unused
        """
        hexs = ""
        try:
            hexs = getattr(self._bridge, "_last_raw_reply", "") or ""
        except Exception:
            pass
        self.gcode.respond_info(
            f"AFC_BAMBU_SENDRAW: {self.name} len={len(hexs)//2}\n{hexs or '(no reply)'}")

    def cmd_AFC_BAMBU_EXTMIMIC(self, gcmd: Any) -> None:
        """
        Emulate the other side of the AP2 conversation. AFC_BAMBU_EXTMIMIC ON=<0|1>

        On a real printer the AP2 sync is a dialogue: every sync to the extruder
        (0x0E00) is answered by a 35-byte frame back to AP2 (0x0900), one for
        one -- 46 syncs and 46 replies in ht_alone_idle.txt. Our bridge sends the
        syncs and nothing answers them.

        The AMS is not addressed by any of it. The sync goes to the extruder, so
        on a shared bus the AMS can only be sniffing. The open question is
        whether what it watches for is the request (which we already send) or the
        REPLY (which we never did), and this switch is how that gets answered on
        hardware rather than argued about.

        Both emulated frames are constants reconstructed byte-for-byte from 745
        and 740 captured samples; see docs/CAPTURE_FINDINGS.md.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_EXTMIMIC UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_EXTMIMIC UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_EXTMIMIC: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        on = gcmd.get_int("ON", 0, minval=0, maxval=1)
        self._bridge.send({"cmd": "extmimic", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_EXTMIMIC: extruder-side AP2 emulation "
            f"{'ON' if on else 'off'} -- judge it against a real load, not "
            f"against bridge_online")

    def cmd_AFC_BAMBU_HT0FHOLD(self, gcmd: Any) -> None:
        """
        Hold a following HT with the dense statu-0F poll instead of the dense
        ht_poll_seq re-poke. AFC_BAMBU_HT0FHOLD UNIT=<unit> ON=<0|1>

        The capture-faithful follower hold (the real printer streams 5920 0F
        polls at 21ms and zero re-arms). Runtime toggle so it can be tested
        live and flipped back to the safe re-arm fallback instantly if the
        feed sags. Off by default.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_HT0FHOLD UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_HT0FHOLD UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error(f"AFC_BAMBU_HT0FHOLD: bridge not connected for {self.name}")
        on = gcmd.get_int("ON", 0, minval=0, maxval=1)
        self._bridge.send({"cmd": "ht0fhold", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_HT0FHOLD: dense-0F hold {'ON' if on else 'off'} for "
            f"{self.name} (flip ON=0 to return to the re-arm fallback)")

    def cmd_AFC_BAMBU_POLL0F(self, gcmd: Any) -> None:
        """
        Toggle the statu-0x0F loaded-state poll.

        AFC_BAMBU_POLL0F UNIT=<unit> ON=<0|1>

        What a real printer runs at a 21ms median cadence for the whole time a
        tray feeds the toolhead -- 5920 of them in the HT load capture, zero in
        any idle capture -- and, notably, the 11/04 arm does NOT repeat during
        that phase, so this poll rather than re-arming is what holds the
        assist. The reply doubles as live telemetry (state flag, selected
        tray, odometer). Requires MOTION6; the firmware polls at 50ms.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_DRIVE UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_DRIVE UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            raise gcmd.error(f"AFC_BAMBU_POLL0F: bridge not connected for {self.name}")
        on = gcmd.get_int("ON", 0, minval=0, maxval=1)
        self._bridge.send({"cmd": "p0f", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_POLL0F: loaded-state poll {'ON' if on else 'off'} for "
            f"{self.name} (only fires while following, and only with MOTION6)")

    def cmd_AFC_BAMBU_MOTION6(self, gcmd: Any) -> None:
        """
        Send the op-0x03 motion request the length a real bus uses.

        AFC_BAMBU_MOTION6 UNIT=<unit> ON=<0|1>

        Every one of the 3554 op-0x03 requests in the captures is a 12-byte
        frame -- six body bytes. Ours builds five and omits the last, so every
        dynamically-built motion frame this bridge has ever sent is a byte
        shorter than anything a printer sends. The tail tracks `statu`: 0x03
        pairs with 0x00, 0x07 and 0x09 with 0x02.

        Behind a switch rather than simply corrected, because the 5-byte frame
        does drive real loads today. "Closer to the wire" and "behaves better"
        are different claims and only hardware settles the second one.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_MOTION6 UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_MOTION6 UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_MOTION6: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        on = gcmd.get_int("ON", 0, minval=0, maxval=1)
        self._bridge.send({"cmd": "m6", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_MOTION6: 6-byte motion body "
            f"{'ON' if on else 'off'} for {self.name}")

    def cmd_AFC_BAMBU_UNIT80(self, gcmd: Any) -> None:
        """
        Address HT units as 0x80 in SHORT frames, the way a real printer does.

        AFC_BAMBU_UNIT80 UNIT=<unit> ON=<0|1>

        Both real-printer HT captures put 0x80 in the unit byte of every short
        op-0x03 and op-0x04 request -- 349 of 350 frames -- while the HT in
        them is the only unit on the bus, chain index 0. So 0x80 is how a
        printer NAMES an HT, not an index, and our short frames have been
        sending the chain index instead. The long 0x1800 commands already use
        0x80 (HT_ID_DEFAULT) because it was the only value that made the unit
        act; this extends the same treatment to the short frames.

        Independent of AFC_BAMBU_MOTION6 so the two corrections A/B one at a time.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_UNIT80 UNIT=<unit> ON=<0 or 1>`

        Example
        -------
        ```
        AFC_BAMBU_UNIT80 UNIT=BambuAMS_1 ON=1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_UNIT80: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        on = gcmd.get_int("ON", 0, minval=0, maxval=1)
        self._bridge.send({"cmd": "u80", "on": on})
        gcmd.respond_info(
            f"AFC_BAMBU_UNIT80: short-frame HT addressing as 0x80 "
            f"{'ON' if on else 'off'} for {self.name}")

    def cmd_AFC_BAMBU_ARMMS(self, gcmd: Any) -> None:
        """
        Set the 11/04 follower keep-alive cadence at runtime, in ms.

        AFC_BAMBU_ARMMS UNIT=<unit> MS=<ms>       (0 restores the printer's ~507ms)

        This is the ONE per-cycle transmitter with no MUTE_* bit, so when a
        unit ticks at idle it is the only suspect that cannot be bisected by
        AFC_BAMBU_MUTE. Winding the cadence out to tens of seconds is the
        equivalent, and needs no reflash. The frame is never answered, so
        slowing it costs a liveness signal and nothing else -- but the AMS does
        declare itself offline without one, so put it back afterwards.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_ARMMS UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_ARMMS UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_ARMMS: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        ms = _gcmd_int(gcmd, "MS", 0, 0, 600000)
        self._bridge.send({"cmd": "armms", "ms": ms})
        gcmd.respond_info(
            f"AFC_BAMBU_ARMMS: follower keep-alive "
            f"{'default (~507ms)' if ms == 0 else str(ms) + 'ms'}")

    def cmd_AFC_BAMBU_HTID(self, gcmd: Any) -> None:
        """
        Set the id byte the bridge uses on an AMS HT's 0x1800 commands.

        AFC_BAMBU_HTID UNIT=<unit> ID=<0-255>

        An HT only acts on commands addressed to the right id, and which id that
        is has not been settled: the real-printer capture uses 0x80 (128), the
        unit reports ref:165, and the chain index is 0. ID=0 falls back to the
        chain index, ID=255 tracks whatever the AMS reports.

        For finding the answer on hardware instead of by argument.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_HTID UNIT=<unit>`

        Example
        -------
        ```
        AFC_BAMBU_HTID UNIT=BambuAMS_1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_HTID: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        htid = _gcmd_int(gcmd, "ID", 128, 0, 255)
        self._bridge.send({"cmd": "htid", "id": htid})
        gcmd.respond_info(
            f"AFC_BAMBU_HTID: 0x1800 commands now addressed to id {htid} "
            f"(0=chain index, 255=track the AMS ref).")

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
            # Did OUR unit's arm land? fstate above is bus-wide; this is the
            # per-unit receipt, and the two disagreeing is the whole point.
            f"arm_acked={self._follow_arm_acked(latest)} "
            # The narration counters. They were already in every status frame
            # and nothing surfaced them, so "the AMS has nothing to say" and
            # "we stopped asking" stayed indistinguishable -- the exact
            # ambiguity that cost an afternoon of guessing. polls counts drain
            # requests SENT, frames the narration-shaped replies, texts the
            # ones that actually carried words.
            f"dbg_polls={latest.get('dbgpolls')} "
            f"dbg_frames={latest.get('dbgframes')} "
            f"dbg_texts={latest.get('dbgtexts')} "
            f"dbg_cut={latest.get('dbgtrunc')} "
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
            f"tube_len={self.measured_path_mm()} "
            # The unit's END-OF-FEED length, recorded and deliberately wired to
            # nothing. tube_len reads None on all three units under our master
            # while the captures show the printer's master getting it -- so the
            # question is why we never receive it, and this is the measurement
            # that says whether dw_len is even the same quantity. n= is the
            # sample count: one reading proves only that the word exists.
            f"dw_len={self._dw_len_str()} "
            f"bowden={self.afc_bowden_length:.0f} "
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

    def cmd_AFC_BAMBU_SCAN(self, gcmd: Any) -> None:
        """
        Trigger an RFID/tag scan on demand -- the exact read the auto-scan runs
        on a fresh insert, but callable by hand.

        AFC_BAMBU_SCAN UNIT=<unit> [LANE=<lane>]

        With no LANE, scans every slot on the unit; with LANE, just that lane's
        slot. Mainly for the AMS HT, whose tag only reads when the bridge polls
        it at 0x1800 -- run this if a spool's material never populated. The read
        also clears the per-slot auto-scan latch so the result is fresh.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_SCAN UNIT=<unit> LANE=<lane>`

        Example
        -------
        ```
        AFC_BAMBU_SCAN UNIT=BambuAMS_1 LANE=lane1
        ```
        """
        if self._bridge is None:
            msg = f"AFC_BAMBU_SCAN: bridge not connected for {self.name}"
            raise gcmd.error(msg)
        lane_name = gcmd.get("LANE", None)
        if lane_name is not None:
            lane = self.lanes.get(lane_name)
            if lane is None:
                raise gcmd.error(
                    f"AFC_BAMBU_SCAN: lane '{lane_name}' not on unit {self.name} "
                    f"(lanes: {', '.join(self.lanes) or 'none'})")
            slot = self._slot_of(lane)
            if slot is None:
                raise gcmd.error(
                    f"AFC_BAMBU_SCAN: {lane_name} is not mapped to an AMS slot")
            # Drop the latch so the manual scan always re-reads (even if an
            # earlier auto-scan already fired for this slot), and open the scan
            # so it resolves through the SAME two outcomes an auto-scan does:
            # the unit reads a tag, or the lane gets its defaults. Without the
            # open, a manual scan asked a question nothing was listening for --
            # the bay's leftover record surfaced immediately as if it were the
            # answer, and a re-scan after a failed one stayed latched no-tag.
            if 0 <= slot < len(self._auto_scanned):
                self._auto_scanned[slot] = False
            self._open_scan(slot)
            # A manual scan measures as well -- one tag read then the spool
            # measurement, so a scan always answers how much filament is on
            # the spool (measured percent -> slot remain + lane grams).
            if not afcBambuAMS._start_capscan(self, slot):
                msg = (f"AFC_BAMBU_SCAN: scan command not issued for {lane_name} "
                       f"on {self.name}")
                raise gcmd.error(msg)
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

    def _start_capscan(self, slot: int) -> bool:
        """
        Kick the capacity choreography for one slot: one tag read, then the
        spool measurement, with the narrated percent attributed back to this
        slot (remain_pct + lane grams). The single entry point for the insert
        edge, AFC_BAMBU_SCAN LANE=, and AFC_BAMBU_CAPSCAN, so a scan ALWAYS answers
        "how much filament is on this spool".

        :param slot: 0-based AMS slot index
        :return bool: True if the command was issued
        """
        if self._bridge is None:
            return False
        # ONE SPOOL OPERATION ON THE BUS AT A TIME. Two units scanning at once
        # is not a hypothetical: it happened during a relink and took Klipper
        # down with it. A real printer serialises these and narrates the
        # handoff ("[AMS_CALL] ams1 select, select ams2").
        claim = getattr(self._bridge, "try_claim_bus", None)
        if callable(claim):
            try:
                now = self.afc.reactor.monotonic()
            except Exception:
                now = 0.0
            if not claim(self.name, now):
                who = self._bridge.bus_owner()
                self.logger.info(
                    f"AFC bambu {self.name}: deferring the scan of slot "
                    f"{slot} -- {who} has a spool operation running on this "
                    f"bus. It will be retried; nothing is lost.")
                return False
        self._cap_pending_slot = slot
        try:
            self._cap_pending_t0 = self.afc.reactor.monotonic()
        except Exception:
            self._cap_pending_t0 = 0.0
        if self._is_ht():
            # The HT's scan machinery is firmware-armed at 0x1800 and already
            # opens the measurement window on its own; the plain scan command
            # drives it. The pending marker above still attributes the
            # narrated percent here.
            return self.scan(slot)
        self._bridge.send({"cmd": "capscan", "trig": 1,
                           "unit": self.ams_index, "slot": slot})
        return True

    def cmd_AFC_BAMBU_RDINFO(self, gcmd: Any) -> None:
        """
        Dump the raw 0x0211 filament-info reply for one bay.

        Reads the frame the AMS actually sends, before decode_filament_reply
        touches it. Exists because "the field is absent" and "we parsed the
        wrong offset" are indistinguishable from a decoded record, and this
        project has drawn the wrong conclusion from that ambiguity more than
        once.

        :param gcmd: The Klipper GCodeCommand
        """
        if self._bridge is None:
            raise gcmd.error(f"AFC_BAMBU_RDINFO: bridge not connected for {self.name}")
        lane_name = gcmd.get("LANE")
        lane = self.lanes.get(lane_name) if hasattr(self, "lanes") else None
        if lane is None:
            raise gcmd.error(
                f"AFC_BAMBU_RDINFO: lane '{lane_name}' not on unit {self.name}")
        slot = self._slot_of(lane)
        self._bridge.send({"cmd": "rdinfo", "unit": int(self.ams_index),
                           "slot": int(slot),
                           "addr": int(getattr(self, "dry_dev_addr", 0) or 0)})
        self.afc.reactor.pause(self.afc.reactor.monotonic() + 0.6)
        rec = self._bridge.last_rdinfo() or {}
        hexs = str(rec.get("hex", ""))
        gcmd.respond_info(
            f"{self.name} slot {slot}: len={rec.get('len')}\n{hexs}")

    def cmd_AFC_BAMBU_REID(self, gcmd: Any) -> None:
        """
        Send the printer's "re-identify" to one bay, on its own.

        AFC_BAMBU_REID UNIT=<unit> LANE=<lane>

        ONE FRAME, DELIBERATELY. The type-07 select with payload byte[7]=0x00 --
        what the printer's menu sends -- and nothing around it, so whatever the
        unit narrates next is its answer to THAT and not to a burst.

        Why it is worth a command of its own: on a boxed AMS an insert produces
        "first detected" and stops there. The SECOND detection comes from this
        frame, and the unit needs two tag passes to derive a circumference and
        hence a spool measurement. Captured on a real printer
        (ams2_insert_reidentify.txt): insert -> "first detected", menu
        re-identify -> "second detected".

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_REID UNIT=<unit> LANE=<lane>`

        Example
        -------
        ```
        AFC_BAMBU_REID UNIT=BambuAMS_1 LANE=lane1
        ```
        """
        lane_name = gcmd.get("LANE")
        lane = self.lanes.get(lane_name) if hasattr(self, "lanes") else None
        if lane is None:
            raise gcmd.error(
                f"AFC_BAMBU_REID: lane '{lane_name}' not on unit {self.name}")
        if self._bridge is None:
            raise gcmd.error(f"AFC_BAMBU_REID: bridge not connected for {self.name}")
        slot = self._slot_of(lane)
        if slot is None:
            raise gcmd.error(
                f"AFC_BAMBU_REID: {lane_name} is not mapped to an AMS slot")
        self._bridge.send({"cmd": "reid", "unit": self.ams_index, "slot": slot})
        gcmd.respond_info(
            f"AFC_BAMBU_REID: sent re-identify to {self.name} bay {slot} "
            f"({lane_name}). Watch for 'second detected' and a measurement.")

    def cmd_AFC_BAMBU_CAPSCAN(self, gcmd: Any) -> None:
        """
        Run the printer's capacity-measuring re-scan on one bay.

        AFC_BAMBU_CAPSCAN UNIT=<unit> LANE=<lane>

        The insert choreography a real printer performs, which plain
        AFC_BAMBU_SCAN does not: re-identify trigger, statu-01 probe, then the
        05/80 capacity ENABLE that arms ams_state 3 -- the state in which the
        AMS measures the spool's radius during its preload pull and persists
        remain% to the tag record ("odom calib success, dis:0.776"). Read the
        result a minute later: the slot's remain_pct in status.

        Works on both unit classes -- the HT gets the same enable at unit
        byte 0x80, straight from the real-printer HT insert capture. Refused
        while printing and while any lane on the unit is tool-loaded: the probe visibly bounces the
        AMS's link mode (mode 2 -> 0 -> 2 on live hardware), which is nothing
        to do to a unit feeding an extruder.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_CAPSCAN UNIT=<unit> LANE=<lane>`

        Example
        -------
        ```
        AFC_BAMBU_CAPSCAN UNIT=BambuAMS_1 LANE=lane1
        ```
        """
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_CAPSCAN: bridge not connected")
        # HT units are allowed now: the real-printer HT insert capture showed
        # the same enable streamed at unit byte 0x80 (068005008000 x17), and
        # "tray capacity no en" was the unit naming the missing frames. The
        # firmware streams them through the scan window for both classes.
        try:
            if self.afc.function.in_print():
                raise gcmd.error("AFC_BAMBU_CAPSCAN: refused while printing")
        except AttributeError:
            pass
        if _unit_tool_loaded(self):
            raise gcmd.error(
                "AFC_BAMBU_CAPSCAN: refused -- a lane on this unit is tool-loaded "
                "and the probe bounces the unit's link mode")
        lane_name = gcmd.get("LANE")
        lane = self.lanes.get(lane_name)
        if lane is None:
            raise gcmd.error(
                f"AFC_BAMBU_CAPSCAN: lane '{lane_name}' not on unit {self.name}")
        slot = self._slot_of(lane)
        if slot is None:
            raise gcmd.error(
                f"AFC_BAMBU_CAPSCAN: {lane_name} is not mapped to an AMS slot")
        afcBambuAMS._start_capscan(self, slot)
        gcmd.respond_info(
            f"AFC_BAMBU_CAPSCAN: capacity re-scan armed for {lane_name} "
            f"(slot {slot}) on {self.name} -- the AMS pulls the spool, "
            f"re-reads the tag and measures remain%; check the slot's "
            f"remain_pct in about a minute")

    def cmd_AFC_BAMBU_HEATER_START(self, gcmd: Any) -> None:
        """
        Start AMS drying (AMS2 Pro heater).

        AFC_BAMBU_HEATER_START UNIT=<unit> [TEMP=55] [TIME=480] [ROTATE=0]

        TEMP in C (clamped 0..65), TIME in minutes, ROTATE=1 spins the spools
        while drying. Replays the printer's drying command on our bus.

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

        # A new attempt owns the outcome. The stored refusal is the reason the
        # LAST one failed, and nothing clears it on an HT -- so leaving it would
        # keep reporting a stale failure over a dry that started fine.
        try:
            self._bridge.clear_dry_error(getattr(self, "dry_dev_addr", 0))
        except Exception:
            pass
        # Clamp (not hard-error) to this unit's drying ceiling (dry_max_temp:
        # 65 for AMS2 Pro, 85 for AMS HT). Asking for more would be rejected by
        # the AMS or risk the spools, so cap it and tell the user rather than
        # halting.
        temp = gcmd.get_int("TEMP", 55, minval=0)
        if temp > self.dry_max_temp:
            gcmd.respond_info(
                f"AFC_BAMBU_HEATER_START: TEMP {temp}C exceeds {self.name}'s drying "
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
                    f"AFC_BAMBU_HEATER_START: ROTATE disabled for {self.name} -- "
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
                f"AFC_BAMBU_HEATER_START: {names} is loaded to the toolhead, so "
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
                f"AFC_BAMBU_HEATER_START: AMSID/ADDR override ({amsid}/0x{addr:04X}) "
                f"does not match {self.name}'s own addressing "
                f"({self.dry_ams_id}/0x{self.dry_dev_addr:04X}) — another "
                f"unit's heater would be targeted. Drop the override, or add "
                f"FORCE=1 for deliberate diagnostics.")
        # Only command a unit that has actually answered. A real printer runs
        # the bus as a roll-call -- it enumerates every id every ~1.1 s and
        # sends WORK frames only to the ids that replied (captured: 12 ids
        # probed continuously, op-04/op-03 to the one unit present and nothing
        # to the other eleven). Firing a heater command at a unit that is not
        # on the wire puts a frame nobody answers on the bus and, worse, leaves
        # _drying set here -- which idles the follower tick for a dry cycle
        # that is not running.
        # Fail-open on "cannot tell": only refuse when telemetry EXISTS and
        # says the unit is absent. No status at all means we have not heard
        # anything yet -- true for the first moments after a restart -- and
        # refusing then would block a heater on our own ignorance rather than
        # on the unit's absence.
        online = getattr(self, "_unit_online", None)
        getst = getattr(self._bridge, "latest_status", None)
        latest = getst() if callable(getst) else None
        if latest and callable(online) and not online(latest):
            raise gcmd.error(
                f"AFC_BAMBU_HEATER_START: {self.name} is not online -- nothing on "
                f"the bus is answering for it, so the heater command would go "
                f"nowhere. Check the unit is powered and chained, then retry.")
        self._bridge.send({"cmd": "dry", "unit": self.ams_index, "on": 1,
                           "temp": temp, "time": tmin, "rotate": rot,
                           "addr": addr, "amsid": amsid})
        # Remember when this cycle was commanded and for how long, so a display
        # can show time remaining. The AMS does not report it -- the duration
        # only ever exists here, in the command we sent -- so if this is not
        # recorded at the moment of sending it cannot be recovered later.
        self._dry_started_at = _mono(self)
        self._dry_minutes = int(tmin)
        self._dry_rotate = 1 if rot else 0
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
        self._bridge.send({"cmd": "dry", "unit": self.ams_index, "on": 0,
                           "addr": self.dry_dev_addr, "amsid": self.dry_ams_id})
        self._drying = False
        self._dry_started_at = None
        self._dry_minutes = 0
        self._dry_rotate = 0
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
        gcmd.respond_info(f"AFC_BAMBU_HEATER_STOP: {self.name} drying stopped.")

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
            raise gcmd.error("AFC_BAMBU_UIDS: bridge not connected")
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
        lines = [f"Bambu AMS bus (firmware {fw or 'pre-0.3.0'}) -- copy each "
                 f"UID into that unit's `unit_uid`:"]
        for i, u in enumerate(uids):
            occ = [f"slot{s.get('i')}={s.get('material') or 'present'}"
                   for s in slots
                   if s.get("unit") == i and s.get("present")]
            hint = ("  <- " + ", ".join(occ)) if occ else "  <- (empty)"
            ht = "  [HT-flagged]" if htmask & (1 << i) else ""
            # Announce-reply tag byte. CANDIDATE signal for working class out
            # from the bus instead of from config -- observe only, nothing acts
            # on it. Compare it against each unit's ams_model across several
            # re-enrollments: if the tag tracks CLASS, detection is free; if it
            # does not, the fallback is an active 0x1800 probe per new uid.
            # The same HT has shown 0A and 04 minutes apart, so it is certainly
            # not a stable per-UNIT id. Three units is not enough to call it.
            tags = (getattr(self._bridge, "_chain_tags", "") or "").split(",")
            tg = f"  tag={tags[i]}" if i < len(tags) and tags[i] else ""
            lines.append(f"  chain index {i}: {u or '(none)'}{ht}{tg}{hint}")
        # Capacity diagnostics: capn = lifetime capacity-stream frames, capdiag
        # = (last op-04 poll unit byte)<<8 | boxed-burst-ran. If the poll byte
        # is 00 while an HT is being scanned, the measure poll is leaking to the
        # boxed AMS (ams0) instead of the HT (0x80).
        cd = getattr(self._bridge, "_chain_capdiag", 0) if self._bridge else 0
        cn = getattr(self._bridge, "_chain_capn", 0) if self._bridge else 0
        lines.append(f"  capacity: capn={cn} poll_ub=0x{(cd>>8)&0xFF:02X} "
                     f"boxed_burst={cd & 1}")
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
        self._scan_notag = [False] * self.SLOTS_PER_UNIT
        # Inserts whose scan was held back because a lane on THIS unit was
        # threaded to the toolhead. Retried once the unit is free; see
        # _maybe_auto_scan.
        self._scan_defer = [False] * self.SLOTS_PER_UNIT
        # When the current dry cycle was commanded, for how long, and whether
        # it spins the spool. Set here and not only in cmd_AFC_BAMBU_HEATER_START:
        # get_status runs on every Moonraker query from the moment the object
        # exists, which is long before anything is ever told to dry. Leaving one
        # of these to the command path is what took Klippy down with an
        # AttributeError; all three belong here.
        self._dry_started_at = None
        self._dry_minutes = 0
        self._dry_rotate = 0
        self._dry_refusal_logged = False
        self._scan_t0 = [None] * self.SLOTS_PER_UNIT
        self._scan_motion_t0 = [None] * self.SLOTS_PER_UNIT
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
            #
            # And one file PER MASTER. Two Picos writing one log cannot be
            # untangled afterwards -- the only per-line attribution is the
            # device address, and two boxed units on different buses both
            # narrate as 0x0700. The first master keeps the plain
            # AFC_BambuAMS.log so a single-Pico printer is unchanged;
            # subsequent ones are tagged by their serial port.
            try:
                import os
                log_file = self.printer.start_args.get("log_file", None)
                if log_file:
                    bridge.set_narration_log(os.path.dirname(log_file),
                                             _bridge_log_tag(self.serial_port))
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: no narration log ({e})")
            _BRIDGES[self.serial_port] = bridge
        self._bridge = bridge
        bridge.add_listener(self._on_status)
        bridge.add_reconnect_listener(self._on_bridge_reconnect)
        # Take over RESUME so the ordinary resume button reloads a lane a Bambu
        # fault emptied. Armed here (not registered here) because AFC renames
        # RESUME later, from inside PREP -- see _arm_resume_wrap. Once per
        # printer; the guard is module-level.
        self._arm_resume_wrap()
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
        # Same callback fills in defaults for any bay that came up present with
        # no tag: by now the firmware has reported presence and AFC has restored
        # the lanes, so "still blank" means blank for real.
        try:
            def _prime(et):
                self._scan_primed = True
                self._reconcile_empty_bays()
                self._restore_untagged_defaults()
            self.afc.reactor.register_callback(
                _prime, self.afc.reactor.monotonic() + 8.0)
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
        # delay, exactly like AFC_BAMBU_UIDS does.
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
                f"AFC_BAMBU_UIDS. Holding chain index {self.ams_index}.")

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
        self._id_resolved = True          # a real chain index, not the default
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
            self._send_mc_addr(self._bridge)
            if self._announce_deferred:      # held at connect -- send it now
                self._announce_deferred = False
                self._announce_defer_t0 = 0.0
                self._announce_defer_warned = False
                self._announce_unit()
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
        # Do NOT register at a guessed index. When unit_uid is configured but
        # has not resolved yet (the blocking resolve can time out while a
        # just-rebooted Pico re-enumerates a multi-unit chain), ams_index is
        # still the config DEFAULT -- 0 -- and every registration below would
        # be filed against whichever unit actually holds index 0. On a 3-unit
        # bus that means the HT receiving the other two units' HT flag, MC
        # address and self-centre flag on every restart. Defer instead: the
        # resolve retries in the background, and _adopt_index replays this the
        # moment the real index is known.
        if self.unit_uid and not self._id_resolved:
            self._announce_deferred = True
            # DEBUG, not warning: at boot the chain map has simply not arrived
            # yet, and holding is the correct thing to do -- it is the whole
            # point of resolving by UID. Shouting about it on every single
            # start trained the operator to ignore the log.
            #
            # It only becomes a problem if it NEVER resolves, so remember when
            # the wait started and let _check_chain_resolve escalate.
            if not getattr(self, "_announce_defer_t0", 0.0):
                try:
                    self._announce_defer_t0 = self.afc.reactor.monotonic()
                except Exception:
                    self._announce_defer_t0 = 0.0
            self.logger.debug(
                f"AFC bambu {self.name}: chain index not resolved yet (UID "
                f"{self.unit_uid}); holding this unit's registrations until "
                f"the chain map arrives")
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
                ("mc address", lambda: self._send_mc_addr(self._bridge)),
                ("arm cadence", lambda: self._bridge.send(
                    {"cmd": "armms", "ms": int(FOLLOW_ARM_MS)}))):
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
        # NO Pico reboot here, deliberately.
        #
        # There was one: the bridge had wedged into the log drain's 0x0700
        # fallback and stayed there across Klipper restarts and even
        # reflashes, and a AFC_BAMBU_RESTART cleared it every time -- so rebooting
        # on every reconnect looked like the fix.
        #
        # It was not. The wedge is the drain PAYLOAD: an HT answers 0x00, never
        # 0x80, so sending 0x80 draws nothing at all and the drain falls back.
        # With the payload right the narration holds up on its own, so a reset
        # here solves nothing.
        #
        # It is not free. The reset is a watchdog_reboot with no graceful USB
        # detach, so the Pico vanishes and re-enumerates -- and this fired on
        # EVERY reconnect, which means every module deploy. Suspected of
        # taking a USB-CAN adapter down with it (twice in one evening).
        #
        # And no manual command for it either. Rebooting the Pico is a
        # watchdog_reboot with no graceful USB detach, and on this machine it
        # took the USB-CAN adapter down with it -- twice in one evening.
        # Nothing needs it: narration survives Klipper and firmware restarts
        # on its own now that the drain payload is right, which is what the
        # reset was really compensating for.
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
                    # Same number, but it was only ever the CONFIG DEFAULT --
                    # never confirmed against the chain. Adopt so the index
                    # counts as resolved and any registrations held back at
                    # connect are released. Without this a unit whose real
                    # index happens to equal the default (an HT at 0) would sit
                    # deferred forever, because the test above only fires on a
                    # CHANGE.
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
            # IDENTITY BEFORE DATA. With unit_uid configured, ams_index is the
            # config DEFAULT (0) until the chain map resolves the UID -- so
            # every unit on the bus matches unit 0's slots and applies the same
            # spool to its own lanes. Observed live: one HT tag written onto
            # lane15, lane19 AND lane23 in the same instant, because all three
            # units were still sitting at index 0.
            #
            # _announce_unit already defers REGISTRATIONS for this reason; slot
            # data needs the same gate. Status frames arrive continuously, so
            # the first frame after resolution applies correctly -- nothing is
            # lost by waiting, and applying early is actively wrong.
            if (getattr(self, "unit_uid", None)
                    and not getattr(self, "_id_resolved", True)):
                return
            # LATCH THE UNIT'S "I HAVE GIVEN UP" ON EVERY FRAME.
            #
            # byte[19] == 0x07 is the park signal and it is INTERMITTENT.
            # Counted in the AMS 1 fault capture, per phase:
            #
            #     HOLD (printing)   1333 frames    0 x 0x07
            #     RETRY             2686 frames   12 x 0x07  (0.4%)
            #     PARK              2523 frames  278 x 0x07  (11.0%)
            #     HOLD (after)      1333 frames    0 x 0x07
            #
            # It appears ONLY in the park, so the signal is sound -- but at
            # 11% of frames. Sampling it once, at the end of a ~90 s recovery
            # attempt, is roughly a one-in-nine chance of catching it, which
            # is why the check never fired on hardware while the operator was
            # looking at a unit latched red.
            #
            # So: watch every frame and LATCH. Cleared when a fault is armed
            # (_raise_ams_fault), so what it means is precisely "this unit has
            # declared since THIS fault began".
            #
            # In its OWN try. Everything below shares one except, so a throw
            # here would abandon the whole frame -- no slot data, no lane sync,
            # silently, for every frame. A diagnostic latch must not be able to
            # take status mirroring down with it.
            try:
                if self._unit_state(obj) == self.AMS_STATE_STALLED:
                    self._declared_since_fault = True
                self._track_odom(obj)
            except Exception:
                pass
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

    #: The unit's own state, from op-04 reply byte[19]. 0x07 is STALLED -- the
    #: same "state:7" the HT and AMS 2 print in words, and the ONLY fault
    #: signal an AMS 1 emits at all (it narrates nothing through either the
    #: fault or the recovery). Firmware debounces it; 255 = not heard from.
    AMS_STATE_STALLED = 0x07

    def _unit_state(self, latest: Optional[dict]) -> Optional[int]:
        """
        THIS unit's reported state from a bridge status frame.

        :param latest: A bridge status dict (or None)
        :return int: the state byte, or None if this unit has not reported one
        """
        if not latest:
            return None
        for u in latest.get("units") or []:
            if u.get("n") == self.ams_index:
                st = u.get("ustate")
                # 255 is the firmware saying "not heard from yet", which is not
                # a state and must never be read as one.
                return None if st is None or st == 0xFF else int(st)
        return None

    #: How far the AMS must have moved filament, in mm, during a recovery
    #: attempt before we will say it was moving. Measured: a working AMS swings
    #: the FULL tube during its retry (1.839 m of a 1.864 m tube in the AMS 1
    #: capture), and a unit that cannot move filament sits near zero. So this
    #: only has to separate "swung the tube" from "barely twitched" -- it is
    #: nowhere near either population, deliberately.
    ODOM_MOVED_MM = 200.0

    def _track_odom(self, obj: dict) -> None:
        """
        Record the range the AMS's odometer covers while a fault is pending.

        The odometer is a POSITION, not a consumption counter: 0 is home in
        the AMS, ~1.86 m is at the toolhead. During a print it sits pinned at
        tube length however much filament is consumed, so there is no clog
        signal in it and no clog detector here.

        The range does answer WHERE a jam is, and only once a recovery is
        already running -- a jammed retry sweeps most of the tube while a park
        barely moves. Cheap: no extra bus traffic, no new poll, and it never
        runs during a normal load.

        :param obj: A decoded bridge status event
        """
        fault = self._follow_fault_hold
        load = getattr(self, "_load_in_progress", False)
        if not fault and not load:
            return                          # nothing is asking the question
        for u in obj.get("units") or []:
            if int(u.get("n", -1)) != int(self.ams_index):
                continue
            v = u.get("odom")
            if v is None or int(v) == -1:   # firmware's unknown sentinel
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
            return

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

        A RANGE, NOT A DELTA, for the reason _track_odom gives: the odometer is
        a POSITION (0 = home in the bay, ~tube length = at the toolhead), so a
        unit that swings out and back reads a delta of zero having moved the
        whole tube twice. Start-vs-end would have called the busiest failure we
        have "the AMS never moved".

        :return float: the span, or None if we never got two readings
        """
        if self._load_odom_lo is None or self._load_odom_hi is None:
            return None
        return self._load_odom_hi - self._load_odom_lo

    def _jam_location(self, span: Optional[float] = None) -> str:
        """
        Say WHERE the jam is, from how far the AMS moved filament.

        A toolhead jam and a spool tangle need opposite responses -- the first
        wants the cut and retract, the second wants a human at the AMS -- so a
        hedged "tangled or jammed" message helps nobody.

        The wire answers it. If the AMS swung the filament up and down the tube
        and the toolhead sensor still never triggered, the AMS did its job and
        the blockage is downstream. If it barely moved, the blockage is at the
        AMS end.

        Hedges when it cannot tell, rather than picking one. An unknown span is
        genuinely unknown -- it means we got fewer than two readings, which
        happens if the unit went quiet.

        :param span: the range to judge, in mm. Defaults to the fault
          recovery's. A failed LOAD passes its own -- same question, different
          window, and the load's is the one that answers "the tube is not
          connected", because the AMS feeds happily into thin air and only its
          odometer knows how far the filament went.
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

        The PRIMARY detector, and the only one covering every unit: all three
        set op-04 reply byte[19] to 0x07 when they stall, at the same moment
        the two that narrate print "state:7". An AMS 1 sets the byte while
        emitting no fault text at all, so reading the words leaves it
        undetected.

        Being a byte, it avoids every dialect trap the narration carries --
        "[AMS_RFID]" vs "[AMS_DEV]", "STEP," vs "STEP:", "sucess"/"success",
        silent generations.

        0x07 never appears across 13,000+ healthy replies spanning prints,
        loads, unloads, scans and enrollment, and the firmware requires three
        agreeing replies before committing a state, so a misframed reply
        cannot pause a print.

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
            # A scan is a question we asked the unit. Until it answers, its bay
            # record is still the PREVIOUS spool's -- a bay the AMS already has
            # a record for reports that record from the instant a new spool goes
            # in, long before the reader has seen the new tag.
            #
            # So there are exactly three things a scan can be, and the unit
            # decides which: still working, read a tag, or finished with none.
            verdict = self._scan_verdict(slot)
            if verdict == "waiting":
                continue                         # it has not answered yet
            if verdict == "notag":
                # It finished and read nothing. Defaults, not leftovers -- and
                # once, not once per status frame, hence the latch. The hold
                # stays on deliberately: this bay's profile is the PREVIOUS
                # spool's until the spool comes out or a new scan is asked for,
                # so _surface_slot_info must not put it back over the defaults.
                #
                # AND NOTHING ELSE. There is no weight to let through: THE
                # UNIT'S OWN FIRMWARE WILL NOT MEASURE A SPOOL IT DID NOT READ
                # A BAMBU TAG ON. Not our gate and not one we can lift -- we
                # send the capacity choreography either way and the unit
                # declines. So anything the bay still reports for remain% or
                # nominal weight came off the PREVIOUS spool's tag, exactly
                # like the rest of the profile.
                if not self._scan_notag[slot]:
                    self._scan_notag[slot] = True
                    self._finalize_scan(slot)
                continue
            if verdict == "read":
                self._release_scan_hold(slot)    # the record is this spool's
                self._scan_notag[slot] = False
            self._surface_slot_info(lane, info)
            # A measurement finishes before the record it describes catches up
            # (see _queue_spool_summary). Now that the record has surfaced, the
            # summary can say what is actually in the bay.
            if getattr(self, "_pending_summary", None):
                self._drain_spool_summary(slot)

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

    def _restore_untagged_defaults(self) -> None:
        """
        At startup, give a bay that is present but carries no tag its defaults.

        Spools already in the unit at boot are deliberately never re-scanned
        (see ``_maybe_auto_scan``'s priming) -- a scan physically moves filament
        and a reboot must not do that. But applying DEFAULTS moves nothing, and
        without this a bay whose tag does not read comes back blank after a
        restart while a tagged bay comes back populated, because the tagged one
        re-derives its profile from the AMS record and the untagged one has no
        record to re-derive from.

        Only touches a lane that is genuinely empty: a Spoolman link, a restored
        profile, or an AMS record all leave it alone. If a different spool went
        in while Klipper was down and that one HAS a tag, ``_surface_slot_info``
        overwrites these defaults the moment the record arrives -- it overwrites
        a default on purpose, for exactly this reason.

        The "genuinely empty" test belongs here rather than in
        ``_finalize_scan``. Both paths end in "apply lane defaults" but answer
        opposite questions about existing data:

            boot restore   the lane has data -> IT IS THE USER'S, keep
            failed scan    the lane has data -> IT IS THE LAST SPOOL'S, clear

        A profile restored from saved vars is what this function exists to
        preserve; the same profile after a scan that read nothing is the
        previous spool's and must go. Sharing one guard between the two makes
        the scan path keep leftover records.
        """
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
            self._finalize_scan(slot)

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
        this bay. Without it the pre-scan blank is undone in the same pass --
        ``_surface_slot_info`` runs over the same pre-scan info dict and puts
        the old spool's profile straight back.

        Every scan goes through here, auto or manual, so both end the same two
        ways: the unit read a tag, or the lane gets defaults.

        :param slot: 0-based AMS slot index on this unit
        """
        self._scan_notag[slot] = False   # asking again -- the old answer is void
        t0 = getattr(self, "_scan_t0", None)
        if t0 is None or not (0 <= slot < len(t0)):
            return
        try:
            t0[slot] = self.afc.reactor.monotonic()
        except Exception:
            t0[slot] = None
            return
        # BACKSTOP ONLY. The scan normally resolves in _sync_lanes, which runs
        # on every status frame and asks _scan_verdict what the unit said -- so
        # a read reaches the lane within a frame of the unit announcing it, and
        # a cycle that ends with no tag falls back just as promptly.
        #
        # This callback exists for the one case that loop cannot cover: status
        # frames stopping (a wedged bridge, a unit that went quiet mid-cycle).
        # It is not a scan window and must not be tuned like one. Tightened to
        # a per-model read time it would expire on tags still in flight and
        # announce them as "no readable tag".
        try:
            self.afc.reactor.register_callback(
                lambda et, s=slot: self._scan_timeout(s),
                t0[slot] + self.SCAN_FALLBACK_CAP + 1.0)
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

        THE WHOLE SCAN STATE MACHINE. We command a scan, the unit carries it
        out and narrates what happened; this reads that narration back. There
        is no inspection of the record's CONTENT here and no guessing from a
        clock -- both were tried, and both got it wrong in ways that took days
        to unpick. The unit is the only thing that knows whether it read a tag,
        so it is the only thing asked.

        ``"none"``     no scan is open; the record is just the bay's, use it.
        ``"waiting"``  commanded, no answer yet. The bay still reports the
                       PREVIOUS spool's record until the reader sees the new
                       tag, so nothing may be surfaced during this.
        ``"read"``     the unit narrated a successful read. Its record is now
                       this spool's -- even when it is byte-for-byte the old
                       one, which is exactly what re-inserting the same spool
                       produces.
        ``"notag"``    the unit narrated the END of its cycle without a read.
                       That is the honest moment "no tag" becomes a fact.

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
        bridge = getattr(self, "_bridge", None)
        if bridge is None:
            return "notag"           # nothing can answer; do not wait for it
        dev = getattr(self, "dry_dev_addr", 0) or None
        try:
            if bridge.rfid_read_succeeded_since(started, addr=dev):
                return "read"
            if bridge.rfid_cycle_ended_since(started, addr=dev):
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

    def _is_ht(self) -> bool:
        """True if this unit is an AMS HT (device 0x1800). The HT scans its RFID
        itself on its preload switch, so the firmware -- not the module -- drives
        the scan (armed on the insert edge)."""
        return bool(self.has_heater) and getattr(self, "dry_dev_addr", 0) == 0x1800

    def _send_ht_flag(self, bridge: Any) -> None:
        """
        Tell the firmware whether this unit is an AMS HT, so it arms the RFID
        scan on the slot's insert edge (device 0x1800).

        Harmless for a boxed AMS. Also enables the dense-0F follower hold for
        an HT, which holds mode:4 smoothly under a poop without a re-arm tick
        or feed starvation. Re-asserted here so it survives a Pico reboot.

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
            # single door into the measurement window for EVERY unit type --
            # an HT reaches it from ht_scan_arm() on the insert edge, where the
            # module never gets a say. Re-asserted here so it survives a Pico
            # reboot, exactly like the HT flag above.
            bridge.send({"cmd": "capen", "unit": self.ams_index,
                         "on": 1 if getattr(self, "measure_on_insert", True)
                         else 0})
            # Class registry (gap 2 groundwork, INERT today). A real printer
            # enrolls HTs into 0x80-0x87 and boxed units into 0x00-0x03, which
            # it can do because it knows what a unit IS before it hands out an
            # address. We cannot -- htunit above takes an INDEX, and the index
            # does not exist until enrollment has happened. The UID is the one
            # identifier known first, and it is already in printer.cfg, so push
            # it and let enrollment pick the range once the index/address split
            # lands. Registering changes nothing on the bus today.
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

        This is the thing a real printer has and we did not. Across every
        capture, one UID sits at one bus id forever -- the roll-call walk IS
        that table being re-asserted. We assigned by ANNOUNCE ORDER instead, so
        the chain shuffled across reboots and, after one relink, two boxed units
        landed on the SAME index, both tried to scan a spool at once, and
        Klipper crashed.

        The order here is deterministic and comes from config, not from the bus:
        boxed units first in config-name order, then HTs. That also puts the
        classes in the ranges class addressing needs (boxed 0-3, HT 4-11), so
        the two features agree by construction instead of racing each other.

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
                units.append((is_ht, name, uid))
            if not units:
                return
            # Plain sequential order: boxed first, then HTs, numbered 0..N-1.
            # NOT boxed 0-3 / HT 4-11 -- that is the CLASS layout, and with
            # class addressing off index 4 maps to bus id 0x04, an address no
            # real printer ever uses. The firmware does the class placement
            # itself when the toggle is on, because only it knows the toggle.
            # What the host owns is the ORDER, which is what was missing.
            units.sort()                      # boxed (False) first, then by name
            for i, (_, _, uid) in enumerate(units):
                if i < 12:
                    bridge.send({"cmd": "bind", "uid": uid, "idx": i})
        except Exception:
            pass

    def _send_rc_span(self, bridge: Any) -> None:
        """
        Tell the firmware how much of the address space to roll-call.

        A real printer walks all twelve ids forever -- 4 boxed (0x00-0x03) and 8
        HT (0x80-0x87) -- whether or not anyone has ever answered, which is how
        it finds a hot-plugged unit within a second. That is also expensive: an
        id with nobody on it burns the full 8ms reply timeout, so on a typical
        bus most of the register is spent asking unanswerable questions.

        So the span is derived from the units actually CONFIGURED, per class,
        plus headroom. Headroom is the point: sending the exact count would make
        a newly added unit undiscoverable, which trades a real capability for a
        few milliseconds. One spare id per class keeps hot-plug working for the
        next unit you add, and you only pay for it once per cycle.

        Set rollcall_span_boxed / rollcall_span_ht to pin it by hand; 0 means
        "all of that class", the printer-faithful behaviour.

        :param bridge: the bridge to send on
        """
        if bridge is None:
            return
        try:
            boxed = getattr(self, "rollcall_span_boxed", None)
            ht = getattr(self, "rollcall_span_ht", None)
            # OPT-IN ONLY. Nothing is sent unless one of the options is set, so
            # the firmware keeps its printer-faithful default of all twelve
            # ids. Deriving a span automatically was written and deliberately
            # backed out: it is a real behaviour change (a unit outside the span
            # is never discovered) and it belongs behind an explicit decision,
            # not switched on for everyone by a config file they did not edit.
            if boxed is None and ht is None:
                return
            bridge.send({"cmd": "rcspan",
                         "boxed": int(boxed or 0), "ht": int(ht or 0)})
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

    def _forget_spoolman_miss(self, slot: int) -> None:
        """
        Drop the "Spoolman has no spool for this UID" memo for a slot.

        Called when a spool leaves the bay: the next one deserves its own
        lookup, and the same spool re-inserted after being added to Spoolman
        must be able to bind.

        :param slot: 0-based AMS slot index
        """
        try:
            miss = getattr(self, "_spoolman_no_match", None)
            if not miss:
                return
            info = (self._slots or [None] * self.SLOTS_PER_UNIT)[slot]
            uid = (info or {}).get("uid") or (info or {}).get("tray_uid")
            if uid:
                miss.discard(uid)
        except Exception:
            pass

    def _unbind_spool(self, lane: Any, reason: str = "the bay is empty") -> None:
        """
        Drop a lane's Spoolman link.

        Used when the physical spool leaves the bay: a binding to an empty bay
        is stale, and -- worse -- Spoolman-linked lanes are treated as
        authoritative elsewhere, so a stale one BLOCKS the next real tag from
        applying. Observed live: lane23 kept showing another unit's colour
        because it was still bound to spool 124.

        Also used when a scan finishes without reading a tag, where the binding
        is the PREVIOUS spool's claim on a bay that now holds something else --
        same staleness, same consequence for the next scan. ``reason`` says
        which, because "unbinding, the bay is empty" against an occupied bay
        reads as a bug in the presence detection.

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
        except Exception as e:
            self.logger.debug(
                f"AFC bambu {self.name}: could not unbind {lane}: {e}")

    def _reconcile_empty_bays(self) -> None:
        """
        Clear any lane whose bay the unit reports EMPTY, once at startup.

        AFC restores lanes from saved vars, and until this runs nothing checks
        those against the hardware -- so a spool pulled while Klipper was down
        leaves its material, colour and Spoolman link on the lane forever. The
        insert/removal edges cannot fix it either: there is no edge, the bay was
        already empty when we started watching.

        Observed live: AMS 1 bays 2 and 4 still showed filament (one bound to
        spool 130) with the unit reporting only bay 1 present.

        The unit's presence bits are the truth here, so this runs at priming --
        after the firmware has reported presence and AFC has restored the lanes,
        which is exactly when "still blank" and "still full" both mean it.
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
        bay switch. Bounded by SCAN_FALLBACK_CAP so a stuck scan cannot mask a
        real removal forever.

        :param slot: 0-based AMS slot index
        :return bool: True while this slot's scan is still within its window
        """
        # Deliberately NOT _scan_t0. That timestamp means "waiting for a tag"
        # and is cleared by _release_scan_hold the moment the read succeeds --
        # which is BEFORE the unit finishes retracting the filament. Hanging
        # the guard on it left exactly the window that matters unguarded: read
        # OK -> hold released -> unit pulls the filament back off the bay
        # switch -> "REMOVED" -> "INSERTED" -> scan again. Those are two
        # different lifetimes, so this gets its own clock, cleared only by
        # expiry.
        t0s = getattr(self, "_scan_motion_t0", None)
        if not t0s or not (0 <= slot < len(t0s)):
            return False
        started = t0s[slot]
        if not started:
            return False
        # THE UNIT SAYS WHEN IT IS DONE. Every model announces the end of its
        # scan/measure cycle -- "Calibration rst:0" (HT), "odom calib success
        # exit 0" (AMS 1), "STEP7:cali end" (AMS 2) -- so wait for that rather
        # than running a clock beside it. A timer is wrong in both directions:
        # too short and the unit's own retract re-triggers the scan, which
        # loops every 38-42 s; too long and a real removal goes unnoticed for
        # a minute and a half.
        try:
            getend = getattr(self._bridge, "last_scan_end", None)
            ended = getend() if callable(getend) else None
            if ended is not None and ended >= started:
                t0s[slot] = None        # the unit finished; stop guarding
                return False
        except Exception:
            pass
        # Backstop ONLY, for a unit that never announces an end -- a scan that
        # dies mid-cycle must not gate this bay's presence forever.
        try:
            now = self.afc.reactor.monotonic()
        except Exception:
            return False
        if (now - started) >= self.SCAN_MOTION_QUIET_S:
            t0s[slot] = None
            return False
        return True

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
        # A SCAN MOVES THE FILAMENT PAST THE BAY SWITCH, so the unit reports
        # the bay empty in the middle of its own scan. Treating that as a
        # removal starts a self-sustaining loop: scan -> filament retracts ->
        # "REMOVED" -> filament returns -> "INSERTED" -> scan again, forever.
        # Observed live on an AMS 2 bay 3, re-scanning every ~30 s indefinitely.
        #
        # So while a scan is in flight for THIS slot, track presence silently:
        # no edge, no log, no new scan. The window is bounded by
        # SCAN_FALLBACK_CAP, so a spool genuinely pulled mid-scan is still
        # noticed once the scan gives up -- this defers the edge, it does not
        # discard it.
        inflight = getattr(self, "_scan_in_flight", None)
        if callable(inflight) and inflight(slot):
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
                if lane is not None:
                    # Clear REGARDLESS of a Spoolman binding. Treating a
                    # Spoolman-linked lane as "authoritative" and skipping it
                    # leaves a bound lane claiming filament for a bay the unit
                    # reports EMPTY -- observed live on lane18 (spool 130) and
                    # lane23 (spool 124), where the stale binding then blocked
                    # the next real tag from applying at all.
                    #
                    # A removal is a physical fact the unit reported. A binding
                    # to an empty bay is stale by definition, so the spool link
                    # goes with the filament -- a re-insert re-binds from the
                    # tag, which is the normal path anyway.
                    self._clear_lane_filament(lane)
                    self._unbind_spool(lane)
                # Drop the fresh-measure override for this slot -- it belonged
                # to the spool just removed. Without this it would win over the
                # NEXT spool's record until that spool is itself measured.
                mr = getattr(self, "_measured_remain", None)
                if isinstance(mr, dict):
                    mr.pop(slot, None)
                # And any summary still waiting on a record that is now gone --
                # it described the spool that just came out.
                ps = getattr(self, "_pending_summary", None)
                if isinstance(ps, dict):
                    ps.pop(slot, None)
                # And forget which tag this bay's binding was made from -- the
                # binding is gone, so a record of it would only make the next
                # spool look "already bound by this tag".
                bu = getattr(self, "_bound_uid", None)
                if isinstance(bu, dict):
                    bu.pop(slot, None)
                # PERSIST IT. A REMOVAL IS A PHYSICAL FACT.
                #
                # This edge cleared the lane and dropped the Spoolman link in
                # MEMORY ONLY -- nothing here saved. So a spool pulled and then
                # a Klipper restart before anything else wrote vars brought the
                # whole record back: AFC restores lanes from the var file, and
                # the file still named the departed spool.
                #
                # spool_id is the one that matters. filament_name is not written
                # to the file at all (AFC_lane only emits it when NOT saving), so
                # it is re-derived after a restart -- and a restored spool_id is
                # what it gets re-derived from. Clearing the link in memory and
                # not saving leaves the file able to resurrect both.
                #
                # _finalize_scan already saves on the no-tag path; this covers
                # the gap before it, and the case where no insert follows.
                #
                # Best-effort, like everything else on this edge: a removal is
                # reported by the hardware and the bookkeeping that follows it
                # must never be able to raise. Losing the save costs a stale
                # record after a restart; raising here would abandon the rest of
                # the edge, including the auto-scan re-arm below.
                try:
                    self._save_lane_vars()
                except Exception:
                    pass
            self._auto_scanned[slot] = False        # reinsertion re-scans
            self._scan_notag[slot] = False          # a new spool is a new answer
            defer = getattr(self, "_scan_defer", None)
            if defer is not None and 0 <= slot < len(defer):
                defer[slot] = False                 # nothing left to replay
            self._release_scan_hold(slot)
            return
        # A scan held back because a lane on this unit was at the toolhead.
        # Replay it the moment that stops being true -- the spool is still
        # sitting there untagged, and the edge that would have triggered it has
        # long since passed.
        defer = getattr(self, "_scan_defer", None)
        if defer is not None and 0 <= slot < len(defer) and defer[slot]:
            if _unit_tool_loaded(self):
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
        if (not self.auto_scan or was_present
                or self._auto_scanned[slot] or self._bridge is None):
            return
        afc = getattr(self, "afc", None)
        try:
            if afc is not None and afc.function.in_print():
                return                               # never move filament mid-print
        except Exception:
            pass
        # A lane on this unit is threaded to the toolhead: hold the scan.
        #
        # A tag read is not a passive operation. The AMS feeds the new spool past
        # its bay reader and pulls it back -- "STEP2:from Five pull tray N to
        # switch", "rfid pull back" -- and it runs that cycle for the whole unit,
        # which makes the follower pulse for the best part of a minute. With a
        # lane from the SAME unit loaded to the toolhead, that is filament motion
        # nobody asked for against a threaded path.
        #
        # in_print() below does not cover this: loaded-but-idle is not a print,
        # and that is exactly when someone refills the other bays. Observed on
        # BambuAMS_1 with lane16 at the toolhead and a spool put into bay 4.
        #
        # Held, not dropped: the edge is consumed so it cannot re-fire every
        # poll, and _scan_defer replays it once the unit is free. An HT is exempt
        # because its scan is firmware-driven on its own preload switch -- there
        # is nothing here to withhold -- and it has one lane, so the situation
        # cannot arise.
        if not self._is_ht() and _unit_tool_loaded(self):
            self._auto_scanned[slot] = True
            defer = getattr(self, "_scan_defer", None)
            if defer is not None and 0 <= slot < len(defer):
                defer[slot] = True
            self.logger.info(
                f"AFC bambu {self.name}: spool in slot {slot} will be scanned "
                f"when the unit is free -- a lane here is loaded to the toolhead "
                f"and a scan would move filament on it")
            return
        afcBambuAMS._start_tag_scan(self, slot, info)

    def _start_tag_scan(self, slot: int, info: dict) -> None:
        """
        Begin a tag read for a slot, and arm the defaults fallback behind it.

        Shared by _maybe_auto_scan and the deferred retry so both run the
        identical sequence -- a held scan must take the same path, or the old
        spool's data survives.

        :param slot: 0-based AMS slot index on this unit
        :param info: Normalized slot info as of the insert
        """
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
        self._open_scan(slot)
        # The motion guard's own clock -- see _scan_in_flight. Set unconditionally
        # (not inside the stale-snapshot branch above), because the filament
        # moves whether or not a pre-scan profile was recorded.
        mt = getattr(self, "_scan_motion_t0", None)
        if mt is not None and 0 <= slot < len(mt):
            try:
                mt[slot] = self.afc.reactor.monotonic()
            except Exception:
                mt[slot] = None
        # The AMS HT scans its own tag on its preload switch -- the FIRMWARE arms
        # the 0x1800 poll the instant it sees the insert edge (no module round-
        # trip, no settle), which is the only way to catch the HT's scan while it
        # runs. So for the HT we do NOT send a scan here (that would just read its
        # stale flash and could clobber the firmware's min-window). The boxed AMS
        # scans by feeding past its bay reader, which the module drives via scan().
        # Both branches restate the INSERTED line above with the mechanism that
        # scans -- useful when a tag doesn't turn up, not on every insert.
        if self._is_ht():
            # The HT's scan AND its capacity window are both armed in firmware
            # on this edge (ht_scan_arm -> cap_open), so the module sends
            # nothing -- but it must still MARK the slot, or the measurement
            # that follows has nothing to attribute itself to.
            #
            # Without this the HT measured correctly and the result was thrown
            # away: the whole apply path is gated on _cap_pending_slot, which
            # only _start_capscan sets, and the HT branch never calls it. On
            # the wire the unit reported "odom C:0.522,R:0.083,P:102%" and
            # "Calibration rst:0"; in AFC the lane never changed.
            self._cap_pending_slot = slot
            try:
                self._cap_pending_t0 = self.afc.reactor.monotonic()
            except Exception:
                self._cap_pending_t0 = 0.0
            self.logger.debug(
                f"AFC bambu {self.name}: new spool in slot {slot}; HT scans and "
                f"measures it on insert (firmware-driven at 0x1800)")
        else:
            self.logger.debug(
                f"AFC bambu {self.name}: new spool detected in slot {slot}, "
                f"scanning tag")
            # The capacity choreography, not the burst scan. A physical insert
            # arms exactly ONE autonomous preload, and what rides on it depends
            # on what gets sent while it runs: the burst's selects/RFID reads
            # reset the odometer and the AMS stops at "cali tray"; the
            # printer's trigger+probe+enable (bb_do_capscan) lets the same
            # preload also measure the spool and persist remain%. Same tag
            # read either way -- the info fill collects the record after. If
            # no readable tag lands, _finalize_scan applies defaults, and
            # AFC_BAMBU_SCAN remains available for a manual burst.
            # The capacity choreography on every insert: one tag read, then
            # the spool measurement (verified live: 78-86% across repeated
            # runs of the same part-used spool, the measured percent applied
            # to the slot and the lane's grams). The pending-slot marker is
            # what attributes the narrated result to this bay.
            # measure_on_insert is enforced in the FIRMWARE (cap_open), not
            # here: that is the one door into the measurement window for every
            # unit type, including an HT whose window is armed on the insert
            # edge without the module being involved. Branching here would
            # cover boxed units only.
            if not afcBambuAMS._start_capscan(self, slot):
                self.scan(slot)

    def _clear_lane_filament(self, lane: Any) -> None:
        """
        Blank a lane's filament profile so a previous spool's data doesn't linger
        in the UI until a fresh tag reads or defaults are applied. Best-effort
        per attribute.

        EVERY FIELD A TAG SETS HAS TO BE HERE, and filament_name is the one that
        got missed. It is the field the Mainsail card actually DISPLAYS, and
        apply_filament_defaults does not write it -- it sets material, color,
        weight, sub_type and spool_vendor, so a defaulted lane keeps whatever
        name the last tag left. Measured on an untagged insert into a bay that
        had held Bambu PLA Matte:

            firmware slot record   material:"" uid:"" color:00000000   (correct)
            lane after defaults    material:'PLA'  color:''  spool_id:None
                                   filament_name:'Bambu PLA Matte'     <- stale

        The whole "two outcomes, never three" rule was already working -- the
        removal unbound, the scan finalised, defaults applied. It just left one
        field behind, and that field is the one the operator sees.

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

    def _finalize_scan(self, slot: int) -> None:
        """
        The no-tag outcome: give the lane its AFC defaults.

        Called when the unit has FINISHED a scan without reading a tag (see
        ``_scan_verdict``), and at boot for a present bay that carries no
        record at all (``_restore_untagged_defaults``). It does not decide
        whether a tag read -- that is the unit's to say, and asking it is
        ``_scan_verdict``'s single job.

        :param slot: 0-based AMS slot index on this unit
        """

        if apply_filament_defaults is None:
            return
        if not (0 <= slot < len(self._slots)):
            return
        info = self._slots[slot]
        if not info or not info.get("present"):
            self._release_scan_hold(slot)
            return                                   # spool gone
        # TWO OUTCOMES, NEVER THREE. A scan either read a tag -- and then
        # _surface_slot_info applies the unit's record -- or it did not, and
        # then the lane gets DEFAULTS. There is no third outcome in which the
        # previous spool's material, colour, weight or Spoolman link is left
        # sitting on the lane because clearing it looked risky.
        #
        # Two tempting early returns would create exactly that third outcome:
        # skipping a Spoolman-linked lane as "authoritative", or skipping any
        # lane that already had a material. Both conditions are TRUE OF THE
        # PREVIOUS SPOOL, which is the only thing that could have set them --
        # the scan we are finalising found nothing. So they read as "this lane
        # already has good data" when what they actually mean is "this lane
        # still has the last spool's data", and they preserved it.
        #
        # Leaving the link is not the cautious option either. The removal edge
        # already unbinds for a measured reason: a stale binding is treated as
        # authoritative elsewhere and BLOCKS THE NEXT REAL TAG FROM APPLYING AT
        # ALL -- observed live on lane18 (spool 130) and lane23 (spool 124).
        # Keeping it through a failed scan breaks the following scan too.
        lane = self._lane_for_slot(slot)
        if lane is None:
            self._release_scan_hold(slot)
            return
        # Blanked here rather than relied on from the insert edge, because the
        # status loop may have re-armed it in between. The Spoolman link goes
        # with the filament, exactly as it does on a removal: a binding that
        # outlives the spool it named is the previous spool's claim on this bay.
        self._clear_lane_filament(lane)
        self._unbind_spool(
            lane, "the scan read no tag, so this link is the previous spool's")
        afc = getattr(self, "afc", None)
        # Hand the helper an info with the profile stripped. The unit read no
        # tag, so any profile still in this bay's record is the PREVIOUS
        # spool's -- and the helper prefers slot_info's own material/color over
        # the AFC defaults, which would put it straight back on the lane we
        # just blanked, the same value by a different route.
        info = {k: v for k, v in info.items()
                if k not in ("material", "sku", "color", "color_hex",
                             "temp_min", "temp_max", "weight",
                             "extruder_temp", "bed_temp")}
        try:
            apply_filament_defaults(
                lane, info,
                afc_defaults={
                    "default_material_type": getattr(
                        afc, "default_material_type", None),
                    "default_color": getattr(afc, "default_color", None),
                })
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
            # SAY WHAT ACTUALLY HAPPENED. "The bay reader saw no chip" is
            # simply wrong for a third-party spool: the unit saw one and could
            # not open it, and it says so --
            #     STEP:stop goto auth / STEP:auth fail:-4
            #     STEP7:info_valid 0 or bbl:-1        (bbl = Bambu Lab)
            # Those are different problems with different fixes, and telling an
            # operator their reader is blind when it is working is worse than
            # saying nothing.
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
            elif _foreign:
                _why = (" -- the bay HAS a tag, but its keys are not Bambu's "
                        "so the unit could not read the profile "
                        "(auth failed). A third-party spool: set the material "
                        "on the lane, or bind it in Spoolman by hand")
            else:
                _why = " (and no tag UID either -- the bay reader saw no chip)"
            self.logger.info(
                f"AFC bambu {self.name}: no readable tag profile in slot "
                f"{slot}; applied lane defaults to {lane.name}" + _why)
            # Persist them. Without this the defaults live only in the lane
            # object: measured on hardware, a bay whose tag does not read showed
            # PLA/1000g right up to a Klipper restart and came back blank,
            # because AFC restores lanes from vars and the saved record still
            # held the pre-default None/0. A tagged bay hid the gap -- its
            # profile is re-derived from the AMS record every boot -- so only
            # the untagged bay, which has no record to re-derive from, lost it.
            self._save_lane_vars()
        except Exception as e:
            self.logger.warning(
                f"AFC bambu {self.name}: default apply for slot {slot} "
                f"failed: {e}")

    def _spoolman_slot_info(self, info: dict) -> dict:
        """
        Translate a bridge slot dict into the shape sync_rfid_to_spoolman and
        find_spool_by_uid expect (the same dict every other AFC reader builds).

        The match key is "uid" -- the 4-byte Mifare chip UID -- so a spool
        registered on ANY reader on this printer (OpenAMS, ACE2, U1) matches
        here, and vice versa. tray_uid rides along for richness.

        :param info: normalized bridge slot info
        :return dict: Spoolman-shaped slot_info
        """
        material, sub_type = _split_bambu_material(info.get("material") or "")
        color = info.get("color")
        color_hex = ((color if color.startswith("#") else "#" + color)
                     if color else None)
        try:
            w = int(info.get("weight")) if info.get("weight") else 1000
        except (TypeError, ValueError):
            w = 1000
        si = {
            "uid": info.get("rfid_uid") or "",
            "brand": BAMBU_BRAND,
            "material": material or "",
            "sub_type": sub_type or "",
            "color_hex": (color_hex or "").lstrip("#") or None,
            "diameter": 1.75,
            "extruder_temp": info.get("temp_min"),
            "extruder_temp_min": info.get("temp_min"),
            "extruder_temp_max": info.get("temp_max"),
            "weight_g": w,
        }
        if info.get("tray_uid"):
            si["tray_uid"] = info["tray_uid"]
        return si

    def _remember_bound_uid(self, slot: Optional[int], uid: str) -> None:
        """
        Record that this slot's Spoolman binding was made from tag ``uid``.

        Best-effort: a diagnostic bookkeeping entry must never be able to break
        a bind that already succeeded.

        :param slot: 0-based AMS slot index, or None when unknown
        :param uid: the tag UID the binding was made from
        """
        if slot is None or not uid:
            return
        try:
            if getattr(self, "_bound_uid", None) is None:
                self._bound_uid = {}
            self._bound_uid[slot] = uid
        except Exception:
            pass

    def _spoolman_sync(self, lane: Any, info: dict) -> None:
        """
        Bind this lane's spool to Spoolman by tag UID, creating it if allowed.

        Two paths, and the UID is the key to both:
          - FULL decode (material known): sync_rfid_to_spoolman binds an
            existing spool by UID or, with auto-create on, makes a new
            filament+spool from the tag's own values.
          - UID-ONLY (a good UID but no usable profile -- a foreign tag the AMS
            surfaced a chip UID for but could not decode): match by UID ALONE
            and bind if Spoolman already knows it. Never create from nothing.

        Silent no-op without Spoolman configured or without a UID. A lane that
        already carries a spool_id is left alone -- a manual/prior binding wins.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (must carry rfid_uid)
        """
        uid = info.get("rfid_uid")
        if not uid or lane is None:
            return
        # AN EMPTY BAY HAS NO TAG TO BIND. The unit keeps a bay's UID in its
        # record after the spool leaves -- the removal edge clears presence, not
        # the tag fields -- so without this the leftover UID re-bound the lane
        # 66 ms after the removal unbound it:
        #
        #   02:22:21  spool REMOVED from slot 0
        #   02:22:21  unbinding lane23 from spool 87 -- the bay is empty
        #   02:22:21  matched lane23 to Spoolman spool 87 by UID 0a1882ac
        #
        # and the lane went into the next insert already welded to the departed
        # spool.
        if not info.get("present"):
            return
        slot = info.get("index")
        bound = getattr(lane, "spool_id", None)
        if bound not in (None, "", 0):
            prev = (getattr(self, "_bound_uid", None) or {}).get(slot)
            if prev is None:
                return           # not bound by a tag read -- someone else's call
            if prev == uid:
                return           # already bound BY THIS TAG: nothing to do
            # A DIFFERENT TAG IS IN THIS BAY. The binding names the previous
            # spool, so it is a leftover record like any other -- and holding it
            # is not passive: every measurement this bay produces gets written
            # to the wrong Spoolman spool (810 g of PLA Basic onto the PLA Glow
            # reel, measured). The tag identifies the spool; the binding follows.
            self._unbind_spool(
                lane, f"tag {str(uid).upper()} is in this bay now, not "
                      f"{str(prev).upper()}")
        afc = getattr(self, "afc", None)
        if afc is None or getattr(afc, "spoolman", None) is None:
            return
        si = self._spoolman_slot_info(info)
        have_profile = bool(si.get("material"))
        # A UID Spoolman does not know is a PERMANENT answer, not a retry.
        #
        # This runs on every status pass. Without a memo, a spool whose tag has
        # no Spoolman entry (and auto-create off) re-queries Spoolman once per
        # second, forever -- a blocking HTTP call on the reactor. Observed live:
        # ~20 minutes of "no Spoolman spool matches UID ECB61CD0" at 1 Hz, 1061
        # "Resetting prediction variance" events as the host lost its MCU clock,
        # then "MCU 'mcu' shutdown: Timer too close" and every MCU down. The
        # lookup did not fail -- it succeeded, and the answer was "no match".
        #
        # Keyed by UID so it is self-invalidating: a different spool has a
        # different UID and gets its own lookup. Cleared on removal, so
        # re-inserting the same spool after adding it to Spoolman re-checks.
        if uid:
            miss = getattr(self, "_spoolman_no_match", None)
            if miss is None:
                miss = self._spoolman_no_match = set()
            if uid in miss:
                return
        try:
            if have_profile and sync_rfid_to_spoolman is not None:
                allow = False
                if get_auto_spoolman_create is not None:
                    allow = get_auto_spoolman_create(
                        lane, getattr(self, "auto_spoolman_create", False))
                sync_rfid_to_spoolman(afc, lane, si, self.logger,
                                      "Bambu RFID", allow_create=allow)
                # Bound? Then it matched. Still unbound means Spoolman has no
                # spool for this UID -- remember it so the next status pass
                # does not ask again.
                if uid and getattr(lane, "spool_id", None) in (None, "", 0):
                    self._spoolman_no_match.add(uid)
                else:
                    self._remember_bound_uid(slot, uid)
            elif find_spool_by_uid is not None:
                # UID-only: bind if Spoolman already carries this UID.
                client = _bambu_spoolman_client(afc)
                spool = find_spool_by_uid(client, uid) if client else None
                if spool and afc.spool is not None:
                    afc.spool.set_spoolID(lane, spool.get("id"))
                    self._remember_bound_uid(slot, uid)
                    self.logger.info(
                        f"AFC bambu {self.name}: matched {lane.name} to "
                        f"Spoolman spool {spool.get('id')} by UID {uid} "
                        f"(no tag profile decoded)")
                elif uid:
                    # THE MEMO BELONGS ON BOTH BRANCHES. This path is a bay with
                    # a readable UID whose profile has not landed yet, which is
                    # EVERY scan for its first seconds. Without the memo it
                    # re-queries Spoolman on every status pass, blocking the
                    # reactor in HTTP each time -- the exact loop the note above
                    # describes, whose measured cost is 1061 "Resetting
                    # prediction variance" events and an MCU shutdown, with lane
                    # data, Mainsail and the panel all queued behind it.
                    self._spoolman_no_match.add(uid)
        except Exception:
            self.logger.debug("AFC bambu: spoolman sync failed",
                              exc_info=True)

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

    def _apply_remain_weight(self, lane: Any, info: dict) -> None:
        """
        Set a lane's weight from the AMS remain% (nominal x remain%) and push
        the correction to Spoolman -- for ANY present lane, bound or not.

        The AMS's physical remain% is authoritative over Spoolman's stored
        value, so a bound spool that Spoolman thinks is nearly full (998 g)
        shows the measured 80% (800 g) instead. Skipped while the lane feeds
        the toolhead -- there its weight is being decremented by extrusion and
        must not be stomped every status frame.

        :param lane: the AFC lane
        :param info: normalized bridge slot info (remain_pct, weight)
        """
        if lane is None or getattr(lane, "tool_loaded", False):
            return
        rp = info.get("remain_pct")
        # THE MEASUREMENT BEATS THE TAG RECORD, which is the same rule
        # get_status already applies to remain_pct -- and it was missing here,
        # so this function undid the measurement seconds after it landed.
        # Captured on an AMS 1 insert, spool #87, three writes in 106 ms:
        #
        #   14:03:27  wrote 700 g            (tag record, 70%)
        #   14:03:40  measured ... 69% (~690 g) [capscan]
        #   14:03:40  wrote 690 g            (the measurement)
        #   14:03:40  wrote 700 g            (THIS function, tag record again)
        #
        # The physical measurement is what the unit just did with a ruler; the
        # tag record is what some previous life wrote down. Whenever they
        # disagree the ruler wins, and it has to win HERE too or the last write
        # is the stale one.
        idx = info.get("index")
        measured = (getattr(self, "_measured_remain", None) or {}).get(idx)
        # Remember WHICH of the two produced the number, so the write can say
        # so. Announcing a tag record as a physical measurement is the machine
        # claiming work it did not do.
        src = "the spool's tag record, not a fresh measurement"
        if isinstance(measured, int) and measured > 0:
            rp = measured
            src = "physical AMS measurement"
        if not (isinstance(rp, int) and rp > 0):
            return
        tag_w = info.get("weight")
        try:
            nominal = int(tag_w) if tag_w else 1000
        except (TypeError, ValueError):
            nominal = 1000
        # A SPOOL CANNOT HOLD MORE THAN ITS OWN NOMINAL WEIGHT, so cap the
        # percentage before it becomes grams. Measured on a real printer with
        # our hardware off the bus entirely -- the SAME HT spool, eight minutes
        # apart:
        #
        #     16:47:13  C:0.531  R:0.084  P:107%  od:1.132
        #     16:55:28  C:0.551  R:0.088  P:119%  od:1.143
        #
        # The radius went UP by 3.5mm while filament was being consumed, which
        # cannot happen, so at least one reading is wrong by 12 points. The AMS
        # derives R from a circumference sampled over od/C = ~2.1 SPOOL
        # REVOLUTIONS, and two turns is a thin basis for a circumference.
        #
        # This is the unit's own arithmetic and we cannot improve it -- but
        # 119% of a 1kg spool is 1190g, and publishing that to Spoolman as a
        # measured weight is the machine stating something impossible.
        if rp > 100:
            # ONCE PER READING, NOT ONCE PER STATUS FRAME. This function runs
            # on every frame, and the cap notice sat outside the "did anything
            # change" guard below -- so an HT measuring 113% printed this line
            # about once a second, for as long as the spool stayed above 100%.
            # Measured: 38 identical lines in 38 seconds, drowning the console.
            #
            # The value is still worth saying -- it is the unit reporting a
            # spool proud of the reference radius -- so it is deduped by lane
            # and value rather than dropped: a NEW reading still speaks up.
            seen = getattr(self, "_cap_notice", None)
            if seen is None:
                seen = self._cap_notice = {}
            key = getattr(lane, "name", "?")
            if seen.get(key) != rp:
                seen[key] = rp
                self.logger.debug(
                    f"AFC bambu {self.name}: {key} measured {rp}% remaining; "
                    f"capping at 100% (the AMS samples ~2 spool revolutions, "
                    f"+/-12 points observed)")
            rp = 100
        else:
            # Forget the notice once the reading comes back in range, so the
            # next excursion is reported instead of being suppressed forever.
            try:
                (getattr(self, "_cap_notice", None) or {}).pop(
                    getattr(lane, "name", "?"), None)
            except Exception:
                pass
        w = max(1, (nominal * rp) // 100)
        if int(getattr(lane, "weight", 0) or 0) != w:
            lane.weight = w
            self._push_measured_to_spoolman(lane, w, src)

    def _push_measured_to_spoolman(self, lane: Any, grams: int,
                                   source: str = "") -> None:
        """
        Write a remaining-weight figure back to the lane's Spoolman spool.

        Two things produce that figure and they are NOT the same claim:

          measurement  the AMS physically pulled the spool and derived a radius
                       (P:NN% -> grams). A real reading, and the reason this
                       write exists.
          tag record   the percentage written on the tag in some previous life.
                       Better than nothing, but nothing was measured now.

        ``source`` records which, so the log cannot announce a tag record as a
        physical measurement -- the machine asserting work it did not do.

        Applies to any lane bound to a Spoolman spool, tagged or not, which is
        also what stops a no-tag spool from looking full when it is not.

        Gated by sync_measured_to_spoolman (default on). No-op without Spoolman
        or a bound spool.

        :param lane: the AFC lane
        :param grams: remaining net weight, grams
        :param source: where the figure came from, for the log
        """
        if not getattr(self, "sync_measured_to_spoolman", True):
            return
        if lane is None:
            return
        sid = getattr(lane, "spool_id", None)
        if sid in (None, "", 0):
            return
        afc = getattr(self, "afc", None)
        client = _bambu_spoolman_client(afc)
        setter = getattr(client, "set_remaining_weight", None)
        if client is None or not callable(setter):
            return
        try:
            setter(sid, float(grams))
            self.logger.info(
                f"AFC bambu {self.name}: wrote {grams} g remaining to "
                f"Spoolman spool {sid} "
                f"({source or 'physical AMS measurement'})")
        except Exception:
            self.logger.debug("AFC bambu: spoolman weight write failed",
                              exc_info=True)

    def _surface_slot_info(self, lane: Any, info: dict) -> None:
        """
        Apply the AMS tag's PROFILE to a lane, base-ACE style: material, color,
        Bambu type code, and print temps — only when the lane doesn't already
        have them (a manual/Spoolman value wins). The tag's unique UID (the
        Mifare chip UID, confirmed against an OpenAMS read) drives Spoolman
        binding/creation via _spoolman_sync.

        :param lane: The AFC lane object
        :param info: Normalized slot info from bridge_slot_to_info
        """
        tag_material = info.get("material")
        if tag_material and tag_material.lower() == "unknown":
            tag_material = None

        # NO "IS THIS LANE ALREADY BOUND?" GATE. A lane with a spool_id gets
        # the tag applied like any other. Spoolman is a RECORD of what is in
        # the bay; the tag is the bay. Gating on the link would leave a bound
        # lane's material and colour coming from Spoolman on Spoolman's
        # schedule rather than from the tag, and no other reader does it --
        # OpenAMS, ACE 2, U1 and Vivid all run read -> apply to the lane ->
        # sync to Spoolman, in that order, with nothing between the tag and
        # the lane.
        #
        # Spoolman stays authoritative for the two things it legitimately
        # owns: the spool_id itself, and the remaining weight it decrements
        # through a print.
        if tag_material:
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
            # A spool with 0 weight renders as empty/hidden in the UI. The AMS
            # has no scale, but the tag carries the nominal filament weight
            # (1 kg / 250 g mini) AND, on a spool the AMS has calibrated, a
            # persisted remain fraction -- so a part-used Bambu spool can show
            # its true remaining grams, not a hopeful 1000. remain 0 is
            # treated as "not measured", not "empty": every fresh tag in the
            # captures reads 0.0 until its first calibration, and turning a
            # brand-new spool into a 0 g one would be worse than the default.
            #
            # Re-seed on two conditions, not one: weight unset, OR weight still
            # exactly the UNSCALED nominal. The second covers a lane seeded
            # before the remain fraction was known (or read): AFC only ever
            # DECREMENTS from here as filament is used, so a value still
            # sitting on the nominal means nothing has been consumed from it
            # and scaling it is a correction, not a fight with the estimate.
            tag_w = info.get("weight")
            try:
                nominal = int(tag_w) if tag_w else 1000
            except (TypeError, ValueError):
                nominal = 1000
            w = nominal
            rp = info.get("remain_pct")
            if isinstance(rp, int) and rp > 0:
                w = max(1, (nominal * rp) // 100)
            # The AMS's measured remain% is authoritative for a spool sitting in
            # the bay -- more so than Spoolman's stored value, which binding
            # just wrote into lane.weight (e.g. 998 g, nearly-full, while the
            # tag says 80%). So when we HAVE a real remain% and the lane is not
            # currently feeding the toolhead (a loaded lane's weight is being
            # decremented by extrusion and must not be stomped), apply the
            # measured grams and push the correction to Spoolman. Otherwise the
            # old rule: seed only a default/unset weight.
            cur_w = getattr(lane, "weight", 0)
            if not cur_w or int(cur_w) == nominal:
                lane.weight = w
            if changed:
                self.logger.info(
                    f"AFC bambu {self.name}: applied tag to {lane.name}: "
                    f"{getattr(lane, 'filament_name', '') or tag_material} "
                    f"{color_hex or ''}".rstrip())
                # Change-only, so this is not a write per status frame. A tagged
                # bay usually survives a restart anyway (its profile is
                # re-derived from the AMS record at boot), but that is a
                # coincidence of the AMS still holding the record -- persist it
                # so the lane does not depend on that.
                self._save_lane_vars()
            # Full decode + a UID -> bind/create in Spoolman, keyed on the UID.
            self._spoolman_sync(lane, info)
        # No readable tag yet (a bay is staged but not yet fed past the reader):
        # do NOT apply an AFC default here. The tag arrives after the scan feeds
        # the spool past the reader, and a default applied on stage would show
        # the wrong material until (and lock out) the real tag. Leave the lane's
        # material untouched; the tag lands when the scan reads it.
        elif info.get("rfid_uid"):
            # UID-only: no usable profile, but a good UID -- match Spoolman by
            # the UID alone and bind if it already knows this spool. That is
            # all a third-party reel gives us, and it is enough to match on.
            #
            # Reaching here at all means no scan is open for this bay: a bay's
            # UID survives in the unit's record after the profile fields are
            # cleared, so mid-scan this branch would bind the PREVIOUS spool's
            # UID. Observed live, announcing the match before the insert edge
            # was even logged:
            #
            #   22:12  spool REMOVED from slot 0
            #   22:12  unbinding lane23 from spool 137 -- the bay is empty
            #   22:12  matched lane23 to Spoolman spool 137 by UID 01d0ec0f
            #   22:12  spool INSERTED in slot 0        <- the insert is AFTER
            #
            # _sync_lanes is what keeps that out now: it does not call this
            # function at all until the unit has answered.
            self._spoolman_sync(lane, info)

        # The AMS remain% sets the weight for ANY present lane -- bound or not,
        # tagged or not. The unit physically measured this reel; Spoolman's
        # stored weight is a memory of the last time something told it. So a
        # bound lane that kept Spoolman's 998 g (nearly-full) while the unit
        # measured 80% now shows 800 g, and the correction is pushed back.
        self._apply_remain_weight(lane, info)

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

    def _follow_arm_acked(self, latest: Any) -> Any:
        """
        Whether the bridge has seen this unit acknowledge its follower arm.

        The arm frame (0x11/0x04) is never answered at the frame level, so the
        only receipt is the unit narrating ``state:4``. The bridge tracks that
        per unit and reports it as an ``armack`` bitmask; this pulls out our
        bit.

        Returns ``None``, not ``False``, on firmware that predates the mask --
        "not acknowledged" and "cannot tell" are different answers, and this
        field exists precisely to make a silent arm visible.

        :param latest: The most recent bridge status frame, or None
        :return: True/False when known, None when the firmware does not report
        """
        if not latest:
            return None
        mask = latest.get("armack")
        if not isinstance(mask, int):
            return None
        return bool(mask & (1 << self.ams_index))

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
        # AN HT HAS ONE BAY. EVER.
        #
        # The internal arrays are SLOTS_PER_UNIT (4) wide on every unit type
        # deliberately -- the bridge indexes them by slot number and a
        # short array would fault on a stray frame naming slot 3. But
        # PUBLISHING all four made an HT look like a four-bay unit to Mainsail,
        # to `slots=` consumers and to anything counting bays:
        #
        #     BambuAMS_HT  online=True idx=2 slots=4 present=1
        #
        # unit_slots already carries the truth (1 for HT models, 4 otherwise);
        # it just was not being applied on the way out. Trim here rather than
        # narrowing the arrays, so the storage stays forgiving and only the
        # reported shape is correct.
        status["slots"] = self._published_slots()
        # Apply a narrated capacity measurement to the slot that asked for it.
        #
        # The AMS measures the spool at the end of a capscan and narrates the
        # percent ("P:84%", reproducible within 2% across runs) but does NOT
        # persist it to the tag record on our bus -- the "odom save" gate is
        # still unbroken. The narrated number IS the measurement, so the
        # bridge captures it and the module applies it here: the slot's
        # remain_pct and the lane's weight both become the measured value.
        try:
            pend = getattr(self, "_cap_pending_slot", None)
            # An AMS HT reports its calibration as a VERDICT CODE and never
            # narrates the boxed units' percent line, so a completed HT
            # calibration is indistinguishable from silence unless the verdict
            # is read: the unit's own log says "Calibration rst:0" (completed)
            # while a window wait for "P:NN%" never sees one on this model.
            # Surface it: the measured percent itself lands on the tag, so it
            # arrives through the next filament-info read of the slot rather
            # than from this line.
            if pend is not None and self._bridge is not None:
                getc = getattr(self._bridge, "last_ht_cali", None)
                cali = getc(getattr(self, "dry_dev_addr", 0)) if callable(getc) else None
                if cali and cali.get("t", 0) > getattr(self, "_cap_pending_t0", 0.0):
                    if cali.get("t", 0) != getattr(self, "_cap_cali_seen_t", None):
                        self._cap_cali_seen_t = cali.get("t", 0)
                        rst = cali.get("rst")
                        # RE-READ THE SLOT NOW. The measurement lands AFTER
                        # the tag read, and the firmware's background info fill
                        # is gated on `!info_valid` -- so once the tag reads
                        # successfully we stop reading that bay and the result
                        # is never collected. An AMS 2 does the work (watched
                        # physically -- one pull for the scan, a second for the
                        # measure) and does not narrate a percent, so the read
                        # is its only route to the host. An AMS 1 or HT narrates
                        # theirs and would not need it.
                        if rst == 0:
                            try:
                                self._bridge.send({"cmd": "reread",
                                                   "unit": self.ams_index,
                                                   "slot": pend})
                            except Exception:
                                pass
                        self.logger.info(
                            f"AFC bambu {self.name}: slot {pend} calibration "
                            + ({0: "completed (rst:0) -- re-reading the bay to "
                                   "collect the measured percent",
                                1: "refused (rst:1) -- capacity not enabled for "
                                   "this tray",
                                4: "aborted (rst:4) -- stalled during calibration"
                                }.get(rst, f"returned rst:{rst}")))
            if pend is not None and self._bridge is not None:
                meas = None
                try:
                    meas = self._bridge.last_cap_measure(
                        getattr(self, "dry_dev_addr", 0))
                except Exception:
                    meas = None
                if meas and meas.get("t", 0) > getattr(
                        self, "_cap_pending_t0", 0.0):
                    # Prefer the UNCLAMPED reading. A fresh spool legitimately
                    # measures over 100% -- 102, 107 and 119 all captured on real
                    # hardware -- and flattening those to 100 records a spool as
                    # exactly full when the unit actually said it holds more than
                    # nominal. Upper bound is a sanity check on a garbled read,
                    # not a statement about spools.
                    # A RESTORE IS NOT A MEASUREMENT OF THIS SPOOL.
                    #
                    # "odom load from flash 0,R:0.083,P:102" is the unit
                    # recalling what it last measured in that BAY -- which is
                    # the PREVIOUS spool once you swap one in. The live form
                    # ("odom C:0.490,R:0.078,P:84%") is the only one that means
                    # a spool was physically pulled just now. The bridge has
                    # always told them apart; nothing here read the flag.
                    #
                    # Caught by moving one spool between units. AMS 2 measured
                    # it at 73%. Put into the HT:
                    #
                    #   17:09:18  spool INSERTED in slot 0
                    #   17:09:22  odom load from flash 0,R:0.083,P:102  <- stale
                    #   17:09:22  "Measured full -- roughly 1000 g"     <- wrong
                    #   17:10:14  second detected, odom C:0.490,R:0.078,P:84%
                    #             ...ignored: pending already consumed above
                    #
                    # So we published 1000 g for a 730 g spool AND threw away
                    # the real reading 52 s later. Leaving `pend` set is the
                    # other half of the fix: the live measurement that follows
                    # still has something to attribute itself to.
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
                    else:
                        pct = int(meas.get("pct_raw") or meas.get("pct") or 0)
                        self._adopt_measured_remain(pend, pct, "capscan")
                        self._cap_pending_slot = None
                        # Measurement in, operation over: hand the bus back so
                        # the next unit can start. The claim also lapses on the
                        # unit's own cycle-end marker, so a failed measurement
                        # cannot wedge the bus -- this is the clean path, not
                        # the only one.
                        rel = getattr(self._bridge, "release_bus", None)
                        if callable(rel):
                            rel(self.name)
            # Re-apply every stored measurement on every pass, and let the
            # FRESH measurement WIN over the tag record. The record is what a
            # previous life wrote to the tag; a value we measured THIS session
            # (via capscan, on this spool) is newer and truer. Live example:
            # the HT measured 75% while its tag record still read 80% from an
            # earlier printer -- the record was re-read a beat later and
            # clobbered the fresh number. Overriding fixes that. _measured_remain
            # is cleared on spool removal (see _maybe_auto_scan), so a stale
            # measurement can never linger onto a different spool.
            for s_idx, pct in (getattr(self, "_measured_remain", None) or {}).items():
                for sl in (self._slots or []):
                    if sl.get("index") == s_idx and sl.get("present"):
                        sl["remain_pct"] = pct
        except Exception:
            pass
        # Follower + buffer telemetry, surfaced like an FPS buffer so it can be
        # watched in the UI. buff is the AMS's FPS "fullness" 0..100, stated by
        # SPRING state because "compressed"/"stretched" invert depending on
        # whether you mean the spring or the buffer travel:
        #   100 = spring compressed, the two PTFE ends pushed APART (fed)
        #     0 = spring extended, PTFE ends together, bottomed out (feed me)
        # buffer_state mirrors AFC buffer wording: compressed (fed) / expanded
        # (demand) / neutral.
        # ATTRIBUTION. Everything below comes from latest_status(), which is ONE
        # bridge-wide object: buff/buffraw/buffn/bufflen/fstate/fstaten are
        # top-level globals, and only units[] and slots[] are per unit. Every
        # configured unit was therefore republishing the same numbers as its
        # own -- measured on hardware with an HT following and an AMS 2 idle,
        # both reported identical buff, raw, fstate AND an identical
        # follow_buff_reads counter, which two independent units cannot have.
        #
        # The visible cost: a unit that was doing nothing showed fstate 4,
        # arm_acked True and a live buffer, so a panel could not tell which unit
        # was actually feeding.
        #
        # The firmware tracks one follower because it drives one at a time. So
        # the reading belongs to whichever unit is following, and for any other
        # unit the honest answer is None -- not the global value, and not zero.
        # Whose reading is it? The unit actively following, or failing that the
        # unit with a lane threaded to the toolhead -- that is whose filament is
        # in the buffer. Following alone was too narrow: it is host state that a
        # Klipper restart clears, and the follower does not re-arm until the
        # machine moves, so a real reading disappeared for every unit in the
        # meantime. When no unit owns the path, nobody claims the number, which
        # is the honest answer and not the same as reporting zero.
        mine = (self._following_lane is not None) or _unit_tool_loaded(self)
        latest_own = latest if mine else None
        buff = latest_own.get("buff") if latest_own else None
        # Odometer: filament length in the path, from the 0x03 motion reply.
        #
        # PER UNIT, unlike the follower telemetry above -- it rides in units[]
        # rather than at the top level, so it needs no attribution guessing.
        #
        # Live at the poll rate: decode_motion_reply runs on EVERY op-0x03
        # reply, and the idle status poll is one, so this refreshes every
        # round. (An earlier comment here claimed it updated only on motion --
        # written from where send_motion is called for feed/retract, without
        # noticing the idle poll goes through the same function.)
        #
        # NEGATIVE IS A READING, NOT AN ERROR. The unit's resting state
        # measures slightly below zero -- -74mm on this AMS 2 Pro, and the
        # capture that decoded the field starts at -0.032m. A `>= 0` gate here
        # held odom_m at None on live hardware for days while the value sat one
        # sign check away. Only the firmware's exact unknown-sentinel (-1mm) is
        # excluded; a true reading of exactly -1mm loses, and that is the
        # cheapest collision on offer.
        odom = None
        try:
            for u in (latest.get("units") or []):
                if int(u.get("n", -1)) == int(self.ams_index):
                    v = u.get("odom")
                    if v is not None and int(v) != -1:
                        odom = int(v) / 1000.0     # mm -> metres
                    break
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
        # Length of the last motion reply we tried to decode, and the raw 16-bit
        # field before calibration. A length <= 26 means the reply carries no
        # buffer at all on this AMS model; the raw value is what BUFF_POS_FULL /
        # BUFF_POS_EMPTY are calibrated against.
        status["follow_buff_replylen"] = latest_own.get("bufflen") if latest_own else None
        # Observability. follow_buff above is the firmware's MAPPED value, whose
        # calibration is known wrong on an AMS HT -- follow_buff_raw is the field
        # itself (signed LE) and is the one to trust.
        status["follow_buff_raw"] = latest_own.get("buffraw") if latest_own else None
        status["follow_state"] = latest_own.get("fstate") if latest_own else None
        # 0 means the AMS has never reported a follower state, so follow_state is
        # the firmware's seed (4) rather than a confirmation that it is following.
        status["follow_state_reads"] = latest_own.get("fstaten") if latest_own else None
        # Whether THIS unit's follower arm has been acknowledged. The arm frame
        # is never answered, so the only receipt is the unit narrating state:4;
        # until that lands the bridge re-sends the arm a few times rather than
        # leave a dropped arm unnoticed until the next slow sweep. False while
        # armed-and-following means the arm is not landing; False while idle is
        # simply "not armed" and says nothing.
        status["follow_arm_acked"] = self._follow_arm_acked(latest)
        # The AMS's own reference id, from its "[AMS_COMMON]...ref:N" narration.
        # An AMS HT only acts on a SELECT addressed to this id, so a mismatch
        # here means tray selects are ignored and the unit reports tray:255.
        status["ams_ref"] = latest.get("amsref") if latest else None
        # Which phase the firmware's op-04 state channel is in. The channel
        # runs at ~148ms and only its mode/ref change; this is that value, and
        # it is the fastest way to see whether a transition actually happened
        # instead of inferring it from narration after the fact.
        # decode_presence instrumentation: did a reply arrive and get thrown
        # away by the address/index guard, or was it never asked for? Those look
        # identical from outside, and telling them apart is the open question
        # for class addressing (the HT answered the roll-call at 0x80 yet
        # reported no bays). presdrop climbing while presok is flat means the
        # frame arrives and the guard discards it; presaddr is what byte[5]
        # actually held against preswant.
        # nexp: how many unit indices the status poll actually walks -- if it
        # does not reach the HT's index, its slot data is never even asked
        # for. htmask: which indices are HT-flagged, since the 0x1800 paths
        # gate on it and a stale bit after re-enrollment silences a unit.
        status["poll_nexp"] = latest.get("nexp") if latest else None
        status["ht_mask"] = latest.get("htmask") if latest else None
        # Per-unit: the reply LENGTH decode_presence accepted for THIS unit and
        # the raw byte[9] it read as the slot bitmap. Splits "the unit says it
        # is empty" from "we are reading the wrong offset in ITS reply" -- the
        # question left after addressing, reach, the HT flag and the discard
        # counter all came back clean with the HT still showing no bays.
        _u = None
        if latest:
            for _un in (latest.get("units") or []):
                if _un.get("n") == self.ams_index:
                    _u = _un
                    break
        status["pres_len"] = _u.get("preslen") if _u else None
        status["pres_byte"] = _u.get("presbyte") if _u else None
        status["pres_ok"] = latest.get("presok") if latest else None
        status["pres_drop"] = latest.get("presdrop") if latest else None
        status["pres_addr"] = latest.get("presaddr") if latest else None
        status["pres_want"] = latest.get("preswant") if latest else None
        _ph = latest.get("phase") if latest else None
        status["ams_phase"] = _ph
        status["ams_phase_name"] = {
            0: "idle 01/00", 1: "drive 03/00", 2: "enter 09/A5",
            3: "pre 07/00", 4: "hold 07/7F", 5: "release 07/00",
            6: "done 09/3F",
        }.get(_ph)
        # Why the follower may be standing down. Neither of these was in
        # get_status, which is why four deploy cycles could not see that a
        # latch set an hour earlier was holding the follower off.
        status["follow_manual_off"] = bool(
            getattr(self, "_follow_manual_off", False))
        status["follow_when_loaded"] = bool(
            getattr(self, "follow_when_loaded", False))
        # The build actually running on the Pico, as the firmware reports it on
        # the chain reply. Published because a panel showing AMS state should be
        # able to say which bus master produced it, and because "did the flash
        # take" is otherwise only answerable from a G-code console.
        try:
            status["bridge_fw"] = getattr(self._bridge, "_chain_fw", "") or None
        except Exception:
            status["bridge_fw"] = None
        # The AMS's last self-reported stall, and the motor current it came
        # with. Surfaced so a fault is inspectable after the fact, not only at
        # the moment it paused.
        if self._bridge is not None:
            _seq, ftext, famps = self._bridge.last_fault()
            status["ams_fault"] = ftext or None
            status["ams_motor_amps"] = famps or None
        # True while a stall has the follower held off, waiting for a resume.
        status["follow_fault_hold"] = self._follow_fault_hold
        # Armed = a fault is latched AND the buffer has been seen on the floor
        # under it, so relieving the pressure will clear it by itself. False
        # while held means the fault was raised on narration with a healthy
        # buffer: nothing to relieve, so it waits for a resume.
        status["fault_recover_armed"] = bool(
            self._follow_fault_hold and self._fault_floor_seen)
        # Narration accounting. The AMS returns its PENDING log text in reply to
        # a 1A/02 poll, and an empty reply is ordinary traffic -- so a quiet
        # AFC.log cannot be read as "the unit said nothing". polls climbing with
        # frames flat means it is not answering the log drain at all; frames
        # climbing with texts flat means it is answering and has nothing queued.
        status["ams_narration_polls"] = latest.get("dbgpolls") if latest else None
        status["ams_narration_frames"] = latest.get("dbgframes") if latest else None
        status["ams_narration_texts"] = latest.get("dbgtexts") if latest else None
        # Narration lines the firmware had to CUT. Reads 0, and that is the
        # point: lines ending mid-token at exactly 159 characters were
        # diagnosed as a buffer overflow and are not one -- 159 is what a full
        # 174-byte narration frame yields, so the old 160-byte buffer was
        # correctly sized. Raising it to 256 yields no longer lines.
        #
        # Kept as an instrument rather than a fix. While it reads 0, "the unit
        # never said X" is a claim about the unit; if it ever climbs, every
        # such claim is suspect and this is how you find out.
        status["ams_narration_cut"] = latest.get("dbgtrunc") if latest else None
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
        # Drying is host state, set by AFC_BAMBU_HEATER_START -- so a Klipper
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
        # A cycle the unit REFUSED is not a cycle.
        #
        # _drying is set optimistically by AFC_BAMBU_HEATER_START, before the AMS has
        # said anything. When it then answers "[AMS_CHMB]err, filament hub load!"
        # nothing cleared the flag: the symmetric adoption above cannot, because
        # it needs _dry_seen_live and a refused cycle never streams telemetry to
        # set it. So the panel reported DRYING, indefinitely, for a heater that
        # had plainly declined.
        #
        # Safe to key on the error alone because clear_dry_error() runs as each
        # start is commanded -- so a refusal on record now belongs to THIS
        # attempt. _dry_seen_live guards the other direction: once the unit has
        # actually reported for this cycle, a later error does not un-dry it.
        # Read the cycle stamps BEFORE anything that tests them. Fetching them
        # further down, next to the remaining-time calculation, leaves the
        # refusal guard below referencing `started` before it exists -- an
        # UnboundLocalError that takes Klippy down on the first status query.
        started = getattr(self, "_dry_started_at", None)
        minutes = getattr(self, "_dry_minutes", 0) or 0

        refused_now = self._bridge_call_arg("last_dry_error", self.dry_dev_addr)
        settled = (started is not None
                   and (_mono(self) - started) > DRY_REFUSE_GRACE)
        if (self._drying and refused_now and settled
                and not getattr(self, "_dry_seen_live", False)):
            self._drying = False
            # The stamps STAY. Clearing them looks tidy and quietly breaks the
            # rotate readout: if the unit later starts reporting after all, the
            # adoption above sets _drying again, but with _dry_started_at gone
            # the "cycle we commanded" branch stops matching and dry_rotate
            # falls through to the unit's echo -- which reports rotate:0 -- so
            # the toggle clears itself. Nothing reads the stamps while _drying
            # is False, and the next start overwrites them.
            if not getattr(self, "_dry_refusal_logged", False):
                self._dry_refusal_logged = True
                self.logger.info(
                    f"AFC bambu {self.name}: not drying -- the unit refused: "
                    f"{refused_now}")
        elif not refused_now:
            self._dry_refusal_logged = False
        status["drying"] = bool(self._drying)
        # Has the UNIT confirmed this cycle, or are we still going on the fact
        # that we asked?
        #
        # _drying is optimistic from the moment AFC_BAMBU_HEATER_START runs, and the
        # unit takes its time -- self-check first, chamber telemetry after. In
        # that gap the honest answer is neither "drying" nor "refused", and a
        # panel that has to pick one will pick wrong. Publishing the difference
        # lets it say "starting" instead of flickering between the two.
        status["dry_confirmed"] = bool(getattr(self, "_dry_seen_live", False))
        # Seconds left in the commanded cycle, or None.
        #
        # None is a real answer and not a zero: a cycle ADOPTED from live
        # chamber telemetry (one this host did not start) has no known duration,
        # and reporting 0 there would render as "finishing now" on a dryer that
        # may have hours to run. Absent means "it is drying and nobody here
        # knows for how long", which is the truth.
        remaining = None
        if self._drying and started is not None and minutes:
            remaining = max(0, int(minutes * 60 - (_mono(self) - started)))
        status["dry_remaining"] = remaining
        status["dry_minutes"] = minutes or None
        # Whether the running cycle is spinning the spool. Like the duration,
        # this only ever exists in the command we sent -- the AMS does not report
        # it back -- so it is None unless THIS host started the cycle. A panel
        # showing a rotate toggle needs to reflect what the heater is doing, not
        # what someone last tapped.
        # The UNIT's own echo wins over what we remember sending.
        #
        # [AMS_CHMB]rotate:R,R, pw_lim:P, cool_down:C,T, dur:M, tmpr:T is the
        # AMS reporting the settings it is holding, so it survives a Klipper
        # restart and it is the only source for a cycle this host did not start.
        # Fall back to the recorded command when the unit has not echoed --
        # older firmware, or a bridge that reconnected mid-cycle.
        cfg = None
        try:
            cfg = self._bridge_call_arg("last_dry_cfg", self.dry_dev_addr)
        except Exception:
            cfg = None
        if cfg and cfg.get("dur"):
            status["dry_minutes"] = int(cfg["dur"])
            if not remaining and started is not None:
                status["dry_remaining"] = max(
                    0, int(cfg["dur"] * 60 - (_mono(self) - started)))
        # The AMS's OWN countdown outranks everything above. The 0x3C
        # telemetry reply carries dry-remaining in seconds (payload[33:35] --
        # 28766 at the start of an 8h dry, ticking down monotonically in the
        # printer-driven capture), decoded by firmware >= 1.0.25.0 as
        # "dryrem" per unit. It survives Klipper restarts, knows the duration
        # of a cycle this host never started, and is the unit's number rather
        # than our stopwatch's. 0 doubles as "no dry running", so it only
        # replaces the estimate while a dry is actually believed active.
        try:
            for u in (latest.get("units") or []):
                if int(u.get("n", -1)) == int(self.ams_index):
                    dr = u.get("dryrem")
                    if dr is not None and int(dr) > 0:
                        status["dry_remaining"] = int(dr)
                        status["drying"] = True
                    break
        except Exception:
            pass
        # Chamber temp/humidity from the same 0x3C decode, preferred over the
        # narration-scraped values when present (envt climbs 35->57 through a
        # captured dry; envh falls 38->20 in step).
        try:
            for u in (latest.get("units") or []):
                if int(u.get("n", -1)) == int(self.ams_index):
                    et, eh = u.get("envt"), u.get("envh")
                    if et is not None and int(et) > 0:
                        status["temperature"] = float(int(et))
                    # Fill-in only: the 0x04-reply humidity is long-established
                    # and these are likely different sensors -- replacing a
                    # good reading with a different sensor's number would just
                    # make the dashboards argue with themselves.
                    if (status.get("humidity") is None and eh is not None
                            and 0 <= int(eh) <= 100):
                        status["humidity"] = int(eh)
                    break
        except Exception:
            pass

        # ROTATE is the one field where the COMMAND outranks the echo.
        #
        # Duration and target are readings -- the unit is the authority on what
        # it is holding. Rotate, in every capture so far, echoes as rotate:0,0
        # even for a cycle commanded with ROTATE=1, so trusting the echo made
        # the panel's toggle clear itself the moment drying began: the operator
        # asked for rotation and the display told them it was off. Whether the
        # unit honours the flag is a separate question and not one this field
        # can answer; what it CAN report faithfully is what was asked for.
        #
        # For a cycle this host did not start there is nothing asked-for to
        # report, so the echo is used, and None when there is neither.
        if self._drying and started is not None:
            status["dry_rotate"] = 1 if getattr(self, "_dry_rotate", 0) else 0
        elif cfg:
            status["dry_rotate"] = 1 if cfg.get("rotate") else 0
        else:
            status["dry_rotate"] = None
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
        # AFC_BAMBU_HEATER_START sends, whether or not the AMS accepted -- so
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

        HIGH IS COMPRESSED. The sign matters -- these callers invert with it:

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
        v = b / 100.0                      # compressed(buff 100)->1.0, empty(0)->0.0
        return 0.0 if v < 0.0 else 1.0 if v > 1.0 else v

    def _tool_loaded_lane(self) -> Optional[Any]:
        """
        Return a lane on this unit threaded to the toolhead AND belonging to
        the ACTIVE extruder, or None.

        Used to auto-arm the follower. On a toolchanger several lanes can be
        tool_loaded at once -- one per toolhead -- but only one extruder is
        active (receiving E moves); the rest are not pulling filament and need
        no follower. A loaded lane is skipped ONLY when another extruder is
        positively known to be active: uncertainty must never strip a follower,
        because a real printer holds a loaded tray unconditionally.

        "Active" is Klipper's current extruder (AFC's get_current_extruder),
        NOT on_shuttle(): a docked toolhead can legitimately be the active
        extruder during async/pre-load, and gating on the shuttle would strip
        the follower exactly when that load needs it. Unknown/unwired cases
        fall back to on_shuttle(), then to "active", so single-toolhead and
        partially-configured setups never lose their follower.

        :return Optional[Any]: a tool-loaded, slot-mapped, ACTIVE lane, or None
        """
        # AFC's own answer first. current_load is the lane AFC believes is
        # threaded to the toolhead right now, and it survives a G28 and a
        # Klipper restart -- current_lane does not (it was None here while
        # current_load was lane23 and the tray sat unheld). If AFC names a lane
        # on this unit, that IS the lane to follow; nothing else needs asking.
        try:
            # afc.current, NOT afc.current_load: the status dict publishes
            # afc.current under the key "current_load" (AFC.py get_status), and
            # "current_lane" is afc.current_loading. Reading the status KEY as
            # an attribute name silently returns None, which is exactly what it
            # did on the first attempt -- the fix deployed and changed nothing.
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
            # NO on_shuttle() fallback. Running it whenever the active extruder
            # is unknown breaks on a docked toolhead, which answers False -- so
            # after a G28, or after a Klipper restart with a lane still
            # tool_loaded, this returns None, the follower tick takes the
            # "nothing loaded here anymore" branch, and actively STOPS the
            # follower. Observed: the tray took the arm, showed state:4 for
            # under a second, dropped to state:0, and filament pulled by hand
            # was never recovered.
            #
            # A real printer holds a loaded tray unconditionally -- 3664 hold
            # frames in 547.6 s, ~100% continuous -- so "loaded to the toolhead"
            # IS the condition. The only reason to skip a loaded lane is
            # positive knowledge that a DIFFERENT extruder is the active one,
            # which is the branch above. Not knowing is not a reason to drop it.
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
        # Arm the auto-reset watcher against THIS fault. The lane is kept so the
        # reset feed can be aimed at the slot that actually stalled, rather than
        # at whatever happens to be loaded by the time it recovers.
        self._fault_lane = lane
        # Arm the declaration latch against THIS fault. Cleared here, set by
        # any status frame carrying byte[19] == 0x07, so "declared" always
        # means "since this fault", never a leftover from the last one.
        self._declared_since_fault = False
        # Same for the odometer range: how far the AMS moves filament during
        # THIS fault's recovery is what says where the jam is.
        self._odom_lo = None
        self._odom_hi = None
        self._fault_floor_seen = False
        self._fault_recover_since = 0.0
        self._fault_recover_reads = None
        try:
            self.set_feed_assist(lane, False)
        except Exception as e:
            # A fault report must never be able to break the follower tick.
            self.logger.debug(
                f"AFC bambu {self.name}: could not drop assist on stall: {e}")
        # Pausing runs the PAUSE macro, which moves Z. Outside a print -- or
        # before the axes are homed -- that move raises "Must home axis first",
        # and because this executes inside the follower's reactor timer an
        # escaped exception shuts down ALL of Klipper (observed: a stall during
        # an idle HT capacity calibration emergency-stopped every MCU). So only
        # ask to pause when a print is actually running, and never let the pause
        # path throw past here -- a fault report must never break the tick.
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
        # already is) empty and RESUME alone would print air. Remember that
        # here, not inside _maybe_auto_recover: the reload-on-resume is what
        # makes the ordinary resume button correct, and it has to work with
        # auto_error_recovery OFF -- that is the default, and the case where a
        # human is doing all the recovering.
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

        Expressed through AFC's own lane routines rather than by driving the
        bus: the unload macro already does cut -> retract -> unload and the
        load macro does the reload, so reimplementing either would duplicate
        the cutter logic and drift from it.

        OFF BY DEFAULT (auto_error_recovery) -- a recovery that moves the
        toolhead and the filament unasked is opt-in.

        Two guards:
          * ONE ATTEMPT per fault. A recovery that can retrigger itself grinds
            filament, and the AMS is already retrying inside its own 70 s
            window; a second retry on top fights it.
          * NEVER INLINE. This runs inside the follower's reactor timer, where
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
            return
        self.logger.info(
            f"AFC bambu {self.name}: auto error recovery for {name} -- "
            f"unloading (cut, retract, unload) and reloading the same spool. "
            f"The print stays PAUSED either way. "
            f"Disable with auto_error_recovery: False.")

        def _declared():
            """
            Has the unit said it gave up AT ANY POINT since this fault?

            The LATCH, not a fresh sample. byte[19] == 0x07 appears in only
            11% of frames during the park (278 of 2523 in the AMS 1 capture),
            so reading the current frame once at the end of a ~90 s recovery
            attempt misses it eight times in nine -- which is exactly what
            happened on hardware, at a unit the operator could see was latched.

            _on_status sets this on any frame carrying it;
            _raise_ams_fault clears it when a new fault is armed.
            """
            return bool(getattr(self, "_declared_since_fault", False))

        def _paused():
            try:
                return bool(self.afc.function.is_paused())
            except Exception:
                try:
                    return bool(self.printer.lookup_object(
                        "pause_resume").is_paused)
                except Exception:
                    return False

        def _done(rv):
            """
            Clear the in-progress flag on every exit from the attempt.

            _in_auto_recover must be cleared on ALL paths including the
            failures: left set, it suppresses the legitimate re-arm on the
            NEXT fault.

            :param rv: the value to hand back to the caller
            :return: rv, unchanged
            """
            self._in_auto_recover = False
            return rv

        def _run(eventtime, _n=name):
            # Queue only. AFC's own macros own the cutter and the toolhead.
            try:
                # TOOL_UNLOAD, not UNSET_LANE_LOADED. UNSET only tells AFC the
                # lane is no longer loaded -- it does not move any filament, so
                # the severed strand stays in the toolhead and the reload then
                # drives a SECOND strand into an occupied path. A jam on top of
                # a jam.
                #
                # TOOL_UNLOAD is the routine that actually cuts, retracts and
                # unloads, which is the printer's own sequence (cut at the
                # toolhead, ~12 s retract, then the reload attempt).
                self.gcode.run_script(
                    f"TOOL_UNLOAD LANE={_n}\nCHANGE_TOOL LANE={_n}")
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: auto error recovery for {_n} "
                    f"could not run ({e}); the lane is still paused and held. "
                    f"Recover it by hand.")
                return _done(self.afc.reactor.NEVER)
            # ONE ATTEMPT. THERE IS NOTHING FOR US TO RETRY.
            #
            # The AMS retries the load BY ITSELF, automatically, and by the
            # time it gives up the unit is ALREADY HELD IN ERROR. From that
            # point it will not move again until it is told to load -- which,
            # in the captures, is what the printer sends after a human presses
            # continue. So a retry loop out here is not a second chance, it is
            # a machine talking to a unit that has stopped listening.
            #
            # This was built as an unbounded 5 s loop and watched on hardware:
            # it cycled unload/reload for over two minutes at a latched unit
            # and only ever "worked" at the moment a human freed the jam by
            # hand. Worse, it was a retry wrapped around a retry -- the reload
            # inside unit_load_lane already kicks 23 times over two recovery
            # rounds, roughly 90 s, before it reports failure.
            #
            # So: one unload-and-reload, then hand it to the operator. That is
            # also what the arc shows the printer doing -- retry within a
            # window, then PARK and wait for continue.
            if _declared():
                self.logger.warning(
                    f"AFC bambu {self.name}: {_n} -- the unit has given up "
                    f"(state 7). Parked; the print stays PAUSED. Clear the jam "
                    f"and resume. {self._jam_location()}".rstrip())
                return _done(self.afc.reactor.NEVER)
            # THIS NEVER RESUMES THE PRINT. NOT EVEN ON SUCCESS.
            #
            # It resumed unconditionally on its first hardware run and restarted
            # the print with NOTHING in the toolhead. That was made conditional
            # on the lane coming back tool_loaded -- and on the SECOND hardware
            # run it resumed again, this time correctly loaded, and that was
            # still wrong:
            #
            #   57672.9  lane15 reached the toolhead sensor after 7 kick(s)
            #   57705.0  AFC_RESUME                       <- us, nobody asked
            #
            # "Verified before resuming" was never the requirement. The
            # requirement is that the machine does not decide to restart
            # somebody's print. A real printer does not either: it holds at the
            # fault until a human presses continue.
            #
            # So the division of labour is:
            #     THIS restores the FILAMENT   (unload, cut, retract, reload)
            #     THE HUMAN restores the PRINT (the resume button)
            #
            # and the RESUME wrap is what makes that button correct -- it
            # reloads first if the lane is still empty, and refuses to continue
            # into an empty toolhead. Nothing here needs to press it.
            ln = self.lanes.get(_n)
            loaded = bool(getattr(ln, "tool_loaded", False)) if ln else False
            if not loaded:
                # The unit already retried on its own and gave up, so it is
                # held in error and waiting to be told to load. Say what is
                # true and what to do; do not keep poking it.
                self.logger.warning(
                    f"AFC bambu {self.name}: {_n} did NOT reload -- the AMS is "
                    f"HELD IN ERROR and will not try again on its own. Clear "
                    f"the jam, then press resume: that is what tells it to load."
                    + (f" {self._jam_location()}" if self._jam_location() else "")
                    + ("" if _paused() else
                       " NOTE: the print is no longer paused and the toolhead "
                       "is empty."))
                return _done(self.afc.reactor.NEVER)
            # The reload took. Stop the loop, drop the reload the resume wrap
            # would otherwise owe, and LEAVE THE PRINT PAUSED.
            self._resume_needs_reload = False
            self.logger.info(
                f"AFC bambu {self.name}: {_n} is reloaded and ready. THE PRINT "
                f"IS STILL PAUSED -- press resume when you are ready. Nothing "
                f"here will resume it for you.")
            return _done(self.afc.reactor.NEVER)

        try:
            self.afc.reactor.register_callback(
                _run, self.afc.reactor.monotonic() + 1.0)
        except Exception:
            self._auto_recover_armed = False
            self._in_auto_recover = False

    #: How long a UID may stay unresolved before it is worth saying out loud.
    #: A healthy chain resolves in well under a second; this is the threshold
    #: between "still starting up" and "this unit is not on the bus".
    CHAIN_RESOLVE_WARN_S = 30.0

    def _check_chain_resolve(self, eventtime: float) -> None:
        """
        Say something only if a UID never resolves.

        The deferral itself is normal and correct -- registrations are held
        until the chain map says which index this UID actually holds, rather
        than being filed against the config default and landing on whichever
        unit happens to be there. That is routine at every boot.

        What is NOT routine is still waiting half a minute later: that means
        the unit is not answering the bus at all, and its registrations (HT
        flag, MC address, self-centre, capacity enable) have never been sent.
        Warn ONCE for that, then stay quiet.

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
            # Waiting for a resume that can only follow a pause -- and outside
            # a print there will never BE one. A fault raised while idle used
            # to latch the follower off permanently: nothing paused, so
            # _follow_fault_saw_pause stayed False and this returned True for
            # the life of the object. Observed as a follower that "stopped
            # working" with no way back short of AFC_BAMBU_FOLLOWER ENABLE=1 or a
            # fresh load, and easily mistaken for state surviving a restart --
            # it is not saved anywhere; it was simply being re-set each time.
            try:
                printing = bool(self.afc.function.in_print())
            except Exception:
                printing = True     # unknown: keep the safer, held behaviour
            if printing:
                return True
        was_printing = self._follow_fault_saw_pause
        self._follow_fault_hold = False
        # Re-arm auto recovery: one attempt per FAULT, not per print -- but
        # NEVER from inside the recovery's own attempt (see _in_auto_recover).
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
        low = text.lower()
        # "stall exit" is the scan path finishing its pull-in, not a failure.
        if "stall exit" in low:
            return
        # A stall "during calib" is the AMS's own capacity choreography pulling
        # the spool to a hard stop to measure its radius -- it narrates
        # "check stall during calib" / "Calibration rst" as a normal step, not a
        # jam. Acting on it once emergency-stopped the printer mid-measurement.
        if "calib" in low:
            return
        current = f", motor {amps:.2f}A" if amps else ""
        # The unit's own error level, when it has stated one. Captured on both
        # types -- "err_code: 0 -> 23" (HT), "err_code:0x00->0x80" (AMS 2) --
        # and worth surfacing: it is the unit's verdict, not our inference, and
        # it is what changes when the jam is genuinely cleared.
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
        # Resuming into a jam that is still there will stall again and pause
        # again: the unit accepts a clear immediately but cannot leave its
        # error state until the filament actually moves (captured -- an HT
        # reported err_code 18 -> 0 on the first clear and still could not
        # pull). So say what to fix rather than blocking the resume; if it is
        # still stuck, this fires again with the same instruction.
        try:
            if self.afc.function.in_print():
                msg += "\nOnce cleared, click resume to continue printing"
        except Exception:
            pass
        self._raise_ams_fault(lane, msg)

    # No buffer-starvation detector. Inferring a stall from the buffer
    # bottoming out while the extruder pulls is a second opinion about a fault
    # the unit has not declared, and it fires on healthy prints. It also reads
    # a buffer SHARED between units, so a unit can act on pressure that is not
    # its own. byte[19] == 0x07 reports the state directly on all three unit
    # types, including the one that reports faults as state rather than words
    # ("state:6" / "en:0,mode:7"). The AMS says when it errors; nothing needs
    # to override that.

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
            # follower off. Idle the tick until AFC_BAMBU_HEATER_STOP clears the
            # flag. When a lane IS loaded, fall through -- dry-while-printing
            # still needs the follower or the extruder fights the pull.
            return eventtime + self.follow_poll_interval
        lane = self._following_lane
        # Runs BEFORE the hold is read, so a unit whose pressure has come off
        # re-arms on this same tick instead of waiting a poll interval. Contained
        # like every other detector here: this is a reactor timer, and an escaped
        # exception takes all of Klipper down with it.
        try:
            chk = getattr(self, "_check_chain_resolve", None)
            if callable(chk):
                chk(eventtime)
            # NO BUFFER-BASED AUTO-RESET either: "buffer came off the floor"
            # does not mean "somebody freed the jam", and acting on it fires a
            # feed into a nozzle that may still be being cut. A hold clears the
            # way it is raised -- by the unit leaving its fault, or by the
            # human resuming.
        except Exception as e:
            log = getattr(self, "logger", None)
            if log:
                log.warning(
                    f"AFC bambu {self.name}: fault auto-reset raised {e!r}; "
                    f"follower tick continuing.")
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
                and not getattr(self, '_unload_in_progress', False)
                # AND NOT MID-LOAD. cur_lane.status is not set to TOOL_LOADED
                # until the END of unit_load_lane, so through the whole arrival
                # _tool_loaded_lane() answers None and this tick takes its
                # "nothing loaded here anymore" branch -- dropping the assist
                # the load path armed a moment earlier, then re-arming it once
                # the status lands. Caught by the stand-down diagnostic:
                #
                #   03:52:24  ack select, ack assist      (load path)
                #   03:52:24  standing the follower down for lane23
                #   03:52:26  ack select, ack assist      (re-armed)
                #
                # Three mode changes in two seconds at a unit that was loading
                # correctly, each one a motor action -- audible as a noise at
                # the moment the load "stops or releases".
                # DISABLED AT THE OPERATOR'S REQUEST -- suspected of making
                # loads worse. The guard stopped the follower tick dropping and
                # re-arming the assist mid-load; that drop may have been
                # interrupting the unit's pull in a way that helped. Left in
                # place, inverted to a no-op, so it is one word to re-enable:
                #     and not getattr(self, '_load_in_progress', False)):
                and True):
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
                    #
                    # The old extra condition here was "and the extruder moved
                    # recently" (_follow_last_demand within follow_rearm_window).
                    # That went with the demand gate below: a unit that dropped
                    # to IDLE while the printer was between extrusions stayed
                    # dropped. Being loaded IS the condition now -- we are inside
                    # `loaded is not None`, so there is nothing further to ask.
                    # AND THE EXTRUDER HAS TO ACTUALLY WANT FILAMENT.
                    #
                    # The gate below was deleted with the note that "with the
                    # hold sending op-04 07/7F on its own 148 ms metronome there
                    # is no re-arm to storm". Measured at a healthy, loaded,
                    # IDLE unit: 14 assist re-arms in 30 seconds, one every two
                    # seconds, each one narrated by the unit and each one a
                    # motor nudge. The knob and the explanation for this were
                    # left in the config when the code went -- follow_rearm_
                    # window's own comment describes this loop exactly:
                    #
                    #   "state:0 is the AMS's RESTING state, not a fault -- it
                    #    arms, finishes its assist within a second or two, and
                    #    reports 0 until something asks it for filament again.
                    #    Re-arming on state alone therefore never settles."
                    #
                    # Right. state 0 at an idle unit means CENTRED, not dropped,
                    # and the printer holds a loaded tray without re-issuing
                    # anything (3664 hold frames in 547.6 s, ~100% continuous --
                    # the hold is a stream, not a repeated command).
                    #
                    # So track real extrusion, and only treat state 0 as
                    # "dropped" when the extruder has asked for filament inside
                    # follow_rearm_window. Between extrusions a resting unit is
                    # left alone.
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
                        # No toolhead to read: fall back to the old behaviour
                        # rather than never re-arming.
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
                # Nothing loaded from this unit anymore -> stop so it can't twitch.
                #
                # SAY WHY. This branch stands the follower down, and the next
                # tick can re-engage it, so if _tool_loaded_lane() flickers the
                # pair becomes a visible engage/stand-down cycle -- which is
                # what an operator sees as the console flapping at a lane that
                # is loaded and idle. The suspicion is the ACTIVE EXTRUDER gate
                # inside _tool_loaded_lane: on a toolchanger the active tool is
                # unsettled until a home, and this function has taken that
                # branch by mistake once before ("a docked toolhead answers
                # False -- so after a G28 this returned None ... and it
                # actively STOPPED the follower").
                #
                # Rate-limited to once every 5s: this must never become the
                # flood it exists to diagnose.
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
        # Fault detection follows the LOADED lane, not the FOLLOWED one.
        # AFC_BAMBU_FOLLOWER ENABLE=0 clears _following_lane, and running without
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
            # Both detectors run inside this reactor timer, so an exception
            # escaping either one propagates to reactor.run() and shuts down all
            # of Klipper. Contain them here as a final backstop: a fault reporter
            # must never be able to stop -- let alone crash -- the follower.
            try:
                # NO byte[19] STALL DETECTOR. It raised a fault TWICE during a
                # load that completed perfectly:
                #
                #   16:04:36  reached the toolhead sensor after 4 feed kick(s)
                #   16:04:46  the AMS reports it has STALLED (state 7)
                #   16:05:08  lane21 is now loaded in toolhead
                #   16:05:17  the AMS reports it has STALLED (state 7)
                #
                # and each time the only thing it achieved was releasing the
                # follower hold ("no print to resume") on a healthy lane.
                #
                # The byte is a good PARK signal -- 0x07 appears only in the
                # park across 32,000 captured frames -- and it is not a load
                # signal. Measured through a printer-driven load it reads
                # 0x00/0x9F/0x5B/0x2E, and the reply LENGTH varies per phase
                # (44/19/32/21/60/130), so it is not even the same field
                # throughout. We were reading a park indicator during a load.
                #
                # THE UNIT DETECTS ITS OWN PROBLEMS. It retries, it latches, it
                # goes red, and it refuses to move until told to load. Raising
                # a Klipper fault on top of that adds no information and does
                # interfere. The narration check below stays: that is the unit
                # telling us in its own words, which is a different thing from
                # us inferring from a byte.
                self._check_ams_fault(watch)
                # NO BUFFER WATCHDOG. It was written when narration was the only
                # detector and the AMS 1 never uses fault WORDS, so something had
                # to cover it. byte[19] does now -- measured on all three units,
                # including the wordless one -- and that made the watchdog a
                # second opinion inferring a fault the unit had not declared.
                #
                # It fired at fullness 24 on a healthy print. It also reads a
                # buffer SHARED between units (one user at a time), so a unit
                # can act on pressure that is not even its own. The AMS tells us
                # when it errors; we do not need a layer that can override it.
            except Exception as e:
                self.logger.warning(
                    f"AFC bambu {self.name}: fault check raised {e!r}; "
                    f"follower tick continuing.")
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
                # UNCONDITIONAL, for as long as a lane is loaded to the toolhead.
                # That is what a real printer does and the capture is emphatic
                # about it: 3664 hold frames in 547.6 s at a 149 ms median, and
                # 547.6/0.149 = 3675 -- so the hold is ~100% continuous, with no
                # gaps for travel, non-extruding moves, or idle between layers.
                # The AMS is "active" whenever a tray is loading or loaded; it is
                # never asked to prove the extruder moved first.
                #
                # NO demand gate on the active extruder's E. The hold sends
                # op-04 07/7F on its own 148 ms metronome, so there is no
                # re-arm to storm and nothing to rate-limit; gating on real
                # advance only starves the follower between extrusions, which
                # is what "not active or assisting" looks like on the HT.
                self._bridge.send({"cmd": "follow"})
                return eventtime + self.follow_poll_interval
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
                   mmps: Optional[float] = None,
                   fault_mark: Optional[int] = None) -> bool:
        """
        Wait for a bridge move to finish, preferring the AMS's own report.

        The AMS announces completion itself ("[AMS_SWITCH]feed finish...",
        "[AMS_PRELOAD]preload finish..."), so wait for that rather than sleeping
        for a computed duration: the unit moves at its own speed, not the mm/s
        we ask for, and a distance/speed estimate is therefore only ever a
        guess. It also tells us whether the move actually succeeded -- a stall
        reports "finish -1".

        Falls back to the estimated duration as a timeout, so hardware that
        does not narrate still works.

        :param mm: Distance commanded in mm
        :param mmps: Commanded speed in mm/s
        :param fault_mark: A sequence from _ams_fault_seq taken before the
          move. When given, a fault raised past it ends the wait immediately
          instead of sitting out the deadline. Peeked, never consumed -- the
          caller that acts on it is the one that reports it.
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
        # unload would miss this signal and fall through to the deadline --
        # exactly what this check exists to prevent.
        try:
            end = reactor.monotonic() + deadline_s
            while reactor.monotonic() < end:
                if bridge is not None:
                    seq, ok, _text = bridge.last_finish()
                    if seq != start_seq:
                        return ok
                    # A THIRD SIGNAL: the unit saying it has given up.
                    #
                    # A latched unit reports no completion, so before this the
                    # wait sat out its whole deadline -- measured 22s on an
                    # AMS 2 that had already declared "TIMEOUT error 2/3" and
                    # stopped listening. Worse, the fault only surfaced AFTER
                    # that, because both detectors mute themselves while an
                    # unload runs and the follower tick could not see it until
                    # the mute lifted. The unit's verdict was sitting in the
                    # bridge the whole time; nothing was reading it.
                    if (fault_mark is not None
                            and self._ams_fault_since(fault_mark,
                                                      consume=False)):
                        return False
                reactor.pause(reactor.monotonic() + 0.1)
        except Exception:
            pass
        return False

    def _toolhead_sensor_triggered(self, cur_lane: Any) -> bool:
        """
        Whether the lane's toolhead pre-sensor (or buffer) reports filament.

        Reads the PIN, not the cache. ``pin_tool_start`` has two consumers in
        AFC_extruder: a filament switch whose ``runout_helper.filament_present``
        is the live state -- what AFC's own runout path reads -- and a button
        callback maintaining ``tool_start_state``, which is what
        ``get_toolhead_pre_sensor_state()`` returns and is only as current as
        the last callback.

        The distinction gates recovery, not just reporting. unit_load_lane's
        retry (stop, re-home, feed again, ``load_recover_attempts`` times) is
        gated on `if not loaded`, so a cache that briefly reads "filament"
        makes _feed_until_sensor return True and the retry never runs -- a
        false success disables the recovery.

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

    # _arrived REMOVED, and the load path is back to the bare stop() that has
    # always worked.
    #
    # WHAT WAS EVER BROKEN: nothing. The gap was that our load never sent
    # 09/A5 -- found by reading the phase machine, not by anything failing.
    # Three attempts to close it, each measured on hardware by the operator:
    #
    #   assist at the sensor          -> "chunky"
    #   mark-loaded without arming    -> "still a little off, binds up at the
    #                                     feeder"
    #   op-03 motion byte = ref       -> jammed, TIMEOUT error 2/3, motor 1.39A
    #
    # The last one is the clearest lesson: the motion byte correlates with the
    # ref across 32,000 captured frames and putting it on the wire faulted the
    # unit, because a capture says what ACCOMPANIES what, never what CAUSES
    # what. The printer sends motion 7F once the tray is genuinely held; we
    # got to "held" by our own state machine, so the same byte arrived in a
    # different context and meant something else.
    #
    # Gating the transitions on the unit's own byte[19] was the next idea and
    # the data killed it before it was built: through a printer-driven load
    # that byte reads 0x00/0x9F/0x5B/0x2E, and in the two phases that would
    # need it (ARRIVED, PRE) the unit sends no 32-byte reply at all. Reply
    # LENGTH varies per phase (44/19/32/21/60/130), so byte[19] is not even
    # the same field throughout.
    #
    # WHAT WOULD MAKE THIS ANSWERABLE: we cannot see what we transmit. The
    # sniff build is listen-only and we are the master, so every question here
    # has been settled by inference. A TX echo -- the master reporting its own
    # frames, behind a toggle -- would let a load be diffed against
    # ht_clean_load frame by frame. Build that before touching this again.

    def _ams_fault_seq(self) -> int:
        """
        The bridge's current fault sequence, WITHOUT consuming it.

        A mark to compare against later: take one before asking the unit to
        move, and anything past it is a fault this move provoked rather than
        one left over from before.

        :return int: the current sequence, or 0 when unavailable
        """
        getf = getattr(self._bridge, "last_fault", None) if self._bridge else None
        if not callable(getf):
            return 0
        try:
            return int(getf()[0] or 0)
        except Exception:
            return 0

    def _ams_fault_since(self, mark: int,
                         consume: bool = True) -> Optional[str]:
        """
        The AMS's own words for a REAL fault raised since ``mark``.

        NOT EVERY FAULT-SHAPED LINE IS A FAULT, and this is the one place that
        judgement lives now. The bridge classifies by text and bumps its
        sequence on the match, so a scan ending its pull-in ("bldc stall exit")
        and the capacity choreography pulling the spool to a hard stop
        ("check stall during calib") both advance it while nothing is wrong.
        _check_ams_fault has always dropped those two after reading; every new
        caller needs the same filter, so it moved here rather than being
        copied.

        Consuming marks the fault seen, which SUPPRESSES the follower tick's
        _check_ams_fault for that event. That is the point: whoever consumes it
        owns reporting it. Pass ``consume=False`` to peek -- to break a wait
        early, say -- and leave the reporting to the caller that acts on it.

        :param mark: A sequence from _ams_fault_seq taken before the move
        :param consume: Whether to mark the fault seen (default True)
        :return str: the unit's own text, or None if nothing real is new
        """
        getf = getattr(self._bridge, "last_fault", None) if self._bridge else None
        if not callable(getf):
            return None
        try:
            seq, text, _amps = getf()
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
        Has the AMS reported a NEW fault since the last time we looked?

        The unit knows things we cannot see -- which motor, which tray, rocker
        state -- and it latches when it gives up. Polling its own report is how
        a load stops kicking a unit that has already stopped listening.

        Consumes the sequence, so one fault is reported once -- but KEEPS THE
        WORDS in _declared_fault_text.

        Consuming stops a second consumer raising the same event twice, but the
        words must survive it: without them the load's final error falls back
        to a generic "check afc_bowden_length calibration" for a unit that had
        just said "TIMEOUT error 0", sending the operator to measure a bowden
        that was never the problem.

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
            # THE UNIT DECLARES ITS OWN ERRORS -- STOP KICKING A LATCHED ONE.
            #
            # A jammed AMS latches and will not move again until told to load.
            # Kicking it every load_retry_interval for the rest of the window
            # is the "stuck in the load loop" the operator hit: minutes of
            # feed commands at a unit that has already given up, with the real
            # failure invisible underneath.
            #
            # Its own report is the authority ("feed finish -1, stall",
            # "pull err, bdc stall", err_code), so break out and let the
            # caller's error path run -- which is where the retry/park
            # decisions already live.
            if self._ams_declared_fault():
                self.logger.warning(
                    f"AFC bambu {self.name}: {cur_lane.name} -- the AMS "
                    f"reported a fault during the load; stopping rather than "
                    f"kicking a latched unit")
                break
            if self._toolhead_sensor_triggered(cur_lane):
                self.stop()          # halt instantly so the AMS can't retract it
                # READ THE ODOMETER NOW, not after the load finishes.
                #
                # It keeps climbing once the extruder takes over: 3.346 m at
                # the trip, 3.469 m thirty seconds later on the traced AMS 1
                # load. That 123 mm is not drift -- it is tool_stn plus the
                # purge, real filament pulled through by the extruder, which
                # the odometer counts because it counts filament. Correct
                # behaviour, and exactly why the reading has to be taken HERE:
                # anywhere later and the path length silently absorbs whatever
                # the toolhead consumed after arrival.
                self._load_odom_at_sensor = self._odom_now_mm()
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
            # NO fallback to the AMS's own arrival report here.
            #
            # There was one, for a lane with no toolhead signal at all. That
            # lane does not exist: tool_start is always either a real PIN or
            # "buffer", and both are authorities. So the branch could never
            # run, while looking like a safety net -- and before it was gated
            # on the sensor it did fire, accepting "feed finish, dw_len:3.532 m"
            # as a completed load while the filament was stuck in the PTFE and
            # the sensor had never tripped.
            #
            # The unit reaching the end of ITS measured tube and the filament
            # reaching the toolhead are different facts, and they diverge
            # exactly when the path binds. The kicks above are what seat it.
            try:
                self.afc.reactor.pause(now + 0.05)
            except Exception:
                break
        self.stop()
        return False

    def _pull_seq_now(self) -> int:
        """
        The bridge's current mode:4 pull counter, or 0 if it cannot be read.

        :return int: sequence number
        """
        try:
            return int(self._bridge.last_pull())
        except Exception:
            return 0

    def _assist_seq_now(self) -> int:
        """
        The bridge's completed-assist counter, or 0 if it cannot be read.

        :return int: sequence number
        """
        try:
            return int(self._bridge.last_assist_done())
        except Exception:
            return 0

    def _wait_for_pull(self, seq0: int, aseq0: int = 0) -> bool:
        """
        Block until the AMS finishes its native mode:4 pull, or the ceiling.

        The pull is what the extruder must not advance into. Waiting for the
        unit to report it beats waiting a fixed time: measured, the pull ended
        2.02s after the assist while the fixed wait was 2.00s, so the advance
        began 20ms early -- and the captures range 0.5-2.2s.

        pull_settle_s is now the CEILING, not the wait. A unit that never
        narrates the pull falls back to exactly the old behaviour.

        :param seq0: the pull counter read before the mode change
        :return bool: True if the unit reported the pull, False on the ceiling
        """
        # NO COMMANDED SWITCH, NO PULL TO WAIT FOR.
        #
        # The pull is what a mode-09 select causes ("pull sucess,MODE CHANGE,
        # mode:4"). With arrival_select off we never command the switch, the
        # unit finishes into mode:4 by itself as the printer's does, and there
        # is no pull -- so this would sit out its whole ceiling waiting for an
        # event that cannot arrive. Stacked on the arrival assist delay that is
        # ten seconds of dead time before the tool_stn advance, which is what
        # the operator felt as "extra delays, but the motion was perfect".
        if not self.arrival_select:
            return False
        if self.pull_settle_s <= 0:
            return False
        reactor = self.afc.reactor
        deadline = reactor.monotonic() + self.pull_settle_s
        while reactor.monotonic() < deadline:
            # WAIT ON THE PULL, THEN DWELL FOR THE PUSH-FORWARD.
            #
            # The cycle is pull back THEN push forward and both must finish --
            # but "assist finish" is NOT the end of the push. Measured, it
            # fires BEFORE the sensor, not after the pull:
            #
            #   08:49:42  assist finish 0, ref:0        <- before the arrival
            #   08:49:59  reached the toolhead sensor
            #   08:50:01  pull sucess,mode change,mode:4
            #   08:50:05  did not report a completed pull+push  (6s ceiling)
            #
            # Requiring both therefore made the wait ALWAYS fall through to the
            # ceiling -- strictly worse than waiting on the pull alone. The
            # unit gives us a reliable end-of-pull and no end-of-push we have
            # identified, so wait for the pull and then dwell
            # PULL_PUSH_DWELL_S for the half we cannot observe.
            if self._pull_seq_now() != seq0:
                # Seen. Give the motor a beat to actually stop before the
                # extruder takes over -- the narration is the unit's report,
                # not a guarantee the shaft is stationary.
                try:
                    reactor.pause(
                        reactor.monotonic() + self.pull_push_dwell_s)
                except Exception:
                    pass
                self.logger.debug(
                    f"AFC bambu {self.name}: pull reported; dwelling "
                    f"{self.pull_push_dwell_s:.2f}s for the transmission to "
                    f"reverse and push forward, then advancing")
                return True
            try:
                reactor.pause(reactor.monotonic() + 0.05)
            except Exception:
                break
        self.logger.debug(
            f"AFC bambu {self.name}: no pull reported within "
            f"{self.pull_settle_s:.2f}s; advancing on the ceiling")
        return False

    def _advance_into_extruder(self, cur_lane: Any, cur_extruder: Any) -> None:
        """
        Hand the filament to the extruder without fighting the AMS for it.

        Bite, let the unit pull, then advance:

            bite (tool_bite_mm)       gears grip, nothing else moving
            select + assist           the unit pulls back and pushes: native
            settle (pull_settle_s)    stay off the filament while it does
            advance (tool_stn - bite) the rest, into a path that has settled

        The AMS pulls the tray back on the mode change into mode:4 (native,
        0.5-2.2s) to seat the filament against its own switch. Driving the
        gears forward through that turns them backwards and fights the filament
        from both ends, so the advance waits it out instead.

        The re-select before the assist is required: the pull is native to the
        switch cycle, and every load that reaches the toolhead sensor has it.

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
        # Read the pull counter BEFORE the mode change that causes the pull,
        # so a pull that completes quickly cannot be missed between the two.
        # THE SELECT IS WHAT CAUSES THE PULL, AND THE PRINTER DOES NOT SEND IT.
        #
        # The unit's own words: ours is "pull sucess,MODE CHANGE,mode:4". The
        # printer's AMS has no mode change to make -- it reaches mode:4 as the
        # natural end of its own feed ("feed finish 0, mode:4") because the
        # printer stops driving and lets it finish, sending no select and no
        # assist at the arrival, and its good load contains no pull at all.
        #
        # This line was restored because every load in the logs that reached
        # the sensor had it and none without it did -- but those loads also had
        # bridge_finish, the kicks and the stop, all at once, so the result was
        # attributed to the one line most recently put back. The printer's
        # evidence is direct where ours is correlational.
        #
        # Kept ON by default because it IS in the loads that work, and turned
        # off with AFC_BAMBU_ARRIVAL SELECT=0 so the two can be compared on the
        # machine instead of argued about.
        seq0 = self._pull_seq_now()
        aseq0 = self._assist_seq_now()
        if self.arrival_select:
            self.select_lane(cur_lane)        # mode-09 -> mode:4 (loaded)
        # The printer waits ~4s here before it says anything to the unit.
        if self.arrival_assist_delay_s > 0:
            try:
                afc.reactor.pause(
                    afc.reactor.monotonic() + self.arrival_assist_delay_s)
            except Exception:
                pass
        self.set_feed_assist(cur_lane, True)  # hold mode:4 via AP2 sync
        # WAIT FOR THE PULL TO ACTUALLY FINISH -- do not guess at it.
        #
        # The unit's pull-and-push happens HERE, on the mode change into mode:4.
        # It is native, and the extruder must not advance into it. A blind timer
        # cannot win, measured on hardware:
        #
        #     assist -> pull END: 2.02s   pull took 0.9s   back 58mm
        #     pull_settle_s was 2.00s
        #
        # The advance started TWENTY MILLISECONDS before the unit finished
        # pulling, and the captures show the pull ranging 0.5-2.2s, so a slower
        # one is started into properly. That is the operator's read exactly:
        # "we don't wait for the AMS' natural pull back and push back forward
        # to happen before we start tugging away with the extruder".
        #
        # So watch for the unit to SAY it is done ("pull sucess,mode change,
        # mode:4") and use pull_settle_s only as the ceiling. A unit that never
        # says it -- or a dialect that does not narrate -- still proceeds, on
        # the same wait it had before.
        self._wait_for_pull(seq0, aseq0)
        if tool_stn > bite:
            afc.move_e_pos(tool_stn - bite, speed, "tool stn")

    def unit_load_lane(self, cur_lane: Any, cur_extruder: Any = None) -> bool:
        """
        Load a lane to the toolhead, with the follower tick held off throughout.

        The guard is the point of this wrapper. cur_lane.status only becomes
        TOOL_LOADED at the very END of the load, so throughout the arrival
        _tool_loaded_lane() answers None; the follower tick reads that as
        "nothing loaded here anymore", drops the assist the load path just
        armed, and re-arms it when the status lands -- three mode changes in
        two seconds at a unit that is loading correctly.

        try/finally rather than a flag set at each exit: this method has five
        returns and can raise, and a load that leaves the guard set would
        silence the follower for the rest of the session.

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
        # A verdict belongs to ONE load. Left set, the previous failure's words
        # would describe this one's.
        self._declared_fault_text = None
        # Same for the odometer range: how far the AMS moved filament during
        # THIS load is what says whether the failure is upstream or downstream
        # of it.
        self._load_odom_lo = None
        self._load_odom_hi = None
        # Re-arm auto recovery: one attempt per FAULT, not per print -- but
        # NEVER from inside the recovery's own attempt. THIS line is the one
        # that made the recovery retrigger itself: the attempt runs CHANGE_TOOL,
        # which lands here, which cleared the guard the attempt was holding.
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
        # Where the odometer stands BEFORE any filament moves. The distance to
        # the toolhead sensor is the delta from here, not the raw reading --
        # see _measure_path_from_odom.
        self._load_odom_start = self._odom_now_mm()
        self._load_odom_at_sensor = None
        # Freshness mark for the spool measurement this load may produce, so a
        # reading left in the bridge from an earlier load is not adopted as
        # this one's.
        self._load_t0 = afc.reactor.monotonic()
        self.feed(cur_lane, feed_dist)
        loaded = self._feed_until_sensor(cur_lane, timeout)
        # Printer "Retry": the AMS ran its own retries and still stalled (likely a
        # latched state:7). Re-home the AMS (mode 0F/0E reset) and feed again --
        # exactly what pressing Retry on the printer does (re-home -> re-feed,
        # missing once then succeeding in the captured recovery). Bounded.
        # ONE RETRY AT A UNIT THAT HAS DECLARED, NOT TWO.
        #
        # The re-home IS the right answer to a latch -- mode 0F/0E is what the
        # printer's Retry sends, and the capture shows it missing once and then
        # succeeding. What it is not is something to do twice.
        #
        # The first window is long (bulk_time + load_retry_timeout) because it
        # has to contain the AMS's OWN feed/stall/retract/retry cycles. By the
        # time the unit has declared, it has spent them: it says so, and that is
        # what the declaration means. Measured on the HT, lane23 -- the fault
        # break fired correctly 7ms after "TIMEOUT error 0", and then the
        # recovery opened two more full 101s windows at a unit that never moved
        # or spoke again until both had expired:
        #
        #     10:46:05  break on the fault           (5 kicks -- correct)
        #     10:46:08  recover 1/2, 26 kicks, 101s  (nothing)
        #     10:47:53  recover 2/2, 26 kicks, 101s  (nothing)
        #     10:49:34  failed
        #
        # 3.5 minutes, of which the part that worked was the first six seconds.
        # A unit that did not answer the first Retry will not answer the second.
        attempts = self.load_recover_attempts
        if self._declared_fault_text and attempts > 1:
            attempts = 1
            self.logger.info(
                f"AFC bambu {self.name}: {cur_lane.name} -- the AMS declared a "
                f"fault, so this gets ONE re-home retry rather than "
                f"{self.load_recover_attempts}; a unit that ignores the first "
                f"will ignore the second.")
        recover = 0
        while not loaded and recover < attempts:
            recover += 1
            self.logger.info(
                f"AFC bambu {self.name}: load of {cur_lane.name} stalled; "
                f"re-homing AMS and retrying (recover {recover}/"
                f"{attempts})")
            self.stop()                          # halt before the reset motion
            self.rehome()                        # ~3s mode-0F/0E re-home reset
            # Re-baseline AFTER the re-home: mode 0F/0E is a reset and the
            # odometer moves with it, so a delta measured from before it would
            # be the re-home's motion plus the load's, not the path.
            self._load_odom_start = self._odom_now_mm()
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
            # SAY WHAT THE UNIT SAID. A load that ended because the AMS gave up
            # is a different failure from one that ran out of window, and only
            # the first sentence of this message was ever true of both. Sending
            # an operator to re-measure their bowden because the HT reported
            # "TIMEOUT error 0" is worse than saying nothing.
            if self._declared_fault_text:
                cause = (f"The AMS declared a fault during the load and stopped "
                         f"trying; {tail}.\nAMS said: "
                         f"{_fault_reason(self._declared_fault_text)}\n"
                         f"Clear the path at the unit, then re-run the load.")
            else:
                cause = (f"Filament did not reach the toolhead sensor for "
                         f"{cur_lane.name} within {timeout:.0f}s; {tail}.\n"
                         f"Check the filament path and afc_bowden_length "
                         f"calibration.")
            # WHERE, from the unit's own odometer. It knows how far the filament
            # went and we never asked, so a load that fed five metres onto the
            # floor -- the PTFE tube was not connected to the HT -- was reported
            # as "check your afc_bowden_length calibration". The number was in
            # the status frame the whole time.
            #
            # Nothing here claims to detect a disconnected tube specifically.
            # Going far PAST the tube length would say it, but the unit it
            # happened on reports tube_len 0.000 (never learned), so the one
            # measurement of this failure cannot calibrate that test. The
            # documented upstream/downstream split is what is verified, and the
            # raw span is quoted alongside it so the operator can see 5004mm on
            # a 1.9m tube and draw the conclusion the code will not.
            try:
                where = self._jam_location(self._load_odom_span_mm())
            except Exception:
                where = ""
            if where:
                cause = f"{cause}\n{where}"
            afc.error.handle_lane_failure(
                cur_lane, cause, pause=afc.function.in_print())
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
        # NO RE-SELECT AT THE ARRIVAL. The printer never says it here, and the
        # unit told us exactly what hearing it costs. At the sensor moment, in
        # the unit's own narration:
        #
        #   16:14:54  set ams state assist, mode:4     <- the ASSIST alone did it
        #   16:14:55  [AMS_RFID]STEP:odom reset tray 2
        #   16:14:55  err_code:0x25
        #   16:14:55  AMS_CTRL_switch start
        #             need to pull tray:2, tray_sw:3   <- the unit PULLS the tray
        #
        # A mode-09 feeder select means "begin the switch cycle for this tray",
        # and the switch cycle STARTS BY PULLING THE TRAY BACK toward the
        # switch. With the filament pinned in the toolhead that pull is the
        # spool trying to unwind, then the jam. In the clean capture,
        # AMS_CTRL_switch start appears exactly once per drive -- at the START
        # (16:32:13) -- and NEVER at the arrival; the printer's arrival is
        # 09/A5 on the state channel and nothing else.
        #
        # The tray is already selected: unit_load_lane selected it before the
        # feed began. And the narration above shows mode:4 was ALREADY UP from
        # the assist before the old re-select went out -- the thing the
        # re-select was believed necessary for. (The old comment claimed
        # "select <loaded-tray> -> mode:4, confirmed live"; what it observed
        # was the assist doing the work a moment earlier.)
        #
        # THE RULE, from the operator, after a night that proved it: we have
        # the printer's language and cadence on disk -- speak it in the proper
        # situations, and do not invent utterances it never makes.
        try:
            self._advance_into_extruder(cur_lane, cur_extruder)
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
        # Adopt the path measurement AGAIN, now that this load has finished.
        #
        # The first call happens at the top of this method, before the feed --
        # but the unit narrates its tube_len at the END of a load, so that call
        # can only ever see the PREVIOUS one. Adoption therefore needed two
        # loads in a single Klipper session, and the measurement lives only in
        # the bridge's memory, so any restart in between put it back to square
        # one. On this rig an AMS 2 reported 3532 mm on two consecutive loads
        # and stayed on the 3000 mm default throughout, purely because a deploy
        # landed between them.
        #
        # Calling it here makes a single load enough and survives a restart.
        # Cheap and safe to call twice: it is latched to once per session and
        # only writes when the figure differs by more than the tolerance.
        #
        # ODOMETER FIRST, narration second. The odometer is a typed field every
        # unit fills in, so it works on all three; tube_len is text and the
        # AMS 1 does not use the word (measured alone on the wire at a 100%
        # drain answer rate -- that is the unit, not our listening). The
        # fallback keeps the units that DO narrate it working.
        _mm, _src = self._path_measurement()
        self._adopt_measured_path(_mm, _src)
        afc.save_vars()
        return True

    def _published_slots(self) -> list:
        """
        The slots this unit reports, trimmed to the bays it actually has.

        AN HT HAS ONE BAY, EVER. The internal arrays are SLOTS_PER_UNIT (4)
        wide on every unit type deliberately -- the bridge indexes them by slot
        number and a short array would fault on a stray frame naming slot 3 --
        so the trim belongs on the way OUT, not in the storage.

        Without it an HT reported four bays to Mainsail, to `slots=` consumers
        and to anything counting bays:

            BambuAMS_HT  online=True idx=2 slots=4 present=1

        :return list: the slot records for bays this unit has
        """
        # SCANNING IS NOT "NO TAG". A consumer sees `present` with an empty
        # `material` and has no way to tell "the reader looked and found
        # nothing" from "the reader is looking RIGHT NOW" -- so it renders the
        # conclusion during the work. The BB Master panel does exactly that:
        #
        #     present && !material  ->  BB_BAY_UNTAGGED  ->  "No tag read"
        #
        # and shows it for the whole 12 s a read-plus-measure takes, because the
        # scan blanks the bay's material on the way in. Watched on hardware: the
        # tag authenticated 1 s into the cycle and the panel still said "No tag
        # read" until the measurement landed 12 s later.
        #
        # The unit knows the difference and now says so. `scanning` is true from
        # the moment the scan is armed until its window closes, which is the one
        # period where an empty material means "not yet" rather than "none".
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
            # Mark BEFORE the first retract: everything past this is a fault
            # this unload provoked, not one left lying around from the load
            # that preceded it.
            fault0 = self._ams_fault_seq()
            self.retract(cur_lane, retract_dist)
            self._wait_move(retract_dist, fault_mark=fault0)

            # VERIFY the filament actually left the toolhead — a fire-and-
            # forget retract that the AMS ignores (busy/error state) would
            # otherwise report "unload done" with the filament never having
            # moved. If the toolhead sensor still sees filament,
            # re-kick with the eject discipline (stop -> select -> retract),
            # and fail loudly if it still won't clear.
            #
            # THREE CHECKS, TWO RE-KICKS. range(2) would check the sensor at
            # the top of each pass and re-kick at the bottom, so the SECOND
            # re-kick's result is never read -- a lane the second attempt
            # genuinely freed still falls into handle_lane_failure. The extra
            # pass is a sensor check only;
            # `attempt == RETRIES` breaks before kicking again.
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
                # STOP ASKING A UNIT THAT HAS GIVEN UP.
                #
                # Only consulted once the sensor says something IS wrong, which
                # is what makes it safe to act on: an unload retracts against
                # resistance by design and a lone stall mid-reel is ordinary,
                # so the unit's report is worthless as a trigger on its own.
                # Here it is a verdict on a failure we have already observed.
                #
                # Measured on an AMS 2 (lane20, filament left in the gears):
                # the unit retried three times itself -- "pull err,bdc stall"
                # at 0.042m, 0.001m, 0.000m -- then escalated to "TIMEOUT
                # error 2/3" and latched. Every kick after that point went to
                # a unit that had stopped listening.
                #
                # Consumed, not peeked: we report it ourselves below, and the
                # follower tick must not raise the same event again once the
                # unload's mute lifts.
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
                    f"state: {ams_state}) — re-kicking (stop/select/retract, "
                    f"attempt {attempt + 1})")
                self.stop()
                self.select_lane(cur_lane)
                self.retract(cur_lane, retract_dist)
                self._wait_move(retract_dist, fault_mark=fault0)
            if not cleared:
                if latched:
                    # Halt ONLY when the unit said it gave up. A failure with
                    # no verdict leaves it alone deliberately -- it may still
                    # be reeling, and a stop there aborts a retract that could
                    # yet succeed. A latched unit is not reeling; it is
                    # streaming a retract it will never finish, which is the
                    # same ~2 Hz "there is no tray" noise the stop on the
                    # success path exists to end.
                    self.stop()
                    msg = (f"AFC bambu unload failed for {cur_lane.name}: the "
                           f"AMS gave up reeling it back and latched — the "
                           f"filament is still gripped at the toolhead. Free "
                           f"it by hand (heat, then retract), then run "
                           f"{self.get_lane_reset_command(cur_lane, 0.0)}.\n"
                           f"AMS said: {_fault_reason(latched)}")
                else:
                    msg = (f"AFC bambu unload failed for {cur_lane.name}: "
                           f"filament still at the toolhead sensor after "
                           f"retract retries — AMS did not reel; run "
                           f"LANE_UNLOAD (eject) or check the unit")
                afc.error.handle_lane_failure(
                    cur_lane, msg, pause=afc.function.in_print())
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
        AFC_BAMBU_RECOVER (stop + reel to bay + reset state), the same recovery the
        eject path uses. Mirrors AFC_ACE.get_lane_reset_command.

        :param lane: Lane to reset
        :param dis: Reset distance (unused; the recover reels the full path)
        :return str: the AFC_BAMBU_RECOVER command for this unit and lane
        """
        return f"AFC_BAMBU_RECOVER UNIT={self.name} LANE={lane.name}"

    #: Bounds on a path length derived from the odometer, in mm. Not a
    #: measurement of anything -- purely a garbled-read guard. The three tubes
    #: on this rig read 3346 (AMS 1), ~3500 (AMS 2) and ~3660 (HT), and the
    #: capture rig's were 1693 and 2186, so the range is deliberately wide
    #: enough to hold every real machine and narrow enough to reject a sign
    #: flip or a stuck sentinel.
    ODOM_PATH_MIN_MM = 300.0
    ODOM_PATH_MAX_MM = 8000.0

    def _odom_now_mm(self) -> Optional[float]:
        """
        This unit's odometer position in mm, from the BINARY STATUS FRAME.

        NOT NARRATION, which is the entire point. tube_len and dw_len are text,
        and the three units disagree about the words: the AMS 2 Pro and the HT
        narrate tube_len, the AMS 1 narrates neither -- measured alone on the
        wire at a 100% drain answer rate, so that is the unit and not our
        listening. The odometer is a typed field every unit fills in, so a path
        length taken from it works on all three without knowing any dialect.

        Negative is a READING, not an error: the resting position measures
        slightly below zero and an unload runs frankly negative. Only the
        firmware's -1 mm unknown-sentinel is excluded, matching get_status.

        :return float: position in mm, or None if the unit has not reported one
        """
        try:
            latest = self._bridge.latest_status() if self._bridge else None
            for u in (latest or {}).get("units") or []:
                if int(u.get("n", -1)) != int(self.ams_index):
                    continue
                v = u.get("odom")
                if v is None or int(v) == -1:
                    return None
                return float(v)
        except Exception:
            pass
        return None

    def cmd_AFC_BAMBU_SLOTTRACE(self, gcmd: Any) -> None:
        """
        Record BOTH sides of the slot-info traffic for a few seconds.

        AFC_BAMBU_SLOTTRACE UNIT=<unit> [S=<seconds>]

        Answers "what keeps asking for that bay?" with evidence instead of
        inference. Prints, per unit and bay: whether we believe a spool is
        present, whether our cached tag info is valid (the ONLY thing that
        makes the firmware's background fill read a bay), and the firmware's
        read counters -- then the same again after the window, so what MOVED is
        visible rather than what merely IS.

        Built because an empty AMS 1 was seen being polled repeatedly and
        neither side's state was recorded at the time, so the question could
        only be argued about.

        :param gcmd: The Klipper GCodeCommand

        Usage
        -------
        `AFC_BAMBU_SLOTTRACE UNIT=<unit> S=<value>`

        Example
        -------
        ```
        AFC_BAMBU_SLOTTRACE UNIT=BambuAMS_1 S=5.0
        ```
        """
        secs = gcmd.get_float("S", 5.0, minval=1.0, maxval=60.0)
        if self._bridge is None:
            raise gcmd.error("AFC_BAMBU_SLOTTRACE: bridge not connected")

        def snap():
            latest = self._bridge.latest_status() or {}
            rows = []
            for sl in (self._slots or []):
                rows.append((sl.get("index"), bool(sl.get("present")),
                             sl.get("state"), sl.get("rfid_uid"),
                             sl.get("remain_pct")))
            return rows, (latest.get("dbgpolls"), latest.get("dbgframes"),
                          latest.get("dbgtexts"), latest.get("dbgtrunc"))

        a_rows, a_ctr = snap()
        self.afc.reactor.pause(self.afc.reactor.monotonic() + secs)
        b_rows, b_ctr = snap()
        gcmd.respond_info(
            f"AFC_BAMBU_SLOTTRACE {self.name} over {secs:.0f}s -- "
            f"polls {a_ctr[0]}->{b_ctr[0]}  frames {a_ctr[1]}->{b_ctr[1]}  "
            f"texts {a_ctr[2]}->{b_ctr[2]}  cut {a_ctr[3]}->{b_ctr[3]}")
        for (i, p, st, uid, pct) in b_rows:
            was = next((r for r in a_rows if r[0] == i), None)
            moved = " CHANGED" if was != (i, p, st, uid, pct) else ""
            gcmd.respond_info(
                f"  bay {i}: present={p} state={st} uid={uid} "
                f"remain={pct}{moved}")

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

        THE HT HAS NO OTHER SOURCE. It reports odom as None -- 225 consecutive
        samples across a full load, measured -- and has never narrated the word
        tube_len here. What it does say, at the end of every feed, is
        "dw_len:3.672 m". Two loads gave 3.661 and 3.672, 11 mm apart, against
        a configured 3679: repeatable, and agreeing with the other method's
        accuracy on the AMS 1 (3338 measured vs 3346 read off a 1 Hz sampler).

        ADDRESS-CHECKED, because the unit key alone is not enough. dw_len is
        filed under _active_unit, which a load SETS and nothing clears, so it
        names whichever unit loaded last -- correct during a load, stale after
        one. A value that arrived on a device address this unit does not use is
        refused. That catches HT-vs-boxed (0x1800 vs 0x0700); the two boxed
        units share an address and cannot be separated this way, which is what
        the unit key is for.

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
        THIS unit's own PTFE length, from whichever source THIS unit provides.

        The three units are different tubes and different dialects, and no
        single source covers them all, so the chain falls through per unit
        rather than naming one source authoritative. Order is by directness:

            odometer   distance actually travelled to the toolhead sensor
                       THIS load
            dw_len     the unit's own end-of-feed figure for that journey
            tube_len   a stored self-calibration; last, because no unit here
                       has been seen to narrate it

        A missing source means apply nothing: (None, "") is a real answer and
        the caller writes nothing, leaving the configured value. Do not
        substitute a derived, averaged or defaulted figure -- odom_m reads
        None on two of three units at any given moment.

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
            # By UNIT first: two units of the same class share a device
            # address (an AMS 1 and an AMS 2 Pro are both 0x0700), so the
            # address alone would let one unit's measurement be adopted as the
            # other's bowden length. Address is the fallback for a single-unit
            # bus, where it has always been unambiguous.
            return br.tube_len(getattr(self, "dry_dev_addr", None),
                               unit=self.ams_index)
        except TypeError:
            return br.tube_len(getattr(self, "dry_dev_addr", None))
        except Exception:
            return None

    def _queue_spool_summary(self, slot: int, pct: int, grams: int,
                             nominal: int) -> None:
        """
        Hold the operator's summary until the bay's record can answer it.

        THE MEASUREMENT FINISHES BEFORE THE RECORD CATCHES UP, BY DESIGN. Our
        firmware does not read 0x0211 during the capacity window -- hammering
        it mid-scan aborts the feed -- so it clears ``info_valid`` at the
        window close and lets the round-robin fill collect the result after.
        The measurement result arrives from NARRATION, which is why it is here
        first.

        Said immediately, the line reads the record in exactly the gap it is
        being refreshed through, and announces the blank as a conclusion:

            13:15:xx  STEP:card auth success! ... read success,valid
            13:15:xx  slot 1 calibration completed -- re-reading the bay
            13:15:xx  lane16: NO TAG ON THIS SPOOL. Measured about 25% left
            13:16:xx  applied tag to lane16: Bambu PLA Sparkle #2D2B28

        The unit had just narrated a successful read. Nothing was wrong except
        the moment we chose to speak.

        So: if the record already answers, or the UNIT says no tag read during
        this scan, say it now. Otherwise wait for the re-read -- _sync_lanes
        drains this as soon as the record lands.

        :param slot: bay index the measurement belongs to
        :param pct: measured remaining percent, RAW (may exceed 100)
        :param grams: remaining weight in grams, already capped
        :param nominal: the spool's full weight in grams
        """
        pend = getattr(self, "_pending_summary", None)
        if pend is None:
            pend = self._pending_summary = {}
        try:
            deadline = self.afc.reactor.monotonic() + self.SCAN_FALLBACK_CAP
        except Exception:
            deadline = None
        pend[slot] = (pct, grams, nominal, deadline)
        self._drain_spool_summary(slot)

    def _drain_spool_summary(self, slot: int) -> None:
        """
        Say a held summary once the bay's record can answer it, or give up.

        :param slot: bay index the measurement belongs to
        """
        pend = getattr(self, "_pending_summary", None) or {}
        held = pend.get(slot)
        if not held:
            return
        pct, grams, nominal, deadline = held
        info = {}
        for sl in (self._slots or []):
            if sl.get("index") == slot:
                info = sl or {}
                break
        # WAIT FOR THE RECORD, full stop. No reasoning about which outcome this
        # is -- the record answers or the backstop expires.
        #
        # There is no impatient case to optimise for. An untagged spool is
        # never measured (the unit declines), so nothing reaches here without
        # a tag; and a third-party tag has a UID even when its profile will not
        # decode, which satisfies this immediately. The wait only ever covers
        # our own poll catching up.
        ready = bool(info.get("material") or info.get("rfid_uid"))
        if not ready:
            try:
                ready = (deadline is None
                         or self.afc.reactor.monotonic() >= deadline)
            except Exception:
                ready = True
        if not ready:
            return
        pend.pop(slot, None)
        self._say_spool_summary(slot, self._lane_for_slot(slot),
                                pct, grams, nominal)

    def _say_spool_summary(self, slot: int, lane: Any, pct: int,
                           grams: int, nominal: int) -> None:
        """
        One plain-English line for the operator when a spool is measured.

        Says what was read, how much is left, and where it went -- the three
        things worth knowing at the machine, in one line. The underlying facts
        are otherwise spread across three machine-shaped log lines seconds
        apart, each phrased for whoever wrote the code.

        :param slot: bay index the measurement belongs to
        :param lane: the AFC lane, or None
        :param pct: measured remaining percent, RAW (may exceed 100)
        :param grams: remaining weight in grams, already capped
        :param nominal: the spool's full weight in grams
        """
        info = {}
        for sl in (self._slots or []):
            if sl.get("index") == slot:
                info = sl or {}
                break
        where = getattr(lane, "name", None) or f"bay {slot}"
        # What the tag said, if there was one. A no-tag spool is ordinary --
        # third-party reels have none -- so it is stated, not warned about.
        material = info.get("material")
        colour = info.get("color")
        uid = info.get("rfid_uid")
        if material:
            what = f"{material}" + (f" ({colour})" if colour else "")
            read = f"tag read: {what}"
            if uid:
                read += f" [tag {str(uid).upper()}]"
        elif uid:
            # A TAG WE CANNOT DECODE IS STILL A TAG, and its UID is the thing
            # that binds it in Spoolman. Third-party reels carry a plain Mifare
            # chip: the anticollision returns a UID perfectly well, only the
            # Bambu profile fails to parse. Saying "no tag on this spool" there
            # was wrong AND unhelpful -- it hid the one value the operator needs
            # to type into Spoolman to make the spool track from then on.
            read = (f"tag {str(uid).upper()} read but its profile could not be "
                    f"decoded (not a Bambu tag?) -- bind that UID to a spool in "
                    f"Spoolman and it will match from now on")
        else:
            read = "no tag on this spool"
        # Over 100% is the unit measuring a spool slightly proud of its
        # reference full radius, not extra filament. Say that in words rather
        # than printing a number the operator has to know how to discount.
        if pct > 100:
            amount = (f"full -- roughly {grams} g of a {nominal} g spool "
                      f"(the AMS read {pct}%, meaning it measures a little "
                      f"larger than a reference full spool)")
        else:
            amount = (f"about {pct}% left -- roughly {grams} g of a "
                      f"{nominal} g spool")
        # Where the number went. Spoolman is the interesting case because its
        # absence is silent everywhere else, and that silence has already cost
        # two rounds of "why didn't it write?".
        sid = getattr(lane, "spool_id", None)
        if not getattr(self, "sync_measured_to_spoolman", True):
            went = "Spoolman sync is off, so this is kept on the lane only"
        elif sid in (None, "", 0):
            went = ("not linked to a Spoolman spool, so this is kept on the "
                    "lane only")
        else:
            went = f"updated Spoolman spool {sid}"
        self.logger.info(
            f"{self.name} {where}: {read}. Measured {amount}; {went}.")

    def _adopt_measured_remain(self, slot: int, pct: int,
                               source: str = "capscan") -> bool:
        """
        Record a physical spool measurement: slot remain%, lane grams, Spoolman.

        The single place a measured percent becomes state, so the capacity scan
        and a load-time reading cannot drift apart in how they round, cap or
        report it.

        :param slot: slot index the measurement belongs to
        :param pct: remaining percent, RAW (may legitimately exceed 100)
        :param source: what produced it, for the log line
        :return bool: True if the measurement was accepted
        """
        if not (0 < pct <= 150):
            return False
        # Persist module-side: self._slots is REPLACED by every bridge status
        # frame, so an in-place edit survives one pass at most. The dict is
        # re-applied on every pass in get_status.
        if not hasattr(self, "_measured_remain"):
            self._measured_remain = {}
        self._measured_remain[slot] = pct
        nominal = 1000
        for sl in (self._slots or []):
            if sl.get("index") == slot:
                nominal = sl.get("weight") or 1000
                break
        lane = self._lane_for_slot(slot)
        # Grams are CAPPED at the tag's nominal weight even when the
        # measurement reads over 100%. The percent itself is honest -- the unit
        # derives it from the filament's cross-sectional area (R^2 - core^2),
        # and a fit across four measurements on two unit types matches to 0.2%:
        #     core 47.5 mm hub radius, P=100% at R = 82.6 mm.
        # So 102-119% means a full spool sitting slightly proud of that
        # reference geometry, NOT 19% more filament than the tag declares.
        # Believing it would write 1190 g onto a 1 kg spool.
        grams = max(1, (int(nominal) * min(pct, 100)) // 100)
        if lane is not None:
            lane.weight = grams
        self._queue_spool_summary(slot, pct, grams, nominal)
        # Push the PHYSICAL measurement back to Spoolman for a bound spool --
        # the loop-closer. The AMS measured this spool by radius, which is
        # truer than extrusion accounting, so the bound spool's
        # remaining_weight is corrected to what is actually on it. Covers a
        # no-tag spool too: it still gets measured, so a manually-bound
        # part-used spool stops looking full. Gated so a user who does not want
        # the AMS writing Spoolman can opt out.
        self._push_measured_to_spoolman(lane, grams)
        return True

    def _measure_path_from_odom(self) -> Optional[float]:
        """
        The bay-to-toolhead-sensor distance this load just travelled, in mm.

        The odometer counts filament and is available on every unit, so it is
        the measurement. Both ends of the delta are taken at defined moments --
        before any motion, and at the instant the toolhead sensor trips --
        because the odometer keeps climbing afterwards as the toolhead consumes
        filament through tool_stn and the purge.

        A DELTA, not the raw reading: the odometer is a POSITION, zero only if
        the previous unload finished cleanly. A lane staged at the hub starts
        partway along, so the raw value would record a path shorter than the
        real one.

        Not taken from the unload: that trace runs 0 -> -0.999 and resets to 0
        without ever spanning the tube, so there is no path length in it.

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

        :param measured: length in mm to adopt; None looks one up
        :param source: short label for where the figure came from, for the log

        afc_unload_bowden_length follows only if it was tracking the bowden
        length (its default). Somebody who set it deliberately keeps it.
        """
        if getattr(self, "_path_adopted", False):
            return
        if measured is None:
            measured, source = self._path_measurement()
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
               f"path at {measured:.0f}mm (was {old:.0f}mm)"
               f"{' via ' + source if source else ''}. Adopting it -- "
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

        NOT the same question as which extruder is SELECTED -- the toolhead's
        selected extruder does not change across a restart or a home, so only
        the energised state distinguishes them.

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

    def _ready_to_follow(self, lane: Any = None) -> bool:
        """
        Always True. A loaded tray is followed, full stop.

        Deliberately does NOT gate on the extruder motor being energised. A
        real Bambu holds a loaded tray with op-04 mode 07 / ref 7F continuously
        (~100% of the time a tray is in) and never asks whether a stepper is
        energised; gating on it means idle timeout drops the steppers, the
        follower stands down, and filament pulled by hand is never recovered --
        indistinguishable from a broken follower.

        If feeding into un-gripped gears does pack the buffer on some hardware,
        that shows up as a stretched buffer at idle, which the unit reports as
        buff on every poll. Observe it rather than pre-empting it with a gate
        the printer does not have.

        The signature is kept so callers and tests do not change.

        :param lane: unused; kept for call-site compatibility
        :return bool: True
        """
        return True

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
        try:
            self.stop()
            self.select_lane(lane)
            dist = self._eject_distance()
            fault0 = self._ams_fault_seq()
            self.retract(lane, dist)
            finished = self._wait_move(dist, fault_mark=fault0)
            self.stop()
            # The unit's own verdict, if it reached one. Consuming it here is
            # what makes this path REPORT the latch instead of leaking it: both
            # detectors are muted while _unload_in_progress, so before this the
            # fault sat in the bridge until the mute lifted and the next
            # follower tick happened to pick it up -- measured 22s late, and
            # silently lost altogether if anything else consumed the sequence
            # first.
            latched = self._ams_fault_since(fault0)
            if latched:
                self.logger.warning(
                    f"AFC bambu {self.name}: "
                    f"{getattr(lane, 'name', 'lane')} could not be reeled back "
                    f"-- the AMS gave up and latched. The filament is still "
                    f"out of the bay; free it by hand before loading this lane "
                    f"again.\nAMS said: {_fault_reason(latched)}")
            elif not finished:
                # This path has NO sensor: when the AMS reports a completion we
                # know the filament is home, and when it does not, _wait_move
                # returns on its deadline and the stop() above is what ends the
                # retract. Time x the AMS's own speed then decides how far it
                # actually came back -- which may be short. Silently treating
                # that as success is how a lane ends up half-ejected with
                # nothing in the log, so say it plainly.
                # getattr: eject is a RECOVERY path -- it runs from
                # AFC_BAMBU_RECOVER and AFC_RESET when things are already wrong,
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
