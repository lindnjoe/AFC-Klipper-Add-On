# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# Bambu AMS bridge TRANSPORT: the link to the Pi Pico that masters the AMS's
# RS-485 bus, and the wire-format helpers that go with it.
#
# The transport is a threaded serial reader with reconnect, a newline-JSON
# protocol, and the parsing of the AMS's own plain-text narration. None of it
# needs a printer, a lane or a config section: everything here is import-safe
# and can be driven with a fake serial port.
#
# The unit driver (AFC_BambuAMS.py) imports from here and re-exports the public
# names, so existing configs and any code importing them are unaffected.
from __future__ import annotations
import json
import logging
import logging.handlers
import os
import re
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple


class _TruncatingRotatingFileHandler(logging.handlers.RotatingFileHandler):
    """RotatingFileHandler that bounds the file when keeping no backups.

    The stdlib handler only RENAMES on rollover, and the rename is guarded by
    `if self.backupCount > 0`; with backupCount=0 it reopens the stream in
    append mode and the file grows without bound. Truncating on rollover gives
    a rolling window for diagnosis that cannot fill an SD card.
    """

    def doRollover(self) -> None:
        if self.stream:
            self.stream.close()
            self.stream = None
        if self.backupCount > 0:
            return super().doRollover()
        mode, self.mode = self.mode, "w"
        try:
            self.stream = self._open()
        finally:
            self.mode = mode


# ── Pure helpers (unit-tested; no Klipper/hardware needed) ──────────────────────

def parse_bridge_line(line: str) -> Optional[dict]:
    """
    Parse one newline-JSON line from the bridge into an event dict.

    :param line: A single line of text from the Pico (without the newline)
    :return Optional[dict]: the decoded object, or None if blank/invalid
    """
    line = line.strip()
    if not line:
        return None
    try:
        obj = json.loads(line)
    except (ValueError, TypeError):
        return None
    return obj if isinstance(obj, dict) else None

# "bldc_i:0.319A" -- the AMS's motor current, reported on feed/switch
# lines. Parsed here because it arrives in the unit's narration.
_BLDC_I_RE = re.compile(r"bldc_i:([0-9.]+)A")


# ── Plain-English narration ──────────────────────────────────────────────────
# The AMS narrates in its own terse format: a refused dry command reads
# "[AMS_CHMB]ignore dry_mode:1, ams_state:2", which distinguishes "busy, try
# again" from an outright failure -- a distinction the return code does not
# carry, since a refusal and a success both report success.
#
# Deliberately small. Everything goes to AFC.log verbatim; only entries matched
# here reach the console, and the chatty drying telemetry is rate-limited
# separately. Anything not matched stays silent.
# The AMS's drying telemetry, emitted every ~10s while a cycle runs:
#
#   [AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|hts:34,31,00|pw:100|ad:2|...
#     s  = chamber state code       rf = target C (a second field follows it)
#     vt = chamber probe C          ap = a second, lower-reading probe
#
# Groups: state, target, chamber, humidity. Humidity is optional: on both the
# AMS 2 Pro and the AMS HT the comma follows `rf` and `vt` terminates with a
# pipe, but a model that does attach humidity is still read.
#
# TWO SEPARATORS, both valid. The rendering follows the ADDRESSING, not the
# model: once the drain is addressed to each unit's OWN device (per-unit MC
# addressing) the units emit a comma-separated, space-padded form carrying an
# extra `cd:` field:
#
#   [AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0, hts:46,23,0 pw:100, ad:2, ...
#
# So either separator is accepted and `cd:` is skipped where present. `rf`'s
# optional `,N` suffix appears only in the pipe form, and cannot swallow ", cd"
# or ", vt" because a digit must follow the comma.
_CHMB_STATE_RE = re.compile(
    r"\[AMS_CHMB\]s:(\d+)(?:,\d+)?\s*[|,]\s*"
    r"rf:(\d+)(?:,\d+)?\s*[|,]\s*"
    r"(?:cd:\d+\s*[|,]\s*)?"
    r"vt:([0-9.]+)(?:\s*,\s*(\d+))?")
# The AMS announcing its self-calibrated PTFE path length. Two forms, because
# it uses different units in different lines:
#   [AMS_SWITCH]new tube_len:3481 mm, list:3491,3472,0 mm, err:19 mm
#   [AMS_SWITCH]feed finish -1, stall, len_det:3.711 m, tube_len:0.000 m
# The metre form is matched too because it is the one that appears on a STALL,
# which is exactly when knowing the calibrated length is most useful. Both
# report 0 before the unit has enough samples; the caller drops non-positive
# values rather than treating "not yet calibrated" as "zero-length path".
#: A MOTION completion, and only that. The AMS says "finish" about several
#: things that are not a move ending:
#:
#:   [AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, err_code:0x00
#:   [AMS_SWITCH]assist finish 0, ref:0
#:
#: The state_switch one repeats roughly ten times in the seconds BEFORE the
#: real feed completes, so a bare "finish" substring test hands _wait_move a
#: completion while the filament is still travelling -- AFC would call the load
#: done somewhere mid-bowden. Harmless only while the bus was silent; live the
#: moment narration came back.
#:
#: Matches feed/pull/preload finish, with or without a trailing index:
#:   [AMS_SWITCH]feed finish, buff_pos:1.29, bldc_i:1.593A       (HT)
#:   [AMS_SWITCH]feed finish 0, dw_len:3.508 m                   (AMS 2 Pro)
#:   [AMS_SWITCH]pull finish 0, tray_sw:0, len_det:0.265 m
#:   [AMS_PRELOAD]preload finish
#: Matches feed/pull/preload finish, with or without a trailing index, PLUS
#: the AMS 2 Pro's own word for a completed pull. That unit does not say
#: "finish" on the way out:
#:
#:   [AMS_SWITCH]pull sucess,cond match,... bdc_i:0.464A;spd:-20.1cm/s
#:
#: (the misspelling is the firmware's; both spellings are accepted). Without
#: it an AMS 2 Pro's every unload runs the full watchdog, exactly as a boxed
#: AMS's did before the odometer lines were read. From the captures only --
#: no AMS 2 Pro on the rig to confirm it against.
#:
#: Deliberately NOT matched by the "sucess" half: the state-machine line
#: "AMS_CTRL_state_switch finish, sucessful", which occurs 242 times in one
#: night's log. Hence "pull" is required immediately before it rather than
#: looking for the word on its own.
_MOTION_FINISH_RE = re.compile(
    r"\b(?:(?:feed|pull|preload)\s+finish|pull\s+suc+ess)\b",
    re.IGNORECASE)

#: The filament reached the EXTRUDER, as an AMS 2 Pro / HT reports it:
#:
#:   [AMS_SWITCH]e_in tray:0,buff_pos:-0.34,i:0.566A,len:1.670m
#:
#: "e_in" is the unit's own toolhead-sensor equivalent and it fires EARLIER
#: than the feed completion -- in the captured AMS 2 load, e_in at +56.8 s and
#: the path measurement at +57.9 s. It is an arrival, not an end of motion:
#: the capture it comes from went on to fail downstream on a hotend jam, and
#: e_in fired all the same. That is the right semantics here -- it says where
#: the filament is, not that everything went well. Captures only, unconfirmed
#: on hardware.
_EXTRUDER_IN_RE = re.compile(r"\be_in\s+tray:", re.IGNORECASE)

#: Buffer position as an instantaneous reading -- e_in during a load, feed
#: finish at the end of one.
_BUFF_POS_RE = re.compile(r"buff_pos:(-?[0-9]+\.[0-9]+)")

#: A buffer REFILL, which is the ramming event itself:
#:
#:   [AMS_SWITCH]BUFF,pos:0.09->0.74, det:6mm,  i:0.583A
#:   [AMS_SWITCH]BUFF,pos:0.10->0.74, det:28mm, i:0.521A
#:
#: The extruder drew filament, the buffer sagged to `pos` before, the unit fed
#: until it recovered to `pos` after, and `det` is how much filament that
#: took. That is precisely what a buffer-driven ram needs to know, measured by
#: the unit rather than inferred from a sensor: how far it sagged and how much
#: restored it. Note the separate spelling -- "BUFF,pos:" here against
#: "buff_pos:" above -- so one pattern cannot cover both.
#:
#: Recovery is consistently to ~0.74 across every captured sample, with the
#: sag varying (0.09, 0.10) and `det` varying widely (6, 12, 24, 28 mm), which
#: is the shape of a unit refilling to a fixed setpoint on demand.
_BUFF_REFILL_RE = re.compile(
    r"BUFF,\s*pos:(-?[0-9.]+)\s*->\s*(-?[0-9.]+)"
    r"(?:[^\n]*?det:(\d+)\s*mm)?",
    re.IGNORECASE)

#: A motion completion carrying NO failure marker -- the AMS HT's clean end of
#: a load, "feed finish, buff_pos:1.28". Distinguished from "feed finish -1"
#: by requiring what follows to be a comma or end-of-line, so the -1 form
#: cannot match it.
_CLEAN_FINISH_RE = re.compile(r"\b(?:feed|pull|preload)\s+finish\s*(?:,|$)",
                              re.IGNORECASE)

#: How far short of its measured path the AMS may stall and still be counted
#: as arrived, in mm.
#:
#: Sized from both sides, measured on an HT with a 3619 mm path: a NORMAL load
#: ends stalled against the extruder 18 mm short, and a genuinely short unload
#: reported 336 mm short and did need its retry. 100 mm sits an order of
#: magnitude clear of the first and well clear of the second.
FINISH_ARRIVAL_TOLERANCE_MM = 100.0

#: Completion in the [AMS_DEV] dialect, which never says "finish".
#:
#: A boxed AMS narrates its moves in ODOMETER terms, and the transitions are
#: unambiguous. Verbatim, one load and one unload of the same lane:
#:
#:   20:13:55  AFC: lane15 reached the toolhead sensor
#:   20:13:56  [AMS_DEV] STEP:odom reset tray 0          <- feed arrived
#:   20:13:58  set ams state assist, mode:4  (repeats while loaded)
#:   20:15:41  set ams state switch                      <- retract starts
#:   20:16:06  [AMS_DEV] STEP:odom tray_id error 255     <- tray is gone
#:   20:16:16  AFC gave up and used the 35 s watchdog
#:
#: Ten seconds of watchdog we did not need to spend, on every unload. The
#: reset line is cross-dialect -- an HT emits it inside its own finish blob
#: ("...tube_len:3.619 m [AMS_RFID] STEP,odom reset tray 0 ...").
_ODOM_RESET_RE = re.compile(r"odom\s+reset\s+tray\s*\d+", re.IGNORECASE)
_ODOM_NO_TRAY_RE = re.compile(r"odom\s+tray_id\s+error\s*255", re.IGNORECASE)

#: Distance the AMS says it actually moved, in metres.
_LEN_DET_M_RE = re.compile(r"len_det:([0-9]+\.[0-9]+)\s*m\b")

_TUBE_LEN_MM_RE = re.compile(r"tube_len:(\d+)\s*mm")
_TUBE_LEN_M_RE = re.compile(r"tube_len:([0-9]+\.[0-9]+)\s*m\b")


#: A drying command the AMS REFUSED, in its own words:
#:
#:   [AMS_LINK]ams0 dry,req ams 0
#:   [AMS_LINK]ret:1,mode:1,temp:55,time:480      <- our parameters, echoed back
#:   [AMS_CHMB]err, filament hub load!            <- and refused
#:
#: The echo is what proves the command was addressed correctly -- a frame sent
#: to a unit id it does not own draws nothing at all, let alone the values we
#: sent. So a refusal is the UNIT declining, not a delivery failure, and it
#: has to be reported as such: we return success either way, because the
#: command was delivered, and without this the panel just sits at "not drying"
#: with no reason given.
_DRY_REFUSED_RE = re.compile(r"\[AMS_CHMB\]\s*err,\s*([^\[\r\n]{1,60})")

_AMS_HUMAN = (
    (_DRY_REFUSED_RE,
     lambda m: (f"AMS refused the drying command: {m.group(1).strip()}. "
                f"An AMS will not dry with filament out in the hub -- reel "
                f"the lane back to its bay first (LANE_UNLOAD).")),
    (re.compile(r"\[AMS_CHMB\]ignore[^,]*,\s*ams_state:(\d+)"),
     lambda m: (f"AMS refused the drying command -- it was busy "
                f"(state {m.group(1)}). Wait for it to settle and try again.")),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_SELF_CHECK.*?ref:(\d+)"),
     lambda m: f"AMS drying: self-check started, target {m.group(1)}C"),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_HEATING"),
     lambda m: "AMS drying: self-check passed, now heating"),
    (re.compile(r"\[AMS_CHMB\]set state CTC_STATE_OFF"),
     lambda m: "AMS drying: heater off"),
    (re.compile(r"\[AMS_SWITCH\]new tube_len:(\d+) mm.*?err:(-?\d+) mm"),
     lambda m: (f"AMS measured the PTFE path at {m.group(1)}mm "
                f"(+/-{m.group(2)}mm)")),
    # A REGULAR AMS narrates in a different namespace: "[AMS_DEV] STEP:.." and
    # "[RF] .." rather than [AMS_RFID]/[AMS_TRAY]/[AMS_COMMON]. Note the SPACE
    # after the bracket, which the other models do not have. None of the rules
    # below fired on an AMS 1 before these were added -- the firmware half of
    # this project was written watching [AMS_DEV] (its comments say so) while
    # these host rules were written against an AMS 2 or HT.
    (re.compile(r"\[AMS_DEV\]\s*STEP:read success"),
     lambda m: "AMS: tag read OK"),
    (re.compile(r"\[AMS_DEV\]\s*STEP:card auth success"),
     lambda m: "AMS: tag authenticated"),
    (re.compile(r"\[AMS_DEV\]\s*STEP:feed with rfid success"),
     lambda m: "AMS: spool fed and tag read"),
    (re.compile(r"\[AMS_DEV\]\s*STEP,first detected"),
     lambda m: "AMS: spool detected"),
    (re.compile(r"\[RF\]\s*tray(\d+): info write to flash"),
     lambda m: (f"AMS: tag for bay {int(m.group(1)) + 1} cached in the unit's "
                f"flash (a later read returns it even after a swap)")),
    (re.compile(r"\[AMS_RFID\]STEP:read success"),
     lambda m: "AMS read the spool tag"),
    (re.compile(r"\[AMS_RFID\]STEP:select card fail, err (\d+)"),
     lambda m: f"AMS could not read the spool tag (err {m.group(1)})"),
    (re.compile(r"\[AMS_PRELOAD\]preload finish"),
     lambda m: "AMS staged the spool at its feeder"),
)

# Narration that means the AMS has COMMITTED to the physical tag read -- it has
# pulled the filament off the switch and is working the reader. Used to hold off
# the "no readable tag, apply lane defaults" fallback while a read is genuinely
# still running.
#
# The distinction is measured, not guessed. Two consecutive inserts in the same
# bay of the same AMS 1 (AFC.log 16:45:32 and 16:46:18):
#
#   insert #1  tray_preload -> tray_readid xN -> SILENCE. Never pulled the tray.
#              Nothing below ever matched. Defaults at 14s were correct.
#   insert #2  tray_preload -> tray_readid xN -> STEP2:pull tray 0 from switch
#              -> rfid pull -> read all card -> search finished, found 0 card
#              -> feed and judge place -> STEP5:no card in RF -> card auth
#              success -> read success,valid. Tag applied 13.0s after the edge,
#              against a 14.0s fallback: a ONE SECOND margin.
#
# So tray_preload/tray_readid are deliberately NOT here. They precede the pull
# and an AMS can sit in them and give up (insert #1), which must still fall back
# on schedule. Everything below only appears once the read is really underway,
# and "no card in RF" is not terminal -- insert #2 emitted it 7s before the tag
# authenticated.
_RFID_INFLIGHT_RE = re.compile(
    r"STEP\d?:?\s*(?:pull tray|rfid pull|start,read all card|search finished|"
    r"feed and judge|no card in RF|card auth|read success|feed with rfid)"
    r"|\[RF\]\s*tray\d+:"
    r"|\[AMS_RFID\]STEP:")

# Bus chatter with no operational content. These are the AMS's own link-layer
# bookkeeping -- who is selected, what mode/ref it moved to -- repeated many
# times a second, forever, by every unit on the wire. They are worth keeping in
# AFC.log (a select storm is how a chain-addressing fault looks) but they carry
# nothing an operator can act on, and they were burying the lines that do:
#
#   AMS: [AMS_CALL] ams0 select,select ams1 [AMS_CALL] ams0 select,select ams1
#   AMS: [AMS_COMMON]mode: 4 -> 0 [AMS_COMMON]ref: 128 -> 128
#
# A line is suppressed from the console only if EVERY bracketed segment in it is
# noise. The AMS bundles several segments per line and mixes registers freely --
# "[AMS_CALL] ams0 select,select ams0 [AMS_DEV] STEP:set 0 tray_preload" is one
# line -- so a naive "contains noise -> drop" would have thrown away the tag-read
# narration riding alongside it.
_AMS_NOISE_RE = re.compile(
    r"^(?:\s*(?:"
    r"\[AMS_CALL\]\s*ams\d+\s+select,\s*select\s+ams\d+"
    r"|\[AMS_LINK\]\s*ams\d+\s+select,\s*req\s+ams\d+"
    r"|\[AMS_COMMON\]\s*(?:mode|ref):\s*-?\d+\s*->\s*-?\d+"
    r"|\[AMS_IDLE\]\s*set ams state switch"
    r"))+\s*$")

def _ams_is_noise(text: str) -> bool:
    """
    Whether a narration line is pure link-layer chatter (console-suppressed).

    :param text: One raw narration line from the AMS
    :return bool: True if nothing in the line is worth an operator's attention
    """
    # Leading junk: the drain reply often starts with one stray byte rendered as
    # a character ("\\ [AMS_CHMB]...", "q [AMS_DEV]..."), which is framing, not
    # content. Strip up to the first bracket before judging the line.
    i = text.find("[")
    return bool(text) and i >= 0 and bool(_AMS_NOISE_RE.match(text[i:]))

# Events the reader consumes or deliberately ignores. Anything outside this set
# is surfaced (to AFC.log) rather than dropped -- see handle_line. The command
# echoes are listed because the bridge answers every command with one and they
# are not interesting on their own.
#: Sentinel for "no value yet", where None is itself meaningful.
_UNSET = object()

_BRIDGE_EVENTS_KNOWN = frozenset((
    "status", "reply", "error", "ack", "amsdbg", "sniff", "chain", "info",
    "sniff_mode",
    # command echoes
    "dry", "mon", "resync", "mcaddr", "selfc", "armms", "hb",
    "htpoll", "htid", "htunit", "drain", "mute", "units", "variant", "baud",
    "parity", "en", "replay", "load", "unload", "rdinfo", "relink", "rehome",
))

# ── Bridge connection (threaded reader, reactor hop) ────────────────────────────

class BambuBridge:
    """Serial link to the Pico bridge: background reader + JSON command writer.

    One bridge per physical Pico. Multiple AFC units (daisy-chained AMS on the
    same bus) share it and each register a status listener via add_listener().
    """

    def __init__(self, serial_factory: Callable[[], Any], reactor: Any,
                 logger: Any) -> None:
        """
        :param serial_factory: Zero-arg callable returning an open pyserial-like
          port (injectable so tests can supply a fake)
        :param reactor: The Klipper reactor (for register_async_callback)
        :param logger: AFC logger
        """
        self._serial_factory = serial_factory
        self.reactor = reactor
        self.logger = logger
        # _narrate_human interpolates self.name, and nothing ever assigned it:
        # every line matching _AMS_HUMAN raised AttributeError into
        # handle_line's `except Exception: pass`, so the whole say-it-in-English
        # feature was dead in production while the tests passed, because their
        # shim set `name` and production had no equivalent. Deliberately NOT a
        # unit name -- one bridge can carry several units, and the narration is
        # bus-wide; the address in each message is what identifies the unit.
        self.name = "bridge"
        self._listeners: List[Callable[[dict], None]] = []
        self._serial: Optional[Any] = None
        self._thread: Optional[threading.Thread] = None
        self._run = False
        self._latest: Optional[dict] = None
        self._lock = threading.Lock()
        self._last_dbg: Optional[str] = None
        self._last_human: Optional[str] = None
        self._last_chmb_t: float = 0.0
        self._last_human_t: float = 0.0
        self._chmb_temp: Optional[float] = None
        self._chmb_t_seen: float = 0.0
        # The rest of the same [AMS_CHMB] line: the AMS's own state code and
        # the target it is driving to. Kept alongside the chamber temperature
        # so a UI can say what the heater is DOING (heating toward 55 vs
        # holding vs idle) instead of only how warm the box is. Both go stale
        # with _chmb_t_seen -- the AMS only streams this while it is drying.
        self._chmb_state: Optional[int] = None
        self._chmb_target: Optional[float] = None
        # Last drying refusal per device address, so the panel can say WHY a
        # start did nothing. Cleared when that unit actually begins a cycle.
        self._dry_err: Dict[int, str] = {}
        # Same records keyed by the device address that sent them, so several
        # AMS units on one bridge do not overwrite each other's chamber.
        self._chmb_by_addr: Dict[int, dict] = {}
        # Dedicated narration log; see set_narration_log().
        self._nar_lg: Optional[Any] = None
        # The AMS's OWN measurement of its PTFE path, in mm, keyed by the
        # address that narrated it. The unit self-calibrates this from
        # consecutive feeds and announces it, reporting 0 until it has enough
        # samples, so only positive values are stored. This is the distance
        # the filament travels on this machine, so it is preferred over any
        # configured value.
        self._tube_by_addr: Dict[int, float] = {}
        # Repeat tracking for the narration dedupe. An identical line is
        # re-emitted periodically with a count, so a repeating fault stays
        # visible as a fault rather than being suppressed into silence.
        self._last_dbg_n: int = 0
        self._last_dbg_t: float = 0.0
        # Monotonic time of the last narration line showing a tag read actually
        # in flight (see _RFID_INFLIGHT_RE). Bridge-wide rather than per-unit:
        # the narration text does not reliably name its unit, and the failure
        # mode of getting it wrong is only that a chain-mate's defaults land a
        # few seconds later -- never that wrong data is applied.
        # None (not 0.0) for "never seen": a reactor whose monotonic clock
        # reads 0.0 would otherwise have its stamp treated as absent.
        self._rfid_step_t: Optional[float] = None
        # Last motion completion the AMS itself reported ("feed finish",
        # "preload finish", "pull finish"), as (sequence, ok, text). The bridge
        # gives no ack for move COMPLETION -- only that the command was
        # accepted -- so without this the host can only guess a move's duration
        # from distance/speed, and the AMS does not move at the speed we ask
        # for. Sequence increments per event so a waiter can tell a fresh
        # completion from a stale one.
        self._finish_seq: int = 0
        self._finish_ok: bool = False
        self._finish_text: str = ""
        # The AMS's own fault reports. It names stalls explicitly -- "feed
        # finish -1, stall", "switch_feed rocker stall", "pull err, bdc stall"
        # -- which is far more reliable than inferring a fault from buffer
        # position, because the unit knows things we cannot see (rocker state,
        # which motor, which tray). Sequence increments per report so a consumer
        # can tell a fresh fault from one it has already handled.
        self._fault_seq: int = 0
        self._fault_text: str = ""
        self._bldc_i: float = 0.0
        self._chain_uids: List[str] = []       # index -> 24-hex UID (from `chain`)
        self._last_raw_reply: str = ""         # last `reply` frame (diagnostic)
        # unit -> the MC address the FIRMWARE read back after being told one.
        # Receipt for the announce. From Klipper an mcaddr command that never
        # arrives and one that arrives and is applied look identical, so the
        # ack is what distinguishes them -- the distinction that matters when
        # the narration drain falls back to the captured 0x0700 pair.
        self._mcaddr_ack: Dict[int, int] = {}
        # Last fstate seen, for the change-only trace. _UNSET (not None) so the
        # very first frame is recorded -- a unit that comes up in a mode and
        # never leaves it is a finding, and None is a legitimate value here.
        self._fstate_last: Any = _UNSET
        # Latch for the tray-gone edge. The unit repeats "odom tray_id error
        # 255" for as long as it is asked, so only the RISING edge is a
        # completion; re-armed when a tray is engaged again.
        self._tray_gone: bool = False
        # Last buffer position the unit reported, from e_in or feed finish.
        # None until it says one -- 0.0 is a legitimate reading.
        self._buff_pos: Optional[float] = None
        # Last buffer refill as (sagged_to, recovered_to, mm_fed). mm is None
        # when the line omitted det.
        self._buff_refill: Optional[Tuple[float, float, Optional[float]]] = None
        self._reconnect_cbs: List[Callable[[], None]] = []

    def add_listener(self, cb: Callable[[dict], None]) -> None:
        """
        Register a callback invoked (on the reactor) with each status frame.

        :param cb: Callable taking one decoded status dict
        """
        self._listeners.append(cb)

    def add_reconnect_listener(self, cb: Callable[[], None]) -> None:
        """
        Register a callback invoked (on the reactor) after the serial link is
        re-established. A reconnect usually means the Pico REBOOTED (reflash,
        power-cycle, replug), which resets the firmware's per-unit state (polled
        unit count, HT flags) and may reshuffle enrollment -- units re-push their
        config from here.

        :param cb: Zero-arg callable
        """
        self._reconnect_cbs.append(cb)

    def start(self) -> None:
        """Open the port and spin up the reader thread."""
        self._serial = self._serial_factory()
        self._run = True
        self._thread = threading.Thread(target=self._reader,
                                        name="afc-bambu-bridge", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal the reader to stop and close the port."""
        self._run = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass

    def last_fault(self) -> Tuple[int, str, float]:
        """
        Return the AMS's last self-reported stall.

        :return tuple: (sequence, text, last motor current in A)
        """
        with self._lock:
            return (self._fault_seq, self._fault_text, self._bldc_i)

    def set_narration_log(self, log_dir: str,
                          max_bytes: int = 10 * 1024 * 1024) -> bool:
        """
        Send the AMS's own narration to its own file.

        The narration gets its own file and handler: always written, never on
        the console, and independent of any AFC setting. That keeps every
        STEP, finish, stall and measured length on record on a printer running
        with AFC's `debug` flag off, which gates logger.debug() and is the
        normal state for a working printer since the AMS narrates
        continuously. Rotates at 10 MB keeping NO backups -- narration is a
        rolling window for diagnosis, not an archive, and an unbounded log on
        a Pi's SD card is its own hazard.

        :param log_dir: directory to write into (Klipper's log directory)
        :param max_bytes: rotate at this size; 0 disables rotation
        :return bool: True if the log is ready
        """
        if self._nar_lg is not None:
            return True
        lg = logging.getLogger("AFC_BambuAMS_file")
        # isinstance, NOT `if not lg.handlers`. logging.getLogger() is
        # process-global, so anything that attached a handler first -- pytest,
        # another unit, a reload -- makes the truthy check skip setup and hand
        # back a logger with no file, which reports success and writes
        # nowhere. That exact bug shipped once in the bus-monitor logger.
        if not any(isinstance(h, logging.handlers.RotatingFileHandler)
                   for h in lg.handlers):
            try:
                fh = _TruncatingRotatingFileHandler(
                    os.path.join(log_dir, "AFC_BambuAMS.log"),
                    maxBytes=max_bytes, backupCount=0)
                fh.setFormatter(logging.Formatter(
                    "%(asctime)s %(message)s", datefmt="%H:%M:%S"))
                lg.addHandler(fh)
            except Exception as e:
                self.logger.warning(
                    "AFC bambu: could not open AFC_BambuAMS.log: %s" % (e,))
                return False
        lg.setLevel(logging.DEBUG)
        # Never propagate: this would otherwise duplicate every narration line
        # into AFC.log, which is the flooding this file exists to avoid.
        lg.propagate = False
        self._nar_lg = lg
        return True

    def _narrate_to_file(self, text: str, addr: Optional[int]) -> None:
        """
        Record one narration line verbatim.

        Written BEFORE the console dedupe, deliberately: a line repeating
        hundreds of times is how a stuck loop looks, and collapsing it in the
        file would hide the shape of the fault. The address is included so a
        bus carrying several units stays attributable.

        :param text: the raw narration line
        :param addr: device address that produced it, if known
        """
        lg = self._nar_lg
        if lg is None or not text:
            return
        try:
            lg.debug("%s %s" % (
                ("0x%04X" % int(addr)) if addr else "0x----", text))
        except Exception:
            pass          # a log must never take the reader thread down

    def last_buff_pos(self) -> Optional[float]:
        """
        The buffer position the unit last reported, or None.

        Reported at two moments worth having: "e_in" while the filament
        enters the extruder, and "feed finish" when the load completes. At
        end-of-load it reads ~1.28 on an HT (five consecutive loads, spread
        0.01) -- hard compressed -- which is the reference point for buffer
        ramming.

        :return Optional[float]: the reading, or None if it has said none
        """
        with self._lock:
            return self._buff_pos

    def last_buff_refill(self):
        """
        The unit's last buffer refill: (sagged_to, recovered_to, mm_fed).

        This is the ramming event as the AMS measures it -- how far the buffer
        sagged when the extruder pulled, and how much filament it fed to bring
        it back. mm_fed is None if the line carried no `det`.

        :return Optional[Tuple[float, float, Optional[float]]]: the refill
        """
        with self._lock:
            return self._buff_refill

    def _note_dry_refusal(self, text: Optional[str],
                          addr: Optional[int]) -> None:
        """
        Record, or clear, a drying refusal for the device that narrated it.

        Recorded BEFORE the dedupe, like the file log: the AMS repeats the
        refusal on every retry, and a deduped repeat still means "still
        refusing". Cleared the moment that unit reports it is heating or
        self-checking, so a stale reason cannot outlive the condition.

        :param text: the narration line
        :param addr: device address that produced it
        """
        if not text or not addr:
            return
        try:
            key = int(addr)
            m = _DRY_REFUSED_RE.search(text)
            if m:
                with self._lock:
                    self._dry_err[key] = m.group(1).strip()
            elif "CTC_STATE_HEATING" in text or "CTC_STATE_SELF_CHECK" in text:
                with self._lock:
                    self._dry_err.pop(key, None)
        except Exception:
            pass          # a diagnostic must never take the reader thread down

    def last_dry_error(self, addr: Optional[int]) -> Optional[str]:
        """
        Why this device last refused to dry, or None if it has not.

        :param addr: device address (an HT is 0x1800, a boxed AMS 0x0700)
        :return Optional[str]: the AMS's own wording, e.g. "filament hub load!"
        """
        if not addr:
            return None
        with self._lock:
            return self._dry_err.get(int(addr))

    def _finish_succeeded(self, text: str, low: str,
                          addr: Optional[int]) -> bool:
        """
        Whether a motion completion means the filament ARRIVED.

        "finish -1, stall" is not a failure on every unit. An AMS HT ends a
        normal load by feeding to the end of its PTFE and stalling against the
        extruder gear -- that IS how it knows it is there, and it says so:

          feed finish -1, stall, len_det:3.601 m, tube_len:3.619 m
          feed finish, buff_pos:1.28, bldc_i:1.600A

        18 mm short of a 3619 mm path, immediately followed by a clean finish.
        Reading the word "stall" as failure marks a perfectly good load failed.
        A REAL stall looks nothing like it -- the unload that genuinely came up
        short reported len_det:3.283 m against the same 3619 mm, 336 mm out,
        and did need its retry.

        So the question is not whether it stalled, it is how far it got:

        1. no stall reported at all -> success, as before
        2. stalled, but len_det reached tube_len (within tolerance) -> success
        3. the same line also carries a clean finish -> success
        4. otherwise -> failure

        tube_len comes from the line when present, else from this unit's last
        reported measurement, so a stall line that omits it is still judged
        against the right distance rather than defaulting to failure.

        :param text: the narration line
        :param low: the same line, lowercased (already computed by the caller)
        :param addr: device address that narrated, if known
        :return bool: True if the move achieved what it was asked to
        """
        if "finish -1" not in low and "stall" not in low:
            return True
        # A clean completion sharing the line -- narration arrives as several
        # bracketed segments, so the stall and the success routinely do.
        if _CLEAN_FINISH_RE.search(text):
            return True
        det = _LEN_DET_M_RE.search(text)
        if not det:
            return False
        travelled = float(det.group(1)) * 1000.0
        tube = _TUBE_LEN_M_RE.search(text)
        target = (float(tube.group(1)) * 1000.0 if tube
                  else self.tube_len(addr))
        if not target:
            return False          # nothing to judge against; stall stands
        return travelled >= target - FINISH_ARRIVAL_TOLERANCE_MM

    def _trace_fstate(self, obj: dict) -> None:
        """
        Record every CHANGE of the AMS's own mode into the narration log.

        `fstate` is what the move-completion wait keys on, so whether it
        actually moves during a load is not something to reason about -- it is
        something to read off a trace next to the narration that shares its
        clock. Changes only: the field rides every status frame, several a
        second, and logging all of them would bury the narration it sits
        beside.

        :param obj: a decoded status event
        """
        if self._nar_lg is None:
            return
        try:
            v = obj.get("fstate")
            if v == self._fstate_last:
                return
            prev, self._fstate_last = self._fstate_last, v
            self._nar_lg.debug("HOST-- fstate %s -> %s (buff=%s)" % (
                "-" if prev is _UNSET else prev, v, obj.get("buff")))
        except Exception:
            pass          # a trace must never take the reader thread down

    def tube_len(self, addr: Optional[int] = None) -> Optional[float]:
        """
        The AMS's own measured PTFE path length in mm, if it has told us.

        The unit learns this itself from consecutive feed measurements and
        narrates it. It is the real distance on this machine, so it beats any
        configured estimate -- but it is only available once the unit has
        enough samples, and it reports 0 until then.

        :param addr: Device address to look up (0x0700 AMS, 0x1800 HT); None
            returns the most recent from any unit
        :return Optional[float]: length in mm, or None if never reported
        """
        with self._lock:
            if addr is not None:
                v = self._tube_by_addr.get(int(addr))
                if v:
                    return v
                return None
            # No address: only safe to answer when exactly one unit has
            # reported. With two units on the bridge, "the most recent" could
            # hand an HT's path length to an AMS 2.
            vals = list(self._tube_by_addr.values())
            return vals[0] if len(vals) == 1 else None

    def last_finish(self) -> Tuple[int, bool, str]:
        """
        Return the AMS's last reported motion completion.

        :return tuple: (sequence, ok, text); sequence increments per event
        """
        with self._lock:
            return (self._finish_seq, self._finish_ok, self._finish_text)

    def latest_status(self) -> Optional[dict]:
        """
        Return the most recent status frame (thread-safe copy).

        :return Optional[dict]: the last status dict, or None if none yet
        """
        with self._lock:
            return dict(self._latest) if self._latest is not None else None

    def send(self, obj: dict) -> None:
        """
        Write one JSON command line to the bridge.

        :param obj: The command object to serialize and send
        """
        s = self._serial
        if s is None:
            return
        try:
            s.write((json.dumps(obj) + "\n").encode())
        except Exception as e:
            # A write failure means the link is broken (e.g. Errno 5 after the
            # Pico glitched or another process grabbed the port). Drop the port so
            # the reader thread reconnects, instead of failing every write forever.
            self.logger.warning(
                f"AFC bambu: bridge write failed: {e}; reconnecting")
            self._drop_port()

    def _narrate_human(self, text: str, now: float,
                       addr: Optional[int] = None) -> None:
        """
        Surface the AMS's own words on the console, in English.

        Everything the unit says already goes to AFC.log verbatim; this picks
        out the handful an operator would actually want to see and says them
        plainly. A refused dry command is the motivating case: the AMS answers
        "[AMS_CHMB]ignore dry_mode:1, ams_state:2", which is the difference
        between "the heater is broken" and "it was busy, try again" -- and
        BAMBU_HEATER_START reports success either way, so without this the
        refusal is invisible.

        :param text: One narration line from the AMS
        :param now: Reactor monotonic time, for rate limiting
        :param addr: Device address the line came from (0x0700 = AMS 2 Pro,
            0x1800 = HT), so chamber telemetry can be attributed to the unit
            that produced it instead of shared across the bridge.
        """
        for pattern, render in _AMS_HUMAN:
            m = pattern.search(text)
            if not m:
                continue
            msg = f"AFC bambu {self.name}: {render(m)}"
            # Consecutive duplicates say nothing; the AMS repeats state lines.
            if msg == getattr(self, "_last_human", None):
                continue
            # Hard floor between console lines. The AMS narrates continuously
            # and a burst (a load, a dry start) can match several patterns in
            # under a second; the console is the operator's, not a log tail.
            if now - getattr(self, "_last_human_t", 0.0) < 1.0:
                continue
            self._last_human = msg
            self._last_human_t = now
            self.logger.info(msg)
        # The unit's own PTFE measurement. Stored against the address that said
        # it, for the same reason the chamber record is: a bridge can carry an
        # AMS 2 and an HT with very different path lengths, and mixing them up
        # would hand one unit the other's distance.
        # No try/except around the float(): both patterns match digits only
        # (\d+ and [0-9]+\.[0-9]+), so the conversion cannot raise. A guard
        # here would be unreachable, and unreachable defences are what make a
        # module look covered when it is not.
        mm = None
        tl = _TUBE_LEN_MM_RE.search(text)
        if tl:
            mm = float(tl.group(1))
        else:
            tl = _TUBE_LEN_M_RE.search(text)
            if tl:
                mm = float(tl.group(1)) * 1000.0
        # > 0 only: the unit reports 0.000 m until it has calibrated, and
        # adopting that would set every deadline to zero.
        if mm and mm > 0.0 and addr:
            with self._lock:
                prev = self._tube_by_addr.get(int(addr))
                self._tube_by_addr[int(addr)] = mm
            if prev is None:
                self.logger.info(
                    f"AFC bambu {self.name}: AMS 0x{int(addr):04X} reports its "
                    f"measured filament path as {mm:.0f}mm -- using it to size "
                    f"move timeouts instead of the configured estimate")

        # Drying telemetry arrives every ~10s and would be console spam, so it
        # is reported on its own slow cadence rather than per line.
        m = _CHMB_STATE_RE.search(text)
        if m:
            # Chamber temperature. NOT in the binary protocol -- temp_c10 is
            # hardcoded -1 there -- but the AMS streams it here every ~10s
            # while drying. vt is the chamber probe; ap runs ~4C higher and
            # tracks it, so it reads like a second probe nearer the heater.
            # Only vt is published, and only while this telemetry is arriving.
            # Keyed by the address the frame came from, so a bridge carrying
            # both an AMS 2 Pro (0x0700) and an HT (0x1800) keeps their
            # chambers apart. The flat attributes stay as the last-seen value
            # for callers that predate this.
            try:
                rec = {"temp": float(m.group(3)), "seen": now,
                       "state": int(m.group(1)), "target": float(m.group(2))}
            except (TypeError, ValueError):
                rec = None
            if rec is not None:
                self._chmb_temp = rec["temp"]
                self._chmb_t_seen = rec["seen"]
                self._chmb_state = rec["state"]
                self._chmb_target = rec["target"]
                if addr:
                    self._chmb_by_addr[int(addr)] = rec
        if m and now - getattr(self, "_last_chmb_t", 0.0) >= 60.0:
            self._last_chmb_t = now
            # Humidity rides along only on models that attach it to vt.
            hum = f", humidity {m.group(4)}%" if m.group(4) else ""
            self.logger.info(
                f"AFC bambu {self.name}: drying -- chamber {m.group(3)}C"
                f"{hum}, target {m.group(2)}C")

    def rfid_read_in_flight(self, now: float, quiet: float = 3.0) -> bool:
        """
        Whether an AMS on this bridge is mid tag-read right now.

        True while narration matching ``_RFID_INFLIGHT_RE`` has been seen within
        ``quiet`` seconds. The AMS emits those steps every few hundred ms once a
        read is underway, so a gap of several seconds means it has finished or
        given up.

        :param now: Reactor monotonic time
        :param quiet: Seconds of narration silence that end a read
        :return bool: True if a read appears to still be running
        """
        t = self._rfid_step_t
        return t is not None and (now - t) < quiet

    def handle_line(self, line: str) -> None:
        """
        Decode one bridge line and, for a status frame, cache + hop to reactor.

        Split out from the reader loop so it's unit-testable.

        :param line: One line of text from the bridge
        """
        obj = parse_bridge_line(line)
        if obj is None:
            return
        if obj.get("evt") == "status":
            with self._lock:
                self._latest = obj
            self._trace_fstate(obj)
            for cb in self._listeners:
                self.reactor.register_async_callback(
                    lambda et, o=obj, c=cb: c(o))
        elif obj.get("evt") == "reply":
            # Raw AMS frame, requested by BAMBU_BUFFER_PROBE. Diagnostic only --
            # held here so the probe can print it without a shell on the printer.
            with self._lock:
                self._last_raw_reply = str(obj.get("hex") or "")
        elif obj.get("evt") == "error":
            self.logger.warning(f"AFC bambu: bridge error: {obj.get('msg')}")
        elif obj.get("evt") == "ack":
            # Motion-command acknowledgements (select/feed/retract/assist/
            # stop/...). AFC.log via the AFC logger, not python logging.debug,
            # which klipper runs at INFO and therefore discards -- these are the
            # record of what the bridge was actually asked to do.
            self.logger.debug(
                f"AFC bambu: bridge ack {obj.get('cmd')} (slot {obj.get('slot')})")
        elif obj.get("evt") == "amsdbg":
            # The AMS's own narration. Identical consecutive lines are
            # de-duplicated, but NOT suppressed forever: a repeating line is how
            # a stuck loop presents itself, and swallowing it makes the fault
            # indistinguishable from silence. The 10s "[DBG] ams time" heartbeat
            # is dropped outright (its timestamp defeats the dedupe entirely and
            # it carries nothing).
            text = obj.get("text")
            # Defensive: this runs on the reader thread, and a diagnostic must
            # never be able to take the bridge down.
            mono = getattr(self.reactor, "monotonic", None)
            now = mono() if callable(mono) else 0.0
            # Stamp read-in-flight from the RAW line, BEFORE the dedupe below
            # blanks a repeat. The AMS repeats these steps while it works the
            # reader, and a deduped repeat is still evidence the read is alive.
            # Verbatim to the dedicated file first: unconditional, unfiltered
            # and before any dedupe or noise suppression.
            self._narrate_to_file(text, obj.get("addr"))
            self._note_dry_refusal(text, obj.get("addr"))
            if text and _RFID_INFLIGHT_RE.search(text):
                self._rfid_step_t = now
            if text and text == self._last_dbg:
                # Same line again: count it, and re-emit once a minute so a
                # loop that is going nowhere still shows up.
                self._last_dbg_n += 1
                if now - self._last_dbg_t >= 60.0:
                    self._last_dbg_t = now
                    n = self._last_dbg_n
                    self.logger.debug(f"AMS: (x{n} repeated) {text}",
                                      only_debug=True)
                text = None
            elif text:
                self._last_dbg = text
                self._last_dbg_n = 1
                self._last_dbg_t = now
            if text:
                # Buffer position, from whichever line carries it. Recorded
                # before the completion branches below so a line that is both
                # (a feed finish carrying buff_pos) contributes both.
                bp = _BUFF_POS_RE.search(text)
                if bp:
                    with self._lock:
                        self._buff_pos = float(bp.group(1))
                rf = _BUFF_REFILL_RE.search(text)
                if rf:
                    det = float(rf.group(3)) if rf.group(3) else None
                    with self._lock:
                        # The position AFTER recovery is the current one.
                        self._buff_pos = float(rf.group(2))
                        self._buff_refill = (float(rf.group(1)),
                                             float(rf.group(2)), det)
                # Motion completion.
                low = text.lower()
                if _MOTION_FINISH_RE.search(text):
                    # Judged BEFORE the lock: it reads tube_len(), which takes
                    # the same non-reentrant lock, and doing this inside the
                    # with-block deadlocks the reader thread outright.
                    ok = self._finish_succeeded(text, low, obj.get("addr"))
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = ok
                        self._finish_text = text
                    self._tray_gone = False
                elif _EXTRUDER_IN_RE.search(text):
                    # Arrival at the extruder. Same standing as an odom reset:
                    # it says the filament got there, which is what a load is
                    # waiting to hear.
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = text
                    self._tray_gone = False
                elif _ODOM_RESET_RE.search(text):
                    # A tray was engaged and its odometer zeroed: the feed
                    # arrived. This is the only completion an [AMS_DEV]-dialect
                    # unit gives -- it never says "feed finish" -- and it also
                    # re-arms the tray-gone edge below.
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = text
                    self._tray_gone = False
                elif _ODOM_NO_TRAY_RE.search(text) and not self._tray_gone:
                    # The odometer has no tray, so the filament has left the
                    # unit: a retract completed.
                    #
                    # EDGE-triggered, and that is not a nicety. The unit
                    # repeats this at ~2 Hz for as long as it is asked, so
                    # counting every one would leave a completion permanently
                    # pending and the NEXT move would return the instant it
                    # started waiting. Re-armed only by an odom reset, i.e. by
                    # a tray actually being engaged again.
                    self._tray_gone = True
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = text
                # Motor current, when the AMS reports it. It clamps at its
                # limiter (~1.6A observed) against a ~0.07A median, so this is a
                # threshold signal rather than a proportional one.
                # Say the useful ones out loud, in English.
                try:
                    # addr identifies WHICH AMS narrated (0x0700 AMS 2 Pro,
                    # 0x1800 HT); absent on firmware older than 1.0.7.0.
                    self._narrate_human(text, now, obj.get("addr"))
                except Exception:
                    pass          # a nicety must never break the reader thread
                mi = _BLDC_I_RE.search(text)
                if mi:
                    try:
                        with self._lock:
                            self._bldc_i = float(mi.group(1))
                    except ValueError:
                        pass
                # Explicit stall. Deliberately NOT triggered by assist_err or
                # err_code: both cycle constantly during normal operation
                # (assist_err 0->65536->0 around every successful feed), and
                # treating them as faults would pause a healthy print.
                # "TIMEOUT error N" is how a boxed AMS 2 reports a jam; an HT
                # says "stall" outright. These strings are fault-specific --
                # they do not appear during boots, loads, unloads, scans,
                # drying or sustained follower runs -- and the unit emits them
                # as soon as its own motor stalls, ahead of the buffer
                # finishing its drain.
                if ("stall" in low or "finish -1" in low
                        or "timeout error" in low):
                    with self._lock:
                        self._fault_seq += 1
                        self._fault_text = text.strip()
                if "[DBG] ams time" in text:
                    pass
                else:
                    # The AMS's own narration -- its feed cycle
                    # ("[AMS_SWITCH]BUFF,pos:0.09->0.74,det:12mm"), motor
                    # transitions and scan steps. All of it goes to AFC.log via
                    # the AFC logger, never the console.
                    #
                    # This is the only place the AMS reports whether it is
                    # actually feeding, so it is written unconditionally
                    # rather than through python logging.debug, which klipper
                    # runs at INFO and would discard outright.
                    #
                    # Pure bus chatter is additionally kept OFF the console
                    # (only_debug=True) -- see _AMS_NOISE_RE. Narration that
                    # says something stays on the console, where an operator
                    # with AFC's debug flag on watches a load happen.
                    self.logger.debug(f"AMS: {text}",
                                      only_debug=_ams_is_noise(text))
        elif obj.get("evt") == "mcaddr":
            # Receipt for the announce. The firmware does not echo what we
            # asked for -- it echoes what bb_get_mc_addr() reads back AFTER
            # applying it, so a zero here means the address did not take and
            # the log drain will use the 0x0700 fallback that never asks an
            # AMS HT at 0x1800.
            try:
                with self._lock:
                    self._mcaddr_ack[int(obj.get("unit", 0))] = \
                        int(obj.get("addr", 0))
            except Exception:
                pass
        elif obj.get("evt") not in _BRIDGE_EVENTS_KNOWN:
            # Anything the bridge says that nothing here consumes. Kept because
            # silent unknown events make "the command never landed" and "the
            # reply never came" look identical -- which cost hours once. File
            # only: the routine command echoes (mcaddr, selfc, armms...) land
            # here on every prep and have no business on an operator's console.
            self.logger.debug(f"AFC bambu: unhandled bridge event {obj}",
                              only_debug=True)
        elif obj.get("evt") == "sniff":
            # Passive bus-sniffer frame (real printer <-> AMS). Log every raw frame
            # verbatim so a capture can be pulled from AFC.log -- no dedup, each
            # frame matters. Only present when the firmware is in sniff mode.
            # File-only: a sniff runs at hundreds of frames a second and would
            # make the console unusable.
            self.logger.debug(f"SNIFF {obj.get('hex')}", only_debug=True)
        elif obj.get("evt") == "chain":
            # Enrollment map: uids is a comma-separated list of 12-byte (24-hex)
            # UIDs, POSITION = the unit's polling address (ams_index). Keep empty
            # fields -- dropping them would shift every later unit's index.
            raw = (obj.get("uids") or "").strip().upper()
            uids = [] if not raw else [u.strip() for u in raw.split(",")]
            with self._lock:
                self._chain_uids = uids
                # Diagnostics riding on the chain reply (older firmware omits
                # them): which indices the firmware has HT-flagged, and the
                # EXACT build running on the Pico -- the only way to verify a
                # flash took.
                try:
                    self._chain_htmask = int(obj.get("htmask") or 0)
                except Exception:
                    self._chain_htmask = 0
                self._chain_fw = str(obj.get("fw") or "")
                # Per-unit MC addressing as the FIRMWARE holds it. Absent on
                # firmware older than 1.0.10.9, which is why None (unknown) is
                # kept distinct from [] (known-empty).
                mc = obj.get("mcaddr")
                self._chain_mcaddr = (
                    [int(x) for x in mc] if isinstance(mc, list) else None)
                try:
                    self._chain_sel = (int(obj.get("selid", -1)),
                                       int(obj.get("selsent", 0)),
                                       int(obj.get("selack", 0)))
                except Exception:
                    self._chain_sel = (-1, 0, 0)

    def chain_uids(self) -> List[str]:
        """Return the cached chain UID list (index -> UID); empty until known."""
        with self._lock:
            return list(self._chain_uids)

    def mcaddr_ack(self, unit: int) -> Optional[int]:
        """
        What the firmware read back the last time this unit was told an MC
        address, or None if it has never acknowledged one.

        None and 0 mean different things and both are failures worth telling
        apart: None is "the command never reached the Pico" (announce dropped,
        link not up yet, JSON malformed); 0 is "the Pico got it and the address
        still is not set".

        :param unit: AMS chain index
        :return Optional[int]: the acknowledged address, or None
        """
        with self._lock:
            return self._mcaddr_ack.get(int(unit))

    def chain_mcaddr(self) -> Optional[List[int]]:
        """
        Per-unit MC device addresses as the firmware holds them.

        None means the firmware did not report them (pre-1.0.10.9), which is
        deliberately distinct from an empty/zero list meaning "reported, and
        nothing is set". That distinction is the whole point: an unset address
        drops the narration log drain back to the captured 0x0700 pair, which
        never asks an AMS HT at 0x1800 -- a failure that was invisible from
        Klipper until this was surfaced.

        :return Optional[List[int]]: addresses by unit index, or None
        """
        with self._lock:
            return getattr(self, "_chain_mcaddr", None)

    def chain_diag(self) -> tuple:
        """Return (htmask, fw, (selid, selsent, selack)) from the last chain
        reply ((0, '', (-1, 0, 0)) until known)."""
        with self._lock:
            return (getattr(self, "_chain_htmask", 0),
                    getattr(self, "_chain_fw", ""),
                    getattr(self, "_chain_sel", (-1, 0, 0)))

    def _drop_port(self) -> None:
        """Close the current serial and mark it gone so the reader reconnects."""
        s = self._serial
        self._serial = None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    def _reader(self) -> None:
        buf = b""
        backoff = 0.5
        while self._run:
            # Reconnect if the port is gone (first-open failure, a read/write
            # error, or a Pico re-plug). Back off so we don't spin while it's
            # absent, and reset the backoff once we're reading again.
            if self._serial is None:
                try:
                    self._serial = self._serial_factory()
                    self.logger.info("AFC bambu: bridge reconnected")
                    buf = b""
                    backoff = 0.5
                    # The firmware likely just booted (reflash/power-cycle): its
                    # unit count and HT flags are factory-fresh. Let each unit
                    # re-push its config (on the reactor, not this thread).
                    for cb in self._reconnect_cbs:
                        try:
                            self.reactor.register_async_callback(
                                lambda et, c=cb: c())
                        except Exception:
                            pass
                except Exception:
                    time.sleep(min(backoff, 5.0))
                    backoff = min(backoff * 2, 5.0)
                continue
            try:
                chunk = self._serial.read(64)
            except Exception as e:
                # Do NOT die on a read error -- drop the port and reconnect, so a
                # transient USB/serial glitch self-heals instead of bricking the
                # bridge until a Klipper restart.
                self.logger.warning(
                    f"AFC bambu: bridge read failed: {e}; reconnecting")
                self._drop_port()
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.handle_line(raw.decode(errors="replace"))
