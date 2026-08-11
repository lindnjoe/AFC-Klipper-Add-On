# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 J0eB0l. All Rights Reserved.
#
# LIMITED USE LICENSE
#
# This file is NOT distributed under the GNU GPL.
#
# Permission is granted, free of charge, to download, install and execute this
# file solely as an add-on component of the official, unmodified AFC
# (Automated Filament Control) distribution hosted at:
#
#     https://github.com/AFCProject/AFC-Klipper-Add-On
#
# The AFC Project is granted permission to host and distribute this file as
# part of that repository, unmodified and with this notice intact.
#
# RESTRICTIONS
#   1. You may NOT modify, reverse-engineer, decompile or create derivative
#      works of this file.
#   2. You may NOT bundle, redistribute, re-host or include this file in any
#      third-party software, installer or package manager without express
#      written consent of the copyright holder.
#   3. You may NOT use this file with modified forks or unauthorised
#      distributions of the AFC ecosystem.
#
# THIS FILE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED. THE COPYRIGHT HOLDER IS NOT LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY ARISING FROM ITS USE.
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
#: ...and the AMS HT's OWN completion words, which say neither "finish" nor
#: "sucess". Measured on the rig during a healthy load:
#:
#:   [AMS_SWITCH]feed to dw ok, len_det:0.126 m, bldc_i:0.3A
#:   [AMS_SWITCH]feed to normal, len_det:0.253 m, bldc_i:0.267A
#:
#: Neither matched anything here, so EVERY HT move ran its full watchdog
#: instead of ending on the unit's own report -- the module kept driving a
#: move the unit had already finished, which is what "it keeps sending the
#: wrong signal" looks like from the outside. The module's own _MC table
#: comment had already named "feed to normal" as "the move-completion
#: narration whose absence has been making every move wait out its deadline"
#: -- it was documented and never added to the pattern. Both HT forms are
#: accepted; "to dw ok" is the mid-path (drive wheel) stage and "to normal"
#: the end of the move.
_MOTION_FINISH_RE = re.compile(
    r"\b(?:(?:feed|pull|preload)\s+finish|pull\s+suc+ess"
    r"|feed\s+to\s+(?:normal|dw\s+ok))\b",
    re.IGNORECASE)

#: The AMS HT's capacity CALIBRATION verdict. The HT does not narrate the
#: boxed units' "odom C:..,R:..,P:NN%" line at all -- it runs the odometer
#: (start_odo -> first detected) and reports a result code:
#:
#:   [AMS_RFID] STEP4,Calibration rst:0   <- completed (clean run, measured)
#:   [AMS_RFID] STEP4,Calibration rst:1   <- refused ("odom tray capacity no en")
#:   [AMS_RFID] STEP4,Calibration rst:4   <- aborted ("check stall during calib")
#:
#: Watching only for "P:NN%" meant a SUCCESSFUL HT calibration read as a
#: silent failure, which is why the HT "never measured" while its own log
#: said it had. The percent itself is written to the tag, so the value is
#: picked up by the next filament-info read of that slot, not from this line.
_HT_CALI_RST_RE = re.compile(r"Calibration\s+rst:(\d+)", re.IGNORECASE)

#: The filament reached the EXTRUDER, as an AMS 2 Pro / HT reports it:
#:
#:   [AMS_SWITCH]e_in tray:0,buff_pos:-0.34,i:0.566A,len:1.670m
#:
#: "e_in tray:N,buff_pos:...,len:N.Nm" has NO pattern here, deliberately.
#:
#: Reading it as "extruder in" and completing a load on it is probably wrong:
#: in both captures containing it an err_code transition follows within
#: a second (0 -> 37, 0x00 -> 0x25), neither capture is of a healthy load, and
#: a live AMS 2 Pro has never emitted it across a full day of cycles -- which
#: fits an error that has not happened rather than an arrival that should occur
#: every load. "Error in tray" is the better reading.
#:
#: Nothing is lost by not matching it: the line still reaches AFC_BambuAMS.log
#: like all narration, and its buff_pos is still read by _BUFF_POS_RE below. If
#: it turns out to be a fault it belongs with "finish -1" / "stall" /
#: "timeout error", not with the completions.

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
#: The unit's CURRENT error level, both forms it is written in:
#:   [AMS_LINK]err_code: 0 -> 23        decimal, spaced (HT)
#:   [AMS_LINK]err_code:0x00->0x80      hex, unspaced (AMS 2)
#: Captured on both unit types. The value after the arrow is the new level;
#: 0 means "no error". Deliberately a LEVEL, not an edge -- see handle_line.
_ERR_CODE_RE = re.compile(
    r"err_code:\s*(0x[0-9A-Fa-f]+|\d+)\s*->\s*(0x[0-9A-Fa-f]+|\d+)")

_ODOM_NO_TRAY_RE = re.compile(r"odom\s+tray_id\s+error\s*255", re.IGNORECASE)

#: NOT a completion marker, though it looked like one.
#:
#:   [AMS_COMMON]state:2,tray_now:255,tray_exit:1
#:
#: On one AMS 2 unload this tracked perfectly -- retract at 13:42:27,
#: tray_now:255 from 13:42:43, 19 s before AFC gave up on its watchdog -- so it
#: was read as the AMS 2's wording for "the tray has left" and wired up like
#: the odometer form.
#:
#: It is not. The same line appears while the unit is LOADED and FOLLOWING:
#:
#:   [AMS_COMMON]state:4,tray_now:255,tray_exit:1
#:   [AMS_SWITCH]tray:0, bldc slip, dw_pos:-0.000 m
#:
#: so 255 here does not mean the filament is out of the unit. Used as a
#: completion it would end a move early on a unit that is merely idle between
#: trays, which is the failure the whole completion path exists to avoid.
#: Left unmatched until its meaning is actually known.

#: Distance the AMS says it actually moved, in metres.
_LEN_DET_M_RE = re.compile(r"len_det:([0-9]+\.[0-9]+)\s*m\b")

#: The AMS's own pull-and-push at the mode change into mode:4, e.g.
#:   [AMS_SWITCH]pull sucess,mode change,mode:4,tray_sw[2]:3;
#:               len:3.441m,0.052m,3.497m;bdc_i:0.555A;spd:-15.6cm/s;t:0.9s
#: This is the NATIVE seating tug -- it is in every working load. What matters
#: to the host is only WHEN IT IS OVER, because the extruder must not advance
#: into it. "cond match" pulls (mode:1) are the unload's multi-metre unwind and
#: are deliberately NOT matched here.
_PULL_DONE_RE = re.compile(r"pull sucess,\s*mode change,\s*mode:4")

#: The unit's seating cycle is PULL BACK **and then PUSH BACK FORWARD**, and
#: "pull sucess" is only the first half. This ends the second:
#:   [AMS_SWITCH]assist finish 0, ref:0
#: Waiting on the pull alone released the extruder while the unit was still
#: pushing -- which is the same fight, just moved later.
_ASSIST_DONE_RE = re.compile(r"assist finish\s*-?\d*")

_TUBE_LEN_MM_RE = re.compile(r"tube_len:(\d+)\s*mm")
_TUBE_LEN_M_RE = re.compile(r"tube_len:([0-9]+\.[0-9]+)\s*m\b")
# The length a unit reports at the END OF A FEED, which is a DIFFERENT
# quantity from tube_len and is recorded here so the difference can be
# measured rather than assumed:
#
#   [AMS_SWITCH]feed finish 0, mode:0, dw_len:3.661 m, ...        (HT)
#   [AMS_SWITCH]feed finish 0, dw_len:3.508 m                     (AMS 2 Pro)
#
# WHY THIS EXISTS. tube_len is a self-CALIBRATION, narrated with the samples
# behind it ("new tube_len:3481 mm, list:3491,3472,0 mm, err:19 mm"). Every
# unit on this rig reports tube_len=None -- it has never once been narrated
# here -- so measured_path_mm() is None everywhere and _adopt_measured_path
# takes its "this dialect cannot measure itself" branch on all three. That
# branch was believed; it was never true. The units DO say how far the
# filament went, in a word nothing was listening for.
#
# NOT WIRED TO ANYTHING. dw_len read 0.000 on a load that failed and 3.661 on
# the one that worked, which is the signature of a per-LOAD measurement rather
# than a stored calibration -- so it must not size a deadline or rewrite a
# config until that is settled. Reported only, so the next few loads gather
# the evidence.
_DW_LEN_M_RE = re.compile(r"dw_len:([0-9]+\.[0-9]+)\s*m\b")


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

#: The unit's OWN echo of the drying settings it is holding, emitted when a dry
#: is commanded:
#:
#:   [AMS_CHMB]rotate:0,0, pw_lim:100, cool_down:0,45, dur:480, tmpr:45
#:
#: rotate flags, power limit, cool-down, duration in MINUTES, target in C. This
#: is better than remembering what we sent: it survives a Klipper restart, and
#: it is the only source for a cycle this host did not start. It was missed for
#: a long time because the mid-dry telemetry line shares the [AMS_CHMB] tag and
#: carries entirely different fields -- s/rf/cd/vt/ap/hts/pw -- so a capture
#: taken during a dry never contains this one.
# The spool capacity measurement, narrated by the AMS at the end of its
# insert calibration: "STEP:odom C:0.491,R:0.078,P:84%, od:1.009" --
# circumference, spool radius (metres), REMAINING PERCENT, odometer length
# used. Reproducible on live hardware (84/85/86% across three runs of the
# same part-used spool). The AMS computes and narrates this but does not
# persist it to the tag record on our bus (the "odom save" gate is still
# unbroken), so the bridge captures the narrated number instead -- the
# measurement itself is what the feature needs.
#: FOUR real forms, captured with each unit ALONE on the wire so attribution
#: is exact. Two shapes (a live measurement, and a restore from flash at
#: power-on) across three units that punctuate differently:
#:
#:   HT     [AMS_RFID] STEP4,odom C:0.531,R:0.084,P:107%,od:1.132
#:   AMS 1  [AMS_DEV]  STEP:odom C:0.480, R:0.076, P:78%, od:0.988
#:   AMS 2  [AMS_RFID]STEP:odom load from flash 2,R:0.072,P:65
#:   HT     [AMS_RFID] STEP:odom load from flash 0,R:0.088,P:119
#:
#: The restore form has no C: and no % sign, and it is the one seen MOST of
#: the time -- a measured spool stays measured across reboots. The previous
#: pattern required both, so it matched only the live HT form and silently
#: ignored every restore and the whole AMS 2 dialect. That is the "measured
#: weight never updates" bug: a missing regex, not a missing capability.
#:
#: Whitespace is optional everywhere because the units disagree about it
#: ("P:107%,od:" vs "P:78%, od:"). Do NOT anchor on any one unit's
#: punctuation -- that is what cost us the AMS 2 for an hour.
#: DO NOT WIDEN THIS TO MATCH THE LOAD-TIME "r:N" LINES. They look like a
#: measurement and are not one -- they are the radius search still running:
#:
#:     [AMS_DEV] STEP:odom r:0, dt0.442, R:0.073, P:70%, od:0.741
#:     [AMS_DEV] STEP:odom r:1, dt0.887, R:0.071, P:65%
#:
#: Measured against a dedicated calibration of THE SAME SPOOL minutes earlier,
#: which said 73%. Two loads, and the estimates do not converge on it or on
#: each other -- one ran 26% -> 54% (up 28 points), the other 70% -> 65% (down
#: 5). "Take the last one of the load" was tried, and it lands 8 points under
#: the calibrated figure.
#:
#: The real measurement is the one with a CIRCUMFERENCE in it ("C:0.469,
#: R:0.075, P:73%"), sampled over ~2 spool revolutions by the calibration
#: cycle. This pattern requires R: to follow `odom` closely, which is what
#: excludes the search lines -- that is load-bearing, not incidental.
_CAP_MEASURE_RE = re.compile(
    r"odom\s+"
    r"(?:load\s+from\s+flash\s+(\d+)\s*,\s*"     # 1 tray (restore form)
    r"|C:([0-9.]+)\s*,\s*)?"                       # 2 circumference (live)
    r"R:([0-9.]+)\s*,\s*"                          # 3 radius, metres
    r"P:(\d+)\s*%?",                               # 4 remaining percent
    re.IGNORECASE)

#: The calibration verdict, also two forms. The HT MISSPELLS it:
#:   HT     [AMS_RFID] STEP4,odom calib sucess      (one s)
#:   AMS 1  [AMS_DEV]  STEP:odom calib success exit 0,dis:0.989
#: succ?ess covers both. exit 0 is the AMS 1's status code.
_CALI_DONE_RE = re.compile(
    r"odom\s+calib\s+succ?ess(?:\s+exit\s+(\d+))?", re.IGNORECASE)

_DRY_CFG_RE = re.compile(
    r"\[AMS_CHMB\]\s*rotate:(\d+),\s*(\d+),\s*pw_lim:(\d+),"
    r"\s*cool_down:\d+,\s*(\d+),\s*dur:(\d+),\s*tmpr:(\d+)")

#: THE STEP MARKER'S PUNCTUATION, IN ALL THREE DIALECTS -- one source of truth.
#:
#:   AMS 1   [AMS_DEV] STEP:read success,valid          STEP + colon
#:   AMS 2   [AMS_RFID]STEP:read success,valid          STEP + colon
#:   HT      [AMS_RFID] STEP3,read success ,goto Cali   STEP + digit + COMMA
#:
#: Pulled out of _STEP() below so the patterns that cannot use that helper --
#: the ones matching a STEP marker mid-line, away from its bracket tag -- share
#: the same definition instead of restating it and getting it wrong. Restating
#: it is not a hypothetical failure: _RFID_READ_OK_RE hard-coded "STEP:" and was
#: therefore HT-blind for months while reading as a working matcher.
_STEP_SEP = r"STEP\d*\s*[,:]\s*"


def _STEP(tail: str) -> "re.Pattern":
    """
    Build a pattern for a STEP event that ignores which dialect said it.

    The three units tag and punctuate the same event differently:

        HT     [AMS_RFID] STEP4,odom calib sucess
        AMS 2  [AMS_RFID]STEP:read success
        AMS 1  [AMS_DEV]  STEP:odom calib success exit 0

    The pattern therefore accepts any [AMS_*] tag, an optional space, STEP with
    an optional step number, ',' or ':' as the separator, then the shared
    wording. Anchoring on one dialect matches nothing on the other two.

    :param tail: the event wording, as a regex fragment
    :return: a compiled, dialect-tolerant pattern
    """
    return re.compile(r"\[AMS_[A-Z_]+\]\s*" + _STEP_SEP + tail, re.IGNORECASE)


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
    # PREFIX-AGNOSTIC from here down. Every rule above that names a bracket tag
    # is blind to at least one unit, because the three do not share a
    # vocabulary of tags at all -- counted over tonight's single-unit captures:
    #
    #   HT     [AMS_SWITCH] [AMS_COMMON] [AMS_LINK] [AMS_LED] [AMS_TRAY] [AMS_CHMB]
    #   AMS 2  the same, plus [AMS_RFID] [AMS_PMSM]
    #   AMS 1  [AMS_DEV] almost exclusively (63 of 64 fragments), + [AMS_CALL]
    #
    # So an [AMS_RFID]-anchored rule never fires on an AMS 1 and an
    # [AMS_DEV]-anchored one never fires on an HT -- which is why the same
    # event was written twice below, once per dialect, and still missed the
    # third unit. Match the CONTENT instead: the wording is shared, only the
    # tag and the punctuation around STEP differ ("STEP4," / "STEP:" / "STEP2:",
    # with or without a space after the bracket).
    # "read success" and "feed with rfid success" are NOT here, deliberately.
    # Captured on an HT: a failed attempt emits BOTH, then "info_valid 0",
    # then the unit retries and only the auth/flash pair marks the read that
    # actually landed. Announcing "AMS read the spool tag" on those told the
    # operator a read had succeeded seconds before one had -- the same
    # mistake the firmware's read latch used to make, in words. The two
    # lines below are true on every unit.
    (_STEP("card auth success"),
     lambda m: "AMS: tag authenticated"),
    (_STEP("auth card successful"),
     lambda m: "AMS: tag authenticated"),
    (_STEP("first detected"),
     lambda m: "AMS: spool detected"),
    (_STEP(r"select card fail, err (\d+)"),
     lambda m: f"AMS could not read the spool tag (err {m.group(1)})"),
    (_STEP(r"odom calib succ?ess(?:\s+exit\s+(\d+))?"),
     lambda m: "AMS finished measuring the spool"),
    (re.compile(r"\[RF\]\s*tray(\d+): info write to flash"),
     lambda m: (f"AMS: tag for bay {int(m.group(1)) + 1} cached in the unit's "
                f"flash (a later read returns it even after a swap)")),
    (re.compile(r"preload\s+finish", re.IGNORECASE),
     lambda m: "AMS staged the spool at its feeder"),
    # ── PLAIN ENGLISH FOR THE LINES THAT ACTUALLY MATTER ─────────────────
    # Suppressing the chatter is only half the job: what survives should
    # read like a sentence, not like a register dump. These are the events
    # an operator acts on, in the wording they would use themselves.
    #
    # The measurement result. "odom C:0.478,R:0.076,P:79%, od:0.491" is the
    # circumference, radius and percent from the unit's own two-edge
    # measure -- the percent is the only part a human wants.
    (re.compile(r"odom\s+C:[0-9.]+\s*,\s*R:([0-9.]+)\s*,\s*P:(\d+)%"),
     lambda m: (f"AMS measured the spool: about {m.group(2)}% left "
                f"(spool radius {float(m.group(1)) * 1000:.0f} mm)")),
    # The stored per-tray value, read back at power-up. This is the number
    # the unit keeps in its own flash and the only place it says it out
    # loud -- worth surfacing rather than burying.
    (re.compile(r"odom\s+load\s+from\s+flash\s*(\d+)\s*,\s*"
                r"R:[0-9.]+\s*,\s*P:(\d+)"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} remembers about "
                f"{m.group(2)}% left from its last measurement")),
    # The saved calibration -- the unit committing a fresh measure to flash.
    (re.compile(r"odom\s+save\s+tray:(\d+)"),
     lambda m: f"AMS stored a new measurement for bay {int(m.group(1)) + 1}"),
    # A feed that stalled. len_det is how far the filament actually got,
    # tube_len how far it should have gone -- the two numbers that tell you
    # whether it barely moved or nearly made it.
    (re.compile(r"feed\s+finish\s+-?\d+\s*,\s*stall\s*,\s*"
                r"len_det:([0-9.]+)\s*m\s*,\s*tube_len:([0-9.]+)\s*m"),
     lambda m: (f"AMS: the filament STALLED after {float(m.group(1)):.2f} m "
                f"of a {float(m.group(2)):.2f} m path -- check for a jam "
                f"between the bay and the toolhead")),
    # The unit's own error register changing. 0x00 -> anything is a fault
    # being raised; anything -> 0x00 is it clearing.
    (re.compile(r"err_code:0x00\s*->\s*0x([0-9A-Fa-f]+)"),
     lambda m: f"AMS raised error 0x{m.group(1).upper()}"),
    (re.compile(r"err_code:0x[0-9A-Fa-f]+\s*->\s*0x0+\b"),
     lambda m: "AMS cleared its error"),
    # A spool leaving the bay, and the calibration that goes with it.
    (re.compile(r"tray\s+(\d+)\s+out\s*,\s*clear\s+magic_num"),
     lambda m: f"AMS: bay {int(m.group(1)) + 1} is now empty"),
    # Power-up. The self-check is the unit's boot, which is worth one line
    # because it means everything it knew about follower state is gone.
    (re.compile(r"ams\s+pmsm\s+cali\s+finish", re.IGNORECASE),
     lambda m: "AMS finished its power-up self-check"),
    (re.compile(r"pmsm\s+self\s+check\s+good", re.IGNORECASE),
     lambda m: "AMS motor self-check passed"),
    # ── THE REMAINING EVENTS, FROM THE SAME INVENTORY ────────────────────
    # The second odometer edge -- one full turn of the spool, which is what
    # a real measurement needs. Its absence is the fast-path.
    (_STEP("second detected"),
     lambda m: "AMS: spool turned a full revolution (measuring)"),
    (_STEP(r"odom calib\s*,\s*tray (\d+)"),
     lambda m: f"AMS started measuring bay {int(m.group(1)) + 1}"),
    (_STEP(r"cali end"),
     lambda m: "AMS finished its measuring cycle"),
    (_STEP(r"Calibration rst:(\d+)"),
     lambda m: f"AMS finished measuring (result {m.group(1)})"),
    # A tag the unit cannot open -- almost always a non-Bambu spool.
    (_STEP(r"auth fail:-?(\d+)"),
     lambda m: ("AMS could not authenticate the tag -- third-party spool, "
                "or the tag is unreadable")),
    (re.compile(r"\[RF\]\s*tray(\d+): info same as last read"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} holds the same spool as "
                f"before")),
    # No stored calibration for this bay -- why a fast-path cannot happen.
    (_STEP(r"odom invalid tray (\d+)"),
     lambda m: (f"AMS: bay {int(m.group(1)) + 1} has no stored measurement "
                f"yet")),
    (_STEP(r"odom load tray (\d+) info invailed"),
     lambda m: f"AMS: bay {int(m.group(1)) + 1} has no stored measurement yet"),
    # The load fault we chased for hours -- worth naming exactly.
    (_STEP(r"odom tray_id error (\d+)"),
     lambda m: ("AMS: asked to move with NO TRAY SELECTED -- the unit "
                "rejected the command")),
    (re.compile(r"\[AMS_LED\]\s*TIMEOUT error (\d+)"),
     lambda m: "AMS: TIMEOUT -- the unit gave up on the move"),
    # Feed milestones, with the distance that makes them meaningful.
    (re.compile(r"feed to dw ok\s*,\s*len_det:([0-9.]+)\s*m"),
     lambda m: f"AMS: filament reached the hub after {float(m.group(1)):.2f} m"),
    (re.compile(r"feed finish\s*,\s*buff_pos:[0-9.]+\s*,\s*bldc_i:[0-9.]+A"
                r"\s*,\s*t:([0-9.]+)s"),
     lambda m: f"AMS finished feeding ({float(m.group(1)):.1f} s)"),
    (re.compile(r"new tube_len:(\d+)\s*mm"),
     lambda m: f"AMS learned the bay-to-hub path length: {m.group(1)} mm"),
    # Staging the spool at the feeder, and the bay lock that goes with it.
    (re.compile(r"preload start", re.IGNORECASE),
     lambda m: "AMS is staging the spool at its feeder"),
    # Power-up of the whole unit -- everything it knew about follower state
    # is gone, which is worth one visible line.
    (re.compile(r"\[AMS_ADA\]\s*init", re.IGNORECASE),
     lambda m: "AMS powered up (any follower state it held is gone)"),
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
    _STEP_SEP + r"(?:pull tray|rfid pull|start,read all card|search finished|"
    r"feed and judge|no card in RF|card auth|read success|feed with rfid|"
    r"empty to read|anticoll get UID|direct read card|search \d+ card)"
    r"|\[RF\]\s*tray\d+:"
    r"|\[AMS_RFID\]STEP:")

# The TERMINAL success markers, as opposed to the in-flight steps above. A read
# that runs is not a read that lands: the same insert emits "search finished,
# found 0 card" and "STEP5:no card in RF" on the way to a successful auth, so
# only these say a tag was actually recovered. Measured on an AMS 1 bay 3
# insert: "card auth success! [RF] tray2: info write to flash / read
# success,valid / read_done=1", 16s after the insert edge.
#
# The failure end of the same window looks like "tray pull over 790 mm, but no
# card detected" -- deliberately NOT matched here, because absence of success
# is what the caller tests and a unit can also simply go quiet.
#
# THIS PATTERN WAS HT-BLIND, AND THAT WAS THE WHOLE "HT KEEPS APPLYING THE OLD
# TAG" BUG. Every alternative began with a literal "STEP:" -- the boxed
# punctuation. The HT says "STEP3,": digit, comma. So on an AMS HT this regex
# could not match anything, ever, and rfid_read_succeeded_since() was hard-wired
# to False for that model. Measured, not deduced, from AFC_BambuAMS.log:
#
#   17:49:02  0x1800  [AMS_RFID] STEP3,auth card successful
#                     [RF] tray0: info write to flash
#                     [AMS_RFID] STEP3,save to flash ,card info valid
#   17:49:04  0x1800  [AMS_RFID] STEP3,read success ,goto Cali
#   17:49:22  module: "no readable tag profile in slot 0 ... the bay reader
#                      saw no chip"
#
# The unit read the tag, said so twice in plain language, and the module
# answered that it saw no chip. _finalize_scan's read_ok test is the only thing
# standing between a real read and the "apply lane defaults / keep the leftover
# record" path, so an HT took that path on every single insert -- including a
# re-insert of the SAME spool, which is precisely the case that has to work.
#
# Four firmware rounds were spent hunting this in the bridge (min holds, rescan
# evidence, insert gates, a stale-tag refusal), and every one of them was
# looking at a unit that had already answered correctly.
_RFID_READ_OK_RE = re.compile(
    _STEP_SEP + r"read success"
    r"|" + _STEP_SEP + r"read_done=1"
    r"|" + _STEP_SEP + r"feed with rfid success"
    # The HT's commit sentence: it has authenticated the chip and written the
    # record to its own flash. Stronger than "read success" -- this is the unit
    # stating the tag it now serves BELONGS to the spool in the bay, which is
    # exactly the question the caller is asking.
    r"|" + _STEP_SEP + r"save to flash"
    r"|card info valid"
    # The [RF]-prefixed commit lines: "trayN: info write to flash" on a fresh
    # read, "trayN: info same as last read" on a re-insert of the same spool.
    r"|info write to flash"
    r"|info same as last read")

# The HT's TERMINAL-only subset. On an HT, "feed with rfid success" and
# "read success" fire on FAILED sub-cycles too (measured: a retry loop
# emitted both, then "info_valid 0 or bbl:1", then finally committed) --
# only the flash-commit family says the tag landed. handle_line consults
# this pattern for 0x1800 narration and the full one for boxed.
_RFID_READ_OK_HT_RE = re.compile(
    _STEP_SEP + r"save to flash"
    r"|card info valid"
    r"|info write to flash"
    r"|info same as last read")

# End of the scan CYCLE, whatever its outcome. The unit emits this on both ends
# -- after "feed with rfid success" and after "tray pull over 790 mm, but no
# card detected" -- which makes it the only honest moment to say a tag did not
# read. Before it, "no tag" is a guess against a clock.
#
# The clock was wrong in practice: a bay-3 insert went quiet for 11s between its
# tray_readid chatter and the auth, so a 14s fallback announced "no readable tag"
# two seconds before the tag landed and then corrected itself. tray_readid cannot
# be used to bridge that gap (see _RFID_INFLIGHT_RE -- a unit can sit in it
# forever and never pull the tray), but STEP7 can, because a unit that gives up
# there never reaches it and the caller's hard cap ends the wait instead.
#: THE UNIT SAYS WHEN IT IS DONE -- listen, do not run a timer alongside it.
#: One terminal marker per dialect, taken from complete successful cycles in
#: the 2026-08-05 single-unit captures:
#:
#:   HT     [AMS_RFID] STEP4,Calibration rst:0
#:   AMS 1  [AMS_DEV]  STEP:odom calib success exit 0,dis:0.989
#:   AMS 2  [AMS_RFID] STEP7:cali end
#:
#: "STEP7:" on its own is NOT terminal -- the same captures carry
#: "STEP7:ready to cali tray" and "STEP7:info_valid 0 or bbl:-1" mid-cycle, so
#: matching the bare prefix would end the cycle before the measurement runs.
#: And the HT MISSPELLS success ("calib sucess"), hence succ?ess.
#: THE UNIT REFUSING A FOREIGN TAG, in its own words. A Mifare chip whose keys
#: are not Bambu's answers anticollision -- so the UID is readable -- and then
#: fails authentication:
#:
#:     [AMS_RFID]STEP:stop goto auth
#:     [AMS_RFID]STEP:auth fail:-4
#:     [AMS_RFID]STEP7:info_valid 0 or bbl:-1        (bbl = Bambu Lab)
#:
#: Worth telling apart from an empty bay, because "the bay reader saw no chip"
#: is simply WRONG for a Snapmaker or Elegoo spool: it saw one and could not
#: open it. Different problem, different thing for an operator to do.
_RFID_FOREIGN_TAG_RE = re.compile(
    # ONLY the auth refusal. "info_valid 0 or bbl:N" looked like foreign-tag
    # wording but the corpus shows it on EMPTY-BAY cycles (no card detected ->
    # info_valid 0 -> cali end) and mid-retry on HT reads that then succeeded
    # -- matching it worded empty bays as refused chips.
    r"auth fail\s*:\s*-?\d+",
    re.IGNORECASE)

_RFID_CYCLE_END_RE = re.compile(
    r"Calibration\s+rst:\d+"
    r"|odom\s+calib\s+succ?ess"
    r"|STEP7:\s*(?:finish|cali\s+end)"
    # An HT with the capacity measure disabled ends its cycle with "tray
    # capacity no en" and never says "Calibration rst:" at all -- without
    # this the scan-end stamp never advances and the bus claim rides its
    # 120 s backstop.
    r"|tray\s+capacity\s+no\s+en",
    # NOT "tray pull over N mm, no card detected" -- that line is INFLIGHT.
    # Tried 2026-08-10 and reverted the same hour: the dialect fixture
    # (ams2_insert_untagged, from a real capture) shows the unit still
    # finishing after it, with the true terminal -- "STEP7:finish,cali tray",
    # already matched above -- arriving next. Stamping on the pull-over line
    # ends the cycle EARLY, which is cueing on the wrong answer: the exact
    # failure the end-on-answers rule exists to prevent. The fixture caught
    # it before the hardware had to.
    re.IGNORECASE)

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
    # "[AMS_COMMON]preload_disable:1, tmpr:25.8, cd:0" then :0 again, both in
    # one frame, every 90 seconds on a boxed unit. The unit's own housekeeping
    # on a fixed timer -- it toggles preload, samples temperature, re-enables --
    # with no motion alongside and nothing an operator can act on.
    #
    # CONSOLE ONLY. only_debug suppresses the console line and nothing else:
    # AFC_BambuAMS.log still records it verbatim, and every parser has already
    # run by the time this is decided. Chamber temperature reaches the card
    # through [AMS_CHMB], not this line.
    r"|\[AMS_COMMON\]\s*preload_disable:\d+\s*,\s*tmpr:[0-9.]+\s*,\s*cd:\d+"
    # The RFID poller idling on an empty selection, and the state line it
    # rides with. Measured on the console: these three segments accounted
    # for essentially every line an operator saw while nothing was
    # happening -- "state:0,tray_now:255" / "STEP0:checking" /
    # "STEP0:idx 255 > 4", repeating a few times a second with tray_now 255
    # meaning NO TRAY IS SELECTED. It is the unit asking itself a question
    # about nothing. Console only, as above: the narration log still keeps
    # every line verbatim and every parser has already run.
    # states 0 and 3 ONLY, and only with NO tray selected. state:6 is an
    # AMS 1 fault and state:1 is a load in progress -- both are things an
    # operator must still see, and a bare state:\d+ swallowed them.
    # ...and states 0/3 with ANY tray, not only tray_now:255. During a print
    # the unit sits at "state:0,tray_now:1" between assist pulses -- idle,
    # with a tray simply still selected -- and that leaked to the console on
    # every frame. state 0 and 3 are both "not doing anything"; 1 (load in
    # progress), 6 (loaded/engaged) and 7 (STALLED) are excluded above and
    # stay visible.
    r"|\[AMS_COMMON\]\s*state:[03]\s*,\s*tray_now:\d+\s*,\s*tray_exit:\d+"
    r"|\[AMS_COMMON\]\s*en:\d+\s*,\s*mode:\d+\s*,\s*idx:\d+\s*,\s*ref:\d+"
    r"|\[AMS_RFID\]\s*STEP0:\s*checking"
    r"|\[AMS_RFID\]\s*STEP0:\s*idx\s+\d+\s*>\s*\d+"
    # ── THE FOUR THAT FILL THE CONSOLE DURING A PRINT ────────────────────
    # Counted off a live print: these were essentially every line an
    # operator saw for minutes at a time, and not one of them is
    # actionable. They are the assist mechanism doing its job.
    #
    #   [AMS_PMSM]mode:0->2 / 2->0   the assist motor cycling on and off,
    #                                several times a second, forever
    #   [AMS_LED]tray 1 loading      the bay LED restating itself
    #   [AMS_SWITCH]BUFF,pos:..      buffer arm position + motor current,
    #                                telemetry that already reaches the
    #                                dryer/buffer card as numbers
    #   [AMS_COMMON]state:4,..       "feeding, tray N selected" repeated for
    #                                the whole load; AFC already prints
    #                                "Loading laneN" once, which is the line
    #                                a human wants
    #
    # CONSOLE ONLY, like every rule above: AFC_BambuAMS.log still keeps all
    # of it verbatim, and every parser has already run before this is
    # decided. state 1/6/7 are deliberately NOT here -- load-in-progress,
    # loaded-engaged and STALLED are things an operator must still see.
    r"|\[AMS_PMSM\]\s*mode:\s*\d+\s*->\s*\d+"
    r"|\[AMS_LED\]\s*tray\s+\d+\s+\w+"
    r"|\[AMS_SWITCH\]\s*BUFF\s*,[^\[]*"
    r"|\[AMS_COMMON\]\s*state:4\s*,\s*tray_now:\d+\s*,\s*tray_exit:\d+"
    # ── THE REST OF THE MECHANISM, INVENTORIED NOT GUESSED ───────────────
    # Counted over 41,736 lines of live narration -- 161 distinct shapes.
    # Everything below is the unit talking to itself while it works. The
    # single biggest source by an order of magnitude:
    #
    #   [AMS_IDLE]set ams state assist, mode:4      9,541 lines
    #
    # ...the follower announcing it is still assisting, forever. The rest
    # are the RFID state machine's internal steps, the link layer's select
    # acks, and per-move bookkeeping. None of it is a decision, an outcome
    # or a fault; the ones that ARE get plain-English sentences instead
    # (see _AMS_HUMAN). Console only -- AFC.log keeps every line verbatim.
    r"|\[AMS_IDLE\]\s*set ams state assist[^\[]*"
    r"|\[AMS_LINK\]\s*en:\d+\s*,\s*mode:\d+\s*,\s*idx:\d+\s*,\s*ref:\d+"
    # The select ack the old rule missed: it expected "ams1", the wire says
    # "ams-0x00". 184 lines it never caught.
    r"|\[AMS_(?:LINK|CALL)\]\s*ams-?(?:0x)?[0-9A-Fa-f]+\s+select[^\[]*"
    # ([AMS_LINK]get_slot was NOT added here on purpose. It repeats hard,
    # but TestTheHeartbeatCannotBreakTheDedupe pins it visible: that
    # repetition is handled by the dedupe/"(xN repeated)" loop, and muting
    # it instead would hide a stuck unit re-asking for the same slot.)
    # Only the FEED-CYCLE transitions (state 3, filament riding through
    # the switch). "0 -> 1" and "1 -> 0" are a spool arriving at or
    # leaving the bay and stay visible -- pinned by the noise-filter test.
    r"|\[AMS_TRAY\]\s*tray\[?\d*\]?\s*sw_sta\s*update\s*,\s*"
    r"(?:3\s*->\s*\d+|\d+\s*->\s*3)[^\[]*"
    r"|\[AMS_BDC\]\s*tray lock:[^\[]*"
    r"|\[AMS_LED\]\s*mc set tray[^\[]*"
    r"|\[AMS_ENC\]\s*clc[^\[]*"
    r"|\[AMS_LINK\]\s*assist_err:[^\[]*"
    r"|\[AMS_SWITCH\]\s*(?:assist finish|reset dw length|retry:|AMS_CTRL_"
    r"|SWITCH_pull ignore|SWITHC_feed ignore|need to pull tray|feed tray:"
    r"|pull tray:|pull sucess)[^\[]*"
    # The RFID reader's own step machine. These are stages, not results --
    # the results (auth success, first/second detected, the measurement,
    # cali end) are translated and stay.
    r"|\[AMS_(?:RFID|DEV)\]\s*STEP\d*[:,]?\s*(?:odom search|set \d+ tray_readid"
    r"|rfid pull|time_reset|stop goto auth|check pass|checking|open_PCD"
    r"|reader \d+ enable|pull tension|goS\d|done\d|search \d+ card"
    r"|anticoll get UID|direct read card|empty to read|select card success"
    r"|pull back|start,read all card|ready to cali tray|no card in RF"
    r"|after tension|confirm RF have no card|odom select rslt"
    r"|odom reset tray|cali read tray)[^\[]*"
    # Motor/encoder self-test internals at power-up. "self check good" and
    # "cali finish" are translated; the ADC dumps behind them are not.
    r"|\[AMS_PMSM(?:_[A-Z])?\]\s*(?:adc\d|timeout, retry|get ams_id"
    r"|has ams_id|P cali init|table_xy)[^\[]*"
    r"))+\s*$")

#: The AMS's 10-second liveness heartbeat, as a segment rather than a line.
#: The unit bundles it into whatever frame is going out, so it has to be
#: removable from the middle of a sentence, not just recognised as a whole one.
#: The colon is OPTIONAL. Both forms are real -- "[DBG] ams time: now=42044054ms
#: diff=10005ms" on the wire today, and a bare "[DBG] ams time 12345" that a
#: test pins because it was seen too. Requiring the colon let the second form
#: through to the console the moment the substring drop below was removed.
_DBG_AMSTIME_RE = re.compile(r"\[DBG\]\s*ams time\b[^\[]*")


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
#: Motion acks that ride the follower's own cadence rather than marking an
#: operator-visible decision. Console-suppressed (AFC.log keeps them all):
#: during a print these repeat every few seconds for the length of the job.
_ACK_ROUTINE = frozenset(("assist", "select", "stop", "hold", "follow"))

#: Sentinel for "no value yet", where None is itself meaningful.
_UNSET = object()

_BRIDGE_EVENTS_KNOWN = frozenset((
    "status", "reply", "error", "ack", "amsdbg", "sniff", "chain", "info",
    "sniff_mode", "m3", "rc", "rollcall", "clsprobe",
    # command echoes
    "dry", "mon", "resync", "mcaddr", "armms", "arrivems", "hb",
    "htpoll", "htid", "htunit", "drain", "mute", "units", "variant", "baud",
    # ("load" was here too -- the firmware's bb_do_load replayed feed frames
    # hard-wired to unit 0x00 and nothing ever sent it; removed with the
    # addressing sweep. "unload" stayed: bridge_unload() genuinely uses it,
    # and it is unit/slot-addressed now.)
    "parity", "en", "replay", "unload", "rdinfo", "relink", "rehome",
    "capscan", "m6", "p0f", "poll", "extmimic", "ht0fhold",
    "tail", "arrived", "txecho",
    # Scan-path echoes. Both are the bridge repeating back a command we sent
    # ("scan" with state start/done, "reread" naming the bay it invalidated),
    # so they belong with the echoes above and not in the catch-all -- where
    # every tag scan logged an "unhandled bridge event" line for a message
    # that was working exactly as designed. The bind/htuid echoes once flooded
    # this same path badly enough to starve the MCU.
    "scan", "reid", "reread", "prime",
    # "tx" is the EVENT (the frames we transmit); "txecho" is only the command
    # echo. Both must be here: the not-in-_BRIDGE_EVENTS_KNOWN catch-all above
    # sits BEFORE the per-event branches, so an event missing from this set is
    # swallowed as "unhandled" and its handler never runs -- which is exactly
    # how rc/rollcall went missing, and then how the TX echo recorded nothing
    # on its first real use.
    "tx", "loops",     # both need a HANDLER below, not just membership here:
                       # being "known" without one is worse than unknown --
                       # the catch-all stops logging it and it vanishes.
    # ENROLLMENT ECHOES. The firmware emits one `bind` per known unit and a
    # `htuid` per HT every time the chain is re-asserted, which is every status
    # round. With three units on the wire that is a steady stream, and being
    # absent from this set sent every one of them down the "unhandled bridge
    # event" path -- 69 log lines a second inside Klipper's process, measured.
    #
    # THAT IS NOT A COSMETIC COST. It starved the reactor until the CAN toolhead
    # missed a scheduled pin event and the MCU shut down:
    #
    #   MCU 'EBBT0' shutdown: Missed scheduling of next digital out event
    #
    # They are pure telemetry -- the host already learns the chain from `chain`
    # -- so they belong here as known-and-ignored rather than as a per-round
    # logging storm. If a handler is ever wanted, add it BELOW as well; membership
    # alone would silence them completely.
    "bind", "htuid",
))

# ── Bridge connection (threaded reader, reactor hop) ────────────────────────────

class BambuBridge:
    """Serial link to the Pico bridge: background reader + JSON command writer.

    One bridge per physical Pico. Multiple AFC units (daisy-chained AMS on the
    same bus) share it and each register a status listener via add_listener().
    """

    # Console-bound narration lines per second before the rest of a burst is
    # sent to AFC.log only. Generous enough that a normal load or scan (a few
    # lines a second) never trips it; low enough that a retry storm cannot
    # fill Klipper's gcode pipe and stall the reactor. See handle_line.
    NARRATION_CONSOLE_MAX_PER_S = 12

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
        #: addr -> {"rotate": int, "dur": int, "tmpr": int}, as the unit last
        #: echoed them. See _DRY_CFG_RE.
        self._dry_cfg: Dict[int, Dict[str, int]] = {}
        self._cap_measure: Dict[int, Dict[str, Any]] = {}
        #: addr -> {"rst": int, "t": float}; the AMS HT's calibration verdict,
        #: which it reports INSTEAD of the boxed units' percent line.
        self._ht_cali: Dict[int, Dict[str, Any]] = {}
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
        # Per-UNIT path length, and which unit is currently being commanded.
        #
        # The device address cannot tell two units of the same class apart:
        # an AMS 1 and an AMS 2 Pro both narrate as 0x0700, so keying by
        # address alone let one unit's measurement land in the other's config.
        # Live risk on this rig -- AMS 2 measured 3532 mm while AMS 1 was still
        # on the 3000 mm default.
        #
        # The host knows which unit it commanded, and tube_len is only ever
        # narrated by the unit doing the feed, so the active unit is a better
        # key than the address. The address map is kept as the fallback for a
        # single-unit bus and for anything that has not set an active unit.
        self._tube_by_unit: Dict[int, float] = {}
        # Per-unit dw_len, and how many times each unit has said it. The COUNT
        # is the point: one reading proves the word exists, and only a run of
        # them says whether the figure is stable enough to trust. Keyed by unit
        # for the same reason tube_len is -- two boxed units share address
        # 0x0700, so the address cannot tell them apart.
        self._dw_by_unit: Dict[int, float] = {}
        self._dw_n_by_unit: Dict[int, int] = {}
        # The DEVICE ADDRESS the value actually arrived on, kept beside it.
        #
        # _active_unit is set when a load starts and never cleared, so it names
        # whichever unit loaded LAST -- fine while a load is running, wrong for
        # anything narrated afterwards. Storing the addr lets a consumer refuse
        # a value that reached it under another unit's device (an HT is 0x1800
        # and a boxed AMS 0x0700, so the mismatch that matters is catchable).
        # The two boxed units share 0x0700 and cannot be told apart this way,
        # which is exactly why the unit key exists as well.
        self._dw_addr_by_unit: Dict[int, int] = {}
        self._active_unit: Optional[int] = None
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
        # Monotonic time of the last narration line reporting a tag read that
        # actually SUCCEEDED (see _RFID_READ_OK_RE). In-flight above says a read
        # is running; this says one landed, which is a different question and the
        # one that decides whether a slot record can be trusted as the new
        # spool's. Bridge-wide for the same reason and with the same consequence.
        self._rfid_ok_t: Optional[float] = None
        # Monotonic time the last scan CYCLE ended (see _RFID_CYCLE_END_RE),
        # success or failure alike. Bridge-wide, as above.
        self._rfid_end_t: Optional[float] = None
        # THE SAME THREE STAMPS, KEYED BY THE DEVICE THAT SAID IT.
        #
        # The bridge-wide stamps above stay as the fallback (and are what a
        # caller that passes no address still gets), but they cross-credit: an
        # AMS 1 narrating "read success" while an HT is mid-scan hands the HT a
        # success it never had. That is not a corner case here -- an insert in
        # one unit routinely overlaps a scan in another, and it is on record:
        # at 17:49:05 an AMS 1 bay-3 insert began 3s into the HT's scan window.
        #
        # The address separates an HT (0x1800) from a boxed unit (0x0700) and
        # NOT the two boxed units from each other, which both answer at 0x0700.
        # That is the same limit the chamber telemetry has, documented in the
        # same terms: it is a real improvement over one shared stamp, and it is
        # not attribution by chain index. Where a boxed pair must be told apart,
        # the unit key is the answer -- narration does not carry one.
        self._rfid_step_by_addr: Dict[int, float] = {}
        self._rfid_ok_by_addr: Dict[int, float] = {}
        self._rfid_end_by_addr: Dict[int, float] = {}
        # The unit said a chip is present but its keys are not Bambu's
        # ("auth fail:-4"). Distinguishes a THIRD-PARTY tag from an empty bay.
        self._rfid_foreign_t: Optional[float] = None
        self._rfid_foreign_by_addr: Dict[int, float] = {}
        # Last motion completion the AMS itself reported ("feed finish",
        # "preload finish", "pull finish"), as (sequence, ok, text). The bridge
        # gives no ack for move COMPLETION -- only that the command was
        # accepted -- so without this the host can only guess a move's duration
        # from distance/speed, and the AMS does not move at the speed we ask
        # for. Sequence increments per event so a waiter can tell a fresh
        # completion from a stale one.
        self._finish_seq: int = 0
        # Bumped each time the unit finishes its native mode:4 pull.
        self._pull_seq: int = 0
        # Bumped when the unit finishes the PUSH-FORWARD half.
        self._assist_seq: int = 0
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
        # The unit's last reported error LEVEL (0 = healthy) and when.
        # None means it has never said, which is not the same as zero.
        self._err_code: Optional[int] = None
        self._err_code_t: float = 0.0
        self._bldc_i: float = 0.0
        self._chain_uids: List[str] = []       # index -> 24-hex UID (from `chain`)
        self._last_raw_reply: str = ""         # last `reply` frame (diagnostic)
        # Last {"evt":"idsave"} outcome as (state, n), or None if none since
        # the caller cleared it. AFC_BAMBU_SAVEIDS waits on this rather than a
        # clock: the firmware answers every idsave, so there is a real answer
        # to wait for and no reason to guess how long a flash write takes.
        self._last_idsave: Optional[tuple] = None
        # Last {"evt":"mmfix"} payload, so AFC_BAMBU_MMFIX can report the
        # firmware's own counters rather than a second copy kept here.
        self._last_mmfix: Optional[dict] = None
        # True between sending {"cmd":"reset"} and the disconnect it causes, so
        # the reader can tell "the Pico is rebooting because we said so" from
        # "the link died".
        self._expect_reset: bool = False
        # Last {"cmd":"rdinfo"} result: the RAW 0x0211 filament-info reply for
        # one bay, straight off the wire and before any decode. The one way to
        # tell "the unit did not send this field" from "our decode missed it".
        self._last_rdinfo: dict = {}
        self._last_m3: Optional[dict] = None   # last m3 diagnostics (on request)
        # Roll-call: the firmware's address register (probes/answers/mask), and
        # whether it is running. Both only on request; None means never asked.
        self._last_rc: Optional[dict] = None
        self._last_clsprobe: Optional[dict] = None
        # Bus-wide spool-operation ownership (see try_claim_bus).
        self._bus_owner: Optional[str] = None
        self._bus_claim_t: float = 0.0
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

    def last_err_code(self) -> Tuple[Optional[int], float]:
        """
        Return the unit's last reported error level and when it said it.

        This is a LEVEL, not an event: 0 means the unit currently reports no
        error, and None means it has never reported one at all -- which is not
        the same thing and must not be treated as healthy.

        Use it to answer "is this unit still in error", e.g. before resuming a
        print. Do NOT use it to detect a fault occurring -- err_code cycles
        during healthy operation; the stall detector keys on the unit's words
        instead.

        :return tuple: (err_code or None, monotonic time it was reported)
        """
        with self._lock:
            return (self._err_code, self._err_code_t)

    def last_scan_end(self) -> Optional[float]:
        """
        When the AMS last said its scan/measure cycle finished.

        The unit announces this -- "Calibration rst:0" on an HT, "odom calib
        success exit 0" on an AMS 1, "STEP7:cali end" on an AMS 2 -- so a
        caller can wait for the real end instead of guessing with a timer.

        :return float: monotonic time of the last cycle end, or None
        """
        with self._lock:
            return self._rfid_end_t

    # ── BUS-WIDE SPOOL-OPERATION OWNERSHIP ──────────────────────────────────
    # One spool operation on the bus at a time. We guarded a scan PER SLOT and
    # nothing guarded the BUS, so during a relink two units scanned at once and
    # Klipper crashed.
    #
    # A real printer never overlaps them and narrates the handoff out loud:
    # "[AMS_CALL] ams1 select, select ams2". One owner, and the select lives
    # inside an owned transaction.
    #
    # The claim is released by the UNIT's own cycle-end marker (_rfid_end_t,
    # what last_scan_end reports -- "Calibration rst:0" on an HT, "odom calib
    # success exit 0" on an AMS 1, "STEP7:cali end" on an AMS 2), not by a
    # timer. The timer is only a backstop for a unit that never announces, and
    # it is deliberately generous: a real scan-and-measure runs ~60 s.
    BUS_CLAIM_MAX_S = 120.0

    def try_claim_bus(self, owner: str, now: float) -> bool:
        """
        Claim the bus for a spool operation, or report that someone else has it.

        :param owner: a stable name for the claimant (the unit's name)
        :param now: reactor monotonic time
        :return bool: True if the caller may proceed
        """
        with self._lock:
            cur = getattr(self, "_bus_owner", None)
            if cur is not None and cur != owner:
                claimed = getattr(self, "_bus_claim_t", 0.0)
                ended = self._rfid_end_t
                done = (ended is not None and ended >= claimed)
                if not done and (now - claimed) < self.BUS_CLAIM_MAX_S:
                    return False                  # genuinely busy elsewhere
            self._bus_owner = owner
            self._bus_claim_t = now
            return True

    def release_bus(self, owner: str) -> None:
        """
        Drop the claim if we hold it. Safe to call when we do not.

        :param owner: the claimant name that was passed to claim_bus
        """
        with self._lock:
            if getattr(self, "_bus_owner", None) == owner:
                self._bus_owner = None

    def bus_owner(self) -> Optional[str]:
        """Who holds the bus for a spool operation, if anyone."""
        with self._lock:
            return getattr(self, "_bus_owner", None)

    def last_fault(self) -> Tuple[int, str, float]:
        """
        Return the AMS's last self-reported stall.

        :return tuple: (sequence, text, last motor current in A)
        """
        with self._lock:
            return (self._fault_seq, self._fault_text, self._bldc_i)

    def set_narration_log(self, log_dir: str, tag: str = "",
                          max_bytes: int = 10 * 1024 * 1024) -> bool:
        """
        Send the AMS's own narration to its own file.

        Narration gets its own file and handler: always written, never on the
        console, and independent of AFC's `debug` flag, so every STEP, finish,
        stall and measured length stays on record even with debug off.
        Rotates at 10 MB keeping NO backups -- it is a rolling window for
        diagnosis, not an archive.

        :param log_dir: directory to write into (Klipper's log directory)
        :param max_bytes: rotate at this size; 0 disables rotation
        :return bool: True if the log is ready
        """
        if self._nar_lg is not None:
            return True
        # ONE LOG PER BUS MASTER. Two Picos writing one file cannot be
        # untangled afterwards: the only per-line attribution is the device
        # address, and two boxed units on different buses both narrate as
        # 0x0700. `tag` names the master -- empty for the first, so a
        # single-Pico printer writes plain AFC_BambuAMS.log.
        suffix = ("_" + tag) if tag else ""
        lg = logging.getLogger("AFC_BambuAMS_file" + suffix)
        # isinstance, NOT `if not lg.handlers`. logging.getLogger() is
        # process-global, so anything that attached a handler first -- pytest,
        # another unit, a reload -- makes the truthy check skip setup and hand
        # back a logger with no file, which reports success and writes
        # nowhere. That exact bug shipped once in the bus-monitor logger.
        if not any(isinstance(h, logging.handlers.RotatingFileHandler)
                   for h in lg.handlers):
            try:
                fh = _TruncatingRotatingFileHandler(
                    os.path.join(log_dir, "AFC_BambuAMS%s.log" % suffix),
                    maxBytes=max_bytes, backupCount=0)
                fh.setFormatter(logging.Formatter(
                    "%(asctime)s %(message)s", datefmt="%H:%M:%S"))
                lg.addHandler(fh)
            except Exception as e:
                self.logger.warning(
                    "AFC bambu: could not open AFC_BambuAMS%s.log: %s"
                    % (suffix, e))
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

        Written BEFORE the console dedupe, so a line repeating hundreds of
        times still shows the shape of a stuck loop. The address is included
        so a bus carrying several units stays attributable.

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

        Reported at two moments: "e_in" as the filament enters the extruder,
        and "feed finish" when the load completes. End-of-load reads ~1.28 on
        an HT -- hard compressed -- which is the reference for buffer ramming.

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

        Recorded BEFORE the dedupe, because the AMS repeats the refusal on
        every retry and a deduped repeat still means "still refusing". Cleared
        as soon as that unit reports heating or self-checking, so a stale
        reason cannot outlive the condition.

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

    def _note_dry_cfg(self, text: Optional[str], addr: Optional[int]) -> None:
        """
        Record the drying settings a device echoes back.

        Kept even when the command is refused -- the echo reports what the unit
        is HOLDING, which a refusal does not erase. Cleared only by a cycle
        finishing with dur:0, which the unit emits itself.

        :param text: the narration line
        :param addr: device address that produced it
        """
        if not text or not addr:
            return
        try:
            m = _DRY_CFG_RE.search(text)
            if not m:
                return
            with self._lock:
                self._dry_cfg[int(addr)] = {
                    "rotate": 1 if (int(m.group(1)) or int(m.group(2))) else 0,
                    "dur":    int(m.group(5)),
                    "tmpr":   int(m.group(6)),
                }
        except Exception:
            pass          # a diagnostic must never take the reader thread down

    def last_dry_cfg(self, addr: Optional[int]) -> Optional[Dict[str, int]]:
        """
        The drying settings this device last echoed, or None.

        :param addr: device address (an HT is 0x1800, a boxed AMS 0x0700)
        :return: dict with rotate/dur/tmpr, or None if it has never echoed
        """
        if not addr:
            return None
        with self._lock:
            cfg = self._dry_cfg.get(int(addr))
            return dict(cfg) if cfg else None

    def _note_cap_measure(self, text: Optional[str], addr: Optional[int],
                          now: float) -> None:
        """
        Record a narrated capacity measurement (see _CAP_MEASURE_RE).

        :param text: the narration line
        :param addr: device address that produced it
        :param now: reactor-monotonic receive time, so consumers can tell a
            fresh measurement from a stale one
        """
        if not text or not addr:
            return
        try:
            # The AMS HT's calibration verdict, which carries no percent (see
            # _HT_CALI_RST_RE). Recorded alongside the boxed measurement so a
            # caller can tell "the cycle finished" from "the cycle was never
            # heard from" -- watching only for a percent made every successful
            # HT calibration look like silence.
            c = _HT_CALI_RST_RE.search(text)
            if c:
                with self._lock:
                    self._ht_cali[int(addr)] = {"rst": int(c.group(1)),
                                                "t": now}
            else:
                # The AMS 1 has no "Calibration rst:N" line at all -- it says
                # "odom calib success exit 0" instead. Map a success onto the
                # same rst 0 the other units report, so a caller does not need
                # to know which generation it is talking to.
                d = _CALI_DONE_RE.search(text)
                if d:
                    with self._lock:
                        self._ht_cali[int(addr)] = {"rst": 0, "t": now}
            m = _CAP_MEASURE_RE.search(text)
            if not m:
                return
            tray, circ, radius, pct = m.groups()
            with self._lock:
                self._cap_measure[int(addr)] = {
                    # Clamped for callers that drive a 0-100 display, but the
                    # raw value is kept: a fresh spool legitimately reads over
                    # 100 (107% captured on the HT), and silently flattening
                    # that to 100 would throw away a real measurement.
                    "pct": min(100, int(pct)),
                    "pct_raw": int(pct),
                    "radius_m": float(radius),
                    # Present only on a live measurement; a restore from flash
                    # reports no circumference. None means "not stated", which
                    # is not the same as zero.
                    "circumference_m": float(circ) if circ else None,
                    # The restore form names its tray; the live form does not,
                    # because it can only be the tray just measured.
                    "tray": int(tray) if tray is not None else None,
                    # Distinguishes "the unit just measured this" from "the
                    # unit recalled it at power-on". Both are valid readings,
                    # but only the first means a spool was physically pulled.
                    "restored": tray is not None,
                    "t": now,
                }
        except Exception:
            pass          # a diagnostic must never take the reader thread down

    def last_ht_cali(self, addr: Optional[int]) -> Optional[Dict[str, Any]]:
        """
        The most recent AMS HT calibration verdict, or None.

        :param addr: device address (an HT is 0x1800)
        :return: dict with rst / t -- rst 0 = completed, 1 = refused
            ("capacity no en"), 4 = aborted (stall during calib)
        """
        if not addr:
            return None
        with self._lock:
            m = self._ht_cali.get(int(addr))
            return dict(m) if m else None

    def last_cap_measure(self, addr: Optional[int]) -> Optional[Dict[str, Any]]:
        """
        The most recent capacity measurement this device narrated, or None.

        :param addr: device address (an HT is 0x1800, a boxed AMS 0x0700)
        :return: dict with pct / radius_m / t, or None
        """
        if not addr:
            return None
        with self._lock:
            m = self._cap_measure.get(int(addr))
            return dict(m) if m else None

    def clear_dry_error(self, addr: Optional[int]) -> None:
        """
        Forget why this device last refused, because a NEW attempt is starting.

        The error describes the LAST ATTEMPT, so a new attempt is what resets
        it. Called as a dry is commanded, before anything reaches the bus.

        Narration alone is not enough: the only lines that clear it are
        CTC_STATE_HEATING and CTC_STATE_SELF_CHECK, and an AMS HT emits
        neither, so without this one refusal would read as "refused" forever.

        :param addr: device address (an HT is 0x1800, a boxed AMS 0x0700)
        """
        if not addr:
            return
        with self._lock:
            self._dry_err.pop(int(addr), None)

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

        "finish -1, stall" is not a failure on every unit: an AMS HT ends a
        normal load by feeding to the end of its PTFE and stalling against the
        extruder gear, which is how it knows it has arrived. Reading the word
        "stall" as failure marks a good load failed. What matters is how far
        it got, not that it stalled:

        1. no stall reported at all -> success
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

        `fstate` is what the move-completion wait keys on, so the trace sits
        next to the narration on the same clock. Changes only: the field rides
        every status frame, several a second, and logging all of them would
        bury the narration beside it.

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

    def set_active_unit(self, unit: Optional[int]) -> None:
        """
        Name the unit currently being commanded, so narration it produces is
        attributed to it rather than to its device address.

        Two units of the same class share an address (an AMS 1 and an AMS 2 Pro
        are both 0x0700), so the address alone cannot say which one spoke --
        but the host knows, having issued the move. Set around a load, cleared
        after.

        :param unit: chain index, or None to clear
        """
        with self._lock:
            self._active_unit = None if unit is None else int(unit)

    def dw_len(self, unit: Optional[int] = None) -> tuple:
        """
        The last dw_len a unit reported at the end of a feed, and how many.

        REPORTED, NEVER USED -- see _DW_LEN_M_RE for why. The count matters as
        much as the value: one reading proves only that the word exists.

        :param unit: chain index; None returns the most recent from any unit
        :return tuple: (mm, count, addr) -- (None, 0, 0) if never reported
        """
        with self._lock:
            if unit is not None:
                u = int(unit)
                return (self._dw_by_unit.get(u),
                        self._dw_n_by_unit.get(u, 0),
                        self._dw_addr_by_unit.get(u, 0))
            if not self._dw_by_unit:
                return (None, 0, 0)
            u = max(self._dw_n_by_unit, key=self._dw_n_by_unit.get)
            return (self._dw_by_unit.get(u),
                    self._dw_n_by_unit.get(u, 0),
                    self._dw_addr_by_unit.get(u, 0))

    def tube_len(self, addr: Optional[int] = None,
                 unit: Optional[int] = None) -> Optional[float]:
        """
        The AMS's own measured PTFE path length in mm, if it has told us.

        The unit learns this from consecutive feed measurements and narrates
        it, so it is the real distance on this machine and beats a configured
        estimate. Only available once it has enough samples; it reports 0
        until then.

        :param addr: Device address to look up (0x0700 AMS, 0x1800 HT); None
            returns the most recent from any unit
        :return Optional[float]: length in mm, or None if never reported
        """
        with self._lock:
            if unit is not None:
                if int(unit) in self._tube_by_unit:
                    return self._tube_by_unit[int(unit)]
                if self._tube_by_unit:
                    # Per-unit attribution is in play and this unit has not
                    # measured yet. Do NOT fall through to the address: two
                    # units of the same class share one (an AMS 1 and an AMS 2
                    # Pro are both 0x0700), so the address map holds whichever
                    # of them measured last.
                    #
                    # That fallback made the fix useless. Observed with both
                    # boxed units present: AMS 2 measured 3532 mm, and AMS 1 --
                    # which had never measured -- read 3532 mm through the
                    # address and would have adopted it as its own path on its
                    # next load.
                    #
                    # "Unknown" is the correct answer for a unit that has not
                    # measured. It keeps the configured value, which is what
                    # an un-calibrated unit should use.
                    return None
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

    def last_pull(self) -> int:
        """
        Sequence number of the AMS's last completed mode:4 pull.

        The unit pulls the tray back on the mode change into mode:4 (native,
        0.5-2.2s) and the extruder must not advance into it. Wait for this to
        change rather than for a fixed settle: measured, a pull ended 2.02s
        after the assist was armed against a 2.00s blind settle, so a timer
        releases the advance 20ms early.

        :return int: increments once per completed mode:4 pull
        """
        with self._lock:
            return self._pull_seq

    def last_assist_done(self) -> int:
        """
        Sequence of the last completed assist cycle (the PUSH-FORWARD half).

        The unit's seating cycle is pull back THEN push forward. `last_pull`
        marks the end of the pull; this marks the end of the push. Releasing
        the extruder on the pull alone just moves the fight later.

        :return int: increments once per "assist finish"
        """
        with self._lock:
            return self._assist_seq

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
        # Arm the expected-disconnect flag BEFORE the write: a reset can take
        # the port down before this call even returns.
        if obj.get("cmd") in ("reset", "bootsel"):
            self._expect_reset = True
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

        Everything the unit says already reaches AFC.log verbatim; this picks
        the handful an operator wants to see and renders them plainly. A
        refused dry is the case that matters: the AMS answers
        "[AMS_CHMB]ignore dry_mode:1, ams_state:2" and AFC_BAMBU_HEATER_START
        reports success either way, so the refusal is otherwise invisible.

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
                if self._active_unit is not None:
                    self._tube_by_unit[int(self._active_unit)] = mm
            if prev is None:
                self.logger.info(
                    f"AFC bambu {self.name}: AMS 0x{int(addr):04X} reports its "
                    f"measured filament path as {mm:.0f}mm -- using it to size "
                    f"move timeouts instead of the configured estimate")

        # dw_len: RECORDED, NOT USED. See _DW_LEN_M_RE. Same > 0 rule as
        # tube_len and for a sharper reason -- a load that fails reports
        # dw_len:0.000, so zero here means "this feed measured nothing",
        # which is exactly the value that must never reach a deadline.
        dw = _DW_LEN_M_RE.search(text)
        if dw:
            dw_mm = float(dw.group(1)) * 1000.0
            if dw_mm > 0.0 and self._active_unit is not None:
                with self._lock:
                    u = int(self._active_unit)
                    self._dw_by_unit[u] = dw_mm
                    self._dw_n_by_unit[u] = self._dw_n_by_unit.get(u, 0) + 1
                    self._dw_addr_by_unit[u] = int(addr or 0)

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

    def _rfid_stamp(self, wide: Optional[float], by_addr: Dict[int, float],
                    addr: Optional[int]) -> Optional[float]:
        """
        Pick the narration stamp for ``addr``, or the bridge-wide one.

        An address the bridge has never heard narrate falls back to the shared
        stamp rather than reading as "never", so a unit whose firmware predates
        per-device attribution is not reported silent. Once it HAS narrated its
        own stamp wins, and another unit's chatter cannot be credited to it.

        :param wide: the bridge-wide stamp
        :param by_addr: per-device stamps
        :param addr: device address to resolve, or None for bridge-wide
        :return: the stamp to compare against, or None if there is none
        """
        if addr:
            got = by_addr.get(int(addr))
            if got is not None:
                return got
            if by_addr:
                # This bridge attributes narration and this device has said
                # nothing of the kind. That is an answer, not a gap.
                return None
        return wide

    def rfid_read_in_flight(self, now: float, quiet: float = 3.0,
                            addr: Optional[int] = None) -> bool:
        """
        Whether an AMS on this bridge is mid tag-read right now.

        True while narration matching ``_RFID_INFLIGHT_RE`` has been seen within
        ``quiet`` seconds. The AMS emits those steps every few hundred ms once a
        read is underway, so a gap of several seconds means it has finished or
        given up.

        :param now: Reactor monotonic time
        :param quiet: Seconds of narration silence that end a read
        :param addr: device address to scope the answer to (0x1800 = an HT);
                     None answers for the bridge as a whole
        :return bool: True if a read appears to still be running
        """
        t = self._rfid_stamp(self._rfid_step_t, self._rfid_step_by_addr, addr)
        return t is not None and (now - t) < quiet

    def rfid_read_succeeded_since(self, since: Optional[float],
                                  addr: Optional[int] = None) -> bool:
        """
        Whether a tag read has LANDED since ``since``.

        Distinct from ``rfid_read_in_flight``: that says a read is running, this
        says one recovered a tag. The caller needs the second to decide whether a
        slot's profile record belongs to the spool now in the bay or to the one
        before it -- an AMS reports its stored record for a bay from the moment a
        spool goes in, long before the reader has seen the new tag.

        Scoped to ``addr`` when one is given: this decides whether a bay's
        record is the NEW spool's, so crediting it to the wrong unit applies
        wrong data. Two boxed units still share 0x0700 and cannot be separated
        this way.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to (0x1800 = an HT);
                     None answers for the bridge as a whole
        :return bool: True if a successful read was narrated at or after ``since``
        """
        ok = self._rfid_stamp(self._rfid_ok_t, self._rfid_ok_by_addr, addr)
        return ok is not None and since is not None and ok >= since

    def last_rdinfo(self) -> dict:
        """
        The last raw 0x0211 reply captured by {"cmd":"rdinfo"}.

        :return dict: {"unit","slot","len","hex"} or {} if none yet
        """
        with self._lock:
            return dict(self._last_rdinfo)

    def rfid_foreign_tag_since(self, since: Optional[float],
                               addr: Optional[int] = None) -> bool:
        """
        Whether the unit refused a chip it could not authenticate since ``since``.

        A Mifare tag whose keys are not Bambu's still answers anticollision, so
        its UID is readable; only the profile is locked. That is a THIRD-PARTY
        SPOOL, not an empty bay, and the two need different words.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to
        :return bool: True if a foreign tag was refused at or after ``since``
        """
        t = self._rfid_stamp(self._rfid_foreign_t,
                             self._rfid_foreign_by_addr, addr)
        return t is not None and since is not None and t >= since

    def rfid_cycle_ended_since(self, since: Optional[float],
                               addr: Optional[int] = None) -> bool:
        """
        Whether a scan cycle has RUN TO COMPLETION since ``since``.

        The unit narrates ``STEP7:finish,cali tray`` at the end of a scan
        whatever the outcome, so this is the moment -- and the only moment --
        at which "no tag read" is a fact rather than a guess against a clock.

        Scoped to ``addr`` when one is given, like the stamps above.

        :param since: Reactor monotonic time to compare against (None = never)
        :param addr: device address to scope the answer to (0x1800 = an HT);
                     None answers for the bridge as a whole
        :return bool: True if a cycle ended at or after ``since``
        """
        end = self._rfid_stamp(self._rfid_end_t, self._rfid_end_by_addr, addr)
        return end is not None and since is not None and end >= since

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
        elif obj.get("evt") in ("reply", "raw"):
            # Raw AMS frame. Diagnostic only -- held here so a probe can print
            # it without a shell on the printer.
            #
            # TWO event names, and they were not both handled. AFC_BAMBU_BUFFER_PROBE
            # gets {"evt":"reply","hex":...}; the firmware's {"cmd":"raw"}
            # answers with {"evt":"raw","rx":...}. Listening for the first alone
            # meant every raw transaction completed on the wire and its answer
            # was dropped on the floor here, reporting "no reply" for a frame the
            # AMS had in fact returned.
            with self._lock:
                self._last_raw_reply = str(
                    obj.get("hex") or obj.get("rx") or "")
        elif obj.get("evt") == "error":
            self.logger.warning(f"AFC bambu: bridge error: {obj.get('msg')}")
        elif obj.get("evt") == "idsave":
            # The identity table (uid -> chain index + model) persisted on the
            # Pico, so the next power-up enrols with the class, order and model
            # already known instead of guessing from announce order.
            #
            # "match" is the answer on every ordinary boot and means no flash
            # was touched. "written" should appear once after a config change
            # or on a fresh Pico -- if it appears every boot, the record is not
            # sticking and that is worth knowing, so it is INFO, not debug.
            state = obj.get("state")
            n = obj.get("n")
            # Latched for AFC_BAMBU_SAVEIDS, which must not reboot the Pico
            # until the firmware has actually said the record is down.
            with self._lock:
                self._last_idsave = (str(state or ""), n)
            if state == "wiped":
                self.logger.info(
                    "AFC bambu: bridge ERASED its stored unit identities; the "
                    "next power-up will enroll from announce order until prep "
                    "writes a fresh record")
            elif state == "written":
                self.logger.info(
                    f"AFC bambu: bridge stored {n} unit identities -- the next "
                    f"restart will enroll from them")
            elif state == "failed":
                self.logger.warning(
                    "AFC bambu: bridge could NOT store the unit identities; "
                    "the chain will keep enrolling from announce order")
            else:
                self.logger.debug(
                    f"AFC bambu: bridge identities already stored ({n} units)")
        elif obj.get("evt") == "mmfix":
            # THE ADDRESS-MISMATCH REMEDY, AND ITS ONLY HONEST VERIFICATION.
            #
            # An op-05 reply carries the sender's UID, so a disagreement with
            # our address map is a fact off the wire, not an inference. Acting
            # on it broke the chain twice, so the action ships OFF and these
            # counters are how it gets judged:
            #
            #   remedy stops climbing  -> it worked
            #   remedy climbs at the cooldown rate -> it did not, and the map
            #                                         is still contradicted
            #   defer climbing         -> the bus was busy feeding, held back
            #
            # 1.18.0.0 was "verified" by watching the console for an event the
            # module does not handle and could never print. Numbers that can be
            # wrong, or nothing.
            with self._lock:
                self._last_mmfix = dict(obj)
            det, rem = obj.get("detect"), obj.get("remedy")
            if obj.get("on") and rem:
                self.logger.info(
                    f"AFC bambu: address-mismatch remedy has run {rem}x "
                    f"({det} disagreements seen, last at address "
                    f"{obj.get('addr')}, {obj.get('defer')} held back for a "
                    f"busy bus) -- if this keeps climbing the map is still "
                    f"wrong and the remedy is not fixing it")
            else:
                self.logger.debug(
                    f"AFC bambu: mmfix on={obj.get('on')} detect={det} "
                    f"remedy={rem} defer={obj.get('defer')} "
                    f"addr={obj.get('addr')} streak={obj.get('streak')} "
                    f"quiet={obj.get('quiet')}")
        elif obj.get("evt") == "ack":
            # Motion-command acknowledgements (select/feed/retract/assist/
            # stop/...). AFC.log via the AFC logger, not python logging.debug,
            # which klipper runs at INFO and therefore discards -- these are the
            # record of what the bridge was actually asked to do.
            # ══ THE ROUTINE ONES ARE LOG-ONLY. ══
            # assist/select/stop fire on the follower's own cadence -- during
            # a print "bridge ack assist (slot 1)" repeated every couple of
            # seconds for the whole job, which is the record of a mechanism
            # working correctly and not something an operator reads. The
            # motion acks that mark a real decision (feed, retract) stay on
            # the console. AFC.log keeps ALL of them either way, so the audit
            # trail of what the bridge was asked to do is unchanged.
            _ack_cmd = str(obj.get("cmd") or "")
            self.logger.debug(
                f"AFC bambu: bridge ack {_ack_cmd} (slot {obj.get('slot')})",
                only_debug=_ack_cmd in _ACK_ROUTINE)
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
            self._note_dry_cfg(text, obj.get("addr"))
            self._note_cap_measure(text, obj.get("addr"), now)
            # Stamp bridge-wide AND per-device. The address comes off the same
            # frame as the text (firmware capture_dbg reads it from bytes
            # [9:10]), so attribution costs nothing and is not a guess.
            _addr = obj.get("addr")
            _addr = int(_addr) if isinstance(_addr, int) and _addr else None
            if text and _RFID_INFLIGHT_RE.search(text):
                self._rfid_step_t = now
                if _addr:
                    self._rfid_step_by_addr[_addr] = now
            _read_re = (_RFID_READ_OK_HT_RE if _addr == 0x1800
                        else _RFID_READ_OK_RE)
            if text and _read_re.search(text):
                self._rfid_ok_t = now
                if _addr:
                    self._rfid_ok_by_addr[_addr] = now
            if text and _RFID_CYCLE_END_RE.search(text):
                self._rfid_end_t = now
                if _addr:
                    self._rfid_end_by_addr[_addr] = now
            # The unit refusing a chip it cannot authenticate. Stamped with the
            # other RFID markers -- above the dedupe, so a repeat still counts.
            if text and _RFID_FOREIGN_TAG_RE.search(text):
                self._rfid_foreign_t = now
                if _addr:
                    self._rfid_foreign_by_addr[_addr] = now
            # STRIP THE 10s HEARTBEAT BEFORE THE DEDUPE, NOT AFTER.
            #
            # "[DBG] ams time: now=42024044ms diff=10005ms" carries nothing and
            # its timestamp changes every time, so it was already being dropped
            # from the console further down -- but only AFTER the dedupe below
            # had stored it as _last_dbg. The unit bundles it into the SAME
            # frame as other narration ("[AMS_LINK]get_slot ... [DBG] ams time:
            # ..."), so every 10 s it broke the run of identical lines and the
            # next repeat printed as if it were new.
            #
            # That is the whole reason a sentence repeating 200 times a minute
            # reached the console 6 times a minute instead of once: 6 = 60/10,
            # the heartbeat period, not anything about the message.
            #
            # The narration FILE keeps the line verbatim -- it was written above,
            # before this -- so nothing is lost from the record.
            if text and "[DBG] ams time" in text:
                stripped = _DBG_AMSTIME_RE.sub("", text).strip()
                # NOTHING LEFT BUT THE FRAME'S JUNK BYTE -> NOTHING TO REPORT.
                #
                # The drain reply starts with one stray byte rendered as a
                # character, so a heartbeat-only frame strips down to something
                # like "," -- no bracket, no content. Publishing that put a bare
                # comma on the operator's console every 10 seconds.
                #
                # This is the bug the strip itself created: the console drop
                # further down tests for the "[DBG] ams time" substring, and
                # stripping the segment first meant that test could no longer
                # match. Fixed by making the decision HERE, once: keep the line
                # only if real narration rode along with the heartbeat.
                text = stripped if "[" in stripped else None
            # THE DEDUPE IS A CONSOLE CONCERN. THE PARSERS GET THE RAW LINE.
            #
            # Everything below used to read `text`, which the dedupe blanks on
            # an exact consecutive repeat -- so two byte-identical, back-to-back
            # completion lines for two genuinely separate moves would bump the
            # sequence once, and a waiter would sit through the second move and
            # time out. That reports a load that SUCCEEDED as a failure, which
            # is the expensive direction.
            #
            # The RFID stamps above were hoisted over the dedupe for exactly
            # this reason and the motion completions were left behind. They read
            # `raw` now, so suppressing a repeat can only ever cost a console
            # line. Double-counting one physical event is harmless here: every
            # consumer asks "has the sequence CHANGED since I started"
            # (_wait_move, _pull_seq_now), never "how many".
            raw = text
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
            if raw:
                # Buffer position, from whichever line carries it. Recorded
                # before the completion branches below so a line that is both
                # (a feed finish carrying buff_pos) contributes both.
                bp = _BUFF_POS_RE.search(raw)
                if bp:
                    with self._lock:
                        self._buff_pos = float(bp.group(1))
                rf = _BUFF_REFILL_RE.search(raw)
                if rf:
                    det = float(rf.group(3)) if rf.group(3) else None
                    with self._lock:
                        # The position AFTER recovery is the current one.
                        self._buff_pos = float(rf.group(2))
                        self._buff_refill = (float(rf.group(1)),
                                             float(rf.group(2)), det)
                # Motion completion.
                low = raw.lower()
                # INDEPENDENT OF THE FINISH CHAIN BELOW. A single narration
                # line can carry both a pull and a finish, and the chain's
                # if/elif ordering exists to make a finish WIN over an odom
                # reset in the HT's combined blob -- folding this into it broke
                # that rule and a test caught it. So it is asked separately.
                if _PULL_DONE_RE.search(raw):
                    with self._lock:
                        self._pull_seq += 1
                if _ASSIST_DONE_RE.search(raw):
                    with self._lock:
                        self._assist_seq += 1
                if _MOTION_FINISH_RE.search(raw):
                    # Judged BEFORE the lock: it reads tube_len(), which takes
                    # the same non-reentrant lock, and doing this inside the
                    # with-block deadlocks the reader thread outright.
                    ok = self._finish_succeeded(raw, low, obj.get("addr"))
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = ok
                        self._finish_text = raw
                    self._tray_gone = False
                elif _ODOM_RESET_RE.search(raw):
                    # A tray was engaged and its odometer zeroed: the feed
                    # arrived. This is the only completion an [AMS_DEV]-dialect
                    # unit gives -- it never says "feed finish" -- and it also
                    # re-arms the tray-gone edge below.
                    with self._lock:
                        self._finish_seq += 1
                        self._finish_ok = True
                        self._finish_text = raw
                    self._tray_gone = False
                elif _ODOM_NO_TRAY_RE.search(raw) and not self._tray_gone:
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
                        self._finish_text = raw
                # Motor current, when the AMS reports it. It clamps at its
                # limiter (~1.6A observed) against a ~0.07A median, so this is a
                # threshold signal rather than a proportional one.
                # Say the useful ones out loud, in English.
                try:
                    # addr identifies WHICH AMS narrated (0x0700 AMS 2 Pro,
                    # 0x1800 HT); absent on firmware older than 1.0.7.0.
                    self._narrate_human(raw, now, obj.get("addr"))
                except Exception:
                    pass          # a nicety must never break the reader thread
                mi = _BLDC_I_RE.search(raw)
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
                # THREE DIALECTS, THREE WAYS OF SAYING "I GAVE UP".
                #
                #   AMS 2 Pro   "feed finish -1, stall", "pull err, bdc stall"
                #   AMS HT      "TIMEOUT error N"
                #   AMS 1       says none of those -- it drops to
                #               state:6 / en:0,mode:7,idx:255
                #
                # The AMS 1 was long recorded as "genuinely silent about
                # faults". It is not; it just answers in STATE rather than
                # words, so a word-matching detector walked past it and a
                # jammed AMS 1 rode out the entire load window.
                #
                # VERIFIED AS A DISCRIMINATOR, not merely a signal: captured on
                # a failing lane15 load, and counted ZERO times across a lane15
                # load that genuinely reached the toolhead (18 kicks, state:4
                # and state:0 only).
                #
                # An earlier count said the opposite -- state:6 alongside
                # "successful" loads in hist.log and full.log. Those logs
                # predate the pin-read fix, so their successes were FALSE ones
                # on filament that never arrived: state:6 was there because the
                # unit HAD given up. The control was contaminated. Check that
                # the good loads are actually good before trusting a negative.
                if ("stall" in low or "finish -1" in low
                        or "timeout error" in low
                        or "state:6" in low or "en:0,mode:7" in low):
                    with self._lock:
                        self._fault_seq += 1
                        self._fault_text = raw.strip()
                # ...but DO track err_code's CURRENT VALUE, which is a
                # different question from "did a fault just happen". The note
                # above is about triggering: err_code cycles during healthy
                # operation, so an edge is not a fault. The LEVEL still answers
                # "is this unit in error right now", which is what a resume
                # guard needs -- and the unit states it plainly on both types:
                #   [AMS_LINK]err_code: 0 -> 23      HT declaring a stall
                #   [AMS_LINK]err_code: 18 -> 0      HT accepting the clear
                #   [AMS_LINK]err_code:0x00->0x80    AMS 2, hex form
                me = _ERR_CODE_RE.search(raw)
                if me:
                    try:
                        raw = me.group(2)
                        # 0x-prefixed is hex, bare is decimal -- "18" and
                        # "0x18" are different numbers and both occur.
                        val = int(raw, 16) if raw.lower().startswith("0x") \
                            else int(raw, 10)
                        with self._lock:
                            self._err_code = val
                            self._err_code_t = now
                    except ValueError:
                        pass
                # THE ONE THING THAT STILL USES THE DEDUPED TEXT, on purpose.
                #
                # Everything above reads `raw` so a suppressed repeat cannot
                # cost a parse. This is the operator's console, where a repeat
                # IS noise -- so it reads `text`, which the dedupe blanks, and
                # a run of identical lines re-emits once a minute with a count.
                #
                # The AMS's own narration -- its feed cycle
                # ("[AMS_SWITCH]BUFF,pos:0.09->0.74,det:12mm"), motor
                # transitions and scan steps. All of it goes to AFC.log via
                # the AFC logger, never bare python logging.debug, which
                # klipper runs at INFO and would discard outright.
                #
                # Pure bus chatter is additionally kept OFF the console
                # (only_debug=True) -- see _AMS_NOISE_RE. Narration that says
                # something stays on the console, where an operator with AFC's
                # debug flag on watches a load happen.
                if text:
                    # ══ THE CONSOLE IS A RATE-LIMITED CHANNEL, AND EXCEEDING
                    # IT SHUT KLIPPER DOWN. ══
                    #
                    # Narration that "says something" goes to the console by
                    # design, which is right until the unit says something
                    # thousands of times. Live: a scan whose selects were
                    # being acked by the wrong unit retried in a tight loop,
                    # each retry narrating --
                    #
                    #   [AMS_LINK]ams-0x00 select ack, req ams-0x01, mode:1
                    #   [AMS_DEV] STEP:set 0 tray_readid ...
                    #
                    # -- dozens of lines a second. Klipper's gcode responder
                    # is a pipe; filling it raises
                    # `BlockingIOError: [Errno 11]` inside _respond_raw, and
                    # the reactor stalls behind it. The clocksync went with
                    # it ("Resetting prediction variance ... diff=-921873102"
                    # on a 520 MHz mcu) and every mcu was shut down.
                    #
                    # So the console gets a ceiling. Overflow is NOT lost --
                    # it still goes to AFC.log, which is where a flood should
                    # be read anyway. A burst is allowed through so normal
                    # narration is unaffected; only a genuine storm is
                    # throttled, and it says so once.
                    # ══ JUDGE THE LINE WITHOUT THE HEARTBEAT RIDING ON IT.
                    # ══  The AMS bundles its 10-second "[DBG] ams time"
                    # liveness into whatever frame is going out. The dedupe
                    # above already strips it for that reason -- but this
                    # test ran on the RAW text, so any pure-chatter line
                    # that happened to carry a heartbeat matched no noise
                    # rule and went to the console anyway. Replaying the
                    # live log: 2,916 lines reached the console on that
                    # technicality alone, more than every other survivor
                    # combined. Strip it here too, then judge.
                    _quiet = _ams_is_noise(_DBG_AMSTIME_RE.sub("", text).strip())
                    if not _quiet:
                        _now = time.monotonic()
                        _win = getattr(self, "_narr_win", 0.0)
                        if _now - _win >= 1.0:
                            self._narr_win = _now
                            self._narr_n = 0
                            self._narr_said = False
                        self._narr_n = getattr(self, "_narr_n", 0) + 1
                        if self._narr_n > self.NARRATION_CONSOLE_MAX_PER_S:
                            _quiet = True            # log-only from here
                            if not getattr(self, "_narr_said", False):
                                self._narr_said = True
                                self.logger.info(
                                    "AFC bambu: the AMS is narrating faster "
                                    "than the console can take "
                                    f"(>{self.NARRATION_CONSOLE_MAX_PER_S}/s); "
                                    "the rest of this burst is in AFC.log")
                    self.logger.debug(f"AMS: {text}", only_debug=_quiet)
        elif obj.get("evt") == "rdinfo":
            with self._lock:
                self._last_rdinfo = dict(obj)
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
        elif obj.get("evt") == "loops":
            # Main-loop iterations + a timestamp. Everything the master can do
            # is bounded by this rate: the 21ms drive channel needs 48 passes a
            # second. Straight to the narration file, like tx.
            try:
                self._narrate_to_file(
                    f'{{"evt":"loops","n":{int(obj.get("n", 0))},'
                    f'"us":{int(obj.get("us", 0))}}}', None)
            except Exception:
                pass
        elif obj.get("evt") == "tx":
            # The frames WE transmit. Same shape as "sniff" on purpose, so the
            # capture tools read a TX log without changes -- which is the whole
            # point: a load can be DIFFED against a printer capture instead of
            # reasoned about. Straight to the narration file; this is a
            # diagnostic stream, not console output.
            try:
                # dir distinguishes what WE sent from what the AMS sent back.
                # Without it a transcript is half a conversation, and "the unit
                # never says X" cannot be told from "we never recorded X".
                _dir = str(obj.get("dir", "tx"))
                self._narrate_to_file(
                    f'{{"evt":"tx","dir":"{_dir}",'
                    f'"us":{int(obj.get("us", 0))},'
                    f'"n":{int(obj.get("n", 0))},'
                    f'"hex":"{obj.get("hex", "")}"}}', None)
            except Exception:
                pass
        elif obj.get("evt") == "sniff":
            # Passive bus-sniffer frame (real printer <-> AMS). Log every raw frame
            # verbatim so a capture can be pulled from AFC.log -- no dedup, each
            # frame matters. Only present when the firmware is in sniff mode.
            # File-only: a sniff runs at hundreds of frames a second and would
            # make the console unusable.
            self.logger.debug(f"SNIFF {obj.get('hex')}", only_debug=True)
        elif obj.get("evt") == "m3":
            # Short-motion poll diagnostics, on request ({"cmd":"m3"}). Held
            # whole for AFC_BAMBU_M3DIAG to print; the fields are the firmware's
            # own counters and mean nothing individually out of order.
            with self._lock:
                self._last_m3 = dict(obj)
        elif obj.get("evt") == "clsprobe":
            # Active class probe result: did the unit at this bus id answer on
            # device 0x1800 (HT) or not (boxed). Held whole for AFC_BAMBU_CLSPROBE.
            with self._lock:
                self._last_clsprobe = dict(obj)
        elif obj.get("evt") in ("rc", "rollcall"):
            # NB: this branch sits AFTER the not-in-_BRIDGE_EVENTS_KNOWN
            # catch-all above, so an event only reaches here if it is IN that
            # set. Adding a handler without adding the name silently routes the
            # event to the debug log instead -- which is exactly what happened
            # on the first try: firmware 1.0.59.0 was answering and the console
            # still said "no rc reply (firmware older than 1.0.59.0?)".
            # Roll-call state. "rollcall" is the toggle's echo, "rc" the
            # diagnostics; both land here so AFC_BAMBU_RC can print whichever came
            # last. Held whole -- these are the firmware's own counters.
            with self._lock:
                self._last_rc = dict(obj)
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
                # Announce-reply tag byte per discovered unit, in chain order.
                # Candidate signal for bus-based class detection -- observe
                # only, nothing acts on it. See the CLASS DETECTION notes.
                self._chain_tags = str(obj.get("tags") or "")
                try:
                    self._chain_capn = int(obj.get("capn") or 0)
                    self._chain_capdiag = int(obj.get("capdiag") or 0)
                except Exception:
                    self._chain_capn = 0
                    self._chain_capdiag = 0
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
        distinct from an empty/zero list meaning "reported, and nothing is
        set". The distinction matters: an unset address drops the narration
        log drain back to the captured 0x0700 pair, which never asks an AMS HT
        at 0x1800.

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
                if self._expect_reset:
                    # WE ASKED FOR THIS. A reset drops the USB CDC endpoint by
                    # definition, so the read failing is the command working,
                    # not a fault -- and shouting WARNING at an operator who
                    # just typed AFC_BAMBU_SAVEIDS reads like something broke.
                    self._expect_reset = False
                    self.logger.info(
                        "AFC bambu: bridge resetting as asked; reconnecting")
                else:
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
