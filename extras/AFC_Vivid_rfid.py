# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# BigTreeTech MMS / ViViD RFID integration for AFC.
#
# The ViViD carries two MFRC522 (MIFARE Classic 1K, 13.56 MHz) readers on a
# shared Klipper SPI bus, distinguished only by their CS pin — each reader serves
# a PAIR of slots. Unlike the ACE 2 Pro there is no coil-enable / antenna-mux
# GPIO: the selector stepper mechanically positions the chosen slot's filament
# tip in front of its reader's fixed antenna, so a read targets whichever slot is
# currently selected. The RF field is toggled per-read via the MFRC522
# TxControlReg (handled by the shared Mfrc522 class).
#
# This module is pure transport + wiring: it builds a Klipper MCU_SPI per reader,
# adapts it to the reg_read/reg_write contract, and hands it to the SAME
# transport-agnostic reader stack the ACE2 uses (read_tag), so Bambu, Anycubic,
# Snapmaker, Creality AND BigTreeTech's own "BQ Tech" tags all decode here. The
# decoded filament flows through the shared AFC_RFID Spoolman sync (one tag = one
# spool, dual-colour, offline cache) exactly like the ACE2/U1 paths.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import traceback
from configparser import Error as error
import logging
from typing import Any, Callable, Dict, List, Optional, TYPE_CHECKING

# NB: Klipper's core `bus` module (sibling of `extras/`, not inside it) is
# imported lazily inside AFC_Vivid_rfid_reader.__init__ — mirroring how AFC.py
# defers `import mcu` — so this module imports without the full Klipper tree
# (e.g. in unit tests, where no SPI hardware is built).

# The shared transport-agnostic MFRC522 + MIFARE + multi-manufacturer decode
# stack. read_tag() only needs an object exposing reg_read(reg)/reg_write(reg,val).
# Mfrc522/MifareClassic give a FAST UID-only detect (activate) for the stage poll.
try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_rfid_readers import read_tag, Mfrc522, MifareClassic
except: raise error(ERROR_STR.format(import_lib="AFC_rfid_readers", trace=traceback.format_exc()))

try: from extras.AFC_RFID import (format_tag_summary, AFCUnitRFID,
    map_tag_to_slot_info,
    resolve_rfid_keys)
except: raise error(ERROR_STR.format(import_lib="AFC_RFID", trace=traceback.format_exc()))

try: from extras.AFC_lane import SpeedMode, AssistActive, MoveDirection   # move enums
except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))
if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC_lane import AFCLane

# MFRC522 SPI: mode 0, and the address byte is (reg << 1) with the MSB set for a
# read — BTT's driver uses 0x80 as that MSB (bit0 of the shifted byte is always 0
# so the two conventions coincide). 5 MHz matches the ViViD firmware default.
_MFRC522_SPI_MODE = 0
_MFRC522_SPI_SPEED = 5000000


class _VividSpiRegLink:
    """Adapt a Klipper ``MCU_SPI`` to the reg_read/reg_write reader contract that
    the shared Mfrc522/MifareClassic/read_tag stack expects.

    MFRC522 SPI framing: write = ``[(reg<<1), val]``; read = ``[0x80|(reg<<1),
    0x00]`` with the value in the second returned byte.
    """

    def __init__(self, spi: Any) -> None:
        """
        Store the Klipper ``MCU_SPI`` used for MFRC522 register access.

        :param spi: The Klipper ``MCU_SPI`` transport for this reader.
        """
        self.spi = spi

    def reg_read(self, reg: int) -> int:
        """
        Read one MFRC522 register over SPI (address byte 0x80|(reg<<1)).

        :param reg: The MFRC522 register index to read.
        :return int: The register value.
        """
        addr = 0x80 | ((reg << 1) & 0x7E)
        params = self.spi.spi_transfer([addr, 0x00])
        resp = bytearray(params['response'])
        return resp[1] if len(resp) > 1 else 0

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one MFRC522 register over SPI (address byte (reg<<1)).

        :param reg: The MFRC522 register index to write.
        :param val: The byte value to store.
        """
        self.spi.spi_send([(reg << 1) & 0x7E, val & 0xFF])

    def reader_power(self, on: bool) -> None:
        """
        No dedicated reader-power line on the ViViD — the RF field is driven
        through the MFRC522 TxControlReg by the Mfrc522 class. No-op here.

        :param on: Requested power state (ignored).
        """
        return None


class AFC_Vivid_rfid_reader:
    """One MFRC522 reader on the ViViD, wrapping a Klipper ``MCU_SPI``.

    Configured as ``[AFC_Vivid_rfid <name>]`` with ``cs_pin`` (+ ``spi_bus`` or
    software-SPI pins) and the ``slots`` it serves. Two readers share one SPI bus
    and are told apart only by their CS pin.
    """

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Build the MFRC522 SPI transport and parse the slots this reader serves.

        :param config: Klipper config wrapper for the named reader section.
        """
        self.printer = config.get_printer()
        # Section is "AFC_Vivid_rfid <name>"; keep the trailing name.
        self.name = config.get_name().split()[-1]
        self.logger = logging.getLogger("AFC_Vivid_rfid")
        # Build the SPI at config time (registers the config commands). mode 0,
        # active-low CS, 5 MHz — matching the ViViD firmware. `bus` is a Klipper
        # core module (klippy/bus.py), imported here so the module still loads
        # without it (tests) — only a real reader section needs it.
        try:
            from . import bus            # hardware: extras.bus
        except (ImportError, ValueError):
            import bus                   # top-level import fallback (tests)
        self.spi = bus.MCU_SPI_from_config(
            config, _MFRC522_SPI_MODE, pin_option="cs_pin",
            default_speed=_MFRC522_SPI_SPEED, cs_active_high=False)
        self.link = _VividSpiRegLink(self.spi)
        # Physical slots this reader covers (the selector positions one at a time).
        self.slots: List[int] = []
        for s in (config.get("slots", "") or "").split(","):
            s = s.strip()
            if not s:
                continue
            try:
                self.slots.append(int(s))
            except ValueError:
                raise config.error(
                    "AFC_Vivid_rfid %s: bad slot number %r in 'slots'"
                    % (self.name, s))


class AFC_Vivid_rfid(AFCUnitRFID):
    """Coordinator for the ViViD RFID readers. Configured as ``[AFC_Vivid_rfid]``.

    Maps AFC lanes to physical slots, resolves each slot to its reader, and reads
    a lane's tag through the shared multi-manufacturer ``read_tag`` stack, syncing
    the result to the lane + Spoolman.

    A spool's tag is only within range of the reader's fixed antenna WHILE THE
    SPOOL SPINS during a feed, and each reader's antenna sees BOTH of its two
    slots at once. So the read rides along on the unit's NORMAL load feed: on
    afc_vivid:stage_read_begin/end this module polls the reader on a reactor timer
    (no chunking — the feed motion is untouched, so the load position is
    unaffected), with a shared-antenna sibling dedup, and the first confirmed read
    wins. ``VIVID_RFID_READ`` also does an on-demand read (best with the spool
    spinning).
    """

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Parse the coordinator config, build the lane/slot maps and register the
        stage-read event handlers and the ``VIVID_RFID_READ`` command.

        :param config: Klipper config wrapper for the ``[AFC_Vivid_rfid]`` section.
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.logger = logging.getLogger("AFC_Vivid_rfid")

        # Brand keys (optional): Bambu master key + Creality CFS keys enable those
        # decoders; Snapmaker/Anycubic/Elegoo/BQ-Tech need no config key.
        bmk = (config.get("bambu_master_key", "") or "").strip()
        self.bambu_master_key = bytes.fromhex(bmk) if bmk else None
        ck = (config.get("creality_key", "") or "").strip()
        cek = (config.get("creality_encryption_key", "") or "").strip()
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        self.auto_create = config.getboolean("auto_spoolman_create", False)
        self.log_prefix = "ViViD RFID"         # AFCUnitRFID.apply_to_lane / Spoolman

        # Stage-read: while the unit runs its NORMAL load feed (untouched, so the
        # load position is unaffected), poll the reader on a reactor timer as the
        # spool spins — smooth, no chunking. The first confirmed read wins.
        self.stage_read = config.getboolean("stage_read", True)
        # Seconds between reader polls during the feed. The tag is in the antenna's
        # arc only briefly per revolution, so poll often enough to catch a pass.
        self.stage_poll_interval = config.getfloat(
            "stage_poll_interval", 0.15, minval=0.02)
        # Require the SAME decoded UID on this many CONSECUTIVE reads before
        # accepting it — a stray/partial read (or a neighbour tag flicking into
        # the shared antenna) can't then match/create the wrong spool. Confirmed
        # in-place while the feed is aborted (spool parked, tag held in range).
        self.stage_confirm_reads = config.getint(
            "stage_confirm_reads", 2, minval=1)
        # Max feed-aborts before giving up the read — bounds how many times a
        # stubborn (undecodable) tag can abort+re-issue the load feed.
        self.stage_max_aborts = config.getint("stage_max_aborts", 3, minval=1)
        # Sister-retract recovery (secondary — only when the HALT dedup fails to
        # let the active tag read past a shared-antenna sibling). Mirrors the
        # ACE2: retract the idle sibling off the antenna, re-read, then re-feed
        # the same distance to restore it exactly. auto_tag_adjust_dist default
        # matches the ACE2's 75 mm.
        self.auto_tag_adjust = config.getboolean("auto_tag_adjust", True)
        self.auto_tag_adjust_dist = config.getfloat(
            "auto_tag_adjust_dist", 75.0, minval=1.0)

        # lane_slot_map: "lane0:0, lane1:1, lane2:2, lane3:3" -> {lane: slot}
        self._lane_slot: Dict[str, int] = {}
        for pair in (config.get("lane_slot_map", "") or "").split(","):
            pair = pair.strip()
            if not pair:
                continue
            name, sep, slot = pair.partition(":")
            if not sep:
                raise config.error(
                    "AFC_Vivid_rfid: 'lane_slot_map' entries must be "
                    "'lane:slot', got %r" % pair)
            try:
                self._lane_slot[name.strip()] = int(slot.strip())
            except ValueError:
                raise config.error(
                    "AFC_Vivid_rfid: bad slot number in %r" % pair)

        self.afc: Any = None
        self._slot_reader: Dict[int, AFC_Vivid_rfid_reader] = {}
        self._slot_lane: Dict[int, str] = {}           # slot -> lane (reverse map)
        self._last: Dict[int, dict] = {}               # slot -> last slot_info
        # Last decoded UID per physical slot — used to halt a shared-antenna
        # sibling's parked tag so a read returns THIS slot's own tag.
        self._slot_uid: Dict[int, str] = {}
        self._probe: Optional[dict] = None             # active stage-read state
        self._poll_timer: Any = None                   # reactor timer during feed

        self.printer.register_event_handler("klippy:ready", self._on_ready)
        self.printer.register_event_handler(
            "afc_vivid:stage_read_begin", self._stage_read_begin)
        self.printer.register_event_handler(
            "afc_vivid:stage_read_end", self._stage_read_end)
        self.gcode.register_command(
            "VIVID_RFID_READ", self.cmd_VIVID_RFID_READ,
            desc="Read the ViViD RFID tag for a lane (LANE=) or slot (SLOT=) and "
                 "apply it to the lane / Spoolman")

    # ── wiring ──────────────────────
    def _on_ready(self) -> None:
        """
        Resolve the AFC object and shared brand keys, then discover every reader
        section and index it by physical slot (klippy:ready handler).
        """
        self.afc = self.printer.lookup_object("AFC", None)
        # Fall back to the shared [AFC_rfid_keys] section for any brand key this
        # section didn't set — configure the keys once, use them everywhere.
        if resolve_rfid_keys is not None:
            (self.bambu_master_key, self.creality_key,
             self.creality_encryption_key) = resolve_rfid_keys(
                self.printer, self.bambu_master_key, self.creality_key,
                self.creality_encryption_key)
        # Discover every [AFC_Vivid_rfid <name>] reader and index by physical slot.
        self._slot_reader = {}
        for name, obj in self.printer.lookup_objects():
            if name.startswith("AFC_Vivid_rfid ") and isinstance(
                    obj, AFC_Vivid_rfid_reader):
                for slot in obj.slots:
                    if slot in self._slot_reader:
                        self.logger.warning(
                            "AFC_Vivid_rfid: slot %d served by more than one "
                            "reader; keeping %s", slot,
                            self._slot_reader[slot].name)
                        continue
                    self._slot_reader[slot] = obj
        # Reverse map: slot -> lane, for sibling-present checks during dedup.
        self._slot_lane = {slot: name for name, slot in self._lane_slot.items()}
        if not self._slot_reader:
            self.logger.warning(
                "AFC_Vivid_rfid: no [AFC_Vivid_rfid <name>] reader sections "
                "found — RFID reads disabled")
        else:
            self.logger.info(
                "AFC_Vivid_rfid: %d slot(s) mapped across %d reader(s)",
                len(self._slot_reader),
                len({id(r) for r in self._slot_reader.values()}))

    def _get_slot(self, lane_name: str) -> Optional[int]:
        """
        Map a lane name to its configured physical slot.

        :param lane_name: The AFC lane name.
        :return Optional[int]: The mapped slot, or None if the lane is unmapped.
        """
        return self._lane_slot.get(lane_name)

    def _sibling_slot(self, slot: int) -> Optional[int]:
        """
        The OTHER slot served by the same reader (shared antenna), or None.

        :param slot: The physical slot to find the sibling of.
        :return Optional[int]: The sibling slot, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        others = [s for s in reader.slots if s != slot]
        return others[0] if len(others) == 1 else None

    def _sibling_has_spool(self, sib: int) -> bool:
        """
        Whether the sibling slot still has filament present (so its last-known
        tag is worth halting). Unknown -> assume present (safer to dedup).

        :param sib: The sibling physical slot.
        :return bool: True if the sibling likely still holds a spool.
        """
        lane_name = self._slot_lane.get(sib)
        if not lane_name or self.afc is None or not hasattr(self.afc, "lanes"):
            return True
        lane = self.afc.lanes.get(lane_name)
        if lane is None:
            return True
        return bool(getattr(lane, "prep_state", True))

    def _sibling_excluder(self, slot: int) -> Optional[Callable[[str], bool]]:
        """
        A ``uid_hex -> bool`` predicate that HALTS the shared-antenna sibling's
        last-known tag, so a read for THIS slot returns this slot's own tag rather
        than the neighbour's (both spools sit in the one reader's antenna range).
        None when the sibling has no known tag, or its spool has been removed.

        :param slot: The physical slot being read.
        :return Optional[Callable[[str], bool]]: The excluder predicate, or None.
        """
        sib = self._sibling_slot(slot)
        if sib is None:
            return None
        sib_uid = self._slot_uid.get(sib)
        if not sib_uid or not self._sibling_has_spool(sib):
            return None
        sib_uid = sib_uid.lower()

        def _excluded(uid_hex: str) -> bool:
            return (uid_hex or "").lower() == sib_uid
        return _excluded

    def _excluder_with(
            self, slot: int, extra: Optional[str] = None
    ) -> Optional[Callable[[str], bool]]:
        """
        The sibling excluder plus an optional extra UID to halt (a parked
        sibling tag captured at the stage baseline whose UID we don't otherwise
        know). Either may be None.

        :param slot: The physical slot being read.
        :param extra: An extra UID hex to also halt, or None.
        :return Optional[Callable[[str], bool]]: The combined predicate, or None.
        """
        base = self._sibling_excluder(slot)
        if not extra:
            return base
        ex = extra.lower()

        def _c(uid_hex: str) -> bool:
            u = (uid_hex or "").lower()
            return u == ex or (base is not None and base(u))
        return _c

    def _parked_tag(self, slot: int) -> Optional[str]:
        """
        RAW tag probe (NO excluder): read whatever tag is on the antenna right
        now. Called before the spool spins, so anything read is a parked,
        interfering tag (the stationary sibling's).

        :param slot: The physical slot to probe.
        :return Optional[str]: The parked tag UID hex, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        try:
            uid, _sak = MifareClassic(Mfrc522(reader.link)).activate()
        except Exception:
            return None
        return uid.hex() if uid is not None else None

    # ── read ──────────────────────
    def read_slot(
            self, slot: int, extra_excluded: Optional[str] = None
    ) -> Optional[dict]:
        """
        Read the tag on a physical slot via its reader; return the raw tag dict
        (uid + decoded filament) or None.

        Two slots share one reader's antenna, so a read can pick up the neighbour
        slot's parked tag — the sibling excluder halts that known tag so this read
        returns THIS slot's own tag. On a successful read the slot's UID is
        remembered to feed the sibling's future dedup.

        :param slot: The physical slot to read.
        :param extra_excluded: An extra parked-sibling UID hex to halt, or None.
        :return Optional[dict]: The raw tag dict, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            self.logger.warning(
                "AFC_Vivid_rfid: no reader configured for slot %d", slot)
            return None
        tag = read_tag(
            reader.link, bambu_master_key=self.bambu_master_key,
            creality_key=self.creality_key,
            creality_encryption_key=self.creality_encryption_key,
            is_excluded=self._excluder_with(slot, extra_excluded))
        uid = (tag or {}).get("uid")
        if uid:
            self._slot_uid[slot] = uid
        return tag

    def _map(self, tag: dict) -> dict:
        """
        Convert a read_tag() result to AFC slot_info (shared rich shape, AFC_RFID).

        :param tag: The raw tag dict from read_tag().
        :return dict: The AFC slot_info dict.
        """
        return map_tag_to_slot_info(tag)

    # apply_to_lane() is inherited from AFCUnitRFID (shared AFC_RFID path); it
    # uses self._map, self.log_prefix ("ViViD RFID") and self.auto_create.

    def read_lane(self, lane_name: str) -> Optional[dict]:
        """
        Read a lane's tag (by its mapped slot) and apply it. The caller must have
        the lane's slot selected at the reader.

        :param lane_name: The AFC lane name to read.
        :return Optional[dict]: The applied slot_info, or None.
        """
        slot = self._get_slot(lane_name)
        if slot is None:
            self.logger.warning(
                "AFC_Vivid_rfid: lane %r has no slot (set lane_slot_map)",
                lane_name)
            return None
        tag = self.read_slot(slot)
        if not tag or not tag.get("filament"):
            # A tag may still have been SEEN (e.g. Bambu without a decode key):
            # record its UID/type so get_status shows what was detected.
            if tag:
                self.record_tag_read(lane_name, None, decoded=False,
                                     uid=tag.get("uid", "") or "",
                                     tag_type=tag.get("tag_type", "") or "")
            return None
        self._last[slot] = tag
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        if lane is None:
            slot_info = self._map(tag)
            self.record_tag_read(lane_name, slot_info)
            return slot_info
        return self.apply_to_lane(lane, tag)

    # ── stage-read (poll the reader while the unit's load feed spins the spool) ──
    def _stage_read_begin(self, lane: "AFCLane") -> None:
        """
        The ViViD unit's NORMAL load feed is starting for this lane — begin
        polling the reader concurrently on a reactor timer while the spool spins.
        We don't touch the feed, so the load position is unaffected; we just ride
        along and grab the tag on whichever pass sweeps it into the antenna.

        :param lane: The AFC lane whose load feed is starting.
        """
        self._cancel_poll()
        if not self.stage_read:
            return
        lane_name = getattr(lane, "name", None)
        slot = self._get_slot(lane_name) if lane_name else None
        if slot is None or slot not in self._slot_reader:
            return                                     # not a lane we read
        self._probe = {"slot": slot, "lane": lane_name, "uid": None,
                       "count": 0, "done": False, "sib_lane": None,
                       "sib_dist": 0.0, "baseline": None, "blocked_sib": None,
                       "read_ok": False}
        # Only touch the sibling if its tag is ACTUALLY on the shared antenna. The
        # spool hasn't spun yet, so probe the reader ONCE (raw, no excluder): any
        # tag read here is a parked, interfering tag (the stationary sibling's). If
        # nothing's there, the sibling isn't in range — leave it be. If a tag IS
        # parked: clear it — retract a movable idle sibling off the coil, or (if it
        # can't be moved) exclude that UID for the whole read. Dedup alone can't
        # catch a sibling with no recorded UID (staged in a prior session).
        try:
            sib = self._sibling_slot(slot)
            sib_name = self._slot_lane.get(sib) if sib is not None else None
            sib_lane = (self.afc.lanes.get(sib_name)
                        if sib_name and self.afc is not None
                        and hasattr(self.afc, "lanes") else None)
            if sib_lane is not None and getattr(sib_lane, "prep_state", False):
                parked = self._parked_tag(slot)
                if parked:
                    self.logger.info(
                        "ViViD RFID: tag %s parked on the shared reader for slot "
                        "%d — clearing the sibling before the read", parked, slot)
                    self._retract_sibling(self._probe)
                    if self._probe.get("sib_lane") is None:
                        # Couldn't move it -> exclude its UID for the whole read and
                        # try anyway; if the read comes up empty, tell the user to
                        # move it / set the id (see _stage_read_end).
                        self._probe["baseline"] = parked
                        self._probe["blocked_sib"] = sib_name
        except Exception as e:
            self.logger.warning("ViViD RFID: sibling pre-check failed: %s", e)
        try:
            self._poll_timer = self.reactor.register_timer(
                self._stage_poll,
                self.reactor.monotonic() + self.stage_poll_interval)
        except Exception as e:
            self.logger.warning("ViViD RFID: could not start stage poll: %s", e)
            self._restore_sibling(self._probe)
            self._probe = None

    def _stage_poll(self, eventtime: float) -> float:
        """
        Reactor-timer tick DURING the continuous load feed. First do a FAST UID
        detect (a few SPI ops) — cheap, so the concurrent poll doesn't stall the
        move. If a tag is in the antenna's arc, STOP the feed (fake the load
        sensor so the homing move halts in place — the spool parks with the tag
        held in range) and do the full multi-manufacturer read + confirm parked.

        :param eventtime: The reactor event time for this tick.
        :return float: The next wake time, or NEVER once done/torn down.
        """
        p = self._probe
        if p is None or p.get("done"):
            return self.reactor.NEVER
        try:
            uid = self._detect_uid(p["slot"])
        except Exception as e:
            self.logger.warning(
                "ViViD RFID: detect error on slot %d: %s", p["slot"], e)
            uid = None
        if not uid:
            return eventtime + self.stage_poll_interval   # nothing in range yet
        if uid == p.get("baseline"):
            # The stationary parked sibling tag (couldn't be moved) — never this
            # lane's; keep polling so our own tag sweeps in.
            return eventtime + self.stage_poll_interval

        # A tag is in range. Any interfering sibling was cleared up front (retracted
        # off the antenna, or excluded via baseline above), so this is THIS lane's
        # tag. Stop-on-detect: abort the feed so the spool parks with the tag in the
        # antenna, then read + confirm in place.
        self._abort_feed(p["lane"])
        p["aborts"] = p.get("aborts", 0) + 1
        tag = self._read_confirmed(p["slot"], p.get("baseline"))
        if tag:
            p["done"] = True
            p["read_ok"] = True
            self._apply_staged(p, tag)
            return self.reactor.NEVER
        # Detected but couldn't decode. Give up after a few aborts so a stubborn
        # read can't keep aborting (and re-issuing) the load feed forever.
        if p["aborts"] >= self.stage_max_aborts:
            p["done"] = True
            self.logger.info(
                "ViViD RFID: gave up reading %s after %d attempts",
                p["lane"], p["aborts"])
            return self.reactor.NEVER
        return eventtime + self.stage_poll_interval

    def _detect_uid(self, slot: int) -> Optional[str]:
        """
        FAST tag presence check: MFRC522 activate only (REQA/anticoll/select),
        no block reads/decode. Applies the shared-antenna sibling excluder so a
        halted neighbour tag doesn't count.

        :param slot: The physical slot to detect on.
        :return Optional[str]: The detected UID hex, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        mc = MifareClassic(Mfrc522(reader.link))
        uid, _sak = mc.activate(is_excluded=self._sibling_excluder(slot))
        return uid.hex() if uid is not None else None

    def _read_confirmed(
            self, slot: int, baseline: Optional[str] = None
    ) -> Optional[dict]:
        """
        Full read + in-place confirm: the feed is aborted, so the spool is parked
        with the tag in range — read it stage_confirm_reads times and require a
        consistent decoded UID. ``baseline`` (a parked sibling UID we couldn't
        move) is halted so the read returns this lane's own tag.

        :param slot: The physical slot to read.
        :param baseline: A parked-sibling UID hex to halt, or None.
        :return Optional[dict]: The confirmed tag dict, or None.
        """
        tag = None
        for _ in range(self.stage_confirm_reads):
            try:
                t = self.read_slot(slot, extra_excluded=baseline)
            except Exception as e:
                self.logger.warning(
                    "ViViD RFID: stage read error on slot %d: %s", slot, e)
                return None
            if not t or not t.get("filament"):
                return None
            if tag is not None and (t.get("uid") or "") != (tag.get("uid") or ""):
                return None                            # inconsistent — reject
            tag = t
        return tag

    def _abort_feed(self, lane_name: str) -> None:
        """
        Stop the in-progress load feed by FAKING the load-sensor trigger:
        force-complete the endstop's MCU trsync so the homing move halts in
        place. The REAL sensor is still untriggered, so the unit knows to
        re-issue the feed and finish the load. Best-effort + guarded: if the
        trsync can't be reached the feed simply runs to the real sensor (no
        read this pass) — the load is never broken.

        :param lane_name: The AFC lane whose load feed to abort.
        """
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        endstop_name = getattr(lane, "load_endstop_name", None) if lane else None
        if not endstop_name:
            return
        try:
            qe = self.printer.lookup_object("query_endstops", None)
            mcu_endstop = None
            for es, name in (getattr(qe, "endstops", []) or []):
                if name == endstop_name:
                    mcu_endstop = es
                    break
            if mcu_endstop is None:
                return
            dispatch = getattr(mcu_endstop, "_dispatch", None)
            trsyncs = getattr(dispatch, "_trsyncs", None) if dispatch else None
            if not trsyncs:
                return
            trsync = trsyncs[0]
            trsync._trsync_trigger_cmd.send(
                [trsync._oid, trsync.REASON_HOST_REQUEST])
        except Exception as e:
            self.logger.debug(
                "ViViD RFID: fake-trigger of %s not available (%s) — feeding to "
                "the real sensor instead", endstop_name, e)

    def _apply_staged(self, p: dict, tag: dict) -> None:
        """
        Cache and apply a staged read to its lane.

        :param p: The active probe state.
        :param tag: The confirmed tag dict to apply.
        """
        self._last[p["slot"]] = tag
        lane_obj = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane_obj = self.afc.lanes.get(p["lane"])
        try:
            if lane_obj is not None:
                self.apply_to_lane(lane_obj, tag)
        except Exception as e:
            self.logger.warning(
                "ViViD RFID: applying %s read failed: %s", p["lane"], e)

    def _retract_sibling(self, p: dict) -> None:
        """
        PROACTIVELY move the shared-antenna sibling off the reader before we read,
        so a strong sibling tag can't drown out the active lane's tag. Mirrors the
        ACE2's auto_tag_adjust (default 75 mm). Gated — only an IDLE, present
        sibling, never tool-loaded, never mid-print. Records the moved distance on
        ``p`` so _restore_sibling puts it back exactly.

        :param p: The active probe state; sib_lane/sib_dist are filled if moved.
        """
        if not self.auto_tag_adjust:
            return
        sib = self._sibling_slot(p["slot"])
        sib_lane_name = self._slot_lane.get(sib) if sib is not None else None
        sib_lane = None
        if sib_lane_name and self.afc is not None and hasattr(self.afc, "lanes"):
            sib_lane = self.afc.lanes.get(sib_lane_name)
        if sib_lane is None:
            return
        # Nothing to move if the sibling is empty; refuse to move a loaded or
        # printing sibling (moving loaded/printing filament is a hard no).
        if not getattr(sib_lane, "prep_state", False):
            return
        if getattr(sib_lane, "tool_loaded", False) or self._afc_is_printing():
            self.logger.info(
                "ViViD RFID: sibling %s is loaded/printing — not moving it; "
                "relying on the HALT dedup (nudge by hand if the read misses)",
                sib_lane_name)
            return
        dist = self.auto_tag_adjust_dist
        self.logger.info(
            "ViViD RFID: retracting sibling %s %.0fmm to clear the antenna "
            "before reading slot %d", sib_lane_name, dist, p["slot"])
        sib_lane.move_to(dist * MoveDirection.NEG, SpeedMode.SHORT,
                         assist_active=AssistActive.NO, use_homing=False)
        p["sib_lane"], p["sib_dist"] = sib_lane, dist

    def _restore_sibling(self, p: Optional[dict]) -> None:
        """
        Re-feed the sibling the exact distance we retracted it, back to its prior
        position. Failure to restore is loud — the sibling is now short.

        :param p: The active probe state, or None.
        """
        if not p:
            return
        sib_lane, dist = p.get("sib_lane"), p.get("sib_dist") or 0.0
        if sib_lane is None or dist <= 0.0:
            return
        p["sib_lane"], p["sib_dist"] = None, 0.0       # once
        try:
            sib_lane.move_to(dist * MoveDirection.POS, SpeedMode.SHORT,
                             assist_active=AssistActive.NO, use_homing=False)
        except Exception as e:
            self.logger.error(
                "ViViD RFID: FAILED to restore sibling by %.0fmm: %s", dist, e)

    def _afc_is_printing(self) -> bool:
        """
        Best-effort 'are we printing' guard — never move a sibling mid-print.

        :return bool: True if the printer reports it is printing.
        """
        try:
            ps = self.printer.lookup_object("print_stats", None)
            if ps is not None:
                st = ps.get_status(self.reactor.monotonic())
                return st.get("state") == "printing"
        except Exception:
            pass
        return False

    def _sister_blocked_hint(
            self, active_name: Optional[str], sib_name: Optional[str]
    ) -> None:
        """
        A stage read came up empty and a present sibling we couldn't move was
        parked on the shared reader — almost certainly blocking it. Tell the user
        to move that sister spool and re-read, or set the id by hand.

        :param active_name: The lane we were trying to read, or None.
        :param sib_name: The blocking sibling lane name, or None.
        """
        act = active_name or "this lane"
        self.gcode.respond_info(
            "ViViD RFID: couldn't read %s's RFID — lane %s's spool is on the "
            "shared reader blocking it. Manually move lane %s's spool and re-run "
            "VIVID_RFID_READ, or set %s's spool id by hand."
            % (act, sib_name, sib_name, act))

    def _stage_read_end(self, lane: "AFCLane") -> None:
        """
        The feed finished — stop polling and RESTORE the sibling to its exact
        prior position (if we moved it). If we never decoded a tag AND a present
        sibling we couldn't move was blocking the reader, tell the user to move it
        or set the id by hand; otherwise it's just a tagless/foreign spool.

        :param lane: The AFC lane whose load feed finished.
        """
        p = self._probe
        if p is not None and not p.get("read_ok"):
            self.logger.info(
                "ViViD RFID: no tag decoded on %s during staging", p.get("lane"))
            if p.get("blocked_sib"):
                self._sister_blocked_hint(p.get("lane"), p.get("blocked_sib"))
        self._restore_sibling(p)
        self._cancel_poll()

    def _cancel_poll(self) -> None:
        """
        Tear down the poll timer + probe state, restoring a still-retracted
        sibling first so it's never left short (defensive for aborted reads).
        """
        self._restore_sibling(self._probe)
        if self._poll_timer is not None:
            try:
                self.reactor.unregister_timer(self._poll_timer)
            except Exception:
                pass
            self._poll_timer = None
        self._probe = None

    # ── gcode ──────────────────────
    def cmd_VIVID_RFID_READ(self, gcmd: GCodeCommand) -> None:
        """
        Read a ViViD RFID tag for a lane or slot and apply it to the lane/Spoolman.

        Usage
        -----
        `VIVID_RFID_READ LANE=<name> | SLOT=<n>`

        Example
        -----
        ```
        VIVID_RFID_READ LANE=lane0
        ```
        """
        lane_name = gcmd.get("LANE", None)
        slot = gcmd.get_int("SLOT", None)
        if lane_name is None and slot is None:
            raise gcmd.error("VIVID_RFID_READ requires LANE= or SLOT=")
        if lane_name is not None:
            # read_lane -> apply_to_lane already prints the console read-out.
            si = self.read_lane(lane_name)
            if si is None:
                hint = self.undecoded_hint(lane_name)
                gcmd.respond_info(f"ViViD RFID: no tag decoded on {lane_name}{hint}")
            return
        tag = self.read_slot(slot)
        if not tag or not tag.get("filament"):
            hint = ""
            if tag and tag.get("uid"):
                hint = f" (saw tag UID {tag['uid']}"
                if tag.get("tag_type"):
                    hint += f", {tag['tag_type']}"
                hint += " — no decoder/key matched)"
            gcmd.respond_info(f"ViViD RFID: no tag decoded on slot {slot}{hint}")
            return
        self._respond_tag(gcmd, self._map(tag), "slot %d" % slot)

    def _respond_tag(self, gcmd: GCodeCommand, si: dict, where: str) -> None:
        """
        Echo a decoded tag to the console using the shared U1-style scan format
        (header + indented Name/Brand/Material/Color/temp lines), so every RFID
        surface reads the same. Colour is hex only ("#hex + #hex" for dual).

        :param gcmd: The gcode command to respond on.
        :param si: The AFC slot_info dict.
        :param where: A label for the source (lane or slot).
        """
        gcmd.respond_info(
            format_tag_summary(si, "ViViD RFID tag on %s:" % where))

    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        Report which slots have a reader and the lane->slot map, so the GUI /
        macros can see the ViViD RFID wiring.

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: The configured reader slots and the lane_slot_map.
        """
        return {
            "slots": sorted(self._slot_reader.keys()),
            "lane_slot_map": dict(self._lane_slot),
            "last_reads": self.last_reads_status(),
        }


def load_config(config: "ConfigWrapper") -> AFC_Vivid_rfid:
    """
    Klipper entry point for the bare ``[AFC_Vivid_rfid]`` section — the one
    coordinator that ties the per-reader sections to lanes.

    :param config: Klipper config wrapper for the section.
    :return AFC_Vivid_rfid: The coordinator object.
    """
    return AFC_Vivid_rfid(config)


def load_config_prefix(config: "ConfigWrapper") -> AFC_Vivid_rfid_reader:
    """
    Klipper entry point for a named ``[AFC_Vivid_rfid <name>]`` section — one
    physical MFRC522 reader serving a pair of slots.

    :param config: Klipper config wrapper for the named reader section.
    :return AFC_Vivid_rfid_reader: The reader object.
    """
    return AFC_Vivid_rfid_reader(config)
