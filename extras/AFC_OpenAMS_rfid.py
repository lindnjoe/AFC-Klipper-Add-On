# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# OpenAMS RFID integration for AFC.
#
# The OpenAMS mainboard has two RFID antenna boards (RFID A / RFID B), each an
# FM17580 reader — a Fudan MFRC522-class chip (13.56 MHz, ISO14443A, MIFARE
# Classic). Each is an 8-pin MFRC522 header (CS/SCK/MOSI/MISO/IRQ/RST + 3V3/GND).
# The two readers are on separate hardware SPI peripherals — RFID A on SPI1,
# RFID B on SPI2 — so each reader section gets its own `spi_bus:` (spi1 / spi2)
# plus a `cs_pin`. (The module also supports a shared bus + per-reader CS, like
# the ViViD, if a future board wires it that way.)
#
# NOTE: the stock OpenAMS mainboard firmware is built WITHOUT SPI, so this module
# only comes alive once the firmware is rebuilt with SPI enabled on the RFID pins
# (a CAN node with a Katapult bootloader — a build + reflash). Until then a real
# [AFC_OpenAMS_rfid <name>] section fails to build its SPI at config time, which
# is the expected tell. An i2c_hook transport is also supported for firmware
# patched by build_spi_patch.py, which detours the i2c_read path to run MFRC522
# ops over SPI.
#
# Like the ViViD path this is pure transport + wiring: it builds a Klipper
# MCU_SPI per reader, adapts it to the reg_read/reg_write contract, and hands it
# to the SAME transport-agnostic reader stack the ACE2/ViViD use (read_tag), so
# Bambu/Anycubic/Snapmaker/Creality tags all decode here and flow through the
# shared AFC_RFID Spoolman sync.
from __future__ import annotations

import traceback
from configparser import Error as error
import logging
from typing import TYPE_CHECKING, Any, Dict, List, Optional

# The shared transport-agnostic MFRC522 + MIFARE + multi-manufacturer
# decode stack. read_tag() only needs an object exposing reg_read/reg_write.
try: from extras.AFC_utils import ERROR_STR
except: raise error("Error when trying to import AFC_utils.ERROR_STR\n{trace}".format(trace=traceback.format_exc()))

try: from extras.AFC_rfid_readers import read_tag, Mfrc522, MifareClassic
except: raise error(ERROR_STR.format(import_lib="AFC_rfid_readers", trace=traceback.format_exc()))

try: from extras.AFC_RFID import (AFCUnitRFID,
    map_tag_to_slot_info,
    resolve_rfid_keys)
except: raise error(ERROR_STR.format(import_lib="AFC_RFID", trace=traceback.format_exc()))

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand

# MFRC522 SPI: mode 0. The address byte is (reg<<1); read sets the MSB (0x80).
# The FM17580 is register-compatible with the MFRC522, so this framing is shared
# with the ViViD/ACE2. 5 MHz is a safe default; the FM17580 SPI tops out at 10.
_MFRC522_SPI_MODE = 0
_MFRC522_SPI_SPEED = 5000000


class _OamsSpiRegLink:
    """
    Adapt a Klipper ``MCU_SPI`` to the reg_read/reg_write reader contract the
    shared Mfrc522/MifareClassic/read_tag stack expects.

    MFRC522 SPI framing: write = ``[(reg<<1), val]``; read = ``[0x80|(reg<<1),
    0x00]`` with the value in the second returned byte.

    :param spi: Klipper ``MCU_SPI`` for this reader.
    """

    def __init__(self, spi: Any) -> None:
        self.spi = spi

    def reg_read(self, reg: int) -> int:
        """
        Read one MFRC522 register over SPI (address byte 0x80|(reg<<1)).

        :param reg: MFRC522 register address.
        :return int: The register value.
        """
        addr = 0x80 | ((reg << 1) & 0x7E)
        params = self.spi.spi_transfer([addr, 0x00])
        resp = bytearray(params['response'])
        return resp[1] if len(resp) > 1 else 0

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one MFRC522 register over SPI (address byte (reg<<1)).

        :param reg: MFRC522 register address.
        :param val: Byte value to write.
        """
        self.spi.spi_send([(reg << 1) & 0x7E, val & 0xFF])

    def reader_power(self, on: bool) -> None:
        """
        No-op RF-field toggle (the OpenAMS readers have no coil-enable line).

        The field is driven through the MFRC522 TxControlReg by the Mfrc522 class.

        :param on: Requested power state (ignored).
        """
        return None


# Magic reg[] prefix understood by the injected firmware shim (build_spi_patch.py).
_HOOK_MAGIC0, _HOOK_MAGIC1 = 0x52, 0x46      # 'R','F'
_HOOK_OP_READ, _HOOK_OP_WRITE = 0x52, 0x57   # 'R','W'


class _OamsHookedI2cLink:
    """
    reg_read/reg_write for OpenAMS firmware patched by build_spi_patch.py.

    That firmware has no native SPI; instead the patch detours the i2c read path
    so an ``i2c_read`` to a magic device runs an MFRC522 op over SPI and returns
    the byte through the normal ``i2c_read_response``. This link speaks that
    protocol over a Klipper ``MCU_I2C``:

        read  reg -> i2c_read([0x52,0x46,'R',which,reg], 1) -> response[0]
        write reg -> i2c_read([0x52,0x46,'W',which,reg,val], 1)  (ack ignored)

    ``which`` is the reader index (0 = RFID A / SPI1, 1 = RFID B / SPI2). Pairs
    1:1 with the firmware patch; on stock firmware use the SPI link instead.

    :param i2c: Klipper ``MCU_I2C`` for the hooked read path.
    :param which: Reader index (0 = RFID A / SPI1, 1 = RFID B / SPI2).
    """

    def __init__(self, i2c: Any, which: int) -> None:
        self.i2c = i2c
        self.which = which & 0xFF

    def reg_read(self, reg: int) -> int:
        """
        Read one MFRC522 register through the hooked i2c_read path.

        :param reg: MFRC522 register address.
        :return int: The register value.
        """
        seq = [_HOOK_MAGIC0, _HOOK_MAGIC1, _HOOK_OP_READ, self.which,
               reg & 0xFF]
        params = self.i2c.i2c_read(seq, 1)
        resp = bytearray(params['response'])
        return resp[0] if resp else 0

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one MFRC522 register through the hooked i2c_read path.

        :param reg: MFRC522 register address.
        :param val: Byte value to write.
        """
        seq = [_HOOK_MAGIC0, _HOOK_MAGIC1, _HOOK_OP_WRITE, self.which,
               reg & 0xFF, val & 0xFF]
        # Still an i2c_read (read_len 1) so it flows through the hooked read path;
        # the returned byte is a don't-care ack.
        self.i2c.i2c_read(seq, 1)

    def reader_power(self, on: bool) -> None:
        """
        No-op RF-field toggle (no coil-enable line on the OpenAMS readers).

        :param on: Requested power state (ignored).
        """
        return None


class AFC_OpenAMS_rfid_reader:
    """
    One FM17580 reader on the OpenAMS (RFID A or RFID B), wrapping a Klipper
    ``MCU_SPI``.

    Configured as ``[AFC_OpenAMS_rfid <name>]`` with ``cs_pin`` (+ ``spi_bus`` or
    software-SPI pins), an optional ``reset_pin``, and the ``slots`` it serves.
    Both readers share one SPI bus and differ only by their CS pin.

    :param config: Klipper config wrapper for the named reader section.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        self.printer = config.get_printer()
        # Section is "AFC_OpenAMS_rfid <name>"; keep the trailing name.
        self.name = config.get_name().split()[-1]
        self.logger = logging.getLogger("AFC_OpenAMS_rfid")
        # Build the SPI at config time (registers the config commands). `bus` is
        # a Klipper core module (klippy/bus.py), imported here so this module
        # still loads without it (unit tests) — only a real reader needs it.
        try:
            from . import bus            # hardware: extras.bus
        except (ImportError, ValueError):
            import bus                   # top-level import fallback (tests)
        # transport: "spi" (stock firmware with native SPI, default) or
        # "i2c_hook" (firmware patched by oams_rfid/firmware/build_spi_patch.py,
        # which exposes the readers through a hooked i2c_read).
        transport = config.get("transport", "spi").strip().lower()
        self.spi = None
        if transport == "i2c_hook":
            i2c = bus.MCU_I2C_from_config(
                config, default_addr=config.getint("i2c_address", 0x7F),
                default_speed=100000)
            self.link = _OamsHookedI2cLink(i2c, config.getint("reader_index"))
        elif transport == "spi":
            self.spi = bus.MCU_SPI_from_config(
                config, _MFRC522_SPI_MODE, pin_option="cs_pin",
                default_speed=_MFRC522_SPI_SPEED, cs_active_high=False)
            self.link = _OamsSpiRegLink(self.spi)
        else:
            raise config.error(
                "AFC_OpenAMS_rfid %s: unknown transport %r (spi | i2c_hook)"
                % (config.get_name().split()[-1], transport))
        # Optional hardware reset / power-down line (MFRC522 NRSTPD). Held HIGH
        # to keep the chip enabled; the Mfrc522 class does a soft reset on read.
        self._reset_pin = None
        reset_pin = config.get("reset_pin", None)
        if reset_pin:
            ppins = self.printer.lookup_object("pins")
            self._reset_pin = ppins.setup_pin("digital_out", reset_pin)
            self._reset_pin.setup_start_value(1, 1)     # enabled at boot, stays high
        # Physical slots this reader's antenna covers.
        self.slots: List[int] = []
        for s in (config.get("slots", "") or "").split(","):
            s = s.strip()
            if not s:
                continue
            try:
                self.slots.append(int(s))
            except ValueError:
                raise config.error(
                    "AFC_OpenAMS_rfid %s: bad slot number %r in 'slots'"
                    % (self.name, s))


class AFC_OpenAMS_rfid(AFCUnitRFID):
    """
    Coordinator for the OpenAMS RFID readers. Configured as
    ``[AFC_OpenAMS_rfid]``.

    Ties the per-reader ``[AFC_OpenAMS_rfid <name>]`` sections to lanes via a
    ``lane_slot_map`` and drives a read: pick the lane's slot, read that slot's
    reader, decode and apply to the lane through the shared AFC_RFID path.

    :param config: Klipper config wrapper for the bare ``[AFC_OpenAMS_rfid]``
        section.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.logger = logging.getLogger("AFC_OpenAMS_rfid")
        self.afc = None
        self.log_prefix = "OpenAMS RFID"       # AFCUnitRFID.apply_to_lane / Spoolman
        self.auto_create = config.getboolean("auto_spoolman_create", False)

        # Shared decode keys: a reader section may override, else fall back to the
        # printer-wide [AFC_rfid_keys].
        bk = config.get("bambu_master_key", None)
        ck = config.get("creality_key", None)
        cek = config.get("creality_encryption_key", None)
        self.bambu_master_key = bytes.fromhex(bk) if bk else None
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        self.bambu_master_key, self.creality_key, self.creality_encryption_key = \
            resolve_rfid_keys(self.printer, self.bambu_master_key,
                              self.creality_key, self.creality_encryption_key)
        # Make a missing key impossible to miss: without it every Bambu tag is
        # uid-only ("answers but won't decode") at every position, which looks
        # deceptively like an antenna/positioning problem.
        if self.bambu_master_key is None:
            self.logger.warning(
                "AFC_OpenAMS_rfid: no bambu_master_key resolved (section "
                "option or [AFC_rfid_keys]) — Bambu tags will not decode")

        # lane -> physical slot (e.g. "lane4:0, lane5:1, ...").
        self._lane_slot: Dict[str, int] = {}
        for pair in (config.get("lane_slot_map", "") or "").split(","):
            pair = pair.strip()
            if not pair:
                continue
            try:
                lane, slot = pair.split(":")
                self._lane_slot[lane.strip()] = int(slot)
            except ValueError:
                raise config.error(
                    "AFC_OpenAMS_rfid: 'lane_slot_map' entries must be "
                    "'lane:slot' — bad entry %r" % pair)

        self._slot_reader: Dict[int, AFC_OpenAMS_rfid_reader] = {}
        self._last: Dict[int, dict] = {}       # slot -> last raw tag
        # Slots already reported as having no reader. A scan polls read_slot
        # many times a second, so an unconfigured slot would repeat the same
        # line for the whole window; it is a config fact, so say it once.
        self._no_reader_warned: set[int] = set()

        self.gcode.register_command(
            "OAMS_RFID_READ", self.cmd_OAMS_RFID_READ,
            desc="Read an OpenAMS RFID tag: OAMS_RFID_READ LANE=<name> | SLOT=<n>")
        self.printer.register_event_handler("klippy:connect", self._on_connect)

    def _on_connect(self) -> None:
        """
        Resolve the AFC object and index each reader by the slots it serves.

        Indexing by physical slot lets a lane's slot map straight to its reader.
        """
        self.afc = self.printer.lookup_object("AFC", None)
        for name, obj in self.printer.lookup_objects():
            if name.startswith("AFC_OpenAMS_rfid ") and isinstance(
                    obj, AFC_OpenAMS_rfid_reader):
                for slot in obj.slots:
                    if slot in self._slot_reader:
                        self.logger.warning(
                            "AFC_OpenAMS_rfid: slot %d served by more than one "
                            "reader (%s and %s)", slot,
                            self._slot_reader[slot].name, obj.name)
                    self._slot_reader[slot] = obj
        if not self._slot_reader:
            self.logger.warning(
                "AFC_OpenAMS_rfid: no [AFC_OpenAMS_rfid <name>] reader sections "
                "found — RFID reads will no-op")

    def _get_slot(self, lane_name: str) -> Optional[int]:
        """
        Return the physical slot a lane maps to, or None if unmapped.

        :param lane_name: AFC lane name.
        :return int: The mapped physical slot, or None if unmapped.
        """
        return self._lane_slot.get(lane_name)

    def read_slot(self, slot: int) -> Optional[dict]:
        """
        Read the tag on a physical slot via its reader; return the raw tag
        dict (uid + decoded filament), or None.

        The tag is only reliably in range while the spool is spinning during a
        feed, so callers read repeatedly across a feed (stage-read integration).

        :param slot: Physical OpenAMS slot index.
        :return dict: Raw read_tag() result, or None if no reader / no tag.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            if slot not in self._no_reader_warned:
                self._no_reader_warned.add(slot)
                self.logger.warning(
                    "AFC_OpenAMS_rfid: no reader configured for slot %d", slot)
            return None
        tag = read_tag(
            reader.link, bambu_master_key=self.bambu_master_key,
            creality_key=self.creality_key,
            creality_encryption_key=self.creality_encryption_key)
        if tag:
            self._last[slot] = tag
        return tag

    def scan_slot_uids(self, slot: int) -> List[str]:
        """
        Enumerate every tag UID currently in a slot's reader field.

        Used before a scan feed to take the sister-tag baseline: on the shared
        antenna a seated neighbour spool's tag may already be in range, and
        only a NEW uid appearing during the feed belongs to the moving spool.
        Each found tag is HALTed so the next request reaches the one behind it.

        :param slot: Physical OpenAMS slot index.
        :return list: uid hex strings for every tag seen (empty field = []).
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return []
        seen: List[Any] = []
        try:
            MifareClassic(Mfrc522(reader.link)).activate(
                is_excluded=lambda h: True, seen=seen)
        except Exception as e:
            self.logger.debug("scan_slot_uids failed on slot %d: %s", slot, e)
        return [u for (u, _sak, _ex) in seen]

    def detect_slot_tag(self, slot: int, exclude: Any) -> Optional[str]:
        """
        Light presence probe: REQA/anticollision/select only — no auth, no
        block reads — fast enough to poll while the spool is turning. Sister
        tags (uids in ``exclude``) are HALTed per the shared-reader protocol so
        the moving spool's own tag can answer.

        :param slot: Physical OpenAMS slot index.
        :param exclude: container of uid hex strings to ignore (sisters).
        :return str: a NEW tag's uid hex, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        try:
            uid, _sak = MifareClassic(Mfrc522(reader.link)).activate(
                is_excluded=lambda h: h in exclude)
        except Exception as e:
            self.logger.debug("detect_slot_tag failed on slot %d: %s", slot, e)
            return None
        return uid.hex() if uid is not None else None

    def read_slot_excluding(self, slot: int, exclude: Any) -> Optional[dict]:
        """
        Full tag read on a slot with sister uids excluded (HALTed), so the
        stationary read after a scan-feed detection can't decode the
        neighbour's seated tag by mistake.

        :param slot: Physical OpenAMS slot index.
        :param exclude: container of uid hex strings to ignore (sisters).
        :return dict: Raw read_tag() result, or None.
        """
        reader = self._slot_reader.get(slot)
        if reader is None:
            return None
        tag = read_tag(
            reader.link, bambu_master_key=self.bambu_master_key,
            creality_key=self.creality_key,
            creality_encryption_key=self.creality_encryption_key,
            is_excluded=lambda h: h in exclude)
        if tag:
            self._last[slot] = tag
        return tag

    def read_lane(self, lane_name: str) -> Optional[dict]:
        """
        Read a lane's tag (by its mapped slot) and apply it. Returns slot_info
        or None.

        :param lane_name: AFC lane name.
        :return dict: The applied slot_info, or None if unmapped / no tag.
        """
        slot = self._get_slot(lane_name)
        if slot is None:
            self.logger.warning(
                "AFC_OpenAMS_rfid: lane %r has no slot (set lane_slot_map)",
                lane_name)
            return None
        tag = self.read_slot(slot)
        if not tag or not tag.get("filament"):
            # A tag may still have been SEEN (e.g. missing decode key): record
            # its UID/type so get_status shows what was detected.
            if tag:
                self.record_tag_read(lane_name, None, decoded=False,
                                     uid=tag.get("uid", "") or "",
                                     tag_type=tag.get("tag_type", "") or "")
            return None
        lane = None
        if self.afc is not None and hasattr(self.afc, "lanes"):
            lane = self.afc.lanes.get(lane_name)
        if lane is None:
            slot_info = self._map(tag)
            self.record_tag_read(lane_name, slot_info)
            return slot_info
        return self.apply_to_lane(lane, tag)

    def _map(self, tag: dict) -> dict:
        """read_tag() result -> AFC slot_info (shared rich shape, AFC_RFID).

        :param tag: Raw read_tag() dict.
        :return dict: AFC slot_info (uid, brand, material, color_hex, ...).
        """
        return map_tag_to_slot_info(tag)

    # apply_to_lane() is inherited from AFCUnitRFID (shared AFC_RFID path); it
    # uses self._map, self.log_prefix ("OpenAMS RFID") and self.auto_create.

    def get_status(self, eventtime: Optional[float] = None) -> dict:
        """
        Report the configured reader slots and the lane->slot map.

        :param eventtime: Reactor event time (unused; kept for the status API).
        :return dict: The reader slots and lane_slot_map.
        """
        return {
            "slots": sorted(self._slot_reader.keys()),
            "lane_slot_map": dict(self._lane_slot),
            "last_reads": self.last_reads_status(),
        }

    def cmd_OAMS_RFID_READ(self, gcmd: GCodeCommand) -> None:
        """
        Read an OpenAMS RFID tag and apply it to the lane.

        Usage: ``OAMS_RFID_READ LANE=<name> | SLOT=<n>``
        Example: ``OAMS_RFID_READ LANE=lane4``
        """
        lane_name = gcmd.get("LANE", None)
        slot = gcmd.get_int("SLOT", None)
        if lane_name is None and slot is None:
            raise gcmd.error("OAMS_RFID_READ requires LANE= or SLOT=")
        if lane_name is not None:
            si = self.read_lane(lane_name)
            if si is None:
                hint = self.undecoded_hint(lane_name)
                gcmd.respond_info(
                    f"OpenAMS RFID: no tag decoded on {lane_name}{hint}")
            else:
                gcmd.respond_info(
                    "OpenAMS RFID: %s -> %s %s" % (
                        lane_name, si.get("brand", ""), si.get("material", "")))
            return
        tag = self.read_slot(slot)
        gcmd.respond_info(
            "OpenAMS RFID: slot %d -> %s" % (slot, self._map(tag) if tag else None))


def load_config(config: ConfigWrapper) -> AFC_OpenAMS_rfid:
    """
    Klipper entry point for the bare ``[AFC_OpenAMS_rfid]`` coordinator.

    :param config: Klipper config wrapper for the section.
    :return AFC_OpenAMS_rfid: The coordinator object.
    """
    return AFC_OpenAMS_rfid(config)


def load_config_prefix(config: ConfigWrapper) -> AFC_OpenAMS_rfid_reader:
    """
    Klipper entry point for a named ``[AFC_OpenAMS_rfid <name>]`` reader.

    :param config: Klipper config wrapper for the named reader section.
    :return AFC_OpenAMS_rfid_reader: The reader object.
    """
    return AFC_OpenAMS_rfid_reader(config)
