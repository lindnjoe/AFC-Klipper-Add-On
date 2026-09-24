# AFCProject Automated Filament Changer
#
# Copyright (C) 2024-2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# BoxTurtle lane RFID: read a spool's tag by turning the spool.
#
# Two RC522-family readers share one Pico running the rfid_bridge firmware,
# one reader per pair of lanes:
#
#   [AFC_BoxTurtle_rfid]           coordinator -- serial port, keys, knobs
#   [AFC_BoxTurtle_rfid reader0]   bus: 0   lanes: lane8, lane9
#   [AFC_BoxTurtle_rfid reader1]   bus: 1   lanes: lane10, lane11
#
# THE PICO IS NOT A KLIPPER [mcu]. It is a plain USB-CDC device this module
# opens itself, because a Klipper MCU that is absent fails the whole printer
# at connect. Unplugged, wedged or mid-reflash, the readers report offline
# and everything else prints normally. A spool reader must never be able to
# take the printer down.
#
# The firmware is a dumb register wire -- "r<bus> <reg>" and "w<bus> <reg>
# <val>", hex, one op per line -- so the shared reader stack runs HOST side
# unchanged (AFC_rfid_readers.read_tag: MFRC522 + MIFARE + Bambu, Anycubic,
# Snapmaker, Creality, Elegoo and BTT decode) and results reach Spoolman
# through the shared AFC_RFID path. Bus 0 is GP4/GP5, bus 1 is GP6/GP7.
#
# A read is hundreds of register round-trips, about a second over USB, so
# reads run on a WORKER THREAD and post back through the reactor. The reactor
# never waits on the serial port.
#
# THE TAG IS ON THE SPOOL, at one angular position, so reading it means
# TURNING the spool: feed the lane while polling, stop the instant the tag
# answers, put the filament back. The read is the endstop; the distance is
# only a failsafe, and it is bounded in spool REVOLUTIONS because one
# revolution is what guarantees every angle passes the coil.
#
# Two lanes share each antenna, so a sister lane's parked tag can be read as
# this lane's. It is rolled clear before the sweep rather than filtered out
# afterwards -- see _clear_sibling for why filtering cannot work.
#
# This runs by itself on insert, from afc:lane_prep_loaded: after AFC has
# homed the filament to the load switch and before it feeds the hub, the one
# moment in the cycle with both a known tip position and an empty bowden to
# feed into. AFC_BT_RFID_READ and AFC_BT_RFID_STAGE do it on demand.
from __future__ import annotations
import math
import threading
import chelper
from typing import (TYPE_CHECKING, Any, Callable, Dict, List, Optional,
                    Tuple)

# pyserial is only needed when the bridge actually opens a port; the tests
# and any host without a reader Pico must import cleanly without it.
try:                                        # pragma: no cover - import shim
    import serial
except Exception:                           # pragma: no cover
    serial = None

try:                                        # pragma: no cover - import shim
    from extras.AFC_lane import AssistActive, MoveDirection, SpeedMode
except Exception:                           # pragma: no cover
    AssistActive = MoveDirection = SpeedMode = None

from extras.AFC_rfid_readers import read_tag
from extras.AFC_rfid_write import register_reader
from extras.AFC_RFID import (AFCUnitRFID,
                             map_tag_to_slot_info,
                             resolve_rfid_keys)

if TYPE_CHECKING:
    from configfile import ConfigWrapper

#: VersionReg -- the probe register. 0x91/0x92 = genuine MFRC522 v1/v2;
#: clones (FM17522 and friends) answer other non-0x00/0xFF values.
_VERSION_REG = 0x37

#: Per-op reply deadline. One register op is one short line each way; a
#: healthy bridge answers in a few ms, so a slow reply IS a failed reply.
_OP_TIMEOUT_S = 0.25

#: How often the connect timer retries an absent bridge.
_RETRY_S = 5.0


class _BridgeSerial:
    """The rfid_bridge Pico's USB-CDC port, opened and owned by this module.

    Fail-soft is the contract: every method returns a failure value instead
    of raising into Klipper, and a port error closes the port so the connect
    timer quietly retries. One lock serialises request/reply pairs -- reads
    run on worker threads and the probe runs on the reactor, and the protocol
    is strictly one outstanding op.
    """

    def __init__(self, port: str, logger: Any) -> None:
        """
        :param port: the /dev/serial/by-id path of the rfid_bridge Pico
        :param logger: module logger for connect/drop notes
        """
        self.port = port
        self.logger = logger
        self._ser: Optional[Any] = None
        self._lock = threading.Lock()

    def connected(self) -> bool:
        """
        :return bool: True while the port is open
        """
        return self._ser is not None

    def connect(self) -> bool:
        """
        Open the port if it is not already open. Never raises.

        :return bool: True when the port is open after the attempt
        """
        if serial is None:
            return False
        with self._lock:
            if self._ser is not None:
                return True
            try:
                self._ser = serial.Serial(
                    self.port, 115200, timeout=_OP_TIMEOUT_S,
                    write_timeout=_OP_TIMEOUT_S)
                self.logger.info(
                    f"BT RFID: bridge connected on {self.port}")
                return True
            except Exception:
                self._ser = None
                return False

    def _drop(self) -> None:
        """Close the port after an error; the connect timer takes it from
        here. Called with the lock held."""
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        if self._ser is not None:
            self.logger.warning(
                f"BT RFID: bridge dropped on {self.port} -- retrying in "
                f"the background")
        self._ser = None

    def request(self, line: str) -> Optional[str]:
        """
        One register op: send the line, return the "="-reply payload.

        The stream also carries the firmware's JSON events (hello/scan/chip);
        anything that is not an op reply is skipped. None means the op failed
        -- port absent, I2C NAK ("!"), or timeout -- and the caller treats
        the reader as not answering.

        :param line: the op, e.g. "r0 37" or "w1 2A 8D"
        :return: reply payload ("" for a write ack, hex for a read), or None
        """
        with self._lock:
            if self._ser is None:
                return None
            try:
                self._ser.reset_input_buffer()
                self._ser.write((line + "\n").encode())
                for _ in range(8):          # skip interleaved JSON events
                    resp = self._ser.readline().decode(errors="replace")
                    if not resp:
                        return None         # timeout
                    resp = resp.strip()
                    if resp.startswith("="):
                        return resp[1:]
                    if resp.startswith("!"):
                        return None
                return None
            except Exception:
                self._drop()
                return None


class _SerialRegLink:
    """reg_read/reg_write over the rfid_bridge -- the link contract the
    shared Mfrc522 driver expects, with the wire being a serial line."""

    def __init__(self, bridge: _BridgeSerial, bus: int) -> None:
        """
        :param bridge: the shared serial bridge
        :param bus: which I2C controller on the Pico (0 or 1)
        """
        self.bridge = bridge
        self.bus = bus

    def reg_read(self, reg: int) -> int:
        """
        Read one register through the bridge.

        :param reg: MFRC522 register index
        :return int: the register value
        """
        resp = self.bridge.request(f"r{self.bus} {reg & 0xFF:02X}")
        if not resp:
            error_str = f"rfid bridge bus {self.bus}: no answer reading " \
                        f"reg 0x{reg:02X}"
            raise OSError(error_str)
        return int(resp, 16)

    def reg_write(self, reg: int, val: int) -> None:
        """
        Write one register through the bridge.

        :param reg: MFRC522 register index
        :param val: value to write
        """
        resp = self.bridge.request(
            f"w{self.bus} {reg & 0xFF:02X} {val & 0xFF:02X}")
        if resp is None:
            error_str = f"rfid bridge bus {self.bus}: no answer writing " \
                        f"reg 0x{reg:02X}"
            raise OSError(error_str)


class AFC_BoxTurtle_rfid_reader:
    """One RC522 reader section: which bridge bus it lives on and the lanes
    its antenna covers. Configured as ``[AFC_BoxTurtle_rfid <name>]``."""

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Parse the bus number and the lanes this reader serves. No hardware
        is touched at config time -- the coordinator wires the link once the
        bridge connects.

        :param config: Klipper config wrapper for the named reader section
        """
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # AFC's logger from construction: load_object CONSTRUCTS AFC when this
        # section is reached first, the way AFC_lane/AFC_buffer/AFC_extruder
        # take it. self.afc stays None until klippy:ready deliberately -- it is
        # this module's READY MARKER and its guards read it that way.
        self.logger = self.printer.load_object(config, "AFC").logger
        self.bus = config.getint("bus", 0, minval=0, maxval=1)
        self.link: Optional[_SerialRegLink] = None
        self.lanes: List[str] = []
        for ln in (config.get("lanes", "") or "").split(","):
            ln = ln.strip()
            if ln:
                self.lanes.append(ln)
        if not self.lanes:
            raise config.error(
                "AFC_BoxTurtle_rfid %s: 'lanes' must name at least one lane"
                % (self.name,))
        # Filled by the coordinator's connect probe; None = offline.
        self.version: Optional[int] = None


class AFC_BoxTurtle_rfid(AFCUnitRFID):
    """Coordinator for the BoxTurtle RFID readers.

    Maps AFC lanes to readers (each reader's fixed antenna covers two adjacent
    lanes), reads a lane's tag through the shared multi-manufacturer
    ``read_tag`` stack, and applies the result to the lane + Spoolman.

    A shared antenna means the OTHER lane's at-rest tag can answer instead of
    the one being read: the partner lane's last-known UID is passed to
    ``read_tag`` as excluded, so the stack halts on the sibling and keeps
    hunting for the lane's own tag.
    """

    def __init__(self, config: "ConfigWrapper") -> None:
        """
        Parse the coordinator config and register the g-code commands. The
        serial bridge is opened by a background timer after ready -- config
        time touches no hardware, so a missing reader Pico can never fail
        the printer's startup.

        :param config: Klipper config wrapper for ``[AFC_BoxTurtle_rfid]``
        """
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        # AFC's logger from construction: load_object CONSTRUCTS AFC when this
        # section is reached first, the way AFC_lane/AFC_buffer/AFC_extruder
        # take it. self.afc stays None until klippy:ready deliberately -- it is
        # this module's READY MARKER and its guards read it that way.
        self.logger = self.printer.load_object(config, "AFC").logger
        self.afc: Any = None
        self.log_prefix = "BT RFID"

        self.serial_port = config.get("serial")
        self.bridge = _BridgeSerial(self.serial_port, self.logger)

        # Brand keys: the reader's own section wins, [AFC_rfid_keys] fills the
        # gaps -- so the keys configured once for the other readers serve this
        # one too.
        bmk = (config.get("bambu_master_key", "") or "").strip()
        ck = (config.get("creality_key", "") or "").strip()
        cek = (config.get("creality_encryption_key", "") or "").strip()
        self.bambu_master_key = bytes.fromhex(bmk) if bmk else None
        self.creality_key = bytes.fromhex(ck) if ck else None
        self.creality_encryption_key = bytes.fromhex(cek) if cek else None
        self.auto_create = config.getboolean("auto_spoolman_create", False)

        # Tag-homing defaults (all overridable per AFC_BT_RFID_STAGE call).
        #
        # The sweep is bounded in SPOOL REVOLUTIONS, not millimetres, because
        # that is the quantity that actually matters: a tag sits at one angular
        # position on the spool, so the reader only sees it once per turn. The
        # filament distance for a turn is the spool's circumference, which is
        # why a nearly-full 200mm spool costs ~630mm of filament per look.
        self.spool_diameter_mm = config.getfloat(
            "spool_diameter_mm", 200.0, above=0.0)
        self.tag_sweep_revs = config.getfloat(
            "tag_sweep_revs", 2.0, above=0.0)
        # Slow on purpose: the read is the endstop, and a fast sweep can carry
        # the tag through the coil's arc between two polls.
        # What limits this is DWELL, not the motor: the tag has to stay inside
        # the coil's field long enough for one read to complete, and a read is
        # hundreds of serial round-trips. The spool's surface speed equals the
        # feed speed, so this is directly how fast the tag crosses the antenna
        # -- roughly a third of a second in a ~20mm field at 60mm/s.
        #
        # 100 is measured, not derived: 20, 40, 60 and 100 all read the same
        # tag on the same spool.
        #
        # Know the failure mode before pushing it further. A tag crossed too
        # quickly is not read slowly, it is MISSED, and a miss costs another
        # full revolution of searching. So the symptom is not an error, it is
        # a lane that quietly needs more distance than it used to: if a lane
        # starts wanting a second turn to find a tag it always found, come
        # down before suspecting the mounting.
        self.tag_sweep_speed = config.getfloat(
            "tag_sweep_speed", 100.0, above=0.0)
        # Motion is issued in chunks so a hit can stop the sweep promptly --
        # this is the abort granularity, not the poll interval.
        #
        # 0 (the default) sizes it from the lane's own speed and acceleration
        # instead, and that is almost always what you want -- see
        # _sweep_step_for. A fixed chunk that is shorter than the ramps is a
        # chunk the motor spends entirely accelerating and decelerating, which
        # is what "chunky" actually is.
        self.tag_sweep_step_mm = config.getfloat(
            "tag_sweep_step_mm", 0.0, minval=0.0)
        self.read_timeout_s = config.getfloat(
            "read_timeout_s", 6.0, above=0.0)
        self.retract_after_read = config.getboolean(
            "retract_after_read", True)
        # Speed for the restoring retract. The sweep is slow because a tag has
        # to be given time to answer inside the coil; the way BACK has nothing
        # to look for, so running it at the sweep's speed just makes the
        # operator wait. 0 means "the lane's own long-move speed", which is
        # what every other full-length move on this lane already uses.
        self.tag_retract_speed = config.getfloat(
            "tag_retract_speed", 0.0, minval=0.0)
        # Scan the tag automatically when a spool is inserted. Only lanes a
        # reader actually claims are ever scanned, so a BoxTurtle with readers
        # on half its lanes leaves the other half alone.
        self.scan_on_insert = config.getboolean("scan_on_insert", True)
        # Two lanes share one antenna, so the sister lane's PARKED tag can sit
        # in the field and be read as this lane's. Rolling that sister back a
        # little rotates its tag off the coil for the duration of the scan.
        # Same idea as ACE2's auto_tag_adjust, and the same hard gates.
        self.sibling_tag_adjust = config.getboolean(
            "sibling_tag_adjust", True)
        # 45mm, measured rather than guessed: on hardware a 75mm ask was cut
        # short at 60mm by lane10's load switch and the tag was already clear
        # of the coil, so the extra travel was only bringing the sister closer
        # to coming out of its lane for nothing.
        self.sibling_tag_adjust_dist = config.getfloat(
            "sibling_tag_adjust_dist", 45.0, minval=1.0)
        # Set while a sweep owns the lane. See _refuse_if_busy.
        self._sweeping = False
        # One probe thread at a time -- see _connect_timer.
        self._probing = False
        self.printer.register_event_handler(
            "afc:lane_prep_loaded", self._on_lane_prep_loaded)

        self._readers: List[AFC_BoxTurtle_rfid_reader] = []
        self._reader_by_lane: Dict[str, AFC_BoxTurtle_rfid_reader] = {}
        self._last_uid_by_lane: Dict[str, str] = {}
        # ACE2's `baseline`: the parked sibling UID to exclude from a read when
        # the sister could not be moved off the antenna. See _clear_sibling.
        self._baseline_uid: Optional[str] = None

        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.gcode.register_command(
            "AFC_BT_RFID_READ", self.cmd_AFC_BT_RFID_READ,
            desc="Read the tag on a BoxTurtle lane's reader and apply it. "
                 "AFC_BT_RFID_READ LANE=<lane>")
        self.gcode.register_command(
            "AFC_BT_RFID_STATUS", self.cmd_AFC_BT_RFID_STATUS,
            desc="Report the BoxTurtle RFID readers: bridge link, chip "
                 "versions, lane map, last reads. AFC_BT_RFID_STATUS")
        self.gcode.register_command(
            "AFC_BT_RFID_STAGE", self.cmd_AFC_BT_RFID_STAGE,
            desc="Experiment feed: advance the lane so the spool spins the "
                 "tag past the antenna, poll for a read while it moves, then "
                 "retract. AFC_BT_RFID_STAGE LANE=<lane> [ADVANCE=mm] "
                 "[SPEED=] [TIMEOUT=s] [RETRACT=0|1]")

    def _register_writers(self) -> None:
        """
        Offer every reader to the shared tag writer (AFC_RFID_WRITE).

        A reader is only usable while the Pico is there, so the link is fetched
        per call rather than captured: unplug the bridge and the writer reports
        the reader offline instead of handing out a dead port.
        """
        for rdr in self._readers:
            register_reader(
                self.printer, f"bt:{rdr.name}",
                f"BoxTurtle {rdr.name} (bus {rdr.bus}, "
                f"lanes {', '.join(rdr.lanes)})", self,
                lambda r=rdr: (r.link if r.link is not None
                               and self.bridge.connected() else None),
                threaded=True, stage=self._stage_for_write,
                unstage=self._unstage_after_write)

    def _stage_for_write(self, lane_name: str) -> Optional[tuple]:
        """
        Spin the lane's spool until its tag reaches the antenna and hold it,
        so a write lands on a tag the reader can actually see.

        The tag is only in the coil's field while the spool turns, so a write
        has to bring it back -- and the read sweep already stops the instant the
        tag answers, leaving it parked, which is exactly where the write needs
        it. Runs ON THE REACTOR (it feeds filament). Returns a token for
        _unstage_after_write, or None when there is nothing to spin.

        :param lane_name: the lane whose spool carries the tag
        :return: (lane, fed_mm), or None
        """
        lane = self._lane_obj(lane_name)
        rdr = self._reader_by_lane.get(lane_name)
        if lane is None or rdr is None or rdr.version is None:
            self.logger.info(
                f"AFC_BT_RFID: stage-for-write skipped {lane_name} "
                f"(lane={lane is not None}, rdr={rdr is not None}, "
                f"ver={None if rdr is None else rdr.version})")
            return None
        if self._sweeping:
            self.logger.info(
                f"AFC_BT_RFID: stage-for-write busy, skipping {lane_name}")
            return None
        self._sweeping = True
        advance = math.pi * self.spool_diameter_mm * self.tag_sweep_revs
        step = (self.tag_sweep_step_mm
                or self._sweep_step_for(lane, self.tag_sweep_speed))
        try:
            tag, fed = self._sweep_for_tag(lane, lane_name, rdr, advance,
                                           step, self.tag_sweep_speed)
            settled = 0.0
            if tag is not None:
                settled = self._settle_on_tag(lane, lane_name, rdr,
                                              step + 25.0)
                fed += settled
        except Exception:
            self._sweeping = False
            raise
        self.logger.info(
            f"AFC_BT_RFID: staged {lane_name} for write -- tag "
            f"{'found ' + str(tag.get('uid')) if tag else 'NOT found'} after "
            f"{fed:.0f}mm (re-centred {settled:.0f}mm); holding for the write.")
        return (lane, fed)

    def _settle_on_tag(self, lane: Any, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader,
                       back_limit: float) -> float:
        """
        Ease a just-swept tag back into the coil's field for the write.

        The coarse sweep can only stop on a chunk boundary -- up to one step
        past the tag -- which is fine for a read taken DURING the sweep but can
        carry the tag out of range before a write reads it afterwards. Nudge
        back in short steps, polling, until the reader answers again. Bounded
        and best-effort: if it never re-reads, leave the filament where the
        sweep left it and let the write report the miss.

        :param lane: the AFC lane
        :param lane_name: the lane's name
        :param rdr: the reader watching it
        :param back_limit: the most to retract, mm
        :return float: net mm moved (<= 0, a retract), for the restore
        """
        move = 3.0
        speed = getattr(lane, "short_moves_speed", None) or self.tag_sweep_speed
        moved = 0.0
        # Ease back until the reader re-reads the over-shot tag. It re-enters
        # at the FAR edge of the coil's field.
        found = self._read_once(lane_name, rdr) is not None
        while not found and -moved < back_limit:
            self._lane_move(lane, -move, speed, assist=True)
            moved -= move
            found = self._read_once(lane_name, rdr) is not None
        if not found:
            return moved
        # A read holds at the edge, but an NTAG page write and its read-back
        # need the tag nearer the CENTRE of the field. Ease further in while the
        # read holds; when it drops off the near edge, step back onto the last
        # good spot. That lands it mid-field, where the write couples.
        for _ in range(6):
            self._lane_move(lane, -move, speed, assist=True)
            moved -= move
            if self._read_once(lane_name, rdr) is None:
                self._lane_move(lane, move, speed, assist=True)
                moved += move
                break
        return moved

    def _unstage_after_write(self, token: Optional[tuple]) -> None:
        """
        Put the filament back where _stage_for_write found it, then release the
        sweep guard.

        :param token: the (lane, fed) _stage_for_write returned
        """
        try:
            if not token:
                return
            lane, fed = token
            back = (self.tag_retract_speed
                    or getattr(lane, "long_moves_speed", None)
                    or self.tag_sweep_speed)
            self._restore(lane, fed, back)
        finally:
            self._sweeping = False

    def _handle_ready(self) -> None:
        """Collect the reader sections, resolve keys, wire each reader's
        serial link, and start the connect timer. No hardware I/O here --
        the timer owns the port so an absent bridge only ever costs a retry.
        """
        self.afc = self.printer.lookup_object("AFC", None)
        # AFC's own logger from here on: it writes AFC.log with timestamps and
        # call sites, and echoes to the g-code console, which is where anyone
        # debugging a lane is already looking. The stdlib logger this was
        # The readers take AFC's logger in their own __init__ now, so only the
        # bridge -- which is a plain object this class owns, not a Klipper one
        # with a config of its own -- still needs handing it.
        self.bridge.logger = self.logger
        (self.bambu_master_key, self.creality_key,
         self.creality_encryption_key) = resolve_rfid_keys(
            self.printer, self.bambu_master_key, self.creality_key,
            self.creality_encryption_key)
        self._readers = [
            obj for name, obj in self.printer.lookup_objects("AFC_BoxTurtle_rfid")
            if isinstance(obj, AFC_BoxTurtle_rfid_reader)]
        self._reader_by_lane = {}
        for rdr in self._readers:
            rdr.link = _SerialRegLink(self.bridge, rdr.bus)
            for ln in rdr.lanes:
                self._reader_by_lane[ln] = rdr
        self._register_writers()
        self.reactor.register_timer(
            self._connect_timer, self.reactor.monotonic() + 1.0)

    def _probe_all(self) -> Tuple[bool, List[Tuple[Any, bool]]]:
        """
        THE BLOCKING HALF, and it must not run on the reactor.

        Opening the port and reading VersionReg are serial ops behind a lock a
        sweep worker can be holding, each bounded by _OP_TIMEOUT_S. Doing that
        on a reactor timer means the reactor can sit for a quarter of a second
        per offline reader, plus however long the sweep in front of it holds
        the lock -- and Klipper measures its own lateness in milliseconds.

        Returns what it learned rather than applying it: rdr.version is read by
        the scan gates on the reactor, so it is latched there.

        :return tuple: (bridge is connected, [(reader, fresh), ...] probed)
        """
        was = self.bridge.connected()
        if not self.bridge.connect():
            return (False, [])
        fresh = not was
        # A sweep is a timed read against a moving spool on this same link;
        # a probe in the middle of one is the HT-polling mistake again. Not
        # urgent -- the next tick will do.
        if not fresh and self._sweeping:
            return (True, [])
        probed = []
        for rdr in self._readers:
            if not (fresh or rdr.version is None):
                continue                  # answering already: leave it alone
            try:
                ver = rdr.link.reg_read(_VERSION_REG)
                probed.append((rdr, fresh, ver, None))
            except Exception as e:
                probed.append((rdr, fresh, None, e))
        return (True, probed)

    def _probe_worker(self) -> None:
        """Run _probe_all off-reactor and hand the answer back to it."""
        try:
            thread_name = threading.current_thread().name
            chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
        except Exception:
            pass
        try:
            try:
                ok, probed = self._probe_all()
            except Exception:
                ok, probed = (False, [])
            # Registered BEFORE the flag clears, deliberately: clearing first
            # would let the next tick start a probe while this one's results
            # were still on their way to the reactor.
            self.reactor.register_async_callback(
                lambda et: self._apply_probe(ok, probed))
        finally:
            self._probing = False

    def _apply_probe(self, ok: bool, probed: List[Any]) -> None:
        """
        Latch probe results on the reactor, where the scan gates read them.

        :param ok: whether the bridge is connected
        :param probed: (reader, fresh, version, error) from _probe_all
        """
        if not ok:
            for rdr in self._readers:
                rdr.version = None
            return
        for rdr, fresh, ver, err in probed:
            was = rdr.version
            rdr.version = ver
            # Edge-driven: at _RETRY_S an absent reader would otherwise warn
            # 720 times an hour in the log someone reads to find out what
            # else went wrong.
            if err is not None:
                if fresh or was is not None:
                    self.logger.warning(
                        f"BT RFID {rdr.name}: bridge is up but bus "
                        f"{rdr.bus} has no reader ({err}) -- check wiring")
            elif fresh or was is None:
                self.logger.info(
                    f"BT RFID {rdr.name}: version reg "
                    f"0x{ver:02X} (bus {rdr.bus}, lanes "
                    f"{', '.join(rdr.lanes)})")

    def _connect_timer(self, eventtime: float) -> float:
        """
        Timer: keep the bridge connected, and keep asking an OFFLINE reader
        whether it is back. Runs forever at a slow cadence and TOUCHES NO
        HARDWARE ITSELF -- it hands the port work to a thread and returns, so
        the reactor tick is a flag check.

        THE PROBE USED TO HAPPEN ONCE. It was gated on a fresh connect, so a
        reader that did not answer that single moment stayed `version = None`
        for the life of the session -- and None is not cosmetic: it skips the
        tag scan on insert and makes AFC_BT_RFID_STAGE refuse outright. A
        reader wedged at boot, plugged in afterwards, or briefly unhappy on a
        shared bus was therefore dead until Klipper restarted or the bridge
        link happened to drop. Retrying costs one register read per offline
        reader per tick and nothing at all once it answers.

        A READER THAT IS ANSWERING IS LEFT ALONE. Re-reading a chip that is
        already known good buys nothing and puts avoidable traffic on the link
        the sweeps use.

        NOT DURING A SWEEP. A sweep is a timed read against a moving spool on
        the same serial bridge; a probe landing in the middle of one is the
        same mistake as polling an HT mid-scan. The retry simply waits for the
        next tick -- an offline reader is not urgent, and a sweep is.

        ONE PROBE IN FLIGHT AT A TIME. The tick is faster than a probe can be
        when the port is timing out, so without the flag a wedged bus would
        spawn a new thread every _RETRY_S for as long as it stayed wedged.

        :param eventtime: the reactor time the timer fired at
        :return float: the next wake time
        """
        if not self._probing:
            self._probing = True
            threading.Thread(target=self._probe_worker, daemon=True,
                             name="afc_bt_rfid_probe").start()
        return eventtime + _RETRY_S

    def _lane_obj(self, lane_name: str) -> Any:
        """
        Resolve an AFC lane object by name, or None.

        :param lane_name: the AFC lane name
        :return: the lane object or None
        """
        if self.afc is None:
            return None
        return getattr(self.afc, "lanes", {}).get(lane_name)

    def _excluder_for(self, lane_name: str,
                      rdr: AFC_BoxTurtle_rfid_reader) -> Optional[
                          Callable[[str], bool]]:
        """
        Build the shared-antenna exclusion: the partner lane's last-known UID
        answers "excluded", so the read stack keeps hunting past the sibling.

        :param lane_name: the lane being read
        :param rdr: the reader both lanes share
        :return: uid_hex -> bool, or None when no sibling UID is known
        """
        sibling_uids = {
            uid.lower() for ln, uid in self._last_uid_by_lane.items()
            if ln != lane_name and ln in rdr.lanes}
        # And the tag the probe SAW parked, when the sister could not be moved
        # off it. That one has no history to be looked up by -- it is the
        # first-encounter case the last-known-UID set cannot cover -- so
        # without it a parked tag gets read as this lane's.
        if self._baseline_uid:
            sibling_uids.add(self._baseline_uid.lower())
        if not sibling_uids:
            return None
        return lambda uid_hex: uid_hex.lower() in sibling_uids

    def _parked_tag(self, rdr: AFC_BoxTurtle_rfid_reader) -> Optional[str]:
        """
        Raw probe with NO excluder: whatever tag is on the antenna right now.

        Run before this lane's spool turns, so anything it answers is a
        PARKED tag -- and the only thing parked on a shared antenna is the
        sister lane's. That is what makes it a test for "is there actually a
        collision", rather than assuming one.

        Deliberately un-excluded. The excluder exists to skip past a known
        sibling UID during the sweep; here the sibling UID is exactly what is
        being looked for, so filtering it out would defeat the probe.

        WORKER THREAD ONLY, like _read_blocking: it is the same stack of
        serial round-trips.

        :param rdr: the reader whose antenna both lanes share
        :return: the parked tag's UID hex, or None if the antenna is clear
        """
        if rdr.link is None or not self.bridge.connected():
            return None
        try:
            tag = read_tag(
                rdr.link,
                bambu_master_key=self.bambu_master_key,
                creality_key=self.creality_key,
                creality_encryption_key=self.creality_encryption_key)
        except Exception:
            return None                   # a probe that fails is "no collision"
        return (tag or {}).get("uid") or None

    def _read_blocking(self, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader) -> Optional[dict]:
        """
        One pass of the shared read stack -- WORKER THREAD ONLY. A tag read
        is hundreds of serial round-trips (~1s), which must never run on the
        reactor; _read_once is the reactor-safe wrapper.

        :param lane_name: the lane the read is for (drives sibling exclusion)
        :param rdr: the reader to poll
        :return: the raw read_tag dict, or None
        """
        if rdr.link is None or not self.bridge.connected():
            return None
        try:
            return read_tag(
                rdr.link,
                bambu_master_key=self.bambu_master_key,
                is_excluded=self._excluder_for(lane_name, rdr),
                creality_key=self.creality_key,
                creality_encryption_key=self.creality_encryption_key)
        except Exception as e:
            self.logger.warning(
                f"BT RFID {rdr.name}: read failed: {e}")
            return None

    def _read_once(self, lane_name: str,
                   rdr: AFC_BoxTurtle_rfid_reader) -> Optional[dict]:
        """
        Run one read on a worker thread and wait reactor-friendly: the
        calling g-code blocks, the printer does not.

        :param lane_name: the lane the read is for
        :param rdr: the reader to poll
        :return: the raw read_tag dict, or None
        """
        completion = self.reactor.completion()
        result: List[Any] = [None]

        def worker() -> None:
            """
            Background read; completes the reactor completion when done.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            result[0] = self._read_blocking(lane_name, rdr)
            self.reactor.register_async_callback(
                lambda et: completion.complete(None))

        threading.Thread(target=worker, daemon=True,
                         name="afc_bt_rfid_rd").start()
        completion.wait(self.reactor.monotonic() + 30.0)
        return result[0]

    def _parked_tag_once(self, rdr: AFC_BoxTurtle_rfid_reader) -> Optional[str]:
        """
        _parked_tag on a worker thread, waited for reactor-friendly.

        _clear_sibling runs on the REACTOR -- from the insert handler and from
        the g-code command -- and the probe is the same second of serial
        round-trips a read is. Calling it directly there stalls the reactor on
        every insert, which is the bug the closing read already had once.

        :param rdr: the reader whose antenna both lanes share
        :return: the parked tag's UID hex, or None
        """
        completion = self.reactor.completion()
        result: List[Any] = [None]

        def worker() -> None:
            """
            Background parked-tag read; completes the reactor completion when done.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            result[0] = self._parked_tag(rdr)
            self.reactor.register_async_callback(
                lambda et: completion.complete(None))

        threading.Thread(target=worker, daemon=True,
                         name="afc_bt_rfid_rd").start()
        completion.wait(self.reactor.monotonic() + 30.0)
        return result[0]

    def _apply(self, lane_name: str, tag: dict, gcmd: Any) -> None:
        """
        Apply a successful read to the lane through the shared framework.

        :param lane_name: the lane the tag belongs to
        :param tag: the raw read_tag result
        :param gcmd: the invoking command, for console output
        """
        self._last_uid_by_lane[lane_name] = (tag.get("uid") or "").lower()
        lane = self._lane_obj(lane_name)
        if lane is None:
            gcmd.respond_info(
                f"AFC_BT_RFID: read uid {tag.get('uid')} but lane {lane_name} "
                f"is not an AFC lane -- nothing applied.")
            return
        self.apply_to_lane(lane, tag)

    def _map(self, tag: dict) -> dict:
        """
        AFCUnitRFID adapter hook: raw read -> shared slot_info shape.

        :param tag: the raw read_tag result
        :return dict: AFC slot_info
        """
        return map_tag_to_slot_info(tag)

    def _require_lane_reader(self, gcmd: Any):
        """
        Resolve LANE= to (lane_name, reader) or raise the g-code error.

        :param gcmd: the invoking command
        :return tuple: (lane_name, reader)
        """
        lane_name = gcmd.get("LANE")
        rdr = self._reader_by_lane.get(lane_name)
        if rdr is None:
            error_str = (f"AFC_BT_RFID: no reader serves lane {lane_name}. "
                         f"Configured: "
                         f"{sorted(self._reader_by_lane) or 'none'}")
            raise gcmd.error(error_str)
        return lane_name, rdr

    def cmd_AFC_BT_RFID_READ(self, gcmd: Any) -> None:
        """
        Read the tag in a lane's antenna right now and apply it.

        AFC_BT_RFID_READ LANE=<lane>

        A spool at rest may hold its tag outside the antenna's arc -- that is
        what AFC_BT_RFID_STAGE's feed is for. This command is the no-motion probe:
        whatever is in the field right now.

        :param gcmd: The Klipper GCodeCommand
        """
        lane_name, rdr = self._require_lane_reader(gcmd)
        tag = self._read_once(lane_name, rdr)
        if tag is None:
            gcmd.respond_info(
                f"AFC_BT_RFID_READ: nothing readable in {rdr.name}'s field for "
                f"{lane_name}. If the tag rides the spool, use AFC_BT_RFID_STAGE "
                f"to spin it past the antenna.")
            return
        self._apply(lane_name, tag, gcmd)

    def cmd_AFC_BT_RFID_STATUS(self, gcmd: Any) -> None:
        """
        Report readers, chip versions, the lane map and last reads.

        AFC_BT_RFID_STATUS

        :param gcmd: The Klipper GCodeCommand
        """
        if not self._readers:
            gcmd.respond_info(
                "AFC_BT_RFID: no reader sections configured "
                "([AFC_BoxTurtle_rfid <name>] with bus + lanes).")
            return
        lines = [f"bridge: "
                 f"{'connected' if self.bridge.connected() else 'OFFLINE'} "
                 f"({self.serial_port})"]
        for rdr in self._readers:
            # "retrying", because offline is no longer a verdict for the
            # session: the connect timer keeps asking, so a reader that is
            # plugged in or unwedged comes back on its own.
            ver = ("offline (retrying)" if rdr.version is None
                   else f"version reg 0x{rdr.version:02X}")
            lines.append(f"{rdr.name}: {ver}  lanes: {', '.join(rdr.lanes)}")
        for ln, uid in sorted(self._last_uid_by_lane.items()):
            lines.append(f"{ln}: last uid {uid}")
        gcmd.respond_info("AFC_BT_RFID status\n" + "\n".join(lines))

    def _on_lane_prep_loaded(self, lane: Any) -> None:
        """
        Scan the inserted spool's tag, once AFC has seated the filament.

        WHY THIS EXACT MOMENT. It is after prep_load, which pulls the filament
        in and HOMES IT TO THE LOAD SENSOR, and before prep_post_load, which
        feeds dist_hub to the hub. That pair of facts is what the sweep needs
        and no other point in the insert has both:

        - The tip is at a known position. Hooking on the prep switch instead
          looks earlier and better, and is not: prep trips the instant the
          operator's filament touches it, before the gears have taken hold, so
          the tip is wherever the person pushing left it. The return then has
          no honest reference and over-runs.
        - There is room. The sweep rotates the spool by FEEDING filament, over
          a metre of it on a full reel, and here the whole bowden to the hub
          is still empty. After prep_post_load, or after a TD-1 capture has
          fed td1_bowden_length, the same sweep would push past the hub.

        Running first is also the right order for the data. The tag carries
        what the spool IS -- material, brand, colour, its Spoolman identity --
        and the TD-1 measures what the filament actually looks like. Identity
        first, measurement second, so the measurement refines the record
        rather than being overwritten by a nominal value that arrived later.

        Fire-and-forget: this is called from inside AFC's insert cycle, and a
        tag that cannot be read is not a reason to fail the insert.

        :param lane: the AFCLane being inserted
        """
        name = getattr(lane, "name", None)
        if not self.scan_on_insert or name is None:
            return
        rdr = self._reader_by_lane.get(name)
        if rdr is None:
            return                    # no reader watches this lane
        if self._sweeping:
            self.logger.warning(
                f"AFC_BT_RFID: {name} inserted while a sweep was already "
                f"running -- not scanning this one.")
            return
        if rdr.version is None:
            self.logger.warning(
                f"AFC_BT_RFID: {name} inserted but {rdr.name} is offline -- "
                f"skipping the tag scan. Check AFC_BT_RFID_STATUS.")
            return
        advance = math.pi * self.spool_diameter_mm * self.tag_sweep_revs
        back = self.tag_retract_speed or getattr(
            lane, "long_moves_speed", None) or self.tag_sweep_speed
        self._sweeping = True
        token = None
        try:
            token = self._clear_sibling(name, rdr)
            step = (self.tag_sweep_step_mm
                    or self._sweep_step_for(lane, self.tag_sweep_speed))
            tag, fed = self._sweep_for_tag(lane, name, rdr, advance,
                                           step, self.tag_sweep_speed)
            self._restore(lane, fed, back)
            if tag is None:
                self.logger.info(
                    f"AFC_BT_RFID: no tag on {name} after {fed:.0f}mm "
                    f"({fed / (math.pi * self.spool_diameter_mm):.1f} turns); "
                    f"filament restored.")
                return
            self.apply_to_lane(lane, tag)
            self._last_uid_by_lane[name] = (tag.get("uid") or "").lower()
            self.logger.info(
                f"AFC_BT_RFID: {name} tag {tag.get('uid')} read after "
                f"{fed:.0f}mm at {self.tag_sweep_speed:.0f}mm/s in "
                f"{step:.0f}mm chunks; filament restored.")
        except Exception as e:
            self.logger.warning(
                f"AFC_BT_RFID: tag scan failed for {name} ({e}) -- the insert "
                f"carries on without it.")
        finally:
            self._restore_sibling(token)
            self._sweeping = False

    def _sibling_lane(self, lane_name: str,
                      rdr: AFC_BoxTurtle_rfid_reader) -> Optional[Any]:
        """
        The other lane on this reader's antenna, if AFC knows it.

        :param lane_name: the lane being scanned
        :param rdr: the reader both lanes share
        :return: the sibling's AFCLane, or None
        """
        for ln in rdr.lanes:
            if ln == lane_name:
                continue
            obj = self._lane_obj(ln)
            if obj is not None:
                return obj
        return None

    def _clear_sibling(self, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader) -> Optional[tuple]:
        """
        Roll the sister lane back so its parked tag leaves the shared antenna.

        WHY THIS AND NOT UID EXCLUSION. The excluder can only reject a tag it
        has seen before -- it needs the sibling's UID already on file. The
        first time a sister spool's tag sits in the field there is nothing to
        compare against, and its tag is simply read as this lane's. That is
        the case that matters, because it is the one that happens on a fresh
        BoxTurtle, and it is silent: two lanes end up pointing at one spool.

        Moving the tag out of the field removes the ambiguity instead of
        trying to resolve it, so this runs BEFORE the sweep rather than in
        response to a suspicious read.

        SAFETY GATES, the same ones ACE2 uses, because this moves a lane the
        operator did not ask about: only ever an idle, hub-staged sibling.
        Never one loaded to the toolhead, never during a print.

        :param lane_name: the lane being scanned
        :param rdr: the reader both lanes share
        :return: a token for _restore_sibling, or None if nothing was moved
        """
        if not self.sibling_tag_adjust:
            return None
        sib = self._sibling_lane(lane_name, rdr)
        if sib is None:
            return None
        # IS THERE ACTUALLY A COLLISION? Until now this moved the sister on
        # every insert, tag on the antenna or not -- and that motion is what
        # jammed the hub twice and pulled a spool out of lane9 once. ACE2 and
        # ViViD both probe first and only move when something is parked
        # there; this now matches them. Most inserts stop touching the
        # neighbour at all.
        self._baseline_uid = None
        parked = self._parked_tag_once(rdr)
        if parked is None:
            return None
        # WHOSE TAG IS IT? A shared antenna is shared in both directions: the
        # lane just inserted is stationary too, and ITS tag can be the one
        # resting on the coil. Seen on hardware -- lane8's own 7bf0afff read
        # as parked, and lane9 was rolled back 75mm and re-staged for a
        # collision that did not exist.
        #
        # _last_uid_by_lane is the evidence for that, and it is the same
        # comparison ACE2's _is_sibling_tag makes. An unknown UID stays a
        # collision: that is the first-encounter case the clear exists for,
        # and guessing wrong there is the silent two-lanes-one-spool failure.
        mine = (self._last_uid_by_lane.get(lane_name) or "").lower()
        if mine and parked.lower() == mine:
            self.logger.info(
                f"AFC_BT_RFID: tag {parked} on {rdr.name}'s antenna is "
                f"{lane_name}'s own -- no collision, leaving {sib.name} "
                f"alone.")
            return None
        # OWNERSHIP BEFORE EXCLUSION. A parked tag is only the sister's if she
        # actually has a spool seated in the field. An empty sister -- no
        # filament at prep, or filament not on her load switch -- cannot be the
        # source, so the tag belongs to the lane being read. Excluding it there
        # is exactly how a fresh tag on THIS lane got reported as "no tag": its
        # own read was skipped as the neighbour's. Set no baseline in that case;
        # let the sweep read the tag. (self._baseline_uid was cleared above.)
        if not getattr(sib, "prep_state", False) or not self._seated(sib):
            self.logger.info(
                f"AFC_BT_RFID: tag {parked} on {rdr.name}'s antenna, but "
                f"{sib.name} holds no seated spool -- it is {lane_name}'s own, "
                f"reading it.")
            return None
        # From here the sister is present and seated, so the parked tag is
        # hers. ACE2's other half, and the one that needs no motion: hold the
        # parked UID as a baseline exclusion from here on. Every safety gate
        # below returns without moving the sister, and in those cases the tag
        # is still sitting on the coil -- so the read must be told to skip it or
        # it gets attributed to this lane. Cleared once the sister has actually
        # been rolled clear.
        self._baseline_uid = parked
        self.logger.info(
            f"AFC_BT_RFID: tag {parked} is parked on {rdr.name}'s antenna "
            f"while {lane_name} is read -- clearing {sib.name} off it.")
        if getattr(sib, "tool_loaded", False):
            return None                   # loaded to the toolhead: never move
        # loaded_to_hub is REMEMBERED, not measured, and a stale one is how
        # this pulled lane9 clean out of its lane: the flag said staged while
        # the filament was not actually there, so a blind retract had
        # nothing to give back. The load switch (checked above via _seated) is
        # the measurement, so it gets the last word over the flag.
        if not getattr(sib, "loaded_to_hub", False):
            return None                   # position unknown: do not guess
        try:
            if self.afc.function.is_printing():
                return None               # never mid-print
        except Exception:
            pass
        want = self.sibling_tag_adjust_dist
        speed = getattr(sib, "long_moves_speed", None) or self.tag_sweep_speed
        step = getattr(sib, "short_move_dis", None) or 10.0
        moved = 0.0
        try:
            # Stepped, watching the switch, so the distance is a MAXIMUM
            # rather than a commitment. A properly staged sister has metres of
            # filament past its load switch and simply runs the full distance; a
            # badly staged one stops the moment it would come off the switch,
            # which is the case that used to end with it on the floor.
            while moved < want and self._seated(sib):
                hop = min(step, want - moved)
                self._lane_move(sib, -hop, speed, assist=True)
                moved += hop
        except Exception as e:
            self.logger.warning(
                f"AFC_BT_RFID: could not roll {sib.name} back off the shared "
                f"antenna ({e}); reading {lane_name} anyway.")
            return (sib, moved, speed) if moved > 0.0 else None
        if moved <= 0.0:
            return None
        self._baseline_uid = None     # it is off the antenna now
        short = " (stopped early at its load switch)" if moved < want else ""
        self.logger.info(
            f"AFC_BT_RFID: rolled {sib.name} back {moved:.0f}mm to clear its "
            f"tag off {rdr.name}'s antenna while {lane_name} is read{short}.")
        return (sib, moved, speed)

    def _restore_sibling(self, token: Optional[tuple]) -> None:
        """
        Put a rolled-back sister lane back where a normal insert leaves it:
        staged at the hub, with the hub itself CLEAR.

        THE TARGET IS THE STAGED POSITION, NOT THE DISTANCE TAKEN. Giving back
        exactly what was taken only lands right if neither move slipped, and
        both do -- the gears slip against a spool that has not started turning,
        the same effect that walked the scanned lane out of its own lane. The
        error is one-directional and silent: the sister finishes short of where
        it started while ``loaded_to_hub`` still says True, so its next
        toolchange begins from a remembered position that is now a guess.

        So the give-back is the coarse move and the HUB is the answer. Step
        forward until the hub switch sees the tip, then step off it. That is
        the position dist_hub staging aims for, arrived at by measuring the hub
        instead of trusting the load sensor plus an assumed distance, so slip
        in either direction is absorbed rather than accumulated.

        LEAVING THE HUB CLEAR IS NOT OPTIONAL, and is why the hunt does not
        simply stop on the switch. A hub-staged lane sits just SHORT of the
        hub, and a TD-1 capture refuses outright while the hub reads filament
        ("Hub for <lane> detects filament") -- so a sister parked ON the switch
        breaks the very capture that follows the scan. An earlier version fed
        until the hub triggered and left it there, which is that bug; the step
        off, plus one short move of clearance, is the fix.

        If the hub is never found, or the sister does not come back to its own
        load switch, ``loaded_to_hub`` is cleared rather than left True. A
        wrong position AFC does not know about is worse than no position: the
        flag being False just makes the next load re-stage it.

        :param token: the value _clear_sibling returned
        """
        if not token:
            return
        sib, dist, speed = token
        try:
            self._lane_move(sib, dist, speed, assist=False)
            # If the roll-back came off the load switch after all, home back
            # onto it before worrying about the hub -- a sister behind its own
            # load switch is the state that reads as "spool removed".
            step = getattr(sib, "short_move_dis", None) or 10.0
            slowf = getattr(sib, "short_moves_speed", None) or speed
            crept = 0.0
            for _ in range(int(dist / step) + 4):
                if self._seated(sib):
                    break
                self._lane_move(sib, step, slowf, assist=False)
                crept += step
            if crept:
                self.logger.info(
                    f"AFC_BT_RFID: fed {sib.name} a further {crept:.0f}mm to "
                    f"put it back on its load switch.")
            if not self._seated(sib):
                # Say it plainly. A sister off its own load switch reads as
                # "spool removed", and the operator needs to know it was this
                # that moved it rather than finding out later.
                sib.loaded_to_hub = False
                self.logger.warning(
                    f"AFC_BT_RFID: {sib.name} is NOT back on its load switch "
                    f"after the read -- its filament may have come out of the "
                    f"lane. Re-seat it and check the spool.")
                return
            self._restage_sibling(sib, dist)
        except Exception as e:
            sib.loaded_to_hub = False
            self.logger.warning(
                f"AFC_BT_RFID: could not restore {sib.name} after the read "
                f"({e}); its next load will re-home it.")

    def _home_back_to_load(self, lane: Any, dist: float) -> None:
        """
        Retract to the load switch with the espooler running.

        Same move as the unit's move_to_load -- same endstop, same speed mode,
        homing so the MCU stops it -- with one deliberate difference: assist
        is YES rather than DYNAMIC.

        DYNAMIC is literally `abs(distance) > 200`, and every dist_hub on a
        BoxTurtle here is 131-188mm, so DYNAMIC means "off" for every retract
        this module makes. Feeding is the direction that free-wheels; the
        retract is the one that needs the spooler, because without it the
        filament it pulls back simply piles up loose on the reel instead of
        winding on. That was observed on hardware, and it is why the stepped
        version of this used assist on retracts and not on feeds.

        AFC is itself inconsistent here -- eject_lane uses DYNAMIC for its
        dist_hub retract and YES for the short ones after it -- so a machine
        with dist_hub over 200 gets the spooler and this one would not.

        :param lane: the AFC lane to retract
        :param dist: how far to retract, mm -- the endstop is what stops it
        """
        lane.move_to(dist * MoveDirection.NEG, SpeedMode.LONG,
                     endstop=lane.load_es, assist_active=AssistActive.YES,
                     use_homing=True)

    def _homing_available(self, lane: Any) -> bool:
        """
        Whether AFC's own homing moves can be used on this lane.

        :param lane: the AFC lane
        :return bool: True when move_to, the enums and homing are all present
        """
        return (SpeedMode is not None and MoveDirection is not None
                and AssistActive is not None
                and callable(getattr(lane, "move_to", None))
                and bool(getattr(self.afc, "homing_enabled", False)))

    def _restage_sibling(self, lane: Any, given_back: float) -> None:
        """
        Put the sibling back exactly where a normal insert leaves it, by
        making the two moves a normal insert makes.

        WHY NOT HUNT FOR THE HUB -- IT JAMMED THE HUB. The previous version
        stepped the tip forward until the hub switch saw it and then backed
        off a clearance. That parks the tip AT the hub bore, which on a
        BoxTurtle is the shared Y every other lane has to pass through. It
        cost two runs: the sister sat in the bore and the scanned lane could
        not get past it, so the hub never triggered and both ended up jammed
        in it. `Failed to trigger hub Turtle_1 for lane8` was lane9's filament
        in the way, not a hub fault.

        AFC never does that, and the reason is the same one. Its load path is
        two moves and neither goes near the hub:

          prep_load       home onto the LOAD sensor (a real endstop move --
                          the MCU stops it, so it lands on the switch edge)
          prep_post_load  feed dist_hub, a calibrated distance that stops
                          SHORT of the hub

        So this does exactly that, through AFC's own methods rather than a
        re-implementation: the same homing retract eject_lane makes, then the
        unit's own prep_post_load. The sibling ends up in the state a fresh
        insert leaves it in, dist_hub is the authority on where "staged" is,
        and no filament is ever driven at the hub.

        Gated like prep_post_load: load_to_hub must be set, and a direct-hub
        lane has no gap to stage in. Without homing there is no endstop to
        stop the retract, so it keeps the plain give-back and clears
        loaded_to_hub instead of guessing -- the next load re-stages it.

        :param lane: the sibling lane to re-stage
        :param given_back: distance already re-fed, mm -- for the log only
        """
        hub = getattr(lane, "hub_obj", None)
        if hub is None or getattr(lane, "is_direct_hub", lambda: False)():
            return                        # nothing between the lane and the tool
        if not getattr(lane, "load_to_hub", False):
            return                        # config says this lane does not stage
        unit = getattr(lane, "unit_obj", None)
        dist_hub = getattr(lane, "dist_hub", None)
        if unit is None or not dist_hub or not callable(
                getattr(unit, "prep_post_load", None)) or not callable(
                getattr(unit, "prep_load", None)):
            return
        if not self._homing_available(lane):
            lane.loaded_to_hub = False
            self.logger.warning(
                f"AFC_BT_RFID: homing is off, so {lane.name} was only re-fed "
                f"{given_back:.0f}mm rather than re-staged. Its next load "
                f"will put it back at the hub.")
            return
        # 1. Back to the load switch, the way a load and an unload do it.
        #    First the reverse home eject_lane makes...
        self._home_back_to_load(lane, dist_hub)
        # ...and then forward onto it, because the reverse home stops where
        # the switch RELEASES. prep_load is that forward home -- the insert
        # cycle's own -- so this pair is literally unload-then-load. Without
        # it prep_post_load's load_state guard is false and step 2 below is a
        # silent no-op.
        unit.prep_load(lane)
        if not self._seated(lane):
            # Not a case to paper over: the proven move says the switch is
            # made when it returns, so this means something else is wrong.
            lane.loaded_to_hub = False
            self.logger.warning(
                f"AFC_BT_RFID: {lane.name} would not come back to its load "
                f"switch, so it is not staged -- check the spool. Its next "
                f"load will re-home it.")
            return
        # 2. Feed dist_hub. prep_post_load's own job, and it sets
        #    loaded_to_hub itself; it only runs when the flag is clear.
        lane.loaded_to_hub = False
        unit.prep_post_load(lane)
        self.logger.info(
            f"AFC_BT_RFID: re-staged {lane.name} the way a load does -- "
            f"homed to its load switch, then fed dist_hub "
            f"({dist_hub:.0f}mm). The hub is untouched.")

    def _seated(self, lane: Any) -> bool:
        """
        Whether the filament is still at the sensor the sweep started from.

        The LOAD sensor, not prep. Prep trips the moment the operator's
        filament touches it, before the gears have taken hold, so the tip is
        not seated and its position is whatever the person pushing happened
        to leave. The load sensor is where prep_load HOMES the filament, so
        it is a real reference in both directions.

        :param lane: the AFC lane
        :return bool: True while filament covers the load sensor
        """
        raw = getattr(lane, "raw_load_state", None)
        if raw is not None:
            return bool(raw)
        return bool(getattr(lane, "load_state", False))

    def _restore(self, lane: Any, fed: float, speed: float) -> float:
        """
        Put the filament back on the load sensor the sweep started from.

        A measured ``-fed`` retract is only right if the feed advanced exactly
        what it was asked for. It does not: the gears slip against a spool
        that has not started turning yet, so the tip ends up SHORT of the
        commanded distance while the retract gives back the full amount. The
        error is one-directional, so it walks the tip backwards out of the
        lane -- which is what pulled the spool out on lane9.

        So the sensor is the answer, and the way to ask it is AFC's own
        homing move: retract with the LOAD sensor as the endstop and let the
        MCU stop the move on the switch. That is the same thing prep_load
        does on the way in and eject_lane does on the way out, so the tip
        lands on exactly the edge prep_load left it on -- not on an
        accumulated estimate, and not on a Python loop watching a switch
        between chunks, which can only ever stop a chunk late.

        Without homing there is no endstop to stop on, so it falls back to
        the stepped version: give back the bulk fast, then step the tail while
        checking the sensor, then creep forward if it went past.

        :param lane: the AFC lane to move
        :param fed: distance the sweep fed, mm
        :param speed: speed for the bulk of the return, mm/s
        :return float: distance actually given back, mm
        """
        if fed <= 0.0:
            return 0.0
        # Nothing on the sensor means nothing to home against -- the manual
        # command run on an empty lane, for instance. Give back the measured
        # distance and say so by returning it.
        if not self._seated(lane):
            self._lane_move(lane, -fed, speed, assist=True)
            return fed
        unit = getattr(lane, "unit_obj", None)
        if self._homing_available(lane) and callable(
                getattr(unit, "prep_load", None)):
            # AFC's homing direction decides what "home to load" MEANS.
            # move_to passes `triggered = distance > 0` into home_to, so:
            #   reverse  -> stop when the load switch RELEASES  (the unload)
            #   forward  -> stop when the load switch is MADE    (the load)
            # One reverse home therefore leaves the tip BEHIND the switch, not
            # on it. That is half the motion, and on its own it is what left
            # lane8 reading "LOCKED NOT LOADED" after a scan.
            #
            # The other half is prep_load, which is the forward home the
            # insert cycle itself uses -- plus its own short-move fallback and
            # its own fault reporting. Together they are exactly the two moves
            # a normal load makes to put filament on the load switch.
            self._home_back_to_load(lane, fed + 2.0 * (
                getattr(lane, "short_move_dis", None) or 10.0))
            unit.prep_load(lane)
            return fed
        short = getattr(lane, "short_move_dis", None) or 10.0
        slow = getattr(lane, "short_moves_speed", None) or speed
        tail = min(fed, max(3.0 * short, 30.0))
        back = 0.0
        bulk = fed - tail
        if bulk > 0.0:
            self._lane_move(lane, -bulk, speed, assist=True)
            back += bulk
        # The tail, stepped, so the sensor is checked between moves. A queued
        # Klipper move cannot be interrupted, so the step size IS the
        # overshoot bound.
        while self._seated(lane) and back < fed + tail:
            self._lane_move(lane, -short, slow, assist=True)
            back += short
        # Off the sensor now: creep forward until it catches again, which is
        # where the operator left the filament.
        for _ in range(int(tail / short) + 4):
            if self._seated(lane):
                break
            self._lane_move(lane, short, slow, assist=False)
            back -= short
        return back

    def _refuse_if_busy(self, gcmd: Any) -> None:
        """
        Refuse to start a sweep while anything else is moving filament.

        THIS COMMAND TOOK KLIPPER DOWN. An AFC lane move works by swapping the
        stepper onto its own trapq, submitting the move and swapping back
        (AFC_stepper._move). Two of those overlapping on one stepper interleave
        the swap, and step generation then fails with "Internal error in
        stepcompress", which shuts down every MCU on the printer. It happened
        for real: a sweep issued while a TD-1 capture was retracting.

        The overlap is possible because a lane move ends in
        ``toolhead.wait_moves()``, which pauses the reactor -- so a command
        arriving over the API runs in that gap, inside the other one.

        AFC has no "a lane is moving" flag to consult, so this uses the one
        stock Klipper does have. idle_timeout reads "Printing" whenever the
        toolhead has motion queued, which covers a TD-1 capture, a toolchange,
        a real print and anything else that moves, in one check.

        :param gcmd: the command to refuse through
        """
        if self._sweeping:
            error_str = ("AFC_BT_RFID_STAGE: a sweep is already running -- "
                         "wait for it to finish or restart the firmware.")
            raise gcmd.error(error_str)
        idle = self.printer.lookup_object("idle_timeout", None)
        if idle is None:
            return
        try:
            state = idle.get_status(self.reactor.monotonic()).get("state")
        except Exception:
            return
        if state == "Printing":
            error_str = ("AFC_BT_RFID_STAGE: the printer is moving something "
                         "already (idle_timeout says Printing) -- a TD-1 "
                         "capture, a toolchange or a print. Two filament "
                         "moves at once corrupt step generation and shut down "
                         "every MCU, so this waits rather than joining in. "
                         "Try again once it is idle.")
            raise gcmd.error(error_str)

    def _lane_move(self, lane: Any, distance: float, speed: float,
                   assist: bool) -> None:
        """
        Move the lane, driving the espooler only when asked.

        The two directions of this sweep want opposite things. Feeding pulls
        filament OFF the spool, which free-wheels on its own -- an assist
        there is just the espooler fighting the pull. The retract is the one
        that needs it: without the espooler winding the slack back on, the
        filament simply piles up loose on the reel, which is what showed up
        on hardware.

        :param lane: the AFC lane to move
        :param distance: signed distance, mm (negative retracts)
        :param speed: feed speed, mm/s
        :param assist: run the espooler for this move
        """
        accel = getattr(lane, "long_moves_accel", 400)
        active = False
        if assist:
            active = True
            getter = getattr(lane, "get_active_assist", None)
            if callable(getter) and AssistActive is not None:
                try:
                    active = getter(distance, AssistActive.YES)
                except Exception:
                    active = True
        lane.move(distance, speed, accel, active)

    # Fraction of each chunk that should be spent at speed rather than ramping.
    # 0.6 puts the ramps at 40% of the move, which reads as continuous motion;
    # pushing it higher only buys a coarser abort for very little smoothness.
    SWEEP_CRUISE_FRACTION = 0.6
    SWEEP_STEP_MIN = 20.0
    SWEEP_STEP_MAX = 150.0

    def _sweep_step_for(self, lane: Any, speed: float) -> float:
        """
        Choose a sweep chunk long enough to actually reach ``speed``.

        WHY THIS IS NOT A CONSTANT. A chunk is a complete move: accelerate to
        speed, cruise, decelerate to a stop. Reaching v at acceleration a takes
        v**2/(2a) at each end, so a chunk shorter than v**2/a never gets there
        at all -- it is a triangle that peaks partway up and comes straight
        back down.

        That is what "chunky" is, and it was the default here. At the 100mm/s
        and 250mm/s**2 a BoxTurtle actually runs, the ramps need 40mm and the
        chunk was 20mm: every hop peaked at 71mm/s, cruised for exactly zero
        millimetres, and averaged 35mm/s. It also explains why raising
        tag_sweep_speed barely helped -- 60mm/s averaged 34.9mm/s and 100mm/s
        averaged 35.4mm/s, because the chunk, not the speed, was the limit.

        So the chunk is sized from the speed and acceleration in play, to leave
        SWEEP_CRUISE_FRACTION of it at constant velocity. Bounded at both ends:
        never shorter than the old default, and never so long that a hit costs
        a lot of extra filament before the sweep can notice it.

        :param lane: the AFC lane, for its acceleration
        :param speed: sweep speed, mm/s
        :return float: chunk length, mm
        """
        accel = getattr(lane, "long_moves_accel", None) or 400.0
        ramps = (speed * speed) / accel          # both ramps together
        step = ramps / max(0.05, 1.0 - self.SWEEP_CRUISE_FRACTION)
        return min(self.SWEEP_STEP_MAX, max(self.SWEEP_STEP_MIN, step))

    def _sweep_for_tag(self, lane: Any, lane_name: str,
                       rdr: AFC_BoxTurtle_rfid_reader, advance: float,
                       step: float, speed: float) -> tuple:
        """
        Home on the tag: feed the lane while polling the reader CONTINUOUSLY,
        and stop the moment a tag answers -- the RFID read is the endstop, the
        distance bound is the failsafe. Mirrors the BoxTurtle's own
        move-until-state homing, with two differences that the hardware
        forces:

        - The poll runs on its own THREAD rather than between moves. A read is
          hundreds of serial round-trips (~0.3s even when nothing is there),
          so polling between chunks would leave the coil blind for most of the
          sweep and the tag can cross the arc in that gap.
        - Motion is still issued in chunks, because a Klipper move cannot be
          interrupted once queued. The chunk is the abort granularity only.

        :param lane: the AFC lane object to move
        :param lane_name: the lane's name (drives sibling exclusion)
        :param rdr: the reader whose antenna watches this lane
        :param advance: maximum distance to sweep, mm
        :param step: chunk size, mm
        :param speed: feed speed, mm/s
        :return tuple: (tag or None, distance fed when it answered)
        """
        stop = threading.Event()
        found: List[Any] = [None, 0.0]
        fed = [0.0]

        def poller() -> None:
            """
            Poll the reader until a tag is read or stop is set.
            """
            try:
                thread_name = threading.current_thread().name
                chelper.get_ffi()[1].set_thread_name(thread_name.encode("utf-8"))
            except Exception:
                pass
            while not stop.is_set():
                tag = self._read_blocking(lane_name, rdr)
                if tag:
                    found[0] = tag
                    found[1] = fed[0]
                    stop.set()
                    return

        th = threading.Thread(target=poller, daemon=True, name="afc_bt_rfid_sw")
        th.start()
        try:
            while fed[0] < advance and not stop.is_set():
                hop = min(step, advance - fed[0])
                self._lane_move(lane, hop, speed, assist=False)
                fed[0] += hop
        finally:
            stop.set()
            th.join(timeout=10.0)
        # The tag may have entered the arc during the final chunk, after the
        # poller's last look. One more read closes that window -- through
        # _read_once, because this runs on the REACTOR. _read_blocking is a
        # second of register round-trips and belongs on a worker thread; the
        # poller above is one, this is not, and calling it directly here
        # stalled the reactor on every sweep that found nothing.
        if found[0] is None:
            tag = self._read_once(lane_name, rdr)
            if tag:
                found[0], found[1] = tag, fed[0]
        return found[0], fed[0]

    def cmd_AFC_BT_RFID_STAGE(self, gcmd: Any) -> None:
        """
        Home on the lane's tag: pull filament in slowly while polling the
        reader continuously, stop as soon as the tag answers, and give up
        after a bounded number of spool revolutions.

        AFC_BT_RFID_STAGE LANE=<lane> [REVS=2] [ADVANCE=mm] [SPEED=mm/s]
                          [STEP=mm] [RETRACT=0|1] [RETRACT_SPEED=mm/s]

        REVS bounds the search in spool turns (converted with
        ``spool_diameter_mm``); ADVANCE overrides it with an explicit
        distance. Two revolutions is the sensible default: one guarantees
        every angular position passes the coil, and the second covers a spool
        whose effective diameter is smaller than assumed.

        Refuses outright if anything else is moving filament: two AFC lane
        moves at once corrupt step generation and shut down every MCU.

        RETRACT=1 (the default) puts the filament back exactly where it
        started, so a failed search costs nothing. It goes back in ONE move of
        the distance actually fed, at RETRACT_SPEED -- the lane's normal
        long-move speed unless told otherwise. Only the search has to be slow;
        the way back has nothing to read.

        :param gcmd: The Klipper GCodeCommand
        """
        self._refuse_if_busy(gcmd)
        lane_name, rdr = self._require_lane_reader(gcmd)
        lane = self._lane_obj(lane_name)
        if lane is None:
            error_str = f"AFC_BT_RFID_STAGE: {lane_name} is not an AFC lane"
            raise gcmd.error(error_str)
        if rdr.version is None:
            error_str = (f"AFC_BT_RFID_STAGE: {rdr.name} is offline -- no "
                         f"point moving filament at a reader that cannot "
                         f"answer. Check the bridge with AFC_BT_RFID_STATUS.")
            raise gcmd.error(error_str)
        revs = gcmd.get_float("REVS", self.tag_sweep_revs, above=0.0)
        default_mm = math.pi * self.spool_diameter_mm * revs
        advance = gcmd.get_float("ADVANCE", default_mm, minval=0.0)
        speed = gcmd.get_float("SPEED", self.tag_sweep_speed, above=0.0)
        step = gcmd.get_float("STEP", self.tag_sweep_step_mm, minval=0.0)
        if step <= 0.0:
            step = self._sweep_step_for(lane, speed)
        retract = gcmd.get_int("RETRACT", 1 if self.retract_after_read else 0,
                               minval=0, maxval=1)
        back = gcmd.get_float("RETRACT_SPEED", self.tag_retract_speed,
                              minval=0.0)
        if back <= 0.0:
            back = getattr(lane, "long_moves_speed", None) or speed

        gcmd.respond_info(
            f"AFC_BT_RFID_STAGE: homing on {lane_name}'s tag -- up to "
            f"{advance:.0f}mm ({revs:.1f} turns of a "
            f"{self.spool_diameter_mm:.0f}mm spool) at {speed:.0f}mm/s, "
            f"polling {rdr.name} throughout. Coming back at {back:.0f}mm/s.")
        self._sweeping = True
        token = None
        try:
            token = self._clear_sibling(lane_name, rdr)
            tag, fed = self._sweep_for_tag(lane, lane_name, rdr, advance,
                                           step, speed)
        finally:
            self._restore_sibling(token)
            self._sweeping = False
        if retract:
            # Homes on the prep sensor when there is filament there to home
            # against; falls back to the measured distance when there is not.
            self._restore(lane, fed, back)
        one_turn = math.pi * self.spool_diameter_mm
        if tag is None:
            if fed >= one_turn:
                verdict = ("That is a full pass of the spool, so the tag "
                           "never enters this antenna's field -- move the "
                           "reader rather than sweeping further.")
            else:
                verdict = (f"That is only {fed / one_turn:.1f} of a turn, so "
                           f"the tag may simply not have come round yet -- "
                           f"raise REVS before suspecting the mounting.")
            gcmd.respond_info(
                f"AFC_BT_RFID_STAGE: no tag in {fed:.0f}mm of sweep on "
                f"{rdr.name}"
                f"{' (filament restored)' if retract else ''}. {verdict}")
            return
        self._apply(lane_name, tag, gcmd)
        # NOT a number to tune with: where the tag sits in the spool's
        # rotation when it is loaded is arbitrary, so this distance is a
        # fresh draw every time -- anywhere from nothing to a full turn. The
        # bound has to stay a whole revolution no matter what today's spool
        # happened to cost.
        gcmd.respond_info(
            f"AFC_BT_RFID_STAGE: tag answered after {fed:.0f}mm of feed"
            f"{' (filament restored)' if retract else ''}.")


def load_config(config: "ConfigWrapper") -> AFC_BoxTurtle_rfid:
    """
    Klipper entry point for the bare ``[AFC_BoxTurtle_rfid]`` coordinator.

    :param config: Klipper config wrapper
    :return AFC_BoxTurtle_rfid: the coordinator
    """
    return AFC_BoxTurtle_rfid(config)


def load_config_prefix(config: "ConfigWrapper") -> AFC_BoxTurtle_rfid_reader:
    """
    Klipper entry point for ``[AFC_BoxTurtle_rfid <name>]`` reader sections.

    :param config: Klipper config wrapper
    :return AFC_BoxTurtle_rfid_reader: the reader
    """
    return AFC_BoxTurtle_rfid_reader(config)
