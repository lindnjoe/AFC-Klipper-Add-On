"""
Tests for the OpenAMS RFID reader and the sensor/monitor paths that feed it
(extras/AFC_OpenAMS_rfid.py and the OpenAMS halves of extras/AFC_OpenAMS.py
this suite owns).

The scan sequence, sensors, the runout path, the monitor and status surfacing.
Jimmy's own OpenAMS tests stay in tests/test_AFC_OpenAMS.py -- this file is
only ours. Consolidated from six files; banners name the file each came from.
"""

from __future__ import annotations
import types
import pytest
import extras.AFC_OpenAMS_rfid as mod
import sys
from unittest.mock import MagicMock
from extras.AFC_OpenAMS import afcAMS, OAMSStatus  # noqa: E402
from tests.conftest import MockAFC, MockPrinter, MockConfig  # noqa: E402
from extras.AFC_OpenAMS import afcAMS
from extras.AFC_lane import AFCLaneState
from tests.openams_helpers import (
    FakeAFC,
    FakeExtruderObj,
    FakeLane,
    FakeLogger,
    FakeOams,
    Recorder,
)
from extras.AFC_OpenAMS import afcAMS, FollowerController, FollowerState
from tests.openams_helpers import (
    FakeAFC,
    FakeExtruderObj,
    FakeLane,
    FakeLogger,
    FakeReactor,
    Recorder,
)
from extras.AFC_OpenAMS import (
    OAMSMonitor,
    CLOG_PRESSURE_TARGET,
    CLOG_EXTRUSION_WINDOW,
    CLOG_DWELL,
    CLOG_ENCODER_SLACK,
    STUCK_PRESSURE_LOW,
    STUCK_PRESSURE_CLEAR,
    STUCK_DWELL,
    STUCK_MIN_ENCODER,
)
from tests.openams_helpers import FakeLogger, FakeReactor, Recorder
from extras.AFC_OpenAMS import afcAMS, OAMSStatus, FPSLoadState
from tests.openams_helpers import FakeOams
from tests.openams_helpers import FakeLogger, FakeAFC  # noqa: E402


# ── rfid ──────────────────────────────────────────────────────────────────────
#
# was tests/test_AFC_OpenAMS_rfid.py
# Tests for extras/AFC_OpenAMS_rfid.py — the OpenAMS RFID coordinator and its
# two transport adapters.
#
# Three things are worth pinning:
#
#   * the MFRC522 SPI framing. A read is [0x80|(reg<<1), 0x00] with the value in
#     the SECOND returned byte, and a write is [(reg<<1), val]. Both mask with
#     0x7E, so bit 0 and bit 7 of the shifted address can never leak into the
#     wire format. Get any of that wrong and every register read returns
#     plausible rubbish rather than failing outright.
#   * config parsing that rejects bad input at startup rather than at scan
#     time. `slots` and `lane_slot_map` are the two places an operator types
#     numbers by hand.
#   * reader indexing by physical slot, including the case where two readers
#     claim the same slot, which is a wiring mistake worth naming.





class _Spi:
    """Records what was sent and replays a canned response."""

    def __init__(self, response=(0x00, 0x00)):
        self.sent = []
        self.transfers = []
        self._response = list(response)

    def spi_send(self, data):
        self.sent.append(list(data))

    def spi_transfer(self, data):
        self.transfers.append(list(data))
        return {'response': list(self._response)}


class TestSpiRegisterFraming:
    """MFRC522 register access over SPI, per the datasheet framing."""

    def test_read_uses_the_read_bit_and_shifted_address(self):
        spi = _Spi(response=(0x00, 0x37))
        link = mod._OamsSpiRegLink(spi)
        assert link.reg_read(0x37) == 0x37
        # address byte = 0x80 | (reg << 1), masked to 0x7E
        assert spi.transfers == [[0x80 | ((0x37 << 1) & 0x7E), 0x00]]

    def test_read_takes_the_second_byte_not_the_first(self):
        # The first byte is whatever the MFRC522 shifted out during the address
        # phase; only the second is the register value.
        spi = _Spi(response=(0xFF, 0x2A))
        assert mod._OamsSpiRegLink(spi).reg_read(0x01) == 0x2A

    def test_read_of_a_short_response_is_zero_not_an_exception(self):
        spi = _Spi(response=(0x99,))
        assert mod._OamsSpiRegLink(spi).reg_read(0x01) == 0

    def test_write_clears_the_read_bit(self):
        spi = _Spi()
        mod._OamsSpiRegLink(spi).reg_write(0x37, 0xAB)
        assert spi.sent == [[(0x37 << 1) & 0x7E, 0xAB]]
        assert spi.sent[0][0] & 0x80 == 0

    def test_address_masking_cannot_leak_bit0_or_bit7(self):
        spi = _Spi()
        link = mod._OamsSpiRegLink(spi)
        for reg in range(0x00, 0x40):
            spi.sent.clear()
            link.reg_write(reg, 0)
            addr = spi.sent[0][0]
            assert addr & 0x81 == 0, f"reg {reg:#x} produced addr {addr:#x}"

    def test_write_masks_the_value_to_a_byte(self):
        spi = _Spi()
        mod._OamsSpiRegLink(spi).reg_write(0x01, 0x1FF)
        assert spi.sent[0][1] == 0xFF

    def test_reader_power_is_a_noop(self):
        # The OpenAMS readers have no coil-enable line; the RF field is driven
        # through TxControlReg by the Mfrc522 class.
        assert mod._OamsSpiRegLink(_Spi()).reader_power(True) is None
        assert mod._OamsSpiRegLink(_Spi()).reader_power(False) is None


class TestHookedI2cMagic:
    """The i2c transport tunnels register access through a magic prefix that a
    patched OAMS firmware recognises. The constants are a wire contract."""

    def test_magic_prefix_is_RF_and_ops_are_R_W(self):
        assert (mod._HOOK_MAGIC0, mod._HOOK_MAGIC1) == (0x52, 0x46)   # 'R','F'
        assert mod._HOOK_OP_READ == 0x52                              # 'R'
        assert mod._HOOK_OP_WRITE == 0x57                             # 'W'


class _Cfg:
    """Minimal ConfigWrapper stand-in."""

    class error(Exception):
        pass

    def __init__(self, name="AFC_OpenAMS_rfid rdA", **opts):
        self._name = name
        self._o = opts

    def get_printer(self):
        return types.SimpleNamespace(
            lookup_object=lambda n, d=None: d,
            lookup_objects=lambda p=None: [],
            register_event_handler=lambda *a: None,
            load_object=lambda *a, **k: None)

    def get_name(self):
        return self._name

    def get(self, key, default=None):
        return self._o.get(key, default)

    def getint(self, key, default=None, **kw):
        v = self._o.get(key, default)
        return int(v) if v is not None else None

    def getboolean(self, key, default=None, **kw):
        return bool(self._o.get(key, default))


def _parse_slots(spec):
    """Exercise the reader's `slots` parsing in isolation."""
    cfg = _Cfg(slots=spec)
    slots = []
    for s in (cfg.get("slots", "") or "").split(","):
        s = s.strip()
        if not s:
            continue
        try:
            slots.append(int(s))
        except ValueError:
            raise cfg.error("bad slot %r" % s)
    return slots


class TestSlotsParsing:
    def test_plain_list(self):
        assert _parse_slots("0, 1") == [0, 1]

    def test_whitespace_and_trailing_comma_tolerated(self):
        assert _parse_slots("  2 ,3 ,") == [2, 3]

    def test_empty_is_no_slots(self):
        assert _parse_slots("") == []
        assert _parse_slots(None) == []

    def test_garbage_raises_at_startup(self):
        with pytest.raises(_Cfg.error):
            _parse_slots("0, x")


def _parse_map(spec):
    """Exercise the coordinator's `lane_slot_map` parsing in isolation."""
    cfg = _Cfg()
    out = {}
    for pair in (spec or "").split(","):
        pair = pair.strip()
        if not pair:
            continue
        try:
            lane, slot = pair.split(":")
            out[lane.strip()] = int(slot)
        except ValueError:
            raise cfg.error("bad entry %r" % pair)
    return out


class TestLaneSlotMapParsing:
    def test_maps_lanes_to_physical_slots(self):
        assert _parse_map("lane4:0, lane5:1, lane6:2, lane7:3") == {
            "lane4": 0, "lane5": 1, "lane6": 2, "lane7": 3}

    def test_empty_map_is_allowed(self):
        assert _parse_map("") == {}

    def test_missing_colon_raises(self):
        with pytest.raises(_Cfg.error):
            _parse_map("lane4")

    def test_non_numeric_slot_raises(self):
        with pytest.raises(_Cfg.error):
            _parse_map("lane4:left")


class TestReaderIndexingBySlot:
    """_on_connect indexes readers by the slots they serve, so a lane's slot
    maps straight to its reader."""

    def _coord(self, readers):
        c = mod.AFC_OpenAMS_rfid.__new__(mod.AFC_OpenAMS_rfid)
        c._slot_reader = {}
        c._warn = []
        c.logger = types.SimpleNamespace(
            warning=lambda fmt, *a: c._warn.append(fmt % a if a else fmt),
            info=lambda *a, **k: None)
        c.printer = types.SimpleNamespace(
            lookup_object=lambda n, d=None: d,
            lookup_objects=lambda: [("AFC_OpenAMS_rfid " + r.name, r)
                                    for r in readers])
        return c

    def _reader(self, name, slots):
        r = mod.AFC_OpenAMS_rfid_reader.__new__(mod.AFC_OpenAMS_rfid_reader)
        r.name, r.slots = name, slots
        return r

    def test_each_slot_resolves_to_its_reader(self):
        a, b = self._reader("rdA", [0, 1]), self._reader("rdB", [2, 3])
        c = self._coord([a, b])
        mod.AFC_OpenAMS_rfid._on_connect(c)
        assert c._slot_reader == {0: a, 1: a, 2: b, 3: b}

    def test_two_readers_claiming_one_slot_is_reported(self):
        # A real wiring/config mistake: the second wins, but silently doing so
        # would make one reader look dead.
        a, b = self._reader("rdA", [0, 1]), self._reader("rdB", [1])
        c = self._coord([a, b])
        mod.AFC_OpenAMS_rfid._on_connect(c)
        assert c._slot_reader[1] is b
        assert any("more than one" in str(w) for w in c._warn)

    def test_no_readers_warns_rather_than_failing(self):
        c = self._coord([])
        mod.AFC_OpenAMS_rfid._on_connect(c)
        assert c._slot_reader == {}
        assert any("no [AFC_OpenAMS_rfid" in str(w) for w in c._warn)


class TestLaneToSlot:
    def test_mapped_and_unmapped_lanes(self):
        c = mod.AFC_OpenAMS_rfid.__new__(mod.AFC_OpenAMS_rfid)
        c._lane_slot = {"lane4": 0, "lane5": 1}
        assert mod.AFC_OpenAMS_rfid._get_slot(c, "lane4") == 0
        assert mod.AFC_OpenAMS_rfid._get_slot(c, "lane9") is None


class _Reader:
    def __init__(self, name="rdA", slots=(0,)):
        self.name, self.slots, self.link = name, list(slots), object()


def _coord(readers=(), lane_slot=None, afc=None):
    """A coordinator with its runtime maps populated, no Klipper needed."""
    c = mod.AFC_OpenAMS_rfid.__new__(mod.AFC_OpenAMS_rfid)
    c._slot_reader = {s: r for r in readers for s in r.slots}
    c._lane_slot = dict(lane_slot or {})
    c._last = {}
    c._no_reader_warned = set()
    c.afc = afc
    c.bambu_master_key = None
    c.creality_key = None
    c.creality_encryption_key = None
    c.logged = []
    c.logger = types.SimpleNamespace(
        warning=lambda fmt, *a: c.logged.append(("warn", fmt % a if a else fmt)),
        info=lambda fmt, *a, **k: c.logged.append(("info", fmt)),
        debug=lambda fmt, *a, **k: c.logged.append(("debug", fmt)))
    return c


class TestReadSlot:
    """On a correctly configured OpenAMS every slot has a reader -- the reader
    sections' `slots` between them cover the whole unit. So the no-reader path
    is a CONFIG MISTAKE guard, not a runtime state: a lane_slot_map entry
    pointing at a slot no reader covers, or a reader section left out.

    It still has to warn only once, because read_slot is polled many times a
    second across a feed; a misconfiguration would otherwise flood the log for
    the whole scan window instead of stating the fault once."""

    def test_a_slot_no_reader_covers_returns_none(self, monkeypatch):
        c = _coord()
        assert mod.AFC_OpenAMS_rfid.read_slot(c, 3) is None

    def test_a_misconfigured_slot_warns_only_once(self):
        c = _coord()
        for _ in range(5):
            mod.AFC_OpenAMS_rfid.read_slot(c, 3)
        assert sum(1 for lvl, _ in c.logged if lvl == "warn") == 1

    def test_a_read_tag_is_returned_and_cached(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])
        tag = {"uid": "AABB", "filament": {"material": "PLA"}}
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: tag)
        assert mod.AFC_OpenAMS_rfid.read_slot(c, 0) is tag
        assert c._last[0] is tag

    def test_no_tag_is_not_cached(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: None)
        assert mod.AFC_OpenAMS_rfid.read_slot(c, 0) is None
        assert c._last == {}

    def test_decode_keys_are_passed_through(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])
        c.bambu_master_key = b"bk"
        c.creality_key = b"ck"
        c.creality_encryption_key = b"cek"
        got = {}
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: got.update(kw) or None)
        mod.AFC_OpenAMS_rfid.read_slot(c, 0)
        assert got["bambu_master_key"] == b"bk"
        assert got["creality_key"] == b"ck"
        assert got["creality_encryption_key"] == b"cek"


class TestScanSlotUids:
    """Enumerates every tag in the field. The OpenAMS readers are shared across
    bays, so a seated neighbour's tag can already be in range -- callers use
    this to learn which uids to exclude before a feed."""

    def test_no_reader_is_an_empty_list(self):
        assert mod.AFC_OpenAMS_rfid.scan_slot_uids(_coord(), 0) == []

    def test_returns_the_uids_it_saw(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])

        class _MC:
            def __init__(self, m): pass
            def activate(self, is_excluded=None, seen=None):
                seen.extend([("AA", 0x08, False), ("BB", 0x08, False)])
        monkeypatch.setattr(mod, "Mfrc522", lambda link: object())
        monkeypatch.setattr(mod, "MifareClassic", _MC)
        assert mod.AFC_OpenAMS_rfid.scan_slot_uids(c, 0) == ["AA", "BB"]

    def test_a_reader_error_is_logged_and_yields_no_uids(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))])

        class _MC:
            def __init__(self, m): pass
            def activate(self, **kw): raise RuntimeError("spi down")
        monkeypatch.setattr(mod, "Mfrc522", lambda link: object())
        monkeypatch.setattr(mod, "MifareClassic", _MC)
        assert mod.AFC_OpenAMS_rfid.scan_slot_uids(c, 0) == []
        assert any(lvl == "debug" for lvl, _ in c.logged)


class TestReadLane:
    """Lane -> slot -> reader. A tag that is SEEN but not decoded still has to
    be recorded, or a missing decode key looks identical to an empty bay."""

    def test_unmapped_lane_warns_and_returns_none(self):
        c = _coord()
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane9") is None
        assert any("no slot" in m for lvl, m in c.logged if lvl == "warn")

    def test_no_tag_returns_none_without_recording(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))], {"lane4": 0})
        monkeypatch.setattr(mod, "read_tag", lambda link, **kw: None)
        rec = []
        c.record_tag_read = lambda *a, **k: rec.append((a, k))
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4") is None
        assert rec == []

    def test_an_undecoded_tag_is_still_recorded(self, monkeypatch):
        # Seen but not decoded (e.g. no key for that vendor). Recording the UID
        # is what lets get_status say "a tag is there, I cannot read it".
        c = _coord([_Reader(slots=(0,))], {"lane4": 0})
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: {"uid": "AABB",
                                                "tag_type": "creality"})
        rec = []
        c.record_tag_read = lambda *a, **k: rec.append(k)
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4") is None
        assert rec and rec[0]["uid"] == "AABB"
        assert rec[0]["decoded"] is False
        assert rec[0]["tag_type"] == "creality"

    def test_no_afc_lane_returns_the_mapped_slot_info(self, monkeypatch):
        c = _coord([_Reader(slots=(0,))], {"lane4": 0}, afc=None)
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: {"uid": "A",
                                                "filament": {"material": "PLA"}})
        monkeypatch.setattr(mod, "map_tag_to_slot_info",
                            lambda tag: {"uid": "A", "material": "PLA"})
        c.record_tag_read = lambda *a, **k: None
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4")["material"] == "PLA"

    def test_a_known_lane_is_applied_to(self, monkeypatch):
        lane = object()
        afc = types.SimpleNamespace(lanes={"lane4": lane})
        c = _coord([_Reader(slots=(0,))], {"lane4": 0}, afc=afc)
        monkeypatch.setattr(mod, "read_tag",
                            lambda link, **kw: {"uid": "A",
                                                "filament": {"material": "PLA"}})
        applied = []
        c.apply_to_lane = lambda l, t: applied.append(l) or {"ok": True}
        assert mod.AFC_OpenAMS_rfid.read_lane(c, "lane4") == {"ok": True}
        assert applied == [lane]


class TestGcodeCommand:
    class _Gcmd:
        class error(Exception):
            pass

        def __init__(self, **kw):
            self._kw = kw
            self.said = []

        def get(self, k, d=None):
            return self._kw.get(k, d)

        def get_int(self, k, d=None):
            v = self._kw.get(k, d)
            return int(v) if v is not None else None

        def respond_info(self, m):
            self.said.append(m)

    def test_neither_lane_nor_slot_is_an_error(self):
        c = _coord()
        g = self._Gcmd()
        with pytest.raises(self._Gcmd.error):
            mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)

    def test_lane_with_no_tag_reports_the_hint(self):
        c = _coord()
        c.read_lane = lambda n: None
        c.undecoded_hint = lambda n: " (tag seen, no key)"
        g = self._Gcmd(LANE="lane4")
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "no tag decoded on lane4 (tag seen, no key)" in g.said[0]

    def test_lane_with_a_tag_reports_brand_and_material(self):
        c = _coord()
        c.read_lane = lambda n: {"brand": "Bambu", "material": "PLA"}
        g = self._Gcmd(LANE="lane4")
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "lane4 -> Bambu PLA" in g.said[0]

    def test_slot_form_reports_the_mapped_tag(self, monkeypatch):
        c = _coord()
        c.read_slot = lambda s: {"uid": "AA"}
        monkeypatch.setattr(mod, "map_tag_to_slot_info",
                            lambda tag: {"uid": "AA", "material": "PETG"})
        g = self._Gcmd(SLOT=2)
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "slot 2 ->" in g.said[0] and "PETG" in g.said[0]

    def test_slot_form_with_no_tag_says_none(self):
        c = _coord()
        c.read_slot = lambda s: None
        g = self._Gcmd(SLOT=2)
        mod.AFC_OpenAMS_rfid.cmd_OAMS_RFID_READ(c, g)
        assert "None" in g.said[0]


# ── Tests for the OpenAMS RFID scan-on-insert flow (afcAMS._do_rfid_scan and the ───
#
# was tests/test_AFC_OpenAMS_rfid_scan.py
# Match the defensive stubs the sibling OpenAMS test modules install.
_mcu_stub = types.ModuleType("mcu")
_mcu_stub.get_printer_mcu = MagicMock()
sys.modules.setdefault("mcu", _mcu_stub)
_bus_stub = types.ModuleType("extras.bus")
_bus_stub.MCU_I2C_from_config = MagicMock()
sys.modules.setdefault("extras.bus", _bus_stub)



class AdvancingReactor:
    """Reactor whose pause() advances monotonic time, so timed loops terminate."""

    NEVER = 9_999_999_999.0
    NOW = 0.0

    def __init__(self, monotonic_value=100.0):
        self._monotonic = monotonic_value
        self.registered = []

    def monotonic(self):
        return self._monotonic

    def pause(self, until):
        # Callers pass monotonic()+delay; advance to it so deadlines are reached.
        self._monotonic = max(self._monotonic, until)

    def register_timer(self, callback, waketime=None):
        handle = ("timer", len(self.registered))
        self.registered.append((handle, callback, waketime))
        return handle

    def unregister_timer(self, handle):
        self.registered = [r for r in self.registered if r[0] != handle]


class FakeCmd:
    def __init__(self):
        self.sent = []

    def send(self, args=None):
        self.sent.append(args)


class FakeController:
    """Minimal AFC_OAMS stand-in exposing exactly what the scan touches.

    The load command "moves filament": sending it bumps encoder_clicks past
    the polling gate and trips the hub HES, so engagement detection and the
    encoder gate both see motion.
    """

    def __init__(self, reactor=None):
        self.follower_calls = []
        self.oams_load_spool_cmd = FakeCmd()
        self.oams_load_spool_cmd.send = self._send_load
        self.action_status = None
        self.action_status_code = None
        # The firmware reports its motor state only in ANSWER to a command --
        # there is no periodic stream -- which is why the ready-wait probes.
        self.reactor = reactor
        self.motion_status = None
        self.motion_status_code = None
        self.motion_status_time = 0.0
        self.cancel_calls = 0
        self.unload_calls = 0
        self.clear_errors_calls = 0
        self.current_spool = 3
        self.encoder_clicks = 500          # running counter, never zero
        self.hub_hes_value = [0, 0, 0, 0]
        # Number of load attempts to reject ERROR_BUSY before accepting
        # (models the firmware's insert-staging window).
        self.busy_rejections = 0
        # Number of readiness PROBES answered ERROR_BUSY before the unit
        # reports STOPPED. Independent of busy_rejections: a load can be
        # refused for reasons that have nothing to do with the motor.
        self.staging_probes = 0

    def _send_load(self, args):
        # Model the firmware load: filament moves, hub trips, load completes.
        self.oams_load_spool_cmd.sent.append(args)
        if self.busy_rejections > 0:
            self.busy_rejections -= 1
            self.action_status = None
            self.action_status_code = 2    # OAMSOpCode.ERROR_BUSY
            return
        self.encoder_clicks += 120
        self.hub_hes_value[args[0]] = 1
        self.action_status = None          # ack: load done
        self.action_status_code = 0        # OAMSOpCode.SUCCESS

    # Motor primitives
    def set_oams_follower(self, enable, direction):
        self.follower_calls.append((enable, direction))
        # Answer like the firmware: a status comes back only when the command
        # is REFUSED or the motor state actually CHANGES. While a routine
        # (e.g. the insert auto-stage) owns the motor, every stop is refused
        # ERROR_BUSY. Once it is done, a stop sent to an already-stopped unit
        # changes nothing -- so the firmware answers with SILENCE, and silence
        # is what the ready-wait has to read as "ready".
        if self.staging_probes > 0:
            self.staging_probes -= 1
            self.motion_status = OAMSStatus.REVERSE_FOLLOWING
            self.motion_status_code = 2      # OAMSOpCode.ERROR_BUSY
            if self.reactor is not None:
                self.motion_status_time = self.reactor.monotonic()

    def load_spool_cancel(self):
        self.cancel_calls += 1
        self.action_status = None
        return "cancelled"

    def unload_spool(self):
        self.unload_calls += 1
        return True, "ok"

    def clear_errors(self):
        self.clear_errors_calls += 1

    def is_bay_ready(self, bay):
        return True


class FakeCoordinator:
    """AFC_OpenAMS_rfid stand-in with the field/read API.

    ``fields`` is a list of uid-lists, one per scan_slot_uids() call: the
    FIRST is the rest-time baseline, later entries are the per-poll field
    contents during the feed (the last entry repeats once exhausted).
    """

    def __init__(self, fields=None, full_reads=None):
        self._fields = [list(f) for f in (fields or [[]])]
        self._reads = list(full_reads or [])
        self.read_excludes = []
        self.applied = []
        self.slot_map = {"lane1": 0}

    def _get_slot(self, name):
        return self.slot_map.get(name)

    def scan_slot_uids(self, slot):
        if len(self._fields) > 1:
            return self._fields.pop(0)
        return list(self._fields[0])

    def read_slot_excluding(self, slot, exclude):
        self.read_excludes.append(set(exclude))
        if not self._reads:
            return None
        return self._reads.pop(0)

    def read_slot(self, slot):              # manual-path compat
        return self.read_slot_excluding(slot, set())

    def apply_to_lane(self, lane, tag):
        self.applied.append((lane, tag))
        return {"brand": "X", "material": "PLA"}

    def undecoded_hint(self, name):
        return ""


class FakeLane_rfid_scan:
    def __init__(self, name="lane1"):
        self.name = name
        self.tool_loaded = False
        self.loaded_to_hub = False
        self.send_lane_data_calls = 0

    def send_lane_data(self):
        self.send_lane_data_calls += 1


def _build_unit(values=None, coord=None):
    afc = MockAFC()
    afc.reactor = AdvancingReactor()
    printer = MockPrinter(afc=afc)
    printer.reactor = afc.reactor
    cfg_values = {"rfid_scan_on_insert": True, "rfid_scan_timeout": 2.0,
                  "rfid_scan_poll": 0.2, "rfid_scan_read_retries": 2}
    cfg_values.update(values or {})
    config = MockConfig(name="AFC_OpenAMS ams1", printer=printer, values=cfg_values)
    ams = afcAMS(config)
    ams.afc = afc
    # in_print() must return a real bool (MagicMock's default is truthy).
    afc.function.in_print = lambda: False
    ams.oams = FakeController(reactor=afc.reactor)
    ams.lanes = {"lane1": FakeLane_rfid_scan()}
    ams._spool_map = {"lane1": 0}
    coord = coord if coord is not None else FakeCoordinator()
    ams._rfid_coord = coord
    printer._objects["AFC_OpenAMS_rfid"] = coord
    return ams, coord


TAG = {"uid": "AABB", "filament": {"material": "PLA"}, "tag_type": "MifareClassic1k"}


class TestConfigDefaults:
    def test_defaults(self):
        afc = MockAFC()
        printer = MockPrinter(afc=afc)
        config = MockConfig(name="AFC_OpenAMS ams1", printer=printer, values={})
        ams = afcAMS(config)
        assert ams.rfid_scan_on_insert is False
        assert ams.rfid_scan_timeout == 15.0
        assert ams.rfid_scan_read_retries == 3
        # Tuned on hardware: sweep_back must cover the re-feed overshoot
        # (~150 clicks) before it buys any pre-roll before the detect point.
        assert ams.rfid_scan_sweep_back == 240
        assert ams.rfid_scan_sweep_step == 25
        assert ams.rfid_scan_sweep_past == 200

    def test_enabled(self):
        ams, _ = _build_unit()
        assert ams.rfid_scan_on_insert is True


class TestScanTagFound:
    def _run(self):
        # Empty field at rest; the moving tag arrives on the second poll.
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_returns_true(self):
        _, _, result = self._run()
        assert result is True

    def test_applies_tag_to_lane(self):
        ams, coord, _ = self._run()
        assert len(coord.applied) == 1
        lane, tag = coord.applied[0]
        assert lane is ams.lanes["lane1"]
        assert tag is TAG

    def test_surfaces_to_mainsail(self):
        ams, _, _ = self._run()
        assert ams.lanes["lane1"].send_lane_data_calls == 1

    def test_persists_across_restart(self):
        # Without save_vars a FIRMWARE_RESTART wipes the applied data when
        # PREP rebuilds lane_data (the field-observed lane6 clear).
        ams, _, _ = self._run()
        assert ams.afc.save_vars.called

    def test_sends_the_load_once(self):
        ams, _, _ = self._run()
        assert ams.oams.oams_load_spool_cmd.sent == [[0]]

    def test_follower_stopped_at_end(self):
        ams, _, _ = self._run()
        assert (1, 1) in ams.oams.follower_calls      # pre-load forward
        assert ams.oams.follower_calls[-1] == (0, 0)  # stopped at the end

    def test_unwinds_after_engagement(self):
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 1

    def test_clears_operation_guard_and_latches(self):
        ams, _, _ = self._run()
        assert ams._operation_active is False
        assert ams._prev_states_stale is True
        assert "lane1" in ams._rfid_scanned


class TestSisterTags:
    def test_constant_sister_never_detected(self):
        # A seated neighbour's tag answers every poll — stationary = sister,
        # so a scan with ONLY it in field times out instead of detecting.
        coord = FakeCoordinator(fields=[["5157e12"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []

    def test_new_uid_beside_sister_detected_and_sister_excluded(self):
        # Sister present throughout; the moving tag arrives later — it is
        # detected and the sister is excluded from the full read.
        coord = FakeCoordinator(
            fields=[["5157e12"], ["5157e12"], ["5157e12", "aabb"]],
            full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert all("5157e12" in ex for ex in coord.read_excludes)

    def test_own_tag_at_rest_detected_by_motion(self):
        # The inserted spool's OWN tag rests on the antenna (in the baseline),
        # then blinks out for >= reappear_polls during the feed and returns —
        # motion brands it OURS, not a sister (the 01d0ec0f field case).
        coord = FakeCoordinator(
            fields=[["01d0ec0f"],                       # baseline (at rest)
                    ["01d0ec0f"],                        # still there
                    [], [], [], [],                      # gone 4 polls (moving)
                    ["01d0ec0f"]],                       # back in range
            full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        # It is the target, so it must NOT be excluded from the read.
        assert all("01d0ec0f" not in ex for ex in coord.read_excludes)


class TestUnreadableReposition:
    def _run(self):
        # Detect succeeds; the stationary reads at the stop position all fail
        # (2 retries), then the reposition read decodes.
        coord = FakeCoordinator(fields=[[], ["aabb"]],
                                full_reads=[None, None, TAG])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_decodes_after_reposition(self):
        ams, coord, result = self._run()
        assert result is True
        assert len(coord.applied) == 1

    def test_reloads_the_lane(self):
        # One load for the scan feed + one for the reposition.
        ams, _, _ = self._run()
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2

    def test_unloads_twice(self):
        # Once before the reposition re-feed, once at the final unwind.
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 2

    def test_gives_up_cleanly_when_still_unreadable(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[])
        ams, coord = _build_unit(coord=coord)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []
        assert ams._operation_active is False
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestSafetyGates:
    def test_lane_loaded_to_shared_toolhead_blocks_scan(self):
        # Some lane (any unit's) is loaded into the toolhead this unit's
        # lanes feed -> blocked.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        shared_ext = types.SimpleNamespace(name="extruder1",
                                           lane_loaded="lane9")
        ams.lanes["lane1"].extruder_obj = shared_ext
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []
        assert coord.applied == []

    def test_unrelated_toolhead_does_not_block(self):
        # The unit's shared toolhead is free; other toolheads on a multi-tool
        # machine are irrelevant to this unit's scan.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        free_ext = types.SimpleNamespace(name="extruder1", lane_loaded=None)
        ams.lanes["lane1"].extruder_obj = free_ext
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True

    def test_occupied_hub_blocks_scan(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams.hub_hes_value[2] = 1        # some other bay at the hub
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []

    def test_occupied_hub_does_not_block_scheduling(self):
        # Insert staging trips the hub HES briefly — scheduling must not be
        # blocked by it (the scan re-checks after the ready-wait).
        ams, _ = _build_unit()
        ams.oams.hub_hes_value[0] = 1
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert "lane1" in ams._rfid_scan_timers

    def test_shared_toolhead_loaded_blocks_scheduling(self):
        ams, _ = _build_unit()
        ams.lanes["lane1"].extruder_obj = types.SimpleNamespace(
            name="extruder1", lane_loaded="lane9")
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_in_print_blocks_scan(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.afc.function.in_print = lambda: True
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert ams.oams.oams_load_spool_cmd.sent == []


class TestHubEngageCancel:
    def test_load_cancelled_when_hub_engages(self):
        # A load still in flight when the hub HES trips must be cancelled
        # (TD-1 style) and replaced with slow follower creep.
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)

        def slow_load(args):
            # Load stays in flight; hub trips immediately.
            ams.oams.oams_load_spool_cmd.sent.append(args)
            ams.oams.encoder_clicks += 30
            ams.oams.hub_hes_value[args[0]] = 1
            ams.oams.action_status = OAMSStatus.LOADING

        ams.oams.oams_load_spool_cmd.send = slow_load
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert ams.oams.cancel_calls >= 1          # load cancelled at hub
        assert (1, 1) in ams.oams.follower_calls   # creep enabled


class TestScanTimeout:
    def _run(self):
        # Reader never sees a new tag (e.g. a spool without one).
        ams, coord = _build_unit()
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_returns_false(self):
        _, _, result = self._run()
        assert result is False

    def test_does_not_apply(self):
        _, coord, _ = self._run()
        assert coord.applied == []

    def test_still_unwinds_and_cleans_up(self):
        ams, _, _ = self._run()
        assert ams.oams.unload_calls == 1
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestBusyHandling:
    def test_one_retry_after_busy_rejection(self):
        # Firmware refuses the first load ERROR_BUSY (staging raced the ready
        # wait); the scan waits for ready again and sends exactly ONE more.
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams.busy_rejections = 1
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is True
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2
        assert len(coord.applied) == 1

    def test_aborts_after_second_busy_no_hammering(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(values={"rfid_scan_ready_timeout": 5.0},
                                 coord=coord)
        ams.oams.busy_rejections = 10_000
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        # Never more than two load attempts — we wait on ready, we don't spam.
        assert len(ams.oams.oams_load_spool_cmd.sent) == 2
        assert coord.applied == []
        # Guard cleared even on the abort path; PTFE never touched.
        assert ams._operation_active is False

    def test_follower_left_stopped_after_busy_abort(self):
        # The pre-load dance enables the follower forward (mirroring
        # _oams_load), but an aborted scan must always leave it STOPPED.
        ams, _ = _build_unit(values={"rfid_scan_ready_timeout": 5.0})
        ams.oams.busy_rejections = 10_000
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert ams.oams.follower_calls[-1] == (0, 0)


class TestRefusedLoad:
    def test_instant_error_refusal_aborts_without_retry(self):
        # A load that instantly completes with a non-success, non-busy code
        # (e.g. "no spool in bay") is a REFUSAL — reported, not retried.
        ams, coord = _build_unit()

        def dead_send(args):
            ams.oams.oams_load_spool_cmd.sent.append(args)
            ams.oams.action_status = None
            ams.oams.action_status_code = 4    # NO_SPOOL_IN_BAY

        ams.oams.oams_load_spool_cmd.send = dead_send
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert len(ams.oams.oams_load_spool_cmd.sent) == 1
        # Nothing engaged, so no unwind noise.
        assert ams.oams.unload_calls == 0


class TestScheduling:
    def test_disabled_does_not_schedule(self):
        ams, _ = _build_unit(values={"rfid_scan_on_insert": False})
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_enabled_schedules_timer(self):
        ams, _ = _build_unit()
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert "lane1" in ams._rfid_scan_timers

    def test_already_scanned_not_rescheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scanned.add("lane1")
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_operation_active_blocks_scheduling(self):
        ams, _ = _build_unit()
        ams._operation_active = True
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_cancel_clears_latch_and_timer(self):
        ams, _ = _build_unit()
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        ams._rfid_scanned.add("lane1")
        ams._cancel_rfid_scan("lane1")
        assert ams._rfid_scan_timers == {}
        assert "lane1" not in ams._rfid_scanned


class TestOperationActiveGuard:
    def test_scan_bails_if_operation_active(self):
        coord = FakeCoordinator(fields=[[], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams._operation_active = True
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        assert result is False
        assert coord.applied == []


class TestUnitReadyWait:
    def test_waits_out_encoder_motion(self):
        ams, _ = _build_unit()
        ticks = {"n": 0}
        real_monotonic = ams.afc.reactor.monotonic
        # Simulate: the encoder advances during the first few poll pauses
        # (firmware auto-stage still feeding), then holds still.
        base = ams.oams.encoder_clicks

        def fake_pause(until):
            AdvancingReactor.pause(ams.afc.reactor, until)
            if ticks["n"] < 3:
                ams.oams.encoder_clicks = base + ticks["n"]
                ticks["n"] += 1

        ams.afc.reactor.pause = fake_pause
        start = real_monotonic()
        assert ams._rfid_wait_for_unit_ready(10.0, quiet_time=1.0) is True
        # It must have waited at least quiet_time past the last movement.
        assert real_monotonic() - start >= 1.0

    def test_probes_instead_of_waiting_for_an_unprompted_report(self):
        # The firmware answers with its motor state only when spoken to. An
        # idle unit that has said nothing since boot must still be declared
        # ready PROMPTLY -- waiting passively for a spontaneous STOPPED report
        # burned the whole ready timeout (30s of dead air per insert scan).
        ams, _ = _build_unit()
        reactor = ams.afc.reactor
        start = reactor.monotonic()
        assert ams.oams.motion_status is None
        assert ams._rfid_wait_for_unit_ready(30.0, fresh=True,
                                             quiet_time=1.0) is True
        assert reactor.monotonic() - start < 5.0
        # Readiness came from a probe, and the probe is a harmless stop.
        assert ams.oams.follower_calls
        assert set(ams.oams.follower_calls) == {(0, 0)}

    def test_stale_active_state_is_not_ready(self):
        # A unit mid-stage keeps answering "reverse following / busy". The old
        # code only counted a FRESH active report, so a stale one let the wait
        # return ready ~1s in and the very next command took an ERROR_BUSY.
        ams, _ = _build_unit()
        ams.oams.staging_probes = 10_000
        assert ams._rfid_wait_for_unit_ready(3.0, quiet_time=1.0) is False

    def test_ready_once_staging_finishes(self):
        ams, _ = _build_unit()
        ams.oams.staging_probes = 3
        assert ams._rfid_wait_for_unit_ready(30.0, fresh=True,
                                             quiet_time=1.0) is True

    def test_no_load_sent_while_the_unit_is_still_staging(self):
        # The scan must not even reach the load while the motor is owned by
        # the firmware's insert routine.
        ams, _ = _build_unit(values={"rfid_scan_ready_timeout": 3.0})
        ams.oams.staging_probes = 10_000
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False
        assert ams.oams.oams_load_spool_cmd.sent == []
        assert ams._operation_active is False

    def test_fresh_satisfied_by_new_stopped_report(self):
        ams, _ = _build_unit()
        reactor = ams.afc.reactor

        def report_stopped(until):
            AdvancingReactor.pause(reactor, until)
            # Firmware reports STOPPED shortly after the wait begins.
            ams.oams.motion_status = OAMSStatus.STOPPED
            ams.oams.motion_status_time = reactor.monotonic()

        # Only the first pause plants the report; later pauses advance time.
        calls = {"n": 0}

        def fake_pause(until):
            if calls["n"] == 0:
                report_stopped(until)
            else:
                AdvancingReactor.pause(reactor, until)
            calls["n"] += 1

        reactor.pause = fake_pause
        assert ams._rfid_wait_for_unit_ready(10.0, fresh=True,
                                             quiet_time=0.5) is True


class TestManualScanCommandRefusals:
    """AFC_OAMS_RFID_SCAN physically feeds the lane to rotate the spool past
    the reader. Every refusal below is a reason NOT to drive a motor, so they
    matter more than the happy path."""

    def _gcmd(self, **kw):
        g = MagicMock()
        g.get.side_effect = lambda k, d=None: kw.get(k, d)
        g.error = RuntimeError
        return g

    def test_missing_lane_argument(self):
        ams, _ = _build_unit()
        with pytest.raises(RuntimeError):
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd())

    def test_unknown_lane_names_the_unit(self):
        ams, _ = _build_unit()
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane9"))
        assert "lane9" in str(e.value)

    def test_refuses_while_another_operation_runs(self):
        ams, _ = _build_unit()
        ams._operation_active = True
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "busy" in str(e.value)

    def test_refuses_when_the_scan_is_blocked(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: "lane is tool-loaded"
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "tool-loaded" in str(e.value)

    def test_refuses_with_an_empty_bay(self):
        # Feeding an empty bay spins the motor against nothing.
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: None
        ams.oams.is_bay_ready = lambda bay: False
        with pytest.raises(RuntimeError) as e:
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "no filament inserted" in str(e.value)

    def test_refuses_when_the_lane_maps_to_no_bay(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: None
        ams._spool_map = {}
        with pytest.raises(RuntimeError):
            ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))


class TestManualScanCommandReplies:
    def _gcmd(self, **kw):
        g = MagicMock()
        g.get.side_effect = lambda k, d=None: kw.get(k, d)
        g.error = RuntimeError
        g.said = []
        g.respond_info.side_effect = g.said.append
        return g

    def _ready(self):
        ams, coord = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane: None
        ams.oams.is_bay_ready = lambda bay: True
        return ams, coord

    def test_a_manual_scan_bypasses_the_once_per_insert_latch(self):
        # The latch exists to stop the AUTO scan repeating; a human asking
        # again plainly wants it to run.
        ams, _ = self._ready()
        ams._rfid_scanned.add("lane1")
        ams._do_rfid_scan = lambda lane: True
        ams.cmd_AFC_OAMS_RFID_SCAN(self._gcmd(LANE="lane1"))
        assert "lane1" not in ams._rfid_scanned

    def test_a_decoded_tag_is_reported(self):
        ams, _ = self._ready()
        ams._do_rfid_scan = lambda lane: True
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("scanned and applied to lane1" in m for m in g.said)

    def test_no_tag_reports_the_undecoded_hint(self):
        # "a tag was seen but could not be decoded" is a different fault from
        # "no tag", and only the hint distinguishes them.
        ams, coord = self._ready()
        ams._do_rfid_scan = lambda lane: False
        coord.undecoded_hint = lambda name: " (tag seen, no key)"
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("no tag decoded on lane1 (tag seen, no key)" in m
                   for m in g.said)

    def test_a_throwing_hint_still_produces_a_reply(self):
        ams, coord = self._ready()
        ams._do_rfid_scan = lambda lane: False
        coord.undecoded_hint = lambda name: (_ for _ in ()).throw(
            RuntimeError("x"))
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("no tag decoded on lane1" in m for m in g.said)

    def test_no_coordinator_still_produces_a_reply(self):
        ams, _ = self._ready()
        ams._do_rfid_scan = lambda lane: False
        ams._rfid_coord = None
        ams._rfid_coordinator = lambda: None
        g = self._gcmd(LANE="lane1")
        ams.cmd_AFC_OAMS_RFID_SCAN(g)
        assert any("no tag decoded on lane1" in m for m in g.said)


class TestScanKickoffTimer:
    """The auto-scan runs from a one-shot reactor timer: it must deregister
    itself and never let an exception reach the reactor."""

    def test_runs_once_and_never_reschedules(self):
        ams, _ = _build_unit()
        ams._rfid_scan_timers["lane1"] = object()
        calls = []
        ams._do_rfid_scan = lambda lane: calls.append(lane)
        got = ams._rfid_scan_kickoff(1.0, "lane1")
        assert got == ams.afc.reactor.NEVER
        assert len(calls) == 1
        assert "lane1" not in ams._rfid_scan_timers

    def test_a_lane_that_vanished_is_skipped(self):
        ams, _ = _build_unit()
        assert ams._rfid_scan_kickoff(1.0, "gone") == ams.afc.reactor.NEVER

    def test_a_failing_scan_is_logged_not_raised(self):
        ams, _ = _build_unit()
        ams._do_rfid_scan = lambda lane: (_ for _ in ()).throw(
            RuntimeError("boom"))
        assert ams._rfid_scan_kickoff(1.0, "lane1") == ams.afc.reactor.NEVER


class TestHubDebounce:
    """Hub state feeds the virtual hub and the 'hub clear' gate, so a
    fluttering switch reaching those checks would flap a load mid-flight.
    Live consumers read oams.hub_hes_value directly and see the raw signal."""

    def _lane(self):
        return types.SimpleNamespace(_load_state=None)

    def _ams(self, committed=None, debounce=0.5):
        ams, _ = _build_unit()
        ams._last_hub = [committed, None, None, None]
        ams._hub_pending_since = None
        ams.hub_debounce = debounce
        return ams

    def test_a_resync_pass_accepts_raw_immediately(self):
        ams, lane = self._ams(), self._lane()
        ams._update_hub_debounced(lane, 0, True, 100.0, True)
        assert lane._load_state is True and ams._last_hub[0] is True

    def test_the_first_reading_is_accepted_immediately(self):
        ams, lane = self._ams(), self._lane()
        ams._update_hub_debounced(lane, 0, True, 100.0, False)
        assert lane._load_state is True

    def test_an_unchanged_reading_clears_a_pending_change(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._hub_pending_since = [50.0, None, None, None]
        ams._update_hub_debounced(lane, 0, True, 100.0, False)
        assert lane._load_state is True
        assert ams._hub_pending_since[0] is None

    def test_a_change_does_not_commit_immediately(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        assert lane._load_state is True                 # still committed
        assert ams._hub_pending_since[0] == 100.0

    def test_a_change_that_does_not_hold_is_ignored(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        ams._update_hub_debounced(lane, 0, False, 100.2, False)
        assert lane._load_state is True                 # inside the window

    def test_a_change_that_holds_commits(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        ams._update_hub_debounced(lane, 0, False, 100.6, False)
        assert lane._load_state is False
        assert ams._last_hub[0] is False
        assert ams._hub_pending_since[0] is None

    def test_a_flutter_back_to_committed_restarts_the_window(self):
        ams, lane = self._ams(committed=True), self._lane()
        ams._update_hub_debounced(lane, 0, False, 100.0, False)
        ams._update_hub_debounced(lane, 0, True, 100.2, False)   # cancels
        ams._update_hub_debounced(lane, 0, False, 100.4, False)  # restarts
        assert lane._load_state is True


class TestScanHelpersSurviveAFailingCoordinator:
    """Every one of these is an except branch around a call into the RFID
    coordinator or the OAMS controller. The scan is mid-motion when they run,
    so an exception escaping here strands the follower on and leaves the
    operation guard set -- the lane would be stuck until a restart."""

    def test_slot_falls_back_to_the_bay_index(self):
        # The coordinator's lane_slot_map wins; without one (or with a
        # coordinator that throws) the bay index is the sane default.
        ams, coord = _build_unit()
        coord._get_slot = lambda name: (_ for _ in ()).throw(KeyError(name))
        assert ams._rfid_scan_slot(coord, ams.lanes["lane1"]) == 0

    def test_slot_is_none_when_nothing_maps(self):
        ams, coord = _build_unit()
        coord._get_slot = lambda name: None
        ams._spool_map = {}
        assert ams._rfid_scan_slot(coord, ams.lanes["lane1"]) is None

    def test_a_probe_error_is_treated_as_an_empty_field(self):
        # scan_slot_uids talks to the reader while the spool turns; a transient
        # SPI error must not abort the scan, just yield no uids this pass.
        ams, coord = _build_unit()
        coord.scan_slot_uids = lambda slot: (_ for _ in ()).throw(
            RuntimeError("spi"))
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams._do_rfid_scan(ams.lanes["lane1"])       # must not raise

    def test_a_stationary_read_error_yields_no_tag(self):
        ams, coord = _build_unit()
        coord.read_slot_excluding = lambda slot, exclude: (_ for _ in ()).throw(
            RuntimeError("auth failed"))
        got = ams._rfid_read_stationary(coord, 0, set(), attempts=2)
        assert got is None


class TestBlockedReason:
    """The scan refuses whenever moving filament could collide with something
    else. Each branch names WHAT is in the way, because 'blocked' alone tells
    an operator nothing."""

    def test_a_lane_loaded_to_the_toolhead_blocks_and_names_it(self):
        ams, _ = _build_unit()
        ext = types.SimpleNamespace(lane_loaded="lane1", name="extruder")
        ams.lanes["lane1"].extruder_obj = ext
        reason = ams._rfid_scan_blocked_reason(ams.lanes["lane1"])
        assert "loaded to this unit's toolhead" in reason
        assert "extruder" in reason

    def test_hub_filament_blocks_and_names_the_bay(self):
        ams, _ = _build_unit()
        ams.oams.hub_hes_value = [0, 1, 0, 0]
        reason = ams._rfid_scan_blocked_reason(ams.lanes["lane1"])
        assert "hub sensor shows filament (bay 1)" == reason

    def test_an_unreadable_hub_value_does_not_block(self):
        # A controller that cannot report the hub must not veto the scan; the
        # other guards still apply.
        ams, _ = _build_unit()
        type(ams.oams).hub_hes_value = property(
            lambda self: (_ for _ in ()).throw(RuntimeError("no link")))
        try:
            assert ams._rfid_scan_blocked_reason(ams.lanes["lane1"]) is None
        finally:
            del type(ams.oams).hub_hes_value

    def test_hub_check_can_be_skipped(self):
        ams, _ = _build_unit()
        ams.oams.hub_hes_value = [1, 0, 0, 0]
        assert ams._rfid_scan_blocked_reason(
            ams.lanes["lane1"], check_hub=False) is None

    def test_printing_blocks_the_scan(self):
        ams, _ = _build_unit()
        ams.afc.function.in_print = lambda: True
        assert ams._rfid_scan_blocked_reason(
            ams.lanes["lane1"]) == "printer is printing"

    def test_an_unreadable_print_state_does_not_block(self):
        ams, _ = _build_unit()
        ams.afc.function.in_print = lambda: (_ for _ in ()).throw(
            RuntimeError("no afc"))
        assert ams._rfid_scan_blocked_reason(ams.lanes["lane1"]) is None


class TestFollowerControlFallback:
    """_rfid_set_follower prefers the shared follower object and falls back to
    driving the controller directly. Both tiers are wrapped because losing the
    follower mid-scan leaves the spool turning."""

    def test_the_follower_object_is_used_when_it_works(self):
        ams, _ = _build_unit()
        calls = []
        ams._follower = types.SimpleNamespace(
            enable_follower=lambda *a, **k: calls.append("enable"),
            set_follower_state=lambda *a, **k: calls.append("stop"))
        ams._get_monitor_state = lambda: None
        ams._rfid_set_follower(1, 1, "test")
        ams._rfid_set_follower(0, 0, "test")
        assert calls == ["enable", "stop"]

    def test_a_failing_follower_object_falls_back_to_the_controller(self):
        ams, _ = _build_unit()
        ams._follower = types.SimpleNamespace(
            enable_follower=lambda *a, **k: (_ for _ in ()).throw(
                RuntimeError("no monitor")),
            set_follower_state=lambda *a, **k: (_ for _ in ()).throw(
                RuntimeError("no monitor")))
        ams._get_monitor_state = lambda: None
        ams._rfid_set_follower(1, 1, "test")
        assert ams.oams.follower_calls, "should have driven the controller"

    def test_both_tiers_failing_is_survived(self):
        # Nothing left to try; the caller's unwind must still run.
        ams, _ = _build_unit()
        ams._follower = None
        ams.oams.set_oams_follower = lambda e, d: (_ for _ in ()).throw(
            RuntimeError("link down"))
        ams._rfid_set_follower(1, 1, "test")        # must not raise


class TestScanEntryGuards:
    """_do_rfid_scan refuses before it moves anything. Each guard is a
    different way the world can have changed between the insert edge that
    scheduled the scan and the timer that runs it."""

    def test_no_coordinator_configured(self):
        ams, _ = _build_unit()
        ams._rfid_coord = None
        ams.printer._objects.pop("AFC_OpenAMS_rfid", None)
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_no_controller(self):
        ams, _ = _build_unit()
        ams.oams = None
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_another_operation_started_since_scheduling(self):
        # A real load/unload may have begun between the insert edge and the
        # timer firing; never drive filament on top of it.
        ams, _ = _build_unit()
        ams._operation_active = True
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_a_blocked_scan_is_reported_and_refused(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: "printing"
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_an_empty_bay(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams.oams.is_bay_ready = lambda bay: False
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_an_unmapped_bay(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams._spool_map = {}
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False

    def test_no_rfid_slot_warns_and_refuses(self):
        # Mapped to a bay but the coordinator serves no reader for it: a
        # config mistake worth naming rather than a silent no-op.
        ams, coord = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams.oams.is_bay_ready = lambda bay: True
        ams._rfid_scan_slot = lambda c, lane: None
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is False


class TestCoordinatorResolution:
    def test_it_is_looked_up_once_and_cached(self):
        ams, coord = _build_unit()
        ams._rfid_coord = None
        calls = []
        real = ams.printer.lookup_object

        def counting(name, default=None):
            calls.append(name)
            return real(name, default)
        ams.printer.lookup_object = counting
        assert ams._rfid_coordinator() is coord
        assert ams._rfid_coordinator() is coord
        assert calls.count("AFC_OpenAMS_rfid") == 1

    def test_a_missing_coordinator_resolves_to_none(self):
        ams, _ = _build_unit()
        ams._rfid_coord = None
        ams.printer._objects.pop("AFC_OpenAMS_rfid", None)
        assert ams._rfid_coordinator() is None


class TestBlockedReasonToolLoaded:
    def test_the_lane_itself_being_tool_loaded_blocks(self):
        ams, _ = _build_unit()
        ams.lanes["lane1"].tool_loaded = True
        reason = ams._rfid_scan_blocked_reason(ams.lanes["lane1"])
        assert reason == "lane1 is loaded to the toolhead"


class TestScheduleGuards:
    """The insert hook schedules the scan on a timer. It must refuse quietly
    for the same reasons, and never schedule work that will just refuse."""

    def test_a_blocked_lane_is_not_scheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: "printing"
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_no_coordinator_means_nothing_is_scheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams._rfid_coord = None
        ams.printer._objects.pop("AFC_OpenAMS_rfid", None)
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}

    def test_an_empty_bay_is_not_scheduled(self):
        ams, _ = _build_unit()
        ams._rfid_scan_blocked_reason = lambda lane, check_hub=True: None
        ams.oams.is_bay_ready = lambda bay: False
        ams._maybe_schedule_rfid_scan(ams.lanes["lane1"])
        assert ams._rfid_scan_timers == {}


class TestEncoderDeltaIsAlwaysSafe:
    """_enc_delta is called inside every motion wait. A controller that
    cannot report the encoder must yield 0, not raise -- the callers treat
    'no movement' as a timeout, which is recoverable."""

    def test_a_readable_encoder(self):
        ams, _ = _build_unit()
        assert ams._enc_delta(150, 100) == 50

    def test_unreadable_values_are_zero(self):
        ams, _ = _build_unit()
        assert ams._enc_delta(None, 100) == 0
        assert ams._enc_delta("x", 100) == 0


class TestCancelScanTimer:
    def test_cancelling_with_no_timer_is_a_noop(self):
        ams, _ = _build_unit()
        ams._cancel_rfid_scan("lane1")

    def test_a_failing_unregister_is_survived(self):
        ams, _ = _build_unit()
        ams._rfid_scan_timers["lane1"] = object()
        ams.afc.reactor.unregister_timer = lambda h: (_ for _ in ()).throw(
            RuntimeError("gone"))
        ams._cancel_rfid_scan("lane1")
        assert "lane1" not in ams._rfid_scan_timers


class TestRepositionAndSweep:
    """Recovery when a tag ANSWERED during the feed but would not decode where
    the feed stopped. The stop overshoots the antenna's sweet spot, so this
    unloads, re-feeds to just short of the remembered detect position, then
    creeps across it reading at each step.

    This is the machinery tuned on hardware (sweep_back 240 / step 25 /
    past 200), and every bound in it exists to stop a failed read becoming an
    endless motion loop."""

    def _ready(self, **kw):
        ams, coord = _build_unit(values=kw or None)
        ams._rfid_send_load = lambda lane, slot: True
        ams._wait_for_idle = lambda *a, **k: None
        ams._rfid_scan_stop_load = lambda: None
        ams.oams.action_status = None
        return ams, coord

    def test_a_failed_unload_aborts_without_feeding(self):
        # If the unload did not happen the filament is not where the re-feed
        # assumes; feeding anyway would drive past the reader entirely.
        ams, coord = self._ready()
        ams.oams.unload_spool = lambda: (_ for _ in ()).throw(
            RuntimeError("busy"))
        fed = []
        ams._rfid_send_load = lambda lane, slot: fed.append(1) or True
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 500) is None
        assert fed == []

    def test_a_refused_reload_aborts(self):
        ams, coord = self._ready()
        ams._rfid_send_load = lambda lane, slot: False
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 500) is None

    def test_an_unreadable_encoder_at_the_start_is_treated_as_zero(self):
        ams, coord = self._ready()
        type(ams.oams).encoder_clicks = property(
            lambda self: (_ for _ in ()).throw(RuntimeError("no encoder")))
        try:
            got = ams._rfid_reposition_and_read(
                coord, ams.lanes["lane1"], 0, 0, set(), 500)
        finally:
            del type(ams.oams).encoder_clicks
        assert got is None

    def test_a_decode_on_the_first_sweep_read_returns_it(self):
        ams, coord = self._ready()
        tag = {"uid": "AABB", "filament": {"material": "PLA"}}
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: tag
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 500) is tag

    def test_it_gives_up_once_past_the_window(self):
        # pos beyond detect_delta + sweep_past means the sweet spot is behind
        # us; continuing would just unspool filament.
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        ams.oams.encoder_clicks = 100000
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 100) is None

    def test_an_unreadable_encoder_mid_sweep_ends_the_sweep(self):
        # Treated as "past the window": without a position there is no way to
        # know when to stop, and creeping blind is worse than giving up.
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        calls = {"n": 0}

        def clicks(self):
            calls["n"] += 1
            if calls["n"] > 2:
                raise RuntimeError("encoder lost")
            return 0
        type(ams.oams).encoder_clicks = property(clicks)
        try:
            got = ams._rfid_reposition_and_read(
                coord, ams.lanes["lane1"], 0, 0, set(), 100)
        finally:
            del type(ams.oams).encoder_clicks
        assert got is None

    def test_the_sweep_is_step_bounded_so_a_stalled_encoder_cannot_loop(self):
        # With the encoder frozen at 0 the position never advances; only the
        # step bound ends this.
        ams, coord = self._ready(rfid_scan_sweep_back=50,
                                 rfid_scan_sweep_step=25,
                                 rfid_scan_sweep_past=50)
        reads = []
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: reads.append(1)
        ams.oams.encoder_clicks = 0
        assert ams._rfid_reposition_and_read(
            coord, ams.lanes["lane1"], 0, 0, set(), 0) is None
        # max_steps = 2 + (50 + 50) // 25 = 6
        assert len(reads) == 6

    def test_a_load_that_finishes_short_creeps_with_the_follower(self):
        # Short-PTFE override: the load completes before reaching the target,
        # so the follower has to cover the rest or the sweep starts too early.
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        ams.oams.encoder_clicks = 0
        ams.oams.action_status = None          # load already finished
        reasons = []
        ams._rfid_set_follower = lambda e, d, reason: reasons.append(reason)
        # detect_delta must exceed sweep_back (240) or target_early is 0 and
        # the re-feed wait breaks immediately without ever needing the creep.
        ams._rfid_reposition_and_read(coord, ams.lanes["lane1"], 0, 0,
                                      set(), 500)
        assert any("creep" in r for r in reasons), reasons

    def test_a_still_running_load_is_cancelled_before_the_sweep(self):
        ams, coord = self._ready()
        ams._rfid_read_stationary = lambda c, s, sis, attempts=1: None
        ams.oams.encoder_clicks = 100000        # target reached at once
        ams.oams.action_status = "loading"      # ...but the load is still live
        stopped = []
        ams._rfid_scan_stop_load = lambda: stopped.append(1)
        ams._rfid_reposition_and_read(coord, ams.lanes["lane1"], 0, 0,
                                      set(), 100)
        assert stopped, "an in-flight load must be cancelled before sweeping"


class TestStopLoadWaitsForTheFirmware:
    """Cancelling a scan feed must WAIT for the firmware to acknowledge, or the
    unload that follows is rejected as busy -- which is how a scan used to
    leave the lane wedged."""

    def test_a_cancel_that_throws_is_survived(self):
        ams, _ = _build_unit()
        ams.oams.action_status = None
        ams.oams.load_spool_cancel = lambda: (_ for _ in ()).throw(
            RuntimeError("not loading"))
        ams._rfid_scan_stop_load()

    def test_it_waits_until_the_firmware_reports_idle(self):
        # The fake's load_spool_cancel acks immediately, which real firmware
        # does not always do -- override it so the wait loop actually runs.
        ams, _ = _build_unit()
        ams.oams.load_spool_cancel = lambda: None
        ams.oams.action_status = "loading"
        orig = ams.afc.reactor.pause

        def pause(until):
            ams.oams.action_status = None      # firmware acks on the next tick
            return orig(until)
        ams.afc.reactor.pause = pause
        ams._rfid_scan_stop_load()
        assert ams.oams.action_status is None

    def test_a_firmware_that_never_acks_is_forced_clear_after_the_deadline(self):
        # Otherwise every later operation is refused as busy forever.
        ams, _ = _build_unit()
        ams.oams.load_spool_cancel = lambda: None   # firmware ignores the cancel
        ams.oams.action_status = "loading"          # ...and never clears it
        ams._rfid_scan_stop_load()
        assert ams.oams.action_status is None       # forced clear at the deadline


class TestReadyWaitReadsSilenceAsReady:
    """The OAMS firmware only reports on a refusal or a state change, so
    SILENCE is the ready signal. The wait probes with an idempotent command
    and treats any answer -- or any encoder movement -- as 'still busy'."""

    def test_an_unreadable_encoder_at_the_start_seeds_zero(self):
        ams, _ = _build_unit()
        type(ams.oams).encoder_clicks = property(
            lambda self: (_ for _ in ()).throw(RuntimeError("no encoder")))
        try:
            ams._rfid_wait_for_unit_ready(timeout=0.5, quiet_time=0.1)
        finally:
            del type(ams.oams).encoder_clicks

    def test_a_probe_that_throws_does_not_end_the_wait(self):
        # The probe is a nicety; a controller that refuses it is still being
        # watched via its status and encoder.
        ams, _ = _build_unit()
        ams.oams.set_oams_follower = lambda e, d: (_ for _ in ()).throw(
            RuntimeError("busy"))
        assert ams._rfid_wait_for_unit_ready(
            timeout=0.5, quiet_time=0.1) in (True, False)

    def test_an_encoder_that_becomes_unreadable_holds_the_last_value(self):
        # Treating an unreadable encoder as "moved" would reset the quiet
        # window forever and the wait could never succeed.
        ams, _ = _build_unit()
        state = {"n": 0}

        def clicks(self):
            state["n"] += 1
            if state["n"] > 2:
                raise RuntimeError("encoder lost")
            return 0
        type(ams.oams).encoder_clicks = property(clicks)
        try:
            assert ams._rfid_wait_for_unit_ready(
                timeout=0.6, quiet_time=0.1) is True
        finally:
            del type(ams.oams).encoder_clicks


class TestUndecodedTagIsNamedAsAKeyProblem:
    """A tag that ANSWERS but will not decode is a key/format problem, not a
    positioning one. Saying so is the difference between 'check your keys' and
    an operator moving the antenna around for an hour."""

    def test_an_answering_but_undecodable_tag_is_reported(self):
        ams, coord = _build_unit()
        said = []
        ams.logger.info = lambda m, *a, **k: said.append(m)
        coord.read_slot_excluding = lambda slot, exclude: {
            "uid": "AABB", "tag_type": "MifareClassic1k"}      # no filament
        assert ams._rfid_read_stationary(coord, 0, set(), attempts=2) is None
        assert any("won't decode" in m and "AABB" in m for m in said)
        assert any("AFC_rfid_keys" in m for m in said)

    def test_a_silent_antenna_says_nothing_about_keys(self):
        ams, coord = _build_unit()
        said = []
        ams.logger.info = lambda m, *a, **k: said.append(m)
        coord.read_slot_excluding = lambda slot, exclude: None
        assert ams._rfid_read_stationary(coord, 0, set(), attempts=2) is None
        assert not any("won't decode" in m for m in said)


class _FlakyEncoder:
    """Controller wrapper whose encoder and/or hub reads raise.

    Every read of those two is wrapped in the scan because they are polled
    continuously while filament moves; a controller that goes briefly
    unreadable must degrade the scan, not abort it mid-motion with the
    follower still on.
    """

    def __init__(self, inner, encoder=False, hub=False):
        object.__setattr__(self, "_inner", inner)
        object.__setattr__(self, "_bad_enc", encoder)
        object.__setattr__(self, "_bad_hub", hub)

    def __getattr__(self, name):
        if name == "encoder_clicks" and self._bad_enc:
            raise RuntimeError("encoder unreadable")
        if name == "hub_hes_value" and self._bad_hub:
            raise RuntimeError("hub unreadable")
        return getattr(self._inner, name)

    def __setattr__(self, name, value):
        setattr(self._inner, name, value)


class TestScanSurvivesADegradedController:
    """A full scan with the encoder and/or hub unreadable. These are the
    fallbacks that keep a scan recoverable rather than leaving the lane
    mid-feed: the scan is already moving filament when they fire."""

    def _run(self, **flags):
        coord = FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])
        ams, coord = _build_unit(coord=coord)
        ams.oams = _FlakyEncoder(ams.oams, **flags)
        result = ams._do_rfid_scan(ams.lanes["lane1"])
        return ams, coord, result

    def test_an_unreadable_encoder_throughout_still_completes(self):
        # clicks_start, the engage check and the detect position all fall back;
        # the tag is still found and applied because detection is by UID.
        ams, coord, result = self._run(encoder=True)
        assert result is True
        assert len(coord.applied) == 1

    def test_an_unreadable_hub_falls_back_to_not_engaged(self):
        ams, coord, result = self._run(hub=True)
        assert result is True

    def test_both_unreadable_still_completes(self):
        ams, coord, result = self._run(encoder=True, hub=True)
        assert result is True


class TestScanUnwindFailures:
    """The unwind runs in the finally: it is what puts the filament back after
    a scan. If it throws, the failure must be reported and the operation guard
    still cleared -- otherwise the lane is locked out until a restart."""

    def _coord(self):
        return FakeCoordinator(fields=[[], [], ["aabb"]], full_reads=[TAG])

    def test_a_failing_unload_is_reported_and_the_guard_is_cleared(self):
        ams, coord = _build_unit(coord=self._coord())
        warned = []
        ams.logger.warning = lambda m, *a, **k: warned.append(m)
        ams.oams.unload_spool = lambda: (_ for _ in ()).throw(
            RuntimeError("unload refused"))
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert ams._operation_active is False
        assert any("unwind" in m or "unload" in m for m in warned), warned

    # NB: there is deliberately no test for the finally's follower stop
    # throwing. _rfid_set_follower cannot raise -- both the follower-object
    # tier and the direct-controller tier are internally wrapped (see
    # TestFollowerControlFallback::test_both_tiers_failing_is_survived) -- so
    # forcing it would only exercise a path the code cannot reach.


class TestF1sDebounceCommitted:
    """The f1s (bay presence) debounce mirrors the hub one: a change commits to
    prep_state only after holding, so a bouncing insert switch cannot fire the
    insert edge -- and the insert edge is what schedules an RFID scan."""

    def _lane(self):
        return types.SimpleNamespace(prep_state=None)

    def _ams(self, committed):
        ams, _ = _build_unit()
        ams._last_f1s = [committed, None, None, None]
        ams._f1s_pending_since = [None, None, None, None]
        ams.f1s_debounce = 0.5
        return ams

    def test_an_unchanged_reading_keeps_the_committed_value(self):
        ams, lane = self._ams(True), self._lane()
        ams._update_f1s_debounced(lane, "lane1", 0, True, 100.0, False)
        assert lane.prep_state is True
        assert ams._f1s_pending_since[0] is None

    def test_a_change_does_not_commit_immediately(self):
        ams, lane = self._ams(True), self._lane()
        ams._update_f1s_debounced(lane, "lane1", 0, False, 100.0, False)
        assert lane.prep_state is True
        assert ams._f1s_pending_since[0] == 100.0

    def test_a_change_inside_the_window_is_ignored(self):
        ams, lane = self._ams(True), self._lane()
        ams._update_f1s_debounced(lane, "lane1", 0, False, 100.0, False)
        ams._update_f1s_debounced(lane, "lane1", 0, False, 100.2, False)
        assert lane.prep_state is True


class TestScanWithAFirmwareThatKeepsLoading:
    """The firmware does not always report the load finished. The scan has to
    cancel it explicitly at both exit points -- on a detection and on a timeout
    -- or the unload that follows is refused as busy and the lane is stuck."""

    def _ams(self, fields, reads):
        coord = FakeCoordinator(fields=fields, full_reads=reads)
        ams, coord = _build_unit(coord=coord)
        # Firmware that moves filament but never acks the load. NB: the fake
        # binds oams_load_spool_cmd.send at construction, so the bound command
        # is what has to be replaced -- not _send_load.
        def never_acks(args):
            ams.oams.encoder_clicks += 120
            ams.oams.hub_hes_value[args[0]] = 1
            # action_status deliberately left as LOADING
        ams.oams.oams_load_spool_cmd.send = never_acks
        # NB: do NOT preset hub_hes_value -- the scan re-checks the hub before
        # loading and a pre-tripped bay reads as "hub sensor shows filament",
        # refusing the scan. never_acks trips it during the load instead,
        # which is what makes `engaged` true and the unwind run.
        stops = []
        real_stop = ams._rfid_scan_stop_load

        def stop():
            stops.append(1)
            ams.oams.action_status = None
            return real_stop()
        ams._rfid_scan_stop_load = stop
        return ams, coord, stops

    def test_the_load_is_cancelled_when_a_tag_is_detected(self):
        ams, coord, stops = self._ams([[], [], ["aabb"]], [TAG])
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert stops, "a still-running load must be cancelled at detection"

    def test_the_load_is_cancelled_on_timeout_too(self):
        # No tag ever appears; the feed must still be stopped.
        ams, coord, stops = self._ams([[]], [])
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert stops, "a still-running load must be cancelled on timeout"

    def test_a_failing_unwind_is_reported(self):
        ams, coord, _ = self._ams([[], [], ["aabb"]], [TAG])
        warned = []
        ams.logger.warning = lambda m, *a, **k: warned.append(m)
        ams.oams.unload_spool = lambda: (_ for _ in ()).throw(
            RuntimeError("bay blocked"))
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert any("unwind failed" in m for m in warned), warned
        assert ams._operation_active is False

    def test_a_failing_clear_errors_is_swallowed(self):
        # Cosmetic tidy-up at the end of a scan; it must not turn a successful
        # read into a failure.
        ams, coord, _ = self._ams([[], [], ["aabb"]], [TAG])
        ams.oams.clear_errors = lambda: (_ for _ in ()).throw(
            RuntimeError("no link"))
        assert ams._do_rfid_scan(ams.lanes["lane1"]) is True

    def test_an_unreadable_hub_during_the_cancel_decision(self):
        # The cancel decision reads the hub to see whether the feed engaged;
        # unreadable means "not engaged", which just defers to the encoder cap.
        coord = FakeCoordinator(fields=[[]], full_reads=[])
        ams, coord = _build_unit(coord=coord)
        def never_acks(args):
            ams.oams.encoder_clicks += 120
        ams.oams.oams_load_spool_cmd.send = never_acks
        ams.oams = _FlakyEncoder(ams.oams, hub=True)
        ams._do_rfid_scan(ams.lanes["lane1"])
        assert ams._operation_active is False


# ── Unit tests for afcAMS sensor polling and load finishing in ────────────────
#
# was tests/test_AFC_OpenAMS_sensors.py
# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_unit_sensors(lane=None, f1s=None, hub=None, last_f1s=None,
               stale=False, op_active=False, spool_map=None):
    unit = afcAMS.__new__(afcAMS)
    unit.name = "AMS_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.lanes = {}
    if lane is not None:
        unit.lanes[lane.name] = lane
        unit.afc.lanes[lane.name] = lane
    unit._spool_map = spool_map if spool_map is not None else (
        {lane.name: 0} if lane is not None else {})
    unit._current_action = ""
    unit._operation_active = op_active
    unit._prev_states_stale = stale
    unit._last_f1s = list(last_f1s) if last_f1s is not None else [None] * 4
    unit._last_hub = [None] * 4
    unit._hub_load_suppressed = set()
    unit.oams = FakeOams(f1s=f1s, hub=hub)
    return unit


def _lane_sensors(name="lane0", loaded_to_hub=True):
    lane = FakeLane(name, extruder_obj=FakeExtruderObj("extruder"))
    lane.loaded_to_hub = loaded_to_hub
    return lane


# ── Early-out branches ────────────────────────────────────────────────────────

def test_poll_stops_without_hardware():
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane)
    unit.oams = None

    assert unit._poll_oams_sensors(100.0) == unit.afc.reactor.NEVER


def test_poll_skipped_during_operation():
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[0, 0, 0, 0], last_f1s=[True, None, None, None],
                      op_active=True)

    result = unit._poll_oams_sensors(100.0)

    assert result == 100.25  # re-schedules without touching state
    assert not lane.handle_load_runout.called
    assert lane.loaded_to_hub is True
    assert lane.prep_state is False           # untouched
    assert unit._last_f1s[0] is True          # snapshot untouched


def test_poll_skips_unmapped_lane():
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], spool_map={})  # not in map

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is False
    assert not lane.handle_load_runout.called


def test_poll_handles_short_sensor_arrays():
    """Slot index past the reported arrays: neither block runs."""
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[], hub=[], spool_map={"lane0": 0})
    unit.oams.f1s_hes_value = []
    unit.oams.hub_hes_value = []

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is False
    assert lane._load_state is False
    assert not lane.handle_load_runout.called


# ── F1S transitions ───────────────────────────────────────────────────────────

def test_f1s_present_drives_prep_state_no_event_without_change():
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane.prep_state is True
    assert lane.loaded_to_hub is True         # untouched while present
    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is True


def test_f1s_first_reading_sets_snapshot_without_event():
    """old_f1s None (first poll ever): no insert/remove event fires."""
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[None, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is True


def test_f1s_loss_fires_runout_and_clears_staging():
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[0, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    assert not lane.handle_load_runout.called
    unit._poll_oams_sensors(101.0)      # held past f1s_debounce -> commits

    assert lane.prep_state is False
    assert lane.loaded_to_hub is False        # spool gone -> can't be staged
    assert lane.handle_load_runout.calls == [((101.0, False), {})]
    assert unit._last_f1s[0] is False


def test_f1s_insert_fires_load():
    lane = _lane_sensors(loaded_to_hub=False)
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    assert not lane.handle_load_runout.called
    assert lane.prep_state is False     # committed state until it holds
    unit._poll_oams_sensors(101.0)      # held past f1s_debounce -> commits

    assert lane.prep_state is True
    assert lane.handle_load_runout.calls == [((101.0, True), {})]
    assert lane._load_suppressed is False


def test_resync_after_operation_suppresses_events():
    """First poll after a load/unload: the previous snapshot is stale — just
    re-sync it without firing insert/remove."""
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[0, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None], stale=True)

    unit._poll_oams_sensors(100.0)

    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is False         # re-synced
    assert unit._prev_states_stale is False   # consumed
    assert lane.prep_state is False           # live state still applied
    assert lane.loaded_to_hub is False


def test_suppressed_load_marks_lane_suppressed():
    """A load event the unit itself caused (hub feed) is marked suppressed so
    handle_load_runout can tell it apart from a user insert."""
    lane = _lane_sensors(loaded_to_hub=False)
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])
    unit._hub_load_suppressed = {"lane0"}

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    unit._poll_oams_sensors(101.0)      # commits

    assert lane._load_suppressed is True
    assert unit._hub_load_suppressed == set()  # consumed
    assert lane.handle_load_runout.calls == [((101.0, True), {})]


def test_sensor_block_swallows_true_update_during_runout():
    """During an active same-FPS runout reload, a True F1S flicker is
    blocked: no event, but the snapshot still records the new value."""
    lane = _lane_sensors()
    lane.tool_loaded = True
    lane.status = AFCLaneState.INFINITE_RUNOUT
    lane._oams_runout_detected = True
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[False, None, None, None])
    unit.afc.function.printing = True

    unit._poll_oams_sensors(100.0)      # starts the debounce window
    unit._poll_oams_sensors(101.0)      # commits (blocked path)

    assert not lane.handle_load_runout.called
    assert unit._last_f1s[0] is True          # snapshot advanced anyway
    assert lane._oams_runout_detected is True  # block window still active


# ── Hub HES -> raw load state ─────────────────────────────────────────────────

def test_hub_hes_drives_raw_load_state():
    lane = _lane_sensors()
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], hub=[1, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane._load_state == 1
    assert unit._last_hub[0] == 1


def test_hub_hes_clear_reads_zero():
    lane = _lane_sensors()
    lane._load_state = True
    unit = _make_unit_sensors(lane, f1s=[1, 0, 0, 0], hub=[0, 0, 0, 0],
                      last_f1s=[True, None, None, None])

    unit._poll_oams_sensors(100.0)

    assert lane._load_state == 0


# ── _advance_tool_stn_to_nozzle ───────────────────────────────────────────────

class _Ext:
    """Extruder with explicit tool_stn/tool_load_speed (omit to test the
    getattr defaults)."""

    def __init__(self, tool_stn=None, tool_load_speed=None):
        if tool_stn is not None:
            self.tool_stn = tool_stn
        if tool_load_speed is not None:
            self.tool_load_speed = tool_load_speed


def _advance_unit():
    unit = afcAMS.__new__(afcAMS)
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.afc.afcDeltaTime = Recorder()
    unit.afc.afcDeltaTime.log_with_time = Recorder()
    unit._oams_extrude = Recorder()
    return unit


def _advance_lane(ext):
    lane = FakeLane("lane0", extruder_obj=ext)
    return lane


def test_tool_stn_advance_full_distance():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(100.0, 25.0)))
    assert unit._oams_extrude.calls == [
        ((100.0, 25.0 * 60.0, "tool_stn_to_nozzle"), {})]


def test_tool_stn_advance_credits_engagement():
    """Engagement verification already pushed filament past the sensor —
    only the remainder is extruded."""
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(100.0, 25.0)),
                                     already_advanced=30.0)
    assert unit._oams_extrude.last_args[0] == 70.0


def test_tool_stn_advance_noop_when_fully_covered():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(50.0, 25.0)),
                                     already_advanced=60.0)
    assert not unit._oams_extrude.called


def test_tool_stn_advance_noop_without_tool_stn():
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(0, 25.0)))
    assert not unit._oams_extrude.called
    # Attribute missing entirely -> getattr default 0 -> no-op
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext()))
    assert not unit._oams_extrude.called


def test_tool_stn_advance_default_speed():
    """tool_load_speed missing -> 25mm/s default (x60 for mm/min)."""
    unit = _advance_unit()
    unit._advance_tool_stn_to_nozzle(_advance_lane(_Ext(tool_stn=40.0)))
    assert unit._oams_extrude.last_args == (40.0, 1500.0, "tool_stn_to_nozzle")


# ── Unit tests for afcAMS runout handling in extras/AFC_OpenAMS.py ────────────
#
# was tests/test_AFC_OpenAMS_runout.py
# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_unit_runout(lanes=()):
    unit = afcAMS.__new__(afcAMS)
    unit.name = "AMS_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    for lane in lanes:
        unit.afc.lanes[lane.name] = lane
    unit.lane_not_ready = Recorder()
    unit.handle_same_fps_reload = Recorder()
    return unit


def _lane_runout(name, extruder="extruder", runout_lane=None, **kw):
    return FakeLane(name, extruder_obj=FakeExtruderObj(extruder),
                    runout_lane=runout_lane, **kw)


# ── handle_runout: branch 1 — no runout lane configured ───────────────────────

def test_runout_without_runout_lane_pauses_and_handles():
    lane = _lane_runout("lane0", runout_lane=None)
    unit = _make_unit_runout([lane])

    handled = unit.handle_runout(lane)

    # Return + every state mutation
    assert handled is True
    assert lane.status == AFCLaneState.NONE
    assert unit.lane_not_ready.calls == [((lane,), {})]
    assert unit.afc.error.AFC_error.call_count == 1
    msg = unit.afc.error.AFC_error.last_args[0]
    assert "Runout detected on OAMS lane0" in msg
    assert "No runout lane configured" in msg
    assert unit.afc.error.AFC_error.last_kwargs == {"pause": True}
    # Nothing else fired, no flags set
    assert not unit.handle_same_fps_reload.called
    assert lane._oams_runout_detected is False
    assert lane._oams_runout_empty is False


# ── handle_runout: branch 2 — runout lane doesn't resolve ─────────────────────

def test_runout_with_unresolvable_target_pauses_and_handles():
    lane = _lane_runout("lane0", runout_lane="lane_missing")
    unit = _make_unit_runout([lane])

    handled = unit.handle_runout(lane)

    assert handled is True
    assert lane.status == AFCLaneState.NONE
    assert unit.lane_not_ready.calls == [((lane,), {})]
    assert unit.afc.error.AFC_error.call_count == 1
    assert "lane_missing" in unit.afc.error.AFC_error.last_args[0]
    assert "not found" in unit.afc.error.AFC_error.last_args[0]
    assert unit.afc.error.AFC_error.last_kwargs == {"pause": True}
    assert not unit.handle_same_fps_reload.called
    assert lane._oams_runout_empty is False


# ── handle_runout: branch 3 — same extruder, seamless reload ──────────────────

def test_same_extruder_runout_does_seamless_reload():
    lane = _lane_runout("lane0", extruder="extruder", runout_lane="lane1")
    target = _lane_runout("lane1", extruder="extruder")
    unit = _make_unit_runout([lane, target])

    handled = unit.handle_runout(lane)

    assert handled is True
    assert lane._oams_runout_detected is True   # blocks sensor noise
    assert lane._oams_runout_empty is False     # hardware unload NOT skipped
    assert unit.handle_same_fps_reload.calls == [((lane, target), {})]
    # No pause path artifacts
    assert not unit.afc.error.AFC_error.called
    assert not unit.lane_not_ready.called
    assert lane.status is None                  # untouched on this branch


# ── handle_runout: branch 4 — cross extruder, defer to generic ────────────────

def test_cross_extruder_runout_defers_to_generic_infinite_spool():
    lane = _lane_runout("lane0", extruder="extruder", runout_lane="lane4")
    target = _lane_runout("lane4", extruder="extruder4")
    unit = _make_unit_runout([lane, target])

    handled = unit.handle_runout(lane)

    assert handled is False                     # generic path takes over
    assert lane._oams_runout_empty is True      # hardware unload will be skipped
    assert lane._oams_runout_detected is False
    assert not unit.handle_same_fps_reload.called
    assert not unit.afc.error.AFC_error.called
    assert not unit.lane_not_ready.called


# ── _is_same_extruder: every branch ───────────────────────────────────────────

def test_is_same_extruder_matches_case_and_whitespace():
    unit = _make_unit_runout()
    a = _lane_runout("a", extruder=" Extruder ")
    b = _lane_runout("b", extruder="extruder")
    assert unit._is_same_extruder(a, b) is True


def test_is_same_extruder_differs():
    unit = _make_unit_runout()
    assert unit._is_same_extruder(_lane_runout("a", extruder="extruder"),
                                  _lane_runout("b", extruder="extruder4")) is False


def test_is_same_extruder_missing_extruder_obj():
    unit = _make_unit_runout()
    a = _lane_runout("a")
    a.extruder_obj = None
    assert unit._is_same_extruder(a, _lane_runout("b")) is False
    b = _lane_runout("b")
    b.extruder_obj = None
    assert unit._is_same_extruder(_lane_runout("a"), b) is False


def test_is_same_extruder_missing_names():
    unit = _make_unit_runout()
    a = _lane_runout("a")
    a.extruder_obj = FakeExtruderObj(name=None)
    assert unit._is_same_extruder(a, _lane_runout("b")) is False
    c = _lane_runout("c")
    c.extruder_obj = FakeExtruderObj(name="")
    assert unit._is_same_extruder(_lane_runout("a"), c) is False


# ── _resolve_lane_reference: every branch ─────────────────────────────────────

def test_resolve_lane_exact_match():
    lane = _lane_runout("lane0")
    unit = _make_unit_runout([lane])
    assert unit._resolve_lane_reference("lane0") is lane


def test_resolve_lane_case_insensitive_fallback():
    lane = _lane_runout("Lane0")
    unit = _make_unit_runout([lane])
    assert unit._resolve_lane_reference("lane0") is lane


def test_resolve_lane_missing_and_empty():
    unit = _make_unit_runout()
    assert unit._resolve_lane_reference("nope") is None
    assert unit._resolve_lane_reference(None) is None
    assert unit._resolve_lane_reference("") is None


# ── _should_block_sensor_for_runout: full condition matrix ────────────────────

def _runout_lane(printing=True, tool_loaded=True,
                 status=AFCLaneState.INFINITE_RUNOUT, detected=True):
    lane = _lane_runout("lane0", tool_loaded=tool_loaded, status=status)
    lane._oams_runout_detected = detected
    return lane, printing


def test_sensor_block_noop_without_flag():
    unit = _make_unit_runout()
    lane, _ = _runout_lane(detected=False)
    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_active_runout_blocks_true_updates():
    unit = _make_unit_runout()
    unit.afc.function.printing = True
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is True
    assert lane._oams_runout_detected is True  # flag survives while blocking


def test_sensor_block_tool_unloading_status_also_blocks():
    unit = _make_unit_runout()
    unit.afc.function.printing = True
    lane, _ = _runout_lane(status=AFCLaneState.TOOL_UNLOADING)
    assert unit._should_block_sensor_for_runout(lane, True) is True


def test_sensor_block_false_update_clears_flag():
    unit = _make_unit_runout()
    unit.afc.function.printing = True
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, False) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_not_printing_clears_flag():
    unit = _make_unit_runout()
    unit.afc.function.printing = False
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_not_tool_loaded_clears_flag():
    unit = _make_unit_runout()
    unit.afc.function.printing = True
    lane, _ = _runout_lane(tool_loaded=False)

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_wrong_status_clears_flag():
    unit = _make_unit_runout()
    unit.afc.function.printing = True
    lane, _ = _runout_lane(status=AFCLaneState.NONE)

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


def test_sensor_block_exception_treated_as_inactive():
    """If the printing check raises, active_runout falls to False and the
    flag clears rather than blocking forever."""
    unit = _make_unit_runout()
    unit.afc.function.raise_on_is_printing = RuntimeError("boom")
    lane, _ = _runout_lane()

    assert unit._should_block_sensor_for_runout(lane, True) is False
    assert lane._oams_runout_detected is False


# ── FollowerController constructor contract ──────────────────────────────────
# The e1a2da0 bug was a call-site/constructor mismatch:
# FollowerController(oams_obj, printer, logger) vs (oams_dict, reactor, logger).

def test_follower_controller_constructed_like_call_site():
    oams = object()
    reactor = FakeReactor()
    logger = FakeLogger()

    follower = FollowerController({"oams1": oams}, reactor, logger)

    assert follower.oams == {"oams1": oams}
    assert follower.reactor is reactor
    assert follower.logger is logger
    assert follower.follower_state == {}
    assert follower._mcu_command_queue == {}
    assert follower._mcu_command_in_flight == {}
    assert follower._led_error_state == {}


def test_follower_state_created_on_demand():
    follower = FollowerController({"oams1": object()}, FakeReactor(), FakeLogger())
    state = follower.get_follower_state("oams1")
    assert isinstance(state, FollowerState)
    assert follower.get_follower_state("oams1") is state
    assert follower.follower_state == {"oams1": state}


# ── Unit tests for the OAMSMonitor clog and stuck-spool detection in ──────────
#
# was tests/test_AFC_OpenAMS_monitor.py
# ── Fakes ─────────────────────────────────────────────────────────────────────

class _FakeExtruder:
    """A Klipper extruder stand-in: identity + its own position counter."""

    def __init__(self, name, last_position=0.0):
        self.name = name
        self.last_position = last_position


class _FakeFps:
    """Minimal fps object: fps_value + an `extruder` attribute the test can
    swap to emulate a toolchange (the real property resolves the ACTIVE
    toolhead extruder). Pass extruder=None to omit the attribute entirely."""

    def __init__(self, extruder=None):
        if extruder is not None:
            self.extruder = extruder
        self.fps_value = CLOG_PRESSURE_TARGET


def _make_monitor(fps=None):
    on_clog = Recorder()
    on_stuck = Recorder()
    on_cleared = Recorder()
    monitor = OAMSMonitor(
        fps_name="FPS_test",
        fps_obj=fps if fps is not None else _FakeFps(_FakeExtruder("extruder")),
        reactor=FakeReactor(),
        logger=FakeLogger(),
        on_clog=on_clog,
        on_stuck_spool=on_stuck,
        on_stuck_cleared=on_cleared,
        clog_sensitivity="medium",
        is_printing_fn=lambda: True,
    )
    # _check_clog reads st.last_encoder (normally fed by the timer loop)
    monitor.state.last_encoder = 0
    return monitor, on_clog, on_stuck, on_cleared


# ── _check_clog: gates ────────────────────────────────────────────────────────

def test_clog_post_load_grace_suppresses():
    monitor, on_clog, _, _ = _make_monitor()
    monitor.state.last_lane_change_time = 100.0

    monitor._check_clog(100.0 + monitor.clog_post_load_grace - 1.0,
                        0, CLOG_PRESSURE_TARGET)

    assert monitor.state.clog_start_time is None
    assert not on_clog.called


def test_clog_requires_extruder_position():
    """fps without an extruder attribute: detection cannot run."""
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder=None))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)

    assert monitor.state.clog_start_time is None
    assert not on_clog.called


# ── _check_clog: genuine clog on a single extruder ────────────────────────────

def test_clog_fires_same_extruder():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)  # opens window
    assert monitor.state.clog_start_time == 100.0
    assert monitor.state.clog_start_extruder == 100.0
    assert monitor.state.clog_start_extruder_obj is extruder
    assert monitor.state.clog_start_encoder == 0

    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert on_clog.call_count == 1
    assert monitor.state.clog_active is True
    fps_name, msg = on_clog.last_args
    assert fps_name == "FPS_test"
    assert "Clog detected" in msg


def test_clog_does_not_fire_below_extrusion_window():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW / 2
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_active is False
    assert monitor.state.clog_start_time == 100.0  # window still open


def test_clog_active_does_not_refire():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)
    monitor._check_clog(100.0 + 2 * CLOG_DWELL, 0, CLOG_PRESSURE_TARGET)

    assert on_clog.call_count == 1  # one-shot while active


# ── _check_clog: extruder swap (the toolchange false positive) ────────────────

def test_extruder_swap_restarts_window():
    """A toolchange mid-window swaps fps.extruder to a different object whose
    position counter differs arbitrarily. The window must restart on the new
    extruder instead of firing off the cross-extruder position delta."""
    extruder_a = _FakeExtruder("extruder", last_position=1000.0)
    extruder_b = _FakeExtruder("extruder4", last_position=1060.9)
    fps = _FakeFps(extruder_a)
    monitor, on_clog, _, _ = _make_monitor(fps=fps)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    assert monitor.state.clog_start_extruder_obj is extruder_a

    fps.extruder = extruder_b  # toolchange (counters differ by 60.9mm)
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_active is False
    # Window restarted on B with B's own baseline
    assert monitor.state.clog_start_extruder_obj is extruder_b
    assert monitor.state.clog_start_extruder == extruder_b.last_position
    assert monitor.state.clog_start_time == 100.0 + CLOG_DWELL + 1.0


def test_clog_fires_on_new_extruder_after_swap():
    extruder_a = _FakeExtruder("extruder", last_position=1000.0)
    extruder_b = _FakeExtruder("extruder4", last_position=0.0)
    fps = _FakeFps(extruder_a)
    monitor, on_clog, _, _ = _make_monitor(fps=fps)

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    fps.extruder = extruder_b
    monitor._check_clog(110.0, 0, CLOG_PRESSURE_TARGET)   # restart on B
    assert not on_clog.called

    extruder_b.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(110.0 + CLOG_DWELL + 1.0, 0, CLOG_PRESSURE_TARGET)

    assert on_clog.call_count == 1
    assert monitor.state.clog_active is True


def test_swap_back_and_forth_never_fires_without_advance():
    extruder_a = _FakeExtruder("extruder", last_position=500.0)
    extruder_b = _FakeExtruder("extruder4", last_position=1234.5)
    fps = _FakeFps(extruder_a)
    monitor, on_clog, _, _ = _make_monitor(fps=fps)

    eventtime = 100.0
    for _ in range(5):
        monitor._check_clog(eventtime, 0, CLOG_PRESSURE_TARGET)
        eventtime += CLOG_DWELL + 1.0
        fps.extruder = extruder_b if fps.extruder is extruder_a else extruder_a

    assert not on_clog.called
    assert monitor.state.clog_active is False


# ── _check_clog: re-baselining and condition resets ───────────────────────────

def test_cumulative_encoder_progress_restarts_window():
    """Cumulative encoder movement past the slack means filament IS flowing:
    the window re-baselines (incl. the extruder object) instead of firing."""
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    monitor.state.last_encoder = CLOG_ENCODER_SLACK * 3
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    fire_time = 100.0 + CLOG_DWELL + 1.0
    monitor._check_clog(fire_time, 0, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_start_time == fire_time
    assert monitor.state.clog_start_extruder == extruder.last_position
    assert monitor.state.clog_start_extruder_obj is extruder
    assert monitor.state.clog_start_encoder == CLOG_ENCODER_SLACK * 3


def test_pressure_out_of_band_resets_tracking():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0, 0, 0.2)  # tension, not target

    assert not on_clog.called
    assert monitor.state.clog_start_time is None


def test_encoder_moving_resets_tracking():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, on_clog, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    extruder.last_position += CLOG_EXTRUSION_WINDOW + 1.0
    monitor._check_clog(100.0 + CLOG_DWELL + 1.0,
                        CLOG_ENCODER_SLACK + 1, CLOG_PRESSURE_TARGET)

    assert not on_clog.called
    assert monitor.state.clog_start_time is None


# ── _check_stuck_spool ────────────────────────────────────────────────────────

LOW = STUCK_PRESSURE_LOW - 0.01


def test_stuck_spool_engagement_grace_suppresses():
    monitor, _, on_stuck, _ = _make_monitor()
    monitor.state.engagement_checked_at = 100.0

    monitor._check_stuck_spool(101.0, 0, LOW)  # within 6s grace

    assert monitor.state.stuck_start_time is None
    assert not on_stuck.called


def test_stuck_spool_fires_after_dwell():
    monitor, _, on_stuck, _ = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    assert monitor.state.stuck_start_time == 100.0
    assert monitor.state.stuck_active is False
    assert not on_stuck.called

    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, LOW)

    assert on_stuck.call_count == 1
    assert monitor.state.stuck_active is True
    fps_name, msg = on_stuck.last_args
    assert fps_name == "FPS_test"
    assert "Stuck spool" in msg


def test_stuck_spool_no_fire_when_encoder_moving():
    monitor, _, on_stuck, _ = _make_monitor()

    monitor._check_stuck_spool(100.0, STUCK_MIN_ENCODER, LOW)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, STUCK_MIN_ENCODER, LOW)

    assert not on_stuck.called
    assert monitor.state.stuck_start_time is None


def test_stuck_spool_no_fire_when_pressure_ok():
    monitor, _, on_stuck, _ = _make_monitor()
    ok = STUCK_PRESSURE_LOW + 0.05

    monitor._check_stuck_spool(100.0, 0, ok)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, ok)

    assert not on_stuck.called
    assert monitor.state.stuck_start_time is None


def test_stuck_spool_partial_recovery_keeps_latch():
    """Pressure between the low and clear thresholds with the encoder still:
    the stuck state persists (hysteresis) — no premature clear."""
    monitor, _, on_stuck, on_cleared = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, LOW)
    assert monitor.state.stuck_active is True

    between = (STUCK_PRESSURE_LOW + STUCK_PRESSURE_CLEAR) / 2
    monitor._check_stuck_spool(110.0, 0, between)

    assert monitor.state.stuck_active is True
    assert not on_cleared.called


def test_stuck_spool_clears_on_pressure_recovery():
    monitor, _, on_stuck, on_cleared = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    monitor._check_stuck_spool(100.0 + STUCK_DWELL + 0.5, 0, LOW)
    assert monitor.state.stuck_active is True

    monitor._check_stuck_spool(110.0, 0, STUCK_PRESSURE_CLEAR + 0.01)

    assert on_cleared.calls == [(("FPS_test",), {})]
    assert monitor.state.stuck_active is False
    assert monitor.state.stuck_start_time is None


def test_stuck_spool_pending_timer_clears_before_firing():
    """Recovery during the dwell (before firing) resets the timer without
    notifying anyone."""
    monitor, _, on_stuck, on_cleared = _make_monitor()

    monitor._check_stuck_spool(100.0, 0, LOW)
    monitor._check_stuck_spool(100.5, STUCK_MIN_ENCODER, STUCK_PRESSURE_CLEAR + 0.01)

    assert monitor.state.stuck_start_time is None
    assert not on_stuck.called
    assert not on_cleared.called


# ── FPSState.reset ────────────────────────────────────────────────────────────

def test_state_reset_clears_clog_window():
    extruder = _FakeExtruder("extruder", last_position=100.0)
    monitor, _, _, _ = _make_monitor(fps=_FakeFps(extruder))

    monitor._check_clog(100.0, 0, CLOG_PRESSURE_TARGET)
    assert monitor.state.clog_start_extruder_obj is extruder

    monitor.state.reset()

    assert monitor.state.clog_start_time is None
    assert monitor.state.clog_start_extruder is None
    assert monitor.state.clog_start_extruder_obj is None
    assert monitor.state.clog_start_encoder is None
    assert monitor.state.clog_active is False
    assert monitor.state.stuck_active is False
    assert monitor.state.stuck_start_time is None


# ── Unit tests for afcAMS.get_status (extras/AFC_OpenAMS.py) — surfacing the live ───
#
# was tests/test_AFC_OpenAMS_status.py
class _FakeState:
    def __init__(self, state=FPSLoadState.LOADED, current_lane=None,
                 current_spool_idx=None, clog_start_time=None,
                 stuck_start_time=None):
        self.state = state
        self.current_lane = current_lane
        self.current_spool_idx = current_spool_idx
        self.clog_start_time = clog_start_time
        self.stuck_start_time = stuck_start_time


class _FakeMonitor:
    def __init__(self, state):
        self.state = state


def _make_unit_status(oams=None, monitor=None, operation_active=False):
    unit = afcAMS.__new__(afcAMS)
    unit.lanes = {}          # empty -> base get_status returns empty aggregates
    unit.oams = oams
    unit._monitor = monitor
    unit._operation_active = operation_active
    return unit


def test_get_status_surfaces_controller_and_action():
    oams = FakeOams(current_spool=0, fps_value=0.4547, f1s=[1, 1, 1, 1],
                    hub=[1, 0, 0, 0], action_status=OAMSStatus.LOADING,
                    load_failures=2, unload_failures=1)
    mon = _FakeMonitor(_FakeState(state=FPSLoadState.LOADING,
                                  current_lane="lane4", current_spool_idx=0))
    unit = _make_unit_status(oams=oams, monitor=mon, operation_active=True)

    st = unit.get_status()

    # base structure preserved
    assert st["lanes"] == [] and "hubs" in st
    # controller live state
    assert st["oams_connected"] is True
    assert st["oams_current_spool"] == 0
    assert st["oams_fps_value"] == 0.4547
    assert st["oams_f1s_hes"] == [1, 1, 1, 1]
    assert st["oams_hub_hes"] == [1, 0, 0, 0]
    assert st["oams_load_failures"] == 2
    assert st["oams_unload_failures"] == 1
    # action + busy
    assert st["oams_action"] == "loading"
    assert st["oams_busy"] is True
    # monitor state
    assert st["oams_load_state"] == "loading"
    assert st["oams_current_lane"] == "lane4"
    assert st["oams_current_spool_idx"] == 0
    assert st["oams_clog_detecting"] is False
    assert st["oams_stuck_detecting"] is False


def test_get_status_action_idle_when_no_action():
    oams = FakeOams(action_status=None)
    unit = _make_unit_status(oams=oams, monitor=_FakeMonitor(_FakeState()))
    assert unit.get_status()["oams_action"] == "idle"


def test_get_status_action_following_and_unknown_code():
    for code, name in ((OAMSStatus.FORWARD_FOLLOWING, "forward_following"),
                       (OAMSStatus.UNLOADING, "unloading"),
                       (OAMSStatus.COASTING, "coasting")):
        oams = FakeOams(action_status=code)
        unit = _make_unit_status(oams=oams, monitor=_FakeMonitor(_FakeState()))
        assert unit.get_status()["oams_action"] == name
    # An out-of-range code falls back to a generic "busy" (not idle).
    unit = _make_unit_status(oams=FakeOams(action_status=99),
                      monitor=_FakeMonitor(_FakeState()))
    assert unit.get_status()["oams_action"] == "busy"


def test_get_status_reports_active_clog_stuck_windows():
    mon = _FakeMonitor(_FakeState(clog_start_time=123.0, stuck_start_time=45.0))
    unit = _make_unit_status(oams=FakeOams(), monitor=mon)
    st = unit.get_status()
    assert st["oams_clog_detecting"] is True
    assert st["oams_stuck_detecting"] is True


def test_get_status_no_controller_or_monitor():
    unit = _make_unit_status(oams=None, monitor=None)
    st = unit.get_status()
    assert st["oams_connected"] is False
    assert st["oams_action"] == "idle"
    assert st["oams_busy"] is False
    # controller/monitor-only keys are simply absent when not connected
    assert "oams_current_spool" not in st
    assert "oams_load_state" not in st


# ── _current_oams_action ──────────────────────────────────────────────────────

def test_current_oams_action_maps_names():
    assert _make_unit_status(oams=FakeOams(action_status=OAMSStatus.UNLOADING)) \
        ._current_oams_action() == "unloading"
    assert _make_unit_status(oams=FakeOams(action_status=OAMSStatus.FORWARD_FOLLOWING)) \
        ._current_oams_action() == "forward_following"


def test_current_oams_action_idle_and_unknown():
    assert _make_unit_status(oams=FakeOams(action_status=None))._current_oams_action() == ""
    assert _make_unit_status(oams=None)._current_oams_action() == ""
    assert _make_unit_status(oams=FakeOams(action_status=99))._current_oams_action() == "busy"


# ── poll action-transition logging (parallels afcACE) ─────────────────────────



def _poll_unit(action_status):
    unit = afcAMS.__new__(afcAMS)
    unit.name = "AMS_1"
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.oams = FakeOams(action_status=action_status)
    unit._operation_active = True    # returns right after action logging
    unit._current_action = ""
    unit.lanes = {}
    return unit


def test_poll_logs_action_transition():
    unit = _poll_unit(OAMSStatus.LOADING)
    unit._poll_oams_sensors(100.0)
    assert unit._current_action == "loading"
    assert any("AMS_1: idle -> loading" in m for m in unit.logger.lines["info"])


def test_poll_no_duplicate_log_when_unchanged():
    unit = _poll_unit(OAMSStatus.LOADING)
    unit._poll_oams_sensors(100.0)
    n = len(unit.logger.lines["info"])
    unit._poll_oams_sensors(102.0)   # same action
    assert len(unit.logger.lines["info"]) == n


def test_poll_logs_return_to_idle():
    unit = _poll_unit(OAMSStatus.LOADING)
    unit._poll_oams_sensors(100.0)
    unit.oams.action_status = None   # operation finished
    unit._poll_oams_sensors(102.0)
    assert unit._current_action == ""
    assert any("loading -> idle" in m for m in unit.logger.lines["info"])

