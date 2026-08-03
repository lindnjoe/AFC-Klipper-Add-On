"""
Unit tests for the BigTreeTech MMS / ViViD RFID host module
(extras/AFC_Vivid_rfid.py):
  - _VividSpiRegLink MFRC522 SPI framing (read/write address bytes)
  - lane_slot_map parsing + _on_ready slot->reader indexing
  - read_slot delegates to the shared read_tag with the reader's link
  - _map turns a read_tag result into AFC slot_info (single + dual colour)
The MFRC522/MIFARE/decode stack is exercised by test_AFC_ACE2_rfid; here the
transport-agnostic read_tag is monkeypatched so no hardware/SPI is needed.
"""
from __future__ import annotations

import importlib.util
import logging
import os
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def _load(name, relpath):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


vivid = _load("AFC_Vivid_rfid", "extras/AFC_Vivid_rfid.py")


# ── stubs ─────────────────────────────────────────────────────────────────────

class _CmdError(Exception):
    pass


class _Reactor:
    NEVER = float("inf")

    def __init__(self):
        self.timers = []

    def monotonic(self):
        return 0.0

    def register_timer(self, callback, waketime=None):
        self.timers.append(callback)
        return callback                                # handle == the callback

    def unregister_timer(self, handle):
        if handle in self.timers:
            self.timers.remove(handle)


class _Gcode:
    def __init__(self):
        self.commands = {}
        self.info = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def respond_info(self, msg, log=True):
        self.info.append(msg)


class _Printer:
    command_error = _CmdError

    def __init__(self, objects=None):
        self.reactor = _Reactor()
        self.gcode = _Gcode()
        self.events = []
        self._objects = dict(objects or {})

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        return self._objects.get(name, default)

    def lookup_objects(self, module=None):
        return list(self._objects.items())

    def register_event_handler(self, name, cb):
        self.events.append((name, cb))


class _Config:
    def __init__(self, printer, opts):
        self._printer = printer
        self._opts = opts

    def get_printer(self):
        return self._printer

    def get(self, key, default=None):
        return self._opts.get(key, default)

    def getboolean(self, key, default=None):
        return bool(self._opts.get(key, default))

    def getfloat(self, key, default=None, minval=None, maxval=None):
        return float(self._opts.get(key, default))

    def getint(self, key, default=None, minval=None, maxval=None):
        return int(self._opts.get(key, default))

    def error(self, msg):
        return _CmdError(msg)


class _GCmd:
    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=0):
        v = self.params.get(key, default)
        return None if v is None else int(v)

    def error(self, msg):
        return _CmdError(msg)

    def respond_info(self, msg):
        self.responses.append(msg)


class _FakeReader(vivid.AFC_Vivid_rfid_reader):
    """Stand-in for AFC_Vivid_rfid_reader that skips the real __init__ (which
    builds a Klipper MCU_SPI). Subclasses the real type so the coordinator's
    isinstance() check accepts it."""
    def __init__(self, name, slots):
        self.name = name
        self.slots = list(slots)
        self.link = object()                          # identity checked in tests


def _make(opts=None, readers=None):
    """Build AFC_Vivid_rfid through its real __init__, then register fake reader
    objects and fire _on_ready to index them."""
    objs = {}
    for r in (readers or []):
        objs["AFC_Vivid_rfid %s" % r.name] = r
    printer = _Printer(objs)
    obj = vivid.AFC_Vivid_rfid(_Config(printer, dict(opts or {})))
    obj.logger = logging.getLogger("test.vivid_rfid")
    return obj, printer


def _fire_ready(obj, printer):
    for name, cb in printer.events:
        if name == "klippy:ready":
            cb()


# ── _VividSpiRegLink SPI framing ──────────────────────────────────────────────

class _SpySpi:
    def __init__(self, read_byte=0x37):
        self.sent = []
        self.transfers = []
        self._read_byte = read_byte

    def spi_send(self, data):
        self.sent.append(list(data))

    def spi_transfer(self, data):
        self.transfers.append(list(data))
        # MFRC522 returns the value in the 2nd byte; first is discarded.
        return {"response": [0x00, self._read_byte]}


def test_reg_write_address_byte_is_reg_shifted_left():
    spi = _SpySpi()
    link = vivid._VividSpiRegLink(spi)
    link.reg_write(0x11, 0x3D)                        # ModeReg
    assert spi.sent == [[0x11 << 1, 0x3D]]            # write MSB clear


def test_reg_read_address_byte_has_read_bit_and_returns_second_byte():
    spi = _SpySpi(read_byte=0x42)
    link = vivid._VividSpiRegLink(spi)
    val = link.reg_read(0x37)                         # VersionReg
    assert spi.transfers == [[0x80 | (0x37 << 1), 0x00]]
    assert val == 0x42


def test_reader_power_is_noop():
    link = vivid._VividSpiRegLink(_SpySpi())
    assert link.reader_power(True) is None            # no dedicated power line


# ── lane_slot_map + reader indexing ───────────────────────────────────────────

def test_lane_slot_map_parsed():
    obj, _ = _make({"lane_slot_map": "lane0:0, lane1:1, lane2:2, lane3:3"})
    assert obj._get_slot("lane2") == 2
    assert obj._get_slot("nope") is None


def test_bad_lane_slot_map_raises():
    with pytest.raises(_CmdError):
        _make({"lane_slot_map": "lane0"})             # missing ':slot'


def test_on_ready_indexes_slots_to_readers():
    r0 = _FakeReader("reader0", [0, 1])
    r1 = _FakeReader("reader1", [2, 3])
    obj, printer = _make({"lane_slot_map": "lane0:0, lane3:3"}, readers=[r0, r1])
    _fire_ready(obj, printer)
    assert obj._slot_reader[0] is r0 and obj._slot_reader[1] is r0
    assert obj._slot_reader[2] is r1 and obj._slot_reader[3] is r1
    assert obj.get_status()["slots"] == [0, 1, 2, 3]


# ── read_slot delegates to read_tag with the right reader link ────────────────

def test_read_slot_uses_the_slots_reader_link(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    r1 = _FakeReader("reader1", [2, 3])
    obj, printer = _make(readers=[r0, r1])
    _fire_ready(obj, printer)

    seen = {}

    def fake_read_tag(link, **kw):
        seen["link"] = link
        return {"uid": "aabb", "filament": {"type": "PLA", "manufacturer": "BQ Tech"}}

    monkeypatch.setattr(vivid, "read_tag", fake_read_tag)
    tag = obj.read_slot(3)
    assert seen["link"] is r1.link                    # slot 3 -> reader1
    assert tag["filament"]["type"] == "PLA"


def test_read_slot_none_when_no_reader():
    obj, printer = _make()
    _fire_ready(obj, printer)                          # no readers configured
    assert obj.read_slot(0) is None


class _SharedKeys:
    def __init__(self, bambu=None, creality=None, creality_enc=None):
        self.bambu_master_key = bambu
        self.creality_key = creality
        self.creality_encryption_key = creality_enc


def test_shared_keys_fallback_fills_unset_key():
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make(readers=[r0])                 # no bambu key configured
    printer._objects["AFC_rfid_keys"] = _SharedKeys(bambu=b"\xaa\xbb")
    _fire_ready(obj, printer)
    assert obj.bambu_master_key == b"\xaa\xbb"          # pulled from shared section


def test_shared_keys_do_not_override_own_key():
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"bambu_master_key": "0011"}, readers=[r0])
    printer._objects["AFC_rfid_keys"] = _SharedKeys(bambu=b"\xaa")
    _fire_ready(obj, printer)
    assert obj.bambu_master_key == bytes.fromhex("0011")  # local key wins


def test_read_slot_passes_configured_brand_keys(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"bambu_master_key": "00112233445566778899aabbccddeeff"},
                         readers=[r0])
    _fire_ready(obj, printer)
    captured = {}

    def fake_read_tag(link, bambu_master_key=None, **kw):
        captured["bambu"] = bambu_master_key
        return None

    monkeypatch.setattr(vivid, "read_tag", fake_read_tag)
    obj.read_slot(0)
    assert captured["bambu"] == bytes.fromhex("00112233445566778899aabbccddeeff")


# ── _map: read_tag result -> slot_info ────────────────────────────────────────

def test_map_single_color_btt():
    obj, _ = _make()
    tag = {"uid": "d13fdb0e", "tag_type": "MifareClassic1k",
           "filament": {"manufacturer": "BQ Tech", "type": "PET",
                        "detailed": "PET (CEP)", "color_argb": 0xFFC0FFEE,
                        "diameter_mm": 1.75, "bed_temp_c": 60,
                        "hotend_min_c": 200, "hotend_max_c": 240}}
    si = obj._map(tag)
    assert si["brand"] == "BQ Tech" and si["material"] == "PET"
    assert si["color_hex"] == "c0ffee"
    assert si["multi_color"] == ["c0ffee"] and si["is_dual_color"] is False
    assert si["extruder_temp"] == 220                 # (200+240)//2
    assert si["bed_temp"] == 60 and si["uid"] == "d13fdb0e"


def test_map_dual_color_bambu():
    obj, _ = _make()
    tag = {"uid": "7bf0afff",
           "filament": {"manufacturer": "Bambu", "type": "PLA",
                        "color_argb": 0xFFE7C1D5,
                        "colors_argb": [0xFFE7C1D5, 0xFF8EC9E9]}}
    si = obj._map(tag)
    assert si["multi_color"] == ["e7c1d5", "8ec9e9"]
    assert si["is_dual_color"] is True


def test_map_empty_when_no_filament():
    obj, _ = _make()
    si = obj._map({"uid": "aa", "filament": None})
    assert si["brand"] == "" and si["multi_color"] == []
    assert si["is_dual_color"] is False


# ── gcode command registration ────────────────────────────────────────────────

def test_read_command_registered():
    obj, _ = _make()
    assert "VIVID_RFID_READ" in obj.gcode.commands


def test_read_command_requires_lane_or_slot():
    obj, printer = _make()
    _fire_ready(obj, printer)
    with pytest.raises(_CmdError):
        obj.cmd_VIVID_RFID_READ(_GCmd({}))


# ── shared-antenna dedup (two slots per reader) ───────────────────────────────

def test_sibling_slot_is_the_other_slot_on_the_reader():
    r0 = _FakeReader("reader0", [0, 1])
    r1 = _FakeReader("reader1", [2, 3])
    obj, printer = _make(readers=[r0, r1])
    _fire_ready(obj, printer)
    assert obj._sibling_slot(0) == 1 and obj._sibling_slot(1) == 0
    assert obj._sibling_slot(2) == 3 and obj._sibling_slot(3) == 2


def test_read_slot_excludes_known_sibling_tag(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make(readers=[r0])
    _fire_ready(obj, printer)
    # Slot 1 already read a spool; its UID must be halted when reading slot 0.
    obj._slot_uid[1] = "AABBCCDD"
    captured = {}

    def fake_read_tag(link, is_excluded=None, **kw):
        captured["excluder"] = is_excluded
        return None

    monkeypatch.setattr(vivid, "read_tag", fake_read_tag)
    obj.read_slot(0)
    ex = captured["excluder"]
    assert ex is not None
    assert ex("aabbccdd") is True                     # sibling's tag -> halted
    assert ex("11223344") is False                    # a different tag -> kept


def test_read_slot_no_excluder_without_sibling_uid(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make(readers=[r0])
    _fire_ready(obj, printer)
    captured = {}
    monkeypatch.setattr(vivid, "read_tag",
                        lambda link, is_excluded=None, **kw: captured.setdefault(
                            "excluder", is_excluded))
    obj.read_slot(0)                                   # sibling slot 1 unknown
    assert captured["excluder"] is None


def test_read_slot_remembers_uid_for_future_dedup(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make(readers=[r0])
    _fire_ready(obj, printer)
    monkeypatch.setattr(vivid, "read_tag",
                        lambda link, **kw: {"uid": "DEADBEEF",
                                            "filament": {"type": "PLA"}})
    obj.read_slot(0)
    assert obj._slot_uid[0] == "DEADBEEF"
    # Now reading the sibling slot 1 must exclude slot 0's remembered tag.
    ex = obj._sibling_excluder(1)
    assert ex is not None and ex("deadbeef") is True


def test_sibling_excluder_skipped_when_sibling_spool_removed(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"}, readers=[r0])
    _fire_ready(obj, printer)
    obj._slot_uid[1] = "AABBCCDD"                      # sibling had a tag...
    lane1 = _Lane("lane1")
    lane1.prep_state = False                           # ...but its spool is gone
    obj.afc = _AFCStub(lanes={"lane1": lane1})
    assert obj._sibling_excluder(0) is None            # stale UID not halted


# ── stage-read: concurrent reader poll during the untouched load feed ─────────

class _Lane:
    def __init__(self, name):
        self.name = name
        self.prep_state = True
        self.raw_load_state = False


class _AFCStub:
    def __init__(self, lanes=None, spoolman=None):
        self.lanes = lanes or {}
        self.spoolman = spoolman


def test_stage_read_handlers_registered():
    obj, printer = _make()
    names = {n for n, _ in printer.events}
    assert {"afc_vivid:stage_read_begin", "afc_vivid:stage_read_end"} <= names


def test_stage_read_begin_starts_poll_for_mapped_lane():
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
    _fire_ready(obj, printer)
    obj._stage_read_begin(_Lane("lane0"))
    assert obj._probe["slot"] == 0
    assert obj._poll_timer is not None                 # a reactor timer is armed
    assert obj._stage_poll in printer.reactor.timers


def test_stage_read_begin_ignores_lane_without_reader():
    obj, printer = _make({"lane_slot_map": "lane0:0"})   # no reader sections
    _fire_ready(obj, printer)
    obj._stage_read_begin(_Lane("lane0"))
    assert obj._probe is None and obj._poll_timer is None


def test_stage_read_begin_disabled_by_config():
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0", "stage_read": False},
                         readers=[r0])
    _fire_ready(obj, printer)
    obj._stage_read_begin(_Lane("lane0"))
    assert obj._probe is None and obj._poll_timer is None


def test_stage_poll_detects_aborts_reads_and_applies(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0", "stage_confirm_reads": 2,
                          "stage_poll_interval": 0.1}, readers=[r0])
    _fire_ready(obj, printer)
    lane = _Lane("lane0")
    obj.afc = _AFCStub(lanes={"lane0": lane})
    applied = []
    aborts = []
    monkeypatch.setattr(obj, "apply_to_lane",
                        lambda l, t: applied.append((l, t)))
    monkeypatch.setattr(obj, "_detect_uid", lambda slot: "AA")   # tag in range
    monkeypatch.setattr(obj, "_abort_feed", lambda ln: aborts.append(ln))
    monkeypatch.setattr(vivid, "read_tag",
                        lambda link, **kw: {"uid": "AA",
                                            "filament": {"type": "PLA"}})
    obj._stage_read_begin(lane)
    nxt = obj._stage_poll(0.0)
    assert aborts == ["lane0"]                          # detect -> abort the feed
    assert len(applied) == 1                            # read + confirmed + applied
    assert nxt == printer.reactor.NEVER                 # stops polling
    assert obj._probe["done"] is True


def test_stage_poll_no_tag_in_range_keeps_polling(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0", "stage_poll_interval": 0.2},
                         readers=[r0])
    _fire_ready(obj, printer)
    aborts = []
    monkeypatch.setattr(obj, "_detect_uid", lambda slot: None)   # nothing yet
    monkeypatch.setattr(obj, "_abort_feed", lambda ln: aborts.append(ln))
    obj._stage_read_begin(_Lane("lane0"))
    nxt = obj._stage_poll(1.0)
    assert aborts == [] and nxt == 1.2                  # no detect -> no abort
    assert not obj._probe.get("done")


def test_stage_poll_gives_up_after_max_aborts(monkeypatch):
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0", "stage_max_aborts": 2},
                         readers=[r0])
    _fire_ready(obj, printer)
    monkeypatch.setattr(obj, "_detect_uid", lambda slot: "AA")
    monkeypatch.setattr(obj, "_abort_feed", lambda ln: None)
    monkeypatch.setattr(vivid, "read_tag",           # detected but never decodes
                        lambda link, **kw: {"uid": "AA", "filament": None})
    obj._stage_read_begin(_Lane("lane0"))
    assert obj._stage_poll(0.0) != printer.reactor.NEVER   # abort 1 — keep trying
    assert obj._stage_poll(0.1) == printer.reactor.NEVER   # abort 2 — give up
    assert obj._probe["done"] is True


def test_stage_read_end_tears_down_the_poll():
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
    _fire_ready(obj, printer)
    obj._stage_read_begin(_Lane("lane0"))
    assert obj._probe is not None
    obj._stage_read_end(_Lane("lane0"))
    assert obj._probe is None and obj._poll_timer is None
    assert obj._stage_poll not in printer.reactor.timers


def test_stage_sibling_left_alone_when_antenna_clear(monkeypatch):
    # Sibling has a spool, but its tag is NOT on the shared reader -> don't move it.
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"}, readers=[r0])
    _fire_ready(obj, printer)
    obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0"), "lane1": _Lane("lane1")})
    calls = []
    monkeypatch.setattr(obj, "_retract_sibling", lambda p: calls.append(p))
    monkeypatch.setattr(obj, "_parked_tag", lambda slot: None)   # antenna clear
    obj._stage_read_begin(_Lane("lane0"))
    assert calls == []                               # sibling not touched
    assert obj._probe.get("baseline") is None


def test_stage_sibling_cleared_when_its_tag_parked(monkeypatch):
    # Sibling's tag IS on the shared reader -> clear it; when it can't be moved
    # (mock leaves sib_lane unset) the parked UID is excluded via baseline.
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"}, readers=[r0])
    _fire_ready(obj, printer)
    obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0"), "lane1": _Lane("lane1")})
    calls = []
    monkeypatch.setattr(obj, "_retract_sibling", lambda p: calls.append(p))
    monkeypatch.setattr(obj, "_parked_tag", lambda slot: "CAFE")   # sibling parked
    obj._stage_read_begin(_Lane("lane0"))
    assert len(calls) == 1                           # cleared the sibling
    assert obj._probe.get("baseline") == "CAFE"      # unmovable -> excluded


def test_stage_poll_skips_parked_baseline_sibling(monkeypatch):
    # With a baseline set (unmovable parked sibling), the poll must NOT abort/read
    # on that UID — it keeps polling for our own tag.
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0", "stage_poll_interval": 0.2},
                         readers=[r0])
    _fire_ready(obj, printer)
    aborts = []
    monkeypatch.setattr(obj, "_abort_feed", lambda ln: aborts.append(ln))
    monkeypatch.setattr(obj, "_detect_uid", lambda slot: "CAFE")   # sibling UID
    obj._stage_read_begin(_Lane("lane0"))
    obj._probe["baseline"] = "CAFE"
    nxt = obj._stage_poll(1.0)
    assert aborts == [] and nxt == 1.2               # baseline UID never accepted
    assert not obj._probe.get("done")


def test_stage_hint_when_unmovable_sister_blocks_and_no_read(monkeypatch):
    # Unmovable sister parked on the reader + no tag read for our lane -> the feed
    # end tells the user to move that spool or set the id by hand.
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"}, readers=[r0])
    _fire_ready(obj, printer)
    obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0"), "lane1": _Lane("lane1")})
    monkeypatch.setattr(obj, "_retract_sibling", lambda p: None)   # can't move it
    monkeypatch.setattr(obj, "_parked_tag", lambda slot: "CAFE")   # sibling parked
    obj._stage_read_begin(_Lane("lane0"))
    assert obj._probe.get("blocked_sib") == "lane1"
    obj._stage_read_end(_Lane("lane0"))               # feed ended, no read
    assert any("Manually move" in m and "lane1" in m for m in printer.gcode.info)


def test_stage_no_hint_when_read_succeeded(monkeypatch):
    # Blocked sister but our own tag WAS read -> no hint.
    r0 = _FakeReader("reader0", [0, 1])
    obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"}, readers=[r0])
    _fire_ready(obj, printer)
    obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0"), "lane1": _Lane("lane1")})
    monkeypatch.setattr(obj, "_retract_sibling", lambda p: None)
    monkeypatch.setattr(obj, "_parked_tag", lambda slot: "CAFE")
    obj._stage_read_begin(_Lane("lane0"))
    obj._probe["read_ok"] = True                      # our tag was read
    obj._stage_read_end(_Lane("lane0"))
    assert not any("Manually move" in m for m in printer.gcode.info)


# ── get_status: last_reads records ────────────────────────────────────────────

class TestVividGetStatusLastReads:
    def test_empty_before_first_read(self):
        obj, _ = _make()
        assert obj.get_status()["last_reads"] == {}

    def test_read_lane_decoded_records_slot_info(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _fire_ready(obj, printer)
        tag = {"uid": "AABB", "tag_type": "MifareClassic1k",
               "filament": {"type": "PLA", "manufacturer": "Elegoo"}}
        monkeypatch.setattr(obj, "read_slot", lambda s: tag)
        si = obj.read_lane("lane0")
        assert si["material"] == "PLA"
        rec = obj.get_status()["last_reads"]["lane0"]
        assert rec["decoded"] is True
        assert rec["material"] == "PLA"
        assert rec["uid"] == "AABB"


# ── read_lane: seen-but-undecoded recording ───────────────────────────────────

class TestVividReadLaneRecording:
    def test_undecoded_tag_recorded_with_uid(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _fire_ready(obj, printer)
        monkeypatch.setattr(
            obj, "read_slot",
            lambda s: {"uid": "AABB", "tag_type": "MifareClassic1k",
                       "filament": None})
        assert obj.read_lane("lane0") is None
        rec = obj.get_status()["last_reads"]["lane0"]
        assert rec["decoded"] is False
        assert rec["uid"] == "AABB"
        assert rec["tag_type"] == "MifareClassic1k"

    def test_no_tag_at_all_records_nothing(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _fire_ready(obj, printer)
        monkeypatch.setattr(obj, "read_slot", lambda s: None)
        assert obj.read_lane("lane0") is None
        assert obj.get_status()["last_reads"] == {}
