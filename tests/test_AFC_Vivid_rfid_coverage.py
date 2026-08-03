"""
Branch-coverage tests for extras/AFC_Vivid_rfid.py, complementing
tests/test_AFC_Vivid_rfid.py. One test class per method under test; each log
line and changed attribute is asserted exactly. read_tag / MFRC522 hardware is
monkeypatched so no SPI is needed.
"""
from __future__ import annotations

import importlib.util
import os
import sys
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


class _Logger:
    def __init__(self):
        self.messages = []

    def _log(self, level, msg, args):
        self.messages.append((level, msg % args if args else msg))

    def info(self, msg, *args):
        self._log("info", msg, args)

    def warning(self, msg, *args):
        self._log("warning", msg, args)

    def error(self, msg, *args):
        self._log("error", msg, args)

    def debug(self, msg, *args):
        self._log("debug", msg, args)


class _Reactor:
    NEVER = float("inf")

    def __init__(self):
        self.timers = []

    def monotonic(self):
        return 0.0

    def register_timer(self, callback, waketime=None):
        self.timers.append(callback)
        return callback

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
    def __init__(self, printer, opts, name="AFC_Vivid_rfid"):
        self._printer = printer
        self._opts = opts
        self._name = name

    def get_printer(self):
        return self._printer

    def get_name(self):
        return self._name

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


class _Lane:
    def __init__(self, name):
        self.name = name
        self.prep_state = True
        self.raw_load_state = False


class _MoveLane:
    def __init__(self, name, prep_state=True, tool_loaded=False):
        self.name = name
        self.prep_state = prep_state
        self.tool_loaded = tool_loaded
        self.moves = []

    def move_to(self, dist, speed, assist_active=None, use_homing=None):
        self.moves.append((dist, speed, assist_active, use_homing))


class _AFCStub:
    def __init__(self, lanes=None, spoolman=None):
        self.lanes = lanes or {}
        self.spoolman = spoolman


class _FakeReader(vivid.AFC_Vivid_rfid_reader):
    def __init__(self, name, slots):
        self.name = name
        self.slots = list(slots)
        self.link = object()


# ── enum + MFRC522 patching helpers ───────────────────────────────────────────

class _MoveDir:
    NEG = -1
    POS = 1


class _Speed:
    SHORT = "SHORT"


class _Assist:
    NO = "NO"


def _patch_enums(monkeypatch):
    monkeypatch.setattr(vivid, "MoveDirection", _MoveDir)
    monkeypatch.setattr(vivid, "SpeedMode", _Speed)
    monkeypatch.setattr(vivid, "AssistActive", _Assist)


def _patch_mifare(monkeypatch, uid=None, raise_exc=False, captured=None):
    monkeypatch.setattr(vivid, "Mfrc522", lambda link: link)

    class _MC:
        def __init__(self, dev):
            self.dev = dev

        def activate(self, is_excluded=None):
            if captured is not None:
                captured["is_excluded"] = is_excluded
            if raise_exc:
                raise RuntimeError("boom")
            return uid, 0x08

    monkeypatch.setattr(vivid, "MifareClassic", _MC)


def _make(opts=None, readers=None):
    objs = {}
    for r in (readers or []):
        objs[f"AFC_Vivid_rfid {r.name}"] = r
    printer = _Printer(objs)
    obj = vivid.AFC_Vivid_rfid(_Config(printer, dict(opts or {})))
    obj.logger = _Logger()
    return obj, printer


def _fire_ready(obj, printer):
    for name, cb in printer.events:
        if name == "klippy:ready":
            cb()


def _ready(obj, printer):
    _fire_ready(obj, printer)
    obj.logger.messages.clear()


def _boom(*args, **kwargs):
    raise RuntimeError("boom")


# ── AFC_Vivid_rfid_reader.__init__ ────────────────────────────────────────────

class TestVividReaderInit:
    def _cfg(self, monkeypatch, opts, name="AFC_Vivid_rfid reader0"):
        fake_bus = types.ModuleType("bus")
        spi = object()
        seen = {}

        def _mk(config, mode, pin_option=None, default_speed=None,
                cs_active_high=None):
            seen.update(mode=mode, pin_option=pin_option,
                        speed=default_speed, cs_active_high=cs_active_high)
            return spi

        fake_bus.MCU_SPI_from_config = _mk
        monkeypatch.setitem(sys.modules, "bus", fake_bus)
        return _Config(_Printer(), opts, name=name), spi, seen

    def test_parses_slots_and_builds_link(self, monkeypatch):
        cfg, spi, seen = self._cfg(monkeypatch, {"slots": "0, 1"})
        r = vivid.AFC_Vivid_rfid_reader(cfg)
        assert r.name == "reader0"
        assert r.slots == [0, 1]
        assert r.spi is spi
        assert isinstance(r.link, vivid._VividSpiRegLink)
        assert r.link.spi is spi
        assert seen["pin_option"] == "cs_pin"
        assert seen["mode"] == vivid._MFRC522_SPI_MODE
        assert seen["speed"] == vivid._MFRC522_SPI_SPEED
        assert seen["cs_active_high"] is False

    def test_blank_slot_entries_skipped(self, monkeypatch):
        cfg, _spi, _seen = self._cfg(monkeypatch, {"slots": " , 2 , "})
        r = vivid.AFC_Vivid_rfid_reader(cfg)
        assert r.slots == [2]

    def test_empty_slots_gives_empty_list(self, monkeypatch):
        cfg, _spi, _seen = self._cfg(monkeypatch, {"slots": ""})
        r = vivid.AFC_Vivid_rfid_reader(cfg)
        assert r.slots == []

    def test_bad_slot_number_raises(self, monkeypatch):
        cfg, _spi, _seen = self._cfg(monkeypatch, {"slots": "0, x"})
        with pytest.raises(_CmdError):
            vivid.AFC_Vivid_rfid_reader(cfg)


# ── AFC_Vivid_rfid.__init__ ───────────────────────────────────────────────────

class TestVividInit:
    def test_bad_slot_number_in_lane_slot_map_raises(self):
        with pytest.raises(_CmdError):
            _make({"lane_slot_map": "lane0:x"})


# ── _on_ready ─────────────────────────────────────────────────────────────────

class TestVividOnReady:
    def test_resolves_keys_when_helper_present(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        monkeypatch.setattr(
            vivid, "resolve_rfid_keys",
            lambda pr, b, c, ce: (b"\x01", b"\x02", b"\x03"))
        _fire_ready(obj, printer)
        assert obj.bambu_master_key == b"\x01"
        assert obj.creality_key == b"\x02"
        assert obj.creality_encryption_key == b"\x03"

    def test_skips_resolve_when_helper_absent(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"bambu_master_key": "0011"}, readers=[r0])
        monkeypatch.setattr(vivid, "resolve_rfid_keys", None)
        _fire_ready(obj, printer)
        assert obj.bambu_master_key == bytes.fromhex("0011")

    def test_duplicate_slot_warns_and_keeps_first(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        r1 = _FakeReader("reader1", [0, 2])
        obj, printer = _make(readers=[r0, r1])
        monkeypatch.setattr(vivid, "resolve_rfid_keys", None)
        _fire_ready(obj, printer)
        assert obj._slot_reader[0] is r0
        assert obj._slot_reader[2] is r1
        assert obj.logger.messages == [
            ("warning", "AFC_Vivid_rfid: slot 0 served by more than one "
                        "reader; keeping reader0"),
            ("info", "AFC_Vivid_rfid: 3 slot(s) mapped across 2 reader(s)")]


# ── _sibling_slot ─────────────────────────────────────────────────────────────

class TestVividSiblingSlot:
    def test_returns_other_slot(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        assert obj._sibling_slot(0) == 1
        assert obj._sibling_slot(1) == 0

    def test_none_when_no_reader(self):
        obj, printer = _make()
        _ready(obj, printer)
        assert obj._sibling_slot(7) is None

    def test_none_when_multiple_others(self):
        r0 = _FakeReader("reader0", [0, 1, 2])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        assert obj._sibling_slot(0) is None

    def test_none_when_single_slot_reader(self):
        r0 = _FakeReader("reader0", [5])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        assert obj._sibling_slot(5) is None


# ── _sibling_has_spool ────────────────────────────────────────────────────────

class TestVividSiblingHasSpool:
    def _obj(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane1:1"}, readers=[r0])
        _ready(obj, printer)
        return obj

    def test_true_when_sib_not_mapped(self):
        obj = self._obj()
        obj.afc = _AFCStub(lanes={"lane1": _Lane("lane1")})
        assert obj._sibling_has_spool(9) is True

    def test_true_when_afc_none(self):
        obj = self._obj()
        obj.afc = None
        assert obj._sibling_has_spool(1) is True

    def test_true_when_afc_has_no_lanes(self):
        obj = self._obj()
        obj.afc = object()
        assert obj._sibling_has_spool(1) is True

    def test_true_when_lane_missing(self):
        obj = self._obj()
        obj.afc = _AFCStub(lanes={})
        assert obj._sibling_has_spool(1) is True

    def test_true_when_prep_state_true(self):
        obj = self._obj()
        lane = _Lane("lane1")
        lane.prep_state = True
        obj.afc = _AFCStub(lanes={"lane1": lane})
        assert obj._sibling_has_spool(1) is True

    def test_false_when_prep_state_false(self):
        obj = self._obj()
        lane = _Lane("lane1")
        lane.prep_state = False
        obj.afc = _AFCStub(lanes={"lane1": lane})
        assert obj._sibling_has_spool(1) is False


# ── _sibling_excluder ─────────────────────────────────────────────────────────

class TestVividSiblingExcluder:
    def _obj(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"},
                             readers=[r0])
        _ready(obj, printer)
        obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0"),
                                  "lane1": _Lane("lane1")})
        return obj

    def test_none_when_no_sibling(self):
        r0 = _FakeReader("reader0", [5])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        assert obj._sibling_excluder(5) is None

    def test_none_when_sibling_uid_unknown(self):
        obj = self._obj()
        assert obj._sibling_excluder(0) is None

    def test_none_when_sibling_spool_removed(self):
        obj = self._obj()
        obj._slot_uid[1] = "AABB"
        obj.afc.lanes["lane1"].prep_state = False
        assert obj._sibling_excluder(0) is None

    def test_predicate_halts_sibling_uid(self):
        obj = self._obj()
        obj._slot_uid[1] = "AABBCCDD"
        ex = obj._sibling_excluder(0)
        assert ex is not None
        assert ex("aabbccdd") is True
        assert ex("AABBCCDD") is True
        assert ex("11223344") is False
        assert ex(None) is False


# ── _excluder_with ────────────────────────────────────────────────────────────

class TestVividExcluderWith:
    def _obj(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"},
                             readers=[r0])
        _ready(obj, printer)
        obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0"),
                                  "lane1": _Lane("lane1")})
        return obj

    def test_returns_base_when_no_extra(self):
        obj = self._obj()
        obj._slot_uid[1] = "AABB"
        result = obj._excluder_with(0, None)
        assert result is not None
        assert result("aabb") is True and result("beef") is False

    def test_returns_none_base_when_no_extra_and_no_sibling(self):
        r0 = _FakeReader("reader0", [5])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        assert obj._excluder_with(5, None) is None

    def test_extra_only_when_base_none(self):
        r0 = _FakeReader("reader0", [5])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        ex = obj._excluder_with(5, "CAFE")
        assert ex is not None
        assert ex("cafe") is True
        assert ex("beef") is False
        assert ex(None) is False

    def test_extra_and_base_both_halt(self):
        obj = self._obj()
        obj._slot_uid[1] = "AABB"
        ex = obj._excluder_with(0, "CAFE")
        assert ex("cafe") is True
        assert ex("aabb") is True
        assert ex("beef") is False


# ── _parked_tag ───────────────────────────────────────────────────────────────

class TestVividParkedTag:
    def test_none_when_no_reader(self):
        obj, printer = _make()
        _ready(obj, printer)
        assert obj._parked_tag(9) is None

    def test_returns_hex_uid(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        _patch_mifare(monkeypatch, uid=b"\xca\xfe")
        assert obj._parked_tag(0) == "cafe"

    def test_none_when_uid_missing(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        _patch_mifare(monkeypatch, uid=None)
        assert obj._parked_tag(0) is None

    def test_none_on_activate_exception(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        _patch_mifare(monkeypatch, raise_exc=True)
        assert obj._parked_tag(0) is None


# ── read_lane ─────────────────────────────────────────────────────────────────

class TestVividReadLane:
    def test_warns_when_lane_unmapped(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        assert obj.read_lane("ghost") is None
        assert obj.logger.messages == [
            ("warning", "AFC_Vivid_rfid: lane 'ghost' has no slot "
                        "(set lane_slot_map)")]

    def test_records_when_afc_none(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _ready(obj, printer)
        obj.afc = None
        tag = {"uid": "AABB",
               "filament": {"type": "PLA", "manufacturer": "Elegoo"}}
        monkeypatch.setattr(obj, "read_slot", lambda s: tag)
        si = obj.read_lane("lane0")
        assert si["material"] == "PLA"
        assert obj._last[0] is tag
        assert obj.get_status()["last_reads"]["lane0"]["decoded"] is True

    def test_records_when_afc_has_no_lanes(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _ready(obj, printer)
        obj.afc = object()
        tag = {"uid": "AABB",
               "filament": {"type": "PLA", "manufacturer": "Elegoo"}}
        monkeypatch.setattr(obj, "read_slot", lambda s: tag)
        si = obj.read_lane("lane0")
        assert si["material"] == "PLA"

    def test_records_when_lane_not_in_afc(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _ready(obj, printer)
        obj.afc = _AFCStub(lanes={})
        tag = {"uid": "AABB",
               "filament": {"type": "PLA", "manufacturer": "Elegoo"}}
        monkeypatch.setattr(obj, "read_slot", lambda s: tag)
        applied = []
        monkeypatch.setattr(obj, "apply_to_lane",
                            lambda ln, t: applied.append(1))
        si = obj.read_lane("lane0")
        assert si["material"] == "PLA"
        assert applied == []

    def test_applies_when_lane_present(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _ready(obj, printer)
        lane = _Lane("lane0")
        obj.afc = _AFCStub(lanes={"lane0": lane})
        tag = {"uid": "AABB", "filament": {"type": "PLA"}}
        monkeypatch.setattr(obj, "read_slot", lambda s: tag)
        sentinel = {"applied": True}
        monkeypatch.setattr(obj, "apply_to_lane", lambda ln, t: sentinel)
        result = obj.read_lane("lane0")
        assert result is sentinel
        assert obj._last[0] is tag


# ── _stage_read_begin ─────────────────────────────────────────────────────────

class TestVividStageReadBegin:
    def _obj(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1"},
                             readers=[r0])
        _ready(obj, printer)
        obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0"),
                                  "lane1": _Lane("lane1")})
        return obj, printer

    def test_no_sibling_lane_skips_precheck(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _ready(obj, printer)
        obj._stage_read_begin(_Lane("lane0"))
        assert obj._probe is not None
        assert obj._poll_timer is not None
        assert obj._probe["baseline"] is None
        assert obj.logger.messages == []

    def test_parked_unmovable_sibling_sets_baseline_and_logs(self, monkeypatch):
        obj, printer = self._obj()
        monkeypatch.setattr(obj, "_retract_sibling", lambda p: None)
        monkeypatch.setattr(obj, "_parked_tag", lambda slot: "CAFE")
        obj._stage_read_begin(_Lane("lane0"))
        assert obj._probe["baseline"] == "CAFE"
        assert obj._probe["blocked_sib"] == "lane1"
        assert obj.logger.messages == [
            ("info", "ViViD RFID: tag CAFE parked on the shared reader for "
                     "slot 0 — clearing the sibling before the read")]

    def test_movable_sibling_no_baseline(self, monkeypatch):
        obj, printer = self._obj()

        def _retract(p):
            p["sib_lane"] = object()
            p["sib_dist"] = 75.0

        monkeypatch.setattr(obj, "_retract_sibling", _retract)
        monkeypatch.setattr(obj, "_parked_tag", lambda slot: "CAFE")
        obj._stage_read_begin(_Lane("lane0"))
        assert obj._probe["baseline"] is None
        assert obj._probe["blocked_sib"] is None
        assert obj.logger.messages == [
            ("info", "ViViD RFID: tag CAFE parked on the shared reader for "
                     "slot 0 — clearing the sibling before the read")]

    def test_precheck_exception_warns(self, monkeypatch):
        obj, printer = self._obj()
        monkeypatch.setattr(obj, "_parked_tag", _boom)
        obj._stage_read_begin(_Lane("lane0"))
        assert obj._probe is not None
        assert obj._poll_timer is not None
        assert obj.logger.messages == [
            ("warning", "ViViD RFID: sibling pre-check failed: boom")]

    def test_timer_register_failure_clears_probe(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _ready(obj, printer)
        monkeypatch.setattr(printer.reactor, "register_timer", _boom)
        obj._stage_read_begin(_Lane("lane0"))
        assert obj._probe is None
        assert obj.logger.messages == [
            ("warning", "ViViD RFID: could not start stage poll: boom")]


# ── _stage_poll ───────────────────────────────────────────────────────────────

class TestVividStagePoll:
    def test_returns_never_when_no_probe(self):
        obj, printer = _make()
        obj._probe = None
        assert obj._stage_poll(1.0) == printer.reactor.NEVER
        assert obj.logger.messages == []

    def test_returns_never_when_probe_done(self):
        obj, printer = _make()
        obj._probe = {"slot": 0, "lane": "lane0", "done": True}
        assert obj._stage_poll(1.0) == printer.reactor.NEVER
        assert obj.logger.messages == []

    def test_detect_exception_warns_and_keeps_polling(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0",
                              "stage_poll_interval": 0.2}, readers=[r0])
        _ready(obj, printer)
        obj._probe = {"slot": 0, "lane": "lane0", "done": False,
                      "baseline": None}
        monkeypatch.setattr(obj, "_detect_uid", _boom)
        assert obj._stage_poll(1.0) == pytest.approx(1.2)
        assert obj._probe["done"] is False
        assert obj.logger.messages == [
            ("warning", "ViViD RFID: detect error on slot 0: boom")]


# ── _detect_uid ───────────────────────────────────────────────────────────────

class TestVividDetectUid:
    def test_none_when_no_reader(self):
        obj, printer = _make()
        _ready(obj, printer)
        assert obj._detect_uid(5) is None

    def test_returns_hex_when_tag_present(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        cap = {}
        _patch_mifare(monkeypatch, uid=b"\xaa\xbb", captured=cap)
        assert obj._detect_uid(0) == "aabb"
        assert cap["is_excluded"] is None

    def test_none_when_no_uid(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        _patch_mifare(monkeypatch, uid=None)
        assert obj._detect_uid(0) is None

    def test_passes_sibling_excluder(self, monkeypatch):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make(readers=[r0])
        _ready(obj, printer)
        obj._slot_uid[1] = "DEAD"
        cap = {}
        _patch_mifare(monkeypatch, uid=b"\xaa", captured=cap)
        obj._detect_uid(0)
        assert callable(cap["is_excluded"])
        assert cap["is_excluded"]("dead") is True


# ── _read_confirmed ───────────────────────────────────────────────────────────

class TestVividReadConfirmed:
    def _obj(self, confirm=2):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0",
                              "stage_confirm_reads": confirm}, readers=[r0])
        _ready(obj, printer)
        return obj

    def test_read_slot_exception_warns_and_none(self, monkeypatch):
        obj = self._obj(confirm=1)

        def _rs(slot, extra_excluded=None):
            raise RuntimeError("boom")

        monkeypatch.setattr(obj, "read_slot", _rs)
        assert obj._read_confirmed(0) is None
        assert obj.logger.messages == [
            ("warning", "ViViD RFID: stage read error on slot 0: boom")]

    def test_none_when_no_tag(self, monkeypatch):
        obj = self._obj(confirm=1)
        monkeypatch.setattr(obj, "read_slot",
                            lambda s, extra_excluded=None: None)
        assert obj._read_confirmed(0) is None

    def test_none_when_no_filament(self, monkeypatch):
        obj = self._obj(confirm=1)
        monkeypatch.setattr(
            obj, "read_slot",
            lambda s, extra_excluded=None: {"uid": "A", "filament": None})
        assert obj._read_confirmed(0) is None

    def test_none_when_uid_inconsistent(self, monkeypatch):
        obj = self._obj(confirm=2)
        seq = [{"uid": "AA", "filament": {"type": "PLA"}},
               {"uid": "BB", "filament": {"type": "PLA"}}]
        monkeypatch.setattr(obj, "read_slot",
                            lambda s, extra_excluded=None: seq.pop(0))
        assert obj._read_confirmed(0) is None

    def test_returns_tag_when_consistent(self, monkeypatch):
        obj = self._obj(confirm=2)
        tag = {"uid": "AA", "filament": {"type": "PLA"}}
        monkeypatch.setattr(obj, "read_slot",
                            lambda s, extra_excluded=None: dict(tag))
        assert obj._read_confirmed(0) == tag

    def test_passes_baseline_as_extra_excluded(self, monkeypatch):
        obj = self._obj(confirm=1)
        cap = {}

        def _rs(s, extra_excluded=None):
            cap["extra"] = extra_excluded
            return {"uid": "AA", "filament": {"type": "PLA"}}

        monkeypatch.setattr(obj, "read_slot", _rs)
        obj._read_confirmed(0, baseline="BASE")
        assert cap["extra"] == "BASE"


# ── _abort_feed ───────────────────────────────────────────────────────────────

class _TrigCmd:
    def __init__(self, raise_exc=False):
        self.sent = []
        self._raise = raise_exc

    def send(self, data):
        if self._raise:
            raise RuntimeError("boom")
        self.sent.append(list(data))


class _Trsync:
    REASON_HOST_REQUEST = 4

    def __init__(self, oid, cmd):
        self._oid = oid
        self._trsync_trigger_cmd = cmd


class _Dispatch:
    def __init__(self, trsyncs):
        self._trsyncs = trsyncs


class _McuEndstop:
    def __init__(self, dispatch):
        self._dispatch = dispatch


class _QueryEndstops:
    def __init__(self, endstops):
        self.endstops = endstops


def _lane_with_endstop(name, es="load_es"):
    lane = _Lane(name)
    lane.load_endstop_name = es
    return lane


class TestVividAbortFeed:
    def test_noop_when_afc_none(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = None
        cmd = _TrigCmd()
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(_Dispatch([_Trsync(7, cmd)])), "load_es")])
        obj._abort_feed("lane0")
        assert cmd.sent == []
        assert obj.logger.messages == []

    def test_noop_when_afc_has_no_lanes(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = object()
        cmd = _TrigCmd()
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(_Dispatch([_Trsync(7, cmd)])), "load_es")])
        obj._abort_feed("lane0")
        assert cmd.sent == []

    def test_noop_when_lane_missing_endstop_name(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={"lane0": _Lane("lane0")})
        cmd = _TrigCmd()
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(_Dispatch([_Trsync(7, cmd)])), "load_es")])
        obj._abort_feed("lane0")
        assert cmd.sent == []

    def test_returns_when_query_endstops_absent(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={"lane0": _lane_with_endstop("lane0")})
        obj._abort_feed("lane0")
        assert obj.logger.messages == []

    def test_returns_when_endstop_not_matched(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={"lane0": _lane_with_endstop("lane0")})
        cmd = _TrigCmd()
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(_Dispatch([_Trsync(7, cmd)])), "other")])
        obj._abort_feed("lane0")
        assert cmd.sent == []

    def test_returns_when_dispatch_none(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={"lane0": _lane_with_endstop("lane0")})
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(None), "load_es")])
        obj._abort_feed("lane0")
        assert obj.logger.messages == []

    def test_returns_when_trsyncs_empty(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={"lane0": _lane_with_endstop("lane0")})
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(_Dispatch([])), "load_es")])
        obj._abort_feed("lane0")
        assert obj.logger.messages == []

    def test_success_sends_trigger(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={"lane0": _lane_with_endstop("lane0")})
        cmd = _TrigCmd()
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(_Dispatch([_Trsync(7, cmd)])), "load_es")])
        obj._abort_feed("lane0")
        assert cmd.sent == [[7, _Trsync.REASON_HOST_REQUEST]]
        assert obj.logger.messages == []

    def test_send_exception_logs_debug(self):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={"lane0": _lane_with_endstop("lane0")})
        cmd = _TrigCmd(raise_exc=True)
        printer._objects["query_endstops"] = _QueryEndstops(
            [(_McuEndstop(_Dispatch([_Trsync(7, cmd)])), "load_es")])
        obj._abort_feed("lane0")
        assert obj.logger.messages == [
            ("debug", "ViViD RFID: fake-trigger of load_es not available "
                      "(boom) — feeding to the real sensor instead")]


# ── _apply_staged ─────────────────────────────────────────────────────────────

class TestVividApplyStaged:
    def _p(self):
        return {"slot": 0, "lane": "lane0"}

    def test_caches_without_apply_when_afc_none(self, monkeypatch):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = None
        calls = []
        monkeypatch.setattr(obj, "apply_to_lane",
                            lambda ln, t: calls.append(1))
        tag = {"uid": "AA", "filament": {"type": "PLA"}}
        obj._apply_staged(self._p(), tag)
        assert obj._last[0] is tag
        assert calls == []
        assert obj.logger.messages == []

    def test_caches_without_apply_when_no_lanes_attr(self, monkeypatch):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = object()
        calls = []
        monkeypatch.setattr(obj, "apply_to_lane",
                            lambda ln, t: calls.append(1))
        tag = {"uid": "AA", "filament": {"type": "PLA"}}
        obj._apply_staged(self._p(), tag)
        assert obj._last[0] is tag
        assert calls == []

    def test_skips_apply_when_lane_not_found(self, monkeypatch):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        obj.afc = _AFCStub(lanes={})
        calls = []
        monkeypatch.setattr(obj, "apply_to_lane",
                            lambda ln, t: calls.append(1))
        tag = {"uid": "AA", "filament": {"type": "PLA"}}
        obj._apply_staged(self._p(), tag)
        assert obj._last[0] is tag
        assert calls == []

    def test_applies_when_lane_present(self, monkeypatch):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        lane = _Lane("lane0")
        obj.afc = _AFCStub(lanes={"lane0": lane})
        calls = []
        monkeypatch.setattr(obj, "apply_to_lane",
                            lambda ln, t: calls.append((ln, t)))
        tag = {"uid": "AA", "filament": {"type": "PLA"}}
        obj._apply_staged(self._p(), tag)
        assert obj._last[0] is tag
        assert calls == [(lane, tag)]

    def test_warns_when_apply_raises(self, monkeypatch):
        obj, printer = _make({"lane_slot_map": "lane0:0"})
        lane = _Lane("lane0")
        obj.afc = _AFCStub(lanes={"lane0": lane})

        def _apply(ln, t):
            raise RuntimeError("boom")

        monkeypatch.setattr(obj, "apply_to_lane", _apply)
        tag = {"uid": "AA", "filament": {"type": "PLA"}}
        obj._apply_staged(self._p(), tag)
        assert obj._last[0] is tag
        assert obj.logger.messages == [
            ("warning", "ViViD RFID: applying lane0 read failed: boom")]


# ── _retract_sibling ──────────────────────────────────────────────────────────

class TestVividRetractSibling:
    def _obj(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1",
                              "auto_tag_adjust_dist": 75.0}, readers=[r0])
        _ready(obj, printer)
        return obj

    def test_noop_when_disabled(self, monkeypatch):
        _patch_enums(monkeypatch)
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0, lane1:1",
                              "auto_tag_adjust": False}, readers=[r0])
        _ready(obj, printer)
        sib = _MoveLane("lane1")
        obj.afc = _AFCStub(lanes={"lane1": sib})
        p = {"slot": 0}
        obj._retract_sibling(p)
        assert sib.moves == []
        assert p.get("sib_lane") is None

    def test_return_when_no_sibling(self, monkeypatch):
        _patch_enums(monkeypatch)
        r0 = _FakeReader("reader0", [5])
        obj, printer = _make({"lane_slot_map": "lane0:5"}, readers=[r0])
        _ready(obj, printer)
        p = {"slot": 5}
        obj._retract_sibling(p)
        assert p.get("sib_lane") is None

    def test_return_when_afc_none(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj = self._obj()
        obj.afc = None
        p = {"slot": 0}
        obj._retract_sibling(p)
        assert p.get("sib_lane") is None

    def test_return_when_afc_no_lanes(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj = self._obj()
        obj.afc = object()
        p = {"slot": 0}
        obj._retract_sibling(p)
        assert p.get("sib_lane") is None

    def test_return_when_sibling_empty(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj = self._obj()
        sib = _MoveLane("lane1", prep_state=False)
        obj.afc = _AFCStub(lanes={"lane1": sib})
        p = {"slot": 0}
        obj._retract_sibling(p)
        assert sib.moves == []
        assert p.get("sib_lane") is None

    def test_skips_loaded_sibling_and_logs(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj = self._obj()
        sib = _MoveLane("lane1", prep_state=True, tool_loaded=True)
        obj.afc = _AFCStub(lanes={"lane1": sib})
        monkeypatch.setattr(obj, "_afc_is_printing", lambda: False)
        p = {"slot": 0}
        obj._retract_sibling(p)
        assert sib.moves == []
        assert obj.logger.messages == [
            ("info", "ViViD RFID: sibling lane1 is loaded/printing — not "
                     "moving it; relying on the HALT dedup (nudge by hand "
                     "if the read misses)")]

    def test_skips_printing_sibling(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj = self._obj()
        sib = _MoveLane("lane1", prep_state=True, tool_loaded=False)
        obj.afc = _AFCStub(lanes={"lane1": sib})
        monkeypatch.setattr(obj, "_afc_is_printing", lambda: True)
        p = {"slot": 0}
        obj._retract_sibling(p)
        assert sib.moves == []

    def test_retracts_idle_sibling_and_records(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj = self._obj()
        sib = _MoveLane("lane1", prep_state=True, tool_loaded=False)
        obj.afc = _AFCStub(lanes={"lane1": sib})
        monkeypatch.setattr(obj, "_afc_is_printing", lambda: False)
        p = {"slot": 0}
        obj._retract_sibling(p)
        assert sib.moves == [(75.0 * _MoveDir.NEG, _Speed.SHORT,
                              _Assist.NO, False)]
        assert p["sib_lane"] is sib
        assert p["sib_dist"] == 75.0
        assert obj.logger.messages == [
            ("info", "ViViD RFID: retracting sibling lane1 75mm to clear "
                     "the antenna before reading slot 0")]


# ── _restore_sibling ──────────────────────────────────────────────────────────

class TestVividRestoreSibling:
    def test_noop_when_probe_none(self):
        obj, printer = _make()
        obj._restore_sibling(None)
        assert obj.logger.messages == []

    def test_noop_when_no_sib_lane(self):
        obj, printer = _make()
        p = {"sib_lane": None, "sib_dist": 0.0}
        obj._restore_sibling(p)
        assert obj.logger.messages == []

    def test_noop_when_dist_zero(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj, printer = _make()
        sib = _MoveLane("lane1")
        p = {"sib_lane": sib, "sib_dist": 0.0}
        obj._restore_sibling(p)
        assert sib.moves == []

    def test_restores_and_resets(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj, printer = _make()
        sib = _MoveLane("lane1")
        p = {"sib_lane": sib, "sib_dist": 75.0}
        obj._restore_sibling(p)
        assert sib.moves == [(75.0 * _MoveDir.POS, _Speed.SHORT,
                              _Assist.NO, False)]
        assert p["sib_lane"] is None and p["sib_dist"] == 0.0
        assert obj.logger.messages == []

    def test_error_logged_and_state_reset(self, monkeypatch):
        _patch_enums(monkeypatch)
        obj, printer = _make()
        sib = _MoveLane("lane1")
        sib.move_to = _boom
        p = {"sib_lane": sib, "sib_dist": 75.0}
        obj._restore_sibling(p)
        assert p["sib_lane"] is None and p["sib_dist"] == 0.0
        assert obj.logger.messages == [
            ("error", "ViViD RFID: FAILED to restore sibling by 75mm: boom")]


# ── _afc_is_printing ──────────────────────────────────────────────────────────

class _PrintStats:
    def __init__(self, state=None, raise_exc=False):
        self._state = state
        self._raise = raise_exc

    def get_status(self, eventtime):
        if self._raise:
            raise RuntimeError("boom")
        return {"state": self._state}


class TestVividAfcIsPrinting:
    def test_false_when_no_print_stats(self):
        obj, printer = _make()
        assert obj._afc_is_printing() is False

    def test_true_when_printing(self):
        obj, printer = _make()
        printer._objects["print_stats"] = _PrintStats(state="printing")
        assert obj._afc_is_printing() is True

    def test_false_when_not_printing(self):
        obj, printer = _make()
        printer._objects["print_stats"] = _PrintStats(state="paused")
        assert obj._afc_is_printing() is False

    def test_false_on_exception(self):
        obj, printer = _make()
        printer._objects["print_stats"] = _PrintStats(raise_exc=True)
        assert obj._afc_is_printing() is False


# ── _cancel_poll ──────────────────────────────────────────────────────────────

class TestVividCancelPoll:
    def test_no_timer_just_clears_probe(self):
        obj, printer = _make()
        obj._probe = {"slot": 0}
        obj._poll_timer = None
        obj._cancel_poll()
        assert obj._probe is None
        assert obj._poll_timer is None

    def test_unregister_exception_swallowed(self, monkeypatch):
        obj, printer = _make()
        obj._probe = {"slot": 0}
        obj._poll_timer = object()
        monkeypatch.setattr(printer.reactor, "unregister_timer", _boom)
        obj._cancel_poll()
        assert obj._poll_timer is None
        assert obj._probe is None


# ── cmd_VIVID_RFID_READ ───────────────────────────────────────────────────────

class TestVividCmdRead:
    def _obj(self):
        r0 = _FakeReader("reader0", [0, 1])
        obj, printer = _make({"lane_slot_map": "lane0:0"}, readers=[r0])
        _ready(obj, printer)
        return obj

    def test_requires_lane_or_slot(self):
        obj = self._obj()
        with pytest.raises(_CmdError):
            obj.cmd_VIVID_RFID_READ(_GCmd({}))

    def test_lane_success_no_message(self, monkeypatch):
        obj = self._obj()
        monkeypatch.setattr(obj, "read_lane", lambda ln: {"material": "PLA"})
        g = _GCmd({"LANE": "lane0"})
        obj.cmd_VIVID_RFID_READ(g)
        assert g.responses == []

    def test_lane_no_decode_plain_hint(self, monkeypatch):
        obj = self._obj()
        monkeypatch.setattr(obj, "read_lane", lambda ln: None)
        g = _GCmd({"LANE": "lane0"})
        obj.cmd_VIVID_RFID_READ(g)
        assert g.responses == ["ViViD RFID: no tag decoded on lane0"]

    def test_lane_no_decode_with_seen_uid_hint(self, monkeypatch):
        obj = self._obj()
        monkeypatch.setattr(obj, "read_lane", lambda ln: None)
        obj.record_tag_read("lane0", None, decoded=False, uid="AABB",
                            tag_type="MifareClassic1k")
        g = _GCmd({"LANE": "lane0"})
        obj.cmd_VIVID_RFID_READ(g)
        assert g.responses == [
            "ViViD RFID: no tag decoded on lane0 (saw tag UID AABB, "
            "MifareClassic1k — no decoder/key matched)"]

    def test_slot_success_calls_respond_tag(self, monkeypatch):
        obj = self._obj()
        cap = []
        monkeypatch.setattr(obj, "read_slot",
                            lambda s: {"uid": "AA",
                                       "filament": {"type": "PLA"}})
        monkeypatch.setattr(obj, "_map", lambda t: {"si": True})
        monkeypatch.setattr(obj, "_respond_tag",
                            lambda g, si, where: cap.append((g, si, where)))
        g = _GCmd({"SLOT": "3"})
        obj.cmd_VIVID_RFID_READ(g)
        assert cap == [(g, {"si": True}, "slot 3")]

    def test_slot_no_tag_plain(self, monkeypatch):
        obj = self._obj()
        monkeypatch.setattr(obj, "read_slot", lambda s: None)
        g = _GCmd({"SLOT": "3"})
        obj.cmd_VIVID_RFID_READ(g)
        assert g.responses == ["ViViD RFID: no tag decoded on slot 3"]

    def test_slot_tag_without_uid(self, monkeypatch):
        obj = self._obj()
        monkeypatch.setattr(obj, "read_slot",
                            lambda s: {"uid": "", "filament": None})
        g = _GCmd({"SLOT": "3"})
        obj.cmd_VIVID_RFID_READ(g)
        assert g.responses == ["ViViD RFID: no tag decoded on slot 3"]

    def test_slot_tag_uid_no_type_hint(self, monkeypatch):
        obj = self._obj()
        monkeypatch.setattr(obj, "read_slot",
                            lambda s: {"uid": "AABB", "filament": None})
        g = _GCmd({"SLOT": "3"})
        obj.cmd_VIVID_RFID_READ(g)
        assert g.responses == [
            "ViViD RFID: no tag decoded on slot 3 (saw tag UID AABB — "
            "no decoder/key matched)"]

    def test_slot_tag_uid_with_type_hint(self, monkeypatch):
        obj = self._obj()
        monkeypatch.setattr(obj, "read_slot",
                            lambda s: {"uid": "AABB", "tag_type": "MC1k",
                                       "filament": None})
        g = _GCmd({"SLOT": "3"})
        obj.cmd_VIVID_RFID_READ(g)
        assert g.responses == [
            "ViViD RFID: no tag decoded on slot 3 (saw tag UID AABB, MC1k "
            "— no decoder/key matched)"]


# ── _respond_tag ──────────────────────────────────────────────────────────────

class TestVividRespondTag:
    def test_echoes_formatted_summary(self):
        obj, printer = _make()
        tag = {"uid": "d13fdb0e", "tag_type": "MifareClassic1k",
               "filament": {"manufacturer": "BQ Tech", "type": "PET",
                            "color_argb": 0xFFC0FFEE, "diameter_mm": 1.75,
                            "bed_temp_c": 60, "hotend_min_c": 200,
                            "hotend_max_c": 240}}
        si = obj._map(tag)
        g = _GCmd({})
        obj._respond_tag(g, si, "slot 3")
        expected = vivid.format_tag_summary(si, "ViViD RFID tag on slot 3:")
        assert g.responses == [expected]


# ── load_config / load_config_prefix ──────────────────────────────────────────

class TestVividLoadConfig:
    def test_load_config_builds_coordinator(self):
        cfg = _Config(_Printer(), {"lane_slot_map": "lane0:0"})
        obj = vivid.load_config(cfg)
        assert isinstance(obj, vivid.AFC_Vivid_rfid)
        assert obj._get_slot("lane0") == 0

    def test_load_config_prefix_builds_reader(self, monkeypatch):
        fake_bus = types.ModuleType("bus")
        fake_bus.MCU_SPI_from_config = lambda *a, **k: object()
        monkeypatch.setitem(sys.modules, "bus", fake_bus)
        cfg = _Config(_Printer(), {"slots": "0, 1"},
                      name="AFC_Vivid_rfid reader0")
        reader = vivid.load_config_prefix(cfg)
        assert isinstance(reader, vivid.AFC_Vivid_rfid_reader)
        assert reader.slots == [0, 1]
