"""
Tests for the ACE 2 Pro RFID reader, extras/AFC_ACE2_rfid.py.

The V2 serial frame and its CRC, the register link the shared reader library
sits on, the unit class, and the reader itself driven end to end.
Consolidated from three files; section banners name the file each came from.
"""

from __future__ import annotations
import importlib.util
import logging
import os
import types
import pytest
import extras.AFC_rfid_readers as readers  # noqa: E402
import struct
import hashlib


# ── Unit tests for the ACE2 RFID host wiring (extras/AFC_ACE2_rfid.py): ───────
#
# was tests/test_AFC_ACE2_rfid.py
HERE_rfid = os.path.dirname(os.path.abspath(__file__))
ROOT_rfid = os.path.dirname(HERE_rfid)


def _load_rfid(name, relpath):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT_rfid, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod



rfid_rfid = _load_rfid("AFC_ACE2_rfid", "extras/AFC_ACE2_rfid.py")
ace2 = _load_rfid("AFC_ACE2", "extras/AFC_ACE2.py")


class _Conn_rfid:
    connected = True

    def __init__(self):
        self.calls = []

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        # set_rfid_enable is fire-and-forget; record in wire order alongside sync.
        self.calls.append((name, dict(params or {})))


class _Ace2_rfid:
    def __init__(self):
        self._ace = _Conn_rfid()


class _Reactor_rfid:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        return waketime

    def register_callback(self, cb, waketime=None):
        cb(0.0)                                  # run inline for tests


class _AdvReactor_rfid:
    """Reactor whose monotonic advances on each pause, so a scan's deadline
    actually fires (the plain _Reactor_rfid freezes monotonic at 0)."""

    def __init__(self, step=0.5):
        self._t = 0.0
        self._step = step

    def monotonic(self):
        return self._t

    def pause(self, when):
        self._t += self._step
        return when

    def register_callback(self, cb, when=None):
        pass


class _CmdError_rfid(Exception):
    pass


class _Gcode_rfid:
    def __init__(self):
        self.commands = {}
        self.raw = []
        self.info = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def respond_info(self, msg, log=True):
        self.info.append(msg)

    def respond_raw(self, msg):
        self.raw.append(msg)


class _Printer_rfid:
    command_error = _CmdError_rfid

    def __init__(self):
        self.reactor = _Reactor_rfid()
        self.gcode = _Gcode_rfid()
        self.events = []

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        return default

    def register_event_handler(self, name, cb):
        self.events.append((name, cb))


class _Config_rfid:
    """Minimal Klipper config stub: real __init__ reads these via get*()."""

    def __init__(self, printer, opts):
        self._printer = printer
        self._opts = opts

    def get_printer(self):
        return self._printer

    def get(self, key, default=None):
        return self._opts.get(key, default)

    def getboolean(self, key, default=None):
        return bool(self._opts.get(key, default))

    def getint(self, key, default=None, minval=None):
        return int(self._opts.get(key, default))

    def getfloat(self, key, default=None, minval=None):
        return float(self._opts.get(key, default))


class _GCmd_rfid:
    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=0):
        return int(self.params.get(key, default))

    def get_float(self, key, default=0.0, minval=None, maxval=None):
        return float(self.params.get(key, default))

    def respond_info(self, msg):
        self.responses.append(msg)


def _make_rfid(ace2_obj, opts=None):
    """Build AFC_ACE2_RFID through its real __init__ with a stub config, then
    bind the ACE2 object directly (klippy:ready isn't fired in tests)."""
    obj = rfid_rfid.AFC_ACE2_RFID(_Config_rfid(_Printer_rfid(), dict(opts or {})))
    obj.logger = logging.getLogger("test.ace2_rfid")
    obj.ace2 = ace2_obj
    return obj


def _bare_rfid(ace2_obj, lane_slot=None, read_on_insert=True, attempts=1, delay=0.0):
    """AFC_ACE2_RFID via the real constructor, wired for the legacy test API
    (stage_read off, instant settles)."""
    pairs = ", ".join(f"{n}:{s}" for n, s in (lane_slot or {}).items())
    return _make_rfid(ace2_obj, {
        "lane_slot_map": pairs,
        "read_on_insert": read_on_insert,
        "read_on_insert_attempts": attempts,
        "read_on_insert_delay": delay,
        "stage_read": False,
        "probe_settle": 0.0,
        "skip_factory_autostage": False,   # legacy tests expect identify restore
    })


_UNSET = object()


def _patch_read_tag(monkeypatch, recorded, result=_UNSET, exc=None):
    """Patch the module-level read_tag (read_slot calls it directly).

    result=_UNSET -> return a default tag; result=None -> return None (no tag);
    otherwise return the given value.
    """
    def fake(link, bambu_master_key=None, is_excluded=None, **kw):
        recorded.append(("read_tag", link.slot))
        if exc is not None:
            raise exc
        if result is _UNSET:
            # A fully-decoded tag by default (staging/scan only accept a decode).
            return {"uid": "aa", "tag_type": "MifareClassic1k",
                    "filament": {"type": "PLA"}}
        return result
    monkeypatch.setattr(rfid_rfid, "read_tag", fake)


# ── AFC_ACE2 command builder ────────────────────────────────────────────────

def test_builder_maps_reader_power_to_0x52():
    cmd, payload = ace2.method_to_v2("mfrc522_reader_power", {"arg": 0x10001})
    assert cmd == 0x52 == ace2.Cmd.MFRC522_READER_POWER
    assert payload == ace2.pb_uint32(1, 0x10001)


# ── _Ace2RegLink power/enable encodings ─────────────────────────────────────

def test_reader_power_arg_encoding():
    a = _Ace2_rfid()
    link = rfid_rfid._Ace2RegLink(a, 1)          # reader index 1
    link.reader_power(True)
    link.reader_power(False)
    assert a._ace.calls == [
        ("mfrc522_reader_power", {"arg": (1 << 16) | 1}),
        ("mfrc522_reader_power", {"arg": (1 << 16) | 0}),
    ]


def test_set_rfid_enable_uses_physical_slot():
    a = _Ace2_rfid()
    link = rfid_rfid._Ace2RegLink(a, 0)
    link.set_rfid_enable(2, False)
    assert a._ace.calls == [("set_rfid_enable", {"index": 2, "enable": False})]


# ── read_slot: mapping + power sequence ─────────────────────────────────────

def test_read_slot_power_sequence_order(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, recorded)
    tag = obj.read_slot(0)
    assert tag is not None
    names = [c[0] for c in a._ace.calls]
    # disable BOTH shared slots -> power on -> (read) -> power off -> re-enable both
    assert names == [
        "set_rfid_enable", "set_rfid_enable",   # (0,False),(1,False)
        "mfrc522_reader_power",                  # reader0 on
        "mfrc522_reader_power",                  # reader0 off
        "set_rfid_enable", "set_rfid_enable",   # (0,True),(1,True)
    ]
    assert a._ace.calls[0][1] == {"index": 0, "enable": False}
    assert a._ace.calls[1][1] == {"index": 1, "enable": False}
    assert a._ace.calls[2][1] == {"arg": (0 << 16) | 1}
    assert a._ace.calls[3][1] == {"arg": (0 << 16) | 0}
    assert a._ace.calls[4][1] == {"index": 0, "enable": True}
    assert a._ace.calls[5][1] == {"index": 1, "enable": True}
    assert ("read_tag", 0) in recorded


@pytest.mark.parametrize("phys_slot,reader_idx", [(0, 0), (1, 0), (2, 1), (3, 1)])
def test_read_slot_reader_index_mapping(monkeypatch, phys_slot, reader_idx):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, recorded)
    obj.read_slot(phys_slot)
    sib = reader_idx * 2
    # 2 readers cover the 4 slots: reg r/w AND power both use the per-pair index
    assert a._ace.calls[2][1] == {"arg": (reader_idx << 16) | 1}
    assert ("read_tag", reader_idx) in recorded
    # identify disabled on BOTH physical slots the reader pair serves
    disabled = {a._ace.calls[0][1]["index"], a._ace.calls[1][1]["index"]}
    assert disabled == {sib, sib + 1}
    assert phys_slot in disabled


def test_read_slot_per_slot_reg_index_optin(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a)
    obj.reader_reg_per_slot = True           # opt in to per-slot chip-select
    _patch_read_tag(monkeypatch, recorded)
    obj.read_slot(3)
    assert ("read_tag", 3) in recorded        # reg index == physical slot
    assert a._ace.calls[2][1] == {"arg": (1 << 16) | 1}   # power still per-pair


def test_read_slot_reg_override_sets_chip_and_power(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, recorded)
    obj.read_slot(2, reg_slot=1)              # probe reader index 1 explicitly
    assert ("read_tag", 1) in recorded        # reg index forced to 1
    assert a._ace.calls[2][1] == {"arg": (1 << 16) | 1}   # power = same reader (1)


def test_read_slot_disables_identify_but_skips_restore_when_autostage_skipped(monkeypatch):
    a = _Ace2_rfid()
    obj = _make_rfid(a, {"skip_factory_autostage": True})
    assert obj.probe_restore_identify is False   # keep identify off after the read
    _patch_read_tag(monkeypatch, [])
    obj.read_slot(2)
    enables = [c for c in a._ace.calls if c[0] == "set_rfid_enable"]
    # identify DISABLED on the pair (suppresses the firmware autostage) and NOT
    # re-enabled, so the next insert doesn't autostage either
    assert enables == [("set_rfid_enable", {"index": 2, "enable": False}),
                       ("set_rfid_enable", {"index": 3, "enable": False})]
    assert any(c[0] == "mfrc522_reader_power" for c in a._ace.calls)


def test_read_slot_restores_power_on_read_failure(monkeypatch):
    a = _Ace2_rfid()
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, [], exc=RuntimeError("read blew up"))
    with pytest.raises(RuntimeError):
        obj.read_slot(0)
    # power still turned off and identify restored on BOTH slots despite the error
    names = [c[0] for c in a._ace.calls]
    assert names[-3:] == ["mfrc522_reader_power", "set_rfid_enable", "set_rfid_enable"]
    assert a._ace.calls[-3][1] == {"arg": 0}
    assert {a._ace.calls[-2][1]["index"], a._ace.calls[-1][1]["index"]} == {0, 1}
    assert a._ace.calls[-2][1]["enable"] is True


def test_read_slot_manage_power_false_skips_sequence(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, recorded)
    obj.read_slot(0, manage_power=False)
    assert a._ace.calls == []
    assert ("read_tag", 0) in recorded


# ── cmd_ACE_RFID_READ never crashes Klipper ─────────────────────────────────

def test_cmd_read_error_does_not_propagate(monkeypatch):
    a = _Ace2_rfid()
    obj = _bare_rfid(a)
    # A generic (non-command_error) exception from the read must be swallowed.
    _patch_read_tag(monkeypatch, [], exc=RuntimeError("serial timed out"))
    gcmd = _GCmd_rfid({"SLOT": 0})
    obj.cmd_ACE_RFID_READ(gcmd)                  # must NOT raise
    assert any("read error" in r for r in gcmd.responses)


def test_cmd_read_reports_no_tag(monkeypatch):
    a = _Ace2_rfid()
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, [], result=None)
    gcmd = _GCmd_rfid({"SLOT": 0})
    obj.cmd_ACE_RFID_READ(gcmd)
    assert any("no tag found" in r for r in gcmd.responses)


# ── scanner lane (ACE_RFID_SCAN -> next spool id) ────────────────────────────

def test_scanner_lanes_config_parsed():
    obj = _make_rfid(_Ace2_rfid(), {"scanner_lanes": "lane1, lane2",
                               "scan_seconds": 45, "scan_interval": 0.5})
    assert obj._scanner_lanes == {"lane1", "lane2"}
    assert obj.scan_seconds == 45.0
    assert obj.scan_interval == 0.5


def test_scan_seconds_defaults_to_30():
    obj = _make_rfid(_Ace2_rfid(), {})
    assert obj.scan_seconds == 30.0


def test_scan_lane_stages_next_spool_id(monkeypatch):
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    tag = {"uid": "abcd", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA"}}
    monkeypatch.setattr(obj, "_scan_slot", lambda slot, dur, lane_name="": tag)
    calls = []

    def _sync(afc, lane, slot_info, logger, prefix, **kw):
        calls.append((prefix, kw))

    monkeypatch.setattr(rfid_rfid, "sync_rfid_to_spoolman", _sync)
    lane = types.SimpleNamespace(name="scan_lane")
    obj.afc = types.SimpleNamespace(lanes={"scan_lane": lane}, spoolman=object())

    out = obj.scan_lane("scan_lane", 5)

    assert out is tag
    assert len(calls) == 1
    prefix, kw = calls[0]
    assert kw.get("set_next") is True            # staged, not applied to lane
    assert prefix == "ACE2 RFID scan"


def test_scan_emits_popup_notification(monkeypatch):
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    tag = {"uid": "abcd", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA"}}
    monkeypatch.setattr(obj, "_scan_slot", lambda slot, dur, lane_name="": tag)
    monkeypatch.setattr(rfid_rfid, "sync_rfid_to_spoolman", lambda *a, **k: None)
    lane = types.SimpleNamespace(name="scan_lane")
    obj.afc = types.SimpleNamespace(
        lanes={"scan_lane": lane}, spoolman=object(),
        spool=types.SimpleNamespace(next_spool_id=120))

    obj.scan_lane("scan_lane", 5)

    raw = obj.gcode.raw
    assert any("action:prompt_begin" in r for r in raw)   # popup opened
    assert any("action:prompt_show" in r for r in raw)    # and shown
    assert any("Spoolman ID: 120" in r for r in raw)      # with the staged id
    assert any("Material: PLA" in r for r in raw)


def test_scan_lane_no_tag_no_sync(monkeypatch):
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    monkeypatch.setattr(obj, "_scan_slot", lambda slot, dur, lane_name="": None)
    called = []
    monkeypatch.setattr(rfid_rfid, "sync_rfid_to_spoolman",
                        lambda *a, **k: called.append(1))
    obj.afc = types.SimpleNamespace(lanes={}, spoolman=object())

    assert obj.scan_lane("scan_lane", 5) is None
    assert called == []                           # nothing staged


def test_scan_slot_reads_until_tag_then_powers_off(monkeypatch):
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 1})
    # two misses (None), then a FULL decode on the 3rd attempt
    seq = [None, None, {"uid": "beef", "filament": {"type": "PLA"}}]
    monkeypatch.setattr(rfid_rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=5.0)

    assert tag == {"uid": "beef", "filament": {"type": "PLA"}}
    powers = [c[1]["arg"] for c in a._ace.calls if c[0] == "mfrc522_reader_power"]
    assert 1 in powers                            # reader powered on to read
    assert powers[-1] == 0                         # and powered off when done


def test_scan_prefers_full_decode_over_uid_only(monkeypatch):
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 1})
    # a UID-only read first, then the full decode: must wait for the full one
    seq = [{"uid": "beef", "filament": None},
           {"uid": "beef", "filament": {"type": "PLA", "manufacturer": "Snapmaker"}}]
    monkeypatch.setattr(rfid_rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=5.0)

    assert tag["filament"]["manufacturer"] == "Snapmaker"   # not the UID-only read


def test_scan_confirms_same_uid_over_consecutive_reads(monkeypatch):
    # A stray one-off decode must NOT be accepted: with confirm=2 the scan only
    # returns once the SAME UID decodes twice in a row; a different UID in
    # between resets the streak.
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 2})
    good = {"uid": "beef", "filament": {"type": "PLA"}}
    stray = {"uid": "cafe", "filament": {"type": "ABS"}}
    seq = [stray, good, stray, good, good]        # good confirmed on the last two
    monkeypatch.setattr(rfid_rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=10.0)

    assert tag is good                            # the twice-in-a-row UID wins
    assert obj._slot_uid[0] == "beef"


def test_scan_returns_none_without_full_decode(monkeypatch):
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0})
    obj.reactor = _AdvReactor_rfid(step=0.5)           # advance so the deadline fires
    # only ever a UID (decode never completes) -> a UID-only read (possibly a
    # corrupted UID) is NOT trusted; the scan reports no read.
    monkeypatch.setattr(rfid_rfid, "read_tag",
                        lambda link, **kw: {"uid": "beef", "filament": None})

    tag = obj._scan_slot(0, duration=2.0)

    assert tag is None                            # no full decode -> no match
    assert 0 not in obj._slot_uid                  # and the UID was not cached


def test_scan_reads_only_the_presented_tag(monkeypatch):
    # No prime: the scan reads whatever is presented (a full decode) and does not
    # pre-read/cache the neighbour slot.
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}                  # slot 0; sibling is slot 1
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 1})
    seq = [{"uid": "new", "filament": {"type": "PLA"}}]
    monkeypatch.setattr(rfid_rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=5.0)

    assert tag == {"uid": "new", "filament": {"type": "PLA"}}
    assert 1 not in obj._slot_uid                  # neighbour slot never pre-read


def test_scan_lane_missing_slot_raises():
    a = _Ace2_rfid()                                    # no slot map for the lane
    obj = _make_rfid(a, {"scanner_lanes": "nolane"})
    with pytest.raises(_CmdError_rfid):
        obj.scan_lane("nolane", 5)


def test_cmd_scan_defaults_to_sole_scanner_lane(monkeypatch):
    a = _Ace2_rfid()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    obj.afc = types.SimpleNamespace(spool=types.SimpleNamespace(next_spool_id=7))
    seen = {}

    def _scan(name, secs):
        seen["name"], seen["secs"] = name, secs
        return {"uid": "abcd", "filament": {"type": "PLA"}}

    monkeypatch.setattr(obj, "scan_lane", _scan)
    gcmd = _GCmd_rfid({})                               # no LANE -> sole scanner lane
    obj.cmd_ACE_RFID_SCAN(gcmd)

    assert seen["name"] == "scan_lane"
    assert seen["secs"] == 30.0                     # default scan_seconds
    assert any("staged next spool" in r for r in gcmd.responses)


def test_cmd_scan_requires_lane_when_multiple_scanners():
    obj = _make_rfid(_Ace2_rfid(), {"scanner_lanes": "a, b"})
    with pytest.raises(_CmdError_rfid):
        obj.cmd_ACE_RFID_SCAN(_GCmd_rfid({}))


# ── auto-read on insert ─────────────────────────────────────────────────────

def test_auto_read_on_insert_reads_mapped_lane(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    # the read sequence ran for the mapped lane
    assert ("read_tag", 0) in recorded
    assert any(c[0] == "mfrc522_reader_power" for c in a._ace.calls)


def test_auto_read_skips_unmapped_lane(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="laneX"))
    assert recorded == [] and a._ace.calls == []


def test_auto_read_disabled_by_config(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0}, read_on_insert=False)
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    assert recorded == [] and a._ace.calls == []


# ── continuous stage scan (stop-on-detect + parked read) ────────────

class _ScanConn(_Conn_rfid):
    """_Conn_rfid plus the motion + status calls the continuous scan issues."""

    def feed_filament(self, index, length, speed):
        self.calls.append(
            ("feed_filament", {"index": index, "length": length, "speed": speed}))

    def stop_feed_filament(self, index):
        self.calls.append(("stop_feed_filament", {"index": index}))

    def unwind_filament(self, index, length, speed, mode="normal"):
        self.calls.append(
            ("unwind_filament", {"index": index, "length": length, "speed": speed}))

    def get_status(self, timeout=2.0):
        return {"status": "ready", "slots": []}

    def stop_feed_assist_sync(self, index, timeout=2.0):
        self.calls.append(("stop_feed_assist_sync", {"index": index}))


class _ScanAce2(_Ace2_rfid):
    """ACE2 stub for the stage scan: records motion and answers the ready/moving/
    empty helpers the scan polls. _slot_is_moving reports a couple of 'moving'
    polls then 'stopped' so a no-tag scan ends promptly."""

    def __init__(self, moving_polls=1):
        self._ace = _ScanConn()
        self.feed_speed = 80.0
        self.retract_speed = 80.0
        self._moving_polls = moving_polls
        self.waits = []

    def _wait_for_ace_ready(self, timeout=30.0):
        return True

    def _slot_reports_empty(self, slot):
        return False

    def _slot_is_moving(self, status, slot):
        self._moving_polls -= 1
        return self._moving_polls >= 0

    def _wait_for_feed_complete(self, slot, length, speed, **kw):
        self.waits.append((slot, length, speed))
        return True


def _patch_activate(monkeypatch, uids):
    """Script MifareClassic(...).activate() to yield the given UIDs (hex str or
    None) one per poll; None / exhausted means 'no tag this poll'."""
    seq = list(uids)

    class _MC:
        def __init__(self, mfrc):
            pass

        def activate(self):
            u = seq.pop(0) if seq else None
            if u is None:
                return None, None
            return bytes.fromhex(u), 0x08

    monkeypatch.setattr(rfid_rfid, "MifareClassic", _MC)


def _scan_obj(monkeypatch, ace2=None, opts=None, lane_slot=None):
    """Staging obj wired for the continuous scan: real constructor, an advancing
    reactor (so the poll deadline fires) and a scan-capable ACE2 stub."""
    a = ace2 or _ScanAce2()
    ls = lane_slot or {"lane1": 0}
    a._slot_map = dict(ls)
    pairs = ", ".join(f"{n}:{s}" for n, s in ls.items())
    base = {"lane_slot_map": pairs, "stage_read": True, "probe_settle": 0.0,
            "skip_factory_autostage": True}
    base.update(opts or {})
    obj = _make_rfid(a, base)
    obj.logger = logging.getLogger("test.ace2_rfid")
    obj.reactor = _AdvReactor_rfid(step=0.5)
    return obj, a


def _decoded(uid="beef"):
    return {"uid": uid, "tag_type": "MifareClassic1k", "filament": {"type": "PLA"}}


def test_stage_scan_reads_and_stops_on_detect(monkeypatch):
    obj, a = _scan_obj(monkeypatch)
    _patch_activate(monkeypatch, ["beef"])           # tag detected on first poll
    _patch_read_tag(monkeypatch, [], result=_decoded("beef"))
    applied = []
    monkeypatch.setattr(obj, "apply_to_lane", lambda lane, tag: applied.append(tag))
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)

    assert ctx["active"] is True
    assert ctx["done"] is True
    assert obj._slot_uid[0] == "beef"
    assert applied and applied[0]["uid"] == "beef"
    names = [c[0] for c in a._ace.calls]
    assert "feed_filament" in names                  # scan fed
    assert "stop_feed_filament" in names             # stopped on detect
    assert "fed" in ctx                              # reported for the feeder


def test_stage_begin_disables_identify_with_barrier(monkeypatch):
    obj, a = _scan_obj(monkeypatch)
    monkeypatch.setattr(obj, "_run_stage_scan", lambda lane, ctx: 0.0)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)

    assert ctx["active"] is True
    assert ctx["initial"] == obj.stage_initial_dist  # skip_factory_autostage -> 500
    assert sum(1 for c in a._ace.calls if c[0] == "set_rfid_enable") == 2
    assert any(c[0] == "mfrc522_reader_power" and c[1] == {"arg": 0}
               for c in a._ace.calls)


def test_stage_begin_no_initial_when_factory_stages(monkeypatch):
    obj, a = _scan_obj(monkeypatch, opts={"skip_factory_autostage": False})
    monkeypatch.setattr(obj, "_run_stage_scan", lambda lane, ctx: 0.0)
    ctx = {"active": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)
    assert ctx["initial"] == 0.0                      # factory does the load


def test_disable_rfid_stage_begin_plain_feed_with_initial():
    # disable_rfid: no chunk-probe, no reader touch — but the mandatory initial
    # load is still requested so a skip_factory_autostage insert reaches the hub.
    a = _Ace2_rfid()
    a._slot_map = {"lane1": 0}
    obj = _make_rfid(a, {"disable_rfid": True, "skip_factory_autostage": True,
                         "stage_read": True, "lane_slot_map": "lane1:0"})
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)
    assert ctx.get("active") is False                 # no chunk-probe
    assert ctx["initial"] == obj.stage_initial_dist   # plain initial load fed
    assert "scan_dist" not in ctx                     # no scanning
    assert a._ace.calls == []                         # reader never touched


def test_disable_rfid_stage_begin_no_initial_when_factory_stages():
    a = _Ace2_rfid()
    a._slot_map = {"lane1": 0}
    obj = _make_rfid(a, {"disable_rfid": True, "skip_factory_autostage": False,
                         "stage_read": True, "lane_slot_map": "lane1:0"})
    ctx = {"active": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)
    assert ctx.get("active") is False
    assert ctx["initial"] == 0.0                      # factory does the load


def test_disable_rfid_post_insert_no_read():
    a = _Ace2_rfid()
    a._slot_map = {"lane1": 0}
    obj = _make_rfid(a, {"disable_rfid": True, "stage_read": False,
                         "read_on_insert": True, "lane_slot_map": "lane1:0"})
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    assert a._ace.calls == []                         # nothing read on insert


def test_stage_scan_no_tag_releases_reader(monkeypatch):
    obj, a = _scan_obj(monkeypatch)
    _patch_activate(monkeypatch, [])                 # never any tag
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)

    assert ctx["done"] is False
    names = [c[0] for c in a._ace.calls]
    assert "feed_filament" in names                  # scan still fed
    assert "stop_feed_filament" not in names         # nothing to stop on
    powers = [c[1]["arg"] for c in a._ace.calls if c[0] == "mfrc522_reader_power"]
    assert powers and powers[-1] == 0                # reader released
    assert ctx.get("fed", 0.0) > 0.0                 # fed the scan window


def test_stage_teardown_leaves_identify_off_when_configured(monkeypatch):
    obj, a = _scan_obj(monkeypatch)
    obj.probe_restore_identify = False               # skip_factory_autostage default
    link = rfid_rfid._Ace2RegLink(a, 0, power_index=0)
    obj._probe = {"link": link, "shared": (0, 1)}
    a._ace.calls.clear()
    obj._safe_probe_teardown()
    names = [c[0] for c in a._ace.calls]
    assert "mfrc522_reader_power" in names            # reader powered down
    assert "set_rfid_enable" not in names             # identify NOT restored


def test_stage_teardown_restores_identify_when_configured(monkeypatch):
    obj, a = _scan_obj(monkeypatch)
    obj.probe_restore_identify = True
    link = rfid_rfid._Ace2RegLink(a, 0, power_index=0)
    obj._probe = {"link": link, "shared": (0, 1)}
    a._ace.calls.clear()
    obj._safe_probe_teardown()
    enables = [c for c in a._ace.calls if c[0] == "set_rfid_enable"]
    assert {c[1]["index"] for c in enables} == {0, 1}
    assert all(c[1]["enable"] is True for c in enables)


def test_disable_factory_identify_all_slots():
    a = _Ace2_rfid()
    a.slot_count = 4
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj._disable_factory_identify()          # no slots arg -> all of them
    disabled = [c[1]["index"] for c in a._ace.calls if c[0] == "set_rfid_enable"]
    assert sorted(disabled) == [0, 1, 2, 3]
    assert all(c[1]["enable"] is False
               for c in a._ace.calls if c[0] == "set_rfid_enable")


def test_retry_disable_identify_when_connected():
    a = _Ace2_rfid()                                      # a._ace.connected is True
    obj = _make_rfid(a, {"skip_factory_autostage": True})
    obj._retry_disable_identify(0.0)
    disabled = [c[1]["index"] for c in a._ace.calls if c[0] == "set_rfid_enable"]
    assert sorted(disabled) == [0, 1, 2, 3]          # disabled once connected
    assert all(c[1]["enable"] is False
               for c in a._ace.calls if c[0] == "set_rfid_enable")


def test_retry_disable_identify_gives_up_when_never_ready(caplog):
    a = _Ace2_rfid()
    a._ace = None                                    # serial never connects
    obj = _make_rfid(a, {"skip_factory_autostage": True,
                         "identify_disable_max_tries": 3,
                         "identify_disable_retry": 0.05})
    with caplog.at_level(logging.WARNING):
        obj._retry_disable_identify(0.0)
    assert obj._identify_disable_tries == 3
    assert any("ACE serial not ready after 3 tries" in r.getMessage()
               for r in caplog.records)


def test_retry_disable_identify_handles_missing_ace2():
    a = _Ace2_rfid()
    obj = _make_rfid(a, {"skip_factory_autostage": True,
                         "identify_disable_max_tries": 1})
    obj.ace2 = None
    obj._retry_disable_identify(0.0)                 # must not raise; gives up
    assert obj._identify_disable_tries == 1


def test_stage_scan_suppresses_and_restores_feed_assist(monkeypatch):
    a = _ScanAce2()
    a._assist_suppressed = set()
    a._feed_assist_active = {0, 1}
    stopped = []
    a._stop_feed_assist = lambda s: (a._feed_assist_active.discard(s),
                                     stopped.append(s))
    obj, a = _scan_obj(monkeypatch, ace2=a)
    _patch_activate(monkeypatch, [])
    _patch_read_tag(monkeypatch, [])
    lane = types.SimpleNamespace(name="lane1")
    ctx = {"active": False, "done": False}

    obj._stage_probe_begin(lane, ctx)
    assert a._assist_suppressed == {0, 1}             # stopped + suppressed on both
    assert set(stopped) == {0, 1}

    obj._stage_probe_end(lane, ctx)                   # end of staging -> restore
    assert a._assist_suppressed == set()


def test_stage_scan_dedups_sibling_then_reads_own(monkeypatch):
    a = _ScanAce2()
    obj, a = _scan_obj(monkeypatch, ace2=a, lane_slot={"lane3": 2, "lane4": 3})
    lane4 = types.SimpleNamespace(name="lane4", prep_state=True)  # sibling present
    lane3 = types.SimpleNamespace(name="lane3", prep_state=True)
    obj.afc = types.SimpleNamespace(
        lanes={"lane3": lane3, "lane4": lane4},
        function=types.SimpleNamespace(is_printing=lambda: False))
    obj._slot_uid = {3: "cafe"}                       # sibling slot 3 read "cafe"
    _patch_activate(monkeypatch, ["cafe", "beef"])    # sibling first, then our tag
    _patch_read_tag(monkeypatch, [], result=_decoded("beef"))
    monkeypatch.setattr(obj, "apply_to_lane", lambda lane, tag: None)
    ctx = {"active": False, "done": False}

    obj._stage_probe_begin(lane3, ctx)
    assert ctx["done"] is True
    assert obj._slot_uid[2] == "beef"                 # our tag, not the sibling's


def test_stage_scan_aborts_on_removal(monkeypatch):
    a = _ScanAce2()
    a._slot_reports_empty = lambda slot: True         # spool pulled mid-scan
    obj, a = _scan_obj(monkeypatch, ace2=a)
    _patch_activate(monkeypatch, [])
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)
    assert ctx.get("removed") is True
    assert ctx["done"] is False


def test_stage_scan_recenters_when_parked_read_misses(monkeypatch):
    obj, a = _scan_obj(monkeypatch)
    _patch_activate(monkeypatch, ["beef"])
    reads = {"n": 0}

    def fake(link, **kw):
        reads["n"] += 1
        # the at-rest attempts miss; a read only succeeds after a re-center retract
        if reads["n"] <= obj.stage_read_hold_attempts:
            return None
        return _decoded("beef")

    monkeypatch.setattr(rfid_rfid, "read_tag", fake)
    monkeypatch.setattr(obj, "apply_to_lane", lambda lane, tag: None)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)

    assert ctx["done"] is True
    assert any(c[0] == "unwind_filament" for c in a._ace.calls)  # re-centered


def test_stage_scan_uid_only_not_accepted(monkeypatch):
    obj, a = _scan_obj(monkeypatch, opts={"stage_recenter_max": 6.0})
    _patch_activate(monkeypatch, ["aa"])
    _patch_read_tag(monkeypatch, [], result={
        "uid": "aa", "tag_type": "MifareClassic1k", "filament": None})
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)

    assert ctx["done"] is False                       # UID-only -> not accepted
    assert 0 not in obj._slot_uid


def _dedup_obj(sibling_present=True, session_uid=None):
    """Shared-reader obj: reader1 serves slots 2 (active) & 3 (sibling=lane3).
    Dedup is same-session only (we remember a UID read on the sibling slot); no
    offline cache."""
    a = _Ace2_rfid()
    a._slot_map = {"lane2": 2, "lane3": 3}
    obj = _bare_rfid(a, lane_slot={"lane2": 2, "lane3": 3})
    lane3 = types.SimpleNamespace(name="lane3", prep_state=sibling_present)
    obj.afc = types.SimpleNamespace(lanes={"lane3": lane3})
    if session_uid is not None:
        obj._slot_uid[3] = session_uid
    return obj


def test_is_sibling_tag_by_session_uid():
    # The exact UID was already read on the sibling slot this session.
    obj = _dedup_obj(session_uid="beef")
    assert obj._is_sibling_tag(2, 3, "beef") is True


def test_is_sibling_tag_false_without_a_session_read():
    # No UID recorded on the sibling this session -> nothing to collide with.
    obj = _dedup_obj()
    assert obj._is_sibling_tag(2, 3, "4cb2dea6") is False


def test_is_sibling_tag_false_when_sibling_absent():
    # Spool moved out of the sibling slot -> its old tag no longer collides.
    obj = _dedup_obj(sibling_present=False, session_uid="4cb2dea6")
    assert obj._is_sibling_tag(2, 3, "4cb2dea6") is False


def test_is_sibling_tag_false_when_dedup_disabled():
    obj = _dedup_obj(session_uid="4cb2dea6")
    obj.shared_reader_dedup = False
    assert obj._is_sibling_tag(2, 3, "4cb2dea6") is False


def test_is_sibling_tag_false_on_empty_uid():
    obj = _dedup_obj(session_uid="4cb2dea6")
    assert obj._is_sibling_tag(2, 3, "") is False
    assert obj._is_sibling_tag(2, 3, None) is False


def test_is_sibling_tag_false_when_no_sibling():
    obj = _dedup_obj(session_uid="4cb2dea6")
    assert obj._is_sibling_tag(2, None, "4cb2dea6") is False


# ── lane -> slot numbering (ACE map authoritative) ──────────────────────────

def test_rfid_enabled_uses_configured_map_as_allowlist():
    a = _Ace2_rfid()
    a._slot_map = {"lane0": 0, "lane1": 1}
    obj = _bare_rfid(a, lane_slot={"lane1": 1})     # explicit allow-list
    assert obj._rfid_enabled("lane1") is True
    assert obj._rfid_enabled("lane0") is False       # present on ACE, not allowed


def test_rfid_enabled_auto_from_ace_when_no_map():
    a = _Ace2_rfid()
    a._slot_map = {"lane0": 0, "lane1": 1}
    obj = _bare_rfid(a)                              # no lane_slot_map
    assert obj._rfid_enabled("lane0") is True
    assert obj._rfid_enabled("laneX") is False


def test_slot_for_lane_prefers_ace_map_over_stale_config():
    a = _Ace2_rfid()
    a._slot_map = {"lane2": 2}
    obj = _bare_rfid(a, lane_slot={"lane2": 1})      # wrong/stale config value
    assert obj._slot_for_lane("lane2") == 2          # ACE map wins


def test_stage_probe_skips_unconfigured_lane(monkeypatch):
    a = _Ace2_rfid()
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj.stage_read = True
    _patch_read_tag(monkeypatch, [])
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="laneX"), ctx)
    assert ctx["active"] is False and a._ace.calls == []


def test_stage_probe_disabled_leaves_ctx_inactive():
    a = _Ace2_rfid()
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj.stage_read = False
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)
    assert ctx["active"] is False and a._ace.calls == []


def test_post_insert_skipped_when_stage_read(monkeypatch):
    a = _Ace2_rfid()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj.stage_read = True                    # stage-probe handles it instead
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    assert recorded == [] and a._ace.calls == []


def test_auto_read_retries_until_tag(monkeypatch):
    a = _Ace2_rfid()
    obj = _bare_rfid(a, lane_slot={"lane1": 2}, attempts=3, delay=0.0)
    calls = {"n": 0}

    def fake(link, bambu_master_key=None, is_excluded=None, **kw):
        calls["n"] += 1
        return None if calls["n"] < 2 else {"uid": "bb", "tag_type": "x", "filament": None}
    monkeypatch.setattr(rfid_rfid, "read_tag", fake)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    assert calls["n"] == 2                        # stopped on first success


# ── sister-retract (auto_tag_adjust): clear a dominating sibling tag ─────────

class _MoveConn(_Conn_rfid):
    """_Conn_rfid plus the ACE motion commands the sister-retract issues."""

    def unwind_filament(self, index, length, speed, mode="normal"):
        self.calls.append(
            ("unwind_filament", {"index": index, "length": length, "speed": speed}))

    def feed_filament(self, index, length, speed):
        self.calls.append(
            ("feed_filament", {"index": index, "length": length, "speed": speed}))

    def stop_feed_assist_sync(self, index, timeout=2.0):
        self.calls.append(("stop_feed_assist_sync", {"index": index}))


class _MoveAce2(_Ace2_rfid):
    """ACE2 stub that records feed/unwind and the feed-complete waits."""

    def __init__(self):
        self._ace = _MoveConn()
        self.feed_speed = 80.0
        self.waits = []

    def _wait_for_feed_complete(self, slot, length, speed, **kw):
        self.waits.append((slot, length, speed))
        return True


def _sister_obj(monkeypatch, opts=None, sibling_kwargs=None, printing=False):
    """Staging obj on reader1 (slot 2 active = lane2, slot 3 sibling = lane3),
    with a movable sibling by default (idle, staged at hub, not printing)."""
    a = _MoveAce2()
    a._slot_map = {"lane2": 2, "lane3": 3}
    base = {"lane_slot_map": "lane2:2, lane3:3", "stage_read": True,
            "probe_settle": 0.0, "skip_factory_autostage": False}
    base.update(opts or {})
    obj = _make_rfid(a, base)
    obj.logger = logging.getLogger("test.ace2_rfid")
    skw = {"name": "lane3", "prep_state": True, "tool_loaded": False,
           "loaded_to_hub": True, "spool_id": None}
    skw.update(sibling_kwargs or {})
    lane3 = types.SimpleNamespace(**skw)
    lane2 = types.SimpleNamespace(name="lane2", prep_state=True)
    obj.afc = types.SimpleNamespace(
        lanes={"lane2": lane2, "lane3": lane3},
        function=types.SimpleNamespace(is_printing=lambda: printing))
    return obj, a, lane2


def _sibling_block_read(seen=None, **kw):
    """A read that sees only the halted SISTER tag (own tag absent)."""
    if seen is not None:
        seen.append(("cafe", 0x08, True))
    return None


def _sister_scan_obj(monkeypatch, sibling_kwargs=None, printing=False, opts=None):
    """Staging obj on reader1 (slot 2 active = lane2, slot 3 sibling = lane3),
    with a movable sibling by default (idle, staged at hub, not printing) whose
    parked tag 'cafe' dominates the shared antenna."""
    a = _ScanAce2(moving_polls=1)
    obj, a = _scan_obj(monkeypatch, ace2=a, lane_slot={"lane2": 2, "lane3": 3},
                       opts=dict(opts or {}))
    skw = {"name": "lane3", "prep_state": True, "tool_loaded": False,
           "loaded_to_hub": True, "spool_id": None}
    skw.update(sibling_kwargs or {})
    lane3 = types.SimpleNamespace(**skw)
    lane2 = types.SimpleNamespace(name="lane2", prep_state=True)
    obj.afc = types.SimpleNamespace(
        lanes={"lane2": lane2, "lane3": lane3},
        function=types.SimpleNamespace(is_printing=lambda: printing))
    obj._slot_uid = {3: "cafe"}                       # sibling's parked tag
    return obj, a, lane2


def test_sister_retract_clears_then_reads_and_restores(monkeypatch):
    obj, a, lane2 = _sister_scan_obj(monkeypatch)     # movable sibling
    _patch_activate(monkeypatch, ["cafe", "beef"])    # sibling dominates, then own
    _patch_read_tag(monkeypatch, [], result=_decoded("beef"))
    monkeypatch.setattr(obj, "apply_to_lane", lambda lane, tag: None)
    ctx = {"active": False, "done": False}

    obj._stage_probe_begin(lane2, ctx)
    unwinds = [c for c in a._ace.calls if c[0] == "unwind_filament"]
    assert any(c[1]["index"] == 3 and c[1]["length"] == obj.auto_tag_adjust_dist
               for c in unwinds)                      # retracted the sibling
    assert ctx["done"] is True and obj._slot_uid[2] == "beef"
    assert obj._sister_retracted is not None

    obj._stage_probe_end(lane2, ctx)                  # restore the sibling
    feeds = [c for c in a._ace.calls
             if c[0] == "feed_filament" and c[1]["index"] == 3]
    assert feeds and feeds[-1][1]["length"] == obj.auto_tag_adjust_dist
    assert obj._sister_retracted is None


def test_sister_retract_only_moves_once_per_stage(monkeypatch):
    obj, a, lane2 = _sister_scan_obj(monkeypatch)
    _patch_activate(monkeypatch, ["cafe", "cafe", "cafe"])
    _patch_read_tag(monkeypatch, [], result=None)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(lane2, ctx)
    unwinds = [c for c in a._ace.calls
               if c[0] == "unwind_filament" and c[1]["index"] == 3]
    assert len(unwinds) == 1                          # guarded by _sister_retracted


def test_sister_not_moved_when_tool_loaded_but_user_told(monkeypatch):
    obj, a, lane2 = _sister_scan_obj(
        monkeypatch, sibling_kwargs={"tool_loaded": True})
    _patch_activate(monkeypatch, ["cafe", "cafe"])
    _patch_read_tag(monkeypatch, [], result=None)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(lane2, ctx)
    assert not any(c[0] == "unwind_filament" and c[1]["index"] == 3
                   for c in a._ace.calls)
    assert any("Manually move" in m and "lane3" in m for m in obj.gcode.info)


def test_sister_not_moved_when_printing_but_user_told(monkeypatch):
    obj, a, lane2 = _sister_scan_obj(monkeypatch, printing=True)
    _patch_activate(monkeypatch, ["cafe", "cafe"])
    _patch_read_tag(monkeypatch, [], result=None)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(lane2, ctx)
    assert not any(c[0] == "unwind_filament" and c[1]["index"] == 3
                   for c in a._ace.calls)
    assert any("Manually move" in m for m in obj.gcode.info)


def test_sister_not_moved_when_not_hub_staged(monkeypatch):
    obj, a, lane2 = _sister_scan_obj(
        monkeypatch, sibling_kwargs={"loaded_to_hub": False})
    _patch_activate(monkeypatch, ["cafe", "cafe"])
    _patch_read_tag(monkeypatch, [], result=None)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(lane2, ctx)
    assert not any(c[0] == "unwind_filament" and c[1]["index"] == 3
                   for c in a._ace.calls)
    assert any("Manually move" in m for m in obj.gcode.info)


def test_auto_tag_adjust_false_never_moves_but_tells_user(monkeypatch):
    obj, a, lane2 = _sister_scan_obj(monkeypatch, opts={"auto_tag_adjust": False})
    _patch_activate(monkeypatch, ["cafe", "cafe", "cafe"])
    _patch_read_tag(monkeypatch, [], result=None)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(lane2, ctx)
    assert not any(c[0] == "unwind_filament" and c[1]["index"] == 3
                   for c in a._ace.calls)
    hints = [m for m in obj.gcode.info if "Manually move" in m]
    assert len(hints) == 1                            # shown once, not spammed


def test_stage_scan_never_assigns_unmovable_sibling_parked_tag(monkeypatch):
    # The reported bug: a sibling spool's parked tag on the shared antenna must
    # NOT be read as this lane's — even when its UID is unknown (fresh session)
    # and the sibling can't be moved. Baseline captures the pre-spin tag and
    # excludes it for the whole scan.
    obj, a, lane2 = _sister_scan_obj(
        monkeypatch, sibling_kwargs={"tool_loaded": True})   # unmovable
    obj._slot_uid = {}                                       # sibling UID unknown
    _patch_activate(monkeypatch, ["cafe", "cafe", "cafe", "cafe"])  # only sibling
    _patch_read_tag(monkeypatch, [], result=_decoded("cafe"))
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(lane2, ctx)

    assert ctx["done"] is False                  # sibling tag NOT assigned
    assert 2 not in obj._slot_uid


def test_stage_scan_reads_own_tag_past_unmovable_sibling(monkeypatch):
    # With an unmovable sibling parked on the antenna, our own tag (sweeping in as
    # we feed) is a DIFFERENT UID from the baseline and is still read correctly.
    obj, a, lane2 = _sister_scan_obj(
        monkeypatch, sibling_kwargs={"tool_loaded": True})
    obj._slot_uid = {}
    _patch_activate(monkeypatch, ["cafe", "cafe", "beef"])   # sibling, then own
    _patch_read_tag(monkeypatch, [], result=_decoded("beef"))
    monkeypatch.setattr(obj, "apply_to_lane", lambda lane, tag: None)
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(lane2, ctx)

    assert ctx["done"] is True
    assert obj._slot_uid[2] == "beef"


def test_scanner_triggers_sister_retract(monkeypatch):
    # The spool_scanner also clears a dominating sibling automatically, then
    # restores it when the scan ends.
    obj, a, _lane2 = _sister_obj(
        monkeypatch, opts={"scan_interval": 0.0, "scanner_confirm_reads": 1})
    obj.reactor = _AdvReactor_rfid(step=0.5)              # advance so the scan deadline fires
    monkeypatch.setattr(rfid_rfid, "read_tag",
                        lambda link, is_excluded=None, seen=None, **kw:
                        _sibling_block_read(seen=seen))

    tag = obj._scan_slot(2, duration=2.0)            # scan the active slot (2)

    assert tag is None                               # sibling blocked the whole scan
    unwinds = [c for c in a._ace.calls if c[0] == "unwind_filament"]
    assert len(unwinds) == 1 and unwinds[0][1]["index"] == 3   # retracted the sibling
    # scan finished -> sibling restored
    assert any(c[0] == "feed_filament" and c[1]["index"] == 3 for c in a._ace.calls)
    assert obj._sister_retracted is None


# ── Bambu dual-color (block 16) decode + _map multi_color ─────────────────────

def _b16(count, abgr_bytes=None):
    """1024-byte image with block 16 populated: color count @258 (LE), second
    color @260 as 4 ABGR bytes."""
    img = bytearray(1024)
    img[258] = count & 0xFF
    img[259] = (count >> 8) & 0xFF
    if abgr_bytes:
        img[260:264] = bytes(abgr_bytes)
    return bytes(img)


def test_bambu_apply_multicolor_dual():
    # count=2, second color stored reversed-ABGR [A,B,G,R] = FF,33,22,11 -> #112233
    fil = {"color_argb": 0xFFE94B3C}
    readers._bambu_apply_multicolor(fil, _b16(2, [0xFF, 0x33, 0x22, 0x11]))
    assert fil["color_count"] == 2
    assert fil["colors_argb"] == [0xFFE94B3C, (0xFF << 24) | (0x11 << 16) | (0x22 << 8) | 0x33]


def test_bambu_apply_multicolor_single_count():
    fil = {"color_argb": 0xFFE94B3C}
    readers._bambu_apply_multicolor(fil, _b16(1))
    assert fil["color_count"] == 1 and fil["colors_argb"] == [0xFFE94B3C]


def test_bambu_apply_multicolor_missing_block16():
    fil = {"color_argb": 0xFFE94B3C}
    readers._bambu_apply_multicolor(fil, None)
    assert fil["color_count"] == 1 and fil["colors_argb"] == [0xFFE94B3C]


def test_bambu_apply_multicolor_count2_but_empty_second_stays_single():
    # count says 2 but the second-color bytes are all zero (block 16 not really
    # written) -> don't invent a black second colour.
    fil = {"color_argb": 0xFFE94B3C}
    readers._bambu_apply_multicolor(fil, _b16(2, [0, 0, 0, 0]))
    assert fil["colors_argb"] == [0xFFE94B3C]


def test_map_builds_multi_color_when_dual():
    obj = _bare_rfid(_Ace2_rfid(), lane_slot={"lane1": 0})
    tag = {"uid": "aa", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA", "manufacturer": "Bambu",
                        "color_argb": 0xFFE94B3C,
                        "colors_argb": [0xFFE94B3C, 0xFF112233]}}
    si = obj._map(tag)
    assert si["color_hex"] == "e94b3c"
    assert si["multi_color"] == ["e94b3c", "112233"]
    assert si["is_dual_color"] is True


def test_map_single_color_not_dual():
    obj = _bare_rfid(_Ace2_rfid(), lane_slot={"lane1": 0})
    tag = {"uid": "aa", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA", "manufacturer": "Bambu",
                        "color_argb": 0xFFE94B3C, "colors_argb": [0xFFE94B3C]}}
    si = obj._map(tag)
    assert si["multi_color"] == ["e94b3c"] and si["is_dual_color"] is False


# ── BQ Tech (BigTreeTech MMS / ViViD) decode ─────────────────────────────────

def _le16(v):
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def _build_btt_image(version=1000, manufacturer="BQ Tech", mfg="20240812_162600",
                     material="PET", detailed="PET (CEP)", serial="IP243ZCXV67",
                     rgb=(0x12, 0x34, 0x56), diameter=1750, weight=1000,
                     ptmin=200, ptmax=240, bed_max=60, bed=60,
                     density=0, drying_time=0, drying_temp=0):
    """Assemble a 1024-byte MIFARE Classic image in BTT's BQ Tech layout
    (block N at byte N*16; BTT source offsets are hex-chars, halved to bytes)."""
    d = bytearray(1024)

    def put(block, off, raw):
        p = block * 16 + off
        d[p:p + len(raw)] = raw

    put(1, 0, _le16(version))                        # tag_version
    put(1, 2, manufacturer.encode("ascii"))          # filament_manufacturer
    put(2, 0, mfg.encode("ascii"))                   # manufacture_datetime
    put(4, 0, material.encode("ascii"))              # filament_material_type
    put(5, 0, detailed.encode("ascii"))              # filament_type_detailed
    put(6, 0, serial.encode("ascii"))                # serial_number
    put(8, 0, bytes(rgb))                            # color_code RRGGBB
    put(10, 0, _le16(diameter))                      # filament_diameter
    put(10, 2, _le16(density))                       # density (1240 -> 1.240)
    put(17, 0, _le16(weight))                        # spool_weight
    put(18, 0, _le16(drying_time))                   # drying_time (hours)
    put(18, 4, _le16(drying_temp))                   # drying_temp_max (C)
    put(18, 8, _le16(bed_max))                       # bed_temerature_max
    put(18, 10, _le16(ptmin))                        # printing_temperature_min
    put(18, 12, _le16(ptmax))                        # printing_temperature_max
    put(20, 0, _le16(bed))                           # bed_temperature
    return bytes(d)


def test_decode_btt_fields():
    d = _build_btt_image()
    f = readers.decode_btt(d)
    assert f is not None
    assert f["manufacturer"] == "BQ Tech"
    assert f["type"] == "PET"
    assert f["detailed"] == "PET (CEP)"
    assert f["sku"] == "IP243ZCXV67"
    assert f["production"] == "20240812_162600"
    # color_code 12 34 56 -> ARGB 0xFF123456
    assert f["color_argb"] == 0xFF123456
    assert round(f["diameter_mm"], 3) == 1.75         # 1750 / 1000
    assert f["weight_g"] == 1000
    assert f["hotend_min_c"] == 200 and f["hotend_max_c"] == 240
    assert f["bed_temp_c"] == 60


def test_decode_btt_density_parsed():
    # 1240 on the tag -> 1.240 g/cm^3 (re-derived: 1240 / 1000)
    f = readers.decode_btt(_build_btt_image(density=1240))
    assert f["density"] == 1240 / 1000


def test_decode_btt_density_zero_is_none():
    f = readers.decode_btt(_build_btt_image(density=0))
    assert f["density"] is None


def test_decode_btt_drying_parsed():
    f = readers.decode_btt(_build_btt_image(drying_time=8, drying_temp=70))
    assert f["drying_time_h"] == 8
    assert f["drying_temp_c"] == 70


def test_decode_btt_drying_zero_is_none():
    f = readers.decode_btt(_build_btt_image(drying_time=0, drying_temp=0))
    assert f["drying_time_h"] is None
    assert f["drying_temp_c"] is None


def test_decode_btt_fingerprint_rejects_non_btt():
    # A dump read with the default FF key that ISN'T a BQ Tech tag (version != 1000)
    # must be rejected so it can't mis-decode a foreign tag.
    d = bytearray(_build_btt_image(version=2))
    assert readers.decode_btt(bytes(d)) is None
    # A blank/all-zero image is likewise rejected.
    assert readers.decode_btt(bytes(1024)) is None


def test_decode_btt_maps_to_slot_info_single_color():
    # The shared _map() turns a BQ Tech decode into AFC slot_info like any brand.
    obj = _bare_rfid(_Ace2_rfid(), lane_slot={"lane1": 0})
    tag = {"uid": "aabbccdd", "tag_type": "MifareClassic1k",
           "filament": readers.decode_btt(_build_btt_image(rgb=(0xC0, 0xFF, 0xEE)))}
    si = obj._map(tag)
    assert si["brand"] == "BQ Tech"
    assert si["material"] == "PET"
    assert si["color_hex"] == "c0ffee"
    assert si["multi_color"] == ["c0ffee"]
    assert si["is_dual_color"] is False
    assert si["uid"] == "aabbccdd"


class _FakeMc:
    """Minimal MifareClassic stand-in: returns a prebuilt 1024-byte image for
    read_blocks so _classic_btt can be exercised without hardware."""
    def __init__(self, image):
        self._img = image

    def read_blocks(self, uid, keys_a, blocks):
        return self._img


def test_classic_btt_reads_and_decodes():
    mc = _FakeMc(_build_btt_image(material="PLA"))
    f = readers._classic_btt(mc, b"\xaa\xbb\xcc\xdd")
    assert f is not None and f["type"] == "PLA" and f["manufacturer"] == "BQ Tech"


def test_classic_btt_none_when_read_fails():
    class _NoRead:
        def read_blocks(self, *a):
            return None
    assert readers._classic_btt(_NoRead(), b"\x01\x02\x03\x04") is None


# ── get_status: lane map + last_reads records ────────────────────────────────

class TestAce2GetStatus:
    def test_shape_when_empty(self):
        obj = _bare_rfid(_Ace2_rfid(), lane_slot={"lane1": 0})
        status = obj.get_status()
        assert status["lane_slot_map"] == {"lane1": 0}
        assert status["last_reads"] == {}

    def test_last_reads_after_record(self):
        obj = _bare_rfid(_Ace2_rfid(), lane_slot={"lane1": 0})
        obj.record_tag_read("lane1", {"material": "PLA", "uid": "AABB"})
        rec = obj.get_status()["last_reads"]["lane1"]
        assert rec["material"] == "PLA"
        assert rec["uid"] == "AABB"
        assert rec["decoded"] is True


# ── Additional branch-coverage tests for extras/AFC_ACE2_rfid.py and the shared ───
#
# was tests/test_AFC_ACE2_rfid_coverage.py
HERE_rfid_coverage = os.path.dirname(os.path.abspath(__file__))
ROOT_rfid_coverage = os.path.dirname(HERE_rfid_coverage)


def _load_rfid_coverage(name, relpath):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT_rfid_coverage, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# Package import (not _load_rfid_coverage) so this is the SAME module object whose globals
# rfid_rfid_coverage.read_tag resolves — patching readers.MifareClassic must affect it.

rfid_rfid_coverage = _load_rfid_coverage("AFC_ACE2_rfid", "extras/AFC_ACE2_rfid.py")


# ── stubs ────────────────────────────────────────────────────────────────────
class _Logger:
    """Recording logger: stores (level, fully-formatted-message) tuples."""

    def __init__(self):
        self.messages = []

    def _rec(self, level, msg, args):
        self.messages.append((level, msg % args if args else msg))

    def info(self, msg, *args):
        self._rec("info", msg, args)

    def warning(self, msg, *args):
        self._rec("warning", msg, args)

    def exception(self, msg, *args):
        self._rec("exception", msg, args)

    def debug(self, msg, *args):
        self._rec("debug", msg, args)


class _CmdError_rfid_coverage(Exception):
    pass


class _Reactor_rfid_coverage:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        return waketime

    def register_callback(self, cb, waketime=None):
        cb(0.0)


class _RecReactor:
    """Records callbacks without running them."""

    def __init__(self):
        self.callbacks = []

    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        return waketime

    def register_callback(self, cb, waketime=None):
        self.callbacks.append((cb, waketime))


class _AdvReactor_rfid_coverage:
    def __init__(self, step=0.5):
        self._t = 0.0
        self._step = step

    def monotonic(self):
        return self._t

    def pause(self, when):
        self._t += self._step
        return when

    def register_callback(self, cb, when=None):
        pass


class _Gcode_rfid_coverage:
    def __init__(self):
        self.commands = {}
        self.raw = []
        self.info = []

    def register_command(self, name, func, desc=None):
        self.commands[name] = func

    def respond_info(self, msg, log=True):
        self.info.append(msg)

    def respond_raw(self, msg):
        self.raw.append(msg)


class _Printer_rfid_coverage:
    command_error = _CmdError_rfid_coverage

    def __init__(self):
        self.reactor = _Reactor_rfid_coverage()
        self.gcode = _Gcode_rfid_coverage()
        self.events = []

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        return default

    def register_event_handler(self, name, cb):
        self.events.append((name, cb))


class _Config_rfid_coverage:
    def __init__(self, printer, opts):
        self._printer = printer
        self._opts = opts

    def get_printer(self):
        return self._printer

    def get(self, key, default=None):
        return self._opts.get(key, default)

    def getboolean(self, key, default=None):
        return bool(self._opts.get(key, default))

    def getint(self, key, default=None, minval=None):
        return int(self._opts.get(key, default))

    def getfloat(self, key, default=None, minval=None):
        return float(self._opts.get(key, default))


class _GCmd_rfid_coverage:
    error = _CmdError_rfid_coverage

    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_int(self, key, default=0):
        return int(self.params.get(key, default))

    def get_float(self, key, default=0.0, minval=None, maxval=None):
        return float(self.params.get(key, default))

    def respond_info(self, msg):
        self.responses.append(msg)


class _Conn_rfid_coverage:
    connected = True

    def __init__(self):
        self.calls = []

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))


class _Ace2_rfid_coverage:
    def __init__(self):
        self._ace = _Conn_rfid_coverage()


def _mk(ace2_obj, opts=None):
    """AFC_ACE2_RFID via the real constructor, with a recording logger and the
    ACE2 object bound directly (klippy:ready is not fired in tests)."""
    obj = rfid_rfid_coverage.AFC_ACE2_RFID(_Config_rfid_coverage(_Printer_rfid_coverage(), dict(opts or {})))
    obj.logger = _Logger()
    obj.ace2 = ace2_obj
    return obj


# ── Ace2Link.reg_read / reg_write / _parse_field1 ────────────────────────────
class _CapTx:
    def __init__(self, resp=b""):
        self.frames = []
        self._resp = resp

    def __call__(self, frame):
        self.frames.append(frame)
        return self._resp


def _build_resp(cmd, payload):
    return rfid_rfid_coverage.Ace2Link(lambda f: f).build_frame(cmd, payload)


class TestAce2LinkRegRead:
    def test_returns_parsed_value_and_encodes_arg(self):
        val = 0x42
        resp = _build_resp(rfid_rfid_coverage.CMD_MFRC522_REG_READ, b"\x08" + rfid_rfid_coverage._varint(val))
        tx = _CapTx(resp)
        link = rfid_rfid_coverage.Ace2Link(tx, slot=1)
        assert link.reg_read(0x0A) == val
        sent = tx.frames[0]
        payload = sent[7:7 + sent[6]]
        arg, _ = rfid_rfid_coverage._varint_decode(payload, 1)
        assert arg == (1 << 16) | 0x0A


class TestAce2LinkRegWrite:
    def test_encodes_reg_and_val_into_arg(self):
        tx = _CapTx()
        link = rfid_rfid_coverage.Ace2Link(tx, slot=2)
        assert link.reg_write(0x0A, 0x99) is None
        sent = tx.frames[0]
        payload = sent[7:7 + sent[6]]
        arg, _ = rfid_rfid_coverage._varint_decode(payload, 1)
        assert arg == (2 << 16) | (0x0A << 8) | 0x99


class TestAce2LinkParseField1:
    def test_bad_preamble_raises(self):
        with pytest.raises(IOError):
            rfid_rfid_coverage.Ace2Link._parse_field1(b"\x00\x00\x00\x00\x00\x00\x00")

    def test_empty_raises(self):
        with pytest.raises(IOError):
            rfid_rfid_coverage.Ace2Link._parse_field1(b"")

    def test_field1_value_decoded(self):
        resp = _build_resp(rfid_rfid_coverage.CMD_MFRC522_REG_READ, b"\x08" + rfid_rfid_coverage._varint(7))
        assert rfid_rfid_coverage.Ace2Link._parse_field1(resp) == 7

    def test_non_field1_payload_returns_zero(self):
        resp = _build_resp(rfid_rfid_coverage.CMD_MFRC522_REG_READ, b"\x0a\x01")
        assert rfid_rfid_coverage.Ace2Link._parse_field1(resp) == 0

    def test_short_payload_returns_zero(self):
        resp = _build_resp(rfid_rfid_coverage.CMD_MFRC522_REG_READ, b"")
        assert rfid_rfid_coverage.Ace2Link._parse_field1(resp) == 0


# ── Mfrc522._to_card ─────────────────────────────────────────────────────────
class _ToCardLink:
    def __init__(self, comirq, errorreg=0, fifo=b"", ctrl=0):
        self.comirq = comirq
        self.errorreg = errorreg
        self.fifo_bytes = list(fifo)
        self.ctrl = ctrl
        self.writes = []

    def reg_read(self, r):
        if r == readers.ComIrqReg:
            return self.comirq
        if r == readers.ErrorReg:
            return self.errorreg
        if r == readers.FIFOLevelReg:
            return len(self.fifo_bytes)
        if r == readers.FIFODataReg:
            return self.fifo_bytes.pop(0) if self.fifo_bytes else 0
        if r == readers.ControlReg:
            return self.ctrl
        return 0

    def reg_write(self, r, v):
        self.writes.append((r, v))


class TestMfrc522ToCard:
    def test_timer_irq_returns_failure(self):
        link = _ToCardLink(comirq=0x01)          # Timer bit, no Rx/Idle
        m = rfid_rfid_coverage.Mfrc522(link)
        assert m._to_card(readers.PCD_TRANSCEIVE, b"\x26", 7) == (False, b"", 0)

    def test_error_reg_returns_failure(self):
        link = _ToCardLink(comirq=0x30, errorreg=0x02)   # Rx set, error bit
        m = rfid_rfid_coverage.Mfrc522(link)
        assert m._to_card(readers.PCD_TRANSCEIVE, b"\x26", 7) == (False, b"", 0)

    def test_poll_times_out_then_reads_fifo(self):
        # ComIrq never sets Rx/Idle/Timer -> the 2000-poll loop runs to
        # completion, then the error check passes and the FIFO is read.
        link = _ToCardLink(comirq=0x00, errorreg=0x00, fifo=b"\xab\xcd", ctrl=0x03)
        m = rfid_rfid_coverage.Mfrc522(link)
        ok, rx, last = m._to_card(readers.PCD_TRANSCEIVE, b"\x26", 7)
        assert ok is True and rx == b"\xab\xcd" and last == 3

    def test_authent_returns_no_rx(self):
        link = _ToCardLink(comirq=0x30, errorreg=0x00)
        m = rfid_rfid_coverage.Mfrc522(link)
        assert m._to_card(readers.PCD_MFAUTHENT, b"\x60\x00") == (True, b"", 0)


# ── Mfrc522.anticoll ─────────────────────────────────────────────────────────
def _mfrc_with_tocard(result):
    m = rfid_rfid_coverage.Mfrc522(object())
    m._to_card = lambda cmd, send, tx_last_bits=0: result
    return m


class TestMfrc522Anticoll:
    def test_transceive_failure_returns_none(self):
        m = _mfrc_with_tocard((False, b"", 0))
        m.l = _ToCardLink(comirq=0)
        assert m.anticoll() is None

    def test_bad_bcc_returns_none(self):
        m = _mfrc_with_tocard((True, bytes([1, 2, 3, 4, 5]), 0))   # BCC 4 != 5
        m.l = _ToCardLink(comirq=0)
        assert m.anticoll() is None

    def test_good_bcc_returns_uid(self):
        m = _mfrc_with_tocard((True, bytes([1, 2, 3, 4, 4]), 0))   # 1^2^3^4 == 4
        m.l = _ToCardLink(comirq=0)
        assert m.anticoll() == bytes([1, 2, 3, 4])


# ── Mfrc522.antenna_on ───────────────────────────────────────────────────────
class _AntennaLink:
    def __init__(self, txcontrol):
        self._txcontrol = txcontrol
        self.writes = []

    def reg_read(self, r):
        return self._txcontrol if r == readers.TxControlReg else 0

    def reg_write(self, r, v):
        self.writes.append((r, v))


class TestMfrc522AntennaOn:
    def test_turns_on_when_off(self):
        link = _AntennaLink(txcontrol=0x00)
        rfid_rfid_coverage.Mfrc522(link).antenna_on()
        assert (readers.TxControlReg, 0x03) in link.writes

    def test_noop_when_already_on(self):
        link = _AntennaLink(txcontrol=0x03)
        rfid_rfid_coverage.Mfrc522(link).antenna_on()
        assert link.writes == []


# ── MifareClassic._activate_once ─────────────────────────────────────────────
class _AoMfrc:
    def __init__(self, req_wupa=b"\x04\x00", req_reqa=b"\x04\x00",
                 uid=b"\x01\x02\x03\x04", sak=0x08):
        self._req_wupa = req_wupa
        self._req_reqa = req_reqa
        self._uid = uid
        self._sak = sak

    def request(self, req):
        return self._req_wupa if req == readers.PICC_WUPA else self._req_reqa

    def anticoll(self):
        return self._uid

    def select(self, uid):
        return self._sak


class TestMifareClassicActivateOnce:
    def test_wake_success(self):
        mc = rfid_rfid_coverage.MifareClassic(_AoMfrc())
        assert mc._activate_once(wake=True) == (b"\x01\x02\x03\x04", 0x08)

    def test_wake_both_none_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_AoMfrc(req_wupa=None, req_reqa=None))
        assert mc._activate_once(wake=True) == (None, None)

    def test_no_wake_reqa_none_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_AoMfrc(req_reqa=None))
        assert mc._activate_once(wake=False) == (None, None)

    def test_anticoll_none_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_AoMfrc(uid=None))
        assert mc._activate_once(wake=False) == (None, None)

    def test_select_none_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_AoMfrc(sak=None))
        assert mc._activate_once(wake=False) == (None, None)


# ── MifareClassic.read_ntag ──────────────────────────────────────────────────
class _BlockMfrc:
    def __init__(self, blocks, auth_ok=True):
        self._blocks = blocks
        self._auth_ok = auth_ok

    def read_block(self, block):
        return self._blocks.get(block)

    def auth(self, key_type, block, key6, uid4):
        return self._auth_ok


class TestMifareClassicReadNtag:
    def test_reads_until_nbytes(self):
        pages = {p: bytes([p]) * 16 for p in range(0, 32, 4)}
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc(pages))
        data = mc.read_ntag(128)
        assert len(data) == 128 and data[:16] == bytes([0]) * 16

    def test_missing_page_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc({}))
        assert mc.read_ntag(128) is None


# ── MifareClassic.read_all ───────────────────────────────────────────────────
class TestMifareClassicReadAll:
    def test_auth_failure_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc({}, auth_ok=False))
        assert mc.read_all(b"\x01\x02\x03\x04", [[0] * 6], sectors=1) is None

    def test_read_block_failure_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc({}, auth_ok=True))
        assert mc.read_all(b"\x01\x02\x03\x04", [[0] * 6], sectors=1) is None

    def test_success_returns_image(self):
        blocks = {b: bytes([b]) * 16 for b in range(4)}
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc(blocks, auth_ok=True))
        out = mc.read_all(b"\x01\x02\x03\x04", [[0] * 6], sectors=1)
        assert len(out) == 64 and out[16:18] == bytes([1, 1])


# ── MifareClassic.read_blocks ────────────────────────────────────────────────
class TestMifareClassicReadBlocks:
    def test_auth_failure_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc({}, auth_ok=False))
        assert mc.read_blocks(b"\x01\x02\x03\x04", [[0] * 6] * 16, (4,)) is None

    def test_read_block_failure_returns_none(self):
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc({}, auth_ok=True))
        assert mc.read_blocks(b"\x01\x02\x03\x04", [[0] * 6] * 16, (4,)) is None

    def test_success_places_blocks(self):
        blocks = {5: bytes([0x55]) * 16}
        mc = rfid_rfid_coverage.MifareClassic(_BlockMfrc(blocks, auth_ok=True))
        out = mc.read_blocks(b"\x01\x02\x03\x04", [[0] * 6] * 16, (5,))
        assert out[80:82] == bytes([0x55, 0x55]) and len(out) == 1024


# ── brand decode edge cases ──────────────────────────────────────────────────
class TestDecodeBambu:
    def test_wrong_length_raises(self):
        with pytest.raises(ValueError):
            readers.decode_bambu(b"\x00" * 100)

    def test_optional_fields_present(self):
        d = bytearray(1024)
        struct.pack_into("<f", d, 140, 0.4)      # nozzle in (0, 2)
        struct.pack_into("<H", d, 164, 6000)     # spool width 60.0 mm
        struct.pack_into("<H", d, 228, 250)      # length 250 m
        out = readers.decode_bambu(bytes(d))
        assert out["nozzle_diameter"] == 0.4
        assert out["spool_width_mm"] == 60.0
        assert out["length_m"] == 250

    def test_optional_fields_absent(self):
        d = bytearray(1024)
        struct.pack_into("<f", d, 140, 5.0)      # nozzle out of (0, 2) -> None
        out = readers.decode_bambu(bytes(d))
        assert out["nozzle_diameter"] is None
        assert out["spool_width_mm"] is None
        assert out["length_m"] is None


class TestDecodeAnycubic:
    def test_short_returns_none(self):
        assert readers.decode_anycubic(b"\x00" * 0x40) is None

    def test_wrong_magic_returns_none(self):
        assert readers.decode_anycubic(b"\x11" * 0x80) is None


class TestDecodeSnapmaker:
    def test_short_returns_none(self):
        assert readers.decode_snapmaker(b"\x00" * 100) is None


class TestDecodeCreality:
    def test_short_returns_none(self):
        assert readers.decode_creality(b"\x00" * 33) is None

    def test_bad_hex_color_and_length_default(self):
        payload = (b"PRODU" + b"0276" + b"XX" + b"101001"
                   + b"ZZZZZZZ" + b"YYYY" + b"SERIAL" + b"\x00" * 14)
        assert len(payload) == 48
        out = readers.decode_creality(payload)
        assert out["type"] == "PLA"
        assert out["color_argb"] is None       # color hex unparsable
        assert out["length_m"] is None         # length hex unparsable
        assert out["weight_g"] is None         # length 0 -> no weight


class TestDecodeBtt:
    def test_short_returns_none(self):
        assert readers.decode_btt(b"\x00" * 300) is None


# ── read_tag branches ────────────────────────────────────────────────────────
class TestReadTag:
    def test_no_tag_returns_none(self, monkeypatch):
        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self, is_excluded=None, seen=None, reset=True):
                return None, None

        monkeypatch.setattr(readers, "MifareClassic", _MC)
        assert readers.read_tag(object()) is None

    def test_dump_blocks_attaches_raw_blocks(self, monkeypatch):
        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self, is_excluded=None, seen=None, reset=True):
                return b"\x04\xa1\xb2\xc3", 0x08

            def read_blocks(self, uid, keys, blocks):
                img = bytearray(1024)
                for b in blocks:
                    img[b * 16:b * 16 + 16] = bytes([b & 0xFF]) * 16
                return bytes(img)

        monkeypatch.setattr(readers, "MifareClassic", _MC)
        master = bytes.fromhex("00" * 16)
        out = readers.read_tag(object(), bambu_master_key=master,
                               dump_blocks=(5, 16))
        assert out["raw_blocks"][5] == (bytes([5]) * 16).hex()
        assert out["raw_blocks"][16] == (bytes([16]) * 16).hex()

    def test_ntag_without_decode(self, monkeypatch):
        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self, is_excluded=None, seen=None, reset=True):
                return b"\x04\xa1\xb2\xc3", 0x00      # Ultralight/NTAG

            def read_ntag(self, n=128):
                return b"\x11" * n                    # not Anycubic/Elegoo

        monkeypatch.setattr(readers, "MifareClassic", _MC)
        out = readers.read_tag(object())
        assert out["tag_type"] == "MifareUltralight"
        assert out["filament"] is None


# ── _Ace2RegLink reg_read / reg_write / _conn ────────────────────────────────
class _ValConn:
    connected = True

    def __init__(self, val=None):
        self.calls = []
        self._val = val

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return self._val

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))


class TestAce2RegLinkRegRead:
    def test_masks_value_and_encodes_arg(self):
        conn = _ValConn({"val": 0x1FF})
        ace2 = types.SimpleNamespace(_ace=conn)
        link = rfid_rfid_coverage._Ace2RegLink(ace2, 1)
        assert link.reg_read(0x0A) == 0xFF
        assert conn.calls[-1] == ("mfrc522_reg_read", {"arg": (1 << 16) | 0x0A})

    def test_none_response_reads_zero(self):
        conn = _ValConn(None)
        ace2 = types.SimpleNamespace(_ace=conn)
        assert rfid_rfid_coverage._Ace2RegLink(ace2, 0).reg_read(0x05) == 0


class TestAce2RegLinkRegWrite:
    def test_encodes_arg(self):
        conn = _ValConn({})
        ace2 = types.SimpleNamespace(_ace=conn)
        rfid_rfid_coverage._Ace2RegLink(ace2, 2).reg_write(0x0A, 0x99)
        assert conn.calls[-1] == (
            "mfrc522_reg_write", {"arg": (2 << 16) | (0x0A << 8) | 0x99})


class TestAce2RegLinkConn:
    def test_missing_serial_raises_ioerror(self):
        ace2 = types.SimpleNamespace(_ace=None)
        with pytest.raises(IOError):
            rfid_rfid_coverage._Ace2RegLink(ace2, 0).reg_read(0x01)


# ── _on_ready ────────────────────────────────────────────────────────────────
class _ReadyPrinter:
    command_error = _CmdError_rfid_coverage

    def __init__(self, objects, named=None):
        self._objects = objects
        self._named = named or []

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)

    def lookup_objects(self):
        return list(self._objects.items()) + self._named

    def register_event_handler(self, name, cb):
        pass


class TestOnReady:
    def test_binds_ace2_and_marks_scanner(self, monkeypatch):
        obj = _mk(_Ace2_rfid_coverage(), {"scanner_lanes": "scan1"})
        monkeypatch.setattr(
            rfid_rfid_coverage, "resolve_rfid_keys",
            lambda pr, b, c, d: (b"\xaa", c, d))
        ace2obj = types.SimpleNamespace(name="Ace2_1", _ace=None)
        lane = types.SimpleNamespace(name="scan1")
        afc = types.SimpleNamespace(lanes={"scan1": lane})
        obj.printer = _ReadyPrinter({"AFC": afc, "AFC_ACE2": ace2obj})
        obj.reactor = _RecReactor()
        obj._on_ready()
        assert obj.ace2 is ace2obj
        assert obj.bambu_master_key == b"\xaa"
        assert lane.spool_scanner is True
        assert obj.logger.messages == [("info", "ACE2 RFID bound to Ace2_1")]
        assert len(obj.reactor.callbacks) == 1     # retry-disable scheduled

    def test_warns_when_ace2_missing(self, monkeypatch):
        obj = _mk(_Ace2_rfid_coverage(), {})
        monkeypatch.setattr(
            rfid_rfid_coverage, "resolve_rfid_keys", lambda pr, b, c, d: (b, c, d))
        obj.printer = _ReadyPrinter({})
        obj.reactor = _RecReactor()
        obj._on_ready()
        assert obj.ace2 is None
        assert obj.logger.messages == [
            ("warning", "ACE2 object not found; RFID disabled")]

    def test_discovers_named_object_no_callback_when_not_skipping(
            self, monkeypatch):
        obj = _mk(_Ace2_rfid_coverage(), {"skip_factory_autostage": False})
        monkeypatch.setattr(rfid_rfid_coverage, "resolve_rfid_keys", None)
        ace2obj = types.SimpleNamespace(name="Ace2_9", _ace=None)
        obj.printer = _ReadyPrinter({}, named=[("AFC_ACE2 Ace2_9", ace2obj)])
        obj.reactor = _RecReactor()
        obj._on_ready()
        assert obj.ace2 is ace2obj
        assert obj.logger.messages == [("info", "ACE2 RFID bound to Ace2_9")]
        assert obj.reactor.callbacks == []          # skip off -> no retry


# ── _disable_factory_identify ────────────────────────────────────────────────
class _RaisingConn:
    connected = True

    def send_command_async(self, name, params=None):
        raise RuntimeError("not ready")


class TestDisableFactoryIdentify:
    def test_noop_without_ace2(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.ace2 = None
        obj._disable_factory_identify()
        assert obj.logger.messages == []

    def test_logs_when_enable_call_fails(self):
        ace2 = types.SimpleNamespace(_ace=_RaisingConn())
        obj = _mk(ace2, {})
        obj._disable_factory_identify([0])
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: could not disable factory identify on "
                     "slot 0 yet (serial not ready?)")]

    def test_logs_success_per_slot(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj._disable_factory_identify([1])
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: factory identify disabled on slot 1")]


# ── _auto_read ───────────────────────────────────────────────────────────────
class TestAutoRead:
    def test_command_error_logged_and_returns(self):
        obj = _mk(_Ace2_rfid_coverage(), {"read_on_insert_attempts": 3})
        obj.read_lane = lambda name: (_ for _ in ()).throw(_CmdError_rfid_coverage("boom"))
        obj._auto_read("lane1")
        assert obj.logger.messages == [("info", "ACE2 RFID auto-read lane1: boom")]

    def test_retries_then_reports_no_tag(self):
        obj = _mk(_Ace2_rfid_coverage(), {"read_on_insert_attempts": 2,
                            "read_on_insert_delay": 0.1})
        obj.reactor = _Reactor_rfid_coverage()

        def _raise(name):
            raise RuntimeError("glitch")

        obj.read_lane = _raise
        obj._auto_read("lane1")
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID auto-read failed for lane1"),
            ("exception", "ACE2 RFID auto-read failed for lane1"),
            ("info", "ACE2 RFID auto-read lane1: no tag after 2 attempts")]

    def test_success_stops_without_logging(self):
        obj = _mk(_Ace2_rfid_coverage(), {"read_on_insert_attempts": 3})
        obj.read_lane = lambda name: {"uid": "aa"}
        obj._auto_read("lane1")
        assert obj.logger.messages == []


# ── _spool_details ───────────────────────────────────────────────────────────
def _slot_info():
    return {"brand": "Bambu", "material": "PLA", "color_hex": "#112233",
            "diameter": 1.75, "extruder_temp": 220, "bed_temp": 60,
            "weight_g": 1000, "uid": "aa"}


class TestSpoolDetails:
    def test_base_when_no_spool_id(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.afc = types.SimpleNamespace(moonraker=object())
        d = obj._spool_details(None, _slot_info())
        assert d["brand"] == "Bambu" and d["color"] == "112233"
        assert d["name"] == "" and d["weight"] == 1000

    def test_base_when_moonraker_missing(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.afc = types.SimpleNamespace(moonraker=None)
        d = obj._spool_details(5, _slot_info())
        assert d["material"] == "PLA" and d["name"] == ""

    def test_base_when_get_spool_raises(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        mr = types.SimpleNamespace(
            get_spool=lambda sid: (_ for _ in ()).throw(RuntimeError("x")))
        obj.afc = types.SimpleNamespace(moonraker=mr)
        d = obj._spool_details(5, _slot_info())
        assert d["name"] == "" and d["brand"] == "Bambu"

    def test_base_when_spool_not_dict(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.afc = types.SimpleNamespace(
            moonraker=types.SimpleNamespace(get_spool=lambda sid: None))
        d = obj._spool_details(5, _slot_info())
        assert d["name"] == ""

    def test_enriched_from_spoolman(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        spool = {"remaining_weight": 750,
                 "filament": {"name": "Galaxy", "material": "PETG",
                              "color_hex": "#ff0000", "diameter": 1.75,
                              "settings_extruder_temp": 240,
                              "settings_bed_temp": 70,
                              "vendor": {"name": "Polymaker"}}}
        obj.afc = types.SimpleNamespace(
            moonraker=types.SimpleNamespace(get_spool=lambda sid: spool))
        d = obj._spool_details(5, _slot_info())
        assert d["name"] == "Galaxy" and d["brand"] == "Polymaker"
        assert d["material"] == "PETG" and d["color"] == "ff0000"
        assert d["ext"] == 240 and d["bed"] == 70
        assert d["weight"] == 750                # remaining preferred

    def test_weight_falls_back_when_no_remaining(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        spool = {"remaining_weight": None,
                 "filament": {"weight": 500, "vendor": {}}}
        obj.afc = types.SimpleNamespace(
            moonraker=types.SimpleNamespace(get_spool=lambda sid: spool))
        d = obj._spool_details(5, _slot_info())
        assert d["weight"] == 500


# ── _notify_scan ─────────────────────────────────────────────────────────────
class TestNotifyScan:
    def test_full_details_popup(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.reactor = _Reactor_rfid_coverage()
        obj.afc = types.SimpleNamespace(moonraker=None)
        obj._notify_scan(_slot_info(), "lane1", 120)
        raw = obj.gcode.raw
        assert any("Brand: Bambu" in r for r in raw)
        assert any("Diameter: 1.75mm" in r for r in raw)
        assert any("Nozzle temp: 220" in r for r in raw)
        assert any("Bed temp: 60" in r for r in raw)
        assert any("Remaining: 1000g" in r for r in raw)
        assert any("Spoolman ID: 120" in r for r in raw)
        assert any("action:prompt_end" in r for r in raw)   # auto-dismiss ran

    def test_uid_fallback_when_no_fields(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.reactor = _Reactor_rfid_coverage()
        obj.afc = types.SimpleNamespace(moonraker=None)
        obj._notify_scan({"uid": "deadbeef"}, "", None)
        assert any("uid: deadbeef" in r for r in obj.gcode.raw)

    def test_notification_error_logged(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj._spool_details = lambda sid, si: (_ for _ in ()).throw(
            RuntimeError("boom"))
        obj._notify_scan(_slot_info(), "lane1", 5)
        assert obj.logger.messages == [
            ("warning", "ACE2 RFID scan: notification error: boom")]


# ── cmd_ACE_RFID_READ ────────────────────────────────────────────────────────
class TestCmdRfidRead:
    def test_success_response_with_color(self, monkeypatch):
        obj = _mk(_Ace2_rfid_coverage(), {})
        monkeypatch.setattr(
            rfid_rfid_coverage, "read_tag",
            lambda link, **kw: {"uid": "deadbeef", "tag_type": "MifareClassic1k",
                                "filament": {"manufacturer": "Bambu",
                                             "type": "PLA",
                                             "color_argb": 0xFF112233}})
        gcmd = _GCmd_rfid_coverage({"SLOT": 0})
        obj.cmd_ACE_RFID_READ(gcmd)
        assert gcmd.responses == [
            "ACE2 RFID: uid=deadbeef type=MifareClassic1k brand=Bambu "
            "material=PLA color=112233"]

    def test_success_response_without_color(self, monkeypatch):
        obj = _mk(_Ace2_rfid_coverage(), {})
        monkeypatch.setattr(
            rfid_rfid_coverage, "read_tag",
            lambda link, **kw: {"uid": "aa", "tag_type": "MifareClassic1k",
                                "filament": {"manufacturer": "", "type": "",
                                             "color_argb": None}})
        gcmd = _GCmd_rfid_coverage({"SLOT": 0})
        obj.cmd_ACE_RFID_READ(gcmd)
        assert gcmd.responses == [
            "ACE2 RFID: uid=aa type=MifareClassic1k brand= material= color="]


# ── cmd_ACE_RFID_BLOCKS ──────────────────────────────────────────────────────
class TestCmdRfidBlocks:
    def test_bad_blocks_string(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        gcmd = _GCmd_rfid_coverage({"SLOT": 0, "BLOCKS": "x,y"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.responses == ["ACE2 RFID DUMP: bad BLOCKS='x,y'"]

    def test_lane_without_slot_raises(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        gcmd = _GCmd_rfid_coverage({"LANE": "nope", "BLOCKS": "5,16"})
        with pytest.raises(_CmdError_rfid_coverage):
            obj.cmd_ACE_RFID_BLOCKS(gcmd)

    def test_no_tag_found(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.read_slot = lambda slot, dump_blocks=None: None
        gcmd = _GCmd_rfid_coverage({"SLOT": 0, "BLOCKS": "5,16"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.responses == ["ACE2 RFID DUMP: no tag found"]

    def test_read_error_swallowed(self):
        obj = _mk(_Ace2_rfid_coverage(), {})

        def _boom(slot, dump_blocks=None):
            raise RuntimeError("wedge")

        obj.read_slot = _boom
        gcmd = _GCmd_rfid_coverage({"SLOT": 0, "BLOCKS": "5,16"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        assert gcmd.responses == ["ACE2 RFID DUMP: error: wedge"]
        assert obj.logger.messages == [("exception", "ACE2 RFID dump failed")]

    def test_dump_reports_dual_color(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        b5 = bytes([0x11, 0x22, 0x33, 0xFF]).hex()
        b16 = bytes([0x00, 0x00, 0x02, 0x00,      # fmt=0, count=2
                     0xFF, 0x33, 0x22, 0x11]).hex()   # A,B,G,R
        tag = {"uid": "aabb", "tag_type": "MifareClassic1k",
               "raw_blocks": {5: b5, 16: b16},
               "filament": {"colors_argb": [0xFF112233, 0xFF112233]}}
        obj.read_slot = lambda slot, dump_blocks=None: tag
        gcmd = _GCmd_rfid_coverage({"SLOT": 0, "BLOCKS": "5,16"})
        obj.cmd_ACE_RFID_BLOCKS(gcmd)
        out = gcmd.responses[0]
        assert "block5 primary -> #112233 (a=ff)" in out
        assert "color_count=2" in out
        assert "DUAL-COLOR (count=2)" in out
        assert "decoded colors: #112233, #112233" in out


# ── cmd_ACE_RFID_STAGE_TEST ──────────────────────────────────────────────────
class _StageConn:
    connected = True

    def __init__(self, feed_raises=False):
        self.calls = []
        self._feed_raises = feed_raises

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))

    def unwind_filament(self, index, length, speed, mode="normal"):
        self.calls.append(("unwind_filament", {"index": index,
                                                "length": length, "speed": speed}))

    def feed_filament(self, index, length, speed):
        if self._feed_raises:
            raise RuntimeError("feed rejected")
        self.calls.append(("feed_filament", {"index": index,
                                             "length": length, "speed": speed}))

    def stop_feed_filament(self, index):
        self.calls.append(("stop_feed_filament", {"index": index}))

    def get_status(self, timeout=2.0):
        return {"status": "ready"}


class _StageAce2:
    def __init__(self, conn, moving_polls=1):
        self._ace = conn
        self.feed_speed = 80.0
        self.retract_speed = 80.0
        self._slot_map = {"lane1": 0}
        self._moving_polls = moving_polls

    def _wait_for_ace_ready(self, timeout=30.0):
        return True

    def _slot_is_moving(self, status, slot):
        self._moving_polls -= 1
        return self._moving_polls >= 0

    def _slot_in_error(self, slot):
        return False


def _stage_obj(conn, moving_polls=1, opts=None):
    ace2 = _StageAce2(conn, moving_polls=moving_polls)
    base = {"lane_slot_map": "lane1:0", "skip_factory_autostage": False}
    base.update(opts or {})
    obj = _mk(ace2, base)
    obj.reactor = _AdvReactor_rfid_coverage(step=0.5)
    return obj, ace2


class TestCmdStageTest:
    def test_ace2_missing_raises(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.ace2 = None
        with pytest.raises(_CmdError_rfid_coverage):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd_rfid_coverage({"LANE": "lane1"}))

    def test_unknown_lane_raises(self):
        conn = _StageConn()
        obj, _ = _stage_obj(conn)
        with pytest.raises(_CmdError_rfid_coverage):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd_rfid_coverage({"LANE": "ghost"}))

    def test_serial_not_connected_raises(self):
        conn = _StageConn()
        conn.connected = False
        obj, _ = _stage_obj(conn)
        with pytest.raises(_CmdError_rfid_coverage):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd_rfid_coverage({"LANE": "lane1"}))

    def test_detect_and_decode_restages(self, monkeypatch):
        conn = _StageConn()
        obj, _ = _stage_obj(conn)

        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self):
                return b"\xbe\xef", 0x08          # detected on first poll

        monkeypatch.setattr(rfid_rfid_coverage, "MifareClassic", _MC)
        monkeypatch.setattr(rfid_rfid_coverage, "Mfrc522", lambda link: None)
        obj._read_tag = lambda link, **kw: {"uid": "beef",
                                            "filament": {"type": "PLA"}}
        gcmd = _GCmd_rfid_coverage({"LANE": "lane1", "DIST": 500})
        obj.cmd_ACE_RFID_STAGE_TEST(gcmd)
        names = [c[0] for c in conn.calls]
        assert names.count("unwind_filament") >= 1     # initial retract
        assert "stop_feed_filament" in names           # stopped on detect
        assert names[-1] == "feed_filament"            # re-staged remainder
        assert any("DETECTED" in r for r in gcmd.responses)
        assert any("DONE" in r for r in gcmd.responses)

    def test_no_detect_reports_and_restages(self, monkeypatch):
        conn = _StageConn()
        obj, _ = _stage_obj(conn, moving_polls=1)

        class _MC:
            def __init__(self, mfrc):
                pass

            def activate(self):
                return None, None                # never a tag

        monkeypatch.setattr(rfid_rfid_coverage, "MifareClassic", _MC)
        monkeypatch.setattr(rfid_rfid_coverage, "Mfrc522", lambda link: None)
        gcmd = _GCmd_rfid_coverage({"LANE": "lane1", "DIST": 500})
        obj.cmd_ACE_RFID_STAGE_TEST(gcmd)
        assert any("no tag detected during the feed" in r
                   for r in gcmd.responses)
        assert any("DONE" in r for r in gcmd.responses)

    def test_feed_error_path(self, monkeypatch):
        conn = _StageConn(feed_raises=True)
        obj, _ = _stage_obj(conn)
        monkeypatch.setattr(rfid_rfid_coverage, "MifareClassic",
                            lambda mfrc: types.SimpleNamespace(
                                activate=lambda: (None, None)))
        monkeypatch.setattr(rfid_rfid_coverage, "Mfrc522", lambda link: None)
        gcmd = _GCmd_rfid_coverage({"LANE": "lane1", "DIST": 500})
        obj.cmd_ACE_RFID_STAGE_TEST(gcmd)
        assert any("feed returned/err" in r for r in gcmd.responses)

    def test_initial_retract_failure_raises(self):
        conn = _StageConn()
        conn.unwind_filament = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("stuck"))
        obj, _ = _stage_obj(conn)
        with pytest.raises(_CmdError_rfid_coverage):
            obj.cmd_ACE_RFID_STAGE_TEST(_GCmd_rfid_coverage({"LANE": "lane1", "DIST": 500}))


# ── cmd_ACE_RFID_SCAN ────────────────────────────────────────────────────────
class TestCmdRfidScan:
    def test_error_swallowed(self):
        obj = _mk(_Ace2_rfid_coverage(), {"scanner_lanes": "scan1"})
        obj.scan_lane = lambda name, secs: (_ for _ in ()).throw(
            RuntimeError("glitch"))
        gcmd = _GCmd_rfid_coverage({"LANE": "scan1"})
        obj.cmd_ACE_RFID_SCAN(gcmd)
        assert obj.logger.messages == [("exception", "ACE2 RFID scan failed")]
        assert any("scan: error: glitch" in r for r in gcmd.responses)

    def test_no_tag(self):
        obj = _mk(_Ace2_rfid_coverage(), {"scanner_lanes": "scan1"})
        obj.scan_lane = lambda name, secs: None
        gcmd = _GCmd_rfid_coverage({"LANE": "scan1"})
        obj.cmd_ACE_RFID_SCAN(gcmd)
        assert any("no tag found on scan1" in r for r in gcmd.responses)

    def test_success_reports_staged_spool(self):
        obj = _mk(_Ace2_rfid_coverage(), {"scanner_lanes": "scan1"})
        obj.scan_lane = lambda name, secs: {"uid": "beef",
                                            "filament": {"type": "PLA"}}
        obj.afc = types.SimpleNamespace(
            spool=types.SimpleNamespace(next_spool_id=42))
        gcmd = _GCmd_rfid_coverage({"LANE": "scan1"})
        obj.cmd_ACE_RFID_SCAN(gcmd)
        assert any("staged next spool from scan1 — uid=beef type=PLA "
                   "(spool #42)" in r for r in gcmd.responses)


# ── motion stubs for sister-retract / teardown ───────────────────────────────
class _MotionConn:
    connected = True

    def __init__(self, unwind_raises=False):
        self.calls = []
        self._unwind_raises = unwind_raises

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        self.calls.append((name, dict(params or {})))

    def unwind_filament(self, index, length, speed, mode="normal"):
        if self._unwind_raises:
            raise RuntimeError("unwind wedged")
        self.calls.append(("unwind_filament", {"index": index,
                                               "length": length, "speed": speed}))

    def feed_filament(self, index, length, speed):
        self.calls.append(("feed_filament", {"index": index,
                                             "length": length, "speed": speed}))


class _MotionAce2:
    def __init__(self, conn):
        self._ace = conn
        self.feed_speed = 80.0
        self._slot_map = {"lane2": 2, "lane3": 3}

    def _wait_for_feed_complete(self, slot, length, speed, **kw):
        return True


def _retract_obj(opts=None, sibling=None, printing=False, conn=None):
    a_conn = conn if conn is not None else _MotionConn()
    ace2 = _MotionAce2(a_conn)
    base = {"lane_slot_map": "lane2:2, lane3:3", "auto_tag_adjust": True}
    base.update(opts or {})
    obj = _mk(ace2, base)
    skw = {"name": "lane3", "tool_loaded": False, "loaded_to_hub": True}
    skw.update(sibling or {})
    lane3 = types.SimpleNamespace(**skw)
    obj.afc = types.SimpleNamespace(
        lanes={"lane3": lane3},
        function=types.SimpleNamespace(is_printing=lambda: printing))
    return obj, a_conn


class TestMaybeRetractSister:
    def test_none_sibling_blocked(self):
        obj, _ = _retract_obj()
        assert obj._maybe_retract_sister(None) == "blocked"

    def test_already_retracted_returns_retracted(self):
        obj, _ = _retract_obj()
        obj._sister_retracted = (3, 75.0, 80.0)
        assert obj._maybe_retract_sister(3) == "retracted"

    def test_feature_off_blocked(self):
        obj, _ = _retract_obj(opts={"auto_tag_adjust": False})
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_no_lane_at_slot_blocked(self):
        obj, _ = _retract_obj()
        assert obj._maybe_retract_sister(9) == "blocked"

    def test_tool_loaded_blocked(self):
        obj, _ = _retract_obj(sibling={"tool_loaded": True})
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_not_hub_staged_blocked(self):
        obj, _ = _retract_obj(sibling={"loaded_to_hub": False})
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_printing_blocked(self):
        obj, _ = _retract_obj(printing=True)
        assert obj._maybe_retract_sister(3) == "blocked"

    def test_success_retracts_and_logs(self):
        obj, conn = _retract_obj(opts={"auto_tag_adjust_dist": 75.0})
        assert obj._maybe_retract_sister(3) == "retracted"
        assert obj._sister_retracted == (3, 75.0, 80.0)
        assert ("unwind_filament", {"index": 3, "length": 75.0, "speed": 80.0}) \
            in conn.calls
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: retracted sister slot 3 by 75mm to clear "
                     "its tag off the shared antenna")]

    def test_unwind_error_blocked_and_logged(self):
        obj, _ = _retract_obj(conn=_MotionConn(unwind_raises=True))
        assert obj._maybe_retract_sister(3) == "blocked"
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: sister retract on slot 3 failed")]


class TestRestoreSister:
    def test_noop_without_retract(self):
        obj, conn = _retract_obj()
        obj._sister_retracted = None
        obj._restore_sister()
        assert obj._sister_retracted is None
        assert obj.logger.messages == []

    def test_refeeds_and_logs(self):
        obj, conn = _retract_obj()
        obj._sister_retracted = (3, 75.0, 80.0)
        obj._restore_sister()
        assert obj._sister_retracted is None
        assert ("feed_filament", {"index": 3, "length": 75.0, "speed": 80.0}) \
            in conn.calls
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: restored sister slot 3 (+75mm) after "
                     "stage read")]


class TestIsPrinting:
    def test_true_when_printing(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.afc = types.SimpleNamespace(
            function=types.SimpleNamespace(is_printing=lambda: True))
        assert obj._is_printing() is True

    def test_false_on_exception(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.afc = None
        assert obj._is_printing() is False


class TestReaderPowerOff:
    def test_exception_logged(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        link = types.SimpleNamespace(
            reader_power=lambda on: (_ for _ in ()).throw(RuntimeError("x")))
        obj._reader_power_off(link)
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: stage read reader power-off failed")]


class TestSafeProbeTeardown:
    def test_noop_without_probe(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj._probe = None
        obj._safe_probe_teardown()
        assert obj.logger.messages == []

    def test_power_off_and_restore_identify(self):
        conn = _MotionConn()
        ace2 = types.SimpleNamespace(_ace=conn)
        obj = _mk(ace2, {"probe_settle": 0.0})
        obj.probe_restore_identify = True
        link = rfid_rfid_coverage._Ace2RegLink(ace2, 0, power_index=0)
        obj._probe = {"link": link, "shared": (0, 1)}
        obj._safe_probe_teardown()
        assert obj._probe is None
        assert obj.logger.messages == [
            ("info", "ACE2 RFID teardown: reader_power(off)..."),
            ("info", "ACE2 RFID teardown: reader_power(off) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(0,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(0,on) done"),
            ("info", "ACE2 RFID teardown: set_rfid_enable(1,on)..."),
            ("info", "ACE2 RFID teardown: set_rfid_enable(1,on) done")]

    def test_power_off_error_and_identify_not_restored(self):
        conn = _MotionConn()
        conn.send_command = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("wedge"))
        ace2 = types.SimpleNamespace(_ace=conn)
        obj = _mk(ace2, {"probe_settle": 0.0})
        obj.probe_restore_identify = False
        link = rfid_rfid_coverage._Ace2RegLink(ace2, 0, power_index=0)
        obj._probe = {"link": link, "shared": (0, 1)}
        obj._safe_probe_teardown()
        assert obj.logger.messages == [
            ("info", "ACE2 RFID teardown: reader_power(off)..."),
            ("exception", "ACE2 RFID: probe power-off failed"),
            ("info", "ACE2 RFID teardown: identify NOT restored (config)")]


# ── read_slot finally-path branches ──────────────────────────────────────────
class _SelConn:
    connected = True

    def __init__(self, raise_power_off=False, raise_enable_true=False):
        self.calls = []
        self._rpo = raise_power_off
        self._ret = raise_enable_true

    def send_command(self, name, params, timeout=None):
        p = dict(params)
        self.calls.append((name, p))
        if (self._rpo and name == "mfrc522_reader_power"
                and not (p["arg"] & 1)):
            raise RuntimeError("power off wedged")
        return {}

    def send_command_async(self, name, params=None):
        p = dict(params or {})
        self.calls.append((name, p))
        if self._ret and name == "set_rfid_enable" and p.get("enable"):
            raise RuntimeError("enable wedged")


def _slot_obj(conn, opts=None):
    ace2 = types.SimpleNamespace(_ace=conn, slot_count=4,
                                 _slot_map={"laneA": 0, "laneB": 1})
    base = {"lane_slot_map": "laneA:0, laneB:1"}
    base.update(opts or {})
    obj = _mk(ace2, base)
    obj.afc = types.SimpleNamespace(
        lanes={"laneA": types.SimpleNamespace(name="laneA", prep_state=True),
               "laneB": types.SimpleNamespace(name="laneB", prep_state=True)})
    return obj


class TestReadSlot:
    def test_missing_ace2_raises(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.ace2 = None
        with pytest.raises(_CmdError_rfid_coverage):
            obj.read_slot(0)

    def test_sibling_match_warns(self, monkeypatch):
        conn = _SelConn()
        obj = _slot_obj(conn)
        obj._slot_uid[1] = "beef"                 # sibling slot 1 already read beef
        monkeypatch.setattr(
            rfid_rfid_coverage, "read_tag",
            lambda link, **kw: {"uid": "beef", "tag_type": "MifareClassic1k",
                                "filament": {"type": "PLA"}})
        tag = obj.read_slot(0)
        assert tag["uid"] == "beef"
        assert obj._slot_uid[0] == "beef"
        assert obj.logger.messages == [
            ("warning", "ACE2 RFID: slot 0 read matches shared sibling slot 1's "
                        "tag (uid=beef) — check the spool is in slot 0, not 1")]

    def test_power_off_exception_logged(self, monkeypatch):
        conn = _SelConn(raise_power_off=True)
        obj = _slot_obj(conn)
        monkeypatch.setattr(
            rfid_rfid_coverage, "read_tag",
            lambda link, **kw: {"uid": "aa", "tag_type": "MifareClassic1k",
                                "filament": None})
        obj.read_slot(0)
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: reader power-off failed")]

    def test_reenable_exception_logged(self, monkeypatch):
        conn = _SelConn(raise_enable_true=True)
        obj = _slot_obj(conn, opts={"restore_identify": True,
                                    "skip_factory_autostage": False})
        monkeypatch.setattr(
            rfid_rfid_coverage, "read_tag",
            lambda link, **kw: {"uid": "aa", "tag_type": "MifareClassic1k",
                                "filament": None})
        obj.read_slot(0)
        assert obj.logger.messages == [
            ("exception", "ACE2 RFID: re-enable identify failed"),
            ("exception", "ACE2 RFID: re-enable identify failed")]


class TestReadLane:
    def test_missing_slot_raises(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.afc = None
        with pytest.raises(_CmdError_rfid_coverage):
            obj.read_lane("ghost")

    def test_no_tag_logs(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 0})
        obj = _mk(ace2, {"lane_slot_map": "laneA:0"})
        obj.afc = None
        obj.read_slot = lambda slot: None
        assert obj.read_lane("laneA") is None
        assert obj.logger.messages == [
            ("info", "ACE2 RFID: no tag read on slot 0")]

    def test_applies_to_lane(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 0})
        obj = _mk(ace2, {"lane_slot_map": "laneA:0"})
        lane = types.SimpleNamespace(name="laneA")
        obj.afc = types.SimpleNamespace(lanes={"laneA": lane})
        tag = {"uid": "aa", "filament": {"type": "PLA"}}
        obj.read_slot = lambda slot: tag
        applied = []
        obj.apply_to_lane = lambda ln, tg: applied.append((ln, tg))
        assert obj.read_lane("laneA") is tag
        assert applied == [(lane, tag)]


class TestLaneAtSlot:
    def test_none_when_no_afc(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.afc = None
        assert obj._lane_at_slot(0) is None

    def test_finds_lane_via_ace_map(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 2})
        obj = _mk(ace2, {})
        lane = types.SimpleNamespace(name="laneA")
        obj.afc = types.SimpleNamespace(lanes={"laneA": lane})
        assert obj._lane_at_slot(2) is lane

    def test_none_when_slot_unmapped(self):
        ace2 = types.SimpleNamespace(_slot_map={"laneA": 2})
        obj = _mk(ace2, {})
        obj.afc = types.SimpleNamespace(
            lanes={"laneA": types.SimpleNamespace(name="laneA")})
        assert obj._lane_at_slot(7) is None


class TestHandleSisterDomination:
    def test_blocked_shows_hint_once(self):
        obj, _ = _retract_obj(opts={"auto_tag_adjust": False})
        obj._handle_sister_domination(3)
        assert obj._sister_hint_shown is True
        assert len(obj.gcode.info) == 1
        assert "lane lane3" in obj.gcode.info[0]
        obj._handle_sister_domination(3)              # already shown -> no repeat
        assert len(obj.gcode.info) == 1

    def test_retracted_shows_no_hint(self):
        obj, _ = _retract_obj()
        obj._handle_sister_domination(3)              # movable -> auto retract
        assert obj._sister_hint_shown is False
        assert obj.gcode.info == []


class TestScanLane:
    def test_missing_ace2_raises(self):
        obj = _mk(_Ace2_rfid_coverage(), {})
        obj.ace2 = None
        with pytest.raises(_CmdError_rfid_coverage):
            obj.scan_lane("scan1")

    def test_no_tag_logs_and_returns_none(self):
        ace2 = types.SimpleNamespace(_slot_map={"scan1": 0})
        obj = _mk(ace2, {"lane_slot_map": "scan1:0", "scanner_lanes": "scan1",
                        "scan_seconds": 30.0})
        obj._scan_slot = lambda slot, dur, lane_name="": None
        assert obj.scan_lane("scan1") is None
        assert obj.logger.messages == [
            ("info", "ACE2 RFID scan: no tag on lane scan1 in 30s")]

    def test_spoolman_sync_exception_logged(self, monkeypatch):
        ace2 = types.SimpleNamespace(_slot_map={"scan1": 0})
        obj = _mk(ace2, {"lane_slot_map": "scan1:0", "scanner_lanes": "scan1"})
        obj.reactor = _Reactor_rfid_coverage()
        tag = {"uid": "abcd", "tag_type": "MifareClassic1k",
               "filament": {"type": "PLA"}}
        obj._scan_slot = lambda slot, dur, lane_name="": tag
        lane = types.SimpleNamespace(name="scan1")
        obj.afc = types.SimpleNamespace(
            lanes={"scan1": lane}, spoolman=object(), moonraker=None,
            spool=types.SimpleNamespace(next_spool_id=1))
        monkeypatch.setattr(
            rfid_rfid_coverage, "get_auto_spoolman_create",
            lambda ln, default: (_ for _ in ()).throw(RuntimeError("x")))
        monkeypatch.setattr(
            rfid_rfid_coverage, "sync_rfid_to_spoolman",
            lambda *a, **k: (_ for _ in ()).throw(RuntimeError("sync boom")))
        out = obj.scan_lane("scan1", 5)
        assert out is tag
        assert obj.logger.messages == [
            ("warning", "ACE2 RFID scan Spoolman sync failed: sync boom")]


# ── load_config ──────────────────────────────────────────────────────────────
class TestLoadConfig:
    def test_returns_instance(self):
        cfg = _Config_rfid_coverage(_Printer_rfid_coverage(), {})
        obj = rfid_rfid_coverage.load_config(cfg)
        assert isinstance(obj, rfid_rfid_coverage.AFC_ACE2_RFID)


# ── Driver-level verification of the shared RFID reader stack in ──────────────
#
# was tests/test_ace2_rfid_reader.py
HERE_ace2_rfid_reader = os.path.dirname(os.path.abspath(__file__))
ROOT_ace2_rfid_reader = os.path.dirname(HERE_ace2_rfid_reader)


def _load_ace2_rfid_reader(name, relpath):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT_ace2_rfid_reader, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


A = _load_ace2_rfid_reader("AFC_rfid_readers", "extras/AFC_rfid_readers.py")
ACE2 = _load_ace2_rfid_reader("AFC_ACE2_rfid", "extras/AFC_ACE2_rfid.py")


# ── build synthetic tags with known fields ──────────────────────────────────
def make_bambu_tag():
    d = bytearray(1024)
    d[32:32 + 3] = b"PLA"                       # block2 type
    d[64:64 + 9] = b"PLA Basic"                 # block4 detailed
    d[80:84] = bytes([0x00, 0x86, 0xD6, 0xFF])  # block5 color RGBA -> 0086D6
    struct.pack_into("<H", d, 84, 1000)         # weight g
    struct.pack_into("<f", d, 88, 1.75)         # diameter mm
    struct.pack_into("<H", d, 96, 55)           # drying temp
    struct.pack_into("<H", d, 98, 8)            # drying time
    struct.pack_into("<H", d, 102, 55)          # bed temp
    struct.pack_into("<H", d, 104, 230)         # hotend max
    struct.pack_into("<H", d, 106, 190)         # hotend min
    d[192:192 + 16] = b"2026_03_20_11_48\0"[:16]  # production
    return bytes(d)

MASTER = bytes.fromhex("9A759CF2C4F7CAFF222CB9769B41BC96")
UID = bytes([0x04, 0xA1, 0xB2, 0xC3])


def make_anycubic_tag():
    d = bytearray(0x80)
    d[0x10:0x14] = A.ANYCUBIC_MAGIC
    d[0x14:0x14 + 9] = b"HPL19-102"                 # SKU
    d[0x28:0x28 + 8] = b"Anycubic"                  # brand
    d[0x3C:0x3C + 3] = b"PLA"                        # type
    d[0x50:0x54] = bytes([0xFF, 0xD6, 0x86, 0x00])  # a,b,g,r -> argb FF0086D6
    struct.pack_into("<H", d, 0x60, 190)            # hotend min
    struct.pack_into("<H", d, 0x62, 230)            # hotend max
    struct.pack_into("<H", d, 0x76, 60)             # bed
    struct.pack_into("<H", d, 0x78, 175)            # diameter*100
    struct.pack_into("<H", d, 0x7A, 330)            # length -> 1000g
    return bytes(d)


class FakeReader:
    """Register-level MFRC522 emulator holding one tag (Classic or Ultralight)."""
    def __init__(self, uid, tag, master, sak=0x08):
        self.uid = uid
        self.tag = tag
        self.sak = sak
        self.ntag = (sak == 0x00)
        self.keys = A.bambu_keys(uid, master) if master else None
        self.reg = {i: 0 for i in range(0x40)}
        self.txfifo = bytearray()
        self.rxfifo = bytearray()
        self.crypto_sector = None

    # firmware contract:
    def reg_read(self, r):
        if r == A.FIFOLevelReg:
            return len(self.rxfifo)
        if r == A.FIFODataReg:
            return self.rxfifo.pop(0) if self.rxfifo else 0
        if r == A.ControlReg:
            return 0                               # rx_last_bits = 0
        return self.reg.get(r, 0)

    def reg_write(self, r, v):
        v &= 0xFF
        if r == A.FIFOLevelReg and (v & 0x80):
            self.txfifo.clear(); return
        if r == A.FIFODataReg:
            self.txfifo.append(v); return
        if r == A.CommandReg and v in (A.PCD_TRANSCEIVE, A.PCD_MFAUTHENT):
            self._exec(v)
        self.reg[r] = v

    def _set_rx(self, data):
        self.rxfifo = bytearray(data)
        self.reg[A.ComIrqReg] = 0x20               # RxIRq
        self.reg[A.ErrorReg] = 0

    def _exec(self, cmd):
        buf = bytes(self.txfifo); self.txfifo.clear()
        self.reg[A.ComIrqReg] = 0x10               # IdleIRq default
        self.reg[A.ErrorReg] = 0
        if cmd == A.PCD_MFAUTHENT:
            key_type, block = buf[0], buf[1]
            key, uid = buf[2:8], buf[8:12]
            sector = block // 4
            if key_type == A.PICC_AUTH_KEY_A and list(key) == self.keys[sector] and uid == self.uid:
                self.crypto_sector = sector
                self.reg[A.Status2Reg] = 0x08       # MFCrypto1On
            else:
                self.reg[A.Status2Reg] = 0x00
            return
        # Transceive
        if len(buf) == 1 and buf[0] in (A.PICC_REQA, A.PICC_WUPA):
            self._set_rx(b"\x04\x00"); return       # ATQA
        if buf[:2] == bytes([A.PICC_ANTICOLL, 0x20]):
            bcc = self.uid[0] ^ self.uid[1] ^ self.uid[2] ^ self.uid[3]
            self._set_rx(self.uid + bytes([bcc])); return
        if buf[:2] == bytes([A.PICC_SELECT, 0x70]):
            self._set_rx(bytes([self.sak, 0, 0])); return   # SAK (+crc)
        if buf[0] == A.PICC_READ:
            arg = buf[1]
            if self.ntag:                            # NTAG: arg=page(4B), 4 pages back
                off = arg * 4
                self._set_rx(self.tag[off:off + 16] + b"\x00\x00"); return
            sector = arg // 4                        # Classic: arg=block(16B), needs auth
            if self.crypto_sector == sector:
                self._set_rx(self.tag[arg * 16:arg * 16 + 16] + b"\x00\x00"); return
        self.reg[A.ComIrqReg] = 0x01                # TimerIRq (no reply)


# ── tests ───────────────────────────────────────────────────────────────────
def test_key_matches_bambu_salt_hash():
    assert hashlib.sha256(MASTER).hexdigest() == A.BAMBU_SALT_HASH


def test_hkdf_rfc5869_vector():
    got = A.hkdf_sha256(bytes.fromhex("000102030405060708090a0b0c"),
                        bytes.fromhex("0b" * 22), bytes.fromhex("f0f1f2f3f4f5f6f7f8f9"), 42)
    assert got.hex() == ("3cb25f25faacd57a90434f64d0362f2a2d2d0a90cf1a5a4c5"
                         "db02d56ecc4c5bf34007208d5b887185865")


def test_frame_crc_matches_firmware_algo():
    link = ACE2.Ace2Link(lambda f: f)
    f = link.build_frame(0x06, b"")
    assert f[:2] == ACE2.PREAMBLE and f[-1:] == ACE2.END
    body = f[3:-3]
    assert struct.unpack("<H", f[-3:-1])[0] == ACE2.crc16_kermit(body)


def test_reader_power_frame_encodes_cmd_0x52():
    sent = {}
    link = ACE2.Ace2Link(lambda f: sent.setdefault("frame", f) or f, slot=1)
    link.reader_power(True)
    frame = sent["frame"]
    # FF AA seq TYPE(2) CMD LEN PAYLOAD... — CMD byte is at index 5
    assert frame[5] == ACE2.CMD_MFRC522_READER_POWER == 0x52
    # payload varint = (1<<16)|1
    assert frame[7] == 0x08 and ACE2._varint_decode(frame, 8)[0] == (1 << 16) | 1


def test_reg_rw_roundtrip_through_link():
    fake = FakeReader(UID, make_bambu_tag(), MASTER)
    fake.reg_write(A.TxControlReg, 0x03)
    assert fake.reg_read(A.TxControlReg) == 0x03


def test_full_read_and_decode_end_to_end():
    fake = FakeReader(UID, make_bambu_tag(), MASTER)
    out = A.read_bambu(fake, MASTER)               # REAL driver, emulated reader
    assert out is not None, "read failed"
    assert out["uid"] == UID.hex()
    assert out["type"] == "PLA"
    assert out["detailed"] == "PLA Basic"
    assert out["color_argb"] == 0xFF0086D6
    assert out["weight_g"] == 1000
    assert out["diameter_mm"] == 1.75
    assert out["hotend_min_c"] == 190 and out["hotend_max_c"] == 230
    assert out["bed_temp_c"] == 55
    assert out["production"] == "2026_03_20_11_48"


def test_wrong_master_key_fails_auth():
    fake = FakeReader(UID, make_bambu_tag(), MASTER)
    out = A.read_bambu(fake, bytes.fromhex("00" * 16))  # wrong key -> auth fails
    assert out is None


def test_anycubic_ntag_end_to_end():
    fake = FakeReader(UID, make_anycubic_tag(), master=None, sak=0x00)
    out = A.read_tag(fake)                          # no key needed for NTAG
    assert out is not None
    assert out["uid"] == UID.hex()
    assert out["tag_type"] == "MifareUltralight"
    f = out["filament"]
    assert f is not None and f["manufacturer"] == "Anycubic"
    assert f["sku"] == "HPL19-102" and f["type"] == "PLA"
    assert f["color_argb"] == 0xFF0086D6
    assert f["diameter_mm"] == 1.75 and f["weight_g"] == 1000
    assert f["hotend_min_c"] == 190 and f["hotend_max_c"] == 230


def test_read_tag_classic_bambu_with_key():
    fake = FakeReader(UID, make_bambu_tag(), MASTER, sak=0x08)
    out = A.read_tag(fake, MASTER)
    assert out["tag_type"] == "MifareClassic1k"
    assert out["filament"]["type"] == "PLA" and out["filament"]["weight_g"] == 1000


def test_read_tag_returns_uid_even_without_key():
    # Classic (Bambu) tag but no key supplied: still get UID + tag type, no decode
    fake = FakeReader(UID, make_bambu_tag(), MASTER, sak=0x08)
    out = A.read_tag(fake, bambu_master_key=None)
    assert out["uid"] == UID.hex()
    assert out["tag_type"] == "MifareClassic1k"
    assert out["filament"] is None


# ── shared-reader exclusion: HALT the neighbour tag, read this lane's own ────
class _FakeMfrc:
    """Mfrc522 stand-in modelling a shared antenna with several tags. Supports
    WUPA/REQA/anticoll/select/halt so activate()'s exclusion loop can be driven
    without register-level plumbing. A halted tag ignores REQA until a WUPA."""

    def __init__(self, tags):
        self.tags = list(tags)                 # [(uid_bytes, sak), ...], field order
        self.halted = set()
        self.current = None
        self.resets = 0

    def reset(self):
        self.resets += 1

    def stop_crypto(self):
        self.crypto_stops = getattr(self, "crypto_stops", 0) + 1

    def antenna_on(self):
        pass

    def _live(self):
        return [t for t in self.tags if t[0] not in self.halted]

    def request(self, req):
        if req == A.PICC_WUPA:
            self.halted.clear()                # WUPA wakes halted tags
        return b"\x04\x00" if self._live() else None

    def anticoll(self):
        live = self._live()
        if not live:
            return None
        self.current = live[0][0]
        return self.current

    def select(self, uid):
        for u, s in self.tags:
            if u == uid:
                return s
        return None

    def halt(self):
        if self.current is not None:
            self.halted.add(self.current)


NEIGH = b"\xaa\xaa\xaa\xaa"
OWN = b"\xbb\xbb\xbb\xbb"


def test_activate_halts_excluded_neighbour_and_returns_own():
    fake = _FakeMfrc([(NEIGH, 0x08), (OWN, 0x08)])
    mc = A.MifareClassic(fake)
    uid, sak = mc.activate(is_excluded=lambda u: u == NEIGH.hex())
    assert uid == OWN and sak == 0x08          # neighbour skipped, own tag read
    assert NEIGH in fake.halted                # silenced via HLTA


def test_activate_returns_none_when_only_excluded_present():
    fake = _FakeMfrc([(NEIGH, 0x08)])
    mc = A.MifareClassic(fake)
    uid, sak = mc.activate(is_excluded=lambda u: u == NEIGH.hex())
    assert uid is None and sak is None         # neighbour-only -> nothing to read
    assert NEIGH in fake.halted


def test_activate_without_excluder_returns_first_tag():
    fake = _FakeMfrc([(NEIGH, 0x08), (OWN, 0x08)])
    mc = A.MifareClassic(fake)
    uid, sak = mc.activate()
    assert uid == NEIGH and sak == 0x08        # no exclusion: first tag wins


def test_activate_seen_records_halted_neighbour_and_own():
    fake = _FakeMfrc([(NEIGH, 0x08), (OWN, 0x08)])
    mc = A.MifareClassic(fake)
    seen = []
    mc.activate(is_excluded=lambda u: u == NEIGH.hex(), seen=seen)
    # both tags are recorded, the neighbour flagged excluded, own tag not
    assert (NEIGH.hex(), 0x08, True) in seen
    assert (OWN.hex(), 0x08, False) in seen


def test_activate_seen_empty_when_field_empty():
    fake = _FakeMfrc([])                        # nothing in the field
    mc = A.MifareClassic(fake)
    seen = []
    uid, _ = mc.activate(is_excluded=lambda u: False, seen=seen)
    assert uid is None and seen == []           # empty field -> nothing seen


class _CaptureLink:
    """reg link that lets _to_card complete and records FIFO writes."""

    def __init__(self):
        self.fifo = []

    def reg_read(self, r):
        if r == A.ComIrqReg:
            return 0x30                        # Rx/Idle set -> poll exits at once
        return 0

    def reg_write(self, r, v):
        if r == A.FIFODataReg:
            self.fifo.append(v)


def test_halt_sends_hlta_frame():
    link = _CaptureLink()
    A.Mfrc522(link).halt()
    assert link.fifo[:2] == [0x50, 0x00]       # HLTA opcode
    assert len(link.fifo) == 4                  # opcode + 2 CRC bytes


# ── Snapmaker U1 MIFARE Classic decode + HKDF key derivation ────────────────
def _make_snapmaker_tag():
    d = bytearray(1024)
    d[16:16 + 9] = b"Snapmaker"                 # vendor
    d[66] = 1                                    # MAIN_TYPE PLA (LE u16)
    d[68] = 2                                    # SUB_TYPE Matte
    d[73] = 0                                    # ALPHA byte -> 0xFF - 0 = 0xFF
    d[80], d[81], d[82] = 0x00, 0x00, 0xFF       # RGB_1 = blue
    d[96:100] = (12345).to_bytes(4, "little")    # SKU
    d[128:130] = (175).to_bytes(2, "little")     # diameter -> 1.75mm
    d[130:132] = (1000).to_bytes(2, "little")    # weight g
    d[148:150] = (220).to_bytes(2, "little")     # hotend max
    d[150:152] = (190).to_bytes(2, "little")     # hotend min
    d[154:156] = (60).to_bytes(2, "little")      # bed temp
    d[160:168] = b"20250101"                     # mfg date
    return bytes(d)


def test_decode_snapmaker_fields():
    f = A.decode_snapmaker(_make_snapmaker_tag())
    assert f["manufacturer"] == "Snapmaker"
    assert f["type"] == "PLA" and f["detailed"] == "Matte"
    assert f["color_argb"] == 0xFF0000FF
    assert f["weight_g"] == 1000 and f["diameter_mm"] == 1.75
    assert f["hotend_max_c"] == 220 and f["hotend_min_c"] == 190
    assert f["bed_temp_c"] == 60
    assert f["sku"] == "12345" and f["production"] == "20250101"


def test_decode_snapmaker_rejects_non_snapmaker():
    assert A.decode_snapmaker(bytes(1024)) is None      # main type 0 -> not ours


def test_snapmaker_key_derivation_matches_spec():
    import hmac
    import hashlib
    uid = bytes.fromhex("80dcf43e")
    keys = A.snapmaker_keys(uid)
    assert len(keys) == 16
    prk = hmac.new(A.SNAPMAKER_SALT_A, uid, hashlib.sha256).digest()
    expect0 = hmac.new(prk, b"key_a_0" + bytes([1]), hashlib.sha256).digest()[:6]
    expect3 = hmac.new(prk, b"key_a_3" + bytes([1]), hashlib.sha256).digest()[:6]
    assert bytes(keys[0]) == expect0
    assert bytes(keys[3]) == expect3


class _ReMifare:
    """Fake MifareClassic serving one Snapmaker tag. Bambu auth (its keys) fails,
    Snapmaker keys pass — so the tag is only decoded on the SECOND scheme, which
    means it exercises the re-select. Records the excluder passed to each
    activate() so the test can prove the re-select does not use the caller's
    (sibling) predicate."""

    def __init__(self, uid, image):
        self._uid = uid
        self._image = image
        self.activate_excluders = []
        self.activate_resets = []

    def activate(self, is_excluded=None, seen=None, reset=True):
        self.activate_excluders.append(is_excluded)
        self.activate_resets.append(reset)
        # Model the exclusion loop: a tag the predicate excludes is halted and
        # nothing readable remains.
        if is_excluded is not None and is_excluded(self._uid.hex()):
            if seen is not None:
                seen.append((self._uid.hex(), 0x08, True))
            return None, None
        if seen is not None:
            seen.append((self._uid.hex(), 0x08, False))
        return self._uid, 0x08                        # Classic 1K

    def read_blocks(self, uid, keys_a, blocks):
        return self._image if keys_a == A.snapmaker_keys(uid) else None

    def read_ntag(self, n=128):
        return None


def test_reselect_pins_to_active_uid_not_sibling_predicate(monkeypatch):
    # Regression: a Snapmaker tag is only reached on the 2nd scheme (Bambu fails
    # first). The re-select before it must pin to the tag just activated, NOT
    # re-run the caller's sibling predicate — which can resolve the active tag's
    # own UID to the neighbour and halt it, leaving the lane with defaults.
    uid = b"\xbb\xbb\xbb\xbb"
    fake = _ReMifare(uid, _make_snapmaker_tag())
    monkeypatch.setattr(A, "Mfrc522", lambda link: None)
    monkeypatch.setattr(A, "MifareClassic", lambda mfrc: fake)

    # A sibling predicate that only misfires on the RE-select (the first
    # activate saw a clean field; sibling state changed by the time we re-select
    # for the 2nd scheme). With the bug (re-select uses this predicate) the
    # active tag is halted on re-select and Snapmaker never decodes.
    calls = {"n": 0}

    def sibling_pred(h):
        calls["n"] += 1
        return h == uid.hex() and calls["n"] > 1

    out = A.read_tag(object(), bambu_master_key=MASTER, is_excluded=sibling_pred)

    # Snapmaker still decodes: the re-select pinned to the active UID
    assert out["filament"] is not None
    assert out["filament"]["manufacturer"] == "Snapmaker"
    # first activate used the caller's predicate; the re-select did NOT
    assert fake.activate_excluders[0] is sibling_pred
    reselect = fake.activate_excluders[1]
    assert reselect(uid.hex()) is False              # active tag is kept
    assert reselect("cccccccc") is True              # a real neighbour still halts
    # first activate does a full reset (cold bring-up); the re-select keeps the
    # RF field UP (reset=False) so a marginal at-rest tag isn't unpowered
    assert fake.activate_resets[0] is True
    assert fake.activate_resets[1] is False


def test_activate_reset_false_keeps_field_no_soft_reset():
    fake = _FakeMfrc([(OWN, 0x08)])
    mc = A.MifareClassic(fake)
    mc.activate(reset=True)                           # cold bring-up
    assert fake.resets == 1
    mc.activate(reset=False)                          # re-select, field stays up
    assert fake.resets == 1                           # no additional SoftReset
    assert getattr(fake, "crypto_stops", 0) == 1      # crypto ended instead


# ── AES-128 + Creality CFS + Elegoo decode ──────────────────────────────────
def test_aes128_fips197_vector():
    key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
    pt = bytes.fromhex("00112233445566778899aabbccddeeff")
    ct = bytes.fromhex("69c4e0d86a7b0430d8cdb78070b4c55a")
    assert A._aes_encrypt_block(pt, key) == ct
    assert A._aes_decrypt_block(ct, key) == pt


def test_aes_cbc_decrypt_roundtrip():
    key = bytes.fromhex("484043466b526e7a404b4174424a7032")
    data = bytes(range(48))

    def cbc_enc(d, k, iv=b"\x00" * 16):
        out, prev = bytearray(), iv
        for i in range(0, len(d), 16):
            b = bytes(x ^ y for x, y in zip(d[i:i + 16], prev))
            e = A._aes_encrypt_block(b, k)
            out += e
            prev = e
        return bytes(out)

    assert A._aes_cbc_decrypt(cbc_enc(data, key), key) == data


def test_creality_key_derivation_matches_reference():
    u_key = bytes.fromhex("713362755e74316e71665a2870662431")
    keyA = A.creality_mifare_key(bytes.fromhex("60EA1221"), u_key)
    assert keyA.hex() == "1f1e83a97182"        # exact value from TAG_FORMAT.md


def test_decode_creality_fields():
    payload = (b"ABC21" + b"0276" + b"01" + b"101001" + b"0FF5F0B"
               + b"0165" + b"736314" + b"\x00" * 14)
    assert len(payload) == 48
    f = A.decode_creality(payload)
    assert f["manufacturer"] == "Creality"
    assert f["type"] == "PLA" and f["sku"] == "101001"
    assert f["color_argb"] == 0xFFFF5F0B
    assert f["weight_g"] == 500                 # 0x0165 = 357 -> 500g spool
    assert f["serial"] == "736314"


def test_decode_creality_rejects_garbage():
    assert A.decode_creality(b"\x00" * 48) is None


def test_decode_creality_end_to_end_encrypted():
    # full path: build plaintext, AES-CBC encrypt with d_key, decrypt+decode
    d_key = bytes.fromhex("484043466b526e7a404b4174424a7032")
    payload = (b"ABC21" + b"0276" + b"01" + b"101002" + b"000FF00"
               + b"0330" + b"000123" + b"\x00" * 14)

    def cbc_enc(dat, k, iv=b"\x00" * 16):
        out, prev = bytearray(), iv
        for i in range(0, len(dat), 16):
            b = bytes(x ^ y for x, y in zip(dat[i:i + 16], prev))
            e = A._aes_encrypt_block(b, k)
            out += e
            prev = e
        return bytes(out)

    f = A.decode_creality(A._aes_cbc_decrypt(cbc_enc(payload, d_key), d_key))
    assert f["type"] == "PETG"                  # 101002
    assert f["color_argb"] == 0xFF00FF00        # 000FF00 -> #00FF00
    assert f["weight_g"] == 1000                 # 0x0330 = 816 -> 1kg


def test_decode_elegoo_fields():
    d = bytearray(64)
    d[16] = 0x36
    d[17:21] = b"\xee\xee\xee\xee"
    d[21:23] = b"\x00\x01"
    d[23:27] = b"PLA "                           # 0x504C4120
    d[27:31] = b"CF20"
    d[31:34] = bytes([0xFF, 0x37, 0x00])
    d[34:36] = (175).to_bytes(2, "big")
    d[36:38] = (1000).to_bytes(2, "big")
    d[38:40] = (2502).to_bytes(2, "big")
    f = A.decode_elegoo(bytes(d))
    assert f["manufacturer"] == "Elegoo"
    assert f["type"] == "PLA" and f["detailed"] == "CF20"
    assert f["color_argb"] == 0xFFFF3700
    assert f["diameter_mm"] == 1.75 and f["weight_g"] == 1000
    assert f["production"] == "2502"


def test_decode_elegoo_rejects_non_elegoo():
    assert A.decode_elegoo(bytes(64)) is None    # header != 0x36

