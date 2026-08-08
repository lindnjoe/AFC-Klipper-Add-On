"""
Unit tests for the ACE2 RFID host wiring (extras/AFC_ACE2_rfid.py):
  - AFC_ACE2 command builder for the reader-power passthrough (cmd 0x52)
  - _Ace2RegLink encodings
  - read_slot: physical-slot -> reader-index mapping, and the reader-ownership
    sequence (disable identify on BOTH shared slots -> power on -> read ->
    power off -> restore), with best-effort restore on failure
  - cmd_ACE_RFID_READ never shuts Klipper down on a read error
  - auto-read on the afc_ace:post_insert event
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


import extras.AFC_rfid_readers as readers  # noqa: E402

rfid = _load("AFC_ACE2_rfid", "extras/AFC_ACE2_rfid.py")
ace2 = _load("AFC_ACE2", "extras/AFC_ACE2.py")


class _Conn:
    connected = True

    def __init__(self):
        self.calls = []

    def send_command(self, name, params, timeout=None):
        self.calls.append((name, dict(params)))
        return {}

    def send_command_async(self, name, params=None):
        # set_rfid_enable is fire-and-forget; record in wire order alongside sync.
        self.calls.append((name, dict(params or {})))


class _Ace2:
    def __init__(self):
        self._ace = _Conn()


class _Reactor:
    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        return waketime

    def register_callback(self, cb, waketime=None):
        cb(0.0)                                  # run inline for tests


class _AdvReactor:
    """Reactor whose monotonic advances on each pause, so a scan's deadline
    actually fires (the plain _Reactor freezes monotonic at 0)."""

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


class _CmdError(Exception):
    pass


class _Gcode:
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


class _Printer:
    command_error = _CmdError

    def __init__(self):
        self.reactor = _Reactor()
        self.gcode = _Gcode()
        self.events = []

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        if name == "gcode":
            return self.gcode
        return default

    def register_event_handler(self, name, cb):
        self.events.append((name, cb))


class _Config:
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


class _GCmd:
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
    obj = rfid.AFC_ACE2_RFID(_Config(_Printer(), dict(opts or {})))
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
    monkeypatch.setattr(rfid, "read_tag", fake)


# ── AFC_ACE2 command builder ────────────────────────────────────────────────

def test_builder_maps_reader_power_to_0x52():
    cmd, payload = ace2.method_to_v2("mfrc522_reader_power", {"arg": 0x10001})
    assert cmd == 0x52 == ace2.Cmd.MFRC522_READER_POWER
    assert payload == ace2.pb_uint32(1, 0x10001)


# ── _Ace2RegLink power/enable encodings ─────────────────────────────────────

def test_reader_power_arg_encoding():
    a = _Ace2()
    link = rfid._Ace2RegLink(a, 1)          # reader index 1
    link.reader_power(True)
    link.reader_power(False)
    assert a._ace.calls == [
        ("mfrc522_reader_power", {"arg": (1 << 16) | 1}),
        ("mfrc522_reader_power", {"arg": (1 << 16) | 0}),
    ]


def test_set_rfid_enable_uses_physical_slot():
    a = _Ace2()
    link = rfid._Ace2RegLink(a, 0)
    link.set_rfid_enable(2, False)
    assert a._ace.calls == [("set_rfid_enable", {"index": 2, "enable": False})]


# ── read_slot: mapping + power sequence ─────────────────────────────────────

def test_read_slot_power_sequence_order(monkeypatch):
    a = _Ace2()
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
    a = _Ace2()
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
    a = _Ace2()
    recorded = []
    obj = _bare_rfid(a)
    obj.reader_reg_per_slot = True           # opt in to per-slot chip-select
    _patch_read_tag(monkeypatch, recorded)
    obj.read_slot(3)
    assert ("read_tag", 3) in recorded        # reg index == physical slot
    assert a._ace.calls[2][1] == {"arg": (1 << 16) | 1}   # power still per-pair


def test_read_slot_reg_override_sets_chip_and_power(monkeypatch):
    a = _Ace2()
    recorded = []
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, recorded)
    obj.read_slot(2, reg_slot=1)              # probe reader index 1 explicitly
    assert ("read_tag", 1) in recorded        # reg index forced to 1
    assert a._ace.calls[2][1] == {"arg": (1 << 16) | 1}   # power = same reader (1)


def test_read_slot_disables_identify_but_skips_restore_when_autostage_skipped(monkeypatch):
    a = _Ace2()
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
    a = _Ace2()
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
    a = _Ace2()
    recorded = []
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, recorded)
    obj.read_slot(0, manage_power=False)
    assert a._ace.calls == []
    assert ("read_tag", 0) in recorded


# ── cmd_ACE_RFID_READ never crashes Klipper ─────────────────────────────────

def test_cmd_read_error_does_not_propagate(monkeypatch):
    a = _Ace2()
    obj = _bare_rfid(a)
    # A generic (non-command_error) exception from the read must be swallowed.
    _patch_read_tag(monkeypatch, [], exc=RuntimeError("serial timed out"))
    gcmd = _GCmd({"SLOT": 0})
    obj.cmd_ACE_RFID_READ(gcmd)                  # must NOT raise
    assert any("read error" in r for r in gcmd.responses)


def test_cmd_read_reports_no_tag(monkeypatch):
    a = _Ace2()
    obj = _bare_rfid(a)
    _patch_read_tag(monkeypatch, [], result=None)
    gcmd = _GCmd({"SLOT": 0})
    obj.cmd_ACE_RFID_READ(gcmd)
    assert any("no tag found" in r for r in gcmd.responses)


# ── scanner lane (ACE_RFID_SCAN -> next spool id) ────────────────────────────

def test_scanner_lanes_config_parsed():
    obj = _make_rfid(_Ace2(), {"scanner_lanes": "lane1, lane2",
                               "scan_seconds": 45, "scan_interval": 0.5})
    assert obj._scanner_lanes == {"lane1", "lane2"}
    assert obj.scan_seconds == 45.0
    assert obj.scan_interval == 0.5


def test_scan_seconds_defaults_to_30():
    obj = _make_rfid(_Ace2(), {})
    assert obj.scan_seconds == 30.0


def test_scan_lane_stages_next_spool_id(monkeypatch):
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    tag = {"uid": "abcd", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA"}}
    monkeypatch.setattr(obj, "_scan_slot", lambda slot, dur, lane_name="": tag)
    calls = []

    def _sync(afc, lane, slot_info, logger, prefix, **kw):
        calls.append((prefix, kw))

    monkeypatch.setattr(rfid, "sync_rfid_to_spoolman", _sync)
    lane = types.SimpleNamespace(name="scan_lane")
    obj.afc = types.SimpleNamespace(lanes={"scan_lane": lane}, spoolman=object())

    out = obj.scan_lane("scan_lane", 5)

    assert out is tag
    assert len(calls) == 1
    prefix, kw = calls[0]
    assert kw.get("set_next") is True            # staged, not applied to lane
    assert prefix == "ACE2 RFID scan"


def test_scan_emits_popup_notification(monkeypatch):
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    tag = {"uid": "abcd", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA"}}
    monkeypatch.setattr(obj, "_scan_slot", lambda slot, dur, lane_name="": tag)
    monkeypatch.setattr(rfid, "sync_rfid_to_spoolman", lambda *a, **k: None)
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
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    monkeypatch.setattr(obj, "_scan_slot", lambda slot, dur, lane_name="": None)
    called = []
    monkeypatch.setattr(rfid, "sync_rfid_to_spoolman",
                        lambda *a, **k: called.append(1))
    obj.afc = types.SimpleNamespace(lanes={}, spoolman=object())

    assert obj.scan_lane("scan_lane", 5) is None
    assert called == []                           # nothing staged


def test_scan_slot_reads_until_tag_then_powers_off(monkeypatch):
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 1})
    # two misses (None), then a FULL decode on the 3rd attempt
    seq = [None, None, {"uid": "beef", "filament": {"type": "PLA"}}]
    monkeypatch.setattr(rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=5.0)

    assert tag == {"uid": "beef", "filament": {"type": "PLA"}}
    powers = [c[1]["arg"] for c in a._ace.calls if c[0] == "mfrc522_reader_power"]
    assert 1 in powers                            # reader powered on to read
    assert powers[-1] == 0                         # and powered off when done


def test_scan_prefers_full_decode_over_uid_only(monkeypatch):
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 1})
    # a UID-only read first, then the full decode: must wait for the full one
    seq = [{"uid": "beef", "filament": None},
           {"uid": "beef", "filament": {"type": "PLA", "manufacturer": "Snapmaker"}}]
    monkeypatch.setattr(rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=5.0)

    assert tag["filament"]["manufacturer"] == "Snapmaker"   # not the UID-only read


def test_scan_confirms_same_uid_over_consecutive_reads(monkeypatch):
    # A stray one-off decode must NOT be accepted: with confirm=2 the scan only
    # returns once the SAME UID decodes twice in a row; a different UID in
    # between resets the streak.
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 2})
    good = {"uid": "beef", "filament": {"type": "PLA"}}
    stray = {"uid": "cafe", "filament": {"type": "ABS"}}
    seq = [stray, good, stray, good, good]        # good confirmed on the last two
    monkeypatch.setattr(rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=10.0)

    assert tag is good                            # the twice-in-a-row UID wins
    assert obj._slot_uid[0] == "beef"


def test_scan_returns_none_without_full_decode(monkeypatch):
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0})
    obj.reactor = _AdvReactor(step=0.5)           # advance so the deadline fires
    # only ever a UID (decode never completes) -> a UID-only read (possibly a
    # corrupted UID) is NOT trusted; the scan reports no read.
    monkeypatch.setattr(rfid, "read_tag",
                        lambda link, **kw: {"uid": "beef", "filament": None})

    tag = obj._scan_slot(0, duration=2.0)

    assert tag is None                            # no full decode -> no match
    assert 0 not in obj._slot_uid                  # and the UID was not cached


def test_scan_reads_only_the_presented_tag(monkeypatch):
    # No prime: the scan reads whatever is presented (a full decode) and does not
    # pre-read/cache the neighbour slot.
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}                  # slot 0; sibling is slot 1
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0", "scan_interval": 0.0,
                         "scanner_confirm_reads": 1})
    seq = [{"uid": "new", "filament": {"type": "PLA"}}]
    monkeypatch.setattr(rfid, "read_tag", lambda link, **kw: seq.pop(0))

    tag = obj._scan_slot(0, duration=5.0)

    assert tag == {"uid": "new", "filament": {"type": "PLA"}}
    assert 1 not in obj._slot_uid                  # neighbour slot never pre-read


def test_scan_lane_missing_slot_raises():
    a = _Ace2()                                    # no slot map for the lane
    obj = _make_rfid(a, {"scanner_lanes": "nolane"})
    with pytest.raises(_CmdError):
        obj.scan_lane("nolane", 5)


def test_cmd_scan_defaults_to_sole_scanner_lane(monkeypatch):
    a = _Ace2()
    a._slot_map = {"scan_lane": 0}
    obj = _make_rfid(a, {"lane_slot_map": "scan_lane:0",
                         "scanner_lanes": "scan_lane"})
    obj.afc = types.SimpleNamespace(spool=types.SimpleNamespace(next_spool_id=7))
    seen = {}

    def _scan(name, secs):
        seen["name"], seen["secs"] = name, secs
        return {"uid": "abcd", "filament": {"type": "PLA"}}

    monkeypatch.setattr(obj, "scan_lane", _scan)
    gcmd = _GCmd({})                               # no LANE -> sole scanner lane
    obj.cmd_ACE_RFID_SCAN(gcmd)

    assert seen["name"] == "scan_lane"
    assert seen["secs"] == 30.0                     # default scan_seconds
    assert any("staged next spool" in r for r in gcmd.responses)


def test_cmd_scan_requires_lane_when_multiple_scanners():
    obj = _make_rfid(_Ace2(), {"scanner_lanes": "a, b"})
    with pytest.raises(_CmdError):
        obj.cmd_ACE_RFID_SCAN(_GCmd({}))


# ── auto-read on insert ─────────────────────────────────────────────────────

def test_auto_read_on_insert_reads_mapped_lane(monkeypatch):
    a = _Ace2()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    # the read sequence ran for the mapped lane
    assert ("read_tag", 0) in recorded
    assert any(c[0] == "mfrc522_reader_power" for c in a._ace.calls)


def test_auto_read_skips_unmapped_lane(monkeypatch):
    a = _Ace2()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="laneX"))
    assert recorded == [] and a._ace.calls == []


def test_auto_read_disabled_by_config(monkeypatch):
    a = _Ace2()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0}, read_on_insert=False)
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    assert recorded == [] and a._ace.calls == []


# ── continuous stage scan (stop-on-detect + parked read) ────────────

class _ScanConn(_Conn):
    """_Conn plus the motion + status calls the continuous scan issues."""

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


class _ScanAce2(_Ace2):
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

    monkeypatch.setattr(rfid, "MifareClassic", _MC)


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
    obj.reactor = _AdvReactor(step=0.5)
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
    a = _Ace2()
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
    a = _Ace2()
    a._slot_map = {"lane1": 0}
    obj = _make_rfid(a, {"disable_rfid": True, "skip_factory_autostage": False,
                         "stage_read": True, "lane_slot_map": "lane1:0"})
    ctx = {"active": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)
    assert ctx.get("active") is False
    assert ctx["initial"] == 0.0                      # factory does the load


def test_disable_rfid_post_insert_no_read():
    a = _Ace2()
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
    link = rfid._Ace2RegLink(a, 0, power_index=0)
    obj._probe = {"link": link, "shared": (0, 1)}
    a._ace.calls.clear()
    obj._safe_probe_teardown()
    names = [c[0] for c in a._ace.calls]
    assert "mfrc522_reader_power" in names            # reader powered down
    assert "set_rfid_enable" not in names             # identify NOT restored


def test_stage_teardown_restores_identify_when_configured(monkeypatch):
    obj, a = _scan_obj(monkeypatch)
    obj.probe_restore_identify = True
    link = rfid._Ace2RegLink(a, 0, power_index=0)
    obj._probe = {"link": link, "shared": (0, 1)}
    a._ace.calls.clear()
    obj._safe_probe_teardown()
    enables = [c for c in a._ace.calls if c[0] == "set_rfid_enable"]
    assert {c[1]["index"] for c in enables} == {0, 1}
    assert all(c[1]["enable"] is True for c in enables)


def test_disable_factory_identify_all_slots():
    a = _Ace2()
    a.slot_count = 4
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj._disable_factory_identify()          # no slots arg -> all of them
    disabled = [c[1]["index"] for c in a._ace.calls if c[0] == "set_rfid_enable"]
    assert sorted(disabled) == [0, 1, 2, 3]
    assert all(c[1]["enable"] is False
               for c in a._ace.calls if c[0] == "set_rfid_enable")


def test_retry_disable_identify_when_connected():
    a = _Ace2()                                      # a._ace.connected is True
    obj = _make_rfid(a, {"skip_factory_autostage": True})
    obj._retry_disable_identify(0.0)
    disabled = [c[1]["index"] for c in a._ace.calls if c[0] == "set_rfid_enable"]
    assert sorted(disabled) == [0, 1, 2, 3]          # disabled once connected
    assert all(c[1]["enable"] is False
               for c in a._ace.calls if c[0] == "set_rfid_enable")


def test_retry_disable_identify_gives_up_when_never_ready(caplog):
    a = _Ace2()
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
    a = _Ace2()
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

    monkeypatch.setattr(rfid, "read_tag", fake)
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
    a = _Ace2()
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
    a = _Ace2()
    a._slot_map = {"lane0": 0, "lane1": 1}
    obj = _bare_rfid(a, lane_slot={"lane1": 1})     # explicit allow-list
    assert obj._rfid_enabled("lane1") is True
    assert obj._rfid_enabled("lane0") is False       # present on ACE, not allowed


def test_rfid_enabled_auto_from_ace_when_no_map():
    a = _Ace2()
    a._slot_map = {"lane0": 0, "lane1": 1}
    obj = _bare_rfid(a)                              # no lane_slot_map
    assert obj._rfid_enabled("lane0") is True
    assert obj._rfid_enabled("laneX") is False


def test_slot_for_lane_prefers_ace_map_over_stale_config():
    a = _Ace2()
    a._slot_map = {"lane2": 2}
    obj = _bare_rfid(a, lane_slot={"lane2": 1})      # wrong/stale config value
    assert obj._slot_for_lane("lane2") == 2          # ACE map wins


def test_stage_probe_skips_unconfigured_lane(monkeypatch):
    a = _Ace2()
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj.stage_read = True
    _patch_read_tag(monkeypatch, [])
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="laneX"), ctx)
    assert ctx["active"] is False and a._ace.calls == []


def test_stage_probe_disabled_leaves_ctx_inactive():
    a = _Ace2()
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj.stage_read = False
    ctx = {"active": False, "done": False}
    obj._stage_probe_begin(types.SimpleNamespace(name="lane1"), ctx)
    assert ctx["active"] is False and a._ace.calls == []


def test_post_insert_skipped_when_stage_read(monkeypatch):
    a = _Ace2()
    recorded = []
    obj = _bare_rfid(a, lane_slot={"lane1": 0})
    obj.stage_read = True                    # stage-probe handles it instead
    _patch_read_tag(monkeypatch, recorded)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    assert recorded == [] and a._ace.calls == []


def test_auto_read_retries_until_tag(monkeypatch):
    a = _Ace2()
    obj = _bare_rfid(a, lane_slot={"lane1": 2}, attempts=3, delay=0.0)
    calls = {"n": 0}

    def fake(link, bambu_master_key=None, is_excluded=None, **kw):
        calls["n"] += 1
        return None if calls["n"] < 2 else {"uid": "bb", "tag_type": "x", "filament": None}
    monkeypatch.setattr(rfid, "read_tag", fake)
    obj._on_post_insert(types.SimpleNamespace(name="lane1"))
    assert calls["n"] == 2                        # stopped on first success


# ── sister-retract (auto_tag_adjust): clear a dominating sibling tag ─────────

class _MoveConn(_Conn):
    """_Conn plus the ACE motion commands the sister-retract issues."""

    def unwind_filament(self, index, length, speed, mode="normal"):
        self.calls.append(
            ("unwind_filament", {"index": index, "length": length, "speed": speed}))

    def feed_filament(self, index, length, speed):
        self.calls.append(
            ("feed_filament", {"index": index, "length": length, "speed": speed}))

    def stop_feed_assist_sync(self, index, timeout=2.0):
        self.calls.append(("stop_feed_assist_sync", {"index": index}))


class _MoveAce2(_Ace2):
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
    obj.reactor = _AdvReactor(step=0.5)              # advance so the scan deadline fires
    monkeypatch.setattr(rfid, "read_tag",
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
    obj = _bare_rfid(_Ace2(), lane_slot={"lane1": 0})
    tag = {"uid": "aa", "tag_type": "MifareClassic1k",
           "filament": {"type": "PLA", "manufacturer": "Bambu",
                        "color_argb": 0xFFE94B3C,
                        "colors_argb": [0xFFE94B3C, 0xFF112233]}}
    si = obj._map(tag)
    assert si["color_hex"] == "e94b3c"
    assert si["multi_color"] == ["e94b3c", "112233"]
    assert si["is_dual_color"] is True


def test_map_single_color_not_dual():
    obj = _bare_rfid(_Ace2(), lane_slot={"lane1": 0})
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
    obj = _bare_rfid(_Ace2(), lane_slot={"lane1": 0})
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
        obj = _bare_rfid(_Ace2(), lane_slot={"lane1": 0})
        status = obj.get_status()
        assert status["lane_slot_map"] == {"lane1": 0}
        assert status["last_reads"] == {}

    def test_last_reads_after_record(self):
        obj = _bare_rfid(_Ace2(), lane_slot={"lane1": 0})
        obj.record_tag_read("lane1", {"material": "PLA", "uid": "AABB"})
        rec = obj.get_status()["last_reads"]["lane1"]
        assert rec["material"] == "PLA"
        assert rec["uid"] == "AABB"
        assert rec["decoded"] is True
