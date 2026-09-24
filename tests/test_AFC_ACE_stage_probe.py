"""
Tests for the RFID stage-probe feed in extras/AFC_ACE.py (_feed_to_hub_probing).

A stage-read listener (afc_ace:stage_probe_begin) runs the WHOLE RFID scan on a
continuous feed and reports how far it fed via ctx['fed'], plus a mandatory
ctx['initial'] load (which replaces the bypassed factory staging). The feeder
then stages the REMAINDER (initial + dist_hub - fed) in ONE move at full speed.
With no listener fed=0 and this is just one initial+dist_hub feed.
"""
from __future__ import annotations

import types

from extras.AFC_ACE import afcACE
from tests.ace_helpers import FakeAce, FakeLane, FakeLogger, Recorder


def _make_unit(feeds, unwinds, handlers):
    unit = afcACE.__new__(afcACE)
    ace = FakeAce(connected=True)
    ace.feed_filament = lambda slot, d, sp: feeds.append(round(d, 3))
    ace.unwind_filament = lambda slot, d, sp: unwinds.append(round(d, 3))
    unit._ace = ace
    unit.logger = FakeLogger()
    unit.feed_speed = 50.0
    unit.retract_speed = 50.0
    unit.prep_ready_timeout = 5.0
    unit._wait_for_ace_ready = Recorder()
    unit._wait_for_feed_complete = Recorder()
    unit._slot_reports_empty = lambda slot: False       # present unless overridden

    class _Printer:
        def send_event(self, name, *a):
            h = handlers.get(name)
            if h is not None:
                h(*a)
    unit.printer = _Printer()
    return unit


def _handlers(fed=0.0, initial=100.0, done=False, removed=False):
    """Stage-read listener: the scan (run inside begin) reports how far it fed
    (``fed``), the mandatory ``initial`` load, whether it read a tag (``done``)
    and whether the spool was pulled mid-scan (``removed``)."""
    def begin(lane, ctx):
        ctx["active"] = True
        ctx["initial"] = initial
        ctx["fed"] = fed
        if done:
            ctx["done"] = True
        if removed:
            ctx["removed"] = True

    return {
        "afc_ace:stage_probe_begin": begin,
        "afc_ace:stage_probe_end": lambda lane, ctx: None,
    }


def test_no_listener_is_plain_dist_hub_feed():
    feeds, unwinds = [], []
    unit = _make_unit(feeds, unwinds, {})              # no listener -> fed=0
    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [200.0]                             # one move, dist_hub only
    assert unwinds == []


def test_inactive_with_initial_is_one_plain_feed():
    # disable_rfid path: the listener requests the initial load but the scan
    # fed nothing -> feed initial + dist_hub in ONE move.
    feeds, unwinds = [], []

    def begin(lane, ctx):
        ctx["initial"] = 100.0                         # load set, scan fed nothing

    handlers = {
        "afc_ace:stage_probe_begin": begin,
        "afc_ace:stage_probe_end": lambda lane, ctx: None,
    }
    unit = _make_unit(feeds, unwinds, handlers)
    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [300.0]                             # initial 100 + dist_hub 200
    assert unwinds == []


def test_read_during_scan_feeds_remainder_in_one_move():
    # Scan fed 150mm and read the tag -> remainder (100+200 - 150) in one move.
    # The smooth scan doesn't trip the stuck-spool latch, so no clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=150.0, initial=100.0, done=True)
    unit = _make_unit(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [150.0]                             # 300 total - 150 already fed
    assert unwinds == []                               # no recovery unwind needed


def test_tagless_feeds_remainder_after_full_scan():
    # Tagless spool: scan fed the whole 200mm window, no read -> feed the
    # remainder (300-200) in one move, no clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=200.0, initial=100.0, done=False)
    unit = _make_unit(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == [100.0]                             # 300 total - 200 scanned
    assert sum(feeds) + 200.0 == 300.0                 # net = initial + dist_hub
    assert unwinds == []


def test_scan_fed_full_load_needs_no_remainder():
    # The scan already fed the entire initial+dist_hub -> no remainder move.
    feeds, unwinds = [], []
    handlers = _handlers(fed=300.0, initial=100.0, done=True)
    unit = _make_unit(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert feeds == []                                  # nothing left to stage
    assert unwinds == []                               # no recovery unwind needed


def test_dist_hub_zero_still_does_initial_load():
    feeds, unwinds = [], []
    handlers = _handlers(fed=0.0, initial=100.0, done=False)
    unit = _make_unit(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=0.0)

    assert feeds == [100.0]                             # just the initial load
    assert unwinds == []


def test_completed_stage_returns_true():
    feeds, unwinds = [], []
    handlers = _handlers(fed=0.0, initial=100.0, done=False)
    unit = _make_unit(feeds, unwinds, handlers)

    result = unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert result is True
    assert feeds == [300.0]


def test_stage_aborts_when_scan_reports_removed():
    # The scan detected the spool pulled mid-feed (ctx['removed']) -> abort, no
    # remainder feed, no clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=120.0, initial=100.0, removed=True)
    unit = _make_unit(feeds, unwinds, handlers)

    result = unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert result is False
    assert feeds == []
    assert unwinds == []
    assert any("filament removed during staging" in m
               for m in unit.logger.lines["debug"])


def test_stage_aborts_when_slot_already_empty():
    feeds, unwinds = [], []
    handlers = _handlers(fed=0.0, initial=100.0)
    unit = _make_unit(feeds, unwinds, handlers)
    unit._slot_reports_empty = lambda slot: True       # gone before the remainder

    result = unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert result is False
    assert feeds == []                                 # never fed an empty slot
    assert unwinds == []


def test_recovery_unwind_skipped_when_no_read():
    # No read (done False) -> no stuck-spool clearing unwind.
    feeds, unwinds = [], []
    handlers = _handlers(fed=200.0, initial=100.0, done=False)
    unit = _make_unit(feeds, unwinds, handlers)

    unit._feed_to_hub_probing(FakeLane("lane3"), 3, dist_hub=200.0)

    assert unwinds == []


# ── prep_post_load must NOT short-circuit at dist_hub=0 ───────────────────────

def _prep_unit():
    """Minimal unit wired for prep_post_load: it should run the stage probe even
    when dist_hub=0 (the stage scan / initial load are what pull the filament
    into the unit; dist_hub is additive on top)."""
    import contextlib
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    unit._unit_load_to_hub = None
    unit._ace = FakeAce(connected=True)
    unit._get_slot = lambda name: 3
    unit.prep_ready_timeout = 5.0
    unit._wait_for_ace_ready = Recorder()
    unit._set_hub_state = lambda lane, state: None
    unit._operation = lambda: contextlib.nullcontext()
    unit.afc = types.SimpleNamespace(
        save_vars=lambda: None, td1_present=False)
    return unit


def test_prep_post_load_runs_probe_when_dist_hub_zero():
    unit = _prep_unit()
    calls = []
    unit._feed_to_hub_probing = lambda lane, slot, dist: (
        calls.append((slot, dist)) or True)

    lane = FakeLane("lane3")
    lane.load_to_hub = True
    lane.loaded_to_hub = False
    lane.prep_state = True
    lane.dist_hub = 0.0
    lane.td1_when_loaded = False
    lane.td1_device_id = None

    unit.prep_post_load(lane)

    assert calls == [(3, 0.0)]          # probe ran even with dist_hub=0
    assert lane.loaded_to_hub is True   # staged once the probe returned True


# ── _slot_reports_empty helper ─────────────────────────────────────────────────

class _StatusAce:
    def __init__(self, status=None, raises=False, connected=True, sequence=None):
        self.connected = connected
        self._status = status
        self._raises = raises
        self._sequence = list(sequence) if sequence else None

    def get_status(self, timeout=2.0):
        if self._raises:
            raise RuntimeError("status timeout")
        if self._sequence is not None:
            return self._sequence.pop(0) if self._sequence else self._status
        return self._status


class _NoWaitReactor:
    def monotonic(self):
        return 0.0

    def pause(self, when):
        pass


def _unit_with_ace(ace):
    unit = afcACE.__new__(afcACE)
    unit._ace = ace
    unit.logger = FakeLogger()
    unit.afc = types.SimpleNamespace(reactor=_NoWaitReactor())
    return unit


def _empty(slot="empty"):
    return {"status": "ready", "slots": [{"status": slot}]}


def test_slot_reports_empty_true_when_stably_empty():
    ace = _StatusAce(_empty())                          # empty on every poll
    assert _unit_with_ace(ace)._slot_reports_empty(0) is True


def test_slot_reports_empty_false_on_single_flicker():
    # empty once, then present: a one-poll flicker must NOT abort a live stage.
    ace = _StatusAce(sequence=[_empty(), _empty("ready")])
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_slot_ready():
    ace = _StatusAce(_empty("ready"))
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_unit_busy():
    # A busy unit's slots can flicker 'empty' mid-motion; never abort on that.
    ace = _StatusAce({"status": "busy", "slots": [{"status": "empty"}]})
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_query_fails():
    assert _unit_with_ace(_StatusAce(raises=True))._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_disconnected():
    ace = _StatusAce(_empty(), connected=False)
    assert _unit_with_ace(ace)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_no_ace():
    assert _unit_with_ace(None)._slot_reports_empty(0) is False


def test_slot_reports_empty_false_when_slot_index_out_of_range():
    ace = _StatusAce(_empty())
    assert _unit_with_ace(ace)._slot_reports_empty(5) is False
