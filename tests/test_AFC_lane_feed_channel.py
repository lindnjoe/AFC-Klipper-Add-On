"""
Unit tests for the external-feeder prep/load source on AFCLane (standalone /
U1 lanes): the "filament_feed:port" event handler that mirrors the feeder's
per-channel filament presence into the lane's prep/load flags.

We trust the feeder to stage the filament correctly, so presence alone marks
the lane ready — there is no stage-watch timer waiting for preload_finish. Both
edges come straight from the feeder event:
  - unload: filament_detected goes False -> event carries the falling edge,
    mirror empty immediately.
  - reinsert: filament_detected goes True -> event carries the rising edge,
    mirror ready immediately (no wait for preload_finish).
"""

from __future__ import annotations

from unittest.mock import MagicMock

from tests.test_AFC_lane import _make_afc_lane


class _FeedReactor:
    NEVER = 9_000_000_000.0
    NOW = 0.0

    def __init__(self, t=100.0):
        self._t = t

    def monotonic(self):
        return self._t


class _FakeFeeder:
    def __init__(self, key, detected=False, state="wait_insert"):
        self.key = key
        self.detected = detected
        self.state = state
        self.raise_on_status = False

    def get_status(self, eventtime):
        if self.raise_on_status:
            raise RuntimeError("feeder offline")
        return {self.key: {"filament_detected": self.detected,
                           "channel_state": self.state}}


def _feed_lane(key="extruder2", ch_index=2, detected=False, state="wait_insert"):
    lane = _make_afc_lane("AFC_stepper lane10")
    lane.afc = MagicMock()               # fresh, for save_vars assertions
    lane.reactor = _FeedReactor()
    lane.feed_module = "right"
    lane.feed_channel = key
    lane._feed_ch_index = ch_index
    lane._feed_obj = _FakeFeeder(key, detected=detected, state=state)
    lane._feed_staged_last = None
    lane.prep_state = False
    lane._load_state = False
    return lane


# ── _feed_channel_present ─────────────────────────────────────────────────────

def test_present_at_preload_finish():
    lane = _feed_lane(detected=True, state="preload_finish")
    assert lane._feed_channel_present() is True


def test_present_at_load_finish():
    lane = _feed_lane(detected=True, state="load_finish")
    assert lane._feed_channel_present() is True


def test_present_regardless_of_channel_state_mid_preload():
    # Detected mid-preload is still present -> ready; we trust the feeder.
    lane = _feed_lane(detected=True, state="preload_feeding")
    assert lane._feed_channel_present() is True


def test_absent_is_not_present():
    lane = _feed_lane(detected=False, state="wait_insert")
    assert lane._feed_channel_present() is False


def test_feeder_exception_reads_absent():
    lane = _feed_lane(detected=True, state="preload_finish")
    lane._feed_obj.raise_on_status = True
    assert lane._feed_channel_present() is False


# ── _feed_reevaluate ──────────────────────────────────────────────────────────

def test_reevaluate_present_marks_ready():
    lane = _feed_lane(detected=True, state="preload_finish")
    lane._feed_reevaluate()
    assert lane.prep_state is True and lane._load_state is True
    assert lane._feed_staged_last is True
    lane.afc.save_vars.assert_called_once()


def test_reevaluate_present_mid_preload_marks_ready():
    # No stage-watch to arm anymore — presence is trusted immediately.
    lane = _feed_lane(detected=True, state="preload_prepare")
    lane._feed_reevaluate()
    assert lane.prep_state is True and lane._load_state is True
    lane.afc.save_vars.assert_called_once()


def test_reevaluate_absent_marks_empty():
    lane = _feed_lane(detected=True, state="preload_finish")
    lane._feed_reevaluate()                 # ready first
    lane.afc.save_vars.reset_mock()
    lane._feed_obj.detected = False
    lane._feed_obj.state = "wait_insert"
    lane._feed_reevaluate()
    assert lane.prep_state is False and lane._load_state is False
    lane.afc.save_vars.assert_called_once()


# ── _feed_port_event (channel filtering + edges) ──────────────────────────────

def test_event_ignores_other_channel():
    lane = _feed_lane(ch_index=2, detected=True, state="preload_finish")
    lane._feed_port_event(1, True)              # a different channel's event
    assert lane.prep_state is False
    lane.afc.save_vars.assert_not_called()


def test_event_our_channel_rising_edge_stages():
    lane = _feed_lane(ch_index=2, detected=True, state="preload_finish")
    lane._feed_port_event(2, True)
    assert lane.prep_state is True and lane._load_state is True


def test_event_falling_edge_unstages():
    lane = _feed_lane(ch_index=2, detected=True, state="preload_finish")
    lane._feed_port_event(2, True)              # ready
    lane.afc.save_vars.reset_mock()
    lane._feed_obj.detected = False
    lane._feed_obj.state = "wait_insert"
    lane._feed_port_event(2, False)             # feeder announces removal
    assert lane.prep_state is False and lane._load_state is False
    lane.afc.save_vars.assert_called_once()


def test_event_unknown_index_still_reevaluates():
    lane = _feed_lane(ch_index=None, detected=True, state="preload_finish")
    lane._feed_port_event(99, True)             # index unknown -> don't filter
    assert lane.prep_state is True


# ── _apply_staged edge guard ──────────────────────────────────────────────────

def test_apply_staged_persists_only_on_change():
    lane = _feed_lane()
    lane._apply_staged(True)
    lane._apply_staged(True)                    # no change
    lane.afc.save_vars.assert_called_once()
    lane._apply_staged(False)                   # change back
    assert lane.afc.save_vars.call_count == 2
