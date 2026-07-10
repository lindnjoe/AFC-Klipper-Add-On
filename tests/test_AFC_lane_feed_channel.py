"""
Unit tests for the external-feeder prep/load source on AFCLane (standalone /
U1 lanes): the "filament_feed:port" event handler + the self-cancelling
stage-watch that bridges the event-less gap to preload_finish.

Validated shape (measured on U1 hardware):
  - unload: filament_detected False and channel_state wait_insert flip together
    -> event carries the falling edge, mirror empty immediately.
  - reinsert: presence event fires at preload_prepare, channel reaches
    preload_finish ~9s later with no event -> stage-watch polls the gap.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from tests.test_AFC_lane import _make_afc_lane


class _FeedReactor:
    NEVER = 9_000_000_000.0
    NOW = 0.0

    def __init__(self, t=100.0):
        self._t = t
        self.timer_updates = []          # (timer, when) in call order

    def monotonic(self):
        return self._t

    def update_timer(self, timer, when):
        self.timer_updates.append((timer, when))

    # test helpers
    @property
    def last_update(self):
        return self.timer_updates[-1] if self.timer_updates else None

    def armed(self):
        return self.last_update == ("TIMER", self.NOW)

    def disarmed(self):
        return self.last_update == ("TIMER", self.NEVER)


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
    lane._feed_stage_timer = "TIMER"
    lane._feed_stage_deadline = 0.0
    lane.prep_state = False
    lane._load_state = False
    return lane


# ── _feed_channel_staged / _feed_channel_present ──────────────────────────────

def test_staged_true_at_preload_finish():
    lane = _feed_lane(detected=True, state="preload_finish")
    assert lane._feed_channel_staged() is True
    assert lane._feed_channel_present() is True


def test_staged_true_at_load_finish():
    lane = _feed_lane(detected=True, state="load_finish")
    assert lane._feed_channel_staged() is True


def test_present_but_not_staged_mid_preload():
    lane = _feed_lane(detected=True, state="preload_feeding")
    assert lane._feed_channel_present() is True
    assert lane._feed_channel_staged() is False


def test_absent_is_neither():
    lane = _feed_lane(detected=False, state="wait_insert")
    assert lane._feed_channel_present() is False
    assert lane._feed_channel_staged() is False


def test_feeder_exception_reads_absent():
    lane = _feed_lane(detected=True, state="preload_finish")
    lane._feed_obj.raise_on_status = True
    assert lane._feed_channel_staged() is False
    assert lane._feed_channel_present() is False


# ── _feed_reevaluate ──────────────────────────────────────────────────────────

def test_reevaluate_staged_marks_ready_and_disarms():
    lane = _feed_lane(detected=True, state="preload_finish")
    lane._feed_reevaluate()
    assert lane.prep_state is True and lane._load_state is True
    assert lane._feed_staged_last is True
    lane.afc.save_vars.assert_called_once()
    assert lane.reactor.disarmed()


def test_reevaluate_present_arms_watch_without_staging():
    lane = _feed_lane(detected=True, state="preload_prepare")
    lane._feed_reevaluate()
    assert lane.prep_state is False and lane._load_state is False
    lane.afc.save_vars.assert_not_called()
    assert lane.reactor.armed()
    assert lane._feed_stage_deadline == 100.0 + 30.0


def test_reevaluate_absent_marks_empty_and_disarms():
    lane = _feed_lane(detected=True, state="preload_finish")
    lane._feed_reevaluate()                 # staged first
    lane.afc.save_vars.reset_mock()
    lane._feed_obj.detected = False
    lane._feed_obj.state = "wait_insert"
    lane._feed_reevaluate()
    assert lane.prep_state is False and lane._load_state is False
    lane.afc.save_vars.assert_called_once()
    assert lane.reactor.disarmed()


# ── _feed_stage_watch (the bridge) ────────────────────────────────────────────

def test_stage_watch_completes_when_preload_finishes():
    lane = _feed_lane(detected=True, state="preload_feeding")
    lane._arm_stage_watch()
    lane._feed_obj.state = "preload_finish"     # gap closed
    ret = lane._feed_stage_watch(lane.reactor.monotonic())
    assert lane.prep_state is True and lane._load_state is True
    assert ret == lane.reactor.NEVER            # self-cancels
    lane.afc.save_vars.assert_called_once()


def test_stage_watch_keeps_polling_while_progressing():
    lane = _feed_lane(detected=True, state="preload_feeding")
    lane._arm_stage_watch()                     # deadline = 130.0
    ret = lane._feed_stage_watch(110.0)         # before deadline, still feeding
    assert ret == 110.5
    assert lane.prep_state is False
    lane.afc.save_vars.assert_not_called()


def test_stage_watch_stops_if_pulled_mid_preload():
    lane = _feed_lane(detected=True, state="preload_feeding")
    lane._arm_stage_watch()
    lane._feed_obj.detected = False             # yanked back out
    ret = lane._feed_stage_watch(200.0)
    assert ret == lane.reactor.NEVER
    assert lane.prep_state is False


def test_stage_watch_gives_up_at_deadline_without_staging():
    lane = _feed_lane(detected=True, state="preload_feeding")
    lane._arm_stage_watch()                     # deadline = 130.0
    ret = lane._feed_stage_watch(200.0)         # past deadline, still not staged
    assert ret == lane.reactor.NEVER
    assert lane.prep_state is False
    lane.afc.save_vars.assert_not_called()


# ── _feed_port_event (channel filtering + edges) ──────────────────────────────

def test_event_ignores_other_channel():
    lane = _feed_lane(ch_index=2, detected=True, state="preload_finish")
    lane._feed_port_event(1, True)              # a different channel's event
    assert lane.prep_state is False
    lane.afc.save_vars.assert_not_called()
    assert lane.reactor.last_update is None


def test_event_our_channel_rising_edge_stages():
    lane = _feed_lane(ch_index=2, detected=True, state="preload_finish")
    lane._feed_port_event(2, True)
    assert lane.prep_state is True and lane._load_state is True


def test_event_falling_edge_unstages():
    lane = _feed_lane(ch_index=2, detected=True, state="preload_finish")
    lane._feed_port_event(2, True)              # staged
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
