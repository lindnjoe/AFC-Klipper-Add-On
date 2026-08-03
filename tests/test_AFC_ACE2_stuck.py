"""
Unit tests for the ACE2 firmware-odometer stuck detection in
extras/AFC_ACE2.py

Style: typed fakes (tests/ace_helpers.py) instead of MagicMock, full state
verification, branch-complete coverage of _check_stuck:

  detection disabled / not printing / paused / no active assist lane /
  lane missing from the slot map / malformed status payloads / healthy
  re-arm / every jam state trips / one-shot latch across heartbeats /
  recovery re-arms for the next jam
"""

from __future__ import annotations

from extras.AFC_ACE2 import afcACE2

from tests.ace_helpers import FakeAFC, FakeLogger, Recorder


def _make_unit(active_lane="lane0", printing=True, paused=False,
               slot_map=None):
    unit = afcACE2.__new__(afcACE2)
    unit.logger = FakeLogger()
    unit._stuck_detection = True
    unit._stuck_tripped = False
    unit._slot_map = slot_map if slot_map is not None else {"lane0": 0, "lane1": 1}
    unit._active_assist_lane = Recorder(result=active_lane)
    unit.afc = FakeAFC()
    unit.afc.function.in_print_flag = printing
    unit.afc.function.paused = paused
    return unit


def _status(slot_statuses):
    return {"slots": [{"slot_status": s} for s in slot_statuses]}


# ── Firing + latch ────────────────────────────────────────────────────────────

def test_jam_on_active_slot_schedules_handler_once():
    unit = _make_unit()

    unit._check_stuck(_status(["stuck_error", "ready"]))
    unit._check_stuck(_status(["stuck_error", "ready"]))  # next heartbeat

    assert unit._stuck_tripped is True
    # one-shot: handler deferred exactly once despite repeated heartbeats
    assert unit.afc.reactor.register_callback.call_count == 1


def test_all_jam_states_trip():
    for state in afcACE2._ENCODER_JAM_STATES:
        unit = _make_unit()
        unit._check_stuck(_status([state]))
        assert unit._stuck_tripped is True, state
        assert unit.afc.reactor.register_callback.call_count == 1, state


def test_recovery_rearms_latch():
    unit = _make_unit()

    unit._check_stuck(_status(["tangled_error"]))
    assert unit._stuck_tripped is True

    unit._check_stuck(_status(["ready"]))        # recovered
    assert unit._stuck_tripped is False

    unit._check_stuck(_status(["stuck_error"]))  # a new jam fires again
    assert unit._stuck_tripped is True
    assert unit.afc.reactor.register_callback.call_count == 2


def test_healthy_slot_never_trips():
    unit = _make_unit()
    unit._check_stuck(_status(["ready"]))
    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_idle_slot_error_never_trips():
    """Only the active assist lane's slot is consulted — a stale error on an
    idle slot can't pause a healthy print."""
    unit = _make_unit(active_lane="lane0")

    unit._check_stuck(_status(["ready", "stuck_error"]))  # error on slot 1

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


# ── Gate branches ─────────────────────────────────────────────────────────────

def test_detection_disabled_by_config():
    unit = _make_unit()
    unit._stuck_detection = False
    unit._stuck_tripped = True  # must stay untouched — gate is before resets

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is True
    assert not unit.afc.reactor.register_callback.called


def test_not_printing_resets_and_skips():
    unit = _make_unit(printing=False)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_paused_print_resets_and_skips():
    unit = _make_unit(paused=True)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_no_active_lane_resets_and_skips():
    unit = _make_unit(active_lane=None)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_active_lane_missing_from_slot_map_skips():
    unit = _make_unit(active_lane="ghost")

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called


def test_malformed_status_is_ignored():
    unit = _make_unit()

    unit._check_stuck({})                          # no slots key
    unit._check_stuck({"slots": "garbage"})        # not a list
    unit._check_stuck(_status([]))                 # index out of range
    unit._check_stuck({"slots": ["not-a-dict"]})   # slot entry not a dict

    assert unit._stuck_tripped is False
    assert not unit.afc.reactor.register_callback.called
