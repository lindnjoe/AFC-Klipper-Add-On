"""
Unit tests for the ACE2 firmware-odometer stuck detection in extras/AFC_ACE2.py

Covers:
  - _check_stuck: fires the jam handler exactly once per jam (one-shot latch),
    re-arms when the slot recovers, resets when paused / not printing / no
    active assist lane, ignores idle slots' stale errors and malformed status
"""

from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_ACE2 import afcACE2


def _make_unit(active_lane="lane0", slot=0, printing=True, paused=False):
    unit = afcACE2.__new__(afcACE2)
    unit.logger = MagicMock()
    unit._stuck_detection = True
    unit._stuck_tripped = False
    unit._slot_map = {"lane0": 0, "lane1": 1}
    unit._active_assist_lane = MagicMock(return_value=active_lane)
    unit.afc = MagicMock()
    unit.afc.function.in_print.return_value = printing
    unit.afc.function.is_paused.return_value = paused
    return unit


def _status(slot_statuses):
    return {"slots": [{"slot_status": s} for s in slot_statuses]}


def test_jam_on_active_slot_schedules_handler_once():
    unit = _make_unit()

    unit._check_stuck(_status(["stuck_error", "ready"]))
    unit._check_stuck(_status(["stuck_error", "ready"]))  # same jam, next heartbeat

    assert unit._stuck_tripped is True
    # one-shot: handler deferred exactly once despite repeated heartbeats
    assert unit.afc.reactor.register_callback.call_count == 1


def test_recovery_rearms_latch():
    unit = _make_unit()

    unit._check_stuck(_status(["tangled_error"]))
    assert unit._stuck_tripped is True

    unit._check_stuck(_status(["ready"]))       # recovered
    assert unit._stuck_tripped is False

    unit._check_stuck(_status(["stuck_error"]))  # a new jam fires again
    assert unit.afc.reactor.register_callback.call_count == 2


def test_idle_slot_error_never_trips():
    """Only the active assist lane's slot is consulted — a stale error on an
    idle slot can't pause a healthy print."""
    unit = _make_unit(active_lane="lane0", slot=0)

    unit._check_stuck(_status(["ready", "stuck_error"]))  # error on slot 1

    assert unit._stuck_tripped is False
    unit.afc.reactor.register_callback.assert_not_called()


def test_paused_print_resets_and_skips():
    unit = _make_unit(paused=True)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    unit.afc.reactor.register_callback.assert_not_called()


def test_not_printing_resets_and_skips():
    unit = _make_unit(printing=False)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    unit.afc.reactor.register_callback.assert_not_called()


def test_no_active_lane_resets_and_skips():
    unit = _make_unit(active_lane=None)
    unit._stuck_tripped = True

    unit._check_stuck(_status(["stuck_error"]))

    assert unit._stuck_tripped is False
    unit.afc.reactor.register_callback.assert_not_called()


def test_detection_disabled_by_config():
    unit = _make_unit()
    unit._stuck_detection = False

    unit._check_stuck(_status(["stuck_error"]))

    unit.afc.reactor.register_callback.assert_not_called()


def test_malformed_status_is_ignored():
    unit = _make_unit()

    unit._check_stuck({})                    # no slots key
    unit._check_stuck({"slots": "garbage"})  # not a list
    unit._check_stuck(_status([]))           # slot index out of range

    assert unit._stuck_tripped is False
    unit.afc.reactor.register_callback.assert_not_called()


def test_all_jam_states_trip():
    for state in afcACE2._ENCODER_JAM_STATES:
        unit = _make_unit()
        unit._check_stuck(_status([state]))
        assert unit._stuck_tripped is True, state
