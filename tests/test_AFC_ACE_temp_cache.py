"""
Unit tests for the ACE get_temp caching path (extras/AFC_ACE.py + AFC_ACE2.py):

  _on_hw_status_callback — routes the two async heartbeat replies correctly:
      a get_status reply (has 'slots') updates the status cache and runs slot
      sync; a get_temp reply (thermal channels, no 'slots') updates the temp
      cache ONLY and never touches status or runs sync.
  _poll_extras — base connection is a no-op (V1 has no get_temp); ACE2 fires an
      async get_temp.

Style: typed fakes, full state verification, branch coverage.
"""

from __future__ import annotations

from extras.AFC_ACE import afcACE, ACEConnection
from extras.AFC_ACE2 import ACE2Connection

from tests.ace_helpers import FakeLogger, Recorder


# ── _on_hw_status_callback routing ────────────────────────────────────────────

def _make_unit(operation_active=False):
    unit = afcACE.__new__(afcACE)
    unit.name = "Ace_1"
    unit.logger = FakeLogger()
    unit._cached_hw_status = {}
    unit._cached_temp_info = {}
    unit._current_action = ""
    unit._operation_active = operation_active
    unit._sync_slot_states = Recorder()
    unit._maybe_assist_watchdog = Recorder()
    unit._check_stuck = Recorder()
    return unit


def test_status_reply_updates_status_cache_and_syncs():
    unit = _make_unit()
    status = {"status": "ready", "slots": [{"status": "ready"}]}

    unit._on_hw_status_callback({"result": status})

    assert unit._cached_hw_status == status
    assert unit._cached_temp_info == {}          # untouched
    assert unit._sync_slot_states.call_count == 1
    assert unit._sync_slot_states.last_args == (status,)
    assert unit._maybe_assist_watchdog.call_count == 1
    assert unit._check_stuck.call_count == 1


def test_temp_reply_updates_temp_cache_only():
    unit = _make_unit()
    unit._cached_hw_status = {"status": "ready", "slots": []}
    prev_status = unit._cached_hw_status
    temp = {"box1_temp": 0.0, "ptc1_temp": 55.0, "env_temp": 27.0,
            "env_humidity": 30.0}

    unit._on_hw_status_callback({"result": temp})

    assert unit._cached_temp_info == temp
    assert unit._cached_hw_status is prev_status  # status cache NOT overwritten
    # No slot-state work runs on a thermal reply.
    assert unit._sync_slot_states.call_count == 0
    assert unit._maybe_assist_watchdog.call_count == 0
    assert unit._check_stuck.call_count == 0


def test_temp_reply_detected_by_box_or_env_only():
    # A reply with box1_temp but no ptc/env still routes to the temp cache.
    unit = _make_unit()
    unit._on_hw_status_callback({"result": {"box1_temp": 24.0}})
    assert unit._cached_temp_info == {"box1_temp": 24.0}
    assert unit._sync_slot_states.call_count == 0


def test_status_reply_with_operation_active_caches_but_skips_sync():
    unit = _make_unit(operation_active=True)
    status = {"status": "busy", "slots": []}

    unit._on_hw_status_callback({"result": status})

    assert unit._cached_hw_status == status      # still cached
    assert unit._sync_slot_states.call_count == 0  # but sync suppressed


def test_non_dict_response_ignored():
    unit = _make_unit()
    unit._on_hw_status_callback("not a dict")
    unit._on_hw_status_callback({"result": "not a dict"})
    assert unit._cached_hw_status == {}
    assert unit._cached_temp_info == {}
    assert unit._sync_slot_states.call_count == 0


def test_bare_status_without_result_wrapper():
    # Some callers pass the status dict directly (no 'result' envelope).
    unit = _make_unit()
    status = {"slots": [], "status": "ready"}
    unit._on_hw_status_callback(status)
    assert unit._cached_hw_status == status
    assert unit._sync_slot_states.call_count == 1


# ── _poll_extras ──────────────────────────────────────────────────────────────

def test_base_poll_extras_is_noop():
    conn = ACEConnection.__new__(ACEConnection)
    conn.send_command_async = Recorder()
    # Should not raise and should not send anything (V1 has no get_temp).
    assert conn._poll_extras() is None
    assert conn.send_command_async.call_count == 0


def test_ace2_poll_extras_sends_get_temp():
    conn = ACE2Connection.__new__(ACE2Connection)
    conn.send_command_async = Recorder()

    conn._poll_extras()

    # Polls both the thermal channels and the per-lane buffer/sensor state.
    assert conn.send_command_async.call_count == 2
    methods = [c[0][0] for c in conn.send_command_async.calls]
    assert methods == ["get_temp", "get_sensor_state"]
