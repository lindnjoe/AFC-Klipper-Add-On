"""
Unit tests for the ACE "current action" surfacing (extras/AFC_ACE.py):

  _derive_action        — uniform action string across V1 (top-level 'action' /
                          per-slot 'status') and ACE2 (per-slot 'slot_status'),
                          '' when idle, slot-tagged when busy.
  _on_hw_status_callback — logs action transitions (idle<->busy) once per change,
                          tracks _current_action, and does so even while an
                          operation is active (when it is most useful).
  cmd_ACE_STATUS        — reports the derived action alongside the raw status.

Style: typed fakes, full state verification, branch coverage.
"""

from __future__ import annotations

import types

from extras.AFC_ACE import afcACE

from tests.ace_helpers import FakeLogger, FakeGcmd, FakeAce, Recorder


V1_BUSY = {"status": "busy", "action": "preload",
           "slots": [{"index": 0, "status": "preload"},
                     {"index": 1, "status": "ready"}]}
V1_IDLE = {"status": "ready", "slots": [{"index": 0, "status": "ready"}]}
ACE2_BUSY = {"status": "busy",
             "slots": [{"index": 0, "slot_status": "feeding", "status": "ready"},
                       {"index": 1, "slot_status": "ready", "status": "ready"}]}
ACE2_IDLE = {"status": "ready",
             "slots": [{"index": 0, "slot_status": "ready", "status": "ready"}]}


# ── _derive_action ────────────────────────────────────────────────────────────

def test_derive_action_v1_busy_slot_tagged():
    assert afcACE._derive_action(V1_BUSY) == "preload(slot 0)"


def test_derive_action_ace2_busy_slot_uses_slot_status():
    assert afcACE._derive_action(ACE2_BUSY) == "feeding(slot 0)"


def test_derive_action_top_level_fallback_when_no_busy_slot():
    r = {"action": "drying", "slots": [{"index": 0, "status": "ready"}]}
    assert afcACE._derive_action(r) == "drying"


def test_derive_action_idle_returns_empty():
    assert afcACE._derive_action(V1_IDLE) == ""
    assert afcACE._derive_action(ACE2_IDLE) == ""


def test_derive_action_no_slot_index_untagged():
    assert afcACE._derive_action({"slots": [{"slot_status": "rollback"}]}) == "rollback"


def test_derive_action_non_dict_and_empty():
    assert afcACE._derive_action(None) == ""
    assert afcACE._derive_action("nope") == ""
    assert afcACE._derive_action({}) == ""


# ── _on_hw_status_callback transition logging ─────────────────────────────────

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


def test_callback_logs_transition_and_tracks_action():
    unit = _make_unit()

    unit._on_hw_status_callback({"result": V1_BUSY})
    assert unit._current_action == "preload(slot 0)"
    assert any("Ace_1: idle -> preload(slot 0)" in m
               for m in unit.logger.lines["info"])

    unit._on_hw_status_callback({"result": V1_IDLE})
    assert unit._current_action == ""
    assert any("preload(slot 0) -> idle" in m for m in unit.logger.lines["info"])


def test_callback_no_duplicate_log_when_action_unchanged():
    unit = _make_unit()
    unit._on_hw_status_callback({"result": ACE2_BUSY})
    n = len(unit.logger.lines["info"])
    unit._on_hw_status_callback({"result": ACE2_BUSY})   # same action
    assert len(unit.logger.lines["info"]) == n           # no new transition line


def test_callback_tracks_action_even_during_operation():
    unit = _make_unit(operation_active=True)

    unit._on_hw_status_callback({"result": ACE2_BUSY})

    assert unit._current_action == "feeding(slot 0)"     # logged despite op-active
    assert unit._sync_slot_states.call_count == 0        # but sync still skipped


def test_callback_temp_reply_does_not_touch_action():
    unit = _make_unit()
    unit._current_action = "feeding(slot 0)"
    # A get_temp reply (no slots) routes to the temp cache and returns early.
    unit._on_hw_status_callback({"result": {"ptc1_temp": 55.0}})
    assert unit._current_action == "feeding(slot 0)"     # unchanged


# ── cmd_ACE_STATUS ────────────────────────────────────────────────────────────

def _status_unit(status_result):
    unit = afcACE.__new__(afcACE)
    ace = FakeAce()
    ace.get_status = Recorder(result=status_result)
    unit._ace = ace
    return unit


def test_cmd_ace_status_reports_busy_action():
    unit = _status_unit(ACE2_BUSY)
    gcmd = FakeGcmd()
    unit.cmd_ACE_STATUS(gcmd)
    assert any("action: feeding(slot 0)" in r for r in gcmd.responses)


def test_cmd_ace_status_reports_idle():
    unit = _status_unit(V1_IDLE)
    gcmd = FakeGcmd()
    unit.cmd_ACE_STATUS(gcmd)
    assert any("action: idle" in r for r in gcmd.responses)


def test_cmd_ace_status_not_connected():
    unit = _status_unit(V1_IDLE)
    unit._ace.connected = False
    gcmd = FakeGcmd()
    unit.cmd_ACE_STATUS(gcmd)
    assert gcmd.responses == ["ACE not connected"]


# ── get_status (Moonraker-queryable unit state) ───────────────────────────────

def _status_obj_unit(hw=None, action="", inventory=None, connected=True):
    unit = afcACE.__new__(afcACE)
    unit.lanes = {}          # empty -> base get_status returns empty aggregates
    unit._cached_hw_status = hw or {}
    unit._cached_temp_info = {}
    unit._hw_status_time = None      # no heartbeat yet -> age/stale unset
    unit._current_action = action
    unit._ace = FakeAce(connected=connected)
    unit._slot_inventory = (inventory if inventory is not None
                            else [{} for _ in range(afcACE.SLOTS_PER_UNIT)])
    return unit


def test_get_status_adds_ace_state():
    hw = {"status": "busy", "temp": 28, "dryer_status": {"status": "stop"}}
    inv = ([{"status": "ready", "rfid": 2, "sku": "HPL19-107",
             "material": "PLA", "uid": "BB2613B0102474", "color": [137, 168, 79]}]
           + [{} for _ in range(afcACE.SLOTS_PER_UNIT - 1)])
    unit = _status_obj_unit(hw=hw, action="feeding(slot 0)", inventory=inv)

    st = unit.get_status()

    # base structure preserved
    assert st["lanes"] == [] and "hubs" in st and "buffers" in st
    # ACE live state
    assert st["ace_connected"] is True
    assert st["ace_status"] == "busy"
    assert st["ace_action"] == "feeding(slot 0)"
    assert st["ace_temp"] == 28
    assert st["ace_dryer"] == "stop"
    assert len(st["ace_slots"]) == afcACE.SLOTS_PER_UNIT
    s0 = st["ace_slots"][0]
    assert (s0["sku"], s0["material"], s0["uid"], s0["rfid"]) == \
        ("HPL19-107", "PLA", "BB2613B0102474", 2)
    assert s0["color"] == [137, 168, 79]


def test_get_status_humidity_only_when_present():
    ace2 = _status_obj_unit(hw={"status": "ready", "temp": 26, "humidity": 31})
    assert ace2.get_status()["ace_humidity"] == 31
    v1 = _status_obj_unit(hw={"status": "ready", "temp": 26})  # V1 omits humidity
    assert "ace_humidity" not in v1.get_status()


def test_get_status_disconnected_and_empty():
    unit = _status_obj_unit(connected=False)
    st = unit.get_status()
    assert st["ace_connected"] is False
    assert st["ace_action"] == ""
    assert st["ace_temp"] is None
    assert len(st["ace_slots"]) == afcACE.SLOTS_PER_UNIT


def test_get_status_falls_back_to_temp_cache_for_ace2():
    # ACE2's get_status payload has no temp/humidity (they arrive via get_temp
    # into _cached_temp_info); env channels must still surface.
    unit = _status_obj_unit(hw={"status": "ready"})   # no temp/humidity in status
    unit._cached_temp_info = {"env_temp": 24.5, "env_humidity": 38}
    st = unit.get_status()
    assert st["ace_temp"] == 24.5
    assert st["ace_humidity"] == 38


def test_get_status_reports_stale_when_cache_ages():
    unit = _status_obj_unit(hw={"status": "ready"})
    unit.afc = types.SimpleNamespace(
        reactor=types.SimpleNamespace(monotonic=lambda: 100.0))
    unit._hw_status_time = 100.0                       # fresh -> not stale
    assert unit.get_status()["ace_status_stale"] is False
    unit._hw_status_time = 100.0 - 20.0                # 20s old (> 3 heartbeats)
    st = unit.get_status()
    assert st["ace_status_stale"] is True
    assert st["ace_status_age"] >= 20.0


# ── ACE2 _decode_status now indexes slots (so _derive_action can tag them) ─────

from extras.AFC_ACE2 import _decode_status, pb_uint32  # noqa: E402


def test_ace2_decode_status_indexes_and_tags_busy_slot():
    # One real slot: slot_state=1 (feeding), filament_state=1 (present); padded.
    slot0 = pb_uint32(1, 1) + pb_uint32(2, 1)
    status = _decode_status({9: [(2, slot0)]})
    assert [s["index"] for s in status["slots"]] == [0, 1, 2, 3]
    assert status["slots"][0]["slot_status"] == "feeding"
    # The whole point: the ACE2 action is now slot-tagged like the V1's.
    assert afcACE._derive_action(status) == "feeding(slot 0)"


def test_ace2_decode_status_pads_slots_with_index():
    status = _decode_status({})          # no field-9 slots -> 4 padded
    assert [s["index"] for s in status["slots"]] == [0, 1, 2, 3]
