"""
Unit tests for afcACE._sync_slot_states in extras/AFC_ACE.py

Style: typed fakes (tests/ace_helpers.py) instead of MagicMock, full state
verification after every call, branch-complete coverage of the hardware-poll
state machine:

  - slot inventory status copy
  - lane-not-in-slots skip, malformed slot entries
  - virtual-hub occupancy refresh (from tool_loaded) every poll
  - transient statuses (shifting/feeding/unwinding) leave everything alone
  - ready/empty -> prep_state; genuine ready->empty clears staging and fires
    the removal (runout) event; unit-busy and stale-snapshot suppression
  - empty->ready fresh insert: V1 preload-to-hub honoring load_to_hub,
    ACE2 (no preload) leaves staging to prep_post_load
  - ready + prep-done + lane stuck in NONE: TOOLED restore for the current
    tool (active spool, LEDs, buffer, assist) and for an idle tool; the
    insert path for an un-tooled lane incl. suppressed-load marking
  - previous-state snapshot only updates on non-transient statuses
"""

from __future__ import annotations

from extras.AFC_ACE import afcACE
from extras.AFC_lane import AFCLaneState

from tests.ace_helpers import (
    FakeAFC,
    FakeExtruderObj,
    FakeHub,
    FakeLane,
    FakeLogger,
    Recorder,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _lane(name="lane0", prev_ready=None, tool_loaded=False,
          status=AFCLaneState.NONE, lane_loaded=None, hub=None):
    lane = FakeLane(name, extruder_obj=FakeExtruderObj("extruder",
                                                       lane_loaded=lane_loaded),
                    hub_obj=hub, tool_loaded=tool_loaded, status=status)
    lane.prep_state = bool(prev_ready)
    lane.loaded_to_hub = bool(prev_ready)
    lane.load_to_hub = True
    return lane


def _make_unit(lane, prev_ready=None, preloads=False, stale=False,
               current=None):
    unit = afcACE.__new__(afcACE)
    unit.logger = FakeLogger()
    unit.afc = FakeAFC()
    unit.afc.current = current
    unit.lanes = {lane.name: lane}
    unit._slot_map = {lane.name: 0}
    unit._slot_inventory = [{} for _ in range(afcACE.SLOTS_PER_UNIT)]
    unit._prev_slot_states = {} if prev_ready is None else {lane.name: prev_ready}
    unit._prev_states_stale = stale
    unit._hub_load_suppressed = set()
    unit._preloads_to_hub_on_insert = preloads
    unit._use_feed_assist = Recorder(result=False)
    unit._start_feed_assist = Recorder()
    unit.lane_tool_loaded = Recorder()
    unit.lane_tool_loaded_idle = Recorder()
    # afc collaborators the TOOLED-restore path touches
    unit.afc.spool = Recorder()
    unit.afc.spool.set_active_spool = Recorder()
    lane.spool_id = 42
    return unit


def _hw(slot_status, unit_status="ready"):
    return {"status": unit_status, "slots": [{"status": slot_status}]}


# ── Inventory + malformed input ───────────────────────────────────────────────

def test_inventory_status_copied_per_slot():
    lane = _lane(prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states({"status": "ready",
                            "slots": [{"status": "ready"}, {"status": "empty"}]})

    assert unit._slot_inventory[0]["status"] == "ready"
    assert unit._slot_inventory[1]["status"] == "empty"


def test_lane_beyond_reported_slots_skipped():
    lane = _lane(prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states({"status": "ready", "slots": []})

    assert lane.prep_state is True            # untouched
    assert not lane.handle_load_runout.called


def test_malformed_slot_entry_skipped():
    lane = _lane(prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states({"status": "ready", "slots": ["garbage"]})

    assert lane.prep_state is True
    assert not lane.handle_load_runout.called


# ── Virtual hub refresh ───────────────────────────────────────────────────────

def test_virtual_hub_refreshed_from_tool_loaded():
    lane = _lane(prev_ready=True, tool_loaded=True, status=AFCLaneState.TOOLED,
                 hub=FakeHub(virtual=True))
    unit = _make_unit(lane, prev_ready=True)
    lane._load_state = False  # stale

    unit._sync_slot_states(_hw("ready"))

    assert lane._load_state is True  # derived from tool_loaded every poll


def test_real_hub_load_state_untouched():
    lane = _lane(prev_ready=True, tool_loaded=True, status=AFCLaneState.TOOLED,
                 hub=None)
    unit = _make_unit(lane, prev_ready=True)
    lane._load_state = "sentinel"

    unit._sync_slot_states(_hw("ready"))

    assert lane._load_state == "sentinel"


# ── Transient statuses ────────────────────────────────────────────────────────

def test_transient_status_leaves_everything_alone():
    lane = _lane(prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    for status in ("shifting", "feeding", "unwinding"):
        unit._sync_slot_states(_hw(status))
        assert lane.prep_state is True                  # untouched
        assert lane.loaded_to_hub is True               # untouched
        assert unit._prev_slot_states["lane0"] is True  # snapshot untouched
        assert not lane.handle_load_runout.called


# ── Removal / runout ──────────────────────────────────────────────────────────

def test_ready_to_empty_fires_runout_and_clears_staging():
    lane = _lane(prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("empty"))

    assert lane.prep_state is False
    assert lane.loaded_to_hub is False
    assert lane.handle_load_runout.call_count == 1
    eventtime, state = lane.handle_load_runout.last_args
    assert state is False
    assert unit._prev_slot_states["lane0"] is False


def test_unit_busy_suppresses_removal():
    """ACE2 flickers slots 'empty' while its own cycles run — a unit-level
    'busy' must not fire a false runout or drop the staged state."""
    lane = _lane(prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("empty", unit_status="busy"))

    assert not lane.handle_load_runout.called
    assert lane.loaded_to_hub is True         # staged state survives
    assert lane.prep_state is False           # live prep still tracks the slot


def test_stale_prev_states_resync_without_events():
    lane = _lane(prev_ready=True)
    unit = _make_unit(lane, prev_ready=True, stale=True)

    unit._sync_slot_states(_hw("empty"))

    assert not lane.handle_load_runout.called
    assert unit._prev_slot_states["lane0"] is False  # re-synced
    assert unit._prev_states_stale is False          # consumed
    assert lane.loaded_to_hub is True                # not cleared on resync


def test_empty_stays_empty_no_event():
    lane = _lane(prev_ready=False)
    unit = _make_unit(lane, prev_ready=False)

    unit._sync_slot_states(_hw("empty"))

    assert not lane.handle_load_runout.called
    assert unit._prev_slot_states["lane0"] is False


# ── Insert / staging ──────────────────────────────────────────────────────────

def test_fresh_insert_v1_preloads_to_hub():
    """V1 ACE preloads filament to the hub on insert: empty -> ready stages
    the lane (honoring load_to_hub)."""
    lane = _lane(prev_ready=False)
    unit = _make_unit(lane, prev_ready=False, preloads=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is True


def test_fresh_insert_respects_load_to_hub_off():
    lane = _lane(prev_ready=False)
    lane.load_to_hub = False
    unit = _make_unit(lane, prev_ready=False, preloads=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is False


def test_fresh_insert_ace2_does_not_preload():
    """ACE2 stages via prep_post_load's real dist_hub feed instead."""
    lane = _lane(prev_ready=False)
    unit = _make_unit(lane, prev_ready=False, preloads=False)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is False


def test_tool_loaded_lane_never_preloaded():
    lane = _lane(prev_ready=False, tool_loaded=True, status=AFCLaneState.TOOLED)
    lane.loaded_to_hub = False
    unit = _make_unit(lane, prev_ready=False, preloads=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.loaded_to_hub is False


# ── Insert path for an un-tooled NONE lane ────────────────────────────────────

def test_ready_untooled_lane_fires_insert_path():
    lane = _lane(prev_ready=True, tool_loaded=False, status=AFCLaneState.NONE)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.handle_load_runout.call_count == 1
    _, state = lane.handle_load_runout.last_args
    assert state is True
    assert lane._load_suppressed is False


def test_ready_untooled_suppressed_marks_lane():
    lane = _lane(prev_ready=True, tool_loaded=False, status=AFCLaneState.NONE)
    unit = _make_unit(lane, prev_ready=True)
    unit._hub_load_suppressed = {"lane0"}

    unit._sync_slot_states(_hw("ready"))

    assert lane._load_suppressed is True
    assert lane.handle_load_runout.call_count == 1


def test_prep_not_done_skips_insert_path():
    lane = _lane(prev_ready=True, tool_loaded=False, status=AFCLaneState.NONE)
    lane._afc_prep_done = False
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("ready"))

    assert not lane.handle_load_runout.called


# ── TOOLED restore ────────────────────────────────────────────────────────────

def test_ready_tooled_current_lane_restores_full_state():
    lane = _lane(prev_ready=True, tool_loaded=True, status=AFCLaneState.NONE,
                 lane_loaded="lane0")
    unit = _make_unit(lane, prev_ready=True, current="lane0")
    unit._use_feed_assist = Recorder(result=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.loaded_to_hub is True
    assert lane.sync_to_extruder.call_count == 1
    assert lane.status == AFCLaneState.TOOLED
    assert unit.afc.spool.set_active_spool.calls == [((42,), {})]
    assert unit.lane_tool_loaded.calls == [((lane,), {})]
    assert not unit.lane_tool_loaded_idle.called
    assert lane.enable_buffer.call_count == 1
    assert unit._start_feed_assist.calls == [((0,), {})]
    assert not lane.handle_load_runout.called


def test_ready_tooled_idle_lane_restores_idle_state():
    """A tool-loaded lane on a NOT-current tool restores as idle-tooled (and
    is marked TOOLED so the restore doesn't re-fire every poll)."""
    lane = _lane(prev_ready=True, tool_loaded=True, status=AFCLaneState.NONE,
                 lane_loaded="lane0")
    unit = _make_unit(lane, prev_ready=True, current="other_lane")

    unit._sync_slot_states(_hw("ready"))

    assert unit.lane_tool_loaded_idle.calls == [((lane,), {})]
    assert not unit.lane_tool_loaded.called
    assert not unit.afc.spool.set_active_spool.called
    assert lane.status == AFCLaneState.TOOLED
    assert lane.enable_buffer.call_count == 1


def test_restore_does_not_refire_once_tooled():
    lane = _lane(prev_ready=True, tool_loaded=True, status=AFCLaneState.TOOLED,
                 lane_loaded="lane0")
    unit = _make_unit(lane, prev_ready=True, current="lane0")

    unit._sync_slot_states(_hw("ready"))

    assert not lane.sync_to_extruder.called
    assert not unit.lane_tool_loaded.called
