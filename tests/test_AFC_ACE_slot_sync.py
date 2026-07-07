"""
Unit tests for afcACE._sync_slot_states in extras/AFC_ACE.py

Covers the hardware-poll state machine that drives lane prep state,
insert/removal (runout) events, and tooled-state restore:
  - ready -> empty transition fires handle_load_runout(False) and clears
    the staged (loaded_to_hub) state
  - unit-level 'busy' suppresses removal handling (ACE2 startup flicker)
  - a stale previous-state snapshot (resync after an operation) suppresses
    false insert/remove events and just re-syncs
  - transient slot statuses (shifting/feeding/unwinding) are ignored
  - empty -> ready on a V1 unit preloads the lane to the hub
  - a ready slot with a lane stuck in NONE restores the TOOLED state
  - a ready slot with an un-tooled lane fires the insert path
"""

from __future__ import annotations

from unittest.mock import MagicMock

from extras.AFC_ACE import afcACE
from extras.AFC_lane import AFCLaneState


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_lane(name, prev_ready=None, tool_loaded=False, status=AFCLaneState.NONE):
    lane = MagicMock()
    lane.name = name
    lane.hub_obj = None            # real _is_virtual_hub -> False
    lane.tool_loaded = tool_loaded
    lane.status = status
    lane.prep_state = bool(prev_ready)
    lane.loaded_to_hub = bool(prev_ready)
    lane.load_to_hub = True
    lane._afc_prep_done = True
    lane._load_suppressed = False
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.lane_loaded = None
    return lane


def _make_unit(lane, prev_ready=None, preloads=False, stale=False):
    unit = afcACE.__new__(afcACE)
    unit.logger = MagicMock()
    unit.afc = MagicMock()
    unit.afc.current = None
    unit.afc.reactor.monotonic.return_value = 123.0
    unit.lanes = {lane.name: lane}
    unit._slot_map = {lane.name: 0}
    unit._slot_inventory = [{} for _ in range(afcACE.SLOTS_PER_UNIT)]
    unit._prev_slot_states = {} if prev_ready is None else {lane.name: prev_ready}
    unit._prev_states_stale = stale
    unit._hub_load_suppressed = set()
    unit._preloads_to_hub_on_insert = preloads
    unit._use_feed_assist = MagicMock(return_value=False)
    unit._start_feed_assist = MagicMock()
    unit.lane_tool_loaded = MagicMock()
    unit.lane_tool_loaded_idle = MagicMock()
    return unit


def _hw(slot_status, unit_status="ready"):
    return {"status": unit_status, "slots": [{"status": slot_status}]}


# ── Removal / runout ──────────────────────────────────────────────────────────

def test_ready_to_empty_fires_runout_and_clears_staging():
    lane = _make_lane("lane0", prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("empty"))

    lane.handle_load_runout.assert_called_once_with(123.0, False)
    assert lane.prep_state is False
    assert lane.loaded_to_hub is False
    assert unit._prev_slot_states["lane0"] is False


def test_unit_busy_suppresses_removal():
    """ACE2 flickers slots 'empty' while its own cycles run — a unit-level
    'busy' must not fire a false runout or drop the staged state."""
    lane = _make_lane("lane0", prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("empty", unit_status="busy"))

    lane.handle_load_runout.assert_not_called()
    assert lane.loaded_to_hub is True  # staged state survives the flicker


def test_stale_prev_states_resync_without_events():
    """First poll after a load/unload op: _prev_states_stale means the previous
    snapshot is meaningless — re-sync it without firing insert/remove."""
    lane = _make_lane("lane0", prev_ready=True)
    unit = _make_unit(lane, prev_ready=True, stale=True)

    unit._sync_slot_states(_hw("empty"))

    lane.handle_load_runout.assert_not_called()
    assert unit._prev_slot_states["lane0"] is False  # re-synced
    assert unit._prev_states_stale is False          # consumed


def test_empty_stays_empty_no_event():
    lane = _make_lane("lane0", prev_ready=False)
    unit = _make_unit(lane, prev_ready=False)

    unit._sync_slot_states(_hw("empty"))

    lane.handle_load_runout.assert_not_called()


def test_transient_status_is_ignored():
    lane = _make_lane("lane0", prev_ready=True)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("feeding"))

    lane.handle_load_runout.assert_not_called()
    assert lane.prep_state is True                   # untouched
    assert unit._prev_slot_states["lane0"] is True   # untouched


# ── Insert ────────────────────────────────────────────────────────────────────

def test_fresh_insert_v1_preloads_to_hub():
    """V1 ACE preloads filament to the hub on insert: empty -> ready stages
    the lane (honoring load_to_hub)."""
    lane = _make_lane("lane0", prev_ready=False)
    unit = _make_unit(lane, prev_ready=False, preloads=True)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is True


def test_fresh_insert_ace2_does_not_preload():
    """ACE2 stages via prep_post_load's real dist_hub feed instead."""
    lane = _make_lane("lane0", prev_ready=False)
    unit = _make_unit(lane, prev_ready=False, preloads=False)

    unit._sync_slot_states(_hw("ready"))

    assert lane.prep_state is True
    assert lane.loaded_to_hub is False


def test_ready_untooled_lane_fires_insert_path():
    """Slot ready, prep done, lane stuck in NONE and not tool-loaded:
    the insert handler runs (handle_load_runout with True)."""
    lane = _make_lane("lane0", prev_ready=True, tool_loaded=False,
                      status=AFCLaneState.NONE)
    unit = _make_unit(lane, prev_ready=True)

    unit._sync_slot_states(_hw("ready"))

    lane.handle_load_runout.assert_called_once_with(123.0, True)


# ── Tooled-state restore ──────────────────────────────────────────────────────

def test_ready_tooled_lane_restores_state():
    lane = _make_lane("lane0", prev_ready=True, tool_loaded=True,
                      status=AFCLaneState.NONE)
    lane.extruder_obj.lane_loaded = "lane0"
    unit = _make_unit(lane, prev_ready=True)
    unit.afc.current = "lane0"

    unit._sync_slot_states(_hw("ready"))

    assert lane.loaded_to_hub is True
    lane.sync_to_extruder.assert_called_once()
    assert lane.status == AFCLaneState.TOOLED
    unit.lane_tool_loaded.assert_called_once_with(lane)
    lane.enable_buffer.assert_called_once()
    lane.handle_load_runout.assert_not_called()


def test_ready_tooled_idle_lane_restores_idle_state():
    """A tool-loaded lane on a NOT-current tool restores as idle-tooled
    (and is marked TOOLED so the restore doesn't re-fire every poll)."""
    lane = _make_lane("lane0", prev_ready=True, tool_loaded=True,
                      status=AFCLaneState.NONE)
    lane.extruder_obj.lane_loaded = "lane0"
    unit = _make_unit(lane, prev_ready=True)
    unit.afc.current = "other_lane"

    unit._sync_slot_states(_hw("ready"))

    unit.lane_tool_loaded_idle.assert_called_once_with(lane)
    assert lane.status == AFCLaneState.TOOLED
