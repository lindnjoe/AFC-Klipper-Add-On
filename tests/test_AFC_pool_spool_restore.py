# A bay HELD for a re-plug must come back with the spool that was in it.
#
# AFC_BridgeBox releases a unit whose UID goes quiet for longer than the grace
# (every bridge OTA flash does exactly that -- the USB device drops for the
# reboot), and re-claims it seconds later onto the same lanes: "slot kept for
# re-plug". deactivate_to_pool() wipes spool_id/material/color/weight on the
# way out, and nothing put them back, so the re-claimed lane came up blank.
#
# Measured on the HT: released 00:50:24, re-claimed 00:50:45, and lane28 sat
# with no spool for 75 minutes -- Mainsail drew the spool black with tooltip
# "#0", and three measurements in a row reported "not linked to a Spoolman
# spool" about a reel Spoolman held all along (spool 136, card_uids 4B8E44F6).
# It only recovered when the operator pulled and re-seated the spool, which
# forced a tag re-read.
#
# The var file cannot cover this: save_vars() walks unit_obj.lanes, and the
# release pops the lane out of it, so the lane's record is gone from
# AFC.var.unit within a second. The release snapshot is the only copy.
from __future__ import annotations

import types

from extras.AFC_BridgeBox import (activate_from_pool, deactivate_to_pool,
                                  restore_pool_spool)
from extras.AFC_lane import AFCLane


def _live_lane(spool_id=136, color="#0086D6", material="PLA", weight=750.0):
    """A claimed, spool-carrying lane -- the state a release starts from."""
    lane = AFCLane.__new__(AFCLane)
    lane.name = "lane28"
    lane.fullname = "AFC_lane lane28"
    lane.unassigned = False
    lane.buffer_obj = None
    lane.buffer_name = None
    lane.hub_obj = None
    lane.extruder_obj = None
    lane.unit_obj = types.SimpleNamespace(lanes={"lane28": None},
                                          type="AFC_BambuAMS")
    lane.afc = types.SimpleNamespace(lanes={"lane28": None}, spoolman=None)
    lane.is_direct_hub = lambda: False
    lane.map = ["T28"]
    lane._map = []
    lane.current_map = "T28"
    lane._load_state = True
    lane.spool_id = spool_id
    lane._material = material
    lane.color = color
    lane.weight = weight
    return lane


def test_release_still_clears_the_lane():
    # The wipe is not the bug and must stay: a lane in the pool carries no
    # spool. The snapshot is taken beside it, not instead of it.
    lane = _live_lane()
    deactivate_to_pool(lane)
    assert lane.spool_id is None
    assert lane.color == ""
    assert lane.weight == 0.
    # Reached at all only because the wipe no longer dies one line earlier on
    # `self.load_state = False` -- load_state is a read-only property, and the
    # caller's bare `except Exception: pass` hid the AttributeError, so every
    # released lane went on calling itself assigned.
    assert lane.raw_load_state is False
    assert lane.unassigned is True


def test_a_replug_gets_its_spool_back():
    lane = _live_lane()
    deactivate_to_pool(lane)
    activate_from_pool(lane)
    assert restore_pool_spool(lane) is True
    assert lane.spool_id == 136
    assert lane.color == "#0086D6"
    assert lane._material == "PLA"
    assert lane.weight == 750.0


def test_the_snapshot_is_consumed_once():
    # A second claim on the same slot by a different unit must not resurrect
    # the first unit's reels; the restore empties the snapshot behind it.
    lane = _live_lane()
    deactivate_to_pool(lane)
    assert restore_pool_spool(lane) is True
    deactivate_to_pool(lane)
    lane._pool_spool = None                 # what the claim path does for a
    assert restore_pool_spool(lane) is False   # DIFFERENT uid on this slot
    assert lane.spool_id is None


def test_a_lane_that_never_held_a_spool_restores_cleanly():
    lane = _live_lane(spool_id=None, color="", material=None, weight=0.)
    deactivate_to_pool(lane)
    assert restore_pool_spool(lane) is True   # a snapshot exists, it is empty
    assert lane.spool_id is None
    assert lane.color == ""


def test_restore_rebinds_through_spoolman_when_configured():
    # Not a bare attribute write: AFC_prep re-binds a restored spool_id through
    # set_spoolID so the lane refreshes from Spoolman, and a re-claim is the
    # same situation as a boot.
    seen = []
    lane = _live_lane()
    lane.afc.spoolman = "http://localhost:7912"
    lane.afc.spool = types.SimpleNamespace(
        set_spoolID=lambda ln, sid, save_vars=True: seen.append(
            (ln.name, sid, save_vars)))
    deactivate_to_pool(lane)
    restore_pool_spool(lane)
    assert seen == [("lane28", 136, False)]


def test_no_snapshot_means_no_restore():
    lane = _live_lane()
    assert restore_pool_spool(lane) is False
