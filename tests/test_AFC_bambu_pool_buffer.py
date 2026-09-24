# A lane claimed out of the pool must publish its buffer NAME, not just hold
# the object.
#
# activate_from_pool documents itself as "the exact mirror of the
# add_to_other_obj block in handle_connect". That block ends with
#   self.buffer_name = self.buffer_obj.name
# the back-fill for a lane whose buffer comes from its UNIT rather than its own
# config -- which is every lane AFC_BridgeBox fabricates. The mirror was missing
# exactly that line, so a claimed lane ran with a working buffer_obj and
# buffer_name None. Nothing misbehaved (buffer_status() keys off the object),
# but get_status publishes the NAME, so Mainsail's AFC panel rendered the
# buffer as "-:-" on a unit that was using it -- seen on bridgebox with lane28
# loaded and following while its unit reported buff 31 / expanded.
from __future__ import annotations

import types

from extras.AFC_BridgeBox import (activate_from_pool, deactivate_to_pool,
                                  restore_pool_spool)
from extras.AFC_lane import AFCLane


class _Buffer:
    def __init__(self, name="Bambu_AMS_Buffer"):
        self.name = name
        self.lanes = {}


def _pool_lane(buffer_obj, buffer_name=None):
    """A lane in the state activate_from_pool is called on."""
    lane = AFCLane.__new__(AFCLane)
    lane.name = "lane28"
    lane.fullname = "AFC_lane lane28"
    lane.unassigned = True
    lane.buffer_obj = buffer_obj
    lane.buffer_name = buffer_name
    lane.hub_obj = None
    lane.extruder_obj = None
    lane.unit_obj = types.SimpleNamespace(lanes={}, type="AFC_BambuAMS")
    lane.afc = types.SimpleNamespace(lanes={})
    lane.is_direct_hub = lambda: False
    return lane


def test_a_claimed_lane_publishes_its_units_buffer_name():
    buf = _Buffer()
    lane = _pool_lane(buf)
    activate_from_pool(lane)
    assert lane.buffer_name == "Bambu_AMS_Buffer"
    assert lane.name in buf.lanes            # and the registry write still happens
    assert lane.unassigned is False


def test_a_lane_with_its_own_buffer_name_keeps_it():
    # An explicit per-lane buffer: overrides the unit's, and the back-fill must
    # only fill a gap, never overwrite.
    lane = _pool_lane(_Buffer(), buffer_name="MyOwnBuffer")
    activate_from_pool(lane)
    assert lane.buffer_name == "MyOwnBuffer"


def test_a_lane_with_no_buffer_at_all_is_left_alone():
    # Valid configuration -- AFC_lane says so in as many words -- and it must
    # not raise on the way through.
    lane = _pool_lane(None)
    activate_from_pool(lane)
    assert lane.buffer_name is None
    assert lane.unassigned is False


def test_the_mirror_is_complete():
    # Guards the claim the docstring makes: every assignment handle_connect's
    # add_to_other_obj block ends with should exist in activate_from_pool too.
    import inspect
    mirror = inspect.getsource(activate_from_pool)
    assert "buffer_name = lane.buffer_obj.name" in mirror, \
        "activate_from_pool must mirror handle_connect's buffer_name back-fill"
