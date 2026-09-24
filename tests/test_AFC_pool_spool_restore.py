# A pooled lane carries no spool, and a claimed one is PREP-done.
#
# deactivate_to_pool() returns a released lane to the inert pool state: no
# map, no spool, none of its profile, runout lane or TD-1 data. What the lane
# held is not kept on the lane: AFC_BridgeBox holds the lane's record for the
# unit it was released from (see test_AFC_bridgebox_lane_records.py, where an
# HT re-plug brings Spoolman spool 136 back).
#
# activate_from_pool() gives a claimed lane the PREP-done mark PREP gives the
# lanes it walks: CHANGE_TOOL unloads the loaded lane first only for a target
# lane that carries it.
from __future__ import annotations

import types

from extras.AFC_BridgeBox import activate_from_pool, deactivate_to_pool
from extras.AFC_lane import AFCLane


def _live_lane(spool_id=136, color="#0086D6", material="PLA", weight=750.0,
               prep_done=True):
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
    lane.afc = types.SimpleNamespace(lanes={"lane28": None}, spoolman=None,
                                     prep_done=prep_done)
    lane.is_direct_hub = lambda: False
    lane.map = ["T28"]
    lane._map = []
    lane.current_map = "T28"
    lane._load_state = True
    lane.spool_id = spool_id
    lane._material = material
    lane.color = color
    lane.weight = weight
    lane._afc_prep_done = False
    return lane


def test_release_still_clears_the_lane():
    # A lane in the pool carries no spool, and keeps no copy of one.
    lane = _live_lane()
    deactivate_to_pool(lane)
    assert lane.spool_id is None
    assert lane.color == ""
    assert lane.weight == 0.
    assert lane.map == [] and lane.current_map == ""
    assert "_pool_spool" not in vars(lane)
    # _load_state, not the read-only load_state property: an AttributeError
    # there would be swallowed by the caller and leave the lane assigned.
    assert lane.raw_load_state is False
    assert lane.unassigned is True


def test_release_leaves_nothing_of_the_spool_for_the_next_claimant():
    # A different unit claiming the bay next must not start from this
    # spool's variant, temperatures, runout lane or TD-1 data. Its own unit
    # gets them back from the record held for it.
    lane = _live_lane()
    lane.sub_type, lane.extruder_temp, lane.bed_temp = "Matte", 225.0, 60.0
    lane.multi_color, lane.spool_vendor = ["FF0000", "00FF00"], "Bambu Lab"
    lane.filament_name, lane.runout_lane = "Bambu PLA Matte", "lane29"
    lane.td1_data, lane.need_purge = {"td": 1.2}, True
    deactivate_to_pool(lane)
    assert (lane.sub_type, lane.extruder_temp, lane.bed_temp) == (
        "", None, None)
    assert (lane.multi_color, lane.spool_vendor, lane.filament_name) == (
        [], "", "")
    assert (lane.runout_lane, lane.td1_data, lane.need_purge) == (
        None, {}, False)
    assert lane.color == "" and lane.material is None


def test_release_gives_the_lane_back_its_configured_tare_and_profile():
    # A Spoolman link or a record set the tare, density and diameter, and
    # the tag the SKU: the next claimant starts from the lane's config, as
    # AFC_lane reads it at startup.
    lane = _live_lane()
    lane.empty_spool_weight, lane.filament_density = 250.0, 1.27
    lane.filament_diameter, lane.bambu_sku = 2.85, "GFA00"
    cfg = {"empty_spool_weight": 210.0}
    lane._config = types.SimpleNamespace(
        getfloat=lambda key, default: cfg.get(key, default))
    deactivate_to_pool(lane)
    assert (lane.empty_spool_weight, lane.filament_density,
            lane.filament_diameter, lane.bambu_sku) == (210.0, 1.24, 1.75, "")


def test_a_claim_after_prep_marks_the_lane_prep_done():
    lane = _live_lane()
    deactivate_to_pool(lane)
    activate_from_pool(lane)
    assert lane.unassigned is False
    assert lane._afc_prep_done is True


def test_a_claim_before_prep_leaves_the_mark_to_prep():
    lane = _live_lane(prep_done=False)
    deactivate_to_pool(lane)
    activate_from_pool(lane)
    assert lane._afc_prep_done is False


def test_activation_brings_no_spool_back():
    lane = _live_lane()
    deactivate_to_pool(lane)
    activate_from_pool(lane)
    assert (lane.spool_id, lane.color, lane.weight) == (None, "", 0.)


class _LaneConfig:
    """The lane's own config: what its density, diameter and tare read as."""

    def __init__(self, **opts):
        self._opts = opts

    def getfloat(self, option, default=None, minval=None, **_kw):
        return float(self._opts.get(option, default))


def test_release_leaves_none_of_the_spools_details():
    # A PC spool on the released unit must not set the load temperature, the
    # variant or the weight maths for whichever unit claims the bay next.
    lane = _live_lane()
    lane._config = _LaneConfig(filament_density=1.20)
    lane.extruder_temp = 280
    lane.bed_temp = 110
    lane.sub_type = "CF"
    lane.filament_name = "PC-CF"
    lane.spool_vendor = "Bambu"
    lane.bambu_sku = "GFC01"
    lane.filament_density = 1.3
    lane.filament_diameter = 2.85
    lane.empty_spool_weight = 250.0
    deactivate_to_pool(lane)
    assert (lane.extruder_temp, lane.bed_temp) == (None, None)
    assert (lane.sub_type, lane.filament_name, lane.spool_vendor,
            lane.bambu_sku) == ("", "", "", "")
    assert (lane.filament_density, lane.filament_diameter,
            lane.empty_spool_weight) == (1.20, 1.75, 190.0)
