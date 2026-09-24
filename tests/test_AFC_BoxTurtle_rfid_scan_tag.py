"""
Scan Tag on Box Turtle lanes (extras/AFC_BoxTurtle_rfid.py).

The BridgeBox display offers Scan Tag on a Box Turtle lane when the printer
lists [AFC_BoxTurtle_rfid], and sends AFC_BT_RFID_STAGE LANE=. Moonraker only
lists objects with a get_status, so the module reports its lane map.

The scan turns the spool by feeding filament, up to two turns of a full reel,
which from a lane staged at the hub, or into a hub another lane's filament is
through, drives the tip where it must not go. So a staged lane is brought back
onto its load switch first and re-staged afterwards, a lane in the toolhead is
refused, and a busy hub bounds the sweep to the room before it.
"""

from __future__ import annotations

import types

import pytest

from extras.AFC_rfid_write import StageError
from tests.test_AFC_BoxTurtle_rfid import _FakeLane, _Gcmd, _reader, _shim


class _Hub:
    def __init__(self, name="Turtle_1", state=False, virtual=False):
        self.name = name
        self.state = state
        self._virtual = virtual

    def is_virtual_pin(self):
        return self._virtual


class _Unit:
    """The two insert moves: prep_load homes onto the load switch, and
    prep_post_load feeds dist_hub and marks the lane staged."""

    def __init__(self):
        self.calls = []

    def prep_load(self, lane):
        self.calls.append("prep_load")
        lane.move_to(100.0, "short", endstop=lane.load_es, use_homing=True)

    def prep_post_load(self, lane):
        self.calls.append("prep_post_load")
        if not lane.loaded_to_hub and lane.raw_load_state:
            lane.move(lane.dist_hub, 100.0, 400)
            lane.loaded_to_hub = True


def _lane(name, hub, dist_hub=150.0, staged=False, tool_loaded=False):
    lane = _FakeLane(name)
    lane.hub_obj = hub
    lane.dist_hub = dist_hub
    lane.loaded_to_hub = staged
    lane.tool_loaded = tool_loaded
    lane.unit_obj = _Unit()
    if staged:
        lane.pos += dist_hub
    return lane


def _setup(staged=False, busy=False, dist_hub=150.0, tag=True, homing=True):
    hub = _Hub()
    lane = _lane("lane9", hub, dist_hub=dist_hub, staged=staged)
    lanes = {"lane9": lane}
    if busy:
        lanes["lane8"] = _lane("lane8", hub, tool_loaded=True)
        hub.state = True
    u = _shim([_reader("reader0", ["lane8", "lane9"])], lanes=lanes)
    u.afc.homing_enabled = homing
    u.tag_retract_speed = 0.0
    swept = {}

    def sweep(ln_obj, ln, rdr, advance, step, speed):
        swept["advance"] = advance
        swept["start_pos"] = ln_obj.pos
        swept["staged_during"] = ln_obj.loaded_to_hub
        fed = min(advance, 120.0)
        ln_obj.move(fed, speed, 400)
        return ({"uid": "cafef00d"} if tag else None), fed

    u._sweep_for_tag = sweep
    u._clear_sibling = lambda ln, rdr: None
    u._restore_sibling = lambda token: None
    u.applied = []
    u._apply = lambda ln, t, gcmd: u.applied.append(ln)
    return u, lane, swept


# ── Moonraker lists it, so the display offers Scan Tag ─────────────────────

def test_get_status_names_the_reader_for_each_lane():
    u = _shim([_reader("reader0", ["lane8", "lane9"]),
               _reader("reader1", ["lane10", "lane11"])])
    st = u.get_status()
    assert st["lane_slot_map"] == {"lane8": "reader0", "lane9": "reader0",
                                   "lane10": "reader1", "lane11": "reader1"}
    assert st["readers"] == {"reader0": True, "reader1": True}


# ── Refusals ────────────────────────────────────────────────────────────────

def test_a_lane_in_the_toolhead_is_refused():
    u, lane, swept = _setup()
    lane.tool_loaded = True
    with pytest.raises(RuntimeError, match="loaded in the toolhead"):
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane9"))
    assert "advance" not in swept and not lane.moves


def test_a_staged_lane_without_homing_is_refused():
    u, lane, swept = _setup(staged=True, homing=False)
    with pytest.raises(RuntimeError, match="homing is off"):
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane9"))
    assert "advance" not in swept and not lane.moves


def test_no_room_before_a_busy_hub_is_refused():
    u, lane, swept = _setup(busy=True, dist_hub=20.0)
    with pytest.raises(RuntimeError, match="no room before it"):
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane9"))
    assert "advance" not in swept


# ── A staged lane: back to the load switch, scan, re-stage ─────────────────

def test_a_staged_lane_is_scanned_from_its_load_switch_and_restaged():
    u, lane, swept = _setup(staged=True)
    staged_pos = lane.pos
    gcmd = _Gcmd(LANE="lane9")
    u.cmd_AFC_BT_RFID_STAGE(gcmd)
    # the sweep started on the load switch, not at the hub
    assert swept["start_pos"] < staged_pos - 100.0
    assert swept["staged_during"] is False
    # full two turns: nothing else is using the hub
    assert swept["advance"] == pytest.approx(2 * 3.141592653589793 * 200.0)
    # and it ends staged again, the way an insert leaves it
    assert lane.loaded_to_hub is True
    assert lane.unit_obj.calls[-1] == "prep_post_load"
    # on the switch edge prep_load homes to, plus dist_hub
    assert lane.pos == pytest.approx(lane.load_at + 0.5 + lane.dist_hub)
    assert u.applied == ["lane9"]
    assert "re-staged at the hub" in gcmd.info[-1]
    assert u._sweeping is False


def test_a_staged_lane_is_restaged_when_no_tag_answers():
    u, lane, swept = _setup(staged=True, tag=False)
    gcmd = _Gcmd(LANE="lane9")
    u.cmd_AFC_BT_RFID_STAGE(gcmd)
    assert lane.loaded_to_hub is True
    assert u.applied == []
    assert "no tag" in gcmd.info[-1]


def test_retract_off_is_ignored_on_a_staged_lane():
    u, lane, swept = _setup(staged=True)
    u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane9", RETRACT=0))
    assert lane.loaded_to_hub is True


def test_an_unstaged_lane_is_not_staged_by_the_scan():
    u, lane, swept = _setup(staged=False)
    u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane9"))
    assert lane.loaded_to_hub is False
    assert "prep_post_load" not in lane.unit_obj.calls
    # back on its load switch, where the scan started
    assert lane.raw_load_state
    assert lane.pos - lane.load_at <= lane.short_move_dis


# ── A hub in use bounds the sweep ──────────────────────────────────────────

def test_a_busy_hub_stops_the_sweep_short_of_it():
    u, lane, swept = _setup(busy=True, dist_hub=150.0)
    gcmd = _Gcmd(LANE="lane9")
    u.cmd_AFC_BT_RFID_STAGE(gcmd)
    assert swept["advance"] == pytest.approx(150.0 - u.HUB_MARGIN_MM)
    assert "lane8 is loaded through it" in gcmd.info[0]


def test_a_staged_lane_on_a_busy_hub_gets_the_room_from_its_load_switch():
    u, lane, swept = _setup(staged=True, busy=True, dist_hub=150.0)
    u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane9"))
    assert swept["advance"] == pytest.approx(150.0 - u.HUB_MARGIN_MM)
    assert lane.loaded_to_hub is True


def test_a_real_hub_switch_reading_filament_counts_as_busy():
    u, lane, swept = _setup()
    lane.hub_obj.state = True
    advance, cut = u._sweep_room(lane, 1000.0)
    assert advance == pytest.approx(150.0 - u.HUB_MARGIN_MM)
    assert "switch reads filament" in cut


def test_a_virtual_hub_switch_is_not_evidence():
    # A virtual hub's state is every lane's load sensor, this one's included.
    u, lane, swept = _setup()
    lane.hub_obj.state = True
    lane.hub_obj._virtual = True
    assert u._sweep_room(lane, 1000.0) == (1000.0, None)


def test_a_staged_start_leaves_no_room_before_a_busy_hub():
    u, lane, swept = _setup(staged=True, busy=True)
    advance, cut = u._sweep_room(lane, 1000.0)
    assert advance == 0.0 and cut


# ── The insert scan and write staging are bounded the same way ─────────────

def test_the_insert_scan_is_bounded_by_a_busy_hub():
    u, lane, swept = _setup(busy=True, dist_hub=150.0)
    u.apply_to_lane = lambda ln, t: None
    u._restore = lambda ln, fed, back: fed
    u._on_lane_prep_loaded(lane)
    assert swept["advance"] == pytest.approx(150.0 - u.HUB_MARGIN_MM)


def test_the_insert_scan_skips_when_there_is_no_room():
    u, lane, swept = _setup(busy=True, dist_hub=20.0)
    warned = []
    u.logger = types.SimpleNamespace(info=lambda *a: None,
                                     warning=lambda m: warned.append(m))
    u._on_lane_prep_loaded(lane)
    assert "advance" not in swept
    assert warned and "not scanning" in warned[0]


# ── Write staging (AFC_RFID_WRITE / AFC_RFID_ENROLL LANE=) matches Scan Tag ─

def _write_setup(**kw):
    u, lane, swept = _setup(**kw)
    u._settle_on_tag = lambda *a: 0.0
    return u, lane, swept


def test_write_staging_brings_a_staged_lane_back_and_restages_it():
    u, lane, swept = _write_setup(staged=True)
    staged_pos = lane.pos
    token = u._stage_for_write("lane9")
    assert swept["start_pos"] < staged_pos - 100.0
    assert swept["staged_during"] is False
    assert swept["advance"] == pytest.approx(2 * 3.141592653589793 * 200.0)
    assert u._sweeping is True             # held for the write
    u._unstage_after_write(token)
    assert lane.loaded_to_hub is True
    assert lane.unit_obj.calls[-1] == "prep_post_load"
    assert lane.pos == pytest.approx(lane.load_at + 0.5 + lane.dist_hub)
    assert u._sweeping is False


def test_write_staging_leaves_an_unstaged_lane_unstaged():
    u, lane, swept = _write_setup(staged=False)
    u._unstage_after_write(u._stage_for_write("lane9"))
    assert lane.loaded_to_hub is False
    assert "prep_post_load" not in lane.unit_obj.calls
    assert lane.pos - lane.load_at <= lane.short_move_dis


def test_write_staging_rolls_the_sister_tag_off_and_back():
    u, lane, swept = _write_setup(staged=True)
    order = []
    u._clear_sibling = lambda ln, rdr: order.append("clear") or "sib"
    u._restore_sibling = lambda tok: order.append(("restore", tok))
    u._unstage_after_write(u._stage_for_write("lane9"))
    assert order == ["clear", ("restore", "sib")]


def test_write_staging_refuses_a_lane_in_the_toolhead():
    u, lane, swept = _write_setup()
    lane.tool_loaded = True
    with pytest.raises(StageError, match="loaded in the toolhead"):
        u._stage_for_write("lane9")
    assert "advance" not in swept and not lane.moves
    assert u._sweeping is False


def test_write_staging_refuses_a_staged_lane_without_homing():
    u, lane, swept = _write_setup(staged=True, homing=False)
    with pytest.raises(StageError, match="homing is off"):
        u._stage_for_write("lane9")
    assert not lane.moves


def test_write_staging_on_a_busy_hub_gets_the_room_from_the_load_switch():
    u, lane, swept = _write_setup(staged=True, busy=True, dist_hub=150.0)
    u._unstage_after_write(u._stage_for_write("lane9"))
    assert swept["advance"] == pytest.approx(150.0 - u.HUB_MARGIN_MM)
    assert lane.loaded_to_hub is True


def test_write_staging_refuses_when_there_is_no_room():
    u, lane, swept = _write_setup(busy=True, dist_hub=20.0)
    with pytest.raises(StageError, match="no room before it"):
        u._stage_for_write("lane9")
    assert "advance" not in swept
    assert u._sweeping is False


def test_a_failed_sweep_releases_the_guard_without_restaging():
    u, lane, swept = _write_setup(staged=True)

    def boom(*a):
        raise RuntimeError("stepper fault")
    u._sweep_for_tag = boom
    with pytest.raises(RuntimeError, match="stepper fault"):
        u._stage_for_write("lane9")
    assert u._sweeping is False
    assert lane.loaded_to_hub is False


def test_write_staging_refuses_a_lane_on_the_other_reader():
    u, lane, swept = _write_setup()
    other = _reader("reader1", ["lane10", "lane11"])
    with pytest.raises(StageError, match="READER=bt:reader0"):
        u._stage_for_write("lane9", other)
    assert "advance" not in swept


def test_the_write_passes_over_the_sister_lanes_known_tag():
    u, lane, swept = _write_setup()
    u._last_uid_by_lane["lane8"] = "5157e12"
    excl = u._write_excluder(u._stage_for_write("lane9"))
    assert excl("5157E12") is True and excl("04ab") is False


def test_the_written_tag_is_applied_and_noted():
    u, lane, swept = _write_setup()
    got = []
    u.apply_to_lane = lambda ln, t: got.append((ln.name, t["uid"]))
    u.apply_written_tag("lane9", {"uid": "04AB", "filament": {}})
    assert got == [("lane9", "04AB")]
    assert u._last_uid_by_lane["lane9"] == "04ab"


def test_write_staging_settles_on_a_found_tag():
    # _settle_on_tag is real here: the write path must not call a method
    # that is not there.
    u, lane, swept = _setup()
    u._sweep_for_tag = lambda *a: ({"uid": "04AB"}, 40.0)
    u._read_once = lambda *a: {"uid": "04AB"}
    token = u._stage_for_write("lane9")
    assert token[1] == pytest.approx(40.0 - 3.0 * 6)
    u._unstage_after_write(token)
    assert u._sweeping is False


# ── _settle_on_tag ──────────────────────────────────────────────────────────

class TestSettleOnTag:
    def _run(self, reads, back_limit=30.0):
        u, lane, swept = _setup()
        moves = []
        u._lane_move = lambda ln, d, sp, assist=False: moves.append(d)
        seq = iter(reads)
        u._read_once = lambda *a: {"uid": "04AB"} if next(seq) else None
        moved = u._settle_on_tag(lane, "lane9", None, back_limit)
        return moved, moves

    def test_a_tag_still_in_range_is_eased_to_mid_field(self):
        # Reads at once, holds two more steps, drops on the third.
        moved, moves = self._run([True, True, True, False])
        assert moves == [-3.0, -3.0, -3.0, 3.0]
        assert moved == -6.0

    def test_an_overshot_tag_is_backed_onto_first(self):
        moved, moves = self._run([False, False, True, False])
        assert moves == [-3.0, -3.0, -3.0, 3.0]
        assert moved == -6.0

    def test_a_tag_that_never_rereads_stops_at_the_limit(self):
        moved, moves = self._run([False] * 5, back_limit=9.0)
        assert moves == [-3.0, -3.0, -3.0]
        assert moved == -9.0

    def test_a_read_that_always_holds_stops_after_six_steps(self):
        moved, moves = self._run([True] * 7)
        assert moves == [-3.0] * 6
        assert moved == -18.0
