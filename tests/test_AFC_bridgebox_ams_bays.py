"""AFC_BridgeBox: a chain builds at most four AMS bays.

A Bambu bus addresses at most four boxed units (AMS, AMS 2 Pro, AMS 1) at
once, so the AMS band never holds more than four bays and the HT band starts
past every AMS bay built. A recorded AMS whose saved name is not one of the
four AMS bay names draws a free one; one that finds all four held waits
without a bay until FORGET frees one. None of these stop Klipper from
starting, and the HT lanes stay where the recorded AMS left them. A lane that
changes unit this way keeps AFC's loaded-lane record from driving the new
unit's follower.
"""
from __future__ import annotations

import types

import pytest

from tests.test_AFC_BridgeBox import (_Bridge, _FakeReactor, _FileConfig,
                                      _GCmd, _Logger, _mk_files, _Printer)

SEC = "AFC_BridgeBox chain1"
POOL = dict(pool_ams=4, pool_ht=2)
A, B, C, D, E, G, H = ("AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "GGGG",
                       "HHHH")
FOUR = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{D}, ht:{H}"


def _record(tmp_path, roster, name_map=None, lane_map=None):
    """Leave the state file as an earlier session would: the recorded
    roster, and the maps when given."""
    m = _mk_files(tmp_path, roster="")[0]     # no roster, no pool: builds nothing
    keys = {"roster": roster}
    if name_map is not None:
        keys["name_map"] = name_map
    if lane_map is not None:
        keys["lane_map"] = lane_map
    m._state_set({SEC: keys})


def _boot(tmp_path, **over):
    """Boot from the recorded roster, as a RESTART does."""
    opts = dict(POOL)
    opts.update(over)
    m, printer, _a, _s = _mk_files(tmp_path, roster="", **opts)
    return m, printer


def _layout(printer):
    """:return dict: lane number -> the unit name it was fabricated for"""
    return {int(s.rsplit("lane", 1)[1]):
            w.fileconfig.get(s, "unit").split(":")[0]
            for s, w in printer.loaded if s.startswith("AFC_lane ")}


def _uids(printer):
    """:return dict: fabricated unit name -> its unit_uid, "" for a spare"""
    return {s.split(" ", 1)[1]: w.fileconfig.get(s, "unit_uid", fallback="")
            for s, w in printer.loaded if s.startswith("AFC_BambuAMS ")}


def _ams_bays(printer):
    return sorted(n for n in _uids(printer) if "_HT_" not in n)


def _ready(m, printer):
    """Run klippy:ready and return what reached the console."""
    log = _Logger()
    printer.objects["AFC"] = types.SimpleNamespace(logger=log)
    printer.get_reactor = lambda: _FakeReactor()
    m._scout_ready()
    return log.lines


# ── four AMS known, a replacement recorded ──────────────────────────────────

class TestAFifthRecordedAms:
    @pytest.mark.parametrize("first", [False, True],
                             ids=["listed-last", "listed-first"])
    def test_it_waits_without_a_bay_and_takes_the_forgotten_ones(
            self, tmp_path, first):
        _record(tmp_path, FOUR)
        _m1, p1 = _boot(tmp_path)
        before = _layout(p1)
        assert before[40] == "Bambu_AMS_HT_1"
        # E replaces D: plugged in with all four bays held, so the watch
        # recorded it and nothing else. Where the roster lists it does not
        # matter: the four named units keep their bays.
        _record(tmp_path, (f"boxed:{E}, " + FOUR) if first
                else (FOUR + f", boxed:{E}"))
        m2, p2 = _boot(tmp_path)
        assert _layout(p2) == before                  # HT lanes unchanged
        assert _ams_bays(p2) == [f"Bambu_AMS_{i}" for i in (1, 2, 3, 4)]
        assert E not in _uids(p2).values()
        assert E in {u["uid"] for u in m2.units}      # still recorded
        assert E not in m2._name_map and E not in m2._lane_map
        notes = _ready(m2, p2)
        (note,) = [n for n in notes if E in n]
        assert "all 4 AMS bays belong to other units" in note
        assert f"Bambu_AMS_4 ({D})" in note
        assert "AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=" in note
        assert "frees that bay and this AMS claims it live." in note
        assert len(notes) == 1                        # nothing else moved
        # FORGET the unit it replaced; the next boot gives E that bay.
        m2.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        m3, p3 = _boot(tmp_path)
        assert _layout(p3) == before
        assert _uids(p3)["Bambu_AMS_4"] == E
        assert m3._layout_notes == []

    def test_without_a_pool_it_takes_the_bay_at_the_next_restart(
            self, tmp_path):
        _record(tmp_path, FOUR)
        _m1, p1 = _boot(tmp_path, pool_ams=0, pool_ht=0)
        before = _layout(p1)
        _record(tmp_path, FOUR + f", boxed:{E}")
        m2, p2 = _boot(tmp_path, pool_ams=0, pool_ht=0)
        assert _layout(p2) == before
        assert E not in _uids(p2).values()
        (note,) = [n for n in _ready(m2, p2) if E in n]
        assert "frees that bay for it at the next RESTART." in note
        assert ("An AMS offline on a live chain for 120s is removed from the "
                "recorded roster, which frees its bay at the next RESTART "
                "too.") in note
        m2.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        _m3, p3 = _boot(tmp_path, pool_ams=0, pool_ht=0)
        assert _layout(p3) == before
        assert _uids(p3)["Bambu_AMS_4"] == E

    def test_a_state_recording_five_ams_bays_boots(self, tmp_path):
        # E recorded on Bambu_AMS_5 (lane40-lane43), the HT on lane44.
        _record(tmp_path, FOUR + f", boxed:{E}",
                name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                          f"{C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
                          f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
                lane_map=(f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, "
                          f"{E}:40:4, {H}:44:1"))
        m, p = _boot(tmp_path)
        assert _ams_bays(p) == [f"Bambu_AMS_{i}" for i in (1, 2, 3, 4)]
        assert _layout(p)[40] == "Bambu_AMS_HT_1"
        assert E not in _uids(p).values()
        assert E not in (m._state_get(SEC, "name_map") or "")
        assert E not in (m._state_get(SEC, "lane_map") or "")
        notes = _ready(m, p)
        (note,) = [n for n in notes if E in n]
        assert "Its record as Bambu_AMS_5 is dropped." in note
        (moved,) = [n for n in notes if H in n]
        assert (f"HT {H} (Bambu_AMS_HT_1) keeps its name, and its lanes and "
                f"T# changed: lane44 (T44) -> lane40 (T40).") in moved


# ── a saved name past the fourth AMS bay ────────────────────────────────────

def test_an_ams_saved_past_the_fourth_bay_redraws_inside_the_four(tmp_path):
    _record(tmp_path, f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{E}, ht:{H}",
            name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, {C}:Bambu_AMS_3, "
                      f"{E}:Bambu_AMS_5, {H}:Bambu_AMS_HT_1"),
            lane_map=(f"{A}:24:4, {B}:28:4, {C}:32:4, {E}:40:4, "
                      f"{H}:44:1"))
    m, p = _boot(tmp_path)
    assert _uids(p)["Bambu_AMS_4"] == E
    assert "Bambu_AMS_5" not in _uids(p)
    lay = _layout(p)
    assert [lay[n] for n in range(36, 41)] == ["Bambu_AMS_4"] * 4 + [
        "Bambu_AMS_HT_1"]
    assert m._lane_map[E] == (36, 4)
    assert f"{E}:Bambu_AMS_4" in m._state_get(SEC, "name_map")
    (note,) = [n for n in _ready(m, p) if E in n]
    for part in ("Bambu_AMS_5", "It is now Bambu_AMS_4",
                 "lane40-lane43 (T40-T43) -> lane36-lane39 (T36-T39)"):
        assert part in note, part
    # The next boot reads the rewritten record: nothing moves, nothing said.
    m2, p2 = _boot(tmp_path)
    assert _layout(p2) == lay
    assert m2._layout_notes == []


def test_an_ams_saved_as_the_fifth_ams_names_entry_redraws(tmp_path):
    names = "N1, N2, N3, N4, N5"
    _record(tmp_path, f"boxed:{A}, ht:{H}",
            name_map=f"{A}:N5, {H}:Bambu_AMS_HT_1",
            lane_map=f"{A}:40:4, {H}:44:1")
    m, p = _boot(tmp_path, pool_ams=1, pool_ht=1, ams_names=names)
    assert _uids(p) == {"N1": A, "Bambu_AMS_HT_1": H}
    (note,) = [n for n in _ready(m, p) if A in n]
    assert ("was recorded as N5, past the 4 AMS bays a Bambu bus addresses. "
            "It is now N1, and its lanes and T# changed: lane40-lane43 "
            "(T40-T43) -> lane24-lane27 (T24-T27).") in note


def test_an_ams_that_redraws_keeps_its_lanes_when_their_rank_is_free(
        tmp_path):
    # A's saved name is off the list, but its lanes are the third AMS bay's
    # and nothing holds that name: it keeps lane32-lane35.
    _record(tmp_path, f"boxed:{A}, ht:{H}",
            name_map=f"{A}:Alpha, {H}:Bambu_AMS_HT_1",
            lane_map=f"{A}:32:4, {H}:40:1")
    m, p = _boot(tmp_path)
    assert _uids(p)["Bambu_AMS_3"] == A
    assert m._lane_map[A] == (32, 4)
    assert _layout(p)[40] == "Bambu_AMS_HT_1"
    (note,) = [n for n in _ready(m, p) if A in n]
    assert "It is now Bambu_AMS_3, and its lanes stay lane32-lane35" in note


def test_a_recorded_ams_draws_before_a_new_uid_listed_ahead_of_it(tmp_path):
    # roster: lists the new E first; A (saved as Alpha, off the list) has
    # recorded lanes, so it takes the free bay and E waits.
    _record(tmp_path, FOUR, name_map=(
        f"{A}:Alpha, {B}:Bambu_AMS_2, {C}:Bambu_AMS_3, {D}:Bambu_AMS_4, "
        f"{H}:Bambu_AMS_HT_1"),
        lane_map=f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, {H}:40:1")
    m, p, _a, _s = _mk_files(
        tmp_path, roster=f"boxed:{E}, " + FOUR, **POOL)
    assert _uids(p)["Bambu_AMS_1"] == A
    assert E not in _uids(p).values()
    notes = _ready(m, p)
    assert [n for n in notes if f"AMS {E} is recorded but has no bay" in n]
    assert not [n for n in notes if f"AMS {A} is recorded but has no bay" in n]


# ── pool_ams lowered below recorded AMS ─────────────────────────────────────

def test_lowering_pool_ams_keeps_every_recorded_bay_and_the_ht_lanes(
        tmp_path):
    _record(tmp_path, FOUR)
    _m1, p1 = _boot(tmp_path, pool_ams=4)
    m2, p2 = _boot(tmp_path, pool_ams=2)
    assert _layout(p2) == _layout(p1)
    assert _uids(p2)["Bambu_AMS_4"] == D
    assert m2._layout_notes == []
    # Three AMS left, the highest on the fourth bay: the band still covers
    # it, and pool_ams builds no spare past the three recorded.
    m2.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=B))
    _m3, p3 = _boot(tmp_path, pool_ams=2)
    lay = _layout(p3)
    assert sorted(lay) == [*range(24, 28), *range(32, 42)]
    assert _uids(p3) == {"Bambu_AMS_1": A, "Bambu_AMS_3": C,
                         "Bambu_AMS_4": D, "Bambu_AMS_HT_1": H,
                         "Bambu_AMS_HT_2": ""}
    assert lay[40] == "Bambu_AMS_HT_1"


def _held_note(need):
    """The ready line for HHHH holding the AMS band at four bays."""
    return (f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) on lane40 (T40) "
            f"keeps its lanes, so the AMS band stays 4 bays although pool_ams "
            f"and the recorded AMS need only {need}. Set pool_ams: 4 to "
            f"silence this. AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={H} lets "
            f"the band shrink to {need} at the next RESTART, and erases what "
            f"it learned.")


def test_lowering_pool_ams_keeps_a_recorded_ht_past_the_spares(tmp_path):
    # Two AMS at ranks 0-1 and the HT past four AMS bays: pool_ams 2 builds
    # no spare on ranks 2-3, and the HT keeps lane40 at every boot.
    _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
    m1, p1 = _boot(tmp_path, pool_ams=4)
    assert _layout(p1)[40] == "Bambu_AMS_HT_1"
    assert _ready(m1, p1) == []
    for _again in range(2):
        m, p = _boot(tmp_path, pool_ams=2)
        lay = _layout(p)
        assert _uids(p) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                            "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
        assert lay[40] == "Bambu_AMS_HT_1" and lay[41] == "Bambu_AMS_HT_2"
        assert not set(range(32, 40)) & set(lay)
        assert m._lane_map[H] == (40, 1)
        assert m._lane_moves == {}
        assert _ready(m, p) == [_held_note(2)]


def test_forgetting_the_top_ams_keeps_the_ht_lanes(tmp_path):
    # pool_ams 2 and the fourth AMS forgotten, then the third: the HT stays
    # on lane40 at each of two boots, and the next new AMS takes a free
    # bay inside the band.
    _record(tmp_path, FOUR)
    _m1, p1 = _boot(tmp_path, pool_ams=2)
    assert _layout(p1)[40] == "Bambu_AMS_HT_1"
    m, p = _boot(tmp_path, pool_ams=2)
    assert _ready(m, p) == []
    m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
    for _again in range(2):
        m, p = _boot(tmp_path, pool_ams=2)
        assert _layout(p)[40] == "Bambu_AMS_HT_1"
        assert _uids(p) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                            "Bambu_AMS_3": C, "Bambu_AMS_HT_1": H,
                            "Bambu_AMS_HT_2": ""}
        assert _ready(m, p) == [_held_note(3)]
    m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=C))
    for _again in range(2):
        m, p = _boot(tmp_path, pool_ams=2)
        assert _layout(p)[40] == "Bambu_AMS_HT_1"
        assert m._lane_map[H] == (40, 1)
        assert _ready(m, p) == [_held_note(2)]
    _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}, boxed:{E}")
    m, p = _boot(tmp_path, pool_ams=2)
    assert _uids(p)["Bambu_AMS_3"] == E
    assert _layout(p)[40] == "Bambu_AMS_HT_1"


def test_lanes_saved_by_a_start_that_never_got_ready_hold_no_band(
        tmp_path):
    # pool_ams 2 -> 4 saves the HT on lane40, but a later section stops that
    # start before klippy:ready. Back to pool_ams 2, the HT is on the lanes
    # the last start that reached ready gave it.
    _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
    m1, p1 = _boot(tmp_path, pool_ams=2)
    assert _layout(p1)[32] == "Bambu_AMS_HT_1"
    assert _ready(m1, p1) == []
    m2, p2 = _boot(tmp_path, pool_ams=4)
    assert _layout(p2)[40] == "Bambu_AMS_HT_1" and m2._lane_map[H] == (40, 1)
    for _again in range(2):
        m, p = _boot(tmp_path, pool_ams=2)
        assert _layout(p)[32] == "Bambu_AMS_HT_1"
        assert m._lane_map[H] == (32, 1)
        _ready(m, p)


@pytest.mark.parametrize("where", ["printer", "config"])
def test_an_ht_lane_that_exists_elsewhere_ends_the_hold(tmp_path, where):
    # Recorded on lane40, then lanes 40-43 are written by hand and pool_ams
    # lowered to make room: the HT lanes follow pool_ams rather than stop
    # Klipper at lane40.
    _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
    m1, p1 = _boot(tmp_path, pool_ams=4)
    assert _ready(m1, p1) == []
    lanes = [f"AFC_lane lane{n}" for n in range(40, 44)]

    def _around():
        if where == "printer":
            return dict(printer=_Printer({s: object() for s in lanes}))
        return dict(fileconfig=_FileConfig(
            {s: {"unit": f"Box_1:{i + 1}"} for i, s in enumerate(lanes)}))
    m, p = _boot(tmp_path, pool_ams=2, **_around())
    lay = _layout(p)
    assert lay[32] == "Bambu_AMS_HT_1" and lay[33] == "Bambu_AMS_HT_2"
    assert not set(range(36, 44)) & set(lay)
    assert m._lane_map[H] == (32, 1)
    assert _ready(m, p) == [
        f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) on lane40 (T40) "
        f"cannot keep its lanes: [AFC_lane lane40] already exists outside "
        f"this chain. The AMS band is 2 bays, what pool_ams and the recorded "
        f"AMS need, and the HT lanes follow it.",
        f"AFC_BridgeBox chain1: HT {H} (Bambu_AMS_HT_1) keeps its name, and "
        f"its lanes and T# changed: lane40 (T40) -> lane32 (T32)."]
    m, p = _boot(tmp_path, pool_ams=2, **_around())
    assert _layout(p)[32] == "Bambu_AMS_HT_1"
    assert _ready(m, p) == []


def test_a_learned_value_filed_under_a_held_lane_is_no_clash(tmp_path):
    # AFC filed a learned key of lane40 in auto_vars, which klippy parsed:
    # the section names no lane of its own, so the HT keeps lane40.
    _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
    m1, p1 = _boot(tmp_path, pool_ams=4)
    _ready(m1, p1)
    (tmp_path / "AFC_auto_vars.cfg").write_text(
        "[AFC_lane lane40]\ndist_hub : 61.0\n")
    fc = _FileConfig({"AFC_lane lane40": {"dist_hub": "61.0"}})
    m, p = _boot(tmp_path, pool_ams=2, fileconfig=fc)
    assert _layout(p)[40] == "Bambu_AMS_HT_1"
    assert _ready(m, p) == [_held_note(2)]


# ── an ams_names / ht_names entry renamed ───────────────────────────────────

class TestARenamedNameEntry:
    def test_the_unit_redraws_and_no_bay_doubles_up(self, tmp_path):
        _record(tmp_path, f"boxed:{A}, ht:{H}")
        m1, _p1 = _boot(tmp_path, pool_ams=2, pool_ht=1,
                        ams_names="Alpha, Bravo")
        # What A learned on Alpha is kept under its uid.
        m1._state_set({f"{SEC} learned {A}": {"afc_bowden_length": "1234.0"}})
        m, p = _boot(tmp_path, pool_ams=2, pool_ht=1, ams_names="Red, Blue")
        assert _uids(p) == {"Red": A, "Blue": "", "Bambu_AMS_HT_1": H}
        lay = _layout(p)
        assert [lay[n] for n in (24, 28, 32)] == ["Red", "Blue",
                                                  "Bambu_AMS_HT_1"]
        red = dict(p.loaded)["AFC_BambuAMS Red"].fileconfig
        assert red.get("AFC_BambuAMS Red", "afc_bowden_length") == "1234.0"
        (note,) = [n for n in _ready(m, p) if A in n]
        for part in ("was recorded as Alpha, which no ams_names entry or "
                     "default name gives an AMS.", "It is now Red",
                     "its lanes stay lane24-lane27 (T24-T27)",
                     "Lane records saved under Alpha (spool, material, "
                     "colour, T# map) do not carry over to Red; a tagged "
                     "spool is read again from its tag. Its learned bowden "
                     "lengths stay with the unit."):
            assert part in note, part
        assert "values learned under" not in note

    def test_a_renamed_second_entry_leaves_the_first_unit_alone(
            self, tmp_path):
        _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        _boot(tmp_path, pool_ams=2, pool_ht=1, ams_names="Alpha, Bravo")
        m, p = _boot(tmp_path, pool_ams=2, pool_ht=1,
                     ams_names="Alpha, Charlie")
        assert _uids(p) == {"Alpha": A, "Charlie": B, "Bambu_AMS_HT_1": H}
        assert m._lane_map[A] == (24, 4) and m._lane_map[B] == (28, 4)
        (note,) = _ready(m, p)
        assert f"AMS {B} was recorded as Bravo" in note
        assert "It is now Charlie, and its lanes stay lane28-lane31" in note

    def test_adding_ams_names_renames_the_units_wearing_defaults(
            self, tmp_path):
        _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
        _boot(tmp_path, pool_ams=2, pool_ht=1)
        m, p = _boot(tmp_path, pool_ams=2, pool_ht=1, ams_names="Red, Blue")
        assert _uids(p) == {"Red": A, "Blue": B, "Bambu_AMS_HT_1": H}
        (note,) = [n for n in _ready(m, p) if A in n]
        assert (f"AMS {A} was recorded as Bambu_AMS_1, which no ams_names "
                f"entry or default name gives an AMS (ams_names replaces the "
                f"first 2 default names). It is now Red, and its lanes stay "
                f"lane24-lane27 (T24-T27). Lane records saved under "
                f"Bambu_AMS_1 (spool, material, colour, T# map) do not carry "
                f"over to Red; a tagged spool is read again from its tag. Its "
                f"learned bowden lengths stay with the unit.") in note

    def test_a_renamed_ht_redraws_too(self, tmp_path):
        _record(tmp_path, f"boxed:{A}, ht:{H}")
        _boot(tmp_path, pool_ams=1, pool_ht=2, ht_names="Hot, Warm")
        m, p = _boot(tmp_path, pool_ams=1, pool_ht=2, ht_names="Red, Warm")
        assert _uids(p) == {"Bambu_AMS_1": A, "Red": H, "Warm": ""}
        assert m._name_map[H] == "Red"
        (note,) = [n for n in _ready(m, p) if H in n]
        assert "was recorded as Hot" in note and "ht_names" in note


# ── pool_ams above four ─────────────────────────────────────────────────────

def test_pool_ams_above_four_builds_four_and_says_so(tmp_path):
    m, p, _a, _s = _mk_files(tmp_path, roster=f"ht:{H}", pool_ams=6,
                             pool_ht=1)
    assert m.pool_ams == 4
    assert _ams_bays(p) == [f"Bambu_AMS_{i}" for i in (1, 2, 3, 4)]
    assert _layout(p)[40] == "Bambu_AMS_HT_1"
    (warn,) = [n for n in _ready(m, p) if "pool_ams" in n]
    assert "pool_ams is 6" in warn and "4 AMS bays are built" in warn
    assert "and the HT lanes start at lane40." in warn
    (tmp_path / "four").mkdir()
    m4, p4, _a, _s = _mk_files(tmp_path / "four", roster=f"ht:{H}",
                               pool_ams=4, pool_ht=1)
    assert not [n for n in _ready(m4, p4) if "pool_ams" in n]


def test_pool_ams_above_four_moves_an_ht_laid_out_past_four_ams_bays(
        tmp_path):
    # The state as a six-bay AMS band left it: the HT on lane48.
    _record(tmp_path, f"boxed:{A}, ht:{H}",
            name_map=f"{A}:Bambu_AMS_1, {H}:Bambu_AMS_HT_1",
            lane_map=f"{A}:24:4, {H}:48:1")
    m, p = _boot(tmp_path, pool_ams=6, pool_ht=2)
    lay = _layout(p)
    assert lay[40] == "Bambu_AMS_HT_1" and lay[41] == "Bambu_AMS_HT_2"
    assert 48 not in lay
    assert m._lane_map[H] == (40, 1)
    notes = _ready(m, p)
    (warn,) = [n for n in notes if "pool_ams is 6" in n]
    assert "the HT lanes start at lane40" in warn
    (moved,) = [n for n in notes if H in n]
    assert "lane48 (T48) -> lane40 (T40)" in moved
    assert m._lane_moves["lane48"]["now"] is None


# ── no pool, a lower AMS pruned from the roster ─────────────────────────────

def test_without_a_pool_the_ht_stays_clear_of_a_surviving_higher_ams(
        tmp_path):
    _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}")
    _m1, p1 = _boot(tmp_path, pool_ams=0, pool_ht=0)
    assert _layout(p1)[32] == "Bambu_AMS_HT_1"
    # Auto-removal drops A from the recorded roster; its tombstone stays.
    _record(tmp_path, f"boxed:{B}, ht:{H}")
    m, p = _boot(tmp_path, pool_ams=0, pool_ht=0)
    lay = _layout(p)
    assert sorted(lay) == [28, 29, 30, 31, 32]
    assert lay[28] == "Bambu_AMS_2" and lay[32] == "Bambu_AMS_HT_1"
    assert m._layout_notes == []


def test_without_a_pool_a_new_ams_takes_a_free_bay_inside_the_band(
        tmp_path):
    # A and B departed (tombstones); D kept the fourth bay, so the band
    # spans four and the third bay moves no HT lane: E takes it rather than
    # a departed unit's name and learned values.
    _record(tmp_path, f"boxed:{D}, ht:{H}, boxed:{E}",
            name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, {D}:Bambu_AMS_4, "
                      f"{H}:Bambu_AMS_HT_1"),
            lane_map=f"{A}:24:4, {B}:28:4, {D}:36:4, {H}:40:1")
    m, p = _boot(tmp_path, pool_ams=0, pool_ht=0)
    assert _uids(p) == {"Bambu_AMS_3": E, "Bambu_AMS_4": D,
                        "Bambu_AMS_HT_1": H}
    assert m._name_map[A] == "Bambu_AMS_1"       # tombstones kept
    assert m._name_map[B] == "Bambu_AMS_2"
    assert _layout(p)[40] == "Bambu_AMS_HT_1"


# ── names a family is given, and two uids on one name ───────────────────────

class TestSavedNames:
    def test_rank_of_reads_ht_names_past_sixteen(self, tmp_path):
        m, _p = _boot(tmp_path)
        assert m._rank_of("ht", "Bambu_AMS_HT_1") == 0
        assert m._rank_of("ht", "Bambu_AMS_HT_17") == 16
        assert m._rank_of("ht", "Bambu_AMS_HT_40") == 39
        assert m._rank_of("ht", "Bambu_AMS_HT_01") is None
        assert m._rank_of("ht", "Bambu_AMS_2") is None

    def test_rank_of_gives_an_ams_the_four_bay_names_only(self, tmp_path):
        m, _p = _boot(tmp_path, ams_names="Alpha")
        assert m._rank_of("ams", "Alpha") == 0
        assert m._rank_of("ams", "Bambu_AMS_1") is None   # index 0 is Alpha
        assert m._rank_of("ams", "Bambu_AMS_4") == 3
        assert m._rank_of("ams", "Bambu_AMS_5") is None
        assert m._rank_of("ams", "Bambu_AMS_HT_1") is None

    def test_an_ht_saved_past_sixteen_keeps_its_bay(self, tmp_path):
        _record(tmp_path, f"ht:{H}", name_map=f"{H}:Bambu_AMS_HT_18")
        m, p = _boot(tmp_path, pool_ams=1, pool_ht=1)
        assert _uids(p)["Bambu_AMS_HT_18"] == H
        assert _layout(p)[28 + 17] == "Bambu_AMS_HT_18"
        assert m._layout_notes == []

    @pytest.mark.parametrize("roster, name, second", [
        (f"boxed:{A}, boxed:{B}", "Bambu_AMS_1", "Bambu_AMS_2"),
        (f"ht:{G}, ht:{H}", "Bambu_AMS_HT_1", "Bambu_AMS_HT_2"),
    ])
    def test_of_two_uids_on_one_name_the_first_keeps_it(
            self, tmp_path, roster, name, second):
        # A hand-edited state file: both uids recorded with one name.
        first, other = [e.split(":")[1] for e in roster.split(", ")]
        _record(tmp_path, roster, name_map=f"{first}:{name}, {other}:{name}")
        m, p = _boot(tmp_path)
        assert _uids(p)[name] == first
        assert _uids(p)[second] == other
        (note,) = [n for n in _ready(m, p) if other in n]
        assert f"which {first} also holds" in note

    def test_the_overlap_backstop_names_both_bays_and_the_record(
            self, tmp_path):
        m, _p = _boot(tmp_path)
        a = {"name": "X", "uid": A, "family": "ams", "rank": 0, "lane": 24,
             "slots": 4, "spare": False}
        b = {"name": "Y", "uid": B, "family": "ams", "rank": 0, "lane": 24,
             "slots": 4, "spare": False}
        msg = m._overlap_error(a, b)
        assert "X (lane24-lane27)" in msg and "Y (lane24-lane27)" in msg
        assert m.state_file in msg and f"{A} or {B}" in msg
        assert "need Klipper running" in msg and "by hand" in msg
        for gone in ("pool_ams", "ams_names", "ht_names"):
            assert gone not in msg, gone


# ── name lists that give one name to two bays ───────────────────────────────

class TestNamesThatClash:
    """A name is one bay. Within a family an entry keeps its name, and the
    default of an index past the list that it holds takes a suffix; an
    entry repeated in its family, or equal to a name of the other family,
    gives way to its bay's default. Every AMS rank keeps a name, and no
    recorded unit loses its bay or moves."""

    def test_an_ams_entry_equal_to_an_ht_name_costs_no_bay(self, tmp_path):
        _record(tmp_path, FOUR)
        _m1, p1 = _boot(tmp_path)
        for _again in range(2):
            m, p = _boot(tmp_path, ams_names="Bambu_AMS_HT_1")
            assert _uids(p) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                "Bambu_AMS_3": C, "Bambu_AMS_4": D,
                                "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
            assert _layout(p) == _layout(p1)
            notes = _ready(m, p)
            assert notes == [
                "AFC_BridgeBox chain1: ams_names entry 1 (Bambu_AMS_HT_1) is "
                "also the default name of HT bay 1, so AMS bay 1 is named "
                "Bambu_AMS_1 -- give each bay a name of its own."]

    @pytest.mark.parametrize("pool", [POOL, dict(pool_ams=0, pool_ht=0)],
                             ids=["pool", "no-pool"])
    def test_a_repeated_ams_entry_still_builds_four_ams_bays(
            self, tmp_path, pool):
        _record(tmp_path, FOUR)
        for _again in range(2):
            m, p = _boot(tmp_path, ams_names="X, X, Y, Z", **pool)
            assert {n: u for n, u in _uids(p).items() if u} == {
                "X": A, "Bambu_AMS_2": B, "Y": C, "Z": D,
                "Bambu_AMS_HT_1": H}
            assert _layout(p)[40] == "Bambu_AMS_HT_1"
            notes = _ready(m, p)
            assert ("AFC_BridgeBox chain1: ams_names entry 2 (X) is also "
                    "ams_names entry 1, so AMS bay 2 is named Bambu_AMS_2 -- "
                    "give each bay a name of its own.") in notes
            assert not [n for n in notes if "has no bay" in n]

    def test_a_repeated_ht_entry_names_its_bay_by_default(self, tmp_path):
        _record(tmp_path, f"boxed:{A}, ht:{G}, ht:{H}")
        for _again in range(2):
            m, p = _boot(tmp_path, pool_ams=1, ht_names="Hot, Hot")
            assert _uids(p) == {"Bambu_AMS_1": A, "Hot": G,
                                "Bambu_AMS_HT_2": H}
            assert _layout(p)[28] == "Hot"
            assert _layout(p)[29] == "Bambu_AMS_HT_2"
            assert ("AFC_BridgeBox chain1: ht_names entry 2 (Hot) is also "
                    "ht_names entry 1, so HT bay 2 is named Bambu_AMS_HT_2 -- "
                    "give each bay a name of its own.") in _ready(m, p)

    def test_a_fallback_name_an_entry_holds_takes_a_suffix(self, tmp_path):
        # Entry 2 repeats entry 1, and its default is entry 3.
        m, p = _boot(tmp_path, pool_ams=3, pool_ht=0,
                     ams_names="X, X, Bambu_AMS_2")
        assert sorted(_uids(p)) == ["Bambu_AMS_2", "Bambu_AMS_2_2", "X"]
        assert m._rank_of("ams", "Bambu_AMS_2_2") == 1
        assert m._rank_of("ams", "Bambu_AMS_2") == 2

    def test_lists_without_a_clash_say_nothing(self, tmp_path):
        _record(tmp_path, FOUR)
        m, p = _boot(tmp_path, ams_names="W, X, Y, Z", ht_names="Hot")
        assert _ready(m, p) == []
        assert m._given_names()[2:] == ({}, {})

    # Each test below seeds the maps of a unit wearing an entry that is
    # also the default of an index past its list, and checks that two boots
    # leave its name and lanes as recorded.

    def test_an_entry_named_like_a_later_default_keeps_its_lanes(
            self, tmp_path):
        _record(tmp_path, f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}",
                name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, "
                          f"{C}:Bambu_AMS_4, {H}:Bambu_AMS_HT_1"),
                lane_map=f"{A}:24:4, {B}:28:4, {C}:32:4, {H}:40:1")
        for _again in range(2):
            m, p = _boot(tmp_path,
                         ams_names="Bambu_AMS_1, Bambu_AMS_2, Bambu_AMS_4")
            lay = _layout(p)
            assert _uids(p) == {"Bambu_AMS_1": A, "Bambu_AMS_2": B,
                                "Bambu_AMS_4": C, "Bambu_AMS_4_2": "",
                                "Bambu_AMS_HT_1": H, "Bambu_AMS_HT_2": ""}
            assert (lay[24], lay[28], lay[32], lay[36], lay[40]) == (
                "Bambu_AMS_1", "Bambu_AMS_2", "Bambu_AMS_4", "Bambu_AMS_4_2",
                "Bambu_AMS_HT_1")
            assert m._lane_map[C] == (32, 4) and m._lane_moves == {}
            assert _ready(m, p) == [
                "AFC_BridgeBox chain1: the default name of AMS bay 4 "
                "(Bambu_AMS_4) is ams_names entry 3, so AMS bay 4 is named "
                "Bambu_AMS_4_2."]

    def test_without_a_pool_the_entry_and_the_ht_stay_put(self, tmp_path):
        _record(tmp_path, f"boxed:{A}, ht:{H}",
                name_map=f"{A}:Bambu_AMS_3, {H}:Bambu_AMS_HT_1",
                lane_map=f"{A}:24:4, {H}:28:1")
        for _again in range(2):
            m, p = _boot(tmp_path, ams_names="Bambu_AMS_3", pool_ams=0,
                         pool_ht=0)
            assert _layout(p) == {24: "Bambu_AMS_3", 25: "Bambu_AMS_3",
                                  26: "Bambu_AMS_3", 27: "Bambu_AMS_3",
                                  28: "Bambu_AMS_HT_1"}
            assert _ready(m, p) == []

    def test_an_ht_entry_named_like_an_unbuilt_default_says_nothing(
            self, tmp_path):
        # No HT bay 2 is built, so its default needs no other name.
        _record(tmp_path, f"boxed:{A}, ht:{H}",
                name_map=f"{A}:Bambu_AMS_1, {H}:Bambu_AMS_HT_2",
                lane_map=f"{A}:24:4, {H}:28:1")
        for _again in range(2):
            m, p = _boot(tmp_path, ht_names="Bambu_AMS_HT_2", pool_ams=1,
                         pool_ht=0)
            assert _uids(p) == {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": H}
            assert _layout(p)[28] == "Bambu_AMS_HT_2"
            assert _ready(m, p) == []
        # A new HT takes the entry too.
        _record(tmp_path, f"boxed:{A}, ht:{G}", name_map="", lane_map="")
        m, p = _boot(tmp_path, ht_names="Bambu_AMS_HT_2", pool_ams=1,
                     pool_ht=0)
        assert _uids(p) == {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": G}

    def test_a_built_ht_default_an_entry_holds_takes_a_suffix(
            self, tmp_path):
        _record(tmp_path, f"boxed:{A}, ht:{H}")
        for _again in range(2):
            m, p = _boot(tmp_path, ht_names="Bambu_AMS_HT_2", pool_ams=1)
            assert _uids(p) == {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": H,
                                "Bambu_AMS_HT_2_2": ""}
            assert _layout(p)[28] == "Bambu_AMS_HT_2"
            assert _layout(p)[29] == "Bambu_AMS_HT_2_2"
            assert m._rank_of("ht", "Bambu_AMS_HT_2_2") == 1
            assert _ready(m, p) == [
                "AFC_BridgeBox chain1: the default name of HT bay 2 "
                "(Bambu_AMS_HT_2) is ht_names entry 1, so HT bay 2 is named "
                "Bambu_AMS_HT_2_2."]

    @pytest.mark.parametrize("family", ["ams", "ht"])
    def test_an_entry_chosen_around_a_hand_written_unit_boots(
            self, tmp_path, family):
        # The hand-written unit wears the default name; the entry names the
        # chain's bay after the next one.
        taken = "Bambu_AMS_1" if family == "ams" else "Bambu_AMS_HT_1"
        fc = _FileConfig({"AFC_stepper lane1": {"unit": f"{taken}:1"},
                          "AFC_stepper lane2": {"unit": f"{taken}:2"}})
        names = ({"ams_names": "Bambu_AMS_2"} if family == "ams"
                 else {"ht_names": "Bambu_AMS_HT_2"})
        _record(tmp_path, f"boxed:{A}, ht:{H}")
        for _again in range(2):
            m, p = _boot(tmp_path, pool_ams=1, pool_ht=1, fileconfig=fc,
                         **names)
            assert _uids(p) == (
                {"Bambu_AMS_2": A, "Bambu_AMS_HT_1": H} if family == "ams"
                else {"Bambu_AMS_1": A, "Bambu_AMS_HT_2": H})
            assert _layout(p)[24] == ("Bambu_AMS_2" if family == "ams"
                                      else "Bambu_AMS_1")
            assert _ready(m, p) == []

    def test_an_ams_entry_an_ht_entry_takes_says_so(self, tmp_path):
        _record(tmp_path, f"boxed:{A}, ht:{H}")
        _m1, _p1 = _boot(tmp_path, ams_names="Hot")
        m, p = _boot(tmp_path, ams_names="Hot", ht_names="Hot")
        assert _uids(p)["Bambu_AMS_1"] == A and _layout(p)[24] == \
            "Bambu_AMS_1"
        (note,) = [n for n in _ready(m, p) if f"AMS {A}" in n]
        assert note.startswith(
            f"AFC_BridgeBox chain1: AMS {A} was recorded as Hot, whose "
            f"ams_names entry 1 gives way to ht_names entry 1. It is now "
            f"Bambu_AMS_1, and its lanes stay lane24-lane27 (T24-T27).")


# ── the console line when an AMS finds no free bay ──────────────────────────

def _on_the_wire(m, monkeypatch, online, uids=(A, B, C, D, E, G, H)):
    """Register a bridge for the chain that reads ``online`` as online.

    :return: the bridge, whose _online list a test may flip
    """
    from extras import AFC_BambuAMS_bridge as bridge_mod
    bridge = _Bridge(uids=list(uids), online=[u in online for u in uids])
    monkeypatch.setattr(bridge_mod, "_BRIDGES", {m.serial_port: bridge},
                        raising=False)
    return bridge


class TestNoFreeBayMessage:
    def _chain(self, tmp_path, roster=FOUR, **over):
        _record(tmp_path, roster)
        m, p = _boot(tmp_path, **over)
        m.logger = _Logger()
        p.objects["AFC"] = types.SimpleNamespace(lanes={}, tool_cmds={})
        return m, p

    def test_with_four_ams_bays_held_it_says_forget_not_restart(
            self, tmp_path, monkeypatch):
        m, _p = self._chain(tmp_path)
        # A-C on the wire; D unplugged, still claimed (auto_drop off).
        _on_the_wire(m, monkeypatch, {A, B, C, E})
        for pu in m._pool_units:
            if pu["uid"] in (A, B, C, D):
                pu["bound"] = pu["uid"]
        assert m._claim_pool_unit(E, "boxed") is None
        (msg,) = m.logger.lines
        assert f"AMS {E} has no bay" in msg
        assert "all 4 AMS bays belong to other units" in msg
        assert "neither pool_ams nor RESTART adds one" in msg
        assert (f"Bambu_AMS_4 ({D}) is offline: if this AMS replaces it, "
                f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={D} frees that bay "
                f"and this AMS claims it live.") in msg
        assert "raise pool_ams" not in msg
        assert "RESTART to enroll" not in msg
        m._claim_pool_unit(E, "boxed")           # the watch retries each tick
        assert len(m.logger.lines) == 1

    def test_with_several_offline_it_lists_them(self, tmp_path,
                                                monkeypatch):
        m, _p = self._chain(tmp_path)
        _on_the_wire(m, monkeypatch, {A, B, E})
        m._claim_pool_unit(E, "boxed")
        (msg,) = m.logger.lines
        assert f"Offline: Bambu_AMS_3 ({C}), Bambu_AMS_4 ({D})." in msg
        assert ("If this AMS replaces one of them, AFC_BRIDGEBOX_FORGET "
                "CHAIN=chain1 UID=<that unit's uid> frees that bay and this "
                "AMS claims it live.") in msg

    def test_with_roster_set_it_says_to_swap_the_entry_there(
            self, tmp_path, monkeypatch):
        # roster: is the whole roster: FORGET alone leaves D's bay built
        # for D at the next restart, and E unlisted.
        _record(tmp_path, FOUR)
        m, _p, _a, _s = _mk_files(tmp_path, roster=FOUR, **POOL)
        m.logger = _Logger()
        _p.objects["AFC"] = types.SimpleNamespace(lanes={}, tool_cmds={})
        _on_the_wire(m, monkeypatch, {A, B, C, E, H})
        m._claim_pool_unit(E, "boxed")
        (msg,) = m.logger.lines
        assert (f"Bambu_AMS_4 ({D}) is offline: if this AMS replaces it, "
                f"replace its entry in roster: with boxed:{E}, run "
                f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={D}, and "
                f"RESTART.") in msg
        assert "claims it live" not in msg
        # What the line says works: FORGET D, then the swapped roster: gives
        # E that bay.
        m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
        swapped = f"boxed:{A}, boxed:{B}, boxed:{C}, boxed:{E}, ht:{H}"
        _m2, p2, _a, _s = _mk_files(tmp_path, roster=swapped, **POOL)
        assert _uids(p2)["Bambu_AMS_4"] == E

    def test_with_roster_set_a_listed_unit_is_told_to_drop_the_other(
            self, tmp_path, monkeypatch):
        _record(tmp_path, FOUR)
        m, _p, _a, _s = _mk_files(tmp_path, roster=FOUR + f", boxed:{E}",
                                  **POOL)
        (note,) = [n for n in _ready(m, _p) if E in n]
        assert ("AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID=<that unit's uid> "
                "frees that bay and this AMS claims it live; remove that unit "
                "from roster: too.") in note

    def test_all_ams_ranks_held_by_a_bay_with_no_uid_does_not_fail(
            self, tmp_path):
        # A bay bound live to a unit the restart roster does not hold yet:
        # four AMS bays are built and each is held, so the line names the
        # four holders, and neither pool_ams nor a restart is offered.
        m, _p = self._chain(tmp_path)
        bay4 = next(pu for pu in m._pool_units if pu["name"] == "Bambu_AMS_4")
        bay4["uid"], bay4["bound"] = None, G
        m._state_set({SEC: {"roster": f"boxed:{A}, boxed:{B}, boxed:{C}, "
                                      f"ht:{H}"}})
        msg = m._no_bay_message(E, "ams")
        assert msg.startswith(
            f"AFC_BridgeBox chain1: AMS {E} has no bay: all 4 AMS bays belong "
            f"to other units (Bambu_AMS_1 ({A}), Bambu_AMS_2 ({B}), "
            f"Bambu_AMS_3 ({C}), Bambu_AMS_4 ({G})), and a Bambu bus "
            f"addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
            f"one.")
        assert "RESTART builds" not in msg and "raise pool_ams" not in msg

    def test_two_new_ams_and_one_spare_the_second_is_told_to_forget(
            self, tmp_path, monkeypatch):
        # A-C saved on Bambu_AMS_1-3; D takes the Bambu_AMS_4 spare live,
        # and E comes on before D is recorded. All four AMS bays are held,
        # one by a unit not recorded yet: E is pointed to FORGET, never to
        # a restart, and the restart keeps D on its bay and E waiting.
        _record(tmp_path, f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
        m, p = _boot(tmp_path)
        m.logger = _Logger()
        _claimable(m, p, monkeypatch)
        from extras import AFC_BambuAMS_bridge as bridge_mod
        bridge = _Bridge(uids=[A, B, C, D, E, H],
                         online=[True, True, False, True, False, True],
                         htmask=1 << 5)
        monkeypatch.setattr(bridge_mod, "_BRIDGES", {m.serial_port: bridge},
                            raising=False)
        m._scout_tick(100.0)
        assert m._bay_of_uid(D)["name"] == "Bambu_AMS_4"
        bridge._online[4] = True
        m._scout_tick(105.0)
        assert m._bay_of_uid(E) is None
        (msg,) = [x for x in m.logger.lines if f"AMS {E}" in x]
        assert msg.startswith(
            f"AFC_BridgeBox chain1: AMS {E} has no bay: all 4 AMS bays belong "
            f"to other units (Bambu_AMS_1 ({A}), Bambu_AMS_2 ({B}), "
            f"Bambu_AMS_3 ({C}), Bambu_AMS_4 ({D})), and a Bambu bus "
            f"addresses at most 4 AMS, so neither pool_ams nor RESTART adds "
            f"one.")
        assert (f"Bambu_AMS_3 ({C}) is offline: if this AMS replaces it, "
                f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={C} frees that bay "
                f"and this AMS claims it live.") in msg
        assert "RESTART builds" not in msg and "raise pool_ams" not in msg
        for t in range(106, 122):
            m._scout_tick(float(t))
        assert m._name_map[D] == "Bambu_AMS_4"
        assert f"boxed:{E}" in m._state_get(SEC, "roster")
        assert len([x for x in m.logger.lines if "has no bay" in x]) == 1
        m2, p2 = _boot(tmp_path)
        assert _uids(p2)["Bambu_AMS_4"] == D
        assert E not in _uids(p2).values()
        (note,) = [n for n in _ready(m2, p2) if E in n]
        assert f"AMS {E} is recorded but has no bay" in note

    def test_with_fewer_ams_bays_a_restart_builds_one(self, tmp_path):
        m, _p = self._chain(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}",
                            pool_ams=2)
        m._claim_pool_unit(E, "boxed")
        (msg,) = m.logger.lines
        assert (f"new AMS {E} has no free bay: every AMS bay built belongs "
                f"to a known unit.") in msg
        assert ("RESTART builds it one, past the AMS band, which moves the "
                "HT lanes up 4; raise pool_ams") in msg
        assert "Bambu bus" not in msg
        _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}, boxed:{E}")
        m2, p2 = _boot(tmp_path, pool_ams=2)
        assert _uids(p2)["Bambu_AMS_3"] == E
        assert _layout(p2)[36] == "Bambu_AMS_HT_1"         # was lane32
        (moved,) = [n for n in _ready(m2, p2) if H in n]
        assert "lane32 (T32) -> lane36 (T36)" in moved

    def test_a_bay_inside_the_ams_band_moves_nothing(self, tmp_path):
        # B holds the fourth bay, so the band spans four and E's bay at
        # restart is the second, inside it.
        _record(tmp_path, f"boxed:{A}, boxed:{B}, ht:{H}",
                name_map=f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_4")
        m, p = _boot(tmp_path, pool_ams=1)
        m.logger = _Logger()
        p.objects["AFC"] = types.SimpleNamespace(lanes={}, tool_cmds={})
        m._claim_pool_unit(E, "boxed")
        (msg,) = m.logger.lines
        assert "RESTART builds it one; raise pool_ams" in msg

    def test_an_ht_is_told_a_restart_builds_its_bay(self, tmp_path):
        m, _p = self._chain(tmp_path, f"boxed:{A}, ht:{H}", pool_ams=1,
                            pool_ht=0)
        m._claim_pool_unit(G, "ht")
        (msg,) = m.logger.lines
        assert f"new HT {G} has no free bay" in msg
        assert "RESTART builds it one; raise pool_ht" in msg
        assert "Bambu bus" not in msg and "HT lanes" not in msg
        _record(tmp_path, f"boxed:{A}, ht:{H}, ht:{G}")
        _m2, p2 = _boot(tmp_path, pool_ams=1, pool_ht=0)
        assert _uids(p2)["Bambu_AMS_HT_2"] == G


# ── a waiting AMS claims the bay FORGET frees, live ─────────────────────────

def _claimable(m, printer, monkeypatch):
    """Give every bay the lanes and unit object a live claim works on."""
    from extras import AFC_BridgeBox as bb
    printer.objects["AFC"] = types.SimpleNamespace(lanes={}, tool_cmds={})
    monkeypatch.setattr(bb, "activate_from_pool", lambda lane: None)
    monkeypatch.setattr(bb, "deactivate_to_pool", lambda lane: None)
    monkeypatch.setattr(bb, "assign_pool_tcmd", lambda lane, afc=None: None)
    monkeypatch.setattr(m, "_take_home_tool", lambda afc, lane: None)
    for pu in m._pool_units:
        for ln in pu["lanes"]:
            printer.objects["AFC_lane " + ln] = types.SimpleNamespace(
                name=ln, map=[], _map=[], current_map="", unassigned=True)
        unit = types.SimpleNamespace(name=pu["name"], pool=True, claimed=[])
        unit.set_master = lambda master: None
        unit.claim = (lambda uid, model, u=unit:
                      u.claimed.append(uid) or True)
        unit.release = lambda: None
        printer.objects["AFC_BambuAMS " + pu["name"]] = unit


def _waiting_chain(tmp_path, monkeypatch):
    """E recorded beside four AMS, booted: it waits with no bay."""
    _record(tmp_path, FOUR + f", boxed:{E}")
    m, p = _boot(tmp_path)
    m.logger = _Logger()
    _claimable(m, p, monkeypatch)
    bridge = _on_the_wire(m, monkeypatch, {A, B, C, E}, uids=(A, B, C, D, E))
    return m, p, bridge


@pytest.mark.parametrize("d_bound", [False, True],
                         ids=["forgotten-offline", "forgotten-still-bound"])
def test_a_waiting_ams_claims_the_bay_forget_frees(tmp_path, monkeypatch,
                                                   d_bound):
    m, p, _bridge = _waiting_chain(tmp_path, monkeypatch)
    bay4 = next(pu for pu in m._pool_units if pu["name"] == "Bambu_AMS_4")
    if d_bound:
        bay4["bound"] = D              # unplugged; auto_drop off keeps it
    m._scout_tick(100.0)
    m._scout_tick(101.0)
    assert bay4["bound"] == (D if d_bound else None)
    # The ready note told E; the watch does not say it again.
    assert not [x for x in m.logger.lines if f"AMS {E} has no bay" in x]
    m.cmd_AFC_BRIDGEBOX_FORGET(_GCmd(UID=D))
    m._scout_tick(102.0)
    assert bay4["bound"] == E
    assert p.objects["AFC_BambuAMS Bambu_AMS_4"].claimed == [E]
    assert {pu["name"]: pu["bound"] for pu in m._pool_units
            if pu["family"] == "ams"} == {
        "Bambu_AMS_1": A, "Bambu_AMS_2": B, "Bambu_AMS_3": C,
        "Bambu_AMS_4": E}
    # The claim is saved, so the restart keeps E on that bay.
    m2, p2 = _boot(tmp_path)
    assert _uids(p2)["Bambu_AMS_4"] == E
    assert m2._layout_notes == []


def test_a_bay_a_waiting_ams_claims_after_unassign_survives_a_restart(
        tmp_path, monkeypatch):
    # UNASSIGN keeps D recorded, unnamed and ahead of E in the roster; the
    # bay E claimed live is saved once E has held it for enroll_grace, and
    # stays E's at the next boot.
    m, _p, _bridge = _waiting_chain(tmp_path, monkeypatch)
    m.cmd_AFC_BRIDGEBOX_UNASSIGN(_GCmd(UID=D))
    m._scout_tick(100.0)
    assert m._bay_of_uid(E)["name"] == "Bambu_AMS_4"
    assert E not in m._state_get(SEC, "name_map")
    m._scout_tick(100.0 + m.enroll_grace)
    assert f"{E}:Bambu_AMS_4" in m._state_get(SEC, "name_map")
    m2, p2 = _boot(tmp_path)
    assert _uids(p2)["Bambu_AMS_4"] == E
    assert D not in _uids(p2).values()
    (note,) = [n for n in _ready(m2, p2) if D in n]
    assert f"AMS {D} is recorded but has no bay" in note


def test_a_waiting_ams_that_returns_is_told_once(tmp_path, monkeypatch):
    m, _p, bridge = _waiting_chain(tmp_path, monkeypatch)
    m._scout_tick(100.0)
    bridge._online[4] = False                   # E unplugged
    m._scout_tick(101.0)
    bridge._online[4] = True                    # and back
    m._scout_tick(102.0)
    m._scout_tick(103.0)
    (msg,) = [x for x in m.logger.lines if f"AMS {E} has no bay" in x]
    assert (f"AFC_BRIDGEBOX_FORGET CHAIN=chain1 UID={D} frees that bay"
            in msg)


@pytest.mark.parametrize("dark", [[True], [False, True]],
                         ids=["before-the-first-status", "bridge-dropout"])
def test_a_tick_with_nothing_online_does_not_retell_a_waiting_ams(
        tmp_path, monkeypatch, dark):
    # A tick with no unit online proves no one unit left: the ready note
    # stands, before the first status and across a bridge dropout alike.
    m, _p, bridge = _waiting_chain(tmp_path, monkeypatch)
    online = list(bridge._online)
    t = 100.0
    for all_off in dark:
        bridge._online = [False] * len(online) if all_off else list(online)
        m._scout_tick(t)
        t += 1.0
    bridge._online = list(online)
    m._scout_tick(t)
    m._scout_tick(t + 1.0)
    assert m._bay_of_uid(E) is None and E in m._no_bay_told   # still waiting
    assert not [x for x in m.logger.lines if f"AMS {E} has no bay" in x]


# ── the watch's enrollment line ─────────────────────────────────────────────

@pytest.mark.parametrize("pool", [POOL, dict(pool_ams=0, pool_ht=0)],
                         ids=["pool", "no-pool"])
def test_a_new_ams_with_every_bay_held_is_not_told_to_restart(
        tmp_path, monkeypatch, pool):
    _record(tmp_path, FOUR)
    m, p = _boot(tmp_path, **pool)
    m.logger = _Logger()
    _claimable(m, p, monkeypatch)
    # E in D's place on the wire, and a new HT G beside H.
    _on_the_wire(m, monkeypatch, {A, B, C, E, H, G},
                 uids=(A, B, C, E, H, G))
    for t in range(100, 101 + int(m.enroll_grace)):
        m._scout_tick(float(t))
    roster = m._state_get(SEC, "roster")
    assert f"boxed:{E}" in roster and f"ht:{G}" in roster
    # One line says E has no bay, from one helper; with a pool the
    # enrollment line also lists E among the units it recorded.
    about_e = [x for x in m.logger.lines
               if E in x and "NEW unit(s) on the chain" not in x]
    (told,) = about_e
    assert ("all 4 AMS bays belong to other units" in told
            and "neither pool_ams nor RESTART adds one" in told)
    assert f"Bambu_AMS_4 ({D}) is offline: if this AMS replaces it" in told
    if pool is POOL:
        assert told.startswith(f"AFC_BridgeBox chain1: AMS {E} has no bay: ")
    else:
        assert told.startswith(f"AFC_BridgeBox chain1: NEW AMS on the chain: "
                               f"boxed:{E} -- recorded, but it has no bay: ")
        assert (f"FORGET CHAIN=chain1 UID={D} frees that bay for it at the "
                f"next RESTART") in told
    if pool is POOL:
        (enroll,) = [x for x in m.logger.lines
                     if "NEW unit(s) on the chain" in x]
        assert f"ht:{G}" in enroll and "has no bay yet" not in enroll
        assert "RESTART to enroll" not in enroll
    else:
        (enroll,) = [x for x in m.logger.lines if "RESTART to enroll" in x]
        assert f"ht:{G}" in enroll and E not in enroll


@pytest.mark.parametrize("pool", [POOL, dict(pool_ams=0, pool_ht=0)],
                         ids=["pool", "no-pool"])
def test_with_roster_set_a_new_ams_with_every_bay_held_is_not_told_to_add_it(
        tmp_path, monkeypatch, pool):
    # roster: lists four AMS: adding E there gives it no bay. It is pointed
    # to the swap and FORGET, and nothing promises it the bay it is on.
    _record(tmp_path, FOUR)
    m, p, _a, _s = _mk_files(tmp_path, roster=FOUR, **pool)
    m.logger = _Logger()
    _claimable(m, p, monkeypatch)
    _on_the_wire(m, monkeypatch, {A, B, C, E, H}, uids=(A, B, C, E, H))
    for t in range(100, 101 + int(m.enroll_grace)):
        m._scout_tick(float(t))
    assert f"boxed:{E}" in m._state_get(SEC, "roster")
    about_e = [x for x in m.logger.lines if E in x]
    assert not [x for x in about_e if "to enroll it" in x]
    assert not [x for x in about_e if "keep its bay" in x]
    (told,) = [x for x in about_e if "has no bay" in x]
    assert "all 4 AMS bays belong to other units" in told
    assert (f"Bambu_AMS_4 ({D}) is offline: if this AMS replaces it, replace "
            f"its entry in roster: with boxed:{E}, run AFC_BRIDGEBOX_FORGET "
            f"CHAIN=chain1 UID={D}, and RESTART.") in told
    if pool is POOL:
        # The claim said it; the enrollment line only names it recorded.
        assert told.startswith(f"AFC_BridgeBox chain1: AMS {E} has no bay: ")
        assert (f"AFC_BridgeBox chain1: NEW unit(s) on the chain: boxed:{E} "
                f"-- recorded.") in m.logger.lines
    else:
        assert told.startswith(f"AFC_BridgeBox chain1: NEW AMS on the chain: "
                               f"boxed:{E} -- recorded, but it has no bay: ")
        assert len(about_e) == 1


def test_without_a_pool_a_new_ams_in_an_auto_removed_units_place_enrolls(
        tmp_path, monkeypatch):
    # D was auto-removed from the recorded roster, so a restart gives E its
    # bay: the enrollment line says RESTART, not that no bay is left.
    _record(tmp_path, FOUR)
    m, _p = _boot(tmp_path, pool_ams=0, pool_ht=0)
    m.logger = _Logger()
    _record(tmp_path, f"boxed:{A}, boxed:{B}, boxed:{C}, ht:{H}")
    _on_the_wire(m, monkeypatch, {A, B, C, E, H}, uids=(A, B, C, E, H))
    m._scout_tick(100.0)
    m._scout_tick(100.0 + m.enroll_grace)
    (enroll,) = [x for x in m.logger.lines if E in x]
    assert (f"NEW unit(s) on the chain: boxed:{E} -- recorded. RESTART to "
            f"enroll.") in enroll
    _m2, p2 = _boot(tmp_path, pool_ams=0, pool_ht=0)
    assert _uids(p2)["Bambu_AMS_4"] == E


# ── a loaded lane that changes unit at boot ─────────────────────────────────

def _five_bay_state(tmp_path):
    """E recorded on Bambu_AMS_5 (lane40-lane43), the HT on lane44: this
    boot gives lane40 to the HT and leaves E waiting."""
    _record(tmp_path, FOUR + f", boxed:{E}",
            name_map=(f"{A}:Bambu_AMS_1, {B}:Bambu_AMS_2, {C}:Bambu_AMS_3, "
                      f"{D}:Bambu_AMS_4, {E}:Bambu_AMS_5, "
                      f"{H}:Bambu_AMS_HT_1"),
            lane_map=(f"{A}:24:4, {B}:28:4, {C}:32:4, {D}:36:4, {E}:40:4, "
                      f"{H}:44:1"))


def _new_owner(master, lanes, loaded):
    """The claimed unit's side of the follower restore, as a shim."""
    from extras.AFC_BambuAMS import afcBambuAMS
    engaged = []
    unit = types.SimpleNamespace(
        name="Bambu_AMS_HT_1", logger=_Logger(), _bridge=object(),
        _id_resolved=True, _master=master,
        lanes={n: types.SimpleNamespace(name=n, tool_loaded=False)
               for n in lanes},
        afc=types.SimpleNamespace(
            tools={"extruder": types.SimpleNamespace(lane_loaded=loaded)},
            reactor=_FakeReactor()),
        _startup_restore_loaded=lambda: engaged.append(True))
    return (lambda: afcBambuAMS._restore_loaded_follower(unit)), unit, engaged


class TestALoadedLaneThatChangesUnit:
    def _chain(self, tmp_path, loaded, prep_done=True):
        _five_bay_state(tmp_path)
        m, p = _boot(tmp_path)
        m.logger = _Logger()
        ext = types.SimpleNamespace(lane_loaded=loaded)
        p.objects["AFC"] = types.SimpleNamespace(
            prep_done=prep_done, tools={"extruder": ext})
        return m, p, ext

    def test_the_layout_records_whose_lanes_change_hands(self, tmp_path):
        m, _p, _ext = self._chain(tmp_path, None)
        assert m._lane_moves["lane40"] == {
            "uid": E, "family": "ams", "name": "Bambu_AMS_5",
            "now": "Bambu_AMS_HT_1"}
        assert m._lane_moves["lane41"]["now"] == "Bambu_AMS_HT_2"   # spare
        assert m._lane_moves["lane44"] == {
            "uid": H, "family": "ht", "name": "Bambu_AMS_HT_1", "now": None}
        assert "lane24" not in m._lane_moves and "lane39" not in m._lane_moves

    def test_the_new_owner_leaves_the_follower_off_and_the_user_is_told(
            self, tmp_path):
        m, _p, ext = self._chain(tmp_path, "lane40")
        restore, unit, engaged = _new_owner(m, ["lane40"], "lane40")
        assert restore() is False
        assert unit.lanes["lane40"].tool_loaded is False
        assert engaged == []
        m._check_moved_loaded()
        m._check_moved_loaded()
        (warn,) = m.logger.lines
        assert (f"extruder records lane40 as loaded, but lane40 belonged to "
                f"AMS {E} (Bambu_AMS_5), and this boot's layout gives it to "
                f"Bambu_AMS_HT_1. The filament in extruder is from {E}, so "
                f"Bambu_AMS_HT_1 leaves its follower off on lane40.") in warn
        assert "UNSET_LANE_LOADED" in warn
        # Lanes no record names stop being tracked; the loaded one stays.
        assert set(m._lane_moves) == {"lane40"}
        # Once the record is cleared, a lane loaded from the new owner is
        # the new owner's.
        ext.lane_loaded = None
        m._check_moved_loaded()
        assert not m.loaded_lane_moved("lane40")
        restore, unit, engaged = _new_owner(m, ["lane40"], "lane40")
        assert restore() is True and engaged == [True]
        assert unit.lanes["lane40"].tool_loaded is True

    def test_nothing_is_decided_before_prep_restores_the_records(
            self, tmp_path):
        m, _p, _ext = self._chain(tmp_path, "lane40", prep_done=False)
        m._check_moved_loaded()
        assert m.logger.lines == []
        assert m.loaded_lane_moved("lane40") and m.loaded_lane_moved("lane41")

    def test_a_record_on_a_lane_no_bay_holds_is_told_too(self, tmp_path):
        m, _p, _ext = self._chain(tmp_path, "lane44")
        m._scout_tick(100.0)           # the watch runs the check
        (warn,) = [x for x in m.logger.lines if "lane44" in x]
        assert (f"lane44 belonged to HT {H} (Bambu_AMS_HT_1), and this boot's "
                f"layout gives it to no bay. The filament in extruder is from "
                f"{H}. Unload") in warn

    def test_an_unmoved_layout_tracks_nothing(self, tmp_path):
        _record(tmp_path, FOUR)
        _boot(tmp_path)
        m, _p = _boot(tmp_path)
        assert m._lane_moves == {}
        assert not m.loaded_lane_moved("lane40")
