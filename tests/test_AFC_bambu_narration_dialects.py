# Tests for the narration dialects of the three AMS models.
#
# The bridge decides what a unit did by matching its narration. The three models
# say the same things in three dialects -- STEP+colon on the boxed units,
# STEP+digit+comma on the HT, [AMS_DEV] rather than [AMS_RFID] on an AMS 1 --
# and a pattern anchored on one of them matches NOTHING on the other two while
# still compiling, still matching its own model, and still reading as working
# code. Every dialect bug this project has had was invisible in exactly that
# way.
#
# So the dialect is asserted here against real lines, in fixtures under
# fixtures/ams_narration/, rather than described in a document. See that
# directory's README for the file format and for how to add a capture.
from __future__ import annotations

from pathlib import Path

import pytest

from extras.AFC_BambuAMS_bridge import (
    _RFID_CYCLE_END_RE,
    _RFID_FOREIGN_TAG_RE,
    _RFID_INFLIGHT_RE,
    _RFID_READ_OK_RE,
)

FIXTURE_DIR = Path(__file__).parent / "fixtures" / "ams_narration"

#: marker in a fixture's `expect` column -> the pattern it names
PATTERNS = {
    "read": _RFID_READ_OK_RE,
    "end": _RFID_CYCLE_END_RE,
    "inflight": _RFID_INFLIGHT_RE,
    "foreign": _RFID_FOREIGN_TAG_RE,
}


class Line:
    """One narration line from a fixture, with what should be made of it."""

    def __init__(self, fixture: str, index: int, offset: str, expect: set,
                 text: str):
        self.fixture = fixture
        self.index = index
        self.offset = offset
        self.expect = expect
        self.text = text

    def __repr__(self):
        return f"{self.fixture}[{self.index}]"


class Capture:
    """A fixture file: its header metadata plus its lines, in order."""

    def __init__(self, path: Path):
        self.name = path.stem
        self.meta = {}
        self.lines = []
        for raw in path.read_text().splitlines():
            raw = raw.strip()
            if not raw:
                continue
            if raw.startswith("#"):
                body = raw.lstrip("#").strip()
                key, sep, value = body.partition(":")
                if sep and key in ("model", "outcome", "source", "verbatim"):
                    self.meta.setdefault(key, value.strip())
                continue
            offset, _, rest = raw.partition("|")
            expect, _, text = rest.partition("|")
            expect = expect.strip()
            self.lines.append(Line(
                self.name, len(self.lines), offset.strip(),
                set() if expect == "." else set(expect.split()), text.strip()))

    @property
    def model(self):
        return self.meta.get("model", "")

    @property
    def outcome(self):
        return self.meta.get("outcome", "")

    def first(self, marker: str):
        """Index of the first line expecting `marker`, or None."""
        for line in self.lines:
            if marker in line.expect:
                return line.index
        return None

    def __repr__(self):
        return self.name


def _load():
    caps = [Capture(p) for p in sorted(FIXTURE_DIR.glob("*.txt"))]
    assert caps, f"no narration fixtures found under {FIXTURE_DIR}"
    return caps


CAPTURES = _load()
ALL_LINES = [ln for cap in CAPTURES for ln in cap.lines]


class TestTheFixturesAreWellFormed:
    """A malformed fixture must fail loudly rather than silently test nothing.

    An `expect` typo would otherwise read as "this line matches no pattern",
    which is a valid expectation for most lines -- so the whole file could go
    green while asserting nothing at all."""

    @pytest.mark.parametrize("cap", CAPTURES, ids=repr)
    def test_the_header_declares_a_model_and_an_outcome(self, cap):
        assert cap.model in ("ams1", "ams2", "ht"), cap.meta
        assert cap.outcome in ("read", "notag", "foreign"), cap.meta
        assert cap.meta.get("source"), "a fixture without provenance is folklore"
        # The keyword is the first word; anything after it is prose saying what
        # was reconstructed and from where.
        verbatim = cap.meta.get("verbatim", "").split()[:1]
        assert verbatim and verbatim[0] in ("yes", "abbreviated"), cap.meta

    @pytest.mark.parametrize("line", ALL_LINES, ids=repr)
    def test_every_marker_names_a_real_pattern(self, line):
        unknown = line.expect - set(PATTERNS)
        assert not unknown, f"unknown marker(s) {unknown} on {line}: {line.text}"
        assert line.text, f"{line} has no narration text"


class TestEveryLineClassifiesAsTheFixtureRecords:
    """The drift lock: each real line matches exactly the patterns claimed.

    This catches a dialect regression in BOTH directions -- a pattern that stops
    firing on a model it used to serve, and one that starts firing on a line it
    should not. The second matters as much as the first: "STEP7:" on its own is
    not terminal, and matching the bare prefix would end a cycle before the
    measurement runs."""

    @pytest.mark.parametrize("line", ALL_LINES, ids=repr)
    def test_a_line_matches_exactly_what_it_claims(self, line):
        got = {name for name, pat in PATTERNS.items() if pat.search(line.text)}
        assert got == line.expect, (
            f"{line.fixture} line {line.index} ({line.offset}):\n"
            f"  {line.text}\n"
            f"  fixture expects: {sorted(line.expect) or ['.']}\n"
            f"  patterns give:   {sorted(got) or ['.']}")


class TestTheCycleReachesTheOutcomeTheFixtureDeclares:
    """A scan the unit completed must be classifiable from its narration alone.

    `_scan_verdict` reads nothing else: no record contents, no clock. If a
    model's successful read produces no `read` line, or an empty bay produces no
    `end`, then that model has no way to resolve a scan and the bay waits for
    the backstop."""

    @pytest.mark.parametrize("cap", [c for c in CAPTURES if c.outcome == "read"],
                             ids=repr)
    def test_a_successful_scan_narrates_a_read(self, cap):
        assert cap.first("read") is not None, (
            f"{cap.name}: the unit read a tag and nothing in its narration says "
            f"so, so _scan_verdict can only wait for SCAN_FALLBACK_CAP")

    @pytest.mark.parametrize("cap", [c for c in CAPTURES if c.outcome == "notag"],
                             ids=repr)
    def test_an_empty_bay_narrates_an_end_and_never_a_read(self, cap):
        assert cap.first("end") is not None, (
            f"{cap.name}: nothing marks the end of the cycle, so 'no tag' can "
            f"never become a fact and the lane keeps the last spool's profile")
        assert cap.first("read") is None, (
            f"{cap.name}: an empty bay narrated a READ -- the lane would take "
            f"the previous spool's record as this spool's")

    @pytest.mark.parametrize("cap",
                             [c for c in CAPTURES if c.outcome == "foreign"],
                             ids=repr)
    def test_a_foreign_chip_is_told_apart_from_an_empty_bay(self, cap):
        assert cap.first("foreign") is not None, cap.name
        assert cap.first("read") is None, cap.name


class TestACycleNeverEndsBeforeItReads:
    """THE ORDERING INVARIANT. `_scan_verdict` asks "did it read?" and then "did
    it finish?", so a terminal marker that fires BEFORE the read resolves the
    scan as `notag`: the lane takes AFC defaults, the scan closes, and the real
    tag arrives afterwards through `_surface_slot_info` with no scan open to
    hold it. The operator sees the tag land, late.

    A pattern is only terminal if it cannot appear mid-cycle. That is checked
    here rather than argued in a comment."""

    @pytest.mark.parametrize("cap", [
        pytest.param(c, marks=pytest.mark.xfail(strict=True, reason=(
            "AMS 1: 'odom calib success' is in _RFID_CYCLE_END_RE as this "
            "model's terminal marker, but the boxed odometer calibration runs "
            "on the INSERT EDGE -- measured at +5.571s against a read at "
            "+37.980s. It is the start of the scan, not the end. Awaiting a "
            "timestamped host log to confirm the same on an AMS 2 before the "
            "pattern is changed."))
        ) if c.name == "ams1_insert_tagged" else c
        for c in CAPTURES if c.outcome == "read"
    ], ids=repr)
    def test_the_terminal_marker_does_not_fire_before_the_read(self, cap):
        first_read, first_end = cap.first("read"), cap.first("end")
        if first_end is None:
            return                      # nothing terminal in this capture
        assert first_read is not None and first_read <= first_end, (
            f"{cap.name}: the cycle ends at line {first_end} "
            f"({cap.lines[first_end].offset}) but does not read until line "
            f"{first_read} ({cap.lines[first_read].offset if first_read is not None else '-'})"
            f"\n  end:  {cap.lines[first_end].text}"
            f"\n  read: {cap.lines[first_read].text if first_read is not None else '(never)'}"
            f"\n  _scan_verdict resolves this scan as 'notag'.")


class TestTheSameEventIsRecognisedAcrossDialects:
    """The cross-model coverage table, asserted rather than described.

    Each model announces the same three milestones in its own words. Where a
    milestone is recognised on one model and not another, the module's behaviour
    silently differs by hardware -- which is the shape of every dialect bug this
    project has had. These assert the CURRENT state, so any change to it is
    deliberate and shows up as this test rather than as a machine behaving
    oddly."""

    #: the chip is open. No model's auth line counts as a read.
    AUTH = {
        "ams1": ("[AMS_DEV] STEP:card auth success!", False),
        "ams2": ("[AMS_RFID]STEP:card auth success!", False),
        "ht": ("[AMS_RFID] STEP3,auth card successful", False),
    }
    #: the unit has written the record to its own flash. Two spellings: the
    #: boxed models say "info write to flash", the HT "save to flash ,card info
    #: valid". Both count.
    COMMIT = {
        "ams1": ("[RF] tray0: info write to flash", True),
        "ams2": ("[RF] tray0: info write to flash", True),
        "ht": ("[AMS_RFID] STEP3,save to flash ,card info valid", True),
    }
    #: the read landed.
    READ = {
        "ams1": ("[AMS_DEV] STEP:read success,valid", True),
        "ams2": ("[AMS_RFID]STEP:feed with rfid success", True),
        "ht": ("[AMS_RFID] STEP3,read success ,goto Cali", True),
    }

    @pytest.mark.parametrize("model,line,recognised",
                             [(m, ln, ok) for m, (ln, ok) in READ.items()],
                             ids=lambda v: v if isinstance(v, str) else "")
    def test_every_model_has_a_read_the_pattern_recognises(
            self, model, line, recognised):
        # If this ever goes False for a model, that model cannot resolve a scan
        # except by timing out -- the HT's state for months.
        assert recognised, model
        assert bool(_RFID_READ_OK_RE.search(line)) is True, (
            f"{model}: '{line}' is this model's read, and the pattern is blind "
            f"to it")

    @pytest.mark.parametrize("model,line,recognised",
                             [(m, ln, ok) for m, (ln, ok) in COMMIT.items()],
                             ids=lambda v: v if isinstance(v, str) else "")
    def test_every_models_commit_sentence_counts_as_a_read(
            self, model, line, recognised):
        # A commit is the strongest statement a unit makes about a tag: it has
        # authenticated the chip and written the record it will now serve. Both
        # spellings are in _RFID_READ_OK_RE, so the read is recognised at the
        # commit rather than at some later sentence -- which is what decides
        # how promptly a tag reaches the lane.
        assert bool(_RFID_READ_OK_RE.search(line)) is recognised, (
            f"{model}: commit-sentence recognition changed. Update this table "
            f"and say why in the commit.")

    @pytest.mark.parametrize("model,line,recognised",
                             [(m, ln, ok) for m, (ln, ok) in AUTH.items()],
                             ids=lambda v: v if isinstance(v, str) else "")
    def test_no_model_treats_authentication_alone_as_a_read(
            self, model, line, recognised):
        # Authenticating the chip is not reading it: the HT can auth and still
        # be serving its flash cache. Deliberate, and asserted so it does not
        # get "fixed" into the read pattern by someone chasing a slow read.
        assert bool(_RFID_READ_OK_RE.search(line)) is recognised, model
