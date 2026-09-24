# Tests for the stale dry-countdown latch.
from __future__ import annotations
import types
import pytest
from extras.AFC_BambuAMS import afcBambuAMS


class _Logger:
    def __init__(self): self.msgs = []
    def _rec(self, m, *a, **k): self.msgs.append(str(m))
    info = debug = warning = error = raw = _rec


def _unit(now=1000.0):
    u = types.SimpleNamespace()
    u.name = "Bambu_AMS_HT_1"
    u.logger = _Logger()
    u._drying = False
    u._dryrem_seen = None
    u._dry_adopt_after = 0.0
    u._dryrem_at_stop = None
    u.afc = types.SimpleNamespace(reactor=types.SimpleNamespace(monotonic=lambda: now))
    return u


def _say(u, dr):
    return afcBambuAMS._dryrem_says_drying(u, dr)


def test_a_frozen_countdown_never_arms_drying_from_cold():
    # THE BUG. A cycle that ended leaves a positive countdown frozen in the
    # firmware; after a Klipper restart the stop-stamp that guarded against it
    # is gone, so the ghost re-armed drying and every load on the unit was
    # refused with "its heater is running" at a cold, idle HT.
    u = _unit()
    for _ in range(10):
        assert _say(u, 28766) is False
    assert u._drying is False
    assert not u.logger.msgs


def test_a_ticking_countdown_is_adopted_and_says_so():
    u = _unit()
    assert _say(u, 28766) is False          # first sight proves nothing
    assert _say(u, 28765) is True           # it moved: a real cycle
    assert u._drying is True
    assert any("adopting the unit's own dry cycle" in m for m in u.logger.msgs)


def test_a_cycle_we_started_still_reports_drying():
    u = _unit()
    u._drying = True
    assert _say(u, 28766) is True
    assert _say(u, 28766) is True           # frozen or not, it is ours


def test_the_stop_stamp_still_wins_over_a_frozen_echo():
    u = _unit()
    u._dryrem_at_stop = 500                 # we stopped a cycle at 500s
    assert _say(u, 500) is False            # the echo of it must not re-arm
    assert _say(u, 500) is False
    assert u._drying is False


def test_the_adopt_grace_still_holds_off_a_fresh_start():
    u = _unit()
    u._dry_adopt_after = 2000.0             # in the future: still settling
    assert _say(u, 28766) is False
    assert _say(u, 28765) is False          # even a tick waits for the grace
    assert u._drying is False
