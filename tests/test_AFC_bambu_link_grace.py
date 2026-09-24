"""
The silence clock behind link_loss_pause_s, and the grace a fresh link gets.

_last_frame_t used to be set when the socket opened, which made a connection
that opened and then said nothing count as the bridge having spoken. A flapping
link therefore re-armed the pause watchdog on every reconnect: measured
2026-09-06 on pp-next, reconnects 5.0-5.4s apart against a 5s pause threshold,
none of them passing traffic.

Now the clock moves only when a frame arrives, and a fresh connection may
discount its first CONNECT_GRACE_S -- but only if the PREVIOUS connection
actually spoke, so a mute flapper gets the allowance once rather than forever.
"""

import types

import pytest

import extras.AFC_BambuAMS_bridge as mod
from extras.AFC_BambuAMS_bridge import BambuBridge, CONNECT_GRACE_S


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


@pytest.fixture
def clock(monkeypatch):
    c = _Clock()
    monkeypatch.setattr(mod.time, "monotonic", c)
    return c


def _bridge():
    """A bridge built through __init__, with no port and no threads."""
    class _Reactor:
        def monotonic(self):
            return 0.0

        def register_async_callback(self, cb):
            cb(0.0)

    logger = types.SimpleNamespace(warning=lambda m: None, info=lambda m: None,
                                   debug=lambda m: None)
    b = BambuBridge(lambda: None, _Reactor(), logger)
    b._run = True
    return b


def _connect(b, port=object()):
    """What the reader does when a connection comes up."""
    b._serial = port
    BambuBridge._mark_connected(b)


def _frame(b, clock):
    """What the reader does when a chunk arrives."""
    b._last_frame_t = clock.t
    b._spoke_since_connect = True


# ── silent_for ────────────────────────────────────────────────────────────────

class TestSilentFor:
    def test_none_before_anything_has_ever_arrived(self, clock):
        b = _bridge()
        _connect(b)
        assert BambuBridge.silent_for(b) is None, (
            "opening a socket is not the bridge speaking")

    def test_it_counts_from_the_last_frame(self, clock):
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        clock.advance(3.0)
        assert BambuBridge.silent_for(b) == pytest.approx(3.0)

    def test_a_reconnect_after_speaking_gets_the_grace(self, clock):
        b = _bridge()
        _connect(b)
        _frame(b, clock)                     # this connection spoke
        clock.advance(30.0)                  # ... then the link died
        _connect(b)                          # and came back
        clock.advance(0.5)
        # Inside the grace the count is held down to the age of the link, not
        # the 30.5s of real silence, so the pause cannot fire on the handshake.
        assert BambuBridge.silent_for(b) == pytest.approx(0.5)

    def test_the_grace_expires_and_the_real_silence_shows_through(self, clock):
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        clock.advance(30.0)
        _connect(b)
        clock.advance(CONNECT_GRACE_S + 0.5)
        # Not a reset: the clock underneath kept running.
        expected = 30.0 + CONNECT_GRACE_S + 0.5
        assert BambuBridge.silent_for(b) == pytest.approx(expected)

    def test_a_mute_connection_earns_no_second_grace(self, clock):
        """The flapper case. The first reconnect is allowed its grace; because
        that connection never spoke, the next one is not."""
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        clock.advance(10.0)
        _connect(b)                          # grace 1, earned by the frame
        assert BambuBridge.silent_for(b) == pytest.approx(0.0)
        clock.advance(1.0)                   # says nothing, drops, reconnects
        _connect(b)                          # grace 2 -- must NOT be granted
        assert BambuBridge.silent_for(b) == pytest.approx(11.0), (
            "a link that keeps connecting mutely must not re-arm the watchdog")

    def test_speaking_again_re_earns_the_grace(self, clock):
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        clock.advance(10.0)
        _connect(b)
        _frame(b, clock)                     # this one did speak
        clock.advance(5.0)
        _connect(b)
        clock.advance(0.5)
        assert BambuBridge.silent_for(b) == pytest.approx(0.5)

    def test_a_dropped_link_is_measured_plainly(self, clock):
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        clock.advance(4.0)
        BambuBridge._drop_port(b)
        clock.advance(1.0)
        assert b._connected_t is None
        assert BambuBridge.silent_for(b) == pytest.approx(5.0)


# ── _drop_if_silent ──────────────────────────────────────────────────────────

class TestDropIfSilent:
    def test_a_fresh_link_is_never_dropped_for_older_silence(self, clock):
        """Without the floor this is a drop/reconnect loop: the clock is no
        longer reset on connect, so a reconnect after a long outage would
        inherit the old stamp and be dropped on its first tick."""
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        clock.advance(BambuBridge.QUIET_DROP_S * 3)      # a long outage
        dropped = []
        b._drop_port = lambda: dropped.append(True)
        _connect(b)                                      # back up
        clock.advance(1.0)
        assert BambuBridge._drop_if_silent(b) is False
        assert dropped == []

    def test_an_open_link_that_stays_quiet_is_still_dropped(self, clock):
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        dropped = []
        b._drop_port = lambda: dropped.append(True)
        clock.advance(BambuBridge.QUIET_DROP_S + 1.0)
        assert BambuBridge._drop_if_silent(b) is True
        assert dropped == [True]

    def test_a_quiet_link_is_dropped_on_its_own_age_after_a_reconnect(self, clock):
        """The floor delays the watchdog, it does not disable it: a reconnected
        link that never speaks is dropped QUIET_DROP_S after it opened."""
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        clock.advance(100.0)
        _connect(b)
        dropped = []
        b._drop_port = lambda: dropped.append(True)
        clock.advance(BambuBridge.QUIET_DROP_S - 1.0)
        assert BambuBridge._drop_if_silent(b) is False
        clock.advance(2.0)
        assert BambuBridge._drop_if_silent(b) is True

    def test_nothing_is_dropped_during_a_firmware_transfer(self, clock):
        b = _bridge()
        _connect(b)
        _frame(b, clock)
        b._fw_raw = True
        dropped = []
        b._drop_port = lambda: dropped.append(True)
        clock.advance(BambuBridge.QUIET_DROP_S * 2)
        assert BambuBridge._drop_if_silent(b) is False
        assert dropped == []
