# Flow-level tests for extras/AFC_BambuAMS.py — the paths that need a REACTOR
# that can be paused, and a bridge that answers.
#
# The existing test file covers parsing and decision logic with duck-typed
# shims. What it cannot reach is everything that blocks: the synchronous UID
# resolution at startup, the move-completion wait, and the reconnect/announce
# handshake. Those are exactly the paths where a wrong answer is expensive --
# a unit that resolves to the wrong chain index drives the wrong physical AMS,
# and a move wait that returns True on a stall reports a load that never
# happened.
#
# The harness here is a reactor whose pause() ADVANCES time, so a bounded
# retry/timeout loop terminates in a test instead of spinning.
from __future__ import annotations

import types

import pytest

from extras import AFC_BambuAMS as bambu_mod
from extras.AFC_BambuAMS import afcBambuAMS


class _PausingReactor:
    """Reactor whose pause() advances monotonic time.

    Without this, every `while monotonic() < end: pause(...)` loop in the
    module is untestable -- it either spins forever or needs the loop rewritten
    for the benefit of the test, which is worse.
    """

    NOW = 0.0

    def __init__(self):
        self._now = 100.0
        self.async_cbs = []
        self.callbacks = []

    def monotonic(self):
        return self._now

    def pause(self, until):
        self._now = max(self._now, until)
        return self._now

    def register_async_callback(self, cb):
        self.async_cbs.append(cb)

    def register_callback(self, cb, when=None):
        self.callbacks.append((cb, when))

    def register_timer(self, cb, when=None):
        return cb

    def update_timer(self, *a):
        pass


class _Bridge:
    """Bridge stand-in: records sends, replays a chain map and finish events."""

    def __init__(self, uids=(), finish=(0, False, "")):
        self.sent = []
        self._uids = list(uids)
        self._finish = finish
        self._status = None

    def send(self, obj):
        self.sent.append(obj)

    def chain_uids(self):
        return list(self._uids)

    def last_finish(self):
        return self._finish

    def latest_status(self):
        return self._status

    def cmds(self):
        return [s.get("cmd") for s in self.sent]


class _Logger:
    def __init__(self):
        self.messages = []

    def info(self, m):
        self.messages.append(("info", m))

    def warning(self, m):
        self.messages.append(("warning", m))

    def error(self, m, **k):
        self.messages.append(("error", m))

    def debug(self, m, only_debug=False, traceback=None):
        self.messages.append(("debug", m))


def _unit(bridge=None, **kw):
    """A duck-typed afcBambuAMS with just enough state for the flow under test."""
    u = afcBambuAMS.__new__(afcBambuAMS)
    u.name = kw.pop("name", "BambuAMS_1")
    u._bridge = bridge
    u.unit_uid = kw.pop("unit_uid", "")
    u.ams_index = kw.pop("ams_index", 0)
    u.logger = _Logger()
    u.max_speed = 30.0
    reactor = kw.pop("reactor", None) or _PausingReactor()
    u.afc = types.SimpleNamespace(reactor=reactor)
    u.reactor = reactor
    for k, v in kw.items():
        setattr(u, k, v)
    return u


class TestResolveUidBlocking:
    """unit_uid -> ams_index must be resolved BEFORE lanes are seeded, or PREP
    reads the wrong physical unit. It runs synchronously at klippy:ready and
    falls back to the async retry when the chain is slow."""

    def test_no_bridge_is_not_resolved(self):
        u = _unit(None, unit_uid="AABB")
        assert afcBambuAMS._resolve_uid_blocking(u) is False

    def test_no_uid_configured_is_not_resolved(self):
        u = _unit(_Bridge(uids=["AABB"]), unit_uid="")
        assert afcBambuAMS._resolve_uid_blocking(u) is False

    def test_uid_present_adopts_its_chain_index(self):
        b = _Bridge(uids=["ZZZZ", "AABB", "CCCC"])
        u = _unit(b, unit_uid="AABB", ams_index=0)
        adopted = []
        u._adopt_index = lambda i: adopted.append(i)
        assert afcBambuAMS._resolve_uid_blocking(u, timeout=5.0) is True
        assert adopted == [1]                    # position in the chain
        assert "chain" in b.cmds()               # it asked for the map

    def test_uid_absent_times_out_without_adopting(self):
        b = _Bridge(uids=["ZZZZ"])
        u = _unit(b, unit_uid="AABB")
        adopted = []
        u._adopt_index = lambda i: adopted.append(i)
        assert afcBambuAMS._resolve_uid_blocking(u, timeout=2.0) is False
        assert adopted == []

    def test_it_rerequests_the_map_rather_than_asking_once(self):
        # A single request that arrives while the Pico is still enumerating
        # would strand the unit on its config index for the whole session.
        b = _Bridge(uids=[])
        u = _unit(b, unit_uid="AABB")
        u._adopt_index = lambda i: None
        afcBambuAMS._resolve_uid_blocking(u, timeout=5.0)
        assert b.cmds().count("chain") >= 2

    def test_a_throwing_bridge_is_survived(self):
        b = _Bridge(uids=[])
        b.chain_uids = lambda: (_ for _ in ()).throw(RuntimeError("down"))
        u = _unit(b, unit_uid="AABB")
        u._adopt_index = lambda i: None
        assert afcBambuAMS._resolve_uid_blocking(u, timeout=1.0) is False


class TestWaitMove:
    """The AMS reports move COMPLETION in its own narration; the bridge turns
    that into a (sequence, ok) pair. Returning True on a stall would report a
    load that never happened."""

    def _u(self, finish, reactor=None):
        b = _Bridge(finish=finish)
        u = _unit(b, reactor=reactor)
        return u, b

    def test_returns_ok_when_a_new_completion_says_ok(self):
        u, b = self._u((5, True, "feed finish"))
        b._finish = (5, True, "")            # start_seq
        # Bump the sequence on the first poll, as a real completion would.
        seq = {"n": 5}

        def finish():
            seq["n"] += 1
            return (seq["n"], True, "feed finish")
        b.last_finish = lambda: (5, True, "") if not seq.pop("armed", None) else finish()
        seq["armed"] = True
        assert afcBambuAMS._wait_move(u, 100.0, 20.0) is True

    def test_returns_false_when_the_completion_reports_a_stall(self):
        b = _Bridge(finish=(1, True, ""))
        u = _unit(b)
        state = {"first": True}

        def lf():
            if state["first"]:
                state["first"] = False
                return (1, True, "")
            return (2, False, "feed finish -1, stall")
        b.last_finish = lf
        assert afcBambuAMS._wait_move(u, 100.0, 20.0) is False

    def test_times_out_to_false_when_nothing_completes(self):
        b = _Bridge(finish=(7, True, ""))
        u = _unit(b)
        assert afcBambuAMS._wait_move(u, 10.0, 20.0) is False

    def test_no_bridge_times_out_to_false(self):
        u = _unit(None)
        assert afcBambuAMS._wait_move(u, 10.0, 20.0) is False

    def test_zero_speed_does_not_divide_by_zero(self):
        b = _Bridge(finish=(1, True, ""))
        u = _unit(b)
        assert afcBambuAMS._wait_move(u, 10.0, 0.0) is False


class TestAnnounceUnit:
    """On connect and on every bridge reconnect the unit must re-announce
    itself: a Pico reboot (reflash, power-cycle, replug) resets the unit count,
    the HT flag, the self-centre flag and the MC address to factory defaults."""

    def test_no_bridge_is_a_noop(self):
        u = _unit(None)
        afcBambuAMS._announce_unit(u)          # must not raise

    def test_sends_the_full_handshake(self):
        b = _Bridge()
        u = _unit(b, ams_index=2)
        sent = []
        u._send_ht_flag = lambda br: sent.append("ht")
        u._send_selfcentre_flag = lambda br: sent.append("selfc")
        u._send_mc_addr = lambda br: sent.append("mcaddr")
        afcBambuAMS._announce_unit(u)
        assert {"cmd": "units", "n": 3} in b.sent      # index 2 -> 3 units
        assert sent == ["ht", "selfc", "mcaddr"]

    def test_a_throwing_send_is_swallowed(self):
        b = _Bridge()
        b.send = lambda o: (_ for _ in ()).throw(RuntimeError("closed"))
        u = _unit(b)
        u._send_ht_flag = lambda br: None
        u._send_selfcentre_flag = lambda br: None
        u._send_mc_addr = lambda br: None
        afcBambuAMS._announce_unit(u)          # must not raise


class TestOnBridgeReconnect:
    """The reconnect handshake. Two things must hold, and both have been got
    wrong on hardware: the FIRST connection reboots the Pico rather than
    announcing into whatever state it was left in, and the announce that
    follows waits for the firmware to come up before it is sent."""

    def _u(self, **kw):
        u = _unit(_Bridge(), **kw)
        u._announce_unit = lambda: u.logger.messages.append(("t", "announce"))
        u._resolve_uid_index = lambda n: None
        return u

    def test_first_connect_resets_and_does_not_announce_yet(self):
        u = self._u()
        afcBambuAMS._on_bridge_reconnect(u)
        assert {"cmd": "reset"} in u._bridge.sent
        assert ("t", "announce") not in u.logger.messages
        assert u.afc.reactor.callbacks == []

    def test_the_reconnect_after_the_reset_defers_the_announce(self):
        u = self._u()
        afcBambuAMS._on_bridge_reconnect(u)          # resets
        afcBambuAMS._on_bridge_reconnect(u)          # cooldown -> announce path
        # Deferred, NOT sent inline: the firmware is still booting.
        assert ("t", "announce") not in u.logger.messages
        assert len(u.afc.reactor.callbacks) == 1
        cb, when = u.afc.reactor.callbacks[0]
        assert when == pytest.approx(
            u.afc.reactor.monotonic() + bambu_mod.ANNOUNCE_SETTLE_S)
        cb(when)
        assert ("t", "announce") in u.logger.messages

    def test_settle_delay_is_long_enough_to_outlast_a_usb_reenumeration(self):
        # A Pico re-enumerates in well under a second; anything shorter than
        # this is the race we are here to close.
        assert bambu_mod.ANNOUNCE_SETTLE_S >= 0.5

    def test_a_reactor_that_cannot_defer_still_announces(self):
        u = self._u()
        u._bridge._last_reset_t = 1e9                # pretend cooldown active
        u.afc.reactor.monotonic = lambda: 1e9
        u.afc.reactor.register_callback = lambda *a, **k: (
            _ for _ in ()).throw(RuntimeError("no reactor"))
        afcBambuAMS._on_bridge_reconnect(u)
        assert ("t", "announce") in u.logger.messages

    def test_the_announce_asks_for_a_status_frame(self):
        u = self._u()
        afcBambuAMS._announce_after_settle(u)
        assert {"cmd": "status"} in u._bridge.sent

    def test_a_dead_bridge_during_the_settle_does_not_raise(self):
        u = self._u()
        u._bridge.send = lambda o: (_ for _ in ()).throw(RuntimeError("gone"))
        afcBambuAMS._announce_after_settle(u)        # must not raise

    def test_uid_is_re_resolved_after_a_reboot(self):
        # The chain can re-enrol in a different order across a Pico reboot, so
        # a configured UID has to be re-looked-up or the unit drives the wrong
        # physical AMS.
        u = self._u(unit_uid="AABB")
        seen = []
        u._resolve_uid_index = lambda n: seen.append(n)
        afcBambuAMS._announce_after_settle(u)
        assert seen == [0]


class TestResetBridgeOnce:
    """The cooldown is load-bearing: the reset drops the USB link, which fires
    the reconnect handler, which calls this again. Without it that is a reboot
    loop. It lives on the BRIDGE because several units share one Pico."""

    def test_no_bridge_is_false(self):
        assert afcBambuAMS._reset_bridge_once(_unit(None)) is False

    def test_first_call_sends_the_reset(self):
        u = _unit(_Bridge())
        assert afcBambuAMS._reset_bridge_once(u) is True
        assert {"cmd": "reset"} in u._bridge.sent

    def test_second_call_inside_the_cooldown_is_refused(self):
        u = _unit(_Bridge())
        afcBambuAMS._reset_bridge_once(u)
        assert afcBambuAMS._reset_bridge_once(u) is False
        assert u._bridge.sent.count({"cmd": "reset"}) == 1

    def test_the_cooldown_is_shared_by_units_on_one_pico(self):
        b = _Bridge()
        first, second = _unit(b), _unit(b, name="BambuAMS_2")
        assert afcBambuAMS._reset_bridge_once(first) is True
        assert afcBambuAMS._reset_bridge_once(second) is False

    def test_a_failed_send_is_not_reported_as_a_reset(self):
        u = _unit(_Bridge())
        u._bridge.send = lambda o: (_ for _ in ()).throw(RuntimeError("closed"))
        assert afcBambuAMS._reset_bridge_once(u) is False


class TestFpsBufferValue:
    """The AMS buffer is published as a virtual FPS/PSF ADC, and the FPS driver
    documents the convention: low = stretched/tension, mid = centred, HIGH =
    COMPRESSED (its aliases are max_tension -> low_point and max_compression ->
    high_point).

    This returned the inverse for a long time, chosen to make a display label
    read nicely, and it silently inverted everything that DECIDES something --
    advance_state, buffer_triggered, the pre-feed guard. An empty buffer
    reported filament at the toolhead, which is why ramming could not work."""

    def _u(self, status):
        b = _Bridge()
        b._status = status
        return _unit(b)

    def test_no_bridge_is_none(self):
        assert afcBambuAMS.fps_buffer_value(_unit(None)) is None

    def test_no_status_yet_is_none(self):
        assert afcBambuAMS.fps_buffer_value(self._u(None)) is None

    def test_status_without_buff_is_none(self):
        assert afcBambuAMS.fps_buffer_value(self._u({"online": True})) is None

    def test_compressed_maps_to_ONE(self):
        # max_compression -> high_point. The whole point.
        assert afcBambuAMS.fps_buffer_value(self._u({"buff": 100})) == 1.0

    def test_empty_maps_to_ZERO(self):
        # max_tension -> low_point: an empty buffer is under tension.
        assert afcBambuAMS.fps_buffer_value(self._u({"buff": 0})) == 0.0

    def test_midpoint(self):
        assert afcBambuAMS.fps_buffer_value(self._u({"buff": 50})) == 0.5

    def test_a_loaded_self_centred_unit_reads_near_the_set_point(self):
        # Measured at rest, loaded and following: buff 56..60. A unit that
        # holds its own buffer centred should read just above the 0.5
        # set_point, and this is the physical check that the sign is right --
        # inverted it reads just BELOW, which looks almost as plausible.
        for b in (56, 57, 60):
            v = afcBambuAMS.fps_buffer_value(self._u({"buff": b}))
            assert 0.5 < v < 0.65

    def test_an_unloaded_unit_reads_as_tension_not_compression(self):
        # Measured with both lanes unloaded: buff 1..2. This is the reading
        # that used to come back as 0.99 and tell AFC the toolhead was full.
        for b in (1, 2):
            assert afcBambuAMS.fps_buffer_value(self._u({"buff": b})) < 0.1

    def test_out_of_range_readings_are_clamped(self):
        # The unit has been seen reporting outside 0..100; an ADC pin that
        # returns 1.4 would look like a broken sensor to the buffer code.
        assert afcBambuAMS.fps_buffer_value(self._u({"buff": 140})) == 1.0
        assert afcBambuAMS.fps_buffer_value(self._u({"buff": -40})) == 0.0


class TestClearLaneFilament:
    """A removed spool's profile must not linger, or a swap shows the previous
    spool's material until a new tag reads. Spoolman-linked lanes are left
    alone -- Spoolman is authoritative there."""

    def _lane(self, **kw):
        base = dict(material="PLA", color="FF0000", weight=1000,
                    bambu_sku="X", spool_id=None)
        base.update(kw)
        return types.SimpleNamespace(**base)

    def test_blanks_material_colour_weight_and_sku(self):
        lane = self._lane()
        afcBambuAMS._clear_lane_filament(_unit(None), lane)
        assert lane.material == "" and lane.color == ""
        assert lane.weight == 0 and lane.bambu_sku == ""

    def test_missing_attributes_are_tolerated(self):
        lane = types.SimpleNamespace()
        afcBambuAMS._clear_lane_filament(_unit(None), lane)   # must not raise


class TestFinalizeScanDeferral:
    """The 'no readable tag, apply lane defaults' fallback must not stomp a tag
    that is still landing. Measured on hardware: a real read finished 13.0s
    after the insert edge against a 14.0s fallback -- a one-second margin. So
    the fallback waits on the UNIT (narration showing a read in flight) rather
    than on a fixed clock, up to a hard cap."""

    def _u(self, *, present=True, material="", in_flight=False, cap=None,
           spool_id=None, lane_material=None):
        b = _Bridge()
        b.rfid_read_in_flight = lambda now, quiet=3.0: in_flight
        u = _unit(b)
        u._slots = [{"present": present, "material": material}]
        u._slot_map = {"lane1": 0}
        lane = types.SimpleNamespace(spool_id=spool_id, material=lane_material,
                                     name="lane1")
        u.lanes = {"lane1": lane}
        u.lane = lane
        return u, lane

    def test_slot_out_of_range_is_ignored(self):
        u, _ = self._u()
        afcBambuAMS._finalize_scan(u, 9)             # must not raise

    def test_empty_bay_is_ignored(self):
        u, lane = self._u(present=False)
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.material is None                 # untouched

    def test_a_tag_that_read_in_time_is_left_alone(self):
        u, lane = self._u(material="PLA")
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.material is None                 # defaults never applied

    def test_a_read_in_flight_defers_instead_of_applying(self):
        u, lane = self._u(in_flight=True)
        u.afc.reactor._now = 100.0
        afcBambuAMS._finalize_scan(u, 0, cap=200.0)
        # It re-armed rather than applying anything.
        assert u.afc.reactor.callbacks, "should have rescheduled itself"
        cb, when = u.afc.reactor.callbacks[0]
        assert when == pytest.approx(102.0)          # +2s

    def test_past_the_cap_it_stops_deferring(self):
        u, lane = self._u(in_flight=True)
        u.afc.reactor._now = 300.0
        afcBambuAMS._finalize_scan(u, 0, cap=200.0)  # cap already passed
        assert u.afc.reactor.callbacks == []

    def test_a_spoolman_linked_lane_is_never_defaulted(self):
        # Spoolman is authoritative; overwriting it with tag defaults would
        # silently detach the lane from its tracked spool.
        u, lane = self._u(spool_id=42)
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.material is None

    def test_a_lane_that_already_has_material_is_left_alone(self):
        u, lane = self._u(lane_material="PETG")
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.material == "PETG"

    def test_unmapped_slot_is_ignored(self):
        u, _ = self._u()
        u._slot_map = {}
        afcBambuAMS._finalize_scan(u, 0)             # must not raise




class TestMcAddrAck:
    """The receipt for the announce. An mcaddr command that never arrives and
    one that arrives and is applied look identical from Klipper -- and that
    ambiguity IS the narration-drain fault, so the probe must show which."""

    def test_no_bridge_is_unknown(self):
        assert afcBambuAMS._mcaddr_ack_str(_unit(None)) == "?"

    def test_old_bridge_without_the_accessor_is_unknown(self):
        assert afcBambuAMS._mcaddr_ack_str(_unit(_Bridge())) == "?"

    def test_never_acknowledged_is_none_not_zero(self):
        b = _Bridge()
        b.mcaddr_ack = lambda u: None
        assert afcBambuAMS._mcaddr_ack_str(_unit(b)) == "none"

    def test_acknowledged_as_unset_is_zero_not_none(self):
        b = _Bridge()
        b.mcaddr_ack = lambda u: 0
        assert afcBambuAMS._mcaddr_ack_str(_unit(b)) == "0x0000"

    def test_an_applied_address_is_shown_in_hex(self):
        b = _Bridge()
        b.mcaddr_ack = lambda u: 0x1800
        assert afcBambuAMS._mcaddr_ack_str(_unit(b)) == "0x1800"

    def test_it_asks_about_its_own_chain_index(self):
        b = _Bridge()
        asked = []
        b.mcaddr_ack = lambda u: asked.append(u)
        afcBambuAMS._mcaddr_ack_str(_unit(b, ams_index=2))
        assert asked == [2]

    def test_a_throwing_bridge_is_unknown_not_a_crash(self):
        b = _Bridge()
        b.mcaddr_ack = lambda u: (_ for _ in ()).throw(RuntimeError("down"))
        assert afcBambuAMS._mcaddr_ack_str(_unit(b)) == "?"


class TestGcmdIntAcceptsHex:
    """Every bus address in this module's docs, comments and captures is
    written in hex, because that is how the protocol documents itself. Klipper's
    get_int is decimal-only, so BAMBU_DRAIN ADDR=0x0700 -- copied straight out
    of the command's own help text -- was rejected as unparsable."""

    class _G:
        """Klipper's GCodeCommand as far as this helper is concerned: get_int
        parses decimal only and raises on anything else, which is the exact
        behaviour being worked around."""

        def __init__(self, **kw):
            self._kw = kw
            self.error = RuntimeError

        def get(self, name, default=None):
            return self._kw.get(name, default)

        def get_int(self, name, default=None, minval=None, maxval=None):
            raw = self._kw.get(name)
            if raw is None:
                return default
            v = int(raw)                      # decimal only, like Klipper
            if ((minval is not None and v < minval)
                    or (maxval is not None and v > maxval)):
                raise RuntimeError("out of range")
            return v

        def get_commandline(self):
            return "BAMBU_DRAIN ADDR=?"

    def test_a_missing_parameter_takes_the_default(self):
        assert bambu_mod._gcmd_int(self._G(), "ADDR", 7, 0, 99) == 7

    def test_decimal_still_works(self):
        g = self._G(ADDR="1792")
        assert bambu_mod._gcmd_int(g, "ADDR", 0, 0, 0xFFFF) == 1792

    def test_hex_is_accepted(self):
        g = self._G(ADDR="0x0700")
        assert bambu_mod._gcmd_int(g, "ADDR", 0, 0, 0xFFFF) == 0x0700

    def test_hex_and_decimal_agree(self):
        hexed = bambu_mod._gcmd_int(self._G(ADDR="0x1800"), "ADDR", 0, 0, 0xFFFF)
        dec = bambu_mod._gcmd_int(self._G(ADDR="6144"), "ADDR", 0, 0, 0xFFFF)
        assert hexed == dec == 6144

    def test_surrounding_whitespace_is_tolerated(self):
        g = self._G(P=" 0x80 ")
        assert bambu_mod._gcmd_int(g, "P", 255, 0, 255) == 0x80

    def test_garbage_is_an_error_not_a_default(self):
        # Silently falling back to the default would send the drain somewhere
        # other than where the operator asked, which is worse than refusing.
        with pytest.raises(RuntimeError):
            bambu_mod._gcmd_int(self._G(ADDR="nonsense"), "ADDR", 0, 0, 0xFFFF)

    def test_out_of_range_is_refused(self):
        with pytest.raises(RuntimeError):
            bambu_mod._gcmd_int(self._G(P="0x1FF"), "P", 255, 0, 255)

    def test_the_bounds_are_inclusive(self):
        assert bambu_mod._gcmd_int(self._G(P="0xFF"), "P", 0, 0, 255) == 255
        assert bambu_mod._gcmd_int(self._G(P="0"), "P", 9, 0, 255) == 0


class TestBambuArmms:
    """The 11/04 follower keep-alive is the only per-cycle transmitter with no
    MUTE_* bit, so when a unit ticks at idle it is the one suspect BAMBU_MUTE
    cannot rule out. Winding its cadence out is the substitute, and unlike a
    mute bit it needs no reflash."""

    class _G:
        def __init__(self, **kw):
            self._kw = kw
            self.error = RuntimeError
            self.said = []

        def get(self, name, default=None):
            return self._kw.get(name, default)

        def get_int(self, name, default=None, minval=None, maxval=None):
            raw = self._kw.get(name)
            if raw is None:
                return default
            v = int(raw)
            if ((minval is not None and v < minval)
                    or (maxval is not None and v > maxval)):
                raise RuntimeError("out of range")
            return v

        def get_commandline(self):
            return "BAMBU_ARMMS"

        def respond_info(self, m):
            self.said.append(m)

    def _u(self):
        b = _Bridge()
        return _unit(b), b

    def test_no_bridge_is_an_error_not_a_silent_noop(self):
        u = _unit(None)
        with pytest.raises(RuntimeError):
            afcBambuAMS.cmd_BAMBU_ARMMS(u, self._G(MS="1000"))

    def test_it_sends_the_cadence(self):
        u, b = self._u()
        afcBambuAMS.cmd_BAMBU_ARMMS(u, self._G(MS="30000"))
        assert {"cmd": "armms", "ms": 30000} in b.sent

    def test_zero_restores_the_default_and_says_so(self):
        u, b = self._u()
        g = self._G(MS="0")
        afcBambuAMS.cmd_BAMBU_ARMMS(u, g)
        assert {"cmd": "armms", "ms": 0} in b.sent
        assert any("default" in m for m in g.said)

    def test_it_defaults_to_restoring(self):
        # No MS at all must not leave a bisect value stuck on the unit.
        u, b = self._u()
        afcBambuAMS.cmd_BAMBU_ARMMS(u, self._G())
        assert {"cmd": "armms", "ms": 0} in b.sent

    def test_hex_is_accepted_like_the_other_diagnostics(self):
        u, b = self._u()
        afcBambuAMS.cmd_BAMBU_ARMMS(u, self._G(MS="0x64"))
        assert {"cmd": "armms", "ms": 100} in b.sent

    def test_the_range_reaches_ten_minutes(self):
        # The point is to take it far out of the way, not to nudge it.
        u, b = self._u()
        afcBambuAMS.cmd_BAMBU_ARMMS(u, self._G(MS="600000"))
        assert {"cmd": "armms", "ms": 600000} in b.sent


class TestUnloadStopsAsking:
    """After the reel finishes, the bridge must be told to STOP. Without it it
    stays in retract motion and keeps polling the target tray, which the unit
    answers ~2 Hz with "there is no tray" until an internal deadline expires --
    12 exchanges over 34 s after every unload, measured, audible at the unit."""

    def _rig(self, still_loaded=False):
        calls = []
        lane = types.SimpleNamespace(
            name="lane15", status=None, loaded_to_hub=False, _load_state=True,
            disable_buffer=lambda: None, sync_to_extruder=lambda: None,
            unsync_to_extruder=lambda: None, select_lane=lambda: None,
            set_tool_unloaded=lambda normal_toolchange=True: None,
            extruder_obj=None)
        extruder = types.SimpleNamespace(tool_unload_speed=10.0,
                                         tool_stn_unload=50.0)
        u = _unit(_Bridge())
        u.afc_unload_bowden_length = 3000.0
        u._unload_in_progress = False
        u.gcode = types.SimpleNamespace(run_script_from_command=lambda s: None)
        u.afc = types.SimpleNamespace(
            reactor=u.afc.reactor,
            move_e_pos=lambda *a, **k: None,
            do_tool_cut_tip_form=lambda l, e: None,
            post_unload_macro=None,
            save_vars=lambda: None,
            function=types.SimpleNamespace(in_print=lambda: False),
            error=types.SimpleNamespace(
                handle_lane_failure=lambda l, m, pause=False:
                    calls.append("fail")))
        u.set_feed_assist = lambda l, on: calls.append(f"assist:{on}")
        u.stop = lambda: calls.append("stop")
        u.retract = lambda l, d: calls.append("retract")
        u.select_lane = lambda l: (True, 0)
        u._wait_move = lambda *a, **k: True
        u._toolhead_sensor_triggered = lambda l: still_loaded
        u._is_virtual_hub = lambda l: False
        return u, lane, extruder, calls

    def test_a_stop_follows_the_retract(self):
        u, lane, ext, calls = self._rig()
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is True
        assert "retract" in calls
        assert calls.index("stop", calls.index("retract")) > \
            calls.index("retract")

    def test_the_stop_is_the_last_thing_said_to_the_unit(self):
        # Anything after it would restart the polling it exists to end.
        u, lane, ext, calls = self._rig()
        afcBambuAMS.unit_unload_lane(u, lane, ext)
        assert calls[-1] == "stop"

    def test_the_pre_retract_stop_is_still_there(self):
        # A failed load can leave the AMS mid feed/retry, which swallows the
        # retract -- that stop is a different one and must not be lost.
        u, lane, ext, calls = self._rig()
        afcBambuAMS.unit_unload_lane(u, lane, ext)
        assert calls.index("stop") < calls.index("retract")

    def test_a_failed_unload_does_not_reach_the_stop(self):
        # Filament still at the sensor is a lane failure; the AMS is left
        # alone rather than being told to stand down mid-recovery.
        u, lane, ext, calls = self._rig(still_loaded=True)
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is False
        assert "fail" in calls
