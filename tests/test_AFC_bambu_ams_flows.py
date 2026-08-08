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

    def __init__(self, uids=(), finish=(0, False, ""), fault=None):
        self.sent = []
        self._uids = list(uids)
        self._finish = finish
        self._status = None
        # (seq, text, amps), the bridge's latched fault. Default None means the
        # accessor is absent entirely -- a quiet unit, which is what nearly
        # every flow wants.
        self._fault = fault
        if fault is not None:
            self.last_fault = lambda: self._fault

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
        u._send_mc_addr = lambda br: sent.append("mcaddr")
        afcBambuAMS._announce_unit(u)
        assert {"cmd": "units", "n": 3} in b.sent      # index 2 -> 3 units
        assert sent == ["ht", "mcaddr"]

    def test_a_throwing_send_is_swallowed(self):
        b = _Bridge()
        b.send = lambda o: (_ for _ in ()).throw(RuntimeError("closed"))
        u = _unit(b)
        u._send_ht_flag = lambda br: None
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

    def test_a_reconnect_announces_without_rebooting_the_pico(self):
        u = self._u()
        afcBambuAMS._on_bridge_reconnect(u)
        assert {"cmd": "reset"} not in u._bridge.sent

    def test_the_announce_is_deferred_not_sent_inline(self):
        u = self._u()
        afcBambuAMS._on_bridge_reconnect(u)
        # Deferred: the link being back is not the firmware being ready.
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


class TestNoAutomaticPicoReset:
    """The reconnect handler used to reboot the Pico before announcing.

    A AFC_BAMBU_RESTART clears a log drain wedged into its 0x0700 fallback, but
    that wedge is the drain PAYLOAD -- an HT answers 0x00, never 0x80 -- so
    with the payload right there is nothing for the reset to solve, and it
    hard-re-enumerates USB on every reconnect (and therefore every deploy).
    """

    def test_the_helper_is_gone(self):
        assert not hasattr(afcBambuAMS, "_reset_bridge_once")

    def test_the_cooldown_constant_is_gone(self):
        assert not hasattr(bambu_mod, "BRIDGE_RESET_COOLDOWN_S")

    def test_a_reconnect_sends_no_reset(self):
        u = _unit(_Bridge())
        u._announce_unit = lambda: None
        u._resolve_uid_index = lambda n: None
        afcBambuAMS._on_bridge_reconnect(u)
        for cb, _when in u.afc.reactor.callbacks:
            cb(0)
        assert {"cmd": "reset"} not in u._bridge.sent

    def test_there_is_no_manual_command_either(self):
        # Rebooting the Pico is a watchdog_reboot with no graceful USB detach,
        # and on hardware it took the USB-CAN adapter down with it -- twice in
        # one evening. Nothing needs it: narration survives Klipper and
        # firmware restarts on its own now that the drain payload is right,
        # which is what the reset was really compensating for.
        assert not hasattr(afcBambuAMS, "cmd_AFC_BAMBU_RESTART")

    def test_nothing_sends_a_reset(self):
        u = _unit(_Bridge())
        u._announce_unit = lambda: None
        u._resolve_uid_index = lambda n: None
        afcBambuAMS._on_bridge_reconnect(u)
        for cb, _when in u.afc.reactor.callbacks:
            cb(0)
        assert not any(o.get("cmd") == "reset" for o in u._bridge.sent)

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


class TestFinalizeScanIsTheNoTagOutcome:
    """_finalize_scan is one half of "two outcomes, never three": the unit
    finished its scan and read nothing, so the lane gets its AFC defaults.

    It does not decide WHETHER a tag read -- that is the unit's to say, and
    _scan_verdict is the only thing that asks. It carries no deferral loop and
    no per-model window: a window announces "no readable tag" for any real read
    that lands after it expires."""

    def _u(self, *, present=True, material="", in_flight=False, cap=None,
           spool_id=None, lane_material=None, cycle_ended=True):
        b = _Bridge()
        b.rfid_read_in_flight = lambda now, quiet=3.0, addr=None: in_flight
        b.rfid_cycle_ended_since = lambda since, addr=None: cycle_ended
        u = _unit(b)
        u.afc.save_vars = lambda: None
        u.afc.default_material_type = "Unknown"
        u.afc.default_color = "#FFFFFF"
        u._slots = [{"present": present, "material": material}]
        u._scan_t0 = [0.0]
        u._slot_map = {"lane1": 0}
        lane = types.SimpleNamespace(spool_id=spool_id, material=lane_material,
                                     name="lane1", weight=0)
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

    def test_it_never_reschedules_itself(self):
        # The deferral loop is gone. _sync_lanes runs on every status frame and
        # asks the unit; there is nothing here to wait for.
        u, lane = self._u(in_flight=True)
        u.afc.reactor._now = 100.0
        afcBambuAMS._finalize_scan(u, 0)
        assert u.afc.reactor.callbacks == []

    def test_a_leftover_profile_is_not_handed_back_as_a_default(self):
        # The unit read no tag, so any profile still in this bay's record is
        # the PREVIOUS spool's -- and apply_filament_defaults prefers
        # slot_info's own material over the AFC default, which would put it
        # straight back on the lane by a different route.
        u, lane = self._u(material="PLA Matte")
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.material == "Unknown"            # the AFC default, not PLA

    # TWO OUTCOMES, NEVER THREE. A finalised scan either read a tag or applies
    # defaults. Both of the cases below used to take a third path -- return
    # early and leave the lane exactly as it was -- and in both of them "as it
    # was" means the PREVIOUS spool's data, because the scan being finalised
    # read nothing that could have set it.

    def test_a_failed_scan_clears_a_spoolman_link_and_defaults(self):
        # A binding that outlives the spool it named is the last spool's claim
        # on this bay -- and keeping it is not the safe option: a stale link is
        # treated as authoritative elsewhere and BLOCKS the next real tag from
        # applying at all (observed live on lane18/spool 130, lane23/spool 124).
        u, lane = self._u(spool_id=42)
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.spool_id in (None, "", 0)
        assert lane.material == "Unknown"

    def test_a_failed_scan_replaces_a_leftover_material_with_defaults(self):
        u, lane = self._u(lane_material="PETG")
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.material == "Unknown"

    def test_boot_restore_still_keeps_what_the_user_already_had(self):
        # The same "apply defaults" ending, the opposite question: at boot a
        # profile restored from saved vars is the USER'S, not a leftover, so
        # _restore_untagged_defaults must not reach _finalize_scan for it.
        u, lane = self._u(lane_material="PETG")
        u.unit_slots = 1
        afcBambuAMS._restore_untagged_defaults(u)
        assert lane.material == "PETG"

    def test_boot_restore_still_keeps_a_spoolman_link(self):
        u, lane = self._u(spool_id=42)
        u.unit_slots = 1
        afcBambuAMS._restore_untagged_defaults(u)
        assert lane.spool_id == 42

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
    get_int is decimal-only, so AFC_BAMBU_DRAIN ADDR=0x0700 -- copied straight out
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
            return "AFC_BAMBU_DRAIN ADDR=?"

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
    MUTE_* bit, so when a unit ticks at idle it is the one suspect AFC_BAMBU_MUTE
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
            return "AFC_BAMBU_ARMMS"

        def respond_info(self, m):
            self.said.append(m)

    def _u(self):
        b = _Bridge()
        return _unit(b), b

    def test_no_bridge_is_an_error_not_a_silent_noop(self):
        u = _unit(None)
        with pytest.raises(RuntimeError):
            afcBambuAMS.cmd_AFC_BAMBU_ARMMS(u, self._G(MS="1000"))

    def test_it_sends_the_cadence(self):
        u, b = self._u()
        afcBambuAMS.cmd_AFC_BAMBU_ARMMS(u, self._G(MS="30000"))
        assert {"cmd": "armms", "ms": 30000} in b.sent

    def test_zero_restores_the_default_and_says_so(self):
        u, b = self._u()
        g = self._G(MS="0")
        afcBambuAMS.cmd_AFC_BAMBU_ARMMS(u, g)
        assert {"cmd": "armms", "ms": 0} in b.sent
        assert any("default" in m for m in g.said)

    def test_it_defaults_to_restoring(self):
        # No MS at all must not leave a bisect value stuck on the unit.
        u, b = self._u()
        afcBambuAMS.cmd_AFC_BAMBU_ARMMS(u, self._G())
        assert {"cmd": "armms", "ms": 0} in b.sent

    def test_hex_is_accepted_like_the_other_diagnostics(self):
        u, b = self._u()
        afcBambuAMS.cmd_AFC_BAMBU_ARMMS(u, self._G(MS="0x64"))
        assert {"cmd": "armms", "ms": 100} in b.sent

    def test_the_range_reaches_ten_minutes(self):
        # The point is to take it far out of the way, not to nudge it.
        u, b = self._u()
        afcBambuAMS.cmd_AFC_BAMBU_ARMMS(u, self._G(MS="600000"))
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


class TestTheUnloadStopsAskingALatchedUnit:
    """The load stops kicking a unit that has given up. The unload did not.

    Measured on an AMS 2 with the filament left in the extruder gears
    (AFC_BAMBU_TEST_STUCK_UNLOAD, lane20): the unit retried three times on its own
    -- "pull err,bdc stall" at 0.042m, then 0.001m, then 0.000m -- escalated to
    "TIMEOUT error 2/3" and latched. Everything after that went to a unit that
    had stopped listening, and the fault only surfaced 22s later, when the
    unload's own mute lifted and a follower tick happened to read it. Both
    detectors sit behind `_unload_in_progress` by design, so nothing inside the
    unload was reading the verdict at all.
    """

    FAULT = (7, "[AMS_COMMON]en:1,mode:3 [AMS_LED]TIMEOUT error 2", 0.0)

    def _rig(self, fault=None, preexisting=None):
        """A unit that declares `fault` in ANSWER to the first retract.

        The timing is the point. The unload marks the bridge's sequence just
        before it asks the unit to reel, so only what the unit says AFTER being
        asked counts -- `preexisting` sets a fault already latched at that
        moment, which is the jam from the load that preceded this unload and
        must not be re-read as the unload's own.
        """
        base = TestUnloadStopsAsking()
        u, lane, ext, calls = base._rig(still_loaded=True)
        bridge = _Bridge(fault=preexisting or (0, "", 0.0))
        u._bridge = bridge

        def retract(lane_, dist):
            calls.append("retract")
            if fault is not None:
                bridge._fault = fault
        u.retract = retract
        u.stop = lambda: calls.append("stop")
        u.fault_detect = True
        u._fault_seen = 0
        return u, lane, ext, calls

    def test_a_latched_unit_is_not_kicked_again(self):
        u, lane, ext, calls = self._rig(fault=self.FAULT)
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is False
        # One retract -- the unload's own. The unit declared before the first
        # re-kick, so there is no second.
        assert calls.count("retract") == 1

    def test_a_quiet_unit_still_gets_both_re_kicks(self):
        # No verdict means no reason to stop trying: the retries are what
        # recover a unit that merely swallowed the command.
        u, lane, ext, calls = self._rig(fault=None)
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is False
        assert calls.count("retract") == 3      # the unload's + two re-kicks

    def test_the_units_own_words_reach_the_operator(self):
        said = []
        u, lane, ext, calls = self._rig(fault=self.FAULT)
        u.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        afcBambuAMS.unit_unload_lane(u, lane, ext)
        assert said and "TIMEOUT error 2" in said[0]
        assert "latched" in said[0]
        # ...and the keep-alive beside it does not.
        assert "en:1,mode:3" not in said[0]

    def test_the_ams1_state_only_verdict_lands_the_same_way(self):
        # AMS 1 narrates plenty; it just never says "stall" or "timeout". Its
        # give-up is a STATE, and it arrives on the keep-alive tags, so it has
        # to survive the chatter filter to reach the operator.
        said = []
        u, lane, ext, calls = self._rig(
            fault=(3, "[AMS_COMMON]state:6,tray_now:255,tray_exit:6 "
                      "[AMS_LINK]en:0,mode:7,idx:255,ref:0", 0.0))
        u.afc.error.handle_lane_failure = \
            lambda l, m, pause=False: said.append(m)
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is False
        assert calls.count("retract") == 1
        assert said and "en:0,mode:7,idx:255" in said[0]

    def test_a_latched_unit_is_told_to_stand_down(self):
        # A latched unit is streaming a retract it will never finish, which is
        # the same ~2 Hz "there is no tray" noise the success path's stop ends.
        u, lane, ext, calls = self._rig(fault=self.FAULT)
        afcBambuAMS.unit_unload_lane(u, lane, ext)
        assert calls[-1] == "fail"                    # the report is last
        assert calls[-2] == "stop"                    # halted before reporting
        assert calls.index("stop", calls.index("retract")) > \
            calls.index("retract")

    def test_a_failure_with_no_verdict_still_leaves_the_unit_alone(self):
        # The existing rule, unchanged: with nothing said the AMS may still be
        # reeling, and stopping it aborts a retract that could yet succeed.
        u, lane, ext, calls = self._rig(fault=None)
        afcBambuAMS.unit_unload_lane(u, lane, ext)
        # Only the pre-retract stops and the re-kicks' own -- none after the
        # last retract.
        assert calls[-1] == "fail"
        assert calls[-2] == "retract"

    def test_the_verdict_is_consumed_so_it_is_not_raised_twice(self):
        # The follower tick's _check_ams_fault shares `_fault_seen`. Reporting
        # here and leaving the sequence unconsumed would pause the printer a
        # second time for the same event once the unload's mute lifts.
        u, lane, ext, calls = self._rig(fault=self.FAULT)
        afcBambuAMS.unit_unload_lane(u, lane, ext)
        assert u._fault_seen == self.FAULT[0]

    def test_a_benign_stall_exit_is_not_a_latch(self):
        # A scan ending its pull-in bumps the bridge's sequence and means
        # nothing. Treating it as a give-up would abandon a healthy unload
        # after a single retract.
        u, lane, ext, calls = self._rig(
            fault=(2, "[AMS_SWITCH]bldc stall exit", 0.0))
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is False
        assert calls.count("retract") == 3      # rode out both re-kicks

    def test_a_fault_left_over_from_before_the_unload_is_ignored(self):
        # The mark is taken at the first retract precisely so a jam from the
        # load that preceded this unload is not re-read as the unload's own.
        u, lane, ext, calls = self._rig(preexisting=self.FAULT)
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is False
        assert calls.count("retract") == 3


class TestTheLastReKickIsActuallyChecked:
    """The loop ran range(2): sensor at the top, re-kick at the bottom -- so
    the SECOND re-kick's result was never read. A lane the second attempt
    genuinely freed still fell out of the loop into handle_lane_failure, and
    the operator got a lane failure for an unload that had worked."""

    def test_the_second_re_kick_can_still_succeed(self):
        base = TestUnloadStopsAsking()
        u, lane, ext, calls = base._rig(still_loaded=True)
        u._bridge = _Bridge()
        u.retract = lambda l, d: calls.append("retract")
        # Three sensor reads, two re-kicks: loaded, loaded, then clear. The
        # third read is the one the old range(2) loop never took.
        reads = iter([True, True, False])
        u._toolhead_sensor_triggered = lambda l: next(reads, False)
        assert afcBambuAMS.unit_unload_lane(u, lane, ext) is True
        assert "fail" not in calls


class TestScanHoldOnStaleRecord:
    """
    A bay the AMS already holds a record for reports that record from the
    instant a spool goes in -- it belongs to the PREVIOUS spool, and it is live
    in the status frame long before the reader has seen the new tag.

    Measured on hardware (AMS 1, bay 3): the insert edge and "applied tag to
    lane17: Bambu PLA" landed in the same second, 16s before the unit narrated
    "card auth success / read success,valid". The applied value could not have
    come from the new tag. Worse, that material then satisfied the fallback's
    "a tag read in time" test, so the lane defaults never ran either and the
    lane kept the previous spool's profile.

    The hold is one timestamp: while a scan is open nothing from the bay
    reaches the lane, and only the UNIT ends it -- by narrating a read, or by
    narrating the end of its cycle. Comparing the record's CONTENT against a
    pre-scan snapshot cannot do it: a re-scan of the same spool returns
    identical bytes, and a scan-cleared profile reads as "the unit re-read".
    """

    STALE = {"present": True, "material": "PLA", "sku": "GFA10",
             "color": None, "temp_min": None, "temp_max": None, "weight": None}

    def _u(self, *, read_ok=False, ended=True, slot_info=None):
        b = _Bridge()
        b.rfid_read_succeeded_since = lambda since, addr=None: read_ok
        b.rfid_cycle_ended_since = lambda since, addr=None: ended
        u = _unit(b)
        u.afc.default_material_type = "Unknown"
        u.afc.default_color = "#FFFFFF"
        u._slots = [dict(slot_info if slot_info is not None else self.STALE)]
        u._slot_map = {"lane1": 0}
        u._scan_t0 = [None]
        u._scan_notag = [False]
        u.SCAN_FALLBACK_CAP = 45.0
        lane = types.SimpleNamespace(spool_id=None, material=None,
                                     name="lane1", weight=0, bambu_sku=None)
        u.lanes = {"lane1": lane}
        return u, lane

    def _arm(self, u):
        """Open a scan, as the insert edge does."""
        u._scan_t0[0] = u.afc.reactor.monotonic()

    def test_an_open_scan_with_no_word_yet_is_waiting(self):
        u, _ = self._u(ended=False)
        self._arm(u)
        assert afcBambuAMS._scan_verdict(u, 0) == "waiting"

    def test_a_finished_cycle_with_no_read_is_no_tag(self):
        u, _ = self._u()
        self._arm(u)
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"

    def test_no_scan_open_is_no_verdict(self):
        u, _ = self._u()
        assert afcBambuAMS._scan_verdict(u, 0) == "none"

    def test_a_leftover_material_no_longer_counts_as_a_tag_that_read(self):
        # The regression: material was present, so the fallback returned early
        # and the lane was left holding the previous spool's profile. Nothing
        # reads the record's content to decide this now.
        u, lane = self._u()
        self._arm(u)
        afcBambuAMS._finalize_scan(u, 0)
        assert lane.material == "Unknown"     # defaults applied, not "PLA"

    def test_the_same_spool_put_back_is_still_a_read(self):
        # The record never changes, but the unit SAYS it read it. A snapshot
        # comparison could not tell this from "nothing has happened yet" and
        # held the lane stale for the whole fallback window -- the reported
        # "it takes forever to update".
        u, _ = self._u(read_ok=True)
        self._arm(u)
        assert afcBambuAMS._scan_verdict(u, 0) == "read"

    def test_the_hold_survives_a_failed_read(self):
        # Releasing here would let the next status pass surface the leftover
        # record straight over the defaults just applied.
        u, _ = self._u()
        self._arm(u)
        afcBambuAMS._finalize_scan(u, 0)
        assert u._scan_t0[0] is not None

    def test_removing_the_spool_releases_the_hold(self):
        u, _ = self._u()
        self._arm(u)
        u._slots[0]["present"] = False
        afcBambuAMS._finalize_scan(u, 0)
        assert u._scan_t0[0] is None


class TestUntaggedDefaultsSurviveARestart:
    """
    A bay whose tag does not read must show defaults, and keep them across a
    Klipper restart while the spool stays put.

    Measured on hardware: AMS 1 bay 2 (tag genuinely unreadable -- the unit
    narrated "tray pull over 790 mm, but no card detected") showed PLA/1000g
    right up to a restart and came back blank. Two separate gaps. The defaults
    were applied to the lane object but never persisted, so AFC restored the
    pre-default record; and nothing re-applies them at boot, because a spool
    present at startup is deliberately never re-scanned. A TAGGED bay hid both,
    since its profile is re-derived from the AMS record every boot -- only the
    untagged bay, which has no record to re-derive from, lost it.
    """

    def _u(self, slots):
        b = _Bridge()
        b.rfid_read_in_flight = lambda now, quiet=3.0, addr=None: False
        b.rfid_read_succeeded_since = lambda since, addr=None: False
        b.rfid_cycle_ended_since = lambda since, addr=None: True
        u = _unit(b)
        u.afc.default_material_type = "Unknown"
        u.afc.default_color = "#FFFFFF"
        u.afc.saves = 0

        def _save():
            u.afc.saves += 1
        u.afc.save_vars = _save
        u._slots = [dict(s) for s in slots]
        u.unit_slots = len(slots)
        u._scan_stale = [None] * len(slots)
        u._scan_t0 = [None] * len(slots)
        u._slot_map = {"lane%d" % i: i for i in range(len(slots))}
        u.lanes = {n: types.SimpleNamespace(spool_id=None, material=None,
                                            name=n, weight=0, bambu_sku=None)
                   for n in u._slot_map}
        return u

    def test_applying_defaults_persists_them(self):
        u = self._u([{"present": True, "material": None}])
        afcBambuAMS._finalize_scan(u, 0)
        assert u.lanes["lane0"].material == "Unknown"
        assert u.afc.saves == 1, "defaults were never written to vars"

    def test_boot_fills_in_a_present_untagged_bay(self):
        u = self._u([{"present": True, "material": None}])
        afcBambuAMS._restore_untagged_defaults(u)
        assert u.lanes["lane0"].material == "Unknown"

    def test_boot_leaves_an_empty_bay_alone(self):
        u = self._u([{"present": False, "material": None}])
        afcBambuAMS._restore_untagged_defaults(u)
        assert u.lanes["lane0"].material is None

    def test_boot_leaves_a_tagged_bay_to_its_record(self):
        # It re-derives from the AMS record; defaults here would race it.
        u = self._u([{"present": True, "material": "PLA Matte"}])
        afcBambuAMS._restore_untagged_defaults(u)
        assert u.lanes["lane0"].material is None

    def test_boot_keeps_a_profile_restored_from_vars(self):
        u = self._u([{"present": True, "material": None}])
        u.lanes["lane0"].material = "PETG"           # AFC restored it
        afcBambuAMS._restore_untagged_defaults(u)
        assert u.lanes["lane0"].material == "PETG"

    def test_boot_keeps_a_spoolman_linked_lane(self):
        u = self._u([{"present": True, "material": None}])
        u.lanes["lane0"].spool_id = 42
        afcBambuAMS._restore_untagged_defaults(u)
        assert u.lanes["lane0"].material is None

    def test_boot_handles_every_bay_independently(self):
        u = self._u([{"present": True, "material": None},
                     {"present": True, "material": "PLA Basic"},
                     {"present": False, "material": None}])
        afcBambuAMS._restore_untagged_defaults(u)
        assert u.lanes["lane0"].material == "Unknown"
        assert u.lanes["lane1"].material is None
        assert u.lanes["lane2"].material is None

    def test_phantom_bays_beyond_unit_slots_are_skipped(self):
        # A 1-slot HT can report garbage bits for bays it does not have.
        u = self._u([{"present": True, "material": None},
                     {"present": True, "material": None}])
        u.unit_slots = 1
        afcBambuAMS._restore_untagged_defaults(u)
        assert u.lanes["lane1"].material is None


class TestFallbackWaitsForTheScanCycleToEnd:
    """
    "No readable tag" must be a fact, not a guess against a clock.

    Measured on hardware, AMS 1 bay 3 re-insert: the unit went quiet for 11s
    between its tray_readid chatter and the auth, so the 14s fallback logged
    "no readable tag in slot 2; applied lane defaults to lane17" at 18:05:02 --
    and the real tag landed at 18:05:04, overwriting it two seconds later. The
    outcome was right and the message was false.

    The unit narrates STEP7:finish,cali tray at the end of a scan whichever way
    it goes -- after "feed with rfid success" (18:05:10) and after "tray pull
    over 790 mm, but no card detected" (17:46:29) -- so waiting for that is
    exact. tray_readid cannot serve: a unit can sit in it forever and never pull
    the tray, which is why it is excluded from the in-flight matcher. That case
    never reaches STEP7 either, so the hard cap is what ends it.
    """

    def _u(self, *, read_ok=False, cycle_ended=False, material="", now=101.0):
        b = _Bridge()
        b.rfid_cycle_ended_since = lambda since, addr=None: cycle_ended
        b.rfid_read_succeeded_since = lambda since, addr=None: read_ok
        u = _unit(b)
        u.afc.default_material_type = "Unknown"
        u.afc.default_color = "#FFFFFF"
        u.afc.save_vars = lambda: None
        u.afc.reactor._now = now
        u._slots = [{"present": True, "material": material}]
        u._scan_t0 = [100.0]
        u._scan_notag = [False]
        u.SCAN_FALLBACK_CAP = 45.0
        u._slot_map = {"lane1": 0}
        u.lanes = {"lane1": types.SimpleNamespace(
            spool_id=None, material=None, name="lane1", weight=0)}
        return u

    def test_a_mid_cycle_silence_is_not_an_answer(self):
        # The exact 18:05:02 moment: the unit has said neither thing yet.
        u = self._u(cycle_ended=False)
        assert afcBambuAMS._scan_verdict(u, 0) == "waiting"

    def test_a_finished_cycle_with_no_tag_is_the_no_tag_answer(self):
        u = self._u(cycle_ended=True)
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"
        afcBambuAMS._finalize_scan(u, 0)
        assert u.lanes["lane1"].material == "Unknown"

    def test_the_backstop_still_ends_a_unit_that_never_finishes(self):
        # tray_preload -> tray_readid xN -> silence, never pulls the tray: no
        # STEP7 ever comes, so only the backstop can end this one.
        u = self._u(cycle_ended=False, now=100.0 + 46.0)
        assert afcBambuAMS._scan_verdict(u, 0) == "notag"

    def test_a_narrated_read_outranks_everything(self):
        # It read the tag. Nothing about the cycle's end changes that, and no
        # inspection of the record is involved in knowing it.
        u = self._u(read_ok=True, cycle_ended=False)
        assert afcBambuAMS._scan_verdict(u, 0) == "read"
