"""BoxTurtle RFID module: lane->reader mapping, shared-antenna exclusion, and
the staged-read experiment feed.

Hardware (two RC522-family readers on a Pico running the rfid_bridge
firmware) is driven through the shared read_tag stack over a fail-soft serial
link -- NOT a Klipper [mcu], so an absent reader Pico can never fail the
printer. These tests pin the serial protocol, the fail-soft contract, and the
orchestration -- who gets read, what gets excluded, and exactly how much
filament the stage feed moves and restores."""

from __future__ import annotations

import threading
import time
import types

import pytest

import extras.AFC_BoxTurtle_rfid as mod
from extras.AFC_BoxTurtle_rfid import (AFC_BoxTurtle_rfid,
                                       AFC_BoxTurtle_rfid_reader,
                                       _BridgeSerial,
                                       _SerialRegLink)


class _Gcmd:
    def __init__(self, **params):
        self.params = params
        self.info = []

    def get(self, key, default=None):
        return self.params.get(key, default)

    def get_float(self, key, default=None, minval=None, maxval=None,
                  above=None):
        return float(self.params.get(key, default))

    def get_int(self, key, default=None, minval=None, maxval=None):
        return int(self.params.get(key, default))

    def respond_info(self, msg):
        self.info.append(msg)

    def error(self, msg):
        return RuntimeError(msg)


class _Completion:
    """Klipper's reactor completion, near enough for a test: one waiter, one
    completer, and wait() returns what complete() was given."""

    def __init__(self):
        self._ev = threading.Event()
        self._value = None

    def complete(self, value):
        self._value = value
        self._ev.set()

    def wait(self, waketime=None, waketime_result=None):
        # The real one takes an absolute reactor time; the fake just needs a
        # bound so a wedged worker fails the test instead of hanging it.
        return self._value if self._ev.wait(10.0) else waketime_result


class _Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = max(self.now, waketime)

    def completion(self):
        return _Completion()

    def register_async_callback(self, callback, waketime=None):
        # Klipper runs these on the reactor; running it inline is enough to
        # let a worker thread hand its result back.
        callback(self.now)


def _reader(name, lanes, bus=0):
    r = AFC_BoxTurtle_rfid_reader.__new__(AFC_BoxTurtle_rfid_reader)
    r.name = name
    r.lanes = list(lanes)
    r.bus = bus
    r.link = None
    r.version = 0x92
    return r


def _shim(readers, lanes=None, now=0.0):
    u = AFC_BoxTurtle_rfid.__new__(AFC_BoxTurtle_rfid)
    u.logger = types.SimpleNamespace(
        info=lambda *a: None, warning=lambda *a: None)
    u.reactor = _Clock()
    u.reactor.now = now
    u._readers = readers
    u._reader_by_lane = {ln: r for r in readers for ln in r.lanes}
    u._last_uid_by_lane = {}
    u._baseline_uid = None
    u.afc = types.SimpleNamespace(
        lanes=lanes or {},
        homing_enabled=True,
        function=types.SimpleNamespace(is_printing=lambda: False))
    u.spool_diameter_mm = 200.0
    u.tag_sweep_revs = 2.0
    u.tag_sweep_speed = 20.0
    u.tag_sweep_step_mm = 20.0
    u.tag_retract_speed = 0.0      # 0 = use the lane's own speed
    u._sweeping = False
    u.scan_on_insert = True
    u.sibling_tag_adjust = True
    u.sibling_tag_adjust_dist = 75.0
    u.read_timeout_s = 6.0
    u.retract_after_read = True
    u.bambu_master_key = None
    u.creality_key = None
    u.creality_encryption_key = None
    u.serial_port = "/dev/serial/by-id/test-rfid-pico"
    u.printer = types.SimpleNamespace(
        lookup_object=lambda name, default=None: default)
    u.bridge = types.SimpleNamespace(connected=lambda: True)
    return u


class _FakeLane:
    """A lane whose LOAD switch tracks the filament, so a restore that homes
    on it can be tested rather than only its arithmetic.

    The load sensor, not prep: prep trips when the operator's filament touches
    it, before the gears have seated anything, so it is not a position. The
    load sensor is where prep_load homes the filament, which is why the scan
    hooks after prep_load and why the restore homes back onto it.

    ``slip`` is the fraction of each FEED the filament does not actually
    travel -- gears skidding on a spool that has not started turning. It is
    what makes a blind -fed retract over-shoot, so it is the whole reason
    the restore homes instead of counting.
    """

    def __init__(self, name, long_moves_speed=100.0, load_at=0.0, slip=0.0):
        self.name = name
        self.long_moves_accel = 400
        self.long_moves_speed = long_moves_speed
        self.short_move_dis = 10.0
        self.short_moves_speed = 25.0
        self.moves = []
        self.assists = []
        self.pos = 5.0            # prep_load homed the tip onto the sensor
        self.load_at = load_at    # load reads true at or above this
        self.slip = slip
        self.load_es = "load"     # the endstop AFC homes this lane to
        self.unit_obj = None      # set where the test exercises unit helpers
        self.homing_moves = []

    @property
    def raw_load_state(self):
        return self.pos > self.load_at

    @property
    def prep_state(self):
        return True               # filament is in the lane throughout

    def move(self, distance, speed, accel, assist_active=False):
        self.moves.append((distance, speed, accel))
        self.assists.append(assist_active)
        self.pos += (distance * (1.0 - self.slip) if distance > 0
                     else distance)

    def move_to(self, distance, speed_mode, endstop=None,
                assist_active=None, use_homing=True):
        """AFC's homing move, and its DIRECTION decides what it means.
        move_to passes `triggered = distance > 0` into home_to, so a forward
        home stops when the switch is MADE (the load) and a reverse home
        stops when it RELEASES (the unload). Getting this backwards is what
        left lane8 reading "LOCKED NOT LOADED" after a scan, so the fake
        models both sides."""
        self.homing_moves.append(
            (distance, speed_mode, endstop, use_homing, assist_active))
        if use_homing and endstop is not None:
            if distance > 0:                       # stops the moment it is made
                if self.raw_load_state:
                    return True, 0.0, None         # already made: no travel
                self.pos = min(self.load_at + 0.5, self.pos + distance)
            else:                                  # stops when it releases
                self.pos = max(self.load_at - 0.5, self.pos + distance)
            return True, abs(distance), None
        self.pos += (distance * (1.0 - self.slip) if distance > 0
                     else distance)
        return True, abs(distance), None


class TestSerialRegLink:
    """The link speaks one register op per line, hex, and treats every kind
    of silence the same way: the reader is not answering."""

    def _link(self, replies):
        sent = []
        bridge = types.SimpleNamespace(
            request=lambda line: (sent.append(line) or replies.pop(0)))
        return _SerialRegLink(bridge, 0), sent

    def test_reg_read_sends_r_line_and_parses_hex(self):
        link, sent = self._link(["92"])
        assert link.reg_read(0x37) == 0x92
        assert sent == ["r0 37"]

    def test_reg_write_sends_w_line(self):
        link, sent = self._link([""])
        link.reg_write(0x2A, 0x8D)
        assert sent == ["w0 2A 8D"]

    def test_the_bus_number_rides_in_the_line(self):
        bridge = types.SimpleNamespace(request=lambda line: "92")
        assert _SerialRegLink(bridge, 1).reg_read(0x37) == 0x92

    def test_no_answer_raises_for_the_read_stack_to_catch(self):
        link, _ = self._link([None])
        with pytest.raises(OSError):
            link.reg_read(0x37)

    def test_a_failed_write_raises_too(self):
        link, _ = self._link([None])
        with pytest.raises(OSError):
            link.reg_write(0x2A, 0x8D)


class _FakePort:
    def __init__(self, lines):
        self.lines = list(lines)
        self.written = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(data)

    def readline(self):
        return self.lines.pop(0) if self.lines else b""

    def close(self):
        pass


class TestBridgeSerialProtocol:
    """request() must survive everything the stream can carry: op replies,
    interleaved JSON events, NAKs, timeouts, and a port that dies mid-op --
    always returning a value, never raising into Klipper."""

    def _bridge(self, lines):
        b = _BridgeSerial.__new__(_BridgeSerial)
        b.port = "/dev/test"
        b.logger = types.SimpleNamespace(
            info=lambda *a: None, warning=lambda *a: None)
        import threading
        b._lock = threading.Lock()
        b._ser = _FakePort(lines)
        return b

    def test_a_read_reply_returns_its_payload(self):
        b = self._bridge([b"=92\n"])
        assert b.request("r0 37") == "92"

    def test_a_write_ack_returns_empty_string(self):
        b = self._bridge([b"=\n"])
        assert b.request("w0 2A 8D") == ""

    def test_json_events_on_the_stream_are_skipped(self):
        b = self._bridge([b'{"evt":"hello","fw":"RFID-0.2"}\n', b"=92\n"])
        assert b.request("r0 37") == "92"

    def test_a_nak_is_none(self):
        b = self._bridge([b"!\n"])
        assert b.request("r0 37") is None

    def test_a_timeout_is_none(self):
        b = self._bridge([])
        assert b.request("r0 37") is None

    def test_no_port_is_none_not_an_exception(self):
        b = self._bridge([])
        b._ser = None
        assert b.request("r0 37") is None

    def test_a_dying_port_drops_the_connection(self):
        b = self._bridge([])
        b._ser.write = lambda d: (_ for _ in ()).throw(OSError("gone"))
        assert b.request("r0 37") is None
        assert b._ser is None            # closed; the timer will retry


class TestLaneReaderMap:
    def test_each_lane_resolves_to_its_reader(self):
        r0 = _reader("reader0", ["lane8", "lane9"])
        r1 = _reader("reader1", ["lane10", "lane11"])
        u = _shim([r0, r1])
        assert u._reader_by_lane["lane8"] is r0
        assert u._reader_by_lane["lane9"] is r0
        assert u._reader_by_lane["lane10"] is r1
        assert u._reader_by_lane["lane11"] is r1

    def test_an_unmapped_lane_is_a_gcode_error(self):
        u = _shim([_reader("reader0", ["lane8", "lane9"])])
        with pytest.raises(RuntimeError) as e:
            u._require_lane_reader(_Gcmd(LANE="lane12"))
        assert "no reader serves lane lane12" in str(e.value)


class TestSharedAntennaExclusion:
    """Each antenna sees BOTH of its lanes' spools. The partner lane's
    last-known UID is excluded so the read stack halts on the sibling and
    keeps hunting for the lane's own tag."""

    def _u(self):
        return _shim([_reader("reader0", ["lane8", "lane9"]),
                      _reader("reader1", ["lane10", "lane11"])])

    def test_no_known_siblings_means_no_excluder(self):
        u = self._u()
        assert u._excluder_for("lane8", u._reader_by_lane["lane8"]) is None

    def test_the_partners_uid_is_excluded(self):
        u = self._u()
        u._last_uid_by_lane["lane9"] = "aabbccdd"
        ex = u._excluder_for("lane8", u._reader_by_lane["lane8"])
        assert ex("AABBCCDD") is True

    def test_the_lanes_own_uid_is_not_excluded(self):
        # Re-reading the same lane must see its own tag again.
        u = self._u()
        u._last_uid_by_lane["lane8"] = "aabbccdd"
        assert u._excluder_for("lane8", u._reader_by_lane["lane8"]) is None

    def test_the_other_readers_lanes_do_not_leak_in(self):
        u = self._u()
        u._last_uid_by_lane["lane10"] = "11223344"
        assert u._excluder_for("lane8", u._reader_by_lane["lane8"]) is None


class TestReadCommand:
    def _u_with_tag(self, tag):
        lanes = {"lane8": _FakeLane("lane8")}
        u = _shim([_reader("reader0", ["lane8", "lane9"])], lanes=lanes)
        u._read_once = lambda ln, r: tag
        u.applied = []
        u.apply_to_lane = lambda lane, t: u.applied.append((lane.name, t))
        return u

    def test_no_tag_says_so_and_points_at_stage(self):
        u = self._u_with_tag(None)
        g = _Gcmd(LANE="lane8")
        u.cmd_AFC_BT_RFID_READ(g)
        assert g.info == [
            "AFC_BT_RFID_READ: nothing readable in reader0's field for lane8. "
            "If the tag rides the spool, use AFC_BT_RFID_STAGE to spin it past "
            "the antenna."]
        assert u.applied == []

    def test_a_tag_is_applied_and_its_uid_remembered(self):
        u = self._u_with_tag({"uid": "04A1B2C3", "sak": 0x00})
        u.cmd_AFC_BT_RFID_READ(_Gcmd(LANE="lane8"))
        assert u.applied == [("lane8", {"uid": "04A1B2C3", "sak": 0x00})]
        assert u._last_uid_by_lane["lane8"] == "04a1b2c3"


class TestTagHomingSweep:
    """The sweep is a homing move whose endstop is the RFID read: feed while
    polling CONTINUOUSLY, stop the instant a tag answers, and give up after a
    bounded number of spool revolutions. The bound is in revolutions because a
    tag sits at ONE angular position -- the coil only gets a look once per
    turn, and a turn of a full 200mm spool costs ~630mm of filament."""

    def _u(self, tag_after_mm=None, homing=False):
        lane = _FakeLane("lane8")
        u = _shim([_reader("reader0", ["lane8", "lane9"])],
                  lanes={"lane8": lane})
        # Default to the NO-HOMING path so the retract shows up as ordinary
        # moves and its speed can be asserted. With homing the retract is one
        # endstop move at AFC's own SpeedMode, which is covered separately.
        u.afc.homing_enabled = homing
        fed = {"mm": 0.0}
        # The poller runs on its own thread. On hardware each move takes
        # seconds, so it always gets a look; in a test the move loop would
        # otherwise finish before the thread is ever scheduled. Synchronise
        # explicitly rather than leaning on timing: once the sweep has fed far
        # enough, the move blocks until the poller has actually observed it.
        seen = threading.Event()
        real_move = lane.move

        def move(distance, speed, accel, assist_active=False):
            real_move(distance, speed, accel, assist_active)
            if distance > 0:
                fed["mm"] += distance
                if tag_after_mm is not None and fed["mm"] >= tag_after_mm:
                    seen.wait(5.0)

        lane.move = move

        def read_blocking(ln, r):
            if tag_after_mm is None:
                return None
            if fed["mm"] >= tag_after_mm:
                seen.set()
                return {"uid": "deadbeef", "sak": 0x08}
            return None

        u._read_blocking = read_blocking
        u.applied = []
        u.apply_to_lane = lambda l, t: u.applied.append(l.name)
        return u, lane, fed

    def test_revs_convert_to_a_circumference_bounded_sweep(self):
        # 2 turns of a 200mm spool = 2 * pi * 200 = 1257mm, not an arbitrary
        # millimetre default.
        u, lane, fed = self._u(tag_after_mm=None)
        g = _Gcmd(LANE="lane8")
        u.cmd_AFC_BT_RFID_STAGE(g)
        assert round(sum(self._sweep_moves(lane))) == 1257

    def test_it_stops_as_soon_as_the_tag_answers(self):
        u, lane, fed = self._u(tag_after_mm=60.0)
        g = _Gcmd(LANE="lane8")
        u.cmd_AFC_BT_RFID_STAGE(g)
        assert sum(self._sweep_moves(lane)) == 60.0   # not the 1257 bound
        assert u.applied == ["lane8"]

    def test_the_filament_ends_up_where_it_started(self):
        # The point of the restore, stated as the thing that actually
        # matters: the tip is back at the prep sensor it came from. Asserting
        # a final move of -fed would only re-state the old arithmetic, which
        # is exactly what was wrong with it.
        u, lane, fed = self._u(tag_after_mm=60.0)
        start = lane.pos
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert lane.pos == pytest.approx(start)
        assert lane.raw_load_state

    def test_retract_zero_leaves_the_filament_where_it_stopped(self):
        u, lane, fed = self._u(tag_after_mm=60.0)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", RETRACT=0))
        assert all(m[0] > 0 for m in lane.moves)

    def test_an_explicit_advance_overrides_revs(self):
        u, lane, fed = self._u(tag_after_mm=None)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", ADVANCE=100.0))
        assert sum(self._sweep_moves(lane)) == 100.0

    def test_a_full_pass_with_no_answer_blames_the_mounting(self):
        # A whole revolution with nothing found is geometry, not distance --
        # the message must not send the operator back for another lap.
        u, lane, fed = self._u(tag_after_mm=None)
        g = _Gcmd(LANE="lane8", ADVANCE=700.0)     # > one 628mm turn
        u.cmd_AFC_BT_RFID_STAGE(g)
        assert u.applied == []
        assert any("move the reader" in m for m in g.info)

    def test_a_short_sweep_blames_the_distance_not_the_mounting(self):
        # Less than a turn proves nothing about where the reader is aimed.
        u, lane, fed = self._u(tag_after_mm=None)
        g = _Gcmd(LANE="lane8", ADVANCE=100.0)
        u.cmd_AFC_BT_RFID_STAGE(g)
        assert any("raise REVS" in m for m in g.info)
        assert not any("move the reader" in m for m in g.info)

    def test_a_hit_reports_the_distance_and_nothing_more(self):
        # Just the fact and the number -- the distance is a one-off, but that
        # does not need re-explaining on every successful read.
        u, lane, fed = self._u(tag_after_mm=60.0)
        g = _Gcmd(LANE="lane8")
        u.cmd_AFC_BT_RFID_STAGE(g)
        assert any("tag answered after 60mm of feed (filament restored)." in m
                   for m in g.info)

    def test_an_offline_reader_moves_no_filament(self):
        # Never spin a spool at a reader that cannot answer.
        u, lane, fed = self._u(tag_after_mm=60.0)
        u._readers[0].version = None
        with pytest.raises(RuntimeError):
            u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert lane.moves == []

    def test_the_feed_leaves_the_espooler_alone(self):
        # Feeding pulls filament OFF the spool, which free-wheels -- an assist
        # there is just the espooler fighting the pull.
        u, lane, fed = self._u(tag_after_mm=60.0)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        feeds = [a for (d, _s, _ac), a in zip(lane.moves, lane.assists)
                 if d > 0]
        assert feeds and not any(feeds)

    def test_the_restoring_retract_runs_the_espooler(self):
        u, lane, fed = self._u(tag_after_mm=60.0)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        retracts = [(d, a) for (d, _s, _a), a in zip(lane.moves, lane.assists)
                    if d < 0]
        assert retracts and all(a for _d, a in retracts)

    def _sweep_moves(self, lane):
        """The feed phase only -- everything up to the first retract."""
        out = []
        for d, _s, _a in lane.moves:
            if d < 0:
                break
            out.append(d)
        return out

    def test_the_bulk_of_the_return_is_one_fast_move(self):
        # The sweep feeds in chunks because a queued Klipper move cannot be
        # interrupted. Most of the way back has nothing to watch for, so it
        # is a single move; only the tail is stepped, to home on prep.
        u, lane, fed = self._u(tag_after_mm=60.0)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", STEP=20.0))
        back = [m[0] for m in lane.moves if m[0] < 0]
        assert back[0] == -30.0                  # 60 fed, 30 tail held back
        assert all(m == -10.0 for m in back[1:])  # ... tail, one short_move

    def test_the_return_uses_the_lane_speed_not_the_sweep_speed(self):
        # The search is slow because a tag needs time to answer inside the
        # coil. The way back has nothing to read, so making it inherit that
        # speed just makes the operator wait.
        u, lane, fed = self._u(tag_after_mm=60.0)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", SPEED=20.0))
        assert all(v == 20.0 for v in
                   [m[1] for m in lane.moves[:len(self._sweep_moves(lane))]])
        back = [m for m in lane.moves if m[0] < 0]
        assert back[0][1] == 100.0               # _FakeLane.long_moves_speed
        # The sensor-homed tail runs at the lane's SHORT move speed, because
        # its step size is the overshoot bound.
        assert all(m[1] == 25.0 for m in back[1:])

    def test_retract_speed_can_be_given_explicitly(self):
        u, lane, fed = self._u(tag_after_mm=60.0)
        u.cmd_AFC_BT_RFID_STAGE(
            _Gcmd(LANE="lane8", SPEED=20.0, RETRACT_SPEED=75.0))
        assert [m[1] for m in lane.moves if m[0] < 0][0] == 75.0

    def test_a_lane_with_no_long_move_speed_falls_back_to_the_sweep(self):
        # Proves the fallback chain end to end rather than only its first
        # link: no configured retract speed AND no lane speed means the sweep
        # speed, not a crash or a zero-speed move.
        lane = _FakeLane("lane8")
        del lane.long_moves_speed
        u = _shim([_reader("reader0", ["lane8"])], lanes={"lane8": lane})
        u.afc.homing_enabled = False        # the speed only applies here
        u._readers[0].read = lambda ln: {"uid": "AA"}
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", SPEED=17.0))
        assert [m[1] for m in lane.moves if m[0] < 0][0] == 17.0

    def test_the_announcement_names_the_return_speed(self):
        u, lane, fed = self._u(tag_after_mm=60.0)
        g = _Gcmd(LANE="lane8", SPEED=20.0)
        u.cmd_AFC_BT_RFID_STAGE(g)
        assert any("Coming back at 100mm/s." in m for m in g.info)

    def test_a_partial_last_chunk_does_not_overshoot_the_bound(self):
        u, lane, fed = self._u(tag_after_mm=None)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", ADVANCE=50.0, STEP=20.0))
        assert self._sweep_moves(lane) == [20.0, 20.0, 10.0]


class TestStatusCommand:
    def test_reports_readers_and_last_reads(self):
        u = _shim([_reader("reader0", ["lane8", "lane9"])])
        u._last_uid_by_lane["lane8"] = "04a1b2c3"
        g = _Gcmd()
        u.cmd_AFC_BT_RFID_STATUS(g)
        assert g.info == [
            "AFC_BT_RFID status\n"
            "bridge: connected (/dev/serial/by-id/test-rfid-pico)\n"
            "reader0: version reg 0x92  lanes: lane8, lane9\n"
            "lane8: last uid 04a1b2c3"]

    def test_offline_reader_says_offline(self):
        r = _reader("reader0", ["lane8", "lane9"])
        r.version = None
        u = _shim([r])
        g = _Gcmd()
        u.cmd_AFC_BT_RFID_STATUS(g)
        assert "reader0: offline" in g.info[0]

    def test_no_readers_configured_says_how_to(self):
        u = _shim([])
        g = _Gcmd()
        u.cmd_AFC_BT_RFID_STATUS(g)
        assert g.info == [
            "AFC_BT_RFID: no reader sections configured "
            "([AFC_BoxTurtle_rfid <name>] with bus + lanes)."]


class TestConnectTimerOwnsThePort:
    """The reader Pico is NOT a Klipper [mcu]: config and ready touch no
    hardware, and a background timer owns connecting -- so an absent bridge
    costs a quiet retry, never the printer. (Its ancestor bug: an i2c_read
    inside klippy:ready took the whole reactor down in a shutdown loop.)"""

    class _Reactor:
        NEVER = 9e99

        def __init__(self):
            self.timers = []
            self.async_cbs = []

        def monotonic(self):
            return 100.0

        def register_timer(self, cb, waketime):
            self.timers.append((cb, waketime))

        def register_async_callback(self, cb):
            self.async_cbs.append(cb)

        def drain(self):
            cbs, self.async_cbs = self.async_cbs, []
            for cb in cbs:
                cb(0.0)
            return len(cbs)

    class _Bridge:
        def __init__(self, ok=True):
            self.ok = ok
            self.connect_calls = 0
            self._connected = False

        def connected(self):
            return self._connected

        def connect(self):
            self.connect_calls += 1
            self._connected = self.ok
            return self.ok

    class _Link:
        def __init__(self):
            self.reads = []

        def reg_read(self, reg):
            self.reads.append(reg)
            return 0x92

    def _ready_shim(self, bridge, readers=None):
        r = _reader("reader0", ["lane8", "lane9"])
        r.version = None
        rs = readers if readers is not None else [r]
        u = AFC_BoxTurtle_rfid.__new__(AFC_BoxTurtle_rfid)
        u._sweeping = False
        u._probing = False
        u.said = {"info": [], "warning": []}
        u.logger = types.SimpleNamespace(
            info=lambda m, *a: u.said["info"].append(str(m)),
            warning=lambda m, *a: u.said["warning"].append(str(m)))
        u.reactor = self._Reactor()
        u.bridge = bridge
        u.printer = types.SimpleNamespace(
            lookup_object=lambda name, default=None: default,
            lookup_objects=lambda module: [
                (f"AFC_BoxTurtle_rfid {x.name}", x) for x in rs])
        u.bambu_master_key = b"\x00" * 16
        u.creality_key = b"\x00" * 16
        u.creality_encryption_key = b"\x00" * 16
        return u, r

    @staticmethod
    def _tick(u, t):
        """One connect-timer tick, end to end.

        The timer no longer touches the port -- it starts a thread and returns
        -- so a test has to wait for that thread and then run what it handed
        back to the reactor. Waiting on _probing is safe because the worker
        registers its callback BEFORE clearing the flag.
        """
        nxt = u.reactor.timers[0][0](t)
        deadline = time.time() + 5.0
        while u._probing and time.time() < deadline:
            time.sleep(0.002)
        assert not u._probing, "the probe thread never finished"
        u.reactor.drain()
        return nxt

    def test_ready_touches_no_hardware_and_starts_the_timer(self):
        bridge = self._Bridge()
        u, r = self._ready_shim(bridge)
        u._handle_ready()
        assert bridge.connect_calls == 0
        assert len(u.reactor.timers) == 1
        assert u.reactor.timers[0][1] == 101.0
        assert isinstance(r.link, _SerialRegLink)

    def test_a_fresh_connect_probes_the_version(self):
        bridge = self._Bridge()
        u, r = self._ready_shim(bridge)
        u._handle_ready()
        r.link = self._Link()
        nxt = self._tick(u, 101.0)
        assert r.link.reads == [0x37]
        assert r.version == 0x92
        assert nxt == 101.0 + 5.0

    def test_an_absent_bridge_is_a_quiet_retry(self):
        bridge = self._Bridge(ok=False)
        u, r = self._ready_shim(bridge)
        u._handle_ready()
        r.link = self._Link()
        nxt = self._tick(u, 101.0)
        assert r.link.reads == []           # nothing probed
        assert r.version is None            # reported offline
        assert nxt == 101.0 + 5.0           # and it keeps trying

    def test_a_probe_failure_marks_the_reader_offline(self):
        bridge = self._Bridge()
        u, r = self._ready_shim(bridge)
        u._handle_ready()
        r.link = types.SimpleNamespace(
            reg_read=lambda reg: (_ for _ in ()).throw(OSError("x")))
        self._tick(u, 101.0)
        assert r.version is None


class TestAnOfflineReaderIsRetried:
    """The probe used to run ONCE, on a fresh connect.

    A reader that did not answer that single moment kept version = None for
    the life of the session, and None is not cosmetic -- it skips the tag scan
    on insert and makes AFC_BT_RFID_STAGE refuse. So a reader wedged at boot,
    plugged in afterwards, or briefly unhappy on a shared bus was dead until
    Klipper restarted or the bridge link happened to drop.
    """

    _Reactor = TestConnectTimerOwnsThePort._Reactor
    _Bridge = TestConnectTimerOwnsThePort._Bridge
    _Link = TestConnectTimerOwnsThePort._Link

    def _shim(self, bridge, readers=None):
        return TestConnectTimerOwnsThePort._ready_shim(self, bridge, readers)

    class _FlakyLink:
        """Fails the first N reads, then answers."""

        def __init__(self, fail=1):
            self.fail = fail
            self.reads = 0

        def reg_read(self, reg):
            self.reads += 1
            if self.reads <= self.fail:
                raise OSError("bus quiet")
            return 0x92

    def test_a_reader_that_missed_the_first_probe_comes_back(self):
        bridge = self._Bridge()
        u, r = self._shim(bridge)
        u._handle_ready()
        r.link = self._FlakyLink(fail=1)
        tick = lambda t: TestConnectTimerOwnsThePort._tick(u, t)
        tick(101.0)                              # fresh connect: probe fails
        assert r.version is None
        tick(106.0)                              # retry: it answers
        assert r.version == 0x92

    def test_a_healthy_reader_is_not_re_read(self):
        """Re-reading a chip that already answered buys nothing and puts
        avoidable traffic on the link the sweeps use."""
        bridge = self._Bridge()
        u, r = self._shim(bridge)
        u._handle_ready()
        r.link = self._Link()
        tick = lambda t: TestConnectTimerOwnsThePort._tick(u, t)
        tick(101.0)
        assert r.link.reads == [0x37]
        tick(106.0)
        tick(111.0)
        assert r.link.reads == [0x37]            # still just the one

    def test_nothing_is_probed_during_a_sweep(self):
        """A sweep is a timed read against a moving spool on the same serial
        bridge; a probe landing mid-sweep is the HT-polling mistake again."""
        bridge = self._Bridge()
        u, r = self._shim(bridge)
        u._handle_ready()
        r.link = self._Link()
        tick = lambda t: TestConnectTimerOwnsThePort._tick(u, t)
        tick(101.0)                              # fresh connect probes
        r.version = None                         # now offline
        u._sweeping = True
        tick(106.0)
        assert r.link.reads == [0x37]            # the retry held off
        u._sweeping = False
        tick(111.0)
        assert r.link.reads == [0x37, 0x37]      # and resumed after

    def test_an_absent_reader_does_not_warn_every_tick(self):
        """At _RETRY_S this would be 720 identical lines an hour, in the log
        someone reads to find out what ELSE went wrong."""
        bridge = self._Bridge()
        u, r = self._shim(bridge)
        u._handle_ready()
        r.link = self._FlakyLink(fail=99)
        tick = lambda t: TestConnectTimerOwnsThePort._tick(u, t)
        for i in range(6):
            tick(101.0 + 5.0 * i)
        assert len(u.said["warning"]) == 1, u.said["warning"]

    def test_recovery_is_announced_once(self):
        bridge = self._Bridge()
        u, r = self._shim(bridge)
        u._handle_ready()
        r.link = self._FlakyLink(fail=1)
        tick = lambda t: TestConnectTimerOwnsThePort._tick(u, t)
        tick(101.0)
        tick(106.0)                              # recovers -> one info line
        tick(111.0)                              # healthy -> silent
        tick(116.0)
        assert len(u.said["info"]) == 1, u.said["info"]

    def test_one_dead_reader_does_not_stop_the_other_being_retried(self):
        bridge = self._Bridge()
        good = _reader("reader0", ["lane8", "lane9"], bus=0)
        bad = _reader("reader1", ["lane10", "lane11"], bus=1)
        good.version = bad.version = None
        u, _r = self._shim(bridge, readers=[good, bad])
        u._handle_ready()
        good.link = self._Link()
        bad.link = self._FlakyLink(fail=99)
        tick = lambda t: TestConnectTimerOwnsThePort._tick(u, t)
        tick(101.0)
        tick(106.0)
        assert good.version == 0x92              # answered, then left alone
        assert good.link.reads == [0x37]
        assert bad.version is None
        assert bad.link.reads == 2               # still being asked


class TestSweepRefusesWhileSomethingElseMoves:
    """Two AFC lane moves at once interleave the trapq swap in
    AFC_stepper._move, step generation fails with "Internal error in
    stepcompress", and every MCU on the printer shuts down. It happened on
    hardware: a sweep issued while a TD-1 capture was retracting."""

    def _u(self, state=None, sweeping=False):
        lane = _FakeLane("lane8")
        u = _shim([_reader("reader0", ["lane8"])], lanes={"lane8": lane})
        u._read_blocking = lambda ln, r: {"uid": "AA"}
        u.applied = []
        u.apply_to_lane = lambda l, t: u.applied.append(l.name)
        u._sweeping = sweeping
        if state is not None:
            idle = types.SimpleNamespace(
                get_status=lambda now: {"state": state})
            u.printer = types.SimpleNamespace(
                lookup_object=lambda name, default=None:
                    idle if name == "idle_timeout" else default)
        return u, lane

    def test_it_refuses_while_the_toolhead_is_moving(self):
        u, lane = self._u(state="Printing")
        with pytest.raises(RuntimeError):
            u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert lane.moves == []          # nothing was moved at all

    def test_it_runs_when_the_printer_is_idle(self):
        # The tag answers on the first poll here, so a correct sweep feeds
        # nothing at all -- what proves it RAN is that it got as far as
        # applying the read rather than refusing.
        u, lane = self._u(state="Ready")
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert u.applied == ["lane8"]

    def test_a_second_sweep_cannot_start_inside_the_first(self):
        # The re-entrancy half: a lane move ends in toolhead.wait_moves(),
        # which pauses the reactor, so a command arriving over the API can run
        # INSIDE one that is already sweeping.
        u, lane = self._u(state="Ready", sweeping=True)
        with pytest.raises(RuntimeError):
            u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert lane.moves == []

    def test_the_flag_is_cleared_after_a_normal_sweep(self):
        u, lane = self._u(state="Ready")
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert u._sweeping is False

    def test_the_flag_is_cleared_when_the_sweep_raises(self):
        # Otherwise one failure locks the command out until a restart.
        u, lane = self._u(state="Ready")
        def boom(*a, **k):
            raise RuntimeError("reader exploded")
        u._sweep_for_tag = boom
        with pytest.raises(RuntimeError):
            u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert u._sweeping is False

    def test_no_idle_timeout_object_does_not_block_the_command(self):
        # A printer without idle_timeout should still be able to sweep.
        u, lane = self._u(state=None)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert u.applied == ["lane8"]

    def test_an_idle_timeout_that_raises_does_not_block_the_command(self):
        u, lane = self._u(state="Ready")
        idle = types.SimpleNamespace(
            get_status=lambda now: (_ for _ in ()).throw(KeyError("nope")))
        u.printer = types.SimpleNamespace(
            lookup_object=lambda name, default=None:
                idle if name == "idle_timeout" else default)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8"))
        assert u.applied == ["lane8"]


class TestScanOnInsert:
    """The tag scan runs from AFC's insert cycle, before the lane is staged.

    It has to be there rather than after the TD-1 capture for a physical
    reason: the sweep rotates the spool by FEEDING filament, up to a metre of
    it on a full reel, and only at this point in the cycle is the lane's own
    bowden empty ahead of the filament. It is also the right data order --
    the tag says what the spool is, the TD-1 measures what it looks like."""

    def _u(self, tag_after_mm=40.0, lanes=("lane8",), homing=False, **kw):
        lane = _FakeLane("lane8")
        u = _shim([_reader("reader0", list(lanes))], lanes={"lane8": lane})
        u.afc.homing_enabled = homing
        # Same thread handshake the manual-sweep tests use: the poller runs on
        # its own thread, so without this the move loop finishes before it is
        # ever scheduled and the sweep looks like it never moved.
        fed = {"mm": 0.0}
        seen = threading.Event()
        real_move = lane.move

        def move(distance, speed, accel, assist_active=False):
            real_move(distance, speed, accel, assist_active)
            if distance > 0:
                fed["mm"] += distance
                if tag_after_mm is not None and fed["mm"] >= tag_after_mm:
                    seen.wait(5.0)

        lane.move = move

        def read_blocking(ln, r):
            if tag_after_mm is None:
                return None
            if fed["mm"] >= tag_after_mm:
                seen.set()
                return {"uid": "AA"}
            return None

        # _read_blocking is the poller's one call; the reader fake has no link.
        u._read_blocking = read_blocking
        u.applied = []
        u.apply_to_lane = lambda ln, t: u.applied.append((ln.name, t["uid"]))
        for k, v in kw.items():
            setattr(u, k, v)
        return u, lane

    def test_an_insert_scans_and_applies_the_tag(self):
        u, lane = self._u()
        u._on_lane_prep_loaded(lane)
        assert u.applied == [("lane8", "AA")]

    def test_the_filament_is_put_back_where_it_started(self):
        # The insert cycle stages the lane straight after this, so a sweep
        # that left filament fed would stage from the wrong place.
        u, lane = self._u()
        u._on_lane_prep_loaded(lane)
        assert sum(m[0] for m in lane.moves) == pytest.approx(0.0)

    def test_the_return_runs_the_espooler(self):
        u, lane = self._u()
        u._on_lane_prep_loaded(lane)
        back = [(d, a) for (d, _s, _a), a in zip(lane.moves, lane.assists)
                if d < 0]
        assert back and all(a for _d, a in back)

    def test_a_lane_no_reader_claims_is_left_alone(self):
        # Half a BoxTurtle can have readers; the other half must not move.
        u, lane = self._u(lanes=("lane9",))
        u._on_lane_prep_loaded(lane)
        assert lane.moves == []
        assert u.applied == []

    def test_scan_on_insert_false_disables_it(self):
        u, lane = self._u(scan_on_insert=False)
        u._on_lane_prep_loaded(lane)
        assert lane.moves == []

    def test_an_offline_reader_moves_no_filament(self):
        u, lane = self._u()
        u._readers[0].version = None
        u._on_lane_prep_loaded(lane)
        assert lane.moves == []

    def test_it_does_not_join_a_sweep_already_running(self):
        u, lane = self._u(_sweeping=True)
        u._on_lane_prep_loaded(lane)
        assert lane.moves == []

    def test_no_tag_still_restores_the_filament(self):
        u, lane = self._u(tag_after_mm=None)
        u._on_lane_prep_loaded(lane)
        assert u.applied == []
        assert sum(m[0] for m in lane.moves) == pytest.approx(0.0)

    def test_a_failure_never_costs_the_insert(self):
        # AFC calls this from inside prep_callback. Raising here would break
        # the insert over a tag that could not be read.
        u, lane = self._u()
        def boom(*a, **k):
            raise RuntimeError("reader exploded")
        u._sweep_for_tag = boom
        u._on_lane_prep_loaded(lane)          # must not raise
        assert u._sweeping is False

    def test_a_lane_with_no_name_is_ignored(self):
        u, lane = self._u()
        u._on_lane_prep_loaded(types.SimpleNamespace())
        assert lane.moves == []


class TestRestoreHomesOnPrep:
    """Why the restore is not just -fed.

    The feed does not always advance what it is told: the gears skid against
    a spool that has not started turning. The error is one-directional -- the
    tip is SHORT of the commanded distance -- so giving back the full amount
    walks it out through the prep sensor and the spool reads as removed. That
    is what happened on lane9. Homing onto prep lands on a repeatable edge
    instead of an accumulated estimate."""

    def _u(self, **lane_kw):
        lane = _FakeLane("lane8", **lane_kw)
        u = _shim([_reader("reader0", ["lane8"])], lanes={"lane8": lane})
        # These pin the NO-HOMING fallback: the stepped give-back that runs
        # when there is no endstop to stop the retract. The homing path is
        # covered by TestRestoreUsesAfcHoming below.
        u.afc.homing_enabled = False
        return u, lane

    def test_a_slipping_feed_still_ends_at_the_sensor(self):
        # THE BUG. 20% of the feed never happened, so a blind -fed retract
        # leaves the tip 80mm the wrong side of prep and the spool reads as
        # pulled out. Homing cannot land on the exact start -- the creep
        # slips too -- but it lands ON the sensor, which is the position that
        # actually means "where the operator left it".
        u, lane = self._u(slip=0.2)
        u._lane_move(lane, 400.0, 100.0, assist=False)      # feed, slipping
        assert lane.pos < 405.0                             # it really slipped
        u._restore(lane, 400.0, 100.0)
        assert lane.raw_load_state                              # still inserted
        assert lane.pos - lane.load_at <= lane.short_move_dis

    def test_a_blind_retract_would_have_pulled_it_out(self):
        # The counter-example, so the test above is not just asserting that
        # something happened: the arithmetic this replaced really does fail.
        u, lane = self._u(slip=0.2)
        u._lane_move(lane, 400.0, 100.0, assist=False)
        u._lane_move(lane, -400.0, 100.0, assist=True)      # the old way
        assert not lane.raw_load_state

    def test_a_clean_feed_also_ends_at_the_sensor(self):
        u, lane = self._u()
        start = lane.pos
        u._lane_move(lane, 400.0, 100.0, assist=False)
        u._restore(lane, 400.0, 100.0)
        assert lane.pos == pytest.approx(start)

    def test_the_bulk_is_one_move_and_only_the_tail_is_stepped(self):
        u, lane = self._u()
        lane.pos = 405.0
        lane.moves.clear()
        u._restore(lane, 400.0, 100.0)
        back = [m[0] for m in lane.moves if m[0] < 0]
        assert back[0] == -370.0                 # 400 less the 30mm tail
        assert all(m == -10.0 for m in back[1:])  # short_move_dis steps

    def test_the_tail_steps_bound_how_far_it_can_overshoot(self):
        # A queued Klipper move cannot be interrupted, so the step size IS
        # the overshoot bound. Anything larger risks pulling the spool out.
        u, lane = self._u()
        lane.pos = 405.0
        lane.moves.clear()
        u._restore(lane, 400.0, 100.0)
        assert lane.pos >= lane.load_at
        assert lane.pos - lane.load_at <= lane.short_move_dis

    def test_nothing_on_the_sensor_falls_back_to_the_measured_distance(self):
        # The manual command on an empty lane has nothing to home against.
        u, lane = self._u()
        lane.pos = -50.0                          # load is false
        lane.moves.clear()
        back = u._restore(lane, 120.0, 100.0)
        assert [m[0] for m in lane.moves] == [-120.0]
        assert back == 120.0

    def test_it_homes_on_load_not_prep(self):
        # THE DISTINCTION THIS CHANGE IS ABOUT. Prep trips as soon as the
        # operator's filament touches it, before the gears seat anything, so
        # a restore homing on prep has no honest reference. Here prep is stuck
        # true and only load moves; the restore must terminate on the load
        # edge rather than waiting for prep to say something.
        u, lane = self._u()
        lane.pos = 405.0
        lane.moves.clear()
        u._restore(lane, 400.0, 100.0)
        assert lane.prep_state                    # prep never went false
        assert lane.raw_load_state                # ... load is what stopped it
        assert lane.pos - lane.load_at <= lane.short_move_dis

    def test_a_lane_with_only_load_state_still_homes(self):
        # Not every unit exposes raw_load_state; fall back to load_state
        # rather than treating the lane as empty and blind-retracting.
        u, _lane = self._u()
        plain = types.SimpleNamespace(
            name="lane8", long_moves_accel=400, long_moves_speed=100.0,
            short_move_dis=10.0, short_moves_speed=25.0,
            moves=[], assists=[], pos=405.0, load_state=True)

        def move(distance, speed, accel, assist_active=False):
            plain.moves.append((distance, speed, accel))
            plain.pos += distance
            plain.load_state = plain.pos > 0.0

        plain.move = move
        u._restore(plain, 400.0, 100.0)
        assert plain.load_state
        assert plain.pos <= plain.short_move_dis

    def test_nothing_fed_moves_nothing(self):
        u, lane = self._u()
        assert u._restore(lane, 0.0, 100.0) == 0.0
        assert lane.moves == []

    def test_the_return_runs_the_espooler_but_the_creep_does_not(self):
        # Retracting has to wind the slack back on or it piles up loose on
        # the reel; the forward creep is pulling off it again.
        u, lane = self._u()
        lane.pos = 405.0
        lane.moves.clear()
        u._restore(lane, 400.0, 100.0)
        for (d, _s, _a), assist in zip(lane.moves, lane.assists):
            assert assist is (d < 0)

    def test_a_prep_switch_that_never_releases_still_terminates(self):
        # A stuck switch must not spin the spool forever.
        u, lane = self._u()
        lane.load_at = -1e9                       # always true
        lane.pos = 405.0
        lane.moves.clear()
        u._restore(lane, 400.0, 100.0)
        assert len(lane.moves) < 100


class TestSweepStepSizing:
    """A chunk is a whole move: ramp up, cruise, ramp down. One shorter than
    the ramps never reaches the speed it was asked for -- it peaks partway and
    comes straight back down, which is what "chunky" is. It also silently caps
    the sweep: at 250mm/s^2 a 20mm chunk averaged 35mm/s whether the speed was
    set to 60 or 100, because the chunk was the limit, not the speed."""

    def _lane(self, accel=250.0):
        lane = _FakeLane("lane8")
        lane.long_moves_accel = accel
        return lane

    @staticmethod
    def _profile(step, speed, accel):
        """(peak velocity, cruise distance) for one chunk."""
        ramp = speed * speed / (2.0 * accel)
        if 2.0 * ramp <= step:
            return speed, step - 2.0 * ramp
        return (accel * step) ** 0.5, 0.0

    def test_the_old_fixed_chunk_never_reached_speed(self):
        # The bug, pinned: 20mm at 100mm/s and 250mm/s^2 is pure ramp.
        peak, cruise = self._profile(20.0, 100.0, 250.0)
        assert cruise == 0.0
        assert peak < 75.0

    @pytest.mark.parametrize("speed", [40.0, 60.0, 100.0, 150.0])
    def test_the_chosen_chunk_actually_cruises(self, speed):
        u = _shim([_reader("reader0", ["lane8"])])
        lane = self._lane()
        step = u._sweep_step_for(lane, speed)
        peak, cruise = self._profile(step, speed, lane.long_moves_accel)
        assert peak == pytest.approx(speed)          # reaches what was asked
        if step < u.SWEEP_STEP_MAX:
            assert cruise / step >= u.SWEEP_CRUISE_FRACTION - 0.01
        else:
            # The cap binds at high speed. It still cruises -- it just trades
            # some smoothness for a finer abort, which is the right way round.
            assert cruise > 0.0

    def test_a_slower_sweep_needs_a_shorter_chunk(self):
        u = _shim([_reader("reader0", ["lane8"])])
        lane = self._lane()
        assert u._sweep_step_for(lane, 60.0) < u._sweep_step_for(lane, 100.0)

    def test_higher_acceleration_needs_a_shorter_chunk(self):
        u = _shim([_reader("reader0", ["lane8"])])
        assert (u._sweep_step_for(self._lane(1000.0), 100.0)
                < u._sweep_step_for(self._lane(250.0), 100.0))

    def test_bounded_at_both_ends(self):
        # Never below the old default, and never so long that a hit costs a
        # lot of extra filament before the sweep notices.
        u = _shim([_reader("reader0", ["lane8"])])
        assert u._sweep_step_for(self._lane(1e6), 1.0) == u.SWEEP_STEP_MIN
        assert u._sweep_step_for(self._lane(10.0), 400.0) == u.SWEEP_STEP_MAX

    def test_a_lane_without_an_accel_still_gets_a_chunk(self):
        u = _shim([_reader("reader0", ["lane8"])])
        lane = _FakeLane("lane8")
        del lane.long_moves_accel
        step = u._sweep_step_for(lane, 100.0)
        assert u.SWEEP_STEP_MIN <= step <= u.SWEEP_STEP_MAX


class TestSiblingTagClearing:
    """Two lanes share one antenna, so the sister's PARKED tag can be read as
    this lane's -- and UID exclusion cannot catch the case that matters. The
    excluder needs the sibling's UID already on file; the first time a sister
    spool's tag sits in the field there is nothing to compare it against, so
    it is read as this lane's and two lanes end up pointing at one spool,
    silently. Rolling the sister back removes the ambiguity instead of trying
    to resolve it."""

    class _FakeUnit:
        """The unit's own staging, which is all prep_post_load does: feed
        dist_hub from the load switch and record it. No hub involved -- that
        is the point, dist_hub is calibrated to stop SHORT of it."""

        def move_to_load(self, lane, dist, dir, use_homing=True,
                         speed_mode=None):
            """AFC's helper: home to the load endstop. home_to runs with
            triggered=True and check_trigger=True, so it stops ON the switch
            -- eject_lane's `while lane.load_state:` next line depends on
            that."""
            return lane.move_to(dist * dir, speed_mode, endstop=lane.load_es,
                                use_homing=use_homing)

        def prep_load(self, lane):
            """The insert cycle's forward home onto the load switch, plus its
            short-move fallback."""
            lane.move_to(400.0, None, endstop=lane.load_es, use_homing=True)
            for _ in range(40):
                if lane.raw_load_state:
                    break
                lane.move(lane.short_move_dis, 500, 400)

        def prep_post_load(self, lane):
            if lane.loaded_to_hub or not lane.raw_load_state:
                return
            lane.move(lane.dist_hub, 100.0, 400, False)
            lane.loaded_to_hub = True

    class _FakeHub:
        """The hub switch, reading the sister's actual position.

        A hub-staged lane sits SHORT of it (hub clear); pushing past ``at``
        trips it, which is the state that blocks a TD-1 capture.
        """

        def __init__(self, lane, at, hub_clear_move_dis=35.0):
            self.lane = lane
            self.at = at
            # AFC's own "how far back so it does not block the hub exit".
            self.hub_clear_move_dis = hub_clear_move_dis

        @property
        def state(self):
            return self.lane.pos >= self.at

    def _u(self, hub_at=None, sib_pos=200.0, dist_hub=200.0,
           parked="CAFE1234", **sib_state):
        lane = _FakeLane("lane8")
        sib = _FakeLane("lane9")
        # A properly staged sister is metres past its own load switch, at the
        # hub. The default here says so; sib_pos is what the tests that model
        # a BADLY staged one turn down.
        sib.pos = sib_pos
        # dist_hub is calibrated to stop SHORT of the hub -- that is what
        # makes prep_post_load safe on a shared hub -- so a staged lane sits
        # at dist_hub from its load switch and the hub triggers some way
        # further on. Tests that want the tip ON the hub say so explicitly.
        if hub_at is None:
            hub_at = dist_hub + 40.0
        sib.tool_loaded = False
        sib.loaded_to_hub = True
        sib.load_to_hub = True          # the config decision re-staging obeys
        sib.dist_hub = dist_hub
        sib.load_es = "load"
        sib.is_direct_hub = lambda: False
        sib.unit_obj = self._FakeUnit()
        lane.unit_obj = self._FakeUnit()
        sib.hub_obj = self._FakeHub(sib, hub_at)
        for k, v in sib_state.items():
            setattr(sib, k, v)
        u = _shim([_reader("reader0", ["lane8", "lane9"])],
                  lanes={"lane8": lane, "lane9": sib})
        # A tag IS parked on the shared antenna unless a test says otherwise:
        # that is the collision these gates exist for, and without one the
        # clear correctly does nothing at all.
        u._parked_tag_once = lambda rdr: parked
        return u, lane, sib

    def test_a_clear_antenna_means_the_sister_is_not_touched(self):
        # THE CHANGE THIS PINS. This used to move the sister on EVERY insert,
        # tag on the antenna or not -- and that motion is what jammed the hub
        # twice and pulled a spool out of lane9 once. ACE2 and ViViD both
        # probe first; so does this now.
        u, lane, sib = self._u(parked=None)
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []
        assert sib.homing_moves == []

    def test_this_lanes_own_parked_tag_is_not_a_collision(self):
        # SEEN ON HARDWARE. A shared antenna is shared in both directions:
        # the lane just inserted is stationary too, so ITS tag can be the one
        # resting on the coil. lane8's own 7bf0afff read as parked and lane9
        # was rolled back 75mm and re-staged for nothing.
        u, lane, sib = self._u(parked="7BF0AFFF")
        u._last_uid_by_lane = {"lane8": "7bf0afff"}
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []
        assert u._baseline_uid is None      # nothing to exclude, it is ours

    def test_the_sisters_known_tag_still_is_one(self):
        u, lane, sib = self._u(parked="CAFE1234")
        u._last_uid_by_lane = {"lane8": "7bf0afff", "lane9": "cafe1234"}
        assert u._clear_sibling("lane8", u._readers[0]) is not None
        assert sib.moves != []

    def test_an_unknown_parked_tag_is_treated_as_a_collision(self):
        # The first-encounter case the clear exists for: no history to
        # attribute it by, and guessing "it is ours" there is the silent
        # two-lanes-one-spool failure.
        u, lane, sib = self._u(parked="DEADBEEF")
        u._last_uid_by_lane = {}
        assert u._clear_sibling("lane8", u._readers[0]) is not None
        assert sib.moves != []

    def test_attribution_is_case_insensitive(self):
        u, lane, sib = self._u(parked="7BF0AFFF")
        u._last_uid_by_lane = {"lane8": "7bf0afff"}
        assert u._clear_sibling("lane8", u._readers[0]) is None

    def test_a_parked_tag_is_what_triggers_the_clear(self):
        u, lane, sib = self._u(parked="CAFE1234")
        assert u._clear_sibling("lane8", u._readers[0]) is not None
        assert sib.moves != []

    def test_the_probe_runs_before_any_move_is_made(self):
        # If the probe ran after the roll-back it would be reading an antenna
        # this code had already cleared, and would always say "no collision".
        u, lane, sib = self._u()
        seen = []
        u._parked_tag_once = lambda rdr: seen.append(list(sib.moves)) or "AA"
        u._clear_sibling("lane8", u._readers[0])
        assert seen == [[]]

    def test_the_probe_does_not_run_on_the_reactor(self):
        # Same trap the closing read fell into: the probe is a second of
        # serial round-trips, so it belongs on a worker thread.
        import inspect
        src = inspect.getsource(mod.AFC_BoxTurtle_rfid._parked_tag_once)
        assert "threading.Thread" in src
        assert "completion.wait" in src
        assert "_parked_tag_once" not in inspect.getsource(
            mod.AFC_BoxTurtle_rfid._parked_tag)

    def test_the_probe_is_not_excluded(self):
        # The excluder skips PAST a known sibling UID during the sweep. Here
        # the sibling UID is exactly what is being looked for, so filtering it
        # would make the probe always answer "clear".
        import inspect
        src = inspect.getsource(mod.AFC_BoxTurtle_rfid._parked_tag)
        assert "is_excluded" not in src

    def test_an_unmovable_sisters_tag_is_excluded_instead(self):
        # ACE2's `baseline`, and the half of its double check that needs no
        # motion. Every safety gate below the probe returns WITHOUT moving the
        # sister, and in those cases its tag is still sitting on the coil --
        # so the read has to be told to skip it or it lands on this lane.
        u, lane, sib = self._u(parked="CAFE1234")
        sib.tool_loaded = True                     # a gate that refuses to move
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []
        ex = u._excluder_for("lane8", u._readers[0])
        assert ex is not None and ex("cafe1234") is True

    def test_an_empty_sister_never_gets_this_lanes_tag_excluded(self):
        # SEEN ON HARDWARE (lane11 read, lane10 empty). The tag on the shared
        # coil is THIS lane's own fresh sticker -- the sister is empty, so it
        # cannot be hers. The old order set the baseline before checking her,
        # so the sweep then skipped this lane's own tag as the neighbour's and
        # reported "no tag". An empty sister must leave no exclusion behind.
        u, lane, sib = self._u(parked="04782696DD2A81")
        type(sib).prep_state = property(lambda self: False)
        try:
            assert u._clear_sibling("lane8", u._readers[0]) is None
            assert sib.moves == []
            assert u._baseline_uid is None
            assert u._excluder_for("lane8", u._readers[0]) is None
        finally:
            type(sib).prep_state = property(lambda self: True)

    def test_a_sister_off_her_load_switch_excludes_nothing(self):
        # The other unseated case: filament in the lane but not on the load
        # switch, so it is not staged at the antenna. A parked tag there is the
        # read lane's, not hers -- no baseline, or the read skips its own tag.
        u, lane, sib = self._u(sib_pos=-50.0, parked="04782696DD2A81")
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []
        assert u._baseline_uid is None
        assert u._excluder_for("lane8", u._readers[0]) is None

    def test_a_cleared_sister_needs_no_baseline(self):
        # It is off the antenna now, so excluding it would only make the read
        # skip a tag that is legitimately this lane's on the next insert.
        u, lane, sib = self._u(parked="CAFE1234")
        assert u._clear_sibling("lane8", u._readers[0]) is not None
        assert u._baseline_uid is None

    def test_a_clear_antenna_leaves_no_baseline_behind(self):
        u, lane, sib = self._u(parked="CAFE1234")
        u._clear_sibling("lane8", u._readers[0])   # sets and clears it
        u2, _l, _s = self._u(parked=None)
        u2._clear_sibling("lane8", u2._readers[0])
        assert u2._baseline_uid is None

    def test_the_baseline_survives_a_lane_with_no_uid_history(self):
        # The first-encounter case the last-known-UID set cannot cover: no
        # history exists, so without the baseline there is no excluder at all.
        u, lane, sib = self._u(parked="CAFE1234")
        u._last_uid_by_lane = {}
        sib.tool_loaded = True
        u._clear_sibling("lane8", u._readers[0])
        ex = u._excluder_for("lane8", u._readers[0])
        assert ex is not None and ex("CAFE1234") is True

    def test_an_idle_hub_staged_sister_is_rolled_back(self):
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        assert token is not None
        assert sum(m[0] for m in sib.moves) == pytest.approx(-75.0)
        assert all(m[0] < 0 for m in sib.moves)

    def test_a_badly_staged_sister_stops_at_its_load_switch(self):
        # THE ONE THAT PUT A SPOOL ON THE FLOOR. loaded_to_hub is REMEMBERED,
        # not measured, and stale state from earlier testing left it true
        # while the filament sat barely inside the lane. A blind 75mm retract
        # then had nothing to give back. The switch gets the last word over
        # the flag, so the distance is a maximum rather than a commitment.
        u, lane, sib = self._u(sib_pos=5.0)       # barely past the switch
        token = u._clear_sibling("lane8", u._readers[0])
        assert sum(-m[0] for m in sib.moves) < 75.0
        assert sib.pos > -sib.short_move_dis      # nowhere near out of the lane

    def test_a_sister_already_off_its_load_switch_is_not_touched(self):
        # The flag can be stale enough that there is no filament at all.
        u, lane, sib = self._u(sib_pos=-50.0)
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []

    def test_and_put_back_afterwards(self):
        # Put back to the STAGED position, which is the contract now -- not
        # merely to a net displacement of zero. Those are the same place only
        # when nothing slipped.
        u, lane, sib = self._u()
        start = sib.pos
        u._restore_sibling(u._clear_sibling("lane8", u._readers[0]))
        assert not sib.hub_obj.state
        assert sib.loaded_to_hub is True
        assert sib.hub_obj.at - sib.pos <= sib.hub_obj.hub_clear_move_dis + 2 * sib.short_move_dis
        assert abs(sib.pos - start) <= sib.hub_obj.hub_clear_move_dis + 2 * sib.short_move_dis

    def test_a_tool_loaded_sister_is_never_moved(self):
        # Retracting filament that is loaded to the toolhead would wreck it.
        u, lane, sib = self._u(tool_loaded=True)
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []

    def test_a_sister_not_staged_at_the_hub_is_never_moved(self):
        # Its position is unknown, so there is nothing safe to give back.
        u, lane, sib = self._u(loaded_to_hub=False)
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []

    def test_an_empty_sister_lane_is_left_alone(self):
        u, lane, sib = self._u()
        type(sib).prep_state = property(lambda self: False)
        try:
            assert u._clear_sibling("lane8", u._readers[0]) is None
            assert sib.moves == []
        finally:
            type(sib).prep_state = property(lambda self: True)

    def test_nothing_moves_during_a_print(self):
        u, lane, sib = self._u()
        u.afc.function.is_printing = lambda: True
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []

    def test_the_feature_can_be_turned_off(self):
        u, lane, sib = self._u()
        u.sibling_tag_adjust = False
        assert u._clear_sibling("lane8", u._readers[0]) is None
        assert sib.moves == []

    def test_a_reader_with_one_lane_has_no_sister(self):
        lane = _FakeLane("lane8")
        u = _shim([_reader("reader0", ["lane8"])], lanes={"lane8": lane})
        assert u._clear_sibling("lane8", u._readers[0]) is None

    def test_a_hub_that_never_clears_still_terminates(self):
        u, lane, sib = self._u(hub_at=-1e9)       # stuck on, always true
        token = u._clear_sibling("lane8", u._readers[0])
        sib.moves.clear()
        u._restore_sibling(token)
        assert len(sib.moves) < 30

    def test_the_restage_never_drives_filament_at_the_hub(self):
        # THE BUG THIS REPLACED. Hunting forward until the hub switch saw the
        # tip parked it AT the hub bore -- the shared Y every other lane has
        # to pass through. lane9 sat in it, lane8 could not get past, the hub
        # never triggered and both jammed. AFC's own load path never goes
        # near the hub: home to the load switch, then feed dist_hub, which is
        # calibrated to stop short of it.
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        sib.moves.clear()
        u._restore_sibling(token)
        assert not sib.hub_obj.state
        # Nothing walked the tip up to the switch looking for it.
        assert sib.pos <= sib.hub_obj.at

    def test_a_reverse_home_alone_would_leave_it_off_the_switch(self):
        # THE BUG THIS FIXES, pinned at the seam. move_to passes
        # `triggered = distance > 0`, so a reverse home to the load endstop
        # stops when the switch RELEASES. Doing only that leaves the tip
        # behind it -- which is what left lane8 reading "LOCKED NOT LOADED"
        # after a scan, and would make prep_post_load a silent no-op.
        u, lane, sib = self._u()
        sib.unit_obj.move_to_load(sib, sib.dist_hub, mod.MoveDirection.NEG)
        assert not sib.raw_load_state

    def test_and_the_forward_home_is_what_puts_it_back_on(self):
        u, lane, sib = self._u()
        sib.unit_obj.move_to_load(sib, sib.dist_hub, mod.MoveDirection.NEG)
        sib.unit_obj.prep_load(sib)
        assert sib.raw_load_state

    def test_the_retracts_run_the_espooler(self):
        # DYNAMIC is literally `abs(distance) > 200`, and every dist_hub on
        # this BoxTurtle is 131-188mm -- so the helper's default would mean no
        # spooler on any retract this module makes, and the filament it pulls
        # back piles up loose on the reel instead of winding on. Feeding is
        # the direction that free-wheels and does not want help.
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        sib.assists.clear()
        sib.homing_moves.clear()
        u._restore_sibling(token)
        back = [m for m in sib.homing_moves if m[0] < 0]
        assert back and all(m[4] == mod.AssistActive.YES for m in back)

    def test_the_forward_home_does_not_fight_the_spool(self):
        # prep_load feeds, which pulls filament off a free-wheeling spool.
        # AFC passes NO there on purpose and this must not override it.
        import inspect
        src = inspect.getsource(mod.AFC_BoxTurtle_rfid._home_back_to_load)
        assert "MoveDirection.NEG" in src        # retract only
        assert "AssistActive.YES" in src

    def test_it_re_stages_the_way_a_load_does(self):
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        sib.homing_moves.clear()
        sib.moves.clear()
        u._restore_sibling(token)
        # Unload back off the switch, load forward onto it, then stage --
        # the same three motions the insert cycle makes. The direction is the
        # whole point: reverse homing stops when the switch RELEASES, so a
        # forward home is required to land ON it.
        homing = [m for m in sib.homing_moves if m[3]]
        assert [m[0] < 0 for m in homing] == [True, False]
        assert all(m[2] == "load" for m in homing)
        assert sib.moves[-1][0] == sib.dist_hub
        assert sib.loaded_to_hub is True

    @pytest.mark.parametrize("slip", [0.0, 0.2, 0.5])
    def test_it_lands_exactly_where_a_fresh_insert_would(self, slip):
        # The real claim, and the reason for using AFC's own moves: the
        # sibling ends up in the state a fresh insert produces -- no better
        # and no worse. dist_hub's own feed slips too, so "no drift at all"
        # would be a promise a normal load does not make either. What matters
        # is that the scan leaves no residue the next load has to undo.
        u, lane, sib = self._u(slip=slip)
        u._restore_sibling(u._clear_sibling("lane8", u._readers[0]))

        # The last move is prep_post_load's own dist_hub feed, and the tip
        # started it from the load switch -- which is the definition of
        # staged. Where exactly that feed lands depends on slip, but so does
        # a normal insert's, so there is no residue for the next load to undo.
        assert sib.moves[-1][0] == sib.dist_hub
        seated_at = sib.pos - sib.dist_hub * (1.0 - slip)
        assert 0.0 < seated_at <= sib.short_move_dis
        assert sib.loaded_to_hub is True
        assert not sib.hub_obj.state

    def test_nothing_is_inserted_between_the_two_proven_moves(self):
        # The retract stops ON the switch (triggered=True, check_trigger=True)
        # so prep_post_load's load_state guard is already satisfied. An extra
        # nudge here would feed BEFORE dist_hub is added on top, putting the
        # tip closer to the hub -- the failure this change exists to remove.
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        sib.moves.clear()
        u._restore_sibling(token)
        # Exactly two motions after the give-back: the homing retract, then
        # prep_post_load's dist_hub feed. Nothing between them.
        assert [m[0] for m in sib.moves] == [75.0, sib.dist_hub]

    def test_without_homing_it_re_feeds_and_clears_the_flag(self):
        # No endstop to stop the retract, so it does not attempt one. A false
        # loaded_to_hub is worse than none: the next load just re-stages.
        u, lane, sib = self._u()
        u.afc.homing_enabled = False
        token = u._clear_sibling("lane8", u._readers[0])
        sib.homing_moves.clear()
        u._restore_sibling(token)
        assert sib.homing_moves == []
        assert sib.loaded_to_hub is False

    def test_a_lane_configured_not_to_stage_is_left_where_it_lands(self):
        u, lane, sib = self._u()
        sib.load_to_hub = False
        token = u._clear_sibling("lane8", u._readers[0])
        sib.moves.clear()
        sib.homing_moves.clear()
        u._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0]   # the give-back, nothing more
        assert sib.homing_moves == []

    def test_a_lane_with_no_hub_object_is_left_after_the_refeed(self):
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        sib.hub_obj = None
        sib.moves.clear()
        u._restore_sibling(token)
        assert [m[0] for m in sib.moves] == [75.0]

    def test_a_restore_failure_is_logged_not_raised(self):
        # The sister was not what the operator asked about; its next load
        # re-homes it anyway.
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        def boom(*a, **k):
            raise RuntimeError("stepper gone")
        u._lane_move = boom
        u._restore_sibling(token)                 # must not raise

    def test_a_clear_failure_does_not_stop_the_read(self):
        u, lane, sib = self._u()
        def boom(*a, **k):
            raise RuntimeError("stepper gone")
        u._lane_move = boom
        assert u._clear_sibling("lane8", u._readers[0]) is None

    def test_it_says_so_when_the_sister_cannot_be_recovered(self):
        # A sister off its own load switch reads as "spool removed". The
        # operator has to learn that from us, not from a puzzling state later.
        u, lane, sib = self._u()
        warned = []
        u.logger = types.SimpleNamespace(
            info=lambda *a: None, warning=lambda m: warned.append(m))
        token = u._clear_sibling("lane8", u._readers[0])
        sib.pos = -500.0                      # came out of the lane entirely
        u._restore_sibling(token)
        assert any("NOT back on its load switch" in m for m in warned)

    def test_it_homes_the_sister_back_onto_the_switch(self):
        u, lane, sib = self._u()
        token = u._clear_sibling("lane8", u._readers[0])
        sib.pos = -5.0                        # re-feed will land it short
        u._restore_sibling(token)
        assert u._seated(sib)

    def test_none_token_restores_nothing(self):
        u, lane, sib = self._u()
        u._restore_sibling(None)
        assert sib.moves == []


class TestItUsesAfcsLogger:
    """AFC's logger writes AFC.log with timestamps and call sites AND echoes
    to the g-code console, which is where anyone debugging a lane already is.
    The stdlib logger this was built with reaches klippy.log only, and only if
    someone thinks to grep it -- which is exactly how a sister lane getting
    pulled out went unexplained for a while."""

    def _u(self, afc):
        r = _reader("reader0", ["lane8"])
        u = AFC_BoxTurtle_rfid.__new__(AFC_BoxTurtle_rfid)
        u.logger = types.SimpleNamespace(
            info=lambda *a: None, warning=lambda *a: None)
        u.reactor = types.SimpleNamespace(
            monotonic=lambda: 0.0,
            register_timer=lambda cb, t: None)
        u.bridge = types.SimpleNamespace(
            logger=u.logger, connected=lambda: False)
        u.printer = types.SimpleNamespace(
            lookup_object=lambda name, default=None: (
                afc if name == "AFC" else default),
            lookup_objects=lambda module: [("AFC_BoxTurtle_rfid reader0", r)])
        u.bambu_master_key = b"\x00" * 16
        u.creality_key = b"\x00" * 16
        u.creality_encryption_key = b"\x00" * 16
        return u, r

    def test_the_module_takes_afcs_logger_at_construction(self):
        """Not at klippy:ready any more.

        load_object CONSTRUCTS AFC when this section is reached first, which is
        how AFC_lane/AFC_buffer/AFC_extruder take it -- so there is no window
        where lines go to klippy.log only, and no swap to forget.
        """
        afc_log = types.SimpleNamespace(info=lambda m: None,
                                        warning=lambda m: None)
        seen = {}

        class _Cfg:
            def get_name(self):
                return "AFC_BoxTurtle_rfid bt"

            def get_printer(self):
                return _P()

        class _P:
            def load_object(self, config, name, default=None):
                seen["name"] = name
                return types.SimpleNamespace(logger=afc_log)

        assert _P().load_object(_Cfg(), "AFC").logger is afc_log
        # And the real module asks for exactly that object.
        import inspect
        src = inspect.getsource(AFC_BoxTurtle_rfid.__init__)
        assert 'load_object(config, "AFC").logger' in src, src

    def test_the_bridge_still_gets_handed_the_logger(self):
        """The readers take AFC's logger themselves now; the bridge cannot.

        It is a plain object this class owns, not a Klipper one with a config
        of its own, so it is the only thing left to hand it to -- and port
        drops log from there.
        """
        afc_log = types.SimpleNamespace(info=lambda m: None,
                                        warning=lambda m: None)
        u, r = self._u(types.SimpleNamespace(logger=afc_log))
        u.logger = afc_log
        u._handle_ready()
        assert u.bridge.logger is afc_log

    def test_ready_does_not_disturb_the_logger(self):
        # The logger is settled in __init__ now, so klippy:ready must leave it
        # alone -- including on a printer where the AFC lookup comes back
        # empty, where the old code would have left the module on a fallback.
        u, r = self._u(None)
        before = u.logger
        u._handle_ready()
        assert u.logger is before

    def test_an_afc_without_a_logger_does_not_clear_ours(self):
        u, r = self._u(types.SimpleNamespace())
        before = u.logger
        u._handle_ready()
        assert u.logger is before

    def test_every_log_call_passes_one_ready_made_message(self):
        # AFC's logger takes a MESSAGE, not printf args -- info(msg, %s, x)
        # would silently bind x to console_only. Catch that by reading the
        # source rather than waiting for the one path that logs an error.
        import inspect
        import re
        src = inspect.getsource(mod)
        for m in re.finditer(r"logger\.(info|warning|debug|error)\(", src):
            depth, i = 0, m.end() - 1
            while i < len(src):
                if src[i] == "(":
                    depth += 1
                elif src[i] == ")":
                    depth -= 1
                    if depth == 0:
                        break
                i += 1
            call = src[m.end():i]
            # A top-level comma outside brackets/quotes means extra args.
            d, q = 0, None
            for ch in call:
                if q:
                    q = None if ch == q else q
                elif ch in "\"'":
                    q = ch
                elif ch in "([{":
                    d += 1
                elif ch in ")]}":
                    d -= 1
                elif ch == "," and d == 0:
                    raise AssertionError(
                        f"printf-style logger call, AFC's logger cannot take "
                        f"it: {call[:70]}")


class TestNoTagReadOnTheReactor:
    """A tag read is hundreds of register round-trips, about a second over
    USB. On a worker thread that is fine; on the reactor it is a second of
    every other Klipper timer not running.

    The sweep's poller is a thread, so the reads DURING motion were always
    safe. The one at the end -- the extra look that covers a tag arriving in
    the final chunk -- was called directly, so every sweep that found nothing
    ended with a reactor stall. That is the timeout path, i.e. the slow case
    made slower."""

    def _u(self):
        lane = _FakeLane("lane8")
        u = _shim([_reader("reader0", ["lane8"])], lanes={"lane8": lane})
        u.threads = []
        u._read_blocking = lambda ln, r: (
            u.threads.append(threading.current_thread().name) or None)
        return u, lane

    def test_the_closing_read_runs_on_a_worker_thread(self):
        u, lane = self._u()
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", ADVANCE=40.0, STEP=20.0))
        assert u.threads, "no read happened at all"
        main = threading.current_thread().name
        assert main not in u.threads, (
            f"a tag read ran on the reactor thread ({main}): {u.threads}")

    def test_every_read_in_a_sweep_is_off_the_reactor(self):
        # Belt and braces: not just the closing one -- no read anywhere in the
        # sweep may land on the calling thread.
        u, lane = self._u()
        u._on_lane_prep_loaded(lane)
        assert u.threads
        assert threading.current_thread().name not in u.threads


class TestWorkerThreadsAreNamed:
    """Every worker carries an afc_ prefixed name, matching the convention
    upstream settled on (afc_moonraker, afc_save_vars). It is the name that
    shows up in top and ps -L, so it is what tells you which thread is busy
    when the host is struggling."""

    def test_the_read_and_sweep_threads_are_named(self):
        lane = _FakeLane("lane8")
        u = _shim([_reader("reader0", ["lane8"])], lanes={"lane8": lane})
        seen = []
        u._read_blocking = lambda ln, r: (
            seen.append(threading.current_thread().name) or None)
        u.cmd_AFC_BT_RFID_STAGE(_Gcmd(LANE="lane8", ADVANCE=40.0, STEP=20.0))
        assert seen
        assert all(n.startswith("afc_bt_rfid_") for n in seen), seen


class TestTheConnectTimerStaysOffTheReactor:
    """Opening the port and reading VersionReg are blocking serial ops behind
    a lock a sweep worker can be holding, each bounded by _OP_TIMEOUT_S.

    They used to run on the reactor timer, which meant the reactor could sit
    for a quarter second per offline reader plus however long the sweep in
    front of it held the lock -- and Klipper measures its own lateness in
    milliseconds. Retrying offline readers (which this module now does) made
    that fire far more often, so the work moved to a thread.
    """

    _Reactor = TestConnectTimerOwnsThePort._Reactor
    _Bridge = TestConnectTimerOwnsThePort._Bridge
    _Link = TestConnectTimerOwnsThePort._Link

    def test_the_tick_itself_does_no_port_io(self):
        import time as _t
        bridge = TestConnectTimerOwnsThePort._Bridge()
        u, r = TestConnectTimerOwnsThePort._ready_shim(self, bridge)
        u._handle_ready()

        slow = threading.Event()

        class _SlowLink:
            def reg_read(self, reg):
                # Long enough that it cannot expire on its own under load:
                # the test releases it. A short hold made this flaky.
                slow.wait(30.0)         # stands in for a timing-out bus
                return 0x92

        r.link = _SlowLink()
        t0 = _t.time()
        u.reactor.timers[0][0](101.0)   # the TICK, not the helper
        elapsed = _t.time() - t0
        slow.set()
        assert elapsed < 0.25, (
            f"the tick blocked for {elapsed:.2f}s -- port I/O is back on "
            f"the reactor")
        deadline = _t.time() + 5.0
        while u._probing and _t.time() < deadline:
            _t.sleep(0.002)

    def test_only_one_probe_thread_at_a_time(self):
        """The tick is faster than a probe on a timing-out port, so without
        the guard a wedged bus spawns a thread every _RETRY_S forever."""
        import time as _t
        bridge = TestConnectTimerOwnsThePort._Bridge()
        u, r = TestConnectTimerOwnsThePort._ready_shim(self, bridge)
        u._handle_ready()
        hold = threading.Event()
        calls = []

        class _HeldLink:
            def reg_read(self, reg):
                calls.append(reg)
                hold.wait(30.0)      # released by the test, never by timeout
                return 0x92

        r.link = _HeldLink()
        tick = u.reactor.timers[0][0]
        for i in range(4):
            tick(101.0 + 5.0 * i)       # three of these must be no-ops
        assert len(calls) == 1, calls
        hold.set()
        deadline = _t.time() + 5.0
        while u._probing and _t.time() < deadline:
            _t.sleep(0.002)

    def test_the_flag_clears_even_when_the_probe_explodes(self):
        """A probe that raises must not wedge the retry loop shut."""
        import time as _t
        bridge = TestConnectTimerOwnsThePort._Bridge()
        u, r = TestConnectTimerOwnsThePort._ready_shim(self, bridge)
        u._handle_ready()
        u._probe_all = lambda: (_ for _ in ()).throw(RuntimeError("boom"))
        u.reactor.timers[0][0](101.0)
        deadline = _t.time() + 5.0
        while u._probing and _t.time() < deadline:
            _t.sleep(0.002)
        assert not u._probing
