# Tests for extras/AFC_BambuAMS_bridge.py — the serial transport.
#
# This module imports only the standard library, so it can be driven end to end
# with a fake serial port and no printer. That matters because the reader
# thread, the reconnect path and the event dispatch are where a fault is
# invisible — the bridge keeps reporting the last thing it knew.
from __future__ import annotations

import json
import logging
import logging.handlers
import threading
import time
import types

import pytest

import extras.AFC_BambuAMS_bridge as br
from extras.AFC_BambuAMS_bridge import BambuBridge


class _Reactor:
    def __init__(self):
        self._now = 100.0
        self.async_cbs = []

    def monotonic(self):
        return self._now

    def advance(self, dt):
        self._now += dt

    def register_async_callback(self, cb):
        self.async_cbs.append(cb)

    def run_pending(self):
        cbs, self.async_cbs = self.async_cbs, []
        for cb in cbs:
            cb(0.0)


class _Logger:
    def __init__(self):
        self.msgs = []
        self.file_only = []

    def info(self, m):
        self.msgs.append(("info", m))

    def warning(self, m):
        self.msgs.append(("warning", m))

    def debug(self, m, only_debug=False, traceback=None):
        self.msgs.append(("debug", m))
        if only_debug:
            self.file_only.append(m)

    def texts(self, lvl=None):
        return [m for l, m in self.msgs if lvl is None or l == lvl]


class _Serial:
    """Fake serial: replays queued chunks, then blocks-as-empty."""

    def __init__(self, chunks=(), fail_on_read=None):
        self.chunks = list(chunks)
        self.written = []
        self.closed = False
        self._fail = fail_on_read

    def read(self, n):
        if self._fail is not None:
            raise self._fail
        return self.chunks.pop(0) if self.chunks else b""

    def write(self, data):
        self.written.append(data)

    def close(self):
        self.closed = True


def _bridge(serial=None, factory=None):
    r, lg = _Reactor(), _Logger()
    f = factory or (lambda: serial if serial is not None else _Serial())
    b = BambuBridge(f, r, lg)
    b._serial = serial if serial is not None else _Serial()
    return b, r, lg


def _feed(b, obj_json):
    b.handle_line(obj_json)


class TestEventDispatch:
    """Every event the firmware can send must land somewhere. An event nothing
    consumes is surfaced (to file) rather than dropped -- silence makes 'the
    command never landed' and 'the reply never came' indistinguishable."""

    def test_unknown_event_is_surfaced_file_only(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"nonsense","x":1}')
        assert any("unhandled bridge event" in m for m in lg.file_only)

    def test_routine_command_echoes_are_not_surfaced(self):
        # These arrive on every prep; they would be console noise at startup.
        b, r, lg = _bridge()
        for e in ("mcaddr", "armms", "hb", "mute", "units"):
            _feed(b, '{"evt":"%s"}' % e)
        assert not any("unhandled" in m for m in lg.file_only)

    def test_sniff_frames_are_file_only(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"sniff","hex":"3DC5"}')
        assert any(m.startswith("SNIFF 3DC5") for m in lg.file_only)
        assert not lg.texts("info")

    def test_error_event_warns(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"error","msg":"bus down"}')
        assert any("bus down" in m for m in lg.texts("warning"))

    def test_ack_is_logged(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"ack","cmd":"dry","slot":55}')
        assert any("bridge ack dry (slot 55)" in m for m in lg.texts("debug"))

    def test_reply_is_cached_for_the_probe(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"reply","hex":"3D05AA"}')
        assert b._last_raw_reply == "3D05AA"

    def test_garbage_line_is_ignored(self):
        b, r, lg = _bridge()
        _feed(b, "not json at all")
        _feed(b, "")
        assert lg.msgs == []


class TestChainMap:
    """The chain reply carries the enrollment map plus the running firmware
    version -- the only way to confirm a flash actually took."""

    def test_uids_are_split_and_uppercased(self):
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"aabb,ccdd"}')
        assert b.chain_uids() == ["AABB", "CCDD"]

    def test_empty_fields_are_kept_so_indices_do_not_shift(self):
        # Dropping a blank would renumber every later unit on the wire.
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"AA,,CC"}')
        assert b.chain_uids() == ["AA", "", "CC"]

    def test_no_uids_is_an_empty_list(self):
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":""}')
        assert b.chain_uids() == []

    def test_diagnostics_ride_along(self):
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"AA","htmask":5,"fw":"1.0.10.6",'
                 '"selid":2,"selsent":7,"selack":6}')
        htmask, fw, sel = b.chain_diag()
        assert (htmask, fw, sel) == (5, "1.0.10.6", (2, 7, 6))

    def test_malformed_diagnostics_fall_back_to_defaults(self):
        # Older firmware omits them; a bad value must not poison the map.
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"AA","htmask":"x","selid":"y"}')
        htmask, fw, sel = b.chain_diag()
        assert htmask == 0 and sel == (-1, 0, 0)

    def test_diag_defaults_before_any_chain_reply(self):
        b, _, _ = _bridge()
        assert b.chain_diag() == (0, "", (-1, 0, 0))


class TestNarrationToConsole:
    """The AMS narrates continuously. Only curated lines reach the console, at
    most one a second, and never the same line twice in a row."""

    def _b(self):
        b, r, lg = _bridge()
        b.name = "BambuAMS_1"
        return b, r, lg

    # One dialect-agnostic rule, so one message. Splitting it per prefix
    # ([AMS_DEV] vs [AMS_RFID]) gives the same event two phrasings and still
    # misses the third unit's punctuation.
    def test_a_matched_line_is_rendered_in_english(self):
        b, r, lg = self._b()
        b._narrate_human("[AMS_DEV] STEP:card auth success!", 100.0)
        assert any("tag authenticated" in m for m in lg.texts("info"))

    def test_every_dialect_renders_the_SAME_message(self):
        """One event, one sentence -- whichever unit said it.

        Anchored on the AUTHENTICATION rather than "read success": an HT
        emits "read success ,goto Cali" on an attempt that then FAILS and
        retries, so that phrasing was removed from the console table for
        announcing a read which had not happened."""
        for line in ("[AMS_DEV] STEP:card auth success!",
                     "[AMS_RFID]STEP:card auth success!",
                     "[AMS_RFID] STEP3,auth card successful"):
            b, r, lg = self._b()
            b._narrate_human(line, 100.0)
            assert any("tag authenticated" in m for m in lg.texts("info")), line

    def test_the_same_line_twice_is_said_once(self):
        b, r, lg = self._b()
        b._narrate_human("[AMS_DEV] STEP:card auth success!", 100.0)
        b._narrate_human("[AMS_DEV] STEP:card auth success!", 200.0)
        assert len([m for m in lg.texts("info")
                    if "tag authenticated" in m]) == 1

    def test_a_burst_is_rate_limited_to_one_a_second(self):
        b, r, lg = self._b()
        b._narrate_human("[AMS_DEV] STEP:card auth success!", 100.0)
        b._narrate_human("[RF] tray0: info write to flash", 100.2)
        assert len(lg.texts("info")) == 1

    def test_an_unmatched_line_says_nothing(self):
        b, r, lg = self._b()
        b._narrate_human("[AMS_FOO] something unremarkable", 100.0)
        assert lg.texts("info") == []

    def test_chamber_telemetry_updates_the_cached_reading(self):
        b, r, lg = self._b()
        b._narrate_human(
            "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:22.0", 100.0)
        assert b._chmb_temp == 23.1
        assert b._chmb_target == 55.0
        assert b._chmb_state == 2
        assert b._chmb_t_seen == 100.0

    def test_unparseable_chamber_numbers_leave_the_cache_alone(self):
        b, r, lg = self._b()
        b._chmb_temp = None
        b._narrate_human("[AMS_CHMB]s:x, rf:y|vt:z", 100.0)
        assert b._chmb_temp is None


class TestRawNarrationRouting:
    """Bus chatter goes to file; anything that says something stays on the
    console, because with AFC's debug flag on that is where an operator
    watches a load happen."""

    def test_pure_chatter_is_file_only(self):
        b, r, lg = _bridge()
        b.name = "u"
        _feed(b, '{"evt":"amsdbg","text":"[AMS_CALL] ams0 select,select ams1"}')
        assert lg.file_only

    def test_narration_with_content_is_not_file_only(self):
        b, r, lg = _bridge()
        b.name = "u"
        _feed(b, '{"evt":"amsdbg","text":"[AMS_SWITCH]feed finish -1, stall"}')
        assert not lg.file_only

    def test_repeated_lines_are_deduped_then_re_emitted_with_a_count(self):
        b, r, lg = _bridge()
        b.name = "u"
        line = '{"evt":"amsdbg","text":"[AMS_TRAY]tray[0] sw_sta update"}'
        _feed(b, line)
        _feed(b, line)
        assert len(lg.texts("debug")) == 1        # second is suppressed
        r.advance(61.0)
        _feed(b, line)
        assert any("x2 repeated" in m or "x3 repeated" in m
                   for m in lg.texts("debug"))


class TestReaderThread:
    """The reader must survive a port that disappears -- a transient USB glitch
    should self-heal rather than brick the bridge until a Klipper restart."""

    def test_lines_are_split_on_newlines_across_chunks(self):
        seen = []
        s = _Serial([b'{"evt":"ack","cmd":"a","slot":1}\n{"evt":"ac',
                     b'k","cmd":"b","slot":2}\n'])
        b, r, lg = _bridge(serial=s)
        b.handle_line = lambda l: seen.append(l)
        b._run = True
        t = threading.Thread(target=b._reader, daemon=True)
        t.start()
        time.sleep(0.2)
        b._run = False
        t.join(timeout=2)
        assert len(seen) == 2 and '"cmd":"b"' in seen[1]

    def test_a_read_error_drops_the_port_and_reconnects(self):
        opened = []

        def factory():
            opened.append(1)
            # First port fails on read; the replacement is quiet.
            return _Serial(fail_on_read=OSError("input/output error")) \
                if len(opened) == 1 else _Serial()

        b, r, lg = _bridge(serial=factory(), factory=factory)
        b._run = True
        t = threading.Thread(target=b._reader, daemon=True)
        t.start()
        time.sleep(0.3)
        b._run = False
        t.join(timeout=2)
        assert any("read failed" in m for m in lg.texts("warning"))
        assert len(opened) >= 2, "should have reopened the port"

    def test_reconnect_notifies_listeners_on_the_reactor(self):
        # A reconnect usually means the Pico rebooted, so units must re-push
        # their config -- and that has to happen on the reactor, not here.
        calls = []
        b, r, lg = _bridge(serial=_Serial())
        b.add_reconnect_listener(lambda: calls.append(1))
        b._serial = None                      # force the reconnect branch
        b._run = True
        t = threading.Thread(target=b._reader, daemon=True)
        t.start()
        time.sleep(0.2)
        b._run = False
        t.join(timeout=2)
        r.run_pending()
        assert calls == [1]
        assert any("reconnected" in m for m in lg.texts("info"))

    def test_a_failing_factory_backs_off_instead_of_spinning(self):
        tries = []

        def factory():
            tries.append(time.time())
            raise OSError("no such port")

        b, r, lg = _bridge(serial=_Serial(), factory=factory)
        b._serial = None
        b._run = True
        t = threading.Thread(target=b._reader, daemon=True)
        t.start()
        time.sleep(0.6)
        b._run = False
        t.join(timeout=3)
        # 0.5s initial backoff: a spin would be hundreds of attempts.
        assert 1 <= len(tries) <= 4, tries


class TestStopAndDropPort:
    def test_stop_closes_the_port(self):
        s = _Serial()
        b, _, _ = _bridge(serial=s)
        b.stop()
        assert b._run is False and s.closed

    def test_stop_survives_a_close_that_throws(self):
        s = _Serial()
        s.close = lambda: (_ for _ in ()).throw(OSError("already gone"))
        b, _, _ = _bridge(serial=s)
        b.stop()                       # must not raise
        assert b._run is False

    def test_stop_with_no_port_is_a_noop(self):
        b, _, _ = _bridge()
        b._serial = None
        b.stop()

    def test_drop_port_clears_and_closes(self):
        s = _Serial()
        b, _, _ = _bridge(serial=s)
        b._drop_port()
        assert b._serial is None and s.closed

    def test_drop_port_survives_a_close_that_throws(self):
        s = _Serial()
        s.close = lambda: (_ for _ in ()).throw(OSError("gone"))
        b, _, _ = _bridge(serial=s)
        b._drop_port()
        assert b._serial is None


class TestMalformedNarrationIsSurvivable:
    """Every one of these is a defensive branch on the READER THREAD. If any of
    them let an exception out, the thread dies and the bridge goes quiet while
    still reporting the last state it knew -- the worst failure mode this
    module has, and the one that is hardest to notice."""

    def test_chamber_numbers_that_match_but_will_not_parse(self):
        # The regex captures [0-9.]+, so "1.2.3" matches and float() still
        # fails. The cached reading must be left alone rather than crashing.
        b, r, lg = _bridge()
        b.name = "u"
        b._chmb_temp = 42.0
        b._narrate_human("[AMS_CHMB]s:2, rf:55, cd:55, vt:1.2.3, ap:22.0", 100.0)
        assert b._chmb_temp == 42.0

    def test_motor_current_that_matches_but_will_not_parse(self):
        b, r, lg = _bridge()
        b.name = "u"
        b._bldc_i = None
        _feed(b, '{"evt":"amsdbg","text":"[AMS_SWITCH]feed bldc_i:1.2.3A"}')
        assert b._bldc_i is None

    def test_a_valid_motor_current_is_cached(self):
        b, r, lg = _bridge()
        b.name = "u"
        _feed(b, '{"evt":"amsdbg","text":"[AMS_SWITCH]feed bldc_i:0.319A"}')
        assert b._bldc_i == pytest.approx(0.319)

    def test_a_throwing_narrator_does_not_break_the_reader(self):
        b, r, lg = _bridge()
        b.name = "u"
        b._narrate_human = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("x"))
        _feed(b, '{"evt":"amsdbg","text":"[AMS_SWITCH]feed finish -1, stall"}')
        # The stall was still recorded despite the narrator blowing up.
        assert b.last_fault()[0] == 1

    def test_the_ten_second_heartbeat_line_is_dropped_entirely(self):
        # "[DBG] ams time" carries nothing and its timestamp defeats the
        # dedupe, so it would log forever at 6 lines a minute.
        b, r, lg = _bridge()
        b.name = "u"
        _feed(b, '{"evt":"amsdbg","text":"[DBG] ams time 12345"}')
        assert lg.msgs == []


class TestReconnectListenerFailure:
    """A listener that cannot be scheduled must not stop the OTHER listeners
    being scheduled, or one bad unit silently strands the rest un-announced
    after a Pico reboot."""

    def test_a_failing_reactor_schedule_is_swallowed(self):
        b, r, lg = _bridge(serial=_Serial())
        b.add_reconnect_listener(lambda: None)
        calls = []

        def boom(cb):
            calls.append(1)
            raise RuntimeError("reactor gone")
        r.register_async_callback = boom
        b._serial = None
        b._run = True
        t = threading.Thread(target=b._reader, daemon=True)
        t.start()
        time.sleep(0.2)
        b._run = False
        t.join(timeout=2)
        assert calls, "it tried to schedule"
        assert any("reconnected" in m for m in lg.texts("info"))


# ── Dedicated narration log ───────────────────────────────────────────────────

class TestNarrationLog:
    """Narration used to reach AFC.log only through logger.debug(), which AFC's
    `debug` flag gates. Turn debug off -- the normal state for a working
    printer, since the AMS narrates continuously -- and every STEP, finish,
    stall and measured length vanished. That is the record you want when
    something goes wrong, and exactly when nobody has debug on."""

    def _feed(self, bridge, text, addr=None):
        obj = {"evt": "amsdbg", "text": text}
        if addr is not None:
            obj["addr"] = addr
        bridge.handle_line(json.dumps(obj))

    def _clear(self):
        lg = logging.getLogger("AFC_BambuAMS_file")
        for h in list(lg.handlers):
            h.close()
            lg.removeHandler(h)

    def setup_method(self):
        self._clear()

    def teardown_method(self):
        self._clear()

    def test_it_writes_narration_to_its_own_file(self, tmp_path):
        b, _r, _l = _bridge()
        assert b.set_narration_log(str(tmp_path)) is True
        self._feed(b, "[AMS_SWITCH]feed finish 0, dw_len:3.508 m", addr=0x1800)
        for h in logging.getLogger("AFC_BambuAMS_file").handlers:
            h.flush()
        text = (tmp_path / "AFC_BambuAMS.log").read_text()
        assert "dw_len:3.508 m" in text

    def test_the_address_is_recorded_for_attribution(self, tmp_path):
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path))
        self._feed(b, "[AMS_SWITCH]pull finish 0", addr=0x1800)
        for h in logging.getLogger("AFC_BambuAMS_file").handlers:
            h.flush()
        assert "0x1800" in (tmp_path / "AFC_BambuAMS.log").read_text()

    def test_repeats_are_kept_verbatim(self, tmp_path):
        # The console dedupes; the file must not. A line repeating hundreds of
        # times is how a stuck loop looks, and collapsing it hides the shape.
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path))
        for _ in range(5):
            self._feed(b, "[AMS_IDLE]set ams state assist, mode:4", addr=0x700)
        for h in logging.getLogger("AFC_BambuAMS_file").handlers:
            h.flush()
        body = (tmp_path / "AFC_BambuAMS.log").read_text()
        assert body.count("set ams state assist") == 5

    def test_it_does_not_propagate_into_afc_log(self, tmp_path):
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path))
        assert logging.getLogger("AFC_BambuAMS_file").propagate is False

    def test_rotation_defaults_to_10mb_with_no_backups(self, tmp_path):
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path))
        h = [x for x in logging.getLogger("AFC_BambuAMS_file").handlers
             if isinstance(x, logging.handlers.RotatingFileHandler)][0]
        assert h.maxBytes == 10 * 1024 * 1024
        assert h.backupCount == 0

    def test_it_actually_rotates_and_keeps_nothing(self, tmp_path):
        # backupCount=0 means the file is truncated, not renamed: a rolling
        # window for diagnosis, not an archive filling an SD card.
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path), max_bytes=200)
        for i in range(60):
            self._feed(b, "[AMS_SWITCH]line %d padding padding padding" % i)
        for h in logging.getLogger("AFC_BambuAMS_file").handlers:
            h.flush()
        assert (tmp_path / "AFC_BambuAMS.log").stat().st_size <= 400
        assert not list(tmp_path.glob("AFC_BambuAMS.log.*"))

    def test_an_unwritable_directory_is_reported_not_raised(self, tmp_path):
        b, _r, logger = _bridge()
        assert b.set_narration_log("/nonexistent-dir-xyz") is False
        assert any("could not open" in str(m) for _lvl, m in logger.msgs)

    def test_narration_without_a_log_is_a_safe_noop(self):
        # Never configured: handle_line must not care.
        b, _r, _l = _bridge()
        self._feed(b, "[AMS_SWITCH]feed finish 0")     # must not raise

    def test_setup_is_idempotent(self, tmp_path):
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path))
        assert b.set_narration_log(str(tmp_path)) is True
        n = len([h for h in logging.getLogger("AFC_BambuAMS_file").handlers
                 if isinstance(h, logging.handlers.RotatingFileHandler)])
        assert n == 1

    def test_a_preexisting_unrelated_handler_does_not_defeat_setup(self, tmp_path):
        # logging.getLogger() is process-global. A truthy `if not lg.handlers`
        # check would skip setup here and hand back a logger with no file --
        # reporting success and writing nowhere. That bug shipped once.
        lg = logging.getLogger("AFC_BambuAMS_file")
        lg.addHandler(logging.NullHandler())
        try:
            b, _r, _l = _bridge()
            assert b.set_narration_log(str(tmp_path)) is True
            self._feed(b, "[AMS_SWITCH]feed finish 0, dw_len:3.5 m")
            for h in lg.handlers:
                h.flush()
            assert "dw_len" in (tmp_path / "AFC_BambuAMS.log").read_text()
        finally:
            self._clear()


class TestChainMcAddr:
    """What the FIRMWARE holds per unit, not what the host thinks it announced.
    An unset address drops the narration log drain back to the captured 0x0700
    pair, which never asks an AMS HT at 0x1800 -- a failure that was invisible
    from Klipper and cost an afternoon of guessing."""

    def test_it_is_read_from_the_chain_reply(self):
        b, _r, _l = _bridge()
        b.handle_line(json.dumps(
            {"evt": "chain", "uids": "", "mcaddr": [6144, 1792]}))
        assert b.chain_mcaddr() == [6144, 1792]

    def test_absent_is_none_not_empty(self):
        # None = firmware too old to report. [] / zeros = reported and unset.
        # Conflating them would turn "cannot tell" into "definitely broken".
        b, _r, _l = _bridge()
        b.handle_line(json.dumps({"evt": "chain", "uids": ""}))
        assert b.chain_mcaddr() is None

    def test_all_zero_is_reported_as_such(self):
        b, _r, _l = _bridge()
        b.handle_line(json.dumps({"evt": "chain", "uids": "", "mcaddr": [0, 0]}))
        assert b.chain_mcaddr() == [0, 0]

    def test_before_any_chain_reply_it_is_none(self):
        b, _r, _l = _bridge()
        assert b.chain_mcaddr() is None


class TestMcAddrAck:
    """The firmware echoes what bb_get_mc_addr() reads back AFTER applying an
    mcaddr command, so the echo is a receipt, not a repeat of the request. It
    is the only way to tell a dropped announce from an applied one, and that
    distinction is what makes the narration drain fall back to 0x0700."""

    def test_unacknowledged_unit_is_none(self):
        b, r, lg = _bridge()
        assert b.mcaddr_ack(0) is None

    def test_the_echo_is_recorded_per_unit(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"mcaddr","unit":0,"addr":6144}')
        _feed(b, '{"evt":"mcaddr","unit":1,"addr":1792}')
        assert b.mcaddr_ack(0) == 6144           # 0x1800, an HT
        assert b.mcaddr_ack(1) == 1792           # 0x0700, a boxed AMS

    def test_an_address_that_did_not_take_records_zero_not_none(self):
        # Acknowledged-as-unset and never-acknowledged are different faults:
        # one is the firmware refusing, the other the command not arriving.
        b, r, lg = _bridge()
        _feed(b, '{"evt":"mcaddr","unit":0,"addr":0}')
        assert b.mcaddr_ack(0) == 0
        assert b.mcaddr_ack(0) is not None

    def test_a_later_echo_replaces_the_earlier_one(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"mcaddr","unit":0,"addr":1792}')
        _feed(b, '{"evt":"mcaddr","unit":0,"addr":6144}')
        assert b.mcaddr_ack(0) == 6144

    def test_a_malformed_echo_does_not_take_the_reader_down(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"mcaddr","unit":"x","addr":"y"}')
        assert b.mcaddr_ack(0) is None

    def test_it_is_still_a_known_event_and_not_logged_as_unhandled(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"mcaddr","unit":0,"addr":6144}')
        assert not any("unhandled" in m for m in lg.file_only)


class TestFstateTrace:
    """fstate is what the move-completion wait keys on. Whether it actually
    moves during a load is a question for a trace sharing the narration's
    clock, not for reasoning -- so every change is recorded, and only changes."""

    def _clear(self):
        lg = logging.getLogger("AFC_BambuAMS_file")
        for h in list(lg.handlers):
            h.close()
            lg.removeHandler(h)

    def setup_method(self):
        self._clear()

    def teardown_method(self):
        self._clear()

    def _b(self, tmp_path):
        b, r, lg = _bridge()
        b.set_narration_log(str(tmp_path))
        return b, r, lg

    def _lines(self, tmp_path):
        for h in logging.getLogger("AFC_BambuAMS_file").handlers:
            h.flush()
        p = tmp_path / "AFC_BambuAMS.log"
        return p.read_text().splitlines() if p.exists() else []

    def test_the_first_frame_is_recorded(self, tmp_path):
        # A unit that comes up in a mode and never leaves it is itself the
        # finding, so the opening value must not be swallowed as "no change".
        b, r, lg = self._b(tmp_path)
        _feed(b, '{"evt":"status","fstate":4,"buff":59}')
        assert any("fstate - -> 4" in m for m in self._lines(tmp_path))

    def test_a_change_is_recorded_with_both_ends(self, tmp_path):
        b, r, lg = self._b(tmp_path)
        _feed(b, '{"evt":"status","fstate":0}')
        _feed(b, '{"evt":"status","fstate":2}')
        assert any("fstate 0 -> 2" in m for m in self._lines(tmp_path))

    def test_repeats_are_not_recorded(self, tmp_path):
        # Several frames a second; logging every one would bury the narration.
        b, r, lg = self._b(tmp_path)
        for _ in range(20):
            _feed(b, '{"evt":"status","fstate":4}')
        assert len([m for m in self._lines(tmp_path) if "fstate" in m]) == 1

    def test_the_buffer_reading_rides_along(self, tmp_path):
        # Buffer position is the other live number during a feed; having it on
        # the same line is what makes the trace readable.
        b, r, lg = self._b(tmp_path)
        _feed(b, '{"evt":"status","fstate":2,"buff":97}')
        assert any("buff=97" in m for m in self._lines(tmp_path))

    def test_no_narration_log_configured_is_a_noop(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"status","fstate":2}')     # must not raise

    def test_status_listeners_still_run(self, tmp_path):
        # The trace is inserted into the status path; it must not displace it.
        b, r, lg = self._b(tmp_path)
        seen = []
        b.add_listener(lambda o: seen.append(o))
        _feed(b, '{"evt":"status","fstate":2}')
        for cb in r.async_cbs:
            cb(0)
        assert len(seen) == 1


class TestMotionFinishIsNotJustTheWordFinish:
    """_wait_move returns the instant the finish sequence bumps, so what counts
    as a finish decides where AFC thinks the filament is. Verbatim lines from
    an AMS HT load, in the order the unit emitted them."""

    def _seq(self, b):
        return b.last_finish()[0]

    def _say(self, b, text):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text,
                                  "addr": 0x1800}))

    def test_a_real_feed_completion_counts(self):
        b, r, lg = _bridge()
        before = self._seq(b)
        self._say(b, "[AMS_SWITCH]feed finish, buff_pos:1.29, bldc_i:1.593A")
        assert self._seq(b) == before + 1

    def test_the_ams2_form_with_an_index_counts(self):
        b, r, lg = _bridge()
        before = self._seq(b)
        self._say(b, "[AMS_SWITCH]feed finish 0, dw_len:3.508 m")
        assert self._seq(b) == before + 1

    def test_a_pull_completion_counts(self):
        b, r, lg = _bridge()
        before = self._seq(b)
        self._say(b, "[AMS_SWITCH]pull finish 0, tray_sw:0, len_det:0.265 m")
        assert self._seq(b) == before + 1

    def test_a_state_machine_switch_does_NOT_count(self):
        # Emitted ~10 times in the seconds before the feed completes. Counting
        # it called the load done somewhere mid-bowden.
        b, r, lg = _bridge()
        before = self._seq(b)
        for _ in range(10):
            self._say(b, "[AMS_SWITCH]AMS_CTRL_state_switch finish, "
                         "sucessful, err_code:0x00")
        assert self._seq(b) == before

    def test_the_follower_dropping_does_NOT_count(self):
        b, r, lg = _bridge()
        before = self._seq(b)
        self._say(b, "[AMS_COMMON]mode: 4 -> 0 [AMS_SWITCH]assist finish 0, "
                     "ref:0 [AMS_LED]other to idle 0")
        assert self._seq(b) == before

    def test_a_preload_completion_still_counts(self):
        b, r, lg = _bridge()
        before = self._seq(b)
        self._say(b, "[AMS_PRELOAD]preload finish")
        assert self._seq(b) == before + 1

    def test_a_real_finish_in_a_blob_of_noise_still_counts(self):
        # Narration arrives as several bracketed segments per line, so the
        # completion routinely shares a line with the noise above.
        b, r, lg = _bridge()
        before = self._seq(b)
        self._say(b, "[AMS_SWITCH]feed finish, buff_pos:1.29 [AMS_IDLE]set "
                     "ams_state:2 --> 0 [AMS_SWITCH]AMS_CTRL_state_switch "
                     "finish, sucessful, err_code:0x00")
        assert self._seq(b) == before + 1

    def test_a_stalled_completion_is_still_reported_but_not_ok(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]feed finish -1, stall")
        seq, ok, _t = b.last_finish()
        assert seq and ok is False


class TestStallIsNotAlwaysFailure:
    """An AMS HT ends a NORMAL load by feeding to the end of its PTFE and
    stalling against the extruder gear -- that is how it knows it arrived.
    Reading the word "stall" as failure marks a good load failed. What
    separates the two is how far it got, not that it stopped."""

    def _say(self, b, text, addr=0x1800):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr}))
        return b.last_finish()[1]

    def test_a_clean_finish_is_ok(self):
        b, r, lg = _bridge()
        assert self._say(
            b, "[AMS_SWITCH]feed finish, buff_pos:1.28, bldc_i:1.595A") is True

    def test_the_ht_end_of_load_stall_is_ok(self):
        # Verbatim: 18 mm short of a 3619 mm path.
        b, r, lg = _bridge()
        assert self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m, "
               "tube_len:3.619 m") is True

    def test_a_genuinely_short_stall_is_not_ok(self):
        # Verbatim shape of the unload that really did come up short: 336 mm
        # out, and it needed its retry.
        b, r, lg = _bridge()
        assert self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:3.283 m, "
               "tube_len:3.619 m") is False

    def test_a_stall_at_the_very_start_is_not_ok(self):
        b, r, lg = _bridge()
        assert self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:0.050 m, "
               "tube_len:3.619 m") is False

    def test_the_stored_measurement_is_used_when_the_line_omits_it(self):
        # A stall line without tube_len must still be judged against the right
        # distance rather than defaulting to failure.
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]old tube_len:3619 mm, list:3617,3645,0 mm")
        assert self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m") is True
        assert self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:1.000 m") is False

    def test_a_stall_with_nothing_to_judge_against_stays_a_failure(self):
        # No len_det, no measurement: the safe reading is that it failed.
        b, r, lg = _bridge()
        assert self._say(b, "[AMS_SWITCH]feed finish -1, stall") is False

    def test_a_clean_finish_sharing_the_line_wins(self):
        # Exactly what the HT emitted: the stall and the real completion
        # arrive in one narration blob.
        b, r, lg = _bridge()
        assert self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m, "
               "tube_len:3.619 m [AMS_RFID] STEP,odom reset tray 0 "
               "[AMS_SWITCH]feed finish, buff_pos:1.28, bldc_i:1.600A") is True

    def test_the_minus_one_form_alone_does_not_read_as_clean(self):
        # The clean-finish pattern must not match "feed finish -1".
        b, r, lg = _bridge()
        assert self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:0.100 m, "
               "tube_len:3.619 m") is False

    def test_tolerance_is_clear_of_both_measured_cases(self):
        from extras.AFC_BambuAMS_bridge import FINISH_ARRIVAL_TOLERANCE_MM
        assert FINISH_ARRIVAL_TOLERANCE_MM > 18      # normal end-of-load
        assert FINISH_ARRIVAL_TOLERANCE_MM < 336     # the real short unload

    def test_a_stalled_completion_still_bumps_the_sequence(self):
        # Whatever the verdict, the caller must be told the move ended --
        # otherwise it waits out the deadline it was meant to be spared.
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        self._say(b, "[AMS_SWITCH]feed finish -1, stall")
        assert b.last_finish()[0] == before + 1


class TestFinishJudgementDoesNotDeadlock:
    """_finish_succeeded reads tube_len(), which takes the same non-reentrant
    lock the finish bookkeeping holds. Judging inside that with-block wedged
    the reader thread solid -- no error, no narration, no status frames, just
    a bridge that stops. Pinned because the failure is silent."""

    def test_a_stall_line_that_consults_the_measurement_returns(self):
        b, r, lg = _bridge()
        b.handle_line(json.dumps({"evt": "amsdbg", "addr": 0x1800,
                                  "text": "[AMS_SWITCH]old tube_len:3619 mm"}))
        # Would hang forever, not fail, if the lock were taken twice.
        b.handle_line(json.dumps({
            "evt": "amsdbg", "addr": 0x1800,
            "text": "[AMS_SWITCH]feed finish -1, stall, len_det:3.601 m"}))
        assert b.last_finish()[1] is True

    def test_the_lock_is_free_afterwards(self):
        b, r, lg = _bridge()
        b.handle_line(json.dumps({
            "evt": "amsdbg", "addr": 0x1800,
            "text": "[AMS_SWITCH]feed finish -1, stall, len_det:1.0 m"}))
        assert b.tube_len(0x1800) is None      # takes the lock again


class TestOdometerCompletions:
    """A boxed AMS narrates in the [AMS_DEV] dialect and NEVER says "finish",
    so without these its moves each run the full 35 s watchdog. It does say
    when a tray engages and when one leaves -- in odometer terms. Lines and
    order verbatim from one load and one unload of lane15."""

    def _say(self, b, text, addr=0x0700):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr}))
        return b.last_finish()

    def test_an_odom_reset_completes_a_feed(self):
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        seq, ok, _t = self._say(b, "[AMS_DEV] STEP:odom reset tray 0")
        assert seq == before + 1 and ok is True

    def test_the_tray_going_away_completes_a_retract(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_DEV] STEP:odom reset tray 0")     # engaged
        before = b.last_finish()[0]
        seq, ok, _t = self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        assert seq == before + 1 and ok is True

    def test_the_repeat_does_NOT_keep_completing(self):
        # ~2 Hz for as long as it is asked. Counting every one leaves a
        # completion permanently pending, and the NEXT move returns the
        # instant it starts waiting -- reporting a move that never happened.
        b, r, lg = _bridge()
        self._say(b, "[AMS_DEV] STEP:odom reset tray 0")
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        settled = b.last_finish()[0]
        for _ in range(20):
            self._say(b, "[AMS_IDLE]set ams state switch")
            self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        assert b.last_finish()[0] == settled

    def test_a_new_tray_re_arms_the_edge(self):
        # Load, unload, load, unload must give four completions, not two.
        b, r, lg = _bridge()
        start = b.last_finish()[0]
        self._say(b, "[AMS_DEV] STEP:odom reset tray 0")
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        self._say(b, "[AMS_DEV] STEP:odom reset tray 0")
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        assert b.last_finish()[0] == start + 4

    def test_the_interleaved_state_lines_do_not_re_arm_it(self):
        # The churn alternates with "set ams state switch"; if that re-armed
        # the latch we would be back to counting every repeat.
        b, r, lg = _bridge()
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        settled = b.last_finish()[0]
        self._say(b, "[AMS_IDLE]set ams state switch")
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        assert b.last_finish()[0] == settled

    def test_a_real_finish_line_also_re_arms_the_edge(self):
        # An HT-dialect completion means a tray is engaged again just as much
        # as an odom reset does.
        b, r, lg = _bridge()
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        settled = b.last_finish()[0]
        self._say(b, "[AMS_SWITCH]feed finish, buff_pos:1.28", addr=0x1800)
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        assert b.last_finish()[0] == settled + 2

    def test_the_ht_blob_is_still_judged_as_a_finish_not_an_odom_reset(self):
        # The HT emits odom reset INSIDE its finish blob. The finish rule must
        # win, or a stalled-short feed would be scored a success by the reset.
        b, r, lg = _bridge()
        seq, ok, _t = self._say(
            b, "[AMS_SWITCH]feed finish -1, stall, len_det:1.000 m, "
               "tube_len:3.619 m [AMS_RFID] STEP,odom reset tray 0",
            addr=0x1800)
        assert ok is False

    def test_ordinary_dev_narration_is_not_a_completion(self):
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        self._say(b, "[AMS_DEV] STEP2:feed tray 0 to switch")
        self._say(b, "[AMS_IDLE]set ams state assist, mode:4")
        self._say(b, "[AMS_DEV] STEP3:start,read all card")
        assert b.last_finish()[0] == before


class TestAms2ProVocabulary:
    """The AMS 2 Pro's own words, taken verbatim from docs/ams2_pro_protocol.md
    and the ams2_* captures. No AMS 2 Pro on the rig, so these captures ARE the
    verification -- which is why the lines are copied exactly rather than
    paraphrased."""

    def _say(self, b, text, addr=0x0700):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr}))
        return b.last_finish()

    def test_pull_sucess_completes_an_unload(self):
        # The unit does NOT say "finish" on the way out. Without this its
        # every unload runs the full watchdog.
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        seq, ok, _t = self._say(
            b, "[AMS_SWITCH]pull sucess,cond match,... bdc_i:0.464A;"
               "spd:-20.1cm/s")
        assert seq == before + 1 and ok is True

    def test_the_spaced_spelling_also_completes(self):
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        self._say(b, "[AMS_SWITCH]pull sucess, cond match")
        assert b.last_finish()[0] == before + 1

    def test_the_state_machine_sucessful_still_does_NOT_complete(self):
        # Shares the misspelling and occurs 242 times in one night's log.
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        for _ in range(5):
            self._say(b, "[AMS_SWITCH]AMS_CTRL_state_switch finish, "
                         "sucessful, err_code:0x80")
        assert b.last_finish()[0] == before

    def test_the_feed_completion_form_is_covered(self):
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        seq, ok, _t = self._say(
            b, "[AMS_SWITCH]feed finish 0, mode:4, dw_len:3.508 m, "
               "idx_set:3, idx_ref:3")
        assert seq == before + 1 and ok is True

    def test_e_in_does_NOT_complete_a_move(self):
        # Read as "extruder in" and treated as an arrival at first. Probably
        # wrong: in BOTH captures containing it, an err_code transition
        # follows within a second, and neither capture is of a healthy load.
        # Nothing on hardware has emitted it in a full day of cycles, which
        # fits an error that has not happened rather than an arrival that
        # should occur every load. Completing a move on what may be an error
        # report is the outcome worth ruling out while the meaning is unknown.
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        self._say(
            b, "[AMS_SWITCH]e_in tray:0,buff_pos:-0.34,i:0.566A,len:1.670m")
        assert b.last_finish()[0] == before

    def test_e_in_still_yields_its_buffer_reading(self):
        # The pattern is kept: the line reaches the log and its buff_pos is
        # still read. Only the completion effect is removed.
        b, r, lg = _bridge()
        self._say(
            b, "[AMS_SWITCH]e_in tray:0,buff_pos:-0.34,i:0.566A,len:1.670m")
        assert b.last_buff_pos() == pytest.approx(-0.34)

    def test_e_in_records_the_buffer_position(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]e_in tray:0,buff_pos:-0.34,i:0.566A,"
                     "len:1.670m")
        assert b.last_buff_pos() == pytest.approx(-0.34)

    def test_the_new_tube_len_form_is_read(self):
        # "new tube_len" here against the HT's "old tube_len".
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]new tube_len:3503 mm, list:3500,3507,0 mm, "
                     "err:7 mm")
        assert b.tube_len(0x0700) == pytest.approx(3503.0)


class TestBufferRefill:
    """BUFF,pos:A->B, det:Nmm is the ramming event as the unit measures it:
    how far the buffer sagged when the extruder pulled, and how much filament
    it fed to bring it back. Note the spelling differs from buff_pos:, so one
    pattern cannot cover both."""

    def _say(self, b, text):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text,
                                  "addr": 0x0700}))

    def test_nothing_reported_yet_is_none(self):
        b, r, lg = _bridge()
        assert b.last_buff_refill() is None
        assert b.last_buff_pos() is None

    def test_a_refill_records_sag_recovery_and_distance(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]BUFF,pos:0.09->0.74, det:6mm,  i:0.583A")
        assert b.last_buff_refill() == (pytest.approx(0.09),
                                        pytest.approx(0.74), 6.0)

    def test_the_recovered_position_becomes_the_current_one(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]BUFF,pos:0.10->0.74, det:28mm, i:0.521A")
        assert b.last_buff_pos() == pytest.approx(0.74)

    def test_the_unspaced_form_is_read(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]BUFF,pos:0.09->0.74,det:12mm")
        assert b.last_buff_refill()[2] == 12.0

    def test_a_refill_without_det_still_records_the_positions(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]BUFF,pos:0.10->0.74")
        sag, rec, det = b.last_buff_refill()
        assert (sag, rec) == (pytest.approx(0.10), pytest.approx(0.74))
        assert det is None

    def test_the_distance_varies_while_the_setpoint_does_not(self):
        # Every captured sample recovers to ~0.74 with det ranging 6..28 mm:
        # a unit refilling to a fixed setpoint on demand. Pinned because that
        # shape is what makes it usable for ramming.
        b, r, lg = _bridge()
        seen = []
        for line in ("BUFF,pos:0.09->0.74, det:6mm,  i:0.583A",
                     "BUFF,pos:0.10->0.73, det:24mm, i:0.740A",
                     "BUFF,pos:0.10->0.74, det:28mm, i:0.521A"):
            self._say(b, "[AMS_SWITCH]" + line)
            seen.append(b.last_buff_refill())
        assert [s[2] for s in seen] == [6.0, 24.0, 28.0]
        assert all(0.72 <= s[1] <= 0.76 for s in seen)

    def test_a_refill_is_not_mistaken_for_a_motion_completion(self):
        # It happens continuously during a print; counting it would report a
        # move finishing every time the extruder pulled.
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        for _ in range(10):
            self._say(b, "[AMS_SWITCH]BUFF,pos:0.10->0.74, det:28mm")
        assert b.last_finish()[0] == before


class TestDryRefusal:
    """An AMS refuses to dry with filament out in the hub, and it refuses AFTER
    echoing our parameters back:

        [AMS_LINK]ams0 dry,req ams 0
        [AMS_LINK]ret:1,mode:1,temp:55,time:480
        [AMS_CHMB]err, filament hub load!

    The echo is what proves the command was addressed correctly -- a frame sent
    to a unit id it does not own draws nothing at all. So this is the UNIT
    declining, not a delivery failure, and we report success either way. Left
    unread, a refused dry is indistinguishable from an accepted one."""

    def _say(self, b, text, addr=0x1800):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr}))

    def test_nothing_refused_yet_is_none(self):
        b, r, lg = _bridge()
        assert b.last_dry_error(0x1800) is None

    def test_the_refusal_is_recorded_in_the_units_own_words(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_LINK]ret:1,mode:1,temp:55,time:480 "
                     "[AMS_CHMB]err, filament hub load! "
                     "[AMS_CHMB]update dry_mode:1, ams_state:0")
        assert b.last_dry_error(0x1800) == "filament hub load!"

    def test_it_is_recorded_against_the_device_that_said_it(self):
        # Two units share the bus and the log is bus-wide; attributing a
        # refusal to the wrong card is worse than not showing it.
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!", addr=0x1800)
        assert b.last_dry_error(0x0700) is None
        assert b.last_dry_error(0x1800) == "filament hub load!"

    def test_heating_clears_it(self):
        # A stale reason must not outlive the condition.
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!")
        self._say(b, "[AMS_CHMB]set state CTC_STATE_HEATING")
        assert b.last_dry_error(0x1800) is None

    def test_a_self_check_clears_it_too(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!")
        self._say(b, "[AMS_CHMB]set state CTC_STATE_SELF_CHECK, from off, ref:55")
        assert b.last_dry_error(0x1800) is None

    def test_a_repeat_is_still_recorded(self):
        # The AMS repeats the refusal on every retry, and a deduped repeat
        # still means "still refusing" -- so this is read before the dedupe.
        b, r, lg = _bridge()
        for _ in range(3):
            self._say(b, "[AMS_CHMB]err, filament hub load!")
        assert b.last_dry_error(0x1800) == "filament hub load!"

    def test_it_is_said_in_english_on_the_console(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!")
        said = " ".join(m for _l, m in
                        [(x[0], x[1]) for x in lg.msgs]) if lg.msgs else ""
        assert "refused" in said.lower() or b.last_dry_error(0x1800)

    def test_an_unaddressed_line_is_ignored(self):
        b, r, lg = _bridge()
        b.handle_line(json.dumps({"evt": "amsdbg",
                                  "text": "[AMS_CHMB]err, filament hub load!"}))
        assert b.last_dry_error(0x1800) is None


class TestTrayNowIsNotACompletion:
    """tray_now:255 looked like the AMS 2 Pro's wording for "the tray has
    left". On one unload it tracked perfectly -- retract at 13:42:27,
    tray_now:255 from 13:42:43, 19 s before AFC gave up on its watchdog.

    It is not that. The same line appears while the unit is LOADED and
    FOLLOWING:

        [AMS_COMMON]state:4,tray_now:255,tray_exit:1
        [AMS_SWITCH]tray:0, bldc slip, dw_pos:-0.000 m

    Used as a completion it ends a move early on a unit merely sitting between
    trays, which is the failure the completion path exists to prevent."""

    def _say(self, b, text, addr=0x0700):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr}))

    def test_the_state_line_does_NOT_complete_a_retract(self):
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        self._say(b, "[AMS_COMMON]state:2,tray_now:255,tray_exit:1")
        assert b.last_finish()[0] == before

    def test_it_does_not_complete_while_following_either(self):
        # The verbatim line that disproved the reading.
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        self._say(b, "[AMS_COMMON]state:4,tray_now:255,tray_exit:1")
        self._say(b, "[AMS_SWITCH]tray:0, bldc slip, dw_pos:-0.000 m")
        assert b.last_finish()[0] == before

    def test_the_odometer_form_still_completes_one(self):
        # A boxed AMS's own marker is unaffected and still works.
        b, r, lg = _bridge()
        before = b.last_finish()[0]
        self._say(b, "[AMS_DEV] STEP:odom tray_id error 255")
        assert b.last_finish()[0] == before + 1

class TestTubeLenPerUnit:
    """An AMS 1 and an AMS 2 Pro both narrate as 0x0700, so the device address
    cannot say which measured a path. Live risk on hardware: AMS 2 measured
    3532 mm while AMS 1 was still on the 3000 mm default, and address-keyed
    lookup would have adopted one into the other's config."""

    def _say(self, b, text, addr=0x0700):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr}))

    def test_address_keying_still_works_with_no_active_unit(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        assert b.tube_len(0x0700) == pytest.approx(3532.0)

    def test_two_units_on_one_address_do_not_collide(self):
        b, r, lg = _bridge()
        b.set_active_unit(1)
        self._say(b, "[AMS_SWITCH]new tube_len:3000 mm, list:3000,3000,0 mm")
        b.set_active_unit(2)
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        assert b.tube_len(0x0700, unit=1) == pytest.approx(3000.0)
        assert b.tube_len(0x0700, unit=2) == pytest.approx(3532.0)

    def test_a_unit_that_never_measured_reads_nothing(self):
        # No falling back to the address: on a bus with two boxed units that
        # hands unit 1 whatever unit 2 last measured.
        b, r, lg = _bridge()
        b.set_active_unit(2)
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        assert b.tube_len(0x0700, unit=1) is None

    def test_clearing_the_active_unit_stops_attributing(self):
        b, r, lg = _bridge()
        b.set_active_unit(2)
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        b.set_active_unit(None)
        self._say(b, "[AMS_SWITCH]new tube_len:9999 mm, list:9999,9999,0 mm")
        assert b.tube_len(0x0700, unit=2) == pytest.approx(3532.0)
        assert b.tube_len(0x0700) == pytest.approx(9999.0)

    def test_an_uncalibrated_zero_is_still_dropped(self):
        b, r, lg = _bridge()
        b.set_active_unit(2)
        self._say(b, "[AMS_SWITCH]old tube_len:0 mm, list:3534,0,0 mm")
        assert b.tube_len(0x0700, unit=2) is None


class TestTubeLenDoesNotLeakBetweenUnits:
    """Two units of the same class share a device address, so the address map
    holds whichever measured last. Falling back to it for a unit that has not
    measured hands one unit the other's path.

    Observed on hardware with both boxed units present: AMS 2 measured 3532 mm
    and AMS 1, which had never measured, read 3532 mm through the address --
    and would have adopted it as its own bowden length on its next load."""

    def _say(self, b, text, addr=0x0700):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr}))

    def test_a_unit_that_never_measured_reads_nothing(self):
        b, r, lg = _bridge()
        b.set_active_unit(2)
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        assert b.tube_len(0x0700, unit=2) == pytest.approx(3532.0)
        assert b.tube_len(0x0700, unit=1) is None      # NOT 3532

    def test_the_address_fallback_still_works_before_any_attribution(self):
        # Single-unit bus, or anything that never set an active unit.
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        assert b.tube_len(0x0700, unit=1) == pytest.approx(3532.0)
        assert b.tube_len(0x0700) == pytest.approx(3532.0)

    def test_each_unit_keeps_its_own_once_both_have_measured(self):
        b, r, lg = _bridge()
        b.set_active_unit(1)
        self._say(b, "[AMS_SWITCH]new tube_len:2900 mm, list:2900,2900,0 mm")
        b.set_active_unit(2)
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        assert b.tube_len(0x0700, unit=1) == pytest.approx(2900.0)
        assert b.tube_len(0x0700, unit=2) == pytest.approx(3532.0)

    def test_an_unmeasured_unit_keeps_its_configured_value(self):
        # None is the correct answer: _adopt_measured_path leaves the
        # configured length alone rather than adopting a neighbour's.
        b, r, lg = _bridge()
        b.set_active_unit(2)
        self._say(b, "[AMS_SWITCH]new tube_len:3532 mm, list:3534,3531,0 mm")
        assert b.tube_len(0x1800, unit=0) is None


class TestNarrationLogPerMaster:
    """One narration log per BUS MASTER. Two Picos writing one file cannot be
    untangled afterwards: the only per-line attribution is the device address,
    and two boxed units on different buses both narrate as 0x0700."""

    def _clear(self, *names):
        for n in names:
            lg = logging.getLogger(n)
            for h in list(lg.handlers):
                h.close()
                lg.removeHandler(h)

    def setup_method(self):
        self._clear("AFC_BambuAMS_file", "AFC_BambuAMS_file_ttyACM1")

    teardown_method = setup_method

    def test_no_tag_keeps_the_original_filename(self):
        # A single-Pico printer must be completely unchanged.
        b, r, lg = _bridge()
        assert b.set_narration_log(str(pytest.importorskip("tempfile") and
                                        __import__("tempfile").mkdtemp())) is True

    def test_a_tagged_master_writes_its_own_file(self, tmp_path):
        b, r, lg = _bridge()
        b.set_narration_log(str(tmp_path), "ttyACM1")
        b.handle_line(json.dumps({"evt": "amsdbg", "addr": 0x0700,
                                  "text": "[AMS_DEV] STEP:odom reset tray 0"}))
        for h in logging.getLogger("AFC_BambuAMS_file_ttyACM1").handlers:
            h.flush()
        assert (tmp_path / "AFC_BambuAMS_ttyACM1.log").exists()
        assert not (tmp_path / "AFC_BambuAMS.log").exists()

    def test_two_masters_do_not_share_a_file(self, tmp_path):
        a, _r, _l = _bridge()
        b, _r2, _l2 = _bridge()
        a.set_narration_log(str(tmp_path))
        b.set_narration_log(str(tmp_path), "ttyACM1")
        a.handle_line(json.dumps({"evt": "amsdbg", "addr": 0x0700,
                                  "text": "bus one speaking"}))
        b.handle_line(json.dumps({"evt": "amsdbg", "addr": 0x0700,
                                  "text": "bus two speaking"}))
        for n in ("AFC_BambuAMS_file", "AFC_BambuAMS_file_ttyACM1"):
            for h in logging.getLogger(n).handlers:
                h.flush()
        one = (tmp_path / "AFC_BambuAMS.log").read_text()
        two = (tmp_path / "AFC_BambuAMS_ttyACM1.log").read_text()
        assert "bus one speaking" in one and "bus two speaking" not in one
        assert "bus two speaking" in two and "bus one speaking" not in two


# ── capacity narration: three units, four forms ─────────────────────────────
# Captured 2026-08-05 with each unit ALONE on the wire, so every string below
# is provably that unit's. The units disagree about the prefix, the separator,
# the spacing AND the spelling, and a pattern anchored on any one of them
# silently matches nothing on the other two -- which is exactly how the
# "measured weight never updates" bug survived: the old pattern required both
# a "C:" and a "%", so it saw only the live HT form and ignored every restore
# and the whole AMS 2 dialect.
CAP_LINES = [
    # (label, narration, tray, circumference, radius, percent, restored)
    ("HT live",
     "[AMS_RFID] STEP4,odom C:0.531,R:0.084,P:107%,od:1.132",
     None, 0.531, 0.084, 107, False),
    ("AMS 1 live -- [AMS_DEV] prefix, spaces after every comma",
     "[AMS_DEV] STEP:odom C:0.480, R:0.076, P:78%, od:0.988",
     None, 0.480, 0.076, 78, False),
    ("AMS 2 restore -- no C:, no % sign",
     "[AMS_RFID]STEP:odom load from flash 2,R:0.072,P:65",
     2, None, 0.072, 65, True),
    ("HT restore",
     "[AMS_RFID] STEP:odom load from flash 0,R:0.088,P:119",
     0, None, 0.088, 119, True),
]


@pytest.mark.parametrize(
    "label,text,tray,circ,radius,pct,restored", CAP_LINES,
    ids=[c[0] for c in CAP_LINES])
def test_capacity_narration_parses_every_dialect(
        label, text, tray, circ, radius, pct, restored):
    m = br._CAP_MEASURE_RE.search(text)
    assert m is not None, f"{label}: no match -- {text}"
    g_tray, g_circ, g_radius, g_pct = m.groups()
    assert (int(g_tray) if g_tray is not None else None) == tray
    assert (float(g_circ) if g_circ else None) == circ
    assert float(g_radius) == radius
    assert int(g_pct) == pct
    assert (g_tray is not None) is restored


def test_capacity_circumference_agrees_with_radius():
    """C = 2*pi*R on every live measurement.

    This is the check that the field mapping is real rather than three numbers
    that happen to line up: the unit computes both, so they must agree. It
    caught nothing when written -- which is the point. If someone reorders the
    groups, this fails where an eyeball would not.
    """
    import math
    for label, text, _tray, circ, radius, _pct, _r in CAP_LINES:
        if circ is None:
            continue                      # restore form states no circumference
        assert abs(circ - 2 * math.pi * radius) < 0.005, label


def test_calibration_done_matches_both_spellings():
    """The HT misspells it. One 's'."""
    assert br._CALI_DONE_RE.search("[AMS_RFID] STEP4,odom calib sucess")
    m = br._CALI_DONE_RE.search(
        "[AMS_DEV] STEP:odom calib success exit 0,dis:0.989")
    assert m and m.group(1) == "0"


def test_capacity_pattern_ignores_unrelated_odom_chatter():
    """Must not fire on the odometer lines that surround a real measurement."""
    for noise in ("[AMS_DEV] STEP:odom search, odo 1.856",
                  "[AMS_DEV] STEP:odom reset tray 0",
                  "[AMS_RFID] STEP,odom load tray 3 info invailed",
                  "[AMS_RFID] STEP,odom save R nan, exit"):
        assert br._CAP_MEASURE_RE.search(noise) is None, noise


# ── dialect tolerance: the three units do not share a vocabulary ────────────
# Counted over 2026-08-05 single-unit captures, where every fragment is
# provably one unit's:
#
#   HT     [AMS_SWITCH] [AMS_COMMON] [AMS_LINK] [AMS_LED] [AMS_TRAY] [AMS_CHMB]
#   AMS 2  the same, plus [AMS_RFID] [AMS_PMSM]
#   AMS 1  [AMS_DEV] almost exclusively (63 of 64 fragments), plus [AMS_CALL]
#
# So a rule anchored on any one bracket tag is structurally blind to at least
# one unit. These are REAL lines, copied verbatim from the captures.
DIALECT_LINES = [
    # (event, HT form, AMS 2 form, AMS 1 form)
    # The AUTHENTICATION, not "read success": the latter fires mid-cycle on
    # an HT attempt that then fails and retries, so it is not the event.
    ("tag authenticated",
     "[AMS_RFID] STEP3,auth card successful",
     "[AMS_RFID]STEP:card auth success!",
     "[AMS_DEV] STEP:card auth success!"),
    ("calibration done",
     "[AMS_RFID] STEP4,odom calib sucess",          # HT misspells it
     "[AMS_RFID]STEP:odom calib success",
     "[AMS_DEV] STEP:odom calib success exit 0,dis:0.989"),
]


@pytest.mark.parametrize("event,ht,ams2,ams1",
                         DIALECT_LINES, ids=[c[0] for c in DIALECT_LINES])
def test_a_rule_fires_on_every_dialect(event, ht, ams2, ams1):
    """Whatever matches one unit must match all three."""
    for label, line in (("HT", ht), ("AMS 2", ams2), ("AMS 1", ams1)):
        hit = any(rx.search(line) for rx, _ in br._AMS_HUMAN)
        assert hit, f"{event}: no rule matched the {label} form -- {line}"


def test_step_helper_tolerates_every_punctuation_seen():
    """STEP4, / STEP: / STEP2: with and without a space after the bracket."""
    rx = br._STEP("read success")
    for line in ("[AMS_RFID] STEP4,read success",
                 "[AMS_RFID]STEP:read success",
                 "[AMS_DEV] STEP:read success",
                 "[AMS_DEV]STEP2: read success",
                 "[AMS_SWITCH] STEP : read success".replace(" :", ":")):
        assert rx.search(line), line


def test_step_helper_does_not_match_a_different_event():
    rx = br._STEP("read success")
    assert not rx.search("[AMS_DEV] STEP:odom search, odo 1.856")
    assert not rx.search("[AMS_RFID] STEP3,search 1 card")


def test_the_load_time_search_lines_are_NOT_a_measurement():
    """The radius search running mid-load looks like a measurement and is not.

        [AMS_DEV] STEP:odom r:0, dt0.442, R:0.073, P:70%, od:0.741
        [AMS_DEV] STEP:odom r:1, dt0.887, R:0.071, P:65%

    A dedicated calibration of THE SAME SPOOL minutes earlier said 73%. Across
    two loads the estimates converge on neither that nor each other: one ran
    26% -> 54% (up 28 points), the other 70% -> 65% (down 5). Adopting "the
    last one of the load" was built, tested against a shimmed accessor, and
    would have written a figure 8 points under the calibrated one on every
    toolchange.

    The real measurement carries a CIRCUMFERENCE and comes from the calibration
    cycle, which samples ~2 spool revolutions. The pattern requires R: to
    follow `odom` closely, and that is what excludes these -- load-bearing, not
    incidental, so this test exists to stop it being "fixed".
    """
    for line in (
            "[AMS_DEV] STEP:odom r:0, dt0.442, R:0.073, P:70%, od:0.741",
            "[AMS_DEV] STEP:odom r:1, dt0.887, R:0.071, P:65%",
            "[AMS_DEV] STEP:odom r:1, dt0.895, R:0.077, P:54%"):
        assert br._CAP_MEASURE_RE.search(line) is None, line


def test_the_calibrated_line_from_the_same_session_does_parse():
    """...and the one that IS a measurement still does, so the exclusion above
    is not simply a broken pattern."""
    m = br._CAP_MEASURE_RE.search(
        "[AMS_DEV] STEP,second detected [AMS_DEV] STEP:odom "
        "C:0.469,R:0.075,P:73%, od:0.724")
    assert m is not None
    assert float(m.group(2)) == 0.469 and int(m.group(4)) == 73


def test_capacity_line_parses_on_all_three_units():
    """The measurement itself, in each unit's own punctuation."""
    for label, line, pct in (
            ("HT",    "[AMS_RFID] STEP4,odom C:0.531,R:0.084,P:107%,od:1.132", 107),
            ("AMS 1", "[AMS_DEV] STEP:odom C:0.480, R:0.076, P:78%, od:0.988", 78),
            ("AMS 2", "[AMS_RFID]STEP:odom load from flash 2,R:0.072,P:65",    65)):
        m = br._CAP_MEASURE_RE.search(line)
        assert m, f"{label}: {line}"
        assert int(m.group(4)) == pct, label


class TestAms1GivesUpInStateNotWords:
    """Three dialects, three ways of saying "I gave up":

        AMS 2 Pro   "feed finish -1, stall", "pull err, bdc stall"
        AMS HT      "TIMEOUT error N"
        AMS 1       none of those -- state:6 / en:0,mode:7,idx:255

    The AMS 1 was long recorded as silent about faults. It is not; it answers
    in STATE rather than words, so a word-matching detector walked past it and
    a jammed AMS 1 rode out the entire load window."""

    def _say(self, b, text, addr=0x0700):
        b.handle_line(json.dumps(
            {"evt": "amsdbg", "text": text, "addr": addr}))
        return b.last_fault()[0]

    def test_ams1_state_6_is_a_fault(self):
        b, r, lg = _bridge()
        before = b.last_fault()[0]
        self._say(b, "[AMS_COMMON]state:6,tray_now:255,tray_exit:6")
        assert b.last_fault()[0] != before

    def test_ams1_en0_mode7_is_a_fault(self):
        b, r, lg = _bridge()
        before = b.last_fault()[0]
        self._say(b, "[AMS_LINK]en:0,mode:7,idx:255,ref:0")
        assert b.last_fault()[0] != before

    def test_the_states_of_a_healthy_load_are_not(self):
        # Counted across a lane15 load that genuinely reached the toolhead:
        # state:4 and state:0 only, and ZERO of the two above.
        b, r, lg = _bridge()
        before = b.last_fault()[0]
        self._say(b, "[AMS_COMMON]state:4,tray_now:255,tray_exit:6")
        self._say(b, "[AMS_COMMON]state:0,tray_now:255,tray_exit:6")
        self._say(b, "[AMS_DEV] STEP:odom search, odo 0.516")
        assert b.last_fault()[0] == before

    def test_the_other_two_dialects_still_fire(self):
        for text in ("[AMS_SWITCH]feed finish -1, stall, len_det:1.0 m",
                     "[AMS_LED]TIMEOUT error 2"):
            b, r, lg = _bridge()
            before = b.last_fault()[0]
            self._say(b, text)
            assert b.last_fault()[0] != before, text
