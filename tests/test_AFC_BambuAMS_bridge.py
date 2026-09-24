# Tests for extras/AFC_BambuAMS_bridge.py, the serial transport.
#
# This module imports only the standard library, so it can be driven end to end
# with a fake serial port and no printer. That matters because the reader
# thread, the reconnect path and the event dispatch are where a fault is
# invisible, the bridge keeps reporting the last thing it knew.
from __future__ import annotations

import json
import logging
import logging.handlers
import threading
import time
import types

import pytest

import hashlib
import hmac
import socket
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

    def test_sniff_mode_ack_is_surfaced(self):
        # Console, not file: this one says whether the bridge is driving the
        # bus at all, and a listen-only bridge still answers status polls out
        # of its last-known state -- so nothing else distinguishes it from
        # units that have simply gone quiet.
        b, r, lg = _bridge()
        _feed(b, '{"evt":"sniff_mode","on":true}')
        assert any("sniff mode ON" in m for m in lg.texts("info"))
        _feed(b, '{"evt":"sniff_mode","on":false}')
        assert any("sniff mode OFF" in m for m in lg.texts("info"))

    def test_error_event_warns(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"error","msg":"bus down"}')
        assert any("bus down" in m for m in lg.texts("warning"))

    def test_ack_is_logged(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"ack","cmd":"dry","slot":55}')
        assert any("bridge ack dry (slot 55)" in m for m in lg.texts("debug"))

    def test_a_slow_pass_is_logged_to_file_only(self):
        b, r, lg = _bridge()
        _feed(b, '{"evt":"slow","ms":1460,"rr_ms":1402,"rr_n":31,'
                 '"rr_max_us":32000,"capped":30,"capscan":0}')
        line = ("AFC bambu: bridge loop held 1460 ms (bus reads 1402 ms over "
                "31, longest 32000 us, capped 30, capscans 0)")
        assert line in lg.texts("debug") and line in lg.file_only

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

    def test_capmask_is_the_measure_readback(self):
        # measure_on_insert is pushed with `capen`, whose ack names only the
        # unit -- so this is the only way to see the value the firmware kept.
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"AA","capmask":18}')
        assert b._chain_capmask == 18

    def test_no_capmask_is_unknown_not_zero(self):
        # Firmware predating the field cannot answer. "No unit measures" and
        # "this build cannot tell you" must not look the same.
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"AA","htmask":5}')
        assert b._chain_capmask is None

    def test_a_malformed_capmask_reads_unknown(self):
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"AA","capmask":"x"}')
        assert b._chain_capmask is None

    def test_an_all_off_capmask_is_a_real_answer(self):
        # Zero is the new default, so it has to be distinguishable from absent.
        b, _, _ = _bridge()
        _feed(b, '{"evt":"chain","uids":"AA","capmask":0}')
        assert b._chain_capmask == 0


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

    NO_TRAY = "[AMS_RFID]STEP3,odom tray_id error 0"

    def test_the_no_tray_refusal_stays_off_the_console(self):
        # The unit drops its tray selection whenever it finishes with a bay,
        # so a CLEAN unload ends with it refusing the next thing sent. On the
        # console that reads as a failure about an operation that just worked.
        b, r, lg = self._b()
        b._narrate_human(self.NO_TRAY, 100.0)
        assert lg.texts("info") == []

    def test_the_no_tray_refusal_is_still_written_to_the_log(self):
        b, r, lg = self._b()
        b._narrate_human(self.NO_TRAY, 100.0)
        assert any("NO TRAY SELECTED" in m for m in lg.texts("debug"))

    def test_a_log_only_line_does_not_spend_the_console_budget(self):
        # The rate limit and the dedupe are console bookkeeping. A line that
        # never reaches the console must not consume either, or a refusal
        # would silence the real message that follows it.
        b, r, lg = self._b()
        b._narrate_human(self.NO_TRAY, 100.0)
        b._narrate_human("[AMS_DEV] STEP:card auth success!", 100.2)
        assert any("tag authenticated" in m for m in lg.texts("info"))

    def test_a_shell_open_err_is_the_lid_and_not_a_refusal(self):
        # Watched live: this err, line landed six seconds into a cycle that
        # kept heating at full power. The generic refusal rule sent the
        # operator to unload a lane that had nothing to do with it.
        b, r, lg = self._b()
        b._narrate_human("j [AMS_CHMB]finish! [AMS_CHMB]set state "
                         "CTC_STATE_HEATING, from selfcheck [AMS_CHMB]err, "
                         "ams-ht shell open!", 100.0)
        assert lg.texts("info") == [
            "AFC bambu BambuAMS_1: AMS HT lid is open -- drying continues, but "
            "the chamber cannot hold temperature until the shell is closed."]

    def test_other_err_lines_still_render_as_refusals(self):
        b, r, lg = self._b()
        b._narrate_human("[AMS_CHMB]err, filament hub load!", 100.0)
        assert lg.texts("info") == [
            "AFC bambu BambuAMS_1: AMS refused the drying command: filament hub "
            "load!. An AMS will not dry with filament out in the hub -- reel "
            "the lane back to its bay first (LANE_UNLOAD)."]

    def test_chamber_telemetry_updates_the_units_record(self):
        b, r, lg = self._b()
        b._narrate_human(
            "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:22.0", 100.0, 0x1800, 2)
        assert b._chmb_by_unit[2]["temp"] == 23.1
        assert b._chmb_by_unit[2]["target"] == 55.0
        assert b._chmb_by_unit[2]["state"] == 2
        assert b._chmb_by_unit[2]["seen"] == 100.0

    def test_humidity_comes_from_ht_not_the_suffix_on_vt(self):
        # A real AMS HT line. The `,00` after vt is not humidity -- it never
        # moves off 00 -- and reading it there is why every drying line said
        # "humidity 00%" while ht carried the real figure.
        b, r, lg = self._b()
        b._narrate_human(
            "[AMS_CHMB]s:2|rf:55|vt:22.4,00|ap:22.3|ht:60,22|pw:000|ad:2|t:7",
            100.0, 0x1800, 2)
        assert b._chmb_by_unit[2]["humidity"] == 60
        assert b._chmb_by_unit[2]["temp"] == 22.4

    def test_humidity_tracks_the_chamber_over_a_cycle(self):
        # Correlated against a whole real cycle: ht's first value falls as the
        # chamber heats, which is what relative humidity does when air warms.
        # If this ever reads flat, it is back on the wrong field.
        b, r, lg = self._b()
        seen = []
        for t, vt, ht in ((7, 22.4, 60), (67, 33.2, 55),
                          (127, 55.6, 40), (178, 52.7, 31)):
            b._narrate_human(
                f"[AMS_CHMB]s:2|rf:55|vt:{vt},00|ap:30.0|ht:{ht},22|t:{t}",
                100.0 + t, 0x1800, 2)
            seen.append(b._chmb_by_unit[2]["humidity"])
        assert seen == [60, 55, 40, 31]

    def test_the_hts_spelling_is_read_too(self):
        # Same field under a firmware that appends a third value.
        b, r, lg = self._b()
        b._narrate_human(
            "[AMS_CHMB]s:2, rf:55, cd:55, vt:23.1, ap:23.0, hts:46,23,0 pw:100",
            100.0, 0x1800, 2)
        assert b._chmb_by_unit[2]["humidity"] == 46
        assert b._chmb_by_unit[2]["temp"] == 23.1

    def test_a_line_without_humidity_records_none(self):
        # Not every model reports it; absent must stay absent rather than 0,
        # which would read as "bone dry".
        b, r, lg = self._b()
        b._narrate_human(
            "[AMS_CHMB]s:2|rf:55,0|vt:44.0|ap:35.3|pw:100|ad:2", 100.0,
            0x1800, 2)
        assert "humidity" not in b._chmb_by_unit[2]
        assert b._chmb_by_unit[2]["temp"] == 44.0

    def test_unparseable_chamber_numbers_leave_the_record_alone(self):
        b, r, lg = self._b()
        b._narrate_human("[AMS_CHMB]s:x, rf:y|vt:z", 100.0, 0x1800, 2)
        assert b._chmb_by_unit == {}


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
    """The AMS narrates continuously -- every STEP, finish, stall and measured
    length -- and that record is what you want when something goes wrong.

    It goes to a dedicated file of its own rather than through logger.debug(),
    which AFC's `debug` flag gates off. Debug is off on a working printer,
    which is precisely when nobody is in a position to turn it on."""

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

    def test_rotation_defaults_to_10mb_with_one_backup(self, tmp_path):
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path))
        h = [x for x in logging.getLogger("AFC_BambuAMS_file").handlers
             if isinstance(x, logging.handlers.RotatingFileHandler)][0]
        assert h.maxBytes == 10 * 1024 * 1024
        assert h.backupCount == 1

    def test_it_rotates_and_keeps_exactly_one_backup(self, tmp_path):
        # backupCount=1: at maxBytes the live file rolls to .log.1 and a fresh
        # live file starts, so the previous chunk survives a rollover -- but
        # disk use stays bounded (live + one backup), never an archive that
        # fills the card.
        b, _r, _l = _bridge()
        b.set_narration_log(str(tmp_path), max_bytes=200)
        for i in range(60):
            self._feed(b, "[AMS_SWITCH]line %d padding padding padding" % i)
        for h in logging.getLogger("AFC_BambuAMS_file").handlers:
            h.flush()
        assert (tmp_path / "AFC_BambuAMS.log").stat().st_size <= 400
        backups = list(tmp_path.glob("AFC_BambuAMS.log.*"))
        assert backups == [tmp_path / "AFC_BambuAMS.log.1"]
        assert (tmp_path / "AFC_BambuAMS.log.1").stat().st_size <= 400

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

    def _say(self, b, text, addr=0x1800, unit=2):
        b.handle_line(json.dumps({"evt": "amsdbg", "text": text, "addr": addr,
                                  "unit": unit}))

    def test_nothing_refused_yet_is_none(self):
        b, r, lg = _bridge()
        assert b.last_dry_error(2) is None

    def test_the_refusal_is_recorded_in_the_units_own_words(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_LINK]ret:1,mode:1,temp:55,time:480 "
                     "[AMS_CHMB]err, filament hub load! "
                     "[AMS_CHMB]update dry_mode:1, ams_state:0")
        assert b.last_dry_error(2) == "filament hub load!"

    def test_it_is_recorded_against_the_unit_that_said_it(self):
        # The log is bus-wide and the address only names the unit CLASS;
        # attributing a refusal to the wrong card is worse than not showing
        # it, which is why the record is keyed by the chain index the
        # firmware stamps.
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!", unit=2)
        assert b.last_dry_error(0) is None
        assert b.last_dry_error(2) == "filament hub load!"

    def test_heating_clears_it(self):
        # A stale reason must not outlive the condition.
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!")
        self._say(b, "[AMS_CHMB]set state CTC_STATE_HEATING")
        assert b.last_dry_error(2) is None

    def test_a_self_check_clears_it_too(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!")
        self._say(b, "[AMS_CHMB]set state CTC_STATE_SELF_CHECK, from off, ref:55")
        assert b.last_dry_error(2) is None

    def test_the_lid_closing_clears_it(self):
        # The HT narrates both edges: "err, ams-ht shell open!" when the lid
        # opens and "ams-ht shell ok!" when it closes. Without the second, the
        # note outlived the open lid by the rest of the cycle.
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, ams-ht shell open!")
        self._say(b, "[AMS_CHMB]ams-ht shell ok!")
        assert b.last_dry_error(2) is None

    def test_an_accepted_start_clears_it(self):
        # "dry_mode:1, check ok!" is how BOTH dialects announce an accepted
        # start; the CTC_STATE strings are boxed-only. The HT narrates its
        # shell-open warning through the same err, line, and without this a
        # running HT cycle sat on "Refused".
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, ams-ht shell open!")
        self._say(b, "[AMS_CHMB]dry_mode:1, check ok!")
        assert b.last_dry_error(2) is None

    def test_a_repeat_is_still_recorded(self):
        # The AMS repeats the refusal on every retry, and a deduped repeat
        # still means "still refusing" -- so this is read before the dedupe.
        b, r, lg = _bridge()
        for _ in range(3):
            self._say(b, "[AMS_CHMB]err, filament hub load!")
        assert b.last_dry_error(2) == "filament hub load!"

    def test_it_is_said_in_english_on_the_console(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_CHMB]err, filament hub load!")
        said = " ".join(m for _l, m in
                        [(x[0], x[1]) for x in lg.msgs]) if lg.msgs else ""
        assert "refused" in said.lower() or b.last_dry_error(2)

    def test_an_unattributed_line_is_ignored(self):
        # unit -1 (or absent) is the firmware saying it could not attribute
        # the line; guessing an owner would put one unit's refusal on
        # another's card.
        b, r, lg = _bridge()
        b.handle_line(json.dumps({"evt": "amsdbg", "addr": 0x1800,
                                  "text": "[AMS_CHMB]err, filament hub load!"}))
        assert b.last_dry_error(2) is None


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
# the spacing AND the spelling, so a pattern anchored on any one of them
# matches nothing on the other two. All four forms are covered here: a live
# measurement from each generation, and the restore an AMS 2 reads back from
# flash, which carries neither a "C:" nor a "%" sign.
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


class TestTheUnitAnnouncingItHasStoppedRetrying:
    """"AMS_CTRL_state_switch finish, fail" is the unit saying its own retry
    budget is spent -- the one case where asking again cannot help.

    It must not be confused with a stall, which the printer feeds straight
    through on purpose, nor with the success form the same unit emits.
    """

    def test_the_fail_form_is_recognised(self):
        assert br._AMS_GAVE_UP_RE.search(
            "[AMS_SWITCH]AMS_CTRL_state_switch finish, fail, retry:5, "
            "feed_ret:0, err_code:0x12")

    def test_the_success_form_is_not(self):
        # Same unit, same sentence shape, opposite meaning -- and Bambu's own
        # spelling of "sucessful".
        assert not br._AMS_GAVE_UP_RE.search(
            "[AMS_SWITCH]AMS_CTRL_state_switch finish, sucessful, "
            "err_code:0x00")

    def test_ordinary_stall_chatter_is_not(self):
        for benign in (
            "[AMS_SWITCH]tray:2, bldc slip, dw_pos:-0.037 m",
            "[AMS_LINK]err_code:0x00->0x16",
            "[AMS_SWITCH]switch_feed rocker stall, tray_cnt:0,0,",
            "[AMS_SWITCH]feed finish -1, stall, len_det:3.711 m",
            "[AMS_RFID]STEP:odom calib success exit 0,dis:0.688",
        ):
            assert not br._AMS_GAVE_UP_RE.search(benign), benign

    def test_it_is_stamped_per_device_and_answered_by_time(self):
        b, r, lg = _bridge()
        addr = 0x0700
        assert not b.gave_up_since(r.monotonic(), addr=addr)
        _feed(b, '{"evt":"amsdbg","addr":%d,"text":"[AMS_SWITCH]'
                 'AMS_CTRL_state_switch finish, fail, retry:5"}' % addr)
        assert b.gave_up_since(0.0, addr=addr), "the give-up was not recorded"
        # A feed attempt that STARTED after the announcement must not inherit
        # it -- that is what keeps the re-home retry from aborting instantly.
        assert not b.gave_up_since(r.monotonic() + 1.0, addr=addr)


# ── the latch and bookkeeping methods ────────────────────────────────────────
# Exercised only as side effects of their callers until now, which is the gap
# the one-test-class-per-method rule closes: a latch can carry 100% branch
# coverage from a caller's test while never being independently verified.

def _lb():
    """A bridge for latch tests: real __init__, no serial attached."""
    b, _r, _lg = _bridge()
    return b


class _NarLog:
    """Stands in for the narration file logger; records the formatted line."""

    def __init__(self):
        self.lines = []

    def debug(self, fmt, *args):
        self.lines.append(fmt % args if args else fmt)


# ── try_claim_bus ────────────────────────────────────────────────────────────

class TestTryClaimBus:
    def test_first_claim_wins_and_records_owner(self):
        b = _lb()
        assert b.try_claim_bus("BambuAMS_1", 10.0) is True
        assert b._bus_owner == "BambuAMS_1"
        assert b._bus_claim_t == 10.0

    def test_a_second_owner_is_refused_while_busy(self):
        b = _lb()
        b.try_claim_bus("BambuAMS_1", 10.0)
        assert b.try_claim_bus("BambuAMS_2", 20.0) is False
        assert b._bus_owner == "BambuAMS_1"

    def test_reclaim_by_the_same_owner_refreshes_the_stamp(self):
        b = _lb()
        b.try_claim_bus("BambuAMS_1", 10.0)
        assert b.try_claim_bus("BambuAMS_1", 30.0) is True
        assert b._bus_claim_t == 30.0

    def test_a_cycle_end_after_the_claim_releases_it(self):
        """The unit's own end marker frees the bus, not a timer."""
        b = _lb()
        b.try_claim_bus("BambuAMS_1", 10.0)
        b._rfid_end_t = 40.0
        assert b.try_claim_bus("BambuAMS_2", 41.0) is True
        assert b._bus_owner == "BambuAMS_2"

    def test_a_cycle_end_before_the_claim_does_not_release_it(self):
        """A stale end marker from an earlier scan must not free a live claim."""
        b = _lb()
        b._rfid_end_t = 5.0
        b.try_claim_bus("BambuAMS_1", 10.0)
        assert b.try_claim_bus("BambuAMS_2", 11.0) is False

    def test_the_backstop_expires_an_unannounced_claim(self):
        b = _lb()
        b.try_claim_bus("BambuAMS_1", 10.0)
        late = 10.0 + BambuBridge.BUS_CLAIM_MAX_S + 1.0
        assert b.try_claim_bus("BambuAMS_2", late) is True


# ── release_bus ──────────────────────────────────────────────────────────────

class TestReleaseBus:
    def test_holder_releases(self):
        b = _lb()
        b.try_claim_bus("BambuAMS_1", 10.0)
        b.release_bus("BambuAMS_1")
        assert b._bus_owner is None

    def test_non_holder_cannot_release(self):
        b = _lb()
        b.try_claim_bus("BambuAMS_1", 10.0)
        b.release_bus("BambuAMS_2")
        assert b._bus_owner == "BambuAMS_1"

    def test_release_with_no_claim_is_safe(self):
        b = _lb()
        b.release_bus("BambuAMS_1")          # no raise
        assert getattr(b, "_bus_owner", None) is None


# ── bus_owner ────────────────────────────────────────────────────────────────

class TestBusOwner:
    def test_none_before_any_claim(self):
        assert _lb().bus_owner() is None

    def test_reports_the_holder(self):
        b = _lb()
        b.try_claim_bus("BambuAMS_1", 10.0)
        assert b.bus_owner() == "BambuAMS_1"


# ── last_err_code ────────────────────────────────────────────────────────────

class TestLastErrCode:
    def test_never_reported_is_none(self):
        """None means never-heard, which is not the same as healthy."""
        assert _lb().last_err_code() == (None, 0.0)

    def test_reports_the_level_and_its_time(self):
        b = _lb()
        b._err_code, b._err_code_t = 0x22, 55.5
        assert b.last_err_code() == (0x22, 55.5)


# ── _narrate_to_file ─────────────────────────────────────────────────────────

class TestNarrateToFile:
    def test_writes_the_line_with_its_address_and_unit(self):
        b = _lb()
        b._nar_lg = _NarLog()
        b._narrate_to_file("[AMS_PMSM]mode:0->2", 0x0700, 1)
        assert b._nar_lg.lines == ["0x0700 u1 [AMS_PMSM]mode:0->2"]

    def test_two_units_on_one_address_are_told_apart(self):
        # The whole point: both boxed AMSs answer at 0x0700, so the address
        # alone made every line from either of them look the same in the file
        # -- and a stretch of [AMS_DEV] narration got read as the AMS 1's when
        # the AMS 2 Pro beside it was the only unit narrating at all.
        b = _lb()
        b._nar_lg = _NarLog()
        b._narrate_to_file("[AMS_DEV] STEP:rfid pull 1", 0x0700, 0)
        b._narrate_to_file("[AMS_DEV] STEP:rfid pull 1", 0x0700, 1)
        assert b._nar_lg.lines == [
            "0x0700 u0 [AMS_DEV] STEP:rfid pull 1",
            "0x0700 u1 [AMS_DEV] STEP:rfid pull 1",
        ]

    def test_an_unattributed_line_says_so_rather_than_guessing(self):
        # The firmware refuses to attribute across a class mismatch, and an
        # older build sends no unit at all. Either way the line is still worth
        # keeping -- marked unknown, never silently pinned on a unit.
        b = _lb()
        b._nar_lg = _NarLog()
        b._narrate_to_file("[AMS_PMSM]mode:0->2", 0x0700, None)
        assert b._nar_lg.lines == ["0x0700 u? [AMS_PMSM]mode:0->2"]

    def test_an_unknown_address_is_dashes(self):
        b = _lb()
        b._nar_lg = _NarLog()
        b._narrate_to_file("hello", None)
        assert b._nar_lg.lines == ["0x---- u? hello"]

    def test_no_log_configured_is_a_noop(self):
        b = _lb()
        b._nar_lg = None
        b._narrate_to_file("hello", 0x0700)      # no raise

    def test_empty_text_writes_nothing(self):
        b = _lb()
        b._nar_lg = _NarLog()
        b._narrate_to_file("", 0x0700)
        assert b._nar_lg.lines == []


# ── _note_dry_refusal ────────────────────────────────────────────────────────

class TestNoteDryRefusal:
    LINE = "[AMS_CHMB]err, filament hub load!"

    def test_a_refusal_is_recorded_for_its_unit(self):
        b = _lb()
        b._note_dry_refusal(self.LINE, 0x0700, unit=1)
        assert b._dry_err_u[1] == "filament hub load!"

    def test_an_unattributable_refusal_is_dropped(self):
        b = _lb()
        b._note_dry_refusal(self.LINE, 0x0700, unit=None)
        assert b._dry_err_u == {}

    def test_heating_clears_that_units_refusal(self):
        b = _lb()
        b._note_dry_refusal(self.LINE, 0x0700, unit=1)
        b._note_dry_refusal("[AMS_CHMB]set state CTC_STATE_HEATING",
                            0x0700, unit=1)
        assert 1 not in b._dry_err_u

    def test_shell_ok_clears_the_hts_open_lid_note(self):
        b = _lb()
        b._note_dry_refusal("[AMS_CHMB]err, ams-ht shell open!",
                            0x1800, unit=4)
        b._note_dry_refusal("dry_mode:1, ams-ht shell ok!", 0x1800, unit=4)
        assert 4 not in b._dry_err_u

    def test_no_address_records_nothing(self):
        b = _lb()
        b._note_dry_refusal(self.LINE, None, unit=1)
        assert b._dry_err_u == {}


# ── _note_dry_cfg / last_dry_cfg ─────────────────────────────────────────────

class TestNoteDryCfg:
    LINE = ("[AMS_CHMB]rotate:1, 0, pw_lim:100, cool_down:0, 0, "
            "dur:480, tmpr:55")

    def test_the_echo_is_recorded_by_unit(self):
        b = _lb()
        b._note_dry_cfg(self.LINE, 0x0700, unit=0)
        assert b._dry_cfg_u[0] == {"rotate": 1, "dur": 480, "tmpr": 55}

    def test_a_non_cfg_line_records_nothing(self):
        b = _lb()
        b._note_dry_cfg("[AMS_PMSM]mode:0->2", 0x0700, unit=0)
        assert b._dry_cfg_u == {}

    def test_unattributable_echo_is_dropped(self):
        b = _lb()
        b._note_dry_cfg(self.LINE, 0x0700, unit=None)
        assert b._dry_cfg_u == {}


class TestLastDryCfg:
    def test_none_unit_is_none(self):
        assert _lb().last_dry_cfg(None) is None

    def test_never_echoed_is_none(self):
        assert _lb().last_dry_cfg(0) is None

    def test_returns_a_copy(self):
        b = _lb()
        b._note_dry_cfg(TestNoteDryCfg.LINE, 0x0700, unit=0)
        got = b.last_dry_cfg(0)
        got["dur"] = 999
        assert b._dry_cfg_u[0]["dur"] == 480


# ── _note_cap_measure / last_cap_measure / last_ht_cali ──────────────────────

class TestNoteCapMeasure:
    def test_a_live_measurement_is_recorded_with_its_time(self):
        b = _lb()
        b._note_cap_measure("odom C:1.234, R:0.084, P:78%", 0x0700, 99.0)
        m = b._cap_measure[0x0700]
        assert m["pct"] == 78
        assert m["t"] == 99.0

    def test_an_ht_verdict_is_recorded_by_unit(self):
        b = _lb()
        b._note_cap_measure("Calibration rst:0", 0x1800, 50.0, unit=4)
        assert b._ht_cali_u[4] == {"rst": 0, "t": 50.0}

    def test_a_plain_line_records_nothing(self):
        b = _lb()
        b._note_cap_measure("[AMS_PMSM]mode:0->2", 0x0700, 99.0)
        assert b._cap_measure == {}

    def test_no_address_records_nothing(self):
        b = _lb()
        b._note_cap_measure("odom C:1.234, R:0.084, P:78%", None, 99.0)
        assert b._cap_measure == {}


class TestCapSaveTray:
    """
    The unit naming the bay its measurement was of.

    The measure line does not say which tray it measured; the "odom save
    tray:N" line right behind it does, and nothing read it. The host had to
    guess from its own pending marker instead, and on 2026-09-20 that guess
    wrote lane13's spool measurement onto lane15's Spoolman spool. See
    _CAP_SAVE_RE.
    """

    #: One batch, exactly as the AMS 1 emitted it at 13:59:35.
    BATCH = ("[AMS_RFID]STEP:odom detect #2, odo:0.609 "
             "[AMS_RFID]STEP:odom C:0.588,R:0.094,P:142%,N:2,od:0.609 "
             "[AMS_RFID]STEP:odom save tray:1, R:0.093643 "
             "[AMS_PMSM]mode:2->0")

    def test_the_tray_is_taken_from_the_same_batch(self):
        b = _lb()
        b._note_cap_measure(self.BATCH, 0x0700, 99.0)
        m = b._cap_measure[0x0700]
        assert m["pct_raw"] == 142 and m["radius_m"] == 0.094
        assert m["save_tray"] == 1
        # The save line's own precision is kept: the measure line rounds to
        # the millimetre, and at this radius a millimetre is ~2% of the reel.
        assert m["save_radius_m"] == 0.093643

    def test_a_later_batch_still_labels_the_reading(self):
        # The save does not always ride with the measurement.
        b = _lb()
        b._note_cap_measure("odom C:0.588,R:0.094,P:142%", 0x0700, 99.0)
        b._note_cap_measure("[AMS_RFID]STEP:odom save tray:2, R:0.093643",
                            0x0700, 99.2)
        assert b._cap_measure[0x0700]["save_tray"] == 2

    def test_a_save_from_another_cycle_cannot_relabel_it(self):
        # Matched on the radius, not on recency: a save line left over from a
        # different measurement describes a different spool, and attaching it
        # here would be worse than having no label at all.
        b = _lb()
        b._note_cap_measure("odom C:0.588,R:0.094,P:142%", 0x0700, 99.0)
        b._note_cap_measure("odom save tray:3, R:0.071200", 0x0700, 99.2)
        assert b._cap_measure[0x0700]["save_tray"] is None
        assert b._cap_measure[0x0700]["save_radius_m"] is None

    def test_a_save_alone_records_no_measurement(self):
        # It carries no percent, so it is not a reading.
        b = _lb()
        b._note_cap_measure("odom save tray:1, R:0.093643", 0x0700, 99.0)
        assert b._cap_measure == {}

    def test_an_unlabelled_measurement_says_so(self):
        b = _lb()
        b._note_cap_measure("odom C:1.234, R:0.084, P:78%", 0x0700, 99.0)
        assert b._cap_measure[0x0700]["save_tray"] is None


class TestLastCapMeasure:
    def test_no_address_is_none(self):
        assert _lb().last_cap_measure(None) is None

    def test_never_measured_is_none(self):
        assert _lb().last_cap_measure(0x0700) is None

    def test_returns_a_copy(self):
        b = _lb()
        b._note_cap_measure("odom C:1.234, R:0.084, P:78%", 0x0700, 99.0)
        got = b.last_cap_measure(0x0700)
        got["pct"] = 1
        assert b._cap_measure[0x0700]["pct"] == 78


class TestLastHtCali:
    def test_none_unit_is_none(self):
        assert _lb().last_ht_cali(None) is None

    def test_never_reported_is_none(self):
        assert _lb().last_ht_cali(4) is None

    def test_reports_the_verdict(self):
        b = _lb()
        b._note_cap_measure("Calibration rst:0", 0x1800, 50.0, unit=4)
        assert b.last_ht_cali(4) == {"rst": 0, "t": 50.0}

    def test_an_ht_done_line_is_not_a_verdict(self):
        """One HT cycle end says BOTH lines, ~6s apart:

            19:37:18  [AMS_RFID] STEP4,odom calib sucess
            19:37:26  [AMS_RFID] STEP4,Calibration rst:0

        Stamping both made one cycle end read as two verdicts, and the second
        landed on whatever came next: the one-edge retry fired off the first,
        the duplicate arrived six seconds later, and the module announced
        "one-edged again on the retry -- giving up" 26 seconds before that
        retry measured 97% and wrote it to Spoolman. On an HT the rst line is
        the verdict; the done-line is its preamble."""
        b = _lb()
        b._note_cap_measure("[AMS_RFID] STEP4,odom calib sucess",
                            0x1800, 44.0, unit=4)
        assert b.last_ht_cali(4) is None          # not yet -- rst decides
        b._note_cap_measure("Calibration rst:0", 0x1800, 50.0, unit=4)
        assert b.last_ht_cali(4) == {"rst": 0, "t": 50.0}   # once

    def test_the_ams1_done_line_still_maps_to_rst_0(self):
        # The mapping exists FOR the AMS 1, which has no rst line at all --
        # "odom calib success exit 0" is its only cycle-end sentence.
        b = _lb()
        b._note_cap_measure("STEP:odom calib success exit 0,dis:0.989",
                            0x0700, 44.0, unit=0)
        assert b.last_ht_cali(0) == {"rst": 0, "t": 44.0}


# ── cap_calibrating ──────────────────────────────────────────────────────────

class TestCapCalibrating:
    def test_no_address_is_false(self):
        assert _lb().cap_calibrating(None) is False

    def test_not_measuring_is_false(self):
        assert _lb().cap_calibrating(0x0700) is False

    def test_live_measurement_is_true(self):
        b = _lb()
        b._meas_live[0x0700] = True
        b._meas_live_t[0x0700] = time.time()
        assert b.cap_calibrating(0x0700) is True

    def test_a_stalled_flag_expires(self):
        """A calibrate that dies at the first edge must not refuse every
        later calibrate for the rest of the session."""
        b = _lb()
        b._meas_live[0x0700] = True
        b._meas_live_t[0x0700] = time.time() - 1000.0
        assert b.cap_calibrating(0x0700) is False
        assert b._meas_live[0x0700] is False


# ── last_terminal ────────────────────────────────────────────────────────────

class TestLastTerminal:
    def test_no_address_is_none(self):
        assert _lb().last_terminal(None) is None

    def test_never_finished_is_none(self):
        assert _lb().last_terminal(0x0700) is None

    def test_reports_the_stamp(self):
        b = _lb()
        b._rfid_term_by_addr[0x0700] = 77.0
        assert b.last_terminal(0x0700) == 77.0


# ── clear_dry_error ──────────────────────────────────────────────────────────

class TestClearDryError:
    def test_clears_that_unit_only(self):
        b = _lb()
        b._dry_err_u[1] = "filament hub load!"
        b._dry_err_u[4] = "ams-ht shell open!"
        b.clear_dry_error(1)
        assert b._dry_err_u == {4: "ams-ht shell open!"}

    def test_none_unit_clears_nothing(self):
        b = _lb()
        b._dry_err_u[1] = "filament hub load!"
        b.clear_dry_error(None)
        assert b._dry_err_u == {1: "filament hub load!"}

    def test_clearing_an_unset_unit_is_safe(self):
        b = _lb()
        b.clear_dry_error(3)          # no raise
        assert b._dry_err_u == {}


# ── last_buff_refill ─────────────────────────────────────────────────────────

class TestLastBuffRefill:
    def test_none_before_any_refill(self):
        assert _lb().last_buff_refill() is None

    def test_reports_the_tuple(self):
        b = _lb()
        b._buff_refill = (0.31, 0.62, 18.0)
        assert b.last_buff_refill() == (0.31, 0.62, 18.0)


# ── _trace_fstate ────────────────────────────────────────────────────────────

class TestTraceFstate:
    def test_a_change_is_logged_with_the_buffer(self):
        b = _lb()
        b._nar_lg = _NarLog()
        b._trace_fstate({"fstate": 3, "buff": 54})
        assert b._nar_lg.lines == ["HOST-- fstate - -> 3 (buff=54)"]

    def test_the_same_value_again_is_not_logged(self):
        b = _lb()
        b._nar_lg = _NarLog()
        b._trace_fstate({"fstate": 3, "buff": 54})
        b._trace_fstate({"fstate": 3, "buff": 60})
        assert len(b._nar_lg.lines) == 1

    def test_a_transition_names_both_states(self):
        b = _lb()
        b._nar_lg = _NarLog()
        b._trace_fstate({"fstate": 3, "buff": 54})
        b._trace_fstate({"fstate": 0, "buff": 58})
        assert b._nar_lg.lines[1] == "HOST-- fstate 3 -> 0 (buff=58)"

    def test_no_log_configured_is_a_noop(self):
        b = _lb()
        b._nar_lg = None
        b._trace_fstate({"fstate": 3})          # no raise


# ── _rfid_stamp ──────────────────────────────────────────────────────────────

class TestRfidStamp:
    def test_bridge_wide_when_no_address_given(self):
        assert _lb()._rfid_stamp(10.0, {0x0700: 20.0}, None) == 10.0

    def test_a_devices_own_stamp_wins(self):
        assert _lb()._rfid_stamp(10.0, {0x0700: 20.0}, 0x0700) == 20.0

    def test_a_silent_device_on_an_attributing_bridge_is_none(self):
        """Another unit's chatter must not be credited to this one."""
        assert _lb()._rfid_stamp(10.0, {0x1800: 20.0}, 0x0700) is None

    def test_no_attribution_at_all_falls_back_to_wide(self):
        """A firmware predating per-device stamps must not read as 'never'."""
        assert _lb()._rfid_stamp(10.0, {}, 0x0700) == 10.0


# ── a silently vanished bridge must not be waited on for ever ────────────────

class TestSilenceForcesReconnect:
    """read() returns b"" on timeout and RAISES at end of stream, so a CLEAN
    disconnect reconnects properly. A bridge that vanishes SILENTLY -- BOOTSEL,
    power cut, AP drop -- sends no FIN and no RST, so read just keeps timing
    out and the reader sits on a socket to nothing.

    Measured twice on hardware: bridge_connected stayed true, pres_ok froze,
    and recovery came only from power-cycling the board.
    """

    def _bridge(self, *, silent_for, fw_raw=False, has_port=True):
        b = BambuBridge.__new__(BambuBridge)
        b.logger = logging.getLogger("test-silence")
        b._serial = object() if has_port else None
        b._fw_raw = fw_raw
        b._last_frame_t = time.monotonic() - silent_for
        # No connection stamp, so the silence is measured purely from the last
        # frame: these cases predate the connect grace and are unaffected by it.
        b._connected_t = None
        b._grace_this_conn = False
        b._spoke_since_connect = True
        b._down_t = None
        b._silence_logged_t = None
        b.dropped = False

        def _drop():
            b.dropped = True
            b._serial = None
        b._drop_port = _drop
        return b

    def test_a_quiet_moment_is_not_a_drop(self):
        b = self._bridge(silent_for=5.0)
        assert b._drop_if_silent() is False
        assert b.dropped is False

    def test_THE_ONE_THAT_BIT_long_silence_forces_a_reconnect(self):
        b = self._bridge(silent_for=BambuBridge.QUIET_DROP_S + 1)
        assert b._drop_if_silent() is True
        assert b.dropped is True

    def test_never_during_a_firmware_transfer(self):
        # The board is legitimately silent while counting image bytes.
        # Dropping the link mid-flash is worse than any slow reconnect.
        b = self._bridge(silent_for=BambuBridge.QUIET_DROP_S + 60, fw_raw=True)
        assert b._drop_if_silent() is False
        assert b.dropped is False

    def test_nothing_to_drop_when_already_disconnected(self):
        b = self._bridge(silent_for=9999, has_port=False)
        assert b._drop_if_silent() is False

    def test_the_threshold_leaves_room_for_a_normal_poll_gap(self):
        # The firmware streams status continuously; the drop threshold must sit
        # well above any legitimate gap, and below the warn-only threshold's
        # usefulness for a print.
        assert 10.0 <= BambuBridge.QUIET_DROP_S <= 60.0


# ── the link key handshake ───────────────────────────────────────────────────

class TestLinkKeyHandshake:
    """The bridge listens on a port anyone on the network can reach. Secure
    boot means they cannot install firmware, but without this they could put
    the board in BOOTSEL, wipe its WiFi credentials, or drive the motors --
    every time Klipper is not holding the socket.
    """

    def _fake_sock(self, script):
        """A socket that replays `script` (list of bytes) and records sends."""
        class S:
            def __init__(s):
                s.inbox = list(script)
                s.sent = b""
                s.timeout = None
                s.closed = False
            def settimeout(s, t): s.timeout = t
            def setsockopt(s, *a): pass
            def recv(s, n):
                if not s.inbox:
                    raise socket.timeout()
                return s.inbox.pop(0)
            def sendall(s, b): s.sent += b
            def send(s, b): s.sent += b; return len(b)
            def close(s): s.closed = True
        return S()

    def _port(self, script, key):
        p = br.TcpPort.__new__(br.TcpPort)
        p.name = "tcp://test:8888"
        p._timeout = 0.1
        p._pushback = b""
        p._sock = self._fake_sock(script)
        p._authenticate(key, 0.5)
        return p

    def _mac_for(self, key, nonce_hex):
        return hmac.new(key.encode(), bytes.fromhex(nonce_hex),
                        hashlib.sha512).digest()[:32].hex()

    NONCE = "000102030405060708090a0b0c0d0e0f"

    def test_a_correct_key_authenticates(self):
        chal = b'{"evt":"auth","nonce":"%s"}\n' % self.NONCE.encode()
        p = self._port([chal, b'{"evt":"auth","ok":1}\n'], "hunter2")
        assert self._mac_for("hunter2", self.NONCE).encode() in p._sock.sent

    def test_THE_POINT_a_wrong_key_is_refused(self):
        chal = b'{"evt":"auth","nonce":"%s"}\n' % self.NONCE.encode()
        with pytest.raises(OSError, match="rejected the link key"):
            self._port([chal, b'{"evt":"auth","ok":0}\n'], "wrong")

    def test_a_keyed_board_with_no_configured_key_is_an_error(self):
        # Fails LOUDLY rather than hanging: the operator needs to be told to
        # set tcp_key, not left watching a link that never comes up.
        chal = b'{"evt":"auth","nonce":"%s"}\n' % self.NONCE.encode()
        with pytest.raises(OSError, match="asked for a link key"):
            self._port([chal], None)

    def test_an_open_board_still_works_with_no_key(self):
        # No challenge at all: silence means the far end is unauthenticated,
        # which is every board built before this existed.
        p = self._port([], None)
        assert p._sock.sent == b""

    def test_an_open_board_works_even_when_a_key_IS_configured(self):
        # The rollout direction that must not break: config updated first,
        # boards keyed later. The board never challenges, so nothing is sent.
        p = self._port([], "hunter2")
        assert p._sock.sent == b""

    def test_a_frame_arriving_instead_of_a_challenge_is_not_eaten(self):
        # An open board that is already talking. The handshake reads by the
        # bufferful, so whatever it over-read must come back out of read().
        frame = b'{"evt":"status","online":true}\n'
        p = self._port([frame], "hunter2")
        assert p._sock.sent == b""
        assert p.read(4096) == frame

    def test_the_key_itself_never_goes_on_the_wire(self):
        chal = b'{"evt":"auth","nonce":"%s"}\n' % self.NONCE.encode()
        p = self._port([chal, b'{"evt":"auth","ok":1}\n'], "hunter2")
        assert b"hunter2" not in p._sock.sent

    def test_the_response_is_bound_to_THIS_nonce(self):
        # Replay protection: the same key against a different challenge must
        # produce a different answer, or a recorded session is reusable.
        a = self._mac_for("k", self.NONCE)
        b_ = self._mac_for("k", "0f0e0d0c0b0a09080706050403020100")
        assert a != b_

    def test_a_closed_socket_mid_handshake_raises(self):
        with pytest.raises(OSError, match="closed during authentication"):
            self._port([b""], "hunter2")


# ── The AMS 2's unload completion: tray_now leaving a tray for 255 ───────────
#
# Every narration string below is verbatim from the U1's AMS 2 on 2026-09-03,
# so these test the shapes the hardware actually emits rather than shapes
# invented to match the parser.

class TestTrayRelease:
    """`tray_now` -> 255 is the AMS 2's real unload completion.

    It beat `state_switch finish` 7/7 to 6/7 across a day of commanded
    unloads and fires at the tray switch release, 0-26s earlier. What makes it
    safe is that it is exposed as an EDGE scoped to a unit, plus the tray it
    left -- because the resting level IS 255, and because 18 of that day's 25
    edges were the operator handling spools rather than an unload.
    """

    @staticmethod
    def _narrate(b, text, unit=0):
        _feed(b, json.dumps({"evt": "amsdbg", "text": text,
                             "addr": 0x0700, "unit": unit}))

    def test_the_release_edge_is_reported_with_the_tray_it_left(self):
        b, _r, _lg = _bridge()
        # Loaded on tray 1, exactly as the unit says it while unloading.
        self._narrate(b, "[AMS_COMMON]state:0,tray_now:1,tray_exit:7")
        assert b.last_tray_release(unit=0) == (0, None)
        # The release, verbatim from 14:22:32.
        self._narrate(
            b,
            "[AMS_TRAY]tray[1] sw_sta update, 3 -> 1, u_in_out:2982,2509 "
            "[AMS_COMMON]state:2,tray_now:255,tray_exit:7")
        assert b.last_tray_release(unit=0) == (1, 1)

    def test_the_resting_level_is_not_an_edge(self):
        """`state:0,tray_now:255` is where the unit SITS -- 120 hits in a day.

        Read as a level it would end every wait instantly.
        """
        b, _r, _lg = _bridge()
        for _ in range(5):
            self._narrate(b, "[AMS_COMMON]state:0,tray_now:255,tray_exit:7")
        assert b.last_tray_release(unit=0) == (0, None)

    def test_one_line_carrying_the_whole_transition_still_counts(self):
        """A frame can hold several [AMS_COMMON] segments; the edge may be
        between two of them, so the last value alone would miss it."""
        b, _r, _lg = _bridge()
        self._narrate(
            b,
            "[AMS_COMMON]state:0,tray_now:0,tray_exit:7 "
            "[AMS_COMMON]state:2,tray_now:255,tray_exit:7")
        assert b.last_tray_release(unit=0) == (1, 0)

    def test_each_unload_advances_the_sequence_once(self):
        b, _r, _lg = _bridge()
        for tray in (1, 0, 1):
            self._narrate(b, f"[AMS_COMMON]state:0,tray_now:{tray},tray_exit:7")
            self._narrate(b, "[AMS_COMMON]state:2,tray_now:255,tray_exit:7")
        seq, left = b.last_tray_release(unit=0)
        assert (seq, left) == (3, 1)

    def test_units_are_kept_apart(self):
        """Both boxed units answer on 0x0700, so the address cannot separate
        them -- the narration's own unit index is what does."""
        b, _r, _lg = _bridge()
        self._narrate(b, "[AMS_COMMON]state:0,tray_now:1,tray_exit:7", unit=0)
        self._narrate(b, "[AMS_COMMON]state:0,tray_now:3,tray_exit:7", unit=1)
        self._narrate(b, "[AMS_COMMON]state:2,tray_now:255,tray_exit:7", unit=1)
        assert b.last_tray_release(unit=0) == (0, None)   # untouched
        assert b.last_tray_release(unit=1) == (1, 3)

    def test_unattributed_narration_is_dropped_not_applied_broadly(self):
        """This ends a move. A guess would let a neighbour end our retract."""
        b, _r, _lg = _bridge()
        _feed(b, json.dumps({"evt": "amsdbg", "addr": 0x0700,
                             "text": "[AMS_COMMON]state:0,tray_now:1"}))
        _feed(b, json.dumps({"evt": "amsdbg", "addr": 0x0700,
                             "text": "[AMS_COMMON]state:2,tray_now:255"}))
        assert b.last_tray_release(unit=0) == (0, None)
        assert b.last_tray_release() == (0, None)

    def test_a_spool_insert_still_produces_an_edge(self):
        """Verbatim from 19:19:02 -- an INSERT, not an unload.

        The parser reports it, as it must (it cannot know why the tray
        changed); rejecting it is the waiter's job, via the tray index and the
        fact that no retract is in flight. This pins that division of labour
        so nobody 'fixes' it in the wrong layer.
        """
        b, _r, _lg = _bridge()
        self._narrate(b, "[AMS_COMMON]state:0,tray_now:2,tray_exit:5")
        self._narrate(
            b,
            "[AMS_TRAY]tray[2] sw_sta update, 3 -> 1, u_in_out:3074,2508 "
            "[AMS_COMMON]state:3,tray_now:255,tray_exit:5")
        assert b.last_tray_release(unit=0) == (1, 2)


# ── the bridge's own diagnostics, timed-out writes, and silences ─────────────
#
# An AMS 1 capscan parks the bridge firmware in a 10-15 s blocking burst. While
# it runs the Pico reads nothing from USB, so the 1 Hz chain poll times out once
# a second, and when it ends the firmware's capacity window prints [HT-MEAS]
# once a second, stamped 0x1800 on a bus with no HT. Printer 2, 2026-09-22.

_HT_MEAS = "[HT-MEAS] fire0 capu1 tun1 dst1 act1 arm0 htm0000 on0 off306 pres0"
_BB_GATE = "[BB-GATE] u4 iv150 ht1 flw1 iv<1 m0010 f0000"
_CAP_OPEN = "[CAP] open u1 s2"
# lane8's reading, verbatim from AFC_BambuAMS.log at 20:52:56 (0x0700 u1).
_AMS1_ODOM = ("[AMS_DEV] STEP,second detected [AMS_DEV] STEP:odom C:0.449,"
              "R:0.071,P:63%, od:1.004 [AMS_DEV] STEP:odom save tray:1, "
              "R:0.071481 [AMS_DEV] STEP:odom calib success exit 0,dis:0.832")


def _amsdbg(b, text, addr=0x1800, unit=1):
    _feed(b, json.dumps({"evt": "amsdbg", "addr": addr, "unit": unit,
                         "text": text}))


class TestFirmwareDiagnosticsStopAtTheFile:
    """[HT-MEAS], [BB-GATE] and [CAP] are the bridge talking about itself.
    They are kept verbatim in AFC_BambuAMS.log and reach nothing else."""

    @pytest.mark.parametrize("text", [_HT_MEAS, _BB_GATE, _CAP_OPEN])
    def test_it_is_file_only(self, text):
        b, _r, lg = _bridge()
        b.name = "u"
        b._nar_lg = _NarLog()
        _amsdbg(b, text)
        assert b._nar_lg.lines == [f"0x1800 u1 {text}"]
        assert lg.msgs == [("debug", f"AFC bambu: bridge diag {text}")]
        assert lg.file_only == [f"AFC bambu: bridge diag {text}"]
        assert not any(m.startswith("AMS:") for m in lg.texts())

    def test_it_does_not_break_the_repeat_collapsing_around_it(self):
        """Its counter changes every second, so it used to replace the dedupe's
        last line each time and a repeating AMS line printed on every repeat."""
        b, _r, lg = _bridge()
        b.name = "u"
        line = "+ [AMS_DEV] STEP:odom card in RF,delay check"
        _amsdbg(b, line, addr=0x0700)
        _amsdbg(b, _HT_MEAS)
        _amsdbg(b, line, addr=0x0700)
        assert lg.texts().count(f"AMS: {line}") == 1
        assert b._last_dbg == line and b._last_dbg_n == 2

    def test_it_is_never_evidence(self):
        b, _r, _lg = _bridge()
        b.name = "u"
        before = {k: dict(v) for k, v in vars(b).items()
                  if isinstance(v, dict)}
        for text in (_HT_MEAS, _BB_GATE, _CAP_OPEN):
            _amsdbg(b, text, addr=6144, unit=1)
        after = {k: dict(v) for k, v in vars(b).items() if isinstance(v, dict)}
        assert after == before, "a firmware diagnostic reached a parser"
        assert b.last_ht_cali(1) is None
        assert b.last_cap_measure(0x1800) is None
        assert not b.rfid_cycle_ended_since(0.0)

    def test_the_ams1_own_success_line_still_gives_rst_0(self):
        b, r, _lg = _bridge()
        b.name = "u"
        _amsdbg(b, _HT_MEAS, addr=6144, unit=1)
        _amsdbg(b, _AMS1_ODOM, addr=0x0700, unit=1)
        assert b.last_ht_cali(1) == {"rst": 0, "t": r.monotonic()}
        assert b.last_cap_measure(0x0700)["pct"] == 63

    def test_an_ams_line_mentioning_it_later_is_still_parsed(self):
        # Anchored at the start: only the firmware's own lines begin with it.
        b, _r, lg = _bridge()
        b.name = "u"
        _amsdbg(b, "[AMS_DEV] STEP:odom calib success exit 0 [HT-MEAS]",
                addr=0x0700, unit=1)
        assert b.last_ht_cali(1) is not None
        assert not any("bridge diag" in m for m in lg.texts())


class TestTheCaliEchoHasAHandler:
    """The echo is an acknowledgement, not an outcome: the firmware sends it
    whether it ran the calibrate, deferred it or ignored it (45 s cooldown),
    so the line must not say that it ran."""

    def test_it_is_known(self):
        assert "cali" in br._BRIDGE_EVENTS_KNOWN

    def test_it_is_logged_file_only_and_not_as_unhandled(self):
        b, _r, lg = _bridge()
        _feed(b, '{"evt":"cali","unit":1,"slot":2}')
        assert lg.file_only == [
            "AFC bambu: bridge answered cali (unit 1, slot 2)"]
        assert not any("unhandled" in m for m in lg.texts())

    def test_extra_fields_ride_along_uninterpreted(self):
        b, _r, lg = _bridge()
        _feed(b, '{"evt":"cali","unit":1,"slot":2,"ran":0,"why":3}')
        assert lg.file_only == [
            "AFC bambu: bridge answered cali (unit 1, slot 2) ran=0 why=3"]


class _StalledPort:
    """Takes the bytes and then times out, as pyserial does on a CDC the Pico
    has stopped reading: os.write() first, then the wait that runs out."""

    write_timeout = 0.5

    def __init__(self):
        self.written = []
        self.closed = False

    def write(self, data):
        self.written.append(bytes(data))
        raise br._SerialTimeout("Write timeout")

    def close(self):
        self.closed = True


def _run_real_writer(b):
    """Run the REAL writer loop until its queue is empty, on this thread."""
    import queue as _q
    real_get = b._wq.get

    def get(timeout=None):
        if b._wq.qsize() == 0:
            b._run = False               # drained: let the loop fall out
            raise _q.Empty
        return real_get(timeout=timeout)

    b._wq.get = get
    b._run = True
    BambuBridge._writer(b)


class TestATimedOutWriteNamesItsCommand:
    def test_the_command_and_the_timeout_are_named(self):
        port = _StalledPort()
        b, _r, lg = _bridge(serial=port)
        b.send({"cmd": "chain"})
        _run_real_writer(b)
        assert port.written == [b'{"cmd": "chain"}\n']
        assert lg.texts("debug") == [
            "AFC bambu: bridge busy, write of 'chain' timed out after 0.5 s "
            "(may still be delivered)"]
        # AFC.log only: it nearly always lands late, so the console read it
        # as a failure even with the debug printout on.
        assert lg.file_only == lg.texts("debug")
        # A timeout keeps the port, and still stamps and counts.
        assert b._serial is port and not port.closed
        assert b._write_drop_t is not None and b._write_timeouts == 1
        assert b.writes_dropped_since(0.0)

    @pytest.mark.parametrize("item", [b"x", b"\xff\xfe\n", b'{"slot": 1}\n',
                                      b"[1, 2]\n", b'{"cmd": ""}\n'])
    def test_an_unnameable_item_does_not_break_the_writer(self, item):
        port = _StalledPort()
        b, _r, lg = _bridge(serial=port)
        b._wq.put_nowait(item)
        b.send({"cmd": "stop"})
        _run_real_writer(b)
        assert lg.texts("debug") == [
            "AFC bambu: bridge busy, write of '?' timed out after 0.5 s "
            "(may still be delivered)",
            "AFC bambu: bridge busy, write of 'stop' timed out after 0.5 s "
            "(may still be delivered)"]
        assert b._write_timeouts == 2

    def test_a_port_that_does_not_say_its_timeout_is_not_guessed(self):
        port = _StalledPort()
        port.write_timeout = None
        b, _r, lg = _bridge(serial=port)
        b.send({"cmd": "chain"})
        _run_real_writer(b)
        assert lg.texts("debug") == [
            "AFC bambu: bridge busy, write of 'chain' timed out "
            "(may still be delivered)"]

    def test_the_tcp_transport_is_named_too(self):
        # TcpPort keeps its budget as _write_timeout; printer 1 runs on it.
        port = _StalledTcpPort()
        b, _r, lg = _bridge(serial=port)
        b.send({"cmd": "info"})
        _run_real_writer(b)
        assert lg.texts("debug") == [
            "AFC bambu: bridge busy, write of 'info' timed out after 0.5 s "
            "(may still be delivered)"]


class _StalledTcpPort(_StalledPort):
    """TcpPort's spelling of the same budget (see TcpPort.write)."""

    write_timeout = None
    _write_timeout = 0.5


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t


class _ScriptedPort:
    """A port whose reads follow a script of (seconds_later, what). Each read
    moves the clock first; ``what`` is a chunk to return, an exception to
    raise, or a callable run with the bridge (standing in for the writer
    thread). The script running out stops the reader, so the REAL loop runs
    on the test's own thread."""

    def __init__(self, clock, bridge, steps):
        self.clock, self.bridge, self.steps = clock, bridge, list(steps)
        self.closed = False

    def read(self, n):
        while self.steps:
            dt, what = self.steps.pop(0)
            self.clock.t += dt
            if isinstance(what, Exception):
                raise what
            if callable(what):
                what(self.bridge)
                continue
            return what
        self.bridge._run = False
        return b""

    def write(self, data):
        pass

    def close(self):
        self.closed = True


_HB = b'{"evt":"hb"}\n'


def _two_timeouts(b):
    b._write_timeouts += 2           # what the writer does per timed-out write


class TestTheReaderSaysHowLongTheBridgeWasSilent:
    """One line per gap, written when the gap ENDS, so it carries the length
    that the one-line-per-poll timeouts could only hint at."""

    def _run(self, monkeypatch, steps, then=()):
        clock = _Clock()
        monkeypatch.setattr(br, "time", types.SimpleNamespace(
            monotonic=clock.monotonic, sleep=lambda s: None, time=time.time))
        ports = []
        b, _r, lg = _bridge(serial=_Serial(), factory=lambda: ports.pop(0))
        b._serial = _ScriptedPort(clock, b, steps)
        ports.append(_ScriptedPort(clock, b, then))
        b._run = True
        b._reader()
        return b, lg

    @staticmethod
    def _lines(lg):
        return [m for m in lg.texts() if "was silent" in m]

    def test_one_line_per_gap_with_the_timeouts_in_it(self, monkeypatch):
        steps = [(0.0, _HB), (0.1, _HB), (0.2, _HB),
                 (5.0, _two_timeouts), (5.1, _HB),     # lane9's burst: 10.1 s
                 (0.1, _HB), (0.1, _HB),
                 (2.4, _HB),                           # under the threshold
                 (3.0, _HB)]                           # a WiFi-sized gap
        _b, lg = self._run(monkeypatch, steps)
        want = ["AFC bambu: bridge was silent 10.1 s; 2 write(s) timed out "
                "meanwhile",
                "AFC bambu: bridge was silent 3.0 s; 0 write(s) timed out "
                "meanwhile"]
        assert self._lines(lg) == want
        assert [m for m in lg.file_only if "was silent" in m] == want

    def test_the_first_frame_of_a_connection_is_not_a_gap(self, monkeypatch):
        _b, lg = self._run(monkeypatch, [(9.0, _HB), (0.1, _HB)])
        assert self._lines(lg) == []

    def test_a_reconnect_starts_over(self, monkeypatch):
        """The first frame after a drop follows an outage, not a gap on this
        link, and the timeouts from before the drop are written off with it."""
        steps = [(0.0, _HB), (0.1, _HB), (1.0, _two_timeouts),
                 (1.0, OSError("input/output error"))]
        then = [(6.0, _HB),                  # first frame on the new link
                (0.1, _HB), (4.0, _HB)]      # a gap on it, counted from zero
        _b, lg = self._run(monkeypatch, steps, then)
        assert any("reconnected" in m for m in lg.texts("info"))
        assert self._lines(lg) == [
            "AFC bambu: bridge was silent 4.0 s; 0 write(s) timed out "
            "meanwhile"]


class TestFaultPerUnit:
    """last_fault(unit=) answers for one unit on a shared chain."""

    def _say(self, b, text, unit):
        b.handle_line(json.dumps(
            {"evt": "amsdbg", "text": text, "addr": 0x0700, "unit": unit}))

    def test_a_chain_mates_stall_does_not_move_this_unit(self):
        b, r, lg = _bridge()
        mine = b.last_fault(unit=0)
        self._say(b, "[AMS_SWITCH]feed finish -1, stall", unit=1)
        assert b.last_fault(unit=0) == mine
        assert b.last_fault(unit=1)[0] != 0

    def test_this_units_stall_moves_it_with_its_own_words(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_SWITCH]feed finish -1, stall", unit=0)
        seq, text, _a = b.last_fault(unit=0)
        assert seq and "finish -1" in text

    def test_an_unattributed_stall_counts_for_every_unit(self):
        # The firmware sends unit -1 when it cannot tell which unit spoke;
        # dropping that stall would let a load ride out a real fault.
        b, r, lg = _bridge()
        before = (b.last_fault(unit=0)[0], b.last_fault(unit=1)[0])
        self._say(b, "[AMS_LED]TIMEOUT error 2", unit=-1)
        assert b.last_fault(unit=0)[0] != before[0]
        assert b.last_fault(unit=1)[0] != before[1]
        assert "TIMEOUT" in b.last_fault(unit=1)[1]

    def test_a_later_own_stall_outranks_an_older_unattributed_one(self):
        b, r, lg = _bridge()
        self._say(b, "[AMS_LED]TIMEOUT error 2", unit=-1)
        self._say(b, "[AMS_SWITCH]feed finish -1, stall", unit=0)
        assert "finish -1" in b.last_fault(unit=0)[1]
        assert "TIMEOUT" in b.last_fault(unit=1)[1]
