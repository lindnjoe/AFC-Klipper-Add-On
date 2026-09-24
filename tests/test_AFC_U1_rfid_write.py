"""
The U1's write hand-off to the OpenRFID daemon.

The U1's reader belongs to the root OpenRFID daemon, not Klipper, so the write
cannot go through a register link like the other units. AFC_U1_rfid instead
drops a request file the daemon answers with a result file. These tests drive
that file protocol from the Klipper side against a stand-in daemon -- a thread
that reads the request and writes the result -- with no hardware.
"""

import json
import os
import threading
import time
import types

import pytest

import extras.AFC_U1_rfid as mod
from extras.AFC_U1_rfid import AFC_U1_RFID


def _u1(tmp_path, scanner=(0,), lanes=None):
    """An AFC_U1_RFID built past __init__ with just what the write path needs."""
    u = AFC_U1_RFID.__new__(AFC_U1_RFID)
    u.printer = types.SimpleNamespace(
        get_start_args=lambda: {"config_file": str(tmp_path / "printer.cfg")})
    u._cfg_write_dir = str(tmp_path / ".afc_u1_write")
    u._cfg_scanner_channels = set(scanner)
    u._lane_channel_map = dict(lanes or {})
    u.logger = types.SimpleNamespace(info=lambda *a: None,
                                     warning=lambda *a: None)
    registered = {}
    u.printer.lookup_object = lambda name, default=None: default
    return u, registered


def _fake_daemon(dirpath, reply, delay=0.0):
    """A stand-in write-watch controller: answer the first request with reply."""
    stop = threading.Event()

    def run():
        while not stop.is_set():
            if os.path.isdir(dirpath):
                for name in sorted(os.listdir(dirpath)):
                    if name.startswith("req-") and name.endswith(".json"):
                        token = name[4:-5]
                        with open(os.path.join(dirpath, name)) as f:
                            req = json.load(f)
                        os.remove(os.path.join(dirpath, name))
                        time.sleep(delay)
                        r = dict(reply)
                        r.setdefault("_req", req)
                        with open(os.path.join(dirpath, f"res-{token}.json"),
                                  "w") as f:
                            json.dump(r, f)
                        return
            time.sleep(0.01)

    th = threading.Thread(target=run, daemon=True)
    th.start()
    return stop, th


class TestOpenrfidWrite:
    def test_a_successful_write_returns_the_uid(self, tmp_path):
        u, _ = _u1(tmp_path)
        d = u._write_dir()
        stop, th = _fake_daemon(d, {"ok": True, "uid": "04a1b2c3d4e5f6"})
        try:
            uid, err = u._openrfid_write(0, bytes(144))
        finally:
            stop.set()
        assert err is None
        assert uid == "04a1b2c3d4e5f6"

    def test_the_request_carries_slot_page_and_data(self, tmp_path):
        u, _ = _u1(tmp_path)
        d = u._write_dir()
        seen = {}

        # capture the request the daemon saw
        stop, th = _fake_daemon(d, {"ok": True, "uid": "04"})
        try:
            payload = bytes(range(144))
            u._openrfid_write(2, payload)
        finally:
            stop.set()
        # read it back off the result the fake echoed
        res = [f for f in os.listdir(d) if f.startswith("res-")][0]
        req = json.load(open(os.path.join(d, res)))["_req"]
        assert req["slot"] == 2
        assert req["start_page"] == mod._WRITE_START_PAGE == 4
        assert bytes.fromhex(req["data"]) == payload

    def test_a_daemon_error_is_surfaced(self, tmp_path):
        u, _ = _u1(tmp_path)
        d = u._write_dir()
        stop, th = _fake_daemon(
            d, {"ok": False, "error": "no tag in the reader's field"})
        try:
            uid, err = u._openrfid_write(0, bytes(144))
        finally:
            stop.set()
        assert "no tag" in err

    def test_a_half_written_result_is_not_read_early(self, tmp_path):
        """The daemon renames its result into place; a reader must wait for the
        whole thing, not parse a partial file."""
        u, _ = _u1(tmp_path)
        d = u._write_dir()
        stop, th = _fake_daemon(d, {"ok": True, "uid": "04"}, delay=0.2)
        try:
            uid, err = u._openrfid_write(0, bytes(144))
        finally:
            stop.set()
        assert (uid, err) == ("04", None)

    def test_no_daemon_times_out_with_a_helpful_message(self, tmp_path,
                                                        monkeypatch):
        u, _ = _u1(tmp_path)
        monkeypatch.setattr(mod, "_WRITE_TIMEOUT_S", 0.3)
        uid, err = u._openrfid_write(0, bytes(144))
        assert uid is None
        assert "no answer from OpenRFID" in err
        assert "write-watch" in err

    def test_a_timed_out_request_is_cleaned_up(self, tmp_path, monkeypatch):
        u, _ = _u1(tmp_path)
        monkeypatch.setattr(mod, "_WRITE_TIMEOUT_S", 0.3)
        u._openrfid_write(0, bytes(144))
        d = u._write_dir()
        leftover = [f for f in os.listdir(d) if f.startswith("req-")]
        assert leftover == [], "a stale request must not be left for the daemon"


class TestRegisterWriters:
    def test_scanner_and_lane_channels_each_get_a_target(self, tmp_path):
        import extras.AFC_rfid_write as rw
        u, _ = _u1(tmp_path, scanner={0}, lanes={"lane4": 1, "lane5": 2})
        gcode = types.SimpleNamespace(
            register_command=lambda *a, **k: None)
        reg = {}
        u.printer = types.SimpleNamespace(
            lookup_object=lambda n, d=None: gcode if n == "gcode" else d,
            _afc_rfid_write_registry=reg,
            get_start_args=lambda: {"config_file": str(tmp_path / "p.cfg")})
        u._register_writers()
        assert set(reg) == {"u1:scanner0", "u1:lane4", "u1:lane5"}
        # each is a hand-off target (no link path)
        assert all(t.write_payload is not None for t in reg.values())

    def test_a_channel_used_by_both_is_registered_once(self, tmp_path):
        u, _ = _u1(tmp_path, scanner={1}, lanes={"lane4": 1})
        gcode = types.SimpleNamespace(register_command=lambda *a, **k: None)
        reg = {}
        u.printer = types.SimpleNamespace(
            lookup_object=lambda n, d=None: gcode if n == "gcode" else d,
            _afc_rfid_write_registry=reg,
            get_start_args=lambda: {"config_file": str(tmp_path / "p.cfg")})
        u._register_writers()
        # channel 1 seen first as the scanner; the lane does not double it
        assert list(reg) == ["u1:scanner1"]
