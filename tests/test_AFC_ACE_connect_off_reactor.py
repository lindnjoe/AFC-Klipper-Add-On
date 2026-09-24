"""
Tests that ACE (re)connect autodetect runs off the reactor (extras/AFC_ACE.py).

The autodetect probes are blocking pyserial reads and sleeps that can take 8 s
or more when a unit is slow or gone. On the reactor they stalled Klipper for
8.2 s, past the 8.26 s a 520 MHz MCU's 32-bit clock takes to wrap, so clock sync
was lost and the printer later shut down with "Timer too close". connect() now
runs the probes on a worker thread and waits with a reactor completion, so
the reactor keeps servicing everything else meanwhile.

The fake reactor below keeps Klipper's contract: async callbacks posted from a
thread run on the reactor thread, and completion.wait() keeps the reactor
turning (counted as ticks) until complete() or the deadline.
"""

from __future__ import annotations

import queue
import sys
import threading
import time
import types

import pytest

from extras import AFC_ACE
from extras.AFC_ACE import (
    ACEConnection,
    ACESerialError,
    _ACE_CLAIMED_PORTS,
    run_off_reactor,
)


class _Completion:
    def __init__(self, reactor):
        self._reactor = reactor
        self._done = False
        self._result = None
        self.completed_on = None

    def complete(self, result):
        self.completed_on = threading.get_ident()
        self._result = result
        self._done = True

    def wait(self, waketime=None, waketime_result=None):
        self._reactor.run_until(lambda: self._done, waketime)
        return self._result if self._done else waketime_result


class ThreadedReactor:
    """Single-threaded reactor stand-in with Klipper's async-callback contract."""

    NEVER = 9999999999999999.0

    def __init__(self):
        self._posted = queue.Queue()
        self.ticks = 0
        self.on_tick = None
        self.completions = []

    def monotonic(self):
        return time.monotonic()

    def register_async_callback(self, cb, waketime=None):
        self._posted.put(cb)

    def completion(self):
        c = _Completion(self)
        self.completions.append(c)
        return c

    def register_fd(self, fd, cb):
        return ("fd", fd)

    def unregister_fd(self, handle):
        pass

    def run_until(self, done, waketime):
        while not done():
            if waketime is not None and self.monotonic() >= waketime:
                return
            self.ticks += 1
            if self.on_tick is not None:
                self.on_tick()
            try:
                cb = self._posted.get(timeout=0.01)
            except queue.Empty:
                continue
            cb(self.monotonic())


class _RecLog:
    def __init__(self):
        self.lines = []

    def _rec(self, level, msg):
        self.lines.append((level, msg, threading.get_ident()))

    def debug(self, msg, *a, **k):
        self._rec("debug", msg)

    def info(self, msg, *a, **k):
        self._rec("info", msg)

    def warning(self, msg, *a, **k):
        self._rec("warning", msg)

    def error(self, msg, *a, **k):
        self._rec("error", msg)


@pytest.fixture(autouse=True)
def _clean_claims():
    _ACE_CLAIMED_PORTS.clear()
    yield
    _ACE_CLAIMED_PORTS.clear()


# ── run_off_reactor ─────────────────────────────────────────────────────────

def test_runs_on_a_worker_while_the_reactor_turns():
    reactor = ThreadedReactor()
    main = threading.get_ident()
    seen = {}

    def slow():
        seen["thread"] = threading.get_ident()
        time.sleep(0.3)
        return "/dev/ttyACM2"

    assert run_off_reactor(reactor, slow) == "/dev/ttyACM2"
    assert seen["thread"] != main
    # the reactor kept servicing events for the whole probe
    assert reactor.ticks >= 10
    # and the wake-up ran on the reactor thread, not the worker
    assert reactor.completions[0].completed_on == main


def test_worker_exception_is_raised_on_the_reactor():
    reactor = ThreadedReactor()

    def boom():
        raise ACESerialError("no unit found")

    with pytest.raises(ACESerialError, match="no unit found"):
        run_off_reactor(reactor, boom)


def test_gives_up_after_the_timeout():
    reactor = ThreadedReactor()
    gate = threading.Event()
    try:
        with pytest.raises(ACESerialError, match="did not finish"):
            run_off_reactor(reactor, lambda: gate.wait(5), timeout=0.2)
    finally:
        gate.set()


def test_reactor_without_async_callbacks_runs_inline():
    class Bare:
        pass

    main = threading.get_ident()
    assert run_off_reactor(Bare(), threading.get_ident) == main


# ── ACEConnection.connect ──────────────────────────────────────────────────

class _FakeSerial:
    opened = []

    def __init__(self, port, baudrate, timeout, write_timeout):
        self.port = port
        _FakeSerial.opened.append(port)

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def fileno(self):
        return 42

    def close(self):
        pass


@pytest.fixture
def conn(monkeypatch):
    _FakeSerial.opened = []
    monkeypatch.setitem(sys.modules, "serial",
                        types.SimpleNamespace(Serial=_FakeSerial))
    reactor = ThreadedReactor()
    log = _RecLog()
    c = ACEConnection(reactor, "auto", logger=log, ace_index=1)
    monkeypatch.setattr(c, "send_command", lambda *a, **k: {"model": "x"})
    monkeypatch.setattr(c, "_start_heartbeat", lambda: None)
    return c, reactor, log


def test_connect_resolves_off_reactor_and_logs_on_it(conn, monkeypatch):
    c, reactor, log = conn
    main = threading.get_ident()
    probe = {}

    def resolve(ace_index, baud, settle=6.0, logger=None):
        probe["thread"] = threading.get_ident()
        logger.info("ACE autodetect: /dev/ttyACM7 reports id 1")
        time.sleep(0.3)
        return "/dev/ttyACM7"

    monkeypatch.setattr(AFC_ACE, "resolve_ace_port_by_index", resolve)
    c.connect()

    assert c.connected
    assert _FakeSerial.opened == ["/dev/ttyACM7"]
    assert probe["thread"] != main
    assert reactor.ticks >= 10
    # the worker's log line was written, from the reactor thread
    probe_lines = [l for l in log.lines if "reports id 1" in l[1]]
    assert probe_lines and probe_lines[0][2] == main
    assert all(t == main for _, _, t in log.lines)


def test_second_connect_while_one_waits_is_refused(conn, monkeypatch):
    c, reactor, log = conn
    refused = []

    def resolve(ace_index, baud, settle=6.0, logger=None):
        time.sleep(0.3)
        return "/dev/ttyACM7"

    def try_again():
        if not refused:
            with pytest.raises(ACESerialError, match="already in progress"):
                c.connect()
            refused.append(True)

    monkeypatch.setattr(AFC_ACE, "resolve_ace_port_by_index", resolve)
    reactor.on_tick = try_again
    c.connect()

    assert refused == [True]
    assert _FakeSerial.opened == ["/dev/ttyACM7"]
    assert c.connected


def test_failed_autodetect_clears_the_flag_and_keeps_its_log(conn, monkeypatch):
    c, reactor, log = conn

    def resolve(ace_index, baud, settle=6.0, logger=None):
        logger.info("ACE autodetect: /dev/ttyACM3 reports id 2")
        return None

    monkeypatch.setattr(AFC_ACE, "resolve_ace_port_by_index", resolve)
    with pytest.raises(ACESerialError, match="no unit reporting id 1"):
        c.connect()
    assert not c.connected
    assert _FakeSerial.opened == []
    assert any("reports id 2" in l[1] for l in log.lines)

    # the next attempt is not refused as "already in progress"
    monkeypatch.setattr(AFC_ACE, "resolve_ace_port_by_index",
                        lambda *a, **k: "/dev/ttyACM3")
    c.connect()
    assert c.connected


def test_ace2_connect_resolves_off_reactor(monkeypatch):
    from extras import AFC_ACE2
    _FakeSerial.opened = []
    monkeypatch.setitem(sys.modules, "serial",
                        types.SimpleNamespace(Serial=_FakeSerial))
    reactor = ThreadedReactor()
    log = _RecLog()
    c = AFC_ACE2.ACE2Connection(reactor, "auto", logger=log, ace_index=1)
    monkeypatch.setattr(c, "send_command", lambda *a, **k: {"model": "x"})
    monkeypatch.setattr(c, "_start_heartbeat", lambda: None)
    main = threading.get_ident()
    probe = {}

    def resolve(ace_index, baud, ace_uid=None, settle=6.0, logger=None):
        probe["thread"] = threading.get_ident()
        logger.info("ACE2 autodetect: /dev/ttyACM2 -> uid (1, 2, 3)")
        time.sleep(0.3)
        return "/dev/ttyACM2"

    monkeypatch.setattr(AFC_ACE2, "resolve_ace2_port", resolve)
    c.connect()

    assert c.connected
    assert _FakeSerial.opened == ["/dev/ttyACM2"]
    assert probe["thread"] != main
    assert reactor.ticks >= 10
    assert any("-> uid" in l[1] and l[2] == main for l in log.lines)


# ── PREP waits for the first connect ───────────────────────────────────────
# Printer 1, 20:11: with the autodetect off the reactor, PREP reached the
# lanes at 20:11:20-21 while the probe ran, and every ACE 2 lane read ACE NOT
# CONNECTED; the connect landed at 20:11:24.

class _PauseReactor:
    def __init__(self, on_pause=None):
        self.t = 100.0
        self.on_pause = on_pause
        self.pauses = 0

    def monotonic(self):
        return self.t

    def pause(self, waketime):
        self.pauses += 1
        self.t = max(self.t, waketime)
        if self.on_pause is not None:
            self.on_pause()
        return self.t


def _unit(reactor):
    u = AFC_ACE.afcACE.__new__(AFC_ACE.afcACE)
    u.name = "Ace2_1"
    u.afc = types.SimpleNamespace(reactor=reactor)
    u.logger = _RecLog()
    return u


def test_prep_waits_until_the_first_connect_has_finished():
    reactor = _PauseReactor()
    u = _unit(reactor)
    u._first_connect_pending = True

    def land():
        if reactor.pauses == 30:          # 3 s of probing
            u._first_connect_pending = False
    reactor.on_pause = land
    u._wait_first_connect()
    assert reactor.pauses == 30
    assert not u._first_connect_pending


def test_prep_does_not_wait_with_no_connect_pending():
    reactor = _PauseReactor()
    u = _unit(reactor)
    u._wait_first_connect()
    u._first_connect_pending = False
    u._wait_first_connect()
    assert reactor.pauses == 0


def test_prep_gives_up_on_a_connect_that_never_returns():
    reactor = _PauseReactor()
    u = _unit(reactor)
    u._first_connect_pending = True
    u._wait_first_connect()
    assert reactor.t >= 100.0 + AFC_ACE.AUTODETECT_WAIT_MAX


@pytest.mark.parametrize("ok", [True, False], ids=["connected", "failed"])
def test_the_first_attempt_clears_the_wait_either_way(ok):
    reactor = _PauseReactor()
    u = _unit(reactor)
    u._first_connect_pending = True
    seen = []

    class _Conn:
        connected = False

        def connect(self):
            seen.append(u._first_connect_pending)
            if not ok:
                raise ACESerialError("no unit reporting id 1")
            self.connected = True

    u._make_connection = lambda *a: _Conn()
    u._create_serial_logger = lambda: None
    u.serial_port, u.baud_rate = "auto", 115200
    u._CONNECT_MAX_RETRIES = 1
    u._CONNECT_RETRY_DELAYS = [0.0]
    try:
        u._deferred_ace_connect(0.0)
    except Exception:
        pass                               # the rest of the connect is not this
    assert seen == [True]
    assert u._first_connect_pending is False
