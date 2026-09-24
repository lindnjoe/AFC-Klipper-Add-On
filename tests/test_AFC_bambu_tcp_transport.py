# Tests for the opt-in TCP transport (TcpPort).
#
# The bridge's reader and writer threads were written against pyserial's
# semantics, and those semantics are load-bearing in ways that are easy to get
# subtly wrong in a socket:
#
#   read()  returns b"" on TIMEOUT   -- a quiet link is not an error
#   read()  RAISES at end of stream  -- this is what triggers reconnect;
#                                       returning b"" here spins the reader
#   write() raises _SerialTimeout    -- lands in the timed-out-write path,
#                                       which must NOT take the link down
#
# So these run against a real loopback socket rather than a mock: the point is
# the behaviour at the boundary, and a mock would assert our own assumptions
# back at us.
from __future__ import annotations

import socket
import threading
import time

import pytest

from extras.AFC_BambuAMS_bridge import TcpPort, _SerialTimeout


class _Server:
    """A loopback listener that hands back one accepted connection."""

    def __init__(self, accept: bool = True) -> None:
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self.host, self.port = self._srv.getsockname()
        self.conn: socket.socket | None = None
        self._t: threading.Thread | None = None
        if accept:
            self._t = threading.Thread(target=self._accept, daemon=True)
            self._t.start()

    def _accept(self) -> None:
        try:
            self.conn, _ = self._srv.accept()
        except OSError:
            pass

    def wait_conn(self, timeout: float = 3.0) -> socket.socket:
        end = time.monotonic() + timeout
        while self.conn is None and time.monotonic() < end:
            time.sleep(0.01)
        assert self.conn is not None, "client never connected"
        return self.conn

    def close(self) -> None:
        for s in (self.conn, self._srv):
            try:
                if s is not None:
                    s.close()
            except OSError:
                pass


@pytest.fixture
def server():
    s = _Server()
    yield s
    s.close()


@pytest.mark.parametrize("spec, want", [
    ("tcp://192.168.1.50:8888", ("192.168.1.50", 8888)),
    ("TCP://bridgebox.local:9000", ("bridgebox.local", 9000)),
    ("tcp://192.168.1.50", ("192.168.1.50", TcpPort.DEFAULT_PORT)),
    ("tcp://192.168.1.50:8888/", ("192.168.1.50", 8888)),
    ("192.168.1.50:8888", ("192.168.1.50", 8888)),
    ("tcp://[fe80::1]:8888", ("fe80::1", 8888)),
    ("tcp://[fe80::1]", ("fe80::1", TcpPort.DEFAULT_PORT)),
    ("tcp://fe80::1:2:3", ("fe80::1:2:3", TcpPort.DEFAULT_PORT)),
])
def test_parse_accepts_the_forms_people_will_write(spec, want):
    assert TcpPort.parse(spec) == want


@pytest.mark.parametrize("spec", ["tcp://", "tcp://:8888", "tcp://host:nope"])
def test_parse_rejects_nonsense(spec):
    with pytest.raises(ValueError):
        TcpPort.parse(spec)


def test_read_returns_what_was_sent(server):
    p = TcpPort(server.host, server.port)
    server.wait_conn().sendall(b'{"evt":"ack"}\n')
    got = b""
    end = time.monotonic() + 3.0
    while b"\n" not in got and time.monotonic() < end:
        got += p.read(64)
    assert got == b'{"evt":"ack"}\n'
    p.close()


def test_read_returns_empty_on_timeout_not_an_error(server):
    # A bridge with nothing to say must not look like a broken one.
    p = TcpPort(server.host, server.port, timeout=0.05)
    server.wait_conn()
    t0 = time.monotonic()
    assert p.read(64) == b""
    assert time.monotonic() - t0 < 1.0        # it timed out, it did not hang
    p.close()


def test_read_raises_at_end_of_stream(server):
    # THE RECONNECT TRIGGER. The reader drops the port on an exception here;
    # a b"" would read as "quiet" and spin forever against a dead bridge.
    p = TcpPort(server.host, server.port, timeout=0.05)
    server.wait_conn().close()
    with pytest.raises(OSError):
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            p.read(64)
    p.close()


def test_write_delivers_whole_lines(server):
    p = TcpPort(server.host, server.port)
    conn = server.wait_conn()
    assert p.write(b'{"cmd":"prime","slot":2}\n') == 25
    conn.settimeout(3.0)
    assert conn.recv(64) == b'{"cmd":"prime","slot":2}\n'
    p.close()


def test_write_raises_serial_timeout_when_the_far_end_stops_reading(server):
    # A far end that never reads must produce the SAME exception a stalled
    # Pico does, so it lands in "bridge busy, write of ... timed out" instead
    # of tearing the link down.
    p = TcpPort(server.host, server.port, write_timeout=0.2)
    server.wait_conn()                        # accepted, then never read from
    with pytest.raises(_SerialTimeout):
        end = time.monotonic() + 10.0
        while time.monotonic() < end:         # fill the socket buffers
            p.write(b"x" * 65536)
    p.close()


def test_connect_failure_raises_so_the_reader_backs_off(server):
    # The reader's reconnect loop relies on the factory RAISING while the
    # bridge is absent; a port object that pretends to be open would starve it.
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.bind(("127.0.0.1", 0))
    host, port = srv.getsockname()
    srv.close()                               # nothing is listening now
    with pytest.raises(OSError):
        TcpPort(host, port, connect_timeout=1.0)


def test_close_is_idempotent(server):
    p = TcpPort(server.host, server.port)
    server.wait_conn()
    p.close()
    p.close()


# ── End to end: the REAL bridge, its real threads, over a real socket ──────────
#
# The transport tests above cover the boundary. This covers the thing that
# actually matters: that BambuBridge -- whose reader and writer are threads
# written for pyserial -- runs unmodified against TcpPort, parses frames off
# the wire, and puts commands back on it.

class _Reactor:
    """Minimal reactor: the bridge only needs monotonic + async callbacks."""

    def __init__(self):
        self._now = 100.0
        self.async_cbs = []

    def monotonic(self):
        return self._now

    def register_async_callback(self, cb):
        self.async_cbs.append(cb)

    def run_pending(self):
        cbs, self.async_cbs = self.async_cbs, []
        for cb in cbs:
            cb(0.0)


class _Logger:
    def __init__(self):
        self.msgs = []

    def _rec(self, m, *a, **k):
        self.msgs.append(str(m))

    info = debug = warning = error = raw = _rec


def test_bridge_runs_end_to_end_over_tcp(server):
    from extras.AFC_BambuAMS_bridge import BambuBridge

    reactor, logger = _Reactor(), _Logger()
    bridge = BambuBridge(
        lambda: TcpPort(server.host, server.port, timeout=0.05), reactor,
        logger)
    seen = []
    bridge.add_listener(lambda st: seen.append(st))
    bridge.start()
    try:
        conn = server.wait_conn()
        conn.settimeout(3.0)

        # host -> bridge: a command written by the writer thread
        bridge.send({"cmd": "status"})
        got = b""
        end = time.monotonic() + 3.0
        while b"\n" not in got and time.monotonic() < end:
            got += conn.recv(256)
        assert b'"cmd": "status"' in got or b'"cmd":"status"' in got

        # bridge -> host: a frame split and dispatched by the reader thread.
        # Sent in two writes with the newline in the SECOND, so the test also
        # proves the reader reassembles across TCP segment boundaries -- a
        # stream can split anywhere, unlike the CDC reads it was written for.
        conn.sendall(b'{"evt":"ack","cmd":"stat')
        time.sleep(0.1)
        conn.sendall(b'us","slot":-1}\n')

        # handle_line runs ON the reader thread (only some paths hop to the
        # reactor), so the evidence it arrived is the line the ack handler
        # logs, not a queued callback.
        end = time.monotonic() + 3.0
        while not any("ack" in m for m in logger.msgs) and time.monotonic() < end:
            time.sleep(0.02)
        assert any("ack" in m for m in logger.msgs), (
            f"reader never dispatched the frame; log={logger.msgs[-5:]}")
    finally:
        bridge._run = False
        if hasattr(bridge, "stop"):
            bridge.stop()


def test_start_defers_a_failed_first_connect_and_the_reader_gets_there(server):
    # A network bridge that is not up yet at Klipper start must NOT fail the
    # unit: the reader's backoff loop is exactly the thing that handles it,
    # and raising here would need a Klipper restart to retry a link that comes
    # good seconds later. Proven by starting with the listener closed, then
    # opening it and watching a frame arrive.
    from extras.AFC_BambuAMS_bridge import BambuBridge

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    host, port = srv.getsockname()
    srv.close()                                   # nothing listening yet

    reactor, logger = _Reactor(), _Logger()
    bridge = BambuBridge(lambda: TcpPort(host, port, timeout=0.05,
                                         connect_timeout=0.5),
                         reactor, logger)
    bridge.start(defer_open=True)                 # must not raise
    try:
        assert bridge._serial is None
        later = _Server()                         # now bring one up...
        later._srv.close()                        # ...on the SAME port
        later._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        later._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        later._srv.bind((host, port))
        later._srv.listen(1)
        t = threading.Thread(target=later._accept, daemon=True)
        t.start()
        conn = later.wait_conn(timeout=15.0)      # the reader's backoff is <=5s
        conn.sendall(b'{"evt":"ack","cmd":"late","slot":0}\n')
        end = time.monotonic() + 5.0
        while not any("late" in m for m in logger.msgs) and time.monotonic() < end:
            time.sleep(0.02)
        assert any("late" in m for m in logger.msgs), logger.msgs[-5:]
        later.close()
    finally:
        bridge._run = False
        bridge.stop()


def test_start_still_raises_for_a_missing_usb_port():
    # The other half of the same rule: a device path that is not there is a
    # configuration fault, not a bridge that is booting, and must still be
    # loud.
    from extras.AFC_BambuAMS_bridge import BambuBridge

    def boom():
        raise OSError("no such device: /dev/serial/by-id/nope")

    bridge = BambuBridge(boom, _Reactor(), _Logger())
    with pytest.raises(OSError):
        bridge.start()


# ── Link loss: what is in place for it going away and coming back ─────────────

def test_down_since_and_is_connected_track_the_outage(server):
    from extras.AFC_BambuAMS_bridge import BambuBridge

    bridge = BambuBridge(lambda: TcpPort(server.host, server.port, timeout=0.05),
                         _Reactor(), _Logger())
    bridge.start()
    try:
        assert bridge.is_connected() is True
        assert bridge.down_since() is None
        # The LISTENER has to go too, not just the connection: with it still
        # up the reader reconnects inside its first backoff and the outage is
        # over before it can be observed -- which is the behaviour we want,
        # and the reason this has to kill the whole far end to test the other.
        server.wait_conn().close()
        server._srv.close()
        end = time.monotonic() + 8.0
        while bridge.is_connected() and time.monotonic() < end:
            time.sleep(0.05)
        assert bridge.is_connected() is False
        assert bridge.down_since() is not None    # and it is timed
    finally:
        bridge._run = False
        bridge.stop()


def test_first_dropped_command_says_so_once_per_outage():
    # Silence was the bug: every command issued while the port was gone
    # vanished with no log at all, so a dead link read as an idle one.
    from extras.AFC_BambuAMS_bridge import BambuBridge

    logger = _Logger()
    bridge = BambuBridge(lambda: None, _Reactor(), logger)
    bridge._serial = None
    bridge._down_t = time.monotonic()
    bridge._down_epoch = 1

    for _ in range(5):
        bridge.send({"cmd": "status"})
    said = [m for m in logger.msgs if "link is down" in m]
    assert len(said) == 1, said                    # once, not five times
    assert "status" in said[0]

    bridge._down_epoch = 2                         # a NEW outage speaks again
    bridge.send({"cmd": "feed"})
    assert len([m for m in logger.msgs if "link is down" in m]) == 2


def test_silence_watchdog_reports_and_then_holds_its_tongue():
    from extras.AFC_BambuAMS_bridge import BambuBridge

    logger = _Logger()
    bridge = BambuBridge(lambda: None, _Reactor(), logger)
    bridge._serial = None
    bridge._down_t = time.monotonic() - (BambuBridge.QUIET_WARN_S + 1)

    bridge._check_quiet()
    hits = [m for m in logger.msgs if "unreachable for" in m]
    assert len(hits) == 1, logger.msgs
    assert "commands are being dropped" in hits[0]

    bridge._check_quiet()                          # not again, not yet
    assert len([m for m in logger.msgs if "unreachable for" in m]) == 1

    # A link that is UP but saying nothing is its own case.
    logger.msgs.clear()
    bridge._serial = object()
    bridge._down_t = None
    bridge._silence_logged_t = None
    bridge._last_frame_t = time.monotonic() - (BambuBridge.QUIET_WARN_S + 1)
    bridge._check_quiet()
    quiet = [m for m in logger.msgs if "silent for" in m]
    assert len(quiet) == 1, logger.msgs
    assert "commands are being dropped" not in quiet[0]


def test_a_quiet_healthy_link_says_nothing():
    from extras.AFC_BambuAMS_bridge import BambuBridge

    logger = _Logger()
    bridge = BambuBridge(lambda: None, _Reactor(), logger)
    bridge._serial = object()
    bridge._down_t = None
    bridge._last_frame_t = time.monotonic()        # a frame just arrived
    bridge._check_quiet()
    assert not [m for m in logger.msgs if "silent" in m or "unreachable" in m]


def test_a_successful_start_clears_the_never_connected_stamp(server):
    # _down_t is stamped at construction ("never connected yet") and start()
    # sets the port directly rather than going through the reader's reconnect
    # path. Without clearing it there, the FIRST outage reports the age of the
    # object instead of its own -- seen live as bridge_down_for 218.2 for a
    # link that had been down 8 seconds, which also defeats the load's grace,
    # since a blip measures as minutes.
    from extras.AFC_BambuAMS_bridge import BambuBridge

    bridge = BambuBridge(lambda: TcpPort(server.host, server.port, timeout=0.05),
                         _Reactor(), _Logger())
    time.sleep(0.3)                                # let the construction stamp age
    bridge.start()
    try:
        assert bridge.down_since() is None
        assert bridge._down_t is None              # the stamp itself is gone
        server.wait_conn().close()
        server._srv.close()
        end = time.monotonic() + 8.0
        while bridge.is_connected() and time.monotonic() < end:
            time.sleep(0.05)
        assert bridge.is_connected() is False
        down_for = time.monotonic() - bridge.down_since()
        assert down_for < 3.0, f"outage reported as {down_for:.1f}s old"
    finally:
        bridge._run = False
        bridge.stop()


def test_a_second_outage_gets_its_own_stamp_and_epoch():
    from extras.AFC_BambuAMS_bridge import BambuBridge

    bridge = BambuBridge(lambda: None, _Reactor(), _Logger())
    bridge._serial = object()
    bridge._down_t = None
    bridge._drop_port()
    first_t, first_epoch = bridge._down_t, bridge._down_epoch
    assert first_t is not None

    time.sleep(0.05)
    bridge._serial = object()                      # link came back...
    bridge._drop_port()                            # ...and died again
    assert bridge._down_t > first_t                # a NEW outage, newly timed
    assert bridge._down_epoch == first_epoch + 1   # so send() speaks again


def test_the_watchdog_fires_from_the_READER_while_the_link_is_down():
    # The gap the direct-call test above could not see. A disconnected reader
    # continues from the reconnect branch on every pass and never reaches the
    # read path, so a watchdog called only from the read path can never fire
    # in the one state it exists for. Drive the real reader loop against a
    # factory that always fails, and require the warning to appear.
    from extras.AFC_BambuAMS_bridge import BambuBridge

    logger = _Logger()

    def never():
        raise OSError("nothing is listening")

    bridge = BambuBridge(never, _Reactor(), logger)
    bridge.QUIET_WARN_S = 0.2                      # don't wait 45s in a test
    bridge.start(defer_open=True)
    try:
        end = time.monotonic() + 10.0
        while (not any("unreachable for" in m for m in logger.msgs)
               and time.monotonic() < end):
            time.sleep(0.1)
        hits = [m for m in logger.msgs if "unreachable for" in m]
        assert hits, f"the reader never reported the outage; log={logger.msgs[-6:]}"
        assert "commands are being dropped" in hits[0]
    finally:
        bridge._run = False
        bridge.stop()


# ── Printer 1's link: timed-out writes and gaps over TCP ──────────────────────

class _KwLogger:
    """Records each line with the keyword arguments it was logged with, so a
    test can tell a file-only line (only_debug=True) from a console one."""

    def __init__(self):
        self.calls = []

    def _rec(self, m, *a, **k):
        self.calls.append((str(m), k))

    info = debug = warning = error = raw = _rec


def test_a_timed_out_tcp_write_is_named_and_keeps_the_link(server):
    import queue as _q
    from extras.AFC_BambuAMS_bridge import BambuBridge

    p = TcpPort(server.host, server.port, write_timeout=0.2)
    server.wait_conn()                        # accepted, then never read from
    with pytest.raises(_SerialTimeout):
        end = time.monotonic() + 10.0
        while time.monotonic() < end:         # fill the socket buffers
            p.write(b"x" * 65536)
    logger = _KwLogger()
    bridge = BambuBridge(lambda: p, _Reactor(), logger)
    bridge._serial = p                        # attached; no threads started
    bridge.send({"cmd": "chain"})
    real_get = bridge._wq.get

    def get(timeout=None):
        if bridge._wq.qsize() == 0:
            bridge._run = False
            raise _q.Empty
        return real_get(timeout=timeout)
    bridge._wq.get = get
    bridge._run = True
    try:
        BambuBridge._writer(bridge)           # the real loop, on this thread
        assert [m for m, _k in logger.calls if "write of" in m] == [
            "AFC bambu: bridge busy, write of 'chain' timed out after 0.2 s "
            "(may still be delivered)"]
        assert bridge._serial is p, "a busy far end must not drop the link"
    finally:
        p.close()


def test_a_gap_on_a_tcp_link_is_logged_once_and_file_only(server, monkeypatch):
    # WiFi gaps are ordinary on printer 1, so the line must stay out of its
    # console (only_debug) and say each gap once, when it ends.
    import extras.AFC_BambuAMS_bridge as br
    from extras.AFC_BambuAMS_bridge import BambuBridge

    monkeypatch.setattr(br, "SILENCE_LOG_S", 0.3)   # not 2.5 s in a test
    logger = _KwLogger()
    bridge = BambuBridge(
        lambda: TcpPort(server.host, server.port, timeout=0.05), _Reactor(),
        logger)
    bridge.start()
    try:
        conn = server.wait_conn()
        conn.sendall(b'{"evt":"hb"}\n')        # the first frame: never a gap
        time.sleep(0.05)
        conn.sendall(b'{"evt":"hb"}\n')
        time.sleep(0.8)
        conn.sendall(b'{"evt":"hb"}\n')        # ends a ~0.8 s gap
        end = time.monotonic() + 3.0
        while (not any("was silent" in m for m, _k in logger.calls)
               and time.monotonic() < end):
            time.sleep(0.02)
        time.sleep(0.2)
        hits = [(m, k) for m, k in logger.calls if "was silent" in m]
        assert len(hits) == 1, logger.calls[-6:]
        assert hits[0][1].get("only_debug") is True
        assert hits[0][0].endswith("0 write(s) timed out meanwhile")
    finally:
        bridge._run = False
        bridge.stop()
