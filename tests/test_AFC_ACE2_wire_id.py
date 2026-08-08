"""
Regression test for the ACE2 16-bit wire-id wrap bug (extras/AFC_ACE2.py).

The V2 binary protocol carries a 16-bit request sequence id (encode_request
masks it to & 0xFFFF), and the unit echoes back only those low 16 bits. The
driver must key its pending/async request tracking by that same masked value.

Keying send_command / send_command_async by the full _next_request_id breaks
once the counter passes 65535: the echoed id comes back masked (e.g. 86480 ->
20944) and matches nothing, so every synchronous command times out and every
async reply logs "unknown request id". Observed live on hardware as:
TX id=86480 get_status / RX id=20944 / "unknown request id=20944".
"""

from __future__ import annotations

import collections

import pytest

from extras.AFC_ACE2 import ACE2Connection, encode_request
from extras.AFC_ACE2 import PREAMBLE, FLAG_REQUEST
from extras.AFC_ACE import ACETimeoutError


WRAPPED = 86480          # a real post-wrap counter value seen on hardware
MASKED = WRAPPED & 0xFFFF  # 20944


# ── Fakes ─────────────────────────────────────────────────────────────────────

class _FakeSerial:
    def __init__(self):
        self.frames = []
        self.on_write = None

    def write(self, frame):
        self.frames.append(bytes(frame))
        if self.on_write is not None:
            self.on_write(bytes(frame))

    def flush(self):
        pass


class _FakeCompletion:
    def __init__(self):
        self._value = None

    def complete(self, value):
        self._value = value

    def wait(self, deadline):
        return self._value


class _FakeReactor:
    def monotonic(self):
        return 0.0

    def completion(self):
        return _FakeCompletion()


class _FakeLogger:
    def __init__(self):
        self.debug_lines = []

    def debug(self, msg, *a, **k):
        self.debug_lines.append(msg)

    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


def _make_conn(next_id):
    conn = ACE2Connection.__new__(ACE2Connection)
    conn._connected = True
    conn._serial = _FakeSerial()
    conn._reactor = _FakeReactor()
    conn._logger = _FakeLogger()
    conn._pending = {}
    conn._pending_cmd = {}
    conn._async_ids = collections.deque(maxlen=256)
    conn._next_request_id = next_id
    conn.status_callback = None
    conn._track_timeout = lambda: None
    conn._track_unsolicited = lambda: None
    return conn


def _wire_seq(frame):
    """Extract the 16-bit sequence id from an encoded request frame."""
    inner = frame[len(PREAMBLE):]
    assert inner[0] == FLAG_REQUEST
    return inner[1] | (inner[2] << 8)


# ── The wire id itself ────────────────────────────────────────────────────────

def test_encode_masks_id_to_16_bits():
    frame = encode_request(WRAPPED, "get_status", {})
    assert _wire_seq(frame) == MASKED


# ── send_command round trip after the counter wraps ───────────────────────────

def test_send_command_completes_after_wrap():
    conn = _make_conn(next_id=WRAPPED)

    def _echo(frame):
        # The unit replies with exactly the 16-bit id it received.
        rid = _wire_seq(frame)
        conn._handle_response({"id": rid, "code": 0, "result": {"ok": 1}})

    conn._serial.on_write = _echo

    result = conn.send_command("get_status", timeout=1.0)

    # The pending completion matched the echoed (masked) id and completed.
    assert result == {"ok": 1}
    assert conn._pending == {}  # popped after completion
    assert _wire_seq(conn._serial.frames[0]) == MASKED


def test_reply_with_mismatched_opcode_is_dropped():
    # A stale reply landing on a reused 16-bit id but carrying a DIFFERENT
    # opcode must not complete the pending request (it would hand the caller
    # another command's data). It's dropped; the request then times out.
    conn = _make_conn(next_id=WRAPPED)

    def _wrong_opcode(frame):
        rid = _wire_seq(frame)
        # get_status is opcode 6; reply tagged as opcode 8 (feed) must be rejected.
        conn._handle_response({"id": rid, "_cmd": 8, "code": 0,
                               "result": {"stale": 1}})

    conn._serial.on_write = _wrong_opcode
    with pytest.raises(ACETimeoutError):
        conn.send_command("get_status", timeout=0.0)
    assert conn._pending == {} and conn._pending_cmd == {}


def test_reply_with_matching_opcode_completes():
    conn = _make_conn(next_id=WRAPPED)

    def _right_opcode(frame):
        rid = _wire_seq(frame)
        conn._handle_response({"id": rid, "_cmd": 6, "code": 0,
                               "result": {"ok": 1}})

    conn._serial.on_write = _right_opcode
    assert conn.send_command("get_status", timeout=1.0) == {"ok": 1}


def test_send_command_pending_keyed_by_masked_id():
    conn = _make_conn(next_id=WRAPPED)
    captured = {}

    def _capture(frame):
        captured["keys"] = list(conn._pending.keys())

    conn._serial.on_write = _capture
    # No echo -> the command times out; we only care that the pending key was
    # the masked id at write time.
    with pytest.raises(ACETimeoutError):
        conn.send_command("get_status", timeout=0.0)

    assert captured["keys"] == [MASKED]  # NOT [86480]


# ── send_command_async round trip after wrap ──────────────────────────────────

def test_async_id_tracked_masked_and_recognised():
    conn = _make_conn(next_id=WRAPPED)

    conn.send_command_async("get_status")

    assert list(conn._async_ids) == [MASKED]
    assert _wire_seq(conn._serial.frames[0]) == MASKED

    # The echoed reply with the masked id is recognised (removed from
    # _async_ids), NOT logged as an unknown request.
    conn._handle_response({"id": MASKED, "code": 0, "result": {"status": "ready"}})

    assert list(conn._async_ids) == []
    assert not any("unknown request" in m for m in conn._logger.debug_lines)


def test_pre_wrap_ids_unaffected():
    """Below 65536 the masking is a no-op — behaviour is unchanged."""
    conn = _make_conn(next_id=5)
    conn.send_command_async("get_status")
    assert list(conn._async_ids) == [5]
    assert _wire_seq(conn._serial.frames[0]) == 5
