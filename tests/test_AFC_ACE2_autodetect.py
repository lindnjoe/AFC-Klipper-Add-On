"""
Tests for the ACE 2 serial_port: auto autodetect (extras/AFC_ACE2.py).

Each ACE 2 is its own USB device (a CH34x adapter). ACE 2 get_info has no
per-unit id, so a section binds by the STM32 UID from discover_device: probe
each CH34x device, and either match ace_uid (pin a specific box) or order by
USB topology and take the ace_index-th (cable order). These tests cover the
discover-response UID parser and the id->port resolver (probe/scan mocked).
"""

from __future__ import annotations

import struct

import pytest

from extras import AFC_ACE2
from extras.AFC_ACE2 import (
    _ace2_extract_uid,
    resolve_ace2_port,
    PREAMBLE,
    END_MARKER,
    FLAG_RESPONSE,
    Cmd,
    crc16_kermit,
)
from extras.AFC_ACE import _ACE_CLAIMED_PORTS


def _pb_varint(num: int, value: int) -> bytes:
    out = bytearray([(num << 3) | 0])   # wire type 0 = varint
    v = value
    while True:
        b = v & 0x7F
        v >>= 7
        out.append(b | (0x80 if v else 0))
        if not v:
            break
    return bytes(out)


def _discover_frame(uid):
    """A framed discover RESPONSE carrying (uid1,uid2,uid3), as the unit sends."""
    payload = b"".join(_pb_varint(i + 1, u) for i, u in enumerate(uid))
    inner = bytes([FLAG_RESPONSE, 0, 0, Cmd.DISCOVER_DEVICE, len(payload)]) + payload
    crc = crc16_kermit(inner)
    return (bytes(PREAMBLE) + inner
            + bytes([crc & 0xFF, (crc >> 8) & 0xFF, END_MARKER]))


# ── UID parser ──────────────────────────────────────────────────────────────

def test_extract_uid_whole_frame():
    uid = (2403054933, 129011976, 892745291)   # the real unit from the log
    assert _ace2_extract_uid(bytearray(_discover_frame(uid))) == uid


def test_extract_uid_tolerates_garbage_prefix():
    uid = (1, 2, 3)
    assert _ace2_extract_uid(bytearray(b"\x00\x11" + _discover_frame(uid))) == uid


def test_extract_uid_truncated_none():
    assert _ace2_extract_uid(bytearray(_discover_frame((5, 6, 7))[:6])) is None


def test_extract_uid_ignores_request_frame():
    # a REQUEST (not FLAG_RESPONSE) must not be mistaken for a unit reply
    uid = (9, 9, 9)
    payload = b"".join(_pb_varint(i + 1, u) for i, u in enumerate(uid))
    inner = bytes([0x00, 0, 0, Cmd.DISCOVER_DEVICE, len(payload)]) + payload
    crc = crc16_kermit(inner)
    frame = bytes(PREAMBLE) + inner + bytes([crc & 0xFF, (crc >> 8) & 0xFF, END_MARKER])
    assert _ace2_extract_uid(bytearray(frame)) is None


# ── resolver ────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def _clean_claims():
    _ACE_CLAIMED_PORTS.clear()
    yield
    _ACE_CLAIMED_PORTS.clear()


def _mock_bus(monkeypatch, uid_by_port, topo=None):
    monkeypatch.setattr(AFC_ACE2, "_ace2_scan_candidates",
                        lambda: list(uid_by_port.keys()))
    monkeypatch.setattr(AFC_ACE2, "probe_ace2_uid",
                        lambda port, baud, timeout=1.5: uid_by_port.get(port))
    # deterministic topology order: use the provided map, else the tty name
    monkeypatch.setattr(AFC_ACE2, "_ace2_topology_key",
                        lambda p: (topo or {}).get(p, p))


def test_resolve_by_topology_cable_order(monkeypatch):
    _mock_bus(monkeypatch,
              {"/dev/ttyACM9": (11, 0, 0), "/dev/ttyACM2": (22, 0, 0)},
              topo={"/dev/ttyACM2": "usb-0:2.2", "/dev/ttyACM9": "usb-0:2.4"})
    # ace_index 1 = first in topology (2.2), regardless of tty number/uid
    assert resolve_ace2_port(1, 230400, settle=0.0) == "/dev/ttyACM2"
    assert resolve_ace2_port(2, 230400, settle=0.0) == "/dev/ttyACM9"
    assert resolve_ace2_port(3, 230400, settle=0.0) is None


def test_resolve_by_ace_uid_pin(monkeypatch):
    _mock_bus(monkeypatch,
              {"/dev/ttyACM2": (11, 0, 0), "/dev/ttyACM9": (22, 33, 44)})
    # pin to the box with a specific UID, ignoring order
    assert resolve_ace2_port(1, 230400, ace_uid=(22, 33, 44), settle=0.0) == "/dev/ttyACM9"
    assert resolve_ace2_port(1, 230400, ace_uid=(1, 2, 3), settle=0.0) is None


def test_resolve_skips_claimed(monkeypatch):
    _mock_bus(monkeypatch,
              {"/dev/ttyACM2": (11, 0, 0), "/dev/ttyACM9": (22, 0, 0)},
              topo={"/dev/ttyACM2": "a", "/dev/ttyACM9": "b"})
    _ACE_CLAIMED_PORTS.add("/dev/ttyACM2")     # taken by another unit
    # ace_index 1 now resolves to the only unclaimed unit
    assert resolve_ace2_port(1, 230400, settle=0.0) == "/dev/ttyACM9"


def test_resolve_ignores_non_ace2(monkeypatch):
    _mock_bus(monkeypatch,
              {"/dev/ttyACM0": None, "/dev/ttyACM3": (7, 0, 0)},
              topo={"/dev/ttyACM3": "a"})
    assert resolve_ace2_port(1, 230400, settle=0.0) == "/dev/ttyACM3"
