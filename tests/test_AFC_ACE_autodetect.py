"""
Tests for the V1 ACE serial_port: auto autodetect (extras/AFC_ACE.py).

Chained ACEs each get their own USB serial device whose /dev/ttyACM* number
shuffles on the firmware's ~3.5s watchdog re-enumeration, so a fixed port is
useless. The firmware self-reports a stable per-unit id in get_info
({'result': {'id': N}}) that counts up by cable order. `serial_port: auto`
binds a section to its unit by matching ace_index to that id: scan the Anycubic
serial devices, probe each with a one-shot get_info, and take the matching one.

These tests cover the frame parser and the id->port resolver (with the probe
and device scan mocked, so no real hardware is needed).
"""

from __future__ import annotations

import json
import struct

import pytest

from extras import AFC_ACE
from extras.AFC_ACE import (
    _ace_extract_get_info_id,
    resolve_ace_port_by_index,
    _ACE_CLAIMED_PORTS,
    FRAME_HEADER,
    FRAME_FOOTER,
    crc16_ccitt_reflected,
)


def _get_info_frame(unit_id: int) -> bytes:
    """The exact framed get_info response a real ACE sends for a given id."""
    payload = json.dumps(
        {"id": 0, "code": 0,
         "result": {"id": unit_id, "slots": 4,
                    "model": "Anycubic Color Engine Pro",
                    "firmware": "V1.3.856"},
         "msg": "success"}).encode("utf-8")
    return (FRAME_HEADER + struct.pack("<H", len(payload)) + payload
            + struct.pack("<H", crc16_ccitt_reflected(payload)) + FRAME_FOOTER)


# ── frame parser ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("unit_id", [1, 2, 3, 4])
def test_extract_id_whole_frame(unit_id):
    assert _ace_extract_get_info_id(_get_info_frame(unit_id)) == unit_id


def test_extract_id_tolerates_garbage_prefix():
    # a partial previous frame / noise ahead of the real header
    assert _ace_extract_get_info_id(b"\x00\x11ff" + _get_info_frame(2)) == 2


def test_extract_id_truncated_returns_none():
    assert _ace_extract_get_info_id(_get_info_frame(1)[:9]) is None


def test_extract_id_no_result_returns_none():
    payload = b"{}"
    frame = (FRAME_HEADER + struct.pack("<H", len(payload)) + payload
             + struct.pack("<H", crc16_ccitt_reflected(payload)) + FRAME_FOOTER)
    assert _ace_extract_get_info_id(frame) is None


# ── resolver ────────────────────────────────────────────────────────────────

@pytest.fixture(autouse=True)
def _clean_claims():
    _ACE_CLAIMED_PORTS.clear()
    yield
    _ACE_CLAIMED_PORTS.clear()


def _mock_bus(monkeypatch, id_by_port):
    """Fake a set of ACE devices: {tty_path: reported_id}."""
    monkeypatch.setattr(AFC_ACE, "_ace_scan_candidates",
                        lambda: list(id_by_port.keys()))
    monkeypatch.setattr(AFC_ACE, "probe_ace_get_info_id",
                        lambda port, baud=115200, timeout=1.5:
                        id_by_port.get(port))


def test_resolve_binds_by_reported_id(monkeypatch):
    _mock_bus(monkeypatch, {"/dev/ttyACM5": 1, "/dev/ttyACM2": 2,
                            "/dev/ttyACM9": 3})
    # index 2 must bind to the device REPORTING id 2, not the 2nd tty by name
    assert resolve_ace_port_by_index(2, settle=0.0) == "/dev/ttyACM2"
    assert resolve_ace_port_by_index(1, settle=0.0) == "/dev/ttyACM5"
    assert resolve_ace_port_by_index(3, settle=0.0) == "/dev/ttyACM9"


def test_resolve_missing_index_returns_none(monkeypatch):
    _mock_bus(monkeypatch, {"/dev/ttyACM5": 1})
    assert resolve_ace_port_by_index(2, settle=0.0) is None


def test_resolve_skips_claimed_ports(monkeypatch):
    # ttyACM5 reports id 1 but is already claimed by another live unit; the
    # resolver must not probe/return it.
    _mock_bus(monkeypatch, {"/dev/ttyACM5": 1, "/dev/ttyACM7": 1})
    _ACE_CLAIMED_PORTS.add("/dev/ttyACM5")
    assert resolve_ace_port_by_index(1, settle=0.0) == "/dev/ttyACM7"


def test_resolve_ignores_non_ace_devices(monkeypatch):
    # a device that doesn't answer get_info (probe -> None) is skipped
    _mock_bus(monkeypatch, {"/dev/ttyACM0": None, "/dev/ttyACM3": 1})
    assert resolve_ace_port_by_index(1, settle=0.0) == "/dev/ttyACM3"
