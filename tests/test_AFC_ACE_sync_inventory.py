"""
Unit tests for the startup-prep RFID inventory sweep gating (extras/AFC_ACE.py
afcACE._sync_inventory, extras/AFC_ACE2.py afcACE2).

V1 ACE reads tags through the firmware (get_filament_info), so its startup prep
sweeps every slot with that command. The ACE 2 instead reads tags host-side
over the MFRC522 passthrough (AFC_ACE2_rfid), so it must NOT run the firmware
sweep at all — the _uses_firmware_rfid class flag gates it off.

Style: typed local fakes, full state verification, branch coverage.
"""

from __future__ import annotations

from extras.AFC_ACE import afcACE
from extras.AFC_ACE2 import afcACE2

from tests.ace_helpers import FakeLogger


class _FakeConn:
    """ACE serial connection fake that records each get_filament_info call and
    returns a canned per-slot payload."""

    def __init__(self, connected=True, payloads=None):
        self.connected = connected
        self._payloads = payloads or {}
        self.filament_calls = []

    def get_filament_info(self, slot):
        self.filament_calls.append(slot)
        return self._payloads.get(slot, {"index": slot})


def _make_v1(conn):
    unit = afcACE.__new__(afcACE)
    unit._ace = conn
    unit.name = "Ace_1"
    unit.logger = FakeLogger()
    unit._slot_inventory = [{} for _ in range(afcACE.SLOTS_PER_UNIT)]
    # _store_slot_rfid runs for real; FakeLogger absorbs its debug/info lines.
    return unit


def _make_v2(conn):
    unit = afcACE2.__new__(afcACE2)
    unit._ace = conn
    unit.name = "Ace2_1"
    unit.logger = FakeLogger()
    unit._slot_inventory = [{} for _ in range(afcACE2.SLOTS_PER_UNIT)]
    return unit


# ── class flag ────────────────────────────────────────────────────────────────

def test_v1_uses_firmware_rfid_true():
    assert afcACE._uses_firmware_rfid is True


def test_v2_uses_firmware_rfid_false():
    assert afcACE2._uses_firmware_rfid is False


# ── V1 sweeps the firmware ──────────────────────────────────────────────────────

def test_v1_sync_inventory_reads_every_slot():
    conn = _FakeConn(connected=True)
    unit = _make_v1(conn)

    unit._sync_inventory()

    assert conn.filament_calls == list(range(afcACE.SLOTS_PER_UNIT))


def test_v1_sync_inventory_stores_payload():
    conn = _FakeConn(
        connected=True,
        payloads={0: {"index": 0, "sku": "HPL19-107", "type": "PLA"}})
    unit = _make_v1(conn)

    unit._sync_inventory()

    assert unit._slot_inventory[0]["sku"] == "HPL19-107"
    assert unit._slot_inventory[0]["material"] == "PLA"


def test_v1_sync_inventory_skips_when_disconnected():
    conn = _FakeConn(connected=False)
    unit = _make_v1(conn)

    unit._sync_inventory()

    assert conn.filament_calls == []


def test_v1_sync_inventory_skips_when_no_conn():
    unit = _make_v1(_FakeConn())
    unit._ace = None

    unit._sync_inventory()  # must not raise


# ── V2 never touches the firmware ───────────────────────────────────────────────

def test_v2_sync_inventory_skips_firmware_even_when_connected():
    conn = _FakeConn(connected=True,
                     payloads={0: {"index": 0, "sku": "HPL19-107"}})
    unit = _make_v2(conn)

    unit._sync_inventory()

    # The whole point: no firmware get_filament_info sweep on the ACE 2.
    assert conn.filament_calls == []
    # And the slot cache is left untouched by the (skipped) sweep.
    assert unit._slot_inventory[0] == {}
