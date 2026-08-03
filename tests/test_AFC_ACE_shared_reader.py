"""
Tests for the shared-reader RFID ambiguity guard in extras/AFC_ACE.py /
AFC_ACE2.py. Two MFRC522 readers cover the ACE Pro 2's four slots (0/1 -> r0,
2/3 -> r1), so a STATIC per-slot read can return the paired slot's tag. The guard
skips applying such an ambiguous read to a lane at startup (only a spinning
insert read can disambiguate).
"""
from __future__ import annotations

from extras.AFC_ACE import afcACE
from extras.AFC_ACE2 import afcACE2


def _inv(cls):
    return [{} for _ in range(cls.SLOTS_PER_UNIT)]


def test_base_ace_has_no_shared_reader_sibling():
    unit = afcACE.__new__(afcACE)
    assert unit._reader_sibling_slot(0) is None
    unit._slot_inventory = _inv(afcACE)
    unit._slot_inventory[0] = {"sku": "X"}
    unit._slot_inventory[1] = {"sku": "X"}
    assert unit._shared_rfid_ambiguous(0) is False   # per-slot reader: never shared


def test_ace2_reader_sibling_pairs():
    unit = afcACE2.__new__(afcACE2)
    assert unit._reader_sibling_slot(0) == 1
    assert unit._reader_sibling_slot(1) == 0
    assert unit._reader_sibling_slot(2) == 3
    assert unit._reader_sibling_slot(3) == 2


def test_ace2_ambiguous_when_sibling_reports_same_sku():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[0] = {"sku": "HPL19-107"}
    unit._slot_inventory[1] = {"sku": "HPL19-107"}   # slot 1 read slot 0's tag
    assert unit._shared_rfid_ambiguous(1) is True
    assert unit._shared_rfid_ambiguous(0) is True


def test_ace2_ambiguous_when_sibling_reports_same_uid():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[2] = {"sku": "", "uid": "deadbeef"}
    unit._slot_inventory[3] = {"sku": "", "uid": "deadbeef"}
    assert unit._shared_rfid_ambiguous(2) is True


def test_ace2_not_ambiguous_when_sibling_differs():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[2] = {"sku": "BAMBU-A", "uid": "aa"}
    unit._slot_inventory[3] = {"sku": "BAMBU-B", "uid": "bb"}
    assert unit._shared_rfid_ambiguous(2) is False
    assert unit._shared_rfid_ambiguous(3) is False


def test_ace2_not_ambiguous_when_sibling_empty():
    unit = afcACE2.__new__(afcACE2)
    unit._slot_inventory = _inv(afcACE2)
    unit._slot_inventory[0] = {"sku": "HPL19-107"}
    unit._slot_inventory[1] = {}                       # empty sibling
    assert unit._shared_rfid_ambiguous(0) is False
