"""
Unit tests for the U1 RFID spool-scanner stable-read gate in
extras/AFC_U1_rfid.py

Covers _check_channel on a standalone scanner channel:
  - scanner_confirm_reads: N consecutive identical UIDs required before a
    scan acts (the duplicate-Spoolman-spool fix for corrupt/partial reads)
  - a different UID mid-confirmation resets the count
  - a webhook read is authoritative and bypasses the gate
  - UID dedup: the same spool never re-fires
  - tag removal (uid 0) clears the pending confirmation
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from extras.AFC_U1_rfid import AFC_U1_RFID


TAG = {
    "CARD_UID": 0x56A36AEA,
    "MAIN_TYPE": "PLA",
    "SUB_TYPE": "",
}
TAG_OTHER = dict(TAG, CARD_UID=0x26A36AEA)


def _make_rfid(confirm_reads=3):
    rfid = AFC_U1_RFID.__new__(AFC_U1_RFID)
    rfid.logger = MagicMock()
    rfid.afc = MagicMock()
    rfid.reactor = MagicMock()
    rfid._filament_detect = MagicMock()
    rfid._cfg_scanner_channels = {0}
    rfid._lane_objects = {}
    rfid._lane_channel_map = {}
    rfid._scanner_confirm_reads = confirm_reads
    rfid._pending_confirm = {}
    rfid._pending_defer = {}
    rfid._last_uid = {}
    rfid._webhook_channels_seen = set()
    rfid._webhook_grace = 0.0
    rfid._scanner_auto_create = True
    rfid._lane_auto_create = True
    rfid._notify_scan = MagicMock()
    rfid._map_to_slot_info = MagicMock(return_value={
        "brand": "Test", "material": "PLA", "color_hex": "FF0000",
        "multi_color": ["FF0000"]})
    return rfid


def _scan(rfid, info=TAG, source='poll'):
    with patch("extras.AFC_U1_rfid.sync_rfid_to_spoolman") as sync:
        rfid._check_channel("", 0, info=dict(info), source=source)
    return sync


# ── Stable-read gate ──────────────────────────────────────────────────────────

def test_scan_waits_for_n_consecutive_reads():
    rfid = _make_rfid(confirm_reads=3)

    sync1 = _scan(rfid)
    sync2 = _scan(rfid)
    assert not sync1.called and not sync2.called
    assert rfid._pending_confirm[0] == (TAG["CARD_UID"], 2)

    sync3 = _scan(rfid)  # third consecutive identical read acts
    sync3.assert_called_once()
    rfid._notify_scan.assert_called_once()
    assert 0 not in rfid._pending_confirm
    assert rfid._last_uid[0] == TAG["CARD_UID"]


def test_different_uid_mid_confirmation_resets_count():
    """A corrupt/partial UID mid-stream must not accumulate — a stable clean
    read wins over transient misreads."""
    rfid = _make_rfid(confirm_reads=3)

    _scan(rfid)                       # uid A: count 1
    _scan(rfid, info=TAG_OTHER)       # uid B: count resets to 1
    assert rfid._pending_confirm[0] == (TAG_OTHER["CARD_UID"], 1)

    sync = _scan(rfid)                # uid A again: count 1, not 2
    assert not sync.called
    assert rfid._pending_confirm[0] == (TAG["CARD_UID"], 1)


def test_confirm_reads_of_one_acts_immediately():
    rfid = _make_rfid(confirm_reads=1)
    sync = _scan(rfid)
    sync.assert_called_once()


def test_webhook_bypasses_gate():
    """A webhook is a full-data authoritative push — no confirmation needed."""
    rfid = _make_rfid(confirm_reads=3)
    sync = _scan(rfid, source='webhook')
    sync.assert_called_once()


def test_same_uid_never_refires():
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()

    # Spool still presented: repeat reads of the same UID are deduped
    sync = _scan(rfid)
    assert not sync.called
    assert rfid._notify_scan.call_count == 1


def test_new_spool_after_first_fires_again():
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()
    _scan(rfid, info=TAG_OTHER).assert_called_once()
    assert rfid._last_uid[0] == TAG_OTHER["CARD_UID"]


def test_tag_removal_clears_pending_confirmation():
    rfid = _make_rfid(confirm_reads=3)
    _scan(rfid)
    assert 0 in rfid._pending_confirm

    sync = _scan(rfid, info=dict(TAG, CARD_UID=0))  # tag removed
    assert not sync.called
    assert 0 not in rfid._pending_confirm
    # Scanner channels keep _last_uid so the same spool doesn't re-fire
    # after being staged; nothing was staged yet here.
    assert rfid._last_uid.get(0) in (None, 0)


def test_scanner_sets_next_spool_staging():
    """Scanner reads stage via next_spool_id (set_next=True) rather than
    assigning to a lane."""
    rfid = _make_rfid(confirm_reads=1)
    sync = _scan(rfid)
    assert sync.call_args.kwargs.get("set_next") is True
