"""
Unit tests for the U1 RFID spool-scanner stable-read gate in
extras/AFC_U1_rfid.py

Style: typed in-file fakes (no MagicMock; self-contained so this module
doesn't couple to the OpenAMS/ACE test helpers), full state verification,
branch-complete coverage of _check_channel on a standalone scanner channel:

  - scanner_confirm_reads N-consecutive-identical-UID gate (the
    duplicate-Spoolman-spool fix), incl. pending-counter state
  - a different UID mid-confirmation resets the count
  - confirm_reads=1 acts immediately; webhook reads bypass the gate
  - UID dedup (same spool never re-fires); a new spool fires again
  - tag removal (uid 0) clears the pending confirmation and (for scanner
    channels) keeps _last_uid so a staged spool can't re-fire
  - MAIN_TYPE 'NONE'/missing stops after recording the UID
  - scanner reads stage via next_spool_id (set_next=True)
"""

from __future__ import annotations

from unittest.mock import patch

from extras.AFC_U1_rfid import AFC_U1_RFID


TAG = {
    "CARD_UID": 0x56A36AEA,
    "MAIN_TYPE": "PLA",
    "SUB_TYPE": "",
}
TAG_OTHER = dict(TAG, CARD_UID=0x26A36AEA)


# ── Typed fakes ───────────────────────────────────────────────────────────────

class _Recorder:
    def __init__(self, result=None):
        self.calls = []
        self.result = result

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return self.result

    @property
    def called(self):
        return len(self.calls) > 0

    @property
    def call_count(self):
        return len(self.calls)


class _FakeLogger:
    def __init__(self):
        self.lines = {"info": [], "debug": [], "warning": [], "error": []}

    def info(self, msg, console_only=False):
        self.lines["info"].append(msg)

    def debug(self, msg, only_debug=False, traceback=None):
        self.lines["debug"].append(msg)

    def warning(self, msg):
        self.lines["warning"].append(msg)

    def error(self, msg, traceback=None, stack_name=""):
        self.lines["error"].append(msg)


class _FakeReactor:
    def __init__(self):
        self.register_callback = _Recorder()

    def monotonic(self):
        return 100.0


class _FakeAFC:
    def __init__(self):
        self.lanes = {}


def _make_rfid(confirm_reads=3, webhook_grace=0.0):
    rfid = AFC_U1_RFID.__new__(AFC_U1_RFID)
    rfid.logger = _FakeLogger()
    rfid.afc = _FakeAFC()
    rfid.reactor = _FakeReactor()
    rfid._filament_detect = object()
    rfid._cfg_scanner_channels = {0}
    rfid._lane_objects = {}
    rfid._lane_channel_map = {}
    rfid._scanner_confirm_reads = confirm_reads
    rfid._pending_confirm = {}
    rfid._pending_defer = {}
    rfid._last_uid = {}
    rfid._webhook_channels_seen = set()
    rfid._webhook_grace = webhook_grace
    rfid._scanner_auto_create = True
    rfid._lane_auto_create = True
    rfid._notify_scan = _Recorder()
    rfid._map_to_slot_info = _Recorder(result={
        "brand": "Test", "material": "PLA", "color_hex": "FF0000",
        "multi_color": ["FF0000"]})
    return rfid


def _scan(rfid, info=TAG, source='poll'):
    # The stand-in HONOURS on_done. The real sync_rfid_to_spoolman promises to
    # call it on every path -- that is how the caller learns the lane actually
    # carries the spool, now that the Spoolman round trips happen off the
    # reactor. A mock that just swallowed the callback would report the
    # notification as never sent, which is a fault in the fake, not the code.
    def _sync(*a, **kw):
        done = kw.get("on_done")
        if done is not None:
            done()

    with patch("extras.AFC_U1_rfid.sync_rfid_to_spoolman",
               side_effect=_sync) as sync:
        rfid._check_channel("", 0, info=dict(info), source=source)
    return sync


# ── Stable-read gate ──────────────────────────────────────────────────────────

def test_scan_waits_for_n_consecutive_reads():
    rfid = _make_rfid(confirm_reads=3)

    sync1 = _scan(rfid)
    assert not sync1.called
    assert rfid._pending_confirm[0] == (TAG["CARD_UID"], 1)
    assert 0 not in rfid._last_uid            # not yet acted

    sync2 = _scan(rfid)
    assert not sync2.called
    assert rfid._pending_confirm[0] == (TAG["CARD_UID"], 2)

    sync3 = _scan(rfid)  # third consecutive identical read acts
    sync3.assert_called_once()
    assert rfid._notify_scan.call_count == 1
    assert 0 not in rfid._pending_confirm     # gate consumed
    assert rfid._last_uid[0] == TAG["CARD_UID"]


def test_different_uid_mid_confirmation_resets_count():
    """A corrupt/partial UID mid-stream must not accumulate, a stable clean
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
    assert rfid._pending_confirm == {}        # gate never engaged
    assert rfid._last_uid[0] == TAG["CARD_UID"]


def test_webhook_bypasses_gate():
    """A webhook is a full-data authoritative push, no confirmation needed."""
    rfid = _make_rfid(confirm_reads=3)
    sync = _scan(rfid, source='webhook')
    sync.assert_called_once()
    assert rfid._pending_confirm == {}
    assert rfid._notify_scan.call_count == 1


# ── Dedup / removal ───────────────────────────────────────────────────────────

def test_same_uid_never_refires():
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()

    sync = _scan(rfid)  # spool still presented
    assert not sync.called
    assert rfid._notify_scan.call_count == 1


def test_new_spool_after_first_fires_again():
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()
    _scan(rfid, info=TAG_OTHER).assert_called_once()
    assert rfid._last_uid[0] == TAG_OTHER["CARD_UID"]
    assert rfid._notify_scan.call_count == 2


def test_tag_removal_clears_pending_confirmation():
    rfid = _make_rfid(confirm_reads=3)
    _scan(rfid)
    assert 0 in rfid._pending_confirm

    sync = _scan(rfid, info=dict(TAG, CARD_UID=0))  # tag removed

    assert not sync.called
    assert 0 not in rfid._pending_confirm
    assert rfid._last_uid.get(0) in (None, 0)  # nothing was staged yet


def test_scanner_keeps_last_uid_after_removal():
    """Scanner channels intentionally keep _last_uid after a completed scan:
    the same spool must not re-fire while/after being presented."""
    rfid = _make_rfid(confirm_reads=1)
    _scan(rfid).assert_called_once()
    assert rfid._last_uid[0] == TAG["CARD_UID"]

    _scan(rfid, info=dict(TAG, CARD_UID=0))         # removed
    assert rfid._last_uid[0] == TAG["CARD_UID"]     # kept (scanner channel)

    sync = _scan(rfid)                              # same spool re-presented
    assert not sync.called                          # still deduped


# ── Content gates ─────────────────────────────────────────────────────────────

def test_main_type_none_records_uid_but_does_not_act():
    rfid = _make_rfid(confirm_reads=1)
    sync = _scan(rfid, info=dict(TAG, MAIN_TYPE="NONE"))
    assert not sync.called
    assert not rfid._notify_scan.called
    assert rfid._last_uid[0] == TAG["CARD_UID"]  # recorded for dedup


def test_scanner_sets_next_spool_staging():
    """Scanner reads stage via next_spool_id (set_next=True) rather than
    assigning to a lane."""
    rfid = _make_rfid(confirm_reads=1)
    sync = _scan(rfid)
    assert sync.call_args.kwargs.get("set_next") is True
    args = sync.call_args.args
    assert args[1] is None  # scanner channel has no lane


# ── The scanner-protection patch, and whose signature it is ───────────────────
#
# _patch_scanner_rfid_update puts our wrapper INTO the U1's own
# _notify_data_update_cb list, so the U1's firmware is what calls it. Naming its
# arguments in our wrapper means a Snapmaker update that adds one raises
# TypeError from inside our code on the notify path -- which shuts Klipper down.
# The same mistake in AFC_autocal's set_spoolID wrapper did exactly that when
# upstream AFC added an on_done argument.

class _FakeFD:
    def __init__(self, cb_list):
        self._notify_data_update_cb = cb_list


class _FakePTC:
    def __init__(self, cb):
        self._rfid_filament_info_update_cb = cb


class _LookupPrinter:
    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name, default=None):
        return self._objs.get(name, default)


def _patched_cb(scanner_channels=None):
    """Install the patch over a recorder and hand back both ends."""
    scanner_channels = {0} if scanner_channels is None else scanner_channels
    original = _Recorder()
    fd = _FakeFD([original])
    rfid = _make_rfid()
    rfid._cfg_scanner_channels = scanner_channels
    rfid.printer = _LookupPrinter({"print_task_config": _FakePTC(original),
                                   "filament_detect": fd})
    rfid._patch_scanner_rfid_update()
    assert fd._notify_data_update_cb[0] is not original
    return fd._notify_data_update_cb[0], original


def test_patch_on_the_shipping_snapmaker_signature():
    """The firmware in the field: (channel, info, is_clear), all positional.

    This is the contract the patch was written against and the one it must
    keep working on -- pinned here so the arity-agnostic forwarding below
    cannot be "simplified" into supporting only a newer API.
    """
    patched, original = _patched_cb()
    patched(0, {"x": 1}, False)          # scanner channel: suppressed
    assert original.called is False
    patched(3, {"y": 2}, True)           # other channel: passes through
    assert original.calls[-1] == ((3, {"y": 2}, True), {})


def test_patch_on_the_shipping_signature_without_the_optional_argument():
    """is_clear has a default upstream, so a two-argument call is legal too."""
    patched, original = _patched_cb()
    patched(3, {"y": 2})
    assert original.calls[-1] == ((3, {"y": 2}), {})
    patched(0, {"x": 1})
    assert original.call_count == 1      # scanner channel still suppressed


def test_patch_forwards_an_argument_a_future_firmware_might_add():
    patched, original = _patched_cb()
    patched(3, {"y": 2}, True, "source")
    assert original.calls[-1] == ((3, {"y": 2}, True, "source"), {})


def test_patch_forwards_keyword_arguments():
    patched, original = _patched_cb()
    patched(3, {"y": 2}, is_clear=True, official=False)
    assert original.calls[-1] == ((3, {"y": 2}),
                                  {"is_clear": True, "official": False})


def test_patch_suppresses_on_a_keyword_channel():
    patched, original = _patched_cb()
    patched(channel=0, info={"x": 1})
    assert original.called is False


def test_patch_passes_on_anything_it_cannot_read_as_a_channel():
    """A missed suppression writes the U1 display; a raise stops the printer."""
    patched, original = _patched_cb()
    patched("not-a-channel", {"z": 3})
    assert original.calls[-1] == (("not-a-channel", {"z": 3}), {})
