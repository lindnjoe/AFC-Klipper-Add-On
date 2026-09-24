# The AMS flash is addressed per MODEL, and the loader gate must not pass on
# our own echo.
#
# WHAT THIS EXISTS FOR. The flash path was written against AMS 2 captures --
# the only ones we had -- and it picked up three AMS-2 constants that do not
# announce themselves as constants:
#
#   1. ams_fw_flash.LOADER_TARGET = 0x0700  "as in every capture"
#   2. build_cmd1's f[14] = 0x07            "constant seen in every capture"
#   3. build_cmd1 left f[19] (the AMS id) zero
#
# All three were true, and all three were true only because every capture was
# an AMS 2. The AMS HT update capture (2026-09-16) shows the same frame with
# 0x1800, 0x18 and 0x80. Two of the three live in the PAYLOAD rather than the
# address, so aiming the flasher at an HT by changing the target alone still
# produces a frame the HT ignores -- it never enters its loader, and the only
# symptom is a flash that refuses for a reason that points nowhere.
#
# AND THE ONE THAT MATTERS MOST. loader_hits() decided "the unit is in its
# loader" -- the gate immediately before the ERASE -- partly on
# `h.startswith("3d04")`. Class 0x04 is the MASTER'S cmd1 poke, which is to
# say the frame we ourselves just sent. ams_update.py step 5 turns on txecho,
# and txecho carries BOTH directions (bambubus.c: "rx: 0 = a frame WE
# transmitted, 1 = a frame the AMS sent back"), so our own transmission was
# fed straight back into that test. A unit that said nothing at all could be
# reported as LOADER ENTERED. The check is now on the SOURCE address, which is
# what the comment beside it always claimed it was.
from __future__ import annotations

import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "bambu_ams_bridge", "tools")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

from ams_cmd1_probe import build_cmd1, crc8, crc16, loader_hits  # noqa: E402
import ams_fw_flash as F                                          # noqa: E402

# The real enter-loader frames, lifted from the two models' update captures.
# Counters and CRCs are theirs; if build_cmd1 reproduces these byte for byte it
# is emitting what a Bambu printer emits.
CAPTURED_CMD1 = {
    "ams2": (0x0700, 0x00, "3d04110a31004800070009060101070000000000000000"
                           "0000000000000000000000000000000000000000000000"
                           "00f0d3"),
    "ht":   (0x1800, 0x80, "3d04a60b3100370018000906010118000000008000000000"
                           "00000000000000000000000000000000000000000000004"
                           "7d4"),
}

# The HT's own loader answer (class 0x00, target 0009 = the updater, source
# 0018 = the unit). This is what a real "it is in its loader" looks like.
HT_LOADER_REPLY = ("3d0000002900ea0009001806010118000000008000000000d0020000"
                   "00000000040000000000007179")


@pytest.mark.parametrize("model", sorted(CAPTURED_CMD1))
def test_build_cmd1_reproduces_the_captured_frame(model):
    target, ams_id, hexed = CAPTURED_CMD1[model]
    want = bytes.fromhex(hexed)
    counter = int.from_bytes(want[2:4], "little")
    assert build_cmd1(target, counter, ams_id) == want


def test_the_two_models_differ_in_the_payload_not_only_the_address():
    # The point of the whole file. If someone "fixes" HT support by passing a
    # different target and nothing else, these bytes go back to the AMS 2
    # values and the HT silently ignores the poke.
    ams2 = build_cmd1(0x0700, 0x0077, 0x00)
    ht = build_cmd1(0x1800, 0x0077, 0x80)
    assert ams2[7:9] == b"\x00\x07" and ht[7:9] == b"\x00\x18"   # address
    assert ams2[14] == 0x07 and ht[14] == 0x18                   # device class
    assert ams2[19] == 0x00 and ht[19] == 0x80                   # AMS id


def test_the_device_class_byte_tracks_the_address():
    # [14] is the target's high byte, so it cannot drift out of step with the
    # address it is derived from -- that is why it is not a parameter.
    for target in (0x0700, 0x1800):
        assert build_cmd1(target, 1, 0)[14] == (target >> 8) & 0xFF


def test_cmd1_stays_crc_correct_for_both_models():
    for target, ams_id in ((0x0700, 0x00), (0x1800, 0x80)):
        f = build_cmd1(target, 0x1234, ams_id)
        assert len(f) == 49
        assert crc8(f[:6]) == f[6]
        assert crc16(f[:47]) == (f[47] | (f[48] << 8))


# ── the loader gate ──────────────────────────────────────────────────────────

def test_our_own_cmd1_is_not_mistaken_for_a_loader():
    # THE DANGEROUS ONE. txecho feeds our own transmission back; if this counts
    # as "the loader answered", enter_loader() hands off to the header and the
    # ERASE goes out at a unit that never responded.
    for target, ams_id in ((0x0700, 0x00), (0x1800, 0x80)):
        echo = build_cmd1(target, 0x0077, ams_id).hex()
        assert loader_hits(echo) == [], (
            f"our own cmd1 for 0x{target:04X} reads as a loader response")


def test_the_units_real_loader_reply_is_recognised():
    assert "ams-origin/op0601" in loader_hits(HT_LOADER_REPLY)


def test_the_gate_keys_on_the_source_not_the_class():
    # Same opcode, same payload, only the source swapped: updater (0x0900) is
    # us, anything else is the unit. Nothing about the class byte may decide
    # this -- both directions use classes 0x00, 0x04 and 0x05.
    unit = bytes.fromhex(HT_LOADER_REPLY)
    assert unit[9:11] == b"\x00\x18"                 # source: the HT
    faked = bytearray(unit)
    faked[9:11] = b"\x00\x09"                        # source: the updater (us)
    assert loader_hits(bytes(faked).hex()) == []


# ── addressing comes from the artifact, so it cannot be mismatched ───────────

def _artifact(target: int, ams_id: int, blocks: int = 2) -> dict:
    """A minimal artifact whose frames carry the routing under test.

    Bodies are captured frame[7:-2], so body[0:2] is the target and body[12]
    (frame byte 19) is the AMS id.
    """
    def body(cmd):
        b = bytearray(40)
        b[0] = target & 0xFF
        b[1] = (target >> 8) & 0xFF
        b[2], b[3] = 0x00, 0x09          # source: the updater
        b[4], b[5] = 0x06, 0x01          # op 0601
        b[6] = cmd
        b[7] = (target >> 8) & 0xFF      # device class
        b[19 - 7] = ams_id
        return bytes(b)
    return {"header": body(0x02), "blocks": [body(0x03)] * blocks}


@pytest.mark.parametrize("target,ams_id", [(0x0700, 0x00), (0x1800, 0x80)])
def test_the_loader_target_is_read_off_the_artifact(target, ams_id):
    assert F.loader_target_of(_artifact(target, ams_id)) == (target, ams_id)


def test_a_real_artifact_addresses_its_own_model():
    # Belt and braces against the shipped AMS 2 artifact if it is present --
    # skipped rather than failed, because the artifacts are not in the repo
    # (they are per-model captures, staged into printer_data/config).
    p = os.path.join(os.path.expanduser("~"), "printer_data", "config",
                     "ams_firmware", "ams_artifact.json")
    if not os.path.exists(p):
        pytest.skip("no staged AMS 2 artifact on this host")
    d = json.load(open(p))
    art = {"header": bytes.fromhex(d["header"]),
           "blocks": [bytes.fromhex(b) for b in d["blocks"]]}
    assert F.loader_target_of(art) == (0x0700, 0x00)


def test_the_hardcoded_loader_target_is_gone():
    # A module-level constant here is how this got model-locked the first time.
    assert not hasattr(F, "LOADER_TARGET"), (
        "LOADER_TARGET is back: the enter-loader poke must be derived from the "
        "artifact, or it can name a different unit than the image does")


# ── completeness is checked against the image, not a block count ─────────────

def test_the_completeness_check_is_not_a_block_count():
    # `len(blocks) == 168` was the AMS 2's count and nothing more. It refused a
    # good AMS HT artifact (157 blocks) while reporting "failed verification",
    # and it would equally have passed a 168-block artifact missing half its
    # bytes. The artifact describes its own size; use that.
    import inspect

    import ams_update
    src = inspect.getsource(ams_update.check_artifact)
    assert "168" not in src, (
        "check_artifact still hardcodes a block count -- it must compare the "
        "blocks against the image the artifact declares")
    assert "artifact_image_bytes" in src


@pytest.mark.parametrize("nblocks,per", [(157, 1020), (168, 1022), (3, 40)])
def test_declared_image_equals_carried_bytes_plus_the_container_header(
        nblocks, per):
    # The invariant, at any block count: what the header declares is what the
    # blocks carry plus the BIMH container header.
    total = nblocks * per + F.BIMH_HEADER_LEN

    def hdr():
        b = bytearray(40)
        b[23 - 7:26 - 7] = total.to_bytes(3, "little")
        return bytes(b)

    def blk():
        b = bytearray(40)
        b[35 - 7:39 - 7] = per.to_bytes(4, "little")
        return bytes(b)

    art = {"header": hdr(), "blocks": [blk()] * nblocks}
    got_total, got_sum = F.artifact_image_bytes(art)
    assert got_total == total
    assert got_total - got_sum == F.BIMH_HEADER_LEN


def test_a_truncated_artifact_is_caught_however_many_blocks_it_has():
    # The failure a count cannot see: the right number of blocks, the wrong
    # number of bytes.
    per, n = 1020, 157
    total = n * per + F.BIMH_HEADER_LEN
    h = bytearray(40); h[23 - 7:26 - 7] = total.to_bytes(3, "little")
    short = bytearray(40); short[35 - 7:39 - 7] = (per // 2).to_bytes(4, "little")
    full = bytearray(40); full[35 - 7:39 - 7] = per.to_bytes(4, "little")
    art = {"header": bytes(h),
           "blocks": [bytes(full)] * (n - 1) + [bytes(short)]}
    got_total, got_sum = F.artifact_image_bytes(art)
    assert got_total - got_sum != F.BIMH_HEADER_LEN


# ── config values, and the gate that never opened the link ───────────────────

def test_an_inline_comment_is_not_part_of_the_config_value():
    # THE BUG THIS EXISTS FOR, TWICE. cfg_value returned everything after the
    # colon, so an ordinary annotated config line
    #
    #     serial_port: tcp://192.168.6.3:8888   # Pico 2 W (WiFi)
    #
    # carried the note into port parsing and step 5 of an ARMED flash died on
    # int('8888   # Pico 2 W (WiFi)'). It failed safe -- before cmd1, nothing
    # erased -- but it failed on the first line of the destructive run, after
    # `check` had reported every gate green, because no read-only gate opens
    # the link. The identical bug had already been fixed in
    # standalone/ams_flash.py's find_in_cfg the same day.
    import os
    import tempfile

    import ams_update as U

    d = tempfile.mkdtemp()
    os.makedirs(os.path.join(d, "AFC"))
    with open(os.path.join(d, "AFC", "AFC_BridgeBox.cfg"), "w") as fh:
        fh.write("[AFC_BridgeBox chain1]\n"
                 "serial_port: tcp://192.168.6.3:8888   # Pico 2 W (WiFi)\n"
                 "tcp_key: deadbeef  ; trailing note\n")
    old, U.CFG_DIR = U.CFG_DIR, d
    try:
        assert U.cfg_value("serial_port") == "tcp://192.168.6.3:8888"
        assert U.cfg_value("tcp_key") == "deadbeef"
    finally:
        U.CFG_DIR = old


def test_the_target_is_parsed_before_the_gates_not_at_the_link():
    # A startswith("tcp://") check passes anything URL-shaped, and steps 1-4
    # never open a socket -- so the first real parse happened inside the armed
    # run. The preflight must do the parse itself.
    import inspect

    import ams_update as U
    src = inspect.getsource(U.main)
    i = src.index('startswith("tcp://")')
    j = src.index("step(1,")
    window = src[i:j]
    assert "int(_p)" in window, (
        "the bridge target is never parsed between the startswith() check and "
        "step 1, so a malformed port survives every read-only gate and only "
        "surfaces once the flash is armed")
