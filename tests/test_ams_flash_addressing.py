# The AMS updater (Firmwares/Bambu_AMS/ams_flash.py) addresses the flash per
# MODEL, and its loader gate must not pass on our own echo.
#
# The enter-loader poke differs between models in three places: the target
# address (0x0700 AMS 2, 0x1800 HT), the device-class byte at frame [14] (the
# target's high byte) and the AMS id at frame [19] (0x00 AMS 2, 0x80 HT). Two
# of the three live in the PAYLOAD rather than the address, so aiming at an HT
# by changing the target alone produces a frame the HT ignores: it never
# enters its loader, and the update refuses for a reason that points nowhere.
#
# The loader gate is the last check before the ERASE. txecho carries BOTH
# directions (its rx flag: 0 = a frame the bridge transmitted, 1 = a frame the
# AMS sent back), so our own cmd1 comes straight back to it. The gate keys on
# the frame's SOURCE address; a check on the class byte would read our own
# poke as "loader entered" from a unit that said nothing at all.
from __future__ import annotations

import json
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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


# ── addressing comes from the artifact, so it cannot be mismatched ───────────

#: The vendor image name each model's header frame carries (BIMH container).
IMAGE_NAMES = {
    (0x0700, 0x00): "n3f_rev5-firmware-v05.00.22.22-20260702164346.bin.sig",
    (0x1800, 0x80): "n3s_rev5-firmware-v05.00.22.19-20260616201708.bin.sig",
}
AMS1_IMAGE = "ams_rev8-firmware-v01.00.06.87-20260109152259.bin.sig"


def _artifact(target: int, ams_id: int, blocks: int = 2,
              image: str = None) -> dict:
    """A minimal artifact whose frames carry the routing under test.

    Bodies are captured frame[7:-2], so body[0:2] is the target and body[12]
    (frame byte 19) is the AMS id. The header's payload holds the BIMH
    container header with the image's vendor file name, as a real one does.
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
    name = image if image is not None else IMAGE_NAMES[(target, ams_id)]
    header = (body(0x02)[:32] + b"BIMH" + bytes(44) + name.encode()
              + bytes(8))
    return {"header": header, "blocks": [body(0x03)] * blocks}


# ── the drop-in flasher (Firmwares/Bambu_AMS/ams_flash.py) ───────────────────
#
# The gcode commands AFC_BAMBU_AMS_UPDATE, AFC_BAMBU_AMS1_UPDATE and
# AFC_BAMBU_HT_UPDATE run this file. It is stdlib-only and needs nothing else
# from this repository on the printer.

STANDALONE = os.path.join(ROOT, "Firmwares", "Bambu_AMS")
_STANDALONE_MOD = None


def _standalone():
    global _STANDALONE_MOD
    if _STANDALONE_MOD is None:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "ams_flash_standalone", os.path.join(STANDALONE, "ams_flash.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _STANDALONE_MOD = mod
    return _STANDALONE_MOD


@pytest.fixture(scope="module")
def S():
    return _standalone()


@pytest.mark.parametrize("model", sorted(CAPTURED_CMD1))
def test_standalone_cmd1_reproduces_the_captured_frame(S, model):
    target, ams_id, hexed = CAPTURED_CMD1[model]
    want = bytes.fromhex(hexed)
    counter = int.from_bytes(want[2:4], "little")
    assert S.build_cmd1(target, ams_id, counter) == want


@pytest.mark.parametrize("model", sorted(CAPTURED_CMD1))
def test_standalone_cmd1_is_addressed_off_the_artifact(S, model):
    target, ams_id, _hex = CAPTURED_CMD1[model]
    art = _artifact(target, ams_id)
    assert S.loader_target_of(art) == (target, ams_id)
    f = S.build_cmd1(*S.loader_target_of(art))
    assert (f[7] | (f[8] << 8), f[14], f[19]) == (target, target >> 8, ams_id)


def test_standalone_has_no_fixed_loader_target(S):
    assert not hasattr(S, "LOADER_TARGET")


def test_the_two_models_differ_in_the_payload_not_only_the_address(S):
    # If HT support is "fixed" by passing a different target and nothing else,
    # these bytes go back to the AMS 2 values and the HT ignores the poke.
    ams2 = S.build_cmd1(0x0700, 0x00)
    ht = S.build_cmd1(0x1800, 0x80)
    assert ams2[7:9] == b"\x00\x07" and ht[7:9] == b"\x00\x18"   # address
    assert ams2[14] == 0x07 and ht[14] == 0x18                   # device class
    assert ams2[19] == 0x00 and ht[19] == 0x80                   # AMS id


def test_the_device_class_byte_tracks_the_address(S):
    # [14] is the target's high byte, so it cannot drift out of step with the
    # address it is derived from; that is why it is not a parameter.
    for target in (0x0700, 0x1800):
        assert S.build_cmd1(target, 0, 1)[14] == (target >> 8) & 0xFF


def test_cmd1_stays_crc_correct_for_both_models(S):
    for target, ams_id in ((0x0700, 0x00), (0x1800, 0x80)):
        f = S.build_cmd1(target, ams_id, 0x1234)
        assert len(f) == 49
        assert S.crc8(f[:6]) == f[6]
        assert S.crc16(f[:47]) == (f[47] | (f[48] << 8))


def test_standalone_gate_ignores_our_own_cmd1(S):
    for target, ams_id in ((0x0700, 0x00), (0x1800, 0x80)):
        assert S.loader_hits(S.build_cmd1(target, ams_id).hex()) == []


def test_standalone_gate_recognises_the_real_loader_reply(S):
    assert "ams-origin/op0601" in S.loader_hits(HT_LOADER_REPLY)
    faked = bytearray(bytes.fromhex(HT_LOADER_REPLY))
    faked[9:11] = b"\x00\x09"                        # source: us
    assert S.loader_hits(bytes(faked).hex()) == []


def test_each_model_command_has_its_own_address(S):
    # The table the commands use must agree with the captured frames.
    assert (S.MODELS["ht"]["dev"], S.MODELS["ht"]["ams_id"]) == (0x1800, 0x80)
    for m in ("ams2", "ams1"):
        assert (S.MODELS[m]["dev"], S.MODELS[m]["ams_id"]) == (0x0700, 0x00)
    files = [S.MODELS[m]["artifact"] for m in S.MODELS]
    assert len(set(files)) == len(files)             # one file per model


@pytest.mark.parametrize("model,ok", [
    ("ht", (0x1800, 0x80)), ("ams2", (0x0700, 0x00))])
def test_an_artifact_for_another_device_is_refused(S, model, ok):
    assert S.artifact_mismatch(_artifact(*ok), model) is None
    other = (0x0700, 0x00) if ok[0] == 0x1800 else (0x1800, 0x80)
    assert "Wrong file" in S.artifact_mismatch(_artifact(*other), model)


def test_an_ams2_image_is_refused_by_the_ams1_command_and_back(S):
    # Same address, so only the image's own name can tell them apart. An AMS 2
    # image saved as ams1_artifact.json would otherwise erase an AMS 1 with no
    # AMS 1 image to put back.
    ams2 = _artifact(0x0700, 0x00)
    ams1 = _artifact(0x0700, 0x00, image=AMS1_IMAGE)
    assert S.artifact_mismatch(ams1, "ams1") is None
    assert "not an AMS 1 image" in S.artifact_mismatch(ams2, "ams1")
    assert "not an AMS 2 Pro image" in S.artifact_mismatch(ams1, "ams2")
    unnamed = _artifact(0x0700, 0x00, image="")
    assert "unnamed" in S.artifact_mismatch(unnamed, "ams2")


def test_a_block_addressed_elsewhere_is_refused(S):
    art = _artifact(0x0700, 0x00, blocks=3)
    stray = bytearray(art["blocks"][1])
    stray[12] = 0x01                                 # another unit's id
    art["blocks"][1] = bytes(stray)
    assert "data block 1" in S.artifact_mismatch(art, "ams2")


@pytest.mark.parametrize("nblocks,per", [(157, 1020), (168, 1022), (3, 40)])
def test_declared_image_equals_carried_bytes_plus_the_container_header(
        S, nblocks, per):
    # Completeness is checked against the image the artifact declares, not a
    # block count: what the header declares is what the blocks carry plus the
    # BIMH container header, at any block count.
    total = nblocks * per + S.BIMH_HEADER_LEN

    def hdr():
        b = bytearray(40)
        b[23 - 7:26 - 7] = total.to_bytes(3, "little")
        return bytes(b)

    def blk():
        b = bytearray(40)
        b[35 - 7:39 - 7] = per.to_bytes(4, "little")
        return bytes(b)

    art = {"header": hdr(), "blocks": [blk()] * nblocks}
    got_total, got_sum = S.artifact_image_bytes(art)
    assert got_total == total
    assert got_total - got_sum == S.BIMH_HEADER_LEN


def test_a_truncated_artifact_is_caught_however_many_blocks_it_has(S):
    # The failure a count cannot see: the right number of blocks, the wrong
    # number of bytes.
    per, n = 1020, 157
    total = n * per + S.BIMH_HEADER_LEN
    h = bytearray(40); h[23 - 7:26 - 7] = total.to_bytes(3, "little")
    short = bytearray(40); short[35 - 7:39 - 7] = (per // 2).to_bytes(4, "little")
    full = bytearray(40); full[35 - 7:39 - 7] = per.to_bytes(4, "little")
    art = {"header": bytes(h),
           "blocks": [bytes(full)] * (n - 1) + [bytes(short)]}
    got_total, got_sum = S.artifact_image_bytes(art)
    assert got_total - got_sum != S.BIMH_HEADER_LEN


def _row(name, online, model, index=None):
    if index is None:
        index = 4 if model == "ht" else 0
    return {"name": name, "online": online, "model": model, "index": index,
            "fw": "AFC-2.81"}


def test_the_one_unit_online_is_the_target(S):
    rows = [_row("Bambu_AMS_1", True, "ams2"),
            _row("Bambu_AMS_HT_1", False, "ht")]
    assert S.pick_target(rows, "ams2", 0x00) == ("Bambu_AMS_1", None, False)
    rows = [_row("Bambu_AMS_1", False, "ams2"),
            _row("Bambu_AMS_HT_1", True, "ht")]
    assert S.pick_target(rows, "ht", 0x80) == ("Bambu_AMS_HT_1", None, False)


def test_a_second_unit_online_refuses_whatever_its_model(S):
    rows = [_row("Bambu_AMS_1", True, "ams2"),
            _row("Bambu_AMS_HT_1", True, "ht")]
    for model, ams_id in (("ams2", 0x00), ("ht", 0x80)):
        unit, why, _r = S.pick_target(rows, model, ams_id)
        assert unit is None and "exactly one AMS" in why


def test_no_unit_online_refuses(S):
    unit, why, _r = S.pick_target([_row("Bambu_AMS_1", False, "ams2")],
                                  "ams2", 0x00)
    assert unit is None and "found 0" in why


@pytest.mark.parametrize("cmd,unit_model", [
    ("ams2", "ams1"), ("ams1", "ams2"), ("ht", "ams2"), ("ams2", "ht"),
    ("ams1", ""), ("ht", "boxed")])
def test_the_online_unit_must_be_the_commands_model(S, cmd, unit_model):
    ams_id = S.MODELS[cmd]["ams_id"]
    index = 4 if cmd == "ht" else 0
    unit, why, _r = S.pick_target(
        [_row("Bambu_AMS_1", True, unit_model, index)], cmd, ams_id)
    assert unit is None and "this command is for" in why


@pytest.mark.parametrize("cmd", ["ams1", "ams2"])
def test_an_unconfirmed_boxed_unit_is_left_to_the_bus_check(S, cmd):
    # BridgeBox calls a unit "boxed" until the bus settles AMS 1 vs AMS 2, and
    # there is no ams_model line to fix. Step 5 asks the unit itself.
    assert S.pick_target([_row("Bambu_AMS_1", True, "boxed")], cmd,
                         0x00) == ("Bambu_AMS_1", None, False)


def test_a_unit_at_another_bus_address_is_refused(S):
    # A boxed image is addressed to id 0x00. A unit enrolled second (index 1,
    # id 0x01) would never see the cmd1 -- or another unit at 0x00 would.
    unit, why, _r = S.pick_target([_row("Bambu_AMS_1", True, "ams2", 1)],
                                  "ams2", 0x00)
    assert unit is None and "chain index 1" in why and "enrolls first" in why
    unit, why, _r = S.pick_target([_row("Bambu_AMS_HT_1", True, "ht", 5)],
                                  "ht", 0x80)
    assert unit is None and "0x81" in why


def test_a_unit_left_in_its_loader_is_resumed_by_its_own_command(S):
    # A unit in its bootloader never shows online, so without the record a
    # stopped update could never be finished from the console.
    state = {"model": "ams2", "unit": "Bambu_AMS_1", "ams_id": 0x00,
             "when": "2026-09-26 20:00"}
    rows = [_row("Bambu_AMS_1", False, "ams2")]
    assert S.pick_target(rows, "ams2", 0x00, state) == ("Bambu_AMS_1", None,
                                                        True)
    unit, why, _r = S.pick_target(rows, "ams1", 0x00, state)
    assert unit is None and "AMS 2 Pro update" in why
    unit, why, _r = S.pick_target(rows, "ht", 0x80, dict(state, model="ht"))
    assert unit is None and "different bus address" in why
    # A unit online again means it was power-cycled out of the loader: the
    # ordinary rules apply and the record is ignored.
    rows = [_row("Bambu_AMS_1", True, "ams2")]
    assert S.pick_target(rows, "ams2", 0x00, state) == ("Bambu_AMS_1", None,
                                                        False)


def test_the_state_record_round_trips(S, tmp_path):
    p = str(tmp_path / S.STATE_NAME)
    assert S.read_state(p) is None
    S.write_state(p, model="ht", unit="Bambu_AMS_HT_1", ams_id=0x80, when="t")
    assert S.read_state(p)["unit"] == "Bambu_AMS_HT_1"
    S.clear_state(p)
    assert S.read_state(p) is None
    (tmp_path / S.STATE_NAME).write_text("not json")
    assert S.read_state(p) is None


def test_the_bridge_comes_from_the_live_config(S):
    settings = {
        "afc_bridgebox chain1": {"serial_port": "/dev/serial/by-id/usb-Pico",
                                 "tcp_key": None},
        "mcu": {"serial": "/dev/ttyACM0"},
    }
    assert S.bridge_from_settings(settings) == ("/dev/serial/by-id/usb-Pico",
                                                None)
    settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                             "tcp_key": "k"}}
    assert S.bridge_from_settings(settings) == ("tcp://h:8888", "k")
    assert S.bridge_from_settings({"mcu": {}}) == (None, None)


def test_a_missing_key_published_as_the_string_none_is_no_key(S):
    # AFC_BridgeBox writes str(None) into the unit sections it builds, and
    # Klipper publishes that. Sending "None" as a key fails the auth of a
    # keyless bridge after Klipper has already been stopped.
    settings = {"afc_bridgebox chain1": {"serial_port": "tcp://h:8888"},
                "afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                             "tcp_key": "None"}}
    assert S.bridge_from_settings(settings) == ("tcp://h:8888", None)


def test_the_bridgebox_sections_own_key_wins(S):
    settings = {"afc_bambuams bambu_ams_1": {"serial_port": "tcp://h:8888",
                                             "tcp_key": "stale"},
                "afc_bridgebox chain1": {"serial_port": "tcp://h:8888",
                                         "tcp_key": "real"}}
    assert S.bridge_keys(settings) == {"tcp://h:8888": "real"}


def test_two_bridges_in_the_config_do_not_guess(S):
    settings = {"afc_bridgebox a": {"serial_port": "tcp://a:8888"},
                "afc_bridgebox b": {"serial_port": "/dev/ttyACM1"}}
    with pytest.raises(ValueError, match="TARGET="):
        S.bridge_from_settings(settings)


class _Moon:
    def __init__(self, status):
        self.status = status

    def objects(self):
        return ["AFC_BambuAMS " + n for n in self.status] + ["toolhead"]

    def q(self, obj):
        v = self.status[obj.split(" ", 1)[1]]
        if isinstance(v, Exception):
            raise v
        return v


def test_a_unit_that_cannot_be_read_stops_the_count(S):
    # Dropping it would let a second unit that is online pass the one-unit
    # rule because of a network hiccup.
    moon = _Moon({"Bambu_AMS_1": {"bridge_online": True, "ams_model": "ams2",
                                  "ams_index": 0},
                  "Bambu_AMS_HT_1": IOError("timed out")})
    with pytest.raises(IOError):
        S.unit_rows(moon)
    moon.status["Bambu_AMS_HT_1"] = {}
    with pytest.raises(IOError):
        S.unit_rows(moon)
    moon.status["Bambu_AMS_HT_1"] = {"bridge_online": False,
                                     "ams_model": "ht", "ams_index": 4}
    rows = S.unit_rows(moon)
    assert [(r["name"], r["index"]) for r in rows] == [("Bambu_AMS_1", 0),
                                                       ("Bambu_AMS_HT_1", 4)]


def test_the_usb_link_needs_no_pyserial_and_never_blocks(S, monkeypatch):
    # A USB bridge is what the testers have. The link must work on the system
    # python3 (no pyserial) and hand control back on a quiet port.
    import builtins
    real_import = builtins.__import__

    def no_serial(name, *a, **k):
        if name == "serial":
            raise ImportError("no pyserial here")
        return real_import(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", no_serial)
    master, slave = os.openpty()
    try:
        link = S.Link(os.ttyname(slave), timeout=0.05)
        gen = link.lines()
        assert next(gen) is None                     # quiet: yields, no hang
        os.write(master, b'{"evt":"info"}\n')
        got = None
        for _ in range(20):
            got = next(gen)
            if got:
                break
        assert got == '{"evt":"info"}'
        link.send({"cmd": "info"})
        assert os.read(master, 100) == b'{"cmd": "info"}\n'
        link.close()
    finally:
        os.close(master)
        try:
            os.close(slave)
        except OSError:
            pass


@pytest.mark.parametrize("model", ["ams2", "ht"])
def test_the_shipped_artifacts_pass_the_flashers_own_checks(S, model):
    # The testers flash these files as they are. They must load, verify, add
    # up to their declared image, and be addressed to their own model.
    path = os.path.join(STANDALONE, S.MODELS[model]["artifact"])
    art = S.load_artifact(path)
    total, summed = S.artifact_image_bytes(art)
    assert total - summed == S.BIMH_HEADER_LEN
    assert S.artifact_mismatch(art, model) is None
    other = "ht" if model == "ams2" else "ams2"
    assert S.artifact_mismatch(art, other) is not None


# ── the bus checks at step 5, against a scripted bridge ──────────────────────
# Captured from a real printer asking two boxed units their generation:
# the addressed 0x3702 query to id 0 and to
# id 1, and the AMS 2's answer from id 1. The AMS 1 at id 0 never answered.
QUERY_3702_ID0 = "3d050000120012000700033702000001c4cf"
QUERY_3702_ID1 = "3d050000120012000700033702010001f4f8"
ANSWER_3702_ID1 = ("3d0000002800760003000737020101000000008e151469ae15bc6891"
                   "0da00d930d011900c901eb31")


class _Bridge:
    """A bridge link that answers each command from a script. Lines come back
    in order, then the port goes quiet (None), as the real Link does."""

    def __init__(self, answer):
        self.answer = answer
        self.sent = []
        self.queue = []
        self.dead = False

    def send(self, obj):
        self.sent.append(obj)
        self.queue.extend(self.answer(obj))

    def lines(self):
        while True:
            yield self.queue.pop(0) if self.queue else None


def test_the_generation_query_is_the_captured_frame(S):
    assert S.build_3702(0x00).hex() == QUERY_3702_ID0
    assert S.build_3702(0x01).hex() == QUERY_3702_ID1


def test_only_the_units_own_answer_counts(S):
    echo = bytes.fromhex(QUERY_3702_ID1)
    answer = bytes.fromhex(ANSWER_3702_ID1)
    assert S.answers_3702(echo + answer, 0x01)
    assert not S.answers_3702(echo + answer, 0x00)    # another unit's answer
    assert not S.answers_3702(echo, 0x01)             # our own query
    damaged = bytearray(answer)
    damaged[20] ^= 0xFF
    assert not S.answers_3702(echo + bytes(damaged), 0x01)


def _raw_reply(rx: bytes):
    return [json.dumps({"evt": "raw", "tx": 18, "rx": rx.hex().upper()})
            .replace(" ", "")]


# The bridge's own log-drain poll to id 0 and a unit's idle answer, from
# an AMS 1 insert capture; an AMS 2 answers it the same way.
DRAIN_ID0 = "3d05000013008e000700031a0200000000e4ce"
DRAIN_ANSWER_ID0 = "3d0000001500f4000300071a020000000000003473"


def _answer_3702_id0():
    answer = bytearray(bytes.fromhex(ANSWER_3702_ID1))
    answer[13] = 0x00                                 # from id 0 this time
    c = S_crc16(bytes(answer[:-2]))
    answer[-2:] = bytes((c & 0xFF, c >> 8))
    return bytes(answer)


def S_crc16(b):
    return _standalone().crc16(b)


def _unit(gen_answers: bool, drain_answers: bool):
    """A boxed unit at id 0: answers the drain if listening, and the
    generation query only if it is an AMS 2."""
    def answer(o):
        tx = bytes.fromhex(o["hex"])
        rx = tx
        if o["hex"] == QUERY_3702_ID0 and gen_answers:
            rx += _answer_3702_id0()
        if o["hex"] == DRAIN_ID0 and drain_answers:
            rx += bytes.fromhex(DRAIN_ANSWER_ID0)
        return _raw_reply(rx)
    return answer


def test_the_control_query_is_the_bridges_own_drain(S):
    assert S.build_drain(0x00).hex() == DRAIN_ID0


def test_an_ams2_answers_the_generation_query(S):
    link = _Bridge(_unit(gen_answers=True, drain_answers=True))
    assert S.probe_generation(link, 0x00) == (S.GEN_ASKS, S.GEN_ASKS)
    sent = [o["hex"] for o in link.sent]
    assert sent == [QUERY_3702_ID0, DRAIN_ID0] * S.GEN_ASKS    # interleaved


def test_an_ams1_listens_but_stays_silent_to_the_generation_query(S):
    link = _Bridge(_unit(gen_answers=False, drain_answers=True))
    assert S.probe_generation(link, 0x00) == (0, S.GEN_ASKS)


def test_a_unit_that_answers_nothing_is_not_an_ams1(S):
    # Silence alone used to read as "AMS 1". A unit in its loader, or a bus
    # that is dropping replies, is silent to both.
    link = _Bridge(_unit(gen_answers=False, drain_answers=False))
    assert S.probe_generation(link, 0x00) == (0, 0)


def _status(units):
    return [json.dumps({"evt": "status", "online": bool(units),
                        "units": [{"n": n, "online": on}
                                  for n, on in units]}).replace(" ", "")]


def test_the_bus_count_comes_from_the_bridge_itself(S):
    link = _Bridge(lambda o: _status([(0, True), (1, False), (4, True)]))
    assert S.wire_online(link) == [0, 4]
    assert link.sent == [{"cmd": "status"}]


def test_a_damaged_status_line_is_never_half_read(S):
    bad = ['{"evt":"status","units":[{"n":0,"online":true},{"n":1,"onl']
    link = _Bridge(lambda o: bad)
    with pytest.raises(IOError):
        S.wire_online(link)
    assert len(link.sent) == 3


def test_a_unit_in_its_loader_is_heard_on_the_bus(S):
    def answer(o):
        if o.get("cmd") != "sniff":
            return []
        out = ['{"evt":"sniff_mode","on":%s}' % ("true" if o["on"] else
                                                 "false")]
        if o["on"]:
            out.append('{"evt":"sniff","us":1,"n":41,"hex":"%s"}'
                       % HT_LOADER_REPLY.upper())
        return out
    link = _Bridge(answer)
    assert S.loader_on_bus(link)
    assert link.sent[-1] == {"cmd": "sniff", "on": 0}   # always switched off


def test_a_quiet_bus_has_no_loader(S, monkeypatch):
    t = [0.0]
    monkeypatch.setattr(S.time, "time", lambda: t.__setitem__(0, t[0] + 1)
                        or t[0])
    link = _Bridge(lambda o: ['{"evt":"sniff_mode","on":true}'])
    assert not S.loader_on_bus(link)
    assert link.sent[-1] == {"cmd": "sniff", "on": 0}


def test_the_loader_gate_needs_a_whole_frame(S):
    # A damaged echo, a 06 01 inside another frame's payload, and a match on
    # half a byte all used to read as "the loader answered".
    cmd1 = bytearray(S.build_cmd1(0x0700, 0x00))
    cmd1[10] = 0x0B                                  # source byte hit by noise
    assert S.loader_hits(bytes(cmd1).hex()) == []
    assert S.loader_hits("3d00aa0030601f00") == []
    assert S.loader_hits("3d05a0601b") == []
    assert "ams-origin/op0601" in S.loader_hits(
        S.build_cmd1(0x0700, 0x00).hex() + HT_LOADER_REPLY)


def test_a_damaged_staged_frame_is_staged_again(S):
    # fw >= 1.75 answers "crc":"bad" and sends nothing when a txbuf line was
    # mangled on the way. One frame gets restaged; it must not read as a
    # missing ack, which would cost a whole erase pass.
    sends = []

    def answer(o):
        if o["cmd"] == "txbuf":
            return ['{"evt":"ack","cmd":"txbuf"}']
        sends.append(o)
        if len(sends) == 1:
            return ['{"evt":"txsend","crc":"bad"}']
        return ['{"evt":"txsend","rx":"AABB"}']
    link = _Bridge(answer)
    assert S.send_frame(link, bytes(250)) == b"\xaa\xbb"
    assert len(sends) == 2
    link = _Bridge(lambda o: ['{"evt":"ack","cmd":"txbuf"}']
                   if o["cmd"] == "txbuf" else ['{"evt":"txsend","crc":"bad"}'])
    with pytest.raises(IOError, match="damaged"):
        S.send_frame(link, bytes(10), stage_tries=2)


def test_a_reply_with_a_lost_byte_is_no_answer_not_a_crash(S):
    assert S._rx_of('{"evt":"txsend","rx":"ABC"}') == b""
    assert S._rx_of('{"evt":"txsend","rx":"ABCD"}') == b"\xab\xcd"


def test_a_usb_hangup_is_a_dead_link_not_a_quiet_one(S):
    master, slave = os.openpty()
    link = S.Link(os.ttyname(slave), timeout=0.05)
    os.close(slave)
    os.close(master)
    with pytest.raises(IOError, match="closed"):
        for _ in range(5):
            link._read()
    assert link.dead
    link.close()


def test_a_dead_link_is_reopened_before_the_next_pass(S, monkeypatch):
    link = _Bridge(lambda o: [])
    link.dead = True
    calls = []

    def reconnect():
        calls.append(1)
        link.dead = False

    monkeypatch.setattr(S, "_one_pass", lambda *a: "success")
    assert S.do_flash(link, _artifact(0x0700, 0x00), reconnect=reconnect) == 0
    assert calls == [1]


def test_the_loader_answer_must_come_from_the_addressed_unit(S):
    reply = bytes.fromhex(HT_LOADER_REPLY)            # HT: 0x1800, id 0x80
    assert S.loader_answer(reply, 0x1800, 0x80)
    assert not S.loader_answer(reply, 0x0700, 0x00)   # another unit's loader
    assert not S.loader_answer(S.build_cmd1(0x1800, 0x80), 0x1800, 0x80)
    banner = b"[MCU_UP] Loader Version: 23"
    assert not S.loader_answer(banner, 0x0700, 0x00)  # names no unit
    assert S.loader_answer(b"..[MCU_UP] 0 resev cmd 0x1..", 0x0700, 0x00)
    assert S.loader_answer(b"[MCU_UP] ams 128 wait cmd1!", 0x1800, 0x80)
    assert not S.loader_answer(b"[MCU_UP] 128 resev cmd 0x1", 0x0700, 0x00)
    assert not S.loader_answer(b"[MCU_UP] 10 resev cmd 0x1", 0x0700, 0x00)


# ── main() end to end, against a scripted bridge and Moonraker ───────────────
def _synthetic_artifact(path, image, target=0x0700, ams_id=0x00, nblk=3,
                        per=40):
    """An artifact that passes step 2: addressed, named, and its blocks add
    up to the header's declared size."""
    def body(cmd):
        b = bytearray(40)
        b[0], b[1] = target & 0xFF, target >> 8
        b[2], b[3] = 0x00, 0x09
        b[4], b[5], b[6] = 0x06, 0x01, cmd
        b[7], b[12] = target >> 8, ams_id
        return b
    hdr = body(0x02)
    hdr[16:19] = (nblk * per + 416).to_bytes(3, "little")
    hdr = bytes(hdr) + b"BIMH" + bytes(44) + image.encode() + bytes(8)
    blocks = []
    for _ in range(nblk):
        b = body(0x03)
        b[28:32] = per.to_bytes(4, "little")
        blocks.append(bytes(b).hex())
    with open(path, "w") as f:
        json.dump({"header": hdr.hex(), "blocks": blocks}, f)


class _BusBridge:
    """One boxed unit at index 0 behind a bridge that behaves like the
    firmware where it matters here: online flags are only refreshed while
    the bridge polls, and neither fwreplay nor sniff polls."""

    def __init__(self, model, state="app", fwreplay=False, frozen=None,
                 drain=True, dryrem=0):
        self.model, self.state = model, state     # state: app / loader / gone
        self.fwreplay, self.sniff = fwreplay, False
        self.frozen = frozen                       # flags held since polling
        self.drain, self.dryrem = drain, dryrem
        self.sent, self.queue, self.dead = [], [], False

    def _online(self):
        if self.fwreplay or self.sniff:
            return list(self.frozen or [])
        return [0] if self.state == "app" else []

    def send(self, o):
        self.sent.append(o)
        c = o["cmd"]
        if c == "fwreplay":
            self.fwreplay = bool(o["on"])
            if not self.fwreplay:
                self.frozen = None
            self.queue.append('{"evt":"fwreplay","on":%s}'
                              % ("true" if o["on"] else "false"))
        elif c == "sniff":
            if o["on"]:
                self.frozen = self._online()
            self.sniff = bool(o["on"])
            self.queue.append('{"evt":"sniff_mode","on":%s}'
                              % ("true" if o["on"] else "false"))
        elif c == "status":
            self.queue.append(json.dumps(
                {"evt": "status", "units": [
                    {"n": n, "online": True, "dryrem": self.dryrem}
                    for n in self._online()]}).replace(" ", ""))
        elif c == "raw":
            tx = bytes.fromhex(o["hex"])
            rx = tx
            op = tx[11:13]
            if self.state == "app" and op == b"\x37\x02" \
                    and self.model == "ams2":
                rx += _answer_3702_id0()
            elif self.state == "app" and op == b"\x1a\x02" and self.drain:
                rx += bytes.fromhex(DRAIN_ANSWER_ID0)
            elif op == b"\x06\x01":
                if self.state == "app":
                    self.state = "loader"              # jumps; banner only
                    rx += b"[MCU_UP] Loader Version: 23"
                elif self.state == "loader":
                    rx += b"[MCU_UP] 0 resev cmd 0x1"
            self.queue.append(json.dumps({"evt": "raw", "tx": len(tx),
                                          "rx": rx.hex().upper()})
                              .replace(" ", ""))

    def lines(self):
        while True:
            yield self.queue.pop(0) if self.queue else None

    def close(self):
        pass


class _MainMoon:
    def __init__(self, units):
        self.units = units

    def q(self, obj):
        if obj == "print_stats":
            return {"state": "standby"}
        return self.units[obj.split(" ", 1)[1]]

    def objects(self):
        return ["AFC_BambuAMS " + n for n in self.units]

    def settings(self):
        return {}

    def state(self):
        return "?"                                 # Klipper is stopped


@pytest.fixture
def run_main(tmp_path, monkeypatch):
    """Run a private copy of the flasher's main() (it writes its state file
    beside itself) against a scripted bridge and Moonraker."""
    import importlib.util
    import shutil
    import signal
    cfg = tmp_path / "config"
    cfg.mkdir()
    (cfg / "printer.cfg").write_text("")
    shutil.copy(os.path.join(STANDALONE, "ams_flash.py"), cfg)
    shutil.copy(os.path.join(STANDALONE, "ams2_artifact.json"), cfg)
    _synthetic_artifact(str(cfg / "ams1_artifact.json"),
                        "ams_rev8-firmware-v01.00.06.87-x.bin.sig")
    port = tmp_path / "fakeport"
    port.write_text("")
    spec = importlib.util.spec_from_file_location("ams_flash_run",
                                                  str(cfg / "ams_flash.py"))
    M = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(M)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        monkeypatch.setattr(sys, "stdout", sys.stdout)   # main() tees these
        signal.signal(sig, signal.getsignal(sig))
    old = {s: signal.getsignal(s) for s in (signal.SIGTERM, signal.SIGHUP)}

    def run(model, bridge, units, state=None, sleep=None, write_ok=True):
        calls = {"klipper": [], "flashed": None}
        if state:
            (cfg / M.STATE_NAME).write_text(json.dumps(state))
        clock = [1000.0]

        def tick():
            clock[0] += 0.25
            return clock[0]
        monkeypatch.setattr(M.time, "time", tick)
        monkeypatch.setattr(M.time, "sleep", sleep or (lambda s: None))
        monkeypatch.setattr(M, "Moon", lambda *a, **k: _MainMoon(units))
        monkeypatch.setattr(M, "klipper", lambda moon, action:
                            calls["klipper"].append(action) or True)
        monkeypatch.setattr(M, "wait_ready", lambda *a, **k: True)
        monkeypatch.setattr(M, "Link", lambda target, **k: bridge)
        monkeypatch.setattr(M, "authenticate", lambda link, key: True)
        if not write_ok:
            monkeypatch.setattr(M, "write_state", lambda *a, **k: False)

        def flash(link, art, **k):
            calls["flashed"] = M.artifact_image_name(art)
            for u in units.values():                 # it reboots into the app
                u["bridge_online"] = True
            return 0
        monkeypatch.setattr(M, "do_flash", flash)
        monkeypatch.setattr(sys, "argv", [
            "ams_flash.py", "--detached", "--cfg-dir", str(cfg), "--target",
            str(port), "--model", model, "--mode", "go"])
        monkeypatch.setattr(sys, "stdout", sys.stdout)
        monkeypatch.setattr(sys, "stderr", sys.stderr)
        try:
            calls["rc"] = M.main()
        except SystemExit as e:
            calls["rc"] = f"exit: {e}"
        finally:
            for s, h in old.items():
                signal.signal(s, h)
        calls["state"] = M.read_state(str(cfg / M.STATE_NAME))
        return calls
    return run


def _units(online, model="boxed", index=0):
    return {"Bambu_AMS_1": {"bridge_online": online, "ams_model": model,
                            "ams_index": index, "bridge_fw": "AFC-2.81"}}


def test_main_updates_a_lone_ams2(run_main):
    r = run_main("ams2", _BusBridge("ams2"), _units(True, "ams2"))
    assert r["rc"] == 0 and r["flashed"].startswith("n3f_")
    assert r["klipper"] == ["stop", "start"]
    assert r["state"] is None                        # cleared on success


def test_main_refuses_the_ams2_image_for_an_ams1(run_main):
    r = run_main("ams2", _BusBridge("ams1"), _units(True, "ams2"))
    assert r["rc"] == 1 and r["flashed"] is None
    assert r["klipper"] == ["stop", "start"]


def test_main_does_not_call_a_silent_unit_an_ams1(run_main):
    r = run_main("ams1", _BusBridge("ams2", drain=False), _units(True))
    assert r["rc"] == 1 and r["flashed"] is None


def test_main_resets_a_bridge_left_in_fwreplay(run_main):
    # The review's worst case: a run died in fwreplay with an AMS 2 in its
    # loader, so the bridge still reports it online. Silent to 0x3702, it
    # used to read as an AMS 1 and take the AMS 1 image.
    state = {"model": "ams2", "unit": "Bambu_AMS_1", "ams_id": 0,
             "when": "earlier"}
    stuck = _BusBridge("ams2", state="loader", fwreplay=True, frozen=[0])
    r = run_main("ams1", stuck, _units(True), state=state)
    assert r["rc"] == 1 and r["flashed"] is None
    assert {"cmd": "fwreplay", "on": 0} in stuck.sent
    # The same bridge, the AMS 2's own command: the unit is resumed.
    stuck = _BusBridge("ams2", state="loader", fwreplay=True, frozen=[0])
    r = run_main("ams2", stuck, _units(True), state=state)
    assert r["rc"] == 0 and r["flashed"].startswith("n3f_")


def test_main_resumes_a_unit_left_in_its_loader(run_main):
    state = {"model": "ams2", "unit": "Bambu_AMS_1", "ams_id": 0,
             "when": "earlier"}
    r = run_main("ams2", _BusBridge("ams2", state="loader"), _units(False),
                 state=state)
    assert r["rc"] == 0 and r["flashed"].startswith("n3f_")


def test_main_clears_a_stale_record_when_the_unit_runs_again(run_main):
    # Preflighted, then power-cycled back to normal: the record must not
    # outlive it.
    state = {"model": "ams2", "unit": "Bambu_AMS_1", "ams_id": 0,
             "when": "earlier"}
    r = run_main("ams1", _BusBridge("ams1"), _units(True), state=state,
                 write_ok=True)
    assert r["rc"] == 0 and r["flashed"].startswith("ams_")
    assert r["state"] is None


def test_main_refuses_during_a_dry(run_main):
    r = run_main("ams2", _BusBridge("ams2", dryrem=600), _units(True, "ams2"))
    assert r["rc"] == 1 and r["flashed"] is None


def test_main_does_not_send_an_unrecorded_unit_into_its_loader(run_main):
    bridge = _BusBridge("ams2")
    r = run_main("ams2", bridge, _units(True, "ams2"), write_ok=False)
    assert r["rc"] == 1 and bridge.state == "app"    # no cmd1 went out


def test_main_restarts_klipper_when_stopped_right_after_the_stop(run_main):
    def killed(_s):
        raise SystemExit("stopped by signal 15")
    r = run_main("ams2", _BusBridge("ams2"), _units(True, "ams2"),
                 sleep=killed)
    assert r["klipper"] == ["stop", "start"]
