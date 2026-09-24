"""
Unit tests for the staged OpenRFID BQ Tech processor
(contrib/openrfid-bqtech/src/tag/bqtech/processor.py).

OpenRFID isn't a dependency of this repo, so the handful of OpenRFID modules the
processor imports (filament, reader, tag.binary, tag.tag_types, the MIFARE base
class) are stubbed into sys.modules with faithful minimal implementations. This
verifies the processor's decode offsets against the SAME synthetic BQ Tech image
the decode_btt tests use, catching any drift between the two.
"""
from __future__ import annotations

import importlib
import logging
import os
import struct
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BQ_SRC = os.path.join(ROOT, "contrib", "openrfid-bqtech", "src")

VALID_BASE_MATERIALS = {"PLA", "PET", "PETG", "ABS", "ASA", "TPU", "PC", "PA"}


class _GenericFilament:
    def __init__(self, **kw):
        if kw["type"] not in VALID_BASE_MATERIALS:
            raise ValueError("Invalid filament type: %s" % kw["type"])
        self.__dict__.update(kw)

    @staticmethod
    def generate_unique_id(*args):
        return "uid|" + "|".join(str(a) for a in args)


class _TagType:
    Unknown = 0
    MifareClassic1k = 8
    MifareUltralight = 1


class _ScanResult:
    def __init__(self, tag_type, uid):
        self.tag_type = tag_type
        self.uid = uid


class _TagAuthentication:
    def __init__(self, hkdf_key_a, hkdf_key_b):
        self.hkdf_key_a = hkdf_key_a
        self.hkdf_key_b = hkdf_key_b


class _MifareClassicTagProcessor:
    def __init__(self, config):
        self.name = config["__name"]
        self.config = config
        self.enabled = str(config.get("enabled", "true")).lower() == "true"
        self.logger = logging.getLogger("test.bqtech")


def _install_openrfid_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    mod("filament", GenericFilament=_GenericFilament)
    mod("filament.valid_materials", VALID_BASE_MATERIALS=VALID_BASE_MATERIALS)
    mod("reader")
    mod("reader.scan_result", ScanResult=_ScanResult)
    mod("tag.binary",
        extract_string=lambda d, p, l: d[p:p + l].split(b"\x00")[0].decode("ascii", "replace"),
        extract_uint16_le=lambda d, p: struct.unpack_from("<H", d, p)[0])
    mod("tag.tag_types", TagType=_TagType)
    mod("tag.mifare_classic_tag_processor",
        MifareClassicTagProcessor=_MifareClassicTagProcessor,
        TagAuthentication=_TagAuthentication)
    # `tag` is a real package on disk (contains bqtech) but its OTHER submodules
    # are the stubs above, make it a package rooted at the contrib src tree.
    tag_pkg = mod("tag")
    tag_pkg.__path__ = [os.path.join(BQ_SRC, "tag")]


@pytest.fixture()
def processor_cls():
    _install_openrfid_stubs()
    if BQ_SRC not in sys.path:
        sys.path.insert(0, BQ_SRC)
    # Drop any cached bqtech package so it re-imports against fresh stubs.
    for k in [m for m in sys.modules if m.startswith("tag.bqtech")]:
        del sys.modules[k]
    mod = importlib.import_module("tag.bqtech.processor")
    return mod.BqTechTagProcessor


def _le16(v):
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def _build_btt_image(version=1000, manufacturer="BQ Tech", mfg="20240812_162600",
                     material="PET", detailed="PET (CEP)", serial="IP243ZCXV67",
                     rgb=(0x12, 0x34, 0x56), diameter=1750, weight=1000,
                     ptmin=200, ptmax=240, bed=60):
    d = bytearray(1024)

    def put(block, off, raw):
        p = block * 16 + off
        d[p:p + len(raw)] = raw

    put(1, 0, _le16(version))
    put(1, 2, manufacturer.encode("ascii"))
    put(2, 0, mfg.encode("ascii"))
    put(4, 0, material.encode("ascii"))
    put(5, 0, detailed.encode("ascii"))
    put(6, 0, serial.encode("ascii"))
    put(8, 0, bytes(rgb))
    put(10, 0, _le16(diameter))
    put(17, 0, _le16(weight))
    put(18, 10, _le16(ptmin))
    put(18, 12, _le16(ptmax))
    put(20, 0, _le16(bed))
    return bytes(d)


def _proc(processor_cls):
    return processor_cls({"__name": "bqtech_tag_processor"})


def test_authenticate_returns_all_default_ff_keys(processor_cls):
    p = _proc(processor_cls)
    scan = _ScanResult(_TagType.MifareClassic1k, b"\xaa\xbb\xcc\xdd")
    auth = p.authenticate_tag(scan)
    assert auth is not None
    assert auth.hkdf_key_a == [[0xFF] * 6] * 16
    assert auth.hkdf_key_b == [[0xFF] * 6] * 16


def test_authenticate_rejects_non_classic(processor_cls):
    p = _proc(processor_cls)
    scan = _ScanResult(_TagType.MifareUltralight, b"\x01\x02\x03\x04")
    assert p.authenticate_tag(scan) is None


def test_process_tag_decodes_fields(processor_cls):
    p = _proc(processor_cls)
    scan = _ScanResult(_TagType.MifareClassic1k, b"\xaa\xbb\xcc\xdd")
    fil = p.process_tag(scan, _build_btt_image())
    assert fil is not None
    assert fil.manufacturer == "BQ Tech"
    assert fil.type == "PET"
    assert fil.colors == [0xFF123456]
    assert round(fil.diameter_mm, 3) == 1.75
    assert fil.weight_grams == 1000
    assert fil.hotend_min_temp_c == 200 and fil.hotend_max_temp_c == 240
    assert fil.bed_temp_c == 60
    assert fil.manufacturing_date == "2024-08-12"


def test_process_tag_fingerprint_rejects_non_btt(processor_cls):
    p = _proc(processor_cls)
    scan = _ScanResult(_TagType.MifareClassic1k, b"\xaa\xbb\xcc\xdd")
    # tag_version != 1000 -> not a BQ Tech tag even though the FF key authed.
    assert p.process_tag(scan, _build_btt_image(version=7)) is None
    assert p.process_tag(scan, bytes(1024)) is None


def test_process_tag_unknown_material_skips(processor_cls):
    p = _proc(processor_cls)
    scan = _ScanResult(_TagType.MifareClassic1k, b"\xaa\xbb\xcc\xdd")
    # A material not in VALID_BASE_MATERIALS must not raise, just return None.
    assert p.process_tag(scan, _build_btt_image(material="UNOBTAINIUM")) is None
