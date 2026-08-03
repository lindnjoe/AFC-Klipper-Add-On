"""
Unit tests for extras/AFC_rfid_keys.py

Covers:
  - _hex_key: parses hex to bytes, None when unset, config error on bad hex
  - AFC_rfid_keys: parses all three shared keys off the config section
  - load_config: returns the shared-keys object
"""

from __future__ import annotations

import pytest

from extras.AFC_rfid_keys import AFC_rfid_keys, load_config, _hex_key


class _ConfigError(Exception):
    pass


class _Config:
    """Minimal Klipper config stand-in: get() + error()."""

    def __init__(self, opts=None):
        self._opts = opts or {}

    def get(self, key, default=None):
        return self._opts.get(key, default)

    def error(self, msg):
        return _ConfigError(msg)


# ── _hex_key ──────────────────────────────────────────────────────────────────

def test_hex_key_parses_to_bytes():
    cfg = _Config({"k": "aabbcc"})
    assert _hex_key(cfg, "k") == b"\xaa\xbb\xcc"


def test_hex_key_unset_is_none():
    assert _hex_key(_Config(), "missing") is None
    assert _hex_key(_Config({"k": "   "}), "k") is None    # blank -> None


def test_hex_key_bad_hex_raises_config_error():
    with pytest.raises(_ConfigError):
        _hex_key(_Config({"k": "nothex!"}), "k")


# ── AFC_rfid_keys ─────────────────────────────────────────────────────────────

def test_keys_parsed_from_section():
    obj = AFC_rfid_keys(_Config({
        "bambu_master_key": "00112233445566778899aabbccddeeff",
        "creality_key": "0f0e0d",
        # creality_encryption_key left unset
    }))
    assert obj.bambu_master_key == bytes.fromhex("00112233445566778899aabbccddeeff")
    assert obj.creality_key == b"\x0f\x0e\x0d"
    assert obj.creality_encryption_key is None


def test_load_config_returns_keys_object():
    obj = load_config(_Config({"bambu_master_key": "abcd"}))
    assert isinstance(obj, AFC_rfid_keys)
    assert obj.bambu_master_key == b"\xab\xcd"
