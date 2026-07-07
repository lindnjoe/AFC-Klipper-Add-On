"""
Unit tests for the pure helper functions in extras/AFC_RFID.py

These feed both the ACE RFID inventory and the U1 RFID scanner paths:
color naming/labels/distance/matching, material density lookup, RGB/hex
conversion, UID normalization, and bed-temp defaults.
"""

from __future__ import annotations

from extras.AFC_RFID import (
    color_name,
    color_label,
    color_distance,
    colors_match,
    density_for_material,
    rgb_array_to_hex,
    default_bed_temp_for_material,
    _norm_uid,
)


# ── color_distance / colors_match ─────────────────────────────────────────────

def test_color_distance_identical_is_zero():
    assert color_distance("FF0000", "FF0000") == 0.0


def test_color_distance_orders_sensibly():
    near = color_distance("FF0000", "FE0100")
    far = color_distance("000000", "FFFFFF")
    assert 0 < near < far


def test_colors_match_exact_and_tolerance():
    assert colors_match(["FF0000"], ["FF0000"]) is True
    # Slightly-off shade matches with a generous tolerance, not with zero
    assert colors_match(["FF0000"], ["F80402"], tol=30.0) is True
    assert colors_match(["FF0000"], ["00FF00"], tol=30.0) is False


def test_colors_match_multi_color_order_free():
    assert colors_match(["FF0000", "0000FF"], ["0000FF", "FF0000"]) is True
    # Different lengths never match
    assert colors_match(["FF0000"], ["FF0000", "0000FF"]) is False


# ── color_name / color_label ──────────────────────────────────────────────────

def test_color_name_primary_colors():
    assert color_name("FF0000").lower() == "red"
    assert color_name("000000").lower() == "black"
    assert color_name("FFFFFF").lower() == "white"


def test_color_label_single_and_multi():
    single = color_label(["FF0000"])
    assert "red" in single.lower()
    multi = color_label(["FF0000", "0000FF"])
    assert "red" in multi.lower() and "blue" in multi.lower()


def test_color_label_empty():
    assert color_label([]) == ""


# ── density_for_material ──────────────────────────────────────────────────────

def test_density_known_materials():
    assert density_for_material("PLA") == 1.24
    assert density_for_material("pla") == 1.24


def test_density_separator_and_case_insensitive():
    """'PLA-CF', 'pla cf', 'pla_cf' all normalize to the same key."""
    d = density_for_material("PLA-CF")
    assert d == density_for_material("pla cf") == density_for_material("pla_cf")


def test_density_prefix_fallback():
    """Unknown variants fall back to the longest matching base material."""
    assert density_for_material("PLA Silk Rainbow") == density_for_material("PLA Silk Rainbow".replace(" ", ""))


def test_density_unknown_defaults_to_pla():
    assert density_for_material("unobtainium") == 1.24
    assert density_for_material("") == 1.24
    assert density_for_material(None) == 1.24


# ── rgb_array_to_hex ──────────────────────────────────────────────────────────

def test_rgb_array_to_hex():
    assert rgb_array_to_hex([255, 0, 0]) == "#ff0000"
    assert rgb_array_to_hex((0, 128, 255)) == "#0080ff"


def test_rgb_array_to_hex_invalid_input():
    assert rgb_array_to_hex(None) == "#000000"
    assert rgb_array_to_hex([255]) == "#000000"
    assert rgb_array_to_hex("FF0000") == "#000000"


# ── _norm_uid ─────────────────────────────────────────────────────────────────

def test_norm_uid_separator_and_case_insensitive():
    """'E5:CA:F0:A1', 'e5-ca-f0-a1' and 'E5CAF0A1' all compare equal."""
    assert _norm_uid("E5:CA:F0:A1") == "E5CAF0A1"
    assert _norm_uid("e5-ca-f0-a1") == "E5CAF0A1"
    assert _norm_uid("e5 ca f0 a1") == "E5CAF0A1"
    assert _norm_uid("") == ""
    assert _norm_uid(None) == ""


def test_norm_uid_distinct_uids_differ():
    assert _norm_uid("56A36AEA") != _norm_uid("26A36AEA")


# ── default_bed_temp_for_material ─────────────────────────────────────────────

def test_bed_temp_defaults():
    pla = default_bed_temp_for_material("PLA")
    abs_temp = default_bed_temp_for_material("ABS")
    assert pla and abs_temp
    assert abs_temp > pla  # ABS beds run hotter than PLA
