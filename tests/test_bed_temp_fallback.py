"""Where a lane's bed temperature comes from, and which claim it is making.

A Bambu tag HAS a bed-temperature field and nearly always writes zero into it.
Mined across the whole capture corpus plus two live reads -- 14 distinct tag
records -- exactly three state one, all of them (type 1, 35 C), the Cool Plate
figure. That looked like a decode bug from this side for a while and is not:
the neighbouring fields in the same frame read correctly on every record, and
sweeping both live frames for an unclaimed value in the bed-temperature range
turns up only bytes already spoken for.

So the number is derived from the material when the tag is silent, and the two
cases must stay distinguishable -- "this spool says 35 C" and "PLA usually
wants 55 C" are different claims, and a lane that blurs them is worse than one
that says nothing.
"""
from __future__ import annotations

import ast
import pathlib
import types

ROOT = pathlib.Path(__file__).resolve().parents[1]
BAMBU = ROOT / "extras" / "AFC_BambuAMS.py"
RFID = ROOT / "extras" / "AFC_RFID.py"


def _load(path: pathlib.Path, *names, extra_globals=None):
    """Lift top-level functions/dicts out of a module without importing it.

    Both modules need Klipper to import; these pieces need nothing.
    """
    tree = ast.parse(path.read_text())
    wanted, ns = [], dict(extra_globals or {})
    ns.setdefault("Optional", __import__("typing").Optional)
    ns.setdefault("Any", __import__("typing").Any)
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef,)) and node.name in names:
            wanted.append(node)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id in names:
                    wanted.append(node)
    missing = set(names) - {
        getattr(n, "name", None) or n.targets[0].id for n in wanted}
    assert not missing, f"{path.name} no longer defines {sorted(missing)}"
    exec(compile(ast.Module(body=wanted, type_ignores=[]), str(path), "exec"), ns)
    return types.SimpleNamespace(**{n: ns[n] for n in names})


def _slot_mapper():
    """bridge_slot_to_info, wired to the real helpers it calls."""
    rfid = _load(RFID, "bed_temp_for_material", "MATERIAL_BED_TEMP")
    mod = _load(
        BAMBU, "bridge_slot_to_info", "bridge_color_to_rgb",
        extra_globals={"bed_temp_for_material": rfid.bed_temp_for_material})
    return mod.bridge_slot_to_info, rfid


def _slot(**kw):
    base = {"i": 0, "present": True, "state": "idle", "material": "PLA Basic",
            "sku": "GFA00", "color": "0086D6FF", "tmin": 190, "tmax": 230,
            "weight": 1000, "remain": 60, "bedt": 0, "uid": "95f2c30c",
            "tray_uid": "4e" * 16}
    base.update(kw)
    return base


def test_a_tag_that_states_a_bed_temperature_wins_and_says_so():
    """The three corpus records that carry one: (type 1, 35 C)."""
    info, _ = _slot_mapper()[0], None
    out = info(_slot(bedt=35))
    assert out["bed_temp"] == 35
    assert out["bed_temp_source"] == "tag"


def test_a_silent_tag_falls_back_to_the_material_and_says_so():
    """The other eleven, including both spools read live on 2026-09-21."""
    info = _slot_mapper()[0]
    pla = info(_slot(material="PLA Basic", bedt=0))
    assert pla["bed_temp"] == 55
    assert pla["bed_temp_source"] == "material"

    abs_ = info(_slot(material="ABS", bedt=0, tmin=240, tmax=270))
    assert abs_["bed_temp"] == 90
    assert abs_["bed_temp_source"] == "material"


def test_an_untagged_bay_claims_nothing():
    """No material, no guess -- the bay the AMS read no tag from."""
    info = _slot_mapper()[0]
    out = info(_slot(material=None, sku=None, bedt=0, uid=""))
    assert out["bed_temp"] is None
    assert out["bed_temp_source"] is None


def test_an_unknown_material_claims_nothing_rather_than_guessing_pla():
    """Density has a defensible generic default. A bed temperature does not.

    Putting 55 C under an unnamed engineering polymer would be a number nobody
    measured, handed to someone with no way to check it.
    """
    _, rfid = _slot_mapper()
    assert rfid.bed_temp_for_material("PEEK") is None
    assert rfid.bed_temp_for_material("Snapmaker Mystery") is None
    assert rfid.bed_temp_for_material("") is None


def test_separator_and_case_handling_matches_the_density_table():
    """The two tables must never disagree about what a material string means."""
    _, rfid = _slot_mapper()
    for spelling in ("PETG-CF", "petg cf", "petg_cf", "PETG/CF", "PetgCf"):
        assert rfid.bed_temp_for_material(spelling) == 70, spelling
    # Longest-prefix, so a variant falls back to its base rather than missing.
    assert rfid.bed_temp_for_material("PLA Basic") == 55
    assert rfid.bed_temp_for_material("ABS Something") == 90


def test_a_derived_value_only_fills_a_blank_but_the_tags_own_overwrites():
    """The write rule, read out of the source it is written in.

    A derived figure must not stomp one somebody set -- Spoolman's, saved
    vars', or a hand-edited lane. The tag's own figure is on the same footing
    as the nozzle temp beside it and does overwrite.
    """
    src = BAMBU.read_text()
    i = src.index("# Bed temp: most tags leave it blank")
    block = src[i:i + 1400]
    assert 'bed_temp_source") == "tag"' in block, (
        "the tag/derived distinction is gone from the write rule -- a material "
        "table lookup can now overwrite a bed temperature somebody set")
    assert 'getattr(lane, "bed_temp", None) is None' in block, (
        "the blank-only guard is gone -- a derived bed temperature will stomp "
        "an existing lane value")


def test_a_zero_is_not_published_as_a_zero_degree_bed():
    """0 in the field means 'not stated'. It must never reach a lane as 0 C."""
    info = _slot_mapper()[0]
    out = info(_slot(material="PLA Basic", bedt=0))
    assert out["bed_temp"] != 0
    # And an out-of-range reading is refused the same way, rather than
    # overriding the table with nonsense.
    assert info(_slot(bedt=-5))["bed_temp_source"] == "material"
    assert info(_slot(bedt=900))["bed_temp_source"] == "material"
