# ONE MODEL VOCABULARY ACROSS BOTH MODULES.
#
# AFC_BridgeBox derives its model tables from AFC_BambuAMS's _AMS_MODELS, so
# a roster tag, an [AFC_BridgeBox <model>] override and a unit's ams_model all
# use the same names. These tests pin that, so an override for a model can
# never silently match nothing.
from __future__ import annotations

import ast
import inspect

from extras.AFC_BridgeBox import (_DRY_CEILING, _HEATED_MODELS,
                                  _SLOTS_BY_MODEL, _norm_model)


def test_model_names_are_case_and_space_insensitive():
    assert _norm_model("  HT ") == "ht"
    assert _norm_model("AMS2") == "ams2"
    assert _norm_model(None) == ""


def test_unknown_names_are_not_coerced_to_a_model():
    # A typo must stay unknown, so it becomes an error, not a wrong unit.
    for name in ("amsht", "ams2pro", "ams"):
        assert _norm_model(name) not in _SLOTS_BY_MODEL


def test_bridgebox_tables_follow_the_unit_model_table():
    from extras.AFC_BambuAMS import _AMS_MODELS
    assert set(_SLOTS_BY_MODEL) == set(_AMS_MODELS)
    assert _SLOTS_BY_MODEL["ht"] == 1
    assert all(n == 4 for m, n in _SLOTS_BY_MODEL.items() if m != "ht")
    assert _HEATED_MODELS == {"ams2", "ht"}
    assert _DRY_CEILING == {"ams2": 65, "ht": 85}


def test_bambuams_accepts_every_model_bridgebox_can_emit():
    # BridgeBox writes the roster tag straight into the section it fabricates
    # ("ams_model": b["model"]), so every tag it can emit -- including its own
    # `boxed` and `lite` -- must be a model AFC_BambuAMS knows.
    from extras.AFC_BambuAMS import _AMS_MODELS, _MC_ADDRESSING
    for model in _SLOTS_BY_MODEL:
        assert model in _AMS_MODELS, (
            f"BridgeBox can emit ams_model: {model}, which AFC_BambuAMS does "
            f"not know")
        assert model in _MC_ADDRESSING, (
            f"no MC addressing for {model}: its polls would go to the frame's "
            f"captured address instead of the unit's own device")


def test_unknown_ams_model_raises_instead_of_defaulting_to_ams2():
    # Source check: a .get(..., _AMS_MODELS["ams2"]) fallback turns a typo into
    # a heated boxed unit at the wrong device address, silently. AST so this
    # comment does not itself satisfy the search.
    from extras import AFC_BambuAMS
    src = inspect.getsource(AFC_BambuAMS)
    tree = ast.parse(src)
    bad = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "get"
                and isinstance(node.func.value, ast.Name)
                and node.func.value.id == "_AMS_MODELS"
                and len(node.args) > 1):
            bad.append(node.lineno)
    assert not bad, (
        f"_AMS_MODELS.get() with a default at line(s) {bad}: an unknown "
        f"ams_model must raise, not silently become another model")


def test_a_model_section_with_no_unit_plugged_in_is_not_reported_as_unknown():
    # THE FALSE POSITIVE THIS EXISTS FOR. The "overrides nothing" warning was
    # keyed on "did this apply to a fabricated unit", so a chain holding an
    # ams2 and an ht reported
    #
    #   [AFC_BridgeBox ams1] overrides nothing -- no unit and no model by that
    #   name ... Models: ams1, ams2, boxed, ht, lite
    #
    # -- naming ams1 as invalid in a sentence that lists ams1 as valid. A model
    # default for hardware that is not currently plugged in is not a mistake;
    # it starts working the moment such a unit is claimed. The test is whether
    # the NAME is known, not whether it matched something today.
    import inspect

    from extras.AFC_BridgeBox import afcBridgeBox
    src = inspect.getsource(afcBridgeBox._fold_and_sweep)
    i = src.index("_unmatched_overrides")
    window = src[max(0, i - 900):i + 200]
    assert "_SLOTS_BY_MODEL" in window, (
        "the unmatched-override check does not treat every known model as a "
        "valid target, so a model section with no unit present is reported "
        "as an unknown name")
