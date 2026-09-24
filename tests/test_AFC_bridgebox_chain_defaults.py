# Chain-wide unit defaults, written on the master section.
#
# Fabricated units have no hand-written [AFC_BambuAMS <name>] section to edit,
# so a setting that applies to the whole chain (auto_spoolman_create is the one
# that prompted this) previously had to be repeated in every model section.
# Any option on the master that the master does not consume itself is now a
# default for every unit it fabricates, with model and per-unit sections still
# overriding it.
from __future__ import annotations

import inspect
import re

from extras import AFC_BridgeBox
from extras.AFC_BridgeBox import _MASTER_OPTIONS, _STRUCTURAL_KEYS, afcBridgeBox


def test_init_never_touches_self_logger():
    # THE BUG THIS EXISTS FOR. self.logger is assigned from the AFC object at
    # connect, hundreds of lines after __init__, so ANY use of it in __init__
    # is a crash waiting for the config that reaches it. Logging the chain
    # defaults there halted a printer with "'afcBridgeBox' object has no
    # attribute 'logger'" the moment one was actually set -- config-time, so
    # the printer would not start at all. AST, not a text search, so the
    # comment explaining this does not itself trip the check.
    import ast
    src = inspect.getsource(AFC_BridgeBox)
    tree = ast.parse(src)
    hits = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.ClassDef) and node.name == "afcBridgeBox"):
            continue
        for fn in node.body:
            if not (isinstance(fn, ast.FunctionDef) and fn.name == "__init__"):
                continue
            for sub in ast.walk(fn):
                if (isinstance(sub, ast.Attribute) and sub.attr == "logger"
                        and isinstance(sub.value, ast.Name)
                        and sub.value.id == "self"):
                    hits.append(sub.lineno)
    assert not hits, (
        f"__init__ uses self.logger at line(s) {hits}; it does not exist until "
        f"connect")


def test_the_master_option_list_matches_what_init_actually_reads():
    # THE GUARD THAT KEEPS THIS HONEST. Adding an option to the master without
    # adding it to _MASTER_OPTIONS would silently start broadcasting the
    # master's own setting into every unit section.
    src = inspect.getsource(afcBridgeBox.__init__)
    read = set(re.findall(r'config\.get\w*\(\s*"([a-z_0-9]+)"', src))
    missing = read - _MASTER_OPTIONS - _STRUCTURAL_KEYS
    assert not missing, (
        f"__init__ reads {sorted(missing)} but they are not in _MASTER_OPTIONS, "
        f"so they would leak into every fabricated unit as a default")


def test_the_fence_covers_the_masters_identity_options():
    for opt in ("serial_port", "pool_ams", "pool_ht", "ams_names", "ht_names",
                "roster", "state_file", "lane_base"):
        assert opt in _MASTER_OPTIONS, f"{opt} must never reach a unit section"


def _fold_layers(chain, model, unit):
    """The overlay order as _fold applies it: chain < model < unit."""
    ov = {}
    ov.update(chain or {})
    ov.update(model or {})
    ov.update(unit or {})
    return ov


def test_a_chain_default_reaches_a_unit_with_no_other_override():
    ov = _fold_layers({"auto_spoolman_create": "True"}, None, None)
    assert ov["auto_spoolman_create"] == "True"


def test_a_model_section_overrides_the_chain_default():
    ov = _fold_layers({"auto_spoolman_create": "True"},
                      {"auto_spoolman_create": "False"}, None)
    assert ov["auto_spoolman_create"] == "False"


def test_a_unit_section_overrides_both():
    ov = _fold_layers({"auto_spoolman_create": "True"},
                      {"auto_spoolman_create": "False"},
                      {"auto_spoolman_create": "True"})
    assert ov["auto_spoolman_create"] == "True"


def test_the_fold_really_applies_the_chain_layer_first():
    # Guards the source: the chain layer must be applied before the model and
    # unit layers, or the precedence above is a fiction.
    src = inspect.getsource(afcBridgeBox._fold_and_sweep)
    i_chain = src.index("_chain_defaults")
    # Anchor on the "model:" key itself, not on a whole `overrides.get("model:`
    # call: the model key is now built a line earlier (mkey = "model:" + the
    # canonicalised ams_model, so an alias still matches its override) and the
    # single-expression form no longer exists. The ORDER is what this guards.
    i_model = src.index('"model:"', i_chain)
    i_unit = src.index("overrides.get(name)", i_model)
    assert i_chain < i_model < i_unit
