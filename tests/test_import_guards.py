# Tests for the house-style import guards.
#
# Every AFC module wraps its intra-package imports as
#
#     try: from extras.AFC_lane import ...
#     except: raise error(ERROR_STR.format(import_lib="AFC_lane", trace=...))
#
# so a broken or partial install fails with the NAME of the module that could
# not be loaded plus a traceback, instead of a bare ImportError from somewhere
# deep in Klipper's config loading. That is a claim worth checking: a guard that
# swallowed, renamed or lost the failing module would be worse than none, and
# nothing else in the suite ever executes these lines.
#
# Each case blocks one dependency at the import machinery level and re-imports
# the module from source, which is the only way to reach an import-time branch.
from __future__ import annotations

import importlib
import importlib.util
import sys
from configparser import Error as config_error

import pytest


class _Blocker:
    """A meta-path finder that makes exactly one module unimportable."""

    def __init__(self, blocked):
        self.blocked = blocked

    def find_spec(self, name, path=None, target=None):
        if name == self.blocked:
            raise ImportError("blocked for test: %s" % name)
        return None


def _import_fresh(module_name, blocked):
    """Import `module_name` from source with `blocked` unimportable."""
    saved_modules = dict(sys.modules)
    saved_meta = list(sys.meta_path)
    # Drop the target and its dependency so the import really re-executes.
    for n in (module_name, blocked):
        sys.modules.pop(n, None)
    sys.meta_path.insert(0, _Blocker(blocked))
    try:
        importlib.import_module(module_name)
    finally:
        sys.meta_path[:] = saved_meta
        sys.modules.clear()
        sys.modules.update(saved_modules)


# (module under test, dependency to break, name expected in the message)
#
# Only AFC_BambuAMS on this branch: it is the module the transport split gave
# guards to. The wider set of guarded modules lives on the branches that
# carry the upstream house-style pass, and asserting them here would be
# testing a property these files do not claim to have.
CASES = [
    ("extras.AFC_BambuAMS", "extras.AFC_BambuAMS_bridge", "AFC_BambuAMS_bridge"),
    # AFC_utils holds ERROR_STR itself, so the FIRST guard has to format its
    # own message rather than use it.
    ("extras.AFC_BambuAMS", "extras.AFC_utils", "AFC_utils"),
]


@pytest.mark.parametrize("module,blocked,expected", CASES,
                         ids=[f"{m.split('.')[-1]}-needs-{e}"
                              for m, _b, e in CASES])
def test_a_missing_dependency_names_itself(module, blocked, expected):
    with pytest.raises(config_error) as exc:
        _import_fresh(module, blocked)
    msg = str(exc.value)
    assert expected in msg, msg
    # The traceback is the useful half: without it the operator knows WHICH
    # module failed but not why (missing file vs a syntax error inside it).
    assert "Traceback" in msg or "Error" in msg


def test_a_missing_AFC_utils_is_reported_without_needing_ERROR_STR():
    # ERROR_STR lives in AFC_utils, so the guard for AFC_utils itself cannot
    # use it -- it formats its own message. If that ever regressed the failure
    # would be a NameError instead of a config error.
    with pytest.raises(config_error) as exc:
        _import_fresh("extras.AFC_BambuAMS", "extras.AFC_utils")
    msg = str(exc.value)
    assert "AFC_utils" in msg, msg
    # The distinction that matters: a config error the operator can read, not
    # the NameError that using ERROR_STR here would produce.
    assert "NameError" not in msg, msg


def test_the_guards_do_not_fire_on_a_healthy_install():
    # Sanity: the same modules import cleanly with nothing blocked, so the
    # tests above are demonstrating the guard rather than a broken package.
    for module, _blocked, _expected in CASES:
        importlib.import_module(module)
