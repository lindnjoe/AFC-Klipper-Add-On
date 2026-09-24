"""extras/ is shipped as standalone GPL source and must not import from or name
paths in Firmwares/. Only the AMS updater's own test may cross.
"""
from __future__ import annotations

import ast
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]

#: The firmware and flashing-tool tree. Anything naming this path is on the
#: wrong side of the release boundary if it also ships.
PRIVATE = "Firmwares"

#: Test modules that legitimately reach across because they test the AMS
#: updater in Firmwares/Bambu_AMS/ itself, so only its test crosses.
ALLOWED_TESTS = {
                 # Pins the AMS flash's per-model addressing and the loader
                 # gate that stands in front of the erase. It loads the AMS
                 # updater's own ams_flash.py, so it travels with it.
                 "test_ams_flash_addressing.py"}


def _mentions_private(path: pathlib.Path) -> bool:
    """Whether this file DEPENDS on Firmwares/, as opposed to mentioning it.

    Comments and docstrings do not count: a released module explaining what
    it talks to is not the same as one that cannot be built without it.
    Imports and path literals do count, because those break the moment the
    tree is gone.
    """
    if path.name == "test_repo_boundary.py":
        return False
    try:
        tree = ast.parse(path.read_text(errors="replace"))
    except SyntaxError:
        return False

    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            body = getattr(node, "body", None)
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                docstrings.add(id(body[0].value))

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            if any(PRIVATE in (a.name or "") for a in node.names):
                return True
        elif isinstance(node, ast.ImportFrom):
            if PRIVATE in (node.module or ""):
                return True
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            if id(node) not in docstrings and PRIVATE in node.value:
                return True
    return False


def test_no_shipped_module_references_the_firmware_tree():
    """extras/ is what ships. It must stand alone."""
    offenders = sorted(
        p.relative_to(ROOT).as_posix()
        for p in (ROOT / "extras").glob("*.py")
        if _mentions_private(p))
    assert offenders == [], (
        "these ship as GPL source and import from or name paths in "
        "Firmwares/, so they no longer stand alone: %s"
        % offenders)


def test_only_the_known_tests_cross_the_boundary():
    """Tests are allowed across, but only the ones for Firmwares/."""
    crossing = sorted(
        p.name for p in (ROOT / "tests").glob("*.py")
        if _mentions_private(p))
    unexpected = [n for n in crossing if n not in ALLOWED_TESTS]
    assert unexpected == [], (
        "new test(s) reaching into Firmwares/: %s. If they "
        "test the tools there, add them to ALLOWED_TESTS; if they test the "
        "shipped module, they must not need it." % unexpected)


def test_the_allowlist_is_not_stale():
    """An allowlist entry that stopped crossing should be removed, or it hides
    the next one that starts."""
    for name in ALLOWED_TESTS:
        p = ROOT / "tests" / name
        assert p.exists(), f"ALLOWED_TESTS names a missing file: {name}"
        assert _mentions_private(p), (
            f"{name} no longer references {PRIVATE}; drop it from "
            f"ALLOWED_TESTS so the guard stays meaningful")
