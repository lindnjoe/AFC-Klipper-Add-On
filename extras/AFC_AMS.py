"""AMS integration layer for AFC.

This module replaces a subset of AFC core modules with
AMS-aware implementations sourced from :mod:`extras.afc_ams_bundle`.
It keeps the upstream modules untouched on disk while dynamically
loading the bundled overrides at runtime.
"""

from __future__ import annotations

import importlib.util
import sys
from importlib.machinery import SourceFileLoader
from pathlib import Path
from types import ModuleType
from typing import Dict, Iterable

from extras.afc_ams_bundle import BUNDLE_ROOT

# Ordered list of modules that need to be overridden. The order matches the
# original bundler so dependencies are satisfied during import.
_MODULE_ORDER: Iterable[str] = [
    'extras.AFC_utils',
    'extras.AFC_functions',
    'extras.AFC_logger',
    'extras.AFC_spool',
    'extras.AFC_buffer',
    'extras.AFC_extruder',
    'extras.AFC_prep',
    'extras.AFC_unit',
    'extras.AFC_lane',
    'extras.AFC_hub',
    'extras.AFC_BoxTurtle',
    'extras.AFC',
    'extras.AFC_error',
]


def _module_filename(module_name: str) -> Path:
    """Return the path to the bundled source file for ``module_name``."""
    short_name = module_name.split('.', 1)[1]
    return BUNDLE_ROOT / f"{short_name}.py"


def _load_module(name: str, path: Path) -> ModuleType:
    """Load ``name`` from ``path`` and register it in :mod:`sys.modules`."""
    loader = SourceFileLoader(name, str(path))
    spec = importlib.util.spec_from_loader(name, loader)
    if spec is None:
        raise ImportError(f"Unable to create spec for {name} from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    loader.exec_module(module)  # type: ignore[arg-type]
    return module


def _load_ams_modules(module_names: Iterable[str]) -> Dict[str, ModuleType]:
    """Load and register all AMS override modules."""
    loaded: Dict[str, ModuleType] = {}
    for module_name in module_names:
        module_path = _module_filename(module_name)
        if not module_path.exists():
            raise ImportError(f"Missing AMS override for {module_name} at {module_path}")
        loaded[module_name] = _load_module(module_name, module_path)
    return loaded


# Load the AMS overrides on import so subsequent imports see the patched modules.
_AMS_MODULES = _load_ams_modules(_MODULE_ORDER)

# Re-export the AFC ``load_config`` function so AFC_AMS can be used as a drop-in
# replacement when referenced from printer.cfg.
from extras.AFC import load_config as _ams_load_config  # noqa: E402  (import after overrides)


def load_config(config):
    """Proxy ``load_config`` that returns the AMS-aware AFC implementation."""
    return _ams_load_config(config)


__all__ = ['load_config']
