"""Bundled AFC AMS module overrides."""

from pathlib import Path

BUNDLE_ROOT = Path(__file__).resolve().parent

__all__ = [
    'BUNDLE_ROOT',
]
