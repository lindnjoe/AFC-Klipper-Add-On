"""
Unit tests for the BT_ convenience macros in config/macros/AFC_macros.cfg.

Regression coverage for GitHub issue #817: BT_LANE_EJECT / BT_LANE_MOVE /
BT_CHANGE_TOOL silently coerced a non-numeric LANE value (e.g. LANE=lane1)
to 0 via Jinja's `int` filter, producing a confusing 'lane0 is not valid'
error instead of a clear message.

These tests render the actual `gcode:` template text straight out of the
.cfg file (via configparser, the same way Klipper's own configfile.py reads
it) through a real jinja2.Environment configured with Klipper's macro
delimiters ('{%' '%}' '{' '}'), so they exercise exactly what klippy would
run -- without needing a full klippy simulation.

Covers, for BT_CHANGE_TOOL / BT_LANE_EJECT / BT_LANE_MOVE:
  - old usage:  LANE=<int>            (backwards compatible)
  - new usage:  LANE=<full lane name> (e.g. LANE=lane2, case-insensitive)
  - failure:    LANE=<garbage>        -> clear error, no command emitted
"""

from __future__ import annotations

import configparser
import pathlib
import re
from types import SimpleNamespace

import jinja2
import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
MACROS_CFG = REPO_ROOT / "config" / "macros" / "AFC_macros.cfg"

STEPPER_NAME = "lane"


def _load_gcode_template(macro_name: str) -> str:
    """Pull the raw `gcode:` script for a [gcode_macro NAME] section."""
    cp = configparser.RawConfigParser()
    cp.read(MACROS_CFG)
    return cp.get(f"gcode_macro {macro_name}", "gcode")


def _render(macro_name: str, params: dict) -> tuple[str, list[str]]:
    """Render a BT_ macro's gcode template like Klipper would.

    Returns (rendered_output, responded_messages) where responded_messages
    mirrors what action_respond_info() would have sent to the console.
    """
    script = _load_gcode_template(macro_name)
    env = jinja2.Environment("{%", "%}", "{", "}")
    template = env.from_string(script)

    responded: list[str] = []

    def action_respond_info(msg):
        responded.append(msg)
        return ""

    context = {
        "printer": {
            "gcode_macro _AFC_GLOBAL_VARS": SimpleNamespace(
                stepper_name=STEPPER_NAME
            ),
        },
        "params": params,
        "action_respond_info": action_respond_info,
    }
    output = str(template.render(context))
    return output, responded


def _emitted_command(output: str) -> str | None:
    """Return the first non-blank rendered gcode line, if any."""
    for line in output.splitlines():
        line = line.strip()
        if line:
            return line
    return None


MACRO_CASES = [
    # (macro_name, wrapped_command)
    ("BT_CHANGE_TOOL", "CHANGE_TOOL"),
    ("BT_LANE_EJECT", "LANE_UNLOAD"),
]


@pytest.mark.parametrize("macro_name,wrapped_command", MACRO_CASES)
class TestChangeToolAndLaneEject:
    def test_old_usage_default(self, macro_name, wrapped_command):
        # No LANE given -> defaults to lane 1 (backwards compatible).
        output, responded = _render(macro_name, {})
        assert _emitted_command(output) == f"{wrapped_command} LANE=lane1"
        assert responded == []

    def test_old_usage_numeric(self, macro_name, wrapped_command):
        # LANE=2 (the pre-#817 integer form) must keep working.
        output, responded = _render(macro_name, {"LANE": "2"})
        assert _emitted_command(output) == f"{wrapped_command} LANE=lane2"
        assert responded == []

    def test_new_usage_full_lane_name(self, macro_name, wrapped_command):
        # LANE=lane2 (the form used by LANE_UNLOAD/CHANGE_TOOL directly).
        output, responded = _render(macro_name, {"LANE": "lane2"})
        assert _emitted_command(output) == f"{wrapped_command} LANE=lane2"
        assert responded == []

    def test_new_usage_full_lane_name_case_insensitive(
        self, macro_name, wrapped_command
    ):
        output, responded = _render(macro_name, {"LANE": "LANE2"})
        assert _emitted_command(output) == f"{wrapped_command} LANE=lane2"
        assert responded == []

    def test_invalid_lane_fails_clearly(self, macro_name, wrapped_command):
        # The original bug: a garbage LANE used to silently become lane0.
        output, responded = _render(macro_name, {"LANE": "garbage"})
        assert _emitted_command(output) is None
        assert wrapped_command not in output
        assert len(responded) == 1
        assert "Invalid LANE parameter" in responded[0]
        assert "garbage" in responded[0]
        # Must never silently coerce to lane0.
        assert "lane0" not in output

    def test_empty_lane_fails_clearly(self, macro_name, wrapped_command):
        output, responded = _render(macro_name, {"LANE": ""})
        assert _emitted_command(output) is None
        assert wrapped_command not in output
        assert len(responded) == 1
        assert "Invalid LANE parameter" in responded[0]


class TestLaneMove:
    def test_old_usage_default(self):
        output, responded = _render("BT_LANE_MOVE", {})
        assert _emitted_command(output) == "LANE_MOVE LANE=lane1 DISTANCE=20.0"
        assert responded == []

    def test_old_usage_numeric_lane(self):
        output, responded = _render(
            "BT_LANE_MOVE", {"LANE": "2", "DISTANCE": "100"}
        )
        assert _emitted_command(output) == "LANE_MOVE LANE=lane2 DISTANCE=100.0"
        assert responded == []

    def test_new_usage_full_lane_name(self):
        output, responded = _render(
            "BT_LANE_MOVE", {"LANE": "lane2", "DISTANCE": "100"}
        )
        assert _emitted_command(output) == "LANE_MOVE LANE=lane2 DISTANCE=100.0"
        assert responded == []

    def test_new_usage_full_lane_name_case_insensitive(self):
        output, responded = _render(
            "BT_LANE_MOVE", {"LANE": "LANE2", "DISTANCE": "100"}
        )
        assert _emitted_command(output) == "LANE_MOVE LANE=lane2 DISTANCE=100.0"
        assert responded == []

    def test_invalid_lane_fails_clearly_even_with_valid_distance(self):
        output, responded = _render(
            "BT_LANE_MOVE", {"LANE": "garbage", "DISTANCE": "100"}
        )
        assert _emitted_command(output) is None
        assert "LANE_MOVE" not in output
        assert len(responded) == 1
        assert "Invalid LANE parameter" in responded[0]
        assert "lane0" not in output

    def test_invalid_distance_still_fails_clearly(self):
        # Pre-existing DISTANCE validation must still work once LANE is valid.
        output, responded = _render(
            "BT_LANE_MOVE", {"LANE": "2", "DISTANCE": "abc"}
        )
        assert _emitted_command(output) is None
        assert "LANE_MOVE" not in output
        assert len(responded) == 1
        assert "Invalid DISTANCE parameter" in responded[0]
