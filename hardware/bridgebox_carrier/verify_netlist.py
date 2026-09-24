#!/usr/bin/env python3
"""
Check the exported netlist against the design.

The schematic connects by label, so a typo in a label name does not fail to
build -- it silently produces two nets where there should be one. This compares
what KiCad actually extracted against NETS in gen_project.py, pin for pin, and
is the reason the project can be trusted without opening the GUI.

    kicad-cli sch export netlist --output out/netlist.net bridgebox_carrier.kicad_sch
    python3 verify_netlist.py
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_project as g  # noqa: E402

HERE = Path(__file__).resolve().parent


def nets_from(path: Path) -> dict[str, set[tuple[str, str]]]:
    text = path.read_text()
    blk = g.sexp_block(text, text.index("(nets"))
    out: dict[str, set[tuple[str, str]]] = {}
    for m in re.finditer(r'\(net \(code "\d+"\) \(name "([^"]*)"\)', blk):
        one = g.sexp_block(blk, blk.rfind("(net ", 0, m.end()))
        out[m.group(1)] = {
            (n.group(1), n.group(2)) for n in
            re.finditer(r'\(node \(ref "([^"]+)"\) \(pin "([^"]+)"\)', one)}
    return out


def main() -> int:
    got = nets_from(HERE / "out" / "netlist.net")
    # KiCad prefixes sheet-local labels with the sheet path.
    got = {k.lstrip("/"): v for k, v in got.items()}
    want = {k: set(v) for k, v in g.NETS.items()}

    bad = 0
    for name in sorted(want):
        if got.get(name) == want[name]:
            print(f"  ok    {name:8s} {len(want[name]):2d} pins")
            continue
        bad += 1
        print(f"  FAIL  {name}")
        print(f"        want {sorted(want[name])}")
        print(f"        got  {sorted(got.get(name, set()))}")

    # Anything left that is not one of ours and not an explicitly unconnected
    # single pin means two nets were joined, or one was split.
    stray = [n for n, v in got.items()
             if n not in want and not n.startswith("unconnected-")]
    if stray:
        bad += 1
        print("\n  FAIL  unexpected nets:", stray)

    # Pins we never mention should be genuinely unconnected, not silently
    # swept into a net by a stub landing on the wrong pin.
    claimed = {p for v in want.values() for p in v}
    for n, v in got.items():
        if n.startswith("unconnected-") and (v & claimed):
            bad += 1
            print(f"\n  FAIL  {n} holds a pin that should be connected: {v}")

    print("\nRESULT:", "netlist matches the design"
          if not bad else f"{bad} problem(s)")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
