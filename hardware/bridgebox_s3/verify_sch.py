#!/usr/bin/env python3
"""
Read the generated schematic back and rebuild the netlist from it.

Checking that the generator ran is not the same as checking that it wrote what
was meant. This parses the .kicad_sch as a stranger would -- symbol instances,
wires, labels -- reconstructs which pin is on which net from the geometry, and
compares that against design.py. A generator bug that puts a stub on the wrong
pin, or a label at the wrong end of a wire, shows up here and nowhere else
short of opening KiCad.

Also checks the things that make a file fail to LOAD, which are cheap to get
wrong and expensive to discover: unbalanced parens, a lib_id with no embedded
symbol, and an embedded symbol with no pins (what an unflattened derived
symbol looks like).

Usage:  python3 verify_sch.py [--libs DIR]
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import design
from verify_design import DEFAULT_LIBS, Libs, resolve, sexp

HERE = Path(__file__).resolve().parent
SCH = HERE / f"{design.NAME}.kicad_sch"
TOL = 0.01


def balanced(text: str) -> bool:
    depth = 0
    in_str = False
    esc = False
    for ch in text:
        if in_str:
            if esc:
                esc = False
            elif ch == "\\":
                esc = True
            elif ch == '"':
                in_str = False
            continue
        if ch == '"':
            in_str = True
        elif ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth < 0:
                return False
    return depth == 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    if not SCH.exists():
        print(f"no {SCH.name} -- run gen_project.py first")
        return 1
    libs = Libs(args.libs)
    text = SCH.read_text()
    bad: list[str] = []

    if not balanced(text):
        bad.append("unbalanced parentheses -- KiCad will refuse the file")

    # ── embedded symbols ────────────────────────────────────────────────────
    lib_start = text.index("  (lib_symbols")
    lib_blk = sexp(text, lib_start)
    embedded: dict[str, str] = {}
    for m in re.finditer(r'^    \(symbol "([^"]+)"', lib_blk, re.M):
        embedded[m.group(1)] = sexp(lib_blk, m.start())
    for libid, blk in embedded.items():
        if "(pin " not in blk:
            bad.append(f"embedded symbol {libid} has NO PINS "
                       f"(an unflattened derived symbol looks exactly like this)")
        if "(extends " in blk:
            bad.append(f"embedded symbol {libid} still carries (extends)")

    # ── instances ───────────────────────────────────────────────────────────
    body = text[lib_start + len(lib_blk):]
    placed: dict[str, tuple[str, float, float]] = {}
    for m in re.finditer(r'^  \(symbol \(lib_id "([^"]+)"\) '
                         r"\(at ([-\d.]+) ([-\d.]+) \d+\)", body, re.M):
        blk = sexp(body, m.start())
        ref = re.search(r'\(reference "([^"]+)"', blk)
        if not ref:
            bad.append("a symbol instance carries no reference")
            continue
        placed[ref.group(1)] = (m.group(1), float(m.group(2)),
                                float(m.group(3)))
        if m.group(1) not in embedded:
            bad.append(f"{ref.group(1)}: lib_id {m.group(1)} is not embedded")

    missing = set(design.PARTS) - set(placed)
    if missing:
        bad.append(f"parts absent from the sheet: {sorted(missing)}")

    # ── where every pin physically sits on the sheet ─────────────────────────
    pin_at: dict[tuple[float, float], list[str]] = {}
    for ref, (libid, sx, sy) in placed.items():
        for pm in re.finditer(r"\(pin\s+\S+\s+\S+\s+\(at ([-\d.]+) ([-\d.]+) "
                              r"(\d+)\)", embedded[libid]):
            num = re.search(r'\(number "([^"]+)"',
                            sexp(embedded[libid], pm.start()))
            if not num:
                continue
            x = sx + float(pm.group(1))
            y = sy - float(pm.group(2))
            pin_at.setdefault((round(x, 2), round(y, 2)), []).append(
                f"{ref}.{num.group(1)}")

    # ── wires and labels -> reconstructed nets ──────────────────────────────
    wires = [(float(a), float(b), float(c), float(d)) for a, b, c, d in
             re.findall(r"\(wire \(pts \(xy ([-\d.]+) ([-\d.]+)\) "
                        r"\(xy ([-\d.]+) ([-\d.]+)\)\)", body)]
    labels = [(lbl, float(x), float(y)) for lbl, x, y in
              re.findall(r'\(label "([^"]+)" \(at ([-\d.]+) ([-\d.]+)', body)]

    def near(ax: float, ay: float, bx: float, by: float) -> bool:
        return math.hypot(ax - bx, ay - by) < TOL

    got: dict[str, set[str]] = {}
    for x1, y1, x2, y2 in wires:
        ends = [(x1, y1), (x2, y2)]
        hit = [p for e in ends for p in pin_at.get((round(e[0], 2),
                                                    round(e[1], 2)), [])]
        lbl = [n for n, lx, ly in labels
               if any(near(lx, ly, *e) for e in ends)]
        if not hit:
            bad.append(f"wire at {x1},{y1}-{x2},{y2} touches no pin")
            continue
        if not lbl:
            bad.append(f"wire on {hit} carries no label")
            continue
        for n in set(lbl):
            got.setdefault(n, set()).update(hit)

    # ── compare against the design ──────────────────────────────────────────
    want: dict[str, set[str]] = {}
    for net, conns in design.NETS.items():
        for ref, pin in conns:
            libid = design.PARTS[ref]["lib"]
            chosen = placed[ref][0] if ref in placed else libid[0]
            for num in resolve(libs.pins(chosen), pin):
                want.setdefault(net, set()).add(f"{ref}.{num}")

    for net in sorted(set(want) | set(got)):
        w, g = want.get(net, set()), got.get(net, set())
        if w != g:
            bad.append(f"net {net}: missing {sorted(w - g)} "
                       f"unexpected {sorted(g - w)}")

    print(f"{len(placed)} instances, {len(embedded)} symbols, "
          f"{len(wires)} wires, {len(labels)} labels, "
          f"{len(got)} nets reconstructed")
    if bad:
        print(f"\n{len(bad)} PROBLEM(S):")
        for b in bad[:40]:
            print(f"  ! {b}")
        return 1
    print("schematic matches design.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
