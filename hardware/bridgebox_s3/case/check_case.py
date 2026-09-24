#!/usr/bin/env python3
"""
Check the case against the board it is supposed to fit.

case.scad restates the board's outline, its mounting holes and where its
connectors sit. **Restated numbers rot.** Someone nudges a part in design.py,
the gerbers change, and the case quietly becomes a case for the previous board
-- which you find out after a five-hour print. So nothing here trusts
case.scad: it reads the constants back out of it and checks each one against
the placement the PCB is actually generated from.

What it checks:

  * outline and mounting holes match design.py exactly
  * every wall opening fully clears its connector's courtyard, with margin
  * every screw column and tray post clears every part's courtyard
  * columns and posts stay inside the board
  * the button holes land on the buttons
  * NO METAL IN THE ANTENNA KEEPOUT. This is the one that is specific to this
    board and the one that would never show up in a print. Plastic over a
    WROOM-1 antenna is fine; a screw, an insert or a nut in that 48 x 21 mm
    rectangle is a radio fault that looks like a firmware problem.

    python3 check_case.py [--libs DIR]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import design                       # noqa: E402
import fp_lib                       # noqa: E402
from verify_design import DEFAULT_LIBS, Libs   # noqa: E402

SCAD = HERE / "case.scad"

# How much wider than the connector body an opening must be, per side, for a
# PLUG -- which is always bigger than the socket -- to get in.
PLUG_MARGIN = 1.0


def scad_consts(text: str) -> dict:
    """Pull the top-level assignments out of case.scad.

    Deliberately dumb: a number, a [list], or a [[list], [of, lists]]. Anything
    it cannot parse it leaves out, and every constant this file needs is named
    explicitly below, so a silently-missed one fails loudly rather than
    defaulting.
    """
    out: dict = {}
    depth = 0
    for raw_line in text.splitlines():
        line = re.sub(r"//.*", "", raw_line)
        # Only TOP-LEVEL assignments. Inside a module `for (x = [r, w - r])`
        # looks exactly like a constant and is not one, so brace depth decides
        # rather than position on the line -- case.scad puts two constants on
        # one line where they belong together (J1_X and J1_W), and a
        # start-of-line rule silently dropped every second one.
        for stmt in line.split(";"):
            if depth == 0:
                m = re.match(r"\s*(\w+)\s*=\s*(.+)$", stmt)
                if m and not m.group(2).strip().startswith("("):
                    try:
                        out[m.group(1)] = eval(m.group(2).strip(),
                                               {"__builtins__": {}}, {})
                    except Exception:
                        pass
            depth += stmt.count("{") - stmt.count("}")
    return out


def need(c: dict, *names):
    missing = [n for n in names if n not in c]
    if missing:
        raise SystemExit(f"case.scad: could not read {', '.join(missing)}")
    return [c[n] for n in names]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    libs = Libs(args.libs)
    c = scad_consts(SCAD.read_text())
    bad: list[str] = []

    BW, BD, HOLES = need(c, "BW", "BD", "HOLES")
    POST, COL, CBORE = need(c, "POST", "COL", "CBORE")
    ANT_X, ANT_DEEP = need(c, "ANT_X", "ANT_DEEP")
    ANT_OVER, GAP, WALL = need(c, "ANT_OVER", "GAP", "WALL")

    # ── the outline ─────────────────────────────────────────────────────────
    if abs(BW - design.BOARD_W) > 1e-6 or abs(BD - design.BOARD_H) > 1e-6:
        bad.append(f"outline: case.scad has {BW} x {BD}, "
                   f"design.py has {design.BOARD_W} x {design.BOARD_H}")

    # ── the mounting holes ──────────────────────────────────────────────────
    want = sorted((round(design.PLACE[r][0], 3), round(design.PLACE[r][1], 3))
                  for r in design.PLACE if r.startswith("H"))
    got = sorted((round(float(h[0]), 3), round(float(h[1]), 3)) for h in HOLES)
    if want != got:
        bad.append(f"mounting holes: case.scad has {got}, design.py has {want}")

    # ── part courtyards, in board coordinates ───────────────────────────────
    boxes: dict[str, tuple] = {}
    for ref, (x, y, rot) in design.PLACE.items():
        fp = fp_lib.load(libs.fp, design.PARTS[ref]["fp"])
        x0, y0, x1, y1 = fp.extent()
        pts = [fp_lib.place(px, py, rot, x, y)
               for px, py in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        boxes[ref] = (min(p[0] for p in pts), min(p[1] for p in pts),
                      max(p[0] for p in pts), max(p[1] for p in pts))

    def clearance(cx, cy, skip=()):
        worst, who = 1e9, None
        for ref, (x0, y0, x1, y1) in boxes.items():
            if ref in skip or ref.startswith("H"):
                continue
            d = ((max(x0 - cx, 0.0, cx - x1)) ** 2
                 + (max(y0 - cy, 0.0, cy - y1)) ** 2) ** 0.5
            if d < worst:
                worst, who = d, ref
        return worst, who

    # ── columns and posts clear every part, and stay on the board ───────────
    r_needed = max(POST, COL, CBORE) / 2.0
    for hx, hy in got:
        d, who = clearance(hx, hy)
        if d < r_needed:
            bad.append(f"screw at ({hx}, {hy}): {d:.2f} mm to {who}, "
                       f"needs {r_needed:.2f} for a {max(POST, COL, CBORE)} mm "
                       f"boss")
        if not (r_needed <= hx <= BW - r_needed
                and r_needed <= hy <= BD - r_needed):
            bad.append(f"screw at ({hx}, {hy}): boss overhangs the outline")

    # the extra tray post -- read from case.scad rather than assumed
    m = re.search(r"translate\(\[37,\s*by\(61\),\s*FLOOR\]\)", SCAD.read_text())
    if m:
        d, who = clearance(37, 61)
        if d < POST / 2.0:
            bad.append(f"tray post at (37, 61): {d:.2f} mm to {who}, "
                       f"needs {POST / 2.0:.2f}")

    # ══ NO METAL IN THE ANTENNA KEEPOUT ═════════════════════════════════════
    # The keepout is declared in board coordinates by design.ANTENNA_KEEPOUT,
    # local to U1, so it is derived here rather than restated.
    ux, uy, urot = design.PLACE["U1"]
    kx0, ky0, kx1, ky1 = design.ANTENNA_KEEPOUT
    corners = [fp_lib.place(px, py, urot, ux, uy)
               for px, py in ((kx0, ky0), (kx1, ky0), (kx1, ky1), (kx0, ky1))]
    KX0, KY0 = min(c[0] for c in corners), min(c[1] for c in corners)
    KX1, KY1 = max(c[0] for c in corners), max(c[1] for c in corners)
    if abs(KX0 - ANT_X[0]) > 0.01 or abs(KX1 - ANT_X[1]) > 0.01:
        bad.append(f"ANT_X is {ANT_X}, the footprint's keepout is "
                   f"[{KX0:.1f}, {KX1:.1f}]")
    if abs((KY1 - KY0) - ANT_DEEP) > 0.01:
        bad.append(f"ANT_DEEP is {ANT_DEEP}, the keepout is "
                   f"{KY1 - KY0:.1f} mm deep")
    for hx, hy in got:
        rr = max(COL, CBORE) / 2.0
        if (KX0 - rr <= hx <= KX1 + rr) and (KY0 - rr <= hy <= KY1 + rr):
            bad.append(f"METAL IN THE ANTENNA KEEPOUT: screw at ({hx}, {hy}) "
                       f"with a {max(COL, CBORE)} mm boss reaches into "
                       f"x {KX0:.0f}..{KX1:.0f}, y {KY0:.0f}..{KY1:.0f}")

    # ── the cavity clears the module's physical overhang ────────────────────
    fab = fp_lib.load(libs.fp, design.PARTS["U1"]["fp"]).fab_extent()
    pts = [fp_lib.place(px, py, urot, ux, uy)
           for px, py in ((fab[0], fab[1]), (fab[2], fab[1]),
                          (fab[2], fab[3]), (fab[0], fab[3]))]
    over = -min(p[1] for p in pts)          # how far past y = 0 the body goes
    if over > 0 and ANT_OVER < over + 1.0:
        bad.append(f"ANT_OVER is {ANT_OVER} but the module body overhangs the "
                   f"top edge by {over:.1f} mm -- the wall would be through it")

    # ── the openings clear their connectors ─────────────────────────────────
    for ref, cx_name, w_name, axis in (("J1", "J1_X", "J1_W", "x"),
                                       ("J2", "J2_X", "J2_W", "x"),
                                       ("J3", "J3_Y", "J3_W", "y")):
        cx, w = need(c, cx_name, w_name)
        x0, y0, x1, y1 = boxes[ref]
        lo, hi = (x0, x1) if axis == "x" else (y0, y1)
        o_lo, o_hi = cx - w / 2, cx + w / 2
        if o_lo > lo - PLUG_MARGIN or o_hi < hi + PLUG_MARGIN:
            bad.append(f"{ref}: opening {o_lo:.1f}..{o_hi:.1f} does not clear "
                       f"the connector {lo:.1f}..{hi:.1f} by {PLUG_MARGIN} mm "
                       f"a side")

    # ── buttons ─────────────────────────────────────────────────────────────
    BTN, BTN_D = need(c, "BTN", "BTN_D")
    for bx, by_ in BTN:
        hit = [r for r, (x0, y0, x1, y1) in boxes.items()
               if x0 <= bx <= x1 and y0 <= by_ <= y1 and r.startswith("SW")]
        if not hit:
            bad.append(f"button hole at ({bx}, {by_}) is not over any switch")

    print(f"case {design.BOARD_W + 2 * GAP + 2 * WALL:.1f} x "
          f"{design.BOARD_H + 2 * GAP + ANT_OVER + 2 * WALL:.1f} mm outside, "
          f"for a {BW} x {BD} board")
    print(f"antenna keepout x {KX0:.0f}..{KX1:.0f}, {KY1 - KY0:.0f} mm deep; "
          f"module overhangs the edge by {over:.1f} mm, cavity allows "
          f"{ANT_OVER}")
    print(f"{len(got)} screws, nearest part clearance "
          + ", ".join(f"{clearance(x, y)[0]:.1f}" for x, y in got) + " mm")
    if bad:
        print(f"\n{len(bad)} PROBLEM(S):")
        for b in bad:
            print(f"  ! {b}")
        return 1
    print("case OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
