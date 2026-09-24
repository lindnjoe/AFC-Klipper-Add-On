#!/usr/bin/env python3
"""
Draw the board as an SVG, from the .kicad_pcb, with no KiCad involved.

This exists because `kicad-cli pcb export svg` needs a KiCad install and the
rest of this directory deliberately does not. It is also a check in its own
right: it renders what is IN THE FILE, so a track that did not get written, a
via with the wrong net, or a footprint at the wrong rotation shows up as a
picture that is wrong rather than as a number that is right.

What it does NOT draw is the ground pour, and that omission is the honest one.
A zone in a KiCad file is a REQUEST for copper, not copper -- the filled
polygons only exist after `finish_board.py` runs the filler. Drawing the zone
outline as if it were a plane is exactly the false reassurance that let the
carrier ship a B.Cu gerber with no ground in it. So the outline is drawn as a
dashed boundary, labelled as a request, and `route.py`'s pour_islands is what
says whether the copper it becomes is one piece.

    python3 plot_board.py [--out FILE] [--layer both|front|back]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import check_drc
import design

HERE = Path(__file__).resolve().parent

# a palette that reads in both themes: front warm, back cool, copper-ish
STYLE = {
    "F.Cu":      ("#d4462a", 0.85),
    "B.Cu":      ("#2f7fd4", 0.85),
    "pad_smd":   ("#e8a33d", 0.95),
    "pad_tht":   ("#b8860b", 0.95),
    "via":       ("#8a8f98", 1.0),
    "edge":      ("#2f3339", 1.0),
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=HERE / "out" / "board.svg")
    ap.add_argument("--layer", default="both",
                    choices=["both", "front", "back"])
    args = ap.parse_args()

    pcb = HERE / f"{design.NAME}.kicad_pcb"
    rects, segs = check_drc.load_board(pcb)
    W, H = design.BOARD_W, design.BOARD_H
    M = 6.0                                   # margin for labels
    S = 10.0                                  # px per mm

    def X(v: float) -> float:
        return (v + M) * S

    def Y(v: float) -> float:
        return (v + M) * S

    out: list[str] = []
    out.append(
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %.0f %.0f" '
        'width="%.0f" height="%.0f">'
        % ((W + 2 * M) * S, (H + 2 * M) * S, (W + 2 * M) * S, (H + 2 * M) * S))
    out.append('<rect width="100%" height="100%" fill="#f6f4ef"/>')
    out.append('<g stroke-linecap="round" fill="none">')

    want = {"both": {"F.Cu", "B.Cu"}, "front": {"F.Cu"}, "back": {"B.Cu"}}[
        args.layer]

    # the pour, as the REQUEST it is -- see the module docstring
    out.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" '
               'fill="none" stroke="#9aa0a6" stroke-width="1.5" '
               'stroke-dasharray="8 6"/>'
               % (X(0), Y(0), W * S, H * S))

    # back copper first, so front sits on top
    for layer in ("B.Cu", "F.Cu"):
        if layer not in want:
            continue
        col, op = STYLE[layer]
        for s in segs:
            if s.kind != "track" or layer not in s.layers:
                continue
            out.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" '
                       'stroke="%s" stroke-width="%.2f" opacity="%.2f"/>'
                       % (X(s.x1), Y(s.y1), X(s.x2), Y(s.y2),
                          col, s.r * 2 * S, op))

    # ══ PADS IN THE SHAPE THEY ARE. ══
    # Everything used to be drawn as a rectangle, which made 21 round pads and
    # a 3 mm round mounting peg look square. A drawing that lies about shape is
    # worse than no drawing: it is the thing people check the board against.
    for r in rects:
        if not (r.layers & want):
            continue
        tht = len(r.layers) > 1
        col, op = STYLE["pad_tht" if tht else "pad_smd"]
        out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" '
                   'fill="%s" opacity="%.2f" stroke="none"/>'
                   % (X(r.x0), Y(r.y0), (r.x1 - r.x0) * S, (r.y1 - r.y0) * S,
                      col, op))
    for s in segs:
        if s.kind != "pad" or not (s.layers & want):
            continue
        tht = len(s.layers) > 1
        col, op = STYLE["pad_tht" if tht else "pad_smd"]
        # a capsule: round caps give a circle when the ends coincide
        out.append('<line x1="%.2f" y1="%.2f" x2="%.2f" y2="%.2f" stroke="%s" '
                   'stroke-width="%.2f" stroke-linecap="round" opacity="%.2f"/>'
                   % (X(s.x1), Y(s.y1), X(s.x2), Y(s.y2), col, s.r * 2 * S, op))

    # vias: copper ring with the drill knocked out
    col, op = STYLE["via"]
    for s in segs:
        if s.kind != "via":
            continue
        out.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="%s" '
                   'opacity="%.2f"/>' % (X(s.x1), Y(s.y1), s.r * S, col, op))
        out.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="#f6f4ef"/>'
                   % (X(s.x1), Y(s.y1), s.r * S * 0.45))

    # non-plated holes: not copper at all, so drawn as what they are -- a hole
    for s in segs:
        if s.kind != "hole":
            continue
        out.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="#f6f4ef" '
                   'stroke="%s" stroke-width="1" stroke-dasharray="3 2"/>'
                   % (X(s.x1), Y(s.y1), s.r * S, STYLE["via"][0]))

    # board edge
    out.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" '
               'fill="none" stroke="%s" stroke-width="2"/>'
               % (X(0), Y(0), W * S, H * S, STYLE["edge"][0]))
    out.append("</g>")

    # reference designators
    out.append('<g font-family="ui-monospace,Menlo,monospace" font-size="11" '
               'fill="#2f3339" text-anchor="middle">')
    for ref, (x, y, rot) in sorted(design.PLACE.items()):
        out.append('<text x="%.1f" y="%.1f">%s</text>'
                   % (X(x), Y(y) - 1, ref))
    out.append("</g>")
    out.append("</svg>")

    args.out.parent.mkdir(exist_ok=True)
    args.out.write_text("\n".join(out))
    # count by KIND, not by whether a thing happens to be zero-length -- round
    # pads are zero-length capsules and were being reported as vias.
    nf = sum(1 for s in segs if s.kind == "track" and "F.Cu" in s.layers)
    nb = sum(1 for s in segs if s.kind == "track" and "B.Cu" in s.layers)
    nv = sum(1 for s in segs if s.kind == "via")
    nh = sum(1 for s in segs if s.kind == "hole")
    npad = len(rects) + sum(1 for s in segs if s.kind == "pad")
    print(f"wrote {args.out.relative_to(HERE)} — {W:.0f}x{H:.0f} mm, "
          f"{nf} F.Cu + {nb} B.Cu tracks, {nv} vias, {npad} pads, "
          f"{nh} non-plated holes")
    print("the ground pour is NOT drawn: a zone is a request for copper until "
          "finish_board.py fills it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
