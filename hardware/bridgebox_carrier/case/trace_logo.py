#!/usr/bin/env python3
"""
Trace the BridgeBox mark from the raster artwork into an SVG the lid imports.

WHY A TRACE AND NOT THE RASTER. OpenSCAD cannot engrave a bitmap; it needs
outlines.

WHAT IT KEEPS. Only the WHITE INK. The mark is white line art with a dark
outline and soft interior highlights; the ink silhouette IS the logo, and the
highlights are shading that means nothing in a recess. So it thresholds to the
ink, drops anything too small to be a real feature, and traces what is left.

    python3 trace_logo.py            # rewrites bridgebox_mark.scad
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np
import potrace
from PIL import Image, ImageFilter

HERE = Path(__file__).resolve().parent
SRC = HERE.parent.parent.parent / "docs" / "img" / "bridgebox_logo.png"
OUT = HERE / "bridgebox_mark.scad"

# The histogram is strongly bimodal -- ~74% below 32, ~30% above 224 -- so the
# threshold is not a judgement call; anything from 100 to 200 gives the same
# ink to within 2%. 128 sits in the middle of that shelf.
THRESHOLD = 128

# Work at 4x. The source is a 1143 px JPEG; tracing it directly quantises every
# curve to whole source pixels, which is most of why an earlier pass came out
# looking hand-drawn. Upscaling first gives the threshold sub-pixel edges to
# find and potrace four times the resolution to fit against.
UPSCALE = 4

# Edge-softening blur, at 4x. The source is a clean PNG now, so this is only
# taking the anti-aliasing off the threshold rather than fighting compression
# ringing -- it was 3 when the artwork was a JPEG off a link-preview card.
DERING = 1.5

# THE ONE THAT MATTERS. The artwork's white shapes are separated by hairline
# DARK lines -- the bevel that gives the logo its 3D look on screen. Traced
# literally those hairlines become both the thinnest strokes (0.29 mm at the
# 45 mm the lid panel allows) and the narrowest gaps (0.33 mm), and a 0.4 mm
# nozzle holds neither: the recess fills in and the mark reads as a smudge.
#
# The bevel is shading. It means nothing in a groove. Closing welds it shut,
# which widens strokes AND real gaps together: median stroke 0.62 -> 0.87 mm,
# 25th-percentile gap 0.53 -> 0.67 mm.
#
# Re-swept against the full-resolution PNG (1674 px wide, up from the 1143 px
# JPEG), where the same physical hairline spans more pixels and so needs a
# bigger radius. 24 is a fidelity ceiling, not a preference: by 28 the 'i'
# starts merging into its dot.
CLOSE_RADIUS = 24

# Final round-off, at 4x. Takes the last of the threshold's stair-stepping off
# the curves without moving any edge far enough to matter.
SMOOTH = 4

# The four filament bays are meant to be identical, and in the artwork they
# very nearly are -- but tracing a JPEG gives each one its own outline, and at
# 45 mm the differences read as sloppiness rather than as detail. Traced widths
# came out within a few percent of each other on an uneven pitch. Stamping ONE
# of them four times on an even pitch fixes both. Set False to keep whatever the trace
# produced.
REGULARISE_BAYS = True

# Smallest feature kept, in source pixels of area. JPEG ringing produces specks
# of a few px; real features (the smallest is the chip's two dots) are hundreds.
TURDSIZE = 40 * UPSCALE * UPSCALE

# Bezier smoothing. The default 0.2 chases compression wobble along what should
# be straight edges; 1.0 lets potrace fit longer, calmer curves, which is what
# a recess wants.
OPTTOLERANCE = 0.6

# Bezier flattening step, in TRACED (upscaled) pixels. At 45 mm across one
# traced pixel is about 0.01 mm, so 12 px chords are 0.12 mm -- finer than the
# nozzle resolves, while keeping the generated file to a sane size.
FLATTEN_PX = 12.0


def regularise_bays(rings: list) -> list:
    """
    Replace the row of filament bays with one shape repeated on an even pitch.

    Finds them by shape, not by position: a run of contours that share a y
    range and a width to within a few percent. That is deliberately tight,
    because the WORDMARK is also a row of similar-height shapes -- but its
    letters differ in width (an 'i' against a 'B') and in y extent (ascenders,
    the 'g' descender), so it does not survive the test.

    The template is whichever bay's width is nearest the mean, so the shape
    stays the artwork's own; only the differences between them are discarded.

    :param rings: traced contours, y already flipped
    :return list: the same contours with the bay row regularised
    """
    def bbox(r):
        xs = [p[0] for p in r]
        ys = [p[1] for p in r]
        return min(xs), min(ys), max(xs), max(ys)

    boxes = [bbox(r) for r in rings]
    best: list[int] = []
    for i, b in enumerate(boxes):
        bh, bw = b[3] - b[1], b[2] - b[0]
        if bh <= 0 or bw <= 0:
            continue
        group = [j for j, o in enumerate(boxes)
                 if abs((o[3] - o[1]) - bh) <= 0.03 * bh
                 and abs(o[1] - b[1]) <= 0.03 * bh
                 and abs((o[2] - o[0]) - bw) <= 0.10 * bw]
        if len(group) > len(best):
            best = group
    if len(best) < 3:
        return rings

    best.sort(key=lambda j: boxes[j][0])
    widths = [boxes[j][2] - boxes[j][0] for j in best]
    mean_w = sum(widths) / len(widths)
    tmpl = min(best, key=lambda j: abs((boxes[j][2] - boxes[j][0]) - mean_w))

    tb = boxes[tmpl]
    tcx, tcy = (tb[0] + tb[2]) / 2, (tb[1] + tb[3]) / 2
    shape = [(x - tcx, y - tcy) for x, y in rings[tmpl]]

    firstc = (boxes[best[0]][0] + boxes[best[0]][2]) / 2
    lastc = (boxes[best[-1]][0] + boxes[best[-1]][2]) / 2
    step = (lastc - firstc) / (len(best) - 1)
    cy = sum((boxes[j][1] + boxes[j][3]) / 2 for j in best) / len(best)

    out = list(rings)
    for n, j in enumerate(best):
        cx = firstc + n * step
        out[j] = [(x + cx, y + cy) for x, y in shape]
    print(f"  bays: {len(best)} regularised to w={tb[2]-tb[0]:.0f} "
          f"on a {step:.0f} pitch (were "
          f"{'/'.join(f'{w:.0f}' for w in widths)})")
    return out


def main() -> int:
    if not SRC.exists():
        sys.exit(f"artwork not found: {SRC}")
    src = Image.open(SRC).convert("L")
    w0, h0 = src.size
    big = src.resize((w0 * UPSCALE, h0 * UPSCALE), Image.LANCZOS)

    # Blur-then-threshold IS morphology, and it is ISOTROPIC -- which is the
    # point. PIL's MaxFilter/MinFilter are SQUARE, and a square kernel
    # staircases every curve and corner it touches; that, plus the JPEG
    # ringing, is what made the first trace look hand-drawn. A Gaussian is
    # round, so dilate and erode round off the way the artwork does.
    # Threshold below 128 grows the shape, above 128 shrinks it.
    def morph(img, radius, level):
        return img.filter(ImageFilter.GaussianBlur(radius)) \
                  .point(lambda v: 255 if v > level else 0)

    mask = morph(big, DERING, THRESHOLD)              # de-ring
    mask = morph(mask, CLOSE_RADIUS, 55)              # dilate ...
    mask = morph(mask, CLOSE_RADIUS, 200)             # ... erode: a true close
    mask = morph(mask, SMOOTH, THRESHOLD)             # round off
    ink = np.asarray(mask) > 127
    h, w = ink.shape

    # POLARITY. potracer treats a set element as the region to trace AROUND,
    # not the region to trace -- handed `ink` it returns the background, whose
    # outer boundary is the whole canvas, and every real shape then sits one
    # level deeper and comes out as a hole. The result is a solid rectangle
    # with the logo punched through it, which is not obviously wrong until it
    # is rendered. Invert going in, and assert it below.
    path = potrace.Bitmap(~ink).trace(turdsize=TURDSIZE,
                                      opttolerance=OPTTOLERANCE)

    # Flatten every contour to a polygon in OpenSCAD's coordinates.
    #
    # NOT an SVG. OpenSCAD 2021.01 imports SVG but ignores fill-rule, so the
    # even-odd holes came back filled and the whole mark arrived inverted
    # inside a frame. Emitting polygons resolves the holes HERE, where the
    # nesting is known, and leaves nothing for the importer to guess.
    #
    # y is flipped on the way out (image y runs down, OpenSCAD's runs up), so
    # the caller needs no mirror.
    def flatten(curve) -> list[tuple[float, float]]:
        pts: list[tuple[float, float]] = []
        cur = (curve.start_point.x, curve.start_point.y)
        pts.append(cur)
        for seg in curve.segments:
            if isinstance(seg, potrace.BezierSegment):
                p0, p3 = cur, (seg.end_point.x, seg.end_point.y)
                p1 = (seg.c1.x, seg.c1.y)
                p2 = (seg.c2.x, seg.c2.y)
                # Steps from the control polygon's length, so a long sweeping
                # curve gets more of them than a short one.
                ln = (abs(p1[0]-p0[0]) + abs(p1[1]-p0[1])
                      + abs(p2[0]-p1[0]) + abs(p2[1]-p1[1])
                      + abs(p3[0]-p2[0]) + abs(p3[1]-p2[1]))
                n = max(2, min(24, int(ln / FLATTEN_PX)))
                for i in range(1, n + 1):
                    t = i / n
                    u = 1 - t
                    pts.append((u*u*u*p0[0] + 3*u*u*t*p1[0] + 3*u*t*t*p2[0] + t*t*t*p3[0],
                                u*u*u*p0[1] + 3*u*u*t*p1[1] + 3*u*t*t*p2[1] + t*t*t*p3[1]))
                cur = p3
            else:
                pts.append((seg.c.x, seg.c.y))
                pts.append((seg.end_point.x, seg.end_point.y))
                cur = (seg.end_point.x, seg.end_point.y)
        return [(x, h - y) for x, y in pts]      # flip to y-up

    rings = [flatten(c) for c in path]

    if REGULARISE_BAYS:
        rings = regularise_bays(rings)

    # The guard for the above: if any contour spans the whole canvas, potrace
    # traced the background and everything downstream is inverted.
    for r in rings:
        xs = [x for x, _ in r]
        ys = [y for _, y in r]
        if (max(xs) - min(xs) > w * 0.99) and (max(ys) - min(ys) > h * 0.99):
            sys.exit("trace is inverted: a contour spans the whole canvas, so "
                     "potrace traced the background. Check the polarity of the "
                     "mask handed to potrace.Bitmap().")

    # Hand the fill rule to OpenSCAD. polygon(points, paths) applies EVEN-ODD
    # across its paths, which is exactly what a contour set from potrace wants:
    # a ring inside a ring is a hole, one inside that is solid again, to any
    # depth. An earlier version worked the nesting out here by ray-casting and
    # got it wrong on the deeper cases -- the spool bays sit two levels in and
    # came out as holes, leaving an empty box. OpenSCAD already knows how.
    pts: list[tuple[float, float]] = []
    paths: list[list[int]] = []
    for r in rings:
        paths.append(list(range(len(pts), len(pts) + len(r))))
        pts.extend(r)

    pt_txt = ",".join(f"[{x:.2f},{y:.2f}]" for x, y in pts)
    path_txt = ",".join("[" + ",".join(str(i) for i in p) + "]" for p in paths)

    OUT.write_text(
        "// GENERATED by trace_logo.py from docs/img/bridgebox_logo.png.\n"
        "// Do not edit: re-run the tracer instead.\n"
        f"// {len(rings)} contours, {len(pts)} points, {w} x {h} traced units,\n"
        "// y already flipped for OpenSCAD, holes by even-odd across paths.\n"
        "module bridgebox_mark_raw() {\n"
        f"    polygon(points=[{pt_txt}],\n            paths=[{path_txt}]);\n"
        "}\n")

    # logo.scad states the mark's aspect for check_case.py's keep-out. Rewrite
    # it here so a re-trace at different proportions cannot leave it stale.
    scad = HERE / "logo.scad"
    text = scad.read_text()
    new = re.sub(r"^LOGO_HF = .*$", f"LOGO_HF = {h}/{w};", text,
                 count=1, flags=re.M)
    new = re.sub(r"^LOGO_SRC_W = .*$", f"LOGO_SRC_W = {w};", new,
                 count=1, flags=re.M)
    new = re.sub(r"^LOGO_SRC_H = .*$", f"LOGO_SRC_H = {h};", new,
                 count=1, flags=re.M)
    if new != text:
        scad.write_text(new)
    print(f"{SRC.name} -> {OUT.name}: {len(rings)} contours, {len(pts)} points, "
          f"{OUT.stat().st_size} bytes, {w}x{h} (LOGO_HF = {h}/{w})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
