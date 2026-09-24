#!/usr/bin/env python3
"""
Place the BridgeBox carrier board.

Mechanical constraint, given: the USB and the 24 V jack share one edge, and the
AMS 4-pin leaves an edge 90 degrees round from them. That fixes almost
everything else -- the Pico has to run inland from the USB edge, which sets the
board's long axis, and J2 then wants the bottom.

Footprints are read from the installed libraries and re-emitted with a position
and a net on every pad, so this file is the placement and nothing else.
Routing is left to KiCad; the ratsnest is correct the moment it opens.

    python3 gen_pcb.py
    kicad-cli pcb export svg --output out/pcb.svg --page-size-mode 2 \\
        --layers Edge.Cuts,F.Cu,B.Cu,F.SilkS bridgebox_carrier.kicad_pcb
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_project as g  # noqa: E402

HERE = Path(__file__).resolve().parent
FPDIR = Path("/usr/share/kicad/footprints")

# Board outline, in sheet coordinates.
X0, Y0 = 100.0, 80.0
W, H = 63.0, 54.0

# ── placement ────────────────────────────────────────────────────────────────
# ref -> (x, y, rotation) relative to the board's top-left corner.
#
# LEFT edge  (x = 0): the Pico's USB and J1. The Pico is turned 90 so its USB
#                     end faces out and the board runs inland from it.
# BOTTOM edge (y = H): J2, a quarter turn round from them as asked. Its
#                     footprint opens towards -y, so it is turned 180.
#
# THE MODULE STANDS UP NOW, and that is what shrank the board. It used to sit
# to the RIGHT of the Pico, which pushed the outline out to 84 mm for a part
# 20 mm wide, and its bus pins then faced away from J2 so A and B had to run
# out to x 81 and back. Turned 90 it tucks UNDER the Pico instead, logic pins
# up towards it and bus pins down towards J2, so both journeys are short and
# the whole right-hand third of the old board stops existing.
PLACE = {
    # A = 90 maps the footprint's local +y onto board +x, which is how the
    # Pico ends up running inland with its USB overhanging the left edge.
    "U1":  (5.0, 13.0, 90),       # Pico, USB out the LEFT edge
    "J1":  (15.0, 45.0, 270),     # barrel jack, barrel out the LEFT edge
    "J2":  (39.0, 45.0, 180),     # AMS 4-pin, out the BOTTOM edge
    "U2":  (47.0, 36.0, 90),      # RS-485 module, bus pins down towards J2
    "F1":  (19.0, 33.0, 0),
}


# M3 mounting holes. NOT simply "one per corner" -- the Pico's body reaches
# the whole top edge and J2 owns the bottom middle, so the corners are mostly
# spoken for. check_placement.py is what says so.
HOLES = (
    (6.0, 32.0),      # left, in the gap between the Pico and the jack
    (24.0, 48.5),     # bottom left, between the jack and J2
    (56.0, 50.0),     # bottom right, under the module
    (59.0, 8.0),      # top right, past the end of the Pico
)


# Where a part's silkscreen designator sits, in FOOTPRINT-local coordinates,
# for the ones whose stock position lands somewhere wrong.
#
# U2's own footprint puts it at local (0, 15), which is sensible for a part
# standing the way the footprint is drawn. Turned 90 degrees that maps to
# board x = 62 on a 63 mm board and the text runs off the edge. Moved to the
# clear strip below the module instead.
REF_POS: dict[str, tuple[float, float]] = {
    "U2": (-15.0, 0.0),          # board (47, 51)
    # J2 mates through the bottom edge, so its stock designator -- placed
    # clear of the connector body -- lands past the outline entirely, printed
    # on nothing. Moved above the connector instead.
    "J2": (-0.5, 6.5),           # board (39.5, 38.5)
}


def sexp(text: str, start: int) -> str:
    return g.sexp_block(text, start)


def load_fp(libid: str) -> str:
    lib, name = libid.split(":", 1)
    path = (HERE / f"{lib}.pretty" / f"{name}.kicad_mod" if lib == "bridgebox"
            else FPDIR / f"{lib}.pretty" / f"{name}.kicad_mod")  # noqa: E501
    return path.read_text()


def pad_nets() -> dict[tuple[str, str], str]:
    out: dict[tuple[str, str], str] = {}
    for net, pins in g.NETS.items():
        for ref, pad in pins:
            out[(ref, pad)] = net
    return out


def main() -> None:
    nets = ["", *sorted(g.NETS)]
    netno = {n: i for i, n in enumerate(nets)}
    p2n = pad_nets()
    fps = {ref: (libid, fp) for ref, libid, _v, fp, *_ in g.PARTS}

    uid = [0]

    def nid() -> str:
        uid[0] += 1
        return "00000000-0000-0000-0000-%012d" % uid[0]

    body: list[str] = []
    for ref, (libid, fp) in fps.items():
        if ref not in PLACE:
            raise SystemExit(f"{ref} has no placement")
        px, py, rot = PLACE[ref]
        text = load_fp(fp)
        inner = sexp(text, text.index("(footprint"))
        # strip the wrapper, keep the contents
        inner = inner[inner.index("\n"):inner.rindex(")")]
        # drop the library-only keys pcbnew does not want inside a board
        inner = re.sub(r"\n\s*\(version [^\)]*\)", "", inner)
        inner = re.sub(r"\n\s*\(generator [^\)]*\)", "", inner)
        inner = re.sub(r'\n\s*\(layer "F.Cu"\)', "", inner, count=1)
        # The reference designator. Older footprints in the stock library
        # write it UNQUOTED -- (fp_text reference REF** ...) -- so matching
        # only the quoted form silently leaves half the board labelled REF**.
        inner = re.sub(r'\(fp_text reference (?:"[^"]*"|\S+)',
                       '(fp_text reference "%s"' % ref, inner, count=1)
        if ref in REF_POS:
            rx, ry = REF_POS[ref]
            inner = re.sub(r'(\(fp_text reference "' + ref + r'" )\(at [^)]*\)',
                           r'\g<1>(at %.2f %.2f)' % (rx, ry),
                           inner, count=1)
        # A net on every pad we know about.
        #
        # THE UNQUOTED-TOKEN TRAP, THIRD TIME. Older stock footprints write
        # (pad 1 smd ...) with no quotes, so a pattern that only matches
        # (pad "1" ...) silently leaves those pads netless -- and a netless
        # pad is not a build error, it is a board where every SMD part is
        # electrically disconnected while the picture looks perfect.
        def net_for(m: re.Match) -> str:
            pad = m.group(1) if m.group(1) is not None else m.group(2)
            name = p2n.get((ref, pad))
            if name is None:
                return m.group(0)
            return "%s (net %d \"%s\")" % (
                m.group(0)[:-1], netno[name], name) + ")"
        inner = re.sub(r'\(pad (?:"([^"]*)"|(\S+))[^\n]*\)', net_for, inner)

        # A footprint inside a board carries (tstamp ...), not (uuid ...) --
        # the schematic uses uuid and the board does not, and mixing them is a
        # hard parse error rather than a warning.
        body.append('  (footprint "%s" (layer "F.Cu")' % fp)
        body.append("    (tstamp %s)" % nid())
        body.append("    (at %.3f %.3f %d)" % (X0 + px, Y0 + py, rot))
        body.append(inner)
        body.append("  )")

    for hn, (hx, hy) in enumerate(HOLES, 1):
        text = load_fp("MountingHole:MountingHole_3.2mm_M3")
        inner = sexp(text, text.index("(footprint"))
        inner = inner[inner.index("\n"):inner.rindex(")")]
        inner = re.sub(r"\n\s*\(version [^\)]*\)", "", inner)
        inner = re.sub(r"\n\s*\(generator [^\)]*\)", "", inner)
        inner = re.sub(r'\n\s*\(layer "F.Cu"\)', "", inner, count=1)
        # Reference on F.Fab, not silkscreen: a mounting hole does not need
        # a designator on the board, and four stray labels are four more
        # chances to collide with something that does.
        inner = re.sub(r'\(fp_text reference (?:"[^"]*"|\S+)',
                       '(fp_text reference "H%d"' % hn, inner, count=1)
        inner = re.sub(r'(\(fp_text reference "H\d+" \(at [^)]*\) )'
                       r'\(layer "F.SilkS"\)',
                       r'\1(layer "F.Fab")', inner, count=1)
        body.append('  (footprint "MountingHole:MountingHole_3.2mm_M3" '
                    '(layer "F.Cu")')
        body.append("    (tstamp %s)" % nid())
        body.append("    (at %.3f %.3f)" % (X0 + hx, Y0 + hy))
        body.append(inner)
        body.append("  )")

    outline = []
    pts = [(X0, Y0), (X0 + W, Y0), (X0 + W, Y0 + H), (X0, Y0 + H), (X0, Y0)]
    for a, b in zip(pts, pts[1:]):
        outline.append('  (gr_line (start %.2f %.2f) (end %.2f %.2f) '
                       '(stroke (width 0.1) (type solid)) (layer "Edge.Cuts") '
                       "(tstamp %s))" % (a[0], a[1], b[0], b[1], nid()))


    gnd = netno["GND"]
    # Zones take tstamp too, and the ground pour is B.Cu ALONE: the whole
    # point of this stackup is an uninterrupted plane under the A/B pair, and
    # pouring the front as well invites routing decisions that break it.
    zone = ['  (zone (net %d) (net_name "GND") (layer "B.Cu") '
            "(tstamp %s) (hatch edge 0.5)" % (gnd, nid()),
            "    (connect_pads (clearance 0.4))",
            "    (min_thickness 0.25) (filled_areas_thickness no)",
            "    (fill yes (thermal_gap 0.4) (thermal_bridge_width 0.6))",
            "    (polygon (pts"]
    for x, y in pts[:-1]:
        zone.append("      (xy %.2f %.2f)" % (x, y))
    zone += ["    ))", "  )"]

    layers = "\n".join([
        "  (layers",
        '    (0 "F.Cu" signal)', '    (31 "B.Cu" signal)',
        '    (32 "B.Adhes" user "B.Adhesive")',
        '    (33 "F.Adhes" user "F.Adhesive")',
        '    (34 "B.Paste" user)', '    (35 "F.Paste" user)',
        '    (36 "B.SilkS" user "B.Silkscreen")',
        '    (37 "F.SilkS" user "F.Silkscreen")',
        '    (38 "B.Mask" user)', '    (39 "F.Mask" user)',
        '    (40 "Dwgs.User" user "User.Drawings")',
        '    (41 "Cmts.User" user "User.Comments")',
        '    (42 "Eco1.User" user "User.Eco1")',
        '    (43 "Eco2.User" user "User.Eco2")',
        '    (44 "Edge.Cuts" user)', '    (45 "Margin" user)',
        '    (46 "B.CrtYd" user "B.Courtyard")',
        '    (47 "F.CrtYd" user "F.Courtyard")',
        '    (48 "B.Fab" user)', '    (49 "F.Fab" user)',
        "  )"])

    out = ["(kicad_pcb (version 20221018) (generator bridgebox)", "",
           "  (general (thickness 1.6))", "", '  (paper "A4")',
           "  (title_block",
           '    (title "BridgeBox carrier")',
           '    (comment 1 "USB + 24 V on one edge, the AMS 4-pin a quarter '
           'turn round. Placement only -- routing is not done.")',
           '    (comment 2 "2 oz. B.Cu is a GND plane and nothing may cross '
           'it; route on F.Cu.")',
           "  )", "", layers, "",
           "  (setup (pad_to_mask_clearance 0))", ""]
    for i, n in enumerate(nets):
        out.append('  (net %d "%s")' % (i, n))
    # Tracks, vias and the one layer hop, from route.py.
    import route as rt
    tr = []
    for (ax, ay, bx, by, w, net) in rt.segments():
        tr.append('  (segment (start %.3f %.3f) (end %.3f %.3f) (width %.2f)'
                  ' (layer "F.Cu") (net %d) (tstamp %s))'
                  % (X0 + ax, Y0 + ay, X0 + bx, Y0 + by, w,
                     netno[net], nid()))
    for (a, b, net) in rt.HOPS:
        tr.append('  (segment (start %.3f %.3f) (end %.3f %.3f) (width %.2f)'
                  ' (layer "B.Cu") (net %d) (tstamp %s))'
                  % (X0 + a[0], Y0 + a[1], X0 + b[0], Y0 + b[1], rt.W_BUS,
                     netno[net], nid()))
        for (vx, vy) in (a, b):
            tr.append('  (via (at %.3f %.3f) (size %.2f) (drill %.2f)'
                      ' (layers "F.Cu" "B.Cu") (net %d) (tstamp %s))'
                      % (X0 + vx, Y0 + vy, rt.VIA_D, rt.VIA_DRILL,
                         netno[net], nid()))
    for (vx, vy) in rt.GND_VIAS:
        tr.append('  (via (at %.3f %.3f) (size %.2f) (drill %.2f)'
                  ' (layers "F.Cu" "B.Cu") (net %d) (tstamp %s))'
                  % (X0 + vx, Y0 + vy, rt.VIA_D, rt.VIA_DRILL,
                     netno["GND"], nid()))

    out += [""] + body + [""] + outline + [""] + tr + [""] + zone + [")"]
    (HERE / "bridgebox_carrier.kicad_pcb").write_text("\n".join(out) + "\n")
    print("placed %d footprints on a %.0f x %.0f mm board" % (len(fps), W, H))


if __name__ == "__main__":
    main()
