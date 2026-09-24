#!/usr/bin/env python3
"""
Write the BridgeBox S3 board: footprints placed and netted, outline, ground zone.

Placement only -- no tracks. That is a deliberate split, not an unfinished job:
`check_placement.py` proves the arrangement before any copper is committed to
it, and `route.py` (next) works from a board that is already known good. The
carrier's title block spent months claiming the opposite of its own contents
because those two states were never separated.

Footprints are lifted whole from the pinned library and rewritten in place:
renamed to their "Lib:Name" id, given their board position, and each electrical
pad given the net it carries. Everything else -- pad geometry, silkscreen,
courtyard, the WROOM-1's antenna keepout zone -- comes across untouched, which
is the point of using the real library rather than drawing land patterns.

Run:  python3 gen_pcb.py [--libs DIR]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import design
import fp_lib
from verify_design import DEFAULT_LIBS, Libs, resolve

HERE = Path(__file__).resolve().parent
NAME = design.NAME

# 0.2 mm for signals, 0.8 mm for 3V3, 2.0 mm for the 24 V pass-through --
# the last one carried forward from the carrier, where 3 A is the budget.
NETCLASS_WIDTH = {"+24V": 2.0, "+24V_IN": 2.0, "+3V3": 0.8, "GND": 0.8}
DEFAULT_WIDTH = 0.25


def net_table() -> dict[str, int]:
    """Net name -> number. 0 is the no-net net and must stay empty."""
    return {name: i + 1 for i, name in enumerate(sorted(design.NETS))}


def pad_nets(libs: Libs) -> dict[tuple[str, str], str]:
    """(ref, pad number) -> net name, resolved through the symbol's pin names."""
    out: dict[tuple[str, str], str] = {}
    for net, conns in design.NETS.items():
        for ref, pin in conns:
            for cand in design.PARTS[ref]["lib"]:
                try:
                    pins = libs.pins(cand)
                    break
                except LookupError:
                    continue
            for num in resolve(pins, pin):
                out[(ref, num)] = net
    return out


def place_footprint(fp: fp_lib.Footprint, ref: str, value: str,
                    x: float, y: float, rot: float,
                    nets: dict[str, int],
                    assign: dict[tuple[str, str], str],
                    tstamp: str) -> str:
    """One library footprint, rewritten as a placed instance on this board."""
    text = fp.text
    lib, name = fp.fpid.split(":", 1)

    # ══ THE LIBRARY MIXES TWO FILE GENERATIONS. ══  Newer footprints quote the
    # name and the layer -- `(footprint "ESP32-S3-WROOM-1" ... (layer "F.Cu")`
    # -- and older ones, regenerated less recently, quote neither:
    # `(footprint D_SOD-123 (version ...) (generator KicadMod) (layer F.Cu)`.
    # A pattern for one finds nothing in the other, which is the same trap the
    # pad numbers set one level down. Accept both, everywhere.
    text = re.sub(r'\(footprint\s+(?:"%s"|%s)' % (re.escape(name),
                                                  re.escape(name)),
                  '(footprint "%s"' % fp.fpid, text, count=1)
    text = re.sub(r"\(version \d+\)\s*", "", text, count=1)
    text = re.sub(r'\(generator [^)]*\)\s*', "", text, count=1)
    text = re.sub(r'\(tedit [^)]*\)\s*', "", text, count=1)

    # position: insert (at x y rot) right after the layer
    at = "(at %.4f %.4f%s)" % (x, y, "" if rot == 0 else " %g" % rot)
    m = re.search(r'\(layer\s+"?F\.Cu"?\)', text)
    if not m:
        raise SystemExit(f"{ref}: footprint has no F.Cu layer line")
    text = (text[:m.end()] + "\n    (tstamp %s)\n    %s" % (tstamp, at)
            + text[m.end():])

    # reference and value text
    text = re.sub(r'\(fp_text reference (?:"[^"]*"|\S+)',
                  '(fp_text reference "%s"' % ref, text, count=1)
    text = re.sub(r'\(fp_text value (?:"[^"]*"|\S+)',
                  '(fp_text value "%s"' % value.replace('"', ""), text, count=1)

    # nets onto pads. A pad's net goes after its (layers ...) clause, which is
    # where KiCad puts it and the only place it is unambiguous.
    def add_net(pm: re.Match) -> str:
        blk = pm.group(0)
        num = re.match(r'\(pad\s+(?:"([^"]*)"|(\S+))', blk)
        pad_no = (num.group(1) if num.group(1) is not None
                  else num.group(2)).strip('"')
        net = assign.get((ref, pad_no))
        if net is None:
            return blk
        return blk + ' (net %d "%s")' % (nets[net], net)

    text = re.sub(r'\(pad [\s\S]*?\(layers[^)]*\)', add_net, text)
    return "\n".join("  " + ln for ln in text.strip().splitlines())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    libs = Libs(args.libs)

    nets = net_table()
    assign = pad_nets(libs)
    fps = {ref: fp_lib.load(libs.fp, p["fp"])
           for ref, p in design.PARTS.items()}

    W, H = design.BOARD_W, design.BOARD_H
    out = ['(kicad_pcb (version 20221018) (generator bridgebox_s3)',
           "",
           "  (general",
           "    (thickness 1.6)",
           "  )",
           "",
           '  (paper "A4")',
           "  (title_block",
           '    (title "BridgeBox S3 carrier")',
           '    (comment 1 "Antenna overhangs the TOP edge: the WROOM-1 keepout '
           'is 48 x 21 mm past it and must stay in free air.")',
           '    (comment 2 "Both layers are signal + GND pour, stitched. '
           'BUS_A/BUS_B stay on F.Cu and cross at the transceiver -- U2.B is '
           'BUS_A.")',
           '    (comment 3 "Placement only -- tracks are route.py. This comment '
           'is true; check it before believing it.")',
           "  )",
           "",
           "  (layers",
           '    (0 "F.Cu" signal)',
           '    (31 "B.Cu" signal)',
           '    (32 "B.Adhes" user "B.Adhesive")',
           '    (33 "F.Adhes" user "F.Adhesive")',
           '    (34 "B.Paste" user)',
           '    (35 "F.Paste" user)',
           '    (36 "B.SilkS" user "B.Silkscreen")',
           '    (37 "F.SilkS" user "F.Silkscreen")',
           '    (38 "B.Mask" user)',
           '    (39 "F.Mask" user)',
           '    (40 "Dwgs.User" user "User.Drawings")',
           '    (41 "Cmts.User" user "User.Comments")',
           '    (42 "Eco1.User" user "User.Eco1")',
           '    (43 "Eco2.User" user "User.Eco2")',
           '    (44 "Edge.Cuts" user)',
           '    (45 "Margin" user)',
           '    (46 "B.CrtYd" user "B.Courtyard")',
           '    (47 "F.CrtYd" user "F.Courtyard")',
           '    (48 "B.Fab" user)',
           '    (49 "F.Fab" user)',
           "  )",
           "",
           "  (setup",
           "    (pad_to_mask_clearance 0)",
           "  )",
           ""]

    out.append('  (net 0 "")')
    for name, num in sorted(nets.items(), key=lambda kv: kv[1]):
        out.append('  (net %d "%s")' % (num, name))
    out.append("")

    uid = [0x100]

    def nid() -> str:
        uid[0] += 1
        return "00000000-0000-0000-0000-%012x" % uid[0]

    for ref in sorted(design.PLACE):
        x, y, rot = design.PLACE[ref]
        out.append(place_footprint(fps[ref], ref, design.PARTS[ref]["value"],
                                   x, y, rot, nets, assign, nid()))
        out.append("")

    # board outline
    for x1, y1, x2, y2 in ((0, 0, W, 0), (W, 0, W, H), (W, H, 0, H),
                           (0, H, 0, 0)):
        out.append('  (gr_line (start %g %g) (end %g %g) (stroke (width 0.1) '
                   '(type default)) (layer "Edge.Cuts") (tstamp %s))'
                   % (x1, y1, x2, y2, nid()))
    out.append("")

    # ══ GROUND POUR ON BOTH SIDES. ══
    #
    # This board started with the carrier's arrangement -- every track on F.Cu,
    # B.Cu an unbroken plane, zero vias -- and route.py disproved it rather than
    # anyone arguing about it. Four of the ten connections it could not make are
    # the USB-C receptacle's own flip pairs (A6-B6, A7-B7, the VBUS trio), which
    # are geometric: tying them on one layer means threading between pads on a
    # 0.5 mm pitch, where a 0.25 mm track with clearance needs 0.65 mm and has
    # 0.2 mm. No placement fixes that. Every USB-C design has a second routing
    # layer for this reason.
    #
    # So B.Cu becomes signal-and-pour rather than plane, and F.Cu gains a pour
    # of its own so the return path is continuous on whichever side a track is
    # not using. The two are stitched by route.py.
    #
    # What this costs is the carrier's "nothing crosses B.Cu" guarantee, which
    # was a property of that board and not a law -- it was free there because
    # every part on it was a through-hole module. The price is paid with care
    # on the bus pair: BUS_A/BUS_B are pinned to F.Cu (route.py's NO_BACK) so
    # the pour beneath them is never cut by their own routing.
    for layer in ("F.Cu", "B.Cu"):
        out += ['  (zone (net %d) (net_name "GND") (layer "%s") (tstamp %s)'
                % (nets["GND"], layer, nid()),
                "    (hatch edge 0.5)",
                "    (connect_pads (clearance 0.4))",
                "    (min_thickness 0.25) (filled_areas_thickness no)",
                "    (fill yes (thermal_gap 0.4) (thermal_bridge_width 0.6))",
                "    (polygon (pts (xy 0 0) (xy %g 0) (xy %g %g) (xy 0 %g)))"
                % (W, W, H, H),
                "  )"]
    out.append(")")

    (HERE / f"{NAME}.kicad_pcb").write_text("\n".join(out) + "\n")
    netted = len({k for k in assign})
    print(f"wrote {NAME}.kicad_pcb — {len(design.PLACE)} footprints, "
          f"{len(nets)} nets, {netted} pads netted, {W:.0f}x{H:.0f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
