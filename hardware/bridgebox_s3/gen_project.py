#!/usr/bin/env python3
"""
Generate the BridgeBox S3 KiCad project from design.py.

Same shape as the carrier's generator: the connectivity is authored as data so
it cannot drift from docs/S3_BOARD.md, and the symbols are lifted out of the
pinned KiCad libraries and embedded, which is what KiCad does itself on save --
it makes the project open standalone even where those libraries are absent or
a different version.

Two things this does that the carrier's does not, both forced by parts it does
not have:

  * pins are addressed by NAME (see design.py). The symbol is the authority on
    which number that is.
  * derived symbols are FLATTENED. MAX3485 extends MAX481E and TPS54202DDC
    extends TPS54302: the child carries properties and nothing else, so
    embedding it raw hands KiCad a reference it cannot resolve and a part with
    no pins. The parent's graphics and pins are merged into the child here.

Run from this directory:  python3 gen_project.py [--libs DIR]
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
NAME = design.NAME
SHEET_UUID = "b41d6e00-5333-4a11-9f21-0c1a7e5b9d01"


def esc(s: str) -> str:
    return s.replace('"', r"\"")


# ── symbols ──────────────────────────────────────────────────────────────────
def raw_symbol(libs: Libs, libid: str) -> str:
    """The symbol block as it stands in the library, un-flattened."""
    lib, name = libid.split(":", 1)
    text = (libs.sym / f"{lib}.kicad_sym").read_text()
    m = re.search(r'^  \(symbol "%s"[\s(]' % re.escape(name), text, re.M)
    if not m:
        raise SystemExit(f"symbol not found: {libid}")
    return sexp(text, m.start())


def flatten(libs: Libs, libid: str) -> str:
    """A self-contained symbol, embedded under its full "Lib:Name" id.

    A derived symbol is `(symbol "Child" (extends "Parent") <properties>)` --
    no rectangle, no pins. Embedding that alone produces a schematic KiCad
    opens with an empty part; embedding it *with* the parent works but leaves
    the resolution to KiCad's loader. Merging is unambiguous: take the
    parent's body, give it the child's id, and let the child's properties win.

    Only the TOP-LEVEL symbol takes the "Lib:Name" id. The child unit blocks
    inside keep their bare "Name_0_1" form -- prefixing those too makes the
    whole file fail to load, which is how the carrier found it.
    """
    lib, name = libid.split(":", 1)
    blk = raw_symbol(libs, libid)
    ext = re.search(r'\(extends "([^"]+)"', blk)
    if not ext:
        return blk.replace('(symbol "%s"' % name, '(symbol "%s"' % libid, 1)

    parent_name = ext.group(1)
    parent = raw_symbol(libs, f"{lib}:{parent_name}")

    # the child's properties override the parent's of the same name
    child_props = {
        m.group(1): sexp(blk, m.start())
        for m in re.finditer(r'\(property "([^"]+)"', blk)
    }
    out = parent
    for pname, ptext in child_props.items():
        pm = re.search(r'\(property "%s"' % re.escape(pname), out)
        if pm:
            out = out.replace(sexp(out, pm.start()), ptext, 1)
        else:                       # a property the parent does not carry
            ins = out.index("\n", out.index('(symbol "')) + 1
            out = out[:ins] + "    " + ptext + "\n" + out[ins:]

    # rename the top level, and the unit blocks that are keyed on the parent
    out = out.replace('(symbol "%s"' % parent_name, '(symbol "%s"' % libid, 1)
    out = out.replace('(symbol "%s_' % parent_name, '(symbol "%s_' % name)
    return out


def pin_geometry(symbol_text: str) -> dict[str, tuple[float, float, int]]:
    """number -> (x, y, rotation) in symbol coordinates (+y up)."""
    out: dict[str, tuple[float, float, int]] = {}
    for m in re.finditer(r"\(pin\s+\S+\s+\S+\s+\(at ([-\d.]+) ([-\d.]+) "
                         r"(\d+)\)", symbol_text):
        x, y, rot = float(m.group(1)), float(m.group(2)), int(m.group(3))
        num = re.search(r'\(number "([^"]+)"', sexp(symbol_text, m.start()))
        if num:
            out[num.group(1)] = (x, y, rot)
    return out


def stub(px: float, py: float, rot: int, sx: float, sy: float
         ) -> tuple[float, float, float, float, str]:
    """Where a wire must start and end to attach to one pin.

    A pin's ``(at)`` IS its connection point -- the ``length`` extends from
    there towards the body, not away from it. Offsetting by the length is the
    obvious reading and it puts the wire inside the symbol, leaving every net
    unconnected. Symbol space has +y up, schematic space +y down, hence the
    flip; the free end runs opposite the pin's angle.
    """
    ax, ay = sx + px, sy - py
    r = math.radians(rot)
    bx = ax - 2.54 * math.cos(r)
    by = ay + 2.54 * math.sin(r)
    return ax, ay, round(bx, 2), round(by, 2), (
        "right" if bx < ax - 0.01 else "left")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    libs = Libs(args.libs)

    # pick the symbol each part actually resolved to, as verify_design does
    chosen: dict[str, str] = {}
    for ref, part in design.PARTS.items():
        for cand in part["lib"]:
            try:
                libs.pins(cand)
                chosen[ref] = cand
                break
            except LookupError:
                continue
        if ref not in chosen:
            raise SystemExit(f"{ref}: no symbol among {part['lib']}")

    # Indented by two, as the carrier's lifter does, so the embedded symbols
    # sit one level inside (lib_symbols ...) like KiCad's own output. Cosmetic
    # to the parser, but it is what a reader diffing against a KiCad-saved file
    # will expect -- and what verify_sch.py keys its extraction on.
    embedded = {
        libid: "\n".join("  " + ln for ln in flatten(libs, libid).splitlines())
        for libid in sorted(set(chosen.values()))
    }
    geom = {ref: pin_geometry(embedded[libid]) for ref, libid in chosen.items()}
    for ref, g in geom.items():
        if not g:
            raise SystemExit(f"{ref}: flattened symbol has no pins")

    uid = [0]

    def nid() -> str:
        uid[0] += 1
        return "00000000-0000-0000-0000-%012d" % uid[0]

    sch = ['(kicad_sch (version 20230121) (generator bridgebox_s3)',
           "  (uuid %s)" % SHEET_UUID,
           '  (paper "A3")',
           "  (title_block",
           '    (title "BridgeBox S3 — ESP32-S3 + MAX3485 → Bambu AMS buffer")',
           '    (company "Sovoron")',
           '    (comment 1 "24 V rated 3 A: buffer + AMS logic + feed motors. '
           'Dryers are on their own supply.")',
           '    (comment 2 "BUS_A/BUS_B are CROSSED at the transceiver -- '
           'U2.B is BUS_A. Metered, not assumed.")',
           '    (comment 3 "Symbols from kicad-symbols 7.0.11; pins addressed '
           'by NAME in design.py.")',
           "  )",
           "  (lib_symbols"]
    sch += [embedded[k] for k in sorted(embedded)]
    sch.append("  )")

    for ref in sorted(design.PARTS):
        part = design.PARTS[ref]
        x, y = part["at"]
        sch += [
            '  (symbol (lib_id "%s") (at %.2f %.2f 0) (unit 1)'
            % (chosen[ref], x, y),
            "    (in_bom yes) (on_board yes) (dnp no)",
            "    (uuid %s)" % nid(),
            '    (property "Reference" "%s" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) (justify left)))" % (ref, x + 12, y - 12),
            '    (property "Value" "%s" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) (justify left)))" % (esc(part["value"]),
                                                    x + 12, y - 9),
            '    (property "Footprint" "%s" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) hide))" % (esc(part["fp"]), x, y),
            '    (property "Datasheet" "" (at %.2f %.2f 0) (effects (font '
            "(size 1.27 1.27)) hide))" % (x, y),
        ]
        for num in sorted(geom[ref], key=lambda n: (len(n), n)):
            sch.append('    (pin "%s" (uuid %s))' % (num, nid()))
        # KiCad 7 keeps the reference on the INSTANCE. Without this the file
        # loads unannotated at best; the path must match the sheet uuid.
        sch += ["    (instances",
                '      (project "%s"' % NAME,
                '        (path "/%s" (reference "%s") (unit 1))'
                % (SHEET_UUID, ref),
                "      )", "    )", "  )"]

    # Connectivity is by LABEL, so nothing is routed across the sheet and no
    # two nets can be joined by a stray crossing. One stub per pin -- and a
    # name may resolve to several pins, which is how a USB-C receptacle gets
    # all four of its GNDs and both of each data pin onto the right net.
    stubs = 0
    for net in sorted(design.NETS):
        for ref, pin in design.NETS[net]:
            for num in resolve(libs.pins(chosen[ref]), pin):
                if num not in geom[ref]:
                    raise SystemExit(f"{ref}: pin {num} absent from symbol")
                px, py, rot = geom[ref][num]
                sx, sy = design.PARTS[ref]["at"]
                ax, ay, bx, by, just = stub(px, py, rot, sx, sy)
                sch += [
                    "  (wire (pts (xy %.2f %.2f) (xy %.2f %.2f))"
                    % (ax, ay, bx, by),
                    "    (stroke (width 0) (type default)) (uuid %s))" % nid(),
                    '  (label "%s" (at %.2f %.2f 0) (fields_autoplaced)'
                    % (net, bx, by),
                    "    (effects (font (size 1.27 1.27)) (justify %s bottom))"
                    % just,
                    "    (uuid %s))" % nid(),
                ]
                stubs += 1

    sch += ["  (sheet_instances", '    (path "/" (page "1"))', "  )", ")"]
    (HERE / f"{NAME}.kicad_sch").write_text("\n".join(sch) + "\n")
    print(f"wrote {NAME}.kicad_sch — {len(design.PARTS)} parts, "
          f"{len(embedded)} symbols embedded, {stubs} pin stubs")
    return 0


if __name__ == "__main__":
    sys.exit(main())
