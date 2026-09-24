#!/usr/bin/env python3
"""
Check the design against the real KiCad libraries, before anything is emitted.

This runs without KiCad installed -- it reads the library files directly -- so
the netlist can be proven self-consistent on any machine. What it catches:

  * a symbol or footprint id that does not exist in the library version pinned
    below (three of the ids in the first draft of design.py were wrong, and a
    wrong id is the cheap failure -- it stops here);
  * a pin referenced by a name or number the part does not have (a WRONG PIN
    NUMBER is the expensive failure: it generates cleanly and is silently the
    wrong net, which is why design.py addresses pins by name wherever the part
    has meaningful ones);
  * an ESP32-S3 signal on a pin that is SPI flash, PSRAM, USB or strapping;
  * a net with fewer than two pins, which is a typo every time.

Usage:  python3 verify_design.py [--libs DIR]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import design

def _default_libs() -> Path:
    """Where to find kicad-symbols/kicad-footprints checkouts.

    In order: $KICAD_LIBS, a ./libs beside this file, then the system install
    (which is laid out differently -- symbols and footprints sit directly in
    /usr/share/kicad, not in ksym/ and kfp/ subdirectories, so Libs handles
    both shapes). Nothing here is committed to the repo: the libraries are
    ~315 MB and pinned by TAG below, which is the reproducible part.
    """
    import os
    env = os.environ.get("KICAD_LIBS")
    if env:
        return Path(env)
    here = Path(__file__).resolve().parent / "libs"
    if here.exists():
        return here
    return Path("/usr/share/kicad")


DEFAULT_LIBS = _default_libs()
SYM_VER = "7.0.11"      # kicad-symbols tag these ids were checked against
FP_VER = "7.0.11"       # kicad-footprints tag


def sexp(text: str, start: int) -> str:
    """The balanced S-expression beginning at `start`."""
    depth, i = 0, start
    while i < len(text):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1
    return text[start:]


class Libs:
    def __init__(self, root: Path) -> None:
        # Two layouts: a pair of git checkouts (ksym/ + kfp/), or a system
        # KiCad install (symbols/ + footprints/). Try both so the same script
        # runs on a machine with KiCad and on one without.
        if (root / "ksym").exists():
            self.sym, self.fp = root / "ksym", root / "kfp"
        elif (root / "symbols").exists():
            self.sym, self.fp = root / "symbols", root / "footprints"
        else:
            self.sym, self.fp = root, root
        self._cache: dict[str, list[tuple[str, str]]] = {}

    def pins(self, libid: str, depth: int = 0) -> list[tuple[str, str]]:
        """[(number, name)] for a symbol, following `extends` to the parent.

        Derived symbols carry no pins of their own -- MAX3485 extends MAX481E,
        TPS54202DDC extends TPS54302 -- so a reader that stops at the top level
        sees a part with no pins and validates nothing.
        """
        if libid in self._cache:
            return self._cache[libid]
        lib, name = libid.split(":", 1)
        path = self.sym / f"{lib}.kicad_sym"
        if not path.exists():
            raise LookupError(f"no symbol library {lib}.kicad_sym")
        text = path.read_text()
        m = re.search(r'^  \(symbol "%s"[\s(]' % re.escape(name), text, re.M)
        if not m:
            raise LookupError(f"no symbol {libid}")
        blk = sexp(text, m.start())
        out: list[tuple[str, str]] = []
        for pm in re.finditer(r"\(pin\s+\S+\s+\S+\s+\(at", blk):
            pb = sexp(blk, pm.start())
            nm = re.search(r'\(name "([^"]*)"', pb)
            nu = re.search(r'\(number "([^"]*)"', pb)
            if nm and nu:
                out.append((nu.group(1), nm.group(1)))
        if not out and depth < 4:
            ext = re.search(r'\(extends "([^"]+)"', blk)
            if ext:
                out = self.pins(f"{lib}:{ext.group(1)}", depth + 1)
        self._cache[libid] = out
        return out

    def has_footprint(self, fpid: str) -> bool:
        lib, name = fpid.split(":", 1)
        return (self.fp / f"{lib}.pretty" / f"{name}.kicad_mod").exists()


def resolve(pins: list[tuple[str, str]], ref: str) -> list[str]:
    """Pin numbers matching `ref`, which may be a pin NAME or a pin NUMBER.

    A name may match several pins and that is correct, not an error: a USB-C
    receptacle has four GND, four VBUS and two of each data pin, and every one
    of them has to land on the net or the connector is not flippable.
    """
    by_name = [n for n, nm in pins if nm == ref]
    if by_name:
        return by_name
    return [n for n, _nm in pins if n == ref]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--libs", type=Path, default=DEFAULT_LIBS)
    args = ap.parse_args()
    libs = Libs(args.libs)
    bad: list[str] = []
    note: list[str] = []

    # ── parts exist, in the pinned library version ──────────────────────────
    chosen: dict[str, str] = {}
    for ref, part in sorted(design.PARTS.items()):
        got = None
        for cand in part["lib"]:
            try:
                libs.pins(cand)
                got = cand
                break
            except LookupError:
                continue
        if got is None:
            bad.append(f"{ref}: no symbol found, tried {part['lib']}")
            continue
        chosen[ref] = got
        if got != part["lib"][0]:
            note.append(f"{ref}: fell back to {got}")
        if not libs.has_footprint(part["fp"]):
            bad.append(f"{ref}: no footprint {part['fp']}")

    # ── every pin reference resolves ────────────────────────────────────────
    used: dict[str, set[str]] = {}
    for net, conns in sorted(design.NETS.items()):
        if len(conns) < 2:
            bad.append(f"net {net}: only {len(conns)} connection")
        for ref, pin in conns:
            if ref not in chosen:
                bad.append(f"net {net}: unknown part {ref}")
                continue
            nums = resolve(libs.pins(chosen[ref]), pin)
            if not nums:
                bad.append(f"net {net}: {ref} has no pin '{pin}'")
                continue
            used.setdefault(ref, set()).update(nums)

    # ══ EVERY ELECTRICAL PAD ON EVERY FOOTPRINT HAS A NET. ══
    #
    # The checks above ask "does every pin design.py names exist?". They cannot
    # ask the question that matters more: is there a pad on the board that
    # design.py never mentions? That is the shape of a wrong SYMBOL, and a
    # wrong symbol resolves perfectly.
    #
    # D2 is the case that prompted this. It is an SM712 -- an RS-485 clamp in
    # SOT-23, three pins, the third of which is the GROUND the surge is
    # supposed to go to. It was drawn with `Device:D_TVS`, which has exactly
    # two pins. Both resolved, every net checked out, the board routed and
    # passed DRC, and the third pad sat there with no net on it. A clamp with
    # no path to ground is a component that cannot do its job, and nothing in
    # this directory would ever have said so.
    #
    # Mechanical pads are exempt: mounting holes, the np_thru_hole pegs under a
    # connector, and shield tabs the design deliberately leaves floating.
    try:
        import fp_lib
        for ref, part in sorted(design.PARTS.items()):
            if ref not in chosen or not libs.has_footprint(part["fp"]):
                continue
            fp = fp_lib.load(libs.fp, part["fp"])
            have = set(used.get(ref, set()))
            why, declared = design.NO_CONNECT.get(ref, ("", set()))
            orphan = sorted({p.number for p in fp.pads
                             if p.electrical and p.number not in have
                             and p.number not in declared})
            if orphan:
                bad.append(f"{ref} ({part['fp'].split(':')[-1]}): pad(s) "
                           f"{', '.join(orphan)} carry no net and are not in "
                           f"NO_CONNECT -- connect them or say why")
            # A declaration that no longer applies is its own kind of stale
            # comment: it silences a pad that is now connected, and the next
            # person reads it as still true.
            stale = sorted(declared & have)
            if stale:
                note.append(f"{ref}: NO_CONNECT lists {', '.join(stale)}, "
                            f"which are now connected -- drop them")
    except ImportError:
        note.append("fp_lib unavailable: pad coverage not checked")

    # ── the ESP32-S3 pins that must never carry a signal ────────────────────
    esp = [c for c, p in design.PARTS.items()
           if "ESP32-S3" in "".join(p["lib"])]
    for ref in esp:
        names = {nm for n, nm in libs.pins(chosen[ref])}
        for net, conns in design.NETS.items():
            for r, pin in conns:
                if r != ref or pin not in names:
                    continue
                if pin in design.FORBIDDEN_IO:
                    bad.append(f"net {net}: {ref}.{pin} is flash/PSRAM")
                elif pin in design.STRAPPING_IO and net not in ("BOOT", "EN"):
                    bad.append(f"net {net}: {ref}.{pin} is a strapping pin")
                elif pin in design.USB_IO and not net.startswith("USB_"):
                    bad.append(f"net {net}: {ref}.{pin} is native USB")

    # ── report ──────────────────────────────────────────────────────────────
    print(f"symbols {SYM_VER}, footprints {FP_VER}")
    print(f"{len(design.PARTS)} parts, {len(design.NETS)} nets, "
          f"{sum(len(v) for v in used.values())} pins connected")
    for n in note:
        print(f"  note: {n}")
    if bad:
        print(f"\n{len(bad)} PROBLEM(S):")
        for b in bad:
            print(f"  ! {b}")
        return 1
    print("design OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
