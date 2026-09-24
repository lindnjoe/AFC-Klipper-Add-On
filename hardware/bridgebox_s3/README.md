# BridgeBox S3 — KiCad project

The board described in
[`../../bambu_ams_bridge/docs/S3_BOARD.md`](../../bambu_ams_bridge/docs/S3_BOARD.md).
Read that first — it carries the reasoning. This directory is the
implementation, generated the same way the carrier is.

```
design.py          the whole design as data: parts, nets, S3 pin rules
verify_design.py   checks it against the real KiCad libraries, no KiCad needed
gen_project.py     writes the schematic, symbols embedded and flattened
verify_sch.py      reads the schematic BACK and rebuilds its netlist
fp_lib.py          footprint reader: pads, courtyards, rotation
check_placement.py courtyards, board edge, and the antenna keepout
gen_pcb.py         writes the board: footprints placed and netted, outline, zone
route.py           A* grid router, two layers, exact-geometry masks
check_drc.py       reads the BOARD back: clearance, ratsnest, containment
```

## Status

**Schematic done and proven. Board placed, routed and DRC-clean.**

```
verify_design.py    28 parts, 19 nets, 91 pins — every id resolved, every pad
                    accounted for, 7.0.11
gen_project.py      28 parts, 13 symbols embedded, 91 pin stubs
verify_sch.py       91 wires, 91 labels, 19 nets reconstructed — matches design.py
check_placement.py  74.1 x 65 mm, courtyards, keepout, and mating faces
gen_pcb.py          28 footprints, 19 nets, 91 pads netted, GND pour BOTH sides
build.sh            the chain in order; stops at the KiCad boundary
route.py            48 of 48, 589 mm, 51 vias — bus pair 0 mm on B.Cu
check_drc.py        clearance OK, every net one body, all copper on board
```

Nothing outstanding in the layout. `build.sh` (gerbers, drill, BOM, CPL) is the
remaining step, and three measurements gate ordering — see **Next**.

### A connector that faces its own board

J2 sat at `rot 0` for the whole of this board's life. At `rot 0` this
footprint's mating face points at local −y, so placed at y = 52 the opening
was at board y = **43.08, aimed into the board**, with 43 mm of PCB in front of
it. It routed, it passed DRC, it plotted like a finished board, and no cable
could ever have gone into it.

Nothing here could see that, because it is not a copper question. And the
courtyard actively misleads: its +y overhang is covering the second pad row,
not mating clearance, which is what made the *x* extent look like it set the
board width. The **F.Fab body outline** is what answers it — the shell spans
y −8.92…0.99 with the pads at y 0 and 3, so the body is on the −y side and the
opening is its far end.

`design.MATING` now declares each connector's mating axis and whether it may
protrude, and `check_placement.py` maps that through the placement rotation and
insists it lands on an edge:

```
J1 faces the bottom edge at (10.00, 70.70) -- protrudes (5.70 mm)
J2 faces the bottom edge at (68.50, 65.00) -- flush (0.00 mm)
J3 faces the left edge at (0.50, 40.05) -- flush (0.50 mm)
```

Put J2 back at `rot 0` and it fails with *"mating face … is 5.6 mm inside the
outline — no cable reaches it"*, which is how the check was confirmed to be
worth having rather than merely passing.

J1 is allowed to protrude because a barrel jack is meant to: that is how it
meets a panel. J2 and J3 are flush, so a case wall can sit against them. Both
cables now leave the bottom and left faces, and the board lost the 3 mm of
width that only existed because J2 was measured on the wrong axis: 80 → 77.1 →
**74.1**.

### Route order is a design decision, not a loop variable

Nets were routed in alphabetical order, which is not an order at all: whoever
goes first gets the corridor. `+24V` is the widest net on the board at 2 mm, it
sorts first, and it took the channel down to J2 before `BUS_B` — five letters
later — ever asked. BUS_B then had nowhere to go on F.Cu and spent 18 mm on the
back, underneath its own partner's return path.

A person laying this out routes the pair first and threads power around it,
because the pair is what the board is for. Doing that:

```
                 BUS_A            BUS_B           total copper
alphabetical     73 mm, 0 back    42 mm, 18 back      627 mm
pair first       33 mm, 0 back    41 mm,  0 back      583 mm
```

Both halves of the bus are now entirely on F.Cu over unbroken pour, and the
board uses 44 mm *less* copper. `route.py` reports what the priced nets actually
did on every run, which is how the 18 mm was noticed rather than shipped.

### The pad-coverage check, and the three faults it found

`verify_design.py` asks "does every pin `design.py` names exist?". It could not
ask the question that matters more — **is there a pad on the board that
`design.py` never mentions?** That is the shape of a wrong symbol, and a wrong
symbol resolves perfectly. Adding the check found three faults on a board that
had already routed and passed DRC:

| | was | is |
|---|---|---|
| `D2` | `Device:D_TVS`, a **two**-pin symbol in a three-pad SOT-23 | `Diode:SM712_SOT23`; pin `common` to GND |
| `R2B` | fail-safe bias **pull-down connected to nothing** | `R2B.2` to GND |
| `J3` | USB-C shell floating | `SHIELD` to GND |

The SM712 is an RS-485 clamp whose third pin is where a surge is supposed to
*go*; without it the part cannot do its only job. `S3_BOARD.md` states the bias
rule outright — "biasing A up to VCC and **B down to GND**" — and the down half
was not connected, so an idle bus had no defined differential at all. None of
these is visible in a netlist, a DRC or a plot. All three are invisible until
something counts pads.

Pins that are *meant* to float are now **declared** in `design.py`'s
`NO_CONNECT`, with a reason each, rather than filtered out. An undeclared
floating pad is a question nobody asked; a declared one is a decision. The check
also reports a declaration that has gone stale.

## Two layers, and the reason is duller than first written

The first version of this README said the USB-C flip pairs were **geometrically**
unroutable on one layer — "tying A6 to B6 means passing between pads on a 0.5 mm
pitch", "no placement fixes that", "which is why every USB-C design has a second
routing layer". That was confident and it was wrong.

The footprint (`HRO TYPE-C-31-M-12`) puts all sixteen contacts in **one row**:

```
A1/B12  A4/B9   B8    A5    B7    A6    A7    B6    A8    B5   A9/B4  A12/B1
 -3.25  -2.45  -1.75 -1.25 -0.75 -0.25  0.25  0.75  1.25  1.75  2.45   3.25
```

Nothing is threaded between anything — each pad escapes straight out, and the
DP/DM crossing happens away from the connector. What actually blocked it was the
router, twice:

* **the pads sat half a grid cell off the grid.** 0.5 mm pitch from an origin
  at y = 40.0 puts every pad centre on an odd multiple of 0.25. The escape
  clears its neighbour by 0.025 mm when perfectly centred, so half a cell of
  offset loses it. `PLACE["J3"]` is now `40.05`.
* **obstacles were rasterised then dilated.** That quantises a pad EDGE to
  ±0.05 mm — twice the entire margin. And `40.15 / 0.1` is `401.4999999999999`
  in binary floating point, so one pad grew a whole cell in one direction only.
  `route.py` now keeps obstacles as geometry and tests exact distances.

The board does still want two layers: one gets 29 connections, two gets 48. But
that is congestion and fan-out — an ordinary reason — and the interesting claim
that replaced it was never true. B.Cu is signal-and-pour, F.Cu gains a pour of
its own, and the two are stitched.

Because the tightest gap on this board is a quarter of a grid cell, the router
cannot be the last word on whether its own answer is legal. `check_drc.py`
re-reads the finished `.kicad_pcb` at full floating-point precision and reports
the tightest track-to-pad gap: **0.200 mm**, exactly the rule, no margin to
spare and none borrowed.

### What check_drc.py caught that nothing else would

`F1` was a 5×20 mm cartridge in a clip holder — and a clip holder has **two pads
numbered 1 and two numbered 2**, one pair per clip. `route.py` looked pads up by
number and returned the first match, so 24 V reached one half of each terminal
and left the other a floating rectangle. It routes cleanly, it passes every
clearance rule, it looks finished on screen, and it is an open circuit through
the fuse. Only the connectivity check sees it. `Board.pads_of` now returns every
pad with a number, and each is its own target.

The fuse is a 2920 PPTC now (see `design.py`), which is what prompted looking.

## The libraries

Not committed — ~315 MB, and the reproducible part is the *tag*, not the bytes:

```
git clone --depth 1 -b 7.0.11 https://gitlab.com/kicad/libraries/kicad-symbols.git ksym
git clone --depth 1 --filter=blob:none --sparse -b 7.0.11 \
    https://gitlab.com/kicad/libraries/kicad-footprints.git kfp
cd kfp && git sparse-checkout set RF_Module.pretty Package_SO.pretty \
    Package_TO_SOT_SMD.pretty Connector_USB.pretty Capacitor_SMD.pretty \
    Resistor_SMD.pretty Diode_SMD.pretty Inductor_SMD.pretty \
    Button_Switch_SMD.pretty Connector_BarrelJack.pretty Fuse.pretty \
    Connector_Molex.pretty MountingHole.pretty
```

Then `KICAD_LIBS=/path/to/parent python3 verify_design.py`, or drop both
checkouts in `./libs/`, or point it at a system KiCad install — it handles all
three layouts.

## Why pins are addressed by NAME

The carrier's generator hardcodes pin numbers, which is safe when you can see
the library while writing it. This design was drafted without one, and the
libraries then corrected three things that would each have produced a board
that generates cleanly and is wrong:

| assumed | actual |
|---|---|
| `Interface_UART:MAX3485E` | no such symbol — it is `MAX3485` |
| `Connector:USB_C_Receptacle_USB2.0` | `USB_C_Receptacle_USB2.0_16P` |
| TPS54202DDC = BOOT/VIN/EN/GND/FB/SW | **GND/SW/VIN/FB/EN/BOOT** |

That last one is the whole argument. Addressed by number, the buck's netlist
would have been silently wrong in five places; addressed by name,
`verify_design.py` resolves it from the symbol and the mistake cannot happen.
Pin *names* are also inherited — `MAX3485` extends `MAX481E` and `TPS54202DDC`
extends `TPS54302`, so a reader that stops at the top level sees a part with no
pins at all and validates nothing.

Parts whose pins have no names (`Device:R`, `Device:C`, the barrel jack, the
fuse — all `~`) are addressed by number, and the resolver takes either.

## Next

1. ~~`gen_project.py` — symbols + schematic~~ done
2. ~~`gen_pcb.py` / `route.py` — placement and tracks~~ done
3. ~~`check_placement.py`, `check_drc.py` — the pure-Python pre-checks~~ done
4. `build.sh` — gerbers, drill, BOM, CPL

Before ordering, and each is one measurement, from `docs/S3_BOARD.md`:

* the breakout's termination and bias values, metered;
* **the 24 V draw during a feed, with a clamp meter.** This one got sharper
  when the fuse became a PPTC: the 2920 part holds 2.6 A at 30 V and derates
  to roughly 1.9 A at 60 C, against a 3 A budget that has never been measured.
  It now decides a part, not just a trace width;
* whether an S3 holds the poll cadence with WiFi associated.
