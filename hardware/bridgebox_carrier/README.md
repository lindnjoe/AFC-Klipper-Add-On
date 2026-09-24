# BridgeBox carrier — KiCad project

The board described in
[`../../bambu_ams_bridge/docs/CARRIER_BOARD.md`](../../bambu_ams_bridge/docs/CARRIER_BOARD.md).
Read that first: it carries the reasoning, the current budget and the two
measured pinouts. This directory is the implementation.

Built against **KiCad 7**.

```
bridgebox_carrier.kicad_pro   project, netclasses and design rules
bridgebox_carrier.kicad_sch   schematic — generated, see below
bridgebox_carrier.kicad_pcb   board outline and stackup only
bridgebox.kicad_sym            symbols for the two modules
bridgebox.pretty/              footprints for the two modules
gen_project.py                writes the schematic and the symbol library
gen_pcb.py                    places and routes the board
route.py                      the tracks, as explicit polylines
geometry.py                   pad positions, shared by router and checker
verify_netlist.py             checks KiCad's netlist against the design
verify_wiring.py              checks the finished board against the proven pinout
check_placement.py            checks the placement for collisions
check_drc.py                  fast clearance and connectivity pre-check
finish_board.py               fills the ground plane, runs KiCad's real DRC
gen_bom.py                    the two BOMs
gen_cpl.py                    rewrites the placement file into CPL format
build.sh                      does all of it, in the one correct order
case/                         a two-piece printed case for the finished board
```

The case is its own project with its own checker and its own build; see
[`case/README.md`](case/README.md).

## Opening it

Open `bridgebox_carrier.kicad_pro`. Both the schematic and the board are
complete: placed, routed and checked. 367 mm of track, no vias and **no layer
changes at all** — B.Cu is an unbroken ground plane.

The board is 63 × 54 mm, two layers, 1 oz, laid out to the arrangement asked
for:

- **Left edge** — the Pico's USB and J1, the 24 V jack. The Pico is turned 90°
  so it runs inland from that edge with the USB overhanging it.
- **Bottom edge** — J2, the AMS 4-pin, a quarter turn round from them.
- Between them: the RS-485 module stands upright UNDER the Pico, logic pins up
  towards it and bus pins down towards J2, with the fuse in the band between.

**It used to be 84 × 58 mm.** Three things shrank it: the two A/B swap jumpers
came out once the bus wiring was settled on hardware, the module's pad columns
turned out to be 15 mm apart rather than the 0.7 in first assumed, and turning
the module upright moved it out from the Pico's right-hand side, which is what
the extra 21 mm of width had been for. Its bus pins now face J2 instead of
away from it, so A and B no longer lap the board — and the layer hop that used
to cost a slot in the ground plane went with them.

The M3 holes are **not** one per corner: the Pico's body runs the length of the
top edge and J2 owns the bottom middle, so most corners are already taken.

Mounting holes are placed in the board rather than drawn in the schematic,
which is normal for mechanical features — so if you ever re-import from the
schematic, leave "delete extra footprints" unticked or they will vanish.

Everything except the two modules comes from KiCad's stock libraries, including
`Molex_Micro-Fit_3.0_43045-0400_2x02_P3.00mm_Horizontal` for J2, so there is no
land pattern to draw by hand.

## Why the schematic is generated

The connectivity is the part that must not drift from the design document, so
it lives as a table — `NETS` in `gen_project.py` — rather than as geometry.
Regenerate with:

```sh
python3 gen_project.py
kicad-cli sch export netlist --output out/netlist.net bridgebox_carrier.kicad_sch
python3 verify_netlist.py
```

`verify_netlist.py` compares what KiCad *actually extracted* against that
table, pin for pin, and fails on a net that is split, merged or holding a pin
it should not. That matters because the schematic connects by label: a
mistyped label does not break the build, it silently makes two nets where
there should be one. The check is what lets this project be trusted without
opening the GUI.

Editing the schematic in KiCad is fine — but then it is the source, and
re-running the generator will overwrite it.

`verify_wiring.py` is narrower and runs LAST, after `finish_board.py` has
filled the zone and rewritten the file: it checks that the handful of pin
assignments established by measurement rather than by reading a silkscreen —
`GP0 → RXD`, `GP1 → TXD`, the A/B crossover, and J2's 24 V and GND — are the
ones on the pads the gerbers are plotted from. Every one of them is a place
where the obvious answer is wrong and the symptom of being wrong is a bus that
goes silent rather than one that misbehaves visibly.

`check_placement.py` does the same job for the board, comparing every
footprint's courtyard against every other one and against the mounting holes.
A render will not show you these: a mounting hole sitting inside the Pico's
outline just looks like a hole among a lot of pads. It found exactly that, plus
a 0.3 mm overlap between the Pico and the RS-485 module and a TVS sitting on
J2's body — none of which were visible by eye.

## What is still on you

Two things the project deliberately does not decide:

1. **A second pair of eyes on J2's pinout.** It is taken from Molex
   SD-43045-001 and the metered mating face, and it is the one mistake that
   puts 24 V on the transceiver.
2. **The remaining DRC warnings**, which are all cosmetic and all understood:

   | Count | Warning | Why it is left |
   |---|---|---|
   | 7 | `lib_footprint_issues` | this project's `fp-lib-table` lists only `bridgebox`; the stock libraries come from the global table on a normal install |
   | 3 | `silk_edge_clearance` | J2's body outline runs past the bottom edge, which is what a connector mating through that edge looks like; fabs clip silkscreen automatically |
   | 2 | `lib_footprint_mismatch` | the board's copy of the two custom footprints omits the library-only `version`/`generator` keys |

   **Zero errors:** no clearance violations, no unconnected pads, no footprint
   errors.

## Parts with no stock symbol

Both were measured, not taken from a datasheet:

- `RaspberryPi_Pico_THT` — 2×20, 0.1 in pitch, rows 0.7 in apart, pin 1 at the
  USB end.
- `MAX3485_Module_3L5R` — a 2×5 field with two positions empty. **Top view is
  three pins on the left (GND, A, B) and five on the right (EN, VCC, RXD, TXD,
  GND).** The underside reads mirrored; the footprint is the top view.
  0.1 in pitch down each column, columns **15 mm** apart — not the Pico's
  0.7 in. Measured on the part after a first revision built to 17.78 mm came
  back 2.8 mm too wide.
