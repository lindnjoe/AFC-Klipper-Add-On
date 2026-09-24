# BridgeBox S3 — printed case

Two parts, M3 screws, 79.7 × 78.6 mm outside for a 74.1 × 65 mm board.

```
case.scad       the model: tray, lid, openings, vents
check_case.py   reads the constants BACK OUT and checks them against design.py
build.sh        check, then STLs and pictures
```

```
./build.sh                       # needs openscad; xvfb-run for the pictures
openscad -D 'part="tray"' …      # or drive it directly
```

## It splits at the board plane

Inherited from the carrier's case, for the reason that was right there. This
project has no 3D models, so every component height is a datasheet number
nobody here can check against the part in your hand.

Put the split at the **top face of the PCB** and the problem goes away. The tray
holds the board and ends there; the lid covers everything above; and every
connector opening becomes a notch in the lid wall whose **bottom edge is the
board surface** — which is exactly where every connector's own bottom edge is,
because they all sit on the board. A guessed height can then only make a notch
too tall, never misaligned.

The barrel jack is the one exception and is called out as such in `case.scad`:
it is a cylinder whose centre is *below* the board plane, so its opening is a
hole through both parts with a real height, not a notch.

## The antenna decides the shape

The WROOM-1 is placed with its antenna end off the top edge, and that has two
consequences the carrier never had.

**The module physically overhangs by 6.0 mm.** Its body runs to board y = −6.0,
outside the outline. A wall at the board edge would not be tight, it would be
*through the module*. So the cavity runs `ANT_OVER` = 8 mm past the top edge and
the board's top edge is not a wall at all.

**The keepout is 48 × 21 mm past that edge, and what it forbids is copper and
metal** — that is what the footprint's keepout declares and what Espressif's
integration guide is about. Plastic is allowed, so this case closes over the
antenna rather than leaving a mouth in the enclosure, under two rules:

* **no fastener, insert or nut inside the keepout.** The top screws sit at board
  x = 4 and x = 70; the keepout spans x = 16…64. `check_case.py` asserts that
  rather than leaving it to luck, because a screw column drifting into that span
  is a WiFi fault that no print inspection would ever show.
* **the plastic over it is thin and unribbed** — `ANT_WALL` = 1.2 mm, not
  `WALL` = 2.4. That applies to the ceiling *and* to the end wall, which is the
  one actually beside the radiator: the module's tip is at case y = 71.0 and the
  cavity ends at 73.4, so full-thickness wall would put 2.4 mm of plastic 2.4 mm
  from the antenna. The ceiling obeyed that rule first and the end wall did not,
  which is the sort of half-applied rule that reads as deliberate a year later.

## The board had no mounting holes

It has four M3 now, and where they are was decided by what the parts left rather
than by symmetry: **the bottom corners are taken**, by the barrel jack on the
left and the Micro-Fit on the right, so the lower pair sits up the sides at
y = 50 and y = 46.

That leaves the bottom edge — the one both cables pull on — with no screw. The
tray carries a fifth post there with nothing through it. A tray supports the
whole board on its floor and standoffs anyway; the screws only stop it lifting.

## check_case.py is the part that matters

`case.scad` restates the board's outline, its holes and where its connectors
sit. **Restated numbers rot.** Someone nudges a part in `design.py`, the gerbers
change, and the case quietly becomes a case for the previous board — which you
find out after a five-hour print.

So nothing here trusts `case.scad`. The checker parses the constants back out of
it and compares each against the placement the PCB is actually generated from:
outline, hole positions, every opening against its connector's courtyard, every
screw column and tray post against every part, the button holes against the
buttons, and the antenna rules above.

It found two faults on its first run, on a case that rendered perfectly and
looked entirely finished:

```
! J1: opening 3.2..16.8 does not clear the connector 5.0..16.5 by 1.0 mm a side
! J3: opening 34.3..45.8 does not clear the connector 34.7..45.4 by 1.0 mm a side
```

Both would have printed, assembled, and refused the plug.

## Not checked, and worth knowing

* **Component heights are still datasheet numbers.** `IH` = 14 mm of clear
  height is generous for a board whose tallest top-side part is a 3.2 mm module,
  but nothing here can verify that — the split-at-the-board-plane trick makes
  heights harmless for *openings*, not for the ceiling.
* **No thermal model.** The vents are over the buck and the transceiver because
  that is where the heat is, not because anything computed a rise.
* **Print orientation and supports** are the slicer's problem. The tray prints
  floor-down with no support; the lid prints top-down, and the connector notches
  are then overhangs on the two walls that carry them.
