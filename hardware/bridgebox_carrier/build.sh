#!/usr/bin/env bash
# Build and check everything, in the one order that is correct.
#
# THE FILL MUST COME BEFORE THE EXPORT. gen_pcb.py rewrites the board from
# scratch, which leaves the ground zone unfilled -- and an unfilled zone plots
# as no copper at all. On this board GND is entirely the pour, so exporting
# before filling produces gerbers for a board with no ground, and nothing about
# the render says so.
set -e
cd "$(dirname "$0")"
rm -rf out && mkdir -p out out/gerbers out/drill

python3 gen_project.py
kicad-cli sch export netlist --output out/netlist.net bridgebox_carrier.kicad_sch >/dev/null
python3 verify_netlist.py | tail -1

python3 gen_pcb.py
python3 check_placement.py | tail -1
python3 check_drc.py | tail -1
python3 finish_board.py | grep -E "filled|Found"
# AFTER the fill: pcbnew rewrites the board here, and this check reads
# the file that the gerbers are actually plotted from.
python3 verify_wiring.py | tail -1

kicad-cli sch export pdf --output out/schematic.pdf bridgebox_carrier.kicad_sch >/dev/null
# ONLY the layers a fab needs. KiCad's default export also emits Courtyard,
# Fab, Margin and the User_* layers, and a house that auto-detects layers from
# a zip can read those as real copper or simply charge for the confusion.
kicad-cli pcb export gerbers --output out/gerbers \
    --layers F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,F.Paste,B.Paste,Edge.Cuts \
    bridgebox_carrier.kicad_pcb >/dev/null
kicad-cli pcb export drill --output out/drill/ bridgebox_carrier.kicad_pcb >/dev/null
python3 gen_bom.py
kicad-cli pcb export pos --output out/cpl-smd.csv --format csv --units mm \
    --smd-only bridgebox_carrier.kicad_pcb >/dev/null
# The same again WITHOUT --smd-only. On an all-through-hole board the smd-only
# file is empty, which is the honest answer for a normal SMT order and useless
# for anything else; this one carries all five parts.
kicad-cli pcb export pos --output out/cpl-all.csv --format csv --units mm \
    bridgebox_carrier.kicad_pcb >/dev/null
python3 gen_cpl.py >/dev/null
kicad-cli pcb export svg --output out/pcb.svg --page-size-mode 2 \
    --layers Edge.Cuts,F.Cu,B.Cu,F.SilkS bridgebox_carrier.kicad_pcb >/dev/null
cp out/drill/*.drl out/gerbers/
(cd out && zip -qr bridgebox_carrier-gerbers.zip gerbers)

# Prove the plane actually reached the gerbers, not just the board file.
n=$(grep -c G36 out/gerbers/*B_Cu.gbl || true)
echo "ground plane regions in the B.Cu gerber: $n"
[ "$n" -gt 0 ] || { echo "NO GROUND PLANE IN THE GERBERS"; exit 1; }
