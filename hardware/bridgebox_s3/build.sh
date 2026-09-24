#!/usr/bin/env bash
# Build and check everything, in the one order that is correct.
#
# THE FILL MUST COME BEFORE THE EXPORT. gen_pcb.py rewrites the board from
# scratch, which leaves the ground pours unfilled -- and an unfilled zone plots
# as no copper at all. On this board GND is ENTIRELY the pour, on BOTH sides,
# and it is what joins 21 via drops and every ground pad; exporting before
# filling produces gerbers for a board with no ground, and nothing about a
# render says so. The carrier shipped that once. See finish_board.py.
#
# TWO HALVES, AND ONLY ONE OF THEM NEEDS KICAD.
#
#   Stages 1-6 are pure Python: they generate the design, prove it against the
#   pinned libraries, place it, route it, check it at full precision and draw
#   it. They run anywhere, and they are where every fault this project has
#   found was actually found.
#
#   Stages 7-9 need a KiCad install -- the zone filler is in pcbnew and the
#   exporters are in kicad-cli. There is no pure-Python substitute for the
#   filler that would be worth trusting, so the script says so and stops rather
#   than producing an unfilled-but-plausible output set.
#
#   ./build.sh            everything, or as far as the tooling allows
#   ./build.sh --checks   stages 1-6 only, never touches KiCad
set -e
cd "$(dirname "$0")"

LIBS_ARG=""
[ -n "${KICAD_LIBS:-}" ] && LIBS_ARG="--libs $KICAD_LIBS"
CHECKS_ONLY=""
[ "${1:-}" = "--checks" ] && CHECKS_ONLY=1

rm -rf out && mkdir -p out out/gerbers out/drill

echo "== 1. the design, against the pinned libraries =="
python3 verify_design.py $LIBS_ARG | tail -2

echo "== 2. schematic, and read back as a stranger =="
python3 gen_project.py $LIBS_ARG | tail -1
python3 verify_sch.py $LIBS_ARG | tail -1

echo "== 3. placement =="
python3 check_placement.py $LIBS_ARG | tail -2

echo "== 4. board, footprints netted, pours declared =="
python3 gen_pcb.py $LIBS_ARG | tail -1

echo "== 5. route =="
python3 route.py $LIBS_ARG

echo "== 6. DRC at full precision, and a picture of what is in the file =="
python3 check_drc.py $LIBS_ARG
python3 plot_board.py | head -1
python3 gen_bom.py $LIBS_ARG

if [ -n "$CHECKS_ONLY" ]; then
    echo "== --checks: stopping before the KiCad stages =="
    exit 0
fi

if ! command -v kicad-cli >/dev/null 2>&1; then
    echo
    echo "== kicad-cli NOT FOUND -- stopping here, deliberately. =="
    echo "   Everything above is done and checked. What is missing is the"
    echo "   zone FILL and the fab exports, and both live in KiCad."
    echo
    echo "   Do NOT work around this. An export without the fill produces"
    echo "   gerbers for a board with no ground pour on either side, and they"
    echo "   look completely normal. Run this on a machine with KiCad."
    exit 3
fi

echo "== 7. fill the pours and run KiCad's own DRC =="
python3 finish_board.py

echo "== 8. fab outputs =="
# ONLY the layers a fab needs. KiCad's default export also emits Courtyard,
# Fab, Margin and the User_* layers, and a house that auto-detects layers from
# a zip can read those as real copper or simply charge for the confusion.
kicad-cli pcb export gerbers --output out/gerbers \
    --layers F.Cu,B.Cu,F.Mask,B.Mask,F.SilkS,B.SilkS,F.Paste,B.Paste,Edge.Cuts \
    "bridgebox_s3.kicad_pcb" >/dev/null
kicad-cli pcb export drill --output out/drill/ "bridgebox_s3.kicad_pcb" >/dev/null
kicad-cli sch export pdf --output out/schematic.pdf \
    "bridgebox_s3.kicad_sch" >/dev/null
# The placement file. This board is 26 SMD parts against 2 through-hole, so
# unlike the carrier the smd-only file is the useful one and the all file is
# the footnote.
kicad-cli pcb export pos --output out/cpl-smd.csv --format csv --units mm \
    --smd-only "bridgebox_s3.kicad_pcb" >/dev/null
kicad-cli pcb export pos --output out/cpl-all.csv --format csv --units mm \
    "bridgebox_s3.kicad_pcb" >/dev/null
cp out/drill/*.drl out/gerbers/
(cd out && zip -qr bridgebox_s3-gerbers.zip gerbers)

echo "== 9. prove the pours reached the GERBERS, not just the board file =="
# The whole reason stage 7 exists. G36/G37 bracket a filled region in RS-274X;
# a copper layer whose pour did not make it has pads and tracks and no G36 at
# all, and is indistinguishable from a good file at a glance.
fail=0
for f in out/gerbers/*F_Cu.gtl out/gerbers/*B_Cu.gbl; do
    n=$(grep -c G36 "$f" || true)
    echo "   $(basename "$f"): $n filled region(s)"
    [ "$n" -gt 0 ] || { echo "   NO GROUND POUR IN $(basename "$f")"; fail=1; }
done
[ "$fail" = 0 ] || exit 1
echo "== done: out/bridgebox_s3-gerbers.zip =="
