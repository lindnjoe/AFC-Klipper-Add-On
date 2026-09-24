#!/usr/bin/env bash
# Check the case against the board, then render the STLs and the pictures.
#
# THE CHECK COMES FIRST AND IT IS NOT A FORMALITY. check_case.py reads the
# board's real placement, so if a part has moved the STLs never get written and
# you find out here rather than after a five-hour print. It caught two openings
# too narrow for their connectors the first time it ran, on a case that
# rendered perfectly.
#
#     ./build.sh
set -e
cd "$(dirname "$0")"
mkdir -p out

LIBS_ARG=""
[ -n "${KICAD_LIBS:-}" ] && LIBS_ARG="--libs $KICAD_LIBS"

python3 check_case.py $LIBS_ARG

# Binary STL. Ascii is still OpenSCAD's default and triples the size of a file
# that lives in git; every slicer reads binary.
for p in tray lid; do
    openscad --export-format=binstl -o "out/$p.stl" -D "part=\"$p\"" case.scad \
        2>/dev/null
    echo "   out/$p.stl  $(wc -c <"out/$p.stl") bytes"
done

# Pictures. Headless, so a virtual X server and software GL -- neither changes
# the geometry, they only let OpenSCAD open a framebuffer to draw into.
if command -v xvfb-run >/dev/null 2>&1; then
    export LIBGL_ALWAYS_SOFTWARE=1
    R="xvfb-run -a openscad --colorscheme=Tomorrow --render=cgal"
    $R --imgsize=1500,1050 -o out/assembled.png \
        --camera=37,33,18,62,0,215,220 case.scad 2>/dev/null
    $R --imgsize=1500,1050 -o out/lid-underside.png -D 'part="lid"' \
        --camera=37,37,8,125,0,200,200 case.scad 2>/dev/null
    $R --imgsize=1400,900 --projection=o -o out/tray-top.png -D 'part="tray"' \
        --camera=37,37,5,0,0,0,190 case.scad 2>/dev/null
    $R --imgsize=1300,560 --projection=o -o out/front.png \
        --camera=37,33,9,90,0,0,100 case.scad 2>/dev/null
else
    echo "no xvfb-run: STLs written, pictures skipped"
fi

ls -l out
