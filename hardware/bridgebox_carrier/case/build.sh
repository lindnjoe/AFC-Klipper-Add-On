#!/bin/sh
# Check the case against the board, then render the two STLs and the pictures.
#
# The order matters: check_case.py reads the PCB placement, so if a part has
# moved the STLs never get written and you find out here rather than after a
# five-hour print.
#
#     ./build.sh
set -e
cd "$(dirname "$0")"
mkdir -p out

python3 check_case.py
# The preview's stand-in blocks come from the real placement, not from
# literals in preview.scad -- see gen_preview_parts.py.
python3 gen_preview_parts.py

# Binary STL. Ascii is still OpenSCAD 2021's default and triples the size of
# a file that lives in git; every slicer reads binary.
for p in tray lid; do
    openscad --export-format=binstl -o "out/$p.stl" -D "part=\"$p\"" case.scad
done

# The first board -- the 84 x 58 one that was built and works -- now gets the
# SAME case revision as the current board, tray included: a deeper lid with the
# USB window at the measured socket height, the BOOTSEL slot at the Pico 2's SW1,
# AND the connector-seating fix (STANDOFF raised 1 mm, the J2 opening dropped
# below the split). Those last two change the TRAY as well as the lid -- it is
# 1 mm taller and its front wall is notched for J2 -- so BOTH parts are rendered
# here and both must be reprinted. The old lid-only, frozen-tray arrangement is
# gone; the board is not re-fabricated, only its printed case is replaced.
#
# The board is fabricated, so its outline, holes and the three opening positions
# are frozen; they were checked against the placement at 2c021adc9^ and every
# opening clears its connector. Only VENT_BANKS and those openings differ from
# the current board; USB_Y and BOOT_Y are stated as BD - 13 in case.scad and
# follow BD on their own. STANDOFF, J2_DROP and BOOT_X now INHERIT case.scad's
# defaults (6.0, 1.0, 25.0) -- the same values the current board uses -- so only
# the outline/opening overrides remain. The Pico vent bank starts at x0 34 to
# leave a ~5 mm wall past the relocated BOOTSEL slot.
for p in lid tray; do
    openscad --export-format=binstl -o "out/$p-84x58.stl" \
        -D "part=\"$p\"" \
        -D 'BW=84.0' -D 'BD=58.0' \
        -D 'HOLES=[[5,30],[5,53],[80,8],[80,53]]' \
        -D 'J1_Y=16.0' -D 'J2_X=64.5' \
        -D 'VENT_BANKS=[[34,50,[5,9,13,17]]]' \
        -D 'LOGO_W=65.0' -D 'LOGO_AT=[42.2,15.2]' \
        case.scad
done

# Pictures. Headless, so a virtual X server and software GL -- neither changes
# the geometry, they only let OpenSCAD open a framebuffer to draw into.
if command -v xvfb-run >/dev/null 2>&1; then
    R="xvfb-run -a openscad --colorscheme=Tomorrow"
    export LIBGL_ALWAYS_SOFTWARE=1
    # preview.scad renders WITHOUT --render: CGAL fuses everything into one
    # solid of one colour, which is exactly the wrong picture when the point is
    # to see the board sitting between two parts.
    $R --imgsize=1400,1000 -o out/exploded.png \
        --camera=42,29,16,62,0,220,300 preview.scad
    $R --imgsize=1400,1000 -o out/assembled.png -D LIFT=0 \
        --camera=42,29,14,74,0,195,270 preview.scad
    R="$R --render=cgal"
    $R --imgsize=1400,1000 -o out/lid-underside.png -D 'part="lid"' \
        --camera=42,29,20,128,0,205,290 case.scad
    $R --imgsize=1200,860 --projection=o -o out/lid-top.png -D 'part="lid"' \
        --camera=42,29,20,0,0,0,240 case.scad
    $R --imgsize=1100,560 --projection=o -o out/lid-left.png -D 'part="lid"' \
        --camera=42,29,17,90,0,270,170 case.scad
    $R --imgsize=1300,520 --projection=o -o out/lid-front.png -D 'part="lid"' \
        --camera=42,29,17,90,0,0,240 case.scad
else
    echo "no xvfb-run: STLs written, pictures skipped"
fi

ls -l out
