/*
 * Drives the firmware's REAL cap_parse_radius_mm() over narration lines.
 *
 * The lines come in on argv, one per argument, exactly as the unit printed
 * them; this prints the millimetre result for each, one per line, and the
 * Python side (tests/test_cap_radius_parse.py) says what each must be.
 *
 * Kept this way round on purpose: a Python re-implementation of the parser
 * would test the re-implementation. The whole value is that the bytes the
 * firmware will run are the bytes under test.
 */
#include <stdio.h>
#include <string.h>

#include "capparse.h"

int main(int argc, char **argv) {
    for (int i = 1; i < argc; i++)
        printf("%u\n", (unsigned)cap_parse_radius_mm(argv[i],
                                                     (int)strlen(argv[i])));
    return 0;
}
