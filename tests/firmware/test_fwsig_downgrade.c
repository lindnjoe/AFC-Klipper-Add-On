/*
 * Drives the firmware's REAL fwsig_check() against manifests the REAL
 * fw_sign.py produced.
 *
 * The signed message is written twice -- once in C (fwsig.c) and once in
 * Python (fw_sign.py) -- and nothing but agreement between them makes an
 * update work. A disagreement of one byte does not look like a disagreement:
 * it looks like "bad signature" on an image that is perfectly good, from a
 * key that is perfectly right, which is an afternoon of suspecting the wrong
 * thing. So the two halves are run against each other here rather than
 * described in a comment.
 *
 * Vectors come in on argv as
 *
 *     <ver> <flags> <len> <sha512-hex> <sig-hex> <expect>
 *
 * where <expect> is "ok" or a substring the refusal must contain, and the
 * image itself is <len> bytes read from stdin. The Python side (see
 * test_fwsig_downgrade.py) signs them; this side only judges.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fwsig.h"
#include "hostapi.h"

int main(int argc, char **argv) {
    if (argc != 7) {
        fprintf(stderr, "usage: %s <ver> <flags> <len> <sha> <sig> <expect>\n",
                argv[0]);
        return 2;
    }
    unsigned long ver = strtoul(argv[1], NULL, 10);
    unsigned long flags = strtoul(argv[2], NULL, 10);
    unsigned long len = strtoul(argv[3], NULL, 10);
    const char *sha = argv[4], *sig = argv[5], *expect = argv[6];

    unsigned char *img = malloc(len ? len : 1);
    if (!img) return 2;
    if (len && fread(img, 1, len, stdin) != len) {
        fprintf(stderr, "short image on stdin\n");
        return 2;
    }

    fwsig_reset();
    if (!fwsig_set_sha(sha)) { fprintf(stderr, "bad sha hex\n"); return 2; }
    if (!fwsig_set_sig((uint32_t)ver, (uint32_t)flags, sig)) {
        fprintf(stderr, "bad sig hex\n"); return 2;
    }

    const char *verdict = fwsig_check(img, (uint32_t)len);
    free(img);

    if (strcmp(expect, "ok") == 0) {
        if (verdict) {
            printf("FAIL: expected accept, got refusal: %s\n", verdict);
            return 1;
        }
        printf("accepted\n");
        return 0;
    }
    if (!verdict) {
        printf("FAIL: expected refusal containing '%s', got ACCEPT\n", expect);
        return 1;
    }
    if (!strstr(verdict, expect)) {
        printf("FAIL: refusal '%s' does not contain '%s'\n", verdict, expect);
        return 1;
    }
    printf("refused: %s\n", verdict);
    return 0;
}
