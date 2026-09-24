# Vendor AMS firmware images

Bambu's own signed `_product.bin.sig` images, kept as backups. This repository
is **private**; these are vendor binaries and must not be redistributed.

| file | model | hardware rev | version | size | sha256 |
|---|---|---|---|---|---|
| `n3s_rev5-firmware-v05.00.22.19-20260616201708_product.bin.sig` | AMS HT | `n3s_rev5` | v05.00.22.19 | 160552 | `ce0fd6154fd789eecb36945f8886a5c86602cf65ce851f9de0bb013264d987b0` |
| `n3f_rev5-firmware-v05.00.22.19-20260616201033_product.bin.sig` | AMS 2 Pro | `n3f_rev5` | v05.00.22.19 | 171968 | `6b10d821da6519c91847beee0c98a9f9857b5546d35fb2c9a0ce907ec518a12c` |
| `ams_rev8-firmware-v01.00.06.87-20260109152259_product.bin.sig` | AMS 1 | `ams_rev8` | v01.00.06.87 | 96200 | `5a54adb6b688d444f89331cb85ea4f41b61bfd30c4aaebe097e5a83a37f3f45d` |

All three are `BIMH` containers whose length field at offset 8 equals the file's
own size. The `n3s` image additionally carries its source filename as ASCII
inside the first 416 bytes.

## The HT image VERIFIES our wire carve, byte for byte

`ht_artifact.json` was built by sniffing a real printer pushing this exact
firmware to an HT (`sniff_ht_fw_v184.log`, 24228 CRC-valid frames, sq 1..9320
gapless) and carving the payload out of the capture. Reassembled from the
artifact and compared against Bambu's own file:

    rebuilt from capture : 160552 bytes  sha256 ce0fd615...d987b0
    Bambu's n3s file     : 160552 bytes  sha256 ce0fd615...d987b0
    *** BYTE-IDENTICAL ***

So the capture was genuinely lossless, the carve is correct, and the flash
performed from it wrote exactly what Bambu would have. Every claim that rested
on that artifact now rests on a verified one.

## The artifact container, pinned by that comparison

`*_artifact.json` holds `header` (one hex string) and `blocks` (a list of hex
strings), each being a whole bus frame as captured. To recover the image:

    image = header[32:] + b"".join(block[32:-4] for block in blocks)

* **32 bytes** of bus framing on the front of every frame, header included.
* **4-byte trailer** on each data block. The header frame has **none** --
  getting that wrong lands you 4 bytes short with a divergence at offset 412,
  which is how this was found.
* Payload sizes for the HT: header 416 B, then 156 x 1024 B, then a final
  392 B. 416 + 159744 + 392 = 160552.

Going the other way (vendor file -> artifact) needs the per-model frame header,
which encodes the device address and unit id -- `0x1800` / id `0x80` for the HT,
`0x0700` / id `0x00` for a boxed unit. Do not assume the HT's header works for
another model; see the note on `MODELS` in `tools/ams_update.py` about artifacts
not being interchangeable.

## A SELF-BUILT ARTIFACT HAS BEEN FLASHED, AND IT WORKED

2026-09-19, on the HT (`536868840`), from `ams_fw_replay.py pack` output rather
than a captured transfer:

    ARMED for 536868840: 158 frames; up to 7 attempt(s).
    == flash attempt 1/7 ==
    loader confirmed after cmd1 #1
    header sent, erase triggered; streaming blocks...
      at block 24/156 ... 144/156
    loader: success! -- image verified, resetting into the app
    FLASH CONFIRMED: the loader verified the image and reset into the new firmware.

First attempt, no retries, and the unit came back enrolled at its own index
with its HT flag intact. Three results fall out of it:

**We no longer need a capture to flash a firmware.** Any image Bambu ships can
be packed and written. Previously an artifact could only come from sniffing a
real printer pushing that exact version to that exact model.

**The 4-byte trailer is not verified by the loader.** We sent zeros where the
printer sent `0x007f8d40`, and the loader ran its OWN hash check and answered
`success! -- image verified`. That moves the trailer from "hypothesis: probably
transfer metadata the unit ignores" to a measured fact, and it is the loader's
verdict rather than ours. It is still the one field we cannot reproduce; it
simply does not matter.

**The flash works with a populated bus.** The loader jump and all 158 frames
went through with three units on the wire (two boxed + the HT), first attempt.
The units did not have to be removed.

Done deliberately on a unit ALREADY running this exact version, so a success
changes nothing functionally and a failure had a known-good fallback staged at
`ams_firmware/ht_artifact_PROVEN_v05.00.22.19.json`. Recovery was never far
off in any case: the loader is never erased, so a refused or interrupted
transfer leaves the unit in its bootloader, not bricked.

## Version notes

`n3f_rev5` here is **v05.00.22.19**, while `ams_artifact.json` (carved
separately) is **v05.00.22.22** -- a different, newer build. They are not
interchangeable and neither supersedes the other as a record.

`ams_rev8` is the AMS 1, for which no artifact has ever been captured.
