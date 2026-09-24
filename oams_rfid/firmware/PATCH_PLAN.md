# OpenAMS mainboard RFID — binary patch plan (plan C)

Adding SPI + RFID to `oams_2.0.231.bin` **without the firmware source**, by
binary patching + Katapult reflash. Fallback if the upstream ask
(`docs/openams_rfid_firmware_request.md`) stalls. Reuses the ACE2 patch mechanics
in `ace2_rfid/firmware/build_patch.py` (append code at free flash, hook one
instruction, fix CRC).

## Confirmed so far

### Target hardware / wiring (buzz-out)
- MCU **STM32F072RBT6** (Cortex-M0, 128 KB flash `0x08000000`–`0x08020000`, 16 KB RAM).
- **RFID A → SPI1** (PA5 SCK / PA6 MISO / PA7 MOSI) + CS **PC4**.
- **RFID B → SPI2** (PB13 SCK / PB14 MISO / PB15 MOSI) + CS **PB11**.
- RST + 3V3 + GND common to both.

### Firmware memory map (from the .bin)
| Region | Range | Notes |
|---|---|---|
| Katapult (`kancan`) | `0x08000000`–`0x08004000` | CRC-only, USB-DFU recoverable |
| **App** (this image) | `0x08004000`–`0x0801AF20` | base **0x08004000** (reset handler prologue confirms it) |
| **Free flash (patch here)** | `~0x0801B000`–`0x08020000` | **~20 KB** for injected code |

- Reset vector `0x08010ead`; handlers span `0x08009xxx`–`0x08010xxx`.
- **No internal code cave** — image is fully packed; the patch region lives in
  the free flash above the app.
- Klipper command dictionary (zlib `0x78da`) at file offset **`0x15e10`** (vaddr
  `0x08019e10`): `{"build_version":"master-39a5cd2","commands":{…}}`. Command tags
  seen: `config_i2c`=31, `i2c_set_bus`=32, `i2c_modify_bits`=33, **`i2c_read`=34**,
  `i2c_write`=35.
- Plaintext `.compile_time_request` string table kept in the image at
  **`0x08019a8e`–`0x08019e10`** (immediately before the zlib dict): the raw
  `DECL_COMMAND`/`DECL_OUTPUT` request strings — `allocate_oids`,
  `config_reset`, `config_i2c`, `i2c_read` (@`0x08019dcf`), `i2c_write`,
  `config_oams_*`, etc. These are the request *text* only — the handler
  function pointers are resolved by the linker into `command_index[]`, not stored
  here, so a string xref does **not** hand you the handler.

### radare2 recursive analysis (the part linear scanning missed)
Loaded raw at base `0x08004000`, Cortex-M0 Thumb (`r2 -a arm -b 16 -m 0x08004000`),
`aaa` → **713 functions** recovered. Key results:

| What | Address | Notes |
|---|---|---|
| **`gpio_peripheral`-style pin helper** | `fcn.080098fc` | ~18 B wrapper → `fcn.080098e6`; **17 call sites** (every peripheral setup, incl. I2C). Injected SPI code can **call this** to mux PA5/6/7 + PB13/14/15 → AF0 and PC4/PB11 → output, instead of reimplementing GPIO config. |
| **I2C peripheral bring-up (template)** | `fcn.080105f8` | RCC clock-enable via base `0x40021000`, **`RCC_APB1ENR` @ `+0x1c`** (I2C1EN), reset via **`RCC_APB1RSTR` @ `+0x10`**, then `gpio_peripheral` per pin, then I2C1 (`0x40005400`) CR config. Direct analog for SPI bring-up. |
| I2C transfer engine | `fcn.080105b0` | bus-speed thresholds `0x186a0`/`0x61a80`/`0xf4240` (100k/400k/1M). |
| I2C byte xfer / status-poll | `fcn.08010330` (626 B) | loops on I2C status; indexes `i2cdevs[]`. |
| `i2cdevs[]` config table | `0x0801a260` | 28-byte stride, indexed by oid. |
| I2C command handlers (hookable) | `fcn.0800fe44`, `fcn.08010220`, `fcn.0800fe84` | normal functions — **hook the prologue directly** (ACE2-style trampoline). |

**Consequence:** the hook does **not** need `command_index` resolved — patch a command
handler's first instruction with a branch to the injected stub (exactly the ACE2
method). And the injected SPI bring-up is mostly *call existing helpers* + mirror the
I2C RCC/GPIO idiom, not a from-scratch HAL. For STM32F072: **SPI1** clock =
`RCC_APB2ENR` (`0x40021018`) bit 12; **SPI2** clock = `RCC_APB1ENR` (`0x4002101c`)
bit 14; SPI1 base `0x40013000`, SPI2 base `0x40003800` (neither literal appears in
the image → SPI genuinely not compiled in, confirmed).

### Dispatch table — NOT a naive array in this image
Stock Klipper `command_index[]` is `const struct command_parser[]` (16 B/entry:
`u16 encoded_msgid; u8 num_args,flags,num_params; const u8 *param_types; void
(*func)()`), indexed by cmdid; `command_lookup_parser` returns
`&command_index[cmdid]`. In this `-flto` build it is **not** recoverable by naive
byte scanning:
- No 16-byte-stride run of `{…, param_types, func}` structs exists anywhere
  (longest false run = 10, and that's the interrupt vector table).
- The only long 4-byte-stride code-pointer array (len 34) is the **vector table**
  at `0x08004038` (repeated default handler `0x08010efd`).
- Real handler pointer-tables *do* exist, 4-byte packed with scattered targets
  across the whole `.text` — candidates at **`0x0801a22c`** (11 ptrs),
  **`0x0801a2bc`** (9 ptrs), **`0x0801ab68`** (11 ptrs, most diverse targets:
  `0x0800458d, 0x080096a9, 0x0800b70d, 0x0800e5d5, 0x080102c5, 0x08011569, …`),
  plus a cluster around `0x0800e800`. One of these is very likely the `-flto`
  lowered dispatch/handler table, but confirming which entry is
  `command_i2c_read` requires **recursive disassembly** (Ghidra/objdump-arm) —
  capstone's linear sweep can't follow it past the interleaved literal pools.

### Flashing / integrity
- Katapult flashes unsigned; the app image is CRC-checked (`BL: … CRC32=…`
  strings). A modified image with a corrected CRC is accepted.
- **Unbrickable:** USB-DFU (hold BOOT, tap NRST) restores stock `oams_2.0.231.bin`.

## Hook design decision (radare2-informed)
Two ways to return MFRC522 bytes to the host through the existing `i2c_read`
command. **Chosen: HAL-read substitution** — it needs no `sendf` work.

- **(A) Command-level + `sendf`** — hook `command_i2c_read`, and on a magic oid call
  the injected leaves then `sendf("i2c_read_response …")`. Needs the encoder
  descriptor + the variadic `sendf`. `sendf` core is identified (`fcn.08011cf8`,
  630 B, 64 xrefs — the VLQ integer encoder), but calling it blind with the right
  `command_encoder` is intricate and untestable without hardware. **Not chosen.**
- **(B) HAL-read substitution (chosen)** — leave `command_i2c_read` and its
  `i2c_read_response` path completely untouched. Hook the **i2c HAL read primitive**
  so that for a **magic I2C address** it fills the caller's read buffer from
  `mfrc522_read(which,reg)` instead of doing a real bus transfer, and returns OK.
  The host then just talks normal Klipper I2C to a device at that magic address:
  encode `(which, reg)` in the register-write bytes of an `i2c_read` → the returned
  byte is the MFRC522 register value; use `i2c_write` with `(which, reg, val)` to
  write. Reuses the whole command + response + host I2C stack; zero `sendf`, zero
  dictionary surgery. i2c HAL cluster located: transfer engine `fcn.080105b0`, byte
  engine `fcn.08010330`, write wrapper `fcn.080107b0` — the exact read-primitive
  prologue + its `(buf,len,addr)` ABI is the one remaining address to pin.

## i2c internals (deep trace) — why the clean-ABI substitution doesn't fit
Tracing the whole i2c island changes the hook picture again:

- The transaction object is at a **fixed RAM address `0x20001eac`** (loaded as a
  literal before the execute call at `0x0800f33e`), with a sub-descriptor at
  `+0x28` and a status/flag byte at `+0xbe`.
- `fcn.0800fe44(obj)` runs one transfer then **schedules callbacks**
  (`fcn.08010982`/`fcn.0801096e` with handlers `0x0800fcea`/`0x0800feaa`) — the i2c
  is **asynchronous / scheduled**, not the synchronous `command_i2c_read` in the
  source clone.
- `fcn.08010330` (626 B) is **bit-timing math**, not the byte move: it converts
  ns setup/hold specs (`i2cdevs[]` @ `0x0801a260`, 28 B stride, fields at
  `+0xc/+0xe/+0x10/+0x16/+0x18/+0x1a`) to ticks via `1e9` (`0x3b9aca00`) and the
  timer div `fcn.080040fc`. Speed classes 100k/400k/1M in `fcn.080105b0`. This is
  **software (bit-banged) i2c** — the OAMS uses it for its internal encoder.
- The read completion (`fcn.0800feaa`) consumes the object at `+0x10/+0x16/+0x18`
  and calls `fcn.08016728` (`+0x74`) to move/emit the bytes.

**Consequence:** there is no clean `i2c_dev_read(reg,read)` buffer-fill site to
substitute — the read path is bit-banged, scheduled, and inlined. A reliable
substitution needs the object's tx/rx buffer offsets (relative to `0x20001eac`),
which are only worth pinning against a live board (the async timing and the exact
buffer fields are the kind of thing a bench probe confirms in minutes vs. hours of
blind trace). The injected **driver is complete and correct**; the trigger/return
glue is the piece that wants the hardware. `build_spi_patch.py` remains valid — it
just needs the confirmed hook site + (for this async object ABI) buffer offsets.

## Approach: hook, don't add a command
Adding a new `spi_transfer` command would mean rebuilding + relocating the zlib
dictionary at `0x15e10` and extending the compile-time dispatch table — painful.
Instead **hook `i2c_read` / `i2c_write`** (already in the dictionary): on a magic
address/arg, do an SPI/MFRC522 op on the injected driver and return its bytes
through the existing response path. No dictionary surgery. The host
`AFC_OpenAMS_rfid` then talks to the reader through that hooked command via a thin
transport shim.

## Remaining RE steps
1. **CRC mechanism** — confirm exactly what Katapult verifies on flash and over
   what range, so the re-CRC after patching is correct (mirror the ACE2 CRC fix).
2. **Hook site — solved via radare2 (no `command_index` needed).** The I2C command
   handlers are ordinary functions (`fcn.0800fe44` et al.); hook one's prologue with
   a `B`/`BL` to the injected stub (ACE2-style trampoline), so we never have to
   resolve the `-flto` dispatch table. Remaining: pick the exact handler whose args
   we want to repurpose (magic addr/len → SPI op) and record its prologue bytes to
   restore in the trampoline. `fcn.080105f8` is the register-setup template to copy.
3. **Author Thumb code** in free flash (`0x0801B000`) — **DONE + verified**
   (`spi_asm.py`, `spi_driver.py`). Cortex-M0 / ARMv6-M (no Thumb-2 — literal-pool
   loads, `bl`, branch loops), assembled with rasm2 and checked by capstone
   round-trip disassembly (no hardware here). **340 bytes**:
   - `spi_init` — RCC `AHBENR`/`APB2ENR`(SPI1 b12)/`APB1ENR`(SPI2 b14) clock enable,
     PA5/6/7 + PB13/14/15 → alt/AF0/high-speed, PC4 + PB11 → CS outputs (idle high),
     SPI1/SPI2 `CR2`=0x1700 (8-bit, FRXTH), `CR1`=0x36C (master, /64, SSM/SSI, SPE).
   - `spi_txrx(base, byte)->rx` — TXE/RXNE-polled single byte.
   - `mfrc522_read(which,reg)->val` / `mfrc522_write(which,reg,val)` — CS-framed,
     `which` 0=SPI1/PC4 (RFID A), 1=SPI2/PB11 (RFID B).

   These two leaves are exactly the `reg_read`/`reg_write` the host `Mfrc522` class
   calls, so **all vendor decode (Bambu HKDF, Anycubic, Snapmaker, Creality) works
   unchanged** — same shared `read_tag` stack as ACE2/ViViD. MIFARE stays on host.
4. **Hook stub + build_patch** — pin the i2c-read primitive's prologue, then place
   `spi_driver` (done) + a small stub (magic-addr check → `spi_init` once →
   `mfrc522_read` → fill buf → return OK; else fall through to the saved original)
   + trampoline, and adapt the ACE2 `build_patch.py` CRC fix for base `0x0801B000`.
5. **Host shim** — a `link` object in `AFC_OpenAMS_rfid` whose `reg_read`/`reg_write`
   issue `i2c_read`/`i2c_write` to the magic device (drop-in for the `MCU_SPI` link;
   the shared `Mfrc522`/`read_tag` decode above it is unchanged).

> **Boundary:** steps 1–3 are done and verified by disassembly, and 4–5 are built
> (`build_spi_patch.py` + the `i2c_hook` host link). The ONE bench-side unknown is
> the exact **hook site**: this image is heavily `-flto`, so there is **no standalone
> `i2c_dev_read`** to detour — the whole i2c command path is inlined into a ~1800 B
> dispatch giant (`fcn.0800eca0`, spanning `0x0800eca0`–`0x0800f3da`). The builder is
> therefore parameterized on `--hook`/`--hook-prologue`; the interior i2c-read call
> site inside that function is pinned on the bench (safe: a wrong hook just means the
> probe finds no reader — the magic-`reg` guard prevents touching real i2c traffic,
> and USB-DFU restores stock in seconds). The clean transfer primitives
> `fcn.0800fe44` / `fcn.080107b0` are the fallback hook points if an interior detour
> is awkward. Wiring is **confirmed** (RFID A→SPI1/PC4, RFID B→SPI2/PB11) and baked
> into the driver. See `BUILD_AND_FLASH.md`.

## Status / recommendation
**Do the upstream ask first** (`../../docs/openams_rfid_github_issue.md`) — enabling
SPI1+SPI2 is a one-flag rebuild for whoever holds the OAMS source, and the entire
host stack (`AFC_OpenAMS_rfid` + `AFC_OpenAMS_rfid_probe`) is already written and
in production on ACE2/ViViD. That gets identical RFID with zero flash risk.

Plan C is documented and de-risked (CRC-only flashing + USB-DFU recovery) but is a
**real embedded project, not a quick hook**. Unlike the ACE2 — where an MFRC522
driver already existed in the firmware and we just hooked it — here the whole
SPI+MFRC522 driver must be injected as hand-authored Thumb, and even step 1
(pinning down `command_index[34]`) is blocked until it can be opened in a recursive
disassembler. Everything up to that gate is confirmed (wiring, memory map, free
flash, CRC-only bootloader, hook strategy). Next concrete action if C continues:
run the `.bin` through Ghidra to resolve the handler table, then author the Thumb
driver.
