# Anycubic ACE 2 Pro — Firmware Behavior Analysis

Dual-source reverse-engineering:

- **gklib** — Anycubic Kobra S1 "Klipper-Go" firmware, version 2.7.0.9.
  ARM 32-bit ELF binary, imagebase `0x10000`. Analysis via IDA Pro decompilation.
- **ACE2 MCU firmware** — `ACE2_V1.1.31_20260306.bin`, STM32F1 Cortex-M3.
  Loaded at `0x08008000`. Analysis via IDA Pro disassembly/decompilation.

---

## Background

The Kobra S1 runs a custom Go port of Klipper (`gklib`). It communicates with the ACE 2 Pro multi-material
unit over USB-UART (CH343 chip) at **230400 baud** using a custom framed Protocol Buffers protocol.

**Frame format:**
```
FF AA [flags] [seq_lo seq_hi] [cmd] [payload_len] [protobuf payload] [crc16_lo crc16_hi] FE
```
CRC is CRC16-Kermit (polynomial 0x8408, init 0xFFFF) computed over `flags` through end of payload.

Key Go structs in the firmware:
- `FilamentHub` — orchestrates all multi-material operations
- `ACEProxyV2` — represents a single connected ACE 2 device
- `V2Manager` — manages multiple ACE 2 devices (daisy-chain)
- `FilamentStuckChecker` — monitors encoder feedback to detect clogs

---

## 1. Error Handling

Error handling is driven by `FilamentHub.update_status` (0x644768), which polls slot states and
transitions the state machine.

### ASSIST_ERROR (slot state 0x83)

```
if slot.Status == "assist_error"
    AND feed_assist is currently active
    AND ACE workstate == BUSY:

    log "ace %d filament %d hardware error detected during feed assist"
    pause_on(ASSIST_ERROR)
    return
```

The printer **pauses immediately**. There is no automatic retry. The operator must physically resolve
the hardware issue and resume the print.

### Other error states

| Condition | Behavior |
|---|---|
| `FilamentStuckChecker.checkClog()` detects excess `feed_assist_count` | Log "filament stuck, go check ace %d filament %d feed_assist_count %d", `pause_on(stuck_error)` |
| `ContAssistTime > 8.0 s` (ACE 2.0 firmware) | Multi-stage tangle detection: increments `filament_tangle_stage` (0 → 1 → 2), pauses on stage 2 |
| `ContAssistTime > 8.0 s` (ACE 1.0 firmware) | Direct `pause_on` (no staging) |
| `status == "home_err"` or `"gear_err"` | `ace_err = status_string`, `pause_on(ace_error)` |
| `ROLLBACK_ERROR` (0x82), `PRELOAD_ERROR` (0x84) | Handled by the same state machine; result in print pause |
| `STUCK_ERROR` (0x85) | Caught by stuck checker before ACE reports it |
| `TANGLED_ERROR` (0x86) | Caught by tangle timer logic above |
| `MOTOR_ERROR` (0x87) | Falls through to `home_err`/`gear_err` path |

All error paths terminate in a **print pause** — there is no silent recovery.

### Error recovery

1. Printer pauses; user resolves the physical issue
2. User resumes print (GCode `M24` or UI button)
3. `update_status` observes slot state returned to `IDLE`/`READY` and clears the internal error flag
4. Feed sequence restarts from the current position

To trigger programmatic recovery without a full resume: send `STOP_FEED_OR_ROLLBACK` (cmd 9) to
cancel current motion, then issue a new `FEED_OR_ROLLBACK` (cmd 8). The ACE state machine resets
to `IDLE` on a successful stop.

---

## 2. Firmware Update (OTA / IAP)

The ACE 2 supports in-application firmware updates over the same UART protocol. The GCode entry
point is `OTA_FILAMENT_START`. The update is a 3-step IAP sequence.

### Step 1 — Announce update: `IAP_UPGRADE` (cmd 2)

Implemented in `ACEProxyV2.OtaStart` (0x62f8d0).

```
Command:  2  (IAP_UPGRADE)
Timeout:  2 s
Payload:  UpgradeRequest {
              size:    uint32   // total firmware image size in bytes
              crc:     uint32   // CRC of full firmware image
              version: string   // target firmware version string
          }
Log: "ACEProxyV2 device %d OtaStart: size=%d, crc=%d, version=%s"
```

### Step 2 — Send firmware chunks: `IAP_FIRMWARE` (cmd 3)

Implemented in `ACEProxyV2.OtaSendChunk` (0x62fe00). Called repeatedly until all chunks are sent.

```
Command:  3  (IAP_FIRMWARE)
Timeout:  2 s per chunk
Payload:  FirmwareRequest {
              address:  uint32  // byte offset of this chunk in the image
              firmware: bytes   // chunk data
          }
Log on failure: "ACEProxyV2 device %d OtaSendChunk failed (addr=0x%08X, size=%d): %v"
```

### Step 3 — Finalise: `IAP_UPGRADE_FINISH` (cmd 4)

Implemented in `ACEProxyV2.OtaFinish` (0x6301cc).

```
Command:  4  (IAP_UPGRADE_FINISH)
Timeout:  5 s
Payload:  GenericRequest {}  // empty protobuf — zero payload bytes
Log: "ACEProxyV2 device %d OtaFinish"
```

After `OtaFinish` the ACE reboots. The printer polls `GET_INFO` (cmd 7) and `IAP_VERSION` (cmd 5)
to confirm the new version is running.

> **Note:** The `crc` field in `UpgradeRequest` is a CRC of the complete firmware image. It is
> separate from the per-packet CRC16-Kermit framing used on every UART frame.

---

## 3. GCode Filament Switching

`SWITCH_FILAMENT` is a GCode command implemented in the firmware.

### GCode syntax

```gcode
SWITCH_FILAMENT ACE=<ace_id> INDEX=<filament_index>
```

Handler: `FilamentHub.Cmd_SWITCH_FILAMENT` (0x65481c). Reads `ACE` and `INDEX` parameters,
calls `switch_filament(ace_id, filament_id)`, logs "switch finish" on completion.

The standard `T<n>` toolchange GCode also routes through this path via the gcode-to-filament
mapping table.

### Full switch logic: `switch_filament` (0x64efc8)

```
1. Set target_ace_id, target_filament_id

2. If currently cleaning or refilling:
       stop current operation + run Cmd_CLEAN_PIPE

3. If already loaded with the requested filament AND filament_present sensor is true:
       invoke completion callback, return (no-op — already done)

4. If filament_tracker_enable AND filament is currently present:
       Cmd_UNWIND_ALL_FILAMENT  (retract the current filament fully)

5. If !bootup_calibrated AND filament_calibrate==1 AND gcodeMapping > 1:
       filament_recalibrate()
       bootup_calibrated = true

6. If recalibrate_period has elapsed since last calibration:
       filament_recalibrate() for the current slot

7. set_current_filament(-1, -1)        // mark: no filament active

8. feed_filament(ace_id, filament_id)  // drive new filament forward

9. On success: update multi_material_temp if a per-material temperature is configured
```

---

## 4. Feed Check

Feed check is a motion-monitoring safety feature. The ACE measures actual filament movement via
its optical encoder and raises an error if the movement deviates beyond set thresholds.

### Setting parameters: `SET_FEED_CHECK` (cmd 19)

Implemented in `ACEProxyV2.SetFilamentFeedCheckParam` (0x62f108).

```
Command:  19  (SET_FEED_CHECK)
Timeout:  200 ms
Payload:  SetFeedCheckRequest {
              check_length: uint32   // minimum encoder reading required to pass the check (mm)
              error_length: uint32   // commanded feed distance at which the check is evaluated (mm)
          }
Log: "ACEProxyV2 device %d set feed check params: check_length=%d, error_length=%d"
```

**Default values** (gklib `initializeDevice` at `0x6292B0`):
```
check_length = 100   // sent as 0x64
error_length =  90   // sent as 0x5A
```

Both values are stored inside the ACE MCU as single bytes at `byte_20000096` (`check_length`)
and `byte_20000097` (`error_length`). Valid range is **1–255** per field.

**How the check works:**

The feed check logic runs entirely inside the **ACE 2's own MCU firmware** — gklib only sends
these two parameters and then monitors the slot state for `FEED_ERROR`. The evaluation is
performed in `sub_8009D74`:

1. After the ACE has been commanded to feed `error_length` units, it samples the encoder count.
2. A fixed **scale factor of 1.2342** (double constant at `0x8009EAC`) converts the commanded
   distance into the expected encoder reading:
   `expected = error_length × 1.2342`
3. If `encoder_count < check_length`, the ACE transitions to `FEED_ERROR (0x81)`.

With the gklib defaults (error_length=90, check_length=100): expected ≈ 111 encoder units;
the slip window before an error is raised is ≈ 11 encoder units (~8.9 mm equivalent).

**Tuning guidance — minimising assist errors from encoder deficit:**

During buffer-assist cycles the buffer spring back-pressure can cause the encoder to under-read
relative to the commanded distance, producing spurious `FEED_ERROR` / `ASSIST_ERROR` transitions.
To widen the tolerance without masking real jams:

- **Decrease `check_length`** (lower the minimum encoder threshold) — directly increases the
  acceptable slip window with no change to when the check fires.
- **Do not increase `error_length` beyond the actual assist stroke** — the check would then fire
  after the assist has already completed, making it useless.
- Example: `check_length=80, error_length=90` raises the tolerance from ~8.9 mm to ~24 mm while
  keeping the evaluation point the same.

The field name `check_length` is the *pass threshold* (minimum acceptable encoder reading),
not the distance at which the check fires. `error_length` is the commanded distance that
triggers the evaluation.

### Web API handler: `handle_set_feed_check` (0x64c6b0)

Accepts a POST with `ace_id` (float), `check_len`, and `error_len`. Looks up the ACE proxy by ID
and calls `SetFilamentFeedCheckParam`.

### Diagnostics: `GET_FEED_INFO` (cmd 76)

Returns per-slot feed statistics:
```
FeedInfoResponse {
    feed_info: repeated FeedInfo {
        steps:   int32   // motor steps commanded
        length:  int32   // length commanded (mm)
        decoder: int32   // length measured by encoder (mm)
    }
}
```

`get_and_record_feedinfo` (0x62a998) reads this data into a circular diagnostic log formatted as
`[idx] %08d,%08d,%08d` (steps, length, decoder). gklib does not evaluate the thresholds itself;
it only logs the raw values for diagnostics. The FEED_ERROR transition is triggered by the ACE 2
MCU when `decoder < check_length` at the `error_length` checkpoint.

---

## 5. Calibration

There are two separate and independent calibration mechanisms.

### A. Klipper-side filament length calibration: `filament_recalibrate` (0x653070)

This calibrates how far the klipper-side firmware unwinds filament for each slot. It runs
automatically inside `switch_filament` at first boot (step 5) and periodically thereafter (step 6).

**Algorithm — for each non-empty slot:**

```
1. move_to_throw_position()         // move printhead to safe purge/discard position

2. get_nonempty_filaments()         // enumerate all loaded slot indices

3. For each slot (skipping any excluded slots):
   a. Start feed assist mode on this slot
   b. Wait up to 100 s (polling every 1 s) for filament_present sensor to confirm
   c. Issue unwind command:
          length = safe_unwind_len + random(0, 50) mm
          speed  = getUnwindFilamentSpeed()
   d. wait_ace_action_with_filament(timeout = 9 s)
   e. If ACE workstate == BUSY: read encoder decoder feedback in a loop:
          if decoder < safe_unwind_len * 8 / 4:
              retry with length = 2 * safe_unwind_len
          log "filament_recalibrate ace %d filament %d unwind success, unwind_length=%d"
```

The randomised unwind length `+ random(0, 50) mm` prevents systematic under/overshoot
accumulating across repeated calibration runs.

### B. ACE-side optical sensor calibration: `LINEAR_KEY_CALIBRATE` (cmd 15)

This calibrates the ACE's internal optical slot sensors (filament insert, empty, and buffer
sensors for each channel). It is a manufacturing and maintenance command.

```
Command:  15  (LINEAR_KEY_CALIBRATE)
Payload:  LinearCalibrationRequest {
              id:   KeyIndex   // which sensor to calibrate (see table below)
              type: uint32     // calibration type parameter
          }
```

**`KeyIndex` sensor map:**

| Value | Sensor |
|---|---|
| 0 | CH1_INSERT — channel 1 filament inserted |
| 1 | CH1_EMPTY — channel 1 filament empty |
| 2 | CH1_BUF_RST — channel 1 buffer reset |
| 3 | CH1_BUF_BACK — channel 1 buffer back |
| 4–7 | CH2 sensors (same pattern) |
| 8–11 | CH3 sensors |
| 12–15 | CH4 sensors |
| 16 | CHN_BUF_FEED — shared buffer feed sensor |

Normal print operation does **not** require triggering this command. It is only needed after
replacing ACE sensor hardware or if sensors drift out of calibration over time.

---

## Quick Reference: ACE 2 Command IDs

Derived from both gklib (host-side, sections above) and ACE2 MCU firmware static analysis
(see [ACE2 MCU Firmware Analysis](#ace2-mcu-firmware-analysis) section below for handler addresses).

| ID | Hex | Name | Direction | Purpose |
|---|---|---|---|---|
| 0 | 0x00 | DISCOVER_DEVICE | host → ACE | Find ACE on bus; ACE replies with 96-bit STM32 UID |
| 1 | 0x01 | ASSIGN_DEVICE_ID | host → ACE | UID verification + seq counter assignment |
| 2 | 0x02 | IAP_UPGRADE | host → ACE | OTA start (size + crc32 + version string) |
| 3 | 0x03 | IAP_FIRMWARE | host → ACE | OTA data chunk (address + bytes) |
| 4 | 0x04 | IAP_UPGRADE_FINISH | host → ACE | OTA complete; ACE reboots |
| 5 | 0x05 | IAP_VERSION | host → ACE | Query boot/app version strings |
| 6 | 0x06 | GET_STATUS | host → ACE | Full status: slots, temps, motors, odometer |
| 7 | 0x07 | GET_INFO | host → ACE | Firmware version + device info (sets init flag) |
| 8 | 0x08 | FEED_OR_ROLLBACK | host → ACE | Start feed or retract on a slot |
| 9 | 0x09 | STOP_FEED_OR_ROLLBACK | host → ACE | Cancel active feed/retract |
| 10 | 0x0A | UPDATE_SPEED | host → ACE | Change feed speed on active operation |
| 11 | 0x0B | DRYING | host → ACE | Start drying cycle (temp + duration + fan) |
| 12 | 0x0C | SET_DRY_TEMP | host → ACE | Set PID target (temp+7°C offset) + auto-adjust setpoint. Does NOT stop drying — send DRYING(temp=0) to stop |
| 13 | 0x0D | GET_RFID_CACHE | host → ACE | Read last-cached RFID data for a slot |
| 14 | 0x0E | SET_RFID_ENABLE | host → ACE | Enable/disable RFID reading per slot |
| 15 | 0x0F | LINEAR_KEY_CALIBRATE | host → ACE | Set optical sensor ADC threshold |
| 16 | 0x10 | GET_MATERIAL_INFO | host → ACE | Read stored material name + status for a slot |
| 17 | 0x11 | SET_SLOT_STATUS | host → ACE | Write slot status byte |
| 18 | 0x12 | SET_MATERIAL_NAME | host → ACE | Write 21-byte material name to a slot |
| 19 | 0x13 | SET_FEED_CHECK | host → ACE | Set encoder feed-check thresholds |
| 20 | 0x14 | SET_PRINTER_STATUS | host → ACE | Notify ACE of current printer state |
| 64 | 0x40 | GET_TEMP | host → ACE | Detailed temperature + dry state readings |
| 65 | 0x41 | SET_DRY_POWER | host → ACE | Set heater power level |
| 66 | 0x42 | SET_VALVE | host → ACE | Control valves / binary actuators |
| 68 | 0x44 | GET_FILAMENT_INFO | host → ACE | Read live RFID tag (full decode). gklib uses CMD 13 instead; CMD 68 is test/debug only |
| 70 | 0x46 | FLASH_LED | host → ACE | LED animation pattern control |
| 71 | 0x47 | SET_FAN | host → ACE | Fan speed (0–100%) |
| 72 | 0x48 | MOTOR_MOVE | host → ACE | Direct motor step command |
| 73 | 0x49 | GET_SENSOR_STATE | host → ACE | 17-channel filament sensor bitmask |
| 75 | 0x4B | DRY_CMD | host → ACE | Dry subtask command |
| 76 | 0x4C | GET_FEED_INFO | host → ACE | Per-slot encoder/step feed diagnostics |
| 77 | 0x4D | MOTOR_TEST | host → ACE | Motor self-test command |
| 78 | 0x4E | GET_MOTOR_STATUS | host → ACE | Motor/drive status query |

### Live opcode probe (firmware V1.1.31)

Confirmed on real hardware during development by sending raw opcodes to the
unit. The following opcodes are dangerous and must never be sent to the device,
based on these findings:

- **Opcode 3 (`IAP_FIRMWARE`) BRICKS the unit.** Sending it drops the ACE2 into
  its firmware-flash (bootloader) path: it stops answering the normal protocol
  and its LEDs flash rapidly. A Klipper `FIRMWARE_RESTART` does **not** recover
  it — only a physical power-cycle. Opcodes 2 (`IAP_UPGRADE`) and 4
  (`IAP_UPGRADE_FINISH`) are the rest of that OTA sequence; 1 reassigns the bus
  address. 72/77 drive the motors directly.
- **Opcodes verified as safe reads:** 5 (→ `V1.1.31`), 6, 7, 13, 16, 64, 73, 76,
  78. Opcode 2 replies `code=0` and 4 is ignored, but both are in the OTA band
  and left blocked.
- **Opcode 16 (`GET_MATERIAL_INFO`)** — request field 1 = slot index; response
  field 2 = nested `{1: name}`. Read back e.g. slot 0 = `S0395MB251230046650C3`.
- **Opcode 18 (`SET_MATERIAL_NAME`)** — request field 1 = slot index, field 2 =
  the name string. Variable length accepted; persists to NVM and clears the slot
  status byte. Round-trip verified (write test name → read back → restore).
- **Opcode 73 (`GET_SENSOR_STATE`)** — the 17-channel sensor bitmask arrives in
  field 1 (e.g. `70928` = `0x114D0`). A generic decoder mis-reads this as an
  error code; the driver decodes it into `sensor_bitmask` + a per-channel list.

---

## Slot State Reference

| Value | Name | Meaning |
|---|---|---|
| 0x00 | READY | Idle, ready for commands |
| 0x01 | FEEDING | Actively feeding filament forward |
| 0x02 | ROLLBACK | Actively retracting filament |
| 0x03 | ASSISTING | Providing buffer assist |
| 0x04 | ROLLBACK_ASSISTING | Retracting while assisting |
| 0x05 | PRELOADING | Pre-loading filament to buffer |
| 0x06 | UPGRADING | Firmware update in progress |
| 0x81 | FEED_ERROR | Feed motion: encoder below check_length threshold |
| 0x82 | ROLLBACK_ERROR | Retract motion error |
| 0x83 | ASSIST_ERROR | Hardware error during buffer assist |
| 0x84 | PRELOAD_ERROR | Preload motion error |
| 0x85 | STUCK_ERROR | Filament jam detected |
| 0x86 | TANGLED_ERROR | Filament tangle detected |
| 0x87 | MOTOR_ERROR | Motor driver fault |

---

## ACE2 MCU Firmware Analysis

Static analysis of `ACE2_V1.1.31_20260306.bin` loaded at base address `0x08008000`.
Performed via IDA Pro decompilation and disassembly of the ARM Thumb-2 binary.

---

### Identity

| Field | Value |
|---|---|
| Firmware version | `V1.1.31` (at `0x08018C20`) |
| Build date | 2026-03-06 (from filename) |
| Binary base | `0x08008000` |
| Code range | `0x08008000`–`0x080197A8` (~71.6 KB) |
| SRAM | 40 KB at `0x20000000`–`0x20009A80` |

---

### Hardware

**MCU:** STM32F1-series (Cortex-M3), confirmed by peripheral base addresses.

| Peripheral | Address | Notes |
|---|---|---|
| UART4 | `0x40004C00` | Main comm port to host via CH343 USB-UART |
| DMA2 | `0x40020400` | Ch4=UART4 RX, Ch3=UART4 TX |
| ADC1 | `0x40012400` | NTC temperature measurement |
| RCC | `0x40021000` | Clock enable (APB1ENR, APB2ENR, AHBENR) |
| EXTI | `0x40010400` | PA0 rising edge = encoder pulse interrupt |
| TIM7 | `0x40001400` | Software PWM for motor (PSC=119, ARR≈20000) |
| IWDG | `0x40003000` | Independent watchdog |
| STM32 UID | `0x1FFFF7E8`–`0x1FFFF7F0` | 96-bit factory UID (3×32-bit words) |

**UART4 configuration:**
- Baud rate: **230,400** (`BRR = 0x38400` passed to BRR calculator at `sub_80139F8`)
- Format: 8N1 (8-bit word, no parity, 1 stop bit)
- DMA2 channel 4: RX (circular buffer at `unk_20001E78`, 0x802 bytes)
- DMA2 channel 3: TX

---

### FreeRTOS Tasks

| Task name | Body address | Purpose |
|---|---|---|
| `Command Processing` | `0x8014488` | UART4 frame parser + command dispatcher |
| `filament move` | `0x8014EAC` / `0x800A0B0` | Motor control and feed/retract sequencing |
| `dry` | `0x800BEC8` | Heater PID + temperature monitoring |
| `rfid` | `0x800EA14` | RFID tag read/write (4 slots) |
| `IAP upgrade` | `0x8013FB8` | Firmware OTA receive + flash write |
| `misc` / motor init | `0x80159B0` area | Fan, LED, sensors, motor calibration |
| `OdometerTimer` | — | Encoder odometry tracking |

---

### Frame Format

Confirmed from ACE2 MCU 8-state parser at `0x8014802` (inside Command Processing task):

```
FF AA [SEQ:1] [TYPE_HI:1] [TYPE_LO:1] [CMD:1] [LEN:1] [PAYLOAD:LEN bytes] [CRC16_LO:1] [CRC16_HI:1] FE
```

| Field | Size | Notes |
|---|---|---|
| `FF AA` | 2 | Sync preamble (parser pre-state: scans until 0xFF then 0xAA) |
| `SEQ` | 1 | Sequence number, validated against expected counter `word_2000111C` |
| `TYPE` | 2 | Frame type / flags (big-endian 16-bit, stored at `dword_200011EA+2`) |
| `CMD` | 1 | Command ID byte (dispatched via linked list) |
| `LEN` | 1 | Payload length in bytes |
| `PAYLOAD` | LEN | Protobuf-encoded request data |
| `CRC16` | 2 | CRC-16/Kermit (poly `0x8408`, init `0xFFFF`) over TYPE..PAYLOAD |
| `FE` | 1 | End-of-frame marker |

**CRC function:** `sub_8010464` — initial value `0xFFFF`, polynomial via lookup
`i = (8*v5) ^ (v5>>4) ^ ((v5<<8)|(v4>>8))` where `v5 = i ^ byte ^ (16*(i^byte))`.

**Sequence counter:** Validated in state 1. After ASSIGN_DEVICE_ID, the host's chosen seq byte
is stored into `word_2000111C`. Subsequent frames must increment the seq counter.

---

### Dispatch Architecture

The firmware uses a three-layer dispatch:

**Layer 1 — Frame parser** (state machine, `dword_200011FC`):
Eight states parse the incoming byte stream. On CRC-valid completion (state 8), the CMD byte
is used to look up the handler in the linked list.

**Layer 2 — Command linked list** (`dword_200012D4`):
Each registered command creates a 24-byte node via `sub_800B7A4(cmd_id, thunk)`:

```
node[0]  = prev_ptr
node[4]  = next_ptr
node[8]  = auto-increment ID
node[12] = cmd_id
node[16] = handler_thunk (Thumb function pointer)
node[20] = status byte
```

The dispatcher walks this list to find a matching `cmd_id`, then calls the thunk.

**Layer 3 — Protobuf dispatch** (`sub_8009C5C`):
Every thunk calls `sub_8009C5C(a1, a2, a3, a4, a5, table_entry_ptr)` which:
1. Decodes the protobuf request payload using the table entry's request descriptor
2. Calls the handler function with `(context, decoded_request, response_buffer)`
3. Encodes the protobuf response using the table entry's response descriptor
4. Queues the response frame for transmission

**Static dispatch table** at `0x8018C5C`: 26 entries × 12 bytes each.
Each entry: `[req_descriptor_ptr:4][resp_descriptor_ptr:4][handler_fn:4]` (Thumb addresses).

**Dynamic dispatch entries** (7 entries): For motor/misc commands, the 12-byte entry struct is
built at runtime in SRAM (e.g., `dword_20000E70`, `dword_20000FC8`, etc.) with the same layout.

---

### Command Handler Map

Complete list of all 33 registered commands extracted from ACE2 MCU firmware `V1.1.31`.

#### Registered by Command Processing task (`0x8014488` init phase)

| CMD | Hex | Thunk | Table Entry | Handler | Function |
|---|---|---|---|---|---|
| 0 | 0x00 | `sub_800FADE` | entry 8 | `sub_800B9CE` | DISCOVER: writes 96-bit UID (`0x1FFFF7E8`) to response |
| 1 | 0x01 | `sub_800FAFA` | entry 9 | `sub_800B9E6` | ASSIGN: verifies 3×UID words + sets seq counter |
| 6 | 0x06 | `sub_800FB16` | entry 7 | `sub_800B840` | GET_STATUS: slots, state, temps (×3), motors, odometer |
| 7 | 0x07 | `sub_800FB32` | entry 6 | `sub_800B7F2` | GET_INFO: version string + sets `byte_20000D98` init flag |

#### Registered by IAP upgrade task (`0x8013FB8`)

| CMD | Hex | Thunk | Table Entry | Handler | Function |
|---|---|---|---|---|---|
| 2 | 0x02 | `sub_800FBDA` | entry 15 | `sub_800D4E4` | IAP_UPGRADE: stores OTA metadata, sets `byte_20001554` flag |
| 3 | 0x03 | `sub_800FBF6` | entry 17 | `sub_800D564` | IAP_FIRMWARE: data chunk receive |
| 4 | 0x04 | `sub_800FC12` | entry 18 | `sub_800D5C4` | IAP_UPGRADE_FINISH: verify + reboot |
| 5 | 0x05 | `sub_800FC2E` | entry 16 | `sub_800D528` | IAP_VERSION: returns 11-byte version struct from `0x08018000` |

#### Registered by dry task (`sub_800BEC8`)

| CMD | Hex | Thunk | Table Entry | Handler | Function |
|---|---|---|---|---|---|
| 11 | 0x0B | `sub_800FB4E` | entry 10 | `sub_800BD1C` | DRYING: sets temp/duration/fan, starts heater state machine (`dword_20000600`=1) |
| 12 | 0x0C | `sub_800FB86` | entry 12 | `sub_800BDF6` | SET_DRY_TEMP: sets PID target = temp+7°C via `sub_800D8AC`; stores setpoint in `word_20000670` for auto-adjust. Does **not** change `dword_20000600` (dryer state) |
| 64 | 0x40 | `sub_800FB6A` | entry 11 | `sub_800BD7E` | GET_TEMP: returns current temps × 3 + heater mode + remaining time |
| 65 | 0x41 | `sub_800FBBE` | entry 14 | `sub_800BE9C` | SET_DRY_POWER: set heater power level |
| 75 | 0x4B | `sub_800FBA2` | entry 13 | `sub_800BE0A` | DRY_CMD: dry subtask control (temp parameter) |

#### Registered by filament task (`sub_8014EAC`)

| CMD | Hex | Thunk | Table Entry | Handler | Function |
|---|---|---|---|---|---|
| 8 | 0x08 | `sub_800FA36` | entry 4 | `sub_800B578` | FEED_OR_ROLLBACK: validate slot/speed/mode, queue `sub_800B22C` |
| 9 | 0x09 | `sub_800FA52` | entry 5 | `sub_800B644` | STOP_FEED_OR_ROLLBACK |
| 10 | 0x0A | `sub_800FA6E` | entry 1 | `sub_800B4A0` | UPDATE_SPEED |
| 19 | 0x13 | `sub_800FAA6` | entry 3 | `sub_800B3AC` | SET_FEED_CHECK: encoder thresholds `check_length` / `error_length` |
| 76 | 0x4C | `sub_800FA8A` | entry 2 | `sub_800B2D8` | GET_FEED_INFO: per-slot steps/length/decoder |
| 77 | 0x4D | `sub_800FAC2` | entry 0 | `sub_800B400` | MOTOR_TEST |

#### Registered by RFID task (`sub_800EA14`)

| CMD | Hex | Thunk | Table Entry | Handler | Function |
|---|---|---|---|---|---|
| 13 | 0x0D | `sub_800FEC8` | entry 24 | `sub_800E910` | GET_RFID_CACHE: returns cached 19+19 byte tag data from `word_20000054[82*slot]` |
| 14 | 0x0E | `sub_800FEE4` | entry 22 | `sub_800E74C` | SET_RFID_ENABLE: enable/disable RFID per slot + motor-side effect |
| 20 | 0x14 | `sub_800FF00` | entry 25 | `sub_800E9EA` | SET_PRINTER_STATUS: stores `byte_20000098`, triggers `byte_20001E60` |
| 68 | 0x44 | `sub_800FEAC` | entry 23 | `sub_800E7A8` | GET_FILAMENT_INFO: live RFID read via `sub_800DEB6`, full decode |

#### Registered by misc/motor task (`0x80159B0` init)

Dynamic SRAM dispatch entries — handler structs built at runtime in SRAM.

| CMD | Hex | Thunk | SRAM entry | Handler | Function |
|---|---|---|---|---|---|
| 15 | 0x0F | `sub_800FD40` | `dword_20000E88` | `sub_800FC98` | LINEAR_KEY_CALIBRATE: ADC threshold → `unk_20001A14`, `word_20000054` |
| 16 | 0x10 | `sub_800FDA6` | static entry 20 | `sub_800DA30` | GET_MATERIAL_INFO: slot material name (21 bytes) + status byte |
| 17 | 0x11 | `sub_800FDDE` | static entry 21 | `sub_800DB44` | SET_SLOT_STATUS: write `byte_200003FF/447/478` per slot |
| 18 | 0x12 | `sub_800FDC2` | static entry 19 | `sub_800DAE2` | SET_MATERIAL_NAME: write 21-byte name, clears status, triggers NVM write |
| 66 | 0x42 | `sub_80101CE` | `dword_20000FBC` | `sub_8010170` | SET_VALVE: `sub_800F0CC(64,…)` + `sub_800F0CC(128,…)` binary actuators |
| 70 | 0x46 | `sub_801025C` | `dword_20000FB0` | `sub_80101EC` | FLASH_LED: stores 6-param pattern, creates "flash led" FreeRTOS timer |
| 71 | 0x47 | `sub_8010154` | `dword_20000FA4` | `sub_8010038` | SET_FAN: speed 0–100% → 5 levels, creates "fan_timer" FreeRTOS timer |
| 72 | 0x48 | `sub_801001A` | `dword_20000FC8` | `sub_8010004` | MOTOR_MOVE: calls `sub_800F278(move_type, param, 0)` |
| 73 | 0x49 | `sub_800FC7C` | `dword_20000E70` | `sub_800FC4C` | GET_SENSOR_STATE: returns 17-bit mask of active `unk_20001A14` channels |
| 78 | 0x4E | `sub_800FD8A` | `dword_20000E7C` | `sub_800FD5C` | GET_MOTOR_STATUS |

---

### Key SRAM Variables

| Address | Name | Purpose |
|---|---|---|
| `0x200011FC` | `dword_200011FC` | Frame parser state (0=idle, 1–8=parsing states) |
| `0x2000111C` | `word_2000111C` | Expected SEQ counter (hi byte) + assigned SEQ (lo byte) |
| `0x200012D4` | `dword_200012D4` | Linked list head pointer (most-recently-registered command) |
| `0x200012DC` | `dword_200012DC` | Linked list tail pointer |
| `0x20001000` | `dword_20001000` | Per-slot state array (`dword_20001000[slot]`) |
| `0x20001014` | `unk_20001014` | Per-slot structures (64 bytes each, 4 slots) |
| `0x20001A14` | `unk_20001A14` | Filament sensor state array (32 bytes × 17 channels) |
| `0x20001BA0` | `dword_20001BA0` | Motor vtable array (16 bytes × slot) |
| `0x20001600` | `dword_20001600` | Motor controller array (slot control structs) |
| `0x20000600` | `dword_20000600` | Dryer state machine state (0=off, 1=start, 2=running, 3=stop) |
| `0x20000604` | `dword_20000604` | Drying duration remaining (seconds) |
| `0x20000608` | `dword_20000608` | Drying start timestamp (tick count) |
| `0x20000010` | `dbl_20000010` | Temperature reading 1 (double-precision FP) |
| `0x20000018` | `dbl_20000018` | Temperature reading 2 |
| `0x20000020` | `dbl_20000020` | Temperature reading 3 |
| `0x20000040` | `dbl_20000040` | Temperature reading 4 |
| `0x20000054` | `word_20000054` | RFID cached data (82 words × 4 slots) |
| `0x2000820C` | `dword_2000820C` | System tick counter |
| `0x20000D98` | `byte_20000D98` | GET_INFO "first call" init flag |
| `0x20001554` | `word_20001554` | OTA in-progress flag (low byte) |
| `0x20000096` | `byte_20000096` | `check_length` — SET_FEED_CHECK pass threshold (encoder units, 1 byte) |
| `0x20000097` | `byte_20000097` | `error_length` — SET_FEED_CHECK evaluation point (commanded units, 1 byte) |

---

### Slot State Machine (filament task)

State values stored in `dword_20001000[slot]`:

| Value | State | Notes |
|---|---|---|
| 0x00 | IDLE / READY | Default resting state |
| 0x03 | ASSISTING | Buffer assist active |
| 0x81 | FEED_ERROR | Encoder measured less than `check_length` at `error_length` checkpoint |
| 0x82 | ROLLBACK_ERROR | Retract motion error |
| 0x83 | ASSIST_ERROR | Hardware error during assist |
| 0x84 | PRELOAD_ERROR | Preload failure |
| 0x85 | STUCK_ERROR | Jam: `cont_assist_time` exceeded or consecutive assist count too high |
| 0x86 | TANGLED_ERROR | Tangle: sustained feed resistance |
| 0x87 | MOTOR_ERROR | Motor driver fault (IRQ or overcurrent) |

Per-slot structures at `unk_20001014 + 64 * slot` (64 bytes each).
Sensor states at `unk_20001A14 + 4 * channel` (byte 1 = active flag).
Motor vtable at `dword_20001BA0 + 16 * slot`.

---

### DISCOVER / ASSIGN Handshake

On first connection the host initiates:

**Step 1 — DISCOVER (CMD 0):**
Host sends CMD 0 with empty payload. ACE responds with its 96-bit STM32 UID:
```
response[0..2] = MEMORY[0x1FFFF7E8], [0x1FFFF7EC], [0x1FFFF7F0]  (3 × uint32)
```

**Step 2 — ASSIGN (CMD 1):**
Host sends CMD 1 with the UID it received plus a desired `seq_byte`:
```
request[0..2] = UID words (must match device UID exactly)
request[3]    = desired initial SEQ counter value
```
ACE validates all three UID words. On match:
- Sets `HIBYTE(word_2000111C)` = current expected seq
- Sets `LOBYTE(word_2000111C)` = host's chosen `seq_byte`
- Returns success (1)

On mismatch: returns `0x100000000` (error).

After ASSIGN succeeds all subsequent frames must use incrementing SEQ values.

---

### OTA / IAP Protocol

The ACE2 supports in-application firmware update over the same UART protocol.
Three-step sequence registered by the IAP upgrade task:

**Step 1 — CMD 2 (`IAP_UPGRADE`):** handler `sub_800D4E4`
```
request: OTA metadata struct
  [0] = firmware image pointer (address)
  [1] = metadata word (size/CRC info)
  [2..12] = version string (11 bytes)
Side effect:
  dword_20001558 = context ptr
  word_20001540 = metadata
  dword_2000153C = image ptr
  byte_20001554 = 1 (OTA active)
```

**Step 2 — CMD 3 (`IAP_FIRMWARE`):** handler `sub_800D564`
Receives firmware chunks. The IAP task flash-writes received data.

**Step 3 — CMD 4 (`IAP_UPGRADE_FINISH`):** handler `sub_800D5C4`
Verifies the written image and triggers reboot into new firmware.

**Version query — CMD 5 (`IAP_VERSION`):** handler `sub_800D528`
Returns 11 bytes from `0x08018000` (version struct in flash ROM constants area).

---

### Dryer State Machine

State variable `dword_20000600`:

| Value | State |
|---|---|
| 0 | OFF (idle) |
| 1 | STARTING (CMD 11 received, valid temp) |
| 2 | RUNNING (PID loop active) |
| 3 | STOPPING (CMD 11 received with temp=0 while running) |

CMD 11 (`DRYING`) handler `sub_800BD1C`:
- `request[0]` = target temperature → validated by `sub_800D8AC`
- `request[1]` = duration in minutes → stored as `60 * request[1]` seconds in `dword_20000604`
- `request[8]` = fan mode byte → `byte_2000061C`
- State transitions (the **only** command that drives the state machine):
  - Valid temp + state==OFF(0) → state = STARTING(1)
  - temp=0 + state==RUNNING(2) → state = STOPPING(3); dryer task then transitions 3→OFF(0)

CMD 12 (`SET_DRY_TEMP`) handler `sub_800BDF6`:
- Calls `sub_800D8AC(request[0])`:
  - Valid temp: `dword_20000674 = temp + 7.0` (PID target with +7°C headroom offset); `word_20000670 = temp` (setpoint for auto-adjust loop)
  - temp=0: `dword_20000674 = NaN` (disables heater); `word_20000670 = 0` (disables auto-adjust)
- **Does not modify `dword_20000600`** — the dryer state machine is unchanged.
  Sending this command with temp=0 stops heating but leaves the dryer in RUNNING(2) state.

CMD 75 (`DRY_CMD`) handler `sub_800BE0A`:
- Sets raw PID target directly: `dword_20000674 = temp` (no +7°C offset)
- Clears auto-adjust: `word_20000670 = 0`
- Sets upper limit: `flt_20000000 = temp + 5.0`
- temp=0: `dword_20000674 = NaN`, `flt_20001CA0 = 0.0`
- Also does not modify `dword_20000600`.

PID heater control runs inside the `dry` task (`sub_800BEC8`). NTC temperature via ADC1.
Motor PWM via TIM7 (PSC=119, ARR≈20000). Encoder interrupt at EXTI0 (PA0) for odometry.

---

### RFID

The RFID task manages 4 slots via byte arrays at `byte_20001A15/25/35/45`.
- `sub_800DD5A` = RFID init
- `sub_80136B4` = RFID read
- Magic values `123` and `456` appear in RFID data validation

GET_FILAMENT_INFO (CMD 68, `sub_800E7A8`):
- Real-time read via `sub_800DEB6`
- Response: 2 × 19-byte tag strings, up to 10 data DWORDs, status/error codes
- Error codes: 0=success, 3=read error, 4=CRC error, 6=tag too short

GET_RFID_CACHE (CMD 13, `sub_800E910`):
- Returns last-read data from SRAM cache `word_20000054[82 * slot]`
- Same response structure as CMD 68, no hardware access

**Host usage:** gklib (`ACEProxyV2.get_filament_info`, `0x62B35C`) sends **CMD 13** (GET_RFID_CACHE), not
CMD 68. The ACE2 MCU continuously updates the cache from background RFID polling; the host simply
reads from it. CMD 68 is a test/debug live trigger only.

**`FilamentInfoResponse` field notes (confirmed from gklib `0x62B35C`):**
- `diameter` (proto field 8): stored as uint32 in **0.01 mm units** (175 = 1.75 mm).
- `remainder` (proto field 11): stored as total remaining length in mm; gklib converts to percentage
  via `100 * remainder / totalLength`.
- Both CMD 13 and CMD 68 share the same req/resp descriptor pair (`0x08019250` / `0x08018FE0`) — the
  decoder is identical regardless of which command is used.

---

## Addendum — RFID reader hardware identification (firmware v1.1.31 teardown)

This section is original analysis from disassembling the v1.1.31 application image
(`ACE2_V1.1.31_20260306.swu` → unpacked `.bin`, ARM Cortex‑M / STM32F1 family, app base
`0x08008000`, 71592 bytes). It corrects a few guesses in the gist above and identifies the
physical RFID front‑end, which is the key fact for "can the ACE2 read more than it reports."

### The reader is an MFRC522‑family chip on hardware SPI

The RFID register access goes through two small leaf functions:

- `sub_0800F574(reg)` — **register read.** Asserts a CS GPIO low (via the port `BSRR`/`BRR`
  bit‑set/bit‑reset registers), pushes one address byte, reads one data byte, releases CS.
  The address byte is formed as `0x80 | (reg << 1)`.
- `sub_0800F5D0(reg, val)` — **register write.** Same CS framing; address byte `(reg << 1) & 0x7E`,
  then the data byte.

That address encoding — bit7 = read/write direction, register number in bits 6..1, bit0 = 0 — is
the **exact MFRC522 / RC522 / FM17522 SPI convention** (NXP MFRC522 datasheet §8.1.2.3). These are
fully capable ISO/IEC 14443A readers. The SPI transfers use the STM32 SPI peripheral directly:
data register at `SPIx_BASE + 0x0C` (`SPI_DR`), status/busy poll on `SPIx_BASE + 0x08` (`SPI_SR`,
TXE/RXNE/BSY bits). CS is a plain GPIO, not the SPI hardware NSS.

### Implication: "limited RFID" is a firmware limitation, not hardware

An MFRC522 performs the full ISO14443A anti‑collision sequence (REQA/WUPA → SELECT cascade) and
therefore **reads the card UID** as a normal part of activation — the firmware cannot talk to a
MIFARE/NTAG tag *without* first obtaining its UID. The teardown shows the UID is acquired during
activation but is simply **not copied into the `FilamentInfoResponse`** that the MCU returns to the
host. There is no missing hardware capability between the current behaviour and exposing the UID (or
raw sector dumps) — only firmware that chooses not to forward those bytes. This is consistent with
what we already proved from the host side: `ACE_RFID_DUMP` shows no UID field anywhere in the
protobuf, even though the chip must have had it.

### Corrections to the gist analysis

- **The "magic constants" `0x45670123` and `0xCDEF89AB` are not RFID values.** They are the STM32
  **FLASH_KEYR unlock key sequence** (`KEY1`/`KEY2`, RM0008 §3.3.3), used by the OTA/firmware‑write
  path to unlock the flash controller — unrelated to the tag protocol.
- **`sub_08010464` is a CRC‑16, not a tag parser.** It is a table/bit CRC‑16 (Kermit/CCITT class)
  used for the V2 frame checksum and for the OTA image integrity check.
- **`sub_080177A4` is an RTOS mutex take/give**, part of the FreeRTOS‑style scheduler glue, not
  RFID logic. It appears in the RFID call path only because the reader is guarded by a lock.

### Firmware‑modification feasibility (honest assessment)

The interesting target is `sub_0800E7A8`, the `GET_FILAMENT_INFO` / cache‑read response builder: to
expose the UID we would inject the already‑captured UID bytes as a new protobuf field there. That is
the *right* place, but a blind binary patch is **high risk and low confidence**, for concrete
reasons, not vague caution:

1. **No symbols, no source.** Every offset is recovered by inference. The UID buffer in SRAM is not
   positively identified — only the activation path that must produce it.
2. **No test loop.** There is no way to validate a patched image short of flashing real hardware;
   the first confirmation is also the moment of brick risk.
3. **Bootloader signature is unknown.** The application image (`0x08008000`+) does **not** contain
   the reset/bootloader region (`0x08000000`–`0x08008000`), so whether the bootloader verifies an
   RSA/ECDSA signature (vs. the CRC‑only check we *can* see in the app) **cannot be determined from
   this image**. If a signature check exists in the bootloader, any modified image is rejected or
   bricks on boot.
4. The CRC over the app image *can* be recomputed (we have `sub_08010464`'s algorithm) and the
   `.swu` repackaged (ZipCrypto, known key from flash history), so the *packaging* is solvable — it
   is the *signature* unknown (#3) that gates the whole thing.

**Recommended path for community verification** rather than a one‑shot brick attempt:
1. Dump the **bootloader** region (`0x08000000`–`0x08008000`) from a live unit over SWD (ST‑Link)
   and confirm whether it checks a signature. This single read settles feasibility.
2. If CRC‑only: build the patch against `sub_0800E7A8`, recompute the app CRC, repackage the `.swu`,
   and keep a full SWD flash dump first so the original can always be restored (un‑bricks #2).
3. Share the bootloader dump + this analysis so others with the same hardware can reproduce before
   anyone flashes a modified image.

The durable result here is the reader identification (MFRC522 → UID is reachable in firmware) and
the precise patch site. The flashing step should wait on the bootloader dump, which converts the
brick risk from "unknown" to "known and recoverable."

---

## SWD wiring & dump procedure (for the hardware teardown)

The ACE2 board exposes a 7-pad debug header next to the MCU, labelled
`RST · CLK · SWDIO · GND · 3V3 · TX · RX`. This is a combined SWD + UART console header.
For an SWD flash dump with an ST-Link, use five of the seven pads:

| Board pad | ST-Link pin | Required? |
|-----------|-------------|-----------|
| `SWDIO`   | SWDIO       | Yes |
| `CLK`     | SWCLK       | Yes (this pad is SWCLK) |
| `GND`     | GND         | Yes |
| `RST`     | NRST/RESET  | Recommended — enables "connect under reset" if the app remaps the SWD pins |
| `3V3`     | VTREF/VAPP (target-voltage sense) | Yes — connect to the ST-Link's voltage-reference pin, not VOUT |
| `TX`/`RX` | —           | No — UART console only, unused for SWD (try 115200 for boot logs later) |

**Power rule:** power the ACE2 from its own supply and wire ST-Link `3V3 → VTREF` for level sensing.
Do **not** drive 3V3 into a board that is already self-powered — pick one source. If the board is
unpowered the ST-Link can supply 3V3, but a board this size may exceed what the ST-Link likes to
source, so external power + VTREF-sense is cleaner.

**Connect under reset:** select this mode in the tool. If the option bytes or the application remap
`SWDIO`/`SWCLK`, a normal attach fails but connect-under-reset still gets in — this is why wiring
`RST` matters.

### ⚠️ Check readout protection (RDP) FIRST — before any read or write

STM32 parts have an RDP level. If RDP is level 1 or 2 the debugger refuses to read flash, and
**attempting to lower RDP triggers a mass-erase** that wipes the firmware. So the very first
operation after connecting must be a **read-only RDP-level check**:

- If `RDP-0` (unprotected): clear to dump everything below.
- If `RDP-1`/`RDP-2`: stop. Do not change it — that erases the chip. Note it and reassess.

### Dump order once RDP-0 is confirmed

1. **Full flash dump first** — this is the guaranteed un-brick restore image. Keep it safe before
   touching anything.
2. **Bootloader region `0x08000000`–`0x08008000`** — the go/no-go read: settles whether a modified
   app is signature-checked (brick) or CRC-only (patchable).
3. **RAM snapshot immediately after an RFID read** — reveals the UID buffer address that the
   `FilamentInfoResponse` patch (`sub_0800E7A8`) needs.

Exact tool commands (open-source `st-info`/`st-flash`, STM32CubeProgrammer CLI, or OpenOCD) to be
filled in once the toolchain is chosen at dump time.
