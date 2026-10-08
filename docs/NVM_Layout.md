# Non-volatile memory (NVM) — layout, rules, and what else could live there

Status 2026-10-08: record format, A/B store, debouncer and bounce meter are written and host-tested (`test_nvm.cpp`).
The real-flash behaviour is learned with `experiments/nvm_probe` (not yet run on hardware).

## What the core gives us (read from renesas_uno 1.6.0 source)
* The UNO R4 has 8 KB of **data flash** behind the standard `EEPROM` library (`EEPROM.length()` reports it).
* Flash is erased and programmed a **whole 1 KB block at a time**. `EEPROM.write()` of a byte that is not 0xFF in the block
  copies the block to RAM, erases it and programs it again. **One changed byte = one block erase.** Rated about 100,000 erases per block.
* The CPU waits during the erase (milliseconds). So: **never save from the control loop**; console / BIST only.
* Blank flash reads 0xFF. A board that ran another program may hold anything: contents are UNKNOWN until we write them.

## Address rules
| Block | Address | Use |
|---|---|---|
| 0 | 0x0000 | settings copy A (16-byte record; the rest of the block is reserved) |
| 1 | 0x0400 | settings copy B (same record) |
| 2.. | 0x0800.. | not used by this program (left alone; reserved for future records, see below) |

Two copies in two blocks: a power failure while one is being written damages at most that copy. The newer valid copy (higher sequence
number, wrap-safe) is used; each save goes to the older one, then is read back and compared. Identical settings are never rewritten.
Several values share ONE record so one save = one erase. A future record kind gets its own block (so its erases do not age the settings).

## Record (22 bytes, little-endian, schema 2)
`4E 32` magic · schema (2) · payload length (8) · **write count** (u32) · TBS ms · TOB ms · board id (1 Minima, 2 WiFi) · reserved · **sketch version** (u16, e.g. 0x0801) · reserved (u16) · **saved-at** (u32: RTC time of the save, seconds since 2026-01-01 00:00:00, 0 = unknown) · CRC-16/CCITT over the first 20 bytes.
The write count is the number of saves (each = one block erase of A or B) and is carried forward on every save; each block is erased about half that many times; rated 100,000 per block, so a warning is logged at 50,000 (`kNvmWearWarnWrites`).
The `nvm` console command (and the probe's `l`) shows every check for each copy separately — **a never-set region fails the magic and the checksum** (blank flash also shows `blank`).
Check bits: a CRC-16 instead of one parity bit — parity passes half of all garbage, the magic+schema+length+CRC pass about 1 in 2^40 (tested on 20,000 random records, and every single-bit flip of a good record is rejected).
Blank (all 0xFF), zeroed, and foreign contents are each reported distinctly. **Invalid or missing = the compiled default** (`kBringupDebounceMs` 30 ms). A stored value is used only if its board id matches this board and both times are 2..100 ms.

## Debounce
`Debouncer` (accepts a level after it holds for debounceMs; unsigned millis math) and `BounceMeter` (every pass: raw level + `micros()`; an *operation*
= first edge until 200 ms of quiet; records edges per operation and first-to-last-edge settle time). Recommended debounce = 2 × the longest settle time, rounded up to ms, limited to 2..100.
RESET cannot be measured (it is the hardware reset line). TBS is a rotary maintained switch and TOB a momentary pushbutton, so each is measured separately, press and release.
Measure on the R4 WiFi bench and again on the Minima production panel; each board stores its own values; afterwards the compiled default is changed to the larger measured recommendation.

## What else might belong in NVM (proposals — none implemented; each would be its own block)
1. **Reset/boot counters and last reset cause** (power-on, watchdog, brown-out) — field evidence of brown-outs and watchdog trips. Writes: once per boot at most.
2. **Last N fault codes with RTC time** (a small ring in its own block, written on fault *assert* only) — what happened while nobody watched. Needs a rate limit (wear).
3. **O2 warm-up credit** (NFR-9, O2-6a): "sensor was powered N minutes at time T" so a quick reset does not force the 5-minute wait again. Written rarely (at most every few minutes while running).
4. **Tuning overrides** (FUT-1): pressure thresholds and timing values changeable from the console without recompiling, each with min/max limits, compiled default when absent.
5. **Pressure-sensor calibration** (offset/gain per transducer) once Stage 3 shows the real sensors.
6. **Commissioning data**: unit serial number, firmware version at last save, date/time of the last debounce characterization.
7. **Per-board options** that differ between the bench and production (LCD address, pin choices) — better left in BoardPins.h unless a site change must not need a recompile.
Not for NVM: anything that changes every loop, run-hour counters updated continuously (use RAM + an hourly write, or an FRAM later), and anything safety-critical that must not depend on a stored value (limits stay compiled).

## Tuning overrides and sketch versions (owner, 2026-10-08)
Stored tuning values override the sketch's compiled values at power-up. When a NEW sketch is delivered its compiled values may have changed on purpose, so the
settings record must carry the **sketch version** that wrote it (schema 2: add a version field, and per-value "source" so we can tell an operator override from a
measured value). At boot: stored version == running version -> use the stored values; different -> do NOT silently apply or discard: log it, show it on the LCD, and
use the sketch's values until the operator confirms (console command: keep stored / take sketch). Rule to decide: measured per-board values (debounce) are kept across
versions unless the new sketch declares them obsolete; operator tuning overrides are asked about. Not implemented yet.

## Debounce test (nvm_probe 1.1)
20 on/off cycles per switch (command c N: 5..30); LCD shows cycle n of N and min / median / mean / max settle time in raw microseconds plus edges per operation.
No rounding is applied or stored: the owner decides the debounce times after seeing the results (command w TBS TOB).

## In the firmware (2026-10-08)
* App::setup() calls `NvmSettingsService::load()` (read only) and gives System the debounce times: stored if valid for this board and 2..100 ms, else the compiled default (`kBringupDebounceMs` 30 ms). The choice and its reason are logged at boot; a record written by another sketch version is used but reported.
* System debounces TBS and TOB (INP-6). Note: TBS OFF now takes effect one debounce time late (30 ms by default).
* Console: `nvm` (what is stored, each check, write count), `debounce` (active values), `debounce set TBS TOB` (saves; refused while TBS is ON; ~50 ms stall).
* Not yet done: the BIST step for bounce characterization (use experiments/nvm_probe for now), erase command, tuning overrides, the sketch-version confirmation dialogue.
