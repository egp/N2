# Bring-up stages

Rather than debugging the whole N2V8 at once, start from what is proven on the bench and add one device at a time.

**The rule for adding a device**
1. Solo test sketch in `experiments/` (driver only), run on the bench, results in `docs/results/`.
2. A `DeviceCheck` class in `src/selftest/` holding the device's **POST check and BIST step** (`DeviceCheck.h`), host-tested with the fake Hal.
3. Add it to the list in `src/app/Bringup.cpp` and re-run the stage on the bench.
4. When every device is in, the stage list *is* the final POST/BIST and the controllers are switched on last.

| Stage | Devices | Status |
|---|---|---|
| 1 | reset cause, LCD, RTC, console, WiFi matrix | built; host tests pass; bench run pending (I2C bus being wired) |
| 2 | TM1650 LED | waiting for the replacement module (solo test first) |
| 3 | TBS/TOB switches, pressure inputs | |
| 4 | O2 sensor | |
| 5 | valves and SSR outputs (BIST with vetoes), then controllers | |

**RTC policy (RTC-7, RTC-8):** the RTC only stamps console log lines. It is read now and then to anchor a software wall clock
(no I2C per log line), never used for scheduling, and written only when untrusted or more than 2 s from a reference time
(`time set`). If it is absent the firmware carries on; log lines simply have no time stamp.

**What is separate from what:** Console (PC) · Lcd20x4 · Rtc3231 + WallClock · DeviceChecks + SelfTest (POST/BIST) ·
StageCommands · FrameSink (matrix) · Bringup (wires them together; no logic of its own).

## Stage 1 bench log: LCD start-up garbage (2026-10-06)
After a **reset** (not a power-up) the LCD often shows random characters ("garbage", random cells), sometimes ending blank, even though
POST reports the LCD OK and the driver counts **0 I2C errors**. Findings so far:
- `lcd bus 500` (write a bit pattern to the PCF8574, read it back, EN/RS/RW held low): 1500 rounds, **0 failures, 0 mismatches** at 100 kHz
  with the matrix running. The I2C link to the backpack is intact: the fault is on the LCD side (controller state or power).
- Not the RTC (removed: same failures). Roughly the same with the matrix off (4 of 5 clean, small sample) and with a 30 kHz bus.
- A second and third full re-initialisation (0.3 s, 1.2 s) and early full rewrites did not cure it.
- Next experiments: delay the first LCD touch until 2.5 s after boot (`-DSTAGE1_LCD_START_MS=2500`), a 100 nF capacitor across the backpack's
  VCC/GND, and a longer power-up wait before the first nibble.
Debug switches: `-DSTAGE1_NO_MATRIX`, `-DN2_I2C_HZ=30000`, `-DSTAGE1_LCD_START_MS=2500`; console `lcd`, `lcd reinit`, `lcd bus [n]`.

**Resolved (2026-10-06 afternoon):** leaving the LCD untouched for the first 2.5 s after boot (`kDefaultLcdStartMs`, in `DisplayManager` and
`Bringup`) gave **5 clean resets in 5** (RTC absent, matrix on, 100 kHz). Earlier builds that initialised the LCD within ~0.1 s of boot failed
roughly half the time. The cause is start-up timing right after a reset, not the bus (the `lcd bus` test is clean), the RTC or the matrix.
The extra re-initialisations and early rewrites are kept as options (`lcdReinit1Ms`, `lcdReinit2Ms`, off by default). To re-test with the RTC
fitted: reset six times, wait 10 s each.

**Self-healing LCD (2026-10-06, afternoon):** with the RTC fitted, one reset in six ended with the LCD garbled then blank and *never recovered*, even
though the 2.5 s start delay had given 5 clean in 5 without the RTC. The HD44780 cannot be read back through this backpack, so the driver
now repairs the display on a schedule (`LcdHealing` in `Lcd20x4.h`, on in both Stage 1 and the full application): full rewrites 250 ms after
it comes up, doubling to every 5 s; full re-initialisations at 0.5 s and 2 s, then every 30 s. A bad display is therefore bad for at most about 30 s.
