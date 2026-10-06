# LCD driver test results — UNO R4 WiFi — 2026-10-06

Sketch: `experiments/lcd_test` version 1.0. Driver under test: the firmware's `Lcd20x4` (20x4 HD44780 behind a PCF8574 backpack at 0x27)
with `HalArduino`, compiled from the firmware tree. The author watched the LCD; the console log was captured over serial.

## Verdict: PASS (author's visual check of all 10 steps + driver log)

| Step | What | I2C writes | Driver |
|---|---|---|---|
| 0 | probe 0x27; scan of the A4/A5 bus finds only 0x27 | 0 | healthy |
| 1 | backlight + banner text | 57 | healthy |
| 2 | 80 cells of '#' | 84 (80 characters + 4 cursor commands) | healthy |
| 3 | four different rows (row addressing) | 84 | healthy |
| 4 | printable characters | 84 | healthy |
| 5 | one-character change | **2** (cursor + character), as designed | healthy |
| 6 | backlight off/on | 32 | healthy |
| 7 | display off/on, text kept | 32 | healthy |
| 8 | firmware normal screen (layout 1) | 74 | healthy |
| 9 | firmware O2 warm-up screen | 18 (only the changed cells) | healthy |

Totals: 559 I2C writes, **0 failed**, `i2cErrors() = 0`. A second pass after the reset button gave identical counts.

The matrix back channel worked: step number in hex on the left; on the right the LOW hex digit of the step's write count (step 9: 18 = 0x12, shown as `2`).

## Not yet covered
- The failure/recovery behavior of the LCD driver on real hardware (unplug SDA while running). The LED test had a step for this; the LCD test does not.
- Per-call timing of `service()` (bounded to 6 characters per pass in the design; not yet measured).
- The LED driver: the LED module was damaged (burnt resistor, hot board) before its test could run; set aside until a replacement arrives.
