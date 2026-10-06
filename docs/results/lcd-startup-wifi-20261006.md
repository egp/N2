# LCD start-up and bus robustness, UNO R4 WiFi, 2026-10-06 (afternoon)

Hardware: UNO R4 WiFi, LCD 20x4 + PCF8574 backpack (0x27), DS3231 RTC (0x68), matrix, shared I2C bus with pull-ups. Firmware: `stages/stage1`.

## Problem
After a **reset** (not a power-up) the LCD often showed random characters, sometimes ending blank, sometimes the I2C bus stopped working until the next reset,
although POST said "LCD ok" and the driver counted 0 errors. Reset by button leaves the LCD and RTC powered (they keep their state): the worst case.

## What was ruled out
- The I2C link itself: `lcd bus 500` (write + read back through the PCF8574, EN low) gave 0 failures in 1500 rounds.
- The RTC (removed: same failures), the matrix (off: roughly the same), the bus speed (the "30 kHz" runs were really 100 kHz: only 100 and 400 kHz exist on the R4).
- Periodic re-initialisation made it **worse** (0 clean in 6): re-initialising a running display left it garbled or blank.

## What fixed it (all together)
1. The LCD is not touched for **2.5 s** after boot (`kDefaultLcdStartMs`).
2. **No** re-initialisation of a running LCD (opt-in only).
3. Full rewrites of the screen: early (250 ms, doubling to 5 s) in the driver, and continuous in the Stage 1 test build.
4. **I2C bus recovery** (`Hal::i2cRecover`): at boot, on every LCD retry, and when the RTC is silent. Found when a run ended with the controller wedged (scan found nothing, firmware alive).

## Results with the final build
- 6 button resets: **6/6 valid** (PPPPPP). The log shows `LCD ready` 2.55 s after each POST, 0 errors, 0 recoveries.
- 3 power cycles: all valid.
- Wire unplug tests (SDA out 5 s, SCL out 5 s): the LCD showed errors, the driver recovered the bus once a second, and the LCD returned about 4 s later with no reset
  (log: `LCD I2C error`, `I2C bus recovery #n`, `LCD ready`). Pull-up removal and both-wires-out repeated separately.
- Soak: 10 minutes, no resets, 0 dropped log lines, loop mean 38 us.
