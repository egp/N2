# I2C speed sweep, UNO R4 WiFi, 2026-10-06

Firmware: `stages/stage1` (`i2c sweep`), LCD backpack (PCF8574, 0x27), DS3231 RTC (0x68, module EEPROM 0x57) on one bus with pull-ups, matrix running.

## Finding 1: the R4 hardware I2C has only two real speeds
`Wire.setClock(freq)` in the renesas_uno 1.6.0 core copies `freq` into an enum and handles only `I2C_MASTER_RATE_STANDARD` (100 kHz) and
`I2C_MASTER_RATE_FAST` (400 kHz); `..._FASTPLUS` (1 MHz) exists but behaved like 400 kHz on the bench. **Any other value (10, 30, 50, 200 kHz...) is silently
ignored and the clock stays at its previous setting.** Measured RTC read time: 1014 us at "10/30/50/100/200 kHz" (all really 100 kHz), 262 us at 400 kHz and at "1 MHz".
Consequence: the earlier "30 kHz" experiments were really 100 kHz. A true 30 kHz bus would need a bit-banged driver (as V5 had).

## Finding 2: the bus is clean at both real speeds
`i2c sweep 100` and `i2c sweep 200` (LCD read-back rounds, 40 RTC reads checked for consistency, 20 address probes):

| Speed | LCD bad | RTC bad | Probe bad | RTC read |
|---|---|---|---|---|
| 100 kHz | 0 | 0 / 40 | 0 / 20 | 1014 us |
| 400 kHz | 0 | 0 / 40 | 0 / 20 | 262 us |

400 kHz works even though the PCF8574 backpack is only rated to 100 kHz. The firmware stays at 100 kHz (`kI2cClockHz` in Config.h, with a compile-time check that it is 100000 or 400000).
