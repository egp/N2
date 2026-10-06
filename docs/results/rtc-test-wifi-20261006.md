# RTC (DS3231) driver test results — UNO R4 WiFi — 2026-10-06

Sketch: `experiments/rtc_test` version 1.1. Driver under test: the firmware's `Rtc3231` with `HalArduino` (including the new register-read
operation), compiled from the firmware tree. Module: DS3231 board on A4/A5; the console log was captured with the Mac's clock stamped on every line.

## Verdict: PASS

| Check | Result |
|---|---|
| I2C scan | **0x68** (DS3231) and **0x57** (the module's EEPROM) on the A4/A5 bus; nothing on the Qwiic bus (Wire1) |
| First read, nothing changed | `2000-06-01 00:36:04`, ticking exactly 1 s per second; oscillator-stop flag SET (a never-set module) |
| Drift vs `millis()` over 5 s | RTC 5 s, millis() 5 s: difference 0 |
| Temperature | 25.50 C (plausible) |
| Speed and reliability | 200 reads in a row, **0 failures**, **962 microseconds per `read()`** |
| Set from the Mac's clock (`T 2026-10-06 10:31:25`) | read back immediately as `10:31:25`; kept 1 s per second afterwards (a set lands to the nearest second, so the RTC can run up to ~1 s behind real time) |
| Battery backup: Arduino unplugged (~tens of seconds), replugged | after replug the RTC read `2026-10-06 10:34:58` while the Mac's clock stamped `10:34:58`; **oscillator-stop flag CLEAR** (the clock never stopped) |

Conclusion: the DS3231 driver, the register-read HAL operation, the BCD/calendar handling, and the module's battery backup all work on real hardware.
