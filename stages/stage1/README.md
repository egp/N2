# Stage 1 — reset cause, LCD, RTC, console, matrix

First step of the staged bring-up (`docs/Bringup_Stages.md`). Everything here has already passed its solo bench test.

## Before you power it (read every time)
1. **Power off / USB unplugged** while wiring. Check each module's supply with a meter before the first power-up: LCD backpack **5 V**, DS3231 module 3.3 V or 5 V (check its silkscreen), nothing hot after 10 s. The burnt TM1650 taught us to look first.
2. **I2C bus**: Arduino SDA/SCL (A4/A5, or the SDA/SCL header pins) to *every* module's SDA/SCL, plus common GND. Modules usually already carry pull-ups (about 4.7 kΩ each, in parallel). Add external pull-ups only if the bus is unreliable; keep the combined value above about 1.5 kΩ, pulled to the same rail as the modules.
3. DS3231: fit an LIR2032 or a CR2032 **only if the charging resistor is removed** (RTC-6).
4. Addresses: LCD 0x27, RTC 0x68 (module EEPROM 0x57 is normal). No two devices may share one.

## Run
Arduino IDE: open `stages/stage1/stage1.ino`, board *UNO R4 WiFi* (or *Minima*), Upload, open the Serial Monitor at **115200**, type `help`.
POST runs by itself. It holds **only** if a check fails (LCD missing); press TOB or type `go` to continue. An absent or unset RTC is only a note.

## Try
| type | expect |
|---|---|
| `status` | reset cause, POST results, whether the log clock is synced |
| `scan` | 0x27, 0x57, 0x68 |
| `time` | the RTC date/time and whether it is trusted |
| `time set 2026-10-06 10:31:00` | sets the RTC **only** if it is untrusted or more than 2 s off |
| `bist` | steps: reset (answer p), LCD (answer p/f), RTC (decides itself) |
| unplug the RTC, press RESET | boots normally, `RTC info absent`, log lines without time stamps |
| `loop` | loop() time in microseconds (should be tiny) |

Copy the Serial Monitor text into `docs/results/stage1-<board>-<date>.md`.
