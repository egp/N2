# Fault list

Every fault the firmware can raise. **This file is checked by a host test** (`test_faults.cpp`): each code and text below must match the table in
`N2V8/src/core/Faults.cpp`, so the list cannot drift from the code. Where it is shown: the LCD fault screen (code, text, effect), the
console `faults` command, and the log line `FAULT Fnn raised: <text>`.

Codes are **hexadecimal** (two digits, shown `Fxx`); the high digit is the group. One byte allows 256 codes; the unit needs about a dozen.

Severity: **INHIBIT** = the firmware switches outputs off (see the invariants); **WARN** = shown and logged, control continues;
**INFO** = logged only. A fault clears after its condition has stayed false for the hold time (`faultHoldMs`), unless it is *latching*.

| Code | Text (as shown) | Severity | Latching | What the system does | Raised when | What to check on site |
|---|---|---|---|---|---|---|
| F01 | AIR SENSOR RANGE | INHIBIT | no | TOWERS HELD OFF | The air-pressure input is outside its valid voltage window for `sensorFaultSamples` samples in a row AND for at least `sensorFaultMs` (50 ms) | Air sensor wiring and 5 V supply, connector, the A0 input; compare with the production gauge |
| F02 | N2L SENSOR RANGE | INHIBIT | no | SSR HELD OFF | The N2-low input is outside its valid window (same rule) | N2-low sensor wiring, the A3 input |
| F03 | N2H SENSOR RANGE | INHIBIT | no | TOWERS+SSR OFF | The N2-high input is outside its valid window (same rule) | N2-high sensor wiring; the pin is provisional (A1): confirm in `BoardPins.h` |
| F04 | N2L ABOVE N2H | INHIBIT | no | TOWERS+SSR OFF | The N2-low reading exceeds N2-high by more than the margin for the hold time, with both sensors valid | The two N2 sensors swapped, or one miscalibrated or stuck; gauge check |
| F10 | LCD NO ACK | INFO | no | LOG ONLY | The LCD backpack (I2C 0x23, A2 bridged) does not acknowledge | LCD power and I2C wires; `scan`; the LCD is re-initialised automatically when it answers again |
| F11 | LED NO ACK | INFO | no | LOG ONLY | The 4-digit LED module (I2C 0x24) does not acknowledge | LED module power (5 V!) and I2C wires; `scan` |
| F12 | O2 SENSOR FAILED | INHIBIT | no | ALL OUTPUTS OFF | The O2 sensor is missing, stops answering, or reads exactly 0.00 % (O2-3a). In a FIELD build this switches **all outputs off** (INV-9) | O2 sensor power and I2C wires (address 0x74), DIP switch; it re-warms 5 min when it returns |
| F13 | RTC UNAVAILABLE | INFO | no | LOG ONLY | The real-time clock is missing, unreadable, or lost power (time not trusted) | RTC module and battery; `time`, `time set ...`. Never affects control; log lines simply have no date/time |
| F20 | INVARIANT BROKEN | INHIBIT | **yes** | FORCED SAFE STATE | A controller asked for an output that breaks a safety rule (INV-1..INV-4); the firmware forced everything off | A firmware bug or corrupted state: save the console log, reset. Reproduce on the host if possible |
| F30 | WATCHDOG RESET | WARN | no | RESET WAS LOGGED | The last reset was caused by the watchdog (a loop pass took more than about 4 s) | Console log before the reset; `loop` timing statistics; I2C devices that hang the bus |
| F31 | BROWN-OUT RESET | WARN | no | RESET WAS LOGGED | The last reset was caused by the supply voltage dipping | Supply quality, loose power connector, relays/compressor sharing the supply |
| F40 | CONSOLE DROPPED | INFO | no | LOG ONLY | Log lines were dropped because the serial port buffer was full (the PC was not reading) | Nothing is wrong with the machine; close and reopen the Serial Monitor |

Fault codes group by area: F0x sensors, F1x devices, F2x safety, F3x resets, F4x console. A new fault is one row in `Faults.cpp`, one row here, and one test.
