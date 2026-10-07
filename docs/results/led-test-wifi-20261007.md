# LED driver (TM1650) solo test, UNO R4 WiFi, 2026-10-07

Replacement TM1650 4-digit module (the first one burned on 2026-10-06). `experiments/led_test` v1.3: after every step the matrix holds the step and
result glyphs and the sketch waits for the owner's one-line answer. LCD and RTC were disconnected; only the LED module on the bus.

| Step | Expected | Owner saw | Result |
|---|---|---|---|
| 0 | I2C probe, display blank | matrix 05, blank | pass: 5 of 5 addresses answer |
| 1 | 8888, one decimal point after digit 2 | 88.88 | pass |
| 2 | count 0000 ... 9999 | 0000,1111,...,9999 (second run, watched) | pass |
| 3 | ABCD then EF-- | ABCD then EF-- | pass |
| 4 | 8888 with moving decimal point | 8888 with moving dot | pass |
| 5 | ---- | ---- | pass |
| 6 | 1234, then 1239 by ONE write | 1234 then 1239 | pass: 1 write for the change |
| 7 | display off 1 s, back on | blank ~1 s then 1234 again (second run) | pass |
| 8 | blank | blank | pass |
| 9 | steady 1234 | 1234 | pass |
| A | pull SDA, replug | driver failed 1 s after the wire was pulled; healthy again ~7 s after the replug; display showed 9999 | pass |

Totals: 97 I2C writes, 1 failed (the deliberate one).

## Address finding
The module answers at **0x24, 0x25, 0x26, 0x27 and 0x34-0x37** (it ignores low address bits), so it also acknowledges **0x27, the LCD backpack's address**.
BoardPins' duplicate-address check only knew 0x24 and 0x34-0x37. Open question: does the shared 0x27 disturb the LCD when both are on the bus?
If yes: move the LCD backpack to 0x20-0x23 with its address solder jumpers.
