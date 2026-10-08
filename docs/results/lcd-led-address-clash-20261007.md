# The TM1650 LED module and the PCF8574 LCD backpack cannot share an I2C bus at 0x27

Bench, UNO R4 WiFi, 2026-10-07. Replacement TM1650 module (the first one burned) and the usual 20x4 LCD with PCF8574 backpack (0x27).

## What happened
With LCD, RTC and LED all on the shared A4/A5 bus the LCD never worked: init steps acknowledged, then the first character write failed, over and over; the LED showed all
four decimal points and failed too. Without the LED module the same LCD runs cleanly (6 of 6 resets, three more on 2026-10-07 after the LED was unplugged).
Removing the external pull-ups changed nothing. Neither POST nor `scan` could tell the LCD was missing: with the LCD unplugged, the LED module answers at 0x27 and the probe passes.

## Measurements
* **The TM1650 answers 0x24, 0x25, 0x26, 0x27 and 0x34-0x37.** It ignores the low address bits of its control range, so it also answers at 0x27, the LCD backpack's address.
* A TM1650 takes **exactly one data byte** per transaction and refuses the rest (`alias_probe`, LED alone: 1 byte accepted, 2 to 6 bytes refused, even at its own 0x24).
* With the LCD backpack also present, the backpack's acknowledge covers the refusal for **even** bytes only. Sweeping a second byte X after a safe first byte (0x11 or 0x19):
  every odd X fails (128 of 256); with three bytes only multiples of 4 pass; with four bytes only multiples of 8 pass.
  An LCD character write has RS (bit 0) = 1, so its bytes are odd: **every character write fails**, whatever first byte is chosen. Commands (RS = 0) pass, which is why initialisation succeeded.
* Whatever reaches 0x27 also reaches the TM1650's control register (brightness, 7/8-segment mode, on/off): hence the four decimal points.
* A read from 0x27 returns the two devices' answers wired together (0x0 with both present, 0x2E with the LED alone): reads at that address are meaningless.
* The R4 core: after a refused write the next transactions keep failing until the bus is recovered (SCL clocked up to nine times, a STOP, `Wire.end/begin` **and `Wire.setClock`**: only `setClock` reopens the controller).

## Conclusion
"Clever data" cannot make both devices work on one bus at 0x27. The LED and LCD need different buses or the LCD needs a different address.

## Options
1. **Bridge the LCD backpack's A2 solder pad: the LCD moves to 0x23**, outside the LED's range. No driver change. Do it on Tom's identical backpack too, and change `kLcdAddress` in `BoardPins.h` to 0x23 (one line, or `-DN2_LCD_ADDRESS=0x23` for a test build). (Owner's preference, pending Tom's agreement.)
2. Give the LED its own two wires on spare pins with a software I2C driver: `SoftI2c`, written and host-tested (waveform model), selectable per board by `BoardDef::ledSdaPin/ledSclPin`. Only two wires move (A4/A5 to D2/D3 on the LED module). Not used by default. D2 and D3 are unused by V6/V7; the physical panel is unconfirmed.
3. A non-I2C LED module (TM1637 on two GPIO pins): a production hardware change.

## What the firmware does now
* `scan` labels 0x24-0x27 and 0x34-0x37 as the TM1650, and `status` warns "WARNING: LCD address 0x27 is inside the LED module's range" while they collide.
* Stage 2 can be built without the LCD (`-DSTAGE2_NO_LCD`) so nothing is ever sent to 0x27 (LED + RTC verified: LED healthy, 0 errors, shows the clock).

## Result of the A2 bridge (2026-10-08, bench, UNO R4 WiFi)
* LCD backpack with **A2 bridged = address 0x23**, nothing else on the bus: `scan` = 0x23 only; Stage 2 built with `-DN2_LCD_ADDRESS=0x23`: LCD ok, 0 I2C errors, 3 of 3 resets
  clean.
* **LCD at 0x23 + LED module (0x24-0x27, 0x34-0x37) on the same hardware bus**: LCD 0 errors, 0 bus recoveries; LED healthy, 0 errors; `i2c sweep` 0 bad at 100 and 400 kHz for the LCD
  read-back; 3 of 3 resets clean (log: `POST ... LED ok`, `LED ready`, `LCD ready` 2.5 s later); both displays show the right content.
  This is the combination that failed every character write at 0x27.
* Still to do: the RTC back on the same bus (all four devices), then the O2 sensor (0x74, no overlap).
* Tom's identical LCD backpack needs the same A2 bridge before `kLcdAddress` can default to 0x23 for everyone; the default stays 0x27 until he agrees.
