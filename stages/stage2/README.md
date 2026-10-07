# Stage 2: stage 1 plus the 4-digit LED

Adds the TM1650 LED display to stage 1 (`../stage1`). Both devices passed their solo tests on the bench first
(`docs/results/lcd-test-wifi-20261006.md`, `docs/results/led-test-wifi-20261007.md`).

## Before you power it (read every time)
1. **USB unplugged** while you wire. Wire the LED module as in its solo test (check its supply voltage first; the first module burned).
2. LCD backpack, RTC and LED module all share SDA/SCL (A4/A5), 5 V and GND.
3. **Address overlap to watch:** the LED module also answers at 0x27, the LCD backpack's address. If the LCD misbehaves with the LED fitted, move the LCD
   backpack to 0x20-0x23 with its address solder jumpers and set `addrLcd` in `BoardPins.h`.

## Run
Arduino IDE: open `stages/stage2/stage2.ino`, board *UNO R4 WiFi* or *Minima*, Upload, Serial Monitor at **115200**, type `help`.
`scan` should list 0x24-0x27, 0x34-0x37 (the LED), 0x57 and 0x68 (the RTC module), and 0x27 (the LCD).
`bist` steps: reset (p/f), LCD (p/f), RTC (decides itself), LED (p/f: 8888 with one dot, count 0-9, one blink).
Copy the Serial Monitor text into `docs/results/stage2-<board>-<date>.md`.
