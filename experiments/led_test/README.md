# led_test — validate the firmware's LED driver on the UNO R4 WiFi

Tests the real `Led1650` driver (TM1650 4-digit, 8-segment LED display, I2C) and the real `HalArduino`, linked from the firmware
tree through the `src` symbolic link in this folder. Only the LED module is needed.

## Before you power it (read every time: the first TM1650 module burned)
1. **USB unplugged** while you wire. Remove the LCD and RTC from the SDA/SCL wires for this solo test, so the LED module is the only thing on the bus.
2. **Read the module's silkscreen / seller page for its supply voltage.** Many TM1650 boards are 5 V logic; some are 3.3 V only. If it says 3.3 V, power it from the Arduino's 3.3 V pin instead of 5 V.
3. **Meter first, module not connected to the Arduino:** with the Arduino powered and nothing on the breadboard but the supply rails, confirm the rail you will use reads the right voltage (5 V or 3.3 V).
4. **Check the module for damage and shorts** before connecting: with the meter on continuity (Arduino unpowered), VCC to GND must NOT beep. Look for a hot, bulging or discoloured part.
5. **Connect GND first, then SDA and SCL, VCC last.** Power up and put a finger near (not on) the module for 10 s: nothing should warm up. Unplug USB at once if anything gets warm or smells.
6. Only then upload the sketch.

## Wiring (bare board + the LED module)
| LED module | UNO R4 WiFi |
|---|---|
| VCC | 5V |
| GND | GND |
| SDA | A4 (the SDA pin) |
| SCL | A5 (the SCL pin) |

Expected I2C addresses (TM1650): **0x24** control, **0x34–0x37** the four digits. Step 0 probes all five.

## Run
Open `experiments/led_test/led_test.ino` in the Arduino IDE (board *Arduino UNO R4 WiFi*), upload, and open the Serial Monitor at
115200 (or let Claude drive the console). The sketch runs steps 0–A once; the matrix shows the step number in hex (left) and a
result in hex (right). Type `g` to run it again, `n` for the next step, `p` to pause.

See the table at the top of `led_test.ino` for what each step should show. Write down, per step, what you actually saw on the display.
Step A is interactive: unplug the SDA wire when told, watch the driver report the failure, and plug it back to see it recover.
