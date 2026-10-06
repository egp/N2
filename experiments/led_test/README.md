# led_test — validate the firmware's LED driver on the UNO R4 WiFi

Tests the real `Led1650` driver (TM1650 4-digit, 8-segment LED display, I2C) and the real `HalArduino`, linked from the firmware
tree through the `src` symbolic link in this folder. Only the LED module is needed.

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
