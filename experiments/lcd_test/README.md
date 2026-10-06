# lcd_test — validate the firmware's LCD driver on the UNO R4 WiFi

Tests the real `Lcd20x4` driver (20x4 HD44780 LCD with a PCF8574 I2C backpack) and the real `HalArduino`, linked from the firmware
tree through the `src` symbolic link in this folder. Only the LCD is needed.

## POWER-UP CHECKLIST — do this BEFORE connecting power (a previous module was damaged)
1. **Arduino alone first:** USB cable and nothing else. It must appear as a serial port and stay cool.
2. **Read the labels on the backpack** (they differ between makers, and the pin ORDER differs too): `GND`, `VCC` (5V), `SDA`, `SCL`.
3. **Wire with the USB cable UNPLUGGED:**

   | LCD backpack | UNO R4 WiFi |
   |---|---|
   | GND | GND |
   | VCC | 5V |
   | SDA | A4 |
   | SCL | A5 |
4. **Check before power:** GND goes to GND (not to 5V), VCC to 5V (not to GND, not to 3.3V), no stray strands touching.
   With a multimeter on resistance, the LCD's VCC-to-GND should read more than a few hundred ohms (not near 0), and the Arduino's
   5V-to-GND likewise.
5. **Power up and feel for heat within 10 seconds.** Anything warm on the backpack, or a hot or smelly part: unplug USB at once.
6. The backlight draws about 100-200 mA; a USB port can supply that.
7. If the display shows only dark blocks in the top row or nothing at all, turn the small **contrast trimmer** on the backpack.

Expected I2C address: **0x27** (some backpacks use 0x3F; step 0 and the `s` command show what answers).

## Run
Open `experiments/lcd_test/lcd_test.ino` in the Arduino IDE (board *Arduino UNO R4 WiFi*), upload, Serial Monitor at 115200
(or let Claude drive the console). Steps 0-9 run once; the matrix shows the step in hex (left) and a result (right). See the table at
the top of `lcd_test.ino` for what each step should show, and write down what you actually see.
