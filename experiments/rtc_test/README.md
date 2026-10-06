# rtc_test — validate the firmware's DS3231 real-time-clock driver on the UNO R4 WiFi

Tests the real `Rtc3231` driver and the real `HalArduino` (including the new register-read operation), linked from the firmware
tree through the `src` symbolic link in this folder. Only the RTC module is needed.

## POWER-UP CHECKLIST — do this BEFORE connecting power
1. **Arduino alone first:** USB only, nothing on the headers; it must show up as a serial port and stay cool.
2. **Identify the module.** DS3231 modules come in several forms. **Look at the battery holder:**
   * **ZS-042 style** (blue board, battery holder, two small chips): it has a **trickle-charging circuit meant for a rechargeable
     LIR2032**. With a normal **CR2032 (non-rechargeable)** it can overheat or even rupture the cell. Either fit an LIR2032, or
     **remove the charging resistor (marked 201 or 200 ohm, near the battery) or the diode beside it**, or run it with no battery.
   * Modules **without** a charging circuit (e.g. DFRobot / Adafruit DS3231 breakouts, ChronoDot) take a CR2032 safely.
   * Never leave a hot module powered.
3. **Read the labels on the module** (GND, VCC, SDA, SCL; the order varies; SQW and 32K are not used).
4. **Wire with USB unplugged:**

   | DS3231 module | UNO R4 WiFi |
   |---|---|
   | GND | GND |
   | VCC | 5V (3.3V also works for most modules; use 5V only if the module says so) |
   | SDA | A4 |
   | SCL | A5 |
5. **Check before power** with a meter on resistance: VCC-to-GND must not read near 0 ohms.
6. **Power up and feel for heat within 10 seconds.** Anything warm on the module: unplug USB at once.

Expected I2C addresses: **0x68** (the RTC) and usually **0x57** (a small EEPROM on the same module).

## Run
Open `experiments/rtc_test/rtc_test.ino` in the Arduino IDE (board *Arduino UNO R4 WiFi*), upload, Serial Monitor at 115200
(or let Claude drive the console). Steps 0-6 run once; the matrix shows the step in hex (left) and a result (right); the table at the
top of `rtc_test.ino` says what each one means. To set the clock exactly, send  `T 2026-10-06 10:31:00`  from the console
(Claude can send the host's current time for you).
