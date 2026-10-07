// ===========================================================================================
// alias_probe  VERSION 1.3   (2026-10-07)    <-- if you do not see this line, the IDE has an older copy
//
// Question: the TM1650 LED module also answers at 0x25, 0x26 and 0x27, where the PCF8574 LCD backpack lives (0x27). What does it do with bytes
// written there? Pure Wire, no firmware code. Every 3 s it sends writes of 1..6 bytes to 0x27 and prints Wire's return code:
//   0 = all bytes acknowledged, 2 = NACK on the address, 3 = NACK on a data byte, 4 = other error.
// The first byte is always 0x11 (what the LED wants as its control byte: brightness 1, 8-segment mode, display on): for a PCF8574 backpack it
// is a harmless nibble (EN low, backlight off for an instant); the other bytes are 0x08 (backlight on, everything else low), also harmless.
// Run once with ONLY the LED module on the bus, then again with the LCD backpack connected too, and compare the lines.
// 1.1: Wire is restarted (end + begin) before every single write, so one refused write cannot make the next ones fail (1.0 showed the R4's Wire
//      stays broken after an error). Also prints what the LED chip returns when READ at 0x27, and the first byte of the answer.
// 1.2: after Wire.begin() the clock is set again (Wire.setClock): on the R4 core only setClock() REOPENS the controller after Wire.end(); without it
//      every later transaction failed with code 2 (1.0 and 1.1 results after the first error are wrong because of that).
// 1.3: the bus is fully recovered between tests exactly like the firmware's Hal::i2cRecover (SCL clocked up to 9 times until SDA is released, a STOP,
//      then Wire restarted): a plain Wire restart did not free the bus after a refused write.
// Console 115200. Command: r = repeat now.
// ===========================================================================================
#include <Arduino.h>
#include <Wire.h>

static void restartWire() {
  Wire.end();
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  delayMicroseconds(10);
  for (uint8_t pulse = 0; pulse < 9 && digitalRead(A4) == LOW; ++pulse) {
    pinMode(A5, OUTPUT); digitalWrite(A5, LOW); delayMicroseconds(10);
    pinMode(A5, INPUT); delayMicroseconds(10);
  }
  pinMode(A4, OUTPUT); digitalWrite(A4, LOW); delayMicroseconds(10);  // STOP
  pinMode(A4, INPUT); delayMicroseconds(10);
  Wire.begin();
  Wire.setClock(100000);
}

static void probe() {
  Serial.println("\n--- writes of N bytes to 0x27 (first byte 0x11, the rest 0x08) ---");
  for (uint8_t n = 1; n <= 6; ++n) {
    restartWire();
    Wire.beginTransmission(0x27);
    Wire.write(static_cast<uint8_t>(0x11));
    for (uint8_t i = 1; i < n; ++i) Wire.write(static_cast<uint8_t>(0x08));
    const uint8_t rc = Wire.endTransmission();
    Serial.print("  "); Serial.print(n); Serial.print(" byte(s): return code "); Serial.println(rc);
    delay(20);
  }
  Serial.println("--- same to 0x24 (the LED's own control address) for comparison ---");
  for (uint8_t n = 1; n <= 3; ++n) {
    restartWire();
    Wire.beginTransmission(0x24);
    Wire.write(static_cast<uint8_t>(0x11));
    for (uint8_t i = 1; i < n; ++i) Wire.write(static_cast<uint8_t>(0x11));
    const uint8_t rc = Wire.endTransmission();
    Serial.print("  "); Serial.print(n); Serial.print(" byte(s): return code "); Serial.println(rc);
    delay(20);
  }
  Serial.println("--- read 1 byte from 0x27 (the LED chip returns its key code; a PCF8574 backpack returns its pin levels) ---");
  restartWire();
  const uint8_t got = Wire.requestFrom(static_cast<uint8_t>(0x27), static_cast<uint8_t>(1));
  Serial.print("  bytes received: "); Serial.print(got);
  if (got > 0) { Serial.print("  value 0x"); Serial.print(Wire.read(), HEX); }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(1500);
  Serial.println("\n==== alias_probe version 1.3 ====");
  probe();
}

void loop() {
  static uint32_t next = 4000;
  if (Serial.available() > 0 && Serial.read() == 'r') next = millis();
  if (millis() >= next) { probe(); next = millis() + 5000; }
}
