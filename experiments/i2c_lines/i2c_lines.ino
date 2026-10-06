// ===========================================================================================
// i2c_lines  VERSION 1.1   (2026-10-06)    <-- if you do not see this line, the IDE has an older copy
//
// READ-ONLY wiring probe, no libraries and no I2C traffic. Every second it prints the level of A4 (SDA), A5 (SCL), D0 (TBS)
// and D1 (TOB) with all internal pull-ups OFF, so what it reports is what YOUR pull-up resistors and the modules do.
//   expected with the bus idle:  SDA=1 SCL=1  (pulled to +5 V)     D0=1 D1=1 (buttons not pressed)   buttons pressed: 0
//   SDA or SCL = 0 : that line is held low (short to GND, a module that is miswired or damaged, or the pull-up goes to GND)
// Then it tries to pull each I2C line low itself for 1 ms and checks it releases (shorted to another line shows up here).
// 1.1: D0 and D1 are read twice, without and with the internal pull-up, to separate wiring from firmware.
// Console 115200.
// ===========================================================================================
#include <Arduino.h>

static const char* level(int v) { return v ? "1 (high)" : "0 (LOW)"; }

void setup() {
  Serial.begin(115200);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(0, INPUT);
  pinMode(1, INPUT);
}

void loop() {
  static uint32_t next = 0;
  if (millis() < next) return;
  next = millis() + 1000;
  const int sda = digitalRead(A4), scl = digitalRead(A5);
  Serial.print("idle: SDA(A4)="); Serial.print(level(sda));
  Serial.print("  SCL(A5)="); Serial.print(level(scl));
  Serial.print("  D0(TBS)="); Serial.print(level(digitalRead(0)));
  Serial.print("  D1(TOB)="); Serial.print(level(digitalRead(1)));
  pinMode(0, INPUT_PULLUP); pinMode(1, INPUT_PULLUP); delayMicroseconds(200);
  Serial.print("   | with internal pull-up: D0="); Serial.print(digitalRead(0)); Serial.print(" D1="); Serial.println(digitalRead(1));
  pinMode(0, INPUT); pinMode(1, INPUT);

  // drive SDA low briefly: SCL must not follow (a short between the two lines would drag it along)
  pinMode(A4, OUTPUT); digitalWrite(A4, LOW); delayMicroseconds(200);
  const int sclWhileSdaLow = digitalRead(A5);
  pinMode(A4, INPUT);
  pinMode(A5, OUTPUT); digitalWrite(A5, LOW); delayMicroseconds(200);
  const int sdaWhileSclLow = digitalRead(A4);
  pinMode(A5, INPUT);
  delayMicroseconds(300);
  Serial.print("   drive test: SCL while SDA forced low = "); Serial.print(sclWhileSdaLow);
  Serial.print(" (want 1);  SDA while SCL forced low = "); Serial.print(sdaWhileSclLow);
  Serial.print(" (want 1);  both released: "); Serial.print(digitalRead(A4)); Serial.println(digitalRead(A5));
}
