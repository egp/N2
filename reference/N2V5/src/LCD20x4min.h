// LCD20x4min.h v3
#pragma once
#include <Wire.h>

class LCD20x4_I2C {
public:
  LCD20x4_I2C(uint8_t addr = 0x27)
    : _addr(addr) {}

  void begin() {
    delay(50);
    write4(0x30); delay(5);
    write4(0x30); delay(5);
    write4(0x30); delay(1);
    write4(0x20);  // 4-bit mode

    cmd(0x28);  // 4-bit, 2-line
    cmd(0x0C);  // display ON, cursor off, blink off
    cmd(0x06);  // entry mode
    cmd(0x01);  // clear
    delay(2);
    _displayOn = true;
  }

  void printRow(uint8_t row, const char* text) {
    setRow(row);
    for (int i = 0; i < 20; i++) {
      char c = text[i];
      if (c == '\0') break;
      writeChar(c);
    }
  }

  // ← NEW: alias matching old TCP20x4 API
  void writeLine(uint8_t row, const char* text) { printRow(row, text); }

  void displayOn()  { _displayOn = true;  cmd(0x0C); }   // ← NEW
  void displayOff() { _displayOn = false; cmd(0x08); }   // ← NEW

  void backlightOn()  { _backlight = BL; pushBacklight(); }   // ← NEW
  void backlightOff() { _backlight = 0;  pushBacklight(); }   // ← NEW

private:
  uint8_t _addr;
  bool    _displayOn = false;

  static constexpr uint8_t RS = 0x01;
  static constexpr uint8_t EN = 0x04;
  static constexpr uint8_t BL = 0x08;

  uint8_t _backlight = BL;

  void setRow(uint8_t row) {
    static const uint8_t rowAddr[] = { 0x00, 0x40, 0x14, 0x54 };
    cmd(0x80 | rowAddr[row & 0x03]);
  }

  void cmd(uint8_t v)    { send(v, 0); }
  void writeChar(char c) { send((uint8_t)c, RS); }

  void send(uint8_t v, uint8_t mode) {
    write4((v & 0xF0) | mode);
    write4(((v << 4) & 0xF0) | mode);
  }

  void write4(uint8_t v) { pulse(v | _backlight); }

  void pulse(uint8_t v) {
    Wire.beginTransmission(_addr);
    Wire.write(v | EN);
    Wire.endTransmission();
    delayMicroseconds(1);
    Wire.beginTransmission(_addr);
    Wire.write(v & ~EN);
    Wire.endTransmission();
  }

  // ← NEW: update PCF8574 latch with new backlight state, no EN pulse
  void pushBacklight() {
    Wire.beginTransmission(_addr);
    Wire.write(_backlight);
    Wire.endTransmission();
  }
};
// LCD20x4min.h v3