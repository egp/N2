    #pragma once
#include <Wire.h>

class LCD20x4_I2C {
public:
  LCD20x4_I2C(uint8_t addr = 0x27) : _addr(addr) {}

  void begin() {
    Wire.begin();
    delay(50);

    write4(0x30); delay(5);
    write4(0x30); delay(5);
    write4(0x30); delay(1);
    write4(0x20); // 4-bit mode

    cmd(0x28); // 4-bit, 2-line (works for 20x4)
    cmd(0x0C); // display ON
    cmd(0x06); // entry mode
    cmd(0x01); // clear
    delay(2);
  }

  void printRow(uint8_t row, const char* text) {
    setRow(row);

    for (int i = 0; i < 20; i++) {
      char c = text[i];
      if (c == '\0') break;
      writeChar(c);
    }
  }

private:
  uint8_t _addr;

  static constexpr uint8_t RS = 0x01;
  static constexpr uint8_t EN = 0x04;
  static constexpr uint8_t BL = 0x08;

  uint8_t backlight = BL;

  // Standard HD44780 20x4 row offsets
  void setRow(uint8_t row) {
    static const uint8_t rowAddr[] = {0x00, 0x40, 0x14, 0x54};
    cmd(0x80 | rowAddr[row & 0x03]);
  }

  void cmd(uint8_t v) {
    send(v, 0);
  }

  void writeChar(char c) {
    send((uint8_t)c, RS);
  }

  void send(uint8_t v, uint8_t mode) {
    write4((v & 0xF0) | mode);
    write4(((v << 4) & 0xF0) | mode);
  }

  void write4(uint8_t v) {
    pulse(v | backlight);
  }

  void pulse(uint8_t v) {
    Wire.beginTransmission(_addr);
    Wire.write(v | EN);
    Wire.endTransmission();

    delayMicroseconds(1);

    Wire.beginTransmission(_addr);
    Wire.write(v & ~EN);
    Wire.endTransmission();
  }
};