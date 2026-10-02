// TM1650min.h v4
#pragma once
#include <Wire.h>

class TM1650 {
public:
  TM1650(uint8_t addr = 0x24)
    : _addr(addr) {}

  void begin() { delay(50); }

  void setBrightness(uint8_t b) {
    // user 0..7 → HW 1..7,0 (HW 0 = brightest, HW 1 = least bright)
    _brightnessBits = (((b & 0x07) + 1) & 0x07) << 4;
    writeControl();
  }

  void displayOn()  { _displayOn = true;  writeControl(); }   // ← NEW
  void displayOff() { _displayOn = false; writeControl(); }   // ← NEW

  void setDot(uint8_t pos, bool on) {
    for (int i = 0; i < 4; i++) _dots[i] = false;
    if (pos < 4) _dots[pos] = on;
    refresh();
  }

  void clearDots() {
    for (int i = 0; i < 4; i++) _dots[i] = false;
    refresh();
  }

  void displayInt(uint16_t value) {
    if (value > 9999) value %= 10000;
    for (int i = 3; i >= 0; --i) {
      _digits[i] = encode(value % 10);
      value /= 10;
    }
    refresh();
  }

  void displayHex(uint16_t value) {
    for (int i = 3; i >= 0; --i) {
      _digits[i] = encode(value & 0x0F);
      value >>= 4;
    }
    refresh();
  }

  // ← NEW: matches old TCP1650 setNumber API
  void setNumber(uint16_t value, bool suppressLeadingZeros) {
    if (value > 9999) value %= 10000;
    uint8_t raw[4];
    for (int i = 3; i >= 0; --i) {
      raw[i] = value % 10;
      value /= 10;
    }
    bool started = !suppressLeadingZeros;
    for (int i = 0; i < 4; i++) {
      if (raw[i] != 0 || i == 3) started = true;
      _digits[i] = started ? encode(raw[i]) : 0x00;  // 0x00 = all segments off
    }
    refresh();
  }

  // ← NEW: TM1650 keypad-scan read; returns raw byte (typically 0x44 = no key)
  uint8_t getButtons() {
    uint8_t received = Wire.requestFrom(_addr, (uint8_t)1);
    if (received < 1) return 0;
    return Wire.read();
  }

private:
  uint8_t _addr;
  uint8_t _digits[4]      = { 0, 0, 0, 0 };
  bool    _dots[4]        = { false, false, false, false };
  uint8_t _brightnessBits = 0x10;     // HW level 1 = least bright
  bool    _displayOn      = true;

  static const uint8_t DIGIT_ADDR[4];
  static const uint8_t lut[16];

  uint8_t encode(uint8_t n) { return lut[n & 0x0F]; }

  void writeControl() {
    // bits 4-6: brightness, bit 3: 0 = 8-seg, bit 0: display on/off
    uint8_t ctrl = _brightnessBits | (_displayOn ? 0x01 : 0x00);
    Wire.beginTransmission(_addr);
    Wire.write(ctrl);
    Wire.endTransmission();
  }

  void refresh() {
    for (int i = 0; i < 4; i++) {
      uint8_t b = _digits[i];
      if (_dots[i]) b |= 0x80;
      Wire.beginTransmission(DIGIT_ADDR[i]);
      Wire.write(b);
      Wire.endTransmission();
    }
  }
};

const uint8_t TM1650::DIGIT_ADDR[4] = { 0x34, 0x35, 0x36, 0x37 };

const uint8_t TM1650::lut[16] = {
  0x3F, 0x06, 0x5B, 0x4F,
  0x66, 0x6D, 0x7D, 0x07,
  0x7F, 0x6F, 0x77, 0x7C,
  0x39, 0x5E, 0x79, 0x71
};
// TM1650min.h v4