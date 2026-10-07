// SoftI2c.h — a bit-banged I2C master on two ordinary GPIO pins (Requirements DRV-4).
//
// Why it exists: the TM1650 LED module also answers at 0x25-0x27, and the LCD backpack lives at 0x27; with both on one bus every LCD character
// write failed (bench, 2026-10-07: docs/results/lcd-led-address-clash-20261007.md). The LED therefore gets its own two wires, on this software
// bus, and shares nothing with the LCD, the RTC or the O2 sensor.
//
// Open-drain like the real thing: a line is "low" when the pin is an output driven LOW and "high" when the pin is an input (the module's own
// pull-up, or the pin's internal pull-up, pulls it up). Master write only (the TM1650 needs nothing else). No clock stretching. Roughly
// 50 kHz at the default half period; one byte takes about 0.2 ms, so the LED driver (one byte per transaction) stays far below the loop budget.
#pragma once

#include <stdint.h>

#include "../hal/Hal.h"
#include "../hal/I2cBus.h"

namespace n2 {

class SoftI2c : public I2cBus {
 public:
  static constexpr uint8_t kNoPin = 0xFF;

  SoftI2c(Hal& hal, uint8_t sdaPin, uint8_t sclPin, uint8_t halfPeriodUs = 10) : hal_(hal), sda_(sdaPin), scl_(sclPin), half_(halfPeriodUs) {}

  bool configured() const { return sda_ != kNoPin && scl_ != kNoPin; }
  void begin();  // release both lines
  // If a device holds SDA low (a reset in the middle of a byte): clock SCL up to nine times, then send a STOP.
  void recover();

  bool write(uint8_t address, const uint8_t* data, size_t n) override;
  bool probe(uint8_t address) override;

 private:
  void sdaLow();
  void sdaHigh();   // release
  void sclLow();
  void sclHigh();   // release
  bool sdaRead();
  void wait() { hal_.delayMicroseconds(half_); }
  void start();
  void stop();
  bool sendByte(uint8_t value);  // true if acknowledged

  Hal& hal_;
  uint8_t sda_;
  uint8_t scl_;
  uint8_t half_;
};

}  // namespace n2
