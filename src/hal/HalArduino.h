// HalArduino.h — the real Hal, backed by the Arduino core. Device builds only.
#pragma once

#ifdef ARDUINO

#include "Hal.h"

namespace n2 {

class HalArduino : public Hal {
 public:
  uint32_t millis() override;
  void pinMode(uint8_t pin, PinMode mode) override;
  void digitalWrite(uint8_t pin, bool high) override;
  bool digitalRead(uint8_t pin) override;
  void setAnalogResolution(uint8_t bits) override;
  uint16_t analogRead(uint8_t pin) override;
  void i2cBegin() override;
  bool i2cProbe(uint8_t address) override;
};

}  // namespace n2

#endif  // ARDUINO
