// Hal.h — the Hardware Access Layer (Requirements ARC-1, ARC-4).
//
// The only door to the hardware. Logic code takes a Hal& and never includes
// Arduino headers, so it runs unchanged on the host against a fake.
// Deliberately thin: no policy, no timing logic, no state.
//
// M2 covers clock, GPIO, ADC and an I2C presence probe. Console, watchdog,
// reset-cause and I2C data transfer arrive with the milestones that need them.
#pragma once

#include <stdint.h>

namespace n2 {

enum class PinMode : uint8_t { kInput, kInputPullup, kOutput };

class Hal {
 public:
  virtual ~Hal() = default;

  virtual uint32_t millis() = 0;

  virtual void pinMode(uint8_t pin, PinMode mode) = 0;
  virtual void digitalWrite(uint8_t pin, bool high) = 0;
  virtual bool digitalRead(uint8_t pin) = 0;

  virtual void setAnalogResolution(uint8_t bits) = 0;
  virtual uint16_t analogRead(uint8_t pin) = 0;

  virtual void i2cBegin() = 0;
  // True if a device acknowledges its address.
  virtual bool i2cProbe(uint8_t address) = 0;
};

}  // namespace n2
