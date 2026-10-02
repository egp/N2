// Hal.h — the Hardware Access Layer (Requirements ARC-1, ARC-4).
//
// The only door to the hardware. Logic code takes a Hal& and never includes
// Arduino headers, so it runs unchanged on the host against a fake.
// Deliberately thin: no policy, no timing logic, no state.
//
// Covers clock, GPIO, ADC, an I2C presence probe, the USB console, the watchdog and the
// reset cause. I2C data transfer (display drivers) arrives with the drivers.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "ResetInfo.h"

namespace n2 {

enum class PinMode : uint8_t { kInput, kInputPullup, kOutput };

class Hal {
 public:
  virtual ~Hal() = default;

  virtual uint32_t millis() = 0;
  virtual uint32_t micros() = 0;

  virtual void pinMode(uint8_t pin, PinMode mode) = 0;
  virtual void digitalWrite(uint8_t pin, bool high) = 0;
  virtual bool digitalRead(uint8_t pin) = 0;

  virtual void setAnalogResolution(uint8_t bits) = 0;
  virtual uint16_t analogRead(uint8_t pin) = 0;

  virtual void i2cBegin() = 0;
  // True if a device acknowledges its address.
  virtual bool i2cProbe(uint8_t address) = 0;

  // USB console (CON-2, CON-4). Nothing here may block.
  virtual bool consoleAttached() = 0;                          // true only while a host has the port open
  virtual int consoleRead() = 0;                               // next input byte, or -1
  virtual size_t consoleWriteSpace() = 0;                      // bytes that can be written without waiting
  virtual size_t consoleWrite(const char* data, size_t n) = 0;

  // Hardware watchdog (WDT-1..3).
  virtual void watchdogBegin(uint32_t timeoutMs) = 0;
  virtual void watchdogRefresh() = 0;

  // Reset cause: read AND cleared (the hardware flags persist until cleared). Call once at boot.
  virtual ResetInfo readResetCause() = 0;
};

}  // namespace n2
