// HalArduino.h — the real Hal, backed by the Arduino core. Device builds only.
#pragma once

#ifdef ARDUINO

#include "../core/TxBudget.h"
#include "Hal.h"

namespace n2 {

class HalArduino : public Hal {
 public:
  uint32_t millis() override;
  uint32_t micros() override;
  void pinMode(uint8_t pin, PinMode mode) override;
  void digitalWrite(uint8_t pin, bool high) override;
  bool digitalRead(uint8_t pin) override;
  void setAnalogResolution(uint8_t bits) override;
  uint16_t analogRead(uint8_t pin) override;
  void i2cBegin() override;
  void i2cSetClock(uint32_t hz) override;
  bool i2cProbe(uint8_t address) override;
  bool i2cWrite(uint8_t address, const uint8_t* data, size_t n) override;
  bool i2cRead(uint8_t address, uint8_t* data, size_t n) override;
  bool i2cReadReg(uint8_t address, uint8_t reg, uint8_t* data, size_t n) override;
  void consoleBegin() override;
  bool consoleCanDetectHost() override;
  bool consoleAttached() override;
  int consoleRead() override;
  size_t consoleWriteSpace() override;
  size_t consoleWrite(const char* data, size_t n) override;
  void watchdogBegin(uint32_t timeoutMs) override;
  void watchdogRefresh() override;
  ResetInfo readResetCause() override;

 private:
  TxBudget txBudget_;  // paces output on the R4 WiFi, whose UART writes block
};

}  // namespace n2

#endif  // ARDUINO
