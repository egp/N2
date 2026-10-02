// HalArduino.cpp — one line per call; no logic here (ARC-4).
#ifdef ARDUINO

#include "HalArduino.h"

#include <Arduino.h>
#include <Wire.h>

namespace n2 {

uint32_t HalArduino::millis() { return ::millis(); }

void HalArduino::pinMode(uint8_t pin, PinMode mode) {
  switch (mode) {
    case PinMode::kInput:       ::pinMode(pin, INPUT); break;
    case PinMode::kInputPullup: ::pinMode(pin, INPUT_PULLUP); break;
    case PinMode::kOutput:      ::pinMode(pin, OUTPUT); break;
  }
}

void HalArduino::digitalWrite(uint8_t pin, bool high) { ::digitalWrite(pin, high ? HIGH : LOW); }
bool HalArduino::digitalRead(uint8_t pin) { return ::digitalRead(pin) == HIGH; }

void HalArduino::setAnalogResolution(uint8_t bits) { ::analogReadResolution(bits); }
uint16_t HalArduino::analogRead(uint8_t pin) { return static_cast<uint16_t>(::analogRead(pin)); }

void HalArduino::i2cBegin() { Wire.begin(); }

bool HalArduino::i2cProbe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

}  // namespace n2

#endif  // ARDUINO
