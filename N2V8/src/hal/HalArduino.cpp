// HalArduino.cpp — one line per call; no logic here (ARC-4).
#ifdef ARDUINO

#include "HalArduino.h"

#include <Arduino.h>
#include <WDT.h>
#include <Wire.h>

namespace n2 {

uint32_t HalArduino::millis() { return ::millis(); }
uint32_t HalArduino::micros() { return ::micros(); }

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

bool HalArduino::i2cWrite(uint8_t address, const uint8_t* data, size_t n) {
  Wire.beginTransmission(address);
  Wire.write(data, n);
  return Wire.endTransmission() == 0;
}

// Console. `Serial` is true only while a host has the USB port open. NEVER call Serial.dtr():
// in this core it forces the connected state permanently (Requirements CON-4).
bool HalArduino::consoleAttached() { return static_cast<bool>(Serial); }
int HalArduino::consoleRead() { return Serial.available() > 0 ? Serial.read() : -1; }
size_t HalArduino::consoleWriteSpace() { return Serial ? static_cast<size_t>(Serial.availableForWrite()) : 0; }
size_t HalArduino::consoleWrite(const char* data, size_t n) { return Serial.write(reinterpret_cast<const uint8_t*>(data), n); }

void HalArduino::watchdogBegin(uint32_t timeoutMs) { WDT.begin(timeoutMs); }
void HalArduino::watchdogRefresh() { WDT.refresh(); }

// UNVERIFIED until the reset probe (experiments/reset_probe) has been run on both boards:
// RSTSR0.PORF = power-on, RSTSR1.WDTRF/IWDTRF = watchdog, RSTSR0.LVD0RF = voltage monitor.
ResetInfo HalArduino::readResetCause() {
  const uint8_t r0 = R_SYSTEM->RSTSR0;
  const uint16_t r1 = R_SYSTEM->RSTSR1;
  ResetInfo info;
  info.known = true;
  info.powerOn = (r0 & 0x01) != 0;
  info.brownout = (r0 & 0x0E) != 0;
  info.watchdog = (r1 & 0x03) != 0;
  R_SYSTEM->RSTSR0 = 0;  // clear so the next reset shows only its own cause
  R_SYSTEM->RSTSR1 = 0;
  return info;
}

}  // namespace n2

#endif  // ARDUINO
