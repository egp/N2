// HalArduino.cpp — one line per call; no logic here (ARC-4).
#ifdef ARDUINO

#include "HalArduino.h"

#include "../Config.h"

#include <Arduino.h>
#include <WDT.h>
#include <Wire.h>

namespace n2 {

uint32_t HalArduino::millis() { return ::millis(); }
uint32_t HalArduino::micros() { return ::micros(); }
void HalArduino::delayMicroseconds(uint32_t us) { ::delayMicroseconds(us); }

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

void HalArduino::i2cBegin() {
  i2cRecover();  // a reset mid-transaction must not leave the bus stuck (this also starts Wire at the configured speed)
}

void HalArduino::i2cSetClock(uint32_t hz) { Wire.setClock(hz); }

bool HalArduino::i2cRecover() {
  Wire.end();
  ::pinMode(A4, INPUT);  // SDA and SCL as plain inputs: the external pull-ups hold them high
  ::pinMode(A5, INPUT);
  ::delayMicroseconds(10);
  for (uint8_t pulse = 0; pulse < 9 && ::digitalRead(A4) == LOW; ++pulse) {  // a slave mid-byte releases SDA after at most 9 clocks
    ::pinMode(A5, OUTPUT);
    ::digitalWrite(A5, LOW);
    ::delayMicroseconds(10);
    ::pinMode(A5, INPUT);
    ::delayMicroseconds(10);
  }
  ::pinMode(A4, OUTPUT);  // STOP condition: SDA rises while SCL is high
  ::digitalWrite(A4, LOW);
  ::delayMicroseconds(10);
  ::pinMode(A4, INPUT);
  ::delayMicroseconds(10);
  const bool sdaFree = ::digitalRead(A4) == HIGH;
  Wire.begin();
  Wire.setClock(kI2cClockHz);
  Wire.setWireTimeout(kI2cTimeoutUs);   // see Config.h: the 100 ms default per transaction stalls the loop on a held bus
  return sdaFree;
}

void HalArduino::noteI2c(uint8_t address, uint32_t t0) {
  if (stall_ == nullptr) return;
  const uint32_t us = ::micros() - t0;
  stall_->i2cLastAddr = address;
  stall_->i2cLastUs = us;
  if (us > stall_->i2cWorstUs) {
    stall_->i2cWorstUs = us;
    stall_->i2cWorstAddr = address;
  }
}

bool HalArduino::i2cProbe(uint8_t address) {
  const uint32_t t0 = ::micros();
  Wire.beginTransmission(address);
  const bool ok = Wire.endTransmission() == 0;
  noteI2c(address, t0);
  return ok;
}

bool HalArduino::i2cWrite(uint8_t address, const uint8_t* data, size_t n) {
  const uint32_t t0 = ::micros();
  Wire.beginTransmission(address);
  Wire.write(data, n);
  const bool ok = Wire.endTransmission() == 0;
  noteI2c(address, t0);
  return ok;
}

bool HalArduino::i2cRead(uint8_t address, uint8_t* data, size_t n) {
  const uint32_t t0 = ::micros();
  const uint8_t want = static_cast<uint8_t>(n);
  const bool ok = Wire.requestFrom(address, want) == want;
  if (ok)
    for (size_t i = 0; i < n; ++i) data[i] = static_cast<uint8_t>(Wire.read());
  noteI2c(address, t0);
  return ok;
}

bool HalArduino::i2cReadReg(uint8_t address, uint8_t reg, uint8_t* data, size_t n) {
  const uint32_t t0 = ::micros();
  Wire.beginTransmission(address);
  Wire.write(reg);
  bool ok = Wire.endTransmission(false) == 0;  // keep the bus (repeated start) for the read
  if (ok) {
    const uint8_t want = static_cast<uint8_t>(n);
    ok = Wire.requestFrom(address, want) == want;
    if (ok)
      for (size_t i = 0; i < n; ++i) data[i] = static_cast<uint8_t>(Wire.read());
  }
  noteI2c(address, t0);
  return ok;
}

// ---- Console -----------------------------------------------------------------------------------------------
// The two boards differ (see Requirements CON-4):
//  * UNO R4 Minima: native USB. The core starts Serial. `if (Serial)` is true only while a PC has the port open (DTR).
//    Serial.availableForWrite() reports free buffer space. NEVER call Serial.dtr() (it forces "connected" forever).
//  * UNO R4 WiFi: the core is built with -DNO_USB, so Serial is a hardware UART to the ESP32 USB bridge. The sketch MUST
//    call Serial.begin(); `if (Serial)` is always true; availableForWrite() is not implemented (returns 0); write() blocks
//    until each byte has left the chip, so output is paced with a byte budget (TxBudget).
#if defined(ARDUINO_UNOR4_WIFI)

void HalArduino::consoleBegin() { Serial.begin(115200); }
bool HalArduino::consoleCanDetectHost() { return false; }
bool HalArduino::consoleAttached() { return true; }
size_t HalArduino::consoleWriteSpace() { return txBudget_.space(millis()); }
size_t HalArduino::consoleWrite(const char* data, size_t n) {
  const size_t sent = Serial.write(reinterpret_cast<const uint8_t*>(data), n);
  txBudget_.consume(static_cast<uint32_t>(sent), millis());
  return sent;
}

#else  // UNO R4 Minima (native USB)

void HalArduino::consoleBegin() { Serial.begin(115200); }  // harmless: the core has already started it
bool HalArduino::consoleCanDetectHost() { return true; }
bool HalArduino::consoleAttached() { return static_cast<bool>(Serial); }
size_t HalArduino::consoleWriteSpace() { return Serial ? static_cast<size_t>(Serial.availableForWrite()) : 0; }
size_t HalArduino::consoleWrite(const char* data, size_t n) { return Serial.write(reinterpret_cast<const uint8_t*>(data), n); }

#endif

int HalArduino::consoleRead() { return Serial.available() > 0 ? Serial.read() : -1; }

void HalArduino::watchdogBegin(uint32_t timeoutMs) { WDT.begin(timeoutMs); }
void HalArduino::watchdogRefresh() { WDT.refresh(); }

// Reset cause, verified on the UNO R4 WiFi with experiments/reset_probe (docs/results/reset-probe-wifi-20261006.md):
//   * RSTSR1.SWRF (0x04) = software reset, RSTSR1.WDTRF (0x02) / IWDTRF (0x01) = watchdog, nothing flagged = reset button.
//   * RSTSR0.PORF is NOT visible (the bootloader clears it), so "was power lost?" is taken from RSTSR2.CWSF instead:
//     it reads 0 after a power loss ("cold start") and the firmware sets it to 1 here, so every later reset that does not
//     lose power reads 1 ("warm start").
//   * The flags persist until cleared, so they are cleared after reading. Plain writes work (no register unlock needed).
ResetInfo HalArduino::readResetCause() {
  const uint8_t r0 = R_SYSTEM->RSTSR0;
  const uint16_t r1 = R_SYSTEM->RSTSR1;
  const uint8_t r2 = R_SYSTEM->RSTSR2;
  ResetInfo info;
  info.known = true;
  info.powerOn = ((r2 & 0x01) == 0) || ((r0 & 0x01) != 0);  // cold start, or PORF if it ever shows
  info.brownout = (r0 & 0x0E) != 0;                         // voltage-monitor resets
  info.watchdog = (r1 & 0x03) != 0;
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
  R_SYSTEM->RSTSR2 = 0x01;  // from now on, a reset without power loss reads as a WARM start
  return info;
}

}  // namespace n2

#endif  // ARDUINO
