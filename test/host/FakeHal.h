// FakeHal.h — scriptable, recording Hal for host tests (Requirements ARC-2).
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "hal/Hal.h"

namespace n2 {

class FakeHal : public Hal {
 public:
  enum class Kind { kPinMode, kWrite, kRead, kAnalogRead, kAnalogResolution, kI2cBegin, kI2cProbe };
  struct Event {
    Kind kind;
    uint8_t pin;   // pin, or I2C address for kI2cProbe, or bits for kAnalogResolution
    int value;     // level / mode / result
  };

  // ---- scripting ----
  uint32_t nowMs = 0;
  uint32_t nowUs = 0;                       // returned by micros()
  bool consoleIsAttached = false;
  bool consoleBegun = false;                 // set by consoleBegin()
  bool canDetectHost = true;                 // false emulates the R4 WiFi (always 'attached', cannot see the PC)
  std::deque<char> consoleIn;                // bytes the "host" has typed
  std::string consoleOut;                    // everything written to the console
  size_t consoleSpace = 4096;                // free TX buffer; a test can shrink it to simulate a stalled host
  uint32_t watchdogTimeoutMs = 0;
  uint32_t watchdogRefreshes = 0;
  ResetInfo resetCause;
  std::array<uint16_t, 32> analogValue{};   // returned by analogRead(pin)
  std::array<bool, 32> inputLevel{};        // returned by digitalRead(pin)
  std::set<uint8_t> i2cPresent;             // addresses that acknowledge (probes AND writes)
  struct I2cWrite {
    uint8_t address;
    std::vector<uint8_t> bytes;
    bool acked;
  };
  std::vector<I2cWrite> i2cWrites;          // every write, in order
  std::map<uint8_t, std::vector<uint8_t>> i2cRegs;  // register file per device (256 bytes, created on demand), for register-style devices
  uint32_t i2cReads = 0;
  uint8_t regPointer(uint8_t address) { auto& r = i2cRegs[address]; if (r.empty()) r.assign(256, 0); return 0; }

  // ---- observed state ----
  std::vector<Event> events;
  std::array<int, 32> mode{};               // -1 = never configured
  std::array<int, 32> level{};              // -1 = never written
  uint8_t adcBits = 0;
  bool i2cStarted = false;

  FakeHal() { mode.fill(-1); level.fill(-1); }

  uint32_t millis() override { return nowMs; }
  uint32_t microsPerCall = 0;               // micros() advances by this much on every call (to give loop time a size)
  uint32_t micros() override { nowUs += microsPerCall; return nowUs; }

  void pinMode(uint8_t pin, PinMode m) override {
    mode[pin] = static_cast<int>(m);
    events.push_back({Kind::kPinMode, pin, static_cast<int>(m)});
  }
  void digitalWrite(uint8_t pin, bool high) override {
    level[pin] = high ? 1 : 0;
    events.push_back({Kind::kWrite, pin, high ? 1 : 0});
  }
  bool digitalRead(uint8_t pin) override {
    events.push_back({Kind::kRead, pin, inputLevel[pin] ? 1 : 0});
    return inputLevel[pin];
  }
  void setAnalogResolution(uint8_t bits) override {
    adcBits = bits;
    events.push_back({Kind::kAnalogResolution, bits, 0});
  }
  uint16_t analogRead(uint8_t pin) override {
    events.push_back({Kind::kAnalogRead, pin, analogValue[pin]});
    return analogValue[pin];
  }
  void i2cBegin() override {
    i2cStarted = true;
    events.push_back({Kind::kI2cBegin, 0, 0});
  }
  bool i2cProbe(uint8_t address) override {
    const bool ack = i2cPresent.count(address) != 0;
    events.push_back({Kind::kI2cProbe, address, ack ? 1 : 0});
    return ack;
  }
  bool i2cWrite(uint8_t address, const uint8_t* data, size_t n) override {
    const bool ack = i2cPresent.count(address) != 0;
    i2cWrites.push_back({address, std::vector<uint8_t>(data, data + n), ack});
    if (ack && n >= 1) lastByteWritten[address] = data[n - 1];
    if (ack && n >= 2) {  // register-style device: data[0] = register pointer, the rest are stored from there
      regPointer(address);
      auto& r = i2cRegs[address];
      for (size_t i = 1; i < n; ++i) r[static_cast<uint8_t>(data[0] + i - 1)] = data[i];
    }
    return ack;
  }
  std::map<uint8_t, uint8_t> lastByteWritten;  // what a port expander (PCF8574) would read back
  uint8_t i2cReadXor = 0;                      // a test can corrupt readback to simulate a bad bus
  bool i2cRead(uint8_t address, uint8_t* data, size_t n) override {
    ++i2cReads;
    if (i2cPresent.count(address) == 0) return false;
    for (size_t i = 0; i < n; ++i) data[i] = static_cast<uint8_t>(lastByteWritten[address] ^ i2cReadXor);
    return true;
  }
  bool i2cReadReg(uint8_t address, uint8_t reg, uint8_t* data, size_t n) override {
    ++i2cReads;
    if (i2cPresent.count(address) == 0) return false;
    regPointer(address);
    auto& r = i2cRegs[address];
    for (size_t i = 0; i < n; ++i) data[i] = r[static_cast<uint8_t>(reg + i)];
    return true;
  }

  // ---- console ----
  void consoleBegin() override { consoleBegun = true; }
  bool consoleCanDetectHost() override { return canDetectHost; }
  bool consoleAttached() override { return consoleIsAttached; }
  int consoleRead() override {
    if (!consoleIsAttached || consoleIn.empty()) return -1;
    const char c = consoleIn.front();
    consoleIn.pop_front();
    return static_cast<unsigned char>(c);
  }
  size_t consoleWriteSpace() override { return consoleIsAttached ? consoleSpace : 0; }
  size_t consoleWrite(const char* data, size_t n) override {
    if (!consoleIsAttached) return 0;
    const size_t take = n < consoleSpace ? n : consoleSpace;
    consoleOut.append(data, take);
    consoleSpace -= take;  // stays consumed until the test drains it (a stalled host)
    return take;
  }
  void type(const std::string& text) { for (char c : text) consoleIn.push_back(c); }
  void drain() { consoleSpace = 4096; }

  // ---- watchdog and reset cause ----
  void watchdogBegin(uint32_t timeoutMs) override { watchdogTimeoutMs = timeoutMs; }
  void watchdogRefresh() override { ++watchdogRefreshes; }
  ResetInfo readResetCause() override { return resetCause; }
};

}  // namespace n2
