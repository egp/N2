// FakeHal.h — scriptable, recording Hal for host tests (Requirements ARC-2).
#pragma once

#include <array>
#include <cstdint>
#include <set>
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
  std::array<uint16_t, 32> analogValue{};   // returned by analogRead(pin)
  std::array<bool, 32> inputLevel{};        // returned by digitalRead(pin)
  std::set<uint8_t> i2cPresent;             // addresses that acknowledge

  // ---- observed state ----
  std::vector<Event> events;
  std::array<int, 32> mode{};               // -1 = never configured
  std::array<int, 32> level{};              // -1 = never written
  uint8_t adcBits = 0;
  bool i2cStarted = false;

  FakeHal() { mode.fill(-1); level.fill(-1); }

  uint32_t millis() override { return nowMs; }

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
};

}  // namespace n2
