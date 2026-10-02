// TestSupport.h — fakes and a ready-made "plant" for the host tests.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "BoardPins.h"
#include "Config.h"
#include "ControlConfig.h"
#include "FakeHal.h"
#include "core/Log.h"
#include "core/O2Reader.h"
#include "core/Scaling.h"
#include "core/System.h"

namespace n2 {
namespace test {

class VectorLog : public LogSink {
 public:
  std::vector<std::string> lines;
  std::vector<LogLevel> levels;
  void write(LogLevel level, const char* line) override {
    lines.push_back(line);
    levels.push_back(level);
  }
  size_t count(const std::string& needle) const {
    size_t n = 0;
    for (const auto& l : lines)
      if (l.find(needle) != std::string::npos) ++n;
    return n;
  }
  bool has(const std::string& needle) const { return count(needle) > 0; }
  void clear() { lines.clear(); levels.clear(); }
};

class FakeO2 : public O2Reader {
 public:
  bool presentOk = true;
  bool readOk = true;
  uint16_t o2x100 = 2090;  // air: 20.90 % O2
  int beginCalls = 0;
  int readCalls = 0;
  bool begin() override { ++beginCalls; return presentOk; }
  bool present() override { return presentOk; }
  bool readO2PercentX100(uint16_t& v) override {
    ++readCalls;
    if (!presentOk || !readOk) return false;
    v = o2x100;
    return true;
  }
};

// Raw ADC count that scales to `value` (fixed-point PSI) for a sensor of the given full scale.
inline uint16_t rawFor(uint16_t value, uint16_t fullScale, uint8_t bits = kAdcBits) {
  const AdcWindow w = adcWindow(bits);
  return static_cast<uint16_t>(w.validMin + (static_cast<uint32_t>(value) * (w.validMax - w.validMin) + fullScale / 2u) / fullScale);
}

// Healthy default pressures, comfortably inside every threshold.
constexpr uint16_t kHealthyAirX10 = 1300;   // 130.0 PSI (> airLowOn 120.0)
constexpr uint16_t kHealthyN2LowX100 = 2500;  // 25.00 PSI (> n2LowOn 20.00)
constexpr uint16_t kHealthyN2HighX10 = 800;   // 80.0 PSI (< n2HighOn 100.0)

// A whole plant on the fake HAL: sets inputs, advances time, reads actual outputs.
// reboot() emulates a reset: a brand-new System, millis() back to 0, pins keep whatever
// level they had (as real hardware does until the firmware drives them).
struct Plant {
  FakeHal hal;
  VectorLog log;
  FakeO2 o2;
  ControlConfig cfg;
  uint8_t bits;
  std::unique_ptr<System> sysp;

  explicit Plant(ControlConfig c = kDefaultControl, uint8_t adcBits = kAdcBits) : cfg(c), bits(adcBits) {
    sysp.reset(new System(hal, kHostBoard, cfg, o2, log, bits));
    healthy();
    tbs(false);
    tob(false);
  }

  System& sys() { return *sysp; }
  const System& sys() const { return *sysp; }

  void reboot(const ResetInfo& reset = ResetInfo(), uint32_t warmCreditMs = 0) {
    sysp.reset(new System(hal, kHostBoard, cfg, o2, log, bits));
    hal.nowMs = 0;
    sysp->begin(reset, warmCreditMs);
  }

  void air(uint16_t x10) { hal.analogValue[pinOf(kHostBoard, Signal::kAirPressure)] = rawFor(x10, kAirFullScaleX10, bits); }
  void n2Low(uint16_t x100) { hal.analogValue[pinOf(kHostBoard, Signal::kN2LowPressure)] = rawFor(x100, kN2LowFullScaleX100, bits); }
  void n2High(uint16_t x10) { hal.analogValue[pinOf(kHostBoard, Signal::kN2HighPressure)] = rawFor(x10, kN2HighFullScaleX10, bits); }
  void rawAir(uint16_t raw) { hal.analogValue[pinOf(kHostBoard, Signal::kAirPressure)] = raw; }
  void rawN2Low(uint16_t raw) { hal.analogValue[pinOf(kHostBoard, Signal::kN2LowPressure)] = raw; }
  void rawN2High(uint16_t raw) { hal.analogValue[pinOf(kHostBoard, Signal::kN2HighPressure)] = raw; }
  void healthy() { air(kHealthyAirX10); n2Low(kHealthyN2LowX100); n2High(kHealthyN2HighX10); }

  void tbs(bool on) {
    const SignalDef& d = def(kHostBoard, Signal::kTbs);
    hal.inputLevel[d.pin] = levelHigh(on, d.active);
  }
  void tob(bool on) {
    const SignalDef& d = def(kHostBoard, Signal::kTob);
    hal.inputLevel[d.pin] = levelHigh(on, d.active);
  }

  void step(uint32_t ms = 10) { hal.nowMs += ms; sysp->step(); }
  void run(uint32_t ms, uint32_t stepMs = 10) {
    for (uint32_t t = 0; t < ms; t += stepMs) step(stepMs);
  }
  // Run until `pred` is true or `limitMs` passes. Returns whether it became true.
  template <typename Pred>
  bool runUntil(Pred pred, uint32_t limitMs, uint32_t stepMs = 10) {
    for (uint32_t t = 0; t < limitMs; t += stepMs) {
      if (pred()) return true;
      step(stepMs);
    }
    return pred();
  }

  bool out(Signal s) const {
    const SignalDef& d = def(kHostBoard, s);
    return isOn(hal.level[d.pin] == 1, d.active);
  }
  bool left() const { return out(Signal::kLeftValve); }
  bool right() const { return out(Signal::kRightValve); }
  bool flush() const { return out(Signal::kFlushValve); }
  bool ssr() const { return out(Signal::kSsr); }
};

// A config with a short warm-up, so scenario tests do not have to run five minutes.
inline ControlConfig quickWarmConfig() {
  ControlConfig c = kDefaultControl;
  c.o2WarmupMs = 2000;
  return c;
}

}  // namespace test
}  // namespace n2
