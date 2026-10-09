// Tests for Sensors: INP-1, INP-4, INP-5, INP-7 (and VER-4: all ADC bit depths).
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "TestSupport.h"

using namespace n2;
using namespace n2::test;

namespace {
struct Rig {
  FakeHal hal;
  VectorLog log;
  FaultSet faults{log};
  ControlConfig cfg = kDefaultControl;
  uint8_t bits;
  SensorMonitor mon;
  Inputs in;
  explicit Rig(uint8_t b = kAdcBits) : bits(b), mon(kHostBoard, cfg, b) {}
  void setAir(uint16_t x10) { hal.analogValue[pinOf(kHostBoard, Signal::kAirPressure)] = rawFor(x10, kAirFullScaleX10, bits); }
  void setLow(uint16_t x100) { hal.analogValue[pinOf(kHostBoard, Signal::kN2LowPressure)] = rawFor(x100, kN2LowFullScaleX100, bits); }
  void setHigh(uint16_t x10) { hal.analogValue[pinOf(kHostBoard, Signal::kN2HighPressure)] = rawFor(x10, kN2HighFullScaleX10, bits); }
  void sample(uint32_t now) { mon.sample(hal, now, faults, in); }
};
}  // namespace

TEST_CASE("INP-1: sample() reads the three sensors and scales them, at every ADC bit depth") {
  const uint8_t bits = static_cast<uint8_t>(GENERATE(10, 12, 14));
  Rig r(bits);
  r.setAir(1234);
  r.setLow(1875);
  r.setHigh(998);
  r.sample(0);
  INFO("bits=" << int(bits));
  CHECK(r.in.airX10 >= 1233);
  CHECK(r.in.airX10 <= 1235);
  CHECK(r.in.n2LowX100 >= 1872);
  CHECK(r.in.n2LowX100 <= 1878);
  CHECK(r.in.n2HighX10 >= 997);
  CHECK(r.in.n2HighX10 <= 999);
  CHECK(r.in.airOk);
  CHECK(r.in.n2LowOk);
  CHECK(r.in.n2HighOk);
}

TEST_CASE("INP-5: a sensor is declared faulty only after N consecutive bad samples") {
  Rig r;
  r.setAir(1000);
  r.setLow(1500);
  r.setHigh(500);
  r.hal.analogValue[pinOf(kHostBoard, Signal::kN2HighPressure)] = 0;  // wire off: 0 V
  r.sample(0);
  r.sample(30);
  CHECK(r.in.n2HighOk);  // two bad samples: not yet
  r.sample(60);
  CHECK_FALSE(r.in.n2HighOk);  // third consecutive, and bad for 60 ms (>= sensorFaultMs)
  CHECK(r.faults.active(FaultId::kN2HighSensor));
  CHECK(r.in.airOk);
  CHECK(r.in.n2LowOk);
}

TEST_CASE("INP-5: one good sample resets the bad-sample count") {
  Rig r;
  r.setAir(1000);
  r.setLow(1500);
  const uint16_t good = rawFor(500, kN2HighFullScaleX10);
  const uint16_t pin = pinOf(kHostBoard, Signal::kN2HighPressure);
  for (int i = 0; i < 10; ++i) {
    r.hal.analogValue[pin] = (i % 3 == 2) ? good : 0;  // bad, bad, good, ...
    r.sample(i * 10);
    CHECK(r.in.n2HighOk);
  }
}

TEST_CASE("INP-4: a disconnected sensor is a fault, not 0 PSI (the V7 failure)") {
  Rig r;
  r.setAir(1000);
  r.setLow(1500);
  r.hal.analogValue[pinOf(kHostBoard, Signal::kN2HighPressure)] = 0;
  for (int i = 0; i < 3; ++i) r.sample(i * 30);
  CHECK(r.in.n2HighX10 == 0);  // the scaled value alone would look like an empty tank
  CHECK_FALSE(r.in.n2HighOk);  // ...so the validity flag is what the controllers must use
}

TEST_CASE("INP-4: a shorted high reading is also a fault") {
  Rig r;
  r.setAir(1000);
  r.setLow(1500);
  r.hal.analogValue[pinOf(kHostBoard, Signal::kN2HighPressure)] = static_cast<uint16_t>(adcMaxRaw(r.bits));
  for (int i = 0; i < 3; ++i) r.sample(i * 30);
  CHECK_FALSE(r.in.n2HighOk);
}

TEST_CASE("FLT-1: a recovered sensor stays 'not ok' until the clear hold has passed") {
  Rig r;
  r.setAir(1000);
  r.setLow(1500);
  const uint16_t pin = pinOf(kHostBoard, Signal::kN2HighPressure);
  r.hal.analogValue[pin] = 0;
  for (int i = 0; i < 3; ++i) r.sample(i * 30);
  REQUIRE_FALSE(r.in.n2HighOk);
  r.setHigh(500);
  r.sample(100);
  CHECK_FALSE(r.in.n2HighOk);
  r.sample(100 + r.cfg.faultHoldMs - 1);
  CHECK_FALSE(r.in.n2HighOk);
  r.sample(100 + r.cfg.faultHoldMs);
  CHECK(r.in.n2HighOk);
}

TEST_CASE("INP-7: N2-low above N2-high for the hold time raises F04") {
  Rig r;
  r.setAir(1300);
  r.setHigh(300);   // 30.0 PSI
  r.setLow(2500);   // 25.00 PSI: fine, below 30.0
  r.sample(0);
  CHECK_FALSE(r.in.sensorOrderFault);
  r.setLow(2900);   // 29.00 PSI vs 30.0 PSI: still lower
  r.sample(10);
  CHECK_FALSE(r.in.sensorOrderFault);
  r.setHigh(200);   // 20.0 PSI: now N2-low (29.00) is above N2-high
  r.sample(20);
  CHECK_FALSE(r.in.sensorOrderFault);                   // not yet: hold time
  r.sample(20 + r.cfg.sensorOrderHoldMs - 1);
  CHECK_FALSE(r.in.sensorOrderFault);
  r.sample(20 + r.cfg.sensorOrderHoldMs);
  CHECK(r.in.sensorOrderFault);
  CHECK(r.faults.active(FaultId::kSensorOrder));
}

TEST_CASE("INP-7: two sensors both near zero do not trip F04 (margin)") {
  Rig r;
  r.setAir(1300);
  r.setLow(40);    // 0.40 PSI
  r.setHigh(0);    // reads 0.0 PSI at 0.5 V
  for (uint32_t t = 0; t < 20000; t += 100) r.sample(t);
  CHECK_FALSE(r.in.sensorOrderFault);
}

TEST_CASE("INP-7: the check is skipped when either sensor is faulty") {
  Rig r;
  r.setAir(1300);
  r.setLow(2900);
  r.hal.analogValue[pinOf(kHostBoard, Signal::kN2HighPressure)] = 0;  // faulty high sensor reads 0
  for (uint32_t t = 0; t < 20000; t += 100) r.sample(t);
  CHECK_FALSE(r.in.n2HighOk);
  CHECK_FALSE(r.in.sensorOrderFault);  // F03 explains it; F04 would be a false second alarm
}

TEST_CASE("INP-5: a few milliseconds of bad readings (a solenoid switching spike) is NOT a fault; 50 ms of it is") {
  Rig r;
  r.setAir(1000);
  r.setLow(1500);
  r.setHigh(500);
  const uint16_t pin = pinOf(kHostBoard, Signal::kN2HighPressure);
  r.hal.analogValue[pin] = 70;                       // the dip seen on the bench: raw 70 = 0.34 V
  for (uint32_t t = 0; t < 6; ++t) r.sample(t);      // six consecutive bad samples, 5 ms in all
  CHECK(r.in.n2HighOk);
  r.setHigh(500);
  r.sample(6);                                       // back to normal
  r.hal.analogValue[pin] = 70;
  for (uint32_t t = 100; t <= 160; t += 5) r.sample(t);   // bad for 60 ms
  CHECK_FALSE(r.in.n2HighOk);
}
