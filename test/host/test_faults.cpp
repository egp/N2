// Tests for Faults: FLT-1..FLT-4.
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <set>

#include "TestSupport.h"
#include "core/Faults.h"

using namespace n2;
using namespace n2::test;

TEST_CASE("FLT-1: a true condition raises the fault at once") {
  VectorLog log;
  FaultSet f(log);
  CHECK_FALSE(f.active(FaultId::kAirSensor));
  f.report(FaultId::kAirSensor, true, 100, 5000);
  CHECK(f.active(FaultId::kAirSensor));
}

TEST_CASE("FLT-1: a fault clears only after its condition has been false for the hold time") {
  VectorLog log;
  FaultSet f(log);
  f.report(FaultId::kN2LowSensor, true, 0, 5000);
  f.report(FaultId::kN2LowSensor, false, 1000, 5000);
  CHECK(f.active(FaultId::kN2LowSensor));
  f.report(FaultId::kN2LowSensor, false, 5999, 5000);
  CHECK(f.active(FaultId::kN2LowSensor));
  f.report(FaultId::kN2LowSensor, false, 6000, 5000);
  CHECK_FALSE(f.active(FaultId::kN2LowSensor));
}

TEST_CASE("FLT-1: the hold timer restarts if the condition returns") {
  VectorLog log;
  FaultSet f(log);
  f.report(FaultId::kAirSensor, true, 0, 5000);
  f.report(FaultId::kAirSensor, false, 1000, 5000);
  f.report(FaultId::kAirSensor, true, 3000, 5000);   // glitch back
  f.report(FaultId::kAirSensor, false, 4000, 5000);  // hold restarts here
  f.report(FaultId::kAirSensor, false, 8999, 5000);
  CHECK(f.active(FaultId::kAirSensor));
  f.report(FaultId::kAirSensor, false, 9000, 5000);
  CHECK_FALSE(f.active(FaultId::kAirSensor));
}

TEST_CASE("FLT-1: the invariant fault latches until reset") {
  VectorLog log;
  FaultSet f(log);
  f.report(FaultId::kInvariant, true, 0, 5000);
  f.report(FaultId::kInvariant, false, 1000, 5000);
  f.report(FaultId::kInvariant, false, 600000, 5000);
  CHECK(f.active(FaultId::kInvariant));
}

TEST_CASE("FLT-1: hold-time arithmetic survives millis() rollover") {
  VectorLog log;
  FaultSet f(log);
  const uint32_t t0 = UINT32_MAX - 1000;
  f.report(FaultId::kAirSensor, true, t0, 5000);
  f.report(FaultId::kAirSensor, false, t0 + 100, 5000);
  f.report(FaultId::kAirSensor, false, t0 + 4000, 5000);  // wrapped
  CHECK(f.active(FaultId::kAirSensor));
  f.report(FaultId::kAirSensor, false, t0 + 5100, 5000);
  CHECK_FALSE(f.active(FaultId::kAirSensor));
}

TEST_CASE("FLT-2: raising and clearing each log one line with the code") {
  VectorLog log;
  FaultSet f(log);
  f.report(FaultId::kN2HighSensor, true, 10, 100);
  f.report(FaultId::kN2HighSensor, true, 20, 100);  // already active: no second line
  CHECK(log.count("F03 raised") == 1);
  f.report(FaultId::kN2HighSensor, false, 30, 100);
  f.report(FaultId::kN2HighSensor, false, 130, 100);
  CHECK(log.count("F03 cleared") == 1);
}

TEST_CASE("FLT-3: counting and listing respects severity") {
  VectorLog log;
  FaultSet f(log);
  f.report(FaultId::kLcd, true, 0, 1);            // INFO
  f.report(FaultId::kWatchdogReset, true, 0, 1);  // WARN
  f.report(FaultId::kAirSensor, true, 0, 1);      // INHIBIT
  CHECK(f.activeCount() == 3);
  CHECK(f.activeCount(Severity::kWarn) == 2);
  CHECK(f.activeCount(Severity::kInhibit) == 1);
  CHECK(f.anyAtLeast(Severity::kInhibit));
  FaultId id;
  REQUIRE(f.nth(0, Severity::kWarn, id));
  CHECK(id == FaultId::kAirSensor);  // table order
  REQUIRE(f.nth(1, Severity::kWarn, id));
  CHECK(id == FaultId::kWatchdogReset);
  CHECK_FALSE(f.nth(2, Severity::kWarn, id));
}

TEST_CASE("FLT-4: the fault table is complete, with unique codes and screen-sized text") {
  std::set<int> codes;
  for (uint8_t i = 0; i < kFaultCount; ++i) {
    const FaultInfo& info = faultInfo(static_cast<FaultId>(i));
    INFO("fault index " << int(i));
    CHECK(codes.insert(info.code).second);
    CHECK(std::strlen(info.text) > 0);
    CHECK(std::strlen(info.text) <= 16);   // "Fnn " + text must fit 20 columns
    CHECK(std::strlen(info.effect) > 0);
    CHECK(std::strlen(info.effect) <= 20);
  }
  CHECK(faultInfo(FaultId::kAirSensor).code == 1);
  CHECK(faultInfo(FaultId::kSensorOrder).code == 4);
  CHECK(faultInfo(FaultId::kO2Comm).code == 12);
  CHECK(faultInfo(FaultId::kInvariant).latching);
  CHECK(faultInfo(FaultId::kO2Comm).severity == Severity::kInhibit);
  CHECK(faultInfo(FaultId::kRtc).code == 13);
  CHECK(faultInfo(FaultId::kRtc).severity == Severity::kInfo);  // the clock must never affect control
  CHECK(faultInfo(FaultId::kSensorOrder).severity == Severity::kInhibit);
}
