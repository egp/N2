// Tests for OutputDriver: ARC-8, OUT-1, INV-5, PIN-6.
#include <catch2/catch_test_macros.hpp>

#include "TestSupport.h"
#include "core/OutputDriver.h"

using namespace n2;
using namespace n2::test;

namespace {
struct Rig {
  FakeHal hal;
  VectorLog log;
  OutputDriver drv{hal, kHostBoard, log, 1000};
  bool pinHigh(Signal s) { return hal.level[pinOf(kHostBoard, s)] == 1; }
};
ForceOff none() { return ForceOff(); }
}  // namespace

TEST_CASE("ARC-8: begin() drives every output to its off level (active-HIGH: LOW)") {
  Rig r;
  r.drv.begin(0);
  for (Signal s : {Signal::kLeftValve, Signal::kRightValve, Signal::kFlushValve, Signal::kSsr}) {
    CHECK_FALSE(r.pinHigh(s));
    CHECK_FALSE(r.drv.actual(s));
  }
}

TEST_CASE("OUT-1: nothing can switch on within the minimum hold of boot") {
  Rig r;
  r.drv.begin(5000);
  r.drv.apply({true, false, false, true}, none(), 5100);
  CHECK_FALSE(r.drv.actual(Signal::kLeftValve));
  CHECK_FALSE(r.drv.actual(Signal::kSsr));
  CHECK(r.drv.deferredCount() >= 2);
  r.drv.apply({true, false, false, true}, none(), 5999);
  CHECK_FALSE(r.drv.actual(Signal::kSsr));
  r.drv.apply({true, false, false, true}, none(), 6000);
  CHECK(r.drv.actual(Signal::kLeftValve));
  CHECK(r.drv.actual(Signal::kSsr));
  CHECK(r.pinHigh(Signal::kSsr));
}

TEST_CASE("OUT-1: a non-safety change is deferred until the hold has passed (both directions)") {
  Rig r;
  r.drv.begin(0);
  r.drv.apply({false, false, false, true}, none(), 2000);  // on at 2000
  REQUIRE(r.drv.actual(Signal::kSsr));
  r.drv.apply({}, none(), 2500);                           // asked off, but only 500 ms on
  CHECK(r.drv.actual(Signal::kSsr));
  r.drv.apply({}, none(), 3000);
  CHECK_FALSE(r.drv.actual(Signal::kSsr));
}

TEST_CASE("INV-5: a forced-off output turns off at once, ignoring the minimum hold") {
  Rig r;
  r.drv.begin(0);
  r.drv.apply({true, true, false, true}, none(), 1000);
  REQUIRE(r.drv.actual(Signal::kSsr));
  ForceOff f;
  f.ssr = true;
  f.left = true;
  r.drv.apply({true, true, false, true}, f, 1001);  // 1 ms after switching on
  CHECK_FALSE(r.drv.actual(Signal::kSsr));
  CHECK_FALSE(r.drv.actual(Signal::kLeftValve));
  CHECK(r.drv.actual(Signal::kRightValve));  // not forced: unaffected
  CHECK_FALSE(r.pinHigh(Signal::kSsr));
}

TEST_CASE("INV-5: a forced output cannot be switched ON even if requested") {
  Rig r;
  r.drv.begin(0);
  ForceOff f;
  f.ssr = true;
  r.drv.apply({false, false, false, true}, f, 5000);
  CHECK_FALSE(r.drv.actual(Signal::kSsr));
  CHECK_FALSE(r.pinHigh(Signal::kSsr));
}

TEST_CASE("OUT-1: the hold timer restarts at every change") {
  Rig r;
  r.drv.begin(0);
  r.drv.apply({true, false, false, false}, none(), 1000);
  ForceOff f;
  f.left = true;
  r.drv.apply({true, false, false, false}, f, 1200);  // safety off at 1200
  CHECK_FALSE(r.drv.actual(Signal::kLeftValve));
  r.drv.apply({true, false, false, false}, none(), 1500);  // only 300 ms since the off
  CHECK_FALSE(r.drv.actual(Signal::kLeftValve));
  r.drv.apply({true, false, false, false}, none(), 2200);
  CHECK(r.drv.actual(Signal::kLeftValve));
}

TEST_CASE("OUT-1: hold arithmetic survives millis() rollover") {
  Rig r;
  r.drv.begin(UINT32_MAX - 200);
  r.drv.apply({false, false, false, true}, none(), UINT32_MAX - 100);
  CHECK_FALSE(r.drv.actual(Signal::kSsr));
  r.drv.apply({false, false, false, true}, none(), 900);  // 1101 ms later, wrapped
  CHECK(r.drv.actual(Signal::kSsr));
}

TEST_CASE("OUT-1: a deferral is logged once per pending change, not every pass") {
  Rig r;
  r.drv.begin(0);
  for (uint32_t t = 10; t < 900; t += 10) r.drv.apply({true, false, false, false}, none(), t);
  CHECK(r.log.count("deferred") == 1);
}

TEST_CASE("PIN-6: outputs honour an active-LOW table") {
  std::array<SignalDef, kSignalCount> sigs;
  for (uint8_t i = 0; i < kSignalCount; ++i) {
    sigs[i] = kHostBoard.signals[i];
    if (sigs[i].dir == Dir::kOutput) sigs[i].active = Active::kLow;
  }
  BoardDef lowBoard = kHostBoard;
  lowBoard.signals = sigs.data();

  FakeHal hal;
  VectorLog log;
  OutputDriver drv(hal, lowBoard, log, 1000);
  drv.begin(0);
  const uint8_t ssrPin = pinOf(lowBoard, Signal::kSsr);
  CHECK(hal.level[ssrPin] == 1);  // off = HIGH
  drv.apply({false, false, false, true}, ForceOff(), 1000);
  CHECK(drv.actual(Signal::kSsr));
  CHECK(hal.level[ssrPin] == 0);  // on = LOW
}

TEST_CASE("BIST: forceDrive ignores the minimum hold (the documented exception)") {
  Rig r;
  r.drv.begin(0);
  r.drv.forceDrive(Signal::kSsr, true, 10);
  CHECK(r.drv.actual(Signal::kSsr));
  r.drv.forceDrive(Signal::kSsr, false, 510);
  CHECK_FALSE(r.drv.actual(Signal::kSsr));
}

TEST_CASE("ARC-8: actualState() reports the four driven outputs") {
  Rig r;
  r.drv.begin(0);
  r.drv.apply({true, false, true, false}, none(), 1000);
  const OutputRequest a = r.drv.actualState();
  CHECK(a.left);
  CHECK_FALSE(a.right);
  CHECK(a.flush);
  CHECK_FALSE(a.ssr);
}
