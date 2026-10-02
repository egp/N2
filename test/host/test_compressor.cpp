// Tests for Compressor: §7.2, RST-4, INV-3, INV-4, INV-8.
#include <catch2/catch_test_macros.hpp>

#include "TestSupport.h"
#include "core/Compressor.h"

using namespace n2;
using namespace n2::test;

namespace {
Inputs goodInputs(uint32_t ms = 0) {
  Inputs in;
  in.ms = ms;
  in.tbs = true;
  in.airX10 = kHealthyAirX10;
  in.n2LowX100 = kHealthyN2LowX100;
  in.n2HighX10 = kHealthyN2HighX10;
  in.o2CommOk = true;
  in.o2Warm = true;
  return in;
}
struct Rig {
  VectorLog log;
  TransitionLogger tl{log};
  ControlConfig cfg = kDefaultControl;
  Compressor c{cfg, tl};
};
}  // namespace

TEST_CASE("RST-4: enable() picks RUNNING when both pressures are in range") {
  Rig r;
  r.c.enable(goodInputs());
  CHECK(r.c.state() == Compressor::State::kRunning);
  CHECK(r.c.ssrOn());
}

TEST_CASE("RST-4: enable() with N2-high not below ON starts STOPPED_HIGH") {
  Rig r;
  Inputs in = goodInputs();
  in.n2HighX10 = r.cfg.n2HighOn;  // equal: not recovered
  r.c.enable(in);
  CHECK(r.c.state() == Compressor::State::kStoppedHigh);
  CHECK_FALSE(r.c.ssrOn());
}

TEST_CASE("RST-4: enable() with N2-low not above ON starts STOPPED_LOW") {
  Rig r;
  Inputs in = goodInputs();
  in.n2LowX100 = r.cfg.n2LowOn;  // equal: not recovered
  r.c.enable(in);
  CHECK(r.c.state() == Compressor::State::kStoppedLow);
}

TEST_CASE("RST-4: enable() with a faulty sensor never starts RUNNING") {
  Rig r1;
  Inputs in = goodInputs();
  in.n2HighOk = false;
  r1.c.enable(in);
  CHECK(r1.c.state() == Compressor::State::kStoppedHigh);

  Rig r2;
  in = goodInputs();
  in.n2LowOk = false;
  r2.c.enable(in);
  CHECK(r2.c.state() == Compressor::State::kStoppedLow);

  Rig r3;
  in = goodInputs();
  in.sensorOrderFault = true;
  r3.c.enable(in);
  CHECK(r3.c.state() == Compressor::State::kStoppedHigh);
}

TEST_CASE("RST-4/V7 fix: an over-pressure tank does not get a one-pass SSR pulse on enable") {
  Rig r;
  Inputs in = goodInputs();
  in.n2HighX10 = r.cfg.n2HighOff + 50;  // tank over its limit
  r.c.enable(in);
  CHECK_FALSE(r.c.ssrOn());
  r.c.update(in);
  CHECK_FALSE(r.c.ssrOn());
}

TEST_CASE("INV-4: RUNNING stops when N2-low falls below OFF") {
  Rig r;
  r.c.enable(goodInputs());
  Inputs in = goodInputs(10);
  in.n2LowX100 = r.cfg.n2LowOff - 1;
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kStoppedLow);
}

TEST_CASE("INV-3: RUNNING stops when N2-high exceeds OFF") {
  Rig r;
  r.c.enable(goodInputs());
  Inputs in = goodInputs(10);
  in.n2HighX10 = r.cfg.n2HighOff + 1;
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kStoppedHigh);
}

TEST_CASE("§7.2: STOPPED_LOW restarts only above the ON threshold (hysteresis)") {
  Rig r;
  Inputs in = goodInputs();
  in.n2LowX100 = r.cfg.n2LowOff - 1;
  r.c.enable(in);
  REQUIRE(r.c.state() == Compressor::State::kStoppedLow);
  in.n2LowX100 = (r.cfg.n2LowOff + r.cfg.n2LowOn) / 2;  // between OFF and ON
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kStoppedLow);
  in.n2LowX100 = r.cfg.n2LowOn + 1;
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kRunning);
}

TEST_CASE("§7.2: STOPPED_HIGH restarts only below the ON threshold") {
  Rig r;
  Inputs in = goodInputs();
  in.n2HighX10 = r.cfg.n2HighOff + 1;
  r.c.enable(in);
  REQUIRE(r.c.state() == Compressor::State::kStoppedHigh);
  in.n2HighX10 = (r.cfg.n2HighOn + r.cfg.n2HighOff) / 2;
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kStoppedHigh);
  in.n2HighX10 = r.cfg.n2HighOn - 1;
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kRunning);
}

TEST_CASE("§7.2 cross-check: STOPPED_LOW does not restart while N2-high is over its limit") {
  Rig r;
  Inputs in = goodInputs();
  in.n2LowX100 = r.cfg.n2LowOff - 1;
  r.c.enable(in);
  REQUIRE(r.c.state() == Compressor::State::kStoppedLow);
  in.n2LowX100 = r.cfg.n2LowOn + 100;  // low recovered...
  in.n2HighX10 = r.cfg.n2HighOff + 1;  // ...but high is over
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kStoppedLow);
  CHECK_FALSE(r.c.ssrOn());
}

TEST_CASE("§7.2 cross-check: STOPPED_HIGH does not restart while N2-low is under its limit") {
  Rig r;
  Inputs in = goodInputs();
  in.n2HighX10 = r.cfg.n2HighOff + 1;
  r.c.enable(in);
  REQUIRE(r.c.state() == Compressor::State::kStoppedHigh);
  in.n2HighX10 = r.cfg.n2HighOn - 1;   // high recovered...
  in.n2LowX100 = r.cfg.n2LowOff - 1;   // ...but low is under
  r.c.update(in);
  CHECK(r.c.state() == Compressor::State::kStoppedHigh);
}

TEST_CASE("INV-3/INV-4/INV-8: sensor faults stop a running compressor") {
  Inputs bad;
  {
    Rig r;
    r.c.enable(goodInputs());
    bad = goodInputs(10);
    bad.n2HighOk = false;
    r.c.update(bad);
    CHECK_FALSE(r.c.ssrOn());
  }
  {
    Rig r;
    r.c.enable(goodInputs());
    bad = goodInputs(10);
    bad.n2LowOk = false;
    r.c.update(bad);
    CHECK_FALSE(r.c.ssrOn());
  }
  {
    Rig r;
    r.c.enable(goodInputs());
    bad = goodInputs(10);
    bad.sensorOrderFault = true;
    r.c.update(bad);
    CHECK_FALSE(r.c.ssrOn());
  }
}

TEST_CASE("§7.2: disable() turns the SSR request off; DISABLED never starts by itself") {
  Rig r;
  r.c.enable(goodInputs());
  REQUIRE(r.c.ssrOn());
  r.c.disable(50);
  CHECK(r.c.state() == Compressor::State::kDisabled);
  CHECK_FALSE(r.c.ssrOn());
  r.c.update(goodInputs(60));
  CHECK(r.c.state() == Compressor::State::kDisabled);
}

TEST_CASE("ARC-7: compressor transitions are logged") {
  Rig r;
  r.c.enable(goodInputs(500));
  CHECK(r.log.lines.back() == "500+500 CMP OF->ON next:-");
  Inputs in = goodInputs(900);
  in.n2LowX100 = r.cfg.n2LowOff - 1;
  r.c.update(in);
  CHECK(r.log.lines.back() == "900+400 CMP ON->LO next:-");
}
