// Tests for Tower: §7.1, INV-2, INV-3, INV-8, INV-10, GOAL-6, ARC-6, ARC-7.
#include <catch2/catch_test_macros.hpp>

#include "TestSupport.h"
#include "core/Tower.h"

using namespace n2;
using namespace n2::test;

namespace {
Inputs goodInputs(uint32_t ms) {
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
  Tower tower{cfg, tl};
};
}  // namespace

TEST_CASE("§7.1: a new tower is DISABLED with both valves closed") {
  Rig r;
  CHECK(r.tower.state() == Tower::State::kDisabled);
  CHECK_FALSE(r.tower.leftOpen());
  CHECK_FALSE(r.tower.rightOpen());
}

TEST_CASE("§7.1: it does not start until enable() and every start condition holds") {
  Rig r;
  r.tower.update(goodInputs(0));
  CHECK(r.tower.state() == Tower::State::kDisabled);  // not enabled yet
  r.tower.enable();
  r.tower.update(goodInputs(10));
  CHECK(r.tower.state() == Tower::State::kLeft);
}

TEST_CASE("§7.1: start conditions use the ON thresholds (hysteresis)") {
  Rig r;
  r.tower.enable();
  Inputs in = goodInputs(0);
  in.airX10 = r.cfg.airLowOn;  // equal is not "above"
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
  in.airX10 = r.cfg.airLowOn + 1;
  in.n2HighX10 = r.cfg.n2HighOn;  // equal is not "below"
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
  in.n2HighX10 = r.cfg.n2HighOn - 1;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kLeft);
}

TEST_CASE("§7.1: the timed cycle L -> LB -> R -> RB -> L with the right valves and times") {
  Rig r;
  r.tower.enable();
  uint32_t t = 1000;
  r.tower.update(goodInputs(t));
  CHECK(r.tower.state() == Tower::State::kLeft);
  CHECK(r.tower.leftOpen());
  CHECK_FALSE(r.tower.rightOpen());

  r.tower.update(goodInputs(t + r.cfg.towerFillMs - 1));
  CHECK(r.tower.state() == Tower::State::kLeft);
  t += r.cfg.towerFillMs;
  r.tower.update(goodInputs(t));
  CHECK(r.tower.state() == Tower::State::kLeftBoth);
  CHECK(r.tower.leftOpen());
  CHECK(r.tower.rightOpen());

  t += r.cfg.towerOverlapMs;
  r.tower.update(goodInputs(t));
  CHECK(r.tower.state() == Tower::State::kRight);
  CHECK_FALSE(r.tower.leftOpen());
  CHECK(r.tower.rightOpen());

  t += r.cfg.towerFillMs;
  r.tower.update(goodInputs(t));
  CHECK(r.tower.state() == Tower::State::kRightBoth);
  CHECK(r.tower.leftOpen());
  CHECK(r.tower.rightOpen());

  t += r.cfg.towerOverlapMs;
  r.tower.update(goodInputs(t));
  CHECK(r.tower.state() == Tower::State::kLeft);
}

TEST_CASE("GOAL-6: the cycle continues correctly across the millis() rollover") {
  Rig r;
  r.tower.enable();
  uint32_t t = UINT32_MAX - 30000;  // 30 s before the wrap
  r.tower.update(goodInputs(t));
  REQUIRE(r.tower.state() == Tower::State::kLeft);
  t += r.cfg.towerFillMs;  // wraps past zero
  r.tower.update(goodInputs(t));
  CHECK(r.tower.state() == Tower::State::kLeftBoth);
  t += r.cfg.towerOverlapMs;
  r.tower.update(goodInputs(t));
  CHECK(r.tower.state() == Tower::State::kRight);
}

TEST_CASE("INV-2: low air pressure closes both valves at once, mid-fill") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  Inputs in = goodInputs(5);
  in.airX10 = r.cfg.airLowOff - 1;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
  CHECK_FALSE(r.tower.leftOpen());
  CHECK_FALSE(r.tower.rightOpen());
}

TEST_CASE("INV-2: air between OFF and ON thresholds keeps running but will not restart") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  Inputs in = goodInputs(5);
  in.airX10 = (r.cfg.airLowOff + r.cfg.airLowOn) / 2;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kLeft);  // above OFF: keeps running
  in.airX10 = r.cfg.airLowOff - 1;
  r.tower.update(in);
  REQUIRE(r.tower.state() == Tower::State::kDisabled);
  in.airX10 = (r.cfg.airLowOff + r.cfg.airLowOn) / 2;
  in.ms = 10;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);  // below ON: stays off
}

TEST_CASE("INV-3: high N2 pressure stops the tower") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  Inputs in = goodInputs(5);
  in.n2HighX10 = r.cfg.n2HighOff + 1;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
}

TEST_CASE("INV-3: a faulty N2-high sensor stops the tower and prevents a start (the V7 hole)") {
  Rig r;
  r.tower.enable();
  Inputs in = goodInputs(0);
  in.n2HighOk = false;
  in.n2HighX10 = 0;  // a dead sensor reads as an empty tank
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
  r.tower.update(goodInputs(10));
  REQUIRE(r.tower.state() == Tower::State::kLeft);
  in.ms = 20;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
}

TEST_CASE("INV-2: a faulty air sensor stops the tower") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  Inputs in = goodInputs(5);
  in.airOk = false;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
}

TEST_CASE("INV-8: F04 stops the tower") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  Inputs in = goodInputs(5);
  in.sensorOrderFault = true;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
}

TEST_CASE("INV-10: with a mandatory O2 sensor the tower waits for warm-up") {
  Rig r;
  r.tower.enable();
  Inputs in = goodInputs(0);
  in.o2Warm = false;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
  in.o2Warm = true;
  in.o2CommOk = false;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
  in.o2CommOk = true;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kLeft);
  in.o2Warm = false;  // e.g. the sensor failed and restarted its warm-up
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kDisabled);
}

TEST_CASE("INV-10: without a mandatory O2 sensor (bench) the tower ignores O2") {
  Rig r;
  r.cfg.o2Mandatory = false;
  r.tower.enable();
  Inputs in = goodInputs(0);
  in.o2Warm = false;
  in.o2CommOk = false;
  r.tower.update(in);
  CHECK(r.tower.state() == Tower::State::kLeft);
}

TEST_CASE("§7.1: disable() closes the valves immediately and stops restarts until enable()") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  REQUIRE(r.tower.leftOpen());
  r.tower.disable(100);
  CHECK(r.tower.state() == Tower::State::kDisabled);
  CHECK_FALSE(r.tower.leftOpen());
  r.tower.update(goodInputs(110));
  CHECK(r.tower.state() == Tower::State::kDisabled);
  r.tower.enable();
  r.tower.update(goodInputs(120));
  CHECK(r.tower.state() == Tower::State::kLeft);
}

TEST_CASE("ARC-7: every transition logs one line with timestamp, delta, names and next deadline") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(1000));
  REQUIRE(r.log.lines.size() == 1);
  CHECK(r.log.lines[0] == "1000+1000 TWR OF->L next:60250");
  r.tower.update(goodInputs(1000 + r.cfg.towerFillMs));
  REQUIRE(r.log.lines.size() == 2);
  CHECK(r.log.lines[1] == "60250+59250 TWR L->LB next:61000");
  r.tower.disable(61100);
  CHECK(r.log.lines.back() == "61100+850 TWR LB->OF next:-");
}
