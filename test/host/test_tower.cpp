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
  Inputs in = goodInputs(r.cfg.airGraceFromOffMs + 5);   // (inside the grace after OFF -> LEFT, low air is tolerated: see "INV-2 grace")
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
  in.ms = r.cfg.airGraceFromOffMs + 5;   // after the grace that follows OFF -> LEFT
  r.tower.update(in);
  REQUIRE(r.tower.state() == Tower::State::kDisabled);
  in.airX10 = (r.cfg.airLowOff + r.cfg.airLowOn) / 2;
  in.ms = r.cfg.airGraceFromOffMs + 15;
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
  CHECK(r.log.lines[1] == "60250+59250 TWR L->LB next:60750");
  r.tower.disable(61100);
  CHECK(r.log.lines.back() == "61100+850 TWR LB->OF next:-");
}

// ---- air sag grace (owner 2026-10-09): the supply sags ~35 PSI for ~300 ms when a valve opens and recovers in ~2 s
TEST_CASE("INV-2 grace: low air right after OFF -> LEFT is tolerated for 3000 ms, then the tower stops") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  REQUIRE(r.tower.state() == Tower::State::kLeft);
  Inputs sag = goodInputs(0);
  sag.airX10 = r.cfg.airLowOff - 50;            // below the limit
  for (uint32_t t = 10; t < 2990; t += 10) { sag.ms = t; r.tower.update(sag); }
  CHECK(r.tower.state() == Tower::State::kLeft); // still open at 2.99 s
  sag.ms = 3010; r.tower.update(sag);
  CHECK(r.tower.state() == Tower::State::kDisabled);   // the grace has ended and the air is still low
}

TEST_CASE("INV-2 grace: low air after LEFT -> BOTH and RIGHT -> BOTH is tolerated for 2000 ms") {
  Rig r;
  r.cfg.towerFillMs = 59250;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  uint32_t t = 0;
  while (r.tower.state() != Tower::State::kLeftBoth && t < 70000) { t += 10; r.tower.update(goodInputs(t)); }
  REQUIRE(r.tower.state() == Tower::State::kLeftBoth);
  const uint32_t bothAt = t;
  Inputs sag = goodInputs(t);
  sag.airX10 = r.cfg.airLowOff - 50;
  for (uint32_t k = 10; k < 1990; k += 10) { sag.ms = bothAt + k; r.tower.update(sag); }
  CHECK(r.tower.state() != Tower::State::kDisabled);     // inside the 2 s grace
  sag.ms = bothAt + 2010; r.tower.update(sag);
  CHECK(r.tower.state() == Tower::State::kDisabled);
}

TEST_CASE("INV-2 grace: a SENSOR fault is never tolerated, grace or not") {
  Rig r;
  r.tower.enable();
  r.tower.update(goodInputs(0));
  REQUIRE(r.tower.state() == Tower::State::kLeft);
  Inputs bad = goodInputs(20);
  bad.airOk = false;
  r.tower.update(bad);
  CHECK(r.tower.state() == Tower::State::kDisabled);
}

// ---- TWR-OV: the overlap ends when the air has passed its minimum --------------------------------------------------
namespace {
// Drives the tower into LEFT_BOTH and then feeds the air curve `air(dt)` (PSI x10, dt ms since the overlap began) every 5 ms. `length` = the overlap
// length in ms (the cap if it was never ended early).
struct OverlapRun {
  Rig r;
  uint32_t t = 0;
  uint32_t length = 0;
  template <typename F>
  void run(F air) {
    r.cfg.overlapAdaptive = true;   // the adaptive rule is OFF in the default config (fixed 500 ms): these tests switch it on
    r.tower.enable();
    r.tower.update(goodInputs(t));
    t += r.cfg.towerFillMs;
    r.tower.update(goodInputs(t));               // LEFT -> LEFT_BOTH
    REQUIRE(r.tower.state() == Tower::State::kLeftBoth);
    const uint32_t start = t;
    for (uint32_t dt = 5; dt < 2000 && r.tower.state() == Tower::State::kLeftBoth; dt += 5) {
      Inputs in = goodInputs(start + dt);
      in.airX10 = air(dt);
      r.tower.update(in);
      length = dt;
    }
  }
};
}  // namespace

TEST_CASE("TWR-OV-1: the overlap ends shortly after the air minimum, well before the cap") {
  OverlapRun o;
  o.run([](uint32_t dt) -> uint16_t {          // 1040 -> falls to 800 at 450 ms -> climbs back to 1000 by 2.4 s
    if (dt < 450) return static_cast<uint16_t>(1040 - (240 * dt) / 450);
    return static_cast<uint16_t>(800 + (dt - 450) / 5);   // 1 PSI per 50 ms recovery
  });
  CHECK(o.r.tower.state() == Tower::State::kRight);
  CHECK(o.length >= 450);                       // not before the minimum
  CHECK(o.length <= 700);                       // soon after it (cap is 750)
}

TEST_CASE("TWR-OV-2: one rippled low sample early in the dip is not taken as the minimum") {
  OverlapRun o;
  o.run([](uint32_t dt) -> uint16_t {
    uint16_t v = dt < 450 ? static_cast<uint16_t>(1040 - (240 * dt) / 450) : static_cast<uint16_t>(800 + (dt - 450) / 5);
    if (dt >= 50 && dt < 60) v = 900;           // a 10 ms spike low in the middle of the fall (50 ms grid: at most one sample)
    return v;
  });
  CHECK(o.length >= 450);
}

TEST_CASE("TWR-OV-3: with flat air (no sag seen) the cap ends the overlap, as before") {
  OverlapRun o;
  o.run([](uint32_t) -> uint16_t { return 1000; });
  CHECK(o.r.tower.state() == Tower::State::kRight);
  CHECK(o.length >= o.r.cfg.towerOverlapMs);
  CHECK(o.length <= o.r.cfg.towerOverlapMs + 5);
}

TEST_CASE("TWR-OV-3: the overlap is never shorter than overlapMinMs") {
  OverlapRun o;
  o.run([](uint32_t dt) -> uint16_t { return dt < 60 ? static_cast<uint16_t>(1040 - dt * 4) : static_cast<uint16_t>(800 + (dt - 60) * 3); });   // minimum at 60 ms
  CHECK(o.length >= 200);
}

TEST_CASE("TWR-OV-8: by default the overlap is the fixed 500 ms (Tom's decision 2026-10-10), whatever the air does") {
  Rig r;
  CHECK_FALSE(r.cfg.overlapAdaptive);
  CHECK(r.cfg.towerOverlapMs == 500);
  r.tower.enable();
  uint32_t t = 0;
  r.tower.update(goodInputs(t));
  t += r.cfg.towerFillMs;
  r.tower.update(goodInputs(t));                 // LEFT -> LEFT_BOTH
  REQUIRE(r.tower.state() == Tower::State::kLeftBoth);
  const uint32_t start = t;
  uint32_t length = 0;
  for (uint32_t dt = 5; dt < 2000 && r.tower.state() == Tower::State::kLeftBoth; dt += 5) {
    Inputs in = goodInputs(start + dt);
    in.airX10 = dt < 100 ? static_cast<uint16_t>(1040 - 2 * dt) : static_cast<uint16_t>(840 + (dt - 100) / 2);   // a dip whose bottom is at 100 ms
    r.tower.update(in);
    length = dt;
  }
  CHECK(length >= 500);
  CHECK(length <= 505);
}
