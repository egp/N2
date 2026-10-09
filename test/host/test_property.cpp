// Randomized property tests: INV-1..INV-4, INV-8..INV-10, INV-6, OUT-1 (Requirements INV-7).
//
// Random sensors, switch, O2 faults and time steps are driven through the whole system. After EVERY
// step the ACTUAL pin states are checked against the safety rules, which are restated here
// independently from the requirements (not by calling checkInvariants()).
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "TestSupport.h"

using namespace n2;
using namespace n2::test;

namespace {

// xorshift32: small, deterministic, no <random> state differences between platforms.
struct Rng {
  uint32_t s;
  explicit Rng(uint32_t seed) : s(seed ? seed : 1) {}
  uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
  uint32_t below(uint32_t n) { return next() % n; }
  bool chance(uint32_t percentTimes100) { return below(10000) < percentTimes100; }  // 100 = 1 %
};

struct Tracker {
  uint32_t lastChange[4];
  bool state[4];
  void init(uint32_t now) { for (int i = 0; i < 4; ++i) { lastChange[i] = now; state[i] = false; } }
};

void checkSafety(Generator& p, const ControlConfig& cfg, Tracker& tr, uint32_t stepNo) {
  const Inputs& in = p.sys().inputs();
  const bool L = p.left(), R = p.right(), F = p.flush(), S = p.ssr();
  INFO("step " << stepNo << " t=" << p.hal.nowMs << " air=" << in.airX10 << "/" << in.airOk << " low=" << in.n2LowX100
                << "/" << in.n2LowOk << " high=" << in.n2HighX10 << "/" << in.n2HighOk << " tbs=" << in.tbs
                << " o2=" << in.o2CommOk << "/" << in.o2Warm << " F04=" << in.sensorOrderFault);

  if (!in.tbs) { REQUIRE_FALSE(L); REQUIRE_FALSE(R); REQUIRE_FALSE(F); REQUIRE_FALSE(S); }                       // INV-1
  if (!in.airOk || (in.airX10 < cfg.airLowOff && !in.airGrace)) { REQUIRE_FALSE(L); REQUIRE_FALSE(R); }         // INV-2 (low air is tolerated only inside the grace after a valve opens)
  if (!in.n2HighOk || in.n2HighX10 > cfg.n2HighOff || in.sensorOrderFault) { REQUIRE_FALSE(L); REQUIRE_FALSE(R); REQUIRE_FALSE(S); }  // INV-3, INV-8
  if (!in.n2LowOk || in.n2LowX100 < cfg.n2LowOff || in.sensorOrderFault) { REQUIRE_FALSE(S); }                // INV-4, INV-8
  if (cfg.o2Mandatory && !in.o2CommOk) { REQUIRE_FALSE(L); REQUIRE_FALSE(R); REQUIRE_FALSE(F); REQUIRE_FALSE(S); }  // INV-9
  if (cfg.o2Mandatory && !in.o2Warm) { REQUIRE_FALSE(L); REQUIRE_FALSE(R); }                                   // INV-10

  REQUIRE(p.sys().invariantViolations() == 0);  // INV-6: no controller ever asked for a forbidden output

  // OUT-1: an output never switches ON within the minimum hold of its previous change.
  const bool now[4] = {L, R, F, S};
  for (int i = 0; i < 4; ++i) {
    if (now[i] != tr.state[i]) {
      if (now[i]) REQUIRE(static_cast<uint32_t>(p.hal.nowMs - tr.lastChange[i]) >= cfg.outputMinHoldMs);
      tr.state[i] = now[i];
      tr.lastChange[i] = p.hal.nowMs;
    }
  }
  // A running tower always has a valve open (the overlap logic never leaves both closed).
  if (p.sys().tower().state() != Tower::State::kDisabled) {
    REQUIRE((p.sys().tower().leftOpen() || p.sys().tower().rightOpen()));
  }
}

void randomizeInputs(Generator& p, Rng& rng) {
  const AdcWindow w = adcWindow(p.bits);
  const uint8_t pins[3] = {pinOf(kHostBoard, Signal::kAirPressure), pinOf(kHostBoard, Signal::kN2LowPressure),
                           pinOf(kHostBoard, Signal::kN2HighPressure)};
  for (uint8_t pin : pins) {
    if (!rng.chance(1500)) continue;  // change a sensor ~15 % of steps
    const uint32_t pick = rng.below(100);
    if (pick < 70) p.hal.analogValue[pin] = static_cast<uint16_t>(w.validMin + rng.below(w.validMax - w.validMin + 1u));
    else if (pick < 80) p.hal.analogValue[pin] = static_cast<uint16_t>(rng.below(w.faultMin));       // wire off / short low
    else if (pick < 85) p.hal.analogValue[pin] = static_cast<uint16_t>(w.faultMax + 1 + rng.below(adcMaxRaw(p.bits) - w.faultMax));  // short high
    else p.hal.analogValue[pin] = static_cast<uint16_t>(w.validMin + rng.below(w.validMax - w.validMin + 1u));
  }
  if (rng.chance(200)) p.tbs(!(p.hal.inputLevel[pinOf(kHostBoard, Signal::kTbs)] == false));  // toggle TBS ~2 %
  if (rng.chance(100)) p.o2.presentOk = !p.o2.presentOk;
  if (rng.chance(150)) p.o2.readOk = !p.o2.readOk;
  if (rng.chance(300)) p.o2.o2x100 = static_cast<uint16_t>(rng.below(2200));
}

uint32_t randomStep(Rng& rng) {
  const uint32_t pick = rng.below(100);
  if (pick < 85) return 5 + rng.below(200);        // normal loop pass
  if (pick < 98) return 200 + rng.below(3000);     // slow pass
  return 3000 + rng.below(40000);                  // long stall (watchdog would catch these on a device)
}

void fuzz(Generator& p, uint32_t seed, uint32_t steps, uint32_t startMs) {
  Rng rng(seed);
  p.hal.nowMs = startMs;
  p.sys().begin();
  Tracker tr;
  tr.init(p.hal.nowMs);
  p.tbs(true);
  for (uint32_t i = 0; i < steps; ++i) {
    randomizeInputs(p, rng);
    p.step(randomStep(rng));
    checkSafety(p, p.cfg, tr, i);
  }
}

}  // namespace

TEST_CASE("INV-7: safety rules hold after every step of randomized runs (quick warm-up)") {
  const uint32_t seed = static_cast<uint32_t>(GENERATE(1, 2, 3, 4, 5, 6, 7, 8));
  Generator p(quickWarmConfig());
  INFO("seed " << seed);
  fuzz(p, seed * 7919u, 6000, 0);
}

TEST_CASE("INV-7: safety rules hold at every ADC bit depth") {
  const uint8_t bits = static_cast<uint8_t>(GENERATE(12, 14));
  Generator p(quickWarmConfig(), bits);
  fuzz(p, 4242u + bits, 5000, 0);
}

TEST_CASE("INV-7: safety rules hold across the millis() rollover") {
  Generator p(quickWarmConfig());
  fuzz(p, 99u, 6000, UINT32_MAX - 200000u);
}

TEST_CASE("INV-7: safety rules hold with the production 5-minute warm-up") {
  Generator p;  // default config
  fuzz(p, 31337u, 8000, 0);
}

TEST_CASE("INV-7: bench configuration (O2 not mandatory) still obeys INV-1..INV-4 and INV-8") {
  ControlConfig c = quickWarmConfig();
  c.o2Mandatory = false;
  Generator p(c);
  fuzz(p, 2024u, 6000, 0);
}

TEST_CASE("INV-7: random resets in the middle of operation never leave an output on") {
  Generator p(quickWarmConfig());
  Rng rng(555);
  p.reboot();
  p.tbs(true);
  for (int i = 0; i < 4000; ++i) {
    randomizeInputs(p, rng);
    p.step(5 + rng.below(300));
    if (rng.chance(30)) {  // reset button
      p.reboot();
      for (Signal s : {Signal::kLeftValve, Signal::kRightValve, Signal::kFlushValve, Signal::kSsr})
        REQUIRE_FALSE(p.out(s));
    }
  }
}
