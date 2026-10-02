// Whole-system scenarios on the fake HAL: §3 data flow, §6 invariants and reset behavior, §7.
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "TestSupport.h"

using namespace n2;
using namespace n2::test;
using Tw = Tower::State;
using Cp = Compressor::State;
using O2s = O2Controller::State;

namespace {
void expectAllOff(Generator& p) {
  CHECK_FALSE(p.left());
  CHECK_FALSE(p.right());
  CHECK_FALSE(p.flush());
  CHECK_FALSE(p.ssr());
}
// Boot and switch TBS on, with a short O2 warm-up so scenarios stay short.
void startQuick(Generator& p) {
  p.reboot();
  p.tbs(true);
}
}  // namespace

TEST_CASE("RST-2: boot drives every output to its off level, whatever it was before") {
  Generator p;
  // The reset left the outputs wherever they were (here: all on).
  for (Signal s : {Signal::kLeftValve, Signal::kRightValve, Signal::kFlushValve, Signal::kSsr})
    p.hal.level[pinOf(kHostBoard, s)] = 1;
  p.reboot();
  expectAllOff(p);
  CHECK(p.sys().tower().state() == Tw::kDisabled);
  CHECK(p.sys().compressor().state() == Cp::kDisabled);
  CHECK(p.sys().o2().state() == O2s::kDisabled);
}

TEST_CASE("INV-1: with TBS off, nothing runs however good the pressures are") {
  Generator p(quickWarmConfig());
  p.reboot();
  p.run(10000);
  expectAllOff(p);
  CHECK(p.sys().invariantViolations() == 0);
}

TEST_CASE("INP-6/RST-3: TBS already ON at boot starts the system (counts as OFF->ON)") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr(); }, 3000));
  CHECK(p.sys().compressor().state() == Cp::kRunning);
  CHECK(p.log.has("TBS OFF->ON"));
}

TEST_CASE("OUT-1/RST-1: the SSR cannot start within the minimum hold of a reset") {
  Generator p(quickWarmConfig());
  startQuick(p);
  p.run(900);
  CHECK_FALSE(p.ssr());
  p.run(200);
  CHECK(p.ssr());
}

TEST_CASE("INV-10/O2-6: production (default config) holds the tower off for the 5-minute warm-up") {
  Generator p;  // default: O2 mandatory, 5 minutes warm-up
  startQuick(p);
  p.run(299000, 100);
  CHECK(p.sys().compressor().state() == Cp::kRunning);  // compressor is allowed to run
  CHECK(p.ssr());
  CHECK(p.sys().tower().state() == Tw::kDisabled);
  CHECK_FALSE(p.left());
  CHECK_FALSE(p.right());
  CHECK(p.sys().o2().state() == O2s::kWarming);
  CHECK_FALSE(p.sys().inputs().o2Warm);
  p.run(3000, 100);
  CHECK(p.sys().tower().state() != Tw::kDisabled);
  CHECK(p.left());
  CHECK(p.sys().invariantViolations() == 0);
}

TEST_CASE("O2-6a: a trusted warm-up credit shortens the wait after a reset") {
  Generator p;
  p.reboot(ResetInfo(), 250000);  // 250 s already warm
  p.tbs(true);
  p.run(45000, 100);
  CHECK(p.sys().tower().state() == Tw::kDisabled);
  p.run(10000, 100);
  CHECK(p.sys().tower().state() != Tw::kDisabled);
}

TEST_CASE("O2-1/INV-9: a missing O2 sensor keeps EVERY output off and raises F12") {
  Generator p(quickWarmConfig());
  p.o2.presentOk = false;
  startQuick(p);
  p.run(20000);
  expectAllOff(p);
  CHECK(p.sys().compressor().state() == Cp::kDisabled);
  CHECK(p.sys().faults().active(FaultId::kO2Comm));
  CHECK(p.sys().o2().state() == O2s::kError);
}

TEST_CASE("INV-9: the system starts as soon as the missing sensor appears") {
  Generator p(quickWarmConfig());
  p.o2.presentOk = false;
  startQuick(p);
  p.run(5000);
  expectAllOff(p);
  p.o2.presentOk = true;
  REQUIRE(p.runUntil([&] { return p.ssr(); }, 70000));  // after the ERROR retry
}

TEST_CASE("INV-9: losing the O2 sensor while running shuts everything down at once") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.left() && p.ssr(); }, 20000));
  p.o2.presentOk = false;
  REQUIRE(p.runUntil([&] { return !p.ssr() && !p.left() && !p.right(); }, 12000));
  expectAllOff(p);
  CHECK(p.sys().invariantViolations() == 0);
}

TEST_CASE("O2-1 bench: without a mandatory O2 sensor the system runs with none attached") {
  ControlConfig c = quickWarmConfig();
  c.o2Mandatory = false;
  Generator p(c);
  p.o2.presentOk = false;
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 5000));
}

TEST_CASE("INV-3/INV-5: over-pressure turns the SSR and towers off at once, even right after they switched on") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.n2High(kDefaultControl.n2HighOff + 30);
  p.step(10);  // ONE pass
  CHECK_FALSE(p.ssr());
  CHECK_FALSE(p.left());
  CHECK_FALSE(p.right());
  CHECK(p.sys().invariantViolations() == 0);
}

TEST_CASE("INV-3: restart only after the pressure has fallen below the ON threshold") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.n2High(kDefaultControl.n2HighOff + 30);
  p.run(2000);
  p.n2High((kDefaultControl.n2HighOn + kDefaultControl.n2HighOff) / 2);  // between ON and OFF
  p.run(3000);
  CHECK_FALSE(p.ssr());
  CHECK_FALSE(p.left());
  p.n2High(kDefaultControl.n2HighOn - 100);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 5000));
}

TEST_CASE("INV-4: low N2 stops the SSR but not the towers") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.n2Low(kDefaultControl.n2LowOff - 50);
  p.step(10);
  CHECK_FALSE(p.ssr());
  CHECK(p.left());
}

TEST_CASE("INV-2: low air stops the towers but not the compressor") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.air(kDefaultControl.airLowOff - 50);
  p.step(10);
  CHECK_FALSE(p.left());
  CHECK_FALSE(p.right());
  CHECK(p.ssr());
}

TEST_CASE("INP-4/INV-3: an N2-high sensor that goes dead stops tower and SSR (the V7 hole), with F03") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.rawN2High(0);  // wire fell off: 0 V
  REQUIRE(p.runUntil([&] { return !p.ssr() && !p.left() && !p.right(); }, 200));
  CHECK(p.sys().faults().active(FaultId::kN2HighSensor));
  p.run(10000);
  expectAllOff(p);  // and it stays that way: a dead sensor is never read as "empty tank"
}

TEST_CASE("INV-8: N2-low reading above N2-high (swapped or failed sensor) shuts the system down with F04") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.n2High(150);   // 15.0 PSI
  p.n2Low(2800);   // 28.00 PSI: above N2-high
  REQUIRE(p.runUntil([&] { return p.sys().faults().active(FaultId::kSensorOrder); }, 8000));
  p.step(10);
  CHECK_FALSE(p.ssr());
  CHECK_FALSE(p.left());
  CHECK_FALSE(p.right());
}

TEST_CASE("INV-1: TBS off during operation closes everything at once") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.tbs(false);
  p.step(10);
  expectAllOff(p);
  CHECK(p.sys().tower().state() == Tw::kDisabled);
  CHECK(p.sys().compressor().state() == Cp::kDisabled);
  CHECK(p.sys().o2().state() == O2s::kDisabled);
  CHECK(p.log.has("TBS ON->OFF"));
}

TEST_CASE("RST-1/RST-4: after a reset the SENSORS decide - over-pressure tank gets no SSR pulse") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr(); }, 20000));
  p.n2High(kDefaultControl.n2HighOff + 40);  // tank over its limit; reset button pressed now
  p.reboot();
  bool everOn = false;
  for (int i = 0; i < 400; ++i) { p.step(10); everOn = everOn || p.ssr(); }
  CHECK_FALSE(everOn);
  CHECK(p.sys().compressor().state() == Cp::kStoppedHigh);
}

TEST_CASE("RST-1: reset in the 'between ON and OFF' pressure band does not restart the compressor") {
  Generator p(quickWarmConfig());
  p.n2High((kDefaultControl.n2HighOn + kDefaultControl.n2HighOff) / 2);
  startQuick(p);
  p.run(5000);
  CHECK_FALSE(p.ssr());
  CHECK(p.sys().compressor().state() == Cp::kStoppedHigh);
}

TEST_CASE("RST-2/RST-1: a reset with outputs ON drives them off immediately, and nothing is ON in the first hold period") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  p.reboot();
  expectAllOff(p);  // straight after begin(), before any step
  for (int i = 0; i < 90; ++i) {
    p.step(10);
    CHECK_FALSE(p.ssr());
    CHECK_FALSE(p.left());
  }
}

TEST_CASE("RST-5/F30: a watchdog reset is reported, and cleared after the first normal cycle") {
  Generator p(quickWarmConfig());
  ResetInfo wd;
  wd.known = true;
  wd.watchdog = true;
  p.reboot(wd);
  CHECK(p.sys().faults().active(FaultId::kWatchdogReset));
  p.tbs(true);
  REQUIRE(p.runUntil([&] { return p.sys().tower().state() == Tw::kRight; }, 120000));
  p.run(p.cfg.faultHoldMs + 100);
  CHECK_FALSE(p.sys().faults().active(FaultId::kWatchdogReset));
}

TEST_CASE("VER-4: the system starts and cycles at every ADC bit depth") {
  const uint8_t bits = static_cast<uint8_t>(GENERATE(10, 12, 14));
  Generator p(quickWarmConfig(), bits);
  startQuick(p);
  INFO("bits=" << int(bits));
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  REQUIRE(p.runUntil([&] { return p.sys().tower().state() == Tw::kRight; }, 120000));
  CHECK(p.sys().invariantViolations() == 0);
}

TEST_CASE("GOAL-6: a full run across the millis() rollover behaves normally") {
  Generator p(quickWarmConfig());
  p.reboot();
  p.hal.nowMs = UINT32_MAX - 60000;  // the clock is 1 minute before the wrap
  p.sys().begin();
  p.tbs(true);
  p.run(200000);
  CHECK(p.sys().invariantViolations() == 0);
  CHECK(p.ssr());
  CHECK(p.sys().tower().state() != Tw::kDisabled);
  CHECK(p.log.count("TWR") >= 3);
}

TEST_CASE("ARC-7: the tower cycle log shows the whole sequence with deltas") {
  Generator p(quickWarmConfig());
  startQuick(p);
  REQUIRE(p.runUntil([&] { return p.sys().tower().state() == Tw::kLeft; }, 20000));
  p.run(p.cfg.towerFillMs + p.cfg.towerOverlapMs + 100);
  CHECK(p.log.has("TWR OF->L"));
  CHECK(p.log.has("TWR L->LB"));
  CHECK(p.log.has("TWR LB->R"));
}
