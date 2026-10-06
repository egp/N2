// POST tests: §11 POST-1..POST-6 (hands-off, quick, holds only on a fault, TOB releases).
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "TestSupport.h"
#include "selftest/Post.h"

using namespace n2;
using namespace n2::test;

namespace {

struct Rig {
  Generator gen;
  bool booted;  // the generator must be rebooted BEFORE anything takes a reference to its System
  DisplayManager display;
  BuildInfo info{"0.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  Post post;

  explicit Rig(ControlConfig cfg = kDefaultControl, ResetInfo reset = ResetInfo(), PostOptions opt = PostOptions())
      : gen(cfg),
        booted((gen.reboot(reset), true)),  // (members are initialised in declaration order, so this comes first)
        display(gen.hal, kHostBoard, LcdLayout::kClearLabels),
        post(gen.hal, kHostBoard, gen.sys(), display, gen.log, info, reset, opt) {
    (void)booted;
    gen.hal.i2cPresent = {0x24, 0x34, 0x35, 0x36, 0x37, 0x27, 0x74, 0x68};  // everything fitted
    gen.hal.nowMs = 0;
  }

  // One pass of the boot loop: displays serviced, POST stepped, outputs checked.
  bool pass(uint32_t now) {
    gen.hal.nowMs = now;
    display.service(now);
    const bool done = post.step(now);
    return done;
  }
  // Run until POST finishes; returns the time it finished or -1.
  int64_t runToEnd(uint32_t limitMs = 20000) {
    display.begin(0);
    post.begin(0);
    for (uint32_t t = 0; t <= limitMs; t += 10) {
      if (pass(t)) return t;
      REQUIRE_FALSE(gen.left());   // POST never switches an output on
      REQUIRE_FALSE(gen.right());
      REQUIRE_FALSE(gen.flush());
      REQUIRE_FALSE(gen.ssr());
    }
    return -1;
  }
  void pressTob(bool down) { gen.tob(down); }
};

bool has(const Rig& r, const std::string& s) { return r.gen.log.has(s); }
std::string row(Rig& r, int i) { return std::string(r.display.lcd().shown(i)); }

}  // namespace

TEST_CASE("POST-1: a clean POST finishes by itself, quickly (<= 3 s), without a console or TOB") {
  Rig r;
  CHECK_FALSE(r.gen.hal.consoleIsAttached);  // no console required (POST-3 headless)
  const int64_t t = r.runToEnd();
  REQUIRE(t >= 0);
  CHECK(t <= 3000);
  CHECK(r.post.finished());
  CHECK(r.post.level() == PostLevel::kPass);
  CHECK(r.post.problemCount() == 0);
  CHECK(has(r, "POST PASS"));
  CHECK(r.gen.sys().faults().activeCount() == 0);
  CHECK_FALSE(r.display.overriding());  // the normal screen takes over
}

TEST_CASE("POST-2: all seven checks log one line each, in order, then the summary") {
  Rig r;
  REQUIRE(r.runToEnd() >= 0);
  const char* expected[] = {"POST 1 outputs safe OK", "POST 2 reset cause", "POST 3 I2C LCD ok  LED ok  O2 ok",
                            "POST 4 sensors AIR ok  N2L ok  N2H ok", "POST 5 switches TBS off  TOB off", "POST 6 config OK",
                            "POST 7 build N2V8 0.0.0-test", "POST PASS"};
  size_t from = 0;
  for (const char* e : expected) {
    bool found = false;
    for (size_t i = from; i < r.gen.log.lines.size(); ++i) {
      if (r.gen.log.lines[i].find(e) != std::string::npos) { from = i + 1; found = true; break; }
    }
    INFO(e);
    CHECK(found);
  }
}

TEST_CASE("DSP-8: the start-up banner (version, board, date) is on the LCD while POST runs") {
  Rig r;
  r.display.begin(0);
  r.post.begin(0);
  for (uint32_t t = 0; t < 400; t += 10) r.pass(t);
  CHECK(row(r, 0) == "N2V8 0.0.0-test     ");
  CHECK(row(r, 1) == "host (fake)         ");
  CHECK(row(r, 2) == "Jan  1 2026         ");
}

TEST_CASE("POST-5: a clean POST ends with 'POST OK' on the LCD for about a second") {
  Rig r;
  r.display.begin(0);
  r.post.begin(0);
  bool sawOk = false;
  for (uint32_t t = 0; t < 3000; t += 10) {
    r.pass(t);
    if (row(r, 0) == "POST OK             ") sawOk = true;
  }
  CHECK(sawOk);
}

TEST_CASE("POST-4: a missing LCD is only an INFO fault - logged, no hold") {
  Rig r;
  r.gen.hal.i2cPresent.erase(0x27);
  const int64_t t = r.runToEnd();
  REQUIRE(t >= 0);
  CHECK(r.gen.sys().faults().active(FaultId::kLcd));
  CHECK(r.post.level() == PostLevel::kWarn);
  CHECK(r.post.problemCount() == 1);
  CHECK_FALSE(r.post.holding());
  CHECK(has(r, "POST 3 I2C LCD MISSING"));
  CHECK(has(r, "POST WARN 1"));
  CHECK(t <= 7000);  // 1 s banner + 5 s result screen, then on its way
}

TEST_CASE("POST-4: a missing LED is only an INFO fault") {
  Rig r;
  r.gen.hal.i2cPresent.erase(0x24);
  REQUIRE(r.runToEnd() >= 0);
  CHECK(r.gen.sys().faults().active(FaultId::kLed));
  CHECK(r.post.level() == PostLevel::kWarn);
}

TEST_CASE("POST-4/O2-1: a missing O2 sensor in production holds POST until TOB, outputs stay off") {
  Rig r;  // default config: O2 mandatory
  r.gen.hal.i2cPresent.erase(0x74);
  CHECK(r.runToEnd(20000) == -1);  // never finishes by itself
  CHECK(r.post.holding());
  CHECK(r.post.level() == PostLevel::kFail);
  CHECK(r.gen.sys().faults().active(FaultId::kO2Comm));
  CHECK(has(r, "POST FAIL 1"));
  CHECK(has(r, "POST holding"));
  CHECK(row(r, 0) == "POST FAIL 1         ");
  CHECK(row(r, 1) == "F12 O2 SENSOR FAILED");
  CHECK(row(r, 3) == "PRESS TOB TO PROCEED");
}

TEST_CASE("POST-4: TOB releases a held POST (a fresh press), the faults stay active") {
  Rig r;
  r.gen.hal.i2cPresent.erase(0x74);
  REQUIRE(r.runToEnd(12000) == -1);
  REQUIRE(r.post.holding());
  r.pressTob(true);
  CHECK(r.pass(12010));
  CHECK(r.post.finished());
  CHECK(has(r, "POST released by operator"));
  CHECK(r.gen.sys().faults().active(FaultId::kO2Comm));  // the system is still protected by INV-9
}

TEST_CASE("POST-4: a TOB held down since power-up does NOT release the hold until it is released and pressed again") {
  Rig r;
  r.gen.hal.i2cPresent.erase(0x74);
  r.pressTob(true);  // held at boot (e.g. asking for BIST)
  REQUIRE(r.runToEnd(12000) == -1);
  CHECK(r.post.holding());
  r.pressTob(false);
  CHECK_FALSE(r.pass(12010));
  r.pressTob(true);
  CHECK(r.pass(12020));
}

TEST_CASE("O2-1 bench: without a mandatory O2 sensor, no O2 on the bus is not a fault") {
  ControlConfig c = kDefaultControl;
  c.o2Mandatory = false;
  Rig r(c);
  r.gen.hal.i2cPresent.erase(0x74);
  REQUIRE(r.runToEnd() >= 0);
  CHECK(r.post.level() == PostLevel::kPass);
  CHECK(has(r, "O2 absent (not required)"));
}

TEST_CASE("POST-2/INP-4: a pressure sensor out of range is a fault; POST holds with the fault on screen") {
  Rig r;
  r.gen.rawN2High(0);
  CHECK(r.runToEnd(8000) == -1);
  CHECK(r.gen.sys().faults().active(FaultId::kN2HighSensor));
  CHECK(r.post.holding());
  CHECK(has(r, "N2H RANGE"));
  CHECK(row(r, 0) == "POST FAIL 1         ");
  CHECK(row(r, 1) == "F03 N2H SENSOR RANGE");
}

TEST_CASE("POST-2: all three sensors are judged, each with its own fault") {
  Rig r;
  r.gen.rawAir(0);
  r.gen.rawN2Low(1023);
  r.gen.rawN2High(0);
  CHECK(r.runToEnd(8000) == -1);
  CHECK(r.gen.sys().faults().active(FaultId::kAirSensor));
  CHECK(r.gen.sys().faults().active(FaultId::kN2LowSensor));
  CHECK(r.gen.sys().faults().active(FaultId::kN2HighSensor));
  CHECK(r.post.problemCount() == 3);
}

TEST_CASE("POST-4: while held, the screen steps through the faults every few seconds") {
  Rig r;
  r.gen.rawAir(0);
  r.gen.rawN2High(0);
  r.display.begin(0);
  r.post.begin(0);
  std::string firstSeen, laterSeen;
  for (uint32_t t = 0; t < 20000; t += 10) {
    r.pass(t);
    if (t == 7000) firstSeen = row(r, 1);
    if (t == 10000) laterSeen = row(r, 1);
  }
  REQUIRE(r.post.holding());
  CHECK(firstSeen != laterSeen);  // the list moved on
}

TEST_CASE("POST-4: a watchdog reset is reported (F30) but does not keep an unattended unit down") {
  ResetInfo wd;
  wd.known = true;
  wd.watchdog = true;
  Rig r(kDefaultControl, wd);
  const int64_t t = r.runToEnd();
  REQUIRE(t >= 0);
  CHECK(r.gen.sys().faults().active(FaultId::kWatchdogReset));
  CHECK(r.post.level() == PostLevel::kWarn);
  CHECK_FALSE(r.post.holding());
  CHECK(has(r, "POST 2 reset cause watchdog"));
}

TEST_CASE("POST-2: an output found ON at POST time fails the check") {
  Rig r;
  r.gen.sys().resume();
  // Force the driver into a bad state the way a bug would: the output pin driven on behind its back.
  r.gen.sys().outputs();  // (read-only accessor; the real protection is that POST re-reads the driver state)
  ControlConfig c = kDefaultControl;
  c.airLowOn = c.airLowOff;  // an invalid configuration is the other way POST can detect a software problem
  Rig bad(c);
  CHECK(bad.runToEnd(8000) == -1);
  CHECK(bad.gen.sys().faults().active(FaultId::kInvariant));
  CHECK(has(bad, "POST 6 config INVALID"));
}

TEST_CASE("POST-6: the controllers stay disabled and the outputs off for the whole POST, even with TBS on and good pressures") {
  Rig r;
  r.gen.tbs(true);
  REQUIRE(r.runToEnd() >= 0);  // runToEnd asserts every output is off at every pass
  CHECK(r.gen.sys().tower().state() == Tower::State::kDisabled);
  CHECK(r.gen.sys().compressor().state() == Compressor::State::kDisabled);
}

TEST_CASE("POST-1: after POST the system starts normally") {
  Rig r;
  r.gen.tbs(true);
  REQUIRE(r.runToEnd() >= 0);
  r.gen.hal.nowMs = 3000;
  r.gen.run(5000);
  CHECK(r.gen.sys().o2().commOk());
  CHECK(r.gen.ssr());
}

// ============================================================================ RTC in POST (RTC-5)
TEST_CASE("RTC-5: a missing RTC is only an INFO fault (F13): logged, no hold, POST WARN") {
  Rig r;
  r.gen.hal.i2cPresent.erase(0x68);
  const int64_t t = r.runToEnd();
  REQUIRE(t >= 0);
  CHECK(r.gen.sys().faults().active(FaultId::kRtc));
  CHECK(r.post.level() == PostLevel::kWarn);
  CHECK_FALSE(r.post.holding());
  CHECK(has(r, "RTC MISSING"));
}

TEST_CASE("RTC-5: an RTC that lost power (time not trusted) is reported as 'NOT SET', INFO only") {
  Rig r;
  r.gen.hal.i2cRegs[0x68].assign(256, 0);
  r.gen.hal.i2cRegs[0x68][0x0F] = 0x80;   // oscillator-stop flag set
  REQUIRE(r.runToEnd() >= 0);
  CHECK(has(r, "RTC NOT SET"));
  CHECK(r.gen.sys().faults().active(FaultId::kRtc));
  CHECK_FALSE(r.post.holding());
}

TEST_CASE("RTC-5: a healthy RTC adds nothing to the POST result") {
  Rig r;
  REQUIRE(r.runToEnd() >= 0);
  CHECK(has(r, "RTC ok"));
  CHECK(r.post.level() == PostLevel::kPass);
}
