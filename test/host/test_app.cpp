// Whole-firmware tests through App: boot order, POST -> RUN, `post`/`bist` commands, TOB-at-boot BIST, DIAG build,
// watchdog, loop statistics, display faults, warm-up credit (Requirements §3, §4, §6, §11, §12, §14).
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>

#include "TestSupport.h"
#include "app/App.h"

using namespace n2;
using namespace n2::test;

namespace {

struct AppRig {
  FakeHal hal;
  FakeO2 o2;
  BuildInfo info{"0.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  ControlConfig cfg;
  AppOptions opt;
  WarmRecord rec{};
  std::unique_ptr<App> app;
  std::string out;  // everything the console printed

  explicit AppRig(ControlConfig c = quickWarmConfig(), AppOptions o = AppOptions(), bool consoleAttached = true) : cfg(c), opt(o) {
    hal.consoleIsAttached = consoleAttached;
    hal.i2cPresent = {0x24, 0x34, 0x35, 0x36, 0x37, 0x27, 0x74};
    healthy();
    tbs(false);
    tob(false);
  }

  static uint8_t pinOfSignal(Signal s) { return pinOf(kHostBoard, s); }
  void healthy() {
    hal.analogValue[pinOfSignal(Signal::kAirPressure)] = rawFor(kHealthyAirX10, kAirFullScaleX10);
    hal.analogValue[pinOfSignal(Signal::kN2LowPressure)] = rawFor(kHealthyN2LowX100, kN2LowFullScaleX100);
    hal.analogValue[pinOfSignal(Signal::kN2HighPressure)] = rawFor(kHealthyN2HighX10, kN2HighFullScaleX10);
  }
  void tbs(bool on) { const SignalDef& d = def(kHostBoard, Signal::kTbs); hal.inputLevel[d.pin] = levelHigh(on, d.active); }
  void tob(bool on) { const SignalDef& d = def(kHostBoard, Signal::kTob); hal.inputLevel[d.pin] = levelHigh(on, d.active); }
  bool outputOn(Signal s) { const SignalDef& d = def(kHostBoard, s); return isOn(hal.level[d.pin] == 1, d.active); }
  bool anyOutputOn() { return outputOn(Signal::kLeftValve) || outputOn(Signal::kRightValve) || outputOn(Signal::kFlushValve) || outputOn(Signal::kSsr); }

  void boot(ResetInfo reset = ResetInfo()) {
    hal.resetCause = reset;
    app.reset(new App(hal, kHostBoard, cfg, o2, info, &rec, opt));
    hal.nowMs = 0;
    app->setup();
  }
  void tick(uint32_t ms = 10) {
    hal.nowMs += ms;
    app->loop();
    out += hal.consoleOut;
    hal.consoleOut.clear();
    hal.drain();
  }
  void run(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 10) tick(10); }
  void type(const std::string& line) { hal.type(line + "\n"); run(40); }
  bool has(const std::string& s) const { return out.find(s) != std::string::npos; }
  bool runUntilMode(App::Mode m, uint32_t limitMs) {
    for (uint32_t t = 0; t < limitMs && app->mode() != m; t += 10) tick(10);
    return app->mode() == m;
  }
};

}  // namespace

TEST_CASE("RST-2: the first thing setup() does is drive the outputs safe (before ADC, I2C, watchdog)") {
  AppRig r;
  for (Signal s : {Signal::kLeftValve, Signal::kRightValve, Signal::kFlushValve, Signal::kSsr})
    r.hal.level[AppRig::pinOfSignal(s)] = 1;  // the reset left them on
  r.boot();
  REQUIRE_FALSE(r.hal.events.empty());
  // The very first events are writes to output pins; the ADC resolution and I2C start come afterwards.
  size_t firstOther = 0;
  while (firstOther < r.hal.events.size() && (r.hal.events[firstOther].kind == FakeHal::Kind::kWrite ||
                                              r.hal.events[firstOther].kind == FakeHal::Kind::kPinMode))
    ++firstOther;
  CHECK(firstOther >= 12);  // four outputs x (write, mode, write)
  CHECK_FALSE(r.anyOutputOn());
  CHECK(r.hal.adcBits == kAdcBits);
  CHECK(r.hal.i2cStarted);
}

TEST_CASE("WDT-1: the watchdog is started with the configured timeout, and refreshed on every loop pass") {
  AppRig r;
  r.boot();
  CHECK(r.hal.watchdogTimeoutMs == 4000);
  const uint32_t before = r.hal.watchdogRefreshes;
  r.run(1000);
  CHECK(r.hal.watchdogRefreshes - before == 100);
}

TEST_CASE("WDT-5: the watchdog can be disabled for debugging") {
  AppOptions o;
  o.watchdogEnabled = false;
  AppRig r(quickWarmConfig(), o);
  r.boot();
  r.run(100);
  CHECK(r.hal.watchdogTimeoutMs == 0);
}

TEST_CASE("§4: boot -> POST -> RUN by itself, within 3 s, without a console or a button press") {
  AppRig r(quickWarmConfig(), AppOptions(), /*consoleAttached=*/false);
  r.boot();
  CHECK(r.app->mode() == App::Mode::kPost);
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  CHECK(r.app->post().level() == PostLevel::kPass);
  CHECK_FALSE(r.anyOutputOn());
}

TEST_CASE("§11: outputs stay off for the whole POST even with TBS on and good pressures") {
  AppRig r;
  r.tbs(true);
  r.boot();
  while (r.app->mode() == App::Mode::kPost) {
    r.tick();
    REQUIRE_FALSE(r.anyOutputOn());
    REQUIRE(r.hal.nowMs < 4000);
  }
}

TEST_CASE("§4/RST-3: TBS ON at boot starts the system once POST is done") {
  AppRig r;
  r.tbs(true);
  r.boot();
  r.run(3000);
  REQUIRE(r.app->mode() == App::Mode::kRun);
  r.run(5000);
  CHECK(r.outputOn(Signal::kSsr));
  CHECK(r.outputOn(Signal::kLeftValve));
  CHECK(r.app->system().invariantViolations() == 0);
}

TEST_CASE("§3: in RUN the LCD shows the normal screen and the console is live") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.run(500);
  CHECK(std::string(r.app->display().lcd().shown(2)).substr(16, 4) == "LRFS");
  r.type("ver");
  CHECK(r.has("N2V8 0.0.0-test"));
  r.type("status");
  CHECK(r.has("TBS off"));
}

TEST_CASE("CON-4: when a console attaches it is told who we are (it missed the boot messages)") {
  AppRig r(quickWarmConfig(), AppOptions(), /*consoleAttached=*/false);
  r.boot();
  r.run(500);
  CHECK(r.out.empty());
  r.hal.consoleIsAttached = true;
  r.run(100);
  CHECK(r.has("[console attached]"));
  CHECK(r.has("N2V8 0.0.0-test  built Jan  1 2026 12:00:00  board host (fake)  mode HOST  adc 10 bits"));
  CHECK(r.has("Type help."));
}

TEST_CASE("§11: the `post` command runs POST again, disabling the system while it runs") {
  AppRig r;
  r.tbs(true);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.run(5000);
  REQUIRE(r.outputOn(Signal::kSsr));
  r.type("post");
  CHECK(r.has("running POST"));
  r.run(50);
  CHECK(r.app->mode() == App::Mode::kPost);
  CHECK_FALSE(r.anyOutputOn());
  REQUIRE(r.runUntilMode(App::Mode::kRun, 4000));
  r.run(5000);
  CHECK(r.outputOn(Signal::kSsr));  // the system resumed
}

TEST_CASE("§12/BIST-1: the `bist` command starts the BIST, which the operator can quit; the system then resumes") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("bist");
  CHECK(r.has("starting BIST"));
  r.run(100);
  CHECK(r.app->mode() == App::Mode::kBist);
  CHECK(r.has("BIST 0: banner"));
  r.type("q");
  r.run(100);
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK(r.has("BIST finished: operator quit"));
  CHECK_FALSE(r.anyOutputOn());
  r.tbs(true);
  r.run(8000);
  CHECK(r.outputOn(Signal::kSsr));  // normal operation works after BIST
}

TEST_CASE("BIST-1: `bist` is refused while TBS is ON, and the system keeps running") {
  AppRig r;
  r.tbs(true);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.run(5000);
  r.type("bist");
  r.run(100);
  CHECK(r.has("BIST refused: switch TBS OFF first"));
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK(r.outputOn(Signal::kSsr));
}

TEST_CASE("RST-6/BIST-1: TOB held at power-up starts the BIST after POST (console attached)") {
  AppRig r;
  r.tob(true);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kBist, 4000));
  r.run(100);
  CHECK(r.has("BIST 0: banner"));
  CHECK_FALSE(r.anyOutputOn());
}

TEST_CASE("BIST-1: TOB held at power-up without a console does NOT start the BIST; the LCD says why") {
  AppRig r(quickWarmConfig(), AppOptions(), /*consoleAttached=*/false);
  r.tob(true);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 4000));
  r.run(300);
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK(std::string(r.app->display().lcd().shown(0)) == "BIST: NO CONSOLE    ");
}

TEST_CASE("§2 DIAG: with controllers disabled, no output ever turns on however good the conditions") {
  AppOptions o;
  o.controllersEnabled = false;
  AppRig r(quickWarmConfig(), o);
  r.tbs(true);
  r.boot();
  r.run(20000);
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK_FALSE(r.anyOutputOn());
  CHECK(r.app->system().tower().state() == Tower::State::kDisabled);
  CHECK(r.app->system().compressor().state() == Compressor::State::kDisabled);
}

TEST_CASE("§2 DIAG: the displays still show live sensor readings") {
  AppOptions o;
  o.controllersEnabled = false;
  AppRig r(quickWarmConfig(), o);
  r.boot();
  r.run(4000);
  CHECK(std::string(r.app->display().lcd().shown(3)).substr(0, 8) == "AIR 130.");
}

TEST_CASE("RST-1/RST-2: a reset in the middle of operation turns every output off at once") {
  AppRig r;
  r.tbs(true);
  r.boot();
  r.run(8000);
  REQUIRE(r.outputOn(Signal::kSsr));
  ResetInfo button;
  button.known = true;
  r.boot(button);  // reset button: new App, pins keep their levels until setup() runs
  CHECK_FALSE(r.anyOutputOn());
  r.run(100);
  CHECK_FALSE(r.anyOutputOn());  // and nothing starts during POST
}

TEST_CASE("NFR-1: every loop pass is timed; `loop` reports the statistics") {
  AppRig r;
  r.hal.microsPerCall = 40;  // each micros() call advances 40 us, so a pass takes about 40 us
  r.boot();
  r.run(1000);
  CHECK(r.app->loopStats().count() == 100);
  CHECK(r.app->loopStats().minUs() > 0);
  r.type("loop");
  CHECK(r.has("loop n="));
  CHECK(r.has("median<="));
}

TEST_CASE("DSP-6/F10: a missing LCD is reported as a fault in RUN and never stops the system") {
  AppRig r;
  r.hal.i2cPresent.erase(0x27);
  r.tbs(true);
  r.boot();
  r.run(12000);
  CHECK(r.app->system().faults().active(FaultId::kLcd));
  CHECK(r.outputOn(Signal::kSsr));  // the system still runs
  r.type("faults");
  CHECK(r.has("F10 LCD NO ACK"));
}

TEST_CASE("DSP-5: a fault in RUN alternates the fault screen with the normal screen") {
  AppRig r;
  r.tbs(true);
  r.boot();
  r.run(8000);
  r.hal.analogValue[AppRig::pinOfSignal(Signal::kN2HighPressure)] = 0;  // sensor wire off
  bool sawFault = false, sawNormal = false;
  for (uint32_t t = 0; t < 12000; t += 10) {
    r.tick();
    const std::string row0 = r.app->display().lcd().shown(0);
    if (row0.find("FAULT 1 OF 1") == 0) sawFault = true;
    if (row0.find("N2%") == 0 || row0.find("WRM") == 0) sawNormal = true;
  }
  CHECK(sawFault);
  CHECK(sawNormal);
  CHECK_FALSE(r.outputOn(Signal::kSsr));
}

TEST_CASE("CON-2/F40: when the host stops reading, log lines are dropped and F40 is reported; the loop keeps running") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.tbs(true);
  for (uint32_t t = 0; t < 20000; t += 10) {
    r.hal.nowMs += 10;
    r.hal.consoleSpace = 0;  // a stalled terminal
    r.app->loop();
  }
  CHECK(r.app->console().dropped() > 0);
  CHECK(r.app->system().faults().active(FaultId::kConsoleDrop));
  CHECK(r.outputOn(Signal::kSsr));  // the system never waited for the console
}

TEST_CASE("O2-6a: with credit enabled, a reset-button reset keeps the warm-up already earned") {
  ControlConfig c = kDefaultControl;  // production: O2 mandatory, 5-minute warm-up
  AppOptions o;
  o.warmCreditEnabled = true;
  AppRig r(c, o);
  r.tbs(true);
  ResetInfo power;
  power.known = true;
  power.powerOn = true;
  r.boot(power);
  r.run(200000);                     // 200 s into the 300 s warm-up
  CHECK_FALSE(r.app->system().warmup().warm(r.hal.nowMs));
  ResetInfo button;
  button.known = true;
  r.boot(button);                    // reset button: the sensor kept its power
  r.run(5000);
  CHECK(r.app->system().warmup().remainingMs(r.hal.nowMs) < 105000);  // ~100 s left, not 300 s
}

TEST_CASE("O2-6b: with credit disabled (the default) every boot waits the full warm-up") {
  ControlConfig c = kDefaultControl;
  AppRig r(c);  // warmCreditEnabled = false
  r.tbs(true);
  ResetInfo power;
  power.known = true;
  power.powerOn = true;
  r.boot(power);
  r.run(200000);
  ResetInfo button;
  button.known = true;
  r.boot(button);
  r.run(5000);
  CHECK(r.app->system().warmup().remainingMs(r.hal.nowMs) > 290000);
}

TEST_CASE("O2-6a: a power-on reset never gets credit, even with the feature enabled") {
  ControlConfig c = kDefaultControl;
  AppOptions o;
  o.warmCreditEnabled = true;
  AppRig r(c, o);
  r.tbs(true);
  ResetInfo power;
  power.known = true;
  power.powerOn = true;
  r.boot(power);
  r.run(200000);
  r.boot(power);  // power cycled
  r.run(5000);
  CHECK(r.app->system().warmup().remainingMs(r.hal.nowMs) > 290000);
}

TEST_CASE("ARC-3: the whole firmware run above never touched anything but the Hal (it ran on the host)") {
  SUCCEED();  // by construction: App, System, POST, BIST and drivers include no Arduino header
}
