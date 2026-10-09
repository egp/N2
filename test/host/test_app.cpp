// Whole-firmware tests through App: boot order, POST -> RUN, `post`/`bist` commands, TOB-at-boot POST mode, DIAG build,
// watchdog, loop statistics, display faults, warm-up credit (Requirements §3, §4, §6, §11, §12, §14).
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>

#include "FakeNvm.h"
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
  ModeRecord modeRec{};   // the run-mode breadcrumb, survives the simulated resets
  BistRecord bistRec{};   // survives the simulated resets, like the RAM record on the board
  std::unique_ptr<App> app;
  std::string out;  // everything the console printed

  explicit AppRig(ControlConfig c = quickWarmConfig(), AppOptions o = AppOptions(), bool consoleAttached = true) : cfg(c), opt(o) {
    opt.lcdStartMs = 0;  // LCD at once; the real default (2.5 s after boot) has its own test
    hal.consoleIsAttached = consoleAttached;
    hal.i2cPresent = {0x24, 0x34, 0x35, 0x36, 0x37, 0x23, 0x74, 0x68};
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
    opt.bistRecord = &bistRec;
    opt.modeRecord = &modeRec;
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

TEST_CASE("POST-1: a normal boot runs no self-test: outputs safe, then straight to RUN, with or without a console") {
  for (bool console : {false, true}) {
    AppRig r(quickWarmConfig(), AppOptions(), console);
    r.boot();
    CHECK(r.app->mode() == App::Mode::kRun);
    r.run(500);
    CHECK(r.app->mode() == App::Mode::kRun);
    CHECK_FALSE(r.has("POST"));
    CHECK_FALSE(r.has("BIST"));
    CHECK_FALSE(r.anyOutputOn());
  }
}

TEST_CASE("POST-1/RST-6: TOB held at power-up or reset selects POST mode, which ends by itself (about 12 s with the POST OK screen) and then runs") {
  AppRig r(quickWarmConfig(), AppOptions(), /*consoleAttached=*/false);
  r.tob(true);
  r.boot();
  CHECK(r.app->mode() == App::Mode::kPostArm);
  r.tob(false);
  REQUIRE(r.runUntilMode(App::Mode::kRun, 14000));
  CHECK(r.app->post().level() == PostLevel::kPass);
  CHECK_FALSE(r.anyOutputOn());
}

TEST_CASE("§11: outputs stay off for the whole POST even with TBS on and good pressures") {
  AppRig r;
  r.tbs(true);
  r.tob(true);   // POST mode
  r.boot();
  r.tob(false);
  REQUIRE(r.runUntilMode(App::Mode::kPost, 1000));
  while (r.app->mode() == App::Mode::kPost) {
    r.tick();
    REQUIRE_FALSE(r.anyOutputOn());
    REQUIRE(r.hal.nowMs < 4000);
  }
}

TEST_CASE("§4/RST-3: TBS ON at a normal boot starts the system (after the sensor rules and warm-up allow it)") {
  AppRig r;
  r.tbs(true);
  r.boot();
  REQUIRE(r.app->mode() == App::Mode::kRun);
  r.run(8000);
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
  CHECK(r.has("N2V8 0.0.0-test  built Jan  1 2026 12:00:00  board host (fake)  mode FIELD  adc 10 bits"));
  CHECK(r.has("Type help."));
}

TEST_CASE("§11: the `post` command is refused while TBS is ON (the system is enabled), with an explanation") {
  AppRig r;
  r.tbs(true);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.run(5000);
  REQUIRE(r.outputOn(Signal::kSsr));
  r.type("post");
  CHECK(r.has("POST refused: the system is enabled (TBS is ON). Switch TBS OFF, then type post."));
  r.run(200);
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK(r.outputOn(Signal::kSsr));   // the running system was not disturbed
}

TEST_CASE("§11: with TBS OFF the `post` command runs the POST, then TBS ON enables the system as usual; TBS ON ends a good POST's OK screen early") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("post");
  CHECK(r.has("running POST"));
  r.run(50);
  CHECK(r.app->mode() == App::Mode::kPost);
  CHECK_FALSE(r.anyOutputOn());
  r.run(3000);                          // checks done, the OK screen is up
  CHECK(r.app->mode() == App::Mode::kPost);
  r.tbs(true);
  r.run(200);
  CHECK(r.app->mode() == App::Mode::kRun);   // TBS ON ended the OK screen
  r.run(8000);
  CHECK(r.outputOn(Signal::kSsr));           // and the system started by the normal TBS rule
}

TEST_CASE("§12: the BIST tests TBS without enabling the system; when it ends with TBS ON the system enables") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("bist");
  r.run(100);
  REQUIRE(r.app->mode() == App::Mode::kBist);
  r.tbs(true);                           // the operator flips TBS during the switch test
  r.run(8000);
  CHECK_FALSE(r.anyOutputOn());          // BIST does not enable the system
  r.type("q");
  r.run(300);
  REQUIRE(r.app->mode() == App::Mode::kRun);
  r.run(8000);
  CHECK(r.outputOn(Signal::kSsr));       // back to normal: TBS ON enables the system
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
  CHECK(r.has("BIST refused: the system is enabled (TBS is ON)"));
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK(r.outputOn(Signal::kSsr));
}

TEST_CASE("BIST-1: TOB held at power-up selects POST, never BIST; BIST starts only on the console command") {
  AppRig r;
  r.tob(true);
  r.boot();
  CHECK(r.app->mode() == App::Mode::kPostArm);
  r.tob(false);
  REQUIRE(r.runUntilMode(App::Mode::kRun, 14000));
  r.run(500);
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK_FALSE(r.has("BIST 0: banner"));
  r.tob(false);
  r.type("bist");
  r.run(200);
  CHECK(r.app->mode() == App::Mode::kBist);
  CHECK(r.has("BIST 0: banner"));
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
  r.hal.i2cPresent.erase(0x23);
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

TEST_CASE("CON-2/F40: lines dropped in the first 3 s after boot (start-up burst, known cause) are counted but do not raise F40") {
  AppRig r;
  r.hal.consoleSpace = 0;    // the boot messages find no room
  r.boot();
  for (uint32_t t = 0; t < 2500; t += 10) {
    r.hal.nowMs += 10;
    r.hal.consoleSpace = 0;
    r.app->loop();
  }
  CHECK(r.app->console().dropped() > 0);                                 // counted (status shows it)
  CHECK_FALSE(r.app->system().faults().active(FaultId::kConsoleDrop));   // but no fault: the cause is known
  CHECK(r.app->system().faults().lastCode() == 0);                       // and the ER field stays blank
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

// ============================================================================ R4 WiFi console (CON-4)
TEST_CASE("CON-4: setup() starts the console (the R4 WiFi core does not)") {
  AppRig r;
  CHECK_FALSE(r.hal.consoleBegun);
  r.boot();
  CHECK(r.hal.consoleBegun);
}

TEST_CASE("CON-4: where a PC cannot be detected (R4 WiFi) the banner repeats every 10 s until the host speaks") {
  AppRig r;
  r.hal.canDetectHost = false;  // emulate the UART: always 'attached', cannot see the PC
  r.boot();
  r.run(35000);
  size_t banners = 0, pos = 0;
  while ((pos = r.out.find("N2V8 0.0.0-test  built", pos)) != std::string::npos) { ++banners; pos += 5; }
  CHECK(banners >= 4);  // at attach (boot), then every 10 s
  const size_t before = banners;
  r.type("ver");        // the host has spoken: stop repeating
  r.out.clear();
  r.run(30000);
  banners = 0;
  pos = 0;
  while ((pos = r.out.find("N2V8 0.0.0-test  built", pos)) != std::string::npos) { ++banners; pos += 5; }
  CHECK(banners == 0);
  (void)before;
}

TEST_CASE("CON-4: where the host CAN be detected (Minima) the banner is printed once, at attach") {
  AppRig r(quickWarmConfig(), AppOptions(), /*consoleAttached=*/false);
  r.boot();
  r.run(500);
  r.hal.consoleIsAttached = true;
  r.run(25000);
  size_t banners = 0, pos = 0;
  while ((pos = r.out.find("N2V8 0.0.0-test  built", pos)) != std::string::npos) { ++banners; pos += 5; }
  CHECK(banners == 1);
}

// ============================================================================ RTC in the application (RTC-3, RTC-5)
TEST_CASE("RTC-5: the banner and `report` carry the RTC date/time when it is fitted and set") {
  AppRig r;
  r.boot();
  Rtc3231 clock(r.hal, 0x68);
  REQUIRE(clock.set({2026, 10, 6, 10, 31, 2}));
  r.hal.canDetectHost = false;
  r.run(1000);
  r.type("report");
  r.run(3000);
  CHECK(r.has("RTC 2026-10-06 10:31:02 (trusted)"));
}

TEST_CASE("RTC-5: F13 appears in RUN when the RTC is missing, never stops the system, and clears when it returns") {
  AppRig r;
  r.hal.i2cPresent.erase(0x68);
  r.tbs(true);
  r.boot();
  r.run(12000);
  CHECK(r.app->system().faults().active(FaultId::kRtc));
  CHECK(r.outputOn(Signal::kSsr));  // control is unaffected
  r.hal.i2cPresent.insert(0x68);
  r.run(30000);
  CHECK_FALSE(r.app->system().faults().active(FaultId::kRtc));
}

TEST_CASE("RTC-3: `time set` works through the whole application console") {
  AppRig r;
  r.boot();
  r.run(3500);
  r.type("time set 2026-10-06 10:31:00");
  CHECK(r.has("RTC set to 2026-10-06 10:31:00"));
  r.type("time");
  CHECK(r.has("RTC 2026-10-06 10:31:00 (trusted)"));
}

// ---------------------------------------------------------------------------- NVM-1 / INP-6: stored debounce times
namespace {
void storeRecord(FakeNvm& nvm, size_t addr, uint32_t seq, uint8_t tbs, uint8_t tob, BoardId b, uint16_t ver = 0x0801) {
  NvmSettings s; s.tbsDebounceMs = tbs; s.tobDebounceMs = tob; s.board = b; s.sketchVersion = ver;
  uint8_t rec[kNvmRecordBytes]; encodeRecord(seq, s, rec); nvm.write(addr, rec, sizeof rec);
}
}  // namespace

TEST_CASE("NVM-1: never-set memory (blank or garbage) -> compiled default, nothing is written, nvm shows each failed check") {
  for (int garbage = 0; garbage < 2; garbage++) {
    FakeNvm nvm(garbage ? FakeNvm::Init::kGarbage : FakeNvm::Init::kBlank, 7);
    AppOptions o; o.nvm = &nvm; o.sketchVersion = 0x0801;
    AppRig r(quickWarmConfig(), o);
    r.boot();
    CHECK(r.app->system().tbsDebounceMs() == kBringupDebounceMs);
    CHECK(r.app->system().tobDebounceMs() == kBringupDebounceMs);
    CHECK(nvm.erases == 0);
    r.type("nvm");
    CHECK(r.has("copy A: magic FAIL"));
    CHECK(r.has("checksum FAIL"));
    CHECK(r.has("nothing valid stored"));
  }
}

TEST_CASE("NVM-1: valid stored times for this board replace the default; other board or out of range do not") {
  { FakeNvm nvm; storeRecord(nvm, 0, 1, 12, 7, BoardId::kMinima);
    AppOptions o; o.nvm = &nvm; AppRig r(quickWarmConfig(), o); r.boot();
    CHECK(r.app->system().tbsDebounceMs() == 12); CHECK(r.app->system().tobDebounceMs() == 7);
    r.type("nvm"); CHECK(r.has("magic ok")); CHECK(r.has("WRITE COUNT 1")); }
  { FakeNvm nvm; storeRecord(nvm, 0, 1, 12, 7, BoardId::kWifi);     // measured on the other board
    AppOptions o; o.nvm = &nvm; AppRig r(quickWarmConfig(), o); r.boot();
    CHECK(r.app->system().tbsDebounceMs() == kBringupDebounceMs); }
  { FakeNvm nvm; storeRecord(nvm, 0, 1, 1, 200, BoardId::kMinima);  // out of range
    AppOptions o; o.nvm = &nvm; AppRig r(quickWarmConfig(), o); r.boot();
    CHECK(r.app->system().tbsDebounceMs() == kBringupDebounceMs); }
  { FakeNvm nvm; storeRecord(nvm, 0, 1, 12, 7, BoardId::kMinima, 0x0700);   // written by an older sketch: still used, and it says so
    AppOptions o; o.nvm = &nvm; o.sketchVersion = 0x0801; AppRig r(quickWarmConfig(), o); r.boot();
    CHECK(r.app->system().tbsDebounceMs() == 12); r.run(100); CHECK(r.has("another sketch version")); }
}

TEST_CASE("INP-6: TBS is accepted only after the debounce time; a glitch shorter than that is ignored") {
  FakeNvm nvm; storeRecord(nvm, 0, 1, 20, 20, BoardId::kMinima);
  AppOptions o; o.nvm = &nvm; AppRig r(quickWarmConfig(), o); r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 20000));
  r.tbs(true); r.tick(10); r.tbs(false); r.tick(10);        // 10 ms glitch
  r.run(100); CHECK_FALSE(r.app->system().inputs().tbs);
  r.tbs(true); r.tick(10); CHECK_FALSE(r.app->system().inputs().tbs);
  r.run(30); CHECK(r.app->system().inputs().tbs);
}

TEST_CASE("NVM-1: `debounce set` saves (one erase), takes effect, and is refused while TBS is ON") {
  FakeNvm nvm; AppOptions o; o.nvm = &nvm; o.sketchVersion = 0x0801; AppRig r(quickWarmConfig(), o); r.boot();
  r.type("debounce set 9 8");
  CHECK(r.has("saved: debounce TBS 9 ms, TOB 8 ms, write count 1"));
  CHECK(nvm.erases == 1);
  CHECK(r.app->system().tbsDebounceMs() == 9);
  REQUIRE(r.runUntilMode(App::Mode::kRun, 20000));
  r.tbs(true); r.run(100);
  r.type("debounce set 15 15");
  CHECK(r.has("switch TBS OFF first"));
  CHECK(nvm.erases == 1);
  r.type("debounce set 1 5");
  CHECK(r.has("times must be 2..100 ms"));
}

TEST_CASE("POST-1: TOB held at start is ACKNOWLEDGED (LED PoSt at once, LCD says release TOB) and POST waits for the release") {
  AppRig r;
  r.tob(true);
  r.boot();
  REQUIRE(r.app->mode() == App::Mode::kPostArm);
  r.run(3000);   // longer than the LCD start delay
  CHECK(r.app->mode() == App::Mode::kPostArm);   // still waiting: TOB is held
  CHECK(r.has("TOB seen. Release TOB"));
  CHECK(std::string(r.app->display().lcd().shown(0)).substr(0, 9) == "POST MODE");
  CHECK(std::string(r.app->display().lcd().shown(2)).substr(0, 11) == "RELEASE TOB");
  CHECK_FALSE(r.anyOutputOn());
  r.tob(false);
  r.run(200);
  CHECK(r.app->mode() == App::Mode::kPost);   // the release starts the POST
}

TEST_CASE("POST-1: a TOB stuck down for 10 s does not stop the unit: POST starts anyway, with a warning") {
  AppRig r;
  r.tob(true);
  r.boot();
  r.run(9000);
  CHECK(r.app->mode() == App::Mode::kPostArm);
  r.run(1500);
  CHECK(r.app->mode() != App::Mode::kPostArm);
  CHECK(r.has("TOB still held after 10 s"));
}

namespace {
bool ledShows(AppRig& r, uint8_t segments) {
  for (uint8_t d = 0; d < 4; ++d)
    if ((r.app->display().led().shownSegments(d) & 0x7F) != segments) return false;
  return true;
}
}  // namespace

TEST_CASE("DSP-11/POST-5: a GOOD POST shows 0000 on the LED and POST OK on the LCD for 10 s, then goes back to normal") {
  AppRig r;
  r.tob(true);
  r.boot();
  r.tob(false);
  REQUIRE(r.runUntilMode(App::Mode::kPost, 1000));
  r.run(3000);
  CHECK(ledShows(r, 0x3F));          // 0000
  CHECK(r.app->mode() == App::Mode::kPost);
  CHECK(std::string(r.app->display().lcd().shown(0)).substr(0, 7) == "POST OK");
  r.run(6000);
  CHECK(ledShows(r, 0x3F));          // still up at about 9 s
  REQUIRE(r.runUntilMode(App::Mode::kRun, 4000));
  r.run(2000);
  CHECK(ledShows(r, 0x00));          // TBS is off: the normal LED is blank
}

TEST_CASE("DSP-11: TBS switched ON ends the 0000 / POST OK screen at once") {
  AppRig r;
  r.tob(true);
  r.boot();
  r.tob(false);
  REQUIRE(r.runUntilMode(App::Mode::kPost, 1000));
  r.run(3000);
  REQUIRE(ledShows(r, 0x3F));
  r.tbs(true);
  r.run(600);
  CHECK(r.app->mode() == App::Mode::kRun);
  CHECK_FALSE(ledShows(r, 0x3F));
}

TEST_CASE("DSP-11: after a FAILED POST the LED shows FFFF while it holds, and the LCD names the fault") {
  ControlConfig c = quickWarmConfig();
  c.o2Mandatory = true;
  AppRig r(c);
  r.hal.i2cPresent.erase(0x74);       // no O2 sensor in production: an INHIBIT fault holds the POST
  r.tob(true);
  r.boot();
  r.tob(false);
  REQUIRE(r.runUntilMode(App::Mode::kPost, 1000));
  r.run(2500);
  CHECK(ledShows(r, 0x71));           // FFFF as soon as the verdict is known
  r.run(6000);
  CHECK(r.app->mode() == App::Mode::kPost);
  CHECK(r.app->post().holding());
  CHECK(ledShows(r, 0x71));           // and still FFFF while the POST holds for TOB
}

TEST_CASE("PIN-8: `pins` lists every pin with its signal and live level, reads only, drives nothing") {
  AppRig r;
  r.boot();
  r.tbs(true);
  const size_t eventsBefore = r.hal.events.size();
  r.type("PINS");
  CHECK(r.has("D0  TBS"));
  CHECK(r.has("in (pull-up)"));
  CHECK(r.has("LOW  -> ON"));                 // TBS is on: active LOW
  CHECK(r.has("A0  AIR"));
  CHECK(r.has("analog in   raw"));
  CHECK(r.has("I2C SDA"));
  CHECK(r.has("I2C SCL"));
  CHECK(r.has("D2  (unassigned)"));
  CHECK_FALSE(r.anyOutputOn());
  (void)eventsBefore;
}

TEST_CASE("§11/§12: post and bist are refused while a POST or BIST is already running (nothing is queued)") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("post");
  r.run(100);
  REQUIRE(r.app->mode() == App::Mode::kPost);
  r.type("bist");
  CHECK(r.has("BIST refused: the POST or BIST is already running"));
  REQUIRE(r.runUntilMode(App::Mode::kRun, 14000));
  r.run(500);
  CHECK(r.app->mode() == App::Mode::kRun);   // the refused bist did not start itself afterwards
}

TEST_CASE("DSP-6: the `lcd` command reports the driver state; `lcd bus` runs; bad arguments get a usage line") {
  AppRig r;
  r.boot();
  r.run(3000);
  r.type("LCD");
  CHECK(r.has("LCD: ready, healthy yes, I2C errors 0"));
  r.type("lcd bus 5");
  CHECK(r.has("LCD bus test: 5 rounds"));
  r.type("lcd frob");
  CHECK(r.has("usage: lcd | lcd reinit | lcd bus [rounds]"));
}

// ---------------------------------------------------------------------------- BIST resume after a reset-button reset
namespace {
ResetInfo buttonReset() { ResetInfo i; i.known = true; return i; }
ResetInfo powerOnReset() { ResetInfo i; i.known = true; i.powerOn = true; return i; }
ResetInfo watchdogReset() { ResetInfo i; i.known = true; i.watchdog = true; return i; }
}  // namespace

TEST_CASE("BIST-7: a reset-button reset in the middle of a BIST resumes it at the same step, with the verdicts so far") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("bist");
  r.run(100);
  r.type("p");          // banner: pass
  r.type("s");          // switches: skipped
  REQUIRE(r.app->bist().current() == BistStep::kI2c);
  r.boot(buttonReset());      // the RESET button: same RAM, new App
  CHECK(r.app->mode() == App::Mode::kBist);
  r.run(100);
  CHECK(r.app->bist().current() == BistStep::kI2c);
  CHECK(r.app->bist().verdict(BistStep::kBanner) == BistVerdict::kPass);
  CHECK(r.app->bist().verdict(BistStep::kSwitches) == BistVerdict::kSkip);
  CHECK(r.has("BIST RESUMED after a reset at step 2 I2C scan"));
  CHECK_FALSE(r.anyOutputOn());
}

TEST_CASE("BIST-7: no resume after a power-on, a watchdog reset, when TBS is ON, or when the BIST was quit") {
  for (int how = 0; how < 4; ++how) {
    AppRig r;
    r.boot();
    REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
    r.type("bist");
    r.run(100);
    r.type("p");
    if (how == 3) { r.type("q"); r.run(100); }       // quit: the record says "not running"
    if (how == 2) r.tbs(true);                        // TBS ON at the reset
    r.boot(how == 0 ? powerOnReset() : (how == 1 ? watchdogReset() : buttonReset()));
    r.run(100);
    INFO("case " << how);
    CHECK(r.app->mode() == App::Mode::kRun);
  }
}

TEST_CASE("BIST-7: after a resume, q ends the BIST and the next reset-button reset starts normally") {
  AppRig r;
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("bist");
  r.run(100);
  r.boot(buttonReset());
  REQUIRE(r.app->mode() == App::Mode::kBist);
  r.type("q");
  r.run(200);
  CHECK(r.app->mode() == App::Mode::kRun);
  r.boot(buttonReset());
  CHECK(r.app->mode() == App::Mode::kRun);
}

// ---------------------------------------------------------------------------- run-time mode (DIAG / BENCH / FIELD)
TEST_CASE("MODE-1: `mode` reports it; diag is immediate; bench/field need the word confirm and TBS OFF; nothing starts by itself") {
  AppOptions o; o.controllersEnabled = false;       // compiled as DIAG
  AppRig r(quickWarmConfig(), o);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  CHECK(r.app->runMode() == RunMode::kDiag);
  r.type("mode");
  CHECK(r.has("mode DIAG: controllers OFF, O2 optional."));
  r.type("MODE bench");
  CHECK(r.has("ENABLES the controllers. Type  mode bench confirm"));
  CHECK(r.app->runMode() == RunMode::kDiag);
  r.tbs(true); r.run(100);
  r.type("mode bench confirm");
  CHECK(r.has("mode change refused: the system is enabled (TBS is ON)"));
  r.tbs(false); r.run(200);
  r.type("mode bench confirm");
  CHECK(r.has("mode BENCH. Controllers ON; outputs are off until TBS is switched ON."));
  CHECK(r.app->runMode() == RunMode::kBench);
  r.run(2000);
  CHECK_FALSE(r.anyOutputOn());            // TBS is OFF: nothing starts by itself
  r.tbs(true);
  r.run(9000);
  CHECK(r.outputOn(Signal::kSsr));          // the normal TBS rule now runs the controllers
  r.type("mode diag");
  CHECK(r.has("mode DIAG. Controllers OFF"));
  r.run(100);
  CHECK_FALSE(r.anyOutputOn());            // diag: every output off at once, even with TBS ON
  r.type("mode frob");
  CHECK(r.has("usage: mode"));
}

TEST_CASE("MODE-2: the mode survives a reset-button or watchdog reset, but not a power-on, a brown-out or a bad record") {
  for (int how = 0; how < 5; ++how) {
    AppOptions o; o.controllersEnabled = false;
    AppRig r(quickWarmConfig(), o);
    r.boot();
    REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
    r.type("mode field confirm");
    REQUIRE(r.app->runMode() == RunMode::kField);
    ResetInfo ri;
    ri.known = true;
    if (how == 0) { /* reset button */ }
    if (how == 1) ri.watchdog = true;
    if (how == 2) ri.powerOn = true;
    if (how == 3) ri.brownout = true;
    if (how == 4) r.modeRec.check ^= 1;           // corrupted breadcrumb
    r.boot(ri);
    INFO("case " << how);
    const bool kept = how <= 1;
    CHECK(r.app->runMode() == (kept ? RunMode::kField : RunMode::kDiag));
    CHECK(r.app->system().controllersEnabled() == kept);
    CHECK(r.app->system().config().o2Mandatory == kept);
  }
}

TEST_CASE("MODE-3: FIELD means the O2 sensor is mandatory (no sensor: everything off); the version leaves the LCD; `mode` is refused during BIST") {
  AppOptions o; o.controllersEnabled = false;
  AppRig r(quickWarmConfig(), o);
  r.o2.presentOk = false;                  // no O2 sensor answers
  r.hal.i2cPresent.erase(0x74);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("mode field confirm");
  REQUIRE(r.app->runMode() == RunMode::kField);
  r.tbs(true);
  r.run(9000);
  CHECK_FALSE(r.anyOutputOn());            // INV-9: the O2 sensor is missing
  r.tbs(false); r.run(200);
  r.type("mode diag");
  r.type("bist");
  r.run(100);
  REQUIRE(r.app->mode() == App::Mode::kBist);
  r.type("mode field confirm");           // during a BIST every typed line goes to the BIST, never to the mode command
  r.run(100);
  CHECK(r.app->runMode() == RunMode::kDiag);
}

TEST_CASE("MODE-4: a mode record written by another build (a new upload) is ignored: the new build starts in its compiled mode") {
  AppOptions o; o.controllersEnabled = false;
  AppRig r(quickWarmConfig(), o);
  r.boot();
  REQUIRE(r.runUntilMode(App::Mode::kRun, 3000));
  r.type("mode field confirm");
  REQUIRE(r.app->runMode() == RunMode::kField);
  r.info.time = "12:34:56";                       // the same board, a different build (new upload), reset-button style
  ResetInfo ri; ri.known = true;
  r.boot(ri);
  CHECK(r.app->runMode() == RunMode::kDiag);
  CHECK_FALSE(r.app->system().controllersEnabled());
}
