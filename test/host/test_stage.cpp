// Stage 1 bring-up: wall clock, device checks, SelfTest runner, stage commands, and the whole Bringup app
// (Requirements §11 POST, §12 BIST, §14b RTC-1..RTC-8, DSP-10).
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "FakeHal.h"
#include "app/Bringup.h"
#include "core/DateTime.h"
#include "core/WallClock.h"
#include "ui/I2cSweep.h"

using namespace n2;

namespace {

struct RecordingMatrix : FrameSink {
  uint32_t last[3] = {0, 0, 0};
  int shown = 0;
  void show(const uint32_t* f) override { for (int i = 0; i < 3; ++i) last[i] = f[i]; ++shown; }
};

struct Rig {
  FakeHal hal;
  RecordingMatrix matrix;
  BuildInfo info{"8.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  std::unique_ptr<Bringup> app;
  std::string out;

  explicit Rig(bool lcd = true, bool rtc = true, bool withMatrix = true) {
    hal.consoleIsAttached = true;
    hal.canDetectHost = false;  // behave like the WiFi board
    hal.resetCause.known = true;
    hal.resetCause.powerOn = true;
    if (lcd) hal.i2cPresent.insert(0x27);
    if (rtc) hal.i2cPresent.insert(0x68);
    const SignalDef& tob = def(kHostBoard, Signal::kTob);
    hal.inputLevel[tob.pin] = levelHigh(false, tob.active);  // TOB not pressed
    BringupOptions opt;
    opt.lcdStartMs = 0;  // LCD at once (the real default waits 2.5 s: see the start-delay tests)
    app.reset(new Bringup(hal, kHostBoard, info, withMatrix ? &matrix : nullptr, opt));
  }
  void setRtc(const DateTime& t) { Rtc3231 r(hal, 0x68); REQUIRE(r.set(t)); }
  void boot() { hal.nowMs = 0; app->setup(); }
  void tick(uint32_t ms = 10) {
    hal.nowMs += ms;
    app->loop();
    out += hal.consoleOut;
    hal.consoleOut.clear();
    hal.drain();
  }
  void run(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 10) tick(10); }
  void type(const std::string& line) { hal.type(line + "\n"); run(60); }
  bool has(const std::string& s) const { return out.find(s) != std::string::npos; }
  void clearOut() { out.clear(); }
};

}  // namespace

// ============================================================================ calendar helpers
TEST_CASE("RTC-7: dateTimeFromSeconds is the inverse of secondsSince2000, including leap days and year ends") {
  const DateTime samples[] = {{2000, 1, 1, 0, 0, 0}, {2024, 2, 29, 23, 59, 59}, {2026, 10, 6, 10, 31, 2},
                              {2099, 12, 31, 23, 59, 59}, {2100, 3, 1, 0, 0, 0}, {2135, 12, 31, 23, 59, 59}};
  for (const DateTime& s : samples) {
    DateTime back{};
    REQUIRE(dateTimeFromSeconds(secondsSince2000(s), back));
    CHECK(secondsBetween(back, s) == 0);
    CHECK(back.year == s.year);
    CHECK(back.month == s.month);
    CHECK(back.day == s.day);
    CHECK(back.hour == s.hour);
  }
  // 32-bit seconds since 2000 reach into February 2136; far beyond any use of this clock.
  DateTime last{};
  REQUIRE(dateTimeFromSeconds(0xFFFFFFFFu, last));
  CHECK(last.year == 2136);
}

TEST_CASE("RTC-7: secondsBetween is signed") {
  CHECK(secondsBetween({2026, 10, 6, 10, 0, 5}, {2026, 10, 6, 10, 0, 2}) == 3);
  CHECK(secondsBetween({2026, 10, 6, 10, 0, 2}, {2026, 10, 6, 10, 0, 5}) == -3);
}

// ============================================================================ wall clock
TEST_CASE("RTC-7: the wall clock is anchored to the RTC and then follows millis(), with no further I2C reads") {
  WallClock w;
  char s[20];
  CHECK_FALSE(w.synced());
  CHECK_FALSE(w.stamp(0, s));
  CHECK(std::string(s) == "");
  w.sync({2026, 10, 6, 23, 59, 58}, 5000);
  REQUIRE(w.stamp(5000, s));
  CHECK(std::string(s) == "2026-10-06 23:59:58");
  REQUIRE(w.stamp(8100, s));
  CHECK(std::string(s) == "2026-10-07 00:00:01");  // rolls over midnight
}

TEST_CASE("RTC-7: the wall clock survives the millis() rollover") {
  WallClock w;
  char s[20];
  w.sync({2026, 1, 1, 0, 0, 0}, 0xFFFFFC00u);  // 1024 ms before the rollover
  REQUIRE(w.stamp(3000, s));                   // 4024 ms later
  CHECK(std::string(s) == "2026-01-01 00:00:04");
}

TEST_CASE("RTC-8: the RTC is only written when untrusted, unreadable, or more than 2 s from the reference") {
  const DateTime ref{2026, 10, 6, 10, 31, 0};
  CHECK_FALSE(rtcNeedsSetting(true, true, {2026, 10, 6, 10, 31, 0}, ref));
  CHECK_FALSE(rtcNeedsSetting(true, true, {2026, 10, 6, 10, 31, 2}, ref));
  CHECK_FALSE(rtcNeedsSetting(true, true, {2026, 10, 6, 10, 30, 58}, ref));
  CHECK(rtcNeedsSetting(true, true, {2026, 10, 6, 10, 31, 3}, ref));
  CHECK(rtcNeedsSetting(true, true, {2026, 10, 6, 10, 30, 57}, ref));
  CHECK(rtcNeedsSetting(true, false, {2026, 10, 6, 10, 31, 0}, ref));  // lost power
  CHECK(rtcNeedsSetting(false, true, {2026, 10, 6, 10, 31, 0}, ref));  // unreadable
}

TEST_CASE("RTC-7: StampedLog prefixes lines with the time when known and leaves them alone when not") {
  struct Capture : LogSink { std::string last; void write(LogLevel, const char* l) override { last = l; } } cap;
  FakeHal hal;
  WallClock w;
  StampedLog log(cap, w, hal);
  log.write(LogLevel::kInfo, "hello");
  CHECK(cap.last == "hello");
  hal.nowMs = 1000;
  w.sync({2026, 10, 6, 10, 31, 2}, 1000);
  log.write(LogLevel::kInfo, "hello");
  CHECK(cap.last == "2026-10-06 10:31:02 hello");
}

// ============================================================================ matrix frame
TEST_CASE("DSP-10: matrix status frame: glyphs, check pixels and heartbeat") {
  uint32_t f[3];
  const CheckLevel levels[3] = {CheckLevel::kPass, CheckLevel::kFail, CheckLevel::kInfo};
  matrixDrawStatus(f, 2, 0xF, levels, 3, true);
  CHECK(matrixGetPixel(f, 7, 0));        // pass
  CHECK_FALSE(matrixGetPixel(f, 7, 1));  // fail is dark
  CHECK(matrixGetPixel(f, 7, 2));        // info is lit
  CHECK(matrixGetPixel(f, 7, 11));       // heartbeat
  CHECK(matrixGetPixel(f, 0, 1));        // top of the '2' glyph
  matrixDrawStatus(f, 0, 0, levels, 0, false);
  CHECK_FALSE(matrixGetPixel(f, 7, 11));
  CHECK(matrixGlyphForLevel(CheckLevel::kFail) == 0xF);
  CHECK(matrixGlyphForLevel(CheckLevel::kPass) == 1);
}

// ============================================================================ device checks
TEST_CASE("POST: reset check never holds; watchdog and unknown are noted as info") {
  ResetCheck c;
  ResetInfo info;
  info.known = true;
  info.powerOn = true;
  c.setResetInfo(info);
  CHECK(c.post(0).level == CheckLevel::kPass);
  CHECK(std::string(c.post(0).text) == "power-on");
  info.powerOn = false;
  c.setResetInfo(info);
  CHECK(std::string(c.post(0).text) == "reset button/other");
  info.watchdog = true;
  c.setResetInfo(info);
  CHECK(c.post(0).level == CheckLevel::kInfo);
  c.setResetInfo(ResetInfo());
  CHECK(c.post(0).level == CheckLevel::kInfo);
  CHECK(std::string(c.post(0).text) == "unknown");
}

TEST_CASE("POST: a missing LCD fails POST, a present one passes") {
  NullLogSink log;
  FakeHal hal;
  Lcd20x4 lcd(hal, 0x27);
  LcdCheck check(hal, lcd, 0x27, log);
  CHECK(check.post(0).level == CheckLevel::kFail);
  hal.i2cPresent.insert(0x27);
  CHECK(check.post(0).level == CheckLevel::kPass);
}

TEST_CASE("RTC-5: POST never holds on the RTC: absent, unreadable and NOT SET are all info") {
  NullLogSink log;
  FakeHal hal;
  Rtc3231 rtc(hal, 0x68);
  RtcCheck check(rtc, log);
  CHECK(check.post(0).level == CheckLevel::kInfo);  // absent
  hal.i2cPresent.insert(0x68);
  CHECK(check.post(0).level == CheckLevel::kInfo);  // all-zero registers: not a valid date
  REQUIRE(rtc.set({2026, 10, 6, 10, 31, 2}));
  CheckResult r = check.post(0);
  CHECK(r.level == CheckLevel::kPass);
  CHECK(std::string(r.text) == "2026-10-06 10:31:02");
  hal.i2cRegs[0x68][0x0F] = 0x80;
  r = check.post(0);
  CHECK(r.level == CheckLevel::kInfo);
  CHECK(std::string(r.text).find("NOT SET") != std::string::npos);
}

TEST_CASE("BIST: the RTC step passes when the RTC advances as fast as millis()") {
  struct Capture : LogSink { void write(LogLevel, const char*) override {} } log;
  FakeHal hal;
  hal.i2cPresent.insert(0x68);
  Rtc3231 rtc(hal, 0x68);
  REQUIRE(rtc.set({2026, 10, 6, 10, 31, 2}));
  hal.i2cRegs[0x68][0x11] = 0x19;
  RtcCheck check(rtc, log);
  check.bistBegin(1000);
  CHECK(check.bistTick(2000) == BistProgress::kRunning);
  REQUIRE(rtc.set({2026, 10, 6, 10, 31, 5}));  // three seconds later
  CHECK(check.bistTick(4000) == BistProgress::kPass);
  CHECK(std::string(check.bistNote()) == "25.00 C");
}

TEST_CASE("BIST: the RTC step fails when the clock does not advance, or the chip is gone") {
  NullLogSink log;
  FakeHal hal;
  hal.i2cPresent.insert(0x68);
  Rtc3231 rtc(hal, 0x68);
  REQUIRE(rtc.set({2026, 10, 6, 10, 31, 2}));
  RtcCheck check(rtc, log);
  check.bistBegin(0);
  CHECK(check.bistTick(3000) == BistProgress::kFail);  // the RTC did not move
  CHECK(std::string(check.bistNote()).find("off by") != std::string::npos);
  hal.i2cPresent.erase(0x68);
  check.bistBegin(5000);
  CHECK(check.bistTick(5000) == BistProgress::kFail);
}

// ============================================================================ the whole app
TEST_CASE("Stage 1: boots, runs POST hands-off, and reaches RUN with everything fitted") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(1500);
  CHECK(r.has("N2V8 8.0.0-test stage 1"));
  CHECK(r.has("POST 1/3 RESET"));
  CHECK(r.has("POST 2/3 LCD"));
  CHECK(r.has("POST 3/3 RTC"));
  CHECK(r.has("POST OK"));
  CHECK(r.app->selfTest().postFinished());
  CHECK(r.app->running());
}

TEST_CASE("RTC-5: with no RTC at all, boot carries on gracefully (info, no hold, no time stamps)") {
  Rig r(true, false);
  r.boot();
  r.run(1500);
  CHECK(r.app->selfTest().postFinished());
  CHECK_FALSE(r.app->selfTest().postHolding());
  CHECK(r.has("POST 3/3 RTC    info absent"));
  CHECK_FALSE(r.app->wall().synced());
  r.type("time");
  CHECK(r.has("RTC: not answering"));
  r.type("status");
  CHECK(r.has("not synced"));
}

TEST_CASE("POST holds ONLY on a fault, and `go` or TOB releases it") {
  Rig r(false, true);  // LCD missing -> fail
  r.boot();
  r.run(1500);
  CHECK(r.app->selfTest().postHolding());
  CHECK(r.has("POST FAILED: holding"));
  CHECK_FALSE(r.app->running());
  r.run(5000);
  CHECK(r.app->selfTest().postHolding());  // still held
  r.type("go");
  CHECK(r.app->selfTest().postFinished());
  CHECK(r.has("released"));
}

TEST_CASE("POST hold is released by TOB, but not by a TOB that was already held at power-up") {
  Rig r(false, true);
  const SignalDef& tob = def(kHostBoard, Signal::kTob);
  r.hal.inputLevel[tob.pin] = levelHigh(true, tob.active);  // TOB held from the start
  r.boot();
  r.run(1500);
  CHECK(r.app->selfTest().postHolding());
  r.hal.inputLevel[tob.pin] = levelHigh(false, tob.active);  // released
  r.run(100);
  CHECK(r.app->selfTest().postHolding());
  r.hal.inputLevel[tob.pin] = levelHigh(true, tob.active);   // pressed again
  r.run(100);
  CHECK(r.app->selfTest().postFinished());
}

TEST_CASE("RTC-7: log lines carry the wall-clock time once the RTC is trusted") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(1500);
  CHECK(r.has("2026-10-06 10:31:0"));
  r.type("log debug");
  r.hal.nowMs += 61000;  // after the next re-anchor the clock has moved on by about a minute
  r.run(100);
  r.type("time");
  CHECK(r.has("RTC 2026-10-06 10:31:02 (trusted)"));
}

TEST_CASE("RTC-8: `time set` leaves a good RTC alone and fixes an untrusted or wrong one") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 0});
  r.boot();
  r.run(100);
  r.clearOut();
  r.type("time set 2026-10-06 10:31:01");
  CHECK(r.has("already within 2 s"));
  r.clearOut();
  r.type("time set 2026-10-06 11:00:00");
  CHECK(r.has("RTC set to 2026-10-06 11:00:00"));
  DateTime t{};
  Rtc3231 rtc(r.hal, 0x68);
  REQUIRE(rtc.read(t));
  CHECK(t.hour == 11);
  r.hal.i2cRegs[0x68][0x0F] = 0x80;  // oscillator stopped: untrusted, so even a close time is written
  r.clearOut();
  r.type("time set 2026-10-06 11:00:01");
  CHECK(r.has("RTC set to"));
}

TEST_CASE("Stage 1: the BIST asks the operator for the LCD, decides the RTC itself, and summarises") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(1500);
  r.clearOut();
  r.type("bist");
  CHECK(r.has("BIST started"));
  r.run(200);
  CHECK(r.has("BIST 1/3 RESET"));
  CHECK(r.has("reset cause = "));
  r.type("p");
  CHECK(r.has("BIST 1/3 RESET  PASS"));
  r.run(300);
  CHECK(r.has("BIST 2/3 LCD"));
  CHECK(r.has("LCD row 0: |####################|"));
  r.run(10000);  // pattern, backlight, display phases
  CHECK(r.has("4 readable rows"));
  r.type("f ghost characters");
  CHECK(r.has("BIST 2/3 LCD    FAIL  ghost characters"));
  // the RTC step needs the RTC to move: advance it as real time would
  r.run(200);
  CHECK(r.has("BIST 3/3 RTC"));
  r.run(1000);
  r.setRtc({2026, 10, 6, 10, 31, 5});
  r.run(4000);
  CHECK(r.has("BIST complete: 2 pass, 1 fail, 0 not run or skipped"));
}

TEST_CASE("Stage 1: BIST q quits at once and puts the LCD back to normal") {
  Rig r;
  r.boot();
  r.run(1500);
  r.type("bist");
  r.run(300);
  r.type("p");
  r.run(600);  // now in the LCD step
  r.type("q");
  CHECK(r.has("BIST quit"));
  CHECK_FALSE(r.app->selfTest().bistRunning());
  CHECK(r.app->lcd().backlightOn());
  CHECK(r.app->lcd().displayOn());
}

TEST_CASE("Stage 1: bist is refused while POST is held") {
  Rig r(false, true);
  r.boot();
  r.run(1500);
  r.type("bist");
  CHECK(r.has("BIST cannot start now"));
}

TEST_CASE("Stage 1: help describes every command on one line each") {
  Rig r;
  r.boot();
  r.run(1500);
  r.type("help");
  for (const char* c : {"help ", "ver ", "status ", "post ", "bist ", "go ", "scan ", "time ", "time set", "log ", "loop "})
    CHECK(r.has(std::string("\n") + c));
}

TEST_CASE("Stage 1: scan names the devices it finds") {
  Rig r;
  r.hal.i2cPresent.insert(0x57);
  r.boot();
  r.run(1500);
  r.type("scan");
  CHECK(r.has("0x27  LCD backpack"));
  CHECK(r.has("0x57  RTC module EEPROM"));
  CHECK(r.has("0x68  DS3231 RTC"));
  CHECK(r.has("3 device(s)"));
}

TEST_CASE("Stage 1: the matrix shows progress, a fail glyph while held, and a heartbeat") {
  Rig r(false, true);
  r.boot();
  r.run(1500);
  CHECK(r.matrix.shown > 3);
  CHECK(matrixGetPixel(r.matrix.last, 7, 0));  // the reset check passed (lit)
  uint32_t first[3] = {r.matrix.last[0], r.matrix.last[1], r.matrix.last[2]};
  r.run(500);
  const bool changed = first[0] != r.matrix.last[0] || first[1] != r.matrix.last[1] || first[2] != r.matrix.last[2];
  CHECK(changed);  // heartbeat toggles
}

TEST_CASE("Stage 1: works with no matrix at all (Minima)") {
  Rig r(true, true, false);
  r.boot();
  r.run(1500);
  CHECK(r.app->running());
}

TEST_CASE("NFR-1: a loop pass is short: no I2C traffic piles up in one pass and the watchdog is refreshed every pass") {
  Rig r;
  r.hal.microsPerCall = 20;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(3000);
  CHECK(r.hal.watchdogTimeoutMs == 4000);
  CHECK(r.hal.watchdogRefreshes >= 290);
  CHECK(r.app->loopStats().maxUs() < 5000);
}

TEST_CASE("Stage 1: a USB console that is not attached never stalls the loop") {
  Rig r;
  r.hal.consoleIsAttached = false;
  r.boot();
  r.run(2000);
  CHECK(r.hal.watchdogRefreshes >= 190);
}

TEST_CASE("Stage 1: the loop command's line fits the console line width") {
  Rig r;
  r.boot();
  r.run(500);
  r.type("loop");
  CHECK(r.has("| >=1 s: 0"));
}

TEST_CASE("Stage 1: switch changes are logged and shown in status") {
  Rig r;
  const SignalDef& tbs0 = def(kHostBoard, Signal::kTbs);
  r.hal.inputLevel[tbs0.pin] = levelHigh(false, tbs0.active);  // TBS off at the start
  r.boot();
  r.run(500);
  const SignalDef& tob = def(kHostBoard, Signal::kTob);
  r.hal.inputLevel[tob.pin] = levelHigh(true, tob.active);
  r.run(50);
  CHECK(r.has("TOB pressed"));
  r.type("status");
  CHECK(r.has("switches: TBS off (pin D0 reads HIGH), TOB pressed (pin D1 reads LOW)"));
  r.hal.inputLevel[tob.pin] = levelHigh(false, tob.active);
  r.run(50);
  CHECK(r.has("TOB released"));
  const SignalDef& tbs = def(kHostBoard, Signal::kTbs);
  r.hal.inputLevel[tbs.pin] = levelHigh(true, tbs.active);
  r.run(50);
  CHECK(r.has("TBS ON"));
}

TEST_CASE("Stage 1: the matrix lights fully while TOB is held, and TBS ON adds a pixel") {
  Rig r;
  const SignalDef& tbs0 = def(kHostBoard, Signal::kTbs);
  r.hal.inputLevel[tbs0.pin] = levelHigh(false, tbs0.active);
  r.boot();
  r.run(1500);
  const SignalDef& tob = def(kHostBoard, Signal::kTob);
  const SignalDef& tbs = def(kHostBoard, Signal::kTbs);
  r.hal.inputLevel[tob.pin] = levelHigh(true, tob.active);
  r.run(30);
  CHECK(r.matrix.last[0] == 0xFFFFFFFFu);
  CHECK(r.matrix.last[2] == 0xFFFFFFFFu);
  r.hal.inputLevel[tob.pin] = levelHigh(false, tob.active);
  r.hal.inputLevel[tbs.pin] = levelHigh(true, tbs.active);
  r.run(30);
  CHECK(r.matrix.last[0] != 0xFFFFFFFFu);
  CHECK(matrixGetPixel(r.matrix.last, 7, 9));
}

// ============================================================================ LCD recovery (DSP-7)
TEST_CASE("DSP-7: refresh() rewrites the whole screen, reinit() restarts the controller, neither blocks") {
  FakeHal hal;
  hal.i2cPresent.insert(0x27);
  Lcd20x4 lcd(hal, 0x27);
  lcd.begin(0);
  lcd.setScreen(makeScreen("ROW0", "ROW1", "ROW2", "ROW3"));
  uint32_t now = 0;
  for (int i = 0; i < 400 && !lcd.inSync(); ++i) lcd.service(now += 10);
  REQUIRE(lcd.inSync());
  const size_t settled = hal.i2cWrites.size();
  for (int i = 0; i < 50; ++i) lcd.service(now += 10);
  CHECK(hal.i2cWrites.size() == settled);  // nothing to write when nothing changed
  lcd.refresh();
  CHECK_FALSE(lcd.inSync());
  for (int i = 0; i < 400 && !lcd.inSync(); ++i) lcd.service(now += 10);
  CHECK(lcd.inSync());
  CHECK(hal.i2cWrites.size() > settled + 70);  // all 80 cells went out again
  const uint32_t before = lcd.reinitCount();
  lcd.reinit(now);
  CHECK(lcd.reinitCount() == before + 1);
  CHECK_FALSE(lcd.ready());
  for (int i = 0; i < 600 && !lcd.inSync(); ++i) lcd.service(now += 10);
  CHECK(lcd.inSync());
}

TEST_CASE("DSP-7: the stage firmware rewrites the LCD every few seconds and `lcd reinit` works from the console") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(2000);
  size_t writes = r.hal.i2cWrites.size();
  r.run(6000);
  CHECK(r.hal.i2cWrites.size() > writes + 60);  // a periodic full rewrite happened
  r.clearOut();
  r.type("lcd");
  CHECK(r.has("LCD: ready, I2C errors 0, re-initialisations 0"));
  r.clearOut();
  r.type("lcd reinit");
  CHECK(r.has("LCD controller restarting"));
  r.run(1000);
  r.type("lcd");
  CHECK(r.has("re-initialisations 1"));
  CHECK(r.has("in sync"));
  r.type("lcd bogus");
  CHECK(r.has("usage: lcd [reinit | bus [rounds]]"));
}

TEST_CASE("DSP-7: after boot the LCD is rewritten early (250 ms, 500 ms, 1 s...) and then settles to every 5 s") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(600);  // init and first draw settle
  const size_t afterFirst = r.hal.i2cWrites.size();
  r.run(1200);
  CHECK(r.hal.i2cWrites.size() > afterFirst + 70);  // at least one early full rewrite already happened
  r.run(30000);
  const size_t a = r.hal.i2cWrites.size();
  r.run(10000);
  const size_t perTenSeconds = r.hal.i2cWrites.size() - a;
  CHECK(perTenSeconds < 700);  // settled: about two full rewrites (plus the changing clock row) per 10 s
}

TEST_CASE("DSP-7: optional extra LCD re-initialisations (0.3 s and 1.2 s after boot) happen when asked for, and only then") {
  FakeHal hal;
  hal.i2cPresent.insert(0x27);
  RecordingMatrix matrix;
  BuildInfo info{"8.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  BringupOptions opt;
  opt.lcdStartMs = 0;
  opt.lcdReinit1Ms = 300;
  opt.lcdReinit2Ms = 1200;
  Bringup app(hal, kHostBoard, info, &matrix, opt);
  hal.consoleIsAttached = true;
  app.setup();
  auto run = [&](uint32_t ms) { for (uint32_t t = 0; t < ms; t += 10) { hal.nowMs += 10; app.loop(); } };
  run(200);
  CHECK(app.lcd().reinitCount() == 0);
  run(300);
  CHECK(app.lcd().reinitCount() == 1);
  run(1000);
  CHECK(app.lcd().reinitCount() == 2);
  run(20000);
  CHECK(app.lcd().reinitCount() == 2);
  CHECK(app.lcd().inSync());
}

TEST_CASE("DSP-7: with lcdStartMs set, nothing is sent to the LCD until that time") {
  FakeHal hal;
  hal.i2cPresent.insert(0x27);
  RecordingMatrix matrix;
  BuildInfo info{"8.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  BringupOptions opt;
  opt.lcdStartMs = 2500;
  opt.lcdReinit1Ms = 0;
  opt.lcdReinit2Ms = 0;
  Bringup app(hal, kHostBoard, info, &matrix, opt);
  hal.consoleIsAttached = true;
  app.setup();
  for (uint32_t t = 0; t < 2400; t += 10) { hal.nowMs += 10; app.loop(); }
  size_t toLcd = 0;
  for (const auto& w : hal.i2cWrites) if (w.address == 0x27) ++toLcd;
  CHECK(toLcd == 0);
  for (uint32_t t = 0; t < 1500; t += 10) { hal.nowMs += 10; app.loop(); }
  CHECK(app.lcd().ready());
  CHECK(app.lcd().reinitCount() == 0);
}

TEST_CASE("DSP-7: `lcd bus` reports a clean link as all zeros and a corrupted one as mismatches") {
  Rig r;
  r.boot();
  r.run(1500);
  r.clearOut();
  r.type("lcd bus 100");
  CHECK(r.has("LCD bus: 100 rounds, write fail 0, read fail 0, mismatch 0"));
  r.hal.i2cReadXor = 0x10;  // one data line reads back wrong
  r.clearOut();
  r.type("lcd bus 100");
  CHECK(r.has("mismatch 100"));
  r.hal.i2cReadXor = 0;
  r.hal.i2cPresent.erase(0x27);
  r.clearOut();
  r.type("lcd bus 10");
  CHECK(r.has("write fail 10"));
}

TEST_CASE("DSP-7: the bus test never pulses EN, so the LCD itself is not disturbed, and restores the backlight byte") {
  FakeHal hal;
  hal.i2cPresent.insert(0x27);
  Lcd20x4 lcd(hal, 0x27);
  lcd.begin(0);
  const size_t before = hal.i2cWrites.size();
  const Lcd20x4::BusTest t = lcd.busTest(40);
  CHECK(t.rounds == 40);
  for (size_t i = before; i < hal.i2cWrites.size(); ++i) {
    for (uint8_t b : hal.i2cWrites[i].bytes) CHECK((b & 0x07) == 0);  // RS, RW, EN all low
  }
  CHECK(hal.i2cWrites.back().bytes.back() == 0x08);  // backlight on, everything else low
}

TEST_CASE("DSP-7: by default the LCD is not touched for 2.5 s after boot (bench finding), then comes up normally") {
  FakeHal hal;
  hal.i2cPresent.insert(0x27);
  RecordingMatrix matrix;
  BuildInfo info{"8.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  Bringup app(hal, kHostBoard, info, &matrix);  // default options
  hal.consoleIsAttached = true;
  app.setup();
  for (uint32_t t = 0; t < 2400; t += 10) { hal.nowMs += 10; app.loop(); }
  size_t toLcd = 0;
  for (const auto& w : hal.i2cWrites) if (w.address == 0x27) ++toLcd;
  CHECK(toLcd == 0);
  for (uint32_t t = 0; t < 3000; t += 10) { hal.nowMs += 10; app.loop(); }
  CHECK(app.lcd().ready());
  CHECK(app.lcd().inSync());
}

// ============================================================================ I2C speed sweep
TEST_CASE("DRV-1: i2c sweep covers the two speeds the R4 really has (100 k, 400 k), reports a good bus clean, and restores the clock") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(1500);
  r.hal.i2cClockChanges.clear();
  r.clearOut();
  r.type("i2c sweep 20");
  CHECK(r.has("100000 Hz: LCD   0  RTC  0/20  probe  0/20  OK"));
  CHECK(r.has("400000 Hz:"));
  CHECK_FALSE(r.has("BAD"));
  const std::vector<uint32_t> expected{100000, 400000, 100000};
  CHECK(r.hal.i2cClockChanges == expected);
  CHECK(r.hal.i2cClockHz == 100000);
}

TEST_CASE("DRV-1: i2c sweep flags the speed where a device misbehaves") {
  FakeHal hal;
  hal.i2cPresent = {0x27, 0x68};
  Lcd20x4 lcd(hal, 0x27);
  Rtc3231 rtc(hal, 0x68);
  REQUIRE(rtc.set({2026, 10, 6, 10, 31, 2}));
  const uint32_t speeds[2] = {100000, 400000};
  SweepRow rows[2];
  hal.i2cReadXor = 0;
  CHECK(runI2cSweep(hal, lcd, rtc, 0x27, 0x68, speeds, 2, 20, 100000, rows) == 2);
  CHECK(rows[0].ok());
  CHECK(rows[1].ok());
  hal.i2cReadXor = 0x10;  // read-back corrupted: every LCD round is bad
  runI2cSweep(hal, lcd, rtc, 0x27, 0x68, speeds, 2, 20, 100000, rows);
  CHECK(rows[0].lcdBad == 20);
  CHECK_FALSE(rows[0].ok());
  hal.i2cReadXor = 0;
  hal.i2cPresent.erase(0x68);  // the RTC stops answering
  runI2cSweep(hal, lcd, rtc, 0x27, 0x68, speeds, 2, 20, 100000, rows);
  CHECK(rows[1].rtcBad == 20);
  CHECK(rows[1].probeBad == 10);
}

TEST_CASE("DRV-1: i2c sweep refreshes the watchdog between speeds and the loop survives it") {
  Rig r;
  r.setRtc({2026, 10, 6, 10, 31, 2});
  r.boot();
  r.run(1500);
  const uint32_t before = r.hal.watchdogRefreshes;
  r.type("i2c sweep 5");
  CHECK(r.hal.watchdogRefreshes >= before + 2 * 3);  // three refreshes per speed
}

TEST_CASE("DRV-1: i2c without arguments says how to use it") {
  Rig r;
  r.boot();
  r.run(1500);
  r.type("i2c");
  CHECK(r.has("usage: i2c sweep [rounds]"));
}
