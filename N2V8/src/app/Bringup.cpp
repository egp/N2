#include "Bringup.h"

#include <stdio.h>
#include <string.h>

namespace n2 {

Bringup::Bringup(Hal& hal, const BoardDef& board, const BuildInfo& info, FrameSink* matrix, BringupOptions options)
    : hal_(hal),
      board_(board),
      info_(info),
      matrix_(matrix),
      opt_(options),
      console_(hal, options.logLevel),
      log_(console_, wall_, hal),
      lcd_(hal, board.addrLcd),
      rtc_(hal, board.addrRtc),
      lcdCheck_(hal, lcd_, board.addrLcd, log_),
      rtcCheck_(rtc_, log_),
      checks_{&resetCheck_, &lcdCheck_, &rtcCheck_},
      selfTest_(console_, checks_, 3),
      commands_(StageContext{hal, board, info, console_, selfTest_, rtc_, wall_, loopStats_, *this, resetText_,
                                     [](void* o) { return static_cast<Bringup*>(o)->switchTbs(); },
                                     [](void* o) { return static_cast<Bringup*>(o)->switchTob(); }, this}) {}

bool Bringup::tobPressed() {
  const SignalDef& d = def(board_, Signal::kTob);
  return isOn(hal_.digitalRead(d.pin), d.active);
}

bool Bringup::tbsOn() {
  const SignalDef& d = def(board_, Signal::kTbs);
  return isOn(hal_.digitalRead(d.pin), d.active);
}

// Every switch change is one log line, so the operator sees at once whether the firmware noticed a press.
void Bringup::logSwitchChanges() {
  const bool tbs = tbsOn(), tob = tobPressed();
  if (tbs != tbsWas_) logf(log_, LogLevel::kInfo, "TBS %s", tbs ? "ON" : "off");
  if (tob != tobWas_) logf(log_, LogLevel::kInfo, "TOB %s", tob ? "pressed" : "released");
  const bool changed = tbs != tbsWas_ || tob != tobWas_;
  tbsWas_ = tbs;
  tobWas_ = tob;
  if (changed) screenRefresh_.arm(hal_.millis(), 0);  // show it on the matrix now, not at the next 250 ms tick
}

void Bringup::setup() {
  hal_.consoleBegin();
  hal_.i2cBegin();
  const SignalDef& tob = def(board_, Signal::kTob);
  hal_.pinMode(tob.pin, tob.active == Active::kLow ? PinMode::kInputPullup : PinMode::kInput);
  const SignalDef& tbs = def(board_, Signal::kTbs);
  hal_.pinMode(tbs.pin, tbs.active == Active::kLow ? PinMode::kInputPullup : PinMode::kInput);
  tbsWas_ = tbsOn();
  tobWas_ = tobPressed();
  reset_ = hal_.readResetCause();
  resetCheck_.setResetInfo(reset_);
  snprintf(resetText_, sizeof resetText_, "%s", ResetCheck::causeText(reset_));

  const uint32_t now = hal_.millis();
  bootMs_ = now;
  lcd_.begin(now);
  lcd_.setScreen(renderBanner(info_.version, info_.board, info_.date));
  syncWallClock(now);
  rtcResync_.arm(now, opt_.rtcResyncMs);
  bannerRepeat_.arm(now, 0);
  screenRefresh_.arm(now, opt_.screenMs);
  selfTest_.postBegin(now);
  if (opt_.watchdogEnabled) hal_.watchdogBegin(opt_.watchdogMs);
}

void Bringup::startPost() {
  goRequested_ = false;
  selfTest_.postBegin(hal_.millis());
}

bool Bringup::startBist() { return selfTest_.bistBegin(hal_.millis()); }

// RTC-7: the RTC is read now and then to anchor the log clock; no scheduling ever depends on it. Absent or untrusted: the
// log simply carries no wall-clock stamp, and the program carries on.
void Bringup::syncWallClock(uint32_t now) {
  DateTime t;
  bool trusted = false;
  if (rtc_.present() && rtc_.timeValid(trusted) && trusted && rtc_.read(t)) wall_.sync(t, now);
  else wall_.unsync();
}

void Bringup::printBanner() {
  char line[96];
  snprintf(line, sizeof line, "N2V8 %s stage 1 | board %s | built %s %s | reset: %s", info_.version, info_.board, info_.date,
           info_.time, resetText_);
  console_.tryPrint(line);
  console_.tryPrint("Type help for commands.");
}

void Bringup::updateScreens(uint32_t now) {
  const uint8_t n = selfTest_.checkCount();
  CheckLevel levels[kMaxChecks];
  for (uint8_t i = 0; i < n; ++i) levels[i] = selfTest_.postResult(i).level;

  // ---- LCD (not while a BIST step owns it) ----
  if (!selfTest_.bistRunning() && now - bootMs_ >= 1000) {
    char r1[24] = "", r2[24], r3[24];
    for (uint8_t i = 0; i < n && i < 3; ++i) {
      const CheckLevel l = levels[i];
      char item[10];
      snprintf(item, sizeof item, "%.3s:%s ", selfTest_.checkName(i),
               l == CheckLevel::kPass ? "ok" : (l == CheckLevel::kInfo ? "i" : (l == CheckLevel::kFail ? "FAIL" : "-")));
      strncat(r1, item, sizeof r1 - strlen(r1) - 1);
    }
    char stamp[20];
    if (wall_.stamp(now, stamp)) snprintf(r2, sizeof r2, "%s", stamp);
    else snprintf(r2, sizeof r2, "no clock (RTC)");
    if (selfTest_.postHolding()) snprintf(r3, sizeof r3, "HOLD: TOB or `go`");
    else if (!selfTest_.postFinished()) snprintf(r3, sizeof r3, "POST running");
    else snprintf(r3, sizeof r3, "up %lus  %s", static_cast<unsigned long>(now / 1000u), resetText_);
    char r0[24];
    snprintf(r0, sizeof r0, "N2V8 %s s1", info_.version);
    lcd_.setScreen(makeScreen(r0, r1, r2, r3));
  }

  // ---- matrix ----
  if (matrix_ != nullptr) {
    heartbeat_ = (now / 500u) % 2u == 0;  // 1 Hz blink
    uint8_t left, right;
    if (selfTest_.bistRunning()) {
      left = static_cast<uint8_t>(selfTest_.bistIndex() + 1);
      const BistVerdict v = selfTest_.bistIndex() > 0 ? selfTest_.bistVerdict(selfTest_.bistIndex() - 1) : BistVerdict::kNotRun;
      right = v == BistVerdict::kPass ? 1 : (v == BistVerdict::kFail ? 0xF : (v == BistVerdict::kSkip ? 2 : 0));
    } else if (!selfTest_.postFinished()) {
      const uint8_t i = selfTest_.postIndex() > 0 ? static_cast<uint8_t>(selfTest_.postIndex() - 1) : 0;
      left = static_cast<uint8_t>(i + 1);
      right = matrixGlyphForLevel(levels[i]);
    } else {
      left = matrixGlyphForLevel(selfTest_.postLevel());
      right = 0;
    }
    uint32_t frame[3];
    matrixDrawStatus(frame, left, right, levels, n, heartbeat_);
    if (tbsWas_) matrixSetPixel(frame, 7, 9);                        // TBS ON: one extra pixel on the bottom row
    if (tobWas_) frame[0] = frame[1] = frame[2] = 0xFFFFFFFFu;       // TOB held: the whole matrix lights
    matrix_->show(frame);
  }
}

void Bringup::loop() {
  const uint32_t startUs = hal_.micros();
  const uint32_t now = hal_.millis();

  lcd_.service(now);
  console_.poll(commands_);
  logSwitchChanges();

  // The banner: on the WiFi board (cannot see the PC) repeat until the PC has typed something; otherwise once per attach.
  const bool attached = console_.attached();
  if (!hal_.consoleCanDetectHost()) {
    if (console_.received() == 0 && bannerRepeat_.reached(now)) {
      printBanner();
      bannerRepeat_.arm(now, opt_.bannerRepeatMs);
    }
  } else if (attached && !wasAttached_) {
    printBanner();
  }
  wasAttached_ = attached;

  if (selfTest_.bistRunning()) {
    selfTest_.bistStep(now);
    console_.setLineHook(&selfTest_);
  } else {
    console_.setLineHook(nullptr);
    selfTest_.postStep(now, tobPressed() || goRequested_);
    if (selfTest_.postFinished()) goRequested_ = false;
  }

  if (rtcResync_.reached(now)) {
    syncWallClock(now);
    rtcResync_.arm(now, opt_.rtcResyncMs);
  }
  if (screenRefresh_.reached(now)) {
    updateScreens(now);
    screenRefresh_.arm(now, opt_.screenMs);
  }

  loopStats_.record(hal_.micros() - startUs);
  if (opt_.watchdogEnabled) hal_.watchdogRefresh();
}

}  // namespace n2
