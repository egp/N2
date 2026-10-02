#include "Post.h"

#include <stdio.h>

#include "../core/Scaling.h"

namespace n2 {

void Post::say(uint32_t now, const char* check, const char* result) {
  logf(log_, LogLevel::kInfo, "%lu POST %s %s", static_cast<unsigned long>(now), check, result);
}

bool Post::tobPressed() {
  const SignalDef& d = def(board_, Signal::kTob);
  return isOn(hal_.digitalRead(d.pin), d.active);
}

void Post::begin(uint32_t now) {
  phase_ = Phase::kChecks;
  startedAt_ = now;
  check_ = 0;
  samples_ = 0;
  level_ = PostLevel::kPass;
  problems_ = 0;
  rotation_ = 0;
  air_ = SensorChannel();
  n2Low_ = SensorChannel();
  n2High_ = SensorChannel();
  tobWasUp_ = !tobPressed();  // a TOB held since power-up must be released and pressed again to continue
  summaryUntil_.clear();
  rotate_.clear();
  display_.setOverride(renderBanner(info_.version, info_.board, info_.date), LedText{{'8', '8', '8', '8', '\0'}, -1}, now);
}

void Post::runCheck(uint32_t now) {
  const uint32_t hold = sys_.config().faultHoldMs;
  FaultSet& faults = sys_.faultSet();
  switch (check_) {
    case 0: {  // outputs verified safe
      const OutputRequest o = sys_.outputs().actualState();
      const bool safe = !o.left && !o.right && !o.flush && !o.ssr;
      faults.report(FaultId::kInvariant, !safe, now, hold);
      say(now, "1 outputs safe", safe ? "OK" : "FAIL: an output is ON");
      break;
    }
    case 1: {  // reset cause (F30/F31 were raised at boot)
      const char* cause = !reset_.known ? "unknown" : (reset_.powerOn ? "power-on" : (reset_.watchdog ? "watchdog" : (reset_.brownout ? "brown-out" : "reset button / other")));
      say(now, "2 reset cause", cause);
      break;
    }
    case 2: {  // I2C devices against the expected list
      const bool lcd = hal_.i2cProbe(board_.addrLcd);
      const bool led = hal_.i2cProbe(board_.addrLed);
      const bool o2 = hal_.i2cProbe(board_.addrO2);
      faults.report(FaultId::kLcd, !lcd, now, hold);
      faults.report(FaultId::kLed, !led, now, hold);
      if (sys_.config().o2Mandatory) faults.report(FaultId::kO2Comm, !o2, now, hold);
      char r[48];
      snprintf(r, sizeof r, "LCD %s  LED %s  O2 %s", lcd ? "ok" : "MISSING", led ? "ok" : "MISSING",
               o2 ? "ok" : (sys_.config().o2Mandatory ? "MISSING" : "absent (not required)"));
      say(now, "3 I2C", r);
      break;
    }
    case 3: {  // pressure sensors inside their valid window: three consecutive samples, as in operation
      const ControlConfig& cfg = sys_.config();
      const uint8_t bits = kAdcBits;
      const uint16_t a = hal_.analogRead(pinOf(board_, Signal::kAirPressure));
      const uint16_t l = hal_.analogRead(pinOf(board_, Signal::kN2LowPressure));
      const uint16_t h = hal_.analogRead(pinOf(board_, Signal::kN2HighPressure));
      air_.update(a, bits, cfg.sensorFaultSamples);
      n2Low_.update(l, bits, cfg.sensorFaultSamples);
      n2High_.update(h, bits, cfg.sensorFaultSamples);
      if (++samples_ < cfg.sensorFaultSamples) return;  // take more samples on the next passes
      faults.report(FaultId::kAirSensor, air_.asserted(), now, hold);
      faults.report(FaultId::kN2LowSensor, n2Low_.asserted(), now, hold);
      faults.report(FaultId::kN2HighSensor, n2High_.asserted(), now, hold);
      char r[64];
      snprintf(r, sizeof r, "AIR %s  N2L %s  N2H %s  (raw %u %u %u)", air_.asserted() ? "RANGE" : "ok", n2Low_.asserted() ? "RANGE" : "ok",
               n2High_.asserted() ? "RANGE" : "ok", static_cast<unsigned>(a), static_cast<unsigned>(l), static_cast<unsigned>(h));
      say(now, "4 sensors", r);
      break;
    }
    case 4: {  // switches
      const SignalDef& tbs = def(board_, Signal::kTbs);
      char r[32];
      snprintf(r, sizeof r, "TBS %s  TOB %s", isOn(hal_.digitalRead(tbs.pin), tbs.active) ? "ON" : "off", tobPressed() ? "pressed" : "off");
      say(now, "5 switches", r);
      break;
    }
    case 5: {  // configuration sanity
      const bool ok = validControl(sys_.config());
      faults.report(FaultId::kInvariant, !ok, now, hold);
      say(now, "6 config", ok ? "OK" : "INVALID");
      break;
    }
    case 6: {  // build identity
      char r[72];
      snprintf(r, sizeof r, "N2V8 %s %s %s  %s %s  adc %u", info_.version, info_.date, info_.time, info_.board, info_.mode,
               static_cast<unsigned>(info_.adcBits));
      say(now, "7 build", r);
      break;
    }
  }
  ++check_;
}

bool Post::shouldHold() const {
  const FaultSet& f = sys_.faults();
  FaultId id;
  for (uint8_t i = 0; f.nth(i, opt_.hangAt, id); ++i) {
    // F30/F31 only record that the previous run ended badly; they must not keep an unattended unit down.
    if (id != FaultId::kWatchdogReset && id != FaultId::kBrownoutReset) return true;
  }
  return false;
}

void Post::showResult(uint32_t now, uint8_t firstFault, bool holdPrompt) {
  const FaultSet& f = sys_.faults();
  char r0[24], r1[24] = "", r2[24] = "";
  if (level_ == PostLevel::kPass) snprintf(r0, sizeof r0, "POST OK");
  else snprintf(r0, sizeof r0, "POST %s %u", level_ == PostLevel::kFail ? "FAIL" : "WARN", static_cast<unsigned>(problems_));
  FaultId id;
  const char* lines[2] = {nullptr, nullptr};
  char* bufs[2] = {r1, r2};
  for (uint8_t k = 0; k < 2; ++k) {
    if (problems_ > 0 && f.nth(static_cast<uint8_t>((firstFault + k) % problems_), Severity::kInfo, id) && (k == 0 || problems_ > 1)) {
      snprintf(bufs[k], 24, "F%02u %s", static_cast<unsigned>(faultInfo(id).code), faultInfo(id).text);
      lines[k] = bufs[k];
    }
  }
  const Screen s = makeScreen(r0, lines[0] ? lines[0] : "", lines[1] ? lines[1] : "", holdPrompt ? "PRESS TOB TO PROCEED" : "");
  LedText led;
  snprintf(led.digit, sizeof led.digit, "    ");
  led.dotAfter = -1;
  if (problems_ > 0 && f.nth(firstFault % problems_, Severity::kInfo, id)) snprintf(led.digit, sizeof led.digit, " F%02u", static_cast<unsigned>(faultInfo(id).code % 100u));
  display_.setOverride(s, led, now);
}

void Post::finishChecks(uint32_t now) {
  const FaultSet& f = sys_.faults();
  problems_ = f.activeCount(Severity::kInfo);
  level_ = f.anyAtLeast(Severity::kInhibit) ? PostLevel::kFail : (problems_ > 0 ? PostLevel::kWarn : PostLevel::kPass);
  if (level_ == PostLevel::kPass) logf(log_, LogLevel::kInfo, "%lu POST PASS", static_cast<unsigned long>(now));
  else logf(log_, LogLevel::kWarn, "%lu POST %s %u", static_cast<unsigned long>(now), level_ == PostLevel::kFail ? "FAIL" : "WARN", static_cast<unsigned>(problems_));
  showResult(now, 0, shouldHold());
  summaryUntil_.arm(now, level_ == PostLevel::kPass ? opt_.okMs : opt_.problemMs);
  phase_ = Phase::kSummary;
}

bool Post::step(uint32_t now) {
  switch (phase_) {
    case Phase::kDone:
      return true;

    case Phase::kChecks:
      if (check_ < kCheckCount) {
        runCheck(now);
      } else if (static_cast<uint32_t>(now - startedAt_) >= opt_.bannerMs) {  // let the banner be read
        finishChecks(now);
      }
      return false;

    case Phase::kSummary:
      if (!summaryUntil_.reached(now)) return false;
      if (shouldHold()) {
        phase_ = Phase::kHold;
        rotate_.arm(now, opt_.holdRotateMs);
        logf(log_, LogLevel::kWarn, "%lu POST holding: fault found, press TOB to continue", static_cast<unsigned long>(now));
        return false;
      }
      display_.clearOverride();
      phase_ = Phase::kDone;
      return true;

    case Phase::kHold: {
      if (rotate_.reached(now)) {  // step through the faults
        ++rotation_;
        showResult(now, rotation_, true);
        rotate_.arm(now, opt_.holdRotateMs);
      }
      const bool pressed = tobPressed();
      if (pressed && tobWasUp_) {  // a fresh press
        logf(log_, LogLevel::kWarn, "%lu POST released by operator (TOB) with %u fault(s) active", static_cast<unsigned long>(now),
             static_cast<unsigned>(problems_));
        display_.clearOverride();
        phase_ = Phase::kDone;
        return true;
      }
      tobWasUp_ = !pressed;
      return false;
    }
  }
  return false;
}

}  // namespace n2
