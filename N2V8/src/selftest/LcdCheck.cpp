#include "LcdCheck.h"

#include "../ui/LcdScreens.h"

namespace n2 {

CheckResult LcdCheck::post(uint32_t) {
  CheckResult r;
  if (!enabled_) {
    r.set(CheckLevel::kInfo, "disabled in this build");
    return r;
  }
  const bool answers = hal_.i2cProbe(address_);
  if (!answers) r.set(CheckLevel::kFail, "no answer at 0x%02X", static_cast<unsigned>(address_));
  else if (!lcd_.healthy()) r.set(CheckLevel::kFail, "0x%02X answers, driver reports errors", static_cast<unsigned>(address_));
  else r.set(CheckLevel::kPass, "0x%02X ok", static_cast<unsigned>(address_));
  return r;
}

void LcdCheck::enter(Phase phase, uint32_t now, uint32_t lengthMs) {
  phase_ = phase;
  phaseEnd_.arm(now, lengthMs);
  blink_.arm(now, 500);
  blinkState_ = true;
}

void LcdCheck::bistBegin(uint32_t now) {
  if (!enabled_) {
    phase_ = Phase::kAsk;  // nothing to show: the step asks at once and the operator skips it with s
    return;
  }
  const Screen pattern = makeScreen("####################", "ABCDEFGHIJKLMNOPQRST", "01234567890123456789", "####################");
  lcd_.setScreen(pattern);
  for (uint8_t i = 0; i < kLcdRows; ++i) logf(log_, LogLevel::kInfo, "   LCD row %u: |%s|", static_cast<unsigned>(i), pattern.row[i]);
  enter(Phase::kPattern, now, 3000);
}

BistProgress LcdCheck::bistTick(uint32_t now) {
  if (phase_ == Phase::kAsk) return BistProgress::kAskOperator;
  if (phase_ != Phase::kPattern && blink_.reached(now)) {
    blink_.arm(now, 500);
    blinkState_ = !blinkState_;
    if (phase_ == Phase::kBacklight) lcd_.setBacklight(blinkState_);
    else lcd_.setDisplayOn(blinkState_);
  }
  if (phaseEnd_.reached(now)) {
    if (phase_ == Phase::kPattern) {
      log_.write(LogLevel::kInfo, "   LCD: backlight blinking");
      enter(Phase::kBacklight, now, 3000);
    } else if (phase_ == Phase::kBacklight) {
      lcd_.setBacklight(true);
      log_.write(LogLevel::kInfo, "   LCD: display blinking (text should vanish and return)");
      enter(Phase::kDisplay, now, 3000);
    } else {
      bistEnd();
      phase_ = Phase::kAsk;
    }
  }
  return phase_ == Phase::kAsk ? BistProgress::kAskOperator : BistProgress::kRunning;
}

void LcdCheck::bistEnd() {
  lcd_.setBacklight(true);
  lcd_.setDisplayOn(true);
}

}  // namespace n2
