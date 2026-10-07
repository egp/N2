#include "LedCheck.h"

#include <string.h>

namespace n2 {

CheckResult LedCheck::post(uint32_t) {
  CheckResult r;
  const uint8_t answering = led_.answeringAddresses();
  if (answering == 5 && led_.healthy()) r.set(CheckLevel::kPass, "0x%02X + 4 digits ok", static_cast<unsigned>(board_.addrLed));
  else if (answering == 0) r.set(CheckLevel::kInfo, "no answer: carrying on without the LED");
  else r.set(CheckLevel::kInfo, "only %u of 5 addresses answer", static_cast<unsigned>(answering));
  return r;
}

void LedCheck::enter(Phase phase, uint32_t now, uint32_t lengthMs) {
  phase_ = phase;
  phaseEnd_.arm(now, lengthMs);
  step_.arm(now, 400);
  digit_ = 0;
  blinkState_ = true;
}

void LedCheck::bistBegin(uint32_t now) {
  LedText t;
  memcpy(t.digit, "8888", 5);
  t.dotAfter = 1;
  led_.setText(t);
  log_.write(LogLevel::kInfo, "   LED: 8888 with one decimal point (all segments)");
  enter(Phase::kAll, now, 3000);
}

BistProgress LedCheck::bistTick(uint32_t now) {
  if (phase_ == Phase::kAsk) return BistProgress::kAskOperator;
  if (phase_ == Phase::kCount && step_.reached(now)) {
    step_.arm(now, 400);
    if (digit_ < 10) {
      LedText t;
      const char c = static_cast<char>('0' + digit_);
      t.digit[0] = t.digit[1] = t.digit[2] = t.digit[3] = c;
      t.digit[4] = '\0';
      t.dotAfter = -1;
      led_.setText(t);
      ++digit_;
    }
  }
  if (phase_ == Phase::kBlink && step_.reached(now)) {
    step_.arm(now, 1000);
    blinkState_ = !blinkState_;
    led_.setDisplayOn(blinkState_);
  }
  if (phaseEnd_.reached(now)) {
    if (phase_ == Phase::kAll) {
      log_.write(LogLevel::kInfo, "   LED: counting 0000 .. 9999");
      enter(Phase::kCount, now, 4600);
    } else if (phase_ == Phase::kCount) {
      log_.write(LogLevel::kInfo, "   LED: display off for 1 s, then on");
      enter(Phase::kBlink, now, 2500);
      step_.arm(now, 500);
    } else {
      led_.setDisplayOn(true);
      phase_ = Phase::kAsk;
    }
  }
  return phase_ == Phase::kAsk ? BistProgress::kAskOperator : BistProgress::kRunning;
}

void LedCheck::bistEnd() { led_.setDisplayOn(true); }

}  // namespace n2
