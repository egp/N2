#include "Led1650.h"

#include "../core/TimedState.h"

namespace n2 {

uint8_t Led1650::segmentsFor(char c) {
  static const uint8_t kHex[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
                                   0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};
  if (c >= '0' && c <= '9') return kHex[c - '0'];
  if (c >= 'A' && c <= 'F') return kHex[10 + (c - 'A')];
  if (c == '-') return 0x40;
  return 0x00;
}

void Led1650::begin(uint32_t now) {
  state_ = State::kPowerWait;
  until_ = now + kPowerUpMs;
  controlDirty_ = true;
  for (uint8_t i = 0; i < kDigits; ++i) writtenValid_[i] = false;
}

void Led1650::setText(const LedText& text) {
  for (uint8_t i = 0; i < kDigits; ++i)
    desired_[i] = static_cast<uint8_t>(segmentsFor(text.digit[i]) | (text.dotAfter == static_cast<int8_t>(i) ? 0x80 : 0x00));
}

void Led1650::setDisplayOn(bool on) {
  if (on != displayOn_) {
    displayOn_ = on;
    controlDirty_ = true;
  }
}

bool Led1650::inSync() const {
  for (uint8_t i = 0; i < kDigits; ++i)
    if (!writtenValid_[i] || written_[i] != desired_[i]) return false;
  return !controlDirty_;
}

void Led1650::fail(uint32_t now) {
  healthy_ = false;
  ++errors_;
  state_ = State::kRetry;
  until_ = now + kRetryMs;
}

void Led1650::service(uint32_t now) {
  switch (state_) {
    case State::kIdle:
      return;
    case State::kPowerWait:
      if (!deadlineReached(now, until_)) return;
      state_ = State::kReady;
      break;
    case State::kRetry:
      if (!deadlineReached(now, until_)) return;
      if (!hal_.i2cProbe(control_)) {  // still not there: keep waiting
        until_ = now + kRetryMs;
        return;
      }
      state_ = State::kReady;  // it is back: re-initialise everything (DSP-7)
      controlDirty_ = true;
      for (uint8_t i = 0; i < kDigits; ++i) writtenValid_[i] = false;
      break;
    case State::kReady:
      break;
  }

  if (refreshMs_ > 0) {
    if (!refresh_.armed()) refresh_.arm(now, refreshMs_);
    else if (refresh_.reached(now)) {
      controlDirty_ = true;
      for (uint8_t i = 0; i < kDigits; ++i) writtenValid_[i] = false;
      refresh_.arm(now, refreshMs_);
    }
  }
  if (controlDirty_) {
    const uint8_t b = displayOn_ ? kControlOn : kControlOff;
    if (!hal_.i2cWrite(control_, &b, 1)) return fail(now);
    controlDirty_ = false;
    healthy_ = true;
  }
  for (uint8_t i = 0; i < kDigits; ++i) {
    if (writtenValid_[i] && written_[i] == desired_[i]) continue;
    const uint8_t seg = desired_[i];
    if (!hal_.i2cWrite(static_cast<uint8_t>(digitBase_ + i), &seg, 1)) return fail(now);
    written_[i] = seg;
    writtenValid_[i] = true;
    healthy_ = true;
  }
}

}  // namespace n2
