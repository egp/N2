// LedCheck.h — POST and BIST for the 4-digit TM1650 LED display (Requirements DRV-1..DRV-3, BIST-5).
//
// POST: the control address and all four digit addresses acknowledge. The LED only repeats information the LCD and console already show, so a
//       missing or partly missing LED is "info" (no hold), like the RTC.
// BIST: shows 8888 with a decimal point, counts 0000..9999, blinks the display off and on, and asks the operator what they saw.
//       The segments can only be judged by eye, so the operator answers p/f. (Validated on the bench 2026-10-07, docs/results/led-test-wifi-20261007.md.)
#pragma once

#include "../BoardPins.h"
#include "../core/Log.h"
#include "../core/TimedState.h"
#include "../drivers/Led1650.h"
#include "DeviceCheck.h"

namespace n2 {

class LedCheck : public DeviceCheck {
 public:
  LedCheck(Hal& hal, Led1650& led, const BoardDef& board, LogSink& log) : hal_(hal), led_(led), board_(board), log_(log) {}

  const char* name() const override { return "LED"; }
  CheckResult post(uint32_t now) override;
  void bistBegin(uint32_t now) override;
  BistProgress bistTick(uint32_t now) override;
  const char* bistPrompt() const override { return "8888 with one dot, count 0-9 on all digits, one blink?"; }
  void bistEnd() override;

 private:
  enum class Phase : uint8_t { kAll, kCount, kBlink, kAsk };
  void enter(Phase phase, uint32_t now, uint32_t lengthMs);

  Hal& hal_;
  Led1650& led_;
  const BoardDef& board_;
  LogSink& log_;
  Phase phase_ = Phase::kAsk;
  Deadline phaseEnd_;
  Deadline step_;
  uint8_t digit_ = 0;
  bool blinkState_ = true;
};

}  // namespace n2
