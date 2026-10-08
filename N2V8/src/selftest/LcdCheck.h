// LcdCheck.h — POST and BIST for the 20x4 LCD (Requirements DRV-1..DRV-3, BIST-5).
//
// POST: the backpack acknowledges its I2C address and the driver reports itself healthy. A missing LCD FAILS POST (the
//       operator then has only the console).
// BIST: shows a full-width test pattern, blinks the backlight, then blinks the display, and asks the operator what they saw.
//       The same four rows are printed on the console so they can be compared with the real display.
#pragma once

#include "../core/Log.h"
#include "../core/TimedState.h"
#include "../drivers/Lcd20x4.h"
#include "DeviceCheck.h"

namespace n2 {

class LcdCheck : public DeviceCheck {
 public:
  LcdCheck(Hal& hal, Lcd20x4& lcd, uint8_t address, LogSink& log) : hal_(hal), lcd_(lcd), address_(address), log_(log) {}

  // A build can leave the LCD out completely (-DSTAGE2_NO_LCD): the driver is never started and nothing is sent to its address, which matters
  // because a TM1650 LED module also answers 0x24-0x27 and would hear everything sent to an LCD at 0x27 (the LCD is at 0x23 now).
  void setEnabled(bool on) { enabled_ = on; }
  const char* name() const override { return "LCD"; }
  CheckResult post(uint32_t now) override;
  void bistBegin(uint32_t now) override;
  BistProgress bistTick(uint32_t now) override;
  const char* bistPrompt() const override { return "4 readable rows, then backlight blink, then display blink?"; }
  void bistEnd() override;

 private:
  enum class Phase : uint8_t { kPattern, kBacklight, kDisplay, kAsk };
  void enter(Phase phase, uint32_t now, uint32_t lengthMs);

  Hal& hal_;
  Lcd20x4& lcd_;
  uint8_t address_;
  LogSink& log_;
  Phase phase_ = Phase::kAsk;
  Deadline phaseEnd_;
  Deadline blink_;
  bool blinkState_ = true;
  bool enabled_ = true;
};

}  // namespace n2
