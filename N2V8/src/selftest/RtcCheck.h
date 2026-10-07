// RtcCheck.h — POST and BIST for the DS3231 real-time clock (Requirements RTC-1..RTC-8).
//
// The RTC is OPTIONAL: it only supplies time stamps for the log. POST therefore never holds on it (an absent or unset clock
// is "info"), and the rest of the system carries on without it.
// BIST (automatic, no operator answer): reads the time twice, three seconds apart, and checks that the RTC advanced by the
// same amount as millis() (within one second), then checks the chip temperature is plausible.
#pragma once

#include "../core/Log.h"
#include "../core/TimedState.h"
#include "../drivers/Rtc3231.h"
#include "DeviceCheck.h"

namespace n2 {

class RtcCheck : public DeviceCheck {
 public:
  RtcCheck(Rtc3231& rtc, LogSink& log) : rtc_(rtc), log_(log) {}

  const char* name() const override { return "RTC"; }
  CheckResult post(uint32_t now) override;
  void bistBegin(uint32_t now) override;
  BistProgress bistTick(uint32_t now) override;
  const char* bistNote() const override { return note_; }

 private:
  Rtc3231& rtc_;
  LogSink& log_;
  Deadline wait_;
  DateTime first_{};
  uint32_t firstMs_ = 0;
  bool started_ = false;
  char note_[32] = {};
};

}  // namespace n2
