#include "RtcCheck.h"

namespace n2 {

CheckResult RtcCheck::post(uint32_t) {
  CheckResult r;
  if (!rtc_.present()) {
    r.set(CheckLevel::kInfo, "absent: carrying on without time stamps");
    return r;
  }
  bool trusted = false;
  DateTime t;
  if (!rtc_.timeValid(trusted) || !rtc_.read(t)) {
    r.set(CheckLevel::kInfo, "answers, but the time is unreadable");
    return r;
  }
  char when[20];
  formatDateTime(when, t);
  r.set(trusted ? CheckLevel::kPass : CheckLevel::kInfo, trusted ? "%s" : "%s NOT SET (lost power)", when);
  return r;
}

void RtcCheck::bistBegin(uint32_t now) {
  started_ = false;
  note_[0] = '\0';
  wait_.arm(now, 3000);
  firstMs_ = now;
  if (!rtc_.read(first_)) {
    wait_.clear();
    snprintf(note_, sizeof note_, "cannot read time");
    return;
  }
  started_ = true;
  char when[20];
  formatDateTime(when, first_);
  logf(log_, LogLevel::kInfo, "   RTC reads %s; waiting 3 s to see it advance", when);
}

BistProgress RtcCheck::bistTick(uint32_t now) {
  if (!started_) return BistProgress::kFail;
  if (!wait_.reached(now)) return BistProgress::kRunning;
  DateTime second;
  if (!rtc_.read(second)) {
    snprintf(note_, sizeof note_, "second read failed");
    return BistProgress::kFail;
  }
  const int32_t rtcSeconds = secondsBetween(second, first_);
  const int32_t millisSeconds = static_cast<int32_t>((now - firstMs_) / 1000u);
  const int32_t error = rtcSeconds - millisSeconds;
  int16_t tempX100 = 0;
  const bool tempOk = rtc_.temperatureX100(tempX100);
  logf(log_, LogLevel::kInfo, "   RTC advanced %ld s while millis() advanced %ld s; chip %d.%02d C", static_cast<long>(rtcSeconds),
       static_cast<long>(millisSeconds), tempX100 / 100, (tempX100 < 0 ? -tempX100 : tempX100) % 100);
  if (error > 1 || error < -1) {
    snprintf(note_, sizeof note_, "clock off by %lds", static_cast<long>(error));
    return BistProgress::kFail;
  }
  if (!tempOk || tempX100 < 0 || tempX100 > 6000) {
    snprintf(note_, sizeof note_, "temperature implausible");
    return BistProgress::kFail;
  }
  snprintf(note_, sizeof note_, "%d.%02d C", tempX100 / 100, tempX100 % 100);
  return BistProgress::kPass;
}

}  // namespace n2
