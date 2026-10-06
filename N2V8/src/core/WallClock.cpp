#include "WallClock.h"

#include <stdio.h>

namespace n2 {

void WallClock::sync(const DateTime& t, uint32_t nowMs) {
  anchorSeconds_ = secondsSince2000(t);
  anchorMs_ = nowMs;
  synced_ = true;
}

bool WallClock::now(uint32_t nowMs, DateTime& out) const {
  if (!synced_) return false;
  const uint32_t elapsedSeconds = (nowMs - anchorMs_) / 1000u;  // unsigned subtraction: safe across the millis() rollover
  return dateTimeFromSeconds(anchorSeconds_ + elapsedSeconds, out);
}

bool WallClock::stamp(uint32_t nowMs, char* out) const {
  DateTime t;
  if (!now(nowMs, t)) {
    out[0] = '\0';
    return false;
  }
  formatDateTime(out, t);
  return true;
}

bool rtcNeedsSetting(bool rtcReadable, bool rtcTrusted, const DateTime& rtcTime, const DateTime& reference,
                     uint32_t toleranceSeconds) {
  if (!rtcReadable || !rtcTrusted) return true;
  int32_t diff = secondsBetween(rtcTime, reference);
  if (diff < 0) diff = -diff;
  return static_cast<uint32_t>(diff) > toleranceSeconds;
}

void StampedLog::write(LogLevel level, const char* line) {
  char stamp[20];
  if (!clock_.stamp(hal_.millis(), stamp)) {
    next_.write(level, line);
    return;
  }
  char buf[120];
  snprintf(buf, sizeof buf, "%s %s", stamp, line);
  next_.write(level, buf);
}

}  // namespace n2
