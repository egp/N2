// WallClock.h — wall-clock time for LOG TIME STAMPS ONLY (Requirements RTC-7).
//
// The RTC is read once to anchor this clock, then again every few minutes to re-anchor it. Between those reads the time is
// the anchor plus the elapsed millis(), so stamping a log line costs no I2C traffic. NOTHING in the control path or the
// scheduler may use this: all timing stays on millis() deadlines. If the RTC is missing or untrusted the clock is simply
// "not synced" and stamps are left out; the program carries on.
#pragma once

#include <stdint.h>

#include "../hal/Hal.h"
#include "DateTime.h"
#include "Log.h"

namespace n2 {

class WallClock {
 public:
  void sync(const DateTime& t, uint32_t nowMs);  // anchor to this date/time, read at nowMs
  void unsync() { synced_ = false; }
  bool synced() const { return synced_; }
  // "2026-10-06 10:31:02" into out (20 bytes). False (and out = "") if not synced.
  bool stamp(uint32_t nowMs, char* out) const;
  bool now(uint32_t nowMs, DateTime& out) const;

 private:
  bool synced_ = false;
  uint32_t anchorSeconds_ = 0;  // seconds since 2000-01-01 at anchorMs_
  uint32_t anchorMs_ = 0;
};

// Whether the RTC should be written to match a reference time (RTC-8): only when the RTC is not trusted, or it differs from
// the reference by more than toleranceSeconds. Otherwise the RTC is left alone (read-only).
bool rtcNeedsSetting(bool rtcReadable, bool rtcTrusted, const DateTime& rtcTime, const DateTime& reference,
                     uint32_t toleranceSeconds = 2);

// A LogSink that puts the wall-clock time in front of every line (when known) and passes it on.
class StampedLog : public LogSink {
 public:
  StampedLog(LogSink& next, const WallClock& clock, Hal& hal) : next_(next), clock_(clock), hal_(hal) {}
  void write(LogLevel level, const char* line) override;

 private:
  LogSink& next_;
  const WallClock& clock_;
  Hal& hal_;
};

}  // namespace n2
