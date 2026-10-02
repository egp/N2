// WarmupTracker.h — how much O2 sensor warm-up is left (Requirements O2-6).
//
// The sensor needs `warmMs` of power-on time. Time already spent is "credit"
// (carried over a reset only when WarmCredit says it can be trusted). Anything
// that may have cut the sensor's power restarts the warm-up.
#pragma once

#include <stdint.h>

namespace n2 {

class WarmupTracker {
 public:
  explicit WarmupTracker(uint32_t warmMs) : warmMs_(warmMs) {}

  // Boot: warm-up counts from `now`, minus any trusted credit.
  void begin(uint32_t now, uint32_t creditMs) {
    startMs_ = now;
    creditMs_ = creditMs > warmMs_ ? warmMs_ : creditMs;
  }
  // The sensor may have lost power (communication failure): start over.
  void restart(uint32_t now) { begin(now, 0); }

  uint32_t remainingMs(uint32_t now) const {
    const uint32_t have = static_cast<uint32_t>(now - startMs_) + creditMs_;
    return have >= warmMs_ ? 0 : warmMs_ - have;
  }
  bool warm(uint32_t now) const { return remainingMs(now) == 0; }

 private:
  uint32_t warmMs_;
  uint32_t startMs_ = 0;
  uint32_t creditMs_ = 0;
};

}  // namespace n2
