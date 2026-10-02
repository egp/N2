// TimedState.h — the non-blocking deadline used by every controller (ARC-6, GOAL-6).
//
// A Deadline is "do the next thing at time T". It never waits; callers poll
// reached(now) each loop pass. Comparison is by unsigned subtraction, so it
// stays correct across the millis() rollover.
#pragma once

#include <stdint.h>

#include "Timing.h"

namespace n2 {

class Deadline {
 public:
  void arm(uint32_t now, uint32_t delayMs) { at_ = now + delayMs; armed_ = true; }
  void armAt(uint32_t timeMs) { at_ = timeMs; armed_ = true; }
  void clear() { armed_ = false; }

  bool armed() const { return armed_; }
  uint32_t at() const { return at_; }
  // A cleared deadline is never reached.
  bool reached(uint32_t now) const { return armed_ && deadlineReached(now, at_); }

 private:
  uint32_t at_ = 0;
  bool armed_ = false;
};

}  // namespace n2
