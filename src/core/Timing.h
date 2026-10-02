// Timing.h — rollover-safe time comparison (Requirements GOAL-6).
#pragma once

#include <stdint.h>

namespace n2 {

// True once `now` has reached `deadline`, correct across the 32-bit millis()
// rollover (about every 49.7 days) as long as the two are < 2^31 ms apart.
constexpr bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

}  // namespace n2
