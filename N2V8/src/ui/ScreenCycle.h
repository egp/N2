// ScreenCycle.h — which screen is on the LCD now (Requirements DSP-5).
//
// With no active fault the normal screen is always shown. With faults, the display
// alternates: normal, fault 1, normal, fault 2, ... each for `cycleMs`, because a
// fault may make the system inoperable while the sensor readings are still useful.
// Non-blocking: a Deadline moves the slot on; no delay().
#pragma once

#include <stdint.h>

#include "../core/TimedState.h"

namespace n2 {

class ScreenCycle {
 public:
  explicit ScreenCycle(uint32_t cycleMs) : cycleMs_(cycleMs) {}

  // Call once per pass with the number of active faults (severity >= WARN).
  void update(uint32_t now, uint8_t faultCount) {
    if (faultCount == 0) {  // back to the plain normal screen
      showFault_ = false;
      faultIndex_ = 0;
      deadline_.clear();
      return;
    }
    if (!deadline_.armed()) {  // a fault just appeared: start with the fault screen
      showFault_ = true;
      faultIndex_ = 0;
      deadline_.arm(now, cycleMs_);
      return;
    }
    if (deadline_.reached(now)) {
      if (showFault_) {
        showFault_ = false;
      } else {
        showFault_ = true;
        faultIndex_ = static_cast<uint8_t>((faultIndex_ + 1u) % faultCount);
      }
      deadline_.arm(now, cycleMs_);
    }
  }

  bool showFault() const { return showFault_; }
  uint8_t faultIndex() const { return faultIndex_; }

 private:
  uint32_t cycleMs_;
  Deadline deadline_;
  bool showFault_ = false;
  uint8_t faultIndex_ = 0;
};

}  // namespace n2
