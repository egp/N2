// ResetInfo.h — what the hardware says about the last reset.
#pragma once

namespace n2 {

struct ResetInfo {
  bool known = false;     // false if the status could not be read -> never trust warm-up credit
  bool powerOn = false;
  bool watchdog = false;
  bool brownout = false;
};

}  // namespace n2
