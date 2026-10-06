#include "ResetCheck.h"

namespace n2 {

const char* ResetCheck::causeText(const ResetInfo& info) {
  if (!info.known) return "unknown";
  if (info.powerOn) return "power-on";
  if (info.watchdog) return "watchdog";
  if (info.brownout) return "brown-out";
  return "reset button/other";
}

CheckResult ResetCheck::post(uint32_t) {
  CheckResult r;
  // A watchdog or brown-out reset is worth a line in the log, but never holds POST (Q25).
  const bool noteworthy = !info_.known || info_.watchdog || info_.brownout;
  r.set(noteworthy ? CheckLevel::kInfo : CheckLevel::kPass, "%s", causeText(info_));
  return r;
}

}  // namespace n2
