#include "Faults.h"

namespace n2 {

namespace {
// Order must match enum FaultId. Text <= 20 characters.
const FaultInfo kTable[kFaultCount] = {
    {1, "AIR SENSOR RANGE", Severity::kInhibit, false, "TOWERS HELD OFF"},
    {2, "N2L SENSOR RANGE", Severity::kInhibit, false, "SSR HELD OFF"},
    {3, "N2H SENSOR RANGE", Severity::kInhibit, false, "TOWERS+SSR OFF"},
    {4, "N2L ABOVE N2H", Severity::kInhibit, false, "TOWERS+SSR OFF"},
    {10, "LCD NO ACK", Severity::kInfo, false, "LOG ONLY"},
    {11, "LED NO ACK", Severity::kInfo, false, "LOG ONLY"},
    {12, "O2 SENSOR FAILED", Severity::kInhibit, false, "ALL OUTPUTS OFF"},
    {13, "RTC UNAVAILABLE", Severity::kInfo, false, "LOG ONLY"},
    {20, "INVARIANT BROKEN", Severity::kInhibit, true, "FORCED SAFE STATE"},
    {30, "WATCHDOG RESET", Severity::kWarn, false, "RESET WAS LOGGED"},
    {31, "BROWN-OUT RESET", Severity::kWarn, false, "RESET WAS LOGGED"},
    {40, "CONSOLE DROPPED", Severity::kInfo, false, "LOG ONLY"},
};
}  // namespace

const FaultInfo& faultInfo(FaultId id) { return kTable[static_cast<uint8_t>(id)]; }

void FaultSet::report(FaultId id, bool condition, uint32_t now, uint32_t holdMs) {
  Slot& s = slot_[static_cast<uint8_t>(id)];
  const FaultInfo& info = faultInfo(id);
  if (condition) {
    s.clearing = false;
    if (!s.active) {
      s.active = true;
      logf(log_, LogLevel::kWarn, "%lu FAULT F%02u raised: %s", static_cast<unsigned long>(now), info.code, info.text);
    }
    return;
  }
  if (!s.active || info.latching) return;
  if (!s.clearing) {
    s.clearing = true;
    s.clearSince = now;
  }
  if (static_cast<uint32_t>(now - s.clearSince) >= holdMs) {
    s.active = false;
    s.clearing = false;
    logf(log_, LogLevel::kInfo, "%lu FAULT F%02u cleared", static_cast<unsigned long>(now), info.code);
  }
}

uint8_t FaultSet::activeCount(Severity atLeast) const {
  uint8_t n = 0;
  for (uint8_t i = 0; i < kFaultCount; ++i)
    if (slot_[i].active && kTable[i].severity >= atLeast) ++n;
  return n;
}

bool FaultSet::nth(uint8_t n, Severity atLeast, FaultId& out) const {
  for (uint8_t i = 0; i < kFaultCount; ++i) {
    if (slot_[i].active && kTable[i].severity >= atLeast) {
      if (n == 0) {
        out = static_cast<FaultId>(i);
        return true;
      }
      --n;
    }
  }
  return false;
}

}  // namespace n2
