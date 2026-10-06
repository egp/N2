#include "Faults.h"

namespace n2 {

namespace {
// Order must match enum FaultId. Codes are HEX (shown as Fxx): the high digit is the group: 0x sensors, 1x devices, 2x safety,
// 3x resets, 4x console. One byte allows 256 codes; the unit needs about a dozen.
const FaultInfo kTable[kFaultCount] = {
    {0x01, "AIR SENSOR RANGE", Severity::kInhibit, false, "TOWERS HELD OFF"},
    {0x02, "N2L SENSOR RANGE", Severity::kInhibit, false, "SSR HELD OFF"},
    {0x03, "N2H SENSOR RANGE", Severity::kInhibit, false, "TOWERS+SSR OFF"},
    {0x04, "N2L ABOVE N2H", Severity::kInhibit, false, "TOWERS+SSR OFF"},
    {0x10, "LCD NO ACK", Severity::kInfo, false, "LOG ONLY"},
    {0x11, "LED NO ACK", Severity::kInfo, false, "LOG ONLY"},
    {0x12, "O2 SENSOR FAILED", Severity::kInhibit, false, "ALL OUTPUTS OFF"},
    {0x13, "RTC UNAVAILABLE", Severity::kInfo, false, "LOG ONLY"},
    {0x20, "INVARIANT BROKEN", Severity::kInhibit, true, "FORCED SAFE STATE"},
    {0x30, "WATCHDOG RESET", Severity::kWarn, false, "RESET WAS LOGGED"},
    {0x31, "BROWN-OUT RESET", Severity::kWarn, false, "RESET WAS LOGGED"},
    {0x40, "CONSOLE DROPPED", Severity::kInfo, false, "LOG ONLY"},
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
      lastCode_ = info.code;
      logf(log_, LogLevel::kWarn, "%lu FAULT F%02X raised: %s", static_cast<unsigned long>(now), info.code, info.text);
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
    logf(log_, LogLevel::kInfo, "%lu FAULT F%02X cleared", static_cast<unsigned long>(now), info.code);
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
