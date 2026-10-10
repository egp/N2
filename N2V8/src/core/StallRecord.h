// StallRecord.h — the "where was it" breadcrumb for a stalled or watchdog-reset loop (owner 2026-10-10, stall hardening A1).
//
// A few words of RAM that survive a reset-button or watchdog reset (not a power cycle). The main loop marks the section it is in; the real Hal
// records the last and the slowest I2C transaction. After a watchdog reset the boot log names the section and the bus device, so a stall costs
// one reset and a named cause. Plain words, no checksum: a wrong magic number means "no record".
#pragma once

#include <stdint.h>

namespace n2 {

enum class StallPhase : uint8_t { kNone, kBoot, kConsole, kBistStep, kPostStep, kRunSystem, kShowNormal, kRtcCheck, kDisplayService, kIdle, kCount };

inline const char* stallPhaseName(uint32_t p) {
  static const char* const names[] = {"none", "boot", "console", "BIST step", "POST step", "system step", "show normal screen", "RTC check", "display service", "idle (end of pass)"};
  return p < static_cast<uint32_t>(StallPhase::kCount) ? names[p] : "unknown";
}

constexpr uint32_t kStallMagic = 0x4E325354u;   // "N2ST"

struct StallRecord {
  uint32_t magic;
  uint32_t phase;        // StallPhase of the last mark
  uint32_t aux;          // small detail of that mark (the run mode, the BIST step ...)
  uint32_t atMs;         // millis() at the last mark
  uint32_t i2cLastAddr;  // the last I2C transaction: address and how long it took
  uint32_t i2cLastUs;
  uint32_t i2cWorstAddr; // the slowest I2C transaction since this boot
  uint32_t i2cWorstUs;
  uint32_t loopMaxUs;    // the longest pass of the main loop since this boot
};

inline bool stallValid(const StallRecord& r) { return r.magic == kStallMagic && r.phase < static_cast<uint32_t>(StallPhase::kCount); }

}  // namespace n2
