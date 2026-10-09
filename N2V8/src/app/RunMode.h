// RunMode.h — DIAG / BENCH / FIELD as a run-time setting (owner 2026-10-08), and its breadcrumb in RAM.
//
//   DIAG   controllers OFF: nothing is ever driven by the control logic (only a confirmed BIST step moves an output)
//   BENCH  controllers ON, the O2 sensor is not mandatory (the home bench has none)
//   FIELD  production: controllers ON, the O2 sensor is mandatory (INV-9), quiet log, no version on the LCD
//
// The compiled build sets the mode a power-up starts in (DIAG for anything uploaded for testing: the safe default). `mode field confirm` and
// `mode bench confirm` change it at run time, only with TBS OFF; `mode diag` is immediate. The mode is kept in a checksummed RAM record
// that survives a reset-button or watchdog reset (NOT a power loss or brown-out, and NOT in NVM): after those resets the unit keeps its mode.
#pragma once

#include <stdint.h>

namespace n2 {

enum class RunMode : uint8_t { kDiag, kBench, kField };

inline const char* runModeName(RunMode m) { return m == RunMode::kField ? "FIELD" : (m == RunMode::kBench ? "BENCH" : "DIAG"); }

struct ModeRecord {
  uint32_t magic;
  uint32_t mode;
  uint32_t check;
};
constexpr uint32_t kModeRecordMagic = 0x4E324D44u;   // "N2MD"
inline uint32_t modeRecordChecksum(const ModeRecord& r) { return (r.magic ^ 0x5A3CC3A5u) * 31u + r.mode; }
inline bool modeRecordValid(const ModeRecord& r) { return r.magic == kModeRecordMagic && r.mode <= 2 && r.check == modeRecordChecksum(r); }

}  // namespace n2
