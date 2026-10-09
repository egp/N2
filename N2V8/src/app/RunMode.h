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
  uint32_t build;   // identity of the program that wrote it: a record from another build (a new upload) is ignored
  uint32_t check;
};
constexpr uint32_t kModeRecordMagic = 0x4E324D44u;   // "N2MD"
inline uint32_t modeRecordChecksum(const ModeRecord& r) { return ((r.magic ^ 0x5A3CC3A5u) * 31u + r.mode) * 31u + r.build; }
inline bool modeRecordValid(const ModeRecord& r) { return r.magic == kModeRecordMagic && r.mode <= 2 && r.check == modeRecordChecksum(r); }

// FNV-1a over version, build date and build time: changes with every upload.
inline uint32_t buildIdentity(const char* a, const char* b, const char* c) {
  uint32_t h = 2166136261u;
  const char* parts[3] = {a, b, c};
  for (const char* s : parts)
    for (; s != nullptr && *s; ++s) h = (h ^ static_cast<uint8_t>(*s)) * 16777619u;
  return h;
}

}  // namespace n2
