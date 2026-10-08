// SettingsStore.h — two-copy (A/B) storage of NvmSettings in non-volatile memory (Requirements NVM-1, NFR-9).
//
// Address rules (the usual guidelines for flash-backed EEPROM):
//   - Copy A at the start of block 0, copy B at the start of block 1: each in its own erase block, so a power failure
//     during a write destroys at most the copy being written, never the other.
//   - Everything after the 16-byte record in each block is left unused for later (counters, fault history, ...).
//   - Blocks 2.. are not touched, so another program's data there is safe.
//   - save() writes the OLDER copy (or A if none is valid) with sequence+1 and reads it back to verify. A power loss mid-write
//     leaves a damaged older copy and an intact newer one: load() simply picks the newer valid one.
//   - save() does nothing when the stored settings are already identical (wear: each save is one block erase, rated 100,000).
//   - Never call save() from the control loop: the erase takes milliseconds with the CPU waiting. Console or BIST only.
#pragma once

#include "../hal/Nvm.h"
#include "NvmRecord.h"

namespace n2 {

struct StoreReport {
  RecordStatus a = RecordStatus::kBlank;
  RecordStatus b = RecordStatus::kBlank;
  uint32_t seqA = 0, seqB = 0;
  bool valid = false;      // some copy is valid
  char which = '-';        // 'A' or 'B': the copy in use
  uint32_t sequence = 0;   // saves so far, of the copy in use
  NvmSettings settings;    // meaningful only when valid
};

class SettingsStore {
 public:
  explicit SettingsStore(Nvm& nvm) : nvm_(nvm) {}
  StoreReport load();                 // reads both copies; never writes
  bool save(const NvmSettings& s);    // true if written and verified OR already identical
  bool lastSaveWrote() const { return wrote_; }
  // True if the device is big enough for the two blocks.
  bool fits() const { return nvm_.size() >= 2 * nvm_.blockSize(); }

 private:
  Nvm& nvm_;
  bool wrote_ = false;
};

}  // namespace n2
