// NvmSettingsService.h — the settings in non-volatile memory as the firmware uses them (Requirements NVM-1, INP-6, NFR-9).
//
// At boot: load() reads both copies and decides the debounce times (chooseDebounce). The compiled default is used when nothing valid is stored,
// when the record was written for the other board, or when a stored time is outside 2..100 ms. A stored record written by a DIFFERENT SKETCH
// VERSION is still used for the debounce times (they are measured, per-board values) but the difference is reported, so it is never silent.
// Saving (console `debounce set`) costs one block erase and about 50 ms with the CPU waiting: only with TBS off.
#pragma once

#include "../hal/Nvm.h"
#include "Debounce.h"
#include "SettingsStore.h"

namespace n2 {

enum class DebounceSource : uint8_t { kCompiledDefault, kStored };

struct DebounceChoice {
  uint8_t tbsMs = kBringupDebounceMs;
  uint8_t tobMs = kBringupDebounceMs;
  DebounceSource source = DebounceSource::kCompiledDefault;
  const char* why = "no memory in this build";   // a short reason, for the log and the console
};

DebounceChoice chooseDebounce(const StoreReport& report, BoardId thisBoard, uint8_t defaultMs);

class NvmSettingsService {
 public:
  // nvm may be nullptr (host builds that do not test it, or a board without data flash): the default is used.
  NvmSettingsService(Nvm* nvm, BoardId board, uint16_t sketchVersion, uint8_t defaultMs = kBringupDebounceMs)
      : nvm_(nvm), board_(board), version_(sketchVersion), defaultMs_(defaultMs) { choice_.tbsMs = choice_.tobMs = defaultMs; }

  void load();                                       // read only; never writes
  bool saveDebounce(uint8_t tbsMs, uint8_t tobMs);   // validates 2..100, writes (one erase), reloads. Blocks ~50 ms.
  bool available() const { return nvm_ != nullptr; }
  const DebounceChoice& choice() const { return choice_; }
  const StoreReport& report() const { return report_; }
  uint16_t sketchVersion() const { return version_; }
  BoardId board() const { return board_; }
  size_t nvmSize() const { return nvm_ ? nvm_->size() : 0; }
  size_t nvmBlock() const { return nvm_ ? nvm_->blockSize() : 0; }

 private:
  Nvm* nvm_;
  BoardId board_;
  uint16_t version_;
  uint8_t defaultMs_;
  StoreReport report_;
  DebounceChoice choice_;
};

}  // namespace n2
