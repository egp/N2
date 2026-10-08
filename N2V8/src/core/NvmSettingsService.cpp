// NvmSettingsService.cpp — see NvmSettingsService.h.
#include "NvmSettingsService.h"

namespace n2 {

DebounceChoice chooseDebounce(const StoreReport& r, BoardId thisBoard, uint8_t defaultMs) {
  DebounceChoice c;
  c.tbsMs = c.tobMs = defaultMs;
  if (!r.valid) { c.why = "nothing valid stored (never set, or damaged)"; return c; }
  if (r.settings.board != thisBoard) { c.why = "stored values are for the other board"; return c; }
  const NvmSettings& s = r.settings;
  if (s.tbsDebounceMs < kMinDebounceMs || s.tbsDebounceMs > kMaxDebounceMs || s.tobDebounceMs < kMinDebounceMs ||
      s.tobDebounceMs > kMaxDebounceMs) {
    c.why = "stored values out of range";
    return c;
  }
  c.tbsMs = s.tbsDebounceMs;
  c.tobMs = s.tobDebounceMs;
  c.source = DebounceSource::kStored;
  c.why = "stored in NVM";
  return c;
}

void NvmSettingsService::load() {
  choice_ = DebounceChoice();
  choice_.tbsMs = choice_.tobMs = defaultMs_;
  report_ = StoreReport();
  if (!nvm_) return;
  report_ = SettingsStore(*nvm_).load();
  choice_ = chooseDebounce(report_, board_, defaultMs_);
}

bool NvmSettingsService::saveDebounce(uint8_t tbsMs, uint8_t tobMs) {
  if (!nvm_ || tbsMs < kMinDebounceMs || tbsMs > kMaxDebounceMs || tobMs < kMinDebounceMs || tobMs > kMaxDebounceMs) return false;
  NvmSettings s;
  s.tbsDebounceMs = tbsMs;
  s.tobDebounceMs = tobMs;
  s.board = board_;
  s.sketchVersion = version_;
  const bool ok = SettingsStore(*nvm_).save(s);
  load();
  return ok;
}

}  // namespace n2
