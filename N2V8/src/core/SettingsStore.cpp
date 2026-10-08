// SettingsStore.cpp — see SettingsStore.h.
#include "SettingsStore.h"

namespace n2 {

StoreReport SettingsStore::load() {
  StoreReport r;
  if (!fits()) return r;
  uint8_t buf[kNvmRecordBytes];
  NvmSettings sa, sb;
  nvm_.read(0, buf, sizeof buf);
  r.a = decodeRecord(buf, r.seqA, sa);
  nvm_.read(nvm_.blockSize(), buf, sizeof buf);
  r.b = decodeRecord(buf, r.seqB, sb);
  const bool okA = r.a == RecordStatus::kOk, okB = r.b == RecordStatus::kOk;
  if (okA && okB) {  // the newer wins; the difference is taken as a signed number so the count may wrap
    const bool aNewer = static_cast<int32_t>(r.seqA - r.seqB) >= 0;
    r.which = aNewer ? 'A' : 'B';
  } else if (okA) {
    r.which = 'A';
  } else if (okB) {
    r.which = 'B';
  }
  if (r.which == 'A') { r.valid = true; r.sequence = r.seqA; r.settings = sa; }
  if (r.which == 'B') { r.valid = true; r.sequence = r.seqB; r.settings = sb; }
  return r;
}

bool SettingsStore::save(const NvmSettings& s) {
  wrote_ = false;
  if (!fits()) return false;
  const StoreReport cur = load();
  if (cur.valid && cur.settings.tbsDebounceMs == s.tbsDebounceMs && cur.settings.tobDebounceMs == s.tobDebounceMs &&
      cur.settings.board == s.board) {
    return true;  // already stored: no erase
  }
  // Write the copy that is NOT in use (A if neither is valid).
  const bool toB = cur.valid && cur.which == 'A';
  const size_t addr = toB ? nvm_.blockSize() : 0;
  uint8_t rec[kNvmRecordBytes];
  encodeRecord(cur.valid ? cur.sequence + 1 : 1, s, rec);
  if (!nvm_.write(addr, rec, sizeof rec)) return false;
  wrote_ = true;
  uint8_t back[kNvmRecordBytes];
  nvm_.read(addr, back, sizeof back);
  for (size_t i = 0; i < sizeof rec; i++) if (back[i] != rec[i]) return false;
  return true;
}

}  // namespace n2
