// NvmEeprom.h — the UNO R4 data flash through the core's EEPROM library, as an Nvm (device builds only).
// ONE EEPROM.put() per record = one block erase. (Byte-by-byte writes rewrite the 1 KB block for every changed byte:
// 16 erases and 0.25-0.7 s per save, measured on the bench 2026-10-08.)
#pragma once
#ifdef ARDUINO
#include <EEPROM.h>

#include "../core/NvmRecord.h"
#include "../hal/Nvm.h"

namespace n2 {

class EepromNvm : public Nvm {
 public:
  size_t size() const override { return static_cast<size_t>(EEPROM.length()); }
  size_t blockSize() const override { return 1024; }
  void read(size_t addr, uint8_t* d, size_t n) override { for (size_t i = 0; i < n; i++) d[i] = EEPROM.read(static_cast<int>(addr + i)); }
  bool write(size_t addr, const uint8_t* d, size_t n) override {
    if (n != kNvmRecordBytes || addr / 1024 != (addr + n - 1) / 1024) return false;
    uint8_t rec[kNvmRecordBytes];
    for (size_t i = 0; i < n; i++) rec[i] = d[i];
    EEPROM.put(static_cast<int>(addr), rec);
    return true;
  }
};

}  // namespace n2
#endif
