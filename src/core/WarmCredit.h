// WarmCredit.h — belt and suspenders for the O2 warm-up across resets (Requirements O2-6a/6b).
//
// A reset button does not power-cycle the O2 sensor, so a completed warm-up need
// not be repeated. Warm-up credit is carried over a reset ONLY when two independent
// indications agree: (1) the reset was not a power-on reset, and (2) a valid record
// survived in RAM. Otherwise the credit is zero and the full warm-up applies.
// `enabled` is false until bench tests prove both indications (O2-6b).
#pragma once

#include <stdint.h>

namespace n2 {

// What the hardware says about the last reset (from the reset-status register, via the HAL).
struct ResetInfo {
  bool known = false;     // false if the status could not be read -> never trust credit
  bool powerOn = false;
  bool watchdog = false;
  bool brownout = false;
};

// Lives in a RAM section that is not cleared by reset (.noinit on the device).
struct WarmRecord {
  uint32_t magic;
  uint32_t boots;
  uint32_t lastSeenMs;
  uint32_t runMs;  // sensor-powered time accumulated across resets
  uint32_t check;
};

constexpr uint32_t kWarmRecordMagic = 0x4E325743;  // "N2WC"

uint32_t warmRecordChecksum(const WarmRecord& r);
bool warmRecordValid(const WarmRecord& r);

class WarmCredit {
 public:
  WarmCredit(WarmRecord& record, bool enabled) : rec_(record), enabled_(enabled) {}

  // Call once at boot. Returns the credit in ms (0 unless enabled, the reset was
  // not a power-on reset, and the record is valid) and re-initialises the record.
  uint32_t begin(const ResetInfo& reset, uint32_t bootMs);

  // Call about once a second while running: keeps the record current.
  void tick(uint32_t nowMs);

  // The sensor may have lost power (communication failure): nothing carries over.
  void restart(uint32_t nowMs);

  bool recordWasValid() const { return recordWasValid_; }

 private:
  void seal(uint32_t nowMs);

  WarmRecord& rec_;
  bool enabled_;
  bool recordWasValid_ = false;
  uint32_t baseMs_ = 0;   // credit at boot (or since restart)
  uint32_t startMs_ = 0;  // millis() when baseMs_ applied
};

}  // namespace n2
