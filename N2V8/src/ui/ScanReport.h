// ScanReport.h — turns an I2C scan (a bitmap of addresses that answered) into lines a person can read: which DEVICE answered at which
// addresses, what is missing, and anything that answered and should not have. Every response is accounted for (owner 2026-10-08).
#pragma once

#include <stdint.h>

#include "../BoardPins.h"

namespace n2 {

struct ScanReport {
  static constexpr uint8_t kMaxLines = 8;
  char line[kMaxLines][100];
  uint8_t count = 0;
  uint8_t responses = 0;     // addresses that answered
  uint8_t missing = 0;       // expected devices that did not answer fully
  uint8_t unexpected = 0;    // addresses nobody expected
};

// found: bitmap, bit (a % 8) of byte (a / 8) set when address a answered (0x08..0x77).
void buildScanReport(const BoardDef& board, const uint8_t found[16], ScanReport& out);

}  // namespace n2
