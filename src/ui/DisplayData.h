// DisplayData.h — everything the displays need, copied from the system once per pass (DSP-1).
//
// The display code depends on this plain struct only, so screens are pure functions
// of it and can be tested as text on the host.
#pragma once

#include <stdint.h>

#include "../core/Faults.h"

namespace n2 {

class System;

struct DisplayData {
  uint16_t airX10 = 0;
  uint16_t n2LowX100 = 0;
  uint16_t n2HighX10 = 0;
  uint16_t rawAir = 0;
  uint16_t rawN2Low = 0;
  uint16_t rawN2High = 0;
  uint8_t adcBits = 10;

  bool n2Valid = false;
  bool n2Stale = false;
  uint16_t n2PercentX100 = 0;
  bool warming = false;             // O2 warm-up in progress: show the countdown instead of N2%
  uint32_t warmRemainingMs = 0;

  const char* tower = "OF";         // short state names (<= 2 characters)
  const char* compressor = "OF";
  const char* o2 = "OF";

  bool left = false;                // the four ACTUAL driven outputs
  bool right = false;
  bool flush = false;
  bool ssr = false;

  bool tbs = false;                 // LED only; not shown on the LCD

  uint8_t faultCount = 0;           // active faults of severity >= WARN, in table order
  FaultId faults[kFaultCount] = {};
};


// Copy the current state of the system into a DisplayData.
DisplayData makeDisplayData(const System& sys, uint8_t adcBits);

}  // namespace n2
