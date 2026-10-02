// Snapshots.h — plain data passed between the layers.
#pragma once

#include <stdint.h>

namespace n2 {

// Everything the controllers need to know about the world, read once per loop pass (INP-1).
struct Inputs {
  uint32_t ms = 0;
  bool tbs = false;
  bool tob = false;
  uint16_t airX10 = 0;
  uint16_t n2LowX100 = 0;
  uint16_t n2HighX10 = 0;
  uint16_t rawAir = 0;     // raw ADC counts, for diagnostics
  uint16_t rawN2Low = 0;
  uint16_t rawN2High = 0;
  bool airOk = true;       // sensor not faulty (debounced, with clear hold)
  bool n2LowOk = true;
  bool n2HighOk = true;
  bool sensorOrderFault = false;  // F04: N2-low reads above N2-high
  bool o2CommOk = false;   // filled from the O2 controller
  bool o2Warm = false;
};

// What the controllers WANT the four outputs to be. The OutputDriver decides what they ARE.
struct OutputRequest {
  bool left = false;
  bool right = false;
  bool flush = false;
  bool ssr = false;
};

}  // namespace n2
