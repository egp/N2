// Sensors.h — read and judge the three pressure sensors (INP-1..INP-7).
#pragma once

#include <stdint.h>

#include "../BoardPins.h"
#include "../hal/Hal.h"
#include "../ControlConfig.h"
#include "Faults.h"
#include "Scaling.h"
#include "Snapshots.h"

namespace n2 {

// Debounces one sensor's out-of-window condition: the fault is asserted after
// `samplesNeeded` consecutive bad samples and dropped on the first good one.
// (FaultSet adds the clear hold.)
class SensorChannel {
 public:
  bool update(uint16_t raw, uint8_t adcBits, uint8_t samplesNeeded, uint32_t now = 0, uint32_t minMs = 0);
  bool asserted() const { return asserted_; }

 private:
  uint32_t badSince_ = 0;
  uint8_t bad_ = 0;
  bool asserted_ = false;
};

class SensorMonitor {
 public:
  SensorMonitor(const BoardDef& board, const ControlConfig& cfg, uint8_t adcBits)
      : board_(board), cfg_(cfg), adcBits_(adcBits) {}

  // Read the three sensors, update the sensor faults, and fill the pressure fields of `in`.
  void sample(Hal& hal, uint32_t now, FaultSet& faults, Inputs& in);

 private:
  const BoardDef& board_;
  const ControlConfig& cfg_;
  uint8_t adcBits_;
  SensorChannel air_, n2Low_, n2High_;
  bool orderBad_ = false;
  uint32_t orderBadSince_ = 0;
};

}  // namespace n2
