// System.h — one loop pass of the whole controller (Requirements §3 data flow).
//
//   HAL -> sensors -> faults -> O2 -> enable/disable -> tower, compressor
//       -> invariants -> OutputDriver -> HAL
//
// Depends only on the Hal interface, so the entire system runs on the host.
#pragma once

#include <stdint.h>

#include "../BoardPins.h"
#include "../Config.h"
#include "../ControlConfig.h"
#include "../hal/Hal.h"
#include "Compressor.h"
#include "Faults.h"
#include "Invariants.h"
#include "Log.h"
#include "O2Controller.h"
#include "O2Reader.h"
#include "OutputDriver.h"
#include "Sensors.h"
#include "Snapshots.h"
#include "Tower.h"
#include "WarmCredit.h"
#include "WarmupTracker.h"

namespace n2 {

class System {
 public:
  System(Hal& hal, const BoardDef& board, const ControlConfig& cfg, O2Reader& o2Reader, LogSink& log,
         uint8_t adcBits = kAdcBits);

  // Boot (RST-1..RST-5): outputs safe, controllers DISABLED, warm-up starts (minus trusted credit).
  void begin(const ResetInfo& reset = ResetInfo(), uint32_t warmCreditMs = 0);

  // After POST/BIST: outputs safe again, controllers forget their state, TBS edge detection restarts (RST-3).
  // The O2 warm-up is NOT restarted: the sensor kept its power.
  void resume();

  // DIAG build (Requirements §2): sensors, faults and safety checks run, but no controller does and every output
  // stays OFF (except under BIST, which drives them through the OutputDriver itself).
  void setControllersEnabled(bool enabled) { controllersEnabled_ = enabled; }

  // One pass of loop().
  void step();

  const Inputs& inputs() const { return in_; }
  const Tower& tower() const { return tower_; }
  const Compressor& compressor() const { return compressor_; }
  const O2Controller& o2() const { return o2_; }
  const FaultSet& faults() const { return faults_; }
  FaultSet& faultSet() { return faults_; }  // POST, BIST and the display manager report their own faults here
  const OutputDriver& outputs() const { return driver_; }
  OutputDriver& outputDriver() { return driver_; }  // BIST only: forceDrive()
  const WarmupTracker& warmup() const { return warmup_; }
  const ControlConfig& config() const { return cfg_; }
  uint32_t invariantViolations() const { return violations_; }
  OutputRequest lastRequest() const { return request_; }
  InvariantResult lastInvariants() const { return invariants_; }

 private:
  bool readOn(Signal s);

  Hal& hal_;
  const BoardDef& board_;
  ControlConfig cfg_;
  LogSink& log_;
  TransitionLogger transitions_;
  FaultSet faults_;
  SensorMonitor sensors_;
  WarmupTracker warmup_;
  Tower tower_;
  Compressor compressor_;
  O2Controller o2_;
  OutputDriver driver_;

  Inputs in_;
  OutputRequest request_;
  InvariantResult invariants_;
  bool prevTbs_ = false;
  bool prevEnabled_ = false;
  bool resetWasWatchdog_ = false;
  bool resetWasBrownout_ = false;
  bool normalCycleSeen_ = false;
  bool controllersEnabled_ = true;
  uint32_t violations_ = 0;
};

}  // namespace n2
