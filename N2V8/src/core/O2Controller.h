// O2Controller.h — O2 sampling cycle with warm-up (Requirements §7.3, O2-1..O2-7).
#pragma once

#include <stdint.h>

#include "../ControlConfig.h"
#include "Log.h"
#include "O2Reader.h"
#include "Snapshots.h"
#include "TimedState.h"
#include "WarmupTracker.h"

namespace n2 {

class O2Controller {
 public:
  enum class State : uint8_t { kUnknown, kWarming, kFlushing, kSampling, kWaiting, kError, kDisabled };

  O2Controller(const ControlConfig& cfg, O2Reader& reader, WarmupTracker& warmup, TransitionLogger& log)
      : cfg_(cfg), reader_(reader), warmup_(warmup), log_(log) {}

  void enable(uint32_t now);   // TBS on: start looking for the sensor
  void disable(uint32_t now);  // TBS off: close the flush valve, forget the reading
  void update(const Inputs& in);

  State state() const { return state_; }
  bool flushOpen() const { return state_ == State::kFlushing; }
  bool commOk() const { return commOk_; }
  bool warm(uint32_t now) const { return warmup_.warm(now); }
  uint32_t warmRemainingMs(uint32_t now) const { return warmup_.remainingMs(now); }
  // N2 purity, percent x100, clamped to 9999. Valid after the first complete cycle.
  uint16_t n2PercentX100() const { return n2x100_; }
  bool n2Valid() const { return n2Valid_; }
  bool n2Stale() const { return n2Stale_; }
  static const char* name(State s);

 private:
  bool gateOk(const Inputs& in) const;  // O2-7: N2 pressures within operating range
  void transition(State to, uint32_t now, bool armDeadline, uint32_t deadlineAt);
  void beginCycle(uint32_t now);
  void fail(uint32_t now);

  const ControlConfig& cfg_;
  O2Reader& reader_;
  WarmupTracker& warmup_;
  TransitionLogger& log_;
  State state_ = State::kDisabled;
  Deadline deadline_;
  uint32_t enabledAt_ = 0;
  uint32_t cycleStart_ = 0;
  uint32_t sampleSum_ = 0;
  uint8_t sampleNum_ = 0;
  bool commOk_ = false;
  bool needWarmRestart_ = false;  // sensor may have been power-cycled while we could not see it
  uint16_t n2x100_ = 0;
  bool n2Valid_ = false;
  bool n2Stale_ = false;
};

}  // namespace n2
