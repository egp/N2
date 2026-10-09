// Tower.h — tower valve controller (Requirements §7.1). Timed state machine (ARC-6).
//
// The controller only REQUESTS valve states; the OutputDriver decides what the pins do.
#pragma once

#include <stdint.h>

#include "../ControlConfig.h"
#include "Log.h"
#include "Snapshots.h"
#include "TimedState.h"

namespace n2 {

class Tower {
 public:
  enum class State : uint8_t { kDisabled, kLeft, kLeftBoth, kRight, kRightBoth };

  Tower(const ControlConfig& cfg, TransitionLogger& log) : cfg_(cfg), log_(log) {}

  void enable() { enabled_ = true; }       // stays DISABLED; update() restarts cycling when conditions allow
  void disable(uint32_t now);              // close both valves, enter DISABLED
  void update(const Inputs& in);

  State state() const { return state_; }
  // True for a short time after a tower valve OPENS (OFF -> LEFT: airGraceFromOffMs; LEFT -> BOTH and RIGHT -> BOTH: airGraceToBothMs).
  bool airGraceActive(uint32_t now) const { return graceArmed_ && static_cast<int32_t>(now - graceUntil_) < 0; }
  bool leftOpen() const { return state_ == State::kLeft || state_ == State::kLeftBoth || state_ == State::kRightBoth; }
  bool rightOpen() const { return state_ == State::kLeftBoth || state_ == State::kRight || state_ == State::kRightBoth; }
  static const char* name(State s);

 private:
  bool mustStop(const Inputs& in) const;
  bool mayStart(const Inputs& in) const;
  void transition(State to, uint32_t now, uint32_t delayMs);

  uint32_t graceUntil_ = 0;
  bool graceArmed_ = false;
  const ControlConfig& cfg_;
  TransitionLogger& log_;
  State state_ = State::kDisabled;
  Deadline deadline_;
  bool enabled_ = false;
};

}  // namespace n2
