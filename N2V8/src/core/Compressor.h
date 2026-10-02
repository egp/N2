// Compressor.h — compressor SSR controller (Requirements §7.2, RST-4).
#pragma once

#include <stdint.h>

#include "../ControlConfig.h"
#include "Log.h"
#include "Snapshots.h"

namespace n2 {

class Compressor {
 public:
  enum class State : uint8_t { kDisabled, kRunning, kStoppedLow, kStoppedHigh };

  Compressor(const ControlConfig& cfg, TransitionLogger& log) : cfg_(cfg), log_(log) {}

  // RST-4: the starting state is chosen from the sensors, never assumed.
  void enable(const Inputs& in);
  void disable(uint32_t now);
  void update(const Inputs& in);

  State state() const { return state_; }
  bool ssrOn() const { return state_ == State::kRunning; }
  static const char* name(State s);

 private:
  bool highTrouble(const Inputs& in) const;  // N2-high above off, or its sensor / the pair is suspect
  bool lowTrouble(const Inputs& in) const;
  void transition(State to, uint32_t now);

  const ControlConfig& cfg_;
  TransitionLogger& log_;
  State state_ = State::kDisabled;
};

}  // namespace n2
