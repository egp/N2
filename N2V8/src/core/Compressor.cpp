#include "Compressor.h"

namespace n2 {

const char* Compressor::name(State s) {
  switch (s) {
    case State::kDisabled:    return "OF";
    case State::kRunning:     return "ON";
    case State::kStoppedLow:  return "LO";
    case State::kStoppedHigh: return "HI";
  }
  return "??";
}

void Compressor::transition(State to, uint32_t now) {
  log_.log(ControllerId::kCompressor, now, name(state_), name(to), false, 0);
  state_ = to;
}

// INV-3 / INV-8: N2-high over its limit, its sensor faulty, or the N2 pair inconsistent.
bool Compressor::highTrouble(const Inputs& in) const {
  return !in.n2HighOk || in.sensorOrderFault || in.n2HighX10 > cfg_.n2HighOff;
}
// INV-4: N2-low under its limit or its sensor faulty.
bool Compressor::lowTrouble(const Inputs& in) const { return !in.n2LowOk || in.n2LowX100 < cfg_.n2LowOff; }

void Compressor::enable(const Inputs& in) {
  State to;
  if (highTrouble(in) || in.n2HighX10 >= cfg_.n2HighOn) to = State::kStoppedHigh;
  else if (lowTrouble(in) || in.n2LowX100 <= cfg_.n2LowOn) to = State::kStoppedLow;
  else to = State::kRunning;
  if (to != state_) transition(to, in.ms);
}

void Compressor::disable(uint32_t now) {
  if (state_ != State::kDisabled) transition(State::kDisabled, now);
}

void Compressor::update(const Inputs& in) {
  const uint32_t now = in.ms;
  switch (state_) {
    case State::kDisabled:
      break;
    case State::kRunning:
      if (lowTrouble(in)) transition(State::kStoppedLow, now);
      else if (highTrouble(in)) transition(State::kStoppedHigh, now);
      break;
    case State::kStoppedLow:  // restart needs low recovered AND high not in trouble
      if (in.n2LowOk && in.n2LowX100 > cfg_.n2LowOn && !highTrouble(in)) transition(State::kRunning, now);
      break;
    case State::kStoppedHigh:  // restart needs high recovered AND low not in trouble
      if (in.n2HighOk && !in.sensorOrderFault && in.n2HighX10 < cfg_.n2HighOn && !lowTrouble(in))
        transition(State::kRunning, now);
      break;
    default:  // unreachable; recover safely (ARC-6)
      transition(State::kDisabled, now);
      break;
  }
}

}  // namespace n2
