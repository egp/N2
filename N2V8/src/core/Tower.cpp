#include "Tower.h"

namespace n2 {

const char* Tower::name(State s) {
  switch (s) {
    case State::kDisabled:  return "OF";
    case State::kLeft:      return "L";
    case State::kLeftBoth:  return "LB";
    case State::kRight:     return "R";
    case State::kRightBoth: return "RB";
  }
  return "??";
}

void Tower::transition(State to, uint32_t now, uint32_t delayMs) {
  if (delayMs > 0) deadline_.arm(now, delayMs);
  else deadline_.clear();
  log_.log(ControllerId::kTower, now, name(state_), name(to), deadline_.armed(), deadline_.at());
  if (state_ == State::kDisabled && to == State::kLeft) { graceUntil_ = now + cfg_.airGraceFromOffMs; graceArmed_ = true; }
  else if ((state_ == State::kLeft && to == State::kLeftBoth) || (state_ == State::kRight && to == State::kRightBoth)) { graceUntil_ = now + cfg_.airGraceToBothMs; graceArmed_ = true; }
  else if (to == State::kDisabled) graceArmed_ = false;
  state_ = to;
}

void Tower::disable(uint32_t now) {
  enabled_ = false;
  if (state_ != State::kDisabled) transition(State::kDisabled, now, 0);
}

// INV-2, INV-3, INV-8, INV-10 as seen by this controller.
bool Tower::mustStop(const Inputs& in) const {
  const bool lowAir = in.airX10 < cfg_.airLowOff && !airGraceActive(in.ms);   // not during the sag right after a valve opens
  const bool airBad = !in.airOk || lowAir;
  const bool n2HighBad = !in.n2HighOk || in.n2HighX10 > cfg_.n2HighOff;
  const bool o2Holds = cfg_.o2Mandatory && !(in.o2CommOk && in.o2Warm);
  return airBad || n2HighBad || in.sensorOrderFault || o2Holds;
}

bool Tower::mayStart(const Inputs& in) const {
  const bool air = in.airOk && in.airX10 > cfg_.airLowOn;
  const bool n2High = in.n2HighOk && in.n2HighX10 < cfg_.n2HighOn;
  const bool o2 = !cfg_.o2Mandatory || (in.o2CommOk && in.o2Warm);
  return enabled_ && in.tbs && air && n2High && !in.sensorOrderFault && o2;
}

void Tower::update(const Inputs& in) {
  if (!enabled_) return;
  const uint32_t now = in.ms;
  if (state_ != State::kDisabled && mustStop(in)) {
    transition(State::kDisabled, now, 0);
    return;
  }
  switch (state_) {
    case State::kDisabled:
      if (mayStart(in)) transition(State::kLeft, now, cfg_.towerFillMs);
      break;
    case State::kLeft:
      if (deadline_.reached(now)) transition(State::kLeftBoth, now, cfg_.towerOverlapMs);
      break;
    case State::kLeftBoth:
      if (deadline_.reached(now)) transition(State::kRight, now, cfg_.towerFillMs);
      break;
    case State::kRight:
      if (deadline_.reached(now)) transition(State::kRightBoth, now, cfg_.towerOverlapMs);
      break;
    case State::kRightBoth:
      if (deadline_.reached(now)) transition(State::kLeft, now, cfg_.towerFillMs);
      break;
    default:  // unreachable; recover safely (ARC-6)
      transition(State::kDisabled, now, 0);
      break;
  }
}

}  // namespace n2
