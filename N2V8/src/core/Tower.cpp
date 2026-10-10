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
  if (to == State::kLeftBoth || to == State::kRightBoth) {   // a new overlap: start watching the air
    ovStart_ = now;
    ovNext_ = now + 50;
    ovN_ = 0;
    ovMin_ = 0xFFFF;
    ovRun_ = 0;
  }
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

// TWR-OV: end the overlap when the air has passed its minimum (owner 2026-10-10): the second valve's opening drops the supply, which then recovers.
// Not before overlapMinMs. Air is sampled every 50 ms; the median of three samples is tracked so a single rippled reading cannot be taken as the minimum.
// The minimum is "past" when two consecutive medians are at least overlapRiseX10 above the lowest median. If that never happens the deadline
// (towerOverlapMs) ends the overlap as before, so this can only shorten the overlap, never lengthen it.
bool Tower::overlapPast(const Inputs& in) {
  const uint32_t now = in.ms;
  if (static_cast<int32_t>(now - ovNext_) >= 0) {
    ovNext_ = now + 50;
    if (ovN_ < 3) ovRing_[ovN_] = in.airX10;
    else {
      ovRing_[0] = ovRing_[1];
      ovRing_[1] = ovRing_[2];
      ovRing_[2] = in.airX10;
    }
    if (ovN_ < 255) ++ovN_;
    if (ovN_ >= 3) {
      uint16_t a = ovRing_[0], b = ovRing_[1], c = ovRing_[2];
      const uint16_t med = a < b ? (b < c ? b : (a < c ? c : a)) : (a < c ? a : (b < c ? c : b));
      if (med < ovMin_) {
        ovMin_ = med;
        ovRun_ = 0;
      } else if (med >= static_cast<uint16_t>(ovMin_ + cfg_.overlapRiseX10)) {
        if (ovRun_ < 255) ++ovRun_;
      } else {
        ovRun_ = 0;
      }
    }
  }
  return ovRun_ >= 2 && static_cast<uint32_t>(now - ovStart_) >= cfg_.overlapMinMs;
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
      if (deadline_.reached(now) || overlapPast(in)) transition(State::kRight, now, cfg_.towerFillMs);
      break;
    case State::kRight:
      if (deadline_.reached(now)) transition(State::kRightBoth, now, cfg_.towerOverlapMs);
      break;
    case State::kRightBoth:
      if (deadline_.reached(now) || overlapPast(in)) transition(State::kLeft, now, cfg_.towerFillMs);
      break;
    default:  // unreachable; recover safely (ARC-6)
      transition(State::kDisabled, now, 0);
      break;
  }
}

}  // namespace n2
