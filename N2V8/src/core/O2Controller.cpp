#include "O2Controller.h"

namespace n2 {

const char* O2Controller::name(State s) {
  switch (s) {
    case State::kUnknown:  return "??";
    case State::kWarming:  return "WM";
    case State::kFlushing: return "F";
    case State::kSampling: return "S";
    case State::kWaiting:  return "W";
    case State::kError:    return "E";
    case State::kDisabled: return "OF";
  }
  return "??";
}

void O2Controller::transition(State to, uint32_t now, bool armDeadline, uint32_t deadlineAt) {
  if (armDeadline) deadline_.armAt(deadlineAt);
  else deadline_.clear();
  log_.log(ControllerId::kO2, now, name(state_), name(to), deadline_.armed(), deadline_.at());
  state_ = to;
}

void O2Controller::enable(uint32_t now) {
  enabledAt_ = now;
  sampleSum_ = 0;
  sampleNum_ = 0;
  transition(State::kUnknown, now, true, now);
}

void O2Controller::disable(uint32_t now) {
  if (state_ != State::kDisabled) transition(State::kDisabled, now, false, 0);
  commOk_ = false;
  n2Valid_ = false;
  n2Stale_ = false;
}

// O2-7: sampling is meaningful while N2-low is above its minimum and N2-high below its maximum.
bool O2Controller::gateOk(const Inputs& in) const {
  return in.n2LowOk && in.n2HighOk && in.n2LowX100 > cfg_.n2LowOff && in.n2HighX10 < cfg_.n2HighOff;
}

void O2Controller::fail(uint32_t now) {
  commOk_ = false;
  n2Valid_ = false;
  n2Stale_ = false;
  needWarmRestart_ = true;
  transition(State::kError, now, true, now + cfg_.o2ErrorRetryMs);
}

void O2Controller::beginCycle(uint32_t now) {
  cycleStart_ = now;
  sampleSum_ = 0;
  sampleNum_ = 0;
  n2Stale_ = false;
  transition(State::kFlushing, now, true, now + cfg_.o2FlushMs);
}

void O2Controller::update(const Inputs& in) {
  const uint32_t now = in.ms;
  switch (state_) {
    case State::kDisabled:
      break;

    case State::kUnknown:
      if (!deadline_.reached(now)) break;
      if (reader_.begin()) {
        commOk_ = true;
        if (needWarmRestart_) {  // it was missing: it is cold now (O2-6)
          warmup_.restart(now);
          needWarmRestart_ = false;
        }
        if (warmup_.warm(now)) beginCycle(now);
        else transition(State::kWarming, now, true, now + cfg_.o2CommRetryMs);
      } else if (static_cast<uint32_t>(now - enabledAt_) >= cfg_.o2CommTimeoutMs) {
        fail(now);
      } else {
        deadline_.arm(now, cfg_.o2CommRetryMs);
      }
      break;

    case State::kWarming:
      if (!deadline_.reached(now)) break;
      if (!reader_.present()) fail(now);
      else if (warmup_.warm(now)) beginCycle(now);
      else deadline_.arm(now, cfg_.o2CommRetryMs);
      break;

    case State::kFlushing:
      if (deadline_.reached(now)) {
        uint16_t value;
        if (!reader_.readO2PercentX100(value) || value == 0) {  // O2-3a: exactly 0.00 % O2 is not a real reading
          fail(now);
          break;
        }
        sampleSum_ = value;
        sampleNum_ = 1;
        transition(State::kSampling, now, true, now + cfg_.o2SampleMs);
      }
      break;

    case State::kSampling:
      if (deadline_.reached(now)) {
        uint16_t value;
        if (!reader_.readO2PercentX100(value) || value == 0) {  // O2-3a: exactly 0.00 % O2 is not a real reading
          fail(now);
          break;
        }
        sampleSum_ += value;
        if (++sampleNum_ >= cfg_.o2SampleCount) {
          const uint32_t avg = (sampleSum_ + sampleNum_ / 2u) / sampleNum_;
          uint16_t n2 = avg >= 10000u ? 0 : static_cast<uint16_t>(10000u - avg);
          if (n2 > 9999) n2 = 9999;
          n2x100_ = n2;
          n2Valid_ = true;
          n2Stale_ = false;
          transition(State::kWaiting, now, true, cycleStart_ + cfg_.o2SampleIntervalMs);
        } else {
          deadline_.arm(now, cfg_.o2SampleMs);
        }
      }
      break;

    case State::kWaiting:
      if (deadline_.reached(now)) {
        if (gateOk(in)) beginCycle(now);
        else if (n2Valid_) n2Stale_ = true;  // keep the last value but flag it (O2-7)
      }
      break;

    case State::kError:
      if (deadline_.reached(now)) {
        enabledAt_ = now;
        transition(State::kUnknown, now, true, now);
      }
      break;

    default:  // unreachable; recover safely (ARC-6)
      commOk_ = false;
      transition(State::kDisabled, now, false, 0);
      break;
  }
}

}  // namespace n2
