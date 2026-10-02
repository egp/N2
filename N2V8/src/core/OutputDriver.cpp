#include "OutputDriver.h"

#include "../board/BoardSetup.h"

namespace n2 {

OutputDriver::Out* OutputDriver::find(Signal s) {
  for (Out& o : out_)
    if (o.signal == s) return &o;
  return nullptr;
}
const OutputDriver::Out* OutputDriver::find(Signal s) const {
  for (const Out& o : out_)
    if (o.signal == s) return &o;
  return nullptr;
}

void OutputDriver::drive(Out& o, bool on, uint32_t now) {
  const SignalDef& d = def(board_, o.signal);
  hal_.digitalWrite(d.pin, levelHigh(on, d.active));
  o.on = on;
  o.lastChange = now;
  o.deferralLogged = false;
}

void OutputDriver::begin(uint32_t now) {
  driveOutputsSafe(hal_, board_);
  for (Out& o : out_) {
    o.on = false;
    o.lastChange = now;
    o.deferralLogged = false;
  }
}

void OutputDriver::apply(const OutputRequest& desired, const ForceOff& forced, uint32_t now) {
  const bool want[4] = {desired.left && !forced.left, desired.right && !forced.right,
                        desired.flush && !forced.flush, desired.ssr && !forced.ssr};
  const bool force[4] = {forced.left, forced.right, forced.flush, forced.ssr};
  for (uint8_t i = 0; i < 4; ++i) {
    Out& o = out_[i];
    if (want[i] == o.on) continue;
    if (!want[i] && force[i]) {  // INV-5: safety is never delayed
      drive(o, false, now);
      continue;
    }
    if (static_cast<uint32_t>(now - o.lastChange) >= minHoldMs_) {
      drive(o, want[i], now);
    } else {
      ++deferred_;
      if (!o.deferralLogged) {
        o.deferralLogged = true;
        logf(log_, LogLevel::kDebug, "%lu OUT %s change to %s deferred (min hold)", static_cast<unsigned long>(now),
             def(board_, o.signal).name, want[i] ? "ON" : "OFF");
      }
    }
  }
}

void OutputDriver::forceDrive(Signal signal, bool on, uint32_t now) {
  if (Out* o = find(signal)) drive(*o, on, now);
}

bool OutputDriver::actual(Signal signal) const {
  const Out* o = find(signal);
  return o != nullptr && o->on;
}

OutputRequest OutputDriver::actualState() const {
  OutputRequest r;
  r.left = actual(Signal::kLeftValve);
  r.right = actual(Signal::kRightValve);
  r.flush = actual(Signal::kFlushValve);
  r.ssr = actual(Signal::kSsr);
  return r;
}

}  // namespace n2
