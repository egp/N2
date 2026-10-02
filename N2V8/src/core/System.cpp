#include "System.h"

namespace n2 {

System::System(Hal& hal, const BoardDef& board, const ControlConfig& cfg, O2Reader& o2Reader, LogSink& log,
               uint8_t adcBits)
    : hal_(hal),
      board_(board),
      cfg_(cfg),
      log_(log),
      transitions_(log),
      faults_(log),
      sensors_(board_, cfg_, adcBits),
      warmup_(cfg_.o2WarmupMs),
      tower_(cfg_, transitions_),
      compressor_(cfg_, transitions_),
      o2_(cfg_, o2Reader, warmup_, transitions_),
      driver_(hal, board_, log, cfg_.outputMinHoldMs) {}

bool System::readOn(Signal s) {
  const SignalDef& d = def(board_, s);
  return isOn(hal_.digitalRead(d.pin), d.active);
}

void System::begin(const ResetInfo& reset, uint32_t warmCreditMs) {
  const uint32_t now = hal_.millis();
  driver_.begin(now);                       // RST-2: outputs safe
  warmup_.begin(now, warmCreditMs);         // RST-3 / O2-6
  prevTbs_ = false;                         // a TBS that is ON at boot counts as OFF->ON (INP-6)
  prevEnabled_ = false;
  resetWasWatchdog_ = reset.watchdog;
  resetWasBrownout_ = reset.brownout;
  faults_.report(FaultId::kWatchdogReset, reset.watchdog, now, cfg_.faultHoldMs);
  faults_.report(FaultId::kBrownoutReset, reset.brownout, now, cfg_.faultHoldMs);
}

void System::resume() {
  const uint32_t now = hal_.millis();
  driver_.begin(now);
  tower_.disable(now);
  compressor_.disable(now);
  o2_.disable(now);
  prevTbs_ = false;
  prevEnabled_ = false;
}

void System::step() {
  const uint32_t now = hal_.millis();
  in_.ms = now;
  in_.tbs = readOn(Signal::kTbs);
  in_.tob = readOn(Signal::kTob);
  sensors_.sample(hal_, now, faults_, in_);

  if (!controllersEnabled_) {  // DIAG: look, but do not touch
    request_ = OutputRequest();
    invariants_ = checkInvariants(in_, cfg_, request_);
    driver_.apply(request_, invariants_.forced, now);
    return;
  }

  // TBS edges drive the O2 controller (it must run to find the sensor).
  if (in_.tbs && !prevTbs_) {
    logf(log_, LogLevel::kInfo, "%lu TBS OFF->ON", static_cast<unsigned long>(now));
    o2_.enable(now);
  } else if (!in_.tbs && prevTbs_) {
    logf(log_, LogLevel::kInfo, "%lu TBS ON->OFF", static_cast<unsigned long>(now));
    o2_.disable(now);
  }
  prevTbs_ = in_.tbs;

  o2_.update(in_);
  in_.o2CommOk = o2_.commOk();
  in_.o2Warm = warmup_.warm(now);
  faults_.report(FaultId::kO2Comm, o2_.state() == O2Controller::State::kError, now, cfg_.faultHoldMs);

  // Tower and compressor run only while TBS is on and (in production) the O2 sensor is present (O2-1, INV-9).
  const bool enabledNow = in_.tbs && (!cfg_.o2Mandatory || in_.o2CommOk);
  if (enabledNow && !prevEnabled_) {
    tower_.enable();
    compressor_.enable(in_);
  } else if (!enabledNow && prevEnabled_) {
    tower_.disable(now);
    compressor_.disable(now);
  }
  prevEnabled_ = enabledNow;

  tower_.update(in_);
  compressor_.update(in_);

  request_.left = tower_.leftOpen();
  request_.right = tower_.rightOpen();
  request_.flush = o2_.flushOpen();
  request_.ssr = compressor_.ssrOn();

  invariants_ = checkInvariants(in_, cfg_, request_);
  if (invariants_.violated != 0) {  // INV-6: a controller asked for something forbidden
    ++violations_;
    logf(log_, LogLevel::kError, "%lu INVARIANT VIOLATED mask=0x%02x", static_cast<unsigned long>(now),
         static_cast<unsigned>(invariants_.violated));
  }
  faults_.report(FaultId::kInvariant, invariants_.violated != 0, now, cfg_.faultHoldMs);

  driver_.apply(request_, invariants_.forced, now);

  // F30/F31 are informational history: clear them once the plant has run a normal cycle.
  if (tower_.state() == Tower::State::kRight) normalCycleSeen_ = true;
  if (normalCycleSeen_) {
    faults_.report(FaultId::kWatchdogReset, false, now, cfg_.faultHoldMs);
    faults_.report(FaultId::kBrownoutReset, false, now, cfg_.faultHoldMs);
  }
}

}  // namespace n2
