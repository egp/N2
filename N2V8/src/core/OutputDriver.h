// OutputDriver.h — the single place that writes the four output pins (ARC-8, OUT-1, INV-5).
//
// Controllers request; the driver decides. It applies the active level from BoardPins.h,
// forces outputs off at once when an invariant says so (never delayed), and otherwise
// refuses to change an output until `minHoldMs` has passed since its last change
// (anti-buzz and short-cycle protection).
#pragma once

#include <stdint.h>

#include "../BoardPins.h"
#include "../hal/Hal.h"
#include "Invariants.h"
#include "Log.h"
#include "Snapshots.h"

namespace n2 {

class OutputDriver {
 public:
  OutputDriver(Hal& hal, const BoardDef& board, LogSink& log, uint32_t minHoldMs)
      : hal_(hal), board_(board), log_(log), minHoldMs_(minHoldMs) {}

  // Drive every output to its safe level. The boot time counts as the last change,
  // so nothing can switch on within minHoldMs of a reset.
  void begin(uint32_t now);

  void apply(const OutputRequest& desired, const ForceOff& forced, uint32_t now);

  // BIST only: set one output directly, ignoring the minimum hold. The caller is
  // responsible for rate (>= 500 ms per change) and for the invariant veto (BIST-11).
  void forceDrive(Signal signal, bool on, uint32_t now);

  bool actual(Signal signal) const;
  OutputRequest actualState() const;
  uint32_t deferredCount() const { return deferred_; }

 private:
  struct Out {
    Signal signal;
    bool on;
    uint32_t lastChange;
    bool deferralLogged;
  };
  Out* find(Signal s);
  const Out* find(Signal s) const;
  void drive(Out& o, bool on, uint32_t now);

  Hal& hal_;
  const BoardDef& board_;
  LogSink& log_;
  uint32_t minHoldMs_;
  uint32_t deferred_ = 0;
  Out out_[4] = {{Signal::kLeftValve, false, 0, false},
                 {Signal::kRightValve, false, 0, false},
                 {Signal::kFlushValve, false, 0, false},
                 {Signal::kSsr, false, 0, false}};
};

}  // namespace n2
