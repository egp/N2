// Invariants.h — the always-true safety rules (Requirements §6). SAFETY-CRITICAL (GOAL-11).
//
// Pure function of the inputs. It says which outputs must be OFF right now, and
// which invariants a controller's request would violate (a violation is a software
// bug: INV-6). It is checked after the controllers and before outputs are driven,
// in addition to the same rules being built into each controller (defence in depth).
#pragma once

#include <stdint.h>

#include "../ControlConfig.h"
#include "Snapshots.h"

namespace n2 {

struct ForceOff {
  bool left = false;
  bool right = false;
  bool flush = false;
  bool ssr = false;
};

// Bit mask of violated invariants, one bit per rule.
enum InvariantBit : uint16_t {
  kInv1TbsOff = 1u << 0,
  kInv2Air = 1u << 1,
  kInv3N2High = 1u << 2,
  kInv4N2Low = 1u << 3,
  kInv8SensorOrder = 1u << 4,
  kInv9O2Missing = 1u << 5,
  kInv10O2Warming = 1u << 6,
};

struct InvariantResult {
  ForceOff forced;
  uint16_t violated = 0;  // rules whose forced-off outputs the request wanted ON
};

InvariantResult checkInvariants(const Inputs& in, const ControlConfig& cfg, const OutputRequest& requested);

}  // namespace n2
