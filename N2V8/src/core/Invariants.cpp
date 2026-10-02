#include "Invariants.h"

namespace n2 {

namespace {
// One rule: when `active`, these outputs must be off.
struct Rule {
  bool active;
  uint16_t bit;
  bool left, right, flush, ssr;
};
}  // namespace

InvariantResult checkInvariants(const Inputs& in, const ControlConfig& cfg, const OutputRequest& req) {
  const bool airBad = !in.airOk || in.airX10 < cfg.airLowOff;                       // INV-2
  const bool highBad = !in.n2HighOk || in.n2HighX10 > cfg.n2HighOff;                // INV-3
  const bool lowBad = !in.n2LowOk || in.n2LowX100 < cfg.n2LowOff;                   // INV-4

  const Rule rules[] = {
      {!in.tbs,                                      kInv1TbsOff,      true,  true,  true,  true},   // INV-1 all off
      {airBad,                                       kInv2Air,         true,  true,  false, false},  // INV-2 towers
      {highBad,                                      kInv3N2High,      true,  true,  false, true},   // INV-3 towers + SSR
      {lowBad,                                       kInv4N2Low,       false, false, false, true},   // INV-4 SSR
      {in.sensorOrderFault,                          kInv8SensorOrder, true,  true,  false, true},   // INV-8 as INV-3 + INV-4
      {cfg.o2Mandatory && !in.o2CommOk,              kInv9O2Missing,   true,  true,  true,  true},   // INV-9 all off
      {cfg.o2Mandatory && !in.o2Warm,                kInv10O2Warming,  true,  true,  false, false},  // INV-10 towers
  };

  InvariantResult r;
  for (const Rule& rule : rules) {
    if (!rule.active) continue;
    r.forced.left |= rule.left;
    r.forced.right |= rule.right;
    r.forced.flush |= rule.flush;
    r.forced.ssr |= rule.ssr;
    if ((rule.left && req.left) || (rule.right && req.right) || (rule.flush && req.flush) || (rule.ssr && req.ssr))
      r.violated |= rule.bit;
  }
  return r;
}

}  // namespace n2
