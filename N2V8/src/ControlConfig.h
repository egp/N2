// ControlConfig.h — thresholds and timings (Requirements §7). Values from V6/V7.
//
// A runtime struct initialised from kDefaultControl, so that a later version can
// change the pressure thresholds from the console (FUT-1). `cfg` prints it.
// Units: air and N2-high are PSI x10; N2-low is PSI x100; times are ms.
#pragma once

#include <stdint.h>

namespace n2 {

struct ControlConfig {
  // Tower
  uint16_t airLowOff;        // disable below
  uint16_t airLowOn;         // may start above
  uint32_t towerFillMs;
  uint32_t towerOverlapMs;
  // Compressor
  uint16_t n2LowOff;         // SSR off below
  uint16_t n2LowOn;          // SSR may start above
  uint16_t n2HighOn;         // SSR/tower may start below
  uint16_t n2HighOff;        // SSR/tower stop above
  // O2
  uint32_t o2SampleIntervalMs;  // cycle start to cycle start
  uint32_t o2FlushMs;
  uint32_t o2SampleMs;          // between samples
  uint8_t o2SampleCount;
  uint32_t o2CommRetryMs;       // while UNKNOWN / WARMING
  uint32_t o2CommTimeoutMs;     // UNKNOWN this long without an answer -> ERROR
  uint32_t o2ErrorRetryMs;      // ERROR -> UNKNOWN after this (O2-2)
  uint32_t o2WarmupMs;          // 5 minutes (O2-6)
  bool o2Mandatory;             // FIELD: no sensor, no production (O2-1, INV-9, INV-10)
  // Outputs and faults
  uint32_t outputMinHoldMs;     // OUT-1
  uint8_t sensorFaultSamples;   // INP-5
  uint32_t faultHoldMs;         // FLT-1
  uint16_t sensorOrderMarginX100;  // INP-7: N2-low may exceed N2-high by this much (PSI x100)
  uint32_t sensorOrderHoldMs;
  uint32_t airGraceFromOffMs;   // owner 2026-10-09: low air is tolerated this long after a tower valve opens from OFF (the supply sags ~35 PSI for ~300 ms and recovers in ~2 s)
  uint32_t airGraceToBothMs;    // ... and this long after LEFT->BOTH or RIGHT->BOTH (the second valve opens)
  uint32_t sensorFaultMs;       // INP-5: an out-of-window reading must ALSO last this long before it is a fault (a switching spike of a few ms must not trip F01..F03)
  uint32_t overlapMinMs;        // TWR-OV: the overlap is never ended by the air minimum earlier than this (towerOverlapMs is the CAP)
  bool overlapAdaptive;         // TWR-OV: true = end the overlap at the air minimum (min/cap above); false = the fixed towerOverlapMs (Tom's decision 2026-10-10: fixed 500 ms until the CMS characterization)
  uint16_t overlapRiseX10;      // TWR-OV: the air counts as past its minimum when it is this much above it (PSI x10), twice in a row on the 50 ms grid
};

inline constexpr ControlConfig kDefaultControl = {
    650, 900, 59250, 500,                          // tower (air off 65 / on 90 PSI: owner 2026-10-09, this machine's supply peaks at 100 PSI and sags when a valve opens)
    1000, 2000, 1000, 1200,                        // compressor
    60000, 2000, 250, 10, 1000, 3000, 60000, 300000, true,  // O2
    1000, 3, 5000, 100, 5000, 3000, 2000, 50, 200, false, 10};     // outputs and faults (then: air grace 3000 ms from OFF, 2000 ms to BOTH; sensor fault persistence 50 ms; overlap: not before 200 ms, air counted past its minimum at +1.0 PSI)

// CFG-5: hysteresis pairs must be ordered.
constexpr bool validControl(const ControlConfig& c) {
  return c.airLowOn > c.airLowOff && c.n2LowOn > c.n2LowOff && c.n2HighOff > c.n2HighOn &&
         c.o2SampleCount > 0 && c.sensorFaultSamples > 0 && c.towerFillMs > 0;
}
static_assert(validControl(kDefaultControl), "ControlConfig.h: default control config is invalid");

}  // namespace n2
