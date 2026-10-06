#include "DisplayData.h"

#include "../core/System.h"

namespace n2 {

DisplayData makeDisplayData(const System& sys, uint8_t adcBits) {
  DisplayData d;
  const Inputs& in = sys.inputs();
  const uint32_t now = in.ms;
  d.airX10 = in.airX10;
  d.n2LowX100 = in.n2LowX100;
  d.n2HighX10 = in.n2HighX10;
  d.rawAir = in.rawAir;
  d.rawN2Low = in.rawN2Low;
  d.rawN2High = in.rawN2High;
  d.adcBits = adcBits;

  const O2Controller& o2 = sys.o2();
  d.n2Valid = o2.n2Valid();
  d.n2Stale = o2.n2Stale();
  d.n2PercentX100 = o2.n2PercentX100();
  d.warmRemainingMs = o2.warmRemainingMs(now);
  d.warming = d.warmRemainingMs > 0 && sys.o2().state() != O2Controller::State::kDisabled;

  d.tower = Tower::name(sys.tower().state());
  d.compressor = Compressor::name(sys.compressor().state());
  d.o2 = O2Controller::name(o2.state());

  const OutputRequest out = sys.outputs().actualState();
  d.left = out.left;
  d.right = out.right;
  d.flush = out.flush;
  d.ssr = out.ssr;
  d.tbs = in.tbs;

  FaultId id;
  d.lastFaultCode = sys.faults().lastCode();
  for (uint8_t i = 0; i < kFaultCount && sys.faults().nth(i, Severity::kWarn, id); ++i) d.faults[d.faultCount++] = id;
  return d;
}

}  // namespace n2
