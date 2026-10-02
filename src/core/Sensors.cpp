#include "Sensors.h"

namespace n2 {

bool SensorChannel::update(uint16_t raw, uint8_t adcBits, uint8_t samplesNeeded) {
  if (classifyRaw(raw, adcBits) == RawStatus::kOk) {
    bad_ = 0;
    asserted_ = false;
  } else {
    if (bad_ < 255) ++bad_;
    asserted_ = bad_ >= samplesNeeded;
  }
  return asserted_;
}

void SensorMonitor::sample(Hal& hal, uint32_t now, FaultSet& faults, Inputs& in) {
  const uint16_t rawAir = hal.analogRead(pinOf(board_, Signal::kAirPressure));
  const uint16_t rawLow = hal.analogRead(pinOf(board_, Signal::kN2LowPressure));
  const uint16_t rawHigh = hal.analogRead(pinOf(board_, Signal::kN2HighPressure));

  faults.report(FaultId::kAirSensor, air_.update(rawAir, adcBits_, cfg_.sensorFaultSamples), now, cfg_.faultHoldMs);
  faults.report(FaultId::kN2LowSensor, n2Low_.update(rawLow, adcBits_, cfg_.sensorFaultSamples), now, cfg_.faultHoldMs);
  faults.report(FaultId::kN2HighSensor, n2High_.update(rawHigh, adcBits_, cfg_.sensorFaultSamples), now, cfg_.faultHoldMs);

  in.rawAir = rawAir;
  in.rawN2Low = rawLow;
  in.rawN2High = rawHigh;
  in.airX10 = scalePressure(rawAir, adcBits_, kAirFullScaleX10);
  in.n2LowX100 = scalePressure(rawLow, adcBits_, kN2LowFullScaleX100);
  in.n2HighX10 = scalePressure(rawHigh, adcBits_, kN2HighFullScaleX10);
  in.airOk = !faults.active(FaultId::kAirSensor);
  in.n2LowOk = !faults.active(FaultId::kN2LowSensor);
  in.n2HighOk = !faults.active(FaultId::kN2HighSensor);

  // INP-7: N2-low must read lower than N2-high (compare in PSI x100). Only judged when both sensors are sound.
  const uint32_t highX100 = static_cast<uint32_t>(in.n2HighX10) * 10u;
  const bool bad = in.n2LowOk && in.n2HighOk && (in.n2LowX100 > highX100 + cfg_.sensorOrderMarginX100);
  if (bad) {
    if (!orderBad_) {
      orderBad_ = true;
      orderBadSince_ = now;
    }
  } else {
    orderBad_ = false;
  }
  const bool sustained = orderBad_ && static_cast<uint32_t>(now - orderBadSince_) >= cfg_.sensorOrderHoldMs;
  faults.report(FaultId::kSensorOrder, sustained, now, cfg_.faultHoldMs);
  in.sensorOrderFault = faults.active(FaultId::kSensorOrder);
}

}  // namespace n2
