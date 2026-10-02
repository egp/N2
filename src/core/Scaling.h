// Scaling.h — ADC counts to fixed-point pressure (Requirements INP-2..INP-4).
//
// Header-only and constexpr so it is trivially host-testable. Every raw limit
// is derived from the ADC bit depth and the sensor's 0.5-4.5 V window; nothing
// is hardcoded for 10 bits. At 10 bits the results equal V7's 102 / 921.
#pragma once

#include <stdint.h>

#include "../Config.h"

namespace n2 {

constexpr uint32_t adcMaxRaw(uint8_t bits) { return (1u << bits) - 1u; }

// Raw ADC count for a voltage, rounded to nearest.
constexpr uint16_t rawFromMillivolts(uint8_t bits, uint16_t millivolts) {
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(millivolts) * adcMaxRaw(bits) + kAdcRefMillivolts / 2u) / kAdcRefMillivolts);
}

struct AdcWindow {
  uint16_t validMin;  // raw count at 0.5 V  (0 PSI)
  uint16_t validMax;  // raw count at 4.5 V  (full scale)
  uint16_t faultMin;  // below this: sensor fault
  uint16_t faultMax;  // above this: sensor fault
};

constexpr AdcWindow adcWindow(uint8_t bits) {
  return {rawFromMillivolts(bits, kSensorMinMillivolts), rawFromMillivolts(bits, kSensorMaxMillivolts),
          rawFromMillivolts(bits, kSensorFaultLowMillivolts), rawFromMillivolts(bits, kSensorFaultHighMillivolts)};
}

// Voltage at the ADC pin for a raw count, in millivolts (diagnostics only).
constexpr uint16_t millivoltsFromRaw(uint16_t raw, uint8_t bits) {
  return static_cast<uint16_t>((static_cast<uint32_t>(raw) * kAdcRefMillivolts + adcMaxRaw(bits) / 2u) / adcMaxRaw(bits));
}

enum class RawStatus : uint8_t { kOk, kBelowWindow, kAboveWindow };

// Single-sample plausibility. (Requiring several consecutive bad samples
// before declaring a fault is the Faults module's job.)
constexpr RawStatus classifyRaw(uint16_t raw, uint8_t bits) {
  const AdcWindow w = adcWindow(bits);
  return raw < w.faultMin ? RawStatus::kBelowWindow : (raw > w.faultMax ? RawStatus::kAboveWindow : RawStatus::kOk);
}

// Linear map of the valid window onto 0..fullScale, clamped, integer arithmetic
// (identical to Arduino's map() as used by V7).
constexpr uint16_t scalePressure(uint16_t raw, uint8_t bits, uint16_t fullScale) {
  const AdcWindow w = adcWindow(bits);
  const uint32_t clamped = raw < w.validMin ? w.validMin : (raw > w.validMax ? w.validMax : raw);
  return static_cast<uint16_t>(((clamped - w.validMin) * static_cast<uint32_t>(fullScale)) /
                               (static_cast<uint32_t>(w.validMax) - w.validMin));
}

}  // namespace n2
