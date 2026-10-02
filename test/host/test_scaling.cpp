// Tests for Scaling.h: INP-2 (ADC_BITS 10/12/14), INP-3 (scaling), INP-4 (fault window).
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "Config.h"
#include "core/Scaling.h"

using namespace n2;

namespace {
// Arduino map() as V7 used it, for the equivalence check at 10 bits.
long arduinoMap(long x, long inMin, long inMax, long outMin, long outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
}  // namespace

TEST_CASE("INP-2: at 10 bits the windows equal V7's constants") {
  const AdcWindow w = adcWindow(10);
  CHECK(w.validMin == 102);
  CHECK(w.validMax == 921);
  CHECK(w.faultMin == 82);
  CHECK(w.faultMax == 941);
}

TEST_CASE("INP-2: window values at 12 and 14 bits") {
  CHECK(adcWindow(12).validMin == 410);  // 409.5 rounds to nearest
  CHECK(adcWindow(12).validMax == 3686);
  CHECK(adcWindow(14).validMin == 1638);
  CHECK(adcWindow(14).validMax == 14745);
}

TEST_CASE("INP-2: window is ordered and inside the ADC range for every bit depth") {
  const uint8_t bits = static_cast<uint8_t>(GENERATE(10, 12, 14));
  const AdcWindow w = adcWindow(bits);
  INFO("bits=" << int(bits));
  CHECK(w.faultMin < w.validMin);
  CHECK(w.validMin < w.validMax);
  CHECK(w.validMax < w.faultMax);
  CHECK(w.faultMax < adcMaxRaw(bits));
}

TEST_CASE("INP-3: scaling endpoints, clamping and monotonicity at every bit depth") {
  const uint8_t bits = static_cast<uint8_t>(GENERATE(10, 12, 14));
  const uint16_t fullScale = static_cast<uint16_t>(GENERATE(1500, 3000));
  const AdcWindow w = adcWindow(bits);
  INFO("bits=" << int(bits) << " fullScale=" << fullScale);

  CHECK(scalePressure(w.validMin, bits, fullScale) == 0);
  CHECK(scalePressure(w.validMax, bits, fullScale) == fullScale);
  CHECK(scalePressure(0, bits, fullScale) == 0);                                      // clamped low
  CHECK(scalePressure(static_cast<uint16_t>(adcMaxRaw(bits)), bits, fullScale) == fullScale);  // clamped high

  uint16_t previous = 0;
  for (uint32_t raw = 0; raw <= adcMaxRaw(bits); ++raw) {
    const uint16_t p = scalePressure(static_cast<uint16_t>(raw), bits, fullScale);
    REQUIRE(p >= previous);
    REQUIRE(p <= fullScale);
    previous = p;
  }
}

TEST_CASE("INP-3: mid-window voltage scales to about half of full scale at every bit depth") {
  const uint8_t bits = static_cast<uint8_t>(GENERATE(10, 12, 14));
  const uint16_t mid = scalePressure(rawFromMillivolts(bits, 2500), bits, 1500);
  INFO("bits=" << int(bits) << " mid=" << mid);
  CHECK(mid >= 749);
  CHECK(mid <= 751);
}

TEST_CASE("INP-3: at 10 bits scaling equals V7's Arduino map() for every raw value") {
  for (uint16_t fullScale : {kAirFullScaleX10, kN2LowFullScaleX100, kN2HighFullScaleX10}) {
    for (int raw = 0; raw <= 1023; ++raw) {
      const long clamped = raw < 102 ? 102 : (raw > 921 ? 921 : raw);
      const long expected = arduinoMap(clamped, 102, 921, 0, fullScale);
      REQUIRE(scalePressure(static_cast<uint16_t>(raw), 10, fullScale) == expected);
    }
  }
}

TEST_CASE("INP-4: classifyRaw flags readings outside the fault window") {
  const uint8_t bits = static_cast<uint8_t>(GENERATE(10, 12, 14));
  const AdcWindow w = adcWindow(bits);
  INFO("bits=" << int(bits));
  CHECK(classifyRaw(0, bits) == RawStatus::kBelowWindow);
  CHECK(classifyRaw(static_cast<uint16_t>(w.faultMin - 1), bits) == RawStatus::kBelowWindow);
  CHECK(classifyRaw(w.faultMin, bits) == RawStatus::kOk);
  CHECK(classifyRaw(w.validMin, bits) == RawStatus::kOk);
  CHECK(classifyRaw(w.validMax, bits) == RawStatus::kOk);
  CHECK(classifyRaw(w.faultMax, bits) == RawStatus::kOk);
  CHECK(classifyRaw(static_cast<uint16_t>(w.faultMax + 1), bits) == RawStatus::kAboveWindow);
  CHECK(classifyRaw(static_cast<uint16_t>(adcMaxRaw(bits)), bits) == RawStatus::kAboveWindow);
}

TEST_CASE("INP-4: a disconnected sensor reads as a fault, not as 0 PSI") {
  // V7 clamped raw 0 to "0 PSI". classifyRaw makes the difference visible.
  CHECK(scalePressure(0, kAdcBits, kN2HighFullScaleX10) == 0);
  CHECK(classifyRaw(0, kAdcBits) == RawStatus::kBelowWindow);
}
