// Config.h — constants that describe the sensors and the ADC.
// Control thresholds and timings are added here in a later milestone (M4).
//
// Naming: constants are kCamelCase. A requirement's ADC_BITS is kAdcBits here.
#pragma once

#include <stdint.h>

namespace n2 {

// ADC resolution (Requirements INP-2). The RA4M1 supports 10, 12 and 14 bits.
// Start at 10; everything derived from it (raw limits, scaling) follows.
constexpr uint8_t kAdcBits = 0x0A;
static_assert(kAdcBits == 10 || kAdcBits == 12 || kAdcBits == 14,
              "kAdcBits must be 10, 12 or 14");

// ADC reference voltage on the UNO R4 (5 V).
constexpr uint16_t kAdcRefMillivolts = 5000;

// Pressure transducers are ratiometric 0.5-4.5 V over their full range.
constexpr uint16_t kSensorMinMillivolts = 500;
constexpr uint16_t kSensorMaxMillivolts = 4500;

// A reading outside this wider window means a wiring or sensor fault, not a
// pressure (Requirements INP-4).
constexpr uint16_t kSensorFaultLowMillivolts = 400;
constexpr uint16_t kSensorFaultHighMillivolts = 4600;

// Full-scale pressures, fixed point. Air and N2-high: PSI x10; N2-low: PSI x100.
constexpr uint16_t kAirFullScaleX10 = 1500;       // 0-150.0 PSI
constexpr uint16_t kN2LowFullScaleX100 = 3000;    // 0-30.00 PSI
constexpr uint16_t kN2HighFullScaleX10 = 1500;    // 0-150.0 PSI

// Where the O2 warm-up record (WarmRecord) lives in RAM (Requirements O2-6a).
// NOT in a .noinit section: on this core the startup code overwrites that area from flash on every boot (measured on the
// R4 WiFi with experiments/reset_probe). Ordinary RAM survives software, watchdog and reset-button resets, so the record sits
// at a fixed address just below the heap limit (0x20007B00 on both boards) and well above the stack's bottom edge.
// The sketch refuses to use it if the heap could ever reach it (see N2V8.ino).
// I2C bus clock for the whole firmware: 100 kHz (the R4 default and the PCF8574 LCD backpack's rated maximum). The owner expected 30 kHz
// to be plenty, but the R4 hardware I2C cannot do it (see the static_assert below). Debug override: -DN2_I2C_HZ=...
#if defined(N2_I2C_HZ)
constexpr uint32_t kI2cClockHz = N2_I2C_HZ;
#else
constexpr uint32_t kI2cClockHz = 100000;
#endif
// Longest time one I2C transaction may take before the call gives up. The R4 Wire default is 100 ms PER TRANSACTION and one LCD refresh is dozens of
// them: on a glitched bus that stalls the loop for seconds (3 freezes / watchdog resets on Tom's machine, 2026-10-09). A real 4-byte transfer takes
// about 0.5 ms at 100 kHz, so 3 ms is generous. (Wire only honours it when the bus is held, not on a NACK: that is the case that stalled.)
constexpr uint32_t kI2cTimeoutUs = 3000;
// The UNO R4 Wire really has ONLY these two rates; any other value is silently ignored (the clock stays where it was).
static_assert(kI2cClockHz == 100000 || kI2cClockHz == 400000,
              "Config.h: kI2cClockHz must be 100000 or 400000 (the R4 Wire.setClock has no other real speed)");

constexpr uint32_t kWarmRecordAddress = 0x20007A00u;
constexpr uint32_t kBistRecordAddress = 0x20007A40u;
constexpr uint32_t kStallRecordAddress = 0x20007AC0u;   // the stall breadcrumb (core/StallRecord.h), 36 bytes
constexpr uint32_t kModeRecordAddress = 0x20007A80u;   // the run-mode breadcrumb (app/RunMode.h)   // the BIST resume record (selftest/Bist.h); same RAM that survives a reset-button reset

}  // namespace n2
