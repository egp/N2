// BoardPins.h — every pin, I2C address, direction and active level (PIN-1).
//
// This is the ONLY file that contains pin numbers, I2C addresses, or the
// meaning of HIGH/LOW for a signal. Everything else refers to signals by name
// and goes through the helpers below.
//
// Pin numbers use the Uno header numbering, identical on the UNO R4 Minima and
// R4 WiFi in the Arduino core: D0..D13 = 0..13, A0..A5 = 14..19.
//
// Wiring facts come from N2V6/N2V7 (they agree). Everything is UNVERIFIED until
// the DIAG firmware has been run in production (PIN-7). Changing a pin or level
// means editing this file only; the compile-time checks at the bottom reject
// duplicate pins, signals on the I2C pins, and missing active levels (PIN-4).
#pragma once

#include <stdint.h>

#include "BuildConfig.h"

namespace n2 {

// ---- Pin numbers ------------------------------------------------------------
namespace pin {
constexpr uint8_t kD0 = 0, kD1 = 1, kD2 = 2, kD3 = 3, kD4 = 4, kD5 = 5, kD6 = 6;
constexpr uint8_t kD7 = 7, kD8 = 8, kD9 = 9, kD10 = 10, kD11 = 11, kD12 = 12, kD13 = 13;
constexpr uint8_t kNoPin = 0xFF;  // "not used" (for example: the LED is on the hardware I2C bus)
constexpr uint8_t kA0 = 14, kA1 = 15, kA2 = 16, kA3 = 17, kA4 = 18, kA5 = 19;

constexpr bool isAnalog(uint8_t p) { return p >= kA0 && p <= kA5; }
constexpr bool isValid(uint8_t p) { return p <= kA5; }
}  // namespace pin

// ---- Signals ----------------------------------------------------------------
enum class Signal : uint8_t {
  kTbs,             // The Black Switch: system on/off (maintained)
  kTob,             // The Other Button: momentary
  kLeftValve,       // left tower valve
  kRightValve,      // right tower valve
  kFlushValve,      // O2 sensor flush valve
  kSsr,             // compressor solid-state relay
  kAirPressure,     // air supply pressure transducer
  kN2LowPressure,   // low-pressure N2 transducer
  kN2HighPressure,  // high-pressure N2 transducer
  kCount
};
constexpr uint8_t kSignalCount = static_cast<uint8_t>(Signal::kCount);

enum class Dir : uint8_t { kInput, kInputPullup, kOutput, kAnalogInput };

// Which physical level means "on" (switch pressed, valve open, relay energized).
enum class Active : uint8_t { kHigh, kLow, kNotApplicable };

struct SignalDef {
  const char* name;
  uint8_t pin;
  Dir dir;
  Active active;
  const char* note;
};

struct BoardDef {
  const char* name;
  const SignalDef* signals;  // kSignalCount entries, in Signal order
  uint8_t signalCount;
  uint8_t sdaPin;
  uint8_t sclPin;
  uint8_t addrLed;        // TM1650 4-digit display: control register address
  uint8_t addrLedDigits;  // TM1650: first digit register address (digits 0..3 are consecutive)
  uint8_t addrLcd;  // 20x4 LCD, PCF8574 backpack
  uint8_t addrO2;   // DFRobot SEN0465 (SEL dip switch = 0)
  uint8_t addrRtc;  // DS3231 real-time clock (the usual module also carries an EEPROM at 0x57, which we do not use)
  // The LED module's OWN two-wire bus (software I2C on two ordinary pins), or kNoPin for the hardware bus. The TM1650 also answers at
  // 0x25-0x27, which clashes with an LCD backpack at 0x27, so the LED gets its own bus (docs/results/lcd-led-address-clash-20261007.md).
  uint8_t ledSdaPin = pin::kNoPin;
  uint8_t ledSclPin = pin::kNoPin;
};

constexpr bool ledOnSoftBus(const BoardDef& b) { return b.ledSdaPin != pin::kNoPin && b.ledSclPin != pin::kNoPin; }

// ---- Logical <-> physical level helpers (PIN-6) --------------------------------
// Code never writes HIGH/LOW for a signal; it states on/off and these convert.
constexpr bool levelHigh(bool on, Active a) { return a == Active::kLow ? !on : (a == Active::kHigh ? on : false); }
constexpr bool isOn(bool levelIsHigh, Active a) { return a == Active::kLow ? !levelIsHigh : (a == Active::kHigh && levelIsHigh); }
// The level an output takes when it is "off" (the safe state).
constexpr bool safeLevelHigh(Active a) { return levelHigh(false, a); }

// ---- The signal tables: ONE PER BOARD (UNVERIFIED — see header comment) --------------------
// Order must match enum Signal. The three boards are separate tables so the production Minima can be corrected to match the
// real wiring without touching the bench board, and the other way round. Today they hold the same values.

// UNO R4 Minima: PRODUCTION (Tom's garage). Must match the real wiring; V6/V7 agree on these rows.
inline constexpr SignalDef kMinimaSignals[kSignalCount] = {
    // name        pin           direction          active           note
    {"TBS",       pin::kD0,  Dir::kInputPullup, Active::kLow,  "maintained; V6/V7"},
    {"TOB",       pin::kD1,  Dir::kInputPullup, Active::kLow,  "momentary; V6/V7"},
    {"LEFT",      pin::kD4,  Dir::kOutput,      Active::kHigh, "HIGH = valve open; V6/V7"},
    {"RIGHT",     pin::kD7,  Dir::kOutput,      Active::kHigh, "HIGH = valve open; V6/V7"},
    {"FLUSH",     pin::kD11, Dir::kOutput,      Active::kHigh, "HIGH = valve open; V6/V7"},
    {"SSR",       pin::kD8,  Dir::kOutput,      Active::kHigh, "HIGH = compressor on; V6/V7"},
    {"AIR",       pin::kA0,  Dir::kAnalogInput, Active::kNotApplicable, "0-150 PSI; V6/V7"},
    {"N2LOW",     pin::kA3,  Dir::kAnalogInput, Active::kNotApplicable, "0-30 PSI; V6/V7"},
    // V6/V7 put N2-high on A5, which is the I2C SCL line. A1 is a PROVISIONAL
    // placeholder until the real wiring is confirmed (Requirements PIN-10).
    {"N2HIGH",    pin::kA1,  Dir::kAnalogInput, Active::kNotApplicable, "0-150 PSI; PROVISIONAL (V6/V7: A5 = SCL)"},
};


// UNO R4 WiFi: the HOME BENCH. Free to differ (the bench wiring may not match production). Today: TBS and TOB are momentary
// buttons on D0 and D1 (production TBS is an SPDT switch: same wiring, active LOW); nothing is on the valve/SSR/analog pins.
inline constexpr SignalDef kWifiSignals[kSignalCount] = {
    // name        pin           direction          active           note
    {"TBS",       pin::kD0,  Dir::kInputPullup, Active::kLow,  "maintained; V6/V7"},
    {"TOB",       pin::kD1,  Dir::kInputPullup, Active::kLow,  "momentary; V6/V7"},
    {"LEFT",      pin::kD4,  Dir::kOutput,      Active::kHigh, "HIGH = valve open; V6/V7"},
    {"RIGHT",     pin::kD7,  Dir::kOutput,      Active::kHigh, "HIGH = valve open; V6/V7"},
    {"FLUSH",     pin::kD11, Dir::kOutput,      Active::kHigh, "HIGH = valve open; V6/V7"},
    {"SSR",       pin::kD8,  Dir::kOutput,      Active::kHigh, "HIGH = compressor on; V6/V7"},
    {"AIR",       pin::kA0,  Dir::kAnalogInput, Active::kNotApplicable, "0-150 PSI; V6/V7"},
    {"N2LOW",     pin::kA3,  Dir::kAnalogInput, Active::kNotApplicable, "0-30 PSI; V6/V7"},
    // V6/V7 put N2-high on A5, which is the I2C SCL line. A1 is a PROVISIONAL
    // placeholder until the real wiring is confirmed (Requirements PIN-10).
    {"N2HIGH",    pin::kA1,  Dir::kAnalogInput, Active::kNotApplicable, "0-150 PSI; PROVISIONAL (V6/V7: A5 = SCL)"},
};


// Host tests: the production table.
inline constexpr const SignalDef (&kHostSignals)[kSignalCount] = kMinimaSignals;

// ---- The LCD backpack's I2C address (one setting for every board) ---------------------------------
// **0x23**: the backpack's A2 solder pad is BRIDGED (owner's decision 2026-10-08; Tom's unit gets the same bridge). The PCF8574 default is 0x27 (all three
// jumpers open, what V6/V7 used), but the TM1650 LED module also answers at 0x24-0x27, so with both on one bus every LCD character write failed
// (docs/results/lcd-led-address-clash-20261007.md). With A2 bridged the LCD is at 0x23, outside the LED's range; A1 would give 0x25 and A0 0x26, still inside it.
// An LCD that still has its default address would be at 0x27: build with -DN2_LCD_ADDRESS=0x27 to use it (then the LED must not share its bus).
#if defined(N2_LCD_ADDRESS)
inline constexpr uint8_t kLcdAddress = N2_LCD_ADDRESS;
#else
inline constexpr uint8_t kLcdAddress = 0x23;
#endif

// ---- Boards -------------------------------------------------------------------
// I2C: the core binds Wire to A4 (SDA) / A5 (SCL) on both boards.
// The owner reports D18/D19 on the WiFi; the core lists the same pins (18/19).
inline constexpr BoardDef kMinimaBoard = {"UNO R4 Minima", kMinimaSignals, kSignalCount,
                                          pin::kA4, pin::kA5, 0x24, 0x34, kLcdAddress, 0x74, 0x68};
inline constexpr BoardDef kWifiBoard = {"UNO R4 WiFi", kWifiSignals, kSignalCount,
                                        pin::kA4, pin::kA5, 0x24, 0x34, kLcdAddress, 0x74, 0x68};
inline constexpr BoardDef kHostBoard = {"host (fake)", kHostSignals, kSignalCount,
                                        pin::kA4, pin::kA5, 0x24, 0x34, kLcdAddress, 0x74, 0x68};

#if defined(N2_BOARD_MINIMA)
inline constexpr const BoardDef& kBoard = kMinimaBoard;
#elif defined(N2_BOARD_WIFI)
inline constexpr const BoardDef& kBoard = kWifiBoard;
#else
inline constexpr const BoardDef& kBoard = kHostBoard;
#endif

constexpr const SignalDef& def(const BoardDef& b, Signal s) { return b.signals[static_cast<uint8_t>(s)]; }
constexpr uint8_t pinOf(const BoardDef& b, Signal s) { return def(b, s).pin; }
constexpr bool isOutput(const SignalDef& d) { return d.dir == Dir::kOutput; }

// ---- The TM1650 answers more addresses than its datasheet lists ------------------------------
// Measured 2026-10-07 on the replacement module: it acknowledges 0x24, 0x25, 0x26, 0x27 (the whole control range, it ignores the low
// address bits) AND 0x34-0x37. So an LCD backpack at 0x27 (the PCF8574 default) shares an address with the LED module. With both on one bus
// the LCD writes failed over and over. Move the LCD to 0x20-0x23 (solder jumper A2 bridged gives 0x23).
constexpr bool inTm1650Range(const BoardDef& b, uint8_t a) {
  if (ledOnSoftBus(b)) return false;  // the LED is not on the shared bus, so it cannot answer there
  return (a >= b.addrLed && a < b.addrLed + 4) || (a >= b.addrLedDigits && a < b.addrLedDigits + 4);
}
// An address the TM1650 answers only because it ignores the low bits of its control address (0x25-0x27 when the control address is 0x24).
// It is the same chip: expected, never reported as an unexpected device (owner 2026-10-08).
constexpr bool isLedAlias(const BoardDef& b, uint8_t a) {
  if (ledOnSoftBus(b)) return false;
  return a > b.addrLed && a < b.addrLed + 4 && a != b.addrLcd;
}
// True if the LCD address is inside the range the LED module answers (a known hardware conflict; not a compile error because the
// production wiring is unconfirmed: see docs/Owner_TODO.md).
constexpr bool lcdOverlapsLed(const BoardDef& b) { return inTm1650Range(b, b.addrLcd); }

// ---- Compile-time validation (PIN-4) ----------------------------------------------
enum class BoardCheck : uint8_t {
  kOk,
  kWrongSignalCount,
  kBadPin,              // pin number outside D0..A5
  kBadPinKind,          // analog signal on a non-analog pin
  kMissingActiveLevel,  // digital signal without active level, or analog with one
  kDuplicatePin,        // two signals share a pin
  kSignalOnI2cPin,      // a signal uses SDA or SCL
  kBadI2cPins,          // SDA/SCL are not valid pins or are equal
  kDuplicateI2cAddress,  // two I2C devices share an address (the four TM1650 digit addresses included)
  kBadLedBusPins         // the LED's own bus pins: only one given, invalid, equal, on the I2C pins, or used by a signal
};

constexpr BoardCheck checkBoard(const BoardDef& b) {
  if (b.signalCount != kSignalCount) return BoardCheck::kWrongSignalCount;
  if (!pin::isValid(b.sdaPin) || !pin::isValid(b.sclPin) || b.sdaPin == b.sclPin)
    return BoardCheck::kBadI2cPins;
  for (uint8_t i = 0; i < b.signalCount; ++i) {
    const SignalDef& d = b.signals[i];
    if (!pin::isValid(d.pin)) return BoardCheck::kBadPin;
    if (d.dir == Dir::kAnalogInput) {
      if (!pin::isAnalog(d.pin)) return BoardCheck::kBadPinKind;
      if (d.active != Active::kNotApplicable) return BoardCheck::kMissingActiveLevel;
    } else if (d.active == Active::kNotApplicable) {
      return BoardCheck::kMissingActiveLevel;
    }
    if (d.pin == b.sdaPin || d.pin == b.sclPin) return BoardCheck::kSignalOnI2cPin;
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < b.signalCount; ++j)
      if (b.signals[j].pin == d.pin) return BoardCheck::kDuplicatePin;
  }
  if ((b.ledSdaPin != pin::kNoPin) != (b.ledSclPin != pin::kNoPin)) return BoardCheck::kBadLedBusPins;
  if (ledOnSoftBus(b)) {
    if (!pin::isValid(b.ledSdaPin) || !pin::isValid(b.ledSclPin) || b.ledSdaPin == b.ledSclPin) return BoardCheck::kBadLedBusPins;
    if (b.ledSdaPin == b.sdaPin || b.ledSdaPin == b.sclPin || b.ledSclPin == b.sdaPin || b.ledSclPin == b.sclPin) return BoardCheck::kBadLedBusPins;
    for (uint8_t i = 0; i < b.signalCount; ++i)
      if (b.signals[i].pin == b.ledSdaPin || b.signals[i].pin == b.ledSclPin) return BoardCheck::kBadLedBusPins;
  }
  // Addresses in use: LED control, LED digits (4 consecutive), LCD, O2. None may overlap.
  const uint8_t addrs[] = {b.addrLed, b.addrLcd, b.addrO2, b.addrRtc};
  for (uint8_t i = 0; i < 4; ++i)
    for (uint8_t j = static_cast<uint8_t>(i + 1); j < 4; ++j)
      if (addrs[i] == addrs[j]) return BoardCheck::kDuplicateI2cAddress;
  for (uint8_t a : addrs)
    if (a >= b.addrLedDigits && a < b.addrLedDigits + 4) return BoardCheck::kDuplicateI2cAddress;
  return BoardCheck::kOk;
}

static_assert(checkBoard(kMinimaBoard) == BoardCheck::kOk, "BoardPins.h: UNO R4 Minima pin table is invalid");
static_assert(checkBoard(kWifiBoard) == BoardCheck::kOk, "BoardPins.h: UNO R4 WiFi pin table is invalid");
static_assert(checkBoard(kHostBoard) == BoardCheck::kOk, "BoardPins.h: host pin table is invalid");

}  // namespace n2
