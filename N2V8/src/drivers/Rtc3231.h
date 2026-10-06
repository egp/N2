// Rtc3231.h — minimal driver for the DS3231 battery-backed real-time clock (I2C address 0x68).
//
// Used for wall-clock time stamps (logs, reports) only; nothing in the control path depends on it, so a missing or unset
// clock is an INFO matter, never a safety one. It reads and sets the date/time, reports whether the time can be trusted
// (the oscillator-stop flag says the clock lost power at some point), and reads the chip's temperature sensor.
// All I2C goes through the Hal (host-testable). Every call reports success or failure and counts errors.
// Register map (DS3231): 0x00 seconds .. 0x06 year (BCD), 0x0E control, 0x0F status (bit7 = OSF), 0x11/0x12 temperature.
#pragma once

#include <stdint.h>

#include "../core/DateTime.h"
#include "../hal/Hal.h"

namespace n2 {

class Rtc3231 {
 public:
  explicit Rtc3231(Hal& hal, uint8_t address = 0x68) : hal_(hal), address_(address) {}

  bool present();                            // does the chip acknowledge its address?
  bool read(DateTime& out);                  // false: no answer, or the registers do not hold a valid date/time
  bool set(const DateTime& t);               // writes the time (24-hour mode) and clears the oscillator-stop flag
  bool timeValid(bool& valid);               // valid = the oscillator-stop flag is CLEAR (the clock has not lost power since set)
  bool temperatureX100(int16_t& out);        // degrees Celsius x 100 (resolution 0.25 degrees)

  uint32_t i2cErrors() const { return errors_; }
  bool lastOk() const { return lastOk_; }

  // Pure helpers, public so the tests can check them directly.
  static uint8_t bcdToDec(uint8_t b) { return static_cast<uint8_t>((b >> 4) * 10 + (b & 0x0F)); }
  static uint8_t decToBcd(uint8_t d) { return static_cast<uint8_t>(((d / 10) << 4) | (d % 10)); }
  static bool bcdOk(uint8_t b) { return (b >> 4) <= 9 && (b & 0x0F) <= 9; }

  static constexpr uint8_t kRegSeconds = 0x00;
  static constexpr uint8_t kRegStatus = 0x0F;
  static constexpr uint8_t kRegTemp = 0x11;
  static constexpr uint8_t kStatusOsf = 0x80;

 private:
  bool ok(bool success) {
    lastOk_ = success;
    if (!success) ++errors_;
    return success;
  }

  Hal& hal_;
  uint8_t address_;
  uint32_t errors_ = 0;
  bool lastOk_ = true;
};

}  // namespace n2
