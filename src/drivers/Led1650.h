// Led1650.h — minimal driver for the TM1650 4-digit, 8-segment LED display (Requirements §3 DRV-1..DRV-3).
//
// Non-blocking: begin() starts a power-up wait, service() does the I2C writes. Only digits whose content
// changed are written. A failed transaction is reported (healthy() false, errors counted) and the driver
// re-initialises itself when the device answers again (DSP-7). Starts from the V7 mini-library.
#pragma once

#include <stdint.h>

#include "../hal/Hal.h"
#include "../ui/LedText.h"

namespace n2 {

class Led1650 {
 public:
  static constexpr uint8_t kDigits = 4;
  static constexpr uint32_t kPowerUpMs = 50;
  static constexpr uint32_t kRetryMs = 1000;
  static constexpr uint8_t kControlOn = 0x11;   // brightness 1, 8-segment mode, display on
  static constexpr uint8_t kControlOff = 0x10;  // same, display off

  Led1650(Hal& hal, uint8_t controlAddress, uint8_t digitBaseAddress)
      : hal_(hal), control_(controlAddress), digitBase_(digitBaseAddress) {}

  void begin(uint32_t now);
  void service(uint32_t now);

  void setText(const LedText& text);  // desired content; written by service()
  void setDisplayOn(bool on);         // BIST: display on/off without changing the content

  bool ready() const { return state_ == State::kReady; }
  bool healthy() const { return healthy_; }
  uint32_t i2cErrors() const { return errors_; }
  bool inSync() const;  // every desired digit has been written

  // Segment bits for a character: '0'-'9', 'A'-'F', '-', ' '. Unknown characters show blank.
  static uint8_t segmentsFor(char c);

 private:
  enum class State : uint8_t { kIdle, kPowerWait, kReady, kRetry };
  void fail(uint32_t now);

  Hal& hal_;
  uint8_t control_;
  uint8_t digitBase_;
  State state_ = State::kIdle;
  uint32_t until_ = 0;
  bool healthy_ = true;
  uint32_t errors_ = 0;
  bool controlDirty_ = true;
  bool displayOn_ = true;
  uint8_t desired_[kDigits] = {0, 0, 0, 0};
  uint8_t written_[kDigits] = {0, 0, 0, 0};
  bool writtenValid_[kDigits] = {false, false, false, false};
};

}  // namespace n2
