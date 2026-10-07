// Led1650.h — minimal driver for the TM1650 4-digit, 8-segment LED display (Requirements §3 DRV-1..DRV-3).
//
// Non-blocking: begin() starts a power-up wait, service() does the I2C writes. Only digits whose content
// changed are written. A failed transaction is reported (healthy() false, errors counted) and the driver
// re-initialises itself when the device answers again (DSP-7). Starts from the V7 mini-library.
#pragma once

#include <stdint.h>

#include "../core/TimedState.h"
#include "../hal/Hal.h"
#include "../hal/I2cBus.h"
#include "../ui/LedText.h"

namespace n2 {

class Led1650 {
 public:
  static constexpr uint8_t kDigits = 4;
  static constexpr uint32_t kPowerUpMs = 50;
  static constexpr uint32_t kRetryMs = 1000;
  static constexpr uint8_t kControlOn = 0x11;   // brightness 1, 8-segment mode, display on
  static constexpr uint8_t kControlOff = 0x10;  // same, display off

  // `bus` selects where the LED sits: nullptr = the hardware I2C bus through the Hal; or a SoftI2c on its own two pins (the TM1650 answers 0x25-0x27
  // too, which clashes with an LCD backpack at 0x27: see SoftI2c.h).
  Led1650(Hal& hal, uint8_t controlAddress, uint8_t digitBaseAddress, I2cBus* bus = nullptr)
      : halBus_(hal), bus_(bus != nullptr ? bus : &halBus_), control_(controlAddress), digitBase_(digitBaseAddress) {}

  void begin(uint32_t now);
  void service(uint32_t now);

  void setText(const LedText& text);  // desired content; written by service()
  // Rewrite the control byte and all four digits every periodMs (0 = only on change, the default). A digit that does not change (an hour
  // digit) would otherwise never be written, so a missing or wedged module would go unnoticed for as long as the text stays the same.
  void setRefresh(uint32_t periodMs) { refreshMs_ = periodMs; }
  void setDisplayOn(bool on);         // BIST: display on/off without changing the content

  bool ready() const { return state_ == State::kReady; }
  bool healthy() const { return healthy_; }
  uint32_t i2cErrors() const { return errors_; }
  uint8_t answeringAddresses();  // how many of the 5 addresses (control + 4 digits) acknowledge now: 5 = all
  bool inSync() const;  // every desired digit has been written
  uint8_t shownSegments(uint8_t digit) const { return written_[digit]; }  // last segments written (diagnostics, tests)
  bool displayOn() const { return displayOn_; }

  // Segment bits for a character: '0'-'9', 'A'-'F', '-', ' '. Unknown characters show blank.
  static uint8_t segmentsFor(char c);

 private:
  enum class State : uint8_t { kIdle, kPowerWait, kReady, kRetry };
  void fail(uint32_t now);

  HalI2cBus halBus_;
  I2cBus* bus_;
  uint8_t control_;
  uint8_t digitBase_;
  State state_ = State::kIdle;
  uint32_t until_ = 0;
  bool healthy_ = true;
  uint32_t errors_ = 0;
  bool controlDirty_ = true;
  uint32_t refreshMs_ = 0;
  Deadline refresh_;
  bool displayOn_ = true;
  uint8_t desired_[kDigits] = {0, 0, 0, 0};
  uint8_t written_[kDigits] = {0, 0, 0, 0};
  bool writtenValid_[kDigits] = {false, false, false, false};
};

}  // namespace n2
