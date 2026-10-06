// DeviceCheck.h — one device's self-test: its POST check and its BIST step live together (Requirements §11, §12).
//
// A device (LCD, RTC, ...) is added to the firmware by writing one class like this and putting it in the list that
// SelfTest runs. POST and BIST then pick it up with no other change. Nothing here touches Arduino headers: a check talks
// to its device only through a driver, and the driver talks to the Hal, so every check runs on the host against FakeHal.
//
//   post()       quick and hands-off; called once per loop pass at power-up; never waits for anyone.
//   bistBegin()  start the operator-confirmed step; bistTick() is then called every loop pass.
//                bistTick() answers one of: still running / ask the operator / pass / fail.
//   bistEnd()    leave the device as normal operation expects it (called after every step, even a quit).
#pragma once

#include <stdint.h>
#include <stdio.h>

namespace n2 {

enum class CheckLevel : uint8_t {
  kNone,  // not run yet
  kPass,
  kInfo,  // worth knowing, never holds POST (for example an optional device is absent)
  kFail   // holds POST until the operator releases it
};

struct CheckResult {
  CheckLevel level = CheckLevel::kNone;
  char text[44] = {};

  void set(CheckLevel l, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
};

enum class BistProgress : uint8_t { kRunning, kAskOperator, kPass, kFail };

class DeviceCheck {
 public:
  virtual ~DeviceCheck() = default;

  virtual const char* name() const = 0;  // short, upper case, at most 8 characters: "LCD", "RTC"

  virtual CheckResult post(uint32_t now) = 0;

  virtual void bistBegin(uint32_t now) = 0;
  virtual BistProgress bistTick(uint32_t now) = 0;
  // One line telling the operator what to look at (shown when bistTick() answers kAskOperator).
  virtual const char* bistPrompt() const { return ""; }
  // Short result text kept in the BIST summary (for example the measured value).
  virtual const char* bistNote() const { return ""; }
  virtual void bistEnd() {}
};

}  // namespace n2
