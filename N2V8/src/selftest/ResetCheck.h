// ResetCheck.h — what caused the last reset (Requirements RST-1..RST-3, CON-3).
//
// The cause comes from Hal::readResetCause() (read once at boot, by the application). POST reports it, never holds. The BIST
// asks the operator to confirm it matches what they just did (power cycle / RESET button / watchdog).
#pragma once

#include "../hal/ResetInfo.h"
#include "DeviceCheck.h"

namespace n2 {

class ResetCheck : public DeviceCheck {
 public:
  void setResetInfo(const ResetInfo& info) { info_ = info; }
  static const char* causeText(const ResetInfo& info);

  const char* name() const override { return "RESET"; }
  CheckResult post(uint32_t now) override;
  void bistBegin(uint32_t) override { snprintf(prompt_, sizeof prompt_, "reset cause = %s. Is that what you just did?", causeText(info_)); }
  BistProgress bistTick(uint32_t) override { return BistProgress::kAskOperator; }
  const char* bistPrompt() const override { return prompt_; }
  const char* bistNote() const override { return causeText(info_); }

 private:
  ResetInfo info_;
  char prompt_[64] = {};
};

}  // namespace n2
