// LedText.h — what the 4-digit LED should show (Requirements DSP-3, DSP-5).
#pragma once

#include <stdint.h>

#include "DisplayData.h"

namespace n2 {

struct LedText {
  char digit[5];     // four characters, NUL-terminated: '0'-'9', '-', 'F', ' '
  int8_t dotAfter;   // decimal point after this digit (0..3), or -1 for none
  bool operator==(const LedText& o) const;
};

// TBS off: blank. TBS on: N2% as "nn.nn" (dot after digit 1), "--.--" if invalid.
// A fault of severity >= WARN is shown as " Fnn" for the first active fault when `showFault` is set.
LedText renderLed(const DisplayData& d, bool showFault);

}  // namespace n2
