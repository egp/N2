#include "LedText.h"

#include <stdio.h>
#include <string.h>

namespace n2 {

bool LedText::operator==(const LedText& o) const { return memcmp(digit, o.digit, 4) == 0 && dotAfter == o.dotAfter; }

LedText renderLed(const DisplayData& d, bool showFault) {
  LedText t;
  memcpy(t.digit, "    ", 5);
  t.dotAfter = -1;
  if (showFault && d.faultCount > 0) {
    snprintf(t.digit, sizeof t.digit, " F%02u", static_cast<unsigned>(faultInfo(d.faults[0]).code % 100u));
    return t;
  }
  if (!d.tbs) return t;
  t.dotAfter = 1;
  if (d.n2Valid && !d.warming) {
    snprintf(t.digit, sizeof t.digit, "%02u%02u", static_cast<unsigned>(d.n2PercentX100 / 100u % 100u),
             static_cast<unsigned>(d.n2PercentX100 % 100u));
  } else {
    memcpy(t.digit, "----", 5);
  }
  return t;
}

}  // namespace n2
