// BuildInfo.h — which firmware is this (Requirements ID-1, ENV-5)?
//
// Printed at boot, by `ver`, and at the top of every report so a returned log can be traced to its build.
#pragma once

#include <stdint.h>

namespace n2 {

struct BuildInfo {
  const char* version;
  const char* date;   // __DATE__ of the sketch
  const char* time;   // __TIME__ of the sketch
  const char* board;
  const char* mode;   // HOST / BENCH / DIAG / FIELD
  uint8_t adcBits;
};

// date/time are passed in from the sketch (so they change whenever the sketch is rebuilt).
BuildInfo makeBuildInfo(const char* date, const char* time);

}  // namespace n2
