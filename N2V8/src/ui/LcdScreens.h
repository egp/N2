// LcdScreens.h — what the 20x4 LCD should show, as pure text (Requirements DSP-1..DSP-8).
//
// A Screen is four rows of exactly 20 characters. Rendering is a pure function of
// DisplayData, so every screen is a golden test on the host, and the console can print
// the same text for the operator to compare with the real display (DRV-3).
// Layouts are described in docs/LCD_Layouts.md.
#pragma once

#include <stdint.h>

#include "DisplayData.h"

namespace n2 {

constexpr uint8_t kLcdRows = 4;
constexpr uint8_t kLcdCols = 20;

struct Screen {
  char row[kLcdRows][kLcdCols + 1];  // each row: 20 characters plus a terminating NUL
  bool operator==(const Screen& o) const;
};

enum class LcdLayout : uint8_t {
  kClearLabels,  // Option 1: full labels; compressor state one row below the pressures
  kCompact       // Option 2: N2-low, N2-high and compressor state on one line
};

Screen renderNormal(const DisplayData& d, LcdLayout layout);

// The fault screen for the n-th active fault (0-based, wraps), full 20x4 (DSP-5).
Screen renderFault(const DisplayData& d, uint8_t faultIndex);

// Startup banner (DSP-8).
Screen renderBanner(const char* version, const char* board, const char* buildDate);

// A screen from up to four lines of text (each clipped or padded to 20 columns). For POST, BIST and banners.
Screen makeScreen(const char* r0, const char* r1 = "", const char* r2 = "", const char* r3 = "");

// Fixed-point formatting used by the screens, exposed for tests and the console.
void formatX10(char* out, uint16_t value);    // "123.4" (5 chars)
void formatX100(char* out, uint16_t value);   // " 12.34" -> "12.34" (5 chars)
void formatCountdown(char* out, uint32_t ms);  // " 4:32" (5 chars)

}  // namespace n2
