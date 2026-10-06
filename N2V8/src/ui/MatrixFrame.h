// MatrixFrame.h — pictures for the UNO R4 WiFi's 12x8 LED matrix, drawn as pure functions (Requirements DSP-10).
//
// The matrix is a spare output channel on the WiFi bench board only (the Minima has none), so nothing may depend on it.
// A frame is 96 bits, row-major, most significant bit first, in three 32-bit words: the layout ArduinoLEDMatrix::loadFrame takes.
// The sketch glue implements FrameSink with the real matrix; the tests use a recording one.
//
// What the bring-up firmware draws (also in the sketch's README):
//   left glyph   POST/BIST: the number of the check being run (hex, from 1).   Normal running: overall POST result.
//   right glyph  that check's result:  0 = no result yet   1 = pass   2 = info (noted, not a fault)   F = fail
//   bottom row   pixels 0..7 = one per check, lit when that check passed or noted info;  pixel 11 = heartbeat (1 Hz blink)
#pragma once

#include <stdint.h>

#include "../selftest/DeviceCheck.h"

namespace n2 {

class FrameSink {
 public:
  virtual ~FrameSink() = default;
  virtual void show(const uint32_t* frame) = 0;  // 3 words
};

constexpr uint8_t kMatrixRows = 8;
constexpr uint8_t kMatrixCols = 12;

void matrixSetPixel(uint32_t* frame, uint8_t row, uint8_t col);
bool matrixGetPixel(const uint32_t* frame, uint8_t row, uint8_t col);
void matrixDrawHex(uint32_t* frame, uint8_t value, uint8_t leftColumn);  // 5x7 glyph for 0..F

// The glyph value for a check result: 0 none, 1 pass, 2 info, 0xF fail.
uint8_t matrixGlyphForLevel(CheckLevel level);

// levels[i] = result of check i (up to 8).
void matrixDrawStatus(uint32_t* frame, uint8_t left, uint8_t right, const CheckLevel* levels, uint8_t count, bool heartbeat);

}  // namespace n2
