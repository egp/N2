#include "MatrixFrame.h"

namespace n2 {

namespace {
// 5x7 hex glyphs, one byte per row, bit 4 = leftmost column.
const uint8_t kHex[16][7] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}};
}  // namespace

void matrixSetPixel(uint32_t* frame, uint8_t row, uint8_t col) {
  if (row >= kMatrixRows || col >= kMatrixCols) return;
  const uint8_t index = static_cast<uint8_t>(row * kMatrixCols + col);
  frame[index / 32] |= 1UL << (31 - (index % 32));
}

bool matrixGetPixel(const uint32_t* frame, uint8_t row, uint8_t col) {
  if (row >= kMatrixRows || col >= kMatrixCols) return false;
  const uint8_t index = static_cast<uint8_t>(row * kMatrixCols + col);
  return (frame[index / 32] >> (31 - (index % 32))) & 1UL;
}

void matrixDrawHex(uint32_t* frame, uint8_t value, uint8_t leftColumn) {
  for (uint8_t row = 0; row < 7; ++row)
    for (uint8_t bit = 0; bit < 5; ++bit)
      if (kHex[value & 15][row] & (0x10 >> bit)) matrixSetPixel(frame, row, static_cast<uint8_t>(leftColumn + bit));
}

uint8_t matrixGlyphForLevel(CheckLevel level) {
  switch (level) {
    case CheckLevel::kPass: return 1;
    case CheckLevel::kInfo: return 2;
    case CheckLevel::kFail: return 0xF;
    default: return 0;
  }
}

void matrixDrawStatus(uint32_t* frame, uint8_t left, uint8_t right, const CheckLevel* levels, uint8_t count, bool heartbeat) {
  frame[0] = frame[1] = frame[2] = 0;
  matrixDrawHex(frame, left, 0);
  matrixDrawHex(frame, right, 7);
  for (uint8_t i = 0; i < count && i < 8; ++i)
    if (levels[i] == CheckLevel::kPass || levels[i] == CheckLevel::kInfo) matrixSetPixel(frame, 7, i);
  if (heartbeat) matrixSetPixel(frame, 7, 11);
}

}  // namespace n2
