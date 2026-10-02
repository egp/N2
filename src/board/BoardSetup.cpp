#include "BoardSetup.h"

#include "../Config.h"

namespace n2 {

void driveOutputsSafe(Hal& hal, const BoardDef& board) {
  for (uint8_t i = 0; i < board.signalCount; ++i) {
    const SignalDef& d = board.signals[i];
    if (!isOutput(d)) continue;
    const bool safeHigh = safeLevelHigh(d.active);
    hal.digitalWrite(d.pin, safeHigh);
    hal.pinMode(d.pin, PinMode::kOutput);
    hal.digitalWrite(d.pin, safeHigh);
  }
}

void configurePins(Hal& hal, const BoardDef& board) {
  driveOutputsSafe(hal, board);
  for (uint8_t i = 0; i < board.signalCount; ++i) {
    const SignalDef& d = board.signals[i];
    switch (d.dir) {
      case Dir::kInput:
      case Dir::kAnalogInput:  hal.pinMode(d.pin, PinMode::kInput); break;
      case Dir::kInputPullup:  hal.pinMode(d.pin, PinMode::kInputPullup); break;
      case Dir::kOutput:       break;  // done by driveOutputsSafe
    }
  }
}

void beginHardware(Hal& hal, const BoardDef& board) {
  configurePins(hal, board);
  hal.setAnalogResolution(kAdcBits);
  hal.i2cBegin();
}

}  // namespace n2
