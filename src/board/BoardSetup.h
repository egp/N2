// BoardSetup.h — table-driven pin setup (Requirements PIN-3, RST-2).
#pragma once

#include "../BoardPins.h"
#include "../hal/Hal.h"

namespace n2 {

// Drive every output to its safe (off) level. Touches output pins only, and
// does it in the order write -> make output -> write, so the pin never shows
// the wrong level while it changes mode. Call as early as possible (RST-2).
void driveOutputsSafe(Hal& hal, const BoardDef& board = kBoard);

// Configure every signal's pin from the table. Outputs are driven safe first.
void configurePins(Hal& hal, const BoardDef& board = kBoard);

// configurePins + ADC resolution (kAdcBits) + I2C bus start. Call from setup().
void beginHardware(Hal& hal, const BoardDef& board = kBoard);

}  // namespace n2
