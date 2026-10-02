// Tests for BoardSetup: PIN-3 (table-driven setup), RST-2 (outputs safe), INP-2 (ADC bits).
#include <catch2/catch_test_macros.hpp>

#include "BoardPins.h"
#include "Config.h"
#include "FakeHal.h"
#include "board/BoardSetup.h"

using namespace n2;

namespace {
// A board whose outputs are active LOW, to prove the code honours the table.
struct ActiveLowBoard {
  std::array<SignalDef, kSignalCount> signals;
  BoardDef board;
  ActiveLowBoard() : board(kHostBoard) {
    for (uint8_t i = 0; i < kSignalCount; ++i) {
      signals[i] = kHostBoard.signals[i];
      if (signals[i].dir == Dir::kOutput) signals[i].active = Active::kLow;
    }
    board.signals = signals.data();
  }
};
}  // namespace

TEST_CASE("RST-2: driveOutputsSafe puts every output at its off level") {
  FakeHal hal;
  driveOutputsSafe(hal, kHostBoard);
  for (uint8_t i = 0; i < kHostBoard.signalCount; ++i) {
    const SignalDef& d = kHostBoard.signals[i];
    if (!isOutput(d)) continue;
    INFO(d.name);
    CHECK(hal.mode[d.pin] == static_cast<int>(PinMode::kOutput));
    CHECK(hal.level[d.pin] == (safeLevelHigh(d.active) ? 1 : 0));
  }
}

TEST_CASE("RST-2: driveOutputsSafe touches output pins only, writing before and after the mode change") {
  FakeHal hal;
  driveOutputsSafe(hal, kHostBoard);
  for (const auto& e : hal.events) {
    bool isOut = false;
    for (uint8_t i = 0; i < kHostBoard.signalCount; ++i)
      if (kHostBoard.signals[i].pin == e.pin && isOutput(kHostBoard.signals[i])) isOut = true;
    CHECK(isOut);
  }
  // Four outputs, three events each: write, mode, write.
  REQUIRE(hal.events.size() == 12);
  for (size_t i = 0; i < hal.events.size(); i += 3) {
    CHECK(hal.events[i].kind == FakeHal::Kind::kWrite);
    CHECK(hal.events[i + 1].kind == FakeHal::Kind::kPinMode);
    CHECK(hal.events[i + 2].kind == FakeHal::Kind::kWrite);
    CHECK(hal.events[i].value == hal.events[i + 2].value);
  }
}

TEST_CASE("PIN-6/RST-2: active-LOW outputs are driven HIGH when off") {
  ActiveLowBoard alb;
  FakeHal hal;
  driveOutputsSafe(hal, alb.board);
  for (Signal s : {Signal::kLeftValve, Signal::kRightValve, Signal::kFlushValve, Signal::kSsr})
    CHECK(hal.level[pinOf(alb.board, s)] == 1);
}

TEST_CASE("PIN-3: configurePins sets each pin's mode from the table") {
  FakeHal hal;
  configurePins(hal, kHostBoard);
  for (uint8_t i = 0; i < kHostBoard.signalCount; ++i) {
    const SignalDef& d = kHostBoard.signals[i];
    INFO(d.name);
    switch (d.dir) {
      case Dir::kInputPullup: CHECK(hal.mode[d.pin] == static_cast<int>(PinMode::kInputPullup)); break;
      case Dir::kInput:
      case Dir::kAnalogInput: CHECK(hal.mode[d.pin] == static_cast<int>(PinMode::kInput)); break;
      case Dir::kOutput: CHECK(hal.mode[d.pin] == static_cast<int>(PinMode::kOutput)); break;
    }
  }
}

TEST_CASE("RST-2: configurePins drives outputs safe before configuring any input") {
  FakeHal hal;
  configurePins(hal, kHostBoard);
  bool sawInputConfig = false;
  for (const auto& e : hal.events) {
    bool isOut = false;
    for (uint8_t i = 0; i < kHostBoard.signalCount; ++i)
      if (kHostBoard.signals[i].pin == e.pin && isOutput(kHostBoard.signals[i])) isOut = true;
    if (!isOut) sawInputConfig = true;
    if (isOut) CHECK_FALSE(sawInputConfig);  // no output event after an input event
  }
  CHECK(sawInputConfig);
}

TEST_CASE("INP-2: beginHardware sets the ADC resolution to kAdcBits and starts I2C") {
  FakeHal hal;
  beginHardware(hal, kHostBoard);
  CHECK(hal.adcBits == kAdcBits);
  CHECK(kAdcBits == 10);  // initial value (0x0A)
  CHECK(hal.i2cStarted);
}
