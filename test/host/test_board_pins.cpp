// Tests for BoardPins.h: PIN-1..PIN-10 (table facts, helpers, validation).
#include <array>
#include <catch2/catch_test_macros.hpp>

#include "BoardPins.h"

using namespace n2;

namespace {

// A mutable copy of a board's signal table, so tests can break it on purpose.
struct BrokenBoard {
  std::array<SignalDef, kSignalCount> signals;
  BoardDef board;
  explicit BrokenBoard(const BoardDef& from) : board(from) {
    for (uint8_t i = 0; i < kSignalCount; ++i) signals[i] = from.signals[i];
    board.signals = signals.data();
  }
  SignalDef& at(Signal s) { return signals[static_cast<uint8_t>(s)]; }
};

const BoardDef* const kAllBoards[] = {&kMinimaBoard, &kWifiBoard, &kHostBoard};

}  // namespace

TEST_CASE("PIN-2/PIN-4: every shipped board table passes validation") {
  for (const BoardDef* b : kAllBoards) {
    INFO(b->name);
    CHECK(checkBoard(*b) == BoardCheck::kOk);
  }
}

TEST_CASE("PIN-7: wiring facts carried from V6/V7") {
  const BoardDef& b = kMinimaBoard;
  CHECK(def(b, Signal::kTbs).pin == pin::kD0);
  CHECK(def(b, Signal::kTbs).dir == Dir::kInputPullup);
  CHECK(def(b, Signal::kTbs).active == Active::kLow);
  CHECK(def(b, Signal::kTob).pin == pin::kD1);
  CHECK(def(b, Signal::kTob).active == Active::kLow);
  CHECK(def(b, Signal::kLeftValve).pin == pin::kD4);
  CHECK(def(b, Signal::kRightValve).pin == pin::kD7);
  CHECK(def(b, Signal::kFlushValve).pin == pin::kD11);
  CHECK(def(b, Signal::kSsr).pin == pin::kD8);
  for (Signal s : {Signal::kLeftValve, Signal::kRightValve, Signal::kFlushValve, Signal::kSsr}) {
    CHECK(def(b, s).dir == Dir::kOutput);
    CHECK(def(b, s).active == Active::kHigh);
  }
  CHECK(def(b, Signal::kAirPressure).pin == pin::kA0);
  CHECK(def(b, Signal::kN2LowPressure).pin == pin::kA3);
  CHECK(b.addrLed == 0x24);
  CHECK(b.addrLedDigits == 0x34);  // TM1650 digit registers 0x34..0x37
  CHECK(b.addrLcd == kLcdAddress);
  CHECK(b.addrO2 == 0x74);
  CHECK(b.addrRtc == 0x68);
}

TEST_CASE("PIN-10: N2-high is not on an I2C pin (V6/V7's A5 is SCL)") {
  CHECK(pinOf(kMinimaBoard, Signal::kN2HighPressure) != kMinimaBoard.sclPin);
  CHECK(pinOf(kMinimaBoard, Signal::kN2HighPressure) != kMinimaBoard.sdaPin);
  // The V6/V7 value would be rejected:
  BrokenBoard v7(kMinimaBoard);
  v7.at(Signal::kN2HighPressure).pin = pin::kA5;
  CHECK(checkBoard(v7.board) == BoardCheck::kSignalOnI2cPin);
}

TEST_CASE("PIN-11: I2C bus is A4/A5 on both R4 boards") {
  CHECK(kMinimaBoard.sdaPin == pin::kA4);
  CHECK(kMinimaBoard.sclPin == pin::kA5);
  CHECK(kWifiBoard.sdaPin == pin::kA4);
  CHECK(kWifiBoard.sclPin == pin::kA5);
}

TEST_CASE("PIN-4: validation detects a duplicate pin") {
  BrokenBoard b(kMinimaBoard);
  b.at(Signal::kSsr).pin = b.at(Signal::kLeftValve).pin;
  CHECK(checkBoard(b.board) == BoardCheck::kDuplicatePin);
}

TEST_CASE("PIN-4: validation detects a signal on SDA or SCL") {
  BrokenBoard sda(kMinimaBoard);
  sda.at(Signal::kAirPressure).pin = pin::kA4;
  CHECK(checkBoard(sda.board) == BoardCheck::kSignalOnI2cPin);
}

TEST_CASE("PIN-4: validation detects an analog signal on a digital pin") {
  BrokenBoard b(kMinimaBoard);
  b.at(Signal::kAirPressure).pin = pin::kD2;
  CHECK(checkBoard(b.board) == BoardCheck::kBadPinKind);
}

TEST_CASE("PIN-4: validation detects an out-of-range pin and bad I2C pins") {
  BrokenBoard b(kMinimaBoard);
  b.at(Signal::kSsr).pin = 99;
  CHECK(checkBoard(b.board) == BoardCheck::kBadPin);

  BoardDef i2c = kMinimaBoard;
  i2c.sdaPin = i2c.sclPin;
  CHECK(checkBoard(i2c) == BoardCheck::kBadI2cPins);
}

TEST_CASE("PIN-4: validation detects a duplicate I2C address") {
  BoardDef b = kMinimaBoard;
  b.addrLcd = b.addrLed;
  CHECK(checkBoard(b) == BoardCheck::kDuplicateI2cAddress);
}

TEST_CASE("PIN-4: validation detects an I2C address inside the LED digit range") {
  BoardDef b = kMinimaBoard;
  b.addrO2 = 0x36;  // collides with TM1650 digit 2
  CHECK(checkBoard(b) == BoardCheck::kDuplicateI2cAddress);
}

TEST_CASE("PIN-4: validation detects the RTC sharing an address with another device") {
  BoardDef b = kMinimaBoard;
  b.addrRtc = b.addrLcd;
  CHECK(checkBoard(b) == BoardCheck::kDuplicateI2cAddress);
  b = kMinimaBoard;
  b.addrRtc = 0x35;  // inside the TM1650 digit range
  CHECK(checkBoard(b) == BoardCheck::kDuplicateI2cAddress);
}

TEST_CASE("PIN-6: every digital signal declares an active level") {
  BrokenBoard b(kMinimaBoard);
  b.at(Signal::kSsr).active = Active::kNotApplicable;
  CHECK(checkBoard(b.board) == BoardCheck::kMissingActiveLevel);

  BrokenBoard analogWithLevel(kMinimaBoard);
  analogWithLevel.at(Signal::kAirPressure).active = Active::kHigh;
  CHECK(checkBoard(analogWithLevel.board) == BoardCheck::kMissingActiveLevel);
}

TEST_CASE("PIN-4: the signal count must match the Signal enum") {
  BoardDef b = kMinimaBoard;
  b.signalCount = kSignalCount - 1;
  CHECK(checkBoard(b) == BoardCheck::kWrongSignalCount);
}

TEST_CASE("PIN-6: logical on/off <-> physical level for active-high, active-low, n/a") {
  // active HIGH: on = HIGH
  CHECK(levelHigh(true, Active::kHigh) == true);
  CHECK(levelHigh(false, Active::kHigh) == false);
  CHECK(isOn(true, Active::kHigh) == true);
  CHECK(isOn(false, Active::kHigh) == false);
  // active LOW: on = LOW
  CHECK(levelHigh(true, Active::kLow) == false);
  CHECK(levelHigh(false, Active::kLow) == true);
  CHECK(isOn(false, Active::kLow) == true);
  CHECK(isOn(true, Active::kLow) == false);
  // safe level = the "off" level
  CHECK(safeLevelHigh(Active::kHigh) == false);
  CHECK(safeLevelHigh(Active::kLow) == true);
  // round trip
  for (Active a : {Active::kHigh, Active::kLow})
    for (bool on : {false, true}) CHECK(isOn(levelHigh(on, a), a) == on);
}

TEST_CASE("PIN-8: D0/D1 are TBS/TOB, so Serial1 must not be used") {
  CHECK(pinOf(kMinimaBoard, Signal::kTbs) == 0);
  CHECK(pinOf(kMinimaBoard, Signal::kTob) == 1);
}

TEST_CASE("PIN-2: each board has its own signal table, so one can change without the other") {
  CHECK(kMinimaBoard.signals != kWifiBoard.signals);
  CHECK(kMinimaBoard.signals != kWifiBoard.signals + 0);
  CHECK(checkBoard(kMinimaBoard) == BoardCheck::kOk);
  CHECK(checkBoard(kWifiBoard) == BoardCheck::kOk);
  CHECK(kHostBoard.signals == kMinimaBoard.signals);  // the host tests run against the production table
  // Production safety: the switches are active LOW and the outputs are active HIGH on the Minima.
  CHECK(def(kMinimaBoard, Signal::kTbs).active == Active::kLow);
  CHECK(def(kMinimaBoard, Signal::kTob).active == Active::kLow);
  CHECK(def(kMinimaBoard, Signal::kSsr).active == Active::kHigh);
}

TEST_CASE("PIN-4: the TM1650 answers 0x24-0x27 and 0x34-0x37, so an LCD at 0x27 overlaps the LED module (known conflict), 0x23 does not") {
  BoardDef b = kWifiBoard;
  b.addrLcd = 0x27;  // the PCF8574 default, whatever kLcdAddress is set to today
  CHECK(inTm1650Range(b, 0x24));
  CHECK(inTm1650Range(b, 0x27));
  CHECK(inTm1650Range(b, 0x34));
  CHECK(inTm1650Range(b, 0x37));
  CHECK_FALSE(inTm1650Range(b, 0x23));
  CHECK_FALSE(inTm1650Range(b, 0x38));
  CHECK(lcdOverlapsLed(b));  // today's default 0x27: the conflict is recorded
  b.addrLcd = 0x23;
  CHECK_FALSE(lcdOverlapsLed(b));
  CHECK(checkBoard(b) == BoardCheck::kOk);
}

TEST_CASE("PIN-4: a board may give the LED its own two-wire bus; the pins are validated and the LED then cannot clash with the LCD") {
  BoardDef b = kWifiBoard;
  b.addrLcd = 0x27;
  CHECK(lcdOverlapsLed(b));
  b.ledSdaPin = pin::kD2;
  b.ledSclPin = pin::kD3;
  CHECK(ledOnSoftBus(b));
  CHECK(checkBoard(b) == BoardCheck::kOk);
  CHECK_FALSE(lcdOverlapsLed(b));
  CHECK_FALSE(inTm1650Range(b, 0x27));
  b.ledSclPin = pin::kNoPin;  // only one given
  CHECK(checkBoard(b) == BoardCheck::kBadLedBusPins);
  b.ledSclPin = b.ledSdaPin;  // equal
  CHECK(checkBoard(b) == BoardCheck::kBadLedBusPins);
  b.ledSclPin = b.sclPin;     // on the hardware I2C pins
  CHECK(checkBoard(b) == BoardCheck::kBadLedBusPins);
  b.ledSclPin = pinOf(b, Signal::kSsr);  // used by a signal (D8)
  CHECK(checkBoard(b) == BoardCheck::kBadLedBusPins);
  b.ledSdaPin = 99;
  b.ledSclPin = pin::kD3;
  CHECK(checkBoard(b) == BoardCheck::kBadLedBusPins);
}

TEST_CASE("PIN-1: the permanent LCD address is 0x23 (A2 bridged), outside the TM1650's range, on every board") {
  CHECK(kLcdAddress == 0x23);
  CHECK(kMinimaBoard.addrLcd == 0x23);
  CHECK(kWifiBoard.addrLcd == 0x23);
  CHECK(kHostBoard.addrLcd == 0x23);
  CHECK_FALSE(lcdOverlapsLed(kMinimaBoard));
  CHECK_FALSE(lcdOverlapsLed(kWifiBoard));
  CHECK_FALSE(lcdOverlapsLed(kHostBoard));
}
