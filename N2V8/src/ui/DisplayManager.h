// DisplayManager.h — owns the LCD and the LED and decides what they show (Requirements §8, DSP-1..DSP-8).
//
//   normal   : the normal screen, alternating with the fault screens while a fault is active (DSP-5)
//   override : exactly the screen/LED text given (start-up banner, POST, BIST), optionally for a limited time
//
// Never blocks. A missing display never stops anything (DSP-6): it only shows up in lcdHealthy()/ledHealthy(),
// which the caller reports as faults F10/F11.
#pragma once

#include <stdint.h>

#include "../BoardPins.h"
#include "../core/TimedState.h"
#include "../drivers/Lcd20x4.h"
#include "../drivers/Led1650.h"
#include "DisplayData.h"
#include "LcdScreens.h"
#include "LedText.h"
#include "ScreenCycle.h"

namespace n2 {

constexpr uint32_t kDefaultFaultCycleMs = 4000;  // LCD_FAULT_CYCLE_MS (3000..4000)

class DisplayManager {
 public:
  DisplayManager(Hal& hal, const BoardDef& board, LcdLayout layout, uint32_t faultCycleMs = kDefaultFaultCycleMs)
      : layout_(layout), lcd_(hal, board.addrLcd), led_(hal, board.addrLed, board.addrLedDigits), cycle_(faultCycleMs) {}

  void begin(uint32_t now);

  // Normal operation: call once per pass with fresh data.
  void showNormal(const DisplayData& data, uint32_t now);

  // Show exactly this. forMs == 0: until clearOverride().
  void setOverride(const Screen& screen, const LedText& led, uint32_t now, uint32_t forMs = 0);
  void clearOverride() { override_ = false; timer_.clear(); }
  bool overriding() const { return override_; }

  // Drive the two drivers (bounded I2C work per call).
  void service(uint32_t now);

  Lcd20x4& lcd() { return lcd_; }
  Led1650& led() { return led_; }
  bool lcdHealthy() const { return lcd_.healthy(); }
  bool ledHealthy() const { return led_.healthy(); }
  bool showingFaultScreen() const { return !override_ && cycle_.showFault(); }

 private:
  LcdLayout layout_;
  Lcd20x4 lcd_;
  Led1650 led_;
  ScreenCycle cycle_;
  Deadline timer_;
  bool override_ = false;
};

}  // namespace n2
