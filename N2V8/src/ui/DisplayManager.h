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

constexpr uint32_t kDefaultLcdStartMs = 2500;    // see DisplayManager::begin
constexpr uint32_t kDefaultLcdMinChangeMs = 1000;  // the normal LCD screen changes at most once a second (no flicker from a wandering last digit)
constexpr uint32_t kDefaultFaultCycleMs = 4000;  // LCD_FAULT_CYCLE_MS (3000..4000)

class DisplayManager {
 public:
  DisplayManager(Hal& hal, const BoardDef& board, LcdLayout layout, uint32_t faultCycleMs = kDefaultFaultCycleMs)
      : layout_(layout), lcd_(hal, board.addrLcd), led_(hal, board.addrLed, board.addrLedDigits), cycle_(faultCycleMs) {}

  // Output "debounce": a changed NORMAL screen is sent to the LCD at most once every minMs (0 = at once). Overrides (start-up, POST, BIST)
  // are never held back. The LED is not limited.
  void setLcdMinChangeMs(uint32_t minMs) { lcdMinChangeMs_ = minMs; }

  // The LCD is not touched until lcdStartMs after boot (bench finding 2026-10-06: initialising it within the first ~2 s after a
  // reset left it showing random characters; waiting 2.5 s gave 5 clean resets in 5). The LED has no such problem.
  void begin(uint32_t now, uint32_t lcdStartMs = kDefaultLcdStartMs);

  // Normal operation: call once per pass with fresh data.
  void showNormal(const DisplayData& data, uint32_t now);

  // Show exactly this. forMs == 0: until clearOverride().
  void setOverride(const Screen& screen, const LedText& led, uint32_t now, uint32_t forMs = 0);
  void clearOverride() { override_ = false; timer_.clear(); }
  // Keep this text on the LED (only) for `ms`, or until TBS is turned ON, whichever is first; the LCD goes back to its normal screen at once.
  // Used for the POST result (0000 = good) after POST has ended and the system is already running.
  void holdLed(const LedText& text, uint32_t now, uint32_t ms) { ledHoldText_ = text; ledHoldUntil_.arm(now, ms); ledHold_ = true; }
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
  Deadline lcdStart_;
  bool override_ = false;
  bool ledHold_ = false;
  LedText ledHoldText_{{' ', ' ', ' ', ' ', '\0'}, -1};
  Deadline ledHoldUntil_;
  uint32_t lcdMinChangeMs_ = 0;
  Screen shownNormal_;           // the last normal screen given to the LCD
  uint32_t shownNormalAt_ = 0;
  bool haveShownNormal_ = false;
};

}  // namespace n2
