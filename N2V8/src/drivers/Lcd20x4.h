// Lcd20x4.h — minimal driver for a 20x4 HD44780 LCD behind a PCF8574 I2C backpack (DRV-1..DRV-3).
//
// Non-blocking: the HD44780 start-up waits are deadlines, not delay(). Writes are spread over loop passes
// (at most kCharsPerService characters each) and only cells that differ from what is already on the
// display are written, so one loop pass never spends more than a few milliseconds on the bus.
// A failed transaction is reported and the display is re-initialised when it answers again (DSP-7).
// Starts from the V7 mini-library; pin mapping is the common YwRobot backpack (RS=P0, EN=P2, BL=P3, D4-D7=P4-P7).
#pragma once

#include <stdint.h>

#include "../core/TimedState.h"
#include "../hal/Hal.h"
#include "../ui/LcdScreens.h"

namespace n2 {

struct LcdHealing {
  uint32_t firstRewriteMs = 250;
  uint32_t rewriteEveryMs = 5000;
  uint32_t reinitAfterMs1 = 500;
  uint32_t reinitAfterMs2 = 2000;
  uint32_t reinitEveryMs = 30000;
};

class Lcd20x4 {
 public:
  static constexpr uint8_t kCharsPerService = 6;
  static constexpr uint32_t kRetryMs = 1000;

  Lcd20x4(Hal& hal, uint8_t address) : hal_(hal), address_(address) {}

  void begin(uint32_t now);
  void service(uint32_t now);

  void setScreen(const Screen& screen);  // desired content
  // The display can be corrupted without any I2C error (electrical noise), and the driver only writes cells that differ from
  // its copy of the screen. refresh() forgets that copy so the whole screen is rewritten (no flicker); reinit() restarts the
  // controller from scratch (the display clears and redraws).
  // Bus-integrity test (diagnostic, blocks for about n x 0.6 ms): writes bit patterns to the PCF8574 with EN, RS and RW all low
  // (so the LCD itself ignores them), reads each back, and counts failures. Restores the backlight byte afterwards.
  struct BusTest {
    uint16_t rounds = 0, writeFailed = 0, readFailed = 0, mismatched = 0;
    uint32_t microsPerRound = 0;
    uint8_t firstOut = 0, firstBack = 0;  // the first pair that did not match (0, 0 if none)
  };
  BusTest busTest(uint16_t rounds);
  // Self-healing (bench finding 2026-10-06: after a reset the display sometimes stayed blank or garbled and nothing we wrote showed;
  // the HD44780 cannot be read back through this backpack, so we cannot tell). With healing on, the driver repairs it on a schedule:
  //   * full rewrites of the screen 250 ms after the display first comes up, then at doubling gaps up to every 5 s
  //     (repairs corrupted cells, no flicker),
  //   * a full re-initialisation 0.5 s and 2 s after it first comes up, then every 30 s (repairs a controller stuck in a bad mode;
  //     a brief clear-and-redraw). So a bad display is bad for at most about 30 s.
  using Healing = LcdHealing;
  void enableHealing(const Healing& healing = Healing()) { heal_ = healing; healOn_ = true; }
  void refresh();
  void reinit(uint32_t now);
  void setBacklight(bool on);            // BIST
  void setDisplayOn(bool on);            // BIST: display on/off, content kept

  bool ready() const { return state_ == State::kReady; }
  bool healthy() const { return healthy_; }
  uint32_t i2cErrors() const { return errors_; }
  uint32_t reinitCount() const { return reinits_; }
  bool inSync() const;                   // the display shows exactly the desired screen
  bool backlightOn() const { return backlight_; }
  bool displayOn() const { return displayOn_; }
  const char* shown(uint8_t row) const { return shadow_[row]; }  // what the driver believes is on the display

 private:
  enum class State : uint8_t { kIdle, kInit, kReady, kRetry };

  bool writeNibble(uint8_t nibbleInHighBits);          // init only: one nibble with an EN pulse
  bool writeByte(uint8_t value, bool data);            // command or data, two nibbles in one transaction
  bool writeRaw(uint8_t value);                        // plain byte (backlight only)
  void fail(uint32_t now);
  void startInit(uint32_t now);
  bool serviceInit(uint32_t now);
  void serviceContent(uint32_t now);

  Hal& hal_;
  uint8_t address_;
  State state_ = State::kIdle;
  uint32_t until_ = 0;
  uint8_t initStep_ = 0;
  bool healthy_ = true;
  uint32_t errors_ = 0;
  uint32_t reinits_ = 0;
  bool backlight_ = true;
  bool displayOn_ = true;
  bool backlightDirty_ = false;
  bool displayDirty_ = false;
  char desired_[kLcdRows][kLcdCols + 1];
  char shadow_[kLcdRows][kLcdCols + 1];
  int8_t curRow_ = -1;  // where the display's cursor is, or -1 if unknown
  int8_t curCol_ = -1;
  bool haveDesired_ = false;
  uint32_t stale_[kLcdRows] = {};  // bit c of stale_[r]: cell must be rewritten even if it matches the shadow (refresh())

  void serviceHealing(uint32_t now);
  bool healOn_ = false;
  Healing heal_;
  bool healStarted_ = false;
  uint32_t healReadyAt_ = 0;
  uint32_t rewriteGapMs_ = 0;
  uint8_t healReinits_ = 0;
  Deadline nextRewrite_;
  Deadline nextReinit_;
};

}  // namespace n2
