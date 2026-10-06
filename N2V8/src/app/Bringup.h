// Bringup.h — the staged bring-up firmware (docs/Bringup_Stages.md).
//
// Instead of debugging the whole N2V8 at once, each stage contains only the devices already proven on the bench, and every new
// device is added with its own DeviceCheck (POST + BIST), its own solo test, and then joins the list here.
//
//   Stage 1: reset cause, 20x4 LCD, DS3231 RTC, console, and (WiFi board only) the 12x8 LED matrix.
//
// Responsibilities, one per member:  Console (talk to the PC)  ·  Lcd20x4 (show)  ·  Rtc3231 + WallClock (log time stamps only)
//   SelfTest + DeviceChecks (POST/BIST)  ·  StageCommands (what the PC can ask)  ·  FrameSink (matrix).
// No control logic and no outputs live here. Nothing waits: setup() starts things and loop() takes one short step of each.
#pragma once

#include <stdint.h>

#include "../BoardPins.h"
#include "../core/LoopStats.h"
#include "../core/WallClock.h"
#include "../drivers/Lcd20x4.h"
#include "../drivers/Rtc3231.h"
#include "../hal/Hal.h"
#include "../selftest/LcdCheck.h"
#include "../selftest/ResetCheck.h"
#include "../selftest/RtcCheck.h"
#include "../selftest/SelfTest.h"
#include "../ui/BuildInfo.h"
#include "../ui/Console.h"
#include "../ui/MatrixFrame.h"
#include "../ui/StageCommands.h"

namespace n2 {

struct BringupOptions {
  LogLevel logLevel = LogLevel::kInfo;
  bool watchdogEnabled = true;
  uint32_t watchdogMs = 4000;        // WDT-1: refreshed at the end of every loop pass
  uint32_t rtcResyncMs = 60000;      // how often the log clock re-anchors to the RTC (RTC-7)
  uint32_t bannerRepeatMs = 3000;    // WiFi board: repeat the banner until the PC has been heard
  uint32_t screenMs = 250;           // LCD and matrix refresh period
};

class Bringup : public StageActions {
 public:
  Bringup(Hal& hal, const BoardDef& board, const BuildInfo& info, FrameSink* matrix, BringupOptions options = BringupOptions());

  void setup();
  void loop();

  // StageActions
  void startPost() override;
  bool startBist() override;
  void releaseHold() override { goRequested_ = true; }

  // for tests
  SelfTest& selfTest() { return selfTest_; }
  const WallClock& wall() const { return wall_; }
  Lcd20x4& lcd() { return lcd_; }
  const LoopStats& loopStats() const { return loopStats_; }
  bool switchTbs() { return tbsOn(); }
  bool switchTob() { return tobPressed(); }
  bool running() const { return selfTest_.postFinished() && !selfTest_.bistRunning(); }

 private:
  void syncWallClock(uint32_t now);
  void printBanner();
  void updateScreens(uint32_t now);
  bool tobPressed();
  bool tbsOn();
  void logSwitchChanges();

  Hal& hal_;
  const BoardDef& board_;
  BuildInfo info_;
  FrameSink* matrix_;
  BringupOptions opt_;

  Console console_;
  WallClock wall_;
  StampedLog log_;      // every log line goes through here to get its time stamp
  Lcd20x4 lcd_;
  Rtc3231 rtc_;
  ResetCheck resetCheck_;
  LcdCheck lcdCheck_;
  RtcCheck rtcCheck_;
  DeviceCheck* checks_[3];
  SelfTest selfTest_;
  LoopStats loopStats_;
  StageCommands commands_;
  ResetInfo reset_;

  Deadline rtcResync_;
  Deadline bannerRepeat_;
  Deadline screenRefresh_;
  bool goRequested_ = false;
  bool wasAttached_ = false;
  bool tbsWas_ = false;
  bool tobWas_ = false;
  bool heartbeat_ = false;
  uint32_t bootMs_ = 0;
  char resetText_[24] = {};
};

}  // namespace n2
