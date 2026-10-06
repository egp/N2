// StageCommands.h — the console commands of the bring-up firmware (Requirements §10).
//
//   help | ver | status | post | bist | go | scan | time | time set YYYY-MM-DD HH:MM:SS | log [error|warn|info|debug] | loop
//
// A flat command table: one `case` per command, each builds its answer with TextResponder::add(). The commands act on the
// rest of the firmware only through StageActions, so they can be tested without hardware.
#pragma once

#include "../BoardPins.h"
#include "../core/LoopStats.h"
#include "../core/WallClock.h"
#include "../drivers/Rtc3231.h"
#include "../hal/Hal.h"
#include "../selftest/SelfTest.h"
#include "BuildInfo.h"
#include "Console.h"
#include "TextResponder.h"

namespace n2 {

class StageActions {
 public:
  virtual ~StageActions() = default;
  virtual void startPost() = 0;
  virtual bool startBist() = 0;   // false if it cannot start now
  virtual void releaseHold() = 0;
};

struct StageContext {
  Hal& hal;
  const BoardDef& board;
  BuildInfo info;
  Console& console;
  SelfTest& selfTest;
  Rtc3231& rtc;
  WallClock& wall;
  const LoopStats& loop;
  StageActions& actions;
  const char* resetCause;
  bool (*tbsOn)(void*);
  bool (*tobPressed)(void*);
  void* switchOwner;
};

class StageCommands : public CommandHandler {
 public:
  explicit StageCommands(const StageContext& ctx) : c_(ctx) {}
  Responder* handle(const Command& command) override;

 private:
  void help();
  void status();
  void scan();
  void time(const Command& command);
  void log(const Command& command);
  void loopStats();

  StageContext c_;
  TextResponder out_;
};

}  // namespace n2
