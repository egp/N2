// App.h — the whole firmware behind two calls, setup() and loop() (Requirements §3, §4).
//
// Everything the sketch does lives here and depends only on the Hal, so the complete firmware (boot, POST, run,
// BIST, console, watchdog) runs on the host under test. The .ino is just glue.
//
//   setup(): outputs safe -> watchdog -> reset cause -> displays -> POST            (RST-2, WDT-1)
//   loop():  watchdog refresh, console, then one of:
//              POST  (hands-off self-test, may hold on a fault)
//              BIST  (operator self-test; started by the `bist` command, or by TOB held at power-up)
//              RUN   (the system: System::step + displays)
#pragma once

#include <stdint.h>

#include "../BoardPins.h"
#include "../ControlConfig.h"
#include "../core/LoopStats.h"
#include "../core/O2Reader.h"
#include "../core/System.h"
#include "../core/WarmCredit.h"
#include "../hal/Hal.h"
#include "../selftest/Bist.h"
#include "../selftest/Post.h"
#include "../ui/BuildInfo.h"
#include "../ui/Commands.h"
#include "../ui/Console.h"
#include "../ui/DisplayManager.h"

namespace n2 {

struct AppOptions {
  bool watchdogEnabled = true;       // WDT-1; a macro in the sketch can turn it off while debugging (WDT-5)
  uint32_t watchdogMs = 4000;        // reduced once real loop times are known (NFR-1)
  LcdLayout layout = LcdLayout::kClearLabels;
  uint32_t faultCycleMs = kDefaultFaultCycleMs;
  bool controllersEnabled = true;    // false in the DIAG build
  bool warmCreditEnabled = false;    // O2-6b: false until the reset probe has proven the credit logic on both boards
  LogLevel logLevel = LogLevel::kDebug;
  PostOptions post;
  BistConfig bist;
};

class App : public CommandLauncher {
 public:
  enum class Mode : uint8_t { kPost, kBist, kRun };

  // `warmRecord` is the RAM record that survives a reset (nullptr: no credit, e.g. on the host).
  App(Hal& hal, const BoardDef& board, const ControlConfig& cfg, O2Reader& o2, const BuildInfo& info,
      WarmRecord* warmRecord = nullptr, AppOptions options = AppOptions());

  void setup();
  void loop();

  Mode mode() const { return mode_; }
  System& system() { return sys_; }
  Console& console() { return console_; }
  DisplayManager& display() { return display_; }
  LoopStats& loopStats() { return loopStats_; }
  Post& post() { return post_; }
  Bist& bist() { return bist_; }
  const ResetInfo& resetInfo() const { return resetInfo_; }

  // CommandLauncher (the console's `post` and `bist` commands): honoured at the start of the next RUN pass.
  const char* requestPost() override;
  const char* requestBist() override;

 private:
  void runPass(uint32_t now);
  void announceConsole();
  void printBanner();
  bool tobPressed();

  Hal& hal_;
  const BoardDef& board_;
  ControlConfig cfg_;
  BuildInfo info_;
  AppOptions opt_;

  Console console_;
  System sys_;
  DisplayManager display_;
  LoopStats loopStats_;
  ConsoleContext ctx_;
  Commands commands_;
  Post post_;
  Bist bist_;
  WarmRecord dummyRecord_ = {};
  WarmCredit credit_;

  Mode mode_ = Mode::kPost;
  ResetInfo resetInfo_;
  bool tobAtBoot_ = false;
  bool postRequested_ = false;
  bool bistRequested_ = false;
  bool consoleWasAttached_ = false;
  uint32_t nextBanner_ = 10000;  // R4 WiFi: repeat the banner until a PC has been heard from
  uint32_t lastDropped_ = 0;
  uint32_t lastCreditTick_ = 0;
  bool o2WasError_ = false;
};

}  // namespace n2
