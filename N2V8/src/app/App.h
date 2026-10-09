// App.h — the whole firmware behind two calls, setup() and loop() (Requirements §3, §4).
//
// Everything the sketch does lives here and depends only on the Hal, so the complete firmware (boot, POST, run,
// BIST, console, watchdog) runs on the host under test. The .ino is just glue.
//
//   setup(): outputs safe -> watchdog -> reset cause -> displays; then RUN, or POST if TOB is held (RST-2, WDT-1, POST-1)
//   loop():  watchdog refresh, console, then one of:
//              POST  (hands-off self-test, only when TOB was held at power-up/reset or on the `post` command; may hold on a fault)
//              BIST  (operator self-test; started ONLY by the `bist` command)
//              RUN   (the system: System::step + displays)
#pragma once

#include <stdint.h>

#include "../BoardPins.h"
#include "../ControlConfig.h"
#include "../core/LoopStats.h"
#include "../core/NvmSettingsService.h"
#include "../core/O2Reader.h"
#include "../core/System.h"
#include "../core/WarmCredit.h"
#include "RunMode.h"
#include "../drivers/Rtc3231.h"
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
  uint32_t lcdStartMs = kDefaultLcdStartMs;  // the LCD is left alone this long after boot (see DisplayManager::begin)
  uint32_t faultCycleMs = kDefaultFaultCycleMs;
  uint32_t lcdMinChangeMs = kDefaultLcdMinChangeMs;  // normal LCD screen changes at most this often (1 Hz); 0 = every pass
  bool controllersEnabled = true;    // false in the DIAG build
  bool warmCreditEnabled = false;    // O2-6b: false until the reset probe has proven the credit logic on both boards
  LogLevel logLevel = LogLevel::kDebug;
  ModeRecord* modeRecord = nullptr;  // RAM breadcrumb: the run mode survives a reset-button or watchdog reset (nullptr: always the compiled mode)
  BistRecord* bistRecord = nullptr;  // RAM record that survives a reset-button reset: lets an interrupted BIST resume (nullptr: no resume)
  Nvm* nvm = nullptr;                // non-volatile memory (data flash); nullptr = none: the compiled debounce default is used
  uint16_t sketchVersion = 0;        // BuildConfig.h N2_VERSION_HEX: stored in every NVM record
  uint8_t debounceDefaultMs = kBringupDebounceMs;  // compiled default; 0 = no debounce (host tests of the controllers)
  PostOptions post;
  BistConfig bist;
};

class App : public CommandLauncher {
 public:
  enum class Mode : uint8_t { kPost, kBist, kRun, kPostArm };   // kPostArm: TOB was held at start; waiting for the operator to release it

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
  const char* requestMode(const char* name, bool confirmed) override;
  const char* requestRec(const char* arg0, const char* arg1) override;
  const char* requestCap(const char* note, bool sample) override;
  RunMode runMode() const { return runMode_; }

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
  Rtc3231 rtc_;
  System sys_;
  DisplayManager display_;
  LoopStats loopStats_;
  NvmSettingsService nvmSvc_;
  ConsoleContext ctx_;
  Commands commands_;
  Post post_;
  Bist bist_;
  WarmRecord dummyRecord_ = {};
  WarmCredit credit_;

  Mode mode_ = Mode::kPost;
  ResetInfo resetInfo_;
  void applyMode(RunMode m);
  void recordTick(uint32_t now);
  void printSample(uint32_t now, const char* why);
  void autoCaptureTick(uint32_t now);
  bool autoInit_ = false, prevSsr_ = false, prevTbs_ = false, prevTob_ = false;
  bool recOn_ = false;
  uint32_t recEveryMs_ = 1000;
  uint32_t recNext_ = 0;
  RunMode runMode_ = RunMode::kDiag;
  bool tobAtBoot_ = false;
  uint32_t postArmStart_ = 0;
  uint32_t tobReleasedAt_ = 0;
  bool tobReleased_ = false;
  bool postRequested_ = false;
  bool bistRequested_ = false;
  bool consoleWasAttached_ = false;
  bool rtcFitted_ = false;       // an RTC answered at boot (or has appeared since); a panel without one is a valid configuration (owner 2026-10-09)
  uint32_t nextRtcCheck_ = 0;    // the RTC is checked every 10 s while running (F13)
  uint32_t nextBanner_ = 10000;  // R4 WiFi: repeat the banner until a PC has been heard from
  uint32_t lastDropped_ = 0;
  uint32_t lastCreditTick_ = 0;
  bool o2WasError_ = false;
};

}  // namespace n2
