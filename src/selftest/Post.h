// Post.h — power-on self-test (Requirements §11, POST-1..POST-6).
//
// Hands-off and quick. Runs at every boot with the controllers disabled and the outputs off. It never needs a
// console. It never waits for anyone UNLESS a real fault is found; then it holds, showing the fault on the LCD,
// until the operator presses TOB (POST-4). Held or not, the plant stays protected by the invariants.
//
// Non-blocking: call step() once per loop pass (and refresh the watchdog); it returns true when finished.
#pragma once

#include <stdint.h>

#include "../core/Faults.h"
#include "../core/Log.h"
#include "../core/Sensors.h"
#include "../core/System.h"
#include "../core/TimedState.h"
#include "../hal/Hal.h"
#include "../ui/BuildInfo.h"
#include "../ui/DisplayManager.h"

namespace n2 {

enum class PostLevel : uint8_t { kPass, kWarn, kFail };

struct PostOptions {
  Severity hangAt = Severity::kWarn;  // POST_HANG_SEVERITY: a fault at or above this holds POST until TOB
  uint32_t bannerMs = 1000;           // start-up banner (DSP-8)
  uint32_t okMs = 1000;               // "POST OK" screen when clean (POST-5)
  uint32_t problemMs = 5000;          // result screen when not clean
  uint32_t holdRotateMs = 3000;       // while held: step through the faults
};

class Post {
 public:
  Post(Hal& hal, const BoardDef& board, System& sys, DisplayManager& display, LogSink& log, const BuildInfo& info,
       const ResetInfo& reset, PostOptions options = PostOptions())
      : hal_(hal), board_(board), sys_(sys), display_(display), log_(log), info_(info), reset_(reset), opt_(options) {}

  void begin(uint32_t now);
  bool step(uint32_t now);  // true once POST is finished and normal operation may start

  PostLevel level() const { return level_; }
  uint8_t problemCount() const { return problems_; }
  bool holding() const { return phase_ == Phase::kHold; }
  bool finished() const { return phase_ == Phase::kDone; }

  static constexpr uint8_t kCheckCount = 7;

 private:
  enum class Phase : uint8_t { kChecks, kSummary, kHold, kDone };

  void runCheck(uint32_t now);
  void finishChecks(uint32_t now);
  bool shouldHold() const;
  void showResult(uint32_t now, uint8_t firstFault, bool holdPrompt);
  bool tobPressed();
  void say(uint32_t now, const char* check, const char* result);

  Hal& hal_;
  const BoardDef& board_;
  System& sys_;
  DisplayManager& display_;
  LogSink& log_;
  BuildInfo info_;
  ResetInfo reset_;
  PostOptions opt_;

  Phase phase_ = Phase::kDone;
  uint32_t startedAt_ = 0;
  uint8_t check_ = 0;
  uint8_t samples_ = 0;
  SensorChannel air_, n2Low_, n2High_;
  Deadline summaryUntil_;
  Deadline rotate_;
  uint8_t rotation_ = 0;
  bool tobWasUp_ = false;
  PostLevel level_ = PostLevel::kPass;
  uint8_t problems_ = 0;
};

}  // namespace n2
