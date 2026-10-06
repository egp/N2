// SelfTest.h — runs POST and BIST over a list of DeviceChecks (Requirements §11 POST, §12 BIST).
//
//   POST  hands-off. Runs every check once, one per loop pass. Holds ONLY if a check fails (kFail); the operator releases
//         the hold with TOB (or the console command `go`). Info results never hold.
//   BIST  interactive, started by the console command `bist`. Runs every check's BIST step in order. The operator answers
//         each step that asks:   p pass   f [note] fail   r rerun   s skip   q quit
//         Steps that decide for themselves (for example an RTC that must advance) need no answer.
//
// Both are non-blocking: call step() / bistStep() once per loop pass. Output goes through a small queue that waits for room in
// the console, so a slow or absent PC never stalls the loop and no result line is lost.
#pragma once

#include <stdint.h>

#include "../core/TimedState.h"
#include "../ui/Console.h"
#include "DeviceCheck.h"

namespace n2 {

constexpr uint8_t kMaxChecks = 8;

enum class BistVerdict : uint8_t { kNotRun, kPass, kFail, kSkip };

class SelfTest : public LineHook {
 public:
  // `checks` must stay valid for the lifetime of this object. The Console is used for output only.
  SelfTest(Console& console, DeviceCheck* const* checks, uint8_t count) : console_(console), checks_(checks), count_(count) {}

  // ---- POST ----
  void postBegin(uint32_t now);
  // Returns true once POST is finished. `release` is true while the operator is pressing TOB (or has typed `go`).
  bool postStep(uint32_t now, bool release);
  bool postHolding() const { return phase_ == Phase::kHold; }
  bool postFinished() const { return phase_ == Phase::kDone; }
  CheckLevel postLevel() const;                       // the worst level seen so far
  const CheckResult& postResult(uint8_t i) const { return post_[i]; }
  uint8_t checkCount() const { return count_; }
  const char* checkName(uint8_t i) const { return checks_[i]->name(); }
  uint8_t postIndex() const { return postIndex_; }    // the check about to run, or being shown

  // ---- BIST ----
  bool bistBegin(uint32_t now);                       // false if POST is still running
  bool bistStep(uint32_t now);                        // true when BIST has finished (completed or quit)
  bool bistRunning() const { return bist_ != Bist::kIdle; }
  uint8_t bistIndex() const { return bistIndex_; }
  BistVerdict bistVerdict(uint8_t i) const { return verdict_[i]; }
  const char* bistPrompt() const;                     // while waiting for the operator, else ""
  void onLine(const char* line) override;             // the operator's answer, from the console

 private:
  enum class Phase : uint8_t { kIdle, kChecks, kHold, kDone };
  enum class Bist : uint8_t { kIdle, kStart, kRunning, kAsking, kSummary };
  enum class Key : uint8_t { kNone, kPass, kFail, kRerun, kSkip, kQuit };

  void say(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void pump();
  void conclude(uint32_t now, BistVerdict v, const char* note);
  void finishBist(const char* why);
  static const char* levelWord(CheckLevel level);

  Console& console_;
  DeviceCheck* const* checks_;
  uint8_t count_;

  Phase phase_ = Phase::kIdle;
  uint8_t postIndex_ = 0;
  CheckResult post_[kMaxChecks];
  bool tobWasUp_ = false;

  Bist bist_ = Bist::kIdle;
  uint8_t bistIndex_ = 0;
  BistVerdict verdict_[kMaxChecks] = {};
  char notes_[kMaxChecks][24] = {};
  Key key_ = Key::kNone;
  char keyNote_[24] = {};
  bool prompted_ = false;
  uint32_t lastTick_ = 0;

  static constexpr uint8_t kQueue = 14;
  char queue_[kQueue][96];
  uint8_t head_ = 0, used_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace n2
