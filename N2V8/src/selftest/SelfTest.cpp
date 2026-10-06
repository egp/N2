#include "SelfTest.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace n2 {

// ------------------------------------------------------------------ output queue
void SelfTest::say(const char* fmt, ...) {
  if (used_ >= kQueue) {
    ++dropped_;
    return;
  }
  char* slot = queue_[(head_ + used_) % kQueue];
  va_list args;
  va_start(args, fmt);
  vsnprintf(slot, sizeof queue_[0], fmt, args);
  va_end(args);
  ++used_;
}

void SelfTest::pump() {
  while (used_ > 0 && console_.tryPrint(queue_[head_])) {
    head_ = static_cast<uint8_t>((head_ + 1) % kQueue);
    --used_;
  }
}

const char* SelfTest::levelWord(CheckLevel level) {
  switch (level) {
    case CheckLevel::kPass: return "ok";
    case CheckLevel::kInfo: return "info";
    case CheckLevel::kFail: return "FAIL";
    default: return "-";
  }
}

// ------------------------------------------------------------------ POST
void SelfTest::postBegin(uint32_t) {
  phase_ = Phase::kChecks;
  postIndex_ = 0;
  for (uint8_t i = 0; i < kMaxChecks; ++i) post_[i] = CheckResult();
  tobWasUp_ = false;  // a TOB held since power-up has to be released and pressed again
}

CheckLevel SelfTest::postLevel() const {
  CheckLevel worst = CheckLevel::kNone;
  for (uint8_t i = 0; i < count_; ++i) {
    if (post_[i].level == CheckLevel::kFail) return CheckLevel::kFail;
    if (post_[i].level == CheckLevel::kInfo) worst = CheckLevel::kInfo;
    else if (post_[i].level == CheckLevel::kPass && worst == CheckLevel::kNone) worst = CheckLevel::kPass;
  }
  return worst;
}

bool SelfTest::postStep(uint32_t now, bool release) {
  pump();
  switch (phase_) {
    case Phase::kIdle:
    case Phase::kDone:
      return phase_ == Phase::kDone;
    case Phase::kChecks: {
      if (postIndex_ < count_) {
        const CheckResult r = checks_[postIndex_]->post(now);
        post_[postIndex_] = r;
        say("POST %u/%u %-6s %-4s %s", static_cast<unsigned>(postIndex_ + 1), static_cast<unsigned>(count_),
            checks_[postIndex_]->name(), levelWord(r.level), r.text);
        ++postIndex_;
        return false;
      }
      const CheckLevel level = postLevel();
      if (level == CheckLevel::kFail) {
        say("POST FAILED: holding. Press TOB (or type `go`) to continue anyway.");
        tobWasUp_ = !release;
        phase_ = Phase::kHold;
        return false;
      }
      say("POST %s", level == CheckLevel::kInfo ? "done, with notes (see 'info' lines above)" : "OK");
      phase_ = Phase::kDone;
      return true;
    }
    case Phase::kHold: {
      if (!release) tobWasUp_ = true;           // released since the hold began
      if (release && tobWasUp_) {
        say("POST hold released by the operator");
        phase_ = Phase::kDone;
        return true;
      }
      return false;
    }
  }
  return false;
}

// ------------------------------------------------------------------ BIST
bool SelfTest::bistBegin(uint32_t now) {
  if (phase_ != Phase::kDone || bist_ != Bist::kIdle) return false;
  bist_ = Bist::kStart;
  bistIndex_ = 0;
  for (uint8_t i = 0; i < kMaxChecks; ++i) {
    verdict_[i] = BistVerdict::kNotRun;
    notes_[i][0] = '\0';
  }
  key_ = Key::kNone;
  lastTick_ = now;
  say("BIST: %u steps. After each step that asks, answer  p pass | f [note] fail | r rerun | s skip | q quit",
      static_cast<unsigned>(count_));
  return true;
}

const char* SelfTest::bistPrompt() const {
  return bist_ == Bist::kAsking ? checks_[bistIndex_]->bistPrompt() : "";
}

void SelfTest::onLine(const char* line) {
  if (bist_ == Bist::kIdle) return;
  while (*line == ' ' || *line == '\t') ++line;
  const char c = *line;
  key_ = Key::kNone;
  keyNote_[0] = '\0';
  switch (c) {
    case 'p': case 'P': key_ = Key::kPass; break;
    case 'f': case 'F': key_ = Key::kFail; break;
    case 'r': case 'R': key_ = Key::kRerun; break;
    case 's': case 'S': key_ = Key::kSkip; break;
    case 'q': case 'Q': key_ = Key::kQuit; break;
    default: say("BIST: answer p, f, r, s or q"); return;
  }
  if (key_ == Key::kFail) {
    const char* note = line + 1;
    while (*note == ' ' || *note == '\t') ++note;
    strncpy(keyNote_, note, sizeof keyNote_ - 1);
    keyNote_[sizeof keyNote_ - 1] = '\0';
  }
}

void SelfTest::conclude(uint32_t now, BistVerdict v, const char* note) {
  checks_[bistIndex_]->bistEnd();
  verdict_[bistIndex_] = v;
  strncpy(notes_[bistIndex_], note, sizeof notes_[0] - 1);
  notes_[bistIndex_][sizeof notes_[0] - 1] = '\0';
  say("BIST %u/%u %-6s %s%s%s", static_cast<unsigned>(bistIndex_ + 1), static_cast<unsigned>(count_), checks_[bistIndex_]->name(),
      v == BistVerdict::kPass ? "PASS" : (v == BistVerdict::kFail ? "FAIL" : "skipped"), *note ? "  " : "", note);
  ++bistIndex_;
  bist_ = Bist::kStart;
  prompted_ = false;
  key_ = Key::kNone;
  lastTick_ = now;
}

void SelfTest::finishBist(const char* why) {
  uint8_t pass = 0, fail = 0, skip = 0;
  for (uint8_t i = 0; i < count_; ++i) {
    if (verdict_[i] == BistVerdict::kPass) ++pass;
    else if (verdict_[i] == BistVerdict::kFail) ++fail;
    else ++skip;
  }
  say("BIST %s: %u pass, %u fail, %u not run or skipped", why, static_cast<unsigned>(pass), static_cast<unsigned>(fail),
      static_cast<unsigned>(skip));
  bist_ = Bist::kIdle;
}

bool SelfTest::bistStep(uint32_t now) {
  pump();
  if (bist_ == Bist::kIdle) return true;
  if (key_ == Key::kQuit) {
    if (bistIndex_ < count_) checks_[bistIndex_]->bistEnd();
    key_ = Key::kNone;
    finishBist("quit");
    return true;
  }
  if (bist_ == Bist::kStart) {
    if (bistIndex_ >= count_) {
      finishBist("complete");
      return true;
    }
    say("BIST %u/%u %s", static_cast<unsigned>(bistIndex_ + 1), static_cast<unsigned>(count_), checks_[bistIndex_]->name());
    checks_[bistIndex_]->bistBegin(now);
    bist_ = Bist::kRunning;
    return false;
  }
  const BistProgress p = checks_[bistIndex_]->bistTick(now);
  switch (p) {
    case BistProgress::kRunning:
      key_ = Key::kNone;  // answers typed before the step asks are ignored
      return false;
    case BistProgress::kPass:
      conclude(now, BistVerdict::kPass, checks_[bistIndex_]->bistNote());
      return false;
    case BistProgress::kFail:
      conclude(now, BistVerdict::kFail, checks_[bistIndex_]->bistNote());
      return false;
    case BistProgress::kAskOperator:
      if (!prompted_) {
        say("   %s", checks_[bistIndex_]->bistPrompt());
        say("   answer: p pass | f [note] fail | r rerun | s skip | q quit");
        prompted_ = true;
        bist_ = Bist::kAsking;
      }
      break;
  }
  if (bist_ == Bist::kAsking || p == BistProgress::kAskOperator) {
    bist_ = Bist::kAsking;
    switch (key_) {
      case Key::kPass: conclude(now, BistVerdict::kPass, checks_[bistIndex_]->bistNote()); break;
      case Key::kFail: conclude(now, BistVerdict::kFail, keyNote_[0] ? keyNote_ : checks_[bistIndex_]->bistNote()); break;
      case Key::kSkip: conclude(now, BistVerdict::kSkip, ""); break;
      case Key::kRerun:
        checks_[bistIndex_]->bistEnd();
        checks_[bistIndex_]->bistBegin(now);
        bist_ = Bist::kRunning;
        prompted_ = false;
        key_ = Key::kNone;
        break;
      default: break;
    }
  }
  return false;
}

}  // namespace n2
