#include "Lcd20x4.h"

#include <string.h>

#include "../core/TimedState.h"

namespace n2 {

namespace {
constexpr uint8_t kRs = 0x01;
constexpr uint8_t kEn = 0x04;
constexpr uint8_t kBl = 0x08;
constexpr uint32_t kPowerUpMs = 50;
const uint8_t kRowAddress[4] = {0x00, 0x40, 0x14, 0x54};

// HD44780 4-bit initialisation (as V7): three 0x30 nibbles, switch to 4-bit, function set, display, entry mode, clear.
struct InitStep {
  bool nibble;      // true: single nibble (before 4-bit mode is active); false: full command byte
  uint8_t value;
  uint32_t waitMs;  // wait after the step
};
const InitStep kInit[] = {
    {true, 0x30, 5}, {true, 0x30, 5}, {true, 0x30, 1}, {true, 0x20, 1},
    {false, 0x28, 0},  // 4-bit, 2 lines, 5x8 font
    {false, 0x0C, 0},  // display on, cursor off
    {false, 0x06, 0},  // entry mode: increment, no shift
    {false, 0x01, 2},  // clear (takes ~1.5 ms)
};
constexpr uint8_t kInitSteps = sizeof kInit / sizeof kInit[0];
}  // namespace

bool Lcd20x4::writeNibble(uint8_t n) {
  const uint8_t v = static_cast<uint8_t>((n & 0xF0) | (backlight_ ? kBl : 0));
  const uint8_t bytes[2] = {static_cast<uint8_t>(v | kEn), v};
  return hal_.i2cWrite(address_, bytes, 2);
}

bool Lcd20x4::writeByte(uint8_t value, bool data) {
  const uint8_t flags = static_cast<uint8_t>((backlight_ ? kBl : 0) | (data ? kRs : 0));
  const uint8_t hi = static_cast<uint8_t>((value & 0xF0) | flags);
  const uint8_t lo = static_cast<uint8_t>(((value << 4) & 0xF0) | flags);
  const uint8_t bytes[4] = {static_cast<uint8_t>(hi | kEn), hi, static_cast<uint8_t>(lo | kEn), lo};
  return hal_.i2cWrite(address_, bytes, 4);
}

bool Lcd20x4::writeRaw(uint8_t value) { return hal_.i2cWrite(address_, &value, 1); }

void Lcd20x4::fail(uint32_t now) {
  healthy_ = false;
  ++errors_;
  state_ = State::kRetry;
  until_ = now + kRetryMs;
}

void Lcd20x4::startInit(uint32_t now) {
  state_ = State::kInit;
  initStep_ = 0;
  until_ = now + kPowerUpMs;
  curRow_ = curCol_ = -1;
  for (auto& r : shadow_) {
    memset(r, ' ', kLcdCols);
    r[kLcdCols] = '\0';
  }
  for (auto& row : stale_) row = 0;  // the clear that ends init has made the shadow true
}

void Lcd20x4::begin(uint32_t now) {
  if (!haveDesired_) {
    for (auto& r : desired_) {
      memset(r, ' ', kLcdCols);
      r[kLcdCols] = '\0';
    }
    haveDesired_ = true;
  }
  backlightDirty_ = false;
  displayDirty_ = false;
  healStarted_ = false;  // the healing schedule starts when the display first comes up
  healReinits_ = 0;
  nextRewrite_.clear();
  nextReinit_.clear();
  startInit(now);
}

void Lcd20x4::setScreen(const Screen& s) {
  for (uint8_t r = 0; r < kLcdRows; ++r) {
    memcpy(desired_[r], s.row[r], kLcdCols);
    desired_[r][kLcdCols] = '\0';
  }
  haveDesired_ = true;
}

void Lcd20x4::setBacklight(bool on) {
  if (on != backlight_) {
    backlight_ = on;
    backlightDirty_ = true;
  }
}

void Lcd20x4::setDisplayOn(bool on) {
  if (on != displayOn_) {
    displayOn_ = on;
    displayDirty_ = true;
  }
}

Lcd20x4::BusTest Lcd20x4::busTest(uint16_t rounds) {
  static const uint8_t kPatterns[] = {0x00, 0xF0, 0x50, 0xA0, 0x30, 0xC0, 0x90, 0x60};
  BusTest t;
  const uint32_t start = hal_.micros();
  for (uint16_t i = 0; i < rounds; ++i) {
    const uint8_t out = static_cast<uint8_t>(kPatterns[i % (sizeof kPatterns)] | (backlight_ ? kBl : 0));  // RS = RW = EN = 0
    ++t.rounds;
    if (!hal_.i2cWrite(address_, &out, 1)) { ++t.writeFailed; continue; }
    uint8_t back = 0;
    if (!hal_.i2cRead(address_, &back, 1)) { ++t.readFailed; continue; }
    // Only P4..P7 (the LCD data lines) are compared: P3 drives the backlight transistor, whose base pulls the pin low, and the
    // low three lines are the LCD's RS/RW/EN, which the backpack may load.
    if ((back & 0xF0) != (out & 0xF0)) {
      if (t.mismatched == 0) {
        t.firstOut = out;
        t.firstBack = back;
      }
      ++t.mismatched;
    }
  }
  t.microsPerRound = rounds ? (hal_.micros() - start) / rounds : 0;
  writeRaw(backlight_ ? kBl : 0);
  return t;
}

void Lcd20x4::refresh() {
  for (auto& row : stale_) row = (1UL << kLcdCols) - 1UL;  // every cell is rewritten; shadow_ stays an honest record of the display
  curRow_ = curCol_ = -1;
}

void Lcd20x4::reinit(uint32_t now) {
  if (state_ == State::kIdle) return;  // begin() was never called
  ++reinits_;
  startInit(now);
  until_ = now;  // the display has been powered all along: no power-up wait
  if (healStarted_) {  // after any re-init the early rewrites start over
    rewriteGapMs_ = heal_.firstRewriteMs;
    nextRewrite_.arm(now, rewriteGapMs_);
  }
}

void Lcd20x4::serviceHealing(uint32_t now) {
  if (!healStarted_) {
    healStarted_ = true;
    healReadyAt_ = now;
    rewriteGapMs_ = heal_.firstRewriteMs;
    nextRewrite_.arm(now, rewriteGapMs_);
    if (heal_.reinitAfterMs1 > 0) nextReinit_.arm(now, heal_.reinitAfterMs1);
    return;
  }
  if (nextReinit_.reached(now)) {
    ++healReinits_;
    if (healReinits_ == 1 && heal_.reinitAfterMs2 > 0) nextReinit_.armAt(healReadyAt_ + heal_.reinitAfterMs2);
    else if (heal_.reinitEveryMs > 0) nextReinit_.arm(now, heal_.reinitEveryMs);
    else nextReinit_.clear();
    reinit(now);
    return;
  }
  if (nextRewrite_.reached(now)) {
    refresh();
    homePending_ = true;
    rewriteGapMs_ = rewriteGapMs_ * 2 < heal_.rewriteEveryMs ? rewriteGapMs_ * 2 : heal_.rewriteEveryMs;
    nextRewrite_.arm(now, rewriteGapMs_);
  }
}

bool Lcd20x4::inSync() const {
  if (state_ != State::kReady || backlightDirty_ || displayDirty_) return false;
  for (uint8_t r = 0; r < kLcdRows; ++r)
    if (memcmp(shadow_[r], desired_[r], kLcdCols) != 0 || stale_[r] != 0) return false;
  return true;
}

// Run init steps whose wait has passed. Returns false if it failed.
bool Lcd20x4::serviceInit(uint32_t now) {
  while (initStep_ < kInitSteps && deadlineReached(now, until_)) {
    const InitStep& s = kInit[initStep_];
    if (!(s.nibble ? writeNibble(s.value) : writeByte(s.value, false))) return false;
    until_ = now + s.waitMs;
    ++initStep_;
    if (s.waitMs > 0) break;  // wait before the next step
  }
  if (initStep_ >= kInitSteps && deadlineReached(now, until_)) {
    state_ = State::kReady;
    healthy_ = true;
    displayDirty_ = !displayOn_;  // the init turned the display on; restore a BIST-chosen "off"
  }
  return true;
}

void Lcd20x4::serviceContent(uint32_t now) {
  if (!deadlineReached(now, homeUntil_)) return;   // 'return home' takes 1.5 ms: write nothing until it is done
  if (homePending_) {
    if (!writeByte(0x06, false) || !writeByte(0x02, false)) return fail(now);
    homePending_ = false;
    homeUntil_ = now + 2;
    curRow_ = curCol_ = -1;
    return;
  }
  if (backlightDirty_) {
    if (!writeRaw(backlight_ ? kBl : 0)) return fail(now);
    backlightDirty_ = false;
  }
  if (displayDirty_) {
    if (!writeByte(displayOn_ ? 0x0C : 0x08, false)) return fail(now);
    displayDirty_ = false;
  }
  uint8_t budget = kCharsPerService;
  for (uint8_t r = 0; r < kLcdRows && budget > 0; ++r) {
    uint8_t c = 0;
    while (c < kLcdCols && budget > 0) {
      if (shadow_[r][c] == desired_[r][c] && !((stale_[r] >> c) & 1UL)) {
        ++c;
        continue;
      }
      if (curRow_ != static_cast<int8_t>(r) || curCol_ != static_cast<int8_t>(c)) {  // move the cursor
        if (!writeByte(static_cast<uint8_t>(0x80 | (kRowAddress[r] + c)), false)) return fail(now);
        curRow_ = static_cast<int8_t>(r);
        curCol_ = static_cast<int8_t>(c);
      }
      if (!writeByte(static_cast<uint8_t>(desired_[r][c]), true)) return fail(now);
      shadow_[r][c] = desired_[r][c];
      stale_[r] &= ~(1UL << c);
      ++c;
      curCol_ = static_cast<int8_t>(curCol_ + 1);
      if (c >= kLcdCols) curRow_ = curCol_ = -1;  // DDRAM address runs on into another row
      --budget;
    }
  }
}

void Lcd20x4::service(uint32_t now) {
  switch (state_) {
    case State::kIdle:
      return;
    case State::kInit:
      if (!serviceInit(now)) return fail(now);
      return;  // content is written on the next pass, so one call never does both
    case State::kRetry:
      if (!deadlineReached(now, until_)) return;
      hal_.i2cRecover();  // the cause may be a wedged bus, not the display: free it before trying again
      ++recoveries_;
      if (!hal_.i2cProbe(address_)) {
        until_ = now + kRetryMs;
        return;
      }
      startInit(now);  // it is back: full re-initialisation (DSP-7)
      until_ = now;    // no extra power-up wait: the display has been powered all along
      return;
    case State::kReady:
      if (healOn_) serviceHealing(now);
      if (alwaysRewrite_ && inSync()) refresh();  // the previous full pass is done: start the next one
      break;
  }
  serviceContent(now);
}

}  // namespace n2
