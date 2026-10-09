#include "Bist.h"

#include "../ui/ScanReport.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <initializer_list>

#include "../core/Scaling.h"

namespace n2 {

namespace {
const char* const kStepNames[kBistSteps] = {"banner",  "TBS and TOB", "I2C scan",  "LED",      "LCD",        "pressures",
                                            "O2 sensor", "LEFT valve", "RIGHT valve", "FLUSH valve", "SSR", "summary"};
char hexDigit(uint8_t v) { return "0123456789ABCDEF"[v & 15]; }

void pinName(uint8_t pin, char* out, size_t n) {
  if (pin::isAnalog(pin)) snprintf(out, n, "A%u", static_cast<unsigned>(pin - pin::kA0));
  else snprintf(out, n, "D%u", static_cast<unsigned>(pin));
}

// "120.5" -> 12050 (x100). False if not a number.
bool parseX100(const char* s, uint32_t& out) {
  if (*s == '\0') return false;
  uint32_t whole = 0, frac = 0, scale = 100;
  bool dot = false, any = false;
  for (; *s != '\0'; ++s) {
    if (*s >= '0' && *s <= '9') {
      any = true;
      if (!dot) whole = whole * 10 + static_cast<uint32_t>(*s - '0');
      else if (scale > 1) { scale /= 10; frac += static_cast<uint32_t>(*s - '0') * scale; }
    } else if (*s == '.' && !dot) {
      dot = true;
    } else {
      return false;
    }
  }
  out = whole * 100 + frac;
  return any;
}

const char* signalLabel(Signal s) {
  switch (s) {
    case Signal::kLeftValve:  return "LEFT valve";
    case Signal::kRightValve: return "RIGHT valve";
    case Signal::kFlushValve: return "FLUSH valve";
    case Signal::kSsr:        return "SSR (compressor)";
    default: return "?";
  }
}
}  // namespace

const char* Bist::stepName(BistStep s) { return kStepNames[static_cast<uint8_t>(s)]; }

// BIST-11: an output step must never create an unsafe condition.
const char* Bist::vetoReason(Signal output, const Inputs& in, const ControlConfig& cfg) {
  const auto bad = [](uint16_t raw) { return classifyRaw(raw, kAdcBits) != RawStatus::kOk; };
  switch (output) {
    case Signal::kLeftValve:
    case Signal::kRightValve:
      if (bad(in.rawAir) || !in.airOk) return "air sensor out of range";
      if (in.airX10 < cfg.airLowOff) return "air supply pressure is low";
      if (bad(in.rawN2High) || !in.n2HighOk) return "N2-high sensor out of range";
      if (in.n2HighX10 > cfg.n2HighOff) return "N2-high pressure is over its limit";
      if (in.sensorOrderFault) return "N2-low reads above N2-high";
      return nullptr;
    case Signal::kSsr:
      if (bad(in.rawN2High) || !in.n2HighOk) return "N2-high sensor out of range";
      if (in.n2HighX10 >= cfg.n2HighOn) return "N2-high pressure is not below its start threshold";
      if (bad(in.rawN2Low) || !in.n2LowOk) return "N2-low sensor out of range";
      if (in.n2LowX100 < cfg.n2LowOff) return "N2-low pressure is too low";
      if (in.sensorOrderFault) return "N2-low reads above N2-high";
      return nullptr;
    default:
      return nullptr;  // the flush valve only vents the sensor line
  }
}

// ---------------------------------------------------------------------------------------------- output queue
void Bist::say(const char* fmt, ...) {
  if (qCount_ >= kQueue) {
    ++queueDrops_;
    return;
  }
  char* slot = queue_[(qHead_ + qCount_) % kQueue];
  va_list args;
  va_start(args, fmt);
  vsnprintf(slot, sizeof queue_[0], fmt, args);
  va_end(args);
  ++qCount_;
}

void Bist::flushOutput() {
  while (qCount_ > 0) {
    if (!console_.tryPrint(queue_[qHead_])) return;  // no room right now: keep it for the next pass
    qHead_ = static_cast<uint8_t>((qHead_ + 1) % kQueue);
    --qCount_;
  }
}

// ---------------------------------------------------------------------------------------------- helpers
bool Bist::rawTbs() {
  const SignalDef& d = def(board_, Signal::kTbs);
  return isOn(hal_.digitalRead(d.pin), d.active);
}
bool Bist::rawTob() {
  const SignalDef& d = def(board_, Signal::kTob);
  return isOn(hal_.digitalRead(d.pin), d.active);
}
bool Bist::tbsOn() { return running_ ? tbsDeb_.level() : rawTbs(); }
bool Bist::tobPressed() { return running_ ? tobDeb_.level() : rawTob(); }

Signal Bist::outputSignal() const {
  switch (step_) {
    case BistStep::kLeft:  return Signal::kLeftValve;
    case BistStep::kRight: return Signal::kRightValve;
    case BistStep::kFlush: return Signal::kFlushValve;
    default:               return Signal::kSsr;
  }
}

void Bist::outputsOff(uint32_t now) {
  for (Signal s : {Signal::kLeftValve, Signal::kRightValve, Signal::kFlushValve, Signal::kSsr})
    sys_.outputDriver().forceDrive(s, false, now);
  outputOn_ = false;
}

void Bist::showStep(const char* l1, const char* l2, const char* l3) {
  char head[24];
  snprintf(head, sizeof head, "BIST %c %s", hexDigit(static_cast<uint8_t>(step_)), kStepNames[static_cast<uint8_t>(step_)]);
  LedText led;
  snprintf(led.digit, sizeof led.digit, "   %c", hexDigit(static_cast<uint8_t>(step_)));
  led.dotAfter = -1;
  display_.setOverride(makeScreen(head, l1, l2, l3), led, hal_.millis());
}

// ---------------------------------------------------------------------------------------------- start / finish
Bist::Start Bist::begin(uint32_t now, bool requireTbsOff) {
  if (!console_.attached()) {  // BIST-1
    LedText blank;
    snprintf(blank.digit, sizeof blank.digit, "    ");
    blank.dotAfter = -1;
    display_.setOverride(makeScreen("BIST: NO CONSOLE", "Open the Serial", "Monitor and retry"), blank, now, 4000);
    return Start::kNoConsole;
  }
  if (requireTbsOff && tbsOn()) {
    console_.tryPrint("BIST refused: the system is enabled (TBS is ON). Switch TBS OFF, then type bist.");
    return Start::kTbsOn;
  }
  running_ = true;
  for (Result& r : result_) r = Result();
  step_ = BistStep::kBanner;
  key_ = Key::kNone;
  qHead_ = qCount_ = 0;
  queueDrops_ = 0;
  airAbortOff_ = false;
  holdOpen_ = false;
  tbsDeb_ = Debouncer(rawTbs(), sys_.tbsDebounceMs());   // start from the present levels: no false edge
  tobDeb_ = Debouncer(rawTob(), sys_.tobDebounceMs());
  tobWasUp_ = !tobDeb_.level();  // a TOB held since boot must be released first
  console_.setLineHook(this);
  outputsOff(now);
  lastTick_ = now;
  enter(now);
  return Start::kOk;
}

uint32_t bistRecordChecksum(const BistRecord& r) {
  uint32_t sum = r.magic ^ 0xA5C3E1F7u;
  sum = sum * 31u + r.running;
  sum = sum * 31u + r.step;
  for (uint8_t i = 0; i < kBistSteps; ++i) sum = sum * 31u + r.verdict[i];
  return sum;
}

bool bistRecordValid(const BistRecord& r) { return r.magic == kBistRecordMagic && r.check == bistRecordChecksum(r) && r.step < kBistSteps; }

void Bist::saveRecord(bool running) {
  if (rec_ == nullptr) return;
  rec_->magic = kBistRecordMagic;
  rec_->running = running ? 1 : 0;
  rec_->step = static_cast<uint8_t>(step_);
  for (uint8_t i = 0; i < kBistSteps; ++i) rec_->verdict[i] = static_cast<uint8_t>(result_[i].verdict);
  rec_->pad[0] = rec_->pad[1] = 0;
  rec_->check = bistRecordChecksum(*rec_);
}

Bist::Start Bist::resume(uint32_t now) {
  if (rec_ == nullptr || !bistRecordValid(*rec_) || !rec_->running) return Start::kNothingToResume;
  const BistRecord saved = *rec_;   // begin() overwrites the record
  const Start s = begin(now, true);   // needs the console and TBS OFF, like any BIST
  if (s != Start::kOk) {
    saveRecord(false);
    return s;
  }
  for (uint8_t i = 0; i < kBistSteps; ++i) {
    const uint8_t v = saved.verdict[i];
    result_[i].verdict = v <= static_cast<uint8_t>(BistVerdict::kSkip) ? static_cast<BistVerdict>(v) : BistVerdict::kNotRun;
  }
  step_ = static_cast<BistStep>(saved.step);
  say("BIST RESUMED after a reset at step %c %s (the results so far are kept; q quits)", hexDigit(saved.step), kStepNames[saved.step]);
  enter(now);
  return Start::kOk;
}

// One summary of the sag test, from the 50 ms samples taken while the valve was open (owner 2026-10-09): how long the pressure fell before it
// started to rise (the minimum), then how long until it was stable (within +-1 PSI for at least 500 ms), and the level it settles at.
void Bist::sagSummary(Signal sig, uint32_t now) {
  (void)now;
  if (sagN_ < 12) { say("  SAG TEST: too few samples (%u)", static_cast<unsigned>(sagN_)); return; }
  const uint16_t base = airBeforeX10_;
  uint8_t iMin = 0;
  for (uint8_t i = 1; i < sagN_; ++i) if (sagHist_[i] < sagHist_[iMin]) iMin = i;
  const uint16_t vmin = sagHist_[iMin];
  int fall10 = -1;
  for (uint8_t i = 0; i < sagN_ && fall10 < 0; ++i) if (sagHist_[i] + 100 <= base) fall10 = i * 50;
  int stableAt = -1;        // first sample from which 10 samples (500 ms) stay within 1.0 PSI of each other
  uint16_t stableLevel = 0;
  for (uint8_t i = iMin; i + 10 <= sagN_ && stableAt < 0; ++i) {
    uint16_t lo = 0xFFFF, hi = 0;
    uint32_t sum = 0;
    for (uint8_t k = i; k < i + 10; ++k) {
      if (sagHist_[k] < lo) lo = sagHist_[k];
      if (sagHist_[k] > hi) hi = sagHist_[k];
      sum += sagHist_[k];
    }
    if (hi - lo <= 10) { stableAt = i * 50; stableLevel = static_cast<uint16_t>(sum / 10); }
  }
  const int down = static_cast<int>(base) - static_cast<int>(vmin);
  say("  SAG TEST %s: before %u.%u PSI, MINIMUM %u.%u PSI (down %d.%d)", signalLabel(sig), static_cast<unsigned>(base / 10u), static_cast<unsigned>(base % 10u),
      static_cast<unsigned>(vmin / 10u), static_cast<unsigned>(vmin % 10u), down / 10, down < 0 ? 0 : down % 10);
  say("  FALL: it fell for %u ms (10 PSI down by %d ms), then started to rise", static_cast<unsigned>(iMin) * 50u, fall10);
  if (stableAt >= 0)
    say("  STABLE (+-1 PSI) at %u.%u PSI from %d ms: %d ms after the minimum", static_cast<unsigned>(stableLevel / 10u), static_cast<unsigned>(stableLevel % 10u),
        stableAt, stableAt - static_cast<int>(iMin) * 50);
  else
    say("  NOT STABLE within the 5 s the valve was open (still moving by more than 1 PSI)");
  char l1[24], l2[24], l3[24];
  snprintf(l1, sizeof l1, "MIN %u.%u PSI", static_cast<unsigned>(vmin / 10u), static_cast<unsigned>(vmin % 10u));
  snprintf(l2, sizeof l2, "fell %u ms", static_cast<unsigned>(iMin) * 50u);
  if (stableAt >= 0) snprintf(l3, sizeof l3, "stable +%d ms", stableAt - static_cast<int>(iMin) * 50);
  else snprintf(l3, sizeof l3, "not stable in 5 s");
  showStep(l1, l2, l3);
  say("  answer p or f (r repeats)");
}

void Bist::finish(uint32_t now, const char* why) {
  outputsOff(now);
  say("BIST finished: %s", why);
  running_ = false;
  saveRecord(false);
  console_.setLineHook(nullptr);
  display_.clearOverride();
}

void Bist::conclude(uint32_t now, BistVerdict v, const char* note) {
  outputsOff(now);
  Result& r = result_[static_cast<uint8_t>(step_)];
  r.verdict = v;
  snprintf(r.note, sizeof r.note, "%s", note);
  say("  -> %c %s: %s%s%s", hexDigit(static_cast<uint8_t>(step_)), kStepNames[static_cast<uint8_t>(step_)],
      v == BistVerdict::kPass ? "PASS" : (v == BistVerdict::kFail ? "FAIL" : "SKIPPED"), note[0] ? "  " : "", note);
  if (static_cast<uint8_t>(step_) + 1 < kBistSteps) {
    step_ = static_cast<BistStep>(static_cast<uint8_t>(step_) + 1);
    enter(now);
  }
}

void Bist::printSummary() {
  uint8_t pass = 0, fail = 0, skip = 0;
  say("---- BIST RESULTS ----");
  for (uint8_t i = 0; i + 1 < kBistSteps; ++i) {
    const Result& r = result_[i];
    const char* v = r.verdict == BistVerdict::kPass ? "PASS" : (r.verdict == BistVerdict::kFail ? "FAIL" : (r.verdict == BistVerdict::kSkip ? "SKIPPED" : "not run"));
    say("  %c %-12s %s%s%s", hexDigit(i), kStepNames[i], v, r.note[0] ? "  " : "", r.note);
    if (r.verdict == BistVerdict::kPass) ++pass;
    else if (r.verdict == BistVerdict::kFail) ++fail;
    else if (r.verdict == BistVerdict::kSkip) ++skip;
  }
  say("BIST: %u pass, %u fail, %u skipped", static_cast<unsigned>(pass), static_cast<unsigned>(fail), static_cast<unsigned>(skip));
}

// ---------------------------------------------------------------------------------------------- enter a step
void Bist::enter(uint32_t now) {
  sagTest_ = false;
  sagN_ = 0;
  airMinX10_ = 0xFFFF;
  lcdAirMark_ = 0;
  firstOnAt_ = 0;
  saveRecord(true);   // the step and the verdicts so far, for a resume after a reset-button reset
  stepStart_ = now;
  mark_ = now;
  phase_ = 0;
  lcdPhase_ = -1;
  refused_ = false;
  outputOn_ = false;
  pulseDone_ = false;
  scanAddr_ = 0x08;
  memset(found_, 0, sizeof found_);
  for (auto& l : last_) l = 0xFFFF;
  lastBool_[0] = lastBool_[1] = false;
  display_.led().setDisplayOn(true);

  say("BIST %c: %s", hexDigit(static_cast<uint8_t>(step_)), kStepNames[static_cast<uint8_t>(step_)]);
  switch (step_) {
    case BistStep::kBanner:
      say("  N2V8 %s  built %s %s", info_.version, info_.date, info_.time);
      say("  board %s  mode %s  adc %u bits", info_.board, info_.mode, static_cast<unsigned>(info_.adcBits));
      say("  Output steps need TBS OFF and are vetoed if conditions are unsafe.");
      showStep("Console attached", "Press TOB or type p");
      break;
    case BistStep::kSwitches:
      say("  Toggle TBS and press TOB; each change is printed. TOB does NOT confirm in this step.");
      showStep("Toggle TBS, press TOB", "then type p or f");
      break;
    case BistStep::kI2c:
      say("  Scanning 0x08-0x77 ...");
      showStep("Scanning I2C ...");
      break;
    case BistStep::kLed:
      say("  EXPECT LED: 0000 1111 2222 3333 4444 5555 6666 7777 8888 9999, then a decimal point");
      say("  walks across 8888 (left to right), then the display blinks off and on. ~4 s, repeats.");
      break;  // the LED step drives the LED itself; the LCD shows the step number
    case BistStep::kLcd:
      say("  EXPECT LCD: the text; all 80 cells '#'; letters and digits. ~4 s, repeats. Only the text changes.");
      break;  // the LCD step drives the LCD itself
    case BistStep::kPressures:
      say("  Raw counts, volts and PSI per sensor (printed on change). Compare with the production gauges and enter");
      say("  them: g air 120.5   g n2l 12.3   g n2h 98.0.  Disconnect a sensor (system OFF) to see its fault values.");
      showStep("See console", "Enter gauges: g air 120");
      break;
    case BistStep::kO2:
      say("  Starting communication with the O2 sensor; readings are printed on change.");
      showStep("O2 sensor ...");
      break;
    case BistStep::kLeft:
    case BistStep::kRight:
    case BistStep::kFlush:
    case BistStep::kSsr: {
      const Signal sig = outputSignal();
      const SignalDef& d = def(board_, sig);
      char pn[8];
      pinName(d.pin, pn, sizeof pn);
      say("  %s: pin %s, active %s.", signalLabel(sig), pn, d.active == Active::kHigh ? "HIGH" : "LOW");
      if (sig == Signal::kSsr && cfg_.ssrSinglePulse)
        say("  The compressor relay gets ONE %lu ms pulse (listen/watch for the contactor). Type r to repeat.", static_cast<unsigned long>(cfg_.ssrPulseMs));
      else
        say("  It toggles at ~%lu Hz for up to %lu s. Listen for it. Any answer stops it.", 500UL * 2UL / cfg_.halfPeriodMs,
            static_cast<unsigned long>(cfg_.outputStepMaxMs / 1000));
      showStep("Needs TBS OFF", "Safety-checked");
      break;
    }
    case BistStep::kSummary:
      printSummary();
      finish(now, "all steps done");
      return;
    case BistStep::kCount:
      return;
  }
  if (step_ != BistStep::kSummary)
    say("  answer: p=pass f=fail r=rerun s=skip q=quit%s", (step_ == BistStep::kSwitches || !cfg_.switchKeys) ? "" : "   (TOB = pass, TBS ON = fail)");
}

// ---------------------------------------------------------------------------------------------- operator input
void Bist::onLine(const char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return;
  const char c = (*line >= 'A' && *line <= 'Z') ? static_cast<char>(*line - 'A' + 'a') : *line;
  const char* rest = line + 1;
  while (*rest == ' ' || *rest == '\t') ++rest;
  if (c == 'g' && step_ == BistStep::kPressures) {  // g <air|n2l|n2h> <psi>
    char which[8] = {};
    char value[16] = {};
    if (sscanf(rest, "%7s %15s", which, value) == 2) {
      for (char* w = which; *w; ++w) if (*w >= 'A' && *w <= 'Z') *w = static_cast<char>(*w - 'A' + 'a');   // AIR = air
      uint32_t x100;
      int idx = -1;
      if (strcmp(which, "air") == 0) idx = 0;
      else if (strcmp(which, "n2l") == 0) idx = 1;
      else if (strcmp(which, "n2h") == 0) idx = 2;
      if (idx >= 0 && parseX100(value, x100)) {
        gaugeSet_[idx] = true;
        gauge_[idx] = static_cast<uint16_t>(x100 > 65535u ? 65535u : x100);
        const uint32_t sensor = idx == 0 ? inputs_.airX10 * 10u : (idx == 1 ? inputs_.n2LowX100 : inputs_.n2HighX10 * 10u);
        const long diff = static_cast<long>(sensor) - static_cast<long>(x100);
        say("  %s gauge %lu.%02lu vs sensor %lu.%02lu PSI: diff %c%lu.%02lu", which, static_cast<unsigned long>(x100 / 100),
            static_cast<unsigned long>(x100 % 100), static_cast<unsigned long>(sensor / 100), static_cast<unsigned long>(sensor % 100),
            diff < 0 ? '-' : '+', static_cast<unsigned long>((diff < 0 ? -diff : diff) / 100), static_cast<unsigned long>((diff < 0 ? -diff : diff) % 100));
        return;
      }
    }
    say("  usage: g air|n2l|n2h <psi>   e.g. g air 120.5");
    return;
  }
  if (c == 't' && rest[0] == '\0' && (step_ == BistStep::kLeft || step_ == BistStep::kRight || step_ == BistStep::kFlush)) {   // timed sag test
    sagTest_ = true;
    sagN_ = 0;
    holdOpen_ = true;
    airAbortOff_ = false;
    say("  timed sag test armed: the valve opens once, stays open 5 s while the air is recorded, then one summary. (Low air is tolerated for the first 1000 ms.)");
    return;
  }
  if (c == 'h' && rest[0] == '\0') {   // toggle hold-open for the valve steps
    holdOpen_ = !holdOpen_;
    say("  hold open %s (key h toggles it): a valve step %s", holdOpen_ ? "ON" : "off", holdOpen_ ? "opens the valve ONCE and keeps it open until you answer or the step limit" : "toggles at about 2 Hz");
    return;
  }
  if (c == 'a' && rest[0] == '\0') {   // toggle the low-air abort (for watching the pressure drop on a valve step)
    airAbortOff_ = !airAbortOff_;
    say("  low-air abort %s (key a toggles it; it never disables the TBS abort)", airAbortOff_ ? "OFF: observe only" : "ON");
    return;
  }
  switch (c) {
    case 'p': key_ = Key::kPass; break;
    case 'f': key_ = Key::kFail; snprintf(keyNote_, sizeof keyNote_, "%s", rest); break;
    case 'r': key_ = Key::kRerun; break;
    case 's': key_ = Key::kSkip; break;
    case 'q': key_ = Key::kQuit; break;
    default: say("  ? answer p, f, r, s or q  (f may be followed by a note)"); break;
  }
}

// ---------------------------------------------------------------------------------------------- per-step work
void Bist::tickSwitches(uint32_t now) {
  const bool tbs = tbsOn();
  const bool tob = tobPressed();
  if (phase_ == 0 || tbs != lastBool_[0] || tob != lastBool_[1]) {
    if (phase_ == 0 || tbs != lastBool_[0]) say("  TBS %s", tbs ? "ON" : "OFF");
    if (phase_ != 0 && tob != lastBool_[1]) say("  TOB %s", tob ? "pressed" : "released");
    lastBool_[0] = tbs;
    lastBool_[1] = tob;
    phase_ = 1;
    char a[24], b[24];
    snprintf(a, sizeof a, "TBS %s", tbs ? "ON" : "OFF");
    snprintf(b, sizeof b, "TOB %s", tob ? "PRESSED" : "released");
    showStep(a, b, "then type p or f");
  }
  (void)now;
}

void Bist::tickI2c(uint32_t now) {
  if (phase_ == 0) {
    for (uint8_t n = 0; n < 16 && scanAddr_ < 0x78; ++n, ++scanAddr_)  // spread over passes
      if (hal_.i2cProbe(scanAddr_)) found_[scanAddr_ / 8] = static_cast<uint8_t>(found_[scanAddr_ / 8] | (1u << (scanAddr_ % 8)));
    if (scanAddr_ < 0x78) return;
    phase_ = 1;
    ScanReport rep;
    buildScanReport(board_, found_, rep);
    for (uint8_t i = 0; i < rep.count; ++i) say("  %s", rep.line[i]);
    char l1[24], l2[24];
    snprintf(l1, sizeof l1, "%u answered", static_cast<unsigned>(rep.responses));
    snprintf(l2, sizeof l2, "%u missing %u unexp", static_cast<unsigned>(rep.missing), static_cast<unsigned>(rep.unexpected));
    showStep(l1, l2, "then p / f");
  }
  (void)now;
}

void Bist::tickLed(uint32_t now) {
  const uint32_t t = static_cast<uint32_t>(now - stepStart_) % 4200u;
  LedText led;
  led.dotAfter = -1;
  bool on = true;
  if (t < 2000) {
    const char d = static_cast<char>('0' + t / 200u);
    for (int i = 0; i < 4; ++i) led.digit[i] = d;
  } else if (t < 3400) {
    for (int i = 0; i < 4; ++i) led.digit[i] = '8';
    led.dotAfter = static_cast<int8_t>((t - 2000u) / 350u);
  } else {
    for (int i = 0; i < 4; ++i) led.digit[i] = '8';
    on = t >= 3800;
  }
  led.digit[4] = '\0';
  display_.led().setText(led);
  display_.led().setDisplayOn(on);
  // The LCD shows the step number during the LED test (BIST-2).
  display_.lcd().setScreen(makeScreen("BIST 3 LED test", "Watch the LED", "then type p or f"));
}

void Bist::tickLcd(uint32_t now) {
  const uint32_t t = static_cast<uint32_t>(now - stepStart_) % 4000u;
  Lcd20x4& lcd = display_.lcd();
  // Only the TEXT changes (owner 2026-10-08): no backlight, no display on/off, no resync, no commands of any kind during this step. That is
  // all production does. The three screens: the explanation, all 80 cells '#', and letters and digits (a bad cell stands out).
  if (t < 1500) {
    lcd.setScreen(makeScreen("BIST 4 LCD test", "Watch the LCD", "then type p or f"));
  } else if (t < 2500) {
    const char* full = "####################";
    lcd.setScreen(makeScreen(full, full, full, full));
  } else {
    lcd.setScreen(makeScreen("ABCDEFGHIJKLMNOPQRST", "01234567890123456789", "abcdefghijklmnopqrst", "!\"#$%&'()*+,-./:;<=>"));
  }
  LedText led;
  snprintf(led.digit, sizeof led.digit, "   4");   // the LED shows the step number during the LCD test
  led.dotAfter = -1;
  display_.led().setText(led);
}

void Bist::tickPressures(uint32_t now) {
  if (static_cast<uint32_t>(now - mark_) < 200 && phase_ != 0) return;
  mark_ = now;
  phase_ = 1;
  struct S { const char* name; uint16_t raw; uint16_t value; bool x100; bool ok; };
  const S s[3] = {{"AIR", inputs_.rawAir, inputs_.airX10, false, inputs_.airOk},
                  {"N2L", inputs_.rawN2Low, inputs_.n2LowX100, true, inputs_.n2LowOk},
                  {"N2H", inputs_.rawN2High, inputs_.n2HighX10, false, inputs_.n2HighOk}};
  char rows[3][24];
  for (uint8_t i = 0; i < 3; ++i) {
    char v[8];
    if (s[i].x100) formatX100(v, s[i].value);
    else formatX10(v, s[i].value);
    const unsigned mv = millivoltsFromRaw(s[i].raw, kAdcBits);
    const RawStatus st = classifyRaw(s[i].raw, kAdcBits);
    const bool changed = s[i].raw + 1u < last_[i] || s[i].raw > last_[i] + 1u || last_[i] == 0xFFFF;
    if (changed) {
      last_[i] = s[i].raw;
      say("  %s raw %5u  %u.%02uV  %s PSI  %s", s[i].name, static_cast<unsigned>(s[i].raw), mv / 1000u, (mv % 1000u) / 10u, v,
          st == RawStatus::kOk ? "in window" : (st == RawStatus::kBelowWindow ? "BELOW WINDOW (wire off?)" : "ABOVE WINDOW"));
    }
    snprintf(rows[i], sizeof rows[i], "%s %s%s", s[i].name, v, st == RawStatus::kOk ? "" : " FAULT");
  }
  showStep(rows[0], rows[1], rows[2]);
}

void Bist::tickO2(uint32_t now) {
  if (phase_ == 0) {
    const bool ok = o2_.begin();
    say("  O2 sensor begin: %s", ok ? "OK" : "FAILED (no response)");
    phase_ = ok ? 1 : 2;
    mark_ = now;
    showStep(ok ? "O2 comm OK" : "O2 NO RESPONSE", "See console");
    return;
  }
  if (phase_ == 1 && static_cast<uint32_t>(now - mark_) >= 500) {
    mark_ = now;
    uint16_t v;
    if (o2_.readO2PercentX100(v)) {
      if (v != last_[0]) {
        last_[0] = v;
        char b[8];
        formatX100(b, v);
        say("  O2 %s %%", b);
        char l[24];
        snprintf(l, sizeof l, "O2 %s %%", b);
        showStep("O2 comm OK", l);
      }
    } else if (last_[0] != 0xFFFE) {
      last_[0] = 0xFFFE;
      say("  O2 read FAILED");
      showStep("O2 read FAILED");
    }
  }
}

void Bist::tickOutputStep(uint32_t now, Signal sig) {
  if (phase_ == 0) airBeforeX10_ = inputs_.airX10;   // before the output is switched on
  if (phase_ == 0) {  // may we start? (BIST-4, BIST-11)
    const char* why = nullptr;
    if (rawTbs()) why = "TBS is ON - switch it OFF";
    else why = vetoReason(sig, inputs_, sys_.config());
    if (why != nullptr) {
      if (!refused_) {
        say("  REFUSED: %s. Fix it, or type s to skip (r to retry).", why);
        showStep("REFUSED", why);
        refused_ = true;
      }
      return;
    }
    refused_ = false;
    phase_ = 1;
    mark_ = now;
    outputOn_ = false;
    say("  running %s", signalLabel(sig));
  }
  if (phase_ != 1) return;

  const char* abortWhy = rawTbs() ? "TBS switched ON" : vetoReason(sig, inputs_, sys_.config());
  if (abortWhy != nullptr && strcmp(abortWhy, "air supply pressure is low") == 0) {
    const bool inGrace = outputOn_ && firstOnAt_ != 0 && static_cast<uint32_t>(now - firstOnAt_) < cfg_.airGraceMs;   // the drop after EVERY opening is expected (owner 2026-10-09)
    if (airAbortOff_ || inGrace) abortWhy = nullptr;   // observe-only (key `a`), or inside the grace time
  }
  if (inputs_.airX10 < airMinX10_) airMinX10_ = inputs_.airX10;
  if (sig != Signal::kSsr && static_cast<uint32_t>(now - lcdAirMark_) >= 100) {   // the LCD shows the air live: now, and the lowest so far
    lcdAirMark_ = now;
    char l1[24], l2[24], l3[24];
    snprintf(l1, sizeof l1, "%s %s", signalLabel(sig), outputOn_ ? "OPEN" : "closed");
    snprintf(l2, sizeof l2, "AIR  %u.%u PSI", static_cast<unsigned>(inputs_.airX10 / 10u), static_cast<unsigned>(inputs_.airX10 % 10u));
    snprintf(l3, sizeof l3, "min  %u.%u PSI", static_cast<unsigned>(airMinX10_ / 10u), static_cast<unsigned>(airMinX10_ % 10u));
    showStep(l1, l2, l3);
  }
  if (sagTest_ && sig != Signal::kSsr && outputOn_ && static_cast<uint32_t>(now - airLogMark_) >= 50) {   // timed sag test: record 5 s of air
    if (sagN_ == 0) sagOpenAt_ = firstOnAt_ != 0 ? firstOnAt_ : now;
    if (sagN_ < kSagSamples) sagHist_[sagN_++] = inputs_.airX10;
    if (static_cast<uint32_t>(now - sagOpenAt_) >= 5000 || sagN_ >= kSagSamples) {
      outputsOff(now);
      sagSummary(sig, now);
      sagTest_ = false;
      holdOpen_ = false;
      phase_ = 2;
      return;
    }
  }
  if (sig != Signal::kSsr && static_cast<uint32_t>(now - airLogMark_) >= 50) {   // the air pressure while the valve works, every 50 ms
    airLogMark_ = now;
    say("  AIR +%lu ms: raw %u = %u.%u PSI  (N2L raw %u, N2H raw %u)%s", static_cast<unsigned long>(now - stepStart_), static_cast<unsigned>(inputs_.rawAir),
        static_cast<unsigned>(inputs_.airX10 / 10u), static_cast<unsigned>(inputs_.airX10 % 10u), static_cast<unsigned>(inputs_.rawN2Low),
        static_cast<unsigned>(inputs_.rawN2High), inputs_.airX10 < sys_.config().airLowOff ? "  BELOW LIMIT" : "");
  }
  if (abortWhy != nullptr) {  // BIST-11: stop at once
    outputsOff(now);
    ++aborts_;
    say("  ABORTED: %s - all outputs OFF", abortWhy);
    say("  at the abort: AIR raw %u = %u.%u PSI, N2L raw %u, N2H raw %u; limit %u.%u PSI (air before the output: %u.%u PSI)", static_cast<unsigned>(inputs_.rawAir),
        static_cast<unsigned>(inputs_.airX10 / 10u), static_cast<unsigned>(inputs_.airX10 % 10u), static_cast<unsigned>(inputs_.rawN2Low),
        static_cast<unsigned>(inputs_.rawN2High), static_cast<unsigned>(sys_.config().airLowOff / 10u), static_cast<unsigned>(sys_.config().airLowOff % 10u),
        static_cast<unsigned>(airBeforeX10_ / 10u), static_cast<unsigned>(airBeforeX10_ % 10u));
    showStep("ABORTED", abortWhy);
    phase_ = 3;
    return;
  }
  if (sig == Signal::kSsr && cfg_.ssrSinglePulse) {
    if (!pulseDone_) {
      if (!outputOn_) {
        sys_.outputDriver().forceDrive(sig, true, now);
        outputOn_ = true;
        mark_ = now;
        say("  SSR ON (pulse)");
      } else if (static_cast<uint32_t>(now - mark_) >= cfg_.ssrPulseMs) {
        sys_.outputDriver().forceDrive(sig, false, now);
        outputOn_ = false;
        pulseDone_ = true;
        say("  SSR OFF - pulse done; answer p or f (r repeats)");
        phase_ = 2;
      }
    }
    return;
  }
  if (static_cast<uint32_t>(now - stepStart_) >= cfg_.outputStepMaxMs) {
    outputsOff(now);
    say("  stopped after %lu s; answer p or f (r repeats)", static_cast<unsigned long>(cfg_.outputStepMaxMs / 1000));
    phase_ = 2;
    return;
  }
  if (static_cast<uint32_t>(now - mark_) >= cfg_.halfPeriodMs && !(holdOpen_ && outputOn_)) {   // key h: once ON, stay ON
    mark_ = now;
    outputOn_ = !outputOn_;
    sys_.outputDriver().forceDrive(sig, outputOn_, now);
    if (outputOn_) firstOnAt_ = now != 0 ? now : 1;   // every opening starts a new grace time (the name is historic: it is the LAST opening)
    say("  %s %s", signalLabel(sig), outputOn_ ? "ON" : "OFF");
  }
}

void Bist::tick(uint32_t now) {
  switch (step_) {
    case BistStep::kBanner:    break;
    case BistStep::kSwitches:  tickSwitches(now); break;
    case BistStep::kI2c:       tickI2c(now); break;
    case BistStep::kLed:       tickLed(now); break;
    case BistStep::kLcd:       tickLcd(now); break;
    case BistStep::kPressures: tickPressures(now); break;
    case BistStep::kO2:        tickO2(now); break;
    case BistStep::kLeft:
    case BistStep::kRight:
    case BistStep::kFlush:
    case BistStep::kSsr:       tickOutputStep(now, outputSignal()); break;
    case BistStep::kSummary:   break;
    case BistStep::kCount:     break;
  }
}

// ---------------------------------------------------------------------------------------------- one pass
bool Bist::step(uint32_t now) {
  if (!running_) {
    flushOutput();
    if (qCount_ > 0 && !console_.attached()) qCount_ = 0;  // nobody is listening: do not hang waiting to print
    return qCount_ == 0;
  }
  if (!console_.attached()) {  // the operator's console went away: stop safely
      outputsOff(now);
    running_ = false;
    saveRecord(false);
    console_.setLineHook(nullptr);
    display_.clearOverride();
    qCount_ = 0;
    ++aborts_;
    return true;
  }

  sensors_.sample(hal_, now, faults_, inputs_);

  tbsDeb_.update(rawTbs(), now);
  tobDeb_.update(rawTob(), now);
  const bool tob = tobDeb_.level();
  if (cfg_.switchKeys && tob && tobWasUp_ && step_ != BistStep::kSwitches) key_ = Key::kPass;  // TOB = p (bench)
  tobWasUp_ = !tob;
  if (cfg_.switchKeys && tbsDeb_.changed() && tbsDeb_.level() && step_ != BistStep::kSwitches && key_ == Key::kNone) {   // TBS switched ON = f
    key_ = Key::kFail;
    snprintf(keyNote_, sizeof keyNote_, "TBS ON = fail");
  }

  tick(now);

  if (key_ != Key::kNone && running_) {
    const Key k = key_;
    key_ = Key::kNone;
    switch (k) {
      case Key::kPass:  conclude(now, BistVerdict::kPass, ""); break;
      case Key::kFail:  conclude(now, BistVerdict::kFail, keyNote_); break;
      case Key::kSkip:  conclude(now, BistVerdict::kSkip, ""); break;
      case Key::kRerun: outputsOff(now); say("  (rerun)"); enter(now); break;
      case Key::kQuit:
        outputsOff(now);
        printSummary();
        finish(now, "operator quit");
        break;
      case Key::kNone: break;
    }
    keyNote_[0] = '\0';
  }
  flushOutput();
  return !running_ && qCount_ == 0;
}

}  // namespace n2
