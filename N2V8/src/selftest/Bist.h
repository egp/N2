// Bist.h — built-in self-test (Requirements §12, BIST-1..BIST-11). THE main debugging tool.
//
// Interactive and operator-confirmed. It needs an attached console. It runs INSTEAD of normal operation
// (controllers are not stepped), starts only on request, and requires TBS OFF. Output steps are subject to
// the safety invariants as vetoes (BIST-11): they refuse to start, and abort at once with every output OFF, if
// the action would be unsafe.
//
// Non-blocking: call step() once per loop pass (and refresh the watchdog); it returns true when finished.
// The operator answers each step by typing one letter (the console hands lines to onLine()) or pressing TOB:
//   p pass   f [note] fail   r rerun   s skip   q quit        (TOB = p, except in the switch test)
//   g air|n2l|n2h <psi>  enters a production gauge reading in the pressure step
#pragma once

#include <stdint.h>

#include "../core/Debounce.h"
#include "../core/Faults.h"
#include "../core/O2Reader.h"
#include "../core/Sensors.h"
#include "../core/System.h"
#include "../ui/BuildInfo.h"
#include "../ui/Console.h"
#include "../ui/DisplayManager.h"

namespace n2 {

enum class BistStep : uint8_t {
  kBanner, kSwitches, kI2c, kLed, kLcd, kPressures, kO2, kLeft, kRight, kFlush, kSsr, kSummary, kCount
};
constexpr uint8_t kBistSteps = static_cast<uint8_t>(BistStep::kCount);

enum class BistVerdict : uint8_t { kNotRun, kPass, kFail, kSkip };

// What the BIST was doing, kept in RAM that survives a reset-button reset (NOT a power loss, and NOT in NVM: no flash wear). After a
// reset-button reset in the middle of a BIST, the BIST resumes at the step it was on, with the verdicts so far (owner 2026-10-08).
struct BistRecord {
  uint32_t magic;
  uint8_t running;     // 1 while a BIST is in progress
  uint8_t step;        // BistStep it was on
  uint8_t verdict[kBistSteps];
  uint8_t pad[2];
  uint32_t check;
};
constexpr uint32_t kBistRecordMagic = 0x4E324252u;   // "N2BR"
uint32_t bistRecordChecksum(const BistRecord& r);
bool bistRecordValid(const BistRecord& r);

struct BistConfig {
  uint32_t halfPeriodMs = 500;       // output steps toggle at ~2 Hz (valves are audible)
  uint32_t outputStepMaxMs = 10000;  // an output step ends by itself after this long
  bool ssrSinglePulse = true;        // HQ8 pending: one short pulse instead of 2 Hz on the compressor SSR
  uint32_t ssrPulseMs = 1000;
  bool switchKeys = true;            // TOB = pass and TBS ON = fail answer the steps (bench only; production answers from the console)
};

class Bist : public LineHook {
 public:
  enum class Start : uint8_t { kOk, kNoConsole, kTbsOn, kNothingToResume };

  Bist(Hal& hal, const BoardDef& board, System& sys, DisplayManager& display, Console& console, O2Reader& o2,
       const BuildInfo& info, BistConfig config = BistConfig())
      : hal_(hal), board_(board), sys_(sys), display_(display), console_(console), o2_(o2), info_(info), cfg_(config),
        sensors_(board_, sys.config(), kAdcBits), faults_(nullLog_) {}

  // requireTbsOff: true for the `bist` command; false when BIST was requested with TOB at power-up
  // (output steps still need TBS OFF, BIST-4).
  Start begin(uint32_t now, bool requireTbsOff = true);
  // The RAM record that survives a reset (nullptr: no resume). resume() restarts an interrupted BIST at its step; forgetRecord() discards it.
  void setRecord(BistRecord* r) { rec_ = r; }
  // Diagnostic: start directly at one step (bench; used by the N2_BOOT_LCD_TEST build flag so the LCD test runs from power-up).
  Start beginAt(uint32_t now, BistStep step) { const Start s = begin(now, false); if (s == Start::kOk) { step_ = step; enter(now); } return s; }
  Start resume(uint32_t now);
  void forgetRecord() { saveRecord(false); }
  bool step(uint32_t now);  // true when BIST has finished (completed or quit)
  bool running() const { return running_; }

  void onLine(const char* line) override;  // operator input from the console

  BistVerdict verdict(BistStep s) const { return result_[static_cast<uint8_t>(s)].verdict; }
  const char* note(BistStep s) const { return result_[static_cast<uint8_t>(s)].note; }
  BistStep current() const { return step_; }
  uint32_t aborts() const { return aborts_; }

  static const char* stepName(BistStep s);
  // BIST-11: why this output step must NOT run now, or nullptr if it may. `sensors` are fresh readings.
  static const char* vetoReason(Signal output, const Inputs& sensors, const ControlConfig& cfg);

 private:
  enum class Key : uint8_t { kNone, kPass, kFail, kRerun, kSkip, kQuit };
  struct Result {
    BistVerdict verdict = BistVerdict::kNotRun;
    char note[24] = {};
  };

  // output queue (lines wait here until the console has room)
  void say(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  void flushOutput();

  void enter(uint32_t now);
  void tick(uint32_t now);
  void conclude(uint32_t now, BistVerdict v, const char* note);
  void finish(uint32_t now, const char* why);
  void outputsOff(uint32_t now);
  void showStep(const char* line1, const char* line2 = "", const char* line3 = "");
  void tickOutputStep(uint32_t now, Signal sig);
  void tickLed(uint32_t now);
  void tickLcd(uint32_t now);
  void tickPressures(uint32_t now);
  void tickO2(uint32_t now);
  void tickSwitches(uint32_t now);
  void tickI2c(uint32_t now);
  void printSummary();
  bool tbsOn();        // debounced while the BIST runs, raw otherwise
  bool tobPressed();
  bool rawTbs();
  bool rawTob();
  Signal outputSignal() const;

  Hal& hal_;
  const BoardDef& board_;
  System& sys_;
  DisplayManager& display_;
  Console& console_;
  O2Reader& o2_;
  BuildInfo info_;
  BistConfig cfg_;
  NullLogSink nullLog_;
  SensorMonitor sensors_;
  FaultSet faults_;
  Inputs inputs_;

  void saveRecord(bool running);
  BistRecord* rec_ = nullptr;
  bool running_ = false;
  BistStep step_ = BistStep::kBanner;
  Result result_[kBistSteps];
  Key key_ = Key::kNone;
  char keyNote_[24] = {};
  uint32_t stepStart_ = 0;
  uint32_t lastTick_ = 0;
  uint32_t aborts_ = 0;
  bool tobWasUp_ = true;
  int8_t lcdPhase_ = -1;
  uint16_t airBeforeX10_ = 0;   // the air reading when an output step began, to show next to the reading at an abort
  Debouncer tbsDeb_, tobDeb_;   // TBS and TOB as the operator's answer keys: TOB = pass, TBS switched ON = fail (debounced like the system's)

  // step-local state
  uint8_t phase_ = 0;
  uint32_t mark_ = 0;
  uint16_t last_[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
  bool lastBool_[2] = {false, false};
  uint8_t scanAddr_ = 0;
  uint8_t found_[16] = {};
  bool refused_ = false;
  bool outputOn_ = false;
  bool pulseDone_ = false;
  bool gaugeSet_[3] = {false, false, false};
  uint16_t gauge_[3] = {0, 0, 0};

  static constexpr uint8_t kQueue = 16;
  char queue_[kQueue][100];
  uint8_t qHead_ = 0, qCount_ = 0;
  uint32_t queueDrops_ = 0;
};

}  // namespace n2
