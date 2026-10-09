#include "App.h"

#include <stdio.h>
#include <string.h>

#include "../board/BoardSetup.h"

namespace n2 {

namespace {
constexpr uint32_t kConsoleDropGraceMs = 3000;   // see runPass
constexpr uint32_t kPostArmTimeoutMs = 10000;   // TOB held longer than this: start the POST anyway (a stuck button must not stop the unit)
BoardId boardIdOf(const BoardDef& b) {
  const char* n = b.name;
  for (; *n; ++n) {
    if (n[0] == 'W' && n[1] == 'i' && n[2] == 'F' && n[3] == 'i') return BoardId::kWifi;
    if (n[0] == 'h' && n[1] == 'o' && n[2] == 's' && n[3] == 't') return BoardId::kMinima;  // host tests use the production (Minima) pin table
    if (n[0] == 'M' && n[1] == 'i' && n[2] == 'n' && n[3] == 'i' && n[4] == 'm' && n[5] == 'a') return BoardId::kMinima;
  }
  return BoardId::kUnknown;
}
}  // namespace

App::App(Hal& hal, const BoardDef& board, const ControlConfig& cfg, O2Reader& o2, const BuildInfo& info,
         WarmRecord* warmRecord, AppOptions options)
    : hal_(hal),
      board_(board),
      cfg_(cfg),
      info_(info),
      opt_(options),
      console_(hal, options.logLevel),
      rtc_(hal, board.addrRtc),
      sys_(hal, board_, cfg_, o2, console_, kAdcBits),
      display_(hal, board_, options.layout, options.faultCycleMs),
      nvmSvc_(options.nvm, boardIdOf(board), options.sketchVersion, options.debounceDefaultMs),
      ctx_{&sys_, &console_, &loopStats_, &hal_, info_, options.layout, this, &rtc_, &nvmSvc_, &board_, &display_.lcd()},
      commands_(ctx_),
      post_(hal, board_, sys_, display_, console_, info_, ResetInfo(), options.post),
      bist_(hal, board_, sys_, display_, console_, o2, info_, options.bist),
      credit_(warmRecord != nullptr ? *warmRecord : dummyRecord_, options.warmCreditEnabled) {
  bist_.setRecord(options.bistRecord);
}

bool App::tobPressed() {
  const SignalDef& d = def(board_, Signal::kTob);
  return isOn(hal_.digitalRead(d.pin), d.active);
}

// POST and BIST need the system disabled: TBS must be OFF (owner 2026-10-08). While they run the system stays disabled; the BIST tests
// the TBS switch itself without enabling the system, and when either ends the normal TBS rule applies again (TBS ON = enable).
const char* App::requestPost() {
  if (mode_ != Mode::kRun) return "POST refused: the POST or BIST is already running. Wait for it to finish (BIST: type q).";
  if (sys_.inputs().tbs) return "POST refused: the system is enabled (TBS is ON). Switch TBS OFF, then type post.";
  postRequested_ = true;
  return "running POST: the system is disabled until it finishes";
}

const char* App::requestBist() {
  if (mode_ != Mode::kRun) return "BIST refused: the POST or BIST is already running. Wait for it to finish (BIST: type q).";
  if (sys_.inputs().tbs) return "BIST refused: the system is enabled (TBS is ON). Switch TBS OFF, then type bist.";
  bistRequested_ = true;
  return "starting BIST: answer each step with p, f, r, s or q";
}

void App::applyMode(RunMode m) {
  runMode_ = m;
  info_.mode = runModeName(m);       // `ver`, the banner and the BOOT line name the mode in force, not the compiled one
  ctx_.info.mode = runModeName(m);
  sys_.setControllersEnabled(m != RunMode::kDiag);
  sys_.setO2Mandatory(m == RunMode::kField);
  console_.setLevel(m == RunMode::kField ? LogLevel::kInfo : opt_.logLevel);
  if (opt_.modeRecord != nullptr) {
    opt_.modeRecord->magic = kModeRecordMagic;
    opt_.modeRecord->mode = static_cast<uint32_t>(m);
    opt_.modeRecord->check = modeRecordChecksum(*opt_.modeRecord);
  }
}

// `mode`: show it, or change it. DIAG is immediate and always safe. BENCH and FIELD enable the controllers: only from the console, only with
// TBS OFF, and only with the word `confirm`. Nothing starts by itself afterwards: the normal TBS rule applies.
const char* App::requestMode(const char* name, bool confirmed) {
  static char msg[130];
  if (name == nullptr) {
    snprintf(msg, sizeof msg, "mode %s: controllers %s, O2 %s. (mode diag|bench|field)", runModeName(runMode_),
             sys_.controllersEnabled() ? "ON" : "OFF", sys_.config().o2Mandatory ? "mandatory" : "optional");
    return msg;
  }
  RunMode m;
  if (strcmp(name, "diag") == 0) m = RunMode::kDiag;
  else if (strcmp(name, "bench") == 0) m = RunMode::kBench;
  else if (strcmp(name, "field") == 0) m = RunMode::kField;
  else return "usage: mode [diag | bench confirm | field confirm]";
  if (m == runMode_) return "already in that mode";
  if (mode_ != Mode::kRun) return "mode change refused: the POST or BIST is running. Wait for it to finish.";
  if (m != RunMode::kDiag) {
    if (sys_.inputs().tbs) return "mode change refused: the system is enabled (TBS is ON). Switch TBS OFF first.";
    if (!confirmed) {
      snprintf(msg, sizeof msg, "mode %s ENABLES the controllers. Type  mode %s confirm  (TBS must be OFF).", runModeName(m) , name);
      return msg;
    }
  }
  applyMode(m);
  sys_.resume();   // controllers start disabled; every output is off now
  logf(console_, LogLevel::kInfo, "%lu MODE %s (from the console)", static_cast<unsigned long>(hal_.millis()), runModeName(m));
  snprintf(msg, sizeof msg, "mode %s. Controllers %s; outputs are off%s.", runModeName(m), m == RunMode::kDiag ? "OFF" : "ON",
           m == RunMode::kDiag ? "" : " until TBS is switched ON");
  return msg;
}

void App::setup() {
  const uint32_t now = hal_.millis();
  beginHardware(hal_, board_);  // RST-2: outputs safe first, then ADC and I2C
  hal_.consoleBegin();          // the R4 WiFi core does not start Serial for us (CON-4)
  if (opt_.watchdogEnabled) hal_.watchdogBegin(opt_.watchdogMs);

  resetInfo_ = hal_.readResetCause();
  {   // the run mode: the compiled one after a power-up, the remembered one after a reset-button or watchdog reset (the RAM breadcrumb)
    const RunMode compiled = !opt_.controllersEnabled ? RunMode::kDiag : (sys_.config().o2Mandatory ? RunMode::kField : RunMode::kBench);
    RunMode m = compiled;
    const bool keeps = resetInfo_.known && !resetInfo_.powerOn && !resetInfo_.brownout;   // button or watchdog
    if (keeps && opt_.modeRecord != nullptr && modeRecordValid(*opt_.modeRecord)) m = static_cast<RunMode>(opt_.modeRecord->mode);
    applyMode(m);
    if (m != compiled) logf(console_, LogLevel::kInfo, "%lu MODE %s restored after a reset (compiled: %s)", static_cast<unsigned long>(now), runModeName(m), runModeName(compiled));
  }
  const uint32_t credit = credit_.begin(resetInfo_, now);
  display_.setLcdMinChangeMs(opt_.lcdMinChangeMs);
  nvmSvc_.load();  // NVM-1: read only. The stored debounce times (if valid for this board) replace the compiled default.
  sys_.setDebounce(nvmSvc_.choice().tbsMs, nvmSvc_.choice().tobMs);
  sys_.begin(resetInfo_, credit);
  post_.setResetInfo(resetInfo_);
  display_.begin(now, opt_.lcdStartMs);
  tobAtBoot_ = tobPressed();  // TOB held at power-up or reset selects POST mode (RST-6, POST-1); a normal boot runs no self-test

  logf(console_, LogLevel::kInfo, "%lu BOOT N2V8 %s %s mode %s reset: %s", static_cast<unsigned long>(now), info_.version,
       info_.board, info_.mode,
       !resetInfo_.known ? "unknown" : (resetInfo_.powerOn ? "power-on" : (resetInfo_.watchdog ? "watchdog" : (resetInfo_.brownout ? "brown-out" : "reset button"))));
  if (nvmSvc_.available()) {
    const StoreReport& r = nvmSvc_.report();
    logf(console_, LogLevel::kInfo, "%lu NVM debounce TBS %u ms TOB %u ms: %s%s", static_cast<unsigned long>(now),
         static_cast<unsigned>(nvmSvc_.choice().tbsMs), static_cast<unsigned>(nvmSvc_.choice().tobMs), nvmSvc_.choice().why,
         nvmSvc_.choice().source == DebounceSource::kStored ? (r.settings.sketchVersion == nvmSvc_.sketchVersion() ? "" : " (written by another sketch version: see `nvm`)") : "");
    if (r.wearWarning()) logf(console_, LogLevel::kWarn, "%lu NVM write count %lu is high (rated life about 200000)", static_cast<unsigned long>(now), static_cast<unsigned long>(r.sequence));
  }
  if (tobAtBoot_) {
    // ACKNOWLEDGE the TOB: the LED shows PoSt at once; the LCD shows it when it starts (2.5 s after boot). POST begins when TOB is
    // released (or after 10 s), so the operator knows when to let go.
    logf(console_, LogLevel::kInfo, "%lu POST mode: TOB seen. Release TOB to start the POST", static_cast<unsigned long>(now));
    display_.setOverride(makeScreen("POST MODE", "TOB held at start", "RELEASE TOB", "to begin the POST"), LedText{{'P', 'o', 'S', 't', '\0'}, -1}, now);
    postArmStart_ = now;
    tobReleased_ = false;
    mode_ = Mode::kPostArm;
  } else {
#if defined(N2_BOOT_LCD_TEST)
    mode_ = bist_.beginAt(now, BistStep::kLcd) == Bist::Start::kOk ? Mode::kBist : Mode::kRun;   // bench diagnostic build only
#else
    mode_ = Mode::kRun;   // normal boot: outputs are safe, the sensor rules and invariants protect the machine as always
#endif
    // A reset-button reset in the middle of a BIST resumes it at the same step. Not after a power loss, a watchdog or a brown-out.
    const bool buttonReset = resetInfo_.known && !resetInfo_.powerOn && !resetInfo_.watchdog && !resetInfo_.brownout;
    if (buttonReset) {
      if (bist_.resume(now) == Bist::Start::kOk) mode_ = Mode::kBist;
    } else {
      bist_.forgetRecord();
    }
  }
}

// When a console attaches (the Serial Monitor is opened) it missed the boot messages: say who we are.
void App::printBanner() {
  char line[100];
  snprintf(line, sizeof line, "N2V8 %s  built %s %s  board %s  mode %s  adc %u bits", info_.version, info_.date, info_.time, info_.board,
           info_.mode, static_cast<unsigned>(info_.adcBits));
  console_.tryPrint(line);
  DateTime clock;
  bool clockValid = false;
  if (rtc_.read(clock)) {
    char when[24];
    formatDateTime(when, clock);
    rtc_.timeValid(clockValid);
    snprintf(line, sizeof line, "RTC %s%s", when, clockValid ? "" : "  (NOT trusted: set it with: time set YYYY-MM-DD HH:MM:SS)");
  } else {
    snprintf(line, sizeof line, "RTC: unavailable");
  }
  console_.tryPrint(line);
  snprintf(line, sizeof line, "up %lu s, state %s. Type help.", static_cast<unsigned long>(hal_.millis() / 1000u),
           mode_ == Mode::kPostArm ? "POST (release TOB)" : (mode_ == Mode::kPost ? "POST" : (mode_ == Mode::kBist ? "BIST" : "RUN")));
  console_.tryPrint(line);
}

// Tell a newly attached console who we are. On the R4 WiFi no attach can be seen (the UART cannot see the PC), so the
// banner is simply repeated every 10 s until the first byte arrives from the host.
void App::announceConsole() {
  const uint32_t now = hal_.millis();
  const bool attached = hal_.consoleAttached();
  if (attached && !consoleWasAttached_) printBanner();
  consoleWasAttached_ = attached;
  if (attached && !hal_.consoleCanDetectHost() && console_.received() == 0 && static_cast<int32_t>(now - nextBanner_) >= 0) {
    nextBanner_ = now + 10000;
    printBanner();
  }
}

void App::runPass(uint32_t now) {
  sys_.step();
  DisplayData data = makeDisplayData(sys_, kAdcBits);
  if (runMode_ != RunMode::kField) data.version = info_.version;   // the version on the LCD while testing (not in FIELD)
  display_.showNormal(data, now);

  // Report what the display side noticed (DSP-6: informational, never stops the system).
  FaultSet& f = sys_.faultSet();
  f.report(FaultId::kLcd, !display_.lcdHealthy(), now, cfg_.faultHoldMs);
  f.report(FaultId::kLed, !display_.ledHealthy(), now, cfg_.faultHoldMs);
  // The R4 WiFi's serial buffer cannot take the boot messages in one burst, so a line or two are dropped at start-up. That has a known
  // cause and is not a fault: drops in the first kConsoleDropGraceMs after boot are counted (`status`) but do not raise F40.
  const bool dropping = console_.dropped() != lastDropped_ && now >= kConsoleDropGraceMs;
  lastDropped_ = console_.dropped();
  f.report(FaultId::kConsoleDrop, dropping, now, cfg_.faultHoldMs);

  // The real-time clock is optional and informational: check it now and then (F13, INFO, never affects control).
  if (static_cast<int32_t>(now - nextRtcCheck_) >= 0) {
    nextRtcCheck_ = now + 10000;
    bool valid = false;
    const bool ok = rtc_.present() && rtc_.timeValid(valid);
    f.report(FaultId::kRtc, !ok || !valid, now, cfg_.faultHoldMs);
  }

  // Keep the warm-up record current; a sensor failure means it may have lost power (O2-6a).
  const bool o2Error = sys_.o2().state() == O2Controller::State::kError;
  if (o2Error && !o2WasError_) credit_.restart(now);
  o2WasError_ = o2Error;
}

void App::loop() {
  const uint32_t us0 = hal_.micros();
  const uint32_t now = hal_.millis();
  hal_.watchdogRefresh();

  console_.poll(commands_);
  announceConsole();

  switch (mode_) {
    case Mode::kPostArm:
      if (tobPressed()) {
        tobReleased_ = false;
      } else if (!tobReleased_) {
        tobReleased_ = true;
        tobReleasedAt_ = now;
      }
      if ((tobReleased_ && static_cast<uint32_t>(now - tobReleasedAt_) >= 50) || static_cast<uint32_t>(now - postArmStart_) >= kPostArmTimeoutMs) {
        if (!tobReleased_) logf(console_, LogLevel::kWarn, "%lu TOB still held after %lu s: starting the POST anyway", static_cast<unsigned long>(now), static_cast<unsigned long>(kPostArmTimeoutMs / 1000u));
        post_.begin(now);
        mode_ = Mode::kPost;
      }
      break;

    case Mode::kPost:
      if (post_.step(now)) {
        sys_.resume();
        mode_ = Mode::kRun;
      }
      break;

    case Mode::kBist:
      if (bist_.step(now)) {
        sys_.resume();
        mode_ = Mode::kRun;
      }
      break;

    case Mode::kRun:
      if (postRequested_ && sys_.inputs().tbs) {   // TBS went ON between the command and this pass
        postRequested_ = false;
        console_.tryPrint("POST cancelled: TBS is ON");
      } else if (postRequested_) {
        postRequested_ = false;
        sys_.resume();
        post_.begin(now);
        mode_ = Mode::kPost;
      } else if (bistRequested_) {
        bistRequested_ = false;
        if (bist_.begin(now, true) == Bist::Start::kOk) {  // a refused request must not disturb the running system
          sys_.resume();
          mode_ = Mode::kBist;
        }
      } else {
        runPass(now);
      }
      break;
  }

  display_.service(now);
  if (static_cast<uint32_t>(now - lastCreditTick_) >= 1000) {
    lastCreditTick_ = now;
    credit_.tick(now);
  }
  loopStats_.record(static_cast<uint32_t>(hal_.micros() - us0));
}

}  // namespace n2
