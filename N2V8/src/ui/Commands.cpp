#include "Commands.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../core/Scaling.h"
#include "LedText.h"

namespace n2 {

namespace { uint32_t rtcSecondsOf(Rtc3231* rtc); }

Commands::Commands(const ConsoleContext& ctx)
    : c_(ctx), ver_(c_), time_(c_), status_(c_), faults_(c_), cfg_(c_), nvmInfo_(c_), pins_(c_), display_(c_), loop_(c_), scan_(c_), report_(*this) {}

Responder* Commands::handle(const Command& cmd) {
  switch (cmd.id) {
    case CommandId::kNone:    return nullptr;
    case CommandId::kHelp:    return &help_;
    case CommandId::kVer:     return &ver_;
    case CommandId::kStatus:  return &status_;
    case CommandId::kFaults:  return &faults_;
    case CommandId::kCfg:     return &cfg_;
    case CommandId::kDisplay: return &display_;
    case CommandId::kTime: {
      if (cmd.argc == 0) return &time_;
      if (c_.rtc == nullptr) {
        message_.set("no real-time clock in this build");
        return &message_;
      }
      DateTime t;
      if (cmd.argc == 3 && strcmp(cmd.arg[0], "set") == 0 && parseDateTime(cmd.arg[1], cmd.arg[2], t)) {
        if (c_.rtc->set(t)) {
          char buf[24];
          formatDateTime(buf, t);
          message_.set("RTC set to %s", buf);
        } else {
          message_.set("could not write the RTC (no answer on the I2C bus)");
        }
      } else {
        message_.set("usage: time set YYYY-MM-DD HH:MM:SS   (24-hour clock)");
      }
      return &message_;
    }
    case CommandId::kNvm:     return &nvmInfo_;
    case CommandId::kMode:
      if (c_.launcher == nullptr) { message_.set("'mode' is not available in this build"); return &message_; }
      message_.set("%s", c_.launcher->requestMode(cmd.argc > 0 ? cmd.arg[0] : nullptr, cmd.argc > 1 && strcmp(cmd.arg[1], "confirm") == 0));
      return &message_;
    case CommandId::kLcd: {
      if (c_.lcd == nullptr) { message_.set("no LCD driver in this build"); return &message_; }
      Lcd20x4& l = *c_.lcd;
      if (cmd.argc == 0) {
        message_.set("LCD: %s, healthy %s, I2C errors %lu, re-inits %lu, bus recoveries %lu, content %s, backlight %s, display %s",
                     l.ready() ? "ready" : "not ready", l.healthy() ? "yes" : "NO", static_cast<unsigned long>(l.i2cErrors()),
                     static_cast<unsigned long>(l.reinitCount()), static_cast<unsigned long>(l.busRecoveries()), l.inSync() ? "in sync" : "being written",
                     l.backlightOn() ? "on" : "off", l.displayOn() ? "on" : "off");
      } else if (strcmp(cmd.arg[0], "reinit") == 0) {
        l.reinit(c_.hal->millis());
        message_.set("LCD: controller restarted (full initialisation; the display clears and redraws)");
      } else if (strcmp(cmd.arg[0], "bus") == 0) {
        const long rounds = cmd.argc > 1 ? strtol(cmd.arg[1], nullptr, 10) : 50;
        const Lcd20x4::BusTest t = l.busTest(static_cast<uint16_t>(rounds < 1 ? 1 : (rounds > 500 ? 500 : rounds)));
        message_.set("LCD bus test: %u rounds, write failures %u, read failures %u, mismatches %u (%lu us per round)", static_cast<unsigned>(t.rounds),
                     static_cast<unsigned>(t.writeFailed), static_cast<unsigned>(t.readFailed), static_cast<unsigned>(t.mismatched), static_cast<unsigned long>(t.microsPerRound));
      } else {
        message_.set("usage: lcd | lcd reinit | lcd bus [rounds]");
      }
      return &message_;
    }
    case CommandId::kPins:    return c_.board != nullptr && c_.hal != nullptr ? static_cast<Responder*>(&pins_) : nullptr;
    case CommandId::kDebounce: {
      NvmSettingsService* nv = c_.nvm;
      if (cmd.argc == 0) {
        message_.set("debounce TBS %u ms, TOB %u ms (%s)", static_cast<unsigned>(c_.sys->tbsDebounceMs()),
                     static_cast<unsigned>(c_.sys->tobDebounceMs()),
                     nv != nullptr ? nv->choice().why : "compiled default");
      } else if (cmd.argc == 3 && strcmp(cmd.arg[0], "set") == 0) {
        const long a = strtol(cmd.arg[1], nullptr, 10), b = strtol(cmd.arg[2], nullptr, 10);
        if (nv == nullptr || !nv->available()) message_.set("no non-volatile memory in this build");
        else if (a < kMinDebounceMs || a > kMaxDebounceMs || b < kMinDebounceMs || b > kMaxDebounceMs) message_.set("times must be %u..%u ms", kMinDebounceMs, kMaxDebounceMs);
        else if (c_.sys->inputs().tbs) message_.set("switch TBS OFF first: saving stalls the program for about 50 ms");
        else if (!nv->saveDebounce(static_cast<uint8_t>(a), static_cast<uint8_t>(b), rtcSecondsOf(c_.rtc))) message_.set("SAVE FAILED (see nvm)");
        else {
          c_.sys->setDebounce(nv->choice().tbsMs, nv->choice().tobMs);
          message_.set("saved: debounce TBS %u ms, TOB %u ms, write count %lu", static_cast<unsigned>(nv->choice().tbsMs),
                       static_cast<unsigned>(nv->choice().tobMs), static_cast<unsigned long>(nv->report().sequence));
        }
      } else {
        message_.set("usage: debounce | debounce set TBS_MS TOB_MS   (2..100 ms, TBS must be OFF)");
      }
      return &message_;
    }
    case CommandId::kScan:    return &scan_;
    case CommandId::kReport:  return &report_;
    case CommandId::kLoop:
      if (cmd.argc > 0 && strcmp(cmd.arg[0], "reset") == 0) {
        c_.loop->reset();
        message_.set("loop statistics reset");
        return &message_;
      }
      return &loop_;
    case CommandId::kLog: {
      LogLevel level;
      if (cmd.argc == 0) {
        message_.set("log level = %s", Console::levelName(c_.console->level()));
      } else if (Console::parseLevel(cmd.arg[0], level)) {
        c_.console->setLevel(level);
        message_.set("log level = %s", Console::levelName(level));
      } else {
        message_.set("usage: log error|warn|info|debug");
      }
      return &message_;
    }
    case CommandId::kPost:
    case CommandId::kBist:
      if (c_.launcher != nullptr) {
        message_.set("%s", cmd.id == CommandId::kPost ? c_.launcher->requestPost() : c_.launcher->requestBist());
        return &message_;
      }
      message_.set("'%s' is not available in this build", cmd.name);
      return &message_;
    case CommandId::kUnknown:
      message_.set("unknown command '%s' - try help", cmd.name);
      return &message_;
  }
  return nullptr;
}

// ---------------------------------------------------------------------------------------------
void Commands::Message::set(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vsnprintf(text_, sizeof text_, fmt, args);
  va_end(args);
}
bool Commands::Message::line(uint8_t i, char* b, size_t n) {
  if (i != 0) return false;
  snprintf(b, n, "%s", text_);
  return true;
}

// ---------------------------------------------------------------------------------------------
bool Commands::Help::line(uint8_t i, char* b, size_t n) {
  static const char* const kLines[] = {
      "commands (case-insensitive):",
      "  help               this list",
      "  ver                firmware version and build",
      "  status             inputs, states, outputs, faults",
      "  faults             active faults",
      "  cfg                thresholds and timings",
      "  display            what the LCD and LED should show",
      "  lcd [reinit|bus]   LCD driver state; reinit = restart its controller; bus = I2C link test",
      "  mode [diag|bench|field [confirm]]  run mode: diag = controllers off; bench/field need TBS OFF and the word confirm",
      "  pins               every pin: which signal, its level now / raw volts (verify the wiring)",
      "  nvm                non-volatile memory: what is stored, each check, write count",
      "  debounce [set T B] TBS/TOB debounce ms: show, or save (TBS off)",
      "  time [set ...]     real-time clock: show it, or: time set YYYY-MM-DD HH:MM:SS",
      "  loop [reset]       loop() timing: min, mean, median, max",
      "  log <level>        error | warn | info | debug",
      "  scan               I2C scan",
      "  report             everything above in one block",
      "  post               run the power-on self-test (needs TBS OFF; the system is disabled while it runs)",
      "  bist               interactive self-test (needs TBS OFF; TBS is tested without enabling the system)",
  };
  if (i >= sizeof kLines / sizeof kLines[0]) return false;
  snprintf(b, n, "%s", kLines[i]);
  return true;
}

bool Commands::Ver::line(uint8_t i, char* b, size_t n) {
  switch (i) {
    case 0: snprintf(b, n, "N2V8 %s", c_.info.version); return true;
    case 1: snprintf(b, n, "built %s %s", c_.info.date, c_.info.time); return true;
    case 2: snprintf(b, n, "board %s  mode %s  adc %u bits", c_.info.board, c_.info.mode, static_cast<unsigned>(c_.info.adcBits)); return true;
  }
  return false;
}

namespace {
void sensorLine(char* b, size_t n, const char* name, uint16_t raw, uint8_t bits, const char* value, bool ok) {
  const unsigned mv = millivoltsFromRaw(raw, bits);
  snprintf(b, n, "%-4s raw %5u %u.%02uV %s PSI %s", name, static_cast<unsigned>(raw), mv / 1000u, (mv % 1000u) / 10u, value,
           ok ? "ok" : "FAULT");
}
}  // namespace

bool Commands::Time::line(uint8_t i, char* b, size_t n) {
  Rtc3231* rtc = c_.rtc;
  if (rtc == nullptr) {
    if (i != 0) return false;
    snprintf(b, n, "RTC: none in this build");
    return true;
  }
  if (i == 0) {
    DateTime t;
    bool valid = false;
    char when[24];
    if (!rtc->read(t)) {
      snprintf(b, n, "RTC: no valid answer from the clock chip (missing, or it holds no valid time)");
    } else {
      formatDateTime(when, t);
      const bool known = rtc->timeValid(valid);
      snprintf(b, n, "RTC %s (%s)", when, !known ? "trust unknown" : (valid ? "trusted" : "NOT trusted: it lost power; set it with: time set ..."));
    }
    return true;
  }
  if (i == 1) {
    int16_t c;
    if (!rtc->temperatureX100(c)) return false;
    const int a = c < 0 ? -c : c;
    snprintf(b, n, "RTC chip temperature %s%d.%02d C", c < 0 ? "-" : "", a / 100, a % 100);
    return true;
  }
  return false;
}

bool Commands::Status::line(uint8_t i, char* b, size_t n) {
  const System& s = *c_.sys;
  const Inputs& in = s.inputs();
  char v[8];
  switch (i) {
    case 0:
      snprintf(b, n, "up %lu s  TBS %s  TOB %s", static_cast<unsigned long>(in.ms / 1000u), in.tbs ? "ON" : "off",
               in.tob ? "pressed" : "off");
      return true;
    case 1: formatX10(v, in.airX10);     sensorLine(b, n, "AIR", in.rawAir, c_.info.adcBits, v, in.airOk); return true;
    case 2: formatX100(v, in.n2LowX100); sensorLine(b, n, "N2L", in.rawN2Low, c_.info.adcBits, v, in.n2LowOk); return true;
    case 3: formatX10(v, in.n2HighX10);  sensorLine(b, n, "N2H", in.rawN2High, c_.info.adcBits, v, in.n2HighOk); return true;
    case 4: {
      char w[8];
      formatCountdown(w, s.o2().warmRemainingMs(in.ms));
      snprintf(b, n, "TWR %s  CMP %s  O2 %s  warm-up left%s", Tower::name(s.tower().state()),
               Compressor::name(s.compressor().state()), O2Controller::name(s.o2().state()), w);
      return true;
    }
    case 5:
      if (s.o2().n2Valid()) {
        formatX100(v, s.o2().n2PercentX100());
        snprintf(b, n, "N2 %s %%%s", v, s.o2().n2Stale() ? " (stale)" : "");
      } else {
        snprintf(b, n, "N2 --.-- %% (no valid reading)");
      }
      return true;
    case 6: {
      const OutputRequest o = s.outputs().actualState();
      snprintf(b, n, "OUT L=%d R=%d F=%d S=%d   (min-hold deferrals: %lu)", o.left, o.right, o.flush, o.ssr,
               static_cast<unsigned long>(s.outputs().deferredCount()));
      return true;
    }
    case 7: {
      const uint8_t k = s.faults().activeCount();
      if (k == 0) {
        snprintf(b, n, "FAULTS none");
      } else {
        int len = snprintf(b, n, "FAULTS %u:", static_cast<unsigned>(k));
        FaultId id;
        for (uint8_t j = 0; s.faults().nth(j, Severity::kInfo, id) && len > 0 && static_cast<size_t>(len) < n; ++j)
          len += snprintf(b + len, n - static_cast<size_t>(len), " F%02X", static_cast<unsigned>(faultInfo(id).code));
      }
      return true;
    }
    case 8:
      snprintf(b, n, "INVARIANT violations %lu   console lines dropped %lu", static_cast<unsigned long>(s.invariantViolations()),
               static_cast<unsigned long>(c_.console->dropped()));
      return true;
  }
  return false;
}

bool Commands::Faults::line(uint8_t i, char* b, size_t n) {
  FaultId id;
  const FaultSet& f = c_.sys->faults();
  if (f.activeCount() == 0) {
    if (i != 0) return false;
    snprintf(b, n, "no active faults");
    return true;
  }
  if (!f.nth(i, Severity::kInfo, id)) return false;
  const FaultInfo& info = faultInfo(id);
  snprintf(b, n, "F%02X %-16s %-7s %s", static_cast<unsigned>(info.code), info.text,
           info.severity == Severity::kInhibit ? "INHIBIT" : (info.severity == Severity::kWarn ? "WARNING" : "INFO"),
           info.effect);
  return true;
}

namespace {
// RTC time now as seconds since 2026-01-01, or 0 when there is no clock or it does not hold a valid time.
uint32_t rtcSecondsOf(Rtc3231* rtc) {
  DateTime t;
  bool valid = false;
  if (rtc == nullptr || !rtc->read(t) || !rtc->timeValid(valid) || !valid) return 0;
  return secondsSince2026(t);
}
const char* okFail(bool ok) { return ok ? "ok" : "FAIL"; }
void copyLine(char* b, size_t n, char which, const RecordChecks& k, RecordStatus st, uint32_t seq) {
  if (st == RecordStatus::kOk) snprintf(b, n, "copy %c: magic ok  schema ok  length ok  checksum ok  write count %lu", which, static_cast<unsigned long>(seq));
  else snprintf(b, n, "copy %c: magic %s  schema %s  length %s  checksum %s  -> %s", which, okFail(k.magic), okFail(k.schema), okFail(k.length), okFail(k.crc), recordStatusName(st));
}
}  // namespace

bool Commands::NvmInfo::line(uint8_t i, char* b, size_t n) {
  const NvmSettingsService* nv = c_.nvm;
  if (nv == nullptr || !nv->available()) {
    if (i != 0) return false;
    snprintf(b, n, "NVM: none in this build (the compiled debounce default is used)");
    return true;
  }
  const StoreReport& r = nv->report();
  switch (i) {
    case 0: snprintf(b, n, "NVM %u B, block %u; A at 0x0000, B at 0x%04X; board %u; sketch 0x%04X",
                     static_cast<unsigned>(nv->nvmSize()), static_cast<unsigned>(nv->nvmBlock()), static_cast<unsigned>(nv->nvmBlock()),
                     static_cast<unsigned>(nv->board()), static_cast<unsigned>(nv->sketchVersion())); return true;
    case 1: copyLine(b, n, 'A', r.checksA, r.a, r.seqA); return true;
    case 2: copyLine(b, n, 'B', r.checksB, r.b, r.seqB); return true;
    case 3:
      if (!r.valid) snprintf(b, n, "in use: nothing valid stored (a never-set memory fails the magic and the checksum)");
      else snprintf(b, n, "in use: copy %c  WRITE COUNT %lu%s  stored by sketch 0x%04X  TBS %u ms  TOB %u ms  board %u", r.which,
                    static_cast<unsigned long>(r.sequence), r.wearWarning() ? " (WEAR WARNING)" : " (of about 200000 rated)",
                    static_cast<unsigned>(r.settings.sketchVersion), static_cast<unsigned>(r.settings.tbsDebounceMs),
                    static_cast<unsigned>(r.settings.tobDebounceMs), static_cast<unsigned>(r.settings.board));
      return true;
    case 4: {
      if (!r.valid) snprintf(b, n, "saved: never");
      else if (r.settings.savedAtSec == 0) snprintf(b, n, "saved: time unknown (the RTC was not valid)");
      else {
        DateTime t;
        char when[24] = "?";
        if (dateTimeFromSecondsSince2026(r.settings.savedAtSec, t)) formatDateTime(when, t);
        const uint32_t nowS = rtcSecondsOf(c_.rtc);
        if (nowS >= r.settings.savedAtSec && nowS != 0) {
          const uint32_t age = nowS - r.settings.savedAtSec;
          if (age >= 2 * 86400u) snprintf(b, n, "saved %s (%lu days ago)", when, static_cast<unsigned long>(age / 86400u));
          else if (age >= 7200u) snprintf(b, n, "saved %s (%lu hours ago)", when, static_cast<unsigned long>(age / 3600u));
          else snprintf(b, n, "saved %s (%lu min ago)", when, static_cast<unsigned long>(age / 60u));
        } else {
          snprintf(b, n, "saved %s (age unknown: RTC not valid now)", when);
        }
      }
      return true;
    }
    case 5: snprintf(b, n, "debounce now: TBS %u ms, TOB %u ms (%s)", static_cast<unsigned>(nv->choice().tbsMs),
                     static_cast<unsigned>(nv->choice().tobMs), nv->choice().why); return true;
  }
  return false;
}

// One line per pin D0..D13 and A0..A5, in order, from the board table. Reads only; it changes no pin mode and drives no output.
bool Commands::Pins::line(uint8_t i, char* b, size_t n) {
  if (c_.board == nullptr || c_.hal == nullptr) return false;   // no pin table in this context (tests); the report skips it
  if (i == 0) {
    snprintf(b, n, "pins of %s (level now; analog = raw / volts). Move a switch or a sensor and ask again to find its pin.", c_.board->name);
    return true;
  }
  const uint8_t p = static_cast<uint8_t>(i - 1);
  if (p > pin::kA5) return false;
  char pn[6];
  if (pin::isAnalog(p)) snprintf(pn, sizeof pn, "A%u", static_cast<unsigned>(p - pin::kA0));
  else snprintf(pn, sizeof pn, "D%u", static_cast<unsigned>(p));
  if (p == c_.board->sdaPin || p == c_.board->sclPin) {
    snprintf(b, n, "%-3s I2C %s", pn, p == c_.board->sdaPin ? "SDA" : "SCL");
    return true;
  }
  if (c_.board->ledSdaPin != pin::kNoPin && (p == c_.board->ledSdaPin || p == c_.board->ledSclPin)) {
    snprintf(b, n, "%-3s LED bus (software I2C)", pn);
    return true;
  }
  const SignalDef* d = nullptr;
  for (uint8_t s = 0; s < c_.board->signalCount; ++s)
    if (c_.board->signals[s].pin == p) d = &c_.board->signals[s];
  if (d == nullptr) {
    if (pin::isAnalog(p)) {
      const uint16_t raw = c_.hal->analogRead(p);
      const unsigned mv = millivoltsFromRaw(raw, kAdcBits);
      snprintf(b, n, "%-3s (unassigned) raw %u  %u.%02uV  (floating pins wander)", pn, static_cast<unsigned>(raw), mv / 1000u, (mv % 1000u) / 10u);
    } else {
      snprintf(b, n, "%-3s (unassigned)", pn);
    }
    return true;
  }
  if (d->dir == Dir::kAnalogInput) {
    const uint16_t raw = c_.hal->analogRead(p);
    const unsigned mv = millivoltsFromRaw(raw, kAdcBits);
    snprintf(b, n, "%-3s %-7s analog in   raw %u  %u.%02uV", pn, d->name, static_cast<unsigned>(raw), mv / 1000u, (mv % 1000u) / 10u);
  } else {
    const bool high = c_.hal->digitalRead(p);
    snprintf(b, n, "%-3s %-7s %-11s %s -> %s", pn, d->name, d->dir == Dir::kOutput ? "output" : (d->dir == Dir::kInputPullup ? "in (pull-up)" : "input"),
             high ? "HIGH" : "LOW ", isOn(high, d->active) ? "ON" : "off");
  }
  return true;
}

bool Commands::Cfg::line(uint8_t i, char* b, size_t n) {
  const ControlConfig& c = c_.sys->config();
  char v[8];
  switch (i) {
    case 0:  snprintf(b, n, "airLowOff   %u  (%s PSI)", c.airLowOff, (formatX10(v, c.airLowOff), v)); return true;
    case 1:  snprintf(b, n, "airLowOn    %u  (%s PSI)", c.airLowOn, (formatX10(v, c.airLowOn), v)); return true;
    case 2:  snprintf(b, n, "towerFill   %lu ms", static_cast<unsigned long>(c.towerFillMs)); return true;
    case 3:  snprintf(b, n, "towerOverlap %lu ms", static_cast<unsigned long>(c.towerOverlapMs)); return true;
    case 4:  snprintf(b, n, "n2LowOff    %u  (%s PSI)", c.n2LowOff, (formatX100(v, c.n2LowOff), v)); return true;
    case 5:  snprintf(b, n, "n2LowOn     %u  (%s PSI)", c.n2LowOn, (formatX100(v, c.n2LowOn), v)); return true;
    case 6:  snprintf(b, n, "n2HighOn    %u  (%s PSI)", c.n2HighOn, (formatX10(v, c.n2HighOn), v)); return true;
    case 7:  snprintf(b, n, "n2HighOff   %u  (%s PSI)", c.n2HighOff, (formatX10(v, c.n2HighOff), v)); return true;
    case 8:  snprintf(b, n, "o2Interval  %lu ms  flush %lu ms  sample %lu ms x %u", static_cast<unsigned long>(c.o2SampleIntervalMs),
                      static_cast<unsigned long>(c.o2FlushMs), static_cast<unsigned long>(c.o2SampleMs), static_cast<unsigned>(c.o2SampleCount)); return true;
    case 9:  snprintf(b, n, "o2Comm      retry %lu ms  timeout %lu ms  error-retry %lu ms", static_cast<unsigned long>(c.o2CommRetryMs),
                      static_cast<unsigned long>(c.o2CommTimeoutMs), static_cast<unsigned long>(c.o2ErrorRetryMs)); return true;
    case 10: snprintf(b, n, "o2Warmup    %lu ms  mandatory %s", static_cast<unsigned long>(c.o2WarmupMs), c.o2Mandatory ? "yes" : "no"); return true;
    case 11: snprintf(b, n, "outputMinHold %lu ms", static_cast<unsigned long>(c.outputMinHoldMs)); return true;
    case 12: snprintf(b, n, "sensorFault %u samples  clear hold %lu ms", static_cast<unsigned>(c.sensorFaultSamples), static_cast<unsigned long>(c.faultHoldMs)); return true;
    case 13: snprintf(b, n, "sensorOrder margin %u (PSI x100)  hold %lu ms", c.sensorOrderMarginX100, static_cast<unsigned long>(c.sensorOrderHoldMs)); return true;
  }
  return false;
}

bool Commands::Display::line(uint8_t i, char* b, size_t n) {
  const DisplayData d = makeDisplayData(*c_.sys, c_.info.adcBits);
  const Screen normal = renderNormal(d, c_.layout);
  if (i == 0) { snprintf(b, n, "LCD normal screen:"); return true; }
  if (i == 1) { snprintf(b, n, "   0         1"); return true; }
  if (i == 2) { snprintf(b, n, "   01234567890123456789"); return true; }
  if (i >= 3 && i < 7) { snprintf(b, n, "  |%s|", normal.row[i - 3]); return true; }
  if (i == 7) {
    const LedText led = renderLed(d, false);
    snprintf(b, n, "LED [%s] dot after %d", led.digit, led.dotAfter);
    return true;
  }
  if (d.faultCount > 0) {  // the fault screens alternate with the normal one on the real LCD
    const uint8_t k = static_cast<uint8_t>((i - 8) / 5);
    const uint8_t r = static_cast<uint8_t>((i - 8) % 5);
    if (k >= d.faultCount) return false;
    const Screen f = renderFault(d, k);
    if (r == 0) snprintf(b, n, "LCD fault screen %u of %u:", static_cast<unsigned>(k + 1), static_cast<unsigned>(d.faultCount));
    else snprintf(b, n, "  |%s|", f.row[r - 1]);
    return true;
  }
  return false;
}

bool Commands::Loop::line(uint8_t i, char* b, size_t n) {
  if (i != 0) return false;
  const LoopStats& s = *c_.loop;
  snprintf(b, n, "loop n=%lu  min %lu  mean %lu  median<=%lu  max %lu us  slow(>=1s) %lu", static_cast<unsigned long>(s.count()),
           static_cast<unsigned long>(s.minUs()), static_cast<unsigned long>(s.meanUs()), static_cast<unsigned long>(s.medianUs()),
           static_cast<unsigned long>(s.maxUs()), static_cast<unsigned long>(s.slowCount()));
  return true;
}

bool Commands::Scan::line(uint8_t i, char* b, size_t n) {
  if (i == 0) {  // run the scan once per answer
    memset(found_, 0, sizeof found_);
    for (uint8_t a = 0x08; a < 0x78; ++a)
      if (c_.hal->i2cProbe(a)) found_[a / 8] = static_cast<uint8_t>(found_[a / 8] | (1u << (a % 8)));
    buildScanReport(c_.board != nullptr ? *c_.board : kBoard, found_, report_);
    snprintf(b, n, "I2C scan 0x08-0x77: %u address(es) answered", static_cast<unsigned>(report_.responses));
    return true;
  }
  if (i - 1u >= report_.count) return false;
  snprintf(b, n, "  %s", report_.line[i - 1u]);
  return true;
}

// One block with everything, for pasting back to the author (LOG-2).
bool Commands::Report::line(uint8_t i, char* b, size_t n) {
  Responder* parts[] = {&o_.ver_, &o_.time_, &o_.status_, &o_.faults_, &o_.cfg_, &o_.loop_, &o_.display_, &o_.nvmInfo_, &o_.pins_};
  if (i == 0) { snprintf(b, n, "==== N2 REPORT BEGIN ===="); return true; }
  uint8_t idx = static_cast<uint8_t>(i - 1);
  for (Responder* p : parts) {
    uint8_t count = 0;
    char probe[96];
    while (p->line(count, probe, sizeof probe)) ++count;
    if (idx < count) return p->line(idx, b, n);
    idx = static_cast<uint8_t>(idx - count);
  }
  if (idx == 0) { snprintf(b, n, "==== N2 REPORT END ===="); return true; }
  return false;
}

}  // namespace n2
