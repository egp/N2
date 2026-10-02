#include "Commands.h"

#include <stdio.h>
#include <string.h>

#include "../core/Scaling.h"
#include "LedText.h"

namespace n2 {

Commands::Commands(const ConsoleContext& ctx)
    : c_(ctx), ver_(c_), status_(c_), faults_(c_), cfg_(c_), display_(c_), loop_(c_), scan_(c_), report_(*this) {}

Responder* Commands::handle(const Command& cmd) {
  switch (cmd.id) {
    case CommandId::kNone:    return nullptr;
    case CommandId::kHelp:    return &help_;
    case CommandId::kVer:     return &ver_;
    case CommandId::kStatus:  return &status_;
    case CommandId::kFaults:  return &faults_;
    case CommandId::kCfg:     return &cfg_;
    case CommandId::kDisplay: return &display_;
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
    case CommandId::kSim:
      message_.set("'sim' is not available in this build");
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
      "  loop [reset]       loop() timing: min, mean, median, max",
      "  log <level>        error | warn | info | debug",
      "  scan               I2C scan",
      "  report             everything above in one block",
      "  post               run the power-on self-test again (disables the plant while it runs)",
      "  bist               interactive self-test (needs TBS OFF)",
      "  sim                bench simulation (not in this build)",
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
          len += snprintf(b + len, n - static_cast<size_t>(len), " F%02u", static_cast<unsigned>(faultInfo(id).code));
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
  snprintf(b, n, "F%02u %-16s %-7s %s", static_cast<unsigned>(info.code), info.text,
           info.severity == Severity::kInhibit ? "INHIBIT" : (info.severity == Severity::kWarn ? "WARNING" : "INFO"),
           info.effect);
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
    count_ = 0;
    for (uint8_t a = 0x08; a < 0x78; ++a) {
      if (c_.hal->i2cProbe(a)) {
        found_[a / 8] = static_cast<uint8_t>(found_[a / 8] | (1u << (a % 8)));
        ++count_;
      }
    }
    snprintf(b, n, "I2C scan 0x08-0x77: %u device(s)", static_cast<unsigned>(count_));
    return true;
  }
  // Responding addresses first, then expected-but-missing ones.
  struct Known { uint8_t addr; const char* name; };
  const Known known[7] = {{kBoard.addrLed, "LED control"},          {static_cast<uint8_t>(kBoard.addrLedDigits + 0), "LED digit 0"},
                          {static_cast<uint8_t>(kBoard.addrLedDigits + 1), "LED digit 1"}, {static_cast<uint8_t>(kBoard.addrLedDigits + 2), "LED digit 2"},
                          {static_cast<uint8_t>(kBoard.addrLedDigits + 3), "LED digit 3"}, {kBoard.addrLcd, "LCD"}, {kBoard.addrO2, "O2 sensor"}};
  uint8_t idx = static_cast<uint8_t>(i - 1);
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    if (!(found_[a / 8] & (1u << (a % 8)))) continue;
    if (idx-- == 0) {
      const char* label = "unexpected";
      for (const Known& k : known) if (k.addr == a) label = k.name;
      snprintf(b, n, "  0x%02X %s", static_cast<unsigned>(a), label);
      return true;
    }
  }
  for (const Known& k : known) {
    if (found_[k.addr / 8] & (1u << (k.addr % 8))) continue;
    if (idx-- == 0) {
      snprintf(b, n, "  0x%02X %s  MISSING", static_cast<unsigned>(k.addr), k.name);
      return true;
    }
  }
  return false;
}

// One block with everything, for pasting back to the author (LOG-2).
bool Commands::Report::line(uint8_t i, char* b, size_t n) {
  Responder* parts[] = {&o_.ver_, &o_.status_, &o_.faults_, &o_.cfg_, &o_.loop_, &o_.display_};
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
