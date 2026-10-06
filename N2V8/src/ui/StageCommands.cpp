#include "StageCommands.h"

#include <stdlib.h>
#include <string.h>

#include "../Config.h"
#include "I2cSweep.h"

namespace n2 {

Responder* StageCommands::handle(const Command& cmd) {
  out_.clear();
  switch (cmd.id) {
    case CommandId::kNone: return nullptr;
    case CommandId::kHelp: help(); break;
    case CommandId::kVer:
      out_.add("N2V8 %s  built %s %s  board %s  mode %s", c_.info.version, c_.info.date, c_.info.time, c_.info.board, c_.info.mode);
      break;
    case CommandId::kStatus: status(); break;
    case CommandId::kScan: scan(); break;
    case CommandId::kTime: time(cmd); break;
    case CommandId::kLog: log(cmd); break;
    case CommandId::kLoop: loopStats(); break;
    case CommandId::kPost:
      c_.actions.startPost();
      out_.add("POST restarted");
      break;
    case CommandId::kBist:
      if (c_.actions.startBist()) out_.add("BIST started");
      else out_.add("BIST cannot start now (POST is still running or held: type `go` to release it)");
      break;
    default:
      if (strcmp(cmd.name, "i2c") == 0) {
        i2cCommand(cmd);
      } else if (strcmp(cmd.name, "lcd") == 0) {
        lcdCommand(cmd);
      } else if (strcmp(cmd.name, "go") == 0) {
        c_.actions.releaseHold();
        out_.add("go: POST hold released (if there was one)");
      } else {
        out_.add("unknown command '%s'. Type help.", cmd.name);
      }
      break;
  }
  return &out_;
}

void StageCommands::help() {
  out_.add("help              this list");
  out_.add("ver               version and build");
  out_.add("status            POST results, reset cause, RTC and clock state");
  out_.add("post              run the power-on self-test again");
  out_.add("bist              run the operator-confirmed self-test (answers: p f r s q)");
  out_.add("go                release a POST hold (same as pressing TOB)");
  out_.add("lcd [reinit|bus]  LCD state; `lcd reinit` restarts the controller; `lcd bus [n]` tests the I2C link to the backpack");
  out_.add("i2c sweep [n]     test the I2C bus at 100 kHz and 400 kHz, the only two speeds the R4 has");
  out_.add("scan              list every device that answers on the I2C bus");
  out_.add("time              show the RTC date/time and whether it can be trusted");
  out_.add("time set D T      set the RTC (YYYY-MM-DD HH:MM:SS) only if it is untrusted or >2 s off");
  out_.add("log [level]       show or set the log level: error warn info debug");
  out_.add("loop              loop() timing: min, mean, median, max");
}

void StageCommands::status() {
  out_.add("N2V8 %s  board %s  mode %s  uptime %lu s", c_.info.version, c_.info.board, c_.info.mode,
           static_cast<unsigned long>(c_.hal.millis() / 1000u));
  out_.add("reset cause: %s", c_.resetCause);
  const SignalDef& tbsDef = def(c_.board, Signal::kTbs);
  const SignalDef& tobDef = def(c_.board, Signal::kTob);
  out_.add("switches: TBS %s (pin D%u reads %s), TOB %s (pin D%u reads %s)", c_.tbsOn(c_.switchOwner) ? "ON" : "off",
           static_cast<unsigned>(tbsDef.pin), c_.hal.digitalRead(tbsDef.pin) ? "HIGH" : "LOW",
           c_.tobPressed(c_.switchOwner) ? "pressed" : "released", static_cast<unsigned>(tobDef.pin),
           c_.hal.digitalRead(tobDef.pin) ? "HIGH" : "LOW");
  for (uint8_t i = 0; i < c_.selfTest.checkCount(); ++i) {
    const CheckResult& r = c_.selfTest.postResult(i);
    const char* word = r.level == CheckLevel::kPass ? "ok" : (r.level == CheckLevel::kInfo ? "info" : (r.level == CheckLevel::kFail ? "FAIL" : "-"));
    out_.add("  %-6s %-4s %s", c_.selfTest.checkName(i), word, r.text);
  }
  out_.add("LCD: I2C errors %lu, re-initialisations %lu", static_cast<unsigned long>(c_.lcd.i2cErrors()),
           static_cast<unsigned long>(c_.lcd.reinitCount()));
  char stamp[20];
  out_.add("log clock: %s", c_.wall.stamp(c_.hal.millis(), stamp) ? stamp : "not synced (no trusted RTC)");
  out_.add("console: dropped %lu log lines, received %lu bytes", static_cast<unsigned long>(c_.console.dropped()),
           static_cast<unsigned long>(c_.console.received()));
}

void StageCommands::scan() {
  uint8_t count = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    if (!c_.hal.i2cProbe(a)) continue;
    const char* who = "unknown";
    if (a == c_.board.addrLcd) who = "LCD backpack";
    else if (a == c_.board.addrRtc) who = "DS3231 RTC";
    else if (a == 0x57) who = "RTC module EEPROM";
    else if (a == c_.board.addrLed || (a >= c_.board.addrLedDigits && a < c_.board.addrLedDigits + 4)) who = "TM1650 LED";
    else if (a == c_.board.addrO2) who = "O2 sensor";
    out_.add("  0x%02X  %s", static_cast<unsigned>(a), who);
    ++count;
  }
  out_.add("I2C scan 0x08-0x77: %u device(s)", static_cast<unsigned>(count));
}

void StageCommands::time(const Command& cmd) {
  if (!c_.rtc.present()) {
    out_.add("RTC: not answering at 0x%02X (the firmware carries on without time stamps)", static_cast<unsigned>(c_.board.addrRtc));
    return;
  }
  DateTime cur{};
  bool trusted = false;
  const bool readable = c_.rtc.read(cur);
  if (readable) c_.rtc.timeValid(trusted);

  if (cmd.argc == 0) {
    if (!readable) {
      out_.add("RTC: answers but holds no valid date/time");
      return;
    }
    char when[20];
    formatDateTime(when, cur);
    out_.add("RTC %s (%s)", when, trusted ? "trusted" : "NOT trusted: it lost power since it was last set");
    int16_t t = 0;
    if (c_.rtc.temperatureX100(t)) out_.add("RTC chip temperature %d.%02d C", t / 100, (t < 0 ? -t : t) % 100);
    return;
  }

  DateTime want{};
  if (cmd.argc != 3 || strcmp(cmd.arg[0], "set") != 0 || !parseDateTime(cmd.arg[1], cmd.arg[2], want)) {
    out_.add("usage: time set YYYY-MM-DD HH:MM:SS   (24-hour clock)");
    return;
  }
  // RTC-8: the RTC is read-only unless it is untrusted or more than 2 s away from the reference time.
  if (!rtcNeedsSetting(readable, trusted, cur, want)) {
    out_.add("RTC already within 2 s of that time: left unchanged");
    return;
  }
  char when[20];
  formatDateTime(when, want);
  if (!c_.rtc.set(want)) {
    out_.add("could not write the RTC (I2C error)");
    return;
  }
  c_.wall.sync(want, c_.hal.millis());
  out_.add("RTC set to %s", when);
}

void StageCommands::log(const Command& cmd) {
  if (cmd.argc >= 1) {
    LogLevel level;
    if (!Console::parseLevel(cmd.arg[0], level)) {
      out_.add("usage: log [error|warn|info|debug]");
      return;
    }
    c_.console.setLevel(level);
  }
  out_.add("log level: %s", Console::levelName(c_.console.level()));
}

void StageCommands::lcdCommand(const Command& cmd) {
  if (cmd.argc >= 1 && strcmp(cmd.arg[0], "bus") == 0) {
    int n = cmd.argc >= 2 ? atoi(cmd.arg[1]) : 200;
    if (n < 1) n = 1;
    if (n > 500) n = 500;  // a diagnostic: about 0.6 ms per round, so at most about 0.3 s
    const Lcd20x4::BusTest t = c_.lcd.busTest(static_cast<uint16_t>(n));
    out_.add("LCD bus: %u rounds, write fail %u, read fail %u, mismatch %u, %lu us/round", t.rounds,
             t.writeFailed, t.readFailed, t.mismatched, static_cast<unsigned long>(t.microsPerRound));
    if (t.mismatched > 0) out_.add("  first mismatch: wrote 0x%02X, read back 0x%02X", static_cast<unsigned>(t.firstOut), static_cast<unsigned>(t.firstBack));
    out_.add("  (all zero = the bytes reach the backpack intact; any other number = a bad bus: noise, pullups, wiring, speed)");
    return;
  }
  if (cmd.argc >= 1) {
    if (strcmp(cmd.arg[0], "reinit") != 0) {
      out_.add("usage: lcd [reinit | bus [rounds]]");
      return;
    }
    c_.lcd.reinit(c_.hal.millis());
    c_.actions.lcdReinitialised();
    out_.add("LCD controller restarting (the screen clears and redraws)");
  }
  out_.add("LCD: %s, I2C errors %lu, re-initialisations %lu, content %s", c_.lcd.ready() ? "ready" : "not ready",
           static_cast<unsigned long>(c_.lcd.i2cErrors()), static_cast<unsigned long>(c_.lcd.reinitCount()),
           c_.lcd.inSync() ? "in sync" : "being written");
}

void StageCommands::i2cCommand(const Command& cmd) {
  if (cmd.argc < 1 || strcmp(cmd.arg[0], "sweep") != 0) {
    out_.add("usage: i2c sweep [rounds]   (bus clock now %lu Hz)", static_cast<unsigned long>(kI2cClockHz));
    return;
  }
  int n = cmd.argc >= 2 ? atoi(cmd.arg[1]) : 100;
  if (n < 1) n = 1;
  if (n > 200) n = 200;
  SweepRow rows[kSweepMaxSpeeds];
  const uint8_t count = runI2cSweep(c_.hal, c_.lcd, c_.rtc, c_.board.addrLcd, c_.board.addrRtc, kSweepSpeeds,
                                    static_cast<uint8_t>(sizeof kSweepSpeeds / sizeof kSweepSpeeds[0]), static_cast<uint16_t>(n),
                                    kI2cClockHz, rows);
  out_.add("I2C sweep (%d LCD rounds, RTC reads, probes) bad counts, 0 = good:", n);
  for (uint8_t i = 0; i < count; ++i)
    out_.add("  %6lu Hz: LCD %3u  RTC %2u/%u  probe %2u/%u  %s  (RTC read %lu us)", static_cast<unsigned long>(rows[i].hz),
             rows[i].lcdBad, rows[i].rtcBad, rows[i].rtcReads, rows[i].probeBad, rows[i].probes, rows[i].ok() ? "OK" : "BAD",
             static_cast<unsigned long>(rows[i].microsPerRtcRead));
  out_.add("  clock restored to %lu Hz. (Specs: PCF8574 LCD backpack 100 kHz, DS3231 400 kHz.)", static_cast<unsigned long>(kI2cClockHz));
}

void StageCommands::loopStats() {
  out_.add("loop passes %lu | us: min %lu mean %lu median %lu max %lu | >=1 s: %lu",
           static_cast<unsigned long>(c_.loop.count()), static_cast<unsigned long>(c_.loop.minUs()),
           static_cast<unsigned long>(c_.loop.meanUs()), static_cast<unsigned long>(c_.loop.medianUs()),
           static_cast<unsigned long>(c_.loop.maxUs()), static_cast<unsigned long>(c_.loop.slowCount()));
}

}  // namespace n2
