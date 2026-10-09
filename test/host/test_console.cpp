// Tests for LoopStats, LineReader, command parsing, Console and Commands: NFR-1, CON-1..CON-7, LOG-1/2.
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "TestSupport.h"
#include "core/LoopStats.h"
#include "ui/Commands.h"
#include "ui/ScanReport.h"
#include "ui/Console.h"

using namespace n2;
using namespace n2::test;

// ---------------------------------------------------------------------------- LoopStats
TEST_CASE("NFR-1: LoopStats with no samples reports zeros") {
  LoopStats s;
  CHECK(s.count() == 0);
  CHECK(s.minUs() == 0);
  CHECK(s.maxUs() == 0);
  CHECK(s.meanUs() == 0);
  CHECK(s.medianUs() == 0);
}

TEST_CASE("NFR-1: LoopStats min, mean, max are exact") {
  LoopStats s;
  for (uint32_t us : {100u, 200u, 300u, 400u, 1000u}) s.record(us);
  CHECK(s.count() == 5);
  CHECK(s.minUs() == 100);
  CHECK(s.maxUs() == 1000);
  CHECK(s.meanUs() == 400);
}

TEST_CASE("NFR-1: LoopStats median is robust against a rare slow pass (histogram)") {
  LoopStats s;
  for (int i = 0; i < 99; ++i) s.record(150);   // typical pass: 150 us
  s.record(900000);                              // one slow pass
  CHECK(s.medianUs() == 200);                    // upper edge of the 100-200 us bucket
  CHECK(s.maxUs() == 900000);
  CHECK(s.meanUs() > 8000);                      // the mean is dragged up, the median is not
  CHECK(s.slowCount() == 0);
  s.record(1000000);
  CHECK(s.slowCount() == 1);
}

TEST_CASE("NFR-1: the median is capped at the true maximum") {
  LoopStats s;
  s.record(130);
  CHECK(s.medianUs() == 130);  // not the bucket edge 200
}

TEST_CASE("NFR-1: LoopStats bucket edges and reset") {
  LoopStats s;
  s.record(99);
  s.record(100);
  CHECK(LoopStats::bucketLimitUs(0) == 100);
  CHECK(s.count() == 2);
  s.reset();
  CHECK(s.count() == 0);
  CHECK(s.minUs() == 0);
  s.record(5);
  CHECK(s.minUs() == 5);
}

// ---------------------------------------------------------------------------- LineReader
TEST_CASE("CON-6: LineReader ends a line at LF, CR or CRLF - one line each, whatever the Serial Monitor sends") {
  for (const char* ending : {"\n", "\r", "\r\n"}) {
    LineReader r;
    int lines = 0;
    std::string typed = std::string("status") + ending + "ver" + ending;
    std::string got;
    for (char c : typed) {
      if (r.feed(c) == LineReader::Result::kLine) { ++lines; got += std::string(r.line()) + "|"; }
    }
    INFO("ending length " << strlen(ending));
    CHECK(lines == 2);          // no extra empty line from a CRLF
    CHECK(got == "status|ver|");
  }
}

TEST_CASE("CON-6: LineReader handles empty lines and back-to-back lines") {
  LineReader r;
  CHECK(r.feed('\n') == LineReader::Result::kLine);
  CHECK(std::string(r.line()) == "");
  r.feed('a'); r.feed('\n');
  CHECK(std::string(r.line()) == "a");
  r.feed('b'); r.feed('\n');
  CHECK(std::string(r.line()) == "b");
}

TEST_CASE("CON-6: an over-long line is discarded, and the next line is fine") {
  LineReader r;
  for (int i = 0; i < 200; ++i) r.feed('x');
  CHECK(r.feed('\n') == LineReader::Result::kOverflow);
  for (char c : std::string("ok")) r.feed(c);
  CHECK(r.feed('\n') == LineReader::Result::kLine);
  CHECK(std::string(r.line()) == "ok");
}

// ---------------------------------------------------------------------------- parseCommand
TEST_CASE("CON-1: commands are case-insensitive and keep up to three arguments") {
  Command c = parseCommand("  LOG   Debug ");
  CHECK(c.id == CommandId::kLog);
  CHECK(c.argc == 1);
  CHECK(std::string(c.arg[0]) == "debug");
  c = parseCommand("loop\treset");
  CHECK(c.id == CommandId::kLoop);
  CHECK(std::string(c.arg[0]) == "reset");
  c = parseCommand("loop a b c d e");
  CHECK(c.id == CommandId::kLoop);
  CHECK(c.argc == 3);
}

TEST_CASE("CON-1: every documented command parses, '?' is help, junk is unknown, empty is none") {
  struct { const char* text; CommandId id; } cases[] = {
      {"help", CommandId::kHelp}, {"?", CommandId::kHelp}, {"ver", CommandId::kVer}, {"status", CommandId::kStatus},
      {"report", CommandId::kReport}, {"log", CommandId::kLog}, {"faults", CommandId::kFaults}, {"cfg", CommandId::kCfg},
      {"display", CommandId::kDisplay}, {"loop", CommandId::kLoop}, {"scan", CommandId::kScan}, {"post", CommandId::kPost},
      {"bist", CommandId::kBist}, {"sim", CommandId::kUnknown}, {"frobnicate", CommandId::kUnknown}, {"", CommandId::kNone},
      {"   ", CommandId::kNone}};
  for (const auto& c : cases) {
    INFO(c.text);
    CHECK(parseCommand(c.text).id == c.id);
  }
}

// ---------------------------------------------------------------------------- Console
namespace {
struct Rig {
  Generator gen;  // booted in the constructor; do NOT call gen.reboot() later (it replaces the System)
  Console console;
  LoopStats loop;
  Rtc3231 rtc;
  ConsoleContext ctx;
  Commands commands;

  Rig()
      : console(gen.hal, LogLevel::kInfo),
        rtc(gen.hal),
        ctx{(gen.reboot(), &gen.sys()), &console, &loop, &gen.hal, BuildInfo{"0.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10}, LcdLayout::kClearLabels, nullptr, &rtc},
        commands(ctx) {
    gen.hal.consoleIsAttached = true;
  }

  // Type a command, then poll (draining the buffer each time) until the answer is complete.
  std::string ask(const std::string& text, int maxPolls = 400) {
    gen.hal.consoleOut.clear();
    gen.hal.type(text + "\n");
    for (int i = 0; i < maxPolls; ++i) {
      console.poll(commands);
      gen.hal.drain();
      if (!console.busy() && gen.hal.consoleIn.empty()) break;
    }
    return gen.hal.consoleOut;
  }
};
bool contains(const std::string& s, const std::string& needle) { return s.find(needle) != std::string::npos; }
}  // namespace

TEST_CASE("CON-4: nothing is written, and nothing blocks, when no console is attached") {
  Generator p;
  Console c(p.hal, LogLevel::kInfo);
  p.hal.consoleIsAttached = false;
  c.write(LogLevel::kError, "1000 something");
  CHECK(p.hal.consoleOut.empty());
  CHECK(c.dropped() == 0);  // not attached is not a "dropped" (F40) condition
  CHECK_FALSE(c.attached());
}

TEST_CASE("LOG-1: log lines go out as '<level letter> <line>'") {
  Generator p;
  Console c(p.hal, LogLevel::kDebug);
  p.hal.consoleIsAttached = true;
  c.write(LogLevel::kError, "100 E-line");
  c.write(LogLevel::kWarn, "200 W-line");
  c.write(LogLevel::kInfo, "300 I-line");
  c.write(LogLevel::kDebug, "400 D-line");
  CHECK(p.hal.consoleOut == "E 100 E-line\nW 200 W-line\nI 300 I-line\nD 400 D-line\n");
}

TEST_CASE("CON-5: the log level filters lines") {
  Generator p;
  Console c(p.hal, LogLevel::kInfo);
  p.hal.consoleIsAttached = true;
  c.write(LogLevel::kDebug, "1 hidden");
  c.write(LogLevel::kInfo, "2 shown");
  CHECK(p.hal.consoleOut == "I 2 shown\n");
  c.setLevel(LogLevel::kError);
  c.write(LogLevel::kWarn, "3 hidden");
  CHECK(p.hal.consoleOut == "I 2 shown\n");
}

TEST_CASE("CON-2: a full TX buffer drops log lines and counts them; it never blocks") {
  Generator p;
  Console c(p.hal, LogLevel::kInfo);
  p.hal.consoleIsAttached = true;
  p.hal.consoleSpace = 20;
  c.write(LogLevel::kInfo, "1 first line fits");  // 19 bytes with the prefix and newline
  c.write(LogLevel::kInfo, "2 second line does not fit");
  c.write(LogLevel::kInfo, "3 third line does not fit either");
  CHECK(c.dropped() == 2);
  CHECK(p.hal.consoleOut.find("first") != std::string::npos);
  CHECK(p.hal.consoleOut.find("second") == std::string::npos);
  p.hal.drain();
  c.write(LogLevel::kInfo, "4 after the host catches up");
  CHECK(p.hal.consoleOut.find("after the host") != std::string::npos);
  CHECK(c.dropped() == 2);
}

TEST_CASE("CON-2: typed commands still get through while the output side is completely stalled") {
  Rig r;
  r.gen.hal.consoleSpace = 0;  // host not reading
  r.gen.hal.type("log debug\n");
  r.console.poll(r.commands);
  CHECK(r.console.level() == LogLevel::kDebug);  // the command ran...
  CHECK(r.console.busy());                       // ...its answer is waiting for room
  CHECK(r.gen.hal.consoleOut.empty());
  r.gen.hal.drain();
  r.console.poll(r.commands);
  CHECK(contains(r.gen.hal.consoleOut, "log level = debug"));
  CHECK_FALSE(r.console.busy());
}

TEST_CASE("CON-2: a multi-line answer is deferred, not dropped, when TX space is short") {
  Rig r;
  r.gen.hal.consoleSpace = 120;  // room for about one line at a time (the real buffer is 256 bytes)
  r.gen.hal.type("help\n");
  std::string collected;
  for (int i = 0; i < 200 && (collected.empty() || r.console.busy()); ++i) {
    r.console.poll(r.commands);
    collected += r.gen.hal.consoleOut;
    r.gen.hal.consoleOut.clear();
    r.gen.hal.consoleSpace = 120;
  }
  CHECK(contains(collected, "commands (case-insensitive):"));
  CHECK(contains(collected, "interactive self-test"));  // the LAST line arrived: nothing was lost
  CHECK(r.console.dropped() == 0);
}

TEST_CASE("CON-4: attach prints a marker; detach forgets partial input and unfinished answers") {
  Rig r;
  r.gen.hal.consoleIsAttached = false;
  r.console.poll(r.commands);
  CHECK(r.gen.hal.consoleOut.empty());
  r.gen.hal.consoleIsAttached = true;
  r.console.poll(r.commands);
  CHECK(contains(r.gen.hal.consoleOut, "[console attached]"));

  r.gen.hal.type("hel");  // half a command...
  r.console.poll(r.commands);
  r.gen.hal.consoleIsAttached = false;  // ...the cable is pulled
  r.console.poll(r.commands);
  r.gen.hal.consoleIsAttached = true;
  r.gen.hal.consoleOut.clear();
  r.gen.hal.type("p\n");  // the rest of an old command must NOT combine with new input
  r.console.poll(r.commands);
  CHECK(contains(r.gen.hal.consoleOut, "unknown command 'p'"));
}

TEST_CASE("CON-6: an over-long line produces an error and the console keeps working") {
  Rig r;
  const std::string out = r.ask(std::string(150, 'x'));
  CHECK(contains(out, "line too long"));
  CHECK(contains(r.ask("ver"), "N2V8 0.0.0-test"));
}

TEST_CASE("CON-1: the console echoes the command it received (so a pasted log shows what was asked)") {
  Rig r;
  const std::string out = r.ask("ver");
  CHECK(contains(out, "> ver"));
}

TEST_CASE("CON-5: 'log' shows and sets the level; a bad level prints usage") {
  Rig r;
  CHECK(contains(r.ask("log"), "log level = info"));
  CHECK(contains(r.ask("log debug"), "log level = debug"));
  CHECK(r.console.level() == LogLevel::kDebug);
  CHECK(contains(r.ask("LOG Error"), "log level = error"));
  CHECK(contains(r.ask("log loud"), "usage: log error|warn|info|debug"));
}

// ---------------------------------------------------------------------------- Commands
TEST_CASE("CON-1: help lists the commands; unknown input is explained; unknown and unavailable commands are explained") {
  Rig r;
  const std::string help = r.ask("help");
  for (const char* w : {"status", "faults", "cfg", "display", "loop", "log <level>", "scan", "report"}) CHECK(contains(help, w));
  CHECK(contains(r.ask("frobnicate"), "unknown command 'frobnicate' - try help"));
  CHECK(contains(r.ask("bist"), "'bist' is not available in this build"));
}

TEST_CASE("ID-1: ver prints the build identity") {
  Rig r;
  const std::string v = r.ask("ver");
  CHECK(contains(v, "N2V8 0.0.0-test"));
  CHECK(contains(v, "built Jan  1 2026 12:00:00"));
  CHECK(contains(v, "board host (fake)  mode HOST  adc 10 bits"));
}

TEST_CASE("§10: status shows inputs (raw, volts, PSI, ok), states, outputs and faults") {
  Rig r;
  r.gen.tbs(true);
  r.gen.run(3000);
  const std::string s = r.ask("status");
  CHECK(contains(s, "TBS ON"));
  CHECK(contains(s, "AIR  raw"));
  CHECK(contains(s, "130.0 PSI ok"));
  CHECK(contains(s, "N2L  raw"));
  CHECK(contains(s, "N2H  raw"));
  CHECK(contains(s, "CMP ON"));
  CHECK(contains(s, "OUT L="));
  CHECK(contains(s, "FAULTS none"));
  CHECK(contains(s, "INVARIANT violations 0"));
}

TEST_CASE("§10: status flags a faulty sensor and lists active faults") {
  Rig r;
  r.gen.tbs(true);
  r.gen.rawN2High(0);
  r.gen.run(3000);
  const std::string s = r.ask("status");
  CHECK(contains(s, "FAULT"));
  CHECK(contains(s, "FAULTS 1: F03"));
}

TEST_CASE("§10: faults lists each active fault with severity and effect, or says none") {
  Rig r;
  CHECK(contains(r.ask("faults"), "no active faults"));
  r.gen.tbs(true);
  r.gen.rawN2High(0);
  r.gen.run(3000);
  const std::string f = r.ask("faults");
  CHECK(contains(f, "F03 N2H SENSOR RANGE"));
  CHECK(contains(f, "INHIBIT"));
  CHECK(contains(f, "TOWERS+SSR OFF"));
}

TEST_CASE("§10: cfg prints the thresholds with their units") {
  Rig r;
  const std::string c = r.ask("cfg");
  CHECK(contains(c, "airLowOff   700  ( 70.0 PSI)"));
  CHECK(contains(c, "n2LowOn     2000  (20.00 PSI)"));
  CHECK(contains(c, "n2HighOff   1200  (120.0 PSI)"));
  CHECK(contains(c, "o2Warmup    300000 ms  mandatory yes"));
  CHECK(contains(c, "outputMinHold 1000 ms"));
}

TEST_CASE("DRV-3: display prints what the LCD and LED should show") {
  Rig r;
  r.gen.tbs(true);
  r.gen.run(2000);
  const std::string d = r.ask("display");
  CHECK(contains(d, "LCD normal screen:"));
  CHECK(contains(d, "|WRM  4:5"));  // production config: 5-minute warm-up countdown instead of N2%
  CHECK_FALSE(contains(d, "ER "));  // no fault raised yet: the ER field is blank; the compressor field only says CMP LO/HI when it is stopped
  CHECK(contains(d, "LED ["));
}

TEST_CASE("NFR-1: loop shows the statistics, and 'loop reset' clears them") {
  Rig r;
  r.loop.record(150);
  r.loop.record(250);
  const std::string l = r.ask("loop");
  CHECK(contains(l, "loop n=2"));
  CHECK(contains(l, "min 150"));
  CHECK(contains(l, "max 250"));
  CHECK(contains(r.ask("loop reset"), "loop statistics reset"));
  CHECK(r.loop.count() == 0);
}

TEST_CASE("§10: scan names each device with its addresses, lists unexpected responders, and accounts for every response") {
  Rig r;
  r.gen.hal.i2cPresent = {0x24, 0x34, 0x35, 0x36, 0x37, 0x23, 0x50, 0x68};
  const std::string s = r.ask("scan");
  CHECK(contains(s, "8 address(es) answered"));
  CHECK(contains(s, "RTC found at 0x68"));
  CHECK(contains(s, "LED found at 0x24,0x34,0x35,0x36,0x37  (control + 0 alias, 4 digits)"));   // the aliases are optional
  CHECK(contains(s, "LCD found at 0x23"));
  CHECK(contains(s, "UNEXPECTED responders: 0x50"));
  CHECK(contains(s, "O2 sensor NOT found (expected 0x74)"));
}

TEST_CASE("LOG-2: report is one delimited block holding every section") {
  Rig r;
  r.gen.tbs(true);
  r.gen.run(3000);
  const std::string rep = r.ask("report");
  const size_t begin = rep.find("==== N2 REPORT BEGIN ====");
  const size_t end = rep.find("==== N2 REPORT END ====");
  REQUIRE(begin != std::string::npos);
  REQUIRE(end != std::string::npos);
  CHECK(begin < end);
  const std::string block = rep.substr(begin, end - begin);
  for (const char* w : {"N2V8 0.0.0-test", "TBS ON", "FAULTS", "airLowOff", "loop n=", "LCD normal screen:", "NVM:"}) {
    INFO(w);
    CHECK(contains(block, w));
  }
}

TEST_CASE("LOG-2: the report survives a slow host (small TX buffer) with every line intact and in order") {
  Rig r;
  r.gen.hal.type("report\n");
  std::string all;
  for (int i = 0; i < 3000 && (all.empty() || r.console.busy() || all.find("REPORT END") == std::string::npos); ++i) {
    r.gen.hal.consoleSpace = 100;
    r.console.poll(r.commands);
    all += r.gen.hal.consoleOut;
    r.gen.hal.consoleOut.clear();
  }
  CHECK(contains(all, "==== N2 REPORT BEGIN ===="));
  CHECK(contains(all, "==== N2 REPORT END ===="));
  CHECK(r.console.dropped() == 0);
}

// ============================================================================ time commands (RTC-3)
TEST_CASE("RTC-3: `time` shows the RTC date and time, whether it can be trusted, and its temperature") {
  Rig r;
  r.gen.hal.i2cPresent.insert(0x68);
  REQUIRE(r.rtc.set({2026, 10, 6, 10, 31, 2}));
  r.gen.hal.i2cRegs[0x68][0x11] = 0x19;
  r.gen.hal.i2cRegs[0x68][0x12] = 0x80;  // 25.50 C
  const std::string t = r.ask("time");
  CHECK(contains(t, "RTC 2026-10-06 10:31:02 (trusted)"));
  CHECK(contains(t, "RTC chip temperature 25.50 C"));
}

TEST_CASE("RTC-3: `time` says when the clock lost power (oscillator-stop flag)") {
  Rig r;
  r.gen.hal.i2cPresent.insert(0x68);
  REQUIRE(r.rtc.set({2026, 10, 6, 10, 31, 2}));
  r.gen.hal.i2cRegs[0x68][0x0F] = 0x80;
  CHECK(contains(r.ask("time"), "NOT trusted"));
}

TEST_CASE("RTC-3: `time` with no chip answering reports it plainly") {
  Rig r;
  CHECK(contains(r.ask("time"), "RTC: no valid answer from the clock chip"));
}

TEST_CASE("RTC-3: `time set YYYY-MM-DD HH:MM:SS` sets the clock and reads it back") {
  Rig r;
  r.gen.hal.i2cPresent.insert(0x68);
  CHECK(contains(r.ask("time set 2026-10-06 10:31:00"), "RTC set to 2026-10-06 10:31:00"));
  DateTime t{};
  REQUIRE(r.rtc.read(t));
  CHECK(t.year == 2026);
  CHECK(t.hour == 10);
  CHECK(t.minute == 31);
}

TEST_CASE("RTC-3: `time set` with bad input is rejected and the chip is not touched") {
  Rig r;
  r.gen.hal.i2cPresent.insert(0x68);
  const size_t writes = r.gen.hal.i2cWrites.size();
  CHECK(contains(r.ask("time set 2026-02-30 10:00:00"), "usage: time set YYYY-MM-DD HH:MM:SS"));
  CHECK(contains(r.ask("time set tomorrow"), "usage: time set"));
  CHECK(contains(r.ask("time bogus"), "usage: time set"));
  CHECK(r.gen.hal.i2cWrites.size() == writes);
}

TEST_CASE("RTC-3: `time set` with the chip absent says so") {
  Rig r;  // 0x68 does not acknowledge
  CHECK(contains(r.ask("time set 2026-10-06 10:31:00"), "could not write the RTC"));
}

TEST_CASE("RTC-3: help describes the time command on one line") {
  Rig r;
  CHECK(contains(r.ask("help"), "time [set ...]"));
  CHECK(parseCommand("TIME set 2026-10-06 10:31:00").id == CommandId::kTime);
  CHECK(parseCommand("time").id == CommandId::kTime);
}

TEST_CASE("CON-1: EVERY command word and its arguments are case-insensitive, in any mix of case") {
  for (const char* word : {"help", "ver", "status", "report", "log", "faults", "cfg", "display", "loop", "scan", "post", "bist", "time", "nvm", "debounce"}) {
    std::string upper = word, mixed = word;
    for (auto& ch : upper) ch = static_cast<char>(toupper(ch));
    for (size_t i = 0; i < mixed.size(); i += 2) mixed[i] = static_cast<char>(toupper(mixed[i]));
    const Command lower = parseCommand(word);
    CHECK(parseCommand(upper.c_str()).id == lower.id);
    CHECK(parseCommand(mixed.c_str()).id == lower.id);
  }
  Command c = parseCommand("LoG DeBuG");
  CHECK(c.id == CommandId::kLog);
  LogLevel level;
  CHECK(Console::parseLevel(c.arg[0], level));
  CHECK(level == LogLevel::kDebug);
  c = parseCommand("DEBOUNCE SET 9 8");
  CHECK(c.id == CommandId::kDebounce);
  CHECK(std::string(c.arg[0]) == "set");
}

TEST_CASE("§10: an LCD at an LED alias address (0x27, spare backpack) is the LCD, not an LED response") {
  BoardDef b = kHostBoard;
  b.addrLcd = 0x27;
  uint8_t found[16] = {};
  for (uint8_t a : {0x27, 0x68}) found[a / 8] = static_cast<uint8_t>(found[a / 8] | (1u << (a % 8)));
  ScanReport rep;
  buildScanReport(b, found, rep);
  std::string all;
  for (uint8_t i = 0; i < rep.count; ++i) all += std::string(rep.line[i]) + "\n";
  CHECK(all.find("LED NOT found") != std::string::npos);
  CHECK(all.find("LCD found at 0x27") != std::string::npos);
  CHECK(all.find("2 address(es) answered: 2 accounted for, 0 unexpected") != std::string::npos);
}
