// Tests for LoopStats, LineReader, command parsing, Console and Commands: NFR-1, CON-1..CON-7, LOG-1/2.
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "TestSupport.h"
#include "core/LoopStats.h"
#include "ui/Commands.h"
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
TEST_CASE("CON-6: LineReader collects a line and ignores CR") {
  LineReader r;
  for (char c : std::string("status")) CHECK(r.feed(c) == LineReader::Result::kNone);
  CHECK(r.feed('\r') == LineReader::Result::kNone);
  CHECK(r.feed('\n') == LineReader::Result::kLine);
  CHECK(std::string(r.line()) == "status");
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
  c = parseCommand("sim a b c d e");
  CHECK(c.id == CommandId::kSim);
  CHECK(c.argc == 3);
}

TEST_CASE("CON-1: every documented command parses, '?' is help, junk is unknown, empty is none") {
  struct { const char* text; CommandId id; } cases[] = {
      {"help", CommandId::kHelp}, {"?", CommandId::kHelp}, {"ver", CommandId::kVer}, {"status", CommandId::kStatus},
      {"report", CommandId::kReport}, {"log", CommandId::kLog}, {"faults", CommandId::kFaults}, {"cfg", CommandId::kCfg},
      {"display", CommandId::kDisplay}, {"loop", CommandId::kLoop}, {"scan", CommandId::kScan}, {"post", CommandId::kPost},
      {"bist", CommandId::kBist}, {"sim", CommandId::kSim}, {"frobnicate", CommandId::kUnknown}, {"", CommandId::kNone},
      {"   ", CommandId::kNone}};
  for (const auto& c : cases) {
    INFO(c.text);
    CHECK(parseCommand(c.text).id == c.id);
  }
}

// ---------------------------------------------------------------------------- Console
namespace {
struct Rig {
  Plant plant;  // booted in the constructor; do NOT call plant.reboot() later (it replaces the System)
  Console console;
  LoopStats loop;
  ConsoleContext ctx;
  Commands commands;

  Rig()
      : console(plant.hal, LogLevel::kInfo),
        ctx{(plant.reboot(), &plant.sys()), &console, &loop, &plant.hal, BuildInfo{"0.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10}, LcdLayout::kClearLabels},
        commands(ctx) {
    plant.hal.consoleIsAttached = true;
  }

  // Type a command, then poll (draining the buffer each time) until the answer is complete.
  std::string ask(const std::string& text, int maxPolls = 400) {
    plant.hal.consoleOut.clear();
    plant.hal.type(text + "\n");
    for (int i = 0; i < maxPolls; ++i) {
      console.poll(commands);
      plant.hal.drain();
      if (!console.busy() && plant.hal.consoleIn.empty()) break;
    }
    return plant.hal.consoleOut;
  }
};
bool contains(const std::string& s, const std::string& needle) { return s.find(needle) != std::string::npos; }
}  // namespace

TEST_CASE("CON-4: nothing is written, and nothing blocks, when no console is attached") {
  Plant p;
  Console c(p.hal, LogLevel::kInfo);
  p.hal.consoleIsAttached = false;
  c.write(LogLevel::kError, "1000 something");
  CHECK(p.hal.consoleOut.empty());
  CHECK(c.dropped() == 0);  // not attached is not a "dropped" (F40) condition
  CHECK_FALSE(c.attached());
}

TEST_CASE("LOG-1: log lines go out as '<level letter> <line>'") {
  Plant p;
  Console c(p.hal, LogLevel::kDebug);
  p.hal.consoleIsAttached = true;
  c.write(LogLevel::kError, "100 E-line");
  c.write(LogLevel::kWarn, "200 W-line");
  c.write(LogLevel::kInfo, "300 I-line");
  c.write(LogLevel::kDebug, "400 D-line");
  CHECK(p.hal.consoleOut == "E 100 E-line\nW 200 W-line\nI 300 I-line\nD 400 D-line\n");
}

TEST_CASE("CON-5: the log level filters lines") {
  Plant p;
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
  Plant p;
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
  r.plant.hal.consoleSpace = 0;  // host not reading
  r.plant.hal.type("log debug\n");
  r.console.poll(r.commands);
  CHECK(r.console.level() == LogLevel::kDebug);  // the command ran...
  CHECK(r.console.busy());                       // ...its answer is waiting for room
  CHECK(r.plant.hal.consoleOut.empty());
  r.plant.hal.drain();
  r.console.poll(r.commands);
  CHECK(contains(r.plant.hal.consoleOut, "log level = debug"));
  CHECK_FALSE(r.console.busy());
}

TEST_CASE("CON-2: a multi-line answer is deferred, not dropped, when TX space is short") {
  Rig r;
  r.plant.hal.consoleSpace = 120;  // room for about one line at a time (the real buffer is 256 bytes)
  r.plant.hal.type("help\n");
  std::string collected;
  for (int i = 0; i < 200 && (collected.empty() || r.console.busy()); ++i) {
    r.console.poll(r.commands);
    collected += r.plant.hal.consoleOut;
    r.plant.hal.consoleOut.clear();
    r.plant.hal.consoleSpace = 120;
  }
  CHECK(contains(collected, "commands (case-insensitive):"));
  CHECK(contains(collected, "bench simulation"));  // the LAST line arrived: nothing was lost
  CHECK(r.console.dropped() == 0);
}

TEST_CASE("CON-4: attach prints a marker; detach forgets partial input and unfinished answers") {
  Rig r;
  r.plant.hal.consoleIsAttached = false;
  r.console.poll(r.commands);
  CHECK(r.plant.hal.consoleOut.empty());
  r.plant.hal.consoleIsAttached = true;
  r.console.poll(r.commands);
  CHECK(contains(r.plant.hal.consoleOut, "[console attached]"));

  r.plant.hal.type("hel");  // half a command...
  r.console.poll(r.commands);
  r.plant.hal.consoleIsAttached = false;  // ...the cable is pulled
  r.console.poll(r.commands);
  r.plant.hal.consoleIsAttached = true;
  r.plant.hal.consoleOut.clear();
  r.plant.hal.type("p\n");  // the rest of an old command must NOT combine with new input
  r.console.poll(r.commands);
  CHECK(contains(r.plant.hal.consoleOut, "unknown command 'p'"));
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
TEST_CASE("CON-1: help lists the commands; unknown input is explained; post/bist/sim say not yet") {
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
  r.plant.tbs(true);
  r.plant.run(3000);
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
  r.plant.tbs(true);
  r.plant.rawN2High(0);
  r.plant.run(3000);
  const std::string s = r.ask("status");
  CHECK(contains(s, "FAULT"));
  CHECK(contains(s, "FAULTS 1: F03"));
}

TEST_CASE("§10: faults lists each active fault with severity and effect, or says none") {
  Rig r;
  CHECK(contains(r.ask("faults"), "no active faults"));
  r.plant.tbs(true);
  r.plant.rawN2High(0);
  r.plant.run(3000);
  const std::string f = r.ask("faults");
  CHECK(contains(f, "F03 N2H SENSOR RANGE"));
  CHECK(contains(f, "INHIBIT"));
  CHECK(contains(f, "TOWERS+SSR OFF"));
}

TEST_CASE("§10: cfg prints the thresholds with their units") {
  Rig r;
  const std::string c = r.ask("cfg");
  CHECK(contains(c, "airLowOff   900  ( 90.0 PSI)"));
  CHECK(contains(c, "n2LowOn     2000  (20.00 PSI)"));
  CHECK(contains(c, "n2HighOff   1200  (120.0 PSI)"));
  CHECK(contains(c, "o2Warmup    300000 ms  mandatory yes"));
  CHECK(contains(c, "outputMinHold 1000 ms"));
}

TEST_CASE("DRV-3: display prints what the LCD and LED should show") {
  Rig r;
  r.plant.tbs(true);
  r.plant.run(2000);
  const std::string d = r.ask("display");
  CHECK(contains(d, "LCD normal screen:"));
  CHECK(contains(d, "|WRM  4:5"));  // production config: 5-minute warm-up countdown instead of N2%
  CHECK(contains(d, "CMP "));
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

TEST_CASE("BIST step 2/§10: scan lists responders, labels them, and reports missing expected devices") {
  Rig r;
  r.plant.hal.i2cPresent = {0x24, 0x34, 0x35, 0x36, 0x37, 0x27, 0x50};
  const std::string s = r.ask("scan");
  CHECK(contains(s, "7 device(s)"));
  CHECK(contains(s, "0x24 LED control"));
  CHECK(contains(s, "0x37 LED digit 3"));
  CHECK(contains(s, "0x27 LCD"));
  CHECK(contains(s, "0x50 unexpected"));
  CHECK(contains(s, "0x74 O2 sensor  MISSING"));
}

TEST_CASE("LOG-2: report is one delimited block holding every section") {
  Rig r;
  r.plant.tbs(true);
  r.plant.run(3000);
  const std::string rep = r.ask("report");
  const size_t begin = rep.find("==== N2 REPORT BEGIN ====");
  const size_t end = rep.find("==== N2 REPORT END ====");
  REQUIRE(begin != std::string::npos);
  REQUIRE(end != std::string::npos);
  CHECK(begin < end);
  const std::string block = rep.substr(begin, end - begin);
  for (const char* w : {"N2V8 0.0.0-test", "TBS ON", "FAULTS", "airLowOff", "loop n=", "LCD normal screen:"}) {
    INFO(w);
    CHECK(contains(block, w));
  }
}

TEST_CASE("LOG-2: the report survives a slow host (small TX buffer) with every line intact and in order") {
  Rig r;
  r.plant.hal.type("report\n");
  std::string all;
  for (int i = 0; i < 3000 && (all.empty() || r.console.busy() || all.find("REPORT END") == std::string::npos); ++i) {
    r.plant.hal.consoleSpace = 100;
    r.console.poll(r.commands);
    all += r.plant.hal.consoleOut;
    r.plant.hal.consoleOut.clear();
  }
  CHECK(contains(all, "==== N2 REPORT BEGIN ===="));
  CHECK(contains(all, "==== N2 REPORT END ===="));
  CHECK(r.console.dropped() == 0);
}
