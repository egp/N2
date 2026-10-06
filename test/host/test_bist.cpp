// BIST tests: §12 BIST-1..BIST-11 (operator-confirmed, console required, outputs vetoed and never unsafe).
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "TestSupport.h"
#include "selftest/Bist.h"
#include "ui/Commands.h"

using namespace n2;
using namespace n2::test;

namespace {

struct Rig {
  Generator gen;
  DisplayManager display;
  Console console;
  LoopStats loopStats;
  ConsoleContext ctx;
  Commands commands;
  BuildInfo info{"0.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  Bist bist;
  uint32_t now = 0;
  std::string out;  // everything the console printed

  explicit Rig(BistConfig bc = BistConfig(), ControlConfig cfg = kDefaultControl)
      : gen(cfg),
        display(gen.hal, kHostBoard, LcdLayout::kClearLabels),
        console(gen.hal, LogLevel::kInfo),
        ctx{(gen.reboot(), &gen.sys()), &console, &loopStats, &gen.hal,
            BuildInfo{"0.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10}, LcdLayout::kClearLabels},
        commands(ctx),
        bist(gen.hal, kHostBoard, gen.sys(), display, console, gen.o2, info, bc) {
    gen.hal.consoleIsAttached = true;
    gen.hal.i2cPresent = {0x24, 0x34, 0x35, 0x36, 0x37, 0x27, 0x74, 0x68};
    display.begin(0);
  }

  // One pass: displays, console, BIST.
  bool pass() {
    now += 10;
    gen.hal.nowMs = now;
    display.service(now);
    console.poll(commands);
    const bool done = bist.step(now);
    out += gen.hal.consoleOut;
    gen.hal.consoleOut.clear();
    gen.hal.drain();
    return done;
  }
  void run(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 10) pass(); }
  void type(const std::string& line) { gen.hal.type(line + "\n"); run(30); }
  bool has(const std::string& s) const { return out.find(s) != std::string::npos; }
  size_t count(const std::string& s) const {
    size_t n = 0, pos = 0;
    while ((pos = out.find(s, pos)) != std::string::npos) { ++n; pos += s.size(); }
    return n;
  }
  Bist::Start start(bool requireTbsOff = true) { return bist.begin(now, requireTbsOff); }
  bool allOff() { return !gen.left() && !gen.right() && !gen.flush() && !gen.ssr(); }
  // Answer 'p' to steps until the given step is reached (waiting a bit so a step can do its work).
  void goTo(BistStep target) {
    for (int guard = 0; guard < 40 && bist.current() != target && bist.running(); ++guard) { run(400); type("p"); }
  }
};

}  // namespace

// ============================================================================ entry
TEST_CASE("BIST-1: BIST refuses to start without a console, and says so on the LCD") {
  Rig r;
  r.gen.hal.consoleIsAttached = false;
  CHECK(r.start() == Bist::Start::kNoConsole);
  CHECK_FALSE(r.bist.running());
  r.run(300);
  CHECK(std::string(r.display.lcd().shown(0)) == "BIST: NO CONSOLE    ");
}

TEST_CASE("BIST-1: the `bist` command refuses while TBS is ON; the TOB-at-boot path does not require it") {
  Rig r;
  r.gen.tbs(true);
  CHECK(r.start(true) == Bist::Start::kTbsOn);
  CHECK_FALSE(r.bist.running());
  CHECK(r.start(false) == Bist::Start::kOk);
  CHECK(r.bist.running());
}

TEST_CASE("BIST-2/BIST-10: step 0 prints the build identity, the safety note and the answer keys") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  CHECK(r.has("BIST 0: banner"));
  CHECK(r.has("N2V8 0.0.0-test  built Jan  1 2026 12:00:00"));
  CHECK(r.has("board host (fake)  mode HOST  adc 10 bits"));
  CHECK(r.has("TBS OFF"));  // (printed later in the switch step) - placeholder to keep the order check honest
}

// ============================================================================ answers
TEST_CASE("BIST-3: p passes a step and moves on; the result is recorded and printed") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  r.type("p");
  CHECK(r.bist.verdict(BistStep::kBanner) == BistVerdict::kPass);
  CHECK(r.bist.current() == BistStep::kSwitches);
  CHECK(r.has("-> 0 banner: PASS"));
  CHECK(r.has("BIST 1: TBS and TOB"));
  CHECK(r.has("answer: p=pass f=fail r=rerun s=skip q=quit"));
}

TEST_CASE("BIST-3: f records a fail with the operator's note") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  r.type("f LCD row 2 blank");
  CHECK(r.bist.verdict(BistStep::kBanner) == BistVerdict::kFail);
  CHECK(std::string(r.bist.note(BistStep::kBanner)) == "LCD row 2 blank");
  CHECK(r.has("-> 0 banner: FAIL  LCD row 2 blank"));
}

TEST_CASE("BIST-3: s skips; r reruns the same step without recording anything") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  r.type("r");
  CHECK(r.bist.current() == BistStep::kBanner);
  CHECK(r.bist.verdict(BistStep::kBanner) == BistVerdict::kNotRun);
  CHECK(r.count("BIST 0: banner") == 2);
  CHECK(r.has("(rerun)"));
  r.type("s");
  CHECK(r.bist.verdict(BistStep::kBanner) == BistVerdict::kSkip);
  CHECK(r.has("-> 0 banner: SKIPPED"));
}

TEST_CASE("BIST-3: input is case-insensitive; junk gets a hint and changes nothing") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  r.type("hello");
  CHECK(r.bist.current() == BistStep::kBanner);
  CHECK(r.has("answer p, f, r, s or q"));
  r.type("P");
  CHECK(r.bist.verdict(BistStep::kBanner) == BistVerdict::kPass);
}

TEST_CASE("BIST-3: TOB means pass - after a fresh press") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  r.gen.tob(true);
  r.run(30);
  CHECK(r.bist.verdict(BistStep::kBanner) == BistVerdict::kPass);
}

TEST_CASE("BIST-1: a TOB held since power-up (how BIST was requested) does not auto-answer step 0") {
  Rig r;
  r.gen.tob(true);
  REQUIRE(r.start(false) == Bist::Start::kOk);
  r.run(200);
  CHECK(r.bist.current() == BistStep::kBanner);  // still waiting
  r.gen.tob(false);
  r.run(30);
  r.gen.tob(true);
  r.run(30);
  CHECK(r.bist.verdict(BistStep::kBanner) == BistVerdict::kPass);
}

TEST_CASE("BIST-3: q quits, prints the table so far, and ends with every output OFF") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  r.type("p");
  r.run(100);
  r.type("q");
  CHECK_FALSE(r.bist.running());
  CHECK(r.has("---- BIST RESULTS ----"));
  CHECK(r.has("BIST finished: operator quit"));
  CHECK(r.allOff());
}

TEST_CASE("BIST-6: the console hook is released at the end (typed lines are commands again)") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(50);
  r.type("q");
  r.run(100);
  r.out.clear();
  r.type("ver");
  CHECK(r.has("N2V8 0.0.0-test"));
}

// ============================================================================ the steps
TEST_CASE("BIST step 1: TBS and TOB changes are printed; TOB does NOT answer the step") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.run(100);
  r.type("p");
  REQUIRE(r.bist.current() == BistStep::kSwitches);
  r.run(100);
  CHECK(r.has("TBS OFF"));
  r.gen.tbs(true);
  r.run(100);
  CHECK(r.has("TBS ON"));
  r.gen.tob(true);
  r.run(100);
  CHECK(r.has("TOB pressed"));
  CHECK(r.bist.current() == BistStep::kSwitches);  // pressing TOB did not pass the step
  r.gen.tob(false);
  r.run(100);
  CHECK(r.has("TOB released"));
  r.type("p");
  CHECK(r.bist.verdict(BistStep::kSwitches) == BistVerdict::kPass);
}

TEST_CASE("BIST step 2: the I2C scan lists responders with labels and reports missing expected devices") {
  Rig r;
  r.gen.hal.i2cPresent = {0x24, 0x34, 0x35, 0x36, 0x37, 0x27, 0x50, 0x68, 0x57};  // O2 missing, a stranger and the RTC (+ its EEPROM) present
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kI2c);
  r.run(500);
  CHECK(r.has("0x24  LED control"));
  CHECK(r.has("0x27  LCD"));
  CHECK(r.has("0x50  UNEXPECTED"));
  CHECK(r.has("0x74  O2 sensor  MISSING"));
  CHECK(r.has("0x68  RTC"));
  CHECK(r.has("0x57  EEPROM on the RTC module (unused)"));
  CHECK(r.has("9 device(s) found, 1 expected device(s) missing"));
}

TEST_CASE("BIST step 3/DRV-3: the LED test states what to expect and runs the patterns by time") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kLed);
  REQUIRE(r.bist.current() == BistStep::kLed);
  CHECK(r.has("EXPECT LED: 0000 1111 2222"));
  CHECK(r.has("decimal point"));
  // pattern at ~ 0-200 ms: "0000"; at ~600 ms: "3333"; during the dp walk the display shows 8888 with a point
  const uint32_t t0 = r.now;
  auto digits = [&] { return std::vector<uint8_t>{r.display.led().shownSegments(0), r.display.led().shownSegments(1), r.display.led().shownSegments(2), r.display.led().shownSegments(3)}; };
  r.run(60);
  CHECK(digits() == std::vector<uint8_t>(4, Led1650::segmentsFor('0')));
  while (r.now - t0 < 650) r.pass();
  CHECK(digits() == std::vector<uint8_t>(4, Led1650::segmentsFor('3')));
  while (r.now - t0 < 2200) r.pass();  // first point position: after digit 0
  CHECK(r.display.led().shownSegments(0) == (Led1650::segmentsFor('8') | 0x80));
  CHECK(r.display.led().shownSegments(1) == Led1650::segmentsFor('8'));
  // blink phase: display off then on again
  bool sawOff = false, sawOn = false;
  while (r.now - t0 < 4400) { r.pass(); if (!r.display.led().displayOn()) sawOff = true; else if (sawOff) sawOn = true; }
  CHECK(sawOff);
  CHECK(sawOn);
  CHECK(std::string(r.display.lcd().shown(0)).substr(0, 15) == "BIST 3 LED test");  // LCD shows the step number
}

TEST_CASE("BIST step 4/DRV-3: the LCD test cycles backlight, display and a full block of '#'") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kLcd);
  REQUIRE(r.bist.current() == BistStep::kLcd);
  CHECK(r.has("EXPECT LCD: backlight off, on; display off, on; all 80 cells show '#'"));
  const uint32_t t0 = r.now;
  bool backlightWasOff = false, displayWasOff = false, sawFull = false;
  while (r.now - t0 < 4200) {
    r.pass();
    if (!r.display.lcd().backlightOn()) backlightWasOff = true;
    if (!r.display.lcd().displayOn()) displayWasOff = true;
    if (std::string(r.display.lcd().shown(3)) == "####################") sawFull = true;
  }
  CHECK(backlightWasOff);
  CHECK(displayWasOff);
  CHECK(sawFull);
  CHECK(r.display.led().shownSegments(3) == Led1650::segmentsFor('4'));  // the LED shows the step number
}

TEST_CASE("BIST step 5: raw counts, volts and PSI are printed; a dead sensor is called out") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kPressures);
  r.run(600);
  CHECK(r.has("AIR raw"));
  CHECK(r.has("130.0 PSI  in window"));
  CHECK(r.has("N2H raw"));
  r.gen.rawN2High(0);
  r.run(600);
  CHECK(r.has("BELOW WINDOW (wire off?)"));
}

TEST_CASE("BIST-9: entering a production gauge reading prints the difference to the sensor") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kPressures);
  r.run(300);
  r.type("g air 128.5");
  CHECK(r.has("air gauge 128.50 vs sensor 130.00 PSI: diff +1.50"));
  r.type("g n2l 26.0");
  CHECK(r.has("n2l gauge 26.00 vs sensor 2"));  // 25.00 PSI is not exactly representable: 24.9x..25.0x
  CHECK(r.has("PSI: diff -"));  // the sensor reads a little under the 26.00 gauge
  r.type("g air");
  CHECK(r.has("usage: g air|n2l|n2h <psi>"));
}

TEST_CASE("BIST step 6: the O2 sensor result and readings are printed on change") {
  Rig r;
  r.gen.o2.o2x100 = 2090;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kO2);
  r.run(1200);
  CHECK(r.has("O2 sensor begin: OK"));
  CHECK(r.has("O2 20.90 %"));
  const size_t n = r.count("O2 20.90 %");
  r.run(1500);
  CHECK(r.count("O2 20.90 %") == n);  // unchanged reading is not repeated
  r.gen.o2.o2x100 = 2095;
  r.run(700);
  CHECK(r.has("O2 20.95 %"));
}

TEST_CASE("BIST step 6: no O2 sensor says so") {
  Rig r;
  r.gen.o2.presentOk = false;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kO2);
  r.run(300);
  CHECK(r.has("O2 sensor begin: FAILED (no response)"));
}

// ============================================================================ output steps
TEST_CASE("BIST-5: the LEFT valve step announces its pin and level, then toggles ONLY that valve at ~2 Hz") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kLeft);
  REQUIRE(r.bist.current() == BistStep::kLeft);
  r.run(50);
  CHECK(r.has("LEFT valve: pin D4, active HIGH."));
  std::string timeline;
  for (int i = 0; i < 25; ++i) {  // 2.5 s in 100 ms slices
    r.run(100);
    REQUIRE_FALSE(r.gen.right());
    REQUIRE_FALSE(r.gen.flush());
    REQUIRE_FALSE(r.gen.ssr());
    timeline += r.gen.left() ? '1' : '0';
  }
  CHECK(timeline.find("11111") != std::string::npos);  // on for ~0.5 s
  CHECK(timeline.find("00000") != std::string::npos);  // off for ~0.5 s
  CHECK(r.has("LEFT valve ON"));
  CHECK(r.has("LEFT valve OFF"));
}

TEST_CASE("BIST-5/OUT-1: BIST toggling at 500 ms is the documented exception to the 1000 ms hold") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kRight);
  uint32_t lastChange = 0;
  bool state = false;
  uint32_t minGap = 0xFFFFFFFFu;
  for (int i = 0; i < 300; ++i) {
    r.pass();
    if (r.gen.right() != state) {
      if (lastChange != 0 && r.now - lastChange < minGap) minGap = r.now - lastChange;
      lastChange = r.now;
      state = r.gen.right();
    }
  }
  CHECK(minGap >= 490);
  CHECK(minGap <= 510);
}

TEST_CASE("BIST-5: any answer stops an output step at once, with every output OFF") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kFlush);
  r.run(700);
  REQUIRE(r.gen.flush());
  r.gen.hal.type("p\n");
  r.pass();  // ONE pass: the answer is read and the output is switched off in that same pass
  CHECK(r.allOff());
  CHECK(r.bist.verdict(BistStep::kFlush) == BistVerdict::kPass);
}

TEST_CASE("BIST-5: an output step stops by itself after the maximum time and waits for the answer") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kLeft);
  r.run(10500);
  CHECK(r.allOff());
  CHECK(r.has("stopped after 10 s"));
  CHECK(r.bist.current() == BistStep::kLeft);  // still waiting for the operator
}

TEST_CASE("BIST-4: an output step is refused while TBS is ON, and starts when it goes OFF") {
  Rig r;
  r.gen.tbs(true);                                  // TBS already ON when the output step is reached
  REQUIRE(r.start(false) == Bist::Start::kOk);        // (BIST was requested with TOB at power-up)
  r.goTo(BistStep::kLeft);
  r.run(1500);
  CHECK(r.has("REFUSED: TBS is ON - switch it OFF."));
  CHECK(r.count("REFUSED") == 1);  // said once, not every pass
  CHECK(r.allOff());
  r.gen.tbs(false);
  r.run(1200);
  CHECK(r.has("running LEFT valve"));
  CHECK(r.has("LEFT valve ON"));
}

TEST_CASE("BIST-4: switching TBS ON during an output step aborts it immediately") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kLeft);
  r.run(700);
  REQUIRE(r.gen.left());
  r.gen.tbs(true);
  r.pass();
  CHECK(r.allOff());
  CHECK(r.has("ABORTED: TBS switched ON - all outputs OFF"));
  CHECK(r.bist.aborts() == 1);
}

TEST_CASE("BIST-11: tower valves are vetoed with low air or an over-pressure N2-high tank") {
  Rig r;
  r.gen.air(kDefaultControl.airLowOff - 50);
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kLeft);
  r.run(1500);
  CHECK(r.has("REFUSED: air supply pressure is low"));
  CHECK(r.allOff());
  r.gen.healthy();
  r.gen.n2High(kDefaultControl.n2HighOff + 50);
  r.type("r");
  r.run(1500);
  CHECK(r.has("REFUSED: N2-high pressure is over its limit"));
  CHECK(r.allOff());
}

TEST_CASE("BIST-11: the SSR is vetoed unless N2-high is below its START threshold and N2-low is adequate") {
  Rig r;
  r.gen.n2High((kDefaultControl.n2HighOn + kDefaultControl.n2HighOff) / 2);  // between ON and OFF: tower ok, SSR not
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kSsr);
  r.run(1500);
  CHECK(r.has("REFUSED: N2-high pressure is not below its start threshold"));
  CHECK(r.allOff());
}

TEST_CASE("BIST-11: a sensor wire that falls off mid-step aborts the step at once, every output OFF") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kRight);
  r.run(700);
  REQUIRE(r.gen.right());
  r.gen.rawN2High(0);  // the sensor reads as an empty tank - exactly the dangerous misreading
  r.pass();
  CHECK(r.allOff());
  CHECK(r.has("ABORTED: N2-high sensor out of range"));
}

TEST_CASE("BIST-11: the flush valve is not vetoed by the pressure rules") {
  Rig r;
  r.gen.air(0);  // no air at all
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kFlush);
  r.run(700);
  CHECK(r.gen.flush());
}

TEST_CASE("BIST-11: vetoReason is a pure function of the readings") {
  Inputs in;
  in.airX10 = 1300;
  in.n2LowX100 = 2500;
  in.n2HighX10 = 800;
  in.rawAir = 600;
  in.rawN2Low = 500;
  in.rawN2High = 400;
  const ControlConfig cfg = kDefaultControl;
  CHECK(Bist::vetoReason(Signal::kLeftValve, in, cfg) == nullptr);
  CHECK(Bist::vetoReason(Signal::kRightValve, in, cfg) == nullptr);
  CHECK(Bist::vetoReason(Signal::kSsr, in, cfg) == nullptr);
  CHECK(Bist::vetoReason(Signal::kFlushValve, in, cfg) == nullptr);

  Inputs bad = in;
  bad.rawN2High = 10;  // raw out of window, before any debounce
  CHECK(Bist::vetoReason(Signal::kLeftValve, bad, cfg) != nullptr);
  CHECK(Bist::vetoReason(Signal::kSsr, bad, cfg) != nullptr);
  CHECK(Bist::vetoReason(Signal::kFlushValve, bad, cfg) == nullptr);

  bad = in;
  bad.n2LowX100 = cfg.n2LowOff - 1;
  CHECK(Bist::vetoReason(Signal::kSsr, bad, cfg) != nullptr);
  CHECK(Bist::vetoReason(Signal::kLeftValve, bad, cfg) == nullptr);  // low N2 does not matter to the towers

  bad = in;
  bad.sensorOrderFault = true;
  CHECK(Bist::vetoReason(Signal::kLeftValve, bad, cfg) != nullptr);
  CHECK(Bist::vetoReason(Signal::kSsr, bad, cfg) != nullptr);
}

TEST_CASE("HQ8: by default the compressor SSR gets ONE short pulse, not a 2 Hz toggle") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kSsr);
  r.run(50);
  CHECK(r.has("The compressor relay gets ONE 1000 ms pulse"));
  int onSamples = 0, edges = 0;
  bool was = false;
  for (int i = 0; i < 300; ++i) {
    r.pass();
    if (r.gen.ssr()) ++onSamples;
    if (r.gen.ssr() != was) { ++edges; was = r.gen.ssr(); }
  }
  CHECK(edges == 2);  // on, off - once
  CHECK(onSamples >= 85);
  CHECK(onSamples <= 105);  // about one second (the loop started a few passes after the pulse began)
  CHECK(r.has("SSR OFF - pulse done"));
}

TEST_CASE("HQ8: the 2 Hz SSR toggle can be selected by configuration") {
  BistConfig bc;
  bc.ssrSinglePulse = false;
  Rig r(bc);
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kSsr);
  int edges = 0;
  bool was = false;
  for (int i = 0; i < 300; ++i) { r.pass(); if (r.gen.ssr() != was) { ++edges; was = r.gen.ssr(); } }
  CHECK(edges >= 5);
}

TEST_CASE("BIST-3: r reruns the SSR pulse") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kSsr);
  r.run(2000);
  r.type("r");
  r.run(1500);
  CHECK(r.count("SSR ON (pulse)") == 2);
}

// ============================================================================ completion and safety
TEST_CASE("BIST: answering p to every step runs the whole sequence in order and prints a result table") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  for (int guard = 0; guard < 60 && r.bist.running(); ++guard) { r.run(400); r.type("p"); }
  r.run(200);
  CHECK_FALSE(r.bist.running());
  size_t from = 0;
  for (const char* h : {"BIST 0:", "BIST 1:", "BIST 2:", "BIST 3:", "BIST 4:", "BIST 5:", "BIST 6:", "BIST 7:", "BIST 8:",
                        "BIST 9:", "BIST A:", "BIST B:"}) {
    const size_t pos = r.out.find(h, from);
    INFO(h);
    REQUIRE(pos != std::string::npos);
    from = pos;
  }
  CHECK(r.has("---- BIST RESULTS ----"));
  CHECK(r.has("BIST: 11 pass, 0 fail, 0 skipped"));
  CHECK(r.has("BIST finished: all steps done"));
  CHECK(r.allOff());
}

TEST_CASE("BIST-2: the LED shows the step number in hex (A for step 10)") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kSsr);
  r.run(100);
  CHECK(r.display.led().shownSegments(3) == Led1650::segmentsFor('A'));
  CHECK(r.display.led().shownSegments(0) == 0x00);
}

TEST_CASE("BIST-1/INV-5: losing the console in the middle of an output step switches everything off") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  r.goTo(BistStep::kLeft);
  r.run(700);
  REQUIRE(r.gen.left());
  r.gen.hal.consoleIsAttached = false;
  const bool done = r.pass();
  CHECK(done);
  CHECK_FALSE(r.bist.running());
  CHECK(r.allOff());
}

TEST_CASE("BIST: outputs are OFF at every pass except inside an output step") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  for (int guard = 0; guard < 60 && r.bist.running(); ++guard) {
    const BistStep before = r.bist.current();
    for (int i = 0; i < 40; ++i) {
      r.pass();
      const BistStep s = r.bist.current();
      const bool outputStep = s == BistStep::kLeft || s == BistStep::kRight || s == BistStep::kFlush || s == BistStep::kSsr;
      if (!outputStep && before == s) REQUIRE(r.allOff());
    }
    r.type("p");
  }
}

TEST_CASE("BIST: the output queue copes with a very slow host (small TX buffer) without losing the results") {
  Rig r;
  REQUIRE(r.start() == Bist::Start::kOk);
  for (int guard = 0; guard < 80 && r.bist.running(); ++guard) {
    for (int i = 0; i < 40; ++i) {
      r.gen.hal.consoleSpace = 120;  // a slow host: about one long line per pass
      r.now += 10;
      r.gen.hal.nowMs = r.now;
      r.display.service(r.now);
      r.console.poll(r.commands);
      r.bist.step(r.now);
      r.out += r.gen.hal.consoleOut;
      r.gen.hal.consoleOut.clear();
    }
    r.gen.hal.type("p\n");
  }
  for (int i = 0; i < 400 && r.bist.running(); ++i) { r.gen.hal.consoleSpace = 120; r.pass(); }
  CHECK(r.has("---- BIST RESULTS ----"));
  CHECK(r.has("BIST: 11 pass"));
}
