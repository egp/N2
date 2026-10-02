// Golden-screen tests: DSP-1..DSP-8 (LCD text, LED text, fault alternation). Layouts: docs/LCD_Layouts.md.
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "TestSupport.h"
#include "core/Scaling.h"
#include "ui/LcdScreens.h"
#include "ui/LedText.h"
#include "ui/ScreenCycle.h"

using namespace n2;
using namespace n2::test;

namespace {

DisplayData running() {
  DisplayData d;
  d.airX10 = 1234;
  d.n2LowX100 = 1234;
  d.n2HighX10 = 987;
  d.n2Valid = true;
  d.n2PercentX100 = 9999;
  d.tower = "LB";
  d.compressor = "ON";
  d.o2 = "S";
  d.left = true;
  d.ssr = true;
  d.tbs = true;
  return d;
}

// Compare a screen with four expected rows (each must be exactly 20 characters).
void expectScreen(const Screen& s, const char* r0, const char* r1, const char* r2, const char* r3) {
  const char* want[4] = {r0, r1, r2, r3};
  for (int i = 0; i < 4; ++i) {
    INFO("row " << i);
    REQUIRE(std::string(want[i]).size() == 20);
    CHECK(std::string(s.row[i]) == std::string(want[i]));
  }
}

}  // namespace

TEST_CASE("DSP-2: number formatting is fixed width") {
  char v[8];
  formatX10(v, 1234);  CHECK(std::string(v) == "123.4");
  formatX10(v, 50);    CHECK(std::string(v) == "  5.0");
  formatX10(v, 0);     CHECK(std::string(v) == "  0.0");
  formatX100(v, 1234); CHECK(std::string(v) == "12.34");
  formatX100(v, 505);  CHECK(std::string(v) == " 5.05");
  formatX100(v, 9999); CHECK(std::string(v) == "99.99");
  formatCountdown(v, 272000);  CHECK(std::string(v) == " 4:32");
  formatCountdown(v, 300000);  CHECK(std::string(v) == " 5:00");
  formatCountdown(v, 1);       CHECK(std::string(v) == " 0:01");  // never shows 0:00 until warm
  formatCountdown(v, 59001);   CHECK(std::string(v) == " 1:00");
}

TEST_CASE("DSP-4 option 1: normal screen") {
  expectScreen(renderNormal(running(), LcdLayout::kClearLabels),
               "N2% 99.99  O2 S     ",
               "N2L 12.34 N2H  98.7 ",
               "CMP ON  TWR LB  LRFS",
               "AIR 123.4       1001");
}

TEST_CASE("DSP-4 option 2: normal screen (N2-low, N2-high and compressor on one line)") {
  expectScreen(renderNormal(running(), LcdLayout::kCompact),
               "N2% 99.99  O2 S     ",
               "L12.34 H 98.7 CMP:ON",
               "TWR LB          LRFS",
               "AIR 123.4       1001");
}

TEST_CASE("DSP-4: no TBS and no O2% anywhere on the LCD") {
  for (LcdLayout l : {LcdLayout::kClearLabels, LcdLayout::kCompact}) {
    const Screen s = renderNormal(running(), l);
    std::string all;
    for (int i = 0; i < 4; ++i) all += std::string(s.row[i]) + "\n";
    CHECK(all.find("TBS") == std::string::npos);
    CHECK(all.find("O2%") == std::string::npos);
  }
}

TEST_CASE("DSP-4: the LRFS bits stack under their letters, one bit per output") {
  DisplayData d = running();
  d.left = false; d.right = true; d.flush = true; d.ssr = false;
  const Screen s = renderNormal(d, LcdLayout::kClearLabels);
  CHECK(std::string(s.row[2]).substr(16, 4) == "LRFS");
  CHECK(std::string(s.row[3]).substr(16, 4) == "0110");
}

TEST_CASE("DSP-4/O2-6: during warm-up the countdown replaces N2% (both layouts)") {
  DisplayData d = running();
  d.warming = true;
  d.warmRemainingMs = 272000;
  d.tower = "OF";
  d.o2 = "WM";
  d.left = false;
  expectScreen(renderNormal(d, LcdLayout::kClearLabels),
               "WRM  4:32  O2 WM    ",
               "N2L 12.34 N2H  98.7 ",
               "CMP ON  TWR OF  LRFS",
               "AIR 123.4       0001");
  CHECK(std::string(renderNormal(d, LcdLayout::kCompact).row[0]) == "WRM  4:32  O2 WM    ");
}

TEST_CASE("DSP-3/DSP-4: everything disabled (TBS off): states OF, bits 0000, N2% invalid") {
  DisplayData d;
  d.airX10 = 1234; d.n2LowX100 = 1234; d.n2HighX10 = 987;
  expectScreen(renderNormal(d, LcdLayout::kClearLabels),
               "N2% --.--  O2 OF    ",
               "N2L 12.34 N2H  98.7 ",
               "CMP OF  TWR OF  LRFS",
               "AIR 123.4       0000");
}

TEST_CASE("O2-7: a stale N2% keeps its value and is marked with '*'") {
  DisplayData d = running();
  d.n2Stale = true;
  d.o2 = "W";
  CHECK(std::string(renderNormal(d, LcdLayout::kClearLabels).row[0]) == "N2% 99.99* O2 W     ");
}

TEST_CASE("§7.2: a stopped compressor shows why (LO / HI)") {
  DisplayData d = running();
  d.compressor = "HI";
  d.ssr = false;
  CHECK(std::string(renderNormal(d, LcdLayout::kClearLabels).row[2]).substr(0, 6) == "CMP HI");
  CHECK(std::string(renderNormal(d, LcdLayout::kCompact).row[1]).substr(14, 6) == "CMP:HI");
}

TEST_CASE("DSP-5: fault screen uses the full 20x4 and describes the fault") {
  DisplayData d = running();
  d.rawN2High = 12;
  d.faultCount = 2;
  d.faults[0] = FaultId::kN2HighSensor;
  d.faults[1] = FaultId::kO2Comm;
  expectScreen(renderFault(d, 0),
               "FAULT 1 OF 2 INHIBIT",
               "F03 N2H SENSOR RANGE",
               "RAW 12  0.05V       ",
               "TOWERS+SSR OFF      ");
  expectScreen(renderFault(d, 1),
               "FAULT 2 OF 2 INHIBIT",
               "F12 O2 SENSOR FAILED",
               "O2 STATE S          ",
               "ALL OUTPUTS OFF     ");
  CHECK(std::string(renderFault(d, 2).row[0]) == std::string(renderFault(d, 0).row[0]));  // wraps
}

TEST_CASE("DSP-5: F04 shows both readings; a warning-level fault says WARNING") {
  DisplayData d = running();
  d.n2LowX100 = 2800;
  d.n2HighX10 = 150;
  d.faultCount = 1;
  d.faults[0] = FaultId::kSensorOrder;
  expectScreen(renderFault(d, 0), "FAULT 1 OF 1 INHIBIT", "F04 N2L ABOVE N2H   ", "L28.00>H 15.0       ", "TOWERS+SSR OFF      ");
  d.faults[0] = FaultId::kWatchdogReset;
  expectScreen(renderFault(d, 0), "FAULT 1 OF 1 WARNING", "F30 WATCHDOG RESET  ", "                    ", "RESET WAS LOGGED    ");
}

TEST_CASE("DSP-5: the raw-count detail uses the ADC bit depth") {
  DisplayData d = running();
  d.adcBits = 12;
  d.rawAir = 4095;  // full scale = 5.00 V
  d.faultCount = 1;
  d.faults[0] = FaultId::kAirSensor;
  CHECK(std::string(renderFault(d, 0).row[2]) == "RAW 4095  5.00V     ");
}

TEST_CASE("every fault in the table renders a screen that fits 20 columns") {
  for (uint8_t i = 0; i < kFaultCount; ++i) {
    DisplayData d = running();
    d.faultCount = 1;
    d.faults[0] = static_cast<FaultId>(i);
    const Screen s = renderFault(d, 0);
    for (int r = 0; r < 4; ++r) CHECK(std::string(s.row[r]).size() == 20);
    CHECK(std::string(s.row[1]).substr(0, 1) == "F");
    CHECK(std::string(s.row[3]).find_first_not_of(' ') == 0);  // effect text present
  }
}

TEST_CASE("DSP-8: startup banner") {
  expectScreen(renderBanner("0.1.0-m2", "UNO R4 Minima", "Oct  2 2026 14:05"),
               "N2V8 0.1.0-m2       ", "UNO R4 Minima       ", "Oct  2 2026 14:05   ", "POST ...            ");
}

TEST_CASE("DSP-3: LED text") {
  DisplayData d = running();
  LedText t = renderLed(d, false);
  CHECK(std::string(t.digit) == "9999");
  CHECK(t.dotAfter == 1);

  d.n2PercentX100 = 905;
  CHECK(std::string(renderLed(d, false).digit) == "0905");

  d.n2Valid = false;
  CHECK(std::string(renderLed(d, false).digit) == "----");
  CHECK(renderLed(d, false).dotAfter == 1);

  d.tbs = false;  // TBS off: blank
  CHECK(std::string(renderLed(d, false).digit) == "    ");
  CHECK(renderLed(d, false).dotAfter == -1);

  d = running();
  d.warming = true;  // no N2% while warming up
  CHECK(std::string(renderLed(d, false).digit) == "----");
}

TEST_CASE("DSP-5: the LED shows Fnn of the first active fault while the fault screen is up") {
  DisplayData d = running();
  d.faultCount = 2;
  d.faults[0] = FaultId::kN2HighSensor;
  d.faults[1] = FaultId::kO2Comm;
  CHECK(std::string(renderLed(d, true).digit) == " F03");
  CHECK(std::string(renderLed(d, false).digit) == "9999");  // normal phase shows N2%
}

TEST_CASE("DSP-5: with no fault the normal screen is shown forever") {
  ScreenCycle c(4000);
  for (uint32_t t = 0; t < 60000; t += 100) {
    c.update(t, 0);
    CHECK_FALSE(c.showFault());
  }
}

TEST_CASE("DSP-5: a fault alternates fault screen and normal screen on the cycle time") {
  ScreenCycle c(4000);
  c.update(1000, 1);
  CHECK(c.showFault());                 // a new fault is shown at once
  c.update(4999, 1);
  CHECK(c.showFault());
  c.update(5000, 1);
  CHECK_FALSE(c.showFault());           // normal screen for 4 s
  c.update(8999, 1);
  CHECK_FALSE(c.showFault());
  c.update(9000, 1);
  CHECK(c.showFault());
  CHECK(c.faultIndex() == 0);           // only one fault: stays on index 0
}

TEST_CASE("DSP-5: several faults take turns: fault 1, normal, fault 2, normal, fault 1 ...") {
  ScreenCycle c(3000);
  std::string seq;
  for (uint32_t t = 0; t < 24000; t += 3000) {
    c.update(t, 2);
    seq += c.showFault() ? (c.faultIndex() == 0 ? "1" : "2") : "n";
  }
  CHECK(seq == "1n2n1n2n");
}

TEST_CASE("DSP-5: when the faults clear the screen returns to normal at once; a new fault shows at once") {
  ScreenCycle c(4000);
  c.update(0, 1);
  REQUIRE(c.showFault());
  c.update(500, 0);
  CHECK_FALSE(c.showFault());
  c.update(900, 1);
  CHECK(c.showFault());
}

TEST_CASE("DSP-5: the cycle is rollover-safe") {
  ScreenCycle c(4000);
  const uint32_t t0 = UINT32_MAX - 1000;
  c.update(t0, 1);
  REQUIRE(c.showFault());
  c.update(t0 + 3999, 1);   // wrapped, just under the cycle
  CHECK(c.showFault());
  c.update(t0 + 4000, 1);
  CHECK_FALSE(c.showFault());
}

TEST_CASE("INP-8: millivolts from raw counts at every bit depth") {
  CHECK(millivoltsFromRaw(0, 10) == 0);
  CHECK(millivoltsFromRaw(1023, 10) == 5000);
  CHECK(millivoltsFromRaw(102, 10) >= 495);
  CHECK(millivoltsFromRaw(102, 10) <= 500);
  CHECK(millivoltsFromRaw(4095, 12) == 5000);
  CHECK(millivoltsFromRaw(16383, 14) == 5000);
  CHECK(millivoltsFromRaw(8192, 14) >= 2499);
  CHECK(millivoltsFromRaw(8192, 14) <= 2501);
}

TEST_CASE("DSP-1: makeDisplayData reflects a running plant, and the faults of severity >= WARN") {
  Plant p(quickWarmConfig());
  p.reboot();
  p.tbs(true);
  REQUIRE(p.runUntil([&] { return p.ssr() && p.left(); }, 20000));
  DisplayData d = makeDisplayData(p.sys(), p.bits);
  CHECK(d.tbs);
  CHECK(d.ssr);
  CHECK(d.left);
  CHECK(std::string(d.compressor) == "ON");
  CHECK(d.faultCount == 0);
  CHECK(d.airX10 >= 1295);
  CHECK(d.airX10 <= 1305);

  p.rawN2High(0);
  p.run(100);
  d = makeDisplayData(p.sys(), p.bits);
  REQUIRE(d.faultCount == 1);
  CHECK(d.faults[0] == FaultId::kN2HighSensor);
  CHECK(d.rawN2High == 0);
  CHECK_FALSE(d.ssr);
  const Screen s = renderFault(d, 0);
  CHECK(std::string(s.row[1]) == "F03 N2H SENSOR RANGE");
  CHECK(std::string(s.row[2]) == "RAW 0  0.00V        ");
}
