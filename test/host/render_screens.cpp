// render_screens — prints the LCD/LED screens for review (not a test). Usage: render_screens
#include <cstdio>
#include <initializer_list>

#include "ui/LcdScreens.h"
#include "ui/LedText.h"

using namespace n2;

static void show(const char* title, const Screen& s) {
  std::printf("%s\n   0         1\n   01234567890123456789\n", title);
  for (int r = 0; r < kLcdRows; ++r) std::printf("  |%s|\n", s.row[r]);
  std::printf("\n");
}

static DisplayData normal() {
  DisplayData d;
  d.airX10 = 1234; d.n2LowX100 = 1234; d.n2HighX10 = 987;
  d.n2Valid = true; d.n2PercentX100 = 9999;
  d.tower = "LB"; d.compressor = "ON"; d.o2 = "S";
  d.left = true; d.right = false; d.flush = false; d.ssr = true;
  d.tbs = true;
  return d;
}

int main() {
  for (LcdLayout layout : {LcdLayout::kClearLabels, LcdLayout::kCompact}) {
    const char* name = layout == LcdLayout::kClearLabels ? "OPTION 1 (clear labels)" : "OPTION 2 (compact)";
    std::printf("=========== %s ===========\n\n", name);
    DisplayData d = normal();
    show("normal running", renderNormal(d, layout));

    DisplayData w = normal();
    w.warming = true; w.warmRemainingMs = 272000; w.tower = "OF"; w.o2 = "WM"; w.left = false;
    show("O2 warming up (4:32 left), tower held off", renderNormal(w, layout));

    DisplayData off;
    off.airX10 = 1234; off.n2LowX100 = 1234; off.n2HighX10 = 987;
    show("TBS off (everything disabled, N2% invalid)", renderNormal(off, layout));

    DisplayData stale = normal();
    stale.n2Stale = true; stale.o2 = "W";
    show("N2% stale (pressures out of range)", renderNormal(stale, layout));

    DisplayData hi = normal();
    hi.compressor = "HI"; hi.ssr = false;
    show("compressor stopped, N2-high too high", renderNormal(hi, layout));
  }

  std::printf("=========== FAULT SCREENS ===========\n\n");
  DisplayData f = normal();
  f.rawN2High = 12; f.adcBits = 10;
  f.faultCount = 2; f.faults[0] = FaultId::kN2HighSensor; f.faults[1] = FaultId::kO2Comm;
  show("fault 1 of 2 (sensor)", renderFault(f, 0));
  show("fault 2 of 2 (O2)", renderFault(f, 1));
  DisplayData ord = normal();
  ord.n2LowX100 = 2800; ord.n2HighX10 = 150;
  ord.faultCount = 1; ord.faults[0] = FaultId::kSensorOrder;
  show("N2L above N2H", renderFault(ord, 0));
  DisplayData wd = normal();
  wd.faultCount = 1; wd.faults[0] = FaultId::kWatchdogReset;
  show("watchdog reset (warning)", renderFault(wd, 0));

  std::printf("=========== BANNER ===========\n\n");
  show("startup", renderBanner("8.1.2", "UNO R4 Minima", "Oct  2 2026 14:05"));

  std::printf("=========== LED ===========\n");
  DisplayData e = normal();
  DisplayData off;
  auto led = [](const char* what, const LedText& t) { std::printf("  %-28s [%s] dot after %d\n", what, t.digit, t.dotAfter); };
  led("running 99.99%", renderLed(e, false));
  led("TBS off", renderLed(off, false));
  e.n2Valid = false; led("N2% invalid", renderLed(e, false));
  led("fault F03 shown", renderLed(f, true));
  return 0;
}
