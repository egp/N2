// N2V8.ino — PSA nitrogen generator controller (UNO R4 Minima / UNO R4 WiFi).
//
// All behaviour lives in src/ and is tested on the host; this file is only glue.
//   build mode (src/BuildConfig.h): DIAG (default) = diagnostics only, no controllers;
//                                   FIELD = production; BENCH = home bench.
// Requirements: docs/Requirements.md      Plan: docs/Project_Plan.md
//
// Objects are created on first use inside functions (not as globals) so that nothing touches hardware
// before the Arduino core has finished initialising.

#include "src/BoardPins.h"
#include "src/BuildConfig.h"
#include "src/Config.h"
#include "src/app/App.h"
#include "src/drivers/O2SensorDfrobot.h"
#include "src/hal/HalArduino.h"

namespace {

n2::HalArduino& hal() {
  static n2::HalArduino instance;
  return instance;
}

n2::O2SensorDfrobot& o2() {
  static n2::O2SensorDfrobot instance(hal(), n2::kBoard.addrO2);
  return instance;
}

// Lives in RAM that a reset does not clear (O2 warm-up credit, Requirements O2-6a; disabled until bench-verified).
n2::WarmRecord warmRecord __attribute__((section(".noinit")));

n2::App& app() {
  static n2::ControlConfig cfg = [] {
    n2::ControlConfig c = n2::kDefaultControl;
    c.o2Mandatory = N2_O2_MANDATORY;
    return c;
  }();
  static n2::AppOptions options = [] {
    n2::AppOptions o;
    o.controllersEnabled = N2_CONTROLLERS_ENABLED;
    o.logLevel = n2::LogLevel::N2_DEFAULT_LOG_LEVEL;
#if defined(N2_NO_WATCHDOG)
    o.watchdogEnabled = false;  // WDT-5: debugging only
#endif
    return o;
  }();
  static n2::App instance(hal(), n2::kBoard, cfg, o2(), n2::makeBuildInfo(__DATE__, __TIME__), &warmRecord, options);
  return instance;
}

}  // namespace

void setup() { app().setup(); }
void loop() { app().loop(); }
