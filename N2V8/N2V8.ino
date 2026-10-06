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

// The O2 warm-up record (Requirements O2-6a; the feature stays disabled until the bench tests pass). It lives at a fixed
// RAM address that survives resets (not .noinit: see Config.h). If the heap could ever reach that address, use no record.
extern "C" char __HeapLimit;  // from the core's linker script
n2::WarmRecord* warmRecord() {
  const uintptr_t address = n2::kWarmRecordAddress;
  if (reinterpret_cast<uintptr_t>(&__HeapLimit) < address + sizeof(n2::WarmRecord)) return nullptr;
  return reinterpret_cast<n2::WarmRecord*>(address);
}

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
  static n2::App instance(hal(), n2::kBoard, cfg, o2(), n2::makeBuildInfo(__DATE__, __TIME__), warmRecord(), options);
  return instance;
}

}  // namespace

void setup() { app().setup(); }
void loop() { app().loop(); }
