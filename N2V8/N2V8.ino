// N2V8.ino — PSA nitrogen generator controller (UNO R4 Minima / UNO R4 WiFi).      VERSION 8.1.8  (N2_VERSION in src/BuildConfig.h)
//
// All behaviour lives in src/ and is tested on the host; this file is only glue.
//   build mode (src/BuildConfig.h): DIAG (default) = diagnostics only, no controllers;
//                                   FIELD = production; BENCH = home bench.
// Requirements: docs/Requirements.md      Plan: docs/Project_Plan.md
//
// Objects are created on first use inside functions (not as globals) so that nothing touches hardware
// before the Arduino core has finished initialising.

// WHERE TO EDIT   (file:line, relative to this folder. In the Arduino IDE 2: open the file, then Ctrl+L = go to line)
// COMMON EDITS (air and N2-high PSI x10, N2-low PSI x100, times in ms)
//   src/ControlConfig.h:42     TOWER      airLowOff airLowOn towerFillMs towerOverlapMs
//   src/ControlConfig.h:43     COMPRESSOR n2LowOff n2LowOn n2HighOn n2HighOff
//   src/ControlConfig.h:44     O2         interval flush sample count retry timeout errorRetry warm-up mandatory
//   src/ControlConfig.h:45     OUTPUTS/FAULTS  minHold sensorFaultSamples faultHold orderMargin orderHold
//   src/Config.h:21            sensor valid window 0.5-4.5 V, fault window 0.4-4.6 V, full scales
//   src/app/App.h:35           watchdog ms, LCD layout, LCD start delay, fault screen cycle, log level
//   src/BuildConfig.h:23       build mode (DIAG default / FIELD), default log level, O2 mandatory
//   src/BoardPins.h:92         pins, active levels (kLcdAddress = 0x23 just above the boards)
// CODE BY AREA
//   src/app/App.cpp:167                loop(): POST mode / BIST / RUN
//   src/app/App.cpp:60                 setup(): outputs safe first, then the rest
//   src/core/System.cpp:47             one pass: inputs, controllers, invariants, outputs
//   src/core/Tower.cpp:43              TOWER controller
//   src/core/Compressor.cpp:39         COMPRESSOR controller
//   src/core/O2Controller.cpp:60       O2 controller (flush, sample, warm-up)
//   src/drivers/O2SensorDfrobot.cpp:31 O2 sensor read (DFRobot library adapter)
//   src/core/Sensors.cpp:16            pressure inputs, fault windows
//   src/core/Invariants.cpp:14         SAFETY rules (INV-1 .. INV-10)
//   src/core/OutputDriver.cpp:35       outputs: active levels, minimum hold
//   src/core/Faults.cpp:8              fault table (codes, text, severity)
//   src/ui/LcdScreens.cpp:68           LCD normal screen
//   src/ui/LedText.cpp:10              LED text
//   src/drivers/Lcd20x4.cpp:245        LCD driver
//   src/drivers/Led1650.cpp:59         LED driver
//   src/drivers/Rtc3231.cpp:7          RTC driver
//   src/selftest/Post.cpp:40           POST
//   src/selftest/Bist.cpp:535          BIST
//   src/ui/Commands.cpp:17             console commands
//   src/hal/HalArduino.cpp:98          hardware access: console, I2C, reset cause
//   N2V8.ino:104                       this file: setup() and loop() glue (:105)
// To refresh these line numbers after editing, run:   python3 deliverables/update_sketch_toc.py N2V8/N2V8.ino
// END WHERE TO EDIT

#include "src/BoardPins.h"
#include "src/BuildConfig.h"
#include "src/Config.h"
#include "src/app/App.h"
#include "src/drivers/NvmEeprom.h"
#include "src/drivers/O2SensorDfrobot.h"
#include "src/hal/HalArduino.h"

namespace {

n2::HalArduino& hal() {
  static n2::HalArduino instance;
  return instance;
}

n2::EepromNvm& nvm() {
  static n2::EepromNvm instance;
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
    o.bist.switchKeys = N2_BIST_SWITCH_KEYS;   // TOB/TBS answer the BIST on the bench only
    o.nvm = &nvm();                    // data flash: stored debounce times (docs/NVM_Layout.md)
    o.sketchVersion = N2_VERSION_HEX;
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
