// ===========================================================================================
// stage2  VERSION 1.0   (2026-10-07)       <-- if you do not see this line, the IDE has an older copy
//                                              (minor versions are written in HEX: 1.A = 1.10)
//
// N2V8 bring-up, STAGE 2: stage 1 plus the 4-digit TM1650 LED display.
//   reset cause  ·  20x4 LCD (I2C 0x27)  ·  DS3231 RTC (I2C 0x68)  ·  4-digit LED (I2C 0x24, 0x34-0x37)  ·  console  ·  12x8 matrix (WiFi only)
// Not in this stage: the O2 sensor, the pressure sensors, the valves, all controllers.
//
// All behaviour is in ../../N2V8/src (reached through the `src` symbolic link in this folder; recreate it with  ln -s ../../N2V8/src src
// if it is missing) and is tested on the host. This file is only glue. READ README.md FIRST (wiring and power-up checklist).
//
// What you see:
//   LCD row 0: version and stage    row 1: one status per check, e.g. "RES+ LCD+ RTC+ LED+"  (+ ok, i info, F fail, - not run)
//          row 2: wall-clock time (or "no clock (RTC)")    row 3: POST state / uptime and reset cause
//   LED: "----" during POST, "FFFF" while POST is held, then the time HHMM with the middle dot blinking (or the uptime in seconds without an RTC)
//   Matrix: first 2 s the version in hex; then left glyph = check being run, right glyph = its result (0 none, 1 pass, 2 info, F fail),
//           bottom row = one pixel per passed check, pixel 9 = TBS ON, pixel 11 = heartbeat; the whole matrix lights while TOB is held.
// Console (115200): type help. The WiFi board repeats its banner every 3 s until you type something.
//
// Change log
//   1.0  first version (stage 1 + the LED; the LED and the LCD are rewritten continuously/periodically, see Led1650::setRefresh)
// ===========================================================================================

#include <Arduino.h>

#include "src/BoardPins.h"
#include "src/BuildConfig.h"
#include "src/app/Bringup.h"
#include "src/hal/HalArduino.h"
#include "src/ui/BuildInfo.h"
#include "src/ui/MatrixFrame.h"

// Debugging switch: build with  -DSTAGE1_NO_MATRIX  to leave the matrix completely untouched.
#if defined(ARDUINO_UNOR4_WIFI) && !defined(STAGE1_NO_MATRIX)
#include <Arduino_LED_Matrix.h>

// Draws on the real matrix; for the first 2 s it shows the version in hex instead.
class WifiMatrix : public n2::FrameSink {
 public:
  void begin() { matrix_.begin(); }
  void show(const uint32_t* frame) override {
    if (millis() < 2000) {
      uint32_t splash[3] = {0, 0, 0};
      n2::matrixDrawHex(splash, 8, 0);  // the "8" of N2V8 version 8.x
      n2::matrixDrawHex(splash, 1, 7);  // the minor version, in hex
      matrix_.loadFrame(splash);
      return;
    }
    matrix_.loadFrame(frame);
  }

 private:
  ArduinoLEDMatrix matrix_;
};
#endif

namespace {

n2::HalArduino& hal() {
  static n2::HalArduino instance;
  return instance;
}

#if defined(ARDUINO_UNOR4_WIFI) && !defined(STAGE1_NO_MATRIX)
WifiMatrix& wifiMatrix() {
  static WifiMatrix instance;
  return instance;
}
#endif

// Created on first use (inside a function) so nothing touches hardware before the Arduino core has finished starting.
n2::Bringup& app() {
  static n2::BringupOptions options = [] {
    n2::BringupOptions o;
    o.stage = "2";
#if defined(N2_NO_WATCHDOG)
    o.watchdogEnabled = false;
#endif
#if defined(STAGE1_LCD_START_MS)  // debugging: override how long the LCD is left alone after boot (default 2500 ms)
    o.lcdStartMs = STAGE1_LCD_START_MS;
#endif
    return o;
  }();
#if defined(ARDUINO_UNOR4_WIFI) && !defined(STAGE1_NO_MATRIX)
  static n2::Bringup instance(hal(), n2::kBoard, n2::makeBuildInfo(__DATE__, __TIME__), &wifiMatrix(), options);
#else
  static n2::Bringup instance(hal(), n2::kBoard, n2::makeBuildInfo(__DATE__, __TIME__), nullptr, options);
#endif
  return instance;
}

}  // namespace

void setup() {
#if defined(ARDUINO_UNOR4_WIFI) && !defined(STAGE1_NO_MATRIX)
  wifiMatrix().begin();
#endif
  app().setup();
}

void loop() { app().loop(); }
