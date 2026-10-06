// ===========================================================================================
// stage1  VERSION 1.0   (2026-10-06)       <-- if you do not see this line, the IDE has an older copy
//                                              (minor versions are written in HEX: 1.A = 1.10)
//
// N2V8 bring-up, STAGE 1: only the parts already proven on the bench, with POST and BIST for each.
//   reset cause  ·  20x4 LCD (I2C 0x27)  ·  DS3231 RTC (I2C 0x68)  ·  USB console  ·  12x8 LED matrix (UNO R4 WiFi only)
// Not in this stage: the TM1650 LED (waiting for hardware), the O2 sensor, the pressure sensors, the valves, all controllers.
//
// This file is only glue (Requirements ARC-1). All behaviour is in ../../N2V8/src (reached through the `src` symbolic link in
// this folder; recreate it with  ln -s ../../N2V8/src src  if it is missing) and is tested on the host.
// READ README.md FIRST (wiring and power-up checklist).
//
// Console (115200 baud): type  help.  The WiFi board cannot tell whether a PC is listening, so it repeats its banner every 3 s
// until you type something.
//
// 12x8 matrix:  first 2 s = the program version in hex ("8" "1" = 8.1).  Then: left glyph = check number being run,
//   right glyph = its result (0 none, 1 pass, 2 info, F fail), bottom row pixels 0..7 = checks that passed, pixel 11 = heartbeat.
//
// Change log
//   1.0  first version
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
