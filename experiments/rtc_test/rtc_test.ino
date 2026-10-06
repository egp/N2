// ===========================================================================================
// rtc_test  VERSION 1.0   (2026-10-06)      <-- if you do not see this line, the IDE has an older copy
//                                               (minor versions are written in HEX: 1.A = 1.10)
//
// Tests the REAL firmware DS3231 real-time-clock driver (N2V8/src/drivers/Rtc3231.cpp, reached through the `src` symbolic
// link in this folder; recreate it with  ln -s ../../N2V8/src src  if missing) on the UNO R4 WiFi. Only the RTC module is needed.
// READ README.md (power-up checklist) FIRST.
//
// Steps run one after another (5 s each). The 12x8 LED matrix on the WiFi board is the back channel: left glyph = step number
// in hex, right glyph = a hex result (below), bottom row: pixel 1 lit = last RTC access succeeded, pixel 12 = heartbeat.
// For 2 s after boot the matrix shows the version in hex ("1" "0" = 1.0).
//
//  step  what happens                                                               right glyph on the matrix
//   0    probe address 0x68 (and scan the bus)                                       1 = the chip answers, 0 = no answer
//   1    read the time once a second and print it                                    the SECONDS digit of the RTC, live
//   2    read the oscillator-stop flag: has the clock ever lost power since set?      1 = time can be trusted, 0 = lost power
//   3    set the RTC to the sketch BUILD time, but only if the time is NOT trusted    1 = set, 0 = left alone, F = failed
//        (use the console command T to set it exactly: see below)
//   4    drift check: does the RTC advance exactly as fast as millis() over 5 s?      1 = within 1 s, 0 = off
//   5    read the DS3231's temperature sensor                                         1 = plausible (0..60 C), 0 = not
//   6    speed/reliability: 200 reads in a row, counts failures, prints microseconds   1 = no failures, 0 = failures
//        per read
//
// Console commands (115200):  g = run again  n = next step  p = pause/resume  s = scan I2C
//                             t = read and print the time now
//                             T YYYY-MM-DD HH:MM:SS = set the RTC to exactly that time, e.g.  T 2026-10-06 10:31:00
//
// Change log
//   1.0  first version
// ===========================================================================================
#define TEST_VERSION "1.0"
#define TEST_MAJOR 1
#define TEST_MINOR 0

#include <Arduino.h>
#include <Wire.h>

#include "src/BoardPins.h"
#include "src/core/DateTime.h"
#include "src/drivers/Rtc3231.h"
#include "src/hal/HalArduino.h"

#include <Arduino_LED_Matrix.h>
ArduinoLEDMatrix matrix;

static n2::HalArduino hal;
static n2::Rtc3231 rtc(hal, 0x68);

// ---- matrix ----
static const uint8_t kHex[16][7] = {
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}};
static void setPixel(uint32_t* frame, uint8_t row, uint8_t col) {
  const uint8_t index = static_cast<uint8_t>(row * 12 + col);
  frame[index / 32] |= 1UL << (31 - (index % 32));
}
static void drawHex(uint32_t* frame, uint8_t value, uint8_t leftColumn) {
  for (uint8_t row = 0; row < 7; ++row)
    for (uint8_t bit = 0; bit < 5; ++bit)
      if (kHex[value & 15][row] & (0x10 >> bit)) setPixel(frame, row, static_cast<uint8_t>(leftColumn + bit));
}

static const uint8_t kSteps = 7;
static uint8_t step = 0;
static uint32_t stepStart = 0;
static uint8_t result = 0;
static uint8_t subStep = 0;
static uint32_t mark = 0;
static uint32_t markSeconds = 0;
static bool paused = false, done = false, started = false;

static void showMatrix(uint32_t now) {
  static uint32_t next = 0;
  static bool heartbeat = false;
  if (static_cast<int32_t>(now - next) < 0) return;
  next = now + 250;
  heartbeat = !heartbeat;
  uint32_t frame[3] = {0, 0, 0};
  if (now < 2000) {  // boot splash: the version in hex
    drawHex(frame, TEST_MAJOR, 0);
    drawHex(frame, TEST_MINOR, 7);
    matrix.loadFrame(frame);
    next = now + 100;
    return;
  }
  drawHex(frame, step, 0);
  drawHex(frame, result, 7);
  if (rtc.lastOk()) setPixel(frame, 7, 0);
  if (heartbeat) setPixel(frame, 7, 11);
  matrix.loadFrame(frame);
}

static void scanBus(TwoWire& bus, const char* name) {
  Serial.print("  scan of "); Serial.print(name); Serial.print(": ");
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) { Serial.print("0x"); Serial.print(a, HEX); Serial.print(' '); ++found; }
  }
  Serial.println(found == 0 ? "(nothing answers)" : "");
}

static bool printTime(const char* prefix) {
  n2::DateTime t;
  char buf[24];
  if (rtc.read(t)) {
    n2::formatDateTime(buf, t);
    Serial.print(prefix); Serial.println(buf);
    result = static_cast<uint8_t>(t.second % 10);
    return true;
  }
  Serial.print(prefix); Serial.println("READ FAILED (no answer, or the chip holds an invalid time)");
  return false;
}

// Build time from the compiler's __DATE__ ("Oct  6 2026") and __TIME__ ("10:31:02").
static bool buildTime(n2::DateTime& out) {
  static const char* kMonths = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* d = __DATE__;
  uint8_t month = 0;
  for (uint8_t m = 0; m < 12; ++m)
    if (d[0] == kMonths[m * 3] && d[1] == kMonths[m * 3 + 1] && d[2] == kMonths[m * 3 + 2]) month = static_cast<uint8_t>(m + 1);
  char date[11], time[9];
  snprintf(date, sizeof date, "%.4s-%02u-%02u", d + 7, static_cast<unsigned>(month), static_cast<unsigned>(atoi(d + 4)));
  snprintf(time, sizeof time, "%s", __TIME__);
  return n2::parseDateTime(date, time, out);
}

static void enterStep(uint8_t s, uint32_t now) {
  step = s;
  stepStart = now;
  mark = now;
  subStep = 0;
  result = 0;
  Serial.print("\n=== STEP "); Serial.print(s, HEX); Serial.print(": ");
  switch (s) {
    case 0: {
      Serial.println("probe 0x68");
      const bool ok = rtc.present();
      Serial.print("  DS3231 at 0x68: "); Serial.println(ok ? "answers" : "NO ANSWER");
      scanBus(Wire, "Wire (A4 SDA / A5 SCL)");
      Serial.println("  (a typical DS3231 module also shows an EEPROM at 0x57)");
      result = ok ? 1 : 0;
      break;
    }
    case 1: Serial.println("reading the time once a second; the seconds must count up. The matrix right glyph shows the seconds digit.");
            printTime("  time: "); break;
    case 2: {
      Serial.println("oscillator-stop flag");
      bool valid = false;
      if (rtc.timeValid(valid)) {
        Serial.println(valid ? "  flag CLEAR: the clock has kept running since the time was set (trustworthy)"
                             : "  flag SET: the clock lost power at some point; the time is NOT trustworthy until set");
        result = valid ? 1 : 0;
      } else {
        Serial.println("  could not read the status register");
      }
      break;
    }
    case 3: {
      Serial.println("set the time if it is not trusted");
      bool valid = false;
      if (!rtc.timeValid(valid)) { Serial.println("  could not read the status register"); result = 0xF; break; }
      if (valid) { Serial.println("  time is trusted: left alone (use the T command to set it exactly)"); result = 0; break; }
      n2::DateTime t;
      char buf[24];
      if (buildTime(t) && rtc.set(t)) {
        n2::formatDateTime(buf, t);
        Serial.print("  set to the sketch BUILD time "); Serial.print(buf); Serial.println(" (approximate: use T to set it exactly)");
        result = 1;
      } else {
        Serial.println("  SET FAILED");
        result = 0xF;
      }
      break;
    }
    case 4: {
      Serial.println("drift check over 5 s: RTC seconds versus millis()");
      n2::DateTime t;
      if (rtc.read(t)) { markSeconds = n2::secondsSince2000(t); subStep = 1; }
      else Serial.println("  could not read the time");
      break;
    }
    case 5: {
      Serial.println("temperature");
      int16_t c = 0;
      if (rtc.temperatureX100(c)) {
        Serial.print("  "); Serial.print(c / 100); Serial.print('.'); Serial.print(abs(c % 100) < 10 ? "0" : ""); Serial.print(abs(c % 100));
        Serial.println(" C");
        result = (c >= 0 && c <= 6000) ? 1 : 0;
      } else {
        Serial.println("  could not read the temperature");
      }
      break;
    }
    case 6: {
      Serial.println("200 reads in a row (speed and reliability)");
      uint32_t failures = 0;
      n2::DateTime t;
      const uint32_t t0 = micros();
      for (int i = 0; i < 200; ++i) if (!rtc.read(t)) ++failures;
      const uint32_t us = micros() - t0;
      Serial.print("  "); Serial.print(failures); Serial.print(" failures, "); Serial.print(us / 200); Serial.println(" microseconds per read()");
      result = failures == 0 ? 1 : 0;
      break;
    }
  }
}

static void finishStep(uint32_t now) {
  Serial.print("  step "); Serial.print(step, HEX); Serial.print(" done; RTC i2cErrors so far = "); Serial.println(rtc.i2cErrors());
  if (step + 1 < kSteps) {
    enterStep(step + 1, now);
  } else {
    done = true;
    Serial.println("\n=== SEQUENCE FINISHED ===");
    Serial.println("Type g to run again, t to read the time, or T YYYY-MM-DD HH:MM:SS to set it.");
  }
}

static void runStep(uint32_t now) {
  const uint32_t t = now - stepStart;
  if (step == 1 && now - mark >= 1000) {   // once a second
    mark = now;
    printTime("  time: ");
  }
  if (step == 4 && subStep == 1 && t >= 5000) {
    n2::DateTime d;
    if (rtc.read(d)) {
      const uint32_t rtcElapsed = n2::secondsSince2000(d) - markSeconds;
      const long diff = static_cast<long>(rtcElapsed) - static_cast<long>(t / 1000);
      Serial.print("  RTC advanced "); Serial.print(rtcElapsed); Serial.print(" s while millis() advanced "); Serial.print(t / 1000);
      Serial.print(" s (difference "); Serial.print(diff); Serial.println(" s; within 1 s is a pass)");
      result = (diff >= -1 && diff <= 1) ? 1 : 0;
    } else {
      Serial.println("  could not read the time");
    }
    subStep = 2;
  }
}

// ---- console line handling ----
static char lineBuf[48];
static uint8_t lineLen = 0;

static void handleLine(const char* line, uint32_t now) {
  if (line[0] == 'T' && line[1] == ' ') {
    char date[16] = {}, time[16] = {};
    n2::DateTime t;
    if (sscanf(line + 2, "%15s %15s", date, time) == 2 && n2::parseDateTime(date, time, t) && rtc.set(t)) {
      printTime("  RTC set; now reads: ");
    } else {
      Serial.println("  could not set: usage  T 2026-10-06 10:31:00   (or the write failed)");
    }
    return;
  }
  switch (line[0]) {
    case 'g': done = false; enterStep(0, now); break;
    case 'n': if (!done) finishStep(now); break;
    case 'p': paused = !paused; Serial.println(paused ? "paused" : "running"); break;
    case 't': printTime("  time: "); break;
    case 's':
      Serial.println("\n=== I2C scan (every address 0x08-0x77)");
      scanBus(Wire, "Wire  (A4 = SDA, A5 = SCL)");
      Wire1.begin();
      scanBus(Wire1, "Wire1 (the Qwiic connector)");
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);   // the R4 WiFi core does not do this for us
  pinMode(LED_BUILTIN, OUTPUT);
  matrix.begin();
  hal.i2cBegin();
  delay(100);
  Serial.println("\n==== rtc_test version " TEST_VERSION " (UNO R4 WiFi) ====");
  Serial.println("Testing the firmware's Rtc3231 driver. Commands: g n p s t, and  T YYYY-MM-DD HH:MM:SS  to set the time");
  Serial.println("(the matrix shows the version in hex for 2 s; the test starts after that)");
}

void loop() {
  const uint32_t now = millis();
  digitalWrite(LED_BUILTIN, (now / 500) % 2);
  showMatrix(now);

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; handleLine(lineBuf, now); lineLen = 0; }
    } else if (lineLen + 1 < sizeof lineBuf) {
      lineBuf[lineLen++] = static_cast<char>(c);
    }
  }

  if (!started) {
    if (now < 2000) return;
    started = true;
    enterStep(0, now);
  }
  if (done || paused) return;
  runStep(now);
  if (now - stepStart >= (step == 6 ? 3000u : 5000u)) finishStep(now);
}
