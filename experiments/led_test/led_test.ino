// ===========================================================================================
// led_test  VERSION 1.0   (2026-10-06)      <-- if you do not see this line, the IDE has an older copy
//                                               (minor versions are written in HEX: 1.A = 1.10)
//
// Tests the REAL firmware LED driver (N2V8/src/drivers/Led1650.cpp, reached through the `src` link in this folder: the TM1650 4-digit, 8-segment display on I2C)
// on the UNO R4 WiFi. The driver and the Arduino HAL come straight from the firmware tree, so this is the
// exact code that ships. Needs only the LED module (SDA -> A4, SCL -> A5, VCC, GND); no LCD, no other sensors.
//
// It runs the 11 steps below, one after another (5 s each, 20 s for step A), and prints on the serial console what the
// display SHOULD show. The 12x8 LED matrix on the WiFi board is the back channel:
//     left glyph  = the step number in HEX (0-A)
//     right glyph = a hex result for the step (see the table)
//     bottom row  = pixel 1 lit: LED driver healthy;  pixel 12: heartbeat
//
//  step  what the LED should show                                        right glyph on the matrix
//   0    nothing yet: I2C probe of 0x24, 0x34..0x37                       number of addresses that answered (5 = all)
//   1    "8888" every segment lit, plus ONE decimal point (after digit 2)  hex count of I2C writes used
//   2    0000 1111 2222 ... 9999, one every 0.5 s                         writes used (a digit change = 4 writes)
//   3    A B C D E F patterns: "ABCD" then "EF--"                         writes used
//   4    a decimal point walks across "8888"                              writes used
//   5    "----" dashes                                                    writes used
//   6    change ONE digit at a time ("1234" -> "1239"): only ONE write    writes for the single-digit change (1 = good)
//   7    display OFF 1 s, ON 1 s (content kept)                           writes used (2 = the control byte twice)
//   8    blank (all segments off)                                         writes used
//   9    "1234" held (a steady reference pattern)                         writes used
//   A    FAILURE TEST: unplug the LED's SDA wire when told, then replug   0 = healthy, 1 = driver reports failure
//
// Console commands (Serial Monitor at 115200, or via Claude):   g = run the sequence again   n = next step   p = pause/resume
//
// Change log
//   1.0  first version
// ===========================================================================================
#define TEST_VERSION "1.0"

#include <Arduino.h>
#include <Wire.h>

// The firmware's own code. `src` in this folder is a SYMBOLIC LINK to ../../N2V8/src (an Arduino sketch cannot see files
// outside its own folder), so the Arduino IDE builds exactly the code that ships. (If the link is missing on your machine,
// recreate it with:  ln -s ../../N2V8/src src   run inside this folder.)
#include "src/BoardPins.h"
#include "src/drivers/Led1650.h"
#include "src/hal/HalArduino.h"

#include <Arduino_LED_Matrix.h>
ArduinoLEDMatrix matrix;

// A HAL that counts the I2C writes the driver makes, so we can check "only changed digits are written".
class CountingHal : public n2::HalArduino {
 public:
  uint32_t writes = 0;
  uint32_t failures = 0;
  bool i2cWrite(uint8_t address, const uint8_t* data, size_t n) override {
    const bool ok = n2::HalArduino::i2cWrite(address, data, n);
    ++writes;
    if (!ok) ++failures;
    return ok;
  }
};
static CountingHal hal;
static n2::Led1650 led(hal, n2::kBoard.addrLed, n2::kBoard.addrLedDigits);

// ---- matrix (see the layout in the header) ----
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

// ---- the test sequence ----
static const uint8_t kSteps = 11;
static uint8_t step = 0;
static uint32_t stepStart = 0;
static uint32_t writesAtStepStart = 0;
static uint8_t result = 0;         // the hex result shown on the right of the matrix
static uint8_t subStep = 0;        // position inside a step that has several patterns
static uint32_t lastSub = 0;
static bool paused = false;
static bool done = false;
static uint8_t stepWrites[kSteps];

static n2::LedText text(const char* d, int8_t dot) {
  n2::LedText t;
  for (int i = 0; i < 4; ++i) t.digit[i] = d[i];
  t.digit[4] = '\0';
  t.dotAfter = dot;
  return t;
}

static void showMatrix(uint32_t now) {
  static uint32_t next = 0;
  static bool heartbeat = false;
  if (static_cast<int32_t>(now - next) < 0) return;
  next = now + 500;
  heartbeat = !heartbeat;
  uint32_t frame[3] = {0, 0, 0};
  drawHex(frame, step, 0);
  drawHex(frame, result, 7);
  if (led.healthy()) setPixel(frame, 7, 0);
  if (heartbeat) setPixel(frame, 7, 11);
  matrix.loadFrame(frame);
}

static uint32_t stepLength(uint8_t s) { return s == 0 ? 3000 : (s == 10 ? 25000 : 5000); }

static void enterStep(uint8_t s, uint32_t now) {
  step = s;
  stepStart = now;
  subStep = 0;
  lastSub = now;
  writesAtStepStart = hal.writes;
  result = 0;
  Serial.print("\n=== STEP "); Serial.print(s, HEX); Serial.print(": ");
  switch (s) {
    case 0: {
      Serial.println("I2C probe");
      uint8_t found = 0;
      const uint8_t addrs[5] = {n2::kBoard.addrLed, static_cast<uint8_t>(n2::kBoard.addrLedDigits), static_cast<uint8_t>(n2::kBoard.addrLedDigits + 1),
                                static_cast<uint8_t>(n2::kBoard.addrLedDigits + 2), static_cast<uint8_t>(n2::kBoard.addrLedDigits + 3)};
      for (uint8_t a : addrs) {
        const bool ok = hal.i2cProbe(a);
        found += ok ? 1 : 0;
        Serial.print("  0x"); Serial.print(a, HEX); Serial.println(ok ? "  answers" : "  NO ANSWER");
      }
      Serial.print("  "); Serial.print(found); Serial.println(" of 5 addresses answer (all 5 expected)");
      result = found;
      break;
    }
    case 1:  Serial.println("EXPECT '8888' (every segment of all four digits lit) with ONE decimal point, after the 2nd digit");
             led.setText(text("8888", 1)); break;
    case 2:  Serial.println("EXPECT 0000 1111 2222 3333 4444 5555 6666 7777 8888 9999, one pattern every 0.5 s"); break;
    case 3:  Serial.println("EXPECT 'ABCD' for 2.5 s then 'EF--' for 2.5 s"); led.setText(text("ABCD", -1)); break;
    case 4:  Serial.println("EXPECT '8888' with a decimal point stepping left to right (after digit 1, 2, 3, 4), 1 s each"); break;
    case 5:  Serial.println("EXPECT four dashes '----'"); led.setText(text("----", -1)); break;
    case 6:  Serial.println("EXPECT '1234', then only the last digit changes to 9 -> '1239' (the driver must write ONE digit, not four)");
             led.setText(text("1234", -1)); break;
    case 7:  Serial.println("EXPECT the display switches OFF for 1 s, then ON again with the same content"); led.setText(text("1234", -1)); break;
    case 8:  Serial.println("EXPECT the display completely blank"); led.setText(text("    ", -1)); break;
    case 9:  Serial.println("EXPECT a steady '1234'"); led.setText(text("1234", -1)); break;
    case 10: Serial.println("FAILURE TEST. In 3 s the console will ask you to UNPLUG the LED's SDA wire; plug it back when told.");
             led.setText(text("5678", -1)); break;
  }
  if (s != 0) Serial.println("  (describe what you actually see; the matrix shows this step's number)");
}

static void finishStep(uint32_t now) {
  stepWrites[step] = static_cast<uint8_t>(hal.writes - writesAtStepStart);
  Serial.print("  step "); Serial.print(step, HEX); Serial.print(" used "); Serial.print(hal.writes - writesAtStepStart);
  Serial.print(" I2C writes, driver "); Serial.println(led.healthy() ? "healthy" : "REPORTS A FAILURE");
  if (step + 1 < kSteps) {
    enterStep(step + 1, now);
  } else {
    done = true;
    Serial.println("\n=== SEQUENCE FINISHED ===");
    Serial.print("totals: "); Serial.print(hal.writes); Serial.print(" I2C writes, "); Serial.print(hal.failures); Serial.println(" failed");
    Serial.print("driver i2cErrors() = "); Serial.println(led.i2cErrors());
    Serial.println("Type g to run again.");
  }
}

static void runStep(uint32_t now) {
  const uint32_t t = now - stepStart;
  switch (step) {
    case 2:  // digits 0..9, 0.5 s each
      if (subStep < 10 && t >= subStep * 500u) {
        const char c = static_cast<char>('0' + subStep);
        const char d[5] = {c, c, c, c, 0};
        led.setText(text(d, -1));
        ++subStep;
      }
      break;
    case 3:
      if (t >= 2500 && subStep == 0) { led.setText(text("EF--", -1)); subStep = 1; }
      break;
    case 4:
      if (t >= subStep * 1000u && subStep < 4) { led.setText(text("8888", static_cast<int8_t>(subStep))); ++subStep; }
      break;
    case 6:
      if (t >= 2500 && subStep == 0) {
        const uint32_t before = hal.writes;
        led.setText(text("1239", -1));
        subStep = 1;
        lastSub = before;  // remember the count so the single-digit change can be measured next pass
      } else if (subStep == 1 && t >= 2600) {
        result = static_cast<uint8_t>(hal.writes - lastSub);
        Serial.print("  writes for the single-digit change: "); Serial.println(result);
        subStep = 2;
      }
      break;
    case 7:
      if (t >= 1500 && subStep == 0) { led.setDisplayOn(false); subStep = 1; }
      else if (t >= 2500 && subStep == 1) { led.setDisplayOn(true); subStep = 2; }
      break;
    case 10:
      if (t >= 3000 && subStep == 0) { Serial.println("  >>> UNPLUG the LED's SDA wire NOW (leave SCL, power and ground connected)"); subStep = 1; }
      if (subStep == 1 && t >= 4000) { led.setText(text("9999", -1)); subStep = 2; }  // forces a write that will fail
      if (subStep >= 2 && subStep < 20 && t >= 4000u + (subStep - 2) * 1000u) {
        Serial.print("  t+"); Serial.print(t / 1000); Serial.print("s  driver "); Serial.print(led.healthy() ? "healthy" : "REPORTS FAILURE");
        Serial.print("  i2cErrors="); Serial.println(led.i2cErrors());
        result = led.healthy() ? 0 : 1;
        ++subStep;
        if (subStep == 12) { Serial.println("  >>> PLUG THE SDA WIRE BACK IN NOW"); }
      }
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);   // the R4 WiFi core does not do this for us
  pinMode(LED_BUILTIN, OUTPUT);
  matrix.begin();
  hal.i2cBegin();
  led.begin(millis());
  delay(100);
  Serial.println("\n==== led_test version " TEST_VERSION " (UNO R4 WiFi) ====");
  Serial.println("Testing the firmware's Led1650 driver. Commands: g = run again, n = next step, p = pause/resume");
  enterStep(0, millis());
}

void loop() {
  const uint32_t now = millis();
  digitalWrite(LED_BUILTIN, (now / 500) % 2);
  led.service(now);
  showMatrix(now);

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'g') { done = false; enterStep(0, now); }
    else if (c == 'n' && !done) finishStep(now);
    else if (c == 'p') { paused = !paused; Serial.println(paused ? "paused" : "running"); }
  }

  if (done || paused) return;
  runStep(now);
  if (now - stepStart >= stepLength(step)) finishStep(now);
}
