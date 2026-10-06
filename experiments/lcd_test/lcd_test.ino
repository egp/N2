// ===========================================================================================
// lcd_test  VERSION 1.0   (2026-10-06)      <-- if you do not see this line, the IDE has an older copy
//                                               (minor versions are written in HEX: 1.A = 1.10)
//
// Tests the REAL firmware LCD driver (N2V8/src/drivers/Lcd20x4.cpp: a 20x4 HD44780 display behind a PCF8574 I2C backpack)
// on the UNO R4 WiFi. The driver and HAL come from the firmware tree through the `src` symbolic link in this folder
// (recreate it with  ln -s ../../N2V8/src src  if it is missing). ONLY the LCD is needed. READ README.md (power-up checklist) FIRST.
//
// The sketch runs the steps below, 5 s each, and prints on the serial console what the LCD SHOULD show. The 12x8 LED matrix on
// the WiFi board is the back channel:  left glyph = step number in hex, right glyph = a hex result, bottom row: pixel 1 lit =
// driver healthy, pixel 12 = heartbeat. For 2 s after boot the matrix shows the version in hex ("1" "0" = 1.0).
//
//  step  what the LCD should show                                          right glyph on the matrix
//   0    nothing yet: I2C probe of the backpack address 0x27 + scan        1 = the backpack answers, 0 = no answer
//   1    backlight on, then the banner text "LCD TEST 1.0" etc.            hex count of I2C writes used
//   2    every one of the 80 cells filled with '#'                         writes used
//   3    four DIFFERENT rows: "ROW 0 ........" ... "ROW 3" (row addressing) writes used
//   4    printable characters: ! to ~ in order, 20 per row                 writes used
//   5    change ONE character ("AIR 123.4" -> "AIR 123.5"): one cursor
//        command + one character = 2 writes                                writes for the change (2 = good)
//   6    backlight OFF for 1.5 s, then ON (text stays)                     writes used
//   7    display OFF for 1.5 s, then ON (text stays)                       writes used
//   8    the firmware's real normal screen (layout 1) with sample data     writes used
//   9    the real O2 warm-up screen "WRM  4:32 ..."                        writes used
//
// Console commands (115200):  g = run again   n = next step   p = pause/resume   s = scan all I2C addresses
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
#include "src/drivers/Lcd20x4.h"
#include "src/hal/HalArduino.h"
#include "src/ui/LcdScreens.h"

#include <Arduino_LED_Matrix.h>
ArduinoLEDMatrix matrix;

// A HAL that counts the I2C writes the driver makes.
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
static n2::Lcd20x4 lcd(hal, n2::kBoard.addrLcd);

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

static const uint8_t kSteps = 10;
static uint8_t step = 0;
static uint32_t stepStart = 0;
static uint32_t writesAtStepStart = 0;
static uint8_t result = 0;
static uint8_t subStep = 0;
static uint32_t mark = 0;
static bool paused = false, done = false, started = false;

static void showMatrix(uint32_t now) {
  static uint32_t next = 0;
  static bool heartbeat = false;
  if (static_cast<int32_t>(now - next) < 0) return;
  next = now + 500;
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
  if (lcd.healthy()) setPixel(frame, 7, 0);
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

static void enterStep(uint8_t s, uint32_t now) {
  step = s;
  stepStart = now;
  mark = now;
  subStep = 0;
  result = 0;
  writesAtStepStart = hal.writes;
  Serial.print("\n=== STEP "); Serial.print(s, HEX); Serial.print(": ");
  switch (s) {
    case 0: {
      Serial.println("I2C probe");
      const bool ok = hal.i2cProbe(n2::kBoard.addrLcd);
      Serial.print("  backpack 0x"); Serial.print(n2::kBoard.addrLcd, HEX); Serial.println(ok ? "  answers" : "  NO ANSWER");
      scanBus(Wire, "Wire (A4 SDA / A5 SCL)");
      result = ok ? 1 : 0;
      break;
    }
    case 1: Serial.println("EXPECT backlight on and the text: LCD TEST 1.0 / row 2 'UNO R4 WiFi' / row 3 'firmware Lcd20x4'");
            lcd.setScreen(n2::makeScreen("LCD TEST " TEST_VERSION, "Hello from the", "UNO R4 WiFi", "firmware Lcd20x4")); break;
    case 2: { Serial.println("EXPECT all 80 cells filled with '#'");
              const char* f = "####################"; lcd.setScreen(n2::makeScreen(f, f, f, f)); break; }
    case 3: Serial.println("EXPECT four different rows: ROW 0 / ROW 1 / ROW 2 / ROW 3 (each filled to the right edge with dots)");
            lcd.setScreen(n2::makeScreen("ROW 0 ..............", "ROW 1 ..............", "ROW 2 ..............", "ROW 3 ..............")); break;
    case 4: { Serial.println("EXPECT printable characters in order, 20 per row, starting with '!'");
              char rows[4][21];
              char c = '!';
              for (int r = 0; r < 4; ++r) { for (int i = 0; i < 20; ++i) rows[r][i] = c++; rows[r][20] = 0; }
              lcd.setScreen(n2::makeScreen(rows[0], rows[1], rows[2], rows[3])); break; }
    case 5: Serial.println("EXPECT 'AIR 123.4' on row 0, then (after 2.5 s) only the last digit changes to 5; that must be 2 I2C writes");
            lcd.setScreen(n2::makeScreen("AIR 123.4", "N2L 12.34", "", "")); break;
    case 6: Serial.println("EXPECT the backlight OFF for 1.5 s then ON again; the text stays"); lcd.setScreen(n2::makeScreen("BACKLIGHT TEST", "text must stay", "", "")); break;
    case 7: Serial.println("EXPECT the whole display OFF for 1.5 s then ON again with the same text"); lcd.setScreen(n2::makeScreen("DISPLAY OFF/ON TEST", "text must come back", "", "")); break;
    case 8: {
      Serial.println("EXPECT the firmware's normal screen:  N2% 99.99  O2 S / N2L 12.34 N2H  98.7 / CMP ON  TWR LB  LRFS / AIR 123.4       1001");
      n2::DisplayData d;
      d.airX10 = 1234; d.n2LowX100 = 1234; d.n2HighX10 = 987;
      d.n2Valid = true; d.n2PercentX100 = 9999;
      d.tower = "LB"; d.compressor = "ON"; d.o2 = "S";
      d.left = true; d.ssr = true;
      lcd.setScreen(n2::renderNormal(d, n2::LcdLayout::kClearLabels));
      break;
    }
    case 9: {
      Serial.println("EXPECT the O2 warm-up screen:  WRM  4:32  O2 WM / N2L 12.34 N2H  98.7 / CMP ON  TWR OF  LRFS / AIR 123.4       0001");
      n2::DisplayData d;
      d.airX10 = 1234; d.n2LowX100 = 1234; d.n2HighX10 = 987;
      d.warming = true; d.warmRemainingMs = 272000;
      d.tower = "OF"; d.compressor = "ON"; d.o2 = "WM";
      d.ssr = true;
      lcd.setScreen(n2::renderNormal(d, n2::LcdLayout::kClearLabels));
      break;
    }
  }
  if (s != 0) Serial.println("  (describe what you actually see; the matrix shows this step's number)");
}

static void finishStep(uint32_t now) {
  Serial.print("  step "); Serial.print(step, HEX); Serial.print(" used "); Serial.print(hal.writes - writesAtStepStart);
  Serial.print(" I2C writes, driver "); Serial.println(lcd.healthy() ? "healthy" : "REPORTS A FAILURE");
  if (step + 1 < kSteps) {
    enterStep(step + 1, now);
  } else {
    done = true;
    Serial.println("\n=== SEQUENCE FINISHED ===");
    Serial.print("totals: "); Serial.print(hal.writes); Serial.print(" I2C writes, "); Serial.print(hal.failures); Serial.println(" failed");
    Serial.print("driver i2cErrors() = "); Serial.println(lcd.i2cErrors());
    Serial.println("Type g to run again.");
  }
}

// Timed actions inside a step.
static void runStep(uint32_t now) {
  const uint32_t t = now - stepStart;
  switch (step) {
    case 5:
      if (subStep == 0 && t >= 2500) {
        mark = hal.writes;
        lcd.setScreen(n2::makeScreen("AIR 123.5", "N2L 12.34", "", ""));
        subStep = 1;
      } else if (subStep == 1 && t >= 3000) {
        result = static_cast<uint8_t>(hal.writes - mark);
        Serial.print("  writes for the single-character change: "); Serial.println(result);
        subStep = 2;
      }
      break;
    case 6:
      if (subStep == 0 && t >= 1000) { lcd.setBacklight(false); subStep = 1; }
      else if (subStep == 1 && t >= 2500) { lcd.setBacklight(true); subStep = 2; }
      break;
    case 7:
      if (subStep == 0 && t >= 1000) { lcd.setDisplayOn(false); subStep = 1; }
      else if (subStep == 1 && t >= 2500) { lcd.setDisplayOn(true); subStep = 2; }
      break;
    default: break;
  }
  if (step != 5 && step != 0) result = static_cast<uint8_t>(hal.writes - writesAtStepStart);
}

void setup() {
  Serial.begin(115200);   // the R4 WiFi core does not do this for us
  pinMode(LED_BUILTIN, OUTPUT);
  matrix.begin();
  hal.i2cBegin();
  lcd.begin(millis());
  delay(100);
  Serial.println("\n==== lcd_test version " TEST_VERSION " (UNO R4 WiFi) ====");
  Serial.println("Testing the firmware's Lcd20x4 driver. Commands: g = run again, n = next step, p = pause, s = scan I2C");
  Serial.println("(the matrix shows the version in hex for 2 s; the test starts after that)");
}

void loop() {
  const uint32_t now = millis();
  digitalWrite(LED_BUILTIN, (now / 500) % 2);
  lcd.service(now);
  showMatrix(now);

  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == 'g') { done = false; enterStep(0, now); }
    else if (c == 'n' && !done) finishStep(now);
    else if (c == 's') {
      Serial.println("\n=== I2C scan (every address 0x08-0x77)");
      scanBus(Wire, "Wire  (A4 = SDA, A5 = SCL)");
      Wire1.begin();
      scanBus(Wire1, "Wire1 (the Qwiic connector)");
    } else if (c == 'p') { paused = !paused; Serial.println(paused ? "paused" : "running"); }
  }

  if (!started) {  // wait for the version splash
    if (now < 2000) return;
    started = true;
    enterStep(0, now);
  }
  if (done || paused) return;
  runStep(now);
  if (now - stepStart >= 5000) finishStep(now);
}
