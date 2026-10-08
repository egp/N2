// Driver tests with golden I2C bytes: DRV-1, DRV-2, DSP-2, DSP-6, DSP-7 (TM1650 LED and HD44780 LCD).
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "FakeHal.h"
#include "drivers/Lcd20x4.h"
#include "drivers/Led1650.h"

using namespace n2;
using Bytes = std::vector<uint8_t>;

namespace {

const uint8_t kLedCtl = 0x24, kLedDig = 0x34, kLcd = 0x23;

LedText text(const char* digits, int8_t dot) {
  LedText t;
  for (int i = 0; i < 4; ++i) t.digit[i] = digits[i];
  t.digit[4] = '\0';
  t.dotAfter = dot;
  return t;
}

Screen screen(const char* r0, const char* r1 = "", const char* r2 = "", const char* r3 = "") {
  Screen s;
  const char* rows[4] = {r0, r1, r2, r3};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 20; ++c) s.row[r][c] = (c < static_cast<int>(std::string(rows[r]).size())) ? rows[r][c] : ' ';
    s.row[r][20] = '\0';
  }
  return s;
}

// Collect the writes made to one address since `from`.
std::vector<FakeHal::I2cWrite> writesTo(const FakeHal& h, uint8_t addr, size_t from = 0) {
  std::vector<FakeHal::I2cWrite> out;
  for (size_t i = from; i < h.i2cWrites.size(); ++i)
    if (h.i2cWrites[i].address == addr) out.push_back(h.i2cWrites[i]);
  return out;
}

}  // namespace

// ============================================================================ LED (TM1650)
TEST_CASE("DRV-2: LED segment table") {
  CHECK(Led1650::segmentsFor('0') == 0x3F);
  CHECK(Led1650::segmentsFor('1') == 0x06);
  CHECK(Led1650::segmentsFor('8') == 0x7F);
  CHECK(Led1650::segmentsFor('9') == 0x6F);
  CHECK(Led1650::segmentsFor('A') == 0x77);
  CHECK(Led1650::segmentsFor('F') == 0x71);
  CHECK(Led1650::segmentsFor('-') == 0x40);
  CHECK(Led1650::segmentsFor(' ') == 0x00);
  CHECK(Led1650::segmentsFor('?') == 0x00);
}

TEST_CASE("DRV-2: LED waits for power-up, then writes the control byte and the four digits") {
  FakeHal hal;
  hal.i2cPresent = {kLedCtl, 0x34, 0x35, 0x36, 0x37};
  Led1650 led(hal, kLedCtl, kLedDig);
  led.setText(text("9999", 1));
  led.begin(1000);
  led.service(1000);
  led.service(1049);
  CHECK(hal.i2cWrites.empty());  // 50 ms power-up wait: no I2C traffic yet
  led.service(1050);
  REQUIRE(hal.i2cWrites.size() == 5);
  CHECK(hal.i2cWrites[0].address == 0x24);  CHECK(hal.i2cWrites[0].bytes == Bytes{0x11});
  CHECK(hal.i2cWrites[1].address == 0x34);  CHECK(hal.i2cWrites[1].bytes == Bytes{0x6F});
  CHECK(hal.i2cWrites[2].address == 0x35);  CHECK(hal.i2cWrites[2].bytes == Bytes{0xEF});  // 9 with the decimal point
  CHECK(hal.i2cWrites[3].address == 0x36);  CHECK(hal.i2cWrites[3].bytes == Bytes{0x6F});
  CHECK(hal.i2cWrites[4].address == 0x37);  CHECK(hal.i2cWrites[4].bytes == Bytes{0x6F});
  CHECK(led.ready());
  CHECK(led.healthy());
  CHECK(led.inSync());
}

TEST_CASE("DSP-2: the LED rewrites only the digits that changed") {
  FakeHal hal;
  hal.i2cPresent = {kLedCtl, 0x34, 0x35, 0x36, 0x37};
  Led1650 led(hal, kLedCtl, kLedDig);
  led.setText(text("9999", 1));
  led.begin(0);
  led.service(100);
  hal.i2cWrites.clear();
  led.service(110);
  CHECK(hal.i2cWrites.empty());  // nothing changed: no traffic
  led.setText(text("9989", 1));
  CHECK_FALSE(led.inSync());
  led.service(120);
  REQUIRE(hal.i2cWrites.size() == 1);
  CHECK(hal.i2cWrites[0].address == 0x36);
  CHECK(hal.i2cWrites[0].bytes == Bytes{0x7F});
  led.setText(text("    ", -1));  // blank
  led.service(130);
  CHECK(hal.i2cWrites.size() == 1 + 4);
  CHECK(hal.i2cWrites.back().bytes == Bytes{0x00});
}

TEST_CASE("DRV-2: the LED shows the fault code ' F03' and dashes '----' with a point") {
  FakeHal hal;
  hal.i2cPresent = {kLedCtl, 0x34, 0x35, 0x36, 0x37};
  Led1650 led(hal, kLedCtl, kLedDig);
  led.setText(text(" F03", -1));
  led.begin(0);
  led.service(100);
  CHECK(writesTo(hal, 0x34)[0].bytes == Bytes{0x00});
  CHECK(writesTo(hal, 0x35)[0].bytes == Bytes{0x71});  // F
  CHECK(writesTo(hal, 0x36)[0].bytes == Bytes{0x3F});  // 0
  CHECK(writesTo(hal, 0x37)[0].bytes == Bytes{0x4F});  // 3
  led.setText(text("----", 1));
  led.service(110);
  CHECK(writesTo(hal, 0x35).back().bytes == Bytes{0x40 | 0x80});
}

TEST_CASE("BIST: the LED display can be switched off and on without losing the content") {
  FakeHal hal;
  hal.i2cPresent = {kLedCtl, 0x34, 0x35, 0x36, 0x37};
  Led1650 led(hal, kLedCtl, kLedDig);
  led.setText(text("1234", -1));
  led.begin(0);
  led.service(100);
  hal.i2cWrites.clear();
  led.setDisplayOn(false);
  led.service(110);
  REQUIRE(hal.i2cWrites.size() == 1);
  CHECK(hal.i2cWrites[0].bytes == Bytes{0x10});
  led.setDisplayOn(true);
  led.service(120);
  CHECK(hal.i2cWrites.back().bytes == Bytes{0x11});
  CHECK(hal.i2cWrites.size() == 2);  // digits were not rewritten
}

TEST_CASE("DRV-1/DSP-7: an absent LED is reported, retried once a second, and re-initialised when it returns") {
  FakeHal hal;  // nothing acknowledges
  Led1650 led(hal, kLedCtl, kLedDig);
  led.setText(text("1234", -1));
  led.begin(0);
  led.service(100);
  CHECK_FALSE(led.healthy());
  CHECK(led.i2cErrors() == 1);
  CHECK_FALSE(led.ready());
  const size_t before = hal.i2cWrites.size();
  led.service(500);  // inside the retry delay: no traffic at all (not even a probe)
  CHECK(hal.i2cWrites.size() == before);
  led.service(1100);  // retry time: probe fails, wait again
  CHECK_FALSE(led.healthy());
  CHECK(hal.i2cWrites.size() == before);

  hal.i2cPresent = {kLedCtl, 0x34, 0x35, 0x36, 0x37};  // plugged back in
  led.service(2100);
  CHECK(led.healthy());
  CHECK(led.ready());
  CHECK(led.inSync());
  CHECK(writesTo(hal, 0x24, before).front().bytes == Bytes{0x11});
  CHECK(writesTo(hal, 0x37, before).size() == 1);  // all digits rewritten
}

TEST_CASE("DRV-1: a device that dies mid-run is detected on the next write") {
  FakeHal hal;
  hal.i2cPresent = {kLedCtl, 0x34, 0x35, 0x36, 0x37};
  Led1650 led(hal, kLedCtl, kLedDig);
  led.setText(text("1111", -1));
  led.begin(0);
  led.service(100);
  REQUIRE(led.healthy());
  hal.i2cPresent.clear();
  led.setText(text("2222", -1));
  led.service(200);
  CHECK_FALSE(led.healthy());
  CHECK(led.i2cErrors() == 1);
}

// ============================================================================ LCD (HD44780 via PCF8574)
namespace {
// Run the LCD until it is ready, one millisecond at a time.
uint32_t bringUp(FakeHal&, Lcd20x4& lcd, uint32_t start = 0) {
  lcd.begin(start);
  uint32_t t = start;
  for (; t < start + 500 && !lcd.ready(); ++t) lcd.service(t);
  return t;
}
}  // namespace

TEST_CASE("DRV-2: LCD initialisation - exact bytes and the HD44780 wait times") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.begin(0);

  struct Step { uint32_t at; Bytes bytes; };
  const std::vector<Step> expected = {
      {50, {0x3C, 0x38}},                          // nibble 0x30, wait 5 ms
      {55, {0x3C, 0x38}},                          // nibble 0x30, wait 5 ms
      {60, {0x3C, 0x38}},                          // nibble 0x30, wait 1 ms
      {61, {0x2C, 0x28}},                          // nibble 0x20 (4-bit mode), wait 1 ms
      {62, {0x2C, 0x28, 0x8C, 0x88}},              // command 0x28: function set
      {62, {0x0C, 0x08, 0xCC, 0xC8}},              // command 0x0C: display on
      {62, {0x0C, 0x08, 0x6C, 0x68}},              // command 0x06: entry mode
      {62, {0x0C, 0x08, 0x1C, 0x18}},              // command 0x01: clear, wait 2 ms
  };
  std::vector<uint32_t> times;
  for (uint32_t t = 0; t <= 64; ++t) {
    const size_t before = hal.i2cWrites.size();
    lcd.service(t);
    for (size_t i = before; i < hal.i2cWrites.size(); ++i) times.push_back(t);
  }
  REQUIRE(hal.i2cWrites.size() == expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    INFO("init write " << i);
    CHECK(hal.i2cWrites[i].address == kLcd);
    CHECK(hal.i2cWrites[i].bytes == expected[i].bytes);
    CHECK(times[i] == expected[i].at);
  }
  CHECK(lcd.ready());
}

TEST_CASE("DRV-2: nothing is written before the 50 ms power-up wait, and nothing blocks") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.begin(1000);
  for (uint32_t t = 1000; t < 1050; ++t) lcd.service(t);
  CHECK(hal.i2cWrites.empty());
  CHECK_FALSE(lcd.ready());
}

TEST_CASE("DRV-2: first character - cursor command then data, exact bytes") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("A"));
  uint32_t t = bringUp(hal, lcd);
  hal.i2cWrites.clear();
  lcd.service(t + 1);
  REQUIRE(hal.i2cWrites.size() == 2);
  CHECK(hal.i2cWrites[0].bytes == Bytes{0x8C, 0x88, 0x0C, 0x08});  // set DDRAM address 0x00 (command 0x80)
  CHECK(hal.i2cWrites[1].bytes == Bytes{0x4D, 0x49, 0x1D, 0x19});  // data 'A' = 0x41 (RS=1, backlight on)
  CHECK(std::string(lcd.shown(0)).substr(0, 2) == "A ");
}

TEST_CASE("DSP-2: the LCD writes in small pieces and ends up showing exactly the screen") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("N2% 99.99  O2 S", "N2L 12.34 N2H  98.7", "CMP ON  TWR LB  LRFS", "AIR 123.4       1001"));
  uint32_t t = bringUp(hal, lcd);
  int passes = 0;
  size_t maxPerPass = 0;
  while (!lcd.inSync() && passes < 100) {
    const size_t before = hal.i2cWrites.size();
    lcd.service(++t);
    maxPerPass = std::max(maxPerPass, hal.i2cWrites.size() - before);
    ++passes;
  }
  CHECK(lcd.inSync());
  CHECK(maxPerPass <= 2u * Lcd20x4::kCharsPerService);  // bounded bus time per loop pass
  CHECK(passes >= 5);                                   // ...which means it took several passes
  CHECK(std::string(lcd.shown(0)) == "N2% 99.99  O2 S     ");
  CHECK(std::string(lcd.shown(3)) == "AIR 123.4       1001");
}

TEST_CASE("DSP-2: a one-character change writes exactly one cursor command and one character") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("AIR 123.4", "", "", "N2% 99.99"));
  uint32_t t = bringUp(hal, lcd);
  while (!lcd.inSync()) lcd.service(++t);
  hal.i2cWrites.clear();
  lcd.service(++t);
  CHECK(hal.i2cWrites.empty());  // nothing changed: no traffic

  lcd.setScreen(screen("AIR 123.5", "", "", "N2% 99.99"));  // 4 -> 5 at row 0, col 8
  lcd.service(++t);
  REQUIRE(hal.i2cWrites.size() == 2);
  CHECK(hal.i2cWrites[0].bytes == Bytes{0x8C, 0x88, 0x8C, 0x88});  // address 0x08 = command 0x88: hi 0x88|... see below
  CHECK(hal.i2cWrites[1].bytes == Bytes{0x3D, 0x39, 0x5D, 0x59});  // '5' = 0x35: hi 0x30, lo 0x50
}

TEST_CASE("DRV-2: DDRAM row addresses for the four rows (0x00, 0x40, 0x14, 0x54)") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("a", "b", "c", "d"));
  uint32_t t = bringUp(hal, lcd);
  hal.i2cWrites.clear();
  while (!lcd.inSync()) lcd.service(++t);
  // Collect the cursor commands: every write whose RS bit (0x01 in the first byte pair) is clear.
  std::vector<uint8_t> cmdHi;
  for (const auto& w : hal.i2cWrites)
    if ((w.bytes[0] & 0x01) == 0) cmdHi.push_back(static_cast<uint8_t>((w.bytes[0] & 0xF0)));
  REQUIRE(cmdHi.size() == 4);
  // High nibbles of 0x80|addr: 0x80 (row0 0x00), 0xC0 (row1 0x40), 0x90 (row2 0x14), 0xD0 (row3 0x54)
  CHECK(cmdHi[0] == 0x80);
  CHECK(cmdHi[1] == 0xC0);
  CHECK(cmdHi[2] == 0x90);
  CHECK(cmdHi[3] == 0xD0);
}

TEST_CASE("DSP-2: consecutive changed characters share one cursor command") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("ABCDE"));
  uint32_t t = bringUp(hal, lcd);
  hal.i2cWrites.clear();
  lcd.service(++t);
  CHECK(hal.i2cWrites.size() == 1 + 5);  // one cursor command + five characters
}

TEST_CASE("DSP-2: after the last column the cursor is treated as unknown, so the next row sets it again") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("XXXXXXXXXXXXXXXXXXXX", "YYYYYYYYYYYYYYYYYYYY"));
  uint32_t t = bringUp(hal, lcd);
  hal.i2cWrites.clear();
  while (!lcd.inSync()) lcd.service(++t);
  int cursorCommands = 0;
  for (const auto& w : hal.i2cWrites) if ((w.bytes[0] & 0x01) == 0) ++cursorCommands;
  CHECK(cursorCommands == 2);  // one per row, none inside a row
  CHECK(std::string(lcd.shown(1)) == "YYYYYYYYYYYYYYYYYYYY");
}

TEST_CASE("BIST: backlight off/on is a plain byte, and later data bytes follow the backlight bit") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen(""));
  uint32_t t = bringUp(hal, lcd);
  while (!lcd.inSync()) lcd.service(++t);
  hal.i2cWrites.clear();
  lcd.setBacklight(false);
  lcd.service(++t);
  REQUIRE(hal.i2cWrites.size() == 1);
  CHECK(hal.i2cWrites[0].bytes == Bytes{0x00});
  lcd.setScreen(screen("A"));
  lcd.service(++t);
  CHECK(hal.i2cWrites.back().bytes == Bytes{0x45, 0x41, 0x15, 0x11});  // 'A' with the backlight bit clear
  lcd.setBacklight(true);
  lcd.service(++t);
  CHECK(hal.i2cWrites.back().bytes == Bytes{0x08});
}

TEST_CASE("BIST: display off/on is the HD44780 display-control command and keeps the content") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("keep me"));
  uint32_t t = bringUp(hal, lcd);
  while (!lcd.inSync()) lcd.service(++t);
  hal.i2cWrites.clear();
  lcd.setDisplayOn(false);
  lcd.service(++t);
  REQUIRE(hal.i2cWrites.size() == 1);
  CHECK(hal.i2cWrites[0].bytes == Bytes{0x0C, 0x08, 0x8C, 0x88});  // command 0x08
  lcd.setDisplayOn(true);
  lcd.service(++t);
  CHECK(hal.i2cWrites.back().bytes == Bytes{0x0C, 0x08, 0xCC, 0xC8});  // command 0x0C
  CHECK(lcd.inSync());
}

TEST_CASE("DRV-1/DSP-6: an absent LCD is reported without blocking anything") {
  FakeHal hal;  // nothing acknowledges
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("hello"));
  lcd.begin(0);
  for (uint32_t t = 0; t < 200; ++t) lcd.service(t);
  CHECK_FALSE(lcd.healthy());
  CHECK_FALSE(lcd.ready());
  CHECK(lcd.i2cErrors() >= 1);
}

TEST_CASE("DSP-7: an LCD that comes back is fully re-initialised and redrawn") {
  FakeHal hal;
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("hello"));
  lcd.begin(0);
  uint32_t t = 0;
  for (; t < 1200; ++t) lcd.service(t);
  REQUIRE_FALSE(lcd.healthy());
  hal.i2cPresent = {kLcd};
  for (; t < 4000 && !lcd.inSync(); ++t) lcd.service(t);
  CHECK(lcd.inSync());
  CHECK(lcd.healthy());
  CHECK(std::string(lcd.shown(0)) == "hello               ");
  // The re-init sequence ran again: a 0x30 nibble pair appears among the writes.
  bool sawInit = false;
  for (const auto& w : hal.i2cWrites) if (w.acked && w.bytes == Bytes{0x3C, 0x38}) sawInit = true;
  CHECK(sawInit);
}

TEST_CASE("DSP-7: an LCD that dies mid-run is detected, then recovers") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("one"));
  uint32_t t = bringUp(hal, lcd);
  while (!lcd.inSync()) lcd.service(++t);
  REQUIRE(lcd.healthy());
  hal.i2cPresent.clear();
  lcd.setScreen(screen("two"));
  lcd.service(++t);
  CHECK_FALSE(lcd.healthy());
  hal.i2cPresent = {kLcd};
  for (uint32_t end = t + 3000; t < end && !lcd.inSync(); ++t) lcd.service(t);
  CHECK(lcd.inSync());
  CHECK(lcd.healthy());
  CHECK(std::string(lcd.shown(0)).substr(0, 3) == "two");
}

TEST_CASE("DRV-3: each scheduled full rewrite starts with entry mode and return home, which cancels a stuck display shift") {
  FakeHal hal;
  hal.i2cPresent = {kLcd};
  Lcd20x4 lcd(hal, kLcd);
  lcd.setScreen(screen("A"));
  lcd.enableHealing();
  uint32_t t = bringUp(hal, lcd);
  hal.i2cWrites.clear();
  for (uint32_t i = 0; i < 400; ++i) lcd.service(t + i);   // the first rewrite is due 250 ms after the display is up
  int homes = 0;
  for (const auto& w : hal.i2cWrites)
    if (w.bytes == Bytes{0x0C, 0x08, 0x2C, 0x28}) ++homes;
  CHECK(homes >= 1);
  CHECK(std::string(lcd.shown(0)).substr(0, 1) == "A");
}
