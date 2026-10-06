// Tests for the DS3231 driver Rtc3231 against a register-file fake (Requirements RTC-1..RTC-4).
#include <catch2/catch_test_macros.hpp>

#include "FakeHal.h"
#include "drivers/Rtc3231.h"

using namespace n2;

namespace {
struct Rig {
  FakeHal hal;
  Rtc3231 rtc{hal, 0x68};
  Rig() { hal.i2cPresent = {0x68}; }
  std::vector<uint8_t>& regs() { hal.regPointer(0x68); return hal.i2cRegs[0x68]; }
};
}  // namespace

TEST_CASE("RTC-1: present() follows the chip's acknowledge") {
  Rig r;
  CHECK(r.rtc.present());
  r.hal.i2cPresent.clear();
  CHECK_FALSE(r.rtc.present());
  CHECK(r.rtc.i2cErrors() == 1);
}

TEST_CASE("RTC-1: BCD helpers") {
  CHECK(Rtc3231::bcdToDec(0x59) == 59);
  CHECK(Rtc3231::decToBcd(59) == 0x59);
  CHECK(Rtc3231::decToBcd(7) == 0x07);
  CHECK(Rtc3231::bcdOk(0x99));
  CHECK_FALSE(Rtc3231::bcdOk(0x1A));
  CHECK_FALSE(Rtc3231::bcdOk(0xA0));
}

TEST_CASE("RTC-2: set() writes the registers in BCD with 24-hour mode and the right weekday") {
  Rig r;
  REQUIRE(r.rtc.set({2026, 10, 6, 14, 31, 2}));
  const auto& g = r.regs();
  CHECK(g[0] == 0x02);   // seconds
  CHECK(g[1] == 0x31);   // minutes
  CHECK(g[2] == 0x14);   // hours, bit 6 clear = 24-hour mode
  CHECK(g[3] == 2);      // Tuesday
  CHECK(g[4] == 0x06);   // date
  CHECK(g[5] == 0x10);   // month (century bit clear)
  CHECK(g[6] == 0x26);   // year 2026
  // one write transaction of 8 bytes (pointer + 7 registers), then the status read-modify-write
  CHECK(r.hal.i2cWrites[0].bytes.size() == 8);
  CHECK(r.hal.i2cWrites[0].bytes[0] == 0x00);
}

TEST_CASE("RTC-2: set() then read() round-trips, including leap day and the year-2100 century bit") {
  Rig r;
  for (DateTime t : {DateTime{2026, 10, 6, 10, 31, 2}, DateTime{2024, 2, 29, 23, 59, 59}, DateTime{2000, 1, 1, 0, 0, 0},
                     DateTime{2099, 12, 31, 23, 59, 59}, DateTime{2100, 3, 1, 12, 0, 0}, DateTime{2199, 12, 31, 0, 0, 0}}) {
    REQUIRE(r.rtc.set(t));
    DateTime back{};
    REQUIRE(r.rtc.read(back));
    CHECK(back.year == t.year); CHECK(back.month == t.month); CHECK(back.day == t.day);
    CHECK(back.hour == t.hour); CHECK(back.minute == t.minute); CHECK(back.second == t.second);
  }
  CHECK(r.regs()[5] & 0x80);  // the last one (2199) set the century bit
}

TEST_CASE("RTC-2: set() refuses an impossible date without touching the bus") {
  Rig r;
  CHECK_FALSE(r.rtc.set({2026, 2, 30, 0, 0, 0}));
  CHECK(r.hal.i2cWrites.empty());
  CHECK(r.rtc.i2cErrors() == 0);  // not a bus error
}

TEST_CASE("RTC-2: read() decodes a 12-hour-mode chip correctly") {
  Rig r;
  auto& g = r.regs();
  g[0] = 0x00; g[1] = 0x15; g[3] = 3; g[4] = 0x06; g[5] = 0x10; g[6] = 0x26;
  DateTime t{};
  g[2] = 0x40 | 0x20 | 0x03;   // 12-hour mode, PM, 3 o'clock -> 15:15
  REQUIRE(r.rtc.read(t));
  CHECK(t.hour == 15);
  g[2] = 0x40 | 0x12;          // 12 AM -> 00
  REQUIRE(r.rtc.read(t));
  CHECK(t.hour == 0);
  g[2] = 0x40 | 0x20 | 0x12;   // 12 PM -> 12
  REQUIRE(r.rtc.read(t));
  CHECK(t.hour == 12);
  g[2] = 0x40 | 0x09;          // 9 AM -> 09
  REQUIRE(r.rtc.read(t));
  CHECK(t.hour == 9);
}

TEST_CASE("RTC-2: read() rejects corrupt registers (not BCD, or an impossible date) and counts the error") {
  Rig r;
  REQUIRE(r.rtc.set({2026, 10, 6, 10, 31, 2}));
  DateTime t{};
  auto& g = r.regs();
  g[1] = 0x7A;  // not BCD
  CHECK_FALSE(r.rtc.read(t));
  g[1] = 0x31;
  g[5] = 0x13;  // month 13
  CHECK_FALSE(r.rtc.read(t));
  g[5] = 0x10;
  g[4] = 0x32;  // day 32
  CHECK_FALSE(r.rtc.read(t));
  g[4] = 0x06;
  CHECK(r.rtc.read(t));  // fixed again
  CHECK(r.rtc.i2cErrors() == 3);
}

TEST_CASE("RTC-2: an absent chip makes read, set, temperature and valid-flag fail cleanly") {
  Rig r;
  r.hal.i2cPresent.clear();
  DateTime t{};
  int16_t temp = 0;
  bool valid = true;
  CHECK_FALSE(r.rtc.read(t));
  CHECK_FALSE(r.rtc.set({2026, 10, 6, 10, 31, 2}));
  CHECK_FALSE(r.rtc.temperatureX100(temp));
  CHECK_FALSE(r.rtc.timeValid(valid));
  CHECK_FALSE(r.rtc.lastOk());
  CHECK(r.rtc.i2cErrors() >= 4);
}

TEST_CASE("RTC-3: the oscillator-stop flag says whether the time can be trusted; set() clears it") {
  Rig r;
  r.regs()[0x0F] = 0x80;   // OSF set: the clock lost power at some point
  bool valid = true;
  REQUIRE(r.rtc.timeValid(valid));
  CHECK_FALSE(valid);
  REQUIRE(r.rtc.set({2026, 10, 6, 10, 31, 2}));
  REQUIRE(r.rtc.timeValid(valid));
  CHECK(valid);
  CHECK((r.regs()[0x0F] & 0x80) == 0);
}

TEST_CASE("RTC-3: set() preserves the other status bits when clearing OSF") {
  Rig r;
  r.regs()[0x0F] = 0x80 | 0x08;  // OSF + EN32kHz
  REQUIRE(r.rtc.set({2026, 10, 6, 10, 31, 2}));
  CHECK(r.regs()[0x0F] == 0x08);
}

TEST_CASE("RTC-4: temperature decodes positive, fractional and negative values (0.25 C steps)") {
  Rig r;
  int16_t t = 0;
  r.regs()[0x11] = 0x19; r.regs()[0x12] = 0x40;   // 25.25 C
  REQUIRE(r.rtc.temperatureX100(t));
  CHECK(t == 2525);
  r.regs()[0x11] = 0x19; r.regs()[0x12] = 0x00;   // 25.00
  REQUIRE(r.rtc.temperatureX100(t));
  CHECK(t == 2500);
  r.regs()[0x11] = 0x19; r.regs()[0x12] = 0xC0;   // 25.75
  REQUIRE(r.rtc.temperatureX100(t));
  CHECK(t == 2575);
  r.regs()[0x11] = 0xFA; r.regs()[0x12] = 0x80;   // -6 + 0.5 = -5.50
  REQUIRE(r.rtc.temperatureX100(t));
  CHECK(t == -550);
  r.regs()[0x11] = 0x00; r.regs()[0x12] = 0x00;   // 0.00
  REQUIRE(r.rtc.temperatureX100(t));
  CHECK(t == 0);
}

TEST_CASE("RTC-2: a successful call after a failed one clears lastOk()") {
  Rig r;
  r.hal.i2cPresent.clear();
  CHECK_FALSE(r.rtc.present());
  CHECK_FALSE(r.rtc.lastOk());
  r.hal.i2cPresent = {0x68};
  CHECK(r.rtc.present());
  CHECK(r.rtc.lastOk());
}
