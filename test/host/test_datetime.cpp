// Tests for DateTime.h: calendar helpers used by the RTC (Requirements RTC-1..RTC-3).
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "core/DateTime.h"

using namespace n2;

TEST_CASE("RTC-1: leap years follow the Gregorian rules") {
  CHECK(isLeapYear(2024));
  CHECK_FALSE(isLeapYear(2026));
  CHECK_FALSE(isLeapYear(2100));  // divisible by 100, not by 400
  CHECK(isLeapYear(2000));        // divisible by 400
  CHECK(daysInMonth(2024, 2) == 29);
  CHECK(daysInMonth(2026, 2) == 28);
  CHECK(daysInMonth(2026, 4) == 30);
  CHECK(daysInMonth(2026, 12) == 31);
  CHECK(daysInMonth(2026, 0) == 0);
  CHECK(daysInMonth(2026, 13) == 0);
}

TEST_CASE("RTC-1: validDateTime accepts real times and rejects impossible ones") {
  CHECK(validDateTime({2026, 10, 6, 10, 31, 2}));
  CHECK(validDateTime({2024, 2, 29, 23, 59, 59}));
  CHECK_FALSE(validDateTime({2026, 2, 29, 0, 0, 0}));   // not a leap year
  CHECK_FALSE(validDateTime({2026, 4, 31, 0, 0, 0}));
  CHECK_FALSE(validDateTime({2026, 13, 1, 0, 0, 0}));
  CHECK_FALSE(validDateTime({2026, 0, 1, 0, 0, 0}));
  CHECK_FALSE(validDateTime({2026, 1, 0, 0, 0, 0}));
  CHECK_FALSE(validDateTime({2026, 1, 1, 24, 0, 0}));
  CHECK_FALSE(validDateTime({2026, 1, 1, 0, 60, 0}));
  CHECK_FALSE(validDateTime({2026, 1, 1, 0, 0, 60}));
  CHECK_FALSE(validDateTime({1999, 12, 31, 0, 0, 0}));  // before the DS3231's range
  CHECK_FALSE(validDateTime({2200, 1, 1, 0, 0, 0}));
}

TEST_CASE("RTC-1: ISO weekday (Monday = 1) for known dates") {
  CHECK(isoWeekday(2026, 10, 6) == 2);   // Tuesday
  CHECK(isoWeekday(2000, 1, 1) == 6);    // Saturday
  CHECK(isoWeekday(2024, 2, 29) == 4);   // Thursday
  CHECK(isoWeekday(2100, 3, 1) == 1);    // Monday
  CHECK(isoWeekday(2026, 10, 4) == 7);   // Sunday
}

TEST_CASE("RTC-1: formatDateTime writes the ISO-like form") {
  char buf[24];
  formatDateTime(buf, {2026, 10, 6, 10, 31, 2});
  CHECK(std::string(buf) == "2026-10-06 10:31:02");
  formatDateTime(buf, {2000, 1, 1, 0, 0, 0});
  CHECK(std::string(buf) == "2000-01-01 00:00:00");
}

TEST_CASE("RTC-3: parseDateTime reads YYYY-MM-DD and HH:MM:SS (or HH:MM)") {
  DateTime t{};
  REQUIRE(parseDateTime("2026-10-06", "10:31:02", t));
  CHECK(t.year == 2026); CHECK(t.month == 10); CHECK(t.day == 6);
  CHECK(t.hour == 10); CHECK(t.minute == 31); CHECK(t.second == 2);
  REQUIRE(parseDateTime("2024-02-29", "23:59", t));
  CHECK(t.second == 0);
  CHECK(t.day == 29);
}

TEST_CASE("RTC-3: parseDateTime rejects malformed input and leaves the output alone") {
  DateTime t{2001, 2, 3, 4, 5, 6};
  CHECK_FALSE(parseDateTime("2026-10-6", "10:31:02", t));    // day needs two digits
  CHECK_FALSE(parseDateTime("2026/10/06", "10:31:02", t));
  CHECK_FALSE(parseDateTime("2026-10-06", "10-31-02", t));
  CHECK_FALSE(parseDateTime("2026-10-06", "10:31:0", t));
  CHECK_FALSE(parseDateTime("2026-10-06", "10:31:02x", t));
  CHECK_FALSE(parseDateTime("2026-02-30", "10:31:02", t));   // not a real date
  CHECK_FALSE(parseDateTime("2026-10-06", "25:00:00", t));
  CHECK_FALSE(parseDateTime("", "", t));
  CHECK_FALSE(parseDateTime("abcd-ef-gh", "10:31:02", t));
  CHECK_FALSE(parseDateTime(nullptr, "10:31:02", t));
  CHECK(t.year == 2001);  // untouched
}

TEST_CASE("RTC-1: secondsSince2000 counts correctly across days, months, leap years") {
  CHECK(secondsSince2000({2000, 1, 1, 0, 0, 0}) == 0);
  CHECK(secondsSince2000({2000, 1, 1, 0, 0, 59}) == 59);
  CHECK(secondsSince2000({2000, 1, 2, 0, 0, 0}) == 86400);
  CHECK(secondsSince2000({2000, 3, 1, 0, 0, 0}) == 60u * 86400u);   // 2000 is a leap year: Jan 31 + Feb 29
  CHECK(secondsSince2000({2001, 1, 1, 0, 0, 0}) == 366u * 86400u);
  CHECK(secondsSince2000({2026, 10, 6, 10, 31, 2}) - secondsSince2000({2026, 10, 6, 10, 30, 2}) == 60);
  CHECK(secondsSince2000({2024, 3, 1, 0, 0, 0}) - secondsSince2000({2024, 2, 28, 0, 0, 0}) == 2u * 86400u);  // leap day in between
}
