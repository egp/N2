// DateTime.h — calendar date and time helpers (used by the RTC driver and the `time` console command).
// Pure functions, no hardware: tested on the host. Valid years are 2000..2199 (what the DS3231 can hold).
#pragma once

#include <stdint.h>

namespace n2 {

struct DateTime {
  uint16_t year;    // 2000..2199
  uint8_t month;    // 1..12
  uint8_t day;      // 1..31
  uint8_t hour;     // 0..23
  uint8_t minute;   // 0..59
  uint8_t second;   // 0..59
};

bool isLeapYear(uint16_t year);
uint8_t daysInMonth(uint16_t year, uint8_t month);   // 0 for an invalid month
bool validDateTime(const DateTime& t);

// ISO weekday: Monday = 1 ... Sunday = 7 (what is written to the DS3231's day-of-week register).
uint8_t isoWeekday(uint16_t year, uint8_t month, uint8_t day);

// "2026-10-06 10:31:02" (19 characters + NUL; `out` needs at least 20 bytes).
void formatDateTime(char* out, const DateTime& t);

// Parse "YYYY-MM-DD" and "HH:MM:SS" (or "HH:MM"). False if malformed or not a real date/time.
bool parseDateTime(const char* date, const char* time, DateTime& out);

// Seconds since 2000-01-01 00:00:00 (for measuring elapsed time and clock drift).
uint32_t secondsSince2000(const DateTime& t);

// The inverse of secondsSince2000. 32-bit seconds reach February 2136, so it only returns false if a year beyond 2199 were reached.
bool dateTimeFromSeconds(uint32_t seconds, DateTime& out);

// a - b in seconds (negative if a is earlier).
int32_t secondsBetween(const DateTime& a, const DateTime& b);

}  // namespace n2
