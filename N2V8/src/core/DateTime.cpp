#include "DateTime.h"

#include <stdio.h>

namespace n2 {

bool isLeapYear(uint16_t y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }

uint8_t daysInMonth(uint16_t y, uint8_t m) {
  static const uint8_t kDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m < 1 || m > 12) return 0;
  return static_cast<uint8_t>(m == 2 && isLeapYear(y) ? 29 : kDays[m - 1]);
}

bool validDateTime(const DateTime& t) {
  return t.year >= 2000 && t.year <= 2199 && t.month >= 1 && t.month <= 12 && t.day >= 1 && t.day <= daysInMonth(t.year, t.month) &&
         t.hour <= 23 && t.minute <= 59 && t.second <= 59;
}

// Whole days since 2000-01-01 (a Saturday).
static uint32_t daysSince2000(uint16_t y, uint8_t m, uint8_t d) {
  uint32_t days = 0;
  for (uint16_t yy = 2000; yy < y; ++yy) days += isLeapYear(yy) ? 366 : 365;
  for (uint8_t mm = 1; mm < m; ++mm) days += daysInMonth(y, mm);
  return days + (d - 1u);
}

uint8_t isoWeekday(uint16_t y, uint8_t m, uint8_t d) { return static_cast<uint8_t>((daysSince2000(y, m, d) + 5u) % 7u + 1u); }  // 2000-01-01 = Saturday = 6

uint32_t secondsSince2000(const DateTime& t) {
  return daysSince2000(t.year, t.month, t.day) * 86400u + t.hour * 3600u + t.minute * 60u + t.second;
}

void formatDateTime(char* out, const DateTime& t) {
  snprintf(out, 20, "%04u-%02u-%02u %02u:%02u:%02u", static_cast<unsigned>(t.year % 10000u), static_cast<unsigned>(t.month % 100u),
           static_cast<unsigned>(t.day % 100u), static_cast<unsigned>(t.hour % 100u), static_cast<unsigned>(t.minute % 100u),
           static_cast<unsigned>(t.second % 100u));
}

static bool digits(const char* s, uint8_t n, uint16_t& out) {
  uint16_t v = 0;
  for (uint8_t i = 0; i < n; ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    v = static_cast<uint16_t>(v * 10 + (s[i] - '0'));
  }
  out = v;
  return true;
}

bool parseDateTime(const char* date, const char* time, DateTime& out) {
  uint16_t y, mo, d, h, mi, s = 0;
  if (!date || !time) return false;
  for (uint8_t i = 0; i < 10; ++i)  // the string must be at least 10 long: check before indexing
    if (date[i] == '\0') return false;
  if (!digits(date, 4, y) || date[4] != '-' || !digits(date + 5, 2, mo) || date[7] != '-' || !digits(date + 8, 2, d) || date[10] != '\0') return false;
  for (uint8_t i = 0; i < 5; ++i)
    if (time[i] == '\0') return false;
  if (!digits(time, 2, h) || time[2] != ':' || !digits(time + 3, 2, mi)) return false;
  if (time[5] == ':') {
    for (uint8_t i = 6; i < 8; ++i)
      if (time[i] == '\0') return false;
    if (!digits(time + 6, 2, s) || time[8] != '\0') return false;
  } else if (time[5] != '\0') {
    return false;
  }
  DateTime t = {y, static_cast<uint8_t>(mo), static_cast<uint8_t>(d), static_cast<uint8_t>(h), static_cast<uint8_t>(mi), static_cast<uint8_t>(s)};
  if (!validDateTime(t)) return false;
  out = t;
  return true;
}

}  // namespace n2
