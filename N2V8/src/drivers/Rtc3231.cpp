#include "Rtc3231.h"

namespace n2 {

bool Rtc3231::present() { return ok(hal_.i2cProbe(address_)); }

bool Rtc3231::read(DateTime& out) {
  uint8_t r[7];
  if (!ok(hal_.i2cReadReg(address_, kRegSeconds, r, sizeof r))) return false;
  // Every field must be valid BCD; otherwise the chip is not giving us a time (corrupt read, wrong device at 0x68...).
  if (!bcdOk(r[0] & 0x7F) || !bcdOk(r[1] & 0x7F) || !bcdOk(r[2] & 0x1F) || !bcdOk(r[4] & 0x3F) || !bcdOk(r[5] & 0x1F) || !bcdOk(r[6])) return ok(false);

  DateTime t;
  t.second = bcdToDec(r[0] & 0x7F);
  t.minute = bcdToDec(r[1] & 0x7F);
  if (r[2] & 0x40) {  // the chip is in 12-hour mode: bit 5 = PM, bits 4..0 = 1..12
    uint8_t h = bcdToDec(r[2] & 0x1F);
    if (h == 12) h = 0;
    t.hour = static_cast<uint8_t>(h + ((r[2] & 0x20) ? 12 : 0));
  } else {
    t.hour = bcdToDec(r[2] & 0x3F);
  }
  t.day = bcdToDec(r[4] & 0x3F);
  t.month = bcdToDec(r[5] & 0x1F);
  t.year = static_cast<uint16_t>(2000 + bcdToDec(r[6]) + ((r[5] & 0x80) ? 100 : 0));  // bit 7 of the month register = century
  if (!validDateTime(t)) return ok(false);
  out = t;
  return true;
}

bool Rtc3231::set(const DateTime& t) {
  if (!validDateTime(t)) return false;  // refuse nonsense; not a bus error
  const uint16_t y = static_cast<uint16_t>(t.year - 2000);
  const uint8_t century = y >= 100 ? 0x80 : 0x00;
  const uint8_t buf[8] = {kRegSeconds,
                          decToBcd(t.second),
                          decToBcd(t.minute),
                          decToBcd(t.hour),  // bit 6 clear = 24-hour mode
                          isoWeekday(t.year, t.month, t.day),
                          decToBcd(t.day),
                          static_cast<uint8_t>(decToBcd(t.month) | century),
                          decToBcd(static_cast<uint8_t>(y % 100))};
  if (!ok(hal_.i2cWrite(address_, buf, sizeof buf))) return false;
  uint8_t status;
  if (!ok(hal_.i2cReadReg(address_, kRegStatus, &status, 1))) return false;
  const uint8_t clear[2] = {kRegStatus, static_cast<uint8_t>(status & ~kStatusOsf)};
  return ok(hal_.i2cWrite(address_, clear, sizeof clear));  // the time is trustworthy again
}

bool Rtc3231::timeValid(bool& valid) {
  uint8_t status;
  if (!ok(hal_.i2cReadReg(address_, kRegStatus, &status, 1))) return false;
  valid = (status & kStatusOsf) == 0;
  return true;
}

bool Rtc3231::temperatureX100(int16_t& out) {
  uint8_t r[2];
  if (!ok(hal_.i2cReadReg(address_, kRegTemp, r, sizeof r))) return false;
  const int16_t quarters = static_cast<int16_t>((static_cast<int16_t>(static_cast<int8_t>(r[0])) << 2) | (r[1] >> 6));  // 0.25 degree units
  out = static_cast<int16_t>(quarters * 25);
  return true;
}

}  // namespace n2
