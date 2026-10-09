// ScanReport.cpp — see ScanReport.h.
#include "ScanReport.h"

#include <stdio.h>
#include <string.h>

namespace n2 {

namespace {
bool has(const uint8_t found[16], uint8_t a) { return a < 128 && (found[a / 8] & (1u << (a % 8))) != 0; }

// Append "0xNN," for each address in [first, first+n) that answered; returns how many.
uint8_t listFound(const uint8_t found[16], uint8_t first, uint8_t n, char* out, size_t cap, uint8_t skip = 0xFF) {
  uint8_t k = 0;
  size_t len = strlen(out);
  for (uint8_t a = first; a < first + n; ++a) {
    if (!has(found, a) || a == skip) continue;   // `skip`: an address the LCD owns is not an LED response
    len += static_cast<size_t>(snprintf(out + len, cap - len, "%s0x%02X", len == 0 ? "" : ",", static_cast<unsigned>(a)));
    ++k;
  }
  return k;
}
}  // namespace

void buildScanReport(const BoardDef& b, const uint8_t found[16], ScanReport& r) {
  r = ScanReport();
  for (uint8_t a = 0x08; a < 0x78; ++a) if (has(found, a)) ++r.responses;
  auto add = [&](const char* fmt, auto... args) {
    if (r.count < ScanReport::kMaxLines) snprintf(r.line[r.count++], sizeof r.line[0], fmt, args...);
  };
  auto addText = [&](const char* text) { add("%s", text); };
  uint8_t accounted = 0;

  // LED (TM1650): the control address and its aliases, and four digit addresses. On a separate software bus it cannot answer here.
  if (ledOnSoftBus(b)) {
    addText("LED: on its own bus (not on this one)");
  } else {
    char list[96] = "";
    const uint8_t nc = listFound(found, b.addrLed, 4, list, sizeof list, b.addrLcd);
    const uint8_t nd = listFound(found, b.addrLedDigits, 4, list, sizeof list);
    accounted += static_cast<uint8_t>(nc + nd);
    if (nc > 0 && nd == 4) add("LED found at %s  (control + %u alias, 4 digits)", list, static_cast<unsigned>(nc - 1));
    else if (nc + nd == 0) { add("LED NOT found (expected 0x%02X and 0x%02X-0x%02X)", static_cast<unsigned>(b.addrLed), static_cast<unsigned>(b.addrLedDigits), static_cast<unsigned>(b.addrLedDigits + 3)); ++r.missing; }
    else { add("LED PARTLY found at %s (expected control 0x%02X and digits 0x%02X-0x%02X)", list, static_cast<unsigned>(b.addrLed), static_cast<unsigned>(b.addrLedDigits), static_cast<unsigned>(b.addrLedDigits + 3)); ++r.missing; }
  }

  // LCD
  if (has(found, b.addrLcd)) {
    if (inTm1650Range(b, b.addrLcd)) add("LCD found at 0x%02X (an address the LED chip also answers: only fit with the LED module unplugged)", static_cast<unsigned>(b.addrLcd));
    else add("LCD found at 0x%02X", static_cast<unsigned>(b.addrLcd));
    ++accounted;
  } else { add("LCD NOT found (expected 0x%02X)", static_cast<unsigned>(b.addrLcd)); ++r.missing; }

  // RTC and the EEPROM that is on the same module
  if (has(found, b.addrRtc)) {
    ++accounted;
    if (has(found, 0x57)) { add("RTC found at 0x%02X (its EEPROM at 0x57, unused)", static_cast<unsigned>(b.addrRtc)); ++accounted; }
    else add("RTC found at 0x%02X", static_cast<unsigned>(b.addrRtc));
  } else {
    add("RTC NOT found (expected 0x%02X)", static_cast<unsigned>(b.addrRtc));
    ++r.missing;
    if (has(found, 0x57)) { addText("EEPROM at 0x57 answered (RTC module present, but the clock chip does not answer)"); ++accounted; }
  }

  // O2 sensor
  if (has(found, b.addrO2)) { add("O2 sensor found at 0x%02X", static_cast<unsigned>(b.addrO2)); ++accounted; }
  else { add("O2 sensor NOT found (expected 0x%02X)", static_cast<unsigned>(b.addrO2)); ++r.missing; }

  // Anything else that answered
  char other[96] = "";
  size_t len = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    if (!has(found, a)) continue;
    const bool known = (!ledOnSoftBus(b) && ((a >= b.addrLed && a < b.addrLed + 4) || (a >= b.addrLedDigits && a < b.addrLedDigits + 4))) ||
                       a == b.addrLcd || a == b.addrRtc || a == 0x57 || a == b.addrO2;
    if (known) continue;
    ++r.unexpected;
    if (len < sizeof other - 8) len += static_cast<size_t>(snprintf(other + len, sizeof other - len, "%s0x%02X", len == 0 ? "" : ",", static_cast<unsigned>(a)));
  }
  if (r.unexpected == 0) addText("Unexpected responders: none");
  else add("UNEXPECTED responders: %s", other);
  add("%u address(es) answered: %u accounted for, %u unexpected", static_cast<unsigned>(r.responses), static_cast<unsigned>(accounted), static_cast<unsigned>(r.unexpected));
}

}  // namespace n2
