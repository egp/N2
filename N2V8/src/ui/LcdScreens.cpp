#include "LcdScreens.h"

#include <stdio.h>
#include <string.h>

#include "../core/Scaling.h"

namespace n2 {

namespace {

void clearScreen(Screen& s) {
  for (auto& r : s.row) {
    memset(r, ' ', kLcdCols);
    r[kLcdCols] = '\0';
  }
}

// Write text at (row, col) without a terminating NUL, clipped to the row.
void put(Screen& s, uint8_t row, uint8_t col, const char* text) {
  for (uint8_t i = 0; text[i] != '\0' && col + i < kLcdCols; ++i) s.row[row][col + i] = text[i];
}

// Left-aligned two-character field (state names).
void put2(Screen& s, uint8_t row, uint8_t col, const char* name) {
  const char padded[3] = {name[0] != '\0' ? name[0] : ' ', name[0] != '\0' && name[1] != '\0' ? name[1] : ' ', '\0'};
  put(s, row, col, padded);
}

void bits(Screen& s, uint8_t row, uint8_t col, const DisplayData& d) {
  const char b[5] = {d.left ? '1' : '0', d.right ? '1' : '0', d.flush ? '1' : '0', d.ssr ? '1' : '0', '\0'};
  put(s, row, col, b);
}

// Row 0 (both layouts): N2% (or the warm-up countdown), and the O2 state.
void purityRow(Screen& s, const DisplayData& d) {
  char v[8];
  if (d.warming) {
    put(s, 0, 0, "WRM ");
    formatCountdown(v, d.warmRemainingMs);
  } else {
    put(s, 0, 0, "N2% ");
    if (d.n2Valid) formatX100(v, d.n2PercentX100);
    else snprintf(v, sizeof v, "--.--");
  }
  put(s, 0, 4, v);
  if (!d.warming && d.n2Valid && d.n2Stale) put(s, 0, 9, "*");  // value kept but not refreshed (O2-7)
  put(s, 0, 11, "O2");
  put2(s, 0, 14, d.o2);
}

}  // namespace

bool Screen::operator==(const Screen& o) const {
  for (uint8_t r = 0; r < kLcdRows; ++r)
    if (memcmp(row[r], o.row[r], kLcdCols) != 0) return false;
  return true;
}

void formatX10(char* out, uint16_t value) { snprintf(out, 8, "%3u.%u", static_cast<unsigned>(value / 10), static_cast<unsigned>(value % 10)); }
void formatX100(char* out, uint16_t value) { snprintf(out, 8, "%2u.%02u", static_cast<unsigned>(value / 100), static_cast<unsigned>(value % 100)); }
void formatCountdown(char* out, uint32_t ms) {
  uint32_t secs = (ms + 999u) / 1000u;  // round up: never shows 0:00 until warm
  if (secs > 99u * 60u + 59u) secs = 99u * 60u + 59u;  // the field holds "99:59" at most
  snprintf(out, 8, "%2u:%02u", static_cast<unsigned>(secs / 60u), static_cast<unsigned>(secs % 60u));
}

Screen renderNormal(const DisplayData& d, LcdLayout layout) {
  Screen s;
  clearScreen(s);
  char v[8];
  purityRow(s, d);

  if (layout == LcdLayout::kClearLabels) {
    put(s, 1, 0, "N2L ");
    formatX100(v, d.n2LowX100);
    put(s, 1, 4, v);
    put(s, 1, 10, "N2H ");
    formatX10(v, d.n2HighX10);
    put(s, 1, 14, v);
    // Row 2, columns 0-5 (owner decision 2026-10-06): the compressor needs no text while it simply runs or is off. The field says
    // WHY it is stopped (CMP LO / CMP HI), otherwise it shows the code of the last fault raised (LF 12, or LF -- for none).
    if (d.compressor[0] == 'L' || d.compressor[0] == 'H') {
      put(s, 2, 0, "CMP ");
      put2(s, 2, 4, d.compressor);
    } else if (d.lastFaultCode != 0) {
      char lf[8];
      snprintf(lf, sizeof lf, "LF %02X", static_cast<unsigned>(d.lastFaultCode));
      put(s, 2, 0, lf);
    } else {
      put(s, 2, 0, "LF --");
    }
    put(s, 2, 8, "TWR ");
    put2(s, 2, 12, d.tower);
  } else {
    put(s, 1, 0, "L");
    formatX100(v, d.n2LowX100);
    put(s, 1, 1, v);
    put(s, 1, 7, "H");
    formatX10(v, d.n2HighX10);
    put(s, 1, 8, v);
    put(s, 1, 14, "CMP:");
    put2(s, 1, 18, d.compressor);
    put(s, 2, 0, "TWR ");
    put2(s, 2, 4, d.tower);
  }
  put(s, 2, 16, "LRFS");
  put(s, 3, 0, "AIR ");
  formatX10(v, d.airX10);
  put(s, 3, 4, v);
  bits(s, 3, 16, d);
  if (d.version[0] != '\0') put(s, 3, 10, d.version);   // row 4, columns 10-15: between the AIR value and the LRFS bits
  return s;
}

namespace {
const char* severityText(Severity s) {
  return s == Severity::kInhibit ? "INHIBIT" : (s == Severity::kWarn ? "WARNING" : "INFO");
}

// Row 2 of the fault screen: the measurement behind the fault, when there is one.
void faultDetail(char* out, size_t n, FaultId id, const DisplayData& d) {
  auto sensor = [&](uint16_t raw) {
    const unsigned mv = millivoltsFromRaw(raw, d.adcBits);
    snprintf(out, n, "RAW %u  %u.%02uV", static_cast<unsigned>(raw), mv / 1000u, (mv % 1000u) / 10u);
  };
  switch (id) {
    case FaultId::kAirSensor:    sensor(d.rawAir); break;
    case FaultId::kN2LowSensor:  sensor(d.rawN2Low); break;
    case FaultId::kN2HighSensor: sensor(d.rawN2High); break;
    case FaultId::kSensorOrder: {
      char lo[8], hi[8];
      formatX100(lo, d.n2LowX100);
      formatX10(hi, d.n2HighX10);
      snprintf(out, n, "L%s>H%s", lo, hi);
      break;
    }
    case FaultId::kO2Comm: snprintf(out, n, "O2 STATE %s", d.o2); break;
    default: out[0] = '\0'; break;
  }
}
}  // namespace

Screen renderFault(const DisplayData& d, uint8_t faultIndex) {
  Screen s;
  clearScreen(s);
  if (d.faultCount == 0) {
    put(s, 0, 0, "NO ACTIVE FAULT");
    return s;
  }
  const uint8_t i = static_cast<uint8_t>(faultIndex % d.faultCount);
  const FaultInfo& info = faultInfo(d.faults[i]);
  char line[24];
  snprintf(line, sizeof line, "FAULT %u OF %u %s", static_cast<unsigned>(i + 1), static_cast<unsigned>(d.faultCount),
           severityText(info.severity));
  put(s, 0, 0, line);
  snprintf(line, sizeof line, "F%02X %s", static_cast<unsigned>(info.code), info.text);
  put(s, 1, 0, line);
  faultDetail(line, sizeof line, d.faults[i], d);
  put(s, 2, 0, line);
  put(s, 3, 0, info.effect);
  return s;
}

Screen makeScreen(const char* r0, const char* r1, const char* r2, const char* r3) {
  Screen s;
  clearScreen(s);
  const char* rows[4] = {r0, r1, r2, r3};
  for (uint8_t r = 0; r < kLcdRows; ++r) put(s, r, 0, rows[r]);
  return s;
}

Screen renderBanner(const char* version, const char* board, const char* buildDate) {
  Screen s;
  clearScreen(s);
  char line[24];
  snprintf(line, sizeof line, "N2V8 %s", version);
  put(s, 0, 0, line);
  put(s, 1, 0, board);
  put(s, 2, 0, buildDate);
  put(s, 3, 0, "POST ...");
  return s;
}

}  // namespace n2
