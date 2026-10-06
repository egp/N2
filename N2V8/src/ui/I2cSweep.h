// I2cSweep.h — does the I2C bus work at every speed? (Requirements PIN-11, DRV-1)
//
// A diagnostic, run from the console (`i2c sweep`). For each clock speed it runs three checks on the devices that are fitted and
// counts failures: (1) the LCD backpack read-back test (lcd.busTest), (2) repeated reads of the RTC, which must succeed and stay
// consistent (each read within 0..2 s of the previous one), (3) address probes of the LCD and the RTC. Afterwards the clock is put back.
// It BLOCKS for up to about half a second per speed (slow speeds are slow), refreshing the watchdog between speeds: a diagnostic, not
// part of the normal loop (NFR-1a).
#pragma once

#include <stdint.h>

#include "../drivers/Lcd20x4.h"
#include "../drivers/Rtc3231.h"
#include "../hal/Hal.h"

namespace n2 {

struct SweepRow {
  uint32_t hz = 0;
  uint16_t lcdRounds = 0, lcdBad = 0;   // write failures + read failures + mismatches
  uint16_t rtcReads = 0, rtcBad = 0;    // failed reads + inconsistent times
  uint16_t probes = 0, probeBad = 0;
  uint32_t microsPerRtcRead = 0;
  bool ok() const { return lcdBad == 0 && rtcBad == 0 && probeBad == 0; }
};

constexpr uint8_t kSweepMaxSpeeds = 8;
// The only two rates the UNO R4 Wire really has: 100 kHz and 400 kHz. Any other value passed to Wire.setClock() is silently ignored
// (the clock stays at its previous setting), and 1 MHz (Fast-mode Plus) behaves exactly like 400 kHz on the bench (same 262 us RTC read).
// So sweeping 10/30/50/200 kHz or 1 MHz would only measure 100 or 400 kHz again (verified 2026-10-06).
extern const uint32_t kSweepSpeeds[2];

// Runs the sweep over `speeds` (count <= kSweepMaxSpeeds), writing one row each; returns the number of rows. `restoreHz` is set at the end.
uint8_t runI2cSweep(Hal& hal, Lcd20x4& lcd, Rtc3231& rtc, uint8_t lcdAddress, uint8_t rtcAddress, const uint32_t* speeds, uint8_t count,
                    uint16_t rounds, uint32_t restoreHz, SweepRow* rows);

}  // namespace n2
