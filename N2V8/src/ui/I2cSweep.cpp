#include "I2cSweep.h"

namespace n2 {

const uint32_t kSweepSpeeds[2] = {100000, 400000};

uint8_t runI2cSweep(Hal& hal, Lcd20x4& lcd, Rtc3231& rtc, uint8_t lcdAddress, uint8_t rtcAddress, const uint32_t* speeds, uint8_t count,
                    uint16_t rounds, uint32_t restoreHz, SweepRow* rows) {
  if (count > kSweepMaxSpeeds) count = kSweepMaxSpeeds;
  const uint16_t rtcReads = rounds < 40 ? rounds : 40;  // an RTC read is slow at the lowest speeds
  for (uint8_t i = 0; i < count; ++i) {
    SweepRow& r = rows[i];
    r = SweepRow();
    r.hz = speeds[i];
    hal.i2cSetClock(speeds[i]);
    hal.watchdogRefresh();

    const Lcd20x4::BusTest t = lcd.busTest(rounds);
    r.lcdRounds = t.rounds;
    r.lcdBad = static_cast<uint16_t>(t.writeFailed + t.readFailed + t.mismatched);
    hal.watchdogRefresh();

    DateTime previous{};
    bool havePrevious = false;
    const uint32_t start = hal.micros();
    for (uint16_t k = 0; k < rtcReads; ++k) {
      DateTime now{};
      ++r.rtcReads;
      if (!rtc.read(now)) {
        ++r.rtcBad;
        continue;
      }
      if (havePrevious) {
        const int32_t step = secondsBetween(now, previous);
        if (step < 0 || step > 2) ++r.rtcBad;  // bits flipped on the way: valid-looking but wrong
      }
      previous = now;
      havePrevious = true;
    }
    r.microsPerRtcRead = r.rtcReads ? (hal.micros() - start) / r.rtcReads : 0;
    hal.watchdogRefresh();

    for (uint8_t k = 0; k < 10; ++k) {
      r.probes = static_cast<uint16_t>(r.probes + 2);
      if (!hal.i2cProbe(lcdAddress)) ++r.probeBad;
      if (!hal.i2cProbe(rtcAddress)) ++r.probeBad;
    }
  }
  hal.i2cSetClock(restoreHz);
  hal.watchdogRefresh();
  return count;
}

}  // namespace n2
