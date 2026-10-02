// O2SensorDfrobot.h — the O2 sensor (DFRobot SEN0465) behind the O2Reader interface (Requirements ARC-5, O2-3).
//
// The DFRobot library is used UNMODIFIED. It has no error signal: readGasConcentrationPPM() returns exactly
// 0.0 when the reply's checksum fails, so a garbled reply looks like a genuine zero. Therefore:
//   * a NON-zero reading must have come from a valid reply and is accepted;
//   * a zero is accepted only after it repeats and the library's checksum-validated queryGasType() answers "O2";
//   * the library blocks ~10 ms inside each call (delay(10)); a zero reading costs up to ~50 ms.
// Device only (needs Arduino and the library); the host uses a fake O2Reader.
#pragma once

#include <stdint.h>

#include "../hal/Hal.h"
#include "../core/O2Reader.h"

namespace n2 {

#if defined(ARDUINO)

class O2SensorDfrobot : public O2Reader {
 public:
  O2SensorDfrobot(Hal& hal, uint8_t address);
  ~O2SensorDfrobot() override;
  bool begin() override;
  bool present() override;
  bool readO2PercentX100(uint16_t& value) override;

 private:
  Hal& hal_;
  uint8_t address_;
  void* impl_;  // the library object, kept out of this header so users need not include the library
};

#endif  // ARDUINO

}  // namespace n2
