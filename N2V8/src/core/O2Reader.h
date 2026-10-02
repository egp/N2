// O2Reader.h — the O2 sensor as the controller sees it (ARC-5).
// The real implementation wraps the DFRobot library, unmodified; tests use a fake.
#pragma once

#include <stdint.h>

namespace n2 {

class O2Reader {
 public:
  virtual ~O2Reader() = default;
  virtual bool begin() = 0;    // establish communication
  virtual bool present() = 0;  // does the sensor still answer?
  // O2 concentration in percent x100 (20.90 % -> 2090). False if the read failed (O2-3).
  virtual bool readO2PercentX100(uint16_t& value) = 0;
};

}  // namespace n2
