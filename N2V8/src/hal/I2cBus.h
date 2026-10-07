// I2cBus.h — "something that can write bytes to an I2C address and probe an address" (Requirements DRV-1).
//
// The hardware bus (Wire, through the Hal) and the software bus (SoftI2c, two ordinary GPIO pins) look the same to a driver, so the LED
// driver can sit on either. Write failures return false; nothing here ever blocks for long.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Hal.h"

namespace n2 {

class I2cBus {
 public:
  virtual ~I2cBus() = default;
  virtual bool write(uint8_t address, const uint8_t* data, size_t n) = 0;  // one transaction; true only if every byte was acknowledged
  virtual bool probe(uint8_t address) = 0;                                 // does the device acknowledge its address?
};

// The hardware bus: forwards to the Hal (which is Wire on the device and a fake on the host).
class HalI2cBus : public I2cBus {
 public:
  explicit HalI2cBus(Hal& hal) : hal_(hal) {}
  bool write(uint8_t address, const uint8_t* data, size_t n) override { return hal_.i2cWrite(address, data, n); }
  bool probe(uint8_t address) override { return hal_.i2cProbe(address); }

 private:
  Hal& hal_;
};

}  // namespace n2
