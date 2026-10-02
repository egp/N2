#include "O2SensorDfrobot.h"

#if defined(ARDUINO)

#include <Arduino.h>
#include <Wire.h>

#if !defined(N2_NO_DFROBOT)
#include <DFRobot_MultiGasSensor.h>  // https://github.com/DFRobot/DFRobot_MultiGasSensor (used as-is)
#endif

namespace n2 {

#if !defined(N2_NO_DFROBOT)

O2SensorDfrobot::O2SensorDfrobot(Hal& hal, uint8_t address)
    : hal_(hal), address_(address), impl_(new DFRobot_GAS_I2C(&Wire, address)) {}  // one object for the life of the sketch

// The library object is created once and lives for the whole run of the sketch, so it is never freed.
O2SensorDfrobot::~O2SensorDfrobot() {}

bool O2SensorDfrobot::present() { return hal_.i2cProbe(address_); }

bool O2SensorDfrobot::begin() {
  DFRobot_GAS_I2C* gas = static_cast<DFRobot_GAS_I2C*>(impl_);
  if (!present() || !gas->begin()) return false;
  gas->changeAcquireMode(gas->PASSIVITY);  // the controller asks for each reading
  return true;
}

bool O2SensorDfrobot::readO2PercentX100(uint16_t& value) {
  if (!present()) return false;
  DFRobot_GAS_I2C* gas = static_cast<DFRobot_GAS_I2C*>(impl_);
  float pct = 0.0f;
  bool nonZero = false;
  for (uint8_t attempt = 0; attempt < 3 && !nonZero; ++attempt) {
    pct = gas->readGasConcentrationPPM();  // note: despite the name, the SEN0465 reports percent O2
    if (pct < 0.0f) return false;
    nonZero = pct > 0.0f;                  // a bad checksum can only produce exactly 0.0
  }
  if (!nonZero && gas->queryGasType() != "O2") return false;  // three zeros AND no valid O2 reply: link failure
  value = static_cast<uint16_t>(pct * 100.0f + 0.5f);
  return true;
}

#else  // N2_NO_DFROBOT: build without the library (CI): the sensor reads as absent

O2SensorDfrobot::O2SensorDfrobot(Hal& hal, uint8_t address) : hal_(hal), address_(address), impl_(nullptr) {}
O2SensorDfrobot::~O2SensorDfrobot() {}
bool O2SensorDfrobot::present() { return false; }
bool O2SensorDfrobot::begin() { return false; }
bool O2SensorDfrobot::readO2PercentX100(uint16_t&) { return false; }

#endif

}  // namespace n2

#endif  // ARDUINO
