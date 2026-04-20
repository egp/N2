// SystemProfile_minima.h v4
#ifndef SYSTEM_PROFILE_MINIMA_H
#define SYSTEM_PROFILE_MINIMA_H

#if defined(ARDUINO_MINIMA)

#include <stdint.h>

#include <TCP0465.h>

#include "O2Controller.h"
#include "SystemContext.h"

class ProfileClock : public IClock {

public:

  uint32_t nowMs() const override;

  void setNowMs(uint32_t nowMs);

  void advanceMs(uint32_t deltaMs);

private:

  uint32_t unusedNowMs_ = 0U;

};

class ProfileO2Sensor : public IO2Sensor {

public:

  ProfileO2Sensor(BBI2C& i2c, uint8_t address);

  bool begin() override;

  bool readOxygenPercent(float& percentVol) override;

  const char* errorString() const override;

private:

  TCP0465 sensor_;

  BBI2C* i2c_;

  uint8_t address_;
  const char* lastError_;

};

SystemConfig makeSystemConfig();

void systemProfileSetup(ProfileClock& clock, ProfileO2Sensor& o2Sensor);

void systemProfileRefreshInputs(SystemContext& ctx, ProfileClock& clock);

#endif // defined(ARDUINO_MINIMA)

#endif

// SystemProfile_minima.h v4