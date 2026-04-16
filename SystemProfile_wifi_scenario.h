// SystemProfile_wifi_scenario.h v4
#ifndef SYSTEM_PROFILE_WIFI_SCENARIO_H
#define SYSTEM_PROFILE_WIFI_SCENARIO_H

#if defined(ARDUINO_UNOWIFIR4)

#include <stdint.h>
#include <TCP1819.h>

#include "O2Controller.h"
#include "SystemContext.h"

class ProfileClock : public IClock {
public:
  ProfileClock();

  uint32_t nowMs() const override;
  void setNowMs(uint32_t nowMs);
  void advanceMs(uint32_t deltaMs);

private:
  uint32_t nowMs_;
};

class ProfileO2Sensor : public IO2Sensor {
public:
  ProfileO2Sensor(BBI2C& i2c, uint8_t address);

  bool begin() override;
  bool readOxygenPercent(float& percentVol) override;
  const char* errorString() const override;

private:
  const char* lastError_;
};

SystemConfig makeSystemConfig();

void systemProfileSetup(ProfileClock& clock, ProfileO2Sensor& o2Sensor);

void systemProfileRefreshInputs(
    SystemContext& ctx,
    ProfileClock& clock,
    bool blackSwitchEnabled,
    uint16_t supplyPsi_x10,
    uint16_t leftTowerPsi_x10,
    uint16_t rightTowerPsi_x10,
    uint16_t lowN2Psi_x100,
    uint16_t highN2Psi_x10);

void systemProfilePrintHelp();
void systemProfilePrintStatus(const SystemContext& ctx);
bool systemProfileConsumeSerialCommand(ProfileClock& clock, SystemContext& ctx);
bool systemProfileIsDone();

#endif  // defined(ARDUINO_UNOWIFIR4)

#endif
// SystemProfile_wifi_scenario.h v4