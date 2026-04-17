// SystemProfile_minima.cpp v3
#include "SystemProfile_minima.h"

#if defined(ARDUINO_MINIMA)

#include <Arduino.h>

uint32_t ProfileClock::nowMs() const {

  return millis();

}

void ProfileClock::setNowMs(uint32_t nowMs) {

  unusedNowMs_ = nowMs;

}

void ProfileClock::advanceMs(uint32_t deltaMs) {

  unusedNowMs_ += deltaMs;

}

ProfileO2Sensor::ProfileO2Sensor(BBI2C& i2c, uint8_t address)

 : sensor_(),

 i2c_(&i2c),

 address_(address),

 lastError_("not initialized") {

}

bool ProfileO2Sensor::begin() {

  if (i2c_ == nullptr) {

    lastError_ = "I2C bus not configured";

    return false;

  }
  if (!sensor_.begin(*i2c_, address_)) {

    lastError_ = sensor_.errorString();

    return false;

  }

  lastError_ = "no error";

  return true;

}

bool ProfileO2Sensor::readOxygenPercent(float& percentVol) {

  if (!sensor_.readOxygenPercent(percentVol)) {

    lastError_ = sensor_.errorString();

    return false;

  }

  lastError_ = "no error";

  return true;

}

const char* ProfileO2Sensor::errorString() const {

  return lastError_;

}

SystemConfig makeSystemConfig() {

  SystemConfig config{};
  config.pressure.adcBits = 10U;

  config.pressure.analogScaleMax =

  static_cast<int>((1UL << config.pressure.adcBits) - 1UL);

  config.pressure.minPressureReading = config.pressure.analogScaleMax / 10;

  config.pressure.maxPressureReading =

  config.pressure.analogScaleMax - config.pressure.minPressureReading;

  config.pressure.supplyFullScalePsi_x10 = 1500U;

  config.pressure.towerFullScalePsi_x10 = 1500U;

  config.pressure.lowN2FullScalePsi_x100 = 3000U;
  config.pressure.highN2FullScalePsi_x10 = 1500U;

  config.n2.lowOffPsi_x100 = 1000U;

  config.n2.lowOnPsi_x100 = 2000U;

  config.n2.highOnPsi_x10 = 1000U;

  config.n2.highOffPsi_x10 = 1200U;

  config.o2.warmupDurationMs = 300000UL;

  config.o2.measurementIntervalMs = 60000UL;

  config.o2.flushDurationMs = 3000UL;

  config.o2.settleDurationMs = 2000UL;

  config.o2.sampleIntervalMs = 250U;

  config.o2.sampleCount = 10U;

  config.o2.freshnessThresholdMs = 15000UL;

  config.o2.errorBackoffMs = 1000UL;

  config.tower.leftOpenMs = 60000UL;

  config.tower.overlapMs = 750UL;

  config.tower.rightOpenMs = 60000UL;
  config.tower.lowSupplyPsi_x10 = 90U;

  return config;

}

void systemProfileSetup(ProfileClock& clock, ProfileO2Sensor& o2Sensor) {

  (void)clock;
  (void)o2Sensor;

}

void systemProfileRefreshInputs(SystemContext& ctx, ProfileClock& clock) {

  ctx.input.sampledAtMs = clock.nowMs();
  ctx.input.supplyPsi_x10 = ctx.runtime.sensors.scaled.supplyPsi_x10;
  ctx.input.leftTowerPsi_x10 = ctx.runtime.sensors.scaled.leftTowerPsi_x10;
  ctx.input.rightTowerPsi_x10 = ctx.runtime.sensors.scaled.rightTowerPsi_x10;
  ctx.input.lowN2Psi_x100 = ctx.runtime.sensors.scaled.lowN2Psi_x100;
  ctx.input.highN2Psi_x10 = ctx.runtime.sensors.scaled.highN2Psi_x10;

}

#endif // defined(ARDUINO_MINIMA)

// SystemProfile_minima.cpp v3