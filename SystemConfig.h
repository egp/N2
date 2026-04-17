// SystemConfig.h v2
#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

struct SystemConfig {
  struct GlobalConfig {
    // Intentionally empty for now.
    // Add stable, non-controller configuration here as needed.
  };

  struct PressureConfig {
    uint8_t adcBits;
    int analogScaleMax;
    int minPressureReading;
    int maxPressureReading;
    uint16_t supplyFullScalePsi_x10;
    uint16_t towerFullScalePsi_x10;
    uint16_t lowN2FullScalePsi_x100;
    uint16_t highN2FullScalePsi_x10;
  };

  struct TowerConfig {
    uint32_t leftOpenMs;
    uint32_t overlapMs;
    uint32_t rightOpenMs;
    uint16_t lowSupplyPsi_x10;
  };

  struct O2Config {
    uint32_t warmupDurationMs;
    uint32_t measurementIntervalMs;
    uint32_t flushDurationMs;
    uint32_t settleDurationMs;
    uint16_t sampleIntervalMs;
    uint8_t sampleCount;
    uint32_t freshnessThresholdMs;
    uint32_t errorBackoffMs;
  };

  struct N2Config {
    uint16_t lowOffPsi_x100;
    uint16_t lowOnPsi_x100;
    uint16_t highOnPsi_x10;
    uint16_t highOffPsi_x10;
  };

  GlobalConfig global;
  PressureConfig pressure;
  TowerConfig tower;
  O2Config o2;
  N2Config n2;
};

#endif

// SystemConfig.h v2