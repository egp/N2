// SystemRuntime.h v2

#ifndef SYSTEM_RUNTIME_H

#define SYSTEM_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

struct SystemRuntime {

  struct PowerRuntime {

    bool systemEnabled;

    bool systemWasEnabled;

  };

  struct SensorRuntime {

    struct RawPressures {

      int supply;

      int leftTower;

      int rightTower;

      int lowN2;

      int highN2;

    } raw;

    struct ScaledPressures {

      uint16_t supplyPsi_x10;

      uint16_t leftTowerPsi_x10;

      uint16_t rightTowerPsi_x10;

      uint16_t lowN2Psi_x100;

      uint16_t highN2Psi_x10;

    } scaled;

  };

  struct DisplayRuntime {

    uint8_t rotarySwitchStatus;

    uint8_t previousButtonValue;

  };

  PowerRuntime power{};

  SensorRuntime sensors{};

  DisplayRuntime display{};

};

#endif

// SystemRuntime.h v2