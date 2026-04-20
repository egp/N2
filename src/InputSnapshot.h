// InputSnapshot.h v2
#ifndef INPUT_SNAPSHOT_H
#define INPUT_SNAPSHOT_H

#include <stdint.h>

struct InputSnapshot {
  uint32_t sampledAtMs;

  bool blackSwitchEnabled;
  uint8_t rotarySwitchStatus;
  
  uint16_t supplyPsi_x10;
  uint16_t leftTowerPsi_x10;
  uint16_t rightTowerPsi_x10;
  uint16_t lowN2Psi_x100;
  uint16_t highN2Psi_x10;
};

#endif

// InputSnapshot.h v2