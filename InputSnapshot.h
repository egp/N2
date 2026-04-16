// InputSnapshot.h v1
#ifndef INPUT_SNAPSHOT_H
#define INPUT_SNAPSHOT_H

#include <stdint.h>

struct InputSnapshot {
  uint32_t sampledAtMs;
  bool blackSwitchEnabled;
  uint16_t supplyPsi_x10;
  uint16_t lowN2Psi_x100;
  uint16_t highN2Psi_x10;
};

#endif

// InputSnapshot.h v1