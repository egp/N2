// SystemSnapshot.h v2
#ifndef SYSTEM_SNAPSHOT_H
#define SYSTEM_SNAPSHOT_H
#include "InputSnapshot.h"
#include "O2Controller.h"
#include "TowerController.h"
#include "N2Controller.h"

struct SystemSnapshot {
  InputSnapshot             input;
  TowerController::Snapshot tower;
  O2Controller::Snapshot    o2;
  N2Controller::Snapshot    n2;
};
#endif
// SystemSnapshot.h v2
