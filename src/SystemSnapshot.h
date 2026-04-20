// SystemSnapshot.h v1
#ifndef SYSTEM_SNAPSHOT_H
#define SYSTEM_SNAPSHOT_H

#include "InputSnapshot.h"
#include "TowerController.h"
#include "O2Controller.h"
#include "N2Controller.h"

using TowerControllerSnapshot = TowerController::Snapshot;
using O2ControllerSnapshot = O2Controller::Snapshot;
using N2ControllerSnapshot = N2Controller::Snapshot;

struct SystemSnapshot {
  InputSnapshot input;
  TowerControllerSnapshot tower;
  O2ControllerSnapshot o2;
  N2ControllerSnapshot n2;
};

#endif

// SystemSnapshot.h v1