// host_tests/test_BB_system_snapshot.cpp v3
//
// Verifies that InputSnapshot and each controller's Snapshot struct
// hold and round-trip their fields correctly. SystemSnapshot.h has
// been removed; snapshots are now used directly.

#include <cstdio>
#include "InputSnapshot.h"
#include "TowerController.h"
#include "O2Controller.h"
#include "N2Controller.h"

static bool require(bool condition, const char* message) {
  if (!condition) { printf("FAIL: %s\n", message); return false; }
  return true;
}

static bool test_BB_inputSnapshotRetainsAllFields() {
  InputSnapshot input{};
  input.sampledAtMs        = 10U;
  input.blackSwitchEnabled = true;
  input.supplyPsi_x10      = 1234U;
  input.leftTowerPsi_x10   = 111U;
  input.rightTowerPsi_x10  = 222U;
  input.lowN2Psi_x100      = 3333U;
  input.highN2Psi_x10      = 444U;

  if (!require(input.sampledAtMs == 10U,          "sampledAtMs"))        return false;
  if (!require(input.blackSwitchEnabled,           "blackSwitchEnabled")) return false;
  if (!require(input.supplyPsi_x10 == 1234U,      "supplyPsi_x10"))      return false;
  if (!require(input.leftTowerPsi_x10 == 111U,    "leftTowerPsi_x10"))   return false;
  if (!require(input.rightTowerPsi_x10 == 222U,   "rightTowerPsi_x10"))  return false;
  if (!require(input.lowN2Psi_x100 == 3333U,      "lowN2Psi_x100"))      return false;
  if (!require(input.highN2Psi_x10 == 444U,       "highN2Psi_x10"))      return false;
  return true;
}

static bool test_BB_towerSnapshotRetainsAllFields() {
  TowerController::Snapshot snap{};
  snap.createdAtMs = 20U;
  snap.state       = TowerController::STATE_LEFT_ONLY;

  if (!require(snap.createdAtMs == 20U,                           "createdAtMs")) return false;
  if (!require(snap.state == TowerController::STATE_LEFT_ONLY,    "state"))       return false;
  return true;
}

static bool test_BB_o2SnapshotRetainsAllFields() {
  O2Controller::Snapshot snap{};
  snap.createdAtMs  = 30U;
  snap.state        = O2Controller::STATE_WAITING_TO_FLUSH;
  snap.hasValue     = true;
  snap.isValueFresh = true;
  snap.o2Percent    = 20.9f;
  snap.n2Percent    = 79.1f;
  snap.liveN2Percent = 78.5f;
  snap.errorString  = "no error";

  if (!require(snap.createdAtMs == 30U,                                  "createdAtMs"))   return false;
  if (!require(snap.state == O2Controller::STATE_WAITING_TO_FLUSH,       "state"))         return false;
  if (!require(snap.hasValue,                                             "hasValue"))      return false;
  if (!require(snap.isValueFresh,                                         "isValueFresh"))  return false;
  if (!require(snap.o2Percent > 20.0f && snap.o2Percent < 21.0f,         "o2Percent"))     return false;
  if (!require(snap.n2Percent > 79.0f,                                   "n2Percent"))     return false;
  if (!require(snap.liveN2Percent > 78.0f,                               "liveN2Percent")) return false;
  if (!require(snap.errorString != nullptr,                               "errorString"))   return false;
  return true;
}

static bool test_BB_n2SnapshotRetainsAllFields() {
  N2Controller::Snapshot snap{};
  snap.createdAtMs = 40U;
  snap.state       = N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT;

  if (!require(snap.createdAtMs == 40U,                                        "createdAtMs")) return false;
  if (!require(snap.state == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,       "state"))       return false;
  return true;
}

int main() {
  if (!test_BB_inputSnapshotRetainsAllFields()) return 1;
  if (!test_BB_towerSnapshotRetainsAllFields()) return 1;
  if (!test_BB_o2SnapshotRetainsAllFields())    return 1;
  if (!test_BB_n2SnapshotRetainsAllFields())    return 1;
  printf("PASS: test_BB_system_snapshot\n");
  return 0;
}
// host_tests/test_BB_system_snapshot.cpp v3