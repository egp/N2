// host_tests/test_BB_system_snapshot.cpp v2
#include <cstdio>
#include "SystemSnapshot.h"

static bool require(bool condition, const char* message) {
  if (!condition) { printf("FAIL: %s\n", message); return false; }
  return true;
}

static bool test_BB_systemSnapshotAggregatesInputAndControllerSnapshots() {
  SystemSnapshot snapshot{};

  snapshot.input.sampledAtMs        = 10U;
  snapshot.input.blackSwitchEnabled = true;
  snapshot.input.supplyPsi_x10      = 1234U;
  snapshot.input.leftTowerPsi_x10   = 111U;
  snapshot.input.rightTowerPsi_x10  = 222U;
  snapshot.input.lowN2Psi_x100      = 3333U;
  snapshot.input.highN2Psi_x10      = 444U;

  snapshot.tower.createdAtMs = 20U;
  snapshot.tower.state       = TowerController::STATE_LEFT_ONLY;

  snapshot.o2.createdAtMs  = 30U;
  snapshot.o2.state        = O2Controller::STATE_WAITING_TO_FLUSH;
  snapshot.o2.hasValue     = true;
  snapshot.o2.isValueFresh = true;
  snapshot.o2.o2Percent    = 20.9f;
  snapshot.o2.n2Percent    = 79.1f;
  snapshot.o2.errorString  = "no error";

  snapshot.n2.createdAtMs = 40U;
  snapshot.n2.state       = N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT;

  if (!require(snapshot.input.leftTowerPsi_x10 == 111U,   "input sub-snapshot")) return false;
  if (!require(snapshot.tower.state == TowerController::STATE_LEFT_ONLY, "tower")) return false;
  if (!require(snapshot.o2.hasValue && snapshot.o2.n2Percent > 79.0f, "o2"))      return false;
  if (!require(snapshot.n2.state == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT, "n2")) return false;

  return true;
}

int main() {
  if (!test_BB_systemSnapshotAggregatesInputAndControllerSnapshots()) return 1;
  printf("PASS: test_BB_system_snapshot\n");
  return 0;
}
