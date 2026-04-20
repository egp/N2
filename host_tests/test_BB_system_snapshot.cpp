// host_tests/test_BB_system_snapshot.cpp v2

#include <cstdio>

#include "SystemSnapshot.h"

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static bool test_BB_systemSnapshotAggregatesInputAndControllerSnapshots() {
  SystemSnapshot snapshot{};

  snapshot.input.sampledAtMs = 10U;
  snapshot.input.blackSwitchEnabled = true;
  snapshot.input.rotarySwitchStatus = 0x6CU;
  snapshot.input.supplyPsi_x10 = 1234U;
  snapshot.input.leftTowerPsi_x10 = 111U;
  snapshot.input.rightTowerPsi_x10 = 222U;
  snapshot.input.lowN2Psi_x100 = 3333U;
  snapshot.input.highN2Psi_x10 = 444U;

  snapshot.tower.createdAtMs = 20U;
  snapshot.tower.state = TowerController::STATE_LEFT_ONLY;

  snapshot.o2.createdAtMs = 30U;
  snapshot.o2.state = O2Controller::STATE_WAITING_TO_FLUSH;
  snapshot.o2.hasValue = true;
  snapshot.o2.isValueFresh = true;
  snapshot.o2.o2Percent = 20.9f;
  snapshot.o2.n2Percent = 79.1f;
  snapshot.o2.errorString = "no error";

  snapshot.n2.createdAtMs = 40U;
  snapshot.n2.state = N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT;

  if (!require(snapshot.input.leftTowerPsi_x10 == 111U,
               "system snapshot should retain input sub-snapshot"))
    return false;

  if (!require(snapshot.input.rotarySwitchStatus == 0x6CU,
               "system snapshot should retain rotary switch status"))
    return false;

  if (!require(snapshot.tower.state == TowerController::STATE_LEFT_ONLY,
               "system snapshot should retain tower sub-snapshot"))
    return false;

  if (!require(snapshot.o2.hasValue && snapshot.o2.n2Percent > 79.0f,
               "system snapshot should retain o2 sub-snapshot"))
    return false;

  if (!require(snapshot.n2.state == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "system snapshot should retain n2 sub-snapshot"))
    return false;

  return true;
}

int main() {
  if (!test_BB_systemSnapshotAggregatesInputAndControllerSnapshots())
    return 1;

  printf("PASS: test_BB_system_snapshot\n");
  return 0;
}

// host_tests/test_BB_system_snapshot.cpp v2