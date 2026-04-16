// host_tests/test_BB_tower_controller.cpp v4
#include <stdio.h>

#include "BinaryOutput.h"
#include "TowerController.h"

class FakeClock : public IClock {
public:
  FakeClock() : nowMs_(0U) {}

  uint32_t nowMs() const override { return nowMs_; }

  void advanceMs(uint32_t deltaMs) { nowMs_ += deltaMs; }

private:
  uint32_t nowMs_;
};

class FakeBinaryOutput : public IBinaryOutput {
public:
  FakeBinaryOutput() : on_(false) {}

  void setOn(bool on) override { on_ = on; }

  bool isOn() const { return on_; }

private:
  bool on_;
};

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static TowerController::Config testConfig() {
  TowerController::Config config;
  config.leftOpenMs = 60000U;
  config.overlapMs = 750U;
  config.rightOpenMs = 60000U;
  config.lowSupplyPsi_x10 = 1000U;
  return config;
}

static TowerController::Config fastConfig() {
  TowerController::Config config;
  config.leftOpenMs = 10U;
  config.overlapMs = 3U;
  config.rightOpenMs = 20U;
  config.lowSupplyPsi_x10 = 1000U;
  return config;
}

static InputSnapshot makeInputs(uint16_t supplyPsi_x10) {
  InputSnapshot inputs{};
  inputs.sampledAtMs = 0U;
  inputs.blackSwitchEnabled = true;
  inputs.supplyPsi_x10 = supplyPsi_x10;
  inputs.lowN2Psi_x100 = 0U;
  inputs.highN2Psi_x10 = 0U;
  return inputs;
}

static bool test_BB_startsInactiveWithBothValvesClosed() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  if (!require(controller.state() == TowerController::STATE_INACTIVE,
               "controller should start inactive")) return false;
  if (!require(!controller.isEnabled(),
               "controller should start disabled")) return false;
  if (!require(!controller.isActive(),
               "inactive controller should not be active")) return false;
  if (!require(!leftValve.isOn(),
               "left valve should start closed")) return false;
  if (!require(!rightValve.isOn(),
               "right valve should start closed")) return false;

  return true;
}

static bool test_BB_enableImmediatelyStartsLeftOnly() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);

  if (!require(controller.isEnabled(),
               "controller should report enabled after setEnabled(true)")) return false;
  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "enable should immediately enter left-only")) return false;
  if (!require(controller.isActive(),
               "left-only should be active")) return false;
  if (!require(leftValve.isOn(),
               "left valve should be on in left-only")) return false;
  if (!require(!rightValve.isOn(),
               "right valve should be off in left-only")) return false;

  return true;
}

static bool test_BB_tickBeforeExpiryDoesNotTransition() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);
  clock.advanceMs(config.leftOpenMs - 1U);
  controller.tick(makeInputs(1500U));

  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "tick before expiry should not leave left-only")) return false;
  if (!require(leftValve.isOn() && !rightValve.isOn(),
               "outputs should remain left-only before expiry")) return false;

  return true;
}

static bool test_BB_fullCycleAdvancesWithMockClockWhenSupplyIsSufficient() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);

  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "enable should enter left-only state")) return false;
  if (!require(leftValve.isOn(),
               "left valve should be open in left-only state")) return false;
  if (!require(!rightValve.isOn(),
               "right valve should be closed in left-only state")) return false;

  clock.advanceMs(config.leftOpenMs);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT,
               "left-only should transition to first overlap")) return false;
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both valves should be open in first overlap")) return false;

  clock.advanceMs(config.overlapMs);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_RIGHT_ONLY,
               "first overlap should transition to right-only")) return false;
  if (!require(!leftValve.isOn() && rightValve.isOn(),
               "only right valve should be open in right-only")) return false;

  clock.advanceMs(config.rightOpenMs);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_RIGHT,
               "right-only should transition to second overlap")) return false;
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both valves should be open in second overlap")) return false;

  clock.advanceMs(config.overlapMs);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "second overlap should transition back to left-only")) return false;
  if (!require(leftValve.isOn() && !rightValve.isOn(),
               "cycle should return to left-only outputs")) return false;

  return true;
}

static bool test_BB_disableForcesImmediateInactiveFromActiveState() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);
  clock.advanceMs(config.leftOpenMs);
  controller.tick(makeInputs(1500U));

  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT,
               "precondition: controller should reach overlap state")) return false;

  controller.setEnabled(false);

  if (!require(!controller.isEnabled(),
               "disable should clear enabled flag")) return false;
  if (!require(controller.state() == TowerController::STATE_INACTIVE,
               "disable should force inactive")) return false;
  if (!require(!controller.isActive(),
               "inactive should not be active")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "disable should close both valves")) return false;

  return true;
}

static bool test_BB_perStateOutputsAndIsActive() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);
  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "enable should enter left-only")) return false;
  if (!require(controller.isActive(),
               "left-only should be active")) return false;
  if (!require(leftValve.isOn() && !rightValve.isOn(),
               "left-only outputs should be left on, right off")) return false;

  clock.advanceMs(config.leftOpenMs);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT,
               "should enter both-after-left")) return false;
  if (!require(controller.isActive(),
               "both-after-left should be active")) return false;
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both-after-left outputs should be both on")) return false;

  clock.advanceMs(config.overlapMs);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_RIGHT_ONLY,
               "should enter right-only")) return false;
  if (!require(controller.isActive(),
               "right-only should be active")) return false;
  if (!require(!leftValve.isOn() && rightValve.isOn(),
               "right-only outputs should be left off, right on")) return false;

  clock.advanceMs(config.rightOpenMs);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_RIGHT,
               "should enter both-after-right")) return false;
  if (!require(controller.isActive(),
               "both-after-right should be active")) return false;
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both-after-right outputs should be both on")) return false;

  controller.tick(makeInputs(900U));
  if (!require(controller.state() == TowerController::STATE_LOW_SUPPLY,
               "low supply should force low-supply state")) return false;
  if (!require(!controller.isActive(),
               "low-supply should not be active")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "low-supply outputs should close both valves")) return false;

  controller.setEnabled(false);
  if (!require(controller.state() == TowerController::STATE_INACTIVE,
               "disable should force inactive from low-supply")) return false;
  if (!require(!controller.isActive(),
               "inactive should not be active")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "inactive outputs should close both valves")) return false;

  return true;
}

static bool test_BB_configOverrideChangesTiming() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;

  TowerController::Config config;
  config.leftOpenMs = 5U;
  config.overlapMs = 2U;
  config.rightOpenMs = 7U;
  config.lowSupplyPsi_x10 = 1000U;

  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);

  clock.advanceMs(4U);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "custom left duration should not expire early")) return false;

  clock.advanceMs(1U);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT,
               "custom left duration should expire at configured boundary")) return false;

  clock.advanceMs(1U);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT,
               "custom overlap should hold before configured boundary")) return false;

  clock.advanceMs(1U);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_RIGHT_ONLY,
               "custom overlap should expire at configured boundary")) return false;

  clock.advanceMs(6U);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_RIGHT_ONLY,
               "custom right duration should hold before configured boundary")) return false;

  clock.advanceMs(1U);
  controller.tick(makeInputs(1500U));
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_RIGHT,
               "custom right duration should expire at configured boundary")) return false;

  return true;
}

static bool test_BB_lowSupplyForcesDedicatedLowSupplyState() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.tick(makeInputs(900U));

  if (!require(controller.state() == TowerController::STATE_LOW_SUPPLY,
               "low supply should force low-supply state")) return false;
  if (!require(!controller.isActive(),
               "low-supply state should not be active")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "low supply should close both valves")) return false;

  return true;
}

static bool test_BB_recoveryFromLowSupplyRestartsAtLeftOnly() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.tick(makeInputs(900U));

  if (!require(controller.state() == TowerController::STATE_LOW_SUPPLY,
               "precondition: controller should enter low-supply")) return false;

  controller.tick(makeInputs(1100U));

  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "recovery should restart at left-only")) return false;
  if (!require(controller.isActive(),
               "recovery left-only should be active")) return false;
  if (!require(leftValve.isOn() && !rightValve.isOn(),
               "recovery should restore left-only outputs")) return false;

  return true;
}

static bool test_BB_disableFromLowSupplyForcesInactive() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.tick(makeInputs(900U));

  if (!require(controller.state() == TowerController::STATE_LOW_SUPPLY,
               "precondition: controller should be in low-supply")) return false;

  controller.setEnabled(false);

  if (!require(controller.state() == TowerController::STATE_INACTIVE,
               "disable from low-supply should force inactive")) return false;
  if (!require(!controller.isEnabled(),
               "disable from low-supply should clear enabled flag")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "disable from low-supply should close both valves")) return false;

  return true;
}

static bool test_BB_lowSupplyWhileInactiveDoesNotActivate() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.tick(makeInputs(0U));

  if (!require(controller.state() == TowerController::STATE_INACTIVE,
               "tick while disabled should stay inactive")) return false;
  if (!require(!controller.isEnabled(),
               "tick while disabled should stay disabled")) return false;
  if (!require(!controller.isActive(),
               "inactive controller should not be active")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "inactive tick should keep both valves closed")) return false;

  return true;
}

static bool test_BB_exactLowSupplyThresholdIsSufficient() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.tick(makeInputs(1000U));

  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "exact low-supply threshold should be treated as sufficient")) return false;
  if (!require(leftValve.isOn() && !rightValve.isOn(),
               "exact low-supply threshold should keep left-only outputs")) return false;

  return true;
}

static bool test_BB_lowSupplyWinsOverTimedTransitionAtExpiryBoundary() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);

  clock.advanceMs(config.leftOpenMs);
  controller.tick(makeInputs(900U));

  if (!require(controller.state() == TowerController::STATE_LOW_SUPPLY,
               "low supply should win over left-only expiry transition at the boundary")) return false;
  if (!require(!controller.isActive(),
               "low-supply boundary result should not be active")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "low-supply boundary result should close both valves")) return false;

  return true;
}

static bool test_BB_snapshotReflectsCurrentTowerState() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);
  TowerController::Snapshot snapshot = controller.snapshot();

  if (!require(snapshot.createdAtMs == 0U,
               "tower snapshot should start at initial transition time")) return false;
  if (!require(snapshot.state == TowerController::STATE_LEFT_ONLY,
               "tower snapshot should report left-only after enable")) return false;

  clock.advanceMs(config.leftOpenMs);
  controller.tick(makeInputs(1500U));
  snapshot = controller.snapshot();

  if (!require(snapshot.createdAtMs == config.leftOpenMs,
               "tower snapshot timestamp should refresh on state transition")) return false;
  if (!require(snapshot.state == TowerController::STATE_BOTH_AFTER_LEFT,
               "tower snapshot should report updated state after transition")) return false;

  return true;
}

int main() {
  if (!test_BB_startsInactiveWithBothValvesClosed()) return 1;
  if (!test_BB_enableImmediatelyStartsLeftOnly()) return 1;
  if (!test_BB_tickBeforeExpiryDoesNotTransition()) return 1;
  if (!test_BB_fullCycleAdvancesWithMockClockWhenSupplyIsSufficient()) return 1;
  if (!test_BB_disableForcesImmediateInactiveFromActiveState()) return 1;
  if (!test_BB_perStateOutputsAndIsActive()) return 1;
  if (!test_BB_configOverrideChangesTiming()) return 1;
  if (!test_BB_lowSupplyForcesDedicatedLowSupplyState()) return 1;
  if (!test_BB_recoveryFromLowSupplyRestartsAtLeftOnly()) return 1;
  if (!test_BB_disableFromLowSupplyForcesInactive()) return 1;
  if (!test_BB_lowSupplyWhileInactiveDoesNotActivate()) return 1;
  if (!test_BB_exactLowSupplyThresholdIsSufficient()) return 1;
  if (!test_BB_lowSupplyWinsOverTimedTransitionAtExpiryBoundary()) return 1;
  if (!test_BB_snapshotReflectsCurrentTowerState()) return 1;

  printf("PASS: test_BB_tower_controller\n");
  return 0;
}
// host_tests/test_BB_tower_controller.cpp v4