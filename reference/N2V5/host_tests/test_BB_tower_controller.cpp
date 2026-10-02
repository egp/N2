// host_tests/test_BB_tower_controller.cpp v5
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
  config.towerOpenMs = 60000U;
  config.overlapMs = 750U;
  config.towerOpenMs = 60000U;
  config.airSupplyOnPsi_x10 = 900U;
  config.airSupplyOffPsi_x10 = 700U;
  return config;
}

static TowerController::Config fastConfig() {
  TowerController::Config config;
  config.towerOpenMs = 10U;
  config.overlapMs = 3U;
  config.towerOpenMs = 20U;
  config.airSupplyOnPsi_x10 = 900U;
  config.airSupplyOffPsi_x10 = 700U;
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

// RESTORED (valid behavior, was previously disabled)
static bool test_BB_tickBeforeExpiryDoesNotTransition() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);

  clock.advanceMs(config.towerOpenMs - 1U);
  controller.step(makeInputs(1500U));

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

  clock.advanceMs(config.towerOpenMs);
  controller.step(makeInputs(1500U));

  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT,
               "left-only should transition to first overlap")) return false;

  clock.advanceMs(config.overlapMs);
  controller.step(makeInputs(1500U));

  if (!require(controller.state() == TowerController::STATE_RIGHT_ONLY,
               "first overlap should transition to right-only")) return false;

  clock.advanceMs(config.towerOpenMs);
  controller.step(makeInputs(1500U));

  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_RIGHT,
               "right-only should transition to second overlap")) return false;

  clock.advanceMs(config.overlapMs);
  controller.step(makeInputs(1500U));

  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
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
  clock.advanceMs(config.towerOpenMs);
  controller.step(makeInputs(1500U));

  controller.setEnabled(false);

  if (!require(controller.state() == TowerController::STATE_INACTIVE,
               "disable should force inactive")) return false;
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

  clock.advanceMs(config.towerOpenMs);
  controller.step(makeInputs(1500U));
  controller.step(makeInputs(700U));

  if (!require(controller.state() == TowerController::STATE_LOW_SUPPLY,
               "low supply should force low-supply state")) return false;

  return true;
}

static bool test_BB_lowSupplyForcesDedicatedLowSupplyState() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.step(makeInputs(700U));

  return require(controller.state() == TowerController::STATE_LOW_SUPPLY,
                 "should enter low-supply at or below off threshold");
}

static bool test_BB_recoveryFromLowSupplyRestartsAtLeftOnly() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);

  controller.step(makeInputs(600U));
  controller.step(makeInputs(900U));

  return require(controller.state() == TowerController::STATE_LEFT_ONLY,
                 "recovery requires >= on threshold");
}

static bool test_BB_lowSupplyWhileInactiveDoesNotActivate() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.step(makeInputs(0U));

  return require(controller.state() == TowerController::STATE_INACTIVE,
                 "inactive stays inactive");
}

static bool test_BB_exactLowSupplyThresholdIsSufficient() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.step(makeInputs(700U));

  return require(controller.state() == TowerController::STATE_LOW_SUPPLY,
                 "700 triggers low supply");
}

static bool test_BB_snapshotReflectsCurrentTowerState() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  const TowerController::Config config = fastConfig();
  TowerController controller(clock, leftValve, rightValve, config);

  controller.setEnabled(true);
  TowerController::Snapshot snapshot = controller.snapshot();

  clock.advanceMs(config.towerOpenMs);
  controller.step(makeInputs(1500U));

  snapshot = controller.snapshot();

  return require(snapshot.state == TowerController::STATE_BOTH_AFTER_LEFT,
                 "snapshot updates after transition");
}

static bool test_BB_hysteresis_activeDropsAtOffThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.step(makeInputs(700U));

  return require(controller.state() == TowerController::STATE_LOW_SUPPLY,
                 "drops at off threshold");
}

static bool test_BB_hysteresis_deadbandHoldsLowSupply() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.step(makeInputs(600U));
  controller.step(makeInputs(800U));

  return require(controller.state() == TowerController::STATE_LOW_SUPPLY,
                 "deadband holds low supply");
}

static bool test_BB_hysteresis_recoveryRequiresOnThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);

  controller.step(makeInputs(600U));
  controller.step(makeInputs(900U));

  return require(controller.state() == TowerController::STATE_LEFT_ONLY,
                 "recovers at ON threshold");
}

static bool test_BB_hysteresis_deadbandHoldsActive() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.step(makeInputs(800U));

  return require(controller.isActive(),
                 "deadband keeps active state");
}

int main() {
  if (!test_BB_startsInactiveWithBothValvesClosed()) return 1;
  if (!test_BB_enableImmediatelyStartsLeftOnly()) return 1;
  if (!test_BB_tickBeforeExpiryDoesNotTransition()) return 1;
  if (!test_BB_fullCycleAdvancesWithMockClockWhenSupplyIsSufficient()) return 1;
  if (!test_BB_disableForcesImmediateInactiveFromActiveState()) return 1;
  if (!test_BB_perStateOutputsAndIsActive()) return 1;
  if (!test_BB_lowSupplyForcesDedicatedLowSupplyState()) return 1;
  if (!test_BB_recoveryFromLowSupplyRestartsAtLeftOnly()) return 1;
  if (!test_BB_lowSupplyWhileInactiveDoesNotActivate()) return 1;
  if (!test_BB_exactLowSupplyThresholdIsSufficient()) return 1;
  if (!test_BB_snapshotReflectsCurrentTowerState()) return 1;
  if (!test_BB_hysteresis_activeDropsAtOffThreshold()) return 1;
  if (!test_BB_hysteresis_deadbandHoldsLowSupply()) return 1;
  if (!test_BB_hysteresis_recoveryRequiresOnThreshold()) return 1;
  if (!test_BB_hysteresis_deadbandHoldsActive()) return 1;

  printf("PASS: test_BB_tower_controller\n");
  return 0;
}