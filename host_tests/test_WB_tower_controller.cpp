// host_tests/test_WB_tower_controller.cpp v2
#include <stdio.h>

#include "BinaryOutput.h"
#include "TowerController.h"

class FakeClock : public IClock {
public:
  FakeClock() : nowMs_(0U) {}

  uint32_t nowMs() const override { return nowMs_; }

  void setNowMs(uint32_t nowMs) { nowMs_ = nowMs; }

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

struct TowerControllerTestProbe {
  static bool isSupplySufficient(const TowerController& controller, uint16_t supplyPsi_x10) {
    return controller.isSupplySufficient(supplyPsi_x10);
  }

  static void transitionTo(
      TowerController& controller,
      TowerController::State nextState,
      uint32_t durationMs,
      bool timed) {
    controller.transitionTo(nextState, durationMs, timed);
  }

  static void applyOutputsForState(
      TowerController& controller,
      TowerController::State state) {
    controller.applyOutputsForState(state);
  }

  static const TimedStateMachine& timedStateMachine(const TowerController& controller) {
    return controller.timedStateMachine_;
  }

  static const TowerController::Config& config(const TowerController& controller) {
    return controller.config_;
  }

  static bool enabled(const TowerController& controller) {
    return controller.enabled_;
  }
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

// static bool test_WB_defaultConfigIncludesLowSupplyThreshold() {
//   ...
// }

// static bool test_WB_constructorSeedsConfigAndDisabledState() {
//   ...
// }

static bool test_WB_isSupplySufficientUsesInclusiveThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  // OFF threshold = 700, ON threshold = 900
  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, 699U),
               "below threshold should be insufficient")) return false;

  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, 900U),
               "on threshold should be sufficient")) return false;

  return true;
}

static bool test_WB_transitionToTimedSetsDeadlineAndOutputs() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  clock.setNowMs(42U);

  TowerControllerTestProbe::transitionTo(
      controller,
      TowerController::STATE_RIGHT_ONLY,
      7U,
      true);

  if (!require(controller.state() == TowerController::STATE_RIGHT_ONLY,
               "state should update")) return false;

  if (!require(TowerControllerTestProbe::timedStateMachine(controller).hasDeadline(),
               "deadline should be set")) return false;

  if (!require(TowerControllerTestProbe::timedStateMachine(controller).deadlineAtMs() == 49U,
               "deadline should be now + duration")) return false;

  if (!require(!leftValve.isOn() && rightValve.isOn(),
               "outputs should match right-only")) return false;

  return true;
}

static bool test_WB_transitionToUntimedClearsDeadlineAndOutputs() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  TowerControllerTestProbe::transitionTo(
      controller,
      TowerController::STATE_LEFT_ONLY,
      10U,
      true);

  clock.advanceMs(5U);

  TowerControllerTestProbe::transitionTo(
      controller,
      TowerController::STATE_LOW_SUPPLY,
      0U,
      false);

  if (!require(controller.state() == TowerController::STATE_LOW_SUPPLY,
               "state should update")) return false;

  if (!require(!TowerControllerTestProbe::timedStateMachine(controller).hasDeadline(),
               "deadline should be cleared")) return false;

  if (!require(TowerControllerTestProbe::timedStateMachine(controller).deadlineAtMs() == 0U,
               "deadline should be zero")) return false;

  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "outputs should match low-supply")) return false;

  return true;
}

static bool test_WB_applyOutputsForStateMatchesEveryState() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_INACTIVE);
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "inactive")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_LEFT_ONLY);
  if (!require(leftValve.isOn() && !rightValve.isOn(),
               "left-only")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_BOTH_AFTER_LEFT);
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both-after-left")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_RIGHT_ONLY);
  if (!require(!leftValve.isOn() && rightValve.isOn(),
               "right-only")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_BOTH_AFTER_RIGHT);
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both-after-right")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_LOW_SUPPLY);
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "low-supply")) return false;

  return true;
}

static bool test_WB_snapshotTimestampMirrorsTimedStateMachineEnteredAt() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  clock.setNowMs(55U);

  TowerControllerTestProbe::transitionTo(
      controller,
      TowerController::STATE_RIGHT_ONLY,
      12U,
      true);

  const TowerController::Snapshot snapshot = controller.snapshot();

  if (!require(snapshot.createdAtMs ==
               TowerControllerTestProbe::timedStateMachine(controller).stateEnteredAtMs(),
               "timestamp mismatch")) return false;

  if (!require(snapshot.state == TowerController::STATE_RIGHT_ONLY,
               "state mismatch")) return false;

  return true;
}

static bool test_WB_isSupplySufficient_activeUsesOffThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);

  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, 700U),
               "active at off threshold fails")) return false;

  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, 701U),
               "active above off threshold passes")) return false;

  return true;
}

static bool test_WB_isSupplySufficient_inactiveUsesOnThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, 899U),
               "inactive below ON fails")) return false;

  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, 900U),
               "inactive at ON passes")) return false;

  return true;
}

static bool test_WB_isSupplySufficient_hasDeadbandGap() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  const uint16_t mid = 800U;

  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, mid),
               "inactive deadband fails")) return false;

  controller.setEnabled(true);

  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, mid),
               "active deadband passes")) return false;

  return true;
}

static bool test_WB_isSupplySufficient_matchesStepSemantics() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);
  controller.step(makeInputs(600U));
  controller.step(makeInputs(900U));

  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY,
               "step recovery expected")) return false;

  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, 900U),
               "helper matches step semantics")) return false;

  return true;
}

int main() {
  if (!test_WB_isSupplySufficientUsesInclusiveThreshold()) return 1;
  if (!test_WB_transitionToTimedSetsDeadlineAndOutputs()) return 1;
  if (!test_WB_transitionToUntimedClearsDeadlineAndOutputs()) return 1;
  if (!test_WB_applyOutputsForStateMatchesEveryState()) return 1;
  if (!test_WB_snapshotTimestampMirrorsTimedStateMachineEnteredAt()) return 1;
  if (!test_WB_isSupplySufficient_activeUsesOffThreshold()) return 1;
  if (!test_WB_isSupplySufficient_inactiveUsesOnThreshold()) return 1;
  if (!test_WB_isSupplySufficient_hasDeadbandGap()) return 1;
  if (!test_WB_isSupplySufficient_matchesStepSemantics()) return 1;

  printf("PASS: test_WB_tower_controller\n");
  return 0;
}
// host_tests/test_WB_tower_controller.cpp v2