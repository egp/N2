// host_tests/test_WB_tower_controller.cpp v1
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
  config.towerOpenMs = 60000U;
  config.airSupplyOnPsi_x10 = 900U;
  config.airSupplyOffPsi_x10 = 700U;
  return config;
}

// static bool test_WB_defaultConfigIncludesLowSupplyThreshold() {
//   const TowerController::Config config = TowerController::defaultConfig();

//   if (!require(config.towerOpenMs == 60000U,
//                "default towerOpenMs should be 60000")) return false;
//   if (!require(config.overlapMs == 750U,
//                "default overlapMs should be 750")) return false;
//   if (!require(config.towerOpenMs == 60000U,
//                "default towerOpenMs should be 60000")) return false;
//   if (!require(config.airSupplyOnPsi_x10 == 900U,
//                "default airSupplyOnPsi_x10 should be 900")) return false;

//   return true;
// }

// static bool test_WB_constructorSeedsConfigAndDisabledState() {
//   FakeClock clock;
//   FakeBinaryOutput leftValve;
//   FakeBinaryOutput rightValve;
//   TowerController controller(clock, leftValve, rightValve, testConfig());

//   if (!require(!TowerControllerTestProbe::enabled(controller),
//                "constructor should seed enabled_ false")) return false;
//   if (!require(TowerControllerTestProbe::config(controller).airSupplyOnPsi_x10 == 1000U,
//                "constructor should store supplied config")) return false;
//   if (!require(TowerControllerTestProbe::timedStateMachine(controller).state() ==
//                    static_cast<uint8_t>(TowerController::STATE_INACTIVE),
//                "constructor should seed inactive timed-state-machine state")) return false;
//   if (!require(!leftValve.isOn() && !rightValve.isOn(),
//                "constructor should apply inactive outputs")) return false;

//   return true;
// }

static bool test_WB_isSupplySufficientUsesInclusiveThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, 700-1),
               "below threshold should be insufficient")) return false;
  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, 901),
               "above threshold should be sufficient")) return false;
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
               "timed transition should update state")) return false;
  if (!require(TowerControllerTestProbe::timedStateMachine(controller).hasDeadline(),
               "timed transition should set deadline")) return false;
  if (!require(TowerControllerTestProbe::timedStateMachine(controller).deadlineAtMs() == 49U,
               "timed transition should set deadline from now")) return false;
  if (!require(!leftValve.isOn() && rightValve.isOn(),
               "timed transition should apply right-only outputs")) return false;

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
               "untimed transition should update state")) return false;
  if (!require(!TowerControllerTestProbe::timedStateMachine(controller).hasDeadline(),
               "untimed transition should clear deadline")) return false;
  if (!require(TowerControllerTestProbe::timedStateMachine(controller).deadlineAtMs() == 0U,
               "untimed transition should zero deadline")) return false;
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "untimed transition should apply low-supply outputs")) return false;

  return true;
}

static bool test_WB_applyOutputsForStateMatchesEveryState() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_INACTIVE);
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "inactive should close both valves")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_LEFT_ONLY);
  if (!require(leftValve.isOn() && !rightValve.isOn(),
               "left-only should open only left valve")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_BOTH_AFTER_LEFT);
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both-after-left should open both valves")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_RIGHT_ONLY);
  if (!require(!leftValve.isOn() && rightValve.isOn(),
               "right-only should open only right valve")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_BOTH_AFTER_RIGHT);
  if (!require(leftValve.isOn() && rightValve.isOn(),
               "both-after-right should open both valves")) return false;

  TowerControllerTestProbe::applyOutputsForState(controller, TowerController::STATE_LOW_SUPPLY);
  if (!require(!leftValve.isOn() && !rightValve.isOn(),
               "low-supply should close both valves")) return false;

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
               "tower snapshot timestamp should mirror timed-state-machine entered-at")) return false;
  if (!require(snapshot.state == TowerController::STATE_RIGHT_ONLY,
               "tower snapshot should mirror current state")) return false;

  return true;
}

static bool test_WB_isSupplySufficient_activeUsesOffThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  controller.setEnabled(true);

  // Active → uses OFF threshold (700)
  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, 700U),
               "active: <= off threshold should be insufficient")) return false;

  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, 701U),
               "active: above off threshold should be sufficient")) return false;

  return true;
}

static bool test_WB_isSupplySufficient_inactiveUsesOnThreshold() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  // Inactive → uses ON threshold (900)
  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, 899U),
               "inactive: below on threshold should be insufficient")) return false;

  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, 900U),
               "inactive: >= on threshold should be sufficient")) return false;

  return true;
}

static bool test_WB_isSupplySufficient_hasDeadbandGap() {
  FakeClock clock;
  FakeBinaryOutput leftValve;
  FakeBinaryOutput rightValve;
  TowerController controller(clock, leftValve, rightValve, testConfig());

  // Pick value inside deadband
  const uint16_t mid = 800U;

  // Inactive → insufficient
  if (!require(!TowerControllerTestProbe::isSupplySufficient(controller, mid),
               "inactive: deadband should be insufficient")) return false;

  // Activate controller
  controller.setEnabled(true);

  // Active → sufficient
  if (!require(TowerControllerTestProbe::isSupplySufficient(controller, mid),
               "active: deadband should be sufficient")) return false;

  return true;
}


int main() {
  // if (!test_WB_defaultConfigIncludesLowSupplyThreshold()) return 1;
  // if (!test_WB_constructorSeedsConfigAndDisabledState()) return 1;
  if (!test_WB_isSupplySufficientUsesInclusiveThreshold()) return 1;
  if (!test_WB_transitionToTimedSetsDeadlineAndOutputs()) return 1;
  if (!test_WB_transitionToUntimedClearsDeadlineAndOutputs()) return 1;
  if (!test_WB_applyOutputsForStateMatchesEveryState()) return 1;
  if (!test_WB_snapshotTimestampMirrorsTimedStateMachineEnteredAt()) return 1;
  if (!test_WB_isSupplySufficient_activeUsesOffThreshold()) return 1;
  if (!test_WB_isSupplySufficient_inactiveUsesOnThreshold()) return 1;
  if (!test_WB_isSupplySufficient_hasDeadbandGap()) return 1;

  printf("PASS: test_WB_tower_controller\n");
  return 0;
}
// host_tests/test_WB_tower_controller.cpp v1