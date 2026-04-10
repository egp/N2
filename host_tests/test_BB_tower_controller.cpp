// host_tests/test_BB_tower_controller.cpp v1
#include <stdio.h>

#include "TowerController.h"

class FakeClock : public IClock {
public:
  FakeClock() : nowMs_(0U) {}

  uint32_t nowMs() const override {
    return nowMs_;
  }

  void setNowMs(uint32_t nowMs) {
    nowMs_ = nowMs;
  }

  void advanceMs(uint32_t deltaMs) {
    nowMs_ += deltaMs;
  }

private:
  uint32_t nowMs_;
};

class FakeTowerValveDriver : public ITowerValveDriver {
public:
  FakeTowerValveDriver() : leftOpen_(false), rightOpen_(false) {}

  void setLeftOpen(bool open) override {
    leftOpen_ = open;
  }

  void setRightOpen(bool open) override {
    rightOpen_ = open;
  }

  bool leftOpen() const {
    return leftOpen_;
  }

  bool rightOpen() const {
    return rightOpen_;
  }

private:
  bool leftOpen_;
  bool rightOpen_;
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
  return config;
}

static bool test_BB_startsInactiveWithBothValvesClosed() {
  FakeClock clock;
  FakeTowerValveDriver valves;
  TowerController controller(clock, valves, testConfig());

  if (!require(controller.state() == TowerController::STATE_INACTIVE, "controller should start inactive")) return false;
  if (!require(!valves.leftOpen(), "left valve should start closed")) return false;
  if (!require(!valves.rightOpen(), "right valve should start closed")) return false;

  return true;
}

static bool test_BB_fullCycleAdvancesWithMockClock() {
  FakeClock clock;
  FakeTowerValveDriver valves;
  TowerController controller(clock, valves, testConfig());

  controller.setEnabled(true);

  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY, "enable should enter left-only state")) return false;
  if (!require(valves.leftOpen(), "left valve should be open in left-only state")) return false;
  if (!require(!valves.rightOpen(), "right valve should be closed in left-only state")) return false;

  clock.advanceMs(60000U);
  controller.tick();
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT, "left-only should transition to first overlap")) return false;
  if (!require(valves.leftOpen() && valves.rightOpen(), "both valves should be open in first overlap")) return false;

  clock.advanceMs(750U);
  controller.tick();
  if (!require(controller.state() == TowerController::STATE_RIGHT_ONLY, "first overlap should transition to right-only")) return false;
  if (!require(!valves.leftOpen() && valves.rightOpen(), "only right valve should be open in right-only")) return false;

  clock.advanceMs(60000U);
  controller.tick();
  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_RIGHT, "right-only should transition to second overlap")) return false;
  if (!require(valves.leftOpen() && valves.rightOpen(), "both valves should be open in second overlap")) return false;

  clock.advanceMs(750U);
  controller.tick();
  if (!require(controller.state() == TowerController::STATE_LEFT_ONLY, "second overlap should transition back to left-only")) return false;
  if (!require(valves.leftOpen() && !valves.rightOpen(), "cycle should return to left-only outputs")) return false;

  return true;
}

static bool test_BB_disableForcesImmediateInactiveFromAnyActiveState() {
  FakeClock clock;
  FakeTowerValveDriver valves;
  TowerController controller(clock, valves, testConfig());

  controller.setEnabled(true);
  clock.advanceMs(60000U);
  controller.tick();

  if (!require(controller.state() == TowerController::STATE_BOTH_AFTER_LEFT, "controller should reach active overlap state")) return false;

  controller.setEnabled(false);

  if (!require(controller.state() == TowerController::STATE_INACTIVE, "disable should force inactive state")) return false;
  if (!require(!valves.leftOpen() && !valves.rightOpen(), "disable should close both valves immediately")) return false;

  return true;
}

int main() {
  if (!test_BB_startsInactiveWithBothValvesClosed()) return 1;
  if (!test_BB_fullCycleAdvancesWithMockClock()) return 1;
  if (!test_BB_disableForcesImmediateInactiveFromAnyActiveState()) return 1;

  printf("PASS: test_BB_tower_controller\n");
  return 0;
}
// host_tests/test_BB_tower_controller.cpp v1