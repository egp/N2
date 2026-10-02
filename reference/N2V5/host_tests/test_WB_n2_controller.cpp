// host_tests/test_WB_n2_controller.cpp v1
#include <stdio.h>

#include "BinaryOutput.h"
#include "N2Controller.h"

class FakeClock : public IClock {
public:
  FakeClock() : nowMs_(0U) {}

  uint32_t nowMs() const override { return nowMs_; }

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

struct N2ControllerTestProbe {
  static void transitionTo(N2Controller& controller, N2Controller::State state) {
    controller.transitionTo(state);
  }

  static void applyOutputForState(N2Controller& controller, N2Controller::State state) {
    controller.applyOutputForState(state);
  }

  static const TimedStateMachine& timedStateMachine(const N2Controller& controller) {
    return controller.timedStateMachine_;
  }

  static const N2Controller::Config& config(const N2Controller& controller) {
    return controller.config_;
  }
};

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static bool test_WB_defaultConfigUsesScaledThresholds() {
  const N2Controller::Config config = N2Controller::defaultConfig();

  if (!require(config.lowOffPsi_x100 == 1000U,
               "default lowOff should be 10.00 PSI in x100 units")) return false;
  if (!require(config.lowOnPsi_x100 == 2000U,
               "default lowOn should be 20.00 PSI in x100 units")) return false;
  if (!require(config.highOnPsi_x10 == 1000U,
               "default highOn should be 100.0 PSI in x10 units")) return false;
  if (!require(config.highOffPsi_x10 == 1200U,
               "default highOff should be 120.0 PSI in x10 units")) return false;

  return true;
}

static bool test_WB_constructorSeedsSafeStateAndDefaultConfig() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor);

  if (!require(N2ControllerTestProbe::timedStateMachine(controller).state() ==
                   static_cast<uint8_t>(N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT),
               "constructor should seed safe low-inhibit/high-permit state")) return false;
  if (!require(!compressor.isOn(),
               "constructor should seed output off")) return false;
  if (!require(N2ControllerTestProbe::config(controller).lowOffPsi_x100 == 1000U,
               "constructor should seed default lowOff config")) return false;
  if (!require(N2ControllerTestProbe::config(controller).lowOnPsi_x100 == 2000U,
               "constructor should seed default lowOn config")) return false;
  if (!require(N2ControllerTestProbe::config(controller).highOnPsi_x10 == 1000U,
               "constructor should seed default highOn config")) return false;
  if (!require(N2ControllerTestProbe::config(controller).highOffPsi_x10 == 1200U,
               "constructor should seed default highOff config")) return false;

  return true;
}

static bool test_WB_transitionToUpdatesStateAndOutput() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, N2Controller::defaultConfig());

  N2ControllerTestProbe::transitionTo(controller, N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT);
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "transitionTo should update timed-state-machine state")) return false;
  if (!require(compressor.isOn(),
               "transitionTo should apply permit/permit output")) return false;

  N2ControllerTestProbe::transitionTo(controller, N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT);
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "transitionTo should update to low-permit/high-inhibit")) return false;
  if (!require(!compressor.isOn(),
               "transitionTo should apply inhibit output")) return false;

  N2ControllerTestProbe::transitionTo(controller, N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT);
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT,
               "transitionTo should update to dual-inhibit")) return false;
  if (!require(!compressor.isOn(),
               "transitionTo should keep dual-inhibit output off")) return false;

  return true;
}

static bool test_WB_applyOutputForStateMatchesOnlyPermitPermit() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, N2Controller::defaultConfig());

  N2ControllerTestProbe::applyOutputForState(controller, N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT);
  if (!require(!compressor.isOn(),
               "applyOutputForState should keep low-inhibit/high-permit off")) return false;

  N2ControllerTestProbe::applyOutputForState(controller, N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT);
  if (!require(compressor.isOn(),
               "applyOutputForState should turn permit/permit on")) return false;

  N2ControllerTestProbe::applyOutputForState(controller, N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT);
  if (!require(!compressor.isOn(),
               "applyOutputForState should keep low-permit/high-inhibit off")) return false;

  N2ControllerTestProbe::applyOutputForState(controller, N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT);
  if (!require(!compressor.isOn(),
               "applyOutputForState should keep dual-inhibit off")) return false;

  return true;
}

static bool test_WB_snapshotTimestampMirrorsTimedStateMachineEnteredAt() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, N2Controller::defaultConfig());

  N2ControllerTestProbe::transitionTo(controller, N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT);
  const N2Controller::Snapshot snapshot = controller.snapshot();

  if (!require(snapshot.createdAtMs ==
                   N2ControllerTestProbe::timedStateMachine(controller).stateEnteredAtMs(),
               "n2 snapshot timestamp should mirror timed-state-machine entered-at")) return false;
  if (!require(snapshot.state == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "n2 snapshot should report current state")) return false;

  return true;
}

int main() {
  if (!test_WB_defaultConfigUsesScaledThresholds()) return 1;
  if (!test_WB_constructorSeedsSafeStateAndDefaultConfig()) return 1;
  if (!test_WB_transitionToUpdatesStateAndOutput()) return 1;
  if (!test_WB_applyOutputForStateMatchesOnlyPermitPermit()) return 1;
  if (!test_WB_snapshotTimestampMirrorsTimedStateMachineEnteredAt()) return 1;

  printf("PASS: test_WB_n2_controller\n");
  return 0;
}
// host_tests/test_WB_n2_controller.cpp v1