// host_tests/test_BB_n2_controller.cpp v3
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

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static N2Controller::Config testConfig() {
  N2Controller::Config config;
  config.lowOffPsi_x100 = 1000U;   // 10.00 PSI
  config.lowOnPsi_x100 = 2000U;    // 20.00 PSI
  config.highOnPsi_x10 = 1000U;    // 100.0 PSI
  config.highOffPsi_x10 = 1200U;   // 120.0 PSI
  return config;
}

static InputSnapshot makeInputs(uint16_t lowN2Psi_x100, uint16_t highN2Psi_x10) {
  InputSnapshot inputs{};
  inputs.sampledAtMs = 0U;
  inputs.blackSwitchEnabled = true;
  inputs.supplyPsi_x10 = 0U;
  inputs.leftTowerPsi_x10 = 0U;
  inputs.rightTowerPsi_x10 = 0U;
  inputs.lowN2Psi_x100 = lowN2Psi_x100;
  inputs.highN2Psi_x10 = highN2Psi_x10;
  return inputs;
}

static bool test_BB_startsInSafeOffState() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "controller should start in low-inhibit/high-permit state")) return false;
  if (!require(!controller.isCompressorOn(),
               "compressor should start off")) return false;
  if (!require(!compressor.isOn(),
               "output should start off")) return false;
  if (!require(controller.isOk(),
               "initial state should report ok")) return false;

  return true;
}

static bool test_BB_turnsOnOnlyWhenBothPermit() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(2500U, 900U));

  if (!require(controller.isOk(),
               "permit/permit state should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "controller should enter permit/permit")) return false;
  if (!require(controller.isCompressorOn(),
               "compressor should be on in permit/permit")) return false;
  if (!require(compressor.isOn(),
               "output should be on in permit/permit")) return false;

  return true;
}

static bool test_BB_lowBandHoldsPreviousPermit() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(2500U, 900U));
  if (!require(controller.isOk(),
               "priming permit/permit state should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "priming update should enter permit/permit")) return false;

  controller.step(makeInputs(1500U, 900U));

  if (!require(controller.isOk(),
               "low-band hold should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "low-side hysteresis should hold permit in 10..20 band")) return false;
  if (!require(controller.isCompressorOn(),
               "compressor should stay on in low band")) return false;
  if (!require(compressor.isOn(),
               "output should stay on in low band")) return false;

  return true;
}

static bool test_BB_lowOffTurnsCompressorOff() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(900U, 900U));

  if (!require(controller.isOk(),
               "single-inhibit state should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "low below off threshold should inhibit low side")) return false;
  if (!require(!controller.isCompressorOn(),
               "compressor should be off when low side inhibits")) return false;
  if (!require(!compressor.isOn(),
               "output should be off when low side inhibits")) return false;

  return true;
}

static bool test_BB_highOffTurnsCompressorOff() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(2500U, 900U));
  controller.step(makeInputs(2500U, 1300U));

  if (!require(controller.isOk(),
               "single-inhibit state should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "high above off threshold should inhibit high side")) return false;
  if (!require(!controller.isCompressorOn(),
               "compressor should be off when high side inhibits")) return false;
  if (!require(!compressor.isOn(),
               "output should be off when high side inhibits")) return false;

  return true;
}

static bool test_BB_highBandHoldsPreviousInhibit() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(2500U, 1300U));
  controller.step(makeInputs(2500U, 1100U));

  if (!require(controller.isOk(),
               "high-band hold should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "high-side hysteresis should hold inhibit in 100..120 band")) return false;
  if (!require(!controller.isCompressorOn(),
               "compressor should stay off in high band")) return false;
  if (!require(!compressor.isOn(),
               "output should stay off in high band")) return false;

  return true;
}

static bool test_BB_dualInhibitReturnsFalse() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(900U, 1300U));

  if (!require(!controller.isOk(),
               "dual-inhibit state should report not ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT,
               "controller should enter dual-inhibit state")) return false;
  if (!require(!controller.isCompressorOn(),
               "compressor should be off in dual-inhibit")) return false;
  if (!require(!compressor.isOn(),
               "output should be off in dual-inhibit")) return false;

  return true;
}

static bool test_BB_singleInhibitStatesReturnTrue() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(900U, 900U));
  if (!require(controller.isOk(),
               "low-inhibit/high-permit should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "controller should enter low-inhibit/high-permit")) return false;

  controller.step(makeInputs(2500U, 1300U));
  if (!require(controller.isOk(),
               "low-permit/high-inhibit should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "controller should enter low-permit/high-inhibit")) return false;

  return true;
}

static bool test_BB_exactThresholdBoundariesHoldPriorLatchState() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  // Exact lowOn should not newly permit from low inhibit.
  controller.step(makeInputs(900U, 900U));
  controller.step(makeInputs(2000U, 900U));
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "exact lowOn should hold low inhibit")) return false;

  // Exact lowOff should not newly inhibit from low permit.
  controller.step(makeInputs(2500U, 900U));
  controller.step(makeInputs(1000U, 900U));
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "exact lowOff should hold low permit")) return false;

  // Exact highOff should not newly inhibit from high permit.
  controller.step(makeInputs(2500U, 900U));
  controller.step(makeInputs(2500U, 1200U));
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "exact highOff should hold high permit")) return false;

  // Exact highOn should not newly permit from high inhibit.
  controller.step(makeInputs(2500U, 1300U));
  controller.step(makeInputs(2500U, 1000U));
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "exact highOn should hold high inhibit")) return false;

  return true;
}

static bool test_BB_outputIsOnOnlyInPermitPermit() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(900U, 900U));
  if (!require(!compressor.isOn(),
               "output should be off in low-inhibit/high-permit")) return false;

  controller.step(makeInputs(2500U, 900U));
  if (!require(compressor.isOn(),
               "output should be on in permit/permit")) return false;

  controller.step(makeInputs(2500U, 1300U));
  if (!require(!compressor.isOn(),
               "output should be off in low-permit/high-inhibit")) return false;

  controller.step(makeInputs(900U, 1300U));
  if (!require(!compressor.isOn(),
               "output should be off in dual-inhibit")) return false;

  return true;
}

static bool test_BB_hysteresisMemorySurvivesOscillationInsideBothHoldBands() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(2500U, 900U));
  if (!require(controller.isOk(),
               "initial permit/permit state should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "controller should enter permit/permit")) return false;

  controller.step(makeInputs(1500U, 1100U));
  if (!require(controller.isOk(),
               "both hold-band inputs should still report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "both hold bands should preserve prior permit/permit latch state")) return false;
  if (!require(compressor.isOn(),
               "compressor should remain on while both latches hold permit")) return false;

  controller.step(makeInputs(1500U, 1300U));
  if (!require(controller.isOk(),
               "high inhibit transition should still report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "controller should enter low-permit/high-inhibit")) return false;
  if (!require(!compressor.isOn(),
               "compressor should turn off when high inhibits")) return false;

  controller.step(makeInputs(1500U, 1100U));
  if (!require(controller.isOk(),
               "high hold-band input should still report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "high hold band should preserve prior high-inhibit latch state")) return false;

  controller.step(makeInputs(900U, 1100U));
  if (!require(!controller.isOk(),
               "dual-inhibit state should report not ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT,
               "controller should enter dual-inhibit")) return false;
  if (!require(!compressor.isOn(),
               "compressor should remain off in dual-inhibit")) return false;

  controller.step(makeInputs(1500U, 1100U));
  if (!require(!controller.isOk(),
               "both hold-band inputs should preserve dual-inhibit not-ok state")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT,
               "both hold bands should preserve prior dual-inhibit latch state")) return false;

  controller.step(makeInputs(1500U, 900U));
  if (!require(controller.isOk(),
               "high permit transition should report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "controller should enter low-inhibit/high-permit")) return false;
  if (!require(!compressor.isOn(),
               "compressor should stay off while low still inhibits")) return false;

  controller.step(makeInputs(1500U, 1100U));
  if (!require(controller.isOk(),
               "final hold-band inputs should still report ok")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "hold bands should preserve low-inhibit/high-permit latch state")) return false;

  return true;
}

static bool test_BB_snapshotReflectsCurrentN2State() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.step(makeInputs(2500U, 900U));
  const N2Controller::Snapshot snapshot = controller.snapshot();

  if (!require(snapshot.createdAtMs == 0U,
               "n2 snapshot timestamp should reflect state entry time")) return false;
  if (!require(snapshot.state == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "n2 snapshot should report current state")) return false;

  return true;
}

int main() {
  if (!test_BB_startsInSafeOffState()) return 1;
  if (!test_BB_turnsOnOnlyWhenBothPermit()) return 1;
  if (!test_BB_lowBandHoldsPreviousPermit()) return 1;
  if (!test_BB_lowOffTurnsCompressorOff()) return 1;
  if (!test_BB_highOffTurnsCompressorOff()) return 1;
  if (!test_BB_highBandHoldsPreviousInhibit()) return 1;
  if (!test_BB_dualInhibitReturnsFalse()) return 1;
  if (!test_BB_singleInhibitStatesReturnTrue()) return 1;
  if (!test_BB_exactThresholdBoundariesHoldPriorLatchState()) return 1;
  if (!test_BB_outputIsOnOnlyInPermitPermit()) return 1;
  if (!test_BB_hysteresisMemorySurvivesOscillationInsideBothHoldBands()) return 1;
  if (!test_BB_snapshotReflectsCurrentN2State()) return 1;

  printf("PASS: test_BB_n2_controller\n");
  return 0;
}
// host_tests/test_BB_n2_controller.cpp v3