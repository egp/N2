// host_tests/test_BB_n2_controller.cpp v2
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
  return true;
}

static bool test_BB_turnsOnOnlyWhenBothPermit() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  const bool ok = controller.update(makeInputs(2500U, 900U));

  if (!require(ok, "permit/permit state should return true")) return false;
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

  controller.update(makeInputs(2500U, 900U));
  const bool ok = controller.update(makeInputs(1500U, 900U));

  if (!require(ok, "low-band hold should return true")) return false;
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

  const bool ok = controller.update(makeInputs(900U, 900U));

  if (!require(ok, "single-inhibit state should return true")) return false;
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

  controller.update(makeInputs(2500U, 900U));
  const bool ok = controller.update(makeInputs(2500U, 1300U));  

  if (!require(ok, "single-inhibit state should return true")) return false;
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

  controller.update(makeInputs(2500U, 1300U));
  const bool ok = controller.update(makeInputs(2500U, 1100U));

  if (!require(ok, "high-band hold should return true")) return false;
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

  const bool ok = controller.update(makeInputs(900U, 1300U));

  if (!require(!ok, "dual-inhibit state should return false")) return false;
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

  const bool lowInhibitHighPermit = controller.update(makeInputs(900U, 900U));
  if (!require(lowInhibitHighPermit,
               "low-inhibit/high-permit should return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "controller should enter low-inhibit/high-permit")) return false;

  const bool lowPermitHighInhibit = controller.update(makeInputs(2500U, 1300U));
  if (!require(lowPermitHighInhibit,
               "low-permit/high-inhibit should return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "controller should enter low-permit/high-inhibit")) return false;

  return true;
}

static bool test_BB_exactThresholdBoundariesHoldPriorLatchState() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  // Exact lowOn should not newly permit from low inhibit.
  controller.update(makeInputs(900U, 900U));
  controller.update(makeInputs(2000U, 900U));
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "exact lowOn should hold low inhibit")) return false;

  // Exact lowOff should not newly inhibit from low permit.
  controller.update(makeInputs(2500U, 900U));
  controller.update(makeInputs(1000U, 900U));
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "exact lowOff should hold low permit")) return false;

  // Exact highOff should not newly inhibit from high permit.
  controller.update(makeInputs(2500U, 900U));
  controller.update(makeInputs(2500U, 1200U));
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "exact highOff should hold high permit")) return false;

  // Exact highOn should not newly permit from high inhibit.
  controller.update(makeInputs(2500U, 1300U));
  controller.update(makeInputs(2500U, 1000U));
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "exact highOn should hold high inhibit")) return false;

  return true;
}

static bool test_BB_outputIsOnOnlyInPermitPermit() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  controller.update(makeInputs(900U, 900U));
  if (!require(!compressor.isOn(),
               "output should be off in low-inhibit/high-permit")) return false;

  controller.update(makeInputs(2500U, 900U));
  if (!require(compressor.isOn(),
               "output should be on in permit/permit")) return false;

  controller.update(makeInputs(2500U, 1300U));
  if (!require(!compressor.isOn(),
               "output should be off in low-permit/high-inhibit")) return false;

  controller.update(makeInputs(900U, 1300U));
  if (!require(!compressor.isOn(),
               "output should be off in dual-inhibit")) return false;

  return true;
}

static bool test_BB_hysteresisMemorySurvivesOscillationInsideBothHoldBands() {
  FakeClock clock;
  FakeBinaryOutput compressor;
  N2Controller controller(clock, compressor, testConfig());

  if (!require(controller.update(makeInputs(2500U, 900U)),
               "initial permit/permit update should return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "controller should enter permit/permit")) return false;

  if (!require(controller.update(makeInputs(1500U, 1100U)),
               "both hold-band inputs should still return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
               "both hold bands should preserve prior permit/permit latch state")) return false;
  if (!require(compressor.isOn(),
               "compressor should remain on while both latches hold permit")) return false;

  if (!require(controller.update(makeInputs(1500U, 1300U)),
               "low inhibit transition should still return true"
               "high inhibit transition should still return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "controller should enter low-permit/high-inhibit")) return false;
  if (!require(!compressor.isOn(),
               "compressor should turn off when high inhibits")) return false;

  if (!require(controller.update(makeInputs(1500U, 1100U)),
               "high hold-band input should still return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
               "high hold band should preserve prior high-inhibit latch state")) return false;

  if (!require(!controller.update(makeInputs(900U, 1100U)),
               "dual-inhibit state should return false")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT,
               "controller should enter dual-inhibit")) return false;
  if (!require(!compressor.isOn(),
               "compressor should remain off in dual-inhibit")) return false;

  if (!require(!controller.update(makeInputs(1500U, 1100U)),
               "both hold-band inputs should preserve dual-inhibit latch state")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT,
               "both hold bands should preserve prior dual-inhibit latch state")) return false;

  if (!require(controller.update(makeInputs(1500U, 900U)),
               "high permit transition should return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "controller should enter low-inhibit/high-permit")) return false;
  if (!require(!compressor.isOn(),
               "compressor should stay off while low still inhibits")) return false;

  if (!require(controller.update(makeInputs(1500U, 1100U)),
               "final hold-band inputs should still return true")) return false;
  if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
               "hold bands should preserve low-inhibit/high-permit latch state")) return false;

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

  printf("PASS: test_BB_n2_controller\n");
  return 0;
}
// host_tests/test_BB_n2_controller.cpp v2