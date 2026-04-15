// host_tests/test_BB_n2_controller.cpp v1
#include <stdio.h>
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
 FakeBinaryOutput() : isOn_(false) {}
 void setOn(bool on) override { isOn_ = on; }
 bool isOn() const { return isOn_; }
private:
 bool isOn_;
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
 config.lowOffPsi_x100 = 1000U;
 config.lowOnPsi_x100 = 2000U;
 config.highOnPsi_x10 = 1000U;
 config.highOffPsi_x10 = 1200U;
 return config;
}

static bool test_BB_startsOff() {
 FakeClock clock;
 FakeBinaryOutput output;
 N2Controller controller(clock, output, testConfig());

 if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
              "controller should start low-inhibit/high-permit")) return false;
 if (!require(!controller.isCompressorOn(), "compressor should start off")) return false;
 if (!require(!output.isOn(), "output should start off")) return false;
 return true;
}

static bool test_BB_turnsOnOnlyWhenBothPermit() {
 FakeClock clock;
 FakeBinaryOutput output;
 N2Controller controller(clock, output, testConfig());

 controller.update(2500U, 900U);

 if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
              "controller should be in permit/permit state")) return false;
 if (!require(controller.isCompressorOn(), "compressor should be on")) return false;
 if (!require(output.isOn(), "output should be on")) return false;
 return true;
}

static bool test_BB_lowBandHoldsPreviousPermit() {
 FakeClock clock;
 FakeBinaryOutput output;
 N2Controller controller(clock, output, testConfig());

 controller.update(2500U, 900U);
 controller.update(1500U, 900U);

 if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT,
              "low-side hysteresis should hold permit inside 10..20 band")) return false;
 if (!require(output.isOn(), "output should stay on inside low band")) return false;
 return true;
}

static bool test_BB_lowOffTurnsCompressorOff() {
 FakeClock clock;
 FakeBinaryOutput output;
 N2Controller controller(clock, output, testConfig());

 controller.update(2500U, 900U);
 controller.update(900U, 900U);

 if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT,
              "low below off threshold should inhibit")) return false;
 if (!require(!output.isOn(), "output should be off when low inhibits")) return false;
 return true;
}

static bool test_BB_highOffTurnsCompressorOff() {
 FakeClock clock;
 FakeBinaryOutput output;
 N2Controller controller(clock, output, testConfig());

 controller.update(2500U, 900U);
 controller.update(2500U, 1300U);

 if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
              "high above off threshold should inhibit")) return false;
 if (!require(!output.isOn(), "output should be off when high inhibits")) return false;
 return true;
}

static bool test_BB_highBandHoldsPreviousInhibit() {
 FakeClock clock;
 FakeBinaryOutput output;
 N2Controller controller(clock, output, testConfig());

 controller.update(2500U, 1300U);
 controller.update(2500U, 1100U);

 if (!require(controller.state() == N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT,
              "high-side hysteresis should hold inhibit inside 100..120 band")) return false;
 if (!require(!output.isOn(), "output should stay off inside high band")) return false;
 return true;
}

static bool test_BB_dualInhibitStateExists() {
 FakeClock clock;
 FakeBinaryOutput output;
 N2Controller controller(clock, output, testConfig());

 controller.update(900U, 1300U);

 if (!require(controller.state() == N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT,
              "dual-inhibit state should be representable")) return false;
 if (!require(!output.isOn(), "output should be off in dual-inhibit")) return false;
 return true;
}

int main() {
 if (!test_BB_startsOff()) return 1;
 if (!test_BB_turnsOnOnlyWhenBothPermit()) return 1;
 if (!test_BB_lowBandHoldsPreviousPermit()) return 1;
 if (!test_BB_lowOffTurnsCompressorOff()) return 1;
 if (!test_BB_highOffTurnsCompressorOff()) return 1;
 if (!test_BB_highBandHoldsPreviousInhibit()) return 1;
 if (!test_BB_dualInhibitStateExists()) return 1;

 printf("PASS: test_BB_n2_controller\n");
 return 0;
}
// host_tests/test_BB_n2_controller.cpp v1