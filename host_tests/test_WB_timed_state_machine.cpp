// host_tests/test_WB_timed_state_machine.cpp v1
#include <stdio.h>

#include "TimedStateMachine.h"

class FakeClock : public IClock {
public:
  FakeClock() : nowMs_(0U) {}

  uint32_t nowMs() const override { return nowMs_; }

  void setNowMs(uint32_t nowMs) { nowMs_ = nowMs; }

  void advanceMs(uint32_t deltaMs) { nowMs_ += deltaMs; }

private:
  uint32_t nowMs_;
};

static const char* testStateName(uint8_t state) {
  switch (state) {
    case 1U: return "One";
    case 2U: return "Two";
    case 3U: return "Three";
    default: return "Unknown";
  }
}

struct TimedStateMachineTestProbe {
  static IClock& clock(TimedStateMachine& machine) {
    return machine.clock_;
  }

  static uint8_t state(const TimedStateMachine& machine) {
    return machine.state_;
  }

  static uint32_t stateEnteredAtMs(const TimedStateMachine& machine) {
    return machine.stateEnteredAtMs_;
  }

  static uint32_t deadlineAtMs(const TimedStateMachine& machine) {
    return machine.deadlineAtMs_;
  }

  static bool hasDeadline(const TimedStateMachine& machine) {
    return machine.hasDeadline_;
  }

  static const char* controllerName(const TimedStateMachine& machine) {
    return machine.controllerName_;
  }

  static TimedStateMachine::StateNameFn stateNameFn(const TimedStateMachine& machine) {
    return machine.stateNameFn_;
  }
};

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static bool test_WB_namedConstructorStoresClockAndOptionalNames() {
  FakeClock clock;
  clock.setNowMs(17U);

  TimedStateMachine machine(clock, 2U, "Tower", testStateName);

  if (!require(&TimedStateMachineTestProbe::clock(machine) == &clock,
               "constructor should store supplied clock reference")) return false;
  if (!require(TimedStateMachineTestProbe::state(machine) == 2U,
               "constructor should store initial state internally")) return false;
  if (!require(TimedStateMachineTestProbe::stateEnteredAtMs(machine) == 17U,
               "constructor should store entered-at time internally")) return false;
  if (!require(TimedStateMachineTestProbe::controllerName(machine) != nullptr,
               "constructor should store controller name pointer")) return false;
  if (!require(TimedStateMachineTestProbe::stateNameFn(machine) != nullptr,
               "constructor should store state-name function pointer")) return false;
  if (!require(TimedStateMachineTestProbe::stateNameFn(machine)(3U) != nullptr,
               "stored state-name function should be callable")) return false;

  return true;
}

static bool test_WB_setDeadlineFromNowWritesInternalDeadlineState() {
  FakeClock clock;
  clock.setNowMs(33U);

  TimedStateMachine machine(clock, 4U);
  machine.setDeadlineFromNow(12U);

  if (!require(TimedStateMachineTestProbe::hasDeadline(machine),
               "setDeadlineFromNow should set internal deadline flag")) return false;
  if (!require(TimedStateMachineTestProbe::deadlineAtMs(machine) == 45U,
               "setDeadlineFromNow should store deadline at now+duration")) return false;
  if (!require(machine.hasDeadline(),
               "public hasDeadline should reflect internal deadline flag")) return false;
  if (!require(machine.deadlineAtMs() == 45U,
               "public deadlineAtMs should reflect internal deadline")) return false;

  return true;
}

static bool test_WB_clearDeadlineZerosInternalDeadlineState() {
  FakeClock clock;
  TimedStateMachine machine(clock, 1U);

  machine.setDeadlineFromNow(9U);
  machine.clearDeadline();

  if (!require(!TimedStateMachineTestProbe::hasDeadline(machine),
               "clearDeadline should clear internal deadline flag")) return false;
  if (!require(TimedStateMachineTestProbe::deadlineAtMs(machine) == 0U,
               "clearDeadline should zero internal deadline")) return false;
  if (!require(!machine.hasDeadline(),
               "public hasDeadline should reflect cleared flag")) return false;
  if (!require(machine.deadlineAtMs() == 0U,
               "public deadlineAtMs should reflect cleared deadline")) return false;

  return true;
}

static bool test_WB_transitionToMutatesInternalStateAndClearsInternalDeadline() {
  FakeClock clock;
  TimedStateMachine machine(clock, 5U);

  machine.setDeadlineFromNow(100U);
  clock.advanceMs(23U);
  machine.transitionTo(9U);

  if (!require(TimedStateMachineTestProbe::state(machine) == 9U,
               "transitionTo should update internal state")) return false;
  if (!require(TimedStateMachineTestProbe::stateEnteredAtMs(machine) == 23U,
               "transitionTo should update internal entered-at time")) return false;
  if (!require(!TimedStateMachineTestProbe::hasDeadline(machine),
               "transitionTo should clear internal deadline flag")) return false;
  if (!require(TimedStateMachineTestProbe::deadlineAtMs(machine) == 0U,
               "transitionTo should zero internal deadline")) return false;

  return true;
}

static bool test_WB_transitionToForUsesTransitionThenSetsInternalDeadline() {
  FakeClock clock;
  clock.setNowMs(100U);

  TimedStateMachine machine(clock, 1U);
  machine.transitionToFor(3U, 7U);

  if (!require(TimedStateMachineTestProbe::state(machine) == 3U,
               "transitionToFor should update internal state")) return false;
  if (!require(TimedStateMachineTestProbe::stateEnteredAtMs(machine) == 100U,
               "transitionToFor should update internal entered-at")) return false;
  if (!require(TimedStateMachineTestProbe::hasDeadline(machine),
               "transitionToFor should set internal deadline flag")) return false;
  if (!require(TimedStateMachineTestProbe::deadlineAtMs(machine) == 107U,
               "transitionToFor should set internal deadline after transition")) return false;

  return true;
}

int main() {
  if (!test_WB_namedConstructorStoresClockAndOptionalNames()) return 1;
  if (!test_WB_setDeadlineFromNowWritesInternalDeadlineState()) return 1;
  if (!test_WB_clearDeadlineZerosInternalDeadlineState()) return 1;
  if (!test_WB_transitionToMutatesInternalStateAndClearsInternalDeadline()) return 1;
  if (!test_WB_transitionToForUsesTransitionThenSetsInternalDeadline()) return 1;

  printf("PASS: test_WB_timed_state_machine\n");
  return 0;
}
// host_tests/test_WB_timed_state_machine.cpp v1