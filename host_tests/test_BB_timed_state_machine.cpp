// host_tests/test_BB_timed_state_machine.cpp v1
#include <stdio.h>

#include "TimedStateMachine.h"

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

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static bool test_BB_constructorSeedsInitialStateAndNoDeadline() {
  FakeClock clock;
  clock.setNowMs(1234U);

  TimedStateMachine machine(clock, 7U);

  if (!require(machine.state() == 7U, "constructor should seed initial state")) return false;
  if (!require(machine.stateEnteredAtMs() == 1234U, "constructor should seed entered-at time")) return false;
  if (!require(!machine.hasDeadline(), "constructor should start without deadline")) return false;

  return true;
}

static bool test_BB_timedTransitionExpiresOnlyAfterDeadline() {
  FakeClock clock;
  TimedStateMachine machine(clock, 0U);

  machine.transitionToFor(3U, 60000U);

  if (!require(machine.state() == 3U, "transitionToFor should update state")) return false;
  if (!require(machine.hasDeadline(), "transitionToFor should set deadline")) return false;
  if (!require(!machine.isExpired(), "state should not be expired immediately")) return false;

  clock.advanceMs(59999U);
  if (!require(!machine.isExpired(), "state should not be expired before deadline")) return false;

  clock.advanceMs(1U);
  if (!require(machine.isExpired(), "state should be expired at deadline")) return false;

  return true;
}

static bool test_BB_plainTransitionClearsDeadline() {
  FakeClock clock;
  TimedStateMachine machine(clock, 1U);

  machine.transitionToFor(2U, 100U);
  clock.advanceMs(25U);
  machine.transitionTo(5U);

  if (!require(machine.state() == 5U, "transitionTo should update state")) return false;
  if (!require(!machine.hasDeadline(), "transitionTo should clear deadline")) return false;
  if (!require(machine.stateEnteredAtMs() == 25U, "transitionTo should reset entered-at time")) return false;

  return true;
}

int main() {
  if (!test_BB_constructorSeedsInitialStateAndNoDeadline()) return 1;
  if (!test_BB_timedTransitionExpiresOnlyAfterDeadline()) return 1;
  if (!test_BB_plainTransitionClearsDeadline()) return 1;

  printf("PASS: test_BB_timed_state_machine\n");
  return 0;
}
// host_tests/test_BB_timed_state_machine.cpp v1