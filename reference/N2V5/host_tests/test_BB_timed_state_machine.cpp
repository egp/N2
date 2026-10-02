// host_tests/test_BB_timed_state_machine.cpp v2
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
  if (!require(machine.deadlineAtMs() == 0U, "constructor should seed zero deadline")) return false;
  return true;
}

static bool test_BB_timeInStateAdvancesAndResetsOnTransition() {
  FakeClock clock;
  clock.setNowMs(50U);

  TimedStateMachine machine(clock, 1U);

  if (!require(machine.timeInStateMs() == 0U, "timeInState should start at zero")) return false;

  clock.advanceMs(25U);
  if (!require(machine.timeInStateMs() == 25U, "timeInState should advance with clock")) return false;

  machine.transitionTo(2U);
  if (!require(machine.state() == 2U, "transitionTo should update state")) return false;
  if (!require(machine.stateEnteredAtMs() == 75U, "transitionTo should reset entered-at time")) return false;
  if (!require(machine.timeInStateMs() == 0U, "transitionTo should reset timeInState")) return false;

  clock.advanceMs(10U);
  if (!require(machine.timeInStateMs() == 10U, "timeInState should restart after transition")) return false;

  return true;
}

static bool test_BB_timedTransitionExpiresOnlyAtDeadlineBoundary() {
  FakeClock clock;
  TimedStateMachine machine(clock, 0U);

  machine.transitionToFor(3U, 60000U);

  if (!require(machine.state() == 3U, "transitionToFor should update state")) return false;
  if (!require(machine.hasDeadline(), "transitionToFor should set deadline")) return false;
  if (!require(machine.deadlineAtMs() == 60000U, "transitionToFor should set deadline from now")) return false;
  if (!require(!machine.isExpired(), "state should not be expired immediately")) return false;

  clock.advanceMs(59999U);
  if (!require(!machine.isExpired(), "state should not be expired before deadline")) return false;

  clock.advanceMs(1U);
  if (!require(machine.isExpired(), "state should be expired exactly at deadline")) return false;

  return true;
}

static bool test_BB_zeroDurationTimedTransitionIsImmediatelyExpired() {
  FakeClock clock;
  clock.setNowMs(99U);

  TimedStateMachine machine(clock, 4U);
  machine.transitionToFor(5U, 0U);

  if (!require(machine.state() == 5U, "zero-duration timed transition should update state")) return false;
  if (!require(machine.hasDeadline(), "zero-duration timed transition should still set deadline")) return false;
  if (!require(machine.deadlineAtMs() == 99U, "zero-duration deadline should equal now")) return false;
  if (!require(machine.isExpired(), "zero-duration timed transition should be immediately expired")) return false;

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
  if (!require(machine.deadlineAtMs() == 0U, "transitionTo should zero deadline")) return false;
  if (!require(machine.stateEnteredAtMs() == 25U, "transitionTo should reset entered-at time")) return false;

  return true;
}

static bool test_BB_sameStateTransitionResetsEnteredAtAndClearsDeadline() {
  FakeClock clock;
  TimedStateMachine machine(clock, 6U);

  machine.transitionToFor(6U, 100U);
  clock.advanceMs(40U);
  machine.transitionTo(6U);

  if (!require(machine.state() == 6U, "same-state transitionTo should keep state")) return false;
  if (!require(machine.stateEnteredAtMs() == 40U, "same-state transitionTo should refresh entered-at time")) return false;
  if (!require(machine.timeInStateMs() == 0U, "same-state transitionTo should reset timeInState")) return false;
  if (!require(!machine.hasDeadline(), "same-state transitionTo should clear deadline")) return false;
  if (!require(machine.deadlineAtMs() == 0U, "same-state transitionTo should zero deadline")) return false;

  return true;
}

static bool test_BB_sameStateTimedTransitionRefreshesDeadlineAndEnteredAt() {
  FakeClock clock;
  TimedStateMachine machine(clock, 8U);

  machine.transitionToFor(8U, 10U);
  clock.advanceMs(7U);
  machine.transitionToFor(8U, 20U);

  if (!require(machine.state() == 8U, "same-state transitionToFor should keep state")) return false;
  if (!require(machine.stateEnteredAtMs() == 7U, "same-state transitionToFor should refresh entered-at time")) return false;
  if (!require(machine.hasDeadline(), "same-state transitionToFor should set deadline")) return false;
  if (!require(machine.deadlineAtMs() == 27U, "same-state transitionToFor should refresh deadline from new now")) return false;
  if (!require(!machine.isExpired(), "refreshed deadline should not be expired immediately")) return false;

  clock.advanceMs(19U);
  if (!require(!machine.isExpired(), "refreshed deadline should hold before new boundary")) return false;

  clock.advanceMs(1U);
  if (!require(machine.isExpired(), "refreshed deadline should expire at new boundary")) return false;

  return true;
}

static bool test_BB_clearDeadlineRemovesExpiryState() {
  FakeClock clock;
  TimedStateMachine machine(clock, 2U);

  machine.transitionToFor(9U, 5U);
  clock.advanceMs(5U);

  if (!require(machine.isExpired(), "precondition: timed state should be expired")) return false;

  machine.clearDeadline();

  if (!require(!machine.hasDeadline(), "clearDeadline should clear deadline flag")) return false;
  if (!require(machine.deadlineAtMs() == 0U, "clearDeadline should zero deadline")) return false;
  if (!require(!machine.isExpired(), "clearDeadline should make state non-expired")) return false;

  return true;
}

int main() {
  if (!test_BB_constructorSeedsInitialStateAndNoDeadline()) return 1;
  if (!test_BB_timeInStateAdvancesAndResetsOnTransition()) return 1;
  if (!test_BB_timedTransitionExpiresOnlyAtDeadlineBoundary()) return 1;
  if (!test_BB_zeroDurationTimedTransitionIsImmediatelyExpired()) return 1;
  if (!test_BB_plainTransitionClearsDeadline()) return 1;
  if (!test_BB_sameStateTransitionResetsEnteredAtAndClearsDeadline()) return 1;
  if (!test_BB_sameStateTimedTransitionRefreshesDeadlineAndEnteredAt()) return 1;
  if (!test_BB_clearDeadlineRemovesExpiryState()) return 1;

  printf("PASS: test_BB_timed_state_machine\n");
  return 0;
}
// host_tests/test_BB_timed_state_machine.cpp v2