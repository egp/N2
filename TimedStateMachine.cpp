// TimedStateMachine.cpp v1
#include "TimedStateMachine.h"

TimedStateMachine::TimedStateMachine(IClock& clock, uint8_t initialState)
    : clock_(clock),
      state_(initialState),
      stateEnteredAtMs_(clock.nowMs()),
      deadlineAtMs_(0U),
      hasDeadline_(false) {}

uint8_t TimedStateMachine::state() const {
  return state_;
}

uint32_t TimedStateMachine::stateEnteredAtMs() const {
  return stateEnteredAtMs_;
}

uint32_t TimedStateMachine::deadlineAtMs() const {
  return deadlineAtMs_;
}

bool TimedStateMachine::hasDeadline() const {
  return hasDeadline_;
}

bool TimedStateMachine::isExpired() const {
  return hasDeadline_ && clock_.nowMs() >= deadlineAtMs_;
}

uint32_t TimedStateMachine::timeInStateMs() const {
  return clock_.nowMs() - stateEnteredAtMs_;
}

void TimedStateMachine::transitionTo(uint8_t newState) {
  state_ = newState;
  stateEnteredAtMs_ = clock_.nowMs();
  hasDeadline_ = false;
  deadlineAtMs_ = 0U;
}

void TimedStateMachine::transitionToFor(uint8_t newState, uint32_t durationMs) {
  transitionTo(newState);
  setDeadlineFromNow(durationMs);
}

void TimedStateMachine::setDeadlineFromNow(uint32_t durationMs) {
  deadlineAtMs_ = clock_.nowMs() + durationMs;
  hasDeadline_ = true;
}

void TimedStateMachine::clearDeadline() {
  deadlineAtMs_ = 0U;
  hasDeadline_ = false;
}
// TimedStateMachine.cpp v1