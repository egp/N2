// TimedStateMachine.cpp v2
#include "TimedStateMachine.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

TimedStateMachine::TimedStateMachine(
    IClock& clock,
    uint8_t initialState,
    const char* controllerName,
    StateNameFn stateNameFn)
    : clock_(clock),
      state_(initialState),
      stateEnteredAtMs_(clock.nowMs()),
      deadlineAtMs_(0U),
      hasDeadline_(false),
      controllerName_(controllerName),
      stateNameFn_(stateNameFn) {}

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
  const uint8_t oldState = state_;
  const uint32_t nowMs = clock_.nowMs();

#if defined(ARDUINO)
  if (newState != oldState && controllerName_ != nullptr && stateNameFn_ != nullptr) {
    Serial.print(controllerName_);
    Serial.print(F(" transition @"));
    Serial.print(nowMs);
    Serial.print(F(" from "));
    Serial.print(stateNameFn_(oldState));
    Serial.print(F(" to "));
    Serial.println(stateNameFn_(newState));
  }
#endif

  state_ = newState;
  stateEnteredAtMs_ = nowMs;
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
// TimedStateMachine.cpp v2