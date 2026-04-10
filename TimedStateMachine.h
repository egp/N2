// TimedStateMachine.h v1
#ifndef TIMED_STATE_MACHINE_H
#define TIMED_STATE_MACHINE_H

#include <stdint.h>

class IClock {
public:
  virtual ~IClock() {}
  virtual uint32_t nowMs() const = 0;
};

class TimedStateMachine {
public:
  explicit TimedStateMachine(IClock& clock, uint8_t initialState = 0);

  uint8_t state() const;
  uint32_t stateEnteredAtMs() const;
  uint32_t deadlineAtMs() const;
  bool hasDeadline() const;
  bool isExpired() const;
  uint32_t timeInStateMs() const;

  void transitionTo(uint8_t newState);
  void transitionToFor(uint8_t newState, uint32_t durationMs);
  void setDeadlineFromNow(uint32_t durationMs);
  void clearDeadline();

private:
  IClock& clock_;
  uint8_t state_;
  uint32_t stateEnteredAtMs_;
  uint32_t deadlineAtMs_;
  bool hasDeadline_;
};

#endif
// TimedStateMachine.h v1