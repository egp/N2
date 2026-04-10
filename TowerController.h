// TowerController.h v2
#ifndef TOWER_CONTROLLER_H
#define TOWER_CONTROLLER_H

#include <stdint.h>

#include "TimedStateMachine.h"

class ITowerValveDriver {
public:
  virtual ~ITowerValveDriver() {}
  virtual void setLeftOpen(bool open) = 0;
  virtual void setRightOpen(bool open) = 0;
};

class TowerController {
public:
  struct Config {
    uint32_t leftOpenMs;
    uint32_t overlapMs;
    uint32_t rightOpenMs;
  };

  enum State : uint8_t {
    STATE_INACTIVE = 0,
    STATE_LEFT_ONLY,
    STATE_BOTH_AFTER_LEFT,
    STATE_RIGHT_ONLY,
    STATE_BOTH_AFTER_RIGHT
  };

  static Config defaultConfig();

  TowerController(IClock& clock, ITowerValveDriver& valveDriver);
  TowerController(IClock& clock, ITowerValveDriver& valveDriver, const Config& config);

  void setEnabled(bool enabled);
  bool isEnabled() const;

  void tick();

  State state() const;
  bool isActive() const;

  const Config& config() const;
  void setConfig(const Config& config);

private:
  void transitionTo(State nextState, uint32_t durationMs, bool timed);
  void applyOutputsForState(State state);

  TimedStateMachine timedStateMachine_;
  ITowerValveDriver& valveDriver_;
  Config config_;
  bool enabled_;
};

#endif
// TowerController.h v2