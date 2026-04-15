// TowerController.h v4
#ifndef TOWER_CONTROLLER_H
#define TOWER_CONTROLLER_H

#include <stdint.h>

#include "BinaryOutput.h"
#include "TimedStateMachine.h"

class TowerController {

public:

 struct Config {
  uint32_t leftOpenMs;
  uint32_t overlapMs;
  uint32_t rightOpenMs;
  uint16_t lowSupplyPsi_x10;
 };

 enum State : uint8_t {
  STATE_INACTIVE = 0,
  STATE_LEFT_ONLY,
  STATE_BOTH_AFTER_LEFT,
  STATE_RIGHT_ONLY,
  STATE_BOTH_AFTER_RIGHT,
  STATE_LOW_SUPPLY
 };

 static Config defaultConfig();

 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve);
 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve, const Config& config);

 void setEnabled(bool enabled);

 bool isEnabled() const;

 void tick(uint16_t supplyPsi_x10);

 State state() const;

 bool isActive() const;

 const Config& config() const;

 void setConfig(const Config& config);

private:

 bool isSupplySufficient(uint16_t supplyPsi_x10) const;

 void transitionTo(State nextState, uint32_t durationMs, bool timed);

 void applyOutputsForState(State state);

 TimedStateMachine timedStateMachine_;

 IBinaryOutput& leftValve_;
 IBinaryOutput& rightValve_;

 Config config_;

 bool enabled_;

};

#endif

// TowerController.h v4