// TowerController.h v7
#ifndef TOWER_CONTROLLER_H
#define TOWER_CONTROLLER_H

#include <stdint.h>

#include "BinaryOutput.h"
#include "InputSnapshot.h"
#include "SystemConfig.h"
#include "TimedStateMachine.h"
#include "SystemConfig.h"

class TowerController {

public:

 using Config = SystemConfig::TowerConfig;

 enum State : uint8_t {
  STATE_INACTIVE = 0,
  STATE_LEFT_ONLY,
  STATE_BOTH_AFTER_LEFT,
  STATE_RIGHT_ONLY,
  STATE_BOTH_AFTER_RIGHT,
  STATE_LOW_SUPPLY
 };

 struct Snapshot {
  uint32_t createdAtMs;
  State state;
 };

 static Config defaultConfig();

 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve);
 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve, const Config& config);
 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve, const SystemConfig& systemConfig);

 bool init();
void step(const InputSnapshot& inputs);
void shutdown();
IClock& clock() const;

 void setEnabled(bool enabled);

 bool isEnabled() const;

 void tick(const InputSnapshot& inputs);

 State state() const;

 bool isActive() const;

 Snapshot snapshot() const;

 const Config& config() const;

 void setConfig(const Config& config);

 friend struct TowerControllerTestProbe;

private:

 bool isSupplySufficient(uint16_t supplyPsi_x10) const;

 void transitionTo(State nextState, uint32_t durationMs, bool timed);

 void applyOutputsForState(State state);

 IClock& clock_;
 TimedStateMachine timedStateMachine_;
 IBinaryOutput& leftValve_;
 IBinaryOutput& rightValve_;
 Config config_;
 bool enabled_;

};

#endif

// TowerController.h v7