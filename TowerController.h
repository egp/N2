// TowerController.h v8
#ifndef TOWER_CONTROLLER_H
#define TOWER_CONTROLLER_H

#include <stdint.h>

#include "BinaryOutput.h"
#include "IController.h"
#include "InputSnapshot.h"
#include "SystemConfig.h"
#include "TimedStateMachine.h"

class TowerController : public IController {

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

 class StateView : public ControllerState {
 public:
  explicit StateView(const TowerController& owner);

  ControllerKind kind() const override;
  uint32_t enteredAtMs() const override;
  uint32_t code() const override;
  const char* name() const override;

 private:
  const TowerController& owner_;
 };

 static Config defaultConfig();

 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve);
 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve, const Config& config);
 TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve, const SystemConfig& systemConfig);

 bool init() override;
 void setEnabled(bool enabled) override;
 void step(const InputSnapshot& inputs) override;
 void shutdown() override;
 IClock& clock() const override;
 const ControllerState& getState() const override;

 bool isEnabled() const;

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
 StateView stateView_;

};

#endif

// TowerController.h v8