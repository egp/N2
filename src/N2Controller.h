// N2Controller.h v7
#ifndef N2_CONTROLLER_H
#define N2_CONTROLLER_H

#include <stdint.h>

#include "BinaryOutput.h"
#include "IController.h"
#include "InputSnapshot.h"
#include "SystemConfig.h"
#include "TimedStateMachine.h"

class N2Controller : public IController {

public:

 using Config = SystemConfig::N2Config;

 enum State : uint8_t {
  STATE_LOW_INHIBIT_HIGH_PERMIT = 0,
  STATE_LOW_PERMIT_HIGH_PERMIT,
  STATE_LOW_PERMIT_HIGH_INHIBIT,
  STATE_LOW_INHIBIT_HIGH_INHIBIT
 };

 struct Snapshot {
  uint32_t createdAtMs;
  State state;
 };

 class StateView : public ControllerState {
 public:
  explicit StateView(const N2Controller& owner);

  ControllerKind kind() const override;
  uint32_t enteredAtMs() const override;
  uint32_t code() const override;
  const char* name() const override;

 private:
  const N2Controller& owner_;
 };

 static Config defaultConfig();

 N2Controller(IClock& clock, IBinaryOutput& compressorOutput);
 N2Controller(IClock& clock, IBinaryOutput& compressorOutput, const SystemConfig& systemConfig);
 N2Controller(IClock& clock, IBinaryOutput& compressorOutput, const Config& config);

 bool init() override;
 void setEnabled(bool enabled) override;
 void step(const InputSnapshot& inputs) override;
 void shutdown() override;
 IClock& clock() const override;
 const ControllerState& getState() const override;

 State state() const;

 Snapshot snapshot() const;

 const Config& config() const;

 void setConfig(const Config& config);

 bool isCompressorOn() const;
 bool isOk() const;

 friend struct N2ControllerTestProbe;

private:

 void transitionTo(State nextState);

 void applyOutputForState(State state);

 IClock& clock_;
 TimedStateMachine timedStateMachine_;
 IBinaryOutput& compressorOutput_;
 Config config_;
 StateView stateView_;

};

#endif

// N2Controller.h v7