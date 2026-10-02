// N2Controller.h v8
#ifndef N2_CONTROLLER_H
#define N2_CONTROLLER_H
#include <stdint.h>
#include "BinaryOutput.h"
#include "InputSnapshot.h"
#include "SystemConfig.h"
#include "TimedStateMachine.h"

class N2Controller {
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

  static Config defaultConfig();

  N2Controller(IClock& clock, IBinaryOutput& compressorOutput);
  N2Controller(IClock& clock, IBinaryOutput& compressorOutput,
               const SystemConfig& systemConfig);
  N2Controller(IClock& clock, IBinaryOutput& compressorOutput, const Config& config);

  bool init();
  void step(const InputSnapshot& inputs);
  void shutdown();

  State state() const;
  Snapshot snapshot() const;
  bool isCompressorOn() const;
  bool isOk() const;

  const Config& config() const;
  void setConfig(const Config& config);

  friend struct N2ControllerTestProbe;

private:
  void transitionTo(State nextState);
  void applyOutputForState(State state);

  IClock& clock_;
  TimedStateMachine timedStateMachine_;
  IBinaryOutput& compressorOutput_;
  Config config_;
};
#endif
// N2Controller.h v8
