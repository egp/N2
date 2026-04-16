// O2Controller.h v5
#ifndef O2_CONTROLLER_H
#define O2_CONTROLLER_H

#include <stdint.h>

#include "BinaryOutput.h"
#include "IController.h"
#include "InputSnapshot.h"
#include "SystemConfig.h"
#include "TimedStateMachine.h"

class IO2Sensor {
public:
  virtual ~IO2Sensor() {}
  virtual bool begin() = 0;
  virtual bool readOxygenPercent(float& percentVol) = 0;
  virtual const char* errorString() const = 0;
};

class O2Controller : public IController {
public:
  using Config = SystemConfig::O2Config;

  enum State : uint8_t {
    STATE_UNINITIALIZED = 0,
    STATE_WARMUP,
    STATE_WAITING_TO_FLUSH,
    STATE_FLUSHING,
    STATE_SETTLING,
    STATE_SAMPLING,
    STATE_WAITING_FOR_NEXT_SAMPLE,
    STATE_ERROR_BACKOFF
  };

  struct Snapshot {
    uint32_t createdAtMs;
    State state;
    bool hasValue;
    bool isValueFresh;
    float o2Percent;
    float n2Percent;
    const char* errorString;
  };

  class StateView : public ControllerState {
  public:
    explicit StateView(const O2Controller& owner);

    ControllerKind kind() const override;
    uint32_t enteredAtMs() const override;
    uint32_t code() const override;
    const char* name() const override;

  private:
    const O2Controller& owner_;
  };

  static Config defaultConfig();

  O2Controller(IClock& clock, IO2Sensor& sensor, IBinaryOutput& flushValve);
  O2Controller(IClock& clock, IO2Sensor& sensor, IBinaryOutput& flushValve, const SystemConfig& systemConfig);
  O2Controller(IClock& clock, IO2Sensor& sensor, IBinaryOutput& flushValve, const Config& config);

  bool init() override;
  void setEnabled(bool enabled) override;
  void step(const InputSnapshot& inputs) override;
  void shutdown() override;
  IClock& clock() const override;
  const ControllerState& getState() const override;

  void requestMeasurementIfStale();

  bool isWarmingUp() const;
  bool isBusy() const;
  bool hasValue() const;
  bool isValueFresh() const;
  float averagedPercent() const;
  const char* errorString() const;
  State state() const;
  Snapshot snapshot() const;
  const Config& config() const;
  void setConfig(const Config& config);

  friend struct O2ControllerTestProbe;

private:
  bool shouldStartScheduledCycle(uint32_t nowMs) const;
  void beginMeasurementCycle();
  void finishMeasurementCycle(float averagedPercent);
  void failMeasurementCycle(const char* error);
  void transitionTo(State nextState);
  void transitionToFor(State nextState, uint32_t durationMs);

  IClock& clock_;
  IO2Sensor& sensor_;
  IBinaryOutput& flushValve_;
  TimedStateMachine timedStateMachine_;
  Config config_;
  bool hasValue_;
  bool earlyMeasurementRequested_;
  uint32_t lastCompletedMeasurementAtMs_;
  float cachedAveragePercent_;
  float runningSumPercent_;
  uint8_t samplesCollected_;
  const char* lastError_;
  StateView stateView_;
};

#endif
// O2Controller.h v5