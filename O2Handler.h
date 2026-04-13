// O2Handler.h v2
#ifndef O2_HANDLER_H
#define O2_HANDLER_H

#include <stdint.h>

class IClock {
public:
  virtual ~IClock() {}
  virtual uint32_t nowMs() const = 0;
};

class TimedStateMachine {
public:
  TimedStateMachine(IClock& clock, uint8_t initialState);
  uint8_t state() const;
  bool isExpired() const;
  void transitionTo(uint8_t nextState);
  void transitionToFor(uint8_t nextState, uint32_t durationMs);

private:
  IClock& clock_;
  uint8_t state_;
  uint32_t enteredAtMs_;
  uint32_t durationMs_;
};

class IO2Sensor {
public:
  virtual ~IO2Sensor() {}
  virtual bool begin() = 0;
  virtual bool readOxygenPercent(float& percentVol) = 0;
  virtual const char* errorString() const = 0;
};

class IBinaryOutput {
public:
  virtual ~IBinaryOutput() {}
  virtual void setOn(bool on) = 0;
};

class O2Handler {
public:
  struct Config {
    uint32_t warmupDurationMs;
    uint32_t measurementIntervalMs;
    uint32_t flushDurationMs;
    uint32_t settleDurationMs;
    uint16_t sampleIntervalMs;
    uint8_t sampleCount;
    uint32_t freshnessThresholdMs;
    uint32_t errorBackoffMs;
  };

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

  static Config defaultConfig();

  O2Handler(IClock& clock, IO2Sensor& sensor, IBinaryOutput& flushValve);
  O2Handler(IClock& clock, IO2Sensor& sensor, IBinaryOutput& flushValve, const Config& config);

  bool begin();
  void tick();
  void requestMeasurementIfStale();

  bool isWarmingUp() const;
  bool isBusy() const;
  bool hasValue() const;
  bool isValueFresh() const;
  float averagedPercent() const;
  const char* errorString() const;
  State state() const;
  const Config& config() const;
  void setConfig(const Config& config);

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
};

#endif
// O2Handler.h v2