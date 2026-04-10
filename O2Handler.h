// O2Handler.h v1
#ifndef O2_HANDLER_H
#define O2_HANDLER_H

#include <Arduino.h>
#include <TCP0465.h>

class O2Handler {
public:
  typedef void (*ValveControlFn)(bool open);

  struct Config {
    uint32_t warmupDurationMs = 300000UL;      // 5 minutes
    uint32_t measurementIntervalMs = 60000UL;  // once per minute
    uint32_t flushDurationMs = 3000UL;         // valve open time
    uint32_t settleDurationMs = 1000UL;        // valve closed before sampling
    uint16_t sampleIntervalMs = 250U;          // time between samples
    uint8_t sampleCount = 10U;                 // averaged sample count
    uint32_t freshnessThresholdMs = 15000UL;   // "recent enough" for display
    uint32_t errorBackoffMs = 1000UL;          // retry delay after failed cycle
  };

  explicit O2Handler(ValveControlFn valveControlFn, const Config& config = Config());

  bool begin();
  void tick();

  void requestMeasurementIfStale();

  bool isWarmingUp() const;
  bool isBusy() const;
  bool hasValue() const;
  bool isValueFresh() const;

  float averagedPercent() const;
  const char* errorString() const;

  void setConfig(const Config& config);
  const Config& config() const;

private:
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

  void transitionTo(State nextState, uint32_t nowMs);
  void beginMeasurementCycle(uint32_t nowMs);
  void finishMeasurementCycle(uint32_t nowMs, float averagedPercent);
  void failMeasurementCycle(uint32_t nowMs, const char* error);

  uint32_t nowMs() const;
  bool sampleSensor(float& percentVol);
  bool shouldStartScheduledCycle(uint32_t nowMs) const;

  TCP0465 sensor_;
  ValveControlFn valveControlFn_;
  Config config_;

  State state_;
  uint32_t stateEnteredAtMs_;
  uint32_t nextActionAtMs_;
  uint32_t lastCompletedMeasurementAtMs_;
  bool hasCompletedMeasurement_;
  bool earlyMeasurementRequested_;

  uint8_t samplesCollected_;
  float runningSumPercent_;
  float cachedAveragePercent_;
  const char* lastError_;
};

#endif
// O2Handler.h v1