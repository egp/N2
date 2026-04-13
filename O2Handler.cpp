// O2Handler.cpp v1
#include "O2Handler.h"
#include <Arduino.h>
O2Handler::Config O2Handler::defaultConfig() {
  Config config;
  config.warmupDurationMs = 300000UL;
  config.measurementIntervalMs = 60000UL;
  config.flushDurationMs = 3000UL;
  config.settleDurationMs = 1000UL;
  config.sampleIntervalMs = 250U;
  config.sampleCount = 10U;
  config.freshnessThresholdMs = 15000UL;
  config.errorBackoffMs = 1000UL;
  return config;
}

O2Handler::O2Handler(IClock& clock, IO2Sensor& sensor, IFlushValveDriver& flushValveDriver)
    : clock_(clock),
      sensor_(sensor),
      flushValveDriver_(flushValveDriver),
      timedStateMachine_(clock, STATE_UNINITIALIZED),
      config_(defaultConfig()),
      hasValue_(false),
      earlyMeasurementRequested_(false),
      lastCompletedMeasurementAtMs_(0U),
      cachedAveragePercent_(0.0f),
      runningSumPercent_(0.0f),
      samplesCollected_(0U),
      lastError_("not initialized") {}

O2Handler::O2Handler(IClock& clock, IO2Sensor& sensor, IFlushValveDriver& flushValveDriver, const Config& config)
    : clock_(clock),
      sensor_(sensor),
      flushValveDriver_(flushValveDriver),
      timedStateMachine_(clock, STATE_UNINITIALIZED),
      config_(config),
      hasValue_(false),
      earlyMeasurementRequested_(false),
      lastCompletedMeasurementAtMs_(0U),
      cachedAveragePercent_(0.0f),
      runningSumPercent_(0.0f),
      samplesCollected_(0U),
      lastError_("not initialized") {}

bool O2Handler::begin() {
  flushValveDriver_.setFlushValveOpen(false);

  if (config_.sampleCount == 0U) {
    lastError_ = "sampleCount must be > 0";
    transitionTo(STATE_UNINITIALIZED);
    return false;
  }

  if (!sensor_.begin()) {
    lastError_ = sensor_.errorString();
    transitionTo(STATE_UNINITIALIZED);
    return false;
  }

  hasValue_ = false;
  earlyMeasurementRequested_ = false;
  lastCompletedMeasurementAtMs_ = 0U;
  cachedAveragePercent_ = 0.0f;
  runningSumPercent_ = 0.0f;
  samplesCollected_ = 0U;
  lastError_ = "no error";

  transitionToFor(STATE_WARMUP, config_.warmupDurationMs);
  return true;
}

void O2Handler::tick() {
  switch (state()) {
    case STATE_UNINITIALIZED:
      return;

    case STATE_WARMUP:
      if (timedStateMachine_.isExpired()) {
        Serial.println("O2 Transitioning from WARMUP to WAITING_TO_FLUSH");
        transitionTo(STATE_WAITING_TO_FLUSH);
      }
      return;

    case STATE_WAITING_TO_FLUSH:
      if (earlyMeasurementRequested_ || shouldStartScheduledCycle(clock_.nowMs())) {
        Serial.println("O2 Transitioning from WAITING_TO_FLUSH to Beginning Measurement Cycle");
        beginMeasurementCycle();
      }
      return;

    case STATE_FLUSHING:
      if (timedStateMachine_.isExpired()) {
        Serial.println("O2 Transitioning from FLUSHING to SETTLING");
        flushValveDriver_.setFlushValveOpen(false);
        transitionToFor(STATE_SETTLING, config_.settleDurationMs);
      }
      return;

    case STATE_SETTLING:
      if (timedStateMachine_.isExpired()) {
        Serial.println("O2 Transitioning from SETTLING to SAMPLING");
        runningSumPercent_ = 0.0f;
        samplesCollected_ = 0U;
        transitionTo(STATE_SAMPLING);
      }
      return;

    case STATE_SAMPLING: {
      float percentVol = 0.0f;
      if (!sensor_.readOxygenPercent(percentVol)) {
        failMeasurementCycle(sensor_.errorString());
        return;
      }

      runningSumPercent_ += percentVol;
      ++samplesCollected_;

      if (samplesCollected_ >= config_.sampleCount) {
        Serial.println("O2 Transitioning from SAMPLING to WAITING_FOR_NEXT_SAMPLE");
        finishMeasurementCycle(runningSumPercent_ / static_cast<float>(samplesCollected_));
        return;
      }

      transitionToFor(STATE_WAITING_FOR_NEXT_SAMPLE, static_cast<uint32_t>(config_.sampleIntervalMs));
      return;
    }

    case STATE_WAITING_FOR_NEXT_SAMPLE:
      if (timedStateMachine_.isExpired()) {
        Serial.println("O2 Transitioning from WAITING_FOR_NEXT_SAMPLE to SAMPLING");
        transitionTo(STATE_SAMPLING);
      }
      return;

    case STATE_ERROR_BACKOFF:
      if (timedStateMachine_.isExpired()) {
        Serial.println("O2 Transitioning from ERROR_BACKOFF to WAITING_TO_FLUSH");
        transitionTo(STATE_WAITING_TO_FLUSH);
      }
      return;

    default:
      return;
  }
}

void O2Handler::requestMeasurementIfStale() {
  if (!isValueFresh()) {
    earlyMeasurementRequested_ = true;
  }
}

bool O2Handler::isWarmingUp() const {
  return state() == STATE_WARMUP;
}

bool O2Handler::isBusy() const {
  return state() == STATE_FLUSHING ||
         state() == STATE_SETTLING ||
         state() == STATE_SAMPLING ||
         state() == STATE_WAITING_FOR_NEXT_SAMPLE;
}

bool O2Handler::hasValue() const {
  return hasValue_;
}

bool O2Handler::isValueFresh() const {
  if (!hasValue_) {
    return false;
  }

  return (clock_.nowMs() - lastCompletedMeasurementAtMs_) <= config_.freshnessThresholdMs;
}

float O2Handler::averagedPercent() const {
  return cachedAveragePercent_;
}

const char* O2Handler::errorString() const {
  return lastError_;
}

O2Handler::State O2Handler::state() const {
  return static_cast<State>(timedStateMachine_.state());
}

const O2Handler::Config& O2Handler::config() const {
  return config_;
}

void O2Handler::setConfig(const Config& config) {
  config_ = config;
}

bool O2Handler::shouldStartScheduledCycle(uint32_t nowMs) const {
  if (!hasValue_) {
    return true;
  }

  return (nowMs - lastCompletedMeasurementAtMs_) >= config_.measurementIntervalMs;
}

void O2Handler::beginMeasurementCycle() {
  earlyMeasurementRequested_ = false;
  runningSumPercent_ = 0.0f;
  samplesCollected_ = 0U;
  flushValveDriver_.setFlushValveOpen(true);
  transitionToFor(STATE_FLUSHING, config_.flushDurationMs);
}

void O2Handler::finishMeasurementCycle(float averagedPercent) {
  flushValveDriver_.setFlushValveOpen(false);
  cachedAveragePercent_ = averagedPercent;
  hasValue_ = true;
  lastCompletedMeasurementAtMs_ = clock_.nowMs();
  lastError_ = "no error";
  transitionTo(STATE_WAITING_TO_FLUSH);
}

void O2Handler::failMeasurementCycle(const char* error) {
  flushValveDriver_.setFlushValveOpen(false);
  lastError_ = error;
  transitionToFor(STATE_ERROR_BACKOFF, config_.errorBackoffMs);
}

void O2Handler::transitionTo(State nextState) {
  timedStateMachine_.transitionTo(static_cast<uint8_t>(nextState));
}

void O2Handler::transitionToFor(State nextState, uint32_t durationMs) {
  timedStateMachine_.transitionToFor(static_cast<uint8_t>(nextState), durationMs);
}
// O2Handler.cpp v1