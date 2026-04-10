// O2Handler.cpp v1
#include "O2Handler.h"

O2Handler::O2Handler(ValveControlFn valveControlFn, const Config& config)
    : sensor_(),
      valveControlFn_(valveControlFn),
      config_(config),
      state_(STATE_UNINITIALIZED),
      stateEnteredAtMs_(0UL),
      nextActionAtMs_(0UL),
      lastCompletedMeasurementAtMs_(0UL),
      hasCompletedMeasurement_(false),
      earlyMeasurementRequested_(false),
      samplesCollected_(0U),
      runningSumPercent_(0.0f),
      cachedAveragePercent_(0.0f),
      lastError_("not initialized") {}

bool O2Handler::begin() {
  if (valveControlFn_ == nullptr) {
    lastError_ = "null valve callback";
    state_ = STATE_UNINITIALIZED;
    return false;
  }

  valveControlFn_(false);

  if (!sensor_.begin()) {
    lastError_ = sensor_.errorString();
    state_ = STATE_UNINITIALIZED;
    return false;
  }

  const uint32_t now = nowMs();

  hasCompletedMeasurement_ = false;
  earlyMeasurementRequested_ = false;
  samplesCollected_ = 0U;
  runningSumPercent_ = 0.0f;
  cachedAveragePercent_ = 0.0f;
  lastError_ = "no error";

  transitionTo(STATE_WARMUP, now);
  nextActionAtMs_ = now + config_.warmupDurationMs;
  return true;
}

void O2Handler::tick() {
  const uint32_t now = nowMs();

  switch (state_) {
    case STATE_UNINITIALIZED:
      return;

    case STATE_WARMUP:
      if (now >= nextActionAtMs_) {
        transitionTo(STATE_WAITING_TO_FLUSH, now);
      }
      return;

    case STATE_WAITING_TO_FLUSH:
      if (earlyMeasurementRequested_ || shouldStartScheduledCycle(now)) {
        beginMeasurementCycle(now);
      }
      return;

    case STATE_FLUSHING:
      if (now >= nextActionAtMs_) {
        valveControlFn_(false);
        transitionTo(STATE_SETTLING, now);
        nextActionAtMs_ = now + config_.settleDurationMs;
      }
      return;

    case STATE_SETTLING:
      if (now >= nextActionAtMs_) {
        samplesCollected_ = 0U;
        runningSumPercent_ = 0.0f;
        transitionTo(STATE_SAMPLING, now);
      }
      return;

    case STATE_SAMPLING: {
      float percentVol = 0.0f;
      if (!sampleSensor(percentVol)) {
        failMeasurementCycle(now, sensor_.errorString());
        return;
      }

      runningSumPercent_ += percentVol;
      ++samplesCollected_;

      if (samplesCollected_ >= config_.sampleCount) {
        finishMeasurementCycle(now, runningSumPercent_ / static_cast<float>(samplesCollected_));
        return;
      }

      transitionTo(STATE_WAITING_FOR_NEXT_SAMPLE, now);
      nextActionAtMs_ = now + static_cast<uint32_t>(config_.sampleIntervalMs);
      return;
    }

    case STATE_WAITING_FOR_NEXT_SAMPLE:
      if (now >= nextActionAtMs_) {
        transitionTo(STATE_SAMPLING, now);
      }
      return;

    case STATE_ERROR_BACKOFF:
      if (now >= nextActionAtMs_) {
        transitionTo(STATE_WAITING_TO_FLUSH, now);
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
  return state_ == STATE_WARMUP;
}

bool O2Handler::isBusy() const {
  return state_ == STATE_FLUSHING ||
         state_ == STATE_SETTLING ||
         state_ == STATE_SAMPLING ||
         state_ == STATE_WAITING_FOR_NEXT_SAMPLE;
}

bool O2Handler::hasValue() const {
  return hasCompletedMeasurement_;
}

bool O2Handler::isValueFresh() const {
  if (!hasCompletedMeasurement_) {
    return false;
  }

  return (nowMs() - lastCompletedMeasurementAtMs_) <= config_.freshnessThresholdMs;
}

float O2Handler::averagedPercent() const {
  return cachedAveragePercent_;
}

const char* O2Handler::errorString() const {
  return lastError_;
}

void O2Handler::setConfig(const Config& config) {
  config_ = config;
}

const O2Handler::Config& O2Handler::config() const {
  return config_;
}

void O2Handler::transitionTo(State nextState, uint32_t nowMs) {
  state_ = nextState;
  stateEnteredAtMs_ = nowMs;
}

void O2Handler::beginMeasurementCycle(uint32_t nowMs) {
  earlyMeasurementRequested_ = false;
  samplesCollected_ = 0U;
  runningSumPercent_ = 0.0f;

  valveControlFn_(true);
  transitionTo(STATE_FLUSHING, nowMs);
  nextActionAtMs_ = nowMs + config_.flushDurationMs;
}

void O2Handler::finishMeasurementCycle(uint32_t nowMs, float averagedPercent) {
  cachedAveragePercent_ = averagedPercent;
  hasCompletedMeasurement_ = true;
  lastCompletedMeasurementAtMs_ = nowMs;
  lastError_ = "no error";
  transitionTo(STATE_WAITING_TO_FLUSH, nowMs);
}

void O2Handler::failMeasurementCycle(uint32_t nowMs, const char* error) {
  valveControlFn_(false);
  lastError_ = error;
  transitionTo(STATE_ERROR_BACKOFF, nowMs);
  nextActionAtMs_ = nowMs + config_.errorBackoffMs;
}

uint32_t O2Handler::nowMs() const {
  return millis();
}

bool O2Handler::sampleSensor(float& percentVol) {
  return sensor_.readOxygenPercent(percentVol);
}

bool O2Handler::shouldStartScheduledCycle(uint32_t nowMs) const {
  if (!hasCompletedMeasurement_) {
    return true;
  }

  return (nowMs - lastCompletedMeasurementAtMs_) >= config_.measurementIntervalMs;
}
// O2Handler.cpp v1