// O2Controller.cpp v2
#include "O2Controller.h"

static const char* o2StateName(uint8_t state) {
  switch (static_cast<O2Controller::State>(state)) {
    case O2Controller::STATE_UNINITIALIZED: return "Uninitialized";
    case O2Controller::STATE_WARMUP: return "Warmup";
    case O2Controller::STATE_WAITING_TO_FLUSH: return "WaitingToFlush";
    case O2Controller::STATE_FLUSHING: return "Flushing";
    case O2Controller::STATE_SETTLING: return "Settling";
    case O2Controller::STATE_SAMPLING: return "Sampling";
    case O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE: return "WaitingForNextSample";
    case O2Controller::STATE_ERROR_BACKOFF: return "ErrorBackoff";
    default: return "Unknown";
  }
}

O2Controller::StateView::StateView(const O2Controller& owner)
    : owner_(owner) {}

ControllerKind O2Controller::StateView::kind() const {
  return ControllerKind::O2;
}

uint32_t O2Controller::StateView::enteredAtMs() const {
  return owner_.timedStateMachine_.stateEnteredAtMs();
}

uint32_t O2Controller::StateView::code() const {
  return static_cast<uint32_t>(owner_.state());
}

const char* O2Controller::StateView::name() const {
  switch (owner_.state()) {
    case O2Controller::STATE_UNINITIALIZED: return "Uninitialized";
    case O2Controller::STATE_WARMUP: return "Warmup";
    case O2Controller::STATE_WAITING_TO_FLUSH: return "WaitingToFlush";
    case O2Controller::STATE_FLUSHING: return "Flushing";
    case O2Controller::STATE_SETTLING: return "Settling";
    case O2Controller::STATE_SAMPLING: return "Sampling";
    case O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE: return "WaitingForNextSample";
    case O2Controller::STATE_ERROR_BACKOFF: return "ErrorBackoff";
    default: return "Unknown";
  }
}

O2Controller::Config O2Controller::defaultConfig() {
  Config config;
  config.warmupDurationMs = 300000UL;
  config.measurementIntervalMs = 60000UL;
  config.flushDurationMs = 3000UL;
  config.settleDurationMs = 2000UL;
  config.sampleIntervalMs = 250U;
  config.sampleCount = 10U;
  config.freshnessThresholdMs = 15000UL;
  config.errorBackoffMs = 1000UL;
  return config;
}
O2Controller::O2Controller(IClock& clock, IO2Sensor& sensor, IBinaryOutput& flushValve)
 : O2Controller(clock, sensor, flushValve, defaultConfig()) {
}

O2Controller::O2Controller(
    IClock& clock,
    IO2Sensor& sensor,
    IBinaryOutput& flushValve,
    const SystemConfig& systemConfig)
 : O2Controller(clock, sensor, flushValve, systemConfig.o2) {
}

O2Controller::O2Controller(
    IClock& clock,
    IO2Sensor& sensor,
    IBinaryOutput& flushValve,
    const Config& config)
 : clock_(clock),
   sensor_(sensor),
   flushValve_(flushValve),
   timedStateMachine_(clock, STATE_UNINITIALIZED, "O2", o2StateName),
   config_(config),
   hasValue_(false),
   earlyMeasurementRequested_(false),
   lastCompletedMeasurementAtMs_(0U),
   cachedAveragePercent_(0.0f),
   runningSumPercent_(0.0f),
   samplesCollected_(0U),
   lastError_("no error"),
   stateView_(*this)  {
  flushValve_.setOn(false);
}
const ControllerState& O2Controller::getState() const {
  return stateView_;
}

bool O2Controller::init() {
  flushValve_.setOn(false);

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

void O2Controller::setEnabled(bool enabled) {
  (void)enabled;
}

void O2Controller::step(const InputSnapshot& inputs) {
  (void)inputs;

  switch (state()) {
    case STATE_UNINITIALIZED:
      return;

    case STATE_WARMUP:
      if (timedStateMachine_.isExpired()) {
        transitionTo(STATE_WAITING_TO_FLUSH);
      }
      return;

    case STATE_WAITING_TO_FLUSH:
      if (earlyMeasurementRequested_ || shouldStartScheduledCycle(clock_.nowMs())) {
        beginMeasurementCycle();
      }
      return;

    case STATE_FLUSHING:
      if (timedStateMachine_.isExpired()) {
        flushValve_.setOn(false);
        transitionToFor(STATE_SETTLING, config_.settleDurationMs);
      }
      return;

    case STATE_SETTLING:
      if (timedStateMachine_.isExpired()) {
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
        finishMeasurementCycle(runningSumPercent_ / static_cast<float>(samplesCollected_));
        return;
      }

      transitionToFor(STATE_WAITING_FOR_NEXT_SAMPLE, static_cast<uint32_t>(config_.sampleIntervalMs));
      return;
    }

    case STATE_WAITING_FOR_NEXT_SAMPLE:
      if (timedStateMachine_.isExpired()) {
        transitionTo(STATE_SAMPLING);
      }
      return;

    case STATE_ERROR_BACKOFF:
      if (timedStateMachine_.isExpired()) {
        transitionTo(STATE_WAITING_TO_FLUSH);
      }
      return;

    default:
      return;
  }
}

void O2Controller::shutdown() {
  flushValve_.setOn(false);
  earlyMeasurementRequested_ = false;
  transitionTo(STATE_UNINITIALIZED);
}

IClock& O2Controller::clock() const {
  return clock_;
}

void O2Controller::requestMeasurementIfStale() {
  if (!isValueFresh()) {
    earlyMeasurementRequested_ = true;
  }
}

bool O2Controller::isWarmingUp() const {
  return state() == STATE_WARMUP;
}

bool O2Controller::isBusy() const {
  return state() == STATE_FLUSHING ||
         state() == STATE_SETTLING ||
         state() == STATE_SAMPLING ||
         state() == STATE_WAITING_FOR_NEXT_SAMPLE;
}

bool O2Controller::hasValue() const {
  return hasValue_;
}

bool O2Controller::isValueFresh() const {
  if (!hasValue_) {
    return false;
  }
  return (clock_.nowMs() - lastCompletedMeasurementAtMs_) <= config_.freshnessThresholdMs;
}

float O2Controller::averagedPercent() const {
  return cachedAveragePercent_;
}

const char* O2Controller::errorString() const {
  return lastError_;
}

O2Controller::State O2Controller::state() const {
  return static_cast<State>(timedStateMachine_.state());
}

O2Controller::Snapshot O2Controller::snapshot() const {
  const bool valuePresent = hasValue();
  const float o2Percent = valuePresent ? averagedPercent() : 0.0f;
  const float n2Percent = valuePresent ? (100.0f - o2Percent) : 0.0f;

  return Snapshot{
      timedStateMachine_.stateEnteredAtMs(),
      state(),
      valuePresent,
      isValueFresh(),
      o2Percent,
      n2Percent,
      errorString(),
  };
}

const O2Controller::Config& O2Controller::config() const {
  return config_;
}

void O2Controller::setConfig(const Config& config) {
  config_ = config;
}

bool O2Controller::shouldStartScheduledCycle(uint32_t nowMs) const {
  if (!hasValue_) {
    return true;
  }
  return (nowMs - lastCompletedMeasurementAtMs_) >= config_.measurementIntervalMs;
}

void O2Controller::beginMeasurementCycle() {
  earlyMeasurementRequested_ = false;
  runningSumPercent_ = 0.0f;
  samplesCollected_ = 0U;
  flushValve_.setOn(true);
  transitionToFor(STATE_FLUSHING, config_.flushDurationMs);
}

void O2Controller::finishMeasurementCycle(float averagedPercent) {
  flushValve_.setOn(false);
  cachedAveragePercent_ = averagedPercent;
  hasValue_ = true;
  lastCompletedMeasurementAtMs_ = clock_.nowMs();
  lastError_ = "no error";
  transitionTo(STATE_WAITING_TO_FLUSH);
}

void O2Controller::failMeasurementCycle(const char* error) {
  flushValve_.setOn(false);
  lastError_ = error;
  transitionToFor(STATE_ERROR_BACKOFF, config_.errorBackoffMs);
}

void O2Controller::transitionTo(State nextState) {
  timedStateMachine_.transitionTo(static_cast<uint8_t>(nextState));
}

void O2Controller::transitionToFor(State nextState, uint32_t durationMs) {
  timedStateMachine_.transitionToFor(static_cast<uint8_t>(nextState), durationMs);
}
// O2Controller.cpp v2