// TowerController.cpp v2
#include "TowerController.h"

TowerController::Config TowerController::defaultConfig() {
  Config config;
  config.leftOpenMs = 60000UL;
  config.overlapMs = 750UL;
  config.rightOpenMs = 60000UL;
  return config;
}

TowerController::TowerController(IClock& clock, ITowerValveDriver& valveDriver)
    : timedStateMachine_(clock, STATE_INACTIVE),
      valveDriver_(valveDriver),
      config_(defaultConfig()),
      enabled_(false) {
  applyOutputsForState(STATE_INACTIVE);
}

TowerController::TowerController(IClock& clock, ITowerValveDriver& valveDriver, const Config& config)
    : timedStateMachine_(clock, STATE_INACTIVE),
      valveDriver_(valveDriver),
      config_(config),
      enabled_(false) {
  applyOutputsForState(STATE_INACTIVE);
}

void TowerController::setEnabled(bool enabled) {
  if (enabled == enabled_) {
    return;
  }

  enabled_ = enabled;

  if (!enabled_) {
    transitionTo(STATE_INACTIVE, 0U, false);
    return;
  }

  transitionTo(STATE_LEFT_ONLY, config_.leftOpenMs, true);
}

bool TowerController::isEnabled() const {
  return enabled_;
}

void TowerController::tick() {
  if (!enabled_) {
    return;
  }

  if (!timedStateMachine_.isExpired()) {
    return;
  }

  switch (state()) {
    case STATE_LEFT_ONLY:
      transitionTo(STATE_BOTH_AFTER_LEFT, config_.overlapMs, true);
      return;

    case STATE_BOTH_AFTER_LEFT:
      transitionTo(STATE_RIGHT_ONLY, config_.rightOpenMs, true);
      return;

    case STATE_RIGHT_ONLY:
      transitionTo(STATE_BOTH_AFTER_RIGHT, config_.overlapMs, true);
      return;

    case STATE_BOTH_AFTER_RIGHT:
      transitionTo(STATE_LEFT_ONLY, config_.leftOpenMs, true);
      return;

    case STATE_INACTIVE:
    default:
      return;
  }
}

TowerController::State TowerController::state() const {
  return static_cast<State>(timedStateMachine_.state());
}

bool TowerController::isActive() const {
  return state() != STATE_INACTIVE;
}

const TowerController::Config& TowerController::config() const {
  return config_;
}

void TowerController::setConfig(const Config& config) {
  config_ = config;
}

void TowerController::transitionTo(State nextState, uint32_t durationMs, bool timed) {
  if (timed) {
    timedStateMachine_.transitionToFor(static_cast<uint8_t>(nextState), durationMs);
  } else {
    timedStateMachine_.transitionTo(static_cast<uint8_t>(nextState));
  }

  applyOutputsForState(nextState);
}

void TowerController::applyOutputsForState(State state) {
  switch (state) {
    case STATE_INACTIVE:
      valveDriver_.setLeftOpen(false);
      valveDriver_.setRightOpen(false);
      return;

    case STATE_LEFT_ONLY:
      valveDriver_.setLeftOpen(true);
      valveDriver_.setRightOpen(false);
      return;

    case STATE_BOTH_AFTER_LEFT:
      valveDriver_.setLeftOpen(true);
      valveDriver_.setRightOpen(true);
      return;

    case STATE_RIGHT_ONLY:
      valveDriver_.setLeftOpen(false);
      valveDriver_.setRightOpen(true);
      return;

    case STATE_BOTH_AFTER_RIGHT:
      valveDriver_.setLeftOpen(true);
      valveDriver_.setRightOpen(true);
      return;

    default:
      valveDriver_.setLeftOpen(false);
      valveDriver_.setRightOpen(false);
      return;
  }
}
// TowerController.cpp v2