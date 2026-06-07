// TowerController.cpp v10
#include "TowerController.h"

static const char* towerStateName(uint8_t state) {
  switch (static_cast<TowerController::State>(state)) {
    case TowerController::STATE_INACTIVE:         return "Inactive";
    case TowerController::STATE_LEFT_ONLY:        return "LeftOnly";
    case TowerController::STATE_BOTH_AFTER_LEFT:  return "BothAfterLeft";
    case TowerController::STATE_RIGHT_ONLY:       return "RightOnly";
    case TowerController::STATE_BOTH_AFTER_RIGHT: return "BothAfterRight";
    case TowerController::STATE_LOW_SUPPLY:       return "LowSupply";
    default:                                      return "Unknown";
  }
}

TowerController::Config TowerController::defaultConfig() {
  Config config;
  config.overlapMs           = 750UL;
  config.towerOpenMs         = 60000UL - config.overlapMs;
  config.airSupplyOnPsi_x10  = 900U;
  config.airSupplyOffPsi_x10 = 700U;
  return config;
}

TowerController::TowerController(IClock& clock, IBinaryOutput& leftValve,
                                 IBinaryOutput& rightValve)
    : TowerController(clock, leftValve, rightValve, defaultConfig()) {}

TowerController::TowerController(IClock& clock, IBinaryOutput& leftValve,
                                 IBinaryOutput& rightValve,
                                 const SystemConfig& systemConfig)
    : TowerController(clock, leftValve, rightValve, systemConfig.tower) {}

TowerController::TowerController(IClock& clock, IBinaryOutput& leftValve,
                                 IBinaryOutput& rightValve, const Config& config)
    : clock_(clock),
      timedStateMachine_(clock, STATE_INACTIVE, "Tower", towerStateName),
      leftValve_(leftValve),
      rightValve_(rightValve),
      config_(config),
      enabled_(false) {
  applyOutputsForState(STATE_INACTIVE);
}

bool TowerController::init() { return true; }

void TowerController::setEnabled(bool enabled) {
  if (enabled == enabled_) return;
  enabled_ = enabled;
  if (!enabled_) {
    transitionTo(STATE_INACTIVE);
    return;
  }
  transitionToFor(STATE_LEFT_ONLY, config_.towerOpenMs);
}

bool TowerController::isSupplySufficient(uint16_t supplyPsi_x10) const {
  if (isActive()) {
    return supplyPsi_x10 > config_.airSupplyOffPsi_x10;
  } else {
    return supplyPsi_x10 >= config_.airSupplyOnPsi_x10;
  }
}

void TowerController::step(const InputSnapshot& inputs) {
  if (!enabled_) return;

  if (isActive()) {
    if (inputs.supplyPsi_x10 <= config_.airSupplyOffPsi_x10) {
      transitionTo(STATE_LOW_SUPPLY);
      return;
    }
  } else {
    if (inputs.supplyPsi_x10 >= config_.airSupplyOnPsi_x10) {
      transitionToFor(STATE_LEFT_ONLY, config_.towerOpenMs);
      return;
    }
  }

  if (!timedStateMachine_.isExpired()) return;

  switch (state()) {
    case STATE_LEFT_ONLY:
      transitionToFor(STATE_BOTH_AFTER_LEFT, config_.overlapMs);
      return;
    case STATE_BOTH_AFTER_LEFT:
      transitionToFor(STATE_RIGHT_ONLY, config_.towerOpenMs);
      return;
    case STATE_RIGHT_ONLY:
      transitionToFor(STATE_BOTH_AFTER_RIGHT, config_.overlapMs);
      return;
    case STATE_BOTH_AFTER_RIGHT:
      transitionToFor(STATE_LEFT_ONLY, config_.towerOpenMs);
      return;
    case STATE_INACTIVE:
    case STATE_LOW_SUPPLY:
    default:
      return;
  }
}

void TowerController::shutdown() { setEnabled(false); }

bool TowerController::isEnabled() const { return enabled_; }

TowerController::State TowerController::state() const {
  return static_cast<State>(timedStateMachine_.state());
}

bool TowerController::isActive() const {
  switch (state()) {
    case STATE_LEFT_ONLY:
    case STATE_BOTH_AFTER_LEFT:
    case STATE_RIGHT_ONLY:
    case STATE_BOTH_AFTER_RIGHT:
      return true;
    case STATE_INACTIVE:
    case STATE_LOW_SUPPLY:
    default:
      return false;
  }
}

TowerController::Snapshot TowerController::snapshot() const {
  return Snapshot{ timedStateMachine_.stateEnteredAtMs(), state() };
}

const TowerController::Config& TowerController::config() const { return config_; }
void TowerController::setConfig(const Config& config) { config_ = config; }

void TowerController::transitionTo(State nextState) {
  timedStateMachine_.transitionTo(static_cast<uint8_t>(nextState));
  applyOutputsForState(nextState);
}

void TowerController::transitionToFor(State nextState, uint32_t durationMs) {
  timedStateMachine_.transitionToFor(static_cast<uint8_t>(nextState), durationMs);
  applyOutputsForState(nextState);
}

void TowerController::applyOutputsForState(State state) {
  switch (state) {
    case STATE_INACTIVE:
    case STATE_LOW_SUPPLY:
      leftValve_.setOn(false);  rightValve_.setOn(false);  return;
    case STATE_LEFT_ONLY:
      leftValve_.setOn(true);   rightValve_.setOn(false);  return;
    case STATE_BOTH_AFTER_LEFT:
    case STATE_BOTH_AFTER_RIGHT:
      leftValve_.setOn(true);   rightValve_.setOn(true);   return;
    case STATE_RIGHT_ONLY:
      leftValve_.setOn(false);  rightValve_.setOn(true);   return;
    default:
      leftValve_.setOn(false);  rightValve_.setOn(false);  return;
  }
}
// TowerController.cpp v10
