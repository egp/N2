// TowerController.cpp v9
#include "TowerController.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

static const char* towerStateName(uint8_t state) {
  switch (static_cast<TowerController::State>(state)) {
    case TowerController::STATE_INACTIVE: return "Inactive";
    case TowerController::STATE_LEFT_ONLY: return "LeftOnly";
    case TowerController::STATE_BOTH_AFTER_LEFT: return "BothAfterLeft";
    case TowerController::STATE_RIGHT_ONLY: return "RightOnly";
    case TowerController::STATE_BOTH_AFTER_RIGHT: return "BothAfterRight";
    case TowerController::STATE_LOW_SUPPLY: return "LowSupply";
    default: return "Unknown";
  }
}

TowerController::StateView::StateView(const TowerController& owner)
 : owner_(owner) {
}

ControllerKind TowerController::StateView::kind() const {
 return ControllerKind::Tower;
}

uint32_t TowerController::StateView::enteredAtMs() const {
 return owner_.timedStateMachine_.stateEnteredAtMs();
}

uint32_t TowerController::StateView::code() const {
 return static_cast<uint32_t>(owner_.state());
}

const char* TowerController::StateView::name() const {
 switch (owner_.state()) {
 case TowerController::STATE_INACTIVE:
  return "Inactive";
 case TowerController::STATE_LEFT_ONLY:
  return "LeftOnly";
 case TowerController::STATE_BOTH_AFTER_LEFT:
  return "BothAfterLeft";
 case TowerController::STATE_RIGHT_ONLY:
  return "RightOnly";
 case TowerController::STATE_BOTH_AFTER_RIGHT:
  return "BothAfterRight";
 case TowerController::STATE_LOW_SUPPLY:
  return "LowSupply";
 default:
  return "Unknown";
 }
}

TowerController::Config TowerController::defaultConfig() {
 Config config;
 config.overlapMs = 750UL;
 config.towerOpenMs = 60000UL-config.overlapMs;
 config.airSupplyOnPsi_x10 = 900U;  // on above this
 config.airSupplyOffPsi_x10 = 700U; // off beloew this
 return config;
}

TowerController::TowerController(IClock& clock, IBinaryOutput& leftValve, IBinaryOutput& rightValve)
 : TowerController(clock, leftValve, rightValve, defaultConfig()) {
}

TowerController::TowerController(
    IClock& clock,
    IBinaryOutput& leftValve,
    IBinaryOutput& rightValve,
    const SystemConfig& systemConfig)
 : TowerController(clock, leftValve, rightValve, systemConfig.tower) {
}

TowerController::TowerController(
    IClock& clock,
    IBinaryOutput& leftValve,
    IBinaryOutput& rightValve,
    const Config& config)
 : clock_(clock),
   timedStateMachine_(clock, STATE_INACTIVE, "Tower", towerStateName),
   leftValve_(leftValve),
   rightValve_(rightValve),
   config_(config),
   enabled_(false),
   stateView_(*this) {
 applyOutputsForState(STATE_INACTIVE);
}

bool TowerController::init() {
 return true;
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

 transitionTo(STATE_LEFT_ONLY, config_.towerOpenMs, true);
}

bool TowerController::isSupplySufficient(uint16_t supplyPsi_x10) const {
   if (TowerController::isActive()) {
    return (supplyPsi_x10 > config_.airSupplyOffPsi_x10);
  } else {
    return (supplyPsi_x10 >= config_.airSupplyOnPsi_x10); 
  }
}

void TowerController::step(const InputSnapshot& inputs) {
 if (!enabled_) {
  return;
 }

/* 
while running, if supply goes below low threshold, disable
while idle, if supply goes above high threshold, enable
*/
  if (TowerController::isActive()) {
    if (inputs.supplyPsi_x10 <= config_.airSupplyOffPsi_x10) {
    transitionTo(STATE_LOW_SUPPLY, 0U, false);
    return;
    }
 } else {
    if (inputs.supplyPsi_x10 >= config_.airSupplyOnPsi_x10) {
    transitionTo(STATE_LEFT_ONLY, config_.towerOpenMs, true);
    return;
    }
 }

 if (!timedStateMachine_.isExpired()) {
  return;
 }

 switch (state()) {
 case STATE_LEFT_ONLY:
  transitionTo(STATE_BOTH_AFTER_LEFT, config_.overlapMs, true);
  return;

 case STATE_BOTH_AFTER_LEFT:
  transitionTo(STATE_RIGHT_ONLY, config_.towerOpenMs, true);
  return;

 case STATE_RIGHT_ONLY:
  transitionTo(STATE_BOTH_AFTER_RIGHT, config_.overlapMs, true);
  return;

 case STATE_BOTH_AFTER_RIGHT:
  transitionTo(STATE_LEFT_ONLY, config_.towerOpenMs, true);
  return;

 case STATE_INACTIVE:
 case STATE_LOW_SUPPLY:
 default:
  return;
 }
}

void TowerController::shutdown() {
 setEnabled(false);
}

IClock& TowerController::clock() const {
 return clock_;
}

const ControllerState& TowerController::getState() const {
 return stateView_;
}

bool TowerController::isEnabled() const {
 return enabled_;
}

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
 return Snapshot{
     timedStateMachine_.stateEnteredAtMs(),
     state(),
 };
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
 case STATE_LOW_SUPPLY:
  leftValve_.setOn(false);
  rightValve_.setOn(false);
  return;

 case STATE_LEFT_ONLY:
  leftValve_.setOn(true);
  rightValve_.setOn(false);
  return;

 case STATE_BOTH_AFTER_LEFT:
  leftValve_.setOn(true);
  rightValve_.setOn(true);
  return;

 case STATE_RIGHT_ONLY:
  leftValve_.setOn(false);
  rightValve_.setOn(true);
  return;

 case STATE_BOTH_AFTER_RIGHT:
  leftValve_.setOn(true);
  rightValve_.setOn(true);
  return;

 default:
  leftValve_.setOn(false);
  rightValve_.setOn(false);
  return;
 }
}
// TowerController.cpp v9
