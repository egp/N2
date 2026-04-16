// TowerController.cpp v5
#include "TowerController.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

TowerController::Config TowerController::defaultConfig() {
 Config config;
 config.leftOpenMs = 60000UL;
 config.overlapMs = 750UL;
 config.rightOpenMs = 60000UL;
 config.lowSupplyPsi_x10 = 1000U; // 100.0 PSI
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
   timedStateMachine_(clock, STATE_INACTIVE),
   leftValve_(leftValve),
   rightValve_(rightValve),
   config_(config),
   enabled_(false) {
  applyOutputsForState(STATE_INACTIVE);
}

bool TowerController::init() {
  return true;
}

void TowerController::step(const InputSnapshot& inputs) {
  tick(inputs);
}

void TowerController::shutdown() {
  setEnabled(false);
}

IClock& TowerController::clock() const {
  return clock_;
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

void TowerController::tick(const InputSnapshot& inputs) {
 if (!enabled_) {
  return;
 }

 if (!isSupplySufficient(inputs.supplyPsi_x10)) {
  if (state() != STATE_LOW_SUPPLY) {
   transitionTo(STATE_LOW_SUPPLY, 0U, false);
  }
  return;
 }

 if (state() == STATE_LOW_SUPPLY) {
  transitionTo(STATE_LEFT_ONLY, config_.leftOpenMs, true);
  return;
 }

 if (!timedStateMachine_.isExpired()) {
  return;
 }

 switch (state()) {
 case STATE_LEFT_ONLY:
#ifdef ARDUINO
  Serial.println(F("Tower Transitioning from LEFT_ONLY to BOTH_AFTER_LEFT"));
#endif
  transitionTo(STATE_BOTH_AFTER_LEFT, config_.overlapMs, true);
  return;

 case STATE_BOTH_AFTER_LEFT:
#ifdef ARDUINO
  Serial.println(F("Tower Transitioning from BOTH_AFTER_LEFT to RIGHT_ONLY"));
#endif
  transitionTo(STATE_RIGHT_ONLY, config_.rightOpenMs, true);
  return;

 case STATE_RIGHT_ONLY:
#ifdef ARDUINO
  Serial.println(F("Tower Transitioning from RIGHT_ONLY to BOTH_AFTER_RIGHT"));
#endif
  transitionTo(STATE_BOTH_AFTER_RIGHT, config_.overlapMs, true);
  return;

 case STATE_BOTH_AFTER_RIGHT:
#ifdef ARDUINO
  Serial.println(F("Tower Transitioning from BOTH_AFTER_RIGHT to LEFT_ONLY"));
#endif
  transitionTo(STATE_LEFT_ONLY, config_.leftOpenMs, true);
  return;

 case STATE_INACTIVE:
 case STATE_LOW_SUPPLY:
 default:
  return;
 }
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

bool TowerController::isSupplySufficient(uint16_t supplyPsi_x10) const {
 return supplyPsi_x10 >= config_.lowSupplyPsi_x10;
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

// TowerController.cpp v5