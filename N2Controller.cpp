// N2Controller.cpp v1
#include "N2Controller.h"

namespace {
const char* n2StateName(uint8_t state) {
  switch (static_cast<N2Controller::State>(state)) {
    case N2Controller::STATE_BELOW_LOW_OFF:   return "BelowLowOff";
    case N2Controller::STATE_LOW_BAND_RISING: return "LowBandRising";
    case N2Controller::STATE_MIDDLE_ON:       return "MiddleOn";
    case N2Controller::STATE_HIGH_BAND_RISING:return "HighBandRising";
    case N2Controller::STATE_ABOVE_HIGH_OFF:  return "AboveHighOff";
    case N2Controller::STATE_HIGH_BAND_FALLING:return "HighBandFalling";
    case N2Controller::STATE_LOW_BAND_FALLING:return "LowBandFalling";
    default:                                  return "Unknown";
  }
}
}

N2Controller::Config N2Controller::defaultConfig() {
  Config config;
  config.lowOffPsi = 5U;        // scaled x100 
  config.lowOnPsi = 10U;
  config.highOnPsi = 15U;
  config.highOffPsi = 20U;
  return config;
}

N2Controller::N2Controller(IClock& clock, IBinaryOutput& compressorOutput)
    : timedStateMachine_(clock, STATE_BELOW_LOW_OFF, "N2", n2StateName),
      compressorOutput_(compressorOutput),
      config_(defaultConfig()) {
  applyOutputForState(STATE_BELOW_LOW_OFF);
}

N2Controller::N2Controller(IClock& clock, IBinaryOutput& compressorOutput, const Config& config)
    : timedStateMachine_(clock, STATE_BELOW_LOW_OFF, "N2", n2StateName),
      compressorOutput_(compressorOutput),
      config_(config) {
  applyOutputForState(STATE_BELOW_LOW_OFF);
}

void N2Controller::update(uint16_t lowPsi) {
  bool changed;

  do {
    changed = false;

    switch (state()) {
      case STATE_BELOW_LOW_OFF:
        if (lowPsi > config_.lowOffPsi) {
          transitionTo(STATE_LOW_BAND_RISING);
          changed = true;
        }
        break;

      case STATE_LOW_BAND_RISING:
        if (lowPsi < config_.lowOffPsi) {
          transitionTo(STATE_BELOW_LOW_OFF);
          changed = true;
        } else if (lowPsi > config_.lowOnPsi) {
          transitionTo(STATE_MIDDLE_ON);
          changed = true;
        }
        break;

      case STATE_MIDDLE_ON:
        if (lowPsi < config_.lowOnPsi) {
          transitionTo(STATE_LOW_BAND_FALLING);
          changed = true;
        } else if (lowPsi > config_.highOnPsi) {
          transitionTo(STATE_HIGH_BAND_RISING);
          changed = true;
        }
        break;

      case STATE_HIGH_BAND_RISING:
        if (lowPsi < config_.highOnPsi) {
          transitionTo(STATE_MIDDLE_ON);
          changed = true;
        } else if (lowPsi > config_.highOffPsi) {
          transitionTo(STATE_ABOVE_HIGH_OFF);
          changed = true;
        }
        break;

      case STATE_ABOVE_HIGH_OFF:
        if (lowPsi < config_.highOffPsi) {
          transitionTo(STATE_HIGH_BAND_FALLING);
          changed = true;
        }
        break;

      case STATE_HIGH_BAND_FALLING:
        if (lowPsi > config_.highOffPsi) {
          transitionTo(STATE_ABOVE_HIGH_OFF);
          changed = true;
        } else if (lowPsi < config_.highOnPsi) {
          transitionTo(STATE_MIDDLE_ON);
          changed = true;
        }
        break;

      case STATE_LOW_BAND_FALLING:
        if (lowPsi > config_.lowOnPsi) {
          transitionTo(STATE_MIDDLE_ON);
          changed = true;
        } else if (lowPsi < config_.lowOffPsi) {
          transitionTo(STATE_BELOW_LOW_OFF);
          changed = true;
        }
        break;

      default:
        transitionTo(STATE_BELOW_LOW_OFF);
        changed = true;
        break;
    }
  } while (changed);
}

N2Controller::State N2Controller::state() const {
  return static_cast<State>(timedStateMachine_.state());
}

const N2Controller::Config& N2Controller::config() const {
  return config_;
}

void N2Controller::setConfig(const Config& config) {
  config_ = config;
}

bool N2Controller::isCompressorOn() const {
  switch (state()) {
    case STATE_MIDDLE_ON:
    case STATE_HIGH_BAND_RISING:
    case STATE_LOW_BAND_FALLING:
      return true;

    case STATE_BELOW_LOW_OFF:
    case STATE_LOW_BAND_RISING:
    case STATE_ABOVE_HIGH_OFF:
    case STATE_HIGH_BAND_FALLING:
    default:
      return false;
  }
}

void N2Controller::transitionTo(State nextState) {
  timedStateMachine_.transitionTo(static_cast<uint8_t>(nextState));
  applyOutputForState(nextState);
}

void N2Controller::applyOutputForState(State state) {
  switch (state) {
    case STATE_MIDDLE_ON:
    case STATE_HIGH_BAND_RISING:
    case STATE_LOW_BAND_FALLING:
      compressorOutput_.setOn(true);
      return;

    case STATE_BELOW_LOW_OFF:
    case STATE_LOW_BAND_RISING:
    case STATE_ABOVE_HIGH_OFF:
    case STATE_HIGH_BAND_FALLING:
    default:
      compressorOutput_.setOn(false);
      return;
  }
}
// N2Controller.cpp v1