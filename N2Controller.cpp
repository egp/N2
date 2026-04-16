// N2Controller.cpp v7
#include "N2Controller.h"

namespace {

const char* n2StateName(uint8_t state) {
 switch (static_cast<N2Controller::State>(state)) {
 case N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT:
  return "LowInhibitHighPermit";
 case N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT:
  return "LowPermitHighPermit";
 case N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT:
  return "LowPermitHighInhibit";
 case N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT:
  return "LowInhibitHighInhibit";
 default:
  return "Unknown";
 }
}

bool lowPermitFromState(N2Controller::State state) {
 switch (state) {
 case N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT:
 case N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT:
  return true;

 case N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT:
 case N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT:
 default:
  return false;
 }
}

bool highPermitFromState(N2Controller::State state) {
 switch (state) {
 case N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT:
 case N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT:
  return true;

 case N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT:
 case N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT:
 default:
  return false;
 }
}

N2Controller::State stateFromLatches(bool lowPermit, bool highPermit) {
 if (lowPermit) {
  return highPermit
      ? N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT
      : N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT;
 }

 return highPermit
     ? N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT
     : N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT;
}

} // namespace

N2Controller::StateView::StateView(const N2Controller& owner)
 : owner_(owner) {
}

ControllerKind N2Controller::StateView::kind() const {
 return ControllerKind::N2;
}

uint32_t N2Controller::StateView::enteredAtMs() const {
 return owner_.timedStateMachine_.stateEnteredAtMs();
}

uint32_t N2Controller::StateView::code() const {
 return static_cast<uint32_t>(owner_.state());
}

const char* N2Controller::StateView::name() const {
 switch (owner_.state()) {
 case N2Controller::STATE_LOW_INHIBIT_HIGH_PERMIT:
  return "LowInhibitHighPermit";
 case N2Controller::STATE_LOW_PERMIT_HIGH_PERMIT:
  return "LowPermitHighPermit";
 case N2Controller::STATE_LOW_PERMIT_HIGH_INHIBIT:
  return "LowPermitHighInhibit";
 case N2Controller::STATE_LOW_INHIBIT_HIGH_INHIBIT:
  return "LowInhibitHighInhibit";
 default:
  return "Unknown";
 }
}

N2Controller::Config N2Controller::defaultConfig() {
 Config config;
 config.lowOffPsi_x100 = 1000U;
 config.lowOnPsi_x100 = 2000U;
 config.highOnPsi_x10 = 1000U;
 config.highOffPsi_x10 = 1200U;
 return config;
}

N2Controller::N2Controller(IClock& clock, IBinaryOutput& compressorOutput)
 : N2Controller(clock, compressorOutput, defaultConfig()) {
}

N2Controller::N2Controller(
    IClock& clock,
    IBinaryOutput& compressorOutput,
    const SystemConfig& systemConfig)
 : N2Controller(clock, compressorOutput, systemConfig.n2) {
}

N2Controller::N2Controller(
    IClock& clock,
    IBinaryOutput& compressorOutput,
    const Config& config)
 : clock_(clock),
   timedStateMachine_(clock, STATE_LOW_INHIBIT_HIGH_PERMIT, "N2", n2StateName),
   compressorOutput_(compressorOutput),
   config_(config),
   stateView_(*this) {
 applyOutputForState(STATE_LOW_INHIBIT_HIGH_PERMIT);
}

bool N2Controller::init() {
 return true;
}

void N2Controller::setEnabled(bool enabled) {
 (void)enabled;
}

void N2Controller::step(const InputSnapshot& inputs) {
  const State currentState = state();

  bool lowPermit = lowPermitFromState(currentState);
  bool highPermit = highPermitFromState(currentState);

  if (inputs.lowN2Psi_x100 < config_.lowOffPsi_x100) {
    lowPermit = false;
  } else if (inputs.lowN2Psi_x100 > config_.lowOnPsi_x100) {
    lowPermit = true;
  }

  if (inputs.highN2Psi_x10 > config_.highOffPsi_x10) {
    highPermit = false;
  } else if (inputs.highN2Psi_x10 < config_.highOnPsi_x10) {
    highPermit = true;
  }

  const State nextState = stateFromLatches(lowPermit, highPermit);
  if (nextState != currentState) {
    transitionTo(nextState);
  }
}

void N2Controller::shutdown() {
 transitionTo(STATE_LOW_INHIBIT_HIGH_PERMIT);
}

IClock& N2Controller::clock() const {
 return clock_;
}

const ControllerState& N2Controller::getState() const {
 return stateView_;
}

N2Controller::State N2Controller::state() const {
 return static_cast<State>(timedStateMachine_.state());
}

N2Controller::Snapshot N2Controller::snapshot() const {
 return Snapshot{
     timedStateMachine_.stateEnteredAtMs(),
     state(),
 };
}

const N2Controller::Config& N2Controller::config() const {
 return config_;
}

void N2Controller::setConfig(const Config& config) {
 config_ = config;
}

bool N2Controller::isCompressorOn() const {
 return state() == STATE_LOW_PERMIT_HIGH_PERMIT;
}

bool N2Controller::isOk() const {
 return state() != STATE_LOW_INHIBIT_HIGH_INHIBIT;
}

void N2Controller::transitionTo(State nextState) {
 timedStateMachine_.transitionTo(static_cast<uint8_t>(nextState));
 applyOutputForState(nextState);
}

void N2Controller::applyOutputForState(State state) {
 switch (state) {
 case STATE_LOW_PERMIT_HIGH_PERMIT:
  compressorOutput_.setOn(true);
  return;

 case STATE_LOW_INHIBIT_HIGH_PERMIT:
 case STATE_LOW_PERMIT_HIGH_INHIBIT:
 case STATE_LOW_INHIBIT_HIGH_INHIBIT:
 default:
  compressorOutput_.setOn(false);
  return;
 }
}

// N2Controller.cpp v7