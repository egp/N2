// N2Controller.h v3
#ifndef N2_CONTROLLER_H
#define N2_CONTROLLER_H

#include <stdint.h>

#include "BinaryOutput.h"
#include "TimedStateMachine.h"

class N2Controller {

public:

 struct Config {
  uint16_t lowOffPsi_x100;
  uint16_t lowOnPsi_x100;
  uint16_t highOnPsi_x10;
  uint16_t highOffPsi_x10;
 };

 enum State : uint8_t {
  STATE_LOW_INHIBIT_HIGH_PERMIT = 0,
  STATE_LOW_PERMIT_HIGH_PERMIT,
  STATE_LOW_PERMIT_HIGH_INHIBIT,
  STATE_LOW_INHIBIT_HIGH_INHIBIT
 };

 static Config defaultConfig();

 N2Controller(IClock& clock, IBinaryOutput& compressorOutput);
 N2Controller(IClock& clock, IBinaryOutput& compressorOutput, const Config& config);

 bool update(uint16_t lowPsi_x100, uint16_t highPsi_x10);

 State state() const;

 const Config& config() const;

 void setConfig(const Config& config);

 bool isCompressorOn() const;

private:

 void transitionTo(State nextState);

 void applyOutputForState(State state);

 TimedStateMachine timedStateMachine_;

 IBinaryOutput& compressorOutput_;

 Config config_;

};

#endif

// N2Controller.h v3