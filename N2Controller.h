// N2Controller.h v1
#ifndef N2_CONTROLLER_H
#define N2_CONTROLLER_H

#include <stdint.h>
#include "BinaryOutput.h"
#include "TimedStateMachine.h"

class N2Controller {
public:
  struct Config {
    uint16_t lowOffPsi;
    uint16_t lowOnPsi;
    uint16_t highOnPsi;
    uint16_t highOffPsi;
  };

  enum State : uint8_t {
    STATE_BELOW_LOW_OFF = 0,
    STATE_LOW_BAND_RISING,
    STATE_MIDDLE_ON,
    STATE_HIGH_BAND_RISING,
    STATE_ABOVE_HIGH_OFF,
    STATE_HIGH_BAND_FALLING,
    STATE_LOW_BAND_FALLING
  };

  static Config defaultConfig();

  N2Controller(IClock& clock, IBinaryOutput& compressorOutput);
  N2Controller(IClock& clock, IBinaryOutput& compressorOutput, const Config& config);

  void update(uint16_t lowPsi);

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
// N2Controller.h v1