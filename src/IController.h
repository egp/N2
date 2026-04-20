// IController.h v1
#ifndef I_CONTROLLER_H
#define I_CONTROLLER_H

#include "ControllerState.h"
#include "InputSnapshot.h"
#include "TimedStateMachine.h"

class IController {
public:
  virtual ~IController() {}

  virtual bool init() = 0;
  virtual void setEnabled(bool enabled) = 0;
  virtual void step(const InputSnapshot& input) = 0;
  virtual void shutdown() = 0;

  virtual IClock& clock() const = 0;
  virtual const ControllerState& getState() const = 0;
};

#endif

// IController.h v1