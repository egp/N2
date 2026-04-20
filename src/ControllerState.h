// ControllerState.h v1
#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

#include <stdint.h>

enum class ControllerKind : uint8_t {
  Tower = 0,
  O2,
  N2,
};

class ControllerState {
public:
  virtual ~ControllerState() {}

  virtual ControllerKind kind() const = 0;
  virtual uint32_t enteredAtMs() const = 0;
  virtual uint32_t code() const = 0;
  virtual const char* name() const = 0;
};

#endif

// ControllerState.h v1