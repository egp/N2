// SystemRuntime.h v1
#ifndef SYSTEM_RUNTIME_H
#define SYSTEM_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

struct SystemRuntime {
  bool systemEnabled;
  bool systemWasEnabled;
  bool lastN2ControllerOk;

  uint8_t selectedDisplay;
  uint8_t previousSelectedDisplay;

  bool previousBlackSwitch;
  bool previousRedSwitch;
  bool previousGreenSwitch;
  bool previousBlueSwitch;

  bool previousCompressorSsr;
};

#endif

// SystemRuntime.h v1