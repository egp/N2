// N2V8.ino — PSA nitrogen generator controller (UNO R4 Minima / WiFi).
//
// M2 skeleton: proves the shared src/ tree builds for both boards and that the
// pins are set up from BoardPins.h. No control logic yet.
// Requirements: docs/Requirements.md   Plan: docs/Project_Plan.md

#include "src/BoardPins.h"
#include "src/BuildConfig.h"
#include "src/Config.h"
#include "src/board/BoardSetup.h"
#include "src/hal/HalArduino.h"

static n2::HalArduino hal;

void setup() {
  // First thing: outputs to their safe level (Requirements RST-2). The core has
  // already run its own init and started USB serial before calling setup(), and
  // its initVariant() cannot be overridden, so this is the earliest hook we have.
  n2::beginHardware(hal);

  if (Serial) {  // never wait for a console (CON-2)
    Serial.print("N2V8 ");
    Serial.print(N2_VERSION);
    Serial.print("  ");
    Serial.print(n2::kBoard.name);
    Serial.print("  ");
    Serial.print(__DATE__);
    Serial.print(' ');
    Serial.println(__TIME__);
  }
}

void loop() {}
