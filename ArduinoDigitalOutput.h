// ArduinoDigitalOutput.h v2
#ifndef ARDUINO_DIGITAL_OUTPUT_H
#define ARDUINO_DIGITAL_OUTPUT_H

#include <Arduino.h>
#include "BinaryOutput.h"

class ArduinoDigitalOutput : public IBinaryOutput {
public:
  explicit ArduinoDigitalOutput(uint8_t pin)
      : pin_(pin), isOn_(false), begun_(false) {}

  void begin(bool initialOn = false) {
    pinMode(pin_, OUTPUT);
    begun_ = true;
    setOn(initialOn);
  }

  void setOn(bool on) override {
    isOn_ = on;
    digitalWrite(pin_, on ? HIGH : LOW);
  }

  bool isOn() const { return isOn_; }
  bool hasBegun() const { return begun_; }
  uint8_t pin() const { return pin_; }

private:
  uint8_t pin_;
  bool isOn_;
  bool begun_;
};

#endif
// ArduinoDigitalOutput.h v2