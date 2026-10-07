#include "SoftI2c.h"

namespace n2 {

void SoftI2c::sdaLow() {
  hal_.digitalWrite(sda_, false);
  hal_.pinMode(sda_, PinMode::kOutput);
}
void SoftI2c::sdaHigh() { hal_.pinMode(sda_, PinMode::kInput); }
void SoftI2c::sclLow() {
  hal_.digitalWrite(scl_, false);
  hal_.pinMode(scl_, PinMode::kOutput);
}
void SoftI2c::sclHigh() { hal_.pinMode(scl_, PinMode::kInput); }
bool SoftI2c::sdaRead() { return hal_.digitalRead(sda_); }

void SoftI2c::begin() {
  if (!configured()) return;
  sdaHigh();
  sclHigh();
  wait();
}

void SoftI2c::start() {  // SDA falls while SCL is high
  sdaHigh();
  sclHigh();
  wait();
  sdaLow();
  wait();
  sclLow();
  wait();
}

void SoftI2c::stop() {  // SDA rises while SCL is high
  sdaLow();
  wait();
  sclHigh();
  wait();
  sdaHigh();
  wait();
}

bool SoftI2c::sendByte(uint8_t value) {
  for (uint8_t bit = 0; bit < 8; ++bit) {
    if (value & 0x80) sdaHigh();
    else sdaLow();
    wait();
    sclHigh();
    wait();
    sclLow();
    value = static_cast<uint8_t>(value << 1);
  }
  sdaHigh();  // let the slave answer
  wait();
  sclHigh();
  wait();
  const bool ack = !sdaRead();  // low = acknowledged
  sclLow();
  wait();
  return ack;
}

bool SoftI2c::write(uint8_t address, const uint8_t* data, size_t n) {
  if (!configured()) return false;
  start();
  bool ok = sendByte(static_cast<uint8_t>(address << 1));  // address + write bit
  for (size_t i = 0; ok && i < n; ++i) ok = sendByte(data[i]);
  stop();
  return ok;
}

bool SoftI2c::probe(uint8_t address) {
  if (!configured()) return false;
  start();
  const bool ok = sendByte(static_cast<uint8_t>(address << 1));
  stop();
  return ok;
}

void SoftI2c::recover() {
  if (!configured()) return;
  sdaHigh();
  sclHigh();
  wait();
  for (uint8_t pulse = 0; pulse < 9 && !sdaRead(); ++pulse) {
    sclLow();
    wait();
    sclHigh();
    wait();
  }
  stop();
}

}  // namespace n2
