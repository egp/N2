// SoftBusModel.h — a pretend I2C slave on two GPIO pins, for testing bit-banged I2C (SoftI2c) on the host.
//
// It watches the FakeHal's pin events (open-drain: a line is LOW when its pin is an OUTPUT driven LOW, otherwise HIGH through the pull-up),
// decodes START, bytes (MSB first, sampled at SCL rising), the acknowledge slot and STOP, and answers the master's reads of SDA: LOW during
// the acknowledge slot of a byte it accepts. It records every complete transaction.
#pragma once

#include <cstdint>
#include <set>
#include <vector>

#include "FakeHal.h"

namespace n2 {

struct SoftBusTransaction {
  uint8_t address = 0;              // 7-bit
  bool write = true;
  std::vector<uint8_t> bytes;       // data bytes after the address (all of them, accepted or not)
  std::vector<bool> acked;          // one per byte after the address byte: did the model acknowledge it
  bool addressAcked = false;
};

class SoftBusModel {
 public:
  SoftBusModel(FakeHal& hal, uint8_t sda, uint8_t scl) : hal_(hal), sda_(sda), scl_(scl) {
    hal_.readHook = [this](uint8_t pin) { return onRead(pin); };
  }

  std::set<uint8_t> addresses;            // addresses this slave answers
  unsigned maxDataBytes = 99;             // data bytes it acknowledges per transaction (a TM1650 takes exactly one)
  unsigned stuckUntilPulse = 0;           // hold SDA low until this many SCL pulses have been seen (a slave stuck mid-byte)
  std::vector<SoftBusTransaction> transactions;

  bool sdaHigh() { sync(); return lineLevel(sda_) && !(stuckUntilPulse > pulses_); }
  bool sclHigh() { sync(); return lineLevel(scl_); }
  unsigned pulses() { sync(); return pulses_; }
  void sync();  // decode everything the master has done so far

 private:
  bool onRead(uint8_t pin);
  bool lineLevel(uint8_t pin) const { return !low_[pin]; }
  void edge(bool prevSda, bool sda, bool prevScl, bool scl);
  bool wantsAck() const;

  FakeHal& hal_;
  uint8_t sda_, scl_;
  size_t processed_ = 0;
  bool low_[32] = {};
  bool lastWrite_[32] = {};
  bool mode_[32] = {};          // true = OUTPUT
  bool prevSda_ = true, prevScl_ = true;
  bool inTransaction_ = false;
  unsigned bitCount_ = 0;       // bits clocked in the current byte (0..8), 9 = acknowledge slot
  unsigned byteIndex_ = 0;      // 0 = the address byte
  uint8_t shift_ = 0;
  bool ackSlot_ = false;
  unsigned pulses_ = 0;
  SoftBusTransaction current_;
};

inline bool SoftBusModel::wantsAck() const {
  if (byteIndex_ == 0) return addresses.count(static_cast<uint8_t>(shift_ >> 1)) != 0;
  return byteIndex_ <= maxDataBytes && current_.addressAcked;
}

inline void SoftBusModel::edge(bool prevSda, bool sda, bool prevScl, bool scl) {
  if (scl && prevScl && prevSda && !sda) {  // START: SDA falls while SCL is high
    inTransaction_ = true;
    bitCount_ = 0;
    byteIndex_ = 0;
    shift_ = 0;
    ackSlot_ = false;
    current_ = SoftBusTransaction();
    return;
  }
  if (scl && prevScl && !prevSda && sda) {  // STOP: SDA rises while SCL is high
    if (inTransaction_) transactions.push_back(current_);
    inTransaction_ = false;
    ackSlot_ = false;
    return;
  }
  if (!inTransaction_) return;
  if (!prevScl && scl) {  // SCL rising
    if (bitCount_ < 8) {
      shift_ = static_cast<uint8_t>((shift_ << 1) | (sda ? 1 : 0));
      ++bitCount_;
    } else {
      bitCount_ = 9;  // the acknowledge pulse
    }
  }
  if (prevScl && !scl) {  // SCL falling
    if (bitCount_ == 8) {  // the 8th bit is done: the slave now answers in the acknowledge slot
      ackSlot_ = wantsAck();
      if (byteIndex_ == 0) {
        current_.address = static_cast<uint8_t>(shift_ >> 1);
        current_.write = (shift_ & 1) == 0;
        current_.addressAcked = ackSlot_;
      } else {
        current_.bytes.push_back(shift_);
        current_.acked.push_back(ackSlot_);
      }
    } else if (bitCount_ == 9) {  // the acknowledge pulse is over: next byte
      ackSlot_ = false;
      bitCount_ = 0;
      shift_ = 0;
      ++byteIndex_;
    }
  }
}

inline void SoftBusModel::sync() {
  while (processed_ < hal_.events.size()) {
    const FakeHal::Event& e = hal_.events[processed_++];
    if (e.pin != sda_ && e.pin != scl_) continue;
    const bool oldSda = lineLevel(sda_) && !(stuckUntilPulse > pulses_), oldScl = lineLevel(scl_);
    if (e.kind == FakeHal::Kind::kWrite) {
      lastWrite_[e.pin] = e.value != 0;
      low_[e.pin] = mode_[e.pin] && !lastWrite_[e.pin];
    } else if (e.kind == FakeHal::Kind::kPinMode) {
      mode_[e.pin] = e.value == static_cast<int>(PinMode::kOutput);
      low_[e.pin] = mode_[e.pin] && !lastWrite_[e.pin];
    } else {
      continue;
    }
    const bool newScl = lineLevel(scl_);
    if (!oldScl && newScl) ++pulses_;  // every SCL rising edge counts, in a transaction or not (a recovery sends bare pulses)
    const bool newSda = lineLevel(sda_) && !(stuckUntilPulse > pulses_);
    edge(oldSda, newSda, oldScl, newScl);
    prevSda_ = newSda;
    prevScl_ = newScl;
  }
}

inline bool SoftBusModel::onRead(uint8_t pin) {
  sync();
  if (pin == sda_) {
    if (ackSlot_ && !low_[sda_]) return false;  // the slave pulls SDA low to acknowledge
    return lineLevel(sda_) && !(stuckUntilPulse > pulses_);
  }
  if (pin == scl_) return lineLevel(scl_);
  return hal_.inputLevel[pin];
}

}  // namespace n2
