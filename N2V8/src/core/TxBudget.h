// TxBudget.h — how many bytes the console may send right now (Requirements CON-2, CON-4).
//
// On the UNO R4 WiFi the console is a hardware UART and every write() BLOCKS until the byte has left the chip
// (about 87 microseconds per byte at 115200 baud). To keep one loop pass short, output is paced by a budget that refills
// at about the line rate (11 bytes per millisecond) and is capped, so a burst can never block for long.
// Pure arithmetic: no hardware, so it is tested on the host.
#pragma once

#include <stdint.h>

namespace n2 {

class TxBudget {
 public:
  // bytesPerMs: refill rate (11 = 115200 baud). cap: the most that can accumulate (and so the longest burst).
  TxBudget(uint32_t bytesPerMs = 11, uint32_t cap = 128) : bytesPerMs_(bytesPerMs), cap_(cap), tokens_(cap) {}

  // Bytes that may be written now.
  uint32_t space(uint32_t nowMs) {
    refill(nowMs);
    return tokens_;
  }

  // Record that n bytes were written.
  void consume(uint32_t n, uint32_t nowMs) {
    refill(nowMs);
    tokens_ = n >= tokens_ ? 0 : tokens_ - n;
  }

 private:
  void refill(uint32_t nowMs) {
    if (!started_) {
      started_ = true;
      last_ = nowMs;
      return;
    }
    const uint32_t elapsed = static_cast<uint32_t>(nowMs - last_);  // rollover-safe
    last_ = nowMs;
    // Guard against overflow for long gaps: anything beyond the cap is lost anyway.
    const uint32_t add = elapsed > cap_ ? cap_ : elapsed * bytesPerMs_;
    tokens_ = tokens_ + add > cap_ ? cap_ : tokens_ + add;
  }

  uint32_t bytesPerMs_;
  uint32_t cap_;
  uint32_t tokens_;
  uint32_t last_ = 0;
  bool started_ = false;
};

}  // namespace n2
