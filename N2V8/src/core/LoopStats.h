// LoopStats.h — loop() timing: min, mean, median, max (Requirements NFR-1, WDT-1).
//
// Median comes from a fixed histogram (no dynamic allocation): it is the upper edge of the bucket that
// holds the middle sample, capped at the true maximum. Goal: max loop() well under one second.
#pragma once

#include <stdint.h>

namespace n2 {

class LoopStats {
 public:
  static constexpr uint8_t kBuckets = 16;

  // Duration of one loop pass in microseconds.
  void record(uint32_t us);
  void reset();

  uint32_t count() const { return count_; }
  uint32_t minUs() const { return count_ ? min_ : 0; }
  uint32_t maxUs() const { return max_; }
  uint32_t meanUs() const { return count_ ? static_cast<uint32_t>(sum_ / count_) : 0; }
  uint32_t medianUs() const;
  uint32_t slowCount() const { return slow_; }  // passes of one second or more

  // Upper edge of bucket i in microseconds (the last bucket is open-ended).
  static uint32_t bucketLimitUs(uint8_t i);

 private:
  uint32_t buckets_[kBuckets] = {};
  uint32_t count_ = 0;
  uint32_t min_ = 0xFFFFFFFFu;
  uint32_t max_ = 0;
  uint32_t slow_ = 0;
  uint64_t sum_ = 0;
};

}  // namespace n2
