#include "LoopStats.h"

namespace n2 {

uint32_t LoopStats::bucketLimitUs(uint8_t i) {
  static const uint32_t kLimits[kBuckets] = {100,    200,    500,     1000,    2000,    5000,    10000,   20000,
                                             50000,  100000, 200000,  500000,  1000000, 2000000, 5000000, 0xFFFFFFFFu};
  return kLimits[i < kBuckets ? i : kBuckets - 1];
}

void LoopStats::record(uint32_t us) {
  uint8_t i = 0;
  while (i < kBuckets - 1 && us >= bucketLimitUs(i)) ++i;
  ++buckets_[i];
  ++count_;
  sum_ += us;
  if (us < min_) min_ = us;
  if (us > max_) max_ = us;
  if (us >= 1000000u) ++slow_;
}

void LoopStats::reset() { *this = LoopStats(); }

uint32_t LoopStats::medianUs() const {
  if (count_ == 0) return 0;
  const uint32_t target = (count_ + 1u) / 2u;
  uint32_t seen = 0;
  for (uint8_t i = 0; i < kBuckets; ++i) {
    seen += buckets_[i];
    if (seen >= target) {
      const uint32_t edge = bucketLimitUs(i);
      return edge < max_ ? edge : max_;
    }
  }
  return max_;
}

}  // namespace n2
