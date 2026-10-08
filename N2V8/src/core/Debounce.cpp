// Debounce.cpp — see Debounce.h.
#include "Debounce.h"

namespace n2 {

bool Debouncer::update(bool raw, uint32_t nowMs) {
  changed_ = false;
  if (raw != raw_) {  // any change restarts the wait
    raw_ = raw;
    sinceMs_ = nowMs;
  } else if (raw_ != stable_ && static_cast<uint32_t>(nowMs - sinceMs_) >= debounceMs_) {
    stable_ = raw_;
    changed_ = true;
  }
  return stable_;
}

void BounceMeter::reset() {
  stats_ = BounceStats();
  started_ = inOp_ = false;
  edges_ = 0;
}

void BounceMeter::finish() {
  const uint32_t settle = static_cast<uint32_t>(lastEdgeUs_ - firstEdgeUs_);
  if (stats_.operations == 0 || settle < stats_.settleMinUs) stats_.settleMinUs = settle;
  if (settle > stats_.settleMaxUs) stats_.settleMaxUs = settle;
  stats_.settleSumUs += settle;
  if (stats_.operations < kMaxKept) kept_[stats_.operations] = settle;
  stats_.lastSettleUs = settle;
  stats_.lastEdges = edges_;
  stats_.operations++;
  stats_.edgesTotal += edges_;
  if (edges_ > stats_.edgesMax) stats_.edgesMax = edges_;
  inOp_ = false;
  edges_ = 0;
}

void BounceMeter::sample(bool raw, uint32_t nowUs) {
  if (!started_) {
    started_ = true;
    level_ = raw;
    lastSampleUs_ = nowUs;
    return;
  }
  const uint32_t gap = static_cast<uint32_t>(nowUs - lastSampleUs_);
  if (gap > stats_.passMaxUs) stats_.passMaxUs = gap;
  lastSampleUs_ = nowUs;
  if (inOp_ && static_cast<uint32_t>(nowUs - lastEdgeUs_) >= quietUs_) finish();
  if (raw != level_) {
    level_ = raw;
    if (!inOp_) {
      inOp_ = true;
      firstEdgeUs_ = nowUs;
    }
    lastEdgeUs_ = nowUs;
    edges_++;
  }
}

void BounceMeter::flush(uint32_t nowUs) {
  if (inOp_ && static_cast<uint32_t>(nowUs - lastEdgeUs_) >= quietUs_) finish();
}

uint8_t BounceMeter::recommendedMs() const {
  if (stats_.operations == 0) return 0;
  uint32_t ms = (2 * stats_.settleMaxUs + 999) / 1000;
  if (ms < kMinDebounceMs) ms = kMinDebounceMs;
  if (ms > kMaxDebounceMs) ms = kMaxDebounceMs;
  return static_cast<uint8_t>(ms);
}

}  // namespace n2

namespace n2 {
uint32_t BounceMeter::settleMedianUs() const {
  const uint8_t n = stats_.operations < kMaxKept ? static_cast<uint8_t>(stats_.operations) : kMaxKept;
  if (n == 0) return 0;
  uint32_t v[kMaxKept];
  for (uint8_t i = 0; i < n; i++) v[i] = kept_[i];
  for (uint8_t i = 1; i < n; i++) {  // insertion sort: n <= 64
    const uint32_t x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return (n & 1) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}
}  // namespace n2
