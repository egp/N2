// Debounce.h — switch debouncing and bounce measurement (Requirements INP-6, INP-10).
//
// All times are unsigned and compared by subtraction, so the 32-bit millis()/micros() rollover is harmless.
#pragma once

#include <stdint.h>

namespace n2 {

constexpr uint8_t kMinDebounceMs = 2;
constexpr uint8_t kMaxDebounceMs = 100;
constexpr uint8_t kBringupDebounceMs = 30;  // used until a measured value is stored; the compiled default is changed after measuring

// Accepts a new level only after the raw input has held it for debounceMs. Call update() every pass.
class Debouncer {
 public:
  explicit Debouncer(bool initial = false, uint8_t debounceMs = kBringupDebounceMs) : stable_(initial), raw_(initial), debounceMs_(debounceMs) {}
  void setDebounceMs(uint8_t ms) { debounceMs_ = ms; }
  uint8_t debounceMs() const { return debounceMs_; }
  // Returns the debounced level. changed() tells whether THIS call accepted a new level.
  bool update(bool raw, uint32_t nowMs);
  bool changed() const { return changed_; }
  bool level() const { return stable_; }

 private:
  bool stable_, raw_, changed_ = false;
  uint32_t sinceMs_ = 0;
  uint8_t debounceMs_;
};

// Measures bounce. Feed it the raw level with a micros() time on EVERY pass. An "operation" starts at the first change from
// the idle level and ends when the input has not changed for quietUs. For each operation it keeps the number of edges and the
// time from the first to the last edge (the settle time). Passes must be much shorter than the bounce (displays quiet).
struct BounceStats {
  uint16_t operations = 0;
  uint32_t edgesTotal = 0;
  uint16_t edgesMax = 0;
  uint32_t settleMinUs = 0, settleMaxUs = 0, settleSumUs = 0;
  uint32_t lastSettleUs = 0; uint16_t lastEdges = 0;   // the most recent operation
  uint32_t passMaxUs = 0;   // longest gap between two samples: edges closer than this may have been missed
};

class BounceMeter {
 public:
  explicit BounceMeter(uint32_t quietUs = 200000) : quietUs_(quietUs) {}
  void reset();
  void sample(bool raw, uint32_t nowUs);
  void flush(uint32_t nowUs);                    // close an operation that is waiting for its quiet time
  const BounceStats& stats() const { return stats_; }
  // 2 x the longest settle time, rounded up to whole ms, limited to 2..100. 0 if nothing was measured.
  uint8_t recommendedMs() const;
  uint32_t settleMeanUs() const { return stats_.operations ? stats_.settleSumUs / stats_.operations : 0; }

 private:
  void finish();
  uint32_t quietUs_;
  BounceStats stats_;
  bool started_ = false, level_ = false, inOp_ = false;
  uint32_t lastSampleUs_ = 0, firstEdgeUs_ = 0, lastEdgeUs_ = 0;
  uint16_t edges_ = 0;
};

}  // namespace n2
