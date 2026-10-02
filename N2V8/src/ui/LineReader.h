// LineReader.h — non-blocking, bounded console line accumulator (Requirements CON-6).
#pragma once

#include <stdint.h>

namespace n2 {

class LineReader {
 public:
  static constexpr uint8_t kMaxLength = 63;
  enum class Result : uint8_t { kNone, kLine, kOverflow };

  // Feed one received character. kLine: line() holds a complete line (maybe empty).
  // kOverflow: the line was longer than kMaxLength and has been discarded.
  Result feed(char c) {
    if (c == '\r') return Result::kNone;
    if (c == '\n') {
      buf_[len_] = '\0';
      const bool over = overflow_;
      len_ = 0;
      overflow_ = false;
      if (over) return Result::kOverflow;
      ready_ = true;
      return Result::kLine;
    }
    if (len_ < kMaxLength) buf_[len_++] = c;
    else overflow_ = true;
    return Result::kNone;
  }

  const char* line() const { return buf_; }
  void discard() { len_ = 0; overflow_ = false; }

 private:
  char buf_[kMaxLength + 1] = {};
  uint8_t len_ = 0;
  bool overflow_ = false;
  bool ready_ = false;
};

}  // namespace n2
