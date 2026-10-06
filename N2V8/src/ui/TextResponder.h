// TextResponder.h — a multi-line console answer built up front, then handed to the Console one line at a time.
// Simple and flat: a command handler calls clear(), then add() for each line. (The full firmware's per-command responders
// avoid the buffer; this one favours being easy to read and change on site.)
#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "Console.h"

namespace n2 {

class TextResponder : public Responder {
 public:
  static constexpr uint8_t kMaxLines = 20;
  static constexpr uint8_t kWidth = 84;

  void clear() { count_ = 0; }
  void add(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (count_ >= kMaxLines) return;
    va_list args;
    va_start(args, fmt);
    vsnprintf(lines_[count_], kWidth, fmt, args);
    va_end(args);
    ++count_;
  }
  bool line(uint8_t index, char* buf, size_t cap) override {
    if (index >= count_) return false;
    snprintf(buf, cap, "%s", lines_[index]);
    return true;
  }

 private:
  char lines_[kMaxLines][kWidth] = {};
  uint8_t count_ = 0;
};

}  // namespace n2
