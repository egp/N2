// Log.h — log sink interface and the controller transition line (ARC-7, FLT-2).
//
// Logic code writes lines to a LogSink; the console (device) or a vector
// (host tests) implements it. Nothing here touches hardware.
#pragma once

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

namespace n2 {

enum class LogLevel : uint8_t { kError, kWarn, kInfo, kDebug };

class LogSink {
 public:
  virtual ~LogSink() = default;
  virtual void write(LogLevel level, const char* line) = 0;
};

class NullLogSink : public LogSink {
 public:
  void write(LogLevel, const char*) override {}
};

// printf-style helper; lines are truncated at 95 characters.
inline void logf(LogSink& sink, LogLevel level, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
inline void logf(LogSink& sink, LogLevel level, const char* fmt, ...) {
  char buf[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  sink.write(level, buf);
}

enum class ControllerId : uint8_t { kTower, kCompressor, kO2, kCount };

constexpr const char* controllerName(ControllerId id) {
  return id == ControllerId::kTower ? "TWR" : (id == ControllerId::kCompressor ? "CMP" : "O2");
}

// One line per state transition: "<now>+<delta> <NAME> <from>-><to> next:<deadline|->"
// where delta is the time since that controller's previous transition line.
class TransitionLogger {
 public:
  explicit TransitionLogger(LogSink& sink) : sink_(sink) {}

  void log(ControllerId id, uint32_t now, const char* from, const char* to, bool hasDeadline, uint32_t deadline) {
    const uint8_t i = static_cast<uint8_t>(id);
    const unsigned long delta = static_cast<unsigned long>(now - last_[i]);
    last_[i] = now;
    if (hasDeadline) {
      logf(sink_, LogLevel::kInfo, "%lu+%lu %s %s->%s next:%lu", static_cast<unsigned long>(now), delta,
           controllerName(id), from, to, static_cast<unsigned long>(deadline));
    } else {
      logf(sink_, LogLevel::kInfo, "%lu+%lu %s %s->%s next:-", static_cast<unsigned long>(now), delta,
           controllerName(id), from, to);
    }
  }

 private:
  LogSink& sink_;
  uint32_t last_[static_cast<uint8_t>(ControllerId::kCount)] = {0, 0, 0};
};

}  // namespace n2
