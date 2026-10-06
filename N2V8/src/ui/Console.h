// Console.h — the USB serial console (Requirements §10, CON-1..CON-7).
//
// Two jobs, both non-blocking:
//   * as a LogSink: log lines go out only if a console is attached and the TX buffer has room;
//     otherwise they are dropped (and counted if it was a full buffer, F40). It never waits.
//   * poll(): read typed characters, run complete command lines, and pump any multi-line answer out a few
//     lines at a time as buffer space allows. Input is read on every pass regardless of output state, so
//     commands always get through (CON-2).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../core/Log.h"
#include "../hal/Hal.h"
#include "Command.h"
#include "LineReader.h"

namespace n2 {

// A multi-line answer, produced one line at a time so no big buffer is needed.
class Responder {
 public:
  virtual ~Responder() = default;
  // Write line `index` (0-based) into buf; return false when there are no more lines.
  virtual bool line(uint8_t index, char* buf, size_t cap) = 0;
};

class CommandHandler {
 public:
  virtual ~CommandHandler() = default;
  virtual Responder* handle(const Command& command) = 0;  // nullptr: nothing to print
};

// While set, complete input lines go here instead of to the command handler (the BIST uses this).
class LineHook {
 public:
  virtual ~LineHook() = default;
  virtual void onLine(const char* line) = 0;
};

class Console : public LogSink {
 public:
  Console(Hal& hal, LogLevel level) : hal_(hal), level_(level) {}

  void poll(CommandHandler& handler);

  // LogSink: "<L> <line>\n" where L is E, W, I or D.
  void write(LogLevel level, const char* line) override;

  // Output regardless of log level. tryPrint returns false (and counts nothing) if there is no room right now;
  // the caller keeps the line and tries again on a later pass.
  bool tryPrint(const char* line);

  void setLineHook(LineHook* hook) { hook_ = hook; }

  LogLevel level() const { return level_; }
  void setLevel(LogLevel level) { level_ = level; }
  uint32_t dropped() const { return dropped_; }
  uint32_t received() const { return received_; }  // bytes ever read from the host (0 = never heard from a PC)
  bool attached() const { return hal_.consoleAttached(); }
  bool busy() const { return responder_ != nullptr; }

  static const char* levelName(LogLevel level);
  static bool parseLevel(const char* text, LogLevel& out);

 private:
  bool emit(const char* text, size_t len, bool dropIfNoRoom);
  void pump();

  Hal& hal_;
  LogLevel level_;
  LineReader reader_;
  Responder* responder_ = nullptr;
  LineHook* hook_ = nullptr;
  uint8_t index_ = 0;
  uint32_t dropped_ = 0;
  uint32_t received_ = 0;
  bool wasAttached_ = false;
};

}  // namespace n2
