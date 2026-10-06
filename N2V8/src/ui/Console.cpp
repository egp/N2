#include "Console.h"

#include <string.h>

namespace n2 {

const char* Console::levelName(LogLevel level) {
  switch (level) {
    case LogLevel::kError: return "error";
    case LogLevel::kWarn:  return "warn";
    case LogLevel::kInfo:  return "info";
    case LogLevel::kDebug: return "debug";
  }
  return "?";
}

bool Console::parseLevel(const char* text, LogLevel& out) {
  for (uint8_t i = 0; i <= static_cast<uint8_t>(LogLevel::kDebug); ++i) {
    if (strcmp(text, levelName(static_cast<LogLevel>(i))) == 0) {
      out = static_cast<LogLevel>(i);
      return true;
    }
  }
  return false;
}

// Write `len` bytes plus a newline if there is room. Never waits.
bool Console::emit(const char* text, size_t len, bool dropIfNoRoom) {
  if (!hal_.consoleAttached()) return false;
  if (hal_.consoleWriteSpace() < len + 1) {
    if (dropIfNoRoom) ++dropped_;
    return false;
  }
  hal_.consoleWrite(text, len);
  hal_.consoleWrite("\n", 1);
  return true;
}

bool Console::tryPrint(const char* line) { return emit(line, strlen(line), false); }

void Console::write(LogLevel level, const char* line) {
  if (level > level_) return;  // more verbose than the chosen level
  char buf[104];
  static const char kLetter[4] = {'E', 'W', 'I', 'D'};
  const int n = snprintf(buf, sizeof buf, "%c %s", kLetter[static_cast<uint8_t>(level)], line);
  const size_t len = n < 0 ? 0 : (static_cast<size_t>(n) < sizeof buf ? static_cast<size_t>(n) : sizeof buf - 1);
  emit(buf, len, true);
}

void Console::pump() {
  char buf[96];
  while (responder_ != nullptr) {
    if (!responder_->line(index_, buf, sizeof buf)) {
      responder_ = nullptr;
      return;
    }
    const size_t len = strlen(buf);
    if (!emit(buf, len, false)) return;  // no room now: try again next pass (answers are deferred, not dropped)
    ++index_;
  }
}

void Console::poll(CommandHandler& handler) {
  const bool attached = hal_.consoleAttached();
  if (!attached) {  // host gone: forget partial input and any unfinished answer
    responder_ = nullptr;
    reader_.discard();
    wasAttached_ = false;
    return;
  }
  if (!wasAttached_) {
    wasAttached_ = true;
    char line[40];
    snprintf(line, sizeof line, "[console attached]");
    emit(line, strlen(line), false);
  }

  // Read whatever has been typed. Input is never held up by output (CON-2).
  for (int budget = 64; budget > 0; --budget) {
    const int c = hal_.consoleRead();
    if (c < 0) break;
    ++received_;
    const LineReader::Result r = reader_.feed(static_cast<char>(c));
    if (r == LineReader::Result::kOverflow) {
      const char* msg = "E line too long, discarded";
      emit(msg, strlen(msg), false);
    } else if (r == LineReader::Result::kLine) {
      char echo[LineReader::kMaxLength + 4];
      const int n = snprintf(echo, sizeof echo, "> %s", reader_.line());
      if (reader_.line()[0] != '\0') emit(echo, n < 0 ? 0 : static_cast<size_t>(n), false);
      if (hook_ != nullptr) {
        hook_->onLine(reader_.line());  // e.g. the BIST: this line is an answer, not a command
      } else {
        responder_ = handler.handle(parseCommand(reader_.line()));
        index_ = 0;
      }
    }
  }
  pump();
}

}  // namespace n2
