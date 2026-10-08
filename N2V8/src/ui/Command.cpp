#include "Command.h"

#include <string.h>

namespace n2 {

namespace {
struct Entry {
  const char* name;
  CommandId id;
};
const Entry kEntries[] = {
    {"help", CommandId::kHelp},     {"?", CommandId::kHelp},       {"ver", CommandId::kVer},
    {"status", CommandId::kStatus}, {"report", CommandId::kReport}, {"log", CommandId::kLog},
    {"faults", CommandId::kFaults}, {"cfg", CommandId::kCfg},      {"display", CommandId::kDisplay},
    {"loop", CommandId::kLoop},     {"scan", CommandId::kScan},    {"post", CommandId::kPost},
    {"bist", CommandId::kBist},     {"time", CommandId::kTime},
    {"nvm", CommandId::kNvm},       {"debounce", CommandId::kDebounce}, {"pins", CommandId::kPins},
};

char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }
bool space(char c) { return c == ' ' || c == '\t'; }

// Copy the next word into `out` (lower case, truncated); returns the position after it, or nullptr at the end.
const char* nextWord(const char* p, char* out, size_t cap) {
  while (*p != '\0' && space(*p)) ++p;
  if (*p == '\0') return nullptr;
  size_t n = 0;
  while (*p != '\0' && !space(*p)) {
    if (n + 1 < cap) out[n++] = lower(*p);
    ++p;
  }
  out[n] = '\0';
  return p;
}
}  // namespace

Command parseCommand(const char* line) {
  Command c;
  const char* p = nextWord(line, c.name, sizeof c.name);
  if (p == nullptr) return c;  // kNone
  c.id = CommandId::kUnknown;
  for (const Entry& e : kEntries)
    if (strcmp(c.name, e.name) == 0) c.id = e.id;
  while (c.argc < 3) {
    p = nextWord(p, c.arg[c.argc], sizeof c.arg[0]);
    if (p == nullptr) break;
    ++c.argc;
  }
  return c;
}

}  // namespace n2
