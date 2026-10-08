// Command.h — console command parsing (Requirements §10).
#pragma once

#include <stdint.h>

namespace n2 {

enum class CommandId : uint8_t {
  kNone,  // empty line
  kHelp, kVer, kStatus, kReport, kLog, kFaults, kCfg, kDisplay, kLoop, kScan, kPost, kBist, kSim, kTime, kNvm, kDebounce,
  kUnknown
};

struct Command {
  CommandId id = CommandId::kNone;
  char name[16] = {};   // the word as typed, lower case (for error messages)
  char arg[3][20] = {};
  uint8_t argc = 0;
};

// Case-insensitive; words separated by spaces or tabs; at most three arguments are kept.
Command parseCommand(const char* line);

}  // namespace n2
