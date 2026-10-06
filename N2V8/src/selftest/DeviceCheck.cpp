#include "DeviceCheck.h"

#include <stdarg.h>

namespace n2 {

void CheckResult::set(CheckLevel l, const char* fmt, ...) {
  level = l;
  va_list args;
  va_start(args, fmt);
  vsnprintf(text, sizeof text, fmt, args);
  va_end(args);
}

}  // namespace n2
