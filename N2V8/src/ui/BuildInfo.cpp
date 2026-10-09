#include "BuildInfo.h"

#include "../BoardPins.h"
#include "../BuildConfig.h"
#include "../Config.h"

namespace n2 {

void lcdVersionTag(const char* version, char* out, size_t n) {
  if (n == 0) return;
  const char* last = version;
  for (const char* p = version; *p != '\0'; ++p)
    if (*p == '.') last = p + 1;
  size_t k = 0;
  if (k + 1 < n) out[k++] = 'v';
  for (; *last != '\0' && k + 1 < n && k < 5; ++last) out[k++] = *last;
  out[k] = '\0';
}

BuildInfo makeBuildInfo(const char* date, const char* time) {
#if defined(N2_BUILD_FIELD)
  const char* mode = "FIELD";
#elif defined(N2_BUILD_BENCH)
  const char* mode = "BENCH";
#elif defined(N2_BUILD_DIAG)
  const char* mode = "DIAG";
#else
  const char* mode = "HOST";
#endif
  return {N2_VERSION, date, time, kBoard.name, mode, kAdcBits};
}

}  // namespace n2
