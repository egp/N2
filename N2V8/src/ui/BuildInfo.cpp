#include "BuildInfo.h"

#include "../BoardPins.h"
#include "../BuildConfig.h"
#include "../Config.h"

namespace n2 {

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
