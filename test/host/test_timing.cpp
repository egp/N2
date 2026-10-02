// Tests for Timing.h: GOAL-6 (rollover-safe deadlines).
#include <catch2/catch_test_macros.hpp>
#include <cstdint>

#include "core/Timing.h"

using namespace n2;

TEST_CASE("GOAL-6: deadlineReached in the ordinary case") {
  CHECK_FALSE(deadlineReached(99, 100));
  CHECK(deadlineReached(100, 100));
  CHECK(deadlineReached(101, 100));
}

TEST_CASE("GOAL-6: deadlineReached across the millis() rollover") {
  const uint32_t nearEnd = UINT32_MAX - 4;  // 4,294,967,291
  // Deadline set shortly before rollover, now just after it: reached.
  CHECK(deadlineReached(5, nearEnd));
  // Deadline just after rollover, now still before it: not yet.
  CHECK_FALSE(deadlineReached(nearEnd, 5));
  // A 60 s deadline armed 10 s before rollover.
  const uint32_t armedAt = UINT32_MAX - 10000 + 1;
  const uint32_t deadline = armedAt + 60000;  // wraps
  CHECK_FALSE(deadlineReached(armedAt + 59999, deadline));
  CHECK(deadlineReached(armedAt + 60000, deadline));
  CHECK(deadlineReached(armedAt + 60001, deadline));
}

TEST_CASE("GOAL-6: deadlineReached is usable at compile time") {
  static_assert(deadlineReached(10, 10), "constexpr");
  static_assert(!deadlineReached(9, 10), "constexpr");
  SUCCEED();
}
