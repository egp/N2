// Tests for TxBudget: console output pacing on the R4 WiFi (Requirements CON-2, CON-4).
#include <catch2/catch_test_macros.hpp>

#include "core/TxBudget.h"

using namespace n2;

TEST_CASE("CON-4: a fresh budget allows one full burst, then must refill") {
  TxBudget b(11, 128);
  CHECK(b.space(1000) == 128);
  b.consume(100, 1000);
  CHECK(b.space(1000) == 28);
  b.consume(28, 1000);
  CHECK(b.space(1000) == 0);
}

TEST_CASE("CON-4: the budget refills at the line rate (11 bytes per millisecond at 115200 baud)") {
  TxBudget b(11, 128);
  b.space(0);
  b.consume(128, 0);
  CHECK(b.space(0) == 0);
  CHECK(b.space(1) == 11);
  CHECK(b.space(5) == 55);
  CHECK(b.space(100) == 128);  // capped
}

TEST_CASE("CON-4: the cap bounds the longest burst, so one pass never blocks long") {
  TxBudget b(11, 128);
  b.space(0);
  CHECK(b.space(3600000) == 128);  // an hour idle still only allows 128 bytes at once
}

TEST_CASE("CON-4: consuming more than is available leaves zero, never wraps") {
  TxBudget b(11, 128);
  b.consume(1000, 0);
  CHECK(b.space(0) == 0);
}

TEST_CASE("GOAL-6: the budget is correct across the millis() rollover") {
  TxBudget b(11, 128);
  b.space(UINT32_MAX - 4);
  b.consume(128, UINT32_MAX - 4);
  CHECK(b.space(UINT32_MAX - 4) == 0);
  CHECK(b.space(5) == 10 * 11);  // 10 ms later, across the wrap
}

TEST_CASE("CON-4: sustained output at the line rate is never starved or overdrawn") {
  TxBudget b(11, 128);
  uint32_t sent = 0;
  for (uint32_t t = 0; t < 1000; ++t) {
    const uint32_t room = b.space(t);
    b.consume(room, t);
    sent += room;
  }
  CHECK(sent >= 11u * 999u);  // about 11 000 bytes in a second
  CHECK(sent <= 11u * 1000u + 128u);
}
