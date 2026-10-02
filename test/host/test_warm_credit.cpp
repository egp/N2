// Tests for WarmCredit: O2-6a / O2-6b (belt and suspenders across resets).
#include <catch2/catch_test_macros.hpp>

#include "core/WarmCredit.h"

using namespace n2;

namespace {
ResetInfo resetButton() { ResetInfo r; r.known = true; r.powerOn = false; return r; }
ResetInfo powerOn() { ResetInfo r; r.known = true; r.powerOn = true; return r; }
ResetInfo unknownCause() { return ResetInfo(); }
WarmRecord garbage() { WarmRecord r; r.magic = 0xDEADBEEF; r.boots = 7; r.lastSeenMs = 1; r.runMs = 99999; r.check = 3; return r; }

// Simulate a session: boot, tick once a second for `seconds`, leaving the record as it would stay in RAM.
void session(WarmRecord& rec, const ResetInfo& reset, bool enabled, uint32_t seconds, uint32_t* creditOut = nullptr) {
  WarmCredit wc(rec, enabled);
  const uint32_t credit = wc.begin(reset, 100);
  if (creditOut) *creditOut = credit;
  for (uint32_t s = 1; s <= seconds; ++s) wc.tick(100 + s * 1000);
}
}  // namespace

TEST_CASE("O2-6a: a record is valid only with the right magic and checksum") {
  WarmRecord r = garbage();
  CHECK_FALSE(warmRecordValid(r));
  r.magic = kWarmRecordMagic;
  r.check = warmRecordChecksum(r);
  CHECK(warmRecordValid(r));
  r.runMs += 1;
  CHECK_FALSE(warmRecordValid(r));
}

TEST_CASE("O2-6a: first boot (garbage RAM) gives no credit and starts a valid record") {
  WarmRecord rec = garbage();
  uint32_t credit = 123;
  session(rec, powerOn(), true, 0, &credit);
  CHECK(credit == 0);
  CHECK(warmRecordValid(rec));
  CHECK(rec.boots == 1);
}

TEST_CASE("O2-6a: after a reset-button press with a valid record, the previous run time is credited") {
  WarmRecord rec = garbage();
  session(rec, powerOn(), true, 120);              // powered up, ran 2 minutes
  uint32_t credit = 0;
  session(rec, resetButton(), true, 60, &credit);  // reset button, 1 more minute
  CHECK(credit >= 120000);
  CHECK(credit <= 121100);
  CHECK(rec.boots == 2);

  uint32_t credit2 = 0;
  session(rec, resetButton(), true, 0, &credit2);  // another reset
  CHECK(credit2 >= 180000);                        // 120 + 60 s carried
}

TEST_CASE("O2-6a: a power-on reset NEVER credits, even if the RAM record happens to look valid") {
  WarmRecord rec = garbage();
  session(rec, powerOn(), true, 200);
  uint32_t credit = 99;
  session(rec, powerOn(), true, 0, &credit);
  CHECK(credit == 0);
  CHECK(rec.boots == 1);
}

TEST_CASE("O2-6a: an unknown reset cause NEVER credits (if the firmware cannot know, assume cold)") {
  WarmRecord rec = garbage();
  session(rec, powerOn(), true, 200);
  uint32_t credit = 99;
  session(rec, unknownCause(), true, 0, &credit);
  CHECK(credit == 0);
}

TEST_CASE("O2-6a: a corrupted record NEVER credits") {
  WarmRecord rec = garbage();
  session(rec, powerOn(), true, 200);
  rec.check ^= 0xFFFFFFFFu;
  uint32_t credit = 99;
  session(rec, resetButton(), true, 0, &credit);
  CHECK(credit == 0);
}

TEST_CASE("O2-6b: while the feature is disabled the credit is always 0 (but the record is still kept)") {
  WarmRecord rec = garbage();
  session(rec, powerOn(), false, 200);
  uint32_t credit = 99;
  session(rec, resetButton(), false, 0, &credit);
  CHECK(credit == 0);
  CHECK(warmRecordValid(rec));
}

TEST_CASE("O2-6a: restart() (sensor comm failure) removes all carried credit") {
  WarmRecord rec = garbage();
  session(rec, powerOn(), true, 200);
  {
    WarmCredit wc(rec, true);
    wc.begin(resetButton(), 100);
    wc.restart(50000);
    wc.tick(60000);  // 10 s since the restart
  }
  uint32_t credit = 0;
  session(rec, resetButton(), true, 0, &credit);
  CHECK(credit >= 10000);
  CHECK(credit <= 11000);  // only the 10 s since the restart, not the earlier 200 s
}

TEST_CASE("O2-6a: the run time saturates instead of wrapping") {
  WarmRecord rec = garbage();
  WarmCredit first(rec, true);
  first.begin(powerOn(), 0);
  rec.runMs = 0xFFFFFF00u;
  rec.check = warmRecordChecksum(rec);
  WarmCredit wc(rec, true);
  wc.begin(resetButton(), 0);
  wc.tick(5000);
  CHECK(rec.runMs == 0xFFFFFFFFu);
}
