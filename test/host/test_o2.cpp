// Tests for O2Controller and WarmupTracker: §7.3, O2-1..O2-7.
#include <catch2/catch_test_macros.hpp>

#include "TestSupport.h"
#include "core/O2Controller.h"
#include "core/WarmupTracker.h"

using namespace n2;
using namespace n2::test;

namespace {
struct Rig {
  VectorLog log;
  TransitionLogger tl{log};
  ControlConfig cfg = quickWarmConfig();
  FakeO2 reader;
  WarmupTracker warmup{cfg.o2WarmupMs};
  O2Controller o2{cfg, reader, warmup, tl};
  uint32_t now = 0;

  Inputs inputs() const {
    Inputs in;
    in.ms = now;
    in.tbs = true;
    in.airX10 = kHealthyAirX10;
    in.n2LowX100 = kHealthyN2LowX100;
    in.n2HighX10 = kHealthyN2HighX10;
    return in;
  }
  void tick(uint32_t ms = 10) { now += ms; o2.update(inputs()); }
  void run(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 10) tick(10); }
  template <typename P> bool until(P p, uint32_t limit) {
    for (uint32_t t = 0; t < limit; t += 10) { if (p()) return true; tick(10); }
    return p();
  }
  using S = O2Controller::State;
};
}  // namespace

TEST_CASE("WarmupTracker: counts down from boot, minus credit, and never goes negative") {
  WarmupTracker w(300000);
  w.begin(1000, 0);
  CHECK(w.remainingMs(1000) == 300000);
  CHECK(w.remainingMs(1000 + 120000) == 180000);
  CHECK_FALSE(w.warm(1000 + 299999));
  CHECK(w.warm(1000 + 300000));
  CHECK(w.remainingMs(1000 + 900000) == 0);

  w.begin(0, 250000);  // credit from before a reset
  CHECK(w.remainingMs(0) == 50000);
  w.begin(0, 999999999);  // credit is capped at the warm-up time
  CHECK(w.warm(0));
  w.restart(5000);
  CHECK(w.remainingMs(5000) == 300000);
}

TEST_CASE("GOAL-6: WarmupTracker survives millis() rollover") {
  WarmupTracker w(60000);
  w.begin(UINT32_MAX - 10000, 0);
  CHECK_FALSE(w.warm(UINT32_MAX - 1));
  CHECK(w.remainingMs(10000) == 60000 - 20001);  // 10 001 ms to the wrap + 10 000 after
  CHECK(w.warm(UINT32_MAX - 10000 + 60000));
}

TEST_CASE("O2-6: after enable the controller finds the sensor, then WARMS UP before sampling") {
  Rig r;
  r.o2.enable(0);
  CHECK(r.o2.state() == Rig::S::kUnknown);
  r.tick(0);
  CHECK(r.o2.commOk());
  CHECK(r.o2.state() == Rig::S::kWarming);
  CHECK_FALSE(r.o2.flushOpen());
  r.run(r.cfg.o2WarmupMs - 100);
  CHECK(r.o2.state() == Rig::S::kWarming);
  CHECK(r.o2.warmRemainingMs(r.now) > 0);
  CHECK_FALSE(r.o2.n2Valid());
  r.run(1000);
  CHECK(r.o2.state() != Rig::S::kWarming);
  CHECK(r.o2.warm(r.now));
}

TEST_CASE("§7.3: a full cycle - flush, 10 samples, average, N2% = 100 - O2%") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);  // already warm
  r.reader.o2x100 = 55;                  // 0.55 % O2
  r.o2.enable(0);
  r.tick(0);
  REQUIRE(r.o2.state() == Rig::S::kFlushing);
  CHECK(r.o2.flushOpen());
  const uint32_t cycleStart = r.now;

  r.run(r.cfg.o2FlushMs - 10);
  CHECK(r.o2.state() == Rig::S::kFlushing);
  r.run(20);
  CHECK(r.o2.state() == Rig::S::kSampling);
  CHECK_FALSE(r.o2.flushOpen());
  CHECK_FALSE(r.o2.n2Valid());

  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kWaiting; }, 5000));
  CHECK(r.reader.readCalls == r.cfg.o2SampleCount);
  CHECK(r.o2.n2Valid());
  CHECK(r.o2.n2PercentX100() == 9945);  // 100.00 - 0.55

  // next cycle starts exactly one interval after the previous cycle START
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kFlushing; }, 70000));
  CHECK(r.now - cycleStart == r.cfg.o2SampleIntervalMs);
}

TEST_CASE("§7.3: the average of the samples is used, rounded to nearest") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.o2.enable(0);
  r.tick(0);
  // Feed alternating readings by changing the fake each tick of the sampling phase.
  uint16_t seq[] = {100, 101};
  int i = 0;
  while (r.o2.state() != Rig::S::kWaiting && r.now < 20000) {
    r.reader.o2x100 = seq[i++ % 2];
    r.tick();
  }
  REQUIRE(r.o2.n2Valid());
  // five 100s and five 101s: average 100.5 -> 101 -> N2 = 10000 - 101
  CHECK(r.o2.n2PercentX100() == 9899);
}

TEST_CASE("§7.3: N2% is clamped to 99.99 and floored at 0") {
  {
    Rig r;
    r.warmup.begin(0, r.cfg.o2WarmupMs);
    r.reader.o2x100 = 1;  // 0.01 % O2: the smallest real reading (exactly 0 is a fault, O2-3a)
    r.o2.enable(0);
    REQUIRE(r.until([&] { return r.o2.n2Valid(); }, 20000));
    CHECK(r.o2.n2PercentX100() == 9999);
  }
  {
    Rig r;
    r.warmup.begin(0, r.cfg.o2WarmupMs);
    r.reader.o2x100 = 10500;  // nonsense above 100 %
    r.o2.enable(0);
    REQUIRE(r.until([&] { return r.o2.n2Valid(); }, 20000));
    CHECK(r.o2.n2PercentX100() == 0);
  }
}

TEST_CASE("O2-4: the flush valve is open only in FLUSHING") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.o2.enable(0);
  for (int i = 0; i < 3000; ++i) {
    r.tick();
    CHECK(r.o2.flushOpen() == (r.o2.state() == Rig::S::kFlushing));
  }
}

TEST_CASE("O2-1/O2-3: no sensor at all -> UNKNOWN retries, then ERROR after the timeout") {
  Rig r;
  r.reader.presentOk = false;
  r.o2.enable(0);
  r.run(r.cfg.o2CommTimeoutMs - 100);
  CHECK(r.o2.state() == Rig::S::kUnknown);
  CHECK(r.reader.beginCalls >= 2);  // it keeps trying
  r.run(300);
  CHECK(r.o2.state() == Rig::S::kError);
  CHECK_FALSE(r.o2.commOk());
  CHECK_FALSE(r.o2.flushOpen());
}

TEST_CASE("O2-3: a failed READ (sensor answers the probe but returns bad data) is an ERROR") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.o2.enable(0);
  r.run(2100);
  REQUIRE(r.o2.state() == Rig::S::kSampling);
  r.reader.readOk = false;
  r.run(300);
  CHECK(r.o2.state() == Rig::S::kError);
  CHECK_FALSE(r.o2.commOk());
  CHECK_FALSE(r.o2.n2Valid());
}

TEST_CASE("O2-2: ERROR is not permanent - it retries via UNKNOWN after o2ErrorRetryMs") {
  Rig r;
  r.reader.presentOk = false;
  r.o2.enable(0);
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kError; }, 10000));
  const uint32_t errorAt = r.now;
  r.reader.presentOk = true;
  REQUIRE(r.until([&] { return r.o2.state() != Rig::S::kError; }, r.cfg.o2ErrorRetryMs + 100));
  CHECK(r.now - errorAt >= r.cfg.o2ErrorRetryMs);
  r.run(100);
  CHECK(r.o2.commOk());
}

TEST_CASE("O2-6: after the sensor was missing it is treated as cold - warm-up restarts when it returns") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);  // pretend we were warm at boot
  r.reader.presentOk = false;
  r.o2.enable(0);
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kError; }, 10000));
  CHECK(r.warmup.warm(r.now));  // not yet restarted: restart happens when it answers again
  r.reader.presentOk = true;
  REQUIRE(r.until([&] { return r.o2.commOk(); }, r.cfg.o2ErrorRetryMs + 2000));
  CHECK_FALSE(r.warmup.warm(r.now));
  CHECK(r.warmup.remainingMs(r.now) > r.cfg.o2WarmupMs - 200);
  CHECK(r.o2.state() == Rig::S::kWarming);
}

TEST_CASE("O2-6: sensor lost while warming up -> ERROR") {
  Rig r;
  r.o2.enable(0);
  r.tick(0);
  REQUIRE(r.o2.state() == Rig::S::kWarming);
  r.reader.presentOk = false;
  r.run(1500);
  CHECK(r.o2.state() == Rig::S::kError);
}

TEST_CASE("O2-7: sampling pauses and N2% is flagged stale when N2 pressures leave the operating range") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.o2.enable(0);
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kWaiting; }, 20000));
  REQUIRE(r.o2.n2Valid());
  const uint16_t last = r.o2.n2PercentX100();

  // Run past the next cycle time with N2-low too low.
  Inputs low = r.inputs();
  low.n2LowX100 = r.cfg.n2LowOff - 1;
  for (uint32_t t = 0; t < r.cfg.o2SampleIntervalMs + 2000; t += 10) {
    r.now += 10;
    low.ms = r.now;
    r.o2.update(low);
  }
  CHECK(r.o2.state() == Rig::S::kWaiting);
  CHECK(r.o2.n2Stale());
  CHECK(r.o2.n2Valid());
  CHECK(r.o2.n2PercentX100() == last);

  // Pressures recover: the cycle starts and the flag clears.
  r.tick();
  CHECK(r.o2.state() == Rig::S::kFlushing);
  CHECK_FALSE(r.o2.n2Stale());
}

TEST_CASE("O2-7: the gate also fails for N2-high at or above its maximum, and for faulty sensors") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.o2.enable(0);
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kWaiting; }, 20000));
  Inputs in = r.inputs();
  in.n2HighX10 = r.cfg.n2HighOff;  // not below the maximum
  for (uint32_t t = 0; t < r.cfg.o2SampleIntervalMs + 500; t += 10) { r.now += 10; in.ms = r.now; r.o2.update(in); }
  CHECK(r.o2.state() == Rig::S::kWaiting);
  in = r.inputs();
  in.n2LowOk = false;
  r.now += 10; in.ms = r.now; r.o2.update(in);
  CHECK(r.o2.state() == Rig::S::kWaiting);
}

TEST_CASE("§7.3: disable() closes the flush valve, forgets the reading and stops") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.o2.enable(0);
  REQUIRE(r.until([&] { return r.o2.flushOpen(); }, 1000));
  r.o2.disable(r.now);
  CHECK(r.o2.state() == Rig::S::kDisabled);
  CHECK_FALSE(r.o2.flushOpen());
  CHECK_FALSE(r.o2.n2Valid());
  CHECK_FALSE(r.o2.commOk());
  r.run(5000);
  CHECK(r.o2.state() == Rig::S::kDisabled);
}

TEST_CASE("ARC-7: O2 transitions are logged with names and deadlines") {
  Rig r;
  r.o2.enable(0);
  r.tick(0);
  CHECK(r.log.has("O2 \?\?->WM"));
  CHECK(r.log.lines.front() == "0+0 O2 OF->?? next:0");
}

TEST_CASE("O2-3a: a reading of exactly zero is a fault, not a perfect nitrogen supply") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.reader.o2x100 = 0;
  r.o2.enable(0);
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kError; }, 20000));
  CHECK_FALSE(r.o2.commOk());
  CHECK_FALSE(r.o2.n2Valid());
}

TEST_CASE("O2-3a: a zero in the middle of a run of good samples also fails the cycle, and the sensor recovers when it reads again") {
  Rig r;
  r.warmup.begin(0, r.cfg.o2WarmupMs);
  r.reader.o2x100 = 150;
  r.o2.enable(0);
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kSampling; }, 20000));
  r.reader.o2x100 = 0;
  REQUIRE(r.until([&] { return r.o2.state() == Rig::S::kError; }, 2000));
  r.reader.o2x100 = 150;
  REQUIRE(r.until([&] { return r.o2.n2Valid(); }, 900000));  // error retry, then a fresh warm-up
  CHECK(r.o2.n2PercentX100() == 9850);  // 1.50 % O2 -> 98.50 % N2
}
