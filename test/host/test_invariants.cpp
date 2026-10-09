// Tests for Invariants: INV-1..INV-4, INV-6, INV-8, INV-9, INV-10 (safety-critical, GOAL-11).
#include <catch2/catch_test_macros.hpp>

#include "TestSupport.h"
#include "core/Invariants.h"

using namespace n2;
using namespace n2::test;

namespace {
Inputs good() {
  Inputs in;
  in.ms = 1000;
  in.tbs = true;
  in.airX10 = kHealthyAirX10;
  in.n2LowX100 = kHealthyN2LowX100;
  in.n2HighX10 = kHealthyN2HighX10;
  in.o2CommOk = true;
  in.o2Warm = true;
  return in;
}
OutputRequest all() { return {true, true, true, true}; }
const ControlConfig cfg = kDefaultControl;
}  // namespace

TEST_CASE("INV: with healthy inputs nothing is forced off and nothing is violated") {
  const InvariantResult r = checkInvariants(good(), cfg, all());
  CHECK_FALSE(r.forced.left);
  CHECK_FALSE(r.forced.right);
  CHECK_FALSE(r.forced.flush);
  CHECK_FALSE(r.forced.ssr);
  CHECK(r.violated == 0);
}

TEST_CASE("INV-1: TBS off forces every output off") {
  Inputs in = good();
  in.tbs = false;
  const InvariantResult r = checkInvariants(in, cfg, all());
  CHECK(r.forced.left);
  CHECK(r.forced.right);
  CHECK(r.forced.flush);
  CHECK(r.forced.ssr);
  CHECK((r.violated & kInv1TbsOff) != 0);
}

TEST_CASE("INV-2: low air (or a faulty air sensor) forces the towers off, not the SSR") {
  for (int variant = 0; variant < 2; ++variant) {
    Inputs in = good();
    if (variant == 0) in.airX10 = cfg.airLowOff - 1;
    else in.airOk = false;
    const InvariantResult r = checkInvariants(in, cfg, all());
    CHECK(r.forced.left);
    CHECK(r.forced.right);
    CHECK_FALSE(r.forced.ssr);
    CHECK_FALSE(r.forced.flush);
    CHECK((r.violated & kInv2Air) != 0);
  }
}

TEST_CASE("INV-2: air exactly at the OFF threshold is still allowed") {
  Inputs in = good();
  in.airX10 = cfg.airLowOff;
  CHECK(checkInvariants(in, cfg, all()).violated == 0);
}

TEST_CASE("INV-3: high N2 (or a faulty N2-high sensor) forces towers AND SSR off") {
  for (int variant = 0; variant < 2; ++variant) {
    Inputs in = good();
    if (variant == 0) in.n2HighX10 = cfg.n2HighOff + 1;
    else in.n2HighOk = false;
    const InvariantResult r = checkInvariants(in, cfg, all());
    CHECK(r.forced.left);
    CHECK(r.forced.right);
    CHECK(r.forced.ssr);
    CHECK_FALSE(r.forced.flush);
    CHECK((r.violated & kInv3N2High) != 0);
  }
}

TEST_CASE("INV-3: N2-high exactly at the OFF threshold is still allowed") {
  Inputs in = good();
  in.n2HighX10 = cfg.n2HighOff;
  CHECK(checkInvariants(in, cfg, all()).violated == 0);
}

TEST_CASE("INV-4: low N2 (or a faulty N2-low sensor) forces the SSR off only") {
  for (int variant = 0; variant < 2; ++variant) {
    Inputs in = good();
    if (variant == 0) in.n2LowX100 = cfg.n2LowOff - 1;
    else in.n2LowOk = false;
    const InvariantResult r = checkInvariants(in, cfg, all());
    CHECK(r.forced.ssr);
    CHECK_FALSE(r.forced.left);
    CHECK_FALSE(r.forced.right);
    CHECK((r.violated & kInv4N2Low) != 0);
  }
}

TEST_CASE("INV-8: F04 acts like both N2 sensors being faulty") {
  Inputs in = good();
  in.sensorOrderFault = true;
  const InvariantResult r = checkInvariants(in, cfg, all());
  CHECK(r.forced.left);
  CHECK(r.forced.right);
  CHECK(r.forced.ssr);
  CHECK((r.violated & kInv8SensorOrder) != 0);
}

TEST_CASE("INV-9: a missing O2 sensor forces all outputs off when the sensor is mandatory") {
  Inputs in = good();
  in.o2CommOk = false;
  const InvariantResult r = checkInvariants(in, cfg, all());
  CHECK(r.forced.left);
  CHECK(r.forced.right);
  CHECK(r.forced.flush);
  CHECK(r.forced.ssr);
  CHECK((r.violated & kInv9O2Missing) != 0);

  ControlConfig bench = cfg;
  bench.o2Mandatory = false;
  CHECK(checkInvariants(in, bench, all()).violated == 0);
}

TEST_CASE("INV-10: while the O2 sensor warms up the towers are forced off, the SSR is not") {
  Inputs in = good();
  in.o2Warm = false;
  const InvariantResult r = checkInvariants(in, cfg, all());
  CHECK(r.forced.left);
  CHECK(r.forced.right);
  CHECK_FALSE(r.forced.ssr);
  CHECK_FALSE(r.forced.flush);
  CHECK((r.violated & kInv10O2Warming) != 0);
}

TEST_CASE("INV-6: a violation is reported only for outputs the request actually wants ON") {
  Inputs in = good();
  in.airX10 = cfg.airLowOff - 1;
  OutputRequest onlySsr{false, false, false, true};
  const InvariantResult r = checkInvariants(in, cfg, onlySsr);
  CHECK(r.violated == 0);   // the tower is forced off, but nobody asked for it
  CHECK(r.forced.left);
}

TEST_CASE("INV: several rules at once combine their forced-off sets and violation bits") {
  Inputs in = good();
  in.airX10 = cfg.airLowOff - 1;
  in.n2LowX100 = cfg.n2LowOff - 1;
  const InvariantResult r = checkInvariants(in, cfg, all());
  CHECK(r.forced.left);
  CHECK(r.forced.right);
  CHECK(r.forced.ssr);
  CHECK((r.violated & kInv2Air) != 0);
  CHECK((r.violated & kInv4N2Low) != 0);
  CHECK((r.violated & kInv3N2High) == 0);
}

TEST_CASE("INV-2 grace: with airGrace set, low air alone does not force the valves closed; a sensor fault still does") {
  Inputs in;
  in.tbs = true; in.o2CommOk = true; in.o2Warm = true;
  in.airX10 = kDefaultControl.airLowOff - 10;
  OutputRequest req; req.left = true;
  CHECK(checkInvariants(in, kDefaultControl, req).forced.left);      // no grace: forced closed
  in.airGrace = true;
  CHECK_FALSE(checkInvariants(in, kDefaultControl, req).forced.left);   // grace: tolerated
  in.airOk = false;
  CHECK(checkInvariants(in, kDefaultControl, req).forced.left);      // a faulty air sensor is never tolerated
}
