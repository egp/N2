// host_tests/test_BB_o2_init_retry.cpp v1
//
// Tests for O2Controller non-blocking sensor initialization and retry behavior.
//
// These tests require the following changes to O2Controller that do not yet exist:
//
//   O2Controller::State:
//     + STATE_INIT_RETRY          -- waiting to retry sensor.begin()
//
//   O2Controller::Config:
//     + initRetryMs               -- interval between sensor.begin() retries
//
//   O2Controller::init():
//     - Must become non-blocking: if sensor.begin() fails, enter STATE_INIT_RETRY
//       rather than returning false and going to STATE_UNINITIALIZED.
//     - Returns true unconditionally (init is now "started", not "completed").
//
//   O2Controller::step():
//     - In STATE_INIT_RETRY: when timer expires, call sensor.begin() again.
//       On success, enter STATE_WARMUP. On failure, restart the retry timer.
//
//   O2Controller::Snapshot:
//     - No structural changes needed; errorString captures the last begin() error.
//
// Until these changes are made, this file will not compile.
// It is written now to define the required behavior before implementation.

#include <stdio.h>
#include <string.h>
#include <vector>
#include "O2Controller.h"

class FakeClock : public IClock {
public:
  FakeClock() : nowMs_(0U) {}
  uint32_t nowMs() const override { return nowMs_; }
  void setNowMs(uint32_t nowMs) { nowMs_ = nowMs; }
  void advanceMs(uint32_t deltaMs) { nowMs_ += deltaMs; }
private:
  uint32_t nowMs_;
};

class FakeBinaryOutput : public IBinaryOutput {
public:
  FakeBinaryOutput() : on_(false) {}
  void setOn(bool on) override { on_ = on; }
  bool isOn() const { return on_; }
private:
  bool on_;
};

class FakeO2Sensor : public IO2Sensor {
public:
  FakeO2Sensor()
      : lastError_("no error"),
        beginCallCount_(0U),
        beginSucceedOnCall_(0U),   // 0 = always fail; N = succeed on Nth call
        readValue_(20.9f) {}

  // Sensor fails begin() until the Nth call, then succeeds forever after.
  void succeedBeginOnCall(uint32_t n) { beginSucceedOnCall_ = n; }

  void setReadValue(float v) { readValue_ = v; }
  uint32_t beginCallCount() const { return beginCallCount_; }

  bool begin() override {
    ++beginCallCount_;
    if (beginSucceedOnCall_ == 0U || beginCallCount_ < beginSucceedOnCall_) {
      lastError_ = "sensor not ready";
      return false;
    }
    lastError_ = "no error";
    return true;
  }

  bool readOxygenPercent(float& percentVol) override {
    percentVol = readValue_;
    return true;
  }

  const char* errorString() const override { return lastError_; }

private:
  const char* lastError_;
  uint32_t beginCallCount_;
  uint32_t beginSucceedOnCall_;
  float readValue_;
};

static bool require(bool condition, const char* message) {
  if (!condition) { printf("FAIL: %s\n", message); return false; }
  return true;
}

// Config with short timings for fast tests.
// initRetryMs is the new field.
static O2Controller::Config testConfig() {
  O2Controller::Config config;
  config.warmupDurationMs      = 10U;
  config.initRetryMs           = 20U;  // NEW FIELD
  config.measurementIntervalMs = 50U;
  config.flushDurationMs       = 3U;
  config.settleDurationMs      = 2U;
  config.sampleIntervalMs      = 4U;
  config.sampleCount           = 1U;
  config.freshnessThresholdMs  = 20U;
  config.errorBackoffMs        = 5U;
  return config;
}

static InputSnapshot dummyInputs() {
  InputSnapshot inputs{};
  inputs.sampledAtMs        = 0U;
  inputs.blackSwitchEnabled = true;
  return inputs;
}

// -----------------------------------------------------------------------
// init() enters STATE_INIT_RETRY when sensor.begin() fails, not UNINITIALIZED
// -----------------------------------------------------------------------
static bool test_BB_initEntersRetryStateWhenSensorBeginFails() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(0U);  // always fail

  O2Controller c(clock, sensor, flushValve, config);
  const bool result = c.init();

  if (!require(result, "init should return true even when begin fails")) return false;
  if (!require(c.state() == O2Controller::STATE_INIT_RETRY,
               "failed begin should enter init-retry, not uninitialized")) return false;
  if (!require(!flushValve.isOn(),
               "flush valve should be closed during init retry")) return false;
  if (!require(sensor.beginCallCount() == 1U,
               "begin should be called exactly once during init")) return false;
  return true;
}

// -----------------------------------------------------------------------
// In STATE_INIT_RETRY, sensor.begin() is not called again before the timer
// -----------------------------------------------------------------------
static bool test_BB_initRetryDoesNotCallBeginBeforeInterval() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(0U);  // always fail

  O2Controller c(clock, sensor, flushValve, config);
  c.init();

  if (!require(c.state() == O2Controller::STATE_INIT_RETRY, "precondition")) return false;

  clock.advanceMs(config.initRetryMs - 1U);
  c.step(dummyInputs());

  if (!require(c.state() == O2Controller::STATE_INIT_RETRY,
               "should remain in retry before interval elapses")) return false;
  if (!require(sensor.beginCallCount() == 1U,
               "begin should not be called again before retry interval")) return false;
  return true;
}

// -----------------------------------------------------------------------
// In STATE_INIT_RETRY, sensor.begin() is retried when timer expires
// -----------------------------------------------------------------------
static bool test_BB_initRetryCallsBeginAgainAfterInterval() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(0U);  // always fail

  O2Controller c(clock, sensor, flushValve, config);
  c.init();

  clock.advanceMs(config.initRetryMs);
  c.step(dummyInputs());

  if (!require(sensor.beginCallCount() == 2U,
               "begin should be retried after interval")) return false;
  if (!require(c.state() == O2Controller::STATE_INIT_RETRY,
               "still-failing begin should stay in retry")) return false;
  return true;
}

// -----------------------------------------------------------------------
// Retry timer resets after each failed attempt
// -----------------------------------------------------------------------
static bool test_BB_initRetryTimerResetsAfterEachFailedAttempt() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(0U);  // always fail

  O2Controller c(clock, sensor, flushValve, config);
  c.init();

  // First retry
  clock.advanceMs(config.initRetryMs);
  c.step(dummyInputs());
  if (!require(sensor.beginCallCount() == 2U, "first retry")) return false;

  // Just before second retry should not trigger
  clock.advanceMs(config.initRetryMs - 1U);
  c.step(dummyInputs());
  if (!require(sensor.beginCallCount() == 2U,
               "begin should not be called before second retry interval")) return false;

  // At second retry interval
  clock.advanceMs(1U);
  c.step(dummyInputs());
  if (!require(sensor.beginCallCount() == 3U, "second retry")) return false;
  if (!require(c.state() == O2Controller::STATE_INIT_RETRY, "still retrying")) return false;
  return true;
}

// -----------------------------------------------------------------------
// Successful retry enters WARMUP, not WAITING_TO_FLUSH
// -----------------------------------------------------------------------
static bool test_BB_initRetrySuccessEntersWarmup() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(2U);  // fail first call, succeed on second

  O2Controller c(clock, sensor, flushValve, config);
  c.init();

  if (!require(c.state() == O2Controller::STATE_INIT_RETRY,
               "precondition: first begin fails")) return false;

  clock.advanceMs(config.initRetryMs);
  c.step(dummyInputs());

  if (!require(sensor.beginCallCount() == 2U, "second begin called")) return false;
  if (!require(c.state() == O2Controller::STATE_WARMUP,
               "successful retry should enter warmup")) return false;
  if (!require(!flushValve.isOn(),
               "flush valve should be closed after successful retry")) return false;
  return true;
}

// -----------------------------------------------------------------------
// After successful retry, warmup and full measurement cycle complete normally
// -----------------------------------------------------------------------
static bool test_BB_initRetryThenWarmupThenFullCycleCompletesNormally() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(2U);  // fail first, succeed second
  sensor.setReadValue(20.5f);

  O2Controller c(clock, sensor, flushValve, config);
  c.init();

  // Retry fires, begin succeeds, enter warmup
  clock.advanceMs(config.initRetryMs);
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WARMUP, "warmup")) return false;

  // Warmup expires
  clock.advanceMs(config.warmupDurationMs);
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "waiting-to-flush after warmup")) return false;

  // Measurement cycle: flush
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "flushing")) return false;

  clock.advanceMs(config.flushDurationMs);
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SETTLING, "settling")) return false;

  clock.advanceMs(config.settleDurationMs);
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SAMPLING, "sampling")) return false;

  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "cycle complete")) return false;
  if (!require(c.hasValue(), "has value")) return false;
  if (!require(c.averagedPercent() > 0.0f, "non-zero value")) return false;

  return true;
}

// -----------------------------------------------------------------------
// STATE_INIT_RETRY reports no cached value and flush valve stays closed
// -----------------------------------------------------------------------
static bool test_BB_initRetryStateReportsNoValueAndValveClosed() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(0U);  // always fail

  O2Controller c(clock, sensor, flushValve, config);
  c.init();

  clock.advanceMs(config.initRetryMs / 2U);
  c.step(dummyInputs());

  if (!require(c.state() == O2Controller::STATE_INIT_RETRY, "in retry")) return false;
  if (!require(!c.hasValue(),
               "init-retry should report no cached value")) return false;
  if (!require(!flushValve.isOn(),
               "flush valve must stay closed during init retry")) return false;

  const O2Controller::Snapshot snapshot = c.snapshot();
  if (!require(!snapshot.hasValue,
               "snapshot should report no value during init retry")) return false;
  if (!require(snapshot.n2Percent == 0.0f,
               "snapshot n2Percent should be zero during init retry")) return false;

  return true;
}

// -----------------------------------------------------------------------
// Multiple retries then success: begin call count matches retry count
// -----------------------------------------------------------------------
static bool test_BB_multipleRetriesThenSuccess() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(4U);  // fail 3 times, succeed on 4th

  O2Controller c(clock, sensor, flushValve, config);
  c.init();  // call 1: fail

  for (uint32_t attempt = 0U; attempt < 2U; ++attempt) {
    clock.advanceMs(config.initRetryMs);
    c.step(dummyInputs());
    if (!require(c.state() == O2Controller::STATE_INIT_RETRY,
                 "should still be retrying")) return false;
  }
  // 3 begin calls so far, all failed

  clock.advanceMs(config.initRetryMs);
  c.step(dummyInputs());  // call 4: succeed

  if (!require(sensor.beginCallCount() == 4U,
               "begin should have been called 4 times total")) return false;
  if (!require(c.state() == O2Controller::STATE_WARMUP,
               "fourth attempt should succeed and enter warmup")) return false;

  return true;
}

// -----------------------------------------------------------------------
// shutdown() from STATE_INIT_RETRY resets cleanly
// -----------------------------------------------------------------------
static bool test_BB_shutdownFromInitRetryResetsCleanly() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();

  sensor.succeedBeginOnCall(0U);  // always fail

  O2Controller c(clock, sensor, flushValve, config);
  c.init();

  if (!require(c.state() == O2Controller::STATE_INIT_RETRY, "precondition")) return false;

  c.shutdown();

  if (!require(c.state() == O2Controller::STATE_UNINITIALIZED,
               "shutdown should return to uninitialized")) return false;
  if (!require(!flushValve.isOn(),
               "flush valve should be closed after shutdown")) return false;

  return true;
}

int main() {
  if (!test_BB_initEntersRetryStateWhenSensorBeginFails()) return 1;
  if (!test_BB_initRetryDoesNotCallBeginBeforeInterval()) return 1;
  if (!test_BB_initRetryCallsBeginAgainAfterInterval()) return 1;
  if (!test_BB_initRetryTimerResetsAfterEachFailedAttempt()) return 1;
  if (!test_BB_initRetrySuccessEntersWarmup()) return 1;
  if (!test_BB_initRetryThenWarmupThenFullCycleCompletesNormally()) return 1;
  if (!test_BB_initRetryStateReportsNoValueAndValveClosed()) return 1;
  if (!test_BB_multipleRetriesThenSuccess()) return 1;
  if (!test_BB_shutdownFromInitRetryResetsCleanly()) return 1;

  printf("PASS: test_BB_o2_init_retry\n");
  return 0;
}
// host_tests/test_BB_o2_init_retry.cpp v1
