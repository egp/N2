// host_tests/test_BB_o2_controller.cpp v3
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <initializer_list>
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
      : beginOk_(true), beginError_("begin failed"), lastError_("no error"),
        pendingReadFailures_(0U), readFailureError_("read failed"),
        nextReadIndex_(0U), beginCallCount_(0U), readCallCount_(0U) {}

  void setBeginResult(bool ok, const char* error = "begin failed") {
    beginOk_ = ok; beginError_ = error;
  }
  void queueRead(float v) { queuedReads_.push_back(v); }
  void queueReads(std::initializer_list<float> values) {
    queuedReads_.insert(queuedReads_.end(), values.begin(), values.end());
  }
  void failNextReads(uint32_t count, const char* error = "read failed") {
    pendingReadFailures_ = count; readFailureError_ = error;
  }
  uint32_t beginCallCount() const { return beginCallCount_; }
  uint32_t readCallCount() const { return readCallCount_; }

  bool begin() override {
    ++beginCallCount_;
    if (!beginOk_) { lastError_ = beginError_; return false; }
    lastError_ = "no error"; return true;
  }
  bool readOxygenPercent(float& percentVol) override {
    ++readCallCount_;
    if (pendingReadFailures_ > 0U) {
      --pendingReadFailures_; lastError_ = readFailureError_; return false;
    }
    if (nextReadIndex_ >= queuedReads_.size()) {
      lastError_ = "no queued sample"; return false;
    }
    percentVol = queuedReads_[nextReadIndex_++]; lastError_ = "no error"; return true;
  }
  const char* errorString() const override { return lastError_; }

private:
  bool beginOk_; const char* beginError_; const char* lastError_;
  uint32_t pendingReadFailures_; const char* readFailureError_;
  std::vector<float> queuedReads_; size_t nextReadIndex_;
  uint32_t beginCallCount_; uint32_t readCallCount_;
};

static bool require(bool condition, const char* message) {
  if (!condition) { printf("FAIL: %s\n", message); return false; }
  return true;
}
static bool requireNear(float actual, float expected, float tolerance, const char* message) {
  if (fabsf(actual - expected) > tolerance) {
    printf("FAIL: %s (actual=%.6f expected=%.6f)\n", message, actual, expected);
    return false;
  }
  return true;
}

static O2Controller::Config testConfig() {
  O2Controller::Config config;
  config.warmupDurationMs = 10U; config.initRetryMs = 20U;
  config.measurementIntervalMs = 50U;
  config.flushDurationMs = 3U;   config.settleDurationMs = 2U;
  config.sampleIntervalMs = 4U;  config.sampleCount = 3U;
  config.freshnessThresholdMs = 20U; config.errorBackoffMs = 5U;
  return config;
}

static O2Controller::Config oneSampleConfig() {
  O2Controller::Config config = testConfig(); config.sampleCount = 1U; return config;
}

static InputSnapshot dummyInputs() {
  InputSnapshot inputs{}; inputs.sampledAtMs = 0U; inputs.blackSwitchEnabled = true;
  return inputs;
}

static bool beginAndReachWaitingToFlush(O2Controller& c, FakeClock& clock,
                                         const O2Controller::Config& config) {
  if (!require(c.init(), "init")) return false;
  if (!require(c.state() == O2Controller::STATE_WARMUP, "warmup")) return false;
  clock.advanceMs(config.warmupDurationMs); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "waiting-to-flush")) return false;
  return true;
}

static bool driveCycleToSampling(O2Controller& c, FakeClock& clock,
                                  const O2Controller::Config& config) {
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "flushing")) return false;
  clock.advanceMs(config.flushDurationMs); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SETTLING, "settling")) return false;
  clock.advanceMs(config.settleDurationMs); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SAMPLING, "sampling")) return false;
  return true;
}

static bool completeSuccessfulCycle(O2Controller& c, FakeClock& clock,
                                     const O2Controller::Config& config) {
  if (!driveCycleToSampling(c, clock, config)) return false;
  for (uint8_t i = 0U; i < config.sampleCount; ++i) {
    c.step(dummyInputs());
    if (i + 1U < config.sampleCount) {
      if (!require(c.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE, "waiting-for-next")) return false;
      clock.advanceMs(config.sampleIntervalMs); c.step(dummyInputs());
      if (!require(c.state() == O2Controller::STATE_SAMPLING, "re-enter sampling")) return false;
    }
  }
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "cycle done")) return false;
  return true;
}

static bool seedSuccessfulMeasurement(O2Controller& c, FakeClock& clock, FakeO2Sensor& sensor,
                                       const O2Controller::Config& config,
                                       std::initializer_list<float> samples) {
  if (!beginAndReachWaitingToFlush(c, clock, config)) return false;
  sensor.queueReads(samples);
  return completeSuccessfulCycle(c, clock, config);
}

static bool test_BB_initStartsWarmupWithClosedFlushValve() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!require(c.init(), "init")) return false;
  if (!require(sensor.beginCallCount() == 1U, "begin called once")) return false;
  if (!require(c.state() == O2Controller::STATE_WARMUP, "warmup")) return false;
  if (!require(c.isWarmingUp(), "isWarmingUp")) return false;
  if (!require(!c.isBusy(), "not busy")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  if (!require(!c.hasValue(), "no value")) return false;
  return true;
}

static bool test_BB_initFailsWhenSampleCountZero() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller::Config config = testConfig(); config.sampleCount = 0U;
  O2Controller c(clock, sensor, flushValve, config);
  if (!require(!c.init(), "init fails")) return false;
  if (!require(c.state() == O2Controller::STATE_UNINITIALIZED, "uninitialized")) return false;
  if (!require(strcmp(c.errorString(), "sampleCount must be > 0") == 0, "error string")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  return true;
}

static bool test_BB_initEntersRetryWhenSensorBeginFails() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  sensor.setBeginResult(false, "sensor begin failed");
  O2Controller c(clock, sensor, flushValve, testConfig());
  // init() now returns true and enters STATE_INIT_RETRY instead of halting
  if (!require(c.init(), "init should return true even when begin fails")) return false;
  if (!require(c.state() == O2Controller::STATE_INIT_RETRY,
               "failed begin should enter init-retry")) return false;
  if (!require(strcmp(c.errorString(), "sensor begin failed") == 0, "error string")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  return true;
}

static bool test_BB_fullCycleProducesCachedAverageAndFreshValue() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!seedSuccessfulMeasurement(c, clock, sensor, testConfig(), {20.0f, 21.0f, 22.0f})) return false;
  if (!require(c.hasValue(), "has value")) return false;
  if (!requireNear(c.averagedPercent(), 21.0f, 0.0001f, "average")) return false;
  if (!require(c.isValueFresh(), "fresh")) return false;
  if (!require(strcmp(c.errorString(), "no error") == 0, "no error")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  if (!require(!c.isBusy(), "not busy")) return false;
  return true;
}

static bool test_BB_staleRequestTriggersEarlyCycle() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!seedSuccessfulMeasurement(c, clock, sensor, testConfig(), {20.0f, 21.0f, 22.0f})) return false;
  clock.advanceMs(testConfig().freshnessThresholdMs + 1U);
  c.requestMeasurementIfStale(); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "flushing")) return false;
  if (!require(flushValve.isOn(), "valve open")) return false;
  return true;
}

static bool test_BB_requestMeasurementIfStaleDoesNothingWhileFresh() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!seedSuccessfulMeasurement(c, clock, sensor, testConfig(), {20.0f, 21.0f, 22.0f})) return false;
  clock.advanceMs(testConfig().freshnessThresholdMs - 1U);
  c.requestMeasurementIfStale(); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "still waiting")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  return true;
}

static bool test_BB_freshnessBoundaryExactThresholdStillFresh() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!seedSuccessfulMeasurement(c, clock, sensor, testConfig(), {20.0f, 21.0f, 22.0f})) return false;
  clock.advanceMs(testConfig().freshnessThresholdMs);
  if (!require(c.isValueFresh(), "still fresh at threshold")) return false;
  clock.advanceMs(1U);
  if (!require(!c.isValueFresh(), "stale after threshold")) return false;
  return true;
}

static bool test_BB_scheduledCycleStartsAutomaticallyAfterMeasurementInterval() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!seedSuccessfulMeasurement(c, clock, sensor, testConfig(), {20.0f, 21.0f, 22.0f})) return false;
  clock.advanceMs(testConfig().measurementIntervalMs - 1U); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "not yet")) return false;
  clock.advanceMs(1U); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "flushing")) return false;
  if (!require(flushValve.isOn(), "valve open")) return false;
  return true;
}

static bool test_BB_busyStatesAndFlushValveBehaviorAcrossCycle() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!beginAndReachWaitingToFlush(c, clock, testConfig())) return false;
  sensor.queueReads({20.0f, 21.0f, 22.0f});
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "flushing")) return false;
  if (!require(c.isBusy(), "busy flushing")) return false;
  if (!require(flushValve.isOn(), "valve open")) return false;
  clock.advanceMs(testConfig().flushDurationMs); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SETTLING, "settling")) return false;
  if (!require(c.isBusy(), "busy settling")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  clock.advanceMs(testConfig().settleDurationMs); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SAMPLING, "sampling")) return false;
  if (!require(c.isBusy(), "busy sampling")) return false;
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE, "waiting-for-next")) return false;
  if (!require(c.isBusy(), "busy waiting-for-next")) return false;
  return true;
}

static bool test_BB_oneSampleModeCompletesCorrectly() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, oneSampleConfig());
  if (!beginAndReachWaitingToFlush(c, clock, oneSampleConfig())) return false;
  sensor.queueRead(20.5f);
  if (!driveCycleToSampling(c, clock, oneSampleConfig())) return false;
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "done")) return false;
  if (!require(c.hasValue(), "has value")) return false;
  if (!requireNear(c.averagedPercent(), 20.5f, 0.0001f, "average")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  return true;
}

static bool test_BB_repeatedFailuresPreserveCachedValueStayBoundedAndRecover() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, oneSampleConfig());
  if (!beginAndReachWaitingToFlush(c, clock, oneSampleConfig())) return false;
  sensor.queueRead(20.0f);
  if (!completeSuccessfulCycle(c, clock, oneSampleConfig())) return false;
  if (!requireNear(c.averagedPercent(), 20.0f, 0.0001f, "seed value")) return false;
  sensor.failNextReads(2U, "forced read failure");
  for (uint32_t attempt = 0U; attempt < 2U; ++attempt) {
    clock.advanceMs(oneSampleConfig().freshnessThresholdMs + 1U);
    c.requestMeasurementIfStale();
    if (!driveCycleToSampling(c, clock, oneSampleConfig())) return false;
    c.step(dummyInputs());
    if (!require(c.state() == O2Controller::STATE_ERROR_BACKOFF, "backoff")) return false;
    if (!requireNear(c.averagedPercent(), 20.0f, 0.0001f, "preserved value")) return false;
    if (!require(c.hasValue(), "has value")) return false;
    if (!require(strcmp(c.errorString(), "forced read failure") == 0, "error")) return false;
    if (!require(!flushValve.isOn(), "valve closed")) return false;
    clock.advanceMs(oneSampleConfig().errorBackoffMs - 1U); c.step(dummyInputs());
    if (!require(c.state() == O2Controller::STATE_ERROR_BACKOFF, "still backoff")) return false;
    clock.advanceMs(1U); c.step(dummyInputs());
    if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "recovered")) return false;
  }
  clock.advanceMs(oneSampleConfig().freshnessThresholdMs + 1U);
  c.requestMeasurementIfStale();
  sensor.queueRead(21.0f);
  if (!driveCycleToSampling(c, clock, oneSampleConfig())) return false;
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "final cycle done")) return false;
  if (!requireNear(c.averagedPercent(), 21.0f, 0.0001f, "updated value")) return false;
  if (!require(strcmp(c.errorString(), "no error") == 0, "no error")) return false;
  return true;
}

static bool test_BB_staleRequestWhileBusyDoesNotPerturbInFlightCycle() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!beginAndReachWaitingToFlush(c, clock, testConfig())) return false;
  sensor.queueReads({20.0f, 21.0f, 22.0f});
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "flushing")) return false;
  c.requestMeasurementIfStale();
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "still flushing")) return false;
  if (!require(flushValve.isOn(), "valve open")) return false;
  clock.advanceMs(testConfig().flushDurationMs); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SETTLING, "settling")) return false;
  clock.advanceMs(testConfig().settleDurationMs); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_SAMPLING, "sampling")) return false;
  c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE, "waiting-for-next")) return false;
  clock.advanceMs(testConfig().sampleIntervalMs); c.step(dummyInputs()); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE, "waiting-for-next 2")) return false;
  clock.advanceMs(testConfig().sampleIntervalMs); c.step(dummyInputs()); c.step(dummyInputs());
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "done")) return false;
  if (!require(c.hasValue(), "has value")) return false;
  if (!requireNear(c.averagedPercent(), 21.0f, 0.0001f, "average")) return false;
  if (!require(sensor.readCallCount() == 3U, "read count")) return false;
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  return true;
}

static bool test_BB_snapshotReflectsCompletedMeasurement() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!seedSuccessfulMeasurement(c, clock, sensor, testConfig(), {20.0f, 21.0f, 22.0f})) return false;
  const O2Controller::Snapshot snapshot = c.snapshot();
  if (!require(snapshot.createdAtMs == clock.nowMs(), "timestamp")) return false;
  if (!require(snapshot.state == O2Controller::STATE_WAITING_TO_FLUSH, "state")) return false;
  if (!require(snapshot.hasValue, "has value")) return false;
  if (!require(snapshot.isValueFresh, "fresh")) return false;
  if (!requireNear(snapshot.o2Percent, 21.0f, 0.0001f, "o2 percent")) return false;
  if (!requireNear(snapshot.n2Percent, 79.0f, 0.0001f, "n2 percent")) return false;
  if (!require(strcmp(snapshot.errorString, "no error") == 0, "no error")) return false;
  return true;
}

int main() {
  if (!test_BB_initStartsWarmupWithClosedFlushValve()) return 1;
  if (!test_BB_initFailsWhenSampleCountZero()) return 1;
  if (!test_BB_initEntersRetryWhenSensorBeginFails()) return 1;
  if (!test_BB_fullCycleProducesCachedAverageAndFreshValue()) return 1;
  if (!test_BB_staleRequestTriggersEarlyCycle()) return 1;
  if (!test_BB_requestMeasurementIfStaleDoesNothingWhileFresh()) return 1;
  if (!test_BB_freshnessBoundaryExactThresholdStillFresh()) return 1;
  if (!test_BB_scheduledCycleStartsAutomaticallyAfterMeasurementInterval()) return 1;
  if (!test_BB_busyStatesAndFlushValveBehaviorAcrossCycle()) return 1;
  if (!test_BB_oneSampleModeCompletesCorrectly()) return 1;
  if (!test_BB_repeatedFailuresPreserveCachedValueStayBoundedAndRecover()) return 1;
  if (!test_BB_staleRequestWhileBusyDoesNotPerturbInFlightCycle()) return 1;
  if (!test_BB_snapshotReflectsCompletedMeasurement()) return 1;
  printf("PASS: test_BB_o2_controller\n");
  return 0;
}
