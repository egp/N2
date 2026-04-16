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
      : beginOk_(true),
        beginError_("begin failed"),
        lastError_("no error"),
        pendingReadFailures_(0U),
        readFailureError_("read failed"),
        nextReadIndex_(0U),
        beginCallCount_(0U),
        readCallCount_(0U) {}

  void setBeginResult(bool ok, const char* error = "begin failed") {
    beginOk_ = ok;
    beginError_ = error;
  }

  void queueRead(float percentVol) { queuedReads_.push_back(percentVol); }

  void queueReads(std::initializer_list<float> values) {
    queuedReads_.insert(queuedReads_.end(), values.begin(), values.end());
  }

  void failNextReads(uint32_t count, const char* error = "read failed") {
    pendingReadFailures_ = count;
    readFailureError_ = error;
  }

  uint32_t beginCallCount() const { return beginCallCount_; }

  uint32_t readCallCount() const { return readCallCount_; }

  bool begin() override {
    ++beginCallCount_;
    if (!beginOk_) {
      lastError_ = beginError_;
      return false;
    }
    lastError_ = "no error";
    return true;
  }

  bool readOxygenPercent(float& percentVol) override {
    ++readCallCount_;

    if (pendingReadFailures_ > 0U) {
      --pendingReadFailures_;
      lastError_ = readFailureError_;
      return false;
    }

    if (nextReadIndex_ >= queuedReads_.size()) {
      lastError_ = "no queued sample";
      return false;
    }

    percentVol = queuedReads_[nextReadIndex_++];
    lastError_ = "no error";
    return true;
  }

  const char* errorString() const override { return lastError_; }

private:
  bool beginOk_;
  const char* beginError_;
  const char* lastError_;
  uint32_t pendingReadFailures_;
  const char* readFailureError_;
  std::vector<float> queuedReads_;
  size_t nextReadIndex_;
  uint32_t beginCallCount_;
  uint32_t readCallCount_;
};

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
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
  config.warmupDurationMs = 10U;
  config.measurementIntervalMs = 50U;
  config.flushDurationMs = 3U;
  config.settleDurationMs = 2U;
  config.sampleIntervalMs = 4U;
  config.sampleCount = 3U;
  config.freshnessThresholdMs = 20U;
  config.errorBackoffMs = 5U;
  return config;
}

static O2Controller::Config oneSampleConfig() {
  O2Controller::Config config = testConfig();
  config.sampleCount = 1U;
  return config;
}

static InputSnapshot dummyInputs() {
  InputSnapshot inputs{};
  inputs.sampledAtMs = 0U;
  inputs.blackSwitchEnabled = true;
  inputs.supplyPsi_x10 = 0U;
  inputs.leftTowerPsi_x10 = 0U;
  inputs.rightTowerPsi_x10 = 0U;
  inputs.lowN2Psi_x100 = 0U;
  inputs.highN2Psi_x10 = 0U;
  return inputs;
}

static bool beginAndReachWaitingToFlush(
    O2Controller& controller,
    FakeClock& clock,
    const O2Controller::Config& config) {
  if (!require(controller.init(), "init() should succeed")) return false;
  if (!require(controller.state() == O2Controller::STATE_WARMUP,
               "init() should enter warmup")) return false;

  clock.advanceMs(config.warmupDurationMs);
  controller.step(dummyInputs());

  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "warmup expiry should enter waiting-to-flush")) return false;
  return true;
}

static bool driveCycleToSampling(
    O2Controller& controller,
    FakeClock& clock,
    const O2Controller::Config& config) {
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_FLUSHING,
               "waiting-to-flush should start flushing")) return false;

  clock.advanceMs(config.flushDurationMs);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_SETTLING,
               "flush expiry should enter settling")) return false;

  clock.advanceMs(config.settleDurationMs);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_SAMPLING,
               "settle expiry should enter sampling")) return false;

  return true;
}

static bool completeSuccessfulCycle(
    O2Controller& controller,
    FakeClock& clock,
    const O2Controller::Config& config) {
  if (!driveCycleToSampling(controller, clock, config)) return false;

  for (uint8_t i = 0U; i < config.sampleCount; ++i) {
    controller.step(dummyInputs());

    if (i + 1U < config.sampleCount) {
      if (!require(controller.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE,
                   "intermediate sample should enter waiting-for-next-sample")) return false;

      clock.advanceMs(config.sampleIntervalMs);
      controller.step(dummyInputs());

      if (!require(controller.state() == O2Controller::STATE_SAMPLING,
                   "sample-interval expiry should re-enter sampling")) return false;
    }
  }

  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "final sample should finish cycle into waiting-to-flush")) return false;
  return true;
}

static bool seedSuccessfulMeasurement(
    O2Controller& controller,
    FakeClock& clock,
    FakeO2Sensor& sensor,
    const O2Controller::Config& config,
    std::initializer_list<float> samples) {
  if (!beginAndReachWaitingToFlush(controller, clock, config)) return false;
  sensor.queueReads(samples);
  return completeSuccessfulCycle(controller, clock, config);
}

static bool test_BB_initStartsWarmupWithClosedFlushValve() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!require(controller.init(), "init() should succeed")) return false;
  if (!require(sensor.beginCallCount() == 1U, "sensor begin should be called exactly once")) return false;
  if (!require(controller.state() == O2Controller::STATE_WARMUP,
               "init() should enter warmup")) return false;
  if (!require(controller.isWarmingUp(), "controller should report warming up")) return false;
  if (!require(!controller.isBusy(), "warmup should not report busy")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed during warmup")) return false;
  if (!require(!controller.hasValue(), "controller should start without a cached value")) return false;
  return true;
}

static bool test_BB_initFailsWhenSampleCountZero() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  O2Controller::Config config = testConfig();
  config.sampleCount = 0U;
  O2Controller controller(clock, sensor, flushValve, config);

  if (!require(!controller.init(), "init() should fail when sampleCount is zero")) return false;
  if (!require(controller.state() == O2Controller::STATE_UNINITIALIZED,
               "failed init should remain uninitialized")) return false;
  if (!require(strcmp(controller.errorString(), "sampleCount must be > 0") == 0,
               "init() should report sampleCount error")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed after failed init")) return false;
  return true;
}

static bool test_BB_initFailsWhenSensorBeginFails() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  sensor.setBeginResult(false, "sensor begin failed");
  O2Controller controller(clock, sensor, flushValve, config);

  if (!require(!controller.init(), "init() should fail when sensor begin fails")) return false;
  if (!require(controller.state() == O2Controller::STATE_UNINITIALIZED,
               "failed sensor init should leave controller uninitialized")) return false;
  if (!require(strcmp(controller.errorString(), "sensor begin failed") == 0,
               "init() should forward sensor error")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed after sensor begin failure")) return false;
  return true;
}

static bool test_BB_fullCycleProducesCachedAverageAndFreshValue() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!seedSuccessfulMeasurement(controller, clock, sensor, config, {20.0f, 21.0f, 22.0f})) return false;

  if (!require(controller.hasValue(), "successful cycle should cache a value")) return false;
  if (!requireNear(controller.averagedPercent(), 21.0f, 0.0001f,
                   "successful cycle should average samples")) return false;
  if (!require(controller.isValueFresh(), "newly completed value should be fresh")) return false;
  if (!require(strcmp(controller.errorString(), "no error") == 0,
               "successful cycle should clear error string")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed after successful cycle")) return false;
  if (!require(!controller.isBusy(), "waiting-to-flush should not report busy")) return false;
  return true;
}

static bool test_BB_staleRequestTriggersEarlyCycle() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!seedSuccessfulMeasurement(controller, clock, sensor, config, {20.0f, 21.0f, 22.0f})) return false;

  clock.advanceMs(config.freshnessThresholdMs + 1U);
  controller.requestMeasurementIfStale();
  controller.step(dummyInputs());

  if (!require(controller.state() == O2Controller::STATE_FLUSHING,
               "stale request should start an early measurement cycle")) return false;
  if (!require(flushValve.isOn(), "flush valve should open for early cycle")) return false;
  return true;
}

static bool test_BB_requestMeasurementIfStaleDoesNothingWhileFresh() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!seedSuccessfulMeasurement(controller, clock, sensor, config, {20.0f, 21.0f, 22.0f})) return false;

  clock.advanceMs(config.freshnessThresholdMs - 1U);
  controller.requestMeasurementIfStale();
  controller.step(dummyInputs());

  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "fresh value should not trigger early measurement")) return false;
  if (!require(!flushValve.isOn(), "flush valve should remain closed while fresh")) return false;
  return true;
}

static bool test_BB_freshnessBoundaryExactThresholdStillFresh() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!seedSuccessfulMeasurement(controller, clock, sensor, config, {20.0f, 21.0f, 22.0f})) return false;

  clock.advanceMs(config.freshnessThresholdMs);
  if (!require(controller.isValueFresh(),
               "value should still be fresh exactly at freshness threshold")) return false;

  clock.advanceMs(1U);
  if (!require(!controller.isValueFresh(),
               "value should become stale immediately after freshness threshold")) return false;
  return true;
}

static bool test_BB_scheduledCycleStartsAutomaticallyAfterMeasurementInterval() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!seedSuccessfulMeasurement(controller, clock, sensor, config, {20.0f, 21.0f, 22.0f})) return false;

  clock.advanceMs(config.measurementIntervalMs - 1U);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "scheduled cycle should not start before interval boundary")) return false;

  clock.advanceMs(1U);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_FLUSHING,
               "scheduled cycle should start at interval boundary")) return false;
  if (!require(flushValve.isOn(), "flush valve should open for scheduled cycle")) return false;
  return true;
}

static bool test_BB_busyStatesAndFlushValveBehaviorAcrossCycle() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!beginAndReachWaitingToFlush(controller, clock, config)) return false;

  sensor.queueReads({20.0f, 21.0f, 22.0f});

  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_FLUSHING,
               "cycle should start in flushing")) return false;
  if (!require(controller.isBusy(), "flushing should report busy")) return false;
  if (!require(flushValve.isOn(), "flush valve should be open in flushing")) return false;

  clock.advanceMs(config.flushDurationMs);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_SETTLING,
               "flush expiry should enter settling")) return false;
  if (!require(controller.isBusy(), "settling should report busy")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed in settling")) return false;

  clock.advanceMs(config.settleDurationMs);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_SAMPLING,
               "settle expiry should enter sampling")) return false;
  if (!require(controller.isBusy(), "sampling should report busy")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed in sampling")) return false;

  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE,
               "intermediate sample should enter waiting-for-next-sample")) return false;
  if (!require(controller.isBusy(), "waiting-for-next-sample should report busy")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed while waiting for next sample")) return false;

  return true;
}

static bool test_BB_oneSampleModeCompletesCorrectly() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = oneSampleConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!beginAndReachWaitingToFlush(controller, clock, config)) return false;

  sensor.queueRead(20.5f);
  if (!driveCycleToSampling(controller, clock, config)) return false;

  controller.step(dummyInputs());

  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "one-sample mode should finish immediately after first sample")) return false;
  if (!require(controller.hasValue(), "one-sample mode should cache a value")) return false;
  if (!requireNear(controller.averagedPercent(), 20.5f, 0.0001f,
                   "one-sample mode should cache the single sample")) return false;
  if (!require(!flushValve.isOn(), "flush valve should be closed after one-sample completion")) return false;
  return true;
}

static bool test_BB_repeatedFailuresPreserveCachedValueStayBoundedAndRecover() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = oneSampleConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!beginAndReachWaitingToFlush(controller, clock, config)) return false;

  sensor.queueRead(20.0f);
  if (!completeSuccessfulCycle(controller, clock, config)) return false;
  if (!requireNear(controller.averagedPercent(), 20.0f, 0.0001f,
                   "seed cycle should establish cached value")) return false;

  sensor.failNextReads(2U, "forced read failure");

  for (uint32_t attempt = 0U; attempt < 2U; ++attempt) {
    clock.advanceMs(config.freshnessThresholdMs + 1U);
    controller.requestMeasurementIfStale();

    if (!driveCycleToSampling(controller, clock, config)) return false;

    controller.step(dummyInputs());
    if (!require(controller.state() == O2Controller::STATE_ERROR_BACKOFF,
                 "failed sample should enter error backoff")) return false;
    if (!requireNear(controller.averagedPercent(), 20.0f, 0.0001f,
                     "failed cycle should preserve cached value")) return false;
    if (!require(controller.hasValue(), "failed cycle should preserve hasValue")) return false;
    if (!require(strcmp(controller.errorString(), "forced read failure") == 0,
                 "failed cycle should preserve sensor error")) return false;
    if (!require(!flushValve.isOn(), "flush valve should be closed during error backoff")) return false;

    clock.advanceMs(config.errorBackoffMs - 1U);
    controller.step(dummyInputs());
    if (!require(controller.state() == O2Controller::STATE_ERROR_BACKOFF,
                 "controller should remain in backoff until deadline")) return false;

    clock.advanceMs(1U);
    controller.step(dummyInputs());
    if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
                 "backoff expiry should return to waiting-to-flush")) return false;
  }

  clock.advanceMs(config.freshnessThresholdMs + 1U);
  controller.requestMeasurementIfStale();
  sensor.queueRead(21.0f);

  if (!driveCycleToSampling(controller, clock, config)) return false;

  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "recovery sample should finish cycle")) return false;
  if (!requireNear(controller.averagedPercent(), 21.0f, 0.0001f,
                   "recovery cycle should update cached value")) return false;
  if (!require(strcmp(controller.errorString(), "no error") == 0,
               "recovery cycle should clear error string")) return false;
  return true;
}

static bool test_BB_staleRequestWhileBusyDoesNotPerturbInFlightCycle() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!beginAndReachWaitingToFlush(controller, clock, config)) return false;

  sensor.queueReads({20.0f, 21.0f, 22.0f});

  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_FLUSHING,
               "precondition: cycle should enter flushing")) return false;

  controller.requestMeasurementIfStale();
  if (!require(controller.state() == O2Controller::STATE_FLUSHING,
               "stale request while busy should not restart or change flushing state")) return false;
  if (!require(flushValve.isOn(),
               "flush valve should remain open while flushing")) return false;

  clock.advanceMs(config.flushDurationMs);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_SETTLING,
               "cycle should continue normally into settling")) return false;

  clock.advanceMs(config.settleDurationMs);
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_SAMPLING,
               "cycle should continue normally into sampling")) return false;

  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE,
               "first sample should enter waiting-for-next-sample")) return false;

  clock.advanceMs(config.sampleIntervalMs);
  controller.step(dummyInputs());
  controller.step(dummyInputs());
  if (!require(controller.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE,
               "second sample should again enter waiting-for-next-sample")) return false;

  clock.advanceMs(config.sampleIntervalMs);
  controller.step(dummyInputs());
  controller.step(dummyInputs());

  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "final sample should finish the in-flight cycle normally")) return false;
  if (!require(controller.hasValue(),
               "completed in-flight cycle should still produce a cached value")) return false;
  if (!requireNear(controller.averagedPercent(), 21.0f, 0.0001f,
                   "busy stale request should not perturb averaging")) return false;
  if (!require(sensor.readCallCount() == 3U,
               "busy stale request should not trigger extra in-flight reads")) return false;
  if (!require(!flushValve.isOn(),
               "flush valve should be closed after normal cycle completion")) return false;

  return true;
}

static bool test_BB_snapshotReflectsCompletedMeasurement() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!seedSuccessfulMeasurement(controller, clock, sensor, config, {20.0f, 21.0f, 22.0f})) return false;

  const O2Controller::Snapshot snapshot = controller.snapshot();

  if (!require(snapshot.createdAtMs == clock.nowMs(),
               "o2 snapshot timestamp should match completion transition time")) return false;
  if (!require(snapshot.state == O2Controller::STATE_WAITING_TO_FLUSH,
               "o2 snapshot should report waiting-to-flush after successful cycle")) return false;
  if (!require(snapshot.hasValue,
               "o2 snapshot should report cached value present")) return false;
  if (!require(snapshot.isValueFresh,
               "o2 snapshot should report fresh value immediately after completion")) return false;
  if (!requireNear(snapshot.o2Percent, 21.0f, 0.0001f,
                   "o2 snapshot should expose cached o2 percent")) return false;
  if (!requireNear(snapshot.n2Percent, 79.0f, 0.0001f,
                   "o2 snapshot should expose derived n2 percent")) return false;
  if (!require(strcmp(snapshot.errorString, "no error") == 0,
               "o2 snapshot should expose no-error state after success")) return false;

  return true;
}

int main() {
  if (!test_BB_initStartsWarmupWithClosedFlushValve()) return 1;
  if (!test_BB_initFailsWhenSampleCountZero()) return 1;
  if (!test_BB_initFailsWhenSensorBeginFails()) return 1;
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
// host_tests/test_BB_o2_controller.cpp v3