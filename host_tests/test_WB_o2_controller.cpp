// host_tests/test_WB_o2_controller.cpp v2
#include <math.h>
#include <stdio.h>
#include <string.h>

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
  FakeO2Sensor() : lastError_("no error") {}

  bool begin() override { return true; }

  bool readOxygenPercent(float& percentVol) override {
    percentVol = 20.9f;
    return true;
  }

  const char* errorString() const override { return lastError_; }

private:
  const char* lastError_;
};

struct O2ControllerTestProbe {
  static bool shouldStartScheduledCycle(O2Controller& controller, uint32_t nowMs) {
    return controller.shouldStartScheduledCycle(nowMs);
  }

  static void beginMeasurementCycle(O2Controller& controller) {
    controller.beginMeasurementCycle();
  }

  static void finishMeasurementCycle(O2Controller& controller, float averagePercent) {
    controller.finishMeasurementCycle(averagePercent);
  }

  static void failMeasurementCycle(O2Controller& controller, const char* error) {
    controller.failMeasurementCycle(error);
  }

  static void setEarlyMeasurementRequested(O2Controller& controller, bool value) {
    controller.earlyMeasurementRequested_ = value;
  }

  static void setRunningSumPercent(O2Controller& controller, float value) {
    controller.runningSumPercent_ = value;
  }

  static void setSamplesCollected(O2Controller& controller, uint8_t value) {
    controller.samplesCollected_ = value;
  }

  static void setHasValue(O2Controller& controller, bool value) {
    controller.hasValue_ = value;
  }

  static void setCachedAveragePercent(O2Controller& controller, float value) {
    controller.cachedAveragePercent_ = value;
  }

  static void setLastCompletedMeasurementAtMs(O2Controller& controller, uint32_t value) {
    controller.lastCompletedMeasurementAtMs_ = value;
  }

  static void setLastError(O2Controller& controller, const char* value) {
    controller.lastError_ = value;
  }

  static bool earlyMeasurementRequested(const O2Controller& controller) {
    return controller.earlyMeasurementRequested_;
  }

  static float runningSumPercent(const O2Controller& controller) {
    return controller.runningSumPercent_;
  }

  static uint8_t samplesCollected(const O2Controller& controller) {
    return controller.samplesCollected_;
  }

  static bool hasValue(const O2Controller& controller) {
    return controller.hasValue_;
  }

  static float cachedAveragePercent(const O2Controller& controller) {
    return controller.cachedAveragePercent_;
  }

  static uint32_t lastCompletedMeasurementAtMs(const O2Controller& controller) {
    return controller.lastCompletedMeasurementAtMs_;
  }

  static const char* lastError(const O2Controller& controller) {
    return controller.lastError_;
  }

  static const TimedStateMachine& timedStateMachine(const O2Controller& controller) {
    return controller.timedStateMachine_;
  }
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

static bool test_WB_shouldStartScheduledCycleBeforeFirstValueAndAfterCachedValue() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  if (!require(O2ControllerTestProbe::shouldStartScheduledCycle(controller, 0U),
               "controller without value should start scheduled cycle immediately")) return false;

  O2ControllerTestProbe::setHasValue(controller, true);
  O2ControllerTestProbe::setLastCompletedMeasurementAtMs(controller, 100U);

  if (!require(!O2ControllerTestProbe::shouldStartScheduledCycle(controller, 149U),
               "controller should not schedule before interval elapses")) return false;
  if (!require(O2ControllerTestProbe::shouldStartScheduledCycle(controller, 150U),
               "controller should schedule exactly at interval boundary")) return false;
  if (!require(O2ControllerTestProbe::shouldStartScheduledCycle(controller, 151U),
               "controller should schedule after interval boundary")) return false;

  return true;
}

static bool test_WB_beginMeasurementCycleClearsAccumulatorsAndOpensFlush() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  clock.setNowMs(42U);
  O2ControllerTestProbe::setEarlyMeasurementRequested(controller, true);
  O2ControllerTestProbe::setRunningSumPercent(controller, 123.4f);
  O2ControllerTestProbe::setSamplesCollected(controller, 7U);

  O2ControllerTestProbe::beginMeasurementCycle(controller);

  if (!require(!O2ControllerTestProbe::earlyMeasurementRequested(controller),
               "beginMeasurementCycle should clear early request")) return false;
  if (!requireNear(O2ControllerTestProbe::runningSumPercent(controller), 0.0f, 0.0001f,
                   "beginMeasurementCycle should clear running sum")) return false;
  if (!require(O2ControllerTestProbe::samplesCollected(controller) == 0U,
               "beginMeasurementCycle should clear collected sample count")) return false;
  if (!require(flushValve.isOn(),
               "beginMeasurementCycle should open flush valve")) return false;
  if (!require(controller.state() == O2Controller::STATE_FLUSHING,
               "beginMeasurementCycle should enter flushing")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(controller).hasDeadline(),
               "beginMeasurementCycle should set a deadline")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(controller).deadlineAtMs() ==
               42U + config.flushDurationMs,
               "beginMeasurementCycle should set flush deadline from now")) return false;

  return true;
}

static bool test_WB_finishMeasurementCycleUpdatesCacheTimestampFreshnessAndError() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  clock.setNowMs(77U);
  flushValve.setOn(true);
  O2ControllerTestProbe::setHasValue(controller, false);
  O2ControllerTestProbe::setCachedAveragePercent(controller, 0.0f);
  O2ControllerTestProbe::setLastCompletedMeasurementAtMs(controller, 0U);
  O2ControllerTestProbe::setLastError(controller, "old error");

  O2ControllerTestProbe::finishMeasurementCycle(controller, 20.5f);

  if (!require(!flushValve.isOn(),
               "finishMeasurementCycle should close flush valve")) return false;
  if (!require(O2ControllerTestProbe::hasValue(controller),
               "finishMeasurementCycle should mark value present")) return false;
  if (!requireNear(O2ControllerTestProbe::cachedAveragePercent(controller), 20.5f, 0.0001f,
                   "finishMeasurementCycle should update cached average")) return false;
  if (!require(O2ControllerTestProbe::lastCompletedMeasurementAtMs(controller) == 77U,
               "finishMeasurementCycle should update completion timestamp")) return false;
  if (!require(strcmp(O2ControllerTestProbe::lastError(controller), "no error") == 0,
               "finishMeasurementCycle should clear error string")) return false;
  if (!require(controller.state() == O2Controller::STATE_WAITING_TO_FLUSH,
               "finishMeasurementCycle should return to waiting-to-flush")) return false;
  if (!require(controller.isValueFresh(),
               "newly completed measurement should be fresh")) return false;

  return true;
}

static bool test_WB_failMeasurementCycleClosesFlushPreservesCachedValueAndEntersBackoff() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  clock.setNowMs(88U);
  flushValve.setOn(true);
  O2ControllerTestProbe::setHasValue(controller, true);
  O2ControllerTestProbe::setCachedAveragePercent(controller, 19.25f);
  O2ControllerTestProbe::setLastCompletedMeasurementAtMs(controller, 40U);
  O2ControllerTestProbe::setLastError(controller, "old error");

  O2ControllerTestProbe::failMeasurementCycle(controller, "forced read failure");

  if (!require(!flushValve.isOn(),
               "failMeasurementCycle should close flush valve")) return false;
  if (!require(O2ControllerTestProbe::hasValue(controller),
               "failMeasurementCycle should preserve cached-value flag")) return false;
  if (!requireNear(O2ControllerTestProbe::cachedAveragePercent(controller), 19.25f, 0.0001f,
                   "failMeasurementCycle should preserve cached average")) return false;
  if (!require(O2ControllerTestProbe::lastCompletedMeasurementAtMs(controller) == 40U,
               "failMeasurementCycle should preserve last-completed timestamp")) return false;
  if (!require(strcmp(O2ControllerTestProbe::lastError(controller), "forced read failure") == 0,
               "failMeasurementCycle should store the provided error")) return false;
  if (!require(controller.state() == O2Controller::STATE_ERROR_BACKOFF,
               "failMeasurementCycle should enter error backoff")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(controller).hasDeadline(),
               "failMeasurementCycle should set backoff deadline")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(controller).deadlineAtMs() ==
               88U + config.errorBackoffMs,
               "failMeasurementCycle should set backoff deadline from now")) return false;

  return true;
}

static bool test_WB_snapshotTimestampMirrorsTimedStateMachineAndZerosMissingValue() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput flushValve;
  const O2Controller::Config config = testConfig();
  O2Controller controller(clock, sensor, flushValve, config);

  clock.setNowMs(12U);
  controller.begin();

  const O2Controller::Snapshot snapshot = controller.snapshot();

  if (!require(snapshot.createdAtMs ==
                   O2ControllerTestProbe::timedStateMachine(controller).stateEnteredAtMs(),
               "o2 snapshot timestamp should mirror timed-state-machine entered-at")) return false;
  if (!require(snapshot.state == O2Controller::STATE_WARMUP,
               "o2 snapshot should report warmup after begin")) return false;
  if (!require(!snapshot.hasValue,
               "o2 snapshot should report no cached value before first measurement")) return false;
  if (!require(!snapshot.isValueFresh,
               "o2 snapshot should report not-fresh before first measurement")) return false;
  if (!requireNear(snapshot.o2Percent, 0.0f, 0.0001f,
                   "o2 snapshot should zero o2 percent when no cached value exists")) return false;
  if (!requireNear(snapshot.n2Percent, 0.0f, 0.0001f,
                   "o2 snapshot should zero n2 percent when no cached value exists")) return false;

  return true;
}

int main() {
  if (!test_WB_shouldStartScheduledCycleBeforeFirstValueAndAfterCachedValue()) return 1;
  if (!test_WB_beginMeasurementCycleClearsAccumulatorsAndOpensFlush()) return 1;
  if (!test_WB_finishMeasurementCycleUpdatesCacheTimestampFreshnessAndError()) return 1;
  if (!test_WB_failMeasurementCycleClosesFlushPreservesCachedValueAndEntersBackoff()) return 1;
  if (!test_WB_snapshotTimestampMirrorsTimedStateMachineAndZerosMissingValue()) return 1;

  printf("PASS: test_WB_o2_controller\n");
  return 0;
}
// host_tests/test_WB_o2_controller.cpp v2