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
  bool readOxygenPercent(float& percentVol) override { percentVol = 20.9f; return true; }
  const char* errorString() const override { return lastError_; }
private:
  const char* lastError_;
};

struct O2ControllerTestProbe {
  static bool shouldStartScheduledCycle(O2Controller& c, uint32_t nowMs) {
    return c.shouldStartScheduledCycle(nowMs);
  }
  static void beginMeasurementCycle(O2Controller& c) { c.beginMeasurementCycle(); }
  static void finishMeasurementCycle(O2Controller& c, float avg) { c.finishMeasurementCycle(avg); }
  static void failMeasurementCycle(O2Controller& c, const char* err) { c.failMeasurementCycle(err); }
  static void setEarlyMeasurementRequested(O2Controller& c, bool v) { c.earlyMeasurementRequested_ = v; }
  static void setRunningSumPercent(O2Controller& c, float v) { c.runningSumPercent_ = v; }
  static void setSamplesCollected(O2Controller& c, uint8_t v) { c.samplesCollected_ = v; }
  static void setHasValue(O2Controller& c, bool v) { c.hasValue_ = v; }
  static void setCachedAveragePercent(O2Controller& c, float v) { c.cachedAveragePercent_ = v; }
  static void setLastCompletedMeasurementAtMs(O2Controller& c, uint32_t v) { c.lastCompletedMeasurementAtMs_ = v; }
  static void setLastError(O2Controller& c, const char* v) { c.lastError_ = v; }
  static bool earlyMeasurementRequested(const O2Controller& c) { return c.earlyMeasurementRequested_; }
  static float runningSumPercent(const O2Controller& c) { return c.runningSumPercent_; }
  static uint8_t samplesCollected(const O2Controller& c) { return c.samplesCollected_; }
  static bool hasValue(const O2Controller& c) { return c.hasValue_; }
  static float cachedAveragePercent(const O2Controller& c) { return c.cachedAveragePercent_; }
  static uint32_t lastCompletedMeasurementAtMs(const O2Controller& c) { return c.lastCompletedMeasurementAtMs_; }
  static const char* lastError(const O2Controller& c) { return c.lastError_; }
  static const TimedStateMachine& timedStateMachine(const O2Controller& c) { return c.timedStateMachine_; }
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
  config.flushDurationMs = 3U; config.settleDurationMs = 2U;
  config.sampleIntervalMs = 4U; config.sampleCount = 3U;
  config.freshnessThresholdMs = 20U; config.errorBackoffMs = 5U;
  return config;
}

static bool test_WB_shouldStartScheduledCycleBeforeFirstValueAndAfterCachedValue() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  if (!require(O2ControllerTestProbe::shouldStartScheduledCycle(c, 0U), "no value: start")) return false;
  O2ControllerTestProbe::setHasValue(c, true);
  O2ControllerTestProbe::setLastCompletedMeasurementAtMs(c, 100U);
  if (!require(!O2ControllerTestProbe::shouldStartScheduledCycle(c, 149U), "before interval")) return false;
  if (!require(O2ControllerTestProbe::shouldStartScheduledCycle(c, 150U), "at interval")) return false;
  if (!require(O2ControllerTestProbe::shouldStartScheduledCycle(c, 151U), "after interval")) return false;
  return true;
}

static bool test_WB_beginMeasurementCycleClearsAccumulatorsAndOpensFlush() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  clock.setNowMs(42U);
  O2ControllerTestProbe::setEarlyMeasurementRequested(c, true);
  O2ControllerTestProbe::setRunningSumPercent(c, 123.4f);
  O2ControllerTestProbe::setSamplesCollected(c, 7U);
  O2ControllerTestProbe::beginMeasurementCycle(c);
  if (!require(!O2ControllerTestProbe::earlyMeasurementRequested(c), "early cleared")) return false;
  if (!requireNear(O2ControllerTestProbe::runningSumPercent(c), 0.0f, 0.0001f, "sum cleared")) return false;
  if (!require(O2ControllerTestProbe::samplesCollected(c) == 0U, "samples cleared")) return false;
  if (!require(flushValve.isOn(), "valve open")) return false;
  if (!require(c.state() == O2Controller::STATE_FLUSHING, "flushing")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(c).hasDeadline(), "deadline set")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(c).deadlineAtMs() ==
               42U + testConfig().flushDurationMs, "deadline value")) return false;
  return true;
}

static bool test_WB_finishMeasurementCycleUpdatesCacheTimestampFreshnessAndError() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  clock.setNowMs(77U); flushValve.setOn(true);
  O2ControllerTestProbe::setHasValue(c, false);
  O2ControllerTestProbe::setCachedAveragePercent(c, 0.0f);
  O2ControllerTestProbe::setLastCompletedMeasurementAtMs(c, 0U);
  O2ControllerTestProbe::setLastError(c, "old error");
  O2ControllerTestProbe::finishMeasurementCycle(c, 20.5f);
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  if (!require(O2ControllerTestProbe::hasValue(c), "has value")) return false;
  if (!requireNear(O2ControllerTestProbe::cachedAveragePercent(c), 20.5f, 0.0001f, "average")) return false;
  if (!require(O2ControllerTestProbe::lastCompletedMeasurementAtMs(c) == 77U, "timestamp")) return false;
  if (!require(strcmp(O2ControllerTestProbe::lastError(c), "no error") == 0, "no error")) return false;
  if (!require(c.state() == O2Controller::STATE_WAITING_TO_FLUSH, "waiting-to-flush")) return false;
  if (!require(c.isValueFresh(), "fresh")) return false;
  return true;
}

static bool test_WB_failMeasurementCycleClosesFlushPreservesCachedValueAndEntersBackoff() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  clock.setNowMs(88U); flushValve.setOn(true);
  O2ControllerTestProbe::setHasValue(c, true);
  O2ControllerTestProbe::setCachedAveragePercent(c, 19.25f);
  O2ControllerTestProbe::setLastCompletedMeasurementAtMs(c, 40U);
  O2ControllerTestProbe::setLastError(c, "old error");
  O2ControllerTestProbe::failMeasurementCycle(c, "forced read failure");
  if (!require(!flushValve.isOn(), "valve closed")) return false;
  if (!require(O2ControllerTestProbe::hasValue(c), "has value")) return false;
  if (!requireNear(O2ControllerTestProbe::cachedAveragePercent(c), 19.25f, 0.0001f, "preserved")) return false;
  if (!require(O2ControllerTestProbe::lastCompletedMeasurementAtMs(c) == 40U, "timestamp")) return false;
  if (!require(strcmp(O2ControllerTestProbe::lastError(c), "forced read failure") == 0, "error")) return false;
  if (!require(c.state() == O2Controller::STATE_ERROR_BACKOFF, "backoff")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(c).hasDeadline(), "deadline")) return false;
  if (!require(O2ControllerTestProbe::timedStateMachine(c).deadlineAtMs() ==
               88U + testConfig().errorBackoffMs, "deadline value")) return false;
  return true;
}

static bool test_WB_snapshotTimestampMirrorsTimedStateMachineAndZerosMissingValue() {
  FakeClock clock; FakeO2Sensor sensor; FakeBinaryOutput flushValve;
  O2Controller c(clock, sensor, flushValve, testConfig());
  clock.setNowMs(12U); c.init();
  const O2Controller::Snapshot snapshot = c.snapshot();
  if (!require(snapshot.createdAtMs ==
               O2ControllerTestProbe::timedStateMachine(c).stateEnteredAtMs(), "timestamp")) return false;
  if (!require(snapshot.state == O2Controller::STATE_WARMUP, "warmup")) return false;
  if (!require(!snapshot.hasValue, "no value")) return false;
  if (!require(!snapshot.isValueFresh, "not fresh")) return false;
  if (!requireNear(snapshot.o2Percent, 0.0f, 0.0001f, "o2 zero")) return false;
  if (!requireNear(snapshot.n2Percent, 0.0f, 0.0001f, "n2 zero")) return false;
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
