// host_tests/test_BB_o2_Controller.cpp v2
#include <stdio.h>

#include "O2Controller.h"

class FakeClock : public IClock {
public:
  FakeClock() : nowMs_(0U) {}

  uint32_t nowMs() const override {
    return nowMs_;
  }

  void setNowMs(uint32_t nowMs) {
    nowMs_ = nowMs;
  }

  void advanceMs(uint32_t deltaMs) {
    nowMs_ += deltaMs;
  }

private:
  uint32_t nowMs_;
};

class FakeO2Sensor : public IO2Sensor {
public:
  struct Sample {
    bool ok;
    float percentVol;
    const char* error;
  };

  FakeO2Sensor()
      : beginOk_(true),
        lastError_("no error"),
        nextSampleIndex_(0U),
        sampleCount_(0U) {}

  bool begin() override {
    if (!beginOk_) {
      lastError_ = "begin failed";
      return false;
    }
    lastError_ = "no error";
    return true;
  }

  bool readOxygenPercent(float& percentVol) override {
    if (nextSampleIndex_ >= sampleCount_) {
      lastError_ = "no queued sample";
      return false;
    }

    const Sample& sample = samples_[nextSampleIndex_++];
    if (!sample.ok) {
      lastError_ = sample.error;
      return false;
    }

    percentVol = sample.percentVol;
    lastError_ = "no error";
    return true;
  }

  const char* errorString() const override {
    return lastError_;
  }

  void setBeginOk(bool beginOk) {
    beginOk_ = beginOk;
  }

  void queueSample(bool ok, float percentVol, const char* error) {
    if (sampleCount_ < kMaxSamples) {
      samples_[sampleCount_].ok = ok;
      samples_[sampleCount_].percentVol = percentVol;
      samples_[sampleCount_].error = error;
      ++sampleCount_;
    }
  }

private:
  static const uint8_t kMaxSamples = 16U;

  bool beginOk_;
  const char* lastError_;
  Sample samples_[kMaxSamples];
  uint8_t nextSampleIndex_;
  uint8_t sampleCount_;
};

class FakeBinaryOutput : public IBinaryOutput {
public:
  FakeBinaryOutput() : on_(false) {}

  void setOn(bool on) override {
    on_ = on;
  }

  bool isOn() const {
    return on_;
  }

private:
  bool on_;
};

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static O2Controller::Config testConfig() {
  O2Controller::Config config;
  config.warmupDurationMs = 100U;
  config.measurementIntervalMs = 60000U;
  config.flushDurationMs = 3000U;
  config.settleDurationMs = 500U;
  config.sampleIntervalMs = 250U;
  config.sampleCount = 3U;
  config.freshnessThresholdMs = 10000U;
  config.errorBackoffMs = 1000U;
  return config;
}

static bool test_BB_beginStartsWarmupWithClosedValve() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput valve;
  O2Controller Controller(clock, sensor, valve, testConfig());

  if (!require(Controller.begin(), "begin() should succeed")) return false;
  if (!require(Controller.state() == O2Controller::STATE_WARMUP, "begin() should enter warmup")) return false;
  if (!require(Controller.isWarmingUp(), "Controller should report warming up")) return false;
  if (!require(!valve.isOn(), "flush valve should be closed during warmup")) return false;
  if (!require(!Controller.hasValue(), "Controller should not have value immediately after begin")) return false;

  return true;
}

static bool test_BB_fullCycleProducesCachedAverage() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput valve;
  O2Controller Controller(clock, sensor, valve, testConfig());

  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 22.0f, "");

  if (!require(Controller.begin(), "begin() should succeed")) return false;

  clock.advanceMs(100U);
  Controller.tick();
  if (!require(Controller.state() == O2Controller::STATE_WAITING_TO_FLUSH, "warmup expiry should enter waiting state")) return false;

  Controller.tick();
  if (!require(Controller.state() == O2Controller::STATE_FLUSHING, "waiting state should start flush cycle")) return false;
  if (!require(valve.isOn(), "flush valve should open during flush")) return false;

  clock.advanceMs(3000U);
  Controller.tick();
  if (!require(Controller.state() == O2Controller::STATE_SETTLING, "flush expiry should enter settling")) return false;
  if (!require(!valve.isOn(), "flush valve should close after flush")) return false;

  clock.advanceMs(500U);
  Controller.tick();
  if (!require(Controller.state() == O2Controller::STATE_SAMPLING, "settle expiry should enter sampling")) return false;

  Controller.tick();
  if (!require(Controller.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE, "first sample should wait for next sample")) return false;

  clock.advanceMs(250U);
  Controller.tick();
  Controller.tick();
  if (!require(Controller.state() == O2Controller::STATE_WAITING_FOR_NEXT_SAMPLE, "second sample should still wait for next sample")) return false;

  clock.advanceMs(250U);
  Controller.tick();
  Controller.tick();

  if (!require(Controller.state() == O2Controller::STATE_WAITING_TO_FLUSH, "third sample should finish cycle")) return false;
  if (!require(Controller.hasValue(), "completed cycle should cache a value")) return false;
  if (!require(Controller.averagedPercent() > 20.9f && Controller.averagedPercent() < 21.1f, "average should be about 21.0")) return false;
  if (!require(Controller.isValueFresh(), "newly completed value should be fresh")) return false;
  if (!require(!valve.isOn(), "flush valve should be closed after completed cycle")) return false;

  return true;
}

static bool test_BB_staleRequestTriggersEarlyCycle() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput valve;
  O2Controller Controller(clock, sensor, valve, testConfig());

  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 21.0f, "");

  if (!require(Controller.begin(), "begin() should succeed")) return false;

  clock.advanceMs(100U);
  Controller.tick();
  Controller.tick();
  clock.advanceMs(3000U);
  Controller.tick();
  clock.advanceMs(500U);
  Controller.tick();
  Controller.tick();
  clock.advanceMs(250U);
  Controller.tick();
  Controller.tick();
  clock.advanceMs(250U);
  Controller.tick();
  Controller.tick();

  if (!require(Controller.hasValue(), "first cycle should produce cached value")) return false;

  clock.advanceMs(10001U);
  if (!require(!Controller.isValueFresh(), "cached value should become stale")) return false;

  Controller.requestMeasurementIfStale();
  Controller.tick();

  if (!require(Controller.state() == O2Controller::STATE_FLUSHING, "stale request should start early flush cycle")) return false;
  if (!require(valve.isOn(), "flush valve should open for early cycle")) return false;

  return true;
}

static bool test_BB_failedCyclePreservesLastCachedValueAndBacksOff() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeBinaryOutput valve;
  O2Controller Controller(clock, sensor, valve, testConfig());

  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 22.0f, "");
  sensor.queueSample(false, 0.0f, "sensor read failed");

  if (!require(Controller.begin(), "begin() should succeed")) return false;

  clock.advanceMs(100U);
  Controller.tick();
  Controller.tick();
  clock.advanceMs(3000U);
  Controller.tick();
  clock.advanceMs(500U);
  Controller.tick();
  Controller.tick();
  clock.advanceMs(250U);
  Controller.tick();
  Controller.tick();
  clock.advanceMs(250U);
  Controller.tick();
  Controller.tick();

  const float firstAverage = Controller.averagedPercent();

  clock.advanceMs(10001U);
  Controller.requestMeasurementIfStale();
  Controller.tick();
  clock.advanceMs(3000U);
  Controller.tick();
  clock.advanceMs(500U);
  Controller.tick();
  Controller.tick();

  if (!require(Controller.state() == O2Controller::STATE_ERROR_BACKOFF, "failed sample should enter error backoff")) return false;
  if (!require(Controller.hasValue(), "failed cycle should preserve prior cached value")) return false;
  if (!require(Controller.averagedPercent() == firstAverage, "failed cycle should not overwrite cached average")) return false;
  if (!require(!valve.isOn(), "flush valve should be closed on failure")) return false;

  clock.advanceMs(1000U);
  Controller.tick();
  if (!require(Controller.state() == O2Controller::STATE_WAITING_TO_FLUSH, "error backoff expiry should return to waiting")) return false;

  return true;
}

int main() {
  if (!test_BB_beginStartsWarmupWithClosedValve()) return 1;
  if (!test_BB_fullCycleProducesCachedAverage()) return 1;
  if (!test_BB_staleRequestTriggersEarlyCycle()) return 1;
  if (!test_BB_failedCyclePreservesLastCachedValueAndBacksOff()) return 1;

  printf("PASS: test_BB_o2_Controller\n");
  return 0;
}
// host_tests/test_BB_o2_Controller.cpp v2