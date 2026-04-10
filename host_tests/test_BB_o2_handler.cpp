// host_tests/test_BB_o2_handler.cpp v1
#include <stdio.h>

#include "O2Handler.h"

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

class FakeFlushValveDriver : public IFlushValveDriver {
public:
  FakeFlushValveDriver() : open_(false) {}

  void setFlushValveOpen(bool open) override {
    open_ = open;
  }

  bool isOpen() const {
    return open_;
  }

private:
  bool open_;
};

static bool require(bool condition, const char* message) {
  if (!condition) {
    printf("FAIL: %s\n", message);
    return false;
  }
  return true;
}

static O2Handler::Config testConfig() {
  O2Handler::Config config;
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
  FakeFlushValveDriver valve;
  O2Handler handler(clock, sensor, valve, testConfig());

  if (!require(handler.begin(), "begin() should succeed")) return false;
  if (!require(handler.state() == O2Handler::STATE_WARMUP, "begin() should enter warmup")) return false;
  if (!require(handler.isWarmingUp(), "handler should report warming up")) return false;
  if (!require(!valve.isOpen(), "flush valve should be closed during warmup")) return false;
  if (!require(!handler.hasValue(), "handler should not have value immediately after begin")) return false;

  return true;
}

static bool test_BB_fullCycleProducesCachedAverage() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeFlushValveDriver valve;
  O2Handler handler(clock, sensor, valve, testConfig());

  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 22.0f, "");

  if (!require(handler.begin(), "begin() should succeed")) return false;

  clock.advanceMs(100U);
  handler.tick();
  if (!require(handler.state() == O2Handler::STATE_WAITING_TO_FLUSH, "warmup expiry should enter waiting state")) return false;

  handler.tick();
  if (!require(handler.state() == O2Handler::STATE_FLUSHING, "waiting state should start flush cycle")) return false;
  if (!require(valve.isOpen(), "flush valve should open during flush")) return false;

  clock.advanceMs(3000U);
  handler.tick();
  if (!require(handler.state() == O2Handler::STATE_SETTLING, "flush expiry should enter settling")) return false;
  if (!require(!valve.isOpen(), "flush valve should close after flush")) return false;

  clock.advanceMs(500U);
  handler.tick();
  if (!require(handler.state() == O2Handler::STATE_SAMPLING, "settle expiry should enter sampling")) return false;

  handler.tick();
  if (!require(handler.state() == O2Handler::STATE_WAITING_FOR_NEXT_SAMPLE, "first sample should wait for next sample")) return false;

  clock.advanceMs(250U);
  handler.tick();
  handler.tick();
  if (!require(handler.state() == O2Handler::STATE_WAITING_FOR_NEXT_SAMPLE, "second sample should still wait for next sample")) return false;

  clock.advanceMs(250U);
  handler.tick();
  handler.tick();

  if (!require(handler.state() == O2Handler::STATE_WAITING_TO_FLUSH, "third sample should finish cycle")) return false;
  if (!require(handler.hasValue(), "completed cycle should cache a value")) return false;
  if (!require(handler.averagedPercent() > 20.9f && handler.averagedPercent() < 21.1f, "average should be about 21.0")) return false;
  if (!require(handler.isValueFresh(), "newly completed value should be fresh")) return false;
  if (!require(!valve.isOpen(), "flush valve should be closed after completed cycle")) return false;

  return true;
}

static bool test_BB_staleRequestTriggersEarlyCycle() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeFlushValveDriver valve;
  O2Handler handler(clock, sensor, valve, testConfig());

  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 21.0f, "");

  if (!require(handler.begin(), "begin() should succeed")) return false;

  clock.advanceMs(100U);
  handler.tick();
  handler.tick();
  clock.advanceMs(3000U);
  handler.tick();
  clock.advanceMs(500U);
  handler.tick();
  handler.tick();
  clock.advanceMs(250U);
  handler.tick();
  handler.tick();
  clock.advanceMs(250U);
  handler.tick();
  handler.tick();

  if (!require(handler.hasValue(), "first cycle should produce cached value")) return false;

  clock.advanceMs(10001U);
  if (!require(!handler.isValueFresh(), "cached value should become stale")) return false;

  handler.requestMeasurementIfStale();
  handler.tick();

  if (!require(handler.state() == O2Handler::STATE_FLUSHING, "stale request should start early flush cycle")) return false;
  if (!require(valve.isOpen(), "flush valve should open for early cycle")) return false;

  return true;
}

static bool test_BB_failedCyclePreservesLastCachedValueAndBacksOff() {
  FakeClock clock;
  FakeO2Sensor sensor;
  FakeFlushValveDriver valve;
  O2Handler handler(clock, sensor, valve, testConfig());

  sensor.queueSample(true, 20.0f, "");
  sensor.queueSample(true, 21.0f, "");
  sensor.queueSample(true, 22.0f, "");
  sensor.queueSample(false, 0.0f, "sensor read failed");

  if (!require(handler.begin(), "begin() should succeed")) return false;

  clock.advanceMs(100U);
  handler.tick();
  handler.tick();
  clock.advanceMs(3000U);
  handler.tick();
  clock.advanceMs(500U);
  handler.tick();
  handler.tick();
  clock.advanceMs(250U);
  handler.tick();
  handler.tick();
  clock.advanceMs(250U);
  handler.tick();
  handler.tick();

  const float firstAverage = handler.averagedPercent();

  clock.advanceMs(10001U);
  handler.requestMeasurementIfStale();
  handler.tick();
  clock.advanceMs(3000U);
  handler.tick();
  clock.advanceMs(500U);
  handler.tick();
  handler.tick();

  if (!require(handler.state() == O2Handler::STATE_ERROR_BACKOFF, "failed sample should enter error backoff")) return false;
  if (!require(handler.hasValue(), "failed cycle should preserve prior cached value")) return false;
  if (!require(handler.averagedPercent() == firstAverage, "failed cycle should not overwrite cached average")) return false;
  if (!require(!valve.isOpen(), "flush valve should be closed on failure")) return false;

  clock.advanceMs(1000U);
  handler.tick();
  if (!require(handler.state() == O2Handler::STATE_WAITING_TO_FLUSH, "error backoff expiry should return to waiting")) return false;

  return true;
}

int main() {
  if (!test_BB_beginStartsWarmupWithClosedValve()) return 1;
  if (!test_BB_fullCycleProducesCachedAverage()) return 1;
  if (!test_BB_staleRequestTriggersEarlyCycle()) return 1;
  if (!test_BB_failedCyclePreservesLastCachedValueAndBacksOff()) return 1;

  printf("PASS: test_BB_o2_handler\n");
  return 0;
}
// host_tests/test_BB_o2_handler.cpp v1