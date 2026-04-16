// SystemProfile_wifi_scenario.cpp v1
#include "SystemProfile_wifi_scenario.h"

#if defined(ARDUINO_UNOWIFIR4)

namespace {

struct ScenarioStep {
  uint32_t atMs;
  bool blackSwitchEnabled;
  uint16_t supplyPsi_x10;
  uint16_t leftTowerPsi_x10;
  uint16_t rightTowerPsi_x10;
  uint16_t lowN2Psi_x100;
  uint16_t highN2Psi_x10;
  bool o2Valid;
  float o2Percent;
};

constexpr uint32_t kScenarioTickMs = 100U;

const ScenarioStep kScenario[] = {
    // Start disabled with healthy supply and low N2 demand.
    {0U, false, 1200U, 350U, 360U, 500U, 900U, true, 20.9f},
    // Enable system.
    {200U, true, 1200U, 355U, 365U, 500U, 900U, true, 20.9f},
    // Low-side N2 rises enough to permit compressor-off state.
    {500U, true, 1200U, 360U, 370U, 2500U, 900U, true, 21.1f},
    // High-side N2 crosses inhibit threshold.
    {800U, true, 1200U, 365U, 375U, 2500U, 1300U, true, 21.1f},
    // High-side N2 recovers.
    {1100U, true, 1200U, 370U, 380U, 2500U, 900U, true, 21.2f},
};

constexpr uint8_t kScenarioCount =
    static_cast<uint8_t>(sizeof(kScenario) / sizeof(kScenario[0]));

float gCurrentO2Percent = 20.9f;
bool gCurrentO2Valid = true;

const ScenarioStep& activeStepFor(uint32_t nowMs) {
  uint8_t index = 0U;
  while ((index + 1U) < kScenarioCount && kScenario[index + 1U].atMs <= nowMs) {
    ++index;
  }
  return kScenario[index];
}

}  // namespace

ProfileClock::ProfileClock()
    : nowMs_(0U) {
}

uint32_t ProfileClock::nowMs() const {
  return nowMs_;
}

void ProfileClock::setNowMs(uint32_t nowMs) {
  nowMs_ = nowMs;
}

void ProfileClock::advanceMs(uint32_t deltaMs) {
  nowMs_ += deltaMs;
}

ProfileO2Sensor::ProfileO2Sensor(BBI2C& i2c, uint8_t address)
    : lastError_("no error") {
  (void)i2c;
  (void)address;
}

bool ProfileO2Sensor::begin() {
  lastError_ = "no error";
  return true;
}

bool ProfileO2Sensor::readOxygenPercent(float& percentVol) {
  if (!gCurrentO2Valid) {
    lastError_ = "scenario O2 invalid";
    return false;
  }

  percentVol = gCurrentO2Percent;
  lastError_ = "no error";
  return true;
}

const char* ProfileO2Sensor::errorString() const {
  return lastError_;
}

SystemConfig makeSystemConfig() {
  SystemConfig config{};

  config.n2.lowOffPsi_x100 = 1000U;
  config.n2.lowOnPsi_x100 = 2000U;
  config.n2.highOnPsi_x10 = 1000U;
  config.n2.highOffPsi_x10 = 1200U;

  // Short timings for scenario smoke test.
  config.o2.warmupDurationMs = 100U;
  config.o2.measurementIntervalMs = 500U;
  config.o2.flushDurationMs = 50U;
  config.o2.settleDurationMs = 50U;
  config.o2.sampleIntervalMs = 50U;
  config.o2.sampleCount = 2U;
  config.o2.freshnessThresholdMs = 250U;
  config.o2.errorBackoffMs = 100U;

  config.tower.leftOpenMs = 300U;
  config.tower.overlapMs = 100U;
  config.tower.rightOpenMs = 300U;
  config.tower.lowSupplyPsi_x10 = 900U;

  return config;
}

void systemProfileSetup(ProfileClock& clock, ProfileO2Sensor& o2Sensor) {
  (void)o2Sensor;
  clock.setNowMs(0U);
  gCurrentO2Percent = kScenario[0].o2Percent;
  gCurrentO2Valid = kScenario[0].o2Valid;
}

void systemProfileRefreshInputs(
    SystemContext& ctx,
    ProfileClock& clock,
    bool blackSwitchEnabled,
    uint16_t supplyPsi_x10,
    uint16_t leftTowerPsi_x10,
    uint16_t rightTowerPsi_x10,
    uint16_t lowN2Psi_x100,
    uint16_t highN2Psi_x10) {
  (void)blackSwitchEnabled;
  (void)supplyPsi_x10;
  (void)leftTowerPsi_x10;
  (void)rightTowerPsi_x10;
  (void)lowN2Psi_x100;
  (void)highN2Psi_x10;

  clock.advanceMs(kScenarioTickMs);

  const ScenarioStep& step = activeStepFor(clock.nowMs());

  gCurrentO2Percent = step.o2Percent;
  gCurrentO2Valid = step.o2Valid;

  ctx.input.sampledAtMs = clock.nowMs();
  ctx.input.blackSwitchEnabled = step.blackSwitchEnabled;
  ctx.input.supplyPsi_x10 = step.supplyPsi_x10;
  ctx.input.leftTowerPsi_x10 = step.leftTowerPsi_x10;
  ctx.input.rightTowerPsi_x10 = step.rightTowerPsi_x10;
  ctx.input.lowN2Psi_x100 = step.lowN2Psi_x100;
  ctx.input.highN2Psi_x10 = step.highN2Psi_x10;
}

#endif  // defined(ARDUINO_UNOWIFIR4)
// SystemProfile_wifi_scenario.cpp v1