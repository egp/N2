// SystemProfile_wifi_scenario.cpp v7
#include "SystemProfile_wifi_scenario.h"

#if defined(ARDUINO_UNOWIFIR4)

#include <Arduino.h>

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

const ScenarioStep kScenario0[] = {
  { 0U, false, 1200U, 350U, 360U, 500U, 900U, true, 20.9f },
  { 200U, true, 1200U, 355U, 365U, 500U, 900U, true, 20.9f },
  { 500U, true, 1200U, 360U, 370U, 2500U, 900U, true, 21.1f },
  { 800U, true, 1200U, 365U, 375U, 2500U, 1300U, true, 21.1f },
  { 1100U, true, 1200U, 370U, 380U, 2500U, 900U, true, 21.2f },
};

const ScenarioStep kScenario1[] = {
  // initial state
  { 0U, false, 0U, 0U, 0U, 0U, 0U, false, 20.9f },

  // TBS on, O2 starts warmup
  { 10, true, 0U, 0U, 0U, 0U, 0U, false, 20.9f },

  // supply < threshold
  { 20, true, 800, 0, 0, 0, 0, false, 20.9f },

  // supply > threshold, left valve on
  { 30, true, 1100, 0, 0, 0, 0, false, 20.9f },

  // left to both, lo n2 rising
  { 90, true, 1200, 0, 0, 1000, 100, false, 20.9f },

  // N2 low > threshold, ssr on
  { 320, true, 1250, 0, 0, 2000, 300, true, 18.8f },

  // N2 hi > threshold, ssr off
  { 400, true, 1250, 0, 0, 2500, 1250, true, 5.5f },

  // n2 hi < threshold, ssr on
  { 430, true, 1250, 0, 0, 2600, 990, true, 2.2f },

};


constexpr uint8_t kScenarioCount =
  static_cast<uint8_t>(sizeof(kScenario1) / sizeof(kScenario1[0]));

float gCurrentO2Percent = 20.9f;
bool gCurrentO2Valid = true;

uint8_t gScenarioIndex = 0U;
bool gScenarioDone = false;
bool gSkipNextLF = false;

const ScenarioStep& currentStep() {
  return kScenario1[gScenarioIndex];
}

void applyCurrentStep(SystemContext& ctx, ProfileClock& clock) {
  const ScenarioStep& step = currentStep();

  clock.setNowMs(step.atMs);
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

void printScenarioLine(const SystemContext& ctx) {
  Serial.print(F("SCEN step "));
  Serial.print(gScenarioIndex);
  Serial.print(F("/"));
  Serial.print(static_cast<int>(kScenarioCount - 1U));
  Serial.print(F(" t="));
  Serial.print(ctx.input.sampledAtMs);
  Serial.print(F(" TBS="));
  Serial.print(ctx.input.blackSwitchEnabled ? 1 : 0);
  Serial.print(F(" air="));
  Serial.print(ctx.input.supplyPsi_x10);
  Serial.print(F(" LT="));
  Serial.print(ctx.input.leftTowerPsi_x10);
  Serial.print(F(" RT="));
  Serial.print(ctx.input.rightTowerPsi_x10);
  Serial.print(F(" lo="));
  Serial.print(ctx.input.lowN2Psi_x100);
  Serial.print(F(" hi="));
  Serial.print(ctx.input.highN2Psi_x10);
  Serial.print(F(" O2="));
  Serial.print(gCurrentO2Valid ? gCurrentO2Percent : -1.0f, 2);
  if (gScenarioDone) {
    Serial.print(F(" Secnario DONE"));
  }
  Serial.println();
}

void resetScenario(ProfileClock& clock, SystemContext& ctx) {
  gScenarioIndex = 0U;
  gScenarioDone = false;
  applyCurrentStep(ctx, clock);
}

void advanceScenario(ProfileClock& clock, SystemContext& ctx) {
  if (gScenarioDone) {
    Serial.println(F("Scenario already DONE. Type r + Enter to reset."));
    return;
  }

  if (gScenarioIndex + 1U < kScenarioCount) {
    ++gScenarioIndex;
    applyCurrentStep(ctx, clock);
    printScenarioLine(ctx);
    return;
  }

  gScenarioDone = true;
  applyCurrentStep(ctx, clock);
  ctx.input.blackSwitchEnabled = false;
  Serial.println(F("Scenario DONE."));
  printScenarioLine(ctx);
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

  config.hardware.i2cAddrLed = 0x2FU;  // TODO Check
  config.hardware.i2cAddrLcd20x4 = 0x27U;
  config.hardware.i2cAddrRtc = 0x68U;
  config.hardware.i2cAddrO2 = 0x74U;      // confirmed by Tom
  config.hardware.i2cAddrRotary = 0x24U;  // TODO Check
  config.hardware.lcdBacklightActiveHigh = true;

  config.display.disp4Brightness = 6U;
  config.display.rotaryOff = 0x44U;
  config.display.rotarySupply = 0x4CU;
  config.display.rotaryLeft = 0x54U;
  config.display.rotaryRight = 0x5CU;
  config.display.rotaryN2Low = 0x64U;
  config.display.rotaryN2Percent = 0x6CU;

  config.pressure.adcBits = 10U;
  config.pressure.analogScaleMax =
    static_cast<int>((1UL << config.pressure.adcBits) - 1UL);
  config.pressure.minPressureReading = config.pressure.analogScaleMax / 10;
  config.pressure.maxPressureReading =
    config.pressure.analogScaleMax - config.pressure.minPressureReading;
  config.pressure.supplyFullScalePsi_x10 = 1500U;
  config.pressure.towerFullScalePsi_x10 = 1500U;
  config.pressure.lowN2FullScalePsi_x100 = 3000U;
  config.pressure.highN2FullScalePsi_x10 = 1500U;

  config.n2.lowOffPsi_x100 = 1000U;
  config.n2.lowOnPsi_x100 = 2000U;
  config.n2.highOnPsi_x10 = 1000U;
  config.n2.highOffPsi_x10 = 1200U;

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
  SystemContext dummy{};
  resetScenario(clock, dummy);

  Serial.println(F("WiFi scenario mode"));
  Serial.println(F("<Enter> = next step"));
  Serial.println(F("r + <Enter> = reset"));
  Serial.println(F("? + <Enter> = help/status"));
}

void systemProfileRefreshInputs(SystemContext& ctx, ProfileClock& clock) {
  applyCurrentStep(ctx, clock);
  if (gScenarioDone) {
    ctx.input.blackSwitchEnabled = false;
  }
}

void systemProfilePrintHelp() {
  Serial.println(F("WiFi scenario mode"));
  Serial.println(F("<Enter> = next step"));
  Serial.println(F("r + <Enter> = reset"));
  Serial.println(F("? + <Enter> = help/status"));
}

void systemProfilePrintStatus(const SystemContext& ctx) {
  printScenarioLine(ctx);
}

bool systemProfileConsumeSerialCommand(ProfileClock& clock, SystemContext& ctx) {
  bool changed = false;
  char command = '\0';

  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());

    if (gSkipNextLF && ch == '\n') {
      gSkipNextLF = false;
      continue;
    }
    gSkipNextLF = false;

    if (ch == '\r' || ch == '\n') {
      if (ch == '\r') {
        gSkipNextLF = true;
      }

      if (command == '\0') {
        advanceScenario(clock, ctx);
        changed = true;
      } else if (command == 'r' || command == 'R') {
        resetScenario(clock, ctx);
        Serial.println(F("Scenario reset."));
        printScenarioLine(ctx);
        changed = true;
      } else if (command == '?') {
        systemProfilePrintHelp();
        printScenarioLine(ctx);
      } else {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
        systemProfilePrintHelp();
      }

      command = '\0';
      continue;
    }

    if (ch == ' ' || ch == '\t') {
      continue;
    }

    command = ch;
  }

  return changed;
}

bool systemProfileIsDone() {
  return gScenarioDone;
}

#endif  // defined(ARDUINO_UNOWIFIR4)

// SystemProfile_wifi_scenario.cpp v7