/*!
 * @file N2V5.ino
 * @brief Arduino controller for the N2 Generator
 * @version V5.4
 * @date 2026-06-07
 */

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_MultiGasSensor.h>  // SEN0465 O2 sensor  https://github.com/DFRobot/DFRobot_MultiGasSensor

#include "src/TM1650min.h"   // 4-digit 7-segment display
#include "src/LCD20x4min.h"  // 20x4 LCD
#include "src/TimedStateMachine.h"
#include "src/InputSnapshot.h"
#include "src/SystemConfig.h"
#include "src/TowerController.h"
#include "src/O2Controller.h"
#include "src/N2Controller.h"
#include "src/BinaryOutput.h"
#include "src/PinAssignments.h"

/* =========================================================
   ArduinoSysClock — wraps millis() for IClock interface
   Named sysClock (not clock) to avoid conflict with time.h clock()
   ========================================================= */
class ArduinoSysClock : public IClock {
public:
  uint32_t nowMs() const override {
    return millis();
  }
};

/* =========================================================
   ArduinoDigitalOutput — wraps digitalWrite() for IBinaryOutput
   ========================================================= */
class ArduinoDigitalOutput : public IBinaryOutput {
public:
  explicit ArduinoDigitalOutput(uint8_t pin)
    : pin_(pin) {}
  void begin(bool initialState = false) {
    pinMode(pin_, OUTPUT);
    setOn(initialState);
  }
  void setOn(bool on) override {
    digitalWrite(pin_, on ? HIGH : LOW);
  }
private:
  uint8_t pin_;
};

/* =========================================================
   DFRobotO2Sensor — IO2Sensor implementation using DFRobot_MultiGasSensor
   Target: SEN0465 Gravity Factory-Calibrated Electrochemical O2 (0-25 %vol, I2C)
   Notes:
   - readGasConcentrationPPM() returns %vol (not PPM) for the SEN0465
   - PASSIVITY mode: controller polls sensor; sensor does not push data
   - Temperature compensation improves accuracy; enabled in begin()
   - Factory-calibrated; no user calibration required
   ========================================================= */
class DFRobotO2Sensor : public IO2Sensor {
public:
  explicit DFRobotO2Sensor(uint8_t i2cAddr)
    : sensor_(&Wire, i2cAddr), lastError_("no error") {}

  bool begin() override {
    if (!sensor_.begin()) {
      lastError_ = "sensor begin failed";
      return false;
    }
    sensor_.changeAcquireMode(sensor_.PASSIVITY);
    sensor_.setTempCompensation(sensor_.ON);
    lastError_ = "no error";
    return true;
  }

  bool readOxygenPercent(float& percentVol) override {
    // readGasConcentrationPPM() returns %vol for SEN0465 despite the name
    const float reading = sensor_.readGasConcentrationPPM();
    if (reading == 0.0f) {
      lastError_ = "sensor read returned zero";
      return false;
    }
    percentVol = reading;
    lastError_ = "no error";
    return true;
  }

  const char* errorString() const override {
    return lastError_;
  }

private:
  DFRobot_GAS_I2C sensor_;
  const char* lastError_;
};

/* =========================================================
   Configuration — all values are best-guess defaults.
   Adjust after production testing. Nothing is hardcoded.
   ========================================================= */
static SystemConfig makeConfig() {
  SystemConfig cfg{};

  cfg.hardware.i2cAddrLed = I2C_ADDR_DISP4;
  cfg.hardware.i2cAddrLcd20x4 = I2C_ADDR_LCD;
  cfg.hardware.i2cAddrO2 = I2C_ADDR_O2;
  cfg.hardware.lcdBacklightActiveHigh = true;

  cfg.display.disp4Brightness = 7U;

  const uint8_t adcBits = 10U;
  cfg.pressure.adcBits = adcBits;
  cfg.pressure.analogScaleMax = (1 << adcBits) - 1;                    // 1023
  cfg.pressure.minPressureReading = cfg.pressure.analogScaleMax / 10;  // ~102
  cfg.pressure.maxPressureReading = cfg.pressure.analogScaleMax
                                    - cfg.pressure.minPressureReading;  // ~921
  cfg.pressure.supplyFullScalePsi_x10 = 1500U;                          // 150.0 PSI
  cfg.pressure.towerFullScalePsi_x10 = 1500U;                           // 150.0 PSI
  cfg.pressure.lowN2FullScalePsi_x100 = 3000U;                          //  30.00 PSI
  cfg.pressure.highN2FullScalePsi_x10 = 1500U;                          // 150.0 PSI


  cfg.tower.overlapMs = 750UL;
  cfg.tower.towerOpenMs = 60000UL - cfg.tower.overlapMs;  // ~59 s
  cfg.tower.airSupplyOnPsi_x10 = 900U;                    // 90.0 PSI
  cfg.tower.airSupplyOffPsi_x10 = 700U;                   // 70.0 PSI

  cfg.o2.warmupDurationMs = 300000UL;      // 5 min
  cfg.o2.initRetryMs = 60000UL;            // 1 min
  cfg.o2.measurementIntervalMs = 60000UL;  // 1 min
  cfg.o2.flushDurationMs = 3000UL;
  cfg.o2.settleDurationMs = 2000UL;
  cfg.o2.sampleIntervalMs = 250U;
  cfg.o2.sampleCount = 10U;
  cfg.o2.freshnessThresholdMs = 90000UL;  // 90 s
  cfg.o2.errorBackoffMs = 5000UL;

  cfg.n2.lowOffPsi_x100 = 1000U;  // 10.00 PSI
  cfg.n2.lowOnPsi_x100 = 2000U;   // 20.00 PSI
  cfg.n2.highOnPsi_x10 = 1000U;   // 100.0 PSI
  cfg.n2.highOffPsi_x10 = 1200U;  // 120.0 PSI

  return cfg;
}

/* =========================================================
   Globals
   ========================================================= */
static const SystemConfig config = makeConfig();

static ArduinoSysClock sysClock;
static ArduinoDigitalOutput leftValve(LEFT_TOWER_VALVE_PIN);
static ArduinoDigitalOutput rightValve(RIGHT_TOWER_VALVE_PIN);
static ArduinoDigitalOutput flushValve(O2_FLUSH_VALVE_PIN);
static ArduinoDigitalOutput compressorSsr(SSR_PIN);
static DFRobotO2Sensor o2Sensor(I2C_ADDR_O2);

static TowerController towerController(sysClock, leftValve, rightValve, config);
static O2Controller o2Controller(sysClock, o2Sensor, flushValve, config);
static N2Controller n2Controller(sysClock, compressorSsr, config);

static TM1650 disp4(I2C_ADDR_DISP4);
static LCD20x4_I2C disp20x4(I2C_ADDR_LCD);

static InputSnapshot input{};
static bool systemWasEnabled = false;

/* =========================================================
   Pressure sensor scaling
   ========================================================= */
static uint16_t scalePressure(int raw, uint16_t fullScale) {
  const int clamped = constrain(raw,
                                config.pressure.minPressureReading,
                                config.pressure.maxPressureReading);
  return static_cast<uint16_t>(
    map(clamped,
        config.pressure.minPressureReading,
        config.pressure.maxPressureReading,
        0,
        static_cast<int>(fullScale)));
}

/* =========================================================
   Read all inputs into InputSnapshot
   ========================================================= */
static void readInputs() {
  input.sampledAtMs = sysClock.nowMs();
  input.blackSwitchEnabled = (digitalRead(BLACK_SWITCH_PIN) == LOW);
  input.supplyPsi_x10 = scalePressure(analogRead(SUPPLY_PRESSURE_PIN),
                                      config.pressure.supplyFullScalePsi_x10);
  input.leftTowerPsi_x10 = scalePressure(analogRead(LEFT_TOWER_PIN),
                                         config.pressure.towerFullScalePsi_x10);
  input.rightTowerPsi_x10 = scalePressure(analogRead(RIGHT_TOWER_PIN),
                                          config.pressure.towerFullScalePsi_x10);
  input.lowN2Psi_x100 = scalePressure(analogRead(LOW_N2_PIN),
                                      config.pressure.lowN2FullScalePsi_x100);
  input.highN2Psi_x10 = scalePressure(analogRead(HIGH_N2_PIN),
                                      config.pressure.highN2FullScalePsi_x10);
}

/* =========================================================
   Display formatting helpers
   ========================================================= */
static void formatFixed10(char* buf, size_t sz, uint16_t value_x10) {
  snprintf(buf, sz, "%u.%u", value_x10 / 10U, value_x10 % 10U);
}

static void formatFixed100(char* buf, size_t sz, uint16_t value_x100) {
  snprintf(buf, sz, "%u.%02u", value_x100 / 100U, value_x100 % 100U);
}

/* =========================================================
   4-digit display: always shows N2 purity %
   During an active measurement cycle shows live in-progress reading.
   Shows 00.00 when no value is available.
   ========================================================= */
static void updateDisp4() {
  const float displayPercent = o2Controller.isBusy()
                                 ? o2Controller.liveN2Percent()
                                 : o2Controller.snapshot().n2Percent;

  const uint16_t n2_x100 = static_cast<uint16_t>(displayPercent * 100.0f + 0.5f);
  disp4.setNumber(n2_x100, true);
  disp4.setDot(1, true);  // XX.XX
}

/* =========================================================
   20x4 LCD: always shows all sensor values
   Row 0: AIRSUPPLY  XXX.X PSI
   Row 1: XXX.X  TOWERS  XXX.X
   Row 2: XX.XX LO N2 HI XXX.X
   Row 3:  NITROGEN  XX.XX %
   ========================================================= */
static void updateLCD() {
  const float displayPercent = o2Controller.isBusy()
                                 ? o2Controller.liveN2Percent()
                                 : o2Controller.snapshot().n2Percent;
  const uint16_t n2_x100 = static_cast<uint16_t>(displayPercent * 100.0f + 0.5f);

  char supplyBuf[8], leftBuf[8], rightBuf[8];
  char lowN2Buf[8], highN2Buf[8], n2Buf[8];
  char line[21];

  formatFixed10(supplyBuf, sizeof(supplyBuf), input.supplyPsi_x10);
  formatFixed10(leftBuf, sizeof(leftBuf), input.leftTowerPsi_x10);
  formatFixed10(rightBuf, sizeof(rightBuf), input.rightTowerPsi_x10);
  formatFixed100(lowN2Buf, sizeof(lowN2Buf), input.lowN2Psi_x100);
  formatFixed10(highN2Buf, sizeof(highN2Buf), input.highN2Psi_x10);
  formatFixed100(n2Buf, sizeof(n2Buf), n2_x100);

  snprintf(line, sizeof(line), "AIRSUPPLY %6s PSI", supplyBuf);
  disp20x4.writeLine(0, line);

  snprintf(line, sizeof(line), "%6s TOWERS %6s", leftBuf, rightBuf);
  disp20x4.writeLine(1, line);

  snprintf(line, sizeof(line), "%5s LO N2 HI %5s", lowN2Buf, highN2Buf);
  disp20x4.writeLine(2, line);

  snprintf(line, sizeof(line), " NITROGEN %6s %%", n2Buf);
  disp20x4.writeLine(3, line);
}

/* =========================================================
   Enable / disable displays
   ========================================================= */
static void enableDisplays() {
  disp4.displayOn();
  disp4.setBrightness(config.display.disp4Brightness);
  disp20x4.backlightOn();
  disp20x4.displayOn();
}

static void disableDisplays() {
  disp4.displayOff();
  disp20x4.backlightOff();
  disp20x4.displayOff();
}

/* =========================================================
   Shutdown: safe state for all outputs and controllers
   ========================================================= */
static void shutdown() {
  if (systemWasEnabled) {
    Serial.println(F("Black switch off — shutting down."));
  }
  towerController.shutdown();
  o2Controller.shutdown();
  n2Controller.shutdown();
  disableDisplays();
  systemWasEnabled = false;
}

/* =========================================================
   setup()
   ========================================================= */
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Serial.print(F("N2 Controller  compiled "));
  Serial.print(__DATE__);
  Serial.print(' ');
  Serial.println(__TIME__);

  pinMode(BLACK_SWITCH_PIN, INPUT_PULLUP);

  leftValve.begin(false);  // valves closed on startup
  rightValve.begin(false);
  flushValve.begin(false);
  compressorSsr.begin(false);  // compressor off on startup

  analogReadResolution(config.pressure.adcBits);

  Wire.begin();

  disp4.begin();
  disp20x4.begin();
  enableDisplays();

  // Controllers start disabled; loop() enables them when switch is on.
  towerController.setEnabled(false);

  // O2 init is non-blocking: if sensor.begin() fails the controller enters
  // STATE_INIT_RETRY and retries every o2.initRetryMs. System runs regardless.
  if (!o2Controller.init()) {
    Serial.println(F("O2 controller config error — check sampleCount."));
  }

  Serial.println(F("Setup complete."));
}

/* =========================================================
   loop()
   ========================================================= */
void loop() {
  readInputs();

  const bool systemIsEnabled = input.blackSwitchEnabled;
  const bool wasEnabled = systemWasEnabled;

  if (systemIsEnabled && !wasEnabled) {
    Serial.println(F("System enabled."));
    enableDisplays();
    systemWasEnabled = true;
  } else if (!systemIsEnabled && wasEnabled) {
    shutdown();
    return;
  }

  if (!systemIsEnabled) {
    return;
  }

  towerController.setEnabled(true);
  towerController.step(input);
  n2Controller.step(input);
  o2Controller.step(input);

  updateDisp4();
  updateLCD();
}
// EOF
