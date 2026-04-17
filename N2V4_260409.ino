#include <Arduino.h>
#include <Wire.h>

#include <TCP1819.h>  // clone of BitBang lib
#include <TCP1650.h>  // https://github.com/egp/TCP1650
#include <TCP20x4.h>  // https://github.com/egp/TCP20x4
#include <TCP0465.h>  // https://github.com/egp/TCP0465
#include <TCP3231.h>  // https://github.com/egp/TCP3231

#include "TimedStateMachine.h"
#include "SystemContext.h"
#include "O2Controller.h"
#include "TowerController.h"
#include "N2Controller.h"
#include "ArduinoDigitalOutput.h"
#include "UnoR4PinAssignments.h"

#if defined(ARDUINO_MINIMA)
#include "SystemProfile_minima.h"
#elif defined(ARDUINO_UNOWIFIR4)
#include "SystemProfile_wifi_scenario.h"
#else
#error "No system profile selected"
#endif

const char* PROGRAM_VERSION = "4.3";  // update this major.minor. TODO add change log

/* -- Unique device addresses on the I2C bus -- */
const uint8_t I2C_LED = 0x2F;      // 0x2F I2C address for TM1650 4-digit 7-segment LED display
const uint8_t I2C_LCD20x4 = 0x27;  // 0x27 I2C address for display 20x4
const uint8_t I2C_O2 = 0x74;       // 0x73 I2C address

// ADC minPressure will be .5 VDC, maxPressure will be 4.5 VDC, so scale is from 10% to 90%
// Applies to supply pressure, left and right tower pressures, and low and high N2 pressures
const uint8_t bitsOfADC = 10;                                        // precision of ADC for analogRead()
const int analogScaleMax = (1 << (bitsOfADC)) - 1;                   // maximum full scale reading
const int minPressureReading = analogScaleMax / 10;                  // smallest expected pressure sensor reading
const int maxPressureReading = analogScaleMax - minPressureReading;  // largest expected pressure sensor reading

/* -- display parameters -- */
const uint8_t DISP4_BRIGHTNESS = 6;  // 0-7

/*
I2C bus declarations, they are initialized during setup()
*/
BBI2C i2c_disp4{};
BBI2C i2c_o2{};
BBI2C i2c_20x4{};
BBI2C i2c_rtc{};

/* Real Time Clock setup */
TCP3231::DateTime rtc_dt{};
bool rtcPresent = false;
TCP3231 rtc(i2c_rtc);

// Forward declarations
SystemContext makeSystemContext();
void refreshInputSnapshot();
void refreshSystemSnapshot();
void refreshInputSnapshot();
void refreshSystemSnapshot();
void readPressureSensors();
void displaySelectedValue();
void displayO2();
void readBlackSwitch();
void setDotTenths();
void setDotHundredths();
void readSupplyPressure();
void readLeftTowerPressure();
void readRightTowerPressure();
void readLowPressureN2();
void readHighPressureN2();
void displayToLCD20x4();
void enableDisplay20x4();
void disableDisplay20x4();
void enableDisplay4();
void disableDisplay4();
void setupI2C();
void printDateTime(const TCP3231::DateTime& dt);
void printTwoDigits(uint8_t value);
static void printFourDigits(uint16_t value);
uint8_t readRotarySwitch();

/*
***************************************************************************************
*/
SystemContext systemContext = makeSystemContext();

SystemConfig& systemConfig = systemContext.config;

SystemRuntime& systemRuntime = systemContext.runtime;

InputSnapshot& inputSnapshot = systemContext.input;

SystemSnapshot& systemSnapshot = systemContext.snapshot;

bool& systemWasEnabled = systemRuntime.power.systemWasEnabled;
bool& systemEnabled = systemRuntime.power.systemEnabled;

int& supplyPressure = systemRuntime.sensors.raw.supply;
int& leftTowerPressure = systemRuntime.sensors.raw.leftTower;
int& rightTowerPressure = systemRuntime.sensors.raw.rightTower;
int& lowPressureN2 = systemRuntime.sensors.raw.lowN2;
int& highPressureN2 = systemRuntime.sensors.raw.highN2;

uint16_t& supplyPsi_x10 = systemRuntime.sensors.scaled.supplyPsi_x10;
uint16_t& scaledLeftPSI = systemRuntime.sensors.scaled.leftTowerPsi_x10;
uint16_t& scaledRightPSI = systemRuntime.sensors.scaled.rightTowerPsi_x10;
uint16_t& lowN2Psi_x100 = systemRuntime.sensors.scaled.lowN2Psi_x100;
uint16_t& highN2Psi_x10 = systemRuntime.sensors.scaled.highN2Psi_x10;

uint8_t& rotarySwitchStatus = systemRuntime.display.rotarySwitchStatus;
uint8_t& previousButtonValue = systemRuntime.display.previousButtonValue;

/* -- output buffers -- */
char sprintfBuffer[80];  // holds debugging information before printing
char dtostrfBuf1[9];     // temp buffers for dtostrf(), longer than needed
char dtostrfBuf2[9];
char display4buffer[] = "1234 ";  // holds data before sending to display4
char LCDline0[21];                // 20 chars needed, but extra to avoid overiting something else
char LCDline1[21];
char LCDline2[21];
char LCDline3[21];

/*
Instances of the library classes
*/
TCP1650 disp4(i2c_disp4);

/*
************************************************************************************
 setup for LCD20x4 library
*/
namespace {
constexpr uint8_t kConfiguredAddress = I2C_LCD20x4;
constexpr bool kConfiguredBacklightActiveHigh = true;
constexpr size_t kCommandBufferSize = 64;

TCP20x4Pcf8574Config makeLcdConfig() {
  TCP20x4Pcf8574Config config = TCP20x4Pcf8574Config::CommonYwRobot(kConfiguredAddress);
  config.pinMap.backlightActiveHigh = kConfiguredBacklightActiveHigh;
  return config;
}

const TCP20x4Pcf8574Config kLcdConfig = makeLcdConfig();
TCP20x4 display20x4(i2c_20x4, kLcdConfig);
char commandBuffer[kCommandBufferSize];
}



/* O2 sensor adapter class */
class TCP0465SensorAdapter : public IO2Sensor {
public:
  TCP0465SensorAdapter()
    : sensor_(),
      i2c_(nullptr),
      address_(TCP0465::DEFAULT_ADDRESS),
      lastError_("I2C bus not configured") {}

  TCP0465SensorAdapter(BBI2C& i2c, uint8_t address)
    : sensor_(),
      i2c_(&i2c),
      address_(address),
      lastError_("not initialized") {}

  bool begin() override {
    if (i2c_ == nullptr) {
      lastError_ = "I2C bus not configured";
      return false;
    }

    if (!sensor_.begin(*i2c_, address_)) {
      lastError_ = sensor_.errorString();
      return false;
    }

    lastError_ = "no error";
    return true;
  }

  bool readOxygenPercent(float& percentVol) override {
    if (!sensor_.readOxygenPercent(percentVol)) {
      lastError_ = sensor_.errorString();
      return false;
    }

    lastError_ = "no error";
    return true;
  }

  const char* errorString() const override {
    return lastError_;
  }

private:
  TCP0465 sensor_;
  BBI2C* i2c_;
  uint8_t address_;
  const char* lastError_;
};

/*
*********************************************************
Setup for timing control
*********************************************************
*/
class ArduinoClock : public IClock {
public:
  uint32_t nowMs() const override {
    return millis();
  }
};

// ArduinoClock timerClock;
ProfileClock timerClock;

ArduinoDigitalOutput leftTowerValve(LEFT_TOWER_VALVE_PIN);
ArduinoDigitalOutput rightTowerValve(RIGHT_TOWER_VALVE_PIN);
ArduinoDigitalOutput o2FlushValve(O2_FLUSH_VALVE_PIN);
ArduinoDigitalOutput compressorSsr(SSR_Pin);
ProfileO2Sensor o2Sensor{ i2c_o2, I2C_O2 };

TowerController towerController(timerClock, leftTowerValve, rightTowerValve, systemConfig);
O2Controller o2Controller(timerClock, o2Sensor, o2FlushValve, systemConfig);
N2Controller n2Controller(timerClock, compressorSsr, systemConfig);

void refreshInputSnapshot() {
  inputSnapshot.sampledAtMs = timerClock.nowMs();
  inputSnapshot.blackSwitchEnabled = systemEnabled;
  inputSnapshot.supplyPsi_x10 = supplyPsi_x10;
  inputSnapshot.leftTowerPsi_x10 = scaledLeftPSI;
  inputSnapshot.rightTowerPsi_x10 = scaledRightPSI;
  inputSnapshot.lowN2Psi_x100 = lowN2Psi_x100;
  inputSnapshot.highN2Psi_x10 = highN2Psi_x10;
}

void refreshSystemSnapshot() {
  systemSnapshot.input = inputSnapshot;
  systemSnapshot.tower = towerController.snapshot();
  systemSnapshot.o2 = o2Controller.snapshot();
  systemSnapshot.n2 = n2Controller.snapshot();
}

SystemContext makeSystemContext() {
  SystemContext ctx{};
  ctx.config = makeSystemConfig();
  return ctx;
}


/*
Each pressure sensor callback set corresponding variable on schedule
*/
void readPressureSensors() {
  // all five sensors are read sequentially.
  readSupplyPressure();
  readLeftTowerPressure();
  readRightTowerPressure();
  readLowPressureN2();
  readHighPressureN2();
}

/*
int scalePressure(int pressure, int fullScale)
*/
int scalePressure(int pressure, int fullScale) {
  // ensure (minPressureReading <= pressure <= maxPressureReading) (1020-9210) (0.5 VDC to 4.5 VDC)
  int constrainedValue = constrain(pressure, minPressureReading, maxPressureReading);

  // map (x, inMin, inMax, outMin, outMax) uses formula:
  // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // which yields pressure 0 to 150 (or 0-30) PSI full scale
  return (map(constrainedValue, minPressureReading, maxPressureReading, 0, fullScale));
}

/* -- read air supply pressure and scale it -- */

void readSupplyPressure() {
  supplyPressure = analogRead(supplyPressurePin);
  supplyPsi_x10 = scalePressure(supplyPressure, 1500);
}

/* -- read left tower pressure and scale it -- */
void readLeftTowerPressure() {
  leftTowerPressure = analogRead(leftTowerPressurePin);
  scaledLeftPSI = scalePressure(leftTowerPressure, 1500);  // tenths
}

/* -- read right tower pressure and scale it -- */
void readRightTowerPressure() {
  rightTowerPressure = analogRead(rightTowerPressurePin);
  scaledRightPSI = scalePressure(rightTowerPressure, 1500);  // tenths
}

/* -- read low and high pressure N2 and scale -- */
void readLowPressureN2() {
  lowPressureN2 = analogRead(lowPressureN2Pin);
  lowN2Psi_x100 = scalePressure(lowPressureN2, 3000);  // hudnredths of PSI
}

void readHighPressureN2() {
  highPressureN2 = analogRead(highPressureN2Pin);
  highN2Psi_x10 = scalePressure(highPressureN2, 1500);  // tenths
}

/*
*********************************************************
Display and rotary-switch support
*********************************************************
*/

// TODO: Verify correct hex values
const uint8_t rotaryOff = 0x44;        // 0x44 OFF
const uint8_t rotarySupply = 0x4C;     // 0x4C Air Supply
const uint8_t rotaryLeft = 0x54;       // 0x54 Left Tower
const uint8_t rotaryRight = 0x5C;      // 0x5C Right Tower
const uint8_t rotaryN2Low = 0x64;      // 0x64 Low Pressure N2
const uint8_t rotaryN2Percent = 0x6C;  // 0x6C N2 percent

/*
displaySelectedValue()
*/
void displaySelectedValue() {

#if defined(ARDUINO_UNOWIFIR4)
  rotarySwitchStatus = rotaryN2Percent;  // or rotarySupply for early smoke tests
#else
  rotarySwitchStatus = readRotarySwitch();
#endif

  switch (rotarySwitchStatus) {
    case rotaryOff:  // 0
      disableDisplay4();
      disableDisplay20x4();
      break;

    case rotarySupply:  // 1 -- air supply
      readSupplyPressure();
      disp4.setNumber(supplyPsi_x10, true);
      setDotTenths();
      break;

    case rotaryLeft:  // 2 -- Left
      readLeftTowerPressure();
      disp4.setNumber(scaledLeftPSI, true);
      setDotTenths();
      break;

    case rotaryRight:  // 3 -- right
      readRightTowerPressure();
      disp4.setNumber(scaledRightPSI, true);
      setDotTenths();
      break;

    case rotaryN2Low:  // low n2
      readLowPressureN2();
      disp4.setNumber(lowN2Psi_x100, true);
      setDotHundredths();
      break;

    case rotaryN2Percent:  // N2 percent
      displayO2();
      break;

    default:
      break;
  }

  displayToLCD20x4();
}

/*
setDotTenths()
Note: decimal point appears to the left of the specified digit
*/
void setDotTenths() {
  disp4.setDot(2, true);
}

/*
setDotHundredths()
Note: decimal point appears to the left of the specified digit
*/
void setDotHundredths() {
  disp4.setDot(1, true);
}

// N2portion should be in the range of (0.0-99.9),
// n2int should be in the range of 0-7500
void displayO2() {
  uint16_t n2_x100 = 0;

  if (systemSnapshot.o2.hasValue) {
    n2_x100 = static_cast<uint16_t>(systemSnapshot.o2.n2Percent * 100.0f + 0.5f);
  }

  disp4.setNumber(n2_x100, true);
  setDotHundredths();
}


/*
readRotarySwitch
*/
uint8_t readRotarySwitch() {
  uint8_t buttonValue;

  Wire.requestFrom(0x24, 1, true);
  buttonValue = 0x3F & Wire.read();  // read from TM1650_DCTRL_BASE == 0x24

  if (buttonValue != previousButtonValue) {
    sprintf(sprintfBuffer, "rotary switch changed from %02X to %02X", previousButtonValue, buttonValue);
    Serial.println(sprintfBuffer);
  };

  previousButtonValue = buttonValue;
  return (buttonValue);
}

void formatFixed1(char* out, size_t outSize, uint16_t value_x10) {
  const uint16_t whole = value_x10 / 10U;
  const uint16_t frac = value_x10 % 10U;
  snprintf(out, outSize, "%u.%u", whole, frac);
}

void formatFixed2(char* out, size_t outSize, uint16_t value_x100) {
  const uint16_t whole = value_x100 / 100U;
  const uint16_t frac = value_x100 % 100U;
  snprintf(out, outSize, "%u.%02u", whole, frac);
}

/*
======================
|AIRSUPPLY  120.0 PSI|
| 35.5  TOWERS   36.5|
| 5.00 LO N2 HI  90.0|
|  NITROGEN 100.00 % |
======================
*/

void displayToLCD20x4() {
  char supplyBuf[8];
  char leftBuf[8];
  char rightBuf[8];
  char lowN2Buf[8];
  char highN2Buf[8];
  char n2Buf[8];

  uint16_t n2_x100 = 0;
  if (systemSnapshot.o2.hasValue) {
    n2_x100 = static_cast<uint16_t>(systemSnapshot.o2.n2Percent * 100.0f + 0.5f);
  }

  display20x4.backlightOn();
  display20x4.displayOn();

  formatFixed1(supplyBuf, sizeof(supplyBuf), supplyPsi_x10);
  formatFixed1(leftBuf, sizeof(leftBuf), scaledLeftPSI);
  formatFixed1(rightBuf, sizeof(rightBuf), scaledRightPSI);
  formatFixed2(lowN2Buf, sizeof(lowN2Buf), lowN2Psi_x100);
  formatFixed1(highN2Buf, sizeof(highN2Buf), highN2Psi_x10);
  formatFixed2(n2Buf, sizeof(n2Buf), n2_x100);

  snprintf(LCDline0, sizeof(LCDline0), "AIRSUPPLY %6s PSI", supplyBuf);
  snprintf(LCDline1, sizeof(LCDline1), " %4s TOWERS %6s", leftBuf, rightBuf);
  snprintf(LCDline2, sizeof(LCDline2), "%5s LO N2 HI %5s", lowN2Buf, highN2Buf);
  snprintf(LCDline3, sizeof(LCDline3), " NITROGEN %6s %%", n2Buf);

  display20x4.writeLine(0, LCDline0);
  display20x4.writeLine(1, LCDline1);
  display20x4.writeLine(2, LCDline2);
  display20x4.writeLine(3, LCDline3);
}

/*
readBlackSwitch()
reads the black on/off switch (frequently)
*/
void readBlackSwitch() {
  systemEnabled = !digitalRead(blackSwitchPin);
}

void enableDisplay20x4() {
  display20x4.backlightOn();
  display20x4.displayOn();
}

void disableDisplay20x4() {
  display20x4.backlightOff();
  display20x4.displayOff();
}

void enableDisplay4() {
  disp4.displayOn();
  disp4.setBrightness(DISP4_BRIGHTNESS);
}

void disableDisplay4() {
  disp4.displayOff();
}

/*
*********************************************************
I2C and RTC helpers
*********************************************************
*/

void setupI2C() {
  i2c_disp4.bWire = 0;  // use hardware I2C when true, use Bit banging when false
  i2c_o2.bWire = 0;     // use bit banging
  i2c_20x4.bWire = 0;   // use bit banging
  i2c_rtc.bWire = 0;    // use bit banging

  i2c_disp4.iSDA = I2C_BUSA_SDA;  // sharing the standard I2C bus for now.
  i2c_disp4.iSCL = I2C_BUSA_SCL;

  i2c_o2.iSDA = I2C_BUSB_SDA;
  i2c_o2.iSCL = I2C_BUSB_SCL;

  i2c_20x4.iSDA = I2C_BUSC_SDA;
  i2c_20x4.iSCL = I2C_BUSC_SCL;

  i2c_rtc.iSDA = I2C_BUSD_SDA;
  i2c_rtc.iSCL = I2C_BUSD_SCL;

  I2CInit(&i2c_disp4, 100000);  // 100K clock
  I2CInit(&i2c_o2, 100000);     // 100K clock
  I2CInit(&i2c_20x4, 100000);   // 100K clock
  I2CInit(&i2c_rtc, 100000);    // 100K clock
}

void printDateTime(const TCP3231::DateTime& dt) {
  printFourDigits(dt.year);
  Serial.print('-');
  printTwoDigits(dt.month);
  Serial.print('-');
  printTwoDigits(dt.day);
  Serial.print(' ');
  printTwoDigits(dt.hour);
  Serial.print(':');
  printTwoDigits(dt.minute);
  Serial.println(" ");
}

void printTwoDigits(uint8_t value) {
  if (value < 10) {
    Serial.print('0');
  }

  Serial.print(value);
}

static void printFourDigits(uint16_t value) {
  if (value < 1000) Serial.print('0');
  if (value < 100) Serial.print('0');
  if (value < 10) Serial.print('0');
  Serial.print(value);
}

void printScenarioBanner() {
#if defined(ARDUINO_UNOWIFIR4)
  static uint32_t lastPrintedMs = 0xFFFFFFFFu;
  if (inputSnapshot.sampledAtMs == lastPrintedMs) {
    return;
  }
  lastPrintedMs = inputSnapshot.sampledAtMs;

  Serial.print(F("SCEN t="));
  Serial.print(inputSnapshot.sampledAtMs);
  Serial.print(F(" en="));
  Serial.print(inputSnapshot.blackSwitchEnabled ? 1 : 0);
  Serial.print(F(" sup="));
  Serial.print(inputSnapshot.supplyPsi_x10);
  Serial.print(F(" L="));
  Serial.print(inputSnapshot.leftTowerPsi_x10);
  Serial.print(F(" R="));
  Serial.print(inputSnapshot.rightTowerPsi_x10);
  Serial.print(F(" lo="));
  Serial.print(inputSnapshot.lowN2Psi_x100);
  Serial.print(F(" hi="));
  Serial.print(inputSnapshot.highN2Psi_x10);
  Serial.print(F(" O2="));
  Serial.print(systemSnapshot.o2.o2Percent, 2);
  Serial.print(F(" Ts="));
  Serial.print(static_cast<uint8_t>(systemSnapshot.tower.state));
  Serial.print(F(" Os="));
  Serial.print(static_cast<uint8_t>(systemSnapshot.o2.state));
  Serial.print(F(" Ns="));
  Serial.println(static_cast<uint8_t>(systemSnapshot.n2.state));
#endif
}

void checkTBS() {
  if (systemEnabled != systemWasEnabled) {
    Serial.print(F("cTBS en="));
    Serial.print(systemEnabled ? 1 : 0);
    Serial.print(F(" was="));
    Serial.println(systemWasEnabled ? 1 : 0);
  }

  if (systemEnabled && !systemWasEnabled) {
    enableDisplay4();
    enableDisplay20x4();
  } else if (!systemEnabled && systemWasEnabled) {
    disableDisplay4();
    disableDisplay20x4();
  }
  systemWasEnabled = systemEnabled;
}

void displaySelfTest() {
  enableDisplay4();
  disp4.setNumber(1234, true);
  disp4.setDot(1, true);
}

const char* lcd20x4StatusName(TCP20x4Status status) {
  switch (status) {
    case TCP20x4Status::Ok: return "Ok";
    case TCP20x4Status::InvalidLine: return "InvalidLine";
    case TCP20x4Status::LineTooLong: return "LineTooLong";
    case TCP20x4Status::InvalidArgument: return "InvalidArgument";
    case TCP20x4Status::NotInitialized: return "NotInitialized";
    case TCP20x4Status::TransportError: return "TransportError";
    case TCP20x4Status::NotImplemented: return "NotImplemented";
    default: return "Unknown";
  }
}

void displaySelfTest20x4() {
  Serial.println(F("20x4 self-test starting"));

  display20x4.begin();
  display20x4.backlightOn();
  display20x4.displayOn();

  display20x4.writeLine(0, "20x4 SELF TEST      ");
  display20x4.writeLine(1, "Pins 9,10 expected  ");
  display20x4.writeLine(2, "If dark: bus/usage  ");
  display20x4.writeLine(3, "Check Serial status ");
}

/* ---------- Arduino setup ---------- */
void setup() {
  Serial.begin(115200);           // handshake with USB
  while (!Serial) { delay(1); };  // wait until Serial is available

  setupI2C();  // setup all the I2C buses

  setPinMode();  // setup all pin modes.

  leftTowerValve.begin(false);
  rightTowerValve.begin(false);
  rtcPresent = rtc.begin();  // init the clock if present.

  analogReadResolution(bitsOfADC);  // defaults to 10 bits

  compressorSsr.begin(false);
  o2FlushValve.begin(false);

  systemProfileSetup(timerClock, o2Sensor);

  if (!o2Controller.init()) {
    Serial.print("O2Controller init() failed: ");
    Serial.println(o2Controller.errorString());
  }

  // Wire.setClock(50000); // slow down the bus for diagnostics
  Wire.begin();

  disp4.begin();
  enableDisplay4();

  display20x4.begin();
  enableDisplay20x4();
  displaySelfTest();
  displaySelfTest20x4();

#if defined(ARDUINO_MINIMA)
  sprintf(sprintfBuffer, "N2 v %s, compiled %s at %s with IDE %d for UNO R4 Minima", PROGRAM_VERSION, __DATE__, __TIME__, ARDUINO);
#elif defined(ARDUINO_UNOWIFIR4)
  sprintf(sprintfBuffer, "N2 v %s, compiled %s at %s with IDE %d for UNO R4 Wifi", PROGRAM_VERSION, __DATE__, __TIME__, ARDUINO);
#endif

  Serial.println(sprintfBuffer);

  if (rtcPresent) {
    rtc.readTime(rtc_dt);
    printDateTime(rtc_dt);
  }

}  // end of setup()

/* ---------- Arduino main loop ---------- */

/*
This is a tight loop, which runs continuously
If the black switch is off, it checks less often.
*/
// void loop() {
// #if defined(ARDUINO_UNOWIFIR4)
//   systemProfileConsumeSerialCommand(timerClock, systemContext);
//   systemProfileRefreshInputs(
//     systemContext,
//     timerClock,
//     systemEnabled,
//     supplyPsi_x10,
//     scaledLeftPSI,
//     scaledRightPSI,
//     lowN2Psi_x100,
//     highN2Psi_x10);

//   systemEnabled = inputSnapshot.blackSwitchEnabled;

//   supplyPsi_x10 = inputSnapshot.supplyPsi_x10;
//   scaledLeftPSI = inputSnapshot.leftTowerPsi_x10;
//   scaledRightPSI = inputSnapshot.rightTowerPsi_x10;
//   lowN2Psi_x100 = inputSnapshot.lowN2Psi_x100;
//   highN2Psi_x10 = inputSnapshot.highN2Psi_x10;
// #else
//   readBlackSwitch();
//   systemEnabled = !digitalRead(blackSwitchPin);

//   if (systemEnabled) {
//     readPressureSensors();
//     refreshInputSnapshot();
//   }
// #endif

//   checkTBS();

//   if (systemEnabled) {
//     o2Controller.step(inputSnapshot);
//     towerController.setEnabled(inputSnapshot.blackSwitchEnabled);
//     towerController.step(inputSnapshot);
//     n2Controller.step(inputSnapshot);

//     bool& lastN2ControllerOk = systemRuntime.lastN2ControllerOk;

//     refreshSystemSnapshot();
// #if defined(ARDUINO_UNOWIFIR4)
//     printScenarioBanner();
// #endif
//     displaySelectedValue();
//   } else {
//     shutdown();
// #if !defined(ARDUINO_UNOWIFIR4)
//     delay(10000);
// #endif
//   }
// }

void loop() {
#if defined(ARDUINO_UNOWIFIR4)
  systemProfileConsumeSerialCommand(timerClock, systemContext);
  systemProfileRefreshInputs(
    systemContext,
    timerClock,
    systemEnabled,
    supplyPsi_x10,
    scaledLeftPSI,
    scaledRightPSI,
    lowN2Psi_x100,
    highN2Psi_x10);

  systemEnabled = inputSnapshot.blackSwitchEnabled;

  // Bridge scenario-fed inputs into the legacy display globals.
  supplyPsi_x10 = inputSnapshot.supplyPsi_x10;
  scaledLeftPSI = inputSnapshot.leftTowerPsi_x10;
  scaledRightPSI = inputSnapshot.rightTowerPsi_x10;
  lowN2Psi_x100 = inputSnapshot.lowN2Psi_x100;
  highN2Psi_x10 = inputSnapshot.highN2Psi_x10;
#else
  readBlackSwitch();
  systemEnabled = !digitalRead(blackSwitchPin);  //

  if (systemEnabled) {
    readPressureSensors();
    refreshInputSnapshot();
  }
#endif

  const bool wasEnabled = systemWasEnabled;
  checkTBS();  // prints when it changes

  if (!systemEnabled) {
    if (wasEnabled) {
      shutdown();  // only shutdown when ON to OFF transition
    }
#if !defined(ARDUINO_UNOWIFIR4)
    delay(10000);  // Pause for this many milliseconds before checking the TBS again,
#endif
    return;
  }

  o2Controller.step(inputSnapshot);

  towerController.setEnabled(inputSnapshot.blackSwitchEnabled);
  towerController.step(inputSnapshot);

  n2Controller.step(inputSnapshot);

  refreshSystemSnapshot();
  displaySelectedValue();

#if defined(ARDUINO_UNOWIFIR4)
  printScenarioBanner();
#endif
}

/*
********************************************************************
*/
void shutdown() {
  if (systemWasEnabled) { Serial.println(F("Black switch off; Shutting down.")); }
  systemWasEnabled = false;

  towerController.setEnabled(false);  // Stop tower, closes both valves
  o2FlushValve.setOn(false);          // close O2 flush valve

  compressorSsr.setOn(false);  // stop compressor

  disableDisplay4();  // turn off displays
  disableDisplay20x4();
}

// EOF