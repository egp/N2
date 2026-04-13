/* N2V4  260412   */
#include <Arduino.h>
#include <Wire.h>
#include <BitBang_I2C.h>  // v2.2.1 https://github.com/bitbank2/BitBang_I2C
#include <TCP1650.h>      // https://github.com/egp/TCP1650
#include <TCP20x4.h>      // https://github.com/egp/TCP20x4
#include <TCP0465.h>      // https://github.com/egp/TCP0465
#include <TCP3231.h>      // https://github.com/egp/TCP3231
#include "O2Handler.h"
#include "TimedStateMachine.h"
#include "TowerController.h"
#include "ArduinoDigitalOutput.h"
#include "N2Controller.h"
#include "UnoR4PinAssignments.h"

/*
1. O2 handler and Tower controller bith need a valve handler, so it should be shared.
2. Resolve N2 state machine
3. will we want to display HP N2 on the rotary switch? Can we get another position?
   or replace OFF with HP N2?

*/

/*
Values to be modified
*/
const char* PROGRAM_VERSION = "4.2";  // update this major.minor. TODO add change log

/* -- N2 pressure sensor thresholds -- */

constexpr uint16_t N2_LOW_OFF_PSI  = 500U;    // These are scaled to hundredths of PSI
constexpr uint16_t N2_LOW_ON_PSI   = 1000U;
constexpr uint16_t N2_HIGH_ON_PSI  = 1500U;
constexpr uint16_t N2_HIGH_OFF_PSI = 2000U;


/* -- Oxygen sensor parameters -- */
bool o2SensorReady;

/* -- these should probably not change -- */
const uint8_t I2C_O2 = 0x74;       // 0x73 I2C address
const uint8_t I2C_LED = 0x2F;      // 0x2F I2C address for TM1650 4-digit 7-segment LED display
const uint8_t I2C_LCD20x4 = 0x27;  // 0x27 I2C address for display 20x4
const uint8_t lcdColumns = 20, lcdRows = 4;


// ADC minPressure will be .5 VDC, maxPressure will be 4.5 VDC, so scale is from 10% to 90%
const uint8_t bitsOfADC = 10;                                        // precision of ADC for analogRead()
const int analogScaleMax = (1 << (bitsOfADC)) - 1;                   // maximum full scale reading
const int minPressureReading = analogScaleMax / 10;                  // smallest expected pressure sensor reading
const int maxPressureReading = analogScaleMax - minPressureReading;  // largest expected pressure sensor reading


const uint8_t DISP4_BRIGHTNESS = 6;  // 0-7

// Create the instance of the O2 sensor adapter, it will be initialized during setup()
TCP3231::DateTime rtc_dt{};
bool rtcPresent = false;

/*
I2C bus declarations, they are initialized during setup()
*/
BBI2C i2c_disp4{};  // disp4
BBI2C i2c_o2{};     // O2 sensor
BBI2C i2c_20x4{};   // displ20x4
BBI2C i2c_rtc{};    // UNUSED (RTC?)


/* N2 Handler setup */
ArduinoDigitalOutput compressorSsr(SSR_Pin);

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


// Create the instance of the O2 sensor adapter, it will be initialized during setup()
TCP0465SensorAdapter o2Sensor{ i2c_o2, TCP0465::DEFAULT_ADDRESS };

ArduinoDigitalOutput o2FlushValve(O2_FLUSH_VALVE_PIN);

/*
variables to hold sensor readings
*/
int supplyPressure, leftTowerPressure, rightTowerPressure, lowPressureN2, highPressureN2;
uint16_t scaledSupplyPSI, scaledLeftPSI, scaledRightPSI, scaledLowN2PSI, scaledHighN2PSI;
float O2portion, N2portion;
uint16_t n2int;              // will hold integer percent x100
uint8_t rotarySwitchStatus;  // holds current status of rotary switch


/* -- previous values to reduce chatter -- */
float previousO2 = 0.0;
uint8_t previousRotarySwitch = 0, previousButtonValue = 0;
bool systemWasEnabled = false;
bool wasO2Selected = false;  // notice when switch changes to N2

/* -- output buffers -- */
char sprintfBuffer[80];  // holds debugging information before printing
char dtostrfBuf1[9];     // temp buffers for dtostrf(), longer than needed
char dtostrfBuf2[9];
char display4buffer[] = "1234 ";  // holds data before sending to display4


char LCDline0[25];  // 20 chars needed, but extra to avoid overiting something else
char LCDline1[25];
char LCDline2[25];
char LCDline3[25];
const char* template0 = " AIRSUPPLY %5s PSI ";
const char* template1 = " %5s TOWERS  %5s";
// const char* template2 = "  N2 LOW %4s PSI  ";
const char* template2 = "%4s LOW N2 HIGH %4s";
const char* template3 = "  NITROGEN %4s %%   ";


/*
Instances of the library classes
*/
TCP1650 disp4(i2c_disp4);

O2Handler::Config o2Config = {
  300000UL,  // warmupDurationMs: 5 minutes
  60000UL,   // measurementIntervalMs: once per minute
  3000UL,    // flushDurationMs
  2000UL,    // settleDurationMs
  250U,      // sampleIntervalMs
  10U,       // sampleCount
  15000UL,   // freshnessThresholdMs
  1000UL     // errorBackoffMs
};

/*
************************************************************************************
  setup for LCD20x4 library
*/
namespace {

constexpr uint8_t kConfiguredAddress = 0x27;
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
/*
*********************************************************************************************
*/


/* ---------- Forward declarations ---  */
// void readO2Sensor();
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
void displayToLCD20x4();
void disableDisplay4();
void disableDisplay20x4();
uint8_t readRotarySwitch();

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
/*
*********************************************************
Setup for N2 controller
*********************************************************
*/
N2Controller::Config n2Config = {
  N2_LOW_OFF_PSI,
  N2_LOW_ON_PSI,
  N2_HIGH_ON_PSI,
  N2_HIGH_OFF_PSI,
};

N2Controller n2Controller(timerClock, compressorSsr, n2Config);

/*
*********************************************************
Setup for Tower control
*********************************************************
*/

constexpr uint16_t airMinimumPSI = 90;               // disable towers when air supply is below this
constexpr uint32_t LEFT_OPEN_MS = 60000UL;
constexpr uint32_t OVERLAP_MS = 750UL;
constexpr uint32_t RIGHT_OPEN_MS = 60000UL;

ArduinoClock timerClock;

ArduinoDigitalOutput leftTowerValve(LEFT_TOWER_VALVE_PIN);
ArduinoDigitalOutput rightTowerValve(RIGHT_TOWER_VALVE_PIN);

TowerController::Config towerConfig = {
  LEFT_OPEN_MS,
  OVERLAP_MS,
  RIGHT_OPEN_MS,
};

TowerController towerController(timerClock, leftTowerValve, rightTowerValve, towerConfig);

O2Handler o2Handler(timerClock, o2Sensor, o2FlushValve, o2Config);
TCP3231 rtc(i2c_rtc);


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
  //   map (x, inMin, inMax, outMin, outMax) uses formula:
  // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  // which yields pressure 0 to 150 (or 0-30) PSI full scale
  return (map(constrainedValue, minPressureReading, maxPressureReading, 0, fullScale));
}

/* -- read air supply pressure and scale it -- */
void readSupplyPressure() {
  supplyPressure = analogRead(supplyPressurePin);         // int
  scaledSupplyPSI = scalePressure(supplyPressure, 1500);  // tenths of PSI
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
  scaledLowN2PSI = scalePressure(lowPressureN2, 3000);  // hudnredths of PSI
}

void readHighPressureN2() {
  highPressureN2 = analogRead(highPressureN2Pin);
  scaledHighN2PSI = scalePressure(highPressureN2, 1500);  // tenths
}


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
  rotarySwitchStatus = readRotarySwitch();
  switch (rotarySwitchStatus) {
    case rotaryOff:  // 0
      disableDisplay4();
      disableDisplay20x4();
      break;

    case rotarySupply:  // 1 -- air supply
      readSupplyPressure();
      disp4.setNumber(scaledSupplyPSI, true);
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
      disp4.setNumber(scaledLowN2PSI, true);
      setDotHundredths();
      break;

    case rotaryN2Percent:  // N2 percent
      readO2Sensor();      // sets N2portion
      displayO2();
      break;

    default:
      break;
  }

  displayToLCD20x4();
}

/* ---------- support functions ------------- */

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

void displayO2() {
  // N2portion should be in the range of (0.0-75.0),
  // n2int should be in the range of 0-7500
  N2portion = 100.0 - O2portion;
  n2int = (int)(N2portion * 100.0);  // show hundredths of a percent
  disp4.setNumber(n2int, true);
  setDotHundredths();
}

/*
readRotarySwitch -- TODO verify this works, and decode appropriately
*/
uint8_t readRotarySwitch() {
  uint8_t buttonValue;
  Wire.requestFrom(0x24, 1, true);
  buttonValue = 0x3F && Wire.read();  // read from TM1650_DCTRL_BASE == 0x24
  if (buttonValue != previousButtonValue) {
    sprintf(sprintfBuffer, "rotary switch changed from %02X to %02X", previousButtonValue, buttonValue);
    Serial.println(sprintfBuffer);
  };
  previousButtonValue = buttonValue;
  return (buttonValue);
}

/*
displayToLCD20x4
Uses most recent sensor values without re-reading
TODO: Figure out weirdness with the display spacing
TODO: replace fake values with real values
*/

void displayToLCD20x4() {

  dtostrf(scaledSupplyPSI, 5, 1, dtostrfBuf1);
  sprintf(LCDline0, template0, dtostrfBuf1);
  display20x4.writeLine(0, LCDline0);

  dtostrf(scaledLeftPSI, 5, 1, dtostrfBuf1);
  dtostrf(scaledRightPSI, 5, 1, dtostrfBuf2);
  sprintf(LCDline1, template1, dtostrfBuf1, dtostrfBuf2);
  display20x4.writeLine(1, LCDline1);

  dtostrf(scaledLowN2PSI, 5, 2, dtostrfBuf1);
  dtostrf(scaledHighN2PSI, 5, 1, dtostrfBuf2);
  sprintf(LCDline2, template2, dtostrfBuf1, dtostrfBuf2);
  display20x4.writeLine(2, LCDline2);

  dtostrf(N2portion, 5, 2, dtostrfBuf1);
  sprintf(LCDline3, template3, dtostrfBuf1);
  display20x4.writeLine(3, LCDline3);
}


/*
readBlackSwitch()
Callback: reads the black on/off switch (frequently)
TODO: check sense (on/off is high or low), and correct if necessary
*/
void readBlackSwitch() {
  systemEnabled = !digitalRead(blackSwitchPin);
}

/*
shutdown()
*/
void shutdown() {

  Serial.println("shutting down");
  if (systemWasEnabled) { Serial.println(F("Black switch off; Scheduler disabled; Shutting down.")); }

  systemWasEnabled = false;
  towerController.setEnabled(false); // Stop tower, closes both valves
  o2FlushValve.setOn(false);  // close O2 flush valve
  compressorSsr.setOn(false); // stop compressor
  previousSSR = false;
  disableDisplay4(); // close displays
  disableDisplay20x4();
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


void setupI2C() {
  i2c_disp4.bWire = 0;  // use hardware I2C when true, use Bit banging when false
  i2c_o2.bWire = 0;     // use bit banging
  i2c_20x4.bWire = 0;   // use bit banging
  i2c_rtc.bWire = 0;    // use bit banging

  i2c_disp4.iSDA = A4;  // sharing the standard I2C bus
  i2c_disp4.iSCL = A5;
  i2c_o2.iSDA = A4;
  i2c_o2.iSCL = A5;

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
  Serial.println(".");
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

/* ---------- Arduino setup ---------- */
void setup() {
  Serial.begin(115200);           // handshake with USB
  while (!Serial) { delay(1); };  // wait until Serial is available

  setupI2C();     // setup all the I2C buses
  pinMode(blackSwitchPin, INPUT);  // TODO decide if this wants to be INPUT_PULLUP
  pinMode(supplyPressurePin, INPUT);      // A0
  pinMode(leftTowerPressurePin, INPUT);   // A1
  pinMode(rightTowerPressurePin, INPUT); // A2
  pinMode(lowPressureN2Pin, INPUT); // A3
  pinMode(highPressureN2Pin, INPUT);  // A4

  leftTowerValve.begin(false);
  rightTowerValve.begin(false);
  rtcPresent = rtc.begin();         // init the clock if present.
  analogReadResolution(bitsOfADC);  // defaults to 10 bits

  compressorSsr.begin(false);
  o2FlushValve.begin(false);

  if (!o2Handler.begin()) {
    Serial.print("O2Handler begin() failed: ");
    Serial.println(o2Handler.errorString());
  }

  // Wire.setClock(50000);  // slow down the bus for diagnostics
  Wire.begin();
  display4.begin();
  display20x4.begin();
  enableDisplay4();
  enableDisplay20x4();

  sprintf(sprintfBuffer, "N2 v %s, compiled %s at %s with IDE %d", PROGRAM_VERSION, __DATE__, __TIME__, ARDUINO);
  Serial.println(sprintfBuffer);

  if (rtcPresent) {
    rtc.readTime(rtc_dt);
    printDateTime(rtc_dt);
  }

  shutdown();  // start off in a known state.

}  // end of setup()

/* ---------- Arduino main loop ---------- */
/*
This is a tight loop, which runs continuously 
If the black switch is off, it only runs the loop every ten seconds, but it could run continuously
*/
void loop() {
  readBlackSwitch();
  if (systemEnabled) {
    towerController.setEnabled(systemEnabled);
    towerController.tick();  // advance tower controller if necessary
    o2Handler.tick();        // advance o2 handler if necessary
    readPressureSensors();
    n2Controller.update(scaledLowN2PSI); // update N2 controller with current low pressure N2 reading
    displaySelectedValue();  // read rotary switch and display corresponding value in disp4

  } else {
    shutdown();    // TODO can this run once per tight loop?
    delay(10000);  // check occassionally
  }
}


// EOF
