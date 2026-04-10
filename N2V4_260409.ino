/* N2_V3A  2026.02.02 08:37   */
#include <Arduino.h>
#include <Wire.h>
// #include <DFRobot_OxygenSensor.h>  // 1.0.2 https://github.com/DFRobot/DFRobot_OxygenSensor
#include <TCP1650.h>  // v0.1.0 https://github.com/egp/TCP1650
#include <TCP20x4.h>  // v0.1.0 https://github.com/egp/TCP20x4
#include <TCP0465.h>  // V0.1.1 https://github.com/egp/TCP0465
#include "O2Handler.h"


/*
N2 -- Nitrogen generator code
    -- This code handles
      -- Three valves via digital outputs (left, right, O2)
      -- one SSR via digital output, Solid State Rectifier for N2 compressor 
      -- Four pressure sensors via Analog inputs
        -- Three 0-150 psi (for air supply and left and right towers)
        -- One 0-30 psi (for low pressure N2)
      -- Oxygen percentage sensor (displayed as N2 percentage)
      -- One On/Off switch (black) to shutdown the system
      -- One six-position rotary switch to select what displays on the seven-segment display
      -- One four-digit seven-segment display      Display4
      -- One LCD display, four rows of 20 columns  Display20x4
      -- Serial Interface for debugging, monitoring
    -- All actions are scheduled, except for setup() and the scheduler itself
---------------------------------------------------------------------------------------------------
Theory of operation:
  Note: TODO All timings are yet to be adjusted based on actual performance
  Two towers take turns separating O2 from N2 over half cycles of about a minute.
  At the end of each half cycle, the valves are changed to swap to the other tower,
  with a short (approximately 750 ms) overlap.

  Pressure sensors are read and used to monitor the system, and need calibration.
  Sensor values are displayed based on the position of the rotary switch.
  They are also displayed on the LCD 20x4 display.
---------------------------------------------------------------------------------------------------
  Questions for Tom
  1. shutdown() closes the valves and turns off displays. What else needs to be done to shutdown
     If we had a 120V relay, we could literally shutdown the Arduino power under Arduino control,
     cutting off the branch we're sitting on. Obviously it would require a monentary contact
     normally open push button to re-enable the relay. The Arduino would then bootup and run setup().
  2. What are the rules for the SSR (see checkN2Compressor())
---------------------------------------------------------------------------------------------------
TODO:
- Verify all pin definitions -- DONE?
- Verify which state HIGH,LOW is VALVE_OPEN or VALVE_CLOSED. Verify all three valves.
- Verify which state the black switch sees when on and off (current logic might be backwards).
- Identify all out-of-bounds pressure readings, monitor, and respond correctly
- Tower cycling
  - verify correct operating pressures
  - verify and adjust correct tower timing
  - verify and adjust correct overlap timing
- Verify all sensor output conversions for display
  - O2 sensor, N2 percentage
  - air supply PSI
  - Left and Right Tower PSI
  - Low pressure N2 PSI
- Walk through taskList and adjust all callback periods.
- validate task timings (ensure each callback runs in a reasonable amount of time)
- Verify all debug outputs
- cleanup display20x4 layout
- Currently there are many debugging statements enabled with verboseSerial.
    Most should be commented out before we ever try the code smoke test. Enable as necessary.
- sensor scaling factors might need to be individualized
---------------------------------------------------------------------------------------------------
- develop debug macro, with line numbers
- develop cmd parser for displaying and tweaking values while running
  - decide on parameters to be tweaked
    - overlap timeout
    - airMinimumPressure 
    - thresholds
- Measure the tight loop. How many milliseconds if there is nothing for the scheduler to run.
- pull scheduler code out into a separate class?
*/

/*
Values to be modified
*/
const char* PROGRAM_VERSION = "3.2";  // update this major.minor. TODO add change log
bool verboseSerial = true;            // enable for verbose debugging messages to the Serial Monitor

/* -- pressure sensor parameters -- */
const uint16_t overlapTimeout = 750;  // milliseconds of overlap time for the two towers
int n2MinimumPSI = 20;                // disable N2 compressor when low pressure N2 is below this
int airMinimumPSI = 90;               // disable towers when air supply is below this

/* -- Oxygen sensor parameters -- */
const uint8_t o2RetryPeriod = 250;      // milliseconds to wait before retrying O2 sensor error at startup
const uint16_t o2FlushPeriod = 2000;    // flush O2 sensor with N2 for this long
const uint16_t o2SamplePeriod = 2000;   // wait for O2 to stabilize
const uint16_t o2AveragingPeriod = 50;  // milliseconds between O2 samples during averaging
const uint8_t COLLECT_NUMBER = 10;      // Returns the average of this many samples
bool o2SensorReady;

/*
I/O pin definitions
*/
const uint8_t onboardLED = LED_BUILTIN;    // digital I/O pin 13
const uint8_t supplyPressurePin = A0;      // analog input pin 14
const uint8_t leftTowerPressurePin = A1;   // analog input pin 15 Relay_1
const uint8_t rightTowerPressurePin = A2;  // analog input pin 16 Relay_2
const uint8_t lowPressureN2Pin = A3;       // analog input pin 17
const uint8_t I2C_SDA = A4;                // digital I/O pin 18 -- I2C DATA
const uint8_t I2C_SCL = A5;                // digital I/O pin 19 -- I2C CLOCK
const uint8_t blackSwitchPin = D0;         // digital input
const uint8_t unusedD1 = D1;
const uint8_t otherButton = D2;         // Other button
const uint8_t unusedD3 = D3;            // PWM pins D3, D5, D6, D9, D10, and D11.
const uint8_t leftTowerValvePin = D4;   // digital output
const uint8_t unusedD5 = D5;            // PWM
const uint8_t unusedD6 = D6;            // PWM
const uint8_t rightTowerValvePin = D7;  // digital output
const uint8_t SSR_Pin = D8;             // digital output TODO verify correct pin Relay_3
const uint8_t unusedD9 = D9;            // PWM
const uint8_t unusedD10 = D10;          // PWM
const uint8_t O2SamplePin = D11;        // PWM digital output TODO verify correct pin Relay_4
const uint8_t unusedD12 = D12;
const uint8_t unusedD13 = D13;
const uint8_t unusedD14 = D14;
const uint8_t unusedD15 = D15;
// const uint8_t onboardLedTx = LED_TX;  // 21
// const uint8_t onboardLedRx = LED_RX;  // 22

/* -- these should probably not change -- */
const uint8_t I2C_O2 = 0x74;   // 0x73 I2C address
const uint8_t I2C_LED = 0x2F;  // 0x2F I2C address for TM1650 4-digit 7-segment LED display

const uint8_t I2C_LCD20x4 = 0x27;  // 0x27 I2C address for display 20x4
const uint8_t lcdColumns = 20, lcdRows = 4;


// ADC minPressure will be .5 VDC, maxPressure will be 4.5 VDC, so scale is from 10% to 90%
const uint8_t bitsOfADC = 10;                                        // precision of ADC for analogRead()
const int analogScaleMax = (1 << (bitsOfADC)) - 1;                   // maximum full scale reading
const int minPressureReading = analogScaleMax / 10;                  // smallest expected pressure sensor reading
const int maxPressureReading = analogScaleMax - minPressureReading;  // largest expected pressure sensor reading

const uint8_t VALVE_OPEN = HIGH;  // TODO verify these states, swap if necessary
const uint8_t VALVE_CLOSED = LOW;
const uint8_t SSR_ON = HIGH;
const uint8_t SSR_OFF = LOW;

const uint8_t DISP4_BRIGHTNESS = 1;  // 0-7

/*
variables to hold sensor readings
*/
int supplyPressure, leftTowerPressure, rightTowerPressure, lowPressureN2;
uint16_t scaledSupplyPSI, scaledLeftPSI, scaledRightPSI, scaledLowN2PSI;
float O2portion, N2portion;
uint16_t n2int;              // will hold integer percent x100
uint8_t rotarySwitchStatus;  // holds current status of rotary switch
bool leftTowerActive, rightTowerActive, N2compressorRunning, systemEnabled;

/* -- previous values to reduce chatter -- */
float previousO2 = 0.0;
uint8_t previousRotarySwitch = 0, previousButtonValue = 0;
bool systemWasEnabled = false;
bool previousSSR = false;

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
const char* template2 = "  N2 LOW %4s PSI  ";
const char* template3 = "  NITROGEN %4s %%   ";


/*
Instances of the library classes
*/
TCP1650 disp4(2, 3);

TCP0465 oxygen;

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
TCP20x4 display20x4(Wire, kLcdConfig);
char commandBuffer[kCommandBufferSize];
}
/*
*********************************************************************************************
*/

/* ---------- Task struct for scheduler ---------- */
struct Task {
  uint32_t interval;   // how often to run (ms)
  uint32_t lastRun;    // last execution time
  void (*callback)();  // task function to be called on schedule
};

/* ---------- Forward declarations for taskList table ---  */
void readO2Sensor();
void readPressureSensors();
void cycleTowers();
void checkN2Compressor();
void displaySelectedValue();
void readBlackSwitch();
/* -------------other forward declarations ---------------*/
void displayO2();
void readBlackSwitch();
uint8_t readRotarySwitch();
void setDotTenths();
void setDotHundredths();
void readSupplyPressure();
void readLeftTowerPressure();
void readRightTowerPressure();
void readLowPressureN2();
void displayToLCD20x4();
void disableDisplay4();
void disableDisplay20x4();

/* ---------- Task taskList table ---------- */
/*
Must follow the task callback forward declarations.
Keep with taskName table
TODO: All these periods need adjustment
*/
Task taskList[] = {
  { 3000, 0, readO2Sensor },  // 0

  { 50, 0, readPressureSensors },     // 1
  { 5900, 0, cycleTowers },           // 2  once/minute
  { 50, 0, checkN2Compressor },       // 3
  { 1000, 0, displaySelectedValue },  // 4
  { 1000, 0, readBlackSwitch }        // 5
};
const uint8_t NUM_TASKS = sizeof(taskList) / sizeof(taskList[0]);
/*
Task names for debugging. Same number and order as taskList, keep together
*/
const char* taskName[NUM_TASKS] = {
  "0 readO2Sensor",
  "1 readPressureSensors",
  "2 cycleTowers",
  "3 checkN2Compressor",
  "4 displaySelectedValue",
  "5 readBlackSwitch"
};

/*
readO2Sensor()
Open sample valve, wait for stable, close sample valve, wait for stable
reads from sensor, sets O2portion, and calculates N2portion.
*/
void readO2Sensor() {
  digitalWrite(O2SamplePin, VALVE_OPEN);
  for (uint16_t flushTime = 0; flushTime < o2FlushPeriod; flushTime += o2AveragingPeriod) {
  }
  digitalWrite(O2SamplePin, VALVE_CLOSED);
  for (uint16_t sampleTime = 0; sampleTime < o2SamplePeriod; sampleTime += o2AveragingPeriod) {
  }
  /* -- here with O2portion as a float from 0.0 to 25.0 (TODO: Verify) */
  N2portion = 100.0 - getOxygenData(10,100); // nSamples, msDelay
  if (verboseSerial && (previousO2 != O2portion)) {  // only print if different than last time
    sprintf(sprintfBuffer, "O2portion %.2f,  N2portion %.2f ", O2portion, N2portion);
    Serial.println(sprintfBuffer);
  };
  previousO2 = O2portion;
}

/*
float getAverageOxygen()
  collect samples spaced apart by 100 ms.  Its effective range is 0~25%Vol.
  Open for 2-3 seconds, close, then wait 2-3 seconds for stability
  Note the documentation says the COLLECT_NUMBER is how many samples it
      averages over, so we don't need to do averaging explicitly.
*/
/*
float getAverageOxygen() {
  // verify float from 0.0 - 25.0 [ or 0.0 to 0.25 ??]
  return (oxygen.getOxygenData(COLLECT_NUMBER)); 
}
*/

float getOxygenData(byte numberOfSamples, uint16_t delayMs) {
  if (numberOfSamples == 0) {
    Serial.println("TCP0465 getOxygenData(): numberOfSamples must be > 0");
    return -1.0f;
  }

  float sum = 0.0f;
  byte validSamples = 0;

  for (byte i = 0; i < numberOfSamples; ++i) {
    float percentVol = 0.0f;

    if (oxygen.readOxygenPercent(percentVol)) {
      sum += percentVol;
      ++validSamples;
    } else {
      Serial.print("TCP0465 readOxygenPercent() failed: ");
      Serial.println(oxygen.errorString());
    }

    if ((i + 1) < numberOfSamples && delayMs > 0) {
      delay(delayMs);
    }
  }

  if (validSamples == 0) {
    Serial.println("TCP0465 getOxygenData(): no valid oxygen samples");
    return -1.0f;
  }

  return sum / static_cast<float>(validSamples);
}

/*
Each pressure sensor callback set corresponding variable on schedule
*/
void readPressureSensors() {
  // all four sensors are read at the same time.
  readSupplyPressure();
  readLeftTowerPressure();
  readRightTowerPressure();
  readLowPressureN2();
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
  scaledLeftPSI = scalePressure(leftTowerPressure, 1500);
}

/* -- read right tower pressure and scale it -- */
void readRightTowerPressure() {
  rightTowerPressure = analogRead(rightTowerPressurePin);
  scaledRightPSI = scalePressure(rightTowerPressure, 1500);
}

/* -- read low pressure N2 and scale it -- */
void readLowPressureN2() {
  lowPressureN2 = analogRead(lowPressureN2Pin);
  scaledLowN2PSI = scalePressure(lowPressureN2, 3000);  // hudnredths of PSI
}

/* -- valve open/close routines -- */
void openLeftTowerValve() {
  digitalWrite(leftTowerValvePin, VALVE_OPEN);
  leftTowerActive = true;
}
void closeLeftTowerValve() {
  digitalWrite(leftTowerValvePin, VALVE_CLOSED);
  leftTowerActive = false;
}
void openRightTowerValve() {
  digitalWrite(rightTowerValvePin, VALVE_OPEN);
  rightTowerActive = true;
}
void closeRightTowerValve() {
  digitalWrite(rightTowerValvePin, VALVE_CLOSED);
  rightTowerActive = false;
}

/*
cycleTowers()
  If the left tower has been running, switch to the right tower,
  and vice-versa, keeping both towers going during the specified overlap time.
  If neither tower is active, start the left tower.

NOTE: This assumes cycling is schedule based, not pressure based, but we should
  add some pressure checks to ensure system integrity before and during cycling
*/
void cycleTowers() {

  if (scaledSupplyPSI < airMinimumPSI) {
    if (verboseSerial) {
      Serial.println(F("Loss of sufficient air supply, closing both towers"));
      Serial.print("Reading ");
      Serial.print(scaledSupplyPSI);
      Serial.print(" supply, expecting ");
      Serial.println(airMinimumPSI);
    };
    closeLeftTowerValve();
    closeRightTowerValve();
    systemWasEnabled = false;
    return;
  } else {
    if (leftTowerActive) {
      if (verboseSerial) { Serial.println(F("Switching Left --> Right Tower...")); };
      openRightTowerValve();
      delay(overlapTimeout);  // delay prevents anything else from running
      closeLeftTowerValve();
      if (verboseSerial) { Serial.println(F("Finished Left --> Right Tower.")); };
      return;
    } else if (rightTowerActive) {
      if (verboseSerial) { Serial.println(F("Switching Right --> Left Tower...")); };
      openLeftTowerValve();
      delay(overlapTimeout);  // delay prevents anything else from running
      closeRightTowerValve();
      if (verboseSerial) { Serial.println(F("Finished Right --> Left.")); };
      return;
    } else {
      // here if neither tower is active, presumably during startup
      if (scaledSupplyPSI >= airMinimumPSI) {  // confirm sufficient supply
        if (verboseSerial) { Serial.println(F("Starting Left Tower...")); };
        openLeftTowerValve();
        systemWasEnabled = true;
      }
    }
  }
}


/*
checkN2Compressor
  scheduled task
  rules:
    Enable compressor if low pressure N2 is > minimum
    TODO: Decide if it is necessary to check if low pressure N2 is too high.
    TODO: Anything else needing to be changed here?
    TODO: Check HIGH/LOW valve states
*/
void checkN2Compressor() {
  if (scaledLowN2PSI > n2MinimumPSI) {
    digitalWrite(SSR_Pin, SSR_ON);
    if (!N2compressorRunning) {
      N2compressorRunning = true;
      if (!previousSSR && verboseSerial) { Serial.println(F("Enabled N2 compressor")); };
      previousSSR = true;
    }
  } else {
    digitalWrite(SSR_Pin, SSR_OFF);
    if (N2compressorRunning) {
      N2compressorRunning = false;
      if (previousSSR && verboseSerial) { Serial.println(F("Disabled N2 compressor")); };
      previousSSR = false;
    }
  }
}

void showScaling(uint8_t hexValue, int rawValue, uint16_t scaledValue) {
  sprintf(sprintfBuffer, "%0X %0d %0d", hexValue, rawValue, scaledValue);
  Serial.println(sprintfBuffer);
}

// TODO: Verify correct hex values
const uint8_t rotaryOff = 0x01;        // 0x01 0x04 1-A OFF
const uint8_t rotarySupply = 0x02;     // 0x02 0x0C 1-B Air Supply
const uint8_t rotaryLeft = 0x04;       // 0x04 0x14 1-C Left Tower
const uint8_t rotaryRight = 0x08;      // 0x08 0x1C 1-D Right Tower
const uint8_t rotaryN2Low = 0x10;      // 0x10 0x24 1-E Low Pressure N2
const uint8_t rotaryN2Percent = 0x20;  // 0x20 0x2C 1-F N2 percent


void displayButtonValue() {
  rotarySwitchStatus = readRotarySwitch();
  disp4.setHex(rotarySwitchStatus, false);
  Serial.println(display4buffer);
  Serial.println(millis());
}
/*
displaySelectedValue()
*/
void displaySelectedValue() {
  rotarySwitchStatus = readRotarySwitch();
  switch (rotarySwitchStatus) {
    case rotaryOff:  // 0 -- off
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
scanI2C() -- tries all possible 7-bit I2C addresses
    The i2c_scanner uses the return value of
    the Write.endTransmisstion to see if
    a device did acknowledge the address.
*/
void scanI2C() {
  uint8_t error, address;
  int nDevices = 0;
  Serial.println(F("Scanning I2C devices..."));
  for (address = 1; address <= 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      sprintf(sprintfBuffer, "I2C device found at address %02X.\n", address);
      Serial.print(sprintfBuffer);
      nDevices++;
    } else {
      sprintf(sprintfBuffer, "Unknown I2C error %02X at address %02X.", error, address);
      Serial.println(sprintfBuffer);
    }
  }
  sprintf(sprintfBuffer, "Scan found %d I2C devices.", nDevices);
  Serial.println(sprintfBuffer);
}  // scanI2C

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

  // dtostrf(scaledSupplyPressure, 5, 1, dtostrfBuf1);
  // sprintf(LCDline0, template0, dtostrfBuf1);
  // display20x4.setCursor(0, 0);
  // display20x4.print(LCDline0);

  // dtostrf(scaledLeftPressure, 5, 1, dtostrfBuf1);
  // dtostrf(scaledRightPressure, 5, 1, dtostrfBuf2);
  // sprintf(LCDline1, template1, dtostrfBuf1, dtostrfBuf2);
  // display20x4.setCursor(0, 1);
  // display20x4.print(LCDline1);

  // dtostrf(scaledLowN2Pressure, 5, 2, dtostrfBuf1);
  // sprintf(LCDline2, template2, dtostrfBuf1);
  // display20x4.setCursor(0, 2);
  // display20x4.print(LCDline2);

  // dtostrf(N2portion, 5, 2, dtostrfBuf1);
  // sprintf(LCDline3, template3, dtostrfBuf1);
  // display20x4.setCursor(0, 3);
  // display20x4.print(LCDline3);
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
schedulerLoop()
This runs frequently (main loop())
millis() starts at zero each time the Arduino board is powered up or reset.
It counts the number of milliseconds since the sketch began running or since
the board was last reset. The value increments every millisecond and will
eventually roll over (wraparound) after approximately 49.7 days
(when it reaches 4294967295 (2^32) milliseconds, the maximum value for an unsigned long).
TODO: Decide how to handle the wraparound (shutdown does not fix it, but power down or reset will).
*/
void schedulerLoop() {
  uint32_t now = millis();
  uint32_t timerStart = micros();
  unsigned int timerDuration;
  uint8_t nTasksRun = 0;
  for (uint8_t i = 0; i < NUM_TASKS; i++) {
    if (now - taskList[i].lastRun >= taskList[i].interval) {  // is it due to run?
      taskList[i].lastRun = now;
      taskList[i].callback();                 // dispatch
      nTasksRun++;                            // Here after a runnable task completes
      timerDuration = micros() - timerStart;  // higher precsion for timing
      now = millis();
      if (verboseSerial) {
        sprintf(sprintfBuffer, "Finished task %s in %u μs", taskName[i], timerDuration);
        Serial.println(sprintfBuffer);
      }
    }
  }
  // here after all runnable tasks have completed
  if (verboseSerial) {
    if (nTasksRun > 0) {
      sprintf(sprintfBuffer, "Finished %d task(s)", nTasksRun);
      Serial.println(sprintfBuffer);
    }
  }
}

/*
shutdown()
*/
void shutdown() {
  if (systemWasEnabled) { Serial.println(F("Black switch off; Scheduler disabled; Shutting down.")); };
  closeLeftTowerValve();
  closeRightTowerValve();
  digitalWrite(SSR_Pin, SSR_OFF);
  previousSSR = false;
  disableDisplay4();
  disableDisplay20x4();

  for (uint8_t i = 0; i < NUM_TASKS; i++) {
    taskList[i].lastRun = 0;  // make everything runnable after restart
  }
  systemWasEnabled = false;
  // TODO: what else should be done to shutdown?
}  // shutdown


void enableDisplay20x4() {
  display20x4.begin();
  display20x4.backlightOn();
  display20x4.displayOn();
}
void disableDisplay20x4() {
  display20x4.displayOff();
  display20x4.backlightOff();
}


void enableDisplay4() {
  disp4.displayOn();
  disp4.setBrightness(DISP4_BRIGHTNESS);
}

void disableDisplay4() {
  disp4.displayOff();
}


void waitForO2Sensor() {
  o2SensorReady = false;
  while (!oxygen.begin()) {
  Serial.print("TCP0465 begin() failed: ");
  Serial.println(oxygen.errorString());
  delay(1000);
}
  o2SensorReady = true;
  Serial.println(F("Oxygen I2c connect success."));
}

/* ---------- Arduino setup ---------- */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(leftTowerPressurePin, INPUT);
  pinMode(rightTowerPressurePin, INPUT);
  pinMode(lowPressureN2Pin, INPUT);
  pinMode(leftTowerValvePin, OUTPUT);
  pinMode(rightTowerValvePin, OUTPUT);
  pinMode(SSR_Pin, OUTPUT);
  pinMode(O2SamplePin, OUTPUT);
  pinMode(blackSwitchPin, INPUT);  // TODO decide if this wants to be INPUT_PULLUP

  Serial.begin(9600);               // handshake with USB
  analogReadResolution(bitsOfADC);  //
  Wire.setClock(50000);             // slow down the bus
  Wire.begin();
  enableDisplay4();
  enableDisplay20x4();

  while (!Serial) { delay(1); };  // wait until Serial is available

  sprintf(sprintfBuffer, "N2 v %s, compiled %s at %s with IDE %d", PROGRAM_VERSION, __DATE__, __TIME__, ARDUINO);
  Serial.println(sprintfBuffer);

  scanI2C();  // scan for I2C devices if necessary
  //while(1); //hang

  systemWasEnabled = false;
  shutdown();  // start off in a known state.
  //waitForO2Sensor();

  Serial.println(F("Scheduler started"));
}  // end of setup()



/* ---------- Arduino main loop ---------- */
/*
This is a tight loop, which runs continuously when the scheduler isn't performing a scheduled task.
If the black switch is off, it only runs the loop every ten seconds, but it could run continuously
*/
void loop() {
  readBlackSwitch();
  if (systemEnabled) {
    displaySelectedValue();
    schedulerLoop();  // scheduler checks taskList for all runnable tasks
  } else {
    shutdown();   // TODO can this run once per tight loop?
    delay(1000);  // check occassionally
  }
  Serial.println("Type any character to comtinue...");
  while (!Serial.available()) { delay(1); }
  Serial.readString();  // wait for any character
  Serial.println("OK, proceeding.");
}

/*
// production integration snippet v1
#include "TimedStateMachine.h"
#include "TowerController.h"

class ArduinoClock : public IClock {
public:
  uint32_t nowMs() const override {
    return millis();
  }
};

class SketchTowerValveDriver : public ITowerValveDriver {
public:
  SketchTowerValveDriver(uint8_t leftPin, uint8_t rightPin)
      : leftPin_(leftPin), rightPin_(rightPin) {}

  void begin() {
    pinMode(leftPin_, OUTPUT);
    pinMode(rightPin_, OUTPUT);
    setLeftOpen(false);
    setRightOpen(false);
  }

  void setLeftOpen(bool open) override {
    digitalWrite(leftPin_, open ? HIGH : LOW);
  }

  void setRightOpen(bool open) override {
    digitalWrite(rightPin_, open ? HIGH : LOW);
  }

private:
  uint8_t leftPin_;
  uint8_t rightPin_;
};

ArduinoClock towerClock;
SketchTowerValveDriver towerValves(LEFT_VALVE_PIN, RIGHT_VALVE_PIN);
TowerController towerController(towerClock, towerValves);

void setup() {
  towerValves.begin();
}

void loop() {
  const bool masterOn = readMasterTowerSwitchSomehow();

  towerController.setEnabled(masterOn);
  towerController.tick();
}

*/



// EOF
