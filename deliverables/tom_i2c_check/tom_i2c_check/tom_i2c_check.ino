// ===========================================================================================
// tom_i2c_check  VERSION 1.5   (2026-10-08)     <-- if you do not see this line, the IDE has an older copy
//                                                   (minor versions are written in HEX: 1.A = 1.10)
//
// Tests the three I2C parts of the N2 generator on the REAL I2C bus: the 20x4 LCD, the DS3231 real-time clock (RTC) and the 4-digit LED display.
// One self-contained file: no libraries except the Arduino core's Wire. READ README.txt FIRST (wiring checklist, A2 bridge, what to expect).
//
// WHAT IT TOUCHES: the two I2C pins (A4 = SDA, A5 = SCL), and it READS the two switch inputs D0 (TBS) and D1 (TOB) with their pull-ups on, exactly like the
// generator program. It never drives any pin as an output: not the valves, not the compressor SSR. The O2 sensor (0x74) is only QUERIED: it is sent
// the same read-only question the generator program asks (command 0x86, "read the concentration") and never any command that changes a setting (no mode,
// no address, no threshold). Still, switch the machine's air and compressor OFF while this runs.
//
//   * It needs NO Serial Monitor: everything is on the LCD and the LED. (The Serial Monitor, 115200 baud, shows the same in more detail and is optional.)
//   * At power-up it runs the POST by itself (about 3 s): a status for each device. Then the TEST starts by itself and REPEATS every minute.
//   * The RTC is set from the time this sketch was COMPILED (your computer's clock at that moment) if it lost power or is behind that time.
//   * The TEST (about 85 s): an LCD step (full-screen patterns, backlight blink, display blink), an RTC step (the clock must advance in step with
//     the board's own timer), an LED step (8888 with one dot, a count 0-9 on all four digits, one blink), an O2 step (a read-only query of the O2
//     sensor, if it is fitted), then a TBS step, a TOB step and a RESET step (you flip
//     the switch ON and OFF / press and release TOB / press RESET within 20 s each). Any I2C write error during a step is a FAIL.
//     What the LCD and LED SHOULD show is written in README.txt; you judge it by eye.
//
// LCD:  row 0 name and version   row 1 one status per device   row 2 the RTC date and time   row 3 what the test is doing / the result
// LED:  the time HHMM with the middle dot blinking once a second (or the seconds since power-up if the RTC is not set); FFFF alternates with it if a device FAILED
// Row 1 codes:  + ok   i info (noted, not a problem)   F FAIL   - not tested yet.        Device names: LCD RTC LED O2
// Row 0 shows the switches live: TBS1 = TBS is ON, TOB1 = TOB is pressed (0 = off / released).
// Result row (row 3 when the TEST is done, alternating every 3 s with the switch result):  "TEST LCD? RTCP LED?", "TEST O2P TBSP TOB?" and "TEST RST?"
//   P = passed   F = FAILED   ? = no I2C error / nothing seen, but not confirmed (nobody looked, or nobody touched the switch)
//
// Serial Monitor commands (optional, 115200 baud):  help  status  scan  time  post  run  lcd  rtc  led  o2  tbs  tob  reset  stop  note <text>
//   During a step that asks, type  p (looks right)  or  f (looks wrong, add a note)  to record what you saw; r repeats the step.
//
// Change log
//   1.5  from the first bench log: the banner (with the last reset cause) prints once when the Serial Monitor connects on the Minima, and at most 10 times
//        on the WiFi board; p / f typed when no step is asking get a helpful reply; a step that was not exercised (O2 not fitted, switch not touched,
//        RESET not pressed) prints '?' with its reason
//   1.4  RESET test: press the RESET pushbutton within 20 s; the program restarts and at the next boot reports whether the reset cause is the button. The
//        marker that survives the restart is one spare byte inside the DS3231 (alarm-2 minutes register, alarms are off). The boot always logs the reset cause
//        and the raw reset-status registers. Command  reset.
//   1.3  O2 sensor test (read-only): the same query the generator program uses (command 0x86, read the concentration); reports the O2 % and checks
//        the reply (header, checksum, gas type = O2, a plausible value; exactly 0 is a fault). Command  o2.
//   1.2  TBS and TOB tests: the two switches are read (D0, D1, with pull-ups, like the generator program) and shown live on LCD row 1; two new test
//        steps wait 20 s for you to flip TBS ON and OFF / press and release TOB; commands tbs and tob run them alone
//   1.1  new commands lcd, rtc, led (one device only) next to run (all three); shorter scan lines
//   1.0  first version
// ===========================================================================================
#include <Arduino.h>
#include <Wire.h>

#define SKETCH_VERSION "1.5"

// ---- Types are declared first on purpose: the Arduino IDE inserts automatic function prototypes above the first function, and a prototype that
// ---- mentions DateTime or Verdict fails to compile if the type is defined further down.
struct DateTime {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
};
enum Level : uint8_t { LV_NONE, LV_PASS, LV_INFO, LV_FAIL };
enum Verdict : uint8_t { V_NONE, V_PASS, V_FAIL, V_SKIP, V_UNSURE };  // V_UNSURE: no I2C error, but nobody answered p/f
enum BistStep : uint8_t { B_LCD, B_RTC, B_LED, B_O2, B_TBS, B_TOB, B_RST, B_COUNT };

// ---------------------------------------------------------------------------------------------------------------------------
// Addresses (7-bit). The LCD backpack's A2 solder pad is BRIDGED, so it is at 0x23: the LED module also answers 0x24-0x27, which a backpack left
// at its default 0x27 would collide with.
// ---------------------------------------------------------------------------------------------------------------------------
static const uint8_t ADDR_LCD = 0x23;      // PCF8574 backpack (A2 bridged)
static const uint8_t ADDR_RTC = 0x68;      // DS3231 (its module also has an EEPROM at 0x57: not used)
static const uint8_t ADDR_LED_CTL = 0x24;  // TM1650 control
static const uint8_t ADDR_LED_DIG = 0x34;  // TM1650 digits 0..3 at 0x34..0x37
static const uint8_t ADDR_O2 = 0x74;       // O2 sensor: only probed, never written

static const uint32_t LCD_START_MS = 2500;  // the LCD is not touched until this long after power-up (bench finding: earlier starts garble it)
static const uint8_t PIN_TBS = 0;  // The Black Switch: maintained, INPUT_PULLUP, ON = LOW (as in the generator program)
static const uint8_t PIN_TOB = 1;  // The Other Button: momentary, INPUT_PULLUP, pressed = LOW
static const uint8_t SDA_PIN = A4;
static const uint8_t SCL_PIN = A5;

// ---------------------------------------------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------------------------------------------
static const char kLevelChar[4] = {'-', '+', 'i', 'F'};
static const char* kLevelWord[4] = {"-", "ok", "info", "FAIL"};

static void say(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void say(const char* fmt, ...) {
  char buf[120];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  Serial.println(buf);
}

static bool reached(uint32_t now, uint32_t at) { return static_cast<int32_t>(now - at) >= 0; }  // safe across the millis() rollover

// ---------------------------------------------------------------------------------------------------------------------------
// I2C access, with bus recovery. A reset in the middle of a transaction can leave the bus stuck; clocking SCL nine times and sending a STOP frees
// it. On the UNO R4 only Wire.setClock() reopens the controller after Wire.end(), so it is always called.
// ---------------------------------------------------------------------------------------------------------------------------
static uint32_t gBusRecoveries = 0;

static void i2cRecover() {
  Wire.end();
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  delayMicroseconds(10);
  for (uint8_t pulse = 0; pulse < 9 && digitalRead(SDA_PIN) == LOW; ++pulse) {
    pinMode(SCL_PIN, OUTPUT);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);
    pinMode(SCL_PIN, INPUT);
    delayMicroseconds(10);
  }
  pinMode(SDA_PIN, OUTPUT);  // STOP: SDA rises while SCL is high
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(10);
  pinMode(SDA_PIN, INPUT);
  delayMicroseconds(10);
  Wire.begin();
  Wire.setClock(100000);
  ++gBusRecoveries;
}

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool i2cWrite(uint8_t addr, const uint8_t* data, uint8_t n) {
  Wire.beginTransmission(addr);
  for (uint8_t i = 0; i < n; ++i) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

static bool i2cWrite1(uint8_t addr, uint8_t value) { return i2cWrite(addr, &value, 1); }

static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) out[i] = Wire.read();
  return true;
}

// ---------------------------------------------------------------------------------------------------------------------------
// Calendar helpers (years 2000-2099, what a DS3231 holds)
// ---------------------------------------------------------------------------------------------------------------------------

static bool isLeap(uint16_t y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }
static uint8_t daysInMonth(uint16_t y, uint8_t m) {
  static const uint8_t d[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (m < 1 || m > 12) return 0;
  return (m == 2 && isLeap(y)) ? 29 : d[m - 1];
}
static bool validDateTime(const DateTime& t) {
  return t.year >= 2000 && t.year <= 2099 && t.month >= 1 && t.month <= 12 && t.day >= 1 && t.day <= daysInMonth(t.year, t.month) && t.hour < 24 &&
         t.minute < 60 && t.second < 60;
}
static uint32_t secondsSince2000(const DateTime& t) {
  uint32_t days = 0;
  for (uint16_t y = 2000; y < t.year; ++y) days += isLeap(y) ? 366 : 365;
  for (uint8_t m = 1; m < t.month; ++m) days += daysInMonth(t.year, m);
  days += t.day - 1u;
  return days * 86400u + t.hour * 3600u + t.minute * 60u + t.second;
}
static void formatDateTime(char* out, const DateTime& t) {  // out needs 20 bytes
  snprintf(out, 20, "%04u-%02u-%02u %02u:%02u:%02u", t.year % 10000u, t.month % 100u, t.day % 100u, t.hour % 100u, t.minute % 100u, t.second % 100u);
}
static uint8_t isoWeekday(const DateTime& t) {  // Monday = 1 .. Sunday = 7; 2000-01-01 was a Saturday
  return static_cast<uint8_t>((secondsSince2000(t) / 86400u + 5u) % 7u + 1u);
}

// The time this sketch was compiled, from __DATE__ ("Oct  8 2026") and __TIME__ ("10:52:19"): the clock of the computer that compiled it.
static bool buildTime(DateTime& t) {
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* d = __DATE__;
  const char* tm = __TIME__;
  t.month = 0;
  for (uint8_t m = 0; m < 12; ++m)
    if (d[0] == months[m * 3] && d[1] == months[m * 3 + 1] && d[2] == months[m * 3 + 2]) t.month = m + 1;
  t.day = static_cast<uint8_t>((d[4] == ' ' ? 0 : d[4] - '0') * 10 + (d[5] - '0'));
  t.year = static_cast<uint16_t>((d[7] - '0') * 1000 + (d[8] - '0') * 100 + (d[9] - '0') * 10 + (d[10] - '0'));
  t.hour = static_cast<uint8_t>((tm[0] - '0') * 10 + (tm[1] - '0'));
  t.minute = static_cast<uint8_t>((tm[3] - '0') * 10 + (tm[4] - '0'));
  t.second = static_cast<uint8_t>((tm[6] - '0') * 10 + (tm[7] - '0'));
  return validDateTime(t);
}

// ---------------------------------------------------------------------------------------------------------------------------
// DS3231 real-time clock
// ---------------------------------------------------------------------------------------------------------------------------
static uint8_t bcdToDec(uint8_t b) { return static_cast<uint8_t>((b >> 4) * 10 + (b & 0x0F)); }
static uint8_t decToBcd(uint8_t d) { return static_cast<uint8_t>(((d / 10) << 4) | (d % 10)); }

static bool rtcRead(DateTime& t) {
  uint8_t r[7];
  if (!i2cReadReg(ADDR_RTC, 0x00, r, 7)) return false;
  for (uint8_t i = 0; i < 7; ++i) {
    const uint8_t v = (i == 2) ? static_cast<uint8_t>(r[i] & 0x3F) : (i == 5 ? static_cast<uint8_t>(r[i] & 0x1F) : r[i]);
    if ((v >> 4) > 9 || (v & 0x0F) > 9) return false;  // not valid BCD
  }
  if (r[2] & 0x40) return false;  // 12-hour mode: we only write 24-hour
  t.second = bcdToDec(r[0] & 0x7F);
  t.minute = bcdToDec(r[1] & 0x7F);
  t.hour = bcdToDec(r[2] & 0x3F);
  t.day = bcdToDec(r[4] & 0x3F);
  t.month = bcdToDec(r[5] & 0x1F);
  t.year = static_cast<uint16_t>(2000 + bcdToDec(r[6]) + ((r[5] & 0x80) ? 100 : 0));
  return validDateTime(t);
}

static bool rtcTrusted(bool& trusted) {  // trusted = the oscillator-stop flag is clear (the clock has not lost power since it was set)
  uint8_t s;
  if (!i2cReadReg(ADDR_RTC, 0x0F, &s, 1)) return false;
  trusted = (s & 0x80) == 0;
  return true;
}

static bool rtcSet(const DateTime& t) {
  uint8_t buf[8] = {0x00, decToBcd(t.second), decToBcd(t.minute), decToBcd(t.hour), isoWeekday(t), decToBcd(t.day), decToBcd(t.month),
                    decToBcd(static_cast<uint8_t>(t.year - 2000))};
  if (!i2cWrite(ADDR_RTC, buf, 8)) return false;
  uint8_t status;
  if (!i2cReadReg(ADDR_RTC, 0x0F, &status, 1)) return false;
  const uint8_t clear[2] = {0x0F, static_cast<uint8_t>(status & ~0x80)};  // clear the oscillator-stop flag
  return i2cWrite(ADDR_RTC, clear, 2);
}

static bool rtcTemperatureX100(int16_t& out) {
  uint8_t r[2];
  if (!i2cReadReg(ADDR_RTC, 0x11, r, 2)) return false;
  out = static_cast<int16_t>(static_cast<int8_t>(r[0]) * 100 + (r[1] >> 6) * 25);
  return true;
}

// ---------------------------------------------------------------------------------------------------------------------------
// TM1650 4-digit LED display
// ---------------------------------------------------------------------------------------------------------------------------
static const uint8_t kHexSeg[16] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};
static uint8_t segmentsFor(char c) {
  if (c >= '0' && c <= '9') return kHexSeg[c - '0'];
  if (c >= 'A' && c <= 'F') return kHexSeg[10 + (c - 'A')];
  if (c == '-') return 0x40;
  return 0x00;
}

static uint8_t gLedSeg[4] = {0, 0, 0, 0};  // what we want shown
static bool gLedOn = true;
static bool gLedOk = false;
static uint32_t gLedErrors = 0;
static uint32_t gLedNext = 0;

static void ledText(const char* four, int8_t dotAfter) {
  for (uint8_t i = 0; i < 4; ++i) gLedSeg[i] = static_cast<uint8_t>(segmentsFor(four[i]) | (dotAfter == static_cast<int8_t>(i) ? 0x80 : 0));
}

// Rewrites the control byte and all four digits four times a second: a missing or confused module is noticed within a second and repaired.
static void ledService(uint32_t now) {
  if (!reached(now, gLedNext)) return;
  gLedNext = now + 250;
  bool ok = i2cWrite1(ADDR_LED_CTL, gLedOn ? 0x11 : 0x10);  // brightness 1, 8-segment mode, display on/off
  for (uint8_t i = 0; ok && i < 4; ++i) ok = i2cWrite1(static_cast<uint8_t>(ADDR_LED_DIG + i), gLedSeg[i]);
  if (!ok) {
    ++gLedErrors;
    if (gLedOk) say("LED: write failed (error #%lu)", static_cast<unsigned long>(gLedErrors));
    i2cRecover();
  } else if (!gLedOk) {
    say("LED: answering");
  }
  gLedOk = ok;
}

// ---------------------------------------------------------------------------------------------------------------------------
// 20x4 LCD (HD44780 behind a PCF8574 backpack: RS = bit 0, EN = bit 2, backlight = bit 3, D4-D7 = bits 4-7)
// ---------------------------------------------------------------------------------------------------------------------------
static const uint8_t LCD_RS = 0x01, LCD_EN = 0x04, LCD_BL = 0x08;
static char gRow[4][21];     // what we want shown (20 characters + NUL)
static bool gLcdBacklight = true;
static bool gLcdDisplayOn = true;
static bool gLcdReady = false;
static bool gLcdOk = true;
static uint32_t gLcdErrors = 0;
static uint32_t gLcdRetryAt = 0;
static uint8_t gLcdNextRow = 0;

static bool lcdNibble(uint8_t nibbleInHighBits, bool rs) {
  const uint8_t v = static_cast<uint8_t>((nibbleInHighBits & 0xF0) | (gLcdBacklight ? LCD_BL : 0) | (rs ? LCD_RS : 0));
  const uint8_t bytes[2] = {static_cast<uint8_t>(v | LCD_EN), v};  // enable pulse
  return i2cWrite(ADDR_LCD, bytes, 2);
}

static bool lcdByte(uint8_t value, bool rs) {  // two nibbles in ONE transaction
  const uint8_t hi = static_cast<uint8_t>((value & 0xF0) | (gLcdBacklight ? LCD_BL : 0) | (rs ? LCD_RS : 0));
  const uint8_t lo = static_cast<uint8_t>(((value << 4) & 0xF0) | (gLcdBacklight ? LCD_BL : 0) | (rs ? LCD_RS : 0));
  const uint8_t bytes[4] = {static_cast<uint8_t>(hi | LCD_EN), hi, static_cast<uint8_t>(lo | LCD_EN), lo};
  return i2cWrite(ADDR_LCD, bytes, 4);
}

static bool lcdInit() {  // the standard HD44780 4-bit initialisation; takes about 20 ms
  bool ok = true;
  ok = ok && lcdNibble(0x30, false);
  delay(5);
  ok = ok && lcdNibble(0x30, false);
  delay(5);
  ok = ok && lcdNibble(0x30, false);
  delay(1);
  ok = ok && lcdNibble(0x20, false);  // switch to 4-bit mode
  delay(1);
  ok = ok && lcdByte(0x28, false);    // 4-bit, 2 lines, 5x8 font
  ok = ok && lcdByte(gLcdDisplayOn ? 0x0C : 0x08, false);
  ok = ok && lcdByte(0x06, false);    // entry mode: increment
  ok = ok && lcdByte(0x01, false);    // clear
  delay(3);
  return ok;
}

static void lcdFail(uint32_t now) {
  gLcdOk = false;
  ++gLcdErrors;
  gLcdReady = false;
  gLcdRetryAt = now + 1000;
}

// One row per call (about 10 ms): the whole screen is rewritten every four calls, so a corrupted cell never stays wrong for more than a moment.
static void lcdService(uint32_t now) {
  if (now < LCD_START_MS) return;  // leave the LCD alone for the first 2.5 s after power-up
  if (!gLcdReady) {
    if (!reached(now, gLcdRetryAt)) return;
    if (!gLcdOk) i2cRecover();  // the cause may be the bus rather than the display
    if (lcdInit()) {
      gLcdReady = true;
      if (!gLcdOk) say("LCD: answering again");
      gLcdOk = true;
    } else {
      lcdFail(now);
    }
    return;
  }
  static const uint8_t rowAddress[4] = {0x00, 0x40, 0x14, 0x54};
  const uint8_t r = gLcdNextRow;
  gLcdNextRow = static_cast<uint8_t>((gLcdNextRow + 1) & 3);
  if (!lcdByte(static_cast<uint8_t>(0x80 | rowAddress[r]), false)) return lcdFail(now);
  for (uint8_t c = 0; c < 20; ++c)
    if (!lcdByte(static_cast<uint8_t>(gRow[r][c]), true)) return lcdFail(now);
}

static void lcdSetRow(uint8_t row, const char* text) {  // pads or clips to 20 columns
  uint8_t i = 0;
  for (; i < 20 && text[i] != '\0'; ++i) gRow[row][i] = text[i];
  for (; i < 20; ++i) gRow[row][i] = ' ';
  gRow[row][20] = '\0';
}

static void lcdSetBacklight(bool on) {
  gLcdBacklight = on;
  if (gLcdReady) i2cWrite1(ADDR_LCD, on ? LCD_BL : 0);
}

static void lcdSetDisplay(bool on) {
  gLcdDisplayOn = on;
  if (gLcdReady && !lcdByte(on ? 0x0C : 0x08, false)) lcdFail(millis());
}

// ---------------------------------------------------------------------------------------------------------------------------
// POST: one quick hands-off check per device
// ---------------------------------------------------------------------------------------------------------------------------
struct Result {
  Level level = LV_NONE;
  char text[48] = "";
};
enum { DEV_LCD, DEV_RTC, DEV_LED, DEV_O2, DEV_COUNT };
static const char* kDevName[DEV_COUNT] = {"LCD", "RTC", "LED", "O2"};
static Result gPost[DEV_COUNT];
static bool gAutoRun = true;        // the test starts by itself after POST and repeats every minute (no keyboard needed)
static uint32_t gAutoAt = 0;
static bool gPostDone = false;
static uint8_t gPostStep = 0;
static uint32_t gPostAt = 0;
static bool gRtcSetNote = false;  // the RTC was set from the compile time during this POST

static void setResult(uint8_t dev, Level level, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
static void setResult(uint8_t dev, Level level, const char* fmt, ...) {
  gPost[dev].level = level;
  va_list args;
  va_start(args, fmt);
  vsnprintf(gPost[dev].text, sizeof gPost[dev].text, fmt, args);
  va_end(args);
  say("POST %d/4 %-3s %-4s %s", dev + 1, kDevName[dev], kLevelWord[level], gPost[dev].text);
}

static void postLcd() {
  if (i2cProbe(ADDR_LCD)) setResult(DEV_LCD, LV_PASS, "answers at 0x%02X", ADDR_LCD);
  else setResult(DEV_LCD, LV_FAIL, "no answer at 0x%02X (is A2 bridged? power? wires?)", ADDR_LCD);
}

static void postRtc() {
  if (!i2cProbe(ADDR_RTC)) {
    setResult(DEV_RTC, LV_FAIL, "no answer at 0x%02X", ADDR_RTC);
    return;
  }
  DateTime now, built;
  bool trusted = false, haveTrust = rtcTrusted(trusted);
  const bool readable = rtcRead(now);
  const bool haveBuilt = buildTime(built);
  // Set it from the compile time if it lost power (stopped oscillator), cannot be read, or is behind the time this sketch was compiled.
  const bool needsSet = haveBuilt && (!haveTrust || !trusted || !readable || secondsSince2000(now) < secondsSince2000(built));
  if (needsSet) {
    char was[20] = "unreadable";
    if (readable) formatDateTime(was, now);
    if (rtcSet(built) && rtcRead(now)) {
      char b[20];
      formatDateTime(b, now);
      gRtcSetNote = true;
      setResult(DEV_RTC, LV_INFO, "was %s; SET from compile time to %s", was, b);
    } else {
      setResult(DEV_RTC, LV_FAIL, "was %s and could not be set", was);
    }
    return;
  }
  char b[20];
  if (readable) formatDateTime(b, now);
  else snprintf(b, sizeof b, "unreadable");
  setResult(DEV_RTC, readable ? LV_PASS : LV_FAIL, "%s", b);
}

static void postLed() {
  uint8_t n = i2cProbe(ADDR_LED_CTL) ? 1 : 0;
  for (uint8_t d = 0; d < 4; ++d)
    if (i2cProbe(static_cast<uint8_t>(ADDR_LED_DIG + d))) ++n;
  if (n == 5) setResult(DEV_LED, LV_PASS, "0x%02X + digits 0x%02X-0x%02X answer", ADDR_LED_CTL, ADDR_LED_DIG, ADDR_LED_DIG + 3);
  else setResult(DEV_LED, LV_FAIL, "only %u of 5 addresses answer", n);
}

static void postO2() {
  if (i2cProbe(ADDR_O2)) setResult(DEV_O2, LV_PASS, "answers at 0x%02X (the O2 test reads it; nothing is changed)", ADDR_O2);
  else setResult(DEV_O2, LV_INFO, "not found at 0x%02X (not part of this test)", ADDR_O2);
}

static void postBegin(uint32_t now) {
  for (uint8_t i = 0; i < DEV_COUNT; ++i) gPost[i] = Result();
  gPostDone = false;
  gPostStep = 0;
  gPostAt = now + 300;
  gRtcSetNote = false;
}

static void postService(uint32_t now) {
  if (gPostDone || !reached(now, gPostAt)) return;
  switch (gPostStep) {
    case DEV_LCD: postLcd(); break;
    case DEV_RTC: postRtc(); break;
    case DEV_LED: postLed(); break;
    case DEV_O2: postO2(); break;
  }
  ++gPostStep;
  gPostAt = now + 150;
  if (gPostStep >= DEV_COUNT) {
    gPostDone = true;
    bool anyFail = false;
    for (uint8_t i = 0; i < DEV_COUNT; ++i) anyFail = anyFail || gPost[i].level == LV_FAIL;
    say(anyFail ? "POST: PROBLEMS FOUND (see FAIL lines above)" : "POST OK. The LCD / RTC / LED test starts by itself in 3 s.");
    gAutoAt = now + 3000;
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
// O2 sensor (DFRobot SEN0465): ONE read-only query. Frame (9 bytes): FF 01 <command> 00 00 00 00 00 <checksum>, written after a register byte 0;
// the 9-byte reply is read back the same way (register 0, then 9 bytes). Command 0x86 = "read the concentration". The checksum is
// (~(sum of bytes 1..7)) + 1. Reply: FF 86 <conc hi> <conc lo> <gas type> <decimals> .. .. <checksum>; gas type 0x05 = O2.
// Nothing here changes the sensor: the commands that change a setting (0x78 mode, 0x89 thresholds, 0x92 I2C address) are never sent.
// ---------------------------------------------------------------------------------------------------------------------------
static uint8_t gO2Raw[9];      // the last reply bytes, printed in the log so a problem can be diagnosed from the log alone
static bool gO2RawValid = false;

static uint8_t o2Checksum(const uint8_t* f) {
  uint8_t sum = 0;
  for (uint8_t i = 1; i < 8; ++i) sum = static_cast<uint8_t>(sum + f[i]);
  return static_cast<uint8_t>((~sum) + 1);
}

// Returns true with the O2 percent x100 in `hundredths` if the sensor answered with a valid reply; otherwise false and `why` says what was wrong.
static bool o2Query(uint16_t& hundredths, uint8_t& gasType, const char*& why) {
  uint8_t frame[9] = {0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0};
  frame[8] = o2Checksum(frame);
  Wire.beginTransmission(ADDR_O2);
  Wire.write(static_cast<uint8_t>(0x00));  // register 0
  for (uint8_t i = 0; i < 9; ++i) Wire.write(frame[i]);
  if (Wire.endTransmission() != 0) {
    why = "no answer to the query";
    return false;
  }
  delay(10);  // the sensor needs a moment to prepare its reply
  Wire.beginTransmission(ADDR_O2);
  Wire.write(static_cast<uint8_t>(0x00));
  if (Wire.endTransmission() != 0) {
    why = "no answer when reading the reply";
    return false;
  }
  uint8_t r[9];
  if (Wire.requestFrom(ADDR_O2, static_cast<uint8_t>(9)) != 9) {
    why = "short reply";
    return false;
  }
  for (uint8_t i = 0; i < 9; ++i) r[i] = Wire.read();
  memcpy(gO2Raw, r, 9);
  gO2RawValid = true;
  if (r[0] != 0xFF || r[1] != 0x86) {
    why = "reply has the wrong header";
    return false;
  }
  if (o2Checksum(r) != r[8]) {
    why = "reply fails its checksum (link problem)";
    return false;
  }
  gasType = r[4];
  uint32_t value = (static_cast<uint32_t>(r[2]) << 8) | r[3];
  if (r[5] == 0) value *= 100;
  else if (r[5] == 1) value *= 10;  // decimals 2: already hundredths
  hundredths = static_cast<uint16_t>(value > 65535 ? 65535 : value);
  why = "";
  return true;
}

// ---------------------------------------------------------------------------------------------------------------------------
// Reset cause, and the RESET-button test. The RA4M1 keeps the cause of the last reset in three status registers. They persist until cleared, so they are
// read ONCE at boot and cleared; RSTSR2 bit 0 is then set by us, so that any later reset that did not lose power reads "warm" (the power-on flag itself is
// not visible after the bootloader). "Cold" = that bit was 0 = power was lost.
// The test marker: one spare byte in the DS3231 (alarm-2 minutes register 0x0B; the alarms are never enabled) survives a reset AND a power loss.
// ---------------------------------------------------------------------------------------------------------------------------
static const uint8_t RTC_SCRATCH_REG = 0x0B;
static const uint8_t RESET_MARKER = 0xA5;

static bool gResetPowerOn = false, gResetWatchdog = false, gResetBrownout = false;
static char gResetText[24] = "unknown";
static char gResetRaw[40] = "";
static char gResetTestMsg[28] = "";     // the result of a RESET test that ended with this boot, shown for a while on the LCD
static uint32_t gResetTestMsgUntil = 0;
static Verdict gResetTestVerdict = V_NONE;

static void readResetCause() {
  const uint8_t r0 = R_SYSTEM->RSTSR0;
  const uint16_t r1 = R_SYSTEM->RSTSR1;
  const uint8_t r2 = R_SYSTEM->RSTSR2;
  gResetPowerOn = ((r2 & 0x01) == 0) || ((r0 & 0x01) != 0);
  gResetBrownout = (r0 & 0x0E) != 0;
  gResetWatchdog = (r1 & 0x03) != 0;
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
  R_SYSTEM->RSTSR2 = 0x01;
  snprintf(gResetRaw, sizeof gResetRaw, "RSTSR0=%02X RSTSR1=%04X RSTSR2=%02X", r0, r1, r2);
  snprintf(gResetText, sizeof gResetText, "%s",
           gResetPowerOn ? "power-on (cold start)" : (gResetWatchdog ? "watchdog" : (gResetBrownout ? "brown-out" : "reset button/other")));
}

// At boot, after the I2C bus is up: was a RESET test running when this restart happened?
static void resetTestEvaluate(uint32_t now) {
  uint8_t marker = 0;
  if (!i2cReadReg(ADDR_RTC, RTC_SCRATCH_REG, &marker, 1)) return;  // no RTC: no RESET test possible
  if (marker != RESET_MARKER) return;
  const uint8_t clear[2] = {RTC_SCRATCH_REG, 0x00};
  i2cWrite(ADDR_RTC, clear, 2);
  if (gResetPowerOn) {
    gResetTestVerdict = V_UNSURE;
    snprintf(gResetTestMsg, sizeof gResetTestMsg, "RESET test: power lost");
    say("RESET TEST: ? The RESET test was running, but this start was a POWER-ON, not a reset button press (power was removed).");
  } else if (gResetWatchdog || gResetBrownout) {
    gResetTestVerdict = V_FAIL;
    snprintf(gResetTestMsg, sizeof gResetTestMsg, "RESET test: FAIL");
    say("RESET TEST: FAIL The program restarted, but the cause was '%s', not the RESET button.", gResetText);
  } else {
    gResetTestVerdict = V_PASS;
    snprintf(gResetTestMsg, sizeof gResetTestMsg, "RESET test: PASS");
    say("RESET TEST: PASS The RESET button restarted the program (reset cause: %s).", gResetText);
  }
  gResetTestMsgUntil = now + 20000;
}

// ---------------------------------------------------------------------------------------------------------------------------
// TBS and TOB: read with debounce (a level must hold for 30 ms), every change is logged. The test steps below use gSawOn / gSawOff.
// ---------------------------------------------------------------------------------------------------------------------------
static bool gTbsOn = false;
static bool gTobOn = false;
static bool gSawOn = false;   // during a TBS / TOB step: has the ON (pressed) state been seen?
static bool gSawOff = false;  // ... and the OFF (released) state?

static void switchService(uint32_t now) {
  static bool rawTbs = false, rawTob = false;
  static uint32_t tbsSince = 0, tobSince = 0;
  const bool tbs = digitalRead(PIN_TBS) == LOW;
  const bool tob = digitalRead(PIN_TOB) == LOW;
  if (tbs != rawTbs) {
    rawTbs = tbs;
    tbsSince = now;
  } else if (tbs != gTbsOn && now - tbsSince >= 30) {
    gTbsOn = tbs;
    say("TBS %s", tbs ? "ON" : "off");
  }
  if (tob != rawTob) {
    rawTob = tob;
    tobSince = now;
  } else if (tob != gTobOn && now - tobSince >= 30) {
    gTobOn = tob;
    say("TOB %s", tob ? "pressed" : "released");
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
// BIST: guided test. LCD and LED need your eyes (answer p or f); the RTC decides by itself.
// ---------------------------------------------------------------------------------------------------------------------------
static bool gBist = false;
static uint8_t gBistStep = 0;
static uint8_t gPhase = 0;
static uint32_t gPhaseAt = 0;
static bool gAsking = false;
static bool gBlink = true;
static uint32_t gBlinkAt = 0;
static uint8_t gDigit = 0;
static Verdict gVerdict[B_COUNT];
static char gNote[B_COUNT][28];
static DateTime gRtcFirst;
static uint32_t gRtcFirstMs = 0;
static char gKey = 0;       // the operator's answer: p f r s q
static char gKeyNote[28];
static bool gBistFinished = false;
static uint8_t gBistLast = B_COUNT - 1;   // the last step of this run (a single-device command runs just one step)
static uint32_t gAskUntil = 0;      // a step that asks waits this long for an answer, then goes on by itself
static uint32_t gErrAtStart = 0;    // I2C error count of the device under test when the step began

static const char* kBistName[B_COUNT] = {"LCD", "RTC", "LED", "O2", "TBS", "TOB", "RST"};

static bool stepHadErrors() {  // did the device under test report an I2C write error since the step began?
  if (gBistStep == B_LCD) return gLcdErrors > gErrAtStart;
  if (gBistStep == B_LED) return gLedErrors > gErrAtStart;
  return false;
}

static char verdictChar(Verdict v) { return v == V_PASS ? 'P' : (v == V_FAIL ? 'F' : (v == V_UNSURE ? '?' : (v == V_SKIP ? 's' : '-'))); }

static void bistShowRows(const char* a, const char* b, const char* c, const char* d) {
  lcdSetRow(0, a);
  lcdSetRow(1, b);
  lcdSetRow(2, c);
  lcdSetRow(3, d);
}

static void bistEndStep() {  // leave the displays normal after every step, even a quit
  lcdSetBacklight(true);
  lcdSetDisplay(true);
  gLedOn = true;
}

static void bistStartStep(uint32_t now) {
  gPhase = 0;
  gPhaseAt = now;
  gAsking = false;
  gKey = 0;
  gBlink = true;
  gDigit = 0;
  gErrAtStart = (gBistStep == B_LCD) ? gLcdErrors : (gBistStep == B_LED ? gLedErrors : 0);
  say("BIST %d/%d %s", gBistStep + 1, B_COUNT, kBistName[gBistStep]);
  if (gBistStep == B_LCD) {
    bistShowRows("####################", "ABCDEFGHIJKLMNOPQRST", "01234567890123456789", "####################");
    say("   LCD rows: |####################|  |ABCDEFGHIJKLMNOPQRST|  |01234567890123456789|  |####################|");
    gPhaseAt = now + 3000;
  } else if (gBistStep == B_RTC) {
    gRtcFirstMs = now;
    if (rtcRead(gRtcFirst)) {
      char b[20];
      formatDateTime(b, gRtcFirst);
      say("   RTC reads %s; waiting 3 s to see it advance", b);
      gPhase = 1;
      gPhaseAt = now + 3000;
    } else {
      gPhase = 9;  // cannot read: fail at once
    }
  } else if (gBistStep == B_RST) {
    const uint8_t mark[2] = {RTC_SCRATCH_REG, RESET_MARKER};
    if (!i2cWrite(ADDR_RTC, mark, 2)) {
      gPhase = 9;  // no RTC: the marker cannot be stored
    } else {
      gPhase = 1;
      gPhaseAt = now + 20000;
      say("   PRESS THE RESET BUTTON NOW (within 20 s). The program restarts; at the next start it reports the result.");
    }
  } else if (gBistStep == B_O2) {
    gPhaseAt = now;  // runs at once, in bistService
  } else if (gBistStep == B_LED) {
    ledText("8888", 1);
    say("   LED: 8888 with one decimal point (all segments)");
    gPhaseAt = now + 3000;
  } else {  // B_TBS or B_TOB: wait up to 20 s for the switch to be seen in both states
    const bool now_on = (gBistStep == B_TBS) ? gTbsOn : gTobOn;
    gSawOn = now_on;
    gSawOff = !now_on;
    gPhaseAt = now + 20000;
    if (gBistStep == B_TBS) say("   TBS is %s now. Flip TBS ON and then OFF within 20 s.", now_on ? "ON" : "off");
    else say("   TOB is %s now. Press TOB and then release it within 20 s.", now_on ? "pressed" : "released");
  }
}

static void bistConclude(uint32_t now, Verdict v, const char* note) {
  bistEndStep();
  gVerdict[gBistStep] = v;
  snprintf(gNote[gBistStep], sizeof gNote[0], "%s", note);
  say("BIST %d/%d %-3s %s%s%s", gBistStep + 1, B_COUNT, kBistName[gBistStep],
      v == V_PASS ? "PASS" : (v == V_FAIL ? "FAIL" : (v == V_UNSURE ? (note[0] ? "?" : "OK? (no I2C error; judge by eye)") : "skipped")), note[0] ? "  " : "",
      note);
  ++gBistStep;
  if (gBistStep > gBistLast) {
    gBist = false;
    gBistFinished = true;
    uint8_t pass = 0, fail = 0, unsure = 0, skip = 0;
    for (uint8_t i = 0; i < B_COUNT; ++i) {
      if (gVerdict[i] == V_PASS) ++pass;
      else if (gVerdict[i] == V_FAIL) ++fail;
      else if (gVerdict[i] == V_UNSURE) ++unsure;
      else if (gVerdict[i] == V_SKIP) ++skip;
    }
    const bool fullRun = (gBistLast == B_COUNT - 1) && gVerdict[B_LCD] != V_NONE;
    if (gAutoRun && fullRun) {
      say("TEST complete: %u pass, %u fail, %u not confirmed (?), %u skipped. It repeats in 45 s (type stop to end the repeats).", pass, fail, unsure, skip);
      gAutoAt = now + 45000;
    } else {
      say("TEST complete: %u pass, %u fail, %u not confirmed (?), %u skipped.", pass, fail, unsure, skip);
      gAutoAt = 0xFFFFFFFFu;
    }
  } else {
    bistStartStep(now);
  }
}

static void bistService(uint32_t now) {
  if (!gBist) return;
  if (gKey == 'q') {
    bistEndStep();
    gBist = false;
    gBistFinished = true;
    gKey = 0;
    say("BIST quit.");
    return;
  }
  if (gBistStep == B_LCD) {
    if (!gAsking && reached(now, gPhaseAt)) {
      if (gPhase == 0) {
        say("   LCD: backlight blinking");
        gPhase = 1;
        gPhaseAt = now + 3000;
        gBlinkAt = now + 500;
      } else if (gPhase == 1) {
        lcdSetBacklight(true);
        say("   LCD: display blinking (the text should vanish and return)");
        gPhase = 2;
        gPhaseAt = now + 3000;
        gBlinkAt = now + 500;
      } else {
        lcdSetDisplay(true);
        gAsking = true;
        gAskUntil = now + 8000;
        say("   Did you see 4 readable rows, then the backlight blink, then the display blink?  (p = yes, f = no; no answer is fine)");
      }
    }
    if (!gAsking && gPhase >= 1 && reached(now, gBlinkAt)) {
      gBlinkAt = now + 500;
      gBlink = !gBlink;
      if (gPhase == 1) lcdSetBacklight(gBlink);
      else lcdSetDisplay(gBlink);
    }
  } else if (gBistStep == B_RTC) {
    if (gPhase == 9) {
      bistConclude(now, V_FAIL, "cannot read the time");
      return;
    }
    if (reached(now, gPhaseAt)) {
      DateTime second;
      int16_t temp = 0;
      if (!rtcRead(second)) {
        bistConclude(now, V_FAIL, "second read failed");
        return;
      }
      const long rtcSeconds = static_cast<long>(secondsSince2000(second)) - static_cast<long>(secondsSince2000(gRtcFirst));
      const long msSeconds = static_cast<long>((now - gRtcFirstMs) / 1000u);
      const bool haveTemp = rtcTemperatureX100(temp);
      say("   RTC advanced %ld s while the clock of this board advanced %ld s; chip %d.%02d C", rtcSeconds, msSeconds, temp / 100,
          (temp < 0 ? -temp : temp) % 100);
      char note[28];
      if (rtcSeconds - msSeconds > 1 || msSeconds - rtcSeconds > 1) {
        snprintf(note, sizeof note, "clock off by %ld s", rtcSeconds - msSeconds);
        bistConclude(now, V_FAIL, note);
      } else if (!haveTemp || temp < 0 || temp > 6000) {
        bistConclude(now, V_FAIL, "temperature implausible");
      } else {
        snprintf(note, sizeof note, "%d.%02d C", temp / 100, temp % 100);
        bistConclude(now, V_PASS, note);
      }
      return;
    }
  } else if (gBistStep == B_RST) {
    if (gPhase == 9) {
      bistConclude(now, V_UNSURE, "needs the RTC (marker)");
      return;
    }
    if (reached(now, gPhaseAt)) {
      const uint8_t clear[2] = {RTC_SCRATCH_REG, 0x00};
      i2cWrite(ADDR_RTC, clear, 2);  // nobody pressed it: take the marker away again
      bistConclude(now, V_UNSURE, "not pressed");
      return;
    }
  } else if (gBistStep == B_O2) {
    if (!i2cProbe(ADDR_O2)) {
      say("   O2 sensor: no answer at 0x%02X (not fitted? it is not part of the I2C parts test)", ADDR_O2);
      bistConclude(now, V_UNSURE, "not found");
      return;
    }
    uint16_t hundredths = 0;
    uint8_t gas = 0;
    const char* why = "";
    gO2RawValid = false;
    const bool queryOk = o2Query(hundredths, gas, why);
    if (gO2RawValid)
      say("   O2 raw reply: %02X %02X %02X %02X %02X %02X %02X %02X %02X", gO2Raw[0], gO2Raw[1], gO2Raw[2], gO2Raw[3], gO2Raw[4], gO2Raw[5], gO2Raw[6],
          gO2Raw[7], gO2Raw[8]);
    if (!queryOk) {
      say("   O2 sensor answers its address but the query failed: %s", why);
      bistConclude(now, V_FAIL, why);
      return;
    }
    say("   O2 sensor reads %u.%02u %%  (gas type 0x%02X; 0x05 = O2)", hundredths / 100u, hundredths % 100u, gas);
    char note[28];
    if (gas != 0x05) bistConclude(now, V_FAIL, "gas type is not O2");
    else if (hundredths == 0) bistConclude(now, V_FAIL, "reads exactly 0.00 % (a fault)");
    else if (hundredths > 10000) bistConclude(now, V_FAIL, "reading above 100 %");
    else {
      snprintf(note, sizeof note, "%u.%02u %%", hundredths / 100u, hundredths % 100u);
      bistConclude(now, V_PASS, note);
    }
    return;
  } else if (gBistStep == B_TBS || gBistStep == B_TOB) {
    const bool on = (gBistStep == B_TBS) ? gTbsOn : gTobOn;
    if (on) gSawOn = true;
    else gSawOff = true;
    if (gSawOn && gSawOff) {
      bistConclude(now, V_PASS, "both states seen");
      return;
    }
    if (reached(now, gPhaseAt)) {
      bistConclude(now, V_UNSURE, gSawOn ? "only ON seen: not exercised" : (gSawOff ? "only OFF seen: not exercised" : "not seen"));
      return;
    }
  } else {  // B_LED
    if (!gAsking && reached(now, gPhaseAt)) {
      if (gPhase == 0) {
        say("   LED: counting 0000 .. 9999");
        gPhase = 1;
        gPhaseAt = now + 4600;
        gBlinkAt = now;
      } else if (gPhase == 1) {
        say("   LED: display off for 1 s, then on");
        gPhase = 2;
        gPhaseAt = now + 2500;
        gBlinkAt = now + 500;
      } else {
        gLedOn = true;
        gAsking = true;
        gAskUntil = now + 8000;
        say("   Did you see 8888 with one dot, the count 0-9 on all digits, and one blink?  (p = yes, f = no; no answer is fine)");
      }
    }
    if (!gAsking && gPhase == 1 && reached(now, gBlinkAt)) {
      gBlinkAt = now + 400;
      if (gDigit < 10) {
        const char c = static_cast<char>('0' + gDigit++);
        const char four[5] = {c, c, c, c, '\0'};
        ledText(four, -1);
      }
    }
    if (!gAsking && gPhase == 2 && reached(now, gBlinkAt)) {
      gBlinkAt = now + 1000;
      gBlink = !gBlink;
      gLedOn = gBlink;
    }
  }
  if (gAsking && gKey == 0 && reached(now, gAskUntil)) {  // nobody answered: the automatic part decides, the eye part stays open
    if (stepHadErrors()) bistConclude(now, V_FAIL, "I2C write errors during the step");
    else bistConclude(now, V_UNSURE, "");
    return;
  }
  if (gAsking && gKey != 0) {
    const char k = gKey;
    gKey = 0;
    if (k == 'p') {
      if (stepHadErrors()) bistConclude(now, V_FAIL, "I2C write errors during the step");
      else bistConclude(now, V_PASS, "");
    } else if (k == 'f') bistConclude(now, V_FAIL, gKeyNote);
    else if (k == 's') bistConclude(now, V_SKIP, "");
    else if (k == 'r') {
      bistEndStep();
      bistStartStep(now);
    }
  }
}

static void bistBegin(uint32_t now, uint8_t first = 0, uint8_t last = B_COUNT - 1) {
  if (!gPostDone) {
    say("The POST is still running; try again in a moment.");
    return;
  }
  gAutoAt = 0xFFFFFFFFu;
  for (uint8_t i = first; i <= last; ++i) {  // only the steps that are about to run are cleared: an earlier result for another device stays
    gVerdict[i] = V_NONE;
    gNote[i][0] = '\0';
  }
  gBist = true;
  gBistFinished = false;
  gBistStep = first;
  gBistLast = last;
  if (first == last) say("TEST: the %s only. Look at it; the Serial Monitor is optional (p = looks right, f = looks wrong).", kBistName[first]);
  else say("TEST: %d steps (LCD, RTC, LED, O2, TBS, TOB, RESET). Look at the displays; the Serial Monitor is optional (p = looks right, f = looks wrong).", B_COUNT);
  bistStartStep(now);
}

// ---------------------------------------------------------------------------------------------------------------------------
// What the LCD and the LED show
// ---------------------------------------------------------------------------------------------------------------------------
static DateTime gClock;
static bool gClockOk = false;
static uint32_t gClockNext = 0;

static void updateDisplays(uint32_t now) {
  if (reached(now, gClockNext)) {  // read the RTC twice a second, not on every pass
    gClockNext = now + 500;
    gClockOk = rtcRead(gClock);
  }
  bool anyFail = false;
  for (uint8_t i = 0; i < DEV_COUNT; ++i) anyFail = anyFail || gPost[i].level == LV_FAIL;
  // LED captions and numbers are set below; the LCD step owns the LCD rows.
  char row[24];
  const bool lcdOwned = gBist && gBistStep == B_LCD;
  if (!lcdOwned) {
  snprintf(row, sizeof row, "CHECK v%s TBS%d TOB%d", SKETCH_VERSION, gTbsOn ? 1 : 0, gTobOn ? 1 : 0);
  lcdSetRow(0, row);
  snprintf(row, sizeof row, "LCD%c RTC%c LED%c O2%c", kLevelChar[gPost[DEV_LCD].level], kLevelChar[gPost[DEV_RTC].level],
           kLevelChar[gPost[DEV_LED].level], kLevelChar[gPost[DEV_O2].level]);
  lcdSetRow(1, row);
  if (gBist && gBistStep == B_RST) {
    snprintf(row, sizeof row, "(the program restarts)");
  } else if (gBist && (gBistStep == B_TBS || gBistStep == B_TOB)) {
    const bool on = (gBistStep == B_TBS) ? gTbsOn : gTobOn;
    snprintf(row, sizeof row, "now:%s seen:%s%s", on ? "ON" : "off", gSawOn ? "ON " : "", gSawOff ? "off" : "");
  } else if (gClockOk) {
    formatDateTime(row, gClock);
  } else {
    snprintf(row, sizeof row, "RTC: no time");
  }
  lcdSetRow(2, row);
  if (gBist && gBistStep == B_RST) {
    snprintf(row, sizeof row, "PRESS RESET NOW");
  } else if (gBist && gBistStep == B_TBS) {
    snprintf(row, sizeof row, "FLIP TBS ON then off");
  } else if (gBist && gBistStep == B_TOB) {
    snprintf(row, sizeof row, "PRESS TOB, RELEASE");
  } else if (gBist && gBistStep == B_O2) {
    snprintf(row, sizeof row, "O2 sensor: query");
  } else if (gBist && gBistStep == B_LED) {
    static const char* caption[3] = {"LED: all segments", "LED: count 0 to 9", "LED: blink off/on"};
    snprintf(row, sizeof row, "%s", gAsking ? "LED: p/f or wait" : caption[gPhase < 3 ? gPhase : 2]);
  } else if (gBist) {
    snprintf(row, sizeof row, "TEST %u/%u: %s", gBistStep + 1, B_COUNT, kBistName[gBistStep]);
  } else if (gResetTestMsgUntil != 0 && !reached(now, gResetTestMsgUntil)) {
    snprintf(row, sizeof row, "%s", gResetTestMsg);
  } else if (!gPostDone) {
    snprintf(row, sizeof row, "POST running...");
  } else if (gBistFinished) {
    const uint8_t screen = (now / 3000u) % 3u;
    if (screen == 0) snprintf(row, sizeof row, "TEST LCD%c RTC%c LED%c", verdictChar(gVerdict[B_LCD]), verdictChar(gVerdict[B_RTC]), verdictChar(gVerdict[B_LED]));
    else if (screen == 1) snprintf(row, sizeof row, "TEST O2%c TBS%c TOB%c", verdictChar(gVerdict[B_O2]), verdictChar(gVerdict[B_TBS]), verdictChar(gVerdict[B_TOB]));
    else snprintf(row, sizeof row, "TEST RST%c", verdictChar(gVerdict[B_RST]));
  } else {
    snprintf(row, sizeof row, "test starts soon");
  }
  lcdSetRow(3, row);
  }  // !lcdOwned

  // The LED: the time (dot blinking each second) or the seconds since power-up; FFFF alternates with it if a device FAILED. A BIST step owns it
  // while it runs: the LED step draws its own patterns, the other steps show the step number.
  if (gBist && gBistStep == B_LED) return;
  char four[5];
  if (gBist) {
    snprintf(four, sizeof four, "-00%u", gBistStep + 1);
    ledText(four, -1);
  } else if (anyFail && (now / 1000u) % 2u == 1) {
    ledText("FFFF", -1);
  } else if (gPost[DEV_RTC].level != LV_NONE && gPost[DEV_RTC].level != LV_FAIL && gClockOk) {
    snprintf(four, sizeof four, "%02u%02u", gClock.hour % 100u, gClock.minute % 100u);
    ledText(four, (now / 500u) % 2u == 0 ? 1 : -1);
  } else {
    const unsigned long up = now / 1000u;
    snprintf(four, sizeof four, "%4lu", up > 9999 ? 9999ul : up);
    ledText(four, -1);
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
// Serial Monitor console
// ---------------------------------------------------------------------------------------------------------------------------
static bool gHeardHost = false;
static char gLine[64];
static uint8_t gLineLen = 0;

static void printBanner() {
  say("==== tom_i2c_check version %s | built %s %s | board %s ====", SKETCH_VERSION, __DATE__, __TIME__,
#if defined(ARDUINO_UNOR4_MINIMA)
      "UNO R4 Minima"
#elif defined(ARDUINO_UNOR4_WIFI)
      "UNO R4 WiFi"
#else
      "unknown"
#endif
  );
  say("Type help for the commands.");
  say("Last reset: %s   (%s)", gResetText, gResetRaw);
}

static void printHelp() {
  say("help      this list");
  say("status    the POST results, the RTC time, and the I2C error counts");
  say("scan      list every device that answers on the I2C bus");
  say("time      show the RTC date and time");
  say("post      run the quick power-up check again");
  say("run       the whole test now: LCD, then RTC, then LED (it also starts by itself after power-up and repeats every minute)");
  say("lcd       the LCD test only");
  say("rtc       the RTC test only");
  say("led       the LED test only");
  say("o2        the O2 sensor test only (a read-only query: it changes nothing in the sensor)");
  say("tbs       the TBS switch test only (flip it ON and OFF)");
  say("reset     the RESET button test only (press RESET within 20 s; the program restarts and reports)");
  say("tob       the TOB button test only (press and release it)");
  say("stop      stop the test and the automatic repeat");
  say("note ...  write a remark of yours into this log, for example: note LED digit 3 was dim");
  say("During a step that asks, type p (looks right) or f (looks wrong); no answer is fine.");
}

static void printStatus() {
  say("---- status (uptime %lu s) ----", static_cast<unsigned long>(millis() / 1000u));
  for (uint8_t i = 0; i < DEV_COUNT; ++i) say("  %-3s %-4s %s", kDevName[i], kLevelWord[gPost[i].level], gPost[i].text);
  DateTime t;
  char b[20];
  if (rtcRead(t)) {
    formatDateTime(b, t);
    say("  RTC time now: %s", b);
  } else {
    say("  RTC time now: cannot read");
  }
  say("  last reset: %s (%s)", gResetText, gResetRaw);
  say("  switches now: TBS %s, TOB %s", gTbsOn ? "ON" : "off", gTobOn ? "pressed" : "released");
  say("  I2C: LCD errors %lu, LED errors %lu, bus recoveries %lu", static_cast<unsigned long>(gLcdErrors), static_cast<unsigned long>(gLedErrors),
      static_cast<unsigned long>(gBusRecoveries));
  if (gBistFinished) {
    for (uint8_t i = 0; i < B_COUNT; ++i)
      say("  BIST %-3s %s %s", kBistName[i], gVerdict[i] == V_PASS ? "PASS" : (gVerdict[i] == V_FAIL ? "FAIL" : (gVerdict[i] == V_UNSURE ? "OK? (judge by eye)" : (gVerdict[i] == V_SKIP ? "skipped" : "not run"))), gNote[i]);
  }
}

static void printScan() {
  uint8_t count = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    if (!i2cProbe(a)) continue;
    const char* who = "unknown";
    if (a == ADDR_LCD) who = "LCD backpack";
    else if (a == ADDR_RTC) who = "DS3231 RTC";
    else if (a == 0x57) who = "RTC module EEPROM";
    else if (a == ADDR_O2) who = "O2 sensor";
    else if ((a >= ADDR_LED_CTL && a <= ADDR_LED_CTL + 3) || (a >= ADDR_LED_DIG && a <= ADDR_LED_DIG + 3)) who = "LED module (TM1650)";
    say("  0x%02X  %s", a, who);
    ++count;
  }
  say("I2C scan 0x08-0x77: %u device(s)", count);
  say("(The LED module answers all of 0x24-0x27 and 0x34-0x37: that is normal. The LCD must be at 0x23, outside that range.)");
}

static void handleLine(const char* line, uint32_t now) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return;
  gHeardHost = true;
  if (gBist && gAsking) {  // the operator's answer to a BIST step
    const char k = static_cast<char>(tolower(*line));
    if ((k == 'p' || k == 'f' || k == 'r' || k == 's' || k == 'q') && (line[1] == '\0' || line[1] == ' ')) {
      gKey = k;
      gKeyNote[0] = '\0';
      if (k == 'f') {
        const char* note = line + 1;
        while (*note == ' ') ++note;
        snprintf(gKeyNote, sizeof gKeyNote, "%s", note);
      }
      return;
    }
  }
  if (gBist && !gAsking && (line[1] == '\0' || line[1] == ' ') && strchr("pfrs", tolower(*line)) != nullptr) {
    say("(No step is asking right now. p / f are taken while an LCD or LED step waits for your answer; no answer is fine.)");
    return;
  }
  if (gBist && (tolower(*line) == 'q') && line[1] == '\0') {
    gKey = 'q';
    return;
  }
  char cmd[16];
  uint8_t i = 0;
  while (line[i] != '\0' && line[i] != ' ' && i < sizeof cmd - 1) {
    cmd[i] = static_cast<char>(tolower(line[i]));
    ++i;
  }
  cmd[i] = '\0';
  if (strcmp(cmd, "help") == 0) printHelp();
  else if (strcmp(cmd, "status") == 0) printStatus();
  else if (strcmp(cmd, "scan") == 0) printScan();
  else if (strcmp(cmd, "time") == 0) {
    DateTime t;
    char b[20];
    if (rtcRead(t)) {
      formatDateTime(b, t);
      say("RTC %s", b);
    } else {
      say("RTC: cannot read the time");
    }
  } else if (strcmp(cmd, "post") == 0) {
    if (gBist) say("A BIST is running: answer or quit it (q) first.");
    else postBegin(now);
  } else if (strcmp(cmd, "run") == 0 || strcmp(cmd, "bist") == 0) {
    gAutoRun = true;
    if (gBist) say("The test is already running.");
    else bistBegin(now);  // the whole test
  } else if (strcmp(cmd, "lcd") == 0 || strcmp(cmd, "rtc") == 0 || strcmp(cmd, "led") == 0 || strcmp(cmd, "o2") == 0 || strcmp(cmd, "tbs") == 0 || strcmp(cmd, "tob") == 0 || strcmp(cmd, "reset") == 0) {
    if (gBist) {
      say("A test is already running: wait for it, or type stop.");
    } else {
      gAutoRun = false;  // a test you asked for is not interrupted by the automatic repeat; type run to start the repeating test again
      const uint8_t step = strcmp(cmd, "lcd") == 0 ? B_LCD : (strcmp(cmd, "rtc") == 0 ? B_RTC : (strcmp(cmd, "led") == 0 ? B_LED : (strcmp(cmd, "o2") == 0 ? B_O2 : (strcmp(cmd, "tbs") == 0 ? B_TBS : (strcmp(cmd, "tob") == 0 ? B_TOB : B_RST)))));
      bistBegin(now, step, step);
    }
  } else if (strcmp(cmd, "stop") == 0) {
    gAutoRun = false;
    if (gBist) gKey = 'q';
    say("Stopped. Type run to start the test again.");
  } else if (strcmp(cmd, "note") == 0) {
    say("NOTE: %s", line + 4);
  } else say("unknown command '%s'. Type help.", cmd);
}

static void consoleService(uint32_t now) {
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      gLine[gLineLen] = '\0';
      gLineLen = 0;
      handleLine(gLine, now);
    } else if (gLineLen < sizeof gLine - 1) {
      gLine[gLineLen++] = static_cast<char>(c);
    }
  }
#if defined(ARDUINO_UNOR4_MINIMA)
  // The Minima's USB serial knows when a Serial Monitor connects: print the banner then (and again if it is closed and opened again).
  static bool wasConnected = false;
  const bool connected = static_cast<bool>(Serial);
  if (connected && !wasConnected) printBanner();
  wasConnected = connected;
#else
  // The WiFi board cannot tell whether a Serial Monitor is open: repeat the banner every 5 s, at most 10 times, until the PC has typed something.
  static uint32_t nextBanner = 5000;
  static uint8_t bannersLeft = 10;
  if (!gHeardHost && bannersLeft > 0 && reached(now, nextBanner)) {
    nextBanner = now + 5000;
    --bannersLeft;
    printBanner();
  }
#endif
}

// ---------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {  // the Minima's USB serial appears when the Serial Monitor opens; do not wait forever
  }
  readResetCause();  // once, at boot: the status registers persist until cleared
  pinMode(PIN_TBS, INPUT_PULLUP);  // inputs only, like the generator program; no output pin is ever driven
  pinMode(PIN_TOB, INPUT_PULLUP);
  Wire.begin();
  Wire.setClock(100000);
  i2cRecover();  // a reset in the middle of a transaction must not leave the bus stuck
  for (uint8_t r = 0; r < 4; ++r) lcdSetRow(r, "");
  memset(gLedSeg, 0, sizeof gLedSeg);
  ledText("----", -1);
  printBanner();
  resetTestEvaluate(millis());
  postBegin(millis());
}

void loop() {
  const uint32_t now = millis();
  consoleService(now);
  switchService(now);
  postService(now);
  if (gPostDone && gAutoRun && !gBist && reached(now, gAutoAt)) bistBegin(now);  // no keyboard needed: the test starts and repeats by itself
  bistService(now);
  updateDisplays(now);
  ledService(now);
  lcdService(now);
}
