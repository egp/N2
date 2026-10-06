// reset_probe.ino — EXPERIMENT, not production code.
//
// Answers three questions on real hardware (Requirements O2-6a/6b, CON-3, RST-2):
//   1. Does RSTSR0.PORF tell a power-on reset from the reset button?
//   2. Does a .noinit RAM record survive the reset button, a watchdog reset and a
//      software reset, but NOT a power loss?
//   3. Does opening/closing the Serial Monitor reset the board?
//
// It starts no devices and drives NO pin except the board's own LED (LED_BUILTIN), which blinks the result so the
// experiment still works even if the USB serial link misbehaves:
//   cause blinks  : 1 = power-on, 2 = reset button / external pin, 3 = watchdog, 4 = software reset, 5 = other
//   then a pause, then ONE long blink = warm-up credit > 0 (valid RAM record AND not a power-on reset),
//                    or ONE very short blink = no credit.   Then the pattern repeats forever.
// On the UNO R4 WiFi the 12x8 LED matrix shows the same thing: the cause digit on the left, Y (credit) or N (no credit)
// on the right, and a blinking pixel in the bottom-right corner that proves loop() is running.
// Safe on a bare board.
// Open the Serial Monitor at any speed; the report prints whenever a console attaches.
//
// Keys (type the letter, press Enter):
//   r  reprint the boot report      s  software reset      w  watchdog reset
//   x  corrupt the RAM record (next boot must show "record invalid")
#include <Arduino.h>
#include <WDT.h>
#if defined(ARDUINO_UNOR4_WIFI)
#include <Arduino_LED_Matrix.h>  // the WiFi board's built-in 12x8 LED matrix (the Minima does not have one)
ArduinoLEDMatrix matrix;
#endif

struct Record {
  uint32_t magic;
  uint32_t boots;
  uint32_t lastSeenMs;
  uint32_t runMs;  // accumulated run time across resets
  uint32_t check;
};
static Record rec __attribute__((section(".noinit")));
static constexpr uint32_t kMagic = 0x4E325052;  // "N2PR"

static uint32_t checksum(const Record& r) {
  return r.magic ^ (r.boots * 0x9E3779B1u) ^ (r.lastSeenMs * 0x85EBCA6Bu) ^ (r.runMs * 0xC2B2AE35u) ^ 0xA5A5A5A5u;
}
static void seal() { rec.check = checksum(rec); }

// ---- captured at boot ----
static uint8_t rst0, rst2, rst0After, rst1After;
static uint16_t rst1;
static bool recordWasValid;
static uint32_t baseRunMs, setupStartMs, creditMs;
static const char* cause;
static bool wasAttached = false;

static const char* classify(uint8_t r0, uint16_t r1) {
  if (r0 & 0x01) return "POWER-ON (PORF)";
  if (r1 & 0x02) return "WATCHDOG (WDTRF)";
  if (r1 & 0x01) return "INDEPENDENT WATCHDOG (IWDTRF)";
  if (r1 & 0x04) return "SOFTWARE (SWRF)";
  if (r0 & 0x0E) return "VOLTAGE MONITOR (LVDxRF)";
  return "EXTERNAL / none flagged (reset pin = reset button?)";
}

static void clearFlags() {
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
  if (R_SYSTEM->RSTSR0 != 0 || R_SYSTEM->RSTSR1 != 0) {  // maybe register-protected: unlock, retry, relock
    R_SYSTEM->PRCR = 0xA50B;
    R_SYSTEM->RSTSR0 = 0;
    R_SYSTEM->RSTSR1 = 0;
    R_SYSTEM->PRCR = 0xA500;
  }
}

static void printReport() {
  Serial.println();
  Serial.println(F("==== RESET PROBE REPORT (reset_probe v1, UNO R4 WiFi) ===="));
  Serial.print(F("built ")); Serial.print(__DATE__); Serial.print(' '); Serial.println(__TIME__);
  Serial.print(F("boot #")); Serial.print(rec.boots);
  Serial.print(F("   RAM record valid at boot: ")); Serial.println(recordWasValid ? F("YES") : F("NO"));
  Serial.print(F("RSTSR0=0x")); Serial.print(rst0, HEX);
  Serial.print(F(" (PORF=")); Serial.print(rst0 & 1); Serial.print(F(" LVD0RF=")); Serial.print((rst0 >> 1) & 1);
  Serial.print(F(")  RSTSR1=0x")); Serial.print(rst1, HEX);
  Serial.print(F(" (IWDTRF=")); Serial.print(rst1 & 1); Serial.print(F(" WDTRF=")); Serial.print((rst1 >> 1) & 1);
  Serial.print(F(" SWRF=")); Serial.print((rst1 >> 2) & 1);
  Serial.print(F(")  RSTSR2=0x")); Serial.println(rst2, HEX);
  Serial.print(F("flags after clearing: RSTSR0=0x")); Serial.print(rst0After, HEX);
  Serial.print(F(" RSTSR1=0x")); Serial.println(rst1After, HEX);
  Serial.print(F("cause: ")); Serial.println(cause);
  Serial.print(F("warm-up credit rule (valid record AND not power-on): credit = "));
  Serial.print(creditMs / 1000); Serial.println(F(" s"));
  Serial.print(F("millis() at start of setup(): ")); Serial.println(setupStartMs);
  Serial.print(F("uptime now: ")); Serial.print(millis() / 1000); Serial.println(F(" s"));
  Serial.println(F("keys: r reprint, s software reset, w watchdog reset, x corrupt record"));
  Serial.println(F("============================"));
}


// ---- LED beacon (no serial needed): non-blocking blink pattern, see the header ----
static uint8_t causeBlinks() {
  if (rst0 & 0x01) return 1;                 // power-on
  if (rst1 & 0x03) return 3;                 // watchdog (WDT or IWDT)
  if (rst1 & 0x04) return 4;                 // software
  if (rst0 & 0x0E) return 5;                 // voltage monitor
  return 2;                                  // nothing flagged: reset pin (the reset button)
}

static void beacon(uint32_t now) {
  // One cycle: N cause blinks (150 ms on / 250 ms off), 1.2 s pause, the credit blink, 2.5 s pause.
  static uint32_t cycleStart = 0;
  static bool started = false;
  if (!started) { started = true; cycleStart = now; }
  const uint32_t n = causeBlinks();
  const uint32_t blinksEnd = n * 400u;
  const uint32_t creditStart = blinksEnd + 1200u;
  const uint32_t creditLen = creditMs > 0 ? 1000u : 60u;
  const uint32_t cycleLen = creditStart + creditLen + 2500u;
  uint32_t t = now - cycleStart;
  if (t >= cycleLen) { cycleStart += cycleLen; t -= cycleLen; }
  bool on = false;
  if (t < blinksEnd) on = (t % 400u) < 150u;
  else if (t >= creditStart && t < creditStart + creditLen) on = true;
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

#if defined(ARDUINO_UNOR4_WIFI)
// ---- 12x8 LED matrix (WiFi board only): glyphs are 5 columns x 7 rows, one 5-bit value per row ----
static const uint8_t kGlyph1[7] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
static const uint8_t kGlyph2[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
static const uint8_t kGlyph3[7] = {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E};
static const uint8_t kGlyph4[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
static const uint8_t kGlyph5[7] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E};
static const uint8_t kGlyphY[7] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};
static const uint8_t kGlyphN[7] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};

// The matrix takes 96 bits (12 columns x 8 rows, row by row, most significant bit first) in three 32-bit words.
static void setPixel(uint32_t* frame, uint8_t row, uint8_t col) {
  const uint8_t index = static_cast<uint8_t>(row * 12 + col);
  frame[index / 32] |= 1UL << (31 - (index % 32));
}

static void drawGlyph(uint32_t* frame, const uint8_t* glyph, uint8_t leftColumn) {
  for (uint8_t row = 0; row < 7; ++row)
    for (uint8_t bit = 0; bit < 5; ++bit)
      if (glyph[row] & (0x10 >> bit)) setPixel(frame, row, static_cast<uint8_t>(leftColumn + bit));
}

static void showMatrix(uint32_t now) {
  static const uint8_t* const kDigits[5] = {kGlyph1, kGlyph2, kGlyph3, kGlyph4, kGlyph5};
  static bool heartbeat = false;
  static uint32_t nextUpdate = 0;
  if (static_cast<int32_t>(now - nextUpdate) < 0) return;
  nextUpdate = now + 500;
  heartbeat = !heartbeat;
  uint32_t frame[3] = {0, 0, 0};
  drawGlyph(frame, kDigits[causeBlinks() - 1], 0);                 // cause 1..5
  drawGlyph(frame, creditMs > 0 ? kGlyphY : kGlyphN, 7);           // warm-up credit?
  if (heartbeat) setPixel(frame, 7, 11);                           // loop() is alive
  matrix.loadFrame(frame);
}
#endif

void setup() {
  // Read the reset flags before anything else can disturb them.
  rst0 = R_SYSTEM->RSTSR0;
  rst1 = R_SYSTEM->RSTSR1;
  rst2 = R_SYSTEM->RSTSR2;
  setupStartMs = millis();
  cause = classify(rst0, rst1);
  clearFlags();
  rst0After = R_SYSTEM->RSTSR0;
  rst1After = static_cast<uint8_t>(R_SYSTEM->RSTSR1 & 0xFF);

  recordWasValid = (rec.magic == kMagic) && (rec.check == checksum(rec));
  const bool powerOn = rst0 & 0x01;
  if (recordWasValid && !powerOn) {
    baseRunMs = rec.runMs;  // runMs already includes the last session's uptime up to lastSeen
    creditMs = baseRunMs;
    rec.boots++;
  } else {
    baseRunMs = 0;
    creditMs = 0;
    rec.magic = kMagic;
    rec.boots = 1;
  }
  rec.lastSeenMs = millis();
  rec.runMs = baseRunMs + millis();
  seal();
  pinMode(LED_BUILTIN, OUTPUT);  // only after the flags are safely read
#if defined(ARDUINO_UNOR4_WIFI)
  matrix.begin();
#endif
}

void loop() {
  beacon(millis());
#if defined(ARDUINO_UNOR4_WIFI)
  showMatrix(millis());
#endif
  rec.lastSeenMs = millis();
  rec.runMs = baseRunMs + millis();
  seal();

  const bool attached = Serial;  // true only while a host has the port open (USB DTR)
  if (attached && !wasAttached) {
    Serial.print(F("[console attached at ")); Serial.print(millis()); Serial.println(F(" ms]"));
    printReport();
  }
  wasAttached = attached;

  static uint32_t nextHb = 2000;
  if (attached && millis() >= nextHb) {
    nextHb = millis() + 2000;
    Serial.print(F("HB boot=")); Serial.print(rec.boots);
    Serial.print(F(" up=")); Serial.print(millis() / 1000);
    Serial.print(F("s credit_total=")); Serial.print(rec.runMs / 1000); Serial.println(F("s"));
  }

  if (attached && Serial.available()) {
    const int c = Serial.read();
    if (c == 'r') printReport();
    if (c == 's') { Serial.println(F("software reset now")); Serial.flush(); delay(50); NVIC_SystemReset(); }
    if (c == 'w') { Serial.println(F("watchdog reset in ~0.5 s")); Serial.flush(); WDT.begin(500); for (;;) {} }
    if (c == 'x') { rec.check ^= 0xFFFFFFFFu; Serial.println(F("record corrupted; it will not be re-sealed until next reset")); for (;;) { delay(1000); } }
  }
}
