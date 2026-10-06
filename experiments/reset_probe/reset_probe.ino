// ===========================================================================================
// reset_probe  VERSION 1.10  (2026-10-06)      <-- if you do not see this line, the IDE has an older copy
//
// Change log
//   1.10 RESULT of 1.9: after a software reset the three plain RAM spots (heap-mid, heap-top, stack-bottom) SURVIVED but the
//        .noinit record did not. So the chip keeps RAM and something clears or overwrites .noinit at startup.
//        This version prints the .noinit record's raw words as found at boot (all zero = cleared; other values = overwritten).
//   1.9  RESULT of 1.8: serial works. A software reset cleared the flags fine (plain writes work, no unlock needed) but the
//        .noinit RAM record did NOT survive (boot count stayed 1, record invalid), although the linker did put it in .noinit
//        (0x20000248). So something overwrites RAM across a reset (the bootloader?). This version also writes signatures to
//        THREE MORE RAM locations and reports which survive each kind of reset:
//          heap-mid 0x20004000, heap-top 0x20007A00, stack-bottom 0x20007B00   (the heap is unused by this sketch)
//   1.8  FOUND WHY THERE WAS NO SERIAL OUTPUT: on the UNO R4 WiFi the core is built with -DNO_USB, so `Serial` is a hardware
//        UART (talking to the ESP32 chip that provides the USB connection) and the core does NOT call Serial.begin() for us
//        (the Minima, which has native USB, does). Until begin() is called, every Serial.print() returns 0. Fix: setup() now
//        calls Serial.begin(115200). Also on the WiFi board `if (Serial)` is ALWAYS true (it cannot see the PC), so the report
//        is repeated every 10 s there, and typing `r` still reprints it at any time. Matrix pixel 1 is therefore always lit
//        on the WiFi board and means nothing there; pixel 5 (bytes accepted) now should light.
//   1.7  matrix cycles through pages, all numbers in HEX (two digits = one byte per page):
//          page 1 (1.5 s)  version        major, minor   e.g. "1 7"  (minor 10..15 shows as A..F)
//          page 2 (1.5 s)  RSTSR0 at boot  bit0 = power-on, bit1 = voltage-monitor 0 (the other bits are other monitors)
//          page 3 (1.5 s)  RSTSR1 at boot  bit0 = IWDT reset, bit1 = WDT reset, bit2 = software reset
//          page 4 (4 s)    the summary: cause digit (1-5), Y/N credit, and the status pixels on the bottom row
//        On pages 1-3 the page number is shown by the lit pixels at the bottom-left (1, 2 or 3 pixels).
//   1.6  result so far (v1.4/1.5): reset button changed 4 (software) to 2 (reset pin) -> the reset-cause flags work and clear.
//        But credit stayed N: the RAM record did not survive. New matrix pixels (bottom row, counting from the left, 1st = 1):
//        8th lit = the record's signature (magic) was still in RAM at boot; 10th lit = its checksum matched too.
//        For 2 s after every boot the matrix shows the version digits ("1" then "6" = version 1.6).
//   1.5  suspect: unlocking/re-locking the register-protect register (PRCR) may have broken USB serial.
//        PRCR is no longer touched; flags are cleared with plain writes only (the report says if that worked).
//        New matrix pixel (bottom row, 5th from left): lit = the core accepted bytes for sending
//   1.4  version header and change log (this); the same version is printed in the serial report
//   1.3  matrix bottom row shows the serial link: lit pixel 0 = console seen (DTR), pixel 2 = a byte received
//   1.2  WiFi board: 12x8 LED matrix shows the cause digit and Y/N for warm-up credit, plus a heartbeat pixel
//   1.1  built-in LED blinks the cause and the credit result (works without any serial output)
//   1.0  first version: serial report of the reset flags, RAM-record survival, console attach/detach
// ===========================================================================================
#define PROBE_VERSION "1.10"
#define PROBE_MAJOR 1
#define PROBE_MINOR 10

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
static uint32_t rawAtBoot[5];           // the .noinit record exactly as found at boot (before anything touched it)
static bool recMagicOk, recCheckOk;   // what the RAM record looked like at boot (shown on the matrix)
static uint32_t baseRunMs, setupStartMs, creditMs;
static const char* cause;
static bool wasAttached = false;
static bool serialAttached = false;   // what `if (Serial)` says right now (shown on the matrix)
static bool everReceived = false;
static bool coreAcceptedBytes = false;   // did Serial.print() ever report bytes taken for sending?     // has any byte ever arrived from the host?

static const char* classify(uint8_t r0, uint16_t r1) {
  if (r0 & 0x01) return "POWER-ON (PORF)";
  if (r1 & 0x02) return "WATCHDOG (WDTRF)";
  if (r1 & 0x01) return "INDEPENDENT WATCHDOG (IWDTRF)";
  if (r1 & 0x04) return "SOFTWARE (SWRF)";
  if (r0 & 0x0E) return "VOLTAGE MONITOR (LVDxRF)";
  return "EXTERNAL / none flagged (reset pin = reset button?)";
}

static void clearFlags() {
  // Plain writes only. (v1.0-1.4 also unlocked and re-locked PRCR, which may have disturbed USB; see the change log.)
  R_SYSTEM->RSTSR0 = 0;
  R_SYSTEM->RSTSR1 = 0;
}


// ---- extra RAM locations to test, in case .noinit is overwritten by the bootloader (v1.9) ----
struct Spot {
  uint32_t address;
  const char* name;
  bool validAtBoot;
  uint32_t bootsAtBoot;
};
static Spot spots[] = {{0x20004000u, "heap-mid", false, 0}, {0x20007A00u, "heap-top", false, 0}, {0x20007B00u, "stack-bottom", false, 0}};
static const uint32_t kSpotMagic = 0x53504F54u;  // "SPOT"

static void checkAndSealSpots() {
  for (Spot& sp : spots) {
    volatile uint32_t* w = reinterpret_cast<volatile uint32_t*>(sp.address);   // w[0] magic, w[1] boots, w[2] check
    sp.validAtBoot = (w[0] == kSpotMagic) && (w[2] == (w[0] ^ w[1] ^ 0x5A5A5A5Au));
    sp.bootsAtBoot = sp.validAtBoot ? w[1] : 0;
    const uint32_t boots = sp.validAtBoot ? w[1] + 1 : 1;
    w[0] = kSpotMagic;
    w[1] = boots;
    w[2] = kSpotMagic ^ boots ^ 0x5A5A5A5Au;
  }
}

static void printReport() {
  Serial.println();
  Serial.print(F("==== RESET PROBE REPORT (reset_probe version " PROBE_VERSION ", UNO R4 WiFi) ===="));
  Serial.println();
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
  Serial.print(F(".noinit record at 0x")); Serial.print(reinterpret_cast<uint32_t>(&rec), HEX); Serial.print(F(" as found at boot: "));
  for (uint8_t i = 0; i < 5; ++i) { Serial.print(F("0x")); Serial.print(rawAtBoot[i], HEX); Serial.print(i < 4 ? F(" ") : F("\n")); }
  for (const Spot& sp : spots) {
    Serial.print(F("RAM spot ")); Serial.print(sp.name); Serial.print(F(" @0x")); Serial.print(sp.address, HEX);
    Serial.print(F(": survived the reset = ")); Serial.print(sp.validAtBoot ? F("YES") : F("NO"));
    Serial.print(F("  boots seen = ")); Serial.println(sp.bootsAtBoot + (sp.validAtBoot ? 1 : 0));
  }
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
static const uint8_t kDigit[16][7] = {   // hex digits 0-F
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},  // 0
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},  // 1
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},  // 2
    {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E},  // 3
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},  // 4
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},  // 5
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},  // 6
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},  // 7
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},  // 8
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},  // 9
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},  // A
    {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},  // B
    {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},  // C
    {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},  // D
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},  // E
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},  // F
};
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
  static bool heartbeat = false;
  static uint32_t nextUpdate = 0;
  if (static_cast<int32_t>(now - nextUpdate) < 0) return;
  nextUpdate = now + 500;
  heartbeat = !heartbeat;
  uint32_t frame[3] = {0, 0, 0};
  // Pages, each shown as two hex digits (one byte). Cycle: version 1.5 s, RSTSR0 1.5 s, RSTSR1 1.5 s, summary 4 s.
  const uint32_t t = now % 8500u;
  uint8_t page = 0;                                     // 0 = summary
  if (t < 1500u) page = 1;
  else if (t < 3000u) page = 2;
  else if (t < 4500u) page = 3;
  if (page != 0) {
    uint8_t value = 0;
    if (page == 1) value = static_cast<uint8_t>((PROBE_MAJOR << 4) | PROBE_MINOR);
    else if (page == 2) value = rst0;
    else value = static_cast<uint8_t>(rst1 & 0xFF);
    drawGlyph(frame, kDigit[value >> 4], 0);
    drawGlyph(frame, kDigit[value & 0x0F], 7);
    for (uint8_t i = 0; i < page; ++i) setPixel(frame, 7, i);   // page number: lit pixels on the bottom-left, 1 to 3
    if (heartbeat) setPixel(frame, 7, 11);
    matrix.loadFrame(frame);
    return;
  }
  drawGlyph(frame, kDigit[causeBlinks()], 0);                      // cause 1..5
  drawGlyph(frame, creditMs > 0 ? kGlyphY : kGlyphN, 7);           // warm-up credit?
  if (heartbeat) setPixel(frame, 7, 11);                           // loop() is alive
  if (serialAttached) setPixel(frame, 7, 0);                       // the board sees a console (DTR) right now
  if (everReceived) setPixel(frame, 7, 2);                         // a byte has arrived from the host
  if (coreAcceptedBytes) setPixel(frame, 7, 4);                    // the core took bytes for sending
  if (recMagicOk) setPixel(frame, 7, 7);                           // the RAM record's signature survived the reset
  if (recCheckOk) setPixel(frame, 7, 9);                           // ...and so did its checksum
  matrix.loadFrame(frame);
}
#endif

void setup() {
  { const uint32_t* r = reinterpret_cast<const uint32_t*>(&rec); for (uint8_t i = 0; i < 5; ++i) rawAtBoot[i] = r[i]; }
  // Read the reset flags before anything else can disturb them.
  rst0 = R_SYSTEM->RSTSR0;
  rst1 = R_SYSTEM->RSTSR1;
  rst2 = R_SYSTEM->RSTSR2;
  setupStartMs = millis();
  cause = classify(rst0, rst1);
  clearFlags();
  rst0After = R_SYSTEM->RSTSR0;
  rst1After = static_cast<uint8_t>(R_SYSTEM->RSTSR1 & 0xFF);

  checkAndSealSpots();            // test the extra RAM locations before anything else can touch them
  recMagicOk = (rec.magic == kMagic);
  recCheckOk = (rec.check == checksum(rec));
  recordWasValid = recMagicOk && recCheckOk;
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
  Serial.begin(115200);          // REQUIRED on the R4 WiFi (see the change log); harmless on the Minima
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
  serialAttached = attached;
  if (attached && !wasAttached) {
    if (Serial.print(F("[console attached at ")) > 0) coreAcceptedBytes = true;
    Serial.print(millis()); Serial.println(F(" ms]"));
    printReport();
  }
  wasAttached = attached;
#if defined(ARDUINO_UNOR4_WIFI)
  // `Serial` cannot tell whether a PC is listening, so repeat the report every 10 s: open the monitor and wait a moment.
  static uint32_t nextReport = 5000;
  if (millis() >= nextReport) {
    nextReport = millis() + 10000;
    printReport();
  }
#endif

  static uint32_t nextHb = 2000;
  if (attached && millis() >= nextHb) {
    nextHb = millis() + 2000;
    if (Serial.print(F("HB boot=")) > 0) coreAcceptedBytes = true;
    Serial.print(rec.boots);
    Serial.print(F(" up=")); Serial.print(millis() / 1000);
    Serial.print(F("s credit_total=")); Serial.print(rec.runMs / 1000); Serial.println(F("s"));
  }

  if (attached && Serial.available()) {
    const int c = Serial.read();
    everReceived = true;
    if (c == 'r') printReport();
    if (c == 's') { Serial.println(F("software reset now")); Serial.flush(); delay(50); NVIC_SystemReset(); }
    if (c == 'w') { Serial.println(F("watchdog reset in ~0.5 s")); Serial.flush(); WDT.begin(500); for (;;) {} }
    if (c == 'x') { rec.check ^= 0xFFFFFFFFu; Serial.println(F("record corrupted; it will not be re-sealed until next reset")); for (;;) { delay(1000); } }
  }
}
