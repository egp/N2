// reset_probe.ino — EXPERIMENT, not production code.
//
// Answers three questions on real hardware (Requirements O2-6a/6b, CON-3, RST-2):
//   1. Does RSTSR0.PORF tell a power-on reset from the reset button?
//   2. Does a .noinit RAM record survive the reset button, a watchdog reset and a
//      software reset, but NOT a power loss?
//   3. Does opening/closing the Serial Monitor reset the board?
//
// It touches NO output pins and starts no devices. Safe on a bare board.
// Open the Serial Monitor at any speed; the report prints whenever a console attaches.
//
// Keys (type the letter, press Enter):
//   r  reprint the boot report      s  software reset      w  watchdog reset
//   x  corrupt the RAM record (next boot must show "record invalid")
#include <Arduino.h>
#include <WDT.h>

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
static uint8_t rst0, rst1lo, rst2, rst0After, rst1After;
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
  Serial.println(F("==== RESET PROBE REPORT ===="));
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
}

void loop() {
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
