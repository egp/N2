// ===========================================================================================
// nvm_probe  VERSION 1.1   (2026-10-08)     <-- if you do not see this line, the IDE has an older copy
//                                               (minor versions are written in HEX: 1.A = 1.10)
//
// Learns how the UNO R4's non-volatile memory (data flash, reached through the core's EEPROM library) behaves, and measures the
// bounce of the TBS (D0) and TOB (D1) switches. Uses the REAL firmware code: SettingsStore / NvmRecord / Debounce (N2V8/src, via
// the `src` link in this folder; recreate with  ln -s ../../N2V8/src src  if missing). Works on the R4 WiFi and the R4 Minima.
//
// SAFE: it writes only to the first two 1 KB blocks of data flash, only when YOU type  w  or  k . It never touches an output pin.
//
// Console 115200 (send with newline):
//   i            info: size, block size, what this program thinks the board is
//   d            hex dump of the first 32 bytes of EVERY 1 KB block (what is in there before we ever wrote: unknown contents)
//   l            load: what the two stored copies (A, B) say, and which one is in use
//   w TBS TOB    save debounce times in ms (2..100), e.g.  w 20 30
//   t            timing: how long a save takes (microseconds) and whether the loop is blocked meanwhile
//   e            erase both copies (back to "nothing stored")  -- asks you to type  e y
//   b            GUIDED BOUNCE TEST (b again cancels): first TBS, then TOB. The LCD shows the progress (cycle 3 of 20) and, live,
//                min / median / mean / max settle time in microseconds and edges per operation. One cycle = switch ON, then OFF
//                (so 2 operations). Each operation also prints a line on the console.
//   c N          number of cycles per switch (5..30, default 20)
//   r            result so far on the console (raw numbers; 2 x max shown unrounded, plus what rounding up to whole ms would give)
//   z            clear the bounce statistics
//   Which board am I?  Type  m 1  (Minima) or  m 2  (WiFi): stored with the times, so a value measured on one board is never
//   applied to the other.
// Bounce rules of the road: leave the console quiet while operating a switch (printing takes time, the program prints only after
// the switch has been still for 0.2 s), and operate each switch 20+ times, press AND release.
//
// Change log
//   1.1  guided bounce test with progress and min/median/mean/max on the LCD (no rounding: raw microseconds, 2 x max in 0.1 ms);
//        20 on/off cycles per switch (c N changes it); the k command is gone (use  w TBS TOB  once you have decided the values)
//   1.0  first version
// ===========================================================================================
#define PROBE_VERSION "1.1"

#include <Arduino.h>
#include <EEPROM.h>

#include "src/BoardPins.h"
#include "src/core/Debounce.h"
#include "src/core/SettingsStore.h"
#include "src/drivers/Lcd20x4.h"
#include "src/hal/HalArduino.h"
#include "src/ui/LcdScreens.h"
#include "src/hal/Nvm.h"

using namespace n2;

// The R4 data flash through the core's EEPROM library. Every EEPROM write call rewrites its whole 1 KB block, so write() collects the
// changed bytes and uses ONE put() per call; unchanged bytes are skipped by the library (update()).
class EepromNvm : public Nvm {
 public:
  size_t size() const override { return static_cast<size_t>(EEPROM.length()); }
  size_t blockSize() const override { return 1024; }
  void read(size_t addr, uint8_t* d, size_t n) override { for (size_t i = 0; i < n; i++) d[i] = EEPROM.read(static_cast<int>(addr + i)); }
  bool write(size_t addr, const uint8_t* d, size_t n) override {
    if (addr / 1024 != (addr + n - 1) / 1024) return false;
    for (size_t i = 0; i < n; i++) EEPROM.update(static_cast<int>(addr + i), d[i]);   // (each update of a changed byte = 1 block write)
    return true;
  }
};

static EepromNvm nvm;
static SettingsStore store(nvm);
static BoardId boardId = BoardId::kUnknown;

static HalArduino hal;
static Lcd20x4 lcd(hal, kLcdAddress);
static bool lcdStarted = false;
static uint8_t cyclesWanted = 20;
static uint8_t stage = 0;               // 0 idle, 1 TBS, 2 TOB, 3 done
static bool screenDirty = true;
static const uint8_t kTbsPin = pin::kD0, kTobPin = pin::kD1;     // BoardPins.h: both on D0/D1, active LOW (pull-up)
static BounceMeter tbsMeter, tobMeter;
static bool measuring = false;
static uint16_t tbsOps = 0, tobOps = 0;
static String line;
static uint32_t lastPassUs = 0, passMaxUs = 0, bannerMs = 0;
static bool heardHost = false;

static void printReport(const StoreReport& r) {
  Serial.print("copy A: "); Serial.print(recordStatusName(r.a)); if (r.a == RecordStatus::kOk) { Serial.print("  seq "); Serial.print(r.seqA); }
  Serial.print("\ncopy B: "); Serial.print(recordStatusName(r.b)); if (r.b == RecordStatus::kOk) { Serial.print("  seq "); Serial.print(r.seqB); }
  Serial.println();
  if (!r.valid) { Serial.println("no valid settings stored -> the compiled default (30 ms) applies"); return; }
  Serial.print("in use: copy "); Serial.print(r.which); Serial.print("  saves "); Serial.print(r.sequence);
  Serial.print("  TBS "); Serial.print(r.settings.tbsDebounceMs); Serial.print(" ms  TOB "); Serial.print(r.settings.tobDebounceMs);
  Serial.print(" ms  board "); Serial.println(static_cast<int>(r.settings.board));
}

static void printMeter(const char* name, const BounceMeter& m) {
  const BounceStats& s = m.stats();
  Serial.print(name); Serial.print(": "); Serial.print(s.operations); Serial.print(" operations");
  if (!s.operations) { Serial.println(); return; }
  Serial.print(", edges/operation mean "); Serial.print(static_cast<float>(s.edgesTotal) / s.operations, 1);
  Serial.print(" max "); Serial.print(s.edgesMax);
  Serial.print("; settle us: min "); Serial.print(s.settleMinUs); Serial.print(" median "); Serial.print(m.settleMedianUs());
  Serial.print(" mean "); Serial.print(m.settleMeanUs()); Serial.print(" max "); Serial.println(s.settleMaxUs);
  Serial.print("    2 x max = "); Serial.print(2 * s.settleMaxUs); Serial.print(" us; rounded up to whole ms (NOT applied) = "); Serial.print(m.recommendedMs()); Serial.println(" ms");
}

static void dump() {
  for (size_t b = 0; b < nvm.size() / 1024; b++) {
    uint8_t d[32]; nvm.read(b * 1024, d, 32);
    char buf[16];
    snprintf(buf, sizeof buf, "%04X: ", static_cast<unsigned>(b * 1024)); Serial.print(buf);
    for (int i = 0; i < 32; i++) { snprintf(buf, sizeof buf, "%02X%s", d[i], (i % 8 == 7) ? "  " : " "); Serial.print(buf); }
    Serial.println();
  }
}

static NvmSettings current(uint8_t tbs, uint8_t tob) { NvmSettings s; s.tbsDebounceMs = tbs; s.tobDebounceMs = tob; s.board = boardId; return s; }

static void command(String c) {
  c.trim();
  if (!c.length()) return;
  if (c == "i") {
    Serial.print("nvm size "); Serial.print(nvm.size()); Serial.print(" bytes, block "); Serial.print(nvm.blockSize());
    Serial.print(" bytes, board id "); Serial.println(static_cast<int>(boardId));
  } else if (c == "d") {
    dump();
  } else if (c == "l") {
    printReport(store.load());
  } else if (c.startsWith("m ")) {
    boardId = static_cast<BoardId>(c.substring(2).toInt()); Serial.print("board id "); Serial.println(static_cast<int>(boardId));
  } else if (c.startsWith("w ")) {
    int a = c.substring(2).toInt(); const int sp = c.indexOf(' ', 2); const int b = sp > 0 ? c.substring(sp + 1).toInt() : a;
    if (a < kMinDebounceMs || a > kMaxDebounceMs || b < kMinDebounceMs || b > kMaxDebounceMs) { Serial.println("times must be 2..100 ms"); return; }
    const uint32_t t0 = micros(); const bool ok = store.save(current(a, b)); const uint32_t dt = micros() - t0;
    Serial.print(ok ? "saved" : "SAVE FAILED"); Serial.print(store.lastSaveWrote() ? "" : " (identical: nothing written)");
    Serial.print("  took "); Serial.print(dt); Serial.println(" us"); printReport(store.load());
  } else if (c == "t") {
    // alternate two values so every save really writes
    const uint8_t v = 40 + (millis() & 7);
    uint32_t t0 = micros(); const bool ok = store.save(current(v, v + 1)); uint32_t dt = micros() - t0;
    Serial.print("save "); Serial.print(ok ? "ok" : "FAILED"); Serial.print(", wrote "); Serial.print(store.lastSaveWrote());
    Serial.print(", took "); Serial.print(dt); Serial.println(" us (the CPU waits this long; nothing else runs)");
    t0 = micros(); store.load(); dt = micros() - t0; Serial.print("load took "); Serial.print(dt); Serial.println(" us");
  } else if (c == "e") {
    Serial.println("type  e y  to erase both copies");
  } else if (c == "e y") {
    uint8_t ff[16]; for (auto& x : ff) x = 0xFF;
    for (size_t b = 0; b < 2; b++) { for (size_t i = 0; i < 16; i++) EEPROM.update(static_cast<int>(b * 1024 + i), 0xFF); }
    Serial.println("both copies erased"); printReport(store.load());
  } else if (c == "b") {
    if (stage == 1 || stage == 2) { stage = 0; Serial.println("bounce test cancelled"); }
    else { tbsMeter.reset(); tobMeter.reset(); tbsOps = tobOps = 0; passMaxUs = 0; stage = 1; Serial.print("BOUNCE TEST: operate TBS ON then OFF, "); Serial.print(cyclesWanted); Serial.println(" times (LCD shows progress)"); }
    screenDirty = true;
  } else if (c == "r") {
    printMeter("TBS", tbsMeter); printMeter("TOB", tobMeter);
    Serial.print("longest loop pass "); Serial.print(passMaxUs); Serial.println(" us (edges closer than this could be missed)");
  } else if (c == "z") {
    tbsMeter.reset(); tobMeter.reset(); tbsOps = tobOps = 0; passMaxUs = 0; stage = 0; screenDirty = true; Serial.println("cleared");
    tbsMeter.reset(); tobMeter.reset(); tbsOps = tobOps = 0; passMaxUs = 0; Serial.println("cleared");
  } else if (c.startsWith("c ")) {
    const int n = c.substring(2).toInt();
    if (n < 5 || n > 30) { Serial.println("cycles must be 5..30"); return; }
    cyclesWanted = static_cast<uint8_t>(n); Serial.print("cycles per switch: "); Serial.println(n);
  } else {
    Serial.println("commands: i d l m 1|2 w TBS TOB t e b c N r z");
  }
}

static void pad(char* out, const char* text) { size_t i = 0; for (; text[i] && i < 20; i++) out[i] = text[i]; for (; i < 20; i++) out[i] = ' '; out[20] = 0; }

static void showScreen() {
  char r[4][24], t[4][24];
  const char* name = stage == 2 ? "TOB" : "TBS";
  BounceMeter& m = stage == 2 ? tobMeter : tbsMeter;
  const BounceStats& s = m.stats();
  if (stage == 1 || stage == 2) {
    snprintf(t[0], 24, "%s cycle %u of %u", name, static_cast<unsigned>(s.operations / 2), static_cast<unsigned>(cyclesWanted));
    if (s.operations == 0) { snprintf(t[1], 24, "operate %s ON, OFF", name); t[2][0] = t[3][0] = 0; }
    else {
      snprintf(t[1], 24, "min%5lu med%5lu us", static_cast<unsigned long>(s.settleMinUs), static_cast<unsigned long>(m.settleMedianUs()));
      snprintf(t[2], 24, "mean%4lu max%5lu us", static_cast<unsigned long>(m.settleMeanUs()), static_cast<unsigned long>(s.settleMaxUs));
      snprintf(t[3], 24, "edges avg %u.%u max %u", static_cast<unsigned>(s.edgesTotal / s.operations), static_cast<unsigned>((s.edgesTotal * 10 / s.operations) % 10), static_cast<unsigned>(s.edgesMax));
    }
  } else if (stage == 3) {
    snprintf(t[0], 24, "DONE %u cycles each", static_cast<unsigned>(cyclesWanted));
    snprintf(t[1], 24, "max us %5lu %5lu", static_cast<unsigned long>(tbsMeter.stats().settleMaxUs), static_cast<unsigned long>(tobMeter.stats().settleMaxUs));
    const unsigned long a = (2 * tbsMeter.stats().settleMaxUs + 50) / 100, b = (2 * tobMeter.stats().settleMaxUs + 50) / 100;
    snprintf(t[2], 24, "2xmax ms %lu.%lu %lu.%lu", a / 10, a % 10, b / 10, b % 10);
    snprintf(t[3], 24, "TBS then TOB; r=detail");
  } else {
    snprintf(t[0], 24, "nvm_probe %s", PROBE_VERSION); snprintf(t[1], 24, "b = bounce test"); t[2][0] = t[3][0] = 0;
  }
  for (int i = 0; i < 4; i++) pad(r[i], t[i]);
  lcd.setScreen(makeScreen(r[0], r[1], r[2], r[3]));
}

void setup() {
  Serial.begin(115200);
  pinMode(kTbsPin, INPUT_PULLUP);
  pinMode(kTobPin, INPUT_PULLUP);
  hal.i2cBegin();
}

void loop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  if (!lcdStarted && nowMs >= 2500) { lcdStarted = true; lcd.begin(nowMs); }   // the LCD needs 2.5 s after power-up
  const bool active = stage == 1 || stage == 2;
  BounceMeter& m = stage == 2 ? tobMeter : tbsMeter;
  uint16_t& ops = stage == 2 ? tobOps : tbsOps;
  if (active) {                                      // the fast part: read the pin, timestamp, nothing else
    const uint8_t pinNo = stage == 2 ? kTobPin : kTbsPin;
    const uint32_t gap = nowUs - lastPassUs;
    if (lastPassUs && gap > passMaxUs) passMaxUs = gap;
    m.sample(digitalRead(pinNo) == LOW, nowUs);
    m.flush(nowUs);
    if (m.stats().operations != ops) {
      ops = m.stats().operations; screenDirty = true;
      Serial.print(stage == 2 ? "TOB #" : "TBS #"); Serial.print(ops); Serial.print(" edges "); Serial.print(m.stats().lastEdges);
      Serial.print(" settle us "); Serial.println(m.stats().lastSettleUs);
      if (ops / 2 >= cyclesWanted) {
        Serial.println(stage == 1 ? "TBS done; now TOB" : "TOB done"); 
        if (stage == 1) stage = 2; else { stage = 3; printMeter("TBS", tbsMeter); printMeter("TOB", tobMeter); }
        screenDirty = true; lastPassUs = 0;
      }
    }
  }
  lastPassUs = nowUs;
  // The LCD is touched only while no operation is open (a row write takes ~10 ms, longer than typical bounce).
  if (lcdStarted && !(active && m.inOperation())) {
    if (screenDirty) { showScreen(); screenDirty = false; }
    lcd.service(nowMs);
  }
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read()); heardHost = true;
    if (ch == '\n' || ch == '\r') { command(line); line = ""; } else if (line.length() < 40) line += ch;
  }
  if (!heardHost && nowMs - bannerMs > 5000) {
    bannerMs = nowMs;
    Serial.print("==== nvm_probe version " PROBE_VERSION " | built " __DATE__ " " __TIME__ " ====  type ? for the commands\n");
  }
}
