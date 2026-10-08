// ===========================================================================================
// nvm_probe  VERSION 1.0   (2026-10-08)     <-- if you do not see this line, the IDE has an older copy
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
//   b            bounce measurement ON/OFF: operate TBS and TOB; each operation prints its edges and settle time
//   r            bounce result so far (operations, edges, settle min/mean/max, recommended debounce)
//   k            keep: save the recommended debounce times from the bounce measurement
//   z            clear the bounce statistics
//   Which board am I?  Type  m 1  (Minima) or  m 2  (WiFi): stored with the times, so a value measured on one board is never
//   applied to the other.
// Bounce rules of the road: leave the console quiet while operating a switch (printing takes time, the program prints only after
// the switch has been still for 0.2 s), and operate each switch 20+ times, press AND release.
//
// Change log
//   1.0  first version
// ===========================================================================================
#define PROBE_VERSION "1.0"

#include <Arduino.h>
#include <EEPROM.h>

#include "src/BoardPins.h"
#include "src/core/Debounce.h"
#include "src/core/SettingsStore.h"
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
  Serial.print(", settle us min "); Serial.print(s.settleMinUs); Serial.print(" mean "); Serial.print(m.settleMeanUs());
  Serial.print(" max "); Serial.print(s.settleMaxUs);
  Serial.print("  -> recommended debounce "); Serial.print(m.recommendedMs()); Serial.println(" ms");
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
    measuring = !measuring;
    if (measuring) { tbsMeter.reset(); tobMeter.reset(); tbsOps = tobOps = 0; passMaxUs = 0; }
    Serial.println(measuring ? "bounce measurement ON: operate TBS and TOB (20+ times each, press and release)" : "bounce measurement OFF");
  } else if (c == "r") {
    printMeter("TBS", tbsMeter); printMeter("TOB", tobMeter);
    Serial.print("longest loop pass "); Serial.print(passMaxUs); Serial.println(" us (edges closer than this could be missed)");
  } else if (c == "z") {
    tbsMeter.reset(); tobMeter.reset(); tbsOps = tobOps = 0; passMaxUs = 0; Serial.println("cleared");
  } else if (c == "k") {
    const uint8_t a = tbsMeter.recommendedMs(), b = tobMeter.recommendedMs();
    if (!a || !b) { Serial.println("measure BOTH switches first (b, operate them, r)"); return; }
    Serial.print("saving TBS "); Serial.print(a); Serial.print(" ms, TOB "); Serial.print(b); Serial.println(" ms");
    Serial.println(store.save(current(a, b)) ? "saved" : "SAVE FAILED"); printReport(store.load());
  } else {
    Serial.println("commands: i d l m 1|2 w TBS TOB t e b r z k");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(kTbsPin, INPUT_PULLUP);
  pinMode(kTobPin, INPUT_PULLUP);
}

void loop() {
  const uint32_t nowUs = micros();
  if (measuring) {                                   // the fast part: read both pins, timestamp, nothing else
    const bool tbs = digitalRead(kTbsPin) == LOW, tob = digitalRead(kTobPin) == LOW;
    const uint32_t gap = nowUs - lastPassUs;
    if (lastPassUs && gap > passMaxUs) passMaxUs = gap;
    tbsMeter.sample(tbs, nowUs); tobMeter.sample(tob, nowUs);
    tbsMeter.flush(nowUs); tobMeter.flush(nowUs);
    if (tbsMeter.stats().operations != tbsOps) { tbsOps = tbsMeter.stats().operations; Serial.print("TBS #"); Serial.print(tbsOps); Serial.print(" edges "); Serial.print(tbsMeter.stats().lastEdges); Serial.print(" settle us "); Serial.println(tbsMeter.stats().lastSettleUs); }
    if (tobMeter.stats().operations != tobOps) { tobOps = tobMeter.stats().operations; Serial.print("TOB #"); Serial.print(tobOps); Serial.print(" edges "); Serial.print(tobMeter.stats().lastEdges); Serial.print(" settle us "); Serial.println(tobMeter.stats().lastSettleUs); }
  }
  lastPassUs = nowUs;
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read()); heardHost = true;
    if (ch == '\n' || ch == '\r') { command(line); line = ""; } else if (line.length() < 40) line += ch;
  }
  if (!heardHost && millis() - bannerMs > 5000) {
    bannerMs = millis();
    Serial.print("==== nvm_probe version " PROBE_VERSION " | built " __DATE__ " " __TIME__ " ====  type ? for the commands\n");
  }
}
