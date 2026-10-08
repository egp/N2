// Tests for the NVM record, the A/B settings store and the debounce code (NVM-1, INP-6, INP-10).
#include <catch2/catch_test_macros.hpp>
#include "FakeNvm.h"
#include "core/Debounce.h"
#include "core/SettingsStore.h"

using namespace n2;

namespace {
NvmSettings mk(uint8_t tbs, uint8_t tob, BoardId b = BoardId::kWifi, uint16_t ver = 0x0103) { NvmSettings s; s.tbsDebounceMs = tbs; s.tobDebounceMs = tob; s.board = b; s.sketchVersion = ver; return s; }
}

TEST_CASE("crc16 matches the CCITT-FALSE check value", "[nvm]") {
  const uint8_t v[] = {'1','2','3','4','5','6','7','8','9'};
  REQUIRE(crc16(v, 9) == 0x29B1);
}

TEST_CASE("record round trip and every single-bit flip is detected", "[nvm]") {
  uint8_t rec[kNvmRecordBytes];
  encodeRecord(7, mk(12, 34), rec);
  uint32_t seq = 0; NvmSettings s;
  REQUIRE(decodeRecord(rec, seq, s) == RecordStatus::kOk);
  REQUIRE(seq == 7); REQUIRE(s.tbsDebounceMs == 12); REQUIRE(s.tobDebounceMs == 34); REQUIRE(s.board == BoardId::kWifi); REQUIRE(s.sketchVersion == 0x0103);
  for (size_t byte = 0; byte < kNvmRecordBytes; byte++)
    for (int bit = 0; bit < 8; bit++) {
      uint8_t bad[kNvmRecordBytes]; for (size_t i = 0; i < sizeof bad; i++) bad[i] = rec[i];
      bad[byte] ^= static_cast<uint8_t>(1 << bit);
      INFO("byte " << byte << " bit " << bit);
      REQUIRE(decodeRecord(bad, seq, s) != RecordStatus::kOk);
    }
}

TEST_CASE("the RTC time of the save is stored (0 = unknown) and covered by the checksum", "[nvm]") {
  NvmSettings in = mk(10, 20); in.savedAtSec = 0x2A3B4C5Du;
  uint8_t rec[kNvmRecordBytes]; encodeRecord(1, in, rec);
  uint32_t seq; NvmSettings out;
  REQUIRE(decodeRecord(rec, seq, out) == RecordStatus::kOk); REQUIRE(out.savedAtSec == 0x2A3B4C5Du);
  rec[18] ^= 0x01; REQUIRE(decodeRecord(rec, seq, out) == RecordStatus::kBadCrc);
  encodeRecord(1, mk(10, 20), rec); REQUIRE(decodeRecord(rec, seq, out) == RecordStatus::kOk); REQUIRE(out.savedAtSec == 0);
}

TEST_CASE("blank and zeroed flash are not records", "[nvm]") {
  uint8_t ff[kNvmRecordBytes], zero[kNvmRecordBytes] = {0};
  for (auto& b : ff) b = 0xFF;
  uint32_t seq; NvmSettings s;
  REQUIRE(decodeRecord(ff, seq, s) == RecordStatus::kBlank);
  REQUIRE(decodeRecord(zero, seq, s) == RecordStatus::kBadMagic);
}

TEST_CASE("random garbage is (almost) never accepted: 20000 trials", "[nvm]") {
  srand(42);
  int accepted = 0;
  for (int t = 0; t < 20000; t++) {
    uint8_t g[kNvmRecordBytes]; for (auto& b : g) b = static_cast<uint8_t>(rand());
    uint32_t seq; NvmSettings s;
    if (decodeRecord(g, seq, s) == RecordStatus::kOk) accepted++;
  }
  REQUIRE(accepted == 0);   // needs the magic, schema AND length to match first: odds ~ 1 in 2^40
}

TEST_CASE("unknown initial contents: blank or garbage gives no valid settings and load never writes", "[nvm]") {
  for (unsigned seed = 1; seed <= 50; seed++) {
    FakeNvm nvm(FakeNvm::Init::kGarbage, seed);
    SettingsStore st(nvm);
    REQUIRE_FALSE(st.load().valid);
    REQUIRE(nvm.erases == 0);
  }
  FakeNvm blank;
  SettingsStore st(blank);
  const StoreReport r = st.load();
  REQUIRE_FALSE(r.valid); REQUIRE(r.a == RecordStatus::kBlank); REQUIRE(r.b == RecordStatus::kBlank);
}

TEST_CASE("save then load; copies alternate A, B, A; sequence counts", "[nvm]") {
  FakeNvm nvm(FakeNvm::Init::kGarbage, 9);
  SettingsStore st(nvm);
  REQUIRE(st.save(mk(10, 20)));
  StoreReport r = st.load(); REQUIRE(r.valid); REQUIRE(r.which == 'A'); REQUIRE(r.sequence == 1); REQUIRE(r.settings.tbsDebounceMs == 10);
  REQUIRE(st.save(mk(11, 21)));
  r = st.load(); REQUIRE(r.which == 'B'); REQUIRE(r.sequence == 2); REQUIRE(r.settings.tobDebounceMs == 21);
  REQUIRE(st.save(mk(12, 22)));
  r = st.load(); REQUIRE(r.which == 'A'); REQUIRE(r.sequence == 3);
}

TEST_CASE("saving identical settings does not erase flash", "[nvm]") {
  FakeNvm nvm; SettingsStore st(nvm);
  REQUIRE(st.save(mk(10, 20))); REQUIRE(st.lastSaveWrote());
  const int e = nvm.erases;
  REQUIRE(st.save(mk(10, 20))); REQUIRE_FALSE(st.lastSaveWrote());
  REQUIRE(nvm.erases == e);
}

TEST_CASE("power failure at every byte of a save never loses the previous settings", "[nvm]") {
  for (size_t cut = 0; cut <= kNvmRecordBytes; cut++) {
    FakeNvm nvm(FakeNvm::Init::kGarbage, 3);
    SettingsStore st(nvm);
    REQUIRE(st.save(mk(10, 20)));
    REQUIRE(st.save(mk(11, 21)));
    nvm.failNextWriteAfter(cut);
    st.save(mk(12, 22));                    // interrupted (or complete when cut == 16)
    const StoreReport r = SettingsStore(nvm).load();
    INFO("cut " << cut);
    REQUIRE(r.valid);
    if (cut < kNvmRecordBytes) { REQUIRE(r.settings.tbsDebounceMs == 11); REQUIRE(r.sequence == 2); }
    else                       { REQUIRE(r.settings.tbsDebounceMs == 12); REQUIRE(r.sequence == 3); }
  }
}

TEST_CASE("power failure during the very first save leaves no valid settings, not wrong ones", "[nvm]") {
  for (size_t cut = 0; cut < kNvmRecordBytes; cut++) {
    FakeNvm nvm(FakeNvm::Init::kGarbage, 5);
    SettingsStore st(nvm);
    nvm.failNextWriteAfter(cut);
    st.save(mk(10, 20));
    REQUIRE_FALSE(SettingsStore(nvm).load().valid);
  }
}

TEST_CASE("the sequence number may wrap", "[nvm]") {
  FakeNvm nvm; uint8_t rec[kNvmRecordBytes];
  encodeRecord(0xFFFFFFFFu, mk(10, 20), rec); nvm.write(0, rec, sizeof rec);
  encodeRecord(0x00000000u, mk(11, 21), rec); nvm.write(1024, rec, sizeof rec);
  const StoreReport r = SettingsStore(nvm).load();
  REQUIRE(r.which == 'B'); REQUIRE(r.settings.tbsDebounceMs == 11);
}

TEST_CASE("a device too small for two blocks is refused", "[nvm]") {
  FakeNvm nvm(FakeNvm::Init::kBlank, 1, 1024, 1024);
  SettingsStore st(nvm);
  REQUIRE_FALSE(st.fits()); REQUIRE_FALSE(st.save(mk(10, 20)));
}

TEST_CASE("Debouncer accepts a level only after it holds, and survives millis rollover", "[debounce]") {
  Debouncer d(false, 10);
  uint32_t t = 0xFFFFFFF0u;
  d.update(true, t);                         t += 5;
  d.update(false, t);                        t += 5;      // bounce back: restarts nothing accepted
  REQUIRE_FALSE(d.level());
  d.update(true, t);                         t += 9;
  REQUIRE_FALSE(d.update(true, t));          t += 1;      // 9 ms: not yet
  REQUIRE(d.update(true, t)); REQUIRE(d.changed());       // 10 ms (wrapped past zero)
  d.update(true, t + 1); REQUIRE_FALSE(d.changed());
}

TEST_CASE("BounceMeter counts edges and settle time from micros()", "[debounce]") {
  BounceMeter m(100000);
  uint32_t t = 4294960000u;                                // near rollover on purpose
  auto run = [&](bool lvl, uint32_t us) { m.sample(lvl, t); t += us; };
  run(false, 1000);
  // press: edges at +0, +300, +700, +1500 us then stable
  run(true, 300); run(false, 400); run(true, 800); run(false, 1000); run(true, 1000);   // the last sample set level true
  for (int i = 0; i < 200; i++) run(true, 1000);           // quiet period > 100 ms
  m.flush(t);
  const BounceStats& s = m.stats();
  REQUIRE(s.operations == 1);
  REQUIRE(s.edgesTotal == 5);
  REQUIRE(s.settleMaxUs == 2500);                          // from first to last edge: 300+400+800+1000 = 2500 us
  REQUIRE(m.recommendedMs() == 5);                         // 2 x 2.5 ms
}

TEST_CASE("BounceMeter recommendation is limited to 2..100 ms and 0 when nothing was measured", "[debounce]") {
  BounceMeter m; REQUIRE(m.recommendedMs() == 0);
  uint32_t t = 0;
  m.sample(false, t); t += 10; m.sample(true, t); t += 500000; m.sample(true, t);      // clean edge, settle 0
  REQUIRE(m.stats().operations == 1); REQUIRE(m.recommendedMs() == kMinDebounceMs);
  BounceMeter big(50000);
  t = 0; big.sample(false, t);
  for (int i = 0; i < 100; i++) { t += 4000; big.sample(i & 1, t); }                  // 400 ms of chatter
  t += 100000; big.sample(false, t);
  REQUIRE(big.recommendedMs() == kMaxDebounceMs);
}

TEST_CASE("BounceMeter median, mean, min, max of the settle times", "[debounce]") {
  BounceMeter m(1000);
  uint32_t t = 0; bool lvl = false; m.sample(lvl, t);
  const uint32_t settles[] = {100, 300, 200, 900};         // microseconds, first edge to last edge of each operation
  for (uint32_t st : settles) {
    t += 10000; lvl = !lvl; m.sample(lvl, t);               // first edge
    t += st;    lvl = !lvl; m.sample(lvl, t);               // second edge st later ...
    t += 1;     lvl = !lvl; m.sample(lvl, t);               // ... and a third (so the settle is st+1)
    t += 5000;  m.sample(lvl, t);                           // quiet: closes the operation
  }
  const BounceStats& s = m.stats();
  REQUIRE(s.operations == 4);
  REQUIRE(s.settleMinUs == 101); REQUIRE(s.settleMaxUs == 901);
  REQUIRE(m.settleMedianUs() == (201 + 301) / 2);
  REQUIRE(m.settleMeanUs() == (101 + 301 + 201 + 901) / 4);
  REQUIRE_FALSE(m.inOperation());
}

TEST_CASE("a press and its release 100 ms later are two operations, hold time is not bounce", "[debounce]") {
  BounceMeter m;                                           // default quiet window
  uint32_t t = 0; m.sample(false, t);
  t += 50000; m.sample(true, t);
  t += 300; m.sample(false, t); t += 200; m.sample(true, t);   // press with a little bounce
  for (int i = 0; i < 100; i++) { t += 1000; m.sample(true, t); }   // held 100 ms
  REQUIRE(m.stats().operations == 1); REQUIRE(m.stats().lastToOn); REQUIRE(m.stats().lastSettleUs == 500);
  t += 1000; m.sample(false, t);                           // release, clean
  for (int i = 0; i < 50; i++) { t += 1000; m.sample(false, t); }
  REQUIRE(m.stats().operations == 2); REQUIRE_FALSE(m.stats().lastToOn); REQUIRE(m.stats().lastSettleUs == 0);
}

TEST_CASE("sketch version is stored; a version change is a write; the write count carries forward and warns", "[nvm]") {
  FakeNvm nvm; SettingsStore st(nvm);
  REQUIRE(st.save(mk(10, 20, BoardId::kWifi, 0x0103)));
  REQUIRE(st.load().settings.sketchVersion == 0x0103);
  const int e = nvm.erases;
  REQUIRE(st.save(mk(10, 20, BoardId::kWifi, 0x0104)));      // same numbers, new sketch: recorded
  REQUIRE(nvm.erases == e + 1); REQUIRE(st.load().settings.sketchVersion == 0x0104); REQUIRE(st.load().sequence == 2);
  for (int i = 0; i < 20; i++) st.save(mk(static_cast<uint8_t>(11 + i), 20));
  REQUIRE(st.load().sequence == 22);                          // write count = saves that wrote
  REQUIRE_FALSE(st.load().wearWarning());
  uint8_t rec[kNvmRecordBytes]; encodeRecord(kNvmWearWarnWrites, mk(10, 20), rec); FakeNvm worn; worn.write(0, rec, sizeof rec);
  REQUIRE(SettingsStore(worn).load().wearWarning());
}
