// NvmRecord.h — the record that is stored in non-volatile memory, and its check (Requirements NVM-1).
//
// Layout, little-endian, 22 bytes (schema 2):
//   0..1   magic  0x324E  ("N2" as stored: 4E 32)      neither 0x0000 nor 0xFFFF, so blank and zeroed flash are both rejected
//   2      schema 2                                     a later program version that changes the payload adds a schema number
//   3      length of the payload (here 8)
//   4..7   WRITE COUNT: how many times settings have been written (= block erases of the two settings blocks, A and B together);
//          the newer of A/B has the higher count. Carried forward on every save, never reset. Each block is erased about half of
//          this number of times; the part is rated 100,000 erases per block, so the count must stay far below 200,000 (kNvmWearWarnWrites).
//   8..19  payload: tbsDebounceMs, tobDebounceMs, boardId, reserved (0), sketchVersion u16 (the sketch that wrote it, e.g. 0x0801),
//           reserved u16 (0), savedAt u32 (RTC time of the save, seconds since 2026-01-01 00:00:00; 0 = unknown, no valid RTC)
//   20..21 CRC-16/CCITT-FALSE over bytes 0..19
// A CRC-16 catches every single-bit error, every double-bit error, every burst up to 16 bits and 99.998 % of anything else, so a
// record left behind by another program passes by chance about 1 time in 65536 — and only if the magic, schema and length also match.
// The sequence number is what tells the newer copy from the older one. Parity (one check bit) would let half of all garbage through.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace n2 {

constexpr uint16_t kNvmMagic = 0x324E;
constexpr uint8_t kNvmSchema = 2;
constexpr size_t kNvmRecordBytes = 22;
constexpr uint8_t kNvmPayloadBytes = 12;
// Warn (log + LCD) when the write count reaches this: a quarter of the rated life of the whole pair (2 blocks x 100,000 erases).
constexpr uint32_t kNvmWearWarnWrites = 50000;

enum class BoardId : uint8_t { kUnknown = 0, kMinima = 1, kWifi = 2 };

struct NvmSettings {
  uint8_t tbsDebounceMs = 0;
  uint8_t tobDebounceMs = 0;
  BoardId board = BoardId::kUnknown;
  uint32_t savedAtSec = 0;      // RTC time of the save (secondsSince2026); 0 = unknown
  uint16_t sketchVersion = 0;   // the sketch version that wrote the record (hex-style: 0x0103 = 1.3)
};

enum class RecordStatus : uint8_t { kOk, kBlank, kBadMagic, kBadSchema, kBadLength, kBadCrc };

// Each check on its own, so a report can say exactly what failed. A region that was never written (blank 0xFF, or another program's
// data) fails the magic AND the checksum: that is what "never set" looks like.
struct RecordChecks {
  bool magic = false, schema = false, length = false, crc = false;
  bool blank = false;   // every byte 0xFF
  bool all() const { return magic && schema && length && crc; }
};
RecordChecks inspectRecord(const uint8_t in[kNvmRecordBytes]);

uint16_t crc16(const uint8_t* data, size_t n);
const char* recordStatusName(RecordStatus s);

void encodeRecord(uint32_t sequence, const NvmSettings& s, uint8_t out[kNvmRecordBytes]);
RecordStatus decodeRecord(const uint8_t in[kNvmRecordBytes], uint32_t& sequence, NvmSettings& s);

}  // namespace n2
