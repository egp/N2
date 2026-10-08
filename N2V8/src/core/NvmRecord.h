// NvmRecord.h — the record that is stored in non-volatile memory, and its check (Requirements NVM-1).
//
// Layout, little-endian, 16 bytes:
//   0..1   magic  0x324E  ("N2" as stored: 4E 32)      neither 0x0000 nor 0xFFFF, so blank and zeroed flash are both rejected
//   2      schema 1                                     a later program version that changes the payload adds a schema number
//   3      length of the payload (here 6)
//   4..7   sequence number (counts saves; the newer of A/B wins)
//   8..13  payload: tbsDebounceMs, tobDebounceMs, boardId, reserved[3] (= 0)
//   14..15 CRC-16/CCITT-FALSE over bytes 0..13
// A CRC-16 catches every single-bit error, every double-bit error, every burst up to 16 bits and 99.998 % of anything else, so a
// record left behind by another program passes by chance about 1 time in 65536 — and only if the magic, schema and length also match.
// The sequence number is what tells the newer copy from the older one. Parity (one check bit) would let half of all garbage through.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace n2 {

constexpr uint16_t kNvmMagic = 0x324E;
constexpr uint8_t kNvmSchema = 1;
constexpr size_t kNvmRecordBytes = 16;
constexpr uint8_t kNvmPayloadBytes = 6;

enum class BoardId : uint8_t { kUnknown = 0, kMinima = 1, kWifi = 2 };

struct NvmSettings {
  uint8_t tbsDebounceMs = 0;
  uint8_t tobDebounceMs = 0;
  BoardId board = BoardId::kUnknown;
};

enum class RecordStatus : uint8_t { kOk, kBlank, kBadMagic, kBadSchema, kBadLength, kBadCrc };

uint16_t crc16(const uint8_t* data, size_t n);
const char* recordStatusName(RecordStatus s);

void encodeRecord(uint32_t sequence, const NvmSettings& s, uint8_t out[kNvmRecordBytes]);
RecordStatus decodeRecord(const uint8_t in[kNvmRecordBytes], uint32_t& sequence, NvmSettings& s);

}  // namespace n2
