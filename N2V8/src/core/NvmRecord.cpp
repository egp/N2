// NvmRecord.cpp — see NvmRecord.h.
#include "NvmRecord.h"

namespace n2 {

uint16_t crc16(const uint8_t* data, size_t n) {  // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

const char* recordStatusName(RecordStatus s) {
  switch (s) {
    case RecordStatus::kOk: return "ok";
    case RecordStatus::kBlank: return "blank (erased)";
    case RecordStatus::kBadMagic: return "not ours (bad magic)";
    case RecordStatus::kBadSchema: return "unknown schema";
    case RecordStatus::kBadLength: return "bad length";
    case RecordStatus::kBadCrc: return "damaged (bad CRC)";
  }
  return "?";
}

RecordChecks inspectRecord(const uint8_t in[kNvmRecordBytes]) {
  RecordChecks c;
  c.blank = true;
  for (size_t i = 0; i < kNvmRecordBytes; i++) c.blank = c.blank && in[i] == 0xFF;
  c.magic = (static_cast<uint16_t>(in[1]) << 8 | in[0]) == kNvmMagic;
  c.schema = in[2] == kNvmSchema;
  c.length = in[3] == kNvmPayloadBytes;
  c.crc = crc16(in, 16) == static_cast<uint16_t>(in[17] << 8 | in[16]);
  return c;
}

void encodeRecord(uint32_t sequence, const NvmSettings& s, uint8_t out[kNvmRecordBytes]) {
  out[0] = static_cast<uint8_t>(kNvmMagic & 0xFF);
  out[1] = static_cast<uint8_t>(kNvmMagic >> 8);
  out[2] = kNvmSchema;
  out[3] = kNvmPayloadBytes;
  for (int i = 0; i < 4; i++) out[4 + i] = static_cast<uint8_t>(sequence >> (8 * i));
  out[8] = s.tbsDebounceMs;
  out[9] = s.tobDebounceMs;
  out[10] = static_cast<uint8_t>(s.board);
  out[11] = 0;
  out[12] = static_cast<uint8_t>(s.sketchVersion & 0xFF);
  out[13] = static_cast<uint8_t>(s.sketchVersion >> 8);
  out[14] = out[15] = 0;
  const uint16_t crc = crc16(out, 16);
  out[16] = static_cast<uint8_t>(crc & 0xFF);
  out[17] = static_cast<uint8_t>(crc >> 8);
}

RecordStatus decodeRecord(const uint8_t in[kNvmRecordBytes], uint32_t& sequence, NvmSettings& s) {
  bool allFf = true;
  for (size_t i = 0; i < kNvmRecordBytes; i++) allFf = allFf && in[i] == 0xFF;
  if (allFf) return RecordStatus::kBlank;
  if ((static_cast<uint16_t>(in[1]) << 8 | in[0]) != kNvmMagic) return RecordStatus::kBadMagic;
  if (in[2] != kNvmSchema) return RecordStatus::kBadSchema;
  if (in[3] != kNvmPayloadBytes) return RecordStatus::kBadLength;
  const uint16_t stored = static_cast<uint16_t>(in[17] << 8 | in[16]);
  if (crc16(in, 16) != stored) return RecordStatus::kBadCrc;
  sequence = 0;
  for (int i = 0; i < 4; i++) sequence |= static_cast<uint32_t>(in[4 + i]) << (8 * i);
  s.tbsDebounceMs = in[8];
  s.tobDebounceMs = in[9];
  s.board = static_cast<BoardId>(in[10]);
  s.sketchVersion = static_cast<uint16_t>(in[13] << 8 | in[12]);
  return RecordStatus::kOk;
}

}  // namespace n2
