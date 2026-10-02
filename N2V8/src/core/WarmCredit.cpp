#include "WarmCredit.h"

namespace n2 {

uint32_t warmRecordChecksum(const WarmRecord& r) {
  return r.magic ^ (r.boots * 0x9E3779B1u) ^ (r.lastSeenMs * 0x85EBCA6Bu) ^ (r.runMs * 0xC2B2AE35u) ^ 0xA5A5A5A5u;
}

bool warmRecordValid(const WarmRecord& r) { return r.magic == kWarmRecordMagic && r.check == warmRecordChecksum(r); }

uint32_t WarmCredit::begin(const ResetInfo& reset, uint32_t bootMs) {
  recordWasValid_ = warmRecordValid(rec_);
  const bool trusted = enabled_ && reset.known && !reset.powerOn && recordWasValid_;
  baseMs_ = trusted ? rec_.runMs : 0;
  startMs_ = bootMs;
  rec_.magic = kWarmRecordMagic;
  rec_.boots = recordWasValid_ && !reset.powerOn ? rec_.boots + 1 : 1;
  seal(bootMs);
  return baseMs_;
}

void WarmCredit::seal(uint32_t nowMs) {
  const uint32_t run = baseMs_ + static_cast<uint32_t>(nowMs - startMs_);
  rec_.runMs = run < baseMs_ ? 0xFFFFFFFFu : run;  // saturate
  rec_.lastSeenMs = nowMs;
  rec_.check = warmRecordChecksum(rec_);
}

void WarmCredit::tick(uint32_t nowMs) { seal(nowMs); }

void WarmCredit::restart(uint32_t nowMs) {
  baseMs_ = 0;
  startMs_ = nowMs;
  seal(nowMs);
}

}  // namespace n2
