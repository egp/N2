// FakeNvm.h — host model of the R4 data flash: blocks erased and written whole, unknown initial contents,
// power failure injected part-way through a write.
#pragma once
#include <vector>
#include <cstdlib>
#include "hal/Nvm.h"

class FakeNvm : public n2::Nvm {
 public:
  enum class Init { kBlank, kGarbage };
  explicit FakeNvm(Init init = Init::kBlank, unsigned seed = 1, size_t total = 8192, size_t block = 1024)
      : mem_(total, 0xFF), block_(block) {
    if (init == Init::kGarbage) { srand(seed); for (auto& b : mem_) b = static_cast<uint8_t>(rand()); }
  }
  size_t size() const override { return mem_.size(); }
  size_t blockSize() const override { return block_; }
  void read(size_t addr, uint8_t* d, size_t n) override { for (size_t i = 0; i < n; i++) d[i] = mem_[addr + i]; }
  bool write(size_t addr, const uint8_t* d, size_t n) override {
    if (addr / block_ != (addr + n - 1) / block_) return false;   // must stay in one block
    erases++;
    const size_t b0 = addr / block_ * block_;
    for (size_t i = 0; i < block_; i++) mem_[b0 + i] = 0xFF;      // the whole block is erased first
    const size_t limit = failAfterBytes_ < n ? failAfterBytes_ : n;  // power fails after this many bytes are programmed
    for (size_t i = 0; i < limit; i++) mem_[addr + i] = d[i];
    failAfterBytes_ = static_cast<size_t>(-1);
    return limit == n;
  }
  void failNextWriteAfter(size_t bytes) { failAfterBytes_ = bytes; }
  std::vector<uint8_t>& raw() { return mem_; }
  int erases = 0;
 private:
  std::vector<uint8_t> mem_;
  size_t block_;
  size_t failAfterBytes_ = static_cast<size_t>(-1);
};
