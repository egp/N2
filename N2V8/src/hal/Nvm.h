// Nvm.h — non-volatile memory interface (Requirements NVM-1, NFR-9).
//
// On the UNO R4 (RA4M1) this is the 8 KB data flash seen through the core's EEPROM library. The facts that shape the design,
// read from the core source (renesas_uno 1.6.0, libraries/BlockDevices/virtualEEPROM.cpp):
//   - the flash is erased and written a whole BLOCK at a time (1 KB): the core copies the block to RAM, changes bytes, and
//     erases + programs the block again. Changing ONE byte costs one block erase. Erased flash reads 0xFF.
//   - so: several settings go in ONE record written with ONE call; a record never straddles two blocks; the two copies of a
//     record (A and B) live in different blocks, so a power failure while writing one cannot damage the other.
//   - an erase takes milliseconds and the CPU waits for it: never write from the control loop. Only from the console or BIST.
//   - the contents of a board never written by this program are UNKNOWN (blank 0xFF, or anything left by another sketch).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace n2 {

class Nvm {
 public:
  virtual ~Nvm() = default;
  virtual size_t size() const = 0;       // total bytes
  virtual size_t blockSize() const = 0;  // erase/program unit in bytes (1024 on the R4)
  virtual void read(size_t addr, uint8_t* data, size_t n) = 0;
  // Replace n bytes at addr. The caller keeps the range inside ONE block. False if the device reported an error.
  virtual bool write(size_t addr, const uint8_t* data, size_t n) = 0;
};

}  // namespace n2
