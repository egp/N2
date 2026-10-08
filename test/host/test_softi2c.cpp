// SoftI2c (bit-banged I2C on two GPIO pins, Requirements DRV-4) and the LED driver on it.
#include <catch2/catch_test_macros.hpp>

#include "SoftBusModel.h"
#include "TestSupport.h"
#include "drivers/Led1650.h"
#include "drivers/SoftI2c.h"

using namespace n2;

namespace {
constexpr uint8_t kSda = 2, kScl = 3;
struct Rig {
  FakeHal hal;
  SoftBusModel bus{hal, kSda, kScl};
  SoftI2c soft{hal, kSda, kScl};
  Rig() { soft.begin(); }
};
}  // namespace

TEST_CASE("DRV-4: a one-byte write is START, address+W, the byte MSB first, STOP, and both lines end released") {
  Rig r;
  r.bus.addresses = {0x24};
  const uint8_t b = 0x11;
  CHECK(r.soft.write(0x24, &b, 1));
  r.bus.sync();
  REQUIRE(r.bus.transactions.size() == 1);
  const SoftBusTransaction& t = r.bus.transactions[0];
  CHECK(t.address == 0x24);
  CHECK(t.write);
  CHECK(t.addressAcked);
  REQUIRE(t.bytes.size() == 1);
  CHECK(t.bytes[0] == 0x11);
  CHECK(t.acked[0]);
  CHECK(r.bus.sdaHigh());
  CHECK(r.bus.sclHigh());
}

TEST_CASE("DRV-4: every byte value goes out bit-exact (all 256), with the address shifted left") {
  Rig r;
  r.bus.addresses = {0x37};
  for (int v = 0; v < 256; ++v) {
    const uint8_t b = static_cast<uint8_t>(v);
    REQUIRE(r.soft.write(0x37, &b, 1));
  }
  r.bus.sync();
  REQUIRE(r.bus.transactions.size() == 256);
  for (int v = 0; v < 256; ++v) {
    CHECK(r.bus.transactions[v].address == 0x37);
    REQUIRE(r.bus.transactions[v].bytes.size() == 1);
    CHECK(r.bus.transactions[v].bytes[0] == v);
  }
}

TEST_CASE("DRV-4: a device that does not answer makes write() and probe() false, and the bus is still released with a STOP") {
  Rig r;
  r.bus.addresses = {0x24};
  const uint8_t b = 0x55;
  CHECK_FALSE(r.soft.write(0x30, &b, 1));
  CHECK_FALSE(r.soft.probe(0x31));
  r.bus.sync();
  REQUIRE(r.bus.transactions.size() == 2);  // both ended in a STOP
  CHECK_FALSE(r.bus.transactions[0].addressAcked);
  CHECK(r.bus.sdaHigh());
  CHECK(r.bus.sclHigh());
  CHECK(r.soft.probe(0x24));
}

TEST_CASE("DRV-4: probe() is the address byte alone") {
  Rig r;
  r.bus.addresses = {0x24};
  CHECK(r.soft.probe(0x24));
  r.bus.sync();
  REQUIRE(r.bus.transactions.size() == 1);
  CHECK(r.bus.transactions[0].bytes.empty());
}

TEST_CASE("DRV-4: a slave that refuses data bytes beyond the first (the TM1650) makes a longer write false") {
  Rig r;
  r.bus.addresses = {0x24};
  r.bus.maxDataBytes = 1;
  const uint8_t two[2] = {0x11, 0x22};
  CHECK_FALSE(r.soft.write(0x24, two, 2));
  const uint8_t one = 0x11;
  CHECK(r.soft.write(0x24, &one, 1));
}

TEST_CASE("DRV-4: an unconfigured soft bus (no pins) never touches a pin and always fails") {
  FakeHal hal;
  SoftI2c soft(hal, SoftI2c::kNoPin, SoftI2c::kNoPin);
  soft.begin();
  const uint8_t b = 1;
  CHECK_FALSE(soft.configured());
  CHECK_FALSE(soft.write(0x24, &b, 1));
  CHECK_FALSE(soft.probe(0x24));
  CHECK(hal.events.empty());
}

TEST_CASE("DRV-4: recover() clocks SCL until a stuck slave lets go of SDA, then sends a STOP") {
  Rig r;
  r.bus.addresses = {0x24};
  r.bus.stuckUntilPulse = 5;  // the slave holds SDA low until it has seen 5 clock pulses
  CHECK_FALSE(r.bus.sdaHigh());
  r.soft.recover();
  CHECK(r.bus.sdaHigh());
  CHECK(r.bus.sclHigh());
  CHECK(r.bus.pulses() >= 5);
  CHECK(r.bus.pulses() <= 5 + 2);
}

TEST_CASE("DRV-4: the bit-banged byte takes about 0.2 ms (the busy-wait is small, so the loop stays fast)") {
  Rig r;
  r.bus.addresses = {0x24};
  const uint8_t b = 0x11;
  const uint64_t before = r.hal.delayedUs;
  r.soft.write(0x24, &b, 1);
  const uint64_t us = r.hal.delayedUs - before;
  CHECK(us > 100);
  CHECK(us < 600);
}

TEST_CASE("DRV-4: the LED driver on a soft bus writes the control byte and the four digits as one-byte transactions") {
  Rig r;
  r.bus.addresses = {0x24, 0x34, 0x35, 0x36, 0x37};
  r.bus.maxDataBytes = 1;
  Led1650 led(r.hal, 0x24, 0x34, &r.soft);
  led.begin(0);
  LedText t;
  memcpy(t.digit, "1234", 5);
  t.dotAfter = 1;
  led.setText(t);
  for (uint32_t now = 0; now < 300 && !led.inSync(); now += 5) led.service(now);
  REQUIRE(led.inSync());
  CHECK(led.healthy());
  r.bus.sync();
  // control byte first, then digits 0..3 with their segments (the dot sets bit 7 of digit 1)
  REQUIRE(r.bus.transactions.size() == 5);
  CHECK(r.bus.transactions[0].address == 0x24);
  CHECK(r.bus.transactions[0].bytes[0] == 0x11);
  CHECK(r.bus.transactions[1].address == 0x34);
  CHECK(r.bus.transactions[1].bytes[0] == Led1650::segmentsFor('1'));
  CHECK(r.bus.transactions[2].address == 0x35);
  CHECK(r.bus.transactions[2].bytes[0] == (Led1650::segmentsFor('2') | 0x80));
  CHECK(r.bus.transactions[4].address == 0x37);
  CHECK(r.bus.transactions[4].bytes[0] == Led1650::segmentsFor('4'));
  CHECK(led.answeringAddresses() == 5);
  // and nothing went to the hardware bus
  CHECK(r.hal.i2cWrites.empty());
}

TEST_CASE("DRV-4: the LED driver on a soft bus reports a missing module and recovers when it answers again") {
  Rig r;
  Led1650 led(r.hal, 0x24, 0x34, &r.soft);
  led.begin(0);
  LedText t;
  memcpy(t.digit, "0000", 5);
  t.dotAfter = -1;
  led.setText(t);
  uint32_t now = 0;
  for (; now < 200; now += 5) led.service(now);
  CHECK_FALSE(led.healthy());            // nothing acknowledges
  r.bus.addresses = {0x24, 0x34, 0x35, 0x36, 0x37};
  r.bus.maxDataBytes = 1;
  for (; now < 3000 && !led.inSync(); now += 5) led.service(now);
  CHECK(led.healthy());
  CHECK(led.inSync());
}

// ============================================================================ the whole firmware with the LED on its own bus
#include "app/Bringup.h"

TEST_CASE("DRV-4: Bringup with the LED on its own pins: the LED works on the soft bus, nothing LED-related touches the hardware bus, the LCD is unaffected") {
  FakeHal hal;
  hal.i2cPresent = {0x23, 0x68};  // the hardware bus: the LCD (0x23) and the RTC; no LED addresses there at all
  hal.consoleIsAttached = true;
  hal.canDetectHost = false;
  const SignalDef& tob = def(kHostBoard, Signal::kTob);
  hal.inputLevel[tob.pin] = levelHigh(false, tob.active);
  SoftBusModel model(hal, kSda, kScl);
  model.addresses = {0x24, 0x25, 0x26, 0x27, 0x34, 0x35, 0x36, 0x37};  // the TM1650 answers all of these
  model.maxDataBytes = 1;
  BoardDef board = kHostBoard;
  board.ledSdaPin = kSda;
  board.ledSclPin = kScl;
  REQUIRE(checkBoard(board) == BoardCheck::kOk);
  BuildInfo info{"8.0.0-test", "Jan  1 2026", "12:00:00", "host (fake)", "HOST", 10};
  BringupOptions opt;
  opt.lcdStartMs = 0;
  opt.lcdAlwaysRewrite = false;
  opt.stage = "2";
  Bringup app(hal, board, info, nullptr, opt);
  app.setup();
  {
    Rtc3231 rtc(hal, 0x68);
    REQUIRE(rtc.set({2026, 10, 7, 9, 15, 0}));
  }
  for (uint32_t t = 0; t < 4000; t += 10) { hal.nowMs += 10; app.loop(); }
  model.sync();
  CHECK(app.selfTest().postFinished());
  CHECK(hal.consoleOut.find("LED    ok   0x24 + 4 digits ok") != std::string::npos);
  CHECK(app.led().healthy());
  CHECK(app.lcd().healthy());
  CHECK(app.lcd().ready());
  // the LED got its writes on the soft bus
  size_t controlWrites = 0, digitWrites = 0;
  for (const auto& t : model.transactions) {
    if (t.address == 0x24 && !t.bytes.empty()) ++controlWrites;
    if (t.address >= 0x34 && t.address <= 0x37 && !t.bytes.empty()) ++digitWrites;
  }
  CHECK(controlWrites >= 1);
  CHECK(digitWrites >= 4);
  // and not one LED address was used on the hardware bus
  for (const auto& w : hal.i2cWrites) {
    const bool ledAddress = (w.address >= 0x24 && w.address <= 0x27) || (w.address >= 0x34 && w.address <= 0x37);
    CHECK_FALSE(ledAddress);
  }
  hal.consoleOut.clear();
  hal.type("scan\n");
  for (uint32_t t = 0; t < 200; t += 10) { hal.nowMs += 10; app.loop(); }
  CHECK(hal.consoleOut.find("LED on its own bus (soft I2C D2/D3): 5 of 5 addresses answer") != std::string::npos);
  CHECK(hal.consoleOut.find("0x23  LCD backpack\n") != std::string::npos);
}
