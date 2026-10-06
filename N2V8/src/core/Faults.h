// Faults.h — the fault table and the active-fault set (Requirements §9, FLT-1..FLT-4).
#pragma once

#include <stdint.h>

#include "Log.h"

namespace n2 {

enum class Severity : uint8_t { kInfo, kWarn, kInhibit };

enum class FaultId : uint8_t {
  kAirSensor,       // F01
  kN2LowSensor,     // F02
  kN2HighSensor,    // F03
  kSensorOrder,     // F04
  kLcd,             // F10
  kLed,             // F11
  kO2Comm,          // F12
  kRtc,             // F13
  kInvariant,       // F20
  kWatchdogReset,   // F30
  kBrownoutReset,   // F31
  kConsoleDrop,     // F40
  kCount
};
constexpr uint8_t kFaultCount = static_cast<uint8_t>(FaultId::kCount);

struct FaultInfo {
  uint8_t code;
  const char* text;    // at most 16 characters, so "Fnn " + text fits one 20-column LCD row
  Severity severity;
  bool latching;       // stays active until reset
  const char* effect;  // what the system does about it, at most 20 characters (fault screen, row 3)
};

// The one fault table (FLT-4): adding a fault is one row here plus one test.
const FaultInfo& faultInfo(FaultId id);

class FaultSet {
 public:
  explicit FaultSet(LogSink& log) : log_(log) {}

  // Report the current truth of a fault's condition. A true condition raises it at once.
  // A false condition clears it only after it has stayed false for holdMs (FLT-1);
  // latching faults never clear.
  void report(FaultId id, bool condition, uint32_t now, uint32_t holdMs);

  bool active(FaultId id) const { return slot_[static_cast<uint8_t>(id)].active; }
  uint8_t activeCount(Severity atLeast = Severity::kInfo) const;
  bool anyAtLeast(Severity s) const { return activeCount(s) > 0; }
  // The n-th active fault of at least the given severity, in table order. False if there is none.
  bool nth(uint8_t n, Severity atLeast, FaultId& out) const;

 private:
  struct Slot {
    bool active = false;
    bool clearing = false;
    uint32_t clearSince = 0;
  };
  LogSink& log_;
  Slot slot_[kFaultCount];
};

}  // namespace n2
