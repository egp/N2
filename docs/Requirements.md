# N2 Generator Controller — Requirements
*First draft. Edit freely. Delete what doesn't matter. Add what's missing.*

---

## 1. Purpose

Control a two-tower pressure-swing nitrogen generator. The controller manages
air supply valves, an N2 compressor, an O2 flush/sample valve, and two displays.
It reads five pressure sensors and one O2 sensor, and responds to an on/off switch.

---

## 2. Core Design Principles

- **No `delay()` calls anywhere.** All timed behavior uses a non-blocking clock
  abstraction (`IClock`, `TimedStateMachine`).
- **Three independent controllers**, each with its own state machine:
  `TowerController`, `O2Controller`, `N2Controller`.
- **Two independent displays**: a 4-digit 7-segment display and a 20×4 LCD.
  Both are updated on every `loop()` iteration.
- **One config struct** per controller holding its tunable parameters.
- **One shared `InputSnapshot`** passed to all controllers on each update cycle.
  Controllers read from it; they do not read hardware directly.
- The main loop calls each controller's `step()` as fast as possible.
  Controllers decide internally when to act.

---

## 3. Hardware I/O

### 3.1 Inputs

| Signal | Type | Notes |
|---|---|---|
| Black (on/off) switch | Digital in | Pull-up to Vcc; switch closes to ground. LOW = ON, HIGH = OFF. |
| Air supply pressure | Analog in | 0–150 PSI range, 0.5–4.5 V output |
| Left tower pressure | Analog in | 0–150 PSI range, 0.5–4.5 V output |
| Right tower pressure | Analog in | 0–150 PSI range, 0.5–4.5 V output |
| Low-pressure N2 | Analog in | 0–30 PSI range, 0.5–4.5 V output |
| High-pressure N2 | Analog in | 0–150 PSI range, 0.5–4.5 V output |
| O2 sensor | I2C | Reads percent O2 by volume (0–25%) |

### 3.2 Outputs

| Signal | Type | Notes |
|---|---|---|
| Left tower valve | Digital out | HIGH = open, LOW = closed |
| Right tower valve | Digital out | HIGH = open, LOW = closed |
| O2 flush valve | Digital out | HIGH = open, LOW = closed |
| N2 compressor (SSR) | Digital out | HIGH = on, LOW = off |
| 4-digit 7-segment display | I2C | |
| 20×4 LCD | I2C | |

### 3.3 Pressure Sensor Scaling

All sensors output 0.5 V at zero and 4.5 V at full scale (10%–90% of ADC range).
Readings outside this window are clamped. Scaled integer values use fixed-point:
- `_x10` suffix = value × 10 (one decimal place, e.g. 120.0 PSI → 1200)
- `_x100` suffix = value × 100 (two decimal places, e.g. 12.50 PSI → 1250)

---

## 4. System Enable / Disable

The black switch enables and disables the entire system.

- **When disabled:** all valves close, compressor stops, displays turn off.
  Controllers reset to their safe initial states.
- **When enabled:** controllers start running normally.
- The switch is read on every main loop iteration.
- Pin uses `INPUT_PULLUP`. System is ON when pin reads LOW.

---

## 5. Tower Controller

Alternates between two adsorption towers to separate N2 from air.
**Cycling is purely time-based.** Tower pressures are not used to gate switching.

### 5.1 States

```
INACTIVE → LEFT_ONLY → BOTH_AFTER_LEFT → RIGHT_ONLY → BOTH_AFTER_RIGHT → LEFT_ONLY → ...
                                                                           (cycle repeats)
LOW_SUPPLY  (entered from any active state; exits when supply recovers)
```

### 5.2 Behavior

- Starts `INACTIVE` with both valves closed.
- On enable, immediately opens left valve and starts timer (`towerOpenMs`).
- When timer expires, opens right valve (overlap begins), starts overlap timer (`overlapMs`).
- When overlap expires, closes left valve. Right tower now running alone.
- Mirrors the above to switch back to left tower.
- `BOTH_AFTER_LEFT` and `BOTH_AFTER_RIGHT` exist only for the overlap duration.
  Both valves are open during overlap.

### 5.3 Air Supply Hysteresis

Tower cycling is gated on air supply pressure with a deadband:

- If **active** and supply falls to `airSupplyOffPsi_x10` or below → enter `LOW_SUPPLY`
  (both valves close, no cycling).
- While in `LOW_SUPPLY`, if supply rises to `airSupplyOnPsi_x10` or above → restart
  from `LEFT_ONLY`.
- Supply in the deadband between the two thresholds holds the current state.

### 5.4 Configuration

| Parameter | Description |
|---|---|
| `towerOpenMs` | How long each tower runs alone |
| `overlapMs` | Duration both valves are open during a switch |
| `airSupplyOnPsi_x10` | Supply threshold to (re)start cycling |
| `airSupplyOffPsi_x10` | Supply threshold to stop cycling |

---

## 6. O2 Controller

Periodically measures oxygen concentration. N2 purity = 100% − O2%.

### 6.1 States

```
UNINITIALIZED → WARMUP → WAITING_TO_FLUSH → FLUSHING → SETTLING → SAMPLING
                              ↑                                        |
                              └──────── WAITING_FOR_NEXT_SAMPLE ←─────┘
                              └──────── ERROR_BACKOFF ←───────────────┘
```

### 6.2 Startup / Initialization

- The O2 sensor requires a **5-minute warm-up** before its readings are valid.
  This is represented by the `WARMUP` state.
- `init()` calls `sensor.begin()`. If that fails:
  - Print error to serial console.
  - Display `00.00` for N2% on both displays.
  - Retry `sensor.begin()` once per minute (`initRetryMs`).
  - Do not block or halt; the rest of the system continues running normally.
- Once `sensor.begin()` succeeds, enter `WARMUP` for `warmupDurationMs`
  (nominally 5 minutes), then proceed to `WAITING_TO_FLUSH`.

### 6.3 Measurement Cycle

A new cycle starts from `WAITING_TO_FLUSH` when:
- No value has ever been successfully measured, **or**
- `measurementIntervalMs` has elapsed since the last successful measurement.

**Cycle steps:**
1. Open flush valve, hold for `flushDurationMs` (`FLUSHING`).
2. Close flush valve, hold for `settleDurationMs` (`SETTLING`).
3. Take `sampleCount` readings spaced `sampleIntervalMs` apart
   (`SAMPLING` / `WAITING_FOR_NEXT_SAMPLE`).
4. Average the samples, store result, return to `WAITING_TO_FLUSH`.

On a read failure → `ERROR_BACKOFF` for `errorBackoffMs`, then return to
`WAITING_TO_FLUSH` and retry. The last successful cached value is preserved.

A cached value has a freshness window (`freshnessThresholdMs`). Stale or missing
values display as `00.00`.

### 6.4 Configuration

| Parameter | Description |
|---|---|
| `warmupDurationMs` | Initial sensor warm-up (nominally 300 000 ms = 5 min) |
| `initRetryMs` | How often to retry `sensor.begin()` on failure (nominally 60 000 ms) |
| `measurementIntervalMs` | Time between measurement cycles |
| `flushDurationMs` | How long to flush with N2 before settling |
| `settleDurationMs` | How long to settle after flush before sampling |
| `sampleIntervalMs` | Time between individual samples within a cycle |
| `sampleCount` | Number of samples to average per cycle |
| `freshnessThresholdMs` | Age beyond which a cached value is considered stale |
| `errorBackoffMs` | Wait after a read failure before retrying |

---

## 7. N2 Controller

Controls the N2 compressor using dual-threshold hysteresis on two independent
pressure signals: low-side N2 pressure and high-side N2 pressure.

### 7.1 Compressor On Condition

Compressor runs **only** when both latches permit:
- **Low-side permit:** low N2 pressure has risen above `lowOnPsi_x100`
- **High-side permit:** high N2 pressure has fallen below `highOnPsi_x10`

### 7.2 Latch Behavior (per side)

Each side is an independent latching comparator with hysteresis:

**Low side:**
- Pressure rises above `lowOnPsi_x100` → latch permits
- Pressure falls below `lowOffPsi_x100` → latch inhibits
- Pressure between the two thresholds → latch holds previous state

**High side:**
- Pressure falls below `highOnPsi_x10` → latch permits
- Pressure rises above `highOffPsi_x10` → latch inhibits
- Pressure between the two thresholds → latch holds previous state

### 7.3 Safe State

On startup and after shutdown: low-side inhibit, high-side permit. Compressor off.

### 7.4 Configuration

| Parameter | Description |
|---|---|
| `lowOffPsi_x100` | Low-side: latch inhibits below this |
| `lowOnPsi_x100` | Low-side: latch permits above this |
| `highOnPsi_x10` | High-side: latch permits below this |
| `highOffPsi_x10` | High-side: latch inhibits above this |

---

## 8. Displays

Both displays are updated on every `loop()` iteration.
Both turn off when the system is disabled via the black switch.

### 8.1 4-Digit 7-Segment Display

Always shows nitrogen purity percentage. Format: `XX.XX` (hundredths of a percent).

During an active measurement cycle (flushing, settling, or sampling), the display
updates live with each new reading as it arrives. This gives real-time visibility
into the measurement in progress.

When no value is available at all (sensor warming up, failed, or no reading yet),
displays `00.00`.

### 8.2 20×4 LCD

Always shows all sensor values simultaneously:

```
AIRSUPPLY  XXX.X PSI
 XX.X  TOWERS   XX.X
 X.XX LO N2 HI  XX.X
 NITROGEN  XX.XX %
```

| Row | Content |
|---|---|
| 0 | Air supply pressure (PSI, one decimal) |
| 1 | Left tower PSI and right tower PSI |
| 2 | Low-side N2 PSI (two decimals) and high-side N2 PSI (one decimal) |
| 3 | Nitrogen purity % (two decimals) |

Zero is displayed for any value not yet available.

---

## 9. TimedStateMachine

Shared infrastructure used by all three controllers. Not a controller itself.

- Tracks current state (as a `uint8_t`).
- Records when the current state was entered (`stateEnteredAtMs`).
- Optionally holds a deadline via `transitionToFor(state, durationMs)`.
- `isExpired()` returns true when `nowMs >= deadlineAtMs`.
- Plain `transitionTo(state)` clears any deadline.
- Clock is injected via `IClock` interface; no direct calls to `millis()`.
- Arithmetic is overflow-safe (unsigned subtraction).

---

## 10. Configuration

One lightly-nested config structure, populated at startup from compile-time
constants. Controllers receive only their own sub-config. Pin assignments are
separate from controller parameters.

### Suggested groupings

- **Hardware:** I2C addresses, pin numbers, ADC resolution
- **Pressure:** sensor scaling constants, full-scale ranges per sensor
- **Tower:** timing and pressure thresholds (Section 5.4)
- **O2:** timing and sample parameters (Section 6.4)
- **N2:** compressor thresholds (Section 7.4)

---

## 11. Testing

The BB (black-box) test suite is the primary executable specification.
Tests run on the host (no Arduino hardware required) using fake clocks,
fake sensors, and fake binary outputs.

Additional BB tests are expected to be written, particularly for:
- Display output formatting
- O2 sensor init-retry behavior
- System enable/disable transitions
- Edge cases identified during real-world testing

WB (white-box) tests that probe internal structure may be added where
BB tests cannot easily reach a specific code path.

---

## 12. What Is Explicitly Out of Scope

These existed in earlier versions and are **not required** in the rewrite:

- Rotary selector switch and associated decode logic
- `IController` base class and virtual dispatch hierarchy
- `StateView` / `ControllerState` inner classes
- `SystemRuntime` / `SystemSnapshot` intermediate structs
- Dual-board profile system and simulator mode
  (was used for debugging without real hardware; no longer needed)
- RTC (DS3231) — not needed; would only have been used for console timestamps
- White-box test infrastructure from previous version (rewrite from scratch
  if WB tests are desired)

---

## 13. Default Values

All pressure thresholds and timing parameters are best-guess starting points.
They are expected to be adjusted after production testing with the real machine.
All must remain runtime-configurable (i.e. in the config struct, not hardcoded).

No parameter should be treated as final until validated on the actual system.

---
*Generated from N2V5 source and BB test suite. Answers incorporated from
design review session. Edit and extend as needed.*
