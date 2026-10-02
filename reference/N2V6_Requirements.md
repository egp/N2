# Nitrogen Generator Controller — Software Requirements

**Document status:** v1.1 — tracks the first-pass sketch
**Target platform:** Arduino UNO R4 Minima **or** UNO R4 WiFi (single sketch runs on both)
**Date:** 2026-06-10

---

## 1. Design Goals

1. **Minimize complexity.** Minimize lines of code (comments excluded). Minimize layers
   of procedure calls. No abstract interfaces; outputs are driven with direct
   `digitalWrite()` calls or one-line helpers at most.
2. **Monolithic sketch.** One `.ino` file, no local tabs/headers. The LED and LCD
   mini-libraries are **pasted into the sketch**, not `#include`d. The only `#include`s
   are `Arduino.h` (implicit), `Wire.h`, and the DFRobot Multigas library.
3. **Non-blocking after startup.** `delay()` and blocking waits are permitted only inside
   `setup()` (which includes the built-in self-test). From the first entry into `loop()`
   onward, no `delay()` anywhere; all timing derives from `millis()`.
4. Structs and plain functions over class hierarchies. No dynamic allocation.

## 2. System Summary

The sketch controls a PSA nitrogen generator. Three controllers — **Tower**,
**Compressor**, **O2** — each follow a shared timed-state-machine pattern. Transitions
are driven by deadlines (time) and by input changes. Inputs are read once per loop into
an **input snapshot**; controller results are recorded in an **output snapshot**;
displays render from the output snapshot only.

Every controller implements four methods:

| Method | Called | Purpose |
|---|---|---|
| `setup()` | once, from main `setup()` | initialize |
| `update()` | once per `loop()` pass | run the state machine |
| `enable()` | on TBS OFF → ON | return controller to its initial state (Tower → DISABLED, Compressor → DISABLED-exit per rules, O2 → UNKNOWN); normal transition rules then run as usual |
| `disable()` | on TBS ON → OFF | enter DISABLED; close all outputs the controller owns |

## 3. Hardware Configuration Constants

```cpp
/* ── Digital inputs ─────────────────────────────────────────────────────── */
const uint8_t TBS_PIN               =  0;   // INPUT_PULLUP, ON = LOW (maintained)
const uint8_t TOB_PIN               =  1;   // INPUT_PULLUP, ON = LOW (momentary)

/* ── Digital outputs (LOW == OFF, HIGH == ON for all) ───────────────────── */
const uint8_t LEFT_TOWER_VALVE_PIN  =  4;   // HIGH = open, LOW = closed
const uint8_t RIGHT_TOWER_VALVE_PIN =  7;   // HIGH = open, LOW = closed
const uint8_t O2_FLUSH_VALVE_PIN    = 11;   // HIGH = open, LOW = closed
const uint8_t SSR_PIN               =  8;   // HIGH = compressor on

/* ── Analog inputs (pressure sensors) ───────────────────────────────────── */
const uint8_t SUPPLY_PRESSURE_PIN   = A0;   // 0–150 PSI air supply
const uint8_t LOW_N2_PIN            = A3;   // 0–30  PSI low-pressure N2
const uint8_t HIGH_N2_PIN           = A5;   // 0–150 PSI high-pressure N2

/* ── I2C addresses (all devices share the hardware Wire bus) ────────────── */
const uint8_t I2C_ADDR_DISP4        = 0x24; // TM1650 4-digit display
const uint8_t I2C_ADDR_LCD          = 0x27; // 20x4 LCD with PCF8574
const uint8_t I2C_ADDR_O2           = 0x74; // SEN0465 default (SEL dip = 0)
```

Pin and address values above are believed accurate; any corrections discovered at the
bench (the BIST I2C scan will confirm addresses) are trivial one-line config changes.

### 3.1 Inputs

- **TBS — "The Black Switch"** (D0): system enable, maintained, no debounce.
- **TOB — "The Other Button"** (D1): N/O momentary "continue/approve"; single-steps the
  BIST. No debounce. During `setup()` only, code may spin-wait on TOB.

### 3.2 Analog inputs (pressure sensors)

| Sensor | Range | Stored as | Display format | Pin |
|---|---|---|---|---|
| airSupply | 0–150 PSI | integer ×10 | `nnn.n` | A0 |
| N2Low | 0–30 PSI | integer ×100 | `nn.nn` | A3 |
| N2High | 0–150 PSI | integer ×10 | `nnn.n` | A5 |

All transducers are 0.5–4.5 V ratiometric. Scaling uses the built-in `map()` from the
0.5–4.5 V ADC window to full-scale 0–max PSI in the sensor's fixed-point representation
(**existing `map()` code to be provided and reused**). **ADC resolution: 10-bit,
hardcoded.** No floating point in the control path.

### 3.3 I2C devices (three)

| Device | Function | Address |
|---|---|---|
| DFRobot SEN0465 | O2 (multigas) sensor | 0x74 (SEL dip switch = 0) |
| TM1650-compatible LED | 4 digits, 8 segments incl. decimal point; HEX-capable | 0x24 |
| 20×4 character LCD (PCF8574 backpack) | status display | 0x27 |

### 3.4 Libraries and includes

- `#include`: `Wire.h`, DFRobot Multigas library (`Arduino.h` implicit).
- **LED and LCD mini-libraries (author's own) are pasted into the monolithic sketch** —
  no `#include`, no local tabs. BIST and production code drive the displays through the
  same mini-library code.

## 4. Program Structure

### 4.1 `setup()`

1. Configure pins, serial console at **115200 baud**, I2C.
2. Call each controller's `setup()`.
3. Run the **built-in self-test** (§9) — exactly once per powerup, regardless of TBS,
   before `loop()` is entered. `delay()` and TOB spin-waits permitted here only.

### 4.2 `loop()` — every pass

1. **Update input snapshot** — airSupply, N2Low, N2High pressures; TBS; TOB. Snapshot
   carries a `millis()` timestamp of last update.
2. **Call each controller's `update()`** — Tower, Compressor, O2.
3. **Output snapshot** — updated by the controllers; carries a `millis()` timestamp;
   holds all four output states (L, R, Flush, SSR) and the N2% result.
4. **Update displays** — LED and LCD render from the output snapshot. Updates are not
   rate-limited; display code writes to the LCD/LED **only when a rendered value has
   changed** (Layout C's fixed field positions make this a per-field row/column write).

### 4.3 TBS semantics

- **ON → OFF:** each controller's `disable()` is called → DISABLED, all owned outputs
  closed.
- **OFF → ON:** each controller's `enable()` is called; controllers restart their normal
  state-transition rules from their initial states.
- **Every TBS transition is logged to the console**, in addition to controller state
  transitions.

## 5. Shared Timed State Machine

- Each controller holds a current state and a **deadline** (`millis()` value) for the
  next timed transition.
- Input-driven transitions may **bypass a pending deadline** (e.g., Tower → DISABLED).
- **Every state transition logs one console line**: current timestamp; delta since that
  same controller's previous transition message; controller name (static table indexed
  by controller ID — TWR, CMP, O2); old state → new state using the **short state-name
  abbreviations** (§7.2 — the only state-name table); deadline for the next transition
  (`next:-` when none is pending).
- A state may **re-arm its own deadline without changing state** (used by O2 SAMPLING
  for per-sample timing); re-arming is not a transition and is not logged.
- Deadline comparisons use unsigned subtraction so `millis()` rollover is safe.

## 6. Controllers

### 6.1 Tower controller

**Inputs:** `airSupplyPSI`, `N2HighPSI`. **Outputs:** LEFT valve (D4), RIGHT valve (D7).

**Config parameters:**

| Parameter | Meaning | Units |
|---|---|---|
| `airLowOff` | disable when air supply falls below | PSI ×10 |
| `airLowOn` | permit DISABLED-exit only above this | PSI ×10 |
| `towerFillTime` | fill phase duration | ms |
| `towerOverlapTime` | overlap phase duration | ms |

The tower controller also observes the compressor's N2High thresholds **with
hysteresis**: it disables above `n2HighOff` and may restart only once `N2HighPSI` is
back within acceptable range (below `n2HighOn`).

**States:** `DISABLED`, `LEFT`, `LEFT_BOTH`, `RIGHT`, `RIGHT_BOTH`

**Transitions:**

| From | To | Trigger | Action |
|---|---|---|---|
| any non-DISABLED | DISABLED | `airSupplyPSI < airLowOff` **OR** `N2HighPSI > n2HighOff` — checked every `update()` tick; bypasses any pending deadline | close both valves |
| DISABLED | LEFT | `airSupplyPSI > airLowOn` **AND** `N2HighPSI < n2HighOn` | open LEFT valve |
| LEFT | LEFT_BOTH | after `towerFillTime` | open RIGHT valve |
| LEFT_BOTH | RIGHT | after `towerOverlapTime` | close LEFT valve |
| RIGHT | RIGHT_BOTH | after `towerFillTime` | open LEFT valve |
| RIGHT_BOTH | LEFT | after `towerOverlapTime` | close RIGHT valve |

`DISABLED` ensures both valves are closed. `enable()` places the controller in DISABLED;
the DISABLED → LEFT rule then restarts cycling as usual when conditions permit.

### 6.2 N2 compressor controller

**Inputs:** `N2LowPSI`, `N2HighPSI`. **Output:** compressor SSR (D8).

**Config parameters:**

| Parameter | Applies to | Meaning | Units |
|---|---|---|---|
| `n2LowOff` | N2LowPSI | stop SSR when N2Low falls below | PSI ×100 |
| `n2LowOn` | N2LowPSI | allow restart once N2Low recovers above | PSI ×100 |
| `n2HighOn` | N2HighPSI | allow restart once N2High falls below | PSI ×10 |
| `n2HighOff` | N2HighPSI | stop SSR when N2High exceeds | PSI ×10 |

**States:** `DISABLED`, `RUNNING`, `STOPPED_LOW`, `STOPPED_HIGH`

**Rule:** `RUNNING` drives the SSR on. RUNNING → `STOPPED_LOW` when
`N2LowPSI < n2LowOff` (protect supply); RUNNING → `STOPPED_HIGH` when
`N2HighPSI > n2HighOff` (tank full). STOPPED_LOW → RUNNING once `N2LowPSI > n2LowOn`;
STOPPED_HIGH → RUNNING once `N2HighPSI < n2HighOn`. All STOPPED/DISABLED states hold
the SSR off. `disable()` forces SSR off → DISABLED.

### 6.3 O2 controller

**Behavior (plain language):** when enabled, the controller runs a sample-cycle every
**60 seconds**: it flushes the sensor line, then takes **10 samples 250 ms apart**,
averages them, and stores the average in the output snapshot.

**Output:** flush valve (D11); N2% (integer ×100) in the output snapshot.

**States:** `UNKNOWN`, `FLUSHING`, `SAMPLING`, `WAITING`, `ERROR`, `DISABLED`

| From | To | Trigger | Action |
|---|---|---|---|
| UNKNOWN | FLUSHING | comm established with SEN0465 | open flush valve; record cycle-start; first cycle begins immediately |
| FLUSHING | SAMPLING | after `O2FlushTime` | close flush valve; take first sample; arm deadline +`O2SampleTime` |
| SAMPLING | SAMPLING (re-arm, no transition) | per-sample deadline reached, count < `O2SampleCount` | take next sample; re-arm deadline +`O2SampleTime` |
| SAMPLING | WAITING | `O2SampleCount` samples taken | store sample average in output snapshot; deadline = cycle-start + `O2SampleInterval` |
| WAITING | FLUSHING | deadline reached | open flush valve; record new cycle-start |
| any | ERROR | sensor communication failure | log failure to console; close flush valve |
| any | DISABLED | `disable()` (TBS off) | close flush valve |

The 10-sample sequence is fully non-blocking: SAMPLING uses the state machine's deadline
re-armed per sample with an internal sample counter — no `delay()`, and no extra state
required. Per-sample re-arms are not logged (only the FLUSHING→SAMPLING and
SAMPLING→WAITING transitions are).

`ERROR` persists until `enable()` (TBS off→on) returns the controller to UNKNOWN, which
re-verifies communication. The 60 s period is measured cycle-start to cycle-start, so
the recalculation cadence is exactly `O2SampleInterval`.

**Config parameters:**

| Parameter | Value | Meaning |
|---|---|---|
| `O2SampleInterval` | 60000 ms | period between sample-cycle starts |
| `O2FlushTime` | 2000 ms | flush valve open duration |
| `O2SampleTime` | 250 ms | spacing between consecutive samples |
| `O2SampleCount` | 10 | samples averaged per cycle |

**N2% arithmetic:** O2 reading and N2% are both integers ×100.
`N2pct_x100 = 10000 − O2pct_x100`, **clamped to 9999** — even if O2% reads zero, the
displayed N2% ceiling is `99.99`.

## 7. Displays

Both displays render from the **output snapshot** only, after all controllers have
updated.

### 7.1 LED (4-digit TM1650)

- Enabled: shows N2% as `nn.nn` (hundredths) using the decimal-point segment.
- Disabled (TBS off): **blank**.
- During BIST: shows the BIST step number in HEX (except during the LED test itself).

### 7.2 LCD (20×4) — Layout C (chosen)

**State name abbreviations:**

| Controller | Full state | LCD |
|---|---|---|
| Tower | DISABLED / LEFT / LEFT_BOTH / RIGHT / RIGHT_BOTH | OFF / L / LB / R / RB |
| Compressor | DISABLED / RUNNING / STOPPED_LOW / STOPPED_HIGH | OFF / ON / LO / HI |
| O2 | UNKNOWN / FLUSHING / SAMPLING / WAITING / ERROR / DISABLED | ?? / F / S / W / E / OFF |

**Layout (two-column grid; outputs as a 4-bit map, `1` = ON/open, `0` = OFF/closed):**

```
....5....0....5....0
AIR 123.4  TWR  LB
N2L 12.34  CMP  ON
N2H 123.4  O2   S
N2% 99.99  LRFS 1001
```

`LRFS 1001` = Left, Right, Flush, SSR states left to right. Every field has a fixed
row/column home, so the changed-value-only write policy (§4.2) is a per-field write.

## 8. Operator Controls

- **TBS** (D0, maintained): system enable per §4.3. No debounce.
- **TOB** (D1, momentary): "continue" wherever operator approval is required;
  single-steps the BIST. No debounce.

## 9. Built-in Self-Test (BIST)

- Runs **once** per powerup, inside `setup()`, regardless of TBS position, before
  `loop()` runs.
- **TOB single-steps** the sequence; each step loops/repeats until TOB is pressed
  (spin-wait permitted).
- **Pass/fail is judged by the operator**; the program records nothing.
- Step numbers are sequential HEX. The LED shows the current step number in HEX (except
  during the LED test); the LCD shows the step number (except during the LCD test). Both
  displays are driven through the same mini-library code used in production.
- Order rationale: TBS/TOB first; **I2C scan before the LED and LCD tests** (both need
  I2C); inputs grouped together, then outputs grouped together.

| Step | Test | Behavior |
|---|---|---|
| 0 | TOB / banner | console banner; wait for TOB |
| 1 | TBS | console and LCD show TBS state, logged only on change, as the operator toggles it; until TOB |
| 2 | I2C scan | scan every valid address **once**; log **responsive addresses only**; wait for TOB |
| 3 | LED | cycle `0000, 1111, … 9999`; walk the decimal point through each digit; display on/off; ~5 s per pass, repeats until TOB |
| 4 | LCD | backlight on/off; display on/off; write every cell of the 20×4; ~5 s per pass, repeats until TOB |
| 5 | PSI inputs | display all three pressures on LCD and console **once, then only when a value changes**; until TOB |
| 6 | O2 sensor | display the first reading, **then only when the value changes**; until TOB |
| 7 | LEFT valve | toggle the output at **~2 Hz** (audible open/close); log and show each change; until TOB |
| 8 | RIGHT valve | same |
| 9 | Flush valve | same |
| A | SSR | same |

After step A, `setup()` returns and `loop()` begins.

## 10. Configuration Parameters (consolidated)

All config as `const`/`constexpr` values at the top of the sketch: §3 constants (pins,
I2C addresses); tower params (`airLowOff`, `airLowOn`, `towerFillTime`,
`towerOverlapTime`); compressor params (`n2LowOff`, `n2LowOn`, `n2HighOn`, `n2HighOff`);
O2 params (`O2SampleInterval`, `O2FlushTime`, `O2SampleTime`, `O2SampleCount`); console
baud **115200**.

## 11. Materials Required Before Implementation

To be provided by the author and incorporated verbatim/adapted:

1. The existing **`map()` pressure-scaling code** (0.5–4.5 V → fixed-point PSI).
2. The **LED mini-library** (TM1650, HEX-capable) — pasted into the sketch.
3. The **LCD mini-library** (20×4 / PCF8574) — pasted into the sketch.
