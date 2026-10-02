# Nitrogen Generator Controller — Requirements (v2.1 DRAFT)

**Status:** DRAFT for iteration — no code written yet. v2.1 folds in the author's review of v2.0.
**Supersedes:** `N2V6/N2V6_Requirements.md` v1.1 (2026-06-10)
**Behavioral reference:** `N2V7/N2V7.ino` (2026-07-13). Where V6 text and V7 code disagree, V7 is
treated as the more recent intent and the difference is called out. Code from V5–V7 that meets a
requirement shall be reused rather than rewritten (GOAL-8).
**Targets:** Arduino UNO R4 **Minima** (production), UNO R4 **WiFi** (home bench), macOS/Ubuntu host (tests)
**Conventions:** "shall" = requirement. IDs (e.g. `PIN-2`) are stable and are referenced by tests.
`[Qn]` = open question (§16). `[HQn]` = question for the hardware team (§17). `[NEW]` = not in V6.
`[CHG]` = changed from V6. Owner action items are in `Owner_TODO.md`.

---

## 1. Goals and constraints

| ID | Requirement |
|---|---|
| GOAL-1 | Production shall run **headless**. Normal operation needs no console, no operator, and no button press. |
| GOAL-2 | The software shall be **debuggable remotely-in-time**: the author cannot easily reach production, so diagnostics (POST, BIST, console, closed-loop log capture, fault display) must make a short site visit sufficient to find a problem. |
| GOAL-3 | Nearly all logic shall be testable on a **host** (macOS and Ubuntu CI) without hardware. A thin Hardware Access Layer (HAL) is the only code that touches Arduino APIs. [CHG] V6 forbade abstraction. |
| GOAL-4 | One code base shall build for UNO R4 Minima and UNO R4 WiFi. The board is detected with the macros the Arduino IDE/CLI defines: `ARDUINO_UNOR4_MINIMA` and `ARDUINO_UNOR4_WIFI`. |
| GOAL-5 | No dynamic allocation. No floating point in control logic (permitted only inside the O2 sensor adapter, because the DFRobot library returns `float`). |
| GOAL-6 | After the end of `setup()`, no blocking waits and no `delay()`. All timing derives from `millis()` with unsigned subtraction (rollover-safe). POST and BIST are the exceptions (§11, §12): they run instead of normal operation. |
| GOAL-7 | The DFRobot Multigas library shall be used **unmodified**. All libraries (DFRobot, any other public library, the board core) shall be the **latest available release** at the time work starts, and the versions used shall be recorded in `BuildInfo` (§10). |
| GOAL-8 | **Reuse over rewrite.** Working code from earlier iterations (V5 controllers and tests, V6/V7 state machines, V7 mini-libraries) shall be used where it meets these requirements, after it has host tests. Earlier iterations worked in parts but never all at once. |
| GOAL-9 | **Diagnostics first.** The first firmware taken to the field shall be the diagnostic-only build (§2, DIAG) that exercises all hardware and **guarantees the pinouts** before any production logic runs. |

## 2. Phases and build configurations

| Phase | Purpose | Console | Logging default |
|---|---|---|---|
| **Debug** (now) | Find software and hardware faults. BIST is the most important feature. | **Guaranteed attached** | Verbose |
| **Production** (later) | Run headless. POST is quick and hands-off. | May or may not be attached | Quiet (faults and state changes only) |

| Build | Board macro | Real devices | Simulated | Purpose |
|---|---|---|---|---|
| `HOST` | none | none | all | macOS/Ubuntu tests; fake HAL |
| `BENCH` | `ARDUINO_UNOR4_WIFI` | LCD, LED | pressures, TBS/TOB, O2, valves, SSR (via console) | home bench |
| `DIAG` | either | whatever is wired | none | **diagnostic-only: HAL + POST + BIST + console, no controllers**; outputs OFF except inside BIST |
| `FIELD` | `ARDUINO_UNOR4_MINIMA` | everything | none (simulation code not compiled in) | production |

| ID | Requirement |
|---|---|
| CFG-1 | Phase and build shall be set by compile-time macros in one file, `BuildConfig.h`. |
| CFG-2 | The `FIELD` and `DIAG` builds shall not contain simulation commands or code paths. |
| CFG-3 | Compiling for any other board shall fail with a clear `#error`. |
| CFG-4 | `DIAG` shall share all sources with `FIELD` (same `src/` tree, same HAL, same `BoardPins.h`), differing only in which top-level logic `loop()` runs. |
| CFG-5 | `Config.h` shall `static_assert` that every `…On`/`…Off` pair has the correct hysteresis ordering. |

## 3. Architecture and HAL

```
 loop():  Hal ──► Inputs ──► Faults/Invariants ──► Controllers ──► OutputDriver ──► Hal
                                   │                                  │
                                   └──────────► Display + Console ◄───┘
```

| ID | Requirement |
|---|---|
| ARC-1 | All hardware access shall go through one HAL interface: millisecond clock, digital read/write/mode, analog read and resolution, I2C probe and transfer (returning success/failure), console byte read/write and attach-state, watchdog kick, reset-cause read. |
| ARC-2 | Three HAL implementations: **Arduino** (real), **Fake** (host tests; scriptable inputs, recorded outputs, controllable clock), **Sim** (bench; real I2C, simulated pins, driven from the console). |
| ARC-3 | Controllers, scaling, invariants, faults, display formatting, console parsing, POST and BIST sequencing shall depend only on the HAL interface, never on Arduino headers. |
| ARC-4 | The HAL shall be as thin as practical: no policy, no timing logic, no state machines. |
| ARC-5 | Device drivers (LCD, LED, O2) shall sit behind small interfaces so each has a fake. |
| ARC-6 | Controllers shall use the V6/V7 **timed state machine** (state, deadline, `setup()`, `update()`, `enable()`, `disable()`). It has worked well; it shall be kept **robust and resilient**: rollover-safe deadline math, no state reachable without a defined transition, every `switch` handles every state, unknown states recover to DISABLED and log. |
| ARC-7 | Every controller state transition shall log one line: timestamp, delta since that controller's previous transition, controller name, old→new (short names), next deadline or `-`. |
| ARC-8 | **Output driver.** All digital outputs shall pass through a single `OutputDriver` that applies the active level from `BoardPins.h` (no code outside it uses `HIGH`/`LOW` for an output) and enforces `OUTPUT_MIN_HOLD_MS` (OUT-1). |

**Display and sensor drivers (Q2 resolved: DIY).** LED (TM1650) and LCD (20×4 PCF8574) drivers shall
be minimal-footprint DIY drivers, starting from the author's V7 mini-libraries, refactored behind an
I2C transport interface and **tested heavily** (§13). The author's `TCP20x4`/`TCP1650` libraries
may be reused for any tested logic that meets these requirements. The DFRobot library is wrapped by an
`O2Sensor` adapter.

| ID | Requirement |
|---|---|
| DRV-1 | Drivers shall report I2C success/failure for every transaction (V7 ignores `endTransmission()` results). A failed transaction shall feed fault F10/F11. |
| DRV-2 | Driver unit tests shall use a fake I2C transport that records bytes, with golden byte sequences for init, clear, set-cursor, write-string, backlight, display on/off, brightness, digit/segment/decimal point writes. |
| DRV-3 | Because the displays need a human to judge, every display write shall also be describable as text: the **display model** (what the LCD's 4×20 characters and the LED's 4 digits *should* show) shall be printable on the console (`display` command; and in BIST steps) so the operator compares the console with the real display and confirms. |

## 4. Pinout — single file

| ID | Requirement |
|---|---|
| PIN-1 | Every pin number, I2C address, I2C bus selection, **direction**, **pull configuration** and **active level** shall be defined in **one header file**, `BoardPins.h` (header-only, `constexpr`; **not** a `.cpp`). No other file shall contain a pin number, I2C address, or `HIGH`/`LOW` meaning for a signal. |
| PIN-2 | `BoardPins.h` shall select the pin set from the board macro (`ARDUINO_UNOR4_MINIMA`, `ARDUINO_UNOR4_WIFI`, or a host fake set) and fail to compile on anything else. |
| PIN-3 | `BoardPins.h` shall hold a single **signal table**; `setup()` shall configure every pin's mode (input, input-pullup, output) **from that table** and drive every output to its safe level. |
| PIN-4 | Pin assignments shall be checked at compile time: no duplicate pins; no signal on an I2C bus pin; every signal has a declared direction and (for binary signals) a declared **active level**. [NEW] |
| PIN-5 | `BoardPins.h` shall contain only wiring facts. Tunable thresholds and timings live in `Config.h` (§7). |
| PIN-6 | Every binary input and output shall have a declared active level (`ACTIVE_HIGH` / `ACTIVE_LOW`) and code shall use `isOn(signal)` / `setOn(signal, bool)` helpers, never raw levels. |
| PIN-7 | `BoardPins.h` shall carry, per signal: name, direction, active level, header label, wiring note, and which boards it has been **verified on** (with date). Initially every entry is "unverified". The author verifies and completes the table before bench testing. |
| PIN-8 | The firmware shall not use `Serial1` (D0/D1 are TBS/TOB). |
| PIN-9 | The R4 core maps D2–D7 and D10–D13 to different MCU port pins on the two boards. Only plain GPIO/ADC is used, so header numbers should behave the same, but this is **unverified on the Minima** and is settled by DIAG on site (GOAL-9). |

Initial contents, from V7 (all **unverified**):

| Signal | Pin | Direction | Active level | Notes |
|---|---|---|---|---|
| TBS (The Black Switch) | D0 | in, pull-up | LOW = ON | maintained |
| TOB (The Other Button) | D1 | in, pull-up | LOW = pressed | momentary |
| LEFT tower valve | D4 | out | HIGH = open | |
| RIGHT tower valve | D7 | out | HIGH = open | |
| O2 flush valve | D11 | out | HIGH = open | |
| SSR (compressor) | D8 | out | HIGH = on | |
| Air supply pressure | A0 | analog in | — | 0–150 PSI |
| Low-pressure N2 | A3 | analog in | — | 0–30 PSI |
| High-pressure N2 | **A5** | analog in | — | 0–150 PSI — **I2C conflict, PIN-10** |
| I2C SDA / SCL | A4 / A5 (core pins 18 / 19) | bus | — | `Wire`; see PIN-11 |
| LED TM1650 | 0x24 | I2C | — | |
| LCD PCF8574 | 0x27 | I2C | — | |
| O2 SEN0465 | 0x74 | I2C | — | SEL dip = 0 |

| ID | Requirement |
|---|---|
| PIN-10 | **[Q1 — top priority]** In the installed R4 core (renesas_uno 1.6.0) `Wire` is bound to core pins 18/19 = **A4/A5** on *both* boards, and V7 reads the high-N2 sensor on **A5**, the I2C SCL line. The sensor shall move to an unused analog pin (A1 or A2) unless a bench test shows A5 coexists with I2C. PIN-4 enforces this at compile time. The author is resolving the pinout before bench work. |
| PIN-11 | The author reports I2C SDA/SCL are on D18/D19 on the WiFi board and on A4/A5 on the Minima. The installed core defines both as 18/19 (= A4/A5). `BoardPins.h` shall record the bus pins per board; the author will confirm the production wiring. |
| PIN-12 | TBS and TOB roles: **TBS** is the system on/off; it determines whether the generator is enabled and producing N2 and affects the displays (details TBD [Q17]). **TOB** is available in Debug and Production as needed: single-step during diagnosis, confirm/acknowledge an event. |

## 5. Inputs and scaling

| ID | Requirement |
|---|---|
| INP-1 | Inputs shall be read once per `loop()` pass into an `InputSnapshot` stamped with `millis()`. |
| INP-2 | **ADC resolution** shall be a single constant `ADC_BITS` in `Config.h`, initially **10** (`0x0A`); `setup()` shall call `analogReadResolution(ADC_BITS)` (via the HAL). All code shall work for `ADC_BITS` = 10, 12 or 14. All raw-value limits (valid window, fault window) shall be **derived** from `ADC_BITS` and the 0.5/4.5 V sensor window, never hardcoded. Host tests shall run at all three. After the system works at 10 bits, the author will decide whether higher resolution helps. |
| INP-3 | Scaling shall map the valid window (0.5–4.5 V of the full-scale range) to 0…full scale, fixed-point: air ×10 (0–1500), N2 low ×100 (0–3000), N2 high ×10 (0–1500). At 10 bits the valid window is raw 102…921. |
| INP-4 | [NEW] **Sensor fault detection.** A raw value outside a fault window around the valid window (proposed: below 0.4 V or above 4.6 V) shall mark that sensor **faulty**, shall not be clamped silently into a valid reading, and shall raise a fault (§9). |
| INP-5 | [NEW] A sensor shall be declared faulty only after N consecutive out-of-window samples (proposed N = 3). |
| INP-6 | TBS and TOB shall be read without debounce (as V6). TBS ON at power-up shall be treated as an OFF→ON transition so the system starts. |
| INP-7 | [NEW] **Sensor consistency.** The N2-low reading shall always be lower than the N2-high reading (compared in common units). If N2-low exceeds N2-high by more than a margin (proposed 1.0 PSI, so that two sensors both near zero do not trip) for a hold time (proposed 5 s), with both sensors valid, fault **F04** shall be raised. |
| INP-8 | The system shall be capable of showing the **raw ADC value, volts and scaled PSI** for each sensor, so they can be compared against the production plant's gauges (the plant has readable gauges for air supply and a few others; which ones is being confirmed by the author). |

## 6. Safety invariants, outputs and reset behavior [NEW]

The author's principle: **the sensor rules are true at all times, so after any reset the sensors, not
remembered state, determine behavior.**

**Invariants** (checked every `loop()` pass, after the controllers and before outputs are committed,
in addition to being implemented inside each controller — defense in depth):

| ID | Condition | Required output state |
|---|---|---|
| INV-1 | TBS off | all four outputs off (L, R, Flush, SSR) |
| INV-2 | air < `airLowOff` or air sensor faulty | LEFT and RIGHT valves closed |
| INV-3 | N2 high > `n2HighOff` or N2-high sensor faulty | LEFT and RIGHT closed **and** SSR off |
| INV-4 | N2 low < `n2LowOff` or N2-low sensor faulty | SSR off |

| ID | Requirement |
|---|---|
| INV-5 | **Safety actions are never delayed.** Turning an output OFF because of an invariant, fault, TBS-off, POST or BIST abort shall happen immediately regardless of `OUTPUT_MIN_HOLD_MS`. |
| INV-6 | If an invariant is violated by a controller's output, the firmware shall force the safe state, log the violation with the controller name, and raise F20. On host this is a test failure (it indicates a bug). |
| INV-7 | On host, property tests shall drive randomized sensor/TBS/clock sequences through all controllers and assert INV-1…INV-4 after every step. |
| OUT-1 | **Minimum hold time.** Every output shall have a minimum time since its last change before a *non-safety* change is made (anti-buzz, short-cycle protection). `OUTPUT_MIN_HOLD_MS` = **1000 ms** until determined (the author suggests 500 ms may suffice [Q6]). The state-machine rules shall guarantee this anyway (tower valves change at most every ~60 s); the OutputDriver enforces it as a backstop and logs when it defers a change. BIST toggling (≥ 500 ms per half-cycle) uses a documented BIST bypass. |

**Reset behavior:**

| ID | Requirement |
|---|---|
| RST-1 | Behavior after a reset shall be a function of sensors and TBS only, not of the reset cause or any stored state. |
| RST-2 | All outputs shall be driven to their safe level as the **first** action in `setup()`, before I2C, display, or POST. |
| RST-3 | At boot every controller starts DISABLED. If TBS is ON, the normal enable path runs on the first loop pass and sensor rules choose the state. |
| RST-4 | Compressor initial state on enable shall be chosen from sensors: N2 high above `n2HighOn` → STOPPED_HIGH; else N2 low below `n2LowOn` → STOPPED_LOW; else RUNNING (subject to OUT-1). [CHG] |
| RST-5 | Reset cause (power-on, external, watchdog, brown-out) shall be read at boot if the core exposes it, and logged. |
| RST-6 | The two pushbuttons are: the board RESET button (hardware; not readable by software) and TOB. TOB held during reset/power-up selects BIST (§12). |
| RST-7 | Running POST or BIST means normal operation is **disabled**: controllers DISABLED, outputs OFF (except those BIST is exercising). |
| RST-8 | **Hardware default.** Output lines should default to OFF while the MCU is in reset or before `pinMode`. The author believes the hardware pulls outputs to OFF; if verification shows otherwise, a pull resistor shall be added [Q14]. |

## 7. Controllers and configuration

Thresholds and timings live in a runtime `Config` struct initialized from `Config.h` constants,
so that a **later version** can change pressure thresholds from the console. In this version only
`cfg` (display) is implemented; `cfg set` is reserved (FUT-1).

| Parameter | Value | Units | Meaning |
|---|---|---|---|
| `ADC_BITS` | 10 | bits | ADC resolution (INP-2) |
| `airLowOff` | 900 | PSI×10 (90.0) | tower disables below |
| `airLowOn` | 1200 | PSI×10 (120.0) | tower may start above |
| `towerFillTime` | 59 250 | ms | fill phase |
| `towerOverlapTime` | 750 | ms | both valves open |
| `n2LowOff` | 1000 | PSI×100 (10.00) | SSR off below |
| `n2LowOn` | 2000 | PSI×100 (20.00) | SSR may start above |
| `n2HighOn` | 1000 | PSI×10 (100.0) | SSR/tower may start below |
| `n2HighOff` | 1200 | PSI×10 (120.0) | SSR/tower stop above |
| `O2SampleInterval` | 60 000 | ms | cycle start to cycle start |
| `O2FlushTime` | 2 000 | ms | |
| `O2SampleTime` | 250 | ms | between samples |
| `O2SampleCount` | 10 | | averaged per cycle |
| `O2CommRetryMs` | 1 000 | ms | retry while UNKNOWN |
| `O2_WARMUP_MS` | TBD | ms | sensor warm-up (O2-6); placeholder until the datasheet value is entered |
| `OUTPUT_MIN_HOLD_MS` | 1 000 | ms | OUT-1 |
| `WDT_TIMEOUT_MS` | 4 000 | ms | WDT-1; revisit after loop-time analysis |

### 7.1 Tower controller
States: `DISABLED, LEFT, LEFT_BOTH, RIGHT, RIGHT_BOTH` (OFF, L, LB, R, RB).

| From | To | Trigger | Action |
|---|---|---|---|
| any ≠ DISABLED | DISABLED | INV-2 or INV-3 condition (every update; bypasses deadline) | close both valves |
| DISABLED | LEFT | TBS on **and** air > `airLowOn` **and** N2 high < `n2HighOn` **and** neither sensor faulty | open LEFT |
| LEFT | LEFT_BOTH | `towerFillTime` | open RIGHT |
| LEFT_BOTH | RIGHT | `towerOverlapTime` | close LEFT |
| RIGHT | RIGHT_BOTH | `towerFillTime` | open LEFT |
| RIGHT_BOTH | LEFT | `towerOverlapTime` | close RIGHT |

`disable()` closes both valves and enters DISABLED. `enable()` leaves it DISABLED; the first row restarts cycling.

### 7.2 Compressor controller
States: `DISABLED, RUNNING, STOPPED_LOW, STOPPED_HIGH` (OFF, ON, LO, HI).
RUNNING → STOPPED_LOW when N2 low < `n2LowOff`; RUNNING → STOPPED_HIGH when N2 high > `n2HighOff`.
STOPPED_LOW → RUNNING when N2 low > `n2LowOn` **and N2 high is not above `n2HighOff`** [CHG];
STOPPED_HIGH → RUNNING when N2 high < `n2HighOn` **and N2 low is not below `n2LowOff`** [CHG].
All non-RUNNING states hold the SSR off. `enable()` selects the state per RST-4.

### 7.3 O2 controller
Monitoring only (Q7: no purity alarm). States: `UNKNOWN, FLUSHING, SAMPLING, WAITING, ERROR, DISABLED` (??, F, S, W, E, OFF).
Cycle as V6/V7: UNKNOWN → (sensor responds) FLUSHING (open flush valve) → after `O2FlushTime` close
valve, first sample → SAMPLING (re-arm per sample, no transition logged) → after `O2SampleCount`
samples store average → WAITING until cycle-start + `O2SampleInterval` → FLUSHING.
N2% ×100 = 10000 − O2% ×100, clamped to 9999 (and 0 if O2 ≥ 100%).

| ID | Requirement |
|---|---|
| O2-1 | The O2 controller shall never inhibit tower or compressor operation. |
| O2-2 | ERROR shall not be permanent: it shall retry via UNKNOWN after a delay (proposed 60 s), showing the fault while it persists. [CHG] |
| O2-3 | A failed read shall be detected from the library's actual failure signaling, not only from an I2C address probe (V7's `sensorPresent()`). The failure modes of the unmodified DFRobot library (absent, wrong address, stuck bus, bad data) shall be characterized at the bench and recorded here. |
| O2-4 | The flush valve shall be closed whenever the O2 controller is not in FLUSHING. |
| O2-5 | N2% shall be marked invalid (`--.--`) until the first complete cycle, after any ERROR, after `disable()`, and **while warming up**. |
| O2-6 | **Warm-up.** The sensor documents a warm-up period. Samples shall not be taken or displayed until the warm-up has elapsed. The warm-up is assumed to run from sensor power-on (thermal settling). A reset (button, watchdog) does not power-cycle the sensor, so if the warm-up had already completed no new warm-up is required — but **if the firmware cannot know that, it shall assume the warm-up is not complete** and wait `O2_WARMUP_MS` from boot. The firmware cannot currently know it (RAM is lost across reset), so every boot waits the full warm-up unless a later requirement adds a trustworthy record. During warm-up the LCD/console show `WARM mm:ss`. |
| O2-7 | **Gating [HQ1].** Whether the O2 cycle runs when the towers are not cycling is a hardware question: the sensor reads the N2 product line downstream of the towers. Until answered, the O2 controller runs whenever TBS is ON (V6/V7 behavior), and N2% shall be flagged **stale** when the tower controller is not cycling. A constant `O2_REQUIRES_TOWER_ACTIVE` shall allow either policy. |

## 8. Displays

| ID | Requirement |
|---|---|
| DSP-1 | Displays render from the output snapshot and fault state only, after controllers update. |
| DSP-2 | Writes to LCD/LED occur only when a rendered field has changed (fixed per-field positions, V6 Layout C). |
| DSP-3 | **LED:** TBS on and N2% valid → `nn.nn`; TBS on and invalid → `--.--`; TBS off → blank. [Q17: how else TBS affects displays] |
| DSP-4 | **LCD normal layout (Layout C):** |

```
....5....0....5....0
AIR 123.4  TWR  LB
N2L 12.34  CMP  ON
N2H 123.4  O2   S
N2% 99.99  LRFS 1001
```

| ID | Requirement |
|---|---|
| DSP-5 | **Fault line.** In Production, with a fault of severity ≥ WARN active, **one LCD line (proposed: row 3) shall be co-opted** to show `Fnn <text>` (≤ 20 chars), alternating with its normal content every 2 s (so N2%/LRFS stay available). With several faults they rotate. The LED shows `Fnn` of the first active fault. This is the fault channel when no console is attached. [Q9] |
| DSP-6 | A display failure (no ACK) shall never stop the control loop; it raises an INFO fault logged to the console. |
| DSP-7 | LCD and LED initialization shall tolerate an absent or slow device and re-initialize when a later probe finds it. |
| DSP-8 | **Startup banner.** At startup the LCD shall show the firmware version and build date for about 1 s (user request); the console prints the full build identity (§10). |

## 9. Faults [NEW]

| Code | Fault | Severity | Effect |
|---|---|---|---|
| F01 | Air sensor out of range | **INHIBIT** | INV-2 |
| F02 | N2-low sensor out of range | **INHIBIT** | INV-4 |
| F03 | N2-high sensor out of range | **INHIBIT** | INV-3 |
| F04 | N2-low reads above N2-high | WARN | log + display [Q18: INHIBIT?] |
| F10 | LCD no ACK | INFO | log only |
| F11 | LED no ACK | INFO | log only |
| F12 | O2 comm failure | WARN | O2 ERROR, N2% invalid |
| F20 | Invariant violation | **INHIBIT** | forced safe state (INV-6); latches until reset |
| F30 | Watchdog reset occurred | WARN | cleared after first normal cycle |
| F31 | Brown-out reset occurred | WARN | same |
| F40 | Console TX dropped output | INFO | counter |

| ID | Requirement |
|---|---|
| FLT-1 | Faults are **self-clearing** when the condition clears for a hold time (proposed 5 s), except F20. |
| FLT-2 | Raise and clear each log one console line with timestamp and code. |
| FLT-3 | TOB pressed during normal run acknowledges and hides WARN/INFO fault display for 60 s. It never clears an INHIBIT condition. Anything beyond a single acknowledgment is done by console command. [Q11] |
| FLT-4 | The fault table is one static table; adding a fault is one row plus one test. |

## 10. Console (USB serial) and closed-loop log capture

Console = USB CDC through the Arduino IDE Serial Monitor (or `arduino-cli monitor`). Line-oriented
text; commands are one line.

| ID | Requirement |
|---|---|
| CON-1 | Baud 115200 (ignored by USB CDC). Newline-terminated, case-insensitive commands; `help` lists the commands of this build. |
| CON-2 | **Output shall never block the loop and shall never inhibit input.** TX is non-blocking; when it cannot keep up, lines are dropped and counted (F40). The console receive path is polled on every pass, independent of TX state, so IDE→Arduino commands always get through. |
| CON-3 | Connecting or disconnecting the Serial Monitor while running **shall not affect operation**, other than enabling logging and commands. This shall be verified on both boards (including whether the board resets on connect; never open the port at 1200 baud, which triggers the bootloader). |
| CON-4 | **Attach detection.** The firmware shall detect whether a console is attached (via the core's `Serial` state/DTR) and expose it as `console.attached()`. BIST requires an attached console; POST does not. |
| CON-5 | Log levels `ERR, WARN, INFO, DEBUG`; default DEBUG in Debug, INFO in Production; `log <level>`. |
| CON-6 | The line reader is a non-blocking accumulator with a bounded line length; over-long lines are discarded with an error. |
| CON-7 | In the `FIELD` build no console command shall drive an output directly. Outputs are exercised only through BIST. [Q12] |

**Build identity (user request):**

| ID | Requirement |
|---|---|
| ID-1 | A `BuildInfo` shall give: firmware version string (manual constant), build date and time (`__DATE__`/`__TIME__`), board, phase/build, ADC_BITS, and library/core versions where obtainable. It is printed at every boot, by `ver`, in `status`, and in the report header; the version/date is also shown on the LCD at startup (DSP-8). A git commit hash is added when a build script provides it. |

**Closed-loop capture:**

| ID | Requirement |
|---|---|
| LOG-1 | The Arduino has no storage. Capture is by the laptop: the console shall be written so that a captured text file is self-contained and machine-readable. Every log line shall start with `ms-since-boot`, a level and a tag (e.g. `123456 I TWR OFF->L next:182706`). |
| LOG-2 | A `report` command shall print one delimited block (`==== N2 REPORT BEGIN ====` … `==== N2 REPORT END ====`) with: build identity, uptime, reset cause, POST results, config, inputs (raw/volts/scaled), controller states, outputs, active faults, loop-time statistics, and the display model. |
| LOG-3 | A host-side capture tool (`tools/n2log`, a small script around `pyserial` or `arduino-cli monitor`) shall save the console to a timestamped file and also forward typed commands. It is outside the firmware. Because only one program can hold the port, capture replaces the IDE's Serial Monitor while in use. [Q19] |
| LOG-4 | The `time <iso>` command (optional) may set a wall-clock reference stored in RAM only; the capture tool also prefixes host wall-clock time, so captured files are datable without it. |

| Command | Builds | Purpose |
|---|---|---|
| `help`, `ver`, `status`, `report` | all | usage; build identity; snapshot; full delimited report |
| `log <level>` | all | log level |
| `faults` | all | active faults |
| `cfg` | all | display config (future: `cfg set`, FUT-1) |
| `display` | all | print what the LCD and LED should currently show |
| `loop` | all | loop-time statistics (min/mean/median/max); `loop reset` |
| `scan` | all | I2C scan |
| `post` | all (system disabled while it runs) | run POST |
| `bist` | all, only if TBS OFF and console attached | run BIST |
| `sim <signal> <value>`, `sim off` | BENCH, HOST | simulated inputs |

## 11. Power-on self-test (POST) [NEW]

POST is **hands-off, quick, and does not require a console**. It runs at every boot (Production) and
disables normal operation while it runs (RST-7).

| ID | Requirement |
|---|---|
| POST-1 | POST shall be quick (proposed ≤ 3 s) and shall not wait for anyone **unless a fault occurs**. |
| POST-2 | Checks: (1) outputs verified safe; (2) reset cause logged; (3) I2C scan against expected addresses (LCD, LED, O2): each OK/MISSING, unexpected responders listed; (4) each pressure sensor within the valid window; (5) TBS/TOB read; (6) config sanity; (7) build identity. |
| POST-3 | Result: one summary line (`POST PASS`, `POST WARN n`, `POST FAIL n`) plus one line per check, to the console if attached, and on the LCD. |
| POST-4 | **POST shall hang if and only if a fault occurs.** On a fault of severity ≥ `POST_HANG_SEVERITY` (proposed WARN; INFO faults do not hang) the system stays disabled with the fault shown on the LCD (and console if attached) until the operator presses **TOB**, which releases it to normal operation. The matching faults stay active, so the INV rules still protect the plant. A constant sets the maximum hold (forever by default). [Q20: does an O2 failure alone hang a headless restart?] |
| POST-5 | A clean POST shall show `N2 vX.Y POST OK` on the LCD for ~1 s and continue. |
| POST-6 | `post` on the console re-runs POST; it shall disable the controllers while it runs and re-enable them after, per RST-3. |

## 12. Built-in self-test (BIST)

Interactive diagnostics **with operator confirmation**, for Debug and for site visits. BIST is the
most important feature during Debug. It **requires an attached console**.

| ID | Requirement |
|---|---|
| BIST-1 | BIST runs **only on request**: TOB held during reset/power-up, or `bist` on the console with TBS OFF. It shall refuse to start without an attached console (LCD shows `BIST: NO CONSOLE`). [CHG] V6/V7 run BIST on every boot and block, which hangs an unattended unit. |
| BIST-2 | BIST disables normal operation (RST-7). Steps are numbered in HEX; the LED shows the step (except in the LED test), the LCD shows the step (except in the LCD test), the console logs every step and observation. |
| BIST-3 | **Operator confirmation.** Each step ends with a prompt on the console: `y` (pass), `n` (fail, optional note), `r` (repeat), `s` (skip), `q` (quit BIST). Pressing **TOB = `y`**. The result is printed as one line per step and a final table, all in the captured log; nothing is stored on the board. |
| BIST-4 | **Display steps echo the expectation to the console** (DRV-3): e.g. `EXPECT LCD  row0 "0123456789ABCDEFGHIJ"` and `EXPECT LED  "8.8.8.8."`, so the operator compares console and display and then confirms. |
| BIST-5 | **Output steps** run only with TBS OFF; each toggles at ~2 Hz for 10 s or until confirmation (the valves are loud enough to confirm audibly) and leaves the output OFF on exit, timeout or any abort (INV-5). Output steps print which signal, which pin, and the active level. |
| BIST-6 | The watchdog (if enabled) is kicked from every BIST wait loop. |
| BIST-7 | After BIST, normal operation resumes without a reset after re-running RST-2/RST-3. |
| BIST-8 | BIST sequencing and formatting shall be host-testable; only HAL calls touch hardware. |
| BIST-9 | The sensor step shall show raw ADC, volts, scaled PSI and the in-window flag, and ask the operator to enter (or confirm) the **plant gauge reading** for the gauges that exist (INP-8). |

| Step | Test | Operator confirms |
|---|---|---|
| 0 | Banner, build identity | start |
| 1 | TBS and TOB | toggle each; log changes |
| 2 | I2C scan | expected vs found table |
| 3 | LED | 0000…9999, decimal points, on/off; console states expectation |
| 4 | LCD | backlight, display on/off, all 80 cells; console states expectation |
| 5 | Pressures | raw / volts / PSI; compare with gauges; optionally disconnect a sensor to see fault values |
| 6 | O2 sensor | begin result, raw library returns, warm-up state |
| 7 | LEFT valve | ~2 Hz; confirm by sound |
| 8 | RIGHT valve | same |
| 9 | Flush valve | same |
| A | SSR | same |
| B | Summary | table of y/n per step |

## 13. Verification

| ID | Requirement |
|---|---|
| VER-1 | Every requirement has a test: host unit/property/scenario, bench, or field. A traceability table is generated from test names (e.g. `TEST_CASE("INV-3: …")`). |
| VER-2 | Host tests run on macOS and in **GitHub Actions on Ubuntu**; the CI also compiles for both boards with `arduino-cli` and reports flash/RAM. |
| VER-3 | Display drivers get exhaustive host tests (DRV-2) **plus** human-in-the-loop checks using the `display` echo (DRV-3). |
| VER-4 | Tests run at `ADC_BITS` = 10, 12, 14 and across `millis()` rollover. |

## 14. Watchdog and timing

| ID | Requirement |
|---|---|
| WDT-1 | Production builds enable the hardware watchdog (the core provides `WDT.h`; R4 maximum ≈ 5.6 s). Initial timeout 4 s, revised after loop-time analysis (NFR-1) [Q13]. |
| WDT-2 | The watchdog is kicked once per `loop()` pass and inside POST-hold and BIST waits. |
| WDT-3 | A watchdog reset is treated like any reset (RST-1) and reported (F30). |
| WDT-4 | Whether `Wire` can hang on a stuck bus shall be tested at the bench; `Wire.setWireTimeout` or an equivalent shall be used; the watchdog is the backstop. |
| WDT-5 | Debug builds allow the watchdog to be disabled by a macro. |
| NFR-1 | **Loop timing.** The firmware shall measure and report loop time: min, mean, **median** and max (median from a fixed-bucket histogram, no dynamic allocation), via `loop` and in `report`. Goal: **max `loop()` < 1 s** (and normally ≪ that); display writes may be spread across passes to meet it. The watchdog timeout is chosen from this data. |
| NFR-2 | Flash and RAM shall be reported at every build for both boards; initial budget **60 %** of each, raised if necessary. |
| NFR-3 | The current production sketch shall be kept as a known-good fallback for the field trip. |
| FUT-1 | **Reserved for a later version:** changing pressure thresholds from the console (`cfg set`). The runtime `Config` struct (§7) shall make this a small change. |

## 15. Owner-supplied facts still to confirm
See `Owner_TODO.md`. Highlights: complete pinout with active levels; which plant gauges exist; I2C
pins on production; SEN0465 warm-up time; output pull-down wiring.

## 16. Open questions

| # | Question | Proposal / status |
|---|---|---|
| Q1 | High-N2 sensor on A5 = SCL | Author is resolving the pinout; move to A1/A2 |
| Q2 | LCD/LED libs | **Resolved:** DIY minimal, from V7 mini-libs; reuse TCP logic where good |
| Q3 | Fixed-point ranges as V7 | Resolved: yes |
| Q4 | Sensor fault window/sample count | Resolved: ok (0.4/4.6 V, 3 samples) |
| Q5 | RST-4 and §7.2 cross-checks | Resolved: ok |
| Q6 | Minimum output hold time | 1000 ms until determined (500 ms proposed by author); BIST bypass for 2 Hz — confirm |
| Q7 | O2 purity alarm | Resolved: none |
| Q8 | O2 auto-retry | Resolved: yes |
| Q9 | Fault display | Resolved: ok, as DSP-5 (one co-opted LCD line) |
| Q10 | Display re-init | Resolved: yes |
| Q11 | TOB = acknowledge | Resolved: ok |
| Q12 | Console BIST only with TBS off; no direct outputs in FIELD | Resolved: ok |
| Q13 | Watchdog 4 s | Resolved: ok, revise after loop analysis |
| Q14 | Output pull-downs | Author believes yes; **verify**; else add |
| Q15 | Independent safety device | Reworded in HQ2 |
| Q16 | (reserved) | |
| Q17 | How does TBS affect LCD/LED beyond the LED blanking? | TBD |
| Q18 | Should F04 (N2 low > N2 high) inhibit outputs, or warn only? | WARN until field data |
| Q19 | Capture tool: Python `pyserial` script vs `arduino-cli monitor | tee` | Propose pyserial script (adds timestamps, forwards commands) |
| Q20 | Should a missing O2 sensor alone hold POST until TOB? | Propose: only INHIBIT-severity faults hang a headless unit |

## 17. Questions for the hardware team

| # | Question |
|---|---|
| HQ1 | The O2 sensor reads the N2 product line downstream of the towers. Can the O2 sample cycle (flush, then 10 samples) run meaningfully when the towers are **not** cycling (TBS on but air low, tank full, or compressor stopped)? Is there flow, or is the sensor then reading stale or air-contaminated gas? |
| HQ2 | Is there a **mechanical or hardware safeguard independent of the Arduino** — a pressure relief valve, a hardware pressure switch on the tank or compressor, or a stop that cuts power? (This tells us how much the firmware alone must guarantee; if none exists, the firmware's fail-safe behavior is the only protection.) |
| HQ3 | Do the valve/SSR driver circuits default to OFF when the Arduino pin is floating or in reset? |
| HQ4 | Which plant gauges are readable (air supply, and which others — left/right tower or N2 low/high)? |
| HQ5 | What is the SEN0465 warm-up time, and is the sensor powered from the same supply as the Arduino or separately? |

---

## Appendix A — Differences from V6

| Area | V6 v1.1 | This draft |
|---|---|---|
| Architecture | Monolithic, no abstraction, pasted libraries | Thin HAL, fakes, host tests, drivers behind interfaces |
| Pinouts | `const` block in the sketch | One header, board-selected, with direction/pull/active level, compile-time checked, set up from a table |
| ADC | 10-bit hardcoded | `ADC_BITS` 10/12/14, limits derived |
| BIST | Every boot, blocks | On request, console required, operator confirms y/n, display echo |
| POST | none | New, hands-off, hangs only on fault; TOB releases |
| Faults | O2 ERROR only | Fault table, co-opted LCD line |
| Invariants/reset | implicit | Explicit, tested; sensors decide after reset |
| Outputs | direct writes | OutputDriver with active levels and min hold time |
| O2 | no warm-up | Warm-up and staleness rules |
| Console | log only | Bidirectional, non-blocking both ways, report block, capture tool |
| Diagnostics-first | none | DIAG build guarantees pinouts before logic |

## Appendix B — Findings in `N2V7.ino` that drive the new requirements

1. **BIST blocks every boot** (`const bool runBIST = true;`, `waitForTob()`): after any power loss the unit hangs at "press TOB" with outputs off. → BIST-1.
2. **A5 conflict**: `HIGH_N2_PIN = A5` and the I2C bus uses A5 as SCL. → PIN-10.
3. **Failed sensors are invisible**: `scalePressure()` clamps, so an unplugged transducer reads 0 PSI. A dead **N2-high** sensor looks like an *empty* tank and the over-pressure stop never fires. The "plausibility warning" in `loop()` can never trigger because the value is clamped before it is compared with 1.1 × full scale. → INP-4, INP-5, INV-3.
4. **One-pass SSR pulse**: `Compressor::enable()` enters STOPPED_LOW; the next `update()` turns the SSR on when N2-low > `n2LowOn` without checking N2-high. → RST-4, §7.2.
5. **O2 failure detection is an address probe**; bad data is not detected. → O2-3.
6. **O2 ERROR is permanent** until a TBS toggle. → O2-2.
7. **No O2 warm-up** handling. → O2-6.
8. `inputs.tob` is read but unused; display drivers ignore I2C errors; `Serial.print` use assumes a host. → DRV-1, CON-2.
