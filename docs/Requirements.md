# Nitrogen Generator Controller — Requirements (v3.0 DRAFT)

**Status:** DRAFT for iteration (v3.0, 2026-10-08). Revised after the bring-up of the LCD, RTC, LED, switches and reset (R4 WiFi bench) and the owner's review of 2026-10-08 (`Requirements_Review_20261008.md` lists every change and its reason). Findings made while building are marked **[found]**.
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
| GOAL-1 | Production **may run headless or with a console attached [owner 2026-10-08]**. Normal operation needs no console, no operator, and no button press; the firmware never depends on a console. **If a console is attached, it receives every state transition** (controllers, TBS, faults, mode changes) at INFO level; with none attached, nothing waits for it (output is dropped, F40 if the host stops reading). |
| GOAL-2 | The software shall be **debuggable remotely-in-time**: the author cannot easily reach production, so diagnostics (POST, BIST, console, closed-loop log capture, fault display) must make a short site visit sufficient to find a problem. |
| GOAL-3 | Nearly all logic shall be testable on a **host** (macOS and Ubuntu CI) without hardware. A thin Hardware Access Layer (HAL) is the only code that touches Arduino APIs. [CHG] V6 forbade abstraction. |
| GOAL-4 | One code base shall build for UNO R4 Minima and UNO R4 WiFi. The board is detected with the macros the Arduino IDE/CLI defines: `ARDUINO_UNOR4_MINIMA` and `ARDUINO_UNOR4_WIFI`. |
| GOAL-5 | No dynamic allocation. No floating point in control logic (permitted only inside the O2 sensor adapter, because the DFRobot library returns `float`). |
| GOAL-6 | After the end of `setup()`, no blocking waits and **no `delay()`, anywhere** (POST and BIST included). All timing derives from `millis()` with unsigned subtraction (rollover-safe); the wall clock (RTC) is for log stamps only and is never used for scheduling (RTC-7). The only documented exception is the DFRobot library read (about 10 ms) when `N2_O2_DRIVER=LIBRARY` is selected (O2-3b). [CHG 2026-10-08: POST and BIST are no longer exceptions; they are non-blocking state machines.] |
| GOAL-7 | The DFRobot Multigas library, **when it is used**, shall be used **unmodified**. Library and core versions shall be the latest available release at the time work starts and shall be recorded in `BuildInfo` (§10). The O2 sensor may instead be read by this project's own read-only driver (O2-3b); the build macro `N2_O2_DRIVER` selects `OWN` or `LIBRARY`, so both can be compared on the same hardware. [CHG 2026-10-08] |
| GOAL-8 | **Reuse over rewrite.** Working code from earlier iterations (V5 controllers and tests, V6/V7 state machines, V7 mini-libraries) shall be used where it meets these requirements, after it has host tests. Earlier iterations worked in parts but never all at once. |
| GOAL-9 | **Diagnostics first.** The first firmware taken to the field shall be the diagnostic-only build (§2, DIAG) that exercises all hardware and **guarantees the pinouts** before any production logic runs. |
| GOAL-10 | **Design priorities, in this order: testability, readability, maintainability.** Where requirements or designs conflict, this order decides. Concretely: small single-purpose modules; pure functions where possible; names that say what a thing is; one place for each fact (pins, constants, text); no clever code; every behavior reachable from a host test. |
| GOAL-12 | **Follow V6/V7 where they agree.** Where V6 and V7 agree, V8 does the same unless a requirement here says otherwise; every deviation is recorded in Appendix A/B. Examples the owner values: the **non-blocking timed state machine with unsigned-subtraction deadlines** (ARC-6) and the **watchdog refreshed once per `loop()` with a 4-second timeout, reduced once real loop times are known** (WDT-1/2, NFR-1). |
| GOAL-11 | **The firmware is the only protection.** The system has no independent hardware safeguard against over-pressure or similar faults (hardware-team answer HQ2: none). The invariants (§6) are therefore safety-critical: they shall be independently reviewed before the first hardware visit and again before production use. |

## 2. Phases and build configurations

| Phase | Purpose | Console | Logging default |
|---|---|---|---|
| **Debug** (now) | Find software and hardware faults. BIST is the most important feature. | **Guaranteed attached** | Verbose |
| **Production** (later) | Run headless. A normal boot runs no self-test; POST mode (TOB at power-up) is quick. | May or may not be attached | Quiet: INFO level = every state transition and fault, nothing chattier; DEBUG on request |

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
| CFG-3 | **Run-time mode [owner 2026-10-09].** `mode` shows DIAG / BENCH / FIELD; `mode diag` is immediate and turns every output off; `mode bench confirm` and `mode field confirm` enable the controllers and are accepted only from the console, only with TBS OFF and only with the word `confirm` (nothing starts until TBS is switched ON). FIELD makes the O2 sensor mandatory (INV-9), sets the quiet log and hides the version on the LCD. The compiled build sets the mode after a power-up (DIAG for anything uploaded for testing); a checksummed RAM record (`ModeRecord`) keeps the mode across a reset-button or watchdog reset, never across a power loss or brown-out, and never in NVM. The record also carries the build identity, so a freshly UPLOADED build ignores it and starts in its compiled mode. Every change is logged at INFO. |
| CFG-3 | Compiling for any other board shall fail with a clear `#error`. |
| CFG-4 | `DIAG` shall share all sources with `FIELD` (same `src/` tree, same HAL, same `BoardPins.h`), differing only in which top-level logic `loop()` runs. |
| CFG-5 | `Config.h` shall `static_assert` that every `…On`/`…Off` pair has the correct hysteresis ordering. |

### 2.1 Staged bring-up [NEW 2026-10-08]

| ID | Requirement |
|---|---|
| STG-1 | The firmware is brought up in **stages** (`Bringup_Stages.md`): each stage adds devices to a base that already works on the bench, with the same host-tested source tree. Stage 1: reset cause, LCD, RTC, console, WiFi matrix. Stage 2: + LED. Stage 3: TBS/TOB and the pressure inputs. Stage 4: O2 sensor. Stage 5: valves, SSR, controllers. |
| STG-2 | A device enters a stage in this order: a solo experiment sketch on the bench, its `DeviceCheck` (CHK-1) with host tests, the stage sketch, a results file in `docs/results/`. |

## 3. Architecture and HAL

```
 loop():  Hal ─► fast reads (GPIO, ADC) ─┐
          I2C read steps (O2, RTC) ───────┴─► InputSnapshot (const during the pass, stamped)
                                                   │
                    previous pass's OutputSnapshot ┤ (cross-controller facts, CTL-2)
                                                   ▼
                                  Controllers (update) ─► Invariants ─► OutputDriver ─► Hal
                                                   │
                                                   ▼
                       OutputSnapshot (stamped) ─► Displays, Console (status/report/log)
```

| ID | Requirement |
|---|---|
| ARC-1 | All hardware access shall go through one HAL interface: millisecond and microsecond clock and a microsecond busy-wait (bit-banged protocols only), digital read/write/mode, analog read and resolution, I2C probe, write, read and register read (each returning success/failure), I2C clock selection and **bus recovery** (I2C-2), console begin/attach-state/byte read/write, watchdog begin/kick, reset-cause read. |
| ARC-2 | Two HAL implementations exist: **Arduino** (real) and **Fake** (host tests; scriptable inputs, a register-file model of I2C devices, recorded outputs, controllable clock, and a pin-event model for bit-banged buses). A third, **Sim** (bench with simulated pins driven from the console), was planned, is **not built**, and the `sim` console command has been removed (owner, 2026-10-08). [CHG 2026-10-08] |
| ARC-3 | Controllers, scaling, invariants, faults, display formatting, console parsing, POST and BIST sequencing shall depend only on the HAL interface, never on Arduino headers. |
| ARC-4 | The HAL shall be as thin as practical: no policy, no timing logic, no state machines. |
| ARC-5 | Device drivers (LCD, LED, O2) shall sit behind small interfaces so each has a fake. |
| ARC-6 | Controllers use the V6/V7 **timed state machine** (state, deadline, `enable()`, `disable()`, `update()`), kept **robust and resilient**: rollover-safe deadline math, no state reachable without a defined transition, every `switch` handles every state, unknown states recover to DISABLED and log. The common interface is CTL-1. |
| ARC-7 | Every controller state transition shall be logged by the base class (CTL-3) as one line in the standard format (LOG-1) whose text is `<NAME> <from>-><to> +<delta> next:<deadline|->`: delta = ms since that controller's previous transition, next = the deadline in `millis()` or `-`. |
| ARC-8 | **Output driver.** All digital outputs shall pass through a single `OutputDriver` that applies the active level from `BoardPins.h` (no code outside it uses `HIGH`/`LOW` for an output) and enforces `OUTPUT_MIN_HOLD_MS` (OUT-1). |

### 3.1 The controller interface [NEW 2026-10-08]

| ID | Requirement |
|---|---|
| CTL-1 | The **Tower**, **Compressor** and **O2** controllers shall each implement one abstract base class `Controller` (one level of inheritance, no templates): `name()`, `enable(now)`, `disable(now)`, `update(const InputSnapshot&)`, `stateName()`, `enabled()`, `nextDeadlineMs()`. `System` holds the three as `Controller*` and drives them through this interface only. |
| CTL-2 | A controller reads **only** the `InputSnapshot` passed to `update()`. It never reads another controller, the HAL or a driver. Facts one controller needs from another (for example *the O2 sensor is warm and answering*) are published in the **previous pass's** `OutputSnapshot` and copied into the `InputSnapshot` at the start of the pass: one pass of latency, no dependence on update order. The `InputSnapshot` is `const` during a pass. |
| CTL-3 | The base class owns `transition(to, now, ...)`; every state change of every controller is logged by it with the same line format (ARC-7). A controller cannot change state without logging. |
| CTL-4 | `update()` never blocks and never calls `delay()`. All waiting is a deadline compared with unsigned subtraction. `nextDeadlineMs()` exposes it for the log and for host tests. |
| CTL-5 | A controller commits no output itself: it exposes what it wants (`OutputRequest`); the invariants (§6) and the `OutputDriver` decide what is driven. |

### 3.2 Snapshots [NEW 2026-10-08]

| ID | Requirement |
|---|---|
| SNP-1 | `InputSnapshot`: `ms` (the `millis()` of the pass), a pass counter `seq`, TBS, TOB, the three pressures (raw counts, scaled values, per-sensor ok flags and the sensor-order fault), and for each I2C device that is read by a background step (INP-9) its **latest value and its age in ms** (O2 %, RTC time). Cross-controller facts from CTL-2 are copied in at the start of the pass. |
| SNP-2 | `OutputSnapshot`: `ms`, `seq`, each controller's state name, the outputs **requested**, the outputs **actually driven**, the invariants mask, the active faults and the **last fault code** (FLT-5), N2 % and its validity, warm-up time remaining. Built once per pass, after the invariants and the `OutputDriver`. |
| SNP-3 | The displays (LCD, LED, matrix) and the console (`status`, `report`, `display`) read **only** the `OutputSnapshot`. They never read inputs, controllers or drivers directly. |
| SNP-4 | Both snapshots are plain data (no pointers), copyable, and printable as text for the console and for host tests. Snapshot time stamps are `millis()` values (scheduling clock). The wall clock never appears in a snapshot; it is added only when a line is logged (LOG-5). |

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
| DRV-4 | **LCD behavior [found 2026-10-06/08].** (a) The LCD is not touched for **2.5 s after boot**; earlier starts left it showing random characters after a reset. (b) A running LCD is **never re-initialised** (it made the display worse, 0 of 6 clean); initialisation happens once, after the start delay, and again only after an I2C failure. (c) The HD44780 cannot be read back through the PCF8574 backpack (R/W is tied to ground), so corruption is invisible to the firmware: the driver **rewrites the whole screen periodically** (early full rewrites after start-up, then every few seconds). |
| DRV-5 | The LED (TM1650) likewise cannot be read back: the driver rewrites the control byte and all four digits periodically (at least 4 Hz in bring-up builds) so a missing or confused module is noticed within about a second. |
| DRV-6 | The driver's record of the display (`shown()`, `inSync()`) shall stay truthful during a periodic rewrite. |

## 4. Pinout — single file

| ID | Requirement |
|---|---|
| PIN-1 | Every pin number, I2C address, I2C bus selection, **direction**, **pull configuration** and **active level** shall be defined in **one header file**, `BoardPins.h` (header-only, `constexpr`; **not** a `.cpp`). No other file shall contain a pin number, I2C address, or `HIGH`/`LOW` meaning for a signal. |
| PIN-2 | `BoardPins.h` shall hold **one signal table per board** (Minima = production, WiFi = home bench, host = tests), so one can be corrected without touching the others; the board macro (`ARDUINO_UNOR4_MINIMA`, `ARDUINO_UNOR4_WIFI`, or a host set) selects the table and any other board fails to compile. [CHG 2026-10-08] |
| PIN-3 | `BoardPins.h` shall hold a single **signal table**; `setup()` shall configure every pin's mode (input, input-pullup, output) **from that table** and drive every output to its safe level. |
| PIN-4 | Pin assignments shall be checked at compile time: no duplicate pins; no signal on an I2C bus pin; every signal has a declared direction and (for binary signals) a declared **active level**. [NEW] |
| PIN-5 | `BoardPins.h` shall contain only wiring facts. Tunable thresholds and timings live in `Config.h` (§7). |
| PIN-6 | Every binary input and output shall have a declared active level (`ACTIVE_HIGH` / `ACTIVE_LOW`) and code shall use `isOn(signal)` / `setOn(signal, bool)` helpers, never raw levels. |
| PIN-7 | `BoardPins.h` shall carry, per signal: name, direction, active level, header label, wiring note, and which boards it has been **verified on** (with date). Initially every entry is "unverified". The author verifies and completes the table before bench testing. |
| PIN-8 | The firmware shall not use `Serial1` (D0/D1 are TBS/TOB). |
| PIN-9 | The R4 core maps D2–D7 and D10–D13 to different MCU port pins on the two boards. Only plain GPIO/ADC is used, so header numbers should behave the same, but this is **unverified on the Minima** and is settled by DIAG on site (GOAL-9). |

Initial contents, from V7 (**V6 and V7 agree on every pin, active level and I2C address**; differences with V5 are in Appendix C). All **unverified** until DIAG on site:

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
| High-pressure N2 | **A1 (provisional)** | analog in | — | 0–150 PSI. V6/V7 say **A5**, which is the I2C SCL line, so the compile-time check (PIN-4) rejects it; A1 is a placeholder until the owner confirms the real wiring (PIN-10) |
| I2C SDA / SCL | A4 / A5 (core pins 18 / 19) | bus | — | `Wire`; see PIN-11 |
| LED TM1650 | 0x24 | I2C | — | |
| LCD PCF8574 | **0x23** (A2 solder pad bridged; was 0x27) | I2C | — | the TM1650 LED module also answers 0x24-0x27, so the LCD had to move: docs/results/lcd-led-address-clash-20261007.md |
| O2 SEN0465 | 0x74 | I2C | — | SEL dip = 0 |

| ID | Requirement |
|---|---|
| PIN-10 | **[Q1]** In the installed R4 core (renesas_uno 1.6.0) `Wire` is bound to core pins 18/19 = **A4/A5** on *both* boards, and V6/V7 read the high-N2 sensor on **A5**, the I2C SCL line. The owner wants V6/V7 pinouts used (V5 is obsolete and ignored), but A5 cannot be both, so `BoardPins.h` carries **A1 as a provisional pin** for N2-high, with the V6/V7 value recorded in its note. A1 and A2 are unused by V6/V7. The owner is finding out which analog pin the sensor is really wired to; PIN-4 enforces whichever is chosen. |
| PIN-11 | The author reports I2C SDA/SCL are on D18/D19 on the WiFi board and on A4/A5 on the Minima. The installed core defines both as 18/19 (= A4/A5). `BoardPins.h` shall record the bus pins per board; the author will confirm the production wiring. |
| PIN-12 | TBS and TOB roles: **TBS** is the system on/off; it determines whether the generator is enabled and producing N2 and affects the displays (the LCD shows its state if there is room: DSP-9). **TOB** is available in Debug and Production as needed: single-step during diagnosis, confirm/acknowledge an event. |
| PIN-13 | **I2C address map [found 2026-10-07].** LCD **0x23** (backpack A2 solder pad bridged), RTC 0x68 (its module's EEPROM 0x57 is unused), O2 sensor 0x74, LED module: **it answers 0x24-0x27 and 0x34-0x37** (it ignores the low address bits of its control range). An LCD backpack left at its default 0x27 shares an address with the LED module and every LCD character write fails; hence 0x23. The LCD address is one setting (`kLcdAddress`). `BoardPins.h` carries a compile-time/host check that the LCD address is outside the LED module's range. The identical LCD and LED on Tom's unit need the same A2 bridge. |
| PIN-14 | A fallback exists, unused by default: the LED on its own two wires (`BoardDef::ledSdaPin/ledSclPin`, D2/D3) driven by a software I2C master (`SoftI2c`). |

| ID | Requirement |
|---|---|
| I2C-1 | One hardware bus (`Wire`, A4/A5) shared by the LCD, RTC, LED and O2 sensor. The R4 core implements only **100 kHz and 400 kHz**; any other `setClock` value is silently ignored. The firmware runs at 100 kHz (`kI2cClockHz`, checked at compile time). |
| I2C-2 | **Bus recovery [found].** A reset in the middle of a transaction, or a refused write, can leave the bus or the controller wedged (every later transaction fails until something resets it). `Hal::i2cRecover()` ends `Wire`, clocks SCL up to nine times until SDA is released, sends a STOP, restarts `Wire` and **calls `setClock`** (on the R4 core only `setClock` reopens the controller). It runs at boot, on each LCD retry, and when the RTC does not answer. This answers WDT-4. |
| I2C-3 | A driver reports failure for every transaction (DRV-1) and never blocks the loop; an unanswered device is retried about once per second. |
| I2C-4 | A diagnostic (`i2c sweep`) runs the LCD read-back, RTC reads and probes at both real clock speeds and restores the clock. It blocks for a fraction of a second and is a documented exception to NFR-1a. |

## 5. Inputs and scaling

| ID | Requirement |
|---|---|
| INP-1 | **Fast inputs are read on every `loop()` pass** (TBS, TOB, the three pressures) into the `InputSnapshot`, stamped with `millis()` (SNP-1). [CHG 2026-10-08] |
| INP-2 | **ADC resolution** shall be a single constant `ADC_BITS` in `Config.h`, initially **10** (`0x0A`); `setup()` shall call `analogReadResolution(ADC_BITS)` (via the HAL). All code shall work for `ADC_BITS` = 10, 12 or 14. All raw-value limits (valid window, fault window) shall be **derived** from `ADC_BITS` and the 0.5/4.5 V sensor window, never hardcoded. Host tests shall run at all three. After the system works at 10 bits, the author will decide whether higher resolution helps. |
| INP-3 | Scaling shall map the valid window (0.5–4.5 V of the full-scale range) to 0…full scale, fixed-point: air ×10 (0–1500), N2 low ×100 (0–3000), N2 high ×10 (0–1500). At 10 bits the valid window is raw 102…921. |
| INP-4 | [NEW] **Sensor fault detection.** A raw value outside a fault window around the valid window (proposed: below 0.4 V or above 4.6 V) shall mark that sensor **faulty**, shall not be clamped silently into a valid reading, and shall raise a fault (§9). |
| INP-5 | [NEW] A sensor shall be declared faulty only after N consecutive out-of-window samples (proposed N = 3). |
| INP-6 | **TBS and TOB debounce shall be measured, not guessed [owner 2026-10-08].** The compiled default `kDefaultDebounceMs` shall be the value measured on the target hardware; until it has been measured the bring-up value (30 ms) applies (V6 read both raw). The bounce is characterized separately on the **R4 WiFi bench** (momentary buttons) and on the **Minima production panel** (TBS is an SPST rotary switch (OFF / ON, one contact to GND), TOB a pushbutton), see INP-10. TBS ON at power-up is treated as an OFF→ON transition so the system starts. |
| INP-7 | [NEW] **Sensor consistency.** The N2-low reading shall always be lower than the N2-high reading (compared in common units). If N2-low exceeds N2-high by more than a margin (proposed 1.0 PSI, so that two sensors both near zero do not trip) for a hold time (proposed 5 s), with both sensors valid, fault **F04** shall be raised. |
| INP-8 | The system shall show the **raw ADC value, volts and scaled PSI** for each sensor so they can be compared with the production gauges. The gauges are needed **only in the diagnostic phase**, to confirm the sensors agree within a tolerance; they are not required for operation (they are just easier to read than the LCD). BIST-9 shall prompt for the gauge reading and report the difference. |
| INP-9 | **I2C devices are read by non-blocking background steps** (O2 sensor: send the query, wait on a deadline, read, validate; RTC: read and re-anchor the wall clock). A step starts on its own deadline, advances a little on each pass, and **publishes** its result into the next `InputSnapshot` when complete, with the value's **age in ms**. No step blocks `loop()` (NFR-1a); a step that does not finish in time is a failure of that device. [NEW 2026-10-08] |
| INP-10 | **Bounce characterization [NEW 2026-10-08].** A BIST step per switch (TBS, TOB) measures the contact bounce: it samples the input with the displays quiet and the loop polling fast (or timestamps every edge with `micros()`; an LCD row write takes about 10 ms, which is longer than a typical bounce, so it must not run during the measurement), asks the operator to operate the switch N times (proposal 10), and records per operation the **number of edges** and the **settle time** (last edge minus first edge). It reports the count and the minimum, median and maximum settle time and the largest edge count, and **recommends** `debounceMs` = twice the maximum settle time, rounded up to whole milliseconds, within 2…100 ms. POST cannot do this (it needs a person). An optional passive monitor (console command `bounce`) records the same statistics from normal operation. |

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
| INV-7 | On host, property tests shall drive randomized sensor/TBS/clock sequences through all controllers and assert INV-1…INV-4 and INV-8…INV-10 after every step. |
| INV-8 | **F04 inhibits.** While F04 (N2-low reads above N2-high) is active, INV-3 and INV-4 apply as if both N2 sensors were faulty: towers closed, SSR off. |
| INV-9 | **O2 mandatory in `FIELD`.** While F12 (O2 sensor missing or failed) is active in a `FIELD` build, **all outputs are off** (towers, SSR, flush valve) until it clears. `BENCH`, `HOST` and `DIAG` builds do not apply this (the home bench has no O2 sensor), selected by `O2_MANDATORY` in `BuildConfig.h`. |
| INV-10 | **Tower held off until the O2 sensor is warm (FIELD, interim, HQ6).** While the O2 sensor is still warming up (O2-6), the tower controller stays DISABLED (both tower valves closed). The compressor remains governed by the sensor rules (INV-4 and its own thresholds). |
| OUT-1 | **Minimum hold time.** Every output shall have a minimum time since its last change before a *non-safety* change is made (anti-buzz, short-cycle protection). `OUTPUT_MIN_HOLD_MS` = **1000 ms** (decided by the author). The state-machine rules shall guarantee this anyway (tower valves change at most every ~60 s); the OutputDriver enforces it as a backstop and logs when it defers a change. BIST toggling (≥ 500 ms per half-cycle) is the one **documented exception** (the BIST output steps toggle at ~2 Hz). |

**Reset behavior:**

| ID | Requirement |
|---|---|
| RST-1 | Behavior after a reset shall be a function of sensors and TBS only, not of the reset cause or any stored state. |
| RST-2 | All outputs shall be driven to their safe level **as early as the platform allows**, i.e. as the very first action in `setup()`, before `Serial` use, I2C, displays or POST, and the sketch shall verify this order. *Platform finding (core 1.6.0):* `main()` runs `_init()`, the variant's `initVariant()` (not overridable), analog setup and **USB start** before `setup()`, so there is an unavoidable window between reset and our first write; the hardware default (HQ3: outputs OFF in reset) covers it. The length of that window shall be measured at the bench. |
| RST-3 | At boot every controller starts DISABLED. If TBS is ON, the normal enable path runs on the first loop pass and sensor rules choose the state. |
| RST-4 | Compressor initial state on enable shall be chosen from sensors: N2 high above `n2HighOn` → STOPPED_HIGH; else N2 low below `n2LowOn` → STOPPED_LOW; else RUNNING (subject to OUT-1). [CHG] |
| RST-5 | Reset cause (power-on, external, watchdog, brown-out) shall be read at boot if the core exposes it, and logged. |
| RST-6 | The two pushbuttons are the board **RESET** button (a hardware reset; the program sees the resulting reset cause, RST-9) and **TOB**. **TOB held during reset or power-up selects POST mode** (POST-1). BIST is never selected by a button: only by the console command `bist` (BIST-1). [CHG 2026-10-08, owner] |
| RST-7 | Running POST or BIST means normal operation is **disabled**: controllers DISABLED, outputs OFF (except those BIST is exercising). |
| RST-8 | **Hardware default.** Output lines should default to OFF while the MCU is in reset or before `pinMode`. The hardware team assumes outputs are OFF while the MCU is in reset (HQ3), which is why RST-2 matters; if verification shows otherwise, a pull resistor shall be added. |
| RST-9 | **Reset cause [found 2026-10-06/08].** It is read once at boot from the RA4M1 status registers (RSTSR0/1/2) and cleared; the power-on flag itself is not visible after the bootloader, so the chip's cold/warm flag `RSTSR2.CWSF` (set by the firmware at boot) separates power loss from a reset. The cause and the raw register values are logged at every boot (RST-5) and shown by `status`. A **RESET-button test** exists: a marker byte in a spare DS3231 register survives the restart, and the next boot reports PASS only if the cause is a reset-button restart (a power-on instead is `?`). Verified on the R4 WiFi; **to be verified on the Minima**. |

## 7. Controllers and configuration

Thresholds and timings live in a runtime `Config` struct initialized from `Config.h` constants,
so that a **later version** can change pressure thresholds from the console. In this version only
`cfg` (display) is implemented; `cfg set` is reserved (FUT-1).

| Parameter | Value | Units | Meaning |
|---|---|---|---|
| `ADC_BITS` | 10 | bits | ADC resolution (INP-2) |
| `airLowOff` | 650 | PSI×10 (65.0) | tower disables below. [CHG 2026-10-09, owner: the machine's supply peaks at 100 PSI and sags to about 70 PSI for ~300 ms when a valve opens (measured); was 900, then 700] |
| `airLowOn` | 900 | PSI×10 (90.0) | tower may start above. [CHG 2026-10-09, owner; was 1200] |
| `airGraceFromOffMs` / `airGraceToBothMs` | 3000 / 2000 | ms | [NEW 2026-10-09, owner, measured on the machine: the supply sags ~35 PSI (to ~70) for ~300 ms when a tower valve opens and recovers within ~2 s] Low air (INV-2 and the tower's own check) is tolerated for 3 s after a tower valve opens from OFF (OFF->LEFT) and for 2 s after LEFT->BOTH and RIGHT->BOTH. A faulty air SENSOR is never tolerated. |
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
| `O2_WARMUP_MS` | 300 000 | ms | sensor warm-up, **5 minutes (published)** (O2-6) |
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

| ID | Requirement (owner 2026-10-10, from the 2026-10-09 measurements) |
|---|---|
| TWR-OV-9 | **Tom's reply (2026-10-10), confirms TWR-OV-0 and TWR-OV-8:** the overlap's purpose is purely cost efficiency, minimising the demand on the air supply (the first tower pre-fills the second). The 500 ms comes from an example system he used for inspiration (8-inch diameter towers, about 4 ft tall: a very different setup), so it will be re-checked once the towers hold CMS. **He prefers the determinism of a fixed time to code that watches the pressure.** The adaptive rule therefore stays OFF; it is not to be enabled without his agreement. **No extra hardware to read the tower pressures** (no ADS1115). Optional experiment: Tom can connect ONE tower sensor temporarily (the free pin A3) to see the overlap behaviour; the two towers are assumed to behave alike; the supply-air minimum is taken as a good indication of equalisation. |
| TWR-OV-8 | **Default: a FIXED 500 ms overlap (Tom, 2026-10-10).** `towerOverlapMs` = 500 and `overlapAdaptive` = false: the overlap is exactly the fixed time, whatever the air does. The adaptive rule (TWR-OV-1..3, 6) stays built and tested but OFF (`overlapAdaptive` = true switches it on, with `overlapMinMs` as floor and `towerOverlapMs` as cap). To be re-characterized after the carbon molecular sieve (CMS) adsorbent is added to the towers: until then the towers hold only air (N2 LOW is about 79% N2). |
| TWR-OV-0 | **Purpose of the overlap (owner 2026-10-10):** with both valves open the first (full) tower pre-fills the second, so the second tower needs less air from the supply and the first tower's pressure is recovered. With zero overlap the second tower would fill entirely from the supply. The overlap is over when the pre-fill is done. |
| TWR-OV-1 | **The overlap ends when the air supply has passed its minimum**, not after a fixed time. Opening the second valve sags the supply (measured: about 24 PSI, minimum at about 450 ms, recovered by about 2 s); the bottom of that dip is the supply-side signal that the second tower's fill demand has fallen off (the towers have about equalised), which is when the pre-fill is done. Tower pressure sensors (not connected) would measure this directly. |
| TWR-OV-2 | **Confidence:** the minimum is "past" when two consecutive 50 ms samples of the 3-sample median are at least `overlapRiseX10` (1.0 PSI) above the lowest median seen in this overlap. The median keeps one rippled reading from being taken as the minimum. |
| TWR-OV-3 | **Bounds:** never before `overlapMinMs` (200 ms), never after `towerOverlapMs` (750 ms, now the cap). If the minimum is not seen (flat air, a noisy sensor) the cap ends the overlap, exactly as the fixed-time design did: the rule can only shorten the overlap. |
| TWR-OV-4 | The existing INV-2/INV-3 stops and the air grace times are unchanged and still take precedence. |
| TWR-OV-6 | **Tuning from data (2026-10-09 recordings, `docs/results/*_20261009.csv`):** the supply minimum is at 400-500 ms into the overlap; with 1.0 PSI / median of 3 / two in a row the rule ends the overlap 600-700 ms after it began (2.0 PSI would end at 700-750 ms, almost at the cap). The cap (750 ms) leaves only 50-150 ms of margin: raise `towerOverlapMs` if the minimum is later on another day. |
| TWR-OV-7 | **(superseded by TWR-OV-9: no extra hardware; one temporary sensor on A3 at most.)** **Tower pressure sensors, if ever connected, are OPTIMIZATION only (owner 2026-10-10).** N2 LOW and N2 HIGH keep their pins: they are the compressor's safety inputs. The Minima has ONE free analog pin (A3); two tower sensors would need an I2C ADC (e.g. ADS1115 at 0x48). A missing, absent or faulty tower sensor shall never raise an INHIBIT fault or stop the towers; the controller falls back to the supply-air dip rule (TWR-OV-1..3). |
| TWR-OV-5 | The BIST (keys `o`, `x`) and the recorder shall keep measuring the real sag and overlap so these constants can be tuned from data. |

### 7.2 Compressor controller
States: `DISABLED, RUNNING, STOPPED_LOW, STOPPED_HIGH` (OFF, ON, LO, HI).
RUNNING → STOPPED_LOW when N2 low < `n2LowOff`; RUNNING → STOPPED_HIGH when N2 high > `n2HighOff`.
STOPPED_LOW → RUNNING when N2 low > `n2LowOn` **and N2 high is not above `n2HighOff`** [CHG];
STOPPED_HIGH → RUNNING when N2 high < `n2HighOn` **and N2 low is not below `n2LowOff`** [CHG].
All non-RUNNING states hold the SSR off. `enable()` selects the state per RST-4.

### 7.3 O2 controller
Monitoring only (no purity alarm), but in `FIELD` the sensor is **mandatory** (O2-1, INV-9). States: `UNKNOWN, WARMING, FLUSHING, SAMPLING, WAITING, ERROR, DISABLED` (??, WM, F, S, W, E, OF). `WARMING` = sensor answers but the 5-minute warm-up (O2-6) is not over; the sensor-comm check continues.
Cycle as V6/V7: UNKNOWN → (sensor responds) FLUSHING (open flush valve) → after `O2FlushTime` close
valve, first sample → SAMPLING (re-arm per sample, no transition logged) → after `O2SampleCount`
samples store average → WAITING until cycle-start + `O2SampleInterval` → FLUSHING.
N2% ×100 = 10000 − O2% ×100, clamped to 9999 (and 0 if O2 ≥ 100%).

| ID | Requirement |
|---|---|
| O2-1 | **The O2 sensor is mandatory in production.** In a `FIELD` build, a missing or failed O2 sensor is an INHIBIT fault (F12) that turns **all outputs off** (INV-9). The O2 controller otherwise has no authority over the compressor (no purity alarm). **While the sensor is present but warming up, the tower stays disabled (INV-10)** — the owner's interim answer to Q21, to be confirmed with the hardware team (HQ6). |
| O2-2 | ERROR shall not be permanent: it shall retry via UNKNOWN after a delay (proposed 60 s), showing the fault while it persists. [CHG] |
| O2-3 | A failed read shall be detected from the library's actual failure signaling, not only from an I2C address probe. **[found]** The unmodified DFRobot library has **no error signal**: `readGasConcentrationPPM()` returns exactly `0.0` when the reply's checksum fails (and `readTempC()` likewise), so a garbled reply looks like a genuine zero. The adapter (`O2SensorDfrobot`) therefore accepts any **non-zero** reading (a bad reply cannot produce one) and accepts a **zero** only after it repeats three times *and* the library's checksum-validated `queryGasType()` answers "O2"; otherwise it reports a failure. The library also blocks about 10 ms inside every call (`delay(10)`), a documented exception to GOAL-6 that NFR-1 will measure. Other failure modes (stuck bus, wrong address) shall still be characterized at the bench. |
| O2-3a | A reading of **exactly 0.00 % O2** is not a real measurement (air is 20.9 %, even very pure N2 leaves a trace) and shall be treated as a sensor failure like any other: the O2 controller goes to ERROR (F-code for O2 communication), N2 % becomes invalid, and recovery follows O2-2/O2-6 (retry, then a fresh warm-up). |
| O2-3b | **Own read-only driver and the read rule [found 2026-10-08].** The O2 sensor may be read by the project's own driver (`N2_O2_DRIVER=OWN`): the 9-byte frame `FF 01 86 00 00 00 00 00 <cs>` written after register 0; the 9-byte reply read back after at least 10 ms **by deadline, never by `delay`**; validated for header `FF 86`, checksum `(~sum of bytes 1..7)+1`, gas type `0x05` (O2), decimals, and range (0 < % <= 100; exactly 0 is a fault, O2-3a). **Only read commands are ever sent**: never `0x78` (acquire mode), `0x89` (thresholds) or `0x92` (I2C address). `N2_O2_DRIVER=LIBRARY` selects the DFRobot library path (GOAL-7). The two shall be compared on the same sensor before a production default is chosen; the site test package contains a first own-driver query (its log prints the raw reply bytes). |
| O2-4 | The flush valve shall be closed whenever the O2 controller is not in FLUSHING. |
| O2-5 | N2% shall be marked invalid (`--.--`) until the first complete cycle, after any ERROR, after `disable()`, and **while warming up**. |
| O2-6 | **Warm-up (5 minutes).** Samples shall not be taken or displayed until `O2_WARMUP_MS` (5 min) of warm-up has elapsed, counted from sensor power-on (thermal settling). During warm-up the LCD/console show `WARM mm:ss` and N2% is invalid. **Baseline rule:** if the firmware cannot know that the warm-up completed, it assumes it did not and waits the full 5 minutes from boot. |
| O2-6a | **Belt and suspenders: warm-up credit across resets.** A reset-button press does not power-cycle the sensor, so a completed warm-up need not repeat — and because INV-10 holds the tower off during warm-up, a pointless 5-minute wait costs production. The firmware shall use **two independent indications**, and shall skip the wait only when **both** agree: (1) the reset-status register says this was **not** a power-on reset (RA4M1 `RSTSR0.PORF` clear), and (2) a valid record `{magic, checksum, warmCreditMs}` survives in RAM, updated about once per second while the sensor is communicating. At boot: not-power-on **and** valid record → start with that credit; anything else → credit 0 (full 5 minutes). Any O2 communication failure and any power-on reset set the credit to 0. Credit never exceeds `O2_WARMUP_MS`. **[found on the R4 WiFi, reset_probe 1.8–1.B (hex), 2026-10-06]** (a) The reset-cause flags work and clear with plain register writes (no register unlock): after a software reset `RSTSR1=0x04` (SWRF), after a watchdog reset `0x02` (WDTRF), after the reset button both registers read 0. (b) **`.noinit` cannot be used on this core**: the startup code overwrites that area with bytes copied from flash on every boot (the same five instruction-like words appear each time). (c) Ordinary RAM **does** survive software and watchdog resets (checked at three addresses). The record shall therefore live at a **fixed RAM address below the heap and stack** (candidate `0x20007A00`, just under `__HeapLimit`), not in `.noinit`. **(d) `PORF` is not visible to the sketch** (the bootloader clears it, so it reads 0 even after a real power loss); the second safeguard is therefore the chip's **cold/warm start flag `RSTSR2.CWSF`**: it reads 0 after a power loss, and the firmware sets it to 1 at boot so every later reset without power loss reads 1. **Verified on the R4 WiFi (all five cases: power loss, upload/software reset, watchdog, reset button)**: credit is given only when the record is valid AND CWSF reads warm; a power loss always gives 0. Results: `docs/results/reset-probe-wifi-20261006.md`. Still to verify: the same on the **Minima**. |
| O2-6b | **Enable only after testing.** The credit feature shall be controlled by `O2_WARMUP_CREDIT_ENABLED`, **false until proven**. Tests: host tests of the credit logic (record validation, checksum, PORF true/false, comm failure, overflow); bench tests on both boards that (a) the status flag differs between power-on and the reset button, (b) the RAM record survives a reset-button press and a watchdog reset, and (c) is invalid after power removal. Results go in `docs/results/`; the constant is set true only after all pass. Until then the 5-minute wait applies after every boot. |
| O2-7 | **Gating (HQ1: yes).** The O2 cycle may run when the towers are not cycling, **as long as the N2 low and high thresholds are met** (proposed: N2-low above `n2LowOff` and N2-high below `n2HighOff`, i.e. gas is available and the tank is within limits). When they are not met, the cycle pauses and N2% is flagged **stale**. [Q22: confirm this reading of "thresholds met"] |

## 8. Displays

| ID | Requirement |
|---|---|
| DSP-1 | Displays render from the **OutputSnapshot** only (SNP-3), after the controllers, invariants and OutputDriver have run. |
| DSP-2 | Writes to LCD/LED occur when a rendered field has changed (fixed per-field positions, V6 Layout C), **plus** the periodic full rewrites of DRV-4 and DRV-5, because the displays cannot be read back. |
| DSP-3 | **LED:** TBS on and N2% valid → `nn.nn`; TBS on and invalid → `--.--`; TBS off → blank. **Bring-up builds** show instead the time `HHMM` with the middle dot blinking once a second (or the uptime in seconds without an RTC), `FFFF` alternating when a device failed, and the step number during a test. |
| DSP-4 | **LCD layout** (owner decision 2026-10-06/08; renders in `LCD_Layouts.md`): Option 1, clear labels. Row 0: `N2%` and its value (or `WRM mm:ss` during warm-up) and the O2 state; row 1: `N2L` and `N2H` values; row 2: `CMP LO` or `CMP HI` **only while the compressor is stopped for that reason**, otherwise **`ER xx`** (the hex code of the last ERror raised, blank if none since reset), then the tower state and the `LROC` letters; row 3: `AIR` and the four actual output bits under `LROC`. **TBS is not shown** (DSP-9). Only N2 % is shown, never O2 %. |

```
N2% 99.99  O2 S
N2L 12.34 N2H  98.7
        TWR LB  LROC
AIR 123.4       1001
```

| ID | Requirement |
|---|---|
| DSP-5 | **Fault screen.** While any fault of severity ≥ WARN is active, the LCD **alternates** between the normal screen and a **full 20×4 fault screen** on a cycle of `LCD_FAULT_CYCLE_MS` (3000–4000 ms; default 4000), because a fault may make the system inoperable while the sensor readings are still useful. With several faults the fault screen steps through them (fault 1, normal, fault 2, normal, …). The fault screen shows: row 0 `FAULT n OF m <severity>`; row 1 `Fnn` and the fault name (≤ 16 characters); row 2 the measurement behind it (raw counts and volts, the two N2 readings, the O2 state); row 3 what the system does about it (the table's `effect` text). The LED shows `Fnn` of the first active fault. This is the fault channel when no console is attached. [Q9] |
| DSP-6 | A display failure (no ACK) shall never stop the control loop; it raises an INFO fault logged to the console. |
| DSP-7 | LCD and LED initialization shall tolerate an absent or slow device and re-initialize when a later probe finds it. A running LCD shall **not** be re-initialised (DRV-4). |
| DSP-9 | **Withdrawn (owner):** the TBS state is not shown on the LCD; `LROC 0000` and the tower state `OF` already show a disabled system, and the physical switch is at least as visible. The LED still blanks when TBS is off (DSP-3). |
| DSP-10 | **R4 WiFi LED matrix (optional, BENCH/DIAG on the WiFi board only).** The WiFi board has a built-in 12×8 LED matrix (`ArduinoLEDMatrix`; the Minima has none, so production cannot depend on it). It may be used as an extra status display on the bench, e.g. a glyph for the mode (P = POST, B = BIST, R = run), the fault count, and a heartbeat pixel proving `loop()` is alive. Code for it shall be compiled only for `ARDUINO_UNOR4_WIFI`, shall sit behind the display interface (not in the controllers), and shall never be needed for any other requirement. Frame format (verified against the library's own example): 96 bits, 12 columns × 8 rows, row by row, most significant bit first, in three 32-bit words. Used first by `experiments/reset_probe`. |
| DSP-8 | **Startup banner.** At startup the LCD shall show the firmware version and build date for about 1 s (user request); the console prints the full build identity (§10). |
| DSP-11 | **POST mode indication [owner 2026-10-08].** While POST mode runs (POST-1) the **LCD shows each step, its progress and its result**; the **LED shows the LED test display** (8888, then counting) as the interim display; when POST ends the LED shows **0000 after a successful POST** and **FFFF after a failed one**, and the LCD keeps the result and names any fault. The console prints each check. **How long the verdict is held [owner 2026-10-08]:** `FFFF` stays while the POST holds for a fault, i.e. until the operator presses TOB; after a good POST, `0000` on the LED and `POST OK` on the LCD are held for **10 s** (`PostOptions::okMs`) or until TBS is switched ON, whichever is first; then the normal displays return. |

## 9. Faults [NEW]

| Code | Fault | Severity | Effect |
|---|---|---|---|
| F01 | Air sensor out of range | **INHIBIT** | INV-2 |
| F02 | N2-low sensor out of range | **INHIBIT** | INV-4 |
| F03 | N2-high sensor out of range | **INHIBIT** | INV-3 |
| F04 | N2-low reads above N2-high | **INHIBIT** | INV-8 |
| F10 | LCD no ACK | INFO | log only |
| F11 | LED no ACK | INFO | log only |
| F12 | O2 sensor missing/failed | **INHIBIT** in `FIELD` (WARN elsewhere) | INV-9: all outputs off; O2 ERROR; N2% invalid |
| F13 | RTC missing, unreadable or lost power | INFO | log only; log lines carry no wall time |
| F20 | Invariant violation | **INHIBIT** | forced safe state (INV-6); latches until reset |
| F30 | Watchdog reset occurred | WARN | cleared after first normal cycle |
| F31 | Brown-out reset occurred | WARN | same |
| F40 | Console TX dropped output | INFO | counter |

| ID | Requirement |
|---|---|
| FLT-1 | Faults are **self-clearing** when the condition clears for a hold time (proposed 5 s), except F20. |
| FLT-2 | Raise and clear each log one console line with timestamp and code. |
| FLT-3 | TOB pressed during normal run acknowledges and hides WARN/INFO fault display for 60 s. It never clears an INHIBIT condition. Anything beyond a single acknowledgment is done by console command. [Q11] |
| FLT-4 | The fault table is one static table; adding a fault is one row plus one test. **Fault codes are hexadecimal** (two digits, `Fxx`; the high digit is the group). `docs/Fault_List.md` is the canonical list and a host test fails if it differs from the code table. |
| FLT-5 | The LCD normal screen shows the hex code of the **last fault raised** (`ER 12`; blank if none since reset), even after it has cleared (DSP-4). |

## 10. Console (USB serial) and closed-loop log capture

Console = USB CDC through the Arduino IDE Serial Monitor (or `arduino-cli monitor`). Line-oriented
text; commands are one line.

| ID | Requirement |
|---|---|
| CON-1 | Baud 115200 (ignored by USB CDC). Newline-terminated, case-insensitive commands; `help` lists the commands of this build. |
| CON-2 | **Output shall never block the loop and shall never inhibit input.** TX is non-blocking; when it cannot keep up, lines are dropped and counted (F40). The console receive path is polled on every pass, independent of TX state, so IDE→Arduino commands always get through. **[found]** In core 1.6.0 `Serial.write()` returns 0 at once with no host attached, but with a host attached that is not reading, the core's write loop spins until the buffer drains. The USB CDC TX buffer is **256 bytes**. The console layer therefore checks free space (`availableForWrite()`) before every line and never waits: log lines are dropped and counted; multi-line answers (`status`, `report`, BIST output) are queued and **deferred, never dropped**; a line is never longer than about 100 bytes so it always fits once the host drains. The watchdog is the backstop. To be verified on the bench. |
| CON-3 | Connecting or disconnecting the Serial Monitor while running **shall not affect operation**, other than enabling logging and commands. This shall be verified on both boards (including whether the board resets on connect; never open the port at 1200 baud, which triggers the bootloader). |
| CON-4 | **Attach detection.** The firmware shall detect whether a console is attached and expose it as `console.attached()`. **[found]** The two boards differ. *UNO R4 Minima* (native USB): `if (Serial)` is true only while a host has the port open (CDC DTR); never call `Serial.dtr()` (it forces the connected state permanently); the core starts `Serial` itself. *UNO R4 WiFi* (core built with `-DNO_USB`): `Serial` is a hardware UART to the ESP32 USB bridge; **the sketch must call `Serial.begin(115200)` itself** (until then every print returns 0); `if (Serial)` is **always true** (a PC cannot be detected); `availableForWrite()` is **not implemented** (always 0); and each `write()` **blocks until the byte has left the UART** (about 87 µs per byte at 115200). The HAL shall therefore provide `consoleBegin()`, and on the WiFi board shall treat the console as always attached, pace output with a byte budget of about 11 bytes per millisecond (capped around 128 bytes per call) so one loop pass never blocks long, and keep a periodic banner so a monitor opened later sees the build identity. BIST "needs an attached console" can only be enforced on the Minima. |
| CON-5 | Log levels `ERR, WARN, INFO, DEBUG`; default DEBUG in Debug, INFO in Production; `log <level>`. **Every state transition is logged at INFO or higher**, so a production unit with a console attached shows them all (and a headless unit simply drops them); DEBUG only adds detail. |
| CON-6 | The line reader is a non-blocking accumulator with a bounded line length; over-long lines are discarded with an error. |
| CON-7 | In the `FIELD` build no console command shall drive an output directly. Outputs are exercised only through BIST. [Q12] |

**Build identity (user request):**

| ID | Requirement |
|---|---|
| ID-1 | A `BuildInfo` shall give: firmware version string (manual constant), build date and time (`__DATE__`/`__TIME__`), board, phase/build, ADC_BITS, and library/core versions where obtainable. It is printed at every boot, by `ver`, in `status`, and in the report header; the version/date is also shown on the LCD at startup (DSP-8). A git commit hash is added when a build script provides it. |

**Closed-loop capture:**

| ID | Requirement |
|---|---|
| LOG-1 | **One line format for every console log line** (POST, BIST, faults, controllers, commands): `[<wall> | ]<ms> <L> <text>`. `<wall>` = `YYYY-MM-DD HH:MM:SS`, present only when an RTC is present and trusted, followed by the delimiter ` | `; `<ms>` = `millis()` (the scheduling clock), always present; `<L>` = level letter E, W, I, D. With no RTC (or an untrusted one) the wall part **and its delimiter are omitted**, with no placeholder: `912345 I TWR LB->R +60000 next:913000`. Typed commands are echoed as `> command`, so a pasted log shows what was asked. [CHG 2026-10-08, owner] |
| LOG-2 | A `report` command shall print one delimited block (`==== N2 REPORT BEGIN ====` … `==== N2 REPORT END ====`) with: build identity, uptime, reset cause, POST results, config, inputs (raw/volts/scaled), controller states, outputs, active faults, loop-time statistics, and the display model. |
| LOG-3 | **Baseline capture is copy/paste from the Arduino IDE Serial Monitor** into a text file; the output format (LOG-1, LOG-2) shall make that sufficient: short sessions (a BIST run, a `report`) fit in the Monitor's buffer, and every file starts with a build-identity header. The Monitor's own timestamp option may be turned on. An optional host-side capture tool (`tools/n2log`) is a later convenience for long sessions, because only one program can hold the port at a time. [Q19 resolved] |
| LOG-4 | The `time <iso>` command (optional) may set a wall-clock reference stored in RAM only; the capture tool also prefixes host wall-clock time, so captured files are datable without it. |
| LOG-5 | The wall time comes from a software clock (`WallClock`) anchored to the RTC at boot and every 60 s (no I2C per line). It is for people only: **scheduling never reads it** (RTC-7), and a missing RTC never stops anything (RTC-8). `status`/`report` answer lines are not log lines and carry no stamp; the report header carries the wall time and `ms`. |

| Command | Builds | Purpose |
|---|---|---|
| `help`, `ver`, `status`, `report` | all | usage; build identity; snapshot; full delimited report |
| `log <level>` | all | log level |
| `faults` | all | active faults |
| `cfg` | all | display config (future: `cfg set`, FUT-1) |
| `display` | all | print what the LCD and LED should currently show |
| `loop` | all | loop-time statistics (min/mean/median/max); `loop reset` |
| `scan` | all | I2C scan |
| `time`, `time set <date> <time>` | all | RTC date/time; set it only when untrusted or more than 2 s off (RTC-3, RTC-8) |
| `lcd`, `lcd reinit`, `lcd bus [n]` | all | LCD state; restart its controller; I2C link read-back test |
| `i2c sweep [n]` | all | bus test at 100 and 400 kHz (I2C-4) |
| `post` | all, only if TBS OFF | run POST (refused with an explanation while TBS is ON) |
| `bist` | all, only if TBS OFF and console attached | run BIST (refused with an explanation while TBS is ON) |
| `nvm` | all | non-volatile memory: both settings copies, each check (magic, schema, length, checksum), write count, saved-at time, sketch version |
| `debounce`, `debounce set <TBS ms> <TOB ms>` | all (set: only with TBS OFF) | TBS/TOB debounce times in use; save new ones to NVM (2..100 ms) |

## 11. Power-on self-test (POST) [NEW]

POST is **hands-off and does not require a console**. [CHG 2026-10-08, owner] It runs **only in POST mode**: when **TOB is held at power-up or reset** (sampled at the start of `setup()` once the outputs are driven safe). A **normal boot runs no self-test** (POST-1).

| ID | Requirement |
|---|---|
| POST-1 | **POST mode** is selected by TOB held during power-up or reset. A **normal boot** only drives the outputs safe (RST-2), logs the reset cause (RST-5) and goes straight to normal operation: the sensor rules and invariants (§6, RST-1) and the fault logic (§9) protect the machine as always, and an O2 sensor missing in production still turns everything off (INV-9) with the fault on the LCD. POST mode is quick (proposed <= 3 s) and shows itself (DSP-11). **TOB is acknowledged [owner 2026-10-08]:** when TOB is seen held at the start, the LED shows `PoSt` at once, the LCD (when it starts, 2.5 s after boot) shows `POST MODE / RELEASE TOB`, the console prints `TOB seen. Release TOB to start the POST`, and the POST **begins when TOB is released** (or after 10 s if TOB is stuck, with a warning). |
| POST-2 | Checks, each as a non-blocking `DeviceCheck::post()` (CHK-1): every device (LCD, RTC, LED, O2 sensor, TBS/TOB) plus: outputs verified safe, reset cause logged, each pressure sensor within the valid window, config sanity, build identity. Unexpected I2C responders are listed. |
| POST-3 | Result: one summary line (`POST PASS`, `POST WARN n`, `POST FAIL n`) plus one line per check, to the console if attached, and on the LCD. |
| POST-4 | **In POST mode, POST shall hang if and only if a fault occurs.** On a fault of severity ≥ `POST_HANG_SEVERITY` (WARN) the system stays disabled with the fault shown on the LCD (and console if attached) until the operator presses **TOB** (a fresh press: a TOB held since power-up must be released first). The matching faults stay active, so the invariants still protect production. **F30/F31 (a watchdog or brown-out reset happened) do not hold POST**: they only record history, and holding on them would keep an unattended unit down after the very reset the watchdog exists to recover from [Q25]. A missing O2 sensor in production is an INHIBIT fault and holds POST. |
| POST-5 | A clean POST ends with `POST OK` on the LCD and **0000** on the LED; a failed one ends with the fault on the LCD and **FFFF** on the LED (DSP-11). POST then continues to normal operation (after TOB, when it held on a fault, POST-4). |
| POST-6 | `post` on the console re-runs POST, **only with TBS OFF [owner 2026-10-08]**: with TBS ON the system is enabled and the command is refused with an explanation (`POST refused: the system is enabled (TBS is ON). Switch TBS OFF, then type post.`). While POST runs the system stays disabled; when it ends the normal TBS rule applies (TBS ON enables the system, per RST-3). Both POST and BIST exist in every build including FIELD. |

## 12. Built-in self-test (BIST)

Interactive diagnostics **with operator confirmation**, for Debug and for site visits. BIST is the
most important feature during Debug. It **requires an attached console**.

| ID | Requirement |
|---|---|
| BIST-1 | BIST runs **only on the console command `bist`**, with TBS OFF, so a console is always attached. It is never selected by a button or by TOB at power-up (that selects POST mode, POST-1). A refused request (TBS on) leaves the system undisturbed and prints why. During the BIST the TBS switch is **tested without enabling the system**; when the BIST ends the normal TBS rule applies again (TBS ON enables the system). [CHG 2026-10-08, owner] V6/V7 ran BIST on every boot and blocked, which hangs an unattended unit. |
| BIST-7 | **A reset-button reset in the middle of a BIST resumes it [owner 2026-10-08].** The BIST keeps its step and the verdicts so far in a small checksummed RAM record (`BistRecord`, fixed address `kBistRecordAddress`) that survives a reset-button reset, not in NVM (no flash wear). At the next boot, if the reset was the button (not a power loss, watchdog or brown-out), TOB is not held (POST mode wins), TBS is OFF and a console is attached, the BIST restarts at that step (`BIST RESUMED after a reset at step N ...`). Quitting (`q`) or finishing clears the record. `q` is the abort key during a BIST: it prints the results so far and turns every output OFF. |
| BIST-2 | BIST disables normal operation (RST-7). Steps are numbered in HEX; the LED shows the step (except in the LED test), the LCD shows the step (except in the LCD test), the console logs every step and observation. |
| BIST-3 | **Operator confirmation.** Each step ends with a prompt on the console; the operator types one letter and Enter (or presses **TOB = `p`**). The result is printed as one line per step and a final table, all in the captured log; nothing is stored on the board. |

| Key | Meaning | Effect |
|---|---|---|
| `p` | **Pass.** What I saw, heard or read matched the console's expectation. | Step recorded PASS; next step. |
| `f` | **Fail.** Optional note follows, e.g. `f LCD row 2 blank`. | Step recorded FAIL with the note; next step (BIST does not stop; the final table lists all failures). |
| `r` | **Rerun / repeat.** I missed it or want another look. | Step runs again from the start; nothing recorded. |
| `s` | **Skip.** Not applicable here (e.g. no O2 sensor on the bench). | Step recorded SKIPPED; next step. |
| `q` | **Quit.** Stop the BIST now. | Outputs forced OFF; the table so far is printed; normal operation resumes (BIST-7). |

| BIST-4 | **Display steps echo the expectation to the console** (DRV-3): e.g. `EXPECT LCD  row0 "0123456789ABCDEFGHIJ"` and `EXPECT LED  "8.8.8.8."`, so the operator compares console and display and then confirms. |
| BIST-5 | **Output steps** run only with TBS OFF; each toggles at ~2 Hz for 10 s or until confirmation (the valves are loud enough to confirm audibly) and leaves the output OFF on exit, timeout or any abort (INV-5). Output steps print which signal, which pin, and the active level. |
| BIST-6 | The watchdog (if enabled) is kicked from every BIST wait loop. |
| BIST-7 | After BIST, normal operation resumes without a reset after re-running RST-2/RST-3. |
| BIST-8 | BIST sequencing and formatting shall be host-testable; only HAL calls touch hardware. |
| CHK-1 | **One class per device.** Each device (reset cause, LCD, RTC, LED, O2 sensor, TBS/TOB, RESET button; later the pressures and outputs) implements `DeviceCheck`, which provides its POST check and its BIST step together. Adding a device is one class plus one line in the list; `SelfTest` runs POST and BIST over the list. [found 2026-10-07] |
| CHK-2 | Where a step can decide by itself it does (I2C acknowledge and error counts, the RTC advancing in step with `millis()`, a valid O2 reply, a switch seen in both states); operator p/f is asked only where eyes or ears are needed (LCD, LED, valves). |
| CHK-3 | Results are visible on the LCD/LED where present, so a console is helpful but never the only place to see a result (the site test package, ENV-8, works with no console at all). |
| BIST-11a | **[found]** The vetoes applied to BIST output steps are INV-2, INV-3, INV-4 and INV-8, evaluated on the **raw** ADC reading as well as the debounced fault flag (a dead sensor must not be able to pass during the first samples). INV-9 and INV-10 (O2 sensor missing / warming up) are **not** vetoes: they protect production, and the BIST is the tool for diagnosing exactly that sensor. The flush valve is not vetoed by pressures. A refused `bist` request leaves a running system undisturbed. |
| BIST-11 | **No BIST step shall create an unsafe condition.** Every output step is subject to the invariants (INV-2…INV-4, INV-8…INV-10) as **vetoes**: it refuses to start, and aborts at once with all outputs OFF, if its action would violate one (for example the SSR step requires N2-high below `n2HighOn`, N2-low above `n2LowOff`, and valid sensors; valves one at a time; TBS must be OFF). Which steps the hardware team may run unattended is their decision; the firmware guarantees the veto. |
| BIST-10 | **Usable by a second person.** The BIST prompts shall be self-explanatory (what to look at, what the right answer looks like, which key to press) so that a colleague who is not the author can run it from a written package (ENV-3) and email back the captured text. |
| BIST-9a | The `g air|n2l|n2h <psi>` line in the pressure step prints the difference between the entered production-gauge reading and the sensor. The compressor SSR step gives **one 1 s pulse** by default (hardware question HQ8 pending); `BistConfig` selects the 2 Hz toggle. |
| BIST-9 | The sensor step shall show raw ADC, volts, scaled PSI and the in-window flag, and ask the operator to enter (or confirm) the **production gauge reading** for the gauges that exist (INP-8). |

The step list is the `DeviceCheck` list (CHK-1). Built and bench-validated so far: **reset cause, LCD, RTC, LED**, plus O2 (own read-only query), TBS/TOB and the RESET button in the site test package. The table is the original plan; the pressure, valve and SSR steps (5, 7-A) are still to be built as `DeviceCheck`s.

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
| B | Summary | table of pass/fail/skip per step |

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
| NFR-1a | **A normal loop pass shall take well under a second: target < 100 ms at the maximum.** The slow parts are spread over passes: LCD writes (at most 6 characters per pass), the console (paced by a byte budget), and I2C sensor reads (one per scheduled deadline). Documented exceptions: the DFRobot library blocks about 10 ms per O2 read, and the diagnostic console command `lcd bus` blocks for up to 0.3 s when typed. Measured on the bench (Stage 1, UNO R4 WiFi): mean 14 µs, median 100 µs, max about 25 ms. |
| NFR-2 | Flash and RAM shall be reported at every build for both boards; initial budget **60 %** of each, raised if necessary. **Current full firmware (core 1.6.0, 2026-10-02):** Minima 36 % flash / 28 % RAM; WiFi 37 % / 39 % (with the DFRobot library; without it 33 % / 34 %). **Baseline (N2V7, core 1.6.0, 2026-10-02):** Minima 67 288 B flash (25 %), 5 648 B RAM (17 %); WiFi 70 816 B flash (27 %), 9 180 B RAM (28 %). |
| NFR-3 | The current production sketch shall be kept as a known-good fallback for the field trip. |
| NFR-9 | **Non-volatile memory (NVM).** The RA4M1 has a small data-flash area, reachable through the core's `EEPROM` library. It is **not used yet**. If it is used later (console-set thresholds, the O2 warm-up record), writes shall be rare and counted, a stored block shall carry a version and a checksum, and a missing or invalid block shall fall back to the compiled defaults, so the machine always starts safely. |
| NVM-1 | **Stored debounce [NEW 2026-10-08].** `debounceTbsMs` and `debounceTobMs` live in the non-volatile memory as one versioned, checksummed block (NFR-9). At boot a valid block with both values in 2…100 ms **overrides** the compiled default; a missing or invalid block means the compiled default is used and nothing is written. The block is written only when the operator accepts a recommendation at the end of the bounce BIST step (rare, counted). After the characterization on both boards, the compiled default `kDefaultDebounceMs` is changed to the measured value and the results are filed in `docs/results/`. |
| FUT-1 | **Reserved for a later version:** changing pressure thresholds from the console (`cfg set`). The runtime `Config` struct (§7) shall make this a small change. |

## 14b. Real-time clock (DS3231, optional) [NEW — draft, 2026-10-06]

A battery-backed DS3231 at I2C address 0x68 (the same bus as the displays and the O2 sensor) gives wall-clock time. The controller
**never depends on it**: a missing, unset or wrong clock is at most an INFO fault.

| ID | Requirement |
|---|---|
| RTC-1 | The driver (`Rtc3231`, behind the Hal's `i2cReadReg`/`i2cWrite`) shall read and set the date and time, validate every register (BCD, month, day with leap years, hour; year 2000–2199 with the century bit), handle a chip left in 12-hour mode, and report success or failure for every call, counting bus errors. Calendar helpers are pure functions (`DateTime.h`). |
| RTC-2 | The driver shall report whether the time can be trusted: the DS3231 oscillator-stop flag (OSF) means the clock lost power since it was last set. `set()` clears OSF and preserves the other status bits. |
| RTC-3 | The console shall provide `time` (print the RTC date and time, and whether it is trusted) and `time set YYYY-MM-DD HH:MM:SS`. A malformed or impossible date shall be rejected without touching the chip. |
| RTC-4 | The driver shall read the chip's temperature (0.25 °C steps) for diagnostics (BIST, `status`). |
| RTC-5 | When fitted and trusted, the date/time shall appear in the boot banner and at the top of the `report` block, so a captured log can be dated (LOG-4). A missing RTC or an untrusted time shall be reported as INFO fault **F13**, never inhibit anything, and never delay POST. |
| RTC-6 | Module safety (hardware, for the owner): many DS3231 boards (ZS-042 style) have a charging circuit meant for a rechargeable LIR2032; with a non-rechargeable CR2032 the cell can overheat. Fit an LIR2032, remove the charging resistor/diode, or use a board without it. |
| RTC-7 | The RTC shall be used **only to time-stamp console log lines** (LOG-1, LOG-5), never for scheduling or control. A software wall clock (`WallClock`) is anchored to the RTC at start-up and every 60 s, so stamping costs no I2C traffic. If the RTC is absent or untrusted the clock is "not synced" and lines carry no wall time. |
| RTC-8 | The RTC is read-only except that `time set` writes it when it is untrusted, unreadable, or differs from the given reference by more than 2 s. A missing RTC shall never stop POST or normal operation. |

Status (2026-10-08): RTC-1…RTC-8 implemented and host-tested; the driver, the battery backup, the console commands, F13, the banner and the periodic re-anchoring validated on the R4 WiFi bench (`docs/results/rtc-test-wifi-20261006.md`, `lcd-led-address-clash-20261007.md`). RTC-6 is a hardware action for the owner. The year range is 2000-2199 in the register layout (the helpers' 32-bit seconds reach 2136).

## 14a. Reproducible environment and remote testing [NEW]

Goal: some tests can be run by the hardware team in production from a package sent by email or fetched
with `git pull`, so the author need not travel for every test (the author attends for debugging).
That only works if both sides build from the **same versions**.

| ID | Requirement |
|---|---|
| ENV-1 | The exact environment shall be recorded in a committed file `env.lock`: Arduino core (`arduino:renesas_uno`, now 1.6.0), `arduino-cli` and IDE versions, DFRobot_MultiGasSensor (now 3.0.0), and any other library. |
| ENV-2 | **The receiving side needs no scripts.** The production-site laptop is a **Windows** machine running the Arduino IDE; it can receive a `.zip` and `git pull`, but external scripts must not be assumed. Everything the receiver needs is therefore plain files (sketch, libraries, README) plus IDE steps written out by hand. Version checking on the receiving side is done **by the firmware**: its banner and `ver` print the build identity and the versions it can see (ENV-5), and the README lists the IDE/core/library versions to match. Author-side scripts (macOS) may check `env.lock`. |
| ENV-3 | The author builds a **test package** (`.zip`), with an author-side script if convenient: the sketch (DIAG build), a copy of every third-party library at the locked version, `env.lock`, a step-by-step README written for Windows and the Arduino IDE (install the board package version, copy the library folder, open the sketch, select the board, upload, open the Serial Monitor, what to answer, what to copy back), an example of the expected output, and a checksum. It shall work with no network access once unpacked. |
| ENV-4 | `git pull` of a tagged commit is an equivalent transport; the tag names the exact package. |
| ENV-5 | The firmware banner and `ver` shall print enough to trace a returned log to its package: firmware version, build date/time, git commit (supplied by `make_package`), board, ADC_BITS and the locked versions (ID-1). |
| ENV-6 | The remote test shall be **DIAG only** (no controllers), and the package README shall say which steps move valves or the compressor and what the system must look like (for example air supply isolated) before they are run. [Q24] |
| ENV-7 | *Option to verify:* an `arduino-cli` **sketch profile** (`sketch.yaml`) can pin the core and libraries; DFRobot_MultiGasSensor is **not** in the Library Manager index (searched 2026-10-02), so it would have to be supplied as a local directory or git URL. Whether the profile mechanism accepts that shall be checked before relying on it. |
| ENV-8 | **Site test package [found 2026-10-08].** For a person who is not the author, the package is a `.zip` with ONE self-contained sketch in a same-named folder (only the core's `Wire`; no libraries to install), a plain-text `README.txt` operator guide for Windows and the Arduino IDE (it says to remove the Arduino's own power supply as well as USB, because the unit has a separate supply), and an `EXPECTED_RESULTS.txt`. The sketch needs no console (results on the LCD and LED), sets the RTC from the compile time, reads the switch inputs but never drives an output. First one: `deliverables/tom_i2c_check.zip` (LCD, RTC, LED, O2 query, TBS, TOB, RESET). Its source is separate from the firmware tree, so it is checked by compiling, not by the host tests. |

## 15. Owner-supplied facts still to confirm
See `Owner_TODO.md`. Highlights: complete pinout with active levels; which production gauges exist; I2C
pins on production; SEN0465 warm-up time; output pull-down wiring.

## 16. Open software questions

Hardware questions are kept separately in §17 and in `Owner_TODO.md` part 1.

| # | Question | Proposal / status |
|---|---|---|
| Q1 | *(moved to hardware: HQ7)* | |
| Q2 | LCD/LED libs | **Resolved:** DIY minimal, from V7 mini-libs; reuse TCP logic where good |
| Q3 | Fixed-point ranges as V7 | Resolved: yes |
| Q4 | Sensor fault window/sample count | Resolved: ok (0.4/4.6 V, 3 samples) |
| Q5 | RST-4 and §7.2 cross-checks | Resolved: ok |
| Q6 | Minimum output hold time | **Resolved:** 1000 ms; BIST has a 2 Hz exception |
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
| Q17 | TBS and the displays | **Resolved:** LCD shows `ON`/`OFF` if room (DSP-9); LED blank when off |
| Q18 | F04 severity | **Resolved:** INHIBIT (INV-8) |
| Q19 | Capture tool | **Resolved:** copy/paste from the IDE Serial Monitor is the baseline (LOG-3); an optional script comes later |
| Q20 | Missing O2 sensor | **Resolved:** mandatory in production; inhibit all; holds POST (O2-1) |
| Q21 | Does production run while the O2 sensor is warming up? | **Interim answer: no — tower disabled until warm (INV-10).** Question for the hardware team: HQ6 |
| Q22 | HQ1 "N2 low and high thresholds are met" | **Resolved:** above the low minimum and below the high maximum, i.e. within operating range (O2-7) |
| Q25 | POST does not hold on F30/F31 (a previous watchdog or brown-out reset), so an unattended unit restarts after a watchdog reset | Proposed: yes (POST-4) |
| Q26 | BIST vetoes exclude INV-9/INV-10 (O2 missing/warming) because the BIST is how that is diagnosed (BIST-11a) | Proposed: yes |
| Q24 | BIST steps run by the hardware team | **Resolved:** the hardware team decides which steps to run; the firmware guarantees BIST never creates an unsafe condition (BIST-11) |
| Q23 | Power-on vs reset discrimination and warm-up credit | **Resolved: yes** (O2-6a/6b), enabled only after bench tests |
| Q27 | Which inputs are read per pass | **Resolved 2026-10-08:** GPIO and ADC every pass; I2C devices by non-blocking background steps with age (INP-9) |
| Q28 | How controllers see each other | **Resolved:** through the previous pass's OutputSnapshot (CTL-2) |
| Q29 | O2 driver | **Resolved:** both, selectable (`N2_O2_DRIVER`, O2-3b); compare on real hardware |
| Q30 | Boot modes | **Resolved:** normal boot runs nothing; TOB at power-up = POST mode; `bist` on the console = BIST (POST-1, BIST-1) |
| Q31 | Log format | **Resolved:** `[wall | ]ms L text`, wall omitted without an RTC (LOG-1) |
| Q32 | Controller interface | **Resolved:** abstract base class `Controller`, `update()` (CTL-1) |
| Q33 | LCD and LED on one bus | **Resolved:** LCD at 0x23, A2 bridged (PIN-13) |
| Q34 | TBS/TOB debounce (INP-6) | **Resolved 2026-10-08:** characterize on both boards (INP-10), store in NVM (NVM-1), then change the compiled default |

## 17. Questions for the hardware team (answers recorded 2026-10-02)

| # | Question | Answer |
|---|---|---|
| HQ1 | Can the O2 sample cycle run when the towers are not cycling? | **Yes**, as long as the N2 low and high thresholds are met (→ O2-7, Q22). |
| HQ2 | Is there a mechanical/hardware safeguard independent of the Arduino (relief valve, pressure switch, power cut)? | **No.** The firmware is the only protection (→ GOAL-11). |
| HQ3 | Do the valve/SSR drivers default OFF in reset or with a floating pin? | **Assume yes (safe)** during reset. More important to set the outputs as early as possible in startup (→ RST-2, RST-8). |
| HQ4 | Which production gauges are readable? | **TBD** (author will check; INP-8). |
| HQ5 | SEN0465 warm-up time? | **5 minutes** (→ O2-6). The owner believes it shares the Arduino's supply; to be confirmed with the hardware team. |
| HQ7 | **Which analog pin is the high-pressure N2 sensor really wired to?** V6/V7 say A5, which is the I2C clock line; `BoardPins.h` carries A1 as a placeholder (PIN-10). | open |
| HQ8 | The BIST SSR step toggles the compressor SSR at ~2 Hz (V6). Is that acceptable for the compressor, or should it be one short pulse (e.g. 1–2 s)? | open |
| HQ6 | **Should the system run while the O2 sensor warms up (5 min)?** Owner's interim answer: no, the tower stays off until the sensor is warm (INV-10). Does the system need N2 production earlier, or is the delay acceptable after every power-up? | open |

---

## Appendix A — Differences from V6

| Area | V6 v1.1 | This draft |
|---|---|---|
| Architecture | Monolithic, no abstraction, pasted libraries | Thin HAL, fakes, host tests, drivers behind interfaces |
| Pinouts | `const` block in the sketch | One header, board-selected, with direction/pull/active level, compile-time checked, set up from a table |
| ADC | 10-bit hardcoded | `ADC_BITS` 10/12/14, limits derived |
| BIST | Every boot, blocks | On request, console required, operator confirms p/f/r/s/q, display echo |
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

## Appendix C — Pin history

V6 and V7 agree on every pin, active level, I2C address and tuning constant (V7 adds `O2CommRetryMs`)
and are the pin source for V8. **V5's pinouts and its bit-banged I2C buses are obsolete and are
ignored** (owner, 2026-10-02): V8 uses the standard hardware `Wire` on SDA/SCL.

The one V6/V7 value that cannot be used as-is is the high-pressure N2 sensor on **A5** (the I2C SCL
line); see PIN-10. Still to confirm in production: which analog pin that sensor is really on, and that
the output modules' active level is as V6/V7 assume.

