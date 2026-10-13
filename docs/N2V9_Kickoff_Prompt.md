# Prompt to start the N2V9 chat (paste everything below the line into a new chat; working directory /Users/egp/Documents/Zephyr/Arduino)

---

You are starting **N2V9**, a lean rewrite of the controller for a PSA nitrogen generator (Arduino UNO R4 Minima in production at Tom's garage; UNO R4 WiFi on my home bench, which has only the RTC, LED and LCD). It runs **in parallel with N2V8**, which stays as it is: do not modify anything under `N2V8/`. The repo is github.com/egp/N2, local clone `/Users/egp/Documents/Zephyr/Arduino/N2V8`. Create `N2V9/N2V9.ino` beside `N2V8/` on a new branch `v9`, cut from `site-20261009` (it carries the latest docs, measurements and decisions; `main` is older).

## Goal
Same machine and same safety behaviour as N2V8 (`docs/Requirements.md`: tower, compressor, O2 controllers, invariants INV-1..INV-10, faults), but **much leaner**:
* **One file: `N2V9/N2V9.ino`. The only includes are `Arduino.h`, `Wire.h` and `DFRobot_MultiGasSensor.h`** (library v3.0.0, installed by ZIP, not in the Library Manager). The Minima's watchdog needs the core's `WDT.h`: tell me, don't silently add it. Everything else is in the sketch.
* **Fewest instructions that do the job, then readability and maintainability.** Shallow call stack: flat `loop()` that reads inputs, runs the state machines, writes outputs, updates the displays; no layers of classes, no virtual interfaces, no Hal abstraction. Prefer plain structs, small functions, `static`/`const` tables and integer arithmetic. Non-blocking `millis()` scheduling, nothing ever `delay()`s. Comments explain WHY (requirement IDs, measured facts), not WHAT.
* **Drop** (tell me if you think any of these should stay): BIST and POST, the run modes DIAG/BENCH/FIELD, NVM-stored settings and the debounce meters, the recorder/capture, the full console command set, the host Hal and its test layers, multiple LCD layouts. Keep a tiny serial output (115200): boot banner with version, and one log line per state transition / fault; production may be headless.
* **Keep:** all safety invariants, fault detection (`docs/Fault_List.md`, F-codes on the LED, text on the LCD), the watchdog **on**, a bounded I2C (see lessons below), and the version in ONE `#define` (hex-style numbering like N2V8).

## Process (my standing rules)
1. **Requirements first.** Start N2V9 with `docs/N2V9_Requirements.md`: derive it from `docs/Requirements.md` (v8), say what is dropped and what is simplified, and give each kept requirement an ID. Show me before coding.
2. Then tests, then code. A single `.ino` is hard to unit-test: **propose** how (for example a host harness that `#include`s the sketch against a small Arduino/Wire shim, with scenario tests ported from N2V8's invariant and property tests, plus the measured curves in `docs/results/`). I decide.
3. **Never flash without my go-ahead.** Minima uploads: I double-tap RESET, then you run `arduino-cli upload` (port changes; check `ls /dev/cu.usbmodem*`). No prebuilt binaries; Tom builds from the IDE on Windows (`git clone -b <branch>`; the DFRobot library by ZIP), no helper scripts for him.
4. Valves/compressor: ask my explicit "go" before anything that opens a valve or starts the compressor on the real machine.

## Hardware facts (verified at Tom's 2026-10-09; see `N2V8/src/BoardPins.h`, `docs/schematics/`, `docs/photos/`)
* Inputs: **TBS D0** (SPST rotary OFF/ON, closed to GND = ON, `INPUT_PULLUP`, active LOW), **TOB D1** (momentary pushbutton, active LOW; barely used).
* Outputs (active HIGH): **LEFT valve D4, RIGHT valve D7, FLUSH (O2 flush) valve D11, compressor SSR D8**. All other valves/regulators on the machine are manual or autonomous.
* Analog (0.5-4.5 V, 10-bit ADC via `analogReadResolution(10)`): **AIR A0 (0-150 PSI), N2 LOW A1 (0-30 PSI), N2 HIGH A2 (0-150 PSI)**; A3 free; tower pressure sensors exist but are NOT connected. N2 LOW/HIGH are the compressor's safety inputs.
* I2C 100 kHz on A4/A5: **LCD 20x4 PCF8574 at 0x23** (backpack A2 bridged), **LED 4-digit TM1650** (control 0x24, digits 0x34-0x37), **O2 sensor DFRobot MultiGas at 0x74**, **RTC DS3231 0x68 (absent at Tom's: not a fault)**.
* Production values (`N2V8/src/ControlConfig.h`, read it): air stop 65 PSI / start 90 PSI (supply peaks about 100-106 PSI and sags about 35 PSI for about 300 ms when a valve opens from closed, about 24 PSI for the second valve); low-air grace 3000 ms OFF->LEFT and 2000 ms for LEFT->BOTH / RIGHT->BOTH (never for a sensor fault); sensor fault = out of the 0.4-4.6 V window for 3 samples AND 50 ms; **tower fill 59250 ms, overlap FIXED 500 ms** (Tom's decision: determinism over pressure-watching; re-check after the CMS adsorbent is added); N2 LOW stop 10 PSI / start 20 PSI, N2 HIGH start below 100 / stop above 120 PSI; TBS/TOB debounce 30 ms.
* O2: the DFRobot library has no error signal: exactly 0.00 % means failure; flush -> sample -> wait cycle; 5-minute warm-up; N2% = 100 - O2%; mandatory in production (no sensor, no production).

## Lessons learned the hard way (all in `docs/`, `N2V8/src`)
* **LCD:** after init send ONLY cursor positions and characters; never touch the backlight, display on/off or entry mode; a periodic Return Home (0x02) fixes the display-shift fault; refresh the normal screen at most 1 Hz; a stuck bus must not stall the loop. LCD content cannot be read back.
* **I2C:** the R4 Wire default timeout is 100 ms PER transaction: call `Wire.setWireTimeout(3000)`; a held bus stalled the N2V8 loop for seconds (3 freezes / watchdog resets on 2026-10-09). Recover the bus (9 clocks + STOP). Do not use software I2C. Never connect or disconnect I2C wires while powered.
* **Watchdog:** keep it ON; record where the loop was in a few bytes of RAM that survive a reset and print them after a watchdog reset (see N2V8 `core/StallRecord.h`).
* TBS ON at power-up = treated as an OFF->ON transition after the start conditions hold; powering with TBS ON must not start outputs by itself. Turning outputs OFF for a safety reason is never delayed by the 1 s output minimum-hold.
* The console lines must stay short (about 96 chars); the R4 WiFi drops console bytes at boot, so do not make boot output depend on being read.

## Where to look first
`docs/Requirements.md`, `docs/Fault_List.md`, `docs/LCD_Layouts.md`, `docs/Next_Visit_Plan.md`, `docs/Owner_TODO.md` (section 2C), `docs/results/*` (measured air sag/overlap curves), `docs/schematics/*.pdf`, `docs/photos/`, `N2V8/src/ControlConfig.h`, `N2V8/src/BoardPins.h`, `N2V8/src/core/{Tower,Compressor,O2Controller,Invariants,Sensors}.cpp`.

## First deliverables
1. A one-page lean-design proposal: what is kept/dropped, the loop structure, the data layout, the test approach, an estimate of code size against N2V8 (about 124,800 bytes on the Minima, 47 % of flash).
2. `docs/N2V9_Requirements.md`.
3. Only after I approve both: the sketch and its tests.

Ask me anything that blocks you; otherwise pick the sensible default and tell me.
