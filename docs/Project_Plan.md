# Nitrogen Generator Controller — Project Plan (v0.2 DRAFT)

Companion to `Requirements.md` **v2.6**. Requirement IDs (`PIN-10`, `INV-3`, …) refer to it.
Owner action items are in `Owner_TODO.md`. v0.2 folds in the author's review of v0.1.

**Status (2026-10-07):** Host M0–M5 done (~408 tests green). Firmware exists on branch `v8`.
**M6 (bench) is in progress:** E1 reset probe **DONE on R4 WiFi**; Stage 1 (LCD+RTC) built with host tests green;
full Stage 1 bench run and DIAG flash still open. LED blocked on a replacement module. `main` stays V5 until M9.

## 1. Principles

0. **Design priorities, in order: testability, readability, maintainability** (GOAL-10). When two designs compete, this order decides.
1. **Requirements first, then tests, then code.** Each requirement ID gets a test before its code.
2. **Host first.** Everything provable on a laptop or in CI is proven there. Hardware time is scarce.
3. **Diagnostics before logic (GOAL-9).** The first thing that touches production is a diagnostic-only build that guarantees the pinouts. Production logic only runs on hardware whose wiring has been proven.
4. **Reuse what worked (GOAL-8).** V5 controllers/tests, V6/V7 state machines and V7 mini-libraries are the starting material, brought under host tests.
5. **Closed loop.** The production unit prints; the laptop captures to a file; the file comes back to Claude for analysis (§8).
6. **Never lose the working unit.** The known-good firmware stays available for the field trip (NFR-3).
7. **The firmware is the only protection** (hardware answer HQ2: no independent safeguard; GOAL-11). The invariants in Requirements §6 get an independent review before the first hardware visit and again before production.

## 1a. Status snapshot (2026-10-07)

Everything that can be tested without hardware is built and tested: **~408 host tests**, all passing, including
randomized property tests of the safety rules and **mutation checks** (a deliberately broken safety rule
is caught by at least one test). The sketch builds for both boards in both modes (DIAG, FIELD):
**36–37 % flash, 28–39 % RAM** with the DFRobot library (sizes from the 2026-10-02 full-firmware snapshot; re-measure after big adds).
CI (Ubuntu) runs the host tests and builds the four board/mode combinations.

**Bench progress (R4 WiFi, 2026-10-06):** E1 reset probe complete (`docs/results/reset-probe-wifi-20261006.md`);
LCD and RTC drivers validated; Stage 1 sketch built (`stages/stage1/`, see `docs/Bringup_Stages.md`).
Still open on the WiFi bench: full Stage 1 soak, DIAG flash (Owner_TODO 10a), LED (replacement module),
then the rest of M6. Minima: reset probe and DIAG still open. Pinout owner-verify before FIELD.

What the host tests **cannot** prove, and so still needs silicon: remaining I2C/display edge cases; USB serial
behavior (no host, stalled host, connect/disconnect); watchdog/reset-cause on Minima; the DFRobot adapter against a
real SEN0465; pin wiring and active levels (DIAG on site).

## 2. Decisions

| # | Decision | Outcome |
|---|---|---|
| P1 | Folder | **Approved.** `N2V8/`. V6/V7 are **used as reference material**: copied into `reference/` (as `.ino.txt`, so the Arduino IDE does not compile them) together with the V6 requirements and the V5 test set. V6/V7 originals stay untouched. |
| P2 | Version control | **New branch `v8` in the existing GitHub repo `egp/N2`**; `main` is overwritten (replaced by a merge or fast-forward) only when V8 works. History is not important; existing code is reused where it meets requirements. |
| P3 | Test framework | **Approved:** CMake + Catch2; BB_/WB_ naming kept. |
| P4 | Pinout file | **Approved:** header `BoardPins.h` (PIN-1). |
| P5 | Display drivers | **DIY minimal**, starting from the V7 mini-libs, behind an I2C transport interface, tested heavily; TCP libs consulted. |
| P6 | Build modes | `HOST`, `BENCH`, `DIAG`, `FIELD` (Requirements §2). |
| P7 | Log capture | **Copy/paste from the IDE Serial Monitor** is the baseline (LOG-3). An optional `pyserial` script may come later for long sessions. |

## 3. Repository layout (branch `v8`; the repo root *is* the sketch folder `N2V8/`)

```
N2V8/                      (repo root; cloned as folder N2V8 so the Arduino sketch name matches)
  N2V8.ino                 thin: setup()/loop() → src/ ; selects DIAG or FIELD mode
  src/
    BoardPins.h            ALL pins, I2C addresses, directions, active levels (PIN-1)
    BuildConfig.h          phase / build macros
    Config.h               tunable constants → runtime Config struct
    hal/     Hal.h  HalArduino.cpp  HalSim.cpp
    core/    Scaling Inputs Faults Invariants OutputDriver TimedState
             Tower Compressor O2Controller Snapshot Log LoopStats
    ui/      DisplayModel LcdView LedView Console Commands Report
    selftest/ Post Bist
    drivers/ Lcd20x4 Led1650 O2Sensor (+ I2C transport interface)
  test/host/               CMake + Catch2; fakes (FakeHal, FakeI2C); golden screens
  tools/                   n2log (capture), traceability script, buildinfo script
  reference/               N2V6.ino.txt  N2V7.ino.txt  N2V6_Requirements.md  V5 tests
  docs/                    Requirements.md  Project_Plan.md  Owner_TODO.md
                           Bringup_Stages.md  field_checklist.md  results/
  stages/                  staged bring-up sketches (stage1 = LCD+RTC+console)
  experiments/             throw-away bench sketches (reset_probe, lcd_test, …)
  .github/workflows/ci.yml
```

Arduino IDE 2 compiles `.cpp`/`.h` under `src/` recursively, so the same tree builds in the IDE,
with `arduino-cli`, and in the host test build. `test/`, `tools/`, `reference/` and `docs/` are
not compiled into the sketch. The V5 files on `main` stay in the history; `v8` begins with a
commit that removes them from the tree.

## 4. Environments and what each can prove

| Capability | Host / CI | Bench (R4 WiFi, home) | Field (R4 Minima) |
|---|---|---|---|
| Controller logic, invariants, reset rules | ✔ primary | ✔ sim inputs | ✔ |
| Scaling at ADC_BITS 10/12/14, fault detection | ✔ | ✔ | ✔ |
| Compiles for both boards, flash/RAM | ✔ CI (Ubuntu) | | |
| LCD/LED drivers on real parts | golden bytes only | ✔ with `display` echo | ✔ |
| I2C behavior (scan, stuck bus, timeouts) | | ✔ partial | ✔ |
| USB serial without a host; connect/disconnect | | ✔ | ✔ **repeat** |
| Watchdog | | ✔ | ✔ |
| Pin mapping on Minima, active levels | | ✘ | ✔ **DIAG, only here** |
| Real pressures, valves, SSR, O2, gauges | | ✘ | ✔ |

## 5. Toolchain status (found during M0)

| Item | Status |
|---|---|
| `arduino-cli` 1.5.1, core `arduino:renesas_uno` 1.6.0 (latest) | installed |
| `cmake`, `clang++` | installed |
| DFRobot_MultiGasSensor | local 3.0.0 = upstream master (latest) |
| **Compiling for the R4 on this Mac** | **Resolved 2026-10-02.** The core's compiler is an x86_64 binary and needs Rosetta; it was missing (`Bad CPU type in executable`). Rosetta 2 was installed by the owner; `arduino-cli` now builds for both boards. CI (GitHub Actions, Ubuntu) is still planned as an independent check. |

**Baseline: `N2V7.ino` compiled with `arduino-cli` 1.5.1, core 1.6.0, DFRobot 3.0.0:**

| Board | Flash | RAM (globals) |
|---|---|---|
| UNO R4 Minima | 67 288 B of 262 144 (25 %) | 5 648 B of 32 768 (17 %) |
| UNO R4 WiFi | 70 816 B of 262 144 (27 %) | 9 180 B of 32 768 (28 %) |

Both build without errors; the only warnings are inside the Arduino core itself. Plenty of headroom against the 60 % budget (NFR-2).

## 6. Milestones (renumbered; the old M3 is split into M3 and M4)

### M0 — Baseline and repository
- Verify toolchain and library versions (done above); resolve the R4 compiler issue (§5).
- **Baseline compile of `N2V7.ino` for both boards — DONE** (§5: Minima 25 % flash / 17 % RAM; WiFi 27 % / 28 %).
- Create branch `v8` in `egp/N2` with the `N2V8/` tree (§9); copy `reference/`.
- **Exit: MET.** Branch `v8` pushed to `egp/N2` (2026-10-02); baseline sizes recorded.

### M1 — Requirements sign-off and pinout verification
- Iterate `Requirements.md` until the open questions are closed or deferred.
- **You** verify and complete the pinout, including **direction, pull, active level, and I2C pins for both boards**, and resolve the A5/SCL conflict (Owner_TODO 1–4). Their result goes into `BoardPins.h` as data.
- **Exit:** requirements APPROVED; signal table complete.

### M2 — HAL, pinout, host skeleton, CI  — **DONE 2026-10-02 (CI green: github.com/egp/N2/actions/runs/37041781290)**
- `src/BoardPins.h`: single signal table (pin, direction, active level, note), board definitions for Minima / WiFi / host, compile-time `checkBoard()` (PIN-2, PIN-4, PIN-6), logical on/off ↔ physical level helpers.
- `src/hal/Hal.h` (thin interface), `HalArduino` (real, device builds only), `test/host/FakeHal.h` (recording, scriptable).
- `src/board/BoardSetup.*`: table-driven `configurePins`, `driveOutputsSafe`, `beginHardware` (PIN-3, RST-2, INP-2).
- `src/core/Scaling.h`, `src/core/Timing.h`: ADC window and scaling derived from `kAdcBits` (10/12/14), sensor-fault classification, rollover-safe deadlines.
- `test/host/`: CMake + Catch2 v3.16.0; **30 tests**, all named after the requirement they cover; `-Wall -Wextra -Wpedantic -Werror`.
- `.github/workflows/ci.yml`: host tests, and `arduino-cli compile` for both boards on Ubuntu.
- `N2V8.ino`: skeleton (pins, ADC bits, I2C start, banner if a console is attached). **Not yet flashed to any board.**
- **Results:** host tests 30/30 pass; sketch compiles for both boards with no warnings from our code. Skeleton size: Minima 45 744 B flash (17 %) / 5 096 B RAM (15 %); WiFi 58 632 B (22 %) / 8 992 B (27 %) — most of that is the Arduino core itself.
- **Findings during M2:** (1) the core's `initVariant()` is not overridable and USB starts before `setup()`, so RST-2 is now "first action in `setup()`" (Requirements RST-2); the reset-to-first-write window is a bench measurement. (2) The macros `ARDUINO_UNOR4_MINIMA` / `ARDUINO_UNOR4_WIFI` are confirmed on both targets. (3) N2-high is a **provisional A1** because V6/V7's A5 is the I2C SCL line (PIN-10); the test suite proves the V6/V7 value would be rejected.
- **Exit criterion met:** `ctest` green locally and in CI; both boards compile in CI.

### E1 — Reset probe experiment  — **DONE on R4 WiFi (2026-10-06)**; Minima still open
`experiments/reset_probe/` — read-only sketch (no output pins, no devices) that reports reset-cause flags, a plain-RAM warm-up record, and console attach. WiFi matrix complete: results in `docs/results/reset-probe-wifi-20261006.md` (CWSF cold/warm + plain RAM; `.noinit` unusable on this core; PORF not visible to the sketch). Still to run: the same matrix on the **Minima**. Settles O2-6a/6b and CON-3.

### M3 — DIAG build: console, drivers, POST, BIST  *(new — diagnostics first)* — **DONE on the host**
- Console (non-blocking both ways, attach detection, command parser, log format, `report`), build identity, loop-time statistics (min/mean/median/max).
- LCD and LED drivers behind a transport interface with exhaustive golden-byte tests; display model and the `display` echo.
- POST (hands-off; hangs only on fault; TOB releases) and BIST (console required; operator confirms p/f/r/s/q; pass/fail table).
- Console TX checks free buffer space before every write (CON-2 finding) and exposes `consoleAttached()` (CON-4); the HAL gains console, watchdog (refresh each loop, 4 s) and reset-cause calls.
- LCD layout chosen from `LCD_Layouts.md` and turned into golden screens (DSP-4, DSP-9).
- **Environment and remote package:** `env.lock`, `tools/check_env`, `tools/make_package` (ENV-1…7), so the hardware team can run DIAG from a zip or a `git pull`.
- No controllers: `DIAG` keeps all outputs OFF except inside BIST.
- **Exit:** host tests green; DIAG compiles for both boards in CI; golden screens for every BIST step.

### M4 — Core logic (TDD on host) — **DONE**
1. Scaling, sensor-fault detection and consistency check (INP-3…8).
2. Fault table (FLT-1…4).
3. Timed state machine — **reuse the proven V7 pattern, harden it** (ARC-6: rollover, exhaustive switches, recovery to DISABLED), plus the log line format.
4. OutputDriver (active levels, min hold, safety bypass; OUT-1, INV-5).
5. Tower, Compressor (RST-4 cross-checks), O2 controller (warm-up, retry, staleness).
6. Invariants layer and **property tests** (INV-1…7).
7. Boot/reset sequencing (RST-1…8) factored so the host can "reset" mid-scenario.
- **Exit:** every M4 requirement has a test; property tests run thousands of randomized steps with no invariant violation.

### M5 — Full build and integration — **DONE on the host** (`App`, the real sketch, DIAG/FIELD builds, O2 adapter)
- `FIELD`/`BENCH` modes: controllers + displays + console + POST/BIST + watchdog; `HalArduino`, `HalSim`, O2 adapter around the DFRobot library.
- **Exit:** all modes compile for both boards; sizes within the 60 % budget (NFR-2).

### M6 — Bench bring-up (R4 WiFi, home) — **IN PROGRESS** (E1 WiFi done; next: Stage 1 bench + DIAG flash)
Results go under `docs/results/` (per-experiment notes already started; a rolled-up `bench-YYYYMMDD.md` later):
1. ~~E1 reset probe on WiFi~~ — **DONE 2026-10-06**.
2. Stage 1 (LCD+RTC+console): built; host green; **full bench run still pending** (`stages/stage1/`, `docs/Bringup_Stages.md`).
3. DIAG: boot, POST, banner, LCD/LED via BIST steps 3–4 with `display` echo — **flash still open** (Owner_TODO 10a). LED blocked on replacement TM1650.
4. Serial: attach/detach with the Serial Monitor; no host attached; commands still received while output is flooding (CON-2, CON-3).
5. I2C stuck-bus behavior and watchdog (WDT-1…4).
6. Simulated inputs through every transition and every invariant; fault line on the LCD.
7. Loop-time statistics; choose the watchdog timeout (NFR-1).
8. Soak test with simulated cycling.
- **Exit:** all bench-verifiable requirements ticked; the rest go to the field checklist.

### M7 — Field kit
- `docs/field_checklist.md` (§7); printed `BoardPins.h` signal table; the known-good binary; laptop with IDE, same core and libraries.
- **First field artifact is the DIAG build.**
- **Exit:** dry run of the checklist on the bench.

### M8 — Field commissioning
- **Visit 1 (DIAG):** capture the full BIST run with operator confirmations, sensor volts vs production gauges, outputs confirmed by sound, I2C scan, O2 comm and warm-up. Update `BoardPins.h` "verified on Minima" column. Take the captured text file back for analysis with Claude.
- **Visit 2 (FULL):** run the checklist; capture POST and the first minutes of operation; observe a full tower cycle.
- **Exit:** field acceptance; PIN-7 verified column complete.

### M9 — Production hardening
- Production phase (quiet default, watchdog on, tuned timeout), re-run host/bench tests, flash, check headless restart (power-cycle with TBS ON, reset button, USB unplugged).
- **Exit:** unit runs headless; documented return procedure. Then V8 replaces `main`.

## 7. Field checklist (outline; expand in M7)

1. Photograph wiring; TBS OFF; note which firmware is installed.
2. Connect the laptop; open the Arduino IDE Serial Monitor (timestamps on); you will copy its contents into a text file at the end of each session.
3. Flash **DIAG**; boot; read POST.
4. Run BIST steps 0–B with TBS OFF, confirming each p/f; record gauge readings against BIST step 5.
5. Verify each output by sound; each input by toggling; each I2C device by echo.
6. Save the captured file → `docs/results/field-DATE.md` (Claude reviews).
7. Fix `BoardPins.h` only; rebuild; re-run POST/BIST.
8. Flash **FULL**; TBS ON; observe a tower cycle and a compressor start with the log running.
9. Reset-button test with TBS ON; confirm sensors decide the state (RST-1…4).
10. Disconnect USB; leave running. If any step fails, flash the known-good firmware.

## 8. Closed-loop feedback

The Arduino has no storage, so the **laptop** is the recorder. Baseline procedure: run the session
in the Arduino IDE Serial Monitor, then **select all, copy, and paste into a text file**
(`docs/results/…`), and give that file to Claude. The firmware makes this sufficient: every file
starts with a build-identity banner, log lines are machine-readable (`ms level tag …`), and the
`report` command prints one delimited block (LOG-1…3). The Monitor keeps a limited scrollback, so
run `report` at the end of a long session. If long unattended captures are ever needed, a small
`pyserial` script can be added later; it would replace the Serial Monitor while it holds the port.

## 9. Git plan

**Done (2026-10-02, with the owner's approval):**
1. `N2V8/` initialized as a repository; remote `origin` = `git@github.com:egp/N2.git` (SSH).
2. Local branch `v8` created from `origin/main` (shared history), V5 moved under `reference/`.
3. First commit `a9248e8` pushed: `origin/v8`. `main` is untouched (`eb3093d`).

**Still to do:**
4. Continue on `v8` (host and bench); each milestone ends in a commit; pushes are confirmed with the owner. Tip as of this status sync: see `git log` on `v8` (do not treat this doc as the tip SHA).
5. At M9 only: merge `v8` into `main` (or replace `main`'s tree), as the owner prefers — **not before**.

Commit messages end with the attribution line shown in the session settings.

## 10. Verification strategy

- **Names:** `BB_` black-box, `WB_` white-box; test names begin with the requirement ID.
- **Traceability:** a script scans test names → `docs/traceability.md` (requirements with no test are listed).
- **Property tests:** random sensors/TBS/clock; INV-1…4 hold after every step.
- **Scenario tests:** scripted air-drop, tank-full, TBS toggles, mid-cycle reset, O2 warm-up.
- **Golden screens/bytes:** exact 4×20 text per state; exact I2C bytes for drivers.
- **Variants:** `ADC_BITS` 10/12/14; rollover at 2³²−1.
- **Human in the loop:** displays are confirmed by the operator against the console `display` echo; results appear in the captured log.
- **CI:** GitHub Actions (Ubuntu) runs host tests and compiles both boards on every push.

## 11. Risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| ~~No R4 compiler on this Mac (no Rosetta)~~ | Resolved | — | Rosetta installed 2026-10-02 |
| A5/SCL conflict; A1/A2 may be left/right tower sensors, leaving no free analog pin for N2-high | High | High | Owner finds out what is wired (M1); PIN-4 compile-time check; if truly six analog sensors, revisit I2C wiring |
| **No independent hardware safeguard (HQ2)** | Certain | High | GOAL-11: invariants reviewed independently; DIAG proves wiring first; owner may wish to consider a hardware over-pressure device |
| Minima pin behavior differs from WiFi | Medium | Medium | DIAG on site proves it before any logic runs |
| `Serial` blocks or a Serial Monitor connection upsets the board | Unknown | High | CON-2/3 tests on bench and on site |
| I2C hang from a bad device | Medium | High | Wire timeout + watchdog; display failure never stops control |
| O2 sensor warm-up unknown after reset | Certain | Low | Assume not warmed (O2-6) |
| Active levels or pull-downs wrong | Medium | High | Table-driven setup, compile-time checks, DIAG confirmation |
| Scope creep before the first visit | Medium | Medium | First visit is DIAG only (M3) |

## 12. Working agreement

- I propose; you decide. Open questions stay in `Requirements.md` §16–17 until answered.
- Host and `v8` firmware work may proceed. **M1 pinout / active-level verify** (Owner_TODO 1–4) must be done before trusting a FIELD build or a production visit. Small bench experiments are fine anytime.
- Do not merge `v8` → `main` until M9 (known-good V5 on `main` stays available for the plant).
- I do not edit `N2V6/` or `N2V7/` originals. New work lives in this tree (`N2V8/` sketch, `stages/`, `experiments/`, `docs/`).
- Anything that touches hardware, flashing, system settings or pushes to GitHub is confirmed with you first.
