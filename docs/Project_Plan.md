# Nitrogen Generator Controller — Project Plan (v0.2 DRAFT)

Companion to `Requirements.md` v2.1. Requirement IDs (`PIN-10`, `INV-3`, …) refer to it.
Owner action items are in `Owner_TODO.md`. v0.2 folds in the author's review of v0.1.
No firmware code exists yet.

## 1. Principles

1. **Requirements first, then tests, then code.** Each requirement ID gets a test before its code.
2. **Host first.** Everything provable on a laptop or in CI is proven there. Hardware time is scarce.
3. **Diagnostics before logic (GOAL-9).** The first thing that touches production is a diagnostic-only build that guarantees the pinouts. Production logic only runs on hardware whose wiring has been proven.
4. **Reuse what worked (GOAL-8).** V5 controllers/tests, V6/V7 state machines and V7 mini-libraries are the starting material, brought under host tests.
5. **Closed loop.** The production unit prints; the laptop captures to a file; the file comes back to Claude for analysis (§8).
6. **Never lose the working unit.** The known-good firmware stays available for the field trip (NFR-3).

## 2. Decisions

| # | Decision | Outcome |
|---|---|---|
| P1 | Folder | **Approved.** `N2V8/`. V6/V7 are **used as reference material**: copied into `reference/` (as `.ino.txt`, so the Arduino IDE does not compile them) together with the V6 requirements and the V5 test set. V6/V7 originals stay untouched. |
| P2 | Version control | **New branch `v8` in the existing GitHub repo `egp/N2`**; `main` is overwritten (replaced by a merge or fast-forward) only when V8 works. History is not important; existing code is reused where it meets requirements. |
| P3 | Test framework | **Approved:** CMake + Catch2; BB_/WB_ naming kept. |
| P4 | Pinout file | **Approved:** header `BoardPins.h` (PIN-1). |
| P5 | Display drivers | **DIY minimal**, starting from the V7 mini-libs, behind an I2C transport interface, tested heavily; TCP libs consulted. |
| P6 | Build modes | `HOST`, `BENCH`, `DIAG`, `FIELD` (Requirements §2). |
| P7 | Log capture | A host-side script around `pyserial` writing a timestamped text file (LOG-3). |

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
                           field_checklist.md  results/
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
| **Compiling for the R4 on this Mac** | **BLOCKED.** The core's compiler (`arm-none-eabi-gcc 7-2017q4`) is an **x86_64** binary; this Mac is arm64 (macOS 27.2) and **Rosetta is not installed** (`Bad CPU type in executable`). The Arduino IDE needs the same compiler, so the IDE cannot build for the R4 either until this is fixed. |

Options (your call, see `Owner_TODO.md`): (a) install Rosetta (`softwareupdate --install-rosetta --agree-to-license`, needs admin rights); (b) install a native arm64 `arm-none-eabi-gcc` and point the build at it (unsupported by the core; needs testing); (c) use **GitHub Actions on Ubuntu** (x86_64) for board compiles and size numbers and keep this Mac for host tests and editing. (c) is useful in any case; (a) is needed anyway to flash from the IDE.

## 6. Milestones (renumbered; the old M3 is split into M3 and M4)

### M0 — Baseline and repository
- Verify toolchain and library versions (done above); resolve the R4 compiler issue (§5).
- **Baseline compile of `N2V7.ino` for both boards** — local if Rosetta/native compiler is available, otherwise in CI after M2. Record flash/RAM.
- Create branch `v8` in `egp/N2` with the `N2V8/` tree (§9); copy `reference/`.
- **Exit:** branch pushed (with your approval); baseline sizes recorded or CI-pending.

### M1 — Requirements sign-off and pinout verification
- Iterate `Requirements.md` until the open questions are closed or deferred.
- **You** verify and complete the pinout, including **direction, pull, active level, and I2C pins for both boards**, and resolve the A5/SCL conflict (Owner_TODO 1–4). Their result goes into `BoardPins.h` as data.
- **Exit:** requirements APPROVED; signal table complete.

### M2 — HAL, pinout, host skeleton, CI
- `BoardPins.h` with PIN-2/PIN-4/PIN-6 checks, `Hal.h`, `FakeHal`, CMake/Catch2, first tests (pin table, scaling windows at 10/12/14 bits, rollover-safe deadlines).
- `.github/workflows/ci.yml` on Ubuntu: (1) host tests, (2) `arduino-cli compile` for both boards using the same `src/` and the latest libraries, with a size report.
- **Exit:** `ctest` green locally and in CI; both boards compile in CI.

### M3 — DIAG build: console, drivers, POST, BIST  *(new — diagnostics first)*
- Console (non-blocking both ways, attach detection, command parser, log format, `report`), build identity, loop-time statistics (min/mean/median/max).
- LCD and LED drivers behind a transport interface with exhaustive golden-byte tests; display model and the `display` echo.
- POST (hands-off; hangs only on fault; TOB releases) and BIST (console required; operator confirms; y/n table).
- No controllers: `DIAG` keeps all outputs OFF except inside BIST.
- **Exit:** host tests green; DIAG compiles for both boards in CI; golden screens for every BIST step.

### M4 — Core logic (TDD on host)
1. Scaling, sensor-fault detection and consistency check (INP-3…8).
2. Fault table (FLT-1…4).
3. Timed state machine — **reuse the proven V7 pattern, harden it** (ARC-6: rollover, exhaustive switches, recovery to DISABLED), plus the log line format.
4. OutputDriver (active levels, min hold, safety bypass; OUT-1, INV-5).
5. Tower, Compressor (RST-4 cross-checks), O2 controller (warm-up, retry, staleness).
6. Invariants layer and **property tests** (INV-1…7).
7. Boot/reset sequencing (RST-1…8) factored so the host can "reset" mid-scenario.
- **Exit:** every M4 requirement has a test; property tests run thousands of randomized steps with no invariant violation.

### M5 — Full build and integration
- `FIELD`/`BENCH` modes: controllers + displays + console + POST/BIST + watchdog; `HalArduino`, `HalSim`, O2 adapter around the DFRobot library.
- **Exit:** all modes compile for both boards; sizes within the 60 % budget (NFR-2).

### M6 — Bench bring-up (R4 WiFi, home)
Results go to `docs/results/bench-YYYYMMDD.md`:
1. DIAG: boot, POST, banner, LCD/LED via BIST steps 3–4 with `display` echo.
2. Serial: attach/detach with the Serial Monitor; no host attached; commands still received while output is flooding (CON-2, CON-3).
3. I2C stuck-bus behavior and watchdog (WDT-1…4).
4. Simulated inputs through every transition and every invariant; fault line on the LCD.
5. Loop-time statistics; choose the watchdog timeout (NFR-1).
6. Soak test with simulated cycling.
- **Exit:** all bench-verifiable requirements ticked; the rest go to the field checklist.

### M7 — Field kit
- `docs/field_checklist.md` (§7); printed `BoardPins.h` signal table; the known-good binary; `tools/n2log`; laptop with IDE, same core and libraries.
- **First field artifact is the DIAG build.**
- **Exit:** dry run of the checklist on the bench.

### M8 — Field commissioning
- **Visit 1 (DIAG):** capture the full BIST run with operator confirmations, sensor volts vs plant gauges, outputs confirmed by sound, I2C scan, O2 comm and warm-up. Update `BoardPins.h` "verified on Minima" column. Take the captured text file back for analysis with Claude.
- **Visit 2 (FULL):** run the checklist; capture POST and the first minutes of operation; observe a full tower cycle.
- **Exit:** field acceptance; PIN-7 verified column complete.

### M9 — Production hardening
- Production phase (quiet default, watchdog on, tuned timeout), re-run host/bench tests, flash, check headless restart (power-cycle with TBS ON, reset button, USB unplugged).
- **Exit:** unit runs headless; documented return procedure. Then V8 replaces `main`.

## 7. Field checklist (outline; expand in M7)

1. Photograph wiring; TBS OFF; note which firmware is installed.
2. Connect the laptop; start `tools/n2log` (or Serial Monitor); keep the capture file.
3. Flash **DIAG**; boot; read POST.
4. Run BIST steps 0–B with TBS OFF, confirming each y/n; record gauge readings against BIST step 5.
5. Verify each output by sound; each input by toggling; each I2C device by echo.
6. Save the captured file → `docs/results/field-DATE.md` (Claude reviews).
7. Fix `BoardPins.h` only; rebuild; re-run POST/BIST.
8. Flash **FULL**; TBS ON; observe a tower cycle and a compressor start with the log running.
9. Reset-button test with TBS ON; confirm sensors decide the state (RST-1…4).
10. Disconnect USB; leave running. If any step fails, flash the known-good firmware.

## 8. Closed-loop feedback

The Arduino has no storage, so the **laptop** is the recorder. `tools/n2log` opens the USB port,
prefixes host wall-clock time, writes `logs/n2-YYYYMMDD-HHMMSS.txt`, and forwards typed commands.
The firmware prints a `==== N2 REPORT ====` block on request, a boot banner with build identity,
and machine-readable log lines (LOG-1…2), so a captured file is self-contained. You then give the
file to Claude to analyze. Note: only one program can hold the port; close the Serial Monitor while
capturing, and stop the capture before uploading.

## 9. Git plan (to do together; nothing pushed until you agree)

Current state: `N2V8/` has two files and is not a repository; `egp/N2` has one branch `main`
(the V5 layout, last push 2026-06-07).

1. In `N2V8/`: `git init`, add `origin` = `https://github.com/egp/N2` (or the SSH remote, since your SSH key exists), `git fetch origin`.
2. Create local branch `v8` **based on `origin/main`** without changing the working tree (so the branches share history and can merge cleanly), then stage the new tree: the V5 files are removed, the V8 docs and `reference/` added.
3. Add `.gitignore` (build outputs, `logs/`, `.DS_Store`), `LICENSE` kept from `main`.
4. First commit on `v8`; show you `git status` and the diff summary.
5. **With your approval:** `git push -u origin v8`. `main` stays untouched.
6. At M9: merge `v8` into `main` (clean because of the shared base), or replace `main`'s tree, as you prefer.

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
| **No R4 compiler on this Mac (no Rosetta)** | Certain (now) | High | Install Rosetta, or CI compile (§5) |
| A5/SCL conflict makes current wiring unworkable | High | High | Owner resolves pinout (M1); PIN-4 compile-time check |
| Minima pin behavior differs from WiFi | Medium | Medium | DIAG on site proves it before any logic runs |
| `Serial` blocks or a Serial Monitor connection upsets the board | Unknown | High | CON-2/3 tests on bench and on site |
| I2C hang from a bad device | Medium | High | Wire timeout + watchdog; display failure never stops control |
| O2 sensor warm-up unknown after reset | Certain | Low | Assume not warmed (O2-6) |
| Active levels or pull-downs wrong | Medium | High | Table-driven setup, compile-time checks, DIAG confirmation |
| Scope creep before the first visit | Medium | Medium | First visit is DIAG only (M3) |

## 12. Working agreement

- I propose; you decide. Open questions stay in `Requirements.md` §16–17 until answered.
- No firmware code until M1 is signed off, except small experiments (e.g. the PIN-10 bench test).
- I do not edit `N2V6/` or `N2V7/`. New work lives in `N2V8/`.
- Anything that touches hardware, flashing, system settings or pushes to GitHub is confirmed with you first.
