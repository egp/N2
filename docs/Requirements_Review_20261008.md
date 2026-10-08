# Requirements review — 2026-10-08

Review of `Requirements.md` (v2.6 draft, 541 lines) against (1) the owner's statements of 2026-10-08 and (2) what the bring-up work taught us. Nothing in
`Requirements.md` has been changed yet: this is the agenda for the dialogue. IDs are the existing ones; **R-n** are review findings.

## A. The owner's new statements, against the document and the code today

| # | Owner's statement | Document says | Code does today | Gap / proposal |
|---|---|---|---|---|
| A1 | **Each controller implements one controller interface**: `enable()`, `disable()`, `update()`/`step()`; prints every state change with timestamps | ARC-6 lists the method names in prose, ARC-7 the log line. No formal interface. | Three unrelated classes with **different signatures**: `Tower::enable()` (no args), `Compressor::enable(const Inputs&)`, `O2Controller::enable(uint32_t now)`; each calls a shared `TransitionLogger` itself. | New **CTL-1**: one abstract `Controller` (one level, no templates): `name()`, `enable(now)`, `disable(now)`, `update(const InputSnapshot&)`, `stateName()`, `enabled()`, `nextDeadlineMs()`, and the base class owns `transition()` which does the logging. `System` holds three `Controller*` and loops over them. Pick `update` or `step` (proposal: `update`, already used everywhere). |
| A2 | **No delays; `millis()` with unsigned math** | GOAL-6 forbids `delay()` after `setup()` **except POST and BIST**, and O2-3 lists the DFRobot library's `delay(10)` as an exception. | Controllers, drivers, POST/BIST, console are all deadline-based (the host tests prove it). The O2 library call still blocks ~10 ms. Tom's test sketch does the O2 query in plain Wire with a deadline-able wait. | Tighten GOAL-6: **no `delay()` anywhere after `setup()`, POST and BIST included**; the single exception is the O2 library read, unless we adopt our own query (finding R-3). |
| A3 | **All reads every `loop()` into an InputSnapshot**; all controllers read only from it | INP-1: inputs read once per pass into an `InputSnapshot` stamped with `millis()`. | `Inputs` (not yet renamed) is filled once per pass **but then mutated**: `o2CommOk`/`o2Warm` are written into it *after* `o2_.update()`, so tower/compressor see values computed in the same pass by another controller. A fast-vs-slow issue too: I2C reads (O2 query ≈ 10 ms, RTC ≈ 1 ms) cannot literally run on every pass without breaking NFR-1a. | Needs a decision (Q-A below): which reads are per-pass and which are on deadlines but published through the snapshot with an **age**. Rename `Inputs` → `InputSnapshot`; make it **const during a pass**. Cross-controller facts move to the OutputSnapshot of the *previous* pass (Q-B). |
| A4 | **Displays read from an OutputSnapshot**; snapshots have timestamps | DSP-1: "render from the output snapshot and fault state". The term is never defined. | `DisplayData` is built from `System` each pass; it carries **no timestamp**. `OutputRequest` / actual-outputs / controller states / faults live in different places. | New **SNP-1/SNP-2**: define `OutputSnapshot` = {stamp, controller state names, requested outputs, actual driven outputs, invariants mask, active faults, last fault code, N2 %, warm-up remaining}; built once per pass after the invariants and the OutputDriver; the only thing displays and the console `status`/`report` read. Both snapshots carry `ms` and a pass counter `seq`. Rename `DisplayData` → `OutputSnapshot`. |
| A5 | **Console logs carry correct wall time when an RTC is present, in addition to the `millis()` stamp used for scheduling** | LOG-1: `<L> <ms> <text>`. LOG-4: optional `time <iso>` in RAM. RTC-5/RTC-7 (misplaced at the end of the file) say the RTC stamps logs. | `StampedLog` prefixes a wall time to **every** line when synced, but only controller transitions and a few POST lines contain `millis()`; the format differs line to line. | New **LOG-5** with one fixed format, e.g. `<wall|-> <ms> <L> <text>`: wall = `YYYY-MM-DD HH:MM:SS` or `-` when not synced; ms always present. Scheduling never reads the wall clock (RTC-7, already there). |

## B. Text that is stale, contradicted or misplaced

| # | Where | Problem |
|---|---|---|
| R-1 | Header, §2 | Still says v2.6 and "M6 in progress"; `BENCH` build and the **Sim HAL (ARC-2)** were never built. The project now works in **stages** (`docs/Bringup_Stages.md`), which the document does not describe. |
| R-2 | ARC-1 | The HAL list is out of date: now also `i2cRead`, `i2cReadReg`, `i2cSetClock`, `i2cRecover`, `delayMicroseconds`, `consoleBegin`, `consoleCanDetectHost`, `readResetCause`. |
| R-3 | O2-3, GOAL-7 | The "unmodified library" rule produced the three-zero/`queryGasType` workaround. Tom's test shows a ~60-line own query (header, checksum, gas type, plausible value; read-only commands only; non-blocking) that is stricter and has no `delay(10)`. Needs an owner decision (Q-C). |
| R-4 | DSP-4 and its picture | Still shows the old layout (CMP ON, AIR first). Decided 2026-10-06/08: Option 1, **CMP only as `LO`/`HI`, otherwise `LF xx` (last fault code)**, TBS not shown. `LCD_Layouts.md` is right; the requirement is not. |
| R-5 | DSP-2, DSP-7 | "Write only when a field changed" conflicts with the bench finding that the LCD (and LED) must be **rewritten periodically** because they cannot be read back and can silently corrupt; DSP-7 must also say re-init of a running LCD is **forbidden**. |
| R-6 | DSP-3 | LED shows N2 % (production) but the bring-up builds show the time/uptime; say which LED content belongs to which build. |
| R-7 | §9 | Fault table lacks **F13 (RTC)**; codes are now **hex**; the canonical list is `docs/Fault_List.md` (a test keeps it in step with the code): the document should point at it and not duplicate it. `LF xx` on the LCD is not described. |
| R-8 | INP-6 | "TBS/TOB read without debounce (as V6)". The test sketch debounces 30 ms. Decide. |
| R-9 | INV list | INV-10 appears before INV-8; INV-7 text mentions them out of order. NFR-1a, NFR-9 sit between NFR-2 and NFR-3. RTC-7/8 and the RTC status paragraph are **after Appendix C**. |
| R-10 | §10, command table | Missing commands that exist: `time`, `time set`, `lcd`, `lcd reinit`, `lcd bus`, `i2c sweep`, `post`, `run/lcd/rtc/led/...`. `sim` (BENCH) was never built. |
| R-11 | §11, §12 | POST/BIST text describes the old monolithic BIST (12 numbered steps, console required, operator must answer p/f). The code now has a **DeviceCheck framework** (each device owns its POST check and BIST step) and Tom's package works **without a console** (automatic verdict, optional p/f, a late `p`/`f` answers the last question, auto-repeat). BIST-1/BIST-3 ("requires an attached console", "operator types a letter") contradict that. Decision Q-D. |
| R-12 | RST-6 | "RESET is not readable by software". Now: reset cause is read from the RA4M1 status registers (CWSF method), and a **RESET-button test** exists (marker in a spare DS3231 register). |
| R-13 | NFR-2, ENV-2/3 | Flash/RAM figures out of date; ENV-3 requires bundling libraries and an `env.lock`, while Tom's package is a single file with no libraries (and `env.lock` does not exist yet). |
| R-14 | RTC-1 | Year range 2000–2199 in the text; the helper is correct to 2136 (32-bit seconds) — immaterial but inconsistent. |
| R-15 | §16, §17 | Q-numbers resolved this week are not recorded (LCD address, O2 zero, per-board pins); HQ7, HQ8, HQ6 still open. |

## C. Requirements the bring-up work discovered that are not in the document (to add)

| # | Finding | Proposed ID |
|---|---|---|
| C1 | **I2C address map and the TM1650 alias clash.** The LED answers 0x24–0x27 and 0x34–0x37; the LCD is at **0x23** (A2 bridged); POST's LCD probe is meaningless at 0x27 when an LED is present. Compile-time check of the map. | PIN-13 (+ BoardPins) |
| C2 | **I2C robustness.** Bus recovery (9 clocks + STOP + `setClock`) at boot, on LCD retry, when the RTC is silent; only 100/400 kHz exist on the R4; `Wire.end()` is followed by `setClock()`; a refused write can wedge the controller. | I2C-1…I2C-4 (answers WDT-4) |
| C3 | **LCD behavior.** 2.5 s start delay; never re-init while running; self-healing rewrites; no readback; LED refresh ≥ 4 Hz. | DRV-4…DRV-6 |
| C4 | **Self-test framework.** Per-device POST+BIST (`DeviceCheck`), automatic verdict by I2C errors, optional operator p/f with late answers, auto-repeat, results visible on LCD/LED without a console. | POST/BIST rewrite (R-11) |
| C5 | **O2 zero = fault** (O2-3a, already added) and **read-only query** rule: never send mode/address/threshold commands. | O2-3b |
| C6 | **Reset-cause method** (CWSF), RAM record at a fixed address, RESET-button test. | RST-9 |
| C7 | **Last fault on the LCD (`LF xx`), hex fault codes**, fault list as a generated document. | FLT-5 |
| C8 | **Staged bring-up** as the delivery method, and the **site test package** (single file, no libraries, README + expected results, no scripts on the receiving side). | ENV-8 |
| C9 | **Per-board signal tables** and one LCD-address setting. | PIN-2 (amend) |
| C10 | Console banner behavior per board (Minima prints on connect; WiFi repeats a few times). | CON-8 |

## D. Questions for the owner (answers recorded in `Requirements.md` when settled)

* **Q-A** Which inputs are read on every pass? (Proposal: GPIO and ADC every pass; I2C devices on their own deadlines, published in the snapshot with an age.)
* **Q-B** How do controllers see each other's state (`o2Warm`, `o2CommOk`)? (Proposal: through the previous pass's OutputSnapshot, copied into the InputSnapshot at the start of the pass.)
* **Q-C** Own O2 query in production (GOAL-7 amended) or keep the library?
* **Q-D** Is a console required for BIST in production, or optional with an automatic verdict as in Tom's package?
* **Q-E** Log line format (A5) and whether sub-second wall time is wanted.
* **Q-F** Controller interface method names and whether `Controller` is a base class with virtual methods (one level) — consistent with "flat, no templates"?

## E. Decisions so far (owner, 2026-10-08)

| Q | Decision | Consequence for `Requirements.md` |
|---|---|---|
| Q-A | **Everything fast is read on every pass; slow I2C devices (O2 sensor, RTC) are read by a non-blocking background step** (start on a deadline, finish over several passes, publish when complete). | INP-1 amended: GPIO and ADC every pass into the `InputSnapshot`; each I2C device has a read state machine whose latest value and **age (ms)** appear in the snapshot. New INP-9. |
| Q-B | **Controllers learn about each other through the previous pass's `OutputSnapshot`**, copied into the `InputSnapshot` at the start of the pass. | CTL-2: a controller reads only the `InputSnapshot`; the `InputSnapshot` is `const` during a pass; one pass of latency is accepted; invariants run every pass on the final outputs. |
| Q-C | **Both O2 read paths, selectable at compile time** (library, or our own non-blocking read-only query). | GOAL-7 amended; new `N2_O2_DRIVER` build macro; a comparison on the same hardware (Tom's run provides the first real-sensor data). |
| Q-D | **Final:** a normal boot runs **nothing** (outputs safe, reset cause logged); **TOB held at power-up/reset = POST mode** (LCD and LED show it); **BIST only on the console command `bist`** (TBS off). | POST-1, POST-2, POST-4, DSP-11, RST-6, BIST-1 rewritten. |
| Q-E | **`[wall | ]ms L text`**: wall time and a delimiter ` | ` only when the RTC is present and trusted; nothing (no placeholder) otherwise. | LOG-1, LOG-5, ARC-7. |
| Q-F | **Abstract base class `Controller`, method `update()`.** | CTL-1…CTL-5. |


`Requirements.md` is now **v3.0 DRAFT** (106 lines added, 42 changed). Still open: INP-6 (debounce TBS/TOB or not, Q34), and the exact POST-mode indication (DSP-11 is marked a proposal).

## F. Code changes the new requirements imply (backlog, test first)

| # | Change | Requirements | Today |
|---|---|---|---|
| F1 | `Controller` abstract base class; Tower, Compressor, O2Controller derive; the base owns `transition()` and the transition log line | CTL-1…CTL-4, ARC-7 | three classes with different `enable()`/`disable()` signatures, each logging via `TransitionLogger` |
| F2 | Rename `Inputs` → `InputSnapshot`; add `seq`; make it `const` during the pass; remove the in-pass writes of `o2CommOk`/`o2Warm` | SNP-1, CTL-2 | `System::step()` mutates `in_` after `o2_.update()` |
| F3 | `OutputSnapshot` (rename/extend `DisplayData`): stamp, `seq`, requested and actual outputs, invariants mask, last fault code; displays and console read only it; previous pass's copy feeds the next `InputSnapshot` | SNP-2, SNP-3, CTL-2 | `DisplayData` has no stamp; the actual outputs live in `OutputDriver` |
| F4 | Background read steps for the O2 sensor and the RTC with an age in the snapshot | INP-9 | O2 read inside the controller with a deadline; RTC read by `Bringup`/`App` ad hoc |
| F5 | `N2_O2_DRIVER` (`OWN`/`LIBRARY`): non-blocking own query next to the library adapter; zero reading = fault in both | O2-3a, O2-3b, GOAL-6/7 | library adapter only (3-zeros workaround) |
| F6 | One log line format with the optional wall prefix; every `logf` goes through it; transition text `<NAME> <from>-><to> +<delta> next:<d>` | LOG-1, LOG-5, ARC-7 | `StampedLog` prefixes the wall time; ms appears only in some lines |
| F7 | Normal boot runs nothing; TOB sampled at the start of `setup()` selects POST mode; `bist` only from the console; POST-mode indication on LCD/LED | POST-1, BIST-1, RST-6, DSP-11 | `App` runs POST at every boot; TOB at power-up selects BIST |
| F8 | `FaultId` F13 in the docs table (done in the code), `LF xx` on the LCD (done) | FLT-5, DSP-4 | done |
| F9 | The full `App` adopts the `DeviceCheck`/`SelfTest` framework (replacing the monolithic POST/BIST) and the bring-up `Bringup` app converges with `App` | CHK-1…CHK-3, STG-1 | two parallel implementations |
| F10 | Check `delay()` is absent: grep + host test that fails if `delay(` appears under `N2V8/src` (except the library adapter) | GOAL-6 | true today in `src/`; not enforced |

## G. Decisions of the second round (owner, 2026-10-08)

| Item | Decision | Where |
|---|---|---|
| Debounce (INP-6, Q34) | **Characterize, do not guess**: measure the bounce on the R4 WiFi bench and on the Minima production panel in a BIST step; store the recommended debounce in NVM; also change the compiled default afterwards. | INP-6, INP-10, NVM-1 |
| POST-mode indication (DSP-11) | **LCD shows steps, progress and results; LED shows the LED test as the interim display, then 0000 after a pass and FFFF after a failure.** | DSP-11, POST-5 |
| Table of contents | **In the real firmware sketch `N2V8/N2V8.ino`** (not Tom's test, which has shipped as v1.6 and is unchanged): `file:line` entries, brief (ends at line 44, limit 50), common edits are **pressure thresholds and timing values**, plus a comment with the command that refreshes it. | `deliverables/update_sketch_toc.py` |

Backlog additions: **F11** bounce BIST step + NVM block + `kDefaultDebounceMs` (INP-10, NVM-1); **F12** done: `N2V8.ino` table of contents.
