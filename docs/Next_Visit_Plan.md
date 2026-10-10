# Next visit to Tom's (date TBD): plan and home preparation

Written 2026-10-10 after the 2026-10-09 visit (8.1.27 on the branch, 8.1.25 on Tom's Minima, DIAG, watchdog off).
Goal of the next visit: **more progress per hour at Tom's.** Everything below that can be done, tested or scripted at home is done at home first.

## A. At home (WiFi bench: RTC, LED, LCD only; the host test suite is the main tool)

| # | Item | Why | Done when |
|---|---|---|---|
| A1 | **Stall hardening** (3 freezes/resets on valve steps) | The board froze with the watchdog off and reset (ER30) with it on. Suspect: LCD written every 100 ms in valve steps next to switching solenoids -> I2C wait. | LCD refresh in BIST valve steps <= 4/s; I2C calls bounded by a timeout with bus recovery; a log line (and a fault) on a recovered stall; host tests with a fake bus that stalls; watchdog back ON in every build. Test on the bench by hammering the LCD+LED bus. |
| A2 | **FLUSH step rewrite** (needs N2-low >= 10 PSI) | Old step toggled a valve that cannot work without N2-low pressure. | Requirement text (BIST-xx) in Requirements.md; the step: air check, tower fill cycle, N2-low >= 10 PSI (time limit, BIST-11 vetoes), toggle FLUSH, report N2-low before/after; host tests with a simulated N2-low rise. |
| A3 | **SSR step review** | Never run on site; it pulses the compressor 1 s. | Preconditions (N2-high below the start threshold, N2-low not too low) already in vetoReason: confirm with tests; the owner's OK prompt; abort paths. |
| A4 | **Sensor mismatch tooling** (N2 LOW gauge ~17 PSI vs sensor 0.0 PSI) | Settle in minutes on site whether the transducer is the problem. | A `sensors` console command: for each of air/N2L/N2H the raw, volts, PSI, the valid window and distance to its edges, with a one-line verdict; and `g <sensor> <psi>` pairs kept in one block for the log. |
| A5 | **Recorder / capture polish** | Data for the future simulator. | `LROC` header done; `cap` with a tower-state summary; a one-line `markers` list in the CSV tool. |
| A6 | **Visit script** | Less typing, same steps every time. | `tools/visit_run.py`: a checked list of steps with the expected console lines, owner "go" gates, a log per step. |

## B. At Tom's (ordered; each step has a stop criterion)

1. Upload the current build (double-tap RESET if the upload fails). `ver`, `scan`, `faults`, `pins`.
2. **Sensor truth first** (A4): gauge vs `sensors` for N2 LOW (and air, N2 HIGH when pressure is up). Fix wiring/transducer BEFORE any test that depends on N2-low.
3. BIST through O2 (known good), LEFT, RIGHT (record real verdicts in the log).
4. **FLUSH** with the new step.
5. **SSR** with the owner's explicit OK.
6. Controlled first run in BENCH, attended, console attached; then decide GO/NO-GO for FIELD with the checklist gates.
7. Collect: N2-low and N2-high curves during a few tower cycles (`rec on 200`), the compressor-on transient, and the sag/overlap data again at the production pressure.

## C. Open questions for the owner (answers shape A1-A3)
* Is the small solenoid on the compressor side the unload/dump valve? Does it need to be driven by the Arduino at all?
* Which output drives the pilot air regulator's solenoid next to the AIR filter (left of the towers)?
* The minimum N2-low at which the flush valve works (to be measured on site by the new FLUSH step).
