# Owner TODO list

Things only you can do or decide. Tick them off as you go. IDs link to `Requirements.md`.

**Priority note:** the pinout table and the high-N2 sensor location are of **critical importance but
low urgency**. `BoardPins.h` is a single table, so wrong values can be corrected at any time and do
not block development; they must be right before the first hardware visit (DIAG).

## A. Open items

1. [ ] **Find out which analog pin the high-N2 sensor is really on.** V6/V7 say A5, but **A4/A5 are the I2C lines**, so `BoardPins.h` carries **A1 as a provisional placeholder** (V5 is ignored). Edit that one table row when you know. (PIN-10, Q1)
2. [ ] **Confirm the I2C pins on production**: A4/A5 (core pins 18/19) on the Minima. You reported D18/D19 on the WiFi; the core lists both boards as 18/19 = A4/A5. (PIN-11)
3. [ ] **Verify the pinout and active levels against the real plant.** V6 and V7 agree on all pins, levels and addresses (adopted); V5 differs for TBS, flush valve and N2-high, and calls the outputs `Relay_1…4` (are they relay modules, and is the active level as in V6/V7?). Flush valve D11 (V6/V7) or D12 (V5)? (PIN-7, Appendix C)
4. [ ] **Identify the plant gauges** you can read by eye (HQ4: TBD). (INP-8)
5. [ ] **Pressure and timing thresholds are all `TODO: tune` in V6.** Decide which values are final before field use. (§7)
6. [ ] **Consider an independent hardware over-pressure safeguard.** The hardware team says there is none, so the firmware is the only protection (GOAL-11). This is a plant decision, not a software one.
7. [ ] **Hardware team: HQ6** — should the plant run while the O2 sensor warms up (5 min)? Interim answer: no, tower held off (INV-10). Also HQ5: does the O2 sensor share the Arduino's supply?
8. [ ] Does the O2 sensor share the Arduino's power supply? (HQ5, needed for O2-6a)
9. [ ] Review `Requirements.md` v2.3 and say what to change.
10. [ ] Bring a laptop with the IDE, core 1.6.0 and the libraries, plus the known-good firmware, on the field trip.

## B. Decided (kept for reference)

- [x] Active levels, pins, addresses: use V6/V7 (they agree); verify on site.
- [x] Output hold: 1000 ms; BIST has a 2 Hz exception (OUT-1).
- [x] LCD shows TBS `ON`/`OFF` if there is room (DSP-9).
- [x] F04 (N2-low above N2-high) is an INHIBIT fault (INV-8).
- [x] O2 sensor is **mandatory in production**; missing means all outputs off and POST holds (INV-9, O2-1).
- [x] O2 warm-up is 5 minutes (O2-6); output wiring assumed safe (OFF) during reset (HQ3).
- [x] HQ1: O2 cycle may run when towers are idle if the N2 thresholds are met (O2-7).
- [x] BIST keys: `p` pass, `f` fail, `r` rerun, `s` skip, `q` quit (BIST-3).
- [x] Capture: copy/paste from the IDE Serial Monitor (LOG-3).
- [x] R4 compiler: Rosetta installed; N2V7 builds for both boards (Project_Plan §5).
- [x] Git: branch `v8` on `egp/N2` over SSH (Project_Plan §9).
- [x] Design priorities: testability, readability, maintainability (GOAL-10).

## C. At the bench (home, R4 WiFi)

11. [ ] Optional early experiment: with only the LCD and LED attached, run an I2C scan, `analogRead(A5)`, scan again, to see whether A5 and I2C coexist. (PIN-10)
12. [ ] Confirm the LCD and LED I2C addresses (0x27, 0x24) with a scan.
13. [ ] **Measure the reset-to-outputs-safe window** (RST-2): the core initializes and starts USB before `setup()`. A scope or logic analyzer on one output pin, or the hardware team's confirmation that outputs default OFF in reset (HQ3), settles whether the gap matters.
14. [ ] Check reset-cause visibility: read `RSTSR0` after power-on vs the reset button, and `.noinit` RAM retention (O2-6a).

## D. On site (DIAG visit)

15. [ ] Run BIST with the console attached; answer p/f/r/s/q for each step; copy the Serial Monitor text to a file.
16. [ ] Read the plant gauges next to BIST step 5 and note them.
17. [ ] Disconnect one sensor (with the system disabled) and record the raw values, to calibrate the fault window. (INP-4)
