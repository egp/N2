# Owner TODO list

Things only you can do or decide, kept in two separate parts. IDs link to `Requirements.md`.

**Priority note:** the pinout and the high-N2 sensor location are of **critical importance but low urgency**.
`BoardPins.h` is one table, so wrong values can be corrected at any time and do not block development;
they must be right before the first hardware visit (DIAG).

# Part 1 — HARDWARE (gen, wiring, bench)

## 1A. Open
1. [ ] **HQ7 — which analog pin carries the high-N2 sensor?** V6/V7 say A5, but A4/A5 are the I2C lines; `BoardPins.h` has **A1 as a placeholder**. Edit that one row when known. (PIN-10)
2. [ ] **Confirm the I2C pins on production**: A4/A5 (core pins 18/19) on the Minima. You reported D18/D19 on the WiFi; the core lists both as 18/19. (PIN-11)
3. [ ] **Verify pins and active levels against production.** V6 and V7 agree (adopted): TBS D0 and TOB D1 pull-up, active LOW; LEFT D4, RIGHT D7, FLUSH D11, SSR D8 active HIGH; AIR A0, N2-low A3. (PIN-7)
4. [ ] **HQ4 — which production gauges can be read by eye?** Needed only in the diagnostic phase to check the sensors. (INP-8)
5. [ ] **HQ5 — does the O2 sensor share the Arduino's supply?** (O2-6a)
6. [ ] **HQ6 — should the system run while the O2 sensor warms up (5 min)?** Interim answer: no, tower held off (INV-10).
7. [ ] **HQ8 — is a 2 Hz toggle of the compressor SSR acceptable** in the BIST, or one short pulse?
8. [ ] **Consider an independent hardware over-pressure safeguard** (HQ2: none exists, so the firmware is the only protection, GOAL-11). A system decision.
9. [ ] Pressure and timing thresholds were all `TODO: tune` in V6. Decide which are final before field use. (§7)
10. [ ] Measure the **reset-to-outputs-safe window** (RST-2) with a scope, or get the hardware team's confirmation that outputs default OFF in reset (HQ3).

## 1B. At the bench (R4 WiFi, LCD + LED; rig a TBS/TOB switch if needed)
10a. [ ] **Flash the DIAG build** (Arduino IDE: open `N2V8.ino`, board *UNO R4 WiFi*, Upload; the default build is DIAG, which runs no controllers). Expect: LCD banner, POST (the O2 sensor and pressure sensors will be missing, so POST will report faults and **hold until you press TOB**), then normal screens. Open the Serial Monitor and type `help`, `status`, `report`, `bist`. Copy the output into `docs/results/`.
10b. [ ] Install the DFRobot library in the IDE's sketchbook (it is already in `Documents/Zephyr/Arduino/libraries`); without it the O2 sensor reads as absent.
11. [ ] **Run the reset probe** (`experiments/reset_probe/README.md`, T1–T9) and paste the output into `docs/results/`. Repeat on the Minima. (O2-6a/6b, CON-3)
12. [ ] I2C scan: confirm 0x24 (LED) and 0x27 (LCD). Optional early experiment: scan, `analogRead(A5)`, scan again. (PIN-10)

## 1C. On site (DIAG visit, Windows laptop with the Arduino IDE)
13. [ ] Run the BIST with the console attached; answer p/f/r/s/q; copy the Serial Monitor text to a file.
14. [ ] Read the production gauges next to BIST step 5 and note them.
15. [ ] With the system disabled, disconnect one sensor and record the raw values to calibrate the fault window. (INP-4)
16. [ ] Bring: the zip package, the same board-package version (core 1.6.0), the known-good firmware.

# Part 2 — SOFTWARE

## 2A. Open
0. [ ] **Review what was built** (`git log`, `test/host/`, `docs/Requirements.md` v2.6). Newly marked **[found]** items: O2 library has no error signal (O2-3); the USB TX buffer is 256 bytes (CON-2); BIST vetoes exclude INV-9/10 (BIST-11a).
0a. [ ] **Q25** — POST does not hold after a watchdog/brown-out reset (so an unattended unit restarts). OK? **Q26** — BIST vetoes exclude the O2-sensor rules. OK?
1. [ ] **LCD layout:** Option 1 (clear labels) or Option 2 (compressor on the pressure line, `L`/`H` labels)? Fault cycle 3 s or 4 s? (`LCD_Layouts.md`)
2. [ ] Review `Requirements.md` v2.5 and say what to change.

## 2B. Decided
- [x] V6/V7 are the source where they agree (GOAL-12); the non-blocking deadline scheduler and the per-loop watchdog (4 s, reduced later) are kept.
- [x] BIST keys `p f r s q`; BIST never creates an unsafe condition (BIST-11); the hardware team decides which steps to run.
- [x] Capture by copy/paste from the Serial Monitor. Receiving laptop is Windows + Arduino IDE; **no scripts assumed there** (ENV-2).
- [x] O2 sensor mandatory in production; warm-up 5 min with belt-and-suspenders credit, enabled only after testing (O2-6a/6b).
- [x] Tower held off until the O2 sensor is warm (interim, INV-10).
- [x] F04 inhibits (INV-8); output hold 1000 ms with a BIST 2 Hz exception (OUT-1).
- [x] LCD: no TBS, N2% only, warm-up countdown replaces N2%, vertical LRFS, fault screen alternates (DSP-4/5/9).
- [x] Priorities: testability, readability, maintainability (GOAL-10). Host tests first, then the R4 WiFi.
