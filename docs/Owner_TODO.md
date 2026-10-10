# Owner TODO list

Things only you can do or decide, kept in two separate parts. IDs link to `Requirements.md`.

**Priority note:** the pinout and the high-N2 sensor location are of **critical importance but low urgency**.
`BoardPins.h` is one table, so wrong values can be corrected at any time and do not block development;
they must be right before the first hardware visit (DIAG).

# Part 1 — HARDWARE (production, wiring, bench)

## 1A. Open
1. [x] **HQ7 — RESOLVED 2026-10-09 on the production panel (owner): A0 = air, A1 = N2 low, A2 = N2 high, A3 unused.** `BoardPins.h` updated (kMinimaSignals). [was: which analog pin carries the high-N2 sensor? V6/V7 say A5, but A4/A5 are the I2C lines] — which analog pin carries the high-N2 sensor?** V6/V7 say A5, but A4/A5 are the I2C lines; `BoardPins.h` has **A1 as a placeholder**. Edit that one row when known. (PIN-10)
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
10a. [ ] **Flash the DIAG build** (Arduino IDE: open `N2V8/N2V8.ino`, board *UNO R4 WiFi*, Upload; the default build is DIAG, which runs no controllers). Expect: LCD banner, POST (the O2 sensor and pressure sensors will be missing, so POST will report faults and **hold until you press TOB**), then normal screens. Open the Serial Monitor and type `help`, `status`, `report`, `bist`. Copy the output into `docs/results/`.
10b. [ ] Install the DFRobot library in the IDE's sketchbook (it is already in `Documents/Zephyr/Arduino/libraries`); without it the O2 sensor reads as absent.
11. [x] **Run the reset probe on R4 WiFi** (`experiments/reset_probe/README.md`, T1–T9) — **DONE 2026-10-06** (`docs/results/reset-probe-wifi-20261006.md`). Still open: repeat on the **Minima**. (O2-6a/6b, CON-3)
12. [ ] I2C scan: confirm 0x24 (LED) and 0x27 (LCD). **Partial (2026-10-06):** LCD at 0x27 and RTC seen on the WiFi bench; LED at 0x24 waiting on a replacement TM1650 module. Optional early experiment: scan, `analogRead(A5)`, scan again. (PIN-10)

## 1B-bis. Bench results so far (R4 WiFi)
- [x] Reset experiment complete: cold/warm flag + plain-RAM record verified (docs/results/reset-probe-wifi-20261006.md).
- [x] **LCD driver validated** on hardware (docs/results/lcd-test-wifi-20261006.md).
- [x] **RTC driver validated** on hardware (docs/results/rtc-test-wifi-20261006.md); now integrated in the firmware (`time`, `time set`, banner, F13). Check the RTC module's charging circuit before relying on a CR2032 (RTC-6).
- [ ] **Stage 1 on the bench** (`stages/stage1/README.md`): wire LCD + RTC on one I2C bus (pull-ups: see README), upload, `help`, `status`, `scan`, `bist`; copy output to `docs/results/`.
- [x] **LCD/LED address clash (found 2026-10-07, fixed 2026-10-08):** the TM1650 LED module also answers at 0x24-0x27, so an LCD at 0x27 on the same bus failed every character write
  (`docs/results/lcd-led-address-clash-20261007.md`). Fix: **A2 solder pad bridged on the LCD backpack = address 0x23**, now the permanent default (`kLcdAddress`). Verified on the bench with
  LCD, RTC and LED on one bus: BIST 4 pass / 0 fail. **Still to do: Tom bridges A2 on his identical backpack** before his unit runs this firmware (his LCD stays dark at 0x23 until then).
  The software-I2C LED driver (D2/D3) stays in the tree as an unused, tested fallback.

- [x] **LED driver (TM1650) validated** on hardware (docs/results/led-test-wifi-20261007.md).
- [ ] **LED driver test** waits for a replacement TM1650 module (the first one was damaged). Before powering the new one: check the power pins with a meter, confirm 5 V vs 3.3 V, and test the bare Arduino first.
- [ ] Run the same reset probe on the Minima (at Tom's, or any Minima): the probe builds for it.

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

## 2C. Next site visit (owner 2026-10-09 evening / 2026-10-10)
- [ ] **BIST FLUSH (O2 flush valve) step: rewrite.** The flush valve needs N2-low pressure (owner's guess: at least 10 PSI; the true minimum is unknown) to work, so toggling it on a dead N2-low proves nothing. New step: check the air supply (minimum air for opening a tower), open a tower and run a fill cycle until N2-low >= 10 PSI (limit the time; abort on any BIST-11 veto), then toggle FLUSH and report N2-low before/after; record the minimum N2-low at which the owner hears/feels it work. Needs a requirement text first (BIST-xx), then tests.
- [x] **LCD `LRFS` -> `LROC`** done 2026-10-10: O = O2 flush valve open (the old F bit), C = compressor SSR on (the old S bit); label on the LCD, recorder header, docs, tests. The console `OUT L= R= F= S=` line keeps its letters.
- [ ] **Watchdog stalls on valve steps (3 times, 2026-10-09 at Tom's)**: board freezes (watchdog off) or resets ER30 (watchdog on) during valve steps/tests. Suspect: LCD updated every 100 ms during valve steps + solenoid EMI -> I2C stall. Candidate fixes: slower LCD refresh in valve steps, I2C timeout/stuck-bus recovery. Not yet confirmed.
- [ ] BIST keys added on site (8.1.25, DIAG tools): `t` timed sag, `b` BOTH, `o` 750 ms overlap, `x` event overlap; measured data in `docs/results/*_20261009.csv/.svg`. BIST sensor-out-of-range now needs 50 ms; N2-high ignored in `o`/`x`.
- [ ] Open from the visit: SSR step not run; RIGHT recorded PASS by owner verdict (no ear confirmation in the log); FLUSH answer pending; the N2-low sensor sits at ~0.43 V at zero (below 0.5 V).
