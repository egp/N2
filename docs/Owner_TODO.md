# Owner TODO list

Things only you can do or decide. Tick them off as you go. IDs link to `Requirements.md`.

## A. Before any firmware code (M1)

1. [ ] **Complete the pinout table** for production (Minima) and bench (WiFi), for **every** signal: pin, direction, pull (none / pull-up), **active level (high or low)**, wiring note. (PIN-1, PIN-6, PIN-7)
2. [ ] **Active level for every binary input and output**: TBS, TOB, LEFT valve, RIGHT valve, flush valve, SSR. V7 assumes TBS/TOB active LOW and outputs active HIGH; confirm.
3. [ ] **Resolve the A5 conflict**: the high-N2 sensor is on A5, which is I2C SCL. Choose another analog pin (A1 or A2 are free) or show that it works. (PIN-10, Q1)
4. [ ] **Confirm the I2C pins**: you say D18/D19 on the WiFi, A4/A5 on the Minima. The core lists both as 18/19 = A4/A5. Confirm what is physically wired on production. (PIN-11)
5. [ ] **Identify the plant gauges** that can be read by eye: air supply, and which others — left/right tower pressure, or N2 low/high? (INP-8, HQ4)
6. [ ] **O2 sensor warm-up time**: look up the SEN0465 value and whether its supply is shared with the Arduino. (O2-6, HQ5)
7. [ ] **Output pull-downs**: confirm the valve and SSR drivers default OFF during reset or when the pin floats. If not, plan a resistor. (RST-8, Q14, HQ3)
8. [ ] **Minimum output hold time**: pick 500 ms or 1000 ms. Note the BIST 2 Hz toggle is 500 ms per half-cycle, so a 1000 ms hold needs the documented BIST bypass. (OUT-1, Q6)
9. [ ] **TBS and the displays**: besides blanking the LED, should TBS change anything on the LCD? (Q17)
10. [ ] **F04 (N2-low above N2-high)**: warn only, or inhibit outputs? (Q18)
11. [ ] **POST hang policy**: should a missing O2 sensor alone hold a headless restart until someone presses TOB? (POST-4, Q20)
12. [ ] Review `Requirements.md` v2.1 and say which items you want changed.

## B. Hardware-team questions (§17 of the requirements)

13. [ ] **HQ1** — Can the O2 sample cycle run when the towers are not cycling? Is there flow past the sensor then?
14. [ ] **HQ2** — Is there a mechanical or hardware safeguard independent of the Arduino (relief valve, pressure switch, power cut)?
15. [ ] HQ3, HQ4, HQ5 as in §17.

## C. Environment

16. [ ] **R4 compiler on this Mac**: Rosetta is not installed, so neither `arduino-cli` nor the Arduino IDE can build for the R4. Choose: install Rosetta (`softwareupdate --install-rosetta --agree-to-license`, needs admin rights), or rely on GitHub Actions for board builds. (Project_Plan §5)
17. [ ] **Approve the git plan**: branch `v8` in `egp/N2`, and the first push. (Project_Plan §9)
18. [ ] Decide which capture tool you prefer (Python `pyserial` script recommended). (Q19)
19. [ ] Bring a laptop with the IDE, the same core version (1.6.0) and the libraries, plus the known-good firmware, on the field trip.

## D. At the bench (home, R4 WiFi)

20. [ ] Optional early experiment: with only the LCD and LED attached, run an I2C scan, `analogRead(A5)`, scan again, to see whether A5 and I2C coexist. (PIN-10)
21. [ ] Confirm the LCD and LED I2C addresses (0x27, 0x24) with a scan.

## E. On site (DIAG visit)

22. [ ] Run BIST with the console attached; answer y/n for each step; save the capture file.
23. [ ] Read the plant gauges next to BIST step 5 and note them.
24. [ ] Disconnect one sensor (with the system disabled) and record the raw values, to calibrate the fault window. (INP-4)
