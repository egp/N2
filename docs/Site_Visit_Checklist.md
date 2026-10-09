# Site visit checklist — Tom's garage (Minima, production)

Plan agreed 2026-10-08: the visit is about **POST and BIST and verifying every pin**. The production V8 (controllers on) goes on the machine only if every GO gate below is met. Anything not green = stay in DIAG.

## Bring
* Laptop with Arduino IDE 2, board package **Arduino UNO R4 Boards 1.6.0**, the **DFRobot_MultiGasSensor** library, and this repo (branch v8, commit noted on the cover of the build folder).
* USB cable. A pen. A multimeter. A small screwdriver for the A2 pad check, if needed.
* Everything goes to the board with the IDE's **Upload** (it compiles and uploads; there are no prebuilt files). Open the sketch, pick the board (**Arduino UNO R4 Minima**) and the port:
  * DIAG (controllers OFF, the default): `N2V8/N2V8/N2V8.ino` as it is.
  * BENCH / FIELD: edit the one word at `N2V8/src/BuildConfig.h:22` (`#define N2_BUILD_DIAG` -> `N2_BUILD_BENCH` or `N2_BUILD_FIELD`), Upload, and change it back to DIAG afterwards.
  * The bounce and NVM probe: `experiments/nvm_probe/nvm_probe.ino` (its `src` link must be present).
* The shipped v1.6 `tom_i2c_check` as the known-good fallback.

## Source on Tom's laptop (Windows): git, no SSH needed (public repo)
* `main` now equals the tested `v8` (2026-10-09, 57eba9f; the old main stays in git history). Everything changed ON SITE goes on the branch **site-20261009**, never on main.
* One time, in a command prompt (Git for Windows installed): `git clone -b site-20261009 https://github.com/egp/N2.git C:\N2`  ->  the sketch is `C:\N2\N2V8\N2V8.ino`.
* To get the owner's latest changes: `cd C:\N2` then `git pull`. Install the `DFRobot_MultiGasSensor` library (v3.0.0) by ZIP: Sketch > Include Library > Add .ZIP Library... > `DFRobot_MultiGasSensor.zip` (from `deliverables/site_kit/` on the owner's Mac, or the GitHub Download ZIP of github.com/DFRobot/DFRobot_MultiGasSensor). It is NOT in the Library Manager index and the IDE cannot update it. To check the installed version, open `Documents\Arduino\libraries\DFRobot_MultiGasSensor\library.properties` in Notepad and read the `version=` line (3.0.0 on GitHub master, 2026-10-09; the repo has no releases or tags).
* After the visit: review the branch, then merge it into main.

## Rules
* **Power off and air off** before touching wiring. Tom's Arduino has its own supply: unplugging USB does not remove power.
* DIAG never drives an output by itself. Only a BIST step you confirm switches a valve or the SSR, and the BIST turns them off again.
* Anything you change on site: write it down (pin, old value, new value) and put it in `BoardPins.h` ("BoardPins.h:92" in the sketch's WHERE TO EDIT table).

## Order
1. **I2C parts first** (known good v1.6, or DIAG `scan`): LCD 0x23, RTC 0x68, LED 0x24..., O2 0x74. Photograph the screen.
2. **Upload DIAG** (as it is). Console 115200. Type `ver`, `nvm` (expect: nothing stored on a new board), `pins`.
3. **`pins`: verify every pin** (below).
4. **POST:** hold TOB, press RESET, keep TOB held until the LED shows `PoSt`, release. Expect POST to end with `0000` for 10 s (or `FFFF` and a held LCD naming the fault).
5. **BIST** (`bist`, TBS OFF; on the Minima it is answered from the console: the TOB/TBS answer keys are for the WiFi bench only; `q` aborts; a RESET-button reset in the middle resumes it at the same step): banner, TBS and TOB, I2C scan, LED, LCD, pressures (compare with the gauges: `g air 120.5`, `g n2l 10.0`, `g n2h 95.0`), O2 sensor, LEFT, RIGHT, FLUSH valves, SSR. Answer `p`/`f` (any case). Save the whole console log.
6. **`loop`** after a few minutes in RUN (min, mean, median, max) for NFR-1.
7. **Bounce on the production switches:** upload `nvm_probe`, `m 1`, `c 30`, `b`; TBS (rotary) 30 cycles, then TOB 30 cycles. Then `r`. Do not store a value until you have decided (`w TBS TOB`).
7b. **Run mode without recompiling:** the build you upload starts in DIAG (controllers off). `mode` shows it; `mode field confirm` (TBS OFF, console only) switches to production without a re-upload; `mode diag` goes back at once with every output off. The mode survives a RESET-button or watchdog reset, not a power cycle (a power cycle returns to DIAG). Nothing starts until TBS is switched ON.
7c. **Record real data for the simulator (owner 2026-10-09).** Captures are always printed as `R,ms,air,n2low,n2high,tbs,tob,LRFS,n2pct,tower,compressor,o2,why` (raw ADC counts; `why` says what triggered it).
    * AUTOMATIC, always on: every SSR change (`ssr+`/`ssr-`), every TBS change (`tbs+`/`tbs-`), every TOB press (`tob`: data only, TOB is unused in normal operation, so press it as a free marker).
    * EXPLICIT: `cap air almost ready` prints a capture now (`cap`) with your comment as a landmark; `note text` writes only a comment.
    * OPTIONAL periodic: `rec on` (1 s; `rec on 200` for fast changes), `rec off`. All read-only, any mode.
    * Keep the console log, then `python3 tools/rec_to_csv.py LOGFILE` makes a CSV with PSI columns, the reason, and your notes. Plain Notepad pastes work too.
8. Decide GO / NO-GO for the production V8 (gates below).

## `pins` — verifying every pin (do this with the unit powered, outputs off)
`pins` prints one line per pin D0..D13, A0..A5: signal name, direction, live level (and ON/off), analog raw and volts. It reads only.
* **TBS (D0), TOB (D1):** operate the switch, type `pins` again: the line must change `HIGH/off` ⇄ `LOW/ON`. If nothing changes, find the pin: look at **every** D line while operating it.
* **AIR (A0), N2 LOW (A3), N2 HIGH:** with air or N2 pressure applied, the volts must move; compare to the gauge (0.5 V = 0, 4.5 V = full scale). A sensor on an unassigned pin shows on a line marked `(unassigned)`: that is its pin.
* **N2 HIGH is the open question (HQ7):** V6/V7 say A5, but A5 is SCL. `BoardPins.h` has **A1** as a placeholder. Find where the wire is: if A1 moves with pressure, keep it; otherwise edit that one table row (kMinimaSignals, "N2HIGH").
* **Valves and SSR (D4 LEFT, D7 RIGHT, D11 FLUSH, D8 SSR):** the BIST steps switch each one; listen/feel for the click and watch the `pins` line go HIGH/ON. Wrong valve moves = swap the pin in the table.
* **LCD rule learned on the bench (2026-10-08):** never send the LCD anything but text and cursor positions after its initialisation (no backlight, display on/off, entry mode, return-home commands): those, not the hardware, put it into a blank/bars state. If the LCD shows two bars or nothing, power-cycle the board and tell me; do not expect software to recover it.
* **Edit on site:** open `BoardPins.h` (Ctrl+L "go to line" at the line shown in the sketch's WHERE TO EDIT table), change the pin, recompile with the **Minima** selected, upload, run `pins` and the same BIST step again. Record every change.

## GO / NO-GO for the production V8 (the FIELD build, controllers on)
ALL must be true:
1. Every signal in `pins` matches the wiring: TBS, TOB, three valves, SSR, three pressure sensors (including N2 HIGH's real pin).
2. The BIST pressures step: all three sensors within the gauge tolerance you set on site, no sensor fault (F01..F03).
3. Every valve and the SSR confirmed at their own BIST step (`p`), and all OFF afterwards.
4. The O2 sensor answers (BIST O2 step) with a plausible reading (about 20.9 % in air). FIELD requires it (INV-9).
5. POST ends `0000` after a clean boot.
6. Tom (or you) is present for the first run, an air supply cut-off is within reach, and the console is attached for the first ten minutes.
7. You can go back to DIAG with one edit and Upload (BuildConfig.h:22).

Not GO if any of: a pin you could not confirm; a sensor you could not compare with a gauge; unexpected valve behaviour; the LCD or LED misbehaving (they are the operator's view of faults); anything you cannot explain.

First production run: TBS ON, watch `status` and the log (level `info` in FIELD; `log debug` for more), watch pressures against the gauges, stop (TBS OFF) at the first surprise. GOAL-11: the firmware is the only protection against over-pressure, and the invariants have not had the independent review the requirements ask for. Run attended.

## What to bring back
Console logs of POST, BIST, `nvm`, `pins`, `loop`; the bounce numbers; any pin or setting you changed; photos of the LCD and LED.
