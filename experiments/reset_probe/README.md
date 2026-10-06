# reset_probe — what to do

**Purpose:** find out on real hardware (1) whether the power-on flag tells a power cycle from the
reset button, (2) whether a RAM record survives reset but not power loss, (3) whether opening the
Serial Monitor resets the board. These decide Requirements O2-6a/6b and CON-3.

**Safe and bare:** the sketch starts no devices (no LCD, no LED display, no I2C) and drives no pin except the board's own
built-in LED. All you need is the Arduino and its USB cable. Nothing needs to be wired.
Run it on the **UNO R4 WiFi** first; repeat on the Minima when convenient.

## Upload
Arduino IDE: open `experiments/reset_probe/reset_probe.ino`, board *Arduino UNO R4 WiFi*, Upload.
(Or ask Claude to flash it with `arduino-cli` once the board is plugged in.)

## Run (Serial Monitor open; the report prints each time a console attaches)

| # | Action | Hypothesis (what we expect) | Write down |
|---|---|---|---|
| T1 | Right after upload | RAM record invalid; cause likely software/other | the full report |
| T2 | Press the **reset button** once | cause *EXTERNAL / none flagged*; PORF=0; **record valid; credit > 0** | report |
| T3 | Press the reset button again | same as T2; boot number +1; credit grows | report |
| T4 | **Unplug USB**, wait 5 s, plug in, open Monitor | **PORF=1**; record **invalid**; credit 0 | report |
| T5 | Type `s` + Enter (software reset) | cause SOFTWARE; record valid; credit > 0 | report |
| T6 | Type `w` + Enter (watchdog reset) | cause WATCHDOG; record valid; credit > 0 | report |
| T7 | **Close** the Serial Monitor, wait 10 s, **reopen** | **boot number unchanged** (no reset on connect); "console attached" line | the attach line |
| T8 | Type `x` + Enter, then press reset | record **invalid** (checksum) | report |
| T9 | Press reset several times fast, then T4 again | PORF only after the unplug | reports |

Also note: do the "flags after clearing" values read 0? If not, the flags cannot be cleared this way and
the design must change (PORF would stay set until power-off).

## The built-in LED also reports the result (works even with no serial output)
After every boot the on-board LED repeats this pattern forever:

| Blinks first | Meaning |
|---|---|
| 1 | power-on reset |
| 2 | reset button / external reset pin (nothing flagged) |
| 3 | watchdog reset |
| 4 | software reset |
| 5 | voltage-monitor reset |

then a pause, then **one long (1 s) blink = warm-up credit > 0** (valid RAM record and not a power-on reset) or **one very short
blink = no credit**; then a longer pause and it starts again. For T2/T3 (reset button) expect 2 blinks then a long blink;
for T4 (unplug/replug) 1 blink then a short blink.

## The WiFi board's 12x8 LED matrix shows it too (R4 WiFi only)
Left: the cause as a digit (same numbers as the blink table). Right: **Y** = warm-up credit, **N** = no credit. The
bottom-right pixel blinks twice a second: if it blinks, `loop()` is running.
Two more pixels on the bottom row describe the serial link: **bottom-left lit = the board sees a console** (the host has the
port open with DTR raised); **third pixel from the left lit = a byte has arrived from the host**;
**fifth pixel lit = the core accepted bytes for sending**;
**eighth pixel lit = the RAM record's signature was still there at boot; tenth lit = its checksum matched too**
(counting pixels from the left, the first being number 1). For 2 s after every boot the matrix shows the sketch version
(e.g. `1` and `6` for version 1.6) (if lit but nothing reaches the PC, the USB transmit path is stuck). Nothing needs to be attached; the matrix is part
of the WiFi board. (The Minima has no matrix; it uses the built-in LED blinks only.)

## If the Serial Monitor goes quiet
Pressing reset or replugging the USB cable disconnects the port for a moment. If the IDE shows the monitor as disconnected,
re-select the board's port (or close and reopen the Serial Monitor); the report prints again each time the monitor attaches.
Typing `r` + Enter reprints the last boot report at any time.

## Return the results
Select all in the Serial Monitor, copy, paste into `docs/results/reset-probe-wifi-YYYYMMDD.txt`,
and tell Claude. The conclusions go into Requirements O2-6a/6b and CON-3.
