# reset_probe — what to do

**Purpose:** find out on real hardware (1) whether the power-on flag tells a power cycle from the
reset button, (2) whether a RAM record survives reset but not power loss, (3) whether opening the
Serial Monitor resets the board. These decide Requirements O2-6a/6b and CON-3.

**Safe:** the sketch touches no output pins and starts no devices. Nothing needs to be wired.
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

## Return the results
Select all in the Serial Monitor, copy, paste into `docs/results/reset-probe-wifi-YYYYMMDD.txt`,
and tell Claude. The conclusions go into Requirements O2-6a/6b and CON-3.
