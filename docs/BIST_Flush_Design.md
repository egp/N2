# BIST FLUSH (O2 flush valve) step: revised logic flow (DRAFT 2026-10-10, owner review pending)

Why: the flush valve needs N2-low pressure to operate (owner, 2026-10-09; threshold unknown, >= 10 PSI assumed; production `n2LowOff` is 10.00 PSI, `n2LowOn` 20.00 PSI). The old step toggled the valve at 2 Hz on a dead N2-low line and proved nothing. The flush valve opens to the low-pressure N2 buffer tank ("N2 @ 10 PSI"); its location is still to be found on site.

## Flow
```
ENTER step
  vetoes (BIST-11, as the other output steps): TBS OFF; air, N2-low, N2-high sensors in window; F04 not active
      N2-low sensor out of range (e.g. 0.43 V)?  -> REFUSED "N2-low sensor out of range": the step cannot see the pressure.
                                                    Owner may enter the gauge value (g n2l <psi>) and type `m` (manual gauge mode), or skip (s).
  read N2L0 (PSI)
  N2L0 >= FLUSH_MIN (10 PSI, configurable)?  -- yes -->  go to FLUSH
  no:
     PRECONDITIONS for a fill: air >= airLowOn (90 PSI), N2-high < n2HighOff, owner types `go`  (a tower valve will open)
     FILL: open LEFT (BIST grace 1 s after the opening, then INV-2 veto applies);
           every 50 ms: record N2-low; stop when N2-low >= FLUSH_MIN + margin (12 PSI) 
           stop on: BIST-11 veto, TBS ON, `q`, or fill time limit (60 s)
           close LEFT; wait 1 s (settle); read N2L1
     N2-low did not reach the target?  -> report "N2-low X PSI after Y s": answer f (fail) / r (repeat) / s (skip)
FLUSH
  record N2L1; for k = 1..3:
     open FLUSH 1 s (record N2-low at open, at +0.5 s, at close), closed 1 s
     abort if N2-low < 3 PSI (do not drain the buffer)
  print: N2L before fill, after fill, per cycle drop (PSI), O2 sensor reading before/after (if present)
  prompt: "Did the valve click / did you feel air? (the buffer gauge moved?)  p / f"
  owner may add:  g n2l <psi>   (the gauge at the moment it first worked)  -> logged as the minimum working pressure
END: all outputs OFF
```

## Differences from the old step
* A fill phase before the toggling (the only output step that needs an earlier output).
* Slower toggling (1 s ON / 1 s OFF, 3 cycles) so the click and the gauge movement can be seen and heard.
* It reports the pressure the valve actually worked at.

## Requirement/test work to do (before code)
* BIST-xx text in Requirements.md; `BistConfig` fields: `flushMinX100`, `flushMarginX100`, `flushFillMaxMs`, `flushCycleMs`.
* Host tests: skips the fill when N2-low is already high; fills then flushes; fill timeout; sensor-out-of-range refusal; abort on air low (after grace); abort on TBS; N2-low floor abort; manual gauge mode.
