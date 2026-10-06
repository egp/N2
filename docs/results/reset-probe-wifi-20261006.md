# Reset probe results — UNO R4 WiFi — 2026-10-06

Board: UNO R4 WiFi on the author's bench, core `arduino:renesas_uno` 1.6.0. Sketch: `experiments/reset_probe` versions 1.8–1.B (hex).
Most tests were driven over serial by Claude; the reset button and USB unplug were done by the author.

## Result table (version 1.B)

| Reset | RSTSR0 | RSTSR1 | RSTSR2.CWSF | RAM record (plain RAM @ 0x20007A40) | Credit |
|---|---|---|---|---|---|
| Power loss (USB unplug ~5 s, no external supply) | 0x00 | 0x00 | **0 (cold)** | **lost** | **0 s** |
| First boot after upload (previous boot was cold, flag not yet set) | 0x00 | 0x04 | 0 (cold) | none | 0 s |
| Software reset (`s`, also what an upload does) | 0x00 | **0x04** (SWRF) | **1 (warm)** | valid, boot #2 | yes |
| Watchdog reset (`w`) | 0x00 | **0x02** (WDTRF) | 1 (warm) | valid, boot #3 | 28 s carried |
| Reset button | 0x00 | **0x00** | 1 (warm) | valid, boot #5 | 106 s carried |

## Findings
1. **Serial:** on the R4 WiFi `Serial` is a hardware UART (core built with `-DNO_USB`); the sketch must call `Serial.begin(115200)` or every
   print returns 0. `if (Serial)` is always true; `availableForWrite()` is not implemented; writes block per byte.
2. **Reset-cause flags work and clear** with plain writes (no register unlock). SWRF = software, WDTRF = watchdog, none = reset pin.
3. **`PORF` (power-on flag) is NOT visible** to the sketch (RSTSR0 reads 0 after a real power loss): the bootloader clears it.
4. **`.noinit` is unusable:** the startup code overwrites it with bytes copied from flash at every boot (same five instruction-like words each time).
5. **Plain RAM survives software, watchdog and reset-button resets** (checked at 0x20004000, 0x20007A00, 0x20007B00) and is **lost on power loss**.
6. **The cold/warm-start flag `RSTSR2.CWSF` is the reliable "was power lost" indicator**: 0 after power loss; the sketch sets it to 1 at boot,
   so every later reset without power loss reads 1.
7. Warm-up credit rule verified: credit only when (valid RAM record) AND (warm start). Power loss always gives zero credit.

## Still open
- The same tests on the **UNO R4 Minima** (production). The probe builds for the Minima (native USB serial; no matrix).
- A very short power glitch that keeps RAM *and* leaves CWSF set has not been tested; it would also leave the O2 sensor powered, so crediting is acceptable.
