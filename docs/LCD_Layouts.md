# LCD layout proposals (20 × 4, fixed-width) — revision 2

Revised after the owner's review (2026-10-02). Requirements DSP-1…DSP-9 apply. The chosen layout becomes the
golden screens in the host tests; changing it later means editing one table and the expected text.

## Owner's decisions applied
1. Show the **O2 warm-up time remaining**, **in place of N2%** while warming.
2. **N2-low and N2-high on the same line**, with the compressor (SSR) state on that line if possible.
3. **AIR** has a physical gauge, so it is **first to go** if space is needed.
4. **LRFS stacked vertically**: the letters above the four bits.
5. **No TBS on the LCD.**
6. **N2% only**, no O2%.
7. Faults: **toggle** between the normal screen and a full-screen fault display every 3–4 s.

## Option 1 — clear labels (recommended for readability)

```
  0         1
  01234567890123456789
  N2% 99.99  O2 S
  N2L 12.34 N2H 123.4
  CMP ON  TWR LB  LRFS
  AIR 123.4       1001
```
- Row 0: purity and the O2 state. Row 1: both N2 pressures with full labels. Row 2: compressor and tower
  states, and the **`LRFS` letters**. Row 3: AIR, and the **four bits under their letters** (L over 1, R over 0, F over 0, S over 1).
- Compressor state is **not** on the pressure line (no room with full labels); it is one row below.
- Dropping AIR leaves row 3 as just the bits.

## Option 2 — compressor on the pressure line (what the owner asked for)

```
  0         1
  01234567890123456789
  N2% 99.99  O2 S
  L12.34 H123.4 CMP:ON
  TWR LB          LRFS
  AIR 123.4       1001
```
- Row 1 carries **N2-low (`L`), N2-high (`H`) and the compressor state** on one line; the shortened `L`/`H` labels
  are the price. (`CMP:LO` / `CMP:HI` also say *why* the compressor is stopped.)
- Row 2 holds the tower state; AIR and the bits share row 3.

## Both options in other states

O2 warming up (countdown replaces N2%; the label becomes `WRM`; the tower is held off, INV-10):
```
  WRM  4:32  O2 WM
  L12.34 H123.4 CMP:ON
  TWR OF          LRFS
  AIR 123.4       0000
```
N2% not valid (first cycle, O2 error, or stale): `N2% --.--`. Everything disabled: states read `OF`, the bits `0000`.

### Fault display — full 20×4, alternating with the normal screen
Shown for 4 s, then the normal screen for 4 s, and so on (`LCD_FAULT_CYCLE_MS`). With two faults the sequence is
fault 1, normal, fault 2, normal.
```
  0         1
  01234567890123456789
  FAULT 1 OF 2  INHIBIT
  F03 N2H SENSOR RANGE
  RAW 12  0.06V  <0.40V
  TOWER+SSR HELD OFF
```
(rows: header with severity; code and short name; the measured value against its limit; what the system is doing about it.)
A warning-level fault uses `WARNING` in the header. The LED shows `F03`.

### Startup banner (about 1 s)
```
  N2V8 0.1.0-m2
  UNO R4 Minima
  Oct  2 2026 14:05
  POST ...
```

## Field map (Option 1; fixed positions, so a changed field is one write)

| Row | Cols | Content |
|---|---|---|
| 0 | 0–2, 4–8 | `N2%`, value (`--.--`, or `WRM` and `m:ss`) |
| 0 | 11–12, 14–15 | `O2`, state `??` `WM` `F ` `S ` `W ` `E ` `OF` |
| 1 | 0–2, 4–8 | `N2L`, value |
| 1 | 10–12, 14–18 | `N2H`, value |
| 2 | 0–2, 4–5 | `CMP`, `ON` `OF` `LO` `HI` |
| 2 | 8–10, 12–13 | `TWR`, `OF` `L ` `LB` `R ` `RB` |
| 2 | 16–19 | `LRFS` |
| 3 | 0–2, 4–8 | `AIR`, value (first to drop) |
| 3 | 16–19 | the four actual output bits |

## Remaining questions (software)
1. Option 1 (clear labels, compressor one row down) or Option 2 (compressor on the pressure line, labels shortened to `L`/`H`)?
2. Fault cycle: 3 s or 4 s?
