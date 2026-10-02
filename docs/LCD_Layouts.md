# LCD layout proposals (20 × 4, fixed-width)

Status: **for discussion.** Requirements DSP-1…DSP-9 apply. Whatever is chosen becomes the golden
screens in the M3 host tests.

## What must be shown

| Item | Example | Source / note |
|---|---|---|
| Air supply pressure | `123.4` | PSI, 1 decimal |
| N2 low pressure | `12.34` | PSI, 2 decimals |
| N2 high pressure | `123.4` | PSI, 1 decimal |
| N2 purity | `99.99` | % (100 − O2%); `--.--` when invalid |
| TBS | `ON` / `OF` | 2 characters (owner) |
| Tower state | `OF L LB R RB` | V6/V7 |
| Compressor state | `OF ON LO HI` | V6/V7 |
| O2 state | `?? F S W E OF` | V6/V7; `W` also used for "waiting" so warm-up needs its own indication |
| Output states | `LRFS 1001` | the four *actual* driven outputs: Left valve, Right valve, Flush valve, SSR |
| Fault text | `F03 N2H SENSOR RANGE` | DSP-5, shown only when a fault is active |

The `LRFS` bits are redundant with the controller states *in normal operation*, but they show what
the OutputDriver really drove (minimum-hold deferrals, invariant overrides), so they are kept.

## Reference: the V6/V7 layout (no TBS)

```
  0         1
  01234567890123456789
  AIR 123.4  TWR  LB
  N2L 12.34  CMP  ON
  N2H 123.4  O2   S
  N2% 99.99  LRFS 1001
```
There is no free room for a labelled TBS field, which is why a new layout is proposed.

## Option A — grouped by function (everything fits, nothing dropped)

```
  0         1
  01234567890123456789
  TBS ON TWR LB CMP ON
  AIR 123.4 N2L 12.34
  N2H 123.4 N2% 99.99
  O2 S      LRFS 1001
```
Pro: pressures together, nothing shortened. Con: furthest from V6/V7, so least familiar; the O2
state sits alone on the last row.

## Option B — status bar + V7's bottom row  **(recommended)**

```
  0         1
  01234567890123456789
  TBS ON  CMP ON O2 S 
  AIR 123.4  TWR LB
  N2L 12.34 N2H 123.4
  N2% 99.99  LRFS 1001
```
- Row 0 is a **status bar**: system switch, compressor, O2 state. All 2-character values (`ON`/`OF`).
- Rows 1–3 keep V7's AIR line and the **unchanged bottom row** (`N2%` and `LRFS`), so the familiar
  parts stay where they were; the two N2 pressures share one row.
- Everything required is shown.

Field map (all fixed positions, so a changed field is a single write, DSP-2):

| Row | Cols | Content | Width |
|---|---|---|---|
| 0 | 0–2 / 4–5 | `TBS` / `ON`·`OF` | 3 + 2 |
| 0 | 8–10 / 12–13 | `CMP` / `ON`·`OF`·`LO`·`HI` | 3 + 2 |
| 0 | 15–16 / 18–19 | `O2` / `??`·`F `·`S `·`W `·`E `·`OF` | 2 + 2 |
| 1 | 0–2 / 4–8 | `AIR` / value | 3 + 5 |
| 1 | 11–13 / 15–16 | `TWR` / `OF`·`L `·`LB`·`R `·`RB` | 3 + 2 |
| 2 | 0–2 / 4–8 | `N2L` / value | 3 + 5 |
| 2 | 10–12 / 14–18 | `N2H` / value | 3 + 5 |
| 3 | 0–2 / 4–8 | `N2%` / value (`--.--` if invalid) | 3 + 5 |
| 3 | 11–14 / 16–19 | `LRFS` / four bits | 4 + 4 |

### Option B in other states

TBS off (everything disabled; N2% invalid):
```
  TBS OF  CMP OF O2 OF
  AIR 123.4  TWR OF
  N2L 12.34 N2H 123.4
  N2% --.--  LRFS 0000
```
O2 sensor warming up (5 min; tower held off, INV-10): the N2% label changes to `WRM` and shows the countdown
```
  TBS ON  CMP ON O2 W 
  AIR 123.4  TWR OF
  N2L 12.34 N2H 123.4
  WRM  4:32  LRFS 0000
```
A fault (row 0 alternates with the status bar every 2 s; the other rows stay live, DSP-5):
```
  F03 N2H SENSOR RANGE
  AIR 123.4  TWR LB
  N2L 12.34 N2H 123.4
  N2% 99.99  LRFS 1001
```
Startup banner (DSP-8, about 1 s):
```
  N2V8 0.1.0-m2
  UNO R4 Minima
  Oct  2 2026 14:05
  POST ...
```

## Option C — outputs spelled out

```
  0         1
  01234567890123456789
  AIR 123.4 N2% 99.99
  N2L 12.34 N2H 123.4
  L ON R OF F OF S ON
  TBS ON TWR LB O2 S
```
Pro: valves readable without decoding `1001`. Con: **drops the compressor state** (`LO`/`HI` tell you
*why* the SSR is off) and the tower state is half redundant; most different from V6/V7.

## Comparison

| | A | **B** | C |
|---|---|---|---|
| Everything required shown | yes | **yes** | no (CMP state) |
| Keeps V7's bottom row | no | **yes** | no |
| TBS ON/OF prominent | yes | **yes** (row 0) | yes |
| Valve states as 0/1 bits | yes | **yes** | spelled out |
| Fault line room | row 0 | **row 0** | row 3 |

## Questions

1. **O2% or N2%?** V6/V7 show only N2% (= 100 − O2%). Show N2% (as proposed), O2% instead, or both (the console and BIST show raw O2% in any case)?
2. Is `WRM mm:ss` in the N2% position acceptable during warm-up, or should the O2 state letter `W` plus a separate countdown be used?
3. Should the fault alternate with the status bar (proposed) or take over the whole screen?
