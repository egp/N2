# LCD layout proposals (20 × 4, fixed-width) — revision 3 (renders are now produced by the code)

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

## Option 1 — clear labels (CHOSEN)

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
  N2V8 8.1.1B
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

## Decisions (owner, 2026-10-06)
1. **Option 1 (clear labels) is chosen.** Option 2 stays in the code only until the golden tests are rewritten, then it goes.
2. **The compressor-running text is removed.** The compressor needs no label while it is simply running or off; the CMP field appears only to say
   **why it is stopped: `LO` (N2-low too low) or `HI` (N2-high too high)**. That reclaims the field for something more useful.
3. **TBS is not shown** on the LCD (TBS on the display was dropped; the LED/LCD simply show the state of the system).
4. Fault screen cycle: still open, 3 s or 4 s (the code uses a constant, `LCD_FAULT_CYCLE_MS`).

**Done in the code (2026-10-06):** row 2, columns 0-5 now show `CMP LO` / `CMP HI` only while the compressor is stopped for that reason; otherwise
the **code of the last fault raised** as two hex digits (`ER 12`, for the last ERror), or **blank** if none has been raised since reset. Fault codes are hex everywhere
(LCD, LED, console, log): `Fxx`, high digit = group (see `docs/Fault_List.md`).

## Real renders (output of `render_screens`, i.e. what the firmware's own code produces)

These are not mock-ups: they come from the same functions the LCD driver uses, and the golden tests in
`test/host/test_display.cpp` pin them exactly. The first two blocks are the two layout options; changing the choice later
means changing which function is selected, not the screens' content.

```
=========== OPTION 1 (clear labels) ===========

normal running
   0         1
   01234567890123456789
  |N2% 99.99  O2 S     |
  |N2L 12.34 N2H  98.7 |
  |        TWR LB  LRFS|
  |AIR 123.4       1001|

O2 warming up (4:32 left), tower held off
   0         1
   01234567890123456789
  |WRM  4:32  O2 WM    |
  |N2L 12.34 N2H  98.7 |
  |        TWR OF  LRFS|
  |AIR 123.4       0001|

TBS off (everything disabled, N2% invalid)
   0         1
   01234567890123456789
  |N2% --.--  O2 OF    |
  |N2L 12.34 N2H  98.7 |
  |        TWR OF  LRFS|
  |AIR 123.4       0000|

N2% stale (pressures out of range)
   0         1
   01234567890123456789
  |N2% 99.99* O2 W     |
  |N2L 12.34 N2H  98.7 |
  |        TWR LB  LRFS|
  |AIR 123.4       1001|

compressor stopped, N2-high too high
   0         1
   01234567890123456789
  |N2% 99.99  O2 S     |
  |N2L 12.34 N2H  98.7 |
  |CMP HI  TWR LB  LRFS|
  |AIR 123.4       1000|

=========== OPTION 2 (compact) ===========

normal running
   0         1
   01234567890123456789
  |N2% 99.99  O2 S     |
  |L12.34 H 98.7 CMP:ON|
  |TWR LB          LRFS|
  |AIR 123.4       1001|

O2 warming up (4:32 left), tower held off
   0         1
   01234567890123456789
  |WRM  4:32  O2 WM    |
  |L12.34 H 98.7 CMP:ON|
  |TWR OF          LRFS|
  |AIR 123.4       0001|

TBS off (everything disabled, N2% invalid)
   0         1
   01234567890123456789
  |N2% --.--  O2 OF    |
  |L12.34 H 98.7 CMP:OF|
  |TWR OF          LRFS|
  |AIR 123.4       0000|

N2% stale (pressures out of range)
   0         1
   01234567890123456789
  |N2% 99.99* O2 W     |
  |L12.34 H 98.7 CMP:ON|
  |TWR LB          LRFS|
  |AIR 123.4       1001|

compressor stopped, N2-high too high
   0         1
   01234567890123456789
  |N2% 99.99  O2 S     |
  |L12.34 H 98.7 CMP:HI|
  |TWR LB          LRFS|
  |AIR 123.4       1000|

=========== FAULT SCREENS ===========

fault 1 of 2 (sensor)
   0         1
   01234567890123456789
  |FAULT 1 OF 2 INHIBIT|
  |F03 N2H SENSOR RANGE|
  |RAW 12  0.05V       |
  |TOWERS+SSR OFF      |

fault 2 of 2 (O2)
   0         1
   01234567890123456789
  |FAULT 2 OF 2 INHIBIT|
  |F12 O2 SENSOR FAILED|
  |O2 STATE S          |
  |ALL OUTPUTS OFF     |

N2L above N2H
   0         1
   01234567890123456789
  |FAULT 1 OF 1 INHIBIT|
  |F04 N2L ABOVE N2H   |
  |L28.00>H 15.0       |
  |TOWERS+SSR OFF      |

watchdog reset (warning)
   0         1
   01234567890123456789
  |FAULT 1 OF 1 WARNING|
  |F30 WATCHDOG RESET  |
  |                    |
  |RESET WAS LOGGED    |

=========== BANNER ===========

startup
   0         1
   01234567890123456789
  |N2V8 8.1.1B          |
  |UNO R4 Minima       |
  |Oct  2 2026 14:05   |
  |POST ...            |

=========== LED ===========
  running 99.99%               [9999] dot after 1
  TBS off                      [    ] dot after -1
  N2% invalid                  [----] dot after 1
  fault F03 shown              [ F03] dot after -1
```

## Update rate (DSP-12, owner decision 2026-10-08)
The NORMAL screen is sent to the LCD at most once per second (`kDefaultLcdMinChangeMs` in ui/DisplayManager.h, `AppOptions::lcdMinChangeMs`; 0 = every pass). A changed value (the last digit of a pressure that wanders by one ADC count) therefore does not flicker; the display shows the newest value once a second. Anything on the normal screen, including the valve/SSR bits, can be up to 1 s behind. Start-up, POST and BIST screens (overrides) are not held back. The LED is not limited.
