# N2V8 — PSA nitrogen generator controller

Arduino UNO R4 Minima (production) / UNO R4 WiFi (bench) firmware, designed so nearly all logic
is tested on a host through a thin hardware access layer.

**Status (2026-10-07)**

| Item | State |
|---|---|
| Branch | `v8` (WIP); `main` remains known-good V5 until V8 earns the replace |
| Host suite | ~**408** tests green (incl. property + mutation checks) |
| Milestones | M0–M5 done on the host; **M6 (bench) in progress** |
| E1 reset probe | **DONE on R4 WiFi** (2026-10-06); Minima still open |
| Stage 1 (LCD+RTC) | Built; host tests pass; full Stage 1 bench run still pending |
| DIAG flash | Still open (Owner_TODO 10a) |
| LED (TM1650) | Blocked on replacement module |
| Pinout / FIELD | Owner verify before trusting FIELD; no `v8`→`main` until M9 |

**To build it, open `N2V8/N2V8.ino` in the Arduino IDE.** The sketch lives in its own `N2V8/` folder inside this repository,
so the folder name matches the `.ino` file name no matter what the repository folder is called after a `git clone`/`git pull`
or a zip download. (The same rule applies to every other sketch here, e.g. `experiments/reset_probe/reset_probe.ino`.)

- `docs/Requirements.md` — requirements (v2.6 draft)
- `docs/Project_Plan.md` — milestones, environments, git and CI plan
- `docs/Owner_TODO.md` — items only the owner can resolve
- `docs/Bringup_Stages.md` — staged bench bring-up (one device at a time)
- `stages/` — stage sketches (e.g. Stage 1 LCD+RTC)
- `docs/results/` — bench and field capture notes
- `N2V8/` — the firmware (open `N2V8/N2V8.ino`); `N2V8/src/` holds the code
- `test/host/` — host tests (CMake + Catch2)
- `experiments/` — small throw-away sketches for the bench
- `reference/` — earlier iterations (V5, V6, V7) kept as source material to reuse; **not compiled**

Branch `v8` replaces `main` when V8 works. The V5 layout remains in the history of `main`.
