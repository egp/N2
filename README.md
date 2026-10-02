# N2V8 — PSA nitrogen generator controller

Arduino UNO R4 Minima (production) / UNO R4 WiFi (bench) firmware, designed so nearly all logic
is tested on a host through a thin hardware access layer.

**To build it, open `N2V8/N2V8.ino` in the Arduino IDE.** The sketch lives in its own `N2V8/` folder inside this repository,
so the folder name matches the `.ino` file name no matter what the repository folder is called after a `git clone`/`git pull`
or a zip download. (The same rule applies to every other sketch here, e.g. `experiments/reset_probe/reset_probe.ino`.)

- `docs/Requirements.md` — requirements (v2.1 draft)
- `docs/Project_Plan.md` — milestones, environments, git and CI plan
- `docs/Owner_TODO.md` — items only the owner can resolve
- `N2V8/` — the firmware (open `N2V8/N2V8.ino`); `N2V8/src/` holds the code
- `test/host/` — host tests (CMake + Catch2)
- `experiments/` — small throw-away sketches for the bench
- `reference/` — earlier iterations (V5, V6, V7) kept as source material to reuse; **not compiled**

Branch `v8` replaces `main` when V8 works. The V5 layout remains in the history of `main`.
