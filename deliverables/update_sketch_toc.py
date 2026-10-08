#!/usr/bin/env python3
"""update_sketch_toc.py SKETCH.ino  -  (re)writes the TABLE OF CONTENTS comment at the top of tom_i2c_check.ino.

Run it after you edit the sketch: it finds every section and every device's code by pattern and writes the line numbers (the numbers are the FINAL
line numbers, with the table itself counted). In the Arduino IDE 2: Ctrl+L (Cmd+L on a Mac) = Go to line. This script is for the author's Mac; it is
NOT part of the package for the site. Needs only Python 3.
"""
import re, sys

path = sys.argv[1] if len(sys.argv) > 1 else "tom_i2c_check/tom_i2c_check.ino"
START = "// TABLE OF CONTENTS"
END = "// END OF TABLE OF CONTENTS"
lines = open(path).read().split("\n")

# remove an old table
if any(l.startswith(START) for l in lines):
    a = next(i for i, l in enumerate(lines) if l.startswith(START))
    b = next(i for i, l in enumerate(lines) if l.startswith(END))
    # also drop the blank comment line before the table, if there is one
    if a > 0 and lines[a - 1] == "//":
        a -= 1
    del lines[a:b + 1]

def banner(title_regex):
    """line number (1-based) of the dashed line above the banner whose first text line matches"""
    for i, l in enumerate(lines):
        if re.match(title_regex, l) and i > 0 and re.match(r"^// -{40,}$", lines[i - 1]):
            return i  # 1-based number of the dashed line (index i-1 is 0-based)
    raise SystemExit("banner not found: " + title_regex)

def code(regex, after=0, within=100000):
    """1-based line number of the first line matching regex at or after line `after` (1-based), within `within` lines"""
    for i in range(max(after - 1, 0), min(len(lines), max(after - 1, 0) + within)):
        if re.search(regex, lines[i]):
            return i + 1
    raise SystemExit("code not found: " + regex)

def code_all(regex, after=0):
    return [i + 1 for i in range(max(after - 1, 0), len(lines)) if re.search(regex, lines[i])]

# (indent, label, line number)
entries = []
def sec(label, n): entries.append((0, label, n))
def sub(label, n): entries.append((1, label, n))

sec("Includes and version", code(r"^#include <Arduino.h>"))
sec("Types (DateTime, Level, Verdict, test steps)", code(r"^struct DateTime"))
sec("Addresses and timing constants", banner(r"^// Addresses \(7-bit\)"))
sec("Small helpers (say, reached)", banner(r"^// Small helpers"))
sec("I2C access and bus recovery", banner(r"^// I2C access, with bus recovery"))
sec("Calendar helpers, compile time", banner(r"^// Calendar helpers"))
sec("DS3231 RTC driver", banner(r"^// DS3231 real-time clock"))
sec("TM1650 LED driver", banner(r"^// TM1650 4-digit LED display"))
sub("ledService (refresh 4x per second)", code(r"^static void ledService"))
sec("20x4 LCD driver (PCF8574 backpack)", banner(r"^// 20x4 LCD"))
sub("lcdInit", code(r"^static bool lcdInit"))
sub("lcdService (start delay, one row per pass)", code(r"^static void lcdService"))
sec("POST: one quick check per device", banner(r"^// POST: one quick hands-off check"))
p = code(r"^static void postLcd")
sub("postLcd", p); sub("postRtc (sets the RTC from the compile time)", code(r"^static void postRtc", p)); sub("postLed", code(r"^static void postLed", p)); sub("postO2", code(r"^static void postO2", p))
sec("O2 sensor: one read-only query", banner(r"^// O2 sensor \(DFRobot SEN0465\)"))
sub("o2Query", code(r"^static bool o2Query"))
sec("Reset cause and the RESET-button test", banner(r"^// Reset cause, and the RESET-button test"))
sub("readResetCause", code(r"^static void readResetCause"))
sub("resetTestEvaluate (at boot)", code(r"^static void resetTestEvaluate"))
sec("TBS and TOB (switch inputs)", banner(r"^// TBS and TOB: read with debounce"))
sub("switchService", code(r"^static void switchService"))
sec("BIST: the guided test (all steps)", banner(r"^// BIST: guided test"))
st = code(r"^static void bistStartStep")
sub("bistStartStep (what each step starts)", st)
sv = code(r"^static void bistService")
sub("bistService (what each step does)", sv)
steps = []
for name, rx in (("LCD step", r"gBistStep == B_LCD\)"), ("RTC step", r"gBistStep == B_RTC\)"), ("LED step (the last branch)", r"else \{  // B_LED"),
                 ("O2 step", r"gBistStep == B_O2\)"), ("TBS / TOB steps", r"gBistStep == B_TBS \|\| gBistStep == B_TOB"), ("RESET step", r"gBistStep == B_RST\)")):
    steps.append((2, name, code(rx, sv, 300)))
entries.extend(sorted(steps, key=lambda e: e[2]))
sub("bistBegin (starts a run: all steps or one)", code(r"^static void bistBegin"))
sec("What the LCD and the LED show", banner(r"^// What the LCD and the LED show"))
sub("updateDisplays", code(r"^static void updateDisplays"))
sec("Serial Monitor console", banner(r"^// Serial Monitor console"))
sub("printHelp", code(r"^static void printHelp"))
sub("printStatus", code(r"^static void printStatus"))
sub("printScan", code(r"^static void printScan"))
sub("handleLine (the commands)", code(r"^static void handleLine"))
sub("consoleService (banner, input)", code(r"^static void consoleService"))
sec("setup()", code(r"^void setup\(\)"))
sec("loop()", code(r"^void loop\(\)"))

common = [
    ("LCD I2C address (0x23, A2 bridged)", [code(r"^static const uint8_t ADDR_LCD")]),
    ("LCD start delay after power-up (2500 ms)", [code(r"^static const uint32_t LCD_START_MS")]),
    ("TBS and TOB pins (D0, D1)", [code(r"^static const uint8_t PIN_TBS")]),
    ("Time to wait for a p / f answer (8000 ms; LCD and LED)", code_all(r"gAskUntil = now \+ 8000")),
    ("Time to wait at the TBS / TOB / RESET steps (20000 ms)", code_all(r"gPhaseAt = now \+ 20000", st)),
    ("Pause between automatic test runs (45000 ms)", code_all(r"gAutoAt = now \+ 45000")),
    ("LED refresh period (250 ms)", code_all(r"gLedNext = now \+ 250")),
    ("O2 sensor commands (read-only; never change these)", [code(r"uint8_t frame\[9\]")]),
]

# build the table text (final line numbers = number in the file now + the number of lines the table adds)
def build(offset):
    out = [START + "  (Arduino IDE 2: Ctrl+L = go to line; Cmd+L on a Mac)",
           "//   line  what"]
    for ind, label, n in entries:
        out.append("//   %4d  %s%s" % (n + offset, "    " * ind, label))
    out.append("//")
    out.append("//   COMMON EDITS")
    for label, n in common:
        ns = n if isinstance(n, list) else [n]
        extra = "" if len(ns) == 1 else "   (also at line " + ", ".join(str(x + offset) for x in ns[1:]) + ")"
        out.append("//   %4d  %s%s" % (ns[0] + offset, label, extra))
    out.append(END)
    return out

n_table = len(build(0)) + 1          # + the blank comment line placed before the table
table = build(n_table)
# insert after the 3-line title block (lines 1-3) of the header
insert_at = 3
lines[insert_at:insert_at] = ["//"] + table
open(path, "w").write("\n".join(lines))
print("table of contents written:", len(table), "lines; sketch now", len(lines), "lines")
