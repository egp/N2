#!/usr/bin/env python3
"""update_sketch_toc.py SKETCH.ino  -  (re)writes the "WHERE TO EDIT" table of contents at the top of a sketch.

  python3 deliverables/update_sketch_toc.py N2V8/N2V8.ino                                     <- the real firmware (file:line entries)
  python3 deliverables/update_sketch_toc.py deliverables/tom_i2c_check/tom_i2c_check/tom_i2c_check.ino   <- the single-file test sketch (NOT used: v1.6 has shipped)

It finds every entry by a search pattern (never by a typed line number), so run it again after any edit and the numbers are right.
For N2V8.ino the entries are  file:line  (relative to the sketch folder N2V8/N2V8/): the code is in src/, which the IDE does not show as tabs; open
the file (File > Open) and use Ctrl+L = Go to line. The whole table is checked to end before line 50 of the sketch.
This script is for the author's Mac. It is not part of any package for the site. Needs only Python 3.
"""
import os, re, sys

MAX_END_LINE = 50

path = sys.argv[1] if len(sys.argv) > 1 else "N2V8/N2V8.ino"
base_dir = os.path.dirname(os.path.abspath(path))
lines = open(path).read().split("\n")

# ---------------------------------------------------------------------------------------------- remove an old table (either style)
def remove_old(start_prefix, end_prefix):
    global lines
    if not any(l.startswith(start_prefix) for l in lines):
        return
    a = next(i for i, l in enumerate(lines) if l.startswith(start_prefix))
    b = next(i for i, l in enumerate(lines) if l.startswith(end_prefix))
    while a > 0 and lines[a - 1] in ("//", ""):
        a -= 1
    del lines[a:b + 1]
    if a < len(lines) and lines[a] != "" and not lines[a].startswith("//"):
        lines.insert(a, "")

# ---------------------------------------------------------------------------------------------- helpers
def read(rel):
    return open(os.path.join(base_dir, rel)).read().split("\n")

def find(rel, regex, nth=1):
    """1-based line number of the nth line in file `rel` (relative to the sketch folder) matching regex"""
    src = lines if rel == os.path.basename(path) else read(rel)
    hits = [i + 1 for i, l in enumerate(src) if re.search(regex, l)]
    if len(hits) < nth:
        raise SystemExit("pattern not found in %s: %s" % (rel, regex))
    return hits[nth - 1]

# ================================================================================================ N2V8.ino
def do_n2v8():
    START = "// WHERE TO EDIT"
    END = "// END WHERE TO EDIT"
    remove_old(START, END)

    common = [  # (file, regex, what)
        ("src/ControlConfig.h", r"^    700, 900", "TOWER      airLowOff airLowOn towerFillMs towerOverlapMs"),
        ("src/ControlConfig.h", r"^    1000, 2000, 1000, 1200", "COMPRESSOR n2LowOff n2LowOn n2HighOn n2HighOff"),
        ("src/ControlConfig.h", r"^    60000, 2000, 250", "O2         interval flush sample count retry timeout errorRetry warm-up mandatory"),
        ("src/ControlConfig.h", r"^    1000, 3, 5000", "OUTPUTS/FAULTS  minHold sensorFaultSamples faultHold orderMargin orderHold"),
        ("src/Config.h", r"kSensorMinMillivolts", "sensor valid window 0.5-4.5 V, fault window 0.4-4.6 V, full scales"),
        ("src/app/App.h", r"uint32_t watchdogMs", "watchdog ms, LCD layout, LCD start delay, fault screen cycle, log level"),
        ("src/BuildConfig.h", r"#define N2_BUILD_DIAG", "build mode (DIAG default / FIELD), default log level, O2 mandatory"),
        ("src/BoardPins.h", r"kMinimaSignals\[", "pins, active levels (kLcdAddress = 0x23 just above the boards)"),
    ]
    areas = [
        ("src/app/App.cpp", r"void App::loop", "loop(): POST mode / BIST / RUN"),
        ("src/app/App.cpp", r"void App::setup", "setup(): outputs safe first, then the rest"),
        ("src/core/System.cpp", r"void System::step", "one pass: inputs, controllers, invariants, outputs"),
        ("src/core/Tower.cpp", r"void Tower::update", "TOWER controller"),
        ("src/core/Compressor.cpp", r"void Compressor::update", "COMPRESSOR controller"),
        ("src/core/O2Controller.cpp", r"void O2Controller::update", "O2 controller (flush, sample, warm-up)"),
        ("src/drivers/O2SensorDfrobot.cpp", r"readO2PercentX100", "O2 sensor read (DFRobot library adapter)"),
        ("src/core/Sensors.cpp", r"::sample", "pressure inputs, fault windows"),
        ("src/core/Invariants.cpp", r"checkInvariants", "SAFETY rules (INV-1 .. INV-10)"),
        ("src/core/OutputDriver.cpp", r"::apply", "outputs: active levels, minimum hold"),
        ("src/core/Faults.cpp", r"kTable\[", "fault table (codes, text, severity)"),
        ("src/ui/LcdScreens.cpp", r"Screen renderNormal", "LCD normal screen"),
        ("src/ui/LedText.cpp", r"LedText renderLed", "LED text"),
        ("src/drivers/Lcd20x4.cpp", r"^void Lcd20x4::service\(", "LCD driver"),
        ("src/drivers/Led1650.cpp", r"^void Led1650::service\(", "LED driver"),
        ("src/drivers/Rtc3231.cpp", r"bool Rtc3231::read", "RTC driver"),
        ("src/selftest/Post.cpp", r"void Post::runCheck", "POST"),
        ("src/selftest/Bist.cpp", r"^void Bist::tick\(", "BIST"),
        ("src/ui/Commands.cpp", r"Commands::handle", "console commands"),
        ("src/hal/HalArduino.cpp", r"HalArduino::consoleBegin", "hardware access: console, I2C, reset cause"),
        ("N2V8.ino", r"^void setup\(\)", "this file: setup() and loop() glue (:%d)"),
    ]

    def make(offset):
        out = [START + "   (file:line, relative to this folder. In the Arduino IDE 2: open the file, then Ctrl+L = go to line)",
               "// COMMON EDITS (air and N2-high PSI x10, N2-low PSI x100, times in ms)"]
        for f, rx, what in common:
            n = find(f, rx) + (offset if f == os.path.basename(path) else 0)
            out.append("//   %-26s %s" % ("%s:%d" % (f, n), what))
        out.append("// CODE BY AREA")
        for f, rx, what in areas:
            n = find(f, rx) + (offset if f == os.path.basename(path) else 0)
            if f == os.path.basename(path):
                what = what % (find(f, r"^void loop\(\)") + offset)
            out.append("//   %-34s %s" % ("%s:%d" % (f, n), what))
        out.append("// To refresh these line numbers after editing, run:   python3 deliverables/update_sketch_toc.py N2V8/N2V8.ino")
        out.append(END)
        return out

    first_include = next(i for i, l in enumerate(lines) if l.startswith("#include"))
    block_len = len(make(0)) + 1          # the table plus one blank line after it
    block = make(block_len + 0)
    # the table goes right after the header comment, with a blank line before the first #include
    lines[first_include:first_include] = block + [""]
    end_line = first_include + len(block)  # 1-based line number of the END marker
    if end_line > MAX_END_LINE:
        raise SystemExit("table ends at line %d; the limit is %d: shorten the entries" % (end_line, MAX_END_LINE))
    open(path, "w").write("\n".join(lines))
    print("N2V8.ino: table of contents written; it ends at line %d (limit %d)" % (end_line, MAX_END_LINE))

# ================================================================================================ the single-file test sketch (kept; not used)
def do_single_file():
    START = "// TABLE OF CONTENTS"
    END = "// END OF TABLE OF CONTENTS"
    remove_old(START, END)

    def banner(title_regex):
        for i, l in enumerate(lines):
            if re.match(title_regex, l) and i > 0 and re.match(r"^// -{40,}$", lines[i - 1]):
                return i
        raise SystemExit("banner not found: " + title_regex)

    def code(regex, after=0):
        for i in range(max(after - 1, 0), len(lines)):
            if re.search(regex, lines[i]):
                return i + 1
        raise SystemExit("code not found: " + regex)

    entries = [("Includes and version", code(r"^#include <Arduino.h>")),
               ("I2C access and bus recovery", banner(r"^// I2C access, with bus recovery")),
               ("DS3231 RTC driver", banner(r"^// DS3231 real-time clock")),
               ("TM1650 LED driver", banner(r"^// TM1650 4-digit LED display")),
               ("20x4 LCD driver", banner(r"^// 20x4 LCD")),
               ("POST", banner(r"^// POST: one quick hands-off check")),
               ("O2 query", banner(r"^// O2 sensor \(DFRobot SEN0465\)")),
               ("setup()", code(r"^void setup\(\)")),
               ("loop()", code(r"^void loop\(\)"))]
    def build(offset):
        out = [START + "  (Arduino IDE 2: Ctrl+L = go to line)", "//   line  what"]
        out += ["//   %4d  %s" % (n + offset, label) for label, n in entries]
        out.append(END)
        return out
    n_table = len(build(0)) + 1
    lines[3:3] = ["//"] + build(n_table)
    open(path, "w").write("\n".join(lines))
    print("single-file sketch: table of contents written")

if os.path.basename(path) == "N2V8.ino":
    do_n2v8()
else:
    do_single_file()
