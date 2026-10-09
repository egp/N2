#!/usr/bin/env python3
"""rec_to_csv.py LOGFILE [OUT.csv]  -  turn the firmware's `rec` lines into a CSV (works on a bridge log, a PuTTY log, or a Notepad paste).

Reads lines like   R,123456,297,258,207,0,0,0000   (samples),   E,123456,tbs,1   (TBS/TOB events) and   N,123456,compressor started   (your notes:
type `note text` or `cap text` at the console). Every E, and N, line is attached to the sample just before it in the CSV's "note" column.
A host timestamp [HH:MM:SS.mmm] at the start of a line (the bridge adds it) is kept in the CSV; plain pastes simply leave it empty.
PSI is computed from the raw counts the same way the firmware does it: 0.5 V = 0 PSI, 4.5 V = full scale, 10-bit ADC, 5 V reference.
Full scales (Config.h): air 150.0 PSI, N2 low 30.00 PSI, N2 high 150.0 PSI. Change them below if Config.h changes.
Needs only Python 3.
"""
import csv, re, sys

ADC_MAX, REF_MV = 1023, 5000
FULL = {"air": 150.0, "n2low": 30.0, "n2high": 150.0}

def psi(raw, full):
    mv = raw * REF_MV / ADC_MAX
    return round((mv - 500.0) / 4000.0 * full, 2)

src = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else src.rsplit(".", 1)[0] + "_rec.csv"
rx_sample = re.compile(r"^(?:\[(\d\d:\d\d:\d\d\.\d+)\]\s+)?(?:[IDWE]\s+)?R,(\d+),(\d+),(\d+),(\d+),([01]),([01]),([01]{4})\s*$")
rx_note = re.compile(r"^(?:\[(\d\d:\d\d:\d\d\.\d+)\]\s+)?(?:[IDWE]\s+)?N,(\d+),(.*)$")
rx_event = re.compile(r"^(?:\[(\d\d:\d\d:\d\d\.\d+)\]\s+)?(?:[IDWE]\s+)?E,(\d+),(tbs|tob),([01])\s*$")
rows, events, pending = [], [], []
for line in open(src, encoding="utf-8", errors="replace"):
    line = line.rstrip("\n")
    m = rx_sample.match(line)
    if m:
        host, ms, a, l, h, tbs, tob, o = m.groups()
        a, l, h = int(a), int(l), int(h)
        rows.append([host or "", int(ms), a, l, h, psi(a, FULL["air"]), psi(l, FULL["n2low"]), psi(h, FULL["n2high"]), int(tbs), int(tob), o[0], o[1], o[2], o[3], ""])
        continue
    m = rx_note.match(line)
    if m:
        text = m.group(3)
        if rows: rows[-1][-1] = (rows[-1][-1] + " | " if rows[-1][-1] else "") + text
        else: pending.append(text)
    m = rx_event.match(line)
    if m:
        events.append((m.group(1) or "", int(m.group(2)), m.group(3), int(m.group(4))))
        text = "%s %s" % (m.group(3).upper(), "ON/pressed" if m.group(4) == "1" else "OFF/released")
        if rows: rows[-1][-1] = (rows[-1][-1] + " | " if rows[-1][-1] else "") + text
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["host_time", "ms", "air_raw", "n2low_raw", "n2high_raw", "air_psi", "n2low_psi", "n2high_psi", "tbs", "tob", "L", "R", "F", "S", "note"])
    w.writerows(rows)
print("%d sample line(s), %d TBS/TOB event(s) -> %s" % (len(rows), len(events), out))
for e in events[:20]:
    print("  event", e)
