#!/usr/bin/env python3
"""Generates the two DRAFT system schematics (electrical, plumbing) as SVG; rsvg-convert makes the PDFs.
Drawn from the owner's description and photos (2026-10-09/10). Solid = confirmed, dashed + '?' = assumed / to be confirmed by Tom."""
import sys

F = 'font-family="Helvetica,Arial,sans-serif"'
BLUE, GRN, RED, GRY, ORG, PUR = "#1565c0", "#2e7d32", "#c62828", "#666", "#ef6c00", "#6a1b9a"

class Page:
    def __init__(self, w, h, title, sub):
        self.w, self.h = w, h
        self.e = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}" {F} font-size="12">',
                  f'<rect width="{w}" height="{h}" fill="white"/>',
                  f'<rect x="8" y="8" width="{w-16}" height="{h-16}" fill="none" stroke="#333" stroke-width="1.5"/>',
                  f'<text x="24" y="38" font-size="20" font-weight="bold">{title}</text>',
                  f'<text x="24" y="58" fill="#555">{sub}</text>',
                  f'<text x="{w-24}" y="38" text-anchor="end" font-size="15" fill="{RED}" font-weight="bold">DRAFT 2026-10-10 - for Tom to verify</text>']
    def add(self, s): self.e.append(s)
    def line(self, x1, y1, x2, y2, col="#222", w=2, dash=False):
        d = ' stroke-dasharray="7 5"' if dash else ''
        self.add(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{col}" stroke-width="{w}"{d}/>')
    def poly(self, pts, col="#222", w=2, dash=False):
        d = ' stroke-dasharray="7 5"' if dash else ''
        self.add('<polyline fill="none" stroke="%s" stroke-width="%s"%s points="%s"/>' % (col, w, d, ' '.join(f'{x},{y}' for x, y in pts)))
    def rect(self, x, y, w, h, fill="white", col="#222", sw=2, dash=False, rx=0):
        d = ' stroke-dasharray="7 5"' if dash else ''
        self.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{col}" stroke-width="{sw}"{d}/>')
    def circ(self, x, y, r, fill="white", col="#222", sw=2, dash=False):
        d = ' stroke-dasharray="7 5"' if dash else ''
        self.add(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{col}" stroke-width="{sw}"{d}/>')
    def text(self, x, y, s, size=12, col="#111", anchor="start", bold=False):
        fw = ' font-weight="bold"' if bold else ''
        self.add(f'<text x="{x}" y="{y}" font-size="{size}" fill="{col}" text-anchor="{anchor}"{fw}>{s}</text>')
    def save(self, name):
        self.e.append('</svg>')
        open(name, 'w').write('\n'.join(self.e))

# ---------------------------------------------------------------- symbols
def solenoid_valve(p, x, y, label, sub="", col="#222", dash=False, vertical=False):
    """ISO 1219 2/2 solenoid valve: two squares (closed | open) with a coil. Flow left->right (or top->bottom if vertical)."""
    if not vertical:
        p.rect(x, y, 30, 30, col=col, dash=dash); p.rect(x+30, y, 30, 30, col=col, dash=dash)
        p.line(x+3, y+15, x+27, y+15, col, 1.5)            # closed square: flow blocked
        p.line(x+15, y+6, x+15, y+24, col, 1.5)
        p.line(x+33, y+15, x+57, y+15, col, 2)             # open square: through
        p.add(f'<polygon points="{x+57},{y+15} {x+50},{y+10} {x+50},{y+20}" fill="{col}"/>')
        p.rect(x+60, y+5, 16, 20, fill="#fff", col=col, dash=dash); p.line(x+60, y+25, x+76, y+5, col, 1.5)    # coil
        p.line(x-16, y+15, x, y+15, col, 2); p.line(x+60, y+15, x+60, y+15, col)
        p.text(x+30, y-8, label, 12, col, "middle", True)
        if sub: p.text(x+30, y+46, sub, 10, "#444", "middle")
        return (x-16, y+15), (x+60, y+15)
    # vertical: inlet at top, outlet at bottom
    p.rect(x, y, 30, 30, col=col, dash=dash); p.rect(x, y+30, 30, 30, col=col, dash=dash)
    p.line(x+15, y+3, x+15, y+27, col, 1.5); p.line(x+6, y+15, x+24, y+15, col, 1.5)
    p.line(x+15, y+33, x+15, y+57, col, 2)
    p.add(f'<polygon points="{x+15},{y+57} {x+10},{y+50} {x+20},{y+50}" fill="{col}"/>')
    p.rect(x+34, y+20, 20, 16, fill="#fff", col=col, dash=dash); p.line(x+34, y+36, x+54, y+20, col, 1.5)
    p.text(x+60, y+28, label, 12, col, "start", True)
    if sub: p.text(x+60, y+43, sub, 10, "#444")
    return (x+15, y-10), (x+15, y+60)

def gauge(p, x, y, label, sub="", col="#222", dash=False, side="below"):
    p.circ(x, y, 16, col=col, dash=dash); p.line(x, y+2, x+8, y-9, col, 2)
    p.text(x, y-22, label, 12, col, "middle", True)
    if sub and side == "right": p.text(x+22, y+4, sub, 10, "#444")
    elif sub: p.text(x, y+34, sub, 10, "#444", "middle")

def transducer(p, x, y, label, sub="", col=BLUE, dash=False, side="below"):
    p.circ(x, y, 16, col=col, dash=dash); p.text(x, y+5, "PT", 12, col, "middle", True)
    p.text(x, y-22, label, 12, col, "middle", True)
    if sub and side == "right": p.text(x+22, y+4, sub, 10, "#444")
    elif sub: p.text(x, y+34, sub, 10, "#444", "middle")

def vessel(p, x, y, w, h, label, sub="", col="#222", dash=False, fill="#f4f6f8"):
    p.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{min(w,h)//2 if w<h else 24}" fill="{fill}" stroke="{col}" stroke-width="2.5"' + (' stroke-dasharray="7 5"' if dash else '') + '/>')
    p.text(x+w/2, y+h/2-2, label, 13, col, "middle", True)
    if sub: p.text(x+w/2, y+h/2+16, sub, 10, "#444", "middle")

def check_valve(p, x, y, col="#222", dash=False, label=""):
    p.circ(x, y, 9, col=col, dash=dash); p.line(x-5, y-6, x-5, y+6, col, 2); p.add(f'<polygon points="{x+5},{y} {x-3},{y-6} {x-3},{y+6}" fill="{col}"/>')
    if label: p.text(x+14, y+4, label, 10, "#444")

def manual_valve(p, x, y, label, col="#222", dash=False):
    p.add(f'<polygon points="{x-12},{y-10} {x+12},{y+10} {x+12},{y-10} {x-12},{y+10}" fill="white" stroke="{col}" stroke-width="2"' + (' stroke-dasharray="7 5"' if dash else '') + '/>')
    p.line(x, y, x, y-20, col, 2); p.line(x-8, y-20, x+8, y-20, col, 3)
    p.text(x, y+28, label, 10, "#444", "middle")

# ================================================================= PLUMBING
def valve_up(p, x, y, label, sub, col=GRN, dash=False):
    """Vertical 2/2 solenoid valve, flow UP (the towers are fed from below). (x, y) = top-left of the symbol, 30 wide, 60 tall."""
    p.rect(x, y, 30, 30, col=col, dash=dash); p.rect(x, y+30, 30, 30, col=col, dash=dash)
    p.line(x+6, y+45, x+24, y+45, col, 1.5); p.line(x+15, y+36, x+15, y+54, col, 1.5)      # lower square: closed
    p.line(x+15, y+27, x+15, y+3, col, 2); p.add(f'<polygon points="{x+15},{y+3} {x+10},{y+10} {x+20},{y+10}" fill="{col}"/>')   # upper: open, arrow up
    p.rect(x+34, y+20, 20, 16, fill="#fff", col=col, dash=dash); p.line(x+34, y+36, x+54, y+20, col, 1.5)    # coil
    p.text(x+60, y+26, label, 12, col, "start", True)
    if sub: p.text(x+60, y+41, sub, 10, "#444")

def plumbing():
    p = Page(1500, 1000, "N2 generator - PLUMBING schematic (pneumatic)", "Tom's garage PSA nitrogen generator. Towers hold only air today (CMS adsorbent to be added). Gas flow is bottom (air in) to top (product out). Pressures in PSI.")
    p.rect(24, 74, 400, 150, fill="#fafafa", col="#999", sw=1)
    p.text(34, 94, "LEGEND", 12, "#333", bold=True)
    p.line(34, 112, 84, 112, "#222", 2.5); p.text(94, 116, "pipe / hose: seen in the photos or described by the owner", 11)
    p.line(34, 132, 84, 132, "#222", 2.5, dash=True); p.text(94, 136, "dashed + '?': ASSUMED, please confirm / correct", 11)
    p.line(34, 152, 84, 152, GRN, 2.5); p.text(94, 156, "green = valve / output driven by the Arduino", 11, GRN)
    p.line(34, 172, 84, 172, BLUE, 2.5); p.text(94, 176, "blue = pressure transducer (PT) to an Arduino analog pin", 11, BLUE)
    p.text(34, 196, "Gauge = local dial gauge    M = manual valve", 11)
    p.text(34, 214, "Names in quotes are labels as seen on the machine", 11, "#555")

    LX, RX, TT, TB = 570, 810, 330, 690
    vessel(p, LX-50, TT, 100, TB-TT, "TOWER L", "CMS to be added")
    vessel(p, RX-50, TT, 100, TB-TT, "TOWER R", "CMS to be added")
    gauge(p, LX-90, 480, "Gauge", "tower L", dash=True, side="below"); p.line(LX-50, 480, LX-74, 480, "#222", 1.5, dash=True)
    gauge(p, RX+90, 480, "Gauge", "tower R", dash=True); p.line(RX+50, 480, RX+74, 480, "#222", 1.5, dash=True)

    MY = 270
    p.poly([(LX, TT), (LX, MY), (RX, MY), (RX, TT)], "#222", 3)
    check_valve(p, LX, 300, label="check"); check_valve(p, RX, 300, label="check")
    p.circ(690, MY, 4, fill="#222")
    p.line(690, MY, 690, 200, "#222", 3)
    p.poly([(690, 200), (1030, 200)], "#222", 3)
    p.text(700, 222, "product N2 line (N2 LOW pressure)", 11, "#444")
    p.line(740, 200, 740, 160, "#222", 2); gauge(p, 740, 144, "Gauge N2 LOW", "0-30 PSI", side="right")
    p.line(840, 200, 840, 160, BLUE, 2); transducer(p, 840, 144, "PT-N2LOW", "A1  0-30 PSI", side="right")

    vessel(p, 1030, 150, 220, 100, "N2 LOW buffer tank", "\"N2 @ 10 PSI\"")
    p.line(1140, 250, 1140, 330, "#222", 3)
    p.circ(1140, 366, 36); p.text(1140, 371, "COMP", 13, "#111", "middle", True)
    p.text(1190, 336, "oil-free compressor", 10, "#444"); p.text(1190, 350, "driven by SSR (D8)", 10, GRN)
    p.line(1176, 366, 1250, 366, "#222", 3)
    p.poly([(1250, 366), (1250, 520)], "#222", 3)
    p.line(1140, 402, 1140, 430, "#222", 2, dash=True)
    p.rect(1090, 430, 100, 28, fill="#fff8e1", col="#8a6d00", dash=True); p.text(1140, 448, "unload valve", 11, "#333", "middle")
    p.text(1040, 476, "small solenoid on the compressor,", 10, "#444"); p.text(1040, 489, "NOT an Arduino output", 10, "#444")
    vessel(p, 1130, 520, 330, 100, "N2 HIGH tank", "(large grey tank, \"N2\")")
    p.line(1320, 520, 1320, 482, BLUE, 2); transducer(p, 1320, 462, "PT-N2HIGH", "A2  0-150 PSI", side="right")
    gauge(p, 1440, 480, "Gauge", "panel", side="below"); p.line(1440, 520, 1440, 496, "#222", 2)
    p.text(1136, 640, "panel on the tank: gauge, regulator (red knob), quick-connect out", 10, "#444")

    manual_valve(p, 1100, 760, "\"air / N2\" selector (olive-green lever)", dash=True)
    p.poly([(1250, 620), (1250, 700), (1100, 700), (1100, 750)], "#222", 2.5, dash=True)
    p.text(1258, 662, "N2 HIGH to the selector (?)", 10, "#444")
    p.poly([(1100, 770), (1100, 830)], "#222", 2.5, dash=True)
    p.rect(1020, 830, 160, 44, fill="#fff8e1", col="#8a6d00", dash=True); p.text(1100, 850, "Tyre-inflation air tower", 11, "#333", "middle", True); p.text(1100, 866, "(not in the photos)", 10, "#333", "middle")

    p.circ(940, 200, 4, fill="#222")
    p.line(940, 200, 940, 250, "#222", 2.5, dash=True)
    solenoid_valve(p, 925, 250, "", "", GRN, dash=True, vertical=True)
    p.text(870, 244, "V-FLUSH (O2 flush)", 12, GRN, "end", True); p.text(870, 258, "D11 (FLUSH); location ?", 10, "#444", "end")
    p.line(940, 310, 940, 340, "#222", 2.5, dash=True)
    p.rect(870, 340, 160, 50, fill="#fff8e1", col="#8a6d00", dash=True); p.text(950, 360, "O2 sensor ?", 12, "#333", "middle", True); p.text(950, 378, "DFRobot, I2C 0x74", 10, "#333", "middle")
    p.text(870, 410, "FLUSH needs N2 LOW pressure (owner).", 10, RED); p.text(870, 423, "Vent / sample path: to be confirmed.", 10, RED)

    VY = 720
    valve_up(p, LX-15, VY, "V-LEFT", "D4 (LEFT)")
    valve_up(p, RX-15, VY, "V-RIGHT", "D7 (RIGHT)")
    p.line(LX, VY, LX, TB, GRN, 3); p.line(RX, VY, RX, TB, GRN, 3)
    HY = 860
    p.line(LX, HY, LX, VY+60, "#222", 3); p.line(RX, HY, RX, VY+60, "#222", 3)
    p.line(40, HY, 960, HY, "#222", 3)
    p.text(40, HY-48, "COMPRESSED AIR IN", 12, "#333", bold=True); p.text(40, HY-33, "shop air, about 100 PSI (peaks 100-106)", 10, "#555")
    p.rect(110, HY-22, 110, 44, fill="#eef3f8"); p.text(165, HY-4, "Filter / regulator", 11, "#111", "middle", True); p.text(165, HY+12, "\"AIR @ 100 PSI\"", 10, "#444", "middle")
    p.circ(300, HY, 4, fill="#222"); p.line(300, HY, 300, HY-50, BLUE, 2); transducer(p, 300, HY-66, "PT-AIR", "A0  0-150 PSI", side="right")
    p.poly([(960, HY), (960, 760), (1088, 760)], "#222", 2.5, dash=True)
    p.text(870, 880, "air to the selector (?)", 10, "#444")
    p.text(LX+60, HY+22, "the brass tube between the towers carries", 10, "#444"); p.text(LX+60, HY+35, "the air feed from the top to these valves (?)", 10, "#444")
    p.circ(450, HY, 4, fill="#222"); p.line(450, HY, 450, 790, "#222", 2.5, dash=True)
    p.rect(415, 740, 70, 50, fill="#fff8e1", col="#8a6d00", dash=True); p.text(450, 760, "Pilot reg ?", 11, "#333", "middle", True); p.text(450, 776, "+ solenoid ?", 10, "#333", "middle")
    gauge(p, 450, 700, "Gauge", "about 10 PSI", dash=True, side="right"); p.line(450, 740, 450, 716, "#222", 2, dash=True)
    p.text(250, 696, "regulator with its own gauge", 10, "#444"); p.text(250, 709, "and a small solenoid:", 10, "#444"); p.text(250, 722, "function and wiring = ?", 10, RED)

    vessel(p, 40, 560, 200, 80, "O2 buffer tank ?", "silver, 0-100 PSI gauge", dash=True)
    gauge(p, 140, 520, "Gauge", "0-100 PSI", dash=True, side="right"); p.line(140, 560, 140, 536, "#222", 1.5, dash=True)
    p.text(40, 660, "the owner calls it 'buffer tank for the O2 released from", 10, RED); p.text(40, 673, "the tower': what it connects to is not yet known", 10, RED)

    notes = ["NOTES (Tom, please correct):",
             "1. Gas flow is drawn bottom (air in, via the two tower valves) to top (product N2 out).",
             "2. Arduino-controlled: V-LEFT D4, V-RIGHT D7, V-FLUSH D11 (location unknown), compressor via SSR D8.",
             "3. Manual / autonomous (not Arduino): lever selector, pilot regulator, unload valve, pressure switch.",
             "4. Each tower has a pressure sensor, NOT connected (their pins became N2 LOW / N2 HIGH).",
             "5. Dashed items are my guesses from photos and the owner's description."]
    for i, n in enumerate(notes):
        p.text(34, 250 + i*16, n, 11, "#222", bold=(i == 0))
    return p


# ================================================================= ELECTRICAL
def electrical():
    p = Page(1500, 1000, "N2 generator - ELECTRICAL / ELECTRONIC schematic", "Arduino UNO R4 Minima controller (production). Pins as VERIFIED at Tom's on 2026-10-09 unless dashed.")
    # legend
    p.rect(1090, 70, 392, 150, fill="#fafafa", col="#999", sw=1)
    p.text(1100, 90, "LEGEND", 12, "#333", bold=True)
    p.line(1100, 108, 1150, 108, "#222", 2.5); p.text(1160, 112, "wire / signal as described or verified", 11)
    p.line(1100, 128, 1150, 128, "#222", 2.5, dash=True); p.text(1160, 132, "dashed + '?': ASSUMED or not connected", 11)
    p.line(1100, 148, 1150, 148, RED, 2.5); p.text(1160, 152, "red = mains 120 VAC (extension-cord end)", 11, RED)
    p.line(1100, 168, 1150, 168, ORG, 2.5); p.text(1160, 172, "orange = 24 V DC", 11, ORG)
    p.line(1100, 188, 1150, 188, BLUE, 2.5); p.text(1160, 192, "blue = analog signal / 5 V sensors", 11, BLUE)
    p.line(1100, 208, 1150, 208, PUR, 2.5); p.text(1160, 212, "purple = I2C bus (SDA/SCL)", 11, PUR)

    # --- mains and PSU
    p.text(30, 100, "MAINS IN", 12, RED, bold=True)
    p.line(30, 130, 120, 130, RED, 3); p.line(30, 150, 120, 150, RED, 3)
    p.text(30, 124, "L", 11, RED); p.text(30, 168, "N", 11, RED)
    p.rect(120, 105, 60, 70, fill="#fff3f3", col=RED); p.text(150, 128, "FUSE", 11, RED, "middle", True); p.text(150, 144, "10 A", 11, RED, "middle"); p.text(150, 160, "holder ?", 10, "#444", "middle")
    p.line(180, 130, 240, 130, RED, 3); p.line(180, 150, 240, 150, RED, 3)
    p.rect(240, 100, 150, 100, fill="#fff3f3", col=RED)
    p.text(315, 122, "Mean Well MDR-60-24", 12, "#111", "middle", True); p.text(315, 140, "24 V DC / 2.5 A", 11, "#444", "middle"); p.text(315, 158, "DC OK LED (green)", 11, "#444", "middle")
    p.text(315, 176, "DIN-rail power supply", 10, "#444", "middle")
    p.line(390, 130, 460, 130, ORG, 3); p.text(425, 122, "+24 V", 11, ORG, "middle")
    p.line(390, 160, 460, 160, "#222", 3); p.text(425, 178, "0 V", 11, "#222", "middle")
    p.text(30, 218, "(Where the 10 A fuse sits - mains or 24 V side - to be confirmed)", 10, RED)

    # --- 24 V distribution
    p.line(460, 130, 700, 130, ORG, 3)
    p.line(460, 160, 700, 160, "#222", 3)

    # --- Arduino block
    p.rect(520, 260, 420, 520, fill="#e8f0fe", col=BLUE, sw=3, rx=8)
    p.text(730, 286, "ARDUINO UNO R4 MINIMA", 15, BLUE, "middle", True)
    p.text(730, 304, "controller (own supply, not USB)", 11, "#444", "middle")
    p.text(530, 324, "Arduino supply: ?", 11, RED)
    p.line(460, 160, 520, 340, "#222", 1.5, dash=True); p.text(440, 262, "power: ? (own supply)", 10, "#444", "end")

    # --- left side: inputs (switch + button)
    def pin(side, y, name, net, col="#222", dash=False):
        if side == "L":
            p.text(530, y+4, name, 12, "#111", bold=True)
            p.line(520, y, 470, y, col, 2.5, dash)
        else:
            p.text(930, y+4, name, 12, "#111", "end", True)
            p.line(940, y, 990, y, col, 2.5, dash)
    pin("L", 380, "D0  TBS", "", "#222")
    pin("L", 440, "D1  TOB", "", "#222")
    # TBS: rotary switch OFF/ON (SPDT, active LOW)
    p.circ(420, 380, 22, fill="#fff"); p.line(420, 380, 440, 368, "#222", 3); p.text(420, 350, "\"Black Switch\"", 11, "#111", "middle", True)
    p.text(420, 418, "rotary OFF / ON", 10, "#444", "middle"); p.text(420, 432, "(SPDT, active LOW)", 10, "#444", "middle")
    p.line(398, 380, 360, 380, "#222", 2.5)
    p.text(350, 384, "to GND", 10, "#444", "end")
    # TOB push button
    p.rect(396, 448+14, 48, 28, fill="#fff"); p.circ(420, 462+0, 0)
    p.line(470, 440, 444, 462, "#222", 2.5)
    p.text(420, 504, "\"OTHER\" pushbutton", 11, "#111", "middle", True); p.text(420, 520, "momentary, active LOW", 10, "#444", "middle")
    p.line(396, 462, 360, 462, "#222", 2.5); p.text(350, 466, "to GND", 10, "#444", "end")
    p.text(420, 482, "TOB", 11, "#111", "middle", True)

    # --- left side: sensors on analog pins
    pin("L", 600, "A0  AIR", "", BLUE)
    pin("L", 650, "A1  N2 LOW", "", BLUE)
    pin("L", 700, "A2  N2 HIGH", "", BLUE)
    pin("L", 750, "A3  spare", "", BLUE, True)
    for y, lab, sub, dash in ((600, "PT-AIR", "0-150 PSI", False), (650, "PT-N2LOW", "0-30 PSI", False), (700, "PT-N2HIGH", "0-150 PSI", False), (750, "tower sensor ?", "NOT connected (optional)", True)):
        p.circ(430, y, 18, col=BLUE, dash=dash); p.text(430, y+5, "PT", 12, BLUE, "middle", True)
        p.line(470, y, 448, y, BLUE, 2.5, dash)
        p.text(398, y+4, lab, 12, "#111", "end", True); p.text(398, y+18, sub, 10, "#444", "end")
    p.text(520, 575, "sensors: 3-wire 0.5-4.5 V (+5 V, GND, signal)", 11, BLUE)
    p.text(370, 800, "Tower L and tower R pressure sensors exist but are", 10, RED); p.text(370, 814, "NOT connected (pins reassigned to N2 LOW / N2 HIGH).", 10, RED)

    # --- right side: outputs
    pin("R", 380, "D4  LEFT valve", "", GRN)
    pin("R", 440, "D7  RIGHT valve", "", GRN)
    pin("R", 500, "D11 FLUSH valve", "", GRN)
    pin("R", 560, "D8  SSR", "", GRN)
    def coil(x, y, lab, sub, dash=False):
        p.rect(x, y-14, 54, 28, fill="#fff", col=GRN, dash=dash); p.line(x, y+14, x+54, y-14, GRN, 1.5)
        p.text(x+60, y-2, lab, 12, "#111", bold=True); p.text(x+60, y+13, sub, 10, "#444")
    for y, lab, sub, dash in ((380, "V-LEFT solenoid", "driver / voltage ?", False), (440, "V-RIGHT solenoid", "driver / voltage ?", False), (500, "V-FLUSH solenoid", "driver / voltage ?; location ?", True)):
        p.rect(1000, y-16, 44, 32, fill="#fff8e1", col="#8a6d00", dash=True); p.text(1022, y+4, "drv ?", 10, "#333", "middle")
        p.line(990, y, 1000, y, GRN, 2.5, True); p.line(1044, y, 1068, y, GRN, 2, True)
        coil(1068, y, lab, sub, dash)
    # SSR and compressor
    p.rect(1000, 540, 70, 40, fill="#fff3f3", col=RED, sw=2.5); p.text(1035, 558, "SSR", 13, RED, "middle", True); p.text(1035, 574, "white box", 10, "#444", "middle")
    p.line(990, 560, 1000, 560, GRN, 2.5)
    p.line(1070, 560, 1100, 560, RED, 3)
    p.text(1000, 650, "extension-cord end: mains in", 10, RED); p.text(1000, 664, "SSR is the COMPRESSOR switch (to be confirmed)", 10, RED)
    p.circ(1140, 560, 30, col=RED, sw=3); p.text(1140, 565, "M", 16, RED, "middle", True)
    p.text(1140, 610, "COMPRESSOR", 11, "#111", "middle", True)
    p.line(1170, 560, 1230, 560, RED, 3)
    p.rect(1230, 530, 110, 60, fill="#fafafa", col="#222", dash=True); p.text(1285, 555, "pressure switch", 11, "#111", "middle"); p.text(1285, 572, "box ?", 11, "#111", "middle")
    p.text(1285, 608, "unload solenoid: NOT Arduino", 10, "#444", "middle")
    p.text(1000, 678, "Mains to the SSR / compressor: its own cord; exact path ?", 10, "#444")

    # --- bottom: I2C bus (one row of devices)
    p.line(940, 720, 980, 720, PUR, 2.5); p.line(940, 740, 970, 740, PUR, 2.5)
    p.text(930, 724, "A4  SDA", 12, "#111", "end", True); p.text(930, 744, "A5  SCL", 12, "#111", "end", True)
    p.poly([(980, 720), (980, 800), (1470, 800)], PUR, 3)
    p.poly([(970, 740), (970, 808), (1470, 808)], PUR, 3)
    p.text(1000, 790, "I2C bus, 100 kHz (SDA / SCL)", 11, PUR, bold=True)
    devs = [("LCD 20x4", "PCF8574 backpack", "0x23 (A2 bridged)", False), ("LED 4-digit", "TM1650 module", "0x24, 0x34-0x37", False),
            ("O2 sensor", "DFRobot Gravity", "0x74", False), ("RTC", "DS3231", "0x68, NOT fitted", True)]
    for i, (a, b, c, d) in enumerate(devs):
        x = 1000 + i * 120
        p.rect(x, 830, 110, 90, fill="#f5f0ff", col=PUR, dash=d)
        p.text(x+55, 856, a, 12, "#111", "middle", True); p.text(x+55, 874, b, 10, "#444", "middle"); p.text(x+55, 896, c, 10, PUR, "middle")
        p.line(x+55, 808, x+55, 830, PUR, 2, d)
    p.text(30, 880, "NOTES:", 11, "#111", bold=True)
    for i, n in enumerate(["1. Pins verified at Tom's 2026-10-09: TBS D0, TOB D1, LEFT D4, RIGHT D7, SSR D8, FLUSH D11, AIR A0, N2 LOW A1, N2 HIGH A2.",
                           "2. LCD at 0x23 (backpack A2 pad bridged). The TM1650 LED module also answers at 0x24-0x27 on the same bus.",
                           "3. Valve drivers and coil voltages are NOT known to me: Tom please fill in.",
                           "4. The Arduino has its own supply (unplugging USB does not remove power): which one / how fed from the 24 V ?"]):
        p.text(30, 898 + i*16, n, 10, "#222")
    return p

if __name__ == "__main__":
    plumbing().save("plumbing_schematic_DRAFT_20261010.svg")
    electrical().save("electrical_schematic_DRAFT_20261010.svg")
    print("written")
