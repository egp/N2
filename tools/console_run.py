# ser3.py SECONDS [command ...]  - reads the board's console for SECONDS, stamping each line with the Mac's clock; sends commands 2 s apart
# (the first after 1.5 s), reconnecting if the port drops. Needs no libraries.
import os, sys, termios, time, select, glob, datetime
secs = float(sys.argv[1]); cmds = sys.argv[2:]
def open_port():
    for _ in range(100):
        p = glob.glob("/dev/cu.usbmodem*")
        if p:
            try:
                fd = os.open(p[0], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                a = termios.tcgetattr(fd); a[0] = 0; a[1] = 0; a[3] = 0
                a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
                termios.tcsetattr(fd, termios.TCSANOW, a); return fd
            except Exception: pass
        time.sleep(0.3)
    return None
def stamp(): return datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
fd = open_port(); t0 = time.time(); buf = b""; nxt = 0
while fd is not None and time.time() - t0 < secs:
    if nxt < len(cmds) and time.time() - t0 > 1.5 + nxt * 2:
        c = cmds[nxt]
        os.write(fd, c.encode() + b"\n"); print(f"[{stamp()}] >>> {c}", flush=True); nxt += 1
    try:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            buf += os.read(fd, 4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                print(f"[{stamp()}] {line.decode(errors='replace').rstrip()}", flush=True)
    except (OSError, ValueError):
        print("--- port dropped; reconnecting ---", flush=True)
        try: os.close(fd)
        except Exception: pass
        fd = open_port()
