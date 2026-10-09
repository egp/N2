# bridge.py LOGFILE FIFO  - keeps the board's serial port open; appends every received line (stamped) to LOGFILE; sends every line written to FIFO.
# Run it in the background; stop it with: echo __QUIT__ > FIFO
import os, sys, termios, time, select, glob, datetime
log = open(sys.argv[1], "a", buffering=1); fifo = sys.argv[2]
if not os.path.exists(fifo): os.mkfifo(fifo)
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
fd = open_port(); cmd = os.open(fifo, os.O_RDONLY | os.O_NONBLOCK); buf = b""; cbuf = b""
while fd is not None:
    try:
        r, _, _ = select.select([fd, cmd], [], [], 0.2)
        if fd in r:
            buf += os.read(fd, 4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                log.write(f"[{stamp()}] {line.decode(errors='replace').rstrip()}\n")
        if cmd in r:
            data = os.read(cmd, 4096)
            if not data:
                os.close(cmd); cmd = os.open(fifo, os.O_RDONLY | os.O_NONBLOCK)
            cbuf += data
            while b"\n" in cbuf:
                line, cbuf = cbuf.split(b"\n", 1)
                if line.strip() == b"__QUIT__": sys.exit(0)
                os.write(fd, line + b"\n"); log.write(f"[{stamp()}] >>> {line.decode(errors='replace')}\n")
    except (OSError, ValueError):
        log.write("--- port dropped; reconnecting ---\n")
        try: os.close(fd)
        except Exception: pass
        fd = open_port()
