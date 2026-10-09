# Console tools (owner's Mac; not for Tom's PC)

Needs only Python 3 on macOS. They talk to the first `/dev/cu.usbmodem*` they find, at 115200, and reconnect when the board resets.

* `console_bridge.py LOGFILE FIFO` — keeps the port open, stamps every received line into LOGFILE, and sends every line written to FIFO to the board.
  Start it in the background, then, e.g.:
  `python3 tools/console_bridge.py /tmp/n2.log /tmp/n2.fifo &`  then  `echo "bist" > /tmp/n2.fifo`  and read `/tmp/n2.log`.
  Stop it with `echo __QUIT__ > /tmp/n2.fifo`. Only ONE program may hold the port: close the Arduino IDE Serial Monitor first.
* `console_run.py SECONDS [command ...]` — a one-shot: reads for SECONDS, sending the commands two seconds apart.

Capturing the console on a Windows PC instead: PuTTY (Serial, 115200) or Tera Term with logging to a file; both raise DTR, which the Minima needs to see a console attached.
Plugging the Minima into the Mac instead of Tom's PC avoids every Windows difference (symlinks, long paths, Serial Monitor settings).
