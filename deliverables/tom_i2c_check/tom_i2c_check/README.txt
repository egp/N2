tom_i2c_check  -  OPERATOR GUIDE                                   version 1.5
=============================================================================

WHAT THIS IS
  A test program for the Arduino in the N2 generator. It checks everything
  on the generator's electronics EXCEPT the pressure sensors and the valves:

      the 20x4 LCD            the real-time clock (RTC)
      the 4-digit LED         the O2 sensor (a read-only question; see below)
      the TBS switch          the TOB button
      the RESET button

  It also sets the RTC to your computer's time (see "The RTC" below).

  This is only a test. It does NOT read the pressure sensors, and it does NOT
  drive the valves or the compressor: it never switches any output pin.

  You do NOT need the Serial Monitor. Everything is on the LCD and the LED.
  The Serial Monitor shows the same in more detail, and lets you type
  commands. It is optional.


WHAT YOU NEED
  - The Arduino UNO R4 Minima in the generator, and a USB cable to your
    Windows laptop.
  - The Arduino IDE (version 2.x) with the board package "Arduino UNO R4
    Boards" installed (version 1.6.0 was used for testing; newer should work).
      Tools > Board > Boards Manager > search "UNO R4" > Install.
  - This folder, "tom_i2c_check", with the file tom_i2c_check.ino in it.
    Keep the folder name exactly as it is: the IDE needs the folder and the
    .ino file to have the same name.


BEFORE YOU START  (read every time)
  1. THE LCD MUST HAVE ITS A2 SOLDER PAD BRIDGED, so that it answers at
     address 0x23. The LED module also answers 0x24-0x27, and an LCD left at
     its factory address 0x27 clashes with it (the LCD then shows garbage or
     nothing). Bridge only A2; leave A0 and A1 open.
  2. THIS REPLACES THE GENERATOR'S NORMAL PROGRAM on the Arduino while you
     test. This program only READS the two switch inputs (TBS and TOB) and
     talks on the two I2C wires. It never switches an output. Even so,
     SWITCH THE MACHINE OFF and take the air pressure off first.
  3. The O2 sensor is only ASKED for a reading (the same question the
     generator program asks). Nothing in the sensor is ever changed. The
     sensor needs about 5 minutes after power-on before its reading is
     steady: a reading right after power-up may be off.
  4. REMOVE POWER from the Arduino whenever you check wiring or change
     anything. The Arduino has its own power supply, so unplugging the USB
     cable is NOT enough: switch off (or unplug) that power supply AND unplug
     the USB cable. Apply power again only when you are ready to test.


RUNNING THE TEST
  1. Start the Arduino IDE. File > Open... and choose tom_i2c_check.ino.
  2. Tools > Board > Arduino UNO R4 Boards > Arduino UNO R4 Minima.
  3. Apply power to the Arduino (switch its power supply on) and plug in the
     USB cable. Tools > Port > choose the Arduino's port (the one that says
     "UNO R4 Minima").
  4. Click the Upload button (the arrow). Wait for "Done uploading".
     The time the program was compiled (your laptop's clock) is built in:
     it is what the RTC is set to.
  5. OPTIONAL: Tools > Serial Monitor, set the speed (bottom right) to
     115200 baud. If you open it after the upload you may see nothing for a
     few seconds: it prints a banner every 5 seconds until you type something.

  From here the test runs by itself and repeats.


WHAT HAPPENS, IN ORDER
  0 s      Power up. The LED shows "----" and the LCD is left alone.
  2.5 s    The LCD starts. It shows four lines (below).
  1-2 s    POST: a quick check that each device answers.
  4 s      THE TEST starts (about 105 seconds). The LED shows the step number
           (-001 ... -007) while a step runs:
    1 LCD  (about 17 s): the whole LCD fills with ####, then the letters
           ABCDEFGHIJKLMNOPQRST, then 0123456789..., then #### again. Then
           the backlight blinks. Then the text vanishes and comes back
           (display off/on). Then it waits 8 s for an answer from you
           (optional) and goes on.
    2 RTC  (about 3 s): the clock is read, 3 seconds pass, it is read again
           and must have moved by 3 seconds.
    3 LED  (about 18 s): the LED shows 8888 with one dot, then counts 0000,
           1111, 2222 ... 9999, then goes dark for one second and comes
           back. Then it waits 8 s for an answer from you (optional).
    4 O2   (about 1 s): the O2 sensor is asked for its reading. The reading
           (for example 20.90 % in room air) is shown in the Serial Monitor.
    5 TBS  (up to 20 s): FLIP THE TBS SWITCH ON, THEN OFF. The LCD shows
           what state the program sees. It passes as soon as it has seen
           both states. If you do nothing it goes on after 20 s (shown as ?).
    6 TOB  (up to 20 s): PRESS THE TOB BUTTON, THEN RELEASE IT. Same.
    7 RESET (up to 20 s): PRESS THE RESET BUTTON. The LCD shows PRESS RESET
           NOW. The program restarts (the screens go through the start-up
           again) and then reports the result: "RESET test: PASS" on row 4
           of the LCD for 20 seconds, and in the Serial Monitor. If nobody
           presses it the step ends after 20 s (shown as ?). If the power was
           removed instead of pressing RESET, the report says so (shown as ?):
           a power-up is not a reset-button press.
  ~110 s   The result stays on the LCD (row 4) for 45 seconds, then the
           test runs again, over and over.

  THE LCD (4 rows of 20 characters), normal screen:
      row 1   CHECK v1.5 TBS0 TOB0     <- the two switches, live (see below)
      row 2   LCD+ RTC+ LED+ O2+       <- one status per device
      row 3   2026-10-08 11:32:16      <- the RTC date and time
      row 4   what the test is doing, or the result

  Row 1 shows TBS1 when the TBS switch is ON and TOB1 while the TOB button is
  pressed (0 = off / released). You can use it at any time to check them.

  The status characters on row 2:
      +   the device answers: OK
      i   information only (for example "O2 not found")
      F   FAILED: the device does not answer
      -   not tested yet

  THE LED normally shows the time, HHMM, with the middle dot blinking once a
  second. (If the RTC is not working it shows the seconds since power-up.)
  If any device FAILED, FFFF alternates with the time.

  THE RESULT (row 4 of the LCD when the test has finished). Three screens
  alternate every 3 seconds:
      TEST LCD? RTCP LED?
      TEST O2P TBSP TOB?
      TEST RST?
      P   passed
      F   FAILED
      ?   no error was seen, but it is not confirmed (nobody looked at the
          LCD/LED; nobody touched the switch; the O2 sensor is not fitted)
  The RTC, O2, TBS, TOB and RST check themselves (P). The LCD and LED need YOUR
  eyes: you are the only one who can see that they look right.


THE RTC
  The program sets the RTC to the time it was compiled (your laptop's local
  clock when you pressed Upload) in two cases: the RTC lost power at some
  point, or the RTC is behind that time. After the first upload the RTC is
  ahead of the compile time, so later uploads leave it alone unless you
  compile again. It is accurate to about the minute. The Serial Monitor
  line tells you what happened:
      RTC   info  was 2020-01-01 00:00:03; SET from compile time to ...
  means it was out of date and has now been set.


WITH THE SERIAL MONITOR (optional)
  Type a command and press Enter (the box at the top must be set to
  "Both NL & CR" or "Newline"):
      help     the list of commands
      run      the whole test now: LCD, RTC, LED, O2, TBS, TOB, RESET
      lcd      the LCD test only
      rtc      the RTC test only
      led      the LED test only
      o2       the O2 sensor test only (a read-only question)
      tbs      the TBS switch test only (flip it ON and OFF)
      tob      the TOB button test only (press and release it)
      reset    the RESET button test only (press RESET within 20 s)
      scan     every device that answers on the I2C bus
      status   the results so far and the error counts
      time     the RTC date and time
      post     the quick power-up check again
      stop     stop the test and the repeats (type run to start again)
      note ... write a remark of yours into the log:  note LED digit 3 dim
  While an LCD or LED step is waiting for you, type
      p    it looked right          f    it looked wrong (add a note, e.g.
                                         f missing segment on digit 2)
  (Typing p or f is optional. No answer is fine; the test goes on by itself.)
  After you use lcd, led, rtc, o2, tbs, tob or reset the repeating test is paused so
  that it does not interrupt you. Type run to start the repeating test again.
  Every change of the switches is also written in the log: TBS ON / TBS off,
  TOB pressed / TOB released.


WHAT TO SEND BACK
  1. In the Serial Monitor: click in the text, press Ctrl+A (select all),
     Ctrl+C (copy), open Notepad, Ctrl+V (paste), save, and email the file.
     (If you did not have the Serial Monitor open, skip this.)
  2. What you saw, in a few words, for each:
        LCD   four readable rows, backlight blink, display blink?  (yes / no)
        LED   8888 with one dot, count 0-9 on all four digits, one blink?
        RTC   row 3 of the LCD showed today's date and the right time?
        TBS   row 1 changed TBS0 -> TBS1 -> TBS0 as you flipped the switch?
        TOB   row 1 changed TOB0 -> TOB1 -> TOB0 as you pressed the button?
        RESET pressing RESET restarted the screens; "RESET test: PASS"?
        O2    the reading in the log (or "not found")
        Anything unusual (garbage, a dark digit, a flickering display ...).
  3. A photo of the LCD and LED at the end helps.


IF SOMETHING LOOKS WRONG
  LCD dark/blank, LED works      The LCD is not at 0x23. Is A2 bridged? Check
                                 power and SDA/SCL. In the Serial Monitor,
                                 type scan: the LCD should appear as 0x23.
  LCD garbage or wrong text      Wait 10 s (it rewrites itself). If it
                                 stays: type lcd, then scan. 0x27 appearing
                                 WITHOUT 0x23 means A2 is not bridged.
  LED shows four dots / odd      The LCD is probably still at 0x27 (A2 not
                                 bridged): the LCD and the LED chip clash.
  LED dark                       Check its power and SDA/SCL. Type scan: the
                                 LED shows as 0x24-0x27 and 0x34-0x37.
  Row 2 shows RTCF               The RTC does not answer at 0x68. Check its
                                 power and SDA/SCL.
  Row 2 shows O2i                The O2 sensor was not found at 0x74. Check
                                 its power and its I2C wires. (Row 4 shows O2?)
  The O2 step says FAIL          The sensor answered, but not with a valid
                                 reading (the Serial Monitor says why: bad
                                 checksum, reads exactly 0.00 %, wrong gas).
  TBS or TOB never changes       Row 1 stays TBS0 / TOB0 when you operate the
                                 switch: check the wiring to pins D0 (TBS)
                                 and D1 (TOB). Each switch connects the pin
                                 to ground when ON / pressed.
  Nothing at all on the screens  Check the Arduino's power supply and the USB
                                 cable; press the RESET button.
  RESET test says "power lost"   The program restarted because power was
                                 removed, not because RESET was pressed (or
                                 the RESET button also cuts the power).
  RESET test never restarts      Pressing RESET does nothing: check the RESET
                                 button wiring to the Arduino's reset pin.

  To start again at any time, press the RESET button on the Arduino.


WHEN YOU ARE FINISHED
  Remove power from the Arduino: switch off its power supply AND unplug the
  USB cable.
