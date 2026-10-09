N2V8 site kit - for a WINDOWS laptop (Tom's) or any Arduino IDE 2 machine. Build products: regenerate from the repo, they are not in git.
  N2V8_sketch.zip            the production sketch (folder N2V8 holding N2V8.ino and src/). Unzip so you get  ...\N2V8\N2V8.ino
  DFRobot_MultiGasSensor.zip the O2 sensor library (version 3.0.0 = GitHub master), for Sketch > Include Library > Add .ZIP Library... (no internet needed; the library is NOT in the Library Manager and the IDE cannot update it). Check the version in Documents\Arduino\libraries\DFRobot_MultiGasSensor\library.properties (version=)

ONE-TIME SETUP (Arduino IDE 2.x)
 1. Tools > Board > Boards Manager: install "Arduino UNO R4 Boards" version 1.6.0 (the version the code was tested with).
 2. Sketch > Include Library > Add .ZIP Library... > DFRobot_MultiGasSensor.zip.
 3. Unzip N2V8_sketch.zip to a SHORT path, e.g.  C:\N2\   so the sketch is  C:\N2\N2V8\N2V8.ino  (Windows paths over 260 characters fail).
    Do NOT unzip it into a folder with the same name twice (C:\N2\N2V8_sketch\N2V8\...): the IDE needs the folder and the .ino to share the name.

BUILD AND UPLOAD
 4. File > Open > C:\N2\N2V8\N2V8.ino. Tools > Board > Arduino UNO R4 Boards > Arduino UNO R4 Minima. Tools > Port: the one that says UNO R4 Minima.
 5. Sketch > Verify/Compile. Write down the line  "Sketch uses NNNNNN bytes". On the owner's Mac the same sources give the same number; a different
    number means a different board package or library version.
 6. Production build: open src\BuildConfig.h and change  #define N2_BUILD_DIAG  to  #define N2_BUILD_FIELD  (the one line the sketch's WHERE TO EDIT table points to). Upload.
 7. The IDE Serial Monitor: any of "New Line", "Carriage Return", "Both NL & CR" works; 115200 baud. Type  report  and press Enter; Ctrl+A, Ctrl+C, paste into Notepad.
