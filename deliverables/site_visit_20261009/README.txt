Site visit builds, 2026-10-09. Version 8.1.0 (the files built earlier were labelled 8.1.1; rebuild to get 8.1.0), commit: 471e37d (+ uncommitted at build time: see git log).
minima_diag   N2V8 DIAG   controllers OFF (the default; use this first)
minima_bench  N2V8 BENCH  controllers ON, O2 not mandatory
minima_field  N2V8 FIELD  production: controllers ON, O2 mandatory, quiet log
minima_probe  nvm_probe 1.6  bounce measurement + NVM
wifi_diag     N2V8 DIAG for the R4 WiFi bench
Upload:  arduino-cli upload -p <port> --fqbn arduino:renesas_uno:minima --input-dir <folder>   (wifi_diag: --fqbn arduino:renesas_uno:unor4wifi)
Read docs/Site_Visit_Checklist.md first.

NOTE: the .bin files are build outputs and are NOT in git (*.bin is ignored); they exist only on the Mac that built them.
If the laptop is a different machine, rebuild from the commit with the Arduino IDE (see docs/Site_Visit_Checklist.md).
sha1 (first 12): minima_diag 9428de0698dc, minima_bench 0292d754dbc1, minima_field 7a75411758ec, wifi_diag cfd63fa3c533
