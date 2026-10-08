Site visit builds, 2026-10-09. Sub-version 8.1.1, commit: 471e37d (+ uncommitted at build time: see git log).
minima_diag   N2V8 DIAG   controllers OFF (the default; use this first)
minima_bench  N2V8 BENCH  controllers ON, O2 not mandatory
minima_field  N2V8 FIELD  production: controllers ON, O2 mandatory, quiet log
minima_probe  nvm_probe 1.6  bounce measurement + NVM
wifi_diag     N2V8 DIAG for the R4 WiFi bench
Upload:  arduino-cli upload -p <port> --fqbn arduino:renesas_uno:minima --input-dir <folder>   (wifi_diag: --fqbn arduino:renesas_uno:unor4wifi)
Read docs/Site_Visit_Checklist.md first.
