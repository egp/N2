// BuildConfig.h — which build is this, and which board is it for?
//
// Build mode (exactly one). Select by defining the macro before this header is
// included (compiler flag), or edit the default below.
//
//   N2_BUILD_HOST   macOS/Ubuntu unit tests, fake HAL      (set by test/host CMake)
//   N2_BUILD_BENCH  UNO R4 WiFi at home: real LCD/LED, simulated everything else
//   N2_BUILD_DIAG   diagnostic-only: POST + BIST + console, no controllers
//   N2_BUILD_FIELD  production
//
// The default for an Arduino build is DIAG: the first firmware taken to the
// system proves the pinouts and runs no production logic (Requirements GOAL-9).
#pragma once

// THE version (one number: major.minor.patch). Shown on the LCD row 4 in every build except FIELD, by `ver`, in the boot banner, and in the top
// comment of N2V8.ino. Bump it with every change you load onto a board, and keep N2_VERSION_HEX in step.
#define N2_VERSION "8.1.7"
#define N2_VERSION_HEX 0x0801   // stored in every NVM record (high byte major, low byte minor)

#if !defined(N2_BUILD_HOST) && !defined(N2_BUILD_BENCH) && \
    !defined(N2_BUILD_DIAG) && !defined(N2_BUILD_FIELD)
#if defined(ARDUINO)
#define N2_BUILD_DIAG
#else
#define N2_BUILD_HOST
#endif
#endif

#if (defined(N2_BUILD_HOST) + defined(N2_BUILD_BENCH) + defined(N2_BUILD_DIAG) + \
     defined(N2_BUILD_FIELD)) != 1
#error "BuildConfig.h: define exactly one of N2_BUILD_HOST/BENCH/DIAG/FIELD"
#endif

// Default console log level: verbose while debugging (HOST/BENCH/DIAG), quiet in production (FIELD).
// (Requirements CON-5: the author debugs with the console attached; production is headless.)
#if defined(N2_BUILD_FIELD)
#define N2_DEFAULT_LOG_LEVEL kInfo
#else
#define N2_DEFAULT_LOG_LEVEL kDebug
#endif

// FIELD: the O2 sensor is mandatory (Requirements O2-1). DIAG: no controllers run at all (CFG, §2).
#if defined(N2_BUILD_FIELD)
#define N2_O2_MANDATORY true
#else
#define N2_O2_MANDATORY false
#endif
#if defined(N2_BUILD_DIAG)
#define N2_CONTROLLERS_ENABLED false
#else
#define N2_CONTROLLERS_ENABLED true
#endif

// The O2 sensor library (DFRobot_MultiGasSensor) is used unmodified. Define N2_NO_DFROBOT (compiler flag)
// to build without it, e.g. in CI where the library is not installed: the O2 sensor then reads as absent.

// Board. The Arduino IDE/CLI define ARDUINO_UNOR4_MINIMA or ARDUINO_UNOR4_WIFI
// (verified with core arduino:renesas_uno 1.6.0).
#if defined(ARDUINO_UNOR4_MINIMA)
#define N2_BOARD_MINIMA
#elif defined(ARDUINO_UNOR4_WIFI)
#define N2_BOARD_WIFI
#elif defined(N2_BUILD_HOST)
#define N2_BOARD_HOST
#else
#error "BuildConfig.h: unsupported board (need UNO R4 Minima or UNO R4 WiFi)"
#endif

// BIST answer keys on the panel switches (TOB = pass, TBS ON = fail): the WiFi bench and BENCH builds only [owner 2026-10-08].
// On the production Minima the BIST is answered from the console, so a stray touch of a switch cannot answer a step.
#if defined(N2_BOARD_WIFI) || defined(N2_BUILD_BENCH) || defined(N2_BUILD_HOST)
#define N2_BIST_SWITCH_KEYS true
#else
#define N2_BIST_SWITCH_KEYS false
#endif
