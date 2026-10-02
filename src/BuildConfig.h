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
// plant proves the pinouts and runs no production logic (Requirements GOAL-9).
#pragma once

#define N2_VERSION "0.1.0-m2"

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
