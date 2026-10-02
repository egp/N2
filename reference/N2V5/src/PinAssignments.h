// PinAssignments.h
// Physical pin assignments for the N2 Generator controller (Arduino Uno R4).
// All values are best-guess defaults — verify against actual wiring before use.
// This file is the single source of truth for pin numbers and I2C addresses.
// Edit here; do not hardcode pin numbers anywhere else.
#ifndef PIN_ASSIGNMENTS_H
#define PIN_ASSIGNMENTS_H
#include <stdint.h>

/* ── Digital inputs ──────────────────────────────────────────────────────── */
const uint8_t BLACK_SWITCH_PIN      =  2;   // INPUT_PULLUP; LOW = system ON

/* ── Digital outputs (valves and SSR) ───────────────────────────────────── */
const uint8_t LEFT_TOWER_VALVE_PIN  =  4;   // HIGH = open, LOW = closed
const uint8_t RIGHT_TOWER_VALVE_PIN =  7;   // HIGH = open, LOW = closed
const uint8_t O2_FLUSH_VALVE_PIN    = 11;   // HIGH = open, LOW = closed
const uint8_t SSR_PIN               =  8;   // HIGH = compressor on

/* ── Analog inputs (pressure sensors) ───────────────────────────────────── */
const uint8_t SUPPLY_PRESSURE_PIN   = A0;   // 0–150 PSI air supply
const uint8_t LEFT_TOWER_PIN        = A1;   // 0–150 PSI left tower
const uint8_t RIGHT_TOWER_PIN       = A2;   // 0–150 PSI right tower
const uint8_t LOW_N2_PIN            = A3;   // 0–30  PSI low-pressure N2
const uint8_t HIGH_N2_PIN           = A5;   // 0–150 PSI high-pressure N2  TODO: verify

/* ── I2C addresses (all devices share the hardware Wire bus) ─────────────── */
const uint8_t I2C_ADDR_DISP4        = 0x24; // TM1650 4-digit display  TODO: verify
const uint8_t I2C_ADDR_LCD          = 0x27; // 20x4 LCD with PCF8574   TODO: verify
const uint8_t I2C_ADDR_O2           = 0x74; // SEN0465 default (SEL dip switch = 0)

#endif
// PinAssignments.h
