// UnoR4PinAssignments.h v1
#ifndef UNO_R4_PIN_ASSIGNMENTS_H
#define UNO_R4_PIN_ASSIGNMENTS_H
#include <Arduino.h>
/*
I/O pin definitions
*/
const uint8_t onboardLED = LED_BUILTIN;    // digital I/O pin 13
const uint8_t supplyPressurePin = A0;      // analog input pin 14
const uint8_t leftTowerPressurePin = A1;   // analog input pin 15 Relay_1
const uint8_t rightTowerPressurePin = A2;  // analog input pin 16 Relay_2
const uint8_t lowPressureN2Pin = A3;       // analog input pin 17
const uint8_t highPressureN2Pin = A4;      // analog input pin 18
const uint8_t unusedA5 = A5;               // analog input pin 19

const uint8_t blackSwitchPin = D0;  // digital input
const uint8_t theOtherButton = D1;  // aux button
const uint8_t I2C_BUSA_SDA = D2;
const uint8_t I2C_BUSA_SCL = D3;
const uint8_t LEFT_TOWER_VALVE_PIN = D4;  // digital output
const uint8_t I2C_BUSB_SDA = D5;
const uint8_t I2C_BUSB_SCL = D6;
const uint8_t RIGHT_TOWER_VALVE_PIN = D7;  // digital output
const uint8_t SSR_Pin = D8;             // digital output TODO verify correct pin Relay_3
const uint8_t I2C_BUSC_SDA = D9;
const uint8_t I2C_BUSC_SCL = D10;
const uint8_t O2_FLUSH_VALVE_PIN = D11;  // PWM digital output TODO verify correct pin Relay_4
const uint8_t I2C_BUSD_SDA = D12;
const uint8_t I2C_BUSD_SCL = D13;
const uint8_t UNAVAILABLE_D14 = D14;  // Alias for (A0 == D14, and D15 == A5)
const uint8_t UNAVAILABLE_D15 = D15;  // Alias -- tied together on both R4 Minima and R4 Wifi
#endif
// UnoR4PinAssignments.h v1