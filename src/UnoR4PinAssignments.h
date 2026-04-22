// UnoR4PinAssignments.h v1
#ifndef UNO_R4_PIN_ASSIGNMENTS_H
#define UNO_R4_PIN_ASSIGNMENTS_H
#include <Arduino.h>

/*
I/O pin definitions
*/

// D14=A0 on Minima and WiFi, serves as a multi-function Analog Input
// that also acts as the board's Digital-to-Analog Converter (DAC) output
const uint8_t supplyPressurePin = A0;      // analog input pin 14

// D15=A1 on Minima, serves as an Analog Input and is specifically
// configured as the positive input (+) for the integrated Operational Amplifier (OPAMP).
// on WiFi it is mapped to GPIO15 (AN000) of the ESP32-S3 microcontroller.
// It functions as an Analog Input or a Digital I/O pin
const uint8_t leftTowerPressurePin = A1;   // analog input pin 15 Relay_1

// D16=A2 functions as GPIO, Interrupt, and PWM on the UNO R4 Minima
// on the UNO R4 WiFi it serves as the OPAMP Negative (Inverting) Input
const uint8_t rightTowerPressurePin = A2;  // analog input pin 16 Relay_2

// D17=A3 on UNO R4 Minima, functions as an Analog In and OPAMP OUT output
const uint8_t lowPressureN2Pin = A3;       // analog input pin 17

// D18=A4 on UNO R4 Minima and WiFi, is an Analog Input
// that can also serve as the SDA line for the I2C bus
const uint8_t highPressureN2Pin = A4;      // analog input pin 18

// D19=A5 on Minima, is an Analog Input and SCL pin for the I2C bus and
// is an Analog Input for the OPAMP (Operational Amplifier) negative input
// on  WiFi, it is the SCL for the primary I2C (TWI) bus and is shorted to A5
const uint8_t unusedA5 = A5;               // analog input pin 19  ** UNUSED** 

// D0 on UNO R4 WiFi and minima, is a Digital GPIO pin and UART Receive (RX) 
const uint8_t blackSwitchPin = D0;  // digital input

// D1 on Minima and WiFi,is primarily as the UART Transmit (TX) 
const uint8_t theOtherButton = D1;  // aux button

// D2 on Minima and WiFi, is a Digital GPIO pin that supports
// External Interrupts (INT0) and Capacitive Touch sensing
const uint8_t I2C_BUSA_SDA = D2;

// D3 on Minima and WiFi, is a Digital I/O pin with PWM and External Interrupt capabilities
const uint8_t I2C_BUSA_SCL = D3;

// D4 on Minima, is a Digital GPIO pin and serves as the CAN Transmitter.
// on WiFi it is configured as GPIO 4 on the and serves as an Interrupt Pin
// (specifically Interrupt 1). Additionally, D4 is utilized for CAN Bus communication
// and it is also connected to the 12×8 LED matrix used for the onboard display.
// this pin operates at 5V logic but has a strict current limit of 8 mA per pin.
const uint8_t LEFT_TOWER_VALVE_PIN = D4;  // digital output

// D5 on Minima and WiFi, serves multiple functions:
// it acts as a GPIO pin, a PWM output, and a dedicated CAN Bus line
const uint8_t I2C_BUSB_SDA = D5;

// D6 on UNO R4 (both Minima and WiFi), is a digital I/O pin that supports
// PWM (Pulse-Width Modulation) and is mapped to GTIOC0B on the Renesas RA4M1 microcontroller
const uint8_t I2C_BUSB_SCL = D6;

// D7 on UNO R4 Minim and WiFi, is a Digital GPIO pin with no dedicated secondary functions
const uint8_t RIGHT_TOWER_VALVE_PIN = D7;  // digital output

// D8 on R4 Minima and WiFi, is a standard GPIO pin
const uint8_t SSR_Pin = D8;  // digital output TODO verify correct pin Relay_3

// D9 on both Minima and WiFi, is a versatile GPIO pin capable of
// PWM output, SPI (CIPO), and Hardware Interrupts
const uint8_t I2C_BUSC_SDA = D9;

// D10 on both Minima and WiFi), serves multiple critical functions,
// primarily acting as the SPI Chip Select (CS) pin,
// which is essential for managing communication with multiple SPI devices
//  It is also a PWM-capable digital pin, allowing for variable voltage simuation,
// and on the WiFi variant, it can function as the CAN Transmitter (CANTX) pin
const uint8_t I2C_BUSC_SCL = D10;

// D11 on R4 Minima and WiFi boards, digital pin D11 functions as a PWM-capable pin
// and serves as the SPI COPI (Controller Out, Peripheral In) pin
const uint8_t I2C_BUSD_SDA = D11;

// D12 on both Minima and WiFi variants), functions as GPIO 12
// and serves as the SPI Controller In Peripheral Out (CIPO)
const uint8_t O2_FLUSH_VALVE_PIN = D12;  // PWM digital output TODO verify correct pin Relay_4

// D13 on Minima and WiFi, serves as GPIO 13, SPI Clock (SCK),
// and the Built-in LED pin.
const uint8_t I2C_BUSD_SCL = D13;
#endif

void setPinMode() {
// Analog input pins
  pinMode(supplyPressurePin, INPUT);
  pinMode(leftTowerPressurePin, INPUT);
  pinMode(rightTowerPressurePin, INPUT);
  pinMode(lowPressureN2Pin, INPUT);
  pinMode(highPressureN2Pin, INPUT);

  // Digital inputs
  pinMode(blackSwitchPin, INPUT_PULLUP);
  pinMode(theOtherButton, INPUT_PULLUP);

  // Digital outputs
  pinMode(LEFT_TOWER_VALVE_PIN, OUTPUT);
  pinMode(RIGHT_TOWER_VALVE_PIN, OUTPUT);
  pinMode(SSR_Pin, OUTPUT);
  pinMode(O2_FLUSH_VALVE_PIN, OUTPUT);

  // Digital I2C pins
  pinMode(I2C_BUSA_SDA, INPUT_PULLUP); // D2
  pinMode(I2C_BUSA_SCL, INPUT_PULLUP); // D3
  pinMode(I2C_BUSB_SDA, INPUT_PULLUP); // D5
  pinMode(I2C_BUSB_SCL, INPUT_PULLUP); // D6
  pinMode(I2C_BUSC_SDA, INPUT_PULLUP); // D9
  pinMode(I2C_BUSC_SCL, INPUT_PULLUP); // D10
  pinMode(I2C_BUSD_SDA, INPUT_PULLUP); // D12
  pinMode(I2C_BUSD_SCL, INPUT_PULLUP); // D13
}

// UnoR4PinAssignments.h v1