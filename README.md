# N2

Arduino control software for the N2 project.

## Current scope
- N2 compressor control with hysteresis/state-machine logic
- Tower valve control
- O2 sensor handling and display
- 4-digit LED and 20x4 LCD display output
- Host-testable controller components

## Main sketch
- `N2V4_260409.ino`

## Main components
- `N2Controller`
- `TowerController`
- `O2Controller`
- `TimedStateMachine`
- `ArduinoDigitalOutput`

## Hardware notes
- Arduino Uno R4 based sketch
- Multiple I2C devices and buses
- Pressure sensors for air, tower, and N2 monitoring
- SSR output for compressor control

## Status
Active development. Code is being refactored toward clearer controller boundaries and more host-testable logic.

## License
MIT