# N2

Arduino control software for the N2 nitrogen generator project.

## Overview

This repository contains the current controller-oriented sketch for the N2 system.

Current sketch:
- `N2V5.ino`

Primary controller components:
- `TimedStateMachine`
- `TowerController`
- `O2Controller`
- `N2Controller`

Support components:
- `SystemConfig`
- `SystemContext`
- `SystemRuntime`
- `SystemSnapshot`
- `ArduinoDigitalOutput`
- board/profile-specific `SystemProfile_*`

The code is structured so that the controller logic is host-testable, while hardware-specific behavior is kept in the sketch and the system profile files.

## What the system does

At a high level, the sketch:

1. Reads the operator enable switch ("black switch") and pressure inputs.
2. Maintains a tower-valve timing sequence for the PSA towers.
3. Samples the oxygen sensor on its own timed cycle and derives nitrogen percentage as `100 - O2`.
4. Uses low-side and high-side pressure hysteresis to decide when the compressor SSR should run.
5. Updates the 4-digit display and 20x4 LCD.
6. Supports a WiFi-board scenario mode that drives the whole system from scripted inputs instead of live hardware.

## Repository layout

Top-level files of interest:

- `N2V5.ino` - main Arduino sketch
- `TowerController.*` - tower valve controller
- `O2Controller.*` - oxygen measurement controller
- `N2Controller.*` - compressor / nitrogen pressure controller
- `TimedStateMachine.*` - shared timed-state helper
- `SystemProfile_minima.*` - live hardware profile for Uno R4 Minima
- `SystemProfile_wifi_scenario.*` - scripted scenario profile for Uno R4 WiFi
- `host_tests/` - host-side tests
- `Makefile` - host test / coverage entry points

## Hardware / bus model

The current sketch uses several separate I2C buses:

- 4-digit LED display on a dedicated bit-banged bus
- O2 sensor on a dedicated bit-banged bus
- 20x4 LCD on a dedicated bit-banged bus
- RTC on a dedicated bit-banged bus
- rotary switch through the standard `Wire` bus

Pressure inputs are read through analog pins and converted into fixed-point engineering units:

- `*_x10` means tenths
- `*_x100` means hundredths

Examples:
- `supplyPsi_x10 = 1200` means 120.0 PSI
- `lowN2Psi_x100 = 2500` means 25.00 PSI

## Build targets / profiles

This repo currently supports two board-profile modes.

### Uno R4 Minima

Selected when `ARDUINO_MINIMA` is defined.

This is the live hardware profile:
- real `millis()` clock
- real oxygen sensor implementation (`TCP0465`)
- real analog pressure reads
- real displays
- full-duration timing values intended for hardware operation

### Uno R4 WiFi scenario

Selected when `ARDUINO_UNOWIFIR4` is defined.

This is a scripted scenario profile:
- synthetic clock
- synthetic pressure inputs
- synthetic O2 sensor values
- serial-driven scenario stepping
- shortened controller timings for fast interactive testing

See [WiFi scenario mode](#wifi-scenario-mode) below.

## Theory of operation

### Main loop

The main loop is intentionally simple:

1. Refresh inputs.
   - On Minima, this means reading the black switch and the pressure sensors.
   - On WiFi scenario mode, this means applying the current scripted scenario step.
2. Apply black-switch power gating.
3. If disabled:
   - shut off displays
   - force compressor output off
   - force O2 flush valve off
   - disable the tower controller
   - return early
4. If enabled:
   - step `O2Controller`
   - enable and step `TowerController`
   - step `N2Controller`
   - refresh a consolidated system snapshot
   - update the displays

The controllers are therefore independent state machines that all consume the same `InputSnapshot`, but each owns its own internal timing and outputs.

### Black switch / operator enable

The black switch is the operator enable gate for the running system.

When the black switch is off, the sketch:
- disables the 4-digit display
- disables the 20x4 LCD
- disables the tower controller
- turns the compressor SSR output off
- turns the O2 flush valve output off

So black-switch off/on is currently an output-gating and loop-gating action, not a full controller reinitialization.

That means, for example:
- tower sequencing is explicitly restarted because `TowerController::setEnabled(false/true)` is used
- O2 warmup is **not** restarted by a black-switch off/on toggle
- N2 hysteresis state is not explicitly reset by the black switch

### TowerController

`TowerController` manages the two tower valves with a timed alternating sequence.

#### Purpose

Its job is to alternate flow between left and right towers with a short overlap interval during switchover, while also refusing to run when the supply pressure is too low.

#### States

The current state set is:

- `Inactive`
- `LeftOnly`
- `BothAfterLeft`
- `RightOnly`
- `BothAfterRight`
- `LowSupply`

#### Normal sequence

When enabled and supply pressure is sufficient, the tower controller cycles like this:

1. `LeftOnly`
2. `BothAfterLeft`
3. `RightOnly`
4. `BothAfterRight`
5. back to `LeftOnly`

This gives a timed repeating left/both/right/both pattern.

#### Output behavior

Valve outputs by state:

- `Inactive` -> both valves off
- `LowSupply` -> both valves off
- `LeftOnly` -> left on, right off
- `BothAfterLeft` -> left on, right on
- `RightOnly` -> left off, right on
- `BothAfterRight` -> left on, right on

#### Supply-pressure interlock

The controller checks `supplyPsi_x10` against `lowSupplyPsi_x10`.

Default threshold:
- 900 (`90.0 PSI`)

Behavior:
- if supply drops below threshold while enabled, the controller enters `LowSupply`
- in `LowSupply`, both tower valves are off
- when supply recovers, the controller restarts at `LeftOnly`

#### Default timing

Minima hardware profile defaults:
- `leftOpenMs = 60000`
- `overlapMs = 750`
- `rightOpenMs = 60000`

WiFi scenario profile defaults:
- `leftOpenMs = 300`
- `overlapMs = 100`
- `rightOpenMs = 300`

### O2Controller

`O2Controller` manages the oxygen sensor measurement cycle and the O2/N2 display value.

#### Purpose

Its job is to:
- initialize the oxygen sensor
- enforce warmup time
- flush the sensor line
- wait for settling
- collect multiple O2 samples
- average them
- cache the result
- mark the result fresh or stale by age
- back off briefly if a read fails

The main sketch displays nitrogen percentage as:

- `N2 % = 100.00 - O2 %`

#### States

The current state set is:

- `Uninitialized`
- `Warmup`
- `WaitingToFlush`
- `Flushing`
- `Settling`
- `Sampling`
- `WaitingForNextSample`
- `ErrorBackoff`

#### Startup behavior

During `init()` the controller:

1. forces flush valve off
2. validates `sampleCount > 0`
3. calls `sensor.begin()`
4. clears cached measurement state
5. enters `Warmup` for `warmupDurationMs`

On the Minima hardware profile, default warmup is:

- `300000 ms` (`5 minutes`)

On the WiFi scenario profile, default warmup is shortened to:

- `100 ms`

#### Measurement cycle

After warmup, the controller waits in `WaitingToFlush`.

A measurement cycle then proceeds as:

1. `Flushing`
   - flush valve on
   - wait `flushDurationMs`
2. `Settling`
   - flush valve off
   - wait `settleDurationMs`
3. `Sampling`
   - read one O2 sample
   - accumulate running sum
4. `WaitingForNextSample`
   - wait `sampleIntervalMs`
5. repeat sampling until `sampleCount` samples are collected
6. compute average O2 percentage
7. cache the result
8. record completion time
9. return to `WaitingToFlush`

#### Error handling

If a sensor read fails during sampling:
- flush valve is turned off
- the controller stores the sensor error string
- the controller enters `ErrorBackoff`
- after `errorBackoffMs`, it returns to `WaitingToFlush`

#### Freshness behavior

A cached reading is considered fresh only for a bounded time.

Minima hardware defaults:
- `measurementIntervalMs = 60000`
- `freshnessThresholdMs = 15000`
- `flushDurationMs = 3000`
- `settleDurationMs = 2000`
- `sampleIntervalMs = 250`
- `sampleCount = 10`
- `errorBackoffMs = 1000`

WiFi scenario defaults are shortened:
- `measurementIntervalMs = 500`
- `freshnessThresholdMs = 250`
- `flushDurationMs = 50`
- `settleDurationMs = 50`
- `sampleIntervalMs = 50`
- `sampleCount = 2`
- `errorBackoffMs = 100`

#### Display behavior

If no O2 value has been completed yet:
- the cached value is absent
- the sketch displays nitrogen as `0.00`

So during warmup or before the first successful measurement, the current display behavior is effectively a zero nitrogen reading rather than a placeholder string.

#### Important current black-switch behavior

`O2Controller` warmup starts during `setup()` when `o2Controller.init()` is called.

The black switch does **not** currently restart that warmup cycle, because:
- the controller is not shut down when the switch turns off
- the controller is simply not stepped while the system is disabled

When the system is re-enabled, the controller resumes from its existing timed state.

### N2Controller

`N2Controller` decides whether the compressor SSR should be on, based on low-side and high-side nitrogen pressure hysteresis.

#### Purpose

Its job is to protect against compressor chatter and to enforce a two-latch hysteresis policy:

- a **low-pressure permit latch**
- a **high-pressure permit latch**

The compressor runs only when **both** latches permit it.

#### States

The current state set is:

- `LowInhibitHighPermit`
- `LowPermitHighPermit`
- `LowPermitHighInhibit`
- `LowInhibitHighInhibit`

These names reflect the two internal permits directly.

#### Logic

The controller starts in:

- `LowInhibitHighPermit`

Each loop it updates two boolean latches:

1. low-pressure permit
2. high-pressure permit

Then it maps those latches to one of the four states.

#### Low-side hysteresis

Inputs:
- `lowN2Psi_x100`

Default thresholds:
- `lowOffPsi_x100 = 1000` (`10.00 PSI`)
- `lowOnPsi_x100 = 2000` (`20.00 PSI`)

Behavior:
- below `10.00 PSI`, low permit is forced off
- above `20.00 PSI`, low permit is forced on
- between thresholds, the previous low permit state is retained

#### High-side hysteresis

Inputs:
- `highN2Psi_x10`

Default thresholds:
- `highOnPsi_x10 = 1000` (`100.0 PSI`)
- `highOffPsi_x10 = 1200` (`120.0 PSI`)

Behavior:
- above `120.0 PSI`, high permit is forced off
- below `100.0 PSI`, high permit is forced on
- between thresholds, the previous high permit state is retained

#### Compressor output behavior

The compressor SSR is on only in:

- `LowPermitHighPermit`

In all other states, the compressor SSR is off.

#### Status meaning

Useful interpretation of the four states:

- `LowInhibitHighPermit`
  - low side says "do not run"
  - high side says "allowed if low side later permits"
- `LowPermitHighPermit`
  - both sides permit
  - compressor on
- `LowPermitHighInhibit`
  - low side wants more pressure
  - high side says stop
  - compressor off
- `LowInhibitHighInhibit`
  - neither side permits
  - compressor off

#### Current black-switch behavior

As with `O2Controller`, the main sketch does not call `n2Controller.shutdown()` when the black switch turns off.

So:
- the compressor output is forced off while disabled
- the controller object itself is not reinitialized by that event

## Display behavior

### 4-digit LED

The 4-digit display shows one selected value at a time.

On Minima hardware, selection comes from the rotary switch.

Selectable values are:
- off
- supply pressure
- left tower pressure
- right tower pressure
- low-side N2 pressure
- nitrogen percent

Decimal-point placement is controlled in software:
- tenths for `*_x10` values
- hundredths for `*_x100` values

### 20x4 LCD

The 20x4 LCD shows a live multi-line summary of the current snapshot.

Current format is:

- line 0: air supply PSI
- line 1: left/right tower PSI
- line 2: low-side N2 PSI and high-side N2 PSI
- line 3: nitrogen percent

### Displays when disabled

If the black switch is off, both displays are disabled.

## WiFi scenario mode

The Uno R4 WiFi profile is not the live hardware mode. It is a scripted controller-exercise mode.

### Purpose

It exists to let you:
- exercise controller sequencing quickly
- inspect serial output
- test state transitions without live sensors
- use a shortened timing model for fast manual stepping

### How it works

Instead of reading live hardware inputs, the WiFi scenario profile applies predefined scenario steps.

Each step sets:
- current simulated time
- black switch state
- supply pressure
- left tower pressure
- right tower pressure
- low-side N2 pressure
- high-side N2 pressure
- whether O2 data is valid
- O2 percent value

The profile also supplies:
- a synthetic `ProfileClock`
- a synthetic `ProfileO2Sensor`

### Current scenario behavior

The current scenario data is hard-coded in `SystemProfile_wifi_scenario.cpp`.

As each step is advanced:
- the profile updates the synthetic time
- the profile updates the synthetic inputs
- the sketch runs the normal controller loop against those values
- serial output prints a scenario banner with current inputs and controller states

### Serial commands

In WiFi scenario mode:

- `Enter` -> advance to the next scenario step
- `r` + `Enter` -> reset the scenario
- `?` + `Enter` -> print help / status

At the end of the scenario:
- the scenario is marked done
- the black switch is forced off

### WiFi display behavior

In WiFi scenario mode, the sketch pretends the rotary switch is set to nitrogen percent, so the 4-digit display always presents the nitrogen-percentage view instead of polling the physical rotary switch.

### WiFi timing differences

The WiFi scenario profile intentionally shortens controller timings so that:
- warmup completes quickly
- flush/settle/sample phases are easy to observe
- tower alternation is easy to step through interactively

This mode is for controller bring-up and demonstration, not for matching full-duration hardware timing.

## Host testing

The repository includes host-side tests under `host_tests/`.

The `Makefile` currently provides:

- `make host-test`
- `make coverage`
- `make clean`

The host build compiles the common controller sources and excludes the board-specific `SystemProfile_*.cpp` files.

That keeps the controller logic testable without the Arduino runtime.

## Dependencies

The sketch currently includes or depends on these libraries/components:

- Arduino core
- `Wire`
- `BitBang_I2C`
- `TCP1650`
- `TCP20x4`
- `TCP0465`
- `TCP3231`

## Current status

This repo is in active controller-centric development.

The current codebase already has:
- separated controller classes
- a shared timed-state abstraction
- host-testable logic
- two board/profile modes
- multiple dedicated I2C buses
- serial-driven WiFi scenario support

## License

MIT