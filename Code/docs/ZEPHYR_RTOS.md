# ODIN Zephyr RTOS application

This repository is the Zephyr RTOS firmware for the ODIN STM32F405RG flight
computer. The active application code lives in `Core/`, preserving the existing
subsystem layout, task periods, and radio dispatch order. The firmware targets
the Zephyr 4.2.0 API baseline.

## Build

Use a Zephyr workspace that contains the upstream
`weact_stm32f405_core` board (the target has the same STM32F405RG SoC). The
ODIN overlay disables the carrier-board peripherals and replaces every pin,
clock, bus, and chip-select setting with the values from `Code.ioc`.

```sh
west build -b weact_stm32f405_core . -d build/zephyr
west flash -d build/zephyr
```

The application expects the standard Zephyr modules, including `hal_stm32`, and
the Zephyr SDK Picolibc used by the estimator's math functions. Application code
does not allocate from the heap at runtime.

## Run a motor-profile simulation

The simulation is hardware-in-the-loop: it runs the real Zephyr application on
the STM32, but `flight_sensors.c` supplies the selected altitude and vertical
acceleration profile instead of reading the three physical sensors. The real
Kalman filter, flight-state machine, task periods, LoRa telemetry, command RX,
flash logging, GPS, CAN, and watchdog remain active.

First disconnect all igniters. Simulation builds also enforce a compile-time
pyro lockout: drogue/main fire events are logged, but the output-high and pulse
work code is not compiled into that build. Both outputs are configured inactive
at startup. Simulated continuity defaults to present so the normal ground-
station arming workflow can be exercised without continuity hardware.

`sim.conf` selects an automatically-started 2,000 m full-flight profile by
default. It scales the L1365 altitude and net acceleration consistently, reaches
2,000 m AGL, and descends to the ground. To use another existing profile,
replace its `CONFIG_ODIN_SIM_PROFILE_2000M=y` line with exactly one of:

```text
CONFIG_ODIN_SIM_PROFILE_M2050=y
CONFIG_ODIN_SIM_PROFILE_L1365=y
CONFIG_ODIN_SIM_PROFILE_L1400=y
CONFIG_ODIN_SIM_PROFILE_K1200=y
CONFIG_ODIN_SIM_PROFILE_J435=y
```

Build and flash the simulation image with:

```sh
./sim.sh
```

Or build without flashing:

```sh
west build -b weact_stm32f405_core . -d build/zephyr-sim --pristine -- \
    -DEXTRA_CONF_FILE=sim.conf
```

The default configuration automatically arms immediately before releasing the
scheduler. Set `CONFIG_ODIN_SIM_AUTO_START=n` to restore the normal ground-
station ARM workflow. The profile advances at 10 ms per sample (100 Hz), while
its altitude is consumed by the normal 40 ms barometer/Kalman release (25 Hz).
UART4 reports simulation time, FSM state, raw altitude, estimated altitude, and
vertical velocity once per second. The final sample is held after landing. The
telemetry packet format is unchanged, so the ground-station decoder needs no
simulation-specific code.

`./release.sh` still creates the normal hardware-sensor flight image in the
separate `build/zephyr` directory. `./sim.sh` always uses
`build/zephyr-sim`, preventing a cached simulation configuration from leaking
into the normal build.

## Scheduling model

| Work | Period | Zephyr context |
|---|---:|---|
| IMU + high-g read and Kalman predict | 10 ms / 100 Hz | priority-1 flight thread |
| BMP388 read and Kalman correction | 40 ms / 25 Hz | priority-1 flight thread |
| FSM update | after each fresh sensor result | priority-1 flight thread |
| Flash log snapshot | 40 ms / 25 Hz | priority-7 logger via 64-record message queue |
| Command RX | 400 ms | priority-6 serialized radio thread |
| Flight telemetry TX | 300 ms | priority-6 serialized radio thread |
| GPS TX | 1000 ms | priority-6 serialized radio thread |
| Continuity TX | 2000 ms | priority-6 serialized radio thread |
| Radio guard | 25 ms | enforced between all radio operations |
| CAN heartbeat | 500 ms while IDLE/PAD | priority-5 CAN thread |
| Pyro pulse | 500 ms | delayable work deasserts the output |

The IMU, barometer, Kalman filter, and FSM deliberately remain in one thread.
They form one ordered control/estimation pipeline and making each stage a
separate thread would add races and latency without improving deadlines. Slow
SPI2 radio and SPI3 flash work runs in lower-priority threads on separate buses.

Periodic threads use absolute uptime deadlines. A late iteration skips missed
releases instead of running a burst of stale samples, so deadlines do not drift.
The watchdog is fed only while the flight-thread heartbeat is recent.

## Buffering decision

There is no general-purpose ring buffer between the sensors and the FSM. The FSM
and telemetry need the newest coherent `FlightSensorData` snapshot; replaying old
samples would be actively harmful.

There are two bounded buffers where producer/consumer decoupling is necessary:

- Flash logging uses a 64-item `k_msgq`. A Zephyr message queue is internally a
  fixed-record ring buffer. At 25 Hz it absorbs 2.56 seconds of flash latency. If
  it fills, the oldest log sample is discarded and `odin_log_drop_count` is
  incremented; the 100 Hz flight thread never blocks on storage.
- GPS UART RX uses a 256-byte `ring_buf`, because an interrupt produces a byte
  stream independently of the NMEA parser.

Increase the flash queue only after measuring worst-case write/erase latency.
Pre-erasing 150 sectors at boot should normally keep the queue nearly empty.

## Before hardware use

This is flight-critical firmware. Before enabling igniters, verify the generated
devicetree, inspect all three pyro pins with a logic analyser, run sensor/radio
fault-injection tests, measure 100/25 Hz jitter, confirm watchdog reset behavior,
and conduct hardware-in-the-loop state-machine tests with inert loads.
