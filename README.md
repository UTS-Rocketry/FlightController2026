# ODIN - UTS Rocketry – Flight Controller 2026

<img width="1661" height="1668" alt="b32aecdc31f343ea89f31d750d7f50b3_T" src="https://github.com/user-attachments/assets/26e293b1-d51f-480d-ab93-dd374c82ecce" />

## Version 1.0 - First Flight Test Release

Primary avionics flight controller for the 2026 AURC rocket.

ODIN is a 3.3V STM32-based embedded flight controller designed to manage flight-state detection, onboard sensor acquisition, telemetry transmission, and flight data logging. This release represents the first hardware-and-firmware test release for an upcoming flight test.

The current firmware is intentionally simple and robust. It runs on a superloop architecture without DMA and uses a watchdog for fault recovery. The system has been tested using hardware-in-the-loop simulated flight data and has operated reliably during testing.

---

## System Overview

### Microcontroller

* STM32 F4-series microcontroller
* 3.3V logic domain
* SWD programming and debugging via ST-Link
* Optional USB device support planned for future debug and firmware workflows
* Hardware timers available for deterministic event timing
* Watchdog enabled for fault recovery

---

## Current Firmware Architecture

The first flight test firmware uses a simple superloop design.

### Current Implementation

* No RTOS
* No DMA
* Polling-based peripheral handling
* Watchdog-supervised main loop
* Deterministic flight-state logic
* Sensor sampling over shared SPI bus
* Flash-based data logging
* LoRa telemetry transmission
* Hardware-in-the-loop test data support

This architecture was selected for the first test release to reduce complexity and improve reliability during initial flight testing.

---

## Sensors

All primary sensors are currently connected on a shared SPI bus.

### Barometer

* SPI pressure sensor
* Used for altitude estimation
* Used for flight-state detection and validation
* Locally decoupled according to sensor requirements

### IMU

* SPI accelerometer and gyroscope
* Used for:

  * Boost detection
  * Apogee validation
  * Motion and orientation monitoring
* Interrupt-capable lines available for future firmware revisions

### GPS

GPS support is planned for ODIN but is not included in the current first-flight test release due to hardware issues.

Planned GPS functions include:

* Real-time position tracking
* Post-flight recovery support
* Telemetry downlink of GPS coordinates
* Position logging to onboard storage

---

## Telemetry - SX1262 LoRa

ODIN uses an SX1262-based LoRa radio for long-range telemetry transmission.

### Current Telemetry Features

* LoRa telemetry transmission enabled
* Flight-state downlink
* Sensor-derived flight data downlink
* Battery voltage downlink
* Packet scheduling is handled in the main superloop

### Planned / Future Telemetry Features

* GPS coordinate transmission
* More complete telemetry packet structure
* Configurable LoRa parameters
* Improved fault handling for radio errors

---

## Data Logging

### Current Storage

The current first flight test release uses onboard flash memory as the primary data storage medium.

Flash logging is used because the microSD card interface was dropped from the current release after wiring issues were found during testing.

### microSD Status

microSD logging was planned for this revision; however, the SD card wiring was incorrect, and the SD card interface is not used in this test release.

### Logged / Stored Parameters

The current firmware is intended to store key flight data, including:

* Timestamp or sample counter
* Raw pressure
* Calculated altitude
* IMU acceleration
* IMU gyroscope data
* Flight-state transitions
* Battery voltage
* Event flags

---

## Power Architecture

* Main system input rail
* 3.3V regulated logic rail
* ADC-based battery voltage monitoring through resistor divider
* Local decoupling at each IC
* Bulk capacitance near high-current and noise-sensitive devices
* Solid ground reference for digital logic and sensors

### Design Considerations

* All digital logic operates at 3.3V
* Shared SPI bus used for sensors and peripherals
* Short SPI traces where possible
* Source termination is considered for high-speed SPI lines
* Switching regulator noise kept away from sensitive sensor and RF sections
* Ground reference continuity is maintained across the board

---

## Programming & Debugging

### SWD Interface

SWD is the primary programming and debugging interface.

Signals:

* SWDIO
* SWCLK
* GND
* 3.3V reference
* NRST

Used for:

* Firmware flashing
* Breakpoint debugging
* Register inspection
* Fault investigation

### USB

USB support is optional and planned for future development.

Potential future uses:

* CDC virtual COM port for debug output
* DFU firmware update support
* Ground-test data streaming

---

## First Flight Test Release Notes

This release is prepared for the first ODIN flight test.

### Included

* STM32-based flight controller firmware
* Superloop firmware architecture
* Watchdog fault recovery
* SPI sensor acquisition
* Barometer-based altitude estimation
* IMU data acquisition
* LoRa telemetry transmission
* Flash-based data logging
* Battery voltage monitoring
* Hardware-in-the-loop simulated flight data testing

### Not Included in This Release

* GPS support
* microSD card logging
* DMA-based peripheral handling
* RTOS task scheduling
* Full sensor-fusion stack
* Final telemetry packet format
* Final competition firmware architecture

### Known Issues

* The microSD card interface is not used due to incorrect wiring
* GPS is not included in this release due to hardware issues
* Firmware currently uses polling rather than DMA
* Current release prioritises reliability and simplicity over final feature completeness

---

## Core Features

* Sensor initialisation and health checks
* Watchdog-supervised main loop
* Deterministic flight-state transitions
* Flash-based onboard data storage
* LoRa telemetry transmission
* Battery voltage monitoring
* Brownout and fault-awareness support
* Hardware-in-the-loop simulation data testing

---

## Development Roadmap

Future firmware and hardware revisions are expected to include:

* GPS integration
* Corrected microSD card interface
* Improved onboard logging backend
* DMA support for selected peripherals
* More complete sensor fusion
* Refined telemetry packet scheduling
* Expanded fault detection and recovery
* USB debug and DFU support
* Full competition-ready flight software

---

## Contributors
* Michael Basangan: Software Lead
* Aditya Sriram: Advisor
