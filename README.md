# UTS Rocketry – Flight Controller 2026

Primary avionics flight controller for the 2026 AURC rocket.

This board manages flight state detection, sensor fusion, telemetry transmission, and onboard data logging using a 3.3V STM32-based embedded system.

---

## System Overview

### Microcontroller
- STM32 (F4 Series)
- 3.3V logic domain
- SWD debugging (ST-Link)
- Optional USB device (CDC / DFU)
- DMA-enabled peripherals
- Hardware timers for deterministic event timing

---

## Sensors

### Barometer (I2C)
- High-resolution pressure sensor
- Used for altitude estimation
- Data-ready interrupt supported (optional)
- VDD and VDDIO tied to 3.3V
- Local decoupling per datasheet

### IMU (Accelerometer + Gyroscope)
- Used for:
  - Boost detection
  - Apogee validation
  - Orientation monitoring
- Connected via SPI or I2C
- Interrupt-capable EXTI lines available

### GPS (UART)
- Real-time position tracking
- Logged to SD card
- Transmitted via LoRa telemetry
- Used for post-flight recovery

---

## Telemetry – SX1262 (SPI)

- LoRa-based long-range telemetry
- Configurable TX power & spreading factor
- SPI with DMA support
- Downlinked data:
  - Altitude
  - Velocity estimate
  - Flight state
  - GPS coordinates
  - Battery voltage

---

## Data Logging – microSD (SPI Mode)

- SPI selected over SDIO for routing simplicity and robustness
- 22Ω series resistor on SCK
- 10k pull-up on CS (DAT3)
- 0.1µF + 10µF local decoupling
- ESD protection on the connector

### Logged Parameters
- Timestamp
- Raw pressure
- Calculated altitude
- IMU acceleration & gyro
- GPS coordinates
- Flight state transitions
- Battery voltage
- Event flags

---

## Power Architecture

- Main system input rail
- 3.3V regulated logic rail
- ADC-based battery voltage monitoring (resistor divider)
- Local decoupling at each IC
- Bulk capacitance near SD and LoRa module

### Design Considerations
- All digital logic operates at 3.3V
- Short SPI traces with source termination
- Separation from switching regulator noise
- Solid ground reference plane

---

##  Programming & Debugging

### SWD (Primary Interface)
- SWDIO
- SWCLK
- GND
- 3.3V reference
- NRST (optional)

Used for:
- Firmware flashing
- Breakpoint debugging
- Register inspection

### USB (Optional)
- CDC virtual COM port for debug output
- Optional DFU firmware update support

---

### Core Features
- Sensor initialisation & health checks
- DMA-based peripheral handling
- Deterministic flight-state transitions
- SD logging with fault handling
- Telemetry packet scheduling
- Brownout and fault detection

---

# Contributors
 - Ollie Charleton: Avionics Lead
 - Michael Basangan: Software Lead
 - Aditya Sriram : Advisor
