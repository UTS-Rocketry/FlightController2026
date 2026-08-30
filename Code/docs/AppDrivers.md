# AppDrivers — Hardware Driver Layer

> **Historical HAL design notes:** active hardware access now uses Zephyr drivers;
> see [ZEPHYR_RTOS.md](ZEPHYR_RTOS.md) and the current files under `Core/`.

`Core/App/AppDrivers/`

The lowest layer of our own code: one module per physical peripheral. Each driver owns a
**handle struct** (which SPI bus, which CS pin) and exposes a small `HAL_StatusTypeDef`
API. Drivers know nothing about flight state — they just move bytes to and from silicon.

```
AppDrivers/
├── Sensors/     BMP388 (baro) · LSM6DSOX (IMU) · H3LIS331DL (high-g)   ── all on SPI1
├── LoRa/        SX1276 radio                                            ── SPI2 + EXTI
├── Memory/      W25Q128 NOR flash · fatfs_sd (SD, disabled)            ── SPI3 / SPI
├── CAN/         vehicle-bus protocol definitions                       ── CAN2  🧩
├── GPS/         NMEA GGA/RMC receiver                                  ── UART5
└── USB/         🧩 empty stub
```

> **Driver contract (the shared pattern).** Every device driver takes a pointer to a
> handle holding `SPI_HandleTypeDef *hspi` + `cs_port`/`cs_pin`. A private
> `platform_read/platform_write` pulls CS low, sends the register address, transfers, and
> raises CS. Read/write is distinguished by setting the register's MSB(s). This keeps the
> ST vendor `*_reg` driver layer hardware-agnostic.

---

## Sensors/

Three motion/pressure sensors, **all sharing SPI1** (distinct CS pins). They cover
complementary regimes — see [SYSTEM_OVERVIEW §2](SYSTEM_OVERVIEW.md#2-hardware-platform).

The ST sensors (LSM6DSOX, H3LIS331DL) ship with vendor register drivers
(`lsm6dsox_reg.c/.h`, `h3lis331dl_reg.c/.h`) — **not documented here**, they're upstream.
What *we* wrote are the thin `_hal.c` glue files that bind those drivers to our SPI + CS
and add init/calibration/read helpers.

### BMP388 — barometric altimeter ✅
**Files:** `Sensors/BMP388.c` (.h), self-contained driver (no vendor layer).
**Bus/CS:** SPI1 / `CSBarometer`.

| Function | Purpose |
|---|---|
| `BMP388_Init` | Verify chip ID, soft-reset, configure OSR/IIR/ODR, load factory calibration |
| `BMP388_FindGroundPressure` | Capture ground reference pressure at startup (altitude zero) |
| `BMP388_ReadRawPressTempTime` | Raw burst read of pressure/temperature/sensor-time |
| `BMP388_CompensateRawPressTemp` | Apply the datasheet compensation polynomials |
| `BMP388_FindAltitude` | Pressure → altitude (barometric formula vs. ground pressure) |
| `BMP388_ExternalReadFunction` | One-call read used by `flight_sensors`: returns pressure, temp, altitude |

**Notes:** the compensation math uses the float `Calib_data` struct loaded at init. The
data-ready wait in the read path is an unbounded `do/while` (see
[CONCURRENCY_SAFETY C4](CONCURRENCY_SAFETY.md#-c4--unbounded-sensor-busy-waits-with-no-timeout)).

### LSM6DSOX — 6-axis IMU (accel + gyro) ✅
**Files:** `Sensors/lsm6dsox_hal.c` (our glue) + `lsm6dsox_reg.c/.h` (ST, vendor).
**Bus/CS:** SPI1 / `CS_IMU`.

| Function | Purpose |
|---|---|
| `lsm6dso_init` | ID check, reset, disable I3C, **±16 g** accel, **±2000 dps** gyro, gyro LPF; ODR set to **104 Hz** |
| `lsm6dso_Calib` | Average 100 samples to capture accel/gyro zero offsets (Z accel referenced to 1 g) |
| `lsm6dso_ExternalReader` | Read raw accel + gyro (used every IMU pass) |

**Notes:** code comments say "416 Hz" but the calls set `*_ODR_104Hz` — a comment/code
mismatch worth tidying. The Z-accel from this IMU is what drives launch/burnout detection
and the Kalman predict step. `platform_read/write` here **swallow SPI errors**
(always return 0) — [CONCURRENCY_SAFETY C5](CONCURRENCY_SAFETY.md#-c5--spi-errors-are-silently-swallowed).
The data-ready wait has no timeout (worst case of [C4](CONCURRENCY_SAFETY.md#-c4--unbounded-sensor-busy-waits-with-no-timeout) — no delay either).

### H3LIS331DL — ±200 g high-g accelerometer ✅
**Files:** `Sensors/h3lis331dl_hal.c` (our glue) + `h3lis331dl_reg.c/.h` (ST, vendor).
**Bus/CS:** SPI1 / `CSAccelerometer`.

| Function | Purpose |
|---|---|
| `h3lis331dl_init` | ID check, boot, **±200 g** full scale, HP filter off, **100 Hz** ODR |
| `h3lis331dl_Calibration` | Average 100 samples for offsets (Z referenced to 1 g) |
| `h3lis331dl_externalRead` | Read raw 3-axis acceleration |

**Why it exists:** during boost the airframe can exceed the LSM6DSOX's ±16 g range; the
H3LIS keeps a valid acceleration reading when the IMU saturates. Same C4/C5 notes as the
IMU apply.

---

## LoRa/

### SX1276 — 915 MHz LoRa radio ✅ 🟡
**Files:** `LoRa/LoRa.c` (.h).
**Bus/CS:** SPI2 / `LoRaNssPin`; reset on `LoRaResetPin`; **DIO0 → EXTI9_5** for TX/RX done.

A from-scratch SX1276 driver (register map + ops). Configured by the app layer at 915 MHz,
SF7, 125 kHz, CR 4/5, +17 dBm PA_BOOST, CRC on, explicit header.

| Function | Purpose |
|---|---|
| `lora_init` | Reset, set LoRa mode, program frequency/SF/BW/CR/power/CRC/sync/preamble |
| `lora_TX` | Load FIFO, transmit, **busy-poll** `IRQ_TX_DONE` until done or timeout |
| `lora_RX` / `lora_receive_cont` / `lora_receive_cont_poll` | Single & continuous receive |
| `lora_sleep` / `lora_standby` | Power-mode control |
| `lora_packet_rssi` / `lora_packet_snr` / `lora_version` | Link diagnostics |

**Notes:** `lora_TX` busy-polls the IRQ register
([C2](CONCURRENCY_SAFETY.md#-c2--lora_tx-busy-polls-up-to-1-s)) even though the DIO0
done-interrupt is wired up but unused
([C3](CONCURRENCY_SAFETY.md#-c3--lora-tx-done-interrupt-is-set-up-but-never-consumed)).
Receive is implemented but the flight build only transmits.

---

## Memory/

### W25Q128 — 16 MB NOR flash ✅
**Files:** `Memory/W25Q128.c` (.h).
**Bus/CS:** SPI3 / `CSFlashmMemory`.

Clean, low-level driver for the Winbond W25Q128JV. **This is the reference for how to do a
blocking driver right** — note `W25Q128_WaitBusy` uses a real `HAL_GetTick()` timeout
(the pattern the sensor drivers should copy for
[C4](CONCURRENCY_SAFETY.md#-c4--unbounded-sensor-busy-waits-with-no-timeout)).

| Function | Purpose |
|---|---|
| `W25Q128_Init` | CS-high, release power-down, verify JEDEC ID (`EF 40 18`) |
| `W25Q128_ReadJEDECID` | Identity read |
| `W25Q128_ReadData` | Read N bytes from a 24-bit address |
| `W25Q128_PageProgram` | Program up to a 256-byte page (must not cross a page boundary) |
| `W25Q128_SectorErase` / `W25Q128_ChipErase` | 4 KB sector / full-chip erase |
| `W25Q128_WaitBusy` | Poll BUSY with timeout ✅ |
| `W25Q128_PowerDown` / `ReleasePowerDown` | Power management |

Geometry constants (page 256 B, sector 4 KB, 16 MB total) live in `W25Q128.h`. The
flight-facing logging wrapper is `ApplicationLayer/.../W25Q128_HAL.c` — see
[ApplicationLayer.md](ApplicationLayer.md) and [Outputs.md](Outputs.md).

### fatfs_sd — SD card over SPI 🧩 (disabled)
**Files:** `Memory/fatfs_sd.c` (.h).

A Chan-FatFs SPI SD diskio implementation. **Currently disabled** — `main.c` notes the SD
is wired wrong and `f_mount` is commented out. The `volatile uint16_t Timer1/Timer2`
counters defined here (decremented in SysTick) are this driver's I/O timeouts and are
otherwise dead weight right now. Treat this module as parked until the hardware is fixed.

---

## CAN/  🧩

### Vehicle-bus protocol
**Files:** `CAN/CAN.h` (defined), `CAN/CAN.c` (**empty stub**).
**Bus:** CAN2 (peripheral initialized in `main.c`).

No transport code yet, but the **protocol is fully specified** in the header and is the
backbone of the planned distributed-actuation design:

- **Nodes:** `ODIN` (flight computer), `KESTREL` (control/actuator board), `RAVEN`
  (camera), `HUGINN` (sniffer).
- **Messages:** heartbeat, FSM-state broadcast, KESTREL commands (activate / deactivate /
  deploy-parafoil), RAVEN/camera commands, ACK, actuator status (`airbrake_pos`,
  `parafoil_dep`), error.
- **ID format:** `CAN_ID(node,msg) = (node << 7) | msg` (11-bit standard ID).

See [SYSTEM_OVERVIEW §8 (Tier 3)](SYSTEM_OVERVIEW.md#8-the-control-and-actuation-path)
for how this fits the control story, and
[CONCURRENCY_SAFETY C8](CONCURRENCY_SAFETY.md#-c8--duplicate-enum-definitions-collide-across-headers)
for the enum-collision fix needed before implementing it.

---

## GPS/ and USB/

`GPS.c/.h` receives NMEA 0183 GGA/RMC sentences from the GPS JST connector on **UART5
(9600 baud, PC12 TX / PD2 RX)**. Reception is interrupt-driven into a ring buffer, while
checksum validation and parsing run in the main loop. Position uses signed degrees × 10^7;
altitude, speed, course, UTC, satellite count, fix quality, and receiver status are retained
for the GPS LoRa packet. `GPS2ResetPin` is pulsed low during initialization. `USBC.c/.h`
remains a placeholder.

---

## At-a-glance status

| Driver | Bus | Status |
|---|---|---|
| BMP388 | SPI1 | ✅ |
| LSM6DSOX | SPI1 | ✅ (C4/C5 notes) |
| H3LIS331DL | SPI1 | ✅ (C4/C5 notes) |
| SX1276 LoRa | SPI2 + EXTI | ✅ 🟡 (C2/C3) |
| W25Q128 flash | SPI3 | ✅ (reference impl) |
| fatfs_sd | SPI | 🧩 disabled |
| CAN | CAN2 | 🧩 protocol only |
| GPS | UART5 | ✅ NMEA GGA/RMC |
| USB | USB | 🧩 stub |
