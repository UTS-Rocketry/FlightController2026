# FlightComputer2026 — Documentation

Avionics flight software for **ODIN**, an STM32F405-based rocket flight computer.
This folder is the human-readable map of the firmware: what each module does, how
the system fits together, and where the code needs hardening before flight.

> **Scope.** The active firmware is a Zephyr application whose source remains
> under `Core/`. Legacy CubeMX and HAL files are retained as hardware-reference
> material but are not part of the root Zephyr build.

---

## Document map

| Document | What it covers | Read it when… |
|---|---|---|
| **[ZEPHYR_RTOS.md](ZEPHYR_RTOS.md)** | Current build, task priorities, exact periods, buffering, and hardware validation checklist. | Start here for the active firmware. |
| **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)** | Historical pre-RTOS architecture and the flight-control concepts retained by the port. | You need background on the original implementation. |
| **[CONCURRENCY_SAFETY.md](CONCURRENCY_SAFETY.md)** | Historical concurrency review that informed the Zephyr task design. | You are auditing shared state or the migration rationale. |
| **[FSM_DISPATCH_TABLE.md](FSM_DISPATCH_TABLE.md)** | Design guidance (no code) for replacing the FSM `switch` with a function-pointer dispatch table — shape, gotchas, migration plan. | You're refactoring the flight state machine. |
| **[AppDrivers.md](AppDrivers.md)** | Hardware driver layer: sensors (baro, IMU, high-g accel), LoRa radio, flash, CAN, GPS/USB stubs. | You're touching a peripheral or adding a new one. |
| **[ApplicationLayer.md](ApplicationLayer.md)** | The "brains": sensor aggregation, Kalman filter, flight state machine, pyro/recovery logic, packet (de)serialization, and CAN tasks. (The airbrake stub was removed — control runs on KESTREL.) | You're working on flight logic, fusion, or control. |
| **[Outputs.md](Outputs.md)** | Telemetry serialization, LoRa downlink, flash data logging, LEDs/buzzer. | You're changing what leaves the board (radio, flash, indicators). |

---

## 30-second architecture summary

```
              Core/Src/main.c — Zephyr initialization
                              │
              priority-1 flight/estimation thread
                    │                    │
          latest coherent snapshot    25 Hz log queue
                    │                    │
       GPS / CAN / radio / indicator   flash writer
                  worker threads         thread
```

- **MCU:** STM32F405 @ 72 MHz, Cortex-M4F, Zephyr RTOS, no runtime heap.
- **Sensing:** BMP388 barometer, LSM6DSOX IMU (accel+gyro), H3LIS331DL high-g accel — all on **SPI1**.
- **Fusion:** 3-state Kalman filter (altitude / velocity / accel-bias).
- **Decisions:** 8-state flight state machine drives pyro deployment.
- **Actuation:** pyro channels (drogue/main/aux) on this board; airbrake/fin control runs on the separate **KESTREL** control board (its own Kalman + FSM), coordinated over CAN.
- **Downlink:** LoRa (SX1276) @ 915 MHz on **SPI2**; data logging to W25Q128 16 MB flash on **SPI3**.

---

## Status legend used throughout these docs

| Symbol | Meaning |
|---|---|
| ✅ | Implemented and exercised |
| 🟡 | Implemented, not yet flight-validated / has open issues |
| 🧩 | Stub — file exists but is empty / planned |
| ⚠️ | Known bug or hazard (see [CONCURRENCY_SAFETY.md](CONCURRENCY_SAFETY.md)) |

---

*Historical documents can contain pre-Zephyr paths and should be checked against
the active source and `ZEPHYR_RTOS.md`.*
