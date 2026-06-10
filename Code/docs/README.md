# FlightComputer2026 — Documentation

Avionics flight software for **ODIN**, an STM32F405-based rocket flight computer.
This folder is the human-readable map of the firmware: what each module does, how
the system fits together, and where the code needs hardening before flight.

> **Scope.** These docs cover the *application* code under `Core/App/` plus a
> high-level view of `Core/Src/`. Vendor / auto-generated code (CubeMX init,
> ST sensor register drivers `*_reg.c`, the FatFs middleware) is intentionally
> **not** documented here — it is upstream code we did not author.

---

## Document map

| Document | What it covers | Read it when… |
|---|---|---|
| **[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md)** | End-to-end architecture, execution model, data flow, the flight state machine, sensor fusion, the actuator/control story, comms, and interview talking points. | You want the big picture, or you're prepping to explain this system out loud. |
| **[CONCURRENCY_SAFETY.md](CONCURRENCY_SAFETY.md)** | Every place the code shares state across execution contexts, the memory-safety and atomicity bugs found, and a prioritized refactor plan. | You're about to make the firmware concurrent, or you want to fix the known landmines. |
| **[FSM_DISPATCH_TABLE.md](FSM_DISPATCH_TABLE.md)** | Design guidance (no code) for replacing the FSM `switch` with a function-pointer dispatch table — shape, gotchas, migration plan. | You're refactoring the flight state machine. |
| **[AppDrivers.md](AppDrivers.md)** | Hardware driver layer: sensors (baro, IMU, high-g accel), LoRa radio, flash, CAN, GPS/USB stubs. | You're touching a peripheral or adding a new one. |
| **[ApplicationLayer.md](ApplicationLayer.md)** | The "brains": sensor aggregation, Kalman filter, flight state machine, pyro/recovery logic, packet (de)serialization, and CAN tasks. (The airbrake stub was removed — control runs on KESTREL.) | You're working on flight logic, fusion, or control. |
| **[Outputs.md](Outputs.md)** | Telemetry serialization, LoRa downlink, flash data logging, LEDs/buzzer. | You're changing what leaves the board (radio, flash, indicators). |

---

## 30-second architecture summary

```
            ┌──────────────────────────────────────────────┐
            │            main.c  —  bare-metal superloop      │
            │   (no RTOS; tasks time-sliced by HAL_GetTick)   │
            └──────────────────────────────────────────────┘
                     │            │            │
        ┌────────────▼──┐  ┌──────▼──────┐  ┌──▼────────────┐
        │  Application  │  │   Outputs   │  │  AppDrivers    │
        │    Layer      │  │             │  │                │
        │ sensors/KF/   │  │ telemetry/  │  │ baro,IMU,accel │
        │ FSM/pyro/     │  │ flash log/  │  │ LoRa,flash,CAN │
        │ CAN tasks     │  │ LED+buzzer  │  │                │
        └───────────────┘  └─────────────┘  └────────────────┘
```

- **MCU:** STM32F405 @ 144 MHz, Cortex-M4F, no dynamic memory, no OS.
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

*Generated from a read-through of the `dev` branch. File/line references are
accurate as of that read; re-verify after large refactors.*
