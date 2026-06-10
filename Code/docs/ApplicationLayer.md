# ApplicationLayer — Flight Logic & Fusion

`Core/App/ApplicationLayer/`

The "brains" of the flight computer. This layer turns raw driver reads into a fused state
estimate, decides what phase of flight the vehicle is in, and commands actuation. It calls
*down* into [AppDrivers](AppDrivers.md) and is called *from* the `main.c` superloop.

```
ApplicationLayer/
├── ApplicationHALS/      flight_sensors · kalman · pyro · Lora_App · packets · W25Q128_HAL · crc16
├── FlightStateMachine/   flight_state · flight_config   (refactor → docs/FSM_DISPATCH_TABLE.md)
├── ControlLoop/          airbrake   ❌  ← REMOVED (staged): airbrake control runs on the KESTREL board
└── CANTasks/             can_tasks  🧩  ← CAN comms tasks live here (stays on ODIN)
```

---

## ApplicationHALS/

"HAL" here means *our* application-level hardware abstraction — one tier above the device
drivers. Each module wraps one or more drivers into a flight-friendly API.

### flight_sensors — sensor aggregation & calibration ✅
**Files:** `ApplicationHALS/flight_sensors.c` (.h).

Owns the three sensor handles and the calibration offsets, and presents the unified
`FlightSensorData` struct that the whole system reads/writes.

| Function | Purpose |
|---|---|
| `flight_sensors_init` | Init baro + capture ground pressure, init high-g accel + calibrate, init IMU + calibrate. Returns first failure. |
| `flight_sensors_update_baro` | Read BMP388 → `pressure`, `temperature`, `altitude` (25 Hz) |
| `flight_sensors_update_IMU_accel` | Read H3LIS + LSM6DSOX → high-g, IMU accel, gyro in engineering units, offset-corrected (100 Hz) |

**`FlightSensorData`** (`flight_sensors.h`) is the system blackboard: altitude, pressure,
temperature, velocity, high-g XYZ (mg), IMU accel XYZ (mg), gyro XYZ (mdps),
`kalman_altitude`, `kalman_velocity`, and `flight_state`. It is instantiated once as a
global in `main.c`.

> Footguns: there are **two** velocity fields (`velocity` and `kalman_velocity`) — only
> `kalman_velocity` is populated/used. The struct is shared with no snapshot protection
> ([CONCURRENCY_SAFETY C6](CONCURRENCY_SAFETY.md#-c6--the-sensordata-blackboard-is-not-snapshot-safe)).

### kalman — 3-state sensor-fusion filter ✅
**Files:** `ApplicationHALS/kalman.c` (.h).

Linear Kalman filter estimating **[altitude, velocity, accel_bias]** with a full 3×3
covariance. See [SYSTEM_OVERVIEW §7](SYSTEM_OVERVIEW.md#7-sensor-fusion--the-kalman-filter)
for the math intuition.

| Function | Called | Purpose |
|---|---|---|
| `kalman_init` | startup | Zero the state, seed covariance diagonal, set Q/R noise constants |
| `kalman_predict(accel_z_mg, dt)` | 100 Hz (IMU) | Integrate accel → velocity/altitude; subtract gravity + bias; clamp ±16 g; reject NaN/Inf |
| `kalman_update(baro_altitude)` | 25 Hz (baro) | Correct state toward measured altitude; learn accel bias |
| `kalman_get_altitude` / `kalman_get_velocity` | reads | Latest estimate (fed back into `sensorData`) |

**Inputs validated** (NaN/Inf rejected; `dt` must be `0 < dt ≤ 1`) — good defensive
coding. **Improvement levers:** `dt` is passed as a hard-coded `0.01` from `main.c`
([R2](CONCURRENCY_SAFETY.md#-r2--kalman-dt-is-hard-coded)); Q/R are fixed constants;
`ground_altitude`/`kalman_get_ground` are declared in the header but unused.

### pyro — pyrotechnic actuation ✅
**Files:** `ApplicationHALS/pyro.c` (.h).

Drives three igniter channels (drogue/main/aux) and a buzzer via GPIO, with two ADC
continuity checks. This is the **implemented actuation path** (Tier 1 in
[SYSTEM_OVERVIEW §8](SYSTEM_OVERVIEW.md#8-the-control-and-actuation-path)).

| Function | Purpose |
|---|---|
| `pyro_init` | Drive all igniters + buzzer low (safe state) |
| `pyro_check_drogue` / `pyro_check_main` | ADC continuity check vs. `PYRO_CONTINUITY_THRESHOLD` |
| `pyro_fire_drogue` / `pyro_fire_main` / `pyro_fire_aux` | Pulse the channel for `PYRO_FIRE_DURATION_MS` (500 ms) |
| `pyro_buzzer_on` / `pyro_buzzer_off` | Buzzer control |

> ⚠️ `pyro_fire_*` holds the line high with a **blocking `HAL_Delay(500)`**, freezing the
> superloop — [CONCURRENCY_SAFETY C1](CONCURRENCY_SAFETY.md#-c1--pyro-and-buzzer-block-the-loop-with-hal_delay).
> This is the cleanest reason to make the loop non-blocking before adding closed-loop
> control. The continuity checks (`pyro_check_*`) are defined but not yet called by the
> FSM — wiring them into a pre-arm check is a good easy win.

### Lora_App — radio config & DIO0 ISR ✅ 🟡
**Files:** `ApplicationHALS/Lora_App.c` (.h).

Bridges the SX1276 driver to the flight app. *(The packet **wire formats** and serializers
were moved out to `packets.c` — see the packets module below. `Lora_App` now just holds the
radio config, the DIO0 ISR, and `lora_App_Init`.)*

| Symbol | Purpose |
|---|---|
| `lora_App_Init` | Fill the SX1276 handle + LoRa config (915 MHz / SF7 / 125 kHz / +17 dBm); call `lora_init` |
| `HAL_GPIO_EXTI_Callback` | DIO0 ISR — sets `lora_tx_done_flag` |
| `FlightStateLoRa` enum | LoRa-side flight-state codes (distinct from `FlightState_t`) |

> Notes (still open): the DIO0 ISR sets `lora_tx_done_flag` but nobody reads it, and the
> header declares a misspelled `HAL_GPIOEXTI_Callback` prototype that doesn't match the
> `HAL_GPIO_EXTI_Callback` implementation
> ([C3](CONCURRENCY_SAFETY.md#-c3--lora-tx-done-interrupt-is-set-up-but-never-consumed)).
> `Lora_App.h` also still defines the duplicate `IDLE/ARMED/…` enum that collides with
> `CAN.h` ([C8](CONCURRENCY_SAFETY.md#-c8--duplicate-enum-definitions-collide-across-headers)).

### W25Q128_HAL — flash logging policy ✅
**Files:** `ApplicationHALS/W25Q128_HAL.c` (.h).

Turns the raw flash driver into an append-only record log.

| Function | Purpose |
|---|---|
| `flash_memory_init` | Bind handle to SPI3 + CS, init the chip |
| `flash_sanity_check` | Print JEDEC ID (expect `EF 40 18`) |
| `flash_log_packet(buff, len)` | Append a 64-byte record; auto-erase each 4 KB sector at its boundary; track write address + count |
| `flash_read_record(index, …)` | Random-access read of record N (for dump/replay) |
| `flash_get_record_count` | Records written this session |

**Policy:** fixed 64-byte records from address 0; ~262 144 max (16 MB / 64). `flash_write_addr`
/ `flash_record_count` are module statics (single-writer today; note for concurrency).
The flight-facing wrapper that builds the record is `flash_log_telemetry` in
[Outputs/Telemetry](Outputs.md).

### packets — packet types & (de)serialization ✅
**Files:** `ApplicationHALS/packets.c` (.h).

Owns the **wire formats** (moved here from `Lora_App.c`). Defines `HeaderPacket`,
`TelemetryPacket`, `ContinuityPacket`, `CommandPacket`, the `PacketType` / `CommandID`
enums, and the (de)serializers:

| Function | Purpose |
|---|---|
| `telemetry_serializer` | 58-byte big-endian telemetry frame + CRC16 |
| `continuity_serializer` | 8-byte pyro-continuity frame + CRC16 |
| `command_deserializer` | Validate + decode a ground command (sync + type + `0xBE` auth + CRC16) |

The command path is properly **authenticated** (auth byte + CRC) — good design. Two nits:
the `CommandPacket` struct field order disagrees with the byte offsets `command_deserializer`
actually reads, and `ContinuityPacket.aux` is declared but hardcoded to `0` in the
serializer — see [CONCURRENCY_SAFETY N3](CONCURRENCY_SAFETY.md#-n3--commandpacket-struct-disagrees-with-the-deserializers-wire-layout).

### crc16 — frame integrity ✅
**Files:** `ApplicationHALS/crc16.c` (.h).

CRC16-CCITT (poly `0x1021`), bit-serial, used to protect every telemetry/flash frame.
`crc16(seed, data, len)` — the `len` param is `uint64_t` (oversized for a 32-bit MCU, but
harmless). A sequence number + CRC per packet is what lets the ground station detect drops
and corruption.

---

## FlightStateMachine/

### flight_state — the 8-state flight FSM ✅
**Files:** `FlightStateMachine/flight_state.c` (.h).

Entry-action state machine over `FlightState_t` { IDLE, PAD, BOOST, COAST, APOGEE, DROGUE,
PARAFOIL, LAND }. Full transition table and the deployment-logic caveat are in
[SYSTEM_OVERVIEW §6](SYSTEM_OVERVIEW.md#6-the-flight-state-machine).

| Function | Purpose |
|---|---|
| `FSM_init` | Zero context, start in `IDLE` |
| `FSM_update(sensorData)` | Run current state's logic; called every superloop pass |
| `FSM_get_state` | Current state (copied into `sensorData` and telemetry) |

**Design highlights:** debounced transitions (N consecutive samples via the
`*_CONFIRM_SAMPLES` configs); **event-or-timeout** redundancy in every dynamic state so a
sensor dropout can't strand the vehicle; pyro fire is gated to state entries with
`main_fired`/`drogue_fired` flags. Context (`FSM_Context_t ctx`) is a file static
(single-context today). Per-state debounce counters are function-`static` locals — fine in
the superloop, not reentrant.

> 🔧 **Planned refactor:** replace the `switch` with a **function-pointer dispatch table** —
> full design guidance, gotchas, and migration plan in
> [FSM_DISPATCH_TABLE.md](FSM_DISPATCH_TABLE.md). Moving the debounce counters into
> `FSM_Context_t` as part of that also fixes the fact that `FSM_init` can't currently reset
> those statics.

> ⚠️ Logic bug: the low-apogee branch fires main then transitions to `DROGUE`, whose exits
> all require `main_fired != 1`, so the FSM gets **stuck in `DROGUE`** (the intentional
> low-apogee main-deploy targets the wrong next state)
> ([M3](CONCURRENCY_SAFETY.md#-m3--low-apogee-deploy-leaves-the-fsm-stuck-in-drogue)) —
> fix by transitioning to `PARAFOIL` in that branch.

### flight_config — all the tunable thresholds ✅
**Files:** `FlightStateMachine/flight_config.h`.

Pure `#define` table — **the one file you touch to tune flight behavior**, with no logic:

| Group | Constants |
|---|---|
| Launch detect | `LAUNCH_ACCEL_THRESHOLD_MG` (3 g), `LAUNCH_CONFIRM_SAMPLES` (5) |
| Burnout detect | `BURNOUT_ACCEL_THRESHOLD_MG` (2 g), `BURNOUT_CONFIRM_SAMPLES` (5) |
| Apogee detect | `APOGEE_VELOCITY_THRESHOLD` (−0.5 m/s), `APOGEE_CONFIRM_SAMPLES` (10) |
| Deploy | `MAIN_DEPLOY_ALT_M` (300 m) |
| Landing | `LAND_VELOCITY_THRESHOLD` (0.5 m/s), `LAND_ALT_THRESHOLD_M` (10 m) |
| Timeouts | `BOOST` 10 s, `COAST` 30 s, `DROGUE` 5 s, `PARAFOIL` 5 min |
| Arming | `ARM_AUTO_DELAY_MS` (10 s — temporary auto-arm; replace with a LoRa/CAN arm command) |

Keeping thresholds out of `flight_state.c` is good practice — tuning never risks a logic
regression.

---

## ControlLoop/  ❌ — *to be removed; airbrake control lives on the KESTREL board*

**Files:** `ControlLoop/airbrake.c` (.h) — **empty stubs, slated for removal.**

Airbrake/fin control does **not** run on this flight computer. It runs on the separate
**KESTREL control board**, which has its own sensors and runs its **own copy of the same
Kalman filter and FSM**, deploying airbrakes only in the `COAST` state — independently of
ODIN — so a control surface can never fire under boost (high-g) loads even if ODIN or the
CAN link drops. These empty stubs should be deleted from the flight computer to keep the
ODIN (sense/estimate/recover) vs. KESTREL (control) responsibility split clean.

See [SYSTEM_OVERVIEW §8 (Tier 2)](SYSTEM_OVERVIEW.md#8-the-control-and-actuation-path) for
the full distributed-control / redundant-estimation framing.

---

## CANTasks/  🧩

**Files:** `CANTasks/can_tasks.c` (.h) — **empty stubs.**

Intended home for the CAN communication tasks: heartbeat, broadcasting FSM state,
commanding the **KESTREL** actuator board, and ingesting actuator status. The protocol is
already defined in [`AppDrivers/CAN/CAN.h`](AppDrivers.md#can--). Implementing this is what
turns single-board actuation into the distributed
sensing-node ↔ actuation-node design
([SYSTEM_OVERVIEW §8 (Tier 3)](SYSTEM_OVERVIEW.md#8-the-control-and-actuation-path)).
Resolve the enum collision ([C8](CONCURRENCY_SAFETY.md#-c8--duplicate-enum-definitions-collide-across-headers))
first.

---

## At-a-glance status

| Module | Status |
|---|---|
| flight_sensors | ✅ |
| kalman | ✅ (dt now dynamic; Q/R still fixed) |
| pyro | ✅ (C1 blocking) |
| Lora_App | ✅ 🟡 (C3, C8) |
| packets (serializers) | ✅ 🟢 (N3 nit) |
| W25Q128_HAL | ✅ |
| crc16 | ✅ |
| flight_state / flight_config | 🟡 (M3 dead-end; refactor → FSM_DISPATCH_TABLE.md) |
| ControlLoop/airbrake | ❌ removed (staged) — control on KESTREL |
| CANTasks | 🧩 planned (stays on ODIN) |
