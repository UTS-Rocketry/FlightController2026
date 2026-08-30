# System Overview

> **Historical document:** this describes the former CubeMX bare-metal
> implementation. The active firmware now uses Zephyr threads and drivers in the
> same `Core/` layout. See [ZEPHYR_RTOS.md](ZEPHYR_RTOS.md) for the current build,
> execution model, priorities, periods, and buffering design.

> **Purpose of this document.** Give you a complete, accurate mental model of the
> flight computer so you can explain it confidently — including the parts a
> software/controls interviewer will probe: real-time execution, the control &
> actuation path, sensor fusion, and the engineering trade-offs you made.
>
> The last section, **[Explaining this in an interview](#explaining-this-in-an-interview)**,
> is written specifically for that conversation.

---

## 1. What it is, in one paragraph

ODIN is the flight computer for a high-power rocket. It reads three motion/pressure
sensors, fuses them into a clean estimate of **altitude and vertical velocity** with a
Kalman filter, runs an **8-state flight state machine** that decides when the rocket
has launched / burned out / reached apogee / should deploy recovery, and **fires pyro
channels** to deploy drogue and main parachutes. Throughout flight it **streams telemetry
over a LoRa radio** and **logs every sample to onboard flash**. It runs bare-metal (no
RTOS) on an STM32F405 as a single timed superloop. Active control (airbrakes/fins for
apogee targeting) runs on a **separate control board (KESTREL)**, which carries its own
sensors and runs its own copy of the same Kalman filter and state machine; the two boards
coordinate over a CAN bus.

---

## 2. Hardware platform

| Block | Part | Interface | Notes |
|---|---|---|---|
| MCU | STM32F405 (Cortex-M4F) | — | 144 MHz, hardware FPU, static memory only |
| Barometer | BMP388 | **SPI1** | Primary altitude source (pressure → altitude) |
| IMU | LSM6DSOX (accel + gyro) | **SPI1** | ±16 g accel, ±2000 dps gyro, launch/burnout detect |
| High-g accel | H3LIS331DL | **SPI1** | ±200 g — survives boost when the IMU saturates |
| Radio | LoRa SX1276 | **SPI2** + EXTI | 915 MHz telemetry downlink |
| Data log | W25Q128 NOR flash | **SPI3** | 16 MB, 64-byte records, ~262 k samples |
| SD card | (FatFs) | SPI | 🧩 present but **disabled** — wired wrong (see `main.c`) |
| Pyro out | 3× MOSFET channels | GPIO | Drogue, Main, Aux igniters |
| Pyro sense | 2× continuity | ADC1, ADC2 | Checks igniter continuity before flight |
| Indicators | Buzzer + RGB LED | GPIO / TIM2 PWM | Buzzer used; RGB PWM configured, not driven |
| Console | UART4 @ 115200 | UART | `printf` is redirected here (`_write` in `main.c`) |
| GPS | (planned) | UART5 | 🧩 stub |
| Vehicle bus | CAN2 | CAN | 🧩 peripheral init'd, driver stubbed |

**Bus allocation is a deliberate design choice** and worth calling out: the three
sensors share **SPI1**, while the **radio (SPI2)** and **flash (SPI3)** each get their
own bus. That separation means a slow radio transmit can't stall a flash write (in a
concurrent design) and the high-rate sensor reads aren't competing with bulk I/O.

---

## 3. Software architecture

The code is organized in three layers under `Core/App/`, with a thin generated
HAL/CubeMX base under `Core/Src/` and `Core/Inc/`.

```
 Core/Src/main.c   ── the superloop + all peripheral init (CubeMX) + global sensorData
        │
        ▼
 ┌─────────────────────────────────────────────────────────────────────┐
 │  Outputs/                 telemetry.c · logger(stub) · indicators.c   │  what leaves the board
 ├─────────────────────────────────────────────────────────────────────┤
 │  ApplicationLayer/        flight_sensors · kalman · flight_state      │  the brains
 │                           pyro · Lora_App · crc16 · W25Q128_HAL       │
 │                           CANTasks(stub)  ·  airbrake → KESTREL board │
 ├─────────────────────────────────────────────────────────────────────┤
 │  AppDrivers/              BMP388 · lsm6dsox · h3lis331dl · LoRa        │  hardware
 │                           W25Q128 · CAN · GPS(stub) · USB(stub)       │
 └─────────────────────────────────────────────────────────────────────┘
        │
        ▼
 ST HAL + CubeMX (Core/Src) · FatFs middleware (vendor)
```

**Layering rule of thumb:** higher layers call down, never up. A driver never knows
about flight state; the FSM never touches SPI. The one place this is slightly blurred
is telemetry/logging, which is split between `Outputs/` and `ApplicationLayer/` — noted
in [Outputs.md](Outputs.md).

---

## 4. Execution model — the superloop

There is **no operating system**. `main()` initializes everything, then runs a single
infinite loop. Inside the loop, each "task" is gated by a millisecond deadline measured
against `HAL_GetTick()` (the 1 kHz SysTick). This is a **cooperative, time-sliced
scheduler in ~30 lines** — sometimes called a "timing wheel" or "superloop."

```c
while (1) {
    uint32_t now = HAL_GetTick();

    if (now - last_imu  >= 10)  { read IMU; kalman_predict(accel, 0.01); }   // 100 Hz
    if (now - last_baro >= 40)  { read baro; kalman_update(altitude);     }   //  25 Hz
    FSM_update(&sensorData);                                                  // every pass
    if (now - last_lora  >= 200 && state >= PAD) lora_tx_telemetry(...);       //   5 Hz
    if (now - last_flash >= 40  && state >= PAD) flash_log_telemetry(...);     //  25 Hz
}
```

(Source: `Core/Src/main.c` lines ~207–259.)

**Cadence summary**

| Task | Period | Rate | Gated on |
|---|---|---|---|
| IMU read + Kalman *predict* | 10 ms | 100 Hz | always |
| Baro read + Kalman *update* | 40 ms | 25 Hz | always |
| FSM update | every pass | ~loop rate | always |
| LoRa telemetry TX | 200 ms | 5 Hz | state ≥ PAD |
| Flash logging | 40 ms | 25 Hz | state ≥ PAD |

**Interrupts.** Only one application interrupt is wired: LoRa **DIO0 → EXTI9_5**, whose
handler sets `lora_tx_done_flag` (`Lora_App.c`). SysTick runs the HAL tick (and
decrements two leftover SD-card timeout counters). Everything else is polled.

**Why this matters (and why an interviewer will ask):** a superloop is simple,
deterministic, and has no scheduler overhead or stack-per-task cost — great for a first
flight computer. Its weakness is that **the loop is only as fast as its slowest blocking
call**, and any unbounded wait hangs *everything*. Several such blocking calls exist
today and are catalogued in **[CONCURRENCY_SAFETY.md](CONCURRENCY_SAFETY.md)** — that
document is essentially the "what breaks when we add real concurrency or active control"
analysis.

---

## 5. Data flow — sensor to decision to action

```
  BMP388 ─┐                               ┌─► LoRa downlink (5 Hz)
  LSM6DSOX├─► flight_sensors ─► sensorData ┼─► flash log    (25 Hz)
  H3LIS331┘     (raw + cal)       │        └─► FSM ─► pyro fire (drogue/main)
                                  │                    │
                                  └─► Kalman filter ◄──┘  (alt + velocity feed both
                                       (alt, vel)          apogee detection and the
                                                           future control loop)
```

1. **Acquire.** `flight_sensors_update_*()` reads each sensor over SPI1, applies the
   calibration offset captured at startup, and writes engineering units into one global
   `FlightSensorData sensorData` struct.
2. **Fuse.** The IMU's vertical acceleration drives `kalman_predict()`; the barometer's
   altitude drives `kalman_update()`. The filter outputs a smoothed altitude and a
   velocity estimate that the raw sensors can't give directly.
3. **Decide.** `FSM_update()` reads `sensorData` (IMU accel, Kalman velocity/altitude)
   and advances the flight state, with debounce counters so a single noisy sample can't
   trigger a transition.
4. **Act.** State entries fire pyro channels (`pyro_fire_main/drogue`). This is the
   actuation path today.
5. **Report.** Telemetry is serialized (big-endian, CRC16-protected) and pushed to both
   the radio and flash.

The single shared `sensorData` struct is the system's "blackboard." It's fine in a
superloop (one reader/writer, sequential), but it becomes the central hazard the moment
anything runs concurrently — see the safety doc.

---

## 6. The flight state machine

`flight_state.c` implements a classic entry-action state machine. Thresholds live in
`flight_config.h` so tuning never touches logic.

```
 IDLE ──(10 s auto-arm)──► PAD ──(accel ≥ 3 g ×5)──► BOOST
                                                       │ (accel ≤ 2 g ×5, or 10 s timeout)
                                                       ▼
   LAND ◄─(low alt+vel, or timeout)─ PARAFOIL ◄─ DROGUE ◄─ APOGEE ◄─ COAST
                                                   │   ▲                 │ (vel < −0.5 m/s ×10,
                                          (main fired)│                 │  or 30 s timeout)
                                                       └─ deploy logic ──┘
```

| State | Enters when | Key action |
|---|---|---|
| `IDLE` | boot | wait; auto-arms after 10 s (`ARM_AUTO_DELAY_MS`, a temporary stand-in for a LoRa arm command) |
| `PAD` | armed | watch for launch: IMU z-accel ≥ 3 g for 5 consecutive samples |
| `BOOST` | launch detected | motor burning; watch for burnout (≤ 2 g ×5) or 10 s timeout |
| `COAST` | burnout | unpowered ascent; **the window where the KESTREL board actuates airbrakes/fins** |
| `APOGEE` | velocity goes negative (×10) or 30 s timeout | record peak altitude; fire recovery |
| `DROGUE` | after apogee deploy | descend under drogue; fire main at `MAIN_DEPLOY_ALT_M` (300 m) |
| `PARAFOIL` | main deployed | controlled descent; the parafoil is itself an actuated recovery system |
| `LAND` | low altitude + near-zero velocity, or timeout | done |

**Debounce pattern.** Every transition that's driven by a sensor requires *N consecutive*
qualifying samples (`LAUNCH_CONFIRM_SAMPLES`, `APOGEE_CONFIRM_SAMPLES`, …). This is the
software equivalent of a Schmitt trigger and is exactly the kind of robustness detail
worth pointing out.

**Redundant apogee detection.** Apogee can trigger two ways — velocity sign change *or* a
hard timeout — so a sensor dropout can't strand the vehicle without deploying recovery.
The same belt-and-suspenders pattern (event *or* timeout) appears in BOOST, COAST,
DROGUE, and PARAFOIL. That defensive design is a genuine strength to highlight.

> ⚠️ **Confirmed logic bug in the low-apogee path.** At `APOGEE` entry, if the recorded
> apogee is below `MAIN_DEPLOY_ALT_M` the code fires **main** directly — an *intentional*
> low-apogee contingency (skip the drogue phase and get the main out), otherwise it fires
> **drogue**. The intent is right, but the branch sets `main_fired = 1` and then
> transitions to `STATE_DROGUE`, whose **only** exits (`flight_state.c:148,153`) both
> require `main_fired != 1`. So the FSM gets **permanently stuck in `DROGUE`**: it never
> reaches `PARAFOIL`/`LAND`, landing is never detected, and telemetry reports `DROGUE` for
> the rest of the flight (the chute still deploys — a state-machine dead-end, not a deploy
> failure). Fix: transition straight to `STATE_PARAFOIL` in the low-apogee branch. Full
> write-up in safety doc
> [M3](CONCURRENCY_SAFETY.md#-m3--low-apogee-deploy-leaves-the-fsm-stuck-in-drogue).

---

## 7. Sensor fusion — the Kalman filter

`kalman.c` is a **3-state linear Kalman filter**:

- **State:** `[ altitude, velocity, accel_bias ]`
- **Predict** (100 Hz, from IMU): integrates acceleration into velocity and altitude,
  subtracts gravity and the estimated accelerometer bias, clamps to ±16 g, and rejects
  NaN/Inf inputs.
- **Update** (25 Hz, from baro): corrects the state toward the measured altitude; the
  filter learns the accelerometer bias over time, which is what keeps the integrated
  velocity from drifting.
- **Covariance** `P` is a full 3×3 propagated as `P = F·P·Fᵀ + Q`, with the
  measurement step `K = P·Hᵀ·S⁻¹`.

**Why fuse at all?** The barometer gives absolute altitude but is noisy and slow; the
accelerometer is fast but integrating it drifts. Fusing them yields a velocity estimate
good enough to detect apogee — and, importantly, good enough to **feed a control loop**.
That velocity/altitude estimate is the natural feedback signal for airbrake/fin control.

> Implementation notes: `dt` is now computed from real elapsed time (`main.c:211`, fixed
> since the first review); the process/measurement noise (`Q`/`R`) are still fixed
> constants — a reasonable v1 choice and an easy tuning lever to mention.

---

## 8. The control and actuation path

> *The most relevant section for a controls / actuator software role.*

This is the part to be fluent in, since the role is about actuator/fin-control software.
It's useful to frame the vehicle's actuation as **three tiers**, and to be clear which
board owns each — **ODIN** (this flight computer) vs. **KESTREL** (the control board):

### Tier 1 — Pyrotechnic actuation (implemented ✅)

`pyro.c` drives three igniter channels through MOSFETs and reads two ADC continuity
checks. The FSM commands them at the right flight phase. This is *open-loop, one-shot*
actuation: fire for a fixed pulse (`PYRO_FIRE_DURATION_MS = 500 ms`) and latch a "fired"
flag so it can't double-fire.

> ⚠️ `pyro_fire_*()` holds the line high with a **blocking `HAL_Delay(500)`**, so firing
> stalls the entire superloop for half a second — no sensor reads, no estimation, no
> telemetry during that window. For pyro that's *arguably* tolerable; for a closed-loop
> actuator it is not. This is the cleanest example of why the loop must go non-blocking
> before active control is added. (Catalogued as **C1** in the safety doc.)

### Tier 2 — Continuous control surfaces / airbrakes ⟶ *runs on the KESTREL control board, not ODIN*

Active airbrake/fin control does **not** run on this flight computer. It lives on a
**separate control board (KESTREL)** with its own sensors that runs **its own copy of the
same Kalman filter and flight state machine**. ODIN's job is sensing, estimation, recovery
(pyro), logging and downlink; KESTREL independently estimates state and drives the control
surfaces. *(The empty `ApplicationLayer/ControlLoop/airbrake.{c,h}` stubs in this repo are
therefore removed (staged for deletion) — that code belongs on KESTREL.)*

**Why duplicate the KF + FSM on both boards?** So the deploy decision doesn't depend on
ODIN being alive or the CAN link being up — and, critically, so KESTREL can **gate airbrake
deployment to the `COAST` state on its own**. Airbrakes must never deploy during boost
(high-g) or the airframe could be damaged; by independently detecting burnout→COAST,
KESTREL has a self-contained safety interlock. This is **redundant / distributed
estimation**: two boards reach the flight-phase decision independently.

The controller itself (on KESTREL): feedback = Kalman altitude/velocity (+ gyro for
attitude); setpoint = a target apogee; output = a PWM servo command modulating drag to
bleed excess energy (classic apogee-targeting). A PID or gain-scheduled law on
velocity-vs-altitude is the usual approach. Authority is gated to `COAST`.

**What to say in an interview:** *"Active control runs on a dedicated board that
independently re-derives flight phase with the same Kalman filter + state machine, so the
airbrakes can only deploy in coast — even if the main flight computer or the bus drops out.
That redundant-estimation interlock is what stops a control surface from firing under boost
loads."* That shows you think about actuator **safety interlocks** and **failure
independence**, not just the control law — and that **control is only as good as the
real-time guarantees underneath it**.

### Tier 3 — Distributed actuation over CAN (planned 🧩 — `CANTasks/can_tasks.c`)

The CAN layer (`AppDrivers/CAN/CAN.h`) defines a small **vehicle bus protocol**:

- **Nodes:** `ODIN` (this flight computer), `KESTREL` (control/actuator board),
  `RAVEN` (camera), `HUGINN` (CAN sniffer/debug).
- **Messages:** heartbeat, FSM-state broadcast, **KESTREL commands** (activate /
  deactivate / **deploy parafoil**), camera commands, **actuator status**
  (`airbrake_pos`, `parafoil_dep`), ACK, error.
- **Addressing:** 11-bit ID packed as `(node << 7) | msg`.

The split the message set implies: **ODIN broadcasts flight state + estimation and issues
high-level commands (e.g. deploy-parafoil); KESTREL runs its own estimation, drives the
airbrakes/fins, and reports `airbrake_pos` / `parafoil_dep` status back.** Both boards
estimate independently (see Tier 2) — CAN coordinates them, but neither blindly trusts the
other for the safety-critical phase decision. That's a textbook *distributed-controls* bus
pattern. The peripheral is initialized (`CAN2` in `main.c`) but `CAN.c` and `can_tasks.c`
are not yet implemented — `CANTasks/` stays on ODIN (it still broadcasts state).

> ⚠️ Landmine to fix before wiring CAN in: `CAN.h` and `Lora_App.h` both define enums
> named `IDLE`, `ARMED`, … — including both in one translation unit won't compile.
> See safety doc item **C8**.

---

## 9. Communications

### LoRa telemetry downlink (✅, 🟡)

- SX1276 at **915 MHz, SF7, 125 kHz BW, CR 4/5, +17 dBm (PA_BOOST), CRC on** (`Lora_App.c`).
- **5 Hz** downlink once armed.
- Packet: a fixed **58-byte** frame — 3-byte header (sync `0xAA`, type, sequence),
  13 big-endian float fields (alt, pressure, temp, 3× high-g, 3× IMU accel, 3× gyro,
  Kalman velocity), 1 state byte, **CRC16-CCITT** (poly `0x1021`) over the first 56 bytes.
- Serialization is hand-rolled and endian-explicit (`telemetry_serializer` in
  `Lora_App.c`) — good practice for a wire format that a ground station must parse.

> ✅ The earlier `lora_tx_telemetry` buffer overflow (**M1**) is now fixed (`buff[62]`). The
> remaining radio issues are the **busy-poll that ignores the DIO0 done-interrupt**
> (**C2**/**C3**) and the new **RX command path blocking ≤1 s** (**N1**). There's also an
> authenticated uplink command path and a pre-launch continuity broadcast now (see
> [Outputs.md](Outputs.md)).

### CAN vehicle bus (🧩)

See [§8 (Tier 3)](#8-the-control-and-actuation-path).
Protocol defined, transport not yet implemented.

---

## 10. Memory & data logging

- **No heap.** Everything is statically allocated — global structs, file-scope `static`
  driver state, stack buffers. No `malloc`, no fragmentation, deterministic footprint.
  This is the right call for flight code and worth stating plainly.
- **Flash log:** `W25Q128_HAL.c` writes fixed **64-byte records** sequentially to the
  16 MB NOR flash, auto-erasing each 4 KB sector as it reaches it. ~262 000 records ≈
  ~2.9 hours at 25 Hz. `flash_dump_serial()` replays them over UART for post-flight
  analysis.
- **SD card:** a FatFs + SPI SD path exists (`fatfs_sd.c`) but is **disabled** — the
  comment in `main.c` notes it's wired wrong. The leftover `Timer1/Timer2` SysTick
  counters belong to this dead path.

---

## 11. Key engineering trade-offs (have an opinion on each)

| Decision | Chosen | Why it's defensible | The cost / when you'd change it |
|---|---|---|---|
| **Superloop vs RTOS** | Superloop | Simple, deterministic, no scheduler/stack overhead, easy to reason about for v1 | One blocking call stalls everything; no priority. Move to RTOS (or at least non-blocking + ISR/DMA) when adding closed-loop control. |
| **Polling vs interrupt/DMA SPI** | Polling | Straightforward, no race windows in a superloop | CPU burns cycles waiting; bus-bound. DMA + ISR needed for headroom once a control task exists. |
| **Static vs dynamic memory** | Static | No fragmentation, bounded RAM, flight-safe | Less flexible; you size for worst case. (Correct trade for avionics.) |
| **One shared `sensorData`** | Global blackboard | Zero copies, simple in a single-context loop | Becomes a data race the instant anything is concurrent; needs snapshotting/double-buffer. |
| **Hand-rolled telemetry serializer** | Manual big-endian | Explicit, portable wire format, no library | Verbose, easy to get a length wrong (and one is — see C2). |
| **Three sensors on one SPI bus** | SPI1 shared | Saves pins/buses; radio & flash isolated | Needs bus arbitration if reads ever move off the single loop. |

---

## 12. Explaining this in an interview

### The 60-second pitch
> "I built the flight software for a rocket flight computer on an STM32F405. It fuses a
> barometer and two accelerometers through a Kalman filter into an altitude/velocity
> estimate, runs an 8-state flight state machine that decides launch, burnout, apogee and
> recovery, and fires the parachute pyros. It logs everything to flash and streams
> telemetry over LoRa. It's bare-metal — a single timed superloop, no RTOS, all static
> memory. Active control — airbrakes for apogee targeting — runs on a separate control
> board that re-derives flight phase independently, coordinated over CAN."

### If they push on the **control / actuator** angle (your role)
- Lead with the **three actuation tiers** (§8): pyro recovery on ODIN, airbrake control on
  the KESTREL board, distributed CAN actuation tying them together.
- Emphasize that **the hard prerequisites for control already exist and work**: a clean
  velocity estimate and a state machine that defines exactly when actuation is authorized
  (the COAST window). The control board reuses both, running them **independently** so it
  can gate airbrake deployment to coast on its own — a **redundant-estimation safety
  interlock** that prevents deploying a control surface under boost (high-g) loads.
- Show you understand the *systems* dependency: **"a controller is only as good as the
  real-time loop under it."** The current superloop has blocking calls (pyro `HAL_Delay`,
  LoRa busy-poll) that would inject jitter into a control task — so step one toward
  active control is making the loop non-blocking / moving I/O to DMA+ISR. You can point
  to the exact lines.
- Mention the **CAN protocol design** (ODIN↔KESTREL, `airbrake_pos` feedback) as
  evidence you think about distributed actuation and closed-loop status, not just one MCU.

### If they push on **real-time / concurrency**
- Explain the superloop scheduler and its cadence table (§4), then be honest about its
  limits and that you've already catalogued them ([CONCURRENCY_SAFETY.md](CONCURRENCY_SAFETY.md)).
- Best signal you can give: *"Here are the exact shared-state hazards and the order I'd
  fix them in."* Walk the priority list. Interviewers love a candidate who already knows
  where their own bugs are.

### If they push on **estimation**
- 3-state KF, why fuse (baro = absolute but noisy/slow, accel = fast but drifts), how the
  bias state absorbs accelerometer offset. Mention the fixed `dt` and fixed noise as the
  obvious tuning improvements.

### Likely hard questions — and honest answers
| Question | Strong answer |
|---|---|
| "What happens if a sensor hangs?" | Today the read loops busy-wait with no timeout, so it would hang the whole computer — that's my #1 fix (add bounded timeouts; safety doc C4). |
| "Why no RTOS?" | Simplicity and determinism for v1; I'd adopt one (or non-blocking + DMA) precisely when adding the control task, because that needs a fixed-rate, jitter-bounded slot. |
| "How do you know apogee?" | Kalman velocity crossing negative, debounced over 10 samples, **with** a timeout fallback so a baro dropout still deploys recovery. |
| "Is your telemetry robust?" | CRC16 per packet and a sequence number so the ground station detects drops/corruption. (I also have a buffer-length bug in the TX path I've already identified.) |
| "How would you add airbrakes?" | On the KESTREL control board, not the flight computer: reuse the same Kalman + FSM so it independently knows it's in COAST, run a controller on velocity/altitude, output PWM to a servo, keep the loop non-blocking for a fixed rate. Gating to COAST is the key safety interlock — never actuate under boost. |

### What I'd build next (shows direction)
1. Make the loop non-blocking (kill `HAL_Delay` in pyro/buzzer; ISR/DMA for SPI & LoRa).
2. Add bounded timeouts + fault states to every sensor read.
3. Consume the LoRa DIO0 done-interrupt (C2/C3) and de-block the RX command path (N1).
4. Implement the airbrake control loop on KESTREL (its own KF+FSM, gated to the COAST window).
5. Bring up the CAN link to KESTREL for distributed actuation.

---

## 13. Glossary

| Term | Meaning |
|---|---|
| **ODIN / KESTREL / RAVEN / HUGINN** | CAN node names: flight computer / control board / camera / bus sniffer |
| **Superloop** | A single `while(1)` that runs all tasks cooperatively, no OS |
| **Apogee** | Highest point of flight; where drogue typically deploys |
| **Drogue / Main** | Small stabilizing chute (deploys high) / main chute (deploys low) |
| **Pyro** | Pyrotechnic igniter channel that cuts a line or deploys a chute |
| **ODR / OSR** | Output Data Rate / Oversampling Ratio (sensor config) |
| **EXTI** | STM32 external interrupt line (used for LoRa DIO0) |
| **SF / BW / CR** | LoRa Spreading Factor / Bandwidth / Coding Rate |

---

*See [CONCURRENCY_SAFETY.md](CONCURRENCY_SAFETY.md) for the detailed hazard list this
overview references, and the per-layer docs for module-level API detail.*
