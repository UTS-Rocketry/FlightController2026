# Concurrency, Memory-Safety & Atomicity — Refactor Notes

> **What this is.** A working list of where the firmware shares state, where it can
> block or hang, and where it is not memory-safe — plus the order I'd fix it in. It's
> written to be actionable: each finding has a **location**, **why it matters**, and a
> **fix**. No code is changed by this document.
>
> **Read the model first.** Today the system is a **single-context superloop** (`main.c`)
> with **one interrupt** (LoRa DIO0 → `EXTI9_5`). That means most "concurrency" bugs are
> *latent* — they don't bite while everything runs sequentially, but they will the moment
> you add an RTOS task, a DMA completion handler, or a second interrupt (e.g. a control
> loop or CAN RX). The findings below mark which are **active now** vs **latent**.
>
> **🔄 Re-review (current working tree).** Since the first pass: **M1** and **R2** are
> fixed, the per-pass hot-loop `printf` is gone, and the `ControlLoop/airbrake` stubs are
> staged for deletion. New code added a LoRa **RX command path** and a **continuity
> broadcast** (`telemetry.c`) plus a `packets.c/.h` serialization module — which introduce
> findings **N1–N3** below. Everything else (C1–C8, M3, R3, R4) still stands.
>
> **🔄 Audit #3 (commit `f925c21`).** That commit fixed the **in-flight** sensor reads
> (timeouts added to `lsm6dso_ExternalReader` 50 ms, `h3lis331dl_externalRead` 50 ms,
> `BMP388_ReadRawPressTempTime` 250 ms, IMU reset 100 ms) and cut the RX timeout 1000→50 ms
> (**N1** mitigated). The C4 fix intentionally left the two **calibration** loops
> (`lsm6dso_Calib`, `h3lis331dl_Calibration`) unbounded — **accepted** as a pre-flight
> fail-stop (see C4). New
> this pass: **N4** (LoRa TX/RX mode handling) and **NR1** (a BMP388 sensortime regression).

---

## Severity & status at a glance

**✅ Resolved / mitigated:** **M1** (`buff[62]`), **R2** (dynamic `dt`), **C4 in-flight reads**
(timeouts), **N1** (RX 50 ms). **Audit #4 (commits `cecc113`, `7d3d9f5` + working tree) adds:**
**C1** — pyro is now non-blocking via `pyro_service()` (timestamp deassert, `HAL_Delay` gone);
**M3** — low-apogee path now fires drogue+main then → `STATE_PARAFOIL` (no dead-end); **C5** —
`platform_*` now deassert CS and return an error code on SPI failure; **R4** — `Error_Handler`
now safes the pyro GPIOs; and the FSM debounce counters moved into `FSM_Context_t` (no more
switch-case statics). *(The C1/C5/M3/R4 detail sections further down describe the original
problem and are now historical.)* 🎉

> ✅ **Both now fixed in commit `b545bfd`** (verified): **P1** — the drogue typo is gone
> (`pyro.c` uses `drogue_fire_start` consistently; no `drouge` left anywhere, and the pin
> defines / `ContinuityPacket` field all align); **R1** — that same commit gated the main-loop
> sensor `printf`s behind `#ifdef DEBUG` (with `(void)` casts) and removed the per-RX prints in
> `lora_rx_command`. *(Heads-up: `b545bfd`'s message says "rename," but it also bundled these
> `printf`/`(void)` changes — worth splitting commits like that for traceability.)*

| ID | Finding | Severity | Active now? |
|----|---------|----------|-------------|
| **P1** | ✅ Fixed (`b545bfd`) — `pyro_fire_drogue` now uses `drogue_fire_start`; drogue deasserts in `pyro_service` | 🟢 Resolved | — |
| **C2** | `lora_TX` busy-polls (≤500 ms tx / ≤100 ms continuity) — still blocks the loop | 🟠 High | **Yes** |
| **R3** | No independent watchdog to recover from a hang | 🟠 High | **Yes** |
| **C1** | ✅ Fixed — `pyro_fire_*` non-blocking via `pyro_service()` *(but see **P1** for the drogue path)* | 🟢 Resolved | — |
| **C5** | ✅ Fixed — `platform_*` deassert CS + return non-zero on SPI error | 🟢 Resolved | — |
| **M3** | ✅ Fixed — low-apogee fires drogue+main → `STATE_PARAFOIL` (no dead-end) | 🟢 Resolved | — |
| **R4** | ✅ Improved — `Error_Handler` safes pyro GPIOs before `__disable_irq()` (still `while(1)`, no reset) | 🟢 Mostly | Minor |
| **C4** | In-flight reads time out ✅. Calibration loops block at boot — **accepted** (pre-flight fail-stop) | 🟢 Accepted | Boot only |
| **N1** | `lora_rx_command` RX poll 50 ms ✅ — still a blocking poll + `printf` | 🟢 Low | Yes (minor) |
| **N2** | Remote `CMD_FIRE` now hits `pyro_fire_*` state guards — but the command path **enables test-mode to bypass them** for ground pops; remote fire still doesn't set the FSM fired-flags | 🟡 Medium | By design |
| **C3** | `lora_tx_done_flag` now cleared before TX but still never *read* (TX still polls); prototype still misspelled | 🟡 Medium | Yes (waste) |
| **C8** | Duplicate `IDLE/ARMED…` enums in `CAN.h` & `Lora_App.h` → won't compile together | 🟡 Medium | Latent |
| **N4** | LoRa TX/RX error out unless already in STANDBY and don't restore it on timeout → rides on chip auto-timeout + loop timing | 🟡 Medium | Latent |
| **R1** | ✅ Largely fixed (`b545bfd`) — main-loop + FSM + RX `printf`s gated/removed. Remaining bare prints are debug-only callers / manual tools (`serial_print`, `flash_dump_serial`) + one "CMD valid" line | 🟢 Low | Minor |
| **NR1** | BMP388 sensortime bytes parsed from the wrong registers (regression) | 🟢 Low | Yes (time unused) |
| **C6** | Shared `sensorData` blackboard has no snapshot/atomicity | 🟡 Medium | Latent |
| **C7** | SPI1 shared by 3 sensors with no arbitration | 🟢 Low | Latent |
| **N3** | `CommandPacket` struct layout disagrees with `command_deserializer` wire offsets | 🟢 Low | Yes (footgun) |
| **M2** | serializers (`packets.c`) have an implicit, unchecked length contract | 🟢 Low | Yes (footgun) |

🔴 fix before next flight · 🟠 fix before adding concurrency/control · 🟡 fix soon · 🟢 cleanup

---

## Memory safety

### 🔴 M1 — Stack buffer overflow in `lora_tx_telemetry`
> **✅ RESOLVED in the current code** — `lora_tx_telemetry` now declares `uint8_t buff[62]`
> (`telemetry.c:37`) and the +4 offset is intentional (the receiver consumes the first 4
> bytes; the 58-byte payload starts at byte 4 → fits exactly in 62). Analysis kept below
> for reference. *(Apply the same care to the new `lora_tx_continuity`/`lora_rx_command`
> buffers — they currently fit: 8-byte continuity in `buff[12]`, 9-byte command in `buff[13]`.)*

**Location:** `Core/App/Outputs/Telemetry/telemetry.c:31-58`

```c
uint8_t buff[58] = {0};                    // 58 bytes: valid indices 0..57
...
telemetry_serializer(&packet, buff + 4);   // writes 58 bytes starting at +4  → buff[4..61]
result = lora_TX(buff, 62, 1000);          // sends 62 bytes               → reads buff[0..61]
```

`telemetry_serializer` always writes a fixed **58-byte** frame relative to the pointer it
is given (`Lora_App.c:44-180`, last write is `buff[57]`). Called with `buff + 4`, it
writes `buff[4]…buff[61]` — **4 bytes past the end** of a 58-byte array. Then `lora_TX`
is told the length is **62**, so it also **reads 4 bytes past the end**. This is a real
stack overwrite/over-read, not a style nit.

Note the comment *"LoRa driver consumes first 4 bytes internally"* is **not true** of the
current `lora_TX` — it writes all `length` bytes from `data[0]` into the FIFO. So the
4-byte pad is also transmitted, shifting the real payload.

**Why it matters:** corrupts whatever sits above `buff` on the stack; behavior depends on
the compiler's stack layout, so it can look fine on the bench and fault in flight. It also
means the last 4 telemetry bytes on the wire are garbage.

**Fix:** make the buffer big enough and make the offset intentional. Either size it to the
real on-wire length (`uint8_t buff[62]` if you genuinely want a 4-byte pad, then send 62)
or drop the `+4` and send 58. Decide what the 4 bytes are *for* (FIFO base address? sync
preamble?) and document it. Cross-check against the flash path, which is correct:
`flash_log_telemetry` uses `buff[64]` and serializes at offset 0 (`telemetry.c:60-76`).

### 🟢 M2 — `telemetry_serializer` has an implicit length contract
**Location:** `Core/App/ApplicationLayer/ApplicationHALS/Lora_App.c:44`

The serializer takes only `(packet, buff)` — no length — and unconditionally writes 58
bytes. The caller must *just know* to pass ≥58 bytes from the pointer. That implicit
contract is exactly what made **M1** possible.

**Fix:** pass the destination length and bounds-check (`telemetry_serializer(packet, buff,
buflen)` returning bytes written, or `static_assert`/comment the required size at every
call site). Cheap insurance against the next caller.

### 🟠 M3 — Low-apogee deploy leaves the FSM stuck in `DROGUE`
**Location:** `Core/App/ApplicationLayer/FlightStateMachine/flight_state.c:121-159`

At `APOGEE` entry the deploy logic branches on apogee altitude:

```c
if (ctx.apogee_alt < MAIN_DEPLOY_ALT_M) { pyro_fire_main();   ctx.main_fired = 1; }  // low-apogee contingency
else                                    { pyro_fire_drogue(); ctx.drogue_fired = 1; }
FSM_transition(STATE_DROGUE);
```

The low-apogee branch is **intentional and sensible** (per the author): if you're already
below the main-deploy altitude at apogee, there's no useful drogue-descent phase, so you
fire main immediately. The bug is the **transition target** — *both* branches go to
`STATE_DROGUE`, but every exit from `STATE_DROGUE` is gated on `main_fired != 1`
(`:148` and `:153`):

```c
if (alt < MAIN_DEPLOY_ALT_M && ctx.main_fired != 1) { ... FSM_transition(STATE_PARAFOIL); }
if (timeout                 && ctx.main_fired != 1) { ... FSM_transition(STATE_PARAFOIL); }
```

In the low-apogee case `main_fired` is already `1` on entry, so **neither exit can ever
fire** and the FSM is **permanently stuck in `DROGUE`**. The main chute still deploys —
this is a state-machine dead-end, not a deployment failure — but `PARAFOIL`/`LAND` are
never reached, so landing is never detected, any parafoil-deploy logic never runs, and
telemetry/logging report `DROGUE` for the rest of the flight. It triggers on a genuinely
low apogee *or* a baro/Kalman under-read at apogee, so fix it before any flight where a
sub-`MAIN_DEPLOY_ALT_M` apogee is possible (including a motor anomaly).

**Fix:** in the low-apogee branch, transition straight to `STATE_PARAFOIL` (post-main
descent) instead of `STATE_DROGUE`, so the normal landing watch takes over:

```c
if (ctx.apogee_alt < MAIN_DEPLOY_ALT_M) { pyro_fire_main();   ctx.main_fired = 1;   FSM_transition(STATE_PARAFOIL); }
else                                    { pyro_fire_drogue(); ctx.drogue_fired = 1; FSM_transition(STATE_DROGUE);   }
```

While here, consider centralizing deploy in one helper that refuses to re-fire a channel
whose flag is already set, so this path stays auditable. (See
[SYSTEM_OVERVIEW.md §6](SYSTEM_OVERVIEW.md#6-the-flight-state-machine).)

---

## Blocking calls (kill these before any control loop)

A closed-loop airbrake/fin controller needs a **fixed-rate, jitter-bounded** slot. Every
blocking call below injects latency into the one loop that would host it.

### 🔴 C4 — Unbounded sensor busy-waits with no timeout
> **🔄 Audit #3 — partially fixed.** The **in-flight read paths are now FIXED** (commit
> `f925c21`): `lsm6dso_ExternalReader` (50 ms), `h3lis331dl_externalRead` (50 ms),
> `BMP388_ReadRawPressTempTime` (250 ms), and the `lsm6dso_init` reset (100 ms) all time out
> and return `HAL_TIMEOUT`/`HAL_ERROR`. **Still unbounded — and they run at *boot* in
> `flight_sensors_init`:** `lsm6dso_Calib` (`lsm6dsox_hal.c:159-161`) and
> `h3lis331dl_Calibration` (`h3lis331dl_hal.c:154-156`); the latter also *lost* its
> `HAL_Delay(1)`, so it now spins the CPU. **Accepted by design (maintainer call):** these run
> only at boot in `flight_sensors_init`, so a dead/flaky sensor hangs *on the pad*, not in
> flight — and since no telemetry/continuity ever comes up, the ground crew sees the board
> never went "ready" and doesn't fly. That's a valid pre-flight fail-stop. It stays valid as
> long as (1) a hung board is **ground-observable** (no telemetry = stuck — true today) and
> (2) calibration/re-init never moves into the in-flight path. Restoring the dropped
> `HAL_Delay` / adding a deadline is nice-to-have (stops the CPU spin) but not flight-critical.

**Locations (the two calibration entries below are the ones still open):**
- `Core/App/AppDrivers/Sensors/lsm6dsox_hal.c:195-197` — `lsm6dso_ExternalReader`: `do { status_get } while(!xlda || !gda);` — **no delay, no timeout**.
- `Core/App/AppDrivers/Sensors/lsm6dsox_hal.c:58-60` — reset wait in `lsm6dso_init`: `do { reset_get } while (rst);`
- `Core/App/AppDrivers/Sensors/lsm6dsox_hal.c:156-158` — same pattern in `lsm6dso_Calib`.
- `Core/App/AppDrivers/Sensors/h3lis331dl_hal.c:134-137` and `157-160` — `do { delay(1); read } while(!(status & 0x08));`
- `Core/App/AppDrivers/Sensors/BMP388.c:244` — `do {…} while (!(status & …));`

**Why it matters:** if a sensor browns out, resets, or the SPI line glitches and
data-ready never asserts, these loops **never exit** — and because it's a superloop, the
*entire flight computer hangs*: no estimation, no FSM, no deploy, no telemetry. This is
the single most dangerous class of bug here. The IMU read is worst (no `HAL_Delay`, so it
also pins the CPU).

**Fix:** every wait gets a `HAL_GetTick()` deadline and returns `HAL_TIMEOUT` on expiry
(the flash driver already does this correctly — `W25Q128_WaitBusy`,
`Core/App/AppDrivers/Memory/W25Q128.c:43-52` — copy that pattern). Propagate the timeout
up so the caller can mark the sensor failed and continue on the remaining sensors. Pair
with **R3** (watchdog) as a backstop.

### 🔴 P1 — Drogue fire-timer variable typo (drogue channel never deasserts)
> **✅ RESOLVED in commit `b545bfd`** — the rename made `pyro.c` use `drogue_fire_start`
> consistently (declared `:13`, set `:63`, cleared in `pyro_service` `:83`), so the drogue line
> now deasserts after `PYRO_FIRE_DURATION_MS`. No `drouge` spelling remains anywhere (verified
> across `Core/`, `FATFS/`, and the `.ioc`), and the pin defines + `ContinuityPacket` field
> stayed aligned. Analysis kept below for reference.

**Location (original):** `Core/App/ApplicationLayer/ApplicationHALS/pyro.c` — `pyro_fire_drogue:62` vs the
declaration at `:13` and `pyro_service:82-84`.

The pyro refactor (good — see C1) deasserts each channel by timestamp in `pyro_service()`.
But `pyro_fire_drogue` writes the start time to **`drouge_fire_start`** (line 62), while the
only declared variable is **`drogue_fire_start`** (line 13) — which is what `pyro_service`
checks (line 82). The spellings differ ("drouge" vs "drogue"):

- If `drouge_fire_start` is undeclared, **the file does not compile** (the latest change may
  not have been rebuilt yet).
- If it *did* build, `pyro_service` watches `drogue_fire_start`, which `pyro_fire_drogue` never
  sets — so after a drogue fire the GPIO is asserted and **never deasserted**: the drogue
  igniter stays energized indefinitely.

Either way the drogue path is broken, it's safety-critical (a pyro channel), and it runs on
**every** apogee (both the normal and low-apogee branches call `pyro_fire_drogue`). **Fix:**
use one spelling everywhere (`drogue_fire_start`). Building with `-Wall -Werror` would have
caught this — worth enabling.

### 🟠 C1 — Pyro and buzzer block the loop with `HAL_Delay`
> **✅ RESOLVED (audit #4).** `pyro_fire_*` now assert the GPIO + record a timestamp, and
> `pyro_service()` (called each loop) deasserts after `PYRO_FIRE_DURATION_MS`; `HAL_Delay` is
> gone from `pyro.c`. **Caveat: the drogue path is broken by a typo — see P1.** (The buzzer
> `HAL_Delay` in `indicators.c` is unchanged, used only as a startup chirp.)

**Locations (original):** `Core/App/ApplicationLayer/ApplicationHALS/pyro.c:34-48`
(`pyro_fire_drogue/main/aux` each `HAL_Delay(500)`); `Core/App/Outputs/LED&buzzer/indicators.c:3-10`.

**Why it matters:** firing a pyro freezes the superloop for **500 ms** — during descent
that's 50 missed IMU samples, ~12 missed baro samples, no telemetry, no logging. At
`APOGEE`→`DROGUE` two fires can chain. For one-shot pyro it's *survivable*; for a
continuous control surface it is disqualifying.

**Fix:** make firing non-blocking — on fire, set the GPIO and record
`fire_start = HAL_GetTick()`; in the loop (or a timer ISR) clear it once
`HAL_GetTick() - fire_start >= PYRO_FIRE_DURATION_MS`. A hardware timer one-pulse is even
cleaner. Same treatment for the buzzer (state, not `HAL_Delay`).

### 🟠 C2 — `lora_TX` busy-polls up to 1 s
**Location:** `Core/App/AppDrivers/LoRa/LoRa.c:432-442` (called from
`lora_tx_telemetry`, `telemetry.c:51`, with a 1000 ms timeout).

```c
while (!(buffer & IRQ_TX_DONE_MASK)) {
    platform_read(&sx1, REG_IRQ_FLAGS, &buffer, 1);   // SPI read every spin
    if ((HAL_GetTick() - start) >= timeout_ms) return HAL_TIMEOUT;
}
```

**Why it matters:** the loop spins on SPI reads for the full packet airtime (and up to
1 s on failure), stalling everything. It also makes the radio and the (future) control
loop fight for time.

**Fix:** this is what the DIO0 interrupt is *for* — see **C3**. Drive TX completion off
the interrupt (or DMA + completion callback) and let the superloop/control task keep
running while the radio transmits.

---

## Interrupts & synchronization

### 🟡 C3 — LoRa TX-done interrupt is set up but never consumed
**Locations:** ISR `Core/App/ApplicationLayer/ApplicationHALS/Lora_App.c:11-15` sets
`lora_tx_done_flag = 1`; the flag (declared `Lora_App.c:9`) is **never read** anywhere in
the codebase, because `lora_TX` busy-polls instead (**C2**).

So the interrupt-driven completion path is fully wired (EXTI configured in
`MX_GPIO_Init`, handler in `stm32f4xx_it.c:205`) but dead. Two smaller issues ride along:

- **Prototype mismatch:** `Lora_App.h:63` declares `HAL_GPIOEXTI_Callback` (missing the
  underscore), but the implementation is the real HAL weak symbol
  `HAL_GPIO_EXTI_Callback`. The declared prototype is wrong *and* unused — delete or fix.
- If you do start reading the flag, treat it as the classic ISR-flag pattern: `volatile`
  (it is), **set in ISR, cleared by the consumer**, and don't do read-modify-write on it
  from both contexts.

**Fix:** consume the flag — clear it before starting TX, return to the loop, and check it
(or use `HAL_GPIO`/DMA TX-complete callbacks). This single change resolves **C2** as well.

### 🟡 C8 — Duplicate enum definitions collide across headers
**Locations:** `Core/App/AppDrivers/CAN/CAN.h:41-53` and
`Core/App/ApplicationLayer/ApplicationHALS/Lora_App.h:48-59` both define an enum with
`IDLE, ARMED, POWERED_ASCENT, …`.

**Why it matters:** including both headers in one `.c` is a **redefinition compile error**.
It hasn't bitten yet only because `CAN.c` is an empty stub. The moment you implement CAN
and a file needs both (very likely — CAN broadcasts FSM state *and* you serialize it for
LoRa), it won't build.

**Fix:** define the flight-state enum **once** (e.g., a shared `flight_state.h` or a new
`flight_ids.h`) and have both subsystems include it. Prefix enumerators
(`FS_IDLE`, `FS_ARMED`, …) to avoid polluting the global namespace. While there, note
these LoRa/CAN state codes are a *different numbering* from `FlightState_t` in
`flight_state.h` — make the mapping explicit and one-directional.

---

## Shared state & atomicity (the part that matters when you add concurrency)

Today there is effectively **one writer and one reader, in sequence**, so nothing below is
*corrupting* data right now. But this section is the core of "make it concurrency-safe and
atomic," because each item becomes a race the instant a second context appears.

### Inventory of shared mutable state

| State | Where | Written by | Read by |
|---|---|---|---|
| `FlightSensorData sensorData` | `main.c:93` (global) | main loop (sensor + KF updates) | FSM, telemetry, flash, KF |
| `KalmanFilter_t kf` | `kalman.c:5` (file static) | `kalman_predict/update` | `kalman_get_*` |
| `FSM_Context_t ctx` | `flight_state.c:9` (file static) | `FSM_update/transition` | `FSM_get_state` |
| `lora_tx_done_flag` | `Lora_App.c:9` (volatile) | **EXTI ISR** | (nobody — see C3) |
| `flash_write_addr`, `flash_record_count` | `W25Q128_HAL.c:9-10` | `flash_log_packet` | read path |
| per-driver `static stmdev_ctx_t dev_ctx`, cal offsets | sensor HALs | init | reads |
| `Timer1/Timer2` | `fatfs_sd.c:11` (volatile) | **SysTick ISR** (dec) | SD path (dead) |

### 🟡 C6 — The `sensorData` blackboard is not snapshot-safe
**Location:** `Core/Src/main.c:93`, passed by pointer everywhere.

`sensorData` is ~60 bytes of floats updated field-by-field across the loop, then handed by
pointer to `FSM_update`, `lora_tx_telemetry`, `flash_log_telemetry`. In the superloop the
writes complete before the reads, so reads are consistent. **Under concurrency** (a comms
or logging task, or reading it from an ISR) a reader can observe a *torn* struct — e.g.
new altitude with old velocity — because a multi-field struct copy is **not atomic** and
individual `float` writes aren't ordered.

**Fix — pick one:**
- **Double-buffer (ping-pong):** writer fills buffer B, then publishes by swapping an
  index/pointer (a single aligned 32-bit store *is* atomic on Cortex-M4). Readers grab the
  current pointer once and use it. Simple, lock-free, ideal for one-writer/many-readers.
- **Seqlock:** writer bumps a counter before/after writing; reader retries if the counter
  changed or is odd. Great for a high-rate producer and occasional readers.
- **Critical section:** wrap the copy in `__disable_irq()/__enable_irq()` (or
  `taskENTER_CRITICAL` under an RTOS). Easiest; costs a little latency.

The rule to adopt: **readers take an atomic snapshot, never walk the live struct.**

### 🟢 C7 — SPI1 is shared by three sensors with no arbitration
**Location:** all three sensor handles set `hspi = &hspi1` (`flight_sensors.c:34,48,61`);
each `platform_read/write` toggles only its own CS.

In the superloop, accesses are naturally serialized, so this is fine. If sensor reads ever
move to DMA, an ISR, or separate tasks, two transfers could interleave on the same bus and
on overlapping CS lines → corrupted reads.

**Fix (only when you go concurrent):** put a single **bus mutex** (or an SPI transaction
queue) around SPI1; acquire it for the whole CS-low→transfer→CS-high sequence. The radio
(SPI2) and flash (SPI3) are already isolated, so only SPI1 needs it.

### 🟠 C5 — SPI errors are silently swallowed
**Locations:** `lsm6dsox_hal.c:106-135` and `h3lis331dl_hal.c:94-122` — `platform_read`
and `platform_write` call `HAL_SPI_Transmit/Receive` but **discard the return value and
always `return 0`** (success).

**Why it matters:** a SPI timeout or bus error looks like a successful read of stale/zero
data. That zero then flows into the Kalman filter and FSM. It also defeats **C4**'s
timeout fix at the wrong layer (the ST `*_reg` layer checks this return code).

**Fix:** return a non-zero code on `HAL` failure so the ST driver and your wrappers can see
it; map it to `HAL_ERROR` at the `flight_sensors_*` boundary and mark the sensor failed.

---

## Real-time hygiene

### 🟡 R1 — `printf` in the hot loop
**Locations:** `main.c:211` prints every single pass; `:218,226` on errors; plus a
`printf` on every FSM state entry (`flight_state.c`) and many driver error paths.

`_write` (`main.c:101`) does a **blocking** `HAL_UART_Transmit(..., HAL_MAX_DELAY)` at
115200 baud (~87 µs/byte). A ~45-char line ≈ **4 ms** — longer than a whole 10 ms IMU
slot, and it's unbounded if the line is long. This is both a latency source and a jitter
source for any future control loop.

**Fix:** gate all hot-path prints behind `#ifdef DEBUG` (the loop already has a `DEBUG`
block — extend it), or move to a DMA/ring-buffer logger that never blocks the producer.
Definitely remove the unconditional per-pass `printf` at `main.c:211`.

### 🟡 R2 — Kalman `dt` is hard-coded
> **✅ RESOLVED in the current code** — `main.c:211` now computes
> `dt = (now - last_imu) / 1000.0f` and passes it to `kalman_predict`. One nit: the *first*
> predict uses time-since-boot as `dt`; `kalman_predict` rejects `dt > 1`, so a long init is
> safely ignored, but a sub-second init still yields one oversized first step — harmless,
> the covariance just re-converges.

**Location (original):** `main.c:219` — `kalman_predict(sensorData.z_mg_IMU, 0.01f);`

`dt` is fixed at 10 ms regardless of how long the pass actually took. Any jitter (from
C1/C2/R1) means the filter integrates over the wrong interval, biasing velocity/altitude.

**Fix:** measure real elapsed time between predicts (`now - last_imu` in seconds) and pass
that. `kalman_predict` already validates `0 < dt ≤ 1`.

### 🟠 R3 — No independent watchdog
**Observation:** no IWDG/WWDG is started. Given **C4**, a single stuck sensor read is an
unrecoverable brick.

**Fix:** enable the independent watchdog (IWDG) and kick it once per healthy loop pass, so
a hang reboots the MCU instead of freezing forever.

> ⚠️ **A watchdog is a backstop, not the primary fix — and a naive reset is dangerous in
> flight.** On reboot, `FSM_init()` resets the state to `IDLE` (`flight_state.c:13-17`), so
> a watchdog reset mid-flight would **lose the entire flight state** — the vehicle could
> re-arm, re-run launch detection, and mis-time recovery. So:
> 1. **Primary fix is C4** — bounded timeouts on every poll, so one bad sensor *degrades
>    gracefully* (skip it, flag it, keep flying on the others) instead of hanging at all.
>    The watchdog should almost never fire.
> 2. If you keep the IWDG (you should), make the reset **recoverable**: persist flight
>    state + key estimator values to **RTC backup registers** or **backup SRAM** (which
>    survive a reset) and restore them on boot, or boot into a dedicated "in-flight
>    recovery" state — never silently back to `IDLE`.
> 3. On boot, verify pyros are in the safe (low) state before doing anything else.

### 🟡 R4 — Fault handlers brick silently
**Locations:** `Error_Handler` (`main.c:761`, `__disable_irq(); while(1){}`), and
`HardFault/MemManage/BusFault/UsageFault_Handler` (`stm32f4xx_it.c:84-139`) all spin
forever.

**Why it matters:** a fault in flight leaves pyros in whatever state they were in, radio
silent, with interrupts off.

**Fix:** in the fault path, first drive all igniter GPIOs low (reuse `pyro_init`'s safe
state), optionally emit a last-gasp telemetry/flash marker, then reset via `NVIC_SystemReset()`
so the watchdog/boot can recover.

---

## New findings (RX / command / continuity paths)

From the new `lora_rx_command` / `lora_tx_continuity` code in `telemetry.c` and the
`packets.c/.h` module.

### 🟠 N1 — `lora_rx_command` blocks the loop up to 1 second
**Location:** `Core/App/Outputs/Telemetry/telemetry.c:137-186`, called from `main.c:241`.

At `STATE_PAD`, after each telemetry TX the loop calls `lora_rx_command()`, which does
`lora_RX(buff, &rxLength, 13, 1000)` — a **busy-poll with a 1000 ms timeout** (same polling
style as `lora_TX`). If no command arrives, the loop stalls for up to a full second: no
sensor reads, no Kalman update, no FSM tick. It also `printf`s on *every* call (including
timeouts) plus a 13-byte hex dump on receipt — more blocking UART in the path.

**Why it matters now:** it's only wired at PAD (pre-launch), so it won't stall an in-flight
loop — but it makes launch detection laggy, and it's exactly the kind of blocking call that
must go before this loop hosts anything time-critical. Same root cause as **C2/C3**: polling
instead of using the DIO0 RX-done interrupt.

**Fix:** drop the RX timeout to the minimum that catches a command between telemetry frames
(tens of ms), or — better — make RX interrupt/DMA-driven (DIO0 RX-done → flag → drain), and
gate the `printf` behind `DEBUG`.

### 🟡 N2 — Remote `CMD_FIRE` bypasses the FSM deploy flags and state guard
**Location:** `Core/App/Outputs/Telemetry/telemetry.c:160-178`.

A valid `CMD_FIRE` calls `pyro_fire_drogue()` / `pyro_fire_main()` directly. Issues:
- It does **not** set `ctx.main_fired` / `ctx.drogue_fired`, so the FSM has no record a
  channel fired. If the autonomous FSM later reaches APOGEE/DROGUE it can fire the *same*
  channel again — the two fire paths share no interlock.
- It's gated only by *where it's called* (main.c calls `lora_rx_command` only at
  `STATE_PAD`), not by a check inside the handler. Fragile — if the call site ever moves, a
  ground command could fire pyros in flight.
- The fire itself is the 500 ms blocking `HAL_Delay` (**C1**), now inside the RX handler.

The command *authentication* is good (sync + `command_packet` type + `0xBE` auth byte +
CRC16 in `command_deserializer`, `packets.c:159`); this is about the command's *effect*, not
its validation.

**Fix:** route manual fire through the **same single deploy helper the FSM uses** (the one
suggested in **M3**) so it sets/honors the fired flags and refuses to re-fire, and add an
explicit allowed-state check inside the handler instead of relying on the call site.

### 🟢 N3 — `CommandPacket` struct disagrees with the deserializer's wire layout
**Location:** `packets.h:47-53` vs `command_deserializer` (`packets.c:159-174`).

The `CommandPacket` struct implies `[sync, type, seq, cmd_id, channel, auth, crc]` (auth at
byte 5). But the deserializer reads `cmd_id` at `buff[3]`, `channel` at `buff[4]`, **`auth`
at `buff[6]`**, CRC at `buff[7..8]` — i.e. a 9-byte frame with an unused byte at `buff[5]`.
The struct and the on-wire format the parser enforces don't match, and there's no
`command_serializer` in this repo to pin it down (the ground station defines the other end).

**Why it matters:** whoever writes the ground-station command frame must match the
*deserializer*, not the struct — easy to get wrong, and a mismatch silently fails CRC
(command ignored). Also `ContinuityPacket.aux` exists but `continuity_serializer` hardcodes
that byte to `0x00` (`packets.c:150`) — aux continuity isn't actually transmitted.

**Fix:** document the exact command wire format in one place (ideally add a matching
`command_serializer`, even if only the ground station uses it) and reconcile the struct
field order with the byte offsets the deserializer uses.

### 🟡 N4 — LoRa TX/RX are order-dependent on radio mode (no forced standby)
**Location:** `Core/App/AppDrivers/LoRa/LoRa.c` — `lora_TX:406`, `lora_RX:462`, and the
timeout/error early-returns in both.

Both `lora_TX` and `lora_RX` start with `if (mode != MODE_STAND_BY) return HAL_ERROR` — they
*assume* the caller left the radio in standby rather than forcing it. Their timeout / CRC /
SPI-error early-returns also leave the radio in `TRANSMIT` / `RX_SINGLE`, not standby. The new
code now **interleaves TX (telemetry/continuity) and RX (command) in the main loop**, so the
radio's mode is shared state across calls.

Concretely: `lora_rx_command` calls `lora_RX(..., 50 ms)`. `REG_SYMB_TIMEOUT` is never set, so
RX_SINGLE uses the chip default (~100 symbols ≈ 100 ms at SF7/BW125). The 50 ms driver poll
expires *first*, so on the normal "no command" path `lora_RX` returns `HAL_TIMEOUT` with the
radio **still in RX_SINGLE**. It recovers only because (a) the SX1276 auto-returns to standby
when its own symbol-timeout fires ~50 ms later, and (b) the next TX is ~200 ms away. So it
works today **by timing coincidence, not by design**: shorten the telemetry interval below
~100 ms, or TX right after RX, and that next TX would hit the standby check and silently fail
(dropped frame).

**Fix:** make the mode explicit — call `lora_standby()` at the *start* of `lora_TX`/`lora_RX`
(force a known mode instead of erroring out), and drive the radio back to standby on every
timeout/error exit. Setting `REG_SYMB_TIMEOUT` to a defined value also makes RX_SINGLE
deterministic.

### 🟢 NR1 — BMP388 sensortime parsed from the wrong registers (regression in `f925c21`)
**Location:** `Core/App/AppDrivers/Sensors/BMP388.c` — the `*time` assignment in
`BMP388_ReadRawPressTempTime`.

The commit changed the sensortime parse from `raw_data[10/9/8]` to `raw_data[8/7/6]`. The
11-byte burst starts at `DATA_0` (0x04), so `SENSORTIME_0..2` (0x0C–0x0E) are indices
**8, 9, 10** — the original was correct; indices 6/7/8 read `0x0A/0x0B/0x0C` (wrong registers).
Pressure (idx 0–2) and temperature (idx 3–5) are unchanged and still correct, and `time` isn't
consumed downstream, so this is **harmless today** — but it's a latent correctness bug if you
ever use sensor time. **Fix:** restore `*time = raw_data[10]<<16 | raw_data[9]<<8 | raw_data[8]`.

---

## Target architecture for "concurrency-safe and atomic"

Two viable paths. **You don't need an RTOS to be safe** — but you do need to stop blocking.

> **Recommendation for ODIN V2: Path A (non-blocking superloop + DMA + IWDG), not an RTOS.**
> DMA and RTOS aren't alternatives — DMA is *how peripherals move data*, an RTOS is *how
> you schedule tasks*; you can use both. But the decision is about workload. Now that
> active control is offloaded to the KESTREL board, ODIN's job is a fixed set of **periodic**
> tasks (sense → estimate → decide → log → downlink). That is the ideal case for a
> **time-triggered / cyclic-executive superloop**: it's more deterministic and far easier
> to verify than a preemptive RTOS, with no context-switch jitter, no per-task stacks, and
> none of the concurrency hazards in this very document. Add **DMA + interrupts** for the
> SPI sensor reads and the LoRa radio so the CPU stops busy-waiting (kills C1/C2/C4), and
> **IWDG** as the backstop. Reach for an RTOS only when you genuinely have many
> asynchronous, mixed-priority, event-driven tasks (multiple comms links, command shells,
> a filesystem) — not the case here. *(On KESTREL the calculus is similar; if its control
> loop must share the CPU with heavy CAN traffic and needs a guaranteed hard-real-time
> slot, a small RTOS or cyclic executive there is more defensible — but DMA + non-blocking
> is usually still enough.)*

### Path A — Stay superloop, go fully non-blocking (recommended next step)
Lowest risk, keeps determinism, unblocks a control loop:
1. Replace every blocking wait with a bounded, non-blocking state machine: pyro fire by
   timestamp (**C1**), sensor reads with timeout (**C4**), LoRa TX off the DIO0 interrupt
   (**C2/C3**), logging via a ring buffer drained incrementally.
2. Convert SPI to **DMA + completion flag** so reads/writes overlap compute.
3. Keep the single writer; publish `sensorData` via **double-buffer** (**C6**) so any ISR
   or DMA callback reads a consistent snapshot.
4. The freed-up determinism directly benefits ODIN's estimation and telemetry timing — and
   the same non-blocking pattern is exactly what KESTREL's control loop should follow so it
   runs at a fixed, jitter-bounded rate.

### Path B — Move to an RTOS (FreeRTOS / CMSIS-RTOS) when complexity grows
Adopt when you have several independent rates (control + comms + logging + CAN):
- **Tasks:** `sensor+estimate` (highest prio, hard 100 Hz), `control` (fixed rate),
  `comms/telemetry` (lower), `logging` (lowest). 
- **ISRs do the minimum:** set a flag / give a semaphore / push to a queue, then return.
- **Share state by message-passing** (queues) where possible; where you must share memory,
  use a **mutex** (with priority inheritance) or the **double-buffer/seqlock** patterns
  above. One **SPI1 bus mutex** (**C7**).
- Size each task's stack from worst-case; keep **static allocation**
  (`xTaskCreateStatic`) to preserve the no-heap property.

### The atomicity rules to adopt either way
- **One writer per piece of state.** Many readers are fine; many writers need a lock.
- **Readers take a snapshot** (double-buffer/seqlock/critical section) — never read a
  live multi-field struct that an ISR/other task can be mid-write on.
- **ISRs only signal** (set `volatile` flag, give semaphore, enqueue). No business logic,
  no blocking, no long SPI transactions in an ISR.
- **Every wait is bounded** by a timeout, and a watchdog backs the whole thing.
- A single aligned ≤32-bit load/store *is* atomic on Cortex-M4; anything wider or any
  read-modify-write across contexts needs protection (`__disable_irq`/PRIMASK, LDREX/STREX,
  or an RTOS primitive).

---

## Suggested order of work

*(Done: **M1**, **R2**, **C4 for the in-flight reads**, **N1** mitigated.)*

1. **C5** — stop swallowing SPI errors (`platform_*` should return the HAL status, not always 0). 🔴
   *(C4's calibration-loop hang is **accepted** as a pre-flight fail-stop — see C4; no action.)*
2. **M3** — fix the low-apogee FSM dead-end (`APOGEE` low branch → `PARAFOIL`, not `DROGUE`). 🔴/🟠
3. **R3** — turn on the IWDG watchdog as a safety net behind #1. 🟠
4. **C1 + N2** — non-blocking pyro fire; route remote `CMD_FIRE` through the same guarded
   deploy helper (sets fired-flags, state-checked). 🟠
5. **C2 + C3 + N4** — interrupt/DMA-driven LoRa TX *and* RX; consume `lora_tx_done_flag`;
   force radio standby on entry/exit so TX and RX can't wedge each other. 🟠
6. **R1** — gate the remaining `printf` (error paths, FSM transitions, RX handler) behind `DEBUG`. 🟡
7. **C8 + N3 + NR1** — unify the duplicate flight-state enums before CAN; pin down the command
   wire format; restore the BMP388 sensortime bytes. 🟡/🟢
8. **C6 + C7** — double-buffer `sensorData` and add an SPI1 mutex *as part of* going
   concurrent / adding a control task. 🟡/🟢
9. **R4** — safe-state fault handlers (drive pyros low, then reset). 🟡

Items 1–3 are "before next flight." Items 4–5 are "before active control." 6–9 ride along
with the concurrency/control work.

---

*Cross-references use the IDs above and match the call-outs in
[SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md). Line numbers are from the `dev` branch at
documentation time — re-verify after edits.*
