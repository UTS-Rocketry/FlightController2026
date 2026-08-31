# Outputs — Telemetry, Logging & Indicators

`Core/App/Outputs/`

Everything that leaves the board or surfaces state to a human: the LoRa downlink, the
onboard flash log, and the buzzer/LED. These modules sit at the top of the stack — they
*read* the `sensorData` blackboard and push it outward.

```
Outputs/
├── Telemetry/    telemetry.c   — serialize + downlink + flash-log + post-flight dump   ✅ 🟡
├── DataLogger/   logger.c      — 🧩 empty stub (logging actually lives in Telemetry)
└── LED&buzzer/   indicators.c  — buzzer beep helper                                     ✅
```

> **Heads-up on organization.** The naming suggests logging lives in `DataLogger/`, but it
> doesn't — `DataLogger/logger.c` is an empty stub, and the actual data-logging code is in
> `Telemetry/telemetry.c` (`flash_log_telemetry`) on top of
> [`W25Q128_HAL`](ApplicationLayer.md#w25q128_hal--flash-logging-policy-). Telemetry also
> reaches *down* to `Lora_App`'s serializer and the LoRa driver. It's a slight layering
> blur worth knowing when you go looking for the logging code.

---

## Telemetry/

### telemetry — downlink, logging, and replay ✅ 🟡
**Files:** `Telemetry/telemetry.c` (.h).

The single hub that builds telemetry frames and sends them to the radio, the flash, or the
console. Frames use the 58-byte format produced by
[`telemetry_serializer`](ApplicationLayer.md#lora_app--radio-config--telemetry-serialization-) (big-endian, CRC16).

| Function | Purpose | Notes |
|---|---|---|
| `serial_print(sensorData)` | Human-readable dump over UART (debug) | Blocking UART; keep behind `DEBUG` ([R1](CONCURRENCY_SAFETY.md#-r1--printf-in-the-hot-loop)) |
| `lora_tx_telemetry(sensorData)` | Build a packet, serialize, transmit over LoRa (3.3 Hz, state ≥ PAD) | ✅ `buff[62]` now (M1 fixed) |
| `lora_tx_gps(fix, state)` | Send the latest GPS fix/status at 1 Hz in a distinct 33-byte payload | Scheduled with all other radio traffic and a 25 ms guard |
| `lora_tx_continuity()` | Broadcast pyro continuity (main/drogue) every 2 s pre-launch (state ≤ PAD) | 12-byte packet, fits `buff[12]` |
| `lora_rx_command()` / `lora_rx_command_service()` | Start an interrupt-driven command RX window; later dispatch ARM/FIRE/DISARM after DIO0 | Non-blocking; remote FIRE still bypasses FSM fired-flags ([N2](CONCURRENCY_SAFETY.md#-n2--remote-cmd_fire-bypasses-the-fsm-deploy-flags-and-state-guard)) |
| `flash_log_telemetry(sensorData)` | Build a packet, serialize, append a 64-byte record to flash (25 Hz, state ≥ PAD) | ✅ correct buffer sizing |
| `flash_dump_serial()` | Read every flash record back and print decoded fields over UART | Post-flight analysis tool |

**Telemetry vs. logging — same data, two sinks, two rates:** the downlink runs at 3.3 Hz
(bandwidth-limited radio) while flash logging runs at 25 Hz (full-resolution post-flight
record). Both are gated to `state ≥ PAD` so the log isn't filled with idle pad data.

> ✅ **The earlier `lora_tx_telemetry` buffer overflow (M1) is fixed** — `buff[62]` now holds
> the 4-byte receiver header + 58-byte payload. The new `lora_tx_continuity` (12 B) and
> `lora_rx_command` (13 B) buffers are also correctly sized. See
> [CONCURRENCY_SAFETY M1](CONCURRENCY_SAFETY.md#-m1--stack-buffer-overflow-in-lora_tx_telemetry).
> **N1 is resolved:** command RX returns after configuring the radio and the FIFO is drained
> after the DIO0 event. The remaining command-path hazard is **N2** (remote FIRE bypasses
> the FSM fired-flags).

**`flash_dump_serial` decoding** mirrors the serializer layout by hand
(`memcpy` of each float at its byte offset). If you ever change the wire format, you must
update *three* places in lockstep: the serializer, this decoder, and the ground-station
parser. Centralizing the field layout (a shared struct/offset table) would remove that
triple-maintenance hazard — related to
[M2](CONCURRENCY_SAFETY.md#-m2--telemetry_serializer-has-an-implicit-length-contract).

---

## DataLogger/  🧩

**Files:** `DataLogger/logger.c` (.h) — **empty stubs.**

Reserved for a dedicated logging module, but unused today; the live logging path is
`flash_log_telemetry` (above) + [`W25Q128_HAL`](ApplicationLayer.md#w25q128_hal--flash-logging-policy-).
If/when logging grows (multiple record types, a ring buffer, SD support), this is the
natural place to consolidate it — and a ring-buffered, non-blocking logger here is part of
the [Path A](CONCURRENCY_SAFETY.md#path-a--stay-superloop-go-fully-non-blocking-recommended-next-step)
non-blocking refactor.

---

## LED&buzzer/

### indicators — audible status ✅
**Files:** `LED&buzzer/indicators.c` (.h).

| Function | Purpose |
|---|---|
| `buzzer_function(time)` | Beep: buzzer on for `time` ms, off for `time` ms |

Minimal today. Two notes:
- `buzzer_function` uses **blocking `HAL_Delay`** twice
  ([C1](CONCURRENCY_SAFETY.md#-c1--pyro-and-buzzer-block-the-loop-with-hal_delay)) — fine
  for a startup chirp, not for use during flight.
- The **RGB LED** is wired to a **TIM2 PWM** pin (configured in `main.c`'s GPIO init) but
  nothing drives it yet — it's just a status indicator on this board. (Servo-driven control
  surfaces live on the KESTREL control board, not here — see
  [SYSTEM_OVERVIEW §8 (Tier 2)](SYSTEM_OVERVIEW.md#8-the-control-and-actuation-path).)
  Buzzer control is also duplicated in `pyro.c` (`pyro_buzzer_on/off`) — worth unifying so
  there's one owner of the buzzer GPIO.

---

## Data-out summary

| Sink | Module | Rate | Format | Status |
|---|---|---|---|---|
| LoRa downlink | `telemetry.c` → `Lora_App` → `LoRa.c` | 3.3 Hz (≥ PAD) | 58-byte CRC16 frame | ✅ |
| GPS downlink | `GPS.c` → `telemetry.c` → `LoRa.c` | 1 Hz (all states, after first NMEA sentence) | 33-byte CRC16 frame | ✅ |
| Flash log | `telemetry.c` → `W25Q128_HAL` → `W25Q128.c` | 25 Hz (≥ PAD) | 64-byte records | ✅ |
| Console / replay | `serial_print`, `flash_dump_serial` | debug / on-demand | text | ✅ (keep behind DEBUG) |
| Buzzer | `indicators.c` / `pyro.c` | events | GPIO | ✅ (C1) |
| RGB LED | (TIM2 PWM configured) | — | PWM | 🧩 not driven |
