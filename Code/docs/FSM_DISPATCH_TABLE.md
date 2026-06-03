# FSM Refactor — Function-Pointer Dispatch Table

> **Goal.** Replace the monolithic `switch(ctx.state)` in `flight_state.c` with a
> **table of function pointers** indexed by the state enum. This document is design
> guidance and pointers only — **no code**. It maps your *current* 8-state FSM onto the
> pattern, calls out the gotchas that bite people, and gives an incremental migration plan.
>
> Files in scope: `Core/App/ApplicationLayer/FlightStateMachine/flight_state.c` / `.h`.

---

## 1. Why move off the switch

Your current `FSM_update()` is one `switch` with eight `case` blocks, each repeating the
same `if (ctx.entry) { ctx.entry = 0; ... }` preamble and calling `FSM_transition()` inline.
It works, but:

- **Every state's logic lives in one growing function** — hard to read, hard to test in
  isolation, easy to fall through a missing `break`.
- **Entry logic is copy-pasted** eight times.
- **Per-state counters** (`launch_count`, `burnout_count`, `apogee_count`) are `static`
  locals scattered in the cases — and they are **not reset by `FSM_init()`** (see §6, this
  is a latent bug today).
- **Transitions are scattered**, so the "what happens on a transition" bookkeeping is
  duplicated.

A dispatch table fixes all of that: each state becomes its own small, testable function;
entry/exit/transition bookkeeping is centralized; and adding or reordering states is a
one-line table edit.

> **Honest trade-off (good interview point):** a `switch` is trivially static-analyzable
> and can't jump to a bad address; a function-pointer table *can* if you index it
> out-of-bounds or leave a NULL slot. So the table buys testability and separation at the
> cost of two invariants you must guarantee (bounds + non-NULL). Some safety-critical
> shops (strict MISRA) prefer switches for exactly this reason. For an 8-state hobby/▶
> research flight computer the table is a clean, defensible choice — just enforce the
> invariants in §7.

---

## 2. The shape (described, not coded)

Three pieces:

**(a) A uniform handler signature.** Every state's per-tick logic is a function with the
*same* prototype so they can share one table. The natural signature mirrors your current
`FSM_update`: it receives the context and the sensor snapshot and **returns the next
state**. Conceptually: `FlightState_t run(FSM_Context_t *ctx, const FlightSensorData *data)`.
A handler returns *its own* state to mean "stay here," or a different state to request a
transition. (Returning the next state — rather than calling a transition helper inside the
handler — keeps all transition bookkeeping in one place; recommended.)

**(b) A per-state descriptor struct.** One small struct per state, holding up to three
function pointers plus optional metadata. Suggested fields:

| Field | Type (conceptual) | Purpose |
|---|---|---|
| `on_enter` | function ptr `(ctx, data) → void` | Runs once when the state is entered. Replaces the repeated `if (ctx.entry)` block — e.g. APOGEE fires pyro here. May be NULL → skip. |
| `on_run` | function ptr `(ctx, data) → FlightState_t` | Per-tick logic; returns next state (or same = stay). This is the required field. |
| `on_exit` | function ptr `(ctx, data) → void` | Optional cleanup when leaving the state. May be NULL. |
| `name` | `const char *` | For the `printf("FSM: …")` logging, table-driven instead of inline. |
| `timeout_ms` | `uint32_t` | *Optional* — see §8; lets the dispatcher handle timeouts generically. 0 = none. |
| `on_timeout` | `FlightState_t` | *Optional* — state to go to when `timeout_ms` elapses. |

**(c) The table itself.** A `static const` array of those descriptors, **indexed by the
`FlightState_t` enum value**. Because your enum is dense (0..7), the state value *is* the
array index — O(1) dispatch, no search. Keep it `const` so it lives in flash, is immutable,
and can't be corrupted by a stray write or stack overflow.

```
            FlightState_t (enum, 0..7)            const descriptor table (in flash)
            ───────────────────────────          ─────────────────────────────────────
   ctx.state ─────────► [STATE_IDLE=0]  ─────────► { on_enter, on_run, on_exit, name, … }
                        [STATE_PAD=1]   ─────────► { … }
                        [STATE_BOOST=2] ─────────► { … }
                              ⋮                          ⋮
                        [STATE_LAND=7]  ─────────► { … }
                        [STATE_COUNT=8]  ◄── sentinel, used only for bounds-checking
```

---

## 3. The dispatcher (what `FSM_update` becomes)

`FSM_update()` shrinks to a handful of lines that do the same job for *every* state:

1. **Bounds-check** `ctx.state` against `STATE_COUNT` (see §7). Bail / fault-safe if bad.
2. Grab `entry = &table[ctx.state]`.
3. **If just entered** (`ctx.entry`): clear `ctx.entry`, log `entry->name`, and call
   `entry->on_enter` if non-NULL.
4. Call `entry->on_run` → `next`.
5. *(Optional, §8)* if `entry->timeout_ms` and the dwell time exceeded it, set
   `next = entry->on_timeout`.
6. **If `next != ctx.state`:** call the old state's `on_exit` (if any), then do the
   transition bookkeeping (`ctx.state = next; ctx.state_entry_time = HAL_GetTick();
   ctx.entry = 1;`) — i.e. your existing `FSM_transition`, now called from exactly one
   place.

That's the whole engine. All eight states route through it identically. `FSM_get_state()`
is unchanged.

---

## 4. Mapping your current states onto the table

| State | `on_enter` does | `on_run` returns next when… |
|---|---|---|
| `IDLE` | log | `> ARM_AUTO_DELAY_MS` → `PAD` |
| `PAD` | log | `launch_count ≥ LAUNCH_CONFIRM_SAMPLES` → `BOOST` |
| `BOOST` | log | `burnout_count ≥ …` **or** boost timeout → `COAST` |
| `COAST` | log | `apogee_count ≥ …` **or** coast timeout → `APOGEE` |
| `APOGEE` | **record apogee + fire pyro** (the deploy decision) → then it transitions | (entry does the work; `on_run` just returns the next state) |
| `DROGUE` | log | altitude < `MAIN_DEPLOY_ALT_M` **or** drogue timeout → `PARAFOIL` |
| `PARAFOIL` | log | landed (low alt + low vel) **or** parafoil timeout → `LAND` |
| `LAND` | log | stays (`on_run` returns `LAND`) |

Notice most `on_enter` are just the log line — which is why moving `name` into the table
(step 3 of the dispatcher) lets several states have a **NULL `on_enter`** and removes all
the duplicated `printf` blocks.

`APOGEE` is the interesting one: its real work (fire main or drogue, set the fired flag) is
an **entry action**. That's a clean fit for `on_enter`. **Fix M3 while you're here** — in
the low-apogee branch, return/transition to `PARAFOIL`, not `DROGUE` (see
[CONCURRENCY_SAFETY.md M3](CONCURRENCY_SAFETY.md#-m3--low-apogee-deploy-leaves-the-fsm-stuck-in-drogue)).
The table refactor is the natural moment to correct it.

---

## 5. Keep the transition bookkeeping in one place

Today `FSM_transition()` is called from ~10 sites. In the table design it's called from
**one** site (dispatcher step 6). That's the single biggest correctness win: there is
exactly one path that updates `ctx.state`, stamps `state_entry_time`, and raises
`ctx.entry`. No state can forget a step. (It also makes the `on_exit` hook trivial to add
later.)

---

## 6. Where the per-state counters go (and a bug this fixes)

`launch_count`, `burnout_count`, `apogee_count` are currently `static` locals inside the
switch. Problem: **`FSM_init()` does a `memset` of `ctx` but cannot touch those statics**,
so re-initializing the FSM in-process does *not* reset them — a latent bug.

In the table design, **move the confirm counter into `FSM_Context_t`** (e.g. a single
`uint8_t confirm_count`, since only one debounce is active at a time) and **reset it in
`on_enter`** (or in the dispatcher's transition step). Now `FSM_init()`'s `memset` clears
everything, the FSM becomes fully and reliably re-initializable, and the counter's lifetime
is explicit. This is a real robustness improvement that comes "for free" with the refactor.

(`FSM_Context_t` already carries unused fields — `prev_altitude`, `vert_velocity`, `armed`.
Either wire them in or drop them while you're restructuring.)

---

## 7. Gotchas — the two invariants you MUST enforce

A dispatch table can jump to a bad address where a `switch` simply can't. Guard both:

1. **Bounds-check the index every dispatch.** Add a `STATE_COUNT` enumerator at the end of
   `FlightState_t` and verify `ctx.state < STATE_COUNT` before indexing. An out-of-range
   state indexing a function-pointer array = a jump to garbage = HardFault. Treat an
   out-of-range state as a fault → safe state. (Belt-and-suspenders with the watchdog,
   [CONCURRENCY_SAFETY.md R3](CONCURRENCY_SAFETY.md#-r3--no-independent-watchdog).)
2. **No NULL `on_run`, and null-check optional hooks.** Every table row needs a real
   `on_run`; the dispatcher must check `on_enter`/`on_exit` for NULL before calling. A
   call through a NULL pointer is also a HardFault.

Three more:

3. **Use designated initializers** (`[STATE_BOOST] = { … }`) when you build the table, not
   positional order. Then reordering the enum can't silently misalign rows with states —
   the #1 way these tables rot.
4. **Keep the table `const`.** Flash-resident, immutable, can't be clobbered. (A
   non-`const` table in RAM is both a waste and a corruption target.)
5. **`STATE_COUNT` must stay the last enumerator** and the table must have exactly that
   many rows. A compile-time size check (a `_Static_assert` that the array element count
   equals `STATE_COUNT`) catches a missing/extra row at build time — cheap insurance.

---

## 8. Optional power-up: make timeouts data-driven

Every dynamic state of yours has a timeout (`BOOST_TIMEOUT_MS`, `COAST_TIMEOUT_MS`,
`DROGUE_TIMEOUT_MS`, `PARAFOIL_TIMEOUT_MS`). If you put `timeout_ms` + `on_timeout` in the
descriptor (table above) and let the **dispatcher** handle them generically (step 5), you
delete four copies of the `HAL_GetTick() - ctx.state_entry_time > …` check from the
handlers. The handlers then only encode the *interesting* (sensor-driven) transitions, and
the boring timeout safety-net is uniform table data. This is the kind of "data, not code"
move that reads very well and is hard to get wrong once the engine is tested.

You can extend the same idea later (allowed-transition masks, per-state telemetry cadence,
per-state pyro-arm flags) — all as columns in the table.

---

## 9. Suggested migration order (incremental, testable at each step)

Do it in small steps so you can flash-and-verify between each, rather than one big rewrite:

1. Add `STATE_COUNT` to the enum; add the descriptor struct type and an empty `const` table
   with designated initializers and the `_Static_assert`.
2. Move the per-state counter(s) into `FSM_Context_t`; confirm `FSM_init` still behaves.
3. Convert **one leaf state first** (e.g. `LAND` or `IDLE`) into `on_enter`/`on_run`
   functions and route just that state through the new dispatcher; keep the `switch` for the
   rest. Verify.
4. Convert the remaining states one at a time, deleting each `case` as you go.
5. When the `switch` is empty, delete it; `FSM_update` is now just the dispatcher.
6. Fold in the M3 fix and (optionally) the data-driven timeouts.
7. Add the bounds-check + NULL-checks and the safe-state-on-bad-index path.

---

## 10. Interview angle

If asked why/how you structured the FSM this way:
- *"I moved from a switch to a `const` function-pointer table indexed by the state enum, so
  each flight phase is an independently testable handler with explicit enter/run/exit hooks,
  and all transition bookkeeping happens in one dispatcher."*
- Show you know the **risk**: *"A table can branch to a bad address, so I bound-check the
  index against a `STATE_COUNT` sentinel, keep the table `const` in flash, guarantee every
  `on_run` is non-NULL, and use a static-assert so the table can't drift out of sync with
  the enum."*
- Bonus: *"Timeouts and other per-state policy live as data columns in the table, so the
  safety-net behavior is uniform and the handlers only encode the real sensor logic."*

That progression — pattern → its failure mode → how you defend against it — is exactly what
a systems interviewer wants to hear.

---

## Quick checklist

- [ ] `STATE_COUNT` sentinel added as the last enumerator
- [ ] Descriptor struct: `on_enter`, `on_run`, `on_exit`, `name`, (opt) `timeout_ms`/`on_timeout`
- [ ] `static const` table, **designated initializers**, `_Static_assert` on row count
- [ ] Dispatcher: bounds-check → enter hook → run → (timeout) → single transition site
- [ ] Confirm counter(s) moved into `FSM_Context_t`, reset on entry
- [ ] Every `on_run` non-NULL; optional hooks null-checked
- [ ] M3 low-apogee fix folded in (`APOGEE` low branch → `PARAFOIL`)
- [ ] Out-of-range state → safe state; watchdog (R3) as backstop
- [ ] Dead `FSM_Context_t` fields (`prev_altitude`, `vert_velocity`, `armed`) wired or removed
