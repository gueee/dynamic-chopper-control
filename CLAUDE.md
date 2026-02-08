# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project: Dynamic Chopper Control (DCC) for Kalico

A Kalico (Klipper fork) module that dynamically adjusts TMC5160 chopper registers synchronized to the motion planner's velocity profile. Analogous to input shaping, but for TMC driver registers instead of step timing.

## Owner

Wiz — GM TC (Günter Michaeller Technical Consulting), Saubersdorf, Austria. 30 years mechatronics experience. Direct, no-nonsense communication preferred.

## Current Status

**Phase 1 prototype implemented** in the Kalico fork (`gueee/kalico`, branch `feature/dynamic-chopper-control`). Architecture verified against actual Kalico source.

## Repositories

- **This repo** (`dynamic-chopper-control`) — docs, planning, example configs
- **Kalico fork** (`gueee/kalico`, branch `feature/dynamic-chopper-control`) — actual implementation lives at `klippy/extras/dynamic_chopper.py`

## Build & Test

No build system or CI configured yet. To test:
1. Copy `klippy/extras/dynamic_chopper.py` from the Kalico fork into a Kalico installation
2. Add a `[dynamic_chopper stepper_x]` section to printer.cfg (see `config/dynamic_chopper.cfg`)
3. Use `DYNAMIC_CHOPPER_STATUS STEPPER=stepper_x` to verify

## Architecture (verified against Kalico source 2026-02-08)

Three-layer design with **no MCU firmware changes**:

1. **Klippy Python layer** — config parsing, profile management, hooks into `LookAheadQueue.add_move()` to intercept every kinematic move
2. **Chelper C layer** (Phase 2) — reads `trapq` for velocity profiles, computes zone crossings, schedules timed SPI register writes (same level as `kin_shaper.c`)
3. **MCU firmware** — existing `spi_transfer` command handles timed SPI writes to TMC5160; untouched

### How Phase 1 hooks into the motion pipeline

- Monkey-patches `toolhead.lookahead.add_move()` to append a `timing_callback` to every kinematic move
- The callback fires inside `ToolHead._process_moves()` AFTER `set_junction()` has populated the velocity profile (`start_v`, `cruise_v`, `end_v`, `accel_t`, `cruise_t`, `decel_t`)
- Callback receives `next_move_time` (end time of the move in print_time coordinates)
- Zone crossings are calculated from the trapezoidal velocity profile and timed SPI writes scheduled via `MCU_TMC_SPI.set_register(reg_name, val, print_time)`

### Key Components

- **`dynamic_chopper.py`** (Phase 1, exists) — Python config module, move interception, zone crossing calculation, timed SPI scheduling
- **`dcc_solver`** (Phase 2, planned) — C implementation of velocity analysis for deterministic timing
- **`spi_schedule`** (Phase 2, planned) — C-level queuing of timed SPI writes

### Velocity Zone Model

Discrete velocity zones, each carrying pre-computed register values (CHOPCONF + PWMCONF + GCONF). Special reversal profile for direction changes (wider hysteresis, slower decay). See `docs/design.md` sections 4 and 8.

## Kalico Source Files Reference

Verified file paths in `KalicoCrew/kalico`:
- `klippy/chelper/trapq.h` / `trapq.c` — `struct move` with `start_v`, `half_accel`, `move_t`, `print_time` (no `cruise_v` in C struct; derived from start_v + accel * accel_t)
- `klippy/chelper/kin_shaper.c` — C-level reference for hooking into kinematics (NOT `input_shaper.c` which doesn't exist)
- `klippy/extras/input_shaper.py` — Python-level shaper; uses `chelper.get_ffi()` and `stepper_kinematics` wrapping
- `klippy/extras/tmc.py` — `TMCCommandHelper`, `FieldHelper`, `set_register(reg_name, val, print_time)`
- `klippy/extras/tmc5160.py` — register definitions, field masks, `TMC5160CurrentHelper`
- `klippy/extras/tmc2130.py` — `MCU_TMC_SPI` class, `MCU_TMC_SPI_chain.reg_write()` with `minclock`
- `klippy/toolhead.py` — `Move` class, `LookAheadQueue`, `_process_moves()`, `register_lookahead_callback()`

**Important API notes:**
- Kalico has NO `register_move_handler()` — use `timing_callbacks` on `Move` objects or `register_lookahead_callback()`
- `register_lookahead_callback()` only hooks the LAST queued move, not all moves
- Timed SPI writes use `minclock` (minimum MCU clock for execution), converted from `print_time` via `mcu.print_time_to_clock()`

## Implementation Phases

1. **Phase 1: Python prototype** (done) — `klippy/extras/dynamic_chopper.py`, hooks `add_move`, calls `set_register()` with `print_time`. Accepts Python-level timing jitter.
2. **Phase 2: C chelper** — `klippy/chelper/dcc.c`, deterministic sub-microsecond timing, CFFI wrapper for Python config layer.
3. **Phase 3: Tuning tooling** — auto-sweep with accelerometer feedback, profile recording, reversal tuning macros.

## Hardware Context

- VzBot CoreXY, AWD (4 XY steppers), TMC5160 @ 48V
- BTT Manta M8P V2 mainboard, Beacon Rev H probes
- Lightweight toolhead (~200-400g)
- Target: FOUZIES industrial insole production system

## Coding Preferences

- No redundant functions or methods — if it's not called, delete it
- Precision and efficiency over verbosity
- Research-backed solutions; verify against actual Kalico source, don't assume
- C for performance-critical paths, Python only for config/UI
- Follow existing Kalico/Klipper coding patterns (no framework imports, match naming conventions)
