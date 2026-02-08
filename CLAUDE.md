# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project: Dynamic Chopper Control (DCC) for Kalico

A Kalico (Klipper fork) module that dynamically adjusts TMC5160 chopper registers synchronized to the motion planner's velocity profile. Analogous to input shaping, but for TMC driver registers instead of step timing.

## Owner

Wiz — GM TC (Günter Michaeller Technical Consulting), Saubersdorf, Austria. 30 years mechatronics experience. Direct, no-nonsense communication preferred.

## Current Status

Architecture draft complete (see `docs/design.md`). No code exists yet. Kalico source verification is needed before Phase 1 implementation begins.

## Build & Test

No build system, test framework, or CI is configured yet. The project is pre-implementation.

When Phase 1 begins:
- Python prototype goes in `klippy/extras/dynamic_chopper.py`
- C chelper code goes in `src/` (maps to `klippy/chelper/dcc.c` in Kalico)
- Test macros go in `test/`
- Example printer.cfg snippets go in `config/`

## Architecture

Three-layer design with **no MCU firmware changes**:

1. **Klippy Python layer** — config parsing, profile management, `register_move_handler()` for move notifications
2. **Chelper C layer** — reads `trapq` for velocity profiles, computes zone crossings, schedules timed SPI register writes (same level as `input_shaper.c`)
3. **MCU firmware** — existing `spi_transfer` command handles timed SPI writes to TMC5160; untouched

The core insight: Klipper's motion planner works ahead of real time. The full velocity profile is known before execution, so DCC pre-computes and pre-schedules all TMC register changes.

### Key Components (all NEW)

- **dcc_solver** — iterates trapq moves, identifies velocity zone boundary crossings, maps to chopper profiles
- **spi_schedule** — queues timed SPI register writes into the MCU command queue alongside step commands
- **dynamic_chopper.py** — Python config module, profile management, move handler registration

### Velocity Zone Model

Discrete velocity zones, each carrying a full register snapshot (CHOPCONF + PWMCONF + GCONF + current scaling). Special reversal profile for direction changes (reduced current, wider hysteresis, slower decay). See `docs/design.md` sections 4 and 8 for details.

## Kalico Source Files to Understand

Before implementing, read these from a Kalico checkout:
- `klippy/chelper/trapq.h` / `trapq.c` — `struct move` with velocity data (`start_v`, `half_accel`, `cruise_v`, `move_t`, `print_time`)
- `klippy/chelper/input_shaper.c` — reference pattern for hooking into the motion pipeline
- `klippy/extras/tmc.py` — `TMCCommandHelper`, `set_register(reg_name, value, print_time)` for timed writes
- `klippy/extras/tmc5160.py` — TMC5160 register definitions, `MCU_TMC_SPI`
- `src/spicmds.c` — MCU-side timed SPI command handling
- `klippy/chelper/stepcompress.c` — how timed commands are queued to MCU

## Implementation Phases

1. **Phase 1: Python prototype** — `klippy/extras/dynamic_chopper.py`, uses `toolhead.register_move_handler()`, calls `set_register()` with `print_time`. Accepts timing jitter.
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
