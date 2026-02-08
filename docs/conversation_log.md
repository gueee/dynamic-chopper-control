# DCC Project — Conversation Log

## Origin: Claude.ai chat, 2026-02-08

This document captures the full technical discussion between Wiz (GM TC) and Claude
that led to the Dynamic Chopper Control (DCC) module concept for Kalico.

---

## Thread Summary

### 1. Initial Question: Can TMC chopper values be dynamically adjusted in Klipper/Kalico?

**Answer:** Yes, via `SET_TMC_FIELD` which writes TMC registers via SPI/UART at runtime
without restart. Changes are volatile (revert on restart). `DUMP_TMC` reads back values.

### 2. Narrowed to TMC5160 on 48V

Key 48V-specific considerations identified:
- `toff` needs to be low (2-3) — faster current decay at higher voltage
- Tighter hysteresis band (hstrt/hend) to avoid overshoot
- `tbl` can be lower (0-1) — better signal-to-noise at 48V
- `tpfd` (passive fast decay) helps current waveform quality
- `globalscaler` important — voltage headroom is massive
- StallGuard sensitivity changes with voltage

### 3. The Real Problem: Violent Vibrations

Wiz's symptoms on VzBot AWD, 48V, TMC5160, lightweight toolhead (~200-400g):
- **StealthChop vibrations at 60-80 mm/s** — PWM regulation too coarse
- **SpreadCycle vibrations at ~300 mm/s** — hysteresis/decay mismatch with back-EMF
- **Violent direction changes** — "like hitting a wall" — 48V current slew rate causes
  hammer-blow impulse at every reversal

Root cause: fixed chopper parameters optimized for one speed are wrong for others.

### 4. The Insight: Dynamic Adjustment is Tempting

Wiz observed that every static setting is a compromise. With:
- Only a few hundred grams of toolhead mass
- Overkill motor/driver combinations (very sensitive to chopper behavior)
- High MCU clock cycles available

...it should be possible to control every chopper value dynamically.

### 5. The Input Shaping Analogy (Key Breakthrough)

Wiz pointed out that input shaping already does real-time, velocity-aware parameter
modulation in Klipper/Kalico. It:
- Sits in the motion pipeline
- Has full knowledge of the velocity/acceleration profile
- Operates at high frequency synced to actual motion
- Is computationally lightweight

This proves the architecture supports what DCC needs. The difference: input shaping
modifies the step stream; DCC would output SPI register writes synchronized to the
same motion profile.

### 6. C vs Python Implementation

Wiz correctly identified that the C approach (chelper layer) is more predictable than
Python (klippy layer). Python has inherent timing jitter. For something synchronized
with motor motion at specific velocities, deterministic timing matters — late register
changes are worse than no changes.

The C chelper layer is where input shaping lives, confirming this is the right level.

### 7. Architecture Discovery

Deep dive into Kalico/Klipper source revealed:
- **No MCU firmware changes needed** — existing `spi_transfer` supports timed writes
- **trapq provides full velocity profiles** — `struct move` has start_v, cruise_v, half_accel, timing
- **chelper C layer** is the right integration point (same as input shaping)
- **SPI bandwidth is not a bottleneck** — 50µs per stepper per zone change, even AWD fits easily

### 8. klipper_tmc_autotune Reference

Discussed andrewmcgr's klipper_tmc_autotune project:
- Calculates register values from motor datasheet parameters
- Supports TMC5160 with voltage parameter up to 60V
- Has `autoswitch` mode (StealthChop/SpreadCycle) but acknowledges it's problematic
- Wiz tested it previously, found untuned drivers gave better results
- Autotune solves "what values" but not "when to change them" — DCC is the missing piece

### 9. Design Document Created

Full architecture spec produced: `docs/design.md`

### 10. Limitation: GitHub Clone Failed

The network proxy blocked git clone of the Kalico repo. Architecture is based on:
- DeepWiki documentation of Klipper source
- GitHub file listings and code snippets
- Mainline Klipper's documented architecture

**Action needed before Phase 1:** Verify against actual Kalico source files:
- `klippy/chelper/trapq.h` — exact move struct
- `klippy/chelper/input_shaper.c` — hook pattern reference
- `klippy/extras/tmc.py` + `tmc5160.py` — set_register timing interface
- `src/spicmds.c` — MCU-side timed SPI commands

---

## Key Technical Decisions Made

1. **Velocity-synchronized TMC register scheduler** — not reactive, pre-computed from trapq
2. **C chelper implementation** — deterministic timing, same layer as input shaping
3. **Discrete velocity zones** — start with 4-6 zones + reversal profile, add interpolation later
4. **Reversal handling as priority** — highest impact for the 48V lightweight toolhead case
5. **Phased approach** — Python prototype → C chelper → auto-tuning tooling
6. **No MCU firmware changes** — pure host-side, uses existing timed SPI infrastructure

## Hardware Context

- **Printer:** VzBot CoreXY, AWD (4 steppers on XY)
- **Drivers:** TMC5160
- **Voltage:** 48V
- **Toolhead mass:** ~200-400g
- **MCU:** BTT Manta M8P V2
- **Host:** Lenovo ThinkCentre (multi-printer control)
- **Probes:** Beacon Rev H
- **Target application:** FOUZIES industrial insole production (12-min cycle, 9 printers per rack)
