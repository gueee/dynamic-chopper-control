# Dynamic Chopper Control (DCC) — Design Document

## Module: `dynamic_chopper` for Kalico

**Author:** GM TC / Günter Michaeller Technical Consulting  
**Target:** Kalico (Klipper fork) — chelper C layer + klippy Python config  
**Hardware:** TMC5160 @ 48V, CoreXY (VzBot AWD), lightweight toolhead (~200-400g)  
**Date:** 2026-02-08  
**Status:** Architecture Draft v0.1

---

## 1. Problem Statement

Fixed TMC chopper parameters are a compromise. On high-voltage (48V), low-mass CoreXY systems with TMC5160 drivers, this compromise produces:

- **StealthChop vibrations at 60-80 mm/s** — PWM regulation becomes coarse due to excessive voltage headroom. The driver overshoots on current injection.
- **SpreadCycle vibrations at ~300 mm/s** — Hysteresis band and decay timing are mismatched for the back-EMF at this velocity.
- **Violent direction changes** — At 48V the current slew rate during coil switching is extreme. The motor effectively hammers the mechanical system at every reversal.

The root cause is that chopper parameters optimized for one velocity range are wrong for another. The TMC5160 provides a few hardware-level velocity zones (TPWMTHRS, TCOOLTHRS, THIGH), but these only switch between operating modes — they don't modulate the chopper parameters within each mode.

Input shaping already proves that velocity-aware, real-time modification of the step stream is architecturally feasible in Klipper/Kalico. DCC extends this concept to TMC register modulation.

---

## 2. Core Concept

DCC is a **velocity-synchronized TMC register scheduler**. It reads the planned velocity profile from the trapezoid move queue (trapq) and schedules SPI register writes to TMC drivers at precise timestamps, ensuring chopper parameters are always optimal for the current operating conditions.

### Key Insight

Klipper's motion planner works **ahead of real time**. The full velocity profile (acceleration, cruise, deceleration, reversal) is known before the MCU executes it. This means DCC can pre-compute and pre-schedule all TMC register changes — no real-time reaction needed. Same principle as input shaping.

---

## 3. Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    Klippy (Python)                       │
│                                                         │
│  printer.cfg ──► [dynamic_chopper] config section        │
│                  - velocity_zones: [v1, v2, v3, ...]    │
│                  - chopper_profiles: {zone → registers}  │
│                  - reversal_profile: {registers}         │
│                  - transition_time: ms                   │
│                                                         │
│  Registers with toolhead.register_move_handler()         │
│  to receive move notifications                           │
└──────────────────────┬──────────────────────────────────┘
                       │ move events
                       ▼
┌─────────────────────────────────────────────────────────┐
│              Chelper C Layer (host-side)                  │
│                                                         │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │  trapq   │───►│ dcc_solver   │───►│ spi_schedule  │  │
│  │ (moves)  │    │ (velocity →  │    │ (timed TMC    │  │
│  │          │    │  profile     │    │  reg writes)  │  │
│  │          │    │  lookup)     │    │               │  │
│  └──────────┘    └──────────────┘    └───────┬───────┘  │
│                                              │          │
└──────────────────────────────────────────────┼──────────┘
                                               │ MCU command queue
                                               ▼
┌─────────────────────────────────────────────────────────┐
│                    MCU Firmware                           │
│                                                         │
│  Existing spi_transfer command executes timed SPI        │
│  writes to TMC5160 registers. No firmware changes.       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### What's New vs What Exists

| Component | Status | Notes |
|-----------|--------|-------|
| trapq (velocity profiles) | **Exists** | Already provides full move kinematics |
| itersolve (step timing) | **Exists** | Already consumes trapq |
| stepcompress (MCU commands) | **Exists** | Already queues timed commands |
| spi_transfer (MCU SPI) | **Exists** | Already writes TMC registers |
| **dcc_solver** | **NEW** | Velocity → profile mapping + timing |
| **spi_schedule** | **NEW** | Queues timed SPI writes alongside steps |
| **Python config module** | **NEW** | Config parsing, profile management |

### Critical: No MCU Firmware Changes Required

The MCU already supports timed SPI transfers. DCC only adds host-side logic to schedule them at the right moments. This makes it safe — worst case, bad register values cause motor noise, not MCU crashes.

---

## 4. Velocity Zone Model

### 4.1 Zone Definition

Each zone defines a velocity range and a complete set of TMC register values:

```
Zone 0: 0 mm/s          (standstill / hold)
Zone 1: 0 → V_stealth   (StealthChop low speed)
Zone 2: V_stealth → V_spread  (StealthChop high speed)
Zone 3: V_spread → V_high     (SpreadCycle normal)
Zone 4: V_high → V_max        (SpreadCycle high speed)
Zone R: reversal         (direction change — special profile)
```

### 4.2 Register Set Per Zone

Each zone carries a complete CHOPCONF + PWMCONF + GCONF register snapshot:

```c
struct dcc_profile {
    // CHOPCONF fields
    uint8_t toff;
    uint8_t hstrt;
    uint8_t hend;
    uint8_t tbl;
    uint8_t tpfd;
    uint8_t chm;        // chopper mode
    uint8_t vhighfs;    // high velocity fullstep
    uint8_t vhighchm;   // high velocity chopper mode

    // PWMCONF fields
    uint8_t pwm_ofs;
    uint8_t pwm_grad;
    uint8_t pwm_freq;
    uint8_t pwm_autoscale;
    uint8_t pwm_autograd;
    uint8_t pwm_reg;
    uint8_t pwm_lim;

    // GCONF fields
    uint8_t en_pwm_mode; // StealthChop enable

    // Current scaling
    uint8_t globalscaler;
    uint8_t irun;
    uint8_t ihold;
};
```

### 4.3 Reversal Profile

Direction changes get special treatment. Before a reversal (velocity → 0 → velocity in opposite direction), DCC can:

1. **Pre-load a soft profile** — reduce globalscaler/irun, widen hysteresis, increase toff
2. **Hold during reversal** — keep soft profile active through the zero-crossing
3. **Restore normal profile** — switch back as velocity ramps up in new direction

The timing for this is derived from the trapq: the deceleration-to-zero and acceleration-from-zero phases are fully known.

---

## 5. Integration Points in Kalico Source

### 5.1 Trapq — Velocity Source

**File:** `klippy/chelper/trapq.c` / `trapq.h`

The `struct move` contains everything DCC needs:

```c
// From trapq.h (Klipper/Kalico)
struct move {
    double print_time;       // when this move starts
    double move_t;           // total move duration
    double start_v;          // velocity at start
    double half_accel;       // acceleration / 2
    double cruise_v;         // peak velocity
    // ... position vectors
};
```

From this, instantaneous velocity at any time t within the move:
- Accel phase: `v(t) = start_v + 2 * half_accel * t`
- Cruise phase: `v(t) = cruise_v`
- Decel phase: `v(t) = cruise_v - 2 * half_accel * (t - t_cruise_end)`

DCC iterates through queued moves and identifies timestamps where velocity crosses zone boundaries.

### 5.2 Input Shaper — Architectural Reference

**File:** `klippy/chelper/input_shaper.c`

Input shaper already demonstrates:
- Hooking into the move pipeline
- Processing moves ahead of real time
- Generating modified output synchronized to motion

DCC follows the same pattern but outputs SPI commands instead of modified step timing.

### 5.3 TMC SPI — Command Interface

**File:** `klippy/extras/tmc.py`, `klippy/extras/tmc5160.py`

The existing `MCU_TMC_SPI` class sends register writes via:
```python
self.mcu_tmc.set_register(reg_name, value, print_time)
```

The `print_time` parameter already supports timed writes. DCC needs to call this with precise timestamps derived from the trapq velocity analysis.

### 5.4 Stepcompress — Command Scheduling

**File:** `klippy/chelper/stepcompress.c`

Step commands are scheduled with MCU clock timestamps. SPI commands go through the same MCU command queue. DCC's SPI writes must be scheduled with timestamps that precede the velocity threshold crossing by enough time to account for SPI transfer latency (~5-10 µs for a 32-bit TMC register write at typical SPI clock rates).

---

## 6. Implementation Plan

### Phase 1: Python Prototype (Proof of Concept)

**Goal:** Validate the concept with Python-level timing, accepting some jitter.

1. Create `klippy/extras/dynamic_chopper.py`
2. Register as a move handler via `toolhead.register_move_handler()`
3. On each move: calculate velocity zone crossings
4. Schedule `SET_TMC_FIELD` equivalent calls with `print_time` offsets
5. Test with 2-3 velocity zones + reversal profile

**Deliverables:**
- Config section `[dynamic_chopper stepper_x]`
- Velocity zone definitions in printer.cfg
- Logging of zone transitions for debugging
- Basic reversal detection and profile switching

**Limitations:**
- Python-level timing jitter (acceptable for proving the concept)
- May miss fast transitions during high-jerk moves

### Phase 2: Chelper C Implementation

**Goal:** Move the velocity analysis and SPI scheduling into the C chelper layer for deterministic timing.

1. Create `klippy/chelper/dcc.c` / `dcc.h`
2. Implement `dcc_process_moves()` — iterates trapq, finds zone crossings
3. Implement `dcc_schedule_spi()` — queues timed SPI register writes
4. Wire into the same flush cycle as stepcompress
5. Python wrapper via CFFI for config and status

**Deliverables:**
- Sub-microsecond timing for zone transitions
- Zero Python overhead during printing
- Same determinism as input shaping

### Phase 3: Tuning and Optimization

**Goal:** Build tooling for finding optimal zone profiles.

1. **Auto-sweep macro** — systematically tests chopper parameters at specific velocities while monitoring accelerometer data
2. **Profile recorder** — captures accelerometer vibration levels across the velocity range with different chopper settings
3. **Reversal tuning** — dedicated test pattern for direction change optimization
4. **Multi-stepper coordination** — ensure AWD steppers on the same axis get synchronized profile changes

---

## 7. Configuration Design

```ini
[dynamic_chopper stepper_x]
# Enable DCC for this stepper
enable: True

# Velocity zone boundaries (mm/s)
# Zones are: 0→v1, v1→v2, v2→v3, v3→v_max
zone_velocities: 50, 120, 250

# Chopper profiles per zone (indexed 0-3 for 4 zones)
# Format: toff, hstrt, hend, tbl, tpfd, en_pwm_mode
zone_0_chopconf: 4, 4, 2, 2, 1, 1    # low speed StealthChop
zone_1_chopconf: 3, 3, 1, 1, 1, 1    # mid speed StealthChop
zone_2_chopconf: 2, 4, 0, 0, 1, 0    # SpreadCycle normal
zone_3_chopconf: 1, 5, 0, 0, 0, 0    # SpreadCycle high speed

# PWM config per zone
zone_0_pwmconf: 20, 10, 1, 1, 4, 12  # pwm_ofs, pwm_grad, pwm_autoscale, pwm_autograd, pwm_reg, pwm_lim
zone_1_pwmconf: 15, 8, 1, 1, 4, 12
zone_2_pwmconf: 0, 0, 0, 0, 0, 0     # irrelevant in SpreadCycle
zone_3_pwmconf: 0, 0, 0, 0, 0, 0

# Reversal profile (applied during direction changes)
reversal_chopconf: 5, 2, 4, 2, 1, 0  # soft SpreadCycle — wide hysteresis, slow decay
reversal_current_scale: 0.8           # reduce current to 80% during reversal
reversal_lead_time: 0.5               # ms before zero-crossing to apply
reversal_hold_time: 1.0               # ms after zero-crossing to hold

# SPI timing
spi_lead_time: 0.01                   # ms before zone crossing to send SPI write
```

### AWD Configuration

For AWD setups, DCC on stepper_x automatically applies to stepper_x1 (the second motor on the same axis) with the same profiles and timing. Override with a separate `[dynamic_chopper stepper_x1]` section if needed.

---

## 8. Reversal Handling — Deep Dive

This is the highest-impact feature for the FP24x use case.

### The Problem in Detail

At 48V with a ~300g toolhead on a CoreXY:
1. Motor decelerates to zero (high current, high back-EMF dissipation)
2. Current in coil A drops to zero, coil B energizes for opposite direction
3. At 48V, this transition happens in microseconds
4. The mechanical system receives an impulse — effectively a hammer blow
5. This excites every mechanical resonance in the gantry

### DCC Reversal Sequence

```
Time ──────────────────────────────────────────────────►

Velocity:  ████████████▓▓▓▓░░░░░░░░▓▓▓▓████████████
                       ↑  decel    ↑  accel
                       │           │
                    switch to    switch to
                    reversal     zone 1
                    profile      profile

Current:   ═══════════╗         ╔═══════════════════
                      ╚═══80%═══╝
                      ↑ reduced during reversal ↑

Chopper:   [zone 2]  [reversal: soft]  [zone 1] [zone 2]
```

The key parameters during reversal:
- **Lower irun** — reduces the current magnitude, thus the force of the transition
- **Higher toff** — slower chopper, gentler current regulation
- **Wider hysteresis (hend)** — allows more current ripple but smoother zero-crossing
- **Disable tpfd** — passive fast decay fights you during reversals

---

## 9. SPI Bandwidth Analysis

**Question:** Can we send enough SPI writes fast enough?

### TMC5160 SPI Transfer Time

- SPI clock: typically 4 MHz on Klipper MCUs
- Register write: 40 bits (8-bit address + 32-bit data)
- Transfer time: 40 / 4,000,000 = 10 µs per register

### Worst Case: Zone Change Requires Writing

| Register | Purpose |
|----------|---------|
| CHOPCONF | toff, hstrt, hend, tbl, tpfd |
| PWMCONF | pwm_ofs, pwm_grad, etc. |
| GCONF | en_pwm_mode |
| IHOLD_IRUN | current scaling |
| GLOBALSCALER | current scaling |

= 5 registers × 10 µs = **50 µs per stepper per zone change**

### Worst Case Frequency

On a CoreXY at 500 mm/s with 0.5mm corner radius, direction changes occur at most every ~2 ms. With 4 steppers (AWD), that's 200 µs of SPI time per event, well within the 2 ms window.

**Verdict: SPI bandwidth is not a bottleneck.**

---

## 10. Safety Considerations

### Fail-Safe Behavior

- If DCC module crashes or is disabled, TMC drivers revert to their printer.cfg static configuration
- All register values are bounded to valid TMC5160 ranges in the config parser
- Current scaling is bounded: reversal_current_scale minimum 0.5 (never below 50% of configured run_current)
- A `DYNAMIC_CHOPPER_DISABLE` emergency command restores static config immediately

### What Can Go Wrong

| Scenario | Risk | Mitigation |
|----------|------|------------|
| Wrong register values | Motor noise, vibration | Bounded ranges in config |
| SPI write too late | Brief vibration at old settings | spi_lead_time configurable |
| SPI write too early | Brief suboptimal at wrong speed | Lead time < 1ms by default |
| Current too low on reversal | Missed steps | Minimum 50% current floor |
| Module crash during print | Reverts to static config | Watchdog in Python layer |

---

## 11. Testing Strategy

### Test 1: Static Zone Verification
- Set velocity zones to known values
- Run constant-speed moves at each zone
- DUMP_TMC to verify correct register values at each speed

### Test 2: Zone Transition Timing
- Print test pattern with known velocity profile
- Log zone transitions with timestamps
- Compare logged transitions to expected velocity crossings

### Test 3: Reversal Impact
- Print square corners at various speeds
- Compare accelerometer data with/without DCC reversal handling
- Measure peak vibration amplitude at direction changes

### Test 4: Print Quality
- Print standard quality test objects (Benchy, voron cube)
- Compare surface quality, dimensional accuracy, and corner sharpness
- A/B test with DCC enabled vs static best-compromise chopper settings

---

## 12. Future Extensions

### 12.1 Adaptive Profiling
Use accelerometer feedback (Beacon, ADXL345) to automatically adjust zone profiles based on measured vibration. The system would:
1. Run calibration moves across the velocity range
2. Measure vibration at each speed with different chopper parameters
3. Build optimal profile table automatically

### 12.2 Per-Axis Independent Zones
CoreXY axes have different mechanical characteristics. DCC could maintain independent zone maps for each axis based on the actual stepper velocity (not toolhead velocity).

### 12.3 Temperature-Dependent Profiles
Motor characteristics change with temperature. DCC could adjust profiles based on TMC driver temperature readings (available via DUMP_TMC).

### 12.4 Load-Dependent Profiles
For the FOUZIES industrial use case: different insole geometries create different toolhead loads. DCC could switch profile sets based on the active print job or detected resonance changes.

---

## 13. Open Questions

1. **Profile interpolation vs discrete zones?** — Discrete zones are simpler and may be sufficient. Linear interpolation between zone profiles would give smoother transitions but adds complexity. Start discrete, add interpolation if needed.

2. **How many zones are enough?** — Hypothesis: 4-6 velocity zones + reversal profile covers the practical range. The two known problem velocities (60-80 and ~300 mm/s) map to specific zone boundaries.

3. **StealthChop autoscale interaction** — When pwm_autoscale is enabled, the driver continuously adjusts pwm_ofs and pwm_grad. If DCC overwrites these, it fights the autoscale. Options: disable autoscale and rely entirely on DCC, or only override during SpreadCycle zones.

4. **Kalico bleeding-edge precision stepping** — Does the new high-precision stepping protocol change the timing of the step queue in a way that affects SPI scheduling? Need to verify compatibility.

5. **Multi-MCU setups** — If steppers are on different MCUs (e.g., toolhead board + main board), SPI writes go to different MCU command queues. Synchronization between them needs consideration.

---

## 14. References

- TMC5160 Datasheet Rev 1.18, Trinamic Motion Control
- Klipper Code Overview: https://www.klipper3d.org/Code_Overview.html
- Kalico repository: https://github.com/KalicoCrew/kalico
- klipper_tmc_autotune: https://github.com/andrewmcgr/klipper_tmc_autotune
- Trinamic TMC5160 Calculations Spreadsheet
- Kalico Bleeding Edge documentation: Precision Stepping, Smooth Input Shapers
