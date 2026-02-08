# CLAUDE.md — Project Context for Claude Code

## Project: Dynamic Chopper Control (DCC) for Kalico

### What This Is
A Kalico (Klipper fork) module that dynamically adjusts TMC5160 chopper registers
synchronized to the motion planner's velocity profile. Think "input shaping but for
TMC driver registers."

### Owner
Wiz — GM TC (Günter Michaeller Technical Consulting), Saubersdorf, Austria.
30 years mechatronics experience. Direct, no-nonsense communication preferred.
Don't introduce redundant functions or methods.

### Hardware Context
- VzBot CoreXY, AWD (4 XY steppers)
- TMC5160 drivers at 48V
- BTT Manta M8P V2 mainboard
- Lightweight toolhead (~200-400g)
- Beacon Rev H probes
- Host: Lenovo ThinkCentre (multi-printer)
- Target: FOUZIES industrial insole production system

### Architecture Summary
- **No MCU firmware changes** — uses existing timed SPI infrastructure
- **Chelper C layer** — same architectural level as input shaping
- **Reads trapq** for velocity profiles, schedules TMC SPI writes at zone crossings
- **Phased:** Python prototype first, then C implementation

### Current Status
Architecture draft complete. Need to verify against actual Kalico source before coding.

### Key Files to Examine in Kalico Source
Before starting Phase 1, read these from a Kalico checkout:
- `klippy/chelper/trapq.h` / `trapq.c` — move struct, velocity data
- `klippy/chelper/input_shaper.c` — reference pattern for hooking into pipeline
- `klippy/extras/tmc.py` — TMCCommandHelper, set_register with print_time
- `klippy/extras/tmc5160.py` — TMC5160 register definitions, MCU_TMC_SPI
- `src/spicmds.c` — MCU-side timed SPI command handling

### Coding Preferences
- No redundant functions or methods
- Precision and efficiency over verbosity
- Research-backed solutions, not assumptions
- C for performance-critical paths, Python only for config/UI
