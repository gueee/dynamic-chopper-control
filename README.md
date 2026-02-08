# Dynamic Chopper Control (DCC)

Velocity-synchronized TMC register modulation for Kalico.

## What

A Kalico module that dynamically adjusts TMC5160 chopper parameters based on the
real-time velocity profile from the motion planner. Eliminates the compromise of
fixed chopper settings across the entire speed range.

## Why

On high-voltage (48V), low-mass CoreXY systems, fixed TMC chopper parameters cause:
- Vibrations at specific speed ranges (StealthChop ~60-80mm/s, SpreadCycle ~300mm/s)
- Violent direction change impacts ("hits a wall" at every reversal)
- Suboptimal noise/efficiency across the full velocity range

Input shaping proved that velocity-aware, real-time modulation is architecturally
feasible in Klipper/Kalico. DCC extends this to TMC register control.

## Status

**Architecture draft** — see `docs/design.md`

## Next Steps

1. Verify architecture against actual Kalico source (see `docs/conversation_log.md`)
2. Phase 1: Python prototype for concept validation
3. Phase 2: C chelper implementation for deterministic timing
4. Phase 3: Automated tuning tooling with accelerometer feedback

## Project Structure

```
dcc-project/
├── README.md
├── docs/
│   ├── design.md              # Full architecture specification
│   └── conversation_log.md    # Origin discussion and decisions
├── src/                       # (Phase 2) C chelper code
│   └── .gitkeep
├── klippy/                    # (Phase 1) Python klippy extra
│   └── .gitkeep
├── config/                    # Example printer.cfg snippets
│   └── .gitkeep
└── test/                      # Test macros and patterns
    └── .gitkeep
```

## Hardware Target

- TMC5160 @ 48V
- CoreXY / AWD (VzBot)
- Lightweight toolhead (200-400g)
- Kalico firmware

## License

TBD
