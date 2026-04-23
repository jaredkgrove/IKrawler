# CLAUDE.md — Hexapod Robot Project

This file gives Claude Code agents the context needed to work effectively in this repo.

## Project Overview

An 18-servo hexapod robot (6 legs × 3 joints each) built on ESP32 hardware with a Webots simulation environment. A **platform-independent C++ core library** (`HexapodCore`) handles all locomotion logic; platform-specific implementations plug in via `ServoInterface`.

Current hardware uses two PCA9685 I2C PWM boards (addresses 0x40, 0x41) to drive all 18 servos. The ESP32 exposes WiFi AP mode + WebSocket control (`IKrawler` / `hexapod123`).

## Key Commands

```bash
# Build ESP32 firmware
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial
pio device monitor
```

See `tests/CLAUDE.md` for running unit tests and `webots/controllers/hexapod_controller/CLAUDE.md` for building the Webots controller.

## PlatformIO / VSCode extension

The PIO VSCode extension's background "rebuild IntelliSense index" is **disabled** (`platformio-ide.autoRebuildAutocompleteIndex: false` in `.vscode/settings.json`) because it was racing CLI `pio run` invocations — triggering library reinstalls mid-build and leaving stale `.sconsign`/missing `.o` paths.

`lib_deps` in `platformio.ini` is pinned to exact versions (no `^` ranges) so PIO has no reason to re-resolve on every build.

**When you edit `platformio.ini`** (change `lib_deps`, `build_flags`, partitions, etc.), you must manually refresh IntelliSense so the C/C++ extension picks up the new include paths. Either:
- VSCode command palette → **"PlatformIO: Rebuild IntelliSense Index"**, or
- CLI: `pio project init --ide vscode`

## Architecture

```
lib/HexapodCore/          # Platform-independent core (shared by ESP32 + Webots)
  include/
    Hexapod.h             # Main control class (gait, IK dispatch, state machine)
    LegIK.h               # Per-leg 3-DOF inverse kinematics
    GaitMath.h            # Stateless gait trajectory math (phase → foot delta)
    ServoInterface.h      # Abstract interface: attach/write/read/detach
lib/ESP32Servos/          # ESP32 GPIO servo implementation (ESP32Servo library)
lib/PCA9685Servos/        # I2C PCA9685 servo implementation (Adafruit library)
src/
  main.cpp                # ESP32 firmware entry point (WiFi AP, WebSocket, main loop)
  DualBoardServo.h        # Routes servo IDs across two PCA9685 boards
  web_ui.h                # Embedded HTML/CSS/JS web interface (PROGMEM)
webots/
  worlds/hexapod.wbt      # Webots world (inline robot model, 18 HingeJoint motors)
  controllers/hexapod_controller/
    hexapod_controller.cpp  # Webots main loop (uses same Hexapod class)
    WebotsServoImpl.h/cpp   # Bridges Webots Motor devices to ServoInterface
tests/                    # PlatformIO native unit tests (kinematics, gait math)
docs/hardware.md          # Servo wiring, PCA9685 channel map, power distribution
```

## Coordinate Systems

- **Body frame**: X = forward, Y = left, Z = up
- **Hip-local frame** (per-leg IK input): X = outward from hip, Y = forward/backward, Z = up
- Leg numbering: 0=FR, 1=MR, 2=RR, 3=RL, 4=ML, 5=FL (clockwise from front-right)
- Joint order per leg: 0=COXA (hip rotation), 1=FEMUR (thigh), 2=TIBIA (shin)

## Servo Mapping (DualBoardServo)

Servo IDs 0–17 map across two PCA9685 boards (addr 0x40 and 0x41):
- Board 1 (0x40): Legs 0–2 (right side)
- Board 2 (0x41): Legs 3–5 (left side)
- One 4-pin bank per leg; joint order within each bank is coxa, femur, tibia (channels 3, 7, 11 spare)
- Board 1 is mounted rotated 180° from board 2, so front/rear banks are flipped on the right side:
  - Board 1 (right): REAR_RIGHT → bank 0-3, MIDDLE_RIGHT → bank 4-7, FRONT_RIGHT → bank 8-11
  - Board 2 (left):  FRONT_LEFT → bank 0-3, MIDDLE_LEFT → bank 4-7, REAR_LEFT → bank 8-11

## Implemented Features

- 18-servo control via PCA9685 over I2C
- 3-DOF inverse kinematics (`LegIK`) with law-of-cosines 2-link solver
- Tripod gait (50% swing duty, 3+3 alternating groups)
- Heading, turn rate, and stride length control
- Body-frame foot positioning and body pose control
- WebSocket remote control (stand/walk/rest/stop/move/identify)
- Webots simulation (same HexapodCore, `WebotsServoImpl`)
- Unit tests for IK and gait math (native PlatformIO env)

## Engineering Rules

These apply to all changes in this repo:

1. **C++ best practices first**: If a request would require breaking encapsulation, testing private methods, or unsafe memory patterns, flag it and propose a better alternative before making changes.

2. **Maintain invariants when bounds change**: When modifying a constraint (e.g., `maxStride_`, servo angle limits), always re-validate dependent values. Use private helper methods to encapsulate update logic and enforce invariants.

3. **Configuration-agnostic code**: Derive limits and behaviors from fundamental parameters (leg segment lengths, servo specs). Use ratios/percentages (e.g., "stand at 50% of reach") so the code adapts when dimensions change.

4. **DRY — no duplicate values or logic**: Extract shared constants and helper functions. Values used in both implementation and tests must come from a single authoritative source.

## Key Design Invariants

- All servo angles are in degrees [0, 180]; IK clamps to this range.
- Leg segment lengths (coxa ~39.5mm, femur ~84.6mm, tibia ~102.5mm) are constants in `LegIK.h`; changing them requires re-validating reachability bounds.
- `Hexapod::update(deltaTime)` must be called at a consistent tick rate; the main loop targets 5ms (200Hz) on ESP32.
- `strideLength_` must always satisfy `strideLength_ <= maxStride_`; use the stride setter to enforce this.
- Tripod groups: GROUP_A = {FR, RR, ML} (even leg IDs), GROUP_B = {MR, RL, FL} (odd leg IDs).

## Hardware Notes

- Servos require 5–6V; ESP32 GPIO is 3.3V (never power servos from GPIO rails).
- PCA9685 boards are genuine Adafruit units (I2C addresses 0x40 and 0x41).
- I2C: GPIO 8 (SDA), GPIO 9 (SCL).
