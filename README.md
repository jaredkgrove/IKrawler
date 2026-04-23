# Hexapod Robot - ESP32 + Webots

A hexapod robot project for ESP32 hardware with Webots simulation support. Uses a shared C++ codebase for locomotion logic with platform-specific servo implementations.

## Project Structure

```
jareds-hexapod/
├── lib/
│   ├── HexapodCore/          # Platform-independent locomotion library
│   ├── ESP32Servos/          # ESP32 GPIO servo implementation
│   └── PCA9685Servos/        # I2C PCA9685 servo implementation
├── src/                      # ESP32 firmware (main.cpp, web UI, DualBoardServo)
├── webots/                   # Webots simulation
│   ├── worlds/               # World files
│   └── controllers/          # Controller code
├── tests/                    # Unit tests (native PlatformIO env)
├── docs/                     # Hardware wiring and pinout docs
├── CLAUDE.md                 # Agent context for Claude Code
├── platformio.ini            # PlatformIO configuration
└── README.md
```

## Hardware Requirements

### ESP32 Build
- ESP32 development board
- 18× servo motors (SG90 or similar)
- 2× PCA9685 PWM servo driver boards (I2C addresses 0x40 and 0x41)
- 5-6V power supply for servos (servos require ~10A total; use separate UBEC)
- I2C wiring: GPIO 8 (SDA), GPIO 9 (SCL)

### Servo Layout
18 servos are organized as 6 legs × 3 joints (coxa/femur/tibia), driven over I2C via two PCA9685 boards. See [docs/hardware.md](docs/hardware.md) for the full channel map and power distribution.

## Software Requirements

### For ESP32 Development
- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- USB drivers for ESP32

### For Webots Simulation
- [Webots R2023b](https://cyberbotics.com/) or later
- C++ compiler (MinGW on Windows, GCC on Linux, Clang on macOS)

## Building and Running

### ESP32 Firmware

1. **Build the project:**
   ```powershell
   cd c:\jaredcode\jareds-hexapod
   pio run
   ```

2. **Upload to ESP32:**
   ```powershell
   pio run --target upload
   ```

3. **Monitor serial output:**
   ```powershell
   pio device monitor
   ```

### Webots Simulation

1. **Open Webots**

2. **Load the world file:**
   - File → Open World
   - Navigate to: `c:\jaredcode\jareds-hexapod\webots\worlds\hexapod.wbt`

3. **Build the controller (first time only):**
   - The controller should auto-compile when you load the world
   - Alternatively, in the controller directory:
     ```bash
     cd webots/controllers/hexapod_controller
     make
     ```

4. **Run the simulation:**
   - Click the Play button in Webots
   - The hexapod should initialize and move to standing position

## Code Architecture

### Shared Library (`lib/HexapodCore`)
Platform-independent hexapod control logic:
- **`Hexapod.h/cpp`**: Main control class — gait state machine, IK dispatch, body pose
- **`LegIK.h/cpp`**: 3-DOF inverse kinematics (law-of-cosines 2-link solver)
- **`GaitMath.h`**: Stateless gait trajectory math (phase → foot delta, lift height)
- **`ServoInterface.h`**: Abstract interface (attach/write/read/detach)

### Platform-Specific Implementations

**ESP32** (`lib/PCA9685Servos`, `src/DualBoardServo.h`):
- Two PCA9685 I2C boards driven via Adafruit PWM library
- `DualBoardServo` routes 18 servo IDs across both boards
- WiFi AP + WebSocket server for remote control

**Webots** (`webots/controllers/hexapod_controller`):
- `WebotsServoImpl` bridges Webots Motor/PositionSensor devices to `ServoInterface`
- Shares the same `Hexapod` class and `HexapodCore` as ESP32 firmware

## Current Features

- ✅ 18-servo control via dual PCA9685 I2C boards
- ✅ 3-DOF inverse kinematics per leg (law-of-cosines solver)
- ✅ Tripod gait (3+3 alternating groups, 50% swing duty)
- ✅ Heading, turn rate, and stride length control
- ✅ Body-frame foot positioning and body pose control
- ✅ WiFi AP + WebSocket remote control (stand/walk/rest/stop/move)
- ✅ Webots simulation (same HexapodCore, simulated servo implementation)
- ✅ Unit tests for IK and gait math (native PlatformIO env)

## Planned Features

- [ ] Wave gait for slow/stable walking
- [ ] IMU integration for balance compensation
- [ ] Autonomous navigation

## Development Workflow

1. **Design gaits in Webots**: Quickly iterate on locomotion algorithms
2. **Test in simulation**: Verify behavior without hardware
3. **Deploy to ESP32**: Same code runs on real hardware
4. **Fine-tune**: Adjust parameters for physical characteristics

## Troubleshooting

### ESP32 Build Errors
- Ensure PlatformIO is installed correctly
- Check that ESP32 board is selected in `platformio.ini`
- Verify USB connection and drivers

### Webots Controller Not Starting
- Check that `WEBOTS_HOME` environment variable is set
- Verify Makefile paths match your installation
- Rebuild controller: `cd webots/controllers/hexapod_controller && make clean && make`

### Servos Not Moving
- Check power supply (servos need 5-6V, ESP32 GPIO is 3.3V)
- Verify pin connections match `SERVO_PINS` array
- Test individual servos with simple sketch

## License

MIT License - see LICENSE file for details.

## Contributing

Contributions welcome! Please open an issue or pull request.
