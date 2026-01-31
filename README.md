# Hexapod Robot - ESP32 + Webots

A hexapod robot project for ESP32 hardware with Webots simulation support. Uses a shared C++ codebase for locomotion logic with platform-specific servo implementations.

## Project Structure

```
jareds-hexapod/
├── lib/
│   ├── HexapodCore/          # Platform-independent locomotion library
│   └── ESP32Servos/          # ESP32 servo implementation
├── src/                      # ESP32 firmware
├── webots/                   # Webots simulation
│   ├── worlds/               # World files
│   ├── controllers/          # Controller code
│   └── protos/               # Custom robot models
├── platformio.ini            # PlatformIO configuration
└── README.md
```

## Hardware Requirements

### ESP32 Build
- ESP32 development board
- 18× servo motors (SG90 or similar)
- 5-6V power supply for servos
- Connection wires

### Servo Pin Configuration
The servos are mapped as follows (see `src/main.cpp`):

| Leg | Position | Joints (Coxa/Femur/Tibia) |
|-----|----------|---------------------------|
| 0 | Front-Right | 13, 12, 14 |
| 1 | Middle-Right | 27, 26, 25 |
| 2 | Rear-Right | 33, 32, 35 |
| 3 | Rear-Left | 34, 39, 36 |
| 4 | Middle-Left | 4, 16, 17 |
| 5 | Front-Left | 5, 18, 19 |

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
- **`Hexapod.h/cpp`**: Main hexapod control class
- **`ServoInterface.h`**: Abstract servo interface
- **`Kinematics.h/cpp`**: Leg kinematics (future)
- **`Gait.h/cpp`**: Gait patterns (future)

### Platform-Specific Implementations

**ESP32** (`lib/ESP32Servos`):
- Uses [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) library
- Implements `ServoInterface` for physical servos

**Webots** (`webots/controllers/hexapod_controller`):
- Uses Webots Motor and PositionSensor devices
- Implements `ServoInterface` for simulated servos
- Shares same `Hexapod` class as ESP32

## Current Features

- ✅ 18-servo hexapod control (6 legs × 3 joints)
- ✅ Hardware abstraction layer
- ✅ ESP32 firmware with servo control
- ✅ Webots simulation environment
- ✅ Basic poses (stand, rest)

## Planned Features

- [ ] Inverse kinematics for leg positioning
- [ ] Tripod gait for walking
- [ ] Wave gait for slow walking
- [ ] Turning and rotation
- [ ] Remote control (Bluetooth/WiFi)
- [ ] IMU integration for balance
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
