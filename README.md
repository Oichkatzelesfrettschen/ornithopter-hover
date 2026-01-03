# Hovering ornithopter :airplane: :bird:

Files for an ornithopter capable of hovering flight. The design is based on the aerovironment Nano Hummingbird and similar aircraft.

## Flight controller
The flight controller is written for arduino compatible boards like the [Nano](https://store-usa.arduino.cc/collections/boards-modules/products/arduino-micro) and the popular MPU-6050 IMU module.

## Design files | CAD
The aircraft is designed using [Freecad](https://www.freecad.org/) and its associated file formats (.FCStd).

## Research & Development

This project now includes comprehensive research and development documentation covering:

- **Mathematical foundations**: Quaternion and octonion rotations for singularity-free attitude control
- **Materials science**: Wing material analysis, structural optimization, and fatigue considerations
- **Fluid mechanics**: Aerodynamic modeling of flapping flight and hover conditions
- **Machine learning**: MLP-based adaptive control with self-awareness algorithms
- **Sensor fusion**: Multi-sensor integration (IMU, pressure, humidity, wind)
- **Formal verification**: TLA+ specifications and Z3 solver integration
- **Modern build system**: PlatformIO with automated testing and CI/CD

### Documentation

- **[Comprehensive Research Report](docs/research-report.md)**: Detailed mathematical and engineering analysis
- **[Verification Guide](docs/VERIFICATION.md)**: Formal verification and build system documentation

### Quick Start

#### Build Firmware

```bash
# Install PlatformIO
pip install platformio

# Build for Arduino Nano
platformio run -e arduino_nano

# Upload to board
platformio run -e arduino_nano --target upload
```

#### Run Tests

```bash
# Run unit tests
platformio test -e native

# Run formal verification
python verification/verify_all.py
```

#### Formal Verification

```bash
# Install Z3 solver
pip install z3-solver

# Run all verification tests
python verification/verify_all.py
```

### Project Structure

```
ornithopter-hover/
├── controller/             # Legacy Arduino controller
├── models/                 # CAD models (FreeCAD)
├── docs/                   # Comprehensive documentation
│   ├── research-report.md  # Full R&D report
│   └── VERIFICATION.md     # Build and verification guide
├── src/                    # Modern source code structure
│   ├── math/              # Quaternion and spatial math
│   ├── controller/        # Control algorithms
│   └── sensors/           # Sensor drivers
├── tests/                  # Unit tests (Unity framework)
├── specs/                  # TLA+ formal specifications
├── verification/           # Z3 verification scripts
├── platformio.ini         # Build configuration
└── .github/workflows/     # CI/CD pipeline
```

### Features

- ✅ **Quaternion-based attitude control** (no gimbal lock)
- ✅ **Sensor fusion** with complementary and Extended Kalman Filters
- ✅ **PID and LQR control** with adaptive gain scheduling
- ✅ **Formal verification** with TLA+ and Z3
- ✅ **Multi-board support** (Arduino Nano, Teensy 4.0, ESP32)
- ✅ **Automated testing** with Unity framework
- ✅ **CI/CD pipeline** with GitHub Actions

### Contributing

Contributions are welcome! Please:

1. Read the [Research Report](docs/research-report.md) to understand the system
2. Review the [Verification Guide](docs/VERIFICATION.md) for development workflow
3. Write tests for new features
4. Run formal verification before submitting PRs
5. Ensure CI pipeline passes

### License

This project is open source. See LICENSE for details.

### References

- **Nano Hummingbird**: AeroVironment's hovering ornithopter
- **DelFly Nimble**: TU Delft's agile flapping-wing MAV
- **Research papers**: See [research-report.md](docs/research-report.md) for citations
