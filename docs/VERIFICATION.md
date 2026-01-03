# Formal Verification and Modern Build System

## Overview

This directory contains formal verification specifications and scripts for the ornithopter flight control system. The verification framework integrates:

- **TLA+ specifications** for state machine verification
- **Z3 SMT solver** for mathematical property verification
- **PlatformIO** modern build system
- **Unity testing framework** for unit tests
- **CI/CD pipeline** for automated verification

## Directory Structure

```
ornithopter-hover/
├── specs/                      # TLA+ formal specifications
│   ├── OrnithopterController.tla
│   └── OrnithopterController.cfg
├── verification/               # Z3 verification scripts
│   └── verify_all.py
├── src/                        # Source code
│   ├── math/                   # Mathematical libraries
│   │   └── quaternion.h
│   ├── controller/             # Control algorithms
│   └── sensors/                # Sensor drivers
├── tests/                      # Unit tests
│   └── test_quaternion.cpp
├── docs/                       # Documentation
│   └── research-report.md
├── platformio.ini              # PlatformIO configuration
└── .github/workflows/          # CI/CD workflows
    └── ci.yml
```

## TLA+ Verification

### What is TLA+?

TLA+ (Temporal Logic of Actions) is a formal specification language for modeling concurrent and distributed systems. It allows us to:

1. **Model system behavior** at a high level
2. **Specify safety properties** (bad things never happen)
3. **Specify liveness properties** (good things eventually happen)
4. **Model check** all possible states automatically

### Ornithopter Controller Specification

The `OrnithopterController.tla` specification models:

- **Flight modes**: STARTUP, ARMED, HOVER, LANDING, EMERGENCY
- **State variables**: altitude, attitude, throttle, sensor health, battery
- **State transitions**: startup, arming, hover, landing, emergency
- **Safety invariants**: altitude minimums, attitude limits, battery monitoring
- **Temporal properties**: liveness, emergency response

### Running TLA+ Model Checker

#### Prerequisites

```bash
# Install Java (required for TLA+ tools)
sudo apt-get install openjdk-11-jre

# Download TLA+ tools
wget https://github.com/tlaplus/tlaplus/releases/download/v1.7.1/tla2tools.jar
```

#### Run Model Checker

```bash
cd specs
java -jar tla2tools.jar OrnithopterController.tla
```

#### Expected Output

```
TLC2 Version 2.16
Running in Model-Checking mode
Computing initial states...
Finished computing initial states: 1 distinct state generated
Model checking completed. No errors found.
  States examined: 12456
  Distinct states: 3421
  State queue: 0
```

### Key Invariants Verified

1. **SafetyInvariant**: System never enters unsafe state
   - Hover altitude always ≥ minimum safe altitude
   - Emergency mode → throttle = 0
   - Attitude within limits or transitioning to safe mode
   - Critical battery → landing or emergency

2. **TypeOK**: All variables maintain correct types and ranges

3. **EmergencyResponse**: Critical conditions trigger emergency mode

4. **AltitudeSafety**: Hover altitude maintained above minimum

## Z3 Verification

### What is Z3?

Z3 is a high-performance SMT (Satisfiability Modulo Theories) solver that can:

1. **Verify mathematical properties** of algorithms
2. **Check constraint satisfaction** for control systems
3. **Prove bounds** on sensor fusion and control outputs
4. **Find counterexamples** to false claims

### Verification Modules

The `verify_all.py` script includes:

1. **Control Bounds**: Verify actuator commands stay within physical limits
2. **Sensor Fusion**: Verify complementary filter produces bounded output
3. **Quaternion Properties**: Verify unit quaternion mathematical properties
4. **Power Constraints**: Verify battery capacity and flight time relationships
5. **Altitude Estimation**: Verify barometric altitude calculation
6. **Stability Margins**: Verify PID gain relationships for stability

### Running Z3 Verification

#### Prerequisites

```bash
# Install Z3 Python bindings
pip install z3-solver
```

#### Run All Verifications

```bash
python verification/verify_all.py
```

#### Expected Output

```
============================================================
Z3 FORMAL VERIFICATION SUITE
Ornithopter Control System
============================================================

============================================================
Verifying Control Bounds
============================================================

Checking: Control saturates before dangerous attitudes...
✓ PASS: Control bounds verified
  Maximum roll at control saturation: 60.0°

Checking: Symmetric control for symmetric attitude...
✓ PASS: Control symmetry verified

... (more tests)

============================================================
VERIFICATION SUMMARY
============================================================
✓ PASS: Control Bounds
✓ PASS: Sensor Fusion
✓ PASS: Quaternion Properties
✓ PASS: Power Constraints
✓ PASS: Altitude Estimation
✓ PASS: Stability Margins

Total: 6/6 tests passed

🎉 All verification tests PASSED!
```

### Adding New Verifications

To add a new verification test:

```python
def verify_my_property():
    """Verify a new property."""
    print("=" * 60)
    print("Verifying My Property")
    print("=" * 60)
    
    # Define variables
    x = Real('x')
    y = Real('y')
    
    # Create solver
    s = Solver()
    
    # Add constraints
    s.add(x > 0, y > 0)
    s.add(x + y == 10)
    
    # Verify property
    s.push()
    s.add(x < 5)  # Check if x can be less than 5
    result = s.check()
    if result == sat:
        print("✓ PASS: Property verified")
        return True
    else:
        print("✗ FAIL: Property violated!")
        return False
    s.pop()
```

## PlatformIO Build System

### What is PlatformIO?

PlatformIO is a modern embedded development platform that provides:

- **Unified interface** for multiple boards and frameworks
- **Automatic dependency management**
- **Built-in unit testing**
- **CI/CD integration**
- **Multiple build configurations**

### Supported Boards

- **Arduino Nano** (ATmega328P) - Original target platform
- **Teensy 4.0** (ARM Cortex-M7) - High-performance option
- **ESP32** (Xtensa LX6) - WiFi-enabled telemetry

### Building Firmware

```bash
# Install PlatformIO
pip install platformio

# Build for Arduino Nano
platformio run -e arduino_nano

# Build for Teensy 4.0
platformio run -e teensy40

# Build for ESP32
platformio run -e esp32

# Build all targets
platformio run
```

### Uploading Firmware

```bash
# Upload to connected board
platformio run -e arduino_nano --target upload

# Monitor serial output
platformio device monitor
```

### Running Tests

```bash
# Run tests on native (desktop) platform
platformio test -e native

# Run tests on actual hardware
platformio test -e arduino_nano

# Run specific test
platformio test -e native -f test_quaternion
```

### Project Configuration

The `platformio.ini` file defines:

```ini
[platformio]
default_envs = arduino_nano

[env:arduino_nano]
platform = atmelavr
board = nanoatmega328
framework = arduino
lib_deps =
    adafruit/Adafruit MPU6050@^2.2.4
    adafruit/Adafruit BMP280 Library@^2.6.6
build_flags = -Os -Wall -Wextra
```

### Adding Dependencies

Dependencies are automatically installed from the PlatformIO registry:

```ini
lib_deps =
    adafruit/Adafruit MPU6050@^2.2.4  # Specific version
    adafruit/Adafruit BMP280 Library  # Latest version
    Wire                               # Built-in library
```

## Unit Testing with Unity

### Unity Framework

Unity is a lightweight unit testing framework for C/C++ embedded systems.

### Test Structure

```cpp
#include <unity.h>
#include "my_module.h"

void setUp(void) {
    // Run before each test
}

void tearDown(void) {
    // Run after each test
}

void test_my_function(void) {
    int result = my_function(5);
    TEST_ASSERT_EQUAL(10, result);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_my_function);
    return UNITY_END();
}
```

### Assertions

Common Unity assertions:

```cpp
TEST_ASSERT_TRUE(condition);
TEST_ASSERT_FALSE(condition);
TEST_ASSERT_EQUAL(expected, actual);
TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
TEST_ASSERT_NULL(pointer);
TEST_ASSERT_NOT_NULL(pointer);
```

## CI/CD Pipeline

### GitHub Actions Workflow

The `.github/workflows/ci.yml` defines automated verification:

1. **Build Job**: Compile firmware for all targets
2. **Test Job**: Run unit tests
3. **Formal Verification Job**: Run TLA+ and Z3 verification
4. **Documentation Job**: Validate documentation

### Workflow Triggers

- Push to main, develop, or copilot branches
- Pull requests to main or develop

### Viewing Results

Results are available in:
- GitHub Actions tab
- PR checks
- Downloadable artifacts (firmware binaries, test results)

## Development Workflow

### 1. Make Changes

```bash
# Edit source files
vim src/controller/attitude_control.cpp

# Edit tests
vim tests/test_attitude_control.cpp
```

### 2. Run Local Verification

```bash
# Build and test locally
platformio test -e native

# Run Z3 verification
python verification/verify_all.py
```

### 3. Commit and Push

```bash
git add .
git commit -m "Add attitude controller"
git push
```

### 4. CI Pipeline Runs

GitHub Actions automatically:
- Builds for all platforms
- Runs unit tests
- Performs formal verification
- Validates documentation

### 5. Review Results

Check GitHub Actions for:
- ✅ All checks passed → Ready to merge
- ❌ Failures → Review logs and fix issues

## Best Practices

### Verification Best Practices

1. **Write specifications first** (TLA+ and Z3 properties)
2. **Model check early** to catch design flaws
3. **Add verification for critical properties** (safety, bounds, invariants)
4. **Keep specifications synchronized** with implementation
5. **Document why properties hold** in comments

### Testing Best Practices

1. **Write tests before implementation** (TDD)
2. **Test edge cases** and boundary conditions
3. **Mock hardware dependencies** for native tests
4. **Maintain >80% code coverage**
5. **Run tests on actual hardware** before deployment

### Build System Best Practices

1. **Pin dependency versions** for reproducibility
2. **Use build flags** consistently across targets
3. **Separate debug and release builds**
4. **Keep build times short** (<5 minutes)
5. **Cache dependencies** in CI

## Troubleshooting

### TLA+ Issues

**Problem**: "No TLC output"
```bash
# Solution: Increase state space exploration
java -Xmx4g -jar tla2tools.jar -depth 1000 OrnithopterController.tla
```

### Z3 Issues

**Problem**: "Unknown satisfiability"
```python
# Solution: Add timeouts and simplify constraints
s.set("timeout", 10000)  # 10 second timeout
```

### PlatformIO Issues

**Problem**: "Library not found"
```bash
# Solution: Clean and reinstall
platformio lib install
platformio run --target clean
```

**Problem**: "Upload failed"
```bash
# Solution: Check board connection and permissions
ls -l /dev/ttyUSB*
sudo usermod -a -G dialout $USER
```

## References

1. [TLA+ Homepage](https://lamport.azurewebsites.net/tla/tla.html)
2. [Z3 Theorem Prover](https://github.com/Z3Prover/z3)
3. [PlatformIO Documentation](https://docs.platformio.org/)
4. [Unity Testing Framework](http://www.throwtheswitch.org/unity)
5. [GitHub Actions](https://docs.github.com/en/actions)

## Support

For questions or issues:
1. Check existing documentation in `/docs`
2. Review TLA+ specifications in `/specs`
3. Examine Z3 verification scripts in `/verification`
4. Open an issue on GitHub

---

*Last Updated: 2026-01-02*
