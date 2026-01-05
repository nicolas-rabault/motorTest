# Motor Test Suite

Complete motor characterization test suite for Pollen Robotics test bench.

This package provides automated testing for BLDC motors using ODrive controllers, measuring electrical characteristics, mechanical properties, and thermal behavior.

## Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Configuration](#configuration)
- [Usage](#usage)
- [Motor Profiles](#motor-profiles)
- [Hardware Setup](#hardware-setup)
- [Test Measurements](#test-measurements)
- [Test Results](#test-results)
- [Command Reference](#command-reference)
- [Troubleshooting](#troubleshooting)

## Installation

### Prerequisites

- Python 3.8 or higher
- A virtual environment (recommended)
- ODrive controller
- Torque sensor and electromagnetic brake (for loaded tests)

### Install from source

1. Clone or navigate to the repository:
```bash
cd /path/to/motorTest
```

2. Activate your virtual environment:
```bash
# If using the project's virtualenv
source ~/.virtualenvs/luos/bin/activate

# Or create a new one
python3 -m venv venv
source venv/bin/activate  # On macOS/Linux
# venv\Scripts\activate   # On Windows
```

3. Install the package in editable mode:
```bash
pip install -e .
```

This will install all dependencies and create the `motor-test` command-line tool.

## Quick Start

### 1. Create a configuration file

Generate a configuration file with your test bench settings:

```bash
motor-test config --create
```

This creates `~/.motor-test-config`. Edit it with your serial ports:

```json
{
  "sensor_port": "/dev/tty.usbserial-FTAK88EN",
  "brake_port": "/dev/tty.usbmodem0000057700001",
  "odrive_port": null,
  "velocity": 10.0,
  "current": 2.0,
  "thermal_duration": 60.0,
  "debug": false
}
```

### 2. Create a motor profile

Create a JSON file describing your motor (e.g., `my_motor_profile.json`):

```json
{
  "name": "GMB2804",
  "description": "Small gimbal motor",
  "motor_type": "gimbal",
  "pole_pairs": 7,
  "kv_rating": 170,
  "current_lim": 8.0,
  "calibration_current": 5.5,
  "phase_resistance": 9.0,
  "phase_inductance": 0.001
}
```

### 3. Run the tests

```bash
motor-test --profile my_motor_profile.json
```

The tool will guide you through:
1. **No-load test** - Motor free to spin (measures impedance, KV, no-load current, inertia)
2. **Loaded test** - With brake and torque sensor (measures KT, thermal behavior, burst torque)

Results are saved to `results/{motor_name}_results.json`.

## Configuration

The motor-test suite uses a flexible configuration system with multiple sources (in order of precedence):

1. **Command-line arguments** (highest priority)
2. **Environment variables** (prefixed with `MOTOR_TEST_`)
3. **Configuration file**
4. **Default values** (lowest priority)

### Configuration Files

The system searches for configuration files in this order:
1. `~/.motor-test-config` (recommended)
2. `~/.config/motor-test/config.json`
3. `motor_test.config` (in current directory)
4. `config.json` (in current directory)

### Configuration Options

| Option | Description | Default |
|--------|-------------|---------|
| `sensor_port` | Torque sensor serial port | `/dev/tty.usbserial-FTAK88EN` |
| `brake_port` | Brake power supply port | `/dev/tty.usbmodem0000057700001` |
| `odrive_port` | ODrive serial number | `null` (auto-detect) |
| `velocity` | Test velocity in turns/s | `10.0` |
| `current` | Inertia test current in A | `2.0` |
| `thermal_duration` | Thermal test duration in s | `60.0` |
| `debug` | Enable debug output | `false` |

### Environment Variables

You can also configure via environment variables:

```bash
export MOTOR_TEST_SENSOR_PORT="/dev/ttyUSB0"
export MOTOR_TEST_BRAKE_PORT="/dev/ttyUSB1"
export MOTOR_TEST_VELOCITY=15.0
export MOTOR_TEST_DEBUG=true
```

### View Configuration

Check your current configuration:

```bash
motor-test config --show
```

## Usage

### Complete Test Sequence (Recommended)

Run both no-load and loaded tests:

```bash
motor-test --profile motor_profile.json
```

### No-Load Test Only

Test without brake/torque sensor (measures electrical properties and inertia):

```bash
motor-test --profile motor_profile.json --no-load-only
```

### Loaded Test Only

Test with brake and torque sensor (requires no-load results):

```bash
motor-test --profile motor_profile.json --loaded-only
```

### Skip Thermal Test

Run tests but skip the thermal characterization:

```bash
motor-test --profile motor_profile.json --skip-thermal
```

### Custom Parameters

Override configuration values:

```bash
motor-test --profile motor_profile.json \
  --velocity 15 \
  --current 3 \
  --thermal-duration 120 \
  --debug
```

### Custom Serial Ports

Override the configured serial ports:

```bash
motor-test --profile motor_profile.json \
  --sensor-port /dev/ttyUSB0 \
  --brake-port /dev/ttyUSB1
```

## Motor Profiles

Motor profiles are JSON files that describe the motor's characteristics.

### Required Fields

| Field | Description | Example |
|-------|-------------|---------|
| `name` | Motor identifier (used for results filename) | `"GMB2804"` |
| `description` | Motor description | `"Small gimbal motor"` |
| `motor_type` | Motor type: `"gimbal"` or `"high_current"` | `"gimbal"` |
| `pole_pairs` | Number of pole pairs | `7` |
| `kv_rating` | KV rating in RPM/V | `170` |
| `current_lim` | Maximum current in A | `8.0` |
| `calibration_current` | Calibration current in A | `5.5` |

### Additional Fields for Gimbal Motors

Gimbal motors require pre-known impedance values:

| Field | Description | Example |
|-------|-------------|---------|
| `phase_resistance` | Phase resistance in Ω | `9.0` |
| `phase_inductance` | Phase inductance in H | `0.001` |

### Optional Fields

| Field | Description | Example |
|-------|-------------|---------|
| `image_url` | URL to motor image | `"https://..."` |

### Example Profiles

#### Gimbal Motor

```json
{
  "name": "GMB2804",
  "description": "Small gimbal motor for testing",
  "motor_type": "gimbal",
  "pole_pairs": 7,
  "kv_rating": 170,
  "current_lim": 8.0,
  "calibration_current": 5.5,
  "phase_resistance": 9.0,
  "phase_inductance": 0.001
}
```

#### High-Current Motor

```json
{
  "name": "V2207_1500KV",
  "description": "High performance racing motor",
  "motor_type": "high_current",
  "pole_pairs": 7,
  "kv_rating": 1500,
  "current_lim": 40.0,
  "calibration_current": 10.0
}
```

## Hardware Setup

### No-Load Test

**Setup requirements:**
- Motor connected to ODrive
- Motor shaft **FREE TO SPIN** (no load)
- NO brake attached
- NO torque sensor attached
- Power supply connected to ODrive

**What it measures:**
- Phase resistance and inductance
- KV (velocity constant)
- No-load current
- Motor inertia

### Loaded Test

**Setup requirements:**
- Motor connected to ODrive
- Torque sensor between motor and brake
- Electromagnetic brake connected to power supply
- Brake positioned to load the motor shaft
- Correct serial ports configured

**What it measures:**
- KT (torque constant)
- Thermal resistance and time constant
- Max torque burst data

**Important:** You CANNOT run calibration with the brake/load attached. Always run the no-load test first to calibrate the motor, then attach the brake for loaded tests.

## Test Measurements

### No-Load Test Results

1. **Impedance**
   - Phase resistance (Ω) from calibration
   - Phase inductance (H) from calibration
   - Only for high-current motors (gimbal motors use profile values)

2. **KV (Velocity Constant)**
   - Measured in RPM/V
   - Calculated from back-EMF at 4 different speeds
   - Validates against profile specification

3. **No-Load Current**
   - Current required to maintain steady-state velocity
   - Indicates bearing friction and windage losses

4. **Rotor Inertia**
   - Moment of inertia (kg·m²)
   - Calculated from: J = τ / α where τ = Kt·Iq, α = dω/dt

### Loaded Test Results

1. **KT (Torque Constant)**
   - Measured in Nm/A
   - Measured from torque sensor at 4 current levels
   - Compared against theoretical value from KV

2. **Thermal Characteristics**
   - Thermal resistance (°C/W)
   - Thermal time constant (s)
   - Measured during sustained load test

3. **Burst Torque Data**
   - High-resolution (1kHz) torque data
   - 100ms duration at maximum current
   - Synchronized motor current and torque measurements

## Test Results

Results are automatically saved to `results/{motor_name}_results.json`.

### Result File Structure

```json
{
  "name": "GMB2804",
  "description": "Small gimbal motor for testing",
  "motor_type": "gimbal",
  "pole_pairs": 7,
  "electrical": {
    "phase_resistance_ohm": 9.0,
    "phase_inductance_mH": 1.0,
    "KV_rpm_per_V": 170.5,
    "KT_Nm_per_A": 0.0561
  },
  "mechanical": {
    "no_load_current_A": 0.15,
    "motor_inertia_kg_m2": 0.0000034
  },
  "thermal": {
    "thermal_resistance_C_per_W": 2.5,
    "thermal_time_constant_s": 45.0
  },
  "limits": {
    "max_current_A": 8.0,
    "calibration_current_A": 5.5
  },
  "torque_burst_test": {
    "duration_ms": 100,
    "max_current_A": 8.0,
    "data": [
      {
        "timestamp_s": 1234567890.123,
        "motor_current_A": 7.95,
        "torque_Nm": 0.446,
        "velocity_turns_s": 12.5
      }
    ]
  },
  "test_history": {
    "no_load": [
      {
        "timestamp": "2024-01-15T10:30:00",
        "bus_voltage_V": 24.1
      }
    ],
    "loaded": [
      {
        "timestamp": "2024-01-15T11:00:00",
        "bus_voltage_V": 24.0
      }
    ]
  }
}
```

### Viewing Results

Results can be viewed:
- As JSON files in the `results/` directory
- Using the included `viewer.html` (if available)
- By parsing the JSON in your own analysis tools

## Command Reference

### Main Commands

```bash
# Run complete test
motor-test --profile <profile.json>

# Run no-load test only
motor-test --profile <profile.json> --no-load-only

# Run loaded test only
motor-test --profile <profile.json> --loaded-only

# Skip thermal test
motor-test --profile <profile.json> --skip-thermal

# Enable debug mode
motor-test --profile <profile.json> --debug
```

### Configuration Commands

```bash
# Create configuration file
motor-test config --create

# Show current configuration
motor-test config --show

# Create config at custom location
motor-test config --create --path /path/to/config.json
```

### Options Reference

| Option | Short | Description | Default |
|--------|-------|-------------|---------|
| `--profile` | `-p` | Motor profile JSON file | Required |
| `--velocity` | `-v` | Test velocity (turns/s) | 10.0 |
| `--current` | `-i` | Inertia test current (A) | 2.0 |
| `--sensor-port` | | Torque sensor serial port | From config |
| `--brake-port` | | Brake power supply port | From config |
| `--odrive-port` | | ODrive serial number | Auto-detect |
| `--thermal-duration` | | Thermal test duration (s) | 60.0 |
| `--skip-thermal` | | Skip thermal test | False |
| `--no-load-only` | | Run only no-load test | False |
| `--loaded-only` | | Run only loaded test | False |
| `--debug` | | Enable debug output | False |
| `--help` | `-h` | Show help message | |
| `--version` | | Show version | |

### Complete Examples

```bash
# Basic complete test
motor-test --profile GMB2804_profile.json

# Custom velocity and current
motor-test -p motor.json -v 15 -i 3

# Custom serial ports
motor-test --profile motor.json \
  --sensor-port /dev/ttyUSB0 \
  --brake-port /dev/ttyUSB1

# Extended thermal test
motor-test --profile motor.json --thermal-duration 120

# Quick no-load test with debug
motor-test -p motor.json --no-load-only --debug
```

## Troubleshooting

### Import Errors After Installation

If you see import errors, ensure you're using the installed package:

```bash
# Good - uses installed package
motor-test --profile profile.json

# May have import issues
python3 motor_test.py --profile profile.json
```

### Serial Port Permission Denied

On Linux/macOS, you may need permissions for serial ports:

```bash
# Add user to dialout group (Linux)
sudo usermod -a -G dialout $USER

# Log out and back in for changes to take effect
```

On macOS, serial ports are usually accessible without special permissions.

### Motor Not Calibrated Error

If you see "Motor is not calibrated!" during loaded test:

1. Disconnect brake and torque sensor
2. Ensure motor shaft is free to spin
3. Run no-load test first: `motor-test --profile profile.json --no-load-only`
4. Reconnect brake/sensor
5. Run loaded test: `motor-test --profile profile.json --loaded-only`

### Torque Sensor Not Responding

Check:
- Sensor is powered and connected
- Correct serial port in configuration
- Baud rate settings (should be auto-configured)
- Try: `motor-test config --show` to verify port settings

### ODrive Connection Issues

- Verify ODrive is powered and connected via USB
- Check bus voltage is adequate (typically 12-24V)
- Try specifying ODrive serial number: `--odrive-port <serial>`
- Enable debug mode to see connection details: `--debug`

### Configuration Not Loading

Verify config file location and format:

```bash
# Check current config
motor-test config --show

# Recreate config file
motor-test config --create

# Edit with correct values
nano ~/.motor-test-config
```

### Results Not Saving

Check:
- `results/` directory exists (created automatically)
- Write permissions in project directory
- Motor name in profile doesn't contain invalid filename characters

## Notes and Best Practices

- **Test Order**: Always run no-load test before loaded test
- **Gimbal Motors**: Must provide `phase_resistance` and `phase_inductance` in profile
- **High-Current Motors**: Impedance measured automatically during calibration
- **KT Validation**: Loaded test compares measured KT against theoretical value from KV (should be within 10%)
- **Safety**: Ensure motor is properly mounted and brake is secure before testing
- **Cooling**: Allow motor to cool between tests, especially after thermal characterization
- **Bus Voltage**: Maintain stable power supply voltage during tests for accurate results

## Project Structure

```
motorTest/
├── motor_test/              # Main package
│   ├── cli.py               # Command-line interface
│   ├── config.py            # Configuration management
│   └── core/                # Core testing modules
│       ├── odrv_board.py    # ODrive interface
│       ├── test_results.py  # Results management
│       ├── no_load_test.py  # No-load test logic
│       ├── loaded_test.py   # Loaded test logic
│       ├── torque_sensor.py # Torque sensor interface
│       ├── brake.py         # Brake control
│       └── power_supply.py  # Power supply interface
├── pyproject.toml           # Package configuration
├── README.md                # This file
└── results/                 # Test results (git-ignored)
```

## License

Copyright © Pollen Robotics

## Support

For issues, questions, or contributions, please contact Pollen Robotics or file an issue in the project repository.
