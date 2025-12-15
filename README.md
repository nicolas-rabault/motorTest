# Motor Test Suite

Complete motor characterization test suite for Pollen Robotics test bench.

## Quick Start

Create a profile for your motor:

````json
{
  "name": "motor_name",
  "description": "motor_description",
  "motor_type": "gimbal", # "gimbal" or "high_current"
  "pole_pairs": 7,
  "kv_rating": 170
}

Run the complete test sequence:
```bash
./motor_test.py --profile motor_name_profile.json
````

The script will guide you through:

1. **No-load test** - motor free to spin (measures impedance, KV, no-load current, inertia)
2. **Loaded test** - with brake and torque sensor (measures KT, thermal, burst torque)

## Test Files

- **`motor_test.py`** - Main script that runs both tests with user prompts
- **`no_load_test.py`** - No-load test only (can be run independently)
- **`loaded_test.py`** - Loaded test only (requires no-load results)

## Usage

### Complete Test (Recommended)

```bash
./motor_test.py --profile motor_name_profile.json
```

### No-Load Test Only

```bash
./motor_test.py --profile motor_name_profile.json --no-load-only
```

### Loaded Test Only (if no-load already done)

```bash
./motor_test.py --profile motor_name_profile.json --loaded-only
```

### Skip Thermal Test

```bash
./motor_test.py --profile motor_name_profile.json --skip-thermal
```

## Hardware Setup

### No-Load Test

- Motor connected to ODrive
- Motor shaft **FREE TO SPIN**
- NO brake attached
- NO torque sensor attached

### Loaded Test

- Motor connected to ODrive
- Torque sensor between motor and brake
- Electromagnetic brake connected to power supply
- Brake positioned to load motor shaft

## Command-Line Options

### Required

- `--profile, -p` - Motor profile JSON file

### No-Load Test Options

- `--velocity, -v` - Test velocity in turns/s (default: 10)
- `--current, -i` - Inertia test current in A (default: 2.0)

### Loaded Test Options

- `--sensor-port` - Torque sensor serial port (default: /dev/tty.usbserial-FTAK88EN)
- `--brake-port` - Brake power supply port (default: /dev/tty.usbmodem0000057700001)
- `--skip-thermal` - Skip thermal characterization
- `--thermal-duration` - Thermal test duration in seconds (default: 60)

### Common Options

- `--odrive-port` - ODrive serial number (optional)
- `--debug` - Enable debug output
- `--no-load-only` - Run only no-load test
- `--loaded-only` - Run only loaded test

## Motor Profile Fields

- `name` - Motor identifier (used for results filename)
- `description` - Motor description
- `motor_type` - "gimbal" or "high_current"
- `pole_pairs` - Number of pole pairs
- `kv_rating` - KV rating in RPM/V
- `current_lim` - Maximum current in A
- `calibration_current` - Calibration current in A
- `phase_resistance` - Phase resistance in Ω (required for gimbal motors)
- `phase_inductance` - Phase inductance in H (required for gimbal motors)

## Test Results

Results are saved to `results/{motor_name}_results.json`:

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
    "data": [...]
  },
  "test_history": {
    "no_load": [...],
    "loaded": [...]
  }
}
```

## Test Measurements

### No-Load Test

1. **Impedance** - Phase resistance (Ω) and inductance (H) from calibration
2. **KV** - Velocity constant (RPM/V) from back-EMF at 4 speeds
3. **No-load current** - Current at steady-state velocity
4. **Inertia** - Rotor moment of inertia (kg·m²)

### Loaded Test

1. **KT** - Torque constant (Nm/A) from torque sensor at 4 currents
2. **Thermal** - Thermal resistance (°C/W) and time constant (s)
3. **Burst** - Max torque burst data at 1kHz for 100ms

## Notes

- **Gimbal motors**: Must provide `phase_resistance` and `phase_inductance` in profile
- **High-current motors**: Impedance measured automatically during calibration
- **KV measurement**: Run no-load test first (without brake)
- **KT validation**: Loaded test compares measured KT against theoretical value from KV
- **Test order**: Always run no-load test before loaded test
