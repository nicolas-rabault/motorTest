# Motor Test Suite

Complete motor characterization test suite for BLDC motors using ODrive controllers.

This package provides automated testing measuring electrical characteristics, mechanical properties, and thermal behavior.

## Installation

### Prerequisites

- Python 3.8 or higher
- ODrive controller
- Torque sensor (for loaded tests)
- Electromagnetic brake (for loaded tests)

### Install from source

```bash
# Clone the repository
git clone https://github.com/nicolas-rabault/motorTest.git
cd motorTest

# Install the package
pip install -e .
```

### Verify installation

```bash
motor-test --help
```

## Quick Start

1. **Create a configuration file:**

```bash
motor-test config --create
```

2. **Create a motor profile** (`my_motor.json`):

```json
{
  "name": "GMB2804",
  "motor_type": "gimbal",
  "pole_pairs": 7,
  "kv_rating": 170,
  "current_lim": 8.0,
  "calibration_current": 5.5,
  "phase_resistance": 9.0,
  "phase_inductance": 0.001
}
```

Profiles are stored in the `profile/` directory.

3. **Run the tests:**

```bash
motor-test --profile my_motor.json
```

Results are saved to `results/{motor_name}_results.json`.

## Viewing Results

### Local Viewing

Open [viewer.html](https://nicolas-rabault.github.io/motorTest/viewer.html) in your browser and load a result file from the `results/` directory.

### GitHub Pages (Online Viewer)

This repository can be published as a GitHub Pages site to view results online:

1. **Enable GitHub Pages** in repository settings:

   - Go to Settings → Pages
   - Source: "Deploy from a branch"
   - Branch: `master` (or `main`)
   - Folder: `/docs`
   - Save

2. **The results index is automatically updated** after each test completes. To manually regenerate:

   ```bash
   python3 generate_index.py
   ```

3. **Commit and push your results**:

   ```bash
   git add results/ docs/results-index.json
   git commit -m "Add new motor test results"
   git push
   ```

4. **Access your results online**:
   - Landing page: `https://<username>.github.io/motorTest/`
   - Direct link: `https://<username>.github.io/motorTest/viewer.html?file=<result_file>.json`

The index generator script scans the `results/` directory and creates a list of available results for the online viewer.

## Tests

The suite performs two test sequences:

1. **No-load test** - Motor free to spin

   - Measures: impedance, KV, no-load current, inertia

2. **Loaded test** - With brake and torque sensor
   - Measures: KT, thermal behavior, burst torque

## Usage Examples

```bash
# Complete test (both no-load and loaded)
motor-test --profile motor.json

# No-load test only
motor-test --profile motor.json --no-load-only

# Loaded test only (requires prior no-load results)
motor-test --profile motor.json --loaded-only

# Skip thermal test
motor-test --profile motor.json --skip-thermal

# Custom parameters
motor-test --profile motor.json --velocity 15 --current 3

# View configuration
motor-test config --show
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Contributing

Issues and pull requests are welcome.
