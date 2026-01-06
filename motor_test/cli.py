#!/usr/bin/env python3
"""
Motor Test Suite CLI

Complete motor characterization test suite for Pollen Robotics test bench.
"""

import argparse
import sys
import time
from pathlib import Path

from motor_test.config import get_config
from motor_test.core.odrv_board import ODriveBoard, MotorProfile
from motor_test.core.test_results import MotorTestResults
from motor_test.core.no_load_test import NoLoadTester
from motor_test.core.loaded_test import LoadedTester
from motor_test.core.torque_sensor import TorqueSensor
from motor_test.core.brake import Brake
from motor_test.core.power_supply import PowerSupply


def print_section_header(title: str, char: str = "="):
    """Print a formatted section header."""
    print("\n" + char * 60)
    print(f"  {title}")
    print(char * 60)


def wait_for_user(prompt: str) -> bool:
    """Wait for user confirmation to continue.

    Returns:
        True if user wants to continue, False if they want to skip
    """
    while True:
        response = input(f"\n{prompt} (y/n/skip): ").strip().lower()
        if response in ['y', 'yes']:
            return True
        elif response == 'skip':
            return False
        elif response in ['n', 'no']:
            print("Test cancelled by user.")
            sys.exit(0)


def run_no_load_test(profile: MotorProfile, args) -> None:
    """Run no-load motor test."""
    print_section_header("PART 1: NO-LOAD TEST")

    print("\nHARDWARE SETUP FOR NO-LOAD TEST:")
    print("  ✓ Motor connected to ODrive")
    print("  ✓ Motor shaft is FREE TO SPIN")
    print("  ✓ NO brake attached")
    print("  ✓ NO torque sensor attached")
    print("  ✓ Power supply connected to ODrive")

    if not wait_for_user("Ready to start no-load test?"):
        print("Skipping no-load test...")
        return

    # Load or create test results
    results = MotorTestResults(profile.name)

    # Connect to ODrive
    odrv = ODriveBoard(debug=args.debug)
    print("\nConnecting to ODrive...")
    odrv.connect(serial_number=args.odrive_port)
    print(f"Connected. Bus voltage: {odrv.read_bus_voltage():.1f}V")

    # Erase old configuration, load profile, save, reboot
    print("Erasing old configuration...")
    odrv.erase_configuration()
    time.sleep(2)
    odrv.connect(serial_number=args.odrive_port)
    print("Configuring ODrive...")
    odrv.load_profile(profile)
    odrv.save_configuration()
    time.sleep(2)
    odrv.connect(serial_number=args.odrive_port)
    odrv.reboot()
    time.sleep(2)
    odrv.connect(serial_number=args.odrive_port)
    print(f"ODrive configured. Bus voltage: {odrv.read_bus_voltage():.1f}V")

    print("\nRunning calibration...")
    odrv.calibrate()
    print("Calibration complete.")

    # Mark calibrations as complete and save to persist pre_calibrated flags
    print("Saving calibration...")
    odrv.mark_as_precalibrated()
    odrv.save_configuration()
    time.sleep(2)
    odrv.connect(serial_number=args.odrive_port)
    print("Calibration saved - motor can skip calibration on next boot")

    # Run no-load tests
    tester = NoLoadTester(odrv, profile, debug=args.debug)
    result = tester.run_full_test(
        velocity_for_noload=args.velocity,
        current_for_inertia=args.current
    )

    # Update results with measurements
    results.update_motor_info(
        description=profile.description,
        image_url=profile.image_url,
        motor_type=profile.motor_type,
        pole_pairs=profile.pole_pairs,
        kv_rating=profile.kv_rating
    )
    results.update_electrical(
        phase_resistance_ohm=result.phase_resistance_ohm,
        phase_inductance_mH=result.phase_inductance_mH
    )
    results.update_mechanical(
        no_load_current_a=result.no_load_current_A,
        motor_inertia_kg_m2=result.motor_inertia_kg_m2
    )
    results.update_limits(
        max_current_a=profile.current_lim,
        calibration_current_a=profile.calibration_current
    )
    results.add_test_metadata("no_load", result.bus_voltage_V)

    # Save results
    results_path = results.save()
    print(f"\nNo-load test results saved to: {results_path}")

    # Cleanup
    odrv.disable()
    odrv.disconnect()

    print("\n✓ No-load test complete!")


def run_loaded_test(profile: MotorProfile, args) -> None:
    """Run loaded motor test."""
    print_section_header("PART 2: LOADED TEST")

    print("\nHARDWARE SETUP FOR LOADED TEST:")
    print("  ✓ Motor connected to ODrive")
    print("  ✓ Torque sensor connected between motor and brake")
    print("  ✓ Electromagnetic brake connected to power supply")
    print("  ✓ Brake positioned to load the motor shaft")
    print("  ✓ Torque sensor port:", args.sensor_port)
    print("  ✓ Brake power supply port:", args.brake_port)

    if not wait_for_user("Ready to start loaded test?"):
        print("Skipping loaded test...")
        return

    odrv = None
    brake = None
    sensor = None

    try:
        # Load test results (should already exist from no-load test)
        results = MotorTestResults(profile.name)

        # Connect to ODrive
        print("\nConnecting to ODrive...")
        odrv = ODriveBoard(debug=True)
        odrv.connect()

        # Load profile configuration
        odrv.load_profile(profile)
        print(f"Bus voltage: {odrv.read_bus_voltage():.1f}V")

        # Check if calibration exists - CANNOT calibrate with load attached
        if not odrv.is_calibrated():
            raise RuntimeError(
                "Motor is not calibrated! The loaded test requires a calibrated motor.\n"
                "You CANNOT run calibration with the brake/load attached.\n\n"
                "To fix this:\n"
                "  1. Disconnect the brake and torque sensor\n"
                "  2. Ensure motor shaft is FREE to spin\n"
                "  3. Run the no-load test first: motor-test --profile <profile.json> --no-load-only\n"
                "  4. Then reconnect brake/sensor and run loaded test\n\n"
                "Or run the complete test sequence without --loaded-only flag."
            )

        # Connect to torque sensor
        print("Connecting to torque sensor...")
        sensor = TorqueSensor(args.sensor_port)
        sensor.connect()
        time.sleep(1.5)

        rate = sensor.get_data_rate()
        if rate < 10:
            raise RuntimeError(f"Torque sensor not receiving data on {args.sensor_port}")

        samples = sensor.read_recent(100)
        if not samples:
            raise RuntimeError("Torque sensor subprocess not writing data")
        avg_torque = sum(s.torque for s in samples) / len(samples)
        print(f"  Sensor: {rate:.1f} Hz, avg={avg_torque:.4f} Nm")

        # Connect to electromagnetic brake via power supply
        print("Connecting to brake...")
        psu = PowerSupply(args.brake_port)
        brake = Brake(psu)

        # Run complete test sequence
        tester = LoadedTester(odrv, profile, sensor, brake, results)
        result = tester.run_full_test(
            profile.current_lim,
            args.skip_thermal,
            args.thermal_duration
        )

        # Update results with new measurements
        results.update_electrical(kt_nm_per_a=result.KT_Nm_per_A)
        results.update_thermal(
            thermal_resistance_c_per_w=result.thermal_resistance_C_per_W,
            thermal_time_constant_s=result.thermal_time_constant_s
        )
        results.add_torque_burst_test(
            max_current_a=result.max_test_current_A,
            duration_ms=result.burst_duration_ms,
            data=result.torque_burst_data
        )
        results.add_test_metadata("loaded", result.bus_voltage_V)

        # Save results to JSON file
        results_path = results.save()
        print(f"\nLoaded test results saved to: {results_path}")

        print("\n✓ Loaded test complete!")

    finally:
        # Stop motor and release brake on exit
        if odrv:
            odrv.disable()
        if brake:
            brake.release()
        if sensor:
            sensor.disconnect()


def create_parser():
    """Create argument parser with configuration defaults."""
    config = get_config()

    parser = argparse.ArgumentParser(
        prog="motor-test",
        description="Complete Motor Test Suite - Characterize motors with ODrive",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Complete test sequence with hardware setup prompts:

1. No-load test (motor free to spin):
   - Measures phase resistance, inductance, KV
   - Measures no-load current and inertia

2. Loaded test (with brake and torque sensor):
   - Measures KT (torque constant)
   - Measures thermal characteristics
   - Records max torque burst data

Configuration:
  Settings can be configured via:
    - Command line arguments (highest priority)
    - Environment variables (MOTOR_TEST_*)
    - Config file (~/.motor-test-config or motor_test.config)
    - Default values (lowest priority)

  Use 'motor-test config' to create an example config file.

Examples:
  motor-test --profile GMB2804_profile.json
  motor-test --profile motor_profile.json --skip-thermal
  motor-test --profile motor_profile.json --velocity 15 --current 3
  motor-test --profile motor_profile.json --no-load-only
        """
    )

    # Subcommands
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # Config subcommand
    config_parser = subparsers.add_parser(
        'config',
        help='Manage configuration'
    )
    config_parser.add_argument(
        '--create',
        action='store_true',
        help='Create example configuration file'
    )
    config_parser.add_argument(
        '--show',
        action='store_true',
        help='Show current configuration'
    )
    config_parser.add_argument(
        '--path',
        type=Path,
        help='Custom path for config file'
    )

    # Test subcommand (default)
    test_parser = subparsers.add_parser(
        'test',
        help='Run motor tests (default command)'
    )

    # Required arguments
    parser.add_argument(
        "--profile", "-p",
        type=str,
        help="Motor profile JSON file (required for test command)"
    )

    # No-load test options
    parser.add_argument(
        "--velocity", "-v",
        type=float,
        default=config.get("velocity"),
        help=f"Velocity for no-load current test (turns/s, default: {config.get('velocity')})"
    )
    parser.add_argument(
        "--current", "-i",
        type=float,
        default=config.get("current"),
        help=f"Current for inertia test (Amps, default: {config.get('current')})"
    )

    # Loaded test options
    parser.add_argument(
        "--sensor-port",
        default=config.get("sensor_port"),
        help=f"Torque sensor serial port (default: {config.get('sensor_port')})"
    )
    parser.add_argument(
        "--brake-port",
        default=config.get("brake_port"),
        help=f"Brake power supply serial port (default: {config.get('brake_port')})"
    )
    parser.add_argument(
        "--skip-thermal",
        action="store_true",
        help="Skip thermal test in loaded test"
    )
    parser.add_argument(
        "--thermal-duration",
        type=float,
        default=config.get("thermal_duration"),
        help=f"Thermal test duration (seconds, default: {config.get('thermal_duration')})"
    )

    # Common options
    parser.add_argument(
        "--odrive-port",
        type=str,
        default=config.get("odrive_port"),
        help="ODrive serial number (default: auto-detect)"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        default=config.get("debug"),
        help="Enable debug output"
    )
    parser.add_argument(
        "--no-load-only",
        action="store_true",
        help="Run only no-load test"
    )
    parser.add_argument(
        "--loaded-only",
        action="store_true",
        help="Run only loaded test (requires no-load test results)"
    )

    # Version
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s 1.0.0"
    )

    return parser


def handle_config_command(args):
    """Handle config subcommand."""
    config = get_config()

    if args.create:
        path = args.path if args.path else None
        created_path = config.create_example_config(path)
        print(f"Example configuration file created at: {created_path}")
        print("\nEdit this file to customize your test bench setup.")
        return 0

    if args.show:
        print("Current configuration:")
        for key, value in config.config.items():
            print(f"  {key}: {value}")
        return 0

    # Default: show help
    print("Use 'motor-test config --help' for options")
    return 0


def main():
    """Main entry point for CLI."""
    parser = create_parser()
    args = parser.parse_args()

    # Handle config command
    if args.command == 'config':
        return handle_config_command(args)

    # Default to test command if no command specified
    if not args.command:
        args.command = 'test'

    # Validate profile argument for test command
    if args.command == 'test' and not args.profile:
        parser.error("--profile is required for test command")

    try:
        # Load motor profile
        profile = MotorProfile.from_json(args.profile)

        print_section_header("COMPLETE MOTOR TEST")
        print(f"\nMotor: {profile.name}")
        print(f"Type: {profile.motor_type}")
        print(f"Pole pairs: {profile.pole_pairs}")
        print(f"KV rating: {profile.kv_rating} RPM/V")
        print(f"Current limit: {profile.current_lim} A")

        # Run tests based on flags
        if args.loaded_only:
            # Only run loaded test
            run_loaded_test(profile, args)
        elif args.no_load_only:
            # Only run no-load test
            run_no_load_test(profile, args)
        else:
            # Run both tests in sequence
            run_no_load_test(profile, args)
            run_loaded_test(profile, args)

        # Print final summary
        print_section_header("ALL TESTS COMPLETE", "=")
        results = MotorTestResults(profile.name)
        print(f"\n{results}")
        print(f"\nFinal results saved to: {results.results_path}")

        # Update GitHub Pages results index
        print("\nUpdating results index for GitHub Pages...")
        try:
            import subprocess
            # Get repository root and docs folder
            repo_root = Path(__file__).parent.parent
            docs_dir = repo_root / 'docs'
            result = subprocess.run(
                ["python3", "generate_index.py"],
                capture_output=True,
                text=True,
                cwd=docs_dir
            )
            if result.returncode == 0:
                print("✓ Results index updated")
            else:
                print(f"⚠ Failed to update results index: {result.stderr}")
        except Exception as e:
            print(f"⚠ Could not update results index: {e}")

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        return 1
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
