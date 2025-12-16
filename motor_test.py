#!/usr/bin/env python3
"""
Complete Motor Test Script

Runs both no-load and loaded tests in sequence with user prompts
for hardware setup between tests.

Test sequence:
    1. No-load test (without brake/sensor):
       - Phase resistance and inductance
       - KV (velocity constant)
       - No-load current
       - Motor inertia

    2. Loaded test (with brake and torque sensor):
       - KT (torque constant)
       - Thermal characteristics
       - Max torque burst

Results are saved to results/{motor_name}_results.json
"""

import argparse
import time
from pathlib import Path

from odrv_board import ODriveBoard, MotorProfile
from test_results import MotorTestResults
from no_load_test import NoLoadTester
from loaded_test import LoadedTester
from torque_sensor import TorqueSensor
from brake import Brake
from power_supply import PowerSupply


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
            exit(0)


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

    # Run no-load tests
    tester = NoLoadTester(odrv, profile, debug=args.debug)
    result = tester.run_full_test(
        velocity_for_noload=args.velocity,
        current_for_inertia=args.current
    )

    # Update results with measurements
    results.update_motor_info(
        description=profile.description,
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

        # Connect to torque sensor
        print("Connecting to torque sensor...")
        sensor = TorqueSensor(args.sensor_port)
        sensor.connect()

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
            duration_ms=100,
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


def main():
    parser = argparse.ArgumentParser(
        description="Complete Motor Test - Run both no-load and loaded tests",
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

Examples:
  %(prog)s --profile GMB2804_profile.json
  %(prog)s --profile motor_profile.json --skip-thermal
  %(prog)s --profile motor_profile.json --velocity 15 --current 3
        """
    )

    parser.add_argument(
        "--profile", "-p",
        type=str,
        required=True,
        help="Motor profile JSON file"
    )

    # No-load test options
    parser.add_argument(
        "--velocity", "-v",
        type=float,
        default=10.0,
        help="Velocity for no-load current test (turns/s, default: 10)"
    )
    parser.add_argument(
        "--current", "-i",
        type=float,
        default=2.0,
        help="Current for inertia test (Amps, default: 2.0)"
    )

    # Loaded test options
    parser.add_argument(
        "--sensor-port",
        default="/dev/tty.usbserial-FTAK88EN",
        help="Torque sensor serial port"
    )
    parser.add_argument(
        "--brake-port",
        default="/dev/tty.usbmodem0000057700001",
        help="Brake power supply serial port"
    )
    parser.add_argument(
        "--skip-thermal",
        action="store_true",
        help="Skip thermal test in loaded test"
    )
    parser.add_argument(
        "--thermal-duration",
        type=float,
        default=60.0,
        help="Thermal test duration (seconds, default: 60)"
    )

    # Common options
    parser.add_argument(
        "--odrive-port",
        type=str,
        default=None,
        help="ODrive serial number"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
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

    args = parser.parse_args()

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
    exit(main())
