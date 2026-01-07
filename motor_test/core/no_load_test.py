#!/usr/bin/env python3
"""
Motor No-Load Test Script

Measures motor characteristics without mechanical load:
    - Phase resistance and inductance (from calibration)
    - No-load current: Current at steady-state velocity
    - Rotor inertia: J = τ / α where τ = Kt·Iq, α = dω/dt

Setup: Motor free to spin, no brake, no sensors.
"""

import argparse
import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional

from motor_test.core.odrv_board import ODriveBoard, MotorProfile, MotorState
from motor_test.core.test_results import MotorTestResults


@dataclass
class NoLoadTestResult:
    """Results from no-load motor testing."""
    motor_name: str

    # Electrical parameters (from calibration)
    phase_resistance_ohm: float
    phase_inductance_mH: float

    # No-load current measurement
    no_load_current_A: float  # Average Iq at steady-state (Amps)
    no_load_current_std: float  # Standard deviation
    test_velocity_turns_s: float  # Velocity used for no-load test

    # Inertia measurement
    motor_inertia_kg_m2: float  # Rotor inertia (kg·m²)
    inertia_test_current_A: float  # Current used for inertia test
    measured_acceleration_rad_s2: float  # Measured angular acceleration

    # Additional info
    bus_voltage_V: float
    ambient_temperature_C: Optional[float]


@dataclass
class DataPoint:
    """Single data point during measurement."""
    timestamp: float
    position: float  # turns
    velocity: float  # turns/s
    iq_measured: float  # Amps
    bus_voltage: float  # V


class NoLoadTester:
    """
    Performs no-load motor tests to characterize motor constants.
    """
    
    def __init__(
        self,
        odrv: ODriveBoard,
        profile: MotorProfile,
        debug: bool = False
    ):
        self.odrv = odrv
        self.profile = profile
        self.debug = debug
    
    def _log(self, msg: str) -> None:
        if self.debug:
            print(f"[NoLoadTest] {msg}")
    
    def _collect_data(
        self,
        duration: float,
        sample_rate: float = 100.0
    ) -> List[DataPoint]:
        """Collect motor data for a specified duration."""
        data = []
        interval = 1.0 / sample_rate
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            state = self.odrv.read_state()
            data.append(DataPoint(
                timestamp=state.timestamp,
                position=state.position,
                velocity=state.velocity,
                iq_measured=state.iq_measured,
                bus_voltage=state.bus_voltage,
            ))
            time.sleep(interval)
        
        return data

    def measure_impedance(self) -> tuple[float, float]:
        """Get motor phase resistance and inductance from ODrive calibration.

        Returns:
            Tuple of (resistance_ohm, inductance_H)
        """
        if self.profile.phase_resistance and self.profile.phase_inductance:
            # Use values from profile (for gimbal motors)
            resistance = self.profile.phase_resistance
            inductance = self.profile.phase_inductance
            self._log(f"Using impedance from profile: R={resistance}Ω, L={inductance}H")
        else:
            # Use values measured during calibration
            resistance = self.odrv.axis.motor.config.phase_resistance
            inductance = self.odrv.axis.motor.config.phase_inductance
            self._log(f"Using measured impedance: R={resistance}Ω, L={inductance}H")

        return resistance, inductance


    def measure_no_load_current(
        self,
        target_velocity: float = 10.0,  # turns/s
        settle_time: float = 2.0,
        measure_time: float = 3.0
    ) -> tuple[float, float]:
        """
        Measure no-load current at constant velocity.
        
        Args:
            target_velocity: Target velocity in turns/s
            settle_time: Time to wait for velocity to stabilize
            measure_time: Duration to measure current
        
        Returns:
            Tuple of (mean_current, std_current) in Amps
        """
        self._log(f"Measuring no-load current at {target_velocity} turns/s")
        
        # Configure velocity control
        self.odrv.set_control_mode('velocity')
        self.odrv.enable()
        
        # Ramp up to target velocity
        self._log("Ramping to target velocity...")
        self.odrv.set_velocity(target_velocity)
        time.sleep(settle_time)
        
        # Measure current at steady state
        self._log(f"Measuring current for {measure_time}s...")
        data = self._collect_data(measure_time)
        
        # Stop motor
        self.odrv.set_velocity(0)
        time.sleep(1.0)
        self.odrv.disable()
        
        # Calculate statistics
        currents = [d.iq_measured for d in data]
        mean_current = sum(currents) / len(currents)
        variance = sum((c - mean_current) ** 2 for c in currents) / len(currents)
        std_current = math.sqrt(variance)

        # Store as absolute value since direction doesn't matter for no-load current
        mean_current_abs = abs(mean_current)

        self._log(f"No-load current: {mean_current_abs:.4f} ± {std_current:.4f} A")
        return mean_current_abs, std_current
    
    def measure_inertia(
        self,
        test_current: float = 2.0,  # Amps
        acceleration_time: float = 0.5,  # seconds
        sample_rate: float = 500.0
    ) -> tuple[float, float]:
        """
        Measure motor inertia by applying a current step and measuring acceleration.
        
        J = τ / α = (Kt * Iq) / (dω/dt)
        
        Args:
            test_current: Current to apply for test (Amps)
            acceleration_time: Duration to accelerate
            sample_rate: Data sampling rate (Hz)
        
        Returns:
            Tuple of (inertia_kg_m2, acceleration_rad_s2)
        """
        self._log(f"Measuring inertia with {test_current}A test current")
        
        # Configure torque control
        self.odrv.set_control_mode('torque')
        self.odrv.enable()
        
        # Ensure motor is stopped
        self.odrv.set_torque(0)
        time.sleep(0.5)
        
        # Apply torque step and measure acceleration
        self._log("Applying torque step...")
        self.odrv.set_current(test_current)
        
        data = self._collect_data(acceleration_time, sample_rate)
        
        # Stop motor
        self.odrv.set_torque(0)
        time.sleep(0.5)
        
        # Decelerate gently
        self.odrv.set_control_mode('velocity')
        self.odrv.set_velocity(0)
        time.sleep(2.0)
        self.odrv.disable()
        
        # Calculate acceleration using linear regression on velocity
        # ω(t) = α*t + ω0
        if len(data) < 10:
            raise ValueError("Not enough data points for inertia measurement")
        
        # Use first timestamp as reference
        t0 = data[0].timestamp
        times = [d.timestamp - t0 for d in data]
        velocities_rad_s = [d.velocity * 2 * math.pi for d in data]  # Convert turns/s to rad/s
        
        # Linear regression: velocity = alpha * time + v0
        n = len(times)
        sum_t = sum(times)
        sum_v = sum(velocities_rad_s)
        sum_tv = sum(t * v for t, v in zip(times, velocities_rad_s))
        sum_t2 = sum(t * t for t in times)
        
        # α = (n*Σ(tv) - Σt*Σv) / (n*Σt² - (Σt)²)
        denominator = n * sum_t2 - sum_t ** 2
        if abs(denominator) < 1e-10:
            raise ValueError("Cannot compute acceleration - insufficient time variation")
        
        alpha = (n * sum_tv - sum_t * sum_v) / denominator  # rad/s²
        
        # Calculate torque: τ = Kt * Iq
        kt = self.profile.torque_constant
        torque = kt * test_current  # Nm
        
        # Calculate inertia: J = τ / α
        if abs(alpha) < 1e-6:
            raise ValueError("Acceleration too small - increase test current or reduce friction")
        
        inertia = abs(torque / alpha)  # kg·m²
        
        self._log(f"Acceleration: {alpha:.2f} rad/s²")
        self._log(f"Torque: {torque:.4f} Nm")
        self._log(f"Inertia: {inertia:.6e} kg·m²")
        
        return inertia, abs(alpha)
    
    def run_full_test(
        self,
        velocity_for_noload: float = 10.0,
        current_for_inertia: float = 2.0
    ) -> NoLoadTestResult:
        """
        Run complete no-load test sequence.

        Args:
            velocity_for_noload: Velocity for no-load current test (turns/s)
            current_for_inertia: Current for inertia measurement (Amps)

        Returns:
            NoLoadTestResult with all measurements
        """
        print("\n" + "=" * 60)
        print("NO-LOAD MOTOR TEST")
        print("=" * 60)

        # Get initial state
        state = self.odrv.read_state()
        bus_voltage = state.bus_voltage
        ambient_temp = state.motor_temperature

        print(f"\nMotor: {self.profile.name}")
        print(f"Bus voltage: {bus_voltage:.1f}V")

        # Test 1: Impedance (from calibration)
        print("\n--- Test 1: Impedance ---")
        phase_resistance, phase_inductance = self.measure_impedance()
        print(f"Phase resistance: {phase_resistance * 1000:.2f} mΩ")
        print(f"Phase inductance: {phase_inductance * 1e6:.2f} µH")

        # Test 2: No-load current
        print("\n--- Test 2: No-Load Current ---")
        mean_current, std_current = self.measure_no_load_current(
            target_velocity=velocity_for_noload
        )
        print(f"Result: {mean_current:.4f} ± {std_current:.4f} A")

        # Wait for motor to cool/settle
        time.sleep(2.0)

        # Test 3: Inertia measurement
        print("\n--- Test 3: Inertia Measurement ---")
        inertia, acceleration = self.measure_inertia(
            test_current=current_for_inertia
        )
        print(f"Result: J = {inertia:.6e} kg·m²")
        print(f"        α = {acceleration:.2f} rad/s²")

        # Compile results
        result = NoLoadTestResult(
            motor_name=self.profile.name,
            phase_resistance_ohm=phase_resistance,
            phase_inductance_mH=phase_inductance * 1000,
            no_load_current_A=mean_current,
            no_load_current_std=std_current,
            test_velocity_turns_s=velocity_for_noload,
            motor_inertia_kg_m2=inertia,
            inertia_test_current_A=current_for_inertia,
            measured_acceleration_rad_s2=acceleration,
            bus_voltage_V=bus_voltage,
            ambient_temperature_C=ambient_temp,
        )

        print("\n" + "=" * 60)
        print("NO-LOAD TEST COMPLETE")
        print("=" * 60)

        return result


def main():
    parser = argparse.ArgumentParser(
        description="Motor No-Load Test - Measure impedance, no-load current, and inertia",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --profile motor_profile.json
  %(prog)s --profile motor_profile.json --velocity 15 --current 3

Results are automatically saved to results/<motor>_results.json
        """
    )
    parser.add_argument(
        "--profile", "-p",
        type=str,
        required=True,
        help="Motor profile JSON file"
    )
    parser.add_argument(
        "--odrive-port",
        type=str,
        default=None,
        help="ODrive serial number"
    )
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
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output"
    )

    args = parser.parse_args()

    try:
        # Load motor profile
        profile = MotorProfile.from_json(args.profile)
        print(f"Loaded profile: {profile.name}")

        # Load or create test results
        results = MotorTestResults(profile.name)

        # Connect to ODrive
        odrv = ODriveBoard(debug=args.debug)
        print("Connecting to ODrive...")
        odrv.connect(serial_number=args.odrive_port)
        print(f"Connected. Bus voltage: {odrv.read_bus_voltage():.1f}V")

        # Load profile, save, reboot, then calibrate
        odrv.load_profile(profile)
        odrv.save_configuration()
        time.sleep(2)
        odrv.connect(serial_number=args.odrive_port)
        odrv.reboot()
        time.sleep(2)
        odrv.connect(serial_number=args.odrive_port)
        print(f"ODrive configured. Bus voltage: {odrv.read_bus_voltage():.1f}V")

        print("Running calibration...")
        odrv.calibrate()
        print("Calibration complete.")

        # Run tests
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
            kv_rating=profile.kv_rating,
            weight_g=profile.weight_g
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
        print(f"\nResults saved to: {results_path}")
        print(f"\n{results}")

        # Cleanup
        odrv.disconnect()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
