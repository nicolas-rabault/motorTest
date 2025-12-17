#!/usr/bin/env python3
"""
Motor Loaded Test Script

Measures motor performance under load using torque sensor and brake:
    - KT: Torque constant (Nm/A), validated against spec KV
    - Thermal resistance and time constant
    - Max torque burst data
"""

import argparse
import math
import time
import numpy as np
from dataclasses import dataclass, asdict, field
from typing import List, Optional, Dict, Any
from scipy import stats

from odrv_board import ODriveBoard, MotorProfile
from torque_sensor import TorqueSensor
from brake import Brake
from power_supply import PowerSupply
from test_results import MotorTestResults


@dataclass
class TorqueBurstDataPoint:
    """Single data point during torque burst test."""
    timestamp_ms: float  # Time from burst start (ms)
    measured_torque_Nm: float  # From torque sensor
    estimated_torque_Nm: float  # From ODrive (Kt * Iq)
    current_A: float  # Iq measured
    velocity_turns_s: float
    position_turns: float
    motor_temperature_C: Optional[float]
    bus_voltage_V: float
    measured_speed_RPM: int = 0  # From torque sensor


@dataclass
class ThermalTestData:
    """Data from thermal characterization."""
    thermal_resistance_C_per_W: float
    time_constant_s: float


@dataclass
class LoadedTestResult:
    """Complete results from loaded motor testing."""
    motor_name: str

    # Motor constants
    KT_Nm_per_A: float  # Measured from torque sensor

    # Thermal parameters
    thermal_resistance_C_per_W: float
    thermal_time_constant_s: float

    # Test conditions
    bus_voltage_V: float
    max_test_current_A: float

    # Torque burst test results
    torque_burst_data: List[Dict[str, Any]] = field(default_factory=list)


class LoadedTester:
    """Performs loaded motor tests to characterize motor constants."""

    def __init__(
        self,
        odrv: ODriveBoard,
        profile: MotorProfile,
        torque_sensor: TorqueSensor,
        brake: Brake,
        results: MotorTestResults
    ):
        self.odrv = odrv
        self.profile = profile
        self.torque_sensor = torque_sensor
        self.brake = brake
        self.results = results

    def measure_kt(self, test_currents: List[float]) -> tuple[float, float]:
        """Measure torque constant KT using torque sensor. Returns (KT, zero_offset)."""
        # Step 1: Measure sensor zero offset with no load
        print("  Measuring sensor zero offset...")
        self.brake.release()
        self.odrv.disable()
        time.sleep(1.0)

        zero_offset = self.torque_sensor.get_zero_offset(1.0)
        print(f"  Zero offset: {zero_offset:.4f} Nm (will be compensated)")

        # Step 2: Enable brake and configure motor
        print("  Engaging brake...")
        self.brake.set_intensity(100.0)
        self.brake.enable()
        time.sleep(1.0)

        self.odrv.disable()
        time.sleep(0.1)
        self.odrv.set_control_mode('torque')
        time.sleep(0.1)
        self.odrv.axis.controller.input_torque = 0.0
        self.odrv.enable()
        time.sleep(0.2)

        # Collect all torque and current data points for linear regression
        all_torques = []
        all_currents = []

        # Test at each current level
        for current in test_currents:
            self.odrv.set_current(current)
            time.sleep(1.5)

            # Collect torque data from last 2 seconds
            samples = self.torque_sensor.read_range(2.0)
            if not samples:
                print(f"  ⚠️  No torque readings at {current}A")
                continue

            torques = [abs(s.torque - zero_offset) for s in samples]

            # Collect current/velocity/position from ODrive
            currents, velocities, positions = [], [], []
            start = time.time()
            while (time.time() - start) < 2.0:
                state = self.odrv.read_state()
                currents.append(abs(state.iq_measured))
                velocities.append(state.velocity)
                positions.append(state.position)
                time.sleep(0.01)

            avg_torque = sum(torques) / len(torques)
            avg_current = sum(currents) / len(currents)
            avg_velocity = sum(velocities) / len(velocities)

            print(f"  {current:.1f}A → {avg_torque:.4f} Nm (Iq={avg_current:.2f}A, ω={avg_velocity:.3f} turns/s)")

            if abs(avg_velocity) > 0.1:
                print(f"     ⚠️  Motor spinning - brake not holding!")
            elif abs(avg_velocity) > 0.05:
                print(f"     ⚠️  Motor creeping")

            # Resample to match lengths and add to dataset for linear regression
            min_len = min(len(torques), len(currents))
            if min_len > 0:
                # Resample both arrays to the minimum length using interpolation
                torque_indices = np.linspace(0, len(torques) - 1, min_len)
                current_indices = np.linspace(0, len(currents) - 1, min_len)

                resampled_torques = np.interp(torque_indices, np.arange(len(torques)), torques)
                resampled_currents = np.interp(current_indices, np.arange(len(currents)), currents)

                all_torques.extend(resampled_torques)
                all_currents.extend(resampled_currents)

        self.odrv.set_current(0)
        self.odrv.disable()
        self.brake.release()

        if not all_torques:
            raise RuntimeError("No valid KT measurements")

        # Linear regression: torque = KT * current
        slope, intercept, r_value, _, _ = stats.linregress(all_currents, all_torques)

        print(f"\n  KT = {slope:.4f} Nm/A  (R²={r_value**2:.4f})")

        if abs(intercept) > 0.02:
            print(f"  ⚠️  Large intercept: {intercept:.4f} Nm")
        if r_value**2 < 0.98:
            print(f"  ⚠️  Poor fit: R²={r_value**2:.4f}")

        return slope, zero_offset
    
    def measure_thermal_parameters(self, test_current: float, heating_duration: float) -> ThermalTestData:
        """Measure thermal resistance and time constant."""
        # Record ambient temperature before heating
        state = self.odrv.read_state()
        ambient_temp = state.motor_temperature or state.fet_temperature or 25.0

        # Enable brake at 30% intensity and configure torque control mode
        self.brake.set_intensity(30.0)
        self.brake.enable()
        time.sleep(0.5)
        self.odrv.disable()
        time.sleep(0.1)
        self.odrv.set_control_mode('torque')
        time.sleep(0.1)
        self.odrv.enable()

        # Apply constant current and record temperature rise over time
        heating_data = []
        self.odrv.set_current(test_current)
        phase_resistance = self.profile.phase_resistance or self.odrv.axis.motor.config.phase_resistance
        start_time = time.time()

        while (time.time() - start_time) < heating_duration:
            state = self.odrv.read_state()
            temp = state.motor_temperature or state.fet_temperature
            current = state.iq_measured
            power = (current ** 2) * phase_resistance * 1.5

            heating_data.append({
                'time': time.time() - start_time,
                'temperature': temp,
                'power': power
            })
            time.sleep(1.0)

        # Stop motor, disable, and release brake
        self.odrv.set_current(0)
        self.odrv.disable()
        self.brake.release()

        # Calculate thermal resistance: R_th = ΔT / P
        final_temp = heating_data[-1]['temperature'] if heating_data else ambient_temp
        temp_rise = final_temp - ambient_temp
        avg_power = sum(d['power'] for d in heating_data) / len(heating_data) if heating_data else 1.0
        thermal_resistance = temp_rise / avg_power if avg_power > 0.1 else 0.0

        # Calculate thermal time constant: time to reach 63.2% of final temperature rise
        target_temp = ambient_temp + 0.632 * temp_rise
        time_constant = heating_duration
        for data in heating_data:
            if data['temperature'] >= target_temp:
                time_constant = data['time']
                break

        return ThermalTestData(
            thermal_resistance_C_per_W=thermal_resistance,
            time_constant_s=time_constant
        )
    
    def run_torque_burst_test(self, max_current: float, zero_offset: float = 0.0) -> List[TorqueBurstDataPoint]:
        """Run max torque burst test and record high-speed data."""
        # Enable brake at 100% intensity to fully lock motor shaft
        self.brake.set_intensity(100.0)
        self.brake.enable()
        time.sleep(0.5)

        # Configure torque control mode and enable
        self.odrv.disable()
        time.sleep(0.1)
        self.odrv.set_control_mode('torque')
        time.sleep(0.1)
        self.odrv.enable()

        # Apply maximum current burst for 100ms
        self.odrv.set_current(max_current)
        time.sleep(0.1)
        self.odrv.set_current(0)

        # Get torque sensor data from the burst (last 100ms)
        sensor_samples = self.torque_sensor.read_range(0.1)

        # Get ODrive state samples (sample every 3ms for ~33 points)
        odrv_samples = []
        elapsed = 0.0
        for _ in range(33):
            state = self.odrv.read_state()
            odrv_samples.append((elapsed * 1000, state))
            elapsed += 0.003
            if elapsed > 0.1:
                break

        # Merge sensor and ODrive data (match by timestamp)
        data_points = []
        burst_start = time.time() - 0.1

        for elapsed_ms, state in odrv_samples:
            # Find closest sensor sample to this ODrive sample
            target_time = burst_start + (elapsed_ms / 1000.0)
            closest = min(sensor_samples, key=lambda s: abs(s.timestamp - target_time))

            data_points.append(TorqueBurstDataPoint(
                timestamp_ms=elapsed_ms,
                measured_torque_Nm=closest.torque - zero_offset,
                estimated_torque_Nm=state.torque_estimate,
                current_A=state.iq_measured,
                velocity_turns_s=state.velocity,
                position_turns=state.position,
                motor_temperature_C=state.motor_temperature,
                bus_voltage_V=state.bus_voltage,
                measured_speed_RPM=closest.speed
            ))

        # Stop motor, disable, and release brake
        self.odrv.disable()
        self.brake.release()

        return data_points
    
    def run_full_test(self, max_current: float, skip_thermal: bool, thermal_duration: float) -> LoadedTestResult:
        """Run complete loaded motor test sequence (KT, thermal, burst)."""
        print("\n" + "=" * 60)
        print("LOADED MOTOR TEST")
        print("=" * 60)

        state = self.odrv.read_state()
        print(f"\nMotor: {self.profile.name}")
        print(f"Bus voltage: {state.bus_voltage:.1f}V")
        print(f"Max current: {max_current}A")

        # Test 1: Measure KT
        print("\n--- KT Measurement ---")
        kt, zero_offset = self.measure_kt([1.0, 2.0, 3.0, 4.0, min(5.0, max_current)])

        if kt < 0.0001:
            raise RuntimeError("KT measurement failed - no torque measured")

        kv_from_kt = 60.0 / (2.0 * math.pi * kt)
        kv_spec = self.profile.kv_rating
        error_percent = abs(kv_from_kt - kv_spec) / kv_spec * 100

        print(f"  KV (spec): {kv_spec:.1f} RPM/V")
        print(f"  KV (measured): {kv_from_kt:.1f} RPM/V  (Δ {error_percent:.1f}%)")

        # Test 2: Thermal
        thermal_resistance = 0.0
        thermal_time_constant = 0.0
        if not skip_thermal:
            print(f"\n--- Thermal Test (~{thermal_duration:.0f}s) ---")
            thermal_data = self.measure_thermal_parameters(min(3.0, max_current), thermal_duration)
            thermal_resistance = thermal_data.thermal_resistance_C_per_W
            thermal_time_constant = thermal_data.time_constant_s
            print(f"  R_th: {thermal_resistance:.2f} °C/W")
            print(f"  τ: {thermal_time_constant:.1f}s")

        # Test 3: Torque burst
        print("\n--- Torque Burst (100ms) ---")
        burst_data = self.run_torque_burst_test(max_current, zero_offset)
        if burst_data:
            avg_measured_torque = sum(d.measured_torque_Nm for d in burst_data) / len(burst_data)
            avg_current = sum(d.current_A for d in burst_data) / len(burst_data)
            print(f"  Torque: {avg_measured_torque:.4f} Nm @ {avg_current:.2f} A")

        print("\n" + "=" * 60)
        print("LOADED TEST COMPLETE")
        print("=" * 60)

        return LoadedTestResult(
            motor_name=self.profile.name,
            KT_Nm_per_A=kt,
            thermal_resistance_C_per_W=thermal_resistance,
            thermal_time_constant_s=thermal_time_constant,
            bus_voltage_V=state.bus_voltage,
            max_test_current_A=max_current,
            torque_burst_data=[asdict(d) for d in burst_data]
        )


def main():
    parser = argparse.ArgumentParser(description="Motor Loaded Test - Measure KT, thermal, and burst torque")
    parser.add_argument("--profile", "-p", required=True, help="Motor profile JSON file")
    parser.add_argument("--sensor-port", default="/dev/tty.usbserial-FTAK88EN", help="Torque sensor serial port")
    parser.add_argument("--brake-port", default="/dev/tty.usbmodem0000057700001", help="Brake power supply serial port")
    parser.add_argument("--skip-thermal", action="store_true", help="Skip thermal test")
    parser.add_argument("--thermal-duration", type=float, default=60.0, help="Thermal test duration (seconds)")
    args = parser.parse_args()

    odrv = None
    brake = None
    sensor = None

    try:
        # Load motor profile from JSON file
        profile = MotorProfile.from_json(args.profile)
        print(f"Profile: {profile.name}")

        # Load or create test results
        results = MotorTestResults(profile.name)

        # Connect to ODrive (assumes already configured and calibrated)
        print("Connecting to ODrive...")
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
        result = tester.run_full_test(profile.current_lim, args.skip_thermal, args.thermal_duration)

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
        print(f"\nResults saved to: {results_path}")
        print(f"\n{results}")

    except KeyboardInterrupt:
        print("\nInterrupted")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # Stop motor and release brake on exit
        if odrv:
            odrv.disable()
        if brake:
            brake.release()
        if sensor:
            sensor.disconnect()

    return 0


if __name__ == "__main__":
    exit(main())
