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
from dataclasses import dataclass, asdict, field
from typing import List, Optional, Dict, Any

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

        self.torque_sensor.flush_old_data()
        zero_readings = []
        for _ in range(50):
            data = self.torque_sensor.read()
            if data:
                zero_readings.append(data.torque)
            time.sleep(0.02)

        if len(zero_readings) == 0:
            raise RuntimeError("Torque sensor not responding during zero calibration")

        zero_offset = sum(zero_readings) / len(zero_readings)
        print(f"  Zero offset: {zero_offset:.4f} Nm (will be compensated)")

        # Step 2: Enable brake at 100% intensity to fully lock motor
        print("  Engaging brake at 100%...")
        self.brake.set_intensity(100.0)
        self.brake.enable()
        time.sleep(1.0)
        
        # Ensure axis is disabled before setting control mode
        self.odrv.disable()
        time.sleep(0.1)
        
        # Set control mode to torque
        self.odrv.set_control_mode('torque')
        time.sleep(0.1)
        
        # Ensure input torque is zero before enabling
        self.odrv.axis.controller.input_torque = 0.0
        
        # Enable motor
        self.odrv.enable()
        time.sleep(0.2)  # Give motor time to initialize in closed-loop mode
        
        # Verify control mode and axis state
        state = self.odrv.read_state()
        print(f"  Axis state: {self.odrv.get_axis_state()}")
        print(f"  Control mode: {self.odrv.get_control_mode()}")
        print(f"  Input torque: {self.odrv.axis.controller.input_torque:.4f} Nm")
        print(f"  Iq setpoint: {state.iq_setpoint:.2f} A, Iq measured: {state.iq_measured:.2f} A")
        print(f"  Velocity: {state.velocity:.4f} turns/s")
        
        # Check for any axis errors
        if self.odrv.axis.error != 0:
            print(f"  WARNING: Axis error detected: {self.odrv.axis.error}")
            self.odrv.print_errors()

        kt_measurements = []

        # Test at each current level (typically 1A, 2A, 3A, 4A)
        for current in test_currents:
            # Apply test current
            expected_torque = current * self.profile.torque_constant
            self.odrv.set_current(current)

            # Wait for mechanical torque to stabilize
            time.sleep(1.5)

            # Flush old buffered sensor data accumulated during stabilization
            # Aggressive multi-stage flush ensures we get fresh data
            self.torque_sensor.flush_old_data()

            # Verify the input torque was set correctly
            actual_input_torque = self.odrv.axis.controller.input_torque
            if abs(actual_input_torque - expected_torque) > 0.001:
                print(f"  Warning: Input torque mismatch! Expected {expected_torque:.4f} Nm, got {actual_input_torque:.4f} Nm")

            # Collect torque and current measurements for 2 seconds
            torques, currents, velocities, positions = [], [], [], []
            timestamps = []
            raw_torques = []  # Track raw values for debugging
            start = time.time()
            while (time.time() - start) < 2.0:
                # Read fresh torque data (read() drains buffer automatically)
                sensor_data = self.torque_sensor.read()
                state = self.odrv.read_state()

                if sensor_data:
                    # Apply zero offset compensation
                    compensated_torque = sensor_data.torque - zero_offset
                    torques.append(compensated_torque)
                    timestamps.append(sensor_data.timestamp)
                    raw_torques.append(sensor_data.torque)

                currents.append(state.iq_measured)
                velocities.append(state.velocity)
                positions.append(state.position)
                time.sleep(0.01)

            # Check if we got any torque readings
            if len(torques) == 0:
                print(f"  Warning: No torque readings at {current}A (sensor not responding)")
                continue

            # Calculate KT = torque / current for this test point
            avg_torque = sum(torques) / len(torques)
            avg_current = sum(currents) / len(currents)
            avg_velocity = sum(velocities) / len(velocities)
            position_change = max(positions) - min(positions)

            # Calculate data freshness (how old is sensor data)
            now = time.time()
            data_ages = [(now - ts) * 1000 for ts in timestamps]  # ms
            avg_age = sum(data_ages) / len(data_ages)
            max_age = max(data_ages)

            # Show detailed debug for first measurement
            if current == test_currents[0]:
                print(f"  [DEBUG] Raw torques: min={min(raw_torques):.4f}, max={max(raw_torques):.4f}, avg={sum(raw_torques)/len(raw_torques):.4f}")
                print(f"  [DEBUG] Zero offset: {zero_offset:.4f} Nm")
                print(f"  [DEBUG] Data age: avg={avg_age:.1f}ms, max={max_age:.1f}ms")
                print(f"  [DEBUG] First 5 raw values: {[f'{t:.4f}' for t in raw_torques[:5]]}")
                print(f"  [DEBUG] First 5 compensated: {[f'{t:.4f}' for t in torques[:5]]}")

            print(f"  {current}A: torque={avg_torque:.4f} Nm, current={avg_current:.2f} A, "
                  f"vel={avg_velocity:.3f} turns/s, pos_delta={position_change:.4f} turns, samples={len(torques)}, age={avg_age:.0f}ms")

            # Verify motor is locked (not spinning)
            if abs(avg_velocity) > 0.05:
                print(f"  ⚠️  WARNING: Motor spinning at {abs(avg_velocity):.3f} turns/s - brake not holding!")
                print(f"     Measurement may be inaccurate (dynamic friction, not static torque)")
            elif position_change > 0.002:
                print(f"  ⚠️  WARNING: Motor creeping ({position_change:.4f} turns) - brake marginally holding")

            if abs(avg_current) > 0.1:
                kt_measurements.append(abs(avg_torque) / abs(avg_current))

        # Stop motor, disable, and release brake
        self.odrv.set_current(0)
        self.odrv.disable()
        self.brake.release()

        # Check if we got any valid KT measurements
        if len(kt_measurements) == 0:
            raise RuntimeError(
                "No valid KT measurements obtained - all test points failed. "
                "Check torque sensor connection and ensure it's receiving data."
            )

        # Return average KT and the zero offset
        avg_kt = sum(kt_measurements) / len(kt_measurements)
        return avg_kt, zero_offset
    
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

        # Aggressively flush old data before burst test
        self.torque_sensor.flush_old_data()
        time.sleep(0.01)  # Brief pause after flush

        # Apply maximum current burst and record data at ~300Hz for 100ms
        # (1kHz is too fast for both USB devices, 300Hz is reliable)
        data_points = []
        sample_interval = 0.003  # 3ms = ~333Hz
        start_time = time.time()
        self.odrv.set_current(max_current)

        while (time.time() - start_time) < 0.1:
            loop_start = time.time()
            elapsed_ms = (loop_start - start_time) * 1000.0

            # Read torque sensor and ODrive state
            sensor_data = self.torque_sensor.read()
            state = self.odrv.read_state()

            # Apply zero offset compensation to torque reading
            compensated_torque = (sensor_data.torque - zero_offset) if sensor_data else 0.0

            # Record all data for this sample point
            data_points.append(TorqueBurstDataPoint(
                timestamp_ms=elapsed_ms,
                measured_torque_Nm=compensated_torque,
                estimated_torque_Nm=state.torque_estimate,
                current_A=state.iq_measured,
                velocity_turns_s=state.velocity,
                position_turns=state.position,
                motor_temperature_C=state.motor_temperature,
                bus_voltage_V=state.bus_voltage,
                measured_speed_RPM=sensor_data.speed if sensor_data else 0
            ))

            # Sleep to maintain target sample rate
            elapsed = time.time() - loop_start
            if elapsed < sample_interval:
                time.sleep(sample_interval - elapsed)

        # Stop motor, disable, and release brake
        self.odrv.set_current(0)
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
        print(f"Max test current: {max_current}A")

        # Get spec KV from motor profile
        kv_spec = self.profile.kv_rating
        print(f"\nKV (spec): {kv_spec:.1f} RPM/V")

        # Test 1: Measure KT (torque constant)
        print("\n--- Test 1: KT Measurement ---")
        kt, zero_offset = self.measure_kt([2.0, 3.0, 4.0, min(5.0, max_current)])
        print(f"KT (measured): {kt:.4f} Nm/A")

        # Check if KT measurement is valid
        if kt < 0.0001:
            raise RuntimeError(
                "KT measurement failed - torque sensor reading zero!\n\n"
                "Possible issues:\n"
                "  1. Torque sensor not mechanically coupled to motor shaft\n"
                "  2. Brake not providing enough resistance\n"
                "  3. Motor not producing torque (check phase connections)\n"
                "  4. Torque sensor calibration/zero offset issue\n\n"
                "The motor is drawing current but no torque is being measured."
            )

        # Calculate KV from measured KT and compare to spec
        kv_from_kt = 60.0 / (2.0 * math.pi * kt)
        print(f"KV (from measured KT): {kv_from_kt:.1f} RPM/V")
        error_percent = abs(kv_from_kt - kv_spec) / kv_spec * 100
        print(f"Difference from spec: {error_percent:.1f}%")

        # Test 2: Thermal characterization
        thermal_resistance = 0.0
        thermal_time_constant = 0.0
        if not skip_thermal:
            print(f"\n--- Test 2: Thermal (~{thermal_duration}s) ---")
            thermal_data = self.measure_thermal_parameters(min(3.0, max_current), thermal_duration)
            thermal_resistance = thermal_data.thermal_resistance_C_per_W
            thermal_time_constant = thermal_data.time_constant_s
            print(f"Thermal resistance: {thermal_resistance:.2f} °C/W")
            print(f"Time constant: {thermal_time_constant:.1f}s")
        else:
            print("\n--- Test 2: Thermal (SKIPPED) ---")

        # Test 3: Max torque burst
        print("\n--- Test 3: Torque Burst (100ms) ---")
        burst_data = self.run_torque_burst_test(max_current, zero_offset)
        if burst_data:
            avg_measured_torque = sum(d.measured_torque_Nm for d in burst_data) / len(burst_data)
            avg_current = sum(d.current_A for d in burst_data) / len(burst_data)
            print(f"Measured torque: {avg_measured_torque:.4f} Nm")
            print(f"Average current: {avg_current:.2f} A")

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

    return 0


if __name__ == "__main__":
    exit(main())
