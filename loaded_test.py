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
import threading
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
    timestamp_s: float  # Absolute timestamp (seconds since epoch)
    time_relative_ms: float  # Time relative to burst start (ms)
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
    burst_duration_ms: float

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
        # Use fixed ambient temperature of 20°C
        ambient_temp = 20.0

        # Verify motor thermistor is available
        state = self.odrv.read_state()
        if state.motor_temperature is None:
            raise RuntimeError("Motor thermistor not available - cannot measure thermal parameters")

        print(f"  Ambient temperature: {ambient_temp}°C (fixed)")
        print(f"  Current motor temperature: {state.motor_temperature:.1f}°C")

        # Enable brake at 30% intensity and configure torque control mode
        self.brake.set_intensity(30.0)
        self.brake.enable()
        time.sleep(0.5)
        self.odrv.disable()
        time.sleep(0.1)
        self.odrv.set_control_mode('torque')
        time.sleep(0.1)
        self.odrv.enable()

        # HEATING PHASE: Apply constant current and record temperature rise over time
        print(f"  Heating phase: {heating_duration:.0f}s at {test_current:.1f}A...")
        heating_data = []
        self.odrv.set_current(test_current)
        phase_resistance = self.profile.phase_resistance or self.odrv.axis.motor.config.phase_resistance
        start_time = time.time()

        while (time.time() - start_time) < heating_duration:
            state = self.odrv.read_state()
            temp = state.motor_temperature
            if temp is None:
                raise RuntimeError("Motor thermistor reading lost during test")

            current = state.iq_measured
            power = (current ** 2) * phase_resistance * 1.5

            heating_data.append({
                'time': time.time() - start_time,
                'temperature': temp,
                'power': power
            })
            time.sleep(0.1)  # 10 Hz sampling rate

        # Stop motor
        self.odrv.set_current(0)
        self.odrv.disable()

        # Calculate thermal resistance from heating phase: R_th = ΔT / P
        final_temp = heating_data[-1]['temperature']
        temp_rise = final_temp - ambient_temp
        avg_power = sum(d['power'] for d in heating_data) / len(heating_data) if heating_data else 1.0
        thermal_resistance = temp_rise / avg_power if avg_power > 0.1 else 0.0

        print(f"  Heating complete: {final_temp:.1f}°C (ΔT={temp_rise:.1f}°C)")
        print(f"  Average power: {avg_power:.2f}W")

        # Calculate heating time constant: time to reach 63.2% of final temperature rise
        target_temp = ambient_temp + 0.632 * temp_rise
        heating_time_constant = heating_duration
        for data in heating_data:
            if data['temperature'] >= target_temp:
                heating_time_constant = data['time']
                break

        # COOLING PHASE: Record temperature decay back to ambient
        print(f"  Cooling phase: monitoring temperature decay...")
        cooling_data = []
        start_temp = final_temp
        start_time = time.time()

        # Continue until temperature drops significantly or timeout (3x heating time constant)
        max_cooling_time = max(300, 3 * heating_time_constant)  # At least 5 minutes

        while (time.time() - start_time) < max_cooling_time:
            state = self.odrv.read_state()
            temp = state.motor_temperature
            if temp is None:
                print("  ⚠️  Motor thermistor reading lost during cooling")
                break

            cooling_data.append({
                'time': time.time() - start_time,
                'temperature': temp
            })

            # Stop if we've cooled to within 10% of ambient
            if abs(temp - ambient_temp) < 0.1 * temp_rise:
                print(f"  Cooled to near-ambient: {temp:.1f}°C")
                break

            time.sleep(0.1)  # 10 Hz sampling rate

        # Release brake after cooling measurement
        self.brake.release()

        # Calculate cooling time constant: time to decay to 36.8% of initial temperature rise
        # T(t) = T_ambient + ΔT_initial × e^(-t/τ)
        # At t=τ, remaining rise = 0.368 × initial rise
        initial_temp_rise = start_temp - ambient_temp
        target_temp_cooling = ambient_temp + 0.368 * initial_temp_rise
        cooling_time_constant = max_cooling_time
        for data in cooling_data:
            if data['temperature'] <= target_temp_cooling:
                cooling_time_constant = data['time']
                break

        # Average the heating and cooling time constants
        time_constant = (heating_time_constant + cooling_time_constant) / 2.0

        print(f"  Heating τ: {heating_time_constant:.1f}s")
        print(f"  Cooling τ: {cooling_time_constant:.1f}s")
        print(f"  Average τ: {time_constant:.1f}s")

        return ThermalTestData(
            thermal_resistance_C_per_W=thermal_resistance,
            time_constant_s=time_constant
        )
    
    def run_torque_burst_test(
        self,
        max_current: float,
        zero_offset: float = 0.0,
        pre_burst_ms: float = 100.0,
        burst_duration_ms: float = 200.0,
        post_burst_ms: float = 20000.0
    ) -> List[TorqueBurstDataPoint]:
        """Run max torque burst test with pre/post capture for time alignment."""
        self.brake.set_intensity(100.0)
        self.brake.enable()
        time.sleep(0.5)

        self.odrv.disable()
        time.sleep(0.1)
        self.odrv.set_control_mode('torque')
        time.sleep(0.1)
        self.odrv.enable()
        time.sleep(0.2)

        # Collect ODrive data in parallel
        odrv_samples = []
        collecting = [True]

        def collect_odrv():
            while collecting[0]:
                odrv_samples.append((time.time(), self.odrv.read_state()))
                time.sleep(0.003)

        thread = threading.Thread(target=collect_odrv)
        thread.start()

        time.sleep(pre_burst_ms / 1000.0)

        print(f"  Setting current to {max_current}A...")
        self.odrv.set_current(max_current)

        # Check what's actually being commanded
        time.sleep(0.05)
        state = self.odrv.read_state()
        print(f"  Commanded torque: {self.odrv.axis.controller.input_torque:.4f} Nm")
        print(f"  Iq setpoint: {state.iq_setpoint:.2f}A, Iq measured: {state.iq_measured:.2f}A")

        time.sleep(burst_duration_ms / 1000.0 - 0.05)
        self.odrv.set_current(0)

        time.sleep(post_burst_ms / 1000.0)

        collecting[0] = False
        thread.join()

        # Get sensor data
        total_duration_s = (pre_burst_ms + burst_duration_ms + post_burst_ms) / 1000.0
        sensor_samples = self.torque_sensor.read_range(total_duration_s)

        # Align timestamps by matching burst start and end using derivatives
        sensor_times = np.array([s.timestamp for s in reversed(sensor_samples)])
        sensor_torques = np.array([abs(s.torque - zero_offset) for s in reversed(sensor_samples)])

        odrv_times = np.array([t for t, _ in odrv_samples])
        kt = self.results.data.get("electrical", {}).get("KT_Nm_per_A")
        odrv_torques = np.array([abs(s.iq_measured * kt) if kt else abs(s.torque_estimate)
                                for _, s in odrv_samples])

        # Compute derivatives (rate of change)
        sensor_dt = np.gradient(sensor_torques, sensor_times)
        odrv_dt = np.gradient(odrv_torques, odrv_times)

        # Find burst start (maximum derivative - rising edge)
        sensor_start_idx = np.argmax(sensor_dt)
        odrv_start_idx = np.argmax(odrv_dt)

        # Find burst end (minimum derivative - falling edge)
        sensor_end_idx = np.argmin(sensor_dt)
        odrv_end_idx = np.argmin(odrv_dt)

        # Calculate time scaling factor to match burst durations
        sensor_burst_duration = sensor_times[sensor_end_idx] - sensor_times[sensor_start_idx]
        odrv_burst_duration = odrv_times[odrv_end_idx] - odrv_times[odrv_start_idx]
        time_scale = odrv_burst_duration / sensor_burst_duration if sensor_burst_duration > 0 else 1.0

        # Align and scale sensor timestamps to match ODrive timing
        # This corrects for time offset and data rate differences
        aligned_sensor_times = (sensor_times - sensor_times[sensor_start_idx]) * time_scale + odrv_times[odrv_start_idx]

        # Merge data - each sensor sample gets ODrive data from closest time
        data_points = []
        for i, sample in enumerate(reversed(sensor_samples)):
            sensor_time_aligned = aligned_sensor_times[i]

            # Find closest ODrive sample
            closest_odrv = min(odrv_samples, key=lambda x: abs(x[0] - sensor_time_aligned))
            odrv_time, state = closest_odrv

            # Calculate relative time from ODrive timestamp (not sensor)
            relative_time_ms = (odrv_time - odrv_times[odrv_start_idx]) * 1000.0

            # Use KT * current for estimated torque
            estimated_torque = abs(state.iq_measured * kt) if kt else abs(state.torque_estimate)

            data_points.append(TorqueBurstDataPoint(
                timestamp_s=odrv_time,
                time_relative_ms=relative_time_ms,
                measured_torque_Nm=abs(sample.torque - zero_offset),
                estimated_torque_Nm=estimated_torque,
                current_A=abs(state.iq_measured),
                velocity_turns_s=state.velocity,
                position_turns=state.position,
                motor_temperature_C=state.motor_temperature,
                bus_voltage_V=state.bus_voltage,
                measured_speed_RPM=sample.speed
            ))

        # Apply 30-sample moving average LPF to temperature using pandas
        sorted_points = sorted(data_points, key=lambda d: d.time_relative_ms)

        # Extract temperature values
        import pandas as pd
        temps = [p.motor_temperature_C if p.motor_temperature_C is not None else np.nan
                 for p in sorted_points]

        # Apply rolling mean with proper edge handling
        temps_series = pd.Series(temps)
        temps_filtered = temps_series.rolling(window=500, min_periods=1, center=True).mean()

        # Update datapoints with filtered values
        for i, point in enumerate(sorted_points):
            if not np.isnan(temps_filtered.iloc[i]):
                point.motor_temperature_C = float(temps_filtered.iloc[i])

        self.odrv.disable()
        self.brake.release()

        return sorted_points
    
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
        burst_duration_ms = 200.0  # Define burst duration
        print(f"\n--- Torque Burst ({burst_duration_ms:.0f}ms) ---")
        burst_data = self.run_torque_burst_test(max_current, zero_offset, burst_duration_ms=burst_duration_ms)
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
            burst_duration_ms=burst_duration_ms,
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

        # Override current limits (temporary, not saved)
        odrv.set_current_limit(profile.current_lim)

        # Debug: Print all current-related config values
        print("\n=== Current Configuration Debug ===")
        print(f"dc_max_positive_current: {odrv.odrv.config.dc_max_positive_current}A")
        print(f"motor.current_lim: {odrv.axis.motor.config.current_lim}A")
        print(f"motor.requested_current_range: {odrv.axis.motor.config.requested_current_range}A")
        print(f"motor.calibration_current: {odrv.axis.motor.config.calibration_current}A")

        # Check if there are any thermal limits
        try:
            print(f"FET temp: {odrv.axis.fet_thermistor.temperature}°C")
            print(f"Motor temp: {odrv.axis.motor_thermistor.temperature}°C")
        except:
            pass
        print("===================================\n")

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
            duration_ms=result.burst_duration_ms,
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
