#!/usr/bin/env python3
"""
ODrive Motor Controller for MKS ODrive Mini V1.0
Compatible with odrive==0.5.1
"""

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Callable

import odrive
from odrive.enums import *
from odrive.utils import dump_errors, set_motor_thermistor_coeffs


class CalibrationError(Exception):
    pass


@dataclass
class MotorProfile:
    """Motor configuration profile loaded from JSON."""
    name: str
    description: str = ""
    motor_type: str = "high_current"  # "high_current" or "gimbal"
    pole_pairs: int = 7
    kv_rating: float = 100.0
    current_lim: float = 10.0
    calibration_current: float = 5.0
    phase_resistance: Optional[float] = None  # Required for gimbal
    phase_inductance: Optional[float] = None  # Required for gimbal
    
    @property
    def torque_constant(self) -> float:
        """Kt = 60 / (2 * pi * KV)"""
        return 60.0 / (2.0 * math.pi * self.kv_rating)
    
    @classmethod
    def from_json(cls, filepath: str) -> "MotorProfile":
        with open(filepath, 'r') as f:
            data = json.load(f)
        return cls(**data)


@dataclass
class MotorState:
    """Current state readings from the motor."""
    position: float = 0.0
    velocity: float = 0.0
    iq_measured: float = 0.0
    id_measured: float = 0.0
    iq_setpoint: float = 0.0
    bus_voltage: float = 0.0
    bus_current: float = 0.0
    fet_temperature: float = 0.0
    motor_temperature: Optional[float] = None
    torque_estimate: float = 0.0
    timestamp: float = field(default_factory=time.time)


class ODriveBoard:
    """ODrive motor controller interface."""
    
    def __init__(self, axis_num: int = 0, debug: bool = False):
        self._odrv = None
        self._axis = None
        self._axis_num = axis_num
        self._profile: Optional[MotorProfile] = None
        self._debug = debug
    
    def _log(self, msg: str) -> None:
        if self._debug:
            print(f"[ODrive] {msg}")
    
    @property
    def odrv(self):
        return self._odrv
    
    @property
    def axis(self):
        return self._axis
    
    # =========================================================================
    # Connection
    # =========================================================================
    
    def connect(self, serial_number: Optional[str] = None, timeout: float = 30.0) -> None:
        self._log(f"Searching for ODrive...")
        self._odrv = odrive.find_any(serial_number=serial_number, timeout=timeout)
        self._axis = getattr(self._odrv, f"axis{self._axis_num}")
        self._log(f"Connected to {hex(self._odrv.serial_number)}")
    
    def disconnect(self) -> None:
        if self._odrv:
            self._axis.requested_state = AXIS_STATE_IDLE
        self._odrv = None
        self._axis = None
    
    # =========================================================================
    # Configuration
    # =========================================================================
    
    def load_profile(self, profile: MotorProfile) -> None:
        self._profile = profile
        self._log(f"Loading profile: {profile.name}")
        
        # Power config
        self._odrv.config.brake_resistance = 2.0
        self._odrv.config.dc_bus_undervoltage_trip_level = 8.0
        self._odrv.config.dc_bus_overvoltage_trip_level = 56.0
        self._odrv.config.dc_max_negative_current = -5.0
        self._odrv.config.dc_max_positive_current = profile.current_lim
        self._odrv.config.max_regen_current = 0
        
        # Motor config
        motor_type = MOTOR_TYPE_GIMBAL if profile.motor_type == "gimbal" else MOTOR_TYPE_HIGH_CURRENT
        self._axis.motor.config.motor_type = motor_type
        self._axis.motor.config.pole_pairs = profile.pole_pairs
        self._axis.motor.config.current_lim = profile.current_lim
        self._axis.motor.config.calibration_current = profile.calibration_current
        self._axis.motor.config.requested_current_range = profile.current_lim * 1.25
        
        # Phase impedance (required for gimbal, optional for high_current)
        if profile.phase_resistance and profile.phase_inductance:
            self._axis.motor.config.phase_resistance = profile.phase_resistance
            self._axis.motor.config.phase_inductance = profile.phase_inductance
            # pre_calibrated=True makes motor calibration skip measurement (uses stored values)
            self._axis.motor.config.pre_calibrated = True
            self._log(f"  Phase: R={profile.phase_resistance}Ω, L={profile.phase_inductance*1e6:.0f}µH")
        
        # AS5047P encoder (GPIO 7)
        self._axis.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
        self._axis.encoder.config.cpr = 2**14
        self._axis.encoder.config.abs_spi_cs_gpio_pin = 7
        self._axis.encoder.config.bandwidth = 3000
        self._axis.encoder.config.calib_range = 10
        
        # Motor thermistor (GPIO 3)
        self._axis.motor_thermistor.config.gpio_pin = 3
        self._axis.motor_thermistor.config.enabled = True
        set_motor_thermistor_coeffs(self._axis, Rload=3300, R_25=10000, Beta=3435, Tmin=-10, TMax=150)
        
        self._log(f"  Type: {profile.motor_type}, Poles: {profile.pole_pairs}")
    
    def save_configuration(self) -> None:
        self._log("Saving configuration...")
        try:
            self._odrv.save_configuration()
        except Exception:
            pass  # Connection lost during reboot
        self._odrv = None
        self._axis = None
    
    def reboot(self) -> None:
        self._log("Rebooting...")
        try:
            self._odrv.reboot()
        except Exception:
            pass  # Connection lost during reboot
        self._odrv = None
        self._axis = None
    
    def erase_configuration(self) -> None:
        self._log("Erasing configuration...")
        try:
            self._odrv.erase_configuration()
        except Exception:
            pass  # Connection lost during reboot
        self._odrv = None
        self._axis = None
    
    def clear_errors(self) -> None:
        self._axis.error = 0
        self._axis.motor.error = 0
        self._axis.encoder.error = 0
        self._axis.controller.error = 0
    
    def get_errors(self) -> dict:
        return {
            "axis": self._axis.error,
            "motor": self._axis.motor.error,
            "encoder": self._axis.encoder.error,
            "controller": self._axis.controller.error,
        }
    
    def print_errors(self) -> None:
        dump_errors(self._odrv)
    
    # =========================================================================
    # Calibration
    # =========================================================================
    
    def calibrate(self, timeout: float = 30.0) -> None:
        self.clear_errors()
        
        # Motor calibration
        self._log("Motor calibration...")
        self._run_state(AXIS_STATE_MOTOR_CALIBRATION, timeout)
        if self._axis.motor.error:
            self.print_errors()
            raise CalibrationError("Motor calibration failed")
        
        self.clear_errors()
        time.sleep(0.5)
        
        # Encoder offset calibration
        self._log("Encoder calibration...")
        self._run_state(AXIS_STATE_ENCODER_OFFSET_CALIBRATION, timeout)
        if self._axis.encoder.error:
            self.print_errors()
            raise CalibrationError("Encoder calibration failed")
        
        self._log("Calibration complete")
    
    def _run_state(self, state: int, timeout: float) -> None:
        self._axis.requested_state = state
        start = time.time()
        while self._axis.current_state != AXIS_STATE_IDLE:
            if time.time() - start > timeout:
                raise CalibrationError(f"Timeout")
            time.sleep(0.1)
    
    # =========================================================================
    # Control
    # =========================================================================
    
    def enable(self, timeout: float = 2.0) -> None:
        self.clear_errors()
        self._axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
        # Wait for axis to enter closed-loop control
        start = time.time()
        while self._axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            if time.time() - start > timeout:
                self._log(f"Warning: Timeout waiting for closed-loop control (state={self._axis.current_state})")
                break
            if self._axis.error:
                self._log(f"Warning: Axis error during enable: {self._axis.error}")
                break
            time.sleep(0.01)
        
        self._log(f"Axis state: {self._axis.current_state}")
    
    def disable(self) -> None:
        self._axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.1)  # Brief wait for state transition
    
    def set_control_mode(self, mode: str) -> None:
        modes = {
            'torque': CONTROL_MODE_TORQUE_CONTROL,
            'velocity': CONTROL_MODE_VELOCITY_CONTROL,
            'position': CONTROL_MODE_POSITION_CONTROL,
        }
        self._axis.controller.config.control_mode = modes[mode.lower()]
        self._axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    
    def set_torque(self, torque_nm: float) -> None:
        self._axis.controller.input_torque = torque_nm
    
    def set_current(self, current_a: float) -> None:
        torque = current_a * self._profile.torque_constant if self._profile else current_a
        self._axis.controller.input_torque = torque
    
    def set_velocity(self, velocity: float) -> None:
        self._axis.controller.input_vel = velocity
    
    def set_position(self, position: float) -> None:
        self._axis.controller.input_pos = position
    
    # =========================================================================
    # Reading
    # =========================================================================
    
    def read_state(self) -> MotorState:
        kt = self._profile.torque_constant if self._profile else 1.0
        iq = self._axis.motor.current_control.Iq_measured

        return MotorState(
            position=self._axis.encoder.pos_estimate,
            velocity=self._axis.encoder.vel_estimate,
            iq_measured=iq,
            id_measured=self._axis.motor.current_control.Id_measured,
            iq_setpoint=self._axis.motor.current_control.Iq_setpoint,
            bus_voltage=self._odrv.vbus_voltage,
            bus_current=self._odrv.ibus,
            fet_temperature=self._axis.fet_thermistor.temperature,
            motor_temperature=self._axis.motor_thermistor.temperature,
            torque_estimate=iq * kt,
        )
    
    def read_position(self) -> float:
        return self._axis.encoder.pos_estimate
    
    def read_velocity(self) -> float:
        return self._axis.encoder.vel_estimate
    
    def read_current(self) -> float:
        return self._axis.motor.current_control.Iq_measured
    
    def read_bus_voltage(self) -> float:
        return self._odrv.vbus_voltage
    
    # =========================================================================
    # Streaming
    # =========================================================================
    
    def stream(self, callback: Callable[[MotorState], None], interval: float = 0.01, 
               duration: Optional[float] = None) -> None:
        start = time.time()
        while True:
            callback(self.read_state())
            if duration and (time.time() - start) >= duration:
                break
            time.sleep(interval)


# =============================================================================
# CLI
# =============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="ODrive Motor Controller")
    parser.add_argument("-c", "--connect", action="store_true", help="Connect and show status")
    parser.add_argument("-p", "--profile", type=str, help="Motor profile JSON")
    parser.add_argument("--erase", action="store_true", help="Erase config before loading profile")
    parser.add_argument("--calibrate", action="store_true", help="Run calibration")
    parser.add_argument("-m", "--monitor", type=float, nargs="?", const=5.0, help="Monitor N seconds")
    parser.add_argument("--serial", type=str, help="ODrive serial number")
    parser.add_argument("--debug", action="store_true", help="Debug output")
    
    args = parser.parse_args()
    
    if not (args.connect or args.profile or args.calibrate or args.monitor or args.erase):
        parser.print_help()
        return 0
    
    odrv = ODriveBoard(debug=args.debug)
    odrv.connect(serial_number=args.serial)
    print(f"Connected (Bus: {odrv.read_bus_voltage():.2f}V)")
    
    if args.erase:
        print("Erasing configuration...")
        odrv.erase_configuration()
        time.sleep(2)
        odrv.connect(serial_number=args.serial)
        print(f"Reconnected (Bus: {odrv.read_bus_voltage():.2f}V)")
    
    if args.profile:
        profile = MotorProfile.from_json(args.profile)
        odrv.load_profile(profile)
        print(f"Loaded: {profile.name}")
        odrv.save_configuration()
        time.sleep(2)
        odrv.connect(serial_number=args.serial)
        # Explicit reboot after save to ensure clean state
        odrv.reboot()
        time.sleep(2)
        odrv.connect(serial_number=args.serial)
        print(f"Reconnected (Bus: {odrv.read_bus_voltage():.2f}V)")
    
    if args.calibrate:
        print("Calibrating...")
        odrv.calibrate()
        print("Done!")
    
    if args.monitor:
        print(f"Monitoring {args.monitor:.1f}s...")
        def show(s): 
            print(f"\rPos:{s.position:8.3f} Vel:{s.velocity:7.2f} Iq:{s.iq_measured:6.2f}A", end="", flush=True)
        odrv.stream(show, interval=0.1, duration=args.monitor)
        print()
    
    errors = odrv.get_errors()
    if any(errors.values()):
        print("Errors:")
        odrv.print_errors()
    
    odrv.disconnect()
    return 0


if __name__ == "__main__":
    exit(main())
