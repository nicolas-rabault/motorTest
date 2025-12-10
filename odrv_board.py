#!/usr/bin/env python3
"""
ODrive Motor Controller for MKS ODrive Mini V1.0

This module provides a class to control BLDC motors through an ODrive
motor controller for test bench applications.

Compatible with odrive==0.5.1
"""

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional, Dict, Any, Callable

import odrive
from odrive.enums import *  # 0.5.x uses flat enum constants
from odrive.utils import dump_errors


class MotorType(str, Enum):
    """Motor type enumeration matching ODrive motor types."""
    HIGH_CURRENT = "high_current"  # Standard BLDC motors
    GIMBAL = "gimbal"              # High-resistance gimbal motors
    
    def to_odrive(self) -> int:
        """Convert to ODrive motor type constant."""
        mapping = {
            MotorType.HIGH_CURRENT: MOTOR_TYPE_HIGH_CURRENT,
            MotorType.GIMBAL: MOTOR_TYPE_GIMBAL,
        }
        return mapping[self]


class ODriveError(Exception):
    """Base exception for ODrive errors."""
    pass


class CalibrationError(ODriveError):
    """Raised when motor calibration fails."""
    pass


class ODriveConnectionError(ODriveError):
    """Raised when ODrive connection fails."""
    pass


@dataclass
class MotorProfile:
    """
    Motor configuration profile loaded from JSON.
    
    Motor types:
        - "high_current": Standard BLDC motors (most common)
        - "gimbal": High-resistance gimbal motors (low current, no current sensing)
    """
    name: str
    description: str = ""
    motor_type: str = "high_current"
    pole_pairs: int = 7
    kv_rating: float = 100.0
    current_lim: float = 10.0
    calibration_current: float = 5.0
    
    @property
    def torque_constant(self) -> float:
        """Calculate torque constant (Kt) from KV rating: Kt = 60 / (2 * pi * KV)"""
        return 60.0 / (2.0 * math.pi * self.kv_rating)
    
    def get_motor_type_enum(self) -> MotorType:
        """Get motor type as enum."""
        return MotorType(self.motor_type)
    
    @classmethod
    def from_json(cls, filepath: str) -> "MotorProfile":
        """Load motor profile from JSON file."""
        path = Path(filepath)
        if not path.exists():
            raise FileNotFoundError(f"Motor profile not found: {filepath}")
        
        with open(path, 'r') as f:
            data = json.load(f)
        
        # Validate motor_type if present
        if "motor_type" in data:
            valid_types = [t.value for t in MotorType]
            if data["motor_type"] not in valid_types:
                raise ValueError(f"Invalid motor_type: '{data['motor_type']}'. Must be one of: {valid_types}")
        
        return cls(**data)
    
    def to_json(self, filepath: str) -> None:
        """Save motor profile to JSON file."""
        data = {
            "name": self.name,
            "description": self.description,
            "motor_type": self.motor_type,
            "pole_pairs": self.pole_pairs,
            "kv_rating": self.kv_rating,
            "current_lim": self.current_lim,
            "calibration_current": self.calibration_current,
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)


@dataclass
class MotorState:
    """Current state readings from the motor/ODrive."""
    # Position and velocity
    position: float = 0.0  # turns
    velocity: float = 0.0  # turns/s
    
    # Electrical measurements
    iq_measured: float = 0.0  # Amps (torque-producing current)
    id_measured: float = 0.0  # Amps (field current)
    iq_setpoint: float = 0.0  # Amps
    bus_voltage: float = 0.0  # Volts
    bus_current: float = 0.0  # Amps
    
    # Temperatures
    fet_temperature: float = 0.0  # °C
    motor_temperature: Optional[float] = None  # °C (if thermistor connected)
    
    # Torque estimation
    torque_estimate: float = 0.0  # Nm
    
    # Timestamps
    timestamp: float = field(default_factory=time.time)
    
    @property
    def power_electrical(self) -> float:
        """Electrical power in Watts."""
        return self.bus_voltage * self.bus_current
    
    @property
    def power_mechanical(self) -> float:
        """Mechanical power estimate in Watts."""
        return self.torque_estimate * self.velocity * 2 * 3.14159


@dataclass
class MotorImpedance:
    """Motor electrical impedance measurements."""
    phase_resistance: float  # Ohms
    phase_inductance: float  # Henrys
    
    @property
    def phase_resistance_mohm(self) -> float:
        """Phase resistance in milliohms."""
        return self.phase_resistance * 1000
    
    @property
    def phase_inductance_uh(self) -> float:
        """Phase inductance in microhenrys."""
        return self.phase_inductance * 1e6


class ODriveBoard:
    """
    ODrive motor controller interface for BLDC motor testing.
    
    Compatible with odrive==0.5.1 (MKS ODrive Mini V1.0)
    
    This class provides methods to:
    - Connect to an ODrive board
    - Configure motor parameters from a profile
    - Run calibration sequences
    - Control motor in torque/current mode
    - Read position, velocity, current, and temperature
    - Measure motor impedance
    
    Usage:
        profile = MotorProfile.from_json("motor_profile.json")
        odrv = ODriveBoard()
        odrv.connect()
        odrv.load_profile(profile)
        odrv.calibrate()
        
        # Torque control
        odrv.enable()
        odrv.set_current(1.0)  # 1.0 A
        
        # Read state
        state = odrv.read_state()
        print(f"Position: {state.position}, Velocity: {state.velocity}")
        
        odrv.disable()
        odrv.disconnect()
    """
    
    def __init__(self, axis_num: int = 0, debug: bool = False):
        """
        Initialize ODrive controller.
        
        Args:
            axis_num: Axis number (0 or 1 for dual-axis ODrives)
            debug: Enable debug output
        """
        self._odrv = None
        self._axis_num = axis_num
        self._axis = None
        self._profile: Optional[MotorProfile] = None
        self._debug = debug
        self._connected = False
    
    def _log(self, msg: str) -> None:
        """Print debug message if debug mode enabled."""
        if self._debug:
            print(f"[ODrive] {msg}")
    
    # =========================================================================
    # Connection Management
    # =========================================================================
    
    def connect(
        self,
        serial_number: Optional[str] = None,
        timeout: float = 30.0
    ) -> None:
        """
        Connect to ODrive board.
        
        Args:
            serial_number: Specific ODrive serial number (None for any)
            timeout: Connection timeout in seconds
        """
        self._log(f"Searching for ODrive (timeout={timeout}s)...")
        
        try:
            # odrive 0.5.x uses find_any()
            self._odrv = odrive.find_any(
                serial_number=serial_number,
                timeout=timeout
            )
        except Exception as e:
            raise ODriveConnectionError(f"Failed to connect to ODrive: {e}")
        
        if self._odrv is None:
            raise ODriveConnectionError("No ODrive found")
        
        # Get the axis
        self._axis = getattr(self._odrv, f"axis{self._axis_num}")
        self._connected = True
        
        self._log(f"Connected to ODrive")
        self._log(f"  Serial: {hex(self._odrv.serial_number)}")
        self._log(f"  HW Version: {self._odrv.hw_version_major}.{self._odrv.hw_version_minor}")
        self._log(f"  FW Version: {self._odrv.fw_version_major}.{self._odrv.fw_version_minor}.{self._odrv.fw_version_revision}")
    
    def disconnect(self) -> None:
        """Disconnect from ODrive."""
        if self._connected:
            try:
                self.disable()
            except Exception:
                pass
        self._odrv = None
        self._axis = None
        self._connected = False
        self._log("Disconnected")
    
    def is_connected(self) -> bool:
        """Check if connected to ODrive."""
        return self._connected and self._odrv is not None
    
    @property
    def odrv(self):
        """Get raw ODrive object for advanced access."""
        return self._odrv
    
    @property
    def axis(self):
        """Get raw axis object for advanced access."""
        return self._axis
    
    # =========================================================================
    # Configuration
    # =========================================================================
    
    def load_profile(self, profile: MotorProfile) -> None:
        """Load motor profile and configure ODrive."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self._profile = profile
        self._log(f"Loading profile: {profile.name}")
        
        # Brake resistor (2 ohms, for 12V power supply without regen)
        self._odrv.config.brake_resistance = 2.0
        self._odrv.config.dc_bus_undervoltage_trip_level = 8.0
        self._odrv.config.dc_bus_overvoltage_trip_level = 56.0
        self._odrv.config.dc_max_negative_current = -5.0
        self._odrv.config.dc_max_positive_current = profile.current_lim
        self._odrv.config.max_regen_current = 0
        
        # Motor configuration
        self._axis.motor.config.motor_type = profile.get_motor_type_enum().to_odrive()
        self._axis.motor.config.pole_pairs = profile.pole_pairs
        self._axis.motor.config.current_lim = profile.current_lim
        self._axis.motor.config.calibration_current = profile.calibration_current
        self._axis.motor.config.requested_current_range = profile.current_lim * 1.25
        
        # AS5047P encoder (onboard, MKS ODrive Mini uses GPIO 4)
        self._axis.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
        self._axis.encoder.config.cpr = 2**14
        self._axis.encoder.config.abs_spi_cs_gpio_pin = 4
        self._axis.encoder.config.bandwidth = 3000
        self._axis.encoder.config.calib_range = 10
        self._axis.encoder.config.abs_spi_cs_gpio_pin = 7
        
        self._log(f"  Motor type: {profile.motor_type}")
        self._log(f"  Pole pairs: {profile.pole_pairs}")
        self._log(f"  Current limit: {profile.current_lim} A")
        self._log("Profile loaded")
    
    def save_configuration(self) -> None:
        """Save current configuration to ODrive non-volatile memory. ODrive will reboot."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self._log("Saving configuration...")
        try:
            self._odrv.save_configuration()
        except Exception:
            pass  # Connection lost during reboot is expected
        self._connected = False
        self._odrv = None
        self._axis = None
        self._log("Configuration saved, ODrive rebooting")
    
    def reboot(self) -> None:
        """Reboot the ODrive. Connection will be lost."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self._log("Rebooting ODrive...")
        try:
            self._odrv.reboot()
        except Exception:
            pass  # Connection lost is expected
        self._connected = False
        self._odrv = None
        self._axis = None
    
    def clear_errors(self) -> None:
        """Clear all errors on the ODrive."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        # Clear errors by setting them to 0
        self._axis.error = 0
        self._axis.motor.error = 0
        self._axis.encoder.error = 0
        self._axis.controller.error = 0
        try:
            self._axis.motor.fet_thermistor.error = 0
        except Exception:
            pass
        try:
            self._axis.motor.motor_thermistor.error = 0
        except Exception:
            pass
        self._log("Errors cleared")
    
    def get_errors(self) -> Dict[str, Any]:
        """
        Get current error status.
        
        Returns:
            Dictionary with error information
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        return {
            "axis": self._axis.error,
            "motor": self._axis.motor.error,
            "encoder": self._axis.encoder.error,
            "controller": self._axis.controller.error,
        }
    
    def print_errors(self) -> None:
        """Print error summary for the current axis."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        dump_errors(self._odrv)
    
    # =========================================================================
    # Calibration
    # =========================================================================
    
    def calibrate(
        self,
        full_calibration: bool = True,
        wait_timeout: float = 30.0
    ) -> None:
        """
        Run motor and encoder calibration.
        
        Args:
            full_calibration: Run full calibration (motor + encoder)
            wait_timeout: Maximum time to wait for calibration
        
        Raises:
            CalibrationError: If calibration fails
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self.clear_errors()
        
        if full_calibration:
            self._log("Starting full calibration sequence...")
            self._run_state(AXIS_STATE_FULL_CALIBRATION_SEQUENCE, wait_timeout)
        else:
            # Motor calibration only
            self._log("Starting motor calibration...")
            self._run_state(AXIS_STATE_MOTOR_CALIBRATION, wait_timeout)
        
        # Check for errors
        errors = self.get_errors()
        if any(v != 0 for v in errors.values()):
            self.print_errors()
            raise CalibrationError("Calibration failed with errors")
        
        self._log("Calibration complete")
    
    def calibrate_motor(self, wait_timeout: float = 15.0) -> None:
        """Run motor calibration only (measures resistance and inductance)."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self.clear_errors()
        self._log("Starting motor calibration...")
        self._run_state(AXIS_STATE_MOTOR_CALIBRATION, wait_timeout)
        
        # Check for errors
        if self._axis.motor.error != 0:
            self.print_errors()
            raise CalibrationError("Motor calibration failed")
        
        self._log("Motor calibration complete")
    
    def calibrate_encoder(self, wait_timeout: float = 30.0) -> None:
        """Run encoder calibration only."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self.clear_errors()
        self._log("Starting encoder calibration...")
        self._run_state(AXIS_STATE_ENCODER_OFFSET_CALIBRATION, wait_timeout)
        
        # Check for errors
        if self._axis.encoder.error != 0:
            self.print_errors()
            raise CalibrationError("Encoder calibration failed")
        
        self._log("Encoder calibration complete")
    
    def _run_state(self, state: int, timeout: float) -> None:
        """Run axis state and wait for completion."""
        self._axis.requested_state = state
        
        start_time = time.time()
        while self._axis.current_state != AXIS_STATE_IDLE:
            if time.time() - start_time > timeout:
                raise CalibrationError(f"Timeout waiting for state {state}")
            time.sleep(0.1)
    
    # =========================================================================
    # Motor Control
    # =========================================================================
    
    def enable(self) -> None:
        """Enable closed-loop control."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self.clear_errors()
        self._axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self._log("Closed-loop control enabled")
    
    def disable(self) -> None:
        """Disable motor (set to IDLE state)."""
        if not self._connected:
            return
        
        self._axis.requested_state = AXIS_STATE_IDLE
        self._log("Motor disabled (IDLE)")
    
    def set_control_mode(self, mode: str) -> None:
        """
        Set control mode.
        
        Args:
            mode: 'torque', 'velocity', or 'position'
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        # Control modes in 0.5.x
        modes = {
            'torque': CONTROL_MODE_TORQUE_CONTROL,
            'velocity': CONTROL_MODE_VELOCITY_CONTROL,
            'position': CONTROL_MODE_POSITION_CONTROL,
        }
        
        if mode.lower() not in modes:
            raise ValueError(f"Invalid mode: {mode}. Use 'torque', 'velocity', or 'position'")
        
        self._axis.controller.config.control_mode = modes[mode.lower()]
        self._axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self._log(f"Control mode set to: {mode}")
    
    def set_torque(self, torque_nm: float) -> None:
        """
        Set target torque (torque control mode).
        
        Args:
            torque_nm: Target torque in Nm
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        # In 0.5.x, use input_torque
        self._axis.controller.input_torque = torque_nm
    
    def set_current(self, current_a: float) -> None:
        """
        Set target current (Iq) directly.
        
        Args:
            current_a: Target q-axis current in Amps
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        # Convert current to torque using profile's torque constant
        if self._profile:
            torque = current_a * self._profile.torque_constant
            self._axis.controller.input_torque = torque
        else:
            # No profile loaded - assume Kt=1
            self._axis.controller.input_torque = current_a
    
    def set_velocity(self, velocity_turns_s: float) -> None:
        """
        Set target velocity (velocity control mode).
        
        Args:
            velocity_turns_s: Target velocity in turns/s
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self._axis.controller.input_vel = velocity_turns_s
    
    def set_position(self, position_turns: float) -> None:
        """
        Set target position (position control mode).
        
        Args:
            position_turns: Target position in turns
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        self._axis.controller.input_pos = position_turns
    
    # =========================================================================
    # State Reading
    # =========================================================================
    
    def read_state(self) -> MotorState:
        """
        Read current motor state.
        
        Returns:
            MotorState with all measurements
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        # Read values with fallbacks for properties that might not exist
        iq_measured = 0.0
        id_measured = 0.0
        iq_setpoint = 0.0
        bus_current = 0.0
        fet_temp = 0.0
        motor_temp = None
        torque_est = 0.0
        
        try:
            iq_measured = self._axis.motor.current_control.Iq_measured
        except Exception:
            pass
        
        try:
            id_measured = self._axis.motor.current_control.Id_measured
        except Exception:
            pass
        
        try:
            iq_setpoint = self._axis.motor.current_control.Iq_setpoint
        except Exception:
            pass
        
        try:
            bus_current = self._odrv.ibus
        except Exception:
            pass
        
        try:
            fet_temp = self._axis.motor.fet_thermistor.temperature
        except Exception:
            try:
                # Some versions use different path
                fet_temp = self._odrv.axis0.fet_thermistor.temperature
            except Exception:
                pass
        
        try:
            motor_temp = self._axis.motor.motor_thermistor.temperature
        except Exception:
            pass
        
        # Calculate torque from current and Kt
        if self._profile:
            torque_est = iq_measured * self._profile.torque_constant
        else:
            torque_est = iq_measured
        
        return MotorState(
            position=self._axis.encoder.pos_estimate,
            velocity=self._axis.encoder.vel_estimate,
            iq_measured=iq_measured,
            id_measured=id_measured,
            iq_setpoint=iq_setpoint,
            bus_voltage=self._odrv.vbus_voltage,
            bus_current=bus_current,
            fet_temperature=fet_temp,
            motor_temperature=motor_temp,
            torque_estimate=torque_est,
            timestamp=time.time(),
        )
    
    def read_position(self) -> float:
        """Read encoder position in turns."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        return self._axis.encoder.pos_estimate
    
    def read_velocity(self) -> float:
        """Read velocity in turns/s."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        return self._axis.encoder.vel_estimate
    
    def read_current(self) -> float:
        """Read measured Iq current in Amps."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        return self._axis.motor.current_control.Iq_measured
    
    def read_torque(self) -> float:
        """Read estimated torque in Nm (calculated from Iq * Kt)."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        iq = self._axis.motor.current_control.Iq_measured
        if self._profile:
            return iq * self._profile.torque_constant
        return iq
    
    def read_temperatures(self) -> Dict[str, Optional[float]]:
        """
        Read all temperature sensors.
        
        Returns:
            Dictionary with temperature readings
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        temps: Dict[str, Optional[float]] = {"fet": None, "motor": None}
        
        try:
            temps["fet"] = self._axis.motor.fet_thermistor.temperature
        except Exception:
            pass
        
        try:
            temps["motor"] = self._axis.motor.motor_thermistor.temperature
        except Exception:
            pass
        
        return temps
    
    def read_bus_voltage(self) -> float:
        """Read DC bus voltage."""
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        return self._odrv.vbus_voltage
    
    # =========================================================================
    # Motor Impedance Measurement
    # =========================================================================
    
    def measure_impedance(self) -> MotorImpedance:
        """
        Measure motor electrical impedance (resistance and inductance).
        
        This runs a motor calibration to measure the parameters.
        
        Returns:
            MotorImpedance with measured values
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        # Run motor calibration to measure impedance
        self.calibrate_motor()
        
        return MotorImpedance(
            phase_resistance=self._axis.motor.config.phase_resistance,
            phase_inductance=self._axis.motor.config.phase_inductance,
        )
    
    def get_impedance(self) -> MotorImpedance:
        """
        Get currently configured motor impedance (without re-measuring).
        
        Returns:
            MotorImpedance with configured values
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        return MotorImpedance(
            phase_resistance=self._axis.motor.config.phase_resistance,
            phase_inductance=self._axis.motor.config.phase_inductance,
        )
    
    
    # =========================================================================
    # Streaming / Continuous Data
    # =========================================================================
    
    def stream(
        self,
        callback: Callable[[MotorState], None],
        interval: float = 0.01,
        duration: Optional[float] = None
    ) -> None:
        """
        Stream motor state continuously.
        
        Args:
            callback: Function called with each MotorState reading
            interval: Time between readings in seconds
            duration: Total duration in seconds (None for infinite)
        """
        if not self._connected:
            raise ODriveConnectionError("Not connected to ODrive")
        
        start_time = time.time()
        try:
            while True:
                state = self.read_state()
                callback(state)
                
                if duration and (time.time() - start_time) >= duration:
                    break
                
                time.sleep(interval)
        except KeyboardInterrupt:
            pass
    
    # =========================================================================
    # Context Manager
    # =========================================================================
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
        return False


# =============================================================================
# Utility Functions
# =============================================================================

def create_profile_template(filepath: str) -> None:
    """Create a template motor profile JSON file."""
    profile = MotorProfile(
        name="Motor Template",
        description="Fill in motor specifications",
    )
    profile.to_json(filepath)
    print(f"Template profile created: {filepath}")


# =============================================================================
# Command-line Interface
# =============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description="ODrive Motor Controller (odrive 0.5.1)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --connect                    # Connect and show status
  %(prog)s --profile motor.json --calibrate
  %(prog)s --create-template motor_template.json
        """
    )
    parser.add_argument(
        "--connect", "-c",
        action="store_true",
        help="Connect to ODrive and show status"
    )
    parser.add_argument(
        "--serial",
        type=str,
        default=None,
        help="ODrive serial number"
    )
    parser.add_argument(
        "--profile", "-p",
        type=str,
        help="Motor profile JSON file"
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Run full calibration"
    )
    parser.add_argument(
        "--measure-impedance",
        action="store_true",
        help="Measure motor impedance"
    )
    parser.add_argument(
        "--monitor", "-m",
        type=float,
        nargs="?",
        const=5.0,
        help="Monitor motor state for N seconds"
    )
    parser.add_argument(
        "--create-template",
        type=str,
        metavar="FILE",
        help="Create template motor profile JSON"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output"
    )
    
    args = parser.parse_args()
    
    # Create template if requested
    if args.create_template:
        create_profile_template(args.create_template)
        return 0
    
    # Connect to ODrive
    if args.connect or args.profile or args.calibrate or args.measure_impedance or args.monitor:
        try:
            odrv = ODriveBoard(debug=args.debug)
            odrv.connect(serial_number=args.serial)
            
            # Show basic info
            print(f"Connected to ODrive")
            print(f"  Bus voltage: {odrv.read_bus_voltage():.2f} V")
            
            # Load profile if specified
            if args.profile:
                profile = MotorProfile.from_json(args.profile)
                odrv.load_profile(profile)
                print(f"Loaded profile: {profile.name}")
                print("Saving configuration (ODrive will reboot)...")
                odrv.save_configuration()
                time.sleep(2)
                print("Reconnecting...")
                odrv.connect(serial_number=args.serial)
                print(f"Reconnected. Bus voltage: {odrv.read_bus_voltage():.2f} V")
            
            # Run calibration if requested
            if args.calibrate:
                print("Running calibration...")
                odrv.calibrate()
                print("Calibration complete!")
            
            # Measure impedance if requested
            if args.measure_impedance:
                print("Measuring motor impedance...")
                impedance = odrv.measure_impedance()
                print(f"  Phase resistance: {impedance.phase_resistance_mohm:.2f} mΩ")
                print(f"  Phase inductance: {impedance.phase_inductance_uh:.2f} µH")
            
            # Monitor if requested
            if args.monitor:
                print(f"\nMonitoring for {args.monitor:.1f} seconds...")
                print("-" * 70)
                
                def print_state(state: MotorState):
                    print(f"\rPos: {state.position:8.3f} turns | "
                          f"Vel: {state.velocity:7.2f} t/s | "
                          f"Iq: {state.iq_measured:6.2f} A | "
                          f"V: {state.bus_voltage:5.1f}V", end="", flush=True)
                
                odrv.stream(print_state, interval=0.1, duration=args.monitor)
                print("\n" + "-" * 70)
            
            # Show errors if any
            errors = odrv.get_errors()
            if any(v != 0 for v in errors.values()):
                print("\nErrors detected:")
                odrv.print_errors()
            
            odrv.disconnect()
            
        except ODriveConnectionError as e:
            print(f"Connection error: {e}")
            return 1
        except CalibrationError as e:
            print(f"Calibration error: {e}")
            return 1
        except KeyboardInterrupt:
            print("\nInterrupted")
            return 0
    else:
        parser.print_help()
    
    return 0


if __name__ == "__main__":
    exit(main())

