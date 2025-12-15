#!/usr/bin/env python3
"""
Motor Test Results Manager

Handles reading and writing motor characterization test results.
Results are stored in a single JSON file per motor that gets updated
by both no-load and loaded tests.
"""

import json
from pathlib import Path
from typing import Optional, Dict, Any
from datetime import datetime


RESULTS_DIR = Path(__file__).parent / "results"


class MotorTestResults:
    """Manages motor test results stored in JSON format."""

    def __init__(self, motor_name: str):
        """Initialize results manager for a specific motor.

        Args:
            motor_name: Name of the motor (used for filename)
        """
        self.motor_name = motor_name
        self.results_path = RESULTS_DIR / f"{motor_name}_results.json"
        self.data = self._load_or_create()

    def _load_or_create(self) -> Dict[str, Any]:
        """Load existing results or create new structure."""
        if self.results_path.exists():
            with open(self.results_path, 'r') as f:
                return json.load(f)
        else:
            return {
                "name": self.motor_name,
                "description": "",
                "motor_type": "",
                "pole_pairs": 0,
                "electrical": {},
                "thermal": {},
                "limits": {},
                "test_history": {
                    "no_load": [],
                    "loaded": []
                }
            }

    def update_motor_info(self, description: str = None, motor_type: str = None,
                         pole_pairs: int = None):
        """Update basic motor information."""
        if description:
            self.data["description"] = description
        if motor_type:
            self.data["motor_type"] = motor_type
        if pole_pairs:
            self.data["pole_pairs"] = pole_pairs

    def update_electrical(self, phase_resistance_ohm: float = None,
                         phase_inductance_mH: float = None,
                         kv_rpm_per_v: float = None,
                         kt_nm_per_a: float = None):
        """Update electrical parameters."""
        if phase_resistance_ohm is not None:
            self.data["electrical"]["phase_resistance_ohm"] = phase_resistance_ohm
        if phase_inductance_mH is not None:
            self.data["electrical"]["phase_inductance_mH"] = phase_inductance_mH
        if kv_rpm_per_v is not None:
            self.data["electrical"]["KV_rpm_per_V"] = kv_rpm_per_v
        if kt_nm_per_a is not None:
            self.data["electrical"]["KT_Nm_per_A"] = kt_nm_per_a

    def update_thermal(self, thermal_resistance_c_per_w: float = None,
                      thermal_time_constant_s: float = None):
        """Update thermal parameters."""
        if thermal_resistance_c_per_w is not None:
            self.data["thermal"]["thermal_resistance_C_per_W"] = thermal_resistance_c_per_w
        if thermal_time_constant_s is not None:
            self.data["thermal"]["thermal_time_constant_s"] = thermal_time_constant_s

    def update_limits(self, max_current_a: float = None,
                     calibration_current_a: float = None):
        """Update motor limits."""
        if max_current_a is not None:
            self.data["limits"]["max_current_A"] = max_current_a
        if calibration_current_a is not None:
            self.data["limits"]["calibration_current_A"] = calibration_current_a

    def add_torque_burst_test(self, max_current_a: float, duration_ms: float,
                             data: list):
        """Add torque burst test data."""
        self.data["torque_burst_test"] = {
            "duration_ms": duration_ms,
            "max_current_A": max_current_a,
            "data": data
        }

    def add_test_metadata(self, test_type: str, bus_voltage_v: float):
        """Add metadata about the test run.

        Args:
            test_type: "no_load" or "loaded"
            bus_voltage_v: Bus voltage during test
        """
        timestamp = datetime.now().isoformat()

        test_record = {
            "timestamp": timestamp,
            "bus_voltage_V": bus_voltage_v
        }

        self.data["test_history"][test_type].append(test_record)
        self.data["last_updated"] = timestamp
        self.data["bus_voltage_V"] = bus_voltage_v

    def get_kv(self) -> Optional[float]:
        """Get KV value from results (measured in no-load test)."""
        return self.data.get("electrical", {}).get("KV_rpm_per_V")

    def get_kt(self) -> Optional[float]:
        """Get KT value from results (measured in loaded test)."""
        return self.data.get("electrical", {}).get("KT_Nm_per_A")

    def get_impedance(self) -> tuple[Optional[float], Optional[float]]:
        """Get phase resistance and inductance.

        Returns:
            (resistance_ohm, inductance_H)
        """
        elec = self.data.get("electrical", {})
        r_ohm = elec.get("phase_resistance_ohm")
        l_mh = elec.get("phase_inductance_mH")
        l_h = l_mh / 1000.0 if l_mh is not None else None
        return r_ohm, l_h

    def save(self) -> Path:
        """Save results to JSON file.

        Returns:
            Path to saved file
        """
        RESULTS_DIR.mkdir(exist_ok=True)
        with open(self.results_path, 'w') as f:
            json.dump(self.data, f, indent=2)
        return self.results_path

    def __str__(self) -> str:
        """String representation of results."""
        lines = [
            f"Motor: {self.motor_name}",
            f"Type: {self.data.get('motor_type', 'N/A')}",
            ""
        ]

        if "electrical" in self.data:
            lines.append("Electrical:")
            elec = self.data["electrical"]
            if "phase_resistance_ohm" in elec:
                lines.append(f"  R: {elec['phase_resistance_ohm']*1000:.2f} mΩ")
            if "phase_inductance_mH" in elec:
                lines.append(f"  L: {elec['phase_inductance_mH']*1000:.2f} µH")
            if "KV_rpm_per_V" in elec:
                lines.append(f"  KV: {elec['KV_rpm_per_V']:.1f} RPM/V")
            if "KT_Nm_per_A" in elec:
                lines.append(f"  KT: {elec['KT_Nm_per_A']:.4f} Nm/A")

        if "thermal" in self.data and self.data["thermal"]:
            lines.append("")
            lines.append("Thermal:")
            th = self.data["thermal"]
            if "thermal_resistance_C_per_W" in th:
                lines.append(f"  R_th: {th['thermal_resistance_C_per_W']:.2f} °C/W")
            if "thermal_time_constant_s" in th:
                lines.append(f"  τ: {th['thermal_time_constant_s']:.1f} s")

        return "\n".join(lines)
