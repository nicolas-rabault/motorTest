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


RESULTS_DIR = Path.cwd() / "results"


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
                "image_url": "",
                "motor_type": "",
                "pole_pairs": 0,
                "kv_rating": 0,
                "weight_g": 0.0,
                "electrical": {},
                "mechanical": {},
                "thermal": {},
                "limits": {},
                "test_history": {
                    "no_load": [],
                    "loaded": []
                }
            }

    def update_motor_info(self, description: str = None, motor_type: str = None,
                         pole_pairs: int = None, kv_rating: float = None, image_url: str = None,
                         weight_g: float = None):
        """Update basic motor information."""
        if description:
            self.data["description"] = description
        if image_url:
            self.data["image_url"] = image_url
        if motor_type:
            self.data["motor_type"] = motor_type
        if pole_pairs:
            self.data["pole_pairs"] = pole_pairs
        if kv_rating is not None:
            self.data["kv_rating"] = kv_rating
        if weight_g is not None:
            self.data["weight_g"] = weight_g

    def update_electrical(self, phase_resistance_ohm: float = None,
                         phase_inductance_mH: float = None,
                         kt_nm_per_a: float = None):
        """Update electrical parameters."""
        if phase_resistance_ohm is not None:
            self.data["electrical"]["phase_resistance_ohm"] = phase_resistance_ohm
        if phase_inductance_mH is not None:
            self.data["electrical"]["phase_inductance_mH"] = phase_inductance_mH
        if kt_nm_per_a is not None:
            self.data["electrical"]["KT_Nm_per_A"] = kt_nm_per_a
            # Calculate KV from KT and add comparison to spec
            kv_from_kt = 60.0 / (2.0 * 3.14159265359 * kt_nm_per_a)
            self.data["electrical"]["KV_from_KT_rpm_per_V"] = kv_from_kt
            if self.data.get("kv_rating"):
                kv_spec = self.data["kv_rating"]
                error_percent = abs(kv_from_kt - kv_spec) / kv_spec * 100
                self.data["electrical"]["KV_error_percent"] = error_percent

    def update_mechanical(self, no_load_current_a: float = None,
                         motor_inertia_kg_m2: float = None):
        """Update mechanical parameters."""
        if no_load_current_a is not None:
            self.data["mechanical"]["no_load_current_A"] = no_load_current_a
        if motor_inertia_kg_m2 is not None:
            self.data["mechanical"]["motor_inertia_kg_m2"] = motor_inertia_kg_m2

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
            if "KT_Nm_per_A" in elec:
                lines.append(f"  KT: {elec['KT_Nm_per_A']:.4f} Nm/A")
            if "KV_from_KT_rpm_per_V" in elec:
                kv_from_kt = elec['KV_from_KT_rpm_per_V']
                lines.append(f"  KV (from KT): {kv_from_kt:.1f} RPM/V")
                if "KV_error_percent" in elec and self.data.get("kv_rating"):
                    kv_spec = self.data["kv_rating"]
                    error = elec['KV_error_percent']
                    lines.append(f"  KV (spec): {kv_spec:.1f} RPM/V (error: {error:.1f}%)")

        if "mechanical" in self.data and self.data["mechanical"]:
            lines.append("")
            lines.append("Mechanical:")
            mech = self.data["mechanical"]
            if "no_load_current_A" in mech:
                lines.append(f"  No-load current: {mech['no_load_current_A']:.4f} A")
            if "motor_inertia_kg_m2" in mech:
                lines.append(f"  Inertia: {mech['motor_inertia_kg_m2']:.6e} kg·m²")

        if "thermal" in self.data and self.data["thermal"]:
            lines.append("")
            lines.append("Thermal:")
            th = self.data["thermal"]
            if "thermal_resistance_C_per_W" in th:
                lines.append(f"  R_th: {th['thermal_resistance_C_per_W']:.2f} °C/W")
            if "thermal_time_constant_s" in th:
                lines.append(f"  τ: {th['thermal_time_constant_s']:.1f} s")

        return "\n".join(lines)
