"""Configuration management for motor-test package."""

import os
import json
from pathlib import Path
from typing import Optional, Dict, Any


class Config:
    """Configuration manager for motor test suite.

    Loads configuration from:
    1. Default values
    2. System-wide config file (if exists)
    3. User config file (if exists)
    4. Environment variables (highest priority)
    """

    # Default configuration
    DEFAULTS = {
        "sensor_port": "/dev/tty.usbserial-FTAK88EN",
        "brake_port": "/dev/tty.usbmodem0000057700001",
        "odrive_port": None,
        "velocity": 10.0,
        "current": 2.0,
        "thermal_duration": 60.0,
        "debug": False,
    }

    # Config file locations (in order of precedence)
    CONFIG_PATHS = [
        Path.home() / ".motor-test-config",
        Path.home() / ".config" / "motor-test" / "config.json",
        Path("motor_test.config"),
        Path("config.json"),
    ]

    def __init__(self):
        self.config: Dict[str, Any] = self.DEFAULTS.copy()
        self._load_config_files()
        self._load_env_vars()

    def _load_config_files(self):
        """Load configuration from available config files."""
        for config_path in self.CONFIG_PATHS:
            if config_path.exists():
                try:
                    with open(config_path, 'r') as f:
                        user_config = json.load(f)
                    self.config.update(user_config)
                    self._config_file_used = config_path
                    break
                except (json.JSONDecodeError, IOError) as e:
                    print(f"Warning: Could not load config from {config_path}: {e}")

    def _load_env_vars(self):
        """Load configuration from environment variables."""
        env_mapping = {
            "MOTOR_TEST_SENSOR_PORT": "sensor_port",
            "MOTOR_TEST_BRAKE_PORT": "brake_port",
            "MOTOR_TEST_ODRIVE_PORT": "odrive_port",
            "MOTOR_TEST_VELOCITY": ("velocity", float),
            "MOTOR_TEST_CURRENT": ("current", float),
            "MOTOR_TEST_THERMAL_DURATION": ("thermal_duration", float),
            "MOTOR_TEST_DEBUG": ("debug", lambda x: x.lower() in ['true', '1', 'yes']),
        }

        for env_var, config_key in env_mapping.items():
            value = os.environ.get(env_var)
            if value is not None:
                if isinstance(config_key, tuple):
                    key, converter = config_key
                    try:
                        self.config[key] = converter(value)
                    except ValueError:
                        print(f"Warning: Invalid value for {env_var}: {value}")
                else:
                    self.config[config_key] = value

    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value."""
        return self.config.get(key, default)

    def set(self, key: str, value: Any):
        """Set configuration value (runtime only)."""
        self.config[key] = value

    def create_example_config(self, path: Optional[Path] = None) -> Path:
        """Create an example configuration file.

        Args:
            path: Optional path for the config file. If None, uses first CONFIG_PATH.

        Returns:
            Path to the created config file.
        """
        if path is None:
            path = self.CONFIG_PATHS[0]

        # Ensure parent directory exists
        path.parent.mkdir(parents=True, exist_ok=True)

        example_config = {
            "_comment": "Motor Test Suite Configuration",
            "_description": "Customize these values for your test bench setup",
            "sensor_port": "/dev/tty.usbserial-FTAK88EN",
            "brake_port": "/dev/tty.usbmodem0000057700001",
            "odrive_port": None,
            "velocity": 10.0,
            "current": 2.0,
            "thermal_duration": 60.0,
            "debug": False,
        }

        with open(path, 'w') as f:
            json.dump(example_config, f, indent=2)

        return path

    def __repr__(self):
        return f"Config({self.config})"


# Global config instance
_config = None


def get_config() -> Config:
    """Get the global configuration instance."""
    global _config
    if _config is None:
        _config = Config()
    return _config


def reset_config():
    """Reset the global configuration (useful for testing)."""
    global _config
    _config = None
