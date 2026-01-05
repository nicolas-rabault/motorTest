#!/usr/bin/env python3

import serial
import time


class PowerSupply:

    def __init__(self, port: str, channel: str = "05", baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.channel = str(channel)
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        self._voltage_set = None
        time.sleep(0.1)

    def _send_command(self, command: str) -> None:
        cmd_bytes = f"{command}\r".encode('ascii')
        self.serial.write(cmd_bytes)
        self.serial.flush()
        time.sleep(0.1)

    def set_voltage(self, voltage: float) -> None:
        command = f"VSET{self.channel}:{voltage:.2f}"
        self._send_command(command)
        self._voltage_set = voltage

    def enable_output(self) -> None:
        if self._voltage_set is None or not (0 <= self._voltage_set <= 24):
            raise ValueError(f"Cannot enable: voltage must be set between 0V and 24V (current: {self._voltage_set}V)")
        self._send_command(f"OUT{self.channel}:1")

    def disable_output(self) -> None:
        self._send_command(f"OUT{self.channel}:0")

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
