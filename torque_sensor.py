#!/usr/bin/env python3
"""
ATO-TQS-DYN-200 Digital Rotary Torque Sensor
RS485 Modbus RTU communication.
"""

import serial
import struct
import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class TorqueData:
    """Torque sensor reading."""
    torque: float  # Nm
    speed: int     # rpm
    timestamp: float  # time.time()


class TorqueSensor:
    """
    ATO-TQS-DYN-200 Torque Sensor interface.
    
    Usage:
        sensor = TorqueSensor("/dev/tty.usbserial-XXXX")
        sensor.connect()
        
        # Single reading
        data = sensor.read()
        print(f"Torque: {data.torque} Nm, Speed: {data.speed} rpm")
        
        # Continuous reading (callback)
        sensor.stream(callback=lambda d: print(d))
        
        sensor.disconnect()
    """
    
    # Modbus registers
    REG_TORQUE = 0x0000
    REG_SPEED = 0x0002
    REG_POWER = 0x0004
    
    def __init__(
        self,
        port: str,
        baudrate: int = 19200,
        address: int = 0x01,
        stopbits: int = 2
    ):
        self.port = port
        self.baudrate = baudrate
        self.address = address
        self.stopbits = stopbits
        self._serial: Optional[serial.Serial] = None
    
    def connect(self) -> None:
        """Open serial connection."""
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO if self.stopbits == 2 else serial.STOPBITS_ONE,
            timeout=0.1
        )
    
    def disconnect(self) -> None:
        """Close serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
    
    def is_connected(self) -> bool:
        """Check if connected."""
        return self._serial is not None and self._serial.is_open
    
    @staticmethod
    def _calculate_crc(data: bytes) -> bytes:
        """Calculate Modbus CRC16."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack("<H", crc)
    
    def _read_registers(self, reg_addr: int, count: int = 2) -> Optional[bytes]:
        """Read Modbus registers."""
        if not self._serial:
            return None
        
        request = bytes([
            self.address,
            0x03,  # Read holding registers
            reg_addr >> 8,
            reg_addr & 0xFF,
            0x00,
            count
        ])
        request += self._calculate_crc(request)
        
        self._serial.reset_input_buffer()
        self._serial.write(request)
        self._serial.flush()
        
        # Wait for response
        time.sleep(0.01)
        
        # Read response
        response = b''
        deadline = time.time() + 0.05
        while time.time() < deadline:
            if self._serial.in_waiting:
                response += self._serial.read(self._serial.in_waiting)
                if len(response) >= 9:
                    break
            time.sleep(0.001)
        
        return response if len(response) >= 9 else None
    
    def read_torque(self) -> Optional[float]:
        """Read torque value in Nm."""
        response = self._read_registers(self.REG_TORQUE, 2)
        if response and len(response) >= 7:
            raw = struct.unpack(">i", response[3:7])[0]
            return raw / 1000.0  # 0.001 precision
        return None
    
    def read_speed(self) -> Optional[int]:
        """Read speed value in rpm."""
        response = self._read_registers(self.REG_SPEED, 2)
        if response and len(response) >= 7:
            return struct.unpack(">i", response[3:7])[0]
        return None
    
    def read(self) -> Optional[TorqueData]:
        """Read torque and speed."""
        torque = self.read_torque()
        speed = self.read_speed()
        
        if torque is not None and speed is not None:
            return TorqueData(
                torque=torque,
                speed=speed,
                timestamp=time.time()
            )
        return None
    
    def read_torque_fast(self) -> Optional[float]:
        """
        Read only torque for maximum speed.
        Use this for high-frequency data acquisition.
        """
        return self.read_torque()
    
    def stream(self, callback, interval: float = 0.0) -> None:
        """
        Continuously read and call callback with TorqueData.
        Set interval=0 for maximum speed.
        Press Ctrl+C to stop.
        """
        try:
            while True:
                data = self.read()
                if data:
                    callback(data)
                if interval > 0:
                    time.sleep(interval)
        except KeyboardInterrupt:
            pass
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


# =============================================================================
# Command-line interface
# =============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="ATO-TQS-DYN-200 Torque Sensor")
    parser.add_argument("--port", default="/dev/tty.usbserial-FTAK88EN", help="Serial port")
    parser.add_argument("--baud", type=int, default=19200, help="Baud rate")
    parser.add_argument("--address", type=int, default=1, help="Modbus address")
    args = parser.parse_args()
    
    print(f"Connecting to {args.port} at {args.baud} baud...")
    
    with TorqueSensor(args.port, args.baud, args.address) as sensor:
        print("Connected. Press Ctrl+C to stop.\n")
        
        try:
            while True:
                data = sensor.read()
                if data:
                    print(f"\rTorque: {data.torque:9.3f} Nm  |  Speed: {data.speed:6d} rpm", end="", flush=True)
                else:
                    print("\rTorque:       --- Nm  |  Speed:    --- rpm", end="", flush=True)
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n\nStopped.")


if __name__ == "__main__":
    main()

