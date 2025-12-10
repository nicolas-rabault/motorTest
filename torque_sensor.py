#!/usr/bin/env python3
"""
ATO-TQS-DYN-200 Digital Rotary Torque Sensor
HEX Active upload mode (Communication mode 0) for 1KHz data rate.
"""

import serial
import struct
import time
from dataclasses import dataclass


@dataclass
class TorqueData:
    torque: float  # Nm
    speed: int     # rpm
    timestamp: float


class TorqueSensor:
    """
    ATO-TQS-DYN-200 in HEX Active upload mode.
    Receives torque and speed at 1000Hz (115200 baud).
    """
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
    
    def connect(self):
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.001
        )
        self._serial.reset_input_buffer()
    
    def disconnect(self):
        if self._serial:
            self._serial.close()
            self._serial = None
    
    @staticmethod
    def _crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def read(self) -> TorqueData | None:
        """Read one 6-byte frame: D1D2=torque, D3D4=speed+sign, D5D6=CRC."""
        if self._serial.in_waiting < 6:
            return None
        
        frame = self._serial.read(6)
        
        # Verify CRC16
        crc_calc = self._crc16(frame[:4])
        crc_recv = struct.unpack("<H", frame[4:6])[0]
        if crc_calc != crc_recv:
            # Out of sync, discard buffer and resync
            self._serial.reset_input_buffer()
            return None
        
        # Parse: D1D2 = torque magnitude, D3 MSB = torque sign, D3D4 lower 15 bits = speed
        torque_raw = struct.unpack(">H", frame[0:2])[0]
        speed_raw = struct.unpack(">H", frame[2:4])[0]
        
        torque_negative = (speed_raw & 0x8000) != 0
        speed = speed_raw & 0x7FFF
        torque = (-torque_raw if torque_negative else torque_raw) / 1000.0
        
        return TorqueData(torque=torque, speed=speed, timestamp=time.time())
    
    def stream(self, callback):
        """Continuous read. Ctrl+C to stop."""
        self._serial.reset_input_buffer()
        while True:
            data = self.read()
            if data:
                callback(data)
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, *args):
        self.disconnect()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/tty.usbserial-FTAK88EN")
    args = parser.parse_args()
    
    count = 0
    t_start = time.time()
    
    def on_data(d):
        global count, t_start
        count += 1
        if count % 100 == 0:
            rate = count / (time.time() - t_start)
            print(f"\rTorque: {d.torque:9.3f} Nm  Speed: {d.speed:6d} rpm  Rate: {rate:6.1f} Hz", end="", flush=True)
    
    with TorqueSensor(args.port) as sensor:
        print("Streaming. Ctrl+C to stop.\n")
        sensor.stream(on_data)
