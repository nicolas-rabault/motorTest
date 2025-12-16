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
        if self._serial and self._serial.is_open:
            self._serial.close()
            time.sleep(0.1)

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )

        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._sync_to_frame()
        self._serial.timeout = 0.001

    def _sync_to_frame(self):
        """Find frame boundary by validating 10 consecutive frames."""
        buffer = bytearray()
        consecutive_valid = 0

        while consecutive_valid < 10:
            byte = self._serial.read(1)
            if not byte:
                continue

            buffer.append(byte[0])

            if len(buffer) < 6:
                continue

            # Check if first 6 bytes form a valid frame
            crc_calc = self._crc16(buffer[:4])
            crc_recv = (buffer[5] << 8) | buffer[4]

            if crc_calc == crc_recv:
                consecutive_valid += 1
                buffer = buffer[6:]  # Remove valid frame
            else:
                consecutive_valid = 0
                buffer = buffer[1:]  # Slide by 1 byte

        time.sleep(0.02)  # Let buffer accumulate

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
        """
        Read one 6-byte frame: [D1D2 D3D4 CRC_L CRC_H]
        D1D2: Torque (mg), big-endian
        D3D4: Speed (rpm, 15 bits) + sign bit (MSB)
        CRC: Little-endian
        """
        if self._serial.in_waiting < 6:
            return None

        frame = self._serial.read(6)

        crc_calc = self._crc16(frame[:4])
        crc_recv = struct.unpack("<H", frame[4:6])[0]
        if crc_calc != crc_recv:
            self._serial.reset_input_buffer()
            return None

        torque_mg = struct.unpack(">H", frame[0:2])[0]
        speed_raw = struct.unpack(">H", frame[2:4])[0]

        torque_nm = torque_mg / 1000.0
        if speed_raw & 0x8000:
            torque_nm = -torque_nm

        speed_rpm = speed_raw & 0x7FFF

        return TorqueData(torque=torque_nm, speed=speed_rpm, timestamp=time.time())

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
        while True:
            data = sensor.read()
            if data:
                on_data(data)
