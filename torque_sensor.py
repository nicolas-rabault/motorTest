#!/usr/bin/env python3
"""
ATO-TQS-DYN-200 Digital Rotary Torque Sensor
Subprocess-based continuous data acquisition with ring buffer.
"""

import serial
import struct
import time
import multiprocessing as mp
import numpy as np
from dataclasses import dataclass


@dataclass
class TorqueData:
    torque: float  # Nm
    speed: int     # rpm
    timestamp: float


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


def _reader_process(port: str, baudrate: int, buffer_raw, write_index, data_rate):
    """Subprocess: continuously read sensor and update ring buffer."""
    buffer_array = np.frombuffer(buffer_raw.get_obj()).reshape(60000, 3)

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1
        )
    except Exception:
        return

    # Sync to frame boundary
    buffer = bytearray()
    consecutive_valid = 0
    sync_timeout = 5.0
    sync_start = time.time()

    while consecutive_valid < 10:
        if (time.time() - sync_start) > sync_timeout:
            return

        byte = ser.read(1)
        if not byte:
            continue
        buffer.append(byte[0])
        if len(buffer) < 6:
            continue
        crc_calc = _crc16(buffer[:4])
        crc_recv = (buffer[5] << 8) | buffer[4]
        if crc_calc == crc_recv:
            consecutive_valid += 1
            buffer = buffer[6:]
        else:
            consecutive_valid = 0
            buffer = buffer[1:]

    # Main acquisition loop
    frame_count = 0
    start_time = time.time()

    while True:
        frame = ser.read(6)
        if len(frame) != 6:
            continue

        crc_calc = _crc16(frame[:4])
        crc_recv = struct.unpack("<H", frame[4:6])[0]
        if crc_calc != crc_recv:
            continue

        torque_mg = struct.unpack(">H", frame[0:2])[0]
        speed_raw = struct.unpack(">H", frame[2:4])[0]

        torque_nm = torque_mg / 1000.0
        if speed_raw & 0x8000:
            torque_nm = -torque_nm
        speed_rpm = speed_raw & 0x7FFF

        idx = write_index.value
        buffer_array[idx, 0] = time.time()
        buffer_array[idx, 1] = torque_nm
        buffer_array[idx, 2] = speed_rpm
        write_index.value = (idx + 1) % 60000

        frame_count += 1
        if frame_count % 100 == 0:
            elapsed = time.time() - start_time
            data_rate.value = frame_count / elapsed


class TorqueSensor:
    """
    ATO-TQS-DYN-200 with subprocess acquisition.
    Ring buffer: 60s @ 1kHz (60,000 entries).
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._process = None
        self._buffer = None
        self._write_index = None
        self._data_rate = None

    def connect(self):
        self._buffer = mp.Array('d', 60000 * 3)
        self._buffer_np = np.frombuffer(self._buffer.get_obj()).reshape(60000, 3)
        self._buffer_np[:, 0] = -1.0
        self._write_index = mp.Value('i', 0)
        self._data_rate = mp.Value('d', 0.0)

        self._process = mp.Process(
            target=_reader_process,
            args=(self.port, self.baudrate, self._buffer, self._write_index, self._data_rate)
        )
        self._process.start()
        time.sleep(0.5)

    def disconnect(self):
        if self._process:
            self._process.terminate()
            self._process.join(timeout=1.0)
            self._process = None

    def read(self) -> TorqueData:
        """Get most recent sample."""
        idx = (self._write_index.value - 1) % 60000
        return TorqueData(
            timestamp=self._buffer_np[idx, 0],
            torque=self._buffer_np[idx, 1],
            speed=int(self._buffer_np[idx, 2])
        )

    def read_recent(self, n: int) -> list[TorqueData]:
        """Get last N samples (newest first)."""
        idx = self._write_index.value
        result = []
        for i in range(min(n, idx, 60000)):
            pos = (idx - 1 - i) % 60000
            result.append(TorqueData(
                timestamp=self._buffer_np[pos, 0],
                torque=self._buffer_np[pos, 1],
                speed=int(self._buffer_np[pos, 2])
            ))
        return result

    def read_range(self, duration_s: float) -> list[TorqueData]:
        """Get all samples from last duration_s seconds (newest first)."""
        cutoff = time.time() - duration_s
        idx = self._write_index.value
        result = []
        for i in range(min(idx, 60000)):
            pos = (idx - 1 - i) % 60000
            ts = self._buffer_np[pos, 0]
            if ts < cutoff:
                break
            result.append(TorqueData(
                timestamp=ts,
                torque=self._buffer_np[pos, 1],
                speed=int(self._buffer_np[pos, 2])
            ))
        return result

    def get_data_rate(self) -> float:
        """Get current acquisition rate (Hz)."""
        return self._data_rate.value

    def get_zero_offset(self, duration_s: float = 1.0) -> float:
        """Calculate zero offset by averaging recent data."""
        samples = self.read_range(duration_s)
        if not samples:
            return 0.0
        return sum(s.torque for s in samples) / len(samples)

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

    with TorqueSensor(args.port) as sensor:
        print("Streaming. Ctrl+C to stop.\n")
        while True:
            data = sensor.read()
            if data:
                rate = sensor.get_data_rate()
                print(f"\rTorque: {data.torque:9.3f} Nm  Speed: {data.speed:6d} rpm  Rate: {rate:6.1f} Hz", end="", flush=True)
            time.sleep(0.1)
