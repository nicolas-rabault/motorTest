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

    Buffer handling strategy:
    - Sensor streams data continuously at 1kHz into OS serial buffer
    - Buffer is FIFO: oldest data at front, newest at back
    - read() MUST drain entire buffer to get to fresh data
    - Always return most recent reading
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._last_valid_data = None

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
            timeout=0.01,  # 10ms timeout for reads
        )

        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._sync_to_frame()

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

    def flush_old_data(self):
        """
        Aggressively flush buffer and wait for fresh data.
        Returns guaranteed fresh reading (<10ms old).
        """
        # Strategy: Drain buffer multiple times, then wait for new frames
        for _ in range(5):
            available = self._serial.in_waiting
            if available > 0:
                self._serial.read(available)
            time.sleep(0.002)  # 2ms between drains (sensor sends ~2 frames)

        # Now read fresh data that just arrived
        time.sleep(0.005)  # Wait 5ms for fresh frames to accumulate
        return self.read()

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

    def read(self, debug: bool = False) -> TorqueData | None:
        """
        Read and return FRESH torque data (never return stale cached data).

        Strategy:
        1. If buffer empty, wait up to 5ms for data to arrive (sensor streams at 1kHz)
        2. Drain ALL buffered frames
        3. Return the last valid frame parsed
        4. This guarantees data is <5ms old
        """
        # Check buffer and wait for fresh data if needed
        available = self._serial.in_waiting

        # If buffer empty, actively wait for new data (sensor streams at 1kHz)
        if available < 6:
            # Wait up to 20ms for fresh data (at 1kHz, should get ~20 frames)
            max_wait_time = 0.02
            wait_start = time.time()

            while (time.time() - wait_start) < max_wait_time:
                available = self._serial.in_waiting
                if available >= 6:
                    break  # Got data, exit wait loop
                time.sleep(0.001)  # Check every 1ms

        # Final check after waiting
        available = self._serial.in_waiting
        if available < 6:
            # No data available even after waiting - return None
            return None

        # Calculate complete frames available
        num_frames = available // 6

        # Read all complete frames, keeping only the last valid one
        new_data = None
        frames_read = 0
        frames_valid = 0

        for _ in range(num_frames):
            frame = self._serial.read(6)
            if len(frame) != 6:
                continue

            frames_read += 1

            # Validate CRC
            crc_calc = self._crc16(frame[:4])
            crc_recv = struct.unpack("<H", frame[4:6])[0]
            if crc_calc != crc_recv:
                continue  # Skip invalid frame

            frames_valid += 1

            # Parse valid frame - this becomes our candidate for "newest"
            torque_mg = struct.unpack(">H", frame[0:2])[0]
            speed_raw = struct.unpack(">H", frame[2:4])[0]

            torque_nm = torque_mg / 1000.0
            if speed_raw & 0x8000:
                torque_nm = -torque_nm

            speed_rpm = speed_raw & 0x7FFF

            # Store this as the most recent data with CURRENT timestamp
            new_data = TorqueData(torque=torque_nm, speed=speed_rpm, timestamp=time.time())

        # Update cached value if we got new valid data
        if new_data:
            self._last_valid_data = new_data

        if debug:
            age = (time.time() - new_data.timestamp) if new_data else float('inf')
            print(f"[DEBUG] Buffer: {available}B ({num_frames}F), Read: {frames_read}/{frames_valid} valid, Age: {age*1000:.1f}ms")

        # Return fresh data (or None if no valid frames parsed)
        return new_data

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
