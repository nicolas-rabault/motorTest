#!/usr/bin/env python3
"""
Power Supply Controller for Tenma 72-13360 (and compatible 72-13300 series)

This module provides a class to control a Tenma power supply via serial communication.
"""

import serial
import time
import argparse


class PowerSupply:
    """
    Controller class for Tenma 72-13360 power supply.
    
    Serial settings:
        - Baud rate: 115200
        - Parity: None
        - Data bits: 8
        - Stop bits: 1
        - Flow control: None
    """
    
    def __init__(self, port: str, channel: str = "05", baudrate: int = 115200, timeout: float = 1.0, debug: bool = False):
        """
        Initialize the power supply controller.
        
        Args:
            port: Serial port path (e.g., '/dev/tty.usbmodem0000057700001')
            channel: Output channel identifier (default: "05" for 72-13360)
            baudrate: Serial baud rate (default: 115200)
            timeout: Serial timeout in seconds (default: 1.0)
            debug: Enable debug output (default: False)
        """
        self.port = port
        self.channel = str(channel)
        self.debug = debug
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        # Small delay to let the connection stabilize
        time.sleep(0.1)
    
    def _send_command(self, command: str) -> None:
        """Send a command to the power supply."""
        cmd_bytes = f"{command}\r".encode('ascii')
        if self.debug:
            print(f"[DEBUG] Sending: {repr(cmd_bytes)}")
        self.serial.write(cmd_bytes)
        self.serial.flush()
        time.sleep(0.1)  # Small delay for command processing
    
    def _query(self, command: str) -> str:
        """Send a query command and return the response."""
        self.serial.reset_input_buffer()
        self._send_command(command)
        # Read all available data
        time.sleep(0.1)
        response = b""
        while self.serial.in_waiting:
            response += self.serial.read(self.serial.in_waiting)
            time.sleep(0.05)
        # Also try readline as fallback
        if not response:
            response = self.serial.readline()
        decoded = response.decode('ascii', errors='ignore').strip()
        if self.debug:
            print(f"[DEBUG] Received: {repr(response)} -> '{decoded}'")
        return decoded
    
    def set_voltage(self, voltage: float) -> None:
        """
        Set the output voltage.
        
        Args:
            voltage: Voltage in volts (e.g., 12.5 for 12.5V)
        """
        command = f"VSET{self.channel}:{voltage:.2f}"
        self._send_command(command)
    
    def get_voltage_setting(self) -> float:
        """
        Get the voltage setting (not actual output).
        
        Returns:
            Voltage setting in volts
        """
        response = self._query(f"VSET{self.channel}?")
        try:
            return float(response)
        except ValueError:
            return 0.0
    
    def get_voltage(self) -> float:
        """
        Get the actual output voltage.
        
        Returns:
            Actual output voltage in volts
        """
        response = self._query(f"VOUT{self.channel}?")
        try:
            return float(response)
        except ValueError:
            return 0.0
    
    def set_current(self, current: float) -> None:
        """
        Set the output current limit.
        
        Args:
            current: Current in amperes (e.g., 2.5 for 2.5A)
        """
        command = f"ISET{self.channel}:{current:.3f}"
        self._send_command(command)
    
    def get_current_setting(self) -> float:
        """
        Get the current limit setting (not actual output).
        
        Returns:
            Current limit setting in amperes
        """
        response = self._query(f"ISET{self.channel}?")
        try:
            return float(response)
        except ValueError:
            return 0.0
    
    def get_current(self) -> float:
        """
        Get the actual output current.
        
        Returns:
            Actual output current in amperes
        """
        response = self._query(f"IOUT{self.channel}?")
        try:
            return float(response)
        except ValueError:
            return 0.0
    
    def enable_output(self) -> None:
        """Enable the power supply output."""
        self._send_command(f"OUT{self.channel}:1")
    
    def disable_output(self) -> None:
        """Disable the power supply output."""
        self._send_command(f"OUT{self.channel}:0")
    
    def set_output(self, enabled: bool) -> None:
        """
        Set the output state.
        
        Args:
            enabled: True to enable output, False to disable
        """
        if enabled:
            self.enable_output()
        else:
            self.disable_output()
    
    def get_status(self) -> str:
        """
        Get the power supply status.
        
        Returns:
            Status string from the power supply
        """
        return self._query("STATUS?")
    
    def get_identification(self) -> str:
        """
        Get the power supply identification.
        
        Returns:
            Identification string (model, version, serial number)
        """
        return self._query("*IDN?")
    
    def lock_panel(self, lock: bool = True) -> None:
        """
        Lock or unlock the front panel.
        
        Args:
            lock: True to lock, False to unlock
        """
        self._send_command(f"LOCK{1 if lock else 0}")
    
    def beep(self, enabled: bool = True) -> None:
        """
        Enable or disable the beep.
        
        Args:
            enabled: True to enable beep, False to disable
        """
        self._send_command(f"BEEP{1 if enabled else 0}")
    
    def close(self) -> None:
        """Close the serial connection."""
        if self.serial.is_open:
            self.serial.close()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        return False


def main():
    """Main function for command-line testing."""
    parser = argparse.ArgumentParser(
        description="Control Tenma 72-13360 Power Supply",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --voltage 12.0 --current 1.0 --enable
  %(prog)s --voltage 5.0 --current 0.5 --enable --monitor 5
  %(prog)s --disable
        """
    )
    parser.add_argument(
        "--port", "-p",
        default="/dev/tty.usbmodem0000057700001",
        help="Serial port (default: /dev/tty.usbmodem0000057700001)"
    )
    parser.add_argument(
        "--channel", "-c",
        type=str,
        default="05",
        help="Output channel (default: 05 for 72-13360)"
    )
    parser.add_argument(
        "--voltage", "-v",
        type=float,
        help="Set output voltage (V)"
    )
    parser.add_argument(
        "--current", "-i",
        type=float,
        help="Set current limit (A)"
    )
    parser.add_argument(
        "--enable", "-e",
        action="store_true",
        help="Enable output"
    )
    parser.add_argument(
        "--disable", "-d",
        action="store_true",
        help="Disable output"
    )
    parser.add_argument(
        "--monitor", "-m",
        type=float,
        nargs="?",
        const=1.0,
        help="Monitor output current for N seconds (default: 1.0)"
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Show power supply identification"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output to see raw commands/responses"
    )
    
    args = parser.parse_args()
    
    try:
        with PowerSupply(args.port, channel=args.channel, debug=args.debug) as psu:
            # Show identification if requested
            if args.info:
                print(f"Identification: {psu.get_identification()}")
            
            # Set voltage if specified
            if args.voltage is not None:
                psu.set_voltage(args.voltage)
                print(f"Voltage set to: {args.voltage:.2f} V")
            
            # Set current if specified
            if args.current is not None:
                psu.set_current(args.current)
                print(f"Current limit set to: {args.current:.3f} A")
            
            # Enable/disable output
            if args.enable and args.disable:
                print("Error: Cannot both enable and disable output")
                return 1
            elif args.enable:
                psu.enable_output()
                print("Output ENABLED")
            elif args.disable:
                psu.disable_output()
                print("Output DISABLED")
            
            # Monitor if requested
            if args.monitor is not None:
                print(f"\nMonitoring output for {args.monitor:.1f} seconds...")
                print("-" * 40)
                start_time = time.time()
                while (time.time() - start_time) < args.monitor:
                    voltage = psu.get_voltage()
                    current = psu.get_current()
                    print(f"  Voltage: {voltage:7.2f} V  |  Current: {current:7.3f} A")
                    time.sleep(0.5)
                print("-" * 40)
            
            # Show current readings
            print(f"\nCurrent readings:")
            print(f"  Voltage setting: {psu.get_voltage_setting():.2f} V")
            print(f"  Current limit:   {psu.get_current_setting():.3f} A")
            print(f"  Output voltage:  {psu.get_voltage():.2f} V")
            print(f"  Output current:  {psu.get_current():.3f} A")
            
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 0
    
    return 0


if __name__ == "__main__":
    exit(main())

