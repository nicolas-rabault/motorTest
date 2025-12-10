#!/usr/bin/env python3
"""
Brake Controller using Power Supply

This module provides a class to control an electromagnetic brake via a power supply.
The brake intensity is controlled by varying the voltage output (0V to 24V).
"""

from power_supply import PowerSupply


class Brake:
    """
    Controller class for an electromagnetic brake.
    
    The brake is controlled via a power supply where:
        - 0% intensity = 0V (no braking)
        - 100% intensity = 24V (full braking)
    """
    
    MAX_VOLTAGE = 24.0  # Maximum voltage for 100% brake intensity
    
    def __init__(self, power_supply: PowerSupply):
        """
        Initialize the brake controller.
        
        Args:
            power_supply: PowerSupply instance to control the brake
        """
        self.power_supply = power_supply
        self._enabled = False
        self._intensity = 0.0
        
        # Initialize brake to disabled state with 0V
        self.power_supply.set_voltage(0.0)
        self.power_supply.disable_output()
    
    def enable(self) -> None:
        """Enable the brake output."""
        self.power_supply.enable_output()
        self._enabled = True
    
    def disable(self) -> None:
        """Disable the brake output."""
        self.power_supply.disable_output()
        self._enabled = False
    
    def set_enabled(self, enabled: bool) -> None:
        """
        Set the brake enabled state.
        
        Args:
            enabled: True to enable brake, False to disable
        """
        if enabled:
            self.enable()
        else:
            self.disable()
    
    def is_enabled(self) -> bool:
        """
        Check if the brake is currently enabled.
        
        Returns:
            True if brake is enabled, False otherwise
        """
        return self._enabled
    
    def set_intensity(self, intensity: float) -> None:
        """
        Set the brake intensity.
        
        Args:
            intensity: Brake intensity from 0 to 100 (percentage)
                       0% = 0V (no braking)
                       100% = 24V (full braking)
        
        Raises:
            ValueError: If intensity is not between 0 and 100
        """
        if not 0 <= intensity <= 100:
            raise ValueError(f"Intensity must be between 0 and 100, got {intensity}")
        
        self._intensity = intensity
        voltage = (intensity / 100.0) * self.MAX_VOLTAGE
        self.power_supply.set_voltage(voltage)
    
    def get_intensity(self) -> float:
        """
        Get the current brake intensity setting.
        
        Returns:
            Brake intensity from 0 to 100 (percentage)
        """
        return self._intensity
    
    def get_voltage(self) -> float:
        """
        Get the actual output voltage from the power supply.
        
        Returns:
            Actual output voltage in volts
        """
        return self.power_supply.get_voltage()
    
    def get_current(self) -> float:
        """
        Get the actual output current from the power supply.
        
        Returns:
            Actual output current in amperes
        """
        return self.power_supply.get_current()
    
    def release(self) -> None:
        """Fully release the brake (set to 0% and disable)."""
        self.set_intensity(0)
        self.disable()
    
    def full_brake(self) -> None:
        """Apply full braking (set to 100% and enable)."""
        self.set_intensity(100)
        self.enable()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Control electromagnetic brake via power supply",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --intensity 50 --enable
  %(prog)s --intensity 100 --enable
  %(prog)s --disable
  %(prog)s --release
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
        help="Output channel (default: 05)"
    )
    parser.add_argument(
        "--intensity", "-i",
        type=float,
        help="Set brake intensity (0-100%%)"
    )
    parser.add_argument(
        "--enable", "-e",
        action="store_true",
        help="Enable brake output"
    )
    parser.add_argument(
        "--disable", "-d",
        action="store_true",
        help="Disable brake output"
    )
    parser.add_argument(
        "--release", "-r",
        action="store_true",
        help="Fully release the brake (0%% and disable)"
    )
    parser.add_argument(
        "--full", "-f",
        action="store_true",
        help="Apply full braking (100%% and enable)"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug output"
    )
    
    args = parser.parse_args()
    
    try:
        with PowerSupply(args.port, channel=args.channel, debug=args.debug) as psu:
            brake = Brake(psu)
            
            # Handle release/full brake shortcuts
            if args.release:
                brake.release()
                print("Brake RELEASED (0%, disabled)")
            elif args.full:
                brake.full_brake()
                print("Brake FULL (100%, enabled)")
            else:
                # Set intensity if specified
                if args.intensity is not None:
                    brake.set_intensity(args.intensity)
                    voltage = (args.intensity / 100.0) * Brake.MAX_VOLTAGE
                    print(f"Brake intensity set to: {args.intensity:.1f}% ({voltage:.2f}V)")
                
                # Enable/disable output
                if args.enable and args.disable:
                    print("Error: Cannot both enable and disable brake")
                    exit(1)
                elif args.enable:
                    brake.enable()
                    print("Brake ENABLED")
                elif args.disable:
                    brake.disable()
                    print("Brake DISABLED")
            
            # Show current state
            print(f"\nCurrent brake state:")
            print(f"  Intensity: {brake.get_intensity():.1f}%")
            print(f"  Voltage:   {brake.get_voltage():.2f}V")
            print(f"  Current:   {brake.get_current():.3f}A")
            print(f"  Enabled:   {brake.is_enabled()}")
            
    except Exception as e:
        print(f"Error: {e}")
        exit(1)

