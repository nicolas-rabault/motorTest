#!/usr/bin/env python3

from motor_test.core.power_supply import PowerSupply


class Brake:

    MAX_VOLTAGE = 24.0

    def __init__(self, power_supply: PowerSupply):
        self.power_supply = power_supply
        self._enabled = False
        self._intensity = 0.0
        self.power_supply.set_voltage(0.0)
        self.power_supply.disable_output()

    def enable(self) -> None:
        self.power_supply.enable_output()
        self._enabled = True

    def disable(self) -> None:
        self.power_supply.disable_output()
        self._enabled = False

    def set_intensity(self, intensity: float) -> None:
        if not 0 <= intensity <= 100:
            raise ValueError(f"Intensity must be between 0 and 100, got {intensity}")
        self._intensity = intensity
        voltage = (intensity / 100.0) * self.MAX_VOLTAGE
        self.power_supply.set_voltage(voltage)

    def get_intensity(self) -> float:
        return self._intensity

    def release(self) -> None:
        self.set_intensity(0)
        self.disable()

    def full_brake(self) -> None:
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
        with PowerSupply(args.port, debug=args.debug) as psu:
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

