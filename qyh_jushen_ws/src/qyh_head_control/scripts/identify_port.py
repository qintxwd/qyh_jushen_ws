#!/usr/bin/env python3
import sys
import os
import time
from unittest.mock import MagicMock

# Mock smbus2 to avoid dependency error since we don't use encoder motors here
sys.modules['smbus2'] = MagicMock()

# Add the package directory to sys.path to import the sdk
# Script is in src/qyh_head_control/scripts/
# We want to add src/qyh_head_control/ to sys.path so we can import qyh_head_control.sdk
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..'))

try:
    from qyh_head_control.sdk.hiwonder_servo_controller import HiwonderServoController
except ImportError as e:
    print(f"ImportError: {e}")
    # Try adding the inner directory directly
    sys.path.append(os.path.join(current_dir, '..', 'qyh_head_control'))
    try:
        from sdk.hiwonder_servo_controller import HiwonderServoController
    except ImportError as e2:
        print(f"Second ImportError: {e2}")
        print("Error: Could not import SDK. Please ensure you are running this script from the correct location.")
        sys.exit(1)

def check_port(port_name):
    print(f"Testing {port_name}...")
    try:
        # Initialize controller
        # Note: The SDK initializes GPIO pins globally on import for direction control.
        # If the hardware uses different GPIOs for different ports, this might fail for the wrong port.
        # However, for Jetson Nano expansion header, it is usually ttyTHS1 and the specific GPIOs defined in the SDK.
        controller = HiwonderServoController(port_name, 115200)
        controller.set_timeout(10) # Set a short timeout for testing
        
        # Try to read ID from broadcast address 0xFE (254)
        # The get_servo_id method with no args uses 0xFE
        print(f"Sending broadcast ID query to {port_name}...")
        servo_id = controller.get_servo_id()
        
        if servo_id is not None:
            print(f"SUCCESS: Found servo with ID {servo_id} on {port_name}")
            return True
        else:
            print(f"No response on {port_name}")
            return False
    except Exception as e:
        print(f"Error accessing {port_name}: {e}")
        return False

if __name__ == "__main__":
    print("Checking serial ports for servos...")
    print("Note: This script assumes the servo board uses the default GPIO pins for direction control defined in the SDK.")
    
    ports = ['/dev/ttyTHS1', '/dev/ttyTHS2']
    found = False
    
    for port in ports:
        if os.path.exists(port):
            if check_port(port):
                found = True
                print(f"\nCONCLUSION: The correct port is {port}")
                break
        else:
            print(f"Skipping {port} (device file does not exist)")
    
    if not found:
        print("\nCould not find servos on any tested port.")
        print("Troubleshooting tips:")
        print("1. Ensure the servo driver board is powered (6V-12V).")
        print("2. Ensure the board is correctly plugged into the Jetson Nano GPIO header.")
        print("3. Check permissions: 'sudo chmod 666 /dev/ttyTHS1' or add user to dialout group.")
        print("4. If using a USB-to-Serial adapter, the port might be /dev/ttyUSB0 instead.")
