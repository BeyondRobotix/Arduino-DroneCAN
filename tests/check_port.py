#!/usr/bin/env python3
"""Check if COM port is available and responding"""
import serial
import sys

def check_port(port='COM7', baud=115200):
    """Check if port can be opened"""
    try:
        print(f"Attempting to open {port}...")
        ser = serial.Serial(port, baud, timeout=0.5)
        print(f"  [OK] Successfully opened {port}")

        # Try to send SLCAN close command
        ser.write(b'C\r')
        import time
        time.sleep(0.1)
        response = ser.read(100)

        if response:
            print(f"  [OK] Device responds: {response}")
        else:
            print(f"  [WARN] No response (device might not be SLCAN)")

        ser.close()
        print(f"\n[SUCCESS] {port} is available and working!")
        return True

    except serial.SerialException as e:
        if "PermissionError" in str(e) or "Access is denied" in str(e):
            print(f"  [ERROR] {port} is LOCKED by another program")
            print("\nTo fix:")
            print("  1. Close any serial monitors (Arduino IDE, PuTTY, etc.)")
            print("  2. Unplug and replug the USB device")
            print("  3. Or restart your terminal/IDE")
        else:
            print(f"  [ERROR] {e}")
        return False

    except Exception as e:
        print(f"  [ERROR] Unexpected error: {e}")
        return False

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    success = check_port(port)
    sys.exit(0 if success else 1)
