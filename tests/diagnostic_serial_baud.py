#!/usr/bin/env python3
"""Test different serial baud rates for SLCAN"""
import serial
import time

def test_slcan_baud(port, baud_rate):
    """Test if device responds to SLCAN commands at given baud rate"""
    try:
        ser = serial.Serial(port, baud_rate, timeout=0.5)
        time.sleep(0.1)  # Let it settle

        # Try SLCAN close command (C\r)
        ser.write(b'C\r')
        time.sleep(0.1)
        response = ser.read(100)

        # Try SLCAN open at 1Mbps command (S8\r)
        ser.write(b'S8\r')
        time.sleep(0.1)
        response2 = ser.read(100)

        ser.close()

        # Check if we got an ACK (0x0D or '\r')
        if response or response2:
            if b'\r' in response or b'\r' in response2 or b'\x07' in response or b'\x07' in response2:
                return True, f"Got response: {response} / {response2}"
        return False, "No ACK received"
    except Exception as e:
        return False, str(e)

def main():
    port = "COM7"
    common_bauds = [115200, 230400, 500000, 921600, 57600, 38400]

    print(f"Testing SLCAN on {port} with different baud rates...\n")

    for baud in common_bauds:
        print(f"Testing {baud} bps... ", end='', flush=True)
        success, msg = test_slcan_baud(port, baud)
        if success:
            print(f"[SUCCESS] {msg}")
            print(f"\n** Device responds at {baud} bps **")
            print(f"\nTry using: slcan:{port}@{baud}")
            return
        else:
            print(f"[FAIL] {msg}")

    print("\n[NO RESPONSE] at any standard baud rate")
    print("\nYour device might:")
    print("  - Not be a SLCAN adapter")
    print("  - Need special initialization")
    print("  - Use a non-standard baud rate")
    print("  - Be in the wrong mode")

if __name__ == '__main__':
    main()
