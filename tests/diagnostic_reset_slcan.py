#!/usr/bin/env python3
"""Reset SLCAN adapter to known state"""
import serial
import time

def reset_slcan(port='COM7', baud=115200):
    """Send close command to SLCAN adapter"""
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.2)

        # Send close command multiple times
        print("Sending SLCAN close commands...")
        for _ in range(3):
            ser.write(b'C\r')
            time.sleep(0.1)
            response = ser.read(100)
            if response:
                print(f"  Response: {response}")

        ser.close()
        print("[OK] SLCAN adapter reset")
        return True
    except Exception as e:
        print(f"[ERROR]: {e}")
        return False

if __name__ == '__main__':
    reset_slcan()
