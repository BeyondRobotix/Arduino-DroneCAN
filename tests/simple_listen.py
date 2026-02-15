#!/usr/bin/env python3
"""Simplest possible CAN listener - just see if anything is on the bus"""
import serial
import time

port = 'COM7'
baud = 115200

print(f"Opening {port} at {baud} bps...")
ser = serial.Serial(port, baud, timeout=0.1)
time.sleep(0.2)

# Close SLCAN channel
print("Sending SLCAN close command...")
ser.write(b'C\r')
time.sleep(0.1)
response = ser.read(100)
print(f"  Response: {response}")

# Set 1Mbps bitrate
print("Setting 1Mbps bitrate...")
ser.write(b'S8\r')
time.sleep(0.1)
response = ser.read(100)
print(f"  Response: {response}")

# Open SLCAN channel
print("Opening SLCAN channel...")
ser.write(b'O\r')
time.sleep(0.1)
response = ser.read(100)
print(f"  Response: {response}")

print("\nListening for CAN frames (10 seconds)...")
print("Raw data from adapter:")
start = time.time()
frame_count = 0

while time.time() - start < 10:
    data = ser.read(100)
    if data:
        frame_count += 1
        print(f"  [{frame_count}] {data}")
    time.sleep(0.01)

print(f"\nReceived {frame_count} responses")
if frame_count == 0:
    print("\nNO CAN TRAFFIC DETECTED")
    print("Check:")
    print("  - Device is powered on")
    print("  - Device firmware is running")
    print("  - CAN wiring (CANH, CANL, GND)")
else:
    print("\nCAN traffic detected! Your setup is working.")

ser.close()
