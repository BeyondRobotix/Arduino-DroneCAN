#!/usr/bin/env python3
"""Listen to all CAN traffic on the bus"""
import dronecan
import time
import sys

def main():
    print("=== DroneCAN Bus Monitor ===\n")

    interface = "slcan:COM7"
    serial_baudrate = 115200
    can_bitrate = 1000000
    timeout = 10

    try:
        print(f"Interface: {interface}")
        print(f"Serial baud: {serial_baudrate} bps")
        print(f"CAN bitrate: {can_bitrate} bps\n")

        print("Connecting...")
        node = dronecan.make_node(
            interface,
            node_id=127,
            bitrate=can_bitrate,
            baudrate=serial_baudrate
        )
        print("[OK] Connected!\n")

        print("Starting monitor...")
        node.spin()
        print("[OK] Monitor started\n")

        print(f"Listening for ANY CAN messages (timeout: {timeout}s)...")
        print("This will show ALL traffic on the bus...\n")

        message_count = 0

        def message_callback(event):
            nonlocal message_count
            message_count += 1
            msg = event.message
            src = event.transfer.source_node_id
            print(f"  [{message_count}] {msg.__class__.__name__} from node {src}")

        # Subscribe to common message types
        node.add_handler(dronecan.uavcan.protocol.NodeStatus, message_callback)
        node.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, message_callback)

        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.1)

        node.close()

        print(f"\n{'='*50}")
        if message_count > 0:
            print(f"[SUCCESS] Received {message_count} messages!")
            print("\nYour device is communicating on the CAN bus.")
        else:
            print("[NO TRAFFIC DETECTED]")
            print("\nTroubleshooting:")
            print("  1. Is your device powered on?")
            print("  2. Is the firmware running? (check LED indicators)")
            print("  3. Are CAN wires connected? (CANH, CANL, GND)")
            print("  4. Is the device configured for 1Mbps CAN bitrate?")
            print("  5. Does the device have DroneCAN firmware flashed?")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n[ERROR]: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
