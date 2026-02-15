#!/usr/bin/env python3
"""Simple diagnostic script to test CAN connection"""
import dronecan
import time
import sys

def main():
    print("=== DroneCAN Connection Diagnostic ===\n")

    interface = "slcan:COM7@115200"
    bitrate = 1000000
    timeout = 5

    try:
        print(f"1. Opening interface: {interface}")
        print(f"   CAN bitrate: {bitrate} bps\n")

        node = dronecan.make_node(interface, node_id=127, bitrate=bitrate)
        print("   [OK] Interface opened successfully\n")

        print("2. Starting CAN monitor...")
        node.spin()
        print("   [OK] Monitor started\n")

        print(f"3. Listening for messages (timeout: {timeout}s)...")
        print("   Waiting for any CAN traffic...\n")

        messages_received = []

        def message_callback(event):
            messages_received.append(event)
            print(f"   -> Received: {event.message.__class__.__name__} from node {event.transfer.source_node_id}")

        # Subscribe to all messages
        node.add_handler(dronecan.uavcan.protocol.NodeStatus, message_callback)

        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.1)
            if messages_received:
                break

        node.close()

        if messages_received:
            print(f"\n[SUCCESS]: Received {len(messages_received)} message(s)")
            print("\nYour DroneCAN connection is working!")
        else:
            print("\n[NO MESSAGES RECEIVED]")
            print("\nPossible issues:")
            print("  - Device not powered on")
            print("  - Device not running DroneCAN firmware")
            print("  - CAN wiring not connected (CANH, CANL, GND)")
            print("  - Wrong CAN bitrate (device might be using different rate)")
            print("  - Device on different node ID or not sending heartbeats")

    except Exception as e:
        print(f"\n[ERROR]: {e}")
        print("\nConnection failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()
