#!/usr/bin/env python3
"""
CAN Network Discovery Script

This script:
1. Connects to a CAN adapter via USB
2. Starts a DNA (Dynamic Node ID Allocation) server
3. Scans for nodes on the CAN network and prints NodeStatus messages
"""

import argparse
import sys
import time

try:
    import dronecan
    import dronecan.app.dynamic_node_id as dna
    import dronecan.app.node_monitor as nm
    from dronecan import uavcan  # type: ignore[attr-defined]
    from dronecan.driver.python_can import PythonCAN
except ImportError as e:
    print(f"Missing dependency: {e}")
    print("Install with: uv pip install pydronecan pyserial python-can")
    sys.exit(1)

HEALTH_LABELS = {0: "OK", 1: "WARNING", 2: "ERROR", 3: "CRITICAL"}
MODE_LABELS = {
    0: "OPERATIONAL",
    1: "INITIALIZATION",
    2: "MAINTENANCE",
    3: "SOFTWARE_UPDATE",
}


class CANNetworkDiscovery:
    """Manages CAN network connection, DNA server, and node discovery."""

    def __init__(
        self,
        interface: str,
        bitrate: int = 1_000_000,
        bustype: str = "slcan",
        node_id: int = 127,
        baudrate: int | None = None,
    ) -> None:
        """
        Initialize CAN network discovery.

        Args:
            interface: CAN device name (e.g. 'COM5', '/dev/ttyACM0', 'can0')
            bitrate: CAN bus bitrate in bits per second (default: 1000000)
            bustype: python-can bus type (e.g. 'slcan', 'socketcan', 'pcan')
            node_id: Local node ID for the discovery tool (default: 127)
            baudrate: Serial port baud rate for SLCAN USB adapters (default: 115200).
                      Ignored for native CAN interfaces.
        """
        self.interface = interface
        self.bitrate = bitrate
        self.bustype = bustype
        self.node_id = node_id
        self.baudrate = baudrate

        self._node: dronecan.node.Node | None = None
        self._node_monitor: nm.NodeMonitor | None = None
        self._dna_server: dna.CentralizedServer | None = None

    def connect(self) -> bool:
        """Connect to the CAN interface and initialise the DroneCAN node."""
        try:
            # dronecan's built-in SLCAN driver spawns a Windows subprocess that
            # cannot reconfigure USB serial ports. Use python-can's in-process
            # SLCAN driver instead for COM ports on Windows.
            is_com_port = self.interface.upper().startswith("COM")
            if is_com_port and self.bustype == "slcan" and PythonCAN is not None:
                can_driver = PythonCAN(
                    self.interface,
                    bustype="slcan",
                    bitrate=self.bitrate,
                )
                # python-can's SLCAN interface doesn't implement flush_tx_buffer;
                # stub it out so the dronecan writer thread doesn't crash.
                can_driver._bus.flush_tx_buffer = lambda: None
                self._node = dronecan.node.Node(can_driver, node_id=self.node_id)
            else:
                kwargs: dict = {
                    "node_id": self.node_id,
                    "bitrate": self.bitrate,
                    "bustype": self.bustype,
                }
                if self.baudrate is not None:
                    kwargs["baudrate"] = self.baudrate
                self._node = dronecan.make_node(self.interface, **kwargs)
            print(
                f"Connected to '{self.interface}' "
                f"(CAN {self.bitrate} bps, node ID {self.node_id})"
            )
            return True
        except Exception as e:
            print(f"Error connecting to CAN: {e}")
            return False

    def start_dna_server(self) -> None:
        """Start the DNA server so nodes without an ID can get one allocated."""
        if self._node is None:
            return
        self._node_monitor = nm.NodeMonitor(self._node)
        self._dna_server = dna.CentralizedServer(self._node, self._node_monitor)
        print("DNA server started")

    def disconnect(self) -> None:
        """Shut down the node and release the CAN interface."""
        if self._node is not None:
            self._node.close()
            self._node = None
        print("Disconnected from CAN interface")

    def discover_nodes(self, timeout: float | None = None) -> list[int]:
        """
        Listen for NodeStatus messages and print them as they arrive.

        Args:
            timeout: How long to listen (seconds). None means listen until Ctrl-C.

        Returns:
            List of node IDs that were seen.
        """
        if self._node is None:
            print("Not connected to CAN interface")
            return []

        seen_node_ids: set[int] = set()

        def on_node_status(event: dronecan.node.TransferEvent) -> None:
            src = event.transfer.source_node_id
            msg = event.message  # type: ignore[attr-defined]  # set dynamically via setattr
            health = HEALTH_LABELS.get(int(msg.health), str(msg.health))
            mode = MODE_LABELS.get(int(msg.mode), str(msg.mode))
            if src not in seen_node_ids:
                seen_node_ids.add(src)
                print(f"  [NEW] Node {src}")
            print(
                f"  Node {src:3d} | uptime {msg.uptime_sec:6d}s "
                f"| health {health:<8s} | mode {mode}"
            )

        self._node.add_handler(uavcan.protocol.NodeStatus, on_node_status)

        print("\nScanning for nodes — press Ctrl-C to stop.\n")
        deadline = (time.monotonic() + timeout) if timeout is not None else None

        try:
            while True:
                remaining = None
                if deadline is not None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                self._node.spin(timeout=min(remaining or 1.0, 1.0))
        except KeyboardInterrupt:
            print("\nInterrupted by user")

        return sorted(seen_node_ids)


def main() -> None:
    """Entry point for the CAN discovery script."""
    parser = argparse.ArgumentParser(
        description="Connect to a USB CAN adapter, start a DNA server, and"
        " print NodeStatus messages from ArduPilot."
    )
    parser.add_argument(
        "-i",
        "--interface",
        required=True,
        help="CAN device (e.g. COM9, /dev/ttyACM0, can0)",
    )
    parser.add_argument(
        "-b",
        "--bitrate",
        type=int,
        default=1_000_000,
        help="CAN bus bitrate in bps (default: 1000000)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=None,
        help=(
            "Serial port baud rate for SLCAN USB adapters (e.g. 115200). "
            "Omit to use the dronecan default (3000000). "
            "CP210x adapters typically need 115200."
        ),
    )
    parser.add_argument(
        "--bustype",
        default="slcan",
        help="python-can bus type (default: slcan). Use 'socketcan' on Linux.",
    )
    parser.add_argument(
        "--node-id",
        type=int,
        default=127,
        help="Local node ID for this tool (default: 127)",
    )
    parser.add_argument(
        "--no-dna",
        action="store_true",
        help="Disable the DNA server",
    )
    parser.add_argument(
        "-t",
        "--timeout",
        type=float,
        default=None,
        help="Stop after this many seconds (default: run until Ctrl-C)",
    )

    args = parser.parse_args()

    discovery = CANNetworkDiscovery(
        interface=args.interface,
        bitrate=args.bitrate,
        bustype=args.bustype,
        node_id=args.node_id,
        baudrate=args.baudrate,
    )

    if not discovery.connect():
        print("Failed to connect to CAN interface. Exiting.")
        sys.exit(1)

    try:
        if not args.no_dna:
            discovery.start_dna_server()

        node_ids = discovery.discover_nodes(timeout=args.timeout)

        if node_ids:
            print(f"\nDiscovered {len(node_ids)} node(s): {node_ids}")
        else:
            print("\nNo nodes discovered.")
    finally:
        discovery.disconnect()


if __name__ == "__main__":
    main()
