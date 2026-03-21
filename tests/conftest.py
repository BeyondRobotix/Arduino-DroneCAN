"""
Pytest configuration and fixtures for DroneCAN hardware-in-the-loop tests.

Requires the CAN adapter to be connected on COM21 at 1 Mbps.
"""

import os
import time

import dronecan
import dronecan.app.dynamic_node_id as dna
import dronecan.app.node_monitor as nm
import pytest
from dronecan.driver.python_can import PythonCAN

INTERFACE = os.environ.get("DRONECAN_INTERFACE", "COM21")
BITRATE = int(os.environ.get("DRONECAN_BITRATE", "1000000"))
LOCAL_NODE_ID = 126  # avoids colliding with the target node (69) or ArduPilot (127)


class DNAServer:
    """Wrapper around the dronecan CentralizedServer that can be
    torn down and recreated to clear the allocation table.

    After restarting a node whose NODEID has changed, call
    ``restart_after_reset(settle)`` — this sleeps for ``settle``
    seconds so the MCU fully resets, then creates a fresh DNA
    server that has no stale entries in the NodeMonitor.
    """

    def __init__(self, node):
        self._node = node
        self._monitor = None
        self._server = None
        self.start()

    def start(self):
        """Create a fresh NodeMonitor + CentralizedServer."""
        self._monitor = nm.NodeMonitor(self._node)
        self._server = dna.CentralizedServer(
            self._node, self._monitor,
        )

    def clear_allocations(self):
        """Wipe the allocation table so the server will honour
        the next preferred-ID request instead of returning a
        cached mapping.  The server and monitor stay running."""
        if self._server is not None:
            self._server._allocation_table._modify(
                "DELETE FROM allocation",
            )

    def close(self):
        if self._server is not None:
            self._server.close()
            self._server = None
        if self._monitor is not None:
            self._monitor.close()
            self._monitor = None


@pytest.fixture(scope="session")
def can_node():
    """Open a DroneCAN node on COM21, shared across the test
    session."""
    can_driver = PythonCAN(
        INTERFACE, bustype="slcan", bitrate=BITRATE,
    )
    # python-can's SLCAN driver doesn't implement
    # flush_tx_buffer; stub it out.
    can_driver._bus.flush_tx_buffer = lambda: None
    node = dronecan.node.Node(
        can_driver, node_id=LOCAL_NODE_ID,
    )

    # Always-on DNA server so nodes can (re-)acquire IDs.
    node._dna = DNAServer(node)

    yield node
    node._dna.close()
    node.close()


@pytest.fixture(scope="session")
def dna_server(can_node):
    """Expose the DNA server for tests that need to restart it
    (e.g. after changing NODEID)."""
    return can_node._dna
