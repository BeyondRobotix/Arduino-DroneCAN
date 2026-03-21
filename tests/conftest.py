"""
Pytest configuration and fixtures for DroneCAN hardware-in-the-loop tests.

Requires the CAN adapter to be connected on COM21 at 1 Mbps.
"""

import dronecan
import dronecan.app.dynamic_node_id as dna
import dronecan.app.node_monitor as nm
import pytest
from dronecan.driver.python_can import PythonCAN

INTERFACE = "COM21"
BITRATE = 1_000_000
LOCAL_NODE_ID = 126  # avoids colliding with the target node (69) or ArduPilot (127)


@pytest.fixture(scope="session")
def can_node():
    """Open a DroneCAN node on COM21, shared across the test session.

    Includes a DNA server so nodes can (re-)acquire their IDs — needed
    for the restart test.
    """
    can_driver = PythonCAN(INTERFACE, bustype="slcan", bitrate=BITRATE)
    # python-can's SLCAN driver doesn't implement flush_tx_buffer; stub it out.
    can_driver._bus.flush_tx_buffer = lambda: None
    node = dronecan.node.Node(can_driver, node_id=LOCAL_NODE_ID)

    # DNA server: allocates node IDs to nodes that request them after reboot.
    monitor = nm.NodeMonitor(node)
    _dna_server = dna.CentralizedServer(node, monitor)  # noqa: F841 — must stay alive

    yield node
    node.close()
