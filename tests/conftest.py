"""
Pytest configuration and fixtures for DroneCAN hardware-in-the-loop tests.

Requires the CAN adapter to be connected on COM21 at 1 Mbps.
"""

import dronecan
import pytest
from dronecan.driver.python_can import PythonCAN

INTERFACE = "COM21"
BITRATE = 1_000_000
LOCAL_NODE_ID = 126  # avoids colliding with the target node (100) or ArduPilot (127)


@pytest.fixture(scope="session")
def can_node():
    """Open a DroneCAN node on COM21, shared across the test session."""
    can_driver = PythonCAN(INTERFACE, bustype="slcan", bitrate=BITRATE)
    # python-can's SLCAN driver doesn't implement flush_tx_buffer; stub it out.
    can_driver._bus.flush_tx_buffer = lambda: None  # type: ignore[attr-defined]
    node = dronecan.node.Node(can_driver, node_id=LOCAL_NODE_ID)
    yield node
    node.close()
