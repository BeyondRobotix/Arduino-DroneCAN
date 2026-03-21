"""
Hardware-in-the-loop tests for the DroneCAN node running src/main.cpp.

The node under test (node ID 69):
  - Broadcasts NodeStatus at ~1 Hz (DroneCAN mandatory)
  - Broadcasts uavcan.equipment.power.BatteryInfo at 10 Hz
    - voltage  = raw analogRead(PA1)  → 0..4095
    - current  = raw analogRead(PA0)  → 0..4095
    - temperature = CPU temp in °C    → reasonable MCU range
  - PARM_1 is set to 50.0 in setup()
  - PARM_2 is user-configurable (range 0-100)
"""

import time

import dronecan
from dronecan import uavcan  # type: ignore[attr-defined]

TARGET_NODE_ID = 69
RECEIVE_TIMEOUT = 5.0  # seconds — generous enough for slow startup

# Rate tolerance: accept ±40% of the nominal interval to avoid flakiness
# (the node loop has no strict scheduler; Windows timer jitter adds to this).
RATE_TOLERANCE = 0.40


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _collect(can_node, msg_type, source_node_id: int, timeout: float, count: int = 1) -> list:
    """
    Spin the node until `count` messages of `msg_type` arrive from
    `source_node_id`, or until `timeout` seconds elapse.

    Returns a list of (monotonic_timestamp, event) tuples.
    """
    received: list = []

    def handler(event) -> None:
        if event.transfer.source_node_id == source_node_id:
            received.append((time.monotonic(), event))

    handle = can_node.add_handler(msg_type, handler)
    deadline = time.monotonic() + timeout
    try:
        while len(received) < count and time.monotonic() < deadline:
            can_node.spin(timeout=0.1)
    finally:
        handle.remove()

    return received


def _get_param(can_node, name: str, timeout: float = 3.0):
    """
    Send a uavcan.protocol.param.GetSet request by name and return the
    response object. Returns None if the node does not respond within
    `timeout` seconds.
    """
    result = [None]
    responded = [False]

    def on_response(event) -> None:
        responded[0] = True
        if event is not None:
            result[0] = event.response

    req = uavcan.protocol.param.GetSet.Request()
    req.name = name
    can_node.request(req, TARGET_NODE_ID, on_response, timeout=timeout)

    # Spin until the dronecan timeout fires (plus a small buffer).
    deadline = time.monotonic() + timeout + 0.5
    while not responded[0] and time.monotonic() < deadline:
        can_node.spin(timeout=0.1)

    return result[0]


def _mean_interval(timestamps: list[float]) -> float:
    """Return mean gap between successive timestamps (seconds)."""
    gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
    return sum(gaps) / len(gaps)


# ---------------------------------------------------------------------------
# NodeStatus — presence and health (smoke tests)
# ---------------------------------------------------------------------------

def test_node_present(can_node):
    """Node must send at least one NodeStatus within RECEIVE_TIMEOUT seconds."""
    events = _collect(
        can_node, uavcan.protocol.NodeStatus, TARGET_NODE_ID, RECEIVE_TIMEOUT
    )
    assert events, (
        f"No NodeStatus received from node {TARGET_NODE_ID} "
        f"within {RECEIVE_TIMEOUT}s. Is the node powered and connected to COM21?"
    )


def test_node_healthy_and_operational(can_node):
    """Node must report health=OK (0) and mode=OPERATIONAL (0)."""
    events = _collect(
        can_node, uavcan.protocol.NodeStatus, TARGET_NODE_ID, RECEIVE_TIMEOUT
    )
    assert events, f"No NodeStatus from node {TARGET_NODE_ID} within {RECEIVE_TIMEOUT}s"

    msg = events[-1][1].message
    assert int(msg.health) == 0, f"Expected health OK (0), got {int(msg.health)}"
    assert int(msg.mode) == 0, f"Expected mode OPERATIONAL (0), got {int(msg.mode)}"


# ---------------------------------------------------------------------------
# NodeStatus rate (~1 Hz)
# ---------------------------------------------------------------------------

def test_node_status_rate(can_node):
    """NodeStatus must arrive at approximately 1 Hz (DroneCAN spec)."""
    n = 4  # 4 messages → 3 intervals
    events = _collect(
        can_node, uavcan.protocol.NodeStatus, TARGET_NODE_ID,
        timeout=n * 2.0, count=n,
    )
    assert len(events) >= n, f"Only got {len(events)}/{n} NodeStatus messages"

    mean = _mean_interval([ts for ts, _ in events])
    nominal = 1.0
    assert abs(mean - nominal) / nominal < RATE_TOLERANCE, (
        f"NodeStatus mean interval {mean:.3f}s is outside "
        f"±{RATE_TOLERANCE * 100:.0f}% of expected {nominal}s"
    )


# ---------------------------------------------------------------------------
# BatteryInfo — presence, rate, and field ranges
# ---------------------------------------------------------------------------

def test_battery_info_received(can_node):
    """Node must broadcast BatteryInfo within RECEIVE_TIMEOUT seconds."""
    events = _collect(
        can_node, uavcan.equipment.power.BatteryInfo, TARGET_NODE_ID, RECEIVE_TIMEOUT
    )
    assert events, (
        f"No BatteryInfo received from node {TARGET_NODE_ID} within {RECEIVE_TIMEOUT}s"
    )


def test_battery_info_rate(can_node):
    """BatteryInfo must arrive at approximately 10 Hz (100 ms interval)."""
    n = 11  # 11 messages → 10 intervals
    events = _collect(
        can_node, uavcan.equipment.power.BatteryInfo, TARGET_NODE_ID,
        timeout=3.0, count=n,
    )
    assert len(events) >= n, f"Only got {len(events)}/{n} BatteryInfo messages in 3 s"

    mean = _mean_interval([ts for ts, _ in events])
    nominal = 0.1  # 10 Hz
    assert abs(mean - nominal) / nominal < RATE_TOLERANCE, (
        f"BatteryInfo mean interval {mean * 1000:.1f} ms is outside "
        f"±{RATE_TOLERANCE * 100:.0f}% of expected {nominal * 1000:.0f} ms"
    )


def test_battery_info_voltage_in_range(can_node):
    """voltage field must be a valid 12-bit ADC reading (0-4095)."""
    events = _collect(
        can_node, uavcan.equipment.power.BatteryInfo, TARGET_NODE_ID, RECEIVE_TIMEOUT
    )
    assert events, f"No BatteryInfo from node {TARGET_NODE_ID}"

    v = float(events[-1][1].message.voltage)
    assert 0.0 <= v <= 4095.0, f"voltage {v} is outside 12-bit ADC range 0-4095"


def test_battery_info_current_in_range(can_node):
    """current field must be a valid 12-bit ADC reading (0-4095)."""
    events = _collect(
        can_node, uavcan.equipment.power.BatteryInfo, TARGET_NODE_ID, RECEIVE_TIMEOUT
    )
    assert events, f"No BatteryInfo from node {TARGET_NODE_ID}"

    i = float(events[-1][1].message.current)
    assert 0.0 <= i <= 4095.0, f"current {i} is outside 12-bit ADC range 0-4095"


def test_battery_info_temperature_plausible(can_node):
    """temperature field must be a plausible MCU die temperature (-40 to 125 °C)."""
    events = _collect(
        can_node, uavcan.equipment.power.BatteryInfo, TARGET_NODE_ID, RECEIVE_TIMEOUT
    )
    assert events, f"No BatteryInfo from node {TARGET_NODE_ID}"

    t = float(events[-1][1].message.temperature)
    assert -40.0 <= t <= 125.0, (
        f"temperature {t} °C is outside plausible MCU range -40..125"
    )


# ---------------------------------------------------------------------------
# Parameters — read back via DroneCAN param service
# ---------------------------------------------------------------------------

def test_param_nodeid(can_node):
    """NODEID parameter must match the node's actual ID on the bus."""
    resp = _get_param(can_node, "NODEID")
    assert resp is not None, "No response to NODEID param request (timeout)"

    value = int(resp.value.integer_value)
    assert value == TARGET_NODE_ID, f"NODEID param = {value}, expected {TARGET_NODE_ID}"


def test_param_parm1(can_node):
    """PARM_1 must be 50.0 — set by dronecan.setParameter() in setup()."""
    resp = _get_param(can_node, "PARM_1")
    assert resp is not None, "No response to PARM_1 param request (timeout)"

    value = float(resp.value.real_value)
    assert abs(value - 50.0) < 0.01, f"PARM_1 = {value}, expected 50.0"


def test_param_parm2(can_node):
    """PARM_2 must be within its configured range (0.0-100.0)."""
    resp = _get_param(can_node, "PARM_2")
    assert resp is not None, "No response to PARM_2 param request (timeout)"

    value = float(resp.value.real_value)
    assert 0.0 <= value <= 100.0, f"PARM_2 = {value} is outside configured range 0-100"
