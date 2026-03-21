"""
Hardware-in-the-loop tests for the DroneCAN node running src/main.cpp.

The node under test (node ID 69):
  - Broadcasts NodeStatus at ~1 Hz (DroneCAN mandatory)
  - Broadcasts uavcan.equipment.power.BatteryInfo at 10 Hz
    - voltage  = raw analogRead(PA1)  → 0..4095
    - current  = raw analogRead(PA0)  → 0..4095
    - temperature = CPU temp in °C    → reasonable MCU range
  - Parameters: NODEID (int), PARM_1 (real, 0-100), PARM_2 (real, 0-100)
  - PARM_1 is forced to 50.0 in setup() on every boot
  - Supports GetNodeInfo, param GetSet, ExecuteOpcode, RestartNode

Tests are ordered so the restart test runs LAST (it reboots the node).
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


def _set_param(can_node, name: str, value, timeout: float = 3.0):
    """
    Send a uavcan.protocol.param.GetSet request that sets a new value.
    Returns the response (which contains the actual stored value).
    """
    result = [None]
    responded = [False]

    def on_response(event) -> None:
        responded[0] = True
        if event is not None:
            result[0] = event.response

    req = uavcan.protocol.param.GetSet.Request()
    req.name = name
    # Setting a field on the Value union auto-selects the union tag.
    if isinstance(value, bool):
        req.value.boolean_value = value
    elif isinstance(value, int):
        req.value.integer_value = value
    else:
        req.value.real_value = float(value)

    can_node.request(req, TARGET_NODE_ID, on_response, timeout=timeout)
    deadline = time.monotonic() + timeout + 0.5
    while not responded[0] and time.monotonic() < deadline:
        can_node.spin(timeout=0.1)

    return result[0]


def _get_node_info(can_node, timeout: float = 3.0):
    """Send a GetNodeInfo request and return the response."""
    result = [None]
    responded = [False]

    def on_response(event) -> None:
        responded[0] = True
        if event is not None:
            result[0] = event.response

    req = uavcan.protocol.GetNodeInfo.Request()
    can_node.request(req, TARGET_NODE_ID, on_response, timeout=timeout)
    deadline = time.monotonic() + timeout + 0.5
    while not responded[0] and time.monotonic() < deadline:
        can_node.spin(timeout=0.1)

    return result[0]


def _send_restart(can_node, timeout: float = 3.0):
    """Send a RestartNode request. Returns the response (or None)."""
    result = [None]
    responded = [False]

    def on_response(event) -> None:
        responded[0] = True
        if event is not None:
            result[0] = event.response

    req = uavcan.protocol.RestartNode.Request()
    req.magic_number = 0xACCE551B1E
    can_node.request(req, TARGET_NODE_ID, on_response, timeout=timeout)
    deadline = time.monotonic() + timeout + 0.5
    while not responded[0] and time.monotonic() < deadline:
        can_node.spin(timeout=0.1)

    return result[0]


def _get_param_by_index(can_node, index: int, timeout: float = 3.0):
    """Request a parameter by numeric index (name left empty)."""
    result = [None]
    responded = [False]

    def on_response(event) -> None:
        responded[0] = True
        if event is not None:
            result[0] = event.response

    req = uavcan.protocol.param.GetSet.Request()
    req.index = index
    can_node.request(req, TARGET_NODE_ID, on_response, timeout=timeout)
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


# ---------------------------------------------------------------------------
# GetNodeInfo — identity and metadata
# ---------------------------------------------------------------------------

def test_get_node_info_name(can_node):
    """GetNodeInfo must return the node name set in main.cpp."""
    info = _get_node_info(can_node)
    assert info is not None, "No GetNodeInfo response (timeout)"

    name = bytes(info.name).decode("ascii", errors="replace").rstrip("\x00")
    assert name == "Beyond Robotix Node", f"Node name = '{name}'"


def test_get_node_info_unique_id(can_node):
    """GetNodeInfo must return a non-zero hardware unique ID."""
    info = _get_node_info(can_node)
    assert info is not None, "No GetNodeInfo response (timeout)"

    uid = bytes(info.hardware_version.unique_id)
    assert uid != b"\x00" * 16, "Hardware unique ID is all zeros"


def test_get_node_info_uptime(can_node):
    """GetNodeInfo uptime must be a sane positive value."""
    info = _get_node_info(can_node)
    assert info is not None, "No GetNodeInfo response (timeout)"

    assert info.status.uptime_sec > 0, "Uptime is 0 — node may have just booted"


# ---------------------------------------------------------------------------
# Parameters — set, clamping, index lookup, nonexistent
# ---------------------------------------------------------------------------

def test_param_set_readback(can_node):
    """Setting PARM_2 over CAN must read back the new value."""
    # Save original
    original = _get_param(can_node, "PARM_2")
    assert original is not None
    orig_val = float(original.value.real_value)

    try:
        # Set to a distinctive value
        resp = _set_param(can_node, "PARM_2", 42.0)
        assert resp is not None, "No response to PARM_2 set request"
        assert abs(float(resp.value.real_value) - 42.0) < 0.01, (
            f"Set response returned {float(resp.value.real_value)}, expected 42.0"
        )

        # Independent read-back
        check = _get_param(can_node, "PARM_2")
        assert check is not None
        assert abs(float(check.value.real_value) - 42.0) < 0.01, (
            f"Read-back returned {float(check.value.real_value)}, expected 42.0"
        )
    finally:
        # Restore original value
        _set_param(can_node, "PARM_2", orig_val)


def test_param_clamping_high(can_node):
    """Setting PARM_2 above max (100) must clamp to 100."""
    original = _get_param(can_node, "PARM_2")
    assert original is not None
    orig_val = float(original.value.real_value)

    try:
        resp = _set_param(can_node, "PARM_2", 999.0)
        assert resp is not None, "No response to PARM_2 set request"
        assert abs(float(resp.value.real_value) - 100.0) < 0.01, (
            f"Expected clamped value 100.0, got {float(resp.value.real_value)}"
        )
    finally:
        _set_param(can_node, "PARM_2", orig_val)


def test_param_clamping_low(can_node):
    """Setting PARM_2 below min (0) must clamp to 0."""
    original = _get_param(can_node, "PARM_2")
    assert original is not None
    orig_val = float(original.value.real_value)

    try:
        resp = _set_param(can_node, "PARM_2", -50.0)
        assert resp is not None, "No response to PARM_2 set request"
        assert abs(float(resp.value.real_value)) < 0.01, (
            f"Expected clamped value 0.0, got {float(resp.value.real_value)}"
        )
    finally:
        _set_param(can_node, "PARM_2", orig_val)


def test_param_by_index(can_node):
    """Requesting parameter by index 0 must return NODEID."""
    resp = _get_param_by_index(can_node, 0)
    assert resp is not None, "No response to index-0 param request"

    name = bytes(resp.name).decode("ascii", errors="replace").rstrip("\x00")
    assert name == "NODEID", f"Index 0 name = '{name}', expected 'NODEID'"


def test_param_nonexistent_name_falls_back_to_index(can_node):
    """When name lookup fails, the firmware falls back to the index field.
    With index=0 (default), this returns NODEID (parameter at index 0)."""
    resp = _get_param(can_node, "NO_SUCH_PARAM")
    assert resp is not None, "No response to nonexistent param request"

    name = bytes(resp.name).decode("ascii", errors="replace").rstrip("\x00")
    assert name == "NODEID", (
        f"Expected index-0 fallback to 'NODEID', got '{name}'"
    )


def test_param_out_of_range_index(can_node):
    """Requesting an out-of-range index with no valid name returns empty."""
    result = [None]
    responded = [False]

    def on_response(event) -> None:
        responded[0] = True
        if event is not None:
            result[0] = event.response

    req = uavcan.protocol.param.GetSet.Request()
    req.name = "NO_SUCH_PARAM"
    req.index = 999  # way beyond the 3 defined parameters
    can_node.request(req, TARGET_NODE_ID, on_response, timeout=3.0)

    deadline = time.monotonic() + 3.5
    while not responded[0] and time.monotonic() < deadline:
        can_node.spin(timeout=0.1)

    assert result[0] is not None, "No response (timeout)"
    name = bytes(result[0].name).decode("ascii", errors="replace").rstrip("\x00")
    assert name == "", f"Expected empty name for out-of-range param, got '{name}'"


# ---------------------------------------------------------------------------
# RestartNode — reboot, DNA re-allocation, parameter persistence
#
# This test MUST run last because it reboots the node.
# ---------------------------------------------------------------------------

def test_restart_node(can_node):
    """RestartNode must reboot the node; it must come back via DNA
    with its parameters intact and uptime reset to near zero."""
    # 1. Record pre-restart uptime
    pre = _collect(
        can_node, uavcan.protocol.NodeStatus, TARGET_NODE_ID, RECEIVE_TIMEOUT
    )
    assert pre, "Cannot read NodeStatus before restart"
    pre_uptime = pre[-1][1].message.uptime_sec

    # 2. Set PARM_2 to a distinctive value so we can check persistence
    _set_param(can_node, "PARM_2", 77.0)

    # 3. Send RestartNode
    _send_restart(can_node)

    # 4. Give the node time to reset and complete DNA (~2-5 s).
    #    Spin the bus so the DNA server in conftest can respond.
    time.sleep(1)  # let the MCU actually reset
    deadline = time.monotonic() + 15.0
    post_events: list = []

    def on_status(event) -> None:
        if event.transfer.source_node_id == TARGET_NODE_ID:
            post_events.append(event)

    handle = can_node.add_handler(uavcan.protocol.NodeStatus, on_status)
    try:
        while not post_events and time.monotonic() < deadline:
            can_node.spin(timeout=0.5)
    finally:
        handle.remove()

    assert post_events, (
        "Node did not reappear after RestartNode within 15 s. "
        "Is the DNA server running?"
    )

    # 5. Verify uptime reset (should be much lower than before)
    new_uptime = post_events[-1].message.uptime_sec
    assert new_uptime < pre_uptime, (
        f"Uptime after restart ({new_uptime}s) >= before ({pre_uptime}s) "
        "-- node may not have actually restarted"
    )

    # 6. Verify parameters survived the reboot
    #    PARM_1 is forced to 50.0 by setup() on every boot.
    p1 = _get_param(can_node, "PARM_1")
    assert p1 is not None, "No PARM_1 response after restart"
    assert abs(float(p1.value.real_value) - 50.0) < 0.01, (
        f"PARM_1 after restart = {float(p1.value.real_value)}, expected 50.0"
    )

    #    PARM_2 should still be 77.0 (persisted to EEPROM before restart).
    p2 = _get_param(can_node, "PARM_2")
    assert p2 is not None, "No PARM_2 response after restart"
    assert abs(float(p2.value.real_value) - 77.0) < 0.01, (
        f"PARM_2 after restart = {float(p2.value.real_value)}, expected 77.0"
    )
