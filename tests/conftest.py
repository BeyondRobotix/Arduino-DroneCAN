"""
Pytest configuration and shared fixtures for Arduino-DroneCAN tests
"""
import pytest
import dronecan
import yaml
import time
from pathlib import Path
from typing import Dict, Any
from collections import deque


def pytest_configure(config):
    """Configure pytest with custom markers and settings"""
    config.addinivalue_line(
        "markers", "requires_hardware: mark test as requiring physical hardware"
    )


@pytest.fixture(scope="session")
def test_config() -> Dict[str, Any]:
    """Load test configuration from YAML file"""
    config_path = Path(__file__).parent / "test_config.yaml"
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


@pytest.fixture(scope="session")
def dronecan_node(test_config):
    """
    Create DroneCAN node connected to hardware.
    This is session-scoped so the connection persists across all tests.
    """
    can_config = test_config['can_interface']
    test_node_config = test_config['test_node']

    interface_str = can_config['interface']
    bitrate = can_config['bitrate']

    # Extract serial baudrate if specified in interface string (e.g., "slcan:COM7@115200")
    serial_baudrate = None
    if '@' in interface_str:
        interface_str, serial_baudrate = interface_str.rsplit('@', 1)
        serial_baudrate = int(serial_baudrate)

    try:
        print(f"\n[Setup] Connecting to CAN interface: {interface_str}")
        if serial_baudrate:
            print(f"[Setup] Serial baud rate: {serial_baudrate} bps")
        print(f"[Setup] CAN bitrate: {bitrate} bps")

        kwargs = {
            'node_id': test_node_config['node_id'],
            'bitrate': bitrate
        }
        if serial_baudrate:
            kwargs['baudrate'] = serial_baudrate

        node = dronecan.make_node(interface_str, **kwargs)

        # Start the node in background mode using a thread
        import threading
        stop_event = threading.Event()

        def node_worker():
            while not stop_event.is_set():
                try:
                    node.process(timeout=0.1)
                except Exception:
                    pass

        thread = threading.Thread(target=node_worker, daemon=True)
        thread.start()
        print("[Setup] Node processing thread started")

        # Give it a moment to start
        time.sleep(0.5)

        # Start node monitor to detect devices
        print("[Setup] Starting node monitor...")
        node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

        # Wait for device to appear
        print("[Setup] Waiting for device (2 seconds)...")
        time.sleep(2.0)

        nodes = list(node_monitor.get_all_node_id())
        if nodes:
            print(f"[Setup] Found nodes: {nodes}")
        else:
            print("[Setup] Warning: No nodes detected yet")

        print("[Setup] DroneCAN node connected successfully")

        yield node

        print("\n[Teardown] Closing DroneCAN connection")
        node.close()

    except Exception as e:
        pytest.fail(f"Failed to connect to CAN interface: {e}\n"
                   f"Check that your CAN adapter is connected and the interface string is correct.")


@pytest.fixture(scope="function")
def message_collector(dronecan_node):
    """
    Factory fixture that creates message collectors for specific message types.
    Returns a function that can be called to create collectors.
    """
    collectors = []

    def create_collector(message_type, max_messages=100):
        """Create a message collector for a specific message type"""
        messages = deque(maxlen=max_messages)

        def callback(event):
            messages.append({
                'timestamp': time.time(),
                'message': event.message,
                'transfer': event.transfer
            })

        # Subscribe to message type
        handle = dronecan_node.add_handler(message_type, callback)

        collector = {
            'messages': messages,
            'callback': callback,
            'handle': handle,
            'type': message_type
        }
        collectors.append(collector)

        return messages

    yield create_collector

    # Cleanup: remove all handlers
    for collector in collectors:
        try:
            dronecan_node.remove_handler(collector['handle'])
        except AttributeError:
            # Handler cleanup not needed or already done
            pass


@pytest.fixture
def device_node_id(test_config):
    """Get the device under test node ID"""
    return test_config['device']['node_id']


@pytest.fixture
def parameter_manager(dronecan_node, device_node_id, test_config):
    """
    Fixture for managing parameters with automatic cleanup.
    Saves parameters at start and optionally restores them after test.
    """
    original_params = {}

    # Save original parameter values
    if test_config['test_settings']['restore_parameters']:
        for param_name in test_config['parameters']['expected_params']:
            try:
                response = dronecan_node.request(
                    dronecan.uavcan.protocol.param.GetSet.Request(name=param_name),
                    device_node_id,
                    timeout=2.0
                )
                if response:
                    original_params[param_name] = response[0].value
            except Exception:
                pass  # Parameter might not exist yet

    def get_param(name: str):
        """Get a parameter value from the device"""
        response = dronecan_node.request(
            dronecan.uavcan.protocol.param.GetSet.Request(name=name),
            device_node_id,
            timeout=test_config['timing']['parameter_response_timeout']
        )
        if response:
            return response[0].value
        return None

    def set_param(name: str, value):
        """Set a parameter value on the device"""
        # Create value object based on type
        if isinstance(value, bool):
            param_value = dronecan.uavcan.protocol.param.Value(boolean_value=value)
        elif isinstance(value, int):
            param_value = dronecan.uavcan.protocol.param.Value(integer_value=value)
        elif isinstance(value, float):
            param_value = dronecan.uavcan.protocol.param.Value(real_value=value)
        else:
            raise ValueError(f"Unsupported parameter type: {type(value)}")

        response = dronecan_node.request(
            dronecan.uavcan.protocol.param.GetSet.Request(
                name=name,
                value=param_value
            ),
            device_node_id,
            timeout=test_config['timing']['parameter_response_timeout']
        )
        return response is not None

    manager = {
        'get': get_param,
        'set': set_param,
        'original': original_params
    }

    yield manager

    # Restore original parameters if configured
    if test_config['test_settings']['restore_parameters']:
        for param_name, value in original_params.items():
            try:
                dronecan_node.request(
                    dronecan.uavcan.protocol.param.GetSet.Request(
                        name=param_name,
                        value=value
                    ),
                    device_node_id,
                    timeout=1.0
                )
            except Exception:
                pass  # Best effort restore


@pytest.fixture
def wait_for_messages():
    """Helper to wait for a specific number of messages"""
    def _wait(message_queue, count=1, timeout=2.0):
        start = time.time()
        while len(message_queue) < count:
            if time.time() - start > timeout:
                raise TimeoutError(
                    f"Timeout waiting for {count} messages. "
                    f"Only received {len(message_queue)}"
                )
            time.sleep(0.01)
        return True
    return _wait
