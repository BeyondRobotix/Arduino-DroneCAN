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

    interface_str = f"{can_config['interface']}:{can_config['bitrate']}"

    try:
        print(f"\n[Setup] Connecting to CAN interface: {interface_str}")
        node = dronecan.make_node(
            interface_str,
            node_id=test_node_config['node_id'],
            bitrate=can_config['bitrate']
        )

        # Start the node monitor thread
        node.spin()

        # Wait a moment for connection to stabilize
        time.sleep(0.5)

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
        dronecan_node.remove_handler(collector['handle'])


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
