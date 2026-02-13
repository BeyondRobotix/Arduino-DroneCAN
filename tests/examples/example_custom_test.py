"""
Example: How to create a custom test for your Arduino-DroneCAN device

This example shows how to:
1. Create a new test file
2. Use fixtures
3. Test custom messages
4. Validate custom behavior
"""

import pytest
import dronecan
import time
from utils.timing_helpers import MessageWaiter
from utils.message_validator import MessageValidator


@pytest.mark.requires_hardware
class TestCustomMessage:
    """
    Example test class for a custom DroneCAN message.

    Suppose your device sends a custom message at 5Hz.
    Here's how you'd test it:
    """

    def test_custom_message_transmitted(self, message_collector):
        """Verify custom message is being sent"""
        # Replace with your actual message type
        # Example: dronecan.uavcan.equipment.device.Temperature
        custom_msgs = message_collector(dronecan.uavcan.equipment.device.Temperature)

        # Wait for at least one message
        success = MessageWaiter.wait_for_count(custom_msgs, count=1, timeout=2.0)

        assert success, "No custom messages received within timeout"

    def test_custom_message_frequency(self, message_collector):
        """Verify custom message frequency"""
        # Collect messages
        custom_msgs = message_collector(dronecan.uavcan.equipment.device.Temperature)

        # Collect for 1.5 seconds
        time.sleep(1.5)

        assert len(custom_msgs) >= 5, f"Expected at least 5 messages, got {len(custom_msgs)}"

        # Validate frequency (expecting 5Hz ±20%)
        valid, error, actual_hz = MessageValidator.validate_frequency(
            custom_msgs,
            expected_hz=5.0,
            tolerance=0.2
        )

        assert valid, f"Frequency check failed: {error}"

    def test_custom_message_content(self, message_collector):
        """Verify custom message content is valid"""
        custom_msgs = message_collector(dronecan.uavcan.equipment.device.Temperature)

        MessageWaiter.wait_for_count(custom_msgs, count=1, timeout=2.0)

        # Get latest message
        msg = custom_msgs[-1]['message']

        # Validate content - example for Temperature message
        assert -40 <= msg.temperature <= 125, (
            f"Temperature {msg.temperature}°C outside valid range"
        )

        # You can also check device_id, error_flags, etc.
        # depending on your message structure


@pytest.mark.requires_hardware
class TestCustomParameter:
    """
    Example test class for testing a custom parameter.

    Shows how to test custom parameters beyond the default ones.
    """

    def test_custom_parameter_exists(self, parameter_manager):
        """Verify custom parameter exists on device"""
        # Replace 'MY_CUSTOM_PARAM' with your actual parameter name
        value = parameter_manager['get']('PARM_1')

        assert value is not None, "Custom parameter not found"

    def test_custom_parameter_range(self, parameter_manager):
        """Test custom parameter accepts valid range"""
        param_name = 'PARM_1'

        # Test minimum value
        success = parameter_manager['set'](param_name, 0.0)
        assert success, "Failed to set minimum value"

        time.sleep(0.1)
        value = parameter_manager['get'](param_name)
        assert abs(value.real_value - 0.0) < 0.01

        # Test maximum value
        success = parameter_manager['set'](param_name, 100.0)
        assert success, "Failed to set maximum value"

        time.sleep(0.1)
        value = parameter_manager['get'](param_name)
        assert abs(value.real_value - 100.0) < 0.01

    def test_custom_parameter_affects_behavior(self, parameter_manager, message_collector):
        """
        Example: Test that changing a parameter affects device behavior

        This is more advanced - it shows how to verify that a parameter
        actually changes the device's operation.
        """
        # Suppose PARM_1 affects some message rate or content

        # Set parameter to known value
        parameter_manager['set']('PARM_1', 50.0)
        time.sleep(0.2)

        # Collect messages and verify expected behavior
        msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)
        time.sleep(1.0)

        # Example: Verify something about the messages based on parameter value
        # This is device-specific - you'd implement your actual check here
        assert len(msgs) > 0, "No messages received after parameter change"


@pytest.mark.requires_hardware
class TestCustomCommandResponse:
    """
    Example: Testing command/response pattern

    Shows how to send a command to the device and verify response.
    """

    def test_device_responds_to_command(self, dronecan_node, message_collector, device_node_id):
        """Send a command and verify device responds"""
        # Example: Send an actuator command
        command_msgs = message_collector(dronecan.uavcan.equipment.actuator.Status)

        # Send command
        cmd = dronecan.uavcan.equipment.actuator.ArrayCommand()
        cmd.commands = [
            dronecan.uavcan.equipment.actuator.Command(
                actuator_id=0,
                command_type=0,
                command_value=0.5
            )
        ]

        dronecan_node.broadcast(cmd)

        # Wait for response (device might send Status message)
        success = MessageWaiter.wait_for_count(command_msgs, count=1, timeout=1.0)

        # Depending on your device, it might respond or not
        # This is just an example of the pattern


# Example of using custom markers
@pytest.mark.slow
@pytest.mark.requires_hardware
def test_long_running_stability(message_collector):
    """
    Example of a long-running test marked as 'slow'

    Run with: pytest -m slow
    Skip with: pytest -m "not slow"
    """
    msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

    # Run for 30 seconds
    time.sleep(30.0)

    # Verify consistent operation
    assert len(msgs) > 250, "Expected ~300 messages in 30s at 10Hz"

    # Check for any gaps in transmission
    valid, error = MessageValidator.validate_message_sequence(msgs, max_gap=0.5)
    assert valid, f"Found gaps in message stream: {error}"


# Example of parametrized tests
@pytest.mark.requires_hardware
@pytest.mark.parametrize("test_value,expected_clamped", [
    (-10.0, 0.0),      # Below minimum, should clamp to 0
    (50.0, 50.0),      # Valid value
    (150.0, 100.0),    # Above maximum, should clamp to 100
])
def test_parameter_clamping(parameter_manager, test_value, expected_clamped):
    """
    Example: Parametrized test for parameter value clamping

    Tests multiple values in a single test function.
    """
    parameter_manager['set']('PARM_1', test_value)
    time.sleep(0.1)

    value = parameter_manager['get']('PARM_1')
    actual = value.real_value

    assert abs(actual - expected_clamped) < 0.01, (
        f"Expected {test_value} to be clamped to {expected_clamped}, got {actual}"
    )


if __name__ == "__main__":
    # This allows running the test file directly
    # Usage: uv run python examples/example_custom_test.py
    pytest.main([__file__, "-v"])
