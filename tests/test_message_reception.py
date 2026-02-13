"""
Tests for device message reception capabilities
"""
import pytest
import dronecan
import time
from utils.timing_helpers import MessageWaiter


@pytest.mark.messaging
@pytest.mark.requires_hardware
class TestMessageReception:
    """Test that device can receive and process incoming messages"""

    def test_device_accepts_magnetic_field_message(self, dronecan_node, message_collector):
        """Test sending MagneticFieldStrength message to device"""
        # Create test message
        msg = dronecan.uavcan.equipment.ahrs.MagneticFieldStrength()
        msg.magnetic_field_ga = [1.0, 2.0, 3.0]

        # Send message
        dronecan_node.broadcast(msg)

        # Device should process this without crashing
        # Verify by checking that normal operation continues
        time.sleep(0.2)

        # Check that device is still sending battery messages
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)
        success = MessageWaiter.wait_for_count(battery_msgs, count=1, timeout=1.0)

        assert success, (
            "Device stopped responding after receiving MagneticFieldStrength message. "
            "Check message reception handler."
        )

    def test_device_handles_multiple_messages(self, dronecan_node, message_collector):
        """Test device handles multiple incoming messages"""
        # Send several different message types
        for i in range(5):
            msg = dronecan.uavcan.equipment.ahrs.MagneticFieldStrength()
            msg.magnetic_field_ga = [float(i), float(i+1), float(i+2)]
            dronecan_node.broadcast(msg)
            time.sleep(0.05)

        # Device should still be operational
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)
        success = MessageWaiter.wait_for_count(battery_msgs, count=3, timeout=1.0)

        assert success, "Device stopped responding after multiple messages"

    def test_device_ignores_unhandled_messages(self, dronecan_node, message_collector):
        """Test device properly ignores messages it doesn't handle"""
        # Send a message type that device doesn't subscribe to
        # (assuming device doesn't handle RawIMU based on shouldAcceptTransfer)
        msg = dronecan.uavcan.equipment.ahrs.RawIMU()
        msg.integration_interval = 0.01

        # Send message
        dronecan_node.broadcast(msg)
        time.sleep(0.1)

        # Device should continue normal operation
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)
        success = MessageWaiter.wait_for_count(battery_msgs, count=1, timeout=1.0)

        assert success, "Device affected by unhandled message type"

    @pytest.mark.slow
    def test_message_reception_doesnt_affect_timing(self, dronecan_node, message_collector):
        """Verify receiving messages doesn't disrupt transmission timing"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        # Collect baseline timing
        MessageWaiter.collect_for_duration(battery_msgs, 1.0)
        baseline_count = len(battery_msgs)

        # Now send messages while collecting
        battery_msgs.clear()
        start_time = time.time()

        while time.time() - start_time < 1.0:
            msg = dronecan.uavcan.equipment.ahrs.MagneticFieldStrength()
            msg.magnetic_field_ga = [1.0, 2.0, 3.0]
            dronecan_node.broadcast(msg)
            time.sleep(0.05)  # Send at 20Hz

        with_messages_count = len(battery_msgs)

        # Should get similar number of messages (±20%)
        assert abs(with_messages_count - baseline_count) / baseline_count < 0.2, (
            f"Message reception affected timing: baseline={baseline_count}, "
            f"with messages={with_messages_count}"
        )
