"""
Tests for DroneCAN node heartbeat and basic connectivity
"""
import pytest
import dronecan
import time
from utils.timing_helpers import MessageWaiter


@pytest.mark.basic
@pytest.mark.requires_hardware
class TestHeartbeat:
    """Test basic node presence and heartbeat functionality"""

    def test_node_is_alive(self, dronecan_node, message_collector, device_node_id, test_config):
        """Verify the device is sending heartbeat messages"""
        # Collect heartbeat messages
        heartbeats = message_collector(dronecan.uavcan.protocol.NodeStatus)

        # Wait for at least one heartbeat
        success = MessageWaiter.wait_for_count(
            heartbeats,
            count=1,
            timeout=test_config['timing']['heartbeat_timeout']
        )

        assert success, (
            f"No heartbeat received from node {device_node_id} within "
            f"{test_config['timing']['heartbeat_timeout']}s. "
            f"Check hardware connection and node ID configuration."
        )

    def test_heartbeat_frequency(self, dronecan_node, message_collector, device_node_id):
        """Verify heartbeat is sent at expected 1Hz rate"""
        heartbeats = message_collector(dronecan.uavcan.protocol.NodeStatus)

        # Collect for 3 seconds (should get ~3 heartbeats)
        time.sleep(3.1)

        assert len(heartbeats) >= 2, "Need at least 2 heartbeats to verify frequency"

        # Calculate actual frequency
        timestamps = [msg['timestamp'] for msg in heartbeats]
        intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
        avg_interval = sum(intervals) / len(intervals)
        actual_hz = 1.0 / avg_interval

        # Should be close to 1Hz (±20%)
        assert 0.8 <= actual_hz <= 1.2, (
            f"Heartbeat frequency {actual_hz:.2f}Hz outside expected range [0.8, 1.2]Hz"
        )

    def test_node_health(self, dronecan_node, message_collector, device_node_id):
        """Verify node reports healthy status"""
        heartbeats = message_collector(dronecan.uavcan.protocol.NodeStatus)

        # Wait for a heartbeat
        MessageWaiter.wait_for_count(heartbeats, count=1, timeout=3.0)

        assert len(heartbeats) > 0, "No heartbeat received"

        # Check health status (0 = OK, 1 = WARNING, 2 = ERROR, 3 = CRITICAL)
        health = heartbeats[-1]['message'].health
        assert health == dronecan.uavcan.protocol.NodeStatus.HEALTH_OK, (
            f"Node health is {health}, expected HEALTH_OK (0)"
        )

    def test_node_mode(self, dronecan_node, message_collector, device_node_id):
        """Verify node is in operational mode"""
        heartbeats = message_collector(dronecan.uavcan.protocol.NodeStatus)

        # Wait for a heartbeat
        MessageWaiter.wait_for_count(heartbeats, count=1, timeout=3.0)

        assert len(heartbeats) > 0, "No heartbeat received"

        # Check mode (0 = OPERATIONAL, 1 = INITIALIZATION, 2 = MAINTENANCE, etc.)
        mode = heartbeats[-1]['message'].mode
        assert mode == dronecan.uavcan.protocol.NodeStatus.MODE_OPERATIONAL, (
            f"Node mode is {mode}, expected MODE_OPERATIONAL (0)"
        )

    def test_uptime_increases(self, dronecan_node, message_collector, device_node_id):
        """Verify node uptime counter increases"""
        heartbeats = message_collector(dronecan.uavcan.protocol.NodeStatus)

        # Get two heartbeats separated by time
        MessageWaiter.wait_for_count(heartbeats, count=1, timeout=3.0)
        first_uptime = heartbeats[-1]['message'].uptime_sec

        time.sleep(2.0)

        MessageWaiter.wait_for_count(heartbeats, count=2, timeout=3.0)
        second_uptime = heartbeats[-1]['message'].uptime_sec

        assert second_uptime > first_uptime, (
            f"Uptime did not increase: {first_uptime} -> {second_uptime}"
        )
        assert second_uptime - first_uptime >= 1, (
            "Uptime increase too small"
        )
