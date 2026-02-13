"""
Tests for BatteryInfo message transmission
"""
import pytest
import dronecan
import time
from utils.message_validator import BatteryInfoValidator, MessageValidator
from utils.timing_helpers import MessageWaiter


@pytest.mark.messaging
@pytest.mark.requires_hardware
class TestBatteryMessages:
    """Test BatteryInfo message transmission and content"""

    @pytest.fixture(autouse=True)
    def setup(self, test_config):
        """Setup battery message validator"""
        msg_config = test_config['messages']['battery_info']
        self.validator = BatteryInfoValidator(
            voltage_range=msg_config['voltage_range'],
            current_range=msg_config['current_range'],
            temp_range=msg_config['temp_range']
        )
        self.expected_frequency = msg_config['expected_frequency']
        self.frequency_tolerance = msg_config['frequency_tolerance']

    def test_battery_messages_transmitted(self, message_collector):
        """Verify BatteryInfo messages are being transmitted"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        # Should receive at least one message quickly
        success = MessageWaiter.wait_for_count(battery_msgs, count=1, timeout=1.0)

        assert success, (
            "No BatteryInfo messages received within 1 second. "
            "Check that the device is sending messages."
        )

    @pytest.mark.basic
    def test_battery_message_frequency(self, message_collector, test_config):
        """Verify BatteryInfo is sent at expected 10Hz rate"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        # Collect messages for configured duration
        collection_time = test_config['timing']['message_collection_time']
        count = MessageWaiter.collect_for_duration(battery_msgs, collection_time)

        assert count >= 2, f"Need at least 2 messages, got {count}"

        # Validate frequency
        valid, error, actual_hz = MessageValidator.validate_frequency(
            battery_msgs,
            self.expected_frequency,
            self.frequency_tolerance
        )

        assert valid, (
            f"Battery message frequency validation failed: {error}. "
            f"Expected {self.expected_frequency}Hz ±{self.frequency_tolerance*100}%, "
            f"got {actual_hz:.2f}Hz"
        )

    def test_battery_voltage_range(self, message_collector):
        """Verify voltage readings are within expected ADC range"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        # Collect some messages
        MessageWaiter.collect_for_duration(battery_msgs, 0.5)

        assert len(battery_msgs) > 0, "No messages received"

        # Check all messages
        for msg_data in battery_msgs:
            msg = msg_data['message']
            valid, errors = self.validator.validate_message(msg)

            # Check voltage specifically
            voltage_valid, voltage_error = MessageValidator.validate_value_range(
                msg.voltage,
                self.validator.voltage_range[0],
                self.validator.voltage_range[1],
                "Voltage"
            )

            assert voltage_valid, voltage_error

    def test_battery_current_range(self, message_collector):
        """Verify current readings are within expected ADC range"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        MessageWaiter.collect_for_duration(battery_msgs, 0.5)

        assert len(battery_msgs) > 0, "No messages received"

        for msg_data in battery_msgs:
            msg = msg_data['message']

            current_valid, current_error = MessageValidator.validate_value_range(
                msg.current,
                self.validator.current_range[0],
                self.validator.current_range[1],
                "Current"
            )

            assert current_valid, current_error

    def test_cpu_temperature_range(self, message_collector):
        """Verify CPU temperature is within valid STM32 range"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        MessageWaiter.collect_for_duration(battery_msgs, 0.5)

        assert len(battery_msgs) > 0, "No messages received"

        for msg_data in battery_msgs:
            msg = msg_data['message']

            temp_valid, temp_error = MessageValidator.validate_value_range(
                msg.temperature,
                self.validator.temp_range[0],
                self.validator.temp_range[1],
                "Temperature"
            )

            assert temp_valid, temp_error

    def test_cpu_temperature_realistic(self, message_collector):
        """Verify CPU temperature is in realistic range for operation"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        MessageWaiter.collect_for_duration(battery_msgs, 0.5)

        assert len(battery_msgs) > 0, "No messages received"

        # CPU should be between 0-80°C during normal operation
        temp = battery_msgs[-1]['message'].temperature

        assert 0 <= temp <= 80, (
            f"CPU temperature {temp}°C seems unrealistic. "
            f"Expected between 0-80°C for normal operation."
        )

    def test_no_message_gaps(self, message_collector):
        """Verify there are no large gaps in message transmission"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        # Collect for 2 seconds
        MessageWaiter.collect_for_duration(battery_msgs, 2.0)

        assert len(battery_msgs) >= 10, "Need sufficient messages to check for gaps"

        # Check for gaps larger than 200ms (at 10Hz, interval is 100ms)
        valid, error = MessageValidator.validate_message_sequence(
            battery_msgs,
            max_gap=0.2
        )

        assert valid, error

    @pytest.mark.slow
    def test_message_timing_statistics(self, message_collector):
        """Calculate and verify message timing statistics"""
        battery_msgs = message_collector(dronecan.uavcan.equipment.power.BatteryInfo)

        # Collect for longer period
        MessageWaiter.collect_for_duration(battery_msgs, 3.0)

        stats = MessageValidator.calculate_message_stats(battery_msgs)

        # Verify we got enough messages
        assert stats['count'] >= 20, f"Only got {stats['count']} messages in 3 seconds"

        # Verify frequency is close to 10Hz
        assert 9.0 <= stats['frequency_hz'] <= 11.0, (
            f"Frequency {stats['frequency_hz']:.2f}Hz outside expected range"
        )

        # Verify jitter is reasonable (should be < 10ms for a well-timed loop)
        assert stats['jitter_ms'] < 20, (
            f"Jitter {stats['jitter_ms']:.2f}ms too high. "
            f"Check for timing issues in firmware."
        )

        # Print stats for information
        print(f"\nMessage timing statistics:")
        print(f"  Count: {stats['count']}")
        print(f"  Frequency: {stats['frequency_hz']:.2f} Hz")
        print(f"  Mean interval: {stats['mean_interval_ms']:.2f} ms")
        print(f"  Jitter: {stats['jitter_ms']:.2f} ms")
        print(f"  Min interval: {stats['min_interval_ms']:.2f} ms")
        print(f"  Max interval: {stats['max_interval_ms']:.2f} ms")
