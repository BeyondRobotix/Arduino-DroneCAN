"""
Message validation utilities for DroneCAN testing
"""
from typing import List, Dict, Any, Tuple
import statistics


class MessageValidator:
    """Validates DroneCAN messages against expected criteria"""

    @staticmethod
    def validate_frequency(messages: List[Dict], expected_hz: float, tolerance: float = 0.2) -> Tuple[bool, str, float]:
        """
        Validate message frequency against expected rate.

        Args:
            messages: List of message dicts with 'timestamp' keys
            expected_hz: Expected frequency in Hz
            tolerance: Allowed deviation (0.2 = ±20%)

        Returns:
            Tuple of (is_valid, error_message, actual_frequency)
        """
        if len(messages) < 2:
            return False, "Need at least 2 messages to calculate frequency", 0.0

        # Calculate time differences between consecutive messages
        timestamps = [msg['timestamp'] for msg in messages]
        intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]

        # Calculate actual frequency from mean interval
        mean_interval = statistics.mean(intervals)
        actual_hz = 1.0 / mean_interval if mean_interval > 0 else 0.0

        # Check if within tolerance
        min_hz = expected_hz * (1 - tolerance)
        max_hz = expected_hz * (1 + tolerance)

        is_valid = min_hz <= actual_hz <= max_hz

        if not is_valid:
            error_msg = f"Frequency {actual_hz:.2f}Hz outside range [{min_hz:.2f}, {max_hz:.2f}]Hz"
        else:
            error_msg = ""

        return is_valid, error_msg, actual_hz

    @staticmethod
    def validate_value_range(value: float, min_val: float, max_val: float, field_name: str = "Value") -> Tuple[bool, str]:
        """
        Validate a value is within expected range.

        Args:
            value: Value to check
            min_val: Minimum allowed value
            max_val: Maximum allowed value
            field_name: Name of field for error message

        Returns:
            Tuple of (is_valid, error_message)
        """
        is_valid = min_val <= value <= max_val

        if not is_valid:
            error_msg = f"{field_name} {value} outside range [{min_val}, {max_val}]"
        else:
            error_msg = ""

        return is_valid, error_msg

    @staticmethod
    def validate_message_sequence(messages: List[Dict], max_gap: float = 1.0) -> Tuple[bool, str]:
        """
        Validate there are no large gaps in message sequence.

        Args:
            messages: List of message dicts with 'timestamp' keys
            max_gap: Maximum allowed gap between messages in seconds

        Returns:
            Tuple of (is_valid, error_message)
        """
        if len(messages) < 2:
            return True, ""

        timestamps = [msg['timestamp'] for msg in messages]
        intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]

        max_interval = max(intervals)
        if max_interval > max_gap:
            return False, f"Found gap of {max_interval:.3f}s (max allowed: {max_gap}s)"

        return True, ""

    @staticmethod
    def calculate_message_stats(messages: List[Dict]) -> Dict[str, Any]:
        """
        Calculate statistics about message timing.

        Args:
            messages: List of message dicts with 'timestamp' keys

        Returns:
            Dict with statistics (count, frequency, jitter, etc.)
        """
        if len(messages) < 2:
            return {
                'count': len(messages),
                'frequency_hz': 0.0,
                'mean_interval_ms': 0.0,
                'jitter_ms': 0.0
            }

        timestamps = [msg['timestamp'] for msg in messages]
        intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]

        mean_interval = statistics.mean(intervals)
        frequency = 1.0 / mean_interval if mean_interval > 0 else 0.0

        # Calculate jitter (standard deviation of intervals)
        jitter = statistics.stdev(intervals) if len(intervals) > 1 else 0.0

        return {
            'count': len(messages),
            'frequency_hz': frequency,
            'mean_interval_ms': mean_interval * 1000,
            'jitter_ms': jitter * 1000,
            'min_interval_ms': min(intervals) * 1000,
            'max_interval_ms': max(intervals) * 1000
        }


class BatteryInfoValidator(MessageValidator):
    """Specialized validator for BatteryInfo messages"""

    def __init__(self, voltage_range: List[float], current_range: List[float], temp_range: List[float]):
        self.voltage_range = voltage_range
        self.current_range = current_range
        self.temp_range = temp_range

    def validate_message(self, message) -> Tuple[bool, List[str]]:
        """
        Validate a single BatteryInfo message.

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        # Validate voltage
        valid, error = self.validate_value_range(
            message.voltage,
            self.voltage_range[0],
            self.voltage_range[1],
            "Voltage"
        )
        if not valid:
            errors.append(error)

        # Validate current
        valid, error = self.validate_value_range(
            message.current,
            self.current_range[0],
            self.current_range[1],
            "Current"
        )
        if not valid:
            errors.append(error)

        # Validate temperature
        valid, error = self.validate_value_range(
            message.temperature,
            self.temp_range[0],
            self.temp_range[1],
            "Temperature"
        )
        if not valid:
            errors.append(error)

        return len(errors) == 0, errors
