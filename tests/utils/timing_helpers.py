"""
Timing and synchronization helpers for hardware tests
"""
import time
from typing import Callable, Any, Optional
from collections import deque


class MessageWaiter:
    """Helper for waiting on messages with timeout"""

    @staticmethod
    def wait_for_count(message_queue: deque, count: int, timeout: float = 2.0) -> bool:
        """
        Wait until queue has at least 'count' messages.

        Args:
            message_queue: Deque to monitor
            count: Number of messages to wait for
            timeout: Maximum time to wait in seconds

        Returns:
            True if messages received, False if timeout
        """
        start = time.time()
        while len(message_queue) < count:
            if time.time() - start > timeout:
                return False
            time.sleep(0.01)
        return True

    @staticmethod
    def wait_for_condition(condition: Callable[[], bool], timeout: float = 2.0, poll_interval: float = 0.01) -> bool:
        """
        Wait until a condition becomes true.

        Args:
            condition: Callable that returns bool
            timeout: Maximum time to wait in seconds
            poll_interval: How often to check condition

        Returns:
            True if condition met, False if timeout
        """
        start = time.time()
        while not condition():
            if time.time() - start > timeout:
                return False
            time.sleep(poll_interval)
        return True

    @staticmethod
    def collect_for_duration(message_queue: deque, duration: float) -> int:
        """
        Clear queue and collect messages for a specific duration.

        Args:
            message_queue: Deque to collect into
            duration: How long to collect in seconds

        Returns:
            Number of messages collected
        """
        message_queue.clear()
        time.sleep(duration)
        return len(message_queue)


class PerformanceTimer:
    """Context manager for timing operations"""

    def __init__(self, name: str = "Operation"):
        self.name = name
        self.start_time = None
        self.elapsed = None

    def __enter__(self):
        self.start_time = time.time()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.elapsed = time.time() - self.start_time

    def get_elapsed_ms(self) -> float:
        """Get elapsed time in milliseconds"""
        return self.elapsed * 1000 if self.elapsed else 0.0


class RateLimiter:
    """Rate limiter for controlling test message transmission"""

    def __init__(self, rate_hz: float):
        """
        Args:
            rate_hz: Target rate in Hz
        """
        self.interval = 1.0 / rate_hz
        self.last_time = 0.0

    def wait(self):
        """Wait until next interval"""
        now = time.time()
        elapsed = now - self.last_time

        if elapsed < self.interval:
            time.sleep(self.interval - elapsed)

        self.last_time = time.time()


def retry_on_failure(func: Callable, max_attempts: int = 3, delay: float = 0.5) -> Optional[Any]:
    """
    Retry a function on failure.

    Args:
        func: Function to call
        max_attempts: Maximum number of attempts
        delay: Delay between attempts in seconds

    Returns:
        Function result or None if all attempts fail
    """
    for attempt in range(max_attempts):
        try:
            return func()
        except Exception as e:
            if attempt == max_attempts - 1:
                raise
            time.sleep(delay)
    return None
