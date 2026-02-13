# Arduino-DroneCAN Hardware Test Suite

Automated hardware-in-the-loop (HIL) testing for Arduino DroneCAN nodes using the DroneCAN Python API.

## Overview

This test suite validates your Arduino DroneCAN firmware running on actual hardware by:
- ✅ Monitoring DroneCAN messages sent by the device
- ✅ Sending test messages to the device
- ✅ Validating message timing and content
- ✅ Testing the parameter system
- ✅ Verifying node health and connectivity

## Requirements

### Hardware
- Arduino DroneCAN node (flashed with your firmware)
- CAN interface adapter (USB-to-CAN)
  - Examples: SLCAN adapter, PCAN, SocketCAN on Linux
- CAN bus connection between PC and device

### Software
- Python 3.8 or later
- DroneCAN Python library
- CAN adapter drivers (if needed)

## Installation

### 1. Install Dependencies with UV

```bash
cd tests
uv sync
```

This will create a virtual environment and install all required dependencies.

### 2. Configure CAN Interface

Edit `test_config.yaml` to match your hardware setup:

```yaml
can_interface:
  interface: "slcan:COM3"  # Change to your adapter
  bitrate: 1000000
```

**Common interface strings:**
- SLCAN (Windows): `"slcan:COM3"`
- SLCAN (Linux): `"slcan:/dev/ttyUSB0"`
- SocketCAN (Linux): `"socketcan:can0"`
- PCAN: `"pcan:PCAN_USBBUS1"`

### 3. Verify Hardware Connection

Make sure your hardware is:
1. Powered on
2. Running the DroneCAN firmware
3. Connected to the CAN adapter
4. Using the correct node ID (default: 100)

## Running Tests

### Quick Start

Run all tests:
```bash
uv run test.py
```

### Test Options

**Run only basic tests (fast verification):**
```bash
uv run test.py --basic
```

**Run specific test categories:**
```bash
uv run test.py -m parameters    # Only parameter tests
uv run test.py -m messaging     # Only message tests
uv run test.py -m "not slow"    # Skip slow tests
```

**Generate HTML report:**
```bash
uv run test.py --html
```

**Verbose output:**
```bash
uv run test.py -v
```

**Combine options:**
```bash
uv run test.py --basic --html -v
```

### Using pytest Directly

You can also run pytest directly via uv for more control:

```bash
# Run all tests
uv run pytest

# Run specific test file
uv run pytest test_parameters.py

# Run specific test
uv run pytest test_heartbeat.py::TestHeartbeat::test_node_is_alive

# Run with specific markers
uv run pytest -m "basic and not slow"

# Show print statements
uv run pytest -s

# Stop on first failure
uv run pytest -x
```

## Test Categories

### Basic Tests (`-m basic`)
Quick sanity checks that verify:
- Node is responding
- Messages are being sent
- Parameters are accessible

### Parameter Tests (`-m parameters`)
Test the DroneCAN parameter system:
- Parameter read/write operations
- Value range validation
- Type checking
- Persistence (if reset capability available)

### Messaging Tests (`-m messaging`)
Validate message transmission and reception:
- BatteryInfo message frequency (10Hz)
- Message content validation
- Timing accuracy and jitter
- Message reception handling

### Slow Tests (`-m slow`)
Long-running tests that collect extensive data:
- Extended timing statistics
- Stress testing
- Endurance tests

## Test Structure

```
tests/
├── conftest.py              # Pytest fixtures and configuration
├── pytest.ini               # Pytest settings
├── test_config.yaml         # Hardware configuration
├── pyproject.toml           # UV project configuration with dependencies
├── test.py                  # Test runner with summary (run with: uv run test.py)
│
├── test_heartbeat.py        # Node health and connectivity
├── test_battery_messages.py # BatteryInfo message tests
├── test_parameters.py       # Parameter system tests
├── test_message_reception.py # Incoming message tests
│
└── utils/
    ├── message_validator.py # Message validation helpers
    └── timing_helpers.py    # Timing utilities
```

## Understanding Test Output

### Success Example
```
Arduino-DroneCAN Hardware Test Suite

┌─────────────────────┬────────┐
│ Setting             │ Value  │
├─────────────────────┼────────┤
│ Test Mode           │ Full   │
│ Verbose             │ No     │
│ HTML Report         │ No     │
└─────────────────────┴────────┘

Running tests...

╔═══════════════════════════════╗
║        Test Results           ║
╠═══════════════════════════════╣
║ Total Tests    │      32      ║
║ ✓ Passed       │      32      ║
║ ✗ Failed       │       0      ║
║ Duration       │   15.23s     ║
╚═══════════════════════════════╝

┌────────────────────────┐
│ ✓ ALL TESTS PASSED     │
└────────────────────────┘
```

### Failure Example
Shows which tests failed and detailed output for debugging.

## Configuration Reference

### test_config.yaml

| Section | Parameter | Description |
|---------|-----------|-------------|
| `can_interface` | `interface` | CAN adapter interface string |
| `can_interface` | `bitrate` | CAN bus bitrate (default: 1000000) |
| `device` | `node_id` | Expected node ID of device under test |
| `messages.battery_info` | `expected_frequency` | Expected message rate in Hz |
| `messages.battery_info` | `frequency_tolerance` | Allowed deviation (0.2 = ±20%) |
| `parameters` | `expected_params` | List of expected parameter names |

## Troubleshooting

### "Failed to connect to CAN interface"

**Causes:**
- CAN adapter not connected
- Wrong interface string in config
- Missing drivers

**Solutions:**
1. Check adapter is plugged in
2. Verify interface string matches your adapter
3. On Windows: Check Device Manager for COM port
4. On Linux: Run `ip link show` to see CAN interfaces

### "No heartbeat received"

**Causes:**
- Device not powered
- Wrong node ID configuration
- CAN bus connection issue
- Wrong bitrate

**Solutions:**
1. Verify device is running (check LED/serial output)
2. Check node ID matches config (default: 100)
3. Verify CAN bus wiring (CANH, CANL, GND)
4. Check bitrate matches (default: 1Mbps)

### "No BatteryInfo messages received"

**Causes:**
- Device not sending messages
- Firmware not running main loop
- Message frequency too slow

**Solutions:**
1. Check device serial output for errors
2. Verify firmware uploaded successfully
3. Check `dronecan.cycle()` is being called

### "Parameter tests failing"

**Causes:**
- Parameter names mismatch
- EEPROM not initialized
- Wrong parameter ranges

**Solutions:**
1. Verify parameter names in test_config.yaml match firmware
2. Check EEPROM is working on device
3. Review parameter value ranges in config

## Adding New Tests

### 1. Create a new test file

```python
# test_my_feature.py
import pytest
import dronecan

@pytest.mark.requires_hardware
class TestMyFeature:
    def test_something(self, dronecan_node, message_collector):
        # Your test code here
        pass
```

### 2. Use available fixtures

- `dronecan_node` - Connected DroneCAN node
- `message_collector` - Factory for collecting messages
- `parameter_manager` - Parameter get/set with cleanup
- `device_node_id` - Device node ID from config
- `test_config` - Full test configuration
- `wait_for_messages` - Helper to wait for message count

### 3. Add custom markers

In pytest.ini:
```ini
markers =
    my_feature: tests for my new feature
```

Then use: `uv run pytest -m my_feature`

## Best Practices

1. **Always use fixtures** for resource management
2. **Mark tests appropriately** (`@pytest.mark.basic`, `@pytest.mark.slow`, etc.)
3. **Use message collectors** instead of polling
4. **Add timeouts** to prevent hanging
5. **Clean up state** after tests (fixtures handle this)
6. **Document test purpose** in docstrings
7. **Use meaningful assertions** with clear error messages

## CI/CD Limitations

**Note:** These tests require physical hardware and cannot run in cloud CI/CD environments (GitHub Actions, etc.) without:
- Self-hosted runners with hardware attached
- Hardware emulation/virtualization (Renode, QEMU)

For cloud CI/CD, consider:
- Unit tests for business logic (separate from hardware tests)
- Firmware build verification
- Static analysis
- Reserve hardware tests for local/lab execution

## Example Workflow

1. **Develop firmware** on your device
2. **Flash firmware** via PlatformIO
3. **Run basic tests** to verify: `uv run test.py --basic`
4. **Run full suite** before committing: `uv run test.py --html`
5. **Review HTML report** for detailed results
6. **Commit** if all tests pass

## Support

For issues with the test suite:
- Check troubleshooting section above
- Review test configuration in `test_config.yaml`
- Run with `-v` flag for verbose output
- Check uv run pytest documentation: https://docs.pytest.org

For DroneCAN Python API help:
- DroneCAN DSDL reference: https://dronecan.github.io
- pydronecan documentation: https://dronecan.github.io/pydronecan

## License

Same as parent Arduino-DroneCAN project.
