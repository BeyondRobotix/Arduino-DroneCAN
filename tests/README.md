# DroneCAN Hardware-in-the-Loop Tests

Pytest-based test suite that talks to a live DroneCAN node (running
`src/main.cpp`) over a USB CAN adapter. The tests verify that the node
broadcasts the expected messages at the right rates and that parameters
are readable via the DroneCAN param service.

## Prerequisites

- **CAN adapter** connected on **COM21** (USB-CDC SLCAN adapter)
- **DroneCAN node** powered and on the same CAN bus (currently node ID **69**)
- **CAN bitrate**: 1 Mbps
- Python 3.13+

## Quick start

```bash
cd tests
uv sync
uv run pytest test_node.py -v
```

## Test summary

| # | Test | What it checks |
|---|------|----------------|
| 1 | `test_node_present` | Node 69 sends NodeStatus within 5 s |
| 2 | `test_node_healthy_and_operational` | health = OK, mode = OPERATIONAL |
| 3 | `test_node_status_rate` | NodeStatus arrives at ~1 Hz |
| 4 | `test_battery_info_received` | BatteryInfo is broadcast |
| 5 | `test_battery_info_rate` | BatteryInfo arrives at ~10 Hz |
| 6 | `test_battery_info_voltage_in_range` | voltage is a valid 12-bit ADC value (0-4095) |
| 7 | `test_battery_info_current_in_range` | current is a valid 12-bit ADC value (0-4095) |
| 8 | `test_battery_info_temperature_plausible` | MCU die temp is -40 to 125 °C |
| 9 | `test_param_nodeid` | NODEID param matches the node's bus ID |
| 10 | `test_param_parm1` | PARM_1 = 50.0 (set in setup()) |
| 11 | `test_param_parm2` | PARM_2 is within configured range 0-100 |

## Other scripts

- **`test_can_discovery.py`** — standalone discovery tool that lists all
  nodes on the bus. See [AGENT.md](AGENT.md) for usage and CLI flags.

## Dependencies

Managed via `pyproject.toml`:

```
pydronecan   — DroneCAN protocol library
pyserial     — serial port access
python-can   — in-process SLCAN driver (Windows workaround)
pytest       — test runner
ruff         — linter (dev)
ty           — type checker (dev)
```
