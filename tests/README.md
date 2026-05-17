# DroneCAN Hardware-in-the-Loop Tests

Pytest-based test suite that talks to a live DroneCAN node (running
`src/main.cpp`) over a USB CAN adapter. The tests verify node presence,
message rates, parameter get/set/clamping, GetNodeInfo, node restart
with DNA re-allocation, NODEID changes, and EEPROM persistence.

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
| 12 | `test_get_node_info_name` | GetNodeInfo returns "Beyond Robotix Node" |
| 13 | `test_get_node_info_unique_id` | Hardware unique ID is non-zero |
| 14 | `test_get_node_info_uptime` | Uptime is a sane positive value |
| 15 | `test_param_set_readback` | Set PARM_2 over CAN, read it back |
| 16 | `test_param_clamping_high` | Value above max (100) clamps to 100 |
| 17 | `test_param_clamping_low` | Value below min (0) clamps to 0 |
| 18 | `test_param_by_index` | Index-based lookup returns correct param |
| 19 | `test_param_nonexistent_name_falls_back_to_index` | Bad name falls back to index 0 |
| 20 | `test_param_out_of_range_index` | Out-of-range index returns empty response |
| 21 | `test_restart_node` | Node reboots, DNA re-allocates, uptime resets, params persist |
| 22 | `test_change_node_id_and_restore` | Change NODEID to 70, restart, verify, restore to 69 |

Tests 21-22 reboot the node and must run last.

## Build, upload, and test pipeline

**`build_upload_test.py`** automates the full development loop: build the
firmware with PlatformIO, upload it via ST-Link, wait for the node to boot,
then run the test suite. Each stage streams its output and reports clear
pass/fail status.

```bash
cd tests

# Full pipeline (default: Micro-Node-Bootloader, COM21, 1 Mbps)
uv run python build_upload_test.py

# Different board environment
uv run python build_upload_test.py -e Micro-Node-No-Bootloader

# Different CAN adapter / bitrate
uv run python build_upload_test.py -i COM5 -b 500000

# Build only (verify it compiles)
uv run python build_upload_test.py --build-only

# Skip upload (node already flashed)
uv run python build_upload_test.py --no-upload

# Skip tests (just build and upload)
uv run python build_upload_test.py --no-test

# Forward extra flags to pytest (after --)
uv run python build_upload_test.py -- -k test_node_present -vv
```

| Flag | Default | Description |
|------|---------|-------------|
| `-e` / `--env` | `Micro-Node-Bootloader` | PlatformIO environment to build/upload |
| `-i` / `--interface` | `COM21` | CAN adapter COM port or device path |
| `-b` / `--bitrate` | `1000000` | CAN bus bitrate in bps |
| `--boot-wait` | `4` | Seconds to wait after upload for the node to boot |
| `--no-upload` | | Skip the upload step |
| `--no-test` | | Skip the test step |
| `--build-only` | | Build only — no upload, no test |

Exit codes: `0` = all passed, `1` = build failed, `2` = upload failed, `3` = tests failed.

## Other scripts

- **`test_can_discovery.py`** -- standalone discovery tool that lists all
  nodes on the bus. See [AGENT.md](AGENT.md) for usage and CLI flags.

## Dependencies

Managed via `pyproject.toml`:

```
pydronecan   -- DroneCAN protocol library
pyserial     -- serial port access
python-can   -- in-process SLCAN driver (Windows workaround)
pytest       -- test runner
ruff         -- linter (dev)
ty           -- type checker (dev)
```
