# DroneCAN Tests -- Agent Notes

## What this does

Three tools live here:

1. **`build_upload_test.py`** -- end-to-end pipeline that builds the firmware
   with PlatformIO, uploads it to the node via ST-Link, waits for boot, then
   runs the pytest test suite.  This is the main entry point for the
   edit → build → flash → verify development loop.

2. **`test_node.py`** -- pytest-based hardware-in-the-loop test suite that
   verifies the DroneCAN node running `src/main.cpp` is broadcasting the
   expected messages (NodeStatus, BatteryInfo) at the correct rates, with
   valid field ranges, and that the DroneCAN library (`lib/libArduinoDroneCAN`)
   correctly handles parameters, GetNodeInfo, node restart, and DNA.

3. **`test_can_discovery.py`** -- standalone discovery script that connects
   to a USB CAN adapter, starts a DroneCAN DNA server, and continuously
   prints `NodeStatus` messages from all nodes on the bus.

## Hardware setup (confirmed working)

- **CAN adapter**: USB Serial Device on **COM21** (silicon USB-CDC adapter)
- **Autopilot**: ArduPilot (Cube Orange+) connected to the adapter via CAN
- **CAN bitrate**: 1 Mbps (`-b 1000000`)
- **Discovered node**: Node ID **69**, health OK, mode OPERATIONAL


## How to run

### Full pipeline (build → upload → test)

```bash
cd tests
uv run python build_upload_test.py
```

With options:

```bash
# Different PlatformIO environment
uv run python build_upload_test.py -e Micro-Node-No-Bootloader

# Different CAN adapter / bitrate
uv run python build_upload_test.py -i COM5 -b 500000

# Build only (check it compiles)
uv run python build_upload_test.py --build-only

# Skip upload (node already flashed)
uv run python build_upload_test.py --no-upload

# Skip tests (just build and flash)
uv run python build_upload_test.py --no-test

# Forward extra flags to pytest
uv run python build_upload_test.py -- -k test_node_present -vv
```

### Pipeline CLI reference

| Flag | Default | Description |
|------|---------|-------------|
| `-e` / `--env` | `Micro-Node-Bootloader` | PlatformIO environment to build/upload |
| `-i` / `--interface` | `COM21` | CAN adapter COM port or device path |
| `-b` / `--bitrate` | `1000000` | CAN bus bitrate in bps |
| `--boot-wait` | `4` | Seconds to wait after upload for node boot |
| `--no-upload` | off | Skip the upload step |
| `--no-test` | off | Skip the test step |
| `--build-only` | off | Build only -- no upload, no test |

Exit codes: `0` all passed, `1` build failed, `2` upload failed, `3` tests failed.

The `-i` and `-b` flags are passed through to the test suite via the
`DRONECAN_INTERFACE` and `DRONECAN_BITRATE` environment variables (read by
`conftest.py`, falling back to COM21 / 1 Mbps).

### Discovery script

```bash
cd tests
uv run python test_can_discovery.py -i COM21 -b 1000000 -t 30
```

Run indefinitely until Ctrl-C:

```bash
uv run python test_can_discovery.py -i COM21 -b 1000000
```

Disable the DNA server (passive listen only):

```bash
uv run python test_can_discovery.py -i COM21 -b 1000000 --no-dna
```

### Discovery CLI reference

| Flag | Default | Description |
|------|---------|-------------|
| `-i` / `--interface` | required | COM port or device path |
| `-b` / `--bitrate` | 1000000 | CAN bus bitrate in bps |
| `--baudrate` | None (3Mbps) | Serial port baud rate -- only needed for non-USB-CDC adapters |
| `--bustype` | slcan | python-can bus type; use `socketcan` on Linux |
| `--node-id` | 127 | Local node ID claimed by this tool |
| `--no-dna` | off | Disable DNA allocation server |
| `-t` / `--timeout` | None | Stop after N seconds; omit to run until Ctrl-C |

## Test suite (`test_node.py`)

Run all tests:

```bash
cd tests
uv run pytest test_node.py -v
```

The test suite uses two **session-scoped** fixtures (see `conftest.py`):

- **`can_node`** -- opens COM21 once, creates an always-on DNA server
  (`DNAServer` wrapper), and shares the connection across all 22 tests.
- **`dna_server`** -- exposes the `DNAServer` wrapper so tests that change
  NODEID can call `dna_server.clear_allocations()` before restarting the
  node.

Handlers are cleaned up after each test via `handle.remove()`.

### Test groups

**NodeStatus (smoke tests)**
- `test_node_present` -- node 69 is on the bus
- `test_node_healthy_and_operational` -- health OK, mode OPERATIONAL
- `test_node_status_rate` -- broadcasts at ~1 Hz

**BatteryInfo (main payload from `src/main.cpp`)**
- `test_battery_info_received` -- BatteryInfo is broadcast
- `test_battery_info_rate` -- rate is ~10 Hz (100 ms interval)
- `test_battery_info_voltage_in_range` -- raw ADC 0-4095
- `test_battery_info_current_in_range` -- raw ADC 0-4095
- `test_battery_info_temperature_plausible` -- MCU die temp -40..125 C

**GetNodeInfo (identity and metadata)**
- `test_get_node_info_name` -- name = "Beyond Robotix Node"
- `test_get_node_info_unique_id` -- 16-byte hardware UID is non-zero
- `test_get_node_info_uptime` -- uptime > 0

**Parameters -- read (via `uavcan.protocol.param.GetSet`)**
- `test_param_nodeid` -- NODEID = 69 (matches bus ID)
- `test_param_parm1` -- PARM_1 = 50.0 (set in `setup()`)
- `test_param_parm2` -- PARM_2 within configured range 0-100

**Parameters -- set, clamping, edge cases**
- `test_param_set_readback` -- write PARM_2=42.0 via CAN, read back, restore
- `test_param_clamping_high` -- value > max clamps to 100
- `test_param_clamping_low` -- value < min clamps to 0
- `test_param_by_index` -- index-based lookup returns correct name
- `test_param_nonexistent_name_falls_back_to_index` -- bad name falls back
  to index field (firmware behaviour, see `dronecan.cpp:270`)
- `test_param_out_of_range_index` -- index 999 + bad name returns empty

**RestartNode and persistence (run last -- reboots the node)**
- `test_restart_node` -- reboot via RestartNode, verify uptime resets and
  PARM_1/PARM_2 survive via EEPROM
- `test_change_node_id_and_restore` -- set NODEID to 70, restart, verify
  node appears at bus ID 70, set back to 69, restart, verify restored

### Key implementation details

- `_collect()` spins the node and records `(timestamp, event)` tuples; the
  handler is registered then removed in a `try/finally` block.
- `_get_param()` / `_set_param()` send `param.GetSet` service requests and
  spin until the response callback fires or the timeout expires. Both accept
  an optional `target` node ID for talking to the node after an ID change.
- `_get_node_info()` sends a `GetNodeInfo` request and returns the response.
- `_send_restart()` sends a `RestartNode` request with the magic number.
- `_wait_for_node()` spins until a `NodeStatus` arrives from a given node ID.
- Rate tests use a +/-40% tolerance (`RATE_TOLERANCE = 0.40`) because the
  node's `millis()` loop and Windows timer resolution introduce jitter.

### DNA server and NODEID changes

The `conftest.py` `DNAServer` class wraps `CentralizedServer` + `NodeMonitor`.
The `CentralizedServer` caches `unique_id -> node_id` mappings in a SQLite
in-memory database. When a test changes NODEID and restarts the node, the
cached mapping must be cleared first, otherwise the server returns the old ID
regardless of the node's preferred ID.

```python
dna_server.clear_allocations()   # wipe the SQLite allocation table
_send_restart(can_node)          # reboot the node
time.sleep(1)                    # let the MCU reset
_wait_for_node(can_node, new_id) # wait for DNA to complete
```

The `clear_allocations()` method directly deletes all rows from the
`allocation` table without tearing down the server or monitor, avoiding the
stale-entry problem where `NodeMonitor.are_all_nodes_discovered()` blocks
DNA allocations for nodes it saw before the restart.

## Dependencies

```
pydronecan   -- DroneCAN protocol library (wraps the dronecan package)
pyserial     -- serial port access
python-can   -- python-can, used as the in-process SLCAN driver on Windows
pytest       -- test runner for test_node.py
ruff         -- linter (dev)
ty           -- type checker (dev)
```

Install:

```bash
uv sync
```

Lint / type-check:

```bash
uv run ruff check test_can_discovery.py
uv run ty check test_can_discovery.py
```

## Issues found and fixed

### 1. Original script was entirely broken

The original `test_can_discovery.py` imported from `pydronecan.dronecan` and
`pydronecan.dna` -- neither of which exist. The `pydronecan` package installs
itself as the `dronecan` top-level package, not as a `pydronecan` namespace.
It also used `can.interface.Bus` directly (python-can API), and was written as
an `async` loop which is not how dronecan works (it is synchronous/event-driven).

**Fix**: Complete rewrite using the correct `dronecan` API:
- `dronecan.make_node()` to create the node
- `dronecan.app.node_monitor.NodeMonitor` to track nodes
- `dronecan.app.dynamic_node_id.CentralizedServer` for DNA allocation
- `node.add_handler(uavcan.protocol.NodeStatus, callback)` to receive messages
- `node.spin(timeout)` loop instead of async iteration

### 2. dronecan's SLCAN driver crashes on Windows USB serial ports

dronecan's built-in SLCAN driver spawns a `multiprocessing.Process` subprocess
to handle serial I/O. On Windows, when that subprocess calls
`serial.Serial._reconfigure_port()` (triggered by setting a read timeout), it
fails with:

```
PermissionError(13, 'A device attached to the system is not functioning.', None, 31)
```

**Fix**: For COM ports on Windows, bypass the dronecan SLCAN subprocess and use
`dronecan.driver.python_can.PythonCAN` directly, which wraps python-can's
in-process SLCAN driver. python-can opens the port in the same process, where
`_reconfigure_port()` succeeds.

```python
can_driver = PythonCAN(interface, bustype="slcan", bitrate=bitrate)
node = dronecan.node.Node(can_driver, node_id=node_id)
```

### 3. python-can SLCAN does not implement `flush_tx_buffer`

When the DNA server sends allocation packets, dronecan's writer thread calls
`bus.flush_tx_buffer()`. python-can's SLCAN interface raises `NotImplementedError`
for this method.

**Fix**: Stub it out after creating the driver:

```python
can_driver._bus.flush_tx_buffer = lambda: None
```

### 4. `uavcan` and `event.message` are dynamic attributes

`dronecan.uavcan` is injected into the module's `__dict__` at runtime (not a
normal importable submodule). Similarly, `TransferEvent.message` is set via
`setattr` in the dronecan internals.

**Fix**: Import with `# type: ignore[attr-defined]` and annotate the dynamic
attribute access accordingly. Both work correctly at runtime.

### 5. `[tool.ty]` in pyproject.toml had an invalid key

The original `pyproject.toml` had `packages = ["pydronecan"]` under `[tool.ty]`
which is not a valid ty config key and caused ty to fail entirely.

**Fix**: Changed to `environment.python = ".venv"`.

### 6. DNA server caches stale node ID allocations

The `CentralizedServer` stores `unique_id -> node_id` mappings in an in-memory
SQLite database. When a test changes NODEID and restarts the node, the server
returns the *old* cached ID, ignoring the node's new preferred ID.

**Fix**: Added `DNAServer.clear_allocations()` in `conftest.py` which deletes
all rows from the allocation table without tearing down the server. Tests that
change NODEID call this before sending `RestartNode`.

Note: an earlier approach that tore down and recreated the entire server failed
because `NodeMonitor.are_all_nodes_discovered()` returned `False` for stale
entries, blocking all DNA allocations.

## dronecan API quick reference

```python
import dronecan
from dronecan import uavcan

# Create node (Linux / non-COM / non-Windows)
node = dronecan.make_node("can0", node_id=127, bitrate=1_000_000)

# Create node (Windows COM port via python-can)
from dronecan.driver.python_can import PythonCAN
can_driver = PythonCAN("COM21", bustype="slcan", bitrate=1_000_000)
can_driver._bus.flush_tx_buffer = lambda: None
node = dronecan.node.Node(can_driver, node_id=127)

# DNA server
import dronecan.app.node_monitor as nm
import dronecan.app.dynamic_node_id as dna
monitor = nm.NodeMonitor(node)
dna_server = dna.CentralizedServer(node, monitor)

# Listen for NodeStatus
def on_status(event):
    print(event.transfer.source_node_id, event.message.health)

node.add_handler(uavcan.protocol.NodeStatus, on_status)

# Service request (param read)
def on_param(event):
    if event:
        print(event.response.value.real_value)

req = uavcan.protocol.param.GetSet.Request()
req.name = "PARM_1"
node.request(req, target_node_id=69, callback=on_param)

# Service request (param write)
req = uavcan.protocol.param.GetSet.Request()
req.name = "PARM_2"
req.value.real_value = 42.0
node.request(req, target_node_id=69, callback=on_param)

# Spin (blocking for up to 1 s, then returns)
while True:
    node.spin(timeout=1.0)
```
