# CAN Network Discovery — Agent Notes

## What this does

`test_can_discovery.py` connects to a USB CAN adapter, starts a DroneCAN DNA
(Dynamic Node ID Allocation) server, and continuously prints `NodeStatus`
messages broadcast by nodes on the CAN bus — primarily an ArduPilot autopilot.

## Hardware setup (confirmed working)

- **CAN adapter**: USB Serial Device on **COM21** (silicon USB-CDC adapter)
- **Autopilot**: ArduPilot (Cube Orange+) connected to the adapter via CAN
- **CAN bitrate**: 1 Mbps (`-b 1000000`)
- **Discovered node**: Node ID 10, health OK, mode OPERATIONAL

Other ports present on this machine (not used for this script):

| Port | Device | Notes |
|------|--------|-------|
| COM19 | ArduPilot SLCAN | ArduPilot's own built-in SLCAN — does **not** respond to SLCAN init, use COM21 instead |
| COM20 | ArduPilot MAVLink | MAVLink telemetry port |
| COM5 | Silicon Labs CP210x | Alternate adapter port — unreliable, prefer COM21 |

## How to run

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

## CLI reference

| Flag | Default | Description |
|------|---------|-------------|
| `-i` / `--interface` | required | COM port or device path |
| `-b` / `--bitrate` | 1000000 | CAN bus bitrate in bps |
| `--baudrate` | None (3Mbps) | Serial port baud rate — only needed for non-USB-CDC adapters |
| `--bustype` | slcan | python-can bus type; use `socketcan` on Linux |
| `--node-id` | 127 | Local node ID claimed by this tool |
| `--no-dna` | off | Disable DNA allocation server |
| `-t` / `--timeout` | None | Stop after N seconds; omit to run until Ctrl-C |

## Dependencies

```
pydronecan   — DroneCAN protocol library (wraps the dronecan package)
pyserial     — serial port access
python-can   — python-can, used as the in-process SLCAN driver on Windows
ruff         — linter (dev)
ty           — type checker (dev)
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
`pydronecan.dna` — neither of which exist. The `pydronecan` package installs
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

This affects COM5, COM19, COM21 — all USB CDC / CP210x ports tested.

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

# Spin (blocking for up to 1 s, then returns)
while True:
    node.spin(timeout=1.0)
```
