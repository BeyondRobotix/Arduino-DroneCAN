#!/usr/bin/env python3
"""
Build, upload, and test pipeline for ArduinoDroneCAN.

Builds the firmware with PlatformIO, uploads it to the connected node
via ST-Link, waits for the node to boot and appear on the CAN bus,
then runs the pytest test suite.

Usage:
    # Default: build + upload + test (Micro-Node-Bootloader env, COM21)
    python build_upload_test.py

    # Specific PlatformIO environment
    python build_upload_test.py -e Micro-Node-No-Bootloader

    # Skip upload (just build + test, node already flashed)
    python build_upload_test.py --no-upload

    # Skip tests (just build + upload)
    python build_upload_test.py --no-test

    # Build only (no upload, no test)
    python build_upload_test.py --build-only

    # Custom CAN interface and bitrate
    python build_upload_test.py -i COM5 -b 500000

    # Extra pytest flags (e.g. run a single test, increase verbosity)
    python build_upload_test.py -- -k test_node_present -vv
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

# ---------------------------------------------------------------------------
# Defaults (match conftest.py / AGENT.md)
# ---------------------------------------------------------------------------
DEFAULT_ENV = "Micro-Node-Bootloader"
DEFAULT_INTERFACE = "COM21"
DEFAULT_BITRATE = 1_000_000
NODE_BOOT_WAIT = 4  # seconds to wait after upload for the node to boot


def banner(msg: str) -> None:
    width = max(len(msg) + 4, 60)
    print()
    print("=" * width)
    print(f"  {msg}")
    print("=" * width)
    print()


def run(cmd: list[str], *, cwd: str | None = None, env: dict | None = None) -> int:
    """Run a command, stream output live, return the exit code."""
    merged_env = {**os.environ, **(env or {})}
    proc = subprocess.run(
        cmd,
        cwd=cwd,
        env=merged_env,
    )
    return proc.returncode


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build(pio_env: str) -> bool:
    banner(f"BUILD  ({pio_env})")
    rc = run(
        ["pio", "run", "-e", pio_env],
        cwd=PROJECT_ROOT,
    )
    if rc != 0:
        print(f"\n*** BUILD FAILED (exit {rc}) ***")
        print("Check the compiler output above for errors.")
        return False
    print("\nBuild succeeded.")
    return True


# ---------------------------------------------------------------------------
# Upload
# ---------------------------------------------------------------------------
def upload(pio_env: str) -> bool:
    banner(f"UPLOAD  ({pio_env})")
    rc = run(
        ["pio", "run", "-e", pio_env, "-t", "upload"],
        cwd=PROJECT_ROOT,
    )
    if rc != 0:
        print(f"\n*** UPLOAD FAILED (exit {rc}) ***")
        print("Possible causes:")
        print("  - ST-Link not connected or not detected")
        print("  - Target board not powered")
        print("  - Wrong PlatformIO environment for the connected board")
        print("  - ST-Link drivers not installed")
        print()
        print("Troubleshooting:")
        print("  1. Run: pio device list")
        print("  2. Check ST-Link USB connection")
        print("  3. Try a different environment with -e <env>")
        return False
    print("\nUpload succeeded.")
    return True


# ---------------------------------------------------------------------------
# Wait for node boot
# ---------------------------------------------------------------------------
def wait_for_boot(seconds: int) -> None:
    banner(f"WAITING {seconds}s FOR NODE BOOT")
    for i in range(seconds, 0, -1):
        print(f"  {i}...", end=" ", flush=True)
        time.sleep(1)
    print("go!\n")


# ---------------------------------------------------------------------------
# Test
# ---------------------------------------------------------------------------
def test(interface: str, bitrate: int, extra_pytest_args: list[str]) -> bool:
    banner("TEST")

    test_env = {
        "DRONECAN_INTERFACE": interface,
        "DRONECAN_BITRATE": str(bitrate),
    }

    cmd = [
        sys.executable, "-m", "pytest", "test_node.py",
        "-v",
        "--tb=short",
        *extra_pytest_args,
    ]

    print(f"Interface: {interface}   Bitrate: {bitrate}")
    print(f"Command:   {' '.join(cmd)}\n")

    rc = run(cmd, cwd=SCRIPT_DIR, env=test_env)
    if rc != 0:
        print(f"\n*** TESTS FAILED (exit {rc}) ***")
        print("Review the pytest output above for details.")
        print("Common causes:")
        print(f"  - Node not responding on {interface} at {bitrate} bps")
        print("  - Node did not finish DNA allocation yet (increase --boot-wait)")
        print("  - Firmware change broke a CAN message or parameter")
        return False
    print("\nAll tests passed.")
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build, upload, and test ArduinoDroneCAN firmware.",
        epilog=(
            "Any arguments after '--' are forwarded to pytest.\n"
            "Example: python build_upload_test.py -- -k test_node_present -vv"
        ),
    )
    parser.add_argument(
        "-e", "--env", default=DEFAULT_ENV,
        help=f"PlatformIO environment (default: {DEFAULT_ENV})",
    )
    parser.add_argument(
        "-i", "--interface", default=DEFAULT_INTERFACE,
        help=f"CAN adapter COM port / device path (default: {DEFAULT_INTERFACE})",
    )
    parser.add_argument(
        "-b", "--bitrate", type=int, default=DEFAULT_BITRATE,
        help=f"CAN bus bitrate in bps (default: {DEFAULT_BITRATE})",
    )
    parser.add_argument(
        "--boot-wait", type=int, default=NODE_BOOT_WAIT,
        help=f"Seconds to wait after upload for node to boot (default: {NODE_BOOT_WAIT})",
    )
    parser.add_argument(
        "--no-upload", action="store_true",
        help="Skip the upload step (node already flashed)",
    )
    parser.add_argument(
        "--no-test", action="store_true",
        help="Skip the test step (just build and upload)",
    )
    parser.add_argument(
        "--build-only", action="store_true",
        help="Build only — no upload, no test",
    )

    # Split on '--' so everything after it goes to pytest
    if "--" in sys.argv:
        split_idx = sys.argv.index("--")
        our_args = sys.argv[1:split_idx]
        pytest_args = sys.argv[split_idx + 1:]
    else:
        our_args = sys.argv[1:]
        pytest_args = []

    args = parser.parse_args(our_args)

    skip_upload = args.no_upload or args.build_only
    skip_test = args.no_test or args.build_only

    banner("ArduinoDroneCAN  Build → Upload → Test")
    print(f"  Environment : {args.env}")
    print(f"  Interface   : {args.interface}")
    print(f"  Bitrate     : {args.bitrate}")
    print(f"  Upload      : {'skip' if skip_upload else 'yes'}")
    print(f"  Test        : {'skip' if skip_test else 'yes'}")
    if pytest_args:
        print(f"  Pytest args : {pytest_args}")
    print()

    # ---- Build ----
    if not build(args.env):
        return 1

    # ---- Upload ----
    if not skip_upload:
        if not upload(args.env):
            return 2
        wait_for_boot(args.boot_wait)

    # ---- Test ----
    if not skip_test:
        if not test(args.interface, args.bitrate, pytest_args):
            return 3

    banner("PIPELINE COMPLETE — ALL STEPS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
