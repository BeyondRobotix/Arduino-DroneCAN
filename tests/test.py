#!/usr/bin/env python3
"""
Arduino-DroneCAN Hardware Test Runner

This script runs the complete hardware test suite and provides a summary.
Run from the tests/ directory or project root.
"""

import sys
import subprocess
import argparse
from pathlib import Path
from datetime import datetime
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich import box
import re


console = Console()


def parse_pytest_output(output: str):
    """Parse pytest output to extract test results"""
    results = {
        'passed': 0,
        'failed': 0,
        'skipped': 0,
        'errors': 0,
        'total': 0,
        'duration': 0.0,
        'failed_tests': []
    }

    # Parse test counts from summary line
    # Example: "10 passed, 2 failed in 5.23s"
    summary_pattern = r'(\d+)\s+passed|(\d+)\s+failed|(\d+)\s+skipped|(\d+)\s+errors?'
    for match in re.finditer(summary_pattern, output):
        if match.group(1):
            results['passed'] = int(match.group(1))
        elif match.group(2):
            results['failed'] = int(match.group(2))
        elif match.group(3):
            results['skipped'] = int(match.group(3))
        elif match.group(4):
            results['errors'] = int(match.group(4))

    # Parse duration
    duration_match = re.search(r'in ([\d.]+)s', output)
    if duration_match:
        results['duration'] = float(duration_match.group(1))

    # Parse failed test names
    failed_pattern = r'FAILED (test_\w+\.py::\w+::\w+)'
    results['failed_tests'] = re.findall(failed_pattern, output)

    # Also parse error test names
    error_pattern = r'ERROR (test_\w+\.py::\w+::\w+)'
    error_tests = re.findall(error_pattern, output)
    results['failed_tests'].extend(error_tests)

    results['total'] = results['passed'] + results['failed'] + results['skipped'] + results['errors']

    return results


def display_header():
    """Display test suite header"""
    console.print()
    console.print(Panel.fit(
        "[bold cyan]Arduino-DroneCAN Hardware Test Suite[/bold cyan]\n"
        "Automated testing for DroneCAN nodes",
        border_style="cyan"
    ))
    console.print()


def display_config_info(args):
    """Display configuration information"""
    table = Table(show_header=False, box=box.ROUNDED, border_style="blue")
    table.add_column("Setting", style="cyan")
    table.add_column("Value", style="white")

    table.add_row("Test Mode", "Basic" if args.basic else "Full Suite")
    table.add_row("Verbose", "Yes" if args.verbose else "No")
    table.add_row("HTML Report", "Enabled" if args.html else "Disabled")
    if args.markers:
        table.add_row("Markers", args.markers)

    console.print(table)
    console.print()


def display_results(results):
    """Display test results in a nice table"""
    console.print()

    # Results table
    table = Table(title="Test Results", box=box.DOUBLE_EDGE, border_style="green" if results['failed'] == 0 else "red")
    table.add_column("Metric", style="cyan", justify="right")
    table.add_column("Count", style="bold white", justify="center")

    table.add_row("Total Tests", str(results['total']))
    table.add_row("[ PASS ] Passed", f"[green]{results['passed']}[/green]")

    if results['failed'] > 0:
        table.add_row("[ FAIL ] Failed", f"[red]{results['failed']}[/red]")
    else:
        table.add_row("[ FAIL ] Failed", "0")

    if results['skipped'] > 0:
        table.add_row("[ SKIP ] Skipped", f"[yellow]{results['skipped']}[/yellow]")

    if results['errors'] > 0:
        table.add_row("[ERROR ] Errors", f"[red]{results['errors']}[/red]")

    table.add_row("Duration", f"{results['duration']:.2f}s")

    console.print(table)

    # Failed tests detail
    if results['failed_tests']:
        console.print()
        console.print(Panel(
            "\n".join(f"  - {test}" for test in results['failed_tests']),
            title="[red]Failed Tests[/red]",
            border_style="red"
        ))

    # Overall status
    console.print()
    if results['failed'] == 0 and results['errors'] == 0:
        console.print(Panel.fit(
            "[bold green]ALL TESTS PASSED[/bold green]",
            border_style="green"
        ))
    else:
        console.print(Panel.fit(
            f"[bold red]{results['failed'] + results['errors']} TEST(S) FAILED[/bold red]",
            border_style="red"
        ))
    console.print()


def run_pytest(args):
    """Run pytest with specified arguments"""
    # Build pytest command
    cmd = [sys.executable, "-m", "pytest"]

    # Add verbosity
    if args.verbose:
        cmd.append("-vv")
    else:
        cmd.append("-v")

    # Add markers
    if args.basic:
        cmd.extend(["-m", "basic"])
    elif args.markers:
        cmd.extend(["-m", args.markers])

    # Add HTML report
    if args.html:
        html_path = Path("test_reports") / f"report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html"
        html_path.parent.mkdir(exist_ok=True)
        cmd.extend(["--html", str(html_path), "--self-contained-html"])

    # Add color
    cmd.append("--color=yes")

    # Add timeout
    cmd.extend(["--timeout", str(args.timeout)])

    # Add test directory
    cmd.append(".")

    # Run pytest
    console.print(f"[dim]Running: {' '.join(cmd)}[/dim]\n")
    console.print("[cyan]Running tests...[/cyan]\n")

    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        cwd=Path(__file__).parent
    )

    return result


def check_hardware_connection():
    """Check if hardware is likely connected"""
    try:
        import yaml
        config_path = Path(__file__).parent / "test_config.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        interface = config['can_interface']['interface']
        console.print(f"[dim]Configured CAN interface: {interface}[/dim]")
        console.print("[yellow]WARNING: Ensure hardware is connected before running tests[/yellow]\n")
        return True
    except Exception as e:
        console.print(f"[yellow]Warning: Could not verify hardware configuration: {e}[/yellow]\n")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Run Arduino-DroneCAN hardware tests",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  uv run test.py                    # Run all tests
  uv run test.py --basic            # Run only basic tests
  uv run test.py -v                 # Verbose output
  uv run test.py --html             # Generate HTML report
  uv run test.py -m "parameters"    # Run only parameter tests
  uv run test.py -m "not slow"      # Skip slow tests
        """
    )

    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose output"
    )

    parser.add_argument(
        "--basic",
        action="store_true",
        help="Run only basic tests (quick verification)"
    )

    parser.add_argument(
        "-m", "--markers",
        type=str,
        help='Run tests matching marker expression (e.g., "parameters" or "not slow")'
    )

    parser.add_argument(
        "--html",
        action="store_true",
        help="Generate HTML test report"
    )

    parser.add_argument(
        "--timeout",
        type=int,
        default=30,
        help="Test timeout in seconds (default: 30)"
    )

    parser.add_argument(
        "--no-check",
        action="store_true",
        help="Skip hardware connection check"
    )

    args = parser.parse_args()

    # Display header
    display_header()

    # Check hardware
    if not args.no_check:
        check_hardware_connection()

    # Display configuration
    display_config_info(args)

    # Run tests
    result = run_pytest(args)

    # Parse and display results
    results = parse_pytest_output(result.stdout + result.stderr)

    # Always show pytest output if verbose or if tests failed
    if args.verbose or results['failed'] > 0:
        console.print("\n[dim]--- Pytest Output ---[/dim]")
        console.print(result.stdout)
        if result.stderr:
            console.print("[red]" + result.stderr + "[/red]")
        console.print("[dim]--- End Output ---[/dim]\n")

    display_results(results)

    # Print HTML report location if generated
    if args.html:
        console.print(f"[cyan]HTML report saved to: test_reports/[/cyan]\n")

    # Exit with appropriate code
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
