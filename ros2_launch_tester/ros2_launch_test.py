#!/usr/bin/env python3
"""
CLI entry point for ROS2 Launch Tester
"""

import argparse
import sys
import os
from pathlib import Path
import rclpy
from rich.console import Console

from ros2_launch_tester.tester import LaunchTester


def create_sample_config(config_path: str):
    """Create a sample configuration file"""
    sample_config = """# ROS2 Launch Tester Configuration
# Specify what to expect from your launch file

nodes:
  - "/my_node"
  - "/another_node"

topics:
  - "/cmd_vel"
  - "/scan"
  - "/odom"

services:
  - "/my_service"
  - "/reset_simulation"

parameters:
  "/my_node/param1": "expected_value"
  "/my_node/param2": 42

# Optional: timeout settings
timeout:
  startup: 5.0    # seconds to wait for launch file startup
  tests: 30.0     # seconds to wait for each test category
"""
    
    with open(config_path, 'w') as f:
        f.write(sample_config)
    
    console = Console()
    console.print(f"[green]Sample configuration created at: {config_path}[/green]")
    console.print("[blue]Edit this file to match your launch file expectations[/blue]")


def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="ROS2 Launch Tester - Automatically test launch files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2-launch-test my_launch.py
  ros2-launch-test my_launch.py --expect config.yaml
  ros2-launch-test my_launch.py --timeout 60 --startup-time 5
  ros2-launch-test --create-config sample_config.yaml
        """
    )
    
    parser.add_argument(
        'launch_file',
        nargs='?',
        help='Path to the ROS2 launch file (.py)'
    )
    
    parser.add_argument(
        '--expect', '--config',
        type=str,
        help='Path to YAML configuration file with test expectations'
    )
    
    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='Timeout for tests in seconds (default: 30.0)'
    )
    
    parser.add_argument(
        '--startup-time',
        type=float,
        default=3.0,
        help='Time to wait for launch file startup in seconds (default: 3.0)'
    )
    
    parser.add_argument(
        '--create-config',
        type=str,
        help='Create a sample configuration file at the specified path'
    )
    
    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='Enable verbose output'
    )
    
    args = parser.parse_args()
    
    console = Console()
    
    # Handle config creation
    if args.create_config:
        create_sample_config(args.create_config)
        return 0
    
    # Validate launch file argument
    if not args.launch_file:
        console.print("[red]Error: Launch file path is required[/red]")
        parser.print_help()
        return 1
    
    if not os.path.exists(args.launch_file):
        console.print(f"[red]Error: Launch file not found: {args.launch_file}[/red]")
        return 1
    
    # Validate config file if provided
    if args.expect and not os.path.exists(args.expect):
        console.print(f"[red]Error: Config file not found: {args.expect}[/red]")
        return 1
    
    # Initialize ROS2
    try:
        rclpy.init()
        
        # Create tester instance
        tester = LaunchTester()
        
        console.print("[bold blue]🚀 ROS2 Launch Tester[/bold blue]")
        console.print(f"Launch file: {args.launch_file}")
        if args.expect:
            console.print(f"Config file: {args.expect}")
        console.print(f"Timeout: {args.timeout}s")
        console.print(f"Startup time: {args.startup_time}s")
        console.print()
        
        # Run the test
        success = tester.test_launch_file(
            launch_file_path=args.launch_file,
            config_path=args.expect,
            timeout=args.timeout,
            startup_time=args.startup_time
        )
        
        # Cleanup
        tester.destroy_node()
        rclpy.shutdown()
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        console.print("\n[yellow]Interrupted by user[/yellow]")
        return 1
    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        return 1
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    sys.exit(main())

