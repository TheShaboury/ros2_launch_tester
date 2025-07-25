"""
Main class to handle testing of ROS2 launch files
"""

import os
import sys
import time
import subprocess
import threading
import signal
from typing import Dict, List, Optional, Any
import yaml
import rclpy
from rclpy.node import Node
from rich.console import Console
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn

from .node_checker import NodeChecker
from .topic_checker import TopicChecker


class LaunchTester(Node):
    """Main class for testing ROS2 launch files"""
    
    def __init__(self, node_name: str = 'launch_tester'):
        super().__init__(node_name)
        self.console = Console()
        self.node_checker = NodeChecker(self)
        self.topic_checker = TopicChecker(self)
        self.launch_process = None
        self.test_results = {}
        self.launch_stdout_thread = None
        self.launch_stderr_thread = None
        
    def load_expectations(self, config_path: str) -> Dict[str, Any]:
        """Load test expectations from YAML config file"""
        try:
            with open(config_path, 'r') as file:
                return yaml.safe_load(file)
        except FileNotFoundError:
            self.console.print(f"[red]Config file not found: {config_path}[/red]")
            return {}
        except yaml.YAMLError as e:
            self.console.print(f"[red]Error parsing YAML config: {e}[/red]")
            return {}
    
    def _read_stream(self, stream, name):
        """Helper to read and print subprocess stream"""
        for line in iter(stream.readline, b''):
            self.console.print(f"[dim blue][{name}][/dim blue] {line.decode().strip()}")
        stream.close()

    def start_launch_file(self, launch_file_path: str) -> bool:
        """Start the launch file in a subprocess"""
        if not os.path.exists(launch_file_path):
            self.console.print(f"[red]Launch file not found: {launch_file_path}[/red]")
            return False
        
        try:
            self.console.print(f"[blue]Starting launch file: {launch_file_path}[/blue]")
            
            # Pass the current environment to the subprocess
            # This is crucial for ROS2 environment variables to be inherited
            env = os.environ.copy()
            
            # Ensure ROS_DOMAIN_ID is explicitly set for the subprocess
            # It's good practice to set it in your main shell too, e.g., export ROS_DOMAIN_ID=0
            if 'ROS_DOMAIN_ID' not in env:
                self.console.print("[yellow]Warning: ROS_DOMAIN_ID not set in environment. Defaulting to 0.[/yellow]")
                env['ROS_DOMAIN_ID'] = '0' # Default to 0 if not set
            else:
                self.console.print(f"[blue]Using ROS_DOMAIN_ID: {env['ROS_DOMAIN_ID']}[/blue]")
            
            self.launch_process = subprocess.Popen(
                ['ros2', 'launch', launch_file_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid, # Ensures process group is created for clean termination
                env=env # Pass the environment
            )
            self.console.print(f"[blue]Launch process PID: {self.launch_process.pid}[/blue]")

            # Start threads to read stdout and stderr
            self.launch_stdout_thread = threading.Thread(
                target=self._read_stream, args=(self.launch_process.stdout, "LAUNCH_STDOUT")
            )
            self.launch_stderr_thread = threading.Thread(
                target=self._read_stream, args=(self.launch_process.stderr, "LAUNCH_STDERR")
            )
            self.launch_stdout_thread.daemon = True # Allow main program to exit
            self.launch_stderr_thread.daemon = True # Allow main program to exit
            self.launch_stdout_thread.start()
            self.launch_stderr_thread.start()

            # Give the process a moment to start and print initial output
            time.sleep(1.0) 
            return True
        except Exception as e:
            self.console.print(f"[red]Failed to start launch file: {e}[/red]")
            return False
    
    def stop_launch_file(self):
        """Stop the launch file process"""
        if self.launch_process:
            try:
                self.console.print("[blue]Attempting to stop launch process...[/blue]")
                # Send SIGTERM to the process group
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGTERM)
                self.launch_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.console.print("[yellow]Launch process did not terminate gracefully, forcing kill...[/yellow]")
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGKILL)
            except Exception as e:
                self.console.print(f"[yellow]Warning: Error stopping launch process: {e}[/yellow]")
            finally:
                # Ensure threads are joined if they are still alive
                if self.launch_stdout_thread and self.launch_stdout_thread.is_alive():
                    self.launch_stdout_thread.join(timeout=1)
                if self.launch_stderr_thread and self.launch_stderr_thread.is_alive():
                    self.launch_stderr_thread.join(timeout=1)
                self.launch_process = None # Clear the process
    
    def wait_for_startup(self, startup_time: float = 3.0):
        """Wait for the launch file to start up"""
        self.console.print(f"[blue]Waiting {startup_time}s for launch file startup...[/blue]")
        time.sleep(startup_time)
    
    def run_tests(self, expectations: Dict[str, Any], timeout: float = 30.0) -> Dict[str, Any]:
        """Run all tests based on expectations"""
        results = {
            'nodes': {},
            'topics': {},
            'services': {},
            'parameters': {},
            'overall_success': True
        }
        
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            console=self.console
        ) as progress:
            
            # Test nodes
            if 'nodes' in expectations:
                task = progress.add_task("Testing nodes...", total=None)
                results['nodes'] = self.node_checker.check_nodes(
                    expectations['nodes'], timeout
                )
                progress.remove_task(task)
            
            # Test topics
            if 'topics' in expectations:
                task = progress.add_task("Testing topics...", total=None)
                results['topics'] = self.topic_checker.check_topics(
                    expectations['topics'], timeout
                )
                progress.remove_task(task)
            
            # Test services
            if 'services' in expectations:
                task = progress.add_task("Testing services...", total=None)
                results['services'] = self.topic_checker.check_services(
                    expectations['services'], timeout
                )
                progress.remove_task(task)
            
            # Test parameters
            if 'parameters' in expectations:
                task = progress.add_task("Testing parameters...", total=None)
                results['parameters'] = self.node_checker.check_parameters(
                    expectations['parameters'], timeout
                )
                progress.remove_task(task)
        
        # Determine overall success
        for category in ['nodes', 'topics', 'services', 'parameters']:
            if category in results:
                for item, status in results[category].items():
                    if not status.get('success', False):
                        results['overall_success'] = False
                        break
        
        return results
    
    def print_results(self, results: Dict[str, Any]):
        """Print test results in a formatted table"""
        self.console.print("\n[bold]Test Results Summary[/bold]")
        
        # Create results table
        table = Table(show_header=True, header_style="bold magenta")
        table.add_column("Category", style="cyan")
        table.add_column("Item", style="white")
        table.add_column("Status", justify="center")
        table.add_column("Details", style="dim")
        
        for category in ['nodes', 'topics', 'services', 'parameters']:
            if category in results and results[category]:
                for item, status in results[category].items():
                    success = status.get('success', False)
                    status_text = "[green]✓ PASS[/green]" if success else "[red]✗ FAIL[/red]"
                    details = status.get('details', '')
                    table.add_row(category.title(), item, status_text, details)
        
        self.console.print(table)
        
        # Overall result
        if results['overall_success']:
            self.console.print("\n[bold green]🎉 All tests passed![/bold green]")
        else:
            self.console.print("\n[bold red]❌ Some tests failed![/bold red]")
    
    def test_launch_file(self, launch_file_path: str, config_path: Optional[str] = None, 
                        timeout: float = 30.0, startup_time: float = 3.0) -> bool:
        """Main method to test a launch file"""
        try:
            # Load expectations
            if config_path:
                expectations = self.load_expectations(config_path)
            else:
                # Default minimal expectations
                expectations = {
                    'nodes': [],
                    'topics': [],
                    'services': []
                }
            
            if not expectations:
                self.console.print("[yellow]No expectations loaded, running basic checks[/yellow]")
            
            # Start launch file
            if not self.start_launch_file(launch_file_path):
                return False
            
            # Wait for startup
            self.wait_for_startup(startup_time)
            
            # Run tests
            results = self.run_tests(expectations, timeout)
            
            # Print results
            self.print_results(results)
            
            return results['overall_success']
            
        except KeyboardInterrupt:
            self.console.print("\n[yellow]Test interrupted by user[/yellow]")
            return False
        except Exception as e:
            self.console.print(f"[red]Error during testing: {e}[/red]")
            return False
        finally:
            # Always stop the launch file
            self.stop_launch_file()
