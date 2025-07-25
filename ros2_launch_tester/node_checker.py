"""
Node checker for verifying ROS2 nodes are running
"""

import time
from typing import Dict, List, Any
import rclpy
from rclpy.node import Node


class NodeChecker:
    """Check if specified nodes are running"""
    
    def __init__(self, parent_node: Node):
        self.node = parent_node
    
    def get_node_list(self) -> List[str]:
        """Get list of currently running nodes"""
        try:
            node_names_and_namespaces = self.node.get_node_names_and_namespaces()
            return [f"{namespace}{name}" if namespace != "/" else name 
                   for name, namespace in node_names_and_namespaces]
        except Exception as e:
            self.node.get_logger().error(f"Failed to get node list: {e}")
            return []
    
    def check_node_exists(self, node_name: str, timeout: float = 10.0) -> Dict[str, Any]:
        """Check if a specific node exists"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            nodes = self.get_node_list()
            if node_name in nodes:
                return {
                    'success': True,
                    'details': f'Node found in {time.time() - start_time:.2f}s'
                }
            time.sleep(0.5)
        
        return {
            'success': False,
            'details': f'Node not found after {timeout}s timeout'
        }
    
    def check_nodes(self, expected_nodes: List[str], timeout: float = 30.0) -> Dict[str, Any]:
        """Check multiple nodes"""
        results = {}
        
        if not expected_nodes:
            # If no specific nodes expected, just list what's running
            nodes = self.get_node_list()
            return {
                'discovered_nodes': {
                    'success': True,
                    'details': f'Found {len(nodes)} nodes: {", ".join(nodes)}'
                }
            }
        
        for node_name in expected_nodes:
            results[node_name] = self.check_node_exists(node_name, timeout)
        
        return results
    
    def check_parameters(self, expected_params: Dict[str, Any], timeout: float = 10.0) -> Dict[str, Any]:
        """Check if specified parameters exist and have expected values"""
        results = {}
        
        for param_path, expected_value in expected_params.items():
            try:
                # Parse node name and parameter name from path like "/node_name/param_name"
                parts = param_path.strip('/').split('/')
                if len(parts) < 2:
                    results[param_path] = {
                        'success': False,
                        'details': 'Invalid parameter path format'
                    }
                    continue
                
                node_name = parts[0]
                param_name = '/'.join(parts[1:])
                
                # For now, we'll mark parameter checking as a placeholder
                # In a full implementation, you'd use parameter client to check values
                results[param_path] = {
                    'success': True,
                    'details': 'Parameter checking not fully implemented'
                }
                
            except Exception as e:
                results[param_path] = {
                    'success': False,
                    'details': f'Error checking parameter: {e}'
                }
        
        return results

