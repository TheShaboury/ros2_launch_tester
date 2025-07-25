"""
Topic and service checker for ROS2
"""

import time
from typing import Dict, List, Any, Set
import rclpy
from rclpy.node import Node


class TopicChecker:
    """Check if specified topics and services exist"""
    
    def __init__(self, parent_node: Node):
        self.node = parent_node
    
    def get_topic_list(self) -> List[str]:
        """Get list of currently available topics"""
        try:
            topic_names_and_types = self.node.get_topic_names_and_types()
            return [name for name, _ in topic_names_and_types]
        except Exception as e:
            self.node.get_logger().error(f"Failed to get topic list: {e}")
            return []
    
    def get_service_list(self) -> List[str]:
        """Get list of currently available services"""
        try:
            service_names_and_types = self.node.get_service_names_and_types()
            return [name for name, _ in service_names_and_types]
        except Exception as e:
            self.node.get_logger().error(f"Failed to get service list: {e}")
            return []
    
    def check_topic_exists(self, topic_name: str, timeout: float = 10.0) -> Dict[str, Any]:
        """Check if a specific topic exists"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            topics = self.get_topic_list()
            if topic_name in topics:
                return {
                    'success': True,
                    'details': f'Topic found in {time.time() - start_time:.2f}s'
                }
            time.sleep(0.5)
        
        return {
            'success': False,
            'details': f'Topic not found after {timeout}s timeout'
        }
    
    def check_service_exists(self, service_name: str, timeout: float = 10.0) -> Dict[str, Any]:
        """Check if a specific service exists"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            services = self.get_service_list()
            if service_name in services:
                return {
                    'success': True,
                    'details': f'Service found in {time.time() - start_time:.2f}s'
                }
            time.sleep(0.5)
        
        return {
            'success': False,
            'details': f'Service not found after {timeout}s timeout'
        }
    
    def check_topics(self, expected_topics: List[str], timeout: float = 30.0) -> Dict[str, Any]:
        """Check multiple topics"""
        results = {}
        
        if not expected_topics:
            # If no specific topics expected, just list what's available
            topics = self.get_topic_list()
            return {
                'discovered_topics': {
                    'success': True,
                    'details': f'Found {len(topics)} topics: {", ".join(topics[:10])}{"..." if len(topics) > 10 else ""}'
                }
            }
        
        for topic_name in expected_topics:
            results[topic_name] = self.check_topic_exists(topic_name, timeout)
        
        return results
    
    def check_services(self, expected_services: List[str], timeout: float = 30.0) -> Dict[str, Any]:
        """Check multiple services"""
        results = {}
        
        if not expected_services:
            # If no specific services expected, just list what's available
            services = self.get_service_list()
            return {
                'discovered_services': {
                    'success': True,
                    'details': f'Found {len(services)} services: {", ".join(services[:10])}{"..." if len(services) > 10 else ""}'
                }
            }
        
        for service_name in expected_services:
            results[service_name] = self.check_service_exists(service_name, timeout)
        
        return results
    
    def check_topic_publishing(self, topic_name: str, timeout: float = 5.0) -> Dict[str, Any]:
        """Check if a topic is actively publishing (advanced feature)"""
        # This is a placeholder for more advanced topic monitoring
        # In a full implementation, you'd subscribe to the topic and check for messages
        return {
            'success': True,
            'details': 'Topic publishing check not fully implemented'
        }

