"""
Unit tests for ROS2 Launch Tester
"""

import unittest
import tempfile
import os
import yaml
from unittest.mock import Mock, patch

import rclpy
from ros2_launch_tester.tester import LaunchTester
from ros2_launch_tester.node_checker import NodeChecker
from ros2_launch_tester.topic_checker import TopicChecker


class TestLaunchTester(unittest.TestCase):
    """Test cases for LaunchTester class"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test class"""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up each test"""
        self.tester = LaunchTester('test_launch_tester')
    
    def tearDown(self):
        """Tear down each test"""
        self.tester.destroy_node()
    
    def test_load_expectations_valid_yaml(self):
        """Test loading valid YAML expectations"""
        # Create temporary YAML file
        test_config = {
            'nodes': ['/test_node'],
            'topics': ['/test_topic'],
            'services': ['/test_service']
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(test_config, f)
            temp_path = f.name
        
        try:
            expectations = self.tester.load_expectations(temp_path)
            self.assertEqual(expectations['nodes'], ['/test_node'])
            self.assertEqual(expectations['topics'], ['/test_topic'])
            self.assertEqual(expectations['services'], ['/test_service'])
        finally:
            os.unlink(temp_path)
    
    def test_load_expectations_missing_file(self):
        """Test loading expectations from non-existent file"""
        expectations = self.tester.load_expectations('/nonexistent/file.yaml')
        self.assertEqual(expectations, {})
    
    def test_node_checker_initialization(self):
        """Test NodeChecker initialization"""
        self.assertIsInstance(self.tester.node_checker, NodeChecker)
        self.assertEqual(self.tester.node_checker.node, self.tester)
    
    def test_topic_checker_initialization(self):
        """Test TopicChecker initialization"""
        self.assertIsInstance(self.tester.topic_checker, TopicChecker)
        self.assertEqual(self.tester.topic_checker.node, self.tester)


class TestNodeChecker(unittest.TestCase):
    """Test cases for NodeChecker class"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test class"""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up each test"""
        self.node = LaunchTester('test_node_checker')
        self.checker = NodeChecker(self.node)
    
    def tearDown(self):
        """Tear down each test"""
        self.node.destroy_node()
    
    def test_get_node_list(self):
        """Test getting node list"""
        nodes = self.checker.get_node_list()
        self.assertIsInstance(nodes, list)
        # Should at least contain our test node
        self.assertIn('test_node_checker', nodes)


class TestTopicChecker(unittest.TestCase):
    """Test cases for TopicChecker class"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test class"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Tear down test class"""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up each test"""
        self.node = LaunchTester('test_topic_checker')
        self.checker = TopicChecker(self.node)
    
    def tearDown(self):
        """Tear down each test"""
        self.node.destroy_node()
    
    def test_get_topic_list(self):
        """Test getting topic list"""
        topics = self.checker.get_topic_list()
        self.assertIsInstance(topics, list)
    
    def test_get_service_list(self):
        """Test getting service list"""
        services = self.checker.get_service_list()
        self.assertIsInstance(services, list)


if __name__ == '__main__':
    unittest.main()

