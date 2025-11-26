"""Unit tests for the driver node.

These tests focus on testing the driver node's business logic with mocked dependencies.
"""
import sys
from unittest.mock import MagicMock
import pytest

# Mock rclpy module before importing driver
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()


def test_driver_module_imports():
    """Test that driver module can be imported."""
    from hlcs import driver
    assert driver is not None
    assert hasattr(driver, 'main')


def test_driver_node_class_exists():
    """Test DriverNode class exists in the module."""
    from hlcs.driver import DriverNode
    assert DriverNode is not None
