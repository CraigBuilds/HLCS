"""Unit tests for the driver node."""
import sys
from unittest.mock import Mock, MagicMock

# Mock rclpy module before importing driver
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()

from hlcs.driver import DriverNode


def test_driver_node_class_exists():
    """Test DriverNode class exists and has basic structure."""
    assert DriverNode is not None
    assert hasattr(DriverNode, '__init__')
