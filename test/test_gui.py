"""Unit tests for the GUI node.

These tests focus on testing the GUI node's business logic with mocked dependencies.
"""
import sys
from unittest.mock import MagicMock
import pytest

# Mock ROS2 and PySide6 modules before importing gui
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.executors'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['std_srvs'] = MagicMock()
sys.modules['std_srvs.srv'] = MagicMock()

# Mock PySide6 with proper QObject base class behavior
mock_qobject = type('QObject', (), {
    '__init__': lambda self: None,
})

# Create a proper Property mock that supports setter
class PropertyMock:
    def __init__(self, *args, **kwargs):
        self._getter = None
        self._setter = None
    
    def __call__(self, func):
        self._getter = func
        return self
    
    def setter(self, func):
        self._setter = func
        return self

mock_qtcore = MagicMock()
mock_qtcore.QObject = mock_qobject
mock_qtcore.Property = lambda *args, **kwargs: PropertyMock()
mock_qtcore.Signal = lambda *args, **kwargs: MagicMock()
mock_qtcore.Slot = lambda *args, **kwargs: lambda func: func

sys.modules['PySide6'] = MagicMock()
sys.modules['PySide6.QtCore'] = mock_qtcore
sys.modules['PySide6.QtGui'] = MagicMock()
sys.modules['PySide6.QtQml'] = MagicMock()

from hlcs.gui import ROS2Bridge, GUINode


def test_gui_module_imports():
    """Test that gui module can be imported."""
    from hlcs import gui
    assert gui is not None
    assert hasattr(gui, 'main')


def test_ros2bridge_class_exists():
    """Test ROS2Bridge class exists in the module."""
    assert ROS2Bridge is not None


def test_gui_node_class_exists():
    """Test GUINode class exists in the module."""
    assert GUINode is not None


def test_ros2bridge_initialization():
    """Test ROS2Bridge initializes with a node."""
    mock_node = MagicMock()
    bridge = ROS2Bridge(mock_node)
    
    assert bridge.node == mock_node
    assert bridge._data_value == 0.0
    assert bridge._counter_value == 0
    assert bridge._status_message == "Waiting for data..."


def test_ros2bridge_increment_counter_calls_node():
    """Test ROS2Bridge incrementCounter slot calls node method."""
    mock_node = MagicMock()
    mock_node.call_increment_service = MagicMock()
    
    bridge = ROS2Bridge(mock_node)
    bridge.incrementCounter()
    
    # Verify the node method was called
    mock_node.call_increment_service.assert_called_once()


def test_ros2bridge_reset_counter_calls_node():
    """Test ROS2Bridge resetCounter slot calls node method."""
    mock_node = MagicMock()
    mock_node.call_reset_service = MagicMock()
    
    bridge = ROS2Bridge(mock_node)
    bridge.resetCounter()
    
    # Verify the node method was called
    mock_node.call_reset_service.assert_called_once()
