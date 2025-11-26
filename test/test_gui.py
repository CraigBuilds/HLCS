"""Unit tests for the GUI node."""
import sys
from unittest.mock import Mock, MagicMock

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


def test_ros2bridge_class_exists():
    """Test ROS2Bridge class exists."""
    assert ROS2Bridge is not None


def test_gui_node_class_exists():
    """Test GUINode class exists and has basic structure."""
    assert GUINode is not None
    assert hasattr(GUINode, '__init__')
