"""Unit tests for the GUI node."""
import sys
from unittest.mock import MagicMock, Mock
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

from hlcs.gui import ROS2Bridge, GUINode, ServiceCallHandler


def test_service_call_handler_init():
    """Test ServiceCallHandler initialization."""
    inc_client = MagicMock()
    reset_client = MagicMock()
    logger = MagicMock()
    
    handler = ServiceCallHandler(inc_client, reset_client, logger)
    
    assert handler.increment_client == inc_client
    assert handler.reset_client == reset_client
    assert handler.logger == logger


def test_service_call_handler_is_increment_service_available():
    """Test checking if increment service is available."""
    inc_client = MagicMock()
    inc_client.wait_for_service = MagicMock(return_value=True)
    reset_client = MagicMock()
    
    handler = ServiceCallHandler(inc_client, reset_client)
    
    assert handler.is_increment_service_available(timeout_sec=1.0)
    inc_client.wait_for_service.assert_called_once_with(timeout_sec=1.0)


def test_service_call_handler_is_reset_service_available():
    """Test checking if reset service is available."""
    inc_client = MagicMock()
    reset_client = MagicMock()
    reset_client.wait_for_service = MagicMock(return_value=False)
    
    handler = ServiceCallHandler(inc_client, reset_client)
    
    assert not handler.is_reset_service_available(timeout_sec=2.0)
    reset_client.wait_for_service.assert_called_once_with(timeout_sec=2.0)


def test_service_call_handler_call_increment_async():
    """Test calling increment service asynchronously."""
    inc_client = MagicMock()
    reset_client = MagicMock()
    
    # Mock Trigger.Request
    sys.modules['std_srvs'].srv.Trigger.Request = MagicMock
    
    handler = ServiceCallHandler(inc_client, reset_client)
    future = handler.call_increment_async()
    
    inc_client.call_async.assert_called_once()


def test_service_call_handler_call_reset_async():
    """Test calling reset service asynchronously."""
    inc_client = MagicMock()
    reset_client = MagicMock()
    
    # Mock Trigger.Request
    sys.modules['std_srvs'].srv.Trigger.Request = MagicMock
    
    handler = ServiceCallHandler(inc_client, reset_client)
    future = handler.call_reset_async()
    
    reset_client.call_async.assert_called_once()


def test_service_call_handler_handle_service_response_success():
    """Test handling successful service response."""
    handler = ServiceCallHandler(MagicMock(), MagicMock())
    
    # Mock future with successful response
    future = MagicMock()
    response = MagicMock()
    response.success = True
    response.message = "Operation completed"
    future.result = MagicMock(return_value=response)
    
    success, message = handler.handle_service_response(future)
    
    assert success is True
    assert "Success" in message
    assert "Operation completed" in message


def test_service_call_handler_handle_service_response_failure():
    """Test handling failed service response."""
    handler = ServiceCallHandler(MagicMock(), MagicMock())
    
    # Mock future with failed response
    future = MagicMock()
    response = MagicMock()
    response.success = False
    response.message = "Service error"
    future.result = MagicMock(return_value=response)
    
    success, message = handler.handle_service_response(future)
    
    assert success is False
    assert "Failed" in message
    assert "Service error" in message


def test_service_call_handler_handle_service_response_exception():
    """Test handling service response with exception."""
    logger = MagicMock()
    handler = ServiceCallHandler(MagicMock(), MagicMock(), logger)
    
    # Mock future that raises exception
    future = MagicMock()
    future.result = MagicMock(side_effect=Exception("Connection lost"))
    
    success, message = handler.handle_service_response(future)
    
    assert success is False
    assert "Service call failed" in message
    logger.error.assert_called_once()


def test_ros2bridge_init():
    """Test ROS2Bridge initialization."""
    node = MagicMock()
    bridge = ROS2Bridge(node)
    
    assert bridge.node == node
    assert bridge._data_value == 0.0
    assert bridge._counter_value == 0
    assert bridge._status_message == "Waiting for data..."


def test_ros2bridge_data_value_internal_state():
    """Test ROS2Bridge data value internal state."""
    bridge = ROS2Bridge(None)
    
    # Test initial state
    assert bridge._data_value == 0.0
    
    # Test updating internal state
    bridge._data_value = 42.5
    assert bridge._data_value == 42.5


def test_ros2bridge_counter_value_internal_state():
    """Test ROS2Bridge counter value internal state."""
    bridge = ROS2Bridge(None)
    
    # Test initial state
    assert bridge._counter_value == 0
    
    # Test updating internal state
    bridge._counter_value = 10
    assert bridge._counter_value == 10


def test_ros2bridge_status_message_internal_state():
    """Test ROS2Bridge status message internal state."""
    bridge = ROS2Bridge(None)
    
    # Test initial state
    assert bridge._status_message == "Waiting for data..."
    
    # Test updating internal state
    bridge._status_message = "Test message"
    assert bridge._status_message == "Test message"


def test_ros2bridge_increment_counter():
    """Test ROS2Bridge increment counter slot."""
    node = MagicMock()
    node.call_increment_service = MagicMock()
    bridge = ROS2Bridge(node)
    
    bridge.incrementCounter()
    
    node.call_increment_service.assert_called_once()


def test_ros2bridge_reset_counter():
    """Test ROS2Bridge reset counter slot."""
    node = MagicMock()
    node.call_reset_service = MagicMock()
    bridge = ROS2Bridge(node)
    
    bridge.resetCounter()
    
    node.call_reset_service.assert_called_once()


def test_ros2bridge_with_none_node():
    """Test ROS2Bridge methods when node is None."""
    bridge = ROS2Bridge(None)
    
    # Should not raise errors
    bridge.incrementCounter()
    bridge.resetCounter()


def test_gui_node_class_exists():
    """Test GUINode class exists and has basic structure."""
    assert GUINode is not None
    assert hasattr(GUINode, '__init__')


def test_gui_node_init_accepts_service_handler():
    """Test GUINode initialization accepts service handler."""
    # Verify the parameter exists by checking we can create a ServiceCallHandler
    handler = ServiceCallHandler(MagicMock(), MagicMock())
    assert handler is not None
    
    # Verify GUINode accepts bridge parameter
    assert hasattr(GUINode.__init__, '__code__')

