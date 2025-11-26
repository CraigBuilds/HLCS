"""GUI Node - ROS2 node with PySide6/QML interface for HLCS."""
import sys
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger

from PySide6.QtCore import QObject, Property, Signal, Slot, QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine


class ROS2Bridge(QObject):
    """Bridge between ROS2 and QML."""

    dataValueChanged = Signal()
    counterValueChanged = Signal()
    statusMessageChanged = Signal()

    def __init__(self, node):
        """Initialize the bridge."""
        super().__init__()
        self.node = node
        self._data_value = 0.0
        self._counter_value = 0
        self._status_message = "Waiting for data..."

    @Property(float, notify=dataValueChanged)
    def dataValue(self):
        """Get current data value."""
        return self._data_value

    @dataValue.setter
    def dataValue(self, value):
        """Set data value and emit signal."""
        if self._data_value != value:
            self._data_value = value
            self.dataValueChanged.emit()

    @Property(int, notify=counterValueChanged)
    def counterValue(self):
        """Get current counter value."""
        return self._counter_value

    @counterValue.setter
    def counterValue(self, value):
        """Set counter value and emit signal."""
        if self._counter_value != value:
            self._counter_value = value
            self.counterValueChanged.emit()

    @Property(str, notify=statusMessageChanged)
    def statusMessage(self):
        """Get status message."""
        return self._status_message

    @statusMessage.setter
    def statusMessage(self, value):
        """Set status message and emit signal."""
        if self._status_message != value:
            self._status_message = value
            self.statusMessageChanged.emit()

    @Slot()
    def incrementCounter(self):
        """Call increment counter service."""
        self.node.call_increment_service()

    @Slot()
    def resetCounter(self):
        """Call reset counter service."""
        self.node.call_reset_service()


class GUINode(Node):
    """ROS2 node with GUI for displaying data and calling services."""

    def __init__(self, bridge):
        """Initialize the GUI node."""
        super().__init__('gui')
        self.bridge = bridge
        
        # Subscribers
        self.data_sub = self.create_subscription(
            Float64, 'hlcs/data', self.data_callback, 10
        )
        self.counter_sub = self.create_subscription(
            Int32, 'hlcs/counter', self.counter_callback, 10
        )
        
        # Service clients
        self.increment_client = self.create_client(Trigger, 'hlcs/increment_counter')
        self.reset_client = self.create_client(Trigger, 'hlcs/reset_counter')
        
        self.get_logger().info('GUI node initialized')
        self.bridge.statusMessage = 'GUI node initialized. Waiting for data...'

    def data_callback(self, msg):
        """Callback for data topic."""
        self.bridge.dataValue = msg.data

    def counter_callback(self, msg):
        """Callback for counter topic."""
        self.bridge.counterValue = msg.data
        self.bridge.statusMessage = f'Received counter update: {msg.data}'

    def call_increment_service(self):
        """Call the increment counter service."""
        if not self.increment_client.wait_for_service(timeout_sec=1.0):
            self.bridge.statusMessage = 'Increment service not available'
            self.get_logger().warn('Increment service not available')
            return
        
        request = Trigger.Request()
        future = self.increment_client.call_async(request)
        future.add_done_callback(self.increment_response_callback)
        self.bridge.statusMessage = 'Calling increment service...'

    def increment_response_callback(self, future):
        """Handle increment service response."""
        try:
            response = future.result()
            if response.success:
                self.bridge.statusMessage = f'Success: {response.message}'
            else:
                self.bridge.statusMessage = f'Failed: {response.message}'
        except Exception as e:
            self.bridge.statusMessage = f'Service call failed: {e}'
            self.get_logger().error(f'Service call failed: {e}')

    def call_reset_service(self):
        """Call the reset counter service."""
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.bridge.statusMessage = 'Reset service not available'
            self.get_logger().warn('Reset service not available')
            return
        
        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        future.add_done_callback(self.reset_response_callback)
        self.bridge.statusMessage = 'Calling reset service...'

    def reset_response_callback(self, future):
        """Handle reset service response."""
        try:
            response = future.result()
            if response.success:
                self.bridge.statusMessage = f'Success: {response.message}'
            else:
                self.bridge.statusMessage = f'Failed: {response.message}'
        except Exception as e:
            self.bridge.statusMessage = f'Service call failed: {e}'
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    """Main entry point for the GUI node."""
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create Qt application
    app = QGuiApplication(sys.argv)
    
    # Create bridge
    bridge = ROS2Bridge(None)
    
    # Create ROS2 node
    node = GUINode(bridge)
    bridge.node = node
    
    # Set up QML engine
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty("bridge", bridge)
    
    # Load QML file
    qml_file = Path(__file__).parent / "qml" / "main.qml"
    engine.load(QUrl.fromLocalFile(str(qml_file)))
    
    if not engine.rootObjects():
        node.get_logger().error('Failed to load QML file')
        return -1
    
    # Create executor for ROS2
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run ROS2 in a separate thread
    from threading import Thread
    ros_thread = Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # Run Qt event loop
    try:
        exit_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
