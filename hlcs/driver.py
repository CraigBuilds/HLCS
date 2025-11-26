"""Driver Node - ROS2 node that connects to OPC UA server and exposes services."""
import asyncio
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger

from asyncua import Client


class OpcuaClientWrapper:
    """Wrapper for OPC UA client operations - separated for testability."""

    def __init__(self, endpoint):
        """Initialize the wrapper.
        
        Args:
            endpoint: OPC UA server endpoint URL
        """
        self.endpoint = endpoint
        self.client = None
        self.data_node = None
        self.counter_node = None
        self.increment_method = None
        self.reset_method = None
        self._connected = False

    def is_connected(self):
        """Check if client is connected and ready."""
        return self._connected and self.client is not None

    async def connect(self):
        """Connect to the OPC UA server and get nodes/methods."""
        self.client = Client(url=self.endpoint)
        await self.client.connect()
        
        # Get namespace index
        nsidx = await self.client.get_namespace_index("http://hlcs.example.com")
        
        # Get nodes
        root = self.client.get_root_node()
        objects = await root.get_child(["0:Objects"])
        hlcs_object = await objects.get_child([f"{nsidx}:HLCSObject"])
        
        self.data_node = await hlcs_object.get_child([f"{nsidx}:Data"])
        self.counter_node = await hlcs_object.get_child([f"{nsidx}:Counter"])
        
        # Get methods
        children = await hlcs_object.get_children()
        for child in children:
            browse_name = await child.read_browse_name()
            if browse_name.Name == "IncrementCounter":
                self.increment_method = child
            elif browse_name.Name == "ResetCounter":
                self.reset_method = child
        
        self._connected = True

    async def read_data_value(self):
        """Read data value from OPC UA server."""
        if not self.is_connected() or self.data_node is None:
            raise RuntimeError("Not connected to OPC UA server")
        return await self.data_node.read_value()

    async def read_counter_value(self):
        """Read counter value from OPC UA server."""
        if not self.is_connected() or self.counter_node is None:
            raise RuntimeError("Not connected to OPC UA server")
        return await self.counter_node.read_value()

    async def call_increment_counter(self):
        """Call increment counter method."""
        if not self.is_connected() or self.increment_method is None:
            raise RuntimeError("Not connected to OPC UA server")
        return await self.client.call_method(self.increment_method, [])

    async def call_reset_counter(self):
        """Call reset counter method."""
        if not self.is_connected() or self.reset_method is None:
            raise RuntimeError("Not connected to OPC UA server")
        await self.client.call_method(self.reset_method, [])

    async def disconnect(self):
        """Disconnect from OPC UA server."""
        if self.client:
            await self.client.disconnect()
            self._connected = False


class DriverNode(Node):
    """ROS2 node that bridges OPC UA server to ROS2 topics and services."""

    def __init__(self, opcua_wrapper=None):
        """Initialize the driver node.
        
        Args:
            opcua_wrapper: Optional OpcuaClientWrapper for testing
        """
        super().__init__('driver')
        
        # Parameters
        self.declare_parameter('opcua_endpoint', 'opc.tcp://localhost:4840/freeopcua/server/')
        self.opcua_endpoint = self.get_parameter('opcua_endpoint').value
        
        # Publishers
        self.data_pub = self.create_publisher(Float64, 'hlcs/data', 10)
        self.counter_pub = self.create_publisher(Int32, 'hlcs/counter', 10)
        
        # Services
        self.increment_srv = self.create_service(
            Trigger, 'hlcs/increment_counter', self.increment_counter_callback
        )
        self.reset_srv = self.create_service(
            Trigger, 'hlcs/reset_counter', self.reset_counter_callback
        )
        
        # OPC UA client wrapper
        self.opcua_wrapper = opcua_wrapper or OpcuaClientWrapper(self.opcua_endpoint)
        self.running = True
        
        # Start OPC UA client in separate thread
        self.loop = None
        self.opcua_thread = Thread(target=self._run_opcua_client, daemon=True)
        self.opcua_thread.start()
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Driver node initialized, connecting to {self.opcua_endpoint}')

    def _run_opcua_client(self):
        """Run the OPC UA client in a separate thread with its own event loop."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self._connect_opcua())
        except Exception as e:
            self.get_logger().error(f'Failed to start OPC UA client: {e}')
            return
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    async def _connect_opcua(self):
        """Connect to the OPC UA server."""
        try:
            await self.opcua_wrapper.connect()
            self.get_logger().info('Connected to OPC UA server')
            self.get_logger().info('OPC UA nodes and methods acquired')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to OPC UA server: {e}')

    def timer_callback(self):
        """Timer callback to publish OPC UA data."""
        if self.opcua_wrapper.is_connected():
            # Schedule both reads concurrently in the OPC UA event loop
            future_data = asyncio.run_coroutine_threadsafe(
                self.opcua_wrapper.read_data_value(), self.loop
            )
            future_counter = asyncio.run_coroutine_threadsafe(
                self.opcua_wrapper.read_counter_value(), self.loop
            )
            
            try:
                data_value = future_data.result(timeout=0.5)
                counter_value = future_counter.result(timeout=0.5)
                
                # Publish to ROS2 topics
                data_msg = Float64()
                data_msg.data = float(data_value)
                self.data_pub.publish(data_msg)
                
                counter_msg = Int32()
                counter_msg.data = int(counter_value)
                self.counter_pub.publish(counter_msg)
            except Exception as e:
                self.get_logger().warn(f'Failed to read OPC UA values: {e}')

    def increment_counter_callback(self, request, response):
        """Service callback to increment counter."""
        if self.opcua_wrapper.is_connected():
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.opcua_wrapper.call_increment_counter(), self.loop
                )
                result = future.result(timeout=2.0)
                response.success = True
                response.message = f'Counter incremented to {result}'
                self.get_logger().info(response.message)
            except Exception as e:
                response.success = False
                response.message = f'Failed to increment counter: {e}'
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = 'OPC UA client not connected'
            self.get_logger().error(response.message)
        return response

    def reset_counter_callback(self, request, response):
        """Service callback to reset counter."""
        if self.opcua_wrapper.is_connected():
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.opcua_wrapper.call_reset_counter(), self.loop
                )
                future.result(timeout=2.0)
                response.success = True
                response.message = 'Counter reset to 0'
                self.get_logger().info(response.message)
            except Exception as e:
                response.success = False
                response.message = f'Failed to reset counter: {e}'
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = 'OPC UA client not connected'
            self.get_logger().error(response.message)
        return response

    def destroy_node(self):
        """Clean up the node."""
        self.running = False
        if self.opcua_wrapper.is_connected() and self.loop:
            try:
                future = asyncio.run_coroutine_threadsafe(self.opcua_wrapper.disconnect(), self.loop)
                future.result(timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f'Error during disconnect: {e}')
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    """Main entry point for the driver node."""
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow graceful shutdown on Ctrl+C without traceback
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
