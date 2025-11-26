"""Driver Node - ROS2 node that connects to OPC UA server and exposes services."""
import asyncio
from threading import Thread

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger

from asyncua import Client


class DriverNode(Node):
    """ROS2 node that bridges OPC UA server to ROS2 topics and services."""

    def __init__(self):
        """Initialize the driver node."""
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
        
        # OPC UA client
        self.client = None
        self.data_node = None
        self.counter_node = None
        self.increment_method = None
        self.reset_method = None
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
        self.loop.run_until_complete(self._connect_opcua())
        try:
            self.loop.run_forever()
        finally:
            self.loop.close()

    async def _connect_opcua(self):
        """Connect to the OPC UA server."""
        try:
            self.client = Client(url=self.opcua_endpoint)
            await self.client.connect()
            self.get_logger().info('Connected to OPC UA server')
            
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
            
            self.get_logger().info('OPC UA nodes and methods acquired')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to OPC UA server: {e}')

    def timer_callback(self):
        """Timer callback to publish OPC UA data."""
        if self.client and self.data_node and self.counter_node:
            # Schedule both reads concurrently in the OPC UA event loop
            future_data = asyncio.run_coroutine_threadsafe(
                self.data_node.read_value(), self.loop
            )
            future_counter = asyncio.run_coroutine_threadsafe(
                self.counter_node.read_value(), self.loop
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
        if self.client and self.increment_method:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.client.call_method(self.increment_method, []), self.loop
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
        if self.client and self.reset_method:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.client.call_method(self.reset_method, []), self.loop
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
        if self.client and self.loop:
            asyncio.run_coroutine_threadsafe(self.client.disconnect(), self.loop)
            self.loop.call_soon_threadsafe(self.loop.stop)
        super().destroy_node()


def main(args=None):
    """Main entry point for the driver node."""
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
