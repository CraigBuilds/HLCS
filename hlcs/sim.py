"""OPC UA Simulator - Simple OPC UA server for HLCS."""
import asyncio
from asyncua import Server, ua


class CounterLogic:
    """Business logic for counter operations - separated for testability."""

    def __init__(self):
        """Initialize counter logic."""
        self._counter_value = 0

    def get_counter(self):
        """Get current counter value."""
        return self._counter_value

    def increment_counter(self):
        """Increment the counter and return new value."""
        self._counter_value += 1
        return self._counter_value

    def reset_counter(self):
        """Reset the counter to 0."""
        self._counter_value = 0
        return self._counter_value


class OpcuaSimulator:
    """Simple OPC UA server that hosts data and methods."""

    def __init__(self, endpoint="opc.tcp://0.0.0.0:4840/freeopcua/server/", counter_logic=None):
        """Initialize the OPC UA server.
        
        Args:
            endpoint: OPC UA server endpoint URL
            counter_logic: Optional CounterLogic instance for testing
        """
        self.endpoint = endpoint
        self.server = None
        self.namespace_idx = None
        self.data_node = None
        self.counter_node = None
        self.counter_logic = counter_logic or CounterLogic()

    async def setup(self):
        """Set up the OPC UA server."""
        self.server = Server()
        await self.server.init()
        self.server.set_endpoint(self.endpoint)
        self.server.set_server_name("HLCS Simulator")

        # Register namespace
        uri = "http://hlcs.example.com"
        self.namespace_idx = await self.server.register_namespace(uri)

        # Create objects
        objects = self.server.get_objects_node()
        hlcs_object = await objects.add_object(self.namespace_idx, "HLCSObject")

        # Add variables
        self.data_node = await hlcs_object.add_variable(
            self.namespace_idx, "Data", 0.0
        )
        await self.data_node.set_writable()

        self.counter_node = await hlcs_object.add_variable(
            self.namespace_idx, "Counter", 0
        )
        await self.counter_node.set_writable()

        # Add methods
        await hlcs_object.add_method(
            self.namespace_idx,
            "IncrementCounter",
            self.increment_counter,
            [],
            [ua.VariantType.Int32]
        )

        await hlcs_object.add_method(
            self.namespace_idx,
            "ResetCounter",
            self.reset_counter,
            [],
            []
        )

        print(f"OPC UA Server started at {self.endpoint}")

    async def increment_counter(self, parent):
        """Method to increment the counter."""
        new_value = self.counter_logic.increment_counter()
        await self.counter_node.write_value(new_value)
        print(f"Counter incremented to {new_value}")
        return new_value

    async def reset_counter(self, parent):
        """Method to reset the counter."""
        self.counter_logic.reset_counter()
        await self.counter_node.write_value(0)
        print("Counter reset to 0")

    async def run(self):
        """Run the OPC UA server."""
        await self.setup()
        async with self.server:
            print("Server is running. Press Ctrl+C to stop.")
            # Update data periodically
            counter = 0
            while True:
                await asyncio.sleep(1)
                counter += 1
                data_value = counter * 0.1
                await self.data_node.write_value(data_value)


def main():
    """Main entry point for the simulator."""
    simulator = OpcuaSimulator()
    try:
        asyncio.run(simulator.run())
    except KeyboardInterrupt:
        print("\nSimulator stopped.")


if __name__ == "__main__":
    main()
