"""Unit tests for the OPC UA simulator."""
import pytest
import asyncio
from hlcs.sim import OpcuaSimulator


def test_opcua_simulator_init():
    """Test OpcuaSimulator initialization with default endpoint."""
    simulator = OpcuaSimulator()
    assert simulator.endpoint == "opc.tcp://0.0.0.0:4840/freeopcua/server/"
    assert simulator.server is None
    assert simulator.namespace_idx is None
    assert simulator.data_node is None
    assert simulator.counter_node is None


def test_opcua_simulator_init_custom_endpoint():
    """Test OpcuaSimulator initialization with custom endpoint."""
    custom_endpoint = "opc.tcp://localhost:4850/custom/server/"
    simulator = OpcuaSimulator(endpoint=custom_endpoint)
    assert simulator.endpoint == custom_endpoint


@pytest.mark.asyncio
async def test_opcua_simulator_setup():
    """Test OPC UA simulator setup creates all required nodes."""
    simulator = OpcuaSimulator()
    
    # Use server context manager for proper cleanup
    async with simulator.server or await _setup_simulator(simulator):
        # Verify server was created
        assert simulator.server is not None
        assert simulator.namespace_idx is not None
        
        # Verify nodes were created
        assert simulator.data_node is not None
        assert simulator.counter_node is not None
        
        # Verify initial values
        data_value = await simulator.data_node.read_value()
        assert data_value == 0.0
        
        counter_value = await simulator.counter_node.read_value()
        assert counter_value == 0


@pytest.mark.asyncio
async def test_opcua_simulator_increment_counter():
    """Test increment counter method increases counter value."""
    simulator = OpcuaSimulator()
    await simulator.setup()
    
    async with simulator.server:
        # Get initial counter value
        initial_value = await simulator.counter_node.read_value()
        
        # Call increment method
        result = await simulator.increment_counter(None)
        
        # Verify counter was incremented
        new_value = await simulator.counter_node.read_value()
        assert new_value == initial_value + 1
        assert result == new_value
        
        # Increment again to verify it continues working
        result2 = await simulator.increment_counter(None)
        final_value = await simulator.counter_node.read_value()
        assert final_value == initial_value + 2
        assert result2 == final_value


@pytest.mark.asyncio
async def test_opcua_simulator_reset_counter():
    """Test reset counter method resets counter to zero."""
    simulator = OpcuaSimulator()
    await simulator.setup()
    
    async with simulator.server:
        # Increment counter a few times
        await simulator.increment_counter(None)
        await simulator.increment_counter(None)
        await simulator.increment_counter(None)
        
        # Verify counter is not zero
        value_before_reset = await simulator.counter_node.read_value()
        assert value_before_reset == 3
        
        # Reset counter
        await simulator.reset_counter(None)
        
        # Verify counter is zero
        value_after_reset = await simulator.counter_node.read_value()
        assert value_after_reset == 0


@pytest.mark.asyncio
async def test_opcua_simulator_data_node_writable():
    """Test that data node can be written to."""
    simulator = OpcuaSimulator()
    await simulator.setup()
    
    async with simulator.server:
        # Write a value to the data node
        test_value = 42.5
        await simulator.data_node.write_value(test_value)
        
        # Read back and verify
        read_value = await simulator.data_node.read_value()
        assert read_value == test_value
        
        # Write another value
        test_value2 = -10.3
        await simulator.data_node.write_value(test_value2)
        
        # Read back and verify
        read_value2 = await simulator.data_node.read_value()
        assert read_value2 == test_value2


@pytest.mark.asyncio
async def test_opcua_simulator_counter_node_writable():
    """Test that counter node can be written to."""
    simulator = OpcuaSimulator()
    await simulator.setup()
    
    async with simulator.server:
        # Write a value to the counter node
        test_value = 100
        await simulator.counter_node.write_value(test_value)
        
        # Read back and verify
        read_value = await simulator.counter_node.read_value()
        assert read_value == test_value


async def _setup_simulator(simulator):
    """Helper to setup simulator and return server for context management."""
    await simulator.setup()
    return simulator.server

